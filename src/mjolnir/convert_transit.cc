#include "mjolnir/convert_transit.h"
#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/vector2.h"
#include "mjolnir/admin.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/ingest_transit.h"
#include "mjolnir/servicedays.h"
#include "proto/transit.pb.h"

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <future>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Struct to hold stats information during each threads work
struct builder_stats {
  uint32_t no_dir_edge_count = 0;
  uint32_t dep_count = 0;
  uint32_t midnight_dep_count = 0;
  uint32_t invalid_service_dates = 0;
  // Accumulate stats from all threads
  void operator()(const builder_stats& other) {
    no_dir_edge_count += other.no_dir_edge_count;
    dep_count += other.dep_count;
    midnight_dep_count += other.midnight_dep_count;
    invalid_service_dates += other.invalid_service_dates;
  }
};

// Shape
struct Shape {
  uint32_t begins;
  uint32_t ends;
  std::vector<valhalla::midgard::PointLL> shape;
};

struct Departure {
  valhalla::baldr::GraphId orig_pbf_graphid; // GraphId in pbf tiles
  valhalla::baldr::GraphId dest_pbf_graphid; // GraphId in pbf tiles
  uint32_t trip;
  uint32_t route;
  uint32_t blockid;
  uint32_t shapeid;
  uint32_t headsign_offset;
  uint32_t dep_time;
  uint32_t schedule_index;
  uint32_t frequency_end_time;
  uint16_t elapsed_time;
  uint16_t frequency;
  float orig_dist_traveled;
  float dest_dist_traveled;
  bool wheelchair_accessible;
  bool bicycle_accessible;
};

// Unique route and stop
struct TransitLine {
  uint32_t lineid;
  uint32_t routeid;
  valhalla::baldr::GraphId dest_pbf_graphid; // GraphId (from pbf) of the destination stop
  uint32_t shapeid;
  float orig_dist_traveled;
  float dest_dist_traveled;
};

struct StopEdges {
  valhalla::baldr::GraphId origin_pbf_graphid;        // GraphId (from pbf) of the origin stop
  std::vector<valhalla::baldr::GraphId> intrastation; // List of intra-station connections
  std::vector<TransitLine> lines;                     // Set of unique route/stop pairs
};

// A platform gets one outbound directed edge per unique (route, next stop) line plus one
// platform connection to its station. NodeInfo can only hold kMaxEdgesPerNode outbound edges.
// A platform that needs more is served by the platform node itself (slot 0) plus extra
// "overflow" platform nodes (slot 1..n). The overflow nodes sit at the platform's location, share
// its stop_index (so stop lookups, names and stop filters see the same stop) and are connected to
// the parent station by platform connections, exactly like a sibling GTFS platform. They are
// appended after the pbf nodes of the tile, so every existing node id (= pbf node id) is stable.
//
// All lines of one route are kept on one slot, and transit lines of that route arriving at the
// platform end at that slot. A vehicle that continues through the stop therefore stays on one
// node: remaining on the same trip costs nothing, as before. Changing to a route on another slot
// is a platform-to-platform transfer through the station.
struct PlatformSplit {
  GraphId station;                                      // graph id of the parent station
  std::vector<GraphId> overflow;                        // graph ids of slots 1..n
  std::unordered_map<std::string, uint32_t> route_slot; // route onestop id -> slot
};

struct PlatformSplitPlan {
  // keyed by the graph id (= pbf graph id) of the split platform
  std::unordered_map<GraphId, PlatformSplit> splits;
  // per transit tile: (platform, slot) of each overflow node, in node id order
  std::unordered_map<GraphId, std::vector<std::pair<GraphId, uint32_t>>> tile_overflow;
};

std::filesystem::path TransitPbfPath(const std::string& transit_dir, const GraphId& tile_id) {
  std::string file_name = GraphTile::FileSuffix(GraphId(tile_id.tileid(), tile_id.level(), 0));
  boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
  file_name += ".pbf";
  std::filesystem::path pbf_fp{transit_dir};
  pbf_fp.append(file_name);
  return pbf_fp;
}

// Count the lines each platform of one transit tile will get and plan the overflow nodes of the
// platforms that need more than one node. This counts every stop pair, including ones that
// ProcessStopPairs later drops for having no valid service day, so it never undercounts; AddToGraph
// checks the real counts again and throws if a node would still be over the limit.
void PlanTileSplits(const std::string& transit_dir,
                    const GraphId& tile_id,
                    std::mutex& lock,
                    PlatformSplitPlan& plan) {
  const auto pbf_fp = TransitPbfPath(transit_dir, tile_id);
  if (!std::filesystem::exists(pbf_fp)) {
    return;
  }
  const Transit tile_pbf = read_pbf(pbf_fp.string(), lock);

  // platform node index -> route index -> next stops
  std::unordered_map<uint32_t, std::unordered_map<uint32_t, std::unordered_set<uint64_t>>> lines;
  auto count_lines = [&lines](const Transit& pbf) {
    for (const auto& stop_pair : pbf.stop_pairs()) {
      GraphId dest(stop_pair.destination_graphid());
      if (!dest.is_valid()) {
        continue; // AddToGraph skips these too ("Unstitched stop pair")
      }
      lines[GraphId(stop_pair.origin_graphid()).id()][stop_pair.route_index()].insert(dest.value);
    }
  };
  count_lines(tile_pbf);
  // stop pairs past the ingest trip limit are written to <tile>.pbf.0, .1, ...
  for (uint32_t ext = 0;; ++ext) {
    std::filesystem::path ext_fp = pbf_fp;
    ext_fp += "." + std::to_string(ext);
    if (!std::filesystem::exists(ext_fp)) {
      break;
    }
    count_lines(read_pbf(ext_fp.string(), lock));
  }

  // lines one node can carry next to its platform connection to the station
  constexpr uint32_t kLinesPerNode = kMaxEdgesPerNode - 1;
  std::vector<uint32_t> full_platforms;
  for (const auto& platform : lines) {
    size_t n = 0;
    for (const auto& route : platform.second) {
      n += route.second.size();
    }
    if (n > kLinesPerNode) {
      full_platforms.push_back(platform.first);
    }
  }
  if (full_platforms.empty()) {
    return;
  }
  std::sort(full_platforms.begin(), full_platforms.end());

  PlatformSplitPlan tile_plan;
  auto& tile_overflow = tile_plan.tile_overflow[tile_id.tile_base()];
  uint32_t next_node_id = static_cast<uint32_t>(tile_pbf.nodes_size());
  for (const uint32_t platform_index : full_platforms) {
    if (platform_index >= static_cast<uint32_t>(tile_pbf.nodes_size())) {
      throw std::runtime_error("convert_transit: stop pair origin " + std::to_string(platform_index) +
                               " is not a node of transit tile " + std::to_string(tile_id.tileid()));
    }
    const Transit_Node& platform = tile_pbf.nodes(platform_index);
    const std::string stop_desc = platform.name() + " (" + platform.onestop_id() +
                                  ") in transit tile " + std::to_string(tile_id.tileid());

    // routes ordered by line count (most first), then by id: deterministic first-fit decreasing
    std::vector<std::pair<std::string, uint32_t>> routes;
    size_t total = 0;
    for (const auto& route : lines.at(platform_index)) {
      if (route.first >= static_cast<uint32_t>(tile_pbf.routes_size())) {
        throw std::runtime_error("convert_transit: route index " + std::to_string(route.first) +
                                 " out of range at platform " + stop_desc);
      }
      routes.emplace_back(tile_pbf.routes(route.first).onestop_id(),
                          static_cast<uint32_t>(route.second.size()));
      total += route.second.size();
    }
    std::sort(routes.begin(), routes.end(), [](const auto& a, const auto& b) {
      return a.second != b.second ? a.second > b.second : a.first < b.first;
    });

    PlatformSplit split;
    split.station = GraphId(platform.prev_type_graphid());
    std::vector<uint32_t> slot_load;
    for (const auto& route : routes) {
      if (route.second > kLinesPerNode) {
        throw std::runtime_error("convert_transit: route " + route.first + " has " +
                                 std::to_string(route.second) + " lines at platform " + stop_desc +
                                 ", more than one node can hold (" + std::to_string(kLinesPerNode) +
                                 ")");
      }
      uint32_t slot = 0;
      while (slot < slot_load.size() && slot_load[slot] + route.second > kLinesPerNode) {
        ++slot;
      }
      if (slot == slot_load.size()) {
        slot_load.push_back(0);
      }
      slot_load[slot] += route.second;
      split.route_slot.emplace(route.first, slot);
    }
    for (uint32_t slot = 1; slot < slot_load.size(); ++slot) {
      split.overflow.emplace_back(tile_id.tileid(), tile_id.level(), next_node_id++);
      tile_overflow.emplace_back(GraphId(platform.graphid()), slot);
    }
    LOG_WARN("Transit platform " + stop_desc + " has " + std::to_string(total) + " lines (" +
             std::to_string(routes.size()) + " routes); split over " +
             std::to_string(slot_load.size()) + " platform nodes");
    tile_plan.splits.emplace(GraphId(platform.graphid()), std::move(split));
  }

  std::lock_guard<std::mutex> guard(lock);
  for (auto& split : tile_plan.splits) {
    plan.splits.emplace(split.first, std::move(split.second));
  }
  for (auto& overflow : tile_plan.tile_overflow) {
    plan.tile_overflow.emplace(overflow.first, std::move(overflow.second));
  }
}

PlatformSplitPlan PlanPlatformSplits(const std::string& transit_dir,
                                     const std::unordered_set<GraphId>& all_tiles,
                                     const unsigned int thread_count) {
  PlatformSplitPlan plan;
  std::vector<GraphId> tiles(all_tiles.begin(), all_tiles.end());
  std::atomic<size_t> next{0};
  std::mutex lock;
  std::exception_ptr error;
  std::vector<std::thread> threads;
  for (unsigned int i = 0; i < std::max(1u, thread_count); ++i) {
    threads.emplace_back([&]() {
      try {
        for (size_t t = next++; t < tiles.size(); t = next++) {
          PlanTileSplits(transit_dir, tiles[t].tile_base(), lock, plan);
        }
      } catch (...) {
        std::lock_guard<std::mutex> guard(lock);
        if (!error) {
          error = std::current_exception();
        }
        next = tiles.size();
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  if (error) {
    std::rethrow_exception(error);
  }
  return plan;
}

// Get scheduled departures for a stop; here we also look at the .pbf.x files,
// as there can be only stop pairs in the extended ones
std::unordered_multimap<GraphId, Departure>
ProcessStopPairs(GraphTileBuilder& transit_tilebuilder,
                 const uint32_t tile_date,
                 const Transit& tile_pbf,
                 std::unordered_map<GraphId, uint16_t>& stop_no_access,
                 const std::filesystem::path& pbf_fp,
                 std::mutex& lock,
                 builder_stats& stats) {
  // Check if there are no schedule stop pairs in this tile
  std::unordered_multimap<GraphId, Departure> departures;

  // Map of unique schedules (validity) in this tile
  uint32_t schedule_index = 0;
  std::map<TransitSchedule, uint32_t> schedules;

  std::filesystem::recursive_directory_iterator transit_file_itr(pbf_fp.parent_path());
  std::filesystem::recursive_directory_iterator end_file_itr;

  // lambda to add a schedule
  auto add_schedule = [&schedule_index, &schedules,
                       &transit_tilebuilder](Departure& dep, const uint64_t days, const uint32_t dow,
                                             const uint32_t end_day) {
    TransitSchedule sched(days, dow, end_day);
    auto sched_itr = schedules.find(sched);
    if (sched_itr == schedules.end()) {
      transit_tilebuilder.AddTransitSchedule(sched);
      // Add to the map and increment the index
      schedules[sched] = schedule_index;
      dep.schedule_index = schedule_index;
      schedule_index++;
    } else {
      dep.schedule_index = sched_itr->second;
    }
  };

  // for each tile.
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (std::filesystem::is_regular_file(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      std::string ext = transit_file_itr->path().extension().string();
      std::string file_name = fname.substr(0, fname.size() - ext.size());

      // make sure we are looking at a pbf file
      if ((ext == ".pbf" && fname == pbf_fp) ||
          (file_name.substr(file_name.size() - 4) == ".pbf" && file_name == pbf_fp)) {

        Transit curr_tile_pbf;
        {
          // already loaded or it's with a xxx.pbf.y extension
          if (ext == ".pbf") {
            curr_tile_pbf = tile_pbf;
          } else {
            curr_tile_pbf = read_pbf(transit_file_itr->path().string(), lock);
          }
        }

        if (curr_tile_pbf.stop_pairs_size() == 0) {
          if (tile_pbf.nodes_size() > 0) {
            LOG_ERROR("Tile " + fname + " has 0 schedule stop pairs but has " +
                      std::to_string(tile_pbf.nodes_size()) + " stops");
          }
          departures.clear();
          return departures;
        }

        // Iterate through the stop pairs in this tile and form Valhalla departure
        // records
        for (const auto& stop_pair : curr_tile_pbf.stop_pairs()) {
          // We do not know in this step if the end node is in a valid (non-empty)
          // Valhalla tile. So just add the stop pair and we will address this later

          // Use transit PBF graph Ids internally until adding to the graph tiles
          // TODO - wheelchair accessible, shape information
          Departure dep;
          dep.orig_pbf_graphid = GraphId(stop_pair.origin_graphid());
          dep.dest_pbf_graphid = GraphId(stop_pair.destination_graphid());
          dep.route = stop_pair.route_index();
          dep.trip = stop_pair.trip_id();

          // Compute the valid days
          // set the bits based on the dow.// Compute days of week mask
          uint32_t dow_mask = kDOWNone;
          for (uint32_t x = 0; x < (uint32_t)stop_pair.service_days_of_week_size(); x++) {
            bool dow = stop_pair.service_days_of_week(x);
            if (dow) {
              switch (x) {
                case 0:
                  dow_mask |= kMonday;
                  break;
                case 1:
                  dow_mask |= kTuesday;
                  break;
                case 2:
                  dow_mask |= kWednesday;
                  break;
                case 3:
                  dow_mask |= kThursday;
                  break;
                case 4:
                  dow_mask |= kFriday;
                  break;
                case 5:
                  dow_mask |= kSaturday;
                  break;
                case 6:
                  dow_mask |= kSunday;
                  break;
              }
            }
          }

          // service_start_date are relative to our pivot date
          auto d = date::floor<date::days>(DateTime::pivot_date_);
          date::sys_days start_date = date::sys_days(date::year_month_day(
              d +
              date::floor<date::days>(date::days(stop_pair.service_start_date() / kSecondsPerDay))));
          date::sys_days end_date = date::sys_days(date::year_month_day(
              d + date::ceil<date::days>(date::days(stop_pair.service_end_date() / kSecondsPerDay))));

          uint64_t days = get_service_days(start_date, end_date, tile_date, dow_mask);

          // if this is a service addition for one day, delete the dow_mask.
          // TODO(nils): why? it's still valid for this one day right? So rather find out that dow?
          if (stop_pair.service_start_date() == stop_pair.service_end_date()) {
            dow_mask = kDOWNone;
          }

          date::sys_days t_d = date::sys_days(date::year_month_day(d + date::days(tile_date)));
          uint32_t end_day = static_cast<uint32_t>((end_date - t_d).count());

          if (end_day > kScheduleEndDay) {
            end_day = kScheduleEndDay;
          }

          // if subtractions are between start and end date then turn off bit.
          for (const auto& x : stop_pair.service_except_dates()) {
            date::sys_days rm_date =
                date::sys_days(date::year_month_day(d + date::days(x / kSecondsPerDay)));
            days = remove_service_day(days, end_date, tile_date, rm_date);
          }

          // if additions are between start and end date then turn on bit.
          for (const auto& x : stop_pair.service_added_dates()) {
            date::sys_days add_date =
                date::sys_days(date::year_month_day(d + date::days(x / kSecondsPerDay)));
            days = add_service_day(days, end_date, tile_date, add_date);
          }

          // skip if no days are valid; this can happen a lot when there's a calendar.txt entry with
          // no valid days but some calendar_dates.txt entries which add a date lying in the past
          if (!days && !dow_mask) {
            stats.invalid_service_dates++;
            continue;
          }

          // if we have shape data then set everything, else shapeid = 0;
          if (stop_pair.has_shape_id() && stop_pair.has_destination_dist_traveled() &&
              stop_pair.has_origin_dist_traveled()) {
            dep.shapeid = stop_pair.shape_id();
            dep.orig_dist_traveled = stop_pair.origin_dist_traveled();
            dep.dest_dist_traveled = stop_pair.destination_dist_traveled();
          } else {
            dep.shapeid = 0;
          }

          dep.blockid = stop_pair.has_block_id() ? stop_pair.block_id() : 0;
          dep.dep_time = stop_pair.origin_departure_time();
          dep.elapsed_time = stop_pair.destination_arrival_time() - dep.dep_time;
          dep.headsign_offset = transit_tilebuilder.AddName(stop_pair.trip_headsign());

          dep.frequency_end_time =
              stop_pair.has_frequency_end_time() ? stop_pair.frequency_end_time() : 0;
          dep.frequency =
              stop_pair.has_frequency_headway_seconds() ? stop_pair.frequency_headway_seconds() : 0;

          if (!stop_pair.bikes_allowed()) {
            stop_no_access[dep.orig_pbf_graphid] |= kBicycleAccess;
            stop_no_access[dep.dest_pbf_graphid] |= kBicycleAccess;
          }

          if (!stop_pair.wheelchair_accessible()) {
            stop_no_access[dep.orig_pbf_graphid] |= kWheelchairAccess;
            stop_no_access[dep.dest_pbf_graphid] |= kWheelchairAccess;
          }

          dep.bicycle_accessible = stop_pair.bikes_allowed();
          dep.wheelchair_accessible = stop_pair.wheelchair_accessible();

          add_schedule(dep, days, dow_mask, end_day);

          // is this past midnight?
          // create a departure for before midnight and one after
          uint32_t origin_seconds = stop_pair.origin_departure_time();
          if (origin_seconds >= kSecondsPerDay) {

            // Add the current dep to the departures list
            // and then update it with new dep time.  This
            // dep will be used when the start time is after
            // midnight.
            stats.midnight_dep_count++;
            departures.emplace(dep.orig_pbf_graphid, dep);
            while (origin_seconds >= kSecondsPerDay) {
              origin_seconds -= kSecondsPerDay;
              // Then we need to fix the dow mask and dates
              // The departure that was initially for every Friday   26h
              // needs to be for                      every Saturday 02h
              // If there was an exception on the Friday 11th of January,
              // then we need an exception on the Saturday 12th of January instead
              days = shift_service_day(days);
              dow_mask =
                  ((dow_mask << 1) & kAllDaysOfWeek) | (dow_mask & kSaturday ? kSunday : kDOWNone);

              add_schedule(dep, days, dow_mask, end_day);
            }

            dep.dep_time = origin_seconds;
            // if there used to be a frequency, there'll be one now
            if (dep.frequency_end_time && dep.frequency) {
              uint32_t frequency_end_time = stop_pair.frequency_end_time();
              // adjust the end time if it is after midnight.
              while (frequency_end_time >= kSecondsPerDay) {
                frequency_end_time -= kSecondsPerDay;
              }

              dep.frequency_end_time = frequency_end_time;
              dep.frequency = stop_pair.frequency_headway_seconds();
            }
          }
          // Add to the departures list
          departures.emplace(dep.orig_pbf_graphid, std::move(dep));
          stats.dep_count++;
        }
      }
    }
  }
  return departures;
}

// Add routes to the tile. Return a vector of route types.
std::vector<uint32_t> AddRoutes(const Transit& pbf_tile, GraphTileBuilder& tilebuilder) {
  // Route types vs. index
  std::vector<uint32_t> route_types;

  for (uint32_t i = 0; i < (uint32_t)pbf_tile.routes_size(); i++) {
    const Transit_Route& r = pbf_tile.routes(i);

    // These should all be correctly set in the fetcher as it tosses types that we
    // don't support.  However, let's report an error if we encounter one.
    TransitType route_type = static_cast<TransitType>(r.vehicle_type());
    switch (route_type) {
      case TransitType::kTram:      // Tram, streetcar, lightrail
      case TransitType::kMetro:     // Subway, metro
      case TransitType::kRail:      // Rail
      case TransitType::kBus:       // Bus
      case TransitType::kFerry:     // Ferry
      case TransitType::kCableCar:  // Cable car
      case TransitType::kGondola:   // Gondola (suspended cable car)
      case TransitType::kFunicular: // Funicular (steep incline)
        break;
      default:
        // Log an unsupported vehicle type, set to bus for now
        LOG_ERROR("Unsupported vehicle type!");
        route_type = TransitType::kBus;
        break;
    }

    TransitRoute route(route_type, tilebuilder.AddName(r.onestop_id()),
                       tilebuilder.AddName(r.operated_by_onestop_id()),
                       tilebuilder.AddName(r.operated_by_name()),
                       tilebuilder.AddName(r.operated_by_website()), r.route_color(),
                       r.route_text_color(), tilebuilder.AddName(r.name()),
                       tilebuilder.AddName(r.route_long_name()), tilebuilder.AddName(r.route_desc()));
    LOG_DEBUG("Route idx = " + std::to_string(i) + ": " + r.name() + "," + r.route_long_name());
    tilebuilder.AddTransitRoute(route);

    // Route type - need this to store in edge.
    route_types.push_back(r.vehicle_type());
  }
  return route_types;
}

// Get Use given the transit route type
// TODO - add separate Use for different types - when we do this change
// the directed edge IsTransit method
Use GetTransitUse(const uint32_t rt) {
  switch (static_cast<TransitType>(rt)) {
    default:
    case TransitType::kTram:      // Tram, streetcar, lightrail
    case TransitType::kMetro:     // Subway, metro
    case TransitType::kRail:      // Rail
    case TransitType::kCableCar:  // Cable car
    case TransitType::kGondola:   // Gondola (suspended cable car)
    case TransitType::kFunicular: // Funicular (steep incline)
      return Use::kRail;
    case TransitType::kBus: // Bus
      return Use::kBus;
    case TransitType::kFerry: // Ferry (boat)
      return Use::kRail;      // TODO - add ferry use
  }
}

std::list<PointLL> GetShape(const PointLL& stop_ll,
                            const PointLL& endstop_ll,
                            uint32_t shapeid,
                            const float orig_dist_traveled,
                            const float dest_dist_traveled,
                            const std::vector<PointLL>& trip_shape,
                            const std::vector<float>& distances) {

  std::list<PointLL> shape;
  if (shapeid != 0 && trip_shape.size() && stop_ll != endstop_ll &&
      orig_dist_traveled < dest_dist_traveled) {

    float distance = 0.0f, d_from_p0_to_x = 0.0f;

    // point x - we are trying to find it on the line segment between p0 and p1
    PointLL x;
    // find out where orig_dist_traveled should be in the list.
    auto lower_bound = std::lower_bound(distances.cbegin(), distances.cend(), orig_dist_traveled);
    // find out where dest_dist_traveled should be in the list.
    auto upper_bound = std::upper_bound(distances.cbegin(), distances.cend(), dest_dist_traveled);
    float prev_distance = *(lower_bound);

    // distance calculations can be off just a bit (i.e., 9372.224609 < 9372.500000) so set it to
    // the last element.
    if (distances.back() < dest_dist_traveled) {
      upper_bound = distances.cend() - 1;
    }

    // lower_bound returns an iterator pointing to the first element which does not compare less
    // than the dist_traveled; therefore, we need to back up one if it does not equal the
    // lower_bound value.  For example, we could be starting at the beginning of the points list
    if (orig_dist_traveled != (*lower_bound)) {
      prev_distance = *(--lower_bound);
    }

    // loop through the points.
    for (auto itr = lower_bound; itr != upper_bound; ++itr) {

      /*    |
       *    |
       *    p0
       *    | }--d_from_p0_to_x (distance from p0 to x)
       *    x -- point we are trying to find on the segment (orig_dist_traveled or
       * dest_dist_traveled on this segment)
       *    |
       *    |
       *    |
       *    |
       *    p1
       *    |
       *    |
       */

      // index into our vector of points
      uint32_t index = (itr - distances.cbegin());
      PointLL p0 = trip_shape[index];
      PointLL p1 = trip_shape[index + 1];

      // this is our distance that is beyond x.
      distance = *(itr + 1);

      // find point x using the orig_dist_traveled - this is our first point added to shape
      if (itr == lower_bound) {
        if (orig_dist_traveled == *itr) { // just add p0
          shape.push_back(p0);
        } else {
          // distance from p0 to x using the orig_dist_traveled
          d_from_p0_to_x = (orig_dist_traveled - prev_distance) / (distance - prev_distance);
          x = p0 + (p1 - p0) * d_from_p0_to_x;
          shape.push_back(x);
        }
      }

      // find point x using the dest_dist_traveled - this is our last point added to the shape
      if ((itr + 1) == upper_bound) {
        if (dest_dist_traveled == *itr) { // just add p0
          if (shape.back() != p0) {       // avoid dups
            shape.push_back(p0);
          }
        } else {
          // distance from p0 to x using the dest_dist_traveled
          d_from_p0_to_x = (dest_dist_traveled - prev_distance) / (distance - prev_distance);
          x = p0 + (p1 - p0) * d_from_p0_to_x;

          if (shape.back() != x) { // avoid dups
            shape.push_back(x);
          }
          // we are done p1 is too far away
        }
        break;
      }
      // add all the midpoints.
      shape.push_back(p1);

      prev_distance = distance;
    }
    // else no shape exists.
  } else {
    shape.push_back(stop_ll);
    shape.push_back(endstop_ll);
  }

  if (shape.size() == 0) {
    LOG_ERROR("Invalid shape between " + stop_ll.to_string() + " and " + endstop_ll.to_string());
    shape.push_back(stop_ll);
    shape.push_back(endstop_ll);
  }

  return shape;
}

// TODO: we cannot leave this function like this! the code in its current state is completely
//  illegible due to copy pasting. there are multiple nested shadowings of variables which encourages
//  mistakes. at the very least we should break out the 3 different types of connections into separate
//  functions. it would be great if it could be a single function with different arguments to get all
//  3 done but if not at least separating them will remove the shadowing. we cannot call transit done
//  until this is rectified
void AddToGraph(GraphTileBuilder& tilebuilder_transit,
                const GraphId& tileid,
                const Transit& tile_pbf,
                const std::string& transit_dir,
                std::mutex& lock,
                const std::map<GraphId, StopEdges>& stop_edge_map,
                const std::unordered_map<GraphId, uint16_t>& stop_no_access,
                const std::unordered_map<uint32_t, Shape>& shape_data,
                const std::vector<float>& distances,
                const std::vector<uint32_t>& route_types,
                const std::multimap<uint32_t, Geometry>& tz_polys,
                const PlatformSplitPlan& split_plan,
                uint32_t& no_dir_edge_count) {
  auto t1 = std::chrono::high_resolution_clock::now();

  std::set<uint64_t> added_stations;
  std::set<uint64_t> added_egress;

  // Never let NodeInfo::set_edge_count clamp a transit node: the edges past the clamp would have
  // no owner and their connections would be lost without a trace.
  auto checked_edge_count = [&tileid](const uint32_t edge_count, const Transit_Node& stop) {
    if (edge_count > kMaxEdgesPerNode) {
      throw std::runtime_error(
          "convert_transit: transit stop " + stop.name() + " (" + stop.onestop_id() +
          ") in transit tile " + std::to_string(tileid.tileid()) + " needs " +
          std::to_string(edge_count) +
          " outbound edges, more than kMaxEdgesPerNode = " + std::to_string(kMaxEdgesPerNode));
    }
    return edge_count;
  };

  // Overflow platform nodes of the split platforms in this tile, filled in while the platform
  // itself is built and appended after all pbf nodes.
  struct OverflowNode {
    NodeInfo node;
    const Transit_Node* platform;
    PointLL platform_ll;
    GraphId station_graphid;
    PointLL station_ll;
    std::vector<const TransitLine*> lines;
  };
  std::unordered_map<GraphId, OverflowNode> overflow_nodes;

  // Data looks like the following.stop_index(
  // Egress1_for_Station_A
  // Egress2_for_Station_A
  // Station_A
  // Platform1_for_Station_A
  // Platform2_for_Station_A
  // Egress_for_Station_B
  // Station_B
  // Platform_for_Station_B
  // . . . and so on

  //  tiles will look like the following with N egresses and N platforms.
  //  osm--------->egress--------->station--------->platform
  //  node<---------node<-----------node<-------------node

  // osm and egress nodes are connected by transitconnections.
  // egress and stations are connected by egressconnections.
  // stations and platforms are connected by platformconnections

  // Add the directed edge of one transit line leaving start_graphid (a platform node or one of
  // its overflow nodes), whose first outbound edge is node_edge_index.
  [[maybe_unused]] uint32_t transitedges = 0;
  auto add_line = [&](const TransitLine& transitedge, const GraphId& start_graphid,
                      const PointLL& start_ll, const uint32_t node_edge_index) {
    // Get the end node. Skip this directed edge if the Valhalla tile is
    // not valid (or empty)
    GraphId end_platform_graphid(transitedge.dest_pbf_graphid);
    if (!end_platform_graphid.is_valid()) {
      LOG_ERROR("Unstitched stop pair detected with origin near " + std::to_string(start_ll.lat()) +
                ',' + std::to_string(start_ll.lng()));
      return;
    }

    // Find the lat,lng of the end stop
    PointLL endll;
    std::string endstopname;
    if (end_platform_graphid.tile_base() == tileid) {
      // End stop is in the same pbf transit tile
      const Transit_Node& endplatform = tile_pbf.nodes(end_platform_graphid.id());
      endstopname = endplatform.name();
      endll = {endplatform.lon(), endplatform.lat()};
    } else {
      // Get Transit PBF data for this tile
      // Get transit pbf tile
      std::string file_name = GraphTile::FileSuffix(
          GraphId(end_platform_graphid.tileid(), end_platform_graphid.level(), 0));
      boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
      file_name += ".pbf";
      std::filesystem::path file_path{transit_dir};
      file_path.append(file_name);
      Transit endtransit = read_pbf(file_path.string(), lock);
      const Transit_Node& endplatform = endtransit.nodes(end_platform_graphid.id());
      endstopname = endplatform.name();
      endll = {endplatform.lon(), endplatform.lat()};
    }

    // Add the directed edge
    // A route that continues through a split platform arrives at the slot that holds its
    // outbound lines there, so its trips stay on one node.
    GraphId end_node_graphid = end_platform_graphid;
    const auto dest_split = split_plan.splits.find(end_platform_graphid);
    if (dest_split != split_plan.splits.end()) {
      const auto slot =
          dest_split->second.route_slot.find(tile_pbf.routes(transitedge.routeid).onestop_id());
      if (slot != dest_split->second.route_slot.end() && slot->second > 0) {
        end_node_graphid = dest_split->second.overflow[slot->second - 1];
      }
    }

    DirectedEdge directededge;
    directededge.set_endnode(end_node_graphid);
    directededge.set_length(start_ll.Distance(endll));
    Use use = GetTransitUse(route_types[transitedge.routeid]);
    directededge.set_use(use);
    directededge.set_speed(5);
    directededge.set_classification(RoadClass::kServiceOther);
    directededge.set_localedgeidx(tilebuilder_transit.directededges().size() - node_edge_index);
    directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
    directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
    directededge.set_lineid(transitedge.lineid);

    LOG_DEBUG("Add transit directededge - lineId = " + std::to_string(transitedge.lineid) +
              " Route Key = " + std::to_string(transitedge.routeid) + " EndStop " + endstopname);

    // Add edge info to the tile and set the offset in the directed edge
    // Leave the name empty. Use the trip Id to look up the route Id and
    // route within TripLegBuilder.
    bool added = false;
    std::vector<std::string> names, tagged_values, linguistics;

    std::vector<PointLL> points;
    std::vector<float> distance;
    // get the indexes and vector of points for this shape id
    const auto& found = shape_data.find(transitedge.shapeid);
    if (transitedge.shapeid != 0 && found != shape_data.cend()) {
      const auto& shape_d = found->second;
      points = shape_d.shape;
      // copy only the distances that we care about.
      std::copy((distances.cbegin() + shape_d.begins), (distances.cbegin() + shape_d.ends),
                back_inserter(distance));
    } else if (transitedge.shapeid != 0) {
      LOG_WARN("Shape Id not found: " + std::to_string(transitedge.shapeid));
    }

    // TODO - if we separate transit edges based on more than just routeindex
    // we will need to do something to differentiate edges (maybe use
    // lineid) so the shape doesn't get messed up.
    auto shape = GetShape(start_ll, endll, transitedge.shapeid, transitedge.orig_dist_traveled,
                          transitedge.dest_dist_traveled, points, distance);

    uint32_t edge_info_offset =
        tilebuilder_transit.AddEdgeInfo(transitedge.routeid, start_graphid, end_node_graphid, 0, 0, 0,
                                        0, shape, names, tagged_values, linguistics, 0, added);

    directededge.set_edgeinfo_offset(edge_info_offset);
    directededge.set_forward(added);

    // Add to list of directed edges
    tilebuilder_transit.directededges().emplace_back(std::move(directededge));
    transitedges++;
  };

  // Iterate through the platform and their edges
  for (const auto& stop_edges : stop_edge_map) {
    // Get the platform information
    GraphId platform_graphid = stop_edges.second.origin_pbf_graphid;
    const Transit_Node& platform = tile_pbf.nodes(platform_graphid.id());
    if (GraphId(platform.graphid()) != platform_graphid) {
      LOG_ERROR("Platform key not equal!");
    }

    LOG_DEBUG("Transit Platform: " + platform.name() +
              " index= " + std::to_string(platform_graphid.id()));
    PointLL platform_ll = {platform.lon(), platform.lat()};

    // the prev_type_graphid is actually the station or parent in
    // platforms
    GraphId parent(platform.prev_type_graphid());
    const Transit_Node& station = tile_pbf.nodes(parent.id());

    // Get the Valhalla graphId of the station node
    GraphId station_graphid(station.graphid());

    PointLL station_ll = {station.lon(), station.lat()};
    // Build the station node if it has not already been added.
    if (added_stations.find(platform.prev_type_graphid()) == added_stations.end()) {

      // Build the station node
      uint32_t n_access = (kPedestrianAccess | kWheelchairAccess | kBicycleAccess);
      auto s_access = stop_no_access.find(station_graphid);
      if (s_access != stop_no_access.end()) {
        n_access &= ~s_access->second;
      }

      // Set the station lat,lon using the tile base LL
      PointLL base_ll = tilebuilder_transit.header_builder().base_ll();
      NodeInfo station_node(base_ll, station_ll, n_access, NodeType::kTransitStation, false, true,
                            false, false);
      station_node.set_stop_index(station_graphid.id());

      const std::string& tz = station.has_timezone() ? station.timezone() : "";
      uint32_t timezone = 0;
      if (!tz.empty()) {
        timezone = DateTime::get_tz_db().to_index(tz);
      }

      if (timezone == 0) {
        // fallback to tz database.
        timezone =
            (tz_polys.size() == 1) ? tz_polys.begin()->first : GetMultiPolyId(tz_polys, station_ll);

        if (timezone == 0) {
          LOG_WARN("Timezone not found for station " + station.name());
        }
      }
      station_node.set_timezone(timezone);

      LOG_DEBUG("Transit Platform: " + platform.name() +
                " index= " + std::to_string(platform_graphid.id()));

      // set the index to the first egress.
      // loop over egresses add the DE to the station from the egress
      // there is always at least one egress and they are before the stations in the pbf
      GraphId eg = GraphId(station.prev_type_graphid());
      uint32_t index = eg.id();

      while (true) {
        const Transit_Node& egress = tile_pbf.nodes(index);
        if (static_cast<NodeType>(egress.type()) != NodeType::kTransitEgress) {
          break;
        }

        // Get the Valhalla graphId of the origin node (transit stop)
        GraphId egress_graphid(egress.graphid());
        DirectedEdge directededge;
        directededge.set_endnode(station_graphid);
        PointLL egress_ll = {egress.lon(), egress.lat()};

        // Build the egress node
        uint32_t n_access = (kPedestrianAccess | kWheelchairAccess | kBicycleAccess);
        auto s_access = stop_no_access.find(egress_graphid);
        if (s_access != stop_no_access.end()) {
          n_access &= ~s_access->second;
        }

        const std::string& tz = egress.has_timezone() ? egress.timezone() : "";
        uint32_t timezone = 0;
        if (!tz.empty()) {
          timezone = DateTime::get_tz_db().to_index(tz);
        }

        if (timezone == 0) {
          // fallback to tz database.
          timezone =
              (tz_polys.size() == 1) ? tz_polys.begin()->first : GetMultiPolyId(tz_polys, egress_ll);
          if (timezone == 0) {
            LOG_WARN("Timezone not found for egress " + egress.name());
          }
        }

        // Set the egress lat,lon using the tile base LL
        PointLL base_ll = tilebuilder_transit.header_builder().base_ll();
        NodeInfo egress_node(base_ll, egress_ll, n_access, NodeType::kTransitEgress, false, true,
                             false, false);
        egress_node.set_stop_index(index);
        egress_node.set_timezone(timezone);
        egress_node.set_edge_index(tilebuilder_transit.directededges().size());
        if (egress.has_osm_connecting_lat() && egress.has_osm_connecting_lon()) {
          egress_node.set_connecting_point(
              PointLL(egress.osm_connecting_lon(), egress.osm_connecting_lat()));
        } else if (egress.has_osm_connecting_way_id()) {
          egress_node.set_connecting_wayid(egress.osm_connecting_way_id());
        }

        // add the egress connection
        // Make sure length is non-zero
        double length = std::max(1.0, egress_ll.Distance(station_ll));
        directededge.set_length(length);
        directededge.set_use(Use::kEgressConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                      egress_node.edge_index());
        directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_named(false);

        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::vector<std::string> names, tagged_values, linguistics;

        std::list<PointLL> shape = {egress_ll, station_ll};

        uint32_t edge_info_offset =
            tilebuilder_transit.AddEdgeInfo(0, egress_graphid, station_graphid, 0, 0, 0, 0, shape,
                                            names, tagged_values, linguistics, 0, added);
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(true);

        // Add to list of directed edges
        tilebuilder_transit.directededges().emplace_back(std::move(directededge));

        // set the count to 1 DE
        // osm connections will be added later.
        egress_node.set_edge_count(1);
        // Add the egress node
        tilebuilder_transit.nodes().emplace_back(std::move(egress_node));
        index++;
      }

      station_node.set_edge_index(tilebuilder_transit.directededges().size());
      // now add the DE to the egress from the station.
      // index now points to the station.
      for (uint32_t j = eg.id(); j < index; j++) {

        const Transit_Node& egress = tile_pbf.nodes(j);
        PointLL egress_ll = {egress.lon(), egress.lat()};

        // Get the Valhalla graphId of the origin node (transit stop)
        GraphId egress_graphid(egress.graphid());
        DirectedEdge directededge;
        directededge.set_endnode(egress_graphid);

        // add the platform connection
        // Make sure length is non-zero
        double length = std::max(1.0, station_ll.Distance(egress_ll));
        directededge.set_length(length);
        directededge.set_use(Use::kEgressConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                      station_node.edge_index());
        directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_named(false);
        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::vector<std::string> names, tagged_values, linguistics;
        std::list<PointLL> shape = {station_ll, egress_ll};

        // TODO - these need to be valhalla graph Ids
        uint32_t edge_info_offset =
            tilebuilder_transit.AddEdgeInfo(0, station_graphid, egress_graphid, 0, 0, 0, 0, shape,
                                            names, tagged_values, linguistics, 0, added);
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(false);

        // Add to list of directed edges
        tilebuilder_transit.directededges().emplace_back(std::move(directededge));
      }

      // advance the index to skip the station and point to the first platform
      index++;
      const uint32_t first_platform = index;
      while (true) {

        if (index == (uint32_t)tile_pbf.nodes_size()) {
          break;
        }

        const Transit_Node& platform = tile_pbf.nodes(index);
        if (static_cast<NodeType>(platform.type()) != NodeType::kMultiUseTransitPlatform) {
          break;
        }

        // Get the Valhalla graphId of the origin node (transit stop)
        GraphId platform_graphid(platform.graphid());
        DirectedEdge directededge;
        directededge.set_endnode(platform_graphid);
        PointLL platform_ll = {platform.lon(), platform.lat()};

        // add the platform connection
        // Make sure length is non-zero
        double length = std::max(1.0, station_ll.Distance(platform_ll));
        directededge.set_length(length);
        directededge.set_use(Use::kPlatformConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                      station_node.edge_index());
        directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_named(false);

        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::vector<std::string> names, tagged_values, linguistics;
        std::list<PointLL> shape = {station_ll, platform_ll};

        // TODO - these need to be valhalla graph Ids
        uint32_t edge_info_offset =
            tilebuilder_transit.AddEdgeInfo(0, station_graphid, platform_graphid, 0, 0, 0, 0, shape,
                                            names, tagged_values, linguistics, 0, added);
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(true);

        // Add to list of directed edges
        tilebuilder_transit.directededges().emplace_back(std::move(directededge));
        index++;
      }

      // Platform connections to the overflow nodes of this station's split platforms
      for (uint32_t p = first_platform; p < index; p++) {
        const Transit_Node& platform = tile_pbf.nodes(p);
        const auto split = split_plan.splits.find(GraphId(platform.graphid()));
        if (split == split_plan.splits.end()) {
          continue;
        }
        if (split->second.station != station_graphid) {
          throw std::runtime_error("convert_transit: split platform " + platform.name() + " (" +
                                   platform.onestop_id() + ") is not a child of station " +
                                   station.name());
        }
        PointLL platform_ll = {platform.lon(), platform.lat()};
        for (const GraphId& overflow_graphid : split->second.overflow) {
          DirectedEdge directededge;
          directededge.set_endnode(overflow_graphid);
          directededge.set_length(std::max(1.0, station_ll.Distance(platform_ll)));
          directededge.set_use(Use::kPlatformConnection);
          directededge.set_speed(5);
          directededge.set_classification(RoadClass::kServiceOther);
          directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                        station_node.edge_index());
          directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
          directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
          directededge.set_named(false);
          bool added = false;
          std::vector<std::string> names, tagged_values, linguistics;
          std::list<PointLL> shape = {station_ll, platform_ll};
          uint32_t edge_info_offset =
              tilebuilder_transit.AddEdgeInfo(0, station_graphid, overflow_graphid, 0, 0, 0, 0, shape,
                                              names, tagged_values, linguistics, 0, added);
          directededge.set_edgeinfo_offset(edge_info_offset);
          directededge.set_forward(true);
          tilebuilder_transit.directededges().emplace_back(std::move(directededge));
        }
      }

      // Get the directed edge count, log an error if no directed edges are added
      uint32_t edge_count = tilebuilder_transit.directededges().size() - station_node.edge_index();
      if (edge_count == 0) {
        // Set the edge index to 0
        // TODO: add ERROR log for no directed edges out of the station
        station_node.set_edge_index(0);
        no_dir_edge_count++;
      }

      // Add the node
      station_node.set_edge_count(checked_edge_count(edge_count, station));
      tilebuilder_transit.nodes().emplace_back(std::move(station_node));
      added_stations.emplace(platform.prev_type_graphid());
    }

    // Build the platform node
    uint32_t n_access = (kPedestrianAccess | kWheelchairAccess | kBicycleAccess);
    auto s_access = stop_no_access.find(platform_graphid);
    if (s_access != stop_no_access.end()) {
      n_access &= ~s_access->second;
    }

    const std::string& tz = platform.has_timezone() ? platform.timezone() : "";
    uint32_t timezone = 0;
    if (!tz.empty()) {
      timezone = DateTime::get_tz_db().to_index(tz);
    }

    if (timezone == 0) {
      // fallback to tz database.
      timezone =
          (tz_polys.size() == 1) ? tz_polys.begin()->first : GetMultiPolyId(tz_polys, platform_ll);
      if (timezone == 0) {
        LOG_WARN("Timezone not found for platform " + platform.name());
      }
    }

    // Set the platform lat,lon using the tile base LL
    PointLL base_ll = tilebuilder_transit.header_builder().base_ll();
    NodeInfo platform_node(base_ll, platform_ll, n_access, NodeType::kMultiUseTransitPlatform, false,
                           true, false, false);
    platform_node.set_mode_change(true);
    platform_node.set_stop_index(platform_graphid.id());
    platform_node.set_timezone(timezone);
    platform_node.set_edge_index(tilebuilder_transit.directededges().size());

    // Add DE to the station from the platform
    DirectedEdge directededge;
    directededge.set_endnode(station_graphid);

    // add the platform connection
    // Make sure length is non-zero
    double length = std::max(1.0, platform_ll.Distance(station_ll));
    directededge.set_length(length);
    directededge.set_use(Use::kPlatformConnection);
    directededge.set_speed(5);
    directededge.set_classification(RoadClass::kServiceOther);
    directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                  platform_node.edge_index());
    directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
    directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
    directededge.set_named(false);
    // Add edge info to the tile and set the offset in the directed edge
    bool added = false;
    std::vector<std::string> names, tagged_values, linguistics;
    std::list<PointLL> shape = {platform_ll, station_ll};

    // TODO - these need to be valhalla graph Ids
    uint32_t edge_info_offset =
        tilebuilder_transit.AddEdgeInfo(0, platform_graphid, station_graphid, 0, 0, 0, 0, shape,
                                        names, tagged_values, linguistics, 0, added);

    directededge.set_edgeinfo_offset(edge_info_offset);
    directededge.set_forward(false);

    // Add to list of directed edges
    tilebuilder_transit.directededges().emplace_back(std::move(directededge));

    // Add transit lines
    // level 3
    const auto split = split_plan.splits.find(platform_graphid);
    if (split != split_plan.splits.end()) {
      for (const GraphId& overflow_graphid : split->second.overflow) {
        overflow_nodes.emplace(overflow_graphid, OverflowNode{platform_node,
                                                              &platform,
                                                              platform_ll,
                                                              station_graphid,
                                                              station_ll,
                                                              {}});
      }
    }
    for (const auto& transitedge : stop_edges.second.lines) {
      uint32_t slot = 0;
      if (split != split_plan.splits.end()) {
        const auto route_slot =
            split->second.route_slot.find(tile_pbf.routes(transitedge.routeid).onestop_id());
        if (route_slot == split->second.route_slot.end()) {
          throw std::runtime_error("convert_transit: no slot planned for route " +
                                   tile_pbf.routes(transitedge.routeid).onestop_id() +
                                   " at platform " + platform.name() + " (" + platform.onestop_id() +
                                   ")");
        }
        slot = route_slot->second;
      }
      if (slot == 0) {
        add_line(transitedge, platform_graphid, platform_ll, platform_node.edge_index());
      } else {
        overflow_nodes.at(split->second.overflow[slot - 1]).lines.push_back(&transitedge);
      }
    }

    // Get the directed edge count, log an error if no directed edges are added
    // TODO: log error
    uint32_t edge_count = tilebuilder_transit.directededges().size() - platform_node.edge_index();
    if (edge_count == 0) {
      // Set the edge index to 0
      platform_node.set_edge_index(0);
      no_dir_edge_count++;
    }

    // Add the node
    platform_node.set_edge_count(checked_edge_count(edge_count, platform));
    tilebuilder_transit.nodes().emplace_back(std::move(platform_node));
  }

  // Append the overflow platform nodes after all pbf nodes, in the node id order planned by
  // PlanTileSplits: a platform connection back to the station, then the lines of its routes.
  const auto tile_overflow = split_plan.tile_overflow.find(tileid.tile_base());
  if (tile_overflow != split_plan.tile_overflow.end()) {
    if (tilebuilder_transit.nodes().size() != static_cast<size_t>(tile_pbf.nodes_size())) {
      throw std::runtime_error("convert_transit: transit tile " + std::to_string(tileid.tileid()) +
                               " has " + std::to_string(tilebuilder_transit.nodes().size()) +
                               " nodes but its pbf has " + std::to_string(tile_pbf.nodes_size()) +
                               "; cannot append overflow platform nodes");
    }
    for (const auto& platform_slot : tile_overflow->second) {
      const GraphId overflow_graphid =
          split_plan.splits.at(platform_slot.first).overflow[platform_slot.second - 1];
      if (overflow_graphid.id() != tilebuilder_transit.nodes().size()) {
        throw std::runtime_error("convert_transit: overflow node id mismatch in transit tile " +
                                 std::to_string(tileid.tileid()));
      }
      OverflowNode& overflow = overflow_nodes.at(overflow_graphid);
      NodeInfo& node = overflow.node;
      node.set_edge_index(tilebuilder_transit.directededges().size());

      DirectedEdge directededge;
      directededge.set_endnode(overflow.station_graphid);
      directededge.set_length(std::max(1.0, overflow.platform_ll.Distance(overflow.station_ll)));
      directededge.set_use(Use::kPlatformConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(0);
      directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
      directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
      directededge.set_named(false);
      bool added = false;
      std::vector<std::string> names, tagged_values, linguistics;
      std::list<PointLL> shape = {overflow.platform_ll, overflow.station_ll};
      uint32_t edge_info_offset =
          tilebuilder_transit.AddEdgeInfo(0, overflow_graphid, overflow.station_graphid, 0, 0, 0, 0,
                                          shape, names, tagged_values, linguistics, 0, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(false);
      tilebuilder_transit.directededges().emplace_back(std::move(directededge));

      for (const TransitLine* line : overflow.lines) {
        add_line(*line, overflow_graphid, overflow.platform_ll, node.edge_index());
      }
      node.set_edge_count(
          checked_edge_count(tilebuilder_transit.directededges().size() - node.edge_index(),
                             *overflow.platform));
      tilebuilder_transit.nodes().emplace_back(node);
    }
  }

  // Log the number of added nodes and edges
  auto t2 = std::chrono::high_resolution_clock::now();
  [[maybe_unused]] uint32_t msecs =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("Tile " + std::to_string(tileid.tileid()) + ": added " + std::to_string(transitedges) +
           " transit edges, and " + std::to_string(tilebuilder_transit.nodes().size()) +
           " nodes. time = " + std::to_string(msecs) + " ms");
}

// We make sure to lock on reading and writing since tiles are now being
// written. Also lock on queue access since shared by different threads.
void build_tiles_impl(const boost::property_tree::ptree& pt,
                      std::mutex& lock,
                      std::unordered_set<GraphId>::const_iterator tile_start,
                      std::unordered_set<GraphId>::const_iterator tile_end,
                      const PlatformSplitPlan& split_plan,
                      std::promise<builder_stats>& results) {
  builder_stats stats;

  GraphReader reader(pt);
  auto database = pt.get_optional<std::string>("timezone");
  auto tz_db = AdminDB::open(*database);
  if (!tz_db) {
    LOG_WARN("Time zone db " + *database + " not found. Not saving time zone information from db.");
  }

  const auto& tiles = TileHierarchy::levels().back().tiles;
  // Iterate through the tiles in the queue and find any that include stops
  for (; tile_start != tile_end; ++tile_start) {
    // Get the next tile Id from the queue and get a tile builder
    if (reader.OverCommitted()) {
      reader.Trim();
    }
    GraphId tile_id = tile_start->tile_base();

    // Get transit pbf tile
    const std::string transit_dir = pt.get<std::string>("transit_dir");
    std::string file_name = GraphTile::FileSuffix(GraphId(tile_id.tileid(), tile_id.level(), 0));
    boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
    file_name += ".pbf";
    std::filesystem::path pbf_fp{transit_dir};
    pbf_fp.append(file_name);

    // Make sure it exists
    if (!std::filesystem::exists(pbf_fp)) {
      LOG_ERROR("File not found.  " + pbf_fp.string());
      results.set_value(stats);
      return;
    }

    Transit tile_pbf = read_pbf(pbf_fp.string(), lock);
    // Get Valhalla tile - get a read only instance for reference and
    // a writeable instance (deserialize it so we can add to it)
    lock.lock();

    GraphId transit_tile_id = GraphId(tile_id.tileid(), tile_id.level(), tile_id.id());
    graph_tile_ptr transit_tile = reader.GetGraphTile(transit_tile_id);
    GraphTileBuilder tilebuilder_transit(transit_dir, transit_tile_id, false);

    // normalize tile creation date on America/New_York timezone
    auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
    uint32_t tile_creation_date =
        DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));
    tilebuilder_transit.AddTileCreationDate(tile_creation_date);

    // Set the tile base LL
    PointLL base_ll = TileHierarchy::GetTransitLevel().tiles.Base(tile_id.tileid());
    tilebuilder_transit.header_builder().set_base_ll(base_ll);

    lock.unlock();

    std::unordered_map<GraphId, uint16_t> stop_no_access;
    // add Transit nodes in order.
    for (const auto& transit_node : tile_pbf.nodes()) {

      if (!transit_node.wheelchair_boarding()) {
        stop_no_access[GraphId(transit_node.graphid())] |= kWheelchairAccess;
      }

      // Store stop information in TransitStops
      tilebuilder_transit.AddTransitStop({tilebuilder_transit.AddName(transit_node.onestop_id()),
                                          tilebuilder_transit.AddName(transit_node.name()),
                                          transit_node.generated(), transit_node.traversability()});
    }

    // Get all the shapes for this tile and calculate the distances
    std::unordered_map<uint32_t, Shape> shapes;
    std::vector<float> distances;
    for (const auto& shape : tile_pbf.shapes()) {
      const std::vector<PointLL> trip_shape = decode7<std::vector<PointLL>>(shape.encoded_shape());

      float distance = 0.0f;
      Shape shape_data;
      // first is always 0.0f.
      distances.push_back(distance);
      shape_data.begins = distances.size() - 1;

      // loop through the points getting the distances.
      for (size_t index = 0; index < trip_shape.size() - 1; ++index) {
        PointLL p0 = trip_shape[index];
        PointLL p1 = trip_shape[index + 1];
        distance += p0.Distance(p1);
        distances.push_back(distance);
      }
      // must be distances.size for the end index as we use std::copy later on and want
      // to include the last element in the vector we wish to copy.
      shape_data.ends = distances.size();
      shape_data.shape = trip_shape;
      // shape id --> begin and end indexes in the distance vector and vector of points.
      shapes[shape.shape_id()] = shape_data;
    }

    // Get all scheduled departures from the stops within this tile.
    std::map<GraphId, StopEdges> stop_edge_map;
    uint32_t unique_lineid = 1;
    std::vector<TransitDeparture> transit_departures;

    // Process schedule stop pairs (departures)
    std::unordered_multimap<GraphId, Departure> departures =
        ProcessStopPairs(tilebuilder_transit, tile_creation_date, tile_pbf, stop_no_access, pbf_fp,
                         lock, stats);

    // Form departures and egress/station/platform hierarchy
    // TODO: get pathways.txt or some other means to better correlate the intra-station edges
    for (const auto& platform : tile_pbf.nodes()) {
      if (static_cast<NodeType>(platform.type()) != NodeType::kMultiUseTransitPlatform) {
        continue;
      }

      GraphId platform_pbf_graphid = GraphId(platform.graphid());
      StopEdges stopedges;
      stopedges.origin_pbf_graphid = platform_pbf_graphid;

      // TODO - perhaps replace this code with use of headsign below
      // to solve problem of a trip that doesn't go the whole way to
      // the end of the route line
      std::map<std::pair<uint32_t, GraphId>, uint32_t> unique_transit_edges;
      auto range = departures.equal_range(platform_pbf_graphid);

      for (auto key = range.first; key != range.second; ++key) {
        Departure dep = key->second;

        // Identify unique route and arrival stop pairs - associate to a
        // unique line Id stored in the directed edge.
        uint32_t lineid;
        auto m = unique_transit_edges.find({dep.route, dep.dest_pbf_graphid});
        if (m == unique_transit_edges.end()) {
          // Add to the map and update the line id
          lineid = unique_lineid;
          unique_transit_edges[{dep.route, dep.dest_pbf_graphid}] = unique_lineid;
          unique_lineid++;
          stopedges.lines.emplace_back(TransitLine{lineid, dep.route, dep.dest_pbf_graphid,
                                                   dep.shapeid, dep.orig_dist_traveled,
                                                   dep.dest_dist_traveled});
        } else {
          lineid = m->second;
        }

        // we prefer the frequency-based schedule if it was set
        try {
          if (dep.frequency == 0) {
            // Form transit departures -- fixed departure time
            TransitDeparture td(lineid, dep.trip, dep.route, dep.blockid, dep.headsign_offset,
                                dep.dep_time, dep.elapsed_time, dep.schedule_index,
                                dep.wheelchair_accessible, dep.bicycle_accessible);
            tilebuilder_transit.AddTransitDeparture(std::move(td));
          } else {

            // Form transit departures -- frequency departure time
            // TODO(nils): this doesn't differentiate between frequencies.txt entries with
            // exact_times true/false; it's a bit random right now what departure_time represents:
            // it's the time set by the stop_time's departure_time which might just be an example,
            // but not represent an actual departure_time. can't we just take the service's start_time
            // if it's exact_times=true or start_time + 0.5 * headway for exact_times=false?
            TransitDeparture td(lineid, dep.trip, dep.route, dep.blockid, dep.headsign_offset,
                                dep.dep_time, dep.frequency_end_time, dep.frequency, dep.elapsed_time,
                                dep.schedule_index, dep.wheelchair_accessible,
                                dep.bicycle_accessible);
            tilebuilder_transit.AddTransitDeparture(std::move(td));
          }
        } catch (const std::exception& e) { LOG_ERROR(e.what()); }
      }

      // TODO Get any transfers from this stop
      // AddTransfers(tilebuilder);

      // Add to stop edge map - track edges that need to be added. This is
      // sorted by graph Id so the stop nodes are added in proper order
      stop_edge_map.insert({platform_pbf_graphid, stopedges});
    }

    // Add routes to the tile. Get vector of route types.
    std::vector<uint32_t> route_types = AddRoutes(tile_pbf, tilebuilder_transit);
    std::multimap<uint32_t, Geometry> tz_polys;
    if (tz_db) {
      tz_polys = GetTimeZones(*tz_db, tiles.TileBounds(tile_id.tileid()));
    }

    // Add nodes, directededges, and edgeinfo
    AddToGraph(tilebuilder_transit, tile_id, tile_pbf, transit_dir, lock, stop_edge_map,
               stop_no_access, shapes, distances, route_types, tz_polys, split_plan,
               stats.no_dir_edge_count);

    LOG_INFO("Tile " + std::to_string(tile_id.tileid()) + ": added " +
             std::to_string(tile_pbf.nodes_size()) + " stops, " +
             std::to_string(tile_pbf.shapes_size()) + " shapes, " +
             std::to_string(route_types.size()) + " routes, and " +
             std::to_string(departures.size()) + " departures");

    // Write the new file
    lock.lock();
    tilebuilder_transit.StoreTileData();
    lock.unlock();
  }

  // Send back the statistics
  results.set_value(stats);
}

// Thread entry point: a tile that cannot be built hands its error to convert_transit instead of
// terminating inside the thread.
void build_tiles(const boost::property_tree::ptree& pt,
                 std::mutex& lock,
                 std::unordered_set<GraphId>::const_iterator tile_start,
                 std::unordered_set<GraphId>::const_iterator tile_end,
                 const PlatformSplitPlan& split_plan,
                 std::promise<builder_stats>& results) {
  try {
    build_tiles_impl(pt, lock, tile_start, tile_end, split_plan, results);
  } catch (...) { results.set_exception(std::current_exception()); }
}

} // namespace

namespace valhalla {
namespace mjolnir {

std::unordered_set<GraphId> convert_transit(const ptree& pt) {

  // figure out which transit tiles even exist
  std::filesystem::path transit_dir{pt.get<std::string>("mjolnir.transit_dir")};
  transit_dir.append(std::to_string(TileHierarchy::GetTransitLevel().level));
  std::filesystem::recursive_directory_iterator transit_file_itr(transit_dir);
  std::filesystem::recursive_directory_iterator end_file_itr;
  std::unordered_set<GraphId> all_tiles;
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    auto tile_path = transit_file_itr->path();
    if (std::filesystem::is_regular_file(transit_file_itr->path()) &&
        (tile_path.extension() == ".pbf" || std::isdigit(tile_path.string().back()))) {
      all_tiles.emplace(GraphTile::GetTileId(tile_path.string()));
    }
  }

  auto thread_count =
      pt.get<unsigned int>("mjolnir.concurrency", std::max(static_cast<unsigned int>(1),
                                                           std::thread::hardware_concurrency()));
  LOG_INFO("Building transit network.");

  auto t1 = std::chrono::high_resolution_clock::now();
  if (!all_tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be added.");
    return all_tiles;
  }

  // TODO - intermediate pass to find any connections that cross into different
  // tile than the stop

  // First pass - plan extra platform nodes for platforms with more lines than one node can hold
  const PlatformSplitPlan split_plan =
      PlanPlatformSplits(pt.get<std::string>("mjolnir.transit_dir"), all_tiles, thread_count);
  size_t overflow_count = 0;
  for (const auto& tile_overflow : split_plan.tile_overflow) {
    overflow_count += tile_overflow.second.size();
  }
  LOG_INFO("Platforms split for exceeding " + std::to_string(kMaxEdgesPerNode) +
           " edges: " + std::to_string(split_plan.splits.size()) +
           ", overflow platform nodes: " + std::to_string(overflow_count));

  // Second pass - for all tiles with transit stops get all transit information
  // and populate tiles

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<builder_stats>> results;

  // Start the threads, divvy up the work
  LOG_INFO("Creating " + std::to_string(all_tiles.size()) + " transit graph tiles...");
  size_t floor = all_tiles.size() / threads.size();
  size_t at_ceiling = all_tiles.size() - (threads.size() * floor);
  std::unordered_set<GraphId>::const_iterator tile_start, tile_end = all_tiles.begin();

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    results.emplace_back();
    threads[i] = std::make_shared<std::thread>(build_tiles, std::cref(pt.get_child("mjolnir")),
                                               std::ref(lock), tile_start, tile_end,
                                               std::cref(split_plan), std::ref(results.back()));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes, to see about maximum density (km/km2)
  builder_stats stats{};
  uint32_t total_no_dir_edge_count = 0;
  uint32_t total_dep_count = 0;
  uint32_t total_midnight_dep_count = 0;
  uint32_t total_invalid_service_dates = 0;

  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
      total_no_dir_edge_count += stats.no_dir_edge_count;
      total_dep_count += stats.dep_count;
      total_midnight_dep_count += stats.midnight_dep_count;
      total_invalid_service_dates += stats.invalid_service_dates;
    } catch (std::exception& e) {
      // A tile that could not be built must not leave a partial transit graph behind
      LOG_ERROR(std::string("Building transit tiles failed: ") + e.what());
      throw;
    }
  }

  if (total_invalid_service_dates) {
    LOG_ERROR("There were " + std::to_string(total_invalid_service_dates) +
              " stop pairs with invalid service dates");
  }

  if (total_no_dir_edge_count) {
    LOG_ERROR("There were " + std::to_string(total_no_dir_edge_count) +
              " nodes with no directed edges");
  }

  if (total_dep_count) {
    [[maybe_unused]] float percent =
        static_cast<float>(total_midnight_dep_count) / static_cast<float>(total_dep_count);
    percent *= 100;

    LOG_INFO("There were " + std::to_string(total_dep_count) + " departures and " +
             std::to_string(total_midnight_dep_count) +
             " midnight departures were added: " + std::to_string(percent) + "% increase.");
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  [[maybe_unused]] uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished building transit network - took " + std::to_string(secs) + " secs");

  return all_tiles;
}

} // namespace mjolnir
} // namespace valhalla
