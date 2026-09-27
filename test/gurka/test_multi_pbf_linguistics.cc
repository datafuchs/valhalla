#include "baldr/graphreader.h"
#include "gurka.h"
#include "midgard/sequence.h"
#include "mjolnir/osmnodelinguistic.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/util.h"
#include "test.h"

#include <gtest/gtest.h>
#include <osmium/builder/attr.hpp>
#include <osmium/io/pbf_output.hpp>

#include <filesystem>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

namespace {

gurka::nodes::mapped_type named_junction(const std::string& name, const std::string& osm_id = "") {
  gurka::nodes::mapped_type tags = {{"junction", "named"},
                                    {"highway", "motorway_junction"},
                                    {"name", name},
                                    {"name:pronunciation", name + ":pronunciation"}};
  if (!osm_id.empty())
    tags["osm_id"] = osm_id;
  return tags;
}

// Returns the pronunciation of the named junction at the given node, or "" if it has none.
std::string junction_pronunciation(GraphReader& reader,
                                   const gurka::nodelayout& layout,
                                   const std::string& node_name) {
  const GraphId node_id = gurka::findNode(reader, layout, node_name);
  auto tile = reader.GetGraphTile(node_id);
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;
  const std::vector<SignInfo> signs = tile->GetSigns(node_id.id(), index_linguistic_map, true);
  EXPECT_EQ(signs.size(), 1) << node_name;
  const auto found = index_linguistic_map.find(0);
  if (found == index_linguistic_map.end())
    return "";
  return std::get<kLinguisticMapTuplePronunciationIndex>(found->second);
}

// Writes a PBF with `way_count` ways of `nodes_per_way` nodes each. Every node is a named junction
// whose pronunciation encodes its OSM id ("p<id>"). Ids start after `first_id`.
void write_junction_pbf(const std::string& filename,
                        uint64_t first_id,
                        uint32_t way_count,
                        uint32_t nodes_per_way) {
  using namespace osmium::builder::attr;
  osmium::memory::Buffer buffer{1024 * 1024, osmium::memory::Buffer::auto_grow::yes};
  const uint64_t node_count = uint64_t(way_count) * nodes_per_way;
  for (uint64_t i = 0; i < node_count; ++i) {
    const uint64_t id = first_id + 1 + i;
    const osmium::Location location{5.1 + (i % 1000) * 1e-5, 52.0 + (i / 1000) * 1e-5};
    osmium::builder::add_node(buffer, _id(id), _version(1), _location(location),
                              _tag("highway", "motorway_junction"), _tag("junction", "named"),
                              _tag("name", "n" + std::to_string(id)),
                              _tag("name:pronunciation", "p" + std::to_string(id)));
  }
  for (uint32_t w = 0; w < way_count; ++w) {
    std::vector<osmium::object_id_type> refs;
    for (uint32_t n = 0; n < nodes_per_way; ++n)
      refs.push_back(first_id + 1 + uint64_t(w) * nodes_per_way + n);
    osmium::builder::add_way(buffer, _id(first_id + node_count + 1 + w), _version(1), _nodes(refs),
                             _tag("highway", "residential"));
  }
  osmium::io::Header header;
  osmium::io::Writer writer{osmium::io::File{filename, "pbf"}, header, osmium::io::overwrite::allow};
  writer(std::move(buffer));
  writer.close();
}

} // namespace

// Same bug at the parser level, with an earlier file large enough to fill the linguistics
// sequence's write buffer (32 MiB) at least once. Small inputs survived the old code by accident:
// the previous file's sequence only flushed its buffer when it was destroyed, which happened after
// the next file's sequence had already truncated the file. Records flushed earlier were lost.
TEST(Standalone, MultiPbfNodeLinguisticsParser) {
  const std::string workdir = "test/data/gurka_multi_pbf_linguistics_parser";
  if (std::filesystem::exists(workdir))
    std::filesystem::remove_all(workdir);
  std::filesystem::create_directories(workdir);

  // Enough records in the first file to force at least one flush of the write buffer.
  const uint32_t buffer_records = 1024 * 1024 * 32 / sizeof(mjolnir::OSMNodeLinguistic);
  const uint32_t nodes_per_way = 1000;
  const uint32_t first_ways = buffer_records / nodes_per_way + 10;
  const std::string first_pbf = workdir + "/first.pbf";
  const std::string second_pbf = workdir + "/second.pbf";
  write_junction_pbf(first_pbf, 0, first_ways, nodes_per_way);
  write_junction_pbf(second_pbf, 100000000, 3, 10);

  const auto config = test::make_config(workdir, {{"mjolnir.tile_dir", workdir}});
  const std::vector<std::string> files = {first_pbf, second_pbf};
  const std::string ways_file = workdir + "/ways.bin", way_nodes_file = workdir + "/way_nodes.bin",
                    access_file = workdir + "/access.bin", from_file = workdir + "/from.bin",
                    to_file = workdir + "/to.bin", bss_file = workdir + "/bss.bin",
                    linguistics_file = workdir + "/linguistics.bin";
  auto osmdata = PBFGraphParser::ParseWays(config.get_child("mjolnir"), files, ways_file,
                                           way_nodes_file, access_file);
  PBFGraphParser::ParseRelations(config.get_child("mjolnir"), files, from_file, to_file, osmdata);
  PBFGraphParser::ParseNodes(config.get_child("mjolnir"), files, way_nodes_file, bss_file,
                             linguistics_file, osmdata);

  const uint64_t expected_nodes = uint64_t(first_ways) * nodes_per_way + 3 * 10;
  midgard::sequence<mjolnir::OSMNodeLinguistic> linguistics(linguistics_file, false);
  // One placeholder at index 0 plus one record per node.
  EXPECT_EQ(osmdata.node_linguistic_count, expected_nodes + 1);
  ASSERT_EQ(linguistics.size(), osmdata.node_linguistic_count);

  // Every node's index must point at its own record.
  midgard::sequence<mjolnir::OSMWayNode> way_nodes(way_nodes_file, false);
  uint64_t checked = 0;
  for (const mjolnir::OSMWayNode way_node : way_nodes) {
    const uint32_t index = way_node.node.linguistic_info_index();
    ASSERT_NE(index, 0) << way_node.node.osmid_;
    ASSERT_LT(index, linguistics.size()) << way_node.node.osmid_;
    const mjolnir::OSMNodeLinguistic ling = *linguistics.at(index);
    ASSERT_EQ(osmdata.node_names.name(ling.name_pronunciation_ipa_index()),
              "p" + std::to_string(way_node.node.osmid_));
    ++checked;
  }
  EXPECT_EQ(checked, expected_nodes);
}

// Node linguistics (junction name language/pronunciation) must survive a build from several input
// PBFs. The node pass used to re-create the linguistics file for every input file while the index
// handed to each node kept counting across files, so nodes from earlier files pointed past the end
// of the file (BuildTileSet threw vector::_M_range_check) or at another node's record.
TEST(Standalone, MultiPbfNodeLinguistics) {
  const std::string ascii_map = R"(
      I    J
      |    |
  A---B----C---D

      K
      |
  E---F----G
  )";

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {5.1079374, 52.0887174});

  const std::string workdir = "test/data/gurka_multi_pbf_linguistics";
  if (std::filesystem::exists(workdir))
    std::filesystem::remove_all(workdir);
  std::filesystem::create_directories(workdir);

  // The first file has more nodes with linguistics than the second one, which is what made the
  // earlier files' indices run past the end of the (truncated) linguistics file.
  const std::string first_pbf = workdir + "/first.pbf";
  detail::build_pbf(layout,
                    {{"ABCD", {{"highway", "trunk"}, {"name", "ABCD"}}},
                     {"BI", {{"highway", "trunk"}}},
                     {"CJ", {{"highway", "trunk"}}}},
                    {{"B", named_junction("junction B")}, {"C", named_junction("junction C")}}, {},
                    first_pbf);

  // Explicit, distinct OSM ids: build_pbf numbers every file from 1.
  const std::string second_pbf = workdir + "/second.pbf";
  detail::build_pbf(layout,
                    {{"EFG", {{"highway", "trunk"}, {"name", "EFG"}, {"osm_id", "201"}}},
                     {"FK", {{"highway", "trunk"}, {"osm_id", "202"}}}},
                    {{"E", {{"osm_id", "101"}}},
                     {"F", named_junction("junction F", "102")},
                     {"G", {{"osm_id", "103"}}},
                     {"K", {{"osm_id", "104"}}}},
                    {}, second_pbf);

  // Junction names are only written where the admin area allows intersection names.
  auto config = test::make_config(workdir, {{"mjolnir.concurrency", "1"},
                                            {"mjolnir.tile_dir", workdir + "/tiles"},
                                            {"mjolnir.admin", VALHALLA_SOURCE_DIR
                                             "test/data/netherlands_admin.sqlite"}});
  ASSERT_TRUE(build_tile_set(config, {first_pbf, second_pbf}, BuildStage::kInitialize,
                             BuildStage::kValidate));

  GraphReader reader(config.get_child("mjolnir"));
  EXPECT_EQ(junction_pronunciation(reader, layout, "B"), "junction B:pronunciation");
  EXPECT_EQ(junction_pronunciation(reader, layout, "C"), "junction C:pronunciation");
  EXPECT_EQ(junction_pronunciation(reader, layout, "F"), "junction F:pronunciation");
}
