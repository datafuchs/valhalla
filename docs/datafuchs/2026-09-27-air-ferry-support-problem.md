# Valhalla air travel + ferry support — problem statement

**Date:** 2026-09-27
**Status:** Problem statement only. Full design (options, recommendation,
implementation plan) is deferred to a dedicated session — not in this doc.
**Decision (CEO, 2026-09-27, verbatim):** "no dont drop the routes we must
add support" — air travel and ferries must be added to our Valhalla-based
routing **properly**, not bypassed or silently discarded.

Paths under `src/`, `proto/` and `valhalla/` are in this repository (branch
`fi-3.6.3-patches`); paths under `scripts/`, `k8s/` and `docs/` are in
`datafuchs/geofuchs-hydra8`, which builds the tiles and the image.

## 1. The problem

Our Valhalla fork (`fi-3.6.3-patches`, image `valhalla-patched:fi-3.6.3-t1`)
only models 8 GTFS transit vehicle types — the base `route_type` values
0–7. GTFS's *extended* route types for air travel (1100/1102), aerial lift
(1300), ferry service (1200), and funicular (1400) sit outside that range
and are not modelled as a first-class concept anywhere in the fork.

Two of our own build scripts (`scripts/valhalla/build_tiles.sh:508-529` and
`scripts/valhalla/build_tiles_unified.sh:805-826`, function
`remap_route_types()`) rewrite six GTFS-extended ranges onto base types:

```
700-799 -> 3 (bus)   900-999 -> 0 (tram)   100-199 -> 2 (rail)
200-299 -> 3 (bus)   400-499 -> 1 (metro)  1000-1099 -> 4 (ferry)
```

Everything else — 300-399, 500-699, 800-899, and everything ≥1100 —
is **left unmapped** and reaches Valhalla ingest exactly as the source feed
wrote it. Norway's Entur national feed (`no_entur`, already in
`config/fi_gtfs_feeds.yaml`'s Norwegian counterpart) carries real routes in
that unmapped space today: `route_type=1102` ("Domestic Air Service"), 101
routes / 2,295 trips, all Norwegian airports (concentrated in Finnmark); and
`route_type=1300` ("Aerial Lift"), 2 routes.

Separately, two internal GTFS feeds we built this week are **not wired into
any Valhalla build**: the flights feed (hydra8 #276,
`scripts/valhalla/flights/`, Entur/Avinor NLOD data, FI/EE/NO hub airports)
and the Gulf of Finland ferries feed (hydra8 #277,
`scripts/valhalla/ferries/`, Eckerö scraped timetable + Tallink via
Digitraffic Marine, CC BY 4.0). Both feeds' own READMEs state plainly that
wiring them into a build is "a separate decision" nobody has made yet.

## 2. What is verified (fork source, cited)

- `proto/transit.proto:50-58` defines a closed 8-value `VehicleType` enum:
  `kTram=0, kMetro=1, kRail=2, kBus=3, kFerry=4, kCableCar=5, kGondola=6,
  kFunicular=7`. There is no Air, Aerial-Lift, or Funicular-beyond-7 slot.
- `valhalla/baldr/graphconstants.h:705-714` defines the same closed 0-7
  `TransitType` enum on the read/costing side — the two enums must agree.
- `src/mjolnir/ingest_transit.cc:645-646` converts a GTFS route into a
  Valhalla route with an **unchecked C-style cast**, no switch, no range
  check, no fallback:
  ```cpp
  route->set_vehicle_type(
      (valhalla::mjolnir::Transit_VehicleType)(static_cast<int>(currRoute.route_type)));
  ```
  So today, if an unmapped extended value (e.g. 1102 or 1300) ever reached
  `valhalla_build_transit` unmapped, it would **not error at ingest time**.
  It produces an out-of-range enum value that has no matching case in the
  (also closed 0-7) transit-costing switch either — an undefined,
  unhandled state, not a clean drop. This is confirmed independently by the
  flights feed's own investigation (`scripts/valhalla/flights/README.md:280-308`).
- Neither internal feed is wired anywhere: `grep` across
  `build_tiles.sh`, `build_tiles_unified.sh`, and `k8s/` for `flights/`,
  `ferries/`, `build_flights_gtfs`, `build_gulf_ferries_gtfs` returns
  nothing. No CronJob/Job runs either feed. The flights feed currently
  emits a placeholder `route_type=2` (Rail) rather than the correct 1102,
  specifically because 1102 is Valhalla-incompatible today
  (`flights/README.md:297-308`). The ferries feed emits `route_type=4`
  (Ferry) — a base type Valhalla already models correctly.

## 3. What is not yet known (for the dedicated session)

- Whether `TransitDeparture`/`TransitSchedule` record field widths
  (`valhalla/baldr/transitdeparture.h`) can represent a multi-hour flight
  leg or an overnight ferry leg — not checked.
- The exact downstream failure mode of an out-of-range `TransitType` value
  once it's past ingest (crash, silently-wrong costing, silently-dropped
  edge) — we only know no case exists for it.
- Full scope of a native "air" type: protobuf bump, tile-format
  compatibility for anything still serving old tiles, `sif/transitcost.cc`
  costing factors, request options (`costing_options.transit`),
  response/narrative strings, isochrone and matrix implications.
- Which of the obvious options — (A) native air type in the fork, (B) map
  air onto an existing type plus buffers as an interim, (C) keep flights
  outside Valhalla entirely (the hub-and-spoke precompute model) — is
  right for a v1. No option has been evaluated; this doc takes no position
  beyond "the routes must not be dropped."

## 4. Constraints any option must respect

- Airport and ferry buffers are documented but **enforced nowhere**:
  flights need 75 min before departure / 20 min after arrival
  (`flights/README.md:310-317`); ferries need ~30 min check-in
  (`ferries/README.md:248-257`). Neither is baked into the GTFS times.
- Service limits in both build scripts cap `multimodal.max_distance` at
  250,000 m (250 km) and `isochrone.max_time_contour` at 120 min
  (`build_tiles_unified.sh:1110-1115`, mirrored in `build_tiles.sh`).
  Domestic flight legs (e.g. Oslo–Alta, ~1,600 km) and some ferry
  itineraries already exceed these; both limits need reconsideration once
  such legs become routable.
- Any tile-format or proto change forces a full rebuild across FI, EE, and
  NO candidate tiles, and needs a compatibility story for tile readers.
- `datafuchs/hausfuchs#60` (weekly-commute design, open PR) currently
  states: "Flights in Valhalla: No. Valhalla has no air vehicle type;
  flights stay a separate leg source so airport buffers apply." That
  decision needs explicit revisiting once this work has a direction — it
  is not this doc's job to overrule it, only to flag that the CEO's
  instruction supersedes its premise. The hub model's block↔hub
  access-leg precompute (and its planned R5 replacement) may still be
  useful independent of whatever Valhalla itself learns to route.

## 5. Open questions for the dedicated session

1. Native air type vs. interim mapping vs. keep-outside-Valhalla (hub
   model) — which for v1, and why?
2. If native type: who owns the tile rebuild and back-compat testing
   across FI/EE/NO?
3. What should the new `multimodal.max_distance` and
   `isochrone.max_time_contour` values be once air/ferry legs are
   routable?
4. Who wires hydra8 #276/#277 into `build_tiles_unified.sh`, and on what
   cadence (weekly, per-rebuild)?
5. Where do the buffer constants (75/20 min flights, 30 min ferries) live
   as the "named constants in one module" hausfuchs#60 calls for — shared
   by the hub model and any native-Valhalla path?
6. Does hausfuchs#60's "Flights in Valhalla: No" line get revised now,
   superseded later, or held as an interim truth while fork work proceeds?
7. Is it worth getting the Digitransit key into the cluster to unlock the
   30 Finnish domestic PSO routes in the flights feed?
8. Viking Line and Helsinki–Mariehamn are not covered by the ferries feed
   today — accept the gap, or pursue a data source?

## 6. Links

- datafuchs/geofuchs-hydra8#276 — flights GTFS feed (`scripts/valhalla/flights/`)
- datafuchs/geofuchs-hydra8#277 — Gulf of Finland ferries GTFS feed (`scripts/valhalla/ferries/`)
- datafuchs/geofuchs-hydra8#286 — first copy of this statement (superseded by this fork PR)
- datafuchs/geofuchs-hydra8#285 — build fix: one merged PBF for multi-country builds
- datafuchs/valhalla#1 — multi-PBF node-linguistics fix (unrelated bug, same build)
- datafuchs/hausfuchs#60 — weekly-commute design (open), "Flights in
  Valhalla: No" decision under review
