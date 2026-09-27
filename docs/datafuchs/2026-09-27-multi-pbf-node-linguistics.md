# Multi-PBF builds corrupt the node-linguistics file (2026-09-27)

**Status:** root cause found and patched on `fix/multi-pbf-node-linguistics`
(base `fi-3.6.3-patches`). Workaround in production: geofuchs-hydra8 PR #285
merges the country extracts into one PBF before `valhalla_build_tiles`.
**Upstream:** the same code is on `valhalla/valhalla` master (3.9.0); not
reported upstream yet.

## Symptom

The shared Finland + Estonia + Norway candidate build (geofuchs-hydra8
`scripts/valhalla/build_tiles_unified.sh`, image
`valhalla-patched:fi-3.6.3-t1`, `mjolnir.concurrency=32`) ran for two hours
and then aborted in `valhalla_build_tiles`:

```
[ERROR] Failed tile 2/864799/0: vector::_M_range_check: __n (which is 39) >= this->size() (which is 0)
[ERROR] Failed tile 2/866258/0: vector::_M_range_check: __n (which is 9) >= this->size() (which is 0)
... 14 tiles in total ...
terminate called after throwing an instance of 'std::exception'
docker step failed (exit 139)
```

All 14 failing level-2 tiles are in Finland (Helsinki/Uusimaa, Kymenlaakso,
Åland, the Vaasa–Pietarsaari coast). None are in Estonia or Norway.

## How it was isolated

Same script, same image, same settings, fresh work directories, same day:

| Input PBFs | Result |
|---|---|
| `fi` only (live FI pipeline, 4 threads) | pass, 30 min |
| `fi` only (candidate pipeline, 32 threads) | pass, 30 min |
| `fi ee` | pass, 37 min |
| `fi ee no` | crash (above) |

Ruled out on the way: the GTFS calendar fix (a no-op on the HSL/VR feeds
that sit in the failing tiles), thread count, feed clipping, the
"invalid service dates" and "Exceeded maximum transit departure time"
messages (the passing `fi` build logs more of both), and the new internal
flight/ferry GTFS feeds (not wired into the build).

## Root cause

`src/mjolnir/pbfgraphparser.cc`, node pass (`ParseNodes`):

```cpp
for (auto& file : input_files) {
  parser.reset(nullptr, new sequence<OSMWayNode>(way_nodes_file, false), nullptr, nullptr, nullptr,
               nullptr, new sequence<OSMNodeLinguistic>(linguistic_node_file, true));  // true = truncate
```

1. The node-linguistics file is **re-created (truncated) once per input
   file**.
2. `osmdata_.node_linguistic_count`, which assigns each node its
   `linguistic_info_index`, is **not** reset, so it keeps counting across
   files. `reset()` also pushes a placeholder record at "index 0" for every
   file.
3. `midgard::sequence` buffers up to 32 MiB of records (466,033
   `OSMNodeLinguistic`s) and writes them out when the buffer is full or the
   object is destroyed. `reset()` constructs the next file's sequence (which
   truncates) **before** it destroys the previous one, so the previous
   file's still-buffered tail survives: it is written after the truncation.
   Everything the previous files had **already flushed** is lost.
4. After the pass, nodes from earlier files hold indices that point either
   **past the end of the file** or at **another node's record**.

Small inputs therefore come through intact by accident (nothing had been
flushed before the truncation). The bug needs an earlier input file with
more than 466,033 nodes with linguistics. Finland, parsed first, is
evidently above that.

In `BuildTileSet` (`graphbuilder.cc`), a named intersection reads
`linguistic_node.at(node.linguistic_info_index())`. `midgard::sequence`'s
iterator falls back to `write_buffer.at(index - memmap.size())` for
out-of-range indices, and the write buffer is empty when reading, so the
call throws exactly `vector::_M_range_check: __n (which is index − file
size) >= this->size() (which is 0)`. The thread records the exception, the
tile fails, and the build terminates.

Why `fi ee` "passes": the Finnish indices that survive happen to land
inside the shorter file, so nothing throws, but Finnish junction names
silently get another node's language and pronunciation data. That build is
**wrong without failing**. In `fi ee no` some indices point past the end
of the file, and the build fails.

Single-file builds are unaffected, which is why the live FI and EE builds
never showed it.

## Fix (this branch)

- Create the file for the **first** input file only and open it in append
  mode for the others. This is the same `create` idiom the bike-share pass
  directly above already uses.
- Push the index-0 placeholder only while the file is empty, so every
  `linguistic_info_index` equals the record's position.
- Release (and thereby flush) the previous file's sequences **before**
  opening new ones. `reset()`'s arguments are constructed before `reset()`
  destroys the old objects, so without this the reopened sequence's
  `size()` misses the old object's buffered records, and the "only while
  empty" placeholder check above misfires for small files.
- The bike-share pass (`bss_nodes_file`) gets the same release-first
  ordering. It already appended correctly; this is for consistency.

## Tests

`test/gurka/test_multi_pbf_linguistics.cc`:

- `MultiPbfNodeLinguisticsParser` parses two PBFs, the first with more
  than 466,033 named junctions, and checks that the linguistics file holds
  exactly one placeholder plus one record per node, and that every node's
  index points at its own record. Against the old parser it fails: 476,032
  records counted, 9,999 in the file.
- `MultiPbfNodeLinguistics` builds tiles end to end from two small PBFs and
  checks each named junction's pronunciation. This one passes on the old
  code too (see above). It guards the tile path.

Both pass with the fix, and so do the existing `graphparser` and
`gurka_phonemes` suites.

## Verification plan

- [ ] Image built from this branch (`scripts/valhalla/patch_and_build_image.sh`
      in geofuchs-hydra8, its hard-coded `FORK_BRANCH` pointed at
      `fix/multi-pbf-node-linguistics`, and a new image tag, never
      `fi-3.6.3-t1`).
- [ ] `fi ee no` built from the three **separate** PBFs with the new image:
      no failed tiles, and the build's country gate passes.
- [x] The `Number of nodes with linguistics = N` log line equals the record
      count of the linguistics file after the node pass (covered by
      `MultiPbfNodeLinguisticsParser`).
- [ ] Spot check: a Finnish named junction returns Finnish/Swedish names,
      not Estonian ones, in `/route` maneuvers.
- [ ] Offer upstream (issue + PR against `valhalla/valhalla` master).

Until all of that is done, production multi-country builds use the
single-merged-PBF workaround (geofuchs-hydra8 #285), which avoids the bug
entirely.
