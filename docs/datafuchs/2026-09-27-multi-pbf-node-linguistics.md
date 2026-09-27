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
3. After the pass the file holds only the **last** file's records. Nodes
   from earlier files keep indices that point either **past the end of the
   file** or at **another node's record**.

In `BuildTileSet` (`graphbuilder.cc`), a named intersection reads
`linguistic_node.at(node.linguistic_info_index())`. `midgard::sequence`'s
iterator falls back to `write_buffer.at(index - memmap.size())` for
out-of-range indices, and the write buffer is empty when reading, so the
call throws exactly `vector::_M_range_check: __n (which is index − file
size) >= this->size() (which is 0)`. The thread records the exception, the
tile fails, and the build terminates.

Why `fi ee` "passes": the Finnish indices happen to land inside Estonia's
records, so nothing throws, but Finnish junction names silently get
Estonian language and pronunciation data. That build is **wrong without
failing**. `fi ee no` fails because Norway, parsed last, has fewer records
than some of the Finnish indices.

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
  destroys the old objects, so without this a reopened append-mode stream
  starts at the old end of file and collides with the old object's final
  flush.

The bike-share pass has the same construct-before-destroy ordering for
`bss_nodes_file`. It's not changed here; we don't use
`import_bike_share_stations`.

## Verification plan

- [ ] Image built from this branch (`scripts/valhalla/patch_and_build_image.sh`
      in geofuchs-hydra8, its hard-coded `FORK_BRANCH` pointed at
      `fix/multi-pbf-node-linguistics`, and a new image tag, never
      `fi-3.6.3-t1`).
- [ ] `fi ee no` built from the three **separate** PBFs with the new image:
      no failed tiles, and the build's country gate passes.
- [ ] The `Number of nodes with linguistics = N` log line equals the record
      count of the linguistics file after the node pass.
- [ ] Spot check: a Finnish named junction returns Finnish/Swedish names,
      not Estonian ones, in `/route` maneuvers.
- [ ] Offer upstream (issue + PR against `valhalla/valhalla` master).

Until all of that is done, production multi-country builds use the
single-merged-PBF workaround (geofuchs-hydra8 #285), which avoids the bug
entirely.
