# Skeleton deadend filtering and dynamic Frontier links

User authorizes diagnosis of excessive junctions, removal of degree-one deadend nodes, and connections from current Frontiers to their nearest green Junction. Work in existing dirty checkout; no navigation/semantic changes, no old TOPOLOGY work, no commits.

## Evidence / decisions

Live read-only domain31 snapshot: 89 nodes (40 Endpoint, 49 Junction), 97 edges, 6 self-edges, 18 Junction-Junction/self edges shorter than 0.35m, 32 protected Unknown-facing Endpoint. The graph matches live data, not leftover RViz markers. Two distinct causes confirmed in sampled cells: tiny Unknown holes create genuine raster loops (e.g. IDs139/136,132/137); a completely FREE 2x2 residual skeleton block creates a false 0.20m self-cycle at J53. Adjacency-only Junction clustering separates junctions connected by short Connector chains.

- Correct only filled-2x2 thinning residue using a sparse GF(2) path basis seeded with filled pixel-face boundaries (no pixel deletion), retaining loops around real Unknown/occupied holes. Do not fill Unknown or merge arbitrary nearby nodes through walls.
- Remove formal Endpoint with one unique neighbor and its incident edge, once per complete extraction. Keep Junctions even if removal leaves them degree 0/1: recursively stripping would erase tree-shaped environments and remove the Frontier anchors. Pure loops stay.
- Copy existing active/dormant Frontier IDs, centroid and member cells directly from FrontierMap2D in the same serialized map callback group. No new semantic state and no mutations to Frontier.
- Nearest = shortest traversable grid path from any valid Junction, using one multi-source Dijkstra over FREE/noninflated cells, no diagonal corner cutting. Current Frontier cells are UNKNOWN; end the safe polyline on their four-neighbor FREE side, retaining original centroid and Frontier ID as association fields. Never draw an unsafe straight segment through Unknown/obstacles to a nonconvex centroid.
- No Junction/reachable approach -> explicit disconnected reason, no invented connection. Frontiers remain handled by original planner.
- Update links when Frontier geometry changes even if map hashes do not, and on any map-tile change independently of the geometry threshold; drop stale/vanished Frontier links before publishing asynchronous results; revalidate paths against current live getters. Keep one background job and reset ownership.

## Tasks

- [x] Add failing regressions for filled-2x2 X-junction and endpoint filtering while preserving junction anchors / real loops.
- [x] Add independent Frontier-connection core and tests: nearest reachable junction, wall detour, no corner cutting, Unknown/inflation exclusion, vanished/no-junction/unreachable frontiers, full path length and IDs.
- [x] Integrate manager sampling, change detection, stale-result filtering, typed message and cyan connection markers; configure and document.
- [x] Build in separate build/install directories while user's old planner is still running (avoid overwriting mapped shared libraries); test isolated domain94; apply final build artifacts safely to normal install if possible, or provide explicit overlay start commands.
- [x] Review, report diagnosis with live measurements and final tests / limitations.

The existing container later exited (exit137, OOMKilled=false), interrupting staged build. Restarted the existing container; no planner was left running. Final build targets normal build/install, so users need no extra overlay.

Final verification: canonical plan_env/exploration_manager build passed (2 packages). plan_env CTest 6/6, Python 50/50, actual ROS Skeleton/Frontier integration in domain94 passed (N1/E0, 3 deadends removed, 1/1 Frontier links; reset and next-scene isolation). Continuous and discrete original semantic alignment/retention/reset regressions passed. git diff --check passed. Reviewer findings for adjacent filled faces and map-only small doorway changes were reproduced and fixed with regressions. No long Habitat exploration or robot test.
