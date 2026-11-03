# Skeleton Graph implementation plan

> Execution: implement in the user's existing ApexNav-ROS2 checkout; preserve all prior changes. User supplied the detailed design and authorized implementation, build and verification. No unrelated refactor, git reset, commits or downloads.

**Goal:** independent, long-lived geometric skeleton of the accumulated SDFMap2D free space; observational only, no navigation/semantic changes.

**Architecture:** MapROS owns SkeletonGraphManager. A low-frequency timer reads SDFMap2D getters into a temporary classification image, and compares tile fingerprints against the last accepted snapshot. One background task crops the free-space bounding box, thins, traces/compresses the skeleton and matches IDs. Only graph geometry and small fingerprints persist; no second global occupancy map. Results/markers are published on the ROS callback thread. Reset follows reconstruction of the original map; finish retains the graph.

**Tech stack:** C++17, existing OpenCV 4.6 ximgproc thinning (present in Jazzy container), optional built-in Zhang-Suen fallback, existing ROS2 messages/visualization stack.

**Spec:** user request in this conversation, sections I–X.

## Decisions / boundaries

- Source enum is UNKNOWN=0 / FREE=1 / OCCUPIED=2 (an old getter comment is misleading). Read getOccupancy/getInflateOccupancy only. Traversable = FREE and not inflated.
- Pixel x is column, y is row; convert to cell centers with SDFMap2D origin/resolution, no vertical image flip. Frame is map_ros/frame_id. SDFMap2D has axis-aligned origin (yaw zero); pure core metadata supports yaw for testing and reuse.
- Exclude diagonal skeleton connections when an orthogonal skeleton route connects the same pixels. Cluster adjacent degree>=3 pixels; choose an actual representative skeleton pixel, route within the cluster rather than drawing shortcuts.
- Pure degree-2 components become one LOOP_ANCHOR and a closed self-edge. Keep parallel edges and full path geometry.
- Clearance = conservative distance to non-traversable boundary, after inflation, including unknown/map exterior. It is not the robot-center distance to raw obstacle centers.
- Prune short leaf-to-junction branches by actual metric length, iterating after each pruning pass. Preserve endpoint-to-endpoint corridors, loops and (by default) unknown-boundary endpoints. No width-based deletion.
- Match same-type nodes greedily by nearest world distance, one-to-one. IDs monotonic within a simulation. Current graph nodes active; unmatched nodes disappear only through changed geometry/pruning. No frontier callbacks/dependencies.
- Long-term means retained across sensing/view changes during the same map episode; no disk persistence requested here.
- Use existing dirty checkout to integrate with user's uncommitted reproduction; tests ROS_DOMAIN_ID=94. Do not touch user manual.

## Tasks

- [x] 1. Write executable C++ tests for line/T/cross/branched indoor/noisy/unknown/closed ring, corner connectivity, metadata, clearance, stable IDs and change gating. Observe failure before core implementation.
- [x] 2. Implement pure extraction/graph structures with ximgproc and fallback, pruning, matching and fingerprints; pass synthetic tests with both backends.
- [x] 3. Implement manager and typed ROS graph/marker publication; integrate read-only into MapROS; configuration and launch toggle (APEXNAV only), RViz entries.
- [x] 4. Build affected ROS packages; test real SDFMap2D read-only integration/reset/throttling, existing semantic regressions and package tests. Review lifecycle/threading/performance.
- [x] 5. Document files, call graph, parameters, compilation/start commands and RViz checks; record actual test counts and limitations.

## Review focus

No callbacks may access SDFMap2D in the worker thread; pending jobs must not outlive the manager or publish stale graphs after reset. No unknown/inflated path pixels. Full skeleton paths must remain connected through merged junctions. Tiny isolated pixels, image borders, self-loops and parallel edges must not be lost or multiply traced. Threshold baseline must accumulate small map changes rather than forgetting them on each timer tick.

## Progress

- Initial core tests failed because the feature did not exist; implemented and passed with ximgproc and built-in thinning.
- Reviewer reproduced lost internal junction cycle; added failing regression, preserved spanning-tree/non-tree cycles, passed independently reviewed core test.
- Actual ROS launch/SDFMap2D/Marker test passed N4/E3, original Frontier and velocity publisher unchanged, live inflation excluded, finish/reset lifecycle correct.
- Final validation: plan_env CTest 5/5, previous topology_exploration CTest 7/7, original Python suite 50/50; APEXNAV continuous and discrete semantic alignment/finish/reset regressions both passed. YAML, launch syntax and git diff --check passed. Seven dependency packages built; final core-cycle fix rebuilt plan_env successfully. No model/resource downloads or git commits.
