# Furnished-scene frontier recovery implementation plan

> Execute inline with superpowers:executing-plans; the user approved this correction.

**Goal:** Episode 550 must leave initial rotation and explore verified reachable frontiers.

**Architecture:** Keep conservative 30-degree descriptor sectors, additionally retain their six angular samples for candidate selection. TOPOLOGY uses an independent open-range threshold and existing local A* to validate detour edges. Legacy APEXNAV observation keeps its defaults. No global grid or semantic additions.

**Tech stack:** ROS2 Jazzy, C++17, existing LocalPlanner/MPC and Habitat.

**Spec:** User confirmation of the diagnosis: separate range threshold, fine angular candidates, verified detours, rejection diagnostics and real episode validation.

## Constraints and review focus

- Never interpret invalid/unobserved depth as free; do not splice displaced observations.
- Verify whole paths including footprint clearance; endpoint validity alone is insufficient.
- Store polyline length, preserve traversed history, invalidate edges on newly seen obstacles.
- Reset diagnostic counters and graph at episode boundaries; no duplicate candidate growth.
- Test in ROS domain 94 only; leave user processes and unrelated edits intact.

## Execution

- [x] Add a regression fixture for a partially obstructed coarse sector and a reachable detour rejected by the old straight-ray check; observe failure.
- [x] Extend `depth_descriptor.h` with six direction samples per sector; expose fine candidates behind a default-off config in `topology_graph.{h,cpp}`. Add a route-validation callback, path verification and generation counters. Re-run fixture and legacy sparse graph tests.
- [x] Wire LocalPlanner and YAML parameters in `explorer_node.cpp`; expose state transitions and diagnostics in `visualization.cpp` and `/topology/state`.
- [x] Build affected packages and run CTests, synthetic ROS safety regressions and actual HM3D episode 550. Require translation, reachable frontier generation and conversion; document measured counts and remaining limits.
- [x] Review and update package documentation, without committing the user's working tree.
