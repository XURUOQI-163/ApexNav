# Skeleton node-centre links and Junction merging

## Requested design / scope

User explicitly requests Frontier lines ending on the existing Frontier node (centroid), plus merging green Junctions closer than 2m at their midpoint. Implement in current APEXNAV Skeleton observer only; no planner/control/semantic changes. Existing dirty files and user manual must be preserved; no commit requested.

Two points use their midpoint; multi-point merges use the arithmetic mean of original member positions. Merge the closest pair of current clusters repeatedly while distance < configurable `junction_merge_distance` (default ROS 2.0m, 0 disables), deterministic ties. Endpoints and Loop Anchors are excluded. Retain original geometric anchor pixels in each logical node, contract the internal spanning forest between merged Junctions, retaining non-tree edges as cycles, remap external edges and neighbors. Preserve existing true self-loops and distinct external parallel paths. This is spatial abstraction, not proof that a centroid or an internal traversal is collision-free.

World-space logical centres may be off-grid or blocked. Keep original actual pixel/world polylines in `path`, and add explicit `display_path` including the logical node centres. RViz uses display_path. Frontier lines finish exactly at original Frontier centroid, labels move there, no duplicate approach sphere. Preserve safe approach/path, validate only that path, never feed display segments to controller. Merged Junctions offer their original free anchor pixels to Dijkstra; no snapping the requested midpoint to a different location.

## Implementation / verification

- [x] Add failing tests: Frontier display endpoint equals centroid; pair midpoint, >2m separation, strict threshold, multiple member mean, off-grid/blocked centre, rewiring and stable IDs.
- [x] Implement core merge and anchor-aware Frontier associations; retain geometry data and add display polylines/messages.
- [x] Update manager, parameter, RViz marker construction and integration assertions.
- [x] Build plan_env and exploration_manager; Only small core logic checks; user explicitly asked to leave actual simulation/effect checking to them. Do not run ROS integration or full regression experiments. Check live planner before replacing mapped libraries.
- [x] Document precise geometry/display field semantics, threshold and startup; report verified results and limitations.

User steering: no complex validation experiments; finish code, basic build/logic checks only, user will evaluate visual effect. Core regressions initially failed for unmerged nodes and approach-only display endpoint, then passed after implementation.

Completed: normal install build of plan_env and exploration_manager passed (exit0); small geometry/frontier core checks passed; git diff --check passed. Updated ROS integration assertions are provided but deliberately not run this round, following user instructions.
