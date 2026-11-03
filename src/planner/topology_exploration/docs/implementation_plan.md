# Topology closed-loop exploration implementation plan

> Implement inline with executing-plans; validate each unit before integration.

Goal: selectable geometric topology exploration with bounded local memory and fail-closed MPC execution.
Architecture: shared SDFMap2D without semantic/ROS attachments, rolling adapter, shared sparse graph plus verified-cost A*, local grid A*, existing MPC library and a single ROS executor.
Tech stack: ROS2 Jazzy, C++17, Eigen/OpenCV/PCL, OsqpEigen, existing Python Habitat runner.
Spec: design.md in this directory.

Constraints: keep APEXNAV default, no new semantic/model dependency, no hardware execution, fixed 6m window, one cmd_vel publisher, original dirty user manual untouched. Work in current user-selected project to include previous uncommitted changes; no commits/reset/stash.
Review focus: remote edges surviving map roll; unknown gaps never becoming motion; initial and final control increment bounds; stale/queued messages after reset; detour costs and failed goal reselection.

- [x] Task 1: add local-only SDF attachment option, RollingLocalMap and shared depth projection. Tests assert fixed capacity, overlap shift, outside Unknown, footprint intersection and known-free traversal. Compile/test in existing Jazzy container.
- [x] Task 2: adapt shared TopologyGraph with optional local scope and verified edge records; GraphPlanner weighted A*, local connection and failure policy. Tests exercise cheaper detour vs Euclidean-nearest goal, blocked/unverified edges, distant graph survival and cooldown.
- [x] Task 3: LocalPlanner A* safe prefix, ControllerGuard and shared MPC strict mode/library. Tests exercise obstacle gap, no corner cutting, finite bounded commands, first-step acceleration, solver failure and watchdog stop.
- [x] Task 4: independent ROS node/launch/config/visualization, exact depth+pose, graph update, observation and tracking state machine, stop/reset contract. Isolated synthetic continuous integration tests.
- [x] Task 5: planner selection in launch/Habitat CLI, skip vision in geometric mode, docs and final regression/build/review.

Execution ledger: this file. No implementation agents; one independent final review per skill. Record rulings and tests here as work proceeds.

Rulings after builder check: inspect every historical edge segment intersecting the rolling window; known obstacles revoke verification, unknown/outside do not erase historic evidence and never authorize local motion. Guard checks the final rate-limited v,w and its braking sweep. Prefix target follows ordered edge-polyline progress. New collision-height band starts at 0.05m (ground rejection), upper bound configurable to match the Habitat collision body; do not inherit 0.28m lower bound. Use strict yaw unwrapping in new MPC mode. Defaults are implementation choices within the supplied scope.


Completed implementation and review:
- Core adapter and graph tests: fixed 14,400-cell capacity, overlap retention, discarded history, immediate fresh hits on saturated-free cells; verified weighted A*, actual 3.1m detour retention, remote edge survival and interior obstacle invalidation.
- Full panoramic geometry regression advances eight frontier steps to x=16.42m with nine waypoints. New coverage neighborhood avoids losing all local candidates inside an older 5m descriptor. Legacy point coverage defaults remain unchanged.
- Ordered local path execution now visits the pending polyline vertex before selecting later segments, preventing shortcut-induced cursor reversal. Controlled waypoint spacing avoids halfway anchors before frontier arrival.
- Existing MPC exported as a shared library. Strict mode checks first-command increments, pi-crossing yaw, bad references and solver rejection; final guard checks limited command and braking arc. Strict=false/delay overrides rejected in TOPOLOGY.
- Independent reviewer found fresh-obstacle latency, coverage exhaustion and route-cursor reversal. All three fixed and regression tested; reviewer reran native graph/local/legacy topology fixtures and reported no remaining blocker.
- 50 existing Python unit tests + 3 new CLI tests passed. Four new CTests + two old plan_env CTests passed. Controlled Habitat runner tests confirm custom-target legacy behavior and geometric execution without visual process, semantic labels/triggers or forced startup rotation.
- Isolated ROS domain94 closed loop: cumulative travel 4.00m, two FRONTIER-to-WAYPOINT conversions, W3/F10/E15 in the final synchronized-pose run, bounded 14,400 cells; depth/odometry timeouts, invalid depth, fresh obstacle and reset checks passed.
- Existing ROS observer regression: W1/F6/E6 steady, zero actual duplicates; arrival W2/F5/E6, exact images and reset passed. Continuous semantic regression: delayed scores update their original frontiers; reset discards old scores and map/markers.
- Five affected packages built in existing Jazzy container. RViz configs include rolling map and both paths. No model download or extra environment created; no HM3D performance benchmark or hardware execution.

Final small alignment correction: graph anchors use the image's synchronized camera x/y (Habitat camera has zero horizontal base offset), while control continues using newest odometry; final closed-loop re-verification is recorded in validation.md.
