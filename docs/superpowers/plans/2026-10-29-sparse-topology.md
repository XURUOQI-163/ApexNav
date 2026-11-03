# Sparse topology observer plan

User-authorized scope: add independent FRONTIER/WAYPOINT geometric topology, retain existing grid exploration and control. No Room/Portal implementation in this phase.
Reference: https://arxiv.org/html/2605.07275v1 sections IV-B/C. Reimplementation for forward depth, not authors' released code.

- [x] Pure C++ DepthDescriptor: world-aligned configurable sectors; per-sector state, nearest obstacle, free range, timestamp, partial angular support, observation origin. Unknown/invalid never implies free. Rotate at waypoint to accumulate views; translated measurements retain their actual origins.
- [x] TopologyGraph: current-frame candidates from open intervals and jumps; retract into known traversable cells, corridor checks, historical descriptor coverage and distance deduplication; real arrival conversion/revisit; invalid-node removal; visibility edges and IDs/weights.
- [x] MapROS observer: reuse existing per-pixel projection; original float validity retained; no PointCloud2 subscription. Exact-stamp RGB association, bounded directional JPEG views, default off, config+RViz markers+stats, reset with map.
- [x] Tests: deterministic forward-only, invalid/max-depth/partial sectors, rotations, translation, jumps, visits/duplicates, obstacle changes, edges/images/reset; real ROS synchronized synthetic streams, default-off regression. Build Jazzy and report observed counts.

Rulings: graph is an observer; arriving within reach radius converts any nearby frontier because graph does not select motion goals. Additional sparse waypoints record legacy trajectory when it doesn't visit graph candidates. FRONTIER descriptor is explicitly source-referenced until visited. Images are bounded per node by directional slots, with pose/stamp, no disk writes by default. Tests use isolated domain 94.

Verification: final Jazzy colcon build succeeded for plan_env, path_searching, trajectory_manager, exploration_manager. CTest 2/2 passed; Python unit tests 50/50 passed. Real ROS domain-94 fixture through topology:=sparse: steady W=1/F=6/E=6, actual duplicates=0; arrival W=2/F=5/E=6, same frontier ID converted; RGB exact stamps and green/red/blue markers verified; reset clears graph and rejects old input. Legacy semantic integration result is recorded in docs/sparse_topology.md.
