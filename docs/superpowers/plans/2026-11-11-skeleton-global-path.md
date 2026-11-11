# Skeleton Global Path Implementation Plan

Goal: preserve ApexNav target scoring/selection and add opt-in skeleton Dijkstra with grid fallback.
Architecture: a ROS-independent graph search consumes actual skeleton polylines and the current safe Frontier attachment; start connects to visible nearby nodes/edge interiors. Live SDF occupancy validates all used geometry. A synchronous manager accessor stays in the mutually-exclusive map callback group. FSM replaces only the selected route and continues existing local-target/KinoAstar/GCopter/MPC execution. No disk graph, new controller, or old TOPOLOGY mode.

- [x] Core search and fixtures: edge length cost, reversed/parallel edges, attachment tail, edge-interior start, blocked/unknown edges and unreachable inputs.
- [x] Manager and FSM integration: preserve selected grid route for failure, current Frontier signature validation, publish selected route through existing RViz path.
- [x] Launch/config: global_path GRID/SKELETON, enable skeleton observer automatically, configurable start attachment radius.
- [x] Build plan_env/exploration_manager; run core and existing CTests; document usage and local trajectory limits.
