# Persistent topology snapshot

User-requested layout: project `topology_snapshot/graph.json` plus `images/node_<id>/`.
Keep the last snapshot across normal exit, Ctrl+C and planner restart. Clear only
at the next TOPOLOGY simulation's existing reset handshake, before sensor frames.

Implementation uses a small C++ writer, direct graph copies and shared JPEG
buffers, with at most one background disk job. Every second publish an atomic
JSON replacement after its referenced images exist. Per the user’s follow-up: reset all in-memory and on-disk state only at simulation
startup. Remove exit resets in both runners and make EPISODE_FINISH stop the old
FSMs without clearing visualization/map. The existing /exploration/reset starts
a new snapshot; no separate disk-reset service. Flush on stop/shutdown. Merely
restarting the planner does not delete disk; the next simulation handshake does.
No new model/environment dependency, no global-grid export or navigation change.

The snapshot is the latest graph, not a frame-by-frame history. Include node IDs,
types, source IDs, actual descriptor origins, 12 coarse and 72 fine observations,
directional image timestamp/pose/slot/path, all edges including invalid edges and
their polylines/costs. Nonfinite unobserved distances use JSON null. Capture time
and state are included. JPEG width/quality remain the existing 320/75 settings.

- [x] Test writer lifecycle, complete JSON, image references, stale-image cleanup and I/O failure preservation.
- [x] Implement background saving through the existing simulation-start reset; add isolated test directories.
- [x] Build, test normal/interrupt exit and second-run clearing using ROS domain 94; check original modes.
- [x] Review and document path, schema, cadence and forced-kill limits.

Tests must never clear the user's project snapshot; set a temporary
`APEXNAV_TOPOLOGY_SNAPSHOT_DIR` ending in `topology_snapshot` for spawned planners.
