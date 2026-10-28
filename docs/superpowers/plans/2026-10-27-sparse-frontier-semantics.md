# Sparse current-frontier semantic memory

The requested replacement stores semantic evidence only on current frontier clusters. No historical graph, start node, edges, three-metre sampling, image archive, or dense semantic grid. Geometry/occupancy remains a grid for collision checking. Existing CLIP and BLIP2 services and stamped scores are unchanged.

- [x] Replace ValueMap's two dense buffers with frontier nodes, preserving the original view confidence and weighted update equations. Validate angular limits, repeated updates, disappearing nodes and delayed samples with small C++ fixtures.
- [x] Give geometrically continuous frontiers stable IDs using overlapping boundary cells. Include active and dormant boundary clusters; discard clusters removed from frontier detection. Refresh clusters after each depth frame before capturing its semantic projections.
- [x] Cache only original-frame frontier IDs/positions/weights with original camera pose, not semantic free-grid lists. Late inference updates surviving IDs only. Retain the existing bounded timestamp join and compact cloud needed for object evidence.
- [x] Read node scores directly in both semantic exploration policies; publish only current nodes/labels in existing RViz configurations. Verify compilation, unit suite and synthetic ROS frame alignment/lifecycle checks; leave Habitat experiments to the user.

Fusion is unchanged: V'=(w*S+C*V)/(w+C), C'=(w*w+C*C)/(w+C). Angular weight is cos²(relative_yaw/(79°/2)*π/2), clamped to zero outside the camera FOV. Visibility also requires original-frame nearby ray-traced free cells and unoccluded geometric line of sight. Camera yaw uses the optical forward axis projected to XY.

Cluster rebuilding matches one-to-one by shared boundary cells. Unmatched clusters get new monotonic IDs and start unscored; splitting/merging preserves the best surviving overlap only. A vanished node is deleted, even if old inference is in flight. Original observation coordinates and timestamps remain with scores so drift cannot silently reproject evidence using a newer pose.

Verification: 32 unit tests and six-package colcon build passed. The sparse ROS probe verified delayed BLIP2 A evidence at A frontiers after moving to B, B evidence at B frontiers, duplicate rejection and sparse cloud values. The probe now confirms acquisition of its sensor fixtures because BEST_EFFORT transport may drop a depth/pose pair; missing production observations remain correctly discarded. Independent read-only review identified split-fragment ordering stealing an old ID; a failing fixture and global strongest-overlap-first matching fixed that finding. No Habitat benchmark, downloads or changes to user notes.
