# Junction visit state and keyframe policy

Implement in the existing graph and UTM JSON. The user's more specific Endpoint rule wins: only Junction has CANDIDATE/CONFIRMED; Endpoint keeps one remotely observed image. No second graph, navigation changes or new confirmation through camera visibility.

- [x] Add Junction visit evidence, 0.5m robot-odometry confirmation, inheritance through graph rebuild/async completion, ROS fields and light/dark-green markers. Verify radius, state inheritance and no camera confirmation.
- [x] Adapt UTM JSON and image selection: confirmed Junction plus capture-time robot proximity; >60deg diversity, closer replacement within one view, maximum four. Endpoint chooses one most centred view, distance tie-break. Preserve original Frontier policy.
- [x] Validate persistence/replacement/cleanup and image timestamp alignment with small fixtures; build all affected ROS packages and run existing checks. Document parameters, migration and restart instructions.

Validation: 5 affected ROS packages built; 7 CTests passed (manager callback test extended and re-run successfully); 21 store + 4 view-policy + 6 ROS/recorder tests passed. Synthetic checks only, no Habitat episode or live-domain process changes.

Implementation: skeleton_graph.{h,cpp}, skeleton_graph_manager.{h,cpp}, SkeletonNode.msg, existing YAML; pure Python keyframe selection helper shared by recorder/store; existing bounded worker and atomic JPEG replacement/GC retained. State transfer carries the actual visit position, so a relocated mean outside that radius is not falsely confirmed. Fresh odometry required; asynchronous snapshots cannot undo a newer visit. Existing node lifecycle and pruning remain unchanged.
