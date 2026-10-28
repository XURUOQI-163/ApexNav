# Frontier cell semantics implementation plan

> Use superpowers:executing-plans for inline execution.

Goal: fuse semantics per current frontier grid cell, reduce each cluster by scored-cell maximum, use a 20-frame original RGB/scan/pose window, disable 0.65 m subdivision.
Spec: user's three requirements in this conversation, 2026-10-27.

Architecture: C++ owns sparse current cell lifetimes and emits timestamped FrontierScan (original ray-observed free cells, camera pose and current frontier cell tokens). An independent Python ROS collector matches RGB to FrontierScan exactly and stores 20 frames. One inference worker processes matching frames chronologically, caches each image score, projects by original angle/distance and publishes cell-token evidence in SemanticScore. Live window is 20; a running batch may hold up to 20 older frame references until it finishes. No unbounded inference queue. C++ rejects removed cell tokens and pre-reset results; cluster value is a derived maximum, never independently fused.

Constraints: preserve current CLIP/BLIP APIs, continuous motion timing, reset handshake, and user's manual edits. No downloads, no full Habitat benchmark. Keep geometric mapping fast and independent of model inference. Window visibility uses original measured free neighbors, not current centroid LOS or nearest pose. Stop subdividing connected clusters; retain extraction minimum-size filtering.

- [x] Cell memory: failing C++ tests for distinct cell weights, negative-score max, split/merge preservation, disappearance/rebirth, duplicate/out-of-order evidence; implement sparse cells + derived nodes.
- [x] Frame window: failing Python tests for 20-frame eviction, original pose/ray support, angle/distance weights, score reuse, context change; implement bounded window/batch worker and exact ROS pairing.
- [x] Wire FrontierScan/SemanticScore, independent collector into both simulation modes; retire centroid fusion; disable split call; color each boundary cell and center aggregate.
- [x] Build and run unit/ROS tests, document behavior, inspect final diff and request read-only review.

Review focus: delayed results after deletion/reset; nonlinear fusion with out-of-order results; inference slower than sensor rate; first RGB preserved before annotations; stale target or late score after shutdown.

Review resolution: sensor RGB remains raw at its source (annotated discrete/manual
images only for display/video). Each batch also publishes one original image
score with empty cell evidence where necessary to retain negative-object updates.
Verification: 38 unit tests; Jazzy packages-up-to exploration_manager build;
independent domain-94 real C++ mapping/reset tests (discrete + continuous), and
real Python collector/worker tests for both backends with controlled inference.
No model downloads or Habitat navigation experiments.

2026-10-28 user correction: inference is now strictly frontier-selected. Removed
the no-frontier object-evidence inference exception and empty score publication.
Frames remain in the live window until eviction and are selected when a current
frontier is supported by their original scan. Log only nonempty frontier updates.
Object evidence may reuse those selected scores, without requesting extra model
calls. Added ROS regressions for unrelated frames, frontier-triggered historical
frame selection and frontier disappearance during inference.
