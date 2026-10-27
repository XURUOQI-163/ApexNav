# Timestamp-aligned semantics and independent backends

Goal: preserve CLIP inference, add BLIP2 ITC as an independent HTTP service, select at simulation startup, and fuse each semantic observation once at its original camera pose.

Architecture: `/semantic/score` carries image Header, backend and cosine. C++ caches the free cells and pose from exactly synchronized depth/pose messages by acquisition timestamp. A bounded, expiring join accepts either arrival order, rejects duplicates, obsolete results and pre-reset frames. Continuous RGB/depth/pose use exact synchronization; discrete publication keeps one timestamp per observation. No model dependencies in simulation clients.

Alternatives: blocking the continuous simulation on inference would alter motion timing; carrying full depth images back with each score would duplicate projection and network payload. Cache the already projected geometry instead. Missing/dropped observations are discarded with diagnostics, never matched to a nearby timestamp.

Constraints: no dependency/model downloads; no changes to CLIP model/HTTP contract; BLIP2 uses original pretrain ITC rather than ITM probabilities; no RViz cleanup or navigation-policy changes. Backend changes require simulation/planner restart. Existing uncommitted work is preserved.

- [x] Tests first: backend routing, BLIP2 ITC contract, CLI, C++ delayed-frame pairing, duplicates, expiry, reset and bounds.
- [x] Add independent BLIP2 module and selector; preserve existing CLIP tests.
- [x] Add stamped SemanticScore and cache; remove unstamped scores from mapping input.
- [x] Publish matching stamps from continuous and discrete paths; ensure exact RGB/depth/pose synchronization.
- [x] Build ROS2, run all tests, delayed-score integration, offline BLIP2 smoke when cached resources permit.
- [x] Document commands, cache limits, HTTP endpoints, new ROS topic and validation results.

Validation: 31 Python/C++ tests; 6 ROS2 packages built; live C++ delayed-frame join verified at poses 0 m / 8 m; both Python backends preserve timestamps under injected latency; real BLIP2 CUDA ITC and Flask request passed using existing cache with downloads disabled. No full navigation benchmark performed.
