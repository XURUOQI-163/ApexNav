# Target-only detection implementation plan

Goal: Remove confusing-object priors, detection and classification competition while preserving target multi-view fusion, room priors and frontier semantic memory.
Design authorized by the user's explicit removal request. Work directly in the current repository; preserve unrelated manual edits.

- [x] Update regression contracts: LLM returns room/threshold only, detector returns target masks/scores only.
- [x] Remove confusing-object prompts/output/cache values and wire all Python runners to the new contracts. Keep target aliases, room and threshold.
- [x] Remove ROS label_indices and C++ per-class arrays/competition. Retain weighted target evidence, observed-but-undetected updates, normal confidence/count gating and target fallback.
- [x] Rebuild affected ROS2 packages; verify target fusion and frontier/reset integration on isolated domain 94, no model downloads/cloud calls.
- [x] Update usage docs, review diff and report restart requirements.

Validation: 50 Python unit tests passed; plan_env/path_searching/exploration_manager/trajectory_manager built in Jazzy. CTest target_object_fusion passed against the real mapping library. Controlled discrete/continuous runners passed. ROS domain 94 semantic alignment/reset integration passed including delayed score projection, duplicate suppression, clearing previous map/frontiers/markers and rejecting old scores. Review finding about legacy custom caches fixed with a regression test. No live model calls or full navigation evaluation performed.
