# Unified Topological Memory Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan in the existing user workspace. The user has requested implementation and limited verification, without unrelated refactoring.

**Goal:** Persist the existing Skeleton, current Frontier semantics and representative RGB observations without changing navigation.

**Architecture:** A read-only C++ Frontier snapshot supplements the existing typed SkeletonGraph topic. An independent Python ROS collector synchronizes original RGB/scan/pose frames; one background worker selects observations and writes a ROS-independent UnifiedTopologyMemory store. Simulator context identifies the scene and episode coordinate origin.

**Tech Stack:** Existing ROS2 Jazzy, Python standard library, cv_bridge/OpenCV. No new model or dependency downloads.

**Spec:** Latest user request in this conversation. Rolling maps/eviction explicitly excluded.

## Global Constraints

- Keep existing extraction, semantic fusion, planning and control algorithms unchanged.
- Store under project `data/topology`; retain data at exit; never combine different Habitat coordinate origins.
- Skeleton paths and Frontier attachments remain distinct; abstract Junction centers are not automatically traversable.
- Record raw versus fused scores explicitly; unknown model versions/normalization/mean clearance remain null.
- Loading memory initializes the UTM query layer, not the live occupancy map or navigation state.

## Review Focus

- Restart in the same scene with a different initial pose: archive old memory, do not merge.
- Old transient-local graph at startup or delayed image inference: reject using acquisition/session timestamps.
- Partial writes and invalid paths/references: atomic per-file replacement, backup recovery and validation.
- Reset source IDs or change semantic goal: namespace transient IDs and revalidate persistent matches.
- Slow disk: bounded, coalescing worker; ROS callbacks never encode images or write files.

## Tasks

- [x] Add small tempfile-based store tests for round trip, goal/session identity, invalid references and image validation; run to confirm missing implementation.
- [x] Implement `topology_memory/store.py` and query CLI: atomic JSON, history, observations, validation and recovery.
- [x] Add typed Frontier snapshots and exact-frame semantic observation hook, with simulator context metadata.
- [x] Implement ROS collector and bounded worker; reuse original-frame projection for keyframe visibility.
- [x] Add launch/configuration and usage/schema examples with recovery instructions.
- [x] Run focused tests, Python syntax checks, build affected ROS packages, and a small synthetic ROS recording smoke check. No full Habitat experiment.

## Execution record

- Used existing clean user workspace; no commits, resets or unrelated refactors.
- Ruling: recovery initializes UTM only, never live occupancy/fusion/control. Previous Frontier scores are not activated in a new episode; geometry must be reconfirmed.
- Ruling: atomic replacement is per file, with backups and reference validation, not an added database or cross-file transaction.
- Ruling: preserve source LOOP_ANCHOR/ENDPOINT if present; distinguish actual skeleton path from merged-centre display geometry; missing mean clearance/version remains null.
- Independent review found first-flush crash recovery, late scoring of existing images, and incremental same-frame evidence overwrite. Reproduced all three, fixed, and verified regression checks. ROS smoke also caught reserved Node context attribute/method names, fixed and verified.
- Verification: 8 store tests, 4 synthetic recorder/ROS tests, 6 original frame-window tests, 8 original simulation/velocity tests, 1 original sparse fusion test, 2 C++ map/fusion tests passed (29 total). ROS build: plan_env + exploration_manager passed. Build retains existing compiler/CMake warnings.
- Simulation-mode tests initially used system Python (missing omegaconf); rerun in the existing Habitat venv passed. Host compileall could not write root-owned pycache; AST syntax validation passed without touching caches.
- No dataset/model downloads; no actual Habitat episode run; synthetic tests used temporary directories and ROS_DOMAIN_ID=94.
