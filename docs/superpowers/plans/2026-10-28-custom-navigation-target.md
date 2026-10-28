# Custom navigation target implementation plan

> Use superpowers:executing-plans for inline execution.

Goal: allow terminal input of a complete object description while using Habitat
as a scene and motion simulator, for visual inspection rather than evaluation.

Design: add optional `--target "yellow chair"` to the shared CLI. Pass it as a
Python argument to discrete/continuous runners (outside Hydra's override parser).
Both YOLOE and the frontier semantic window receive the full description. An
uncached target with `llm_client=none` uses no distractors, room `everywhere` and
the existing fallback confidence 0.5, without contacting an LLM or writing a
fabricated cache entry. Without `--target`, retain episode category selection.

Scope: startup target selection; no live task switching, translation service,
attribute verification, custom dataset annotation or benchmark work. A custom
target selects one episode's scene/start pose (index 0 if unspecified), skips
original-goal floor filtering and metric reporting/recording. Existing planning,
frontier-only inference, model servers and reset lifecycle are reused.

- [x] Add failing CLI/target-context tests, including spaces, Unicode, punctuation,
  empty input, uncached no-LLM fallback and original category compatibility.
- [x] Implement CLI and target resolution, no-LLM cache miss handling, wire both
  runners, and bypass original-goal-only reporting/filtering for custom runs.
- [x] Verify description reaches both detector and both semantic backends with
  controlled clients; verify runner routing without loading Habitat scenes.
- [x] Document commands, run targeted/full lightweight tests and Python checks.

Review focus: full description preserved outside Hydra; unrelated cached goal
context never reused; selecting a custom goal does not accidentally batch-run
episodes; no success/SPL claim for an unannotated goal; no download required.

Verification: 43 lightweight unit tests pass. Controlled runner tests in the
Jazzy/Habitat environment cover actual Hydra CLI routing, full description
propagation, discrete and continuous scene index selection, custom discrete
video, skipped old-goal floor checks/reporting, and Python compilation. No real
scene, model service or live ROS graph was started.

Final review: one P2 finding fixed: custom discrete videos displayed original
Success/SPL metrics. Regression observed the overlay call fail, then passed with
custom video metric overlays disabled. Non-array ROS timestamp metadata is also
excluded from visual sensor inputs. No unresolved review findings.
