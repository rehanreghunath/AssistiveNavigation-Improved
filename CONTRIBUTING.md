# Contributing to AssistiveNav

This is a navigation aid for visually impaired users. A missed obstacle here is not a cosmetic bug — it is a person walking into something. Hold every change, however small, to that standard before opening a PR.

## Before You Start

Open an issue describing what you intend to change and why, before writing code. This project's architecture is still being established, and unsolicited PRs duplicate work or conflict with direction already in progress. PRs without a prior issue will not be reviewed.

Verify your environment first:

1. The app builds cleanly from a fresh checkout (see README for setup).
2. It runs on a physical device and produces a live flow visualization.
3. Logcat is clean — no errors tagged `FlowPipeline`, `JNI`, `ImuFusion`, `ObstacleTracker`, or `AudioEngine` — during a 2-minute run.

Resolve environment issues before touching code. A physical device is required; the emulator cannot provide a real camera feed or rotation sensor, so it cannot validate any change to this pipeline.

## Ethics and Scope

- **No black boxes.** This pipeline is intentionally classical (optical flow, RANSAC, homography) rather than ML-based, so every decision is explainable and debuggable on-device. Do not introduce a model or heuristic whose failure mode you cannot describe.
- **Fail safe, not silent.** If a subsystem cannot produce a confident result, it should suppress output, not guess. A wrong audio cue that points the user toward a hazard is worse than no cue at all. The existing age-gate and confidence-gate in `ObstacleTracker` exist for this reason — do not weaken them without a documented justification in the PR.
- **State limitations honestly.** If your change has a known failure case (a lighting condition, a motion pattern, a device profile), document it in the PR and in the relevant header comment. Do not let a known gap go unstated because the demo looked fine.
- **No telemetry, no network calls.** This pipeline runs entirely on-device. Don't introduce dependencies that phone home, log to a remote service, or require connectivity.

## Code Standards

### C++

- C++17, all new classes in the `assistivenav` namespace.
- No heap allocation in the hot loop (`processFrame`, `trackFeatures`, `renderToRgba`, and equivalents in other modules). Reserve in constructors, clear in methods.
- No exceptions. Return values and `LOGI`/`LOGE` for error handling.
- Every public header method gets a comment on its contract, not its implementation.
- `const` on every local not reassigned. Avoid `auto` where the right-hand type isn't obvious.
- No `using namespace` in headers.
- Naming: classes `PascalCase`; methods/members `camelCase` with member fields prefixed `m`; constants `kCamelCase`; JNI functions follow `Java_package_Class_method` exactly.

### Kotlin

- Kotlin owns UI, lifecycle, and camera plumbing only. No processing logic belongs here — if you're tempted to add a calculation in Kotlin, it belongs in C++.
- Any JNI call that can take >1ms must run off the main thread.
- `@Volatile` on any field shared between the camera executor and main thread.

### Comments

Explain why, not what. A comment restating the line below it is noise; a comment justifying a constant, a threshold, or a design tradeoff is the whole point. See existing files (`FlowClassifier.h`, `ImuFusion.h`) for the standard.

## Performance Contract

These are hard constraints, enforced at review, not aspirational targets:

- Sustained 30 FPS on Helio G80 / Exynos 1330 class hardware.
- No `std::vector`, `cv::Mat`, or any heap allocation inside a hot-loop method.
- No JNI array allocation (`NewFloatArray`, `NewByteArray`, etc.) blocks the camera thread for more than 2ms.
- `analyze()` on the camera executor returns within 33ms, full stop.

If a new feature cannot meet these constraints on the camera thread, it must run on a separate background thread and must never block frame delivery. State in the PR which thread your code runs on and why.

Any change touching a hot loop must report before/after frame time from Logcat timestamps, and the tracked feature count observed (should stay above 120 consistently). A PR that regresses either without justification will not be merged — "it's necessary for the new feature" is a justification; an unexamined regression is not.

## Determinism and Numerical Care

Obstacle detection runs on RANSAC sampling, EMA smoothing, and float thresholds tuned against real hardware. When changing any of these:

- State your testing conditions (lighting, motion pattern, device) — results here are not portable across environments by default.
- Don't change a tuning constant (e.g. `kAnomalyNormScale`, `kMinBlobAnomaly`) without explaining what failure mode you observed and how the new value addresses it.
- RNG seeding (e.g. the LCG in `GridAnalyzer`) must stay deterministic given the same input. Don't introduce nondeterminism without a reason — it makes regressions unreproducible.

## Pull Requests

- One logical change per PR.
- Description must state: what changed, why, how you tested it, which device(s), and the Logcat/feature-count/frame-time evidence above where applicable.
- Every new native function needs a matching `external fun` in `FlowBridge.kt`, exercised in at least one manual test.
- PRs that disable or weaken the forward-backward consistency check, the obstacle age/confidence gate, or any other existing safety gate will not be merged without an explicit, reviewed justification.
- Cosmetic-only refactors are not accepted as standalone PRs — fold them into a substantive change or skip them.

## Testing

There is no automated test suite. All verification is manual, on a physical device, and the burden of proof is on the PR author: if you can't demonstrate it works and didn't regress anything, it isn't ready for review.

## Questions

Open an issue labeled `question`. Don't use PR comments for design discussion — it gets lost.
