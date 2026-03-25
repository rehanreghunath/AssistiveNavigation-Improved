# Contributing to AssistiveNav

---

## Before You Start

This project is under active development and the architecture is still being established. Before writing any code, open an issue describing what you intend to change and why. This avoids duplicate work and ensures the change is aligned with the project direction.

Pull requests submitted without a prior issue will not be reviewed until one is created.

---

## Development Environment

Follow the setup instructions in the README exactly. The most common source of build failures is an incorrect `opencv.dir` path in `local.properties` or a mismatch between NDK and CMake versions.

Verify your environment before making changes:

1. The app builds cleanly from a fresh checkout
2. The app runs on a physical device and produces a live flow visualization
3. Logcat shows no errors tagged `FlowPipeline` or `JNI`

If any of these fail, resolve the environment issue first.

---

## Code Standards

### C++

- Standard: C++17
- All new classes go in the `assistivenav` namespace
- No heap allocation in the hot loop (`processFrame`, `renderToRgba`, `trackFeatures`). Use `reserve()` in constructors and `clear()` in methods.
- No exceptions. Use return values and log macros (`LOGI`, `LOGE`) for error handling.
- Every new public method in a header must have a brief comment explaining its contract, not its implementation.
- Mark parameters unused by the compiler with `/* name */` syntax, not `(void)name`.
- Prefer `const` on all local variables that are not reassigned.
- Do not use `auto` where the type is not immediately obvious from the right-hand side.

### Kotlin

- Kotlin is for UI, lifecycle, and camera management only. No processing logic belongs in Kotlin.
- All JNI calls that are not lifecycle events should be non-blocking. If a native call may take more than 1ms, it must be called from a background thread.
- `@Volatile` on any field shared between the camera executor thread and the main thread.
- Use `runOnUiThread` only for final UI updates, not for logic.

### Naming

- C++ classes: `PascalCase`
- C++ methods and members: `camelCase`, members prefixed with `m` (e.g. `mPrevGray`)
- C++ constants: `kCamelCase`
- Kotlin: follow standard Kotlin conventions
- JNI functions: follow the mandatory `Java_package_Class_method` convention exactly

### Comments

Write comments that explain **why**, not **what**. Code that states the obvious is noise.

```cpp
// Good: explains a non-obvious decision
// kMinEigThreshold filters out textureless patches where LK cannot reliably solve.
static constexpr float kMinEigThreshold = 1e-4f;

// Bad: restates what the code already says
// Set threshold to 1e-4
static constexpr float kMinEigThreshold = 1e-4f;
```

---

## Workflow

### Branching

```
main          stable, tested code only
dev           integration branch
feature/xxx   your work branch, forked from dev
fix/xxx       bug fix branch, forked from dev
```

Never commit directly to `main` or `dev`.

### Commits

Each commit must represent a single logical change. Do not batch unrelated fixes into one commit.

Format:

```
component: short imperative description (under 72 characters)

Optional body: why this change was made, what alternatives were considered,
any non-obvious consequences.
```

Examples:

```
FlowEngine: replace median blur with Gaussian before LK tracking

FlowBridge: add nativeSetRenderRotation for display orientation fix

jni_bridge: cache last FlowResult for renderToRgba access
```

Do not include issue numbers in the commit subject. Reference them in the PR description instead.

### Pull Requests

- One PR per logical change
- PR title matches the commit format above
- Description must include: what changed, why, and how you tested it
- All new native functions must have a corresponding `external fun` declaration in `FlowBridge.kt` and must be exercised in at least one manual test
- PRs that reduce tracked feature count, increase frame time, or break the visualization will not be merged without a documented justification

---

## Testing

There is no automated test suite yet. All testing is manual on a physical device.

When submitting a PR, state explicitly in the description:

- Which device(s) you tested on
- The tracked feature count range you observed (should be above 120 consistently)
- Whether Logcat was clean during a 2-minute run

Performance regressions must be measured. If your change touches the hot loop, provide before/after frame time from Logcat timestamps.

---

## Performance Rules

These are hard constraints, not guidelines:

- The pipeline must sustain 30 FPS on a low-mid range android phone (Helios G80/Exynos 1330)
- No `std::vector`, `cv::Mat`, or any other heap allocation inside `processFrame`, `trackFeatures`, or `renderToRgba`
- No JNI object creation (e.g. `NewFloatArray`, `NewByteArray`) must block the camera thread for more than 2ms
- The camera executor thread must return from `analyze()` within 33ms

If a new feature cannot meet these constraints, it must run on a separate low-priority thread and must not block the camera thread.

---

## What is Not Accepted

- Changes that add Kotlin-side processing logic that belongs in C++
- Changes that add dependencies without prior discussion
- Cosmetic refactors submitted as standalone PRs
- Code that disables or weakens the forward-backward consistency check without a documented reason
- Any introduction of `using namespace cv` or `using namespace std` in a header file

---

## Questions

Open an issue with the label `question`. Do not use pull request comments for design questions.
