# AssistiveNav

A real-time assistive navigation system for visually impaired users. AssistiveNav tracks obstacles in the user's path using sparse optical flow and IMU-compensated motion analysis,
then communicates their position through HRTF-based spatial audio — without requiring the user to look at a screen or wear specialized hardware.
Designed for chest-mounted use on Android devices, with the entire perception pipeline running natively in C++ at a sustained 30 FPS on mid-range hardware.

## Status

Active development. The core five-stage pipeline — optical flow, IMU fusion, grid analysis, obstacle tracking, and spatial audio — is implemented end-to-end and runs live on-device.

## How It Works

A chest-mounted camera feeds grayscale frames into a native pipeline that tracks sparse feature points frame-to-frame, removes the portion of that motion caused by the user's own head 
and body rotation, and identifies image regions where the remaining motion cannot be explained by the camera's forward movement through a static scene. Those regions are treated as obstacles.
Each one is converted into a spatial audio cue — panned left or right based on its position in the frame, and pitched based on how urgent it is — so the user can build a mental map of what's ahead without any visual or haptic interface.

The system deliberately avoids machine learning models currently. Every detection is derived from classical computer vision (optical flow, RANSAC, homography) and on-device sensors (camera, IMU), keeping the pipeline lightweight, deterministic, and explainable.

## Pipeline

```
CameraX (Kotlin)
    │ YUV_420_888 Y-plane + TYPE_ROTATION_VECTOR
    ▼
JNI Bridge
    │
    ▼
C++ Pipeline (camera executor thread)
    │
    ├─ FlowEngine        Shi-Tomasi feature detection, pyramidal Lucas-Kanade
    │                     tracking, forward-backward consistency check
    │
    ├─ ImuFusion          Rotation compensation via device orientation quaternion;
    │                     subtracts camera-rotation-induced flow before analysis
    │
    ├─ GridAnalyzer        3×3 danger grid, RANSAC-estimated Focus of Expansion,
    │                     temporal baseline tracking per cell
    │
    ├─ FlowClassifier      Per-vector anomaly scoring — separates ego-motion-
    │                     consistent flow (floor, walls) from true obstacles
    │
    ├─ ObstacleTracker     12×8 activity grid, flood-fill blob extraction,
    │                     EMA-smoothed temporal matching, age-gated output
    │
    └─ AudioEngine         OpenAL-Soft spatial audio — up to 5 dynamically
                          positioned sources, HRTF-rendered direction and gain
    │
    ▼
ImageView (Kotlin)      RGBA debug overlay (flow vectors + grid), drawn each frame
```

## Technology Stack

| Layer | Technology |
|---|---|
| Language (native) | C++17 |
| Language (app) | Kotlin |
| Optical flow | OpenCV 4.10.0 (prebuilt Android SDK) |
| Spatial audio | OpenAL-Soft (HRTF, OpenSL ES backend) |
| Camera | CameraX 1.3.4 |
| Orientation sensing | Android `TYPE_ROTATION_VECTOR` |
| Build | CMake 3.22.1, Gradle 8.11.1, AGP 8.9.0, NDK 27 |
| Min SDK | API 26 (Android 8.0) |
| Target ABI | arm64-v8a |

## Requirements

- Android Studio Ladybug or later
- NDK 27.0.12077973 (install via SDK Manager)
- CMake 3.22.1 (install via SDK Manager)
- OpenCV 4.10.0 Android SDK — [opencv.org/releases](https://opencv.org/releases/)
- OpenAL-Soft, built for `arm64-v8a` with `ALSOFT_BACKEND_OPENSL=ON` and `ALSOFT_EMBED_HRTF_DATA=ON`
- A physical Android device running API 26+. The emulator does not provide a usable camera feed or rotation sensor.

## Setup

**1. Clone the repository**

```bash
git clone https://github.com/rehanreghunath/AssistiveNav.git
cd AssistiveNav
```

**2. Download and extract the OpenCV Android SDK**

Place it alongside the project folder:

```
workspace/
  OpenCV-android-sdk/
  AssistiveNav/
```

**3. Build OpenAL-Soft for Android**

Build with CMake targeting `arm64-v8a`, with the OpenSL ES backend and embedded HRTF data enabled. The build output must produce `libopenal.so` under `<openal-soft>/build-android/arm64-v8a/`.

**4. Configure `local.properties`**

Add the following, alongside the existing `sdk.dir` entry:

```properties
opencv.dir=/absolute/path/to/OpenCV-android-sdk/sdk/native/jni
openal.dir=/absolute/path/to/openal-soft
```

**5. Sync and build**

Open the project in Android Studio, sync Gradle, then build. No further configuration is required.

## Project Structure

```
app/src/main/
  cpp/
    CMakeLists.txt
    FlowEngine.{h,cpp}        Optical flow tracking and visualization
    ImuFusion.{h,cpp}         Rotation compensation from device orientation
    GridAnalyzer.{h,cpp}      3x3 grid metrics, FOE estimation, temporal baseline
    FlowClassifier.{h,cpp}    Per-vector ego-motion vs. obstacle scoring
    ObstacleTracker.{h,cpp}   Blob extraction and temporal obstacle tracking
    AudioEngine.{h,cpp}       OpenAL-Soft spatial audio rendering
    FlowTypes.h               Shared data structures across all native modules
    jni_bridge.cpp            JNI entry points connecting Kotlin to the pipeline
  java/com/rehanreghunath/assistivenav/
    FlowBridge.kt             External native function declarations
    MainActivity.kt           Camera setup, lifecycle, sensor delivery
  res/layout/
    activity_main.xml
```

## Building for Release

```bash
./gradlew assembleRelease
```

The unsigned release APK is produced at `app/build/outputs/apk/release/app-release-unsigned.apk`. Sign it with your own keystore before distribution.

## Device Notes

Primary development and debug target is the Samsung Galaxy A14 (MediaTek Helio G80, Android 13). All tuning constants — feature count, grid thresholds, anomaly scoring — are calibrated against this hardware profile at 1280x720 input resolution.

Performance on other devices has not been validated. Frame resolution and feature count are configurable in `FlowEngine.h`.

## Limitations

- **Same-speed movers.** A person walking at the same speed and heading as the user produces near-zero relative optical flow and is not reliably detected by motion alone. 
  `GridAnalyzer`'s temporal baseline tracking partially mitigates this by flagging sudden deviations from a cell's recent history, but a mover at a constant relative speed remains a blind spot.
- **Monocular depth ambiguity.** Optical flow yields relative, not absolute, depth. The system uses flow magnitude and time-to-collision approximations in place of metric distance.
- **Low-texture environments.** Blank walls and open sky reduce the number of trackable feature points, weakening downstream analysis. Grid-aware feature seeding mitigates but does not eliminate this.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for code standards, branching conventions, and performance requirements before opening a pull request. An issue is required before any non-trivial change.

## License

MIT License. See [LICENSE](LICENSE) for details.
