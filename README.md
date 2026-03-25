# AssistiveNav

A real-time spatial navigation assistant for visually impaired users. The app uses sparse optical flow computed natively in C++ to detect obstacles in the user's path and deliver low-cognitive-load spatial audio cues to guide navigation.

Designed for chest-mounted use on Android devices.

---

## Status

Active development. Current milestone: optical flow pipeline and visualization complete. Grid analysis, IMU fusion, and audio feedback are in progress.

---

## How It Works

The camera feed is divided into a 3x3 grid. The two center-bottom cells represent the primary danger zone directly ahead of the user. Sparse optical flow (Lucas-Kanade pyramidal tracking) is computed on every camera frame natively in C++. Anomalous flow patterns in the danger zone trigger spatial audio cues that indicate obstacle direction without interrupting the user's natural hearing.

The system is designed around one constraint: it must run at a stable 30 FPS on a mid-range Android device.

---

## Architecture

```
CameraX (Kotlin)
    | YUV_420_888 Y-plane
    v
JNI Bridge
    |
    v
C++ Pipeline (background thread)
    |- FlowEngine        goodFeaturesToTrack + calcOpticalFlowPyrLK + FB check
    |- renderToRgba      OpenCV arrow + grid overlay, rotated to display orientation
    |
    v
ImageView (Kotlin)     RGBA bitmap drawn each frame

[Planned]
    |- GridAnalyzer      per-cell flow metrics, FOE estimation, TTC
    |- IMU Fusion        rotation compensation, ego-motion separation
    |- OpenAL-Soft       spatial bandpass-filtered white noise cues
```

---

## Technology Stack

| Layer | Technology |
|---|---|
| Language (native) | C++17 |
| Language (app) | Kotlin |
| Optical flow | OpenCV 4.10.0 (prebuilt Android SDK) |
| Camera | CameraX 1.3.4 |
| Audio (planned) | OpenAL-Soft |
| Build | CMake 3.22.1, Gradle 8.11.1, AGP 8.9.0 |
| Min SDK | API 26 (Android 8.0) |
| Target ABI | arm64-v8a |

---

## Requirements

- Android Studio Ladybug or later
- NDK 27.0.12077973 (install via SDK Manager)
- CMake 3.22.1 (install via SDK Manager)
- OpenCV 4.10.0 Android SDK — download from https://opencv.org/releases/
- A physical Android device for testing (API 26+). The emulator does not provide a real camera feed.

---

## Setup

**1. Clone the repository**

```bash
git clone https://github.com/rehanreghunath/AssistiveNavigation-Improved.git
cd AssistiveNavigation-Improved
```

**2. Download and extract OpenCV Android SDK**

Place it at the same level as the project folder:

```
workspace/
  OpenCV-android-sdk/
  AssistiveNav/
```

**3. Configure local.properties**

Add one line below the existing `sdk.dir` entry:

```properties
opencv.dir=/absolute/path/to/OpenCV-android-sdk/sdk/native/jni
```

**4. Sync and build**

Open in Android Studio, sync Gradle, then build. No other configuration is required.

---

## Project Structure

```
app/src/main/
  cpp/
    CMakeLists.txt
    FlowEngine.h         core pipeline class declaration
    FlowEngine.cpp       preprocessing, LK tracking, FB check, visualization
    jni_bridge.cpp       JNI entry points connecting Kotlin to C++
  java/com/rehanreghunath/assistivenav/
    FlowBridge.kt        external function declarations
    MainActivity.kt      camera setup, lifecycle, frame delivery
  res/layout/
    activity_main.xml
```

---

## Building for Release

```bash
./gradlew assembleRelease
```

The release APK will be at `app/build/outputs/apk/release/app-release-unsigned.apk`. Sign it with your keystore before distribution.

---

## Device Notes

Primary development and debug device is the Samsung Galaxy A14 (MediaTek Helio G80, Android 13). The pipeline is tuned for this hardware profile: 640x480 resolution, 200 feature points, 4-level LK pyramid.

Performance on other devices may vary. The frame resolution and feature count are configurable in `FlowEngine.h`.

---

## Known Limitations

- Same-speed movers (a person walking at identical speed to the user) produce near-zero relative optical flow and are not detected by flow alone. A hybrid detection layer is planned.
- Absolute depth and metric distance cannot be derived from monocular optical flow. The system uses Time-to-Collision approximations and relative flow magnitude instead.
- Low-texture environments (blank walls, open sky) reduce the number of trackable features. Grid-aware feature seeding mitigates but does not eliminate this.

---

## Planned Features

- IMU fusion for ego-rotation compensation and velocity estimation
- FOE (Focus of Expansion) estimation with RANSAC
- Per-cell TTC (Time-to-Collision) scoring
- Spatial audio cues via OpenAL-Soft
- Lightweight object detector running at low frequency as a fallback for the same-speed mover problem

---

## License

MIT License. See `LICENSE` for details.
