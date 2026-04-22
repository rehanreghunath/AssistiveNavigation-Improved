#include <jni.h>
#include <android/log.h>
#include <memory>
#include <mutex>
#include "FlowEngine.h"
#include "GridAnalyzer.h"
#include "ImuFusion.h"
#include "ObstacleTracker.h"
#include "AudioEngine.h"
#include "FlowClassifier.h"

#define LOG_TAG "JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static std::unique_ptr<assistivenav::FlowEngine>      gPipeline;
static std::unique_ptr<assistivenav::GridAnalyzer>    gGridAnalyzer;
static std::unique_ptr<assistivenav::ImuFusion>       gImuFusion;
static std::unique_ptr<assistivenav::ObstacleTracker> gObstacleTracker;
static std::unique_ptr<assistivenav::AudioEngine>     gAudioEngine;
static std::unique_ptr<assistivenav::FlowResult>      gLastResult;
static std::unique_ptr<assistivenav::GridResult>      gLastGridResult;
static std::unique_ptr<assistivenav::FlowClassifier>  gFlowClassifier;

static std::mutex gPipelineMutex;

static constexpr int kGridResultFloats = 9 * 5 + 3;  // 48

// ─────────────────────────────────────────────────────────────────────────────
//  computeSuppressionFactor
//
//  Maps ImuFusion's angular velocity estimate to a [0, 1] suppression
//  multiplier that AudioEngine applies to all target gains.
//
//  Design intent — three zones:
//    [0, kSoftThresh]      normal walking, slow head adjustment → no suppression
//    [kSoftThresh, kHardThresh]  moderate turn (looking around) → linear ramp down
//    [kHardThresh, ∞)     rapid head/body turn                 → fully silent
//
//  Threshold rationale (rad/s):
//    Walking micro-sway: < 0.15 rad/s  (< 9°/s)
//    Casual look around: 0.3 – 0.8 rad/s  (17 – 46°/s)
//    Fast head snap:     > 1.2 rad/s   (> 69°/s)
//
//  Setting kSoftThresh at 0.30 leaves normal ambulation untouched.
//  Setting kHardThresh at 1.20 ensures any deliberate scan suppresses audio.
// ─────────────────────────────────────────────────────────────────────────────

static float computeSuppressionFactor(const float rotRateRadPerSec) {
    static constexpr float kSoftThresh = 0.30f;   // ~17°/s — suppression begins
    static constexpr float kHardThresh = 1.20f;   // ~69°/s — fully silent

    if (rotRateRadPerSec >= kHardThresh) return 0.0f;
    if (rotRateRadPerSec <= kSoftThresh) return 1.0f;
    return 1.0f - (rotRateRadPerSec - kSoftThresh) / (kHardThresh - kSoftThresh);
}

extern "C" {

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeInit(
        JNIEnv* /*env*/, jobject /*thiz*/, jint width, jint height) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);

    gPipeline        = std::make_unique<assistivenav::FlowEngine>(
            static_cast<int>(width), static_cast<int>(height));
    gGridAnalyzer    = std::make_unique<assistivenav::GridAnalyzer>(
            static_cast<int>(width), static_cast<int>(height));
    gImuFusion       = std::make_unique<assistivenav::ImuFusion>(
            static_cast<int>(width), static_cast<int>(height));
    gObstacleTracker = std::make_unique<assistivenav::ObstacleTracker>(
            static_cast<int>(width), static_cast<int>(height));
    gAudioEngine     = std::make_unique<assistivenav::AudioEngine>();
    gFlowClassifier  = std::make_unique<assistivenav::FlowClassifier>();

    if (!gAudioEngine->isReady()) {
        LOGE("AudioEngine init failed — continuing without audio");
    }
    LOGI("All subsystems init %dx%d", (int)width, (int)height);
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeSetFocalLength(
        JNIEnv* /*env*/, jobject /*thiz*/, jfloat focalLengthPx) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (gImuFusion) gImuFusion->setFocalLength(static_cast<float>(focalLengthPx));
}

// ─────────────────────────────────────────────────────────────────────────────
//  nativeUpdateImu  — sensor callback
//
//  The mutex previously held here was unnecessary: ImuFusion uses atomic
//  double-buffering internally (mReadIdx, mHasData), so updateRotation() is
//  already safe to call from the sensor thread concurrently with compensate()
//  on the camera thread.
//
//  The only genuine race is against nativeInit / nativeDestroy which write
//  gImuFusion itself.  Those are lifecycle events that run on the UI thread;
//  the SensorEventListener is unregistered in onPause() before onDestroy()
//  calls stopPipeline(), so in practice gImuFusion is always valid here.
//  The null check below guards the residual theoretical window.
// ─────────────────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeUpdateImu(
        JNIEnv* env, jobject /*thiz*/,
        jfloatArray quaternion, jlong timestampNs) {
    if (!quaternion) return;
    const jsize len = env->GetArrayLength(quaternion);
    if (len < 4) return;

    jfloat q[5] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    env->GetFloatArrayRegion(quaternion, 0, std::min(len, static_cast<jsize>(5)), q);

    // No pipeline mutex needed — ImuFusion is atomically double-buffered.
    if (gImuFusion)
        gImuFusion->updateRotation(q[0], q[1], q[2], q[3],
                                   static_cast<int64_t>(timestampNs));
}

// ─────────────────────────────────────────────────────────────────────────────
//  nativeProcessFrame — full pipeline
//
//  Audio trigger path (restored and extended):
//
//    FlowEngine → ImuFusion(compensate) → GridAnalyzer
//      → FlowClassifier (anomalyScore per vector)
//      → ObstacleTracker (blob persistence, age gate, confidence gate)
//      → computeSuppressionFactor(ImuFusion::rotationRateRadPerSec())
//      → AudioEngine::updateFromObstacles(frame, suppressionFactor)
//
//  Key improvements over the previous updateFromGrid path:
//
//    SIDE SELECTION: AudioEngine now uses obstacle.normX (blob centroid X)
//    for left/right panning.  This is immune to the flow-direction inversion
//    that occurred when the user turned: background objects generate flow in
//    the opposite direction to the user's motion, whereas the obstacle's
//    pixel position in the frame correctly reflects its real-world side.
//
//    TEMPORAL STABILITY: ObstacleTracker requires an obstacle to persist for
//    kMinAgeForAudio=5 frames and maintain confidenceScore≥kMinBlobAnomaly=0.3.
//    Single-frame flow bursts (specular reflections, door openings) do not fire.
//
//    ROTATION SUPPRESSION: suppressionFactor ramps to 0 when ImuFusion detects
//    angular velocity > kSoftThresh rad/s.  This eliminates false triggers from
//    head/body turns even when the homography compensation leaves residuals.
//
//    EGO-MOTION REJECTION: FlowClassifier's anomalyScore (angular + magnitude
//    anomaly relative to the FOE) ensures ObstacleTracker only promotes blobs
//    whose vectors deviate from the background ego-motion pattern.  Floor
//    texture and distant static walls receive low anomaly and are suppressed
//    at the activity-grid stage.
// ─────────────────────────────────────────────────────────────────────────────

JNIEXPORT jfloatArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeProcessFrame(
        JNIEnv* env, jobject /*thiz*/,
        jbyteArray yPlane, jint width, jint height, jint rowStride, jlong timestampNs) {

    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (!gPipeline) { LOGE("processFrame before init"); return nullptr; }

    jbyte* yData = env->GetByteArrayElements(yPlane, nullptr);
    if (!yData) return nullptr;

    assistivenav::FlowResult result = gPipeline->processFrame(
            reinterpret_cast<const uint8_t*>(yData),
            static_cast<int>(rowStride),
            static_cast<int64_t>(timestampNs));

    env->ReleaseByteArrayElements(yPlane, yData, JNI_ABORT);

    if (!result.isFirstFrame) {
        // Step 1: Remove rotation ego-motion from every flow vector.
        //         Also updates ImuFusion's internal angular-velocity estimate.
        if (gImuFusion) gImuFusion->compensate(result);

        // Step 2: Grid analysis — RANSAC FOE + per-cell metrics + temporal baseline.
        assistivenav::GridResult gridResult{};
        if (gGridAnalyzer) {
            gridResult      = gGridAnalyzer->analyze(result);
            gLastGridResult = std::make_unique<assistivenav::GridResult>(gridResult);
        }

        // Step 3: Assign anomalyScore to each vector.
        //         Score = 0 → consistent with background ego-motion (floor, wall)
        //         Score = 1 → independent of ego-motion (close obstacle, mover)
        //         Requires gridResult for the FOE estimate.
        if (gFlowClassifier) {
            gFlowClassifier->classify(result, gridResult,
                                      static_cast<int>(width),
                                      static_cast<int>(height));
        }

        // Step 4: Blob detection, temporal matching, age-gate, confidence-gate.
        //         ObstacleFrame.obstacles[i].active == true only when the blob
        //         has survived kMinAgeForAudio frames AND confidenceScore ≥ threshold.
        assistivenav::ObstacleFrame obstacleFrame{};
        if (gObstacleTracker) {
            obstacleFrame = gObstacleTracker->update(result, gridResult);
        }

        // Step 5: Rotation suppression.
        //         Angular velocity is derived from the delta quaternion computed
        //         in step 1 — no extra sensor, no extra JNI call.
        const float rotRate = gImuFusion ? gImuFusion->rotationRateRadPerSec() : 0.0f;
        const float suppressionFactor = computeSuppressionFactor(rotRate);

        // Step 6: Spatial audio.
        //         Side selection is based on obstacle normX (frame position),
        //         not flow direction, so it is correct during and after rotation.
        if (gAudioEngine) {
            gAudioEngine->updateFromObstacles(obstacleFrame, suppressionFactor);
        }
    }

    gLastResult = std::make_unique<assistivenav::FlowResult>(result);

    const float stats[3] = {
            static_cast<float>(result.trackedCount),
            static_cast<float>(result.totalPts),
            result.globalMeanMag
    };
    jfloatArray out = env->NewFloatArray(3);
    if (out) env->SetFloatArrayRegion(out, 0, 3, stats);
    return out;
}

JNIEXPORT jbyteArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeGetRgbaFrame(
        JNIEnv* env, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (!gPipeline || !gLastResult) return nullptr;
    const std::vector<uint8_t> rgba = gPipeline->renderToRgba(*gLastResult);
    jbyteArray out = env->NewByteArray(static_cast<jsize>(rgba.size()));
    if (out)
        env->SetByteArrayRegion(out, 0, static_cast<jsize>(rgba.size()),
                                reinterpret_cast<const jbyte*>(rgba.data()));
    return out;
}

JNIEXPORT jfloatArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeGetGridResult(
        JNIEnv* env, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (!gLastGridResult) return nullptr;

    float buf[kGridResultFloats] = {};
    for (int i = 0; i < 9; ++i) {
        const assistivenav::CellMetrics& c = gLastGridResult->cells[i];
        const int base = i * 5;
        buf[base + 0] = c.meanMag;
        buf[base + 1] = c.meanAngle;
        buf[base + 2] = c.dangerScore;
        buf[base + 3] = c.ttc;
        buf[base + 4] = static_cast<float>(c.sampleCount);
    }
    buf[45] = gLastGridResult->foeX;
    buf[46] = gLastGridResult->foeY;
    buf[47] = gLastGridResult->foeValid ? 1.0f : 0.0f;

    jfloatArray out = env->NewFloatArray(kGridResultFloats);
    if (out) env->SetFloatArrayRegion(out, 0, kGridResultFloats, buf);
    return out;
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeSetRenderRotation(
        JNIEnv* /*env*/, jobject /*thiz*/, jint degrees) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (gPipeline) gPipeline->setRenderRotation(static_cast<int>(degrees));
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeDestroy(
        JNIEnv* /*env*/, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    gFlowClassifier.reset();
    gAudioEngine.reset();
    gObstacleTracker.reset();
    gGridAnalyzer.reset();
    gImuFusion.reset();
    gPipeline.reset();
    gLastResult.reset();
    gLastGridResult.reset();
    LOGI("All native resources destroyed");
}

} // extern "C"