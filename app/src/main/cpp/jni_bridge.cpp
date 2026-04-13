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

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeUpdateImu(
        JNIEnv* env, jobject /*thiz*/,
        jfloatArray quaternion, jlong timestampNs) {
    if (!quaternion) return;
    const jsize len = env->GetArrayLength(quaternion);
    if (len < 4) return;
    jfloat q[5] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    env->GetFloatArrayRegion(quaternion, 0, std::min(len, static_cast<jsize>(5)), q);
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (gImuFusion)
        gImuFusion->updateRotation(q[0], q[1], q[2], q[3],
                                   static_cast<int64_t>(timestampNs));
}

// ── Full pipeline ─────────────────────────────────────────────────────────────
//  Audio path change: AudioEngine::updateFromGrid(gridResult) replaces the
//  broken AudioEngine::update(ObstacleFrame) path.
//
//  Old path:  FlowEngine → ImuFusion → GridAnalyzer → FlowClassifier
//             → ObstacleTracker → AudioEngine::update(obstacles)
//             [BROKEN: anomaly score = 0 for forward approach; grid ignored]
//
//  New path:  FlowEngine → ImuFusion → GridAnalyzer
//             → AudioEngine::updateFromGrid(grid)
//             [FIXED: dangerScore is magnitude-based; symmetric for approach
//              and retreat; grid centre column directly drives audio]
//
//  ObstacleTracker and FlowClassifier still run so their output is available
//  for future HUD/haptic use, but they no longer gate audio.

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
        // Step 1: IMU rotation compensation.
        if (gImuFusion) gImuFusion->compensate(result);

        // Step 2: Grid analysis (RANSAC FOE + temporal baseline).
        // Non-const — mutates EMA state.
        assistivenav::GridResult gridResult{};
        if (gGridAnalyzer) {
            gridResult      = gGridAnalyzer->analyze(result);
            gLastGridResult = std::make_unique<assistivenav::GridResult>(gridResult);
        }

        // Step 3: Spatial audio directly from the grid.
        // This is the fix: no anomaly classifier, no obstacle tracker in the
        // audio path.  dangerScore is magnitude-based and fires symmetrically
        // for both approach and retreat.
        if (gAudioEngine) {
            gAudioEngine->updateFromGrid(gridResult);
        }

        // Step 4: Anomaly classification (still runs; output available for HUD).
//        if (gFlowClassifier) {
//            gFlowClassifier->classify(result, gridResult,
//                                      static_cast<int>(width),
//                                      static_cast<int>(height));
//        }

        // Step 5: ObstacleTracker still runs for future use (haptics, display
        // overlay).  Its output no longer affects audio.
//        if (gObstacleTracker) {
//            gObstacleTracker->update(result, gridResult);
//        }
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