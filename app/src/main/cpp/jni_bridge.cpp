#include <jni.h>
#include <android/log.h>
#include <memory>
#include <mutex>
#include "FlowEngine.h"
#include "GridAnalyzer.h"
#include "ImuFusion.h"
#include "ObstacleTracker.h"
#include "AudioEngine.h"

#define LOG_TAG "JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static std::unique_ptr<assistivenav::FlowEngine>     gPipeline;
static std::unique_ptr<assistivenav::GridAnalyzer>   gGridAnalyzer;
static std::unique_ptr<assistivenav::ImuFusion>      gImuFusion;
static std::unique_ptr<assistivenav::ObstacleTracker> gObstacleTracker;
static std::unique_ptr<assistivenav::AudioEngine>    gAudioEngine;
static std::unique_ptr<assistivenav::FlowResult>     gLastResult;
static std::unique_ptr<assistivenav::GridResult>     gLastGridResult;

static std::mutex gPipelineMutex;

// ── nativeGetGridResult layout (unchanged) ────────────────────────────────────
static constexpr int kGridResultFloats = 9 * 5 + 3; // 48

extern "C" {

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeInit(
        JNIEnv* /*env*/, jobject /*thiz*/,
        jint width, jint height) {
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

    if (!gAudioEngine->isReady()) {
        LOGE("AudioEngine init failed — continuing without audio");
    }

    LOGI("All subsystems init %dx%d", (int)width, (int)height);
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
    if (gImuFusion) {
        gImuFusion->updateRotation(q[0], q[1], q[2], q[3],
                                   static_cast<int64_t>(timestampNs));
    }
}

// ── Full pipeline ─────────────────────────────────────────────────────────────
//   1. FlowEngine::processFrame    → raw LK flow
//   2. ImuFusion::compensate       → remove camera-rotation displacement
//   3. GridAnalyzer::analyze       → 3×3 danger grid (HUD / Kotlin)
//   4. ObstacleTracker::update     → temporally stable obstacle list
//   5. AudioEngine::update         → spatial audio from obstacle positions

JNIEXPORT jfloatArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeProcessFrame(
        JNIEnv* env, jobject /*thiz*/,
        jbyteArray yPlane,
        jint width, jint height, jint rowStride,
        jlong timestampNs) {

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
        if (gImuFusion) gImuFusion->compensate(result);

        if (gGridAnalyzer) {
            gLastGridResult = std::make_unique<assistivenav::GridResult>(
                    gGridAnalyzer->analyze(result));
        }

        if (gObstacleTracker && gAudioEngine) {
            const assistivenav::ObstacleFrame obstacles =
                    gObstacleTracker->update(result);
            gAudioEngine->update(obstacles);
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
    if (out) {
        env->SetByteArrayRegion(out, 0, static_cast<jsize>(rgba.size()),
                                reinterpret_cast<const jbyte*>(rgba.data()));
    }
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