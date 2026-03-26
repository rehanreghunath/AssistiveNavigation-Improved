#include <jni.h>
#include <android/log.h>
#include <memory>
#include <mutex>
#include "FlowEngine.h"
#include "GridAnalyzer.h"
#include "ImuFusion.h"
#include "AudioEngine.h"

#define LOG_TAG "JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// ── Module-level state ────────────────────────────────────────────────────────
//
// AudioEngine is intentionally a plain pointer rather than unique_ptr here.
// OpenAL-Soft's alcMakeContextCurrent / alcDestroyContext must be called from
// the same thread that created the context on some Android drivers.  We create
// and destroy on the main thread (via nativeInit / nativeDestroy) and call
// update() from the camera executor thread — which is safe because update()
// only issues alSourcef parameter writes, which are thread-safe in OpenAL-Soft.
// The mutex still serialises init / destroy / update.

static std::unique_ptr<assistivenav::FlowEngine>   gPipeline;
static std::unique_ptr<assistivenav::GridAnalyzer> gGridAnalyzer;
static std::unique_ptr<assistivenav::ImuFusion>    gImuFusion;
static std::unique_ptr<assistivenav::AudioEngine>  gAudioEngine;
static std::unique_ptr<assistivenav::FlowResult>   gLastResult;
static std::unique_ptr<assistivenav::GridResult>   gLastGridResult;

static std::mutex gPipelineMutex;

// ── nativeGetGridResult float-array layout ────────────────────────────────────
// Indices 0..44 : 9 cells × 5 floats, row-major [0]=top-left [8]=bottom-right
//   base+0  meanMag      (px/frame, IMU-compensated)
//   base+1  meanAngle    (radians, −π..π)
//   base+2  dangerScore  (0..1)
//   base+3  ttc          (frames; 0 = no motion)
//   base+4  sampleCount  (cast to float)
// Index 45  foeX         (normalised 0..1)
// Index 46  foeY         (normalised 0..1)
// Index 47  foeValid     (1.0 = valid, 0.0 = unavailable)
static constexpr int kGridResultFloats = 9 * 5 + 3; // 48

extern "C" {

// ─────────────────────────────────────────────────────────────────────────────
//  nativeInit
// ─────────────────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeInit(
        JNIEnv* /*env*/, jobject /*thiz*/,
        jint width, jint height) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);

    gPipeline     = std::make_unique<assistivenav::FlowEngine>(
            static_cast<int>(width), static_cast<int>(height));
    gGridAnalyzer = std::make_unique<assistivenav::GridAnalyzer>(
            static_cast<int>(width), static_cast<int>(height));
    gImuFusion    = std::make_unique<assistivenav::ImuFusion>(
            static_cast<int>(width), static_cast<int>(height));
    gAudioEngine  = std::make_unique<assistivenav::AudioEngine>();

    if (!gAudioEngine->isReady()) {
        // Audio init failure is non-fatal.  The visual pipeline continues;
        // the user simply gets no audio cues.
        LOGE("AudioEngine init failed — continuing without audio");
    }

    LOGI("All subsystems init %dx%d", (int)width, (int)height);
}

// ─────────────────────────────────────────────────────────────────────────────
//  nativeUpdateImu
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

    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (gImuFusion) {
        gImuFusion->updateRotation(q[0], q[1], q[2], q[3],
                                   static_cast<int64_t>(timestampNs));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  nativeProcessFrame
//
//  Full pipeline per frame:
//    1. FlowEngine::processFrame   → raw LK-tracked FlowResult
//    2. ImuFusion::compensate      → subtract camera-rotation displacement
//    3. GridAnalyzer::analyze      → per-cell danger scores and FOE
//    4. AudioEngine::update        → update spatial source gains / modulation
// ─────────────────────────────────────────────────────────────────────────────

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
        if (gImuFusion)    gImuFusion->compensate(result);

        if (gGridAnalyzer) {
            gLastGridResult = std::make_unique<assistivenav::GridResult>(
                    gGridAnalyzer->analyze(result));

            // Audio update happens immediately after grid analysis so the cues
            // are always synchronised with the latest threat assessment.
            if (gAudioEngine && gLastGridResult) {
                gAudioEngine->update(*gLastGridResult);
            }
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

// ─────────────────────────────────────────────────────────────────────────────
//  nativeGetRgbaFrame
// ─────────────────────────────────────────────────────────────────────────────

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

// ─────────────────────────────────────────────────────────────────────────────
//  nativeGetGridResult
// ─────────────────────────────────────────────────────────────────────────────

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

// ─────────────────────────────────────────────────────────────────────────────
//  nativeSetRenderRotation
// ─────────────────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeSetRenderRotation(
        JNIEnv* /*env*/, jobject /*thiz*/, jint degrees) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (gPipeline) gPipeline->setRenderRotation(static_cast<int>(degrees));
}

// ─────────────────────────────────────────────────────────────────────────────
//  nativeDestroy
// ─────────────────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeDestroy(
        JNIEnv* /*env*/, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);

    // Audio must be destroyed before the mutex is released — alDeleteSources
    // and alcDestroyContext must complete fully before the next nativeInit
    // could re-open the device.
    gAudioEngine.reset();

    gPipeline.reset();
    gGridAnalyzer.reset();
    gImuFusion.reset();
    gLastResult.reset();
    gLastGridResult.reset();

    LOGI("All native resources destroyed");
}

} // extern "C"