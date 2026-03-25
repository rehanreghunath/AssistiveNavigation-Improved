#include <jni.h>
#include <android/log.h>
#include <memory>
#include <mutex>
#include "FlowEngine.h"
#include "FlowRenderer.h"

#define LOG_TAG "JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// ── Globals ───────────────────────────────────────────────────────────────────
// gPipeline and gRenderer are separate objects with separate lifetimes:
//   gPipeline: created on first camera frame, destroyed on STOP
//   gRenderer: created on GL surface creation, destroyed on STOP
// They share data through gRenderer->submitFlowResult() which is thread-safe.

static std::unique_ptr<assistivenav::FlowEngine> gPipeline;
static std::unique_ptr<assistivenav::FlowRenderer> gRenderer;
static std::unique_ptr<assistivenav::FlowResult> gLastResult;

// Protects gPipeline. gRenderer needs no mutex of its own because:
//   - onSurfaceCreated / onSurfaceChanged / onDrawFrame are all called from
//     the same GL thread (GLSurfaceView guarantees this)
//   - submitFlowResult() has its own internal double-buffer mutex
static std::mutex gPipelineMutex;

extern "C" {

// ─── Pipeline: nativeInit ────────────────────────────────────────────────────
JNIEXPORT jbyteArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeGetRgbaFrame(
        JNIEnv* env, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (!gPipeline || !gLastResult) return nullptr;

    const auto rgba = gPipeline->renderToRgba(*gLastResult);

    jbyteArray out = env->NewByteArray(static_cast<jsize>(rgba.size()));
    if (out) {
        env->SetByteArrayRegion(out, 0, static_cast<jsize>(rgba.size()),
                                reinterpret_cast<const jbyte*>(rgba.data()));
    }
    return out;
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeSetRenderRotation(
        JNIEnv* /*env*/, jobject /*thiz*/, jint degrees) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (gPipeline) gPipeline->setRenderRotation(static_cast<int>(degrees));
}
JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeInit(
        JNIEnv* /*env*/, jobject /*thiz*/,
        jint width, jint height) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    gPipeline = std::make_unique<assistivenav::FlowEngine>(
            static_cast<int>(width),
            static_cast<int>(height));
    LOGI("Pipeline init %dx%d", (int)width, (int)height);
}

// ─── Pipeline: nativeProcessFrame ────────────────────────────────────────────

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
    gLastResult = std::make_unique<assistivenav::FlowResult>(result);

    if (gRenderer && !result.isFirstFrame) {
        gRenderer->setFrameSize(static_cast<int>(width), static_cast<int>(height));
        gRenderer->submitFlowResult(result);
    }

    // Return minimal stats to Kotlin for the HUD.
    // The full vector data goes to the renderer directly above -> no need to
    // serialize and deserialize it through JNI again.
    const float stats[3] = {
            static_cast<float>(result.trackedCount),
            static_cast<float>(result.totalPts),
            result.globalMeanMag
    };
    jfloatArray out = env->NewFloatArray(3);
    if (out) env->SetFloatArrayRegion(out, 0, 3, stats);
    return out;
}

// ─── Pipeline: nativeDestroy ─────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeDestroy(
        JNIEnv* /*env*/, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    gPipeline.reset();
    gRenderer.reset();
    LOGI("Pipeline and renderer destroyed");
}

// ─── Renderer: nativeSurfaceCreated ──────────────────────────────────────────
// Called on the GL thread. All GL object creation happens here.

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeSurfaceCreated(
        JNIEnv* /*env*/, jobject /*thiz*/) {
    // Create FlowRenderer here on the GL thread because the EGL context
    // is only current on this thread. All subsequent GL calls must also
    // come from this thread (GLSurfaceView guarantees this for the Renderer
    // callbacks).
    if (!gRenderer) {
        gRenderer = std::make_unique<assistivenav::FlowRenderer>();
    }
    gRenderer->onSurfaceCreated();
}

// ─── Renderer: nativeSurfaceChanged ──────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeSurfaceChanged(
        JNIEnv* /*env*/, jobject /*thiz*/,
        jint width, jint height) {
    if (gRenderer) {
        gRenderer->onSurfaceChanged(
                static_cast<int>(width),
                static_cast<int>(height));
    }
}

// ─── Renderer: nativeDrawFrame ────────────────────────────────────────────────
// Called on the GL thread every display frame (~60 Hz).
// Returns [fps, trackedCount] as a float array for the Kotlin HUD.

JNIEXPORT jfloatArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeDrawFrame(
        JNIEnv* env, jobject /*thiz*/) {
    if (!gRenderer) return nullptr;

    const auto stats = gRenderer->onDrawFrame();

    const float arr[2] = {
            stats.fps,
            static_cast<float>(stats.trackedCount)
    };
    jfloatArray out = env->NewFloatArray(2);
    if (out) env->SetFloatArrayRegion(out, 0, 2, arr);
    return out;
}

}