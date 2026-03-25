#include <jni.h>
#include <android/log.h>
#include <memory>
#include <mutex>
#include "FlowEngine.h"

#define LOG_TAG "JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static std::unique_ptr<assistivenav::FlowEngine> gPipeline;
static std::unique_ptr<assistivenav::FlowResult> gLastResult;
static std::mutex gPipelineMutex;

extern "C" {

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

    const float stats[3] = {
            static_cast<float>(result.trackedCount),
            static_cast<float>(result.totalPts),
            result.globalMeanMag
    };
    jfloatArray out = env->NewFloatArray(3);
    if (out) env->SetFloatArrayRegion(out, 0, 3, stats);
    return out;
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeDestroy(
        JNIEnv* /*env*/, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    gPipeline.reset();
    LOGI("Pipeline and renderer destroyed");
}

} // extern "C"