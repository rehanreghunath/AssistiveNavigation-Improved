#include "ImuFusion.h"
#include <android/log.h>
#include <cmath>
#include <algorithm>

#define LOG_TAG "ImuFusion"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    ImuFusion::ImuFusion(int width, int height)
            : mWidth(width),
              mHeight(height),
              mFocalLengthPx(kFallbackFocalLengthPx),
              mReadIdx(0),
              mHasData(false),
              mPrevTimestampNs(0),
              mHasPrev(false)
    {
        mSlot[0] = {0.0f, 0.0f, 0.0f, 1.0f};
        mSlot[1] = {0.0f, 0.0f, 0.0f, 1.0f};
        mPrevQuat = {0.0f, 0.0f, 0.0f, 1.0f};

        LOGI("Created for %dx%d, focal length %.0f px (fallback — call setFocalLength to override)",
             mWidth, mHeight, mFocalLengthPx);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  setFocalLength  — issue 5
    //
    //  Replaces the compile-time constant with a value computed from real
    //  CameraCharacteristics data:
    //    f_px = f_mm × (image_width_px / sensor_width_mm)
    //
    //  This should be called once from the main thread after nativeInit, before
    //  the camera stream begins.  Race safety: the camera executor thread has
    //  not started yet when this is called.
    // ─────────────────────────────────────────────────────────────────────────

    void ImuFusion::setFocalLength(float focalLengthPx) {
        if (focalLengthPx < kMinPlausibleFx || focalLengthPx > kMaxPlausibleFx) {
            LOGW("setFocalLength: %.1f px is outside plausible range [%.0f, %.0f]; "
                 "keeping fallback %.0f px",
                 focalLengthPx, kMinPlausibleFx, kMaxPlausibleFx, mFocalLengthPx);
            return;
        }
        mFocalLengthPx = focalLengthPx;
        LOGI("Focal length set to %.1f px (was %.1f px fallback)",
             mFocalLengthPx, kFallbackFocalLengthPx);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Quaternion helpers
    // ─────────────────────────────────────────────────────────────────────────

    ImuFusion::Quaternion ImuFusion::multiplyQuat(const Quaternion& a,
                                                  const Quaternion& b) {
        return {
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        };
    }

    ImuFusion::Quaternion ImuFusion::conjugateQuat(const Quaternion& q) {
        return {-q.x, -q.y, -q.z, q.w};
    }

    void ImuFusion::quatToRotMat(const Quaternion& q, float out[9]) {
        const float x = q.x, y = q.y, z = q.z, w = q.w;
        const float xx = x*x, yy = y*y, zz = z*z;
        const float xy = x*y, xz = x*z, yz = y*z;
        const float wx = w*x, wy = w*y, wz = w*z;

        out[0] = 1.0f - 2.0f*(yy+zz); out[1] = 2.0f*(xy-wz);        out[2] = 2.0f*(xz+wy);
        out[3] = 2.0f*(xy+wz);         out[4] = 1.0f - 2.0f*(xx+zz); out[5] = 2.0f*(yz-wx);
        out[6] = 2.0f*(xz-wy);         out[7] = 2.0f*(yz+wx);        out[8] = 1.0f - 2.0f*(xx+yy);
    }

    void ImuFusion::predictedDisplacement(const float H[9],
                                          float px, float py,
                                          float cx, float cy,
                                          float& outDx, float& outDy) const {
        const float hx = H[0]*px + H[1]*py + H[2];
        const float hy = H[3]*px + H[4]*py + H[5];
        const float hw = H[6]*px + H[7]*py + H[8];

        if (std::abs(hw) < 1e-6f) {
            outDx = outDy = 0.0f;
            return;
        }
        const float invW = 1.0f / hw;
        outDx = hx * invW - px;
        outDy = hy * invW - py;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  updateRotation  — sensor thread
    // ─────────────────────────────────────────────────────────────────────────

    void ImuFusion::updateRotation(float qx, float qy, float qz, float qw,
                                   int64_t /* timestampNs */) {
        const float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if (norm < 1e-6f) return;

        const float invN     = 1.0f / norm;
        const int   writeSlot = 1 - mReadIdx.load(std::memory_order_relaxed);
        mSlot[writeSlot] = {qx*invN, qy*invN, qz*invN, qw*invN};
        mReadIdx.store(writeSlot, std::memory_order_release);
        mHasData.store(true,      std::memory_order_release);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  compensate  — camera executor thread
    //
    //  Uses mFocalLengthPx (set at startup from CameraCharacteristics) instead
    //  of the old compile-time constant kFocalLengthPx = 500.  All arithmetic
    //  is otherwise identical to the previous implementation.
    // ─────────────────────────────────────────────────────────────────────────

    void ImuFusion::compensate(FlowResult& result) {
        if (!mHasData.load(std::memory_order_acquire)) return;

        const Quaternion currQuat =
                mSlot[mReadIdx.load(std::memory_order_acquire)];

        if (!mHasPrev) {
            mPrevQuat = currQuat;
            mHasPrev  = true;
            return;
        }

        const Quaternion qDelta = multiplyQuat(currQuat, conjugateQuat(mPrevQuat));

        // w > 0.9999990 ↔ |rotation angle| < 0.05°.  Skip — pure numerical noise.
        if (qDelta.w > 0.9999990f) {
            mPrevQuat = currQuat;
            return;
        }

        float R[9];
        quatToRotMat(qDelta, R);

        // H = K · R · K⁻¹, analytically expanded.
        // Now uses mFocalLengthPx (runtime-calibrated) instead of a constant.
        const float f    = mFocalLengthPx;
        const float cx   = static_cast<float>(mWidth)  * 0.5f;
        const float cy   = static_cast<float>(mHeight) * 0.5f;
        const float invF = 1.0f / f;

        // R · K⁻¹
        float RKinv[9];
        for (int row = 0; row < 3; ++row) {
            RKinv[row*3 + 0] =  R[row*3 + 0] * invF;
            RKinv[row*3 + 1] =  R[row*3 + 1] * invF;
            RKinv[row*3 + 2] = -R[row*3 + 0] * cx * invF
                               - R[row*3 + 1] * cy * invF
                               + R[row*3 + 2];
        }

        // K · (R · K⁻¹)
        float H[9];
        for (int col = 0; col < 3; ++col) {
            H[0*3 + col] = f   * RKinv[0*3 + col] + cx * RKinv[2*3 + col];
            H[1*3 + col] = f   * RKinv[1*3 + col] + cy * RKinv[2*3 + col];
            H[2*3 + col] =                                RKinv[2*3 + col];
        }

        float magSum = 0.0f;

        for (FlowVector& fv : result.vectors) {
            float predDx, predDy;
            predictedDisplacement(H, fv.x0, fv.y0, cx, cy, predDx, predDy);

            fv.dx       -= predDx;
            fv.dy       -= predDy;
            fv.magnitude = std::sqrt(fv.dx * fv.dx + fv.dy * fv.dy);
            fv.angle     = std::atan2(fv.dy, fv.dx);
            magSum      += fv.magnitude;
        }

        result.globalMeanMag = result.trackedCount > 0
                               ? magSum / static_cast<float>(result.trackedCount)
                               : 0.0f;

        mPrevQuat = currQuat;
    }

} // namespace assistivenav