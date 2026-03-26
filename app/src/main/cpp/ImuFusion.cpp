#include "ImuFusion.h"
#include <android/log.h>
#include <cmath>
#include <algorithm>

#define LOG_TAG "ImuFusion"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    ImuFusion::ImuFusion(int width, int height)
            : mWidth(width),
              mHeight(height),
              mReadIdx(0),
              mHasData(false),
              mPrevTimestampNs(0),
              mHasPrev(false)
    {
        // Zero-init both slots so there is never uninitialised data in them.
        mSlot[0] = {0.0f, 0.0f, 0.0f, 1.0f}; // identity quaternion
        mSlot[1] = {0.0f, 0.0f, 0.0f, 1.0f};
        mPrevQuat = {0.0f, 0.0f, 0.0f, 1.0f};

        LOGI("Created for %dx%d, focal length approx %.0f px",
             mWidth, mHeight, kFocalLengthPx);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Quaternion helpers  (static, no member state)
    // ─────────────────────────────────────────────────────────────────────────

    ImuFusion::Quaternion ImuFusion::multiplyQuat(const Quaternion& a,
                                                  const Quaternion& b) {
        // Hamilton product: a ⊗ b
        //   w = aw·bw − ax·bx − ay·by − az·bz
        //   x = aw·bx + ax·bw + ay·bz − az·by
        //   y = aw·by − ax·bz + ay·bw + az·bx
        //   z = aw·bz + ax·by − ay·bx + az·bw
        return {
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        };
    }

    ImuFusion::Quaternion ImuFusion::conjugateQuat(const Quaternion& q) {
        // For a unit quaternion conjugate == inverse: negate the vector part.
        return {-q.x, -q.y, -q.z, q.w};
    }

    void ImuFusion::quatToRotMat(const Quaternion& q, float out[9]) {
        // Standard quaternion → rotation matrix conversion.
        // Indices: out[row*3 + col]
        const float x = q.x, y = q.y, z = q.z, w = q.w;

        const float xx = x * x, yy = y * y, zz = z * z;
        const float xy = x * y, xz = x * z, yz = y * z;
        const float wx = w * x, wy = w * y, wz = w * z;

        out[0] = 1.0f - 2.0f * (yy + zz);
        out[1] =        2.0f * (xy - wz);
        out[2] =        2.0f * (xz + wy);

        out[3] =        2.0f * (xy + wz);
        out[4] = 1.0f - 2.0f * (xx + zz);
        out[5] =        2.0f * (yz - wx);

        out[6] =        2.0f * (xz - wy);
        out[7] =        2.0f * (yz + wx);
        out[8] = 1.0f - 2.0f * (xx + yy);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  predictedDisplacement
    //
    //  Computes the pixel displacement that a pure camera rotation R would
    //  induce at point (px, py), given the homography H = K·R·K⁻¹.
    //
    //  K     = [ f  0  cx ]      K⁻¹ = [ 1/f   0  -cx/f ]
    //           [ 0  f  cy ]              [   0 1/f -cy/f ]
    //           [ 0  0   1 ]              [   0   0     1 ]
    //
    //  H·p gives the homogeneous coordinates of where (px,py) would land
    //  under a pure rotation.  The difference is the rotation-induced flow.
    // ─────────────────────────────────────────────────────────────────────────

    void ImuFusion::predictedDisplacement(const float H[9],
                                          float px, float py,
                                          float cx, float cy,
                                          float& outDx, float& outDy) const {
        // Apply H to homogeneous point [px, py, 1]ᵀ
        const float hx = H[0] * px + H[1] * py + H[2];
        const float hy = H[3] * px + H[4] * py + H[5];
        const float hw = H[6] * px + H[7] * py + H[8];

        // Guard against degenerate case (hw should be ~1 for small rotations).
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
        // Normalise to guard against sensor drift producing a non-unit quaternion.
        const float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if (norm < 1e-6f) return; // degenerate reading; discard

        const float invN = 1.0f / norm;

        // Write into the slot that the camera thread is NOT currently reading.
        const int writeSlot = 1 - mReadIdx.load(std::memory_order_relaxed);
        mSlot[writeSlot] = {qx * invN, qy * invN, qz * invN, qw * invN};

        // Flip the read index so the camera thread picks up the new value.
        mReadIdx.store(writeSlot, std::memory_order_release);
        mHasData.store(true, std::memory_order_release);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  compensate  — camera executor thread
    // ─────────────────────────────────────────────────────────────────────────

    void ImuFusion::compensate(FlowResult& result) {
        // Nothing to do until at least two sensor readings have arrived.
        if (!mHasData.load(std::memory_order_acquire)) return;

        const Quaternion currQuat =
                mSlot[mReadIdx.load(std::memory_order_acquire)];

        if (!mHasPrev) {
            // First camera frame after sensor data arrived.  Store the current
            // orientation as the baseline and return without modifying vectors —
            // there is no previous frame to compute a delta from.
            mPrevQuat = currQuat;
            mHasPrev  = true;
            return;
        }

        // ── Delta quaternion ──────────────────────────────────────────────────
        // q_delta represents the rotation the device underwent between the
        // previous and current camera frame.
        //
        //   q_delta = q_curr ⊗ q_prev*
        //
        // where q_prev* is the conjugate (= inverse) of q_prev.
        const Quaternion qDelta = multiplyQuat(currQuat, conjugateQuat(mPrevQuat));

        // If the delta is nearly identity (|angle| < 0.05°), skip the per-vector
        // loop entirely — the homography would be the identity matrix and the
        // result would be numerical noise, not a correction.
        // The w component of a unit quaternion is cos(θ/2); θ < 0.05° ↔ w > 0.9999996.
        if (qDelta.w > 0.9999990f) {
            mPrevQuat = currQuat;
            return;
        }

        // ── Rotation matrix and homography ────────────────────────────────────
        float R[9];
        quatToRotMat(qDelta, R);

        // H = K · R · K⁻¹, expanded analytically to avoid 3×3 matrix multiply
        // overhead in a per-frame call.  All intermediate values are on the stack.
        //
        // K   = diag(f, f, 1) with principal point (cx, cy)
        // K⁻¹ = diag(1/f, 1/f, 1) with principal point (-cx/f, -cy/f)
        //
        // The product K·R·K⁻¹ for a camera with fx=fy=f simplifies to:
        //
        // H[0,0] = R00 + (R02 - R00·cx)·0  ... easiest to just compute K*R*Kinv directly.

        const float f    = kFocalLengthPx;
        const float cx   = static_cast<float>(mWidth)  * 0.5f;
        const float cy   = static_cast<float>(mHeight) * 0.5f;
        const float invF = 1.0f / f;

        // R · K⁻¹  (right-multiply by K⁻¹)
        // Column j of (R·K⁻¹) = R · (column j of K⁻¹)
        // K⁻¹ columns: [1/f, 0, 0]ᵀ, [0, 1/f, 0]ᵀ, [-cx/f, -cy/f, 1]ᵀ
        float RKinv[9];
        for (int row = 0; row < 3; ++row) {
            RKinv[row*3 + 0] = R[row*3 + 0] * invF;
            RKinv[row*3 + 1] = R[row*3 + 1] * invF;
            RKinv[row*3 + 2] = -R[row*3 + 0] * cx * invF
                               - R[row*3 + 1] * cy * invF
                               +  R[row*3 + 2];
        }

        // K · (R · K⁻¹)
        float H[9];
        for (int col = 0; col < 3; ++col) {
            H[0*3 + col] = f   * RKinv[0*3 + col] + cx * RKinv[2*3 + col];
            H[1*3 + col] = f   * RKinv[1*3 + col] + cy * RKinv[2*3 + col];
            H[2*3 + col] =                                RKinv[2*3 + col];
        }

        // ── Per-vector compensation ───────────────────────────────────────────
        float magSum = 0.0f;

        for (FlowVector& fv : result.vectors) {
            float predDx, predDy;
            predictedDisplacement(H, fv.x0, fv.y0, cx, cy, predDx, predDy);

            fv.dx -= predDx;
            fv.dy -= predDy;
            fv.magnitude  = std::sqrt(fv.dx * fv.dx + fv.dy * fv.dy);
            fv.angle      = std::atan2(fv.dy, fv.dx);
            magSum       += fv.magnitude;
        }

        // Keep globalMeanMag consistent with the compensated vectors.
        result.globalMeanMag = result.trackedCount > 0
                               ? magSum / static_cast<float>(result.trackedCount)
                               : 0.0f;

        mPrevQuat = currQuat;
    }

} // namespace assistivenav