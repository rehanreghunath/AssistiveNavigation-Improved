#pragma once

#include "FlowTypes.h"
#include <cstdint>
#include <atomic>

namespace assistivenav {

    // ─────────────────────────────────────────────────────────────────────────
    // ImuFusion
    //
    // Receives Android rotation-vector sensor readings (quaternion) from the
    // sensor thread and uses them to remove the camera-rotation contribution
    // from each optical-flow vector before grid analysis.
    //
    // Why this matters: when the user turns their head or sways while walking,
    // every single tracked point shifts in the same direction.  Without
    // compensation the grid analyser would score every cell as dangerous — a
    // false alarm on every stride.  ImuFusion subtracts the displacement that
    // a pure camera rotation would have caused, so only vectors caused by
    // independently moving objects (or forward translation) remain.
    //
    // Thread-safety contract:
    //   updateRotation() is called from the Android sensor thread.
    //   compensate()     is called from the camera executor thread.
    //   The two share no mutable state — updateRotation writes to an atomic
    //   slot and compensate reads from it with a relaxed load.  No mutex
    //   needed; a slightly stale quaternion (one sensor sample old) has no
    //   perceptible effect on a 30 fps pipeline.
    // ─────────────────────────────────────────────────────────────────────────

    class ImuFusion {
    public:
        ImuFusion(int width, int height);

        ~ImuFusion() = default;
        ImuFusion(const ImuFusion&)            = delete;
        ImuFusion& operator=(const ImuFusion&) = delete;

        /** Store the latest device orientation quaternion [x, y, z, w].
         *  Called from the sensor thread; must be non-blocking. */
        void updateRotation(float qx, float qy, float qz, float qw,
                            int64_t timestampNs);

        /** Subtract rotation-induced displacement from every FlowVector
         *  in-place.  Recomputes magnitude and angle after adjustment.
         *  Called from the camera executor thread. */
        void compensate(FlowResult& result);

    private:
        const int mWidth;
        const int mHeight;

        // Approximate camera intrinsics.
        // Mobile rear cameras at 640×480 typically have a horizontal FOV of
        // 60–70°.  f = w / (2·tan(FOV/2)).  At 65° → f ≈ 512.  We use 500
        // as a round number; a ±10 % error here produces only a ±10 % residual
        // in the compensation, which is acceptable without full calibration.
        static constexpr float kFocalLengthPx = 500.0f;

        // A POD quaternion small enough to fit in two cache lines and simple
        // enough to copy without a mutex.
        struct Quaternion {
            float x, y, z, w;
        };

        // Two-slot "seqlock-lite": the camera thread always reads slot[readIdx],
        // the sensor thread always writes slot[1 - readIdx] then flips the index.
        // In practice a simple flag is fine here because one stale frame is
        // imperceptible, but the two-slot design makes the intent explicit.
        Quaternion           mSlot[2];
        std::atomic<int>     mReadIdx;   // which slot the camera thread should read
        std::atomic<bool>    mHasData;   // false until the first sensor event arrives

        int64_t              mPrevTimestampNs;
        Quaternion           mPrevQuat;
        bool                 mHasPrev;

        /** Multiply two quaternions: result = a * b. */
        static Quaternion multiplyQuat(const Quaternion& a, const Quaternion& b);

        /** Conjugate (= inverse for a unit quaternion): flip the vector part. */
        static Quaternion conjugateQuat(const Quaternion& q);

        /** Convert a unit quaternion to a 3×3 rotation matrix stored row-major. */
        static void quatToRotMat(const Quaternion& q, float out[9]);

        /** Apply homography H = K·R·K⁻¹ to point (px,py) and return the
         *  predicted displacement. */
        void predictedDisplacement(const float H[9],
                                   float px, float py,
                                   float cx, float cy,
                                   float& outDx, float& outDy) const;
    };

} // namespace assistivenav