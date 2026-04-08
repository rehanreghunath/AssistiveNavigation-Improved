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
    // Issue 5 — Runtime focal length calibration:
    //   The original code hardcoded kFocalLengthPx = 500 px, a rough estimate
    //   for a 640×480 crop at ~65° HFOV.  A ±10% error in focal length
    //   produces a ±10% residual in the rotation compensation — noticeable as
    //   a ghost flow pattern on stationary scenes during head movement.
    //
    //   Kotlin now reads LENS_INFO_AVAILABLE_FOCAL_LENGTHS and
    //   SENSOR_INFO_PHYSICAL_SIZE from CameraCharacteristics and calls
    //   setFocalLength() once after nativeInit.  The formula is:
    //
    //     f_px = f_mm × (image_width_px / sensor_width_mm)
    //
    //   If CameraCharacteristics is unavailable (emulator, unit tests), the
    //   hardcoded fallback kFallbackFocalLengthPx = 500 is used automatically.
    //
    // Thread-safety contract:
    //   updateRotation() is called from the Android sensor thread.
    //   compensate()     is called from the camera executor thread.
    //   setFocalLength() is called once from the main thread before the first
    //                    camera frame, so no lock is needed — the caller must
    //                    ensure it races no camera frames.
    // ─────────────────────────────────────────────────────────────────────────

    class ImuFusion {
    public:
        ImuFusion(int width, int height);

        ~ImuFusion() = default;
        ImuFusion(const ImuFusion&)            = delete;
        ImuFusion& operator=(const ImuFusion&) = delete;

        /** Override the focal length used for the rotation homography.
         *  Call once from the main thread after nativeInit and before the first
         *  camera frame arrives.  f_px = f_mm × (image_width_px / sensor_width_mm).
         *  Values outside [100, 5000] are ignored as implausible. */
        void setFocalLength(float focalLengthPx);

        /** Store the latest device orientation quaternion [x, y, z, w].
         *  Called from the sensor thread; must be non-blocking. */
        void updateRotation(float qx, float qy, float qz, float qw,
                            int64_t timestampNs);

        /** Subtract rotation-induced displacement from every FlowVector
         *  in-place.  Recomputes magnitude and angle after adjustment.
         *  Called from the camera executor thread. */
        void compensate(FlowResult& result);

        /** Return the focal length currently in use (pixels). */
        float focalLengthPx() const { return mFocalLengthPx; }

    private:
        const int mWidth;
        const int mHeight;

        // Fallback focal length used until setFocalLength() is called.
        // Mobile rear cameras at 640×480 at ~65° HFOV → f ≈ 500 px.
        static constexpr float kFallbackFocalLengthPx = 500.0f;

        // Plausibility guard for setFocalLength().
        static constexpr float kMinPlausibleFx = 100.0f;
        static constexpr float kMaxPlausibleFx = 5000.0f;

        float mFocalLengthPx;   // set by setFocalLength(); fallback until then

        struct Quaternion {
            float x, y, z, w;
        };

        Quaternion           mSlot[2];
        std::atomic<int>     mReadIdx;
        std::atomic<bool>    mHasData;

        int64_t              mPrevTimestampNs;
        Quaternion           mPrevQuat;
        bool                 mHasPrev;

        static Quaternion multiplyQuat(const Quaternion& a, const Quaternion& b);
        static Quaternion conjugateQuat(const Quaternion& q);
        static void       quatToRotMat(const Quaternion& q, float out[9]);

        void predictedDisplacement(const float H[9],
                                   float px, float py,
                                   float cx, float cy,
                                   float& outDx, float& outDy) const;
    };

} // namespace assistivenav