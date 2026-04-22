#pragma once

#include "FlowTypes.h"

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <array>
#include <cstdint>

#ifndef AL_SOURCE_SPATIALIZE_SOFT
#define AL_SOURCE_SPATIALIZE_SOFT 0x1214
#endif

namespace assistivenav {

    // ─────────────────────────────────────────────────────────────────────────
    // AudioEngine
    //
    // Produces spatial audio cues from a list of temporally stable, classified
    // obstacle detections provided by ObstacleTracker.
    //
    // SIDE SELECTION — position, not flow direction:
    //   The previous design derived which ear to fire from cos(meanAngle) of
    //   the optical flow.  This was broken when the user turned: background
    //   objects produce rightward flow when the user turns left, so the "right"
    //   source fired for an obstacle actually on the user's left.
    //
    //   The fix: use obstacle.normX, the horizontal centroid of the detected
    //   blob in the camera frame.  An obstacle at normX = 0.2 is physically
    //   to the left of frame centre regardless of how it is moving.  This is
    //   immune to flow-direction inversion caused by head/body rotation.
    //
    //   Linear pan law: left gain  = confidence × (1 − normX)
    //                   right gain = confidence × normX
    //   A centre obstacle (normX ≈ 0.5) fires both ears equally, which is the
    //   correct percept for a hazard directly ahead.
    //
    // ROTATION SUPPRESSION — suppressionFactor from ImuFusion:
    //   AudioEngine receives a suppressionFactor ∈ [0, 1] computed from the
    //   angular velocity estimated by ImuFusion.  All target gains are
    //   multiplied by this factor, smoothly silencing the sources during fast
    //   user rotation.  This eliminates false triggers caused by head turning
    //   even when ImuFusion's flow compensation is imperfect.
    //
    //   suppressionFactor = 1 at low rotation (< kSoftRotThreshRad)
    //   suppressionFactor = 0 at fast rotation (> kHardRotThreshRad)
    //
    // TEMPORAL STABILITY:
    //   ObstacleTracker's 5-frame age gate and confidence threshold
    //   (kMinBlobAnomaly) ensure only persistent detections reach this class.
    //   No additional gating is needed inside AudioEngine; the conservative
    //   kMinConfidence threshold here is a second line of defence.
    //
    // PITCH:
    //   Modulated by obstacle.smoothedMag, which is the EMA-filtered
    //   anomaly-weighted flow magnitude.  Higher values indicate either a
    //   faster-moving or closer obstacle, both of which warrant more urgency.
    //
    // VERTICAL POSITION:
    //   Each source's Y position tracks obstacle.normY, giving HRTF a
    //   vertical cue so the user can distinguish a head-level obstacle from a
    //   floor-level one.
    //
    // SOURCE LAYOUT (2 sources):
    //   Source 0 — left  position (−kSideX, smoothedY₀, kSourceDepth)
    //   Source 1 — right position (+kSideX, smoothedY₁, kSourceDepth)
    // ─────────────────────────────────────────────────────────────────────────

    class AudioEngine {
    public:
        AudioEngine();
        ~AudioEngine();

        AudioEngine(const AudioEngine&)            = delete;
        AudioEngine& operator=(const AudioEngine&) = delete;

        /** PRIMARY audio update.
         *
         *  frame           — from ObstacleTracker::update(); only obstacles
         *                    with active=true and age≥kMinAgeForAudio reach
         *                    this method.
         *  suppressionFactor — ∈ [0,1]; 1 = no suppression, 0 = fully silent.
         *                    Computed by jni_bridge from ImuFusion::
         *                    rotationRateRadPerSec().  Scales all gains before
         *                    EMA smoothing so the fade-out is gradual.
         *
         *  Must not allocate heap memory (called from camera executor thread). */
        void updateFromObstacles(const ObstacleFrame& frame,
                                 float suppressionFactor);

        bool isReady() const { return mReady; }

    private:
        ALCdevice*  mDevice;
        ALCcontext* mContext;
        ALuint      mNoiseBuffer;

        static constexpr int kNumSources = 2;   // 0 = left, 1 = right
        std::array<ALuint, kNumSources> mSources;
        std::array<float,  kNumSources> mSmoothedGain;
        std::array<float,  kNumSources> mSmoothedPitch;
        // Vertical position of each source, EMA-tracked independently.
        // Allows the cue to convey whether the obstacle is at head,
        // chest, or floor height without the audio jumping discontinuously.
        std::array<float,  kNumSources> mSmoothedY;

        bool mReady;
        int  mDiagFrames;

        // ── Geometry ──────────────────────────────────────────────────────────
        // AL coordinate system: +Y = up, -Z = forward, listener at origin.
        static constexpr float kSourceDepth = -1.5f;
        static constexpr float kSideX       = 1.0f;

        // ── Confidence → gain mapping ─────────────────────────────────────────
        // Confidence is ObstacleTracker's EMA-smoothed mean anomaly score.
        // kMinConfidence matches ObstacleTracker::kMinBlobAnomaly so the
        // second-order gate is never looser than the first.
        static constexpr float kMinConfidence   = 0.30f;  // below → silent
        static constexpr float kFullConfidence  = 0.75f;  // above → max gain

        static constexpr float kMasterGain      = 0.65f;

        // EMA weight for gain updates (~4-frame window at 30 fps).
        static constexpr float kGainAlpha       = 0.18f;

        // EMA weight for vertical position tracking (deliberately slower than
        // gain so the Y cue settles smoothly even when only one obstacle is hot).
        static constexpr float kYAlpha          = 0.10f;

        // ── Pitch mapping (smoothedMag → urgency) ─────────────────────────────
        // smoothedMag is the anomaly-weighted flow magnitude (px/frame).
        // Higher value → obstacle is either closer or approaching faster.
        // This avoids a TTC-to-pitch conversion that could be undefined when
        // smoothedMag is near zero.
        static constexpr float kPitchMagLow     = 2.0f;   // slow/distant → kPitchLow
        static constexpr float kPitchMagHigh    = 12.0f;  // fast/close   → kPitchHigh
        static constexpr float kPitchLow        = 0.80f;
        static constexpr float kPitchHigh       = 1.40f;
        static constexpr float kPitchAlpha      = 0.15f;

        // ── Noise buffer ──────────────────────────────────────────────────────
        static constexpr int   kSampleRate  = 44100;
        static constexpr int   kNoiseFrames = 44100;
        static constexpr float kHpAlpha     = 0.9715f;

        void initOpenAL();
        void generateNoiseBuffer();
        void initSources();
        void shutdownOpenAL();
    };

} // namespace assistivenav