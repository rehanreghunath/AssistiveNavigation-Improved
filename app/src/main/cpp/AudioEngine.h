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
    // Produces spatial audio cues driven by LATERAL flow in the centre column
    // of the 3×3 grid (landscape cells 1, 4, 7).
    //
    // WHY LATERAL-ONLY:
    //   Forward obstacles produce radially-outward flow in the centre column.
    //   For cells 1 and 7, that flow is mostly vertical; for cell 4, the mean
    //   direction is undefined.  In all cases cos(meanAngle) ≈ 0 — the lateral
    //   component is near zero — so audio stays silent for a straight-on wall.
    //   An object crossing from the centre zone toward the left or right
    //   produces strong lateral flow (cos(angle) → ±1), which is the only
    //   condition that fires a source.
    //
    // SOURCE LAYOUT (2 sources):
    //   Source 0 — left side   position (-kSideX, smoothedY, kSourceDepth)
    //   Source 1 — right side  position (+kSideX, smoothedY, kSourceDepth)
    //
    //   The Y position of each source tracks the weighted centroid of whichever
    //   centre rows are contributing the most lateral signal.  This lets the
    //   user distinguish a head-height obstacle from a floor-level one.
    //
    // GAIN:
    //   lateral_strength = Σ over centre cells of (lateralComponent * dangerScore)
    //   gain = clamp((strength - kMinLateral) / (kFullLateral - kMinLateral)) * kMasterGain
    //
    // PITCH:
    //   Modulated by TTC from the dominant contributing cell.
    //   Low TTC (imminent) → high pitch; high TTC (distant) → low pitch.
    // ─────────────────────────────────────────────────────────────────────────

    class AudioEngine {
    public:
        AudioEngine();
        ~AudioEngine();

        AudioEngine(const AudioEngine&)            = delete;
        AudioEngine& operator=(const AudioEngine&) = delete;

        /** PRIMARY audio update — lateral flow from center-column cells drives
         *  left/right spatial sources.  Called every frame from camera thread.
         *  Must not allocate heap memory. */
        void updateFromGrid(const GridResult& grid);

        /** No-op stub kept so the ObstacleTracker path in jni_bridge compiles
         *  without restructuring; its output is not used for audio. */
        void update(const ObstacleFrame& /*frame*/) {}

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
        // Allows the cue to convey whether the crossing object is at head,
        // chest, or floor height without the audio jumping discontinuously.
        std::array<float,  kNumSources> mSmoothedY;

        bool mReady;
        int  mDiagFrames;

        // ── Geometry ──────────────────────────────────────────────────────────
        // AL coordinate system: +Y = up, -Z = forward, listener at origin.
        // kSideX places the source clearly to the side but not so far that
        // HRTF externalisation fails (HRTF works best at ±90° ≈ ±1 unit).
        static constexpr float kSourceDepth = -1.5f;
        static constexpr float kSideX       = 1.0f;

        // Y positions for the three centre rows (top = head, bot = floor).
        static constexpr float kRowY[3] = { 0.5f, 0.0f, -0.5f };

        // ── Lateral flow thresholds ───────────────────────────────────────────

        // Minimum fractional lateral component (|cos(angle)|) required before
        // a cell contributes to the lateral signal.  Suppresses cells where
        // flow is primarily vertical (approaching wall, floor texture).
        static constexpr float kLateralThresh = 0.35f;

        // Minimum cell sample count for a reliable meanAngle estimate.
        static constexpr int   kMinSamples    = 3;

        // ── Gain mapping ──────────────────────────────────────────────────────

        // Lateral strength below this → silence.
        // strength = Σ(lateralComp * dangerScore) over centre cells.
        static constexpr float kMinLateral  = 0.06f;

        // Lateral strength at which gain reaches kMasterGain.
        static constexpr float kFullLateral = 0.42f;

        static constexpr float kMasterGain  = 0.65f;

        // EMA weight for gain updates (~4-frame window at 30 fps).
        // Fast enough to track a rapidly crossing object; slow enough to
        // prevent clicks from frame-to-frame dangerScore fluctuation.
        static constexpr float kGainAlpha   = 0.18f;

        // EMA weight for vertical position tracking.
        // Deliberately slower than gain so the Y position settles smoothly
        // even when only one row is hot at a time.
        static constexpr float kYAlpha      = 0.10f;

        // ── Pitch mapping (TTC → urgency) ─────────────────────────────────────
        static constexpr float kPitchLow           = 0.80f;
        static constexpr float kPitchHigh          = 1.40f;
        static constexpr float kMinTTCForHighPitch  = 10.0f;
        static constexpr float kMaxTTCForPitch      = 80.0f;
        static constexpr float kPitchAlpha          = 0.15f;

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