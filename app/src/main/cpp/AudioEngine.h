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
    // Drives kMaxObstacles OpenAL sources from GridAnalyzer output.
    //
    // PREVIOUS DESIGN (broken):
    //   update(ObstacleFrame) — audio gated on FlowClassifier anomaly scores.
    //   Problem A: grid dangerScore was never involved in audio decisions.
    //   Problem B: forward approach = radially outward flow = low anomaly = silent.
    //   Problem C: retreat = magnitude ratio triggered = loud. Backwards.
    //
    // NEW DESIGN:
    //   updateFromGrid(GridResult) — audio driven directly from the 3×3 grid.
    //
    //   Portrait mode has the phone chest-mounted pointing forward.
    //   The forward path maps to the CENTER COLUMN: cells 1 (top), 4 (mid), 7 (bot).
    //   Each of these cells gets its own audio source.  The source is panned
    //   left/right by the COLUMN it is in (always centre here), and raised/lowered
    //   by ROW.  Gain is driven by dangerScore, which is magnitude-based and
    //   symmetric — it rises whether you approach or retreat.
    //
    //   TTC (time-to-collision) is used to modulate the source pitch:
    //     TTC large (far away / slow)  →  pitch 0.8  (low rumble)
    //     TTC small (close / fast)     →  pitch 1.4  (urgent tone)
    //   This gives an intuitive "getting closer" feel without changing volume
    //   discontinuously.
    //
    //   All three cells (sources 0/1/2) can be active simultaneously, so the
    //   user hears whether the obstacle is at head height, chest, or floor level.
    //
    //   The peripheral columns (0,2 / 3,5 / 6,8) feed an optional side-warning
    //   source (source 3) that only activates when they have higher danger than
    //   the centre.  This warns of a doorframe or pole to the side without
    //   masking the forward path signal.
    //
    //   The old update(ObstacleFrame) is kept as a no-op stub so the
    //   ObstacleTracker can still run for future classification work without
    //   touching the audio path.
    // ─────────────────────────────────────────────────────────────────────────

    class AudioEngine {
    public:
        AudioEngine();
        ~AudioEngine();

        AudioEngine(const AudioEngine&)            = delete;
        AudioEngine& operator=(const AudioEngine&) = delete;

        /** PRIMARY audio update — driven directly from GridAnalyzer output.
         *  Call every frame from the camera executor thread.
         *  Must not allocate heap memory. */
        void updateFromGrid(const GridResult& grid);

        /** Legacy stub — kept so ObstacleTracker can still be called upstream
         *  without restructuring jni_bridge.  Does nothing. */
        void update(const ObstacleFrame& /*frame*/) {}

        bool isReady() const { return mReady; }

    private:
        ALCdevice*  mDevice;
        ALCcontext* mContext;
        ALuint      mNoiseBuffer;

        // Source layout:
        //   0 — centre-top    (cell 1)
        //   1 — centre-mid    (cell 4)  ← primary danger
        //   2 — centre-bottom (cell 7)  ← primary danger
        //   3 — side warning  (max of left/right columns at highest danger row)
        static constexpr int kNumSources = 4;
        std::array<ALuint, kNumSources> mSources;
        std::array<float,  kNumSources> mSmoothedGain;
        std::array<float,  kNumSources> mSmoothedPitch;

        bool mReady;
        int  mDiagFrames;

        // ── Geometry ──────────────────────────────────────────────────────────
        // Portrait: phone held vertically, chest-mounted.
        // Row 0 = top of frame = head-level,  row 2 = bottom = floor-level.
        // AL: +Y = up, -Z = forward.  Fixed depth; gain encodes proximity.
        static constexpr float kSourceDepth = -1.5f;

        // Vertical positions for each row (AL Y axis, 0=centre=chest).
        static constexpr float kRowY[3] = { 0.5f, 0.0f, -0.5f };

        // Side-warning source: pan hard left (-1) or right (+1) at chest height.
        // Which side is updated in updateFromGrid() based on which column is hotter.
        static constexpr float kSideX = 1.0f;

        // ── Gain / pitch tuning ───────────────────────────────────────────────

        // dangerScore threshold before any sound is produced.
        // 0.15 keeps the pipeline silent during normal walking with minor flow.
        static constexpr float kMinDanger         = 0.15f;

        // dangerScore at which the source reaches full gain.
        static constexpr float kFullDanger        = 0.65f;

        // Maximum gain per source.  Three centre sources at kMasterGain each
        // sum to 3 × 0.55 = 1.65, still well below 0 dBFS with HRTF.
        static constexpr float kMasterGain        = 0.55f;

        // Side-warning gain is lower — it is an advisory, not the primary signal.
        static constexpr float kSideGain          = 0.35f;

        // EMA weight for gain smoothing.  0.20 = ~4-frame window at 30 fps.
        // Fast enough to track approach; slow enough to avoid clicks.
        static constexpr float kGainAlpha         = 0.20f;

        // Pitch range: maps TTC → [kPitchLow, kPitchHigh].
        // TTC = 0 (no motion) → kPitchLow.
        // TTC small (imminent) → kPitchHigh.
        static constexpr float kPitchLow          = 0.80f;
        static constexpr float kPitchHigh         = 1.40f;
        // TTC (frames) that maps to kPitchHigh.  TTC = kTTCScale/mag; at
        // kTTCScale=150 and mag=5, TTC=30 frames ≈ 1 second.  We saturate
        // pitch at TTC ≤ kMinTTCForHighPitch.
        static constexpr float kMinTTCForHighPitch = 10.0f;   // ~0.33 s at 30 fps
        static constexpr float kMaxTTCForPitch     = 80.0f;   // beyond this = low pitch

        // EMA weight for pitch smoothing.
        static constexpr float kPitchAlpha        = 0.15f;

        static constexpr int   kSampleRate  = 44100;
        static constexpr int   kNoiseFrames = 44100;
        static constexpr float kHpAlpha     = 0.9715f;

        void initOpenAL();
        void generateNoiseBuffer();
        void initSources();
        void shutdownOpenAL();
    };

} // namespace assistivenav