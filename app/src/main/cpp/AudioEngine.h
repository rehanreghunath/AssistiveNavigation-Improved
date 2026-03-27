#pragma once

#include "FlowTypes.h"

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <array>
#include <chrono>
#include <cstdint>

#ifndef AL_SOURCE_SPATIALIZE_SOFT
#define AL_SOURCE_SPATIALIZE_SOFT 0x1214
#endif

namespace assistivenav {

    // ─────────────────────────────────────────────────────────────────────────
    // AudioEngine
    //
    // Owns kMaxObstacles OpenAL sources — one per tracked obstacle slot.
    // Each source:
    //   • plays a shared looping high-pass white-noise buffer continuously
    //   • is positioned in 3-D space derived from the obstacle's normalised
    //     image coordinates, giving HRTF-encoded ITD, ILD, and pinna coloration
    //   • has its gain driven by the obstacle's smoothed flow magnitude —
    //     louder = faster-approaching / larger object
    //
    // Audio is continuous (no pulsing) for confirmed obstacles.  Continuity is
    // the cue: sound present = obstacle present, silence = clear.  The gain
    // already encodes proximity; adding modulation on top would increase
    // cognitive load without adding information.
    //
    // Sources are started at init time at gain = 0 to eliminate the ~10 ms
    // startup latency that alSourcePlay would introduce on first detection.
    //
    // Thread-safety: update() is called from the camera executor thread.
    // Constructor and destructor are called from the main thread.
    // ─────────────────────────────────────────────────────────────────────────

    class AudioEngine {
    public:
        AudioEngine();
        ~AudioEngine();

        AudioEngine(const AudioEngine&)            = delete;
        AudioEngine& operator=(const AudioEngine&) = delete;

        /** Reposition and regain all sources from the latest obstacle frame.
         *  Called once per camera frame.  Must not allocate heap memory. */
        void update(const ObstacleFrame& frame);

        /** true if the OpenAL context initialised and audio hardware is reachable. */
        bool isReady() const { return mReady; }

    private:
        ALCdevice*  mDevice;
        ALCcontext* mContext;
        ALuint      mNoiseBuffer;

        // One source per obstacle slot.
        std::array<ALuint, kMaxObstacles> mSources;

        // Per-source smoothed gain — EMA-filtered to avoid clicks when an
        // obstacle suddenly appears or disappears.
        std::array<float, kMaxObstacles> mSmoothedGain;

        bool mReady;
        int  mDiagFrames;

        // ── Tuning ────────────────────────────────────────────────────────────

        // Flow magnitude (px/frame) that maps to maximum gain.
        // 20 px/frame at 640×480 is a fast-approaching object.
        static constexpr float kMaxMagForGain = 20.0f;

        // Global headroom.  kMaxObstacles sources at full gain sum to
        // kMaxObstacles * kMasterGain; 0.75 keeps the mix well below 0 dBFS.
        static constexpr float kMasterGain = 0.75f;

        // EMA weight for gain smoothing.  0.15 = ~6-frame smoothing window;
        // fast enough to track real motion, slow enough to suppress clicks.
        static constexpr float kGainAlpha = 0.15f;

        // Horizontal spread: an obstacle at the far left/right of the frame
        // maps to ±kAzimuthScale OpenAL X units at Z = kSourceDepth.
        // tan(40°) * 1.5 ≈ 1.26 — gives ~40° azimuth at the frame edges.
        static constexpr float kAzimuthScale   = 1.26f;

        // Vertical spread: ±kElevationScale at frame top/bottom.
        // Smaller than azimuth because vertical HRTF resolution is coarser.
        static constexpr float kElevationScale = 0.5f;

        // Fixed depth: all obstacles are placed at this Z distance.
        // Proximity is encoded by gain, not by Z, because AL_NONE distance
        // model means Z has no effect on gain anyway.
        static constexpr float kSourceDepth = -1.5f;

        static constexpr int   kSampleRate  = 44100;
        static constexpr int   kNoiseFrames = 44100;
        static constexpr float kHpAlpha     = 0.9715f;

        void initOpenAL();
        void generateNoiseBuffer();
        void initSources();
        void shutdownOpenAL();

        /** Convert normalised image coordinates to an OpenAL 3-D position. */
        static void imageToALPosition(float normX, float normY,
                                      float& outX, float& outY, float& outZ);
    };

} // namespace assistivenav