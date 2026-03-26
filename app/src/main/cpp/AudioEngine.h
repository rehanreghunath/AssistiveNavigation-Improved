#pragma once

#include "FlowTypes.h"

// OpenAL-Soft headers.  The AL/ prefix is the standard layout for prebuilt SDKs.
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <array>
#include <chrono>
#include <cstdint>

// AL_SOURCE_SPATIALIZE_SOFT forces per-source HRTF even when the listener has
// no explicit orientation set.  Guard the define so builds against older
// alext.h headers that lack it still compile; the code checks for the
// extension at runtime before using it.
#ifndef AL_SOURCE_SPATIALIZE_SOFT
#define AL_SOURCE_SPATIALIZE_SOFT 0x1214
#endif

namespace assistivenav {

    // ─────────────────────────────────────────────────────────────────────────
    // AudioEngine
    //
    // Owns a single OpenAL-Soft context with HRTF enabled and nine looping
    // band-pass white-noise sources — one per grid cell.  Each source is
    // positioned in 3-D space to match the on-screen cell it represents.
    // OpenAL-Soft's HRTF convolution engine translates position into:
    //   • ITD  (Interaural Time Difference)  — delay between left/right ear
    //   • ILD  (Interaural Level Difference) — level difference between ears
    //   • Pinna coloration                   — frequency shaping from ear shape
    // All three are encoded in the built-in HRTF dataset; no external file is
    // required (OpenAL-Soft is compiled with ALSOFT_EMBED_HRTF_DATA=ON).
    //
    // Gain is updated each frame from GridResult::dangerScore.  Urgency is
    // communicated by amplitude modulation: a slow pulse (0.5 Hz) for distant
    // threats, a fast pulse (4 Hz) for imminent ones.
    //
    // Thread-safety: update() is called from the camera executor thread.
    // All other methods (constructor, destructor) must be called from the
    // same thread that called nativeInit / nativeDestroy — in practice the
    // main thread.  Do not call update() concurrently with the destructor.
    // ─────────────────────────────────────────────────────────────────────────

    class AudioEngine {
    public:
        AudioEngine();
        ~AudioEngine();

        AudioEngine(const AudioEngine&)            = delete;
        AudioEngine& operator=(const AudioEngine&) = delete;

        /** Update source gains and modulation from the latest grid analysis.
         *  Called once per camera frame; must not allocate heap memory. */
        void update(const GridResult& grid);

        /** true if the OpenAL context and at least one HRTF dataset loaded. */
        bool isReady() const { return mReady; }

    private:
        ALCdevice*  mDevice;
        ALCcontext* mContext;
        ALuint      mNoiseBuffer;             // shared across all 9 sources

        std::array<ALuint, 9> mSources;
        std::array<float,  9> mPhaseAccum;    // per-source AM modulation phase [0, 2π)

        bool mReady;
        bool mHasLastUpdateTime;

        std::chrono::steady_clock::time_point mLastUpdateTime;

        // ── Spatial layout ────────────────────────────────────────────────────
        // Fixed 3-D positions in OpenAL space (+X=right, +Y=up, −Z=forward).
        // Row 2 (bottom) is placed slightly below horizontal (Y=−0.25) and
        // closest in Z (−0.8) to convey ground-level, near-range threats.
        // Row 0 (top) is placed above horizontal (Y=+0.5) and further in Z
        // (−2.0) to convey elevated, far-range content.
        // Column spread (X=±1.5) gives ±45° azimuth at Z=−1.5, which is
        // wide enough to be clearly left/right without sounding unnatural.
        static constexpr std::array<std::array<float, 3>, 9> kCellPositions = {{
                                                                                       {-1.5f,  0.5f, -2.0f},  // 0  top-left
                                                                                       { 0.0f,  0.5f, -2.0f},  // 1  top-centre
                                                                                       { 1.5f,  0.5f, -2.0f},  // 2  top-right
                                                                                       {-1.5f,  0.0f, -1.5f},  // 3  mid-left
                                                                                       { 0.0f,  0.0f, -1.5f},  // 4  mid-centre   ← primary danger
                                                                                       { 1.5f,  0.0f, -1.5f},  // 5  mid-right
                                                                                       {-1.5f, -0.25f,-0.8f},  // 6  bot-left
                                                                                       { 0.0f, -0.25f,-0.8f},  // 7  bot-centre   ← primary danger
                                                                                       { 1.5f, -0.25f,-0.8f},  // 8  bot-right
                                                                               }};

        // ── Tuning ────────────────────────────────────────────────────────────

        // Sources with dangerScore at or below this are silenced rather than
        // playing at near-zero gain.  Avoids a constant low-level noise carpet
        // when nothing is nearby.
        static constexpr float kDangerThreshold = 0.05f;

        // Headroom factor applied to all source gains before summing.
        // 9 sources at gain=1 would clip the output; 0.7 gives 3 dB of headroom.
        static constexpr float kMasterGain = 0.7f;

        // Amplitude-modulation rate bounds (Hz).
        // 0.5 Hz = barely pulsing (object far away, TTC ≈ 5 s).
        // 4.0 Hz = rapid pulsing (object within ~0.25 s).
        static constexpr float kModFreqMin = 0.5f;
        static constexpr float kModFreqMax = 4.0f;

        // At 30 fps, TTC (frames) → modulation Hz:  modFreq = 30 / TTC.
        // kTTCFrameRate lets this scale correctly if frame rate changes.
        static constexpr float kTTCFrameRate = 30.0f;

        // White-noise buffer parameters.
        static constexpr int kSampleRate  = 44100;
        static constexpr int kNoiseFrames = 44100;  // 1 second, looped

        // First-order IIR high-pass cutoff to remove DC and sub-200 Hz content
        // that is poorly spatialised by HRTF.  α = RC/(RC+dt),
        // RC = 1/(2π·200), dt = 1/44100 → α ≈ 0.9715.
        static constexpr float kHpAlpha = 0.9715f;

        // ── Private helpers ───────────────────────────────────────────────────

        /** Open the AL device, create context, enable HRTF, set listener. */
        void initOpenAL();

        /** Generate one second of high-pass-filtered white noise and upload
         *  it as a looping AL buffer. */
        void generateNoiseBuffer();

        /** Create nine AL sources, attach the shared noise buffer, position
         *  each in 3-D, start them looping at gain = 0. */
        void initSources();

        /** Tear down sources, buffer, context, device in the correct order. */
        void shutdownOpenAL();
    };

} // namespace assistivenav