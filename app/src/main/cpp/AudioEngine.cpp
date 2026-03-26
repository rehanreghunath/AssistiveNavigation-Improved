#include "AudioEngine.h"
#include <android/log.h>
#include <algorithm>
#include <cmath>
#include <random>

#define LOG_TAG "AudioEngine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    // kCellPositions definition — must live in a .cpp to avoid the ODR violation
    // that would occur if the constexpr array were instantiated in multiple TUs.
    constexpr std::array<std::array<float, 3>, 9> AudioEngine::kCellPositions;

    // ─────────────────────────────────────────────────────────────────────────
    //  Constructor / Destructor
    // ─────────────────────────────────────────────────────────────────────────

    AudioEngine::AudioEngine()
            : mDevice(nullptr),
              mContext(nullptr),
              mNoiseBuffer(0),
              mSources{},
              mPhaseAccum{},
              mReady(false),
              mHasLastUpdateTime(false)
    {
        initOpenAL();
        if (mReady) {
            generateNoiseBuffer();
            initSources();
        }
    }

    AudioEngine::~AudioEngine() {
        shutdownOpenAL();
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  initOpenAL
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::initOpenAL() {
        // ── Device ───────────────────────────────────────────────────────────
        mDevice = alcOpenDevice(nullptr); // nullptr = system default
        if (!mDevice) {
            LOGE("alcOpenDevice failed");
            return;
        }

        // ── HRTF context ──────────────────────────────────────────────────────
        // ALC_HRTF_SOFT requests HRTF from the device.  If the extension is
        // absent (very old OpenAL-Soft build), we still create a working context
        // — spatialization will fall back to stereo panning, which is still
        // useful though less immersive.
        //
        // OpenAL-Soft compiled with ALSOFT_EMBED_HRTF_DATA=ON carries the
        // built-in "default" HRTF dataset (based on the Listen HRTF database)
        // compiled directly into libopenal.so.  No external .mhr file is needed.
        if (alcIsExtensionPresent(mDevice, "ALC_SOFT_HRTF")) {
            const ALCint attrs[] = {
                    ALC_HRTF_SOFT, ALC_TRUE,
                    0
            };
            mContext = alcCreateContext(mDevice, attrs);
            LOGI("Requested HRTF-enabled context");
        } else {
            mContext = alcCreateContext(mDevice, nullptr);
            LOGI("ALC_SOFT_HRTF unavailable — falling back to stereo panning");
        }

        if (!mContext) {
            LOGE("alcCreateContext failed (error 0x%x)", alcGetError(mDevice));
            alcCloseDevice(mDevice);
            mDevice = nullptr;
            return;
        }

        if (alcMakeContextCurrent(mContext) == ALC_FALSE) {
            LOGE("alcMakeContextCurrent failed");
            alcDestroyContext(mContext);
            alcCloseDevice(mDevice);
            mContext = nullptr;
            mDevice  = nullptr;
            return;
        }

        // ── Verify HRTF actually activated ────────────────────────────────────
        // alcGetIntegerv with ALC_HRTF_STATUS_SOFT returns one of:
        //   ALC_HRTF_DISABLED_SOFT        (0) — not enabled
        //   ALC_HRTF_ENABLED_SOFT         (1) — active
        //   ALC_HRTF_DENIED_SOFT          (2) — driver refused
        //   ALC_HRTF_REQUIRED_SOFT        (3) — was required but unavailable
        //   ALC_HRTF_HEADPHONES_DETECTED_SOFT (4) — auto-enabled via headphone detection
        //   ALC_HRTF_UNSUPPORTED_FORMAT_SOFT  (5) — sample rate mismatch
        ALCint hrtfStatus = 0;
        alcGetIntegerv(mDevice, ALC_HRTF_STATUS_SOFT, 1, &hrtfStatus);
        LOGI("HRTF status: %d (%s)",
             hrtfStatus,
             hrtfStatus == 1 ? "ENABLED" : "NOT ENABLED — spatial cues reduced");

        // ── Listener ──────────────────────────────────────────────────────────
        // Listener at origin, facing −Z (into the scene), +Y up.
        // This is the OpenAL default; set explicitly to make the intent clear.
        const ALfloat orientation[6] = {
                0.0f, 0.0f, -1.0f,  // forward vector: −Z
                0.0f, 1.0f,  0.0f   // up vector: +Y
        };
        alListener3f(AL_POSITION,    0.0f, 0.0f, 0.0f);
        alListener3f(AL_VELOCITY,    0.0f, 0.0f, 0.0f);
        alListenerfv(AL_ORIENTATION, orientation);
        alListenerf (AL_GAIN,        1.0f);

        // Disable distance-based attenuation globally.  We control each source's
        // gain directly from the danger score, which already encodes proximity.
        // Letting OpenAL also attenuate by distance would double-count.
        alDistanceModel(AL_NONE);

        mReady = true;
        LOGI("AudioEngine ready — %s",
             alGetString(AL_RENDERER) ? alGetString(AL_RENDERER) : "unknown renderer");
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  generateNoiseBuffer
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::generateNoiseBuffer() {
        // ── Generate one second of mono 16-bit high-pass white noise ─────────
        //
        // White noise is the cue signal.  The HRTF filter will colour it
        // differently for each ear depending on source direction, and the human
        // auditory system uses those coloration cues to localise the sound.
        //
        // A first-order IIR high-pass removes DC and content below ~200 Hz.
        // Sub-200 Hz content is:
        //   1. Poorly localised by HRTF — the listener cannot determine direction
        //   2. Consumed disproportionately by the audio driver's output stage
        //   3. More likely to mask speech (critical for a navigation aid)
        // Removing it makes the spatial cues cleaner and preserves headphone headroom.
        //
        // The IIR recurrence is:   y[n] = α·(y[n−1] + x[n] − x[n−1])
        // where α = RC/(RC + dt), RC = 1/(2π·fc), fc = 200 Hz, dt = 1/44100.
        //
        // This allocation happens once at init; not in the hot loop.
        std::vector<int16_t> samples(kNoiseFrames);

        // Fixed seed: the specific noise content is irrelevant — only its
        // spectral properties matter — so reproducibility is preferred over
        // unpredictability.
        std::mt19937 rng(0xA55174 /* arbitrary constant */);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

        float prevX = 0.0f;
        float prevY = 0.0f;

        for (int n = 0; n < kNoiseFrames; ++n) {
            const float x = dist(rng);
            const float y = kHpAlpha * (prevY + x - prevX);
            prevX = x;
            prevY = y;
            // std::clamp before casting: the highpass can momentarily push
            // the output above the input peak; clamp prevents int16 overflow.
            samples[n] = static_cast<int16_t>(
                    std::clamp(y, -1.0f, 1.0f) * 32767.0f);
        }

        alGenBuffers(1, &mNoiseBuffer);
        alBufferData(
                mNoiseBuffer,
                AL_FORMAT_MONO16,
                samples.data(),
                static_cast<ALsizei>(samples.size() * sizeof(int16_t)),
                kSampleRate
        );

        const ALenum err = alGetError();
        if (err != AL_NO_ERROR) {
            LOGE("alBufferData failed: 0x%x", err);
        } else {
            LOGI("Noise buffer generated: %d frames @ %d Hz", kNoiseFrames, kSampleRate);
        }
        // samples goes out of scope here; the data now lives in the AL driver.
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  initSources
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::initSources() {
        alGenSources(9, mSources.data());

        // Check once whether the spatialise extension is available.
        // AL_SOFT_source_spatialize forces per-source HRTF processing even when
        // the source would otherwise be treated as ambient (e.g. very close to
        // the listener origin).
        const ALboolean hasSpatialise =
                alIsExtensionPresent("AL_SOFT_source_spatialize");

        for (int i = 0; i < 9; ++i) {
            const ALuint src = mSources[i];

            alSourcei(src, AL_BUFFER,   static_cast<ALint>(mNoiseBuffer));
            alSourcei(src, AL_LOOPING,  AL_TRUE);

            // Position encodes direction for the HRTF engine.
            alSource3f(src, AL_POSITION,
                       kCellPositions[i][0],
                       kCellPositions[i][1],
                       kCellPositions[i][2]);

            alSource3f(src, AL_VELOCITY, 0.0f, 0.0f, 0.0f);

            // Rolloff is handled entirely through manual gain control in update().
            alSourcef(src, AL_ROLLOFF_FACTOR, 0.0f);

            // Start silent.  The source is already playing (buffer loaded,
            // looping = true); gain=0 means it produces no output until the
            // first update() with a non-zero danger score.
            alSourcef(src, AL_GAIN, 0.0f);

            if (hasSpatialise) {
                alSourcei(src, AL_SOURCE_SPATIALIZE_SOFT, AL_TRUE);
            }

            mPhaseAccum[i] = 0.0f;
        }

        // Start all sources.  They will play silently until update() raises gain.
        // Starting them here avoids the ~5 ms latency of calling alSourcePlay
        // on-demand when the first threat is detected.
        alSourcePlayv(9, mSources.data());

        const ALenum err = alGetError();
        if (err != AL_NO_ERROR) {
            LOGE("Source init error: 0x%x", err);
        } else {
            LOGI("9 spatial sources initialised and playing (silent)");
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  update  — called once per camera frame from the camera executor thread
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::update(const GridResult& grid) {
        if (!mReady) return;

        // ── Delta time ────────────────────────────────────────────────────────
        // Phase accumulation needs elapsed wall-clock time, not frame count,
        // so modulation stays at the correct perceptual frequency regardless
        // of the actual frame rate.
        const std::chrono::steady_clock::time_point now =
                std::chrono::steady_clock::now();

        float dt = 1.0f / kTTCFrameRate; // sensible default for the first frame
        if (mHasLastUpdateTime) {
            const float measured =
                    std::chrono::duration<float>(now - mLastUpdateTime).count();
            // Clamp dt: values > 0.1 s indicate a pipeline stall (app paused,
            // screen off, etc.).  Using the stale dt would cause a phase jump.
            dt = std::clamp(measured, 0.001f, 0.1f);
        }
        mLastUpdateTime    = now;
        mHasLastUpdateTime = true;

        // ── Per-cell gain update ──────────────────────────────────────────────
        // Hot loop: 9 iterations, each is:
        //   • a few float comparisons and multiplies
        //   • one sinf (deferred — only for active sources)
        //   • one alSourcef call (non-blocking AL parameter write)
        // No heap allocation.

        const float twoPi = 2.0f * static_cast<float>(M_PI);

        for (int i = 0; i < 9; ++i) {
            const CellMetrics& c = grid.cells[i];

            if (c.dangerScore <= kDangerThreshold || c.sampleCount == 0) {
                // Silence this source.  Do NOT reset the phase accumulator —
                // a sudden phase reset when the source re-enters would click.
                alSourcef(mSources[i], AL_GAIN, 0.0f);
                continue;
            }

            // ── Modulation frequency ──────────────────────────────────────────
            // TTC is in frames at kTTCFrameRate fps.
            // Higher flow magnitude → lower TTC → faster modulation → more urgent.
            // When TTC == 0 (stationary after noise floor) the danger score is
            // also 0 by construction in GridAnalyzer, so this branch is not
            // reached.  Guard with > 0 anyway.
            const float modFreq = (c.ttc > 0.0f)
                                  ? std::clamp(kTTCFrameRate / c.ttc, kModFreqMin, kModFreqMax)
                                  : kModFreqMin;

            // Accumulate phase with wrap.  Using fmodf is equivalent but the
            // subtraction is branchless on ARM NEON.
            mPhaseAccum[i] += modFreq * dt * twoPi;
            if (mPhaseAccum[i] >= twoPi) mPhaseAccum[i] -= twoPi;

            // AM modulation: depth = 100 %.  The factor (0.5 + 0.5·sin) maps
            // the sinusoid from [−1, 1] to [0, 1], giving a pulsing effect
            // that dips to silence rather than merely dimming, which is more
            // perceptible in noisy environments.
            const float mod  = 0.5f + 0.5f * std::sin(mPhaseAccum[i]);
            const float gain = c.dangerScore * mod * kMasterGain;

            alSourcef(mSources[i], AL_GAIN, gain);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  shutdownOpenAL
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::shutdownOpenAL() {
        if (!mReady) return;
        mReady = false;

        // Stop all sources before deleting them.  Deleting a playing source on
        // some OpenAL implementations produces a brief click or assert.
        alSourceStopv(9, mSources.data());
        alDeleteSources(9, mSources.data());

        if (mNoiseBuffer != 0) {
            alDeleteBuffers(1, &mNoiseBuffer);
            mNoiseBuffer = 0;
        }

        alcMakeContextCurrent(nullptr);

        if (mContext) {
            alcDestroyContext(mContext);
            mContext = nullptr;
        }

        if (mDevice) {
            alcCloseDevice(mDevice);
            mDevice = nullptr;
        }

        LOGI("AudioEngine shut down cleanly");
    }

} // namespace assistivenav