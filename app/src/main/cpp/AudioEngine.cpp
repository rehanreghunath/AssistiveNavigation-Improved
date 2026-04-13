#include "AudioEngine.h"
#include <android/log.h>
#include <algorithm>
#include <cmath>
#include <random>

#define LOG_TAG "AudioEngine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    constexpr float AudioEngine::kRowY[3];

    // ─────────────────────────────────────────────────────────────────────────
    //  Constructor / Destructor
    // ─────────────────────────────────────────────────────────────────────────

    AudioEngine::AudioEngine()
            : mDevice(nullptr), mContext(nullptr), mNoiseBuffer(0),
              mSources{}, mSmoothedGain{}, mSmoothedPitch{}, mSmoothedY{},
              mReady(false), mDiagFrames(0)
    {
        for (float& p : mSmoothedPitch) p = 1.0f;
        initOpenAL();
        if (mReady) { generateNoiseBuffer(); initSources(); }
    }

    AudioEngine::~AudioEngine() { shutdownOpenAL(); }

    // ─────────────────────────────────────────────────────────────────────────
    //  initOpenAL  (unchanged from prior version)
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::initOpenAL() {
        setenv("ALSOFT_ANDROID_STREAM_TYPE", "3", 1);

        mDevice = alcOpenDevice(nullptr);
        if (!mDevice) { LOGE("alcOpenDevice failed (0x%x)", alcGetError(nullptr)); return; }

        if (alcIsExtensionPresent(mDevice, "ALC_SOFT_HRTF")) {
            const ALCint attrs[] = { ALC_HRTF_SOFT, ALC_TRUE, ALC_FREQUENCY, kSampleRate, 0 };
            mContext = alcCreateContext(mDevice, attrs);
            LOGI("HRTF context requested at %d Hz", kSampleRate);
        } else {
            const ALCint attrs[] = { ALC_FREQUENCY, kSampleRate, 0 };
            mContext = alcCreateContext(mDevice, attrs);
            LOGI("ALC_SOFT_HRTF unavailable — stereo panning only");
        }

        if (!mContext) {
            LOGE("alcCreateContext failed (0x%x)", alcGetError(mDevice));
            alcCloseDevice(mDevice); mDevice = nullptr; return;
        }

        if (alcMakeContextCurrent(mContext) == ALC_FALSE) {
            LOGE("alcMakeContextCurrent failed");
            alcDestroyContext(mContext); alcCloseDevice(mDevice);
            mContext = nullptr; mDevice = nullptr; return;
        }

        ALCint hrtfStatus = 0;
        alcGetIntegerv(mDevice, ALC_HRTF_STATUS_SOFT, 1, &hrtfStatus);
        LOGI("HRTF status: %d (%s)", hrtfStatus, hrtfStatus == 1 ? "ENABLED" : "NOT ENABLED");

        const ALfloat orientation[6] = { 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f };
        alListener3f(AL_POSITION,    0.0f, 0.0f, 0.0f);
        alListener3f(AL_VELOCITY,    0.0f, 0.0f, 0.0f);
        alListenerfv(AL_ORIENTATION, orientation);
        alListenerf (AL_GAIN,        1.0f);
        alDistanceModel(AL_NONE);

        mReady = true;
        LOGI("AudioEngine ready");
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  generateNoiseBuffer  (unchanged)
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::generateNoiseBuffer() {
        std::vector<int16_t> samples(kNoiseFrames);
        std::mt19937 rng(0xA55174u);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        float prevX = 0.0f, prevY = 0.0f;
        for (int n = 0; n < kNoiseFrames; ++n) {
            const float x = dist(rng);
            const float y = kHpAlpha * (prevY + x - prevX);
            prevX = x; prevY = y;
            samples[n] = static_cast<int16_t>(std::clamp(y, -1.0f, 1.0f) * 32767.0f);
        }
        alGenBuffers(1, &mNoiseBuffer);
        alBufferData(mNoiseBuffer, AL_FORMAT_MONO16, samples.data(),
                     static_cast<ALsizei>(samples.size() * sizeof(int16_t)), kSampleRate);
        const ALenum err = alGetError();
        if (err != AL_NO_ERROR) LOGE("alBufferData: 0x%x", err);
        else LOGI("Noise buffer OK (%d frames @ %d Hz)", kNoiseFrames, kSampleRate);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  initSources
    //
    //  Two sources, both started at gain=0 to eliminate per-detection
    //  startup latency.  Initial Y is 0 (chest height); updateFromGrid()
    //  will begin tracking the correct height once flow data arrives.
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::initSources() {
        alGenSources(kNumSources, mSources.data());
        const ALboolean hasSpatialise = alIsExtensionPresent("AL_SOFT_source_spatialize");

        const float panX[kNumSources] = { -kSideX, kSideX };

        for (int i = 0; i < kNumSources; ++i) {
            const ALuint src = mSources[i];
            alSourcei (src, AL_BUFFER,         static_cast<ALint>(mNoiseBuffer));
            alSourcei (src, AL_LOOPING,        AL_TRUE);
            alSourcef (src, AL_GAIN,           0.0f);
            alSourcef (src, AL_PITCH,          1.0f);
            alSourcef (src, AL_ROLLOFF_FACTOR, 0.0f);
            alSource3f(src, AL_POSITION,       panX[i], 0.0f, kSourceDepth);
            alSource3f(src, AL_VELOCITY,       0.0f, 0.0f, 0.0f);
            if (hasSpatialise) alSourcei(src, AL_SOURCE_SPATIALIZE_SOFT, AL_TRUE);
            mSmoothedGain[i]  = 0.0f;
            mSmoothedPitch[i] = 1.0f;
            mSmoothedY[i]     = 0.0f;
        }

        alSourcePlayv(kNumSources, mSources.data());
        const ALenum err = alGetError();
        if (err != AL_NO_ERROR) LOGE("Source init: 0x%x", err);
        else LOGI("%d sources started silent (left / right)", kNumSources);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  updateFromGrid
    //
    //  For each centre-column cell (1, 4, 7 in landscape):
    //
    //    lateral = cos(meanAngle)
    //      > 0  →  flow is moving toward the right column
    //      < 0  →  flow is moving toward the left column
    //
    //  A cell only contributes if:
    //    • |lateral| > kLateralThresh  (not mostly vertical)
    //    • sampleCount >= kMinSamples  (enough vectors for a reliable angle)
    //
    //  lateralStrength (per side) is the sum of (lateralComponent * dangerScore)
    //  across contributing cells.  dangerScore is already magnitude-based, so
    //  strength is non-zero only when real flow is present.
    //
    //  The vertical position of each source tracks the weighted centroid of
    //  which rows are contributing; a person walking past at head height sounds
    //  different from an obstacle at floor level.
    //
    //  Pitch is modulated by the TTC of the dominant contributing cell so the
    //  user gets a natural "faster as it gets closer" feel.
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::updateFromGrid(const GridResult& grid) {
        if (!mReady) return;

        // Landscape centre-column cells: 1 (top), 4 (mid), 7 (bot).
        static constexpr int kCentreCells[3] = {1, 4, 7};

        // Per-side accumulators.
        float leftStrength  = 0.0f, leftSumY  = 0.0f, leftTTC  = 0.0f;
        float rightStrength = 0.0f, rightSumY = 0.0f, rightTTC = 0.0f;
        // Track which single cell is the largest contributor so its TTC
        // drives pitch — a weighted average of TTC is less meaningful.
        float leftBest  = 0.0f;
        float rightBest = 0.0f;

        for (int r = 0; r < 3; ++r) {
            const CellMetrics& cell = grid.cells[kCentreCells[r]];

            // Not enough vectors to trust the mean angle estimate.
            if (cell.sampleCount < kMinSamples) continue;

            // cos(meanAngle): +1 = pure rightward flow, -1 = pure leftward.
            const float lateral   = std::cos(cell.meanAngle);
            const float leftComp  = std::max(0.0f, -lateral);
            const float rightComp = std::max(0.0f,  lateral);

            // kLateralThresh filters cells where flow is primarily vertical.
            // For a wall approached straight-on, centre-column angles are
            // near ±π/2 (up/down) → cos ≈ 0 → neither side fires.
            if (leftComp > kLateralThresh) {
                const float s = leftComp * cell.dangerScore;
                leftStrength  += s;
                leftSumY      += kRowY[r] * s;
                if (s > leftBest) { leftBest = s; leftTTC = cell.ttc; }
            }
            if (rightComp > kLateralThresh) {
                const float s = rightComp * cell.dangerScore;
                rightStrength  += s;
                rightSumY      += kRowY[r] * s;
                if (s > rightBest) { rightBest = s; rightTTC = cell.ttc; }
            }
        }

        // Weighted centroid Y; fall back to current smoothed value if no signal
        // so the source drifts back to neutral rather than snapping.
        const float targetY[kNumSources] = {
                leftStrength  > 1e-6f ? leftSumY  / leftStrength  : mSmoothedY[0],
                rightStrength > 1e-6f ? rightSumY / rightStrength : mSmoothedY[1]
        };
        const float strengths[kNumSources] = { leftStrength,  rightStrength  };
        const float ttcs     [kNumSources] = { leftTTC,       rightTTC       };
        const float panX     [kNumSources] = { -kSideX,       kSideX         };

        for (int s = 0; s < kNumSources; ++s) {
            const ALuint src = mSources[s];

            // ── Gain ──────────────────────────────────────────────────────────
            const float targetGain = (strengths[s] < kMinLateral)
                                     ? 0.0f
                                     : std::clamp((strengths[s] - kMinLateral) / (kFullLateral - kMinLateral),
                                                  0.0f, 1.0f) * kMasterGain;

            mSmoothedGain[s] = kGainAlpha * targetGain
                               + (1.0f - kGainAlpha) * mSmoothedGain[s];
            alSourcef(src, AL_GAIN, mSmoothedGain[s]);

            // ── Vertical position ─────────────────────────────────────────────
            mSmoothedY[s] = kYAlpha * targetY[s]
                            + (1.0f - kYAlpha) * mSmoothedY[s];
            alSource3f(src, AL_POSITION, panX[s], mSmoothedY[s], kSourceDepth);

            // ── Pitch ─────────────────────────────────────────────────────────
            float targetPitch = kPitchLow;
            if (ttcs[s] > 0.0f) {
                const float t = 1.0f - std::clamp(
                        (ttcs[s] - kMinTTCForHighPitch) /
                        (kMaxTTCForPitch - kMinTTCForHighPitch),
                        0.0f, 1.0f);
                targetPitch = kPitchLow + t * (kPitchHigh - kPitchLow);
            }
            mSmoothedPitch[s] = kPitchAlpha * targetPitch
                                + (1.0f - kPitchAlpha) * mSmoothedPitch[s];

            // Only update pitch when audible to avoid a click on first activation
            // from a stale pitch value left over from the previous session.
            if (mSmoothedGain[s] > 0.01f) {
                alSourcef(src, AL_PITCH, mSmoothedPitch[s]);
            }
        }

        // ── Diagnostics (every ~3 s at 30 fps) ───────────────────────────────
        if (++mDiagFrames >= 90) {
            mDiagFrames = 0;
            LOGI("Lateral audio | "
                 "left  str=%.2f g=%.3f p=%.2f y=%.2f | "
                 "right str=%.2f g=%.3f p=%.2f y=%.2f",
                 leftStrength,  mSmoothedGain[0], mSmoothedPitch[0], mSmoothedY[0],
                 rightStrength, mSmoothedGain[1], mSmoothedPitch[1], mSmoothedY[1]);
            const ALenum alErr = alGetError();
            if (alErr != AL_NO_ERROR) LOGE("OpenAL error: 0x%x", alErr);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  shutdownOpenAL  (unchanged except kNumSources is now 2)
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::shutdownOpenAL() {
        if (!mReady) return;
        mReady = false;
        alSourceStopv(kNumSources, mSources.data());
        alDeleteSources(kNumSources, mSources.data());
        if (mNoiseBuffer) { alDeleteBuffers(1, &mNoiseBuffer); mNoiseBuffer = 0; }
        alcMakeContextCurrent(nullptr);
        if (mContext) { alcDestroyContext(mContext); mContext = nullptr; }
        if (mDevice)  { alcCloseDevice(mDevice);     mDevice  = nullptr; }
        LOGI("AudioEngine shut down");
    }

} // namespace assistivenav