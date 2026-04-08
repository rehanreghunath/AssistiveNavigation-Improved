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
              mSources{}, mSmoothedGain{}, mSmoothedPitch{},
              mReady(false), mDiagFrames(0)
    {
        for (float& p : mSmoothedPitch) p = 1.0f;
        initOpenAL();
        if (mReady) { generateNoiseBuffer(); initSources(); }
    }

    AudioEngine::~AudioEngine() { shutdownOpenAL(); }

    // ─────────────────────────────────────────────────────────────────────────
    //  initOpenAL
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
    //  generateNoiseBuffer
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
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::initSources() {
        alGenSources(kNumSources, mSources.data());
        const ALboolean hasSpatialise = alIsExtensionPresent("AL_SOFT_source_spatialize");

        // Sources 0-2: centre column (cells 1, 4, 7), stacked vertically.
        for (int row = 0; row < 3; ++row) {
            const ALuint src = mSources[row];
            alSourcei (src, AL_BUFFER,         static_cast<ALint>(mNoiseBuffer));
            alSourcei (src, AL_LOOPING,        AL_TRUE);
            alSourcef (src, AL_GAIN,           0.0f);
            alSourcef (src, AL_PITCH,          1.0f);
            alSourcef (src, AL_ROLLOFF_FACTOR, 0.0f);
            alSource3f(src, AL_POSITION,       0.0f, kRowY[row], kSourceDepth);
            alSource3f(src, AL_VELOCITY,       0.0f, 0.0f, 0.0f);
            if (hasSpatialise) alSourcei(src, AL_SOURCE_SPATIALIZE_SOFT, AL_TRUE);
            mSmoothedGain[row]  = 0.0f;
            mSmoothedPitch[row] = 1.0f;
        }

        // Source 3: side warning — starts centred, panned at runtime.
        {
            const ALuint src = mSources[3];
            alSourcei (src, AL_BUFFER,         static_cast<ALint>(mNoiseBuffer));
            alSourcei (src, AL_LOOPING,        AL_TRUE);
            alSourcef (src, AL_GAIN,           0.0f);
            alSourcef (src, AL_PITCH,          1.0f);
            alSourcef (src, AL_ROLLOFF_FACTOR, 0.0f);
            alSource3f(src, AL_POSITION,       0.0f, 0.0f, kSourceDepth);
            alSource3f(src, AL_VELOCITY,       0.0f, 0.0f, 0.0f);
            if (hasSpatialise) alSourcei(src, AL_SOURCE_SPATIALIZE_SOFT, AL_TRUE);
            mSmoothedGain[3]  = 0.0f;
            mSmoothedPitch[3] = 1.0f;
        }

        alSourcePlayv(kNumSources, mSources.data());
        const ALenum err = alGetError();
        if (err != AL_NO_ERROR) LOGE("Source init: 0x%x", err);
        else LOGI("%d sources started silent", kNumSources);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  updateFromGrid — the redesigned audio path
    //
    //  Grid layout (portrait, 3×3):
    //    col:  0     1     2
    //    row 0: [0]  [1]  [2]   head level
    //    row 1: [3]  [4]  [5]   chest level
    //    row 2: [6]  [7]  [8]   floor level
    //
    //  Source mapping (centre column only for forward path):
    //    src 0 ← cell 1 (top-centre)
    //    src 1 ← cell 4 (mid-centre)   ← primary danger
    //    src 2 ← cell 7 (bot-centre)   ← primary danger
    //    src 3 ← side warning (left/right columns when hotter than centre)
    //
    //  Why this fixes approach=silent / retreat=loud:
    //    dangerScore is derived from meanMag which is the RMS of optical flow
    //    magnitude.  Flow magnitude rises whether the camera moves toward OR
    //    away from an object.  The old anomaly-based path failed for approach
    //    because approach = radially outward flow = anomaly ≈ 0.
    //    Here there is NO anomaly classifier in the audio path at all.
    //
    //  TTC → pitch:
    //    Low TTC (imminent) → high pitch.  Large/zero TTC → low pitch.
    //    Gives a natural "beeping faster as you get closer" feel without
    //    requiring the user to judge gain level consciously.
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::updateFromGrid(const GridResult& grid) {
        if (!mReady) return;

        // Centre column cell indices for portrait mode.
        static constexpr int kCentreCells[3] = {1, 4, 7};

        // ── Centre column sources (forward path) ──────────────────────────────
        for (int srcIdx = 0; srcIdx < 3; ++srcIdx) {
            const CellMetrics& cell = grid.cells[kCentreCells[srcIdx]];
            const ALuint src = mSources[srcIdx];

            // Gain: linear ramp from kMinDanger→kFullDanger mapped to 0→kMasterGain.
            const float danger = cell.dangerScore;
            const float targetGain = (danger < kMinDanger)
                                     ? 0.0f
                                     : std::clamp((danger - kMinDanger) / (kFullDanger - kMinDanger),
                                                  0.0f, 1.0f) * kMasterGain;

            mSmoothedGain[srcIdx] = kGainAlpha * targetGain
                                    + (1.0f - kGainAlpha) * mSmoothedGain[srcIdx];
            alSourcef(src, AL_GAIN, mSmoothedGain[srcIdx]);

            // Pitch: inversely proportional to TTC.
            // TTC=0 (no motion detected) → neutral low pitch.
            // TTC≤kMinTTCForHighPitch    → kPitchHigh (imminent).
            // TTC≥kMaxTTCForPitch        → kPitchLow  (distant / slow).
            float targetPitch = kPitchLow;
            if (cell.ttc > 0.0f) {
                // t=1 when TTC=kMinTTCForHighPitch, t=0 when TTC=kMaxTTCForPitch.
                const float t = 1.0f - std::clamp(
                        (cell.ttc - kMinTTCForHighPitch) /
                        (kMaxTTCForPitch - kMinTTCForHighPitch),
                        0.0f, 1.0f);
                targetPitch = kPitchLow + t * (kPitchHigh - kPitchLow);
            }

            mSmoothedPitch[srcIdx] = kPitchAlpha * targetPitch
                                     + (1.0f - kPitchAlpha) * mSmoothedPitch[srcIdx];

            // Apply pitch only when the source is audible; avoids a click on
            // first activation from a stale pitch value.
            if (mSmoothedGain[srcIdx] > 0.01f) {
                alSourcef(src, AL_PITCH, mSmoothedPitch[srcIdx]);
            }
        }

        // ── Side-warning source ───────────────────────────────────────────────
        // Fires when a peripheral column is meaningfully hotter than the
        // centre cell in the same row.  Warns of doorframes, poles, and walls
        // on one side without masking the forward path signal.
        static constexpr int kLeftCells[3]  = {0, 3, 6};
        static constexpr int kRightCells[3] = {2, 5, 8};

        // How much hotter (in dangerScore) the side must be vs the same-row
        // centre cell before the side warning fires.
        static constexpr float kSideDangerMargin = 0.10f;

        float bestSideDanger = 0.0f;
        float sidePanX = 0.0f;

        for (int row = 0; row < 3; ++row) {
            const float leftD   = grid.cells[kLeftCells[row]].dangerScore;
            const float rightD  = grid.cells[kRightCells[row]].dangerScore;
            const float centreD = grid.cells[kCentreCells[row]].dangerScore;
            const float maxSide = std::max(leftD, rightD);

            // Side must beat both centre AND the current best side candidate.
            if (maxSide > centreD + kSideDangerMargin && maxSide > bestSideDanger) {
                bestSideDanger = maxSide;
                // Negative X = left, positive X = right.
                sidePanX = (leftD >= rightD) ? -kSideX : kSideX;
            }
        }

        const ALuint sideSrc = mSources[3];
        const float sideTargetGain = (bestSideDanger < kMinDanger)
                                     ? 0.0f
                                     : std::clamp((bestSideDanger - kMinDanger) / (kFullDanger - kMinDanger),
                                                  0.0f, 1.0f) * kSideGain;

        mSmoothedGain[3] = kGainAlpha * sideTargetGain
                           + (1.0f - kGainAlpha) * mSmoothedGain[3];
        alSourcef(sideSrc, AL_GAIN, mSmoothedGain[3]);

        // Reposition when contributing — use chest-height row.
        if (bestSideDanger >= kMinDanger) {
            alSource3f(sideSrc, AL_POSITION, sidePanX, kRowY[1], kSourceDepth);
        }

        // ── Diagnostics (every ~3 s at 30 fps) ───────────────────────────────
        if (++mDiagFrames >= 90) {
            mDiagFrames = 0;
            LOGI("Grid audio | "
                 "top(c1) d=%.2f g=%.3f p=%.2f | "
                 "mid(c4) d=%.2f g=%.3f p=%.2f | "
                 "bot(c7) d=%.2f g=%.3f p=%.2f | "
                 "side g=%.3f",
                 grid.cells[1].dangerScore, mSmoothedGain[0], mSmoothedPitch[0],
                 grid.cells[4].dangerScore, mSmoothedGain[1], mSmoothedPitch[1],
                 grid.cells[7].dangerScore, mSmoothedGain[2], mSmoothedPitch[2],
                 mSmoothedGain[3]);
            const ALenum alErr = alGetError();
            if (alErr != AL_NO_ERROR) LOGE("OpenAL error: 0x%x", alErr);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  shutdownOpenAL
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