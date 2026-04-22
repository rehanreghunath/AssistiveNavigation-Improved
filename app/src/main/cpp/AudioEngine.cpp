#include "AudioEngine.h"
#include <android/log.h>
#include <algorithm>
#include <cmath>
#include <random>

#define LOG_TAG "AudioEngine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace assistivenav {

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
    //  initOpenAL  (unchanged)
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
    //  initSources  (unchanged)
    //
    //  Two sources, both started at gain=0 to eliminate per-detection
    //  startup latency.  Initial Y is 0 (chest height); updateFromObstacles()
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
    //  updateFromObstacles
    //
    //  PANNING APPROACH — normX, not flow direction:
    //
    //    Left weight  = (1 − normX):  1.0 at far left,  0.0 at far right
    //    Right weight = normX:         0.0 at far left,  1.0 at far right
    //
    //    Both weights are 0.5 when normX = 0.5 (obstacle directly ahead),
    //    so a forward obstacle fires both ears at equal gain — the correct
    //    HRTF percept for a hazard straight ahead.
    //
    //    This is immune to flow-direction inversion from head/body rotation
    //    because normX is the obstacle's position in the frame, not the
    //    direction its pixels happen to be moving.
    //
    //  SUPPRESSION FACTOR:
    //
    //    All target gains are multiplied by suppressionFactor before EMA.
    //    Factor = 1 during normal walking micro-sway; approaches 0 during
    //    fast head rotation.  The EMA then fades gain smoothly rather than
    //    cutting abruptly.
    //
    //  VERTICAL POSITION:
    //
    //    normY = 0 → top of frame (head height) → alY = +0.5
    //    normY = 0.5 → centre (chest)           → alY =  0.0
    //    normY = 1 → bottom of frame (floor)    → alY = −0.5
    //    Formula:  alY = 0.5 − normY
    //
    //    Each source's Y is EMA-tracked independently so the percept drifts
    //    smoothly as the obstacle moves vertically.
    //
    //  MULTIPLE OBSTACLES:
    //
    //    Iterate all active obstacles.  For each side, track the maximum
    //    weighted confidence seen (dominant obstacle) and its Y and pitch.
    //    Using max rather than sum prevents gain from being additive across
    //    many detections — a cluster of near-zero-confidence ghosts should
    //    not add up to a loud trigger.
    //
    //  PITCH:
    //
    //    Derived from obstacle.smoothedMag (anomaly-weighted px/frame).
    //    Higher values correlate with either faster approach or closer range.
    //    Pitch is only written when the source is audible to avoid a click
    //    from a stale value on first activation.
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::updateFromObstacles(const ObstacleFrame& frame,
                                          const float suppressionFactor) {
        if (!mReady) return;

        // Per-source accumulators: dominant gain and corresponding properties.
        float targetGain [kNumSources] = { 0.0f, 0.0f };
        float targetY    [kNumSources] = { mSmoothedY[0], mSmoothedY[1] };
        float targetMag  [kNumSources] = { 0.0f, 0.0f };  // smoothedMag of best obstacle

        for (int i = 0; i < kMaxObstacles; ++i) {
            const TrackedObstacle& obs = frame.obstacles[i];
            if (!obs.active) continue;

            // ── Confidence → raw gain ─────────────────────────────────────────
            // Map [kMinConfidence, kFullConfidence] → [0, 1], then apply
            // master gain and suppression.  Values below kMinConfidence are
            // silent regardless of suppression.
            const float confNorm = std::clamp(
                    (obs.confidenceScore - kMinConfidence) /
                    (kFullConfidence     - kMinConfidence),
                    0.0f, 1.0f);
            const float rawGain = confNorm * suppressionFactor * kMasterGain;

            if (rawGain < 1e-4f) continue;   // nothing useful to contribute

            // ── Position → left/right weights (linear pan law) ───────────────
            // normX = 0 → all left, normX = 1 → all right, 0.5 → equal
            const float panLeft  = 1.0f - obs.normX;
            const float panRight = obs.normX;

            // ── Vertical position mapping ─────────────────────────────────────
            // normY [0..1] (top..bottom) → alY [+0.5..-0.5] (head..floor).
            const float alY = 0.5f - obs.normY;

            // Keep only the dominant obstacle per side (maximum weighted gain).
            // This prevents a swarm of low-confidence ghosts from accumulating.
            const float leftContrib  = panLeft  * rawGain;
            const float rightContrib = panRight * rawGain;

            if (leftContrib > targetGain[0]) {
                targetGain[0] = leftContrib;
                targetY[0]    = alY;
                targetMag[0]  = obs.smoothedMag;
            }
            if (rightContrib > targetGain[1]) {
                targetGain[1] = rightContrib;
                targetY[1]    = alY;
                targetMag[1]  = obs.smoothedMag;
            }
        }

        // ── Apply to OpenAL sources ───────────────────────────────────────────

        const float panX[kNumSources] = { -kSideX, kSideX };

        for (int s = 0; s < kNumSources; ++s) {
            const ALuint src = mSources[s];

            // ── Gain ──────────────────────────────────────────────────────────
            mSmoothedGain[s] = kGainAlpha * targetGain[s]
                               + (1.0f - kGainAlpha) * mSmoothedGain[s];
            alSourcef(src, AL_GAIN, mSmoothedGain[s]);

            // ── Vertical position ─────────────────────────────────────────────
            mSmoothedY[s] = kYAlpha * targetY[s]
                            + (1.0f - kYAlpha) * mSmoothedY[s];
            alSource3f(src, AL_POSITION, panX[s], mSmoothedY[s], kSourceDepth);

            // ── Pitch ─────────────────────────────────────────────────────────
            // Map smoothedMag [kPitchMagLow, kPitchMagHigh] → [kPitchLow, kPitchHigh].
            // Only applied when audible to prevent a pop on first activation
            // from a stale pitch accumulated during a previous silent period.
            if (mSmoothedGain[s] > 0.01f) {
                const float t = std::clamp(
                        (targetMag[s] - kPitchMagLow) /
                        (kPitchMagHigh - kPitchMagLow),
                        0.0f, 1.0f);
                const float targetPitch = kPitchLow + t * (kPitchHigh - kPitchLow);
                mSmoothedPitch[s] = kPitchAlpha * targetPitch
                                    + (1.0f - kPitchAlpha) * mSmoothedPitch[s];
                alSourcef(src, AL_PITCH, mSmoothedPitch[s]);
            }
        }

        // ── Diagnostics (every ~3 s at 30 fps) ───────────────────────────────
        if (++mDiagFrames >= 90) {
            mDiagFrames = 0;
            LOGI("Obstacle audio | active=%d suppression=%.2f | "
                 "left  g=%.3f p=%.2f y=%.2f | "
                 "right g=%.3f p=%.2f y=%.2f",
                 frame.activeCount, suppressionFactor,
                 mSmoothedGain[0], mSmoothedPitch[0], mSmoothedY[0],
                 mSmoothedGain[1], mSmoothedPitch[1], mSmoothedY[1]);
            const ALenum alErr = alGetError();
            if (alErr != AL_NO_ERROR) LOGE("OpenAL error: 0x%x", alErr);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  shutdownOpenAL  (unchanged)
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