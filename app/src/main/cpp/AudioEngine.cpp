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
            : mDevice(nullptr),
              mContext(nullptr),
              mNoiseBuffer(0),
              mSources{},
              mSmoothedGain{},
              mReady(false),
              mDiagFrames(0)
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
        // Force STREAM_MUSIC so AAudio routes to speaker/headphone output.
        // Must be set before alcOpenDevice; OpenAL-Soft reads it at device init.
        setenv("ALSOFT_ANDROID_STREAM_TYPE", "3", 1);

        mDevice = alcOpenDevice(nullptr);
        if (!mDevice) {
            LOGE("alcOpenDevice failed (error 0x%x)", alcGetError(nullptr));
            return;
        }
        LOGI("Opened device (STREAM_MUSIC forced via env)");

        if (alcIsExtensionPresent(mDevice, "ALC_SOFT_HRTF")) {
            const ALCint attrs[] = {
                    ALC_HRTF_SOFT,  ALC_TRUE,
                    ALC_FREQUENCY,  kSampleRate,
                    0
            };
            mContext = alcCreateContext(mDevice, attrs);
            LOGI("Requested HRTF-enabled context at %d Hz", kSampleRate);
        } else {
            const ALCint attrs[] = { ALC_FREQUENCY, kSampleRate, 0 };
            mContext = alcCreateContext(mDevice, attrs);
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

        ALCint hrtfStatus = 0;
        alcGetIntegerv(mDevice, ALC_HRTF_STATUS_SOFT, 1, &hrtfStatus);
        LOGI("HRTF status: %d (%s)",
             hrtfStatus, hrtfStatus == 1 ? "ENABLED" : "NOT ENABLED");

        const ALfloat orientation[6] = {
                0.0f, 0.0f, -1.0f,   // forward: −Z
                0.0f, 1.0f,  0.0f    // up: +Y
        };
        alListener3f(AL_POSITION,    0.0f, 0.0f, 0.0f);
        alListener3f(AL_VELOCITY,    0.0f, 0.0f, 0.0f);
        alListenerfv(AL_ORIENTATION, orientation);
        alListenerf (AL_GAIN,        1.0f);

        // Gain is controlled entirely from danger scores; no distance attenuation.
        alDistanceModel(AL_NONE);

        const ALCchar* devName = alcGetString(mDevice, ALC_ALL_DEVICES_SPECIFIER);
        LOGI("Output device: %s", devName ? devName : "(unknown)");

        mReady = true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  generateNoiseBuffer
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::generateNoiseBuffer() {
        // One second of mono 16-bit high-pass white noise, shared across all
        // sources.  The IIR high-pass (fc ≈ 200 Hz) removes content that HRTF
        // cannot spatialise and that would mask the user's ambient hearing.
        std::vector<int16_t> samples(kNoiseFrames);

        std::mt19937 rng(0xA55174);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

        float prevX = 0.0f, prevY = 0.0f;
        for (int n = 0; n < kNoiseFrames; ++n) {
            const float x = dist(rng);
            const float y = kHpAlpha * (prevY + x - prevX);
            prevX = x;
            prevY = y;
            samples[n] = static_cast<int16_t>(
                    std::clamp(y, -1.0f, 1.0f) * 32767.0f);
        }

        alGenBuffers(1, &mNoiseBuffer);
        alBufferData(mNoiseBuffer, AL_FORMAT_MONO16,
                     samples.data(),
                     static_cast<ALsizei>(samples.size() * sizeof(int16_t)),
                     kSampleRate);

        const ALenum err = alGetError();
        if (err != AL_NO_ERROR) LOGE("alBufferData failed: 0x%x", err);
        else LOGI("Noise buffer: %d frames @ %d Hz", kNoiseFrames, kSampleRate);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  initSources
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::initSources() {
        alGenSources(kMaxObstacles, mSources.data());

        const ALboolean hasSpatialise =
                alIsExtensionPresent("AL_SOFT_source_spatialize");

        for (int i = 0; i < kMaxObstacles; ++i) {
            const ALuint src = mSources[i];
            alSourcei (src, AL_BUFFER,        static_cast<ALint>(mNoiseBuffer));
            alSourcei (src, AL_LOOPING,       AL_TRUE);
            alSourcef (src, AL_GAIN,          0.0f);
            alSourcef (src, AL_ROLLOFF_FACTOR, 0.0f);
            // Initial position: directly ahead, spread by slot index so sources
            // do not start stacked exactly on top of each other.
            alSource3f(src, AL_POSITION,
                       (i - kMaxObstacles / 2) * 0.01f, 0.0f, kSourceDepth);
            alSource3f(src, AL_VELOCITY, 0.0f, 0.0f, 0.0f);

            if (hasSpatialise) alSourcei(src, AL_SOURCE_SPATIALIZE_SOFT, AL_TRUE);

            mSmoothedGain[i] = 0.0f;
        }

        alSourcePlayv(kMaxObstacles, mSources.data());

        const ALenum err = alGetError();
        if (err != AL_NO_ERROR) LOGE("Source init error: 0x%x", err);
        else LOGI("%d obstacle sources initialised (silent)", kMaxObstacles);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  imageToALPosition
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::imageToALPosition(float normX, float normY,
                                        float& outX, float& outY, float& outZ) {
        // normX [0,1]: 0 = left edge, 1 = right edge → OpenAL +X = right
        outX = (normX - 0.5f) * 2.0f * kAzimuthScale;

        // normY [0,1]: 0 = top,  1 = bottom → OpenAL +Y = up, so invert
        outY = (0.5f - normY) * 2.0f * kElevationScale;

        outZ = kSourceDepth;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  update
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::update(const ObstacleFrame& frame) {
        if (!mReady) return;

        for (int i = 0; i < kMaxObstacles; ++i) {
            const TrackedObstacle& obs = frame.obstacles[i];
            const ALuint src = mSources[i];

            // Target gain: zero for inactive/unconfirmed obstacles, magnitude-
            // mapped gain for confirmed ones.
            const float targetGain = obs.active
                                     ? std::clamp(obs.smoothedMag / kMaxMagForGain, 0.0f, 1.0f)
                                       * kMasterGain
                                     : 0.0f;

            // EMA-smooth the gain to avoid clicks on obstacle appearance /
            // disappearance.  The smoother runs even when targetGain == 0 so
            // that fade-out is gradual rather than an instantaneous cut.
            mSmoothedGain[i] = kGainAlpha * targetGain +
                               (1.0f - kGainAlpha) * mSmoothedGain[i];

            alSourcef(src, AL_GAIN, mSmoothedGain[i]);

            // Only update position when the obstacle is active — stale
            // positions from a just-retired obstacle are irrelevant because
            // the gain is already fading to zero.
            if (obs.active) {
                float alX, alY, alZ;
                imageToALPosition(obs.normX, obs.normY, alX, alY, alZ);
                alSource3f(src, AL_POSITION, alX, alY, alZ);
            }
        }

        // ── Diagnostic every ~3 s ─────────────────────────────────────────────
        if (++mDiagFrames >= 90) {
            mDiagFrames = 0;
            LOGI("active=%d", frame.activeCount);
            for (int i = 0; i < kMaxObstacles; ++i) {
                if (frame.obstacles[i].active) {
                    LOGI("  obs[%d] pos=(%.2f,%.2f) mag=%.1f age=%d gain=%.3f",
                         i,
                         frame.obstacles[i].normX,
                         frame.obstacles[i].normY,
                         frame.obstacles[i].smoothedMag,
                         frame.obstacles[i].age,
                         mSmoothedGain[i]);
                }
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  shutdownOpenAL
    // ─────────────────────────────────────────────────────────────────────────

    void AudioEngine::shutdownOpenAL() {
        if (!mReady) return;
        mReady = false;

        alSourceStopv(kMaxObstacles, mSources.data());
        alDeleteSources(kMaxObstacles, mSources.data());

        if (mNoiseBuffer != 0) {
            alDeleteBuffers(1, &mNoiseBuffer);
            mNoiseBuffer = 0;
        }

        alcMakeContextCurrent(nullptr);
        if (mContext) { alcDestroyContext(mContext); mContext = nullptr; }
        if (mDevice)  { alcCloseDevice(mDevice);     mDevice  = nullptr; }

        LOGI("AudioEngine shut down cleanly");
    }

} // namespace assistivenav