#include "FlowEngine.h"
#include <android/log.h>
#include <cmath>

#define LOG_TAG "FlowPipeline"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    static constexpr float ARROW_SCALE    = 4.0f;
    static constexpr float MAX_FLOW_MAG   = 30.0f;
    static constexpr float ARROW_TIP_FRAC = 0.35f;
    static constexpr int   LINE_THICKNESS = 1;

    FlowEngine::FlowEngine(int width, int height)
            : mWidth(width),
              mHeight(height),
              mInitialized(false),
              mFramesSinceRedetect(0),
              mRenderRotation(0),
              mWinSize(21, 21),
              mCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01)
    {
        mPrevGray.create(mHeight, mWidth, CV_8UC1);
        mCurrGray.create(mHeight, mWidth, CV_8UC1);
        mVisMat.create(mHeight, mWidth, CV_8UC4);

        mPrevPts.reserve(kMaxFeatures);
        mNextPts.reserve(kMaxFeatures);
        mBackPts.reserve(kMaxFeatures);
        mStatusFwd.reserve(kMaxFeatures);
        mStatusBwd.reserve(kMaxFeatures);
        mErrFwd.reserve(kMaxFeatures);
        mErrBwd.reserve(kMaxFeatures);

        LOGI("Created %dx%d, max %d features", mWidth, mHeight, kMaxFeatures);
    }

    void FlowEngine::setRenderRotation(int degrees) {
        mRenderRotation = degrees;
    }

    void FlowEngine::preprocessFrame(const cv::Mat& raw, cv::Mat& out) {
        cv::GaussianBlur(raw, out, cv::Size(kGaussKernelSize, kGaussKernelSize), 0);
    }

    void FlowEngine::detectFeatures() {
        cv::goodFeaturesToTrack(
                mCurrGray, mPrevPts,
                kMaxFeatures, kGFTTQuality, kGFTTMinDist,
                cv::noArray(), kGFTTBlockSize, false
        );

        if (!mPrevPts.empty()) {
            cv::cornerSubPix(
                    mCurrGray, mPrevPts,
                    cv::Size(5, 5),
                    cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 20, 0.03)
            );
        }

        mCurrGray.copyTo(mPrevGray);
        mFramesSinceRedetect = 0;

        LOGI("Detected %zu features", mPrevPts.size());
    }

    void FlowEngine::trackFeatures(FlowResult& result) {
        if (mPrevPts.empty()) {
            result.trackedCount = 0;
            result.totalPts     = 0;
            return;
        }

        cv::calcOpticalFlowPyrLK(
                mPrevGray, mCurrGray,
                mPrevPts,  mNextPts,
                mStatusFwd, mErrFwd,
                mWinSize, 3, mCriteria,
                cv::OPTFLOW_LK_GET_MIN_EIGENVALS,
                kMinEigThreshold
        );

        cv::calcOpticalFlowPyrLK(
                mCurrGray, mPrevGray,
                mNextPts,  mBackPts,
                mStatusBwd, mErrBwd,
                mWinSize, 3, mCriteria,
                cv::OPTFLOW_LK_GET_MIN_EIGENVALS,
                kMinEigThreshold
        );

        result.vectors.clear();
        const int totalAttempted = static_cast<int>(mPrevPts.size());
        float magSum = 0.0f;

        std::vector<cv::Point2f> survivors;
        survivors.reserve(totalAttempted);

        for (int i = 0; i < totalAttempted; ++i) {
            if (mStatusFwd[i] == 0 || mStatusBwd[i] == 0) continue;

            const float fbDx  = mPrevPts[i].x - mBackPts[i].x;
            const float fbDy  = mPrevPts[i].y - mBackPts[i].y;
            const float fbErr = std::sqrt(fbDx * fbDx + fbDy * fbDy);
            if (fbErr > kFBThreshold) continue;

            const float dx  = mNextPts[i].x - mPrevPts[i].x;
            const float dy  = mNextPts[i].y - mPrevPts[i].y;
            const float mag = std::sqrt(dx * dx + dy * dy);

// Keep tracking even sub-threshold points so feature count stays stable.
// Only report vectors that clear the noise floor to downstream consumers.
            survivors.push_back(mNextPts[i]);

            if (mag < kMinFlowMag) continue;

            result.vectors.push_back({
                                             mPrevPts[i].x, mPrevPts[i].y,
                                             dx, dy, mag, std::atan2(dy, dx)
                                     });

            magSum += mag;
        }

        result.trackedCount  = static_cast<int>(result.vectors.size());
        result.totalPts      = totalAttempted;
        result.globalMeanMag = (result.trackedCount > 0)
                               ? magSum / static_cast<float>(result.trackedCount)
                               : 0.0f;

        mPrevPts = std::move(survivors);
    }

    FlowResult FlowEngine::processFrame(const uint8_t* yPlane, int rowStride, int64_t timestampNs) {
        FlowResult result{};
        result.timestampNs  = timestampNs;
        result.isFirstFrame = !mInitialized;

        cv::Mat rawGray(mHeight, mWidth, CV_8UC1,
                        const_cast<uint8_t*>(yPlane),
                        static_cast<size_t>(rowStride));

        preprocessFrame(rawGray, mCurrGray);

        if (!mInitialized) {
            detectFeatures();
            mInitialized = true;
            return result;
        }

        ++mFramesSinceRedetect;
        trackFeatures(result);

        const bool tooFew      = static_cast<int>(mPrevPts.size()) < kMinFeatures;
        const bool timeExpired = mFramesSinceRedetect >= kRedetectInterval;

        if (tooFew || timeExpired) detectFeatures();
        else                       mCurrGray.copyTo(mPrevGray);

        return result;
    }

    std::vector<uint8_t> FlowEngine::renderToRgba(const FlowResult& result) {
        mVisMat.setTo(cv::Scalar(0, 0, 0, 0));

        for (const auto& fv : result.vectors) {
            const float dx  = fv.dx * ARROW_SCALE;
            const float dy  = fv.dy * ARROW_SCALE;
            const float mag = std::sqrt(dx * dx + dy * dy);

            const cv::Point start(static_cast<int>(fv.x0),
                                  static_cast<int>(fv.y0));
            const cv::Point end(static_cast<int>(fv.x0 + dx),
                                static_cast<int>(fv.y0 + dy));

            if (mag < 0.5f) {
                cv::circle(mVisMat, start, 2, cv::Scalar(60, 60, 60, 255), -1);
            } else {
                const float angle = std::fmod(
                        fv.angle * (180.0f / static_cast<float>(M_PI)) + 360.0f, 360.0f);
                const float hue = angle / 2.0f;
                const float val = std::min(fv.magnitude / MAX_FLOW_MAG, 1.0f) * 255.0f;

                cv::Mat3b hsv(1, 1, cv::Vec3b(
                        static_cast<uint8_t>(hue), 255, static_cast<uint8_t>(val)));
                cv::Mat3b bgr;
                cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
                const cv::Vec3b& c = bgr(0, 0);

                cv::arrowedLine(mVisMat, start, end,
                                cv::Scalar(c[0], c[1], c[2], 255),
                                LINE_THICKNESS, cv::LINE_AA, 0, ARROW_TIP_FRAC);
            }
        }

        const int w = mWidth;
        const int h = mHeight;
        const cv::Scalar green(0, 255, 0, 255);
        cv::line(mVisMat, {w / 3,     0}, {w / 3,     h}, green, 2, cv::LINE_AA);
        cv::line(mVisMat, {2 * w / 3, 0}, {2 * w / 3, h}, green, 2, cv::LINE_AA);
        cv::line(mVisMat, {0, h / 3},     {w, h / 3},     green, 2, cv::LINE_AA);
        cv::line(mVisMat, {0, 2 * h / 3}, {w, 2 * h / 3}, green, 2, cv::LINE_AA);

        cv::Mat output;
        switch (mRenderRotation) {
            case 90:  cv::rotate(mVisMat, output, cv::ROTATE_90_CLOCKWISE);        break;
            case 180: cv::rotate(mVisMat, output, cv::ROTATE_180);                 break;
            case 270: cv::rotate(mVisMat, output, cv::ROTATE_90_COUNTERCLOCKWISE); break;
            default:  output = mVisMat;                                             break;
        }

        const uint8_t* ptr = output.data;
        return {ptr, ptr + output.total() * 4};
    }

} // namespace assistivenav