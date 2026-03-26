#pragma once

#include "FlowTypes.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <cstdint>

namespace assistivenav {

    class FlowEngine {
    public:
        explicit FlowEngine(int width, int height);

        /** Process one Y-plane frame and return tracked flow vectors.
         *  The returned FlowResult is also cached internally so that
         *  renderToRgba() and the JNI bridge can retrieve it on demand. */
        FlowResult processFrame(const uint8_t* yPlane, int rowStride, int64_t timestampNs);

        /** Render the last FlowResult as an RGBA overlay (arrows + grid lines).
         *  Returns a byte vector of size width*height*4. */
        std::vector<uint8_t> renderToRgba(const FlowResult& result);

        /** Set the clockwise rotation (0 / 90 / 180 / 270) applied to the
         *  rendered RGBA frame so it matches the display orientation. */
        void setRenderRotation(int degrees);

        FlowEngine(const FlowEngine&)            = delete;
        FlowEngine& operator=(const FlowEngine&) = delete;

    private:
        int mWidth;
        int mHeight;
        int mRenderRotation = 0;

        cv::Mat mPrevGray;
        cv::Mat mCurrGray;
        cv::Mat mVisMat;

        std::vector<cv::Point2f> mPrevPts;
        std::vector<cv::Point2f> mNextPts;
        std::vector<cv::Point2f> mBackPts;
        std::vector<uchar>       mStatusFwd;
        std::vector<uchar>       mStatusBwd;
        std::vector<float>       mErrFwd;
        std::vector<float>       mErrBwd;

        bool mInitialized;
        int  mFramesSinceRedetect;

        cv::Size        mWinSize;
        cv::TermCriteria mCriteria;

        // Tuning constants
        static constexpr int   kMaxFeatures      = 200;
        static constexpr int   kMinFeatures      = 80;
        static constexpr int   kRedetectInterval = 20;
        static constexpr float kFBThreshold      = 1.5f;
        static constexpr float kMinEigThreshold  = 1e-4f;
        static constexpr int   kGaussKernelSize  = 3;
        static constexpr float kGFTTQuality      = 0.01f;
        static constexpr float kGFTTMinDist      = 15.0f;
        static constexpr int   kGFTTBlockSize    = 7;

        static void preprocessFrame(const cv::Mat& raw, cv::Mat& out);
        void detectFeatures();
        void trackFeatures(FlowResult& result);
    };

} // namespace assistivenav