#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <cstdint>

namespace assistivenav {
    struct FlowVector {
        float x0, y0;
        float dx, dy;
        float magnitude;
        float angle;
    };

    struct FlowResult {
        std::vector<FlowVector> vectors;
        int trackedCount;
        int totalPts;
        float globalMeanMag;
        int64_t timestampNs;
        bool isFirstFrame;
    };

    class FlowEngine {
    public:
        explicit FlowEngine(int width, int height);

        FlowResult processFrame(const uint8_t* yPlane, int rowStride, int64_t timestampNs);
        std::vector<uint8_t> renderToRgba(const FlowResult& result);
        void setRenderRotation(int degrees);

        FlowEngine(const FlowEngine&) = delete;
        FlowEngine& operator = (const FlowEngine&) = delete;

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
        std::vector<uchar> mStatusFwd;
        std::vector<uchar> mStatusBwd;
        std::vector<float> mErrFwd;
        std::vector<float> mErrBwd;

        bool mInitialized;
        int mFramesSinceRedetect;

        cv::Size mWinSize;
        cv::TermCriteria mCriteria;

        static constexpr int   kMaxFeatures      = 200;
        static constexpr int   kMinFeatures      = 80;     // re-detect if below this
        static constexpr int   kRedetectInterval = 20;     // forced re-detect every N frames
        static constexpr float kFBThreshold      = 1.5f;   // max FB error (pixels) to keep a track
        static constexpr float kMinEigThreshold  = 1e-4f;  // LK min-eigenvalue filter
        static constexpr int   kGaussKernelSize  = 3;      // Gaussian blur kernel (must be odd)
        static constexpr float kGFTTQuality      = 0.01f;  // goodFeaturesToTrack quality level
        static constexpr float kGFTTMinDist      = 15.0f;  // min px between detected corners
        static constexpr int   kGFTTBlockSize    = 7;      // neighbourhood size for corner score

        void preprocessFrame(const cv::Mat& raw, cv::Mat& out) const;
        void detectFeatures();
        void trackFeatures(FlowResult& result);


    };
}

