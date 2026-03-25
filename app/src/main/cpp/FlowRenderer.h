#pragma once

#include <GLES3/gl3.h>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdint>
#include "FlowEngine.h"

namespace assistivenav {
    class FlowRenderer {
    public:
        struct DrawStats {
            float fps;
            int trackedCount;
            int totalPts;
        };

        FlowRenderer();
        ~FlowRenderer();

        void onSurfaceCreated();
        void onSurfaceChanged(int width, int height);
        DrawStats onDrawFrame();
        void setFrameSize(int w, int h);
        void submitFlowResult(const FlowResult& result);

        FlowRenderer(const FlowRenderer&) = delete;
        FlowRenderer& operator = (const FlowRenderer&) = delete;

    private:
        struct Vertex {float x,y,r,g,b;};
        static_assert(sizeof(Vertex) == 5*sizeof(float),
        "Vertex must be tightly packed - no compiler padding allowed");

        GLuint mProgram  = 0;
        GLuint mArrowVBO = 0;
        GLuint mArrowVAO = 0;
        GLuint mGridVBO  = 0;
        GLuint mGridVAO  = 0;

        int mWidth = 1;
        int mHeight = 1;

        int mFrameWidth  = 640;
        int mFrameHeight = 480;

        std::vector<Vertex> mBuildBuffer;
        std::vector<Vertex> mPendingBuffer;
        std::vector<Vertex> mActiveBuffer;
        std::mutex mSwapMutex;
        std::atomic<bool> mHasNewData {false};

        int   mPendingTracked = 0,  mPendingTotal = 0;
        int   mActiveTracked  = 0,  mActiveTotal  = 0;

        using Clock     = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;
        static constexpr int kFpsWindow = 30;
        TimePoint mFrameTs[kFpsWindow]{};
        int       mFpsHead  = 0;
        int       mFpsCount = 0;
        float     mFps      = 0.0f;

        static constexpr int   kMaxArrows      = 200;
        static constexpr int   kVertsPerArrow  = 6;     // 2 shaft + 2 head wings * 2
        static constexpr int   kGridVerts      = 8;     // 4 lines x 2 endpoints
        static constexpr float kArrowScale     = 4.0f;  // visual multiplier
        static constexpr float kHeadAngle      = 0.40f; // half-angle of arrowhead (~23°)
        static constexpr float kHeadFrac       = 0.35f; // head length as fraction of shaft
        static constexpr float kMinNdcLen      = 0.003f;// skip arrows shorter than this
        static constexpr float kMaxFlowMag     = 30.0f; // magnitude that maps to full color value

        void buildProgram();
        void allocArrowVBO();
        void buildGridVBO();
        void setupVAO(GLuint& vao, GLuint vbo);
        void buildArrowVertices(const FlowResult& result);
        void updateFps();


        inline float ndcX(float px) const {
            return (px / static_cast<float>(mFrameWidth))  * 2.0f - 1.0f;
        }
        inline float ndcY(float py) const {
            return 1.0f - (py / static_cast<float>(mFrameHeight)) * 2.0f;
        }

        static void hsvToRgb(float h_deg, float s, float v, float& r, float& g, float& b);
        static GLuint compileShader(GLenum type, const char* src);

    };
}
