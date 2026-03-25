#include "FlowRenderer.h"
#include <android/log.h>
#include <cmath>

#define LOG_TAG "FlowRenderer"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

#define CHECK_GL(stmt)                                              \
    do {                                                            \
        stmt;                                                       \
        GLenum _err = glGetError();                                 \
        if (_err != GL_NO_ERROR)                                    \
            LOGE("GL error 0x%04x in " #stmt " (%s:%d)", _err,      \
                 __FILE__, __LINE__);                               \
    } while (0)

namespace assistivenav {
    static const char *kVertSrc = R"GLSL(#version 300 es
        layout(location = 0) in vec2 aPos;
        layout(location = 1) in vec3 aColor;
        out vec3 vColor;
        void main() {
        // Depth = 0, w = 1: standard 2-D overlay, no perspective divide needed.
        gl_Position = vec4(aPos, 0.0, 1.0);
        vColor = aColor;
        }
    )GLSL";

    static const char *kFragSrc = R"GLSL(#version 300 es
        precision mediump float;
        in  vec3 vColor;
        out vec4 fragColor;
        void main() {
            // Alpha = 1 for all arrow/grid pixels; the transparent clear color
            // (set in onSurfaceCreated) handles the see-through background.
            fragColor = vec4(vColor, 1.0);
        }
    )GLSL";

    FlowRenderer::FlowRenderer() {
        const int cap = kMaxArrows * kVertsPerArrow;
        mBuildBuffer.reserve(cap);
        mPendingBuffer.reserve(cap);
        mActiveBuffer.reserve(cap);
        LOGI("Created - max %d arrow vertices (%zu bytes each)", cap, sizeof(Vertex));
    }

    FlowRenderer::~FlowRenderer() = default;

    void FlowRenderer::onSurfaceCreated() {
        LOGI("onSurfaceCreated");

        // Delete any leftover GL objects from a previous surface.
        // This happens when the screen rotates or the app is backgrounded/foregrounded.
        if (mProgram) {
            glDeleteProgram(mProgram);
            mProgram = 0;
        }
        if (mArrowVBO) {
            glDeleteBuffers(1, &mArrowVBO);
            mArrowVBO = 0;
        }
        if (mGridVBO) {
            glDeleteBuffers(1, &mGridVBO);
            mGridVBO = 0;
        }
        if (mArrowVAO) {
            glDeleteVertexArrays(1, &mArrowVAO);
            mArrowVAO = 0;
        }
        if (mGridVAO) {
            glDeleteVertexArrays(1, &mGridVAO);
            mGridVAO = 0;
        }

        // Clear color: fully transparent black.
        // Where we don't draw, alpha=0 pixels let the camera PreviewView show through.
        CHECK_GL(glClearColor(0.0f, 0.0f, 0.0f, 0.0f));

        // Blending: needed so the transparent clear works correctly when composited
        // with the SurfaceFlinger (Android's display compositor).
        CHECK_GL(glEnable(GL_BLEND));
        CHECK_GL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

        // Line width: 2px makes arrows readable at phone viewing distance.
        // Note: OpenGL ES spec only guarantees width=1 is supported; 2px works
        // on all real devices but may silently clamp to 1 on some emulators.
        CHECK_GL(glLineWidth(2.0f));

        buildProgram();
        allocArrowVBO();
        buildGridVBO();
        setupVAO(mArrowVAO, mArrowVBO);
        setupVAO(mGridVAO, mGridVBO);

        LOGI("onSurfaceCreated complete, program=%u", mProgram);
    }

    void FlowRenderer::onSurfaceChanged(int width, int height) {
        mWidth = (width > 0) ? width : 1;
        mHeight = (height > 0) ? height : 1;
        CHECK_GL(glViewport(0, 0, mWidth, mHeight));
        LOGI("onSurfaceChanged: %dx%d", mWidth, mHeight);
    }

    FlowRenderer::DrawStats FlowRenderer::onDrawFrame() {

        if (mHasNewData.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(mSwapMutex);
            std::swap(mActiveBuffer, mPendingBuffer);
            mActiveTracked = mPendingTracked;
            mActiveTotal = mPendingTotal;
            mHasNewData.store(false, std::memory_order_release);
        }

        CHECK_GL(glClear(GL_COLOR_BUFFER_BIT));

        CHECK_GL(glUseProgram(mProgram));

        if (!mActiveBuffer.empty()) {
            CHECK_GL(glBindVertexArray(mArrowVAO));
            CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, mArrowVBO));

            const GLsizeiptr bytes =
                    static_cast<GLsizeiptr>(mActiveBuffer.size() * sizeof(Vertex));
            CHECK_GL(glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, mActiveBuffer.data()));

            CHECK_GL(glDrawArrays(GL_LINES, 0,
                                  static_cast<GLsizei>(mActiveBuffer.size())));
            CHECK_GL(glBindVertexArray(0));
        }

        CHECK_GL(glBindVertexArray(mGridVAO));
        CHECK_GL(glDrawArrays(GL_LINES, 0, kGridVerts));
        CHECK_GL(glBindVertexArray(0));

        CHECK_GL(glUseProgram(0));

        updateFps();

        return {mFps, mActiveTracked, mActiveTotal};


    }

    void FlowRenderer::submitFlowResult(const FlowResult &result) {
        buildArrowVertices(result);

        {
            std::lock_guard<std::mutex> lock(mSwapMutex);
            std::swap(mBuildBuffer, mPendingBuffer);
            mPendingTracked = result.trackedCount;
            mPendingTotal   = result.totalPts;
        }

        mHasNewData.store(true, std::memory_order_release);
    }

    void FlowRenderer::setFrameSize(int w, int h) {
        mFrameWidth  = (w > 0) ? w : 1;
        mFrameHeight = (h > 0) ? h : 1;
        LOGI("Frame size set to %dx%d", mFrameWidth, mFrameHeight);
    }

    void FlowRenderer::buildArrowVertices(const FlowResult& result) {
        mBuildBuffer.clear();

        // ── Axis swap explanation ─────────────────────────────────────────────
        //
        // The camera sensor is landscape. CameraX delivers raw YUV_420_888 frames
        // WITHOUT applying the 90° portrait rotation. So for a 640×480 frame
        // on a portrait phone:
        //
        //   Camera X axis (0 → mFrameWidth=640)  runs top → bottom on screen
        //   Camera Y axis (0 → mFrameHeight=480) runs left → right on screen
        //
        // Therefore to convert a camera pixel (cx, cy) to NDC:
        //   ndc_x =  (cy / mFrameHeight) * 2 - 1     ← cy drives horizontal
        //   ndc_y = 1 - (cx / mFrameWidth) * 2        ← cx drives vertical
        //
        // And a flow vector (dx, dy) in camera space maps to NDC displacement:
        //   ndc_dx =  dy / mFrameHeight * 2            ← dy is horizontal motion
        //   ndc_dy = -dx / mFrameWidth  * 2            ← dx is vertical motion (NDC Y flipped)

        const float scaleX = 2.0f / static_cast<float>(mFrameHeight); // dy → NDC X
        const float scaleY = 2.0f / static_cast<float>(mFrameWidth);  // dx → NDC Y

        const float cosH = std::cos(kHeadAngle);
        const float sinH = std::sin(kHeadAngle);

        for (const auto& fv : result.vectors) {

            // ── Origin in NDC (axis-swapped) ──────────────────────────────────
            const float x0 = (fv.y0 / static_cast<float>(mFrameHeight)) * 2.0f - 1.0f;
            const float y0 = (fv.x0 / static_cast<float>(mFrameWidth)) * 2.0f - 1.0f;

            // ── Tip in NDC ────────────────────────────────────────────────────
            // dy drives X (horizontal on screen), dx drives Y (vertical on screen).
            // NDC Y is negated relative to screen Y, so dx is subtracted.
            const float x1 = x0 + fv.dy * kArrowScale * scaleX;
            const float y1 = y0 + fv.dx * kArrowScale * scaleY;

            // ── Skip stationary features ──────────────────────────────────────
            const float sdx = x1 - x0;
            const float sdy = y1 - y0;
            const float len = std::sqrt(sdx * sdx + sdy * sdy);
            if (len < kMinNdcLen) continue;

            // ── Color: HSV → RGB ──────────────────────────────────────────────
            // Recompute angle in the SCREEN-space direction (post-swap) so that
            // the hue correctly reflects the visual direction of the arrow.
            const float screenAngle = std::atan2(-sdx, sdy); // screen: right=0, up=π/2
            const float hDeg = std::fmod(
                    screenAngle * (180.0f / static_cast<float>(M_PI)) + 360.0f, 360.0f);
            const float val = std::min(fv.magnitude / kMaxFlowMag, 1.0f);
            float cr, cg, cb;
            hsvToRgb(hDeg, 1.0f, val, cr, cg, cb);

            // ── Shaft ─────────────────────────────────────────────────────────
            mBuildBuffer.push_back({x0, y0, cr, cg, cb});
            mBuildBuffer.push_back({x1, y1, cr, cg, cb});

            // ── Arrowhead wings ───────────────────────────────────────────────
            const float rx = -sdx / len;
            const float ry = -sdy / len;
            const float hl =  len * kHeadFrac;

            const float w1x = (rx * cosH - ry * sinH) * hl;
            const float w1y = (rx * sinH + ry * cosH) * hl;
            const float w2x = (rx * cosH + ry * sinH) * hl;
            const float w2y = (-rx * sinH + ry * cosH) * hl;

            mBuildBuffer.push_back({x1,       y1,       cr, cg, cb});
            mBuildBuffer.push_back({x1 + w1x, y1 + w1y, cr, cg, cb});
            mBuildBuffer.push_back({x1,       y1,       cr, cg, cb});
            mBuildBuffer.push_back({x1 + w2x, y1 + w2y, cr, cg, cb});
        }
    }

    void FlowRenderer::allocArrowVBO() {
        const GLsizeiptr maxBytes =
                static_cast<GLsizeiptr>(kMaxArrows * kVertsPerArrow * sizeof(Vertex));

        CHECK_GL(glGenBuffers(1, &mArrowVBO));
        CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, mArrowVBO));

        // Allocate GPU memory with nullptr data.
        CHECK_GL(glBufferData(GL_ARRAY_BUFFER, maxBytes, nullptr, GL_DYNAMIC_DRAW));
        CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    }

    void FlowRenderer::buildGridVBO() {
        constexpr float d = 1.0f / 3.0f;
        constexpr float G = 1.0f;

        const Vertex verts[kGridVerts] = {
                // Vertical divider at ndc_x = -1/3  (screen column 1/3)
                {-d, -1.0f,  0, G, 0},  {-d,  1.0f,  0, G, 0},
                // Vertical divider at ndc_x = +1/3  (screen column 2/3)
                { d, -1.0f,  0, G, 0},  { d,  1.0f,  0, G, 0},
                // Horizontal divider at ndc_y = +1/3 (screen row 1/3)
                {-1.0f,  d,  0, G, 0},  { 1.0f,  d,  0, G, 0},
                // Horizontal divider at ndc_y = -1/3 (screen row 2/3)
                {-1.0f, -d,  0, G, 0},  { 1.0f, -d,  0, G, 0},
        };

        CHECK_GL(glGenBuffers(1, &mGridVBO));
        CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, mGridVBO));

        CHECK_GL(glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW));
        CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    }

    void FlowRenderer::setupVAO(GLuint &vao, GLuint vbo) {
        CHECK_GL(glGenVertexArrays(1, &vao));
        CHECK_GL(glBindVertexArray(vao));
        CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, vbo));

        const GLsizei stride = sizeof(Vertex);

        CHECK_GL(glEnableVertexAttribArray(0));
        CHECK_GL(glVertexAttribPointer(
                /*index=*/    0,
                /*size=*/     2,          // components per vertex
                /*type=*/     GL_FLOAT,
                /*normalized*/GL_FALSE,
                /*stride=*/   stride,
                /*offset=*/   reinterpret_cast<void*>(0)
        ));

        CHECK_GL(glEnableVertexAttribArray(1));
        CHECK_GL(glVertexAttribPointer(
                /*index=*/    1,
                /*size=*/     3,
                /*type=*/     GL_FLOAT,
                /*normalized*/GL_FALSE,
                /*stride=*/   stride,
                /*offset=*/   reinterpret_cast<void*>(2 * sizeof(float))
        ));

        CHECK_GL(glBindVertexArray(0));
        CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    }

    void FlowRenderer::buildProgram() {
        const GLuint vert = compileShader(GL_VERTEX_SHADER,   kVertSrc);
        const GLuint frag = compileShader(GL_FRAGMENT_SHADER, kFragSrc);

        mProgram = glCreateProgram();
        CHECK_GL(glAttachShader(mProgram, vert));
        CHECK_GL(glAttachShader(mProgram, frag));
        CHECK_GL(glLinkProgram(mProgram));

        GLint ok = 0;
        glGetProgramiv(mProgram, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetProgramInfoLog(mProgram, sizeof(log), nullptr, log);
            LOGE("Program link error: %s", log);
        }

        // Once linked, the program holds its own copy of the compiled shaders.
        // Detach + delete the shader objects to free their memory.
        glDetachShader(mProgram, vert);
        glDetachShader(mProgram, frag);
        glDeleteShader(vert);
        glDeleteShader(frag);
    }

    GLuint FlowRenderer::compileShader(GLenum type, const char* src) {
        GLuint s = glCreateShader(type);
        CHECK_GL(glShaderSource(s, 1, &src, nullptr));
        CHECK_GL(glCompileShader(s));

        GLint ok = 0;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(s, sizeof(log), nullptr, log);
            LOGE("Shader (%s) compile error: %s",
                 type == GL_VERTEX_SHADER ? "vert" : "frag", log);
        }
        return s;
    }

    void FlowRenderer::updateFps() {
        const auto now = Clock::now();
        mFrameTs[mFpsHead] = now;
        mFpsHead = (mFpsHead + 1) % kFpsWindow;
        if (mFpsCount < kFpsWindow) ++mFpsCount;

        if (mFpsCount >= 2) {
            const int oldIdx = (mFpsHead - mFpsCount + kFpsWindow) % kFpsWindow;
            const int newIdx = (mFpsHead - 1      + kFpsWindow) % kFpsWindow;
            const auto usecs = std::chrono::duration_cast<std::chrono::microseconds>(
                    mFrameTs[newIdx] - mFrameTs[oldIdx]).count();
            if (usecs > 0) {
                mFps = (mFpsCount - 1) * 1'000'000.0f / static_cast<float>(usecs);
            }
        }
    }

    void FlowRenderer::hsvToRgb(float h, float s, float v,
                                float& r, float& g, float& b) {
        if (s <= 0.0f) { r = g = b = v; return; }

        const float hh = std::fmod(h, 360.0f) / 60.0f;
        const int i = static_cast<int>(hh);
        const float f = hh - static_cast<float>(i);
        const float p = v * (1.0f - s);
        const float q = v * (1.0f - s * f);
        const float t = v * (1.0f - s * (1.0f - f));

        switch (i) {
            case 0:  r = v; g = t; b = p; break;
            case 1:  r = q; g = v; b = p; break;
            case 2:  r = p; g = v; b = t; break;
            case 3:  r = p; g = q; b = v; break;
            case 4:  r = t; g = p; b = v; break;
            default: r = v; g = p; b = q; break;
        }
    }

}