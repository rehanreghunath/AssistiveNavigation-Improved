#include "GridAnalyzer.h"
#include <android/log.h>
#include <algorithm>
#include <cmath>

#define LOG_TAG "GridAnalyzer"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    GridAnalyzer::GridAnalyzer(int width, int height)
            : mWidth(width), mHeight(height)
    {
        LOGI("Created for %dx%d", mWidth, mHeight);
    }

    int GridAnalyzer::cellIndex(float x, float y) const {
        const int col = static_cast<int>(
                std::clamp(x * 3.0f / static_cast<float>(mWidth),  0.0f, 2.0f));
        const int row = static_cast<int>(
                std::clamp(y * 3.0f / static_cast<float>(mHeight), 0.0f, 2.0f));
        return row * 3 + col;
    }

    void GridAnalyzer::accumulateCells(const FlowResult&           flow,
                                       std::array<CellMetrics, 9>& cells) const {
        float sumMag[9] = {};
        float sumSin[9] = {};
        float sumCos[9] = {};
        int   counts[9] = {};

        for (const FlowVector& fv : flow.vectors) {
            const int idx = cellIndex(fv.x0, fv.y0);
            sumMag[idx] += fv.magnitude;
            sumSin[idx] += std::sin(fv.angle);
            sumCos[idx] += std::cos(fv.angle);
            ++counts[idx];
        }

        for (int i = 0; i < 9; ++i) {
            CellMetrics& c = cells[i];
            c.sampleCount  = counts[i];

            if (counts[i] == 0) {
                c.meanMag    = 0.0f;
                c.meanAngle  = 0.0f;
                c.dangerScore = 0.0f;
                c.ttc        = 0.0f;
                continue;
            }

            const float n = static_cast<float>(counts[i]);
            c.meanMag     = sumMag[i] / n;

            c.meanAngle = std::atan2(sumSin[i], sumCos[i]);

            const float netMag = std::max(c.meanMag - kMinActionableMag, 0.0f);
            c.dangerScore      = std::min(netMag / kDangerNormScale, 1.0f);

            c.ttc = (c.meanMag > kMinActionableMag)
                    ? kTTCScale / c.meanMag
                    : 0.0f;
        }
    }


    void GridAnalyzer::computeFOE(const FlowResult& flow,
                                  float& outX, float& outY, bool& outValid) const {
        outValid = false;

        if (static_cast<int>(flow.vectors.size()) < kMinVectorsForFOE) return;

        float ata00 = 0.0f;
        float ata11 = 0.0f;
        float ata01 = 0.0f;
        float atb0  = 0.0f;
        float atb1  = 0.0f;

        for (const FlowVector& fv : flow.vectors) {
            if (fv.magnitude < kMinActionableMag) continue;

            const float dx  = fv.dx;
            const float dy  = fv.dy;
            const float rhs = dy * fv.x0 - dx * fv.y0;

            ata00 += dy * dy;
            ata11 += dx * dx;
            ata01 += dx * dy;
            atb0  += dy * rhs;
            atb1  -= dx * rhs;
        }

        const float det = ata00 * ata11 - ata01 * ata01;

        if (std::abs(det) < 1e-3f) return;

        const float invDet = 1.0f / det;
        outX = (ata11 * atb0 + ata01 * atb1) * invDet;
        outY = (ata00 * atb1 + ata01 * atb0) * invDet;

        outX    /= static_cast<float>(mWidth);
        outY    /= static_cast<float>(mHeight);
        outValid = true;
    }


    GridResult GridAnalyzer::analyze(const FlowResult& flow) const {
        GridResult result{};
        result.timestampNs = flow.timestampNs;
        result.foeValid    = false;

        accumulateCells(flow, result.cells);
        computeFOE(flow, result.foeX, result.foeY, result.foeValid);

        return result;
    }

} // namespace assistivenav