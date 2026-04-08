#include "GridAnalyzer.h"
#include <android/log.h>
#include <algorithm>
#include <cmath>
#include <cstring>  // memset

#define LOG_TAG "GridAnalyzer"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    GridAnalyzer::GridAnalyzer(int width, int height)
            : mWidth(width), mHeight(height), mFrameCount(0)
    {
        // Zero EMA baselines so the warm-up phase starts from scratch.
        std::memset(mCellMagEma, 0, sizeof(mCellMagEma));
        LOGI("Created for %dx%d", mWidth, mHeight);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  cellIndex
    // ─────────────────────────────────────────────────────────────────────────

    int GridAnalyzer::cellIndex(float x, float y) const {
        const int col = static_cast<int>(
                std::clamp(x * 3.0f / static_cast<float>(mWidth),  0.0f, 2.0f));
        const int row = static_cast<int>(
                std::clamp(y * 3.0f / static_cast<float>(mHeight), 0.0f, 2.0f));
        return row * 3 + col;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  accumulateCells
    //
    //  Optimization: the original code computed sin(angle) and cos(angle) per
    //  vector, which requires two transcendental function calls each.
    //  Since fv.angle = atan2(dy, dx), we have:
    //    sin(angle) = dy / magnitude
    //    cos(angle) = dx / magnitude
    //  Using the raw (dx, dy) components directly replaces ~200 sin/cos calls
    //  per frame with ~200 divisions — about 4× faster on ARM Cortex-A75.
    // ─────────────────────────────────────────────────────────────────────────

    void GridAnalyzer::accumulateCells(const FlowResult&           flow,
                                       std::array<CellMetrics, 9>& cells) const {
        float sumMag[9] = {};
        // Accumulate unit-vector components instead of sin/cos of angle.
        float sumDyN[9] = {};  // Σ (dy / mag)  = Σ sin(angle)
        float sumDxN[9] = {};  // Σ (dx / mag)  = Σ cos(angle)
        int   counts[9] = {};

        for (const FlowVector& fv : flow.vectors) {
            const int idx = cellIndex(fv.x0, fv.y0);
            sumMag[idx] += fv.magnitude;
            if (fv.magnitude > 1e-6f) {
                const float invMag = 1.0f / fv.magnitude;
                sumDyN[idx] += fv.dy * invMag;
                sumDxN[idx] += fv.dx * invMag;
            }
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
            c.meanMag   = sumMag[i] / n;
            // Mean angle from the accumulated unit-vector components.
            c.meanAngle = std::atan2(sumDyN[i], sumDxN[i]);

            const float netMag = std::max(c.meanMag - kMinActionableMag, 0.0f);
            c.dangerScore      = std::min(netMag / kDangerNormScale, 1.0f);

            c.ttc = (c.meanMag > kMinActionableMag)
                    ? kTTCScale / c.meanMag
                    : 0.0f;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  computeFOE  — RANSAC + least-squares refit on inliers
    //
    //  Geometric model: during forward translation every background point
    //  moves radially AWAY from the FOE (X, Y).  The flow line through
    //  point (x0, y0) along direction (dx, dy) satisfies:
    //
    //    dy * (X - x0) - dx * (Y - y0) = 0
    //
    //  The perpendicular distance from candidate FOE (X, Y) to this line is:
    //
    //    d = |(X - x0)*dy - (Y - y0)*dx| / magnitude
    //
    //  Inlier iff d < kRansacInlierThreshPx.
    //  Using the squared form avoids std::abs and one division per vector:
    //
    //    cross² < thresh² * mag²
    //
    //  Two-vector hypothesis: solve the 2×2 system by Cramer's rule.
    //
    //  After RANSAC, a standard least-squares refit over inliers only gives
    //  a more precise FOE with no outlier influence.
    // ─────────────────────────────────────────────────────────────────────────

    void GridAnalyzer::computeFOE(const FlowResult& flow,
                                  float& outX, float& outY, bool& outValid) const {
        outValid = false;

        // ── Collect usable vectors onto the stack (no heap) ───────────────────
        struct Vec {
            float x0, y0, dx, dy, mag;
            float magThreshSq;  // precomputed: thresh² * mag² — avoids recomputing
        };
        Vec  vecs[kMaxVecBuf];
        int  n = 0;

        const float threshSq = kRansacInlierThreshPx * kRansacInlierThreshPx;

        for (const FlowVector& fv : flow.vectors) {
            if (fv.magnitude < kMinActionableMag) continue;
            if (n >= kMaxVecBuf) break;
            vecs[n++] = {
                    fv.x0, fv.y0, fv.dx, fv.dy, fv.magnitude,
                    threshSq * fv.magnitude * fv.magnitude
            };
        }

        if (n < kMinVectorsForFOE) return;

        const int earlyExitCount = static_cast<int>(
                static_cast<float>(n) * kRansacEarlyExitFrac);

        int   bestCount = 0;
        float bestX     = 0.0f;
        float bestY     = 0.0f;

        // Seed the LCG from the frame timestamp so we get different sample
        // pairs each frame (avoiding degenerate repeated draws) while remaining
        // fully deterministic and free of OS RNG overhead.
        uint32_t rng = 0xDEADBEEFu ^ static_cast<uint32_t>(flow.timestampNs);

        for (int iter = 0; iter < kRansacIters; ++iter) {
            // ── Sample two distinct indices ───────────────────────────────────
            rng = rng * 1664525u + 1013904223u;   // Knuth LCG
            const int i = static_cast<int>((rng >> 16) % static_cast<uint32_t>(n));
            rng = rng * 1664525u + 1013904223u;
            int j = static_cast<int>((rng >> 16) % static_cast<uint32_t>(n - 1));
            if (j >= i) ++j;  // ensure distinct

            // ── Two-vector FOE by Cramer's rule ───────────────────────────────
            // Line i: dy_i*(X - x0_i) = dx_i*(Y - y0_i)
            //   →  dy_i*X − dx_i*Y = dy_i*x0_i − dx_i*y0_i  (= ci)
            // Line j: similar (= cj)
            //
            // det = dx_i*dy_j − dx_j*dy_i  (cross product of flow directions)
            // X   = (dx_i*cj − dx_j*ci) / det
            // Y   = (dy_i*cj − dy_j*ci) / det

            const float det = vecs[i].dx * vecs[j].dy - vecs[j].dx * vecs[i].dy;
            if (std::abs(det) < 1e-2f) continue;  // nearly parallel → skip

            const float ci = vecs[i].dy * vecs[i].x0 - vecs[i].dx * vecs[i].y0;
            const float cj = vecs[j].dy * vecs[j].x0 - vecs[j].dx * vecs[j].y0;
            const float invDet = 1.0f / det;
            const float candX  = (vecs[i].dx * cj - vecs[j].dx * ci) * invDet;
            const float candY  = (vecs[i].dy * cj - vecs[j].dy * ci) * invDet;

            // ── Count inliers (squared distance — no std::abs) ────────────────
            int cnt = 0;
            for (int k = 0; k < n; ++k) {
                const float cross = (candX - vecs[k].x0) * vecs[k].dy
                                    - (candY - vecs[k].y0) * vecs[k].dx;
                if (cross * cross < vecs[k].magThreshSq) ++cnt;
            }

            if (cnt > bestCount) {
                bestCount = cnt;
                bestX     = candX;
                bestY     = candY;
                if (cnt >= earlyExitCount) break;  // sufficient consensus
            }
        }

        if (bestCount < kRansacMinInliers) return;

        // ── Refit: least-squares over RANSAC inliers only ─────────────────────
        // Gives a statistically optimal FOE using only the background vectors
        // that RANSAC identified; obstacle vectors have no influence.
        float ata00 = 0.0f, ata11 = 0.0f, ata01 = 0.0f;
        float atb0  = 0.0f, atb1  = 0.0f;

        for (int k = 0; k < n; ++k) {
            const float cross = (bestX - vecs[k].x0) * vecs[k].dy
                                - (bestY - vecs[k].y0) * vecs[k].dx;
            if (cross * cross >= vecs[k].magThreshSq) continue;  // outlier

            const float dx  = vecs[k].dx;
            const float dy  = vecs[k].dy;
            const float rhs = dy * vecs[k].x0 - dx * vecs[k].y0;

            ata00 += dy * dy;
            ata11 += dx * dx;
            ata01 += dx * dy;
            atb0  += dy * rhs;
            atb1  -= dx * rhs;
        }

        const float det2 = ata00 * ata11 - ata01 * ata01;
        if (std::abs(det2) < 1e-3f) return;

        const float invDet2 = 1.0f / det2;
        outX     = (ata11 * atb0 + ata01 * atb1) * invDet2
                   / static_cast<float>(mWidth);
        outY     = (ata00 * atb1 + ata01 * atb0) * invDet2
                   / static_cast<float>(mHeight);
        outValid = true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  updateTemporalBaseline  — issue 2
    //
    //  Maintains a slow EMA of each cell's mean magnitude.  When the current
    //  frame deviates significantly from that baseline, the cell is behaving
    //  differently from its recent history — a same-speed mover starting,
    //  stopping, or changing speed is the primary physical cause.
    //
    //  The anomaly is folded directly into dangerScore as a multiplicative boost
    //  so no API change is required and ObstacleTracker benefits automatically.
    //
    //  Limitation: a mover traveling at a perfectly steady speed indistinguishable
    //  from background will accumulate into the EMA and score zero anomaly.
    //  A dedicated object detector is required to close that gap.
    // ─────────────────────────────────────────────────────────────────────────

    void GridAnalyzer::updateTemporalBaseline(std::array<CellMetrics, 9>& cells) {
        const float alpha = (mFrameCount < kWarmupFrames)
                            ? kEmaAlphaWarm
                            : kEmaAlphaSteady;

        for (int i = 0; i < 9; ++i) {
            CellMetrics& c      = cells[i];
            const float cur     = c.meanMag;
            const float base    = mCellMagEma[i];

            // Temporal anomaly: normalised positive deviation from baseline.
            // Negative deviations (cell getting quieter) are not anomalous for
            // obstacle detection purposes.
            if (base > 0.5f && cur > base) {
                const float relDelta = (cur - base) / (base + 0.5f);
                if (relDelta > kTemporalAnomalyThresh) {
                    // Cell is meaningfully hotter than its baseline.  Scale the
                    // boost linearly from 1.0 at the threshold to kTemporalBoostMax.
                    const float t     = std::min(relDelta / kTemporalAnomalyThresh - 1.0f,
                                                 1.0f);
                    const float boost = 1.0f + t * (kTemporalBoostMax - 1.0f);
                    c.dangerScore     = std::min(c.dangerScore * boost, 1.0f);
                }
            }

            // Update EMA after using the baseline so this frame's value is
            // reflected only in subsequent frames, not in its own score.
            mCellMagEma[i] = alpha * cur + (1.0f - alpha) * base;
        }

        ++mFrameCount;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  analyze  — public entry point
    // ─────────────────────────────────────────────────────────────────────────

    GridResult GridAnalyzer::analyze(const FlowResult& flow) {
        GridResult result{};
        result.timestampNs = flow.timestampNs;
        result.foeValid    = false;

        accumulateCells(flow, result.cells);
        updateTemporalBaseline(result.cells);   // issue 2: boost after cells are populated
        computeFOE(flow, result.foeX, result.foeY, result.foeValid);  // issue 1: RANSAC

        return result;
    }

} // namespace assistivenav