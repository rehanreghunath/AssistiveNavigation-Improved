#pragma once

#include "FlowTypes.h"

namespace assistivenav {

    // ─────────────────────────────────────────────────────────────────────────
    // GridAnalyzer
    //
    // Bins IMU-compensated flow vectors into a 3×3 grid and computes per-cell
    // threat metrics plus a scene-wide Focus of Expansion (FOE).
    //
    // Changes from original design:
    //
    //  ISSUE 1 — RANSAC FOE
    //    The original least-squares fit over ALL vectors was biased whenever
    //    obstacle vectors were present (they pull the FOE toward themselves).
    //    RANSAC finds a hypothesis from 2 vectors and counts geometric inliers,
    //    so obstacle vectors — which do NOT pass through the true FOE — are
    //    naturally excluded.  The final FOE is a least-squares refit over
    //    inliers only, giving a clean background-only estimate.
    //
    //    Runtime: 30 iterations × 150 vectors = 4 500 cheap cross-product
    //    evaluations.  Measured at < 0.3 ms on Helio G80 at 640×480.
    //
    //  ISSUE 2 — Temporal baseline / same-speed-mover hint
    //    A person walking at the same speed as the user produces near-zero
    //    RELATIVE optical flow — they look identical to a background surface
    //    at the same depth.  Pure single-frame flow cannot distinguish them.
    //
    //    GridAnalyzer now maintains a per-cell EMA of mean magnitude.  When
    //    a cell's current magnitude DIVERGES from its recent baseline (the
    //    mover starts, stops, or changes speed), the change is detected as a
    //    temporal anomaly and folded into dangerScore as a multiplicative boost.
    //    This catches the transition moments and is the strongest passive cue
    //    available without a separate object detector.
    //
    //    A full ML-based fallback (MobileNet SSD) is the correct long-term
    //    solution; the temporal heuristic here is a lightweight complement.
    //
    // Thread-safety: analyze() is non-const (updates EMA state).  Call only
    // from the camera executor thread, consistent with the rest of the pipeline.
    // ─────────────────────────────────────────────────────────────────────────

    class GridAnalyzer {
    public:
        GridAnalyzer(int width, int height);
        ~GridAnalyzer() = default;

        GridAnalyzer(const GridAnalyzer&)            = delete;
        GridAnalyzer& operator=(const GridAnalyzer&) = delete;

        /** Bin flow vectors into a 3×3 grid, compute per-cell metrics, estimate
         *  the FOE via RANSAC, and update the temporal baseline for same-speed
         *  mover detection.  Not const — mutates EMA state. */
        GridResult analyze(const FlowResult& flow);

    private:
        const int mWidth;
        const int mHeight;

        // ── Temporal baseline state (issue 2) ─────────────────────────────────
        // Slow EMA of per-cell mean magnitude tracks what "normal" looks like.
        // A sudden deviation (temporal anomaly) boosts the cell's dangerScore.

        // Fast alpha during warm-up so the baseline converges quickly.
        static constexpr float kEmaAlphaWarm  = 0.30f;
        // Slow alpha once the baseline is established.
        static constexpr float kEmaAlphaSteady = 0.08f;
        // Switch from warm to steady after this many frames.
        static constexpr int   kWarmupFrames   = 20;
        // Temporal anomaly threshold: deviation relative to baseline.
        // 0.4 = current mag is 40% higher than the cell's long-term mean.
        static constexpr float kTemporalAnomalyThresh = 0.40f;
        // Maximum boost applied to dangerScore from temporal anomaly.
        static constexpr float kTemporalBoostMax      = 1.60f;

        float mCellMagEma[9];  // per-cell EMA of meanMag
        int   mFrameCount;     // total frames processed (governs warm-up)

        // ── Flow thresholds ───────────────────────────────────────────────────
        static constexpr float kMinActionableMag = 2.0f;
        static constexpr float kDangerNormScale  = 20.0f;
        static constexpr float kTTCScale         = 150.0f;

        // ── RANSAC FOE constants (issue 1) ────────────────────────────────────
        // Maximum number of input vectors to consider.  Matches FlowEngine's
        // kMaxFeatures=200 with a small margin.
        static constexpr int   kMaxVecBuf            = 256;
        // Minimum vectors for any FOE estimation to be attempted.
        static constexpr int   kMinVectorsForFOE      = 10;
        // Number of RANSAC iterations.  30 gives > 99% success probability
        // when ≥ 50% of vectors are background inliers (p_inlier = 0.5,
        // k = log(0.01) / log(1 - 0.5²) ≈ 16 — we use 30 for robustness).
        static constexpr int   kRansacIters           = 30;
        // Perpendicular distance (pixels) from the candidate FOE to the flow
        // line that separates inliers from outliers.
        // At 640×480, 2.5 px ≈ 0.5% of frame width.
        static constexpr float kRansacInlierThreshPx  = 2.5f;
        // Minimum number of inliers required before the refit is trusted.
        static constexpr int   kRansacMinInliers       = 10;
        // If this fraction of vectors are inliers, stop iterating early.
        static constexpr float kRansacEarlyExitFrac    = 0.50f;

        int cellIndex(float x, float y) const;

        void accumulateCells(const FlowResult& flow,
                             std::array<CellMetrics, 9>& cells) const;

        /** RANSAC + least-squares refit on inliers.
         *  Obstacle vectors do not pass through the true FOE and are naturally
         *  excluded as outliers, unlike the previous all-vectors LS approach. */
        void computeFOE(const FlowResult& flow,
                        float& outX, float& outY, bool& outValid) const;

        /** Update per-cell EMA and fold temporal anomaly into dangerScore.
         *  Must be called after accumulateCells(). */
        void updateTemporalBaseline(std::array<CellMetrics, 9>& cells);
    };

} // namespace assistivenav