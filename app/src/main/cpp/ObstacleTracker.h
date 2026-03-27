#pragma once

#include "FlowTypes.h"

namespace assistivenav {

    // ─────────────────────────────────────────────────────────────────────────
    // ObstacleTracker
    //
    // Converts anomaly-classified, IMU-compensated flow vectors into a small
    // list of temporally stable, spatially localised obstacle detections.
    //
    // Key change vs. the original design: every cell-activity and blob-quality
    // decision is now driven by ANOMALY-WEIGHTED magnitude rather than raw
    // magnitude.  This suppresses floor texture, ceiling reflections, and
    // distant static surfaces that produce ego-motion-consistent flow, while
    // preserving signal from pillars (closer than background → high magnitude
    // anomaly), people approaching (convergent flow → high angular anomaly),
    // and close walls (high combined score).
    //
    // A blob also reports its spatial shape (aspect ratio) so that tall, narrow
    // detections — characteristic of poles and pillars — can be recognised and
    // handled with higher priority.
    //
    // Algorithm:
    //   1. ACTIVITY GRID — bin vectors into a 12×8 grid.  A cell is "active"
    //      if it has >= kMinVectors vectors whose ANOMALY-WEIGHTED mean magnitude
    //      exceeds kMinWeightedMag AND whose mean anomaly score exceeds
    //      kMinMeanAnomaly.
    //   2. BLOB EXTRACTION — 4-connected flood-fill produces compact blobs.
    //      Centroids are magnitude-weighted, biased toward the highest-anomaly
    //      regions.  Bounding box and aspect ratio are tracked for each blob.
    //   3. MATCHING — greedy nearest-centroid within kMatchRadius.
    //   4. SMOOTHING — position, magnitude, and confidence are EMA-filtered.
    //   5. OUTPUT GATE — obstacle is forwarded to AudioEngine only after it has
    //      been tracked for kMinAgeForAudio frames AND its confidenceScore
    //      exceeds kMinBlobAnomaly.
    // ─────────────────────────────────────────────────────────────────────────

    class ObstacleTracker {
    public:
        ObstacleTracker(int width, int height);
        ~ObstacleTracker() = default;

        ObstacleTracker(const ObstacleTracker&)            = delete;
        ObstacleTracker& operator=(const ObstacleTracker&) = delete;

        /** Consume one frame of classified, compensated flow vectors and return
         *  the current set of tracked obstacles.
         *  grid is used only for diagnostics; the anomaly scores embedded in
         *  flow.vectors carry all classification information. */
        ObstacleFrame update(const FlowResult& flow, const GridResult& grid);

    private:
        const int mWidth;
        const int mHeight;

        std::array<TrackedObstacle, kMaxObstacles> mObstacles;

        // ── Analysis grid ─────────────────────────────────────────────────────
        static constexpr int kGridCols  = 12;
        static constexpr int kGridRows  = 8;
        static constexpr int kGridCells = kGridCols * kGridRows; // 96

        // ── Activity thresholds ───────────────────────────────────────────────

        // Minimum flow vectors for a cell to be considered.
        static constexpr int   kMinVectors      = 2;

        // Minimum ANOMALY-WEIGHTED mean magnitude per cell (px/frame).
        // With anomaly weighting, a floor vector (mag=5, anomaly=0.05) contributes
        // 0.25 whereas an obstacle vector (mag=5, anomaly=0.9) contributes 4.5.
        static constexpr float kMinWeightedMag  = 3.0f;

        // Minimum mean anomaly score for a cell to be active.
        // Ensures cells dominated by ego-motion-consistent (floor/wall) vectors
        // are excluded even if their raw magnitude is high.
        static constexpr float kMinMeanAnomaly  = 0.25f;

        // ── Blob / obstacle thresholds ────────────────────────────────────────

        static constexpr float kMatchRadius     = 0.25f;   // normalised distance

        // An obstacle slot accumulates confidence for this many frames before
        // its sound source is unmuted.  Suppresses transient flow bursts.
        static constexpr int   kMinAgeForAudio  = 5;

        // Minimum EMA-smoothed confidence (mean anomaly score of constituent
        // vectors) for the obstacle to be forwarded to AudioEngine.
        static constexpr float kMinBlobAnomaly  = 0.30f;

        // Obstacle slot is freed after this many consecutive unmatched frames.
        static constexpr int   kMaxMissedFrames = 6;

        // Aspect ratio (height / width in grid cells) above which the blob is
        // classified as "pillar-like".  Used to boost confidence for tall,
        // narrow detections.
        static constexpr float kPillarAspect    = 2.2f;

        // EMA weights.
        static constexpr float kAlphaMag        = 0.3f;
        static constexpr float kAlphaPos        = 0.4f;
        static constexpr float kAlphaConf       = 0.3f;

        // ── Internal blob type (stack-only) ───────────────────────────────────
        struct Blob {
            float normX;            // anomaly-weighted centroid, normalised [0,1]
            float normY;
            float meanMag;          // anomaly-weighted mean magnitude (px/frame)
            float meanAnomalyScore; // mean anomaly score across all vectors in blob
            int   cellCount;
            int   minRow, maxRow;   // bounding box in grid coordinates
            int   minCol, maxCol;
            bool  matched;
        };

        static constexpr int kMaxBlobs = 8;

        int  cellIndex(float x, float y) const;

        void extractBlobs(const FlowResult& flow,
                          Blob out[kMaxBlobs], int& outCount) const;

        void matchAndUpdate(const Blob blobs[kMaxBlobs], int blobCount,
                            int64_t timestampNs);
    };

} // namespace assistivenav