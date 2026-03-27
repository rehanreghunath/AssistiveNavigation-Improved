#pragma once

#include "FlowTypes.h"

namespace assistivenav {

    // ─────────────────────────────────────────────────────────────────────────
    // ObstacleTracker
    //
    // Converts a per-frame set of IMU-compensated flow vectors into a small
    // list of temporally stable, spatially localised obstacles.
    //
    // Algorithm (all O(N) in flow vector count, no heap allocation):
    //
    //   1. ACTIVITY GRID — bin vectors into a 12×8 grid.  A cell is "active"
    //      if it contains >= kMinVectors vectors with mean magnitude >=
    //      kMinActionableMag after IMU compensation.
    //
    //   2. BLOB EXTRACTION — 4-connected flood-fill on the 96-cell activity
    //      grid produces compact blobs.  Each blob's centroid is the
    //      magnitude-weighted mean of its active cells.
    //
    //   3. MATCHING — greedy nearest-centroid matching assigns each blob to
    //      an existing tracked obstacle within kMatchRadius.  Unmatched blobs
    //      create new obstacle slots; unmatched obstacles accumulate
    //      missedFrames and are retired after kMaxMissedFrames.
    //
    //   4. SMOOTHING — obstacle position and magnitude are EMA-filtered so
    //      that single-frame noise spikes do not propagate to the audio layer.
    //
    // The resulting ObstacleFrame feeds AudioEngine::update() directly.
    // ─────────────────────────────────────────────────────────────────────────

    class ObstacleTracker {
    public:
        ObstacleTracker(int width, int height);
        ~ObstacleTracker() = default;

        ObstacleTracker(const ObstacleTracker&)            = delete;
        ObstacleTracker& operator=(const ObstacleTracker&) = delete;

        /** Consume one frame of compensated flow vectors and return the current
         *  set of tracked obstacles.  Safe to call every frame. */
        ObstacleFrame update(const FlowResult& flow);

    private:
        const int mWidth;
        const int mHeight;

        // Persistent obstacle state — updated in-place each frame.
        std::array<TrackedObstacle, kMaxObstacles> mObstacles;

        // ── Analysis grid ─────────────────────────────────────────────────────
        static constexpr int kGridCols = 12;
        static constexpr int kGridRows = 8;
        static constexpr int kGridCells = kGridCols * kGridRows; // 96

        // ── Thresholds ────────────────────────────────────────────────────────

        // A cell needs at least this many vectors to be considered active.
        // 2 filters single-point noise while allowing thinly textured regions.
        static constexpr int   kMinVectors       = 2;

        // Below this mean magnitude (px/frame) a cell is background, not obstacle.
        static constexpr float kMinActionableMag = 2.0f;

        // Normalised distance threshold for matching a blob to an existing obstacle.
        static constexpr float kMatchRadius      = 0.25f;

        // Audio does not start until an obstacle has been continuously tracked
        // for this many frames, preventing transient flow bursts from triggering cues.
        static constexpr int   kMinAgeForAudio   = 3;

        // After this many consecutive unmatched frames the obstacle slot is freed.
        static constexpr int   kMaxMissedFrames  = 6;

        // EMA weights for position and magnitude smoothing.
        // Lower = smoother but laggier; these values balance responsiveness vs. jitter.
        static constexpr float kAlphaMag = 0.3f;
        static constexpr float kAlphaPos = 0.4f;

        // ── Intermediate blob type (stack-only, not exposed) ──────────────────
        struct Blob {
            float normX;    // magnitude-weighted centroid, normalised [0,1]
            float normY;
            float meanMag;
            int   cellCount;
            bool  matched;  // set true during matching pass to avoid double-use
        };

        static constexpr int kMaxBlobs = 8;

        /** Return the flat cell index for a given flow vector origin. */
        int cellIndex(float x, float y) const;

        /** Populate the activity grid, then flood-fill blobs into out[]. */
        void extractBlobs(const FlowResult& flow,
                          Blob out[kMaxBlobs], int& outCount) const;

        /** Match blobs to mObstacles, update survivors, retire missed ones. */
        void matchAndUpdate(const Blob blobs[kMaxBlobs], int blobCount,
                            int64_t timestampNs);
    };

} // namespace assistivenav