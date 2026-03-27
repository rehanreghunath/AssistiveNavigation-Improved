#pragma once

#include <vector>
#include <array>
#include <cstdint>

namespace assistivenav {

    // ── Optical-flow output ───────────────────────────────────────────────────

    struct FlowVector {
        float x0, y0;       // pixel origin of the tracked point
        float dx, dy;       // frame-to-frame displacement (next − prev)
        float magnitude;    // sqrt(dx²+dy²), pre-computed
        float angle;        // atan2(dy,dx) in radians
        // [0,1]: 0 = fully explained by camera ego-motion (floor/far wall),
        //         1 = cannot be explained by ego-motion (obstacle).
        // Populated by FlowClassifier after ImuFusion; zero-initialised in FlowEngine.
        float anomalyScore = 0.0f;
    };

    struct FlowResult {
        std::vector<FlowVector> vectors;
        int     trackedCount;
        int     totalPts;
        float   globalMeanMag;
        int64_t timestampNs;
        bool    isFirstFrame;
    };

    // ── Grid-analysis output ──────────────────────────────────────────────────

    struct CellMetrics {
        float meanMag;
        float meanAngle;
        float dangerScore;
        float ttc;
        int   sampleCount;
    };

    struct GridResult {
        std::array<CellMetrics, 9> cells;
        float   foeX;      // normalised [0,1]
        float   foeY;      // normalised [0,1]
        bool    foeValid;
        int64_t timestampNs;
    };

    // ── Obstacle tracking output ──────────────────────────────────────────────

    static constexpr int kMaxObstacles = 5;

    struct TrackedObstacle {
        float normX;            // horizontal centroid, normalised [0,1]
        float normY;            // vertical centroid,   normalised [0,1]
        float smoothedMag;      // EMA-filtered anomaly-weighted flow magnitude (px/frame)
        // Mean anomaly score of the blob's constituent vectors, EMA-smoothed.
        // Reflects how likely the detection is a real obstacle vs ego-motion artefact.
        float confidenceScore = 0.0f;
        int   age;
        int   missedFrames;
        bool  active;
    };

    struct ObstacleFrame {
        std::array<TrackedObstacle, kMaxObstacles> obstacles;
        int     activeCount;
        int64_t timestampNs;
    };

} // namespace assistivenav