#pragma once

#include <vector>
#include <array>
#include <cstdint>

namespace assistivenav {

    // ── Optical-flow output ───────────────────────────────────────────────────

    struct FlowVector {
        float x0, y0;    // pixel origin of the tracked point
        float dx, dy;    // frame-to-frame displacement (next − prev)
        float magnitude; // sqrt(dx²+dy²), pre-computed to avoid repeating it
        float angle;     // atan2(dy,dx) in radians, pre-computed for the same reason
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
        float   foeX;
        float   foeY;
        bool    foeValid;
        int64_t timestampNs;
    };

    // ── Obstacle tracking output ──────────────────────────────────────────────

    // Maximum number of simultaneously tracked obstacles.  5 is sufficient for
    // practical navigation and keeps audio sources well below the point where
    // spatial overlap degrades localisation quality for the user.
    static constexpr int kMaxObstacles = 5;

    struct TrackedObstacle {
        float normX;       // horizontal centroid, normalised [0,1] (0=left, 1=right)
        float normY;       // vertical centroid,   normalised [0,1] (0=top,  1=bottom)
        float smoothedMag; // EMA-filtered mean flow magnitude (px/frame)
        int   age;         // consecutive frames this obstacle has been matched
        int   missedFrames;// consecutive frames without a matching blob (hysteresis)
        bool  active;      // true = slot occupied
    };

    struct ObstacleFrame {
        std::array<TrackedObstacle, kMaxObstacles> obstacles;
        int     activeCount;  // number of slots where active == true
        int64_t timestampNs;
    };

} // namespace assistivenav