#pragma once

#include <vector>
#include <array>
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
        int     trackedCount;   // survivors after forward-backward consistency check
        int     totalPts;       // how many points were attempted
        float   globalMeanMag;  // mean magnitude across all surviving vectors
        int64_t timestampNs;    // sensor capture timestamp from CameraX
        bool    isFirstFrame;   // true on the very first call; no flow data yet
    };

    struct CellMetrics {
        float meanMag;     // mean flow magnitude of all vectors in this cell (px/frame)
        float meanAngle;   // circular mean direction (radians, range −π..π)
        float dangerScore; // [0,1] urgency; 1 = high immediate threat
        float ttc;         // time-to-collision approximation in frames; 0 = no motion
        int   sampleCount; // number of FlowVectors that fell inside this cell
    };

    struct GridResult {
        std::array<CellMetrics, 9> cells;

        float   foeX;     // Focus of Expansion, normalised to [0,1] of frame width
        float   foeY;     // Focus of Expansion, normalised to [0,1] of frame height
        bool    foeValid; // false when vectors are too sparse or purely translational
        int64_t timestampNs;
    };

} // namespace assistivenav