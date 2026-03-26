#pragma once

#include "FlowTypes.h"

namespace assistivenav {

    class GridAnalyzer {
    public:
        GridAnalyzer(int width, int height);
        ~GridAnalyzer() = default;

        GridAnalyzer(const GridAnalyzer&)            = delete;
        GridAnalyzer& operator=(const GridAnalyzer&) = delete;

        /** Bin flow vectors into a 3×3 grid and compute per-cell threat metrics.
         *  Returns a fully populated GridResult.  Safe to call every frame. */
        GridResult analyze(const FlowResult& flow) const;

    private:
        int mWidth;
        int mHeight;

        static constexpr float kMinActionableMag = 2.0f;

        static constexpr float kDangerNormScale  = 20.0f;

        static constexpr float kTTCScale         = 150.0f;

        static constexpr int   kMinVectorsForFOE = 10;

        int cellIndex(float x, float y) const;

        void accumulateCells(const FlowResult& flow,
                             std::array<CellMetrics, 9>& cells) const;

        void computeFOE(const FlowResult& flow,
                        float& outX, float& outY, bool& outValid) const;
    };

} // namespace assistivenav