#pragma once

#include "FlowTypes.h"

namespace assistivenav {

    // ─────────────────────────────────────────────────────────────────────────
    // FlowClassifier
    //
    // Assigns an anomalyScore ∈ [0,1] to every FlowVector in-place.
    //
    //   0 → vector is fully explained by camera ego-motion (floor, ceiling,
    //         distant walls moving radially away from the FOE).
    //   1 → vector cannot be explained by ego-motion: an object with
    //         independent motion, or a surface significantly closer than
    //         the scene background (pillar, person, approaching wall).
    //
    // Two independent cues are combined:
    //
    //  A. ANGULAR ANOMALY
    //     During forward translation every background point flows radially
    //     AWAY from the Focus of Expansion (FOE).  A vector whose direction
    //     deviates from that radial prediction is obstacle-induced.
    //     E.g. someone walking toward the user produces convergent flow;
    //     a pole to the side produces strong lateral flow.
    //
    //  B. MAGNITUDE ANOMALY
    //     For a given angular distance from the FOE, background flow magnitude
    //     scales with ego-speed.  A surface that is closer than the background
    //     (pillar, close wall) produces larger-than-expected flow at the same
    //     angular position.  We estimate the background ego-motion rate from
    //     vectors with low angular deviation, then flag vectors whose magnitude
    //     exceeds that rate.
    //
    //  Spatial priors are also applied:
    //    • Bottom of frame (chest-mount → floor) receives heavy suppression
    //      when the flow is also radially consistent (doubly confirmed floor).
    //    • Top of frame (ceiling) receives moderate suppression.
    //    • Navigation zone (roughly centre) is unsuppressed.
    //
    // Must be called AFTER ImuFusion::compensate() and GridAnalyzer::analyze()
    // (so rotational ego-motion is already subtracted and the current-frame FOE
    // is available).
    // ─────────────────────────────────────────────────────────────────────────

    class FlowClassifier {
    public:
        FlowClassifier()  = default;
        ~FlowClassifier() = default;

        FlowClassifier(const FlowClassifier&)            = delete;
        FlowClassifier& operator=(const FlowClassifier&) = delete;

        /** Tag every vector in result.vectors with an anomalyScore.
         *  grid must be the result from GridAnalyzer::analyze() for the same
         *  frame so that the current FOE estimate is available. */
        void classify(FlowResult& result, const GridResult& grid,
                      int width, int height) const;

    private:
        // ── Tuning constants ──────────────────────────────────────────────────

        // Angular deviation (radians) below which a vector is treated as
        // background for the purpose of estimating the ego-motion scale.
        // 35° gives good separation between background and lateral obstacles.
        static constexpr float kBgAngleThreshRad    = 0.61f;

        // Minimum number of low-deviation background vectors required before
        // the magnitude-anomaly cue is considered reliable.
        static constexpr int   kMinBgVectors        = 8;

        // Minimum pixel distance from the FOE for the angular test to be
        // meaningful (avoids numerical singularity very close to the FOE).
        static constexpr float kMinFoeDistPx        = 20.0f;

        // A magnitude ratio of this value or higher maps to full magnitude
        // anomaly (1.0). Ratio = actual_mag / expected_ego_mag.
        // 3× expected → clear obstacle (surface 3× closer than background).
        static constexpr float kAnomalyRatioCap     = 5.0f;

        // Normalised Y thresholds (0 = top, 1 = bottom of frame).
        // For a chest-mounted camera pointing roughly forward:
        //   • y > kFloorYThresh  →  floor zone
        //   • y < kCeilYThresh   →  ceiling zone
        static constexpr float kFloorYThresh        = 0.85f;
        static constexpr float kCeilYThresh         = 0.20f;

        // Relative contribution of each cue to the final score.
        static constexpr float kAngleWeight         = 0.50f;
        static constexpr float kMagWeight           = 0.50f;
    };

} // namespace assistivenav