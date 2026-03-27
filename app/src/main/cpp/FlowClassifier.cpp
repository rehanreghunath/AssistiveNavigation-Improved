#include "FlowClassifier.h"
#include <android/log.h>
#include <cmath>
#include <algorithm>

#define LOG_TAG "FlowClassifier"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    void FlowClassifier::classify(FlowResult& result, const GridResult& grid,
                                  int width, int height) const {
        const int N = static_cast<int>(result.vectors.size());
        if (N == 0) return;

        const float imgW = static_cast<float>(width);
        const float imgH = static_cast<float>(height);

        // ── No FOE: apply spatial prior only ─────────────────────────────────
        // This happens on the first processed frame or in textureless scenes.
        // Without a FOE we cannot compute angular anomaly; fall back to a
        // moderate score so downstream thresholds are not trivially met.
        if (!grid.foeValid) {
            for (auto& fv : result.vectors) {
                const float yNorm = fv.y0 / imgH;
                float suppress = 0.0f;
                if (yNorm > kFloorYThresh)
                    suppress = (yNorm - kFloorYThresh) / (1.0f - kFloorYThresh) * 0.7f;
                else if (yNorm < kCeilYThresh)
                    suppress = (kCeilYThresh - yNorm) / kCeilYThresh * 0.5f;
                fv.anomalyScore = 0.4f * (1.0f - suppress);
            }
            return;
        }

        const float foeX_px = grid.foeX * imgW;
        const float foeY_px = grid.foeY * imgH;

        // ── Pass 1: estimate background ego-motion scale ──────────────────────
        // For each vector with small angular deviation we compute
        //   magnitude / foe_distance  (= ego-motion rate, px of flow per px of FOE offset)
        // The mean of this ratio across background vectors is our scale estimate.
        // It serves as the "expected magnitude" model for pass 2.
        float bgScaleSum = 0.0f;
        int   bgCount    = 0;

        for (const auto& fv : result.vectors) {
            if (fv.magnitude < 1.0f) continue;   // below noise floor

            const float dxF     = fv.x0 - foeX_px;
            const float dyF     = fv.y0 - foeY_px;
            const float foeDist = std::sqrt(dxF * dxF + dyF * dyF);
            if (foeDist < kMinFoeDistPx) continue;

            const float expectedAngle = std::atan2(dyF, dxF);
            float       diff          = fv.angle - expectedAngle;
            if (diff >  static_cast<float>(M_PI)) diff -= 2.0f * static_cast<float>(M_PI);
            if (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);

            if (std::abs(diff) < kBgAngleThreshRad) {
                bgScaleSum += fv.magnitude / foeDist;
                ++bgCount;
            }
        }

        // If fewer than kMinBgVectors agreed on a radial direction the scene
        // is unusual (very close wall, heavy rotation residual, etc.).
        // Disable the magnitude-anomaly cue to avoid spurious boosts.
        const bool   bgScaleValid = (bgCount >= kMinBgVectors);
        const float  bgScale      = bgScaleValid
                                    ? bgScaleSum / static_cast<float>(bgCount)
                                    : 0.0f;

        // ── Pass 2: score each vector ─────────────────────────────────────────
        for (auto& fv : result.vectors) {
            const float dxF     = fv.x0 - foeX_px;
            const float dyF     = fv.y0 - foeY_px;
            const float foeDist = std::sqrt(dxF * dxF + dyF * dyF);

            // ── A. Angular anomaly ────────────────────────────────────────────
            // Measures how much the flow direction deviates from the radially-
            // outward prediction.  Quadratic curve: small deviations (noise) are
            // suppressed; large deviations (lateral motion, convergence) are
            // sharply flagged.
            float angleAnomaly = 0.5f;   // neutral near the FOE singularity
            if (foeDist >= kMinFoeDistPx && fv.magnitude > 0.5f) {
                const float expectedAngle = std::atan2(dyF, dxF);
                float       diff          = fv.angle - expectedAngle;
                if (diff >  static_cast<float>(M_PI)) diff -= 2.0f * static_cast<float>(M_PI);
                if (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);
                const float t = std::abs(diff) / static_cast<float>(M_PI);
                angleAnomaly = t * t;   // ∈ [0,1], quadratic
            }

            // ── B. Magnitude anomaly ──────────────────────────────────────────
            // How many times larger is this vector than a background vector at
            // the same FOE distance would be?
            // Ratio = 1 → background-consistent.
            // Ratio > 1 → surface closer than background (obstacle).
            float magAnomaly = 0.5f;   // neutral when bgScale unavailable
            if (bgScaleValid && foeDist >= kMinFoeDistPx) {
                const float expectedMag = bgScale * foeDist;
                // Small constant in denominator guards against near-zero expected.
                const float ratio = fv.magnitude / (expectedMag + 0.3f);
                // Map [1, kAnomalyRatioCap] → [0, 1].  Below 1 → 0 (far/slow).
                magAnomaly = std::clamp(
                        (ratio - 1.0f) / (kAnomalyRatioCap - 1.0f), 0.0f, 1.0f);
            }

            // ── C. Spatial prior ──────────────────────────────────────────────
            // For a chest-mounted phone aimed horizontally:
            //   bottom of frame  → floor (navigation hazard from tracking artefacts)
            //   top of frame     → ceiling (rarely a navigation hazard)
            //
            // When a vector is BOTH in the floor zone AND has low angular anomaly
            // (i.e. it agrees with radial ego-motion), it is almost certainly floor
            // texture.  Apply very aggressive suppression in that joint case.
            const float yNorm = fv.y0 / imgH;

            float spatialFactor = 1.0f;

            if (yNorm > kFloorYThresh) {
                if (angleAnomaly < 0.15f) {
                    // Radially consistent flow in floor zone → definitely floor.
                    spatialFactor = 0.08f;
                } else {
                    // Anomalous direction in floor zone (could be a low obstacle,
                    // kerb, etc.) — moderate suppression only.
                    const float depth = (yNorm - kFloorYThresh) / (1.0f - kFloorYThresh);
                    spatialFactor = 1.0f - 0.55f * depth;
                }
            } else if (yNorm < kCeilYThresh) {
                // Ceiling zone: moderate suppression regardless of angle.
                const float depth = (kCeilYThresh - yNorm) / kCeilYThresh;
                spatialFactor = 1.0f - 0.55f * depth;
            }

            // ── Final score ───────────────────────────────────────────────────
            const float raw = kAngleWeight * angleAnomaly + kMagWeight * magAnomaly;
            fv.anomalyScore  = std::clamp(raw * spatialFactor, 0.0f, 1.0f);
        }
    }

} // namespace assistivenav