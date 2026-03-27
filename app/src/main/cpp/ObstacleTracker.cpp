#include "ObstacleTracker.h"
#include <android/log.h>
#include <algorithm>
#include <cmath>

#define LOG_TAG "ObstacleTracker"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    ObstacleTracker::ObstacleTracker(int width, int height)
            : mWidth(width), mHeight(height), mObstacles{}
    {
        for (TrackedObstacle& o : mObstacles) o = {};
        LOGI("Created for %dx%d, grid %dx%d, max %d obstacles",
             mWidth, mHeight, kGridCols, kGridRows, kMaxObstacles);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  cellIndex
    // ─────────────────────────────────────────────────────────────────────────

    int ObstacleTracker::cellIndex(float x, float y) const {
        const int col = static_cast<int>(
                std::clamp(x * kGridCols / static_cast<float>(mWidth),
                           0.0f, static_cast<float>(kGridCols - 1)));
        const int row = static_cast<int>(
                std::clamp(y * kGridRows / static_cast<float>(mHeight),
                           0.0f, static_cast<float>(kGridRows - 1)));
        return row * kGridCols + col;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  extractBlobs
    //
    //  Step 1: bin vectors into the 96-cell activity grid using ANOMALY-WEIGHTED
    //          magnitude.  A cell is active only when both its weighted magnitude
    //          AND its mean anomaly score clear their respective thresholds.
    //
    //  Step 2: 4-connected flood-fill to form blobs.  Centroids and shape
    //          descriptors are computed on the stack — no heap allocation.
    // ─────────────────────────────────────────────────────────────────────────

    void ObstacleTracker::extractBlobs(const FlowResult& flow,
                                       Blob out[kMaxBlobs],
                                       int& outCount) const {
        outCount = 0;

        // ── Step 1: accumulate per-cell stats ─────────────────────────────────
        float sumWeightedMag[kGridCells] = {};   // sum(mag * anomalyScore) per cell
        float sumAnomaly    [kGridCells] = {};   // sum(anomalyScore) per cell
        float sumRawMag     [kGridCells] = {};   // sum(magnitude)  — for diagnostics
        int   counts        [kGridCells] = {};

        for (const FlowVector& fv : flow.vectors) {
            const int idx = cellIndex(fv.x0, fv.y0);
            sumWeightedMag[idx] += fv.magnitude * fv.anomalyScore;
            sumAnomaly    [idx] += fv.anomalyScore;
            sumRawMag     [idx] += fv.magnitude;
            ++counts[idx];
        }

        // A cell is "active" only when:
        //  • it has enough vectors to be statistically meaningful
        //  • its mean ANOMALY-WEIGHTED magnitude clears the noise floor
        //  • its mean anomaly score is above the ego-motion threshold
        bool active[kGridCells] = {};
        for (int i = 0; i < kGridCells; ++i) {
            if (counts[i] < kMinVectors) continue;
            const float n = static_cast<float>(counts[i]);
            if ((sumWeightedMag[i] / n) >= kMinWeightedMag &&
                (sumAnomaly[i]     / n) >= kMinMeanAnomaly) {
                active[i] = true;
            }
        }

        // ── Step 2: 4-connected flood-fill ────────────────────────────────────
        bool visited  [kGridCells] = {};
        int  dfsStack [kGridCells];

        const float cellW = static_cast<float>(mWidth)  / kGridCols;
        const float cellH = static_cast<float>(mHeight) / kGridRows;
        const float imgW  = static_cast<float>(mWidth);
        const float imgH  = static_cast<float>(mHeight);

        for (int seed = 0; seed < kGridCells; ++seed) {
            if (!active[seed] || visited[seed]) continue;
            if (outCount >= kMaxBlobs) break;

            // Anomaly-weighted centroid accumulators.
            float wSumX      = 0.0f;
            float wSumY      = 0.0f;
            float wTotal     = 0.0f;   // sum of anomaly-weighted magnitudes

            // Blob quality & shape accumulators.
            float anomalyTotal   = 0.0f;
            int   totalVectors   = 0;
            int   cells          = 0;
            int   minRow = kGridRows, maxRow = -1;
            int   minCol = kGridCols, maxCol = -1;

            int top = 0;
            dfsStack[top++] = seed;
            visited[seed]   = true;

            while (top > 0) {
                const int cur = dfsStack[--top];
                const int row = cur / kGridCols;
                const int col = cur % kGridCols;

                const float cellCX = (col + 0.5f) * cellW;
                const float cellCY = (row + 0.5f) * cellH;
                const float wMag   = sumWeightedMag[cur];

                // Centroid weighted by anomaly-weighted magnitude so that it
                // is attracted toward the highest-anomaly parts of the blob.
                wSumX      += cellCX * wMag;
                wSumY      += cellCY * wMag;
                wTotal     += wMag;
                anomalyTotal += sumAnomaly[cur];
                totalVectors += counts[cur];
                ++cells;

                minRow = std::min(minRow, row);
                maxRow = std::max(maxRow, row);
                minCol = std::min(minCol, col);
                maxCol = std::max(maxCol, col);

                const int neighbours[4] = {
                        (row > 0)             ? cur - kGridCols : -1,
                        (row < kGridRows - 1) ? cur + kGridCols : -1,
                        (col > 0)             ? cur - 1         : -1,
                        (col < kGridCols - 1) ? cur + 1         : -1
                };
                for (const int nb : neighbours) {
                    if (nb >= 0 && active[nb] && !visited[nb]) {
                        visited[nb]      = true;
                        dfsStack[top++]  = nb;
                    }
                }
            }

            if (wTotal < 1e-6f) continue;

            Blob& b          = out[outCount++];
            b.normX          = (wSumX / wTotal) / imgW;
            b.normY          = (wSumY / wTotal) / imgH;
            // Mean anomaly-weighted magnitude per vector: the quantity that will
            // drive the audio gain through smoothedMag.
            b.meanMag        = (totalVectors > 0)
                               ? wTotal / static_cast<float>(totalVectors) : 0.0f;
            b.meanAnomalyScore = (totalVectors > 0)
                                 ? anomalyTotal / static_cast<float>(totalVectors) : 0.0f;
            b.cellCount      = cells;
            b.minRow         = minRow; b.maxRow = maxRow;
            b.minCol         = minCol; b.maxCol = maxCol;
            b.matched        = false;

            // Pillar boost: tall, narrow blobs in the navigation zone get a
            // confidence bump because the shape is a strong prior for a pole.
            const float blobH      = static_cast<float>(maxRow - minRow + 1);
            const float blobW      = static_cast<float>(maxCol - minCol + 1);
            const float aspect     = blobH / std::max(blobW, 1.0f);
            const float normYCtr   = b.normY;  // 0=top, 1=bottom

            // Only boost blobs in the middle third of the frame vertically —
            // tall shapes in the floor zone are more likely to be a step or kerb.
            if (aspect >= kPillarAspect && normYCtr > 0.25f && normYCtr < 0.75f) {
                b.meanAnomalyScore = std::min(b.meanAnomalyScore * 1.35f, 1.0f);
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  matchAndUpdate
    // ─────────────────────────────────────────────────────────────────────────

    void ObstacleTracker::matchAndUpdate(const Blob blobs[kMaxBlobs],
                                         int blobCount,
                                         int64_t /* timestampNs */) {
        bool obstacleMatched[kMaxObstacles] = {};

        for (int b = 0; b < blobCount; ++b) {
            const Blob& blob = blobs[b];

            float bestDist = kMatchRadius;
            int   bestIdx  = -1;

            for (int i = 0; i < kMaxObstacles; ++i) {
                if (!mObstacles[i].active) continue;
                const float dx = blob.normX - mObstacles[i].normX;
                const float dy = blob.normY - mObstacles[i].normY;
                const float d  = std::sqrt(dx * dx + dy * dy);
                if (d < bestDist) { bestDist = d; bestIdx = i; }
            }

            if (bestIdx >= 0) {
                TrackedObstacle& o = mObstacles[bestIdx];
                o.normX          = kAlphaPos  * blob.normX          + (1.0f - kAlphaPos)  * o.normX;
                o.normY          = kAlphaPos  * blob.normY          + (1.0f - kAlphaPos)  * o.normY;
                o.smoothedMag    = kAlphaMag  * blob.meanMag        + (1.0f - kAlphaMag)  * o.smoothedMag;
                o.confidenceScore= kAlphaConf * blob.meanAnomalyScore+(1.0f - kAlphaConf) * o.confidenceScore;
                o.missedFrames   = 0;
                ++o.age;
                obstacleMatched[bestIdx] = true;
            } else {
                for (int i = 0; i < kMaxObstacles; ++i) {
                    if (!mObstacles[i].active) {
                        TrackedObstacle& o = mObstacles[i];
                        o.normX           = blob.normX;
                        o.normY           = blob.normY;
                        o.smoothedMag     = blob.meanMag;
                        o.confidenceScore = blob.meanAnomalyScore;
                        o.age             = 1;
                        o.missedFrames    = 0;
                        o.active          = true;
                        obstacleMatched[i] = true;
                        break;
                    }
                }
            }
        }

        for (int i = 0; i < kMaxObstacles; ++i) {
            if (!mObstacles[i].active || obstacleMatched[i]) continue;
            ++mObstacles[i].missedFrames;
            if (mObstacles[i].missedFrames >= kMaxMissedFrames) {
                mObstacles[i] = {};
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  update  — public entry point
    // ─────────────────────────────────────────────────────────────────────────

    ObstacleFrame ObstacleTracker::update(const FlowResult& flow,
                                          const GridResult& /* grid */) {
        Blob blobs[kMaxBlobs];
        int  blobCount = 0;

        extractBlobs(flow, blobs, blobCount);
        matchAndUpdate(blobs, blobCount, flow.timestampNs);

        ObstacleFrame frame{};
        frame.timestampNs = flow.timestampNs;

        for (int i = 0; i < kMaxObstacles; ++i) {
            frame.obstacles[i] = mObstacles[i];

            // Gate: the obstacle must have been tracked for long enough AND
            // its anomaly confidence must be above the minimum threshold.
            // Obstacles below the gate are kept in the tracker for continuity
            // but muted in audio — this avoids audio stutter when confidence
            // briefly dips for a real obstacle.
            if (mObstacles[i].active &&
                (mObstacles[i].age           < kMinAgeForAudio ||
                 mObstacles[i].confidenceScore < kMinBlobAnomaly)) {
                frame.obstacles[i].active = false;
            }

            if (frame.obstacles[i].active) ++frame.activeCount;
        }

        return frame;
    }

} // namespace assistivenav