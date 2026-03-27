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
        // Zero-init all obstacle slots so missedFrames/age start clean.
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
    //  Step 1: bin vectors into the 96-cell activity grid.
    //  Step 2: 4-connected flood-fill to group adjacent active cells into blobs.
    //
    //  All state lives on the stack — two 96-element arrays and a small DFS
    //  stack.  No heap allocation.
    // ─────────────────────────────────────────────────────────────────────────

    void ObstacleTracker::extractBlobs(const FlowResult& flow,
                                       Blob out[kMaxBlobs],
                                       int& outCount) const {
        outCount = 0;

        // ── Step 1: accumulate per-cell stats ─────────────────────────────────
        float sumMag[kGridCells] = {};
        int   counts [kGridCells] = {};

        for (const FlowVector& fv : flow.vectors) {
            const int idx = cellIndex(fv.x0, fv.y0);
            sumMag[idx] += fv.magnitude;
            ++counts[idx];
        }

        // A cell is active if it has enough vectors above the noise floor.
        bool active[kGridCells] = {};
        for (int i = 0; i < kGridCells; ++i) {
            if (counts[i] >= kMinVectors &&
                sumMag[i] / counts[i] >= kMinActionableMag) {
                active[i] = true;
            }
        }

        // ── Step 2: 4-connected flood-fill ────────────────────────────────────
        bool visited[kGridCells] = {};

        // Fixed-size DFS stack: at most kGridCells cells can be pushed.
        int  dfsStack[kGridCells];

        const float cellW = static_cast<float>(mWidth)  / kGridCols;
        const float cellH = static_cast<float>(mHeight) / kGridRows;

        for (int seed = 0; seed < kGridCells; ++seed) {
            if (!active[seed] || visited[seed]) continue;
            if (outCount >= kMaxBlobs) break;

            // BFS/DFS from this seed cell.
            float wSumX  = 0.0f; // magnitude-weighted sum of cell centre X
            float wSumY  = 0.0f;
            float wTotal = 0.0f; // total weight (= total magnitude)
            int   cells  = 0;

            int top = 0;
            dfsStack[top++] = seed;
            visited[seed]   = true;

            while (top > 0) {
                const int cur = dfsStack[--top];
                const int row = cur / kGridCols;
                const int col = cur % kGridCols;

                const float cellCX = (col + 0.5f) * cellW;
                const float cellCY = (row + 0.5f) * cellH;
                const float mag    = sumMag[cur];

                wSumX  += cellCX * mag;
                wSumY  += cellCY * mag;
                wTotal += mag;
                ++cells;

                // 4-connected neighbours.
                const int neighbours[4] = {
                        (row > 0)              ? cur - kGridCols : -1,
                        (row < kGridRows - 1)  ? cur + kGridCols : -1,
                        (col > 0)              ? cur - 1         : -1,
                        (col < kGridCols - 1)  ? cur + 1         : -1
                };

                for (const int nb : neighbours) {
                    if (nb >= 0 && active[nb] && !visited[nb]) {
                        visited[nb]    = true;
                        dfsStack[top++] = nb;
                    }
                }
            }

            if (wTotal < 1e-6f) continue;

            Blob& b   = out[outCount++];
            b.normX   = (wSumX / wTotal) / static_cast<float>(mWidth);
            b.normY   = (wSumY / wTotal) / static_cast<float>(mHeight);
            b.meanMag = wTotal / static_cast<float>(cells);
            b.cellCount = cells;
            b.matched   = false;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  matchAndUpdate
    //
    //  Greedy nearest-centroid matching.  Each blob is assigned to the closest
    //  unmatched existing obstacle within kMatchRadius.  Unmatched blobs become
    //  new obstacles if a free slot exists.  Unmatched obstacles accumulate
    //  missedFrames and are cleared after kMaxMissedFrames.
    // ─────────────────────────────────────────────────────────────────────────

    void ObstacleTracker::matchAndUpdate(const Blob blobs[kMaxBlobs],
                                         int blobCount,
                                         int64_t timestampNs) {
        // Track which obstacle slots were matched this frame.
        bool obstacleMatched[kMaxObstacles] = {};

        // ── Match each blob to the nearest obstacle ───────────────────────────
        for (int b = 0; b < blobCount; ++b) {
            const Blob& blob = blobs[b];

            float bestDist = kMatchRadius;
            int   bestIdx  = -1;

            for (int i = 0; i < kMaxObstacles; ++i) {
                if (!mObstacles[i].active) continue;

                const float dx = blob.normX - mObstacles[i].normX;
                const float dy = blob.normY - mObstacles[i].normY;
                const float d  = std::sqrt(dx * dx + dy * dy);

                if (d < bestDist) {
                    bestDist = d;
                    bestIdx  = i;
                }
            }

            if (bestIdx >= 0) {
                // Update existing obstacle with EMA smoothing.
                TrackedObstacle& o = mObstacles[bestIdx];
                o.normX       = kAlphaPos * blob.normX + (1.0f - kAlphaPos) * o.normX;
                o.normY       = kAlphaPos * blob.normY + (1.0f - kAlphaPos) * o.normY;
                o.smoothedMag = kAlphaMag * blob.meanMag + (1.0f - kAlphaMag) * o.smoothedMag;
                o.missedFrames = 0;
                ++o.age;
                obstacleMatched[bestIdx] = true;
            } else {
                // New blob — find a free slot.
                for (int i = 0; i < kMaxObstacles; ++i) {
                    if (!mObstacles[i].active) {
                        TrackedObstacle& o = mObstacles[i];
                        o.normX        = blob.normX;
                        o.normY        = blob.normY;
                        o.smoothedMag  = blob.meanMag;
                        o.age          = 1;
                        o.missedFrames = 0;
                        o.active       = true;
                        obstacleMatched[i] = true;
                        break;
                    }
                }
            }
        }

        // ── Age out unmatched obstacles ───────────────────────────────────────
        for (int i = 0; i < kMaxObstacles; ++i) {
            if (!mObstacles[i].active || obstacleMatched[i]) continue;

            ++mObstacles[i].missedFrames;
            if (mObstacles[i].missedFrames >= kMaxMissedFrames) {
                mObstacles[i] = {};  // clear slot
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  update  — public entry point
    // ─────────────────────────────────────────────────────────────────────────

    ObstacleFrame ObstacleTracker::update(const FlowResult& flow) {
        Blob blobs[kMaxBlobs];
        int  blobCount = 0;

        extractBlobs(flow, blobs, blobCount);
        matchAndUpdate(blobs, blobCount, flow.timestampNs);

        // Build the output frame from the current obstacle state.
        ObstacleFrame frame{};
        frame.timestampNs = flow.timestampNs;

        for (int i = 0; i < kMaxObstacles; ++i) {
            frame.obstacles[i] = mObstacles[i];

            // Suppress obstacles that haven't yet met the minimum age threshold —
            // they exist in the tracker for continuity but should not yet
            // trigger audio.
            if (mObstacles[i].active &&
                mObstacles[i].age < kMinAgeForAudio) {
                frame.obstacles[i].active = false;
            }

            if (frame.obstacles[i].active) ++frame.activeCount;
        }

        return frame;  // NRVO — no copy under C++17
    }

} // namespace assistivenav