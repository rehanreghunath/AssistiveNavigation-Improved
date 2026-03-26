package com.rehanreghunath.assistivenav

object FlowBridge {
    init {
        System.loadLibrary("assistivenav")
    }

    /** Create the C++ FlowEngine, GridAnalyzer, and ImuFusion.
     *  Call once when the camera resolution is known. */
    external fun nativeInit(width: Int, height: Int)

    /**
     * Deliver a rotation-vector sensor event to the IMU fusion layer.
     *
     * Call this from your SensorEventListener on every TYPE_ROTATION_VECTOR
     * event.  Pass [event.values] directly — the array is [x, y, z, w] (unit
     * quaternion) and may optionally carry a 5th element (heading accuracy)
     * which is ignored by the native side.
     *
     * @param quaternion FloatArray of length 4 or 5 from SensorEvent.values
     * @param timestampNs event.timestamp from the SensorEvent
     */
    external fun nativeUpdateImu(quaternion: FloatArray, timestampNs: Long)

    /**
     * Process one camera frame.
     *
     * @param yPlane     Raw bytes of the Y-plane from YUV_420_888
     * @param width      Frame width in pixels
     * @param height     Frame height in pixels
     * @param rowStride  Bytes per row (may be > width due to hardware padding)
     * @param timestampNs Frame timestamp from CameraX in nanoseconds
     * @return Float[3] = [trackedCount, totalPts, globalMeanMag]. Null on error.
     */
    external fun nativeProcessFrame(
        yPlane: ByteArray,
        width: Int,
        height: Int,
        rowStride: Int,
        timestampNs: Long
    ): FloatArray?

    /** Returns the RGBA overlay for the last processed frame, or null if not ready. */
    external fun nativeGetRgbaFrame(): ByteArray?

    /**
     * Returns per-cell grid analysis for the last processed frame (IMU-compensated).
     *
     * Layout — 48 floats flat:
     *   Indices 0..44 — 9 cells × 5 floats, row-major [0]=top-left [8]=bottom-right
     *     [base+0]  meanMag      (px/frame)
     *     [base+1]  meanAngle    (radians, −π..π)
     *     [base+2]  dangerScore  (0..1)
     *     [base+3]  ttc          (frames; 0 = no motion)
     *     [base+4]  sampleCount  (cast to float)
     *   Index 45 — foeX      (normalised 0..1)
     *   Index 46 — foeY      (normalised 0..1)
     *   Index 47 — foeValid  (1.0 = valid, 0.0 = not available)
     *
     * Primary danger zone: cells [4] (centre-middle) and [7] (centre-bottom).
     * Returns null on the first frame or before nativeInit.
     */
    external fun nativeGetGridResult(): FloatArray?

    /** Set the clockwise rotation (0/90/180/270) for the RGBA overlay. */
    external fun nativeSetRenderRotation(degrees: Int)

    /** Release all native resources.  Call from Activity.onDestroy(). */
    external fun nativeDestroy()
}