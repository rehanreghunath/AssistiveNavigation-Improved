package com.rehanreghunath.assistivenav

object FlowBridge {
    init {
        System.loadLibrary("assistivenav")
    }

    /** Create the C++ FlowEngine. Call once when camera resolution is known. */
    external fun nativeInit(width: Int, height: Int)

    /**
     * Process one camera frame.
     *
     * @param yPlane     Raw bytes of the Y-plane from YUV_420_888
     * @param width      Frame width in pixels
     * @param height     Frame height in pixels
     * @param rowStride  Bytes per row (may be > width due to hardware padding)
     * @param timestampNs Frame timestamp from CameraX in nanoseconds
     * @return Packed FloatArray — see jni_bridge.cpp for layout. Null on error.
     */
    external fun nativeProcessFrame(
        yPlane: ByteArray,
        width: Int,
        height: Int,
        rowStride: Int,
        timestampNs: Long
    ): FloatArray?

    /* Release all native resources. Call from Activity.onDestroy(). */
    external fun nativeDestroy()

    external fun nativeSurfaceCreated()
    external fun nativeSurfaceChanged(width: Int, height: Int)

    /** Returns [fps, trackedCount] or null on error. */
    external fun nativeDrawFrame(): FloatArray?

    external fun nativeGetRgbaFrame(): ByteArray?
    external fun nativeSetRenderRotation(degrees: Int)
}