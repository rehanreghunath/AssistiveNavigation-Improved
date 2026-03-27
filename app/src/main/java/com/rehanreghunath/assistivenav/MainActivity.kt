package com.rehanreghunath.assistivenav

import android.Manifest
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.util.Log
import android.util.Size
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.graphics.createBitmap
import com.rehanreghunath.assistivenav.databinding.ActivityMainBinding
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var cameraExecutor: ExecutorService

    // ── IMU ────────────────────────────────────────────────────────────────────
    // SensorManager is a system service; we hold a reference so we can
    // register and unregister cleanly with the activity lifecycle.
    private lateinit var sensorManager: SensorManager

    // TYPE_ROTATION_VECTOR fuses accelerometer + gyroscope + magnetometer into
    // a quaternion representing the device's absolute orientation.  It is more
    // stable than raw gyroscope integration and requires no manual drift correction.
    // Android guarantees this sensor exists on any device with an accelerometer
    // and gyroscope; we check at runtime and degrade gracefully if absent.
    private var rotationSensor: Sensor? = null

    // Delivers rotation-vector events to the native IMU fusion layer.
    // Declared as a property so it can be unregistered by reference.
    private val rotationListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            // event.values for TYPE_ROTATION_VECTOR is always [x, y, z, w] (unit
            // quaternion).  A 5th element (heading accuracy in radians) may be
            // present on some devices; the native side ignores it.
            // event.timestamp is nanoseconds of elapsed real time — the same
            // time base as System.nanoTime(), which the camera pipeline also uses.
            FlowBridge.nativeUpdateImu(event.values, event.timestamp)
        }

        override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
            // Accuracy changes (UNRELIABLE / LOW / MEDIUM / HIGH) are logged but
            // we do not stop compensation on degraded accuracy.  Low accuracy means
            // the heading estimate is off, but the short-term delta between two
            // consecutive readings (which is all we use) remains reliable as long
            // as the gyroscope hardware is working.
            Log.d(TAG, "Rotation sensor accuracy changed to $accuracy")
        }
    }

    @Volatile private var isRunning         = false
    private var pipelineInitialized         = false
    private var cameraProvider: ProcessCameraProvider? = null

    companion object {
        private const val TAG              = "MainActivity"
        private const val REQUEST_CAMERA   = 100
        private const val HUD_UPDATE_EVERY = 10

        // SENSOR_DELAY_GAME (~50 Hz) is the best balance between IMU data
        // freshness and battery consumption for a 30 fps optical-flow pipeline.
        // SENSOR_DELAY_FASTEST (~200 Hz) would deliver more data than the camera
        // can consume and wastes CPU waking the sensor thread unnecessarily.
        private const val IMU_DELAY = SensorManager.SENSOR_DELAY_GAME
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        cameraExecutor = Executors.newSingleThreadExecutor()

        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        if (rotationSensor == null) {
            // The device has no rotation-vector sensor.  The pipeline still works
            // correctly — ImuFusion::compensate returns immediately when no sensor
            // data has arrived, so flow vectors are used as-is without compensation.
            Log.w(TAG, "TYPE_ROTATION_VECTOR unavailable; IMU compensation disabled")
        }

        binding.start.setOnClickListener { onToggle() }
    }

    override fun onResume() {
        super.onResume()
        // Re-register the sensor whenever the activity becomes visible so that
        // orientation data is fresh when the camera restarts.
        rotationSensor?.let {
            sensorManager.registerListener(rotationListener, it, IMU_DELAY)
        }
    }

    override fun onPause() {
        super.onPause()
        // Unregister immediately when the activity is not visible.
        // Leaving a sensor listener registered in the background drains the
        // battery on every device, regardless of whether we're processing frames.
        sensorManager.unregisterListener(rotationListener)
    }

    override fun onDestroy() {
        super.onDestroy()
        stopPipeline()
        cameraExecutor.shutdown()
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Pipeline control
    // ─────────────────────────────────────────────────────────────────────────

    private fun onToggle() {
        if (isRunning) stopPipeline()
        else if (hasCameraPermission()) startPipeline()
        else requestCameraPermission()
    }

    private fun startPipeline() {
        isRunning = true
        binding.start.setText(R.string.stop)
        binding.debugText.text = "Starting..."
        startCamera()
        val audioManager = getSystemService(AUDIO_SERVICE) as android.media.AudioManager
        audioManager.requestAudioFocus(
            null,
            android.media.AudioManager.STREAM_MUSIC,
            android.media.AudioManager.AUDIOFOCUS_GAIN
        )
    }

    private fun stopPipeline() {
        isRunning = false
        cameraProvider?.unbindAll()

        if (pipelineInitialized) {
            FlowBridge.nativeDestroy()
            pipelineInitialized = false
        }

        runOnUiThread {
            binding.start.setText(R.string.start)
            binding.debugText.setText(R.string.waiting)
            binding.flowView.visibility = android.view.View.INVISIBLE
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Camera
    // ─────────────────────────────────────────────────────────────────────────

    private fun startCamera() {
        val future = ProcessCameraProvider.getInstance(this)
        future.addListener({
            cameraProvider = future.get()

            val resSel = ResolutionSelector.Builder()
                .setResolutionStrategy(
                    ResolutionStrategy(
                        Size(640, 480),
                        ResolutionStrategy.FALLBACK_RULE_CLOSEST_LOWER_THEN_HIGHER
                    )
                ).build()

            val preview = Preview.Builder()
                .setResolutionSelector(resSel)
                .build()
                .apply { setSurfaceProvider(binding.previewView.surfaceProvider) }

            val analysis = ImageAnalysis.Builder()
                .setResolutionSelector(resSel)
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                .build()
                .apply { setAnalyzer(cameraExecutor, FrameAnalyzer()) }

            try {
                cameraProvider?.unbindAll()
                cameraProvider?.bindToLifecycle(
                    this, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis
                )
            } catch (e: Exception) {
                Log.e(TAG, "Camera bind failed", e)
                stopPipeline()
                Toast.makeText(this, "Camera failed to start", Toast.LENGTH_SHORT).show()
            }
        }, ContextCompat.getMainExecutor(this))
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Frame analysis
    // ─────────────────────────────────────────────────────────────────────────

    private inner class FrameAnalyzer : ImageAnalysis.Analyzer {
        private var yBuffer: ByteArray? = null
        private var bitmap: android.graphics.Bitmap? = null
        private var hudFrameCount = 0

        private var lastFrameTimeNs = 0L
        private var smoothedFps     = 0f
        private val fpsAlpha        = 0.1f

        override fun analyze(image: ImageProxy) {
            val nowNs = System.nanoTime()

            image.use {
                if (!isRunning) return

                if (lastFrameTimeNs != 0L) {
                    val deltaSeconds = (nowNs - lastFrameTimeNs) / 1_000_000_000f
                    if (deltaSeconds < 1f) {
                        smoothedFps = fpsAlpha * (1f / deltaSeconds) +
                                (1f - fpsAlpha) * smoothedFps
                    }
                }
                lastFrameTimeNs = nowNs

                val w = image.width
                val h = image.height

                if (!pipelineInitialized) {
                    FlowBridge.nativeInit(w, h)
                    val rotation = image.imageInfo.rotationDegrees
                    FlowBridge.nativeSetRenderRotation(rotation)
                    val (bmpW, bmpH) = if (rotation == 90 || rotation == 270) h to w else w to h
                    bitmap = createBitmap(bmpW, bmpH)
                    pipelineInitialized = true
                    Log.d(TAG, "Pipeline init ${w}x${h}, bitmap=${bmpW}x${bmpH}")
                }

                val yPlane    = image.planes[0]
                val rowStride = yPlane.rowStride
                val byteCount = rowStride * h

                if (yBuffer == null || yBuffer!!.size < byteCount) {
                    yBuffer = ByteArray(byteCount)
                }
                yPlane.buffer.rewind()
                yPlane.buffer.get(yBuffer!!, 0, byteCount)

                val stats = FlowBridge.nativeProcessFrame(
                    yBuffer!!, w, h, rowStride, image.imageInfo.timestamp
                )

                val rgba = FlowBridge.nativeGetRgbaFrame() ?: return
                val bmp  = bitmap ?: return

                bmp.copyPixelsFromBuffer(java.nio.ByteBuffer.wrap(rgba))

                if (++hudFrameCount >= HUD_UPDATE_EVERY) {
                    hudFrameCount = 0
                    val tracked    = stats?.get(0)?.toInt() ?: 0
                    val total      = stats?.get(1)?.toInt() ?: 0
                    val fpsDisplay = smoothedFps.toInt()

                    // Fetch grid result to verify the analysis pipeline is running
                    // and to display the two primary danger cell scores on the HUD.
                    // cells[4] = centre-middle, cells[7] = centre-bottom.
                    val grid       = FlowBridge.nativeGetGridResult()
                    val danger4    = grid?.get(4 * 5 + 2)  // dangerScore of cell 4
                    val danger7    = grid?.get(7 * 5 + 2)  // dangerScore of cell 7
                    val d4str      = if (danger4 != null) "%.2f".format(danger4) else "--"
                    val d7str      = if (danger7 != null) "%.2f".format(danger7) else "--"

                    runOnUiThread {
                        if (isRunning) {
                            binding.flowView.setImageBitmap(bmp)
                            binding.flowView.visibility = android.view.View.VISIBLE
                            binding.debugText.text =
                                "FPS: $fpsDisplay  |  Tracked: $tracked / $total\n" +
                                        "Danger  mid: $d4str  fwd: $d7str"
                        }
                    }
                } else {
                    runOnUiThread {
                        if (isRunning) {
                            binding.flowView.setImageBitmap(bmp)
                            binding.flowView.visibility = android.view.View.VISIBLE
                        }
                    }
                }
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Permissions
    // ─────────────────────────────────────────────────────────────────────────

    private fun hasCameraPermission() =
        ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
                PackageManager.PERMISSION_GRANTED

    private fun requestCameraPermission() =
        ActivityCompat.requestPermissions(
            this, arrayOf(Manifest.permission.CAMERA), REQUEST_CAMERA
        )

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_CAMERA) {
            if (grantResults.firstOrNull() == PackageManager.PERMISSION_GRANTED)
                startPipeline()
            else {
                isRunning = false
                binding.start.setText(R.string.start)
                binding.debugText.setText(R.string.waiting)
                Toast.makeText(this, "Camera permission required", Toast.LENGTH_LONG).show()
            }
        }
    }
}