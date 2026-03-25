package com.rehanreghunath.assistivenav

import android.Manifest
import android.content.pm.PackageManager
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
import com.rehanreghunath.assistivenav.databinding.ActivityMainBinding
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import androidx.core.graphics.createBitmap

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var cameraExecutor: ExecutorService

    @Volatile private var isRunning         = false
    private var pipelineInitialized         = false
    private var cameraProvider: ProcessCameraProvider? = null

    companion object {
        private const val TAG              = "MainActivity"
        private const val REQUEST_CAMERA   = 100
        private const val HUD_UPDATE_EVERY = 10
    }


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        cameraExecutor = Executors.newSingleThreadExecutor()

        binding.start.setOnClickListener { onToggle() }
    }

    override fun onDestroy() {
        super.onDestroy()
        stopPipeline()
        cameraExecutor.shutdown()
    }


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


    private inner class FrameAnalyzer : ImageAnalysis.Analyzer {
        private var yBuffer: ByteArray? = null
        private var bitmap: android.graphics.Bitmap? = null
        private var hudFrameCount = 0
        private var lastFrameTimeNs = 0L

        private var smoothedFps = 0f
        private val fpsAlpha = 0.1f

        override fun analyze(image: ImageProxy) {

            val nowNs = System.nanoTime()

            image.use {
                if (!isRunning) return


                if (lastFrameTimeNs != 0L) {
                    val deltaSeconds = (nowNs - lastFrameTimeNs) / 1_000_000_000f

                    if (deltaSeconds < 1f) {
                        val instantFps = 1f / deltaSeconds

                        smoothedFps = fpsAlpha * instantFps + (1f - fpsAlpha) * smoothedFps
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
                    val tracked = stats?.get(0)?.toInt() ?: 0
                    val total   = stats?.get(1)?.toInt() ?: 0
                    val fpsDisplay = smoothedFps.toInt()
                    runOnUiThread {
                        if (isRunning) {
                            binding.flowView.setImageBitmap(bmp)
                            binding.flowView.visibility = android.view.View.VISIBLE
                            binding.debugText.text = "FPS: $fpsDisplay  |  Tracked: $tracked / $total"
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