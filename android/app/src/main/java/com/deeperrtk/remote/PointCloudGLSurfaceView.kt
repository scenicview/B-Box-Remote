package com.deeperrtk.remote

import android.content.Context
import android.opengl.GLSurfaceView
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.ScaleGestureDetector

class PointCloudGLSurfaceView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : GLSurfaceView(context, attrs) {

    private val renderer: PointCloudRenderer
    private var previousX = 0f
    private var previousY = 0f
    private val scaleDetector: ScaleGestureDetector

    init {
        setEGLContextClientVersion(2)
        renderer = PointCloudRenderer()
        setRenderer(renderer)
        renderMode = RENDERMODE_CONTINUOUSLY

        scaleDetector = ScaleGestureDetector(context, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScale(detector: ScaleGestureDetector): Boolean {
                renderer.scale(detector.scaleFactor)
                return true
            }
        })
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleDetector.onTouchEvent(event)

        when (event.action) {
            MotionEvent.ACTION_DOWN -> {
                previousX = event.x
                previousY = event.y
            }
            MotionEvent.ACTION_MOVE -> {
                if (!scaleDetector.isInProgress) {
                    val dx = event.x - previousX
                    val dy = event.y - previousY
                    renderer.rotate(dx * 0.5f, dy * 0.5f)
                    previousX = event.x
                    previousY = event.y
                }
            }
        }
        return true
    }

    fun updatePoints() {
        renderer.updatePoints()
    }

    fun resetView() {
        renderer.resetView()
    }

    fun clearPoints() {
        // Points are cleared via TrackManager.clear(), renderer will pick up the empty list
        renderer.updatePoints()
    }

    fun addPoint(point: DepthPoint) {
        // Points are added via TrackManager, renderer reads from it automatically
        // This is just for API compatibility
    }
}
