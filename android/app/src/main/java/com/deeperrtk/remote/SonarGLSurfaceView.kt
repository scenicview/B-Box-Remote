package com.deeperrtk.remote

import android.content.Context
import android.opengl.GLSurfaceView
import android.util.AttributeSet

class SonarGLSurfaceView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : GLSurfaceView(context, attrs) {

    private val renderer: SonarGLRenderer

    init {
        // Use OpenGL ES 2.0
        setEGLContextClientVersion(2)

        renderer = SonarGLRenderer()
        setRenderer(renderer)

        // Render only when data changes (saves battery)
        renderMode = RENDERMODE_CONTINUOUSLY
    }

    fun setPitchRoll(pitch: Float, roll: Float) {
        renderer.setPitchRoll(pitch, roll)
    }
}
