package com.deeperrtk.remote

import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

class PointCloudRenderer : GLSurfaceView.Renderer {

    // Camera/view parameters
    private var rotationX = 30f  // Pitch
    private var rotationY = 0f   // Yaw
    private var zoom = 50f       // Distance from center

    // Matrices
    private val mvpMatrix = FloatArray(16)
    private val projectionMatrix = FloatArray(16)
    private val viewMatrix = FloatArray(16)
    private val modelMatrix = FloatArray(16)

    // Point data
    private var pointBuffer: FloatBuffer? = null
    private var colorBuffer: FloatBuffer? = null
    private var pointCount = 0

    // Grid data
    private var gridBuffer: FloatBuffer? = null
    private var gridColorBuffer: FloatBuffer? = null
    private var gridVertexCount = 0

    // Shaders
    private var program = 0

    private val vertexShaderCode = """
        uniform mat4 uMVPMatrix;
        attribute vec4 vPosition;
        attribute vec4 vColor;
        varying vec4 fColor;
        uniform float uPointSize;
        void main() {
            gl_Position = uMVPMatrix * vPosition;
            gl_PointSize = uPointSize;
            fColor = vColor;
        }
    """.trimIndent()

    private val fragmentShaderCode = """
        precision mediump float;
        varying vec4 fColor;
        void main() {
            gl_FragColor = fColor;
        }
    """.trimIndent()

    fun rotate(dx: Float, dy: Float) {
        rotationY += dx
        rotationX += dy
        rotationX = rotationX.coerceIn(-89f, 89f)
    }

    fun scale(factor: Float) {
        zoom /= factor
        zoom = zoom.coerceIn(5f, 200f)
    }

    fun resetView() {
        rotationX = 30f
        rotationY = 0f
        zoom = 50f
    }

    fun updatePoints() {
        val points = TrackManager.getPoints()
        if (points.isEmpty()) {
            pointCount = 0
            return
        }

        val (minDepth, maxDepth) = TrackManager.getDepthRange()
        val depthRange = (maxDepth - minDepth).coerceAtLeast(1f)

        // Build vertex and color arrays
        val vertices = FloatArray(points.size * 3)
        val colors = FloatArray(points.size * 4)

        for ((i, p) in points.withIndex()) {
            // Use IMU-corrected bottom position if available
            val lat = if (p.bottomLat != 0.0) p.bottomLat else p.latitude
            val lon = if (p.bottomLon != 0.0) p.bottomLon else p.longitude
            val (x, y) = TrackManager.toLocalXY(lat, lon)
            vertices[i * 3] = x
            vertices[i * 3 + 1] = y
            vertices[i * 3 + 2] = -p.depth  // Negative = down

            // Color by depth (blue=shallow -> red=deep)
            val t = ((p.depth - minDepth) / depthRange).coerceIn(0f, 1f)
            val color = depthToColor(t)
            colors[i * 4] = color[0]
            colors[i * 4 + 1] = color[1]
            colors[i * 4 + 2] = color[2]
            colors[i * 4 + 3] = 1f
        }

        pointBuffer = createFloatBuffer(vertices)
        colorBuffer = createFloatBuffer(colors)
        pointCount = points.size
    }

    private fun depthToColor(t: Float): FloatArray {
        // Gradient: blue -> cyan -> green -> yellow -> red
        return when {
            t < 0.25f -> {
                val s = t / 0.25f
                floatArrayOf(0f, s, 1f)  // Blue to cyan
            }
            t < 0.5f -> {
                val s = (t - 0.25f) / 0.25f
                floatArrayOf(0f, 1f, 1f - s)  // Cyan to green
            }
            t < 0.75f -> {
                val s = (t - 0.5f) / 0.25f
                floatArrayOf(s, 1f, 0f)  // Green to yellow
            }
            else -> {
                val s = (t - 0.75f) / 0.25f
                floatArrayOf(1f, 1f - s, 0f)  // Yellow to red
            }
        }
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.1f, 0.1f, 0.15f, 1.0f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)

        // Compile shaders
        val vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, vertexShaderCode)
        val fragmentShader = loadShader(GLES20.GL_FRAGMENT_SHADER, fragmentShaderCode)

        program = GLES20.glCreateProgram().also {
            GLES20.glAttachShader(it, vertexShader)
            GLES20.glAttachShader(it, fragmentShader)
            GLES20.glLinkProgram(it)
        }

        createGrid()
    }

    private fun createGrid() {
        val gridSize = 100f  // 100 meters
        val gridStep = 10f   // 10 meter grid lines
        val lines = mutableListOf<Float>()
        val colors = mutableListOf<Float>()

        // Horizontal grid lines (XY plane at Z=0)
        var pos = -gridSize
        while (pos <= gridSize) {
            // X lines
            lines.addAll(listOf(pos, -gridSize, 0f, pos, gridSize, 0f))
            // Y lines
            lines.addAll(listOf(-gridSize, pos, 0f, gridSize, pos, 0f))

            // Gray color
            repeat(4) { colors.addAll(listOf(0.3f, 0.3f, 0.3f, 0.5f)) }
            pos += gridStep
        }

        // Vertical lines (depth markers)
        for (depth in listOf(-5f, -10f, -20f, -50f)) {
            lines.addAll(listOf(-gridSize, 0f, depth, gridSize, 0f, depth))
            colors.addAll(listOf(0.2f, 0.2f, 0.4f, 0.3f))
            colors.addAll(listOf(0.2f, 0.2f, 0.4f, 0.3f))
        }

        // Axis lines (X=red, Y=green, Z=blue)
        // X axis
        lines.addAll(listOf(0f, 0f, 0f, 20f, 0f, 0f))
        colors.addAll(listOf(1f, 0f, 0f, 1f, 1f, 0f, 0f, 1f))
        // Y axis
        lines.addAll(listOf(0f, 0f, 0f, 0f, 20f, 0f))
        colors.addAll(listOf(0f, 1f, 0f, 1f, 0f, 1f, 0f, 1f))
        // Z axis (down)
        lines.addAll(listOf(0f, 0f, 0f, 0f, 0f, -20f))
        colors.addAll(listOf(0f, 0f, 1f, 1f, 0f, 0f, 1f, 1f))

        gridBuffer = createFloatBuffer(lines.toFloatArray())
        gridColorBuffer = createFloatBuffer(colors.toFloatArray())
        gridVertexCount = lines.size / 3
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val ratio = width.toFloat() / height.toFloat()
        Matrix.perspectiveM(projectionMatrix, 0, 45f, ratio, 0.1f, 500f)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        // Update points from TrackManager
        updatePoints()

        // Calculate camera position
        val radX = Math.toRadians(rotationX.toDouble())
        val radY = Math.toRadians(rotationY.toDouble())
        val eyeX = (zoom * kotlin.math.cos(radX) * kotlin.math.sin(radY)).toFloat()
        val eyeY = (zoom * kotlin.math.cos(radX) * kotlin.math.cos(radY)).toFloat()
        val eyeZ = (zoom * kotlin.math.sin(radX)).toFloat()

        Matrix.setLookAtM(viewMatrix, 0,
            eyeX, eyeY, eyeZ,  // Eye
            0f, 0f, -10f,      // Look at (center, slightly below)
            0f, 0f, 1f)        // Up (Z up in our coordinate system)

        Matrix.setIdentityM(modelMatrix, 0)

        val tempMatrix = FloatArray(16)
        Matrix.multiplyMM(tempMatrix, 0, viewMatrix, 0, modelMatrix, 0)
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0)

        GLES20.glUseProgram(program)

        // Draw grid
        gridBuffer?.let { gb ->
            gridColorBuffer?.let { cb ->
                drawLines(gb, cb, gridVertexCount)
            }
        }

        // Draw points
        if (pointCount > 0) {
            pointBuffer?.let { pb ->
                colorBuffer?.let { cb ->
                    drawPoints(pb, cb, pointCount)
                }
            }
        }
    }

    private fun drawPoints(vertices: FloatBuffer, colors: FloatBuffer, count: Int) {
        val mvpHandle = GLES20.glGetUniformLocation(program, "uMVPMatrix")
        GLES20.glUniformMatrix4fv(mvpHandle, 1, false, mvpMatrix, 0)

        val pointSizeHandle = GLES20.glGetUniformLocation(program, "uPointSize")
        GLES20.glUniform1f(pointSizeHandle, 8f)

        val posHandle = GLES20.glGetAttribLocation(program, "vPosition")
        GLES20.glEnableVertexAttribArray(posHandle)
        GLES20.glVertexAttribPointer(posHandle, 3, GLES20.GL_FLOAT, false, 0, vertices)

        val colorHandle = GLES20.glGetAttribLocation(program, "vColor")
        GLES20.glEnableVertexAttribArray(colorHandle)
        GLES20.glVertexAttribPointer(colorHandle, 4, GLES20.GL_FLOAT, false, 0, colors)

        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, count)

        GLES20.glDisableVertexAttribArray(posHandle)
        GLES20.glDisableVertexAttribArray(colorHandle)
    }

    private fun drawLines(vertices: FloatBuffer, colors: FloatBuffer, count: Int) {
        val mvpHandle = GLES20.glGetUniformLocation(program, "uMVPMatrix")
        GLES20.glUniformMatrix4fv(mvpHandle, 1, false, mvpMatrix, 0)

        val pointSizeHandle = GLES20.glGetUniformLocation(program, "uPointSize")
        GLES20.glUniform1f(pointSizeHandle, 1f)

        val posHandle = GLES20.glGetAttribLocation(program, "vPosition")
        GLES20.glEnableVertexAttribArray(posHandle)
        GLES20.glVertexAttribPointer(posHandle, 3, GLES20.GL_FLOAT, false, 0, vertices)

        val colorHandle = GLES20.glGetAttribLocation(program, "vColor")
        GLES20.glEnableVertexAttribArray(colorHandle)
        GLES20.glVertexAttribPointer(colorHandle, 4, GLES20.GL_FLOAT, false, 0, colors)

        GLES20.glDrawArrays(GLES20.GL_LINES, 0, count)

        GLES20.glDisableVertexAttribArray(posHandle)
        GLES20.glDisableVertexAttribArray(colorHandle)
    }

    private fun loadShader(type: Int, shaderCode: String): Int {
        return GLES20.glCreateShader(type).also { shader ->
            GLES20.glShaderSource(shader, shaderCode)
            GLES20.glCompileShader(shader)
        }
    }

    private fun createFloatBuffer(arr: FloatArray): FloatBuffer {
        return ByteBuffer.allocateDirect(arr.size * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()
            .put(arr)
            .apply { position(0) }
    }
}
