package com.deeperrtk.remote

import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.cos
import kotlin.math.sin

class SonarGLRenderer : GLSurfaceView.Renderer {

    // Model transformation
    private var pitch = 0f
    private var roll = 0f

    // Matrices
    private val mvpMatrix = FloatArray(16)
    private val projectionMatrix = FloatArray(16)
    private val viewMatrix = FloatArray(16)
    private val modelMatrix = FloatArray(16)
    private val rotationMatrix = FloatArray(16)

    // Sonar model (sphere with ring and details) - 2x scale
    private lateinit var sonarSphere: Sphere
    private lateinit var equatorBand: Ring
    private lateinit var ledIndicator: Disk
    private lateinit var chargingPort: Disk
    private lateinit var ventSlots: List<VentSlot>
    private lateinit var logoText: LogoText

    // Shaders
    private var program = 0

    private val vertexShaderCode = """
        uniform mat4 uMVPMatrix;
        attribute vec4 vPosition;
        attribute vec3 vNormal;
        varying vec3 fNormal;
        varying vec3 fPosition;
        void main() {
            gl_Position = uMVPMatrix * vPosition;
            fNormal = vNormal;
            fPosition = vPosition.xyz;
        }
    """.trimIndent()

    private val fragmentShaderCode = """
        precision mediump float;
        uniform vec4 vColor;
        uniform vec3 uLightPos;
        varying vec3 fNormal;
        varying vec3 fPosition;
        void main() {
            vec3 lightDir = normalize(uLightPos - fPosition);
            float diff = max(dot(normalize(fNormal), lightDir), 0.0);
            float ambient = 0.3;
            float lighting = ambient + diff * 0.7;
            gl_FragColor = vec4(vColor.rgb * lighting, vColor.a);
        }
    """.trimIndent()

    fun setPitchRoll(newPitch: Float, newRoll: Float) {
        pitch = newPitch
        roll = newRoll
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

        // Create sonar model - sphere body (Deeper Chirp+2) - 2x SCALE
        sonarSphere = Sphere(1.0f, 32, 32)

        // Thin black rubber gasket at equator (2x scale)
        equatorBand = Ring(1.00f, 1.04f, 0.05f, 48)

        // LED indicator on upper hemisphere (front-facing) (2x scale)
        ledIndicator = Disk(0.12f, 16, 0.70f, 0f, 0.70f)

        // Charging port at top (2x scale)
        chargingPort = Disk(0.06f, 12, 0f, 1.02f, 0f)

        // Vent slots on upper hemisphere (2x scale)
        ventSlots = listOf(
            // Front vents
            VentSlot(0.76f, 0.30f, 0f, 0.16f, 0.04f),
            VentSlot(0.76f, 0.40f, 0f, 0.16f, 0.04f),
            VentSlot(0.76f, 0.50f, 0f, 0.16f, 0.04f),
            // Back vents
            VentSlot(-0.76f, 0.30f, 0f, 0.16f, 0.04f),
            VentSlot(-0.76f, 0.40f, 0f, 0.16f, 0.04f),
            VentSlot(-0.76f, 0.50f, 0f, 0.16f, 0.04f),
            // Side vents
            VentSlot(0f, 0.30f, 0.76f, 0.16f, 0.04f),
            VentSlot(0f, 0.40f, 0.76f, 0.16f, 0.04f),
            VentSlot(0f, 0.50f, 0.76f, 0.16f, 0.04f)
        )
        
        // "Deeper Chirp+2" logo text on front
        logoText = LogoText()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val ratio = width.toFloat() / height.toFloat()
        Matrix.frustumM(projectionMatrix, 0, -ratio, ratio, -1f, 1f, 2f, 10f)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        // Set camera position
        Matrix.setLookAtM(viewMatrix, 0,
            0f, 0f, 4f,    // Eye position
            0f, 0f, 0f,    // Look at
            0f, 1f, 0f)    // Up vector

        // Create rotation matrix from pitch and roll
        Matrix.setIdentityM(modelMatrix, 0)
        Matrix.setIdentityM(rotationMatrix, 0)

        // Apply roll (rotation around Z axis)
        Matrix.rotateM(rotationMatrix, 0, -roll, 0f, 0f, 1f)
        // Apply pitch (rotation around X axis)
        Matrix.rotateM(rotationMatrix, 0, pitch, 1f, 0f, 0f)

        // Combine matrices
        Matrix.multiplyMM(modelMatrix, 0, rotationMatrix, 0, modelMatrix, 0)

        val tempMatrix = FloatArray(16)
        Matrix.multiplyMM(tempMatrix, 0, viewMatrix, 0, modelMatrix, 0)
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0)

        GLES20.glUseProgram(program)

        // Set light position
        val lightPosHandle = GLES20.glGetUniformLocation(program, "uLightPos")
        GLES20.glUniform3f(lightPosHandle, 2f, 2f, 4f)

        // Deeper Chirp+2 colors
        val tanColor = floatArrayOf(0.55f, 0.47f, 0.38f, 1.0f)      // Military tan/khaki
        val blackColor = floatArrayOf(0.1f, 0.1f, 0.1f, 1.0f)       // Black for rubber band
        val darkTanColor = floatArrayOf(0.40f, 0.34f, 0.28f, 1.0f)  // Darker tan for vents
        val ledColor = floatArrayOf(0.3f, 0.3f, 0.35f, 1.0f)        // LED window (dark)

        // Draw main sphere body (military tan)
        drawMesh(sonarSphere.vertices, sonarSphere.normals, sonarSphere.indices, tanColor)

        // Draw equator rubber band (thin black line)
        drawMesh(equatorBand.vertices, equatorBand.normals, equatorBand.indices, blackColor)

        // Draw LED indicator
        drawMesh(ledIndicator.vertices, ledIndicator.normals, ledIndicator.indices, ledColor)

        // Draw charging port at top
        drawMesh(chargingPort.vertices, chargingPort.normals, chargingPort.indices, blackColor)

        // Draw vent slots
        for (slot in ventSlots) {
            drawMesh(slot.vertices, slot.normals, slot.indices, darkTanColor)
        }
        
        // Draw "Deeper Chirp+2" logo text on front
        val textColor = floatArrayOf(0.25f, 0.22f, 0.18f, 1.0f)  // Dark brown text
        drawMesh(logoText.vertices, logoText.normals, logoText.indices, textColor)
    }

    private fun drawMesh(vertices: FloatBuffer, normals: FloatBuffer, indices: ShortArray, color: FloatArray) {
        val mvpMatrixHandle = GLES20.glGetUniformLocation(program, "uMVPMatrix")
        GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, mvpMatrix, 0)

        val positionHandle = GLES20.glGetAttribLocation(program, "vPosition")
        GLES20.glEnableVertexAttribArray(positionHandle)
        GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false, 0, vertices)

        val normalHandle = GLES20.glGetAttribLocation(program, "vNormal")
        GLES20.glEnableVertexAttribArray(normalHandle)
        GLES20.glVertexAttribPointer(normalHandle, 3, GLES20.GL_FLOAT, false, 0, normals)

        val colorHandle = GLES20.glGetUniformLocation(program, "vColor")
        GLES20.glUniform4fv(colorHandle, 1, color, 0)

        val indexBuffer = ByteBuffer.allocateDirect(indices.size * 2)
            .order(ByteOrder.nativeOrder())
            .asShortBuffer()
            .put(indices)
        indexBuffer.position(0)

        GLES20.glDrawElements(GLES20.GL_TRIANGLES, indices.size, GLES20.GL_UNSIGNED_SHORT, indexBuffer)

        GLES20.glDisableVertexAttribArray(positionHandle)
        GLES20.glDisableVertexAttribArray(normalHandle)
    }

    private fun loadShader(type: Int, shaderCode: String): Int {
        return GLES20.glCreateShader(type).also { shader ->
            GLES20.glShaderSource(shader, shaderCode)
            GLES20.glCompileShader(shader)
        }
    }
}

// Sphere geometry (for the sonar body)
class Sphere(radius: Float, latBands: Int, longBands: Int) {
    val vertices: FloatBuffer
    val normals: FloatBuffer
    val indices: ShortArray

    init {
        val vertexList = mutableListOf<Float>()
        val normalList = mutableListOf<Float>()
        val indexList = mutableListOf<Short>()

        for (lat in 0..latBands) {
            val theta = lat * Math.PI / latBands
            val sinTheta = sin(theta).toFloat()
            val cosTheta = cos(theta).toFloat()

            for (lon in 0..longBands) {
                val phi = lon * 2 * Math.PI / longBands
                val sinPhi = sin(phi).toFloat()
                val cosPhi = cos(phi).toFloat()

                val x = cosPhi * sinTheta
                val y = cosTheta
                val z = sinPhi * sinTheta

                vertexList.add(radius * x)
                vertexList.add(radius * y)
                vertexList.add(radius * z)

                normalList.add(x)
                normalList.add(y)
                normalList.add(z)
            }
        }

        for (lat in 0 until latBands) {
            for (lon in 0 until longBands) {
                val first = (lat * (longBands + 1) + lon).toShort()
                val second = (first + longBands + 1).toShort()

                indexList.add(first)
                indexList.add(second)
                indexList.add((first + 1).toShort())

                indexList.add(second)
                indexList.add((second + 1).toShort())
                indexList.add((first + 1).toShort())
            }
        }

        vertices = createFloatBuffer(vertexList.toFloatArray())
        normals = createFloatBuffer(normalList.toFloatArray())
        indices = indexList.toShortArray()
    }
}

// Ring geometry (for the sonar band)
class Ring(innerRadius: Float, outerRadius: Float, height: Float, segments: Int) {
    val vertices: FloatBuffer
    val normals: FloatBuffer
    val indices: ShortArray

    init {
        val vertexList = mutableListOf<Float>()
        val normalList = mutableListOf<Float>()
        val indexList = mutableListOf<Short>()

        val halfHeight = height / 2

        // Create ring vertices
        for (i in 0..segments) {
            val angle = i * 2 * Math.PI / segments
            val cosA = cos(angle).toFloat()
            val sinA = sin(angle).toFloat()

            // Outer top
            vertexList.addAll(listOf(outerRadius * cosA, halfHeight, outerRadius * sinA))
            normalList.addAll(listOf(cosA, 0f, sinA))

            // Outer bottom
            vertexList.addAll(listOf(outerRadius * cosA, -halfHeight, outerRadius * sinA))
            normalList.addAll(listOf(cosA, 0f, sinA))

            // Inner top
            vertexList.addAll(listOf(innerRadius * cosA, halfHeight, innerRadius * sinA))
            normalList.addAll(listOf(-cosA, 0f, -sinA))

            // Inner bottom
            vertexList.addAll(listOf(innerRadius * cosA, -halfHeight, innerRadius * sinA))
            normalList.addAll(listOf(-cosA, 0f, -sinA))
        }

        // Create indices for outer surface
        for (i in 0 until segments) {
            val base = i * 4
            indexList.addAll(listOf(
                base.toShort(), (base + 1).toShort(), (base + 4).toShort(),
                (base + 1).toShort(), (base + 5).toShort(), (base + 4).toShort()
            ))
        }

        vertices = createFloatBuffer(vertexList.toFloatArray())
        normals = createFloatBuffer(normalList.toFloatArray())
        indices = indexList.toShortArray()
    }
}

private fun createFloatBuffer(arr: FloatArray): FloatBuffer {
    return ByteBuffer.allocateDirect(arr.size * 4)
        .order(ByteOrder.nativeOrder())
        .asFloatBuffer()
        .put(arr)
        .apply { position(0) }
}

// Disk geometry (for LED indicator and charging port)
class Disk(radius: Float, segments: Int, posX: Float, posY: Float, posZ: Float) {
    val vertices: FloatBuffer
    val normals: FloatBuffer
    val indices: ShortArray

    init {
        val vertexList = mutableListOf<Float>()
        val normalList = mutableListOf<Float>()
        val indexList = mutableListOf<Short>()

        // Calculate normal direction (pointing outward from sphere center)
        val len = kotlin.math.sqrt(posX * posX + posY * posY + posZ * posZ)
        val nx = if (len > 0) posX / len else 0f
        val ny = if (len > 0) posY / len else 1f
        val nz = if (len > 0) posZ / len else 0f

        // Center vertex
        vertexList.addAll(listOf(posX, posY, posZ))
        normalList.addAll(listOf(nx, ny, nz))

        // Edge vertices
        for (i in 0..segments) {
            val angle = i * 2 * Math.PI / segments
            val cosA = cos(angle).toFloat()
            val sinA = sin(angle).toFloat()

            // Create perpendicular vectors for the disk plane
            val upX: Float
            val upY: Float
            val upZ: Float
            if (kotlin.math.abs(ny) < 0.9f) {
                upX = 0f; upY = 1f; upZ = 0f
            } else {
                upX = 1f; upY = 0f; upZ = 0f
            }

            // Cross product to get tangent
            val tx = ny * upZ - nz * upY
            val ty = nz * upX - nx * upZ
            val tz = nx * upY - ny * upX
            val tLen = kotlin.math.sqrt(tx * tx + ty * ty + tz * tz)
            val ntx = tx / tLen
            val nty = ty / tLen
            val ntz = tz / tLen

            // Cross product for bitangent
            val bx = ny * ntz - nz * nty
            val by = nz * ntx - nx * ntz
            val bz = nx * nty - ny * ntx

            val vx = posX + radius * (cosA * ntx + sinA * bx)
            val vy = posY + radius * (cosA * nty + sinA * by)
            val vz = posZ + radius * (cosA * ntz + sinA * bz)

            vertexList.addAll(listOf(vx, vy, vz))
            normalList.addAll(listOf(nx, ny, nz))
        }

        // Create triangle fan indices
        for (i in 1..segments) {
            indexList.add(0)
            indexList.add(i.toShort())
            indexList.add((i + 1).toShort())
        }

        vertices = createFloatBuffer(vertexList.toFloatArray())
        normals = createFloatBuffer(normalList.toFloatArray())
        indices = indexList.toShortArray()
    }
}

// Vent slot geometry (small rectangular indent)
class VentSlot(posX: Float, posY: Float, posZ: Float, height: Float, width: Float) {
    val vertices: FloatBuffer
    val normals: FloatBuffer
    val indices: ShortArray

    init {
        val vertexList = mutableListOf<Float>()
        val normalList = mutableListOf<Float>()
        val indexList = mutableListOf<Short>()

        // Calculate normal (pointing outward from center)
        val len = kotlin.math.sqrt(posX * posX + posY * posY + posZ * posZ)
        val nx = if (len > 0) posX / len else 0f
        val ny = 0f
        val nz = if (len > 0) posZ / len else 1f

        // Offset slightly outward
        val offset = 0.01f
        val cx = posX + nx * offset
        val cy = posY
        val cz = posZ + nz * offset

        // Create a small rectangle
        // Perpendicular direction (horizontal)
        val px = -nz
        val pz = nx

        val halfW = width / 2
        val halfH = height / 2

        // Four corners of the slot
        vertexList.addAll(listOf(cx - px * halfW, cy - halfH, cz - pz * halfW))  // bottom-left
        vertexList.addAll(listOf(cx + px * halfW, cy - halfH, cz + pz * halfW))  // bottom-right
        vertexList.addAll(listOf(cx + px * halfW, cy + halfH, cz + pz * halfW))  // top-right
        vertexList.addAll(listOf(cx - px * halfW, cy + halfH, cz - pz * halfW))  // top-left

        // All normals point outward
        for (i in 0..3) {
            normalList.addAll(listOf(nx, ny, nz))
        }

        // Two triangles
        indexList.addAll(listOf(0, 1, 2, 0, 2, 3))

        vertices = createFloatBuffer(vertexList.toFloatArray())
        normals = createFloatBuffer(normalList.toFloatArray())
        indices = indexList.toShortArray()
    }
}

// Logo text "deeper CHIRP+2" - simple block letters on front of sphere
class LogoText {
    val vertices: FloatBuffer
    val normals: FloatBuffer
    val indices: ShortArray

    init {
        val vertexList = mutableListOf<Float>()
        val normalList = mutableListOf<Float>()
        val indexList = mutableListOf<Short>()
        
        val textZ = 1.01f      // Just in front of sphere
        val lineW = 0.025f     // Line thickness
        
        // Helper to add a line segment
        fun line(x1: Float, y1: Float, x2: Float, y2: Float) {
            val idx = (vertexList.size / 3).toShort()
            val dx = x2 - x1
            val dy = y2 - y1
            val len = kotlin.math.sqrt(dx * dx + dy * dy).toFloat()
            if (len < 0.001f) return
            val px = -dy / len * lineW / 2
            val py = dx / len * lineW / 2
            
            vertexList.addAll(listOf(x1 + px, y1 + py, textZ))
            vertexList.addAll(listOf(x1 - px, y1 - py, textZ))
            vertexList.addAll(listOf(x2 - px, y2 - py, textZ))
            vertexList.addAll(listOf(x2 + px, y2 + py, textZ))
            for (i in 0..3) normalList.addAll(listOf(0f, 0f, 1f))
            indexList.addAll(listOf(idx, (idx+1).toShort(), (idx+2).toShort(), idx, (idx+2).toShort(), (idx+3).toShort()))
        }
        
        // Character dimensions
        val w = 0.08f   // char width
        val h = 0.12f   // char height
        val g = 0.03f   // gap between chars
        
        // "deeper" on top line (y = 0.05)
        var x = -0.42f
        val y1 = 0.05f
        
        // d
        line(x, y1, x, y1+h); line(x, y1+h, x+w*0.8f, y1+h); line(x+w*0.8f, y1+h, x+w, y1+h*0.8f)
        line(x+w, y1+h*0.8f, x+w, y1+h*0.2f); line(x+w, y1+h*0.2f, x+w*0.8f, y1); line(x+w*0.8f, y1, x, y1)
        x += w + g
        
        // e
        line(x, y1, x, y1+h); line(x, y1+h, x+w, y1+h); line(x, y1+h/2, x+w*0.8f, y1+h/2); line(x, y1, x+w, y1)
        x += w + g
        
        // e
        line(x, y1, x, y1+h); line(x, y1+h, x+w, y1+h); line(x, y1+h/2, x+w*0.8f, y1+h/2); line(x, y1, x+w, y1)
        x += w + g
        
        // p
        line(x, y1-h*0.4f, x, y1+h); line(x, y1+h, x+w, y1+h); line(x+w, y1+h, x+w, y1+h/2); line(x+w, y1+h/2, x, y1+h/2)
        x += w + g
        
        // e
        line(x, y1, x, y1+h); line(x, y1+h, x+w, y1+h); line(x, y1+h/2, x+w*0.8f, y1+h/2); line(x, y1, x+w, y1)
        x += w + g
        
        // r
        line(x, y1, x, y1+h); line(x, y1+h, x+w, y1+h); line(x+w, y1+h, x+w, y1+h/2); line(x+w, y1+h/2, x, y1+h/2)
        line(x+w*0.4f, y1+h/2, x+w, y1)
        
        // "CHIRP+2" on bottom line (y = -0.15)
        x = -0.42f
        val y2 = -0.15f
        
        // C
        line(x+w, y2+h, x, y2+h); line(x, y2+h, x, y2); line(x, y2, x+w, y2)
        x += w + g
        
        // H
        line(x, y2, x, y2+h); line(x+w, y2, x+w, y2+h); line(x, y2+h/2, x+w, y2+h/2)
        x += w + g
        
        // I
        line(x+w/2, y2, x+w/2, y2+h); line(x, y2+h, x+w, y2+h); line(x, y2, x+w, y2)
        x += w + g
        
        // R
        line(x, y2, x, y2+h); line(x, y2+h, x+w, y2+h); line(x+w, y2+h, x+w, y2+h/2); line(x+w, y2+h/2, x, y2+h/2)
        line(x+w*0.4f, y2+h/2, x+w, y2)
        x += w + g
        
        // P
        line(x, y2, x, y2+h); line(x, y2+h, x+w, y2+h); line(x+w, y2+h, x+w, y2+h/2); line(x+w, y2+h/2, x, y2+h/2)
        x += w + g + 0.02f
        
        // + (smaller)
        val ps = h * 0.35f
        val py = y2 + h/2
        line(x, py, x+ps, py); line(x+ps/2, py-ps/2, x+ps/2, py+ps/2)
        x += ps + g
        
        // 2
        line(x, y2+h, x+w, y2+h); line(x+w, y2+h, x+w, y2+h/2); line(x+w, y2+h/2, x, y2+h/2)
        line(x, y2+h/2, x, y2); line(x, y2, x+w, y2)

        vertices = createFloatBuffer(vertexList.toFloatArray())
        normals = createFloatBuffer(normalList.toFloatArray())
        indices = indexList.toShortArray()
    }
}
