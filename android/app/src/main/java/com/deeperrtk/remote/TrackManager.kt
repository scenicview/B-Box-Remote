package com.deeperrtk.remote

import android.os.Environment
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.*
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.atan2
import kotlin.math.abs

/**
 * Thread-safe singleton to manage track/depth point collection
 * with IMU-corrected bottom position calculation
 */
object TrackManager {
    private val points = mutableListOf<DepthPoint>()
    private var originLat: Double? = null
    private var originLon: Double? = null
    private val lock = Any()

    // Heading tracking
    private var lastHeading: Double = 0.0  // radians, 0 = North, PI/2 = East
    private var lastLat: Double = 0.0
    private var lastLon: Double = 0.0
    private const val MIN_MOVEMENT_M = 0.1  // 10cm minimum movement to update heading

    // Max points to keep in memory (for performance)
    private const val MAX_POINTS = 10000

    // Listeners for point updates
    private val listeners = mutableListOf<(DepthPoint) -> Unit>()

    fun addPoint(point: DepthPoint) {
        synchronized(lock) {
            // Set origin on first valid point
            if (originLat == null && point.latitude != 0.0) {
                originLat = point.latitude
                originLon = point.longitude
                lastLat = point.latitude
                lastLon = point.longitude
            }

            // Only add if we have valid GPS
            if (point.latitude != 0.0 && point.depth > 0) {
                // Update heading if we have moved enough
                updateHeading(point.latitude, point.longitude)

                // Calculate IMU-corrected bottom position
                calculateBottomPosition(point)

                points.add(point)

                // Trim if over max
                if (points.size > MAX_POINTS) {
                    points.removeAt(0)
                }
            }
        }

        // Notify listeners outside of lock to avoid deadlock
        if (point.latitude != 0.0 && point.depth > 0) {
            listeners.forEach { it(point) }
        }
    }

    /**
     * Update heading based on GPS trajectory (direction of travel)
     */
    private fun updateHeading(lat: Double, lon: Double) {
        // Calculate distance moved
        val dLat = lat - lastLat
        val dLon = lon - lastLon
        val distLat = dLat * 110540.0  // meters
        val distLon = dLon * 111320.0 * cos(Math.toRadians(lat))  // meters
        val distance = sqrt(distLat * distLat + distLon * distLon)

        // Only update heading if we have moved enough (avoids noise when stationary)
        if (distance > MIN_MOVEMENT_M) {
            // Calculate bearing from last position to current
            val lat1 = Math.toRadians(lastLat)
            val lat2 = Math.toRadians(lat)
            val dLonRad = Math.toRadians(lon - lastLon)

            val x = sin(dLonRad) * cos(lat2)
            val y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLonRad)

            lastHeading = atan2(x, y)  // radians, 0 = North, PI/2 = East
            lastLat = lat
            lastLon = lon
        }
    }

    /**
     * Calculate the true bottom position accounting for IMU pitch/roll
     *
     * The sonar beam points down from the transducer, but pitch/roll tilt it.
     * We rotate the beam vector and project it to find where it actually hits.
     */
    private fun calculateBottomPosition(point: DepthPoint) {
        val pitch = Math.toRadians(point.pitch.toDouble())
        val roll = Math.toRadians(point.roll.toDouble())
        val heading = lastHeading

        // Initial beam vector pointing straight down in body frame
        var beamX = 0.0
        var beamY = 0.0
        var beamZ = -1.0

        // Apply roll rotation (around Y/forward axis)
        val cosRoll = cos(roll)
        val sinRoll = sin(roll)
        val beamXRoll = beamX * cosRoll + beamZ * sinRoll
        val beamYRoll = beamY
        val beamZRoll = -beamX * sinRoll + beamZ * cosRoll

        // Apply pitch rotation (around X/right axis)
        val cosPitch = cos(pitch)
        val sinPitch = sin(pitch)
        val beamXPitch = beamXRoll
        val beamYPitch = beamYRoll * cosPitch - beamZRoll * sinPitch
        val beamZPitch = beamYRoll * sinPitch + beamZRoll * cosPitch

        // Transform body frame to ENU (East-North-Up) using heading
        val cosH = cos(heading)
        val sinH = sin(heading)
        val beamEast = beamXPitch * cosH + beamYPitch * sinH
        val beamNorth = -beamXPitch * sinH + beamYPitch * cosH
        val beamUp = beamZPitch

        // Scale by depth to get offset in meters
        // beamUp is negative (pointing down), depth is positive
        val scale = if (abs(beamUp) > 0.01) point.depth / abs(beamUp) else point.depth.toDouble()
        val offsetEast = beamEast * scale
        val offsetNorth = beamNorth * scale

        // Convert meter offsets to lat/lon
        point.bottomLat = point.latitude + (offsetNorth / 110540.0)
        point.bottomLon = point.longitude + (offsetEast / (111320.0 * cos(Math.toRadians(point.latitude))))
    }

    fun getPoints(): List<DepthPoint> = synchronized(lock) { points.toList() }

    fun getPointCount(): Int = synchronized(lock) { points.size }

    fun getLastPoint(): DepthPoint? = synchronized(lock) { points.lastOrNull() }

    fun getCurrentHeading(): Double = synchronized(lock) { Math.toDegrees(lastHeading) }

    fun clear() {
        synchronized(lock) {
            points.clear()
            originLat = null
            originLon = null
            lastHeading = 0.0
            lastLat = 0.0
            lastLon = 0.0
        }
    }

    fun addListener(listener: (DepthPoint) -> Unit) {
        listeners.add(listener)
    }

    fun removeListener(listener: (DepthPoint) -> Unit) {
        listeners.remove(listener)
    }

    /**
     * Convert lat/lon to local XY coordinates (meters from origin)
     */
    fun toLocalXY(lat: Double, lon: Double): Pair<Float, Float> {
        val oLat: Double
        val oLon: Double
        synchronized(lock) {
            oLat = originLat ?: lat
            oLon = originLon ?: lon
        }

        // Approximate conversion (valid for small areas)
        val x = ((lon - oLon) * 111320.0 * cos(Math.toRadians(oLat))).toFloat()
        val y = ((lat - oLat) * 110540.0).toFloat()
        return Pair(x, y)
    }

    /**
     * Export track to CSV file
     */
    fun exportToCSV(): File? {
        val pointsCopy = synchronized(lock) { points.toList() }
        if (pointsCopy.isEmpty()) return null

        try {
            val dateFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US)
            val filename = "track_" + dateFormat.format(Date()) + ".csv"
            val downloadsDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS)
            val file = File(downloadsDir, filename)

            FileWriter(file).use { writer ->
                // Header - includes both transducer and bottom positions
                writer.write("timestamp,trans_lat,trans_lon,bottom_lat,bottom_lon,depth,temperature,pitch,roll,fix,satellites\n")

                // Data
                for (p in pointsCopy) {
                    writer.write(String.format(Locale.US,
                        "%d,%.8f,%.8f,%.8f,%.8f,%.2f,%.1f,%.1f,%.1f,%d,%d\n",
                        p.timestamp, p.latitude, p.longitude,
                        p.bottomLat, p.bottomLon,
                        p.depth, p.temperature, p.pitch, p.roll, p.fix, p.satellites))
                }
            }

            return file
        } catch (e: Exception) {
            e.printStackTrace()
            return null
        }
    }

    /**
     * Get depth range for color scaling
     */
    fun getDepthRange(): Pair<Float, Float> {
        val pointsCopy = synchronized(lock) { points.toList() }
        if (pointsCopy.isEmpty()) return Pair(0f, 10f)
        val depths = pointsCopy.map { it.depth }
        return Pair(depths.minOrNull() ?: 0f, depths.maxOrNull() ?: 10f)
    }

    /**
     * Get temperature range for heat map color scaling
     */
    fun getTemperatureRange(): Pair<Float, Float> {
        val pointsCopy = synchronized(lock) { points.toList() }
        if (pointsCopy.isEmpty()) return Pair(10f, 25f)  // Default 10-25C
        val temps = pointsCopy.filter { it.temperature > 0 }.map { it.temperature }
        if (temps.isEmpty()) return Pair(10f, 25f)
        return Pair(temps.minOrNull() ?: 10f, temps.maxOrNull() ?: 25f)
    }
}
