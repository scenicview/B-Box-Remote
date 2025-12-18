package com.deeperrtk.remote

/**
 * Data class representing a single depth measurement point
 */
data class DepthPoint(
    val timestamp: Long,      // System time in milliseconds
    val latitude: Double,     // GPS latitude (transducer position)
    val longitude: Double,    // GPS longitude (transducer position)
    val depth: Float,         // Depth in meters
    val temperature: Float = 0f, // Water temperature in Celsius
    val pitch: Float,         // IMU pitch in degrees
    val roll: Float,          // IMU roll in degrees
    val fix: Int = 0,         // GPS fix quality (0=none, 4=RTK fixed, 5=RTK float)
    val satellites: Int = 0,  // Number of satellites
    // IMU-corrected bottom position (calculated from pitch/roll/heading)
    var bottomLat: Double = 0.0,
    var bottomLon: Double = 0.0
)
