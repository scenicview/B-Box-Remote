package com.deeperrtk.remote

import android.content.Context
import android.util.Log
import java.io.*
import kotlin.math.*

/**
 * PPK Post-Processing for DeeperRTK
 *
 * Pipeline:
 * 1. Convert rover UBX to RINEX using native RTKLIB
 * 2. Run PPK processing using native RTKLIB
 * 3. Parse solution file
 * 4. Merge with rover CSV data
 * 5. Apply IMU correction for true bottom positions
 */
class PPKProcessor(private val context: Context, private val logCallback: (String) -> Unit) {

    companion object {
        private const val TAG = "PPKProcessor"
        private const val METERS_PER_DEG_LAT = 111320.0
    }

    init {
        if (RtklibNative.isLoaded()) {
            log("RTKLIB native library loaded, version: ${RtklibNative.getVersion()}")
        } else {
            log("Warning: RTKLIB native library not loaded")
        }
    }

    private fun log(msg: String) {
        Log.d(TAG, msg)
        logCallback(msg)
    }

    /**
     * Check if RTKLIB native library is available
     */
    fun hasRtklibBinaries(): Boolean {
        return RtklibNative.isLoaded()
    }

    /**
     * Convert UBX file to RINEX using native convbin
     * Returns pair of (obs_file, nav_file) or null on error
     */
    fun convertUbxToRinex(ubxFile: File, outputDir: File): Pair<File, File>? {
        if (!RtklibNative.isLoaded()) {
            log("Error: RTKLIB native library not loaded")
            return null
        }

        val baseName = ubxFile.nameWithoutExtension
        val obsFile = File(outputDir, "$baseName.obs")
        val navFile = File(outputDir, "$baseName.nav")

        log("Converting UBX to RINEX...")
        log("Input: ${ubxFile.absolutePath}")
        log("Output: ${obsFile.name}, ${navFile.name}")

        return try {
            val result = RtklibNative.convbin(
                ubxFile.absolutePath,
                obsFile.absolutePath,
                navFile.absolutePath
            )

            if (result > 0 && obsFile.exists() && navFile.exists()) {
                log("Conversion successful: ${obsFile.name} (${obsFile.length()/1024}KB), ${navFile.name} (${navFile.length()/1024}KB)")
                Pair(obsFile, navFile)
            } else {
                log("Error: convbin returned $result")
                null
            }
        } catch (e: Exception) {
            log("Error in convbin: ${e.javaClass.simpleName}: ${e.message}")
            e.printStackTrace()
            null
        }
    }

    /**
     * Run PPK processing using native rnx2rtkp
     * Returns solution .pos file or null on error
     */
    fun runPpk(roverObs: File, baseObs: File, navFile: File, outputDir: File, elevMask: Int = 15): File? {
        if (!RtklibNative.isLoaded()) {
            log("Error: RTKLIB native library not loaded")
            return null
        }

        val posFile = File(outputDir, "${roverObs.nameWithoutExtension}.pos")

        log("Running PPK processing...")
        log("Rover: ${roverObs.name}")
        log("Base: ${baseObs.name}")
        log("Nav: ${navFile.name}")
        log("Elevation mask: $elevMask deg")

        return try {
            val result = RtklibNative.rnx2rtkp(
                roverObs.absolutePath,
                baseObs.absolutePath,
                navFile.absolutePath,
                posFile.absolutePath,
                elevMask
            )

            if (result > 0 && posFile.exists() && posFile.length() > 0) {
                log("PPK solution created: ${posFile.name} (${posFile.length()/1024}KB), $result solutions")
                posFile
            } else {
                log("Error: rnx2rtkp returned $result")
                null
            }
        } catch (e: Exception) {
            log("Error in rnx2rtkp: ${e.javaClass.simpleName}: ${e.message}")
            e.printStackTrace()
            null
        }
    }

    /**
     * Full PPK processing pipeline
     */
    fun processFullPpk(ubxFile: File, baseRinexFile: File, csvFile: File, outputDir: File): List<MergedRecord> {
        log("Starting full PPK processing...")

        // Step 1: Convert UBX to RINEX
        val (roverObs, navFile) = convertUbxToRinex(ubxFile, outputDir) ?: run {
            log("Failed to convert UBX to RINEX")
            return emptyList()
        }

        // Step 2: Run PPK
        val posFile = runPpk(roverObs, baseRinexFile, navFile, outputDir) ?: run {
            log("PPK processing failed")
            return emptyList()
        }

        // Step 3: Parse solution
        val solutions = parsePosFile(posFile)
        if (solutions.isEmpty()) {
            log("No valid solutions in .pos file")
            return emptyList()
        }

        // Step 4: Parse rover CSV
        val roverRecords = parseRoverCsv(csvFile)
        if (roverRecords.isEmpty()) {
            log("No valid records in rover CSV")
            return emptyList()
        }

        // Step 5: Merge with IMU correction
        return merge(solutions, roverRecords)
    }

    /**
     * Parse rover CSV file from B-Box
     * Format: UTC_Time,Lat,Lon,Alt,Fix,HDOP,Sats,RTCM_Age,Depth,WaterTemp,Pitch,Roll,RSSI
     */
    fun parseRoverCsv(csvFile: File): List<RoverRecord> {
        val records = mutableListOf<RoverRecord>()

        try {
            csvFile.bufferedReader().useLines { lines ->
                lines.drop(1).forEach { line ->
                    try {
                        val parts = line.split(",")
                        if (parts.size >= 12) {
                            val timeStr = parts[0].trim()
                            val timeParts = timeStr.split(":")
                            if (timeParts.size >= 3) {
                                val hours = timeParts[0].toIntOrNull() ?: 0
                                val minutes = timeParts[1].toIntOrNull() ?: 0
                                val secParts = timeParts[2].split(".")
                                val seconds = secParts[0].toIntOrNull() ?: 0
                                val millis = if (secParts.size > 1) secParts[1].padEnd(3, '0').take(3).toIntOrNull() ?: 0 else 0

                                val secondsOfDay = hours * 3600.0 + minutes * 60.0 + seconds + millis / 1000.0

                                records.add(RoverRecord(
                                    utcTime = timeStr,
                                    secondsOfDay = secondsOfDay,
                                    latitude = parts[1].toDoubleOrNull() ?: 0.0,
                                    longitude = parts[2].toDoubleOrNull() ?: 0.0,
                                    altitude = parts[3].toDoubleOrNull() ?: 0.0,
                                    fixQuality = parts[4].toIntOrNull() ?: 0,
                                    depth = parts[8].toDoubleOrNull() ?: -1.0,
                                    waterTemp = parts[9].toDoubleOrNull() ?: 0.0,
                                    pitch = parts[10].toDoubleOrNull() ?: 0.0,
                                    roll = parts[11].toDoubleOrNull() ?: 0.0
                                ))
                            }
                        }
                    } catch (e: Exception) {
                        // Skip malformed lines
                    }
                }
            }
            log("Parsed ${records.size} records from rover CSV")
        } catch (e: Exception) {
            log("Error parsing CSV: ${e.message}")
        }

        return records
    }

    /**
     * Parse RTKLIB .pos solution file
     */
    fun parsePosFile(posFile: File): List<PPKSolution> {
        val solutions = mutableListOf<PPKSolution>()

        try {
            posFile.bufferedReader().useLines { lines ->
                lines.forEach { line ->
                    if (line.startsWith("%") || line.isBlank()) return@forEach

                    val parts = line.trim().split("\\s+".toRegex())
                    if (parts.size >= 10) {
                        try {
                            val timeStr = parts[1]  // HH:MM:SS.SSS
                            val timeParts = timeStr.split(":")
                            if (timeParts.size >= 3) {
                                val hours = timeParts[0].toInt()
                                val minutes = timeParts[1].toInt()
                                val secParts = timeParts[2].split(".")
                                val seconds = secParts[0].toInt()
                                val frac = if (secParts.size > 1) "0.${secParts[1]}".toDouble() else 0.0

                                val secondsOfDay = hours * 3600.0 + minutes * 60.0 + seconds + frac

                                val sdn = parts[7].toDouble()
                                val sde = parts[8].toDouble()
                                val sdu = parts[9].toDouble()

                                solutions.add(PPKSolution(
                                    secondsOfDay = secondsOfDay,
                                    latitude = parts[2].toDouble(),
                                    longitude = parts[3].toDouble(),
                                    altitude = parts[4].toDouble(),
                                    fixQuality = parts[5].toInt(),
                                    numSats = parts[6].toInt(),
                                    hAccuracy = sqrt(sdn * sdn + sde * sde),
                                    vAccuracy = sdu
                                ))
                            }
                        } catch (e: Exception) {
                            // Skip malformed lines
                        }
                    }
                }
            }
            log("Parsed ${solutions.size} PPK solutions")
        } catch (e: Exception) {
            log("Error parsing POS file: ${e.message}")
        }

        return solutions
    }

    /**
     * Merge PPK solutions with rover data and calculate true bottom positions
     */
    fun merge(ppkSolutions: List<PPKSolution>, roverRecords: List<RoverRecord>): List<MergedRecord> {
        if (ppkSolutions.isEmpty() || roverRecords.isEmpty()) {
            log("Error: No data to merge")
            return emptyList()
        }

        val merged = mutableListOf<MergedRecord>()

        val sortedPpk = ppkSolutions.sortedBy { it.secondsOfDay }
        val ppkTimes = sortedPpk.map { it.secondsOfDay }.toDoubleArray()
        val ppkLats = sortedPpk.map { it.latitude }.toDoubleArray()
        val ppkLons = sortedPpk.map { it.longitude }.toDoubleArray()
        val ppkAlts = sortedPpk.map { it.altitude }.toDoubleArray()

        var lastHeading = 0.0
        var lastLat = 0.0
        var lastLon = 0.0
        var fixedCount = 0
        var floatCount = 0
        var singleCount = 0

        for (record in roverRecords) {
            val t = record.secondsOfDay

            if (t < ppkTimes.first() || t > ppkTimes.last()) continue

            val idx = ppkTimes.indexOfFirst { it >= t }
            if (idx <= 0) continue

            val t0 = ppkTimes[idx - 1]
            val t1 = ppkTimes[idx]
            val frac = if (t1 > t0) (t - t0) / (t1 - t0) else 0.0

            val lat = ppkLats[idx - 1] + frac * (ppkLats[idx] - ppkLats[idx - 1])
            val lon = ppkLons[idx - 1] + frac * (ppkLons[idx] - ppkLons[idx - 1])
            val alt = ppkAlts[idx - 1] + frac * (ppkAlts[idx] - ppkAlts[idx - 1])

            val nearestPpk = if (frac < 0.5) sortedPpk[idx - 1] else sortedPpk[idx]

            if (lastLat != 0.0 && lastLon != 0.0) {
                val dist = haversineDistance(lastLat, lastLon, lat, lon)
                if (dist > 0.1) {
                    lastHeading = calculateHeading(lastLat, lastLon, lat, lon)
                }
            }
            lastLat = lat
            lastLon = lon

            val (bottomLat, bottomLon, bottomElev) = calculateBottomPoint(
                lat, lon, alt,
                record.depth, record.pitch, record.roll, lastHeading
            )

            when (nearestPpk.fixQuality) {
                1 -> fixedCount++
                2 -> floatCount++
                else -> singleCount++
            }

            merged.add(MergedRecord(
                utcTime = record.utcTime,
                transLat = lat,
                transLon = lon,
                transAlt = alt,
                bottomLat = bottomLat,
                bottomLon = bottomLon,
                bottomElev = bottomElev,
                depth = record.depth,
                waterTemp = record.waterTemp,
                pitch = record.pitch,
                roll = record.roll,
                heading = Math.toDegrees(lastHeading),
                fixQuality = nearestPpk.fixQuality,
                hAccuracy = nearestPpk.hAccuracy,
                vAccuracy = nearestPpk.vAccuracy
            ))
        }

        val total = fixedCount + floatCount + singleCount
        if (total > 0) {
            log("Merged ${merged.size} points:")
            log("  Fixed: $fixedCount (${fixedCount * 100 / total}%)")
            log("  Float: $floatCount (${floatCount * 100 / total}%)")
            log("  Single: $singleCount (${singleCount * 100 / total}%)")
        }

        return merged
    }

    private fun calculateHeading(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val lat1Rad = Math.toRadians(lat1)
        val lat2Rad = Math.toRadians(lat2)
        val dlon = Math.toRadians(lon2 - lon1)

        val x = sin(dlon) * cos(lat2Rad)
        val y = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dlon)

        return atan2(x, y)
    }

    private fun calculateBottomPoint(
        transLat: Double, transLon: Double, transAlt: Double,
        depth: Double, pitchDeg: Double, rollDeg: Double, headingRad: Double
    ): Triple<Double, Double, Double> {

        if (depth <= 0) {
            return Triple(transLat, transLon, transAlt)
        }

        val pitch = Math.toRadians(pitchDeg)
        val roll = Math.toRadians(rollDeg)

        var beamX = 0.0
        var beamY = 0.0
        var beamZ = -1.0

        val cosRoll = cos(roll)
        val sinRoll = sin(roll)
        val beamXRoll = beamX * cosRoll + beamZ * sinRoll
        val beamYRoll = beamY
        val beamZRoll = -beamX * sinRoll + beamZ * cosRoll

        val cosPitch = cos(pitch)
        val sinPitch = sin(pitch)
        val beamXPitch = beamXRoll
        val beamYPitch = beamYRoll * cosPitch - beamZRoll * sinPitch
        val beamZPitch = beamYRoll * sinPitch + beamZRoll * cosPitch

        val cosH = cos(headingRad)
        val sinH = sin(headingRad)
        val beamEast = beamXPitch * cosH + beamYPitch * sinH
        val beamNorth = -beamXPitch * sinH + beamYPitch * cosH
        val beamUp = beamZPitch

        val offsetEast = depth * beamEast
        val offsetNorth = depth * beamNorth
        val offsetUp = depth * beamUp

        val metersPerDegLon = METERS_PER_DEG_LAT * cos(Math.toRadians(transLat))

        val bottomLat = transLat + (offsetNorth / METERS_PER_DEG_LAT)
        val bottomLon = transLon + (offsetEast / metersPerDegLon)
        val bottomElev = transAlt + offsetUp

        return Triple(bottomLat, bottomLon, bottomElev)
    }

    private fun haversineDistance(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2).pow(2) + cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon / 2).pow(2)
        return 6371000 * 2 * atan2(sqrt(a), sqrt(1 - a))
    }

    fun exportToCsv(records: List<MergedRecord>, outputFile: File): Boolean {
        return try {
            outputFile.printWriter().use { out ->
                out.println("utc_time,trans_lat,trans_lon,trans_alt,bottom_lat,bottom_lon,bottom_elev,depth,water_temp,pitch,roll,heading,fix_quality,h_accuracy,v_accuracy")
                for (r in records) {
                    out.println("${r.utcTime},${r.transLat},${r.transLon},${r.transAlt},${r.bottomLat},${r.bottomLon},${r.bottomElev},${r.depth},${r.waterTemp},${r.pitch},${r.roll},${r.heading},${r.fixQuality},${r.hAccuracy},${r.vAccuracy}")
                }
            }
            log("Exported ${records.size} records to ${outputFile.name}")
            true
        } catch (e: Exception) {
            log("Export error: ${e.message}")
            false
        }
    }
}

data class RoverRecord(
    val utcTime: String,
    val secondsOfDay: Double,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val fixQuality: Int,
    val depth: Double,
    val waterTemp: Double,
    val pitch: Double,
    val roll: Double
)

data class PPKSolution(
    val secondsOfDay: Double,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val fixQuality: Int,
    val numSats: Int,
    val hAccuracy: Double,
    val vAccuracy: Double
)

data class MergedRecord(
    val utcTime: String,
    val transLat: Double,
    val transLon: Double,
    val transAlt: Double,
    val bottomLat: Double,
    val bottomLon: Double,
    val bottomElev: Double,
    val depth: Double,
    val waterTemp: Double,
    val pitch: Double,
    val roll: Double,
    val heading: Double,
    val fixQuality: Int,
    val hAccuracy: Double,
    val vAccuracy: Double
)
