package com.deeperrtk.remote

import org.json.JSONObject
import java.util.UUID

enum class ProjectStatus {
    CREATED,        // Project created, no data yet
    RECORDING,      // Currently recording data
    FILES_READY,    // Has rover files, ready for base import or PPK
    PPK_COMPLETE    // PPK processing done, results available
}

data class Project(
    val id: String = UUID.randomUUID().toString(),
    var name: String,
    val createdAt: Long = System.currentTimeMillis(),
    var status: ProjectStatus = ProjectStatus.CREATED,
    var recordingStartTime: Long = 0,    // UTC timestamp when recording started
    var recordingEndTime: Long = 0,      // UTC timestamp when recording stopped
    var roverUbxFile: String? = null,    // raw_XXXX.ubx from B-BOX
    var roverCsvFile: String? = null,    // log_XXXX.csv from B-BOX
    var baseRtcmFile: String? = null,    // rtcm_XXXX.bin from Shore Station
    var baseRinexFile: String? = null,   // Converted RINEX .obs
    var ppkResultFile: String? = null,   // Final merged CSV after PPK
    var totalPoints: Int = 0,
    var fixedPoints: Int = 0,
    var floatPoints: Int = 0,
    var singlePoints: Int = 0
) {
    fun toJson(): JSONObject {
        return JSONObject().apply {
            put("id", id)
            put("name", name)
            put("createdAt", createdAt)
            put("status", status.name)
            put("recordingStartTime", recordingStartTime)
            put("recordingEndTime", recordingEndTime)
            put("roverUbxFile", roverUbxFile ?: "")
            put("roverCsvFile", roverCsvFile ?: "")
            put("baseRtcmFile", baseRtcmFile ?: "")
            put("baseRinexFile", baseRinexFile ?: "")
            put("ppkResultFile", ppkResultFile ?: "")
            put("totalPoints", totalPoints)
            put("fixedPoints", fixedPoints)
            put("floatPoints", floatPoints)
            put("singlePoints", singlePoints)
        }
    }

    companion object {
        fun fromJson(json: JSONObject): Project {
            return Project(
                id = json.getString("id"),
                name = json.getString("name"),
                createdAt = json.getLong("createdAt"),
                status = ProjectStatus.valueOf(json.optString("status", "CREATED")),
                recordingStartTime = json.optLong("recordingStartTime", 0),
                recordingEndTime = json.optLong("recordingEndTime", 0),
                roverUbxFile = json.optString("roverUbxFile").ifEmpty { null },
                roverCsvFile = json.optString("roverCsvFile").ifEmpty { null },
                baseRtcmFile = json.optString("baseRtcmFile").ifEmpty { null },
                baseRinexFile = json.optString("baseRinexFile").ifEmpty { null },
                ppkResultFile = json.optString("ppkResultFile").ifEmpty { null },
                totalPoints = json.optInt("totalPoints", 0),
                fixedPoints = json.optInt("fixedPoints", 0),
                floatPoints = json.optInt("floatPoints", 0),
                singlePoints = json.optInt("singlePoints", 0)
            )
        }
    }

    fun getFixPercentage(): Int {
        return if (totalPoints > 0) (fixedPoints * 100 / totalPoints) else 0
    }

    fun getFloatPercentage(): Int {
        return if (totalPoints > 0) (floatPoints * 100 / totalPoints) else 0
    }

    fun getSinglePercentage(): Int {
        return if (totalPoints > 0) (singlePoints * 100 / totalPoints) else 0
    }

    fun getStatusText(): String {
        return when (status) {
            ProjectStatus.CREATED -> "Ready to Record"
            ProjectStatus.RECORDING -> "Recording..."
            ProjectStatus.FILES_READY -> "Ready for PPK"
            ProjectStatus.PPK_COMPLETE -> "PPK Complete (${getFixPercentage()}% Fixed)"
        }
    }

    fun getRecordingDurationText(): String {
        if (recordingStartTime == 0L) return "--"
        val endTime = if (recordingEndTime > 0) recordingEndTime else System.currentTimeMillis()
        val durationSec = (endTime - recordingStartTime) / 1000
        val minutes = durationSec / 60
        val seconds = durationSec % 60
        return String.format("%d:%02d", minutes, seconds)
    }
}
