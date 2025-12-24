package com.deeperrtk.remote

import android.content.Context
import android.os.Environment
import android.util.Log
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.io.FileOutputStream
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream

object ProjectManager {
    private const val TAG = "ProjectManager"
    private const val PROJECTS_DIR = "DeeperRTK/projects"
    private const val PROJECTS_INDEX = "projects.json"

    private var projects = mutableListOf<Project>()
    private var currentProject: Project? = null
    private lateinit var baseDir: File

    fun init(context: Context) {
        // Use app-specific external storage (no permission needed on Android 10+)
        val externalDir = context.getExternalFilesDir(null)
        if (externalDir != null) {
            baseDir = File(externalDir, "projects")
        } else {
            // Fall back to internal storage
            baseDir = File(context.filesDir, "projects")
        }
        if (!baseDir.exists()) {
            baseDir.mkdirs()
        }
        loadProjects()
    }

    private fun getProjectDir(projectId: String): File {
        return File(baseDir, projectId)
    }

    private fun getIndexFile(): File {
        return File(baseDir, PROJECTS_INDEX)
    }

    fun loadProjects() {
        projects.clear()
        val indexFile = getIndexFile()
        if (indexFile.exists()) {
            try {
                val json = indexFile.readText()
                val array = JSONArray(json)
                for (i in 0 until array.length()) {
                    val projectJson = array.getJSONObject(i)
                    projects.add(Project.fromJson(projectJson))
                }
                // Sort by createdAt descending (newest first)
                projects.sortByDescending { it.createdAt }
                Log.d(TAG, "Loaded ${projects.size} projects")
            } catch (e: Exception) {
                Log.e(TAG, "Error loading projects: ${e.message}")
            }
        }
    }

    private fun saveProjects() {
        try {
            val array = JSONArray()
            for (project in projects) {
                array.put(project.toJson())
            }
            getIndexFile().writeText(array.toString(2))
            Log.d(TAG, "Saved ${projects.size} projects")
        } catch (e: Exception) {
            Log.e(TAG, "Error saving projects: ${e.message}")
        }
    }

    fun getProjects(): List<Project> = projects.toList()

    fun getProject(id: String): Project? = projects.find { it.id == id }

    fun getCurrentProject(): Project? = currentProject

    fun setCurrentProject(project: Project?) {
        currentProject = project
    }

    fun createProject(name: String): Project {
        val project = Project(name = name)
        val projectDir = getProjectDir(project.id)
        projectDir.mkdirs()
        File(projectDir, "rover").mkdirs()
        File(projectDir, "base").mkdirs()
        File(projectDir, "ppk").mkdirs()

        // Save individual project file
        val projectFile = File(projectDir, "project.json")
        projectFile.writeText(project.toJson().toString(2))

        projects.add(0, project)
        saveProjects()
        Log.d(TAG, "Created project: ${project.name} (${project.id})")
        return project
    }

    fun updateProject(project: Project) {
        val index = projects.indexOfFirst { it.id == project.id }
        if (index >= 0) {
            projects[index] = project
            saveProjects()

            // Update individual project file
            val projectDir = getProjectDir(project.id)
            val projectFile = File(projectDir, "project.json")
            projectFile.writeText(project.toJson().toString(2))
        }
    }

    fun deleteProject(projectId: String) {
        val project = projects.find { it.id == projectId } ?: return
        val projectDir = getProjectDir(projectId)
        if (projectDir.exists()) {
            projectDir.deleteRecursively()
        }
        projects.removeAll { it.id == projectId }
        saveProjects()
        if (currentProject?.id == projectId) {
            currentProject = null
        }
        Log.d(TAG, "Deleted project: ${project.name}")
    }

    fun getRoverDir(projectId: String): File {
        return File(getProjectDir(projectId), "rover")
    }

    fun getBaseDir(projectId: String): File {
        return File(getProjectDir(projectId), "base")
    }

    fun getPpkDir(projectId: String): File {
        return File(getProjectDir(projectId), "ppk")
    }

    fun saveRoverFile(projectId: String, filename: String, data: ByteArray): File? {
        return try {
            val file = File(getRoverDir(projectId), filename)
            file.writeBytes(data)
            Log.d(TAG, "Saved rover file: $filename (${data.size} bytes)")
            file
        } catch (e: Exception) {
            Log.e(TAG, "Error saving rover file: ${e.message}")
            null
        }
    }

    fun saveBaseRinexFile(projectId: String, filename: String, data: ByteArray): File? {
        return try {
            val file = File(getBaseDir(projectId), filename)
            file.writeBytes(data)
            Log.d(TAG, "Saved base RINEX file: $filename (${data.size} bytes)")

            // Update project
            val project = getProject(projectId)
            if (project != null) {
                project.baseRinexFile = filename
                if (project.roverUbxFile != null && project.roverCsvFile != null) {
                    project.status = ProjectStatus.FILES_READY
                }
                updateProject(project)
            }
            file
        } catch (e: Exception) {
            Log.e(TAG, "Error saving base file: ${e.message}")
            null
        }
    }

    fun exportProjectZip(projectId: String): File? {
        val project = getProject(projectId) ?: return null
        val projectDir = getProjectDir(projectId)
        val zipFile = File(baseDir, "${project.name.replace(" ", "_")}_export.zip")

        try {
            ZipOutputStream(FileOutputStream(zipFile)).use { zos ->
                // Add rover files
                val roverDir = getRoverDir(projectId)
                project.roverUbxFile?.let { filename ->
                    val file = File(roverDir, filename)
                    if (file.exists()) {
                        zos.putNextEntry(ZipEntry("rover/$filename"))
                        zos.write(file.readBytes())
                        zos.closeEntry()
                    }
                }
                project.roverCsvFile?.let { filename ->
                    val file = File(roverDir, filename)
                    if (file.exists()) {
                        zos.putNextEntry(ZipEntry("rover/$filename"))
                        zos.write(file.readBytes())
                        zos.closeEntry()
                    }
                }

                // Add base RINEX
                project.baseRinexFile?.let { filename ->
                    val file = File(getBaseDir(projectId), filename)
                    if (file.exists()) {
                        zos.putNextEntry(ZipEntry("base/$filename"))
                        zos.write(file.readBytes())
                        zos.closeEntry()
                    }
                }

                // Add project info
                zos.putNextEntry(ZipEntry("project.json"))
                zos.write(project.toJson().toString(2).toByteArray())
                zos.closeEntry()
            }
            Log.d(TAG, "Exported project ZIP: ${zipFile.absolutePath}")
            return zipFile
        } catch (e: Exception) {
            Log.e(TAG, "Error exporting project: ${e.message}")
            return null
        }
    }

    fun importPpkResult(projectId: String, filename: String, data: ByteArray): Boolean {
        return try {
            val file = File(getPpkDir(projectId), filename)
            file.writeBytes(data)

            val project = getProject(projectId)
            if (project != null) {
                project.ppkResultFile = filename
                project.status = ProjectStatus.PPK_COMPLETE
                // TODO: Parse result file to count fixed points
                updateProject(project)
            }
            true
        } catch (e: Exception) {
            Log.e(TAG, "Error importing PPK result: ${e.message}")
            false
        }
    }
}
