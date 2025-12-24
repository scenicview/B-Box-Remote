package com.deeperrtk.remote

import android.Manifest
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.widget.*
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.io.File
import java.text.SimpleDateFormat
import java.util.*

class ProjectDetailActivity : AppCompatActivity() {

    companion object {
        private const val DEVICE_NAME = "BBOX"
        private val SERVICE_UUID: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        private val CHARACTERISTIC_UUID_RX: UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
        private val CHARACTERISTIC_UUID_TX: UUID = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
        private val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        private const val REQUEST_BLUETOOTH_PERMISSIONS = 100
        private const val SCAN_TIMEOUT_MS = 10000L
    }

    private lateinit var project: Project
    private var projectId: String = ""

    // BLE
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothLeScanner: BluetoothLeScanner? = null
    private var bluetoothGatt: BluetoothGatt? = null
    private var rxCharacteristic: BluetoothGattCharacteristic? = null
    private var txCharacteristic: BluetoothGattCharacteristic? = null
    private var isConnected = false
    private var isScanning = false
    private val handler = Handler(Looper.getMainLooper())
    private val lineBuffer = StringBuilder()

    // File download state
    private var isDownloading = false
    private var downloadFileName = ""
    private var downloadBuffer = ByteArray(0)
    private var downloadFileType = "" // "ubx" or "csv"
    private var availableFiles = mutableListOf<String>()
    private var filesToDownload = mutableListOf<String>()
    private var currentDownloadIndex = 0

    // UI Elements
    private lateinit var tvProjectName: TextView
    private lateinit var tvStatus: TextView
    private lateinit var tvDuration: TextView
    private lateinit var tvCreated: TextView

    // Rover section
    private lateinit var indicatorUbx: View
    private lateinit var indicatorCsv: View
    private lateinit var tvUbxFile: TextView
    private lateinit var tvCsvFile: TextView
    private lateinit var tvUbxStatus: TextView
    private lateinit var tvCsvStatus: TextView
    private lateinit var btnDownloadRover: Button
    private lateinit var layoutRoverProgress: LinearLayout
    private lateinit var progressRover: ProgressBar
    private lateinit var tvRoverProgress: TextView

    // Base section
    private lateinit var indicatorRtcm: View
    private lateinit var tvRtcmFile: TextView
    private lateinit var tvRtcmOverlap: TextView
    private lateinit var btnFindBaseFile: Button
    private lateinit var btnImportRinex: Button
    private lateinit var layoutBaseProgress: LinearLayout
    private lateinit var progressBase: ProgressBar
    private lateinit var tvBaseProgress: TextView

    // PPK section
    private lateinit var tvPpkStatus: TextView
    private lateinit var btnProcessPpk: Button
    private lateinit var layoutPpkProgress: LinearLayout
    private lateinit var progressPpk: ProgressBar
    private lateinit var tvPpkProgress: TextView
    private lateinit var layoutPpkResults: LinearLayout
    private lateinit var tvFixedPercent: TextView
    private lateinit var tvFloatPercent: TextView
    private lateinit var tvSinglePercent: TextView
    private lateinit var tvTotalPoints: TextView

    // Export section
    private lateinit var btnExportCsv: Button
    private lateinit var btnExportZip: Button

    private lateinit var btnDeleteProject: Button
    private lateinit var btnBack: Button

    // BLE Scan callback
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            try {
                val deviceName = device.name ?: result.scanRecord?.deviceName
                val serviceUuids = result.scanRecord?.serviceUuids
                val hasOurService = serviceUuids?.any { it.uuid == SERVICE_UUID } == true

                val isBBox = hasOurService || deviceName?.contains(DEVICE_NAME, ignoreCase = true) == true

                if (isBBox) {
                    stopScan()
                    connectToDevice(device)
                }
            } catch (e: SecurityException) { }
        }

        override fun onScanFailed(errorCode: Int) {
            handler.post {
                Toast.makeText(this@ProjectDetailActivity, "BLE scan failed: $errorCode", Toast.LENGTH_SHORT).show()
                btnDownloadRover.isEnabled = true
                tvRoverProgress.text = "Scan failed"
            }
        }
    }

    // BLE GATT callback
    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    handler.post { tvRoverProgress.text = "Requesting MTU..." }
                    try { gatt.requestMtu(512) } catch (e: SecurityException) { }
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    handler.post { onDisconnected() }
                }
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            handler.post { tvRoverProgress.text = "Discovering services..." }
            try { gatt.discoverServices() } catch (e: SecurityException) { }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt.getService(SERVICE_UUID)
                if (service != null) {
                    rxCharacteristic = service.getCharacteristic(CHARACTERISTIC_UUID_RX)
                    txCharacteristic = service.getCharacteristic(CHARACTERISTIC_UUID_TX)
                    if (txCharacteristic != null) {
                        try {
                            gatt.setCharacteristicNotification(txCharacteristic, true)
                            val descriptor = txCharacteristic!!.getDescriptor(CCCD_UUID)
                            if (descriptor != null) {
                                descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                                gatt.writeDescriptor(descriptor)
                            }
                        } catch (e: SecurityException) { }
                        handler.post { onConnected() }
                    } else {
                        handler.post {
                            Toast.makeText(this@ProjectDetailActivity, "TX characteristic not found", Toast.LENGTH_SHORT).show()
                            disconnect()
                        }
                    }
                } else {
                    handler.post {
                        Toast.makeText(this@ProjectDetailActivity, "UART service not found", Toast.LENGTH_SHORT).show()
                        disconnect()
                    }
                }
            }
        }

        @Deprecated("Deprecated in API 33")
        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            if (characteristic.uuid == CHARACTERISTIC_UUID_TX) {
                @Suppress("DEPRECATION")
                val data = characteristic.value
                if (data != null && data.isNotEmpty()) {
                    handler.post { processReceivedData(data) }
                }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
            if (characteristic.uuid == CHARACTERISTIC_UUID_TX && value.isNotEmpty()) {
                handler.post { processReceivedData(value) }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_project_detail)

        projectId = intent.getStringExtra("project_id") ?: ""
        if (projectId.isEmpty()) {
            Toast.makeText(this, "Invalid project", Toast.LENGTH_SHORT).show()
            finish()
            return
        }

        // Initialize Bluetooth
        val bluetoothManager = getSystemService(BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter
        bluetoothLeScanner = bluetoothAdapter?.bluetoothLeScanner

        initViews()
        loadProject()
        setupListeners()
    }

    private fun initViews() {
        tvProjectName = findViewById(R.id.tvProjectName)
        tvStatus = findViewById(R.id.tvStatus)
        tvDuration = findViewById(R.id.tvDuration)
        tvCreated = findViewById(R.id.tvCreated)

        // Rover section
        indicatorUbx = findViewById(R.id.indicatorUbx)
        indicatorCsv = findViewById(R.id.indicatorCsv)
        tvUbxFile = findViewById(R.id.tvUbxFile)
        tvCsvFile = findViewById(R.id.tvCsvFile)
        tvUbxStatus = findViewById(R.id.tvUbxStatus)
        tvCsvStatus = findViewById(R.id.tvCsvStatus)
        btnDownloadRover = findViewById(R.id.btnDownloadRover)
        layoutRoverProgress = findViewById(R.id.layoutRoverProgress)
        progressRover = findViewById(R.id.progressRover)
        tvRoverProgress = findViewById(R.id.tvRoverProgress)

        // Base section
        indicatorRtcm = findViewById(R.id.indicatorRtcm)
        tvRtcmFile = findViewById(R.id.tvRtcmFile)
        tvRtcmOverlap = findViewById(R.id.tvRtcmOverlap)
        btnFindBaseFile = findViewById(R.id.btnFindBaseFile)
        btnImportRinex = findViewById(R.id.btnImportRinex)
        layoutBaseProgress = findViewById(R.id.layoutBaseProgress)
        progressBase = findViewById(R.id.progressBase)
        tvBaseProgress = findViewById(R.id.tvBaseProgress)

        // PPK section
        tvPpkStatus = findViewById(R.id.tvPpkStatus)
        btnProcessPpk = findViewById(R.id.btnProcessPpk)
        layoutPpkProgress = findViewById(R.id.layoutPpkProgress)
        progressPpk = findViewById(R.id.progressPpk)
        tvPpkProgress = findViewById(R.id.tvPpkProgress)
        layoutPpkResults = findViewById(R.id.layoutPpkResults)
        tvFixedPercent = findViewById(R.id.tvFixedPercent)
        tvFloatPercent = findViewById(R.id.tvFloatPercent)
        tvSinglePercent = findViewById(R.id.tvSinglePercent)
        tvTotalPoints = findViewById(R.id.tvTotalPoints)

        // Export section
        btnExportCsv = findViewById(R.id.btnExportCsv)
        btnExportZip = findViewById(R.id.btnExportZip)

        btnDeleteProject = findViewById(R.id.btnDeleteProject)
        btnBack = findViewById(R.id.btnBack)
    }

    private fun loadProject() {
        val p = ProjectManager.getProject(projectId)
        if (p == null) {
            Toast.makeText(this, "Project not found", Toast.LENGTH_SHORT).show()
            finish()
            return
        }
        project = p
        updateUI()
    }

    private fun updateUI() {
        tvProjectName.text = project.name
        tvStatus.text = project.getStatusText()
        tvDuration.text = project.getRecordingDurationText()

        val dateFormat = SimpleDateFormat("MMM d, yyyy HH:mm", Locale.getDefault())
        tvCreated.text = dateFormat.format(Date(project.createdAt))

        // Rover files
        if (project.roverUbxFile != null) {
            tvUbxFile.text = project.roverUbxFile
            tvUbxStatus.text = "Downloaded"
            tvUbxStatus.setTextColor(0xFF4CAF50.toInt())
            indicatorUbx.setBackgroundColor(0xFF4CAF50.toInt())
        } else {
            tvUbxFile.text = "raw_XXXX.ubx (PPK data)"
            tvUbxStatus.text = "Not downloaded"
            tvUbxStatus.setTextColor(0xFF888888.toInt())
            indicatorUbx.setBackgroundColor(0xFF888888.toInt())
        }

        if (project.roverCsvFile != null) {
            tvCsvFile.text = project.roverCsvFile
            tvCsvStatus.text = "Downloaded"
            tvCsvStatus.setTextColor(0xFF4CAF50.toInt())
            indicatorCsv.setBackgroundColor(0xFF4CAF50.toInt())
        } else {
            tvCsvFile.text = "log_XXXX.csv (depth/IMU)"
            tvCsvStatus.text = "Not downloaded"
            tvCsvStatus.setTextColor(0xFF888888.toInt())
            indicatorCsv.setBackgroundColor(0xFF888888.toInt())
        }

        // Base file
        if (project.baseRtcmFile != null) {
            tvRtcmFile.text = project.baseRtcmFile
            indicatorRtcm.setBackgroundColor(0xFF4CAF50.toInt())
        } else if (project.baseRinexFile != null) {
            tvRtcmFile.text = project.baseRinexFile
            indicatorRtcm.setBackgroundColor(0xFF4CAF50.toInt())
        } else {
            tvRtcmFile.text = "No base file"
            indicatorRtcm.setBackgroundColor(0xFF888888.toInt())
        }

        // PPK status and button
        val hasRover = project.roverUbxFile != null && project.roverCsvFile != null
        val hasBase = project.baseRtcmFile != null || project.baseRinexFile != null

        when {
            project.status == ProjectStatus.PPK_COMPLETE -> {
                tvPpkStatus.text = "Processing complete"
                tvPpkStatus.setTextColor(0xFF4CAF50.toInt())
                btnProcessPpk.isEnabled = true
                btnProcessPpk.text = "Re-process PPK"
                layoutPpkResults.visibility = View.VISIBLE

                tvFixedPercent.text = "${project.getFixPercentage()}%"
                tvFloatPercent.text = "${project.getFloatPercentage()}%"
                tvSinglePercent.text = "${project.getSinglePercentage()}%"
                tvTotalPoints.text = "Total: ${project.totalPoints} points"

                btnExportCsv.isEnabled = true
            }
            hasRover && hasBase -> {
                tvPpkStatus.text = "Ready to process"
                tvPpkStatus.setTextColor(0xFF4CAF50.toInt())
                btnProcessPpk.isEnabled = true
                layoutPpkResults.visibility = View.GONE
                btnExportCsv.isEnabled = false
            }
            hasRover && !hasBase -> {
                tvPpkStatus.text = "Waiting for base station data..."
                tvPpkStatus.setTextColor(0xFFFFEB3B.toInt())
                btnProcessPpk.isEnabled = false
                layoutPpkResults.visibility = View.GONE
                btnExportCsv.isEnabled = false
            }
            else -> {
                tvPpkStatus.text = "Waiting for rover and base data..."
                tvPpkStatus.setTextColor(0xFF888888.toInt())
                btnProcessPpk.isEnabled = false
                layoutPpkResults.visibility = View.GONE
                btnExportCsv.isEnabled = false
            }
        }

        // Update download button text based on connection state
        if (isConnected) {
            btnDownloadRover.text = "Download Files"
        } else {
            btnDownloadRover.text = "Connect & Download"
        }
    }

    private fun setupListeners() {
        btnBack.setOnClickListener {
            finish()
        }

        btnDownloadRover.setOnClickListener {
            if (isConnected) {
                requestFileList()
            } else {
                startDownloadProcess()
            }
        }

        btnFindBaseFile.setOnClickListener {
            findMatchingBaseFile()
        }

        btnImportRinex.setOnClickListener {
            importRinexFile()
        }

        btnProcessPpk.setOnClickListener {
            processPpk()
        }

        btnExportCsv.setOnClickListener {
            exportMergedCsv()
        }

        btnExportZip.setOnClickListener {
            exportAllFilesZip()
        }

        btnDeleteProject.setOnClickListener {
            confirmDeleteProject()
        }
    }

    // ========== BLE Connection ==========

    private fun startDownloadProcess() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }

        val missingPermissions = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missingPermissions.isNotEmpty()) {
            ActivityCompat.requestPermissions(this, missingPermissions.toTypedArray(), REQUEST_BLUETOOTH_PERMISSIONS)
        } else {
            startScan()
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_BLUETOOTH_PERMISSIONS) {
            if (grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
                startScan()
            } else {
                Toast.makeText(this, "Bluetooth permissions required", Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun startScan() {
        if (isScanning) return

        layoutRoverProgress.visibility = View.VISIBLE
        btnDownloadRover.isEnabled = false
        tvRoverProgress.text = "Scanning for B-Box..."
        progressRover.isIndeterminate = true

        try {
            val scanSettings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()
            bluetoothLeScanner?.startScan(null, scanSettings, scanCallback)
            isScanning = true

            handler.postDelayed({
                if (isScanning && !isConnected) {
                    stopScan()
                    tvRoverProgress.text = "B-Box not found"
                    btnDownloadRover.isEnabled = true
                    Toast.makeText(this, "B-Box not found. Make sure it's powered on.", Toast.LENGTH_LONG).show()
                }
            }, SCAN_TIMEOUT_MS)
        } catch (e: SecurityException) {
            Toast.makeText(this, "BLE scan permission denied", Toast.LENGTH_SHORT).show()
            btnDownloadRover.isEnabled = true
            layoutRoverProgress.visibility = View.GONE
        }
    }

    private fun stopScan() {
        if (!isScanning) return
        try { bluetoothLeScanner?.stopScan(scanCallback) } catch (e: SecurityException) { }
        isScanning = false
    }

    private fun connectToDevice(device: BluetoothDevice) {
        handler.post { tvRoverProgress.text = "Connecting..." }
        try {
            bluetoothGatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
        } catch (e: SecurityException) {
            handler.post {
                Toast.makeText(this, "Connection permission denied", Toast.LENGTH_SHORT).show()
                btnDownloadRover.isEnabled = true
                layoutRoverProgress.visibility = View.GONE
            }
        }
    }

    private fun onConnected() {
        isConnected = true
        tvRoverProgress.text = "Connected! Requesting file list..."
        btnDownloadRover.text = "Download Files"

        // Request file list from B-Box
        handler.postDelayed({
            requestFileList()
        }, 500)
    }

    private fun onDisconnected() {
        isConnected = false
        bluetoothGatt = null
        rxCharacteristic = null
        txCharacteristic = null
        btnDownloadRover.isEnabled = true
        btnDownloadRover.text = "Connect & Download"

        if (!isDownloading) {
            layoutRoverProgress.visibility = View.GONE
        }
    }

    private fun disconnect() {
        stopScan()
        try {
            bluetoothGatt?.disconnect()
            bluetoothGatt?.close()
        } catch (e: SecurityException) { }
        onDisconnected()
    }

    private fun sendCommand(command: String) {
        if (!isConnected || rxCharacteristic == null || bluetoothGatt == null) return
        try {
            rxCharacteristic!!.value = ("$command\n").toByteArray()
            bluetoothGatt!!.writeCharacteristic(rxCharacteristic)
        } catch (e: SecurityException) { }
    }

    // ========== File Download ==========

    private fun requestFileList() {
        tvRoverProgress.text = "Requesting file list..."
        availableFiles.clear()
        sendCommand("LIST_LOGS")
    }

    private fun processReceivedData(data: ByteArray) {
        // Handle binary file data separately
        if (isDownloading && downloadBuffer.isNotEmpty()) {
            // Check if this is a text response or binary data
            val text = String(data)
            if (text.startsWith("FILE_END")) {
                saveDownloadedFile()
                return
            } else if (!text.startsWith("FILE_")) {
                // Append binary data
                downloadBuffer = downloadBuffer + data
                val kb = downloadBuffer.size / 1024
                tvRoverProgress.text = "Downloading $downloadFileName... ${kb}KB"
                return
            }
        }

        // Process as text
        val text = String(data)
        lineBuffer.append(text)

        while (lineBuffer.contains("\n")) {
            val newlineIndex = lineBuffer.indexOf("\n")
            val line = lineBuffer.substring(0, newlineIndex).trim()
            lineBuffer.delete(0, newlineIndex + 1)
            if (line.isNotEmpty()) processLine(line)
        }
    }

    private fun processLine(line: String) {
        android.util.Log.d("ProjectDetail", "RX: $line")

        when {
            line.startsWith("LOGS:") -> {
                parseFileList(line.substring(5))
            }
            line.startsWith("FILE_START:") -> {
                downloadBuffer = ByteArray(0)
                tvRoverProgress.text = "Downloading $downloadFileName..."
            }
            line.startsWith("FILE_DATA:") -> {
                // Text-based file data (for CSV files)
                val chunk = line.substring(10)
                downloadBuffer = downloadBuffer + chunk.toByteArray()
                val kb = downloadBuffer.size / 1024
                tvRoverProgress.text = "Downloading $downloadFileName... ${kb}KB"
            }
            line == "FILE_END" -> {
                saveDownloadedFile()
            }
            line == "ERR:NO_FILES" -> {
                tvRoverProgress.text = "No files on B-Box"
                layoutRoverProgress.visibility = View.GONE
                btnDownloadRover.isEnabled = true
                Toast.makeText(this, "No log files found on B-Box", Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun parseFileList(logsData: String) {
        if (logsData.isEmpty() || logsData == "EMPTY") {
            tvRoverProgress.text = "No files on B-Box"
            btnDownloadRover.isEnabled = true
            return
        }

        availableFiles.clear()
        val entries = logsData.split(";")
        for (entry in entries) {
            val parts = entry.split(",")
            if (parts.size >= 1 && parts[0].trim().isNotEmpty()) {
                availableFiles.add(parts[0].trim())
            }
        }

        android.util.Log.d("ProjectDetail", "Found ${availableFiles.size} files: $availableFiles")

        if (availableFiles.isEmpty()) {
            tvRoverProgress.text = "No files on B-Box"
            btnDownloadRover.isEnabled = true
            return
        }

        // Show ALL files - multi-select dialog
        showAllFilesDialog()
    }

    private fun showAllFilesDialog() {
        val fileList = availableFiles.toTypedArray()
        val selected = BooleanArray(fileList.size) { true }  // All selected by default

        AlertDialog.Builder(this, R.style.AlertDialogTheme)
            .setTitle("Select Files to Download")
            .setMultiChoiceItems(fileList, selected) { _, which, isChecked ->
                selected[which] = isChecked
            }
            .setPositiveButton("Download") { _, _ ->
                filesToDownload.clear()
                for (i in fileList.indices) {
                    if (selected[i]) {
                        filesToDownload.add(fileList[i])
                    }
                }
                if (filesToDownload.isNotEmpty()) {
                    startDownloadingFiles()
                } else {
                    tvRoverProgress.text = "No files selected"
                    btnDownloadRover.isEnabled = true
                }
            }
            .setNegativeButton("Cancel") { _, _ ->
                tvRoverProgress.text = "Download cancelled"
                btnDownloadRover.isEnabled = true
                disconnect()
            }
            .show()
    }

    private fun showFileSelectionDialog(ubxFiles: List<String>, csvFiles: List<String>) {
        // Find matching pairs by base name (without extension)
        val pairs = mutableListOf<Pair<String?, String?>>()
        val allBaseNames = mutableSetOf<String>()

        for (f in ubxFiles) {
            // Remove extension and optional raw_ prefix
            var baseName = f.substringBeforeLast(".")
            if (baseName.startsWith("raw_")) baseName = baseName.removePrefix("raw_")
            allBaseNames.add(baseName)
        }
        for (f in csvFiles) {
            // Remove extension and optional log_ prefix
            var baseName = f.substringBeforeLast(".")
            if (baseName.startsWith("log_")) baseName = baseName.removePrefix("log_")
            allBaseNames.add(baseName)
        }

        for (baseName in allBaseNames.sorted()) {
            // Match by base name - check both legacy and project naming
            val ubx = ubxFiles.find {
                val name = it.substringBeforeLast(".")
                name == baseName || name == "raw_$baseName"
            }
            val csv = csvFiles.find {
                val name = it.substringBeforeLast(".")
                name == baseName || name == "log_$baseName"
            }
            pairs.add(Pair(ubx, csv))
        }

        if (pairs.isEmpty()) {
            tvRoverProgress.text = "No file pairs found"
            btnDownloadRover.isEnabled = true
            return
        }

        // If only one pair, download it directly
        if (pairs.size == 1) {
            val pair = pairs[0]
            filesToDownload.clear()
            pair.first?.let { filesToDownload.add(it) }
            pair.second?.let { filesToDownload.add(it) }
            startDownloadingFiles()
            return
        }

        // Multiple pairs - let user choose
        val items = pairs.map { pair ->
            val num = pair.first?.removePrefix("raw_")?.removeSuffix(".ubx")
                ?: pair.second?.removePrefix("log_")?.removeSuffix(".csv") ?: "?"
            val hasUbx = if (pair.first != null) "UBX" else ""
            val hasCsv = if (pair.second != null) "CSV" else ""
            "Session $num ($hasUbx $hasCsv)"
        }.toTypedArray()

        AlertDialog.Builder(this, R.style.AlertDialogTheme)
            .setTitle("Select Session to Download")
            .setItems(items) { _, which ->
                val pair = pairs[which]
                filesToDownload.clear()
                pair.first?.let { filesToDownload.add(it) }
                pair.second?.let { filesToDownload.add(it) }
                startDownloadingFiles()
            }
            .setNegativeButton("Cancel") { _, _ ->
                btnDownloadRover.isEnabled = true
                layoutRoverProgress.visibility = View.GONE
            }
            .show()
    }

    private fun startDownloadingFiles() {
        if (filesToDownload.isEmpty()) {
            tvRoverProgress.text = "No files to download"
            btnDownloadRover.isEnabled = true
            layoutRoverProgress.visibility = View.GONE
            return
        }

        currentDownloadIndex = 0
        downloadNextFile()
    }

    private fun downloadNextFile() {
        if (currentDownloadIndex >= filesToDownload.size) {
            // All downloads complete
            isDownloading = false
            tvRoverProgress.text = "Download complete!"
            progressRover.isIndeterminate = false
            progressRover.progress = 100

            handler.postDelayed({
                layoutRoverProgress.visibility = View.GONE
                btnDownloadRover.isEnabled = true
                disconnect()
            }, 1500)
            return
        }

        downloadFileName = filesToDownload[currentDownloadIndex]
        downloadFileType = if (downloadFileName.endsWith(".ubx")) "ubx" else "csv"
        downloadBuffer = ByteArray(0)
        isDownloading = true

        tvRoverProgress.text = "Downloading $downloadFileName..."
        sendCommand("DOWNLOAD:$downloadFileName")
    }

    private fun saveDownloadedFile() {
        try {
            val roverDir = ProjectManager.getRoverDir(projectId)
            if (!roverDir.exists()) roverDir.mkdirs()

            val file = File(roverDir, downloadFileName)
            file.writeBytes(downloadBuffer)

            android.util.Log.d("ProjectDetail", "Saved ${downloadBuffer.size} bytes to ${file.absolutePath}")

            // Update project
            if (downloadFileType == "ubx") {
                project.roverUbxFile = downloadFileName
            } else {
                project.roverCsvFile = downloadFileName
            }

            // Update status if both files downloaded
            if (project.roverUbxFile != null && project.roverCsvFile != null) {
                project.status = ProjectStatus.FILES_READY
            }

            ProjectManager.updateProject(project)
            updateUI()

            Toast.makeText(this, "Saved: $downloadFileName", Toast.LENGTH_SHORT).show()

        } catch (e: Exception) {
            Toast.makeText(this, "Failed to save: ${e.message}", Toast.LENGTH_SHORT).show()
            android.util.Log.e("ProjectDetail", "Save error: ${e.message}")
        }

        // Download next file
        currentDownloadIndex++
        downloadNextFile()
    }

    // ========== Other Functions ==========

    private fun findMatchingBaseFile() {
        if (project.recordingStartTime == 0L) {
            Toast.makeText(this, "No recording time available. Record data first.", Toast.LENGTH_LONG).show()
            return
        }

        layoutBaseProgress.visibility = View.VISIBLE
        btnFindBaseFile.isEnabled = false
        tvBaseProgress.text = "Searching Shore Station for matching files..."

        Toast.makeText(this, "Shore Station connection not yet implemented", Toast.LENGTH_LONG).show()

        layoutBaseProgress.visibility = View.GONE
        btnFindBaseFile.isEnabled = true
    }

    private fun importRinexFile() {
        Toast.makeText(this, "RINEX import not yet implemented", Toast.LENGTH_SHORT).show()
    }

    private fun processPpk() {
        layoutPpkProgress.visibility = View.VISIBLE
        btnProcessPpk.isEnabled = false
        tvPpkProgress.text = "Processing PPK data..."

        Toast.makeText(this, "PPK processing not yet implemented", Toast.LENGTH_LONG).show()

        layoutPpkProgress.visibility = View.GONE
        btnProcessPpk.isEnabled = true
    }

    private fun exportMergedCsv() {
        Toast.makeText(this, "CSV export not yet implemented", Toast.LENGTH_SHORT).show()
    }

    private fun exportAllFilesZip() {
        val zipFile = ProjectManager.exportProjectZip(projectId)
        if (zipFile != null) {
            Toast.makeText(this, "Exported to: ${zipFile.name}", Toast.LENGTH_LONG).show()
        } else {
            Toast.makeText(this, "Export failed", Toast.LENGTH_SHORT).show()
        }
    }

    private fun confirmDeleteProject() {
        AlertDialog.Builder(this, R.style.AlertDialogTheme)
            .setTitle("Delete Project")
            .setMessage("Are you sure you want to delete '${project.name}'? This cannot be undone.")
            .setPositiveButton("Delete") { _, _ ->
                ProjectManager.deleteProject(projectId)
                Toast.makeText(this, "Project deleted", Toast.LENGTH_SHORT).show()
                finish()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    override fun onDestroy() {
        super.onDestroy()
        disconnect()
    }
}
