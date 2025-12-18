package com.deeperrtk.remote

import android.Manifest
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import android.widget.ViewFlipper
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.android.material.tabs.TabLayout
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.UUID
import kotlin.concurrent.thread
import android.widget.ListView
import android.widget.ArrayAdapter
import android.widget.SeekBar
import android.app.AlertDialog
import android.os.Environment
import java.io.File
import java.io.FileOutputStream

class MainActivity : AppCompatActivity() {

    companion object {
        private const val DEVICE_NAME = "DeeperRTK"

        // Nordic UART Service UUIDs (must match firmware)
        private val SERVICE_UUID: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        private val CHARACTERISTIC_UUID_RX: UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
        private val CHARACTERISTIC_UUID_TX: UUID = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")

        // Client Characteristic Configuration Descriptor
        private val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        private const val REQUEST_BLUETOOTH_PERMISSIONS = 1
        private const val SCAN_TIMEOUT_MS = 10000L

        // Deeper Sonar UDP settings
        private const val DEEPER_IP = "192.168.10.1"
        private const val DEEPER_PORT = 10110
        private const val NMEA_ENABLE_CMD = "\$DEEP230,1*38"
    }

    data class LogFile(val name: String, val size: Long, val dateModified: Long)

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothLeScanner: BluetoothLeScanner? = null
    private var bluetoothGatt: BluetoothGatt? = null
    private var rxCharacteristic: BluetoothGattCharacteristic? = null
    private var txCharacteristic: BluetoothGattCharacteristic? = null

    private var isConnected = false
    private var isScanning = false
    private var isRecording = false

    private val handler = Handler(Looper.getMainLooper())
    private val lineBuffer = StringBuilder()

    // Polling runnable for requesting status updates
    private val pollRunnable = object : Runnable {
        override fun run() {
            if (isConnected) {
                sendCommand("STATUS")
                handler.postDelayed(this, 67) // Poll at 15Hz (~67ms)
            }
        }
    }

    // UI Elements - Remote Tab
    private lateinit var tabLayout: TabLayout
    private lateinit var viewFlipper: ViewFlipper
    private lateinit var connectButton: Button
    private lateinit var connectionIndicator: View
    private lateinit var connectionStatus: TextView
    private lateinit var fixIndicator: View
    private lateinit var fixStatus: TextView
    private lateinit var satellites: TextView
    private lateinit var latLonValue: TextView
    private lateinit var depthValue: TextView
    private lateinit var tempValue: TextView
    private lateinit var pitchValue: TextView
    private lateinit var rollValue: TextView
    private lateinit var recordingIndicator: View
    private lateinit var startButton: Button
    private lateinit var stopButton: Button
    private lateinit var sonarGLView: SonarGLSurfaceView

    // UI Elements - Config Tab
    private lateinit var deeperSsidInput: EditText
    private lateinit var enableNmeaButton: Button
    private lateinit var nmeaIndicator: View
    private lateinit var nmeaStatus: TextView

    // IMU Calibration UI
    private lateinit var levelButton: Button
    private lateinit var imuCalibIndicator: View
    private lateinit var imuCalibStatus: TextView

    // Logs Tab UI
    private lateinit var logsStatus: TextView
    private lateinit var refreshLogsButton: Button
    private lateinit var logsListView: ListView
    private lateinit var deleteAllButton: Button
    private val logFiles = mutableListOf<LogFile>()
    private lateinit var logsAdapter: ArrayAdapter<String>
    private var isDownloading = false
    private var downloadFileName = ""
    private val downloadBuffer = StringBuilder()

    // BLE Scan callback
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            try {
                val deviceName = device.name ?: result.scanRecord?.deviceName
                val serviceUuids = result.scanRecord?.serviceUuids
                val hasOurService = serviceUuids?.any { it.uuid == SERVICE_UUID } == true

                if (deviceName?.contains("DeeperRTK", ignoreCase = true) == true || hasOurService) {
                    android.util.Log.d("DeeperRTK", "Found device: $deviceName, hasService: $hasOurService")
                    stopScan()
                    connectToDevice(device)
                }
            } catch (e: SecurityException) {
                // Ignore devices we can't read
            }
        }

        override fun onScanFailed(errorCode: Int) {
            handler.post {
                Toast.makeText(this@MainActivity, "BLE scan failed: $errorCode", Toast.LENGTH_SHORT).show()
                connectButton.isEnabled = true
                connectionStatus.text = "Scan failed"
            }
        }
    }

    // BLE GATT callback
    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    handler.post { connectionStatus.text = "Requesting MTU..." }
                    try { gatt.requestMtu(512) } catch (e: SecurityException) {
                        handler.post { Toast.makeText(this@MainActivity, "Permission denied", Toast.LENGTH_SHORT).show() }
                    }
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    handler.post { onDisconnected() }
                }
            }
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
                        } catch (e: SecurityException) {
                            handler.post { Toast.makeText(this@MainActivity, "Permission denied", Toast.LENGTH_SHORT).show() }
                        }
                        handler.post { onConnected() }
                    } else {
                        handler.post { Toast.makeText(this@MainActivity, "TX characteristic not found", Toast.LENGTH_SHORT).show(); disconnect() }
                    }
                } else {
                    handler.post { Toast.makeText(this@MainActivity, "UART service not found", Toast.LENGTH_SHORT).show(); disconnect() }
                }
            } else {
                handler.post { Toast.makeText(this@MainActivity, "Service discovery failed", Toast.LENGTH_SHORT).show(); disconnect() }
            }
        }

        @Deprecated("Deprecated in API 33")
        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            if (characteristic.uuid == CHARACTERISTIC_UUID_TX) {
                @Suppress("DEPRECATION")
                val data = characteristic.value
                if (data != null && data.isNotEmpty()) {
                    val text = String(data)
                    handler.post { processReceivedData(text) }
                }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
            if (characteristic.uuid == CHARACTERISTIC_UUID_TX && value.isNotEmpty()) {
                val text = String(value)
                handler.post { processReceivedData(text) }
            }
        }

        override fun onCharacteristicWrite(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                handler.post { Toast.makeText(this@MainActivity, "Write failed", Toast.LENGTH_SHORT).show() }
            }
        }

        override fun onDescriptorWrite(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            android.util.Log.d("DeeperRTK", "onDescriptorWrite status=$status")
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            android.util.Log.d("DeeperRTK", "MTU changed to $mtu")
            handler.post { connectionStatus.text = "Discovering services..." }
            try { gatt.discoverServices() } catch (e: SecurityException) { }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Initialize UI elements - Tabs
        tabLayout = findViewById(R.id.tabLayout)
        viewFlipper = findViewById(R.id.viewFlipper)

        // Remote tab elements
        connectButton = findViewById(R.id.connectButton)
        connectionIndicator = findViewById(R.id.connectionIndicator)
        connectionStatus = findViewById(R.id.connectionStatus)
        fixIndicator = findViewById(R.id.fixIndicator)
        fixStatus = findViewById(R.id.fixStatus)
        satellites = findViewById(R.id.satellites)
        latLonValue = findViewById(R.id.latLonValue)
        depthValue = findViewById(R.id.depthValue)
        tempValue = findViewById(R.id.tempValue)
        pitchValue = findViewById(R.id.pitchValue)
        rollValue = findViewById(R.id.rollValue)
        recordingIndicator = findViewById(R.id.recordingIndicator)
        startButton = findViewById(R.id.startButton)
        stopButton = findViewById(R.id.stopButton)
        sonarGLView = findViewById(R.id.sonarGLView)

        // Config tab elements
        deeperSsidInput = findViewById(R.id.deeperSsidInput)
        enableNmeaButton = findViewById(R.id.enableNmeaButton)
        nmeaIndicator = findViewById(R.id.nmeaIndicator)
        nmeaStatus = findViewById(R.id.nmeaStatus)

        // IMU Calibration bindings
        levelButton = findViewById(R.id.levelButton)
        imuCalibIndicator = findViewById(R.id.imuCalibIndicator)
        imuCalibStatus = findViewById(R.id.imuCalibStatus)

        levelButton.setOnClickListener { calibrateIMU() }

        // Set up tabs
        tabLayout.addTab(tabLayout.newTab().setText("Remote"))
        tabLayout.addTab(tabLayout.newTab().setText("Config"))
        tabLayout.addTab(tabLayout.newTab().setText("Logs"))

        tabLayout.addOnTabSelectedListener(object : TabLayout.OnTabSelectedListener {
            override fun onTabSelected(tab: TabLayout.Tab) {
                viewFlipper.displayedChild = tab.position
            }
            override fun onTabUnselected(tab: TabLayout.Tab) {}
            override fun onTabReselected(tab: TabLayout.Tab) {}
        })

        // Initialize Bluetooth
        val bluetoothManager = getSystemService(BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter

        if (bluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth not supported", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        bluetoothLeScanner = bluetoothAdapter?.bluetoothLeScanner

        // Set up button listeners
        connectButton.setOnClickListener {
            if (isConnected) disconnect() else checkPermissionsAndConnect()
        }

        startButton.setOnClickListener { sendCommand("START") }
        stopButton.setOnClickListener { sendCommand("STOP") }

        // Enable NMEA button - sends UDP command to Deeper sonar
        enableNmeaButton.setOnClickListener { sendNmeaEnableCommand() }

        // Logs tab bindings
        logsStatus = findViewById(R.id.logsStatus)
        refreshLogsButton = findViewById(R.id.refreshLogsButton)
        logsListView = findViewById(R.id.logsListView)
        deleteAllButton = findViewById(R.id.deleteAllButton)

        // Setup logs list adapter
        logsAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, mutableListOf<String>())
        logsListView.adapter = logsAdapter

        refreshLogsButton.setOnClickListener { requestLogsList() }
        deleteAllButton.setOnClickListener { showDeleteConfirmDialog() }
        logsListView.setOnItemClickListener { _, _, position, _ ->
            if (position < logFiles.size) {
                downloadLog(logFiles[position].name)
            }
        }
    }

    private fun calibrateIMU() {
        if (!isConnected) {
            Toast.makeText(this, "Not connected to B-BOX", Toast.LENGTH_SHORT).show()
            return
        }
        imuCalibStatus.text = "Calibrating..."
        imuCalibIndicator.setBackgroundResource(R.drawable.indicator_disconnected)
        sendCommand("IMU_LEVEL")
    }

    private fun sendNmeaEnableCommand() {
        nmeaStatus.text = "Sending..."
        nmeaIndicator.setBackgroundResource(R.drawable.indicator_disconnected)

        thread {
            try {
                val socket = DatagramSocket()
                socket.soTimeout = 2000
                val address = InetAddress.getByName(DEEPER_IP)
                val data = (NMEA_ENABLE_CMD + "\r\n").toByteArray()
                val packet = DatagramPacket(data, data.size, address, DEEPER_PORT)
                socket.send(packet)
                socket.close()

                handler.post {
                    nmeaStatus.text = "Sent OK"
                    nmeaIndicator.setBackgroundResource(R.drawable.indicator_green)
                }
            } catch (e: Exception) {
                handler.post {
                    nmeaStatus.text = "Failed: " + e.message
                    nmeaIndicator.setBackgroundResource(R.drawable.indicator_red)
                }
            }
        }
    }

    private fun checkPermissionsAndConnect() {
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
        connectButton.isEnabled = false
        connectionStatus.text = "Scanning for DeeperRTK..."

        try {
            val scanSettings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()
            bluetoothLeScanner?.startScan(null, scanSettings, scanCallback)
            isScanning = true

            handler.postDelayed({
                if (isScanning && !isConnected) {
                    stopScan()
                    connectionStatus.text = "Device not found"
                    connectButton.isEnabled = true
                    Toast.makeText(this, "DeeperRTK not found. Make sure it's powered on.", Toast.LENGTH_LONG).show()
                }
            }, SCAN_TIMEOUT_MS)
        } catch (e: SecurityException) {
            Toast.makeText(this, "BLE scan permission denied", Toast.LENGTH_SHORT).show()
            connectButton.isEnabled = true
        }
    }

    private fun stopScan() {
        if (!isScanning) return
        try { bluetoothLeScanner?.stopScan(scanCallback) } catch (e: SecurityException) { }
        isScanning = false
    }

    private fun connectToDevice(device: BluetoothDevice) {
        handler.post { connectionStatus.text = "Connecting..." }
        try {
            bluetoothGatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
        } catch (e: SecurityException) {
            handler.post {
                Toast.makeText(this, "Connection permission denied", Toast.LENGTH_SHORT).show()
                connectButton.isEnabled = true
            }
        }
    }

    private fun onConnected() {
        isConnected = true
        connectButton.text = "Disconnect"
        connectButton.isEnabled = true
        connectionIndicator.setBackgroundResource(R.drawable.indicator_green)
        connectionStatus.text = "Connected (BLE)"
        startButton.isEnabled = true
        stopButton.isEnabled = true
        handler.postDelayed(pollRunnable, 500)
    }

    private fun onDisconnected() {
        isConnected = false
        bluetoothGatt = null
        rxCharacteristic = null
        txCharacteristic = null
        lineBuffer.clear()
        handler.removeCallbacks(pollRunnable)

        connectButton.text = "Connect"
        connectButton.isEnabled = true
        connectionIndicator.setBackgroundResource(R.drawable.indicator_disconnected)
        connectionStatus.text = "Disconnected"
        startButton.isEnabled = false
        stopButton.isEnabled = false

        // Reset display
        fixStatus.text = "No Fix"
        fixIndicator.setBackgroundResource(R.drawable.indicator_no_fix)
        satellites.text = "0"
        latLonValue.text = "---, ---"
        depthValue.text = "-- m"
        tempValue.text = "-- °C"
        pitchValue.text = "--°"
        rollValue.text = "--°"
        recordingIndicator.setBackgroundResource(R.drawable.indicator_disconnected)
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
            rxCharacteristic!!.value = ("$command" + "\n").toByteArray()
            bluetoothGatt!!.writeCharacteristic(rxCharacteristic)
        } catch (e: SecurityException) {
            Toast.makeText(this, "Write permission denied", Toast.LENGTH_SHORT).show()
        }
    }

    private fun processReceivedData(data: String) {
        lineBuffer.append(data)
        while (lineBuffer.contains("\n")) {
            val newlineIndex = lineBuffer.indexOf("\n")
            val line = lineBuffer.substring(0, newlineIndex).trim()
            lineBuffer.delete(0, newlineIndex + 1)
            if (line.isNotEmpty()) processData(line)
        }
    }

    private fun processData(line: String) {
        // Handle command responses
        // Handle logs-related responses
        if (line.startsWith("LOGS:") || line.startsWith("FILE_") || line == "FILE_END" ||
            line == "OK:DELETE_ALL" || line == "ERR:NO_FILES") {
            handleLogsResponse(line)
            return
        }

        if (line == "OK:IMU_LEVEL") {
            imuCalibStatus.text = "Calibrated"
            imuCalibIndicator.setBackgroundResource(R.drawable.indicator_green)
            Toast.makeText(this, "IMU leveled successfully", Toast.LENGTH_SHORT).show()
            return
        }

        if (line.startsWith("OK:")) {
            when (line) {
                "OK:REC_ON" -> {
                    isRecording = true
                    recordingIndicator.setBackgroundResource(R.drawable.indicator_red)
                }
                "OK:REC_OFF" -> {
                    isRecording = false
                    recordingIndicator.setBackgroundResource(R.drawable.indicator_disconnected)
                }
            }
            return
        }

        // Parse status: FIX,SATS,DEPTH,TEMP,BATT,REC,PITCH,ROLL
        val parts = line.split(",")
        if (parts.size >= 6) {
            try {
                val fix = parts[0].toIntOrNull() ?: 0
                val sats = parts[1].toIntOrNull() ?: 0
                val depth = parts[2].toFloatOrNull() ?: -1f
                val temp = parts[3].toFloatOrNull() ?: 0f
                val recording = parts[5].trim() == "1"
                val pitch = if (parts.size >= 7) parts[6].toFloatOrNull() ?: 0f else 0f
                val roll = if (parts.size >= 8) parts[7].trim().toFloatOrNull() ?: 0f else 0f
                val lat = if (parts.size >= 9) parts[8].trim().toDoubleOrNull() ?: 0.0 else 0.0
                val lon = if (parts.size >= 10) parts[9].trim().toDoubleOrNull() ?: 0.0 else 0.0
                updateDisplay(fix, sats, depth, temp, recording, pitch, roll, lat, lon)
            } catch (e: Exception) {
                android.util.Log.e("DeeperRTK", "Parse error: " + e.message)
            }
        }
    }

    private fun updateDisplay(fix: Int, sats: Int, depth: Float, temp: Float, recording: Boolean, pitch: Float, roll: Float, lat: Double = 0.0, lon: Double = 0.0) {
        // GPS Fix Status
        when (fix) {
            0 -> { fixStatus.text = "No Fix"; fixStatus.setTextColor(0xFF888888.toInt()); fixIndicator.setBackgroundResource(R.drawable.indicator_no_fix) }
            1 -> { fixStatus.text = "GPS"; fixStatus.setTextColor(0xFFFFEB3B.toInt()); fixIndicator.setBackgroundResource(R.drawable.indicator_gps) }
            2 -> { fixStatus.text = "DGPS"; fixStatus.setTextColor(0xFFFF9800.toInt()); fixIndicator.setBackgroundResource(R.drawable.indicator_dgps) }
            4 -> { fixStatus.text = "RTK Fixed"; fixStatus.setTextColor(0xFF4CAF50.toInt()); fixIndicator.setBackgroundResource(R.drawable.indicator_green) }
            5 -> { fixStatus.text = "RTK Float"; fixStatus.setTextColor(0xFF00BCD4.toInt()); fixIndicator.setBackgroundResource(R.drawable.indicator_float) }
            else -> { fixStatus.text = "???"; fixStatus.setTextColor(0xFF888888.toInt()); fixIndicator.setBackgroundResource(R.drawable.indicator_no_fix) }
        }

        satellites.text = sats.toString()
        // Color code satellites: red <5, yellow 5-8, green >8
        satellites.setTextColor(when {
            sats < 5 -> 0xFFf44336.toInt()   // Red
            sats <= 8 -> 0xFFFFEB3B.toInt()  // Yellow
            else -> 0xFF4CAF50.toInt()       // Green
        })
        if (lat != 0.0 && lon != 0.0) {
            latLonValue.text = String.format("%.6f, %.6f", lat, lon)
        } else {
            latLonValue.text = "---, ---"
        }
        depthValue.text = if (depth >= 0) String.format("%.2f m", depth) else "-- m"
        tempValue.text = if (temp > 0) String.format("%.1f °C", temp) else "-- °C"
        pitchValue.text = String.format("%.1f°", pitch)
        rollValue.text = String.format("%.1f°", roll)

        // Update 3D model with pitch/roll
        sonarGLView.setPitchRoll(pitch, roll)

        isRecording = recording
        recordingIndicator.setBackgroundResource(if (recording) R.drawable.indicator_red else R.drawable.indicator_disconnected)
    }


    // Logs Tab Functions
    private fun requestLogsList() {
        if (!isConnected) {
            Toast.makeText(this, "Not connected", Toast.LENGTH_SHORT).show()
            return
        }
        logsStatus.text = "Requesting logs..."
        logFiles.clear()
        logsAdapter.clear()
        sendCommand("LIST_LOGS")
    }

    private fun downloadLog(filename: String) {
        if (!isConnected) {
            Toast.makeText(this, "Not connected", Toast.LENGTH_SHORT).show()
            return
        }
        isDownloading = true
        downloadFileName = filename
        downloadBuffer.clear()
        logsStatus.text = "Downloading $filename..."
        sendCommand("DOWNLOAD:$filename")
    }

    private fun showDeleteConfirmDialog() {
        if (!isConnected) {
            Toast.makeText(this, "Not connected", Toast.LENGTH_SHORT).show()
            return
        }

        val dialogView = layoutInflater.inflate(R.layout.dialog_delete_confirm, null)
        val slider = dialogView.findViewById<SeekBar>(R.id.deleteSlider)
        val cancelButton = dialogView.findViewById<Button>(R.id.keepFilesButton)

        val dialog = AlertDialog.Builder(this)
            .setView(dialogView)
            .setCancelable(true)
            .create()

        slider.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {}
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {
                if (seekBar != null && seekBar.progress >= 95) {
                    dialog.dismiss()
                    deleteAllLogs()
                } else {
                    seekBar?.progress = 0
                }
            }
        })

        cancelButton.setOnClickListener {
            dialog.dismiss()
        }

        dialog.show()
    }

    private fun deleteAllLogs() {
        logsStatus.text = "Deleting all logs..."
        sendCommand("DELETE_ALL")
    }

    private fun handleLogsResponse(line: String) {
        when {
            line.startsWith("LOGS:") -> {
                // Parse log list: LOGS:name1,size1,date1;name2,size2,date2;...
                val logsData = line.substring(5)
                if (logsData.isEmpty() || logsData == "EMPTY") {
                    logsStatus.text = "No logs found"
                    return
                }
                val entries = logsData.split(";")
                logFiles.clear()
                logsAdapter.clear()
                for (entry in entries) {
                    val parts = entry.split(",")
                    if (parts.size >= 2) {
                        val name = parts[0]
                        val size = parts[1].toLongOrNull() ?: 0L
                        val date = if (parts.size >= 3) parts[2].toLongOrNull() ?: 0L else 0L
                        logFiles.add(LogFile(name, size, date))
                        val sizeStr = formatFileSize(size)
                        logsAdapter.add("$name ($sizeStr)")
                    }
                }
                logsStatus.text = "${logFiles.size} logs found"
            }
            line.startsWith("FILE_START:") -> {
                downloadBuffer.clear()
            }
            line.startsWith("FILE_DATA:") -> {
                downloadBuffer.append(line.substring(10))
            }
            line == "FILE_END" -> {
                saveDownloadedFile()
            }
            line == "OK:DELETE_ALL" -> {
                logFiles.clear()
                logsAdapter.clear()
                logsStatus.text = "All logs deleted"
                Toast.makeText(this, "All logs deleted", Toast.LENGTH_SHORT).show()
            }
            line == "ERR:NO_FILES" -> {
                logsStatus.text = "No logs on device"
            }
        }
    }

    private fun formatFileSize(bytes: Long): String {
        return when {
            bytes < 1024 -> "$bytes B"
            bytes < 1024 * 1024 -> String.format("%.1f KB", bytes / 1024.0)
            else -> String.format("%.1f MB", bytes / (1024.0 * 1024.0))
        }
    }

    private fun saveDownloadedFile() {
        try {
            val downloadsDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS)
            val file = File(downloadsDir, downloadFileName)
            FileOutputStream(file).use { fos ->
                fos.write(downloadBuffer.toString().toByteArray())
            }
            logsStatus.text = "Saved: $downloadFileName"
            Toast.makeText(this, "Downloaded to Downloads/$downloadFileName", Toast.LENGTH_LONG).show()
        } catch (e: Exception) {
            logsStatus.text = "Save failed: ${e.message}"
            Toast.makeText(this, "Failed to save: ${e.message}", Toast.LENGTH_SHORT).show()
        }
        isDownloading = false
        downloadBuffer.clear()
    }

    override fun onDestroy() {
        super.onDestroy()
        handler.removeCallbacks(pollRunnable)
        disconnect()
    }
}
