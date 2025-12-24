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
import android.widget.CheckBox
import androidx.appcompat.widget.SwitchCompat
import android.app.AlertDialog
import android.os.Environment
import java.io.File
import java.io.FileOutputStream
import android.net.Uri
import android.provider.OpenableColumns
import android.app.Activity
import org.osmdroid.config.Configuration
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.tileprovider.tilesource.XYTileSource
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Marker
import org.osmdroid.views.overlay.Polyline
import org.osmdroid.views.overlay.Polygon
import android.graphics.Color
import android.graphics.Paint
import android.content.Intent
import android.content.SharedPreferences
import android.preference.PreferenceManager

class MainActivity : AppCompatActivity() {

    companion object {
        private const val DEVICE_NAME = "BBOX"
        private const val REQUEST_IMPORT_RINEX = 2001
        private const val SHORE_SERVICE_UUID_STR = "6E400001-B5A3-F393-E0A9-E50E24DCCA9F"

        // Nordic UART Service UUIDs - B-Box uses standard Nordic UART
        private val SERVICE_UUID: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        private val CHARACTERISTIC_UUID_RX: UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
        private val CHARACTERISTIC_UUID_TX: UUID = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")

        // Shore Station uses different service UUID but SAME characteristic UUIDs
        private val SHORE_SERVICE_UUID: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9F")

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
    private var isRadioLinkMode = false

    // NTRIP Client state
    private var isNtripConnected = false
    private var ntripSocket: java.net.Socket? = null
    private var ntripThread: Thread? = null
    private var rtcmBytesSent = 0L
    private var rtcmMessagesReceived = 0L

    // BLE write synchronization - prevents concurrent writes that cause disconnects
    private val bleLock = Object()
    private var lastBleWriteTime = 0L
    private val BLE_WRITE_INTERVAL_MS = 30L  // Minimum time between BLE writes

    // RTCM message counters
    private var cntStation = 0L
    private var cntGps = 0L
    private var cntGlo = 0L
    private var cntGal = 0L
    private var cntBds = 0L
    private var cntBias = 0L

    // RTCM rate limits (Hz, 0 = disabled)
    private var hzStation = 0.1
    private var hzGps = 1.0
    private var hzGlo = 1.0
    private var hzGal = 1.0
    private var hzBds = 1.0
    private var hzBias = 0.2

    // Last send times for rate limiting
    private var lastStationTime = 0L
    private var lastGpsTime = 0L
    private var lastGloTime = 0L
    private var lastGalTime = 0L
    private var lastBdsTime = 0L
    private var lastBiasTime = 0L

    private val handler = Handler(Looper.getMainLooper())
    private val lineBuffer = StringBuilder()

    // Polling runnable for requesting status updates
    private val pollRunnable = object : Runnable {
        override fun run() {
            if (isConnected) {
                sendCommand("STATUS")
                // Reduce polling rate when NTRIP is active to prevent BLE congestion
                val pollInterval = if (isNtripConnected) 500L else 67L
                handler.postDelayed(this, pollInterval)
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

    // SharedPreferences for saving settings
    private lateinit var prefs: SharedPreferences

    // UI Elements - NTRIP
    private lateinit var ntripHost: EditText
    private lateinit var ntripPort: EditText
    private lateinit var ntripMountpoint: EditText
    private lateinit var ntripUsername: EditText
    private lateinit var ntripPassword: EditText
    private lateinit var ntripConnectButton: Button
    private lateinit var ntripIndicator: View
    private lateinit var ntripStatus: TextView
    private lateinit var ntripStats: TextView

    // UI Elements - Config Tab
    private lateinit var deeperSsidInput: EditText
    private lateinit var enableNmeaButton: Button
    private lateinit var nmeaIndicator: View
    private lateinit var nmeaStatus: TextView

    // IMU Calibration UI
    private lateinit var levelButton: Button
    private lateinit var shoreNtripSwitch: SwitchCompat
    private lateinit var imuCalibIndicator: View
    private lateinit var imuCalibStatus: TextView

    // RTCM Rate Control UI
    private lateinit var hzStationEdit: EditText
    private lateinit var hzGpsEdit: EditText
    private lateinit var hzGloEdit: EditText
    private lateinit var hzGalEdit: EditText
    private lateinit var hzBdsEdit: EditText
    private lateinit var hzBiasEdit: EditText
    private lateinit var cntStationText: TextView
    private lateinit var cntGpsText: TextView
    private lateinit var cntGloText: TextView
    private lateinit var cntGalText: TextView
    private lateinit var cntBdsText: TextView
    private lateinit var cntBiasText: TextView

    // NMEA Message Filter UI
    private lateinit var filterGGA: CheckBox
    private lateinit var filterRMC: CheckBox
    private lateinit var filterVTG: CheckBox
    private lateinit var filterGSA: CheckBox
    private lateinit var filterDBT: CheckBox
    private lateinit var filterMTW: CheckBox
    private lateinit var filterGSV: CheckBox
    private lateinit var filterOther: CheckBox

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

    // Project Tab UI
    private lateinit var newProjectButton: Button
    private lateinit var importRinexButton: Button
    private lateinit var projectsListView: ListView
    // Removed: currentProjectSection
    // Removed: currentProjectName
    // Removed: currentProjectStatus
    // Removed: exportProjectButton
    private lateinit var projectViewFlipper: ViewFlipper
    private lateinit var projectsAdapter: ArrayAdapter<String>
    private val projectDisplayList = mutableListOf<String>()

    // Project Detail (persists across tab switches)
    private var currentDetailProject: Project? = null



    // Navigation Tab UI
    private lateinit var navViewFlipper: ViewFlipper
    private lateinit var mapView: MapView
    private lateinit var pointCloudView: PointCloudGLSurfaceView
    private lateinit var btn3DView: Button
    private lateinit var btn2DView: Button
    private lateinit var btnDepthMap: Button
    private lateinit var btnHeatMap: Button
    private lateinit var btnMapLayer: Button
    private lateinit var btnOfflineMode: Button
    private lateinit var btnOfflineMap: Button
    private lateinit var navPositionText: TextView
    private lateinit var navDepthText: TextView
    private lateinit var navPointsText: TextView
    private lateinit var navClearButton: Button
    private lateinit var navExportButton: Button
    private lateinit var colorScaleMaxLabel: TextView
    private lateinit var colorScaleMinLabel: TextView
    private lateinit var colorScaleModeLabel: TextView
    private var currentPositionMarker: Marker? = null
    private var trackPolyline: Polyline? = null
    private val depthPointOverlays = mutableListOf<Polygon>()
    private var isHeatMapMode = false
    private var currentMapLayer = 0  // 0=OSM, 1=Topo, 2=Satellite

    // BLE Scan callback
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            try {
                val deviceName = device.name ?: result.scanRecord?.deviceName
                val serviceUuids = result.scanRecord?.serviceUuids
                val hasOurService = serviceUuids?.any { it.uuid == SERVICE_UUID } == true

                // Debug log: show ALL devices found during scan
                android.util.Log.e("BLE_SCAN", "Found: '$deviceName' addr=${device.address} rssi=${result.rssi} hasService=$hasOurService")

                // Device matching based on connection mode
                // Check if device has Shore Station's specific UUID
                val hasShoreService = result.scanRecord?.serviceUuids?.any {
                    it.toString().equals(SHORE_SERVICE_UUID_STR, ignoreCase = true)
                } == true

                // Radio Link mode: Shore Station (has Shore UUID OR name contains "Shore")
                // Direct mode: B-Box only (has Nordic UART but NOT Shore UUID, or name contains DeeperRTK but not Shore)
                val isShoreStation = hasShoreService || deviceName?.contains("Shore", ignoreCase = true) == true
                val isBBox = (hasOurService && !hasShoreService) ||
                             (deviceName?.contains(DEVICE_NAME, ignoreCase = true) == true && !isShoreStation)

                val matchesTarget = if (isRadioLinkMode) {
                    isShoreStation
                } else {
                    isBBox && !isShoreStation
                }

                if (matchesTarget) {
                    android.util.Log.d("DeeperRTK", "MATCH! device: $deviceName, radioLinkMode: $isRadioLinkMode")
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
                // Use correct service UUID based on connection mode
                // Both devices use the SAME characteristic UUIDs, just different service UUID
                val serviceUuid = if (isRadioLinkMode) SHORE_SERVICE_UUID else SERVICE_UUID

                android.util.Log.d("DeeperRTK", "Looking for service: $serviceUuid (radioLink=$isRadioLinkMode)")
                val service = gatt.getService(serviceUuid)
                if (service != null) {
                    // Characteristic UUIDs are the same for both B-Box and Shore Station
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
                android.util.Log.e("DeeperRTK", "BLE write failed with status $status")
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

        // Connection mode buttons
        val btnDirectMode = findViewById<Button>(R.id.btnDirectMode)
        val btnRadioMode = findViewById<Button>(R.id.btnRadioMode)
        val radioModeIndicator = findViewById<View>(R.id.radioModeIndicator)
        val radioModeStatus = findViewById<TextView>(R.id.radioModeStatus)

        btnDirectMode.setOnClickListener {
            if (isRadioLinkMode) {  // Only act if switching modes
                isRadioLinkMode = false
                btnDirectMode.isSelected = true
                btnRadioMode.isSelected = false
                // Update button colors: selected = blue, unselected = gray
                btnDirectMode.backgroundTintList = android.content.res.ColorStateList.valueOf(0xFF00BCD4.toInt())
                btnRadioMode.backgroundTintList = android.content.res.ColorStateList.valueOf(0xFF666666.toInt())
                radioModeIndicator.setBackgroundResource(R.drawable.indicator_gray)
                radioModeStatus.text = "Mode: B-BOX"
                // Disconnect and reconnect with new mode
                if (isConnected) {
                    disconnect()
                }
                handler.postDelayed({ checkPermissionsAndConnect() }, 500)
            }
        }

        btnRadioMode.setOnClickListener {
            if (!isRadioLinkMode) {  // Only act if switching modes
                isRadioLinkMode = true
                btnDirectMode.isSelected = false
                btnRadioMode.isSelected = true
                // Update button colors: selected = blue, unselected = gray
                btnRadioMode.backgroundTintList = android.content.res.ColorStateList.valueOf(0xFF00BCD4.toInt())
                btnDirectMode.backgroundTintList = android.content.res.ColorStateList.valueOf(0xFF666666.toInt())
                radioModeIndicator.setBackgroundResource(R.drawable.indicator_float)
                radioModeStatus.text = "Mode: Radio Link"
                // Disconnect and reconnect with new mode
                if (isConnected) {
                    disconnect()
                }
                handler.postDelayed({ checkPermissionsAndConnect() }, 500)
            }
        }

        // Default to direct mode
        btnDirectMode.isSelected = true
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
        shoreNtripSwitch = findViewById(R.id.shoreNtripSwitch)

        imuCalibIndicator = findViewById(R.id.imuCalibIndicator)
        imuCalibStatus = findViewById(R.id.imuCalibStatus)

        levelButton.setOnClickListener { calibrateIMU() }

        // NTRIP UI bindings
        ntripHost = findViewById(R.id.ntripHost)
        ntripPort = findViewById(R.id.ntripPort)
        ntripMountpoint = findViewById(R.id.ntripMountpoint)
        ntripUsername = findViewById(R.id.ntripUsername)
        ntripPassword = findViewById(R.id.ntripPassword)
        ntripConnectButton = findViewById(R.id.ntripConnectButton)
        ntripIndicator = findViewById(R.id.ntripIndicator)
        ntripStatus = findViewById(R.id.ntripStatus)
        ntripStats = findViewById(R.id.ntripStats)

        // Load saved NTRIP settings
        prefs = getSharedPreferences("DeeperRTK", MODE_PRIVATE)
        ntripHost.setText(prefs.getString("ntrip_host", "192.168.10.21"))
        ntripPort.setText(prefs.getString("ntrip_port", "12345"))
        ntripMountpoint.setText(prefs.getString("ntrip_mount", "LB1U02096"))
        ntripUsername.setText(prefs.getString("ntrip_user", "chris"))
        ntripPassword.setText(prefs.getString("ntrip_pass", "chris"))

        // Shore Station NTRIP toggle (only visible in Radio Link mode)
        shoreNtripSwitch.isChecked = prefs.getBoolean("shore_ntrip_enabled", false)
        shoreNtripSwitch.setOnCheckedChangeListener { _, isChecked ->
            prefs.edit().putBoolean("shore_ntrip_enabled", isChecked).apply()
            if (isConnected && isRadioLinkMode) {
                sendCommand(if (isChecked) "SHORE_NTRIP_ON" else "SHORE_NTRIP_OFF")
            }
        }

        ntripConnectButton.setOnClickListener {
            if (isNtripConnected) disconnectNtrip() else connectNtrip()
        }

        // RTCM Rate Control bindings
        hzStationEdit = findViewById(R.id.hzStation)
        hzGpsEdit = findViewById(R.id.hzGps)
        hzGloEdit = findViewById(R.id.hzGlo)
        hzGalEdit = findViewById(R.id.hzGal)
        hzBdsEdit = findViewById(R.id.hzBds)
        hzBiasEdit = findViewById(R.id.hzBias)
        cntStationText = findViewById(R.id.cntStation)
        cntGpsText = findViewById(R.id.cntGps)
        cntGloText = findViewById(R.id.cntGlo)
        cntGalText = findViewById(R.id.cntGal)
        cntBdsText = findViewById(R.id.cntBds)
        cntBiasText = findViewById(R.id.cntBias)

        // Load saved Hz values
        hzStationEdit.setText(prefs.getString("hz_station", "0.1"))
        hzGpsEdit.setText(prefs.getString("hz_gps", "1"))
        hzGloEdit.setText(prefs.getString("hz_glo", "1"))
        hzGalEdit.setText(prefs.getString("hz_gal", "1"))
        hzBdsEdit.setText(prefs.getString("hz_bds", "1"))
        hzBiasEdit.setText(prefs.getString("hz_bias", "0.2"))

        // Initialize Hz values from saved prefs
        hzStation = prefs.getString("hz_station", "0.1")?.toDoubleOrNull() ?: 0.1
        hzGps = prefs.getString("hz_gps", "1")?.toDoubleOrNull() ?: 1.0
        hzGlo = prefs.getString("hz_glo", "1")?.toDoubleOrNull() ?: 1.0
        hzGal = prefs.getString("hz_gal", "1")?.toDoubleOrNull() ?: 1.0
        hzBds = prefs.getString("hz_bds", "1")?.toDoubleOrNull() ?: 1.0
        hzBias = prefs.getString("hz_bias", "0.2")?.toDoubleOrNull() ?: 0.2

        // Add text watchers to update Hz values when changed
        setupHzTextWatcher(hzStationEdit, "hz_station") { hzStation = it }
        setupHzTextWatcher(hzGpsEdit, "hz_gps") { hzGps = it }
        setupHzTextWatcher(hzGloEdit, "hz_glo") { hzGlo = it }
        setupHzTextWatcher(hzGalEdit, "hz_gal") { hzGal = it }
        setupHzTextWatcher(hzBdsEdit, "hz_bds") { hzBds = it }
        setupHzTextWatcher(hzBiasEdit, "hz_bias") { hzBias = it }

        // NMEA Filter bindings
        filterGGA = findViewById(R.id.filterGGA)
        filterRMC = findViewById(R.id.filterRMC)
        filterVTG = findViewById(R.id.filterVTG)
        filterGSA = findViewById(R.id.filterGSA)
        filterDBT = findViewById(R.id.filterDBT)
        filterMTW = findViewById(R.id.filterMTW)
        filterGSV = findViewById(R.id.filterGSV)
        filterOther = findViewById(R.id.filterOther)

        // Load saved NMEA filter states
        filterGGA.isChecked = prefs.getBoolean("filter_gga", true)
        filterRMC.isChecked = prefs.getBoolean("filter_rmc", true)
        filterVTG.isChecked = prefs.getBoolean("filter_vtg", false)
        filterGSA.isChecked = prefs.getBoolean("filter_gsa", false)
        filterDBT.isChecked = prefs.getBoolean("filter_dbt", true)
        filterMTW.isChecked = prefs.getBoolean("filter_mtw", true)
        filterGSV.isChecked = prefs.getBoolean("filter_gsv", false)
        filterOther.isChecked = prefs.getBoolean("filter_other", false)

        // Save NMEA filter state on change
        filterGGA.setOnCheckedChangeListener { _, isChecked -> prefs.edit().putBoolean("filter_gga", isChecked).apply() }
        filterRMC.setOnCheckedChangeListener { _, isChecked -> prefs.edit().putBoolean("filter_rmc", isChecked).apply() }
        filterVTG.setOnCheckedChangeListener { _, isChecked -> prefs.edit().putBoolean("filter_vtg", isChecked).apply() }
        filterGSA.setOnCheckedChangeListener { _, isChecked -> prefs.edit().putBoolean("filter_gsa", isChecked).apply() }
        filterDBT.setOnCheckedChangeListener { _, isChecked -> prefs.edit().putBoolean("filter_dbt", isChecked).apply() }
        filterMTW.setOnCheckedChangeListener { _, isChecked -> prefs.edit().putBoolean("filter_mtw", isChecked).apply() }
        filterGSV.setOnCheckedChangeListener { _, isChecked -> prefs.edit().putBoolean("filter_gsv", isChecked).apply() }
        filterOther.setOnCheckedChangeListener { _, isChecked -> prefs.edit().putBoolean("filter_other", isChecked).apply() }

        // Set up tabs (Project tab is first)
        tabLayout.addTab(tabLayout.newTab().setText("Proj"))
        tabLayout.addTab(tabLayout.newTab().setText("Remote"))
        tabLayout.addTab(tabLayout.newTab().setText("Config"))
        tabLayout.addTab(tabLayout.newTab().setText("Logs"))
        tabLayout.addTab(tabLayout.newTab().setText("Nav"))

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

        startButton.setOnClickListener {
            val projectName = currentDetailProject?.name
            if (projectName != null && projectName.isNotEmpty()) {
                sendCommand("START:$projectName")
            } else {
                sendCommand("START")
            }
        }
        stopButton.setOnClickListener { sendCommand("STOP") }

        // Enable NMEA button - sends UDP command to Deeper sonar
        enableNmeaButton.setOnClickListener { sendNmeaEnableCommand() }

        // Initialize ProjectManager
        ProjectManager.init(this)

        // Project tab bindings
        projectViewFlipper = findViewById(R.id.projectViewFlipper)
        newProjectButton = findViewById(R.id.newProjectButton)
        importRinexButton = findViewById(R.id.importRinexButton)
        projectsListView = findViewById(R.id.projectsListView)

        // Setup projects list adapter
        projectsAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, projectDisplayList)
        projectsListView.adapter = projectsAdapter

        newProjectButton.setOnClickListener { showNewProjectDialog() }
        importRinexButton.setOnClickListener { importRinexFile() }

        // Back button - returns to project list
        findViewById<Button>(R.id.btnBackToList).setOnClickListener {
            currentDetailProject = null
            projectViewFlipper.displayedChild = 0
        }

        // Delete button
        findViewById<Button>(R.id.btnDetailDeleteProject).setOnClickListener {
            deleteCurrentProject()
        }

        // Download rover files button - uses existing BLE connection
        findViewById<Button>(R.id.btnDetailDownloadRover).setOnClickListener {
            downloadRoverFilesForProject()
        }

        projectsListView.setOnItemClickListener { _, _, position, _ ->
            val projects = ProjectManager.getProjects()
            if (position < projects.size) {
                openProject(projects[position])
            }
        }

        // Load initial project list
        refreshProjectList()

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

        // Navigation Tab bindings
        initNavigationTab()
    }

    private fun initNavigationTab() {
        // Initialize OSMdroid configuration
        Configuration.getInstance().load(applicationContext,
            PreferenceManager.getDefaultSharedPreferences(applicationContext))
        Configuration.getInstance().userAgentValue = "DeeperRTK/1.0"

        navViewFlipper = findViewById(R.id.navViewFlipper)
        mapView = findViewById(R.id.mapView)
        pointCloudView = findViewById(R.id.pointCloudView)
        btn3DView = findViewById(R.id.btn3DView)
        btn2DView = findViewById(R.id.btn2DView)
        btnDepthMap = findViewById(R.id.btnDepthMap)
        btnHeatMap = findViewById(R.id.btnHeatMap)
        btnMapLayer = findViewById(R.id.btnMapLayer)
        btnOfflineMode = findViewById(R.id.btnOfflineMode)
        btnOfflineMap = findViewById(R.id.btnOfflineMap)
        navPositionText = findViewById(R.id.navPositionText)
        navDepthText = findViewById(R.id.navDepthText)
        navPointsText = findViewById(R.id.navPointsText)
        navClearButton = findViewById(R.id.navClearButton)
        navExportButton = findViewById(R.id.navExportButton)
        colorScaleMaxLabel = findViewById(R.id.colorScaleMaxLabel)
        colorScaleMinLabel = findViewById(R.id.colorScaleMinLabel)
        colorScaleModeLabel = findViewById(R.id.colorScaleModeLabel)

        // Initialize map with Esri satellite imagery as default
        val defaultEsri = object : XYTileSource("Esri", 0, 17, 256, "",
            arrayOf("https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/")) {
            override fun getTileURLString(pMapTileIndex: Long): String {
                val z = org.osmdroid.util.MapTileIndex.getZoom(pMapTileIndex)
                val x = org.osmdroid.util.MapTileIndex.getX(pMapTileIndex)
                val y = org.osmdroid.util.MapTileIndex.getY(pMapTileIndex)
                return baseUrl + z + "/" + y + "/" + x
            }
        }
        mapView.setTileSource(defaultEsri)
        mapView.setMultiTouchControls(true)
        mapView.controller.setZoom(15.0)
        
        // Enable overzoom to allow zooming in beyond tile resolution (for 1m scale viewing)
        mapView.setMinZoomLevel(4.0)
        mapView.setMaxZoomLevel(24.0)  // Allow zoom to ~4cm scale
        mapView.isTilesScaledToDpi = true
        currentMapLayer = 2  // Start on satellite
        btnMapLayer.text = "Sat"

        // Create track polyline
        trackPolyline = Polyline().apply {
            outlinePaint.color = Color.CYAN
            outlinePaint.strokeWidth = 3f
        }
        mapView.overlays.add(trackPolyline)

        // 3D/2D view toggle
        btn3DView.setOnClickListener {
            navViewFlipper.displayedChild = 0
            btn3DView.setBackgroundColor(0xFF00bcd4.toInt())
            btn2DView.setBackgroundColor(0xFF666666.toInt())
        }
        btn2DView.setOnClickListener {
            navViewFlipper.displayedChild = 1
            btn3DView.setBackgroundColor(0xFF666666.toInt())
            btn2DView.setBackgroundColor(0xFF00bcd4.toInt())
        }

        // Depth/Heat map toggle
        btnDepthMap.setOnClickListener {
            isHeatMapMode = false
            btnDepthMap.setBackgroundColor(0xFF00bcd4.toInt())
            btnHeatMap.setBackgroundColor(0xFF666666.toInt())
            colorScaleModeLabel.text = "Depth"
            updateColorScale()
            refreshMapOverlays()
        }
        btnHeatMap.setOnClickListener {
            isHeatMapMode = true
            btnDepthMap.setBackgroundColor(0xFF666666.toInt())
            btnHeatMap.setBackgroundColor(0xFF00bcd4.toInt())
            colorScaleModeLabel.text = "Temp"
            updateColorScale()
            refreshMapOverlays()
        }

        // Map layer toggle
        btnMapLayer.setOnClickListener {
            currentMapLayer = (currentMapLayer + 1) % 3
            when (currentMapLayer) {
                0 -> {
                    mapView.setTileSource(TileSourceFactory.MAPNIK)
                    btnMapLayer.text = "OSM"
                }
                1 -> {
                    mapView.setTileSource(TileSourceFactory.OpenTopo)
                    btnMapLayer.text = "Topo"
                }
                2 -> {
                    // Esri World Imagery (max zoom 17 for reliable coverage)
                    val esri = object : XYTileSource("Esri", 0, 17, 256, "",
                        arrayOf("https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/")) {
                        override fun getTileURLString(pMapTileIndex: Long): String {
                            val z = org.osmdroid.util.MapTileIndex.getZoom(pMapTileIndex)
                            val x = org.osmdroid.util.MapTileIndex.getX(pMapTileIndex)
                            val y = org.osmdroid.util.MapTileIndex.getY(pMapTileIndex)
                            return baseUrl + z + "/" + y + "/" + x
                        }
                    }
                    mapView.setTileSource(esri)
                    btnMapLayer.text = "Sat"
                }
            }
        }

        // Clear button
        navClearButton.setOnClickListener {
            TrackManager.clear()
            trackPolyline?.setPoints(ArrayList<GeoPoint>())
            currentPositionMarker?.let { mapView.overlays.remove(it) }
            currentPositionMarker = null
            // Remove all depth point circles
            for (overlay in depthPointOverlays) {
                mapView.overlays.remove(overlay)
            }
            depthPointOverlays.clear()
            mapView.invalidate()
            pointCloudView.clearPoints()
            navPointsText.text = "Points: 0"
            navPositionText.text = "Position: ---, ---"
            navDepthText.text = "Depth: -- m"
            Toast.makeText(this, "Track cleared", Toast.LENGTH_SHORT).show()
        }

        // Save offline tiles button
        btnOfflineMap.setOnClickListener {
            Toast.makeText(this, "Downloading tiles for current area...", Toast.LENGTH_SHORT).show()
            thread {
                try {
                    val cacheManager = org.osmdroid.tileprovider.cachemanager.CacheManager(mapView)
                    val bounds = mapView.boundingBox
                    val zoomMin = mapView.zoomLevelDouble.toInt().coerceAtLeast(10)
                    val zoomMax = (zoomMin + 4).coerceAtMost(18)
                    cacheManager.downloadAreaAsync(this, bounds, zoomMin, zoomMax,
                        object : org.osmdroid.tileprovider.cachemanager.CacheManager.CacheManagerCallback {
                            override fun onTaskComplete() {
                                handler.post { Toast.makeText(this@MainActivity, "Tiles saved!", Toast.LENGTH_SHORT).show() }
                            }
                            override fun onTaskFailed(errors: Int) {
                                handler.post { Toast.makeText(this@MainActivity, "Failed: $errors errors", Toast.LENGTH_SHORT).show() }
                            }
                            override fun updateProgress(progress: Int, currentZoomLevel: Int, zMin: Int, zMax: Int) {
                                handler.post { btnOfflineMap.text = "$progress%" }
                            }
                            override fun downloadStarted() { handler.post { btnOfflineMap.text = "0%" } }
                            override fun setPossibleTilesInArea(total: Int) {}
                        })
                } catch (e: Exception) {
                    handler.post { Toast.makeText(this@MainActivity, "Error: ${e.message}", Toast.LENGTH_SHORT).show() }
                }
            }
        }

        // Export button
        navExportButton.setOnClickListener {
            val file = TrackManager.exportToCSV()
            if (file != null) {
                Toast.makeText(this, "Exported to Downloads/${file.name}", Toast.LENGTH_LONG).show()
            } else {
                Toast.makeText(this, "No points to export", Toast.LENGTH_SHORT).show()
            }
        }

        // Offline mode toggle
        btnOfflineMode.setOnClickListener {
            if (btnOfflineMode.text == "Online") {
                mapView.setUseDataConnection(false)
                btnOfflineMode.text = "Offline"
                btnOfflineMode.setBackgroundColor(0xFF2196F3.toInt())
            } else {
                mapView.setUseDataConnection(true)
                btnOfflineMode.text = "Online"
                btnOfflineMode.setBackgroundColor(0xFF4CAF50.toInt())
            }
        }

        // Start with 2D map view
        navViewFlipper.displayedChild = 1
        btn3DView.setBackgroundColor(0xFF666666.toInt())
        btn2DView.setBackgroundColor(0xFF00bcd4.toInt())

        // Listen for new depth points
        TrackManager.addListener { point ->
            handler.post {
                addPointToMap(point)
                pointCloudView.addPoint(point)
                navPointsText.text = "Points: ${TrackManager.getPointCount()}"
            }
        }
    }

    private fun updateColorScale() {
        if (isHeatMapMode) {
            val (minT, maxT) = TrackManager.getTemperatureRange()
            colorScaleMaxLabel.text = String.format("%.0f\u00B0", minT)
            colorScaleMinLabel.text = String.format("%.0f\u00B0", maxT)
        } else {
            val (minD, maxD) = TrackManager.getDepthRange()
            colorScaleMaxLabel.text = String.format("%.1fm", minD)
            colorScaleMinLabel.text = String.format("%.1fm", maxD)
        }
    }

    private fun addPointToMap(point: DepthPoint) {
        // Use IMU-corrected bottom position if available, otherwise transducer position
        val lat = if (point.bottomLat != 0.0) point.bottomLat else point.latitude
        val lon = if (point.bottomLon != 0.0) point.bottomLon else point.longitude
        val geoPoint = GeoPoint(lat, lon)
        
        // Transducer position for current marker (where the boat is)
        val transducerPoint = GeoPoint(point.latitude, point.longitude)

        if (currentPositionMarker == null) {
            currentPositionMarker = Marker(mapView).apply {
                setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER)
                title = "Current Position"
            }
            mapView.overlays.add(currentPositionMarker)
            // Center map on first point
            mapView.controller.setCenter(transducerPoint)
        }
        currentPositionMarker?.position = transducerPoint  // Marker shows boat position

        // Add to track polyline
        trackPolyline?.addPoint(geoPoint)

        // Add colored depth point circle
        val color = if (isHeatMapMode) {
            val (minT, maxT) = TrackManager.getTemperatureRange()
            val range = (maxT - minT).coerceAtLeast(1f)
            val t = ((point.temperature - minT) / range).coerceIn(0f, 1f)
            temperatureToColor(t)
        } else {
            val (minD, maxD) = TrackManager.getDepthRange()
            val range = (maxD - minD).coerceAtLeast(1f)
            val t = ((point.depth - minD) / range).coerceIn(0f, 1f)
            depthToColor(t)
        }

        val circle = Polygon(mapView).apply {
            points = Polygon.pointsAsCircle(geoPoint, 0.05)  // 5cm radius (10cm diameter)
            fillPaint.color = color
            fillPaint.style = Paint.Style.FILL
            outlinePaint.color = color
            outlinePaint.strokeWidth = 1f
        }
        depthPointOverlays.add(circle)
        mapView.overlays.add(circle)

        // Update nav display
        navPositionText.text = String.format("Pos: %.6f, %.6f", point.latitude, point.longitude)
        navDepthText.text = String.format("Depth: %.2f m", point.depth)

        mapView.invalidate()
    }

    private fun depthToColor(t: Float): Int {
        // Gradient: blue -> cyan -> green -> yellow -> red
        val r: Int
        val g: Int
        val b: Int
        when {
            t < 0.25f -> {
                val s = t / 0.25f
                r = 0; g = (s * 255).toInt(); b = 255
            }
            t < 0.5f -> {
                val s = (t - 0.25f) / 0.25f
                r = 0; g = 255; b = (255 * (1 - s)).toInt()
            }
            t < 0.75f -> {
                val s = (t - 0.5f) / 0.25f
                r = (s * 255).toInt(); g = 255; b = 0
            }
            else -> {
                val s = (t - 0.75f) / 0.25f
                r = 255; g = (255 * (1 - s)).toInt(); b = 0
            }
        }
        return Color.argb(200, r, g, b)
    }

    private fun temperatureToColor(t: Float): Int {
        // Cold (blue) to hot (red)
        val r = (t * 255).toInt()
        val b = ((1 - t) * 255).toInt()
        return Color.argb(200, r, 50, b)
    }

    private fun refreshMapOverlays() {
        // Rebuild overlays with current color mode
        mapView.invalidate()
    }

    private fun setupHzTextWatcher(editText: EditText, prefKey: String, setter: (Double) -> Unit) {
        editText.addTextChangedListener(object : android.text.TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
            override fun afterTextChanged(s: android.text.Editable?) {
                val value = s.toString().toDoubleOrNull() ?: 0.0
                setter(value)
                prefs.edit().putString(prefKey, s.toString()).apply()
            }
        })
    }

    private fun updateRtcmCounters() {
        runOnUiThread {
            cntStationText.text = cntStation.toString()
            cntGpsText.text = cntGps.toString()
            cntGloText.text = cntGlo.toString()
            cntGalText.text = cntGal.toString()
            cntBdsText.text = cntBds.toString()
            cntBiasText.text = cntBias.toString()
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
        connectionStatus.text = "Scanning for B-Box..."
        android.util.Log.e("BLE_SCAN", "=== Starting BLE scan, radioLinkMode=$isRadioLinkMode ===")
        Toast.makeText(this, "Starting BLE scan...", Toast.LENGTH_SHORT).show()

        try {
            val scanSettings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()
            bluetoothLeScanner?.startScan(null, scanSettings, scanCallback)
            isScanning = true
            android.util.Log.d("BLE_SCAN", "Scan started successfully")

            handler.postDelayed({
                if (isScanning && !isConnected) {
                    android.util.Log.d("BLE_SCAN", "=== Scan timeout - device not found ===")
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
        connectionStatus.text = if (isRadioLinkMode) "Connected to Shore Station" else "Connected to B-BOX"
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
        synchronized(bleLock) {
            try {
                // Wait for minimum interval between writes to prevent BLE congestion
                val now = System.currentTimeMillis()
                val elapsed = now - lastBleWriteTime
                if (elapsed < BLE_WRITE_INTERVAL_MS) {
                    Thread.sleep(BLE_WRITE_INTERVAL_MS - elapsed)
                }
                rxCharacteristic!!.value = ("$command\n").toByteArray()
                bluetoothGatt!!.writeCharacteristic(rxCharacteristic)
                lastBleWriteTime = System.currentTimeMillis()
            } catch (e: SecurityException) { }
            catch (e: InterruptedException) { }
        }
    }

    private fun processReceivedData(data: String) {
        android.util.Log.d("DeeperRTK", "RX raw: $data")
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
            line == "OK:DELETE_ALL" || line == "ERR:NO_FILES" || line == "ERR:RADIO_LINK") {
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

        // Add point to TrackManager for navigation tab
        // Filter out (0,0) = Africa (no GPS fix), require 4+ sats and valid ranges
        val validGps = lat != 0.0 && lon != 0.0 && 
                       lat > -90 && lat < 90 && 
                       lon > -180 && lon < 180 &&
                       sats >= 4
        if (validGps && depth > 0) {
            val depthPoint = DepthPoint(
                timestamp = System.currentTimeMillis(),
                latitude = lat,
                longitude = lon,
                depth = depth,
                temperature = temp,
                pitch = pitch,
                roll = roll,
                fix = fix,
                satellites = sats
            )
            TrackManager.addPoint(depthPoint)
        }
    }


    // Logs Tab Functions
    private fun requestLogsList() {
        android.util.Log.d("DeeperRTK", "requestLogsList() called, isConnected=$isConnected")
        if (!isConnected) {
            Toast.makeText(this, "Not connected", Toast.LENGTH_SHORT).show()
            return
        }
        logsStatus.text = "Requesting logs..."
        logFiles.clear()
        logsAdapter.clear()
        android.util.Log.d("DeeperRTK", "Sending LIST_LOGS command")
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

        val dialog = AlertDialog.Builder(this, R.style.AlertDialogTheme)
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
                val logsData = line.substring(5)
                // Check if this is for project download
                if (projectDownloadMode) {
                    handleProjectFileList(logsData)
                    return
                }
                // Normal Logs tab handling
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
                if (projectDownloadMode) {
                    saveProjectFile()
                } else {
                    saveDownloadedFile()
                }
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
            line == "ERR:RADIO_LINK" -> {
                logsStatus.text = "Not available via radio link"
                Toast.makeText(this, "Log management requires direct B-Box connection", Toast.LENGTH_LONG).show()
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

    override fun onResume() {
        super.onResume()
        if (::mapView.isInitialized) {
            mapView.onResume()
        }
        // Refresh project list in case it changed (e.g., returning from detail screen)
        if (::projectsAdapter.isInitialized) {
            refreshProjectList()
        }
    }

    override fun onPause() {
        super.onPause()
        if (::mapView.isInitialized) {
            mapView.onPause()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        handler.removeCallbacks(pollRunnable)
        disconnect()
    }

    // ========== PROJECT MANAGEMENT ==========

    private fun refreshProjectList() {
        projectDisplayList.clear()
        for (project in ProjectManager.getProjects()) {
            val dateStr = java.text.SimpleDateFormat("MM/dd HH:mm", java.util.Locale.US)
                .format(java.util.Date(project.createdAt))
            projectDisplayList.add("${project.name} - ${project.getStatusText()} ($dateStr)")
        }
        projectsAdapter.notifyDataSetChanged()
    }

    private fun showNewProjectDialog() {
        val input = EditText(this).apply {
            hint = "Project name"
            setPadding(48, 32, 48, 32)
        }

        AlertDialog.Builder(this, R.style.AlertDialogTheme)
            .setTitle("New Project")
            .setMessage("Enter a name for the new project:")
            .setView(input)
            .setPositiveButton("Create") { _, _ ->
                val name = input.text.toString().trim()
                if (name.isNotEmpty()) {
                    val project = ProjectManager.createProject(name)
                    refreshProjectList()
                    Toast.makeText(this, "Project created: $name", Toast.LENGTH_SHORT).show()
                    // Open the new project in detail view
                    openProject(project)
                } else {
                    Toast.makeText(this, "Please enter a project name", Toast.LENGTH_SHORT).show()
                }
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun openProject(project: Project) {
        // Show project detail view (persists across tab switches)
        currentDetailProject = project

        // Update detail view content
        findViewById<TextView>(R.id.tvDetailProjectName).text = project.name
        findViewById<TextView>(R.id.tvDetailStatus).text = project.getStatusText()

        val dateFormat = java.text.SimpleDateFormat("yyyy-MM-dd HH:mm", java.util.Locale.US)
        findViewById<TextView>(R.id.tvDetailCreated).text = dateFormat.format(java.util.Date(project.createdAt))

        // Update file status indicators
        val ubxStatus = findViewById<TextView>(R.id.tvDetailUbxStatus)
        val csvStatus = findViewById<TextView>(R.id.tvDetailCsvStatus)
        val ubxIndicator = findViewById<View>(R.id.indicatorDetailUbx)
        val csvIndicator = findViewById<View>(R.id.indicatorDetailCsv)

        if (project.roverUbxFile != null) {
            findViewById<TextView>(R.id.tvDetailUbxFile).text = project.roverUbxFile
            ubxStatus.text = "Downloaded"
            ubxStatus.setTextColor(0xFF4CAF50.toInt())
            ubxIndicator.setBackgroundColor(0xFF4CAF50.toInt())
        } else {
            findViewById<TextView>(R.id.tvDetailUbxFile).text = "raw_XXXX.ubx (PPK data)"
            ubxStatus.text = "Not downloaded"
            ubxStatus.setTextColor(0xFF888888.toInt())
            ubxIndicator.setBackgroundColor(0xFF888888.toInt())
        }

        if (project.roverCsvFile != null) {
            findViewById<TextView>(R.id.tvDetailCsvFile).text = project.roverCsvFile
            csvStatus.text = "Downloaded"
            csvStatus.setTextColor(0xFF4CAF50.toInt())
            csvIndicator.setBackgroundColor(0xFF4CAF50.toInt())
        } else {
            findViewById<TextView>(R.id.tvDetailCsvFile).text = "log_XXXX.csv (depth/IMU)"
            csvStatus.text = "Not downloaded"
            csvStatus.setTextColor(0xFF888888.toInt())
            csvIndicator.setBackgroundColor(0xFF888888.toInt())
        }

        // Switch to detail view
        projectViewFlipper.displayedChild = 1
    }

    private fun deleteCurrentProject() {
        val project = currentDetailProject ?: return

        AlertDialog.Builder(this, R.style.AlertDialogTheme)
            .setTitle("Delete Project")
            .setMessage("Delete '" + project.name + "'? This removes all project files.")
            .setPositiveButton("Delete") { _, _ ->
                ProjectManager.deleteProject(project.id)
                currentDetailProject = null
                projectViewFlipper.displayedChild = 0
                refreshProjectList()
                Toast.makeText(this, "Project deleted", Toast.LENGTH_SHORT).show()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    // Download rover files using existing BLE connection
    private var projectDownloadMode = false
    private var projectFilesToDownload = mutableListOf<String>()
    private var projectTotalFiles = 0
    private var projectCurrentFileIndex = 0
    private var projectAvailableFiles = mutableListOf<Pair<String, Long>>()  // filename, size

    private fun downloadRoverFilesForProject() {
        val project = currentDetailProject
        if (project == null) {
            Toast.makeText(this, "No project selected", Toast.LENGTH_SHORT).show()
            return
        }
        if (!isConnected) {
            Toast.makeText(this, "Connect to B-Box first (use Remote tab)", Toast.LENGTH_SHORT).show()
            return
        }
        // Set project download mode and request file list
        projectDownloadMode = true
        projectFilesToDownload.clear()
        Toast.makeText(this, "Requesting file list...", Toast.LENGTH_SHORT).show()
        sendCommand("LIST_LOGS")
    }

    private fun handleProjectFileList(logsData: String) {
        if (logsData.isEmpty() || logsData == "EMPTY") {
            Toast.makeText(this, "No files on device", Toast.LENGTH_SHORT).show()
            projectDownloadMode = false
            hideDownloadProgress()
            return
        }

        // Parse file list to find .csv and .ubx pairs with sizes
        val entries = logsData.split(";")
        projectAvailableFiles.clear()
        val csvFiles = mutableListOf<Pair<String, Long>>()
        val ubxFiles = mutableListOf<Pair<String, Long>>()

        for (entry in entries) {
            val parts = entry.split(",")
            if (parts.size >= 2) {
                val name = parts[0]
                val size = parts[1].toLongOrNull() ?: 0L
                if (name.endsWith(".csv")) csvFiles.add(Pair(name, size))
                if (name.endsWith(".ubx")) ubxFiles.add(Pair(name, size))
            }
        }

        if (csvFiles.isEmpty() && ubxFiles.isEmpty()) {
            Toast.makeText(this, "No log files found", Toast.LENGTH_SHORT).show()
            projectDownloadMode = false
            hideDownloadProgress()
            return
        }

        // Build list of file pairs (csv + matching ubx)
        // Supports both legacy (log_0001.csv/raw_0001.ubx) and project naming (name.csv/name.ubx)
        val filePairs = mutableListOf<Triple<String, String?, Long>>()  // csv, ubx or null, total size
        for ((csv, csvSize) in csvFiles) {
            // Get base name without extension and optional log_ prefix
            var baseName = csv.substringBeforeLast(".")
            if (baseName.startsWith("log_")) baseName = baseName.removePrefix("log_")

            // Look for matching UBX with same base name (either raw_X.ubx or X.ubx)
            val matchingUbx = ubxFiles.find {
                val ubxBase = it.first.substringBeforeLast(".")
                ubxBase == baseName || ubxBase == "raw_$baseName" || ubxBase.removePrefix("raw_") == baseName
            }
            val totalSize = csvSize + (matchingUbx?.second ?: 0L)
            filePairs.add(Triple(csv, matchingUbx?.first, totalSize))
        }

        if (filePairs.isEmpty()) {
            Toast.makeText(this, "No matching file pairs found", Toast.LENGTH_SHORT).show()
            projectDownloadMode = false
            hideDownloadProgress()
            return
        }

        // Show file selection dialog
        showFileSelectionDialog(filePairs, ubxFiles)
    }

    private fun showFileSelectionDialog(filePairs: List<Triple<String, String?, Long>>, ubxFiles: List<Pair<String, Long>>) {
        val project = currentDetailProject ?: return

        // Build display list
        val displayItems = filePairs.map { (csv, ubx, size) ->
            val sizeStr = formatFileSize(size)
            if (ubx != null) {
                "$csv + $ubx ($sizeStr)"
            } else {
                "$csv ($sizeStr)"
            }
        }.toTypedArray()

        val checkedItems = BooleanArray(displayItems.size) { true }  // All selected by default

        AlertDialog.Builder(this, R.style.AlertDialogTheme)
            .setTitle("Select Files to Download")
            .setMultiChoiceItems(displayItems, checkedItems) { _, which, isChecked ->
                checkedItems[which] = isChecked
            }
            .setPositiveButton("Download") { _, _ ->
                projectFilesToDownload.clear()
                for (i in filePairs.indices) {
                    if (checkedItems[i]) {
                        val (csv, ubx, _) = filePairs[i]
                        projectFilesToDownload.add(csv)
                        if (ubx != null) {
                            projectFilesToDownload.add(ubx)
                        }
                    }
                }

                if (projectFilesToDownload.isEmpty()) {
                    Toast.makeText(this, "No files selected", Toast.LENGTH_SHORT).show()
                    projectDownloadMode = false
                    return@setPositiveButton
                }

                projectTotalFiles = projectFilesToDownload.size
                projectCurrentFileIndex = 0
                showDownloadProgress()
                updateDownloadProgress()
                downloadNextProjectFile()
            }
            .setNegativeButton("Cancel") { _, _ ->
                projectDownloadMode = false
            }
            .show()
    }

    private fun showDownloadProgress() {
        findViewById<View>(R.id.layoutDetailRoverProgress)?.visibility = View.VISIBLE
        findViewById<Button>(R.id.btnDetailDownloadRover)?.isEnabled = false
    }

    private fun hideDownloadProgress() {
        findViewById<View>(R.id.layoutDetailRoverProgress)?.visibility = View.GONE
        findViewById<Button>(R.id.btnDetailDownloadRover)?.isEnabled = true
    }

    private fun updateDownloadProgress() {
        val progress = if (projectTotalFiles > 0) {
            (projectCurrentFileIndex * 100) / projectTotalFiles
        } else 0

        findViewById<android.widget.ProgressBar>(R.id.progressDetailRover)?.progress = progress

        val statusText = if (projectCurrentFileIndex < projectTotalFiles) {
            "Downloading file ${projectCurrentFileIndex + 1} of $projectTotalFiles..."
        } else {
            "Complete!"
        }
        findViewById<TextView>(R.id.tvDetailRoverProgress)?.text = statusText
    }

    private fun downloadNextProjectFile() {
        if (projectFilesToDownload.isEmpty()) {
            projectDownloadMode = false
            projectCurrentFileIndex = projectTotalFiles
            updateDownloadProgress()
            hideDownloadProgress()
            Toast.makeText(this, "Download complete!", Toast.LENGTH_SHORT).show()
            // Refresh project detail view
            currentDetailProject?.let { openProject(it) }
            return
        }

        val filename = projectFilesToDownload.removeAt(0)
        downloadFileName = filename
        downloadBuffer.clear()
        updateDownloadProgress()
        sendCommand("DOWNLOAD:$filename")
    }

    private fun saveProjectFile() {
        val project = currentDetailProject ?: return
        try {
            val projectDir = ProjectManager.getRoverDir(project.id).parentFile!!
            val roverDir = java.io.File(projectDir, "rover")
            if (!roverDir.exists()) roverDir.mkdirs()

            // Rename file to match project name
            val newFileName = if (downloadFileName.endsWith(".csv")) {
                "${project.name}.csv"
            } else if (downloadFileName.endsWith(".ubx")) {
                "${project.name}.ubx"
            } else {
                downloadFileName
            }

            val file = java.io.File(roverDir, newFileName)
            java.io.FileOutputStream(file).use { fos ->
                fos.write(downloadBuffer.toString().toByteArray())
            }

            // Update project with renamed file reference
            if (newFileName.endsWith(".csv")) {
                project.roverCsvFile = newFileName
            } else if (newFileName.endsWith(".ubx")) {
                project.roverUbxFile = newFileName
            }
            ProjectManager.updateProject(project)

            // Increment progress and download next file
            projectCurrentFileIndex++
            downloadNextProjectFile()
        } catch (e: Exception) {
            Toast.makeText(this, "Save failed: ${e.message}", Toast.LENGTH_SHORT).show()
            projectDownloadMode = false
            hideDownloadProgress()
        }
    }

    private fun importRinexFile() {
        val current = ProjectManager.getCurrentProject()
        if (current == null) {
            Toast.makeText(this, "Create or open a project first", Toast.LENGTH_SHORT).show()
            return
        }

        val intent = Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE)
            type = "*/*"
            putExtra(Intent.EXTRA_MIME_TYPES, arrayOf(
                "application/octet-stream",
                "text/plain"
            ))
        }
        startActivityForResult(intent, REQUEST_IMPORT_RINEX)
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == REQUEST_IMPORT_RINEX && resultCode == Activity.RESULT_OK) {
            data?.data?.let { uri ->
                importRinexFromUri(uri)
            }
        }
    }

    private fun importRinexFromUri(uri: Uri) {
        val current = ProjectManager.getCurrentProject() ?: return

        thread {
            try {
                val filename = getFileName(uri) ?: "base.obs"
                val inputStream = contentResolver.openInputStream(uri)
                val bytes = inputStream?.readBytes() ?: return@thread
                inputStream.close()

                ProjectManager.saveBaseRinexFile(current.id, filename, bytes)

                handler.post {
                    refreshProjectList()
                    Toast.makeText(this, "Imported: $filename", Toast.LENGTH_SHORT).show()
                }
            } catch (e: Exception) {
                handler.post {
                    Toast.makeText(this, "Import failed: ${e.message}", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }

    private fun getFileName(uri: Uri): String? {
        var result: String? = null
        if (uri.scheme == "content") {
            val cursor = contentResolver.query(uri, null, null, null, null)
            cursor?.use {
                if (it.moveToFirst()) {
                    val idx = it.getColumnIndex(OpenableColumns.DISPLAY_NAME)
                    if (idx >= 0) result = it.getString(idx)
                }
            }
        }
        if (result == null) {
            result = uri.path
            val cut = result?.lastIndexOf('/')
            if (cut != null && cut != -1) {
                result = result?.substring(cut + 1)
            }
        }
        return result
    }

    private fun exportCurrentProject() {
        val current = ProjectManager.getCurrentProject()
        if (current == null) {
            Toast.makeText(this, "No project open", Toast.LENGTH_SHORT).show()
            return
        }

        thread {
            val zipFile = ProjectManager.exportProjectZip(current.id)
            handler.post {
                if (zipFile != null) {
                    Toast.makeText(this, "Exported to: ${zipFile.name}", Toast.LENGTH_LONG).show()
                } else {
                    Toast.makeText(this, "Export failed", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }

    // ========== NTRIP CLIENT ==========
    private fun connectNtrip() {
        val host = ntripHost.text.toString().trim()
        val port = ntripPort.text.toString().toIntOrNull() ?: 12345
        val mountpoint = ntripMountpoint.text.toString().trim()
        val username = ntripUsername.text.toString()
        val password = ntripPassword.text.toString()

        // Save settings
        prefs.edit()
            .putString("ntrip_host", host)
            .putString("ntrip_port", port.toString())
            .putString("ntrip_mount", mountpoint)
            .putString("ntrip_user", username)
            .putString("ntrip_pass", password)
            .apply()

        if (host.isEmpty() || mountpoint.isEmpty()) {
            Toast.makeText(this, "Enter host and mountpoint", Toast.LENGTH_SHORT).show()
            return
        }

        ntripConnectButton.isEnabled = false
        ntripStatus.text = "Connecting..."
        ntripIndicator.setBackgroundResource(R.drawable.indicator_gps)

        ntripThread = Thread {
            try {
                android.util.Log.d("NTRIP", "Connecting to $host:$port...")
                ntripSocket = java.net.Socket(host, port)
                android.util.Log.d("NTRIP", "Socket connected")
                ntripSocket!!.soTimeout = 30000

                val output = ntripSocket!!.getOutputStream()
                val input = java.io.BufferedInputStream(ntripSocket!!.getInputStream())

                // Send NTRIP request
                val auth = android.util.Base64.encodeToString(
                    "$username:$password".toByteArray(), android.util.Base64.NO_WRAP
                )
                val request = "GET /$mountpoint HTTP/1.1\r\n" +
                    "Host: $host\r\n" +
                    "Ntrip-Version: Ntrip/2.0\r\n" +
                    "User-Agent: DeeperRTK/1.0\r\n" +
                    "Authorization: Basic $auth\r\n" +
                    "\r\n"
                output.write(request.toByteArray())
                output.flush()
                android.util.Log.d("NTRIP", "Request sent: GET /$mountpoint")

                // Read response header
                val headerBuffer = StringBuilder()
                var b: Int
                while (input.read().also { b = it } != -1) {
                    headerBuffer.append(b.toChar())
                    // Break on double CRLF or single CRLF after 200 OK
                    if (headerBuffer.endsWith("\r\n\r\n")) break
                    if (headerBuffer.contains("200 OK") && headerBuffer.endsWith("\r\n")) break
                    if (headerBuffer.contains("ICY 200") && headerBuffer.endsWith("\r\n")) break
                    if (headerBuffer.length > 4096) break
                }
                val header = headerBuffer.toString()
                android.util.Log.d("NTRIP", "Response header: ${header.take(200)}")

                if (!header.contains("200 OK") && !header.contains("ICY 200 OK")) {
                    val errorMsg = if (header.contains("401")) "Auth failed"
                                   else if (header.contains("404")) "Mountpoint not found"
                                   else "Connection rejected"
                    handler.post {
                        ntripStatus.text = errorMsg
                        ntripIndicator.setBackgroundResource(R.drawable.indicator_red)
                        ntripConnectButton.isEnabled = true
                    }
                    ntripSocket?.close()
                    return@Thread
                }

                isNtripConnected = true
                handler.post {
                    ntripStatus.text = "Connected"
                    ntripIndicator.setBackgroundResource(R.drawable.indicator_green)
                    ntripConnectButton.text = "Disconnect"
                    ntripConnectButton.isEnabled = true
                }

                // Read RTCM data
                val buffer = ByteArray(4096)
                val rtcmBuffer = mutableListOf<Byte>()

                while (isNtripConnected && !Thread.currentThread().isInterrupted) {
                    val len = input.read(buffer)
                    if (len <= 0) break

                    for (i in 0 until len) {
                        rtcmBuffer.add(buffer[i])
                    }

                    // Process complete RTCM messages
                    processRtcmBuffer(rtcmBuffer)

                    // Update stats and counters
                    handler.post {
                        ntripStats.text = "RTCM: $rtcmMessagesReceived msgs, ${rtcmBytesSent/1024}KB sent"
                        updateRtcmCounters()
                    }
                }
            } catch (e: Exception) {
                android.util.Log.e("NTRIP", "Connection error: ${e.javaClass.simpleName}: ${e.message}")
                val wasConnected = isNtripConnected
                handler.post {
                    if (!wasConnected) {
                        ntripStatus.text = e.message ?: "Connection failed"
                    } else {
                        ntripStatus.text = "Disconnected"
                    }
                    ntripIndicator.setBackgroundResource(R.drawable.indicator_gray)
                    ntripConnectButton.text = "Connect"
                    ntripConnectButton.isEnabled = true
                }
            } finally {
                isNtripConnected = false
                try { ntripSocket?.close() } catch (ex: Exception) {}
                ntripSocket = null
            }
        }
        ntripThread?.start()
    }

    private fun disconnectNtrip() {
        isNtripConnected = false
        try { ntripSocket?.close() } catch (e: Exception) {}
        ntripSocket = null
        ntripThread?.interrupt()
        ntripThread = null

        ntripStatus.text = "Disconnected"
        ntripIndicator.setBackgroundResource(R.drawable.indicator_gray)
        ntripConnectButton.text = "Connect"
        ntripConnectButton.isEnabled = true
    }

    private fun processRtcmBuffer(buffer: MutableList<Byte>) {
        while (buffer.size >= 6) {
            // Look for RTCM preamble
            if (buffer[0] != 0xD3.toByte()) {
                buffer.removeAt(0)
                continue
            }

            // Get message length
            val len = ((buffer[1].toInt() and 0x03) shl 8) or (buffer[2].toInt() and 0xFF)
            val totalLen = len + 6  // header(3) + data + CRC(3)

            if (buffer.size < totalLen) break  // Need more data

            // Extract message
            val message = buffer.subList(0, totalLen).toByteArray()
            repeat(totalLen) { buffer.removeAt(0) }

            // Get message type
            val msgType = ((message[3].toInt() and 0xFF) shl 4) or ((message[4].toInt() and 0xF0) shr 4)

            rtcmMessagesReceived++

            // Update counters and check rate limits
            val shouldSend = shouldSendMessage(msgType)
            android.util.Log.d("NTRIP", "RTCM type $msgType, shouldSend=$shouldSend")
            if (shouldSend) {
                sendRtcmToDevice(message)
            }
        }
    }

    private fun shouldSendMessage(msgType: Int): Boolean {
        val now = System.currentTimeMillis()

        return when (msgType) {
            1005, 1006 -> {
                val interval = if (hzStation > 0) (1000.0 / hzStation).toLong() else Long.MAX_VALUE
                if (now - lastStationTime >= interval) {
                    lastStationTime = now
                    cntStation++
                    true
                } else false
            }
            in 1071..1077 -> {
                val interval = if (hzGps > 0) (1000.0 / hzGps).toLong() else Long.MAX_VALUE
                if (now - lastGpsTime >= interval) {
                    lastGpsTime = now
                    cntGps++
                    true
                } else false
            }
            in 1081..1087 -> {
                val interval = if (hzGlo > 0) (1000.0 / hzGlo).toLong() else Long.MAX_VALUE
                if (now - lastGloTime >= interval) {
                    lastGloTime = now
                    cntGlo++
                    true
                } else false
            }
            in 1091..1097 -> {
                val interval = if (hzGal > 0) (1000.0 / hzGal).toLong() else Long.MAX_VALUE
                if (now - lastGalTime >= interval) {
                    lastGalTime = now
                    cntGal++
                    true
                } else false
            }
            in 1121..1127 -> {
                val interval = if (hzBds > 0) (1000.0 / hzBds).toLong() else Long.MAX_VALUE
                if (now - lastBdsTime >= interval) {
                    lastBdsTime = now
                    cntBds++
                    true
                } else false
            }
            1230 -> {
                val interval = if (hzBias > 0) (1000.0 / hzBias).toLong() else Long.MAX_VALUE
                if (now - lastBiasTime >= interval) {
                    lastBiasTime = now
                    cntBias++
                    true
                } else false
            }
            else -> false  // Skip unknown message types
        }
    }

    private fun sendRtcmToDevice(rtcmData: ByteArray) {
        if (!isConnected) {
            android.util.Log.w("NTRIP", "sendRtcmToDevice: not connected")
            return
        }
        if (rxCharacteristic == null) {
            android.util.Log.w("NTRIP", "sendRtcmToDevice: rxCharacteristic is null")
            return
        }
        if (bluetoothGatt == null) {
            android.util.Log.w("NTRIP", "sendRtcmToDevice: bluetoothGatt is null")
            return
        }

        synchronized(bleLock) {
            try {
                android.util.Log.d("NTRIP", "Sending ${rtcmData.size} bytes RTCM via BLE")
                // BLE MTU limits chunk size - send in 200-byte chunks with "RTCM:" prefix
                val maxChunk = 200
                var offset = 0
                while (offset < rtcmData.size) {
                    val chunkLen = minOf(maxChunk, rtcmData.size - offset)
                    val chunk = rtcmData.copyOfRange(offset, offset + chunkLen)

                    val cmd = ByteArray(5 + chunkLen)
                    "RTCM:".toByteArray().copyInto(cmd, 0)
                    chunk.copyInto(cmd, 5)

                    // Wait for minimum interval between writes to prevent BLE congestion
                    val now = System.currentTimeMillis()
                    val elapsed = now - lastBleWriteTime
                    if (elapsed < BLE_WRITE_INTERVAL_MS) {
                        Thread.sleep(BLE_WRITE_INTERVAL_MS - elapsed)
                    }

                    rxCharacteristic!!.value = cmd
                    bluetoothGatt!!.writeCharacteristic(rxCharacteristic)
                    lastBleWriteTime = System.currentTimeMillis()
                    rtcmBytesSent += chunkLen

                    offset += chunkLen
                }
            } catch (e: Exception) {
                android.util.Log.e("NTRIP", "RTCM send error: ${e.message}")
            }
        }
    }

}