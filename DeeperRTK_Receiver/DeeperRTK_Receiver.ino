/*
 * TTGO LoRa32 V2.0 - Deeper Sonar + RTK Receiver
 * Combines RTK corrections via LoRa with Deeper Chirp+2 sonar data
 * Logs georeferenced depth data to SD card for PPK processing
 *
 * Hardware: TTGO LoRa32 V2.0 (ESP32 + SX1276 + OLED)
 *
 * Connections:
 *   TTGO IO32 (TX) -> M8P RX  - RTCM corrections IN
 *   TTGO IO33 (RX) <- M8P TX  - NMEA OUT (RTK position)
 *   Built-in SD card slot (SD_MMC 1-bit mode)
 *
 * WiFi: Connects to Deeper Chirp+2 AP for sonar data
 * LoRa: Receives RTK corrections from base station
 * BLE: Nordic UART Service for Android app control
 * IMU: ADXL345 accelerometer for tilt sensing (shared I2C with OLED)
 *
 * Built-in SD Card Pins (DO NOT USE FOR OTHER PURPOSES):
 *   GPIO 14 = SD_CLK
 *   GPIO 15 = SD_CMD
 *   GPIO 2  = SD_DATA0
 *
 * Note: LoRa RST is hardwired to ESP32 EN pin (not GPIO 14)
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SD_MMC.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <Preferences.h>

Preferences prefs;

// ========== BLE CONFIGURATION ==========
// Nordic UART Service UUIDs
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer *pServer = NULL;
NimBLECharacteristic *pTxCharacteristic = NULL;
bool bleConnected = false;
bool oldBleConnected = false;
uint32_t bleConnectTime = 0;    // When client connected
String bleRxBuffer = "";

// ========== ADXL345 IMU CONFIGURATION ==========
#define ADXL345_ADDR     0x53    // I2C address (SDO to GND)
#define ADXL345_DEVID    0x00    // Device ID register
#define ADXL345_POWER_CTL 0x2D   // Power control register
#define ADXL345_DATA_FORMAT 0x31 // Data format register
#define ADXL345_DATAX0   0x32    // X-axis data register
#define ADXL345_BW_RATE  0x2C    // Data rate register
#define IMU_UPDATE_INTERVAL 50   // Update IMU every 50ms (20 Hz)

// Global IMU calibration offsets (can be modified by IMU_LEVEL command)
int16_t imuXOffset = -49;
int16_t imuYOffset = -9;
int16_t imuZOffset = 1453;

// IMU smoothing filter (exponential moving average)
// Lower alpha = more smoothing but more lag
// 0.1 = very smooth, 0.3 = moderate, 0.5 = responsive
#define IMU_FILTER_ALPHA 0.25f
float filteredPitch = 0.0f;
float filteredRoll = 0.0f;
bool imuFilterInitialized = false;

// ========== DEEPER CONFIGURATION ==========
#define DEEPER_SSID      "Deeper CHIRP+2 3CC6"
#define DEEPER_PASSWORD  ""                     // Open network (no password)
#define DEEPER_IP        "192.168.10.1"
#define DEEPER_UDP_PORT  10110
#define DEEPER_ENABLE_CMD "$DEEP230,1*38\r\n"
#define DEEPER_DISABLE_CMD "$DEEP230,0*39\r\n"

// Frequency control commands (optional)
#define DEEPER_WIDE_CMD   "$DEEP231,4*3C\r\n"   // 90-115 kHz
#define DEEPER_MEDIUM_CMD "$DEEP231,5*3D\r\n"   // 270-310 kHz
#define DEEPER_NARROW_CMD "$DEEP231,6*3E\r\n"   // 635-715 kHz

// ========== PIN DEFINITIONS (TTGO LoRa32 V2.0) ==========
// LoRa SX1276
#define LORA_SCK     5
#define LORA_MISO    19
#define LORA_MOSI    27
#define LORA_CS      18
#define LORA_RST     -1   // Hardwired to EN pin, not GPIO (per pinout diagram)
#define LORA_DIO0    26

// Built-in SD Card uses SD_MMC (1-bit mode), pins are fixed by hardware:
// GPIO 14 = SD_CLK, GPIO 15 = SD_CMD, GPIO 2 = SD_DATA0

// OLED Display
#define OLED_SDA     21
#define OLED_SCL     22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Here+ GPS UART (Serial1)
#define GPS_TX_PIN   32   // TTGO TX -> M8P RX - RTCM in
#define GPS_RX_PIN   33   // TTGO RX <- M8P TX - NMEA out
#define GPS_BAUD     115200

// ========== LORA CONFIGURATION (MUST MATCH TRANSMITTER) ==========
#define LORA_FREQUENCY     915E6
#define LORA_BANDWIDTH     125E3
#define LORA_SPREAD_FACTOR 7        // SF7 - DO NOT CHANGE
#define LORA_CODING_RATE   5        // 4/5 - DO NOT CHANGE
#define LORA_SYNC_WORD     0x12
#define LORA_TX_POWER      20

// ========== BUFFER SIZES ==========
#define RTCM_BUFFER_SIZE   1024
#define NMEA_BUFFER_SIZE   256
#define UDP_BUFFER_SIZE    512
#define FRAGMENT_TIMEOUT   500

// ========== LOGGING CONFIGURATION ==========
#define LOG_INTERVAL_MS    50       // Log at 20 Hz (matches IMU rate, captures 14 Hz depth)
#define FLUSH_INTERVAL_MS  5000     // Flush SD card every 5 seconds

// ========== UBX RAW LOGGING FOR PPK ==========
// UBX protocol constants
#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62
#define UBX_CLASS_RXM   0x02
#define UBX_ID_RAWX     0x15        // Raw measurements (pseudorange, carrier phase)
#define UBX_ID_SFRBX    0x13        // Navigation subframes
#define UBX_BUFFER_SIZE 1200        // Max size for 32 satellites

// UBX parser states
enum UBXState {
  UBX_WAIT_SYNC1,
  UBX_WAIT_SYNC2,
  UBX_WAIT_CLASS,
  UBX_WAIT_ID,
  UBX_WAIT_LEN1,
  UBX_WAIT_LEN2,
  UBX_COLLECTING,
  UBX_WAIT_CKA,
  UBX_WAIT_CKB
};

// ========== OBJECTS ==========
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
HardwareSerial GPSSerial(1);
WiFiUDP udp;
File logFile;
File rawFile;                       // UBX raw data for PPK

// UBX parser state
uint8_t ubxBuffer[UBX_BUFFER_SIZE];
uint16_t ubxIndex = 0;
uint16_t ubxPayloadLen = 0;
uint8_t ubxClass = 0;
uint8_t ubxId = 0;
uint8_t ubxCkA = 0;
uint8_t ubxCkB = 0;
UBXState ubxState = UBX_WAIT_SYNC1;
bool ppkLoggingEnabled = true;      // PPK raw logging enabled by default
uint32_t ubxMessagesLogged = 0;
uint32_t ubxBytesWritten = 0;
char rawFileName[32];

// ========== DEEPER/SONAR DATA ==========
struct SonarData {
  float depthMeters;
  float depthFeet;
  float waterTempC;
  int8_t batteryPercent;  // -1 = unknown
  uint32_t lastUpdate;
  // UTC time from GGA for PPK sync
  uint8_t utcHours;
  uint8_t utcMinutes;
  uint8_t utcSeconds;
  uint16_t utcMillis;     // fractional seconds (0-999)
  bool timeValid;
  bool valid;
} sonar = {0, 0, 0, -1, 0, 0, 0, 0, 0, false, false};

// ========== IMU DATA ==========
struct IMUData {
  float accelX;       // Raw acceleration X (g)
  float accelY;       // Raw acceleration Y (g)
  float accelZ;       // Raw acceleration Z (g)
  float pitch;        // Pitch angle (degrees) - rotation around X axis
  float roll;         // Roll angle (degrees) - rotation around Y axis
  uint32_t lastUpdate;
  bool valid;
} imu = {0, 0, 0, 0, 0, 0, false};

bool imuAvailable = false;
uint32_t lastImuUpdate = 0;

// ========== WIFI STATE ==========
enum WiFiState {
  WIFI_DISCONNECTED,
  WIFI_CONNECTING,
  WIFI_CONNECTED,
  WIFI_UDP_READY
};
WiFiState wifiState = WIFI_DISCONNECTED;
uint32_t wifiConnectStart = 0;
uint32_t lastWifiAttempt = 0;
bool deeperEnabled = false;

// ========== FRAGMENT ASSEMBLY ==========
uint8_t rtcmBuffer[RTCM_BUFFER_SIZE];
uint8_t fragmentsReceived[32];
uint16_t fragmentLengths[32];
uint8_t expectedFragments = 0;
uint16_t totalMessageLength = 0;
uint32_t fragmentStartTime = 0;
bool assemblingMessage = false;

// ========== NMEA BUFFERS ==========
char gpsNmeaBuffer[NMEA_BUFFER_SIZE];
uint16_t gpsNmeaIndex = 0;
char udpNmeaBuffer[UDP_BUFFER_SIZE];
uint16_t udpNmeaIndex = 0;

// ========== GPS STATUS (from M8P rover) ==========
struct GPSStatus {
  bool valid;
  uint8_t fixQuality;     // 0=invalid, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float
  uint8_t satellites;
  double latitude;
  double longitude;
  float hdop;
  float altitude;
  float rtcmAge;
  uint32_t lastUpdate;
  // UTC time from GGA for PPK sync
  uint8_t utcHours;
  uint8_t utcMinutes;
  uint8_t utcSeconds;
  uint16_t utcMillis;     // fractional seconds (0-999)
  bool timeValid;
} gps = {false, 0, 0, 0.0, 0.0, 99.9, 0.0, 0.0, 0, 0, 0, 0, 0, false};

// ========== STATISTICS ==========
uint32_t loraPacketsReceived = 0;
uint32_t loraPacketsLost = 0;
uint32_t rtcmMessagesOutput = 0;
uint32_t rtcmBytesTotal = 0;
uint32_t lastRtcmTime = 0;
uint32_t rtcmInterval = 0;
int16_t lastRSSI = 0;
float lastSNR = 0;
uint16_t lastRtcmType = 0;
uint32_t udpPacketsReceived = 0;
uint32_t logEntriesWritten = 0;

// ========== TIMING ==========
uint32_t lastDisplayUpdate = 0;
uint32_t lastStatsReport = 0;
uint32_t lastLogTime = 0;
uint32_t lastFlushTime = 0;
uint32_t startTime = 0;

// ========== SD CARD STATE ==========
bool sdCardReady = false;
bool recordingEnabled = true;  // Controlled via Bluetooth
char logFileName[32];

// ========== CRC-24Q ==========
const uint32_t CRC24Q_POLY = 0x1864CFB;

// ========== FUNCTION PROTOTYPES ==========
void setupWiFi();
void handleWiFi();
void processUDP();
void parseUdpNmea(char* nmea);
void parseSDDBT(char* nmea);
void parseYXMTW(char* nmea);
void setupSDCard();
void logData();
void createNewLogFile();
void updateDisplay();
void onLoRaReceive(int packetSize);
void processFragment(uint8_t fragmentNum, uint8_t totalFragments, uint8_t* data, uint16_t dataLen);
void resetFragmentAssembly();
void outputRTCM(uint8_t* data, uint16_t length);
bool validateRTCM(uint8_t* data, uint16_t length);
uint32_t crc24q(const uint8_t* data, uint16_t length);
void processGPSChar(char c);
void processGPSByte(uint8_t byte);
void processNMEAChar(char c);
bool processUBXByte(uint8_t byte);
void writeUBXMessage();
void parseGPSNmea(char* nmea);
void handleBluetooth();
void sendBluetoothStatus();
void bleSend(const String& data);
void setupBLE();
bool setupIMU();
void readIMU();
void handleListLogs();
void handleDownloadLog(const String& filename);
void handleDeleteAllLogs();
void calibrateIMULevel();
void loadIMUCalibration();
void saveIMUCalibration();
void writeIMURegister(uint8_t reg, uint8_t value);
uint8_t readIMURegister(uint8_t reg);

// ========== BLE SERVER CALLBACKS ==========
class MyServerCallbacks: public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    bleConnected = true;
    bleConnectTime = millis();
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    bleConnected = false;
    Serial.print("[BLE] Client disconnected, reason: ");
    Serial.println(reason);
  }
};

// ========== BLE RX CHARACTERISTIC CALLBACKS ==========
class MyCallbacks: public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.print("[BLE] RX: ");
      Serial.println(rxValue.c_str());
      bleRxBuffer += rxValue.c_str();
    }
  }

  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    Serial.print("[BLE] Subscription changed: ");
    Serial.println(subValue);
  }
};

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  startTime = millis();

  Serial.println();
  Serial.println("========================================");
  Serial.println("  DEEPER + RTK INTEGRATED RECEIVER");
  Serial.println("        (BLE Version)");
  Serial.println("========================================");
  Serial.println();

  // Print configuration
  Serial.println("Configuration:");
  Serial.print("  Deeper SSID: "); Serial.println(DEEPER_SSID);
  Serial.print("  Deeper IP:   "); Serial.println(DEEPER_IP);
  Serial.print("  UDP Port:    "); Serial.println(DEEPER_UDP_PORT);
  Serial.print("  GPS TX Pin:  IO"); Serial.println(GPS_TX_PIN);
  Serial.print("  GPS RX Pin:  IO"); Serial.println(GPS_RX_PIN);
  Serial.println("  SD Card:     Built-in (SD_MMC 1-bit mode)");
  Serial.print("  LoRa SF:     "); Serial.println(LORA_SPREAD_FACTOR);
  Serial.println();

  // Init OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[ERROR] OLED init failed!");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Deeper+RTK BLE");
  display.println("Starting...");
  display.display();

  // Init SPI for LoRa (SD will share this bus)
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Init LoRa
  Serial.print("[INIT] LoRa... ");
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("FAILED!");
    display.println("LoRa FAILED!");
    display.display();
    while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SPREAD_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  Serial.println("OK");

  // Init GPS UART
  Serial.print("[INIT] GPS UART... ");
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("OK");

  // Phase 1: Set WiFi mode (needed for BLE coexistence)
  setupWiFiMode();

  // Init BLE BEFORE SD card (BLE init conflicts with SD_MMC if done after)
  setupBLE();

  // Init IMU (shares I2C bus with OLED)
  imuAvailable = setupIMU();
  loadIMUCalibration();

  // Init SD Card (after BLE to avoid resource conflict)
  setupSDCard();

  // Phase 2: Start WiFi connection (after SD init)
  startWiFiConnection();

  resetFragmentAssembly();

  Serial.println();
  Serial.println("*** RECEIVER READY (BLE) ***");
  Serial.println();
}

// ========== MAIN LOOP ==========
void loop() {
  // 1. Check for LoRa packets (RTK corrections)
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    onLoRaReceive(packetSize);
  }

  // 2. Handle WiFi connection state machine
  handleWiFi();

  // 3. Process UDP data from Deeper
  if (wifiState == WIFI_UDP_READY) {
    processUDP();
  }

  // 4. Read NMEA from RTK GPS (M8P)
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    processGPSChar(c);
  }

  // 5. Fragment timeout check
  if (assemblingMessage && (millis() - fragmentStartTime > FRAGMENT_TIMEOUT)) {
    loraPacketsLost++;
    resetFragmentAssembly();
  }

  // 6. Log data to SD card
  if (sdCardReady && (millis() - lastLogTime > LOG_INTERVAL_MS)) {
    logData();
    lastLogTime = millis();
  }

  // 7. Flush SD card periodically (both CSV and raw files)
  if (sdCardReady && (millis() - lastFlushTime > FLUSH_INTERVAL_MS)) {
    if (logFile) {
      logFile.flush();
    }
    if (rawFile && ppkLoggingEnabled) {
      rawFile.flush();
    }
    lastFlushTime = millis();
  }

  // 8. Update display
  if (millis() - lastDisplayUpdate > 250) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  // 9. Stats report
  if (millis() - lastStatsReport > 10000) {
    printStatsReport();
    lastStatsReport = millis();
  }

  // 10. Read IMU data
  if (imuAvailable && (millis() - lastImuUpdate > IMU_UPDATE_INTERVAL)) {
    readIMU();
    lastImuUpdate = millis();

    // Print IMU data to serial (throttled to 2 Hz)
    static uint32_t lastImuPrint = 0;
    if (imu.valid && (millis() - lastImuPrint > 500)) {
      Serial.print("[IMU] Pitch: ");
      Serial.print(imu.pitch, 1);
      Serial.print(" deg, Roll: ");
      Serial.print(imu.roll, 1);
      Serial.println(" deg");
      lastImuPrint = millis();
    }
  }

  // 11. Handle BLE
  handleBluetooth();

  // 12. Handle BLE reconnection
  if (!bleConnected && oldBleConnected) {
    // Client disconnected - restart advertising
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Restarted advertising");
    oldBleConnected = bleConnected;
  }
  if (bleConnected && !oldBleConnected) {
    oldBleConnected = bleConnected;
  }

  yield();
}

// ========== BLE SETUP ==========
void setupBLE() {
  Serial.print("[INIT] BLE... ");

  NimBLEDevice::init("DeeperRTK");

  // Create BLE Server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service (Nordic UART)
  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  // Create TX Characteristic (notify) - NimBLE handles CCCD automatically
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    NIMBLE_PROPERTY::NOTIFY
  );
  pTxCharacteristic->setCallbacks(new MyCallbacks());

  // Create RX Characteristic (write)
  NimBLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start service
  pService->start();

  // Start advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();

  Serial.println("OK - Name: DeeperRTK");
}

// ========== WIFI SETUP (Phase 1: Mode only) ==========
void setupWiFiMode() {
  Serial.print("[INIT] WiFi mode... ");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("OK");
}

// ========== WIFI SETUP (Phase 2: Start connection) ==========
void startWiFiConnection() {
  wifiState = WIFI_CONNECTING;
  WiFi.begin(DEEPER_SSID, DEEPER_PASSWORD);
  wifiConnectStart = millis();
  Serial.print("[WIFI] Connecting to: ");
  Serial.println(DEEPER_SSID);
}

// ========== WIFI STATE MACHINE ==========
void handleWiFi() {
  switch (wifiState) {
    case WIFI_DISCONNECTED:
      // Retry connection every 10 seconds
      if (millis() - lastWifiAttempt > 10000) {
        wifiState = WIFI_CONNECTING;
        WiFi.begin(DEEPER_SSID, DEEPER_PASSWORD);
        wifiConnectStart = millis();
        lastWifiAttempt = millis();
        Serial.println("[WIFI] Reconnecting...");
      }
      break;

    case WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        wifiState = WIFI_CONNECTED;
        Serial.println("[WIFI] Connected!");
        Serial.print("[WIFI] IP: ");
        Serial.println(WiFi.localIP());

        // Start UDP listener
        udp.begin(DEEPER_UDP_PORT);
        wifiState = WIFI_UDP_READY;
        Serial.println("[UDP] Listening on port 10110");

        // Send enable command to Deeper
        enableDeeper();
      } else if (millis() - wifiConnectStart > 15000) {
        // Timeout after 15 seconds
        Serial.print("[WIFI] Timeout! Status code: ");
        Serial.println(WiFi.status());
        wifiState = WIFI_DISCONNECTED;
        lastWifiAttempt = millis();
      } else {
        // Print status every 3 seconds while connecting
        static uint32_t lastDbg = 0;
        if (millis() - lastDbg > 3000) {
          Serial.print("[WIFI] Connecting... status=");
          Serial.println(WiFi.status());
          lastDbg = millis();
        }
      }
      break;

    case WIFI_CONNECTED:
    case WIFI_UDP_READY:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] Disconnected!");
        wifiState = WIFI_DISCONNECTED;
        deeperEnabled = false;
        lastWifiAttempt = millis();
      }
      break;
  }
}

// ========== ENABLE DEEPER NMEA OUTPUT ==========
void enableDeeper() {
  IPAddress deeperIP;
  deeperIP.fromString(DEEPER_IP);

  udp.beginPacket(deeperIP, DEEPER_UDP_PORT);
  udp.print(DEEPER_ENABLE_CMD);
  udp.endPacket();

  deeperEnabled = true;
  Serial.println("[DEEPER] NMEA enabled");
}

// ========== PROCESS UDP PACKETS ==========
void processUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    udpPacketsReceived++;

    char buffer[UDP_BUFFER_SIZE];
    int len = udp.read(buffer, UDP_BUFFER_SIZE - 1);
    buffer[len] = '\0';

    // Process each line in the packet (may contain multiple NMEA sentences)
    char* line = strtok(buffer, "\r\n");
    while (line != NULL) {
      if (line[0] == '$') {
        parseUdpNmea(line);
      }
      line = strtok(NULL, "\r\n");
    }
  }
}

// ========== PARSE UDP NMEA SENTENCES ==========
void parseUdpNmea(char* nmea) {
  // We only care about depth and temperature from Deeper
  // (GPS data comes from our RTK receiver)

  if (strstr(nmea, "DBT") != NULL) {
    // Depth Below Transducer
    parseSDDBT(nmea);
  } else if (strstr(nmea, "MTW") != NULL) {
    // Mean Temperature of Water
    parseYXMTW(nmea);
  } else if (strstr(nmea, "STA") != NULL || strstr(nmea, "BAT") != NULL) {
    // Status/Battery messages (proprietary Deeper)
    parseDeeperStatus(nmea);
  }

  // Debug: uncomment to see all UDP messages
  // Serial.print("[UDP] "); Serial.println(nmea);
}

// Placeholder for battery parsing - Deeper may send battery in proprietary format
void parseDeeperStatus(char* nmea) {
  // Look for battery percentage in the message
  // Common formats: $PDEEP,BAT,85*xx or similar
  char* batPtr = strstr(nmea, "BAT");
  if (batPtr) {
    batPtr += 3;  // Skip "BAT"
    while (*batPtr == 0x27 || *batPtr == 0x2C) batPtr++;  // Skip comma
    int batt = atoi(batPtr);
    if (batt >= 0 && batt <= 100) {
      sonar.batteryPercent = batt;
      Serial.print("[SONAR] Battery: "); Serial.print(batt); Serial.println("%");
    }
  }
}


// ========== PARSE DEPTH (SDDBT) ==========
// Format: $SDDBT,depth_ft,f,depth_m,M,depth_fathoms,F*hh
void parseSDDBT(char* nmea) {
  char temp[NMEA_BUFFER_SIZE];
  strncpy(temp, nmea, NMEA_BUFFER_SIZE);

  char* p = temp;
  int field = 0;
  char* fields[10];

  fields[0] = p;
  while (*p && field < 9) {
    if (*p == ',' || *p == '*') {
      *p = '\0';
      fields[++field] = p + 1;
    }
    p++;
  }

  if (field >= 3) {
    // Field 1: depth in feet
    // Field 3: depth in meters
    if (strlen(fields[1]) > 0) {
      sonar.depthFeet = atof(fields[1]);
    }
    if (strlen(fields[3]) > 0) {
      sonar.depthMeters = atof(fields[3]);
      sonar.valid = true;
      // Calculate depth update rate
      static uint32_t depthUpdateCount = 0;
      static uint32_t rateWindowStart = 0;

      depthUpdateCount++;
      uint32_t now = millis();
      uint32_t elapsed = now - rateWindowStart;

      if (elapsed >= 2000) {
        float depthHz = (float)depthUpdateCount * 1000.0 / elapsed;
        depthUpdateCount = 0;
        rateWindowStart = now;
        Serial.print("[SONAR] Rate: "); Serial.print(depthHz, 1); Serial.println(" Hz");
      }

      sonar.lastUpdate = now;

      Serial.print("[SONAR] Depth: ");
      Serial.print(sonar.depthMeters, 2);
      Serial.println(" m");
    }
  }
}

// ========== PARSE WATER TEMPERATURE (YXMTW) ==========
// Format: $YXMTW,temp_c,C*hh
void parseYXMTW(char* nmea) {
  char temp[NMEA_BUFFER_SIZE];
  strncpy(temp, nmea, NMEA_BUFFER_SIZE);

  char* p = temp;
  int field = 0;
  char* fields[5];

  fields[0] = p;
  while (*p && field < 4) {
    if (*p == ',' || *p == '*') {
      *p = '\0';
      fields[++field] = p + 1;
    }
    p++;
  }

  if (field >= 1 && strlen(fields[1]) > 0) {
    sonar.waterTempC = atof(fields[1]);

    Serial.print("[SONAR] Water Temp: ");
    Serial.print(sonar.waterTempC, 1);
    Serial.println(" C");
  }
}

// ========== SD CARD SETUP ==========
void setupSDCard() {
  Serial.print("[INIT] SD Card (SD_MMC 1-bit)... ");

  // Initialize SD_MMC in 1-bit mode for built-in SD card slot
  // Pins are fixed by hardware: CLK=GPIO14, CMD=GPIO15, D0=GPIO2
  if (!SD_MMC.begin("/sdcard", true)) {  // true = 1-bit mode
    Serial.println("FAILED!");
    Serial.println("[SD] No card detected or init failed");
    Serial.println("[SD] Check: card inserted? FAT32 formatted?");
    sdCardReady = false;
    return;
  }

  Serial.println("OK");

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD] No card attached");
    sdCardReady = false;
    return;
  }

  Serial.print("[SD] Card type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.print("[SD] Card size: ");
  Serial.print(cardSize);
  Serial.println(" MB");

  sdCardReady = true;
  createNewLogFile();
}

// ========== CREATE NEW LOG FILE ==========
void createNewLogFile() {
  if (!sdCardReady) return;

  // Find next available file number
  int fileNum = 0;
  do {
    snprintf(logFileName, sizeof(logFileName), "/log_%04d.csv", fileNum++);
  } while (SD_MMC.exists(logFileName) && fileNum < 10000);
  fileNum--;  // Back to the number we're using

  logFile = SD_MMC.open(logFileName, FILE_WRITE);
  if (logFile) {
    // Write CSV header
    logFile.println("timestamp_ms,utc_time,gps_fix,lat,lon,alt_m,hdop,sats,rtcm_age_s,depth_m,water_temp_c,rssi,snr,pitch,roll");
    logFile.flush();

    Serial.print("[SD] Logging to: ");
    Serial.println(logFileName);
    
    // Create matching UBX raw file for PPK
    snprintf(rawFileName, sizeof(rawFileName), "/raw_%04d.ubx", fileNum);
    rawFile = SD_MMC.open(rawFileName, FILE_WRITE);
    if (rawFile) {
      Serial.print("[SD] PPK raw file: ");
      Serial.println(rawFileName);
    } else {
      Serial.println("[SD] Failed to create PPK raw file");
    }
  } else {
    Serial.println("[SD] Failed to create log file!");
    sdCardReady = false;
  }
}

// ========== LOG DATA TO SD CARD ==========
void logData() {
  if (!logFile || !recordingEnabled) return;

  // Only log if we have some valid data
  if (!gps.valid && !sonar.valid) return;

  // Format: timestamp_ms,utc_time,gps_fix,lat,lon,alt_m,hdop,sats,rtcm_age_s,depth_m,water_temp_c,rssi,snr,pitch,roll

  logFile.print(millis());
  logFile.print(",");
  // UTC time in HH:MM:SS.mmm format
  if (gps.timeValid) {
    char utcBuf[16];
    snprintf(utcBuf, sizeof(utcBuf), "%02d:%02d:%02d.%03d", gps.utcHours, gps.utcMinutes, gps.utcSeconds, gps.utcMillis);
    logFile.print(utcBuf);
  }
  logFile.print(",");
  logFile.print(gps.fixQuality);
  logFile.print(",");
  logFile.print(gps.latitude, 8);
  logFile.print(",");
  logFile.print(gps.longitude, 8);
  logFile.print(",");
  logFile.print(gps.altitude, 2);
  logFile.print(",");
  logFile.print(gps.hdop, 2);
  logFile.print(",");
  logFile.print(gps.satellites);
  logFile.print(",");
  logFile.print(gps.rtcmAge, 1);
  logFile.print(",");

  if (sonar.valid && (millis() - sonar.lastUpdate < 5000)) {
    logFile.print(sonar.depthMeters, 2);
  }
  logFile.print(",");

  if (sonar.waterTempC > 0) {
    logFile.print(sonar.waterTempC, 1);
  }
  logFile.print(",");

  logFile.print(lastRSSI);
  logFile.print(",");
  logFile.print(lastSNR, 1);
  logFile.print(",");

  // IMU data (pitch and roll)
  if (imu.valid) {
    logFile.print(imu.pitch, 2);
    logFile.print(",");
    logFile.println(imu.roll, 2);
  } else {
    logFile.println(",");  // Empty pitch/roll columns
  }

  logEntriesWritten++;
}

// ========== DISPLAY UPDATE ==========
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  // Row 1: RTK Status + Depth
  display.setTextSize(1);
  switch (gps.fixQuality) {
    case 0: display.print("NO FIX"); break;
    case 1: display.print("GPS"); break;
    case 2: display.print("DGPS"); break;
    case 4: display.print("FIXED"); break;
    case 5: display.print("FLOAT"); break;
    default: display.print("???"); break;
  }

  // Show depth on same row
  if (sonar.valid && (millis() - sonar.lastUpdate < 5000)) {
    display.print(" D:");
    display.print(sonar.depthMeters, 1);
    display.print("m");
  }
  display.println();

  // Row 2: Satellites and HDOP
  display.print("Sat:");
  display.print(gps.satellites);
  display.print(" H:");
  display.print(gps.hdop, 1);

  // Water temp
  if (sonar.waterTempC > 0) {
    display.print(" ");
    display.print((int)sonar.waterTempC);
    display.print("C");
  }
  display.println();

  // Row 3: RTCM Age
  display.print("RTCM:");
  if (gps.rtcmAge > 0 && gps.rtcmAge < 100) {
    display.print(gps.rtcmAge, 1);
    display.print("s");
  } else {
    display.print("N/A");
  }
  display.println();

  // Row 4: LoRa signal
  display.print("R:");
  display.print(lastRSSI);
  display.print(" S:");
  display.println(lastSNR, 1);

  // Row 5: WiFi/Deeper status + BLE
  display.print("WiFi:");
  switch (wifiState) {
    case WIFI_DISCONNECTED: display.print("OFF"); break;
    case WIFI_CONNECTING:   display.print("..."); break;
    case WIFI_CONNECTED:
    case WIFI_UDP_READY:    display.print("OK"); break;
  }

  // BLE status
  display.print(" BLE:");
  display.print(bleConnected ? "Y" : "N");
  display.println();

  // Row 6: SD status / RTCM count
  display.print("SD:");
  if (sdCardReady) {
    display.print(logEntriesWritten);
  } else {
    display.print("NO");
  }
  display.print(" RTCM:");
  display.print(rtcmMessagesOutput);

  display.display();
}

// ========== STATS REPORT ==========
void printStatsReport() {
  uint32_t uptime = (millis() - startTime) / 1000;

  Serial.println();
  Serial.println("========================================");
  Serial.println("       DEEPER + RTK STATUS REPORT");
  Serial.println("              (BLE Version)");
  Serial.println("========================================");

  Serial.print("Uptime: "); Serial.print(uptime); Serial.println(" seconds");
  Serial.println();

  Serial.println("--- GPS STATUS ---");
  Serial.print("  Fix: ");
  switch (gps.fixQuality) {
    case 0: Serial.println("No Fix"); break;
    case 1: Serial.println("GPS"); break;
    case 2: Serial.println("DGPS"); break;
    case 4: Serial.println("RTK FIXED!"); break;
    case 5: Serial.println("RTK Float"); break;
    default: Serial.println("Unknown"); break;
  }
  Serial.print("  Satellites: "); Serial.println(gps.satellites);
  Serial.print("  HDOP: "); Serial.println(gps.hdop, 2);
  Serial.print("  Position: ");
  Serial.print(gps.latitude, 7); Serial.print(", ");
  Serial.println(gps.longitude, 7);
  Serial.print("  RTCM Age: "); Serial.print(gps.rtcmAge, 1); Serial.println("s");
  Serial.println();

  Serial.println("--- SONAR STATUS ---");
  Serial.print("  WiFi: ");
  switch (wifiState) {
    case WIFI_DISCONNECTED: Serial.println("Disconnected"); break;
    case WIFI_CONNECTING:   Serial.println("Connecting..."); break;
    case WIFI_CONNECTED:    Serial.println("Connected"); break;
    case WIFI_UDP_READY:    Serial.println("Ready"); break;
  }
  Serial.print("  UDP Packets: "); Serial.println(udpPacketsReceived);
  Serial.print("  Depth: ");
  if (sonar.valid) {
    Serial.print(sonar.depthMeters, 2); Serial.println(" m");
  } else {
    Serial.println("N/A");
  }
  Serial.print("  Water Temp: ");
  if (sonar.waterTempC > 0) {
    Serial.print(sonar.waterTempC, 1); Serial.println(" C");
  } else {
    Serial.println("N/A");
  }
  Serial.println();

  Serial.println("--- LORA LINK ---");
  Serial.print("  RSSI: "); Serial.print(lastRSSI); Serial.println(" dBm");
  Serial.print("  SNR: "); Serial.print(lastSNR, 1); Serial.println(" dB");
  Serial.print("  Packets RX: "); Serial.println(loraPacketsReceived);
  Serial.print("  Packets Lost: "); Serial.println(loraPacketsLost);
  Serial.print("  RTCM Messages: "); Serial.println(rtcmMessagesOutput);
  Serial.println();

  Serial.println("--- BLE STATUS ---");
  Serial.print("  Connected: "); Serial.println(bleConnected ? "Yes" : "No");
  Serial.println();

  Serial.println("--- IMU STATUS ---");
  Serial.print("  Available: "); Serial.println(imuAvailable ? "Yes" : "No");
  if (imuAvailable && imu.valid) {
    Serial.print("  Pitch: "); Serial.print(imu.pitch, 1); Serial.println(" deg");
    Serial.print("  Roll:  "); Serial.print(imu.roll, 1); Serial.println(" deg");
  }
  Serial.println();

  Serial.println("--- DATA LOGGING ---");
  Serial.print("  SD Card: "); Serial.println(sdCardReady ? "Ready" : "Not available");
  if (sdCardReady) {
    Serial.print("  Log File: "); Serial.println(logFileName);
    Serial.print("  Entries: "); Serial.println(logEntriesWritten);
  }
  Serial.println();
  Serial.println("========================================");
  Serial.println();
}

// ========== UBX MESSAGE LOGGING FOR PPK ==========
void writeUBXMessage() {
  if (!rawFile || !ppkLoggingEnabled) return;

  // Only log RAWX and SFRBX messages (needed for PPK)
  if (ubxClass == UBX_CLASS_RXM && (ubxId == UBX_ID_RAWX || ubxId == UBX_ID_SFRBX)) {
    size_t totalLen = 6 + ubxPayloadLen + 2;
    rawFile.write(ubxBuffer, totalLen);
    ubxBytesWritten += totalLen;
    ubxMessagesLogged++;
  }
}

// ========== UBX PARSER STATE MACHINE ==========
bool processUBXByte(uint8_t byte) {
  switch (ubxState) {
    case UBX_WAIT_SYNC1:
      if (byte == UBX_SYNC1) {
        ubxBuffer[0] = byte;
        ubxIndex = 1;
        ubxCkA = 0;
        ubxCkB = 0;
        ubxState = UBX_WAIT_SYNC2;
        return true;
      }
      return false;

    case UBX_WAIT_SYNC2:
      if (byte == UBX_SYNC2) {
        ubxBuffer[ubxIndex++] = byte;
        ubxState = UBX_WAIT_CLASS;
        return true;
      }
      ubxState = UBX_WAIT_SYNC1;
      return false;

    case UBX_WAIT_CLASS:
      ubxBuffer[ubxIndex++] = byte;
      ubxClass = byte;
      ubxCkA += byte;
      ubxCkB += ubxCkA;
      ubxState = UBX_WAIT_ID;
      return true;

    case UBX_WAIT_ID:
      ubxBuffer[ubxIndex++] = byte;
      ubxId = byte;
      ubxCkA += byte;
      ubxCkB += ubxCkA;
      ubxState = UBX_WAIT_LEN1;
      return true;

    case UBX_WAIT_LEN1:
      ubxBuffer[ubxIndex++] = byte;
      ubxPayloadLen = byte;
      ubxCkA += byte;
      ubxCkB += ubxCkA;
      ubxState = UBX_WAIT_LEN2;
      return true;

    case UBX_WAIT_LEN2:
      ubxBuffer[ubxIndex++] = byte;
      ubxPayloadLen |= ((uint16_t)byte << 8);
      ubxCkA += byte;
      ubxCkB += ubxCkA;
      if (ubxPayloadLen > UBX_BUFFER_SIZE - 8) {
        ubxState = UBX_WAIT_SYNC1;
        return false;
      }
      ubxState = (ubxPayloadLen == 0) ? UBX_WAIT_CKA : UBX_COLLECTING;
      return true;

    case UBX_COLLECTING:
      ubxBuffer[ubxIndex++] = byte;
      ubxCkA += byte;
      ubxCkB += ubxCkA;
      if (ubxIndex >= 6 + ubxPayloadLen) {
        ubxState = UBX_WAIT_CKA;
      }
      return true;

    case UBX_WAIT_CKA:
      ubxBuffer[ubxIndex++] = byte;
      ubxState = (byte == ubxCkA) ? UBX_WAIT_CKB : UBX_WAIT_SYNC1;
      return true;

    case UBX_WAIT_CKB:
      ubxBuffer[ubxIndex++] = byte;
      if (byte == ubxCkB) {
        writeUBXMessage();
      }
      ubxState = UBX_WAIT_SYNC1;
      return true;
  }
  return false;
}

// ========== GPS BYTE PROCESSING (handles both NMEA and UBX) ==========
void processGPSByte(uint8_t byte) {
  // Try UBX parser first
  if (ubxState != UBX_WAIT_SYNC1 || byte == UBX_SYNC1) {
    if (processUBXByte(byte)) {
      return;
    }
  }
  // Otherwise treat as NMEA
  processNMEAChar((char)byte);
}

// ========== NMEA CHARACTER PROCESSING ==========
void processNMEAChar(char c) {
  if (c == '$') {
    gpsNmeaIndex = 0;
  }
  if (gpsNmeaIndex < NMEA_BUFFER_SIZE - 1) {
    gpsNmeaBuffer[gpsNmeaIndex++] = c;
  }
  if (c == '\n') {
    gpsNmeaBuffer[gpsNmeaIndex] = '\0';
    parseGPSNmea(gpsNmeaBuffer);
    gpsNmeaIndex = 0;
  }
}

// Legacy wrapper
void processGPSChar(char c) {
  processGPSByte((uint8_t)c);
}

// ========== PARSE GPS NMEA (GGA from M8P) ==========
void parseGPSNmea(char* nmea) {
  if (strstr(nmea, "GGA") != NULL) {
    char temp[NMEA_BUFFER_SIZE];
    strncpy(temp, nmea, NMEA_BUFFER_SIZE);

    char* p = temp;
    int field = 0;
    char* fields[15];

    fields[0] = p;
    while (*p && field < 14) {
      if (*p == ',') {
        *p = '\0';
        fields[++field] = p + 1;
      }
      p++;
    }

    if (field >= 13) {
      // Parse UTC time from field 1 (format: HHMMSS.SS)
      if (strlen(fields[1]) >= 6) {
        double rawTime = atof(fields[1]);
        int timeInt = (int)rawTime;
        gps.utcHours = timeInt / 10000;
        gps.utcMinutes = (timeInt / 100) % 100;
        gps.utcSeconds = timeInt % 100;
        gps.utcMillis = (uint16_t)((rawTime - timeInt) * 1000);
        gps.timeValid = true;
      }

      gps.fixQuality = atoi(fields[6]);
      gps.satellites = atoi(fields[7]);
      gps.hdop = atof(fields[8]);
      gps.altitude = atof(fields[9]);
      gps.rtcmAge = atof(fields[13]);

      if (strlen(fields[2]) > 0) {
        double rawLat = atof(fields[2]);
        int degrees = (int)(rawLat / 100);
        double minutes = rawLat - (degrees * 100);
        gps.latitude = degrees + (minutes / 60.0);
        if (fields[3][0] == 'S') gps.latitude = -gps.latitude;
      }

      if (strlen(fields[4]) > 0) {
        double rawLon = atof(fields[4]);
        int degrees = (int)(rawLon / 100);
        double minutes = rawLon - (degrees * 100);
        gps.longitude = degrees + (minutes / 60.0);
        if (fields[5][0] == 'W') gps.longitude = -gps.longitude;
      }

      gps.valid = (gps.fixQuality > 0);
      gps.lastUpdate = millis();
    }
  }
}

// ========== LORA RECEIVE ==========
void onLoRaReceive(int packetSize) {
  if (packetSize < 3) return;

  lastRSSI = LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  loraPacketsReceived++;

  uint8_t fragmentNum = LoRa.read();
  uint8_t totalFragments = LoRa.read();

  uint16_t dataLen;
  uint8_t tempBuffer[256];

  if (fragmentNum == 0) {
    if (packetSize < 5) return;
    LoRa.read();  // timestamp low - skip
    LoRa.read();  // timestamp high - skip
    dataLen = packetSize - 4;
  } else {
    dataLen = packetSize - 2;
  }

  for (uint16_t i = 0; i < dataLen && i < 256; i++) {
    tempBuffer[i] = LoRa.read();
  }

  processFragment(fragmentNum, totalFragments, tempBuffer, dataLen);
}

// ========== FRAGMENT PROCESSING ==========
void processFragment(uint8_t fragmentNum, uint8_t totalFragments,
                     uint8_t* data, uint16_t dataLen) {

  if (totalFragments == 0 || totalFragments > 32 || fragmentNum >= totalFragments) {
    return;
  }

  if (!assemblingMessage) {
    assemblingMessage = true;
    expectedFragments = totalFragments;
    fragmentStartTime = millis();
    totalMessageLength = 0;
    memset(fragmentsReceived, 0, sizeof(fragmentsReceived));
    memset(fragmentLengths, 0, sizeof(fragmentLengths));
  }

  if (totalFragments != expectedFragments) {
    resetFragmentAssembly();
    assemblingMessage = true;
    expectedFragments = totalFragments;
    fragmentStartTime = millis();
    memset(fragmentLengths, 0, sizeof(fragmentLengths));
  }

  // Fragment offset calculation (critical!)
  uint16_t fragmentOffset;
  if (fragmentNum == 0) {
    fragmentOffset = 0;
  } else {
    fragmentOffset = 246 + (fragmentNum - 1) * 248;
  }

  if (fragmentOffset + dataLen > RTCM_BUFFER_SIZE) {
    resetFragmentAssembly();
    return;
  }

  memcpy(rtcmBuffer + fragmentOffset, data, dataLen);
  fragmentsReceived[fragmentNum / 8] |= (1 << (fragmentNum % 8));
  fragmentLengths[fragmentNum] = dataLen;

  // Check if all fragments received
  bool allReceived = true;
  for (uint8_t i = 0; i < totalFragments; i++) {
    if (!(fragmentsReceived[i / 8] & (1 << (i % 8)))) {
      allReceived = false;
      break;
    }
  }

  if (allReceived) {
    totalMessageLength = 0;
    for (uint8_t i = 0; i < totalFragments; i++) {
      totalMessageLength += fragmentLengths[i];
    }

    if (rtcmBuffer[0] == 0xD3) {
      outputRTCM(rtcmBuffer, totalMessageLength);
      rtcmMessagesOutput++;
      rtcmBytesTotal += totalMessageLength;

      uint32_t now = millis();
      if (lastRtcmTime > 0) {
        rtcmInterval = now - lastRtcmTime;
      }
      lastRtcmTime = now;
    }

    resetFragmentAssembly();
  }
}

// ========== RESET FRAGMENT STATE ==========
void resetFragmentAssembly() {
  assemblingMessage = false;
  expectedFragments = 0;
  totalMessageLength = 0;
  fragmentStartTime = 0;
  memset(fragmentsReceived, 0, sizeof(fragmentsReceived));
  memset(fragmentLengths, 0, sizeof(fragmentLengths));
}

// ========== CRC-24Q VALIDATION ==========
uint32_t crc24q(const uint8_t* data, uint16_t length) {
  uint32_t crc = 0;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= ((uint32_t)data[i]) << 16;
    for (int j = 0; j < 8; j++) {
      crc <<= 1;
      if (crc & 0x1000000) {
        crc ^= CRC24Q_POLY;
      }
    }
  }
  return crc & 0xFFFFFF;
}

bool validateRTCM(uint8_t* data, uint16_t length) {
  if (length < 6) return false;
  if (data[0] != 0xD3) return false;

  uint16_t msgLen = ((data[1] & 0x03) << 8) | data[2];
  if (length != msgLen + 6) return false;

  uint32_t calcCRC = crc24q(data, length - 3);
  uint32_t msgCRC = ((uint32_t)data[length-3] << 16) |
                    ((uint32_t)data[length-2] << 8) |
                    data[length-1];

  return (calcCRC == msgCRC);
}

// ========== OUTPUT RTCM ==========
void outputRTCM(uint8_t* data, uint16_t length) {
  if (length >= 6 && data[0] == 0xD3) {
    lastRtcmType = (data[3] << 4) | (data[4] >> 4);

    if (!validateRTCM(data, length)) {
      // Skip corrupted messages
      return;
    }
  }

  GPSSerial.write(data, length);
}

// ========== BLE SEND FUNCTION ==========
void bleSend(const String& data) {
  // Only send if connected AND connection is stable
  if (bleConnected && pTxCharacteristic && pServer) {
    // Wait at least 200ms after connection before sending
    if (millis() - bleConnectTime < 200) return;

    // Only notify if we have connected clients
    if (pServer->getConnectedCount() == 0) {
      Serial.println("[BLE] TX skipped - no clients");
      return;
    }

    pTxCharacteristic->setValue(data.c_str());
    pTxCharacteristic->notify();
    Serial.print("[BLE] TX: ");
    Serial.println(data.substring(0, 30));  // First 30 chars
  } else {
    Serial.print("[BLE] TX blocked - connected:");
    Serial.print(bleConnected);
    Serial.print(" pTx:");
    Serial.print(pTxCharacteristic != NULL);
    Serial.print(" pSrv:");
    Serial.println(pServer != NULL);
  }
}

// ========== BLUETOOTH FUNCTIONS ==========
uint32_t lastBtUpdate = 0;

void sendBluetoothStatus() {
  if (!bleConnected) return;

  // Format: FIX,SATS,DEPTH,TEMP,BATT,REC,PITCH,ROLL
  // Example: 4,12,1.25,18.5,85,1,5.2,-3.1
  String status = String(gps.fixQuality);
  status += ",";
  status += String(gps.satellites);
  status += ",";
  if (sonar.valid && (millis() - sonar.lastUpdate < 5000)) {
    status += String(sonar.depthMeters, 2);
  } else {
    status += "-1";
  }
  status += ",";
  status += String(sonar.waterTempC, 1);
  status += ",";
  status += String(sonar.batteryPercent);
  status += ",";
  status += String(recordingEnabled ? 1 : 0);
  status += ",";
  // Add IMU data (pitch and roll)
  if (imu.valid && (millis() - imu.lastUpdate < 1000)) {
    status += String(imu.pitch, 1);
    status += ",";
    status += String(imu.roll, 1);
  } else {
    status += "0,0";  // Default to level if no IMU
  }
  // Add lat/lon for app display
  status += ",";
  if (gps.valid && gps.latitude != 0.0) {
    status += String(gps.latitude, 6);
    status += ",";
    status += String(gps.longitude, 6);
  } else {
    status += "0,0";
  }
  status += "\n";

  bleSend(status);
}

// Send command to Deeper sonar via UDP
void sendDeeperCommand(const char* cmd) {
  if (wifiState == WIFI_UDP_READY) {
    udp.beginPacket(DEEPER_IP, DEEPER_UDP_PORT);
    udp.print(cmd);
    udp.endPacket();
    Serial.print("[DEEPER] Sent: "); Serial.println(cmd);
  }
}

void handleBluetooth() {
  // Process any received BLE data
  while (bleRxBuffer.length() > 0) {
    int newlineIdx = bleRxBuffer.indexOf('\n');
    if (newlineIdx == -1) break;  // No complete command yet

    String cmd = bleRxBuffer.substring(0, newlineIdx);
    bleRxBuffer = bleRxBuffer.substring(newlineIdx + 1);
    cmd.trim();

    if (cmd == "START") {
      recordingEnabled = true;
      bleSend("OK:REC_ON\n");
      Serial.println("[BLE] Recording started");
    } else if (cmd == "STOP") {
      recordingEnabled = false;
      bleSend("OK:REC_OFF\n");
      Serial.println("[BLE] Recording stopped");
    } else if (cmd == "STATUS") {
      sendBluetoothStatus();
    } else if (cmd == "FREQ_WIDE") {
      sendDeeperCommand(DEEPER_WIDE_CMD);
      bleSend("OK:FREQ_WIDE\n");
    } else if (cmd == "FREQ_MED") {
      sendDeeperCommand(DEEPER_MEDIUM_CMD);
      bleSend("OK:FREQ_MED\n");
    } else if (cmd == "FREQ_NARROW") {
      sendDeeperCommand(DEEPER_NARROW_CMD);
      bleSend("OK:FREQ_NARROW\n");
    } else if (cmd == "PPK_ON") {
      ppkLoggingEnabled = true;
      bleSend("OK:PPK_ON\n");
      Serial.println("[BLE] PPK logging enabled");
    } else if (cmd == "PPK_OFF") {
      ppkLoggingEnabled = false;
      bleSend("OK:PPK_OFF\n");
      Serial.println("[BLE] PPK logging disabled");
    } else if (cmd == "PPK_STATUS") {
      char buf[64];
      snprintf(buf, sizeof(buf), "PPK:%s,MSG:%lu,KB:%.1f\n",
               ppkLoggingEnabled ? "ON" : "OFF",
               ubxMessagesLogged,
               ubxBytesWritten / 1024.0);
      bleSend(buf);
    } else if (cmd == "IMU_LEVEL") {
      calibrateIMULevel();
      bleSend("OK:IMU_LEVEL\n");
      Serial.println("[BLE] IMU leveled");
    } else if (cmd == "LIST_LOGS") {
      handleListLogs();
    } else if (cmd.startsWith("DOWNLOAD:")) {
      String filename = cmd.substring(9);
      handleDownloadLog(filename);
    } else if (cmd == "DELETE_ALL") {
      handleDeleteAllLogs();
    }
  }

  // Send status at 15Hz (~67ms) to connected clients
  if (bleConnected && (millis() - lastBtUpdate > 67)) {
    sendBluetoothStatus();
    lastBtUpdate = millis();
  }
}

// ========== IMU FUNCTIONS ==========

void writeIMURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readIMURegister(uint8_t reg) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, (uint8_t)1);
  return Wire.read();
}

bool setupIMU() {
  Serial.print("[INIT] IMU (ADXL345)... ");

  // Check device ID
  uint8_t devId = readIMURegister(ADXL345_DEVID);
  if (devId != 0xE5) {
    Serial.print("NOT FOUND (ID=0x");
    Serial.print(devId, HEX);
    Serial.println(")");
    return false;
  }

  // Configure ADXL345
  // Set data rate to 100 Hz (0x0A)
  writeIMURegister(ADXL345_BW_RATE, 0x0A);

  // Set data format: +/- 16g range, full resolution (0x0B)
  // FULL_RES=1, Range=11 (±16g) -> ~256 LSB/g (3.9mg/LSB)
  // NOTE: ±2g/±4g modes cause Z-axis saturation on many ADXL345 chips
  writeIMURegister(ADXL345_DATA_FORMAT, 0x0B);

  // Enable measurement mode
  writeIMURegister(ADXL345_POWER_CTL, 0x08);

  // Read back and verify config
  uint8_t dataFormat = readIMURegister(ADXL345_DATA_FORMAT);
  Serial.print("OK (DATA_FORMAT=0x");
  Serial.print(dataFormat, HEX);
  Serial.println(")");
  return true;
}

void readIMU() {
  // Read 6 bytes of data (X, Y, Z - each 2 bytes, little-endian)
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(ADXL345_DATAX0);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, (uint8_t)6);

  if (Wire.available() >= 6) {
    // Read bytes explicitly to avoid undefined evaluation order
    uint8_t x0 = Wire.read();
    uint8_t x1 = Wire.read();
    uint8_t y0 = Wire.read();
    uint8_t y1 = Wire.read();
    uint8_t z0 = Wire.read();
    uint8_t z1 = Wire.read();

    int16_t rawX = x0 | (x1 << 8);
    int16_t rawY = y0 | (y1 << 8);
    int16_t rawZ = z0 | (z1 << 8);

    // Convert to g - ADXL345 at +/-16g full resolution mode
    // 256 LSB/g (3.9mg/LSB) - same scale in FULL_RES mode regardless of range
    const float scale = 1.0 / 256.0;

    // Use global calibration offsets (updated by IMU_LEVEL command)
    imu.accelX = (rawX - imuXOffset) * scale;
    imu.accelY = (rawY - imuYOffset) * scale;
    imu.accelZ = (rawZ - imuZOffset) * scale;

    // Calculate raw pitch and roll from accelerometer
    // Pitch: rotation around X axis (nose up/down)
    // Roll: rotation around Y axis (tilt left/right) - negated to match screen
    float rawPitch = atan2(-imu.accelX, sqrt(imu.accelY * imu.accelY + imu.accelZ * imu.accelZ)) * 180.0 / PI;
    float rawRoll = -atan2(imu.accelY, imu.accelZ) * 180.0 / PI;

    // Apply exponential moving average filter for smooth output
    if (!imuFilterInitialized) {
      // Initialize filter with first reading
      filteredPitch = rawPitch;
      filteredRoll = rawRoll;
      imuFilterInitialized = true;
    } else {
      // EMA: filtered = alpha * new + (1-alpha) * old
      filteredPitch = IMU_FILTER_ALPHA * rawPitch + (1.0f - IMU_FILTER_ALPHA) * filteredPitch;
      filteredRoll = IMU_FILTER_ALPHA * rawRoll + (1.0f - IMU_FILTER_ALPHA) * filteredRoll;
    }

    // Use filtered values for output
    imu.pitch = filteredPitch;
    imu.roll = filteredRoll;

    imu.lastUpdate = millis();
    imu.valid = true;
  }
}


// ========== LOG MANAGEMENT FUNCTIONS ==========

void handleListLogs() {
  if (!sdCardReady) {
    bleSend("ERR:NO_SD\n");
    return;
  }

  File root = SD_MMC.open("/");
  if (!root) {
    bleSend("ERR:OPEN_FAILED\n");
    return;
  }

  String response = "LOGS:";
  int fileCount = 0;

  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    // Only list CSV log files
    if (name.startsWith("log_") && name.endsWith(".csv")) {
      if (fileCount > 0) {
        response += ";";
      }
      // Format: name,size,timestamp
      response += name;
      response += ",";
      response += String(file.size());
      response += ",";
      response += String(file.getLastWrite());
      fileCount++;
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();

  if (fileCount == 0) {
    bleSend("LOGS:EMPTY\n");
  } else {
    response += "\n";
    bleSend(response);
  }

  Serial.print("[LOG] Listed ");
  Serial.print(fileCount);
  Serial.println(" log files");
}

void handleDownloadLog(const String& filename) {
  if (!sdCardReady) {
    bleSend("ERR:NO_SD\n");
    return;
  }

  String path = "/" + filename;
  File file = SD_MMC.open(path.c_str(), FILE_READ);
  if (!file) {
    bleSend("ERR:FILE_NOT_FOUND\n");
    return;
  }

  // Send file start marker
  bleSend("FILE_START:" + filename + "\n");

  // Read and send file in chunks
  const size_t chunkSize = 400;  // Stay under BLE MTU
  char buffer[chunkSize + 1];

  while (file.available()) {
    size_t bytesRead = file.readBytes(buffer, chunkSize);
    buffer[bytesRead] = '\0';

    String chunk = "FILE_DATA:";
    chunk += buffer;
    chunk += "\n";
    bleSend(chunk);

    // Small delay to avoid BLE congestion
    delay(20);
    yield();
  }

  file.close();
  bleSend("FILE_END\n");

  Serial.print("[LOG] Downloaded: ");
  Serial.println(filename);
}

void handleDeleteAllLogs() {
  if (!sdCardReady) {
    bleSend("ERR:NO_SD\n");
    return;
  }

  // Close current log files first
  if (logFile) {
    logFile.close();
  }
  if (rawFile) {
    rawFile.close();
  }

  File root = SD_MMC.open("/");
  if (!root) {
    bleSend("ERR:OPEN_FAILED\n");
    return;
  }

  int deletedCount = 0;

  // First pass: collect filenames (can't delete while iterating)
  String filesToDelete[100];
  int fileCount = 0;

  File file = root.openNextFile();
  while (file && fileCount < 100) {
    String name = file.name();
    // Delete both CSV and UBX files
    if ((name.startsWith("log_") && name.endsWith(".csv")) ||
        (name.startsWith("raw_") && name.endsWith(".ubx"))) {
      filesToDelete[fileCount++] = "/" + name;
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();

  // Second pass: delete files
  for (int i = 0; i < fileCount; i++) {
    if (SD_MMC.remove(filesToDelete[i].c_str())) {
      deletedCount++;
      Serial.print("[LOG] Deleted: ");
      Serial.println(filesToDelete[i]);
    }
  }

  bleSend("OK:DELETE_ALL\n");

  Serial.print("[LOG] Deleted ");
  Serial.print(deletedCount);
  Serial.println(" files");

  // Create new log files
  createNewLogFile();
}


// ========== IMU LEVEL CALIBRATION ==========
void calibrateIMULevel() {
  if (!imuAvailable) {
    Serial.println("[IMU] Not available for calibration");
    return;
  }

  // Read current raw values
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(ADXL345_DATAX0);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, (uint8_t)6);

  if (Wire.available() >= 6) {
    uint8_t x0 = Wire.read();
    uint8_t x1 = Wire.read();
    uint8_t y0 = Wire.read();
    uint8_t y1 = Wire.read();
    uint8_t z0 = Wire.read();
    uint8_t z1 = Wire.read();

    int16_t rawX = x0 | (x1 << 8);
    int16_t rawY = y0 | (y1 << 8);
    int16_t rawZ = z0 | (z1 << 8);

    // Update global offsets
    // For level: X and Y should read 0, Z should read 256 (1g)
    imuXOffset = rawX;       // Current X becomes the zero point
    imuYOffset = rawY;       // Current Y becomes the zero point
    imuZOffset = rawZ - 256; // Z should read 1g (256 LSB) when level

    Serial.print("[IMU] Calibrated - X:");
    Serial.print(rawX);
    Serial.print(" Y:");
    Serial.print(rawY);
    Serial.print(" Z:");
    Serial.println(rawZ);

    // Save calibration to flash
    saveIMUCalibration();
  }
}


// ========== IMU CALIBRATION PERSISTENCE ==========
void loadIMUCalibration() {
  prefs.begin("imu", true);  // Read-only mode

  // Check if calibration exists
  if (prefs.isKey("xOffset")) {
    imuXOffset = prefs.getShort("xOffset", -49);
    imuYOffset = prefs.getShort("yOffset", -9);
    imuZOffset = prefs.getShort("zOffset", 1453);

    Serial.println("[IMU] Loaded calibration from flash:");
    Serial.print("  X: "); Serial.print(imuXOffset);
    Serial.print("  Y: "); Serial.print(imuYOffset);
    Serial.print("  Z: "); Serial.println(imuZOffset);
  } else {
    Serial.println("[IMU] No saved calibration, using defaults");
  }

  prefs.end();
}

void saveIMUCalibration() {
  prefs.begin("imu", false);  // Read-write mode

  prefs.putShort("xOffset", imuXOffset);
  prefs.putShort("yOffset", imuYOffset);
  prefs.putShort("zOffset", imuZOffset);

  prefs.end();

  Serial.println("[IMU] Calibration saved to flash");
}
