/*
 * TTGO LoRa32 V2.0 - B-BOX PPK Logger
 * Logs Deeper sonar depth + GPS position for PPK post-processing
 * No real-time RTK - uses PPK correction in desktop app
 *
 * Hardware: TTGO LoRa32 V2.0 (ESP32 + SX1276 + OLED)
 *
 * Connections:
 *   TTGO IO33 (RX) <- M8P TX  - NMEA/UBX position data
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
#define GPS_RX_PIN   33   // TTGO RX <- M8P TX - NMEA out
#define GPS_TX_PIN   25   // TTGO TX -> M8P RX - for config commands
#define GPS_BAUD     115200  // High speed for 10Hz PPK

// ========== LORA CONFIGURATION (MUST MATCH TRANSMITTER) ==========
#define LORA_FREQUENCY     915E6
#define LORA_BANDWIDTH     125E3
#define LORA_SPREAD_FACTOR 7        // SF7 - DO NOT CHANGE
#define LORA_CODING_RATE   5        // 4/5 - DO NOT CHANGE
#define LORA_SYNC_WORD     0x12
#define LORA_TX_POWER      20

// ========== LORA TELEMETRY (Status TX to Shore Station) ==========
#define STATUS_PACKET_TYPE 0xBB     // Magic byte to identify status packets
#define COMMAND_PACKET_TYPE 0xCC   // Magic byte for commands from Shore Station
#define LORA_STATUS_INTERVAL_MS 67 // Send status at 15 Hz (with yield to RTCM)
bool loraTelemEnabled = true;       // Enable LoRa status telemetry by default
uint32_t lastLoraStatusTx = 0;      // Last status TX time
uint32_t loraStatusTxCount = 0;     // Count of status packets sent
uint32_t lastLoraRx = 0;            // Last LoRa packet receive time (for yield logic)
#define LORA_YIELD_MS 50            // Don't TX status within 50ms of receiving RTCM


// ========== BUFFER SIZES ==========
#define NMEA_BUFFER_SIZE   256
#define UDP_BUFFER_SIZE    512

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
#define UBX_CLASS_NAV   0x01
#define UBX_ID_PVT      0x07        // Position, velocity, time solution
#define UBX_ID_RTCM     0x32        // RTCM input status
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

// ========== M8P CONFIGURATION FOR PPK + RTK ==========
// RTCM reception tracking (from UBX-RXM-RTCM)
uint32_t rtcmMsgReceived = 0;       // Count of RTCM messages received
uint32_t rtcmLastReceived = 0;      // Last RTCM reception time
bool rtcmCrcFailed = false;         // Last RTCM CRC status
uint16_t rtcmLastMsgType = 0;       // Last RTCM message type
uint16_t rtcmLastRefStation = 0;    // Last reference station ID

// RTCM via BLE tracking
uint32_t rtcmBleReceived = 0;       // Count of RTCM bytes received via BLE
uint32_t rtcmBleForwarded = 0;      // Count of RTCM bytes forwarded to GPS
uint32_t lastRtcmBleDebug = 0;      // Last debug print time

void configureM8P() {
  Serial.println("[GPS] Configuring M8P for PPK + RTK...");
  delay(100);

  // ========== RTCM INPUT CONFIGURATION ==========

  // UBX-CFG-PRT: Configure UART1 for RTCM3 input
  // inProtoMask = UBX(0x01) + RTCM3(0x20) = 0x21
  // outProtoMask = UBX(0x01) + NMEA(0x02) = 0x03
  // Baud = 115200, Mode = 8N1
  uint8_t cfgPrtUart1[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01,                   // portID = UART1
    0x00,                   // reserved0
    0x00, 0x00,             // txReady (disabled)
    0xD0, 0x08, 0x00, 0x00, // mode = 8N1
    0x00, 0xC2, 0x01, 0x00, // baudRate = 115200
    0x21, 0x00,             // inProtoMask = UBX + RTCM3
    0x03, 0x00,             // outProtoMask = UBX + NMEA
    0x00, 0x00,             // flags
    0x00, 0x00,             // reserved1
    0xDA, 0x4E              // checksum
  };
  GPSSerial.write(cfgPrtUart1, sizeof(cfgPrtUart1));
  delay(100);  // Longer delay after port config
  Serial.println("[GPS] UART1 configured for RTCM3 input");

  // UBX-CFG-TMODE3: Set mode=0 (Disabled) for rover operation
  // This ensures the M8P acts as a rover, not a base station
  uint8_t cfgTmode3[] = {
    0xB5, 0x62, 0x06, 0x71, 0x28, 0x00,
    0x00,                   // version
    0x00,                   // reserved0
    0x00, 0x00,             // flags (mode = 0 = Disabled = Rover)
    0x00, 0x00, 0x00, 0x00, // ecefXOrLat
    0x00, 0x00, 0x00, 0x00, // ecefYOrLon
    0x00, 0x00, 0x00, 0x00, // ecefZOrAlt
    0x00, 0x00, 0x00, 0x00, // ecefXOrLatHP
    0x00, 0x00, 0x00, 0x00, // ecefYOrLonHP
    0x00, 0x00, 0x00, 0x00, // ecefZOrAltHP
    0x00, 0x00, 0x00, 0x00, // fixedPosAcc
    0x00, 0x00, 0x00, 0x00, // svinMinDur
    0x00, 0x00, 0x00, 0x00, // svinAccLimit
    0x00, 0x00, 0x00, 0x00, // reserved1
    0x9F, 0x93              // checksum
  };
  GPSSerial.write(cfgTmode3, sizeof(cfgTmode3));
  delay(50);
  Serial.println("[GPS] TMODE3 set to Rover mode");

  // UBX-CFG-DGNSS: RTK mode = 3 (float + fixed)
  uint8_t cfgDgnss[] = {
    0xB5, 0x62, 0x06, 0x70, 0x04, 0x00,
    0x03,                   // dgnssMode = RTK float + fixed
    0x00, 0x00, 0x00,       // reserved
    0x7D, 0x64              // checksum
  };
  GPSSerial.write(cfgDgnss, sizeof(cfgDgnss));
  delay(50);
  Serial.println("[GPS] DGNSS mode set to RTK float+fixed");

  // ========== PPK OUTPUT CONFIGURATION ==========

  // UBX-CFG-MSG: Enable UBX-RXM-RAWX on UART1 at 1Hz
  uint8_t enableRAWX[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x02, 0x15,             // RXM-RAWX
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x27, 0x47
  };
  GPSSerial.write(enableRAWX, sizeof(enableRAWX));
  delay(50);

  // UBX-CFG-MSG: Enable UBX-RXM-SFRBX on UART1 at 1Hz
  uint8_t enableSFRBX[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x02, 0x13,             // RXM-SFRBX
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x25, 0x3F
  };
  GPSSerial.write(enableSFRBX, sizeof(enableSFRBX));
  delay(50);

  // UBX-CFG-MSG: Enable UBX-NAV-PVT on UART1 at 1Hz
  uint8_t enablePVT[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x01, 0x07,             // NAV-PVT
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x18, 0xE1
  };
  GPSSerial.write(enablePVT, sizeof(enablePVT));
  delay(50);

  // ========== RTCM MONITORING ==========

  // UBX-CFG-MSG: Enable UBX-RXM-RTCM on UART1 for RTCM status
  uint8_t enableRxmRtcm[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x02, 0x32,             // RXM-RTCM
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x44, 0x16
  };
  GPSSerial.write(enableRxmRtcm, sizeof(enableRxmRtcm));
  delay(50);
  Serial.println("[GPS] RXM-RTCM monitoring enabled");

  // ========== SAVE CONFIGURATION TO FLASH ==========
  // UBX-CFG-CFG: Save all settings to BBR + Flash + EEPROM
  // This persists the configuration across power cycles
  uint8_t cfgSave[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00,   // clearMask (don't clear anything)
    0x1F, 0x1F, 0x00, 0x00,   // saveMask (save all sections)
    0x00, 0x00, 0x00, 0x00,   // loadMask (don't load)
    0x17,                     // deviceMask: BBR + Flash + EEPROM + SPI Flash
    0x71, 0xDF                // checksum
  };
  GPSSerial.write(cfgSave, sizeof(cfgSave));
  delay(500);  // Give time for flash write
  Serial.println("[GPS] Configuration saved to flash");

  Serial.println("[GPS] M8P configured for PPK (RAWX/SFRBX) + RTK (RTCM input)");
}

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
bool ppkLoggingEnabled = false;     // PPK raw logging starts OFF with recording
uint32_t ubxMessagesLogged = 0;
uint32_t ubxBytesWritten = 0;
uint32_t rawxMsgReceived = 0;       // Count of RAWX msgs received (for debug)
uint32_t lastRawxDebugPrint = 0;    // Last time we printed RAWX debug
char rawFileName[64];

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
int16_t lastRSSI = 0;
float lastSNR = 0;
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
bool recordingEnabled = false;  // Controlled via Bluetooth, starts OFF
char logFileName[64];
char currentProjectName[64] = "";  // Project name for file naming

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
void createNewLogFileWithName(const char* projectName);
void updateDisplay();
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
      // Check if this is RTCM data (starts with 0xD3 preamble)
      if (rxValue.length() >= 3 && (uint8_t)rxValue[0] == 0xD3) {
        // RTCM binary data - forward directly to GPS
        rtcmBleReceived += rxValue.length();
        GPSSerial.write((const uint8_t*)rxValue.data(), rxValue.length());
        rtcmBleForwarded += rxValue.length();

        // Debug output every 5 seconds
        if (millis() - lastRtcmBleDebug > 5000) {
          Serial.print("[BLE-RTCM] Forwarded ");
          Serial.print(rtcmBleForwarded);
          Serial.println(" bytes to GPS");
          lastRtcmBleDebug = millis();
        }
        return;
      }

      // Text command - add to buffer
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
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // RX and TX for config
  configureM8P();  // Enable RAWX output for PPK
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


  Serial.println();
  Serial.println("*** RECEIVER READY (BLE) ***");
  Serial.println();
}

// ========== MAIN LOOP ==========
void loop() {
  // 1. Handle WiFi connection state machine
  handleWiFi();

  // 3. Process UDP data from Deeper
  if (wifiState == WIFI_UDP_READY) {
    processUDP();
  }

  // 4. Read NMEA from RTK GPS (M8P)
  static uint32_t gpsRxBytes = 0;
  static uint32_t lastGpsDebug = 0;
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    gpsRxBytes++;
    processGPSChar(c);
  }
  // Debug: print GPS byte count every 5 seconds
  if (millis() - lastGpsDebug > 5000) {
    Serial.print("[GPS DEBUG] Bytes received: ");
    Serial.println(gpsRxBytes);
    lastGpsDebug = millis();
  }

  // 4b. Handle USB serial commands
  static char serialCmd[32];
  static int serialCmdIdx = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 10 || c == 13) {
      if (serialCmdIdx > 0) {
        serialCmd[serialCmdIdx] = 0;
        if (strcmp(serialCmd, "LOOPBACK") == 0) {
          Serial.println("[LOOPBACK] Starting test - connect GPIO32 to GPIO33");
          while (GPSSerial.available()) GPSSerial.read();
          uint8_t testPattern[] = {0xAA, 0x55, 0x12, 0x34, 0xD3};
          GPSSerial.write(testPattern, 5);
          GPSSerial.flush();
          Serial.println("[LOOPBACK] Sent: AA 55 12 34 D3");
          delay(100);
          int received = 0;
          uint8_t rxBuf[10];
          while (GPSSerial.available() && received < 10) {
            rxBuf[received++] = GPSSerial.read();
          }
          if (received == 0) {
            Serial.println("[LOOPBACK] FAIL - No data received!");
          } else {
            Serial.print("[LOOPBACK] Received: ");
            for (int i = 0; i < received; i++) {
              if (rxBuf[i] < 0x10) Serial.print("0");
              Serial.print(rxBuf[i], HEX);
              Serial.print(" ");
            }
            Serial.println();
            bool match = (received >= 5);
            for (int i = 0; i < 5 && match; i++) {
              if (rxBuf[i] != testPattern[i]) match = false;
            }
            Serial.println(match ? "[LOOPBACK] PASS!" : "[LOOPBACK] FAIL - mismatch!");
          }
        }
        serialCmdIdx = 0;
      }
    } else if (serialCmdIdx < 31) {
      serialCmd[serialCmdIdx++] = c;
    }
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

  // 13. Send LoRa telemetry status to Shore Station
  if (loraTelemEnabled) {
    sendLoRaStatus();
  }

  yield();
}

// ========== BLE SETUP ==========
void setupBLE() {
  Serial.print("[INIT] BLE... ");

  NimBLEDevice::init("BBOX");

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
  pAdvertising->setName("BBOX");  // Include name in advertisement
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();

  Serial.println("OK - Name: BBOX");
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
  // Don't create log file on startup - wait for START command
  Serial.println("[SD] Ready - waiting for START command to begin recording");
}

// ========== CREATE NEW LOG FILE ==========
void createNewLogFile() {
  // Use current project name if set, otherwise use numbered naming
  createNewLogFileWithName(currentProjectName[0] ? currentProjectName : NULL);
}

void createNewLogFileWithName(const char* projectName) {
  if (!sdCardReady) return;

  if (projectName && projectName[0]) {
    // Use project name for files - using String for safety
    String baseName = "/";
    baseName += projectName;
    String csvName = baseName + ".csv";
    String ubxName = baseName + ".ubx";

    // If files already exist, append a number to avoid overwriting
    int suffix = 1;
    strncpy(logFileName, csvName.c_str(), sizeof(logFileName) - 1);
    logFileName[sizeof(logFileName) - 1] = '\0';
    while (SD_MMC.exists(logFileName) && suffix < 100) {
      csvName = baseName + "_" + String(suffix) + ".csv";
      ubxName = baseName + "_" + String(suffix) + ".ubx";
      strncpy(logFileName, csvName.c_str(), sizeof(logFileName) - 1);
      logFileName[sizeof(logFileName) - 1] = '\0';
      suffix++;
    }
    // Copy final names
    strncpy(logFileName, csvName.c_str(), sizeof(logFileName) - 1);
    logFileName[sizeof(logFileName) - 1] = '\0';
    strncpy(rawFileName, ubxName.c_str(), sizeof(rawFileName) - 1);
    rawFileName[sizeof(rawFileName) - 1] = '\0';

    Serial.print("[SD] Using project name: ");
    Serial.println(projectName);
    Serial.print("[SD] CSV file will be: ");
    Serial.println(logFileName);
    Serial.print("[SD] UBX file will be: ");
    Serial.println(rawFileName);
  } else {
    // Find next available file number (legacy numbered naming)
    int fileNum = 0;
    do {
      snprintf(logFileName, sizeof(logFileName), "/log_%04d.csv", fileNum++);
    } while (SD_MMC.exists(logFileName) && fileNum < 10000);
    fileNum--;  // Back to the number we're using
    snprintf(rawFileName, sizeof(rawFileName), "/raw_%04d.ubx", fileNum);
  }

  logFile = SD_MMC.open(logFileName, FILE_WRITE);
  if (logFile) {
    // Write CSV header
    logFile.println("timestamp_ms,utc_time,gps_fix,lat,lon,alt_m,hdop,sats,depth_m,water_temp_c,rssi,snr,pitch,roll");
    logFile.flush();

    Serial.print("[SD] Logging to: ");
    Serial.println(logFileName);

    // Create matching UBX raw file for PPK
    Serial.print("[SD] rawFileName = '");
    Serial.print(rawFileName);
    Serial.print("' (len=");
    Serial.print(strlen(rawFileName));
    Serial.println(")");
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

  // Row 3: Altitude
  display.print("Alt:");
  display.print(gps.altitude, 1);
  display.print("m");
  display.println();

  // Row 4: LoRa status TX / RSSI
  display.print("TX:");
  display.print(loraStatusTxCount);
  display.print(" RSSI:");
  display.println(lastRSSI);

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

  // Row 6: SD status / PPK status
  display.print("SD:");
  if (sdCardReady) {
    display.print(logEntriesWritten);
  } else {
    display.print("NO");
  }
  display.print(" PPK:");
  display.print(ubxMessagesLogged);
  display.println();

  // Row 7: RTCM status (for RTK)
  display.print("RTCM:");
  // Show BLE RTCM if receiving, otherwise show LoRa RTCM
  if (rtcmBleForwarded > 0) {
    display.print(rtcmBleForwarded / 1000);  // KB received
    display.print("KB");
  } else {
    display.print(rtcmMsgReceived);
  }
  // Show age of last RTCM
  uint32_t rtcmAge = (millis() - rtcmLastReceived) / 1000;
  if (rtcmLastReceived > 0 && rtcmAge < 999) {
    display.print(" ");
    display.print(rtcmAge);
    display.print("s");
  }

  display.display();
}

// ========== STATS REPORT ==========
void printStatsReport() {
  uint32_t uptime = (millis() - startTime) / 1000;

  Serial.println();
  Serial.println("========================================");
  Serial.println("       B-BOX PPK STATUS REPORT");
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

  Serial.println("--- LORA TELEMETRY ---");
  Serial.print("  Status TX Count: "); Serial.println(loraStatusTxCount);
  Serial.print("  RSSI: "); Serial.print(lastRSSI); Serial.println(" dBm");
  Serial.print("  SNR: "); Serial.print(lastSNR, 1); Serial.println(" dB");
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
  // Parse NAV-PVT for position data (replaces NMEA parsing)
  if (ubxClass == UBX_CLASS_NAV && ubxId == UBX_ID_PVT && ubxPayloadLen >= 92) {
    uint8_t* payload = &ubxBuffer[6];  // Skip header

    // Extract time (bytes 0-5)
    uint16_t year = payload[4] | (payload[5] << 8);
    uint8_t month = payload[6];
    uint8_t day = payload[7];
    gps.utcHours = payload[8];
    gps.utcMinutes = payload[9];
    gps.utcSeconds = payload[10];
    int32_t nano = payload[16] | (payload[17] << 8) | (payload[18] << 16) | (payload[19] << 24);
    gps.utcMillis = (nano / 1000000) % 1000;
    if (gps.utcMillis < 0) gps.utcMillis = 0;
    gps.timeValid = (payload[11] & 0x03) == 0x03;  // validDate and validTime

    // Fix type (byte 20)
    uint8_t fixType = payload[20];
    uint8_t flags = payload[21];
    bool gnssFixOK = (flags & 0x01) != 0;
    uint8_t carrSoln = (flags >> 6) & 0x03;  // RTK status: 0=none, 1=float, 2=fixed

    // Convert to NMEA fix quality
    if (!gnssFixOK || fixType < 2) {
      gps.fixQuality = 0;  // No fix
    } else if (carrSoln == 2) {
      gps.fixQuality = 4;  // RTK Fixed
    } else if (carrSoln == 1) {
      gps.fixQuality = 5;  // RTK Float
    } else if (fixType == 2) {
      gps.fixQuality = 2;  // DGPS (2D fix)
    } else {
      gps.fixQuality = 1;  // GPS (3D fix)
    }

    // Satellites (byte 23)
    gps.satellites = payload[23];

    // Longitude (bytes 24-27) - 1e-7 degrees
    int32_t lon = payload[24] | (payload[25] << 8) | (payload[26] << 16) | (payload[27] << 24);
    gps.longitude = lon * 1e-7;

    // Latitude (bytes 28-31) - 1e-7 degrees
    int32_t lat = payload[28] | (payload[29] << 8) | (payload[30] << 16) | (payload[31] << 24);
    gps.latitude = lat * 1e-7;

    // Altitude (bytes 36-39) - mm above ellipsoid
    int32_t alt = payload[36] | (payload[37] << 8) | (payload[38] << 16) | (payload[39] << 24);
    gps.altitude = alt / 1000.0f;

    // PDOP (bytes 76-77) - 0.01 scale
    uint16_t pDOP = payload[76] | (payload[77] << 8);
    gps.hdop = pDOP / 100.0f;

    gps.valid = gnssFixOK && (gps.latitude != 0.0 || gps.longitude != 0.0);
    gps.lastUpdate = millis();

    return;  // Don't log NAV-PVT to raw file
  }

  // Parse RXM-RTCM for RTCM reception status
  if (ubxClass == UBX_CLASS_RXM && ubxId == UBX_ID_RTCM && ubxPayloadLen >= 8) {
    uint8_t* payload = &ubxBuffer[6];
    uint8_t flags = payload[1];
    rtcmCrcFailed = (flags & 0x01) != 0;
    uint8_t msgUsed = (flags >> 1) & 0x03;  // 0=unknown, 1=failed, 2=used
    rtcmLastRefStation = payload[4] | (payload[5] << 8);
    rtcmLastMsgType = payload[6] | (payload[7] << 8);

    if (!rtcmCrcFailed) {
      rtcmMsgReceived++;
      rtcmLastReceived = millis();
    }

    // Debug output every 10 seconds
    static uint32_t lastRtcmDebug = 0;
    if (millis() - lastRtcmDebug > 10000) {
      Serial.print("[RTCM] Received ");
      Serial.print(rtcmMsgReceived);
      Serial.print(" msgs, last type ");
      Serial.print(rtcmLastMsgType);
      Serial.print(" from station ");
      Serial.println(rtcmLastRefStation);
      lastRtcmDebug = millis();
    }
    return;  // Don't log RXM-RTCM to raw file
  }

  // Log RAWX and SFRBX messages to raw file (needed for PPK)
  if (ubxClass == UBX_CLASS_RXM && (ubxId == UBX_ID_RAWX || ubxId == UBX_ID_SFRBX)) {
    rawxMsgReceived++;
    // Debug: print every 10 seconds to confirm RAWX reception
    if (millis() - lastRawxDebugPrint > 10000) {
      Serial.print("[RAWX] Received ");
      Serial.print(rawxMsgReceived);
      Serial.print(" msgs, logged ");
      Serial.println(ubxMessagesLogged);
      lastRawxDebugPrint = millis();
    }

    if (rawFile && ppkLoggingEnabled) {
      size_t totalLen = 6 + ubxPayloadLen + 2;
      rawFile.write(ubxBuffer, totalLen);
      ubxBytesWritten += totalLen;
      ubxMessagesLogged++;
    }
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
  // Debug: print first 20 chars of each NMEA sentence
  static uint32_t lastNmeaDebug = 0;
  if (millis() - lastNmeaDebug > 2000) {
    Serial.print("[NMEA] ");
    char temp[25];
    strncpy(temp, nmea, 24);
    temp[24] = 0;
    Serial.println(temp);
    lastNmeaDebug = millis();
  }

  if (strstr(nmea, "GGA") != NULL) {
    Serial.print("[GGA FOUND] ");
    Serial.println(nmea);
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

// ========== LORA TELEMETRY TX ==========
void sendLoRaStatus() {
  if (!loraTelemEnabled) return;
  if (millis() - lastLoraStatusTx < LORA_STATUS_INTERVAL_MS) return;

  // Yield to incoming packets - don't transmit if we recently received data
  if (millis() - lastLoraRx < LORA_YIELD_MS) return;

  // Make local copies to avoid race conditions with GPS parsing
  // (GPS NMEA parsing can update struct while we're building status string)
  int localFixQuality = gps.fixQuality;
  int localSatellites = gps.satellites;
  double localLatitude = gps.latitude;
  double localLongitude = gps.longitude;
  bool localGpsValid = gps.valid;

  // Sanity check - discard obviously corrupt values
  if (localFixQuality < 0 || localFixQuality > 6 || localSatellites < 0 || localSatellites > 50) {
    localFixQuality = 0;
    localSatellites = 0;
    localGpsValid = false;
  }

  // Format status string (same as BLE format)
  String status = String(localFixQuality);
  status += ",";
  status += String(localSatellites);
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
  if (imu.valid) {
    status += String(imu.pitch, 1);
    status += ",";
    status += String(imu.roll, 1);
  } else {
    status += "0,0";
  }
  status += ",";
  // Validate GPS: need fix, 4+ sats, both coords non-zero and in valid ranges
  bool loraGpsValid = localGpsValid &&
                      localSatellites >= 4 &&
                      localLatitude != 0.0 && localLongitude != 0.0 &&
                      localLatitude > -90.0 && localLatitude < 90.0 &&
                      localLongitude > -180.0 && localLongitude < 180.0;
  if (loraGpsValid) {
    status += String(localLatitude, 7);
    status += ",";
    status += String(localLongitude, 7);
  } else {
    status += "0,0";
  }

  // Send via LoRa with magic byte prefix
  LoRa.beginPacket();
  LoRa.write(STATUS_PACKET_TYPE);  // Magic byte 0xBB
  LoRa.print(status);
  LoRa.endPacket();

  lastLoraStatusTx = millis();
  loraStatusTxCount++;

  Serial.print("[LORA TX] Status sent, count: ");
  Serial.println(loraStatusTxCount);
}

// ========== BLUETOOTH FUNCTIONS ==========
uint32_t lastBtUpdate = 0;

void sendBluetoothStatus() {
  if (!bleConnected) return;

  // Make local copies to avoid race conditions with GPS parsing
  // (GPS NMEA parsing can update struct while we're building status string)
  int localFixQuality = gps.fixQuality;
  int localSatellites = gps.satellites;
  double localLatitude = gps.latitude;
  double localLongitude = gps.longitude;
  bool localGpsValid = gps.valid;

  // Sanity check - discard obviously corrupt values
  if (localFixQuality < 0 || localFixQuality > 6 || localSatellites < 0 || localSatellites > 50) {
    localFixQuality = 0;
    localSatellites = 0;
    localGpsValid = false;
  }

  // Format: FIX,SATS,DEPTH,TEMP,BATT,REC,PITCH,ROLL,LAT,LON
  // Example: 4,12,1.25,18.5,85,1,5.2,-3.1,54.3742363,-126.7221132
  String status = String(localFixQuality);
  status += ",";
  status += String(localSatellites);
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
  // Validate GPS: need fix, 4+ sats, both coords non-zero and in valid ranges
  bool gpsValid = localGpsValid &&
                  localSatellites >= 4 &&
                  localLatitude != 0.0 && localLongitude != 0.0 &&
                  localLatitude > -90.0 && localLatitude < 90.0 &&
                  localLongitude > -180.0 && localLongitude < 180.0;
  if (gpsValid) {
    status += String(localLatitude, 7);
    status += ",";
    status += String(localLongitude, 7);
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

    if (cmd == "START" || cmd.startsWith("START:")) {
      // Extract project name if provided (START:ProjectName)
      if (cmd.startsWith("START:")) {
        String projName = cmd.substring(6);
        projName.trim();
        // Sanitize: remove invalid filename chars
        for (int i = 0; i < projName.length(); i++) {
          char c = projName.charAt(i);
          if (c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' || c == '"' || c == '<' || c == '>' || c == '|') {
            projName.setCharAt(i, '_');
          }
        }
        strncpy(currentProjectName, projName.c_str(), sizeof(currentProjectName) - 1);
        currentProjectName[sizeof(currentProjectName) - 1] = '\0';
        Serial.print("[BLE] Project name: ");
        Serial.println(currentProjectName);
      } else {
        currentProjectName[0] = '\0';  // No project name
      }

      // Create new log files if not already open
      if (!logFile) {
        createNewLogFile();
      }
      recordingEnabled = true;
      ppkLoggingEnabled = true;  // Enable PPK logging with recording
      bleSend("OK:REC_ON\n");
      Serial.println("[BLE] Recording started (CSV + PPK)");
    } else if (cmd == "STOP") {
      recordingEnabled = false;
      ppkLoggingEnabled = false;  // Stop PPK logging with recording
      // Flush and close files
      if (logFile) {
        logFile.flush();
        logFile.close();
        logFile = File();  // Reset to invalid state for next START
        Serial.println("[SD] Log file closed");
      }
      if (rawFile) {
        rawFile.flush();
        rawFile.close();
        rawFile = File();  // Reset to invalid state for next START
        Serial.println("[SD] PPK file closed");
      }
      bleSend("OK:REC_OFF\n");
      Serial.println("[BLE] Recording stopped (files closed)");
    } else if (cmd == "RESTART_LOG" || cmd.startsWith("RESTART_LOG:")) {
      // Force restart logging - closes any stuck files and reopens fresh
      Serial.println("[BLE] RESTART_LOG - forcing fresh start");

      // Extract project name if provided
      if (cmd.startsWith("RESTART_LOG:")) {
        String projName = cmd.substring(12);
        projName.trim();
        for (int i = 0; i < projName.length(); i++) {
          char c = projName.charAt(i);
          if (c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' || c == '"' || c == '<' || c == '>' || c == '|') {
            projName.setCharAt(i, '_');
          }
        }
        strncpy(currentProjectName, projName.c_str(), sizeof(currentProjectName) - 1);
        currentProjectName[sizeof(currentProjectName) - 1] = '\0';
      }

      // Close existing files
      if (logFile) {
        logFile.flush();
        logFile.close();
        logFile = File();
      }
      if (rawFile) {
        rawFile.flush();
        rawFile.close();
        rawFile = File();
      }
      // Create new files
      createNewLogFile();
      recordingEnabled = true;
      ppkLoggingEnabled = true;
      bleSend("OK:RESTART_LOG\n");
      Serial.println("[BLE] Recording restarted with fresh files");
    } else if (cmd == "REC_STATUS") {
      // Detailed recording status - helps app detect stagnant state
      char statusBuf[64];
      snprintf(statusBuf, sizeof(statusBuf), "REC:%d,FILES:%d,%d\n",
               recordingEnabled ? 1 : 0,
               logFile ? 1 : 0,
               rawFile ? 1 : 0);
      bleSend(statusBuf);
    } else if (cmd == "STATUS") {
      sendBluetoothStatus();
    } else if (cmd == "LOOPBACK") {
      // Loopback test: send bytes on TX and check if received on RX
      // User should connect GPIO32 to GPIO33 temporarily
      Serial.println("[LOOPBACK] Starting test - connect GPIO32 to GPIO33");

      // Clear any pending RX data
      while (GPSSerial.available()) GPSSerial.read();

      // Send test pattern
      uint8_t testPattern[] = {0xAA, 0x55, 0x12, 0x34, 0xD3};
      GPSSerial.write(testPattern, 5);
      GPSSerial.flush();
      Serial.println("[LOOPBACK] Sent: AA 55 12 34 D3");

      // Wait for data to come back
      delay(100);

      // Read back
      int received = 0;
      uint8_t rxBuf[10];
      while (GPSSerial.available() && received < 10) {
        rxBuf[received++] = GPSSerial.read();
      }

      if (received == 0) {
        Serial.println("[LOOPBACK] FAIL - No data received! TX not working or not connected.");
        bleSend("LOOPBACK:FAIL,NO_DATA");
      } else {
        Serial.print("[LOOPBACK] Received ");
        Serial.print(received);
        Serial.print(" bytes: ");
        for (int i = 0; i < received; i++) {
          if (rxBuf[i] < 0x10) Serial.print("0");
          Serial.print(rxBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // Check if matches
        bool match = (received >= 5);
        for (int i = 0; i < 5 && match; i++) {
          if (rxBuf[i] != testPattern[i]) match = false;
        }

        if (match) {
          Serial.println("[LOOPBACK] PASS - TX is working!");
          bleSend("LOOPBACK:PASS");
        } else {
          Serial.println("[LOOPBACK] FAIL - Data mismatch!");
          bleSend("LOOPBACK:FAIL,MISMATCH");
        }
      }
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
    } else if (cmd == "LORA_ON") {
      loraTelemEnabled = true;
      bleSend("OK:LORA_ON\n");
      Serial.println("[BLE] LoRa telemetry enabled");
    } else if (cmd == "LORA_OFF") {
      loraTelemEnabled = false;
      bleSend("OK:LORA_OFF\n");
      Serial.println("[BLE] LoRa telemetry disabled");
    } else if (cmd == "LORA_STATUS") {
      char buf[64];
      snprintf(buf, sizeof(buf), "LORA:%s,TX:%lu\n",
               loraTelemEnabled ? "ON" : "OFF",
               loraStatusTxCount);
      bleSend(buf);
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

  // Build response string
  String response = "LOGS:";
  bool first = true;
  int fileCount = 0;

  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    String nameLower = name;
    nameLower.toLowerCase();
    // Include ALL .csv and .ubx files (case-insensitive)
    bool isCsvFile = nameLower.endsWith(".csv");
    bool isUbxFile = nameLower.endsWith(".ubx");
    if (isCsvFile || isUbxFile) {
      if (!first) response += ";";
      first = false;
      response += name + "," + String(file.size()) + "," + String(file.getLastWrite());
      fileCount++;
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();

  if (fileCount == 0) {
    bleSend("LOGS:EMPTY\n");
    return;
  }

  response += "\n";

  // Send response in chunks (BLE MTU is limited)
  const int chunkSize = 180;
  int offset = 0;

  while (offset < response.length()) {
    String chunk = response.substring(offset, min((int)response.length(), offset + chunkSize));
    bleSend(chunk);
    delay(30);
    offset += chunkSize;
  }
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
    String nameLower = name;
    nameLower.toLowerCase();
    // Delete all CSV and UBX files (supports both legacy and project naming)
    if (nameLower.endsWith(".csv") || nameLower.endsWith(".ubx")) {
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

