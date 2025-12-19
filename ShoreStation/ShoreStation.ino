/*
 * TTGO LoRa32 V2.0 - Shore Station (Bidirectional)
 *
 * Combines RTCM relay with status reception from B-Box:
 * - Connects to NTRIP caster via WiFi
 * - Transmits RTCM corrections to B-Box via LoRa
 * - Receives status telemetry from B-Box via LoRa
 * - Forwards status to phone via BLE (Nordic UART Service)
 *
 * Time-Division Protocol:
 * - TX: Send RTCM burst (~200-300ms)
 * - RX: Listen for B-Box status (~300ms)
 * - Cycle: ~1 Hz for both directions
 *
 * Hardware: TTGO LoRa32 V2.0 (ESP32 + SX1276 + OLED)
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <base64.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>

// ========== PIN DEFINITIONS (TTGO LoRa32 V2.0) ==========
#define LORA_SCK     5
#define LORA_MISO    19
#define LORA_MOSI    27
#define LORA_CS      18
#define LORA_RST     14
#define LORA_DIO0    26

#define OLED_SDA     21
#define OLED_SCL     22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// ========== NTRIP CONFIGURATION ==========
const char* WIFI_SSID     = "LiBase-LB1U02096";
const char* WIFI_PASSWORD = "12345678";
const char* NTRIP_HOST    = "192.168.1.8";
const int   NTRIP_PORT    = 12345;
const char* NTRIP_MOUNT   = "LB1U02096";
const char* NTRIP_USER    = "chris";
const char* NTRIP_PASS    = "chris";

// ========== LORA CONFIGURATION (MUST MATCH B-BOX) ==========
#define LORA_FREQUENCY     915E6
#define LORA_BANDWIDTH     125E3
#define LORA_SPREAD_FACTOR 7        // SF7 - DO NOT CHANGE
#define LORA_CODING_RATE   5        // 4/5 - DO NOT CHANGE
#define LORA_SYNC_WORD     0x12
#define LORA_TX_POWER      20

// ========== PROTOCOL CONFIGURATION ==========
#define RTCM_BUFFER_SIZE   1024
#define MAX_FRAGMENT_SIZE  248
#define STATUS_LISTEN_MS   400      // Time to listen for B-Box status after RTCM TX
#define STATUS_PACKET_TYPE 0xBB     // Magic byte to identify status packets

// ========== BLE CONFIGURATION ==========
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9F"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ========== OBJECTS ==========
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WiFiClient ntripClient;
NimBLEServer *pServer = NULL;
NimBLECharacteristic *pTxCharacteristic = NULL;

// ========== BLE STATE ==========
bool bleConnected = false;
uint32_t bleConnectTime = 0;
String bleRxBuffer = "";

// ========== RTCM STATE ==========
uint8_t rtcmBuffer[RTCM_BUFFER_SIZE];
uint16_t rtcmIndex = 0;
uint16_t rtcmLength = 0;
uint8_t rtcmState = 0;

// ========== B-BOX STATUS (received via LoRa) ==========
struct BBoxStatus {
  uint8_t fixQuality;
  uint8_t satellites;
  float depth;
  float waterTemp;
  int8_t battery;
  bool recording;
  float pitch;
  float roll;
  double latitude;
  double longitude;
  int16_t rssi;
  float snr;
  uint32_t lastUpdate;
  bool valid;
} bbox = {0, 0, 0, 0, -1, false, 0, 0, 0, 0, 0, 0, 0, false};

// ========== STATISTICS ==========
uint32_t packetsTransmitted = 0;
uint32_t rtcmMessagesProcessed = 0;
uint32_t statusPacketsReceived = 0;
uint32_t lastDisplayUpdate = 0;
uint32_t lastNtripCheck = 0;
uint32_t lastRtcmTxTime = 0;
bool wifiConnected = false;
bool ntripConnected = false;

// ========== TIMING ==========
bool inRxMode = false;
uint32_t rxModeStartTime = 0;

// ========== CRC-24Q ==========
const uint32_t CRC24Q_POLY = 0x1864CFB;

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
  uint16_t msgLen = ((data[1] & 0x03) << 8) | data[2];
  if (length != msgLen + 6) return false;

  uint32_t calcCRC = crc24q(data, length - 3);
  uint32_t msgCRC = ((uint32_t)data[length-3] << 16) |
                    ((uint32_t)data[length-2] << 8) |
                    data[length-1];
  return calcCRC == msgCRC;
}

// ========== BLE CALLBACKS ==========
class MyServerCallbacks: public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    bleConnected = true;
    bleConnectTime = millis();
    Serial.print("[BLE] Phone connected from: ");
    Serial.println(connInfo.getAddress().toString().c_str());
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    bleConnected = false;
    Serial.print("[BLE] Phone disconnected, reason: ");
    Serial.println(reason);
    NimBLEDevice::startAdvertising();
  }
};

class MyCallbacks: public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      bleRxBuffer += rxValue.c_str();
    }
  }
};

// ========== BLE SETUP ==========
void setupBLE() {
  Serial.print("[INIT] BLE... ");

  NimBLEDevice::init("DeeperRTK-Shore");

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    NIMBLE_PROPERTY::NOTIFY
  );

  NimBLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  
  // Set scan response data with device name
  NimBLEAdvertisementData scanResponse;
  scanResponse.setName("DeeperRTK-Shore");
  pAdvertising->setScanResponseData(scanResponse);
  
  pAdvertising->start();

  Serial.println("OK - Name: DeeperRTK-Shore");
}

// ========== BLE SEND ==========
void bleSend(const String& data) {
  if (bleConnected && pTxCharacteristic && pServer) {
    if (millis() - bleConnectTime < 200) return;
    if (pServer->getConnectedCount() == 0) return;

    pTxCharacteristic->setValue(data.c_str());
    pTxCharacteristic->notify();
  }
}

// ========== WIFI ==========
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    wifiConnected = false;
    Serial.println("\nWiFi connection failed!");
  }
}

// ========== NTRIP ==========
void connectNTRIP() {
  if (!wifiConnected) return;

  Serial.print("Connecting to NTRIP: ");
  Serial.println(NTRIP_HOST);

  if (!ntripClient.connect(NTRIP_HOST, NTRIP_PORT)) {
    Serial.println("NTRIP connection failed!");
    ntripConnected = false;
    return;
  }

  String auth = String(NTRIP_USER) + ":" + String(NTRIP_PASS);
  String authBase64 = base64::encode(auth);

  String request = "GET /" + String(NTRIP_MOUNT) + " HTTP/1.1\r\n";
  request += "Host: " + String(NTRIP_HOST) + ":" + String(NTRIP_PORT) + "\r\n";
  request += "Ntrip-Version: Ntrip/2.0\r\n";
  request += "User-Agent: NTRIP ShoreStation/1.0\r\n";
  request += "Accept: */*\r\n";
  request += "Authorization: Basic " + authBase64 + "\r\n";
  request += "Connection: keep-alive\r\n\r\n";

  ntripClient.print(request);

  unsigned long timeout = millis() + 5000;
  while (ntripClient.available() == 0 && millis() < timeout) {
    delay(10);
  }

  if (ntripClient.available()) {
    String response = ntripClient.readStringUntil('\n');
    if (response.indexOf("200") > 0) {
      ntripConnected = true;
      Serial.println("NTRIP connected!");
      while (ntripClient.available()) {
        String line = ntripClient.readStringUntil('\n');
        if (line.length() <= 2) break;
      }
    } else {
      ntripConnected = false;
      Serial.println("NTRIP auth failed!");
    }
  } else {
    ntripConnected = false;
    Serial.println("NTRIP timeout!");
  }
}

// ========== LORA TRANSMIT RTCM ==========
void sendRTCMviaLoRa(uint8_t* data, uint16_t length) {
  uint16_t firstFragMax = MAX_FRAGMENT_SIZE - 2;
  uint8_t totalFragments = 1;
  if (length > firstFragMax) {
    totalFragments = 1 + ((length - firstFragMax + MAX_FRAGMENT_SIZE - 1) / MAX_FRAGMENT_SIZE);
  }

  uint16_t dataOffset = 0;
  uint32_t txTimestamp = millis();

  for (uint8_t frag = 0; frag < totalFragments; frag++) {
    LoRa.beginPacket();
    LoRa.write(frag);
    LoRa.write(totalFragments);

    if (frag == 0) {
      uint16_t ts = (uint16_t)(txTimestamp & 0xFFFF);
      LoRa.write(ts & 0xFF);
      LoRa.write((ts >> 8) & 0xFF);

      uint16_t fragLen = min((uint16_t)firstFragMax, length);
      LoRa.write(data, fragLen);
      dataOffset = fragLen;
    } else {
      uint16_t remaining = length - dataOffset;
      uint16_t fragLen = min((uint16_t)MAX_FRAGMENT_SIZE, remaining);
      LoRa.write(data + dataOffset, fragLen);
      dataOffset += fragLen;
    }

    LoRa.endPacket();
    packetsTransmitted++;
    yield();

    if (totalFragments > 1 && frag < totalFragments - 1) {
      delay(30);
    }
  }

  lastRtcmTxTime = millis();
}

// ========== PROCESS RTCM BYTE ==========
void processRTCMByte(uint8_t byte) {
  switch (rtcmState) {
    case 0:
      if (byte == 0xD3) {
        rtcmBuffer[0] = byte;
        rtcmIndex = 1;
        rtcmState = 1;
      }
      break;

    case 1:
      rtcmBuffer[rtcmIndex++] = byte;
      if (rtcmIndex == 3) {
        rtcmLength = ((rtcmBuffer[1] & 0x03) << 8) | rtcmBuffer[2];
        if (rtcmLength > RTCM_BUFFER_SIZE - 6) {
          rtcmState = 0;
        } else {
          rtcmState = 2;
        }
      }
      break;

    case 2:
      rtcmBuffer[rtcmIndex++] = byte;
      if (rtcmIndex == rtcmLength + 6) {
        if (validateRTCM(rtcmBuffer, rtcmIndex)) {
          uint16_t msgType = (rtcmBuffer[3] << 4) | (rtcmBuffer[4] >> 4);

          // Filter: GPS + GLONASS only
          bool shouldSend = false;
          if (msgType == 1005 || msgType == 1006) shouldSend = true;
          else if (msgType >= 1071 && msgType <= 1077) shouldSend = true;
          else if (msgType >= 1081 && msgType <= 1087) shouldSend = true;
          else if (msgType == 1230) shouldSend = true;

          if (shouldSend) {
            sendRTCMviaLoRa(rtcmBuffer, rtcmIndex);
            rtcmMessagesProcessed++;

            // After TX, switch to RX mode to listen for B-Box status
            inRxMode = true;
            rxModeStartTime = millis();
          }
        }
        rtcmState = 0;
      }
      break;
  }
}

// ========== RECEIVE B-BOX STATUS ==========
void checkForBBoxStatus() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    // Check for status packet magic byte
    uint8_t firstByte = LoRa.peek();

    if (firstByte == STATUS_PACKET_TYPE) {
      LoRa.read(); // Consume magic byte

      bbox.rssi = LoRa.packetRssi();
      bbox.snr = LoRa.packetSnr();

      // Read status data
      char statusBuf[128];
      int idx = 0;
      while (LoRa.available() && idx < 127) {
        statusBuf[idx++] = LoRa.read();
      }
      statusBuf[idx] = '\0';

      // Parse status: FIX,SATS,DEPTH,TEMP,BATT,REC,PITCH,ROLL,LAT,LON
      parseStatus(statusBuf);

      statusPacketsReceived++;
      bbox.lastUpdate = millis();
      bbox.valid = true;

      // Forward to phone via BLE
      if (bleConnected) {
        String bleMsg = String(statusBuf) + "\n";
        bleSend(bleMsg);
      }

      Serial.print("[RX] Status: ");
      Serial.println(statusBuf);
    }
  }
}

// ========== PARSE B-BOX STATUS ==========
void parseStatus(const char* status) {
  char temp[128];
  strncpy(temp, status, 127);
  temp[127] = '\0';

  char* token;
  int field = 0;

  token = strtok(temp, ",");
  while (token != NULL && field < 10) {
    switch (field) {
      case 0: bbox.fixQuality = atoi(token); break;
      case 1: bbox.satellites = atoi(token); break;
      case 2: bbox.depth = atof(token); break;
      case 3: bbox.waterTemp = atof(token); break;
      case 4: bbox.battery = atoi(token); break;
      case 5: bbox.recording = (atoi(token) == 1); break;
      case 6: bbox.pitch = atof(token); break;
      case 7: bbox.roll = atof(token); break;
      case 8: bbox.latitude = atof(token); break;
      case 9: bbox.longitude = atof(token); break;
    }
    field++;
    token = strtok(NULL, ",");
  }
}

// ========== HANDLE BLE COMMANDS ==========
void handleBLECommands() {
  while (bleRxBuffer.length() > 0) {
    int idx = bleRxBuffer.indexOf('\n');
    if (idx == -1) break;

    String cmd = bleRxBuffer.substring(0, idx);
    bleRxBuffer = bleRxBuffer.substring(idx + 1);
    cmd.trim();

    // Forward commands that should go to B-Box
    // These will be relayed in future version
    if (cmd == "STATUS") {
      // Send cached B-Box status
      if (bbox.valid) {
        String status = String(bbox.fixQuality) + "," +
                       String(bbox.satellites) + "," +
                       String(bbox.depth, 2) + "," +
                       String(bbox.waterTemp, 1) + "," +
                       String(bbox.battery) + "," +
                       String(bbox.recording ? 1 : 0) + "," +
                       String(bbox.pitch, 1) + "," +
                       String(bbox.roll, 1) + "," +
                       String(bbox.latitude, 7) + "," +
                       String(bbox.longitude, 7) + "\n";
        bleSend(status);
      }
    }
    else if (cmd == "START") {
      sendCommandViaLoRa("START");
      bleSend("OK:REC_ON\n");
    }
    else if (cmd == "STOP") {
      sendCommandViaLoRa("STOP");
      bleSend("OK:REC_OFF\n");
    }
    else if (cmd.startsWith("FREQ_")) {
      sendCommandViaLoRa(cmd.c_str());
      bleSend("OK:" + cmd + "\n");
    }
    else if (cmd.startsWith("LIST_") || cmd.startsWith("DELETE") || cmd.startsWith("DOWNLOAD")) {
      bleSend("ERR:RADIO_LINK\n");
    }
    else if (cmd == "PING") {
      bleSend("PONG\n");
    }
  }
}

// Send command to B-Box via LoRa
void sendCommandViaLoRa(const char* cmdStr) {
  LoRa.beginPacket();
  LoRa.write(0xCC);  // Command packet type
  LoRa.print(cmdStr);
  LoRa.endPacket();
  Serial.print("[CMD] Sent via LoRa: ");
  Serial.println(cmdStr);
}

// ========== DISPLAY ==========
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  // Row 1: Mode indicator
  display.println("== SHORE STATION ==");

  // Row 2: Connection status
  display.print("WiFi:");
  display.print(wifiConnected ? "OK " : "NO ");
  display.print("NTRIP:");
  display.println(ntripConnected ? "OK" : "NO");

  // Row 3: RTCM stats
  display.print("RTCM TX:");
  display.print(rtcmMessagesProcessed);
  display.print(" RX:");
  display.println(statusPacketsReceived);

  // Row 4: B-Box status (if recent)
  if (bbox.valid && (millis() - bbox.lastUpdate < 5000)) {
    display.print("Fix:");
    display.print(bbox.fixQuality);
    display.print(" D:");
    display.print(bbox.depth, 1);
    display.println("m");

    // Row 5: Signal
    display.print("R:");
    display.print(bbox.rssi);
    display.print(" S:");
    display.println(bbox.snr, 1);
  } else {
    display.println("B-Box: No signal");
    display.println("");
  }

  // Row 6: BLE status
  display.print("BLE:");
  display.println(bleConnected ? "Connected" : "Waiting...");

  display.display();
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("    SHORE STATION (Bidirectional)");
  Serial.println("========================================");
  Serial.println();

  // Init OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed!");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Shore Station");
  display.println("Starting...");
  display.display();

  // Init LoRa
  Serial.print("[INIT] LoRa... ");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("FAILED!");
    while (1) delay(1000);
  }

  LoRa.setSpreadingFactor(LORA_SPREAD_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();
  Serial.println("OK");

  // Init BLE
  setupBLE();

  // Connect WiFi
  connectWiFi();

  // Connect NTRIP
  if (wifiConnected) {
    connectNTRIP();
  }

  updateDisplay();
  Serial.println("*** SHORE STATION READY ***");
}

// ========== LOOP ==========
void loop() {
  // Check WiFi/NTRIP connections
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    ntripConnected = false;
  }

  if (millis() - lastNtripCheck > 10000) {
    lastNtripCheck = millis();
    if (!wifiConnected) connectWiFi();
    if (wifiConnected && !ntripConnected) connectNTRIP();
  }

  // Process NTRIP data (TX mode)
  if (ntripConnected && ntripClient.connected() && !inRxMode) {
    while (ntripClient.available()) {
      uint8_t byte = ntripClient.read();
      processRTCMByte(byte);
    }
  } else if (ntripConnected && !ntripClient.connected()) {
    ntripConnected = false;
  }

  // ALWAYS listen for B-Box status (not just after RTCM TX)
  // This ensures we receive status even when NTRIP isn't connected
  checkForBBoxStatus();

  // Exit RX mode after timeout (for time-division when RTCM is flowing)
  if (inRxMode && (millis() - rxModeStartTime > STATUS_LISTEN_MS)) {
    inRxMode = false;
  }

  // Handle BLE commands from phone
  handleBLECommands();

  // Update display
  if (millis() - lastDisplayUpdate > 250) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  yield();
}
