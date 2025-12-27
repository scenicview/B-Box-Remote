/*
 * Heltec LoRa32 V3 - Base Link (Bidirectional)
 *
 * Combines RTCM relay with status reception from B-Box:
 * - Connects to NTRIP caster via WiFi (LiBase)
 * - Transmits RTCM corrections to B-Box via LoRa
 * - Receives status telemetry from B-Box via LoRa
 * - Forwards status to phone via BLE (Nordic UART Service)
 *
 * Time-Division Protocol:
 * - TX: Send RTCM burst (~200-300ms)
 * - RX: Listen for B-Box status (~300ms)
 * - Cycle: ~1 Hz for both directions
 *
 * Hardware: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED)
 */

// Heltec library must be first
#include <heltec_unofficial.h>

#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <base64.h>
#include <NimBLEDevice.h>

// RadioLib interrupt flags
volatile bool loraRxFlag = false;
volatile bool loraTxDone = false;

// LoRa RX interrupt handler
void IRAM_ATTR onLoRaRx() {
  loraRxFlag = true;
}
// ========== LORA TX HELPER ==========
// Buffer for building LoRa packets
uint8_t loraTxBuffer[256];
int loraTxLen = 0;

void loraBeginPacket() {
  loraTxLen = 0;
}

void loraWrite(uint8_t b) {
  if (loraTxLen < 256) {
    loraTxBuffer[loraTxLen++] = b;
  }
}

void loraWrite(uint8_t* data, int len) {
  for (int i = 0; i < len && loraTxLen < 256; i++) {
    loraTxBuffer[loraTxLen++] = data[i];
  }
}

void loraPrint(const String& s) {
  for (size_t i = 0; i < s.length() && loraTxLen < 256; i++) {
    loraTxBuffer[loraTxLen++] = s[i];
  }
}

void loraEndPacket() {
  radio.transmit(loraTxBuffer, loraTxLen);
}

// ========== LORA RX HELPER ==========
uint8_t loraRxBuffer[256];
int loraRxLen = 0;
int loraRxIdx = 0;

int loraParsePacket() {
  if (!loraRxFlag) return 0;
  loraRxFlag = false;

  int16_t state = radio.readData(loraRxBuffer, 256);
  if (state != RADIOLIB_ERR_NONE) {
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    return 0;
  }

  loraRxLen = radio.getPacketLength();
  loraRxIdx = 0;
  return loraRxLen;
}

uint8_t loraPeek() {
  if (loraRxIdx < loraRxLen) {
    return loraRxBuffer[loraRxIdx];
  }
  return 0;
}

uint8_t loraRead() {
  if (loraRxIdx < loraRxLen) {
    return loraRxBuffer[loraRxIdx++];
  }
  return 0;
}

bool loraAvailable() {
  return loraRxIdx < loraRxLen;
}

int16_t loraPacketRssi() {
  return (int16_t)radio.getRSSI();
}

float loraPacketSnr() {
  return radio.getSNR();
}

void loraStartReceive() {
  radio.setDio1Action(onLoRaRx);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
}


// ========== PIN DEFINITIONS (Heltec LoRa32 V3) ==========
// LoRa SX1262 pins defined in heltec_unofficial.h:
// SS=GPIO8, DIO1=GPIO14, RST_LoRa=GPIO12, BUSY_LoRa=GPIO13
// SPI: SCK=GPIO9, MISO=GPIO11, MOSI=GPIO10

// OLED handled by Heltec library:
// SDA_OLED=GPIO17, SCL_OLED=GPIO18, RST_OLED=GPIO21
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// ========== NTRIP CONFIGURATION ==========
const char* WIFI_SSID     = "LiBase-LB1U02096";
const char* WIFI_PASSWORD = "12345678";
const char* NTRIP_HOST    = "192.168.10.21";
const int   NTRIP_PORT    = 12345;
const char* NTRIP_MOUNT   = "LB1U02096";
const char* NTRIP_USER    = "chris";
const char* NTRIP_PASS    = "chris";

// ========== LORA CONFIGURATION (MUST MATCH B-BOX) ==========
// RadioLib uses MHz and kHz units
#define LORA_FREQUENCY     915.0    // MHz
#define LORA_BANDWIDTH     125.0    // kHz
#define LORA_SPREAD_FACTOR 7        // SF7 - DO NOT CHANGE
#define LORA_CODING_RATE   5        // 4/5 - DO NOT CHANGE
#define LORA_SYNC_WORD     0x12
#define LORA_TX_POWER      20

// ========== PROTOCOL CONFIGURATION ==========
#define RTCM_BUFFER_SIZE   1024
#define MAX_FRAGMENT_SIZE  248
#define PARITY_FRAGMENT_MARKER 0xFE  // Special fragNum for parity fragment (FEC)
#define STATUS_LISTEN_MS   400      // Time to listen for B-Box status after RTCM TX
#define STATUS_PACKET_TYPE 0xBB     // Magic byte to identify status packets
#define CLEAR_BEACON_TYPE  0xDD     // Beacon sent after RTCM batch - signals B-BOX can TX

// ========== BLE CONFIGURATION ==========
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9F"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ========== OBJECTS ==========
// Display is created by Heltec library as 'display' (SSD1306Wire)
WiFiClient ntripClient;
NimBLEServer *pServer = NULL;
NimBLECharacteristic *pTxCharacteristic = NULL;

// ========== BLE STATE ==========
bool bleConnected = false;
uint32_t bleConnectTime = 0;
String bleRxBuffer = "";

// BLE RTCM buffer (processed in main loop, not in callback)
uint8_t bleRtcmBuffer[512];
volatile uint16_t bleRtcmHead = 0;
volatile uint16_t bleRtcmTail = 0;

// ========== NTRIP CONTROL ==========
bool baseLinkNtripEnabled = true;   // Enable Base Link's own NTRIP

// ========== RTCM STATE ==========
uint8_t rtcmBuffer[RTCM_BUFFER_SIZE];
uint32_t rtcmViaBleCount = 0;   // RTCM messages received via BLE
uint32_t rtcmViaBleBytes = 0;   // RTCM bytes received via BLE
uint16_t rtcmIndex = 0;
uint16_t rtcmLength = 0;
uint8_t rtcmState = 0;

// ========== RTCM RATE LIMITING (match app behavior) ==========
uint32_t lastStationTime = 0;    // 1005/1006 - station position
uint32_t lastBiasTime = 0;       // 1230 - GLONASS biases
uint32_t lastMsmTime = 0;        // All MSM messages share one timer
uint32_t cntGps = 0, cntGlo = 0, cntGal = 0, cntBds = 0, cntStation = 0, cntBias = 0;

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
bool clearBeaconSent = false;

// Epoch batching - detect complete epoch by message pattern
#define EPOCH_BUFFER_SIZE 900   // Single epoch max (~7 msgs * 120 bytes avg)
uint8_t epochBuffer[EPOCH_BUFFER_SIZE];
uint16_t epochBufferIndex = 0;
uint32_t epochStartTime = 0;

// Epoch tracking by message type (complete when all 4 MSM types received)
bool epochHasGps = false;      // 1074 (GPS MSM4)
bool epochHasGlo = false;      // 1084 (GLONASS MSM4)
bool epochHasGal = false;      // 1094 (Galileo MSM4)
bool epochHasBds = false;      // 1124 (BeiDou MSM4)
#define EPOCH_TIMEOUT_MS 500   // Force flush if epoch incomplete after 500ms       // Track if CLEAR beacon sent after RTCM batch
uint32_t rtcmBatchStartTime = 0;    // When current RTCM batch started
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
      // Check for RTCM binary data (starts with "RTCM:")
      if (rxValue.length() > 5 && rxValue.substr(0, 5) == "RTCM:") {
        // Buffer RTCM data for processing in main loop (NOT here - causes crash)
        size_t rtcmLen = rxValue.length() - 5;
        const uint8_t* rtcmData = (const uint8_t*)(rxValue.data() + 5);
        
        // Add to circular buffer
        for (size_t i = 0; i < rtcmLen; i++) {
          uint16_t nextHead = (bleRtcmHead + 1) % 512;
          if (nextHead != bleRtcmTail) {  // Don't overflow
            bleRtcmBuffer[bleRtcmHead] = rtcmData[i];
            bleRtcmHead = nextHead;
          }
        }
        
        rtcmViaBleCount++;
        rtcmViaBleBytes += rtcmLen;
      } else {
        // Regular text command
        bleRxBuffer += rxValue.c_str();
      }
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
  request += "User-Agent: NTRIP BaseLink/1.0\r\n";
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

// ========== LORA TRANSMIT RTCM WITH FEC ==========
void sendRTCMviaLoRa(uint8_t* data, uint16_t length) {
  uint16_t firstFragMax = MAX_FRAGMENT_SIZE - 2;  // 246 bytes for frag 0 (has timestamp)
  uint8_t totalFragments = 1;
  if (length > firstFragMax) {
    totalFragments = 1 + ((length - firstFragMax + MAX_FRAGMENT_SIZE - 1) / MAX_FRAGMENT_SIZE);
  }

  // Only add parity for multi-fragment messages
  bool useParity = (totalFragments > 1);

  // Parity buffer (max 248 bytes - largest fragment size)
  uint8_t parityBuffer[MAX_FRAGMENT_SIZE];
  memset(parityBuffer, 0, MAX_FRAGMENT_SIZE);
  uint16_t parityLen = 0;

  uint16_t dataOffset = 0;
  uint32_t txTimestamp = millis();

  // Send data fragments
  for (uint8_t frag = 0; frag < totalFragments; frag++) {
    loraBeginPacket();
    loraWrite(frag);
    loraWrite(totalFragments);

    uint8_t fragData[MAX_FRAGMENT_SIZE];
    uint16_t fragLen = 0;

    if (frag == 0) {
      uint16_t ts = (uint16_t)(txTimestamp & 0xFFFF);
      loraWrite(ts & 0xFF);
      loraWrite((ts >> 8) & 0xFF);

      fragLen = min((uint16_t)firstFragMax, length);
      memcpy(fragData, data, fragLen);
      loraWrite(fragData, fragLen);
      dataOffset = fragLen;
    } else {
      uint16_t remaining = length - dataOffset;
      fragLen = min((uint16_t)MAX_FRAGMENT_SIZE, remaining);
      memcpy(fragData, data + dataOffset, fragLen);
      loraWrite(fragData, fragLen);
      dataOffset += fragLen;
    }

    // XOR into parity buffer (pad with zeros for shorter fragments)
    if (useParity) {
      for (uint16_t i = 0; i < fragLen; i++) {
        parityBuffer[i] ^= fragData[i];
      }
      if (fragLen > parityLen) parityLen = fragLen;
    }

    loraEndPacket();
    packetsTransmitted++;
    yield();

    if (totalFragments > 1) delay(30);
  }

  // Send parity fragment for multi-fragment messages
  if (useParity && parityLen > 0) {
    delay(30);
    loraBeginPacket();
    loraWrite(PARITY_FRAGMENT_MARKER);  // 0xFE = parity
    loraWrite(totalFragments);
    loraWrite(parityBuffer, parityLen);
    loraEndPacket();
    packetsTransmitted++;
    yield();
  }

  lastRtcmTxTime = millis();
  clearBeaconSent = false;  // New RTCM sent - need to send CLEAR after batch
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

          // Rate limiting (matches Android app behavior)
          uint32_t now = millis();
          bool shouldSend = false;

          // Station position - 0.1 Hz (every 10 seconds)
          if (msgType == 1005 || msgType == 1006) {
            if (now - lastStationTime >= 10000) {
              lastStationTime = now;
              cntStation++;
              shouldSend = true;
            }
          }
          // GLONASS biases - 0.2 Hz (every 5 seconds) - REQUIRED for RTK Fix
          else if (msgType == 1230) {
            if (now - lastBiasTime >= 5000) {
              lastBiasTime = now;
              cntBias++;
              shouldSend = true;
            }
          }
          // ALL MSM messages - forward ALL (no timing window)
          // GPS (1071-1077), GLONASS (1081-1087), Galileo (1091-1097), BeiDou (1121-1127)
          else if ((msgType >= 1071 && msgType <= 1077) ||
                   (msgType >= 1081 && msgType <= 1087) ||
                   (msgType >= 1091 && msgType <= 1097) ||
                   (msgType >= 1121 && msgType <= 1127)) {
            // Count per constellation
            if (msgType >= 1071 && msgType <= 1077) cntGps++;
            else if (msgType >= 1081 && msgType <= 1087) cntGlo++;
            else if (msgType >= 1091 && msgType <= 1097) cntGal++;
            else if (msgType >= 1121 && msgType <= 1127) cntBds++;
            shouldSend = true;  // Always forward MSM - no windowing
          }

          if (shouldSend) {
            // Buffer message and track epoch by message type
            uint32_t now = millis();

            // Track which MSM types we have for this epoch
            if (msgType == 1074) { epochHasGps = true; Serial.println("[EPOCH] +GPS"); }
            else if (msgType == 1084) { epochHasGlo = true; Serial.println("[EPOCH] +GLO"); }
            else if (msgType == 1094) { epochHasGal = true; Serial.println("[EPOCH] +GAL"); }
            else if (msgType == 1124) { epochHasBds = true; Serial.println("[EPOCH] +BDS"); }
            else if (msgType == 1005) Serial.println("[EPOCH] +1005");
            else if (msgType == 1230) Serial.println("[EPOCH] +1230");

            // Add message to epoch buffer
            if (epochBufferIndex + rtcmIndex <= EPOCH_BUFFER_SIZE) {
              memcpy(epochBuffer + epochBufferIndex, rtcmBuffer, rtcmIndex);
              epochBufferIndex += rtcmIndex;
              rtcmMessagesProcessed++;

              if (epochStartTime == 0) epochStartTime = now;
            } else {
              // Buffer full - send what we have and start fresh
              Serial.print("[EPOCH] Buffer full, sending ");
              Serial.print(epochBufferIndex);
              Serial.println(" bytes");
              sendRTCMviaLoRa(epochBuffer, epochBufferIndex);
              lastRtcmTxTime = millis();
              clearBeaconSent = false;

              // Reset for new epoch
              epochBufferIndex = 0;
              epochHasGps = epochHasGlo = epochHasGal = epochHasBds = false;
              memcpy(epochBuffer, rtcmBuffer, rtcmIndex);
              epochBufferIndex = rtcmIndex;
              epochStartTime = now;

              // Track this message type
              if (msgType == 1074) epochHasGps = true;
              else if (msgType == 1084) epochHasGlo = true;
              else if (msgType == 1094) epochHasGal = true;
              else if (msgType == 1124) epochHasBds = true;
            }

            // Check if epoch is complete (all 4 MSM types received)
            if (epochHasGps && epochHasGlo && epochHasGal && epochHasBds) {
              Serial.print("[EPOCH COMPLETE] Sending ");
              Serial.print(epochBufferIndex);
              Serial.println(" bytes");
              sendRTCMviaLoRa(epochBuffer, epochBufferIndex);
              lastRtcmTxTime = millis();
              clearBeaconSent = false;

              // Reset for next epoch
              epochBufferIndex = 0;
              epochStartTime = 0;
              epochHasGps = epochHasGlo = epochHasGal = epochHasBds = false;

              inRxMode = true;
              rxModeStartTime = millis();
            }
          }
        }
        rtcmState = 0;
      }
      break;
  }
}

// ========== RECEIVE B-BOX STATUS ==========
// Send CLEAR beacon to signal B-BOX can transmit status
void sendClearBeacon() {
  loraBeginPacket();
  loraWrite(CLEAR_BEACON_TYPE);  // 0xDD
  loraEndPacket();
  clearBeaconSent = true;
  Serial.println("[LoRa TX] CLEAR beacon sent");
  yield();
}

void checkForBBoxStatus() {
  int packetSize = loraParsePacket();
  if (packetSize > 0) {
    Serial.print("[LoRa RX] Packet: ");
    Serial.print(packetSize);
    Serial.println(" bytes");
    // Check for status packet magic byte
    uint8_t firstByte = loraPeek();

    if (firstByte == STATUS_PACKET_TYPE) {
      loraRead(); // Consume magic byte

      bbox.rssi = loraPacketRssi();
      bbox.snr = loraPacketSnr();

      // Read status data
      char statusBuf[128];
      int idx = 0;
      while (loraAvailable() && idx < 127) {
        statusBuf[idx++] = loraRead();
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
    else if (cmd == "SHORE_NTRIP_ON") {
      baseLinkNtripEnabled = true;
      if (wifiConnected && !ntripConnected) {
        connectNTRIP();
      }
      bleSend("OK:SHORE_NTRIP_ON\n");
      Serial.println("[CMD] Shore NTRIP enabled");
    }
    else if (cmd == "SHORE_NTRIP_OFF") {
      baseLinkNtripEnabled = false;
      if (ntripConnected) {
        ntripClient.stop();
        ntripConnected = false;
      }
      bleSend("OK:SHORE_NTRIP_OFF\n");
      Serial.println("[CMD] Shore NTRIP disabled");
    }
  }
}

// Send command to B-Box via LoRa
void sendCommandViaLoRa(const char* cmdStr) {
  loraBeginPacket();
  loraWrite(0xCC);  // Command packet type
  loraPrint(cmdStr);
  loraEndPacket();
  Serial.print("[CMD] Sent via LoRa: ");
  Serial.println(cmdStr);
}

// ========== DISPLAY ==========
void updateDisplay() {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 500) return;
  lastUpdate = millis();

  display.clear();
  display.setFont(ArialMT_Plain_10);

  // Line 1: Status
  String line1 = "BaseLink";
  if (wifiConnected) line1 += " WiFi";
  if (ntripConnected) line1 += " NTRIP";
  display.drawString(0, 0, line1);

  // Line 2: RTCM stats
  String line2 = "TX:" + String(packetsTransmitted);
  line2 += " RTCM:" + String(rtcmMessagesProcessed);
  display.drawString(0, 12, line2);

  // Line 3: B-Box status
  String line3 = "BBox:";
  if (bbox.valid && (millis() - bbox.lastUpdate < 5000)) {
    line3 += "Fix" + String(bbox.fixQuality);
    line3 += " S" + String(bbox.satellites);
  } else {
    line3 += "--";
  }
  display.drawString(0, 24, line3);

  // Line 4: B-Box depth
  String line4 = "D:";
  if (bbox.valid && bbox.depth > 0) {
    line4 += String(bbox.depth, 2) + "m";
  } else {
    line4 += "--";
  }
  line4 += " R:" + String(bbox.rssi);
  display.drawString(0, 36, line4);

  // Line 5: Status packets
  String line5 = "Status RX:" + String(statusPacketsReceived);
  display.drawString(0, 48, line5);

  display.display();
}

// ========== SETUP ==========
void setup() {
  // Enable external power and init Heltec
  heltec_ve(true);
  delay(50);
  heltec_setup();

  // Reset and init OLED
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, LOW);
  delay(50);
  digitalWrite(RST_OLED, HIGH);
  delay(50);

  display.init();
  display.setContrast(255);
  display.flipScreenVertically();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Base Link");
  display.drawString(0, 12, "Starting...");
  display.display();


  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("    BASE LINK (Bidirectional)");
  Serial.println("========================================");
  Serial.println();

  // Init LoRa SX1262
  Serial.print("[INIT] LoRa SX1262... ");
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("FAILED! Error: ");
    Serial.println(state);
    while (1) delay(1000);
  }
  radio.setFrequency(LORA_FREQUENCY);
  radio.setBandwidth(LORA_BANDWIDTH);
  radio.setSpreadingFactor(LORA_SPREAD_FACTOR);
  radio.setCodingRate(LORA_CODING_RATE);
  radio.setSyncWord(LORA_SYNC_WORD);
  radio.setCRC(true);
  radio.setOutputPower(LORA_TX_POWER);
  loraStartReceive();
  Serial.println("OK");

  // Init BLE
  setupBLE();

  // Connect WiFi
  connectWiFi();

  // Connect NTRIP (only if enabled)
  if (baseLinkNtripEnabled && wifiConnected) {
    connectNTRIP();
  }

  updateDisplay();
  Serial.println("*** BASE LINK READY ***");
}

// ========== LOOP ==========
void loop() {
  heltec_loop();
  // Check WiFi/NTRIP connections
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    ntripConnected = false;
  }

  if (millis() - lastNtripCheck > 10000) {
    lastNtripCheck = millis();
    if (!wifiConnected) connectWiFi();
    if (baseLinkNtripEnabled && wifiConnected && !ntripConnected) connectNTRIP();
  }

  // Process NTRIP data (TX mode) - limit bytes per loop to allow RX
  if (baseLinkNtripEnabled && ntripConnected && ntripClient.connected() && !inRxMode) {
    int bytesRead = 0;
    while (ntripClient.available() && bytesRead < 100) {  // Process max 100 bytes per loop
      uint8_t byte = ntripClient.read();
      processRTCMByte(byte);
      bytesRead++;
    }
  } else if (ntripConnected && !ntripClient.connected()) {
    ntripConnected = false;
  }

  // Flush incomplete epoch if timeout reached
  if (epochBufferIndex > 0 && epochStartTime > 0) {
    if (millis() - epochStartTime > EPOCH_TIMEOUT_MS) {
      Serial.print("[EPOCH TIMEOUT] Sending incomplete ");
      Serial.print(epochBufferIndex);
      Serial.println(" bytes");
      sendRTCMviaLoRa(epochBuffer, epochBufferIndex);
      epochBufferIndex = 0;
      epochStartTime = 0;
      epochHasGps = epochHasGlo = epochHasGal = epochHasBds = false;
      lastRtcmTxTime = millis();
      clearBeaconSent = false;

      inRxMode = true;
      rxModeStartTime = millis();
    }
  }

  // Send CLEAR beacon after epoch TX complete (200ms gap)
  if (!clearBeaconSent && lastRtcmTxTime > 0 && (millis() - lastRtcmTxTime > 200)) {
    sendClearBeacon();
  }

  // ALWAYS listen for B-Box status (not just after RTCM TX)
  // This ensures we receive status even when NTRIP isn't connected
  checkForBBoxStatus();

  // Exit RX mode after timeout (for time-division when RTCM is flowing)
  if (inRxMode && (millis() - rxModeStartTime > STATUS_LISTEN_MS)) {
    inRxMode = false;
  }

  // Process buffered RTCM data from BLE (safe - outside callback)
  while (bleRtcmTail != bleRtcmHead) {
    uint8_t byte = bleRtcmBuffer[bleRtcmTail];
    bleRtcmTail = (bleRtcmTail + 1) % 512;
    processRTCMByte(byte);
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
