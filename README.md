# DeeperRTK - RTK Surveying with Sonar Integration

## Overview
Complete RTK surveying system integrating Deeper Chirp+2 sonar with centimeter-accurate GPS positioning. Includes ESP32 firmware for boat-mounted receiver (B-Box), shore station LoRa relay, and Android remote control app.

## System Components

### 1. B-Box (DeeperRTK_Receiver)
Boat-mounted TTGO LoRa32 that integrates:
- LoRa RTCM reception from shore station
- Deeper Chirp+2 sonar via WiFi UDP
- u-blox M8P RTK GPS rover
- ADXL345 IMU for pitch/roll
- SD card logging (PPK-ready)
- BLE remote control (NimBLE)

### 2. Shore Station
TTGO LoRa32 that relays RTK corrections:
- Connects to base station via WiFi/NTRIP
- Receives RTCM 3.2 corrections
- Transmits via LoRa to B-Box
- OLED status display

### 3. Android App (DeeperRTK Remote)
Remote monitoring and control:
- Real-time GPS/depth/temperature display
- **Navigation Tab** with 2D map and 3D point cloud
- Color-coded depth visualization
- NTRIP client for direct RTK corrections
- SD card recording control
- Offline map support

## Features

### Navigation Tab
- **2D Map View**: OSMdroid with OSM, Topo, and Esri Satellite imagery
- **3D Point Cloud**: OpenGL ES 2.0 visualization with touch rotation/zoom
- **Color-coded depth points**: Blue (shallow) to red (deep) gradient
- **IMU-corrected positions**: True seabed position accounting for pitch/roll
- **Track polyline**: GPS path overlay on map
- **Zoom to 1m scale**: High-resolution viewing (zoom level 24)
- **Offline maps**: Download tiles for use without internet

### GPS Validation
- Requires 4+ satellites before recording points
- Validates coordinate ranges (-90 to 90 lat, -180 to 180 lon)
- Prevents erroneous (0,0) "Africa points" during GPS cold start

### NTRIP Client
- Direct RTK corrections via internet (WiFi/cellular)
- HTTP cleartext support for legacy NTRIP casters
- Configurable server, port, mountpoint, credentials

### Data Logging
- 20 Hz CSV logging with UTC timestamps
- PPK-ready format for post-processing
- Optional UBX raw logging for true PPK correction

## Hardware

### B-Box (TTGO LoRa32 V2.0)
| Component | Connection |
|-----------|------------|
| LoRa SX1276 | Internal (GPIO 5,18,19,26,27) |
| OLED Display | Internal I2C (GPIO 21,22) |
| M8P GPS TX | GPIO 32 |
| M8P GPS RX | GPIO 33 |
| ADXL345 IMU | I2C (shared with OLED) |
| SD Card | Internal SD_MMC (GPIO 2,14,15) |

### Shore Station (TTGO LoRa32 V2.0)
Same hardware, different firmware - connects to WiFi for NTRIP.

### Deeper Chirp+2 Sonar
- WiFi AP: `Deeper CHIRP+ XXXX`
- UDP Port: 10110
- Update rate: ~14 Hz
- Protocols: NMEA 0183 ($SDDBT, $YXMTW)

## LoRa Configuration (MUST MATCH)
```cpp
#define LORA_FREQUENCY     915E6   // 915 MHz Americas
#define LORA_BANDWIDTH     125E3   // 125 kHz
#define LORA_SPREAD_FACTOR 7       // SF7 ONLY - SF9+ breaks M8P!
#define LORA_CODING_RATE   5       // 4/5
#define LORA_SYNC_WORD     0x12
#define LORA_TX_POWER      20      // 20 dBm
```

## Build Instructions

### B-Box Firmware
```bash
# Compile (requires huge_app partition for BLE)
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app \
  DeeperRTK_Receiver/DeeperRTK_Receiver.ino

# Upload
arduino-cli upload -p COM18 --fqbn esp32:esp32:esp32:PartitionScheme=huge_app \
  DeeperRTK_Receiver/DeeperRTK_Receiver.ino
```

### Shore Station Firmware
```bash
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app \
  ShoreStation/ShoreStation.ino

arduino-cli upload -p COM19 --fqbn esp32:esp32:esp32:PartitionScheme=huge_app \
  ShoreStation/ShoreStation.ino
```

### Android App
```bash
cd android
export JAVA_HOME="path/to/jdk-17"
./gradlew assembleDebug

# Install via ADB
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

## BLE Protocol

### Service UUIDs (Nordic UART)
- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- TX (notify): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- RX (write): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`

### Status Format (15 Hz)
```
FIX,SATS,DEPTH,TEMP,BATT,REC,PITCH,ROLL,LAT,LON
```
Example: `4,12,1.25,18.5,85,1,5.2,-3.1,54.325682,-126.660538`

### Commands
| Command | Response | Action |
|---------|----------|--------|
| `START` | `OK:REC_ON` | Enable SD recording |
| `STOP` | `OK:REC_OFF` | Disable SD recording |
| `STATUS` | Status line | Request immediate status |
| `FREQ_WIDE` | `OK:FREQ_WIDE` | Wide beam (90-115 kHz) |
| `FREQ_MED` | `OK:FREQ_MED` | Medium beam (270-310 kHz) |
| `FREQ_NARROW` | `OK:FREQ_NARROW` | Narrow beam (635-715 kHz) |

### Fix Quality Values
| Value | Meaning | Color |
|-------|---------|-------|
| 0 | No Fix | Gray |
| 1 | GPS | Yellow |
| 2 | DGPS | Orange |
| 4 | RTK Fixed | Green |
| 5 | RTK Float | Cyan |

## Recent Changes (December 2024)

### Android App
- Added Navigation tab with 2D/3D view toggle
- Integrated OSMdroid for 2D map (OSM, Topo, Satellite layers)
- Added color-coded depth points on 2D map
- Implemented IMU-corrected bottom position calculation
- Added GPS validation (4+ sats, valid coordinate ranges)
- Fixed NTRIP client (added cleartext traffic permission)
- Added offline map download capability
- Zoom support up to level 24 (sub-meter scale)

### B-Box Firmware
- Added GPS validation in BLE and LoRa status output
- Prevents transmission of invalid (0,0) coordinates
- Requires 4+ satellites before reporting position

### Shore Station
- New firmware for LoRa relay station
- NTRIP client with configurable credentials
- Status display on OLED

## File Structure
```
deeper/
├── DeeperRTK_Receiver/     # B-Box firmware
│   └── DeeperRTK_Receiver.ino
├── ShoreStation/           # Shore relay firmware
│   └── ShoreStation.ino
├── android/                # Android app
│   └── app/src/main/java/com/deeperrtk/remote/
│       ├── MainActivity.kt
│       ├── DepthPoint.kt
│       ├── TrackManager.kt
│       ├── PointCloudGLSurfaceView.kt
│       └── PointCloudRenderer.kt
├── CLAUDE.md               # Development notes
└── README.md               # This file
```

## Troubleshooting

### NTRIP Won't Connect
- Ensure `usesCleartextTraffic="true"` in AndroidManifest.xml
- Check server address and port
- Verify credentials and mountpoint

### Points Recording in Africa (0,0)
- GPS needs time to acquire fix
- Wait for 4+ satellites before surveying
- Firmware now validates GPS before transmission

### Satellite Imagery Not Loading
- Uses Esri World Imagery (requires internet)
- Download offline tiles before going to field
- Switch to OSM if satellite unavailable

## License
MIT License
