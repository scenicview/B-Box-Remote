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

### Navigation Tab (New)
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
- WiFi AP: 
- UDP Port: 10110
- Update rate: ~14 Hz
- Protocols: NMEA 0183 (, )

## LoRa Configuration (MUST MATCH)


## Build Instructions

### B-Box Firmware


### Shore Station Firmware


### Android App

ERROR: JAVA_HOME is set to an invalid directory: path/to/jdk-17

## BLE Protocol

### Service UUIDs (Nordic UART)
- Service: 
- TX (notify): 
- RX (write): 

### Status Format (15 Hz)

Example: 

### Commands
| Command | Response | Action |
|---------|----------|--------|
|  |  | Enable SD recording |
|  |  | Disable SD recording |
|  | Status line | Request immediate status |
|  |  | Wide beam (90-115 kHz) |
|  |  | Medium beam (270-310 kHz) |
|  |  | Narrow beam (635-715 kHz) |

### Fix Quality Values
| Value | Meaning | Color |
|-------|---------|-------|
| 0 | No Fix | Gray |
| 1 | GPS | Yellow |
| 2 | DGPS | Orange |
| 4 | RTK Fixed | Green |
| 5 | RTK Float | Cyan |

## Recent Changes (December 2024)

### v1.3.0 - December 25, 2024
#### Android App
- **Raw RTCM via BLE**: NTRIP client now sends raw RTCM3 data (0xD3 preamble) to B-Box
- **Removed RTCM prefix**: BLE transmission sends raw binary without "RTCM:" prefix for M8P compatibility

#### B-Box Firmware
- **M8P RTCM Input Configuration**: Added complete UBX configuration for receiving RTK corrections
  - UBX-CFG-PRT: UART1 configured for RTCM3 input (inProtoMask=0x21)
  - UBX-CFG-TMODE3: Rover mode (mode=0) for mobile operation
  - UBX-CFG-DGNSS: RTK mode 3 for float+fixed solutions
  - UBX-CFG-MSG: Enabled UBX-RXM-RTCM for correction monitoring
  - UBX-CFG-CFG: Configuration saved to M8P flash memory
- **BLE RTCM Forwarding**: Detects raw RTCM data (0xD3 preamble) from BLE and forwards to GPS
- **RTCM Status Display**: OLED shows RTCM KB received via BLE
- **RAWX Logging**: Enabled UBX-RXM-RAWX and SFRBX for PPK post-processing

#### New Documentation
- **M8P_CONFIG_REFERENCE.md**: Complete u-blox M8P configuration reference with UBX commands

### v1.2.0 - December 24, 2024
#### Android App
- **Project Management**: Added Project tab for organizing survey sessions
- **Project-named files**: Files now named by project (e.g., `survey1.csv`, `survey1.ubx`) instead of numbered
- **Download from B-Box**: Fixed file discovery to support both legacy and project naming
- **File pairing**: Smart matching of CSV/UBX files by base name
- **Dialog styling**: All dialog buttons now have solid backgrounds matching app theme
- **Delete All Logs**: Now deletes all .csv and .ubx files regardless of naming convention

#### B-Box Firmware
- **Project naming support**: Files created with project names sent from app
- **Case-insensitive file listing**: LIST_LOGS finds .csv/.ubx files regardless of case
- **Improved file handling**: Increased filename buffer sizes, String-based filename creation
- **Delete all fix**: DELETE_ALL now removes all .csv and .ubx files

### v1.1.0 - December 2024
#### Android App
- Added Navigation tab with 2D/3D view toggle
- Integrated OSMdroid for 2D map (OSM, Topo, Satellite layers)
- Added color-coded depth points on 2D map
- Implemented IMU-corrected bottom position calculation
- Added GPS validation (4+ sats, valid coordinate ranges)
- Fixed NTRIP client (added cleartext traffic permission)
- Added offline map download capability
- Zoom support up to level 24 (sub-meter scale)

#### B-Box Firmware
- Added GPS validation in BLE and LoRa status output
- Prevents transmission of invalid (0,0) coordinates
- Requires 4+ satellites before reporting position

#### Shore Station
- New firmware for LoRa relay station
- NTRIP client with configurable credentials
- Status display on OLED

## File Structure


## Troubleshooting

### NTRIP Won't Connect
- Ensure  in AndroidManifest.xml
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

### 3D View Not Updating
- Points are added via TrackManager
- Renderer polls TrackManager automatically
- Try clearing and restarting track

## License
MIT License
