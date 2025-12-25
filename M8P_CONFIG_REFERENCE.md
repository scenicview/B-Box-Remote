# u-blox M8P Configuration Reference

This document contains all UBX configuration commands needed to configure the Here+ M8P rover for PPK (Post-Processing Kinematic) and RTK (Real-Time Kinematic) operation.

## Configuration Overview

| Feature | UBX Message | Purpose |
|---------|-------------|---------|
| RTCM Input | UBX-CFG-PRT | Enable RTCM3 protocol on UART1 |
| Rover Mode | UBX-CFG-TMODE3 | Disable base station mode |
| RTK Mode | UBX-CFG-DGNSS | Enable RTK float + fixed |
| PPK Output | UBX-CFG-MSG (RAWX) | Raw measurements for PPK |
| PPK Output | UBX-CFG-MSG (SFRBX) | Navigation subframes for PPK |
| Position | UBX-CFG-MSG (NAV-PVT) | Position/velocity/time |
| RTCM Monitor | UBX-CFG-MSG (RXM-RTCM) | Monitor RTCM reception |
| Save Config | UBX-CFG-CFG | Persist to flash |

## Hardware Wiring

```
TTGO LoRa32 V2.0          Here+ M8P (8-pin JST)
-----------------          -------------------
GPIO25 (TX) ------------>  Pin 3 (GPS_TX/RX input)
GPIO33 (RX) <------------  Pin 2 (GPS_RX/TX output)
GND        --------------> Pin 8 (GND)
```

## UART Settings

- Baud Rate: 115200
- Data Bits: 8
- Parity: None
- Stop Bits: 1

---

## UBX Command Reference

### 1. UBX-CFG-PRT (0x06 0x00) - Port Configuration

Configure UART1 to accept RTCM3 input and output UBX + NMEA.

**Payload (20 bytes):**
| Offset | Size | Field | Value | Description |
|--------|------|-------|-------|-------------|
| 0 | 1 | portID | 0x01 | UART1 |
| 1 | 1 | reserved0 | 0x00 | |
| 2 | 2 | txReady | 0x0000 | Disabled |
| 4 | 4 | mode | 0x000008D0 | 8N1 |
| 8 | 4 | baudRate | 0x0001C200 | 115200 |
| 12 | 2 | inProtoMask | 0x0021 | UBX + RTCM3 |
| 14 | 2 | outProtoMask | 0x0003 | UBX + NMEA |
| 16 | 2 | flags | 0x0000 | |
| 18 | 2 | reserved1 | 0x0000 | |

**Protocol Mask Bits:**
- Bit 0: UBX (0x01)
- Bit 1: NMEA (0x02)
- Bit 5: RTCM3 (0x20)

**Complete Message (28 bytes):**
```cpp
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
```

---

### 2. UBX-CFG-TMODE3 (0x06 0x71) - Time Mode Configuration

Set mode to Disabled (0) for rover operation. This prevents the M8P from acting as a base station.

**Payload (40 bytes):**
| Offset | Size | Field | Value | Description |
|--------|------|-------|-------|-------------|
| 0 | 1 | version | 0x00 | |
| 1 | 1 | reserved0 | 0x00 | |
| 2 | 2 | flags | 0x0000 | mode = 0 (Disabled) |
| 4 | 36 | (unused) | 0x00... | Not used for rover |

**Mode Values:**
- 0 = Disabled (Rover)
- 1 = Survey-In (Base - auto-survey)
- 2 = Fixed Mode (Base - known position)

**Complete Message (48 bytes):**
```cpp
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
```

---

### 3. UBX-CFG-DGNSS (0x06 0x70) - DGNSS Configuration

Set RTK mode to allow both float and fixed solutions.

**Payload (4 bytes):**
| Offset | Size | Field | Value | Description |
|--------|------|-------|-------|-------------|
| 0 | 1 | dgnssMode | 0x03 | RTK float + fixed |
| 1 | 3 | reserved | 0x000000 | |

**DGNSS Mode Values:**
- 2 = RTK Float only
- 3 = RTK Float + Fixed (default, recommended)

**Complete Message (12 bytes):**
```cpp
uint8_t cfgDgnss[] = {
  0xB5, 0x62, 0x06, 0x70, 0x04, 0x00,
  0x03,                   // dgnssMode = RTK float + fixed
  0x00, 0x00, 0x00,       // reserved
  0x7D, 0x64              // checksum
};
```

---

### 4. UBX-CFG-MSG (0x06 0x01) - Message Rate Configuration

Enable various UBX messages on UART1.

**Payload (8 bytes):**
| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | msgClass | Message class |
| 1 | 1 | msgID | Message ID |
| 2 | 1 | rateI2C | Rate on I2C |
| 3 | 1 | rateUART1 | Rate on UART1 |
| 4 | 1 | rateUART2 | Rate on UART2 |
| 5 | 1 | rateUSB | Rate on USB |
| 6 | 1 | rateSPI | Rate on SPI |
| 7 | 1 | reserved | |

#### 4a. Enable UBX-RXM-RAWX (0x02 0x15) - Raw Measurements

```cpp
uint8_t enableRAWX[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x02, 0x15,             // RXM-RAWX
  0x00, 0x01, 0x00, 0x00, 0x00, 0x00,  // UART1 = 1Hz
  0x27, 0x47              // checksum
};
```

#### 4b. Enable UBX-RXM-SFRBX (0x02 0x13) - Navigation Subframes

```cpp
uint8_t enableSFRBX[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x02, 0x13,             // RXM-SFRBX
  0x00, 0x01, 0x00, 0x00, 0x00, 0x00,  // UART1 = 1Hz
  0x25, 0x3F              // checksum
};
```

#### 4c. Enable UBX-NAV-PVT (0x01 0x07) - Position/Velocity/Time

```cpp
uint8_t enablePVT[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x01, 0x07,             // NAV-PVT
  0x00, 0x01, 0x00, 0x00, 0x00, 0x00,  // UART1 = 1Hz
  0x18, 0xE1              // checksum
};
```

#### 4d. Enable UBX-RXM-RTCM (0x02 0x32) - RTCM Status Monitor

```cpp
uint8_t enableRxmRtcm[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x02, 0x32,             // RXM-RTCM
  0x00, 0x01, 0x00, 0x00, 0x00, 0x00,  // UART1 = 1Hz
  0x44, 0x16              // checksum
};
```

---

### 5. UBX-CFG-CFG (0x06 0x09) - Save Configuration

Save all settings to flash memory.

**Payload (13 bytes):**
| Offset | Size | Field | Value | Description |
|--------|------|-------|-------|-------------|
| 0 | 4 | clearMask | 0x00000000 | Don't clear |
| 4 | 4 | saveMask | 0x00001F1F | Save all sections |
| 8 | 4 | loadMask | 0x00000000 | Don't load |
| 12 | 1 | deviceMask | 0x17 | BBR + Flash + EEPROM + SPI |

**Device Mask Bits:**
- Bit 0: BBR (Battery-Backed RAM)
- Bit 1: Flash
- Bit 2: EEPROM
- Bit 4: SPI Flash

**Complete Message (21 bytes):**
```cpp
uint8_t cfgSave[] = {
  0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
  0x00, 0x00, 0x00, 0x00, // clearMask
  0x1F, 0x1F, 0x00, 0x00, // saveMask (all sections)
  0x00, 0x00, 0x00, 0x00, // loadMask
  0x17,                   // deviceMask
  0x71, 0xDF              // checksum
};
```

---

## Complete Arduino Configuration Function

```cpp
void configureM8P() {
  Serial.println("[GPS] Configuring M8P for PPK + RTK...");
  delay(100);

  // 1. Configure UART1 for RTCM3 input
  uint8_t cfgPrtUart1[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    0x00, 0xC2, 0x01, 0x00, 0x21, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xDA, 0x4E
  };
  GPSSerial.write(cfgPrtUart1, sizeof(cfgPrtUart1));
  delay(100);

  // 2. Set rover mode (TMODE3 = Disabled)
  uint8_t cfgTmode3[] = {
    0xB5, 0x62, 0x06, 0x71, 0x28, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x9F, 0x93
  };
  GPSSerial.write(cfgTmode3, sizeof(cfgTmode3));
  delay(50);

  // 3. Set DGNSS mode (RTK float + fixed)
  uint8_t cfgDgnss[] = {
    0xB5, 0x62, 0x06, 0x70, 0x04, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x7D, 0x64
  };
  GPSSerial.write(cfgDgnss, sizeof(cfgDgnss));
  delay(50);

  // 4. Enable RXM-RAWX (PPK raw measurements)
  uint8_t enableRAWX[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x02, 0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x27, 0x47
  };
  GPSSerial.write(enableRAWX, sizeof(enableRAWX));
  delay(50);

  // 5. Enable RXM-SFRBX (PPK nav subframes)
  uint8_t enableSFRBX[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x25, 0x3F
  };
  GPSSerial.write(enableSFRBX, sizeof(enableSFRBX));
  delay(50);

  // 6. Enable NAV-PVT (position/velocity/time)
  uint8_t enablePVT[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x18, 0xE1
  };
  GPSSerial.write(enablePVT, sizeof(enablePVT));
  delay(50);

  // 7. Enable RXM-RTCM (RTCM status monitor)
  uint8_t enableRxmRtcm[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0x02, 0x32, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x44, 0x16
  };
  GPSSerial.write(enableRxmRtcm, sizeof(enableRxmRtcm));
  delay(50);

  // 8. Save configuration to flash
  uint8_t cfgSave[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x17, 0x71, 0xDF
  };
  GPSSerial.write(cfgSave, sizeof(cfgSave));
  delay(500);

  Serial.println("[GPS] M8P configured for PPK + RTK");
}
```

---

## UBX Checksum Calculation

The UBX checksum uses Fletcher-8 algorithm over the message class, ID, length, and payload bytes.

```cpp
void calculateChecksum(uint8_t* msg, int len, uint8_t* ck_a, uint8_t* ck_b) {
  *ck_a = 0;
  *ck_b = 0;
  // Start from byte 2 (after sync bytes), end before checksum
  for (int i = 2; i < len - 2; i++) {
    *ck_a = (*ck_a + msg[i]) & 0xFF;
    *ck_b = (*ck_b + *ck_a) & 0xFF;
  }
}
```

---

## Python Script to Generate UBX Messages

See `D:/rtkrover/deeper/generate_ubx_config.py` for a Python script that generates UBX messages with correct checksums.

---

## RTK Operation Notes

### RTCM Messages Required from Base Station

| Message Type | Description | Rate |
|--------------|-------------|------|
| 1005 | Station coordinates (ARP) | 0.1 Hz |
| 1074/1077 | GPS MSM4/MSM7 observations | 1 Hz |
| 1084/1087 | GLONASS MSM4/MSM7 observations | 1 Hz |
| 1124/1127 | BeiDou MSM4/MSM7 observations | 1 Hz |
| 1230 | GLONASS code-phase biases | 0.2 Hz |

### RTK Fix Requirements

- **Single constellation (GPS only):** 6 satellites with continuous phase lock
- **Dual GPS + GLONASS:** 7 satellites (5 GPS + 2 GLONASS)
- **GPS + BeiDou:** 8 satellites (5 GPS + 3 BeiDou)

### Fix Quality Values (GGA field 6)

| Value | Meaning | Accuracy |
|-------|---------|----------|
| 0 | No Fix | - |
| 1 | GPS | 2-5 m |
| 2 | DGPS | 0.5-2 m |
| 4 | RTK Fixed | 1-5 cm |
| 5 | RTK Float | 10-50 cm |

---

## References

- [u-blox M8 Protocol Specification (UBX-13003221)](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
- [C94-M8P Application Board User Guide](https://content.u-blox.com/sites/default/files/C94-M8P-AppBoard_UserGuide_(UBX-15031066).pdf)
- [NEO-M8P Data Sheet](https://content.u-blox.com/sites/default/files/NEO-M8P_DataSheet_UBX-15016656.pdf)

---

*Last updated: December 2024*
