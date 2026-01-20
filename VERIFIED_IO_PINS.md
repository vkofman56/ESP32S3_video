# Verified IO Pins - ESP32 Boards

**Date:** November 12, 2025
**Status:** Verified and working configurations

---

## Board 1: ESP32-C3 Super Mini (Telemetry - Drone)

**Use:** MAVLink telemetry via DroneBridge firmware
**Status:** ✅ **WORKING** (0.2-0.5% packet loss, 15-42ms latency)
**Baud Rate:** 460800 (CRITICAL - lower rates don't work!)

### Pin Configuration

**UART Pins (for MAVLink):**
- Board has limited GPIO, uses default UART pins
- Exact pins depend on DroneBridge firmware configuration
- **Baud: 460800** ← Must use this rate!

**WiFi:**
- Built-in antenna OR external antenna via IPEX connector
- Recommended: CORONIR 3dBi external antenna

**Notes:**
- ESP32-C3 does NOT have camera interface
- WiFi Station mode required (NOT Access Point mode)
- Must connect to external WiFi router for reliable operation

---

## Board 2: XIAO ESP32-C3 (Ground Station - In Development)

**Use:** Ground station telemetry receiver
**Status:** In development
**Hardware:** Seeed Studio XIAO ESP32-C3

### Pin Configuration

Same as ESP32-C3 Super Mini - details TBD when ground station is developed.

**Reference:** https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html

---

## Board 3: XIAO ESP32S3 Sense (Video + Camera)

**Use:** Low-latency video streaming with OV3660 camera
**Status:** Firmware ready, awaiting hardware testing
**Hardware:** Seeed Studio XIAO ESP32S3 Sense
**Schematic:** https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_SCH_v1.1.pdf

### UART Pins (Serial Communication)

**UART1 - MSP-OSD (DisplayPort OSD):**
- TX: GPIO 2 (D1)
- RX: GPIO 4 (D3)
- Baud: 115200
- Use: DisplayPort OSD communication

**UART2 - MAVLink Telemetry:**
- TX: GPIO 43 (D6)
- RX: GPIO 44 (D7)
- Baud: 115200
- Use: MAVLink telemetry (if using single-board setup)

### Camera Pins (OV3660/OV2640/OV5640)

**I2C (Camera Control):**
- SDA: GPIO 40 (SIOD)
- SCL: GPIO 39 (SIOC)

**DVP Interface (Camera Data):**
- XCLK: GPIO 10 (Camera clock)
- PCLK: GPIO 13 (Pixel clock)
- VSYNC: GPIO 38 (Vertical sync)
- HREF: GPIO 47 (Horizontal reference)

**Camera Data Pins (8-bit parallel):**
- D0 (Y2): GPIO 15
- D1 (Y3): GPIO 17
- D2 (Y4): GPIO 18
- D3 (Y5): GPIO 16
- D4 (Y6): GPIO 14
- D5 (Y7): GPIO 12
- D6 (Y8): GPIO 11
- D7 (Y9): GPIO 48

**Camera Power:**
- PWDN: -1 (not used)
- RESET: -1 (not used)

### Control Pins

**LED:**
- Status LED: GPIO 1
- LED ON: HIGH (1)
- LED OFF: LOW (0)

**Button:**
- REC Button: GPIO 0 (shared with BOOT button)
- Function: Start/stop recording

**SD Card:**
- CS (Chip Select): GPIO 21 (shared with LED on some variants)

### Notes

- Board has **8MB Flash** and **8MB PSRAM**
- PSRAM required for camera frame buffering
- All camera pins are fixed - cannot be changed
- UARTs can be used for telemetry OR OSD (not both simultaneously without multiplexing)

---

## Board 4: ESP32-CAM (AI-Thinker) - Reference

**Use:** Alternative video board (not currently used)
**Status:** Reference only

### Pin Configuration

**UART1 - MAVLink:**
- TX: GPIO 1
- RX: GPIO 3
- Baud: 115200

**UART2 - MSP-OSD:**
- TX: GPIO 12 (must be LOW at boot!)
- RX: GPIO 13
- Baud: 115200

**Camera (OV2640):**
- Similar to XIAO S3 but different GPIO numbers

**LED:**
- Status: GPIO 33
- Flash: GPIO 4

**Camera Power:**
- PWDN: GPIO 32
- RESET: -1

---

## Pin Usage Summary by Function

### Telemetry (MAVLink)

| Board | TX | RX | Baud | Status |
|-------|----|----|------|--------|
| ESP32-C3 Super Mini | Default | Default | 460800 | ✅ Working |
| XIAO ESP32S3 Sense | GPIO 43 | GPIO 44 | 115200 | Ready |
| ESP32-CAM | GPIO 1 | GPIO 3 | 115200 | Reference |

### Camera Interface

| Signal | XIAO ESP32S3 GPIO | ESP32-CAM GPIO |
|--------|-------------------|----------------|
| XCLK | 10 | 0 |
| PCLK | 13 | 22 |
| VSYNC | 38 | 25 |
| HREF | 47 | 23 |
| SDA | 40 | 26 |
| SCL | 39 | 27 |
| D0-D7 | 15,17,18,16,14,12,11,48 | 5,18,19,21,36,39,34,35 |

### Status/Control

| Function | XIAO ESP32S3 | ESP32-CAM | ESP32-C3 |
|----------|--------------|-----------|----------|
| Status LED | GPIO 1 | GPIO 33 | N/A |
| Button | GPIO 0 | GPIO 4 | N/A |
| SD Card CS | GPIO 21 | - | N/A |

---

## Critical Pin Constraints

### XIAO ESP32S3 Sense

**Strapping Pins (affect boot mode):**
- GPIO 0: Used for BOOT/REC button (OK)
- Other strapping pins handled by board design

**Boot Requirements:**
- GPIO 12 (TXD2) should be LOW at boot (auto-handled)
- No conflicts with current pin assignments

**Conflicts:**
- GPIO 21: Shared between LED and SD CS on some boards
- UARTs: Cannot use UART1 and UART2 simultaneously without careful configuration

### ESP32-C3 Super Mini

**Critical:**
- Must use 460800 baud for telemetry (57600 doesn't work)
- WiFi Station mode required (AP mode has 45-50% packet loss)
- External WiFi router necessary for reliable operation

---

## Recommended Dual-Board Wiring

### Drone Setup (Option A - Recommended)

**ESP32-C3 (Telemetry):**
- UART → Flight Controller MAVLink port
- WiFi → Office WiFi router (Station mode)
- Antenna: 3dBi CORONIR IPEX

**XIAO ESP32S3 (Video):**
- Camera → OV3660 via ribbon cable
- WiFi → Same office WiFi router
- Antenna: 3dBi CORONIR IPEX
- Optional: UART2 can connect to FC for backup telemetry

**Benefits:**
- Each board runs proven firmware
- Zero interference
- Maximum reliability
- Easy troubleshooting

---

## WiFi Configuration

**Both Boards:**
- Frequency: 2.4 GHz (ESP32 only supports 2.4 GHz)
- Mode: Station (connect to router)
- Router: Office WiFi (same network for both)

**Antennas:**
- Drone: 3dBi omnidirectional (works at any angle)
- Ground: 10dBi directional (better range, aim at drone)

**Expected Range:**
- Indoor: 50-100m
- Outdoor: 200-300m (with 3dBi on drone, 10dBi on ground)

---

## Pin Assignment Changes for OV3660

**No pin changes needed!**

The OV3660 camera uses the same DVP interface as OV2640/OV5640:
- Same physical connector
- Same GPIO assignments
- Only firmware differences (clock speed, FPS tables)

All pin configurations from main.h remain valid for OV3660.

---

## Testing Checklist

When cameras arrive:

**XIAO ESP32S3 with OV3660:**
- [ ] Verify camera detected on I2C (address 0x3C)
- [ ] Check all camera data pins working
- [ ] Test UART1 for OSD (GPIO 2/4)
- [ ] Test UART2 for MAVLink (GPIO 43/44)
- [ ] Verify SD card access (GPIO 21)
- [ ] Test status LED (GPIO 1)
- [ ] Test REC button (GPIO 0)

**ESP32-C3 Telemetry:**
- [ ] Verify 460800 baud connection
- [ ] Confirm WiFi Station mode stable
- [ ] Test MAVLink packet loss (<1%)
- [ ] Measure latency (target: <50ms)

---

## References

**Hardware:**
- XIAO ESP32S3 Schematic: https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_SCH_v1.1.pdf
- XIAO ESP32C3: https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html

**Firmware:**
- DroneBridge ESP32: https://github.com/DroneBridge/ESP32
- hx-esp32-cam-fpv: https://github.com/RomanLut/hx-esp32-cam-fpv

**This Project:**
- Main repo: https://github.com/vkofman56/ESP32S3_video
- Telemetry: https://github.com/vkofman56/ESP-NOW_MAVLink

---

**Last Updated:** November 12, 2025
**Status:** All pin configurations verified from firmware source
**Next:** Test with actual OV3660 cameras when they arrive
