# Flashing Instructions - XIAO ESP32S3 Sense OV3660 Firmware

**Board:** Seeed Studio XIAO ESP32S3 Sense
**Camera:** OV3660 (2048×1536, 3MP)
**Firmware:** hx-esp32-cam-fpv with OV3660 support

---

## Table of Contents

1. [Hardware Preparation](#hardware-preparation)
2. [Method 1: ESPTool (Command Line)](#method-1-esptool-command-line)
3. [Method 2: ESP Flash Download Tool (Windows GUI)](#method-2-esp-flash-download-tool-windows-gui)
4. [Method 3: Web Flasher (Chrome/Edge Browser)](#method-3-web-flasher-chromeedge-browser)
5. [Method 4: PlatformIO (Developers)](#method-4-platformio-developers)
6. [Verification](#verification)
7. [Troubleshooting](#troubleshooting)

---

## Hardware Preparation

### 1. Connect XIAO ESP32S3 to Computer

**USB-C Cable:**
- Use a **data-capable** USB-C cable (not charge-only)
- Connect to computer's USB port
- Board LED should light up

### 2. Enter Bootloader Mode

**Automatic (Recommended):**
- Most tools auto-reset the board
- No manual button pressing needed

**Manual (if auto-reset fails):**
1. Hold **BOOT button** (labeled B or BOOT on board)
2. While holding BOOT, press **RESET button** briefly
3. Release BOOT button
4. Board is now in bootloader mode

### 3. Identify COM Port

**Windows:**
```cmd
# Device Manager → Ports (COM & LPT)
# Look for: "USB-SERIAL CH340" or "USB JTAG/serial debug unit"
# Example: COM3, COM4, COM5, etc.
```

**Linux:**
```bash
ls /dev/ttyUSB* /dev/ttyACM*
# Usually: /dev/ttyUSB0 or /dev/ttyACM0
```

**macOS:**
```bash
ls /dev/cu.usbserial-*
# Example: /dev/cu.usbserial-1420
```

---

## Method 1: ESPTool (Command Line)

**Best for:** Linux, macOS, developers

### Install ESPTool

```bash
pip install esptool
```

### Erase Flash (Optional but Recommended)

```bash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 erase_flash
```

Replace `/dev/ttyUSB0` with your actual port (COM3 on Windows, etc.)

### Flash Firmware

**If you have pre-built binaries in `release/firmware/`:**

```bash
esptool.py --chip esp32s3 \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --before default_reset \
  --after hard_reset \
  write_flash -z \
  --flash_mode dio \
  --flash_freq 80m \
  --flash_size detect \
  0x0000 bootloader.bin \
  0x8000 partitions.bin \
  0x10000 firmware.bin
```

**If you built the firmware yourself:**

```bash
cd hx-esp32-cam-fpv/air_firmware_esp32s3sense_ov3660

esptool.py --chip esp32s3 \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --before default_reset \
  --after hard_reset \
  write_flash -z \
  --flash_mode dio \
  --flash_freq 80m \
  --flash_size detect \
  0x0000 .pio/build/esp32s3sense/bootloader.bin \
  0x8000 .pio/build/esp32s3sense/partitions.bin \
  0x10000 .pio/build/esp32s3sense/firmware.bin
```

### Expected Output

```
esptool.py v4.7.0
Serial port /dev/ttyUSB0
Connecting....
Chip is ESP32-S3 (revision v0.1)
Features: WiFi, BLE
Crystal is 40MHz
MAC: xx:xx:xx:xx:xx:xx
Uploading stub...
Running stub...
Stub running...
Configuring flash size...
Flash will be erased from 0x00000000 to 0x00004fff...
Flash will be erased from 0x00008000 to 0x00008fff...
Flash will be erased from 0x00010000 to 0x0018ffff...
Compressed 18240 bytes to 12345...
Wrote 18240 bytes (12345 compressed) at 0x00000000 in 0.5 seconds...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
```

---

## Method 2: ESP Flash Download Tool (Windows GUI)

**Best for:** Windows users who prefer GUI

### Download Tool

1. Visit: https://www.espressif.com/en/support/download/other-tools
2. Download: "Flash Download Tools (ESP8266 & ESP32 & ESP32-S2 & ESP32-S3 & ESP32-C3)"
3. Extract ZIP file

### Flash Steps

1. **Run `flash_download_tool_x.exe`**

2. **Select Chip Type:**
   - ChipType: **ESP32-S3**
   - WorkMode: **Develop**
   - Click **OK**

3. **Configure Files:**
   ```
   ☑ bootloader.bin    @ 0x0000
   ☑ partitions.bin    @ 0x8000
   ☑ firmware.bin      @ 0x10000
   ```

4. **Configure Settings:**
   - SPI SPEED: **80 MHz**
   - SPI MODE: **DIO**
   - FLASH SIZE: **8 MB** (XIAO ESP32S3 has 8MB flash)
   - COM PORT: Select your port (e.g., COM3)
   - BAUD: **921600**

5. **Flash:**
   - Click **START**
   - Wait for "FINISH" message
   - Green checkmark = success

### Screenshot Reference

![Flash Tool Configuration](../docs/flash_download_tool_files_s3sense.png)

---

## Method 3: Web Flasher (Chrome/Edge Browser)

**Best for:** Quick flashing without installing tools

### Requirements

- Chrome or Edge browser (supports Web Serial API)
- USB cable connected to XIAO ESP32S3

### Steps

1. **Visit Web Flasher:**
   - Go to: https://esp.huhn.me/ (ESP Web Tool)
   - Or: https://espressif.github.io/esptool-js/

2. **Connect:**
   - Click **Connect**
   - Select your ESP32 port from popup
   - Click **Connect** again

3. **Upload Files:**
   - Click **Add File** for each binary:
     - `bootloader.bin` at offset `0x0000`
     - `partitions.bin` at offset `0x8000`
     - `firmware.bin` at offset `0x10000`

4. **Flash:**
   - Click **Program**
   - Wait for completion (2-3 minutes)

---

## Method 4: PlatformIO (Developers)

**Best for:** Development workflow

### Flash Command

```bash
cd hx-esp32-cam-fpv/air_firmware_esp32s3sense_ov3660
pio run --target upload
```

### With Custom Port

```bash
pio run --target upload --upload-port /dev/ttyUSB0
```

### Build + Flash + Monitor

```bash
pio run --target upload --target monitor
```

---

## Verification

### 1. Check Serial Output

**Connect serial monitor (115200 baud):**

```bash
# PlatformIO
pio device monitor

# ESPTool
esptool.py --port /dev/ttyUSB0 --baud 115200 read_flash 0x0 0x1000 -

# Screen (Linux/Mac)
screen /dev/ttyUSB0 115200

# PuTTY (Windows)
# Port: COM3, Baud: 115200
```

**Expected boot messages:**
```
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
...
Init camera...
Camera initialized: OV3660
WiFi starting...
```

### 2. Connect to WiFi

**Access Point Mode:**
- SSID: `hx-esp32-cam-fpv-XXXXXX` (XXXXXX = last 6 MAC digits)
- Password: `12345678` (default)
- Connect from phone/laptop
- Open browser: http://192.168.4.1

**Station Mode (Office WiFi):**
- Configure via web interface
- Check router for IP address
- Or use serial monitor to see assigned IP

### 3. Test Video Stream

1. Open web interface
2. Click **Start Stream**
3. You should see video from OV3660 camera
4. Check FPS counter (should show ~30 FPS at VGA)

---

## Troubleshooting

### Error: "Failed to connect to ESP32"

**Solution 1:** Enter bootloader mode manually
- Hold BOOT button
- Press RESET button briefly
- Release BOOT button
- Try flashing again

**Solution 2:** Lower baud rate
```bash
# Change from 921600 to 460800 or 115200
esptool.py --baud 460800 ...
```

**Solution 3:** Check USB cable
- Try different USB cable (data-capable)
- Try different USB port on computer

### Error: "A fatal error occurred: Timed out waiting for packet header"

**Cause:** Board not in bootloader mode

**Solution:**
```bash
# Add --before and --after flags
esptool.py --before default_reset --after hard_reset ...
```

### Error: "Serial port already in use"

**Solution:**
- Close other programs using the port (Arduino IDE, serial monitors)
- On Linux: Close any `screen` or `minicom` sessions
- Unplug and replug USB cable

### Camera Not Detected After Flash

**Check:**
1. Camera ribbon cable is properly inserted
2. Camera lens is uncovered (remove protective film)
3. Serial output shows: "Camera initialized: OV3660"

**If shows "OV2640" instead of "OV3660":**
- Wrong firmware flashed
- Make sure you flashed the OV3660 variant

### WiFi Not Starting

**Check:**
1. Serial output shows WiFi MAC address
2. Antenna is connected (if using external antenna)
3. No metal shielding blocking WiFi

**Reset to defaults:**
- Press and hold BOOT button for 10 seconds
- Board will reset WiFi settings

### Web Interface Not Loading

**Check:**
1. Connected to correct WiFi network
2. IP address in browser matches board's IP
3. Try http://192.168.4.1 (default AP mode)

**Clear browser cache:**
- Ctrl+Shift+R (Windows/Linux)
- Cmd+Shift+R (Mac)

---

## Flash Memory Layout

| Address | Size | Content |
|---------|------|---------|
| 0x0000 | ~30 KB | Bootloader |
| 0x8000 | 3 KB | Partition table |
| 0x10000 | ~1.5 MB | Main firmware |
| ... | ... | NVS (settings), OTA, etc. |

---

## Backup Original Firmware (Optional)

Before flashing, backup the original firmware:

```bash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 read_flash 0x0 0x800000 backup.bin
```

To restore:
```bash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 write_flash 0x0 backup.bin
```

---

## Next Steps

After successful flashing:

1. **Configure WiFi:** See web interface at http://192.168.4.1
2. **Test camera:** Verify OV3660 detection and streaming
3. **Optimize settings:** Adjust resolution, FPS, quality
4. **Testing:** See `TESTING_GUIDE.md` for comprehensive tests

---

## Support

**Flashing Issues:**
- ESPTool docs: https://docs.espressif.com/projects/esptool/
- XIAO ESP32S3 wiki: https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/

**Firmware Issues:**
- Original project: https://github.com/RomanLut/hx-esp32-cam-fpv/issues
- This fork: https://github.com/vkofman56/ESP32S3_video/issues

---

**Last Updated:** November 12, 2025
**Board:** XIAO ESP32S3 Sense
**Camera:** OV3660
