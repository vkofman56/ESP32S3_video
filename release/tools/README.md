# Flashing Tools

This folder is for optional flashing tools you can download.

---

## Recommended Tools

### 1. ESPTool (All Platforms)

**Best for:** Command line users, Linux, macOS

**Install:**
```bash
pip install esptool
```

**Usage:**
```bash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 write_flash \
  0x0000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin
```

**Documentation:** https://docs.espressif.com/projects/esptool/

---

### 2. Flash Download Tools (Windows GUI)

**Best for:** Windows users who prefer GUI

**Download:**
- Visit: https://www.espressif.com/en/support/download/other-tools
- Download: "Flash Download Tools (ESP8266 & ESP32 & ESP32-S2 & ESP32-S3 & ESP32-C3)"
- Extract ZIP to this folder

**File:** `flash_download_tool_3.9.x.exe` (~50 MB)

**Usage:** See `../FLASHING_INSTRUCTIONS.md` for detailed steps

---

### 3. Web-Based Flasher (No Install)

**Best for:** Quick flashing without installing software

**Online tools:**
- ESP Web Tool: https://esp.huhn.me/
- Espressif Web Tool: https://espressif.github.io/esptool-js/

**Requirements:**
- Chrome or Edge browser (Web Serial API support)
- USB cable connected to XIAO ESP32S3

**Usage:** Upload binaries via web interface

---

### 4. PlatformIO (Developers)

**Best for:** Development workflow

**Install:**
```bash
pip install platformio
```

**Usage:**
```bash
cd ../../hx-esp32-cam-fpv/air_firmware_esp32s3sense_ov3660
pio run --target upload
```

---

## Tool Comparison

| Tool | Platform | Difficulty | Speed | Features |
|------|----------|------------|-------|----------|
| ESPTool | All | Easy | Fast | Command line |
| Flash Download Tool | Windows | Very Easy | Medium | GUI, many options |
| Web Flasher | All (browser) | Very Easy | Medium | No install needed |
| PlatformIO | All | Medium | Fast | Build + flash |

---

## Downloads Not Included

Tools are **not included in this repository** because:

1. **Large file sizes** (50-100 MB each)
2. **Frequent updates** - better to download latest version
3. **Platform-specific** - different OS need different tools
4. **Easy to install** - most are simple pip installs

---

## Need Help?

See detailed flashing instructions: `../FLASHING_INSTRUCTIONS.md`

---

**Last Updated:** November 12, 2025
