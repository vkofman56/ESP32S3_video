# Build Instructions for OV3660 Firmware

**Target Hardware:** XIAO ESP32S3 Sense with OV3660 Camera
**Firmware Location:** `hx-esp32-cam-fpv/air_firmware_esp32s3sense_ov3660/`
**Date:** November 12, 2025

---

## Prerequisites

### Required Software

1. **Python 3.7 or later**
   ```bash
   python3 --version
   ```

2. **PlatformIO Core**
   ```bash
   pip install platformio
   # or
   pip3 install platformio
   ```

3. **Git** (for cloning repository)
   ```bash
   git --version
   ```

### Verify Installation

```bash
pio --version
# Should show: PlatformIO Core, version 6.1.18 or later
```

---

## Building the Firmware

### Method 1: PlatformIO CLI (Recommended)

1. **Navigate to the firmware directory:**
   ```bash
   cd hx-esp32-cam-fpv/air_firmware_esp32s3sense_ov3660
   ```

2. **Build the firmware:**
   ```bash
   pio run
   ```

3. **First build notes:**
   - First build takes 10-20 minutes (downloads ESP-IDF toolchain)
   - Subsequent builds take 1-2 minutes
   - Downloads ~500MB of tools and dependencies

4. **Build output location:**
   ```
   .pio/build/esp32s3sense/firmware.bin
   .pio/build/esp32s3sense/bootloader.bin
   .pio/build/esp32s3sense/partitions.bin
   ```

### Method 2: PlatformIO IDE (Visual Studio Code)

1. **Install VS Code + PlatformIO extension:**
   - Download VS Code: https://code.visualstudio.com/
   - Install PlatformIO IDE extension from marketplace

2. **Open project:**
   - File → Open Folder
   - Select: `hx-esp32-cam-fpv/air_firmware_esp32s3sense_ov3660/`

3. **Build:**
   - Click "Build" button (checkmark icon) in bottom toolbar
   - Or: PlatformIO → Build

---

## Build Variants

The repository includes 3 firmware variants:

| Folder | Camera Sensor | Clock | Target Use |
|--------|---------------|-------|------------|
| `air_firmware_esp32cam` | OV2640 | 12 MHz | ESP32-CAM boards |
| `air_firmware_esp32s3sense` | OV2640 | 12 MHz | XIAO ESP32S3 Sense (original camera) |
| `air_firmware_esp32s3sense_ov5640` | OV5640 | 20 MHz | XIAO ESP32S3 Sense (upgraded camera) |
| **`air_firmware_esp32s3sense_ov3660`** | **OV3660** | **16 MHz** | **XIAO ESP32S3 Sense (your board)** |

**⚠️ Important:** Make sure you're in the correct folder before building!

---

## Troubleshooting Build Issues

### Error: "Platform espressif32 @ 6.9.0 not found"

**Solution:** PlatformIO will auto-download on first build. If it fails:
```bash
pio pkg install --platform espressif32@6.9.0
```

### Error: "Python 3.11 required"

The build uses ESP-IDF which requires Python 3.11 or lower. If you have Python 3.12+:

**Option A:** Install Python 3.11 and use it specifically:
```bash
python3.11 -m pip install platformio
python3.11 -m platformio run
```

**Option B:** Edit `platformio.ini` line 16:
```ini
;platform = espressif32@ 6.9.0 //current
platform = espressif32@ 6.5.0  //compatible with Python 3.11+
```

### Error: "Permission denied" or "Access denied"

**Linux/Mac:**
```bash
sudo chmod +x ~/.platformio/penv/bin/pio
```

**Windows:** Run command prompt as Administrator

### Error: "Out of memory" during build

**Solution:** Close other applications. ESP32 builds require ~2GB RAM.

### Build succeeds but output files missing

**Check:** `.pio/build/esp32s3sense/` folder should contain:
- `firmware.bin` (~1.5 MB)
- `bootloader.bin` (~30 KB)
- `partitions.bin` (~3 KB)

If missing, try clean rebuild:
```bash
pio run --target clean
pio run
```

---

## Advanced Build Options

### Clean Build

Remove all build artifacts and rebuild from scratch:
```bash
pio run --target clean
pio run
```

### Verbose Build

See detailed compilation output:
```bash
pio run --verbose
```

### Build for Upload

Build and prepare for flashing:
```bash
pio run --target upload
```

### Monitor Serial Output

View debug output after flashing (115200 baud):
```bash
pio device monitor
```

---

## Build Configuration

Key settings in `platformio.ini`:

```ini
[env:esp32s3sense]
platform = espressif32@ 6.9.0
framework = espidf
board = seeed_xiao_esp32s3
build_flags = -DBOARD_XIAOS3SENSE -DSENSOR_OV3660
monitor_speed = 115200
```

**Build Flags:**
- `-DBOARD_XIAOS3SENSE` - Configures GPIO pins for XIAO ESP32S3
- `-DSENSOR_OV3660` - Enables OV3660 camera support

---

## Verifying Build Output

After successful build, verify files:

```bash
ls -lh .pio/build/esp32s3sense/*.bin
```

Expected output:
```
-rw-r--r-- 1 user user  26K  firmware.elf
-rw-r--r-- 1 user user 1.5M  firmware.bin
-rw-r--r-- 1 user user  30K  bootloader.bin
-rw-r--r-- 1 user user 3.0K  partitions.bin
```

---

## Post-Build Steps

### Copy Binaries to Release Folder

```bash
mkdir -p ../../../release/firmware
cp .pio/build/esp32s3sense/firmware.bin ../../../release/firmware/
cp .pio/build/esp32s3sense/bootloader.bin ../../../release/firmware/
cp .pio/build/esp32s3sense/partitions.bin ../../../release/firmware/
```

### Calculate SHA256 Checksums

```bash
cd ../../../release/firmware
sha256sum *.bin > checksums.txt
```

---

## Next Steps

After building successfully:

1. **Flash to board:** See `FLASHING_INSTRUCTIONS.md`
2. **Test functionality:** See `TESTING_GUIDE.md`
3. **Configure settings:** Connect to web interface at http://192.168.4.1

---

## Build Time Expectations

| Build Type | Duration | Notes |
|------------|----------|-------|
| First build | 10-20 min | Downloads ESP-IDF, toolchain (~500MB) |
| Clean build | 3-5 min | Full recompilation |
| Incremental | 30-60 sec | Only changed files |

---

## Support

**Build Issues:**
- Check PlatformIO troubleshooting: https://docs.platformio.org/en/latest/
- ESP-IDF documentation: https://docs.espressif.com/projects/esp-idf/

**Project Issues:**
- Original project: https://github.com/RomanLut/hx-esp32-cam-fpv
- This fork: https://github.com/vkofman56/ESP32S3_video

---

**Last Updated:** November 12, 2025
**Firmware Version:** OV3660 Custom Build
