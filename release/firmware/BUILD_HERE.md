# Firmware Binaries

This folder will contain the compiled firmware binaries after you build them.

---

## How to Build

Follow the instructions in `../BUILD_INSTRUCTIONS.md` to compile the firmware.

**Quick build command:**

```bash
cd ../../hx-esp32-cam-fpv/air_firmware_esp32s3sense_ov3660
pio run
```

---

## Copy Binaries Here

After building, copy the binaries to this folder:

```bash
# From the firmware build directory
cp .pio/build/esp32s3sense/firmware.bin ../../release/firmware/
cp .pio/build/esp32s3sense/bootloader.bin ../../release/firmware/
cp .pio/build/esp32s3sense/partitions.bin ../../release/firmware/
```

---

## Generate Checksums

After copying binaries, generate SHA256 checksums:

```bash
cd ../../release/firmware
sha256sum *.bin > checksums.txt
```

---

## Expected Files

After building and copying, this folder should contain:

```
firmware/
├── BUILD_HERE.md           # This file
├── bootloader.bin          # ~30 KB - ESP32-S3 bootloader
├── partitions.bin          # ~3 KB - Partition table
├── firmware.bin            # ~1.5 MB - Main application
└── checksums.txt           # SHA256 checksums for verification
```

---

## Flash Memory Layout

These binaries are flashed to the following addresses:

| File | Address | Size | Description |
|------|---------|------|-------------|
| bootloader.bin | 0x0000 | ~30 KB | Second-stage bootloader |
| partitions.bin | 0x8000 | 3 KB | Partition table |
| firmware.bin | 0x10000 | ~1.5 MB | Main application code |

---

## Flashing Command

Once binaries are here, flash with:

```bash
esptool.py --chip esp32s3 \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  write_flash -z \
  --flash_mode dio \
  --flash_freq 80m \
  0x0000 bootloader.bin \
  0x8000 partitions.bin \
  0x10000 firmware.bin
```

Replace `/dev/ttyUSB0` with your port (COM3 on Windows, etc.)

---

## Why Binaries Are Not Included

Firmware binaries are **not included in the repository** because:

1. **Large file size** (~1.5 MB total)
2. **Git bloat** - binaries don't compress well
3. **Hardware-specific** - different boards may need different builds
4. **Easy to rebuild** - PlatformIO builds in 1-2 minutes
5. **Verifiable** - building yourself ensures integrity

---

## Build Time

- **First build:** 10-20 minutes (downloads ESP-IDF toolchain)
- **Subsequent builds:** 1-2 minutes
- **Storage required:** ~500 MB for toolchain + build files

---

## Alternative: Download Pre-Built

If pre-built binaries become available, they will be linked here.

**GitHub Releases:** https://github.com/vkofman56/ESP32S3_video/releases

*(Currently no releases - build yourself)*

---

**Status:** Ready to build
**Target:** XIAO ESP32S3 Sense + OV3660
**Date:** November 12, 2025
