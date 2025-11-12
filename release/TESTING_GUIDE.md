# Testing Guide - XIAO ESP32S3 Sense OV3660 Firmware

**Firmware:** hx-esp32-cam-fpv with OV3660 support
**Date:** November 12, 2025
**Purpose:** Verify OV3660 camera functionality and low-latency video streaming

---

## Table of Contents

1. [Quick Start Test](#quick-start-test)
2. [Camera Detection Test](#camera-detection-test)
3. [Video Streaming Tests](#video-streaming-tests)
4. [Latency Measurement](#latency-measurement)
5. [Resolution & FPS Tests](#resolution--fps-tests)
6. [Stability Tests](#stability-tests)
7. [Performance Benchmarks](#performance-benchmarks)
8. [Troubleshooting](#troubleshooting)

---

## Quick Start Test

**Goal:** Verify firmware boots and camera works

### 1. Power On and Boot Check

```bash
# Connect serial monitor (115200 baud)
pio device monitor
# or
screen /dev/ttyUSB0 115200
```

**Expected output:**
```
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
...
[INFO] Init camera...
[INFO] Camera sensor: OV3660
[INFO] Resolution: 800x600 (SVGA)
[INFO] FPS target: 30
[INFO] WiFi starting...
[INFO] AP SSID: hx-esp32-cam-fpv-XXXXXX
[INFO] IP: 192.168.4.1
```

✅ **Pass Criteria:**
- Boot completes in < 5 seconds
- Shows "Camera sensor: OV3660" (not OV2640 or OV5640)
- WiFi starts successfully
- IP address displayed

---

## Camera Detection Test

**Goal:** Confirm OV3660 is properly detected and initialized

### Test Steps

1. **Check serial output for camera init:**
   ```
   [INFO] Camera sensor: OV3660
   [INFO] Camera I2C address: 0x3C
   [INFO] PID: 0x36
   ```

2. **Verify camera settings:**
   - Connect to web interface: http://192.168.4.1
   - Check "Camera Info" section
   - Should show: "OV3660 (3MP, 2048×1536)"

3. **Test pattern (if available):**
   - In web interface, enable "Test Pattern"
   - Should show colorful bars (verify camera interface works)

✅ **Pass Criteria:**
- Camera ID reads as OV3660
- No "Camera init failed" errors
- Test pattern displays correctly

❌ **Fail Indicators:**
- "Camera not found"
- "Unknown sensor"
- Shows OV2640 or OV5640 instead

---

## Video Streaming Tests

**Goal:** Verify video streams at various resolutions

### Test 1: Basic Streaming (VGA)

1. **Connect to WiFi AP:**
   - SSID: `hx-esp32-cam-fpv-XXXXXX`
   - Password: `12345678`

2. **Open web interface:**
   - Browser: http://192.168.4.1
   - Click "Start Stream"

3. **Verify:**
   - Video appears within 1-2 seconds
   - Image is clear and focused
   - FPS counter shows ~30 FPS
   - No frozen frames

✅ **Pass Criteria:**
- Stream starts < 2 seconds
- Steady 25-30 FPS
- No major artifacts or freezing
- Latency feels responsive (< 150ms)

### Test 2: Multiple Resolutions

Test each resolution from the web interface:

| Resolution | Expected FPS | Pass/Fail |
|------------|-------------|-----------|
| QVGA (320×240) | 60 | [ ] |
| VGA (640×480) | 30 | [ ] |
| SVGA (800×600) | 30 | [ ] |
| XGA (1024×768) | 25 | [ ] |
| HD (1280×720) | 25 | [ ] |
| SXGA (1280×960) | 20 | [ ] |

**Testing procedure for each:**
1. Select resolution in web interface
2. Wait 2 seconds for camera to adjust
3. Note actual FPS from counter
4. Check image quality

✅ **Pass Criteria:**
- FPS within 10% of expected
- Image quality acceptable
- No crashes when changing resolution

### Test 3: High FPS Mode

1. **Enable High FPS mode** (web interface)
2. **Test VGA resolution:**
   - Expected: 45 FPS
   - Actual: _____ FPS
3. **Test SVGA resolution:**
   - Expected: 35-40 FPS
   - Actual: _____ FPS

✅ **Pass Criteria:**
- High FPS mode provides 15-50% FPS increase
- Video remains stable
- No overheating issues

---

## Latency Measurement

**Goal:** Measure end-to-end video latency (target: <100ms)

### Method 1: Stopwatch Method

**Equipment needed:**
- Stopwatch (phone app)
- Camera pointed at stopwatch display
- Laptop/phone displaying stream

**Steps:**
1. Position camera to see stopwatch
2. Start stopwatch
3. Open stream on laptop
4. Take photo of both screens showing same frame
5. Compare timestamps

**Formula:**
```
Latency = Stopwatch_on_screen - Stopwatch_in_camera_view
```

**Example:**
- Stopwatch in camera view: 12:34:56.050
- Stopwatch on laptop screen: 12:34:56.120
- Latency: 70ms ✅

✅ **Target Latency:**
- Excellent: < 60ms
- Good: 60-100ms
- Acceptable: 100-150ms
- Poor: > 150ms

### Method 2: LED Flash Method

1. Connect LED to GPIO pin
2. Trigger LED flash via serial command
3. Measure time between command and seeing flash in stream
4. Use oscilloscope or high-speed camera to measure

---

## Resolution & FPS Tests

**Goal:** Verify OV3660 FPS performance matches expectations

### FPS Test Matrix

| Resolution | Normal Mode | High FPS Mode | Actual (Normal) | Actual (High) |
|------------|-------------|---------------|-----------------|---------------|
| QVGA (320×240) | 60 | 60 | _____ | _____ |
| VGA (640×480) | 30 | 45 | _____ | _____ |
| VGA16 (640×360) | 30 | 45 | _____ | _____ |
| SVGA (800×600) | 30 | 35 | _____ | _____ |
| SVGA16 (800×456) | 30 | 45 | _____ | _____ |
| XGA (1024×768) | 25 | 30 | _____ | _____ |
| HD (1280×720) | 25 | 30 | _____ | _____ |
| SXGA (1280×960) | 20 | 25 | _____ | _____ |

### Image Quality Assessment

For each resolution, check:

**Sharpness:**
- [ ] Text is readable at VGA+
- [ ] No excessive blur
- [ ] Focus is correct

**Color Accuracy:**
- [ ] Colors appear natural
- [ ] No green/purple tint
- [ ] White balance works

**Exposure:**
- [ ] Not overexposed (bright)
- [ ] Not underexposed (dark)
- [ ] Auto-exposure adjusts properly

**Noise:**
- [ ] Minimal noise in good lighting
- [ ] Acceptable noise in low light
- [ ] No weird artifacts

---

## Stability Tests

**Goal:** Ensure firmware runs reliably over time

### Test 1: 10-Minute Continuous Streaming

1. Start video stream
2. Let run for 10 minutes
3. Monitor for issues

**Monitor:**
- FPS stability (should stay consistent)
- Memory usage (check serial output)
- Temperature (touch board, should be warm but not hot)
- Stream freezes or disconnects

✅ **Pass Criteria:**
- No crashes for 10+ minutes
- FPS variance < 10%
- No memory leaks (check heap size in serial)
- Temperature < 70°C

### Test 2: Resolution Switching

1. Switch between resolutions 20 times:
   - VGA → SVGA → XGA → VGA → repeat
2. Wait 5 seconds between switches

✅ **Pass Criteria:**
- No crashes
- All resolutions work after switching
- Camera responds within 2 seconds

### Test 3: WiFi Reconnection

1. Disconnect from WiFi
2. Wait 30 seconds
3. Reconnect
4. Verify stream resumes

✅ **Pass Criteria:**
- Auto-reconnects within 10 seconds
- Stream resumes automatically
- No manual reset needed

---

## Performance Benchmarks

**Goal:** Measure performance vs OV2640 and OV5640

### Comparison Test Setup

If you have access to OV2640 or OV5640 boards, test all three:

| Metric | OV2640 | OV3660 | OV5640 | Winner |
|--------|--------|--------|--------|--------|
| VGA FPS (normal) | 30 | _____ | 30 | |
| VGA FPS (high) | 40 | _____ | 50 | |
| SVGA FPS (normal) | 30 | _____ | 30 | |
| XGA FPS (normal) | 12 | _____ | 30 | |
| HD FPS (normal) | 12 | _____ | 30 | |
| Latency (VGA) | 100-150ms | _____ | 60-100ms | |
| Max resolution | 1600×1200 | 2048×1536 | 2592×1944 | |
| Image quality | Good | _____ | Excellent | |

**OV3660 Expected Performance:**
- Better than OV2640 at all resolutions
- Between OV2640 and OV5640 at HD+
- Similar latency to OV5640

### Latency Comparison

| Resolution | Target | OV3660 Actual | Pass/Fail |
|------------|--------|---------------|-----------|
| VGA (640×480) | < 100ms | _____ ms | [ ] |
| SVGA (800×600) | < 120ms | _____ ms | [ ] |
| HD (1280×720) | < 150ms | _____ ms | [ ] |

---

## DVR Recording Test

**Goal:** Verify SD card recording works

### Prerequisites

- MicroSD card installed (Class 10, up to 32GB)
- Formatted as FAT32

### Test Steps

1. **Enable DVR recording:**
   - Web interface → Settings → DVR
   - Click "Start Recording"

2. **Record 1 minute of video**

3. **Stop recording:**
   - Click "Stop Recording"

4. **Check SD card:**
   - Remove SD card
   - Check file size (should be ~15-30 MB for 1 min at VGA)
   - Play video on computer (VLC player)

✅ **Pass Criteria:**
- Recording starts without errors
- File saves correctly
- Video plays back smoothly
- No frame drops in recorded video

---

## Web Interface Test

**Goal:** Verify all web UI features work

### Feature Checklist

**Camera Controls:**
- [ ] Resolution selector works
- [ ] FPS mode toggle works
- [ ] Quality slider adjusts image
- [ ] Brightness/Contrast/Saturation work

**WiFi Settings:**
- [ ] Can switch to Station mode
- [ ] Can connect to office WiFi
- [ ] AP mode can be re-enabled

**Advanced Settings:**
- [ ] Exposure settings work
- [ ] White balance adjusts
- [ ] Special effects apply (if supported)

**OSD (On-Screen Display):**
- [ ] FPS counter visible
- [ ] Resolution displayed
- [ ] Camera info shown

---

## Troubleshooting

### Camera Not Detected

**Symptoms:**
```
[ERROR] Camera init failed
[ERROR] Camera not found
```

**Checks:**
1. Camera ribbon cable fully inserted
2. Cable not reversed (blue side up on XIAO)
3. Camera lens protective film removed
4. No physical damage to camera module

**Solution:**
- Reseat camera cable
- Try different camera module
- Check for bent pins on connector

### Low FPS

**Symptoms:**
- FPS counter shows < 20 FPS at VGA
- Video is choppy

**Checks:**
1. WiFi signal strength (should be > -70 dBm)
2. Clock frequency setting (should be 16 MHz for OV3660)
3. High FPS mode enabled
4. Quality setting (lower quality = higher FPS)

**Solution:**
```cpp
// In main.cpp, verify clock is set correctly:
config.xclk_freq_hz = 16000000;  // OV3660
```

### High Latency

**Symptoms:**
- Latency > 150ms
- Noticeable delay when moving camera

**Checks:**
1. WiFi mode (use 5GHz if possible, but ESP32 only supports 2.4GHz)
2. Distance to WiFi router
3. WiFi congestion
4. Buffer settings

**Solution:**
- Move closer to WiFi router
- Use external antenna (3dBi recommended)
- Reduce resolution
- Enable high FPS mode

### Image Quality Issues

**Blurry:**
- Adjust camera focus (rotate lens)
- Clean lens
- Check if lens is damaged

**Too Dark:**
- Increase brightness setting
- Enable auto-exposure (AEC)
- Increase exposure value

**Too Bright:**
- Decrease brightness
- Reduce exposure value
- Enable auto white balance (AWB)

**Wrong Colors:**
- Enable AWB
- Adjust white balance mode
- Check color saturation setting

---

## Success Criteria Summary

### Minimum Viable (Basic Functionality)

- ✅ Camera detected as OV3660
- ✅ VGA streams at 25+ FPS
- ✅ Latency < 150ms
- ✅ Stable for 10+ minutes
- ✅ No crashes

### Production Ready (Deployment)

- ✅ All resolutions work
- ✅ Latency < 100ms at VGA
- ✅ High FPS mode functional
- ✅ Stable for 30+ minutes
- ✅ DVR recording works
- ✅ WiFi reconnects automatically

### Optimal (Stretch Goals)

- ✅ Latency < 60ms at VGA
- ✅ HD streams at 25+ FPS
- ✅ Stable for hours
- ✅ All web UI features work
- ✅ Performance better than OV2640

---

## Test Report Template

```markdown
# OV3660 Firmware Test Report

**Date:** _______________
**Tester:** _______________
**Hardware:** XIAO ESP32S3 Sense + OV3660
**Firmware Version:** _______________

## Results

### Camera Detection
- Sensor detected: [ ] OV3660 [ ] Other: _____
- Initialization: [ ] Pass [ ] Fail

### Video Streaming
- VGA FPS: _____ (target: 30)
- SVGA FPS: _____ (target: 30)
- HD FPS: _____ (target: 25)

### Latency
- VGA latency: _____ ms (target: <100ms)
- HD latency: _____ ms (target: <150ms)

### Stability
- 10min test: [ ] Pass [ ] Fail
- Resolution switching: [ ] Pass [ ] Fail
- WiFi reconnect: [ ] Pass [ ] Fail

### Overall Assessment
- [ ] Ready for deployment
- [ ] Needs optimization
- [ ] Critical issues found

### Notes:
_______________________________________________
_______________________________________________
```

---

## Next Steps

After passing all tests:

1. **For drone use:**
   - Integrate with MAVLink telemetry (see ESP-NOW_MAVLink repo)
   - Test outdoor range with 3dBi antenna
   - Measure AI control loop latency

2. **For optimization:**
   - Fine-tune clock frequencies
   - Adjust buffer sizes
   - Test different WiFi channels

3. **For production:**
   - Mount securely on drone
   - Weatherproof if needed
   - Document final configuration

---

**Last Updated:** November 12, 2025
**Firmware:** OV3660 Custom Build
**Status:** Ready for testing
