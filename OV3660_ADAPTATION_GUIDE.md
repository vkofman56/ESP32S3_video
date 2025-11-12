# OV3660 Adaptation Guide for hx-esp32-cam-fpv

**Date:** November 12, 2025
**Goal:** Add OV3660 camera sensor support to XIAO ESP32S3 Sense

---

## Key Differences Between OV2640 and OV5640 Builds

### 1. Build Configuration (`platformio.ini`)

**OV2640 (default):**
```ini
build_flags = -DBOARD_XIAOS3SENSE
```

**OV5640:**
```ini
build_flags = -DBOARD_XIAOS3SENSE -DSENSOR_OV5640
```

**For OV3660, we'll need:**
```ini
build_flags = -DBOARD_XIAOS3SENSE -DSENSOR_OV3660
```

---

### 2. Camera Clock Frequency (`main.cpp:2792-2796`)

**OV2640:**
```cpp
config.xclk_freq_hz = 12000000;  // 12 MHz (actual: 13.333 MHz with clk2x)
```

**OV5640:**
```cpp
config.xclk_freq_hz = 20000000;  // 20 MHz
```

**OV3660:** (needs research)
- OV3660 typically supports 6-27 MHz clock
- Likely start with **16-20 MHz** (similar to OV5640)
- Test and optimize for best performance

---

### 3. Frame Rate Selection (`main.cpp:931-935`)

```cpp
#ifdef SENSOR_OV5640
    uint8_t fps = config.camera.ov5640HighFPS ? v->highFPS5640 : v->FPS5640;
#else
    uint8_t fps = config.camera.ov2640HighFPS ? v->highFPS2640 : v->FPS2640;
#endif
```

**For OV3660:**
```cpp
#ifdef SENSOR_OV3660
    uint8_t fps = config.camera.ov3660HighFPS ? v->highFPS3660 : v->FPS3660;
#elif SENSOR_OV5640
    uint8_t fps = config.camera.ov5640HighFPS ? v->highFPS5640 : v->FPS5640;
#else
    uint8_t fps = config.camera.ov2640HighFPS ? v->highFPS2640 : v->FPS2640;
#endif
```

---

### 4. FPS Table Structure (`packets.h` + `packets.cpp`)

**Current TVMode structure:**
```cpp
typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t FPS2640;      // OV2640 normal FPS
    uint8_t FPS5640;      // OV5640 normal FPS
    uint8_t highFPS2640;  // OV2640 high FPS
    uint8_t highFPS5640;  // OV5640 high FPS
} TVMode;
```

**Need to add OV3660 fields:**
```cpp
typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t FPS2640;
    uint8_t FPS5640;
    uint8_t FPS3660;      // NEW: OV3660 normal FPS
    uint8_t highFPS2640;
    uint8_t highFPS5640;
    uint8_t highFPS3660;  // NEW: OV3660 high FPS
} TVMode;
```

**Current FPS capabilities (from `packets.cpp`):**

| Resolution | Size | OV2640 FPS | OV2640 High | OV5640 FPS | OV5640 High |
|------------|------|------------|-------------|------------|-------------|
| QVGA | 320×240 | 60 | 60 | 60 | 60 |
| CIF | 400×296 | 60 | 60 | 60 | 60 |
| HVGA | 480×320 | 30 | 30 | 30 | 30 |
| VGA | 640×480 | 30 | 40 | 30 | 50 |
| VGA16 | 640×360 | 30 | 40 | 30 | 50 |
| SVGA | 800×600 | 30 | 30 | 30 | 30 |
| SVGA16 | 800×456 | 30 | 40 | 30 | 50 |
| XGA | 1024×768 | 12 | 12 | 30 | 30 |
| XGA16 | 1024×576 | 12 | 12 | 30 | 30 |
| SXGA | 1280×960 | 12 | 12 | 30 | 30 |
| HD | 1280×720 | 12 | 12 | 30 | 30 |
| UXGA | 1600×1200 | 10 | 10 | 10 | 10 |

**OV3660 Expected Capabilities:**
- Native resolution: 2048×1536 (3MP)
- Better than OV2640 (2MP)
- Between OV2640 and OV5640 (5MP) performance
- Likely FPS: **Similar to OV5640 at lower resolutions, slightly less at higher**

---

### 5. Camera Settings (`main.cpp:1472-1500`)

**OV2640 specific:**
```cpp
// Dynamic clock adjustment for high FPS modes
if (src.camera.ov2640HighFPS &&
    ((resolution == VGA) || (resolution == VGA16) || (resolution == SVGA16))) {
    s->set_xclk(s, LEDC_TIMER_0, 16);  // 16 MHz for high FPS
} else {
    s->set_xclk(s, LEDC_TIMER_0, 12);  // 12 MHz normal
}
```

**OV5640 specific:**
```cpp
// Uses colorbar register for high FPS mode (not aec2)
s->set_colorbar(s, src.camera.ov5640HighFPS ? 1 : 0);

// Also disables aec2 for OV5640 (line 1472-1475)
// "aec2 is not aec dsp but 'night vision' mode which decimates framerate"
src.camera.aec2 = false;
```

**OV3660:** (needs testing)
- May need similar clock adjustments as OV2640
- Check if aec2 behavior is like OV2640 or OV5640
- Test high FPS modes

---

### 6. Resolution Mapping (`main.cpp:1502-1544`)

Different sensors use different internal resolution settings:

**VGA16 (640×360):**
- OV2640: Custom crop via `set_res_raw()` - 800×456 cropped
- OV5640: `FRAMESIZE_P_3MP` (640×360 native)

**SVGA16 (800×456):**
- OV2640: Custom crop via `set_res_raw()`
- OV5640: `FRAMESIZE_P_HD` (800×456 native)

**XGA16 (1024×576):**
- OV2640: Custom crop from UXGA - 1600×1200 cropped
- OV5640: `FRAMESIZE_P_FHD` (1920×1080 downscaled)

**OV3660:**
- Will likely support native 16:9 resolutions like OV5640
- Need to test which `FRAMESIZE_*` constants work best
- May need custom `set_res_raw()` for some resolutions

---

### 7. Status Reporting (`main.cpp:2090-2094`)

```cpp
#ifdef SENSOR_OV5640
    packet.stats.isOV5640 = 1;
#else
    packet.stats.isOV5640 = 0;
#endif
```

**Need to add OV3660 identification:**
- Add new stats field for `isOV3660`
- Or change to generic `sensorType` enum

---

## OV3660 Sensor Driver (Already Exists!)

The esp32-camera library already includes OV3660 driver:
```
hx-esp32-cam-fpv/components/esp32-camera/sensors/
├── ov2640.c
├── ov3660.c          ✅ Driver exists!
├── ov5640.c
└── private_include/
    ├── ov3660.h
    ├── ov3660_regs.h
    └── ov3660_settings.h
```

**Key OV3660 Specs:**
- Max Resolution: 2048×1536 (3MP)
- Supports: RGB565/555/444, YCbCr422, JPEG
- I2C address: 0x3C (different from OV2640/OV5640)
- Clock range: 6-27 MHz

---

## Implementation Steps

### Phase 1: Create OV3660 Firmware Build

1. **Copy firmware folder:**
   ```bash
   cp -r air_firmware_esp32s3sense_ov5640 air_firmware_esp32s3sense_ov3660
   ```

2. **Update `platformio.ini`:**
   ```ini
   build_flags = -DBOARD_XIAOS3SENSE -DSENSOR_OV3660
   ```

3. **Update `sdkconfig.defaults` (if needed)**

### Phase 2: Add OV3660 Support to Code

1. **Add FPS fields to `packets.h`:**
   - Modify `TVMode` structure to include `FPS3660` and `highFPS3660`

2. **Update FPS table in `packets.cpp`:**
   - Add OV3660 FPS values for each resolution
   - Start conservative (match OV2640), optimize later

3. **Add conditional compilation in `main.cpp`:**
   - Camera clock frequency (start with 16-20 MHz)
   - FPS selection logic
   - Resolution mapping (test which work)
   - Status reporting

4. **Update `CameraConfig` structure:**
   - Add `ov3660HighFPS` boolean field

### Phase 3: Test and Optimize

1. **Initial test:** Flash and verify camera detection
2. **Test resolutions:** Start with VGA (640×480), work up to higher res
3. **Test FPS modes:** Normal vs High FPS
4. **Optimize clock:** Test 16 MHz, 18 MHz, 20 MHz, 24 MHz
5. **Measure latency:** Compare to OV2640 and OV5640

---

## Expected OV3660 Performance

Based on specs, OV3660 should perform:
- **Better than OV2640** at all resolutions (newer sensor, 3MP vs 2MP)
- **Similar to OV5640** at VGA/SVGA (both modern sensors)
- **Slightly slower than OV5640** at 720p+ (3MP vs 5MP sensor)

**Target FPS estimates:**

| Resolution | Size | OV3660 FPS (est) | OV3660 High (est) |
|------------|------|------------------|-------------------|
| VGA | 640×480 | 30 | 45-50 |
| SVGA | 800×600 | 30 | 35-40 |
| XGA | 1024×768 | 20-25 | 25-30 |
| HD | 1280×720 | 20-25 | 25-30 |
| SXGA | 1280×960 | 15-20 | 20-25 |

---

## Files to Modify

### Must Modify:
1. `air_firmware_esp32s3sense_ov3660/platformio.ini` - Build flags
2. `components/common/packets.h` - TVMode structure
3. `components/common/packets.cpp` - FPS table
4. `components/air/main.cpp` - Camera init and config logic

### May Need to Modify:
5. `components/air/main.h` - If adding new config options
6. Ground station code - If adding OV3660 detection/display

---

## Testing Checklist

- [ ] Firmware compiles without errors
- [ ] Camera is detected at boot
- [ ] VGA (640×480) streams successfully
- [ ] SVGA (800×600) streams successfully
- [ ] Higher resolutions work (XGA, HD, SXGA)
- [ ] FPS matches expected values
- [ ] High FPS mode works (if supported)
- [ ] Latency is acceptable (<100ms target)
- [ ] Stable for 10+ minutes
- [ ] DVR recording works
- [ ] Image quality is good (focus, exposure, colors)

---

## Next Steps

1. **Option A: Start fresh** - Create OV3660 build from scratch
2. **Option B: Quick test** - Flash OV2640 firmware, see if OV3660 auto-detects
3. **Option C: Full implementation** - Add complete OV3660 support with optimizations

**Recommendation:** Start with Option B (quick test), then move to Option C if needed.

---

**Status:** Analysis complete, ready to implement OV3660 support.
