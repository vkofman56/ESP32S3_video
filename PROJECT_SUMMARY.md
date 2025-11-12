# ESP32-S3 Video Streaming Project - Context Summary

**Created:** November 12, 2025
**Purpose:** Low-latency video streaming for AI-controlled drone flight

---

## Problem We're Solving

### Current Situation
- **VTX analog video setup:** 170-250ms latency (Camera → VTX → RX → USB Capture → PC)
- **Too slow for AI control:** Need faster visual feedback for responsive autonomous flight
- **Complex hardware:** Requires video capture card, analog-to-digital conversion

### Goal
- **Reduce video latency to 60-100ms** (2-4x improvement)
- Digital video stream directly to PC via WiFi
- Easier integration with AI processing pipeline (Python/OpenCV)

---

## Current Working Telemetry Setup (Reference)

**Working Configuration (from ESP-NOW_MAVLink repository):**
- **Hardware:** ESP32-C3 Super Mini (drone) + XIAO ESP32-C3 (ground - in development)
- **Firmware:** DroneBridge for ESP32 v2.1.0
- **Mode:** WiFi Station (connecting to office WiFi router)
- **Baud Rate:** 460800 (critical - lower rates like 57600 don't work)
- **Connection:** MAVProxy via TCP at 192.168.86.94:5760
- **Performance:** 0.2-0.5% packet loss, 15-42ms latency ✅
- **Status:** WORKING and proven reliable

**Key Learning:** External WiFi router is critical - ESP32-C3 as Access Point was unreliable (45-50% packet loss).

---

## Video Streaming Solution - ESP32-S3

### Why ESP32-S3?
- ✅ Has DVP camera interface (ESP32-C3 does NOT)
- ✅ Dual-core 240MHz (can handle video + telemetry)
- ✅ PSRAM support (needed for camera frame buffering)
- ✅ Proven low-latency video projects exist (esp32-cam-fpv: 20-60ms)

### Architecture Options

**Option A: Dual-Board Setup (RECOMMENDED for testing)**
```
Drone:
  - ESP32-C3 (telemetry only) - DroneBridge firmware ← Proven working
  - ESP32-S3 + Camera (video only) - esp32-cam-fpv firmware ← Proven working

Benefits: Zero development, proven firmware, maximum reliability
Cost: $6 + $15 = $21
Weight: 6-9g
```

**Option B: Single-Board Setup (Future optimization)**
```
Drone:
  - ESP32-S3 + Camera (telemetry + video) - Custom firmware ← Need to develop

Benefits: Lighter (4-6g), cheaper ($15), simpler wiring
Development time: 10-16 hours
Risk: Video processing might affect telemetry reliability
```

**Decision:** Start with Option A (dual-board), optimize to Option B if needed.

---

## Hardware Decisions Made

### Camera Module
- **OV2640** (2MP, 640x480@30fps) - recommended
- Cost: $5-8
- Well-supported by esp32-camera library

### ESP32-S3 Board
- **ESP32-S3-DevKitC-1** with 8MB PSRAM (~$10-12)
- OR **XIAO ESP32-S3** (~$7-8) - more compact
- **CRITICAL:** Must have PSRAM for camera buffering

### Antenna
- **CORONIR IPEX 3dBi** (3-pack ~$9) ← BEST CHOICE
- Lightweight copper tube design (~0.5-1g)
- No plastic housing (vs Seeed official antenna ~2-3g)
- Slightly better gain: 3dBi vs 2.81dBi
- Amazon link: https://www.amazon.com/CORONIR-Bluetooth-Transmission-Omnidirectional-Receiver/dp/B095N2DV2B

**Why 3dBi over 10dBi?**
- 3dBi = omnidirectional (works at any drone orientation)
- 10dBi = directional (loses signal when drone tilts/banks)
- **Solution:** 3dBi on drone, 10dBi on ground station (best of both)

### WiFi Frequency
- **2.4GHz** (not 5GHz) - ESP32 only supports 2.4GHz anyway
- Better range (200-400m vs 100-200m for 5GHz)
- Better obstacle penetration (critical for outdoor flight)
- 5GHz would be worse for drones despite lower latency indoors

---

## Key Technical Findings

### ESP32 Chip Comparison for This Project

| Feature | ESP32-C3 | ESP32-C6 | ESP32-S3 |
|---------|----------|----------|----------|
| **Telemetry Only** | ✅ Perfect | ⚠️ Minimal benefit | ❌ Overkill |
| **Camera Support** | ❌ No | ❌ No | ✅ Yes (DVP) |
| **WiFi 6** | No | Yes | No |
| **Power (WiFi active)** | 80mA | 85mA | 100mA |
| **Weight** | 1-2g | 1-2g | 3-5g |
| **Cost** | $2-3 | $4-6 | $10-15 |
| **Use Case** | Telemetry | Telemetry (WiFi 6 network) | Video streaming |

**Conclusion:**
- **C3:** Best for telemetry-only (current working setup)
- **C6:** Only worth it if office WiFi is WiFi 6 AND congested (marginal improvement)
- **S3:** Required for camera/video, but overkill for telemetry alone

### Power Consumption (2-minute flight)

| Board | Current | Energy (2 min) | Flight Time Impact |
|-------|---------|----------------|-------------------|
| ESP32-C3 | 80mA | 2.67 mAh | Baseline |
| ESP32-S3 | 100mA | 3.33 mAh | ~1-2 seconds less |

**Finding:** Power difference is negligible (~0.5% of total drone power). **Weight penalty (2-3g) is much more significant** (2-4% flight time impact on 100g drone).

### Expected Latency Comparison

| Setup | Video Latency | Telemetry | Total AI Loop |
|-------|---------------|-----------|---------------|
| Current VTX | 170-250ms | 15-42ms | 200-300ms |
| ESP32-S3 (optimized) | 20-60ms | 15-42ms | 50-100ms |
| ESP32-S3 (Arduino) | 100-150ms | 15-42ms | 120-180ms |

**Target:** 50-100ms total for responsive AI control (2-4x improvement)

---

## Development Plan Overview

**Full details in:** `ESP32_S3_UNIFIED_FIRMWARE_PLAN.md` (1,303 lines)

### Phase A: Quick Prototype (2-3 hours)
- Arduino-based simple version
- Test if ESP32-S3 can handle both tasks simultaneously
- Measure actual packet loss and latency
- **Goal:** Prove the concept is viable

### Phase B: Optimized Firmware (10-16 hours)
- Fork esp32-cam-fpv repository
- Add MAVLink telemetry handling
- Dual-core architecture (Core 0: camera, Core 1: telemetry)
- Achieve 20-60ms video latency
- **Goal:** Production-ready firmware

### Testing Strategy
1. Start with proven dual-board setup (C3 + S3)
2. Verify video helps AI performance
3. If weight/cost matters, develop single-board firmware
4. Stress test: parameter download while streaming video

---

## Important Context from Previous Work

### ESP-NOW Initial Attempts (Failed)
- Custom ESP-NOW firmware: 50% packet loss at 1 meter ❌
- DroneBridge ESP-NOW mode: 45% loss, 140ms+ latency ❌
- **Root cause:** ESP32-C3 as WiFi AP is too weak for continuous telemetry

### DroneBridge WiFi Station (Success)
- Switched to external WiFi router architecture ✅
- Achieved 0.5% packet loss, 15-42ms latency ✅
- Works reliably for AI control at 10Hz

### Baud Rate Discovery
- 57600: No connection ❌
- 115200: Not tested
- **460800: Works with 0.5% loss** ✅
- 921600: Works with slightly higher loss (54%)

**Finding:** Higher baud rates work better than expected on DroneBridge + ESP32-C3.

---

## Bill of Materials (BOM)

### Option A: Dual-Board Testing Setup

| Item | Qty | Unit $ | Total | Notes |
|------|-----|--------|-------|-------|
| ESP32-C3 (telemetry) | 1 | $6 | $6 | XIAO ESP32-C3 preferred |
| ESP32-S3 + PSRAM (video) | 1 | $12 | $12 | DevKitC-1 or XIAO |
| OV2640 Camera | 1 | $6 | $6 | 2MP, DVP interface |
| CORONIR 3dBi Antenna | 2 | $3 | $6 | 3-pack = spares |
| Dupont wires | ~15 | $0.10 | $2 | Connections |
| **Total** | | | **$32** | |

**Weight:** 6-9g total

### Option B: Single-Board (Future)

| Item | Qty | Unit $ | Total | Notes |
|------|-----|--------|-------|-------|
| ESP32-S3 + PSRAM | 1 | $12 | $12 | Both tasks on one board |
| OV2640 Camera | 1 | $6 | $6 | 2MP, DVP interface |
| CORONIR 3dBi Antenna | 1 | $3 | $3 | Lightweight |
| Dupont wires | ~10 | $0.10 | $1 | Connections |
| **Total** | | | **$22** | |

**Weight:** 4-6g total (2-3g lighter)
**Savings:** $10 cheaper, but requires custom firmware development

---

## Next Steps

### Immediate (When Hardware Arrives)

1. **Order hardware:**
   - [ ] XIAO ESP32-C3 + external antenna ($6 + $2)
   - [ ] ESP32-S3-DevKitC-1 with PSRAM ($12)
   - [ ] OV2640 camera module ($6)
   - [ ] CORONIR IPEX 3dBi antennas 3-pack ($9)

2. **Test dual-board setup:**
   - [ ] Flash DroneBridge on C3 (telemetry)
   - [ ] Flash esp32-cam-fpv on S3 (video)
   - [ ] Test both simultaneously on office WiFi
   - [ ] Measure combined latency and packet loss

3. **Validate for AI:**
   - [ ] Integrate video stream into AI pipeline
   - [ ] Test control loop responsiveness
   - [ ] Compare to current VTX setup
   - [ ] Decide if improvement justifies the approach

### If Video Proves Valuable

4. **Optimize (optional):**
   - [ ] Decide: keep dual-board OR develop single-board firmware
   - [ ] If single-board: follow Phase A → Phase B in development plan
   - [ ] Stress test: parameter downloads during video streaming
   - [ ] Long-duration flight testing (30+ minutes)

5. **Production:**
   - [ ] Mount hardware on drone (compact, secure)
   - [ ] Outdoor range testing
   - [ ] Failsafe testing (WiFi dropout scenarios)
   - [ ] Document final configuration

---

## Reference Links

### Hardware
- **CORONIR Antenna:** https://www.amazon.com/CORONIR-Bluetooth-Transmission-Omnidirectional-Receiver/dp/B095N2DV2B
- **XIAO ESP32-C3:** https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html
- **ESP32-S3-DevKitC-1:** Search "ESP32-S3-DevKitC-1 8MB PSRAM" on Amazon/AliExpress

### Software/Projects
- **DroneBridge for ESP32:** https://github.com/DroneBridge/ESP32
- **esp32-cam-fpv (low latency):** https://github.com/RomanLut/hx-esp32-cam-fpv
- **ESP32-Camera Library:** https://github.com/espressif/esp32-camera
- **MAVLink Protocol:** https://mavlink.io/en/

### Related Repositories
- **ESP-NOW_MAVLink:** https://github.com/vkofman56/ESP-NOW_MAVLink (telemetry development)
- **ESP32S3_video:** https://github.com/vkofman56/ESP32S3_video (this project)

---

## Success Criteria

### Minimum Viable (Testing Phase)
- ✅ Video streaming at 15+ FPS
- ✅ Telemetry packet loss < 1%
- ✅ Both operating simultaneously without interference
- ✅ Video latency < 150ms (acceptable)
- ✅ Stable for 10+ minutes

### Production Ready (Deployment)
- ✅ Video latency < 100ms (target)
- ✅ Telemetry packet loss < 0.5%
- ✅ Stable for 30+ minutes continuous operation
- ✅ Works at 50m+ range
- ✅ AI control loop responsive (<100ms total)
- ✅ Reliable reconnection after dropout

### Optimal (Stretch Goals)
- ✅ Video latency < 60ms (optimal)
- ✅ Telemetry latency < 15ms
- ✅ Packet loss < 0.2%
- ✅ Works at 200m+ range
- ✅ Single-board solution (weight optimized)
- ✅ Hours of stable operation

---

## Key Reminders for Future Sessions

1. **Always read this file first** to get context
2. **Reference ESP32_S3_UNIFIED_FIRMWARE_PLAN.md** for technical details
3. **Current telemetry works** - don't break it! (460800 baud, DroneBridge WiFi Station)
4. **Start with dual-board approach** - proven firmware, zero risk
5. **CORONIR antenna is best** for drone (lightweight, no plastic housing)
6. **3dBi on drone, 10dBi on ground** for optimal range
7. **2.4GHz only** - better for drones than 5GHz
8. **C3 is best for telemetry** - S3 only needed for camera
9. **Test video value before optimizing** to single board

---

## Questions to Answer in Future Sessions

- [ ] Does 60-100ms video actually improve AI control? (vs 170-250ms)
- [ ] Can ESP32-S3 handle both tasks reliably? (telemetry + video)
- [ ] Is 640x480@30fps sufficient for AI vision? (or need higher res?)
- [ ] What's actual outdoor range with 3dBi antenna?
- [ ] Does single-board save enough weight to justify development time?

---

**Status:** Planning complete, ready for hardware procurement and testing.

**Last Updated:** November 12, 2025
**Next Session:** Review this summary, then proceed with hardware ordering or development.
