# CR-Scan Ferret — Next Steps
## Date: 2026-03-20

---

## What Works Now

| Stream | Status | Details |
|--------|--------|---------|
| **Color (RGB)** | **STREAMING** | MJPG 1280x720, 1920x1080 @ 30fps — live video in OrbbecViewer |
| **Accel (IMU)** | **Working** | Frames received, 4 profiles (100/200/500/1000 Hz) |
| **Gyro (IMU)** | **Working** | Frames received, 4 profiles (100/200/500/1000 Hz) |
| Depth | Opens, no frames | Stream opens but zero frames arrive |
| Left IR | Opens, no frames | Same issue as Depth |
| Right IR | Opens, no frames | Same issue as Depth |

All 7 sensors created. Device init completes in ~2 seconds. No USB pipe corruption.

---

## Root Cause of Depth/IR No-Frames (SOLVED — needs implementation)

### Discovery from USB trace analysis (ert.pcapng)

The Creality Scan Windows driver was captured communicating with the Ferret.
**Key finding: the depth camera does NOT use standard UVC streaming.**

- **Color camera (interface 4/5)**: Standard UVC — VS_PROBE/VS_COMMIT → bulk transfer. Works with libuvc.
- **Depth camera (interface 0/1)**: **Orbbec proprietary XU protocol** — all stream control goes through Unit 4 XU commands using the "GM" protocol (magic bytes `0x47 0x4D`).

### Evidence

In the Creality trace, what looks like VS_PROBE/VS_COMMIT on interface 0 is actually:
- `wIndex = 0x0400` = Unit 4, interface 0 → **XU control transfer**, not VS streaming
- Data payload starts with `47 4D` = "GM" = Orbbec vendor protocol magic
- Stream parameters (format, resolution, fps) are encoded in the vendor protocol, not UVC probe format

### What this means

`uvc_stream_open_ctrl()` sends standard UVC VS_PROBE_CONTROL and VS_COMMIT_CONTROL to the depth streaming interface. The device accepts these (doesn't error) but ignores them — it only responds to the vendor XU protocol for stream control. So the USB bulk endpoint never starts sending frame data.

---

## What Needs to Happen

### Step 1: Analyze the vendor stream protocol from the USB trace

Using `ert.pcapng` (USB 2.0) and the upcoming USB 3.0 trace:

1. Identify the XU command sequence that Creality Scan sends to start a depth stream
2. Map the "GM" protocol fields: which bytes control format, resolution, fps
3. Identify the stream start/stop commands
4. Understand how bulk frame data is received (headers, payload format, frame boundaries)

The relevant packets in the trace are the XU SET_CUR/GET_CUR commands on Unit 4
(controls 0x1, 0x2, 0x3) that happen AFTER the initial device enumeration/calibration
and BEFORE frame data flows.

### Step 2: Implement vendor stream start for the Ferret

In `DabaiADevice.cpp` or a new `FerretStreamHandler`:

1. Instead of calling libuvc's `uvc_stream_open_ctrl` for the depth interface,
   send the vendor XU command sequence discovered in Step 1
2. Set up bulk transfer reads on EP1 (0x81) to receive frame data
3. Parse the incoming bulk data into frames (identify header format, frame boundaries)
4. Feed parsed frames into the SDK's existing frame pipeline

### Step 3: Handle depth format conversion

- Raw depth data from the Ferret is Y12/Y14 disparity
- Conversion to depth requires calibration parameters (baseline, focal length)
- Property 4034 returns 416 bytes of calibration data — format needs reverse-engineering
- The AlgParamManager crashes because it expects DaBaiA-format calibration
- Need Ferret-specific calibration parser

### Step 4: Fix AlgParamManager for Ferret calibration

The AlgParamManager reads 8 calibration data blocks. For the Ferret:
- Depth calib (line 174): seems to succeed (no error logged)
- Align calib (line 201): seems to succeed
- Rectify D2C (line 217): seems to succeed
- Transform params (line 233): **FAILS** — "cmd version unmatched: 65535, expect: 0"
- IR rectify (line 249): unknown
- D2C profile list (line 259): succeeds
- D2C preprocess (line 269): **FAILS** — "init get structure data list return cmd version unmatched"
- IMU calib (line 300): **SUCCEEDS**

The crash happens in `fixD2CParmaList()` or `registerBasicExtrinsics()` when processing
the partially-loaded data. Fix: add null checks for empty vectors, or write Ferret-specific
calibration parsing that handles the format differences.

---

## Source Code Status

**Branch**: `cr-scan-ferret-support` at https://github.com/rafik24/OrbbecSDK_v2

**Files modified** (11 files):
1. `src/shared/environment/DeviceInfoConfig.xml` — PID 069C in device lists
2. `src/shared/environment/OrbbecSDKConfig.xml` — CR-ScanFerret config section
3. `src/device/DevicePids.hpp` — OB_DEVICE_CR_SCAN_FERRET_PID constant
4. `src/device/gemini330/DabaiADevice.cpp` — Skip updateXuUnit, AlgParamManager for Ferret
5. `src/device/gemini330/DaBaiAPresetManager.cpp` — try/catch property reads, skip HUE/BACKLIGHT/POWERLINE
6. `src/device/protocol/Protocol.cpp` — errorCode 2 → warning
7. `src/platform/usb/uvc/ObLibuvcDevicePort.cpp` — Skip ALL PU controls for Ferret, getCtrl error handling, getPuRange probe
8. `3rdparty/libuvc/src/src/ctrl.c` — Timeout 6000ms → 100ms
9. `examples/utils/utils_c.c` — termio.h → termios.h fix

**Patches summary**:
- Skip UVC Processing Unit controls for PID 0x069C (prevents USB pipe corruption)
- Use common XU GUID (unit 4) instead of G330 XU (unit 3)
- Treat errorCode 2 (unsupported request) as warning
- Skip AlgParamManager (crash on Ferret calibration format)
- Disable GlobalTimestampFitter in config
- Reduce UVC control timeout to 100ms
- Handle unsupported control sizes gracefully

---

## Available Reference Data

- `ert.pcapng` — USB 2.0 trace of Creality Scan on Windows (155KB)
- USB 3.0 trace — coming soon (user is capturing)
- USB descriptors — fully documented in `progress.md`
- Ferret XU GUID: `a55751a1-f3c5-4a5e-8d5a-6854b8fa2716` (Unit 4)
- Vendor protocol: "GM" magic (0x47 0x4D), uses XU controls 0x1 (512B), 0x2 (64B), 0x3 (1024B)
- Calibration: Property 4034 = 416 bytes, Property 4041 = extension info (parse fails)

---

## Architecture Decision

Two approaches for depth streaming:

**Option A: Vendor protocol in SDK** (recommended)
- Implement the "GM" vendor stream start/stop in the SDK
- Receive bulk data via libusb bulk transfers
- Parse frame data according to vendor format
- Full integration with SDK pipeline

**Option B: Standalone Python tool**
- Use pyusb to replicate Creality Scan's exact USB command sequence
- Receive and save raw bulk data
- Prove concept before integrating into SDK
- Faster to prototype, useful for protocol analysis

**Recommendation**: Start with Option B to understand the full protocol, then implement Option A.

---

## Quick Reference

```
Device:     CR-Scan Ferret
VID/PID:    2bc5:069c
S/N:        BC2T341012J
FW:         1.2.2
USB:        3.0 SuperSpeed

Interfaces:
  0/1: Depth Camera  — Bulk EP1 (0x81) — vendor XU streaming
  2/3: IR Camera     — Bulk EP2 (0x82) — vendor XU streaming
  4/5: RGB Camera    — Bulk EP3 (0x83) — standard UVC streaming ✓
  6:   IMU           — HID interrupt EP5 (0x85) ✓

XU Unit 4 Controls:
  0x1: 512-byte channel
  0x2: 64-byte channel (main command)
  0x3: 1024-byte channel

Vendor Protocol:
  Magic: "GM" (0x47 0x4D)
  Request: XU SET_CUR on control 0x2 (64 bytes)
  Response: XU GET_CUR on control 0x2 (64 bytes) or 0x3 (1024 bytes)
```
