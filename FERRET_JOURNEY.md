# CR-Scan Ferret — The Full Journey

## From "No matched USB device" to All 6 Streams Working

**Device:** Creality CR-Scan Ferret (USB 2bc5:069c, manufactured by Orbbec)
**Date:** 2026-03-19 to 2026-03-20
**Result:** All streams working — Depth, Color, Left IR, Right IR, Accel, Gyro

---

## The Problem

The CR-Scan Ferret is a 3D scanner sold by Creality but manufactured by Orbbec.
It uses Orbbec's SDK internally but is only supported through Creality's proprietary
Windows-only scanning software. The goal: get it working on Linux with the open-source
OrbbecSDK v2.7.6.

When OrbbecViewer launched, it could SEE the device by name ("CR-Scan Ferret Depth
Camera", "CR-Scan Ferret IR Camera", etc.) but dropped it with:
```
No matched usb device found!
Current found device(s): (0)
```

The SDK recognized the device strings from USB descriptors but PID 0x069C wasn't
in any device whitelist.

---

## Phase 1: Getting the Device Recognized (Binary Patches)

### Patch 1: PID Whitelist
Created `DeviceInfoConfigOEM.xml` to register PID 0x069C in the SDK's device
matching tables. The SDK has an OEM override mechanism that loads this file from
the working directory.

**Result:** Device found! But `set xu failed` — wrong XU GUID.

### Patch 2: XU GUID
The SDK's DaBaiA handler sets XU to GUID `c9606ccb...` (G330 XU, Unit 3), but
the Ferret uses the common Orbbec XU GUID `a55751a1...` (Unit 4). Discovered by
reading the USB descriptors (`lsusb -v`).

Binary-patched 3 locations in libOrbbecSDK.so to change Unit 3 → 4 and the GUID.

**Result:** XU communication working! But `errorCode: 2` throws killed init.

### Patch 3: Error Code Suppression
The Ferret firmware returns errorCode 2 (HP_RESP_ERROR_UNSUPPORTED_REQUEST) for
many vendor properties. The SDK's `checkStatus()` threw fatal exceptions.

Reverse-engineered the `checkStatus` function via objdump to find that errorCode 2
maps to statusCode 9 (not 10 as initially assumed — discovered by tracing the
`validateResp()` function which does `lea -0x8000(%rax),%edx; add $0x9,%edx`).

Binary-patched the `je` for case 9 to redirect to the return path.

**Result:** Init gets further! But 6-second UVC timeouts kill everything.

### Patch 4: UVC Timeout
The Ferret's RGB camera doesn't support HUE, BACKLIGHT, POWER_LINE_FREQUENCY
UVC controls. Each unsupported query waited 6 seconds (hardcoded in Orbbec's
bundled libuvc). Three timeouts × 6 seconds = 18 seconds blocking the USB bus.

Binary-patched the timeout from 6000ms to 100ms.

**Result:** Device creates successfully! Sensors enumerated. But streams fail.

---

## Phase 2: Source Code Patches

### Building from Source
Cloned OrbbecSDK_v2 from GitHub, fixed `termio.h` → `termios.h`, built clean.
Pushed to fork at https://github.com/rafik24/OrbbecSDK_v2

### The PU Control Discovery
Even with all binary patches, `uvc_stream_open_ctrl` failed for all streams.
USB pipe corruption: after UVC Processing Unit control timeouts on the RGB camera,
ALL subsequent XU commands returned "bad magic" (zeros).

**Key evidence from Creality Scan USB trace (ert.pcapng):**
The Windows driver NEVER queries ANY UVC Processing Unit controls — it only uses
the depth camera's XU (Unit 4, controls 0x1/0x2/0x3) with the vendor "GM" protocol.

**Fix:** Skip ALL PU control queries (`getPu`/`setPu`/`getPuRange`) for PID 0x069C.

**Result:** Color stream works! But Depth/IR get no frames.

### The Vendor Stream Enable Discovery
Analyzed the USB 3.0 trace from Creality Scan. Discovered that the depth/IR
cameras require a **vendor XU command** to enable streaming BEFORE standard UVC
VS_PROBE/VS_COMMIT will work:

1. Send GM command: SET property 0x000E = 1 (depth stream enable)
2. Send GM command: SET property 0x0051 = 1 (IR stream enable)
3. THEN standard UVC probe/commit
4. Bulk frame data flows

Proved with a Python POC (`ferret_stream_poc.py`) that received IR frames.

**Fix:** In `ObLibuvcDevicePort::startStream()`, send the vendor XU enable command
before calling `uvc_get_stream_ctrl_format_size()` for the Ferret.

**Result:** ALL video streams working — Depth, Color, Left IR, Right IR!

### Calibration Data
Re-enabled AlgParamManager with try/catch safety wrapper. The calibration data
loads successfully:

```
Depth: baseline=150.04mm, focalPix=956.62, z0=658.42, unit=0.5mm
Depth intrinsics: fx=956.62, fy=956.62, cx=638.94, cy=407.10 (1280x800)
RGB intrinsics:   fx=847.49, fy=847.33, cx=632.66, cy=405.22 (1280x800)
```

**Result:** Depth disparity-to-depth conversion works. IMU values display correctly.

---

## Known Issues

1. **Segfault when opening Depth then Color** — Race condition in AlgParamManager
   component access during concurrent stream startup. The `getComponentT` calls
   need thread-safety guards. Workaround: open streams one at a time.

2. **Kernel UVC driver conflict** — Must unload `uvcvideo` module before running:
   `sudo modprobe -r uvcvideo`. The kernel driver claims the video interfaces
   and conflicts with libuvc.

3. **AlgParamManager partial parse** — Some calibration properties fail:
   - Transform params: "cmd version unmatched: 65535, expect: 0"
   - D2C preprocess: "init get structure data list return cmd version unmatched"
   These are caught by try/catch and don't affect basic streaming.

4. **OrbbecViewer "not responding" dialog** — The many vendor property reads during
   init (each ~100μs) accumulate to a few seconds on the UI thread.

---

## Files Modified (12 files)

1. `src/shared/environment/DeviceInfoConfig.xml` — PID 069C in device lists
2. `src/shared/environment/OrbbecSDKConfig.xml` — CR-ScanFerret config section
3. `src/device/DevicePids.hpp` — OB_DEVICE_CR_SCAN_FERRET_PID constant
4. `src/device/gemini330/DabaiADevice.cpp` — Skip updateXuUnit, AlgParamManager wrapper, vendor stream enable guards
5. `src/device/gemini330/DaBaiAPresetManager.cpp` — try/catch property reads, skip timeout-causing PU controls
6. `src/device/protocol/Protocol.cpp` — errorCode 2 → warning instead of throw
7. `src/platform/usb/uvc/ObLibuvcDevicePort.cpp` — Skip PU controls, getCtrl error handling, getPuRange probe, vendor stream enable
8. `3rdparty/libuvc/src/src/ctrl.c` — Timeout 6000ms → 100ms
9. `examples/utils/utils_c.c` — termio.h → termios.h
10. `next_steps.md` — Protocol analysis documentation
11. `ferret_stream_poc.py` — Python POC for vendor stream start
12. `FERRET_JOURNEY.md` — This file

---

## Technical Reference

```
Device:     CR-Scan Ferret
VID/PID:    2bc5:069c
S/N:        BC2T341012J
FW:         1.2.2

XU Unit 4 (common Orbbec, GUID a55751a1-f3c5-4a5e-8d5a-6854b8fa2716):
  Control 0x1: 512-byte channel
  Control 0x2: 64-byte command channel (main)
  Control 0x3: 1024-byte channel

Vendor Protocol: "GM" magic (0x47 0x4D) / "RB" response (0x52 0x42)
Stream Enable: Property 0x000E (depth), Property 0x0051 (IR)

Calibration (S/N: BC2T341012J):
  Baseline:    150.045 mm
  Focal:       956.615 px
  Depth unit:  0.5 mm
  Depth:       fx=956.62 fy=956.62 cx=638.94 cy=407.10 (1280x800)
  RGB:         fx=847.49 fy=847.33 cx=632.66 cy=405.22 (1280x800)
```

---

## Segfault Fix

The pre-compiled `libFilterProcessor.so` extension (closed-source, shipped with
OrbbecViewer) crashes when processing Ferret depth frames. The segfault occurs in
the disparity-to-depth filter at a fixed code offset (+0x195ec).

**Fix:** Disable the extension:
```bash
mv extensions/filters/libFilterProcessor.so extensions/filters/libFilterProcessor.so.disabled
```

This removes depth post-processing filters (noise removal, hardware D2C) but
eliminates the crash. All streams work stably with this workaround.

## Startup Checklist

```bash
# 1. Unload kernel UVC driver (conflicts with libuvc)
sudo modprobe -r uvcvideo

# 2. Optional: blacklist permanently
echo "blacklist uvcvideo" | sudo tee /etc/modprobe.d/blacklist-uvcvideo.conf

# 3. Run OrbbecViewer
cd ~/Downloads/OrbbecViewer_v2.7.6_202602021245_20730ef_linux_x86_64/
./OrbbecViewer
```
