# Undercarriage EV Inspection System

Portable drive-over undercarriage inspection system designed for EV service bays and Supercharger locations. The system captures visible and thermal imagery of the vehicle underbody (battery pack, cooling lines, brakes, bearings, etc.) as the car drives over a low-profile modular ramp.

**Current Status:** Phase 1 Prototype

---

## Goals

| Phase | Goal |
|-------|------|
| **Phase 1** (current) | Capture visible + thermal images and create a basic combined overlay for technician review |
| **Phase 2** | Improve image stitching and add automatic issue detection |
| **Phase 3** | Deploy as a portable unit for free inspections at Superchargers to generate repair business |

---

## Hardware Overview

### Mechanical
- **2 × VEVOR 2-Channel Cable Protectors** (39.6" × 9.45" × 1.77")
  Used as the low-profile drive-over enclosure. Two units placed side-by-side create an approximately 50" imaging strip between the vehicle tires.

### Compute
- **Raspberry Pi 5 (8GB)**
  Main controller. Handles camera capture, GPIO, and processing.

### Cameras
| Type | Model | Qty | Notes |
|------|-------|-----|-------|
| Visible | Arducam Camera Module 3 Wide (IMX708) | 2 | ~120° FOV, autofocus, CSI |
| Thermal | MLX90640-D110 | 2 | 32×24 resolution, 110° FOV, I2C |

> Starting with 2 visible + 2 thermal cameras. Designed for easy expansion to 3+3 later.

### Lighting
- IP68 Waterproof COB LED Strip (12V, 6000K, high CRI)
- 12V 60W Waterproof Power Supply
- Dual MOSFET modules for GPIO-controlled switching

### Sensing & Control
- **RCWL-0516** Microwave radar sensor – vehicle detection
- **TCA9548A** I2C multiplexer – allows multiple MLX90640 sensors on one bus
- Hookup wire kit

---

## Software Architecture (Phase 1)

The main script (`app/main.py`) performs the following sequence:

1. Continuously monitors the RCWL-0516 radar
2. When a vehicle is detected:
   - Turns on the LED strip
   - Captures calibrated visible/thermal pairs while a vehicle crosses the scanner
3. When capture is complete:
   - Turns off the LEDs
   - Rejects insufficiently sampled or registered passes, otherwise writes one calibrated inspection mosaic
4. Returns to waiting for the next vehicle

### Key Features of the Script
- Explicit visible-camera/TCA-channel pairing in `config/inspection_layout.json`
- Calibrated visible and thermal projection into one vehicle-coordinate canvas
- Fixed Celsius heatmap scale, temperature array, coverage mask, and capture metadata
- Motion-derived speed validation so passes with gaps are rejected rather than stitched incorrectly
- All inspection artifacts saved with timestamps in `/captures`

### Calibration And Speed Limits

`config/inspection_layout.json` is intentionally shipped with `"calibrated": false`, so the scanner will not create uncalibrated inspection images. Hardware engineers must populate it before deployment:

- `pairs` explicitly maps a visible camera index to a TCA9548A channel. Add a third center pair as another object; do not rely on matching indexes.
- `visible_to_canvas` and `thermal_to_canvas` are each pair's 3x3 homographies into the shared top-down canvas.
- `motion_to_canvas` converts phase-correlation movement from the selected motion camera into canvas pixels, including direction and scale.
- `inspection_roi` and coverage thresholds define the required technician-review area.
- `max_traversal_speed_mps` and `max_motion_step_pixels` reject a pass when consecutive samples cannot cover the canvas without a gap.

The RCWL-0516 is presence-only; it cannot report vehicle speed. The application estimates speed from registered camera motion and records average and maximum speed in each event's metadata JSON.

See [`CALIBRATION.md`](CALIBRATION.md) for the required transform, coverage, and speed-limit measurements.

---

## Docker Deployment

Build and run on a 64-bit Raspberry Pi OS Bookworm host. The image starts from
official Debian Bookworm and adds Raspberry Pi's official APT archive because
`Picamera2` and its matching `libcamera` stack are not available from upstream
Debian. Compiled imaging packages are installed with `apt` to keep their NumPy
ABI compatible.

```bash
docker compose build
docker compose up

## Bench And Viewer Console

The default `SCANNER_MODE=bench` starts a browser console at
`http://<pi-address>:8080`. It opens only the sources listed in
`config/bench.json`, so use the shipped direct-I2C thermal entry for initial
one-sensor bench work without the TCA9548A, radar, or LEDs.

The direct MLX90640 test must be the only same-address thermal sensor connected
to the Pi I2C bus. When the TCA9548A is installed, replace the direct source in
`config/bench.json` with one or more `"kind": "tca9548a"` sources and their
channels.

For a direct thermal bench test outside Docker, install Blinka's Raspberry Pi
GPIO backend with `sudo apt install -y python3-lgpio`. The Docker image already
includes this package.

The console provides live visible and thermal previews, saves bench snapshots,
supports point anchoring plus drag/scale/rotate relative registration, and
lists deployed inspection mosaics. Saving a transform never enables deployed capture: it keeps
`calibrated: false` until all calibration values are reviewed.

MLX90640 reads are stored as 32x24 Celsius arrays after acquisition. The
underlying Adafruit driver requires a flat 768-value buffer, so do not pass it a
pre-shaped 32x24 array when adding thermal capture code.

Use viewer-only mode when the deployed scanner process owns the hardware:

```bash
SCANNER_MODE=viewer docker compose up
```

Run unattended capture only after calibration:

```bash
SCANNER_MODE=deployed docker compose up
```
