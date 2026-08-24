# Drive-Over Undercarriage Scanner

## Entry Point And Runtime

- The only application source is `app/main.py`; run it on the Raspberry Pi with `python app/main.py`. It requires live CSI cameras, GPIO, and I2C hardware, so it is not a host-only smoke test.
- Docker starts `app.entrypoint`: `SCANNER_MODE=bench` (default) serves the browser bench/calibration console, `viewer` serves saved inspection artifacts without opening hardware, and `deployed` runs the radar capture loop. Do not run bench and deployed modes together because both open the cameras.
- `config/bench.json` is intentionally independent of deployed calibration and supports direct or TCA9548A thermal sources for bring-up. A direct MLX90640 is valid only when it is the sole same-address sensor on the Pi I2C bus.
- Bench visible and thermal sources are explicitly named and mapped to `Picamera2` indexes and TCA9548A channels; do not infer a pair from list order. A thermal-only bench source cannot enter deployed mosaics until its visible-camera pair is added to `inspection_layout.json`.
- The uncalibrated `center` layout entry is a V4L2 USB-camera template paired to thermal channel `2`; replace its stable by-id path and add the matching `/dev/videoX` Compose mapping before deployment.
- Bench mode can optionally read RCWL-0516 GPIO17 when `config/bench.json` enables its radar entry; this is status-only and must not start deployed capture.
- The capture loop is hardware-triggered: wait for RCWL-0516 to clear -> wait for detection -> LEDs on -> capture until it clears or `maximum_capture_duration_seconds` -> LEDs off -> validate movement/coverage -> write one inspection mosaic. Keep `SIGTERM`/exception cleanup of LEDs and `Picamera2` instances reliable when changing this flow.
- Tuning constants, BCM GPIO pins, thermal dimensions, output location, and the calibrated-mosaic flow are in `app/main.py`. Captures are written to `captures/` at the repository root.

## Hardware Constraints

- `config/inspection_layout.json` explicitly pairs visible source descriptors (`picamera2` CSI/HAT indexes or `v4l2` USB device paths) with TCA9548A channels, and holds every calibrated transform. Keep it `calibrated: false` until all transforms and coverage thresholds are measured; the application must refuse to emit an uncalibrated inspection image.
- `CALIBRATION.md` defines transform, coverage, and speed-limit requirements. Recalibrate after a camera, sensor, focus, orientation, or mounting-height change.
- MLX90640 sensors share an I2C address and must remain behind the TCA9548A. Thermal frames are 32x24 and the current setup uses channels `0` and `1` at 4 Hz.
- `adafruit_mlx90640.MLX90640.getFrame` requires a flat 768-value buffer; reshape it to 32x24 only after acquisition.
- The RCWL-0516 is presence-only. Traversal speed is measured from the calibrated motion camera; passes exceeding the configured speed, frame displacement, visible/thermal skew, or coverage limits are rejected.

## Container Runtime

- Build with `docker compose build` on a 64-bit Raspberry Pi OS Bookworm host. The Dockerfile starts from official Debian and adds Raspberry Pi's official APT archive because upstream Debian lacks `Picamera2`.
- Install `picamera2`, `gpiozero`, `lgpio`, `RPi.GPIO`, NumPy, Pillow, and OpenCV with `apt`; do not install or upgrade them with `pip`, because `Picamera2` includes compiled extensions tied to the distro NumPy ABI.
- The compose service is deliberately privileged and maps I2C, GPIO, and two video devices. Add the resolved `/dev/videoX` mapping when adding a V4L2/USB visible source.

## Dependencies

- Runtime Python dependencies are pinned only by package name in `requirements.txt`; any new hardware library must be added there and documented in `README.md`.
