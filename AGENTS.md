# Drive-Over Undercarriage Scanner

## Entry Point And Runtime

- The only application source is `app/main.py`; run it on the Raspberry Pi with `python app/main.py`. It requires live CSI cameras, GPIO, and I2C hardware, so it is not a host-only smoke test.
- Docker starts `app.entrypoint`: `SCANNER_MODE=bench` (default) serves the browser bench/calibration console, `viewer` serves saved inspection artifacts without opening hardware, and `deployed` runs the radar capture loop. Do not run bench and deployed modes together because both open the cameras.
- `config/bench.json` is intentionally independent of deployed calibration and supports direct or TCA9548A thermal sources for bring-up. A direct MLX90640 is valid only when it is the sole same-address sensor on the Pi I2C bus.
- The capture loop is hardware-triggered: wait for RCWL-0516 to clear -> wait for detection -> LEDs on -> capture until it clears or `maximum_capture_duration_seconds` -> LEDs off -> validate movement/coverage -> write one inspection mosaic. Keep `SIGTERM`/exception cleanup of LEDs and `Picamera2` instances reliable when changing this flow.
- Tuning constants, BCM GPIO pins, thermal dimensions, output location, and the calibrated-mosaic flow are in `app/main.py`. Captures are written to `captures/` at the repository root.

## Hardware Constraints

- `config/inspection_layout.json` explicitly pairs visible camera indexes with TCA9548A channels, and holds every calibrated transform. Keep it `calibrated: false` until all transforms and coverage thresholds are measured; the application must refuse to emit an uncalibrated inspection image.
- `CALIBRATION.md` defines transform, coverage, and speed-limit requirements. Recalibrate after a camera, sensor, focus, orientation, or mounting-height change.
- MLX90640 sensors share an I2C address and must remain behind the TCA9548A. Thermal frames are 32x24 and the current setup uses channels `0` and `1` at 4 Hz.
- The RCWL-0516 is presence-only. Traversal speed is measured from the calibrated motion camera; passes exceeding the configured speed, frame displacement, visible/thermal skew, or coverage limits are rejected.

## Container Runtime

- Build with `docker compose build` on a 64-bit Raspberry Pi OS Bookworm host. The Dockerfile starts from official Debian and adds Raspberry Pi's official APT archive because upstream Debian lacks `Picamera2`.
- Install `picamera2`, `gpiozero`, `lgpio`, `RPi.GPIO`, NumPy, Pillow, and OpenCV with `apt`; do not install or upgrade them with `pip`, because `Picamera2` includes compiled extensions tied to the distro NumPy ABI.
- The compose service is deliberately privileged and maps I2C, GPIO, and two video devices. Add the matching `/dev/videoX` mapping when adding a visible camera pair.

## Dependencies

- Runtime Python dependencies are pinned only by package name in `requirements.txt`; any new hardware library must be added there and documented in `README.md`.
