# Drive-Over Undercarriage Scanner

A Raspberry Pi-based visible and thermal scanner for capturing a vehicle's
undercarriage as it drives over a calibrated sensor strip. It is currently a
hardware-dependent prototype for Raspberry Pi OS Bookworm hosts.

## What You Need

- 64-bit Raspberry Pi OS Bookworm with Docker Compose.
- Connected CSI or V4L2 visible cameras, MLX90640 thermal sensors, and I2C.
- The optional RCWL-0516 presence sensor on BCM GPIO17.
- A completed, reviewed calibration before deployed capture.

The application needs live cameras, GPIO, and I2C. It is not a host-only smoke
test. The container runs privileged and maps the hardware devices declared in
`docker-compose.yml`.

## Quick Start

Build and start the browser bench console:

```bash
docker compose up --build
```

Open `http://<pi-address>:8080`.

The default mode is `bench`. It previews only sources named in
`config/bench.json` and never begins deployed capture.

## Runtime Modes

| Mode | Command | Purpose |
| --- | --- | --- |
| Bench | `docker compose up` | Live hardware preview, snapshots, and calibration. |
| Viewer | `SCANNER_MODE=viewer docker compose up` | Browse saved artifacts without opening hardware. |
| Deployed | `SCANNER_MODE=deployed docker compose up` | Radar-triggered inspection capture. |

Do not run bench and deployed modes together: both open the camera hardware.

## Configure Sources

`config/bench.json` controls bench previews. Visible and thermal sources are
explicitly named and paired; do not infer a pair from list order.

`config/inspection_layout.json` controls deployed capture. It maps every
visible source to one TCA9548A thermal channel and stores the calibration
transforms. The shipped layout is deliberately uncalibrated.

The current layout reserves the center pair for thermal channel `2` and a USB
visible camera. Before deployment, replace
`/dev/v4l/by-id/REPLACE_WITH_CENTER_CAMERA` with the stable by-id device path
and add its resolved `/dev/videoX` device to `docker-compose.yml`.

The mounted `config/` and `captures/` directories persist configuration and
inspection artifacts outside the container.

## Calibrate Before Deployment

The scanner refuses to create deployed inspection mosaics until
`config/inspection_layout.json` is reviewed and marked with
`"calibrated": true`.

Use the bench console to:

1. Verify every visible and thermal source.
2. Calibrate each visible lens with the checkerboard workflow.
3. Place visible and thermal fields into the fixed strip editor.
4. Calibrate motion with a known-distance pass.
5. Review coverage and speed limits, then enable the layout.

See [Calibration](docs/CALIBRATION.md) for the complete procedure.

## Inspection Output

Each accepted deployed pass writes timestamped artifacts to `captures/`,
including the visible mosaic, thermal data, coverage mask, and metadata. Passes
that exceed configured movement, speed, source-skew, sharpness, or coverage
limits are rejected rather than stitched into a misleading inspection.

## Documentation

- [System design](docs/SYSTEM_DESIGN.md): platform, sensor-strip, and deployment architecture.
- [Camera placement](docs/CAMERA_PLACEMENT.md): placement targets, tilt, ramp geometry, and optional side views.
- [Approach lighting](docs/APPROACH_LIGHTING.md): approach mat and driver-facing light states.
- [Calibration](docs/CALIBRATION.md): required visible, thermal, strip, and motion calibration.
- [Repository guidance](AGENTS.md): hardware and runtime constraints for contributors.
