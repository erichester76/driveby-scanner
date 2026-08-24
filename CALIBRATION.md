# Inspection Mosaic Calibration

The scanner will not create an inspection image until
`config/inspection_layout.json` has calibrated transforms and
`"calibrated": true`.

## Pair Mapping

Each `pairs` entry is one physical visible/thermal unit. `name`, the visible
source identity, and `thermal_tca_channel` must be unique. Add a center unit
with a `visible` descriptor and a TCA9548A channel. CSI/HAT sources use
`{"kind": "picamera2", "index": 2, "size": [...]}`; USB UVC sources use
`{"kind": "v4l2", "device": "/dev/v4l/by-id/...", "size": [...]}`. For a
USB source, also map the resolved `/dev/videoX` in `docker-compose.yml`.

## Canvas Transforms

Use a fixed top-down calibration target at the undercarriage plane. The shared
canvas uses pixels as its coordinate system.

- `visible_camera_matrix` and `visible_distortion_coefficients`: visible-lens
  intrinsics at the configured 2304x1296 capture resolution. The application
  undistorts before projection and motion registration.
- `visible_to_canvas`: 3x3 homography from each undistorted visible image to
  the canvas.
- `thermal_camera_matrix` and `thermal_distortion_coefficients`: thermal-lens
  intrinsics at 32x24.
- `thermal_to_canvas`: 3x3 homography from each 32x24 thermal frame to the
  same canvas after undistortion. Measure it independently; resizing thermal
  data to the visible image is not calibration.
- `motion_to_canvas`: 2x2 transform from the motion camera's phase-correlation
  displacement to canvas displacement. It sets scale and travel direction.
- `pixels_per_meter`: measured canvas scale used for traversal-speed reporting.

Keep a calibrated target image and transform-generation record with the
installation. Recalibrate after changing camera positions, focus, sensor
orientation, or the imaging-plane height.

## Browser Registration Workbench

Place each source directly in the corrected multi-layer canvas. The canvas is
the same coordinate system used by deployed capture, so dragging a layer into
its physical position establishes its canvas anchor. Arrange camera fields
side-by-side with the measured overlap, then use Save All to write every source
transform together.

Map bench thermal sources to their physical pair with `pair_name` in
`config/bench.json`. Only mapped thermal sources can be used in the drag
workbench. The display is for fine adjustment; verify the transform with the
calibration target and record the result before setting `calibrated: true`.

The corrected multi-layer canvas uses the same `source -> canvas` transform
contract as deployed capture. It rectifies each layer when its intrinsics are
present, shows every configured source available in bench mode, and lets the
operator select one layer at a time. Save All writes directly placed source
transforms atomically and preserves `calibrated: false`.

The final inspection size is `inspection_roi`, not a camera frame size. Make
the canvas and ROI wide enough for all fixed camera fields plus their overlap;
the deployed mosaic projects every calibrated pair into that larger canvas.

## Coverage And Speed

Set `inspection_roi` to the vehicle area a technician must review. Set coverage
thresholds only after measuring real captures.

Set `max_motion_step_pixels` no larger than the usable longitudinal overlap
between adjacent frame projections. For example, with 200 canvas pixels of
reliable overlap, use at most 100 pixels to retain 50% overlap. Set
`max_traversal_speed_mps` to the same limit expressed in physical units:

```text
max speed = max_motion_step_pixels / pixels_per_meter / worst_observed_capture_interval
```

The application measures each frame-to-frame displacement and elapsed time. It
rejects passes with excessive speed, low registration confidence, source skew,
or incomplete configured coverage. RCWL-0516 only detects presence and cannot
measure speed. `minimum_capture_duration_seconds` and
`maximum_capture_duration_seconds` bound the radar-held capture window: a slow
vehicle can be captured until it clears the sensor, while a stuck trigger cannot
create an unbounded capture.

Set `max_pair_skew_seconds` from the worst measured time between each mapped
visible capture and its MLX90640 read. The scanner captures each configured
pair consecutively; a pass is rejected when the observed skew exceeds this
calibrated limit.

## Thermal Interpretation

`thermal_range_c` is a fixed technician-visible Celsius scale. Set it to the
review range appropriate for the vehicle and thermal sensor. The mosaic also
writes raw mapped temperatures as `.npy` and a thermal coverage mask; absence
of color in the mask is missing data, not a cool measurement.
