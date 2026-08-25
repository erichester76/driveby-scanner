# Inspection Strip Calibration

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

## Strip And Canvas Geometry

The system uses two coordinate systems:

- `strip` is one fixed cross-car slice. The strip editor aligns left, center,
  and right visible/thermal sources side-by-side with deliberate overlap.
- `canvas` is the final full-underbody image. During a vehicle pass, the
  application places each captured fixed strip farther along the calibrated
  travel axis.

Use a calibration target at the undercarriage plane. Populate:

- `visible_camera_matrix` and `visible_distortion_coefficients`: visible-lens
  intrinsics at the configured visible-source resolution. The application
  undistorts before strip projection and motion registration.
- `visible_to_strip`: 3x3 homography from each undistorted visible image to
  the fixed cross-car strip.
- `thermal_camera_matrix` and `thermal_distortion_coefficients`: thermal-lens
  intrinsics at 32x24.
- `thermal_to_strip`: 3x3 homography from each undistorted thermal frame to
  the same fixed strip. Resizing thermal data to a visible image is not
  calibration.
- `motion_to_canvas`: 2x2 transform from the motion camera's phase-correlation
  displacement to final-canvas displacement. It sets travel direction and
  physical scale.
- `pixels_per_meter`: measured final-canvas scale used for speed reporting.

Recalibrate after changing camera positions, focus, sensor orientation, or the
imaging-plane height.

## Browser Strip Editor

Place each source directly in the fixed sensor strip. Arrange source fields
beside one another with the measured overlap. Do not represent vehicle travel
in this editor: direct placement saves only fixed `source -> strip` geometry.

The editor rectifies each layer when intrinsics are present, shows every source
available in bench mode, and lets the operator select one layer at a time.
`Save strip layout` writes all directly placed source-to-strip transforms
atomically and preserves `calibrated: false`.

Map bench thermal sources to their physical pair with `pair_name` in
`config/bench.json`. A source without a deployed pair can be previewed but
cannot enter a deployed strip.

Set `strip.width` and `strip.height` for the union of fixed source fields. Set
`canvas` and `inspection_roi` for the complete underbody. Deployed capture
repeats the calibrated strip along the vehicle travel path to fill that canvas.

## Coverage And Speed

Set `max_motion_step_pixels` no larger than usable longitudinal strip overlap.
For example, with 200 canvas pixels of reliable overlap, use at most 100 pixels
to retain 50% overlap. Set `max_traversal_speed_mps` to the same limit in
physical units:

```text
max speed = max_motion_step_pixels / pixels_per_meter / worst_observed_capture_interval
```

The application measures frame-to-frame displacement and elapsed time. It
rejects passes with excessive speed, low registration confidence, source skew,
or incomplete configured coverage. RCWL-0516 only detects presence and cannot
measure speed. `minimum_capture_duration_seconds` and
`maximum_capture_duration_seconds` bound the radar-held capture window.

Set `max_pair_skew_seconds` from the worst measured interval between a mapped
visible capture and its MLX90640 read.

## Motion Calibration Mode

Use the browser Motion Calibration panel with a sharp calibration target on a
flat surface. Select the motion pair, enter the target's known travel distance,
choose the intended final-image direction, and record a controlled pass. The
tool uses the same motion ROI and phase-correlation method as deployed capture
to estimate `motion_to_canvas`, report confidence and capture interval, and
recommend a conservative speed limit. Saving never enables deployed capture.

## Thermal Interpretation

`thermal_range_c` is a fixed technician-visible Celsius scale. Set it to the
appropriate review range. The mosaic writes raw mapped temperatures as `.npy`
and a thermal coverage mask; missing mask coverage is absent data, not a cool
measurement.
