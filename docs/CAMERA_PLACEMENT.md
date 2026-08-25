# Camera Placement And Deployment Geometry

This document records the current camera-placement direction for the Tesla
Model S, 3, X, and Y inspection target. It is a design starting point, not a
substitute for physical clearance tests and calibration at the installed scan
plane.

## Deployment Profiles

Two profiles share the same calibrated low undercarriage strip.

| Profile | Intended coverage |
| --- | --- |
| Dynamic portable | Low inboard visible/thermal strip: central underbody, battery edges, inboard suspension, and inner brake heat. It may capture partial wheel/tread evidence but does not promise full side, wheel-face, or roof inspection. |
| Static service-bay | The same low strip plus optional protected side pods and arches for wheel faces, outer tire shoulders, outer brake views, rockers, body, and roofline. |

Do not claim full tire, wheel, or side coverage from the dynamic low strip
alone. The application must retain coverage confidence and missing-region data.

## Ramp Envelope

Each referenced VEVOR two-channel cable protector is approximately:

```text
39.57 in long x 9.45 in travel depth x 1.77 in high
1.3 in wide x 1.2 in high cable channels
```

Assuming units are joined end-to-end across the vehicle:

| Configuration | Cross-car width | Travel depth | Use |
| --- | ---: | ---: | --- |
| Two across | 2,010 mm | 240 mm | Minimal low-profile drive corridor. |
| Two by two | 2,010 mm | 480 mm | Preferred four-ramp portable layout; supports a longer sensing area or staggered rows without making the lane excessively wide. |
| Three across | 3,015 mm | 240 mm | Static/expanded layout with approximately 500 mm wings beside a 2,010 mm central drive corridor. Reserve wings for protected side modules, not driving. |
| Four across | 4,020 mm | 240 mm | Not recommended. It encourages excessive lateral vehicle offset without adding useful low-strip coverage. |

Cable channels are for protected cable routing, not complete camera housings.
Their height and width can occlude wide lenses. Use rigid, repeatable mounts
with flush optical windows. Do not stack rubber ramps as vertical camera
supports.

Before cutting mounts, test the unmodified ramp with the lowest-clearance
target vehicle. Verify no underbody, aero-panel, battery, tire sidewall, or
transition-pitch contact.

## Low Inboard Sensor Strip

Use scanner centerline as lateral coordinate zero. Keep paired visible and
thermal optical centers close in the travel direction, ideally within +/-25 mm,
to limit visible/thermal timing skew while a vehicle moves.

The following is the initial placement target for three visible and three
thermal sensors:

| Pair | Lateral location | Initial lateral tilt from vertical | Primary target |
| --- | ---: | ---: | --- |
| Center visible | 0 mm | 0 degrees | Battery enclosure and central underbody. |
| Center thermal | 0 to +/-40 mm | 0 degrees | Matching central thermal footprint. |
| Left visible | -375 mm | 18-22 degrees outward | Left outer belly, inner wheel/brake, and suspension. |
| Left thermal | -420 mm | 20-25 degrees outward | Matching left hotspot footprint. |
| Right visible | +375 mm | 18-22 degrees outward | Right outer belly, inner wheel/brake, and suspension. |
| Right thermal | +420 mm | 20-25 degrees outward | Matching right hotspot footprint. |

The outer cameras deliberately use the wide lens at an oblique angle. They
should overlap the center field and target the useful middle of their footprint;
do not treat the heavily oblique far lens edge as high-quality inspection area.

Published Tesla track widths are roughly 1,584 mm for Model 3, 1,636 mm for
Model Y, and approximately 1,660-1,700 mm for Model S/X. Wheel-center positions
therefore span roughly +/-792 mm to +/-850 mm. Actual vehicle year, tire size,
offset, ride height, and loading must be verified before final fabrication.

The MLX90640 does not need the same mounting distance as its paired visible
camera. Its calibrated thermal footprint must cover the same physical target
polygon. Calibrate each outer pair independently with `thermal_to_strip`.

## Outboard Views

More lateral wing width helps package a side-view module, but a low
upward-facing camera placed farther outboard has a worse line of sight to deep
wheel hardware. It primarily sees outer sidewall and tread shoulder.

For static deployments, optional rigid side pods can add supplemental visible
and thermal views:

```text
Optical-center height: 180-250 mm above floor
Lateral location:      +/-1,150 to +/-1,250 mm from scanner centerline
Aim:                   inward, 10-25 degrees upward from horizontal
Fore/aft adjustment:   5-15 degrees
```

These views can inspect outer sidewall, outer tread shoulder, wheel face,
wheel-well liner, and intermittent rotor/caliper visibility through rotating
wheel spokes. They are useful supplemental thermal hotspot detectors after a
drive, but an MLX90640 at this range should report an abnormal hot region, not
promise component-level diagnosis.

Outboard cameras cannot reliably see the back/inboard side of the hub, bearing,
rotor, or caliper. Preserve low inboard outer pairs for those targets. Do not
move the three planned low thermal sensors outboard; add outboard visible and
thermal pairs only as future static-deployment expansion.

## Calibration Requirements

Every final position and tilt requires a new calibration:

- Visible intrinsics at the deployed resolution and focus.
- Visible-to-strip transform for each low camera.
- Thermal footprint measurement and thermal-to-strip transform for each sensor.
- Motion calibration and coverage validation at the installed scan-plane height.
- Physical coverage tests across representative Model S, 3, X, and Y vehicles.

See [CALIBRATION.md](CALIBRATION.md) for the required transform and speed-limit
workflow.
