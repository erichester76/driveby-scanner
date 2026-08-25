# Scanner System Design

This document records the product and hardware direction for the drive-over EV
inspection system. It is not an operator guide; see [README.md](../README.md) for
installation, configuration, calibration, and runtime modes.

## Product Scope

The scanner collects calibrated visible and thermal imagery while an EV drives
over a low-profile sensor strip. Successive fixed cross-car strips are
registered along vehicle travel into one inspection mosaic. The intended target
vehicles are Tesla Model S, 3, X, and Y, with final dimensions verified against
physical representative vehicles before fabrication.

## Sensor Platform

The planned platform uses three VEVOR two-channel cable protectors, each
approximately 39.57 in by 9.45 in by 1.77 in, joined end-to-end across the
vehicle:

```text
Total platform width: 3,015 mm
Central drive corridor: 2,010 mm
Outer wings: about 500 mm per side
Travel depth: 240 mm
```

The outer wings are reserved for protected future side modules and are not the
normal drive surface. Four units should be arranged two-by-two for a longer
2,010 mm drive corridor, not four across into an unconstrained 4,020 mm lane.

The protected inboard sensor area uses a shared removable rigid tray. Individual
visible and thermal pods mount inside the tray on repeatable fixed-angle wedges.
The VEVOR troughs carry wiring and provide drainage; the pods are serviceable,
not structural, and never epoxy-potted. Keep the tire-contact regions of the
rubber ramp intact. See [CAMERA_PLACEMENT.md](CAMERA_PLACEMENT.md) for geometry
and [CALIBRATION.md](CALIBRATION.md) for the resulting calibration work.

## Low Sensor Strip

The planned low strip has three visible and three thermal sources:

| Pair | Initial lateral placement | Initial pose | Inspection target |
| --- | ---: | --- | --- |
| Center | 0 mm | Vertical | Battery enclosure and central underbody. |
| Left | -375 to -420 mm | 18-25 degrees outward | Left outer belly, inner wheel/brake, and suspension. |
| Right | +375 to +420 mm | 18-25 degrees outward | Right outer belly, inner wheel/brake, and suspension. |

The center visible/thermal pair stays near vertical. The outer pairs use the
wide field of view deliberately, but only the measured, useful portion of the
oblique image is treated as inspection coverage. Thermal cameras must cover the
same physical target polygons as their visible partners; they do not need the
same mounting distance.

## Deployment Profiles

### Dynamic Portable

Dynamic deployments include events, Superchargers, and unattended facility
entrances. They remain low profile and use the inboard strip for central
underbody, battery-edge, inboard suspension, and inner brake-heat inspection.
They report the coverage actually achieved and must not claim full tire,
wheel-face, side, or roof inspection.

### Static Service Bay

Static deployments reuse the low strip and can add protected elevated side pods
and arches. Those modules add wheel-face, outer tire-shoulder, outer brake,
rocker, body, and roofline views. Elevated outboard thermal cameras are useful
supplemental hotspot detectors, but do not replace the inboard thermal views of
brake and hub hardware.

## Driver Guidance

An unattended portable unit needs to read as an inspection station rather than
a speed bump. The planned guidance system is a thin retroreflective approach
mat 8-10 ft before the ramp and a separate addressable RGBW leading-edge strip.
The white undercarriage illumination is reserved for capture and is not a
driver-facing cue.

The driver-facing state sequence is:

```text
idle -> amber approach -> scanning -> red completion -> green ready sweep
```

See [APPROACH_LIGHTING.md](APPROACH_LIGHTING.md) for the approach-mat wording,
lighting behaviors, and capture-time lighting constraints.

## Software Model

The deployed process requires a calibrated `config/inspection_layout.json`. It
waits for radar presence, captures calibrated visible/thermal source pairs while
the vehicle crosses, rejects poor movement or coverage, and saves a timestamped
inspection mosaic plus metadata. The radar detects presence only; motion is
estimated from the calibrated visible imagery.

The browser bench console provides source previews and calibration tools. It
does not enable deployed capture merely by saving calibration values; the layout
must be explicitly reviewed and marked calibrated.
