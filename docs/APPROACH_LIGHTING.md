# Dynamic Deployment Approach And Lighting

This document records the intended driver-facing behavior for an unattended,
portable scanner deployment. It applies to event, Supercharger, facility-entry,
and similar installations where no attendant is available to guide vehicles.

## Purpose

The scanner must read as an intentional inspection device rather than a speed
bump. The driver needs a clear, compact instruction before reaching the ramp:

```text
This is a vehicle scanner.
Slow down and center the wheels.
```

The scanner's undercarriage illumination is not a driver cue: it becomes
visible only when the vehicle is close and it is reserved for image capture.

## Approach Mat

Place a thin, traffic-rated, non-slip, beveled approach mat 8-10 feet before
the scanner. It should be wide enough to visually belong to the scan platform,
not look like a sign that can be driven around.

Use a dark charcoal base, retroreflective white lettering/lines, and a cyan
scan icon. The mat must remain understandable without power and in headlights.

```text
VEHICLE SCAN
SLOW
CENTER WHEELS
```

Print two tapered wheel-path lines below the message. They converge toward the
scanner's target tire-center bands. The approach mat is the primary alignment
cue; lights supplement it.

Avoid yellow/black hazard stripes, which make the assembly read as a speed
bump rather than an inspection station.

## Leading-Edge Light

Install a dedicated, outdoor-rated, diffused addressable RGBW strip along the
scanner's leading edge. Use RGBW rather than RGB so white is readable. Keep
this circuit separate from the bright white undercarriage illumination strip.

The strip announces scanner state. It is not a substitute for the approach mat
and must not be used to claim a vehicle is aligned unless alignment has been
measured.

| State | Leading-edge behavior | Driver meaning |
| --- | --- | --- |
| Idle | Dim cyan/white breathing effect | Scanner is available; no vehicle is nearby. |
| Approach detected | Slow amber pulse; optionally show two tire-target segments | Slow down and follow the approach mat. |
| Scanning | Solid cyan/white or dim/off | A vehicle is being captured. Do not use green. |
| Completing prior pass | Flashing red | Do not approach; capture data is still being finalized. Remain red if another vehicle is detected. |
| Ready | One green sweep across the leading edge | Prior inspection completed; the lane is available. |

After the green sweep, transition directly to the amber approach state if a
new vehicle is already detected; otherwise return to idle.

The state meaning is deliberately simple:

```text
Red: do not enter.
Green sweep: prior scan is complete and the lane is available.
Amber: approach slowly and center.
```

Turn the alignment strip off or to a measured dim setting during visible-image
capture so it does not add glare or a color cast. A controller failure should
fail dark or steady amber, never display a misleading ready indication.

## Dynamic Capture Expectations

Without an attendant or long approach-lane signage, perfect centering cannot be
guaranteed. Dynamic capture must estimate lateral vehicle position, score
per-region coverage and image quality, select the best valid source view, and
explicitly mark low-confidence or missing regions. It must not silently report
an incomplete pass as complete.
