# Flipsky FSESC 6.7 PRO (aluminium case) tray for the clipless mounting system

Files in `out/`: `vesc_fsesc67.step/.stl` (print one), `clipless_foot.step/.stl` (print two,
from `omni20_mount.py`), `vesc_fsesc67_assembly.step` (tray, feet, two clipless pieces in a
stub of baseplate, grey envelope of the case with its leads and button),
`vesc_fsesc67.png` (shaded, two views), `vesc_fsesc67_assembly.png`,
`vesc_fsesc67_lines.png` and `vesc_fsesc67_assembly_lines.png` (hidden-line). Source:
`vesc_fsesc67_mount.py`, importing `clipless.py`, `render.py` and the `lines_png` / `box` /
`inter` helpers of `jetson_orin_nano_mount.py`.

## Component dimensions and where they come from

Flipsky FSESC 6.7 PRO "based upon VESC6 with Aluminum Case and Anti-spark switch", product
page https://flipsky.net/products/flipsky-fsesc-6-7-pro-based-upon-vesc6-with-aluminum-case-and-anti-spark-switch-with-heat-sink
(the shorter URL that search engines return, `.../products/fsesc-6-7-pro-based-upon-vesc6-with-aluminum-case`,
now 404s). The page links two manuals; the one for the case versions (packages A/B) is
https://cdn.shopify.com/s/files/1/0011/4039/1996/files/FSESC_6.7_PRO_-_a_b_compressed.pdf

| Item | Value | Source |
| --- | --- | --- |
| Case size | 95 x 92 x 24.5 mm | manual PDF above, "Size: 95x92x24.5mm" (the 70 A / 200 A PRO, which is what the current photos show printed on the lid) |
| Older figure | 100 x 92 x 22.5 mm, 380 g "include case" | the product page's own spec table, which still describes the 60 A / 150 A version |
| Motor wire | 12 AWG, three phase leads | manual |
| Battery lead | XT90 female, at the opposite end from the phase leads | product photos (end view with the XT90; top view with the phase leads leaving the other end) |
| Signal connectors | PPM, CAN, COMM1/COMM2 (UART, I2C, SPI), SWD, SENSE, micro USB: all inside a windowed panel in the lid, with a removable 2-screw cover | product photo of the lid with the cover off; wiring diagram https://cdn.shopify.com/s/files/1/0011/4039/1996/files/v6.7_pro.jpg |
| Anti-spark button | on one long side, nearer the phase-lead end | product side photo |
| Lid screws | four, in the top corners, about 4 mm in from each edge | measured off the top-view product photo (case 464 x 508 px, screw centres 424 px apart across, ~20 px inset) |
| Mounting holes | none documented | see below |

The two size figures disagree by 5 mm in length and 2 mm in height. The script uses the
manual's 95 x 92 x 24.5 and exposes `CASE_L`, `CASE_W`, `CASE_H` as env vars; **measure the
case height before printing**, because the finger lips sit 0.3 mm above the case top and a
2 mm error either leaves the case loose or stops it seating. `CASE_H=22.5` rebuilds for
the older case.

Correction to the brief: the phase leads and the battery lead are NOT on the same end.
The XT90 (and, inside the lid window, the micro USB) is at one end, the three phase wires
at the other, the button on a side, and the servo/PPM, UART and CAN headers are reached
from the top through the lid window. The tray therefore leaves both ends open, with a
zip-tie tab at each, and needs nothing on the sides except the two fingers.

Mounting holes: Flipsky does not document any, and the photos show only the four lid
screws on the top face. The four M3 heat-set bosses in this tray are on that lid-screw
pattern (`HOLE_INSET = 4.0`, ESTIMATE), at (+-43.5, +-42.0) from the case centre, so they
line up with the lid screws if the case bottom is tapped on the same pattern (some Flipsky
cases are) and otherwise just act as corner standoffs. Do not drill the case for them; the
fingers hold it without screws.

Estimates that only place the grey envelope: XT90 boot 22 x 16 x 18 mm, phase leads
spread over 36 mm, button 12 mm diameter standing 6 mm proud at `BUTTON_X = -14`.

## What the tray does

Case centre at the origin, +x toward the XT90 end, -x toward the phase leads, button on
-y.

- 4 mm floor, 102 x 99 outline plus 12 mm zip-tie tabs at both ends (126 x 99 overall).
  The floor is a 4 mm rim with two full-width bars; the bars carry the foot pads and have
  30 x 20 mm windows on both sides of each pad. Inside the rim everything else is open.
- Two 38 x 44 x 9 mm foot pads at x = +-24.5 with the foot recess in their tops. The
  case stands on the pads (and on the four bosses), 5 mm above the floor, and the foot
  flanges are flush with the pad tops, so the case traps the feet. Air passes under the
  case, out of both open ends and through the floor.
- Four 10 mm bosses with 4.0 x 5 mm holes for M3 heat-set inserts, tops flush with the
  pads (see above for the pattern caveat).
- Four L-shaped corner stubs (3 mm walls, 8 mm legs, 8 mm above the pads) locate the case
  in x and y and leave the end faces free for the leads.
- Two snap fingers on the long sides at x = +18 (`FINGER_X`, toward the XT90 end, clear
  of the button), 14 mm wide, 2.0 mm thick, reaching 35.8 mm, with a 1.0 mm lip on a 45
  degree ramp and a flat underside 0.3 mm above the case top. The case is pressed down
  over them. The lip's flat underside means the case cannot lift without the finger being
  pried outward. Per finger (PLA, E = 3 GPa, 14 x 2 mm section, 31 mm free length) the 1 mm
  deflection to snap needs about 3 N, root stress about 10 MPa. 2 mm rather than 1.6 mm
  because the case is metal and 380 g; raise `FINGER_T` to 2.4 for a stiffer hold.
- Zip-tie tabs: 34 x 12 mm at each end with two 2.6 x 6 mm slots 18 mm apart for a 4.8 mm
  cable tie around the phase bundle and the battery lead.
- Pocket 96 x 93 (0.5 mm a side). 38.6 cm3, about 29 g in PLA at 60 % effective, plus two
  feet. Two feet along the car (380 g is under the 400 g four-foot threshold).

## Verified in CadQuery (`python3 vesc_fsesc67_mount.py`)

```
tray 126.0 x 99.0 x 35.8 mm, 38.6 cm3 (48 g PLA solid, ~29 g at 60% effective), case 95.0 x 92.0 x 24.5, peg pitch 49.0 mm
  solids: 1
  tray_x_feet: 0.0
  feet_x_clipless: 0.0
  tray_x_plate: 0.0
  tray_x_envelope: 0.0
  feet_x_envelope: 0.0
  case_underside_z: 9.0
  lip_underside_z: 33.8
  case_top_z: 33.5
  pocket: (96.0, 93.0)
  bosses: [(-43.5, -42.0), (-43.5, 42.0), (43.5, -42.0), (43.5, 42.0)]
```

Intersection volumes in mm3: tray vs feet, feet vs clipless pieces, tray vs baseplate stub,
tray and feet vs the case envelope (case, XT90 boot, phase leads, button) placed where the
case sits. Mass from volume: 48 g solid PLA, about 29 g as printed.

## Printing

Floor down, no supports. The only overhangs are the 1 mm finger lips (45 degree ramps
underneath, flat tops at 35.8 mm); the foot recesses are open at the top and the pad
windows are through-cuts. 0.2 mm layers, 4 walls, 30 % infill. PETG if the ESC runs hot enough to matter; the tray only
touches the case at the pad tops and bosses.

Fit tuning: `CLR` (0.5), `LIP` (1.0), `LIP_CLR` (0.3), `FINGER_T` (2.0), `FINGER_X` (18; move
it if the button turns out to be elsewhere on the side).
