# Jetson Orin Nano Developer Kit tray for the clipless mounting system

Files in `out/`: `jetson_orin_nano.step/.stl` (print one), `clipless_foot.step/.stl` (print
four, from `omni20_mount.py`), `jetson_orin_nano_assembly.step` (tray, feet, four clipless
pieces in a stub of baseplate, grey envelope of the kit), `jetson_orin_nano.png` (shaded,
two views), `jetson_orin_nano_assembly.png`, `jetson_orin_nano_lines.png` and
`jetson_orin_nano_assembly_lines.png` (hidden-line). Source: `jetson_orin_nano_mount.py`,
importing `clipless.py` and `render.py`.

## Component dimensions and where they come from

Jetson Orin Nano Developer Kit reference carrier board, NVIDIA P3768.

| Item | Value | Source |
| --- | --- | --- |
| Carrier PCB outline | 100.00 +-0.13 x 79.00 +-0.13 mm | Carrier Board Specification SP-11324-001_v1.0, Figure 4-1 (NVIDIA Jetson Download Center; copy read at https://gzhls.at/blob/ldb/d/a/7/6/f487430a9edf313e0aea8393ee3caf5a805c.pdf, page 29) |
| PCB thickness | 1.57 +-0.16 mm | same, Figure 4-1 |
| Tallest part above the board (connectors) | 16.70 mm max | same, Figure 4-1 |
| Tallest part below the board (M.2 cards) | 4.30 mm max | same, Figure 4-1 |
| Whole kit with plastic base, module, heatsink, fan and feet | 103.00 +-0.20 x 90.50 +-0.20 x 34.77 +-1.09 mm | same, Figure 4-2; also the kit datasheet "Mechanical 103mm x 90.5mm x 34.77mm" https://docs.rs-online.com/4051/A700000009607470.pdf |
| Mounting holes | four, 2.7 mm, on 92.0 x 58.0 mm; 4.0 mm in from both short edges, 4.0 mm in from the 40-pin-header edge, 17.0 mm in from the connector edge | measured off the Figure 4-1 drawing, see below |
| Recommended hardware | M2.5 standoffs, hex 4.5 mm A/F, and M2.5 x 3.7 mm pan-head screws | Jetson Orin NX / Orin Nano Product Design Guide, quoted at https://forums.developer.nvidia.com/t/3d-print-template-for-jetson-orin-nano-enclosure/351753 |

NVIDIA does not dimension the hole positions anywhere public; the spec only draws them, and
the reference design files that contain them sit behind a developer login
(https://forums.developer.nvidia.com/t/jetson-orin-nano-carrier-mechanics/339279). The
positions above were taken from the Figure 4-1 raster (951 x 772 px, 9.36 px/mm from the
100.00 mm dimension) by ring-detecting the four plain circles: pixel centres (45, 183),
(906, 183), (45, 726), (906, 726) with radius 12.5 px, board edges at x = 7.5 / 943.5 and
y = 763.5. That is 92.0 x 58.0 mm with about +-0.15 mm uncertainty, which the 0.5 mm
pocket clearance and the 3.5 mm insert holes absorb. Pattern parameters are `HOLE_DX`,
`HOLE_DY`, `HOLE_EDGE` in the script if someone gets the real design files.

Estimates, marked as such in the script: `TOP_H = 27 mm` for the module + heatsink + fan
above the board (kit height minus base and feet); the connector strip starting 2.4 mm in
from the DC-jack end and overhanging the connector edge by 2 mm; the header footprint
(x 22..57 mm from the DC-jack end); a 2280 M.2 card under the board. None of these touch
the tray, they only place the grey envelope.

## What this tray holds

The bare carrier with module and heatsink, without the kit's plastic base. The base
covers the M.2 sockets and has no mounting features; it comes off with its four M2.5
screws once, and after that nothing on the Jetson is ever unscrewed again. If the car
should keep the plastic base on, this tray is wrong: the base is 103 x 90.5 and needs a
different pocket.

Layout, board centre at the origin, +x along the car:

- Connector edge (DC jack, USB-C, two USB-A stacks, Ethernet, DisplayPort) at -y. There is
  no wall on that side at all.
- 40-pin header edge at +y. Two 3 mm corner stubs (12 mm long) outside the header's
  22..57 mm span locate the board in +y.
- Board sits on four 9.5 mm bosses on its own hole pattern, 6 mm above the 4 mm floor, so
  the M.2 sockets and the 4.3 mm of parts underneath are in free air; the floor has a
  cross-shaped window between the four foot pads.
- Each boss has a 3.5 mm x 5 mm hole for an optional M2.5 heat-set insert. Screws are
  optional; the tray does not rely on them.
- -x end: two 3 mm legs along the short edge with fixed 1.2 mm lips 0.3 mm above the
  board. The board hooks under these first.
- +x end: two 1.6 mm snap fingers (12 mm and 6.5 mm wide, the short one keeps clear of the
  right-angle button header J14) with a 0.4 mm lip on 45 degree chamfers. The board is
  pressed down over them. 1.6 mm is below the 3 mm wall rule on purpose: a 3 mm PLA
  cantilever of this height would need about 65 N to snap and would crack. With 1.6 mm it
  is roughly 20 N per finger and 40 MPa at the root (E = 3 GPa, PLA); PETG is softer and
  the better material for this part.
- Camera connectors sit mid-way along the short edges, the corner legs stop 4 mm short
  of them.
- Pocket 101.0 x 80.0 (0.5 mm a side). Tray 107 x 86 x 13.4 mm, 21.6 cm3, about 16 g in
  PLA, plus four feet.
- Four feet on `PEG_PITCH` x `PITCH_Y` = 49.0 x 47.0 mm (env vars), at (+-24.5, +-23.5).
  Each foot pad is 47 x 39.5 mm of solid floor.

Insert the four feet before the board; the board's underside then traps them.

## Verified in CadQuery (`python3 jetson_orin_nano_mount.py`)

```
tray 107.0 x 86.0 x 13.4 mm, 21.6 cm3 (~16 g PLA at 60% effective), feet on 49.0 x 47.0 mm
holes (x, y) relative to board centre: [(-46.0, 35.5), (-46.0, -22.5), (46.0, 35.5), (46.0, -22.5)]
  solids: 1
  tray_x_feet: 0.0
  feet_x_clipless: 0.0
  tray_x_plate: 0.0
  tray_x_envelope: 0.0
  feet_x_envelope: 0.0
  boss_top_z: 10.0
  lip_underside_z: 11.87
  pocket: (101.0, 80.0)
```

Intersection volumes are in mm3 between the tray and the feet, the feet and the clipless
pieces, the tray and the baseplate stub, and the tray and the kit envelope (board, connector
strip, header, module block, M.2 card) placed where the kit sits. The first run of the
envelope check caught the -x,-y fixed lip touching a connector strip that ran the full
board length; the DC jack actually starts 2.4 mm in from that edge, so the strip was
corrected and the lip is clear by 1.7 mm.

## Printing

Floor down, no supports. The only overhangs are the 1.2 mm fixed lips and the 0.4 mm snap
lips at 9.4 mm height, which print as tiny bridges; the snap-lip chamfers are 45 degrees.
0.2 mm layers, 3 walls, 30 % infill. PETG preferred for the snap fingers. Do not fillet the
finger edges in the slicer.

Fit tuning: `CLR` (pocket clearance, 0.5), `LIP_SNAP` (0.4; raise to 0.5 if the board
rattles, lower to 0.3 if it will not go in), `FINGER_T` (1.6).
