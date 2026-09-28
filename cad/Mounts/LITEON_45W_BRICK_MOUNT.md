# Lite-On 45 W Jetson adapter cup for the clipless mounting system

Files in `out/`: `liteon_45w_brick.step/.stl` (print one), `clipless_foot.step/.stl` (print
two, from `omni20_mount.py`), `liteon_45w_brick_assembly.step` (cup, feet, two clipless
pieces in a stub of baseplate, grey envelope of the brick with prongs and cable),
`liteon_45w_brick.png` (shaded, two views), `liteon_45w_brick_assembly.png`,
`liteon_45w_brick_lines.png` and `liteon_45w_brick_assembly_lines.png` (hidden-line).
Source: `liteon_45w_brick_mount.py`, importing `clipless.py`, `render.py` and the
`lines_png` / `box` / `inter` helpers of `jetson_orin_nano_mount.py`.

## Component dimensions: ESTIMATE, measure before printing

The adapter is the one in the Jetson Orin Nano Developer Kit box: Lite-On, marked
NVIDIA45W2520004701, 19 V 2.37 A, folding US (NEMA 1-15) prongs on one end face of the
body, barrel-plug cable out of the opposite end. NVIDIA's kit datasheet only says "19V
Power Supply (45W) with Type B (US, JP) Power Cable"
(https://www.sparkfun.com/nvidia-jetson-orin-nano-developer-kit.html quoting it) and
the kit's carrier board specification does not cover the adapter. Lite-On's public
PA-1450 datasheets are for the desktop-brick family (e.g. PA-1450-26,
https://power.liteon.com/Files/Datasheet/Datasheet_Laptop_Power_Adapter_01_45W_PA-1450-26.pdf,
95 x 38 x 25 mm with an IEC inlet), which is a different housing from the wall-prong unit
in the kit. No dimensioned source for the NVIDIA45W2520004701 housing was found.

| Item | Value | Status |
| --- | --- | --- |
| Body | 85 x 58 x 28 mm (`BRICK_L` prong face to cable face, `BRICK_W`, `BRICK_H`) | ESTIMATE, env vars |
| Prongs | NEMA 1-15 blades, 12.7 mm apart (UL 498 blade spacing), ~16 mm long, centred on the end face (`PRONG_DZ = 0` above mid-height) | blade spacing is standard; position on the face is an ESTIMATE |
| Cable | 3.5 mm, out of the centre of the cable face | ESTIMATE |
| Mass | ~180 g | ESTIMATE |

Measure the brick and rerun, e.g. `BRICK_L=88 BRICK_W=57 BRICK_H=27.5 PRONG_DZ=2 python3
liteon_45w_brick_mount.py`. Everything in the cup scales from these.

## Where it sits: assumed geometry with the Omni 20+ cradle

The brick plugs straight into the Omni 20+'s AC outlet, so it is held with its prong face
free and pointing at the pack. From `omni20_mount.py` / `OMNI20_MOUNT.md`: pack 127 x 122 x
27 mm, cradle inside 128.2 x 123.2, 3 mm corner posts, 4 mm floor, feet at +-24.5 along x;
the AC outlet, USB and DC ports sit mid-edge, with the corner posts leaving every edge open.

Assumptions built into the script (all parameters, `assumed_geometry()` prints them):

1. The cradle is placed so that the pack edge with the AC outlet faces +x, toward this
   cup. That is the 122 mm edge, 61 mm from the pack centre (`OMNI_HALF`); if the outlet
   is on the 127 mm edge the cradle has to be turned 90 degrees on the plate (its feet
   stay along x, the outline becomes 129 x 134). The alternative, cup beside the cradle
   across the car, has no plate holes: the baseplate STL has three rows across (hole
   centres at 47.5 mm pitch) and the cradle's 129 mm width covers all three.
2. The outlet centre is `OUTLET_Z = 13.5` mm above the cradle floor top (ESTIMATE; 27 mm
   pack, outlet at mid-height). Both floors are 4 mm on the same plate, so the brick's
   prong centre (floor 4 + 28/2 = 18.0 above the rim tops) is 0.5 mm above the outlet
   centre (4 + 13.5 = 17.5). If `OUTLET_Z` comes out larger than `BRICK_H/2 + PRONG_DZ`
   the script thickens the cup floor by the difference (`PAD`); if smaller, the brick
   still plugs (the outlet's contacts have that much play) but note it.
3. Brick face 0.5 mm from the pack face when plugged (`PLUG_GAP`).
4. Cup prong end = brick face + 4 mm overhang, which lands 0.9 mm clear of the cradle's
   outer wall (cradle rim at 64.6, cup end at 65.5 from the cradle centre).
5. Feet: two across the car (`foot('y')`, PITCH_Y 47) under a 40 x 83.5 mm transverse bar.
   The pair is `FEET_X_OFFSET` from the brick centre; the default is worked out so that
   it lands on the second plate column beyond the cradle's +x foot: cradle feet at +-24.5,
   columns at 73.5 and 122.5 for `PEG_PITCH = 49`, brick centre at 61 + 0.5 + 42.5 = 104,
   so the offset is 18.5 (20.5 for a 50 mm column pitch). Along the car the pegs are the
   snug 23.8 mm size, so this offset has to match the real plate: set `FEET_X_OFFSET`
   after measuring the column next to the cradle.

Assembly order that works with these: plug the brick into the pack first, then drop pack
and brick together, the pack into its cradle and the brick down over the cup's fingers.
The cup does not fix x (2 mm play at the cable-end wall, no stop at the prong end); the
outlet does.

## What the cup does

Brick centre at the origin, prong face at -x.

- 4 mm floor from 4 mm inside the prong face to 3 mm past the cable-end wall (86.5 mm),
  65 mm wide, with a 40 x 83.5 mm transverse bar under the feet and a lightening window
  ahead of it. The brick lies on the foot flanges (they reach from y 8.5 to 38.75, the
  brick covers them to y 29) and traps them.
- Two 3 mm side walls 14 mm high, a 3 mm cable-end wall of the same height with a 12 mm
  notch down to the floor, and no wall at the prong end.
- In an 18 mm gap in each side wall a 1.6 mm x 16 mm snap finger rises from the floor to
  34.3 mm with a 0.6 mm lip on a 45 degree ramp, 0.3 mm above the brick top. Same
  proportions as the Jetson tray's fingers (about 20 N per finger to snap, PETG
  preferred). The lip's flat underside stops the brick lifting.
- Pocket 59 mm wide (0.5 a side), 88 mm long including the 2 mm x play.
- 23.8 cm3, about 18 g in PLA at 60 % effective, plus two feet.

## Verified in CadQuery (`python3 liteon_45w_brick_mount.py`)

```
cup 86.5 x 83.5 x 34.3 mm, 23.8 cm3 (29 g PLA solid, ~18 g at 60% effective), brick 85.0 x 58.0 x 28.0 (ESTIMATE), feet across at 47.0, offset 18.5 from the brick centre
  solids: 1
  cup_x_feet: 0.0
  feet_x_clipless: 0.0
  cup_x_plate: 0.0
  cup_x_envelope: 0.0
  feet_x_envelope: 0.0
  floor: 4.0
  lip_underside_z: 32.3
  brick_top_z: 32.0
  feet: [(18.5, -23.5), (18.5, 23.5)]
  bar: (40.0, 83.5)
  pack_outlet_face_x: 61.0
  cradle_outer_wall_x: 64.6
  brick_face_x: 61.5
  cup_prong_end_x: 65.5
  gap_cup_to_cradle_wall: 0.9
  brick_centre_x: 104.0
  cup_feet_x: 122.5
  plate_columns_from_cradle_centre: [24.5, 73.5, 122.5]
  outlet_centre_above_plate_rim_tops: 17.5
  prong_centre_above_plate_rim_tops: 18.0
```

Intersection volumes in mm3 (cup vs feet, feet vs clipless pieces, cup vs plate stub, cup
and feet vs the brick envelope with prongs and cable, placed where the brick sits). The
`_x` entries after `bar` are distances from the cradle centre. Mass from volume: 29 g
solid PLA, about 18 g as printed.

## Printing

Floor down, no supports: the fingers are vertical, the lips are 0.6 mm 45 degree ramps,
the walls are plain. 0.2 mm layers, 3 walls, 30 % infill, PETG for the fingers if
available. Do not fillet the finger edges in the slicer.

Fit tuning: `CLR` (0.5), `LIP` (0.6), `LIP_CLR` (0.3), `X_CLR` (2.0), `OVERHANG` (4.0),
`FEET_X_OFFSET`, `OUTLET_Z`, `PRONG_DZ`.

## Change for Baseplate v2 (snap fingers, `FINGER_X`)

The two snap fingers used to stand at `FINGER_X = 8` from the brick centre, over the foot
bar. When `FEET_X_OFFSET` puts a foot recess (34.5 mm along x) under a finger, the recess
cut severs the finger from the floor: the cup with `FEET_X_OFFSET=10.25` (the STL layout in
LAYOUT.md) was three solids and nobody noticed because the layout script did not count
solids. `finger_x()` now centres the fingers in the longer floor span beside the recess
(toward the prong end for every offset tried: -18.6 default, -21.1 on v2, -22.8 on the STL
layout), 1 mm clear of it; `FINGER_X` as an env var overrides. `board_layout.py` now fails a
mount part that is not one solid. For v2 the cup is built with `PITCH_Y=82` (feet on rows A
and C of column 8, bar 40 x 118.5) and `OUT_SUFFIX=_v2` (`out/liteon_45w_brick_v2.*`);
`OUT_SUFFIX` is accepted by every mount script in the set.

## Wall over the foot slot (found on the printed part, September 2026)

The first v2 print could not take its feet. The flange recess for a foot at y 41 spans
y 26 to 56 and the 3 mm side wall runs along y 29.5 to 32.5, 14 mm tall, so the wall
bridged straight over the middle of the slot. `foot_cutout` only cuts to FLOOR + 1 and
nothing checked what stood above that. The part was one solid and every interference
number was zero, which is why it got through: the checks asked whether the foot fits in
its slot, not whether it can get there.

`cup()` now subtracts `foot_access()` for each foot, a clear column over the flange
footprint from the recess floor upward, which opens a 34.5 mm window in each side wall
centred on the foot bar. The fingers at x -21 and the cable-end wall are untouched and
still locate the brick. `checks()` reports `material_over_foot_slots`, which was 2652 mm3
on the old geometry and must be 0. The same test runs over the whole set in
`audit_feet.py`.
