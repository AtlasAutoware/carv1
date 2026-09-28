# Small-board plate (PCA9685 + USB hub) for the clipless mounting system

Files in `out/`: `small_board_plate.step/.stl` (print one), `clipless_foot.step/.stl`
(print two, from `omni20_mount.py`), `small_board_plate_assembly.step` (plate, feet, two
clipless pieces in a stub of baseplate, grey envelopes of the PCA9685 and the hub),
`small_board_plate.png` (shaded, two views), `small_board_plate_assembly.png`,
`small_board_plate_lines.png` and `small_board_plate_assembly_lines.png` (hidden-line).
Source: `small_board_plate_mount.py`, importing `clipless.py`, `render.py` and the
helpers in `jetson_orin_nano_mount.py`.

## Component dimensions and where they come from

Adafruit 16-Channel 12-bit PWM/Servo Driver, product 815 (PCA9685), rev C board.

| Item | Value | Source |
| --- | --- | --- |
| Board | 62.23 x 25.40 mm, corner radius 3.175 (outline x -1.905..60.325, y -6.477..18.923 in the Eagle file); Adafruit rounds it to 62.5 x 25.4 x 3 mm | `Adafruit PCA9685 rev C.brd`, https://github.com/adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB ; https://www.adafruit.com/product/815 |
| Mounting holes | four, 2.5 mm plated, at (1.27, -3.302), (1.27, 15.748), (57.15, -3.302), (57.15, 15.748) = 55.88 x 19.05 mm pattern, 3.175 mm in from every edge | same .brd (MOUNTINGHOLE_2.5_PLATED); "Holes are 2.5mm diameter" on https://learn.adafruit.com/16-channel-pwm-servo-driver/downloads |
| Weight | 9 g with headers and terminal block | https://www.adafruit.com/product/815 |

Estimates (named parameters in the script): `PCA_TOP_H = 12 mm` for the 3x4 servo headers
and the terminal block, `PCA_UNDER = 2.5 mm` solder tails; `HUB = 85 x 26 x 12 mm` for
the 4-port USB hub, which is not identified. The hub only needs to fit the -y half of the
plate (y -30 .. -1.5) and be less than about 40 mm wide for the strap; anything else is
just a different `HUB`.

## The plate

100 x 60 x 4 mm, 4 mm corner radius, two clipless feet along the car on `PEG_PITCH`
(49.0 mm, env var). Coordinates below are relative to the centre of the foot pair; the
plate itself is centred 8.5 mm to +x of that (`PLATE_OFFSET`), so it spans
x -41.5 .. 58.5.

Why the offset: each foot needs a 30.5 x 34.5 mm flange recess, so the middle band of
the plate (|y| < 17.25) is only solid for |x| < 9.25 between the feet and for x > 39.75
past the +x foot. The PCA9685's boss columns are 55.88 mm apart; one goes in the strip
between the feet (x = -3.3) and one past the +x foot (x = 52.58), which is only possible
with the plate shifted. A centred plate cannot carry this board on two feet.

- 2.6 mm holes on a 5 mm grid centred on the plate (78 of them), everywhere that is at
  least 1 mm clear of the foot recesses, the bosses and the strap slots and 4 mm from the
  plate edge. M2.5 standoffs for anything else go there.
- PCA9685 on four 9.5 mm bosses, 6 mm tall, 3.5 x 5 mm heat-set holes, at
  (-3.30, 5.575), (-3.30, 24.625), (52.58, 5.575), (52.58, 24.625): the 55.88 x 19.05
  pattern with the board centre at (24.64, 15.1). The board spans x -6.5..55.75,
  y 2.4..27.8 and sits over the +x foot; the foot must go in first.
- USB hub on the -y half (envelope centred at (8.5, -14.5), on the plate). One 25 mm
  hook-and-loop strap through two 26 x 4.5 mm slots at (-25, -24.75) and (-25, +21.5).
  The -y slot is just outside the hub; the +y slot is in the band above the feet, so
  the strap lies on the plate for 20 mm past the hub before it goes through. There is no
  26 mm run of solid floor next to the hub's +y edge (the foot recesses are in the way),
  and a strap on the top band pulls the hub down just the same. The strap passes under
  the plate in the 1.13 mm rim gap, like the Omni cradle.
- 16.7 cm3, about 12 g in PLA, plus two feet. Total on the plate is well under 100 g,
  so two feet is already more than the rule asks for.

## Verified in CadQuery (`python3 small_board_plate_mount.py`)

```
plate 100.0 x 60.0 x 10.0 mm, 16.7 cm3 (~12 g PLA at 60% effective), feet at [(-24.5, 0.0), (24.5, 0.0)]
  solids: 1
  plate_x_feet: 0.0
  feet_x_clipless: 0.0
  plate_x_plate_stub: 0.0
  plate_x_envelope: 0.0
  feet_x_envelope: 0.0
  grid_holes: 78
  pca_bosses: [(-3.3, 5.575), (-3.3, 24.625), (52.58, 5.575), (52.58, 24.625)]
  slots: [(-25.0, -24.75), (-25.0, 21.5)]
```

Intersection volumes are in mm3 between the plate and the feet, the feet and the clipless
pieces, the plate and the baseplate stub, and the plate and the envelopes (PCA9685 board
with headers and solder tails, hub block) placed where they sit.

## Printing

Flat, bosses up, no supports; everything is vertical holes and vertical bosses. 0.2 mm
layers, 3 walls, 30 % infill. Heat-set the four M2.5 inserts before mounting anything.

## Parameter added for Baseplate v2

`PLATE_OFFSET` (default 8.5, range 8.5 to 10.75 at a 40 mm pitch so the foot recesses keep
a 4 mm rim): plate centre relative to the foot pair. v2 uses 10.75 with the plate turned
180 on holes D1+D2 (`out/small_board_plate_v2.*`, recess variant, two feet); the PCA9685
bosses and strap slots are unchanged relative to the feet, the 5 mm grid follows the plate.

## PCA9685 boss over the foot recess at 40 mm pitch (September 2026)

At 49 mm pitch the strip of solid floor between the two foot recesses is 14.5 mm wide and
the -x PCA9685 boss column (9.5 mm OD at x -3.30) fits in it with 1.2 mm to spare. On
Baseplate v2 the pitch is 40 and the strip is 5.5 mm; the same boss then overhangs the -x
recess by 3.3 mm and the foot cannot drop in. `audit_feet.py` measured 98 mm3 of boss
standing over that slot on the sliced v2 file.

Now when the strip is narrower than the boss (`_NARROW`), the boss column moves to x 0,
exactly between the recesses (`PCA_X` 27.94 instead of 24.64), which puts the far column
at 55.88 and needs `PLATE_OFFSET` 10.75 rather than 8.5 to keep it on the plate. Both are
env vars. `checks()` reports `material_over_foot_slots`. The layout was re-run with the
2.25 mm shift: still the same three documented contacts, nothing new.
