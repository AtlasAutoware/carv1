# Flipsky FSESC 6.7 PRO tub saddle (`vesc_tub_bracket_mount.py`)

Files in `out/`: `vesc_tub_bracket.step/.stl` (print one; it contains the whole VESC
tray, no feet), `vesc_tub_bracket_assembly.step` (saddle on a stub of tub rail, grey case
envelope with leads and button), `vesc_tub_bracket.png`, `vesc_tub_bracket_assembly.png`,
`vesc_tub_bracket_lines.png`, `vesc_tub_bracket_assembly_lines.png`. Source:
`vesc_tub_bracket_mount.py`, which loads `vesc_fsesc67_mount.py` as a private module copy
with `RETAIN=recess` and builds on its `tray()` and `envelope()`; `render.py` and the
`lines_png`/`box`/`inter` helpers of `jetson_orin_nano_mount.py`.

## Component

Flipsky FSESC 6.7 PRO in its aluminium case, 95 x 92 x 24.5 mm, 380 g, XT90 at one end,
three 12 AWG phase leads at the other, anti-spark button on a long side, all signal
connectors through the lid window, no documented mounting holes. All from
VESC_FSESC67_MOUNT.md (manual PDF and product page linked there), nothing new here.

## Why a tub saddle

On Baseplate v2 the VESC tray cannot hang from the clipless grid (LAYOUT.md section 17):
the only bay deep enough is the right-hand one under the plate and the Jetson tray owns
the holes over it. The FSESC replaces the stock XL-5, whose bay is the right side of the
tub between the steering servo and the motor, so the tray goes there, on the tub. The
tub's rail geometry is not available (no Traxxas drawing; the forum threads listed in
LAYOUT.md section 18 give none), so the bracket is a clamp-on saddle with the unknowns as
parameters.

## What it is

Tray frame as in `vesc_fsesc67_mount.py`: case centre at the origin, +x the XT90 end,
-x the phase leads, button on -y, floor underside at z = 0.

- The whole VESC tray (4 mm floor with windows, two 9 mm pads the case stands on, four M3
  bosses on the estimated lid-screw pattern, corner stubs, zip-tie tabs at both ends, two
  snap fingers over the case top). The pads' foot recesses are filled (no feet). The
  fingers are at `FINGER_X` 14 instead of 18 (the copy's env default; still 8 mm clear of
  the button estimate at -14) so that on the car they pass 4.5 mm clear of the Jetson
  tray's corner posts.
- Two 3 mm legs the full tray length (97.5 mm, along tray y = along the car), straddling
  the rail: rail outer face at tray x = `RAIL_X_OUT` (4.5), rail width `RAIL_W` (12,
  ESTIMATE), 0.25 mm a side, legs reaching `RAIL_H` (18, ESTIMATE) below the rail top.
  Legs at tray x 1.25..4.25 and 16.75..19.75.
- `SADDLE_DROP` (6.0) is how far the rail top stands above the tray floor underside. The
  rail comes up through a 12.5 mm slot in the floor and the inboard pad and is bridged by a
  3 mm ceiling (z 6..9) that the case stands on together with the pad remnants and the
  bosses; the case bottom is 3 mm above the rail top. +6 is the maximum (3 mm ceiling
  under the case at 9) and is what the car needs: the case top has to pass under the
  small-board plate's clipless flanges, 16 mm below the plate (LAYOUT.md "Final set on
  v2"). 0 puts the floor on the rail; negative values raise it on the legs alone.
- Straps: two 25 mm hook-and-loop straps, 26 x 4.5 mm slots through the floor either side
  of the legs (tray x -4.75..-0.25 and 21.25..25.75) at two stations 70 mm apart (tray y
  +-35, `STRAP_Y`), on solid floor patches. Use double-sided (back-to-back) strap: down the
  outboard slot, under the rail's lip, up the inboard slot, and each end folded back onto
  itself under the floor. Nothing can close over the top because the case sits on the
  ceiling. This only works if the tub rail is a lip with a gap under it; if the tub side
  is a solid wall down to the floor the straps have nothing to wrap and the screws are the
  retention.
- Screws: four 3.2 mm holes (`ESC_HOLES`, tray coordinates, default (36, +-30), (46, +-30),
  ESTIMATE) on 9 mm floor patches inboard of the inner leg, for M3 screws into the tub's
  stock ESC bosses; `ESC_STANDOFF` adds printed 8 mm standoff tubes of that height under the
  holes once the boss height under the floor is measured (0 = holes only). Both the pattern
  and the standoff are to be measured on the car; nothing is guessed silently, the defaults
  are placeholders on a parameter.
- Ventilation: the tray's floor windows stay open except under the strap patches and the
  hole patches; inboard of the rail the floor is 6 mm below the tub rim with 22 mm of air
  to the tub floor, outboard it is in free air.
- Footprint 126 x 99 (tabs included) x 47.8 tall (18 below the rail top to 29.8 above).
  58 to 60 cm3 depending on the pad pitch inherited from the tray (`PEG_PITCH`: 49 when
  the script is run alone, 40 when `PLATE=v2 board_layout.py` regenerates it, which is
  what `out/vesc_tub_bracket.*` currently hold; the pads are only where the case stands
  and both were checked), about 45 g printed.

## Where it sits on the car (LAYOUT.md "Final set on v2")

Turned 90 degrees: XT90 end inboard, phase leads outboard toward the motor's terminal
end, legs on the tub's right wall (outer face y -50 in the chassis model), tray at x
25.25..124.25, y -3.5..-105.5, case top 27.5 above the rim. Margins in the model: 0.25 mm
to the Jetson tray (exact geometry), 2.35 mm under the small-board plate's D1/D2 flanges
(exact), 3.75 mm to the steering linkage model, 0.5 mm to the front tyre's inner face at
the tray's outboard edge, and 2.25 mm INTO the steering servo model's rear top corner
(247.5 mm3, the only non-zero row in the layout). That corner is an estimate stacked on
an estimate (servo at x 122, top 2 mm above the rim); the interference disappears if the
servo is 2.25 mm further forward or its top is 4 mm below the rim, and the Jetson tray
cannot give the 2.25 mm (its end is at 28.5, the case is at 28.75).

## Verified in CadQuery (`python3 vesc_tub_bracket_mount.py`)

```
saddle 126.0 x 99.0 x 47.8 mm, 60.1 cm3 (75 g PLA solid, ~45 g at 60% effective), rail 12.0 wide, grip 18.0, SADDLE_DROP 6.0
  solids: 1
  saddle_x_envelope: 0.0
  saddle_x_rail_nominal: 0.0
  saddle_x_rail_pushed_out: 0.0
  saddle_x_rail_pushed_in: 0.0
  legs_x: ((1.25, 4.25), (16.75, 19.75))
  leg_bottom_below_rail_top: 18.0
  floor_underside_above_rail_top: -6.0
  case_bottom_above_rail_top: 3.0
  top_above_rail_top: 29.8
  bottom_below_rail_top: 18.0
  strap_slots: {'out': (-4.75, -0.25), 'in': (21.25, 25.75), 'y': [-35.0, 35.0], 'size': (26.0, 4.5)}
  esc_holes: [[36, -30], [46, -30], [36, 30], [46, 30]]
```

Intersections in mm3: saddle against the case envelope (case, XT90 boot, phase leads,
button), against the rail stub (a RAIL_W wall 28 deep) nominal and pushed 0.24 mm against
either leg. On the car: `PLATE=v2 python3 board_layout.py`, section "Final set on v2".

## Printing

Not support-free: the legs hang 18 mm below the floor. Print legs-down: the leg bottoms
are the first layer, the ceiling (z 6..9 in tray coordinates) spans the 12.5 mm between
the legs and needs no support, and the floor outside the legs (6 mm outboard, 31 mm
inboard plus the tabs) needs support under it, which is where supports go. The
alternative, floor-down as the plain tray prints, needs two 3 x 97 mm support walls
under the legs instead and leaves marks on the leg faces that grip the rail, so
legs-down is preferred. 0.2 mm layers, 4 walls, 30 % infill, PETG rather than PLA (the
tub flexes and the ESC is the hottest part on the car).

## Measure on the car before printing

1. Tub right rail: width across the top (`RAIL_W`), height of a lip if any and the depth
   the legs can reach before hitting a rib (`RAIL_H`), whether there is a gap under the lip
   for a strap.
2. Rail top height relative to the tub floor and to the plate underside (LAYOUT.md
   `UNDER_CLEARANCE`); the case top is 27.5 above the rail top.
3. Stock ESC boss pattern and height (`ESC_HOLES`, `ESC_STANDOFF`).
4. Servo rear face x and servo top height; receiver box position (not in the chassis
   model, it sits in this bay on a stock Slash).
