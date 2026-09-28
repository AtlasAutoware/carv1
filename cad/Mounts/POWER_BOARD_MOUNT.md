# AtlasPower-4S plate for the clipless mounting system

Files in `out/`: `power_board_mount.step/.stl` (print one), `clipless_foot.step/.stl`
(print two, from `omni20_mount.py`), `power_board_mount_assembly.step` (plate, feet, two
clipless pieces in a stub of baseplate, grey envelope of the board with its components,
leads and edge connectors), `power_board_mount.png` (shaded, two views),
`power_board_mount_assembly.png`, `power_board_mount_lines.png` and
`power_board_mount_assembly_lines.png` (hidden-line). Source: `power_board_mount.py`,
importing `clipless.py`, `render.py` and the helpers in `jetson_orin_nano_mount.py`.

## Component dimensions and where they come from

AtlasPower-4S power board. **There is no manufacturer datasheet or drawing behind these
numbers.** Every dimension below was supplied as a written specification for the build,
not fetched from a vendor page, so none of them carries a source URL the way the rest of
this set does. They are all named parameters at the top of the script. Before the plate is
printed, the outline, the hole pattern and the component heights should be checked against
the board or its fabrication drawing, because a wrong hole pattern here is a scrapped part.

| Item | Value | Source |
| --- | --- | --- |
| Board outline | 88.0 x 68.0 x 1.6 mm, corners R3 | build specification, not a datasheet |
| Mounting holes | four, 3.2 mm, at (+-38, +-28) from the board centre = 76.0 x 56.0 mm pattern | build specification, not a datasheet |
| Tallest components | three shielded inductors 13.5 x 12.5 x 6.2 mm, two electrolytic capacitors 10 x 10.5 mm | build specification, not a datasheet |
| Component height allowance | 12.0 mm above the board top face (`BOARD_TOP_H`) | build specification; larger than the 6.2 mm inductors and the 10.5 mm capacitors, so it covers both |
| Lead protrusion | 2.0 mm below the board bottom face (`BOARD_UNDER`) | build specification, not a datasheet |
| Dissipation | about 8 W, wants airflow underneath | build specification, not a datasheet |
| Connectors | screw terminal and barrel jack on -x; screw terminals and a servo header on +x. Both edges must stay clear | build specification, not a datasheet |

Estimates on top of that, also named parameters: `BOARD_MASS = 90 g` (nothing was given for
the board's weight; it only feeds the two-feet-or-four decision below), `CONN_OUT = 15 mm`
for how far the edge connectors reach past the outline and `CONN_H = 12 mm` for their
height, used only to build the grey envelope that proves the plate does not foul them.

## The plate

92 x 72 x 4 mm, 4 mm corner radius, two clipless feet along the car on
`PEG_PITCH` (40.0 mm, env var), centred on the foot pair. `PITCH_Y` (41.0 mm, env var) is
read too and is reported in the checks, but this part has a single row of feet; see the
last section for why.

It is a flat plate, not a tray. The board has connectors on both x edges, so there are no
walls anywhere: the board is held by four M3 screws into heat-set inserts and nothing else.

- Four bosses, 9.5 mm OD (3.5 mm insert hole plus a 3 mm wall), 8 mm tall, at
  (-38, -28), (-38, +28), (+38, -28), (+38, +28). Insert holes 3.5 mm diameter, 5 mm deep,
  measured down from the boss top at z = 12.
- The board's underside sits at z = 12, i.e. 8 mm of clearance over the 4 mm floor. The
  2 mm of lead protrusion hangs into that, leaving 6 mm of free air under the solder side,
  which is the "at least 6 mm" the brief asked for.
- Two airflow windows, 58 x 12 mm, in the y bands at |y| 20.5 to 32.5, the only full-width
  bands of floor outside the foot flange recesses. About 1390 mm2 of open floor directly
  under the board. With both x edges open as well, air entering at one end leaves through
  the windows and the other end rather than sitting in a pocket under a 8 W board.
- The plate is 2 mm proud of the board outline all round. That is not for the board, it is
  for the bosses: a 9.5 mm boss centred 38 mm out needs its outer edge at 42.75, and a
  92 mm plate puts 3.25 mm of floor outside it. A plate cut to the 88 x 68 board outline
  would leave a 1.25 mm rim there, under the 3 mm minimum. The 2 mm of overhang is 8 mm
  below the board, so it is nowhere near the connectors, which leave the board at z = 12.
- 16.8 cm3, about 12 g in PLA, plus two feet. With the board at an assumed 90 g the total
  is around 100 g, which is the threshold in `CONVENTIONS.md` for two feet along the car.
  If the board turns out to be heavier than about 400 g the rule asks for four, and the
  last section explains why four is not straightforward here.

### Why nothing stands over a foot slot

Each foot needs a 30.5 x 34.5 mm flange recess, so at a 40 mm pitch the recesses reach
x -35.25 to -4.75 and x +4.75 to +35.25 with |y| < 17.25. The board's hole pattern is
76 mm across, which puts the bosses at x +-38 and y +-28: past the recesses in x and
beside them in y, 6.35 mm clear of the nearest recess corner. The vent windows stop at
|y| 20.5, 3.25 mm outside the recesses. So no boss and no window overhangs a slot, and both
feet drop straight in from above.

The script cuts `foot_access()` (the full column above each recess, to 200 mm) anyway, the
same guard `liteon_45w_brick_mount.py` uses. On this layout that cut removes exactly
0.000 mm3 -- measured, by building the plate with and without it and differencing the
volumes -- so it is a guarantee against a future parameter change, not a repair of a
collision that is there now. `checks()` reports `material_over_foot_slots: 0.0`
independently of the cut, by intersecting the finished plate with the same column.

The feet still have to be installed before the board goes on. The board sits 12 mm up and
covers both slots, so the order is: drop the two feet in, then heat-set the four inserts,
then screw the board down.

## Verified in CadQuery (`python3 power_board_mount.py`)

```
plate 92.0 x 72.0 x 12.0 mm, 16.8 cm3 (~12 g PLA at 60% effective), board 88.0 x 68.0 on 76.0 x 56.0 holes, feet at [(-20.0, 0.0), (20.0, 0.0)] (pitch 40.0 x 41.0)
  solids: 1
  material_over_foot_slots: 0.0
  plate_x_feet: 0.0
  feet_x_clipless: 0.0
  plate_x_plate_stub: 0.0
  plate_x_envelope: 0.0
  feet_x_envelope: 0.0
  feet: [(-20.0, 0.0), (20.0, 0.0)]
  bosses: [(-38.0, -28.0), (-38.0, 28.0), (38.0, -28.0), (38.0, 28.0)]
  board_underside_z: 12.0
  plate_covers_adjacent_hole_across: 9.25
  boss_to_foot_recess: 6.35
  boss_to_plate_edge_x: 3.25
  boss_to_plate_edge_y: 3.25
  vent_to_foot_recess_y: 3.25
  vent_to_plate_edge_y: 3.5
  vent_to_boss: 4.25
  strip_between_foot_recesses_x: 9.5
  clearance_under_board: 8.0
  free_air_under_leads: 6.0
  min_wall_ok: True
```

Intersection volumes are in mm3, between the plate and the feet, the feet and the clipless
pieces, the plate and the baseplate stub, and the plate and the envelope (board outline,
12 mm component block, 2 mm leads inside the hole pattern, and the connector blocks
reaching 15 mm off each x edge) placed where it really sits. `boss_to_foot_recess` is a
true circle-to-rectangle distance in plan, not a single-axis difference: a boss can be past
a recess in x and still be clear because it is beside it in y, which is the case here.
`min_wall_ok` is every `boss_to_*` and `vent_to_*` gap being at least 3 mm.

Installability, from `python3 audit_feet.py` with this part added to `SET`:

```
PASS power_board_mount          solids=1  feet=2 pitch x[40.0] y[] ok  install=+z:0,+z:0  installable

0 part(s) failed
```

and the file actually on disk, measured through the peg holes in the STL:

```
  power_board_mount        holes [(-20.0, 0.0), (20.0, 0.0)]  pitch x[40.0] y[]
```

## Printing

Flat on the bed, bosses up, no supports. Everything is a vertical hole or a vertical boss;
the foot recess is a pocket in the top of the floor over a through-hole that goes to the
bed, so its ledge is built up from the plate rather than bridged over air. 0.2 mm layers,
3 walls, 30 % infill. Heat-set the four M3 inserts before the board goes on, and put the
two clipless feet in first.

## Two open points

`INSERT_D` is 3.5 mm, as the brief specified for the M3 heat-set inserts. `CONVENTIONS.md`
says 4.0 mm for M3 and 3.5 mm for M2.5, so these disagree. The brief won because it is the
more specific instruction, and the value is an env var (`INSERT_D=4 python3
power_board_mount.py`) so it can be moved without touching anything else. The 9.5 mm boss
has enough meat for either: at 4.0 mm the wall is still 2.75 mm, marginally under the 3 mm
rule, so if 4.0 is the right number `BOSS_OD` should go to 10.0 at the same time. Worth
settling against the actual inserts before printing.

`plate_covers_adjacent_hole_across: 9.25` means the plate reaches 9.25 mm over the
neighbouring baseplate hole column across the car, which is unavoidable: the board is 68 mm
wide and the hole pitch across is 41 mm. That column is not usable by another mount's feet
under this plate. It also rules out the obvious four-foot version: a second row of feet at
y +-20.5 would put flange recesses at y 3.25 to 37.75, which collides with the bosses at
y +-28 in exactly the way this whole document is about, and would need the plate about
82 mm deep. If four feet become necessary, the feet have to move in x as well, and the
audit should be re-run before anything is printed.
