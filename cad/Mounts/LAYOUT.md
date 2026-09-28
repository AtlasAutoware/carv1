# Whole-board layout on the real Baseplate, above and below

`python3 board_layout.py` regenerates the four under-board mount variants
(`out/*_under*`), builds the layout, runs the checks and writes
`out/board_layout_assembly.step`, `out/board_layout_top.png`, `out/board_layout_bottom.png`,
`out/board_layout_side.png`, `out/board_layout_front.png`, `out/board_layout_lines.png` and
`out/board_layout_report.json` (hole table, claims, interference rows, clearances, chassis
parameters). `UNDER_CLEARANCE=40 python3 board_layout.py` re-runs at another clearance;
`ATTEMPT=jetson|vesc|tidy` places one of the mounts that do not fit and prints why
(section 7); `SKIP_REGEN=1` skips the mount-script regeneration. Baseplate v2, the
laser-cut replacement plate with the 40 x 41 grid, is sections 12-18
(`PLATE=v2 python3 board_layout.py`); section 19 is the final set on v2, with the cable
tidy on the plate edge and the VESC on a tub saddle, neither on clipless feet, and with the
lidar moved to the front (B1+B2) and the camera mast behind it (A3+B3) so the lidar's
forward half is unobstructed; section 19 also has the lidar's occluded sectors and the
camera's line of sight over the lidar per pitch step.

## 0. Result in one paragraph

The nine mounts do not all fit on this plate. Five of them do, at zero interference, and
only after cutting four extra 28.5 mm holes in the plate: camera mast, RPLIDAR C1 plinth,
Omni 20+ cradle and Lite-On cup above, the small-board plate below. The Jetson tray, the
VESC tray and the cable tidy cannot be placed on the clipless grid of this plate with this
chassis underneath, and the LiPo frame stays the documented above-board alternative (the
pack stays in the chassis tray). The reasons are geometric, not tuning: (a) the plate has
five hole columns and the above-board line (mast 1 column, lidar 2, cradle 2, cup 1 with a
column under its floor) needs seven; (b) a clipless piece inserted from above puts its
33 x 33 x 12.85 mm flange on TOP of the plate, so a hole cannot be used from below where
an above-board mount covers it, and the cradle covers three of the five columns; (c) the
two deep under-board trays (Jetson 46.3 mm, VESC 38.9 mm below the plate top) need the bare
right-hand half of the tub between the steering servo and the motor, and that region has
no usable holes left once (b) is applied. Every one of these is shown by the script with
numbers (section 7). What would change the answer is in section 9.

## 1. The real Baseplate (templates/Baseplate.stl)

Read with trimesh (`read_baseplate()` in `board_layout.py`): the STL is not watertight and
contains ten bodies. The plate body is 452.0 x 177.5 x 3.175 mm (1/8 in stock, not the
2 mm in `clipless.py`; the task said to treat it as 2 mm, so `PLATE_T` defaults to 2.0 and
is an env var. With 3.175 the clipless rim ends 0.045 mm below the plate top and mounts rest
on the plate instead of on the rim tops; nothing in the layout changes except that every
above-board z drops by 1.13 mm). The section at mid-thickness gives, in STL coordinates:

- outline 0..452 x 0..177.5, centre (226, 88.75);
- 15 square holes, 28.5 x 28.5, on an exactly regular 50.0 x 47.5 grid: x = 114.25,
  164.25, 214.25, 264.25, 314.25; y = 40.75, 88.25, 135.75. The "irregular 47.9 to 53.1 mm
  pitch" in CONVENTIONS.md and clipless.py is not in this STL; the grid centre is 11.75 mm
  off the plate centre along the plate and 0.5 mm across;
- two pairs of rectangular cutouts, 40.48 x 37.1 at (45.64, 50.125 / 127.375) and
  40.48 x 38.1 at (365.08, 47.625 / 129.875): pair spacing 77.25 mm at one end, 82.25 at
  the other, 319.4 mm between the pairs;
- four 3.175 mm holes at (52.73 / 375.73, 72.23 / 105.27), 33 mm apart across the centre
  line, 7 to 11 mm inboard of the cutout pairs;
- two clamp assemblies under the plate at x 49.93..65.53 and 371.34..386.94, y 68.7..108.8
  (15.6 x 40.1 mm, two stacked blocks each, reaching 22 mm below the plate underside), on
  the 3.175 holes;
- one loose 33 x 33 x 20 body at (-187.5..-154.5, -80.5..-47.5), a clipless piece left in
  the export, ignored.

Interpretation: the 3.175 mm holes and the clamp blocks are the plate's fixing to the car:
15.6 mm thick clamps, 40 mm wide, centred on the car's centre line and reaching 22 mm down,
grip the top edge of the front and rear shock towers. The 40 x 37 cutouts at +-38.6 and
+-41.1 mm from the centre line are where the Traxxas body posts, which lean outward from
the towers, pass through the plate (the body-clip holes then sit above the plate). So the
plate is tied to the chassis at the two shock towers: tower planes at the clamp centres,
board x = +168.3 and -153.1, body posts at the cutout centres.

Board frame: +x forward, +y left, +z up, origin at the plate centre, plate top at z = 0.
Front = the STL x = 0 end (`FRONT_END=xmin`, so board x = 226 - x_stl, board y =
88.75 - y_stl). Reasons: the cutout pair at that end is the narrower one (77.25 vs 82.25 mm)
and on the Slash 2WD the front body posts (on the front shock tower) sit closer together
than the rear posts on the rear body mount; the hole grid is shifted 11.75 mm toward that
end, which puts the lidar and camera columns closer to the front. Verify by measuring the
post spacing on the car; `FRONT_END=xmax` flips it, but the new holes in section 4 are
placed for `xmin` (the script shows them hitting the clamps and posts if flipped).

Hole names: columns c1 (front) .. c5 (rear) at board x = +111.75, +61.75, +11.75, -38.25,
-88.25; rows L (left, y = +48.0), C (y = +0.5), R (right, y = -47.0).

## 2. Both faces of the board and what that costs

Used from below, a clipless piece has its flange under the plate and its pocket opening
upward (the way the set was designed). Used from above (flange on the plate top, rim down
through the hole, pocket opening downward) a foot plugs in from below and a mount hangs on
it, contact face 3.13 mm below the plate top (`snap_retain.UNDER_Z`). Hanging mounts need
positive retention: `RETAIN=snap` was added to `jetson_orin_nano_mount.py`,
`vesc_fsesc67_mount.py`, `liteon_45w_brick_mount.py` and `small_board_plate_mount.py`
(additive; default `recess` reproduces the old parts exactly). It swaps the flange recesses
for the dovetail slide channel + spring tongues of `rplidar_c1_mount.py` through the new
helper `snap_retain.py` (which imports `slide_channel` / `snap_tongues` from the plinth
script). Outputs get the `_under` suffix; each script re-runs its checks (single solid,
part x feet at nominal / at the stop / at the bump, feet x clipless pieces, part x plate,
part and feet x envelope) and reports the hanging depth. Results at the real 50 mm pitch:

| variant | change | depth below plate top | checks |
| --- | --- | --- | --- |
| `jetson_orin_nano_under` (`SNAP_ROWS=1`, two feet) | floor 4 -> 8.6 mm (channel + 1.6 roof), one channel on the centreline, windows kept outside the 40.5 band | 46.3 mm | all 0.0 mm3, play 1.25 |
| `vesc_fsesc67_under` | the two 9 mm foot pads become one 40.5 mm bar carrying the channel, tabs widened to 54 with the tie slots outside the band | 38.9 mm (unchanged) | all 0.0 mm3, play 1.25 |
| `small_board_plate_under` (`SNAP_FEET=1` or 2) | floor 8.6 mm, grid holes kept out of the band, +y strap slot moved to y 24 | 31.3 mm | all 0.0 mm3, play 1.25 |
| `liteon_45w_brick_under` | floor 8.6 mm, channel across the car in a 40.5 x 87 bar | 42.0 mm | all 0.0 mm3, play 1.25 |

The cost nobody had priced: a piece inserted from above is a 33 x 33 x 12.85 mm block on
the plate top. Any above-board mount that covers that hole collides with it (the Omni
cradle covers c4 and c5 on all rows and reaches 8.6 mm over the c3 flanges). Symmetrically,
every above-board hole is a 33 x 33 x 12.85 block under the plate that an under-board tray
cannot cross. This halves the usable grid on each face.

## 3. Chassis keep-out model (Traxxas Slash 2WD 58034)

Sources: Traxxas owner's manual for the Slash 2WD, model 58034-8 / 58234-8
(https://traxxas.com/media/productattach/C-58234-8/2/58234-8-om-en-r00.pdf): battery
hold-down "25mm side / 23mm side" (stick pack 25 mm tall, side-by-side 23), Titan 12T 550
motor, XL-5 ESC 1.23 x 2.18 x 0.75 in, battery installed "with the battery wires facing the
rear of the model". Traxxas product specifications for the Slash 2WD
(https://traxxas.com/products/models/electric/58034-8-slash, quoted by HobbyTown /
AMain listings): wheelbase 335 mm, length 568, width 296, height 214, ground clearance
89 mm, 2.16 kg. Exploded views (https://traxxas.com/explodedviews/Slash-58034-2-Front-Assembly)
give part numbers, not dimensions. Everything below not in those two sources is an
ESTIMATE and a parameter in the `CHASSIS` dict of `board_layout.py` (override with an env
var of the same name, JSON value). All z values are relative to the tub rim top
`Z_RIM = -PLATE_T - UNDER_CLEARANCE`.

| item | model | source / status |
| --- | --- | --- |
| shock towers | 4 mm plates at the clamp centres, x = +168.3 (front) and -153.1 (rear), 120 wide, top edge 16 mm below the plate underside (inside the clamp) | from the STL clamps; `TOWER_TOP` estimate |
| axles | front tower + 10, rear tower - 4: x = +178.3 / -157.1, wheelbase 335.4 | 335 from Traxxas; the split is an estimate |
| body posts | 7 mm, at the cutout centres (+180.4, +-38.6) and (-139.1, +-41.1), from the tower top to 25 mm above the plate | cutouts from the STL; height estimate |
| shock caps | 20 mm cylinders at y +-42 on each tower, 90 long, from the tower top down | estimate |
| tub | 300 x 100 mm, 28 deep, 3 mm walls, rim at Z_RIM | estimate (Slash tub between the bulkheads) |
| battery tray | left side, x -95..45, y +3..+50, top = rim + 6 (25 mm pack + hold-down + clip) | pack height from the manual; position estimate |
| gearbox + spur cover | x -165..-100, y -30..+22, top = rim + 20 | estimate; must be below the plate clamps (24 mm down), which fixes rim + 20 as the maximum consistent with the plate fitting at all |
| motor | 3660 class (36 dia x 60), axis along y, centre x = -120 (37 mm ahead of the rear axle, where the spur meshes), can from y -22 (pinion end at the transmission) to -82 on the RIGHT, axis 2 mm below the rim, so the can top is rim + 16 | the stock Titan 550 is 36 x 57; the Slash 2WD motor sits to the right of the transmission with the pinion inboard, which is why it is on the right (`MOTOR_SIDE=-1`) |
| steering servo | standard servo lying across the car in the front of the tub, x 122..142, y +-20, top = rim + 2 | estimate |
| linkage sweep | servo saver, bellcrank and drag link, x 128..175, y +-40, up to rim + 15 | estimate |
| tyres | 110 dia x 42, centres at y +-127, tyre top at rim + 34 at ride height, swept 30 mm up for full bump; front also swept +-30 degrees about the vertical axis for full lock | Traxxas width 296; the rest estimate |

`UNDER_CLEARANCE` (plate underside to the tub rim top) defaults to 45 mm. The script's own
consistency checks on the estimates: at 45 mm the plate clamps clear the gearbox model by
2 mm; at 40 mm the rear clamp hits the spur cover model by 1251 mm3 and at 35 mm the front
clamp hits the steering linkage model too, so if the real clearance is under about 42 mm the
gearbox and linkage numbers above are too high and must be re-measured. The front tyres at
full lock plus full bump graze the plate's front corners in every case (0.7 mm3 at 45 mm,
24.6 at 40, 70 at 35): the plate is a rectangle where the body has wheel arches. Check that
on the car before trusting any of the front-corner placements.

## 4. Hole assignment

New holes: this layout needs four more 28.5 mm holes cut in the plate (`EXTRA_HOLES=1`,
default). They are placed so their clipless flanges (33 mm) miss the plate clamps and
the body posts:

- n1C (143.0, +0.5) and n1L (143.0, +48.0): the camera mast pair, 31.25 mm ahead of c1.
  Their flanges end at x 159.5, 1 mm short of the front clamp (160.5).
- n6L (-180.0, +24.25) and n6R (-180.0, -23.25): the Lite-On cup pair behind the
  cradle, straddling the C row at the cup's 47.5 mm foot pitch. Flanges end at x -163.5,
  2.6 mm short of the rear clamp (-160.9); 17 mm clear of the body posts.

| mount | face | holes | centre (x, y) | orientation | notes |
| --- | --- | --- | --- | --- | --- |
| camera mast (Gemini 335) | above | n1C, n1L | (143.0, +24.25) | camera forward, channel open to +y | camera 24 mm left of the centre line, lens at x 162; nothing else can take c1C (section 7) |
| RPLIDAR C1 plinth | above | c1R, c2R | (86.75, -47.0) | connector notch aft | front right, not centre: the centre row cannot hold it (section 7); `LIDAR_SCAN_CLEARANCE=45.5` puts the scan plane 45.93 above the board, 10.5 mm over the tallest other item (cup fingers 35.43) |
| Omni 20+ cradle | above | c4C, c5C | (-63.25, +0.5) | AC outlet edge facing the rear | cradle x -130.35..3.85, 5.2 mm ahead of the rear posts; pack top 32.1 |
| Lite-On 45 W cup | above | n6L, n6R | (-169.75, +0.5 + OUTLET_Y) | turned 180, prongs forward into the pack | brick centre 106.5 behind the cradle centre (pack half 63.5 + 0.5 gap + 42.5); `FEET_X_OFFSET` computed = 10.25; cable end at x -217.75, 8 mm inside the plate edge |
| small-board plate (PCA9685 + hub) | below | c2L (one foot, `SNAP_FEET=1`) | (61.75, +48.0) | channel open aft | x 20.25..120.25; PCA over the tub, hub outboard of the tub |
| Jetson Orin Nano tray | below | none | - | - | does not fit, section 7 |
| VESC FSESC 6.7 tray | below | none | - | - | does not fit, section 7 |
| cable tidy | below | none | - | - | does not fit, section 7 |
| LiPo SMC 9000 frame | (above, optional) | 2 x 2 on any adjacent columns | - | - | the pack stays in the chassis tray; there is no free 2 x 2 block on the plate with the set above, so this stays the alternative for a car without the Omni |

Unused holes: c1L, c1C, c2C, c3L, c3C, c3R, c4L, c4R, c5L, c5R and n1R. None of them is
usable from below except c1L and c2C (all the others are under the cradle or its flange
zone, or their below-side flange would sit under the lidar / mast). None is usable from
above except c1L, c2C, c3C (blocked by the lidar bar / mast bar / cradle front for anything
longer than 20 mm).

Checks in the script: no hole claimed twice (asserted), every foot's peg centre within
2.5 mm of a claimed hole centre (asserted; the pegs float +-2 mm along their pitch axis by
design), every claimed hole exists in the plate model.

## 5. Above / below split and heights

Above (z from the rim tops, 1.13): mast 132.4 tall (camera lens 150 at 0 pitch), lidar top
57.4, cup fingers 35.4, brick 33.1, pack 32.1, cradle posts 19.1. Scan plane 45.93.
Below: small-board plate 31.3 deep (PCA9685 headers), 22.7 (hub).

## 6. Clearance under the board

Mount bottom minus the top of the chassis item it is over (mm), from
`clearance_report()`:

| mount | depth below plate top | UNDER_CLEARANCE 45 | 40 | 35 |
| --- | --- | --- | --- | --- |
| small-board plate (placed, c2L) | 31.3 | +9.7 over the battery hold-down | +4.7 | -0.3 |
| Jetson tray (attempt, c2R + c3R) | 46.3 | +0.7 over the tub rim | -4.3 | -9.3 |
| VESC tray (attempt, c2L + c3L) | 38.9 | +2.1 over the battery hold-down | -2.9 | -7.9 |
| cable tidy (attempt, c1L + c2L) | 26.6 | +5.4 over the steering linkage | +0.4 | -4.6 |
| Lite-On cup under (not used under) | 42.0 | would need the bare tub | - | - |

So even where the hole grid allowed them, the Jetson fits only at 45 mm (0.7 mm) and the
VESC only at 45 mm (2.1 mm, over the battery). At 40 and 35 mm nothing deeper than 31 mm
hangs under this plate anywhere over the tub.

## 7. Interference table

Pairwise intersection volume (mm3) over every pair of solids whose bounding boxes overlap:
mounts (mast, head, plinth, cradle, cup, plate), 9 feet, 9 clipless pieces, 6 component
envelopes, 19 chassis keep-outs, the plate and its 4 clamp blocks; 54 solids, all pairs
except the ones that nest by design inside one mount (part/feet/pocket/envelope), the
brick's prongs inside the pack's outlet (317 mm3 of plug engagement, listed separately),
chassis vs chassis, and the clamps against the towers they clamp.

Final layout, UNDER_CLEARANCE 45: zero interference between any mount, foot, piece,
envelope and the chassis or board. The only non-zero rows are the chassis-vs-plate ones
from section 3 (front tyres at full lock + full bump vs the plate's front corners,
0.7 mm3 each side).

Why the other three do not fit (`ATTEMPT=...`, same script, same checks):

| attempt | holes | non-zero rows (mm3) | meaning |
| --- | --- | --- | --- |
| Jetson (c2R, c3R), the only place its 46 mm depth clears the servo, linkage, motor and battery | c2R, c3R | cradle x clipless_c3R 1031; pack x clipless_c3R 1001; cradle x Jetson foot0 249 | the from-above piece in c3R stands on the plate top where the cradle's floor is (the cradle front edge is at x 3.85, the c3 flange reaches 28.25). Moving the Jetson one column forward puts its heatsink over the steering linkage (bottom -46.3 vs linkage top -32); one column back puts it over the motor (top -31). |
| VESC (c2L, c3L), turned so the channel and phase leads open forward | c2L, c3L | cradle x clipless_c3L 1031; pack x clipless_c3L 1001; cradle x VESC foot1 249 | same c3 flange conflict; c4L/c5L are over the gearbox (case bottom -36.6 vs spur cover -27) and under the cradle's c4C/c5C flanges; c1L/c2L run into the mast's n1 flanges (x 126.5..159.5). |
| cable tidy (c1L, c2L), lip outboard | c1L, c2L | mast piece n1L x rail 1747; n1L x cables 1247; mast x clipless_c1L 961; mast foot1 x clipless_c1L 132; cables x shock cap 5 | the 150 mm rail runs under the mast's pieces at x 126.5..159.5 and its own c1L piece stands on the plate top inside the mast's floor bar. Rows R and C are the lidar's and the cradle's; c4/c5 are under the cradle. |

Also tried by hand and rejected before this table (numbers in the reasoning, all
reproducible by editing `build_mounts()`): lidar on the C row at (c2C, c3C) or (c3C, c4C)
hits the cradle's front border by 11.6 mm (plinth bar 44.5 behind its rear foot, cradle
67.1 ahead of its front foot, 111.6 > 100); lidar on a side row next to the cup or mast
across-bars overlaps them by 1.8 mm (cup) or 3.3 mm (mast), because 42 + 31.05 > 47.5 +
23.75; cradle rotated (one column, rows C+L) lands on the rear-left body post; mast at
c1 (C,L) leaves no adjacent pair for the lidar anywhere.

## 8. Chassis tie-ins and how the mounts sit relative to the car

Front clamp at x 160.5..176.1 grips the front tower (168.3); rear clamp -160.9..-145.3
grips the rear tower (-153.1); body posts through the four cutouts with clips above the
plate. The camera lens ends up 16 mm behind the front axle line, 150 mm above the plate;
the lidar centre 92 mm behind the front axle, 47 mm right of the centre line; the Omni pack
centred 94 mm ahead of the rear axle; the brick over the rear tower with its cable end
57 mm ahead of the plate's rear edge; the small-board plate over the front half of the tub's
left side, outboard of the battery.

## 9. What has to change for the rest to fit

1. The plate. The C row is two holes short for the above-board line and the two deep trays
   need a clean pair of holes over the bare right of the tub. A plate 100 mm longer at the
   front (one more column ahead of the front clamp, c0) or a second row pitch would put the
   mast on c0, the lidar on (c1C, c2C) and free (c2R, c3R) for the Jetson from below and
   (c2L, c3L) for the VESC, still with the cradle's c3 flange problem, so:
2. The Omni cradle. A version 12 mm shorter, or with its feet at the rear pair only
   (c5C plus a hole in the rear overhang, which the clamp forbids), or turned so its outlet
   faces +y with a cup beside it on new holes at y = +-75, would release column c3.
3. A from-above clipless piece with a thin flange. If the flange on the plate top were
   2 mm instead of 12.85 (the mounts rest 1.13 mm above the plate on the rim tops), the
   above-board mounts could pass over holes used from below, and the flange zones in
   section 2 disappear. That is a new template part, not a mount.
4. The VESC does not need the clipless grid: it replaces the stock ESC and the Slash's
   right-hand ESC bay is under the plate at exactly the depth the tray cannot reach. A tub
   bracket for the FSESC is the shorter route.
5. The cable tidy at 100 mm (four posts) fits on (c1L, c2L) only if the mast moves back to
   c1; at 150 mm it needs a row of its own.

## 10. Cable routing

Power: the Omni's DC barrel / USB-C ports open on its +-y edges, mid-edge, so the 19 V
barrel lead from the brick (cable end at the plate's rear edge, x -218) turns forward along
the right edge of the plate, under it, to the Jetson wherever the Jetson ends up (with
this layout: not on the plate), and the USB-C-to-barrel charge lead loops back to the brick
region. Signal: the lidar's XH cable drops out of the plinth's rear notch at x 42 on the
right row and runs aft along the right edge to a USB adapter; the camera's USB-C leaves the
mast head on the -y (right) side at 137 mm and comes straight down the mast's rear legs to
the same right-edge run; the PCA9685 and hub sit under the left front, so their USB lead
goes across under the plate at c2 (the C row holes there are unused on both faces, so
there is a 28 mm slot for it) to the right-edge run. The VESC's phase leads, when it is
mounted, leave its case end toward the motor at the rear right; its XT90 goes to the pack
in the left battery tray (leads face the rear per the manual). With no cable tidy on the
plate, slack is coiled under the cradle's floor windows (14 mm clear between the cradle
floor and the plate) or in the 60 mm of open plate ahead of the cradle on the right.

## 11. Measure on the car before printing

1. Plate thickness (STL says 3.175 mm, the set assumes 2.0). Sets whether mounts rest on
   the rim tops or on the plate.
2. Plate underside to the tub rim (`UNDER_CLEARANCE`), and to the battery hold-down clip,
   the spur cover and the servo horn: these decide whether anything deeper than 31 mm can
   hang under the plate at all.
3. Body post spacing front vs rear (decides `FRONT_END`), and whether the posts are still
   fitted and how far they stand above the plate.
4. Which Omni 20+ edge carries the AC outlet and where along that edge its centre is
   (`OUTLET_Y`, default 0 = mid-edge); the new cup holes n6L/n6R follow it.
5. Motor position (can end and pinion end in x and y, can top height) and which side of the
   transmission it is on (`MOTOR_SIDE`).
6. Front tyre clearance to the plate's front corners at full lock with the suspension
   compressed.
7. The Lite-On brick's real size (the cup is built on an 85 x 58 x 28 estimate) and the
   FSESC case height (95 x 92 x 24.5 vs the older 100 x 92 x 22.5).
8. The clamp height on the towers (the 22 mm in the STL) and that the four 3.175 holes are
   where the STL has them, because the new holes n1 and n6 are placed 1 to 2.6 mm from the
   clamps.

---

# Baseplate v2

`PLATE=v2 python3 board_layout.py` builds the plate from `baseplate_v2.py` instead of
reading the STL, regenerates the mount variants it needs (`out/rplidar_c1_v2.*`,
`out/small_board_plate_v2.*`, `out/liteon_45w_brick_v2.*`, `out/cable_tidy_100.*`,
`out/jetson_orin_nano_under.*`), places the set, runs the same checks and writes
`out/board_layout_v2_assembly.step`, `_top.png`, `_bottom.png`, `_side.png`, `_front.png`,
`_lines.png` and `_report.json`. `python3 baseplate_v2.py` alone writes the cut file
`out/baseplate_v2.dxf`, `out/baseplate_v2.step`, `.stl` and the hole map
`out/baseplate_v2.png`. Variants: `ATTEMPT=tidy` / `ATTEMPT=vesc` add the two mounts that
do not fit at their closest position (outputs `board_layout_v2_tidy_*`, `_vesc_*`);
`PITCH_Y=40 TAG=_p40` is the strict 40 x 40 grid; `POSTS=both TAG=_posts` keeps the rear
body posts; `UNDER_CLEARANCE=40 TAG=_uc40`.

## 12. Result in one paragraph

(Sections 12-18 are the clipless-only attempt; section 19 supersedes their "does not
fit" rows for the cable tidy and the VESC, and their mast / plinth positions: the plinth is
now in front on B1+B2 and the mast behind it on A3+B3, see section 19.) Six of the nine mounts fit on Baseplate v2
with zero interference, one more than on the
STL plate, and the one that was the point of the exercise, the Jetson tray, is among them:
above the board the camera mast (A1+B1), the RPLIDAR C1 plinth (A2+A3), the Omni 20+
cradle (B5+B6), the Lite-On cup plugged into the pack (A8+C8) and the small-board plate
(D1+D2); below it the Jetson tray (D4+D5). The cable tidy, the VESC tray and the LiPo frame
do not fit; the reasons are in section 17 and they are the same reason in three costumes:
a clipless piece is a 33 mm square block on whichever face it is inserted from, so every
hole a mount uses from one face costs the 33 mm around it on the other face, and with
40 mm between holes that is 82 % of the pitch. A grid that is denser in the plate cannot
be denser in use than the piece allows. Two things were changed on the way and are
flagged: the row pitch is 41, not 40 (section 13), and the rear body posts come off
(section 15); `PITCH_Y=40` and `POSTS=both` show what each costs.

## 13. The plate (`baseplate_v2.py`, `out/baseplate_v2.dxf`)

Same outline as the STL (452.0 x 177.5, 3.175 mm; `PLATE_T` is a parameter and
`board_layout.py` uses it for the hanging depths), the same two pairs of body-post
cutouts (40.48 x 37.1 at (180.36, +-38.62), 40.48 x 38.1 at (-139.08, +-41.12)), the same
four 3.175 mm clamp holes at (173.27 / -149.73, +-16.52). All of these are measured from
the STL by `read_baseplate()` (moved into `baseplate_v2.py`, `board_layout.py` imports it
from there) and re-emitted, not typed in, so the plate drops onto the same tower clamps.

Clipless grid: 28 holes, 28.5 mm square, columns 40 mm apart along x, rows 41 mm apart
across y:

| | x | notes |
| --- | --- | --- |
| column 1 | +115 | 30.9 mm web to the front cutouts; a piece from below has its flange 8.5 mm behind the front clamp |
| columns 2..6 | +75, +35, -5, -45, -85 | 11.5 mm webs |
| column 7 | (none) | would be in the rear cutouts and clamp |
| column 8 | -185 | 100 mm behind column 6: 11.45 mm web to the rear cutouts, 26.75 to the rear edge; an on-pitch -205 would leave 6.75 to the edge, -165 would leave 6.45 to the cutouts. A from-below piece here is 7.6 mm clear of the rear clamp block |
| rows A..D | y = +61.5, +20.5, -20.5, -61.5 | A = car left; 13.0 mm web to the long edges, 12.5 between rows |

Why 41 across and not 40: a mount centred on a row reaches out to the flange zone of the
row two pitches away. The Omni cradle is 129.2 wide (64.6 from its row); a clipless piece
inserted from above has a 33 mm flange on the plate top (16.5 from its row); at a 40 mm
pitch these overlap by 64.6 - (80 - 16.5) = 1.1 mm on every row-D hole under the cradle,
and those are the only holes the Jetson tray can hang from (section 15). 41 is the smallest
pitch that clears it (0.9 mm). `PITCH_Y=40 TAG=_p40` reproduces the strict grid: the cradle
hits the Jetson's D4 and D5 pieces by 289 and 145 mm3 and nothing else changes. Everything
that has feet across the car reads the pitch (`PITCH_Y`), so the mast, the cup and the LiPo
frame are built at 41 (or 82) for this plate.

Also cut: 22 zip-tie holes, 6 mm, at y = +-82 every 40 mm (x = -205, -145, -105 .. 215;
3.75 mm to the edge, 3.6 mm to the nearest clipless-hole corner, the check threshold for
these unloaded holes is 3.5) for strapping cable runs along both long edges; four M3 holes
(3.2 mm) at (155, +-82) and (-125, +-82) for optional under-board rails along the long
edges (280 mm between them; the Jetson tray overhangs the right edge, section 15, so a
right-hand rail is not compatible with this layout). The 8 mm web rule for the clipless
holes is asserted in `check_webs()` (to the outline, the cutouts, the clamp holes and each
other). No cable slots were needed (section 16); `CABLE_SLOTS` in `baseplate_v2.py` takes
`(x, y, length, 12)` tuples if that changes.

DXF: mm, R2010, closed LWPOLYLINEs for the outline, the 28 clipless holes and the four
cutouts; CIRCLEs for the 3.175, 6 and 3.2 mm holes; one layer per feature (OUTLINE,
CLIPLESS, CUTOUT, CLAMP, ZIP, M3) plus a LABELS layer with the hole names that the cutter
should ignore. Plate 161 cm3, about 110 g in birch ply.

## 14. Cut instructions

- 3.175 mm (1/8 in) birch ply, as the STL. Cut on the line, no kerf compensation: a
  laser kerf of 0.15 to 0.2 mm makes the 28.5 holes come out about 28.6 to 28.7, which is
  what the 28.0 rim wants (0.3 mm a side; the piece's flange, not the rim, locates it
  against the plate). Grain along x. Seal the edges if the car gets wet: the clipless
  flanges clamp the plate faces, not the edges, so a swollen edge does not matter but a
  swollen face does (12.85 mm flange, 3.13 rim: on a 3.175 plate the rim already ends
  0.045 mm below the top face and every above-board mount sits on the ply, see below).
- 3 mm acrylic instead: the rim then stands 0.13 mm proud and the mounts sit on the rim
  tops as the 2 mm design intended, but cast acrylic cracks from sharp inside corners
  under vibration, so ask for the four corners of every 28.5 hole with a 0.3 mm radius
  (the rim is a sharp 28 x 28 square with 0.25 mm a side of room, so a 0.3 radius does not
  touch it) and expect the holes at 28.5 to 28.6 with the narrower acrylic kerf. Do not
  countersink or drill the 3.175 clamp holes larger in acrylic; the clamps pull the
  plate onto the towers and a cracked clamp hole is a lost plate.
- Either material: check the first plate on the car with only the clamps fitted, then
  with one clipless piece in each of A1, D1, A8, D8 (the corners of the grid) and a
  straight edge across the pieces' rim tops: a plate that is not flat between the two
  clamps will lift the cradle's feet on one side.
- The heights in this section assume the mounts sit on the plate top (`ABOVE_Z = 0`, the
  3.175 case). On 3 mm acrylic they sit 0.13 higher; nothing changes.

## 15. Placement on v2 (hole assignment, both faces)

(Superseded for the camera mast and the lidar plinth, and for the hole table: section 19
has the plinth on B1+B2 and the mast on A3+B3. The rows below are kept as the record of the
first v2 placement, where the mast stood in front of the lidar.)

| mount | face | holes | centre (x, y) | rot | where it ends up |
| --- | --- | --- | --- | --- | --- |
| camera mast (Gemini 335) | above | A1, B1 | (115, 41) | 0 | column 1, feet across on the two left rows; floor bar x 94.75..135.25, y 0.75..81; lens at x 134, 44 mm behind the front axle, 41 mm left of the centre line, 148.9 above the plate top (`CAM_HEIGHT` 150 assumed the 1.13 rim; set 151.13 to restore 150) |
| RPLIDAR C1 plinth | above | A2, A3 | (55, 61.5) | 0 | left row, columns 2 and 3, connector notch aft; body x 23.95..86.05, y 30.45..92.55 (3.8 mm past the left edge); floor bar x 15.5..94.25 with `BAR_END=3.5` (was 5) so it stops 0.5 mm short of the mast bar; scan plane 44.8 above the plate top, 10.5 over the cup's fingers (34.3) |
| Omni 20+ cradle | above | B5, B6 | (-65, 20.5) | 0 | x -132.1..2.1, y -44.1..85.1; 3.5 mm ahead of where the rear posts were; 0.9 mm from the row-D flanges (the 41 mm story) |
| Lite-On 45 W cup | above | A8, C8 | (-171.5, 20.5) | 180 | feet 82 apart (`PITCH_Y=82`) straddling row B, `FEET_X_OFFSET` 13.5 so the pair lands on column 8; brick x -214..-129, prongs into the pack's rear face at -108.5 with the 0.5 mm plug gap; cup ends at x -219.5, 6.5 inside the rear edge; cup bar y -38.75..79.75 |
| small-board plate (PCA9685 + hub) | above | D1, D2 | (95, -61.5) | 180 | right row, turned so the long end points aft, `PLATE_OFFSET=10.75` (was 8.5): x 34.25..134.25, y -91.5..-31.5 (2.75 mm past the right edge); its flanges under the plate end at x 58.5, clear of the Jetson tray; PCA at (70, -77), hub at (87, -47), PCA header tops 23.6 above the plate |
| Jetson Orin Nano tray (`_under`, one channel, two feet) | below | D4, D5 | (-25, -61.5) | 180 | tray x -78.5..28.5, y -104.5..-18.5 (the outboard 15.75 mm hangs past the plate's right edge), connectors outboard, snap fingers at the rear end; module and heatsink x -68..18, y -82.5..-28.5, bottom 46.35 below the plate top, 1.83 mm above the tub rim at UNDER_CLEARANCE 45; 31.5 mm from the battery bay's edge (y 3), 34 mm ahead of the motor can (x -102); its pieces on the plate top are 0.9 mm from the cradle |
| cable tidy | - | - | - | - | does not fit, section 17 (`ATTEMPT=tidy`: C1+C2 below is the closest) |
| VESC FSESC 6.7 tray | - | - | - | - | does not fit, section 17 (`ATTEMPT=vesc`: C2+C3 below) |
| LiPo SMC 9000 frame | (above, optional) | B5, C5, B6, C6 | (-65, 0) | 0 | the pack stays in the chassis tray. If the Omni is not carried, the frame goes where the cradle was, four feet on the 40 x 41 block (`PEG_PITCH=40 PITCH_Y=41 python3 lipo_smc_9000_mount.py`); it covers the same three columns and the cup has nothing to plug into, so it is the alternative to the Omni, not an addition |

Assembly order that the slide channels impose: Jetson tray before the small-board plate
is not needed (they share no space), but the plinth slides on from the rear (toward the
mast) and the mast slides on from the left, so plinth first, then mast; the Jetson tray
slides on from the front under the plate with the board out, the board clips in from
below afterwards.

Rear body posts: the cup at y 20.5 puts the brick's left edge at y 49.5, through the left
rear post at (-139, 41). The posts hold a body this car does not run (the mast replaced
the top plate), so the v2 model takes the rear pair off (`POSTS=front`, default for v2;
the cutouts stay, the plate still clears the mounts they sit in). `POSTS=both` shows the
conflict: 808 mm3 brick and 110 mm3 cup against the post. The alternative that keeps the
posts is `OUTLET_Y=-20.5`: if the pack's AC outlet is 20.5 mm right of the mid-edge (or
the pack is turned over to put it there), the cup centres on y 0, its feet drop to B8+C8
at pitch 41 and the brick clears both posts by 4 mm; the script picks the row pair from
`OUTLET_Y` (must be a multiple of 20.5).

Hole table, all 28, with what a free hole can still take (a from-below piece for a mount
above, a from-above piece for a mount below; "hits" are the intersection volumes in mm3
the piece would have with the placed solids):

| hole | x | y | used by | free for a mount above | free for a mount below |
| --- | --- | --- | --- | --- | --- |
| A1 | 115 | 61.5 | camera mast | | |
| B1 | 115 | 20.5 | camera mast | | |
| C1 | 115 | -20.5 | free, cable pass-through for the camera lead | yes | no: small-board plate 356, hub 750 |
| D1 | 115 | -61.5 | small-board plate | | |
| A2 | 75 | 61.5 | lidar plinth | | |
| B2 | 75 | 20.5 | free | yes (short things only; the plinth wall is 10 mm away) | no: plinth 846 |
| C2 | 75 | -20.5 | free | yes | no: small-board plate 523, hub 876 |
| D2 | 75 | -61.5 | small-board plate | | |
| A3 | 35 | 61.5 | lidar plinth | | |
| B3 | 35 | 20.5 | free, cable pass-through for the lidar lead | yes | no: plinth 846 |
| C3 | 35 | -20.5 | free | no: Jetson tray 1272 | no: small-board plate 276, hub 259 |
| D3 | 35 | -61.5 | free | no: Jetson tray 541 | no: small-board plate 842, hub 1475 |
| A4 | -5 | 61.5 | free (under the cradle border) | yes, but the cradle covers it | no: cradle 970, pack 3275 |
| B4 | -5 | 20.5 | free (under the cradle) | covered | no: cradle 612, pack 3293 |
| C4 | -5 | -20.5 | free (under the cradle) | no: Jetson tray 2046 | no: cradle 970 |
| D4 | -5 | -61.5 | Jetson tray | | |
| A5 | -45 | 61.5 | free (under the cradle) | covered | no: cradle 1823 |
| B5 | -45 | 20.5 | Omni cradle | | |
| C5 | -45 | -20.5 | free (under the cradle) | no: Jetson tray 2046 | no: cradle 1823 |
| D5 | -45 | -61.5 | Jetson tray | | |
| A6 | -85 | 61.5 | free (under the cradle) | covered | no: cradle 1823 |
| B6 | -85 | 20.5 | Omni cradle | | |
| C6 | -85 | -20.5 | free (under the cradle) | no: Jetson tray 1214 | no: cradle 1823 |
| D6 | -85 | -61.5 | free | no: Jetson tray 1703 | yes (the only free hole behind the cradle on the right; over the motor, 26 mm of depth at most) |
| A8 | -185 | 61.5 | Lite-On cup | | |
| B8 | -185 | 20.5 | free (under the cup) | covered | no: cup 2052, brick 5692 |
| C8 | -185 | -20.5 | Lite-On cup | | |
| D8 | -185 | -61.5 | free | yes | yes (behind the rear tower, over nothing; a single-foot mount at most, 14 mm from the rear tyre's inner face at full bump) |

Checks that passed: no hole claimed twice, every foot's peg centre within 2.5 mm of its
hole centre (the play in the channels is 1.25 mm), every claimed hole is in the plate
model, the plate is one solid, every mount part one solid (new check; it caught the cup's
snap fingers being cut off their floor by the foot recess at `FEET_X_OFFSET` 10.25 and
13.5, fixed by `finger_x()` in `liteon_45w_brick_mount.py`, see its MD; the STL layout's
cup had the same fault and the same fix, its numbers do not change), the 8 mm webs.

## 16. Interference, clearances and cables on v2

Pairwise intersection over all 60 solids (6 mounts as 7 parts, 12 feet, 12 clipless
pieces, 6 envelopes, 17 chassis keep-outs, plate and 4 clamp blocks), same exclusions as
section 7: at UNDER_CLEARANCE 45, zero interference between any mount, foot, piece,
envelope, the chassis and the plate; the only non-zero rows are the plate's own front
corners against the front tyres at full lock plus full bump (0.7 mm3 each side, the plate
outline's problem, section 3). Closest approaches, in order:

| pair | gap |
| --- | --- |
| plinth floor bar (x 94.25) to mast floor bar (94.75) | 0.5 mm |
| small-board plate's front end (x 134.25, at y -91.5) to the front right tyre at full lock + full bump (the rear edge of the swept tyre passes x 135.0 at y -91.5) | 0.75 mm in the coarse tyre model; measure it, section 18 |
| cradle outer wall (y -44.1) to the Jetson's D4/D5 pieces (-45.0) | 0.9 mm |
| mast floor bar's end (y 0.75) to the row-C flange zone (nothing placed there) | - |
| Jetson tray's inboard wall (y -18.5) to the cradle's B5/B6 flanges under the plate (y 4.0) | 22.5 mm |
| Jetson module bottom to the tub rim | 1.83 mm at 45 |
| Jetson module (y -28.5 at its inboard edge) to the battery bay (y 3) | 31.5 mm in plan; the tray floor passes over the bay with 30 mm of air |
| Jetson module (x -68) to the motor can (x -102) | 34 mm |
| lidar scan plane (44.8) to the cup fingers (34.3) | 10.5 mm |
| cradle front (x 2.1) to the Jetson's D4 piece on the plate top (11.5 at its near edge; the piece is at x -21.5..11.5, y -78..-45) | 0.9 mm (the same pair as above, in x it is inside the cradle's footprint) |

Under-board clearance, mount bottom minus chassis top (`clearance_report()`):

| mount | depth below plate top | UNDER_CLEARANCE 45 | 40 | 35 |
| --- | --- | --- | --- | --- |
| Jetson tray (D4+D5) | 46.35 | +1.83 over the tub rim | -3.17 | -8.17 |
| cable tidy (attempt, C1+C2) | 26.68 | +6.5 over the steering linkage | +1.5 | -3.5 |
| VESC tray (attempt, C2+C3) | 38.98 | -3.5 over the steering linkage | -8.5 | -13.5 |

So the Jetson needs the full 45 mm (as on the STL plate: its 46.35 mm is the kit's own
height, 8.6 floor + 6 standoff + 1.57 board + 27 module and fan + 3.175 plate). At 40 the
run (`TAG=_uc40`) also shows the plate's own rear clamp in the spur cover model (1251 mm3)
and the tyres deeper into the plate corners (32.6 mm3), as with the STL plate; those are
the chassis estimates, not the layout. At 35 nothing deeper than 31 mm hangs anywhere.

Above the board: mast head 148.9 (lens), lidar top 56.3, scan plane 44.8, cup fingers
34.3, brick 32.0, pack 31.0, PCA9685 headers 23.6, cradle posts 18.0. In this placement
the mast's four legs were 40 to 80 mm from the lidar's axis, bearing 5 to 30 degrees right
of straight ahead: four 6 mm obstacles, about 5 degrees of arc each, in the forward-right
quadrant (in the STL layout they were in the forward-left one at the same distance). That
is the sector a racing car needs most, which is why section 19 swaps the two: the legs are
then in the rear half only.

Cables. Camera: the Gemini 335's USB-C leaves the head on the -y (right) end 137 mm up,
comes straight down the mast's right-hand rear leg (leg base at (98, 3)), and drops through
hole C1 (115, -20.5), which is free on both faces and 20 mm from the leg; under the plate
it runs aft along the plate's right edge to the Jetson's USB-A at the tray's outboard edge
(x -72..-25, y -100). Lidar: the C1's XH lead leaves the plinth's rear notch at (24, 61.5),
crosses 40 mm to hole B3 (35, 20.5), which is free on both faces (the plinth wall is 10 mm
off its edge), and goes down to a USB adapter zip-tied under the plate at the B3/C3 web,
then joins the right-edge run. No cable slots were cut in the plate because both leads
have a free 28.5 mm hole within 40 mm; `CABLE_SLOTS` is there if the real cable ends do
not bend that way. Power: the Omni's DC barrel and USB-C ports are on its +-y edges; the
brick's 19 V barrel lead leaves the cup's cable notch at x -219.5 and its 1.2 m has to be
coiled: with no tidy on the board the coil goes under the plate's rear overhang between
the rear tower and the cup's flanges (x -168..-160, D8 is free) or, better, through the
zip-tie holes along the right edge at x -145 and -105 (three ties through pairs of 6 mm
holes hold a 60 mm coil against the plate underside) and forward to the Jetson's DC jack
(tray outboard edge, x -70). The PCA9685 and hub sit above the plate at the right front;
their USB lead to the hub and the hub's lead to the Jetson go down through D3 (free, 20 mm
from the plate's rear end) to the same right-edge run; the servo lead from the PCA9685 goes
down D3 too and forward to the steering servo (x 122..142). The VESC (not on the board)
sits in the tub's right bay, its USB to the hub up through D6 or D8. With the cable tidy
not fitted, the zip-tie holes are the strain relief: every lead that crosses a plate edge
gets a tie through the nearest pair.

## 17. What does not fit on v2, with the numbers

Cable tidy (100 mm, four posts; `L=100 N_POST=4`, `out/cable_tidy_100.*`). The right edge
row (D) is the small-board plate's at columns 1-2 and the Jetson's at 4-5; D3 alone is one
column. Row C below at C1+C2 (`ATTEMPT=tidy`, rail x 45..145, y -40.75..-0.25, lip
outboard) clears the linkage by 6.5 mm and every mount, but its two pieces are on the plate
top, at y -37..-4, and the small-board plate's inboard edge is at y -31.5: 356 + 523 mm3
plate and 750 + 876 mm3 hub envelope against the C1 and C2 pieces, plus 72 mm3 per foot.
That is the 40.5 + 60 > 80 problem: a 60 mm wide part on one row and a piece on the next
row cannot share a face pair at 41 mm pitch (needs 46.5). Moving the small-board plate
away frees the tidy, and vice versa; the tidy above at D5+D6 hits the cradle by 4.85 mm
(rail y -40.75 vs cradle -44.1) and its under-plate flanges hit the Jetson tray; at C2+C3
the rail overlaps the Jetson tray by 23 mm in x. The 150 mm tidy needs a row of its own.
If the PCA9685 and hub move into the tub (they are 15 mm tall and the tub's right bay ahead
of the VESC is empty), the tidy goes on C1+C2 as attempted, at zero interference.

VESC tray (`_under`, 38.98 deep, 102 x 99 plus 12 mm tabs). The only place under the
plate where 39 mm of depth clears the servo, the linkage, the battery and the motor is
the right bay between x -102 and 122, and the Jetson tray (46.35 deep, the deepest thing
on the car) is there, on the only holes it can hang from (D4+D5; D5+D6 puts its heatsink
6 mm into the motor can model, D2+D3 puts the small-board plate's flanges through it, and
columns 2-3 are also where the linkage starts at x 128, so a VESC on C2+C3 is 3.5 mm into
the linkage model, `ATTEMPT=vesc`, and 9.5 cm3 into the Jetson). Section 9.4 stands: the
FSESC replaces the stock ESC and belongs in the tub's ESC bay on a tub bracket, under the
plate at exactly the depth the plate cannot reach.

LiPo frame: see the table in section 15, it is the alternative to the Omni.

What would fit all nine: (a) the thin-flange from-above piece of section 9.3 (a 2 mm
flange instead of 12.85 on the plate top, so above-board mounts can pass over holes used
from below: that alone puts the tidy on C1+C2 and frees columns 4-6 row D under the
cradle); (b) a 46.5 mm row pitch, which does not fit four rows in 177.5 (three rows do, at
the STL's density); (c) the small boards off the plate.

## 18. Measure on the car before cutting v2

In addition to section 11 (all of which still applies: plate underside to tub rim,
battery hold-down, spur cover, servo horn, body-post spacing, outlet position, motor
position, brick size, clamp height):

1. Front tyre inner face at full lock with the suspension bottomed, at x 134 (the
   small-board plate's front end) and y -91.5: the plan-view figure here was 0.75 mm; the
   solid model's closest approach is 14.4 mm (section 19). With the plate turned
   180, `PLATE_OFFSET=8.5` moves it 2.25 mm FORWARD (10.75 is the most aft the rim rule
   allows); the other way out if the tyre is closer than modelled is a shorter plate
   (the PCA9685 note in section 19).
2. Motor can: front face x and its top height (the free hole D6 is over it; the Jetson's
   heatsink is 34 mm ahead of it in the model).
3. Rear body posts: confirm they can come off, or measure the outlet offset for the
   `OUTLET_Y=-20.5` alternative.
4. Right side of the tub at y -50 to -105, x -80 to 30: the Jetson tray hangs 15.75 mm
   outside the plate there, 18 mm above the tub rim; check nothing on the car (rear
   tyre at full bump and lock is 40 mm behind it, the tub's side is 55 mm inboard) sweeps
   through x -78..28, y -105..-88, z -3 to -22 below the plate top.
5. Forum chassis numbers: the threads found (rctalk.com "Good servos for Slash 2WD",
   "My Slash 2wd build 2.0", "Luke's MM Slash 2wd build", rc10talk.com "Another Traxxas
   Mid-Motor 2wd build") describe the layout but give no millimetre dimensions for the tub
   depth, the motor position or the servo bulkhead, so the manual and the estimates in
   section 3 stand. One manufacturer number was added: the Traxxas 58034 product page
   lists the battery compartment as 165 x 50 x 23 mm (25 mm with the expansion), so the
   battery keep-out is now 165 long (x -120..45; the front end and the side are still
   estimates).

## 19. Final set on v2

`PLATE=v2 python3 board_layout.py` builds all of this (regenerating `baseplate_v2`, the
Jetson `_under` tray, `cable_tidy_edge`, `vesc_tub_bracket`, the `_v2` plinth, mast,
small-board plate and cup) and writes `out/board_layout_v2_*`; `UNDER_CLEARANCE=40
TAG=_uc40` is the 40 mm run, `PITCH_Y=40 TAG=_p40` and `POSTS=both TAG=_posts` the section
13 / 15 variants (their conclusions are unchanged, see the interference table). The two
mounts that had no clipless footprint left (section 17) are on the car without clipless
feet: the cable tidy hangs off the plate's right edge on two M5 screws through the zip-tie
holes (`cable_tidy_mount.py`, `EDGE=1`, CABLE_TIDY_MOUNT.md), and the VESC tray sits on a
clamp-on saddle over the tub's right side rail (`vesc_tub_bracket_mount.py`,
VESC_TUB_BRACKET_MOUNT.md). The old `out/board_layout_v2_tidy_*` and `_vesc_*` files are
the section 17 attempts and predate this section.

### Lidar in front, mast behind it (supersedes the mast / plinth rows of section 15)

Sections 15-16 had the camera mast on A1+B1 (x 115) IN FRONT of the RPLIDAR C1 on A2+A3
(x 55): the mast's four legs crossed the lidar's scan plane (44.8 above the plate) in the
forward-right quadrant, 5 to 30 degrees off dead ahead, which is the sector a racing car
needs most. Fixed by swapping them: the plinth goes to the front on B1+B2 (feet along
x at 115 and 75, centre (95, 20.5), connector notch aft) and the mast behind it on
A3+B3 (feet across on rows A and B at column 3, centre (35, 41)), head looking forward
over the lidar. Nothing on the board now crosses the scan plane in the forward half; the
only things that cross it anywhere are the mast legs, in the rear half (sectors below).

Why A3+B3 and not the centred pairs the owner asked for first (`ATTEMPT=mastB3C3` /
`mastB4C4` reproduce them, outputs `out/board_layout_v2_mastB3C3_*`, `_mastB4C4_*`):

- B3+C3 (centre (35, 0)): an above-board mount uses clipless pieces from BELOW, and the
  C3 piece's 33 x 33 x 12.85 flange under the plate (x 18.5..51.5, y -37..-4) is where the
  Jetson tray's front-inboard corner hangs (tray x -78.5..28.5, y -104.5..-18.5): 1272 mm3
  piece and 413 mm3 foot into the tray, exactly the "C3: no, Jetson tray 1272" row of the
  section 15 hole table. The mast bar (y -40.25..40.25 when centred) also lands on the
  small-board plate's inboard 8.75 mm (360 mm3 plate, 481 hub, 114 foot). Moving the Jetson
  tray is not an option (D4+D5 are the only holes it can hang from, section 17), and the
  small-board plate cannot go further aft on D1+D2 (rim rule, `PLATE_OFFSET` max 10.75) or
  forward (tyre).
- B4+C4 (centre (-5, 0)): the mast bar (x -25.25..15.25) is 27 mm into the Omni cradle
  (front at x 2.1): 3006 mm3 mast x pack, 603 mast x cradle, 1822 + 1700 feet x cradle, and
  the C4 piece is 2046 into the Jetson tray as well.
- A3+B3: pieces under the plate at x 18.5..51.5, y 4..78, over the battery bay model
  (26 mm above its hold-down) and the tub's left wall; nothing else under there. Mast bar
  x 14.75..55.25, y 0.75..81 (legs to y 0 / 82, 6.75 inside the plate edge), 0.25 mm behind
  the plinth's bar, 12.65 ahead of the cradle, 31.6 from the small-board plate. The camera
  ends up 41 mm left of the centreline, where it already was in section 15, and 20.5 mm
  left of the lidar axis. B3+C3 would have centred it; that costs a Jetson tray 10 mm
  shorter or the thin-flange piece of section 9.3.

### The nine mounts

| mount | how it attaches | where | numbers |
| --- | --- | --- | --- |
| RPLIDAR C1 plinth | clipless, B1+B2, above | (95, 20.5), rot 0 | `BAR_END` back to its default 5 (nothing ahead of the bar now): floor bar x 55.5..135.75, y 0.25..40.75; body x 63.95..126.05, y -10.55..51.55 (its 3 mm wall stands on 4.3 mm of the A1, A2, C1 and C2 holes); lidar base x 67.2..122.8, y -7.3..48.3, head R 22, top 56.3, scan plane 44.8; connector housing at the rear, x 61.2..67.2; 41 mm behind the front body posts (25 tall, under the scan plane), 24 mm behind the front cutouts, B1's flange under the plate (x 98.5..131.5, 12.85 deep) passes 17 mm above the steering-linkage model, whose rear 3.5 mm is under it in plan, and ends 29 mm behind the front clamp block; 42 mm from the front-left tyre at full lock and bump (the lidar 53) |
| camera mast (Gemini 335) | clipless, A3+B3, above | (35, 41), rot 0 | `PITCH_Y` 41, `CAM_HEIGHT` 150 unchanged; bar x 14.75..55.25, y 0.75..81; legs from (17.75 / 52.25, 3 / 79) up to (25 / 45, 21.5 / 60.5) at 104; drum axis 112.4, head x 21..54, camera x 24..54, y -21.7 (USB-C plug stub) .. 85.7, z 136.4..161.4; lens (54, 41, 148.9) at 0 pitch, 41 mm behind the lidar axis and 92.6 above its top; legs 8.7 from the plinth body, 9.0 from the lidar envelope |
| Omni 20+ cradle | clipless, B5+B6, above | (-65, 20.5), rot 0 | unchanged (section 15) |
| Lite-On 45 W cup | clipless, A8+C8, above | (-171.5, 20.5), rot 180 | unchanged |
| small-board plate (PCA9685 + hub) | clipless, D1+D2, above | (95, -61.5), rot 180 | unchanged: `PLATE_OFFSET` 10.75, x 34.25..134.25, y -91.5..-31.5 (a larger offset moves it AFT, 10.75 is the rim-rule maximum; section 18 item 1 had the direction backwards); 21 mm from the plinth body; closest approach to the front-right tyre's lock + bump sweep 14.4 mm in the solid model (the 0.75 of section 16 was a plan-view figure against the sweep's footprint at rim level); see the note on the PCA9685 below |
| Jetson Orin Nano tray | clipless, D4+D5, below | (-25, -61.5), rot 180 | unchanged; tray x -78.5..28.5, y -104.5..-18.5, 46.35 deep |
| cable tidy, edge variant | two M5 x 16 + nyloc through the zip holes at (-105, -82) and (-145, -82); no clipless | rail body x -112..-12, y -125.6..-89.0 (inboard face 0.25 off the plate edge at -88.75), z 0..23.5 above the plate top; pads on the plate top to y -76; ledge x -152..-112 along the edge | 100 mm, four posts, 36 wide; 27.8 cm3. Rail top 23.5 (limit 24), 21.3 under the lidar scan plane |
| VESC FSESC 6.7 on the tub saddle | clamp-on saddle on the tub's right rail (y -50, RAIL_W 12, RAIL_H 18), two 25 mm straps, four 3.2 mm holes for the tub's ESC bosses; no clipless | tray x 25.25..124.25, y -3.5..-105.5 (tabs to +8.5 / -117.5), turned 90 (XT90 inboard, phase leads outboard); floor 6 below the rim (`SADDLE_DROP` 6), case 3..27.5 above the rim, fingers 29.8; legs 18 below the rim | 58 cm3; margins: 0.25 to the Jetson tray, 2.35 under the D1/D2 flanges and now also under the plinth's B2 flange (its inboard tab reaches y 8.5, the flange starts at y 4), 3.75 to the linkage, 0.5 to the front tyre's inner face; 2.25 mm INTO the steering servo estimate (below) |
| LiPo SMC 9000 frame | clipless, B5+C5+B6+C6, above (the Omni alternative) | (-65, 0), rot 0 | as section 15: the pack stays in the chassis tray; the frame replaces the cradle if the Omni is not carried |

All nine are placed. What is not zero is one row in the model, and it is against an
estimate: the saddle's front 2.25 mm (x 122..124.25, y -3.5..-20, its floor at rim -6..-2)
is inside the steering servo model (x 122..142, y +-20, top at rim +2): 247.5 mm3. The
bay between the Jetson tray's end (x 28.5, exact) and the servo (122, estimate) is 93.5 mm
and the VESC tray is 99 mm long with its case 92; the other way out, the case over the
Jetson tray's corner posts, needs the case top 0.4 mm lower than the saddle's 3 mm ceiling
allows. So the 2.25 mm is taken from the estimate with the most slack: the servo's rear
face at x 122 came from "standard servo lying across the front of the tub" and its top at
rim +2 from nothing better; either the servo 2.25 mm further forward or its top 4 mm below
the rim makes the row zero. If the servo is where the model says, `SADDLE_X0=23` moves the
saddle 2.25 mm aft and the case then overlaps the Jetson tray's posts by 0.4 mm in z over
2.25 mm in x (about 20 mm3, exact geometry), which is the worse trade.

Why the tidy is where it is: the rail body must be above the plate top (the Jetson tray
hangs under the whole right edge from x -78.5 to 28.5, to y -104.5), ahead of the rear
tyre's bump sweep (x < -110 for anything at z 0..24 outboard of y -106) and behind the
small-board plate, which overhangs the edge by 2.75 mm above the plate from x 34.25 to
134.25. That leaves x -110..34 for the body, and the zip holes in that range are -105,
-65, -25 and 15: the last three have the Jetson tray's 8.6 mm floor directly under them
(a nut at (-65, -82) lands 3 mm from its standoff boss at (-71, -84)), so the second
screw goes to -145 and the rail carries a 40 mm ledge along the edge to reach it. The
ledge is 11.75 mm wide outboard of the edge, 5.75 mm inboard of the rear tyre's inner
face, and the tyre at full bump is 14 mm below the ledge there.

Assembly order: the plinth slides onto B1+B2 from the rear (its channel is open at -x, so
it starts 40 mm aft, over the mast's footprint) and goes on FIRST; the mast then slides
onto A3+B3 from the left edge (channel open at +y). To take the plinth off, the mast comes
off first. The Jetson tray slides on from the front under the plate with the board out,
as before.

### What the lidar sees: occluded sectors

The script slices every solid other than the lidar's own at the scan plane (44.8 above the
plate top) and reports each piece as a range, bearing and width from the lidar axis at
(95, 20.5); bearing is from dead ahead, positive to the LEFT (right-hand, as the board
frame), and the C1 datasheet's clockwise angle is 360 minus it. `camera_mast_mount
.scan_occlusion()` gives the same four legs analytically (leg centres, with a 0.75 x 6 mm
half-width so the 8.5 mm diagonal is covered), and that is the wider, safer set to mask:

| what | range (mm) | bearing (deg, +left) | width (deg) | sector to mask (deg) | C1 datasheet angle |
| --- | --- | --- | --- | --- | --- |
| mast front-right leg | 46.7 | -167.2 | 11.0 | -172.7 .. -161.7 | 161.7 .. 172.7 |
| mast rear-right leg | 75.1 | -172.1 | 6.9 | -175.6 .. -168.7 (merges with the above: -175.6 .. -161.7) | 168.7 .. 175.6 |
| mast front-left leg | 68.6 | 131.6 | 7.5 | 127.9 .. 135.4 | 224.6 .. 232.1 |
| mast rear-left leg | 90.4 | 145.4 | 5.7 | 142.6 .. 148.3 | 211.7 .. 217.4 |
| everything else on the board | - | - | - | none: lidar top 56.3, next tallest the cup fingers at 34.3, brick 32, pack 31, tidy caps 23.5, PCA headers 23.6, front body posts 25, tyres at full bump +15.8 | |

Three sectors, 27.2 degrees in total, all between 128 and 198 degrees, i.e. within 52
degrees either side of dead astern: 4 to 18 degrees right of the tail (both right legs,
one behind the other) and 32 to 52 degrees left of it (the two left legs, not merged).
The forward half, -90 to +90, is clear; the nearest occlusion starts 38 degrees behind
the left beam. The sliced-solid check gives the same bearings with slightly narrower widths
(the legs are 6 mm square, not 8.5 round): (-174.6, -169.4), (-171.4, -162.6), (128.1,
135.2), (142.7, 148.0). In the driver, mask the analytic sectors, or drop returns under
0.10 m (the C1's minimum range is 0.05 m and the legs are at 0.047 to 0.090 m; the
front-right leg at 46.7 mm is inside the datasheet minimum and will read as noise anyway).
Everything the legs hide is behind the car within 9 cm of the lidar, which is the car
itself.

### Camera line of sight over the lidar

Gemini 335 field of view: depth H 90 x V 65 degrees, RGB H 86 x V 55 (orbbec.com product
page, "Depth FOV H90° V65°", read 2026-09-05; the Gemini 330 series datasheet v1.1 gives
the same). A rectilinear camera puts a point at image row f * z'/x' (x' the depth along the
optical axis), so the lateral offset of the lidar (20.5 mm right of the lens) does not
matter; the lidar enters the bottom of the image when its highest edge, as seen from the
lens, reaches the bottom ray at -(pitch + 32.5) degrees. The highest edge from the lens
is the head cylinder's rear top rim, x 73 (axis 95 minus R 22), z 56.3. The lens moves as
the head pitches about the drum axis (`camera_mast_mount.lens_xyz`), which the table
includes (`camera_mast_mount.lidar_in_view()`, called by `board_layout.py` with the real
relative position; the JSON has the same rows):

| head pitch (deg, nose down) | lens (x, z) above the plate | lidar edge from the lens (dx, dz) | lidar edge elevation (deg) | image bottom ray (deg) | lidar below the image bottom (deg) |
| --- | --- | --- | --- | --- | --- |
| 0 | (54.0, 148.9) | (19.0, -92.6) | -78.4 | -32.5 | 45.9 |
| -5 | (57.1, 147.1) | (15.9, -90.8) | -80.1 | -37.5 | 42.6 |
| -10 | (60.0, 145.0) | (13.0, -88.7) | -81.7 | -42.5 | 39.2 |
| -15 | (62.8, 142.7) | (10.2, -86.4) | -83.3 | -47.5 | 35.8 |
| -20 | (65.3, 140.2) | (7.7, -83.9) | -84.8 | -52.5 | 32.3 |

The lidar top would enter the depth image at -60.6 degrees of pitch (the RGB image, V
55, at -65.8), three times the mast's -20 limit; at -20 the lidar is still 32 degrees below
the bottom of the frame. The lidar is almost directly under the lens (19 mm ahead, 93 mm
down), which is the point of putting the mast right behind it. So `CAM_HEIGHT` stays at
150 and nothing is paid in stiffness. (For the record, if it ever had to go up: the legs
are a pinned truss whose tip deflection goes with length cubed, so +10 mm on the 99 mm legs
would cost about 30 % more deflection at the head, from 0.2 to 0.26 mm fore-aft at 5 g,
and about 15 % on the first mode.) What the camera does see at -20 is the plate from
x 173 forward, i.e. the front body posts (at 180, 25 tall) in the bottom corners of the
frame, and its own front-left tyre at full bump; neither is on the board.

### Hole table

All 28, with what a free hole can still take (a from-below piece for a mount above, a
from-above piece for a mount below; "hits" are the intersection volumes in mm3 the piece
would have with the placed solids, from the same model):

| hole | x | y | used by | free for a mount above | free for a mount below |
| --- | --- | --- | --- | --- | --- |
| A1 | 115 | 61.5 | free; the plinth's wall stands on its inboard 4.3 mm | no: a foot's 34 mm flange would meet the plinth wall (the piece itself has no hits) | no: plinth 846 |
| B1 | 115 | 20.5 | lidar plinth | | |
| C1 | 115 | -20.5 | free; plinth wall on its inboard 4.3 mm, small-board plate over its outboard 3.25 | no: plinth wall and small-board plate on the hole (the piece itself has no hits) | no: plinth 846, small-board plate 356, hub 750 |
| D1 | 115 | -61.5 | small-board plate | | |
| A2 | 75 | 61.5 | free; plinth wall on its inboard 4.3 mm (alternative lidar-lead pass-through, left side) | no: as A1 | no: plinth 846 |
| B2 | 75 | 20.5 | lidar plinth | | |
| C2 | 75 | -20.5 | free, cable pass-through for the lidar lead: 21 of its 28.5 mm open between the plinth wall (4.3) and the small-board plate's edge (3.25) | no: as C1 | no: plinth 846, small-board plate 523, hub 876 |
| D2 | 75 | -61.5 | small-board plate | | |
| A3 | 35 | 61.5 | camera mast | | |
| B3 | 35 | 20.5 | camera mast | | |
| C3 | 35 | -20.5 | free, cable pass-through for the camera lead: open except a 15 x 3.25 corner under the small-board plate; 9 mm from the mast's right rear leg | no: Jetson tray 1272 (this is what rules out the centred mast) | no: small-board plate 276, hub 259 |
| D3 | 35 | -61.5 | free, pass-through for the hub and PCA9685 leads (13.5 mm strip open at x 20.75..34.25, the rest under the small-board plate) | no: Jetson tray 541 | no: small-board plate 842, hub 1475 |
| A4 | -5 | 61.5 | free (under the cradle border) | yes, but the cradle covers it | no: cradle 970, pack 3275 |
| B4 | -5 | 20.5 | free (under the cradle) | covered | no: cradle 612, pack 3293 |
| C4 | -5 | -20.5 | free (under the cradle) | no: Jetson tray 2046 | no: cradle 970, pack 3275 |
| D4 | -5 | -61.5 | Jetson tray | | |
| A5 | -45 | 61.5 | free (under the cradle) | covered | no: cradle 1823, pack 5692 |
| B5 | -45 | 20.5 | Omni cradle | | |
| C5 | -45 | -20.5 | free (under the cradle) | no: Jetson tray 2046 | no: cradle 1823, pack 5692 |
| D5 | -45 | -61.5 | Jetson tray | | |
| A6 | -85 | 61.5 | free (under the cradle) | covered | no: cradle 1823, pack 5692 |
| B6 | -85 | 20.5 | Omni cradle | | |
| C6 | -85 | -20.5 | free (under the cradle) | no: Jetson tray 1214 | no: cradle 1823, pack 5692 |
| D6 | -85 | -61.5 | free; the VESC's USB comes up through it | no: Jetson tray 1703 | 25 mm3 against the tidy's pad (the pad ends 0.25 short of the hole edge; a from-above flange reaches 2 mm past it), otherwise nothing under it but the saddle's rear end |
| A8 | -185 | 61.5 | Lite-On cup | | |
| B8 | -185 | 20.5 | free (under the cup) | covered | no: cup 2052, brick 5692 |
| C8 | -185 | -20.5 | Lite-On cup | | |
| D8 | -185 | -61.5 | free | yes | yes (behind the rear tower, over nothing; a single-foot mount at most, 14 mm from the rear tyre's inner face at full bump) |

Checks that passed: no hole claimed twice, every foot's peg centre within 2.5 mm of its
hole centre, every claimed hole in the plate model, the plate one solid, every mount part
one solid, the 8 mm webs.

### Interference table

Pairwise intersections over all 65 solids (8 mounts as 9 parts, 12 feet, 12 clipless
pieces, 9 envelopes including the tidy's screws and nuts, 18 chassis keep-outs, plate and
4 clamp blocks), same exclusions as section 7. Empty cells are zero.

| pair | UNDER_CLEARANCE 45 | 40 | 35 |
| --- | --- | --- | --- |
| VESC saddle x steering servo (estimate) | 247.5 | 247.5 | 247.5 |
| plate front corners x front tyres at full lock + bump (each side; the plate outline, section 3) | 0.7 | 32.6 | 70 |
| VESC case x small-board plate's D1 / D2 pieces (12.85 mm flanges under the plate, bottom at plate -16) | - | 257 / 381 (+ 16 saddle x D1) | 2542 / 3897 (saddle) |
| VESC saddle x lidar plinth's B2 piece (flange y 4..37 over the saddle's inboard tab, which reaches y 8.5) | - | - | 455 |
| VESC saddle x Jetson tray | - | - | 96.9 |
| Jetson kit x tub rim (section 16, unchanged) | - | 817.9 | more |
| plate rear clamp x spur cover model (section 3, unchanged) | - | 1251 | more |
| everything else: every mount, foot, piece, envelope; the mast against the plinth, the lidar, the cradle, the small-board plate and the Jetson's pieces; the plinth against the front posts, cutouts, clamp block, tyres, small-board plate and VESC; the tidy against the rear tyre; the saddle against the tub, motor, battery bay, linkage, front tyre, plate underside, Jetson tray and kit | 0 | 0 | 0 |

`PITCH_Y=40 TAG=_p40`: as in section 13, the cradle hits the Jetson's D4 / D5 pieces by
289 and 145 mm3 and nothing else changes (the mast and plinth rows are the same three).
`POSTS=both TAG=_posts`: as in section 15, 808 mm3 brick and 110 mm3 cup against the left
rear post, nothing else.

The saddle is fixed to the tub, so the plate and its mounts come down toward it as
UNDER_CLEARANCE shrinks (`tub_report()` in the script): plate underside margin 15.2 /
10.2 / 5.2 mm at 45 / 40 / 35; the D1/D2 and B2 flanges are 2.35 mm above the fingers at
45 and 2.65 into them at 40 (B2 only at 35: its overlap with the saddle tab in plan is a
4.5 mm strip). As in section 16, the layout is only consistent at 45 (the Jetson is 3.17
into the rim at 40), so 45 is the number to confirm first.

Closest approaches at 45, in order: mast bar to plinth bar 0.25 mm (mast bar's -y stop
end at x 55.25, plinth bar's open rear end at 55.5, both at floor level, 7 and 10 mm tall;
both parts float +-2 mm on their pegs plus 1.75 of channel play, so they can touch, which
costs nothing; `S_STOP + 4.0` in the mast script is the 4 mm stop wall, 3 would give 1.25);
VESC case rear face to the Jetson tray's end 0.25 (x 28.75 vs 28.5; in z the case top at
plate -20.7 is 0.4 above the posts' bottom at -21.1, so the 0.25 is what keeps them apart);
saddle outboard edge to the front tyre's inner face 0.5 (y -105.5 vs -106, the unsteered
tyre at full bump; the locked tyre's rear edge is 3 mm further away at that corner); cradle
to the Jetson pieces 0.9 (section 16); saddle fingers to the D1/D2 and B2 flanges 2.35;
saddle to the linkage 3.75; tidy pads to the D6 hole edge 0.25 in plan; tidy ledge to the
rear tyre's inner face 5.75 in y and the rail's rear end 6 mm ahead of where the bump
sweep would touch it, 5 to 10 mm of extra bump before contact; mast legs to the plinth body
8.7 and to the lidar envelope 9.0; mast bar to the cradle 12.65; small-board plate to the
front-right tyre sweep 14.4 (solid model); plinth's B1 flange to the linkage model 17.15;
plinth body to the small-board plate 20.95; mast's B3 flange to the Jetson tray 22.5; mast
pieces to the battery hold-down model 26; tidy nuts under the plate to the Jetson tray 26;
plinth to the front-left tyre sweep 41.7, to the front body posts 41.1.

Above-board heights: lidar 56.3, scan plane 44.8, cup fingers 34.3, brick 32, pack 31,
front body posts 25, PCA9685 headers 23.6, tidy caps 23.5, cradle posts 18; mast head
127..154, camera 136..161, lens 148.9.

Cables. Camera: the Gemini 335's USB-C leaves the head on the -y (right) end 137 mm up,
comes down the mast's right-hand rear leg (leg base at (17.75, 3)) and drops through hole
C3 (35, -20.5), which is 9 mm from the leg base and open apart from a 15 x 3.25 mm corner
under the small-board plate's inboard edge; under the plate the Jetson tray's end (x 28.5)
is under the hole's rear 7.75 mm and the VESC saddle's case is 15 mm below the plate under
the rest, so the lead comes down ahead of the tray, onto the case, and runs aft along the
plate's right edge to the Jetson's USB-A at the tray's outboard edge (x -72..-25, y -100).
Lidar: the C1's XH lead leaves the plinth's rear notch onto the bar's rear extension at
(55.5..64, 20.5), crosses about 35 mm aft-right to hole C2 (75, -20.5), of which 21 mm is
open between the plinth wall and the small-board plate's edge, and goes down to the XH-USB
adapter zip-tied to the plate underside at the C2/C3 web, in the 15 mm between the plate
and the saddle's case, then joins the same right-edge run. (A2 on the left is the
alternative pass-through, 4.3 mm under the plinth wall, with the adapter outboard of the
tub's left wall; its USB lead would then have to cross the car under the plate between the
battery bay's front end at x 45 and the saddle's rear at 25, so C2 is the shorter route.)
Hub and PCA9685 leads down D3 as before (13.5 mm strip open); the VESC's USB up through
D6; the brick's 19 V lead and the tidy as in the tidy paragraph above and CABLE_TIDY_MOUNT
.md. No cable slots were cut in the plate (`CABLE_SLOTS` in `baseplate_v2.py` if the real
cable ends do not bend that way).

PCA9685: once the FSESC drives the steering servo from its PPM/UART header (the servo
lead then runs 30 mm inside the tub instead of down D3 from the plate), the PCA9685 on the
small-board plate has nothing to drive and is probably redundant; the small-board plate
could shrink to a hub-only plate (about 60 x 50) on a single hole, which would free D1 or
D2 and 6.5 mm of the right edge. It is left in place here; nothing else depends on it.

### Measure on the car before cutting v2 and printing these two

Everything in sections 11 and 18 still applies, except that section 18 item 1 (the
small-board plate's front end vs the tyre) is now 14.4 mm in the solid model, not 0.75,
and `PLATE_OFFSET=8.5` would move it FORWARD, not aft. Added:

1. Tub right side rail: width across its top (`RAIL_W`, 12 assumed), lip height and
   whether there is a gap under the lip for a strap, depth available for the legs before
   a rib (`RAIL_H`, 18 assumed), and the rail top's height relative to the tub floor.
2. Stock ESC boss pattern and height on the tub floor of the right bay (`ESC_HOLES`,
   `ESC_STANDOFF`), and whether the receiver box sits in that bay (it is not in the
   chassis model; the saddle's floor is 6 mm below the rim there).
3. Steering servo: rear face x and top height relative to the rim (the 2.25 mm above),
   and the bellcrank's rearmost sweep and height (the linkage model is x 128..175, 15 mm
   above the rim; the plinth's B1 flange under the plate reaches x 131.5 and 12.85 mm
   down, so it is over the linkage model's rear 3.5 mm with 17 mm of air; a bellcrank or
   servo saver that stands more than 32 mm above the rim would touch it).
4. Rear tyre at full bump at x -112 to -100, y -106: the tidy's rear end and ledge are
   5.75 to 6 mm from the swept tyre in the model.
5. Zip holes (-105, -82) and (-145, -82): confirm nothing on the car is under them (the
   model has nothing; the nuts reach 8.2 mm below the plate top).
6. Front right tyre inner face at full bump, unsteered, at x 120..125: the saddle's
   outboard tab is at y -117.5 and its body at -105.5.
7. Lidar view: with the car on its wheels, spin the C1 and confirm the only returns under
   0.10 m are in the three rear sectors above (the front-left tyre at full bump tops out
   at +15.8 above the plate, 29 mm under the scan plane, and the front body posts at 25).
