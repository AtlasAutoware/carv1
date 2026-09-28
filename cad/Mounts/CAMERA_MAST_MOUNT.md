# Camera mast for the Orbbec Gemini 335 on the clipless mounting system

Replaces the wooden top plate: the camera stands on its own mast at the front of the car.

Files in `out/`: `camera_mast.step/.stl` (print one; for Baseplate v2 print
`camera_mast_v2.stl`, the same mast at the 41 mm row pitch, written by `board_layout.py`),
`camera_mast_head.step/.stl` (print one, the pitching saddle), `clipless_foot.step/.stl` (print two, from `omni20_mount.py`),
`camera_mast_assembly.step` (mast, head, feet, two clipless pieces in a stub of baseplate,
grey envelope of the camera), `camera_mast.png` (mast and head, shaded, two views),
`camera_mast_head.png`, `camera_mast_assembly.png`, `camera_mast_lines.png` and
`camera_mast_assembly_lines.png` (hidden-line). Source: `camera_mast_mount.py`, importing
`clipless.py`, `render.py`, helpers from `jetson_orin_nano_mount.py` and the slide channel
from `rplidar_c1_mount.py`.

## Component dimensions and where they come from

Orbbec Gemini 335 (model G40155-170). Product page
https://www.orbbec.com/products/stereo-vision-camera/gemini-335/ and store page
https://store.orbbec.com/products/gemini-335. Datasheet: "Gemini 330 Series Datasheet
v1.1", linked from https://www.orbbec.com/docs/g330-orbbec-gemini-330-series-datasheet/,
read at https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2024/06/Orbbec-Gemini-330-Series-Datasheet-v1.1-2.pdf

| Item | Value | Source |
| --- | --- | --- |
| Body | 90 x 25 x 30 mm (width x height x depth); front-view drawing gives 89.46 +-0.30 wide, side view 25.00 +-0.30 tall and 30.00 +-0.30 deep | datasheet section 2 table and 3.2.1 drawings (pages 16-17) |
| Mass | 97 g | section 2 table |
| Tripod thread | 1 x 1/4-20 UNC in the underside, "max insertion depth 5 mm", max torque 4.0 N.m; centred across the width | section 2 table, bottom-view drawing |
| Second fixing | 2 x M3, "M3 thread mounting points, max insertion depth 3 mm", max torque 0.4 N.m, 45.00 +-0.30 mm apart, in the REAR face between the heatsink fins | section 2 table, rear-view drawing |
| Data/power | USB Type-C on one end face, with two M2 points 15.00 +-0.10 apart for a locking cable | side-view drawing |
| Depth FOV | H 90 x V 65 degrees (1280 x 800) | data streams table |

The task text said to check for two M4 threads: on the Gemini 335 they are M3. M4 is the
335L (124 x 29 x 27 mm, 133 g), a different camera.

Estimates, marked as such in the script: the 1/4-20 socket's fore-aft position
(`CAM_TRIPOD_FROM_REAR` = 11 mm, from the bottom-view raster; it is not dimensioned), the
rear M3 holes and the lens axis at mid height (`CAM_M3_Z`, `CAM_LENS_Z` = 12.5), and
which end carries the USB-C (`CAM_USB_SIDE` = -y). The head's 1/4-20 slot has 8 mm of
fore-aft slack and the drum slot 15 degrees of extra arc to absorb the socket-position
estimate; the M3 tab sets the camera's fore-aft position regardless.

## Geometry assumed: the mast stands BEHIND the lidar

Origin at the centre of the foot pair, +x forward, +y left, +z up. The RPLIDAR C1 sits
AHEAD of the mast: `LIDAR_X`, `LIDAR_Y` (env vars, default 60, -20.5) are the lidar axis
relative to the foot-pair centre, which is the Baseplate v2 layout of LAYOUT.md section 19
(mast on A3+B3 at (35, 41), plinth on B1+B2 at (95, 20.5): one column ahead, half a row to
the right). The lens is 19 mm ahead of the foot row, so the lidar axis is 41 mm ahead of
the lens and its top (56.3 above the plate) 92.6 mm below it. Physical margin: the mast's
front-most point is the floor bar at x 20.25, the plinth body's rear wall is at
`LIDAR_X - 31.05` = 28.95, so 8.7 mm (`margin_to_plinth_body`); the bars themselves come
0.25 mm apart in the layout.

Earlier versions had the lidar 120 mm BEHIND the lens (`LIDAR_TO_CAM`, keep-out cylinder R
60), which put the four legs in the lidar's forward sector, 15 to 20 degrees either side
of dead ahead. That is gone: with the mast behind the lidar the legs cross the scan plane
(44.8 above the plate, `LIDAR_SCAN_Z`) in the REAR half only. `scan_occlusion()` gives each
leg's range, bearing (from the lidar's +x, positive to the left; the C1 datasheet's
clockwise angle is 360 minus it) and width (0.75 x 6 mm half-width, covering the 8.5 mm
diagonal), `occluded_sectors()` merges them and flags anything reaching into -90..+90:

| leg | range (mm) | bearing (deg) | width (deg) | sector |
| --- | --- | --- | --- | --- |
| front-right | 46.7 | -167.2 | 11.0 | -172.7 .. -161.7 |
| rear-right | 75.1 | -172.1 | 6.9 | -175.6 .. -168.7 (merges with the above) |
| front-left | 68.6 | 131.6 | 7.5 | 127.9 .. 135.4 |
| rear-left | 90.4 | 145.4 | 5.7 | 142.6 .. 148.3 |

27 degrees in total, all within 52 degrees of dead astern; the forward half is clear
(`occlusion_in_forward_half: none`). Mask those three sectors in the driver or drop returns
under 0.10 m. `board_layout.py` re-derives the same sectors by slicing the placed solids
at the scan plane and agrees to 0.1 degree.

Line of sight over the lidar (`lidar_in_view()`): the Gemini 335's depth FOV is H 90 x
V 65 degrees (RGB H 86 x V 55; orbbec.com product page and the 330 series datasheet). A
rectilinear camera puts a point at image row f * z'/x', so only the fore-aft and vertical
offsets matter; the lidar's highest edge from the lens is the head cylinder's rear top rim
(x `LIDAR_X` - 22, z `LIDAR_TOP_Z` 56.3). With the lens moving as the head pitches
(`lens_xyz`):

| pitch | lidar below the image bottom (deg) |
| --- | --- |
| 0 | 45.9 |
| -5 | 42.6 |
| -10 | 39.2 |
| -15 | 35.8 |
| -20 | 32.3 |

The lidar top would enter the depth image at -60.6 degrees of pitch (the RGB image at
-65.8), three times the -20 the drum allows, so `CAM_HEIGHT` stays at 150 and no stiffness
is given up. (If it ever had to rise: leg deflection goes with length cubed, +10 mm is
about +30 % deflection and -15 % on the first mode.) The camera body itself is well above
the plane (underside at 137.5 mm, top at 162.5 mm).

## What the mast does

- Feet: two clipless feet ACROSS the car, `PITCH_Y` (env var, default 47.0; 41 on
  Baseplate v2, `out/camera_mast_v2.*`). Across, not along, so the mast takes one hole
  column and the lidar plinth can sit on the next one ahead of it. The
  floor bar is 40.5 (x) x 86 (y) x 7 mm and carries the same dovetail slide channel as the
  lidar plinth (`rplidar_c1_mount.slide_channel`), running along y and open on the +y
  side: feet into their pockets first, the mast slides on sideways over both flanges to
  the end stop, the two spring tongues snap up behind the +y flange, the 45 degree lips
  stop it lifting off. 1.75 mm of play between stop and bump plus the +-2 mm the pegs
  float. Set `PITCH_Y` to the measured pitch of the hole pair used.
- Legs: four 6 x 6 mm legs from the floor corners (x +-17.25, y +-38) converging to
  (x +-10, y +-19.5) at the drum yokes 99 mm up: a pyramid, triangulated in both planes.
  Leg half-angles 10.5 degrees laterally and 4.2 degrees fore-aft. Estimated tip
  deflection at 5 g on the 97 g camera plus 15 g head (PLA, E = 3 GPa, legs as a pinned
  truss): about 0.03 mm lateral and 0.2 mm fore-aft, first mode well above 50 Hz; leg
  buckling load is over 300 N against 6 N of load per leg. Legs are 5.7 to 11 degrees
  wide as seen by the lidar, in its rear half (table above).
- Drum: a horizontal half-cylinder shell, R 20 / R 16, 44 mm wide, axis along y at
  z = 112.4, closed at the ends by the yokes and open underneath. Its outer surface is a
  cylindrical rack: 5 degree teeth (1.75 mm pitch, 0.8 mm deep) over +-65 degrees. An arc
  slot 7.5 mm wide from -15 to +35 degrees (from vertical towards the front) lets the
  1/4-20 bolt through the shell.
- Head (saddle): 33 x 44 mm pad with the matching toothed concave underside (0.15 mm
  radial clearance), 4 mm thick at the crest, flat top for the camera, and a 52 x 18 x 3
  rear tab with two 3.4 mm holes on the camera's 45 mm M3 pair. The camera bolts to the
  pad with its own 1/4-20 (a 1/4-20 x 1/2 in socket screw: 4 mm shell + 4 mm pad leaves
  4.7 mm in the camera, under the 5 mm limit; use a thin washer if the head marks the
  shell) from inside the drum, reached from below between the yokes with a hex key. The
  bolt clamps camera, pad and drum together, the teeth carry the pitch load, so the pitch
  cannot creep. Two M3 x 6 screws (3 mm engagement, the limit) through the tab into the
  rear of the camera stop it yawing on the single bolt and fix its fore-aft position.
- Pitch: loosen the bolt half a turn, lift the head one tooth, re-seat, tighten. 0, -5,
  -10, -15, -20 degrees nose-down all clear the mast (checked). The lens axis is
  `CAM_HEIGHT` (env var, default 150) above the board top at 0 degrees and 141.3 mm at
  -20; it moves forward 12 mm over the range. The USB-C end of the camera hangs 22 mm
  outboard of the pad on the -y side with nothing near it; the tab is 26 mm half-wide,
  the camera 44.7.
- Mass: mast 33.9 cm3, head 11.7 cm3; 42 + 15 = 57 g in solid PLA, 34 g at the 60 %
  figure the other notes use, plus two feet. Under the 60 g target either way.

## Verified in CadQuery (`python3 camera_mast_mount.py`)

```
mast 40.5 x 86.2 x 132.4 mm, 33.9 cm3 (42 g PLA solid, ~25 g at 60% effective); head 11.7 cm3 (15 g solid); feet pitch 47.0 mm across, CAM_HEIGHT 150.0
  solids_mast: 1
  solids_head: 1
  mast_x_head: 0.0
  mast_x_head_pitched: {5: 0.0, 10: 0.0, 15: 0.0, 20: 0.0}
  mast_x_envelope_pitched: {0: 0.0, 5: 0.0, 10: 0.0, 15: 0.0, 20: 0.0}
  head_x_envelope: 0.0
  mast_x_feet: 0.0
  feet_x_clipless: 0.0
  mast_x_plate: 0.0
  feet_x_envelope: 0.0
  lens_above_board_top_at_0: 150.0
  lens_above_board_top_at_-20: 141.3
  lens_x_from_feet_row: 19.0
  lidar_axis_from_feet_row_(x,y): (60.0, -20.5)
  lidar_ahead_of_lens: 41.0
  mast_frontmost_x: 20.25
  margin_to_plinth_body: 8.7
  legs_in_scan_plane: [('rear-right', 75.3, -172.0, 6.8), ('rear-left', 90.6, 145.3, 5.7), ('front-right', 46.7, -167.0, 11.0), ('front-left', 68.7, 131.4, 7.5)]
  occluded_sectors_deg_(bearing_from_dead_ahead,_+left): [(-175.4, -161.5), (127.7, 135.2), (142.5, 148.2)]
  occlusion_in_forward_half: none
  lidar_top_below_image_bottom_deg_per_pitch: [(0, 46.0), (-5, 42.7), (-10, 39.3), (-15, 35.9), (-20, 32.3)]
  lidar_top_enters_image_at_pitch: -60.6
  lidar_top_enters_rgb_image_at_pitch: -65.7
  bolt_engagement_in_camera: 4.7
```

Intersection volumes in mm3: mast against the head at 0 and at each 5 degree pitch step to
-20, mast against the camera envelope (body with the rounded ends, USB-C plug stub) at
each step, head against the envelope, mast against the feet, feet against the clipless
pieces, mast against the baseplate stub, feet against the envelope. Then the lidar
geometry: axis position, margin to the plinth body, the four legs in the scan plane and
the merged sectors (none in the forward half), and the lidar top's angle below the image
bottom per pitch step with the pitch at which it would enter the depth and RGB images.
`PITCH_Y=41 LIDAR_X=60 LIDAR_Y=-20.5 OUT_SUFFIX=_v2` is what `board_layout.py` runs for
Baseplate v2 (`out/camera_mast_v2.*`); the sectors then move by 0.2 degree at most (the
feet pitch and the 1.13 mm rim make that difference; the table above is the v2 set).

## Printing

Mast: floor down, standing, no supports. The legs lean 4 to 11 degrees, the channel
lips are 45 degree flanks, and the drum prints as a 32 mm horizontal bore (the crown
sags a little on the inside, where only the bolt head sits) with the teeth as horizontal
ridges around its top; the two 4 mm wide flats at the shell's lower ends bridge 34 mm
between the yokes. 0.2 mm layers, 4 walls, 40 % infill; the legs are essentially all
wall.

Head: on its rear tab (the tab's back face on the bed), so the pad stands vertical and
the teeth print as vertical serrations at full resolution; the 1/4-20 slot is a 6.8 mm
bridge. No supports. PETG for both if the car lives outdoors in summer: the drum and
teeth are under constant bolt preload.

Hardware: 1 x 1/4-20 x 1/2 in socket head screw (do not use 5/8 in: 6.9 mm would exceed
the camera's 5 mm limit), 2 x M3 x 6 (3 mm engagement, the limit; M3 x 5 with the 3 mm
tab also works), two clipless feet.
