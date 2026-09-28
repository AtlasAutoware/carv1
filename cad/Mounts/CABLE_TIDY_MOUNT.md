# Cable tidy rail for the clipless mounting system

Files in `out/`: `cable_tidy.step/.stl` (print one), `clipless_foot.step/.stl` (print two,
from `omni20_mount.py`), `cable_tidy_assembly.step` (rail, feet, two clipless pieces in a
stub of baseplate, grey sample cable slack), `cable_tidy.png` (shaded, two views),
`cable_tidy_assembly.png`, `cable_tidy_lines.png` and `cable_tidy_assembly_lines.png`
(hidden-line). Source: `cable_tidy_mount.py`, importing `clipless.py`, `render.py` and the
`lines_png`/`box`/`inter` helpers of `jetson_orin_nano_mount.py`. `out/_cable_tidy_closeup.png`
is a scratch render the workspace would not let me delete; ignore it.

## What it is for

Not a component mount. It is the place for the slack of the cables that run along the
board: the USB-C to barrel lead from the Omni 20+ to the Lite-On brick, the brick's 19 V
barrel lead to the Jetson, the USB leads of the Orbbec Gemini 335 and the RPLidar C1
adapter, the VESC's USB lead and a servo lead. Owner's numbers: 150 to 400 mm of excess
per cable, 3 to 6 mm cable diameter. There is no datasheet; the dimensions that size the
part are in the script as named parameters:

| Parameter | Value | Why |
| --- | --- | --- |
| `CABLE_D_MAX` | 6.0 mm | thickest lead (19 V barrel, USB-C to barrel) |
| `BEND_R_MIN` | 8.0 mm | surface radius a 6 mm cable may be wrapped around; sets the post diameter |
| `LIDAR_SCAN_ABOVE_BOARD` | 44.9 mm | RPLIDAR_C1_MOUNT.md; the rail must stay under it |
| `PEG_PITCH` | 49.0 mm (env var) | the clipless hole pair used |

## What the rail does

- 150 x 40.5 x 23.5 mm, along the car's right edge: +x forward, -y outboard (toward the
  right wheels), floor underside on the clipless rim tops. Top of the caps is 24.6 mm above
  the board top, 20 mm under the lidar scan plane.
- Width: the brief asked for 36 mm. The rail is 40.5 because the slide channel needs the
  34.5 mm flange slot plus a 3 mm wall each side, exactly as the RPLIDAR plinth's bar;
  36 mm would leave 0.75 mm walls. The extra 4.5 mm went into the cable passages.
- Cable floor at z 7.6 (channel tunnel below it). Six hollow spool posts on the centreline,
  16 mm diameter (8 mm bend radius), 24 mm pitch, x = -60 .. 60, 15.9 mm tall, 1.6 mm
  walls, 2 mm base fillet, and a 1 mm flared cap with a 45 degree underside and a 0.8 mm
  rounded rim so a wound loop cannot ride up off the post. Gap between posts 8 mm, so a
  6 mm cable crosses between them.
- Outboard lip: 2 mm thick, 8.5 mm above the floor (16.1 mm overall), 2 mm fillet at its
  base, 1 mm round on the inner top edge, 1 mm chamfer on the outer top edge. Between the
  posts five comb fingers grow from the lip toward the post row: 2.4 mm at the root with a
  2 degree draft, top sloping from 8 mm at the lip to 4.5 mm at the tip, tip stopping at
  y = -10.5 (2.5 mm short of the post radius, 7.9 mm from the nearest post surface), tip
  edges rounded 0.7. They divide the 10.25 mm outboard trough into one cell per post.
- Inboard edge open: no wall, the floor's top edge is a 3 mm round (`ENTRY_R`), so a cable
  is laid in from above and never threaded. Inboard passage past the posts is 12.25 mm.
- All four vertical corners 2 mm rounds. ATLAS embossed 0.6 mm on the outboard face,
  7 mm bold, centred on the face.
- Feet: two clipless feet along the car, `PEG_PITCH` apart, in the dovetail slide channel of
  `rplidar_c1_mount.py` (24.3 mm peg slot through a 2 mm ledge, 34.5 mm flange slot 2 mm
  tall, 45 degree lips 2 mm tall closing to 30.5 mm at z 6, 1.6 mm roof). The channel is
  open at the rear end and stops 0.75 mm past the front flange. The last 14.5 mm of each
  ledge behind the rear flange is a 2 x 4.5 mm spring tongue (side slit and end slit
  0.6 mm) with a 0.6 mm bump whose steep face sits 1 mm behind the rear flange. Put both
  feet in their pockets, slide the rail on from the rear until it clicks. The tongues are
  35 mm inside the tunnel, so two 3.2 mm release holes through the floor at x = -43,
  y = +-14.4 sit over the tongue tips: a 2.5 mm hex key pressed into each drops the bump
  and the rail slides off aft. Fore-aft play between stop and bump 1.75 mm, on top of the
  +-2 mm the 20 mm pegs float in their 24 mm pockets.
- Ahead of the stop the floor carries the same channel section as a lightening pocket
  (x 43.25 .. 72), open underneath like the tunnel.

## What goes where

Thick leads (19 V barrel, USB-C to barrel) go on as a figure-of-eight around two
neighbouring posts, one cable per pair, from the rear: posts 0-1 and 2-3 for the two power
leads, so their loops sit over the feet where the floor is a stiff closed section. The
thin USB leads (camera, lidar, VESC) and the servo lead each get a single post, coiled two
or three turns; the finger either side keeps the coil in its cell. Each figure-of-eight
wrap takes about 110 mm of cable and each single-post turn about 70 mm, so 400 mm of thick
cable is three or four layers (18 to 24 mm, the full post height) and 400 mm of thin cable
five or six turns; if a cable has more, wrap it over a second pair. Lay cables in from the
inboard side over the 3 mm round; the free ends leave inboard toward their connectors, the
lip and caps stop loops from flopping toward the wheel. The grey solids in the assembly
show one two-layer figure-of-eight of 6 mm cable on posts 0-1 and a three-turn coil of
4 mm on post 4, with tails leaving inboard.

## Verified in CadQuery (`python3 cable_tidy_mount.py`)

```
rail 150.0 x 41.1 x 23.5 mm, 31.5 cm3 (39.1 g PLA solid, ~23 g at 60% effective), peg pitch 49.0 mm
  solids: 1
  rail_x_feet: 0.0
  rail_x_feet_at_stop: 0.0
  rail_x_feet_at_bump: 0.0
  fore_aft_play_between_stop_and_bump: 1.75
  feet_x_clipless: 0.0
  rail_x_plate: 0.0
  rail_x_cables: 0.0
  feet_x_cables: 0.0
  rail_top_above_board_top: 24.633
  lidar_scan_plane_above_board_top: 44.9
  post_surface_radius: 8.0
  outboard_trough_width: 10.25
  inboard_passage_width: 12.25
  post_gap: 8.0
  text_proud: 0.6
```

Intersection volumes in mm3: rail against the feet nominal, pushed to the stop and pushed
to the bump; feet against the clipless pieces; rail against the baseplate stub; rail and
feet against the sample cable slack (the "component" here). The 41.1 mm width is the 40.5
body plus the 0.6 mm lettering. The section was also sampled point by point: roof over the
tunnel solid at z 6.8, tunnel and peg slot void, ledge and 45 degree flank solid where they
should be, tongue free of its side and end slits, bump at z 2.0 .. 2.6, release holes open,
stop wall and front pocket, post bore and cap, finger solid at y -14 and void at -9.5,
lettering proud of the outboard face between x -11 and 10.5, z 6 .. 10.5.

## Printing

Floor down, no supports. The channel and the front pocket print as tunnels: the 45 degree
lip flanks are the only overhang and the 30.5 mm roof is a bridge (1.6 mm, 8 layers at
0.2). The post caps overhang 1 mm at 45 degrees, the lettering 0.6 mm sideways. 0.2 mm
layers, 3 walls (the 1.6 mm post walls become 4 perimeters), 20 % infill, PLA; PETG if
the tongues are to be worked often. Mass 39 g solid, under 30 g as printed, plus two feet.
Fit tuning: `FOOT_CLR` in `clipless.py` (shared), `BUMP_H` (0.6) and `TONGUE_L` (14.5) for
the snap, `ROOF` if the bridge sags.

## Parameters added for Baseplate v2

`L` (rail length, default 150) and `N_POST` (default 6): `L=100 N_POST=4` is the
four-post rail (posts at -36, -12, 12, 36; fingers at -24, 0, 24; the lightening pocket
ahead of the stop is skipped when it would be under 8 mm long), `out/cable_tidy_100.*`,
21.3 cm3, all checks zero, play 1.75. It does not fit on v2 either (LAYOUT.md section 17).

## Edge variant for Baseplate v2 (`EDGE=1`, `out/cable_tidy_edge.*`)

`EDGE=1 L=100 N_POST=4 python3 cable_tidy_mount.py` (what `PLATE=v2 board_layout.py`
runs). Same posts, comb fingers, lip, entry round and ATLAS lettering; no foot channel,
no feet. The rail hangs off the plate's right edge and bolts through two of the v2 plate's
6 mm zip-tie holes.

- Rail body 100 x 36 x 23.5 (`W_EDGE` 36; the 40.5 of the clipless rail was the flange
  slot, which is gone). Inboard face 0.25 mm off the plate edge (`EDGE_GAP`), so cables
  cross from the plate top over the 3 mm entry round onto the cable floor 7.6 mm up. Top
  of the caps 23.5 above the plate top (limit 24). Outboard trough 8.0, inboard passage
  10.0, post gap 8.0.
- Underside pocket 3 mm deep leaves a 3 mm floor and walls, with a 6 mm rib under the pad
  that is inside the rail. 27.8 cm3, about 21 g printed.
- Two pads 14 x 15.5 x 4 mm reach over the plate top to y -76 in board coordinates
  (`PAD_IN` 12.75 from the edge; the row-D clipless flange zone starts at -78 only for
  from-above pieces, and D6/D8 are unused), with 5.3 mm holes on the zip-hole line
  (6.75 mm inboard of the edge, 40 mm apart).
- Fastening: M5 x 16 button-head screws with M5 nyloc nuts under the plate. Not the
  M5 x 12 of the brief: the stack is 4 (pad) + 3.175 (plate) = 7.2 mm and a nyloc is 5 mm
  tall with its collar in the last 1.5 mm, so 12 mm leaves the collar unengaged; 16 gives
  3.8 mm past the nut. Printed 6 mm snap pins were considered and rejected: a pin that
  passes a 6.0 mm laser-cut hole (6.1 to 6.2 after kerf) has a 5.8 mm shank, a split snap
  head makes that two 2.5 mm half-shanks with a 0.8 mm slit, and in PLA those take one or
  two removals before the layer lines let go; on a part whose whole point is being taken
  off to re-dress cables, two nuts are the right answer. Screw play in the 6 mm holes
  (0.5 mm a side) is what lets the rail be pushed against the plate edge before tightening.
- Ledge: the rear pad is 40 mm behind the rail's rear end on a 12 x 7.6 mm ledge along the
  edge (`SPINE_W` 12, ending 11.75 mm outboard of the edge, 5.75 mm inboard of the rear
  tyre's inner face). Why: on v2 the rail has to sit at x -112..-12 (see LAYOUT.md, "Final
  set on v2"), and the zip holes beside it at x -65, -25 and 15 have the Jetson tray's
  floor under them (a nut there needs a pocket in that floor next to its standoff
  bosses), while 55, 95 and 135 have the small-board plate over them. The pair at -105
  and -145 is the only clean one, so the rail reaches back to it. Overall 140 x 49.6.
- `BRACKET_X` (comma list, rail coordinates) overrides the pad positions for another plate.

Verified (`EDGE=1 L=100 N_POST=4 python3 cable_tidy_mount.py`):

```
rail 140.0 x 49.6 x 23.5 mm, 27.8 cm3 (34.5 g PLA solid, ~21 g at 60% effective), edge brackets [-43.0, -83.0]
  solids: 1
  rail_x_plate: 0.0
  rail_x_fasteners: 0.0
  rail_x_cables: 0.0
  fasteners_x_plate: 0.0
  rail_top_above_plate_top: 23.503
  inboard_face_gap_to_plate_edge: 0.25
  outboard_face_from_plate_edge: 36.85
  pad_inboard_reach_from_plate_edge: 12.75
  pad_hole_d: 5.3
  bracket_x: [-43.0, -83.0]
  ledge_width_outboard_of_edge: 11.75
  outboard_trough_width: 8.0
  inboard_passage_width: 10.0
  post_gap: 8.0
```

The plate stub in `cable_tidy_edge_assembly.step` is a piece of the v2 edge with the two
zip holes; the grey solids are the two screws with heads and nuts (checked against the
rail and the plate) and the sample cable slack. On the car it is checked in
`board_layout.py` against the rear tyre at full bump, the Jetson tray under the plate,
the cup and the small-board plate (LAYOUT.md, "Final set on v2").

Printing: floor down, no supports; the underside pocket is open downward so the 36 mm
floor prints as a 3 mm slab on 3 mm walls (no bridge). Post caps and lettering as before.
