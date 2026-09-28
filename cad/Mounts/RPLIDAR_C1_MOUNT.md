# RPLIDAR C1 plinth for the clipless mounting system

Files in `out/`: `rplidar_c1.step/.stl` (print one), `clipless_foot.step/.stl` (print two,
from `omni20_mount.py`), `rplidar_c1_assembly.step` (plinth, feet, two clipless pieces in a
stub of baseplate, grey envelope of the lidar), `rplidar_c1.png` (shaded, two views),
`rplidar_c1_assembly.png`, `rplidar_c1_lines.png` and `rplidar_c1_assembly_lines.png`
(hidden-line). Source: `rplidar_c1_mount.py`, importing `clipless.py`, `render.py` and the
`lines_png`/`box`/`inter` helpers of `jetson_orin_nano_mount.py`. `out/_c1_debug.png` is a
scratch section render that the workspace would not let me delete; ignore it.

## Component dimensions and where they come from

SLAMTEC RPLIDAR C1 (model C1M1). Product page https://www.slamtec.com/en/c1 and spec page
https://www.slamtec.com/en/c1/spec ("Size: 55.6 x 55.6 x 41.3 mm, Height 41.3 mm, Weight
110 g"). Datasheet: "RPLIDAR C1 Introduction and Datasheet", rev 1.1 2024-03-12, linked from
https://www.slamtec.com/en/support#rplidar-c1; copies read at
https://static.generation-robots.com/media/slamtec-rplidar-c1-datasheet.pdf (rev 1.1) and
https://d229kd5ey79jzj.cloudfront.net/3157/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf
(rev 1.0, same drawing). Development kit user manual (no mechanical data):
https://files.waveshare.com/wiki/RPLIDAR-C1/SLAMTEC_rplidarkit_usermanual_C1_v1.0_en.pdf

| Item | Value | Source |
| --- | --- | --- |
| Base outline | 55.6 x 55.6 mm square, rounded corners | datasheet Figure 4-1 (page 18), spec page |
| Total height | 41.3 mm: base 23.1 + optical head 18.2 | Figure 4-1 |
| Scan plane ("Laser Receiving and Emitting Height") | 29.8 mm above the underside | Figure 4-1 |
| Scan field flatness | 0 to 1.5 degrees | Figure 2-1 |
| Mounting holes | 4 x M2.5 in the underside on a 43 x 43 mm square; "depth of 4*M2.5 screws in the bottom should be no longer than 4 mm" | Figure 4-1 and its note 1 |
| Tolerance | +-0.2 mm | Figure 4-1 note 2 |
| Connector | XH2.54-5P male socket at the centre of one base edge, at the bottom of the base | Figure 4-1, Figure 2-5 |
| Zero angle | x-axis is "dead ahead", pointing away from the connector edge; angle increases clockwise (left-hand system) | Figure 2-4 and its text (page 11) |
| Field of view | 360 degrees, 0.72 degree resolution at 10 Hz | Figure 2-1 |
| Mass | 110 g | Figure 2-9, spec page |

Estimates, marked as such in the script: head diameter 44 mm (`LIDAR_HEAD_D`, the drawing
shows the head narrower than the base but does not dimension it); connector housing 6 mm
proud of the base edge, 12 mm wide, 7 mm tall (`CONN`). Both only place the grey envelope
and size the connector notch (14 mm wide). The scan plane, hole pattern, screw depth and
outline come straight from Figure 4-1.

## What this plinth does

- Raises the lidar so its scan plane clears the tallest thing on the board.
  `LIDAR_SCAN_CLEARANCE` (env var, default 34 + 10 = 44 mm above the board top) sets the
  deck height: `Z_DECK = ceil(LIDAR_SCAN_CLEARANCE - 29.8 - RIM_PROUD)` = 14 mm, so the scan
  plane is at 1.13 + 14 + 29.8 = 44.93 mm and the lidar top at 56.4 mm. The C1 already
  carries its scan plane 29.8 mm up, which is why the plinth is only 16 mm tall; a taller
  clearance simply grows the box.
- 62 x 62 x 16 mm hollow box, 3 mm walls, 4 mm deck, open underneath. The deck has a 2 mm
  keyed pocket (56.1 square, 0.25 mm clearance a side, which is what locates the lidar; the
  screws only clamp) and a 26 mm lightening window in its centre.
- Keying: the pocket rim is unbroken except for a 14 mm notch at the rear (-x). The XH
  connector housing sits on the base's underside edge, so the lidar only goes flat into the
  pocket with its connector in that notch, which puts the x-axis / zero-angle mark forward.
  There is also a small arrow notch cut in the front rim top.
- Cable: the notch continues down through the rear wall to the floor bar, so the cable
  drops straight out of the connector onto the 13 mm rear extension of the bar and runs
  aft along it.
- Four M2.5 x 8 socket screws go up through 2.8 mm holes in the 4 mm deck into the lidar's
  own M2.5 threads: 4 mm of engagement, the maximum the datasheet allows. Their heads sit
  in 5.5 mm counterbores that open into the hollow underside, so the lidar is screwed on
  with the plinth upside down on the bench, before it goes on the car. No heat-set inserts:
  the lidar is the tapped part.
- Feet: two clipless feet along the car, `PEG_PITCH` (env var, default 49.0) apart. Nothing
  sits on their flanges here (the deck is 10 mm above them), so the drop-in recess of
  `clipless.foot_cutout` would let the plinth lift straight off its feet. Instead the floor
  bar (40.5 wide, 10 tall, 89 long) carries a dovetail slide channel along x
  (`slide_channel`): 24.3 mm peg slot through the 2 mm ledge, 34.5 mm flange slot 2 mm
  tall, then 45 degree lips closing to 28.5 mm at z 7 and a 3 mm roof. Clearances are the
  same 0.25 mm as the recess. Put both feet in their clipless pockets, slide the plinth on
  from the rear over both flanges until the front flange meets the end stop; the last
  14.5 mm of each ledge is a 2 x 4.5 mm spring tongue whose 0.6 mm bump the rear flange
  rides over and which snaps up behind it. The flange can lift 0.25 mm before its top
  corners wedge under the lips, so the plinth cannot come off the feet; to remove it, press
  the two tongues down through the open rear end and pull. Fore-aft play between the stop
  and the bump is 1.75 mm, on top of the +-2 mm the pegs already float in their pockets
  (the pegs are 20 mm in 24 mm pockets by design). Set `PEG_PITCH` to the measured pitch
  of the hole pair used: if the real pitch is more than 1 mm longer the rear flange parks
  on the bump instead of behind it (still retained by the lips, now spring-loaded); if
  shorter, the play grows by the difference.
- Stiffness: a 16 mm tall closed box under a 110 g lidar; the only compliance is the
  0.25 mm foot clearances. 35.9 cm3, about 44 g in solid PLA (27 g at the 60 % figure the
  other notes use), plus two feet.

## Verified in CadQuery (`python3 rplidar_c1_mount.py`)

```
plinth 89.2 x 62.1 x 16.0 mm, 35.9 cm3 (44 g PLA solid, ~27 g at 60% effective), peg pitch 49.0 mm
  solids: 1
  plinth_x_feet: 0.0
  plinth_x_feet_at_stop: 0.0
  plinth_x_feet_at_bump: 0.0
  fore_aft_play_between_stop_and_bump: 1.75
  feet_x_clipless: 0.0
  plinth_x_plate: 0.0
  plinth_x_envelope: 0.0
  feet_x_envelope: 0.0
  deck_z: 14
  scan_plane_above_board_top: 44.93
  required: 44.0
  lidar_top_above_board_top: 56.43
  screw_engagement_in_lidar: 4.0
```

Intersection volumes in mm3: plinth against the feet in the nominal position, pushed
against the end stop and pushed against the snap bump; feet against the clipless pieces;
plinth against the baseplate stub; plinth and feet against the lidar envelope (base block,
head cylinder, connector housing) sitting in the pocket. The channel cross-section was also
sampled point-by-point to confirm the ledge, flange slot, 45 degree lips, roof, tongue
slot and bump come out at the intended heights.

## Printing

Underside down (the open side on the bed), no supports. The channel is printed as a
tunnel: the 45 degree lip flanks are the only overhang and the 28.5 mm roof and the 7.5 mm
deck spans to the walls are bridges. 0.2 mm layers, 3 walls, 30 % infill, PLA is fine;
PETG if the snap tongues are to be worked many times. Fit tuning: `FOOT_CLR` in
`clipless.py` is shared; `BUMP_H` (0.6) and `TONGUE_L` (14.5) set the snap force.

Screws: 4 x M2.5 x 8 socket head (not longer: 4 mm into the lidar is the limit).

## Where it goes on Baseplate v2, and what the lidar sees

LAYOUT.md section 19: the plinth is the front-most thing on the board, on B1+B2 (feet along
the car at x 115 and 75, centre (95, 20.5), 20.5 mm left of the centreline), connector
notch aft, `PEG_PITCH=40` (`out/rplidar_c1_v2.*`, `OUT_SUFFIX=_v2`, written by
`board_layout.py`). Deck at 15 (`LIDAR_SCAN_CLEARANCE` 45.5 in the layout), so on the
3.175 mm plate the scan plane is 44.8 above the plate top and the lidar top 56.3. The
first v2 placement had it on A2+A3 with the camera mast one column AHEAD of it, whose legs
cut the scan in the forward-right quadrant; the mast is now one column BEHIND it (A3+B3),
so nothing on the board crosses the scan plane in the forward half. What does cross it,
all in the rear half, are the mast's four legs, at 47 to 90 mm:

| bearing from dead ahead (+left) | C1 datasheet angle (clockwise) | what |
| --- | --- | --- |
| -175.6 .. -161.7 | 161.7 .. 175.6 | both right-hand legs, one behind the other |
| 127.9 .. 135.4 | 224.6 .. 232.1 | front-left leg |
| 142.6 .. 148.3 | 211.7 .. 217.4 | rear-left leg |

Mask those in the driver (angle filters) or drop returns under 0.10 m; the front-right leg
at 46.7 mm is inside the C1's 0.05 m minimum range anyway. Everything else on the board is
under the plane (cup fingers 34.3, brick 32, pack 31, front body posts 25, tyres at full
bump +15.8). The lead leaves the rear notch onto the bar's rear extension at (55.5..64,
20.5) and drops through hole C2 (75, -20.5), 21 mm of which is open between this plinth's
wall and the small-board plate's edge; the XH-USB adapter is zip-tied under the plate at
the C2/C3 web, 15 mm above the VESC saddle's case. The plinth slides on from the rear over
B1+B2 BEFORE the mast goes on (it starts over the mast's footprint); to remove it, the
mast comes off first. Its bar's rear end is 0.25 mm ahead of the mast's bar; both parts
float +-2 mm on their pegs, so they may touch, at floor level, which costs nothing.

## Parameter added for Baseplate v2

`BAR_END` (default 5.0, minimum 3.0): thickness of the floor bar's front end wall beyond
the channel stop. The first v2 placement (plinth on columns 2-3 behind the mast on column
1) needed 3.5 to stop 0.5 mm short of the mast bar; with the plinth in front on columns
1-2 nothing is ahead of its bar and the default 5 is used again (bar x 55.5..135.75 on the
board, 24 mm behind the front cutouts and 41 behind the front body posts). All checks
unchanged (single solid, zero intersections, play 1.75).
