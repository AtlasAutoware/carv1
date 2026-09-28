# SMC Racing 9000 mAh 4S retention frame for the clipless mounting system

Files in `out/`: `lipo_smc_9000.step/.stl` (print one), `clipless_foot.step/.stl` (print
four, from `omni20_mount.py`), `lipo_smc_9000_assembly.step` (frame, feet, four clipless
pieces in a stub of baseplate, grey envelope of the pack with its XT90 lead),
`lipo_smc_9000.png` (shaded, two views), `lipo_smc_9000_assembly.png`,
`lipo_smc_9000_lines.png` and `lipo_smc_9000_assembly_lines.png` (hidden-line). Source:
`lipo_smc_9000_mount.py`, importing `clipless.py`, `render.py` and the `lines_png` / `box`
/ `inter` helpers of `jetson_orin_nano_mount.py`.

## Component dimensions and where they come from

SMC Racing "HCL-EC 14.8V 9000mAh 100C G10 Protection Plates", product code 90100-4S1P,
https://www.smc-racing.com/index.php?route=product/product&path=205&product_id=917
(category "HCL-EC Car/Trucks/Boats", https://www.smc-racing.com/index.php?route=product/category&path=205).

| Item | Value | Source |
| --- | --- | --- |
| Size | "47mm x 50mm x 160mm" | product page |
| Weight | 785 g | product page |
| Wire | 10 AWG; XT90 / XT90 anti-spark / Traxxas / T / SC5 / EC8 / QS8 connector options | product page |
| Cells | 4S 14.8 V, 100 % LCO, 100C | product page |

Two things to know about the SKU. SMC's HCL-EC 4S 9000 is sold with G10 protection plates
(hard plates on the large faces, a soft-pack edge), not as a moulded hardcase; the only
4S 9000 hardcases on the site are the HCL-HVC series, which are 139 mm long. If the pack on
the car is a real hardcase, measure it and set `LIPO_L`. There is also an "HCL-EC 14.8V
9000mAh 100C Long" variant (product_id 924) with no size listed. And SMC does not say
which of 47 and 50 is the width; the frame is built for 50 wide and 47 tall (`LIPO_W`,
`LIPO_H`) so the pack fits either way, with 1.5 mm of side play if it is the other way
round, which the straps take up.

Estimate for the envelope only: XT90 on the +x end, 22 x 16 x 12 mm, 6 mm above the floor.

## What the frame does

This is the retention frame for carrying the pack on top of the main board (today it
lives in the Slash's stock tray underneath). It has to hold 785 g, the heaviest thing on
the car, against braking and bumps, so the straps do the work and the frame locates.

Pack centre at the origin, +x along the car, leads out of the +x end.

- 4 mm floor shaped as a cross: a 68 mm wide centre strip the full 167 mm length, and two
  40 mm (x) by 87.5 mm (y) transverse bars at x = +-24.5 carrying the four feet. Windows
  between the bars.
- Four feet, 2 x 2, on `PEG_PITCH` x `PITCH_Y` = 49 x 47 (env vars), at (+-24.5, +-23.5).
  Each foot's 30.5 x 34.5 flange recess sits in a 40 x 87.5 bar with 4.75 mm of floor
  beyond it along x and 3.0 mm across, i.e. every foot lands on a bar at least 40 mm wide
  (rule: 36). The pack rests on 18.5 mm of each flange (flanges span y 6.25 to 40.75, the
  pack reaches y 25) and traps all four.
- Four L-shaped corner posts, 3 mm walls, 20 mm legs along x and 14 mm across, 22 mm
  above the floor (pack is 47 tall). The +x end between the posts is open for the leads;
  the -x end is open too.
- Two 25 mm hook-and-loop straps across the pack through four 26 x 4.5 mm floor slots at
  x = +-60.5 (just outside the foot bars) and y = +-28.75 (just outside the pocket).
- Pocket 161 x 51 (0.5 a side). 167 x 87.5 x 26 mm overall, 46.9 cm3, about 35 g in PLA
  at 60 % effective, plus four feet.

Insert the four feet, then the pack, then the straps.

## Verified in CadQuery (`python3 lipo_smc_9000_mount.py`)

```
frame 167.0 x 87.5 x 26.0 mm, 46.9 cm3 (58 g PLA solid, ~35 g at 60% effective), pack 160.0 x 50.0 x 47.0, feet on 49.0 x 47.0 mm
  solids: 1
  frame_x_feet: 0.0
  feet_x_clipless: 0.0
  frame_x_plate: 0.0
  frame_x_envelope: 0.0
  feet_x_envelope: 0.0
  pocket: (161.0, 51.0)
  post_top_z: 26.0
  pack_top_z: 51.0
  feet: [(-24.5, -23.5), (-24.5, 23.5), (24.5, -23.5), (24.5, 23.5)]
  strap_slots_x: (-60.5, 60.5)
  flange_under_pack_mm: 18.5
  foot0_pad_solid_fraction: 0.9919
  foot1_pad_solid_fraction: 0.9919
  foot2_pad_solid_fraction: 0.9919
  foot3_pad_solid_fraction: 0.9919
  pad_margin_x: 4.75
  pad_margin_y: 3.0
```

Intersection volumes in mm3 (frame vs feet, feet vs clipless pieces, frame vs plate stub,
frame and feet vs the pack envelope where it sits). `footN_pad_solid_fraction` is the
check that the feet land on solid floor: the frame's volume inside a 40 x 40.5 mm box
around each foot, divided by what a fully solid bar with only the foot cutout in it would
hold; the missing 0.8 % is the four 4 mm corner fillets of the bar's outline, nothing
else. Mass from volume: 58 g solid PLA, about 35 g as printed.

## Printing

Floor down, no supports; posts are vertical, slots and windows are through-cuts, foot
recesses are open at the top. 0.2 mm layers, 4 walls, 30 % infill. PETG rather than PLA if
the car sits in the sun: this part carries 785 g and PLA creeps.

Fit tuning: `CLR` (0.5), `POST_H` (22), `LIPO_L/W/H`, `PEG_PITCH`, `PITCH_Y`.
