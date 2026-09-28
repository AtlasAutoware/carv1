# Print queue: Bambu Lab P1S, 0.4 nozzle, generic black PLA 1.75, no AMS

Sliced 2026-09-05 with OrcaSlicer 2.4.2 (flatpak `com.orcaslicer.OrcaSlicer`, CLI). All G-code is on
the printer's SD card in `model/` as `P1S_<part>_PLA.gcode` (plain G-code, same naming as the files
already on the card; nothing on the card was removed). Times are Orca's `total estimated time`
(includes the P1S start sequence, about 6 min); grams are Orca's `filament used [g]` at 1.24 g/cm3.

2026-09-06: `audit_feet.py` found two printed parts that could not take their feet (Lite-On cup,
small-board plate) and three files sliced at the wrong pitch for Baseplate v2 (Omni cradle, VESC
tray, LiPo frame). The five replacements are in `Mounts/` on the printer, sent over FTPS:
`P1S_LiteOn_45W_Brick_v2_PLA`, `P1S_Small_Board_Plate_v2_PLA`, `P1S_Omni20_Cradle_v2_PLA`,
`P1S_VESC_FSESC67_Tray_v2_PLA`, `P1S_LiPo_SMC_9000_Frame_v2_PLA`. For the old car on v2, print
the `_v2` Omni cradle, not `P1S_Omni20_Cradle_PLA`.

| G-code on card | source STL | orientation as sliced | supports / brim | time | PLA |
| --- | --- | --- | --- | --- | --- |
| `P1S_Clipless_Foot_PLA.gcode` (one foot) | `out/clipless_foot.stl` | flange down, peg up (rotated 180 about X: the STL is modelled flange-up) | none | 19 min | 5.8 g |
| `P1S_Plate_12xFoot_CamHead_PLA.gcode` | 12 x `out/clipless_foot.stl` + `out/camera_mast_head.stl` | feet flange down; head on its rear tab back face | none | 2 h 23 min | 79.5 g |
| `P1S_Camera_Mast_Head_PLA.gcode` (one head) | `out/camera_mast_head.stl` | on the rear tab's back face (rotated -90 about Y so the -X face is on the bed); pad vertical, teeth as vertical serrations | none | 35 min | 10.8 g |
| `P1S_Omni20_Cradle_PLA.gcode` (STL plate, pitch 49; WRONG for v2) | `out/omni20_cradle.stl` | floor down, as modelled | none | 1 h 28 min | 53.3 g |
| `P1S_Omni20_Cradle_v2_PLA.gcode` (Baseplate v2, pitch 40) | `out/omni20_cradle_v2.stl` | floor down, as modelled | none | 1 h 27 min | 50.8 g |
| `P1S_LiteOn_45W_Brick_v2_PLA.gcode` (re-sliced 2026-09-06: walls opened over the foot slots) | `out/liteon_45w_brick_v2.stl` | floor down, as modelled | none | 1 h 00 min | 26.2 g |
| `P1S_Jetson_Orin_Nano_Under_PLA.gcode` | `out/jetson_orin_nano_under.stl` | floor down, as modelled | none | 1 h 20 min | 45.0 g |
| `P1S_Small_Board_Plate_v2_PLA.gcode` (re-sliced 2026-09-06: PCA boss moved off the foot recess) | `out/small_board_plate_v2.stl` | flat, bosses up, as modelled | none | 1 h 06 min | 18.4 g |
| `P1S_RPLidar_C1_PLA.gcode` (STL plate, pitch 49) | `out/rplidar_c1.stl` | open underside down, as modelled | none | 1 h 03 min | 33.0 g |
| `P1S_RPLidar_C1_v2_PLA.gcode` (Baseplate v2, pitch 40) | `out/rplidar_c1_v2.stl` | open underside down, as modelled | none | 1 h 04 min | 33.9 g |
| `P1S_Camera_Mast_PLA.gcode` (STL plate, pitch 47) | `out/camera_mast.stl` | standing, feet down, as modelled (132 mm tall) | 5 mm outer brim | 2 h 07 min | 38.5 g |
| `P1S_Camera_Mast_v2_PLA.gcode` (Baseplate v2, pitch 41) | `out/camera_mast_v2.stl` | standing, feet down, as modelled (132 mm tall) | 5 mm outer brim | 2 h 06 min | 37.9 g |
| `P1S_Camera_Mast_Newcar_PLA.gcode` (new car, CAM_HEIGHT 170) | `newcar/out/camera_mast_newcar.stl` | standing, feet down, as modelled (152 mm tall) | 5 mm outer brim | 2 h 22 min | 41.2 g |
| `P1S_Cable_Tidy_Edge_PLA.gcode` | `out/cable_tidy_edge.stl` | floor down, as modelled | none | 56 min | 27.1 g |
| `P1S_VESC_Tub_Bracket_PLA.gcode` | `out/vesc_tub_bracket.stl` | legs down, as modelled (STL translated up 12 mm so the leg bottoms are the first layer) | tree(auto) supports, build plate only, 30 deg threshold | 4 h 05 min | 75.1 g |
| `P1S_VESC_FSESC67_Tray_PLA.gcode` (alternative; pitch 49, STL plate only) | `out/vesc_fsesc67.stl` | floor down, as modelled | none | 1 h 29 min | 36.9 g |
| `P1S_VESC_FSESC67_Tray_v2_PLA.gcode` (alternative, pitch 40) | `out/vesc_fsesc67_v2.stl` | floor down, as modelled | none | 1 h 31 min | 37.5 g |
| `P1S_LiPo_SMC_9000_Frame_PLA.gcode` (alternative to the Omni; 49 x 47, STL plate only) | `out/lipo_smc_9000.stl` | floor down, as modelled | none | 1 h 21 min | 44.4 g |
| `P1S_LiPo_SMC_9000_Frame_v2_PLA.gcode` (alternative, 40 x 41) | `out/lipo_smc_9000_v2.stl` | floor down, as modelled | none | 1 h 19 min | 42.9 g |
| `P1S_TiM561_Plinth_PLA.gcode` | `newcar/out/tim561.stl` | upright, floor down, as modelled | none | 1 h 39 min | 46.2 g |
| `P1S_Battery_Bank_Underslung_PLA.gcode` | `newcar/out/battery_bank_underslung.stl` | upright, floor down, fingers up, as modelled | none | 2 h 17 min | 80.6 g |

Total of all 18 files: 28 h 41 min, 737 g. That double-counts the alternatives; a sensible run is one of
each mount plus the combined feet plate (the single-foot and single-head files are there for
reprints). Old car on Baseplate v2, one of each (Omni cradle, Lite-On cup, Jetson tray, small-board
plate, RPLidar v2, camera mast v2, cable tidy, VESC tub bracket, feet plate): 15 h 30 min,
400 g. New car additions (TiM561 plinth, battery bank tray, 170 mm mast): 6 h 18 min, 168 g.

## What the MDs said and what was done

- Every `*_MOUNT.md` "Printing" section says floor down, no supports, except: camera mast (standing,
  no supports; a 5 mm brim was added because it is 132-152 mm tall on a 1380 mm2 contact patch),
  camera head (on its rear tab, no supports: rotated so the tab back is on the bed), and the VESC tub
  bracket, whose MD says legs-down WITH supports under the floor outside the legs (tree supports
  from the build plate only, so the leg faces that grip the rail stay clean).
- The clipless foot STL is modelled with the flange on top and the peg pointing -Z; `OMNI20_MOUNT.md`
  says feet print flange-down, so it was flipped.
- The MDs ask for 3 walls on some parts and 4 on others, 20 % infill on the cable tidy and 40 % on
  the mast; the whole set was sliced uniformly at 4 walls / 30 % as requested. Nothing failed to slice.
- `RPLIDAR_C1_MOUNT.md` and `TIM561_MOUNT.md` mention an optional 8 mm brim for the plinths; not
  applied (flat parts, 1850-2950 mm2 on the bed).
- PETG is recommended in several MDs (VESC bracket, LiPo frame, snap fingers); everything here is
  PLA per the brief.

## Exact profiles and flags

Orca 2.4.2's CLI does NOT resolve the `inherits` chain of the bundled system profiles: loading
`machine/Bambu Lab P1S 0.4 nozzle.json` directly sliced against a 200 x 200 x 100 default bed at
60 mm/s with a 2 mm3/s volumetric cap and density 0. The three chains were therefore flattened
first (`~/slice_work/flatten.py`, parents merged under children, `inherits`/`setting_id` dropped)
from a copy of `/app/share/OrcaSlicer/profiles/BBL/`:

- machine: `Bambu Lab P1S 0.4 nozzle` <- `fdm_bbl_3dp_001_common` <- `fdm_machine_common`
  (256 x 256 x 250, marlin flavour, stock P1S start/end G-code, `bed_exclude_area` 18 x 28)
- filament: `Generic PLA` <- `Generic PLA @base` <- `fdm_filament_pla` <- `fdm_filament_common`
  (220 C nozzle, 55 C textured plate, flow 0.98, 12 mm3/s, 1.24 g/cm3)
- process: `0.20mm Standard @BBL X1C` <- `fdm_process_single_0.20` <- `fdm_process_single_common`
  <- `fdm_process_common` (0.20 layers, 200/300 mm/s walls, 5 top / 3 bottom, 0.42 line)

Overrides applied on top of the process (as `0.20mm Standard @BBL X1C - AtlasMounts`):
`wall_loops 4`, `sparse_infill_density 30%`, `sparse_infill_pattern grid`, `enable_support 0`,
`brim_type no_brim`, `curr_bed_type "Textured PEI Plate"`, `timelapse_type 0`,
`compatible_printers ["Bambu Lab P1S 0.4 nozzle"]`. Variants: `... brim` adds `brim_type outer_only`,
`brim_width 5`, `brim_object_gap 0.1` (masts); `... supports` adds `enable_support 1`,
`support_type tree(auto)`, `support_on_build_plate_only 1`, `support_threshold_angle 30` (tub bracket).
Single filament, no AMS, no filament changes (the stock start G-code's `M620/M621` are the P1S's
own and are inert without an AMS).

Command per part (`~/slice_work/slice_all.sh`):

    flatpak run --filesystem=home --command=orca-slicer com.orcaslicer.OrcaSlicer \
      --load-settings "~/slice_work/profiles/flat/machine_P1S_0.4.json;~/slice_work/profiles/flat/process_common.json" \
      --load-filaments "~/slice_work/profiles/flat/filament_Generic_PLA.json" \
      --slice 0 --arrange 1 --orient 0 --export-3mf <part>.gcode.3mf --outputdir ~/slice_work/out/<part> \
      ~/slice_work/in/<part>.stl

then `Metadata/plate_1.gcode` is unzipped from the `.gcode.3mf` as `P1S_<part>_PLA.gcode`. The
combined plate passes `Clipless_Foot.stl` twelve times plus `Camera_Mast_Head.stl`. Pre-rotated
inputs were written by `/tmp/prep_stl.py` (trimesh via `uv run --with trimesh`) into `~/slice_work/in/`
(the flatpak cannot see the host `/tmp`), each recentred on the bed with z-min 0; the script verified
for every part that the largest axis-aligned flat face is the one on Z=0 (tub bracket excepted, by
design) and that it fits 256 x 256 x 256. The `.gcode.3mf` project files (with thumbnails) are kept
in `~/slice_work/out/<part>/` if the printer should be fed 3MFs instead.
