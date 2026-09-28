# Mounts

Snap-on, modular mounts for the AtlasAutoware car on the clipless mounting system in
`../Templates`. Everything is CadQuery source; `out/` holds the exported STEP, STL, DXF and
renders so nothing has to be rebuilt to print.

Start with `LAYOUT.md` (section 19 is the final placement of all nine mounts on Baseplate
v2, above and below the board, with the interference table, the lidar's occluded sectors
and the camera's line of sight over the lidar) and `CONVENTIONS.md` (how the
clipless foot interface works and what every part must verify). `OMNI20_MOUNT.md` explains
the measured geometry of the Templates and why the foot is a separate part.

| part | script | note | prints |
| --- | --- | --- | --- |
| clipless foot (the interface, print many) | `clipless.py` | `OMNI20_MOUNT.md` | `out/clipless_foot.stl` |
| Baseplate v2, 28-hole, laser cut | `baseplate_v2.py` | `LAYOUT.md` 13-14 | `out/baseplate_v2.dxf` |
| Omni 20+ cradle | `omni20_mount.py` | `OMNI20_MOUNT.md` | `out/omni20_cradle.stl` (STL plate, pitch 49) / `out/omni20_cradle_v2.stl` (v2, pitch 40) |
| Lite-On 45 W brick cup | `liteon_45w_brick_mount.py` | `LITEON_45W_BRICK_MOUNT.md` | `out/liteon_45w_brick.stl` |
| Jetson Orin Nano tray (snap fingers) | `jetson_orin_nano_mount.py` | `JETSON_ORIN_NANO_MOUNT.md` | `out/jetson_orin_nano_under.stl` (below board) |
| PCA9685 + hub plate | `small_board_plate_mount.py` | `SMALL_BOARD_PLATE_MOUNT.md` | `out/small_board_plate.stl` |
| RPLidar C1 plinth | `rplidar_c1_mount.py` | `RPLIDAR_C1_MOUNT.md` | `out/rplidar_c1.stl` (STL plate, pitch 49) / `out/rplidar_c1_v2.stl` (v2, pitch 40) |
| Camera mast + head (Gemini 335) | `camera_mast_mount.py` | `CAMERA_MAST_MOUNT.md` | `out/camera_mast.stl` (STL plate, pitch 47) / `out/camera_mast_v2.stl` (v2, pitch 41), `out/camera_mast_head.stl` |
| Cable tidy, edge-bolted | `cable_tidy_mount.py` (`EDGE=1`) | `CABLE_TIDY_MOUNT.md` | `out/cable_tidy_edge.stl` |
| VESC FSESC 6.7 tub saddle | `vesc_tub_bracket_mount.py` | `VESC_TUB_BRACKET_MOUNT.md` | `out/vesc_tub_bracket.stl` |
| VESC board tray (alternative) | `vesc_fsesc67_mount.py` | `VESC_FSESC67_MOUNT.md` | `out/vesc_fsesc67.stl` (pitch 49) / `out/vesc_fsesc67_v2.stl` (pitch 40) |
| LiPo SMC 9000 frame (alternative to the Omni) | `lipo_smc_9000_mount.py` | `LIPO_SMC_9000_MOUNT.md` | `out/lipo_smc_9000.stl` (49 x 47) / `out/lipo_smc_9000_v2.stl` (40 x 41) |

Regenerate everything and re-run the fit proof:

    pip install cadquery trimesh cairosvg matplotlib pillow
    PLATE=v2 python3 board_layout.py
    python3 audit_feet.py          # can every foot be installed, is every STL at the plate's pitch

Before cutting or printing, measure the items in `LAYOUT.md` section 19 (board-to-tub
clearance, plate thickness, tub rail, servo position, Lite-On brick body). Several
component dimensions are marked as estimates in their MD files; the scripts take them as
environment variables so a measurement is a one-line change.
