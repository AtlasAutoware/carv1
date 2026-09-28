# New car ("KA2246-R00", Traxxas Slash 4x4, 6822 chassis)

Mount set for the second car on the same clipless system and Baseplate v2 grid as the
parent directory. Everything here imports the shared modules from `..` (`clipless.py`,
`snap_retain.py`, `render.py`, the reused mount scripts) and writes only into `newcar/out/`;
nothing in `../out` is touched. Start with `NEWCAR_LAYOUT.md` (placement, interference,
power tree, cables, what to measure) and `PARTS_LIST.md`.

    cd newcar
    python3 tim561_mount.py                 # TiM561 plinth
    python3 battery_bank_underslung.py      # power bank tray
    python3 baseplate_v2_newcar.py          # plate with the 4x4's tower spacing (parametric)
    python3 board_layout_newcar.py          # whole car: checks, renders, assembly STEP, report JSON
    LAYOUT=above python3 board_layout_newcar.py    # fallback arrangement (bank above, Jetson under)

| part | script | MD | print / cut |
| --- | --- | --- | --- |
| TiM561 plinth, four feet (new) | `tim561_mount.py` | `TIM561_MOUNT.md` | `out/tim561.stl` |
| power bank tray, underslung (new) | `battery_bank_underslung.py` | `BATTERY_BANK_UNDERSLUNG.md` | `out/battery_bank_underslung.stl` |
| Baseplate v2, new-car tower spacing (new cut) | `baseplate_v2_newcar.py` | `NEWCAR_LAYOUT.md` 1, 8 | `out/baseplate_v2_newcar.dxf`, after measuring |
| camera mast at `CAM_HEIGHT` 170 (reused script, new print) | `../camera_mast_mount.py` via the layout | `NEWCAR_LAYOUT.md` 3, `../CAMERA_MAST_MOUNT.md` | `out/camera_mast_newcar.stl` + `../out/camera_mast_head.stl` |
| Jetson tray (reused, identical) | `../jetson_orin_nano_mount.py` | `../JETSON_ORIN_NANO_MOUNT.md` | `../out/jetson_orin_nano_under.stl` (used upright above the plate) |
| VESC saddle (reused, identical) | `../vesc_tub_bracket_mount.py` | `../VESC_TUB_BRACKET_MOUNT.md` | `../out/vesc_tub_bracket.stl`; or `VESC=rear`: `out/vesc_fsesc67_p41.stl` |
| cable tidy, edge (reused, identical) | `../cable_tidy_mount.py` | `../CABLE_TIDY_MOUNT.md` | `../out/cable_tidy_edge.stl` |
| clipless feet x 14, pieces x 12 | `../clipless.py`, `../templates` | `../OMNI20_MOUNT.md` | `../out/clipless_foot.stl` |
| INJORA INJS235 servo | nothing to print | `INJORA_SERVO_MOUNT.md` | - |

Not carried over from the old car: Omni 20+ cradle and Lite-On cup (replaced by the bank and
PD trigger), small-board plate (no hub, no PCA9685), LiPo frame, RPLIDAR C1 plinth.
