"""Baseplate v2 for the new car (Traxxas Slash 4x4, standard 6822 chassis): the same
452 x 177.5 outline and 28-hole 40 x 41 clipless grid as ../baseplate_v2.py, with the
body-post cutouts and shock-tower clamp holes moved to where the 4x4's towers are.

Nothing about the 4x4's tower spacing is published; the only manufacturer numbers are the
wheelbases (Slash 2WD 335 mm, Slash 4x4 324 mm; traxxas.com specifications quoted by
hobbytown.com). The old plate's clamps are at x +173.27 / -149.73 (measured from
templates/Baseplate.stl), and its chassis model puts the axles 10 ahead of the front tower
and 4 behind the rear one (LAYOUT.md section 3, estimates). Keeping the front tower where
it is, an 11.4 mm shorter wheelbase moves the rear tower, its clamp holes, its clamp blocks
and its body-post cutouts 11.4 mm FORWARD: REAR_SHIFT (default 11.4) and FRONT_SHIFT
(default 0) are the parameters. MEASURE BOTH ON THE CAR before cutting: the distance between
the two shock towers' top edges, the post spacing on each tower, and the tower thickness
(see NEWCAR_LAYOUT.md, "Measure on the car"). The web checks of baseplate_v2 still run:
the rear cutouts at x -127.7 keep 8.2 mm to column 6 (was 11.5) and 22.8 to column 8.

    python3 baseplate_v2_newcar.py                writes out/baseplate_v2_newcar.dxf / .step / .stl / .png
    REAR_SHIFT=9 FRONT_SHIFT=-2 python3 baseplate_v2_newcar.py
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
PARENT = os.path.dirname(HERE)
sys.path.insert(0, PARENT)
import baseplate_v2 as B

REAR_SHIFT = float(os.environ.get('REAR_SHIFT', 335.0 - 324.0 + 0.4))   # 11.4: the old model's wheelbase is 335.4
FRONT_SHIFT = float(os.environ.get('FRONT_SHIFT', 0.0))
OUT = os.path.join(HERE, 'out')
os.makedirs(OUT, exist_ok=True)


def _sx(x):
    return x + (REAR_SHIFT if x < 0 else FRONT_SHIFT)


# move the chassis fixings; the grid, zip holes and M3 holes stay
B.CUTOUTS[:] = [((_sx(x), y), w, h) for ((x, y), w, h) in B.CUTOUTS]
B.SMALL[:] = [((_sx(x), y), d) for ((x, y), d) in B.SMALL]
B.BRACKETS[:] = [(_sx(x0), _sx(x1), y0, y1, zr) for (x0, x1, y0, y1, zr) in B.BRACKETS]

# re-export what board_layout_newcar.py uses
HOLES, COLS, ROWS, ZIP_HOLES, ZIP_Y, M3_HOLES = B.HOLES, B.COLS, B.ROWS, B.ZIP_HOLES, B.ZIP_Y, B.M3_HOLES
CUTOUTS, SMALL, BRACKETS = B.CUTOUTS, B.SMALL, B.BRACKETS
L_PLATE, W_PLATE, PLATE_T, HOLE, PITCH, PITCH_Y = B.L_PLATE, B.W_PLATE, B.PLATE_T, B.HOLE, B.PITCH, B.PITCH_Y
plate_solid, check_webs, bracket_solids = B.plate_solid, B.check_webs, B.bracket_solids


def clamp_x():
    return sorted(set(round(x, 2) for (x, _), _ in SMALL))


if __name__ == '__main__':
    import cadquery as cq
    print(f'shifts: rear {REAR_SHIFT:+.1f}, front {FRONT_SHIFT:+.1f}')
    print('cutouts', [(tuple(round(v, 2) for v in c), round(w, 2), round(h, 2)) for c, w, h in CUTOUTS])
    print('clamp holes', [(tuple(round(v, 2) for v in c), d) for c, d in SMALL])
    print('clamp blocks', [tuple(round(v, 2) for v in b[:4]) for b in BRACKETS])
    probs = check_webs()
    print('web problems:', probs or 'none')
    assert not probs
    p = plate_solid()
    assert len(p.val().Solids()) == 1
    B.write_dxf(os.path.join(OUT, 'baseplate_v2_newcar.dxf'))
    cq.exporters.export(p, os.path.join(OUT, 'baseplate_v2_newcar.step'))
    cq.exporters.export(p, os.path.join(OUT, 'baseplate_v2_newcar.stl'), tolerance=0.02, angularTolerance=0.1)
    B.write_png(os.path.join(OUT, 'baseplate_v2_newcar.png'))
    # webs to the moved cutouts, for the MD
    import math
    for n, (x, y) in HOLES.items():
        r = B.hole_rect(x, y)
        for ((cx, cy), w, h) in CUTOUTS:
            g = B._rect_gap(r, (cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2))
            if g < 30:
                print(f'  {n}: {g:.2f} mm to the cutout at ({cx:.1f}, {cy:.1f})')
    print('written out/baseplate_v2_newcar.*')
