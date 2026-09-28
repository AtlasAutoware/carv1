"""Snap-retained tray that hangs a USB-C PD power bank UNDER Baseplate v2 on four feet
plugged into clipless pieces inserted from above (snap_retain.py), new car.

Bank: Baseus Blade HD 100 W, 20,000 mAh, 133.9 x 133.9 x 17.8 mm, 445 g
(baseus.com/products/blade-laptop-power-bank-100w-20000mah, PPBL000301; ports on one
edge). BANK_L / BANK_W / BANK_T are parameters: BANK_L=161.8 is the Baseus Blade 100 W
(PPBL000001, 161.8 x 133.9 x 17.8, 506 g), which the layout script then checks (it is
wide enough to reach the cable tidy's nuts under the plate edge, see the MD).

Built upright, like the Jetson tray, then hung by snap_retain.hang(): a 4 mm floor with two
8.6 mm channel bars along x on rows B and C (y +-PITCH_Y/2, 40.5 wide each, so they merge
into one 81.5 mm band), the bank lying on the bars with 0.5 mm a side inside full-height
3 mm side walls (+-y, with windows), fixed 1.5 mm lips on two corner legs at the +x (port)
end and two 2 mm snap fingers with 1 mm lips at the -x end. The bank clicks into the tray on
the bench; the tray then slides onto its four feet from the rear (-x), the tongues snap
behind the rear feet. Both ends are open between the legs: the ports face +x, and at -x
the plate's rear clamp block can pass inside the leg line (see the layout).

Frame: floor underside at z 0 (the plate contact face once hung), +x forward.

    python3 battery_bank_underslung.py
    BANK_L=161.8 BANK_W=133.9 python3 battery_bank_underslung.py     # Blade 100 W
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
PARENT = os.path.dirname(HERE)
sys.path.insert(0, PARENT)
import cadquery as cq
from clipless import plate_stub, FOOT_FLANGE, FOOT_CLR
from render import render
from jetson_orin_nano_mount import lines_png, box, inter
from snap_retain import (channel_solids, snap_feet, play_check, hang, piece_from_above,
                         under_assembly, under_shaded, SNAP_FLOOR, UNDER_Z)

# ---- component (baseus.com) ----------------------------------------------------------------
BANK_L = float(os.environ.get('BANK_L', 133.9))     # along the car (x); the port edge is +x
BANK_W = float(os.environ.get('BANK_W', 133.9))     # across (y)
BANK_T = float(os.environ.get('BANK_T', 17.8))
BANK_MASS = float(os.environ.get('BANK_MASS', 445.0))
PORT = (12.0, 6.0, 25.0, -30.0)   # ESTIMATE: USB-C plug body w, h, cable stub length, y of the port

# ---- tray -----------------------------------------------------------------------------------
CLR = 0.5
FLOOR = 4.0
WALL = 3.0
LEG = 20.0                   # corner leg length along the end
LIP_FIXED = 1.5              # +x end
LIP_SNAP = 1.0               # -x end fingers
FINGER_T = 2.0               # like the VESC tray (a 445 g load)
FINGER_W = LEG
LIP_CLR = 0.3
LIP_T = 2.0
PEG_PITCH = float(os.environ.get('PEG_PITCH', 40.0))
PITCH_Y = float(os.environ.get('PITCH_Y', 41.0))
WINDOW = (40.0, 12.0)        # side-wall windows: length, height (grip and weight)
X_OFF = float(os.environ.get('BANK_X_OFFSET', 4.0))   # bank/tray outline shifted +x relative to the feet-pair centre:
                                                      # on the car the tray's rear end then clears the plate's rear clamp
                                                      # block and the rear shock caps (board_layout_newcar.py)

OUT = os.path.join(HERE, 'out')
os.makedirs(OUT, exist_ok=True)
N = 'battery_bank_underslung' + os.environ.get('OUT_SUFFIX', '')

IX, IY = BANK_L + 2 * CLR, BANK_W + 2 * CLR
OL, OW = IX + 2 * WALL, IY + 2 * WALL             # 140.9 x 140.9
Z_BANK = SNAP_FLOOR                                # 8.6: the bank lies on the channel bars
Z_LIP = Z_BANK + BANK_T + LIP_CLR
Z_TOP = Z_LIP + LIP_T                              # 28.7
FEET = [(sx * PEG_PITCH / 2, sy * PITCH_Y / 2) for sy in (-1, 1) for sx in (-1, 1)]
ROWS = sorted(set(y for _, y in FEET))


def tray():
    t = cq.Workplane('XY').rect(OL, OW).extrude(FLOOR).edges('|Z').fillet(4)
    # floor windows outside the channel bands (the bars are put back below)
    band = FOOT_FLANGE[1] / 2 + FOOT_CLR + WALL          # 20.25 half band
    for sy in (-1, 1):
        ya, yb = sorted((sy * (PITCH_Y / 2 + band + 2), sy * (OW / 2 - 6)))
        if yb - ya > 10:
            t = t.cut(box(-OL / 2 + 8, OL / 2 - 8, ya, yb, -1, FLOOR + 1).edges('|Z').fillet(3))
    # side walls (+-y) with windows
    for sy in (-1, 1):
        ya, yb = sorted((sy * IY / 2, sy * (IY / 2 + WALL)))
        w = box(-OL / 2, OL / 2, ya, yb, 0, Z_TOP)
        for k in (-1, 0, 1):
            w = w.cut(box(k * (OL / 3) - WINDOW[0] / 2, k * (OL / 3) + WINDOW[0] / 2, ya - 1, yb + 1,
                          Z_BANK + 3, Z_BANK + 3 + WINDOW[1]).edges('|Y').fillet(2))
        t = t.union(w)
    # +x end: two legs with fixed lips over the bank
    for sy in (-1, 1):
        y0, y1 = (IY / 2 - LEG, IY / 2) if sy > 0 else (-IY / 2, -IY / 2 + LEG)
        t = t.union(box(IX / 2, IX / 2 + WALL, y0, y1, 0, Z_TOP))
        t = t.union(box(IX / 2 - LIP_FIXED, IX / 2 + 0.01, y0, y1, Z_LIP, Z_TOP))
    # -x end: snap fingers with chamfered lips
    for sy in (-1, 1):
        y0, y1 = (IY / 2 - FINGER_W, IY / 2) if sy > 0 else (-IY / 2, -IY / 2 + FINGER_W)
        t = t.union(box(-IX / 2 - FINGER_T, -IX / 2, y0, y1, 0, Z_TOP))
        xi = -IX / 2 - 0.01
        prof = [(xi, Z_LIP), (xi + LIP_SNAP, Z_LIP + LIP_SNAP), (xi + LIP_SNAP, Z_TOP - 0.5), (xi, Z_TOP)]
        t = t.union(cq.Workplane('XZ', origin=(0, y1, 0)).polyline(prof).close().extrude(y1 - y0))
    t = t.translate((X_OFF, 0, 0))
    # channel bars along x on both rows, open at -x (the feet stay on the grid)
    for row in ROWS:
        pair = [(x, y) for (x, y) in FEET if y == row]
        bar, cuts, adds = channel_solids(pair, 'x', SNAP_FLOOR, extent=(-OL / 2 + X_OFF, OL / 2 + X_OFF))
        t = t.union(bar)
        for c in cuts:
            t = t.cut(c)
        for a in adds:
            t = t.union(a)
    return t


def feet():
    return snap_feet(FEET, 'x')


def envelope():
    """Bank on the bars, plus a USB-C plug and cable stub leaving the +x edge."""
    e = box(-BANK_L / 2, BANK_L / 2, -BANK_W / 2, BANK_W / 2, Z_BANK, Z_BANK + BANK_T).edges('|Z').fillet(6)
    pw, ph, pl, py = PORT
    e = e.union(box(BANK_L / 2 - 2, BANK_L / 2 + pl, py - pw / 2, py + pw / 2,
                    Z_BANK + BANK_T / 2 - ph / 2, Z_BANK + BANK_T / 2 + ph / 2))
    return e.translate((X_OFF, 0, 0))


def checks(t):
    fs = feet()
    env = envelope()
    stub = plate_stub(FEET, size=(OL + 20, OW + 20))
    pieces = [piece_from_above().translate((x, y, 0)) for (x, y) in FEET]
    at_stop, at_bump, play = play_check(t, fs, FEET, 'x', inter)
    return {
        'solids': len(t.val().Solids()),
        'tray_x_feet': sum(inter(t, f) for f in fs),
        'tray_x_feet_at_stop': at_stop,
        'tray_x_feet_at_bump': at_bump,
        'fore_aft_play_between_stop_and_bump': play,
        'feet_x_clipless': sum(inter(hang(f), p) for f in fs for p in pieces),
        'tray_x_plate': inter(hang(t), stub),
        'tray_x_envelope': inter(t, env),
        'feet_x_envelope': sum(inter(f, env) for f in fs),
        'hanging_depth_below_plate_top_(2mm_plate)': round(-hang(t).val().BoundingBox().zmin, 2),
        'tray_outline': (OL, OW, Z_TOP), 'tray_x_extent_from_feet_centre': (-OL / 2 + X_OFF, OL / 2 + X_OFF),
        'bank_top_below_contact_face': Z_BANK,
        'lip_underside_z': Z_LIP,
    }


if __name__ == '__main__':
    t = tray()
    bb = t.val().BoundingBox()
    cq.exporters.export(t, os.path.join(OUT, N + '.step'))
    cq.exporters.export(t, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    stub = plate_stub(FEET, size=(OL + 20, OW + 20))
    under_assembly('battery_bank_under_clipless', t, feet(), FEET, envelope(), stub).save(
        os.path.join(OUT, N + '_assembly.step'))
    vol = t.val().Volume()
    print(f'tray {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'({vol/1000*1.24:.0f} g PLA solid, ~{vol/1000*1.24*0.6:.0f} g at 60% effective), bank {BANK_L} x {BANK_W} x {BANK_T}, feet {FEET}')
    for k, v in checks(t).items():
        print(f'  {k}: {v}')
    render([(t, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'), views=[(28, -55), (28, 125)], title='power bank tray (upright, as printed)')
    shaded = under_shaded(t, feet(), FEET, envelope(), stub)
    render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(-25, -55), (-25, 125)],
           title='power bank tray hanging under the plate (grey = bank envelope)')
    lines_png([q for q, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'), os.path.join(OUT, N + '_assembly_lines.png'),
              eye=(0.57, -0.82, -0.47))
    lines_png([t], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
