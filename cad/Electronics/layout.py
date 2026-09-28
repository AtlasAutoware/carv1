"""Floorplan for AtlasPower-4S: where every part sits, and why.

Board coordinates: origin at the board centre, +x right, +y down (KiCad's convention).
88 x 68 mm, so x runs -44..+44 and y runs -34..+34.

The board reads left to right, which is also the order power flows:

    x -44..-22   input: terminal block, fuse, TVS, ideal-diode FET, shunt, INA226
    x -22..+14   three buck rails stacked in y, each a copy of the same block
    x  +14..+44  outputs: barrel jack, terminal blocks, headers

Each rail is laid out to the AP64501 datasheet's Figure 31: input ceramics hard against
VIN/GND, the inductor immediately at SW, output ceramics beyond it, and the feedback
divider tucked next to FB away from the switch node. The three rails are stacked rather
than spread so that all three share one input capacitor bank and one ground pour.

Rotations are degrees counter-clockwise, KiCad's convention.
"""

BOARD_W, BOARD_H = 88.0, 68.0
EDGE = 2.0                      # keep-out from the board edge
HOLE_DX, HOLE_DY = 76.0, 56.0   # M3 mounting hole pattern, matches power_board_mount.py

# rail block origins: the three identical rails, stacked in y
RAIL_Y = {1: -20.0, 2: 0.0, 3: 20.0}     # 1 = 10 V, 2 = 7.4 V, 3 = 5 V
RAIL_X = -4.0                             # x of the regulator in each rail


def rail_places(n):
    """Placement for one buck rail, relative to the board. Returns {ref: (x, y, rot)}.

    Left to right: input ceramics, regulator, inductor, output ceramics. The feedback and
    compensation parts sit below the regulator on the quiet side, away from SW and L.
    """
    p = f'{n}0'
    y = RAIL_Y[n]
    x = RAIL_X
    return {
        # input ceramics, as close to the VIN pin as the package allows
        f'C{p}2': (x - 8.5, y - 3.2, 90),
        f'C{p}3': (x - 8.5, y + 0.2, 90),
        # the regulator; pin 1 (BST) top-left, SW on the right toward the inductor
        f'U{n+2}': (x, y - 1.5, 0),
        f'C{p}1': (x - 3.5, y - 5.5, 0),        # bootstrap, across BST and SW
        # inductor, immediately at SW
        f'L{n}': (x + 12.0, y - 1.5, 0),
        # output ceramics, past the inductor
        f'C{p}4': (x + 21.5, y - 4.0, 90),
        f'C{p}5': (x + 21.5, y - 0.5, 90),
        f'C{p}6': (x + 21.5, y + 3.0, 90),
        # feedback divider, close to FB, on the far side from SW
        f'R{p}1': (x + 4.0, y + 4.0, 0),
        f'R{p}2': (x + 1.0, y + 4.0, 0),
        # compensation and soft start, next to COMP/SS
        f'R{p}3': (x - 2.0, y + 6.5, 0),
        f'C{p}10': (x + 1.0, y + 6.5, 0),
        f'C{p}7': (x + 4.0, y + 6.5, 0),
        f'C{p}8': (x + 7.0, y + 6.5, 0),
        f'C{p}9': (x - 5.0, y + 6.5, 0),
        # enable network
        f'R{p}4': (x - 8.0, y + 4.0, 0),
        f'C{p}11': (x - 8.0, y + 6.5, 0),
        # rail-present LED, out at the edge where it can be seen
        f'D{n+1}': (x + 26.5, y - 4.0, 90),
        f'R{p}5': (x + 26.5, y + 0.0, 90),
    }


# Everything that is not part of a rail block.
FIXED = {
    # ---- input, left edge ----
    'J1':  (-38.0,  -8.0, 180),     # pack terminal block, wires leave -x
    'F1':  (-38.0,  12.0,   0),     # blade fuse
    'D1':  (-29.0,  20.0,  90),     # TVS to ground, right at the input
    'Q1':  (-27.0,  -8.0,   0),     # ideal-diode FET
    'U1':  (-27.0, -17.0,   0),     # LM74700
    'C1':  (-32.0, -20.0,   0),
    'C2':  (-21.5, -20.0,   0),
    'R1':  (-32.0, -14.0,   0),
    'J2':  (-31.0, -25.0,   0),     # e-stop header, top edge, inboard of the M3 hole
    # ---- shunt and telemetry ----
    'R2':  (-18.0,  -8.0,  90),     # 2512 shunt, in the VIN path
    'U2':  (-18.0, -17.0,   0),     # INA226 straight across it
    'C3':  (-12.0, -20.0,   0),
    'R3':  (-12.0, -14.0,   0),
    'R4':  ( -9.0, -14.0,   0),
    'J3':  ( -4.0, -28.0,   0),     # Qwiic, top edge
    # ---- bulk ----
    'C41': (-16.0,  10.0,   0),     # input bulk, near the FET and the rails
    'C42': ( 20.0,  10.0,   0),     # servo reservoir, on rail 2's output
    # ---- outputs, right edge ----
    'J4':  ( 38.0, -22.0, 180),     # barrel jack to the Jetson
    'F2':  ( 30.0,  -8.0,   0),     # lidar slow-blow
    'J5':  ( 38.0,  -2.0,   0),     # lidar terminal block
    'J6':  ( 38.0,  10.0,   0),     # servo out
    'J7':  ( 38.0,  16.0,   0),     # PPM in from the VESC
    'J8':  ( 30.0,  25.0,   0),     # 5 V out, inboard of the M3 hole
    # ---- mounting holes ----
    'H1':  (-HOLE_DX / 2, -HOLE_DY / 2, 0),
    'H2':  ( HOLE_DX / 2, -HOLE_DY / 2, 0),
    'H3':  (-HOLE_DX / 2,  HOLE_DY / 2, 0),
    'H4':  ( HOLE_DX / 2,  HOLE_DY / 2, 0),
}


def placement(solved=True):
    """The floorplan. FIXED + rail_places is the hand-written intent; layout_solved.json is
    that same plan after relax.py has separated the courtyards. Pass solved=False to see
    the seed (relax.py does, to report how far it moved things)."""
    places = dict(FIXED)
    for n in (1, 2, 3):
        places.update(rail_places(n))
    if solved:
        import os, json
        f = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'layout_solved.json')
        if os.path.exists(f):
            places.update({k: tuple(v) for k, v in json.load(open(f)).items()})
    return places


# ---- copper plan -------------------------------------------------------------------------
# 4 layers. The two inner layers are the reason this board can carry 8 A in 88 mm without
# heroic trace widths, and the reason the regulators can dump heat somewhere.
STACKUP = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
LAYER_PURPOSE = {
    'F.Cu':   'components, local power pours, short signal routing',
    'In1.Cu': 'solid ground. Unbroken under all three switch nodes: the return current for '
              'every switching loop flows here, directly under its own loop.',
    'In2.Cu': 'VIN distribution to the three regulators, plus output rail pours',
    'B.Cu':   'ground, output rail pours, and the few signals that cannot stay on top',
}
# Nets that get a filled zone rather than tracks, with the layer and (x0,y0,x1,y1) outline.
#
# Priority matters and getting it wrong cost 20 DRC shorts: where two zones of different
# nets overlap on the same layer, the higher priority wins and the lower one pulls back.
# The B.Cu ground pour covers the whole board, so every rail pour that sits on top of it
# must outrank it, and the same for VIN_PRE inside the VBAT pour.
#              net          layer      outline                                     priority
ZONES = [
    ('GND',      'In1.Cu', None,                                                    0),
    ('GND',      'B.Cu',   None,                                                    0),
    ('VIN',      'In2.Cu', (-44 + EDGE, -34 + EDGE, 16.0, 34 - EDGE),               0),
    ('VOUT_10V', 'B.Cu',   (14.0, -34 + EDGE, 44 - EDGE, -10.0),                   20),
    ('VOUT_7V4', 'B.Cu',   (14.0, -10.0, 44 - EDGE, 14.0),                         20),
    ('VOUT_5V',  'B.Cu',   (14.0, 14.0, 44 - EDGE, 34 - EDGE),                     20),
    ('VBAT',     'F.Cu',   (-44 + EDGE, -30.0, -22.0, 26.0),                        5),
    ('VIN_PRE',  'F.Cu',   (-24.0, -24.0, -14.0, 4.0),                             15),
    # top-side pours: a component pad only connects to copper on its own layer (or through
    # a via), so the planes underneath do nothing for the pads sitting on top of them.
    ('GND',      'F.Cu',   None,                                                     1),
    ('VIN',      'F.Cu',   (-16.0, -34 + EDGE, -6.0, 34 - EDGE),                     8),
]
# switch nodes get a compact F.Cu pour each: big enough for the current, small enough not
# to radiate. Bounded by the rail block so they cannot spread.
SW_ZONES = [(f'SW{n}', 'F.Cu', (RAIL_X - 1.0, RAIL_Y[n] - 6.0, RAIL_X + 13.0, RAIL_Y[n] + 2.0), 30)
            for n in (1, 2, 3)]
# each rail's output, poured on top around its own block, above the F.Cu ground
OUT_ZONES = [('VOUT_10V', 'F.Cu', (10.0, RAIL_Y[1] - 9.0, 34.0, RAIL_Y[1] + 9.0), 12),
             ('VOUT_7V4', 'F.Cu', (10.0, RAIL_Y[2] - 9.0, 34.0, RAIL_Y[2] + 9.0), 12),
             ('VOUT_5V',  'F.Cu', (10.0, RAIL_Y[3] - 9.0, 34.0, RAIL_Y[3] + 9.0), 12)]

TRACK_W = {'default': 0.25, 'signal': 0.25, 'power': 1.5, 'bulk': 3.0}
VIA = (0.6, 0.3)          # pad, drill


def checks():
    """Placement sanity that does not need KiCad: overlaps, edge keep-out, hole clearance."""
    import netlist as N
    out, ok = [], True
    parts, _ = N.build()
    places = placement()
    missing = [p.ref for p in parts if p.ref not in places]
    extra = [r for r in places if r not in {p.ref for p in parts}]
    out.append(f'{len(places)} placements for {len(parts)} parts')
    if missing:
        out.append(f'  FAIL no placement for: {missing}'); ok = False
    if extra:
        out.append(f'  FAIL placement for parts that do not exist: {extra}'); ok = False
    for ref, (x, y, _r) in places.items():
        if abs(x) > BOARD_W / 2 - EDGE or abs(y) > BOARD_H / 2 - EDGE:
            out.append(f'  FAIL {ref} at ({x}, {y}) is outside the {EDGE} mm edge keep-out'); ok = False
    # nothing may sit on a mounting hole
    for ref, (x, y, _r) in places.items():
        if ref.startswith('H'):
            continue
        for hx, hy in ((-HOLE_DX/2, -HOLE_DY/2), (HOLE_DX/2, -HOLE_DY/2),
                       (-HOLE_DX/2, HOLE_DY/2), (HOLE_DX/2, HOLE_DY/2)):
            if (x - hx) ** 2 + (y - hy) ** 2 < 3.5 ** 2:
                out.append(f'  FAIL {ref} sits on the M3 hole at ({hx}, {hy})'); ok = False
    return ok, '\n'.join(out)


if __name__ == '__main__':
    ok, text = checks()
    print(text)
    print('PLACEMENT CHECKS PASS' if ok else 'PLACEMENT CHECKS FAILED')
    raise SystemExit(0 if ok else 1)
