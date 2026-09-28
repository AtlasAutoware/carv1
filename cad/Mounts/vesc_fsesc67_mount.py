"""Snap-in tray for the Flipsky FSESC 6.7 PRO in its aluminium case, on the clipless
mounting system.

Case: 95 x 92 x 24.5 mm (FSESC 6.7 PRO manual "FSESC_6.7_PRO_-_a_b_compressed.pdf" linked
from the flipsky.net product page; the page body still carries the older 60 A version's
100 x 92 x 22.5 mm and 380 g, see the MD). Battery XT90 lead and the micro-USB come out
of one end, the three 12 AWG phase wires out of the opposite end, the anti-spark button
is on one long side, and every signal connector (PPM, CAN, COMM/UART, SWD, SENSE) is
reached through the window in the lid, not through the sides. The case has four lid
screws on its top corners and no mounting holes that Flipsky documents; the boss pattern
below is an ESTIMATE on the lid-screw pattern (see the MD) and is only useful if the case
bottom turns out to be tapped.

Layout, case centre at the origin: +x is the XT90/battery end, -x the phase-wire end,
button on the -y side. The case stands 5 mm above the 4 mm floor on two 9 mm foot pads
(the foot flanges are flush with the pad tops, so the case traps the feet) and four
corner bosses; the floor is mostly windows and both ends are open, so air moves under
and past the hottest part on the car. Two 2 mm fingers with 1 mm lips on the long sides
snap over the case top edge; four corner stubs locate it in x and y; a zip-tie tab at
each end takes the strain off the leads. Two feet along the car.

    python3 vesc_fsesc67_mount.py
    CASE_L=100 CASE_H=22.5 PEG_PITCH=50 python3 vesc_fsesc67_mount.py
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import foot, foot_cutout, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP
from render import render
from jetson_orin_nano_mount import lines_png, box, inter

# ---- component (FSESC 6.7 PRO manual, flipsky.net) ------------------------------------
CASE_L = float(os.environ.get('CASE_L', 95.0))      # x, along the car (XT90 end to motor end)
CASE_W = float(os.environ.get('CASE_W', 92.0))      # y
CASE_H = float(os.environ.get('CASE_H', 24.5))      # z; the finger lips depend on this
CASE_MASS = 380.0                                   # g, older listing, ESTIMATE for the PRO
HOLE_INSET = float(os.environ.get('HOLE_INSET', 4.0))   # ESTIMATE: lid screws ~4 mm in from
                                                        # each edge (measured off the photo)
BUTTON_X, BUTTON_D, BUTTON_PROUD = -14.0, 12.0, 6.0     # ESTIMATE: anti-spark button on -y
XT90 = (22.0, 16.0, 18.0)                                # ESTIMATE: XT90 female + wire boot
PHASE_SPREAD = 36.0                                      # ESTIMATE: 3 phase wires across

# ---- tray -----------------------------------------------------------------------------
CLR = 0.5
FLOOR = 4.0
WALL = 3.0
PAD_H = 9.0                     # foot pads: the case stands on these, 5 mm above the floor
PAD_W = 38.0                    # along x (bar rule: >= 36 of solid floor under a foot)
PAD_D = 44.0                    # across
BOSS_OD, INSERT_D, INSERT_DEPTH = 10.0, 4.0, 5.0     # M3 heat-set
STUB_LEG, STUB_H = 8.0, 8.0     # corner stubs above the pad tops
FINGER_T = 2.0                  # firm: the case is metal and 380 g
FINGER_W = 14.0
FINGER_X = float(os.environ.get('FINGER_X', 18.0))   # toward the XT90 end, clear of the button
LIP = 1.0
LIP_CLR = 0.3
LIP_T = 2.0
TAB_L, TAB_W = 12.0, 34.0       # zip-tie tabs at both ends
TIE_SLOT = (2.6, 6.0)           # x, y for a 4.8 mm cable tie
PEG_PITCH = float(os.environ.get('PEG_PITCH', 49.0))

# RETAIN='snap': under-board variant (snap_retain.py). The two foot pads become one
# 40.5 mm wide, 9 mm tall bar along x carrying the dovetail slide channel + snap tongues
# of rplidar_c1_mount.py, open at the phase-lead end; the case still stands on the bar
# top and the bosses, so the depth below the plate is unchanged. The zip-tie tabs widen
# to TAB_W_SNAP so their slots sit outside the channel band.
RETAIN = os.environ.get('RETAIN', 'recess')
TAB_W_SNAP = 54.0
TIE_Y = 9.0                     # tie slots either side of the tab centre
SUFFIX = ('_under' if RETAIN == 'snap' else '') + os.environ.get('OUT_SUFFIX', '')
if RETAIN == 'snap':
    TAB_W = TAB_W_SNAP
    TIE_Y = 24.25               # outside the 40.5 mm channel band

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT, exist_ok=True)

IX, IY = CASE_L + 2 * CLR, CASE_W + 2 * CLR
OL, OW = IX + 2 * WALL, IY + 2 * WALL
Z_CASE = PAD_H
Z_LIP = Z_CASE + CASE_H + LIP_CLR
Z_TOP = Z_LIP + LIP_T
FEET = [(-PEG_PITCH / 2, 0.0), (PEG_PITCH / 2, 0.0)]
BOSSES = [(sx * (CASE_L / 2 - HOLE_INSET), sy * (CASE_W / 2 - HOLE_INSET)) for sx in (-1, 1) for sy in (-1, 1)]


def tray():
    t = cq.Workplane('XY').rect(OL, OW).extrude(FLOOR).edges('|Z').fillet(4)
    # open the floor inside a 4 mm rim, then put back two full-width bars under the feet
    rim = 4.0
    t = t.cut(box(-OL / 2 + rim, OL / 2 - rim, -OW / 2 + rim, OW / 2 - rim, -1, FLOOR + 1))
    for (fx, _) in FEET:
        t = t.union(box(fx - PAD_W / 2, fx + PAD_W / 2, -OW / 2 + 1, OW / 2 - 1, 0, FLOOR))
        # windows through the bars outside the pads (keeps 4 mm strips to the rim)
        for sy in (-1, 1):
            ya, yb = sorted((sy * (PAD_D / 2 + 2), sy * (OW / 2 - rim)))
            t = t.cut(box(fx - PAD_W / 2 + 4, fx + PAD_W / 2 - 4, ya, yb, -1, FLOOR + 1)
                      .edges('|Z').fillet(1.5))
        # raised pad the case stands on, foot recess in its top
        t = t.union(box(fx - PAD_W / 2, fx + PAD_W / 2, -PAD_D / 2, PAD_D / 2, 0, PAD_H)
                    .edges('|Z').fillet(2))
        if RETAIN != 'snap':
            t = t.cut(foot_cutout(PAD_H, 'x').translate((fx, 0, 0)))
    # corner bosses on the (estimated) lid-screw pattern, tops flush with the pads
    for (x, y) in BOSSES:
        t = t.union(cq.Workplane('XY').center(x, y).circle(BOSS_OD / 2).extrude(PAD_H))
        t = t.cut(cq.Workplane('XY').workplane(offset=PAD_H - INSERT_DEPTH).center(x, y)
                  .circle(INSERT_D / 2).extrude(INSERT_DEPTH + 1))
    # corner stubs: L of 3 mm walls outside the case, locate it in x and y, ends stay open
    for sx in (-1, 1):
        for sy in (-1, 1):
            xa, xb = sorted((sx * IX / 2, sx * (IX / 2 + WALL)))
            ya, yb = sorted((sy * (IY / 2 - STUB_LEG), sy * (IY / 2 + WALL)))
            t = t.union(box(xa, xb, ya, yb, 0, Z_CASE + STUB_H))
            xa, xb = sorted((sx * (IX / 2 - STUB_LEG), sx * (IX / 2 + WALL)))
            ya, yb = sorted((sy * IY / 2, sy * (IY / 2 + WALL)))
            t = t.union(box(xa, xb, ya, yb, 0, Z_CASE + STUB_H))
    # snap fingers on the long sides, lips over the case top edge
    for sy in (-1, 1):
        ya, yb = sorted((sy * IY / 2, sy * (IY / 2 + FINGER_T)))
        t = t.union(box(FINGER_X - FINGER_W / 2, FINGER_X + FINGER_W / 2, ya, yb, 0, Z_TOP))
        yi = sy * (IY / 2 + 0.01)
        prof = [(yi, Z_LIP), (yi - sy * LIP, Z_LIP + LIP), (yi - sy * LIP, Z_TOP - 0.5), (yi, Z_TOP)]
        lip = (cq.Workplane('YZ', origin=(FINGER_X - FINGER_W / 2, 0, 0)).polyline(prof).close()
               .extrude(FINGER_W))
        t = t.union(lip)
    # zip-tie tabs at both ends
    for sx in (-1, 1):
        xa, xb = sorted((sx * (OL / 2 - 1), sx * (OL / 2 + TAB_L)))
        tab = box(xa, xb, -TAB_W / 2, TAB_W / 2, 0, FLOOR)
        tab = tab.edges('|Z').edges('>X' if sx > 0 else '<X').fillet(3)
        t = t.union(tab)
        for sy in (-1, 1):
            t = t.cut(box(sx * (OL / 2 + TAB_L / 2) - TIE_SLOT[0] / 2, sx * (OL / 2 + TAB_L / 2) + TIE_SLOT[0] / 2,
                          sy * TIE_Y - TIE_SLOT[1] / 2, sy * TIE_Y + TIE_SLOT[1] / 2, -1, FLOOR + 1))
    if RETAIN == 'snap':
        # one bar joins the pads and carries the slide channel, open at the -x (phase) end;
        # cut last so the tabs and rim do not refill the channel
        from snap_retain import channel_solids
        bar, cuts, adds = channel_solids(FEET, 'x', PAD_H, extent=(-OL / 2 - TAB_L, OL / 2 - 1))
        t = t.union(bar)
        for c in cuts:
            t = t.cut(c)
        for a in adds:
            t = t.union(a)
    return t


def feet():
    if RETAIN == 'snap':
        from snap_retain import snap_feet
        return snap_feet(FEET, 'x')
    return [foot('x').translate((x, y, PAD_H)) for (x, y) in FEET]


def envelope():
    """Case where it sits, XT90 lead at +x, phase wires at -x, button on -y."""
    z0 = Z_CASE
    e = (cq.Workplane('XY').workplane(offset=z0).rect(CASE_L, CASE_W).extrude(CASE_H)
         .edges('|Z').fillet(2))
    e = e.union(box(CASE_L / 2 - 0.01, CASE_L / 2 + 25, -XT90[1] / 2, XT90[1] / 2,
                    z0 + CASE_H / 2 - XT90[2] / 2, z0 + CASE_H / 2 + XT90[2] / 2))
    e = e.union(box(-CASE_L / 2 - 30, -CASE_L / 2 + 0.01, -PHASE_SPREAD / 2, PHASE_SPREAD / 2,
                    z0 + CASE_H / 2 - 3, z0 + CASE_H / 2 + 3))
    e = e.union(cq.Workplane('XZ', origin=(BUTTON_X, -CASE_W / 2 + 0.01, z0 + CASE_H / 2))
                .circle(BUTTON_D / 2).extrude(BUTTON_PROUD))
    return e


def _pieces(dz):
    return [clipless_piece().translate((x, y, dz)) for (x, y) in FEET]


def assembly(t):
    if RETAIN == 'snap':
        from snap_retain import under_assembly
        return under_assembly('vesc_fsesc67_under_clipless', t, feet(), FEET, envelope(),
                              plate_stub(FEET, size=(OL + 2 * TAB_L + 20, OW + 20)))
    a = cq.Assembly(name='vesc_fsesc67_on_clipless')
    a.add(plate_stub(FEET, size=(OL + 2 * TAB_L + 20, OW + 20)), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, p in enumerate(_pieces(-(RIM_TOP - RIM_PROUD))):
        a.add(p, name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(t.translate((0, 0, RIM_PROUD)), name='tray', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='fsesc67_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def checks(t):
    fs = feet()
    env = envelope()
    if RETAIN == 'snap':
        from snap_retain import hang, piece_from_above, play_check
        stub = plate_stub(FEET, size=(OL + 2 * TAB_L + 20, OW + 20))
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
            'hanging_depth_below_plate_top': round(-hang(t).val().BoundingBox().zmin, 2),
            'case_underside_z': Z_CASE,
            'pocket': (IX, IY),
        }
    stub = plate_stub(FEET, size=(OL + 2 * TAB_L + 20, OW + 20)).translate((0, 0, -RIM_PROUD))
    pieces = _pieces(-RIM_TOP)
    return {
        'solids': len(t.val().Solids()),
        'tray_x_feet': sum(inter(t, f) for f in fs),
        'feet_x_clipless': sum(inter(f, p) for f in fs for p in pieces),
        'tray_x_plate': inter(t, stub),
        'tray_x_envelope': inter(t, env),
        'feet_x_envelope': sum(inter(f, env) for f in fs),
        'case_underside_z': Z_CASE,
        'lip_underside_z': Z_LIP,
        'case_top_z': Z_CASE + CASE_H,
        'pocket': (IX, IY),
        'bosses': BOSSES,
    }


if __name__ == '__main__':
    t = tray()
    bb = t.val().BoundingBox()
    N = 'vesc_fsesc67' + SUFFIX
    cq.exporters.export(t, os.path.join(OUT, N + '.step'))
    cq.exporters.export(t, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    assembly(t).save(os.path.join(OUT, N + '_assembly.step'))
    vol = t.val().Volume()
    print(f'tray {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'({vol/1000*1.24:.0f} g PLA solid, ~{vol/1000*1.24*0.6:.0f} g at 60% effective), '
          f'case {CASE_L} x {CASE_W} x {CASE_H}, peg pitch {PEG_PITCH} mm, retention {RETAIN}')
    for k, v in checks(t).items():
        print(f'  {k}: {v}')
    render([(t, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title=f'FSESC 6.7 PRO tray ({RETAIN})')
    fs = feet()
    stub = plate_stub(FEET, size=(OL + 2 * TAB_L + 20, OW + 20))
    if RETAIN == 'snap':
        from snap_retain import under_shaded
        shaded = under_shaded(t, fs, FEET, envelope(), stub)
        title, el, eye = 'FSESC 6.7 PRO tray hanging under the plate (grey = case envelope)', -25, (0.57, -0.82, -0.47)
    else:
        pieces = _pieces(-(RIM_TOP - RIM_PROUD))
        up = lambda s: s.translate((0, 0, RIM_PROUD))
        shaded = [(stub, (0.55, 0.55, 0.6))] + [(p, (0.63, 0.63, 0.63)) for p in pieces] + \
                 [(up(t), (0.9, 0.45, 0.1))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + \
                 [(up(envelope()), (0.35, 0.35, 0.35))]
        title, el, eye = 'FSESC 6.7 PRO tray on clipless (grey = case envelope)', 28, (0.57, -0.82, 0.47)
    render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(el, -55), (el, 125)], title=title)
    lines_png([q for q, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'), eye=eye)
    lines_png([t], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
