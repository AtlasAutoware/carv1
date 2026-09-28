"""Retention frame for the SMC Racing HCL-EC 14.8 V 9000 mAh 100C pack (product code
90100-4S1P, "G10 Protection Plates") carried on top of the main board, on the clipless
mounting system.

Pack: 47 x 50 x 160 mm, 785 g, 10 AWG leads (smc-racing.com product page, see the MD).
SMC lists 47 and 50 without saying which is the width; the pocket is sized for 50 wide
and the posts for 47 tall, so the pack fits either way (1.5 mm side play if it is 47 wide,
which the straps take up). LIPO_L / LIPO_W / LIPO_H are env vars.

The frame is a 4 mm floor with four L-shaped corner posts, two 25 mm hook-and-loop strap
slots per side (the straps are what actually hold 785 g on a car that brakes hard; the
posts only locate), the XT90 lead end (+x) open, and FOUR feet on a 2 x 2 grid,
PEG_PITCH along the car by PITCH_Y across. The floor is a cross: a 68 mm centre strip
under the pack and two 40 mm transverse bars, one under each foot pair, so every foot
sits on 40 x 84 mm of solid floor. The pack lies on the foot flanges and traps them.

    python3 lipo_smc_9000_mount.py
    PEG_PITCH=50 PITCH_Y=47.5 LIPO_W=47 python3 lipo_smc_9000_mount.py
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import foot, foot_cutout, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP, FOOT_FLANGE, FOOT_CLR
from render import render
from jetson_orin_nano_mount import lines_png, box, inter

# ---- component (smc-racing.com, product 90100-4S1P) ----------------------------------
LIPO_L = float(os.environ.get('LIPO_L', 160.0))     # x, along the car; leads out of +x
LIPO_W = float(os.environ.get('LIPO_W', 50.0))      # y (SMC: "47mm x 50mm x 160mm")
LIPO_H = float(os.environ.get('LIPO_H', 47.0))      # z
LIPO_MASS = 785.0                                   # g
XT90 = (22.0, 16.0, 12.0)                           # ESTIMATE: XT90 on 10 AWG leads at the +x end

# ---- frame ----------------------------------------------------------------------------
CLR = 0.5
FLOOR = 4.0
WALL = 3.0
POST_H = 22.0                   # above the floor; straps do the holding
POST_LEG_X, POST_LEG_Y = 20.0, 14.0
STRAP_W, STRAP_T = 26.0, 4.5    # 25 mm hook-and-loop
STRIP_W = 68.0                  # centre floor strip across (pocket + strap slots + walls)
BAR_L = 40.0                    # transverse foot bars along x (rule: >= 36)
PEG_PITCH = float(os.environ.get('PEG_PITCH', 49.0))
PITCH_Y = float(os.environ.get('PITCH_Y', 47.0))

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT, exist_ok=True)

IX, IY = LIPO_L + 2 * CLR, LIPO_W + 2 * CLR
OL = IX + 2 * WALL
BAR_W = PITCH_Y + FOOT_FLANGE[1] + 2 * FOOT_CLR + 2 * WALL   # 87.5 across
FEET = [(sx * PEG_PITCH / 2, sy * PITCH_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]
SLOT_X = PEG_PITCH / 2 + BAR_L / 2 + STRAP_W / 2 + 3          # strap slots just beyond the bars
SLOT_Y = IY / 2 + STRAP_T / 2 + 1.0
Z_TOP = FLOOR + POST_H


def frame():
    f = box(-OL / 2, OL / 2, -STRIP_W / 2, STRIP_W / 2, 0, FLOOR).edges('|Z').fillet(4)
    for sx in (-1, 1):
        bar = box(sx * PEG_PITCH / 2 - BAR_L / 2, sx * PEG_PITCH / 2 + BAR_L / 2, -BAR_W / 2, BAR_W / 2, 0, FLOOR)
        f = f.union(bar.edges('|Z').fillet(4))
    # lightening windows in the strip beyond the strap slots, and between the bars
    for sx in (-1, 1):
        xa, xb = sorted((sx * (SLOT_X + STRAP_W / 2 + 4), sx * (OL / 2 - 8)))
        if xb - xa > 10:
            f = f.cut(box(xa, xb, -IY / 2 + 6, IY / 2 - 6, -1, FLOOR + 1).edges('|Z').fillet(3))
    mid = PEG_PITCH / 2 - BAR_L / 2 - 4
    if mid > 4:
        f = f.cut(box(-mid, mid, -IY / 2 + 6, IY / 2 - 6, -1, FLOOR + 1).edges('|Z').fillet(2))
    # corner posts, L-shaped, outside the pack
    for sx in (-1, 1):
        for sy in (-1, 1):
            xa, xb = sorted((sx * IX / 2, sx * (IX / 2 + WALL)))
            ya, yb = sorted((sy * (IY / 2 - POST_LEG_Y), sy * (IY / 2 + WALL)))
            f = f.union(box(xa, xb, ya, yb, 0, Z_TOP))
            xa, xb = sorted((sx * (IX / 2 - POST_LEG_X), sx * (IX / 2 + WALL)))
            ya, yb = sorted((sy * IY / 2, sy * (IY / 2 + WALL)))
            f = f.union(box(xa, xb, ya, yb, 0, Z_TOP))
    # strap slots: two straps across the pack, each through the floor on both sides
    for sx in (-1, 1):
        for sy in (-1, 1):
            f = f.cut(box(sx * SLOT_X - STRAP_W / 2, sx * SLOT_X + STRAP_W / 2,
                          sy * SLOT_Y - STRAP_T / 2, sy * SLOT_Y + STRAP_T / 2, -1, FLOOR + 1))
    # four feet
    for (x, y) in FEET:
        f = f.cut(foot_cutout(FLOOR, 'x').translate((x, y, 0)))
    return f


def feet():
    return [foot('x').translate((x, y, FLOOR)) for (x, y) in FEET]


def envelope():
    e = (cq.Workplane('XY').workplane(offset=FLOOR).rect(LIPO_L, LIPO_W).extrude(LIPO_H)
         .edges('|Z').fillet(3))
    e = e.union(box(LIPO_L / 2 - 0.01, LIPO_L / 2 + 30, -XT90[1] / 2, XT90[1] / 2,
                    FLOOR + 6, FLOOR + 6 + XT90[2]))
    return e


def _pieces(dz):
    return [clipless_piece().translate((x, y, dz)) for (x, y) in FEET]


def _stub():
    return plate_stub(FEET, size=(OL + 20, BAR_W + 20))


def assembly(f):
    a = cq.Assembly(name='lipo_smc_9000_on_clipless')
    a.add(_stub(), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, p in enumerate(_pieces(-(RIM_TOP - RIM_PROUD))):
        a.add(p, name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(f.translate((0, 0, RIM_PROUD)), name='frame', color=cq.Color(0.9, 0.45, 0.1))
    for i, ft in enumerate(feet()):
        a.add(ft.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='lipo_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def foot_pad_check(f):
    """Solid floor around each foot: the 40 x 87.5 bar minus the recess, and the margin
    between the flange recess and the bar edges."""
    fx = FOOT_FLANGE[0] + 2 * FOOT_CLR
    fy = FOOT_FLANGE[1] + 2 * FOOT_CLR
    res = {}
    for i, (x, y) in enumerate(FEET):
        pad = box(x - BAR_L / 2, x + BAR_L / 2, y - fy / 2 - WALL, y + fy / 2 + WALL, 0, FLOOR)
        cut = foot_cutout(FLOOR, 'x').translate((x, y, 0))
        full = pad.val().Volume() - inter(cut, pad)      # what a solid bar with the recess holds
        res[f'foot{i}_pad_solid_fraction'] = round(inter(f, pad) / full, 4)
    res['pad_margin_x'] = (BAR_L - fx) / 2
    res['pad_margin_y'] = (BAR_W - PITCH_Y - fy) / 2
    return res


def checks(f):
    fs = feet()
    env = envelope()
    stub = _stub().translate((0, 0, -RIM_PROUD))
    pieces = _pieces(-RIM_TOP)
    res = {
        'solids': len(f.val().Solids()),
        'frame_x_feet': sum(inter(f, ft) for ft in fs),
        'feet_x_clipless': sum(inter(ft, p) for ft in fs for p in pieces),
        'frame_x_plate': inter(f, stub),
        'frame_x_envelope': inter(f, env),
        'feet_x_envelope': sum(inter(ft, env) for ft in fs),
        'pocket': (IX, IY),
        'post_top_z': Z_TOP,
        'pack_top_z': FLOOR + LIPO_H,
        'feet': FEET,
        'strap_slots_x': (-SLOT_X, SLOT_X),
        'flange_under_pack_mm': LIPO_W / 2 - (PITCH_Y / 2 - FOOT_FLANGE[1] / 2),
    }
    res.update(foot_pad_check(f))
    return res


if __name__ == '__main__':
    f = frame()
    bb = f.val().BoundingBox()
    N = 'lipo_smc_9000' + os.environ.get('OUT_SUFFIX', '')
    cq.exporters.export(f, os.path.join(OUT, N + '.step'))
    cq.exporters.export(f, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    assembly(f).save(os.path.join(OUT, N + '_assembly.step'))
    vol = f.val().Volume()
    print(f'frame {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'({vol/1000*1.24:.0f} g PLA solid, ~{vol/1000*1.24*0.6:.0f} g at 60% effective), '
          f'pack {LIPO_L} x {LIPO_W} x {LIPO_H}, feet on {PEG_PITCH} x {PITCH_Y} mm')
    for k, v in checks(f).items():
        print(f'  {k}: {v}')
    render([(f, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title='SMC 9000 mAh 4S retention frame')
    fs = feet()
    pieces = _pieces(-(RIM_TOP - RIM_PROUD))
    stub = _stub()
    up = lambda s: s.translate((0, 0, RIM_PROUD))
    shaded = [(stub, (0.55, 0.55, 0.6))] + [(p, (0.63, 0.63, 0.63)) for p in pieces] + \
             [(up(f), (0.9, 0.45, 0.1))] + [(up(ft), (0.2, 0.5, 0.9)) for ft in fs] + \
             [(up(envelope()), (0.35, 0.35, 0.35))]
    render(shaded, os.path.join(OUT, N + '_assembly.png'),
           views=[(28, -55), (28, 125)], title='SMC 9000 frame on clipless (grey = pack envelope)')
    lines_png([stub] + pieces + [up(f)] + [up(ft) for ft in fs] + [up(envelope())],
              os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'))
    lines_png([f], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
