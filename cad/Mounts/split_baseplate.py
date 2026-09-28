"""Baseplate v2 cut in two so each half fits the P1S bed, with a finger joint to bond.

The plate is 452 x 177.5 and the bed is 256, so a half is only short enough if the cut is
within 30 mm of the plate centre. Inside that window almost everything is occupied by the
clipless hole columns and the zip-tie holes; the widest clear band is 2.8 mm, so the cut
line is a straight one at CUT (default 10.6) rather than anything shaped.

Strength comes from fingers running along the cut instead. Rows A-D put clipless holes at
y +-20.5 and +-61.5, so the bands |y| < 6.25 and 34.75 < |y| < 47.25 are clear of holes at
every x, and three fingers live there: one on the centreline and one each side. They reach
FINGER_L either way from the cut, which turns 563 mm2 of end grain into 2400 mm2 of lap and
takes the in-plane shear. Two 3 mm dowel holes locate the halves while the epoxy sets.

    python3 split_baseplate.py
    CUT=10.6 FINGER_L=0 python3 split_baseplate.py     a plain butt cut

This is a mock-up plate, not a structural one. See the deflection figures it prints.
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import baseplate_v2 as B

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
BED = 256.0
CUT = float(os.environ.get('CUT', -20.6))   # the fingers add FINGER_L to the front half, so the cut has to sit forward of +8
T = B.PLATE_T
FINGER_L = float(os.environ.get('FINGER_L', 20.0))
FINGERS = [(0.0, 12.0), (41.0, 10.0), (-41.0, 10.0)]     # (y centre, width)
JCLR = float(os.environ.get('JCLR', 0.15))               # per face, between finger and slot
DOWEL_D, DOWEL = 3.2, [(CUT - FINGER_L - 8, 0.0), (CUT + FINGER_L + 8, 0.0)]


def halves():
    p = B.plate_solid(T)                                  # top face at z = 0
    box = lambda x0, x1, y0, y1: (cq.Workplane('XY').box(x1 - x0, y1 - y0, T + 4)
                                  .translate(((x0 + x1) / 2, (y0 + y1) / 2, -T / 2)))
    front = p.cut(box(CUT, 1e3, -1e3, 1e3))               # x < CUT
    rear = p.cut(box(-1e3, CUT, -1e3, 1e3))               # x > CUT
    for (yc, w) in FINGERS:
        # the finger belongs to the front half and reaches into the rear one
        finger = box(CUT, CUT + FINGER_L, yc - w / 2, yc + w / 2)
        slot = box(CUT - 0.01, CUT + FINGER_L + JCLR, yc - w / 2 - JCLR, yc + w / 2 + JCLR)
        front = front.union(finger.intersect(p.val() and p))
        rear = rear.cut(slot)
    for (x, y) in DOWEL:
        d = cq.Workplane('XY').center(x, y).circle(DOWEL_D / 2).extrude(-T - 2).translate((0, 0, 1))
        front, rear = front.cut(d), rear.cut(d)
    return front, rear


def report(front, rear):
    out = []
    for name, s in (('front', front), ('rear', rear)):
        bb = s.val().BoundingBox()
        fits = bb.xlen < BED - 2 and bb.ylen < BED - 2
        out.append(f'  {name:5s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:.2f} mm  '
                   f'{s.val().Volume()/1000:5.1f} cm3  {s.val().Volume()/1000*1.24:5.0f} g solid PLA  '
                   f'{"fits" if fits else "DOES NOT FIT THE BED"}  solids={len(s.val().Solids())}')
    return '\n'.join(out)


def deflection(t=T, E=3500.0, load_N=17.0):
    """Mid-span sag of the deck between the two shock-tower clamps, simply supported, UDL.

    Second moment from the real section: the plate is 177.5 wide but rows A-D remove four
    28.5 mm holes across most columns, so the worst section carries 177.5 - 4*28.5 = 63.5 mm
    of material. PLA's modulus is about 3.5 GPa when new; it creeps under sustained load and
    softens well before its glass transition, so treat this as the best case.
    """
    xs = sorted(b[0] for b in B.BRACKETS) + sorted(b[1] for b in B.BRACKETS)
    L = max(xs) - min(xs)
    res = []
    for label, b in (('solid section', B.W_PLATE), ('through a hole row', B.W_PLATE - 4 * B.HOLE)):
        I = b * t ** 3 / 12.0
        w = load_N / L
        d = 5 * w * L ** 4 / (384.0 * E * I)
        res.append(f'  {label:22s} b={b:5.1f} I={I:7.0f} mm4  sag {d:6.1f} mm')
    return f'span between the shock-tower clamps {L:.0f} mm, {load_N:.0f} N of electronics\n' + '\n'.join(res)


if __name__ == '__main__':
    f, r = halves()
    for n, s in (('baseplate_v2_front', f), ('baseplate_v2_rear', r)):
        cq.exporters.export(s, os.path.join(OUT, n + '.stl'), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(s, os.path.join(OUT, n + '.step'))
    print(f'cut at x = {CUT}, plate {T} mm, {len(FINGERS)} fingers {FINGER_L} mm long, {JCLR} mm clearance')
    print(report(f, r))
    print('\n' + deflection())
    print(f'\n  ... at {2*T:.1f} mm:')
    print(deflection(t=2 * T))
    # the joint must not eat a clipless hole or a mount's footprint
    bad = [k for k, (x, y) in B.HOLES.items() if abs(x - CUT) < B.HOLE / 2 + 1]
    print(f'\nclipless holes within 1 mm of the cut: {bad or "none"}')
