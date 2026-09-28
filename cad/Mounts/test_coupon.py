"""A corner of Baseplate v2 that fits the P1S bed, for test-fitting the mounts before the
real plate is cut, plus a print plate of twelve clipless mounting pieces.

The real plate is 452 x 177.5 mm and the bed is 256 x 256, so the full plate cannot be
printed. This coupon carries a 4 x 4 patch of the v2 grid (40 along the car, 41 across) at
the real 3.175 mm thickness, which is enough to seat any single mount and to check a foot
pair, the piece fit and the 1.13 mm rim stand-off. It is NOT the plate: no body-post
cutouts, no shock-tower clamps, and PLA at 3.175 mm is far too floppy to carry the car.

    python3 test_coupon.py
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import clipless_piece, HOLE
import baseplate_v2 as BV2

# clipless.PLATE_T is 2.0, the thickness the Templates were drawn for; the Baseplate STL
# measures 3.175 and board_layout.py uses that. The rim is 3.13 tall, so on a 2 mm plate it
# stands 1.13 proud and the mount rests on the rim, while on a 3.175 plate it finishes
# 0.045 BELOW the plate top and the mount rests on the plate instead. Nothing jams either
# way (the peg still has 14.0 of pocket under the plate top against its 12.0 depth) but the
# mount sits 1.13 lower. Print the coupon at whatever v2 will actually be cut from.
T = float(os.environ.get('COUPON_T', 3.175))

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
PX, PY = BV2.PITCH, BV2.PITCH_Y
NX, NY = 4, 4
L, W = (NX - 1) * PX + 60, (NY - 1) * PY + 50


def coupon():
    p = cq.Workplane('XY').rect(L, W).extrude(T).edges('|Z').fillet(6)
    for i in range(NX):
        for j in range(NY):
            x = (i - (NX - 1) / 2) * PX
            y = (j - (NY - 1) / 2) * PY
            p = p.cut(cq.Workplane('XY').center(x, y).rect(HOLE, HOLE).extrude(T + 2).translate((0, 0, -1)))
    return p


def piece_plate(n=12, gap=3.0):
    """n clipless pieces laid out flange-down for one print."""
    a = cq.Assembly()
    cols = 4
    for k in range(n):
        cx = (k % cols - (cols - 1) / 2) * (33 + gap)
        cy = (k // cols - ((n + cols - 1) // cols - 1) / 2) * (33 + gap)
        a.add(clipless_piece().translate((cx, cy, 0)), name=f'piece_{k}')
    return a


if __name__ == '__main__':
    c = coupon()
    bb = c.val().BoundingBox()
    cq.exporters.export(c, os.path.join(OUT, 'baseplate_v2_test_coupon.stl'), tolerance=0.02, angularTolerance=0.1)
    print(f'coupon {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.2f} mm, {NX}x{NY} holes at {PX} x {PY}, '
          f'{c.val().Volume()/1000:.1f} cm3')
    assert bb.xlen < 250 and bb.ylen < 250, 'coupon does not fit the P1S bed'
    p = piece_plate()
    cq.exporters.export(p.toCompound(), os.path.join(OUT, 'clipless_piece_x12.stl'), tolerance=0.02, angularTolerance=0.1)
    pb = p.toCompound().BoundingBox()
    print(f'12 clipless pieces {pb.xlen:.1f} x {pb.ylen:.1f} x {pb.zlen:.1f} mm, '
          f'{p.toCompound().Volume()/1000:.1f} cm3 total')
    assert pb.xlen < 250 and pb.ylen < 250, 'piece plate does not fit'
    one = clipless_piece()
    cq.exporters.export(one, os.path.join(OUT, 'clipless_piece.stl'), tolerance=0.02, angularTolerance=0.1)
    print(f'one piece {one.val().Volume()/1000:.2f} cm3')
