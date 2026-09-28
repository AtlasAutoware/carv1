"""Mast for the Orbbec Gemini 335 depth camera on the clipless mounting system, replacing
the wooden top plate.

Gemini 335 (Gemini 330 Series Datasheet v1.1, section 3.2.1 drawings): 89.46 x 25.00 x
30.00 mm (width x height x depth), 97 g. One 1/4-20 UNC socket in the underside (max
insertion 5 mm), two M3 sockets in the REAR face 45.00 mm apart (max insertion 3 mm),
USB Type-C on one end face with two M2 points for a locking cable.

Layout: four 6 x 6 legs rise from a floor bar on two clipless feet ACROSS the car
(PITCH_Y, default 47) and converge, in both x and y, onto a horizontal drum (axis along
y, R 20) at the top: a four-leg pyramid, so it is triangulated fore-aft and laterally.
The drum's top is a cylindrical rack of 5 degree teeth. The camera head is a saddle with
the matching toothed concave underside; the camera's own 1/4-20 bolt goes up through an
arc slot in the drum shell and the saddle into the camera and clamps the whole stack, so
the pitch is locked at one of the 5 degree tooth positions and cannot drift. A rear tab on
the saddle takes the two M3 screws, which fixes the camera's yaw and fore-aft position.
The lens axis ends up CAM_HEIGHT (150) above the board top at 0 degree pitch.

The feet channel is the dovetail slide of rplidar_c1_mount.slide_channel, open on the +y
side; the mast slides on sideways over both flanges and snaps.

The mast stands BEHIND the RPLIDAR C1 (LIDAR_X, LIDAR_Y: lidar axis relative to the mast's
feet-pair centre; v2 default one column ahead, half a row right) and the camera looks
forward over it. checks() reports the four legs' sectors in the lidar's scan plane (they
must all be in the rear half) and the pitch at which the lidar top would enter the bottom
of the camera image.

    python3 camera_mast_mount.py
    CAM_HEIGHT=140 PITCH_Y=45.4 python3 camera_mast_mount.py
    LIDAR_X=60 LIDAR_Y=-20.5 python3 camera_mast_mount.py    # the v2 layout's relative position
"""
import os, sys, math
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import foot, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP, FOOT_FLANGE, FOOT_CLR
from render import render
from jetson_orin_nano_mount import lines_png, box, inter
from rplidar_c1_mount import slide_channel, snap_tongues, CH_LIP_Z0, CH_LIP_Z1

# ---- component (Orbbec Gemini 330 Series Datasheet v1.1) ------------------------------
CAM = (89.46, 25.0, 30.0)         # width (y), height (z), depth (x, front face to fins)
CAM_MASS = 97.0
CAM_TRIPOD_FROM_REAR = 11.0       # ESTIMATE from the bottom-view drawing (not dimensioned)
CAM_TRIPOD_MAX = 5.0              # max insertion, datasheet
CAM_M3_DY = 45.0                  # rear M3 pair spacing, datasheet
CAM_M3_MAX = 3.0                  # max insertion, datasheet
CAM_M3_Z = CAM[1] / 2             # ESTIMATE: rear holes at mid height
CAM_LENS_Z = CAM[1] / 2           # ESTIMATE: lens axis at mid height
CAM_USB_SIDE = -1                 # ESTIMATE: USB-C on the -y end face

# ---- mast ----------------------------------------------------------------------------------
CAM_HEIGHT = float(os.environ.get('CAM_HEIGHT', 150.0))      # lens axis above the board top
PITCH_Y = float(os.environ.get('PITCH_Y', 47.0))             # feet pitch across the car
# Where the RPLIDAR C1 sits relative to the mast's foot-pair centre (+x forward, +y left).
# Baseplate v2 layout (LAYOUT.md section 19): mast on A3+B3 at (35, 41), plinth on B1+B2 at
# (95, 20.5), so the lidar axis is one column AHEAD of the mast and half a row to its right.
# The lidar must see forward past the mast, so the mast stands BEHIND it and the four legs
# cross the scan plane in the lidar's rear half only; checks() reports the sectors, and
# lidar_in_view() where the lidar body would enter the bottom of the camera image.
LIDAR_X = float(os.environ.get('LIDAR_X', 60.0))
LIDAR_Y = float(os.environ.get('LIDAR_Y', -20.5))
LIDAR_SCAN_Z = float(os.environ.get('LIDAR_SCAN_Z', 44.8))   # scan plane above the board top (v2: deck 15 + 29.8)
LIDAR_TOP_Z = float(os.environ.get('LIDAR_TOP_Z', 56.3))     # lidar top above the board top (v2: deck 15 + 41.3)
CAM_VFOV = float(os.environ.get('CAM_VFOV', 65.0))           # depth V FOV, degrees (RGB is 55; orbbec.com product page)
CAM_HFOV = 90.0
FLOOR = CH_LIP_Z1                  # 7.0, floor bar height
BAR_W = FOOT_FLANGE[1] + 2 * FOOT_CLR + 2 * 3.0     # 40.5 across the channel (x): 34.5 slot + 3 walls
LEG = 6.0
LEG_BASE = (BAR_W / 2 - LEG / 2, 38.0)              # |x|, |y| of leg centres at the floor
LEG_TOP = (10.0, 19.5)                              # |x|, |y| at the drum yokes
DRUM_R, DRUM_RI, DRUM_W = 20.0, 16.0, 44.0     # 4 mm shell: 1/4-20 x 1/2" leaves 4.7 mm for the camera
TOOTH_DEG, TOOTH_DEPTH, TOOTH_SPAN = 5.0, 0.8, 65.0
TOOTH_CLR = 0.15                   # radial, saddle to drum
PAD_T = 4.0                        # saddle at the crest
BOLT_SLOT_W = 7.5                  # 1/4-20 (6.35) through the drum shell
BOLT_SLOT_ANG = (-15.0, 35.0)      # from +z toward +x: 0..20 pitch plus tripod-position slack
TAB_T, TAB_H = 3.0, 18.0
YOKE = (19.0, 5.0, 10.0)           # half x, y thickness, height

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT, exist_ok=True)

FEET = [(0.0, -PITCH_Y / 2), (0.0, PITCH_Y / 2)]
Z_PAD = DRUM_R + PAD_T                                   # pad top above the drum axis
Z_AXIS = CAM_HEIGHT - RIM_PROUD - Z_PAD - CAM_LENS_Z     # drum axis, part coordinates
X_CAM_REAR = -CAM_TRIPOD_FROM_REAR                       # bolt over the drum axis
X_CAM_FRONT = X_CAM_REAR + CAM[2]
S_STOP = PITCH_Y / 2 + FOOT_FLANGE[0] / 2 + 0.75
X_BUMP = -(PITCH_Y / 2 + FOOT_FLANGE[0] / 2) - 1.0
BAR_S0, BAR_S1 = X_BUMP - 3.5, S_STOP + 4.0              # along the channel (before rotation)


def tooth_profile(offset=0.0):
    """(x, z) polyline of the drum's toothed outline about its axis, 2.5 degree steps from
    -90 to +90, tips on DRUM_R, valleys DRUM_DEPTH in, teeth only within TOOTH_SPAN."""
    pts = []
    n = int(round(90 / (TOOTH_DEG / 2)))
    for i in range(-n, n + 1):
        th = i * TOOTH_DEG / 2
        r = DRUM_R + offset
        if abs(th) <= TOOTH_SPAN and i % 2:
            r -= TOOTH_DEPTH
        pts.append((r * math.sin(math.radians(th)), r * math.cos(math.radians(th))))
    return pts


def fan(pts, y0, y1, z0=0.0):
    """Solid swept from the origin-closed polyline in the XZ plane, y0..y1, axis at z0."""
    return (cq.Workplane('XZ', origin=(0, y1, z0)).polyline([(0, 0)] + pts).close()
            .extrude(y1 - y0))


def leg(x0, y0, x1, y1, z0, z1):
    return (cq.Workplane('XY').workplane(offset=z0).center(x0, y0).rect(LEG, LEG)
            .workplane(offset=z1 - z0).center(x1 - x0, y1 - y0).rect(LEG, LEG).loft(combine=True))


def rot90(s):
    """Channel helpers build along x, open at -x; the mast's channel runs along y, open at +y."""
    return s.rotate((0, 0, 0), (0, 0, 1), -90)


def mast():
    # floor bar with the slide channel across the car, open at +y
    m = box(-BAR_W / 2, BAR_W / 2, -BAR_S1, -BAR_S0, 0, FLOOR)
    m = m.cut(rot90(slide_channel(BAR_S0 - 1, S_STOP)))
    cuts, adds = snap_tongues(BAR_S0, X_BUMP)
    for c in cuts:
        m = m.cut(rot90(c))
    for a in adds:
        m = m.union(rot90(a))
    # four legs, floor corners to the drum yokes
    z_top = Z_AXIS - YOKE[2] + 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            m = m.union(leg(sx * LEG_BASE[0], sy * LEG_BASE[1], sx * LEG_TOP[0], sy * LEG_TOP[1], FLOOR, z_top))
    # yokes under the drum ends
    for sy in (-1, 1):
        ya, yb = sorted((sy * (DRUM_W / 2 - YOKE[1]), sy * DRUM_W / 2))
        m = m.union(box(-YOKE[0], YOKE[0], ya, yb, Z_AXIS - YOKE[2], Z_AXIS + 2.0))
    # drum: toothed half cylinder shell, open underneath between the yokes
    drum = fan(tooth_profile(), -DRUM_W / 2, DRUM_W / 2, Z_AXIS)
    bore = cq.Workplane('XZ', origin=(0, DRUM_W / 2 + 1, Z_AXIS)).circle(DRUM_RI).extrude(DRUM_W + 2)
    drum = drum.cut(bore)
    # arc slot for the 1/4-20 bolt
    a0, a1 = BOLT_SLOT_ANG
    arc = [(25 * math.sin(math.radians(t)), 25 * math.cos(math.radians(t)))
           for t in [a0 + k * 2.5 for k in range(int((a1 - a0) / 2.5) + 1)]]
    drum = drum.cut(fan(arc, -BOLT_SLOT_W / 2, BOLT_SLOT_W / 2, Z_AXIS))
    return m.union(drum)


def saddle():
    """Camera head: toothed concave pad on the drum, flat top for the camera, rear tab."""
    s = box(X_CAM_REAR - TAB_T, X_CAM_FRONT, -DRUM_W / 2, DRUM_W / 2, Z_AXIS + 15.0, Z_AXIS + Z_PAD)
    s = s.cut(fan(tooth_profile(TOOTH_CLR), -DRUM_W / 2 - 1, DRUM_W / 2 + 1, Z_AXIS))
    # rear tab with the two M3 holes
    tab_w = CAM_M3_DY + 7.0
    tab = box(X_CAM_REAR - TAB_T, X_CAM_REAR, -tab_w / 2, tab_w / 2, Z_AXIS + Z_PAD - 0.01, Z_AXIS + Z_PAD + TAB_H)
    s = s.union(tab)
    for sy in (-1, 1):
        s = s.cut(cq.Workplane('YZ', origin=(X_CAM_REAR - TAB_T - 1, 0, 0))
                  .center(sy * CAM_M3_DY / 2, Z_AXIS + Z_PAD + CAM_M3_Z).circle(1.7).extrude(TAB_T + 2))
    # 1/4-20 slot through the pad, 8 mm of fore-aft slack for the tripod socket estimate
    s = s.cut(cq.Workplane('XY').workplane(offset=Z_AXIS + 10).slot2D(8.0 + 6.8, 6.8).extrude(Z_PAD))
    return s


def pitched(shape, deg):
    """Nose-down pitch of the head about the drum axis (positive rotation about +y)."""
    return shape.rotate((0, -1, Z_AXIS), (0, 1, Z_AXIS), deg)


def feet():
    return [foot('y').translate((x, y, CH_LIP_Z0)) for (x, y) in FEET]


def envelope():
    """Camera on the pad at 0 pitch, plus a USB-C plug stub on the USB end."""
    z0 = Z_AXIS + Z_PAD
    e = box(X_CAM_REAR, X_CAM_FRONT, -CAM[0] / 2, CAM[0] / 2, z0, z0 + CAM[1]).edges('|X').fillet(6)
    ya, yb = sorted((CAM_USB_SIDE * CAM[0] / 2, CAM_USB_SIDE * (CAM[0] / 2 + 18)))
    e = e.union(box(X_CAM_REAR + 4, X_CAM_REAR + 16, ya, yb, z0 + 6, z0 + CAM[1] - 6))
    return e


def lens_xyz(deg=0.0):
    """Lens axis point (front face, mid height) in part coordinates at a given pitch."""
    x, z = X_CAM_FRONT, Z_PAD + CAM_LENS_Z
    a = math.radians(deg)
    return (x * math.cos(a) + z * math.sin(a), 0.0, Z_AXIS - x * math.sin(a) + z * math.cos(a))


def legs_in_scan_plane(scan_z=None):
    """Plan (x, y) of the four legs' centres where they cross the lidar scan plane, mast
    coordinates (feet-pair centre, +x forward, +y left). scan_z above the board top."""
    zs = (LIDAR_SCAN_Z if scan_z is None else scan_z) - RIM_PROUD
    t = (zs - FLOOR) / ((Z_AXIS - YOKE[2] + 2.0) - FLOOR)
    return [(sx * (LEG_BASE[0] + t * (LEG_TOP[0] - LEG_BASE[0])), sy * (LEG_BASE[1] + t * (LEG_TOP[1] - LEG_BASE[1])))
            for sx in (-1, 1) for sy in (-1, 1)]


def scan_occlusion(lidar_xy=None, scan_z=None):
    """Each leg as seen by the lidar at lidar_xy (mast coordinates): range, bearing and
    angular width. Bearing is measured from the lidar's +x (dead ahead), positive to the
    LEFT (+y, right-hand); the C1 datasheet counts clockwise, so its angle is 360 - this.
    Width uses a 0.75 x LEG half-width (the 6 mm leg is 8.5 across its diagonal)."""
    lx, ly = (LIDAR_X, LIDAR_Y) if lidar_xy is None else lidar_xy
    out = []
    for (x, y) in legs_in_scan_plane(scan_z):
        r = math.hypot(x - lx, y - ly)
        ang = math.degrees(math.atan2(y - ly, x - lx))
        w = math.degrees(2 * math.atan((LEG * 0.75) / r))
        out.append(dict(leg=('rear' if x < 0 else 'front') + ('-left' if y > 0 else '-right'),
                        x=round(x, 2), y=round(y, 2), range=round(r, 1), bearing=round(ang, 1),
                        width=round(w, 1), sector=(round(ang - w / 2, 1), round(ang + w / 2, 1))))
    return out


def occluded_sectors(occ):
    """Merge the legs' sectors into disjoint (start, end) bearings, and say whether any of
    them reaches into the forward half (|bearing| < 90)."""
    iv = sorted(o['sector'] for o in occ)
    merged = []
    for a, b in iv:
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    forward = [s for s in merged if s[0] < 90 and s[1] > -90]
    return merged, forward


def lidar_in_view(lidar_xy=None, lidar_top_z=None, steps=(0, 5, 10, 15, 20), vfov=None):
    """Per nose-down pitch step: the elevation (degrees below the horizontal, seen from the
    lens) of the lidar's nearest top edge, the bottom edge of the image, the margin, and the
    pitch at which the lidar top would enter the bottom of the image. A rectilinear camera
    puts a point at image row f * z'/x' (x' depth along the optical axis), so the lateral
    offset does not matter: the lidar head's rear-most top point at x = lidar_x - head
    radius, z = lidar_top_z is what enters first. All in mast coordinates."""
    from rplidar_c1_mount import LIDAR_HEAD_D, LIDAR_BASE, LIDAR_BASE_H, LIDAR_HEAD_H
    lx, ly = (LIDAR_X, LIDAR_Y) if lidar_xy is None else lidar_xy
    top = LIDAR_TOP_Z if lidar_top_z is None else lidar_top_z
    half = (CAM_VFOV if vfov is None else vfov) / 2
    # candidates: head cylinder's rear top rim, base block's rear top edge
    cands = [(lx - LIDAR_HEAD_D / 2, top), (lx - LIDAR_BASE / 2, top - LIDAR_HEAD_H)]

    def worst(p):
        """(degrees the lidar's highest edge sits BELOW the image bottom, elevation, dx, dz)
        at nose-down pitch p; the lens moves with the head (lens_xyz)."""
        cx, _, cz = lens_xyz(p)
        cz += RIM_PROUD                                          # above the board top
        best = None
        for (x, z) in cands:
            dx, dz = x - cx, z - cz
            e = math.degrees(math.atan2(dz, dx))                 # below horizontal is negative
            below = (-p - half) - e                              # image bottom ray minus the edge
            if best is None or below < best[0]:
                best = (below, e, dx, dz, cx, cz)
        return best

    # pitch at which the lidar top reaches the image bottom (bisection; monotonic in p)
    lo, hi = 0.0, 89.0
    if worst(lo)[0] <= 0:
        enters = 0.0
    elif worst(hi)[0] > 0:
        enters = None
    else:
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if worst(mid)[0] > 0 else (lo, mid)
        enters = round((lo + hi) / 2, 1)
    rows = []
    for p in steps:
        below, e, dx, dz, cx, cz = worst(p)
        rows.append(dict(pitch=-p, lens_x=round(cx, 1), lens_z=round(cz, 1), lidar_edge_dx=round(dx, 1),
                         lidar_edge_dz=round(dz, 1), lidar_elev=round(e, 1), image_bottom=round(-p - half, 1),
                         margin_below_image=round(below, 1), enters_at_pitch=None if enters is None else -enters))
    return rows


def assembly(m, s):
    a = cq.Assembly(name='camera_mast_on_clipless')
    a.add(plate_stub(FEET, size=(BAR_W + 40, PITCH_Y + 60)), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(FEET):
        a.add(clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))), name=f'clipless_{i}',
              color=cq.Color(0.63, 0.63, 0.63))
    a.add(m.translate((0, 0, RIM_PROUD)), name='mast', color=cq.Color(0.9, 0.45, 0.1))
    a.add(s.translate((0, 0, RIM_PROUD)), name='head', color=cq.Color(0.95, 0.7, 0.2))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='gemini_335_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def checks(m, s):
    fs = feet()
    env = envelope()
    stub = plate_stub(FEET, size=(BAR_W + 40, PITCH_Y + 60)).translate((0, 0, -RIM_PROUD))
    pieces = [clipless_piece().translate((x, y, -RIM_TOP)) for (x, y) in FEET]
    res = {
        'solids_mast': len(m.val().Solids()),
        'solids_head': len(s.val().Solids()),
        'mast_x_head': inter(m, s),
        'mast_x_head_pitched': {d: round(inter(m, pitched(s, d)), 4) for d in (5, 10, 15, 20)},
        'mast_x_envelope_pitched': {d: round(inter(m, pitched(env, d)), 4) for d in (0, 5, 10, 15, 20)},
        'head_x_envelope': inter(s, env),
        'mast_x_feet': sum(inter(m, f) for f in fs),
        'feet_x_clipless': sum(inter(f, c) for f in fs for c in pieces),
        'mast_x_plate': inter(m, stub),
        'feet_x_envelope': sum(inter(f, env) for f in fs),
    }
    lx, _, lz = lens_xyz(0)
    res['lens_above_board_top_at_0'] = round(lz + RIM_PROUD, 2)
    res['lens_above_board_top_at_-20'] = round(lens_xyz(20)[2] + RIM_PROUD, 2)
    res['lens_x_from_feet_row'] = round(lx, 2)
    # lidar AHEAD of the mast (LIDAR_X, LIDAR_Y from the feet-pair centre): physical margin
    # from the mast's front-most point (the floor bar) to the plinth body's rear wall, and the
    # legs as the lidar sees them in its scan plane (all four; they must be in the rear half)
    from rplidar_c1_mount import BODY as PLINTH_BODY
    bb = m.val().BoundingBox()
    res['lidar_axis_from_feet_row_(x,y)'] = (LIDAR_X, LIDAR_Y)
    res['lidar_ahead_of_lens'] = round(LIDAR_X - lx, 2)
    res['mast_frontmost_x'] = round(bb.xmax, 2)
    res['margin_to_plinth_body'] = round((LIDAR_X - PLINTH_BODY / 2) - bb.xmax, 2)
    occ = scan_occlusion()
    merged, forward = occluded_sectors(occ)
    res['legs_in_scan_plane'] = [(o['leg'], o['range'], o['bearing'], o['width']) for o in occ]
    res['occluded_sectors_deg_(bearing_from_dead_ahead,_+left)'] = merged
    res['occlusion_in_forward_half'] = forward or 'none'
    los = lidar_in_view()
    res['lidar_top_below_image_bottom_deg_per_pitch'] = [(r['pitch'], r['margin_below_image']) for r in los]
    res['lidar_top_enters_image_at_pitch'] = los[0]['enters_at_pitch']
    res['lidar_top_enters_rgb_image_at_pitch'] = lidar_in_view(vfov=55.0)[0]['enters_at_pitch']
    res['bolt_engagement_in_camera'] = round(12.7 - (DRUM_R - DRUM_RI) - PAD_T, 2)
    return res


if __name__ == '__main__':
    N = 'camera_mast' + os.environ.get('OUT_SUFFIX', '')   # output file stem (the v2 layout writes camera_mast_v2.*)
    m = mast()
    s = saddle()
    bb = m.val().BoundingBox()
    cq.exporters.export(m, os.path.join(OUT, N + '.step'))
    cq.exporters.export(m, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(s, os.path.join(OUT, N + '_head.step'))
    cq.exporters.export(s, os.path.join(OUT, N + '_head.stl'), tolerance=0.02, angularTolerance=0.1)
    a = assembly(m, s)
    a.save(os.path.join(OUT, N + '_assembly.step'))
    vm, vs = m.val().Volume(), s.val().Volume()
    print(f'mast {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vm/1000:.1f} cm3 ({vm/1000*1.24:.0f} g PLA solid, '
          f'~{vm/1000*1.24*0.6:.0f} g at 60% effective); head {vs/1000:.1f} cm3 ({vs/1000*1.24:.0f} g solid); '
          f'feet pitch {PITCH_Y} mm across, CAM_HEIGHT {CAM_HEIGHT}')
    for k, v in checks(m, s).items():
        print(f'  {k}: {v}')
    render([(m, (0.9, 0.45, 0.1)), (s, (0.95, 0.7, 0.2))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title='Gemini 335 camera mast and head')
    render([(s, (0.95, 0.7, 0.2))], os.path.join(OUT, N + '_head.png'),
           views=[(28, -55), (-30, 125)], title='camera mast head (saddle)')
    fs = feet()
    pieces = [clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))) for (x, y) in FEET]
    stub = plate_stub(FEET, size=(BAR_W + 40, PITCH_Y + 60))
    up = lambda q: q.translate((0, 0, RIM_PROUD))
    shaded = [(stub, (0.55, 0.55, 0.6))] + [(c, (0.63, 0.63, 0.63)) for c in pieces] + \
             [(up(m), (0.9, 0.45, 0.1)), (up(s), (0.95, 0.7, 0.2))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + \
             [(up(envelope()), (0.35, 0.35, 0.35))]
    render(shaded, os.path.join(OUT, N + '_assembly.png'),
           views=[(28, -55), (28, 125)], title='Gemini 335 mast on clipless (grey = camera envelope)')
    lines_png([stub] + pieces + [up(m), up(s)] + [up(f) for f in fs] + [up(envelope())],
              os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'))
    lines_png([m, s], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
