"""Cable tidy rail for the right-hand edge of the main board, on the clipless mounting
system.

Not a component mount: this is where the slack of the on-board cables goes (USB-C to
barrel from the Omni 20+ to the Lite-On brick, the brick's 19 V barrel lead to the Jetson,
the Orbbec Gemini 335 and RPLidar C1 USB leads, the VESC USB lead and a servo lead;
150 to 400 mm of excess each, 3 to 6 mm diameter). No datasheet applies; the numbers that
size the part are the cable diameters and the bend radius the thick cables tolerate.

A low rail, 150 long, 40.5 wide, 23.5 tall, that runs along the car's right edge (-y in the
board frame, +x forward). Six round spool posts on the centreline, 16 mm diameter (8 mm
bend radius for the 6 mm cables), 24 mm pitch, hollow, each with a 1 mm flared cap so a
wound loop cannot ride up off the post. Between the posts, on the outboard side only,
five tapered comb fingers rise from the 2 mm outboard lip and divide the outboard trough
into one cell per post, so a single-post coil stays in its own cell and a figure-of-eight
around two neighbouring posts (its crossings pass the gap centre, which the fingers do
not reach) is held from sliding along the rail. The inboard edge is open with a 3 mm
round, so a cable is laid in from above, never threaded. Everything a cable can bear on
is filleted: post bases, post caps, lip base, finger tips, the entry round.

Feet: two clipless feet along the car (PEG_PITCH, default 49). Nothing sits on the
flanges, so the rail uses the dovetail slide channel + snap tongue scheme of
rplidar_c1_mount.py: the feet go into their clipless pockets, the rail slides on from the
rear over both flanges to the front stop, and two spring tongues in the ledge snap up
behind the rear flange. The tongues sit 35 mm inside the tunnel, so two 3.2 mm release
holes in the floor let a 2.5 mm hex key press them down for removal.

    python3 cable_tidy_mount.py
    PEG_PITCH=50 python3 cable_tidy_mount.py
    EDGE=1 L=100 N_POST=4 python3 cable_tidy_mount.py     # edge variant, out/cable_tidy_edge.*

EDGE=1 (Baseplate v2, LAYOUT.md "Final set on v2"): no foot channel. The rail hangs
outboard of the plate's right edge (-y) with its inboard face 0.25 mm off the plate edge
(EDGE_GAP), floor top 7.6 above the plate top, post caps at 23.5. Two pads reach inboard
over the plate top and bolt through two of the plate's 6 mm zip-tie holes (y = -82, 40 mm
pitch) with M5 x 16 button-head screws and M5 nyloc nuts under the plate (12 mm screws do
not reach through a nyloc's collar on 3.175 + 4 mm; printed 6 mm snap pins were rejected:
a split pin that fits a 6.0 mm laser-cut hole has two 2.5 mm half-shanks in PLA and
does not survive removal cycles). The rear pad sits behind the rail on a 12 mm ledge
(SPINE_W) along the edge, because the zip holes beside the rail on v2 have the Jetson
tray under them (see the MD). Width W_EDGE (36) instead of the 40.5 the channel needed.
"""
import os, sys, math
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import (foot, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP, FOOT_FLANGE,
                      FOOT_FLANGE_T, FOOT_CLR, PEG_ACROSS)
from render import render
from jetson_orin_nano_mount import lines_png, box, inter

# ---- what the rail is for --------------------------------------------------------------
CABLE_D_MAX = 6.0            # thickest cable (19 V barrel lead, USB-C to barrel)
CABLE_D_MIN = 3.0            # servo lead
BEND_R_MIN = 8.0             # surface radius the 6 mm cables are allowed to wrap
LIDAR_SCAN_ABOVE_BOARD = 44.9    # RPLIDAR_C1_MOUNT.md: scan plane above the board top

# ---- rail --------------------------------------------------------------------------------
PEG_PITCH = float(os.environ.get('PEG_PITCH', 49.0))
L = float(os.environ.get('L', 150.0))        # along the car (100 = four posts)
WALL = 3.0
LIP_T = 2.0                  # outboard lip thickness
LIP_H = 8.5                  # outboard lip above the floor
Z_TOP = 23.5                 # post caps; must stay under the lidar scan plane
POST_D = 2 * BEND_R_MIN      # 16
POST_PITCH = 24.0
N_POST = int(os.environ.get('N_POST', 6))
POST_WALL = 1.6              # posts are cups (4 perimeters)
CAP_PROUD = 1.0              # cap radius beyond the post, 45 degree underside
FINGER_T = 2.4               # at the root; 2 degree draft above that
FINGER_H = (8.0, 4.5)        # above the floor at the lip, at the tip
FINGER_TIP_Y = -10.5         # tip stops 2.5 mm short of the post radius
ENTRY_R = 3.0                # round on the open inboard edge
FILLET = 2.0
TEXT = 'ATLAS'
TEXT_H = 7.0
TEXT_PROUD = 0.6

# channel (dovetail slide along x, same scheme as rplidar_c1_mount.slide_channel)
CH_FLOOR = 2.0               # ledge under the flange
CH_LIP_Z0 = CH_FLOOR + FOOT_FLANGE_T          # 4.0
CH_LIP = 2.0                                  # 45 degree lip height (rplidar used 3)
CH_LIP_Z1 = CH_LIP_Z0 + CH_LIP                # 6.0
ROOF = 1.6                                    # 8 layers at 0.2, bridged over the tunnel
Z_F = CH_LIP_Z1 + ROOF                        # 7.6, the cable floor
TONGUE_L = 14.5
TONGUE_SLOT = 0.6
BUMP_H = 0.6
RELEASE_D = 3.2

# ---- edge variant (EDGE=1) -----------------------------------------------------------------
EDGE = int(os.environ.get('EDGE', 0))
W_EDGE = float(os.environ.get('W_EDGE', 36.0))          # rail width without the channel
EDGE_GAP = 0.25                                         # inboard face off the plate edge
ZIP_INSET = 6.75                                        # zip hole centre inboard of the plate edge (88.75 - 82)
PAD_T = 4.0                                             # pad on the plate top
PAD_HALF_X = 7.0                                        # pad 14 long along the edge
PAD_IN = 12.75                                          # pad reaches this far inboard of the edge (to y -76)
PAD_HOLE = 5.3                                          # M5 clearance
SPINE_W = 12.0                                          # ledge along the edge to the rear pad
UNDER_POCKET = 3.0                                      # walls / floor left by the underside pocket
PLATE_T_V2 = 3.175
# pad x positions in rail coordinates: the front pad 7 mm inside the rear end of the rail,
# the rear pad 40 mm behind it on the ledge (BRACKET_X overrides, JSON list)
BRACKET_X = [float(v) for v in os.environ.get('BRACKET_X', '').split(',') if v] or None
SCREW = dict(d=5.0, head_d=9.5, head_h=2.75, length=16.0, nut_d=9.2, nut_h=5.0)   # M5 x 16 button head, nyloc

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
N = 'cable_tidy' + ('_edge' if EDGE else '') + os.environ.get('OUT_SUFFIX', '')   # output file stem
os.makedirs(OUT, exist_ok=True)

HP = PEG_ACROSS / 2 + FOOT_CLR                # 12.15 half peg slot
HF = FOOT_FLANGE[1] / 2 + FOOT_CLR            # 17.25 half flange slot
HL = HF - CH_LIP                              # 15.25 half opening above the lips
W = 2 * HF + 2 * WALL                         # 40.5: the 34.5 flange slot plus 3 mm walls
if EDGE:
    W = W_EDGE
POST_H = Z_TOP - Z_F
POST_R = POST_D / 2
FEET = [(-PEG_PITCH / 2, 0.0), (PEG_PITCH / 2, 0.0)]
POSTS = [(-(N_POST - 1) / 2 + i) * POST_PITCH for i in range(N_POST)]      # -60 .. 60
FINGERS = [(POSTS[i] + POSTS[i + 1]) / 2 for i in range(N_POST - 1)]      # -48 .. 48
# channel: open at the rear (-x), stop face 0.75 past the front flange
S_STOP = PEG_PITCH / 2 + FOOT_FLANGE[0] / 2 + 0.75            # 40.25
X_BUMP = -(PEG_PITCH / 2 + FOOT_FLANGE[0] / 2) - 1.0          # -40.5, steep face
X_TONGUE_TIP = X_BUMP - 3.5                                   # -44.0, free end
X_RELEASE = X_TONGUE_TIP + 1.0
Y_RELEASE = (HP + HF - TONGUE_SLOT) / 2                       # over the tongue
POCKET_X0, POCKET_X1 = S_STOP + WALL, L / 2 - WALL            # lightening pocket ahead of the stop
if EDGE:
    BRACKET_X = BRACKET_X or [-L / 2 + PAD_HALF_X, -L / 2 + PAD_HALF_X - 40.0]
    Y_EDGE = W / 2 + EDGE_GAP                                  # plate edge in rail coordinates
    Y_HOLE = Y_EDGE + ZIP_INSET                                # zip hole centre line


def slide_channel(x_open, x_stop):
    """Cut solid for the channel from x_open to x_stop, centred on y=0, underside z=0:
    24.3 peg slot through the 2 mm ledge, 34.5 flange slot 2 mm tall, 45 degree lips
    closing to 30.5 at z 6, roof above."""
    pts = [(-HP, -1), (HP, -1), (HP, CH_FLOOR), (HF, CH_FLOOR), (HF, CH_LIP_Z0), (HL, CH_LIP_Z1),
           (-HL, CH_LIP_Z1), (-HF, CH_LIP_Z0), (-HF, CH_FLOOR), (-HP, CH_FLOOR)]
    return (cq.Workplane('YZ', origin=(x_open, 0, 0)).polyline(pts).close()
            .extrude(x_stop - x_open))


def snap_tongues():
    """Turn TONGUE_L of each ledge, free end at X_TONGUE_TIP pointing aft, into a spring
    tongue (2 x 4.5 mm) carrying a bump whose steep face at X_BUMP snaps up behind the
    rear foot flange. Returns (cuts, adds)."""
    root = X_TONGUE_TIP + TONGUE_L
    cuts, adds = [], []
    for sy in (-1, 1):
        ya, yb = sorted((sy * (HF - TONGUE_SLOT), sy * HF))
        cuts.append(box(X_TONGUE_TIP - TONGUE_SLOT, root, ya, yb, -1, CH_FLOOR + 0.6))   # side slit
        ya, yb = sorted((sy * (HP - 0.5), sy * HF))
        cuts.append(box(X_TONGUE_TIP - TONGUE_SLOT, X_TONGUE_TIP, ya, yb, -1, CH_FLOOR + 0.6))  # end slit
        y0, y1 = sorted((sy * (HP + 0.1), sy * (HF - TONGUE_SLOT - 0.1)))
        prof = [(X_TONGUE_TIP, CH_FLOOR - 0.3), (X_BUMP - 0.6, CH_FLOOR + BUMP_H),
                (X_BUMP - 0.3, CH_FLOOR + BUMP_H), (X_BUMP, CH_FLOOR - 0.3)]
        adds.append(cq.Workplane('XZ', origin=(0, y0, 0)).polyline(prof).close().extrude(-(y1 - y0)))
    return cuts, adds


def body():
    """Floor slab plus outboard lip as one extruded section: lip base fillet, inner top
    round and outer top chamfer are in the profile."""
    yo, yi = -W / 2, W / 2
    zl = Z_F + LIP_H
    a = math.sqrt(0.5)
    prof = (cq.Workplane('YZ', origin=(-L / 2, 0, 0))
            .moveTo(yo, 0).lineTo(yi, 0).lineTo(yi, Z_F)
            .lineTo(yo + LIP_T + FILLET, Z_F)
            .threePointArc((yo + LIP_T + FILLET - FILLET * a, Z_F + FILLET - FILLET * a), (yo + LIP_T, Z_F + FILLET))
            .lineTo(yo + LIP_T, zl - 1.0)
            .threePointArc((yo + LIP_T - 1.0 + a, zl - 1.0 + a), (yo + LIP_T - 1.0, zl))
            .lineTo(yo, zl - 1.0).close()
            .extrude(L))
    # 2 mm rounds on the four outside vertical corners: intersect with a rounded prism
    # (a direct fillet fails where the corner edge meets the lip chamfer)
    prof = prof.intersect(cq.Workplane('XY').rect(L, W).extrude(zl + 1).edges('|Z').fillet(FILLET))
    # open inboard edge: 3 mm round so a cable laid in from above bends over a radius
    cut = box(-L / 2 - 1, L / 2 + 1, yi - ENTRY_R, yi + 1, Z_F - ENTRY_R, Z_F + 1)
    cyl = (cq.Workplane('YZ', origin=(-L / 2 - 1, 0, 0)).center(yi - ENTRY_R, Z_F - ENTRY_R)
           .circle(ENTRY_R).extrude(L + 2))
    return prof.cut(cut.cut(cyl))


def post(x):
    """Hollow spool post: 2 mm base fillet, 45 degree flared cap with a rounded rim."""
    R, H = POST_R, POST_H
    a = math.sqrt(0.5)
    rc = R + CAP_PROUD
    z_cap = H - 2.0 - CAP_PROUD
    p = (cq.Workplane('XZ')
         .moveTo(0, 0).lineTo(R + FILLET, 0)
         .threePointArc((R + FILLET - FILLET * a, FILLET - FILLET * a), (R, FILLET))
         .lineTo(R, z_cap).lineTo(rc, z_cap + CAP_PROUD)
         .lineTo(rc, H - 0.8)
         .threePointArc((rc - 0.8 + 0.8 * a, H - 0.8 + 0.8 * a), (rc - 0.8, H))
         .lineTo(0, H).close()
         .revolve(360, (0, 0, 0), (0, 1, 0)))
    bore = cq.Workplane('XY').workplane(offset=1.2).circle(R - POST_WALL).extrude(H)
    bore = bore.faces('<Z').edges().fillet(1.0)
    return p.cut(bore).translate((x, 0, Z_F))


def finger(x):
    """Tapered comb finger from the lip inner face to FINGER_TIP_Y: 2 degree draft on
    the sides, top sloping from FINGER_H[0] at the lip to FINGER_H[1] at the tip, tip
    edges rounded."""
    y_root = -W / 2 + LIP_T - 0.5          # buried 0.5 in the lip
    y_tip = FINGER_TIP_Y
    yc, ln = (y_root + y_tip) / 2, y_tip - y_root
    f = (cq.Workplane('XY').workplane(offset=Z_F - 0.5).center(x, yc)
         .rect(FINGER_T, ln).extrude(FINGER_H[0] + 0.5, taper=2.0))
    z0, z1 = Z_F + FINGER_H[0], Z_F + FINGER_H[1]
    slope = (z1 - z0) / (y_tip - (-W / 2 + LIP_T))
    top = (cq.Workplane('YZ', origin=(x - 3, 0, 0))
           .polyline([(y_root - 1, Z_F - 1), (y_tip + 1, Z_F - 1),
                      (y_tip + 1, z0 + slope * (y_tip + 1 - (-W / 2 + LIP_T))),
                      (y_root - 1, z0 + slope * (y_root - 1 - (-W / 2 + LIP_T)))]).close()
           .extrude(6))
    f = f.intersect(top)
    tip = cq.selectors.BoxSelector((x - 3, y_tip - 1.5, Z_F), (x + 3, y_tip + 1.5, Z_TOP))
    f = f.edges(tip).fillet(0.7)
    return f


def rail():
    r = body()
    for x in POSTS:
        r = r.union(post(x))
    for x in FINGERS:
        r = r.union(finger(x))
    # embossed name on the outboard face, centred on the floor + lip height
    txt = (cq.Workplane('XZ', origin=(0, -W / 2, (Z_F + LIP_H) / 2))
           .text(TEXT, TEXT_H, TEXT_PROUD, halign='center', valign='center', kind='bold'))
    r = r.union(txt)
    if EDGE:
        return edge_features(r)
    # slide channel from the open rear end to the stop, tongues, release holes
    r = r.cut(slide_channel(-L / 2 - 1, S_STOP))
    cuts, adds = snap_tongues()
    for c in cuts:
        r = r.cut(c)
    for a in adds:
        r = r.union(a)
    for sy in (-1, 1):
        r = r.cut(cq.Workplane('XY').workplane(offset=CH_FLOOR + 0.6).center(X_RELEASE, sy * Y_RELEASE)
                  .circle(RELEASE_D / 2).extrude(Z_F))
    # lightening pocket ahead of the stop, same section so it prints the same way
    if POCKET_X1 - POCKET_X0 >= 8:
        r = r.cut(slide_channel(POCKET_X0, POCKET_X1))
    return r


def edge_features(r):
    """EDGE=1: underside pocket instead of the channel, the ledge to the rear pad, two
    pads over the plate top with M5 clearance holes on the zip-hole line."""
    # underside pocket: 3 mm floor and walls, ribs (6 mm) under each pad position
    x0, x1 = -L / 2 + UNDER_POCKET, L / 2 - UNDER_POCKET
    pocket = box(x0, x1, -W / 2 + UNDER_POCKET, W / 2 - UNDER_POCKET, -1, Z_F - UNDER_POCKET)
    for bx in BRACKET_X:
        if x0 < bx < x1:
            pocket = pocket.cut(box(bx - 3, bx + 3, -W, W, -2, Z_F))
    r = r.cut(pocket)
    # ledge along the plate edge from the rail's rear end back to the rear pad
    xs = min(BRACKET_X) - PAD_HALF_X
    if xs < -L / 2:
        ledge = box(xs, -L / 2 + 1, W / 2 - SPINE_W, W / 2, 0, Z_F).edges('|Z and <X').fillet(FILLET)
        r = r.union(ledge)
    # pads: from 3 mm inside the inboard wall, over the gap, PAD_IN onto the plate top
    for bx in BRACKET_X:
        pad = box(bx - PAD_HALF_X, bx + PAD_HALF_X, W / 2 - 3.0, Y_EDGE + PAD_IN, 0, PAD_T)
        pad = pad.edges('|Z and >Y').fillet(FILLET)
        r = r.union(pad)
        r = r.cut(cq.Workplane('XY').workplane(offset=-1).center(bx, Y_HOLE).circle(PAD_HOLE / 2).extrude(Z_F + 2))
    return r


def fasteners():
    """M5 x 16 button-head screws through the pads and nyloc nuts under the plate, as one
    grey envelope (for the layout's interference check)."""
    S = SCREW
    f = None
    for bx in BRACKET_X:
        head = cq.Workplane('XY').workplane(offset=PAD_T).center(bx, Y_HOLE).circle(S['head_d'] / 2).extrude(S['head_h'])
        shank = cq.Workplane('XY').workplane(offset=PAD_T - S['length']).center(bx, Y_HOLE).circle(S['d'] / 2).extrude(S['length'])
        nut = (cq.Workplane('XY').workplane(offset=-PLATE_T_V2 - S['nut_h']).center(bx, Y_HOLE)
               .polygon(6, S['nut_d']).extrude(S['nut_h']))
        s = head.union(shank).union(nut)
        f = s if f is None else f.union(s)
    return f


def edge_stub():
    """Piece of the v2 plate: right edge at Y_EDGE, two 6 mm zip holes, top at z=0."""
    x0 = min(BRACKET_X) - 20
    p = box(x0, L / 2 + 20, Y_EDGE, Y_EDGE + 60, -PLATE_T_V2, 0)
    for bx in BRACKET_X:
        p = p.cut(cq.Workplane('XY').workplane(offset=-PLATE_T_V2 - 1).center(bx, Y_HOLE).circle(3.0).extrude(PLATE_T_V2 + 2))
    return p


def feet():
    if EDGE:
        return []
    return [foot('x').translate((x, y, CH_LIP_Z0)) for (x, y) in FEET]


def cables():
    """Grey sample slack for the render: a two-layer figure-of-eight of 6 mm cable on
    posts 0-1, a three-turn coil of 4 mm cable on post 4, tails leaving inboard."""
    def ring(x, r_path, d, z):
        return cq.Workplane('XY').add(cq.Solid.makeTorus(r_path, d / 2)).translate((x, 0, z))

    def tube(p0, p1, d):
        v = cq.Vector(*p1) - cq.Vector(*p0)
        return cq.Workplane('XY').add(cq.Solid.makeCylinder(d / 2, v.Length, cq.Vector(*p0), v))

    parts = []
    # figure-of-eight around posts 0 and 1 (6 mm)
    d = CABLE_D_MAX
    rp = POST_R + CAP_PROUD * 0 + 0.5 + d / 2           # path radius, 0.5 off the post
    x0, x1 = POSTS[0], POSTS[1]
    half = (x1 - x0) / 2
    t_len = math.sqrt(half ** 2 - rp ** 2)
    ang = math.acos(rp / half)
    for k in range(2):
        z = Z_F + d / 2 + 0.5 + k * d
        parts.append(ring(x0, rp, d, z))
        parts.append(ring(x1, rp, d, z))
        for s in (1, -1):
            a0 = (x0 + rp * math.cos(ang), s * rp * math.sin(ang), z)
            a1 = (x1 - rp * math.cos(ang), -s * rp * math.sin(ang), z)
            parts.append(tube(a0, a1, d))
    # tail of that cable leaving inboard from the top layer
    zt = Z_F + d / 2 + 0.5 + d
    parts.append(tube((x0 + 3, rp, zt), (x0 + 3, W / 2 + 14, zt), d))
    # coil around post 4 (4 mm), three turns
    d2 = 4.0
    rp2 = POST_R + 0.5 + d2 / 2
    for k in range(3):
        parts.append(ring(POSTS[min(4, N_POST - 1)], rp2, d2, Z_F + d2 / 2 + 0.5 + k * d2))
    zt2 = Z_F + d2 / 2 + 0.5 + 2 * d2
    parts.append(tube((POSTS[min(4, N_POST - 1)] + 2, rp2, zt2), (POSTS[min(4, N_POST - 1)] + 2, W / 2 + 14, zt2), d2))
    c = parts[0]
    for p in parts[1:]:
        c = c.union(p)
    return c


def edge_checks(r):
    stub, fas, cab = edge_stub(), fasteners(), cables()
    bb = r.val().BoundingBox()
    # what the pads leave for the plate's holes: the pad's inboard edge vs the row-D flange
    # zone of a from-above clipless piece (y -78 in board coordinates = Y_EDGE + 10.75)
    return {
        'solids': len(r.val().Solids()),
        'rail_x_plate': inter(r, stub),
        'rail_x_fasteners': inter(r, fas),
        'rail_x_cables': inter(r, cab),
        'fasteners_x_plate': inter(fas, stub),          # the shank passes the 6 mm hole
        'rail_top_above_plate_top': bb.zmax,
        'inboard_face_gap_to_plate_edge': round(Y_EDGE - W / 2, 3),
        'outboard_face_from_plate_edge': round(Y_EDGE + W / 2 + TEXT_PROUD, 2),
        'pad_inboard_reach_from_plate_edge': PAD_IN,
        'pad_hole_d': PAD_HOLE,
        'bracket_x': BRACKET_X,
        'ledge_width_outboard_of_edge': SPINE_W - EDGE_GAP,
        'lidar_scan_plane_above_board_top': LIDAR_SCAN_ABOVE_BOARD,
        'post_surface_radius': POST_R,
        'outboard_trough_width': -POST_R - (-W / 2 + LIP_T),
        'inboard_passage_width': W / 2 - POST_R,
        'post_gap': POST_PITCH - POST_D,
    }


def assembly(r):
    if EDGE:
        a = cq.Assembly(name='cable_tidy_edge_on_plate')
        a.add(edge_stub(), name='baseplate_edge', color=cq.Color(0.55, 0.55, 0.6))
        a.add(r, name='cable_tidy_edge', color=cq.Color(0.9, 0.45, 0.1))
        a.add(fasteners(), name='m5_screws_nuts', color=cq.Color(0.35, 0.35, 0.35))
        a.add(cables(), name='sample_cables', color=cq.Color(0.35, 0.35, 0.35))
        return a
    a = cq.Assembly(name='cable_tidy_on_clipless')
    a.add(plate_stub(FEET, size=(L + 20, W + 40)), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(FEET):
        a.add(clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))), name=f'clipless_{i}',
              color=cq.Color(0.63, 0.63, 0.63))
    a.add(r.translate((0, 0, RIM_PROUD)), name='cable_tidy', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(cables().translate((0, 0, RIM_PROUD)), name='sample_cables', color=cq.Color(0.35, 0.35, 0.35))
    return a


def checks(r):
    fs = feet()
    cab = cables()
    stub = plate_stub(FEET, size=(L + 20, W + 40)).translate((0, 0, -RIM_PROUD))
    pieces = [clipless_piece().translate((x, y, -RIM_TOP)) for (x, y) in FEET]
    fl = FOOT_FLANGE[0] / 2
    to_stop = r.translate((-(S_STOP - (PEG_PITCH / 2 + fl + FOOT_CLR)), 0, 0))
    to_bump = r.translate((-(PEG_PITCH / 2 + fl + FOOT_CLR) - X_BUMP, 0, 0))
    bb = r.val().BoundingBox()
    return {
        'solids': len(r.val().Solids()),
        'rail_x_feet': sum(inter(r, f) for f in fs),
        'rail_x_feet_at_stop': sum(inter(to_stop, f) for f in fs),
        'rail_x_feet_at_bump': sum(inter(to_bump, f) for f in fs),
        'fore_aft_play_between_stop_and_bump': (S_STOP - (PEG_PITCH / 2 + fl)) + (-(PEG_PITCH / 2 + fl) - X_BUMP),
        'feet_x_clipless': sum(inter(f, c) for f in fs for c in pieces),
        'rail_x_plate': inter(r, stub),
        'rail_x_cables': inter(r, cab),
        'feet_x_cables': sum(inter(f, cab) for f in fs),
        'rail_top_above_board_top': RIM_PROUD + bb.zmax,
        'lidar_scan_plane_above_board_top': LIDAR_SCAN_ABOVE_BOARD,
        'post_surface_radius': POST_R,
        'outboard_trough_width': -POST_R - (-W / 2 + LIP_T),
        'inboard_passage_width': W / 2 - POST_R,
        'post_gap': POST_PITCH - POST_D,
        'text_proud': TEXT_PROUD,
    }


if __name__ == '__main__':
    r = rail()
    bb = r.val().BoundingBox()
    cq.exporters.export(r, os.path.join(OUT, N + '.step'))
    cq.exporters.export(r, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    a = assembly(r)
    a.save(os.path.join(OUT, N + '_assembly.step'))
    vol = r.val().Volume()
    print(f'rail {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'({vol/1000*1.24:.1f} g PLA solid, ~{vol/1000*1.24*0.6:.0f} g at 60% effective), '
          f'{"edge brackets " + str(BRACKET_X) if EDGE else "peg pitch %s mm" % PEG_PITCH}')
    for k, v in (edge_checks(r) if EDGE else checks(r)).items():
        print(f'  {k}: {v}')
    render([(r, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title='Cable tidy rail' + (' (edge variant)' if EDGE else ''))
    if EDGE:
        stub, fas, cab = edge_stub(), fasteners(), cables()
        shaded = [(stub, (0.55, 0.55, 0.6)), (r, (0.9, 0.45, 0.1)), (fas, (0.35, 0.35, 0.35)), (cab, (0.35, 0.35, 0.35))]
        render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(28, -55), (28, 125)],
               title='Cable tidy rail on the plate edge (grey = M5 screws and nuts, sample cable slack)')
        lines_png([q for q, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'),
                  os.path.join(OUT, N + '_assembly_lines.png'))
        lines_png([r], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
        sys.exit(0)
    fs = feet()
    pieces = [clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))) for (x, y) in FEET]
    stub = plate_stub(FEET, size=(L + 20, W + 40))
    up = lambda s: s.translate((0, 0, RIM_PROUD))
    cab = cables()
    shaded = [(stub, (0.55, 0.55, 0.6))] + [(c, (0.63, 0.63, 0.63)) for c in pieces] + \
             [(up(r), (0.9, 0.45, 0.1))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + \
             [(up(cab), (0.35, 0.35, 0.35))]
    render(shaded, os.path.join(OUT, N + '_assembly.png'),
           views=[(28, -55), (28, 125)], title='Cable tidy rail on clipless (grey = sample cable slack)')
    lines_png([stub] + pieces + [up(r)] + [up(f) for f in fs] + [up(cab)],
              os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'))
    lines_png([r], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
