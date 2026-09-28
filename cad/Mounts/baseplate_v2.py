"""Baseplate v2: the same outline and chassis fixings as templates/Baseplate.stl, with a
denser clipless grid so the whole mount set fits (see LAYOUT.md, "Baseplate v2").

    python3 baseplate_v2.py          writes out/baseplate_v2.dxf / .step / .stl / .png

What is kept from the STL (measured from it with read_baseplate(), not typed in): the
452 x 177.5 outline, the two pairs of body-post cutouts, the four 3.175 mm shock-tower
clamp holes. What changes: the fifteen 28.5 mm holes on a 50 x 47.5 grid become 28
holes on a 40 x 41 grid (six columns ahead of the rear cutouts plus one column behind
them, four rows), 6 mm zip-tie holes every 40 mm along both long edges, and four M3 holes for optional under-board rails. Web rule: 8 mm minimum between any two clipless
holes, and from a clipless hole to the outline, a cutout or a clamp hole (checked below).

Board frame, same as board_layout.py: +x forward, +y left, +z up, origin at the plate
centre, plate top at z = 0. FRONT_END='xmin' (the STL x = 0 end is the front). Hole
names: rows A (car left, +y) .. D (car right, -y), columns 1 (front) .. 6 on the pitch and
column 8 at the rear (column 7 would be in the rear cutouts and clamp; column 8 is 100 mm
behind column 6, an on-pitch 120 mm would leave only 6.75 mm to the rear edge).

Row pitch: PITCH_Y defaults to 41, not 40. A mount centred on one row reaches out to the
flange zone of the row two pitches away: the Omni cradle is 129.2 wide (64.6 from its row),
a clipless piece put in from above has a 33 mm flange on the plate top (16.5 from its
row), so at 40 mm they overlap by 64.6 - (80 - 16.5) = 1.1 mm on every row-D hole under
the cradle, which are the only holes the Jetson tray can hang from. 41 is the smallest
pitch that clears it (0.9 mm). PITCH_Y=40 reproduces the strict spec; board_layout.py
then reports the conflict. Columns stay on 40 along x, starting at COL0 = -85 (the cradle
on columns 5 and 6 then clears the rear body posts by 3.5 mm); the rear column is 100 mm
behind column 6 so its clipless flange clears the rear clamp and the hole keeps 11 mm to
the cutouts. Rows are symmetric about the centre line unless ROW_OFFSET is set.
"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, 'out')
os.makedirs(OUT, exist_ok=True)

PLATE_T = float(os.environ.get('PLATE_T', 3.175))     # 1/8 in birch ply (the STL thickness)
HOLE = 28.5
PITCH = 40.0                                           # along x
PITCH_Y = float(os.environ.get('PITCH_Y', 41.0))       # across y, see above
COL0 = float(os.environ.get('COL0', -85.0))            # column 6, rear-most regular column
N_COLS = 6
REAR_COL_X = float(os.environ.get('REAR_COL_X', -185.0))   # column 8
ROW_OFFSET = float(os.environ.get('ROW_OFFSET', 0.0))
ROWS = {'A': 1.5 * PITCH_Y + ROW_OFFSET, 'B': 0.5 * PITCH_Y + ROW_OFFSET,
        'C': -0.5 * PITCH_Y + ROW_OFFSET, 'D': -1.5 * PITCH_Y + ROW_OFFSET}
COLS = {i + 1: COL0 + (N_COLS - 1 - i) * PITCH for i in range(N_COLS)}
COLS[8] = REAR_COL_X
MIN_WEB = 8.0
ZIP_D = 6.0
ZIP_Y = 82.0                                           # 3.75 mm web to the plate edge, 3.6 to the
                                                       # nearest clipless-hole corner (they carry no load)
ZIP_XS = [15.0 + 40.0 * k for k in range(-4, 6)] + [-205.0]
M3_D = 3.2
M3_HOLES = [(155.0, ZIP_Y), (155.0, -ZIP_Y), (-125.0, ZIP_Y), (-125.0, -ZIP_Y)]
CABLE_SLOTS = []      # (x, y, len_x, w) 12 mm slots; none needed, see LAYOUT.md (cables use free holes)
FRONT_END = os.environ.get('FRONT_END', 'xmin')


def read_baseplate(path=os.path.join(HERE, 'templates', 'Baseplate.stl')):
    """Outline, holes, cutouts, small holes and clamp blocks of the STL, in STL coordinates."""
    import trimesh
    from shapely.geometry import Polygon
    m = trimesh.load(path)
    parts = m.split(only_watertight=False)
    plate = max(parts, key=lambda p: p.extents[0] * p.extents[1])
    zmin, zmax = plate.bounds[0][2], plate.bounds[1][2]
    sec = plate.section(plane_origin=[0, 0, (zmin + zmax) / 2], plane_normal=[0, 0, 1])
    polys = [Polygon(np.array(d)[:, :2]) for d in sec.discrete]
    polys.sort(key=lambda p: -p.area)
    outline = polys[0]
    holes, cutouts, small = [], [], []
    for p in polys[1:]:
        x0, y0, x1, y1 = p.bounds
        w, h, c = x1 - x0, y1 - y0, ((x0 + x1) / 2, (y0 + y1) / 2)
        if abs(w - HOLE) < 0.2 and abs(h - HOLE) < 0.2:
            holes.append(c)
        elif w > 35 and h > 35:
            cutouts.append((c, w, h))
        else:
            small.append((c, w))
    brackets = []
    for p in parts:
        if p is plate or p.extents[2] < 5:
            continue
        b = p.bounds
        if b[1][2] <= zmin + 1e-6 and p.extents[0] < 20:
            brackets.append(((b[0][0], b[1][0]), (b[0][1], b[1][1]), (b[0][2], b[1][2])))
    ox0, oy0, ox1, oy1 = outline.bounds
    return dict(size=(ox1 - ox0, oy1 - oy0), centre=((ox0 + ox1) / 2, (oy0 + oy1) / 2),
                thickness=zmax - zmin, holes=sorted(holes), cutouts=cutouts, small=small,
                brackets=brackets)


BP = read_baseplate()
CX, CY = BP['centre']
L_PLATE, W_PLATE = BP['size']


def to_board(x, y):
    if FRONT_END == 'xmin':
        return (CX - x, CY - y)
    return (x - CX, y - CY)


CUTOUTS = [(to_board(*c), w, h) for (c, w, h) in BP['cutouts']]          # ((x, y), w, h)
SMALL = [(to_board(*c), d) for (c, d) in BP['small']]                     # ((x, y), d)
BRACKETS = []
for (xr, yr, zr) in BP['brackets']:
    (bx0, by0), (bx1, by1) = to_board(xr[0], yr[0]), to_board(xr[1], yr[1])
    BRACKETS.append((min(bx0, bx1), max(bx0, bx1), min(by0, by1), max(by0, by1), zr))

HOLES = {f'{r}{c}': (COLS[c], ROWS[r]) for c in sorted(COLS) for r in 'ABCD'}
ZIP_HOLES = [(x, sy * ZIP_Y) for x in ZIP_XS for sy in (1, -1)]


# ---- web checks -----------------------------------------------------------------------------
def _rect_gap(a, b):
    """Gap between two axis-aligned rectangles (x0, x1, y0, y1); negative = overlap."""
    dx = max(b[0] - a[1], a[0] - b[1])
    dy = max(b[2] - a[3], a[2] - b[3])
    if dx < 0 and dy < 0:
        return max(dx, dy)
    return math.hypot(max(dx, 0), max(dy, 0))


def hole_rect(x, y, s=HOLE):
    return (x - s / 2, x + s / 2, y - s / 2, y + s / 2)


def check_webs():
    """Every clipless hole: >= MIN_WEB to the outline, the cutouts, the clamp holes, the
    other clipless holes; zip-tie / M3 holes >= 3.5 mm to everything (they carry no load)."""
    problems = []
    rects = {n: hole_rect(*p) for n, p in HOLES.items()}
    for n, r in rects.items():
        edge = min(r[0] + L_PLATE / 2, L_PLATE / 2 - r[1], r[2] + W_PLATE / 2, W_PLATE / 2 - r[3])
        if edge < MIN_WEB:
            problems.append(f'{n}: {edge:.2f} mm to the outline')
        for ((x, y), w, h) in CUTOUTS:
            g = _rect_gap(r, (x - w / 2, x + w / 2, y - h / 2, y + h / 2))
            if g < MIN_WEB:
                problems.append(f'{n}: {g:.2f} mm to the cutout at ({x:.1f}, {y:.1f})')
        for ((x, y), d) in SMALL:
            g = _rect_gap(r, (x - d / 2, x + d / 2, y - d / 2, y + d / 2))
            if g < MIN_WEB:
                problems.append(f'{n}: {g:.2f} mm to the clamp hole at ({x:.1f}, {y:.1f})')
        for m, q in rects.items():
            if m > n:
                g = _rect_gap(r, q)
                if g < MIN_WEB:
                    problems.append(f'{n}-{m}: {g:.2f} mm web')
    for (x, y, d, what) in [(x, y, ZIP_D, 'zip') for (x, y) in ZIP_HOLES] + [(x, y, M3_D, 'M3') for (x, y) in M3_HOLES]:
        edge = min(x + L_PLATE / 2, L_PLATE / 2 - x, y + W_PLATE / 2, W_PLATE / 2 - y) - d / 2
        if edge < 3.5:
            problems.append(f'{what} ({x}, {y}): {edge:.2f} mm to the outline')
        for n, r in rects.items():
            g = _rect_gap((x - d / 2, x + d / 2, y - d / 2, y + d / 2), r)
            if g < 3.5:
                problems.append(f'{what} ({x}, {y}): {g:.2f} mm to {n}')
    return problems


# ---- solids -------------------------------------------------------------------------------
def plate_solid(t=PLATE_T):
    """Top face at z = 0."""
    import cadquery as cq
    p = cq.Workplane('XY').rect(L_PLATE, W_PLATE).extrude(-t)
    for (x, y) in HOLES.values():
        p = p.cut(cq.Workplane('XY').workplane(offset=-t - 1).center(x, y).rect(HOLE, HOLE).extrude(t + 2))
    for ((x, y), w, h) in CUTOUTS:
        p = p.cut(cq.Workplane('XY').workplane(offset=-t - 1).center(x, y).rect(w, h).extrude(t + 2))
    for ((x, y), d) in SMALL:
        p = p.cut(cq.Workplane('XY').workplane(offset=-t - 1).center(x, y).circle(d / 2).extrude(t + 2))
    for (x, y) in ZIP_HOLES:
        p = p.cut(cq.Workplane('XY').workplane(offset=-t - 1).center(x, y).circle(ZIP_D / 2).extrude(t + 2))
    for (x, y) in M3_HOLES:
        p = p.cut(cq.Workplane('XY').workplane(offset=-t - 1).center(x, y).circle(M3_D / 2).extrude(t + 2))
    for (x, y, lx, w) in CABLE_SLOTS:
        p = p.cut(cq.Workplane('XY').workplane(offset=-t - 1).center(x, y).slot2D(lx, w).extrude(t + 2))
    return p


def bracket_solids(t=PLATE_T):
    """The STL's clamp blocks, hung from the plate underside."""
    from jetson_orin_nano_mount import box
    return [box(x0, x1, y0, y1, -t + z0, -t + z1) for (x0, x1, y0, y1, (z0, z1)) in BRACKETS]


# ---- exports ------------------------------------------------------------------------------
def write_dxf(path):
    """Cut file: mm, closed LWPOLYLINEs for the outline, clipless holes and cutouts, CIRCLEs
    for the round holes, one layer per feature type. Laser kerf is left to the operator
    (0.15 to 0.2 mm on 3 mm ply: cut the holes on the line, they come out 0.1 mm large)."""
    import ezdxf
    doc = ezdxf.new('R2010', setup=True)
    doc.header['$INSUNITS'] = 4          # mm
    for name, colour in [('OUTLINE', 1), ('CLIPLESS', 5), ('CUTOUT', 3), ('CLAMP', 6), ('ZIP', 4), ('M3', 2), ('SLOT', 30)]:
        doc.layers.add(name, color=colour)
    msp = doc.modelspace()

    def rect(x, y, w, h, layer):
        pts = [(x - w / 2, y - h / 2), (x + w / 2, y - h / 2), (x + w / 2, y + h / 2), (x - w / 2, y + h / 2)]
        msp.add_lwpolyline(pts, close=True, dxfattribs={'layer': layer})
    rect(0, 0, L_PLATE, W_PLATE, 'OUTLINE')
    for (x, y) in HOLES.values():
        rect(x, y, HOLE, HOLE, 'CLIPLESS')
    for ((x, y), w, h) in CUTOUTS:
        rect(x, y, w, h, 'CUTOUT')
    for ((x, y), d) in SMALL:
        msp.add_circle((x, y), d / 2, dxfattribs={'layer': 'CLAMP'})
    for (x, y) in ZIP_HOLES:
        msp.add_circle((x, y), ZIP_D / 2, dxfattribs={'layer': 'ZIP'})
    for (x, y) in M3_HOLES:
        msp.add_circle((x, y), M3_D / 2, dxfattribs={'layer': 'M3'})
    for (x, y, lx, w) in CABLE_SLOTS:
        r = w / 2
        a, b = x - lx / 2 + r, x + lx / 2 - r
        pts = [(a, y - r), (b, y - r), (b, y + r), (a, y + r)]
        pl = msp.add_lwpolyline([(a, y - r, 0, 0, 0), (b, y - r, 0, 0, 1), (b, y + r, 0, 0, 0), (a, y + r, 0, 0, 1)],
                                format='xyseb', close=True, dxfattribs={'layer': 'SLOT'})
    # text labels (a separate layer the cutter ignores)
    doc.layers.add('LABELS', color=8)
    for n, (x, y) in HOLES.items():
        msp.add_text(n, height=5, dxfattribs={'layer': 'LABELS'}).set_placement((x, y), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    msp.add_text('FRONT +x', height=6, dxfattribs={'layer': 'LABELS'}).set_placement((L_PLATE / 2 - 30, -W_PLATE / 2 - 10))
    doc.saveas(path)


def write_png(path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    fig, ax = plt.subplots(figsize=(13, 6), dpi=140)
    ax.add_patch(Rectangle((-L_PLATE / 2, -W_PLATE / 2), L_PLATE, W_PLATE, fill=False, lw=1.5, color='k'))
    for n, (x, y) in HOLES.items():
        ax.add_patch(Rectangle((x - HOLE / 2, y - HOLE / 2), HOLE, HOLE, fill=False, color='tab:blue', lw=1))
        ax.text(x, y, n, ha='center', va='center', fontsize=8, color='tab:blue')
    for ((x, y), w, h) in CUTOUTS:
        ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, fill=False, color='tab:green', lw=1))
        ax.text(x, y, 'post', ha='center', va='center', fontsize=7, color='tab:green')
    for ((x, y), d) in SMALL:
        ax.add_patch(Circle((x, y), d / 2, fill=False, color='m', lw=1))
    for (x0, x1, y0, y1, _) in BRACKETS:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, color='m', lw=0.8, ls='--'))
    for (x, y) in ZIP_HOLES:
        ax.add_patch(Circle((x, y), ZIP_D / 2, fill=False, color='tab:orange', lw=0.8))
    for (x, y) in M3_HOLES:
        ax.add_patch(Circle((x, y), M3_D / 2, fill=False, color='r', lw=1))
        ax.text(x, y + 5, 'M3', ha='center', fontsize=6, color='r')
    for (x, y, lx, w) in CABLE_SLOTS:
        ax.add_patch(Rectangle((x - lx / 2, y - w / 2), lx, w, fill=False, color='tab:brown', lw=1))
    ax.annotate('FRONT (+x)', xy=(L_PLATE / 2 - 5, -W_PLATE / 2 - 4), ha='right', va='top', fontsize=9)
    ax.annotate('car left (+y)', xy=(-L_PLATE / 2 - 5, W_PLATE / 2 - 4), ha='right', va='top', fontsize=9, rotation=90)
    for c, x in COLS.items():
        ax.text(x, W_PLATE / 2 + 3, f'col {c}\nx={x:g}', ha='center', va='bottom', fontsize=7)
    for r, y in ROWS.items():
        ax.text(L_PLATE / 2 + 3, y, f'{r}  y={y:g}', ha='left', va='center', fontsize=7)
    ax.set_aspect('equal'); ax.set_xlim(-L_PLATE / 2 - 20, L_PLATE / 2 + 40); ax.set_ylim(-W_PLATE / 2 - 15, W_PLATE / 2 + 20)
    ax.set_xlabel('x (mm), board frame'); ax.set_ylabel('y (mm)')
    ax.set_title(f'Baseplate v2, top view: {L_PLATE:.1f} x {W_PLATE:.1f} x {PLATE_T} mm, {len(HOLES)} clipless holes '
                 f'28.5 on a {PITCH:g} x {PITCH_Y:g} grid,\n{len(ZIP_HOLES)} zip-tie holes 6 mm (orange), 4 x M3 (red); '
                 f'green = body-post cutouts, magenta = tower clamp holes and blocks', fontsize=10)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


if __name__ == '__main__':
    import cadquery as cq
    print(f'STL: {L_PLATE:.1f} x {W_PLATE:.1f} x {BP["thickness"]:.3f}; cutouts {[(tuple(round(v, 2) for v in c), round(w, 2), round(h, 2)) for c, w, h in CUTOUTS]}')
    print(f'clamp holes {[(tuple(round(v, 2) for v in c), round(d, 3)) for c, d in SMALL]}')
    print(f'clamp blocks {[tuple(round(v, 2) for v in b[:4]) for b in BRACKETS]}')
    print('columns', COLS); print('rows', ROWS)
    probs = check_webs()
    print('web problems:', probs or 'none')
    assert not probs
    p = plate_solid()
    assert len(p.val().Solids()) == 1
    write_dxf(os.path.join(OUT, 'baseplate_v2.dxf'))
    cq.exporters.export(p, os.path.join(OUT, 'baseplate_v2.step'))
    cq.exporters.export(p, os.path.join(OUT, 'baseplate_v2.stl'), tolerance=0.02, angularTolerance=0.1)
    write_png(os.path.join(OUT, 'baseplate_v2.png'))
    vol = p.val().Volume()
    print(f'plate volume {vol / 1000:.1f} cm3, ~{vol / 1000 * 0.68:.0f} g birch ply (0.68 g/cm3); '
          f'holes: {len(HOLES)} clipless, {len(ZIP_HOLES)} zip, {len(M3_HOLES)} M3')
    print('hole table:')
    for n, (x, y) in HOLES.items():
        print(f'  {n:4s} ({x:7.1f}, {y:6.1f})')
