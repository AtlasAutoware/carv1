"""A dimensioned drawing sheet per plate, to go with the DXF.

A DXF alone tells a vendor the shape but not the material, thickness, finish, tolerance or
which way is up. This draws the plate to scale with the datums and the callouts a quoting
engineer needs, one PDF page per part.
"""
import os, sys, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out', 'fab')
MATERIAL = 'Aluminium 5052-H32'
THICK_IN, THICK_MM = 0.080, 2.032
FINISH = 'Deburr all edges. Clear anodise or bare mill finish, fabricator preference.'
TOL = '+-0.25 mm on hole positions, +-0.5 mm on outline'


def sheet(mod, title, fname, note):
    L, W = mod.L_PLATE, mod.W_PLATE
    fig = plt.figure(figsize=(16.5, 11.7), dpi=110)          # A3 landscape
    ax = fig.add_axes([0.05, 0.30, 0.90, 0.62])
    ax.add_patch(FancyBboxPatch((-L/2, -W/2), L, W, boxstyle='round,pad=0,rounding_size=3',
                                fill=False, lw=2, ec='k'))
    for name, (x, y) in mod.HOLES.items():
        ax.add_patch(Rectangle((x - mod.HOLE/2, y - mod.HOLE/2), mod.HOLE, mod.HOLE,
                               fill=False, lw=1.1, ec='tab:blue'))
        ax.text(x, y, name, ha='center', va='center', fontsize=6, color='tab:blue')
    for (c, w, h) in mod.CUTOUTS:
        ax.add_patch(Rectangle((c[0]-w/2, c[1]-h/2), w, h, fill=False, lw=1.1, ec='tab:green'))
        ax.text(c[0], c[1], 'body\npost', ha='center', va='center', fontsize=6, color='tab:green')
    import baseplate_v2 as B
    for (x, y) in mod.ZIP_HOLES:
        ax.add_patch(Circle((x, y), getattr(mod, 'ZIP_D', B.ZIP_D)/2, fill=False, lw=1, ec='tab:orange'))
    for (x, y) in mod.M3_HOLES:
        ax.add_patch(Circle((x, y), getattr(mod, 'M3_D', B.M3_D)/2, fill=False, lw=1, ec='r'))
    for (c, d) in mod.SMALL:
        ax.add_patch(Circle((c[0], c[1]), d/2, fill=False, lw=1, ec='r'))
    # datums
    ax.annotate('', xy=(-L/2, -W/2-14), xytext=(L/2, -W/2-14),
                arrowprops=dict(arrowstyle='<->', lw=1))
    ax.text(0, -W/2-19, f'{L:.1f}', ha='center', fontsize=10)
    ax.annotate('', xy=(L/2+14, -W/2), xytext=(L/2+14, W/2),
                arrowprops=dict(arrowstyle='<->', lw=1))
    ax.text(L/2+19, 0, f'{W:.1f}', va='center', rotation=90, fontsize=10)
    ax.plot([0, 0], [-W/2, W/2], ls=':', lw=0.6, c='0.6')
    ax.plot([-L/2, L/2], [0, 0], ls=':', lw=0.6, c='0.6')
    ax.text(L/2-30, -W/2+13, 'FRONT  +x', fontsize=9, weight='bold')
    ax.set_xlim(-L/2-40, L/2+40); ax.set_ylim(-W/2-32, W/2+22)
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_title(title, fontsize=15, weight='bold', pad=14)

    rows = [
        ('Part', title),
        ('Material', f'{MATERIAL}, {THICK_IN:.3f} in ({THICK_MM:.2f} mm)'),
        ('Quantity', '2'),
        ('Outline', f'{L:.1f} x {W:.1f} mm, 3 mm corner radius'),
        ('Square holes', f'{len(mod.HOLES)} off, {mod.HOLE} x {mod.HOLE} mm, 1.5 mm internal corner radius acceptable'),
        ('Rect cutouts', f'{len(mod.CUTOUTS)} off, see DXF, 2 mm internal corner radius acceptable'),
        ('Round holes', f'{len(mod.ZIP_HOLES)} x {getattr(mod, "ZIP_D", B.ZIP_D)} mm, '
                        f'{len(mod.M3_HOLES) + len(mod.SMALL)} x 3.2 mm'),
        ('Bends', 'None. Flat part.'),
        ('Hardware', 'None.'),
        ('Tolerance', TOL),
        ('Finish', FINISH),
        ('Units', 'Millimetres. DXF is 1:1, geometry only, single CUT layer.'),
        ('Note', note),
    ]
    y0 = 0.245
    fig.text(0.05, y0 + 0.015, 'FABRICATION NOTES', fontsize=11, weight='bold')
    for i, (k, v) in enumerate(rows):
        yy = y0 - i * 0.0175
        fig.text(0.05, yy, k, fontsize=8.5, weight='bold')
        fig.text(0.17, yy, v, fontsize=8.5)
    fig.text(0.05, 0.012, 'AtlasAutoware  ·  TJHSST Senior Research  ·  admin@atlasautoware.org',
             fontsize=8, color='0.35')
    fig.savefig(os.path.join(OUT, fname), format='pdf')
    plt.close(fig)
    return fname


if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else None
    if which == 'newcar':
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'newcar'))
        import baseplate_v2_newcar as BN
        print(sheet(BN, 'Baseplate v2 - new car', 'AtlasAutoware_Baseplate_v2_newcar.pdf',
                    'Body-post cutouts shifted 11.4 mm aft of the other plate for the longer '
                    'wheelbase. Otherwise identical.'))
    elif which == 'oldcar':
        import baseplate_v2 as B
        print(sheet(B, 'Baseplate v2 - old car', 'AtlasAutoware_Baseplate_v2_oldcar.pdf',
                    'Mounts to a Traxxas Slash chassis via the four 3.2 mm holes and the '
                    'four body-post cutouts. No countersinks.'))
    else:
        for w in ('oldcar', 'newcar'):
            r = subprocess.run([sys.executable, __file__, w], capture_output=True, text=True)
            print(r.stdout.strip() or r.stderr.strip().splitlines()[-1])
