"""Fab-ready DXF for the baseplates, for a sheet-metal vendor.

The working DXF that baseplate_v2.py writes carries a LABELS layer (hole names, a FRONT
marker) which is useful on a bench print and wrong to hand a cutter: it pushes the file
extents from 452 x 177.5 out to 646 x 396, and a vendor who flattens layers would cut the
lettering into the plate. This writes geometry only - outline, holes, cutouts - on one
layer, in millimetres, closed polylines, nothing else.

    python3 fab_export.py       writes out/fab/*.dxf and prints the cut summary
"""
import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ezdxf
import baseplate_v2 as B

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out', 'fab')
THICK_IN = 0.080          # 5052-H32; 2.03 mm keeps the clipless rim 1.10 mm proud
THICK_MM = THICK_IN * 25.4


def write(path, holes, cutouts, circles, outline):
    doc = ezdxf.new('R2010')
    doc.header['$INSUNITS'] = 4                     # millimetres
    doc.header['$MEASUREMENT'] = 1
    msp = doc.modelspace()
    doc.layers.add('CUT', color=7)
    msp.add_lwpolyline(outline, close=True, dxfattribs={'layer': 'CUT'})
    for pts in holes + cutouts:
        msp.add_lwpolyline(pts, close=True, dxfattribs={'layer': 'CUT'})
    for x, y, r in circles:
        msp.add_circle((x, y), r, dxfattribs={'layer': 'CUT'})
    doc.saveas(path)
    return 1 + len(holes) + len(cutouts) + len(circles)


def rect(cx, cy, w, h, r=0.0):
    """Corner-radiused rectangle as a polyline with bulges, or plain if r == 0."""
    hw, hh = w / 2, h / 2
    if r <= 0:
        return [(cx - hw, cy - hh), (cx + hw, cy - hh), (cx + hw, cy + hh), (cx - hw, cy + hh)]
    b = math.tan(math.pi / 8)      # 90 degree arc bulge
    return [(cx - hw + r, cy - hh, 0, 0, 0), (cx + hw - r, cy - hh, 0, 0, b),
            (cx + hw, cy - hh + r, 0, 0, 0), (cx + hw, cy + hh - r, 0, 0, b),
            (cx + hw - r, cy + hh, 0, 0, 0), (cx - hw + r, cy + hh, 0, 0, b),
            (cx - hw, cy + hh - r, 0, 0, 0), (cx - hw, cy - hh + r, 0, 0, b)]


def build(module, name):
    L, W = module.L_PLATE, module.W_PLATE
    outline = rect(0, 0, L, W, 3.0)
    # 28.5 mm clipless holes. A 1.5 mm corner radius is added deliberately: a laser or
    # punch leaves a small radius anyway, and the clipless piece's 28 mm rim has 0.25 mm
    # clearance a side, so it drops in regardless. Asking for a perfectly sharp internal
    # corner would mean EDM money for nothing.
    holes = [rect(x, y, module.HOLE, module.HOLE, 1.5) for (x, y) in module.HOLES.values()]
    cutouts = [rect(c[0], c[1], w, h, 2.0) for (c, w, h) in module.CUTOUTS]
    circles = [(x, y, getattr(module, 'ZIP_D', B.ZIP_D) / 2) for (x, y) in module.ZIP_HOLES]
    circles += [(x, y, getattr(module, 'M3_D', B.M3_D) / 2) for (x, y) in module.M3_HOLES]
    circles += [(c[0], c[1], d / 2) for (c, d) in module.SMALL]
    os.makedirs(OUT, exist_ok=True)
    n = write(os.path.join(OUT, name + '.dxf'), holes, cutouts, circles, outline)
    per = sum(2 * (module.HOLE + module.HOLE) for _ in holes)
    return dict(name=name, size=(L, W), entities=n, holes=len(holes), cutouts=len(cutouts),
                circles=len(circles),
                area_cm2=L * W / 100,
                mass_g=(L * W * THICK_MM / 1000) * 2.68 / 1000 * 1000 -
                       (len(holes) * module.HOLE ** 2 * THICK_MM / 1000) * 2.68 / 1000 * 1000)


if __name__ == '__main__':
    # baseplate_v2_newcar.py shifts the body-post cutouts for the longer wheelbase by
    # mutating baseplate_v2's lists IN PLACE. Importing both in one process therefore
    # corrupts whichever was built second, and the only reason the first run came out
    # right was that the old car happened to be built first. Each plate gets its own
    # interpreter so the order cannot matter.
    which = sys.argv[1] if len(sys.argv) > 1 else None
    if which == 'newcar':
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'newcar'))
        import baseplate_v2_newcar as BN
        r = build(BN, 'AtlasAutoware_Baseplate_v2_newcar')
        print(f"{r['name']}|{r['size'][0]}|{r['size'][1]}|{r['entities']}|{r['holes']}|"
              f"{r['cutouts']}|{r['circles']}|{r['mass_g']:.0f}")
        raise SystemExit(0)
    if which == 'oldcar':
        r = build(B, 'AtlasAutoware_Baseplate_v2_oldcar')
        print(f"{r['name']}|{r['size'][0]}|{r['size'][1]}|{r['entities']}|{r['holes']}|"
              f"{r['cutouts']}|{r['circles']}|{r['mass_g']:.0f}")
        raise SystemExit(0)
    import subprocess
    rows = []
    for w in ('oldcar', 'newcar'):
        out = subprocess.run([sys.executable, __file__, w], capture_output=True, text=True)
        line = [l for l in out.stdout.splitlines() if '|' in l]
        if not line:
            print(f'{w} FAILED:', out.stderr.strip().splitlines()[-1] if out.stderr else '?')
            continue
        f = line[0].split('|')
        rows.append(dict(name=f[0], size=(float(f[1]), float(f[2])), entities=int(f[3]),
                         holes=int(f[4]), cutouts=int(f[5]), circles=int(f[6]),
                         mass_g=float(f[7])))
    print(f'material: aluminium 5052-H32, {THICK_IN:.3f} in ({THICK_MM:.2f} mm)')
    for r in rows:
        print(f"  {r['name']}: {r['size'][0]:.1f} x {r['size'][1]:.1f} mm, "
              f"{r['entities']} cut entities ({r['holes']} square holes, "
              f"{r['cutouts']} cutouts, {r['circles']} round holes), ~{r['mass_g']:.0f} g")
    print(f'\nwrote {OUT}')
