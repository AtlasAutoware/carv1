"""Write out/AtlasPower4S/AtlasPower4S.kicad_pcb from netlist.py + layout.py.

Footprints are read from the KiCad libraries (and the generated atlaspower.pretty), placed
per layout.py, and every pad is given its net from the netlist, so the board and the
schematic cannot disagree - they are generated from the same dict.

Connectivity is made three ways, in this order of preference:

1. Zones, for the power nets. A 4-layer board with a solid ground plane and poured rails
   carries 8 A without any trace being the limiting factor, and gives the regulators
   somewhere to put their heat.
2. Explicit tracks, for signal nets, computed from real pad positions.
3. Thermal via stitching under every exposed pad, because on SO-8EP the exposed pad IS
   the thermal path and 45 C/W assumes it is connected to a plane.

What this does NOT do is autoroute. Every track it draws is one this file asked for by
name, and DRC is what says whether the result is manufacturable.
"""
import os, math, hashlib, re
import netlist as N
import layout as LO

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out', 'AtlasPower4S')
# Project-local copies, so the board can be regenerated anywhere without a KiCad install.
# Only the 20 footprints this board uses are vendored; they are unmodified KiCad library
# files, plus the one generated in out/atlaspower.pretty.
FP_DIRS = [os.path.join(HERE, 'libs', 'footprints'), os.path.join(HERE, 'out'),
           os.environ.get('KICAD_FP_DIR', '/home/eshanki/atlas_pcb/libs/footprints')]
ORIGIN = (150.0, 100.0)          # board centre on the KiCad page


def uid(*p):
    h = hashlib.sha1(('atlaspcb:' + ':'.join(map(str, p))).encode()).hexdigest()
    return f'{h[0:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}'


# ---- minimal s-expression reader, enough for .kicad_mod --------------------------------
def parse(text):
    tok = re.findall(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+', text)
    def walk(i):
        out = []
        while i < len(tok):
            t = tok[i]
            if t == '(':
                sub, i = walk(i + 1)
                out.append(sub)
            elif t == ')':
                return out, i + 1
            else:
                out.append(t[1:-1] if t.startswith('"') else t)
                i += 1
        return out, i
    return walk(0)[0][0]


def dump(node, indent=1):
    if isinstance(node, str):
        return node if re.fullmatch(r'[-\w./]+', node) else '"' + node.replace('"', '\\"') + '"'
    return '(' + ' '.join(dump(c, indent) for c in node) + ')'


def find_fp(name):
    lib, fp = name.split(':')
    for d in FP_DIRS:
        p = os.path.join(d, lib + '.pretty', fp + '.kicad_mod')
        if os.path.exists(p):
            return parse(open(p).read())
    raise FileNotFoundError(name)


def get(node, key):
    return next((c for c in node if isinstance(c, list) and c and c[0] == key), None)


def getall(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def rot(x, y, deg):
    a = math.radians(-deg)          # KiCad rotates counter-clockwise, y is down
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


def pad_positions(fpnode, px, py, prot):
    """Absolute board coords of every pad in a placed footprint: {pad name: (x, y)}."""
    out = {}
    for pad in getall(fpnode, 'pad'):
        at = get(pad, 'at')
        lx, ly = float(at[1]), float(at[2])
        dx, dy = rot(lx, ly, prot)
        out.setdefault(pad[1], []).append((px + dx, py + dy))
    return {k: v[0] for k, v in out.items()}, {k: v for k, v in out.items()}


def build():
    parts, nets = N.build()
    places = LO.placement()
    by_ref = {p.ref: p for p in parts}
    netnums = {'': 0}
    for i, n in enumerate(sorted(nets), start=1):
        netnums[n] = i

    ox, oy = ORIGIN
    body, padpos = [], {}

    for part in parts:
        px, py, prot = places[part.ref]
        node = find_fp(part.footprint)
        # strip the library's own at/uuid/property text and re-emit with ours
        pos_single, pos_all = pad_positions(node, px, py, prot)
        padpos[part.ref] = pos_all
        lines = [f'  (footprint "{part.footprint}"',
                 f'    (layer "F.Cu") (uuid {uid("fp", part.ref)})',
                 f'    (at {ox+px:.4f} {oy+py:.4f}{" " + str(prot) if prot else ""})',
                 f'    (descr "{part.desc}")',
                 f'    (attr {" ".join(get(node, "attr")[1:]) if get(node, "attr") else "through_hole"})',
                 f'    (property "Reference" "{part.ref}" (at 0 -3 {prot}) (layer "F.Fab")'
                 f' (uuid {uid("ref", part.ref)}) (effects (font (size 0.8 0.8) (thickness 0.12))))',
                 f'    (property "Value" "{part.value}" (at 0 3 {prot}) (layer "F.Fab")'
                 f' (uuid {uid("val", part.ref)}) (effects (font (size 0.8 0.8) (thickness 0.12))))',
                 f'    (property "LCSC" "{part.lcsc}" (at 0 0 {prot}) (layer "F.Fab") (hide yes)'
                 f' (uuid {uid("lcsc", part.ref)}) (effects (font (size 0.8 0.8) (thickness 0.12))))']
        for key in ('fp_line', 'fp_rect', 'fp_circle', 'fp_arc', 'fp_poly'):
            for g in getall(node, key):
                g = [c for c in g if not (isinstance(c, list) and c and c[0] == 'uuid')]
                lines.append('    ' + dump(g))
        for pad in getall(node, 'pad'):
            pad = [c for c in pad if not (isinstance(c, list) and c and c[0] == 'uuid')]
            net = part.pins.get(pad[1], '')
            s = dump(pad)[:-1]
            if net:
                s += f' (net {netnums[net]} "{net}")'
            s += f' (uuid {uid("pad", part.ref, pad[1])}))'
            lines.append('    ' + s)
        lines.append('  )')
        body.append('\n'.join(lines))

    return parts, nets, netnums, places, padpos, body


# ---- board furniture --------------------------------------------------------------------
def outline():
    ox, oy = ORIGIN
    w, h, r = LO.BOARD_W / 2, LO.BOARD_H / 2, 3.0
    seg = []
    pts = [(-w + r, -h), (w - r, -h), (w, -h + r), (w, h - r), (w - r, h), (-w + r, h),
           (-w, h - r), (-w, -h + r)]
    lines = [(pts[0], pts[1]), (pts[2], pts[3]), (pts[4], pts[5]), (pts[6], pts[7])]
    for (x1, y1), (x2, y2) in lines:
        seg.append(f'  (gr_line (start {ox+x1:.3f} {oy+y1:.3f}) (end {ox+x2:.3f} {oy+y2:.3f})'
                   f' (stroke (width 0.1) (type default)) (layer "Edge.Cuts")'
                   f' (uuid {uid("edge", x1, y1, x2, y2)}))')
    arcs = [((w - r, -h), (w, -h + r), (w - r, -h + r)), ((w, h - r), (w - r, h), (w - r, h - r)),
            ((-w + r, h), (-w, h - r), (-w + r, h - r)), ((-w, -h + r), (-w + r, -h), (-w + r, -h + r))]
    for (sx, sy), (ex, ey), (cx, cy) in arcs:
        mx = cx + (r) * math.cos(math.atan2(sy - cy, sx - cx) / 2 + math.atan2(ey - cy, ex - cx) / 2)
        my = cy + (r) * math.sin(math.atan2(sy - cy, sx - cx) / 2 + math.atan2(ey - cy, ex - cx) / 2)
        seg.append(f'  (gr_arc (start {ox+sx:.3f} {oy+sy:.3f}) (mid {ox+mx:.3f} {oy+my:.3f})'
                   f' (end {ox+ex:.3f} {oy+ey:.3f}) (stroke (width 0.1) (type default))'
                   f' (layer "Edge.Cuts") (uuid {uid("arc", sx, sy, ex, ey)}))')
    return seg


def zone(net, netnum, layer, rect, prio=0, name=''):
    ox, oy = ORIGIN
    if rect is None:
        x0, y0 = -LO.BOARD_W / 2 + LO.EDGE, -LO.BOARD_H / 2 + LO.EDGE
        x1, y1 = LO.BOARD_W / 2 - LO.EDGE, LO.BOARD_H / 2 - LO.EDGE
    else:
        x0, y0, x1, y1 = rect
    pts = ' '.join(f'(xy {ox+x:.3f} {oy+y:.3f})' for x, y in
                   ((x0, y0), (x1, y0), (x1, y1), (x0, y1)))
    return (f'  (zone (net {netnum}) (net_name "{net}") (layer "{layer}")'
            f' (uuid {uid("zone", net, layer, x0, y0)}) (name "{name or net}")'
            f' (hatch edge 0.5) (priority {prio})\n'
            f'    (connect_pads (clearance 0.25))\n'
            f'    (min_thickness 0.25) (filled_areas_thickness no)\n'
            f'    (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.5)'
            f' (island_removal_mode 1) (island_area_min 5))\n'
            f'    (polygon (pts {pts}))\n  )')


def track(x1, y1, x2, y2, w, layer, netnum, tag):
    ox, oy = ORIGIN
    return (f'  (segment (start {ox+x1:.4f} {oy+y1:.4f}) (end {ox+x2:.4f} {oy+y2:.4f})'
            f' (width {w}) (layer "{layer}") (net {netnum}) (uuid {uid("seg", tag, x1, y1, x2, y2)}))')


def via(x, y, netnum, tag):
    ox, oy = ORIGIN
    return (f'  (via (at {ox+x:.4f} {oy+y:.4f}) (size {LO.VIA[0]}) (drill {LO.VIA[1]})'
            f' (layers "F.Cu" "B.Cu") (net {netnum}) (uuid {uid("via", tag, x, y)}))')


# ---- routing ---------------------------------------------------------------------------
# Only signal nets are routed as tracks; power nets are zones. Each entry is routed as an
# L: out from pad A along x, then along y into pad B, on F.Cu. Placement was chosen so
# these stay short and inside their own rail block.
def _obstacles(parts, padpos, places):
    """Every pad as (x0,y0,x1,y1,net) in board coords, for collision checking."""
    obs = []
    for part in parts:
        px, py, prot = places[part.ref]
        node = find_fp(part.footprint)
        for pad in getall(node, 'pad'):
            at, size = get(pad, 'at'), get(pad, 'size')
            x, y = rot(float(at[1]), float(at[2]), prot)
            w, h = float(size[1]) / 2, float(size[2]) / 2
            if prot in (90, 270):
                w, h = h, w
            net = part.pins.get(pad[1], '')
            obs.append((px + x - w, py + y - h, px + x + w, py + y + h, net,
                        (part.ref, pad[1])))
    return obs


def _seg_hits(x1, y1, x2, y2, net, obs, clr, exempt=()):
    """Does an axis-aligned segment pass through a pad of a different net?

    `exempt` is the two (refdes, pin) the segment actually terminates on. Exempting the
    whole part instead was worse than useless: it let a COMP track run straight across its
    own capacitor's GND pad, which DRC then reported as a short. Only the exact pads the
    track lands on are excused.
    """
    sx0, sx1 = min(x1, x2) - clr, max(x1, x2) + clr
    sy0, sy1 = min(y1, y2) - clr, max(y1, y2) + clr
    for (a0, b0, a1, b1, onet, oref) in obs:
        if onet == net or oref in exempt:  # oref is (ref, pin)
            continue
        if not (a1 < sx0 or a0 > sx1 or b1 < sy0 or b0 > sy1):
            return oref
    return None


def route_signals(parts, nets, netnums, padpos, places):
    """Route signal nets as L-paths, but only ones that are actually clear.

    The first attempt drew every L blindly and produced 73 shorts, because a track from FB
    to its divider happily ran straight through the neighbouring capacitor's pad. Now each
    candidate path is checked against every pad of a different net, both orientations are
    tried, and a net that cannot be routed cleanly is LEFT UNROUTED and reported rather
    than shorted. An honest ratsnest line beats a track that ruins the board.
    """
    segs, unrouted = [], []
    SKIP_ZONED = {'GND', 'VIN', 'VBAT', 'VIN_PRE', 'VOUT_10V', 'VOUT_7V4', 'VOUT_5V',
                  'SW1', 'SW2', 'SW3'}
    obs = _obstacles(parts, padpos, places)
    w = LO.TRACK_W['signal']
    clr = 0.2 + w / 2
    for net, conns in sorted(nets.items()):
        if net in SKIP_ZONED or len(conns) < 2:
            continue
        pts = []
        for ref, pin in conns:
            p = padpos.get(ref, {}).get(pin)
            if p:
                pts.append((p[0], ref, pin))
        if len(pts) < 2:
            continue
        pts.sort(key=lambda t: (t[0][0], t[0][1]))
        for (a, ra, pa), (b, rb, pb) in zip(pts, pts[1:]):
            (x1, y1), (x2, y2) = a, b
            ex = ((ra, pa), (rb, pb))
            # Candidate paths, cheapest first. A plain L fails whenever the net's parts sit
            # in a row - the track then runs along the row through every pad in between,
            # including its own neighbours' grounds. So also try escaping perpendicular by
            # an offset, running along at that offset, and coming back in. That is what a
            # person draws, and it is what makes a dense rail block routable at all.
            cands = [[(x1, y1), (x2, y1), (x2, y2)],
                     [(x1, y1), (x1, y2), (x2, y2)]]
            for off in (1.4, -1.4, 2.2, -2.2, 3.0, -3.0):
                cands.append([(x1, y1), (x1, y1 + off), (x2, y1 + off), (x2, y2)])
                cands.append([(x1, y1), (x1 + off, y1), (x1 + off, y2), (x2, y2)])
            done = False
            for pathpts in cands:
                segl = [(pathpts[i], pathpts[i + 1]) for i in range(len(pathpts) - 1)]
                segl = [(s0, e0) for s0, e0 in segl
                        if abs(s0[0] - e0[0]) > 1e-6 or abs(s0[1] - e0[1]) > 1e-6]
                if not segl:
                    continue
                if any(abs(s0[0] - e0[0]) > 1e-6 and abs(s0[1] - e0[1]) > 1e-6 for s0, e0 in segl):
                    continue                      # keep every segment axis aligned
                if all(_seg_hits(s0[0], s0[1], e0[0], e0[1], net, obs, clr, ex) is None
                       for s0, e0 in segl):
                    for s0, e0 in segl:
                        segs.append(track(s0[0], s0[1], e0[0], e0[1], w, 'F.Cu',
                                          netnums[net], net))
                    obs.extend((min(s0[0], e0[0]) - w / 2, min(s0[1], e0[1]) - w / 2,
                                max(s0[0], e0[0]) + w / 2, max(s0[1], e0[1]) + w / 2,
                                net, (net, 'trk')) for s0, e0 in segl)
                    done = True
                    break
            if not done:
                blocker = _seg_hits(x1, y1, x2, y1, net, obs, clr, ex) or \
                          _seg_hits(x1, y1, x1, y2, net, obs, clr, ex)
                unrouted.append((net, ra, pa, rb, pb, blocker))
    return segs, unrouted, [o for o in obs if o[5][1] == 'trk']


def thermal_vias(parts, netnums, places, padpos):
    """Stitch every exposed pad and every large ground pad down to the planes.

    On SO-8EP the exposed pad is the entire thermal path: the datasheet's 45 C/W assumes
    it reaches a plane. Without these the three regulators have nowhere to put ~8 W.
    """
    out = []
    for part in parts:
        ep = None
        if part.value == N.AP:
            ep = '9'
        elif 'INA226' in part.value:
            ep = '11'
        if ep is None or ep not in padpos.get(part.ref, {}):
            continue
        cx, cy = padpos[part.ref][ep][0]
        for i, (dx, dy) in enumerate([(-0.9, -0.9), (0.9, -0.9), (-0.9, 0.9), (0.9, 0.9),
                                      (0, 0), (-1.8, 0), (1.8, 0)]):
            out.append(via(cx + dx, cy + dy, netnums['GND'], f'{part.ref}ep{i}'))
    return out


def gnd_pad_vias(netnums, parts, padpos, places, extra_obs=()):
    """One via next to each GND pad, wherever there is room.

    The top-side ground pour connects GND pads to each other, but the pour itself is cut
    into islands by the rail blocks, and an island with no via is not connected to
    anything. Stitching each GND pad down to In1 fixes both at once.
    """
    obs = _obstacles(parts, padpos, places) + list(extra_obs)
    out, done = [], set()
    for part in parts:
        for pin, net in part.pins.items():
            if net != 'GND' or pin not in padpos.get(part.ref, {}):
                continue
            cx, cy = padpos[part.ref][pin][0]
            for dx, dy in ((0, 1.3), (0, -1.3), (1.3, 0), (-1.3, 0),
                           (1.1, 1.1), (-1.1, -1.1), (0, 1.9), (0, -1.9)):
                x, y = round(cx + dx, 3), round(cy + dy, 3)
                if (x, y) in done:
                    continue
                if _seg_hits(x, y, x, y, 'GND', obs, 0.85, ()) is None:   # via pad 0.6 + clearance
                    out.append(via(x, y, netnums['GND'], f'g{part.ref}{pin}'))
                    obs.append((x - 0.55, y - 0.55, x + 0.55, y + 0.55, 'GND', ('via', 'g')))
                    done.add((x, y))
                    break
    return out


def stitch_zone_vias(netnums, parts, padpos, places, extra_obs=()):
    """Ground stitching, but only where a via actually lands in bare board.

    Vias dropped on a grid without checking produced 49 dangling ones sitting on top of
    pads and holes. Each candidate is now tested against every pad first.
    """
    obs = _obstacles(parts, padpos, places) + list(extra_obs)
    out, skipped = [], 0
    cands = [(float(x), y) for x in range(-40, 41, 8) for y in (-31.0, 31.0)]
    cands += [(float(x), y) for y in (-10.0, 10.0) for x in range(-18, 15, 8)]
    for x, y in cands:
        if _seg_hits(x, y, x, y, 'GND', obs, 0.9, ()) is None:
            out.append(via(x, y, netnums['GND'], f'stitch{x}{y}'))
        else:
            skipped += 1
    return out, skipped


HEADER = '''(kicad_pcb (version 20240108) (generator "atlaspower_gen_pcb") (generator_version "9.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A3")
  (title_block (title "AtlasPower-4S") (company "AtlasAutoware")
    (comment 1 "4S LiPo to 10 V / 7.4 V / 5 V")
    (comment 2 "generated by gen_pcb.py from netlist.py + layout.py - do not hand edit"))
  (layers
    (0 "F.Cu" signal)
    (1 "In1.Cu" signal "ground")
    (2 "In2.Cu" signal "power")
    (31 "B.Cu" signal)
    (32 "B.Adhes" user "B.Adhesive")
    (33 "F.Adhes" user "F.Adhesive")
    (34 "B.Paste" user)
    (35 "F.Paste" user)
    (36 "B.SilkS" user "B.Silkscreen")
    (37 "F.SilkS" user "F.Silkscreen")
    (38 "B.Mask" user)
    (39 "F.Mask" user)
    (40 "Dwgs.User" user "User.Drawings")
    (41 "Cmts.User" user "User.Comments")
    (42 "Eco1.User" user "User.Eco1")
    (43 "Eco2.User" user "User.Eco2")
    (44 "Edge.Cuts" user)
    (45 "Margin" user)
    (46 "B.CrtYd" user "B.Courtyard")
    (47 "F.CrtYd" user "F.Courtyard")
    (48 "B.Fab" user)
    (49 "F.Fab" user)
  )
  (setup
    (pad_to_mask_clearance 0)
    (allow_soldermask_bridges_in_footprints no)
    (pcbplotparams
      (layerselection 0x00010fc_ffffffff) (plot_on_all_layers_selection 0x0000000_00000000)
      (disableapertmacros no) (usegerberextensions no) (usegerberattributes yes)
      (usegerberadvancedattributes yes) (creategerberjobfile yes) (dashed_line_dash_ratio 12.000000)
      (dashed_line_gap_ratio 3.000000) (svgprecision 4) (plotframeref no) (mode 1)
      (useauxorigin no) (dxfpolygonmode yes) (dxfimperialunits yes) (dxfusepcbnewfont yes)
      (psnegative no) (psa4output no) (plot_black_and_white yes) (plotinvisibletext no)
      (sketchpadsonfab no) (plotpadnumbers no) (hidednponfab no) (sketchdnponfab yes)
      (crossoutdnponfab yes) (subtractmaskfromsilk no) (outputformat 1) (mirror no)
      (drillshape 1) (scaleselection 1) (outputdirectory "gerbers/")
    )
  )
'''


def main():
    parts, nets, netnums, places, padpos, body = build()
    doc = [HEADER]
    for n, i in sorted(netnums.items(), key=lambda kv: kv[1]):
        doc.append(f'  (net {i} "{n}")')
    doc += outline()
    doc += body
    for net, layer, rect, prio in LO.ZONES:
        doc.append(zone(net, netnums[net], layer, rect, prio=prio))
    for net, layer, rect, prio in LO.SW_ZONES + LO.OUT_ZONES:
        doc.append(zone(net, netnums[net], layer, rect, prio=prio))
    segs, unrouted, trkobs = route_signals(parts, nets, netnums, padpos, places)
    doc += segs
    doc += thermal_vias(parts, netnums, places, padpos)
    stitches, skipped = stitch_zone_vias(netnums, parts, padpos, places, trkobs)
    doc += stitches
    gvias = gnd_pad_vias(netnums, parts, padpos, places, trkobs)
    doc += gvias
    doc.append(')')
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, 'AtlasPower4S.kicad_pcb')
    with open(path, 'w') as f:
        f.write('\n'.join(doc) + '\n')
    print(f'wrote {path}')
    print(f'  {len(parts)} footprints, {len(nets)} nets, '
          f'{len(LO.ZONES) + len(LO.SW_ZONES) + len(LO.OUT_ZONES)} zones, {len(segs)} track segments, '
          f'{len(stitches)} stitching + {len(gvias)} ground-pad vias '
          f'({skipped} candidates skipped as blocked)')
    if unrouted:
        print(f'  {len(unrouted)} connection(s) left unrouted rather than shorted:')
        for net, ra, pa, rb, pb, blk in unrouted:
            print(f'    {net:10s} {ra}.{pa} -> {rb}.{pb}   blocked by {blk[0]}.{blk[1]}')
    return path


if __name__ == '__main__':
    main()
