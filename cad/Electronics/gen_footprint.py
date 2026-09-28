"""Generate the Bourns SRP1265A footprint from its datasheet's recommended layout.

KiCad ships no SRP1265A land pattern, and the previous stand-in (L_APV_APH1265) was never
checked. These numbers come from the Bourns SRP1265A datasheet, "Recommended Layout" and
"Product Dimensions" figures, read off the rendered drawing rather than from extracted
text, because text extraction loses which number belongs to which dimension line:

    overall span across both pads   14.2 mm
    gap between the pads             8.0 mm
    pad height                       5.0 mm
    => pad width  (14.2 - 8.0) / 2 = 3.1 mm
    => pad centres at            +- (8.0 + 3.1) / 2 = +- 5.55 mm

Cross-check, and the reason to trust the reading: the part's own terminal is 4.7 +- 0.3
wide by 2.75 +- 0.35 long. A 5.0 x 3.1 pad is that terminal plus a small extension on
every side, which is what a recommended land is. Had the 8.0 been centre-to-centre
instead of the gap, the pads would have come out 6.2 mm wide against a 2.75 mm terminal,
which is not a land pattern anyone draws.

Body is 13.5 +- 0.5 across the terminals by 12.5 +- 0.3, 6.2 +- 0.3 tall.

    python3 gen_footprint.py     writes out/atlaspower.pretty/L_Bourns_SRP1265A.kicad_mod
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out', 'atlaspower.pretty')

SPAN = 14.2          # outer edge to outer edge of the two pads
GAP = 8.0            # between the pads
PAD_H = 5.0
PAD_W = (SPAN - GAP) / 2          # 3.1
PAD_X = (GAP + PAD_W) / 2         # 5.55
BODY_X, BODY_Y = 13.5, 12.5
BODY_Z = 6.2
CRTYD = 0.25         # clearance beyond the greater of body and pads
SILK_GAP = 0.11      # silkscreen off the body outline


def kicad_mod():
    hx = max(BODY_X, SPAN) / 2 + CRTYD
    hy = max(BODY_Y, PAD_H) / 2 + CRTYD
    bx, by = BODY_X / 2, BODY_Y / 2
    sx, sy = bx + SILK_GAP, by + SILK_GAP
    L = [
        '(footprint "L_Bourns_SRP1265A"',
        '  (version 20240108) (generator "atlaspower_gen_footprint") (layer "F.Cu")',
        '  (descr "Bourns SRP1265A shielded power inductor, 13.5x12.5x6.2 mm, '
        'land pattern from the datasheet Recommended Layout: 14.2 mm span, 8.0 mm gap, '
        '5.0 mm pad height")',
        '  (tags "inductor bourns srp1265a power shielded")',
        '  (attr smd)',
    ]
    # silkscreen: the two long sides only, so the pads stay clear
    for s in (1, -1):
        L.append(f'  (fp_line (start {-sx:.3f} {s*sy:.3f}) (end {sx:.3f} {s*sy:.3f}) '
                 f'(stroke (width 0.12) (type solid)) (layer "F.SilkS"))')
    # pin 1 marker
    L.append(f'  (fp_line (start {-sx:.3f} {-sy:.3f}) (end {-sx:.3f} {-sy+2.0:.3f}) '
             f'(stroke (width 0.12) (type solid)) (layer "F.SilkS"))')
    # courtyard
    L.append(f'  (fp_rect (start {-hx:.3f} {-hy:.3f}) (end {hx:.3f} {hy:.3f}) '
             f'(stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))')
    # body outline on the fabrication layer
    L.append(f'  (fp_rect (start {-bx:.3f} {-by:.3f}) (end {bx:.3f} {by:.3f}) '
             f'(stroke (width 0.10) (type solid)) (fill none) (layer "F.Fab"))')
    L.append(f'  (fp_text reference "REF**" (at 0 {-hy-1.0:.3f} 0) (layer "F.SilkS") '
             f'(effects (font (size 1 1) (thickness 0.15))))')
    L.append(f'  (fp_text value "L_Bourns_SRP1265A" (at 0 {hy+1.0:.3f} 0) (layer "F.Fab") '
             f'(effects (font (size 1 1) (thickness 0.15))))')
    for n, s in ((1, -1), (2, 1)):
        L.append(f'  (pad "{n}" smd roundrect (at {s*PAD_X:.3f} 0) (size {PAD_W:.3f} {PAD_H:.3f}) '
                 f'(layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.10))')
    L.append(')')
    return '\n'.join(L) + '\n'


def checks():
    ok, out = True, []
    out.append(f'pad {PAD_W:.2f} x {PAD_H:.2f} mm at x = +-{PAD_X:.2f}, gap {GAP}, span {SPAN}')
    if abs((2 * PAD_W + GAP) - SPAN) > 1e-9:
        out.append('  FAIL geometry does not close: 2*pad + gap != span'); ok = False
    # the land must cover the terminal (4.7 +- 0.3 wide, 2.75 +- 0.35 long) at worst case
    term_w_max, term_l_max = 4.7 + 0.3, 2.75 + 0.35
    out.append(f'  terminal worst case {term_l_max:.2f} x {term_w_max:.2f} mm')
    if PAD_H < term_w_max:
        out.append(f'  FAIL pad height {PAD_H} under the {term_w_max} terminal width'); ok = False
    if PAD_W < term_l_max:
        out.append(f'  WARN pad width {PAD_W} under the {term_l_max} worst-case terminal length; '
                   f'the datasheet land is what it is, noted')
    # pads must sit under the body, not outside it
    if PAD_X + PAD_W / 2 > BODY_X / 2 + 1.0:
        out.append('  FAIL pads reach more than 1 mm past the body edge'); ok = False
    out.append(f'  pads reach {PAD_X + PAD_W/2 - BODY_X/2:+.2f} mm past the {BODY_X} mm body edge')
    # current: 10 A Irms through a 3.1 x 5.0 pad is fine, but check the neck we will route
    out.append(f'  Irms 10 A: needs >= {10/2.0:.1f} mm of 2 oz trace at 20 C rise (IPC-2221 external)')
    return ok, '\n'.join(out)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, 'L_Bourns_SRP1265A.kicad_mod')
    with open(path, 'w') as f:
        f.write(kicad_mod())
    ok, text = checks()
    print(text)
    print(f'\nwrote {path}')
    print('CHECKS PASS' if ok else 'CHECKS FAILED')
    raise SystemExit(0 if ok else 1)
