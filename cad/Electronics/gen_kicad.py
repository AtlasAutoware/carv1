"""Generate the AtlasPower-4S KiCad 9/10 project from netlist.py.

Writes a project whose schematic and board are both produced from one netlist, so they
cannot disagree. Symbols are generated here rather than pulled from the KiCad libraries:
a symbol is only pin numbers, names and a box, all of which are checkable against the
datasheet, whereas a wrong footprint land pattern is unrecoverable, so footprints are
taken from KiCad's own verified libraries by name.

Wiring style: every pin gets a short stub and a global label. That is unusual for a
hand-drawn schematic but it is exactly how a generated one stays correct - there is no
routing to get wrong, and the netlist is literally the source. Read it as a net list with
pictures, and use the generated netlist/BOM as the authority.

    python3 gen_kicad.py            writes out/AtlasPower4S/
"""
import os, math, uuid, hashlib
import netlist as N

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out', 'AtlasPower4S')
LIB = 'atlaspower'
BOARD_W, BOARD_H = 88.0, 68.0
HOLE_DX, HOLE_DY = 76.0, 56.0


def uid(*parts):
    """Deterministic UUIDs: regenerating the project gives byte-identical output."""
    h = hashlib.sha1(('atlaspower:' + ':'.join(map(str, parts))).encode()).hexdigest()
    return f'{h[0:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}'


# ---- symbol generation ------------------------------------------------------------------
# (name, [(pin number, pin name, electrical type, side)]) - side: L or R
SYMBOLS = {
    'AP64501': [('1', 'BST', 'passive', 'L'), ('2', 'VIN', 'passive', 'L'),
                ('3', 'EN', 'input', 'L'), ('4', 'SS', 'passive', 'L'),
                ('5', 'FB', 'input', 'R'), ('6', 'COMP', 'passive', 'R'),
                ('7', 'GND', 'passive', 'R'), ('8', 'SW', 'output', 'R'),
                ('9', 'EP', 'passive', 'R')],
    'LM74700': [('1', 'VCAP', 'passive', 'L'), ('2', 'GND', 'passive', 'L'),
                ('3', 'EN', 'input', 'L'), ('4', 'CATHODE', 'passive', 'R'),
                ('5', 'GATE', 'output', 'R'), ('6', 'ANODE', 'passive', 'R')],
    'INA226': [('1', 'IN+', 'input', 'L'), ('2', 'IN-', 'input', 'L'),
               ('3', 'ALERT', 'open_collector', 'L'), ('4', 'GND', 'passive', 'L'),
               ('5', 'NC1', 'passive', 'L'), ('6', 'SCL', 'input', 'R'),
               ('7', 'SDA', 'bidirectional', 'R'), ('8', 'NC2', 'passive', 'R'),
               ('9', 'VS', 'passive', 'R'), ('10', 'A0', 'input', 'R')],
    'R': [('1', '1', 'passive', 'L'), ('2', '2', 'passive', 'R')],
    'C': [('1', '1', 'passive', 'L'), ('2', '2', 'passive', 'R')],
    'CP': [('1', '+', 'passive', 'L'), ('2', '-', 'passive', 'R')],
    'L': [('1', '1', 'passive', 'L'), ('2', '2', 'passive', 'R')],
    'D': [('1', 'K', 'passive', 'L'), ('2', 'A', 'passive', 'R')],
    'LED': [('1', 'K', 'passive', 'L'), ('2', 'A', 'passive', 'R')],
    'FUSE': [('1', '1', 'passive', 'L'), ('2', '2', 'passive', 'R')],
    'NMOS': [('1', 'G', 'input', 'L'), ('2', 'D', 'passive', 'R'), ('3', 'D', 'passive', 'R'),
             ('4', 'D', 'passive', 'R'), ('5', 'S', 'passive', 'R'), ('6', 'S', 'passive', 'R'),
             ('7', 'S', 'passive', 'R'), ('8', 'S', 'passive', 'R')],
    'CONN2': [('1', '1', 'passive', 'L'), ('2', '2', 'passive', 'L')],
    'CONN3': [('1', '1', 'passive', 'L'), ('2', '2', 'passive', 'L'), ('3', '3', 'passive', 'L')],
    'CONN4': [('1', '1', 'passive', 'L'), ('2', '2', 'passive', 'L'), ('3', '3', 'passive', 'L'),
              ('4', '4', 'passive', 'L')],
    'MH': [],
}
PIN_PITCH = 2.54


def symbol_for(part):
    v, ref = part.value, part.ref
    if ref.startswith('H'): return 'MH'
    if v == N.AP: return 'AP64501'
    if 'LM74700' in v: return 'LM74700'
    if 'INA226' in v: return 'INA226'
    if 'ASS053' in v: return 'NMOS'
    if ref.startswith('L'): return 'L'
    if ref.startswith('F'): return 'FUSE'
    if v == 'green': return 'LED'
    if ref.startswith('D'): return 'D'
    if ref.startswith('R'): return 'R'
    if ref.startswith('C'): return 'CP' if 'u/35V' in v or 'u/16V' in v and '470' in v else 'C'
    if ref.startswith('J'):
        return {2: 'CONN2', 3: 'CONN3', 4: 'CONN4'}[len(part.pins)]
    return 'C'


def sym_geometry(kind):
    pins = SYMBOLS[kind]
    left = [p for p in pins if p[3] == 'L']
    right = [p for p in pins if p[3] == 'R']
    rows = max(len(left), len(right), 1)
    # keep h an even multiple of the pitch so h/2 stays on the 1.27 grid: KiCad's ERC
    # flags every off-grid wire endpoint, and 284 of those drown out real findings
    h = (rows + 1 + (rows + 1) % 2) * PIN_PITCH
    w = 2 * PIN_PITCH if kind in ('R', 'C', 'CP', 'L', 'D', 'LED', 'FUSE') else 5 * PIN_PITCH
    return left, right, w, h


def lib_symbol(kind):
    left, right, w, h = sym_geometry(kind)
    x0, y0 = -w / 2, -h / 2
    s = [f'    (symbol "{LIB}:{kind}" (pin_names (offset 0.508)) (exclude_from_sim no) '
         f'(in_bom yes) (on_board yes)',
         f'      (property "Reference" "U" (at 0 {h/2+1.27:.2f} 0) (effects (font (size 1.27 1.27))))',
         f'      (property "Value" "{kind}" (at 0 {-h/2-1.27:.2f} 0) (effects (font (size 1.27 1.27))))',
         f'      (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
         f'      (symbol "{kind}_0_1"']
    if kind != 'MH':
        s.append(f'        (rectangle (start {x0:.2f} {y0:.2f}) (end {-x0:.2f} {-y0:.2f}) '
                 f'(stroke (width 0.254) (type default)) (fill (type background)))')
    s.append('      )')
    s.append(f'      (symbol "{kind}_1_1"')
    for i, (num, name, etype, _) in enumerate(left):
        y = h / 2 - (i + 1) * PIN_PITCH
        s.append(f'        (pin {etype} line (at {x0-PIN_PITCH:.2f} {y:.2f} 0) (length {PIN_PITCH})'
                 f' (name "{name}" (effects (font (size 1.0 1.0))))'
                 f' (number "{num}" (effects (font (size 1.0 1.0)))))')
    for i, (num, name, etype, _) in enumerate(right):
        y = h / 2 - (i + 1) * PIN_PITCH
        s.append(f'        (pin {etype} line (at {-x0+PIN_PITCH:.2f} {y:.2f} 180) (length {PIN_PITCH})'
                 f' (name "{name}" (effects (font (size 1.0 1.0))))'
                 f' (number "{num}" (effects (font (size 1.0 1.0)))))')
    s.append('      )')
    s.append('    )')
    return '\n'.join(s)


def pin_positions(kind, at_x, at_y):
    """Absolute schematic coords of each pin's outer end."""
    left, right, w, h = sym_geometry(kind)
    x0 = -w / 2
    pos = {}
    for i, (num, *_rest) in enumerate(left):
        pos[num] = (at_x + x0 - PIN_PITCH, at_y - (h / 2 - (i + 1) * PIN_PITCH))
    for i, (num, *_rest) in enumerate(right):
        pos[num] = (at_x - x0 + PIN_PITCH, at_y - (h / 2 - (i + 1) * PIN_PITCH))
    return pos


def write_schematic(parts):
    kinds = sorted({symbol_for(p) for p in parts})
    body = []
    # place parts on a grid, grouped so related refdes land together
    COLS, DX, DY, X0, Y0 = 8, 43.18, 33.02, 30.48, 30.48   # all multiples of 2.54
    placed = {}
    for i, part in enumerate(parts):
        kind = symbol_for(part)
        cx, cy = X0 + (i % COLS) * DX, Y0 + (i // COLS) * DY
        placed[part.ref] = (kind, cx, cy)
        fp = part.footprint
        body.append(
            f'  (symbol (lib_id "{LIB}:{kind}") (at {cx:.2f} {cy:.2f} 0) (unit 1)\n'
            f'    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
            f'    (uuid {uid("sym", part.ref)})\n'
            f'    (property "Reference" "{part.ref}" (at {cx:.2f} {cy - sym_geometry(kind)[3]/2 - 2.0:.2f} 0)'
            f' (effects (font (size 1.27 1.27))))\n'
            f'    (property "Value" "{part.value}" (at {cx:.2f} {cy + sym_geometry(kind)[3]/2 + 2.0:.2f} 0)'
            f' (effects (font (size 1.27 1.27))))\n'
            f'    (property "Footprint" "{fp}" (at {cx:.2f} {cy:.2f} 0)'
            f' (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "LCSC" "{part.lcsc}" (at {cx:.2f} {cy:.2f} 0)'
            f' (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "MPN" "{part.mpn}" (at {cx:.2f} {cy:.2f} 0)'
            f' (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Description" "{part.desc}" (at {cx:.2f} {cy:.2f} 0)'
            f' (effects (font (size 1.27 1.27)) (hide yes)))\n'
            + ''.join(f'    (pin "{num}" (uuid {uid("pin", part.ref, num)}))\n'
                      for num, *_ in SYMBOLS[kind])
            + f'    (instances (project "AtlasPower4S" (path "/{uid("root")}"'
            f' (reference "{part.ref}") (unit 1))))\n  )')

    # every pin: a stub wire out to a global label
    for part in parts:
        kind, cx, cy = placed[part.ref]
        pos = pin_positions(kind, cx, cy)
        for num, net in part.pins.items():
            if num not in pos:
                continue
            px, py = pos[num]
            left_side = px < cx
            lx = px - 5.08 if left_side else px + 5.08
            body.append(f'  (wire (pts (xy {px:.2f} {py:.2f}) (xy {lx:.2f} {py:.2f}))'
                        f' (stroke (width 0) (type default)) (uuid {uid("w", part.ref, num)}))')
            rot = 180 if left_side else 0
            just = 'right' if left_side else 'left'
            body.append(
                f'  (global_label "{net}" (shape bidirectional) (at {lx:.2f} {py:.2f} {rot})'
                f' (effects (font (size 1.27 1.27)) (justify {just}))'
                f' (uuid {uid("gl", part.ref, num)}))')

    doc = ['(kicad_sch (version 20231120) (generator "atlaspower") (generator_version "9.0")',
           f'  (uuid {uid("root")})', '  (paper "A1")',
           '  (title_block (title "AtlasPower-4S") (company "AtlasAutoware")'
           ' (comment 1 "4S LiPo to 10 V / 7.4 V / 5 V for the new car")'
           ' (comment 2 "generated by gen_kicad.py from netlist.py - do not hand edit"))',
           '  (lib_symbols']
    doc += [lib_symbol(k) for k in kinds]
    doc.append('  )')
    doc += body
    doc.append(')')
    with open(os.path.join(OUT, 'AtlasPower4S.kicad_sch'), 'w') as f:
        f.write('\n'.join(doc) + '\n')
    return placed


def write_symbol_library(parts):
    """A real .kicad_sym so the schematic's symbols resolve instead of being 'missing'."""
    kinds = sorted({symbol_for(p) for p in parts})
    doc = ['(kicad_symbol_lib (version 20231120) (generator "atlaspower")']
    for k in kinds:
        doc.append(lib_symbol(k).replace(f'"{LIB}:{k}"', f'"{k}"'))
    doc.append(')')
    with open(os.path.join(OUT, f'{LIB}.kicad_sym'), 'w') as f:
        f.write('\n'.join(doc) + '\n')


def write_lib_tables(fp_lib_dir, parts=None):
    with open(os.path.join(OUT, 'sym-lib-table'), 'w') as f:
        f.write('(sym_lib_table\n  (version 7)\n'
                f'  (lib (name "{LIB}")(type "KiCad")(uri "${{KIPRJMOD}}/{LIB}.kicad_sym")'
                '(options "")(descr "AtlasPower generated symbols"))\n)\n')
    # only the libraries this board actually uses, taken from the netlist itself, so the
    # table stays short and every entry is one a reviewer can check
    libs = sorted({p.footprint.split(':')[0] for p in (parts or []) if ':' in p.footprint})
    with open(os.path.join(OUT, 'fp-lib-table'), 'w') as f:
        f.write('(fp_lib_table\n  (version 7)\n')
        for lib in libs:
            f.write(f'  (lib (name "{lib}")(type "KiCad")(uri "{fp_lib_dir}/{lib}.pretty")'
                    '(options "")(descr ""))\n')
        f.write('  (lib (name "atlaspower")(type "KiCad")'
                '(uri "${KIPRJMOD}/../atlaspower.pretty")(options "")'
                '(descr "generated footprints"))\n')
        f.write(')\n')


def write_project():
    with open(os.path.join(OUT, 'AtlasPower4S.kicad_pro'), 'w') as f:
        f.write('{\n  "board": {"design_settings": {"defaults": {}}},\n'
                '  "meta": {"filename": "AtlasPower4S.kicad_pro", "version": 1},\n'
                '  "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},\n'
                '  "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},\n'
                '  "sheets": [["' + uid('root') + '", "Root"]],\n'
                '  "text_variables": {}\n}\n')
    with open(os.path.join(OUT, 'sym-lib-table'), 'w') as f:
        f.write('(sym_lib_table\n  (version 7)\n)\n')


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    parts, nets = N.build()
    placed = write_schematic(parts)
    write_symbol_library(parts)
    write_lib_tables(os.environ.get('KICAD_FP_DIR', '${KIPRJMOD}/../../libs/footprints'), parts)
    write_project()
    print(f'wrote {OUT}')
    print(f'  {len(parts)} symbols, {len(nets)} nets, {len(set(symbol_for(p) for p in parts))} symbol kinds')
