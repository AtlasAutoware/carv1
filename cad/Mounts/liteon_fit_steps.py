"""Three-step picture of how the Lite-On brick goes into its cup next to the Omni cradle.
Builds both parts at the Baseplate v2 configuration and renders: (1) cup and cradle on
their clipless pieces, feet dropping in from above; (2) brick plugged into the pack, the
pair lowered together; (3) everything seated, the brick snapped under the fingers."""
import os, sys
os.environ.update({'PEG_PITCH': '40', 'PITCH_Y': '82', 'FEET_X_OFFSET': '13.5', 'RETAIN': 'recess', 'OUT_SUFFIX': '_v2'})
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cadquery as cq
import liteon_45w_brick_mount as L
os.environ['PITCH_Y'] = '41'
import omni20_mount as O
from clipless import clipless_piece, plate_stub, RIM_PROUD, RIM_TOP
from render import render

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
cup = L.cup(); brick = L.envelope(); cup_feet = L.feet()
# the cradle sits with its outlet edge toward the cup: cradle centre at -brick_centre_x
g = L.assumed_geometry(); dx = -g['brick_centre_x']
cradle = O.cradle().translate((dx, 0, 0)); cradle_feet = [f.translate((dx, 0, 0)) for f in O.feet()]
pack = (cq.Workplane('XY').rect(*O.OMNI[:2]).extrude(O.OMNI[2]).edges('|Z').fillet(8)
        .translate((dx, 0, O.FLOOR)))
holes = [(x, y) for (x, y) in L.FEET] + [(dx + sx * O.PEG_PITCH / 2, 0) for sx in (-1, 1)]
stub = plate_stub([(x - (dx - 70) / 2, y) for (x, y) in holes], size=(215, 104)).translate(((dx - 70) / 2 + 3, 0, -RIM_PROUD))
pieces = [clipless_piece().translate((x, y, -RIM_TOP)) for (x, y) in holes]
GREY, ORANGE, BLUE, DARK, BRICK = (0.55, 0.55, 0.6), (0.9, 0.45, 0.1), (0.2, 0.5, 0.9), (0.15, 0.15, 0.15), (0.35, 0.35, 0.35)
up = lambda s, z: s.translate((0, 0, z))
base = [(stub, GREY)] + [(p, (0.63, 0.63, 0.63)) for p in pieces] + [(cup, ORANGE), (cradle, ORANGE)]
steps = [
    ('1. cup and cradle on the plate, feet drop in from above',
     base + [(up(f, 18), BLUE) for f in cup_feet + cradle_feet]),
    ('2. plug the brick into the pack first, then lower the pair together',
     base + [(f, BLUE) for f in cup_feet + cradle_feet] + [(up(brick, 40), BRICK), (up(pack, 40), DARK)]),
    ('3. seated: pack in its posts, brick under the two snap fingers, prongs in the outlet',
     base + [(f, BLUE) for f in cup_feet + cradle_feet] + [(brick, BRICK), (pack, DARK)]),
]
for i, (title, parts) in enumerate(steps, 1):
    render(parts, os.path.join(OUT, f'liteon_fit_step{i}.png'), views=[(30, -35), (8, 90)], title=title)
print('ok', g['gap_cup_to_cradle_wall'])
