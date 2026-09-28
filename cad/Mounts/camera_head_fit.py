"""How the Gemini 335 goes onto the camera mast head: exploded and seated, top of the mast only."""
import os, sys
os.environ.update({'PITCH_Y': '41', 'LIDAR_X': '60', 'LIDAR_Y': '-20.5', 'OUT_SUFFIX': '_v2'})
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cadquery as cq
import camera_mast_mount as C
from render import render
from jetson_orin_nano_mount import box
from PIL import Image, ImageChops

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
mast = C.mast(); head = C.saddle(); cam = C.envelope()
Z = C.Z_AXIS
# keep only the top of the mast so the drum is readable
top = mast.intersect(box(-60, 60, -60, 60, Z - 30, Z + 40))
# hardware: 1/4-20 x 1/2" socket screw up through the drum slot into the camera; two M3 x 6 through the rear tab
bolt_x = (C.X_CAM_REAR + C.X_CAM_FRONT) / 2 + (C.CAM[2] / 2 - C.CAM_TRIPOD_FROM_REAR) * 0  # slot centre
bolt = (cq.Workplane('XY').workplane(offset=Z + C.DRUM_RI - 6).circle(3.1).extrude(12.7)
        .union(cq.Workplane('XY').workplane(offset=Z + C.DRUM_RI - 6 - 4).circle(4.8).extrude(4)))
m3 = []
for sy in (-1, 1):
    m3.append(cq.Workplane('YZ', origin=(C.X_CAM_REAR - C.TAB_T - 4, 0, 0))
              .center(sy * C.CAM_M3_DY / 2, Z + C.Z_PAD + C.CAM_M3_Z).circle(1.4).extrude(6 + 4)
              .union(cq.Workplane('YZ', origin=(C.X_CAM_REAR - C.TAB_T - 4 - 2.5, 0, 0))
                     .center(sy * C.CAM_M3_DY / 2, Z + C.Z_PAD + C.CAM_M3_Z).circle(2.6).extrude(2.5)))
OR, GREEN, CAM_C, STEEL = (0.9, 0.45, 0.1), (0.2, 0.6, 0.3), (0.3, 0.3, 0.32), (0.7, 0.7, 0.75)
up = lambda s, dz, dx=0: s.translate((dx, 0, dz))
frames = [
    ('1. head onto the drum: teeth mesh, the pad slot sits over the drum slot',
     [(top, OR), (up(head, 25), GREEN), (up(cam, 60), CAM_C)]),
    ('2. 1/4-20 x 1/2 in socket screw up from inside the drum, through both slots, into the tripod socket',
     [(top, OR), (head, GREEN), (up(cam, 30), CAM_C), (up(bolt, -22), STEEL)]),
    ('3. camera down onto the pad, two M3 x 6 through the rear tab into the heatsink face; tighten the 1/4-20 last',
     [(top, OR), (head, GREEN), (cam, CAM_C), (bolt, STEEL)] + [(up(s, 0, -8), STEEL) for s in m3]),
    ('4. re-pitch: back the 1/4-20 off half a turn, lift one tooth, re-seat (shown at -15)',
     [(top, OR), (C.pitched(head, -15), GREEN), (C.pitched(cam, -15), CAM_C), (C.pitched(bolt, -15), STEEL)]
     + [(C.pitched(s, -15), STEEL) for s in m3]),
]
paths = []
for i, (t, parts) in enumerate(frames, 1):
    p = os.path.join(OUT, f'camera_head_fit{i}.png'); paths.append(p)
    render(parts, p, views=[(22, -50), (0, 90)], title=t)


def trim(im):
    bg = Image.new(im.mode, im.size, (255, 255, 255)); b = ImageChops.difference(im, bg).getbbox()
    return im.crop((b[0], max(0, b[1] - 4), b[2], b[3] + 4))
ims = [trim(Image.open(p).convert('RGB')) for p in paths]
w = max(i.width for i in ims); h = sum(i.height for i in ims) + 10 * len(ims)
o = Image.new('RGB', (w, h), 'white'); y = 0
for im in ims: o.paste(im, (0, y)); y += im.height + 10
o.save(os.path.join(OUT, 'camera_head_fit.png')); print(o.size)
print('pad top above board', round(Z + C.Z_PAD + C.RIM_PROUD, 1), 'mm; camera M3 pair', C.CAM_M3_DY, 'mm apart;',
      '1/4-20 reach into camera', round(12.7 - C.DRUM_R + C.DRUM_RI - C.PAD_T, 1), 'mm (limit', C.CAM_TRIPOD_MAX, ')')
