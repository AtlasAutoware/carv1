"""Positive retention for mounts that hang UNDER the baseplate.

The clipless piece can be put into a plate hole from above as well as from below: flange
on the plate top, rim down through the hole, pocket opening downward. A foot then plugs in
from below and the mount hangs on the feet. Gravity pulls a hanging mount off its feet, so
the drop-in flange recess of clipless.foot_cutout (which relies on the component sitting on
the flange) is not enough. This module turns the two-foot flange-recess floor of a tray into
the dovetail slide channel + spring-tongue scheme of rplidar_c1_mount.py (whose helpers it
imports), which works in any orientation: feet into the pockets first, tray slides on over
both flanges to the end stop, the tongues snap up behind the trailing flange, the 45 degree
lips stop the flanges leaving the channel.

    bar, cuts, adds = channel_solids(feet, axis)
    tray = tray.union(bar)          # solid band the channel is cut through
    for c in cuts: tray = tray.cut(c)
    for a in adds: tray = tray.union(a)

The channel needs a floor at least SNAP_FLOOR (= 7 mm channel + roof) thick in a band
BAND_W wide along the foot line; `bar` provides it. The tray floor outside the band can stay
whatever it was.
"""
import cadquery as cq
from clipless import foot, clipless_piece, RIM_TOP, PLATE_T, FOOT_FLANGE, FOOT_CLR
from rplidar_c1_mount import slide_channel, snap_tongues, CH_LIP_Z0, CH_LIP_Z1, TONGUE_L, TONGUE_SLOT
from jetson_orin_nano_mount import box

ROOF = 1.6                                   # bridged over the channel, 8 layers at 0.2
SNAP_FLOOR = CH_LIP_Z1 + ROOF                # 8.6, minimum floor thickness in the band
WALL = 3.0
BAND_W = FOOT_FLANGE[1] + 2 * FOOT_CLR + 2 * WALL    # 40.5
HALF_FLANGE = FOOT_FLANGE[0] / 2             # 15, along the channel
TIP_BEYOND_BUMP = 3.5                        # tongue tip beyond the bump's steep face
STOP_CLR = 0.75                              # end stop clear of the leading flange
STOP_WALL = 4.0
FOOT_Z = CH_LIP_Z0                           # 4.0: foot origin (flange top) above the floor underside
UNDER_Z = -(RIM_TOP - 12.85)                 # -3.13: hanging mount's contact face below the plate top


def channel_solids(feet, axis='x', floor_t=SNAP_FLOOR, band_t=None, open_end='-', extent=None):
    """Solids for a slide channel through two feet on a line along `axis`.

    feet: two (x, y) foot centres in tray coordinates, same y (axis 'x') or same x ('y').
    open_end: '-' the tray slides on from the -axis side (tongues at the -axis foot), '+'
    the other way. extent: (s_min, s_max) of the tray floor along the axis; the bar spans
    it (default: from the tongue tips to the stop wall). band_t: bar height (default
    floor_t). Returns (bar, cuts, adds) in tray coordinates, floor underside at z=0.
    """
    band_t = floor_t if band_t is None else band_t
    assert floor_t >= SNAP_FLOOR - 1e-6, f'floor {floor_t} thinner than SNAP_FLOOR {SNAP_FLOOR}'
    ax = 0 if axis == 'x' else 1
    s = sorted(p[ax] for p in feet)
    t = feet[0][1 - ax]
    assert abs(feet[0][1 - ax] - feet[1][1 - ax]) < 1e-6, 'feet must be on one line'
    flip = open_end == '+'
    if flip:                       # build mirrored along the axis, then mirror back
        s = [-v for v in s][::-1]
    s_rear, s_front = s
    s_stop = s_front + HALF_FLANGE + STOP_CLR
    s_bump = s_rear - HALF_FLANGE - 1.0
    s_tip = s_bump - TIP_BEYOND_BUMP
    if extent is None:
        s0, s1 = s_tip, s_stop + STOP_WALL
    else:
        s0, s1 = (sorted([-extent[0], -extent[1]]) if flip else extent)
        # the band bar reaches the tongue tips and the stop wall even if the floor is shorter
        s0, s1 = min(s0, s_tip), max(s1, s_stop + STOP_WALL)
    bar = box(s0, s1, -BAND_W / 2, BAND_W / 2, 0, band_t)
    cuts = [slide_channel(s0 - 1, s_stop)]
    tcuts, adds = snap_tongues(s_tip, s_bump)
    cuts += tcuts
    # end slit so the tongue tip is free even where the floor continues past it
    hp = 23.8 / 2 + FOOT_CLR
    hf = FOOT_FLANGE[1] / 2 + FOOT_CLR
    for sy in (-1, 1):
        ya, yb = sorted((sy * (hp - 0.5), sy * hf))
        cuts.append(box(s_tip - TONGUE_SLOT, s_tip, ya, yb, -1, CH_LIP_Z0 - 1.4))
    xf = lambda sol: sol.mirror('YZ') if flip else sol
    if axis == 'y':
        rot = lambda sol: xf(sol).rotate((0, 0, 0), (0, 0, 1), 90).translate((t, 0, 0))
    else:
        rot = lambda sol: xf(sol).translate((0, t, 0))
    return rot(bar), [rot(c) for c in cuts], [rot(a) for a in adds]


def snap_feet(feet, axis='x'):
    """Feet seated in the channel, flange on the ledge (z 2..4)."""
    return [foot(axis).translate((x, y, FOOT_Z)) for (x, y) in feet]


def play_check(part, feet_solids, feet, axis, inter):
    """Intersection of the part with its feet pushed to the stop and to the bump."""
    ax = 0 if axis == 'x' else 1
    d_stop = STOP_CLR - FOOT_CLR
    d_bump = 1.0 - FOOT_CLR
    v = lambda d: (d, 0, 0) if axis == 'x' else (0, d, 0)
    at_stop = sum(inter(part.translate(v(-d_stop)), f) for f in feet_solids)
    at_bump = sum(inter(part.translate(v(d_bump)), f) for f in feet_solids)
    return at_stop, at_bump, d_stop + d_bump


def hang(shape, dz=0.0):
    """Turn an upright tray (floor underside at z=0) into a hanging one under the plate:
    rotated 180 degrees about x, contact face at UNDER_Z. (x, y, z) -> (x, -y, -z)."""
    return shape.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, UNDER_Z + dz))


def piece_from_above():
    """Clipless piece inserted from above: flange on the plate top, rim down the hole."""
    return clipless_piece().rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, 12.85))


def under_assembly(name, tray, feet_solids, holes, envelope, stub, colors=None):
    """Assembly of a hanging tray: plate stub (top at z=0), clipless pieces from above,
    tray + feet + envelope hanging under it."""
    colors = colors or {}
    a = cq.Assembly(name=name)
    a.add(stub, name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(holes):
        a.add(piece_from_above().translate((x, y, 0)), name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(hang(tray), name='tray', color=cq.Color(*colors.get('tray', (0.9, 0.45, 0.1))))
    for i, f in enumerate(feet_solids):
        a.add(hang(f), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    if envelope is not None:
        a.add(hang(envelope), name='envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def under_shaded(tray, feet_solids, holes, envelope, stub):
    parts = [(stub, (0.55, 0.55, 0.6))]
    parts += [(piece_from_above().translate((x, y, 0)), (0.63, 0.63, 0.63)) for (x, y) in holes]
    parts += [(hang(tray), (0.9, 0.45, 0.1))] + [(hang(f), (0.2, 0.5, 0.9)) for f in feet_solids]
    if envelope is not None:
        parts.append((hang(envelope), (0.35, 0.35, 0.35)))
    return parts
