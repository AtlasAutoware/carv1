"""The clipless mounting system, measured from Templates/ in AtlasAutoware/atlasautoware-cad.

Baseplate: a 2 mm plate with a grid of 28.5 x 28.5 mm square holes (measured pitch is
irregular, 47.9 to 53.1 mm along the car and 45.4 to 47.5 mm across).

Clipless Mounting Piece (33 x 33 x 15.98 mm), used from BELOW the plate:
    z  0.00 .. 12.85   33 x 33 flange, sits against the underside of the plate
    z 12.85 .. 15.98   28 x 28 rim, passes up through the 28.5 hole (0.25 mm a side)
    z  2.00 .. 15.98   24 x 24 pocket, open at the top, 13.98 mm deep
    rim has 8 mm gaps at +-y where the clip version keeps its snap arms
So a mount attaches to the car by carrying 24 x 24 pegs that drop into those pockets, and
its underside rests on the rim tops, 1.13 mm above the plate.

Everything here is in millimetres. Parameters are the numbers a printer will want to tune.
"""
import cadquery as cq

PLATE_T = 2.0
HOLE = 28.5
POCKET = 24.0
POCKET_DEPTH = 13.98
RIM_TOP = 15.98          # rim top above the flange underside
PLATE_TOP = 12.85 + PLATE_T
RIM_PROUD = RIM_TOP - PLATE_TOP   # 1.13 mm the rim stands above the plate

# Peg that goes into the 24 x 24 pocket. Across the pitch direction it is a light
# friction fit; along the pitch direction it is narrowed so one part tolerates the
# plate's irregular hole spacing (+-2 mm) instead of binding.
PEG_ACROSS = 23.8
PEG_ALONG = 20.0
PEG_DEPTH = 12.0
PEG_CHAMFER = 0.8


# A separate "foot" is what most mounts should use instead of a peg grown on their
# underside: the peg is on one side of the floor and the component on the other, so a
# one-piece tray cannot be printed without supporting one of them. The foot is peg plus a
# 2 mm flange; the tray has a matching through-hole with a flange recess on top, the foot
# drops in from above, and the component sitting on the flange traps it. Every mount in
# this set uses the same foot and the same cutout.
FOOT_FLANGE = (30.0, 34.0)      # x along the pitch, y across
FOOT_FLANGE_T = 2.0
FOOT_CLR = 0.25                 # per side, so the foot drops into the recess by hand


def foot(along_axis='x'):
    """Peg plus flange. Origin at the flange top centre; peg extends down -z."""
    fx, fy = FOOT_FLANGE if along_axis == 'x' else FOOT_FLANGE[::-1]
    f = cq.Workplane('XY').rect(fx, fy).extrude(-FOOT_FLANGE_T).edges('|Z').fillet(2)
    return f.union(peg(along_axis).translate((0, 0, -FOOT_FLANGE_T)))


def foot_cutout(floor_t, along_axis='x'):
    """The solid to subtract from a tray floor (floor spans z 0..floor_t) for one foot."""
    fx, fy = FOOT_FLANGE if along_axis == 'x' else FOOT_FLANGE[::-1]
    px, py = (PEG_ALONG, PEG_ACROSS) if along_axis == 'x' else (PEG_ACROSS, PEG_ALONG)
    recess = (cq.Workplane('XY').workplane(offset=floor_t - FOOT_FLANGE_T)
              .rect(fx + 2 * FOOT_CLR, fy + 2 * FOOT_CLR).extrude(FOOT_FLANGE_T + 1)
              .edges('|Z').fillet(2 + FOOT_CLR))
    through = cq.Workplane('XY').rect(px + 2 * FOOT_CLR, py + 2 * FOOT_CLR).extrude(floor_t).translate((0, 0, -0.5))
    return recess.union(through)


def peg(along_axis='x'):
    """A peg, origin at its top centre, extending down -z."""
    w, l = (PEG_ALONG, PEG_ACROSS) if along_axis == 'x' else (PEG_ACROSS, PEG_ALONG)
    p = cq.Workplane('XY').rect(w, l).extrude(-PEG_DEPTH)
    return p.faces('<Z').chamfer(PEG_CHAMFER)


def clipless_piece():
    """The template part itself, flange underside at z=0, for assemblies and renders."""
    body = cq.Workplane('XY').rect(33, 33).extrude(12.85)
    rim = cq.Workplane('XY').workplane(offset=12.85).rect(28, 28).extrude(RIM_TOP - 12.85)
    body = body.union(rim)
    pocket = cq.Workplane('XY').workplane(offset=2.0).rect(POCKET, POCKET).extrude(RIM_TOP)
    body = body.cut(pocket)
    for sy in (1, -1):
        gap = cq.Workplane('XY').workplane(offset=12.85).center(0, sy * 13).rect(8, 2.2).extrude(4)
        body = body.cut(gap)
    return body


def plate_stub(holes, size=(140, 90), t=PLATE_T):
    """A piece of baseplate with holes at the given (x, y) centres, top face at z=0."""
    p = cq.Workplane('XY').rect(*size).extrude(-t)
    for (x, y) in holes:
        p = p.cut(cq.Workplane('XY').center(x, y).rect(HOLE, HOLE).extrude(-t - 1).translate((0, 0, 0.5)))
    return p
