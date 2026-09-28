# Conventions for mounts in this set

Every mount attaches to the car through the clipless mounting system measured in
`clipless.py` (read its docstring first). Use the foot interface, not pegs grown on
the part: subtract `foot_cutout(FLOOR, axis)` from your floor for each foot position and
the printed `clipless_foot` drops in from above. Feet sit in clipless pieces whose pockets
are 24 x 24; neighbouring plate holes are 47 to 53 mm apart along the car and 45 to 48 mm
across, so put feet on a parametric pitch (env var, default 49.0 along / 47.0 across) and
prefer two feet along the car for anything heavier than 100 g, four for anything heavier
than 400 g. Feet must land on at least a 36 mm wide bar of solid floor.

Layout: +x is the car's long axis, +z up, floor spans z 0..FLOOR, the part's underside
rests on the clipless rim tops. Floors are 4 mm unless there is a reason.

Every part: one script `<part>_mount.py` next to `clipless.py`, importing it, that writes
`out/<part>.step`, `out/<part>.stl`, `out/<part>_assembly.step` (part + feet + clipless
pieces in a `plate_stub` + a grey envelope of the component), and `out/<part>.png` via
`render.py` (two views) plus an `out/<part>_lines.png` hidden-line view (see the end of
`omni20_mount.py` history for the SVG export call). Then a `<PART>_MOUNT.md` that gives
the component dimensions WITH THE SOURCE URL they came from, what the mount does, the
verified checks, and print orientation. No emoji, no marketing tone.

Verification that must run and be reported, in CadQuery, before a part is called done:
- single solid (`len(part.val().Solids()) == 1`)
- zero intersection volume between: part and feet, feet and clipless pieces, part and
  plate stub, part and the component envelope (place the envelope where it really sits)
- print orientation is support-free, or say exactly where supports go
- every foot can actually be installed: zero material in the column above a flange recess
  (recess mounts), or a clear slide path to the open end of the channel apart from the
  snap-tongue bumps (channel mounts). `python3 audit_feet.py` runs this over the set;
  a part passes the interference checks and still fails this one when a wall or a boss
  bridges the slot. Two printed parts did.
- the exported STL has its feet at the pitch of the plate it is for. The model being
  rebuilt at the right pitch inside `board_layout.py` does not update `out/`; the part's
  own script has to be run with `OUT_SUFFIX=_v2`. `audit_feet.py` measures the holes in
  the STL files too.
- mass estimate from volume

Design rules: 3 mm minimum wall, 0.5 mm envelope clearance for snap-free drop-in, 0.25 mm
for things that should be snug, 2 mm fillets on vertical outside edges, all cable ports
left open, strap slots 26 x 4.5 mm for 25 mm hook-and-loop wherever weight or vibration
says friction is not enough. Screws only where the component has threaded inserts or
through-holes that were made for them; then use M2.5 or M3 heat-set inserts (4.0 mm hole
for M3, 3.5 mm for M2.5, 5 mm deep bosses).

Dimensions of components must come from the manufacturer's datasheet or drawing, fetched
and cited. If a dimension cannot be found, make it a named parameter with the best
estimate and say so in the MD file. Never guess a mounting-hole pattern silently.
