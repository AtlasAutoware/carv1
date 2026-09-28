# How these mounts were made

Every part here is a Python script, not a file someone dragged around in a GUI. That was
not a stylistic choice. The car's mounting plate has 28 holes and the components have to
sit above and below it without touching each other, the chassis, or the wheels at full
lock. Checking that by eye does not work. Checking it with code does, and the check runs
again every time a number changes.

The toolchain is CadQuery (an OpenCASCADE kernel with a Python API), trimesh for reading
the STLs that came with the repo, and matplotlib for the renders. `pip install cadquery
trimesh cairosvg matplotlib pillow` and `PLATE=v2 python3 board_layout.py` rebuilds
everything from scratch, exports, and re-runs the checks.

## Measuring the system instead of guessing it

The `Templates/` directory had three files and no dimensions: `Baseplate.stl`,
`Clipless Mounting Piece.step`, `Mounting Piece.step`. So the first job was to read the
geometry back out of them.

The STEP files parse into a B-rep, so the faces carry exact numbers. Walking the 25 faces
of the clipless piece gave the whole story: a 33 x 33 mm flange 12.85 mm tall, a 28 x 28 mm
rim above it, a 24 x 24 mm pocket 14 mm deep opening upward, and two 8 mm gaps in the rim
where the clip version keeps its snap arms. The plate is 28.5 mm square holes in a 3.175 mm
sheet, so the piece goes in from underneath, the rim passes up through the hole with
0.25 mm a side, and the flange stops against the plate's underside.

That means anything mounted on the car needs a 24 x 24 peg dropping into that pocket, and
it rests on the rim tops, 1.13 mm above the plate. Nothing in the Templates says this. It
falls out of the numbers.

The Baseplate STL is not watertight, so it needed different treatment: section it at
z = 1 mm, pull the closed loops, and read the hole centres off the resulting polygons. That
gave a 3 x 5 grid at exactly 50 x 47.5 mm, two pairs of body-post cutouts, and four 3.175 mm
clamp holes. The clamps reach 22 mm below the plate, which is how the plate grips the
shock-tower tops.

## The foot, and why it exists

The first version of the Omni cradle grew its pegs straight out of the floor. It is
unprintable. The pegs point down and the component sits on top, so whichever way it goes on
the bed, one of them needs support.

The fix is a separate `clipless_foot`: a 20 x 23.8 x 12 mm peg under a 30 x 34 x 2 mm
flange. Every mount's floor has a matching through-hole with a flange recess, the foot
drops in from above, and the component sitting on the flange traps it. Both parts print
flat. One foot design serves every mount in the set, and there are twelve on a plate in
`P1S_Plate_12xFoot_CamHead_PLA.gcode` because you need a lot of them.

The pegs are 23.8 mm across the pitch direction and only 20 mm along it. That is deliberate.
The original plate's hole spacing measured 47.9, 49.0, 50.0 and 53.1 mm in different places,
so a part with two square pegs binds on every pair but one. Narrowing one axis lets a single
print fit anything from 47 to 51 mm.

Mounts that hang under the board cannot use the flange recess at all, because gravity pulls
them off. Those use a dovetail slide channel with two snap tongues instead
(`snap_retain.py`), same foot, same clearances.

## What every part has to prove

`CONVENTIONS.md` is the contract. Before a part counts as done, its script has to show:

- it is a single solid, not a pile of disconnected lumps
- zero intersection volume against its feet, the clipless pieces, the plate, and a grey
  envelope of the component placed where it really sits
- a print orientation that needs no supports, or an explicit statement of where they go
- a mass estimate from the volume

These are not decorative. The Lite-On cup's snap fingers turned out to be severed from
their own floor by the foot recess, and it was the single-solid assertion that caught it,
not a render. The Jetson tray's retaining lip was found touching a connector because the
envelope was placed at the real DC-jack offset rather than at the board edge.

Dimensions come from datasheets, with the URL in the part's MD file. Where a number could
not be found, it is a named parameter with the estimate written down and flagged. The
Lite-On brick has no public drawing at all, so its body is `BRICK_L/W/H` env vars and the MD
says so in plain language.

## Reading holes out of a drawing

NVIDIA does not publish the Orin Nano carrier's mounting-hole positions; the design files
need a developer login. The Carrier Board Specification does draw them, though. So the
figure was rendered, the embedded 951 x 772 raster pulled out, and the four circles
ring-detected: a 92 x 58 mm pattern, 2.7 mm diameter, 4 mm in from three edges and 17 mm from
the connector edge, good to about +-0.15 mm. It is recorded as measured-from-a-drawing, not
as a specification, because that is what it is.

## The layout, and the thing it proved

Individual parts passing on a stub of plate says nothing about whether they all fit
together. `board_layout.py` places every mount on the real hole grid, above and below,
adds a keep-out model of the Slash chassis built from the Traxxas manual and exploded
views (tub, motor, battery tray, servo, shock towers, body posts, tyre sweeps at full lock
and full bump), and runs the intersection test over every pair of solids in the scene.

It found that the original plate cannot carry the set. Fifteen holes, and the Omni cradle
alone covers six of them; the only under-board bay deep enough for the Jetson is exactly
where the cradle's clipless pieces land from above. Every rejected position is reproducible
(`ATTEMPT=jetson|vesc|tidy`) with the interference volume that killed it.

That is what `baseplate_v2.py` is for: same outline, same body-post cutouts and clamp holes
so it drops onto the same chassis points, 28 holes on a denser grid. On v2 all nine mounts
place at zero interference. The row pitch is 41 mm rather than 40 for one measured reason:
a mount centred on a row reaches 64.6 mm out, and a clipless flange two pitches away starts
at 63.5, which collides by 1.1 mm on exactly the holes the Jetson needs. 41 clears it by
0.9. `PITCH_Y=40 TAG=_p40` reproduces the conflict.

The lidar started at the front, moved to the middle, and went back to the front, because
the camera mast's legs were cutting the scan plane in the forward sector. Now the forward
half is clear and the only occlusion is four leg sectors totalling 27 degrees, all behind
the car, listed by bearing in `LAYOUT.md`.

## Renders

Two kinds. Shaded PNGs come from tessellating the solids and drawing them as one
Poly3DCollection, which matters: matplotlib depth-sorts within a collection but not
between them, so one collection per part draws in insertion order and the baseplate ends up
painted over everything sitting on it. Hidden-line views come from CadQuery's SVG exporter
through cairosvg, with a 180-degree correction because that exporter comes out upside down.

## Slicing

OrcaSlicer 2.4.2 from the CLI, one G-code per part, into the Bambu P1S profile with
Generic PLA at 0.20 mm, 4 walls, 30 % infill, no AMS. One trap worth recording: **Orca's
CLI does not resolve `inherits` in the bundled system profiles.** Loading
`Bambu Lab P1S 0.4 nozzle.json` directly slices silently against a 200 x 200 x 100 bed at
60 mm/s with a 2 mm3/s volumetric cap and filament density zero. The masts failed as
"outside plate" and everything else produced plausible-looking but wrong estimates. The
three inheritance chains are flattened by `flatten.py` before slicing, and every resulting
header now reads 256 x 256 x 250 at 220/55 C.

## Things that were wrong

Worth keeping a list, because the useful part of a design log is the mistakes.

- The cradle's floor windows had a `< 8 mm` guard to stop slivers. Two of the three windows
  came out 7.6 mm wide and were silently skipped, so the floor printed 81 % solid instead of
  windowed. Found by measuring the exported STL, not by looking at it.
- `CONVENTIONS.md` recorded the plate's hole pitch as irregular, from the first STEP
  measurement. The Baseplate STL is an exact 50 x 47.5 grid. Both statements are in the
  repo; the STL wins.
- The plate is 3.175 mm, not the 2 mm the templates assumed.
- Section 18 of `LAYOUT.md` claimed `PLATE_OFFSET=8.5` moved the small-board plate aft. It
  moves it forward.
- The Lite-On cup printed fine and could not take its feet: the side wall bridged the
  flange recess. Every check passed because none of them asked whether the foot could
  reach the slot. `audit_feet.py` now sweeps each foot out along every axis and measures
  what it hits; the small-board plate failed it too, a PCA9685 boss 3.3 mm over a recess
  at the 40 mm pitch.
- `board_layout.py` rebuilt the Omni cradle at 40 mm pitch in memory and proved the layout
  with it, but never wrote it out. `out/omni20_cradle.stl` stayed the 49 mm part and that
  is what was sliced and printed. The same for the VESC tray and LiPo frame. The regen
  list now includes them and the audit reads the hole pitch back out of every STL.

## Still unmeasured

The under-board mounts depend on the board-to-tub clearance, which is assumed at 45 mm. At
40 mm the Jetson tray no longer fits anywhere. That measurement, the tub rail section for
the VESC saddle, the servo's fore-aft position, and the Lite-On brick's body are the four
numbers that would turn several parameters into facts. They are listed at the end of
`LAYOUT.md` and `newcar/NEWCAR_LAYOUT.md`.
