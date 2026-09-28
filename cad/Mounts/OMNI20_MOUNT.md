# Omni 20+ cradle for the clipless mounting system

Files in `out/`: `omni20_cradle.step/.stl` (print one), `clipless_foot.step/.stl` (print two),
`omni20_assembly.step` (cradle, feet and two clipless pieces in a stub of baseplate, with the
pack), and PNG renders. Source: `omni20_mount.py`, which imports the measured system from
`clipless.py`.

## How the clipless system actually works (measured from the Templates)

The Baseplate is a 2 mm plate with 28.5 x 28.5 mm square holes. The Clipless Mounting Piece
is a 33 x 33 mm cup that goes in from below the plate: its 28 x 28 rim passes up through
the hole (0.25 mm a side), the 33 x 33 flange stops against the plate's underside, and its
24 x 24 mm pocket, 14 mm deep, opens upward through the plate. The clip version is the same
part with two snap arms living in the 8 mm rim gaps. So anything mounted on the car carries
24 x 24 pegs that drop into those pockets, and rests on the rim tops, 1.13 mm above the
plate.

The hole pitch on the Baseplate is not regular: 47.9, 49.0, 53.1 and 50.0 mm between
neighbouring holes along the car, 45.4 and 47.5 mm across. A part with two fixed 24 mm pegs
would bind on all but one pair. The pegs here are therefore 23.8 mm across the pitch
direction (light friction in the pocket) but only 20 mm along it, so one print fits any
pair from 47 to 51 mm. `PEG_PITCH=50 python3 omni20_mount.py` re-centres them for a
different pair.

## The clipless foot

The peg is not grown on the cradle. A one-piece tray has the peg on one face and the
component on the other, so it cannot be printed without supporting one of them. Instead
the cradle floor has a through-hole with a 2 mm flange recess on top, and a separate
foot (20 x 23.8 x 12 mm peg under a 30 x 34 x 2 mm flange, 0.25 mm clearance) drops
in from above. The pack sits on the flange and traps it; sideways loads go from the pack
into the posts, through the floor into the peg. Both parts print flat with no supports.
The same foot and the same cutout (`foot()` and `foot_cutout()` in `clipless.py`) are the
interface for every other mount in this set.

## The cradle

- Inside 128.2 x 123.2 mm for the 127 x 122 x 27 mm pack (0.6 mm a side). 611 g.
- Four L-shaped corner posts, 14 mm tall, 3 mm walls. Every edge is open, because the
  AC outlet, USB-C, two USB-A and the DC barrel are all mid-edge; the posts only touch
  the corners.
- Two 25 mm hook-and-loop straps across the width through four floor slots. Friction
  alone in the pockets is not enough for 611 g on a car that brakes hard; the straps
  hold the pack to the cradle and the pack's weight holds the pegs down.
- 4 mm floor with a centre window. 134 x 129 x 18 mm overall, 63 cm3, about 47 g in PLA,
  plus two feet.

Verified in CadQuery: single solid; zero interference between feet and pockets, feet and
cradle, feet and pack, cradle and plate, cradle and pack; foot flange top flush with the floor.

## Printing

Cradle floor-down, feet flange-down, no supports for either. 0.2 mm layers, 4 walls, 30 %
infill. PETG if the car lives in a hot trunk; the pack itself is rated to 40 C.

Chamfer on the peg tips is 0.8 mm, enough to find the pocket. If the pegs are loose on your
printer, raise `PEG_ACROSS` in `clipless.py` by 0.1; if they will not go in, lower it.
