# SICK TiM561 plinth (`tim561_mount.py`)

Four-foot plinth for the SICK TiM561-2050101 (part 1071419) at the front of Baseplate v2,
holes A1+B1+A2+B2, on the clipless system with the dovetail slide + snap-tongue retention of
`../snap_retain.py`. Outputs `out/tim561.step`, `out/tim561.stl`, `out/tim561_assembly.step`,
`out/tim561.png`, `out/tim561_assembly.png`, `out/tim561_lines.png`, `out/tim561_assembly_lines.png`.

## Component

Sources: SICK datasheet `dataSheet_TiM561-2050101_1071419_en.pdf`
(https://www.sick.com/media/pdf/6/46/446/dataSheet_TiM561-2050101_1071419_en.pdf, dated
2026-08-04, dimensional drawing read off the rendered page), operating instructions
TiM55x/56x/57x 8015886 (mirror: generationrobots.com/media/Operating_instructions_TiM55x_TiM56x_TiM57x_en_IM0053143.PDF),
technical information 8015883 "Mounting and electrical installation" (manualslib mirror, pp. 11, 14).

| item | value | source |
| --- | --- | --- |
| housing | 60 wide x 61 deep, 58.45 to the hood base, 85.75 to the hood top ("60 x 60 x 86") | datasheet drawing |
| scan plane | 62.46 above the bottom face (the mark on the hood is at that level) | datasheet drawing |
| mass | 250 g without cables | datasheet, OI |
| material | die-cast aluminium lower part, polycarbonate hood | OI |
| bottom-face threads | 2 x M3, 51 apart, 16.79 from the rear face, 2.8 mm deep blind, 0.8 Nm max | datasheet drawing legend |
| rear-face threads | 2 x M3, 51 apart, 24.4 above the bottom face, 2.8 deep, 0.8 Nm | datasheet drawing legend |
| swivel connector unit | rear-bottom, turns 180 degrees in 30 degree steps; horizontal: 17.37 behind the rear face (total depth 74.39); turned down: 15.4 below the bottom face (total height 101.12) | datasheet drawing, OI |
| power connector | M12 5-pin A-coded male: pin 1 brown +V, pin 2 white sync/device-ready output, pin 3 blue 0 V, 4/5 n.c. (the note that said 4-pin is wrong; the donor's brown/blue is right) | TI p.14, OI |
| Ethernet | M12 4-pin D-coded female, 100BASE-TX: 1 TX+, 2 RX+, 3 TX-, 4 RX- | OI |
| service port | micro-USB B behind a cover on the side; IP67 only with it closed | OI |
| aperture | 270 degrees (-45 to 225), 0.33 degree steps, 15 Hz; blind sector at the rear where the connectors are | datasheet |
| range | 0.05-10 m, 8 m at 10 % remission | datasheet |
| supply | 9-28 V DC, typ. 4 W; external 0.8 A slow-blow fuse; no dips under 8 V longer than 2 ms | datasheet, OI |
| environment | -25..+50 C, IP67 | datasheet |
| mounting set 1 (included) | two straight plates with M3 x 4 screws, for the rear or the bottom; without them "max. 2.8 mm into the thread" | datasheet, OI step 2 |

Not in any drawing, so ESTIMATES and parameters: the swivel unit's width `SWIVEL_W` (40) and
height `SWIVEL_H` (18); the M12 plug bodies `M12` (15 dia, 45 long behind the swivel unit, axis
9 up, at |y| 10); the hood as a cone `HOOD_R0` 29 to `HOOD_R1` 21 (the drawing shows a
conical black hood; the base radius matters for the camera line of sight, see NEWCAR_LAYOUT.md
section 3).

## What the part does

Frame: feet-pair centre at the origin, +x forward. Feet at (+-20, +-20.5) (`PEG_PITCH` 40,
`PITCH_Y` 41). Two dovetail slide channels along x, one per row, open at the rear, in a 10 mm
bar (`BAR_T`, above the 8.6 minimum; the two 40.5 mm bands are 0.5 apart so they read as one
81.5 wide bar, x -39.5..39.75). Body 68 x 66.5 (3 mm walls, 3.5 at the rear) from the floor
to a 4 mm deck at z 14 with a 2 mm keyed pocket, 0.25 mm a side, that only fits the housing
one way (the rear is open between the tabs). The lidar is shifted so its rear face is at
x -16.79: its bottom threads then fall at x 0, midway between the two feet flanges of
each channel (flanges at |x| 5..35), and are screwed from below with M3 x 6 socket heads
whose heads sit in 6 mm counterbores in the channel roof (z 7..10), above the flange path
(flanges ride at z 2..4 and sweep the whole channel when the plinth slides on; a head inside
the channel would block it). The driver reaches them through the peg slot. Engagement 2.0 mm
(max 2.8). Two rear tabs (the 3.5 mm rear wall, |y| 21.5..33.25, up to z 44.4) take M3 x 6
through 3.2 holes at (|y| 25.5, z 38.4) into the rear-face threads, engagement 2.5. Between
the tabs 43 mm is open from the deck up for the swivel connector unit and its cables, which
point aft (horizontal swivel position: turned down they would hang 15.4 into the deck).
Deck window 28 x 36, front arrow mark, 2 mm fillets on the vertical outside edges.

Scan plane: 14 + 62.46 = 76.46 above the plate top (77.59 on a 2 mm plate where the mount
sits on the rim tops); hood top 99.75. The deck cannot go lower: 10 mm bar + 4 mm deck is the
minimum over the channels, and the lidar's own scan height is 62.46.

Mass: 50.6 cm3, 63 g solid PLA (about 38 g at 60 % effective infill), plus four feet.

## Checks (run output)

| check | result |
| --- | --- |
| single solid | 1 |
| plinth x feet, at rest / pushed to the stop / pushed to the bump | 0 / 0 / 0 mm3 (fore-aft play 1.25) |
| feet x clipless pieces | 0 |
| plinth x plate stub | 0 |
| plinth x lidar envelope (housing, hood, swivel unit, M12 plugs) | 0 (the tabs clear a 40 mm swivel unit by 1.5 a side) |
| feet x envelope | 0 |
| bottom screw head above the flange path | 3.0 mm |
| screw engagement bottom / rear vs the 2.8 max | 2.0 / 2.5 |
| in the layout (NEWCAR_LAYOUT.md section 4): against the mast, Jetson tray, body posts, tyres, plate, servo | 0; mast bar 0.25 behind the plinth bar |

## Print orientation

Upright as modelled (floor down). The channel roofs are 34.5 mm bridges at z 7 and the deck
is a 4 mm plate over a hollow body with 3 mm walls: both print without supports as on the
C1 plinth and the Jetson `_under` tray (bridging), or use a 0.2 mm layer and 8 mm brim. The
rear tabs are 3.5 mm walls printed vertically; the 3.2 holes through them are horizontal
holes, fine at this size. Snap tongues are 2 mm thick along the layer direction: PETG is the
better material for the tongues if the printer has it; PLA works.

## Installation

Two feet per channel go into the from-below clipless pieces at A1, A2 (left channel) and B1,
B2 (right). The lidar is screwed to the plinth on the bench (four M3 x 6, 0.8 Nm), then the
plinth slides forward onto the feet from the rear until the tongues snap. The mast goes on
after it (its bar ends 0.25 behind the plinth's). Cables leave aft between the mast legs.
