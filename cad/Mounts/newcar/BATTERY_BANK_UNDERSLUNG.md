# Underslung power bank tray (`battery_bank_underslung.py`)

Snap-retained tray that hangs a USB-C PD power bank under Baseplate v2 on four feet plugged
into clipless pieces inserted from above (`../snap_retain.py`), holes B5+C5+B6+C6. The same
part used upright above the plate is the `LAYOUT=above` fallback (NEWCAR_LAYOUT.md section
5). Outputs `out/battery_bank_underslung.step` / `.stl` / `_assembly.step` / `.png` /
`_assembly.png` / `_lines.png` / `_assembly_lines.png`.

## Component

Baseus Blade HD 100 W, 20,000 mAh (PPBL000301): 133.9 x 133.9 x 17.8 mm, 445 g, ports on
one edge, USB-C 100 W PD (https://www.baseus.com/products/blade-laptop-power-bank-100w-20000mah,
which lists both Blade models; USD 69.99, out of stock there on 2026-09-05, stocked at
Walmart/Amazon). The PDO list confirmed for the Blade 100 W (PPBL000001, 161.8 x 133.9 x 17.8,
506 g, USD 129.99) is 5 V 3 A / 9 V 3 A / 12 V 3 A / 15 V 3 A / 20 V 5 A
(androidcentral.com review); the HD shares the 100 W rating, but confirm 15 V 3 A on the HD
before buying (a PD tester, or the Adafruit trigger cable: it outputs 5 V if the PDO is
missing). `BANK_L` / `BANK_W` / `BANK_T` / `BANK_MASS` are parameters; `BANK_L=161.8` is the
Blade 100 W and does NOT fit the car (NEWCAR_LAYOUT.md section 4). The USB-C port position on
the port edge (`PORT`, y -30) is an ESTIMATE.

Why this bank: it is the only one found with a 15 V 3 A PDO and under 30 mm thick. Anker 737
/ PowerCore 24K (155.7 x 54.6 x 49.5, 630 g), Anker Prime 20K (124 x 53 x 48) are twice too
thick to hang under the plate; the Omni 20+ (127 x 122 x 27, 611 g, 15 V 3 A on USB-C) would
fit the same tray at `BANK_T=27` (41 mm deep, too deep over the battery bay at UC 45) and
weighs 166 g more.

## What the part does

Built upright like the Jetson tray, hung by `snap_retain.hang()`. 4 mm floor (140.9 square,
windows outside the channel bands), two 8.6 mm channel bars along x on rows B and C (y
+-20.5, 40.5 wide each, 0.5 apart), open at -x; the bank lies on the bars (its top 8.6 below
the tray's contact face) with 0.5 mm a side inside 3 mm full-height side walls (+-y, three
40 x 12 windows each), two corner legs with 1.5 mm fixed lips at the +x end and two 2 mm x
20 mm snap fingers with 1 mm lips at the -x end (the VESC tray's finger section, for a 445 g
load; the Jetson's 1.6 mm fingers carry 250 g). Both ends are otherwise open: the ports face
+x, and the bank's rear face is exposed between the fingers. The outline is shifted 4 mm
forward on the feet (`BANK_X_OFFSET`) so the tray's rear end (x -131.45 on the car) clears
the plate's moved rear clamp blocks (2.49 mm) and the rear shock caps.

Height 28.7 (8.6 + 17.8 + 0.3 + 2 lip); hanging, the tray bottom is 31.88 below the plate
top (28.7 below the underside of the 3.175 plate). Mass 82.6 cm3, 102 g solid, about 61 g
at 60 % infill, plus four feet.

The bank clicks into the tray on the bench (hook under the fixed lips, press down past the
fingers), the tray then slides forward onto its four feet from the rear and the tongues snap
behind the rear feet. To take the bank out, unclip the fingers with the tray on the car
(they face aft, reachable from behind the plate) or slide the tray off. No straps: a strap
through the floor would have to pass between the tray and the plate, and one around the
outside has nothing to close on; the two fingers plus two fixed lips hold 445 g x 5 g = 22 N
against 1.0 and 1.5 mm lips, the same margin the VESC tray has on 380 g of metal case.

## Checks (run output)

| check | result |
| --- | --- |
| single solid | 1 |
| tray x feet, at rest / at the stop / at the bump | 0 / 0 / 0 (play 1.25) |
| feet x from-above clipless pieces (hung) | 0 |
| tray x plate stub (hung) | 0 |
| tray x bank envelope (bank + USB-C plug stub) | 0 |
| feet x envelope | 0 |
| in the layout (NEWCAR_LAYOUT.md section 4): against the plate, clamp blocks, towers, shock caps, tidy nuts, saddle, mast pieces, battery bay | 0; **transmission and motor models: 5.7 / 6.7 mm into them (estimates, measure)** |
| `LAYOUT=above`, upright on B5+C5+B6+C6 | 0 against everything (66 solids) |

## Print orientation

Upright as modelled (floor down, fingers up). Channel roofs are 34.5 mm bridges at z 7 over
the channels, as on every snap tray in the set; the side-wall windows are horizontal
bridges of 40 mm at 12 mm height: fine at 0.2 mm layers, or add supports in the windows if
the printer sags. No other overhangs.

## Installation

Four clipless pieces go into B5, C5, B6, C6 from above (flange on the plate top, rim down);
four feet plug into them from below. With the bank clicked in, slide the tray forward onto
the feet from behind the plate; it stops 0.75 mm short of the front flanges and the tongues
snap. Plug the PD trigger cable into the bank's USB-C1 first (the port edge faces forward and
is 6..31 mm ahead of the tray's front legs once on the car). The rear body posts must be
off: they would pass through the tray.
