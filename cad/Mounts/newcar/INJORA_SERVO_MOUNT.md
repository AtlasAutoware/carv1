# INJORA INJS235 in the Slash 4x4 servo position: nothing to print

## The servo

INJORA INJS235 35 kg waterproof brushless servo (https://www.injora.com/products/injora-injs235-35kg-waterproof-brushless-servo-large-torque-digital-servo,
spec list and dimension drawing on that page):

| item | value |
| --- | --- |
| case | 40.5 x 20 x 37.5 mm; 41.5 to the top of the output spline |
| ears | 56 mm ear-to-ear, 2.7 thick, underside 24.7 above the case bottom; four holes on 49 x 10 |
| spline | 25T, 5.9 mm |
| voltage | 4.8-8.4 V |
| torque | 25 kg.cm at 6.0 V, 35 kg.cm at 8.4 V |
| speed | 0.10 s/60 at 6.0 V, 0.07 at 8.4 V |
| stall current | not published |
| mass | 70 g |
| construction | brushless motor, full metal gears, two ball bearings, plastic case with an aluminium centre band, IP66 per the INJORA manual |
| lead | JR 3-pin, 300 mm |
| price | USD 33.99 (servo), 34.99 with the 25T aluminium horn (SKU INJS235_Arm); Amazon B0B56SN46D USD 34.99 |

## The chassis's servo mount

On the standard-chassis Slash 4x4 (68086-4 exploded view page 2, mirrored at
https://www.rcscrapyard.net/manuals/traxxas/68086-4/68086-4-002.jpg) the stock Traxxas 2075
servo bolts inverted, output shaft down, to the front bulkhead 6830 (https://traxxas.com/6830x-front-bulkhead)
with four 3 x 6 mm flat-head screws (3932); the servo saver 6845 hangs below it and drives
the bellcranks through servo horn 6846 (the current set is 6845X,
https://traxxas.com/6845x). There is no separate servo mount part: the pocket is moulded into
the bulkhead and takes a standard-size case.

Traxxas 2075: 55.1 ear-to-ear x 20.1 x 38.1 mm, 45 g (servodatabase.com/servo/traxxas/2075);
Traxxas servos are 25T (2255 spec table, https://traxxas.com/2255-digital-high-torque-brushless-servo).

## Verdict

The INJS235 is a drop-in: do not print anything. Same 20 mm width, same 49 x 10 hole
pattern under the ears, 25T spline, and the Traxxas servo saver 6845/6845X fits its spline
(use the Traxxas saver, not the INJORA aluminium horn: the 4x4 bellcrank set needs the saver's
spring). Two things differ by under a millimetre and are worth checking with the servo in
hand before the screws go in:

1. Ear span 56 vs 55.1: the 6830 pocket has clearance for the 2255 (an INJORA-sized
   brushless Traxxas servo listed as a drop-in), so 0.45 mm a side is expected to fit; if the
   ears bind, file the ears, not the bulkhead.
2. Case height 37.5 vs 38.1 (0.6 shorter) and the ear underside 24.7 above the case bottom:
   inverted, the output face is 12.8 below the ears (2075: about 13.5). The servo saver sits
   0.7 mm higher on the spline; the 6845 saver's spline depth covers that. Check the saver
   does not touch the bulkhead when centred.
3. Screws: the Traxxas 3 x 6 FCS (3932) go through the 2.7 mm ears into the bulkhead as
   before. Torque by hand.

Power and wiring are in NEWCAR_LAYOUT.md section 6: the servo's red lead to the Hobbywing
UBEC at 7.4 V, signal and ground from the FSESC's PPM header. The 35 kg.cm (8.4 V) is far
more than a Slash needs; at 7.4 V it is about 31 kg.cm, and the 6845 servo saver's spring
protects the bellcranks, not the servo's gears, so leave the saver's spring at the stock
preload rather than locking it out.

The chassis keep-out model uses the servo as a 20 x 56 x 40.5 block standing in the bulkhead
with its top 25 mm above the chassis rim, right of the centreline (x 128..150, y -53..3,
ESTIMATE); the plinth's B1 clipless flange passes 7 mm above it and the VESC saddle stops 3.75
mm behind it. Measure its top and side (NEWCAR_LAYOUT.md section 8, item 4).
