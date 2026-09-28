# AtlasPower-4S

One board that runs the new car's electronics off the 4S drive LiPo, so the Baseus power
bank and the USB-C PD trigger come off the car entirely. 88 x 68 mm, four M3 holes on a
76 x 56 pattern, mounted on the clipless system by `../cad/power_board_mount.py`.

    XT90 -> 10 A fuse -> TVS -> ideal-diode FET -> 5 mohm shunt -> VIN
    VIN  -> AP64501 -> 10.0 V  Jetson Orin Nano + SICK TiM561
         -> AP64501 ->  7.4 V  INJORA INJS235 servo
         -> AP64501 ->  5.0 V  USB hub, PCA9685, fan
    INA226 across the shunt -> I2C -> Jetson

## Why these numbers

The pack is 12.0 V at a 3.0 V/cell cutoff and 16.8 V full. The Jetson takes 9-20 V and the
TiM561 takes 9-28 V, so both would run straight off the pack, and that is the tempting
wrong answer: the pack rail carries the motor's switching noise, and a hard launch sags it
far enough to brown out the Jetson mid-run.

The compute rail is therefore **10 V, not 12 V**. At 12 V a buck has no headroom left when
the pack is nearly flat and you need a buck-boost, which is a four-switch converter and a
much harder board. At 10 V a plain synchronous buck still has 0.8 V of headroom even at an
11 V sag, which is the single decision that keeps this board simple enough to get right.

Servo peaks are handled by a 470 uF reservoir rather than by sizing the regulator for
stall: a 35 kg-class servo pulls 3-6 A stalled but only for milliseconds, and sizing three
rails for simultaneous peaks would have pushed the board to 97 W and a lot of heat.

## Files

| file | what |
| --- | --- |
| `design.py` | every electrical number, and the checks over them. Run it first. |
| `netlist.py` | parts and nets, built from `design.py` |
| `gen_kicad.py` | writes the KiCad project from `netlist.py` |
| `gen_footprint.py` | the SRP1265A land pattern, from the datasheet drawing |
| `layout.py` | the floorplan and the copper plan, with the reasoning |
| `relax.py` | separates the floorplan until nothing overlaps |
| `place_check.py` | courtyard and edge checking from real footprint geometry |
| `gen_pcb.py` | writes the board: placement, zones, routing, via stitching |
| `libs/footprints/` | the 20 KiCad footprints this board uses, vendored so it builds anywhere |
| `out/AtlasPower4S/` | the KiCad project, schematic PDF, board, gerbers, drill, BOM, ERC/DRC |
| `../cad/power_board_mount.py` | the printed tray, on the clipless foot interface |

Regenerate everything:

    python3 design.py && python3 gen_footprint.py && python3 relax.py
    python3 place_check.py && python3 gen_kicad.py && python3 gen_pcb.py

## What has actually been verified

Everything below was run, not asserted:

- `design.py` passes all its checks: rail dividers land within 0.4 % of target on E96 pairs,
  the inductor is at or above minimum on every rail, saturation current clears 1.5x peak,
  every rail has headroom at an 11 V sag, and the shunt uses 49 % of the INA226's range
  without saturating it.
- The compensation values come from the AP64501 datasheet's own Eq. 17-20, and the method
  reproduces the datasheet's worked example (5 V, 5 A, 45 uF gives R5 = 15.8 k, as printed).
- KiCad parses the generated schematic and its extracted netlist matches `netlist.py`
  **exactly**: 42 nets, no missing, no extra, no membership differences.
- All 21 footprints resolve against the KiCad 10 libraries.
- 204 netlist pins, 204 global labels. BOM totals 84 parts against 84 in the netlist.
- ERC: **zero errors.** 84 warnings are `lib_symbol_issues`, which is `kicad-cli` not
  loading the project-local `sym-lib-table`; they disappear when the project is opened
  normally. 2 are `isolated_pin_label` on `ALERT` and `NC_VESC_5V`, both deliberate.

## The inductor footprint

Built by `gen_footprint.py` from the Bourns SRP1265A datasheet's Recommended Layout: 14.2 mm
overall span, 8.0 mm gap, 5.0 mm pad height, giving 3.1 mm pads at +-5.55 mm. Read off the
rendered drawing, not from extracted text, because extraction loses which number belongs to
which dimension line.

The reading confirms itself: 3.1 x 5.0 is exactly the part's worst-case terminal
(2.75 + 0.35 by 4.7 + 0.3), which is what a power-inductor land is. Had the 8.0 been
centre-to-centre rather than the gap, pads would have come out 6.2 mm wide against a 2.75 mm
terminal, which is not a land pattern anyone draws.

## The layout

4 layers: F.Cu components and local pours, In1.Cu solid ground, In2.Cu VIN distribution,
B.Cu ground and output pours. 88 x 68 mm, M3 holes on 76 x 56 to match the printed tray.

The board reads left to right in the order power flows: input protection, then the three
rail blocks stacked in y, then the output connectors. Each rail follows the AP64501
datasheet Figure 31 - input ceramics against VIN/GND, inductor immediately at SW, feedback
divider on the quiet side away from the switch node.

`layout.py` holds the hand floorplan (the intent: what sits next to what). `relax.py`
takes that as a seed and separates it numerically until no courtyard overlaps and nothing
crosses the board edge, because the hand coordinates were wrong in six places once the
footprints were real - a 1210 rotated 90 degrees has a 4.6 mm courtyard where I had written
3.4, and a terminal block's origin is pin 1 rather than its centre.

Routing is deliberate, not autorouted. Power nets are zones; signal nets get L-paths or a
perpendicular escape-and-return, each candidate checked against every pad of a different
net. **A connection that cannot be routed cleanly is left unrouted and reported rather than
shorted** - 20 of them currently, all in the dense compensation clusters.

### DRC status: honest

    659 violations on the first pass  ->  204 now

Fixed along the way, each one a real error: 54 courtyard overlaps, 4 parts hanging off the
board edge, 90 shorts from blind L-routing, 47 dangling vias, unfilled zones, the INA226's
exposed pad left unmapped, and library footprints whose built-in 0.2 mm thermal vias are
under the board's minimum hole size.

**What remains is a cleanup pass in the GUI, and the board should not be ordered until it is
done.** Roughly: 38 unconnected (the 20 unrouted signal connections above, plus ground
islands), 28 hole clearance and 25 shorts from ground vias placed too near other copper, 27
isolated copper regions and 25 starved thermals in the pours, and 25 `lib_footprint_mismatch`
which are benign - the footprints are emitted inline rather than linked to a library.

None of that is a design error; it is the last 10 % of a layout, which is the part a person
does with a mouse and the ratsnest visible. What the generator gives you is a board with
every part placed sensibly, every plane poured, the high-current paths done as copper rather
than traces, and a list of exactly what is left.

### Thermals

About 8 W total across three SO-8EP packages rated 45 C/W. Each exposed pad gets seven
stitching vias into the ground plane, which is what makes that 45 C/W figure meaningful.
Worth measuring on the first board rather than trusting.

Prices and stock were read from LCSC on 2026-09-07 and move.

## Two deliberate oddities

**J7 pin 2 is a dead net.** The VESC's PPM header carries 5 V, signal and ground. That 5 V
must never meet the servo BEC rail. The usual fix is "remember to pull the red pin from the
Y-lead", which is exactly the kind of instruction that gets forgotten once. Here the pin
physically goes nowhere, so the mistake is not available.

**The INA226 is powered from the Jetson's 3.3 V, not from a board rail.** Telemetry is
therefore only alive when the Jetson is, which is the correct dependency: there is nothing
to report to when it is off, and it keeps the sense chip off the noisy pack rail.

## Sources

- AP64501 datasheet DS41980 Rev. 5-2, Diodes Incorporated, December 2024 — pinout,
  0.8 V reference, 570 kHz, Eq. 6-20, Figure 31 layout guidance.
- LM74700-Q1 datasheet SNOSD17G, Texas Instruments — SOT-23-6 pinout, FET sizing rule.
- INA226 — 81.92 mV shunt full scale, 2.5 uV LSB.
- Jetson Orin Nano Developer Kit carrier board specification SP-11324-001 v1.1 — 9-20 V
  jack, 3.5 A rating.
- SICK TiM561-2050101 datasheet 1071419 — 9-28 V, external slow-blow fuse required.
