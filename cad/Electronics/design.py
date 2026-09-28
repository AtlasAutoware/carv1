"""AtlasPower-4S: the electrical design, as data.

One place for every number, so the schematic, the PCB, the BOM and the documentation are
all generated from the same source and cannot drift apart. Run `python3 design.py` to print
the rail calculations and the design-rule checks that do not need KiCad.

Card of the problem: the new car currently runs its compute off a USB-C power bank (Baseus
Blade HD) through a PD trigger, and its servo off a BEC on the drive LiPo. That means two
batteries to charge and a bank that cannot be monitored. This board deletes the bank: every
rail comes from the 4S drive pack.

The pack is 12.0 V (3.0 V/cell, the sensible cutoff) to 16.8 V (4.2 V/cell full), and sags
under motor draw. Both the Jetson (9-20 V) and the TiM561 (9-28 V) accept that range
directly, so the temptation is to wire them straight to the pack. Do not: the pack rail
carries the motor's switching noise, and a hard launch drops it far enough to brown-out the
Jetson mid-run. Every rail here is regulated, and the compute rail is set to 10 V rather
than 12 V so that a buck (not a buck-boost) still has headroom at the bottom of the pack.
"""

VREF = 0.8            # AP64501 feedback reference, datasheet DS41980 "0.8 V +-1%"
FSW = 570e3           # AP64501 fixed switching frequency
VIN_MIN, VIN_NOM, VIN_MAX = 12.0, 14.8, 16.8
VIN_SAG = 11.0        # worst measured-style dip under a hard launch; design against this

# (name, Vout, I_continuous, I_peak, what it feeds)
RAILS = [
    ('VOUT_10V', 10.0, 3.3, 4.0, 'Jetson Orin Nano devkit (9-20 V jack) + SICK TiM561 (9-28 V)'),
    ('VOUT_7V4', 7.4, 3.0, 5.0, 'INJORA INJS235 servo, 4.8-8.4 V; peaks ride on bulk capacitance'),
    ('VOUT_5V', 5.0, 3.0, 4.0, 'USB hub, PCA9685, cooling fan'),
]
RIPPLE_FRACTION = 0.30        # of I_continuous, the usual 20-40 % target
# The datasheet's own worked examples and its Table 1 all use a 22.1 k bottom resistor, and
# its compensation equations assume that divider current. Matching it means the loop
# component values below come out of the datasheet's method rather than being invented.
R_FB_BOTTOM = 22.1            # kohm, fixed; the top resistor sets the rail
COUT_PER_RAIL = 66e-6         # 3 x 22 uF ceramic, derated ~ the datasheet's 45 uF effective
COUT_EFFECTIVE = 45e-6
FC = 15e3                     # target crossover, under the datasheet's fsw/10 rule
GM = 0.15e-3                  # error amplifier transconductance, datasheet Eq. 17
ESR_COUT = 1e-3               # ceramic, datasheet's worked example


def compensation(vout, iout, cout=COUT_EFFECTIVE, fc=FC):
    """Type II network on COMP, by the datasheet's Eq. 17-20. Returns (R5 k, C5 nF, C6 pF)."""
    r5 = 4.67e3 * fc * vout * cout                       # Eq. 17, ohms
    r5e = e96(r5 / 1000)                                 # kohm
    c5 = vout * cout / (iout * r5e * 1000)               # Eq. 18, farads
    c6 = max(ESR_COUT * cout / (r5e * 1000), 1 / (3.14159 * FSW * r5e * 1000))   # Eq. 19
    return r5e, c5 * 1e9, c6 * 1e12

E96 = [10.0, 10.2, 10.5, 10.7, 11.0, 11.3, 11.5, 11.8, 12.1, 12.4, 12.7, 13.0, 13.3, 13.7,
       14.0, 14.3, 14.7, 15.0, 15.4, 15.8, 16.2, 16.5, 16.9, 17.4, 17.8, 18.2, 18.7, 19.1,
       19.6, 20.0, 20.5, 21.0, 21.5, 22.1, 22.6, 23.2, 23.7, 24.3, 24.9, 25.5, 26.1, 26.7,
       27.4, 28.0, 28.7, 29.4, 30.1, 30.9, 31.6, 32.4, 33.2, 34.0, 34.8, 35.7, 36.5, 37.4,
       38.3, 39.2, 40.2, 41.2, 42.2, 43.2, 44.2, 45.3, 46.4, 47.5, 48.7, 49.9, 51.1, 52.3,
       53.6, 54.9, 56.2, 57.6, 59.0, 60.4, 61.9, 63.4, 64.9, 66.5, 68.1, 69.8, 71.5, 73.2,
       75.0, 76.8, 78.7, 80.6, 82.5, 84.5, 86.6, 88.7, 90.9, 93.1, 95.3, 97.6]


def e96(value_k):
    """Nearest E96 value in the same decade, in kohm."""
    import math
    dec = 10 ** math.floor(math.log10(value_k))
    return min(E96, key=lambda v: abs(v * dec / 10 - value_k)) * dec / 10


def fb_top(vout):
    return e96(R_FB_BOTTOM * (vout / VREF - 1))


def actual_vout(rtop):
    return VREF * (1 + rtop / R_FB_BOTTOM)


def inductor(vout, iout, vin=VIN_MAX):
    """Minimum L for RIPPLE_FRACTION ripple at the worst duty (highest Vin)."""
    d_il = RIPPLE_FRACTION * iout
    return vout * (vin - vout) / (vin * FSW * d_il) * 1e6      # uH


# Chosen inductors. The first pass used three different values, one per rail, sized at the
# LM61460's 400 kHz; moving to the AP64501's 570 kHz made two of them undersized. Rather
# than three part numbers, all three rails now take the same 10 uH: it is at or above the
# minimum everywhere, keeps ripple between 21 and 24 %, and there is one inductor to buy,
# stock and place. Isat 15.5 A against a 5 A worst-case peak.
L_CHOSEN = {'VOUT_10V': (10.0, 'SRP1265A-100M', 15.5), 'VOUT_7V4': (10.0, 'SRP1265A-100M', 15.5),
            'VOUT_5V': (10.0, 'SRP1265A-100M', 15.5)}     # (uH, MPN, Isat A)

R_SHUNT = 0.005       # ohm, INA226 input shunt. 5 mohm, not 2: see the check below.
I_MAX_INPUT = 8.0     # A, board input budget
EFF = 0.90            # assumed rail efficiency for the input-current sums


def checks():
    out = []
    ok = True
    out.append(f'{"rail":10s} {"Vset":>6s} {"Ract":>6s} {"Rtop":>7s} {"Lmin":>6s} {"Lused":>6s} '
               f'{"Isat":>6s} {"headroom@sag":>13s} {"P":>6s}')
    for name, vout, ic, ip, _ in RAILS:
        rt = fb_top(vout)
        va = actual_vout(rt)
        lmin = inductor(vout, ic)
        lu, mpn, isat = L_CHOSEN[name]
        # AP64501: 45 mohm high-side FET; dropout at the top of the duty cycle
        need = vout + ip * 0.045
        head = VIN_SAG - need
        out.append(f'{name:10s} {vout:6.2f} {va:6.2f} {rt:6.1f}k {lmin:5.1f}u {lu:5.1f}u '
                   f'{isat:5.1f}A {head:+12.2f}V {vout*ic:5.1f}W')
        if abs(va - vout) / vout > 0.02:
            out.append(f'   FAIL {name}: E96 pair gives {va:.2f} V, more than 2 % from {vout}'); ok = False
        if lu < lmin * 0.95:
            out.append(f'   FAIL {name}: {lu} uH is under the {lmin:.1f} uH minimum'); ok = False
        if isat < ip * 1.5:
            out.append(f'   FAIL {name}: Isat {isat} A under 1.5x the {ip} A peak'); ok = False
        if head < 0:
            out.append(f'   FAIL {name}: no headroom at a {VIN_SAG} V sag'); ok = False

    out.append('')
    out.append(f'{"rail":10s} {"R5":>7s} {"C5":>7s} {"C6":>7s}   loop, datasheet Eq. 17-20 at fc = 15 kHz')
    for name, vout, ic, _, _ in RAILS:
        r5, c5, c6 = compensation(vout, ic)
        out.append(f'{name:10s} {r5:6.1f}k {c5:6.2f}n {c6:6.0f}p')
        if r5 <= 0 or c5 <= 0:
            out.append(f'   FAIL {name}: compensation solved to a non-physical value'); ok = False

    p_cont = sum(v * i for _, v, i, _, _ in RAILS)
    p_peak = sum(v * i for _, v, _, i, _ in RAILS)
    out.append('')
    out.append(f'continuous out {p_cont:.0f} W -> {p_cont/EFF:.0f} W in -> '
               f'{p_cont/EFF/VIN_NOM:.1f} A at {VIN_NOM} V, {p_cont/EFF/VIN_MIN:.1f} A at {VIN_MIN} V')
    out.append(f'all-peaks-at-once {p_peak:.0f} W -> {p_peak/EFF/VIN_MIN:.1f} A in (transient only)')
    if p_cont / EFF / VIN_MIN > I_MAX_INPUT:
        out.append(f'   FAIL continuous input current over the {I_MAX_INPUT} A budget'); ok = False

    # realistic mission load, which is what the run time is actually set by
    mission = 25 * 1.1 + 4 + 7.4 * 0.5 + 5
    wh = 14.8 * 9.0
    out.append(f'mission load {mission:.0f} W out, {mission/EFF:.0f} W in; a 9000 mAh 4S is '
               f'{wh:.0f} Wh, so {wh*0.8/(mission/EFF):.1f} h at 80 % usable')

    # INA226: +-81.92 mV full scale, 2.5 uV LSB
    v_fs = R_SHUNT * I_MAX_INPUT * 1000
    out.append(f'INA226 shunt {R_SHUNT*1000:.0f} mohm: {v_fs:.1f} mV at {I_MAX_INPUT} A of 81.92 mV '
               f'full scale ({v_fs/81.92*100:.0f} % of range), LSB {2.5e-6/R_SHUNT*1000:.1f} mA, '
               f'dissipating {I_MAX_INPUT**2*R_SHUNT:.2f} W')
    if v_fs > 81.92:
        out.append('   FAIL shunt saturates the INA226'); ok = False
    if v_fs < 20:
        out.append('   WARN under a quarter of the ADC range in use; a larger shunt would resolve better')
    return ok, '\n'.join(out)


if __name__ == '__main__':
    ok, text = checks()
    print(__doc__)
    print(text)
    print('\nALL CHECKS PASS' if ok else '\nCHECKS FAILED')
    raise SystemExit(0 if ok else 1)
