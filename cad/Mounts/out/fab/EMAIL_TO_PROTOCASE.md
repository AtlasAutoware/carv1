# Email to send to Protocase

Send from **admin@atlasautoware.org** (the address they contacted). Attach the four files
in this folder: two DXFs and two PDF drawings.

---

**Subject:** Atlas Autoware (TJHSST) — sponsorship part submission, 2 flat aluminium plates

Hi [name],

Thank you for the sponsorship — it is genuinely the piece we could not solve ourselves.

Atlas Autoware is a student team at Thomas Jefferson High School for Science and Technology
building 1/10-scale autonomous racing cars for F1TENTH. Everything on the car mounts to one
long flat plate, and that plate is the one part we cannot make in-house: it is 452 mm long,
so it does not fit any printer we have, and we have been running a plywood stand-in that
sags about 5 mm under the electronics.

**What we would like machined**

| item | part | qty |
| --- | --- | --- |
| 1 | Baseplate v2 — old car | 2 |
| 2 | Baseplate v2 — new car | 2 |

Both are flat parts, no bends, no hardware:

- Material: aluminium 5052-H32, 0.080 in (2.03 mm)
- Outline: 452.0 × 177.5 mm, 3 mm corner radius
- 28 square holes, 28.5 × 28.5 mm (a 1.5 mm internal corner radius is fine — see note below)
- 4 rectangular cutouts for the chassis body posts, 2 mm internal radius fine
- 30 round holes: 22 × 6.0 mm, 8 × 3.2 mm
- Tolerance: ±0.25 mm on hole positions, ±0.5 mm on the outline
- Finish: deburr all edges; clear anodise or bare mill finish, whichever is easier for you
- The two designs are identical except that the body-post cutouts are shifted 11.4 mm for
  the second car's longer wheelbase

The 28.5 mm square holes take a moulded plastic clip that carries every mount on the car.
The clip's rim is 28 mm with 0.25 mm clearance a side, so a small internal corner radius
from a laser or punch is not a problem — please do not add cost chasing sharp corners.

The 0.080 in thickness is deliberate rather than convenient: our clip stands 3.13 mm above
the plate underside, so a 2.03 mm plate leaves 1.10 mm of rim proud, which is what the
mounts were designed around. Thicker and the clip is buried; thinner and the plate flexes.

**If four plates is over the sponsorship**, please cut one of each instead — that is enough
to get both cars running, and we would rather stay inside what you offered.

Files attached: a DXF per plate (1:1, millimetres, geometry only on a single CUT layer, no
text or annotation) and a dimensioned PDF drawing per plate with the material and finish
callouts.

Happy to send STEP files, change the material or thickness, or adjust anything that makes
this easier or cheaper to run. We are in no rush.

We would be glad to credit Protocase on the car and in our project write-up — just tell us
how you would like to be listed.

Thank you again,

Eshan Iyer
Atlas Autoware — TJHSST Senior Research
admin@atlasautoware.org
github.com/AtlasAutoware

---

## Before sending, check

- **Recipient.** Reply directly to the Protocase thread in `admin@atlasautoware.org` so it
  keeps their reference, rather than starting a new one.
- **Their terms.** I could not read that email — the mailbox connected here is
  `eshan.k.iyer@gmail.com`, not the org address. If the offer names a dollar cap, a part
  count, a deadline, or a specific submission portal instead of email, adjust accordingly.
  Protocase normally quote from a drawing package like this one, so it should fit.
- **The wheelbase number.** The 11.4 mm cutout shift for the new car came from the CAD, not
  from a measurement of the actual chassis. `LAYOUT.md` still lists the shock-tower and
  body-post spacing as "measure on the real car before cutting". Worth putting a ruler on
  both chassis before these are cut, because that is the one dimension that would make the
  plates not fit.
