# DOCS1 — COMPLETED (2026-08-14)

> The full documentation pass. Written FROM `DOCS1-PROGRESS.md` (the live log), not instead of it.
> Predecessor: R1b. Successor: **the release to `main`**, then R3.
> **Ships no library behaviour.** Two code files were touched and both are documentation:
> a `//` header comment that stated a model the code does not implement, and a `//` comment in a
> test that the guide quotes verbatim.

## The one-sentence honest status

Every public document and every internal document was read end to end and 60-odd stale claims
were corrected — including a **red release gate**, a homepage saying the hardware layer was
"unwritten", a limitations chapter contradicting itself twelve lines apart, and a shipped header
teaching a battery model the code rejects — **and the pass ran without the adversarial
verification it was designed around**, because a session limit killed all 45 verifier agents;
every applied finding was instead verified by the reviewer against source.

## The release gate was red before a document was read

`prepare_site.py` exited 1: `docs/roadmap.md` cited `E3-COMPLETED.md`. Fixed.

**The gate that should have caught it was then fixed three times, each fix finding the next hole:**

1. `check-removability` screened `internal/|chunks/|RESUMING|build-order` and was blind to
   `-COMPLETED.md`. Added — plus a **self-test that reads `prepare_site.py`'s screen list out of
   its source** and proves this gate covers every term in it, so the two cannot drift apart again.
2. **That self-test failed on its first run**: `prepare_site.py` screens the bare string
   `docs/internal`, which `internal/` does not match.
3. **Reading then found what neither gate had:** five public documents cited completion records
   *without the extension* (`C2-COMPLETED §Mutations`, `A3-COMPLETED §3.7`, and three more).
   Requiring `\.md` missed every one. The extension was dropped from the pattern.

Negative control: planted a violation → gate named it file:line; removed → clean.

**The lesson worth keeping: the hole was not in either list. It was in the gap between two lists
that were maintained by hand and believed to agree.**

## What was actually wrong, by weight

**The site homepage** said *"the adapters that would reach real hardware are **unwritten**."*
Fourteen exist and have commanded real motors.

**The master plan's front matter** — published to docs.shurobotics.com — read *"Status:
DESIGN-FIRST (no code yet) … sections marked `_PENDING_` … Last updated 2026-06."* All three
false; `grep -c _PENDING_` returns 1, which is that line itself.

**Guide ch. 14 contradicted itself twelve lines apart** (":43 settled seven beliefs" vs ":55 every
one still unsettled") — in the chapter this pass was warned to guard hardest — and listed the SD
blackbox and the published API site as "not built" when both exist (the site returns 200).

**The flagship tutorial** taught in four separate sentences that the hardware layer did not exist.
One lived inside a CI-quoted code block, so the fix had to land in `test/guide_examples_test.cpp`
and the guide in lockstep.

**A stale test count** (`659 / 915,570`, four chunks old) sat in a ```` ```text ```` block —
which `check-examples` does not scan, because it scans ```` ```cpp ````. It rotted in silence.

**The register violated its own Rule 2.** Rule 2 says a settled entry is *"marked `[x]` with the
measured value recorded next to the guess it replaces."* Seven index rows said **settled**;
**zero** detail entries were `[x]`, and all seven still read *"Confidence: reasoned — unverified
against firmware"* with future-tense Settle lines. The index and the detail contradicted each
other for eight entries. All eight reconciled, each now carrying its measured value and the
one-robot-once provenance; HA-98 stays open because its persistence half genuinely is.

**A wrong mental model in shipped source.** Guide ch. 6 said the library *"compensates command
voltages for the measured sag."* `compensateForBattery()` is `std::clamp` — a ceiling. It
inherited the claim from `hal/battery.hpp`, which said the control layer *"scales motor commands
by the measured battery voltage"*, flatly contradicting `feedforward.hpp` in the same tree. That
is the percent-output model this design deliberately rejects. Both fixed.

**Arithmetic that contradicted its own listing:** cookbook 04 said *"multiplying by the distance
gives travel time"* nine lines under a helper that divides. Speed × distance is in²/s.

**A fault code missing from the catalogue:** `fault.hpp` defines eleven raisable codes; guide
ch. 11's table — billed *"the complete vocabulary"* — listed ten.

**The changelog skipped five API-adding chunks** (E1–E4, F2). The 2.0/2.1 boundary was established
from commit timestamps, not assumed: F1 at 08-13 03:23 is the bump, so E1/E2/E3/E4 belong in 2.0
and F2 in 2.1.

**`RESUMING.md` — the canonical protocol — carried an ARM command that does not work.** Its
`sed 's|.*/include/||'` needs a `/` before `include`; run from the repo root as every instruction
says, it emitted `#include "include/shulib/…"` and died with `fatal error: No such file or
directory`. The briefing had the working form all along; the two had drifted and the canonical one
was wrong.

**The toolchain diagnosis in the roadmap was wrong and is now recorded as wrong.** Verified
`firmware/liblvgl.a` is gone, `Makefile` pins `CXX_STANDARD:=gnu++20`, and a forced relink
succeeded (exit 0, fresh `bin/hot.package.bin`). The distro toolchain was never the problem.

## Numbers

| Measure | Result |
|---|---|
| Documents read end to end | **34 public + 11 internal** — the whole surface |
| Files changed | 33 |
| Suite | 1120 cases / 1,523,324 assertions / 3 skipped — **green** |
| ARM gate | 148 headers, unamended, clean |
| Both CI guards | PASS |
| Doc gates | coverage · fresh · examples · removability · self-test · briefing · staleness self-test · staleness audit — **all PASS** |
| `prepare_site.py` (the release gate) | **PASS** — was RED at chunk start |
| `make` | exit 0, package produced (forced relink) |

## Decisions where a viable alternative existed

1. **`ORIENTATION.md` gets no public sibling.** It is mechanically publishable, but
   `docs/README.md` *is* the homepage and was already doing that job badly. Rejected: a sixth
   hand-maintained public document, in the chunk that exists because documents go stale. The
   orientation content was folded into the homepage instead.
2. **The autonomous-period figure was removed, not replaced.** The cookbook asserted "fifteen
   seconds" four times against guide ch. 1 and the master plan (Skills = 1:00). Rejected:
   substituting a rulebook number not derivable from this repo — guessing a fact into the docs is
   what this chunk exists to prevent. It is also independently right: F2's ruling is that the
   library holds no default match length. **Still owed an answer.**
3. **Stale counts were deleted, not corrected**, wherever the number is maintained by hand
   (register sizes, chunk counts, workstream counts, test-case counts, "49 hardware claims").
   A corrected count is stale again at the next chunk; a pointer to the source is not.
4. **The API reference's scope paragraph was rewritten twice**, and the first version was wrong —
   see below.
5. **Historical records were superseded, never rewritten** (L2). The `HANDOFF-*` files and
   completion records were left alone; the *live* documents that pointed at them were fixed.

## The mistake I made, recorded because the standard demands it

The first fix to `docs/api/README.md` explained its narrow scope by saying the page *"covers the
surfaces that are frozen, because those are the ones whose exact spelling is a promise."*

**That is false, and the team lead caught it within the hour** by asking why fourteen subsystems
produce two documented types. **Seven contracts are LOCKED** — frame, accuracy, units, HAL
interfaces, kinematics, `Chassis`, `Routine` — and only two are on that page. I wrote a tidy
rationale that the repo contradicts, which is the exact failure this chunk exists to catch,
committed by the person auditing for it. Rewritten to the honest version: the selection is about
*audience* (routine authors), the gap is named as a gap, and the reference now says so on its
face.

## Not finished, named honestly

- **`[~]` The adversarial verification stage did not run.** 45 of 66 agents died on a session
  limit, including **every verifier**. The design was: find, then have an independent agent argue
  both sides with a calibrated probability. What happened: I verified each applied finding myself
  against source. That is weaker, and it is the honest state.
- **`[~]` The API reference still documents 2 of ~160 public types.** Measured: 971 public items
  across 71 headers, **399 undocumented**, and the generator **cannot parse four headers at all**
  (nested public types — it refuses rather than dropping members). Called by the team lead as its
  own chunk; brief written at `DOCS2-full-api-reference.md`.
- **The autonomous-period length is unanswered** and no document now states one.
- **The lift-homing question remains open** — recorded in the master plan's new open-decisions
  table, along with the sensor loadout, characterization timing, and the unverified ADI expander.

## Handoffs

**To the release:** every gate green including `prepare_site.py`; the merge mechanic is verified
by dry run in a throwaway clone — `git merge --squash` is **wrong** (49 add/add conflicts, `main`
is deliberately disjoint); the release is a tree snapshot,
`git commit-tree $(git rev-parse release/v2^{tree}) -p main`, matching every prior release's
single-parent shape. **Two questions ride with it: whether to push at all, and HTTPS** (verified
2026-08-14: the site answers over HTTP and fails TLS on a name mismatch).

**To DOCS2:** the measured debt table, the four unparseable headers, and the `TARGETS`-vs-coverage
tension — four Freeze Register rows say in writing they are deliberately ungated, and expanding
overrides that. Allowed; doing it silently is not.

**To R3:** the register's seven settled entries now carry their measured values in the detail
bodies, so the bench has a clean baseline to work from, and HA-98's open half is stated where it
will be seen.
