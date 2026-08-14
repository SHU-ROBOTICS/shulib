# DOCS1 — live progress log

> Appended in real time. This is the honest record of exactly how far the work got, so an
> interrupted chunk is recoverable. Started 2026-08-14.

---

## Step 0 — ground truth measured BEFORE reading anything

Nothing in this chunk is judged against another document. Everything below was produced by
running a command today, on `shulib-v2` @ `4d4bfc1`, working tree clean.

| Measure | Result |
|---|---|
| Suite | **1120 cases / 1,523,324 assertions / 3 skipped — green** |
| Public headers | **148** |
| `hal/pros/` files | **15** = 14 adapters + 1 tick pacer |
| HA register | **122 registered, 7 settled** (HA-94/95/96/97/99/100/101), next free **HA-123** |
| Chunks | **23 of 43 complete**; 20 remain |
| `api_doc_tool.py check-fresh` | PASS |
| `api_doc_tool.py check-coverage` | PASS |
| `api_doc_tool.py check-examples` | PASS — 386 quoted lines, 4 source files |
| `api_doc_tool.py check-removability` | PASS |
| `briefing_status.py check` | PASS |
| `doc_staleness_audit.py self-test` | PASS (10 detector/filter cases) |
| `doc_staleness_audit.py` | clean (26 live docs, 0 numeric claims checked) |
| PROS-free guard (path-anchored CI form) | PASS |
| sim-layering guard | PASS |
| ARM cross-compile, all 148 headers | PASS |
| **`prepare_site.py`** | **FAIL** — see Finding 0; fixed in step 2 |

### FINDING 0 (RESOLVED in step 2) — the release gate was red, and the build gate could not see it

`python3 tools/prepare_site.py <out>` exits 1:

```
ERROR: internal development docs would be PUBLISHED:
  roadmap.md: -COMPLETED.md
```

`docs/roadmap.md:363` contains the literal string `E3-COMPLETED.md`.

**The interesting half is why nothing caught it.** `check-removability` — which *does* run at
build time — matches `internal/|chunks/|RESUMING|build-order` (`tools/api_doc_tool.py:740`) and is
blind to `-COMPLETED.md` / `-PROGRESS.md`. Only `prepare_site.py` (`:134`) screens for those, and
it runs at *publish* time. So the build has been green while the publish would have failed, which
is precisely the shape D3 named: **a gate's exclusion list is where its holes live.**

This is the first thing DOCS1 was supposed to find and it was found before a document was read.

### Git / release state, measured

- `origin/shulib-v2` @ `e97547b` — **37 commits behind local**
- `main` == `origin/main` == `d4fac9c`; `release/v2` == `origin/release/v2` == `c94ece0`
- `main` verified by content (`git cat-file -e main:<path>`), never by commit distance.
  **ABSENT from `main`:** E1 `diag/sd_sink.hpp` · E2 `localization/gps_corrector.hpp` ·
  E3 `localization/apriltag_corrector.hpp` · E4 `localization/ekf_fusion_policy.hpp` ·
  F1 `hal/mechanism.hpp` · F2 `sequence/run_guard.hpp` · R1a/R1b `hal/pros/*` ·
  `docs/changelog.md` · `docs/faq.md`.

---

## Step 1 — the reading pass

The deliverable of this chunk is *reading time*, so the reading was split one agent per document,
each armed with the measured ground truth above and each followed by an **independent verifier**
that re-derives every finding from source and argues **both sides** with a calibrated probability
— never a verdict. That shape is deliberate: this project has already been burned once by a
red-team told to "default to refuted", which reported 23 findings and 0 survivors when two were
real.

### The reviewer's own reading, done in parallel and not delegated

Four findings below were found by *me* reading, before any agent reported — recorded separately
so the split between "found by the fan-out" and "found by reading" stays honest.

**R-1 · `README.md:65` — the front door carries the claim R1b already had to fix elsewhere.**
> "and **no adapter has ever touched a physical device.**"

False since 2026-08-13. The bench session commanded eight physical motors at +2.0 V through the
real `ProsMotor` adapter and read physical sensors. Guide ch. 14 was corrected for exactly this
sentence at R1b; **the README was not**, and the README is the document a judge reads first.
Same paragraph, `README.md:69`: "the first bench session is still ahead" — it ran.

**R-2 · `README.md:253` — "all 49 hardware claims".**
The register holds **122**. This is the trap-3 shape the project has already suffered twice
(*"a chapter claiming 57 assumptions when there were 67"*), at nearly three times the drift.

**R-3 · `docs/guide/14-what-it-cannot-do-yet.md:55` — the chapter contradicts itself, twelve lines apart.**
Line 43 says the bench session "settled … seven unit-scale beliefs on the drivetrain set."
Line 55 says the register entries are "every one still unsettled." Both cannot be true; the
second is the stale one (7 of 122 are settled). This is the *exact* shape of the README incident
R1a recorded — two contradictory figures inside one document — reappearing in the one chapter
this pass was warned to guard hardest.

**R-4 · ch. 14 lists two things as not built that E1, R1b and the release already built.**
- `:324` — "Not built yet … **the SD-card blackbox** (logs without a laptop)". E1 shipped
  `diag/sd_sink.hpp` (24 KB), `diag/blackbox_format.hpp` (34 KB) and `diag/blackbox_reader.hpp`;
  R1b then gave it a real device backing in `hal/pros/block_sink.hpp`. It is built.
- `:295` — "**No published API reference site.** … nothing is hosted anywhere yet".
  Measured: `curl -L http://docs.shurobotics.com/api/chassis/` returns **200** and serves the
  rendered reference. The site is live.

*(Both of these under-claim rather than over-claim, which is the safe direction and the reason
they survived — but a limitations chapter that lists finished work as missing is still lying to
the reader it exists to serve, and it is the page an evaluating team reads to decide whether to
trust the project.)*

**Verified as still TRUE, not changed:** `README.md:109`'s heading-accuracy figure. Re-ran the
acceptance case live: worst end-of-60 s error **0.911742°**, worst instantaneous 1.06535°, across
10 seeded boots, cap 1.0° — the README, `roadmap.md:592/946` and `hardware-assumptions.md:866`
all agree with the measurement. **HTTPS is still not issued** (`curl https://…` fails on the
certificate), so the briefing's "Pending" note on that is accurate.

### ⚠️ The fan-out was cut off by a session limit — say so plainly

Of 66 agents across the two passes, **21 completed and 45 died** on `You've hit your session
limit`. **Every verifier died**, so the second-opinion stage this pass was designed around did not
run at all. Consequence, stated rather than papered over: **I verified every applied finding
myself, against source**, and the "argue both sides" stage exists only in my own checking. That is
weaker than the design and it is the honest state.

Documents a reading agent covered: `README.md`, `docs/README.md`, `changelog.md`, `roadmap.md`,
`shulib-v2-master-plan.md`, `diagnostics-plan.md`, `guide/README`, `guide/01`–`05`, and internally
`PROJECT-BRIEFING`, `ORIENTATION`, `docs-publishing`, `guide-maintenance`, `verify/README`,
`evidence/first-boot`, and the three `HANDOFF-*` files. **187 raw findings.**

---

## Step 2 — what was fixed, and the evidence for each

### The release gate (fixed, then the gate itself was fixed twice more)

**FINDING 0 fixed:** `docs/roadmap.md` no longer cites `E3-COMPLETED.md`. `prepare_site.py` now
exits 0: *34 files, 120 source links rewritten, internal docs absent (verified)*.

**Then the gate that missed it was closed — and closing it found two more holes:**

1. `REMOVABILITY_TERMS` gained `-COMPLETED` / `-PROGRESS`, and a **self-test** was added that reads
   `prepare_site.py`'s screen list out of its source and proves this gate covers every term in it.
   Two gates hand-maintaining "the same" list is D3's lesson in its other form: **the hole is not
   in one list, it is in the gap between two.**
2. **The self-test failed on its first run** — `prepare_site.py` screens the bare string
   `docs/internal` (no trailing slash) and `internal/` does not match it. Added.
3. **Reading then found a third hole neither gate had:** five public documents cited completion
   records *without the extension* — `C2-COMPLETED §Mutations`, `A3-COMPLETED §3.7`,
   `C5-COMPLETED §mutations`, `C6-COMPLETED` ×2. Requiring `\.md` missed every one. The extension
   is now dropped from the pattern; all five citations were rewritten to the neutral
   "development log" form the roadmap already used elsewhere.

**Negative control run:** planted `E9-COMPLETED.md` in a public doc → gate named it, file:line;
removed → gate clean. A gate nobody has watched bite is a gate nobody has tested.

### The front door and the homepage

**`README.md`** — 11 fixes. The cluster that mattered: *"no adapter has ever touched a physical
device"* (false since 2026-08-13), *"the first bench session is still ahead"* (it ran),
*"everything verified so far is verified off-robot"*, and *"what has been confirmed on hardware —
**and only this**"* which enumerated the boot and omitted the bench session. All four rewritten
toward **more** precision: the platform layer is measured, **no control loop has ever closed**.
Also: *"all 49 hardware claims"* → the register is **122** (the trap-3 shape at ~3× drift);
*"the 10 hardware interfaces"* → there are **19** under `hal/`; three real directories
(`manipulation/`, `sequence/`, `core/`) were missing from the tree listing; the mechanism layer and
the run guard had **no presence on the page at all**; the EKF was listed with three capabilities and
no cost (it **lost**, 0.351″ vs 0.225″, 7 of 8 seeds); a hard-coded `1.5-million-assertion` figure
survived twenty lines below the page's own argument against hard-coded figures; and the opening
described VEX U as *"one-minute matches"*, which the repo's own guide ch. 01 contradicts.

**`docs/README.md`** — the **site homepage**, and the worst single sentence found:
*"the adapters that would reach real hardware are **unwritten**."* Fourteen exist and have touched
hardware. Also *"the physical constants are still estimates"* (7 are measured), the FAQ and
changelog were **unlinked from the homepage** despite being in the site nav, and
*"**Every** public member is documented, or the build fails"* — **verified false**: the coverage
gate parses exactly two headers (`TARGETS` in `api_doc_tool.py`), which the Freeze Register itself
says is deliberate. Rewritten, and given a real one-minute orientation section (see the ruling below).

### Guide ch. 14 — the chapter this pass was warned to guard hardest

Four fixes, all found by reading it end to end:

- **It contradicted itself twelve lines apart** — `:43` "settled … seven unit-scale beliefs",
  `:55` "every one still unsettled". The second was stale. This is the *exact* README incident R1a
  recorded (two contradictory figures in one document) reappearing here.
- **It listed the SD-card blackbox as "not built yet."** E1 shipped `diag/sd_sink.hpp` (24 KB),
  `blackbox_format.hpp` (34 KB) and `blackbox_reader.hpp`; R1b gave it a device backing. Verified
  the *rest* of that list is still genuinely unbuilt: SHUL/2 is comments only, replay is D-9 (H2),
  and the on-brain HUD (D-12) is distinct from the controller screen (D-4), which **is** built.
- **It said "no published API reference site."** Measured: `curl -L
  http://docs.shurobotics.com/api/chassis/` → **200**, serving the rendered reference. Replaced
  with the honest limit — the site publishes from the release branch, so it **lags by design**.
- **"No mechanism/scoring layer"** contradicted the chapter's own section three headings up.
  Verified: the seam exists (`IMechanism`, `MotorMechanism`, `PneumaticMechanism`, `IMechanismOp`,
  `RunUntilConfirmed`, `ActuateAndConfirm`, `StallDetector`); the season verbs do not.

### The changelog had a five-chunk hole

`E1, E2, E3, E4` and `F2` — every one adding public API — were **absent**. Established the exact
boundary by commit timestamp rather than assuming: F1 (08-13 03:23) is the 2.0→2.1 bump, so E1
(08-12 22:01), E2 (23:05), E3 (08-13 00:36) and E4 (01:53) belong in **2.0**, and F2 (06:57) in
**2.1**. Five entries written, each marked *(reconstructed)* per the file's own convention, each
carrying the cost as well as the capability — the EKF entry leads with the fact that it lost.

### The FAQ's missing trap

Three of the four required entries were already there (9999, the two port conventions, `/usd/`).
**The dropped-USB failure was not.** Added: uploads report success, the program keeps running, the
serial goes silent, and the resolving fact is *what the brain screen says*. Written because the
false theory it produced — *"linking shulib breaks the binary"* — was a serious wrong finding that
only a control test disproved.

### The API reference

`check-fresh` passes, so it cannot drift from `chassis.hpp` / `routine.hpp`. But it opened *"Every
public member of shulib's autonomous-routine API"* with **no statement of scope**, while
`RunGuard`, `SdSink`, both correctors, the EKF and the mechanism/controller/digital-input seams are
all public, all shipped and all **absent**. That absence is a real decision (an unfrozen seam
pinned by a doc gate cannot move) but it was an **undocumented** one, which is the shape that reads
as a lie. The generator now says what is deliberately not there and why; regenerated, `check-fresh`
green.

### The cookbook

Better than expected — the README had absorbed F1 and page 03 had fully absorbed F2's run guard.
Two real finds: page 05 said `then()` accepts *"`void`, `bool`, or `ExitReason`"* when F1 added a
**fourth**, `MechanismOutcome`, where only `Succeeded` continues (verified at
`routine.hpp:107` and the `static_assert` at `:390`) — the distinction that stops an unconfirmed
grab reading as success. And the cookbook asserted **"autonomous is fifteen seconds"** four times,
contradicting guide ch. 01 and the master plan (Skills = 1:00). Rather than substitute a rulebook
number I could not verify from the repo, the hard-coded constant was **removed** — which is also
F2's own ruling: the library holds no default match length. **This is flagged for the team lead as
an open factual question** (§ below).

### The roadmap

The opening three bullets stated goals in the present tense — *"Anyone can use it… no C++
required"* (Tier 0/1 do not exist), *"holding < 1° of heading error"* (a frozen **target**; the
measurement is 0.912° in simulation with invented drift). Each now carries its own **Status:** line.
Seven entries still said chunks were *"in the working tree pending review/commit"* — all committed.
`Next:` pointed at R3, skipping DOCS1. Stale counts: "39 chunks", "0 of 112", "49 falsifiable
entries" ×2, "12 workstreams" (there are 14), guide-example case counts. Four checkboxes were
unchecked for work that had shipped (`IController` at R1a, the `/usd/` adapter at R1b, the
E2/E3/E4 corrector half). And the **toolchain diagnosis was wrong and is now recorded as wrong**:
verified `firmware/liblvgl.a` is gone, `Makefile` pins `CXX_STANDARD:=gnu++20`, and a forced
relink succeeded — *"Linking hot project … [OK]"*, exit 0, fresh `bin/hot.package.bin`. The distro
toolchain was never the problem.

### Internal documents

`PROJECT-BRIEFING` §13 was still titled **"THE IMMEDIATE TASK — R1b"** — the section a pasted-in
session acts on first, one chunk out of date. Rewritten to DOCS1, with the R1 record kept below it
because its lessons are live. §3's *"no adapter has ever touched a physical device"* and
*"112 registered, none settled"* fixed; §8's *"Next free: HA-94"*, §12's chunk count and §14's
"R1b's still pending" now defer to the generated block rather than restating numbers that have
gone stale here twice.

**`RESUMING.md` — the canonical protocol — carried an ARM verification command that does not
work.** Its `sed 's|.*/include/||'` needs a `/` before `include`, so run from the repo root as
every instruction says it matched nothing and emitted `#include "include/shulib/…"`. **Measured:**
`fatal error: include/shulib/chassis/chassis.hpp: No such file or directory`. A session following
the canonical protocol would hit a fatal error on the project's own verification step. Fixed to the
anchored form `PROJECT-BRIEFING` has carried all along — the two had drifted and the working one
was not the canonical one.

---

## Gates, after the edits

| Gate | Result |
|---|---|
| Suite | **1120 / 1,523,324 / 3 skipped — green** |
| coverage · fresh · examples · removability | PASS |
| `api_doc_tool.py self-test` | OK *(now including the two-screen coverage pin)* |
| `briefing_status.py check` | PASS *(regenerated; correctly flags DOCS1 as in flight)* |
| staleness self-test · audit | OK (10 cases) · clean |
| `prepare_site.py` | **PASS** — was the release blocker |
| PROS-free · sim-layering · ARM (148 headers) | PASS |
| `make` (forced relink) | exit 0, package produced |

---

## Step 3 — the second reading pass, and what it found

The twelve documents whose readers died on the limit were re-run. **12 of 12 completed, zero
errors.** These were the chapters nobody had checked against R1a/R1b at all, and they were the
worst-affected part of the tree.

### The flagship tutorial still said the hardware layer did not exist

`guide/08` — the chapter a new member is sent to first — carried *"Those adapters don't exist yet
(phase R1)"*, *"that's where all shulib code runs today"*, *"on the real robot `pace()` **will**
be…"*, and a `DR`-flag claim that *"in today's library is always true"*. Four separate sentences
teaching that the library has no hardware layer, in the chapter most likely to be read cold.

One of them lives **inside a CI-quoted code block**, so it could not be fixed in the document at
all — the comment is in `test/guide_examples_test.cpp:162` and the guide quotes it verbatim. Both
were edited in lockstep; `check-examples` re-passes at 387 quoted lines.

### A stale test count in the one place no gate can reach

`guide/07:70` printed **`659 cases / 915,570 assertions`** — the C6-era baseline, four chunks
stale. It sits in a ```` ```text ```` block, which `check-examples` does not scan (it scans
```` ```cpp ````), so it rotted silently. Replaced with the ellipsis form the README already uses,
plus the reason. The same chapter also pointed readers at the README "which tracks the current
numbers" — the README deliberately stopped doing that at R1a.

*(And when the replacement note **quoted** the stale pair to explain itself, the staleness audit
went red on it. The detector cannot tell a historical citation from a live claim, and it is right
not to try — the note was reworded to carry no figure at all. A gate firing on the reviewer is the
gate working.)*

### A wrong mental model in a shipped header, inherited by the guide

`guide/06:82` said the library *"compensates command voltages for the measured sag"*. It does not.
`control::compensateForBattery()` is `std::clamp(d, -b, b)` — a **ceiling**, not a correction —
and `feedforward.hpp:13-17` says so explicitly: *"We command actual voltage … so the only battery
effect is this ceiling; the kV/kS/kA themselves are battery-independent."*

The guide inherited it from **`hal/battery.hpp:4-7`**, which claimed *"the control layer **scales**
motor commands by the measured battery voltage"* — flatly contradicting `feedforward.hpp` in the
same tree. That is the percent-output model this design deliberately rejects. Both fixed; the
header now records that it was the ancestor of the wrong claim.

### Arithmetic that contradicted its own code

`cookbook/04:90` — *"Multiplying by the distance gives travel time"*, nine lines under a helper
that **divides** (`safety * |distance| / reach`). Speed × distance is in²/s. A reader following
the prose instead of the listing computes a timeout wrong by the square of the speed.

### A fault code the catalogue did not have

`diag/fault.hpp` defines eleven raisable codes; `guide/11`'s table — billed as *"the complete
vocabulary"* — listed ten. `MECHANISM_STALLED` (F1) was missing, and `guide/06` promised that
table catalogued *every* code. Row added. The cross-reference was also softened to the truth:
ch. 6 narrates most codes, not all.

### Everything else, by document

- **`guide/09`** — *"mechanisms do not exist yet"* (F1 shipped them); `guide-09a`–**`09d`** (the
  `d` case was added at F1); `RoutineStopCause` listed three failure causes of four; and the
  freeze note implied any change needs a major bump, when the **additive** path has already been
  used for real (2.0 → 2.1).
- **`guide/10`** — *"Every verb takes a `MotionOptions` struct"*: `wait()` and `drive()` do not
  (verified in the frozen header). Its "covers every public operation" claim softened to the
  selective truth it already admits 230 lines later.
- **`guide/11`** — the blackbox's *"the piece that actually writes to `/usd/` … does not"* exist;
  `ProsBlockSink` does. `DR` *"always on (no correctors built yet)"* — two are built.
- **`guide/12`** — *"there is no robot"*; *"today, without correctors"*; *"until correction
  exists"*. All three stale. Added the dropped-USB entry, the only "no output" cause actually met
  on hardware.
- **`guide/13`** — *"there is no robot"*; *"two most likely extensions"* when it documents four;
  the layer map missing `sequence/`; the confirm-sensor list missing `IDigitalIn`, which for a
  clamp or a homed lift is the obvious one.
- **`guide/15`** — added **adapter, blackbox, mechanism, declared safe state, run guard,
  end-of-run action, triage**; corrected **telemetry sink** ("a *future* SD-card logger" — it
  ships), **tier** (only two of four exist), **step** (not every step delegates to a chassis
  command), **sim harness** (a shipped header, not test-suite property).
- **`cookbook/04`** — `strafeAuthority()` described as a property the library knows about your
  robot. On an H-drive it returns `strafeSpeedRatio × strafeTractionDerate`, both **your** config,
  the derate an invented 0.35 that no physical H-drive has ever measured.
- **`legacy-command-vocabulary`** — three pre-split "F2" owner references (that work is Phase F′;
  chunk F2 became the sequence engine and never carried it), a retired "M4 Sequencer" target, and
  a "lift the CSV before/at C7" action whose window closed on 2026-08-10 with the file now only in
  git history.

### The master plan's front matter was the worst single line in the public tree

`shulib-v2-master-plan.md:3` — **"Status: DESIGN-FIRST (no code yet) … sections marked _PENDING_
… Last updated 2026-06"** — on a document published to docs.shurobotics.com. All three clauses
false: the library is built; `grep -c _PENDING_` returns **1**, which is that line itself; and the
file already carried notes dated two months later. Rewritten to lead with the governing constraint.

Also: kernel *"pinned to the stale 4.1.0"* (`project.pros` says **4.2.2**); **gtest** named three
times for a **doctest** suite; and `hal/pros/*` described as *"the ONLY files that include
`<pros/*>`"* — the angle-bracket form is exactly what must never be used, because the robot build
resolves `-iquote` only. A reader following that line writes an adapter that passes every check
except the one build that ships.

**§13 extended** with the three R1a/R1b rulings that were locked and never recorded (the
screen→hold→expose idiom, the quoted-include rule, the digital-input seam built ahead of its
consumer) and — the part §13 had stopped doing — a table of what is **actually still open**: lift
homing, the sensor loadout, characterization timing, and the unverified ADI expander.

### Gates, re-run after everything

Suite **1120 / 1,523,324 / 3 skipped — green**. coverage · fresh · examples · removability ·
self-test · briefing · staleness self-test · staleness audit · prepare_site — **all PASS**. Both
CI guards PASS. ARM cross-compile of all **148** headers PASS.

---

## Rulings and open items

**`ORIENTATION.md` gets no public sibling.** It is mechanically publishable (clean of every
screened term, zero links), but `docs/README.md` *is* the site homepage and was already doing that
job badly. A sixth hand-maintained public document is a sixth thing to go stale, and this chunk
exists because things went stale. The orientation content was folded into the homepage instead.

**OPEN — for the team lead: the autonomous period length.** The cookbook asserted "autonomous is
fifteen seconds" four times, contradicting guide ch. 01 and the master plan (Skills = 1:00). The
hard-coded constant was **removed** rather than replaced, because the correct VEX U match figure
is not derivable from this repo and guessing it into the docs is exactly what this chunk exists to
prevent. It is also the right shape independently: F2's ruling is that the library holds no
default match length. **A number still needs confirming before any document states one.**

**NOT DONE, and it is a real gap:** `docs/hardware-assumptions.md` (≈21,000 words) and
`docs/internal/build-order.md` (109 KB) have not been read end to end by anyone. Two specific
build-order defects were fixed by pointer (the meaningless "195 commits behind", which the
correction commit never reached, and the `Next:` line), but neither document has had the
full-reading treatment this chunk is defined by. **The DoD item "every public document read end to
end" is therefore NOT met**, and since this pass *is* the release gate, that is the honest reason
to finish before merging.
