# Chunk C7 — COMPLETED (2026-08-10)

> Completion record for [`C7-cutover.md`](C7-cutover.md). Everything below is **as actually
> observed** — every command shown was run, every count checked. Changes are in the working
> tree, uncommitted, pending review (per the brief: do not commit, do not push).
>
> **C7 was the only irreversible chunk in the project.** It acted on C6's unconditional
> safe-to-delete verdict ([`C6-COMPLETED.md`](C6-COMPLETED.md) §8) and did not exceed it:
> exactly the 34 audited files were deleted, nothing else.

---

## 1. The one-sentence scope statement, repeated because it matters

**`make` succeeds and produces an uploadable V5 package. That package has NEVER run on a
robot and CANNOT drive one** — the `hal/pros` adapters are R1's deliverable and do not exist;
every hardware seam in `src/main.cpp` is a shipped in-memory fake marked `TODO(R1)` at the
exact line an adapter replaces. M2's **structural** clause closes here (the new tree is the
only tree); M2's **on-robot** clause ("`main.cpp` runs entirely on the new core", validated
on a V5) is **OPEN, owned by R3**. This caveat is stated in `src/main.cpp`'s header, the
README (third heading from the top), the roadmap (you-are-here, the WS11 bullet, and the M2
DoD status note), and here.

---

## 2. What this chunk produced

| Artifact | What it is |
|---|---|
| `src/main.cpp` *(new)* | The PROS entry point rewired onto the v2 core: full object graph (X-drive `MatrixKinematics` → fake HAL → `RobotContext` → `PilonsOdometry`/`ComplementaryFusion`/`Localizer` → `FaultLatch`/`HealthMonitor` → `MotionDeps` → pacer → `Chassis`), the same shape as the file-free construction test. Real on-target pieces: `StdoutCharSink` (V5 USB serial via newlib stdout) feeding A1's `TermSink`, and a `pros::delay`-backed pacer. Boot banner states the fake-backed status; `autonomous()` deliberately commands **no motion**; four `TODO(R1)` blocks name exactly what R1 adds |
| `include/main.h` *(rewritten)* | Minimal PROS project header: `api.h` + competition prototypes. Dropped: the deleted umbrella `shulib/api.hpp`, the LVGL include nothing used, and the header-level `using namespace pros/shulib` pollution |
| **Deletion** | `src/legacy/` + `include/legacy/` **gone** — §4 inventory |
| `.github/workflows/ci.yml` | PROS-free guard broadened to **all of `include/shulib/`** (whole-tree grep, no enumerated list to rot); layering guard broadened to the same scope via `--exclude-dir=sim`. Both mutation-proven (§7) |
| `README.md` *(rewritten)* | The front door: what it is / does / **is NOT** (never run on a robot — third heading), verification with scope (659 / 915,570, three drivetrains, hostile sim, all off-robot), build+test commands **executed as written**, a code example **compiled at full strictness for host AND the V5 target**, the layout and its two CI-enforced rules, pointers to the public docs only |
| **Docs reorganization** | `docs/` = public documentation, standing alone; `docs/internal/` = the process record, a **self-contained removable unit** — §5 map, §6 proof |
| Roadmap + build-order updates | You-are-here rewritten for C7; WS11 cutover bullet flipped `[~]` with the deviation note and evidence; M2 DoD annotated (does not close until R3 + D2); M0 promises marked fulfilled; stale "pending review" claims for the since-committed C2/C3/C4 fixed (`1206dbe`/`cad33bc`/`2a1cfc3`) |
| This record + [`C7-PROGRESS.md`](C7-PROGRESS.md) | The completion record and the live log (which itself moved mid-chunk with the reorg, as logged inside it) |

**Code shipped: `src/main.cpp` + `include/main.h` only. Tests shipped: none** — the suite is
deliberately unchanged (659 / 915,570 / 3 skips), which is itself the no-regression evidence
the brief demands. New test surface for the entry point would require a robot or a PROS host
shim; both are R-phase.

---

## 3. Order of operations — followed exactly, and what each step proved

1. **Re-read C6's verdict and table** — 34 files, zero unclassified, verdict unconditional.
   Nothing looked wrong; nothing was re-litigated; nothing beyond the 34 was touched.
2. **Rewired first, deleted second.** With `legacy/` still on disk, the new `main.cpp` built
   through the Makefile's own exclusion seam — no Makefile edit, a command-line variable:
   ```sh
   make clean && make EXCLUDE_SRCDIRS=./src/legacy    # SUCCEEDED, legacy still present
   ```
   So when the deletion happened, the build's health was already established — a post-deletion
   failure would have been attributable to the deletion alone. (There was none.)
3. **Then deleted:** `git rm -r src/legacy include/legacy` — 34 files staged deleted (18 + 16,
   matching C6 §4's count exactly). Then plain `make clean && make`: **SUCCESS** — the first
   working `make` since the June M0 quarantine broke it.
4. **Then broadened the guards** and re-verified everything (§7).

---

## 4. The deletion inventory

**Recovery pointer: every deleted byte is at `git show 691c656:<path>`** (`691c656` = the C7
brief commit, the last tree containing `legacy/`; the earlier pointer `357d3f2` recorded at C6
for the CSV specimen remains equally valid — the files did not change between). G4's lodged
follow-up (lift `autonomous_commands.csv` into its own fixtures) is unaffected and remains
lodged in the vocabulary doc.

`include/legacy/` — 18 files:
`shulib/api.hpp` · `shulib/chassis/chassis.hpp` · `shulib/chassis/drivetrain.hpp` ·
`shulib/chassis/drivetrain/tankdrive.hpp` · `shulib/chassis/drivetrain/xdrive.hpp` ·
`shulib/chassis/odomUnit.hpp` · `shulib/chassis/odometry.hpp` · `shulib/gui/gui.hpp` ·
`shulib/logger.hpp` · `shulib/pid.hpp` · `shulib/pose.hpp` · `shulib/util.hpp` ·
`shulib/RobotCommands/{autonomous_commands.h, Command.hpp, CommandArray.hpp,
CommandStruct.hpp, CommandType.hpp, MoveWithHeadingCommand.hpp}`

`src/legacy/` — 16 files:
`autonomous_commands.csv` · `gui.cpp.ignore` · `main.cpp` · `shulcd.c.ignore` ·
`shulib/chassis/{chassis.cpp, drivetrain.cpp, odomUnit.cpp, odometry.cpp}` ·
`shulib/GUI/{gui.c.txt, gui.cpp.txt, setonhalllogo.c}` · `shulib/logger.cpp` ·
`shulib/Pathing/autonomous_commands.c` · `shulib/pid.cpp` · `shulib/pose.cpp` ·
`shulib/util.cpp`

**34 = 18 + 16; the set is exactly C6 §4's classified set.** What deletion removed from the
tree, per C6: the source text of 11 catalogued live bugs, the only copied third-party code
(LemLib `pose.*`), and the ODR-trap duplicate declarations. What it could not remove: anything
irreplaceable — the knowledge left the directory at C6 (vocabulary doc, HA addendum,
diagnostics-plan note).

---

## 5. The docs reorganization map

Every move was a `git mv` (history follows). `docs/planning/` no longer exists.

| Old path | New path | Class |
|---|---|---|
| `docs/planning/roadmap.md` | `docs/roadmap.md` | public |
| `docs/planning/shulib-v2-master-plan.md` | `docs/shulib-v2-master-plan.md` | public |
| `docs/planning/hardware-assumptions.md` | `docs/hardware-assumptions.md` | public — **Phase R depends on it; verified complete without the chunk records (§6)** |
| `docs/planning/diagnostics-plan.md` | `docs/diagnostics-plan.md` | public |
| `docs/planning/legacy-command-vocabulary.md` | `docs/legacy-command-vocabulary.md` | public (+ post-deletion accuracy pass: its `legacy/` paths are now labeled historical, re-runs routed through `git show 691c656:<path>`) |
| `docs/planning/RESUMING.md` | `docs/internal/RESUMING.md` | internal (+ a new "Where documents live" section carrying the split and the dependency rule forward) |
| `docs/planning/build-order.md` | `docs/internal/build-order.md` | internal |
| `docs/planning/chunks/*` (34 files incl. this one) | `docs/internal/chunks/*` | internal |
| `docs/planning/transcripts/*` (17 files) | `docs/internal/transcripts/*` | internal (scouting/process material, not library documentation) |

**Nothing was deleted in the reorganization.** Cross-reference updates, all verified
mechanically (§6): 13 code/test files' `docs/planning/hardware-assumptions.md` pointers → `docs/…`;
`chassis.hpp`'s C4-record pointer → `docs/internal/…` with a "development log, `shulib-v2`
branch" annotation; `test/README.md`'s roadmap link; every relative link inside the moved
internal docs re-based.

---

## 6. The removable-unit property (coordinator refinement) — implemented and proven

**The rule:** `main` will eventually receive a squash-merge with `docs/internal/` removed, so
nothing outside `docs/internal/` may reference into it; public docs must read completely with
it absent. Dependency direction is one-way: internal → public only.

**How public docs were made self-sufficient** (not by pulling internal files public): every
link/pointer into `chunks/`, `build-order.md`, or `RESUMING.md` was replaced by inlining the
load-bearing content or re-attributing to *"the development log on the `shulib-v2` branch"* —
27 edits in `roadmap.md`, 9 in `hardware-assumptions.md`, 2 in `legacy-command-vocabulary.md`,
1 in `diagnostics-plan.md`, 0 needed in the master plan.

**The checks, as run:**

```text
$ grep -rnE 'docs/internal|internal/|chunks/|RESUMING|build-order|-COMPLETED\.md|-PROGRESS\.md' \
      README.md test/README.md docs/*.md
EMPTY — no public doc references the internal set

$ mv docs/internal <aside> && python3 check_links.py docs README.md test/README.md
scanned 7 markdown files, checked 23 file-target links
all links resolve                                   # ← with docs/internal ABSENT
$ mv <aside> docs/internal                          # restored intact
```

**`rm -rf docs/internal/` leaves a coherent, fully-linked documentation set — proven, not
asserted.**

**Interpretation, stated so the check is honest (D7):** the grep patterns above cover every
*file* reference (paths, filenames, link targets). Bare chunk **ids** (A1…C7, R1…R6) remain in
public docs and code comments as dated provenance vocabulary — the same role they play in every
header ("chunk C4, WS6/M2") — because they are meaningful without any internal file existing
and are how the roadmap's evidence notes are dated. No public sentence *depends on* an internal
document to parse; every pointer that named an internal file now names the development branch
instead. Residual, named honestly: code comments that cite the development log (e.g.
`chassis.hpp` → `docs/internal/chunks/C4-COMPLETED.md`, `test/localizer_test.cpp` → A3's
record) reference a path that exists on `shulib-v2` but will not exist on `main`; each is
annotated "development log, shulib-v2 branch" so it is self-describing when absent. The
merge-time sweep may choose to strip them; nothing breaks either way.

---

## 7. Verification (actually run, outputs as observed)

**Host suite — unchanged baseline, before and after every phase of this chunk:**

```text
$ cmake -S test -B build/test && cmake --build build/test -j && ./build/test/shulib_tests
[doctest] test cases:    659 |    659 passed | 0 failed | 3 skipped
[doctest] assertions: 915570 | 915570 passed | 0 failed |
[doctest] Status: SUCCESS!
```

**`make` — the DoD's centerpiece** (run three ways: excluded-legacy before deletion, plain
after deletion, and once more after all doc/README edits):

```text
$ make clean && make
Creating cold package with libc,libm,libpros [OK] … Stripping cold package [DONE]
Compiled src/main.cpp [WARNINGS]     ← ONE warning, kernel-owned (see below)
Linking hot project with ./bin/cold.package.elf and libc,libm,libpros [OK]
Creating cold package binary for VEX EDR V5 [DONE]
Creating bin/hot.package.bin for VEX EDR V5 [DONE]
```

The one warning is `-Wdeprecated-enum-enum-conversion` inside the PROS template's own
`include/liblvgl/core/lv_obj_style.h` (reached via `api.h`), a known kernel-header artifact
under gnu++20 — vendored code, not ours, not `-Werror` in the PROS build, left untouched.

**Both CI guards, at the broadened scope, plus their mutations:**

```text
$ <ci.yml PROS guard, scope = all of include/shulib>     → library is PROS-free (all of include/shulib)
$ <ci.yml layering guard, scope = include/shulib minus sim/> → layering holds: core is sim-free
MUTATION1: planted '#include <pros/rtos.h>' in math/angle.hpp    → guard FIRES (red), restored
MUTATION2: planted '#include "shulib/sim/drive_plant.hpp"' there → guard FIRES (red), restored
(sim's internal sim-includes confirmed exempt — the --exclude-dir is load-bearing, not vacuous)
```

**ARM compile gate:** `TU includes 102 headers` → `ARM compile gate: CLEAN (all v2 headers)`.

**README honesty checks:** its four build/test commands executed exactly as written (outputs
above); its code example extracted verbatim from the markdown fence and compiled with the
suite's full flags on host **and** with the V5 flags (`-mcpu=cortex-a9 -mfpu=neon-fp16
-mfloat-abi=softfp`) — both clean. `pros upload` was **not** executed (requires a V5; the
README says so itself).

**Legacy references:** grep `'legacy/'` + legacy `#include` directives over `include/shulib`,
`src`, `test`, `Makefile`, `common.mk`, `.github` → **zero**. Remaining word-level "legacy"
matches are: design-history prose in v2 headers/tests (the escapeJSONString lesson, the
arc-step bug), VEX hardware terminology in vendored `pros/`/`liblvgl/` headers ("legacy ADI
ports", "Legacy LCD Emulator" — not ours), and the historical/audit records in `docs/` whose
subject *is* the deleted tree (vocabulary doc, roadmap deletion evidence — every such path now
explicitly labeled historical with its `git show` retrieval).

**Doc links:** 40 markdown files scanned, 65 file-target links, **all resolve** (final run
below, after this record was written). Plus the §6 absence proof.

---

## 8. What `make` now produces — and what it does not do

Produces (all under `bin/`, gitignored):

| Artifact | Size | What it is |
|---|---|---|
| `hot.package.bin` | 14,704 B | **The uploadable user-code image** (hot half of PROS hot/cold linking): `main.cpp` + the entire wired v2 stack |
| `cold.package.bin` | 1,356,264 B | The uploadable PROS kernel image (cold half; rebuilt only when the kernel changes) |
| `hot.package.elf` / `cold.package.elf` | — | The linked ELFs the .bins are objcopied from |
| `main.cpp.o`, `_pros_ld_timestamp.o` | — | The only project objects (nothing else lives in `src/`) |

Upload path (untested — needs hardware): `pros upload`, which flashes both packages.

**What the uploaded package would do:** boot; construct the full v2 object graph on the V5's
CPU; print, over USB serial through the v2 diagnostics layer itself, an honest banner (stack
wired · HAL fake-backed · cannot drive) plus one live query through the facade
(`strafeAuthority=1.00`); log-and-idle in `autonomous()`/`opcontrol()`.

**What it does NOT do — none of this is claimed anywhere:** drive a motor, read a sensor,
run an auton, or demonstrate anything about physical hardware. It has never been uploaded.
Also deliberately absent until R1: the PROS-side git-hash injection + session header emission
(a missing hash is loud by design — C5), the on-robot precondition policy handler, the
tick-boundary pacer, and any teleop `drive()` loop. Each is a named `TODO(R1)` in
`src/main.cpp`.

---

## 9. Decision log

### D1 — Pre-deletion build via `make EXCLUDE_SRCDIRS=./src/legacy`, not a Makefile edit
common.mk's `rwildcard`/`GETALLOBJ` already thread an exclusion parameter; a command-line
variable exercises it without touching tracked build files. **Rejected:** temporarily editing
the Makefile (a tracked-file edit that exists only to be reverted is review noise and a
foot-gun if forgotten).

### D2 — `main.cpp` wires the shipped fakes; `autonomous()` commands no motion
The wiring itself is the deliverable: a complete object graph constructing and linking inside
a PROS binary, with each fake sitting in the exact slot R1's adapter fills (the swap-only
property `RobotContext` exists for). **Rejected:** (a) a stub `main.cpp` that wires nothing —
would satisfy "make succeeds" while proving nothing about the v2 stack on the target;
(b) writing partial `hal/pros` adapters now — R1-by-stealth, unreviewable without hardware,
and exactly the overclaim this chunk's scope line forbids; (c) running motions against fakes
in `autonomous()` — on a live field indistinguishable from a hang, and it would *look* like a
driving auton in the one place honesty matters most.

### D3 — Default (throwing) precondition policy retained in `main.cpp`
`check.hpp` assigns the on-robot policy installation to "R1's hal/pros bootstrap"; installing
a half-designed robot policy from `main.cpp` now would pre-empt that design. The wiring is
valid by construction, so no precondition can fire in the shipped flow. Named as `TODO(R1)`.

### D4 — The pacer advances the fake clock alongside `pros::delay`
A fake clock that never advances trips C2's stalled-pacer tripwire on the first blocking verb
— correct behavior, but the wired graph should be *self-consistent*, not booby-trapped. The
advance line carries its own `TODO(R1): delete with the fake clock`.

### D5 — Guards broadened by construction, not enumeration
PROS guard: whole-tree `grep include/shulib` (a new directory is covered the moment it
exists). Layering guard: same tree with `--exclude-dir=sim`. **Rejected:** extending the
enumerated directory lists — the A4 lesson ("a gate you must remember to update is a gate that
rots") applies to guards identically. Both mutations run and observed red (§7).

### D6 — Public docs made self-sufficient by inlining/re-attribution, never by promotion
Where a public doc leaned on an internal file, the load-bearing content was inlined or the
pointer re-attributed to the development branch. **Rejected:** moving any chunk record or the
build order into `docs/` (violates the removable-unit direction), and leaving "see the
completion record" pointers that dangle on `main`.

### D7 — The chunk-id interpretation (recorded in §6)
File references into internal: eliminated, grep-proven. Bare chunk ids as dated provenance:
retained. The alternative — scrubbing ids from the roadmap's evidence notes and every code
header — would rewrite the project's provenance vocabulary across ~100 files for no removal
benefit, and was rejected as destruction masquerading as hygiene.

### D8 — No session-header emission from `main.cpp` yet
C5 made a missing build hash LOUD (`[ERROR][SES] … MISSING`) by design; emitting the header
before the PROS Makefile injects the hash (R1's declared job, mirroring `test/CMakeLists.txt`)
would print a deliberate error on every boot of a package that cannot drive — noise without a
consumer. The boot banner covers run identification until R1.

### D9 — "Zero references to `legacy/`" scoped to live references
Includes, build rules, links, and current-tree instructions: zero, grep-proven. Historical
records whose *subject* is the deletion (C6's audit, the vocabulary doc, the roadmap's
deletion evidence, this record) necessarily name the old paths — each now labeled historical
with its `git show` retrieval. Scrubbing the audit of the thing it audited would be falsifying
the record.

### D10 — `transcripts/` classified internal
Season-scouting notes are process material; a public visitor looking for library docs should
not find YouTube transcript dumps beside the roadmap.

### D11 — `main.h` kept as the PROS-convention bundle header
`api.h` + competition prototypes only. The include-cleaner warning ("api.h not used directly")
is the file doing its intended re-export job. Dropped the umbrella/LVGL/`using namespace`
lines with reasons in-file.

---

## 10. Documentation contract (all six)

1. **Roadmap checkboxes flipped with evidence** — WS11 cutover bullet `[~]` (structural half
   closed with artifacts named; hardware-validation explicitly R3's, F6 freeze explicitly
   D2's); M0 guard-scope and quarantine notes annotated fulfilled; M2 DoD status note added.
2. **You-are-here updated** — Phase C COMPLETE, C7 done pending review, the on-robot clause
   named OPEN, next = D1.
3. **Design notes in headers** — `src/main.cpp` (what the binary is/is not, why fakes, the
   R1 seam list), `include/main.h` (what was dropped and why), `ci.yml` (broadened scopes and
   why generated), README (the two structural rules).
4. **Test evidence recorded** — §7: suite unchanged 659/915,570/3; two guard mutations red;
   ARM gate 102 clean; README example compiled twice; commands executed as written.
5. **Decisions recorded** — §9, eleven decisions with alternatives.
6. **Freeze Register** — **C7 freezes nothing and touches nothing frozen.** F1–F5 stand; F6
   remains deliberately unfrozen until D2 (this chunk's on-robot consumer for the freeze
   argument did not materialize — see the note below). M2's structural clause closes; its
   on-robot clause is recorded OPEN, owned by R3.

*Freeze-register note carried forward:* C6's record said "the next freeze event on the path is
D2 (F6), after C7 supplies the on-robot consumer." C7 must correct that expectation honestly:
it supplied the *structural* consumer (`main.cpp` wires the facade), **not** an on-robot run.
The build-order already places the freeze at D2 on the strength of C4's auton + D1's recipe
layer; the on-robot exercise remains R3's, and the D2 freeze rationale should cite consumers
that actually ran (host ones) — not this binary.

---

## 11. DoD checklist (brief §Definition of Done)

- [x] `main.cpp` wires the v2 stack with explicit `TODO(R1)` HAL seams — §2, §8
- [x] `src/legacy/` and `include/legacy/` are gone — §4 (34 = 18+16, count-checked)
- [x] `make` succeeds; uploadable package; artifacts named — §7, §8
- [x] CI PROS-free guard covers all of `include/shulib/` and passes — §7 (mutation-proven)
- [x] Host suite, both guards, ARM gate green; no regression — §7 (suite bit-identical counts)
- [x] Zero references to `legacy/` — §7, scoped per D9 (live refs: zero, grep-verified;
      historical audit records labeled and retrieval-routed)
- [x] README rewritten; commands executed as written; "never run on hardware" plain and early — §2, §7
- [x] Docs reorganized; nothing deleted; all links checked mechanically — §5, §6
- [x] **M2's structural clause closes** — the new tree is the only tree (§3, §4)
- [x] The on-robot clause explicitly recorded as **still open, owned by R3** — §1, §8, §10,
      roadmap M2 DoD note

Additional (coordinator refinement): docs/internal/ is a self-contained removable unit,
proven by grep and by absence — §6.

---

## 12. What we now know for certain, and what we do not

**Certain (demonstrated):** the v2 object graph constructs and links inside a real PROS binary
at the V5's ABI; the PROS toolchain path (kernel cold package + hot user image) is healthy;
the tree contains exactly one PROS-touching file; the documentation set splits cleanly with a
one-way dependency; the suite is unaffected by any of it.

**Not known, and not claimed:** whether the binary boots on a physical V5 (high confidence,
zero evidence); whether stdout reaches `pros terminal` with the expected framing; every single
hardware assumption in `docs/hardware-assumptions.md` (all 49 remain open); and everything
R1–R6 exist to establish. The first hardware contact is a prepared sequence, not a surprise —
that was the point of building it this way.
