# Chunk C6 — COMPLETED (2026-08-10)

> Completion record for [`C6-legacy-salvage.md`](C6-legacy-salvage.md).
> Everything below is **as actually observed** — every file read in full, every count scripted,
> every claim checked against source. Changes are in the working tree, uncommitted, pending review.
>
> **This chunk's product is an audit, not code. The port list is empty — and §3 shows why that is
> the finding, not a shortfall.** C7 acts on §8's safe-to-delete statement.

---

## 1. What this chunk produced

| Artifact | What it is |
|---|---|
| [`docs/legacy-command-vocabulary.md`](../../legacy-command-vocabulary.md) *(new)* | Every command id ever declared or authored in `legacy/` (7 distinct), mechanically cross-checked from all four sources, each mapped to its v2 home or flagged as a gap; the two data-file dialects characterized as G4 importer specimens (7 concrete importer requirements); the de-facto hand-written verb vocabulary for F2 |
| [`hardware-assumptions.md`](../../hardware-assumptions.md) §"Legacy-measured reference points" *(new section)* | The old robot's hard-won numbers (odom offsets ±6.5″/2.5″, 2.75″ tracking wheels, 36000 ticks/rev provenance for HA-16, measured min-output 20/25 of 127 ⇒ kS ≈ 1.9 V vs HA-45's 1.0 V placeholder, drive geometry, full port map, two calibration-routine specimens) — scoped explicitly as *evidence and R-phase provenance, not HA values* (different robot) |
| [`diagnostics-plan.md`](../../diagnostics-plan.md) §"Legacy evidence, mined at C6" *(new section)* | The two legacy diagnostic capabilities worth keeping as requirements: per-tracking-wheel stuck *identification* (→ the E-phase estimator-side detector `odo_stall_check.hpp` already names) and drive veer/imbalance triage (→ H2/R-phase bench tooling); plus the 900-byte serial-backpressure corroboration for E1 |
| This record | The 34-file classification (§4), the three roadmap-claim verifications (§2), the decision log (§5), and the safe-to-delete verdict (§8) |
| Roadmap + build-order updates | Salvage checkbox flipped with evidence; "you are here" moved to C6 |

**Code shipped: none. Tests shipped: none.** Per the brief: "an empty port list is a fine outcome
if the audit justifies it" — §3 is that justification, made file by file in §4. Because no logic
was ported, the brief's mutation requirement ("a proven-red mutation **if it carries logic**") is
vacuously discharged; the mechanical-cross-check requirement on the vocabulary was executed and its
commands are preserved re-runnably in the vocabulary doc §6.

**Freeze Register note (documentation contract #6):** C6 freezes nothing and touches nothing
frozen — F1–F5 stand untouched, and F6 remains deliberately unfrozen until D2. The next freeze
event on the path is D2 (F6), after C7 supplies the on-robot consumer.

---

## 2. The three roadmap salvage claims — verified against source, not assumed

### Claim 1 — "the Pilons arc math was re-derived at M2 (`arcStep`)" — ✅ VERIFIED, both halves

- **The legacy bug is real and is where the roadmap says it is.** `src/legacy/shulib/chassis/odometry.cpp`
  lines 338–355: `odomPose.theta += localPose.theta;` executes **first**, then the chord is rotated
  by `cos/sin(odomPose.theta)` — the **new** heading, not the tick-average. Exactly the bug the
  roadmap describes.
- **The v2 replacement exists and designs it out.** `include/shulib/localization/arc_step.hpp`
  derives the exact SE(2) step via half-angle identities and rotates by the **average** heading
  `θ₀ + Δθ/2` (lines 25–39 derivation; line 99 `thAvg`), and its header comment names the legacy
  bug it was re-derived to avoid.
- **Archaeological bonus:** legacy `estimatePose()` (`odometry.cpp:119`) used
  `avgHeading = θ + Δθ/2` **correctly** — the prediction path did what the integration path didn't.
  The bug was an inconsistency they had already half-solved and never noticed.

### Claim 2 — "`logger.hpp` was superseded by A1's clean-room diagnostics, no port needed" — ✅ VERIFIED mechanically

All three defects confirmed in source (grep results in `C6-PROGRESS.md`):

| Defect | Evidence in legacy source | v2 supersession |
|---|---|---|
| `escapeJSONString` written but never applied | Defined `logger.hpp:141`; **zero call sites** in the whole tree; messages enter JSON raw at `logger.cpp:36` | `TermSink::Line::appendSanitized` is the *only* text path (A1 D4) |
| `sendDebugMessages` dead | Declared `logger.hpp:172`, **called** `logger.cpp:17`, never defined — the TU cannot link | No dead paths shipped (every public member tested, A1 D4) |
| Manual `update()` racing the background flush | `init()` spawns a 100 ms `sendTelemetry` task while `update()` also calls it; `pros::delay(5)` held **under the mutex** (`logger.cpp:59,100`) | No background task, no shared mutable buffers exist in `diag/` (A1 D4) |

Every *role* the logger played has a v2 owner: telemetry snapshots → `DebugRecord`/`emit()` (A1),
messages → `log()` levels + `TermSink` (A1), per-motion/run reporting → C5, machine-readable wire →
H1 `SHUL/2` (planned, schema already frozen in shape). **Nothing to port; nothing was ported.**

### Claim 3 — "`RobotCommands` → `sequence/` seed is the real remaining work" — ✅ resolved, with a finding that CHANGES its shape

The brief expected a possible port seed. The audit found **there is no code to seed from**:

- **No executor ever existed.** The three parallel data representations (C array, CSV, C++ vector)
  have zero consumers; `Chassis::commands` is never populated; the one OO command's `moveTo()` body
  is empty. Verified by grep (vocabulary doc §1).
- The declarations themselves are an ODR trap (same enum twice, same struct twice, a `Command`
  class colliding with an `extern "C"` `Command` struct).

So the salvage is **knowledge, not code** — exactly what the brief's §"The real work" predicted —
and it is complete: the vocabulary document carries every id, both data dialects, and the
requirements they impose on F2/G2/G4.

**Stale-plan flag raised by this verification:** master plan §14 says the M4 time-budgeted
sequencer will be built "*extending the existing `RobotCommands`/`Command` queue*". C6 establishes
there is no queue to extend — that clause is counterfactual and the sequencer must be (and per
build-order already is) planned as a fresh build. Flagged here so M4/G2 briefs don't inherit the
phantom dependency. *(Not edited in the master plan itself: §16.3's design is already
extension-free; the stale words are §14 color, and master-plan edits are not C6's to make
unilaterally — decision D6.)*

---

## 3. Why the port list is empty (the audit's core argument)

Four candidate categories were weighed for porting; each fails on the same two clean-room tests —
*is there a v2 consumer today?* and *would re-derivation at the consumer's chunk be better than a
port now?*

1. **Motion/odometry/control code** — superseded outright by C1–C5/M2, which exist *because* this
   code's defects (11 distinct ones catalogued in §4's notes) made clean-room the policy. Nothing
   here is better than what replaced it.
2. **The command pipeline** (`RobotCommands`, data files) — no executor ever existed to port; the
   value is requirements, now captured. Building a runner now would be building G2 early, which the
   brief forbids.
3. **The diagnostics ideas** (wheel-health, veer triage) — real value, wrong time and wrong form:
   their v2 consumers are the E-phase estimator-side detector and H2/R-phase bench tooling, neither
   of which exists yet. Porting now ships dead code (violates A1's no-dead-paths rule); the
   requirements are lodged with their owners in `diagnostics-plan.md`.
4. **Utility scraps** (`slew`, `getCurvature`, `ema`…) — trivial to re-derive at need (minutes
   each), several carrying live bugs (`angleError`'s double-sanitize; `sanitizeAngle`'s
   constexpr-in-cpp NDR), and `getCurvature`'s consumer (pure pursuit) is deliberately not planned.
   A port would preserve risk to save nothing.

---

## 4. The classification — all 34 files, none unclassified

Legend: **SUP** superseded by X · **SALV** salvaged (where it went) · **DISC** discard, nothing of
value. Files carrying both code (superseded/discarded) and knowledge (salvaged) show both; the
knowledge column is what survives deletion.

### `include/legacy/` (18 files)

| # | File | Verdict | Reason / where it went |
|---|---|---|---|
| 1 | `shulib/api.hpp` | **DISC** | 5-line umbrella include. v2 policy is include-what-you-use; no umbrella header exists or is wanted |
| 2 | `shulib/chassis/chassis.hpp` | **SUP** → `shulib/chassis/chassis.hpp` (C4) + `motion/motion_scheduler.hpp` (C2) + M2 localization | Same verbs, done with contracts (exit reasons, watchdogs, frames). Legacy defects noted: `Drivetrain` stored **by value** (every derived drive object-sliced); `commands` vector declared, never used. `AngularDirection` concept (CW/CCW/AUTO forced-direction turns) already in `TurnTo` |
| 3 | `shulib/chassis/drivetrain.hpp` | **SUP** → `kinematics/` (F5: `MatrixKinematics` + pseudo-inverse) + `hal/motor.hpp` (F4) | The `MotorConfig` coefficient table is a hand-rolled kinematics matrix without desaturation or geometry — F5 is its correct form. `getTemps()` role → `HealthMonitor`/`DebugRecord` per-wheel fields |
| 4 | `shulib/chassis/drivetrain/tankdrive.hpp` | **SUP** → `kinematics/tank.hpp` | Note: stored `trackWidth` but never used it — legacy tank turns ignored geometry entirely (turn coefficient ±1 regardless) |
| 5 | `shulib/chassis/drivetrain/xdrive.hpp` | **SUP** → `kinematics/x_drive.hpp` | Legacy ±1 coefficients: no 45° projection, no desaturation — both fixed in F5 |
| 6 | `shulib/chassis/odometry.hpp` | **SUP** → `localization/pilons_odometry.hpp` + `localizer.hpp` (M2) | Global-function API + x/y/θ "correction factors" (scale band-aids; v2 fixes causes: IMU-owned heading, calibrated wheel scale at M3). `estimatePose(t)` (linear extrapolation) has its v2 analogue in M3's latency compensation |
| 7 | `shulib/chassis/odomUnit.hpp` | **SUP** → `localization/tracking_wheel.hpp` (role-stamped) + `hal/rotation.hpp` | Same concept, with typed units and a seam |
| 8 | `shulib/gui/gui.hpp` | **DISC** | LVGL screen stub (3 members). The one idea worth keeping — glanceable alliance-color/status screen — is already planned as D-12 `BrainHud` |
| 9 | `shulib/logger.hpp` | **SUP** → `diag/` (A1, completed C5) — clean-room, per §2 claim 2 | The three defects became design constraints, not code. Nothing ported |
| 10 | `shulib/pid.hpp` | **SUP** → `control/pid.hpp` (M2 WS4) | Legacy had no integral clamp (windup), derivative-on-error, unguarded ÷time |
| 11 | `shulib/pose.hpp` | **SUP** → `math/pose2d.hpp` + `math/angle.hpp` (F1/F3) | **Line 1: "// from lemlib" + URL — this is copied third-party (MIT) code.** Deleting it removes the tree's only copied-in library code; v2 math is original + typed. Unitless float x/y/θ vs typed `Length`/`Angle` |
| 12 | `RobotCommands/autonomous_commands.h` | **SALV** (vocabulary → `legacy-command-vocabulary.md`) / code **DISC** | The 6-id C enum + 5-field payload + extern decls (duplicate #1 of 2). No consumer ever existed |
| 13 | `RobotCommands/CommandArray.hpp` | **DISC** (shape recorded in vocabulary doc) | Extern decls for a `CommandStruct` array nobody defined a reader for |
| 14 | `RobotCommands/Command.hpp` | **DISC** | Abstract `execute()` interface; name-collides with the C `Command` struct (ODR trap). Pattern superseded by C1 `IMotion` + G2 registered handlers |
| 15 | `RobotCommands/CommandStruct.hpp` | **SALV** (the 5-field row shape is the G4 specimen schema) / code **DISC** | Duplicate payload struct #2 of 2 |
| 16 | `RobotCommands/CommandType.hpp` | **SALV** (vocabulary) / code **DISC** | Duplicate 6-id enum #2 of 2. Misses `NONE`, which its own sibling CSV uses — the round-trip gap G4 must handle |
| 17 | `RobotCommands/MoveWithHeadingCommand.hpp` | **DISC** | The only concrete command; its private `moveTo()` body is **empty**. Maps to C1 `MoveToPose`, which exists |
| 18 | `shulib/util.hpp` | **SUP** → `math/angle.hpp` (wrap/shortest-path, F3), `units/` (conversions), `control/trapezoid_profile.hpp` (slew's role) | LemLib-derived helpers. Live bugs: `angleError` sanitizes `target` twice and `position` never; `sanitizeAngle` declared `constexpr` in-header, defined in the .cpp (ill-formed NDR). `getCurvature` (pure-pursuit/boomerang) has no v2 consumer by design — re-derive if path-following ever wants it |

### `src/legacy/` (16 files)

| # | File | Verdict | Reason / where it went |
|---|---|---|---|
| 19 | `autonomous_commands.csv` | **SALV** — characterized in `legacy-command-vocabulary.md` §4 (dialect A: inches, 18 `NONE` segment markers, constant speed 50, ~1″ spacing, 826+18 rows) | The G4 importer's primary real specimen. **Action lodged with G4:** lift a copy into G4's test fixtures from git history at/after C7 (`git show 357d3f2:src/legacy/autonomous_commands.csv`) |
| 20 | `gui.cpp.ignore` | **DISC** | 21 lines of LVGL scaffolding; constructor shadows its own members with locals, wiring nothing. Already `.ignore`-suffixed by the team |
| 21 | `main.cpp` | **SALV** (knowledge) / code **SUP** → C1–C5 + C7's fresh `main.cpp` | The richest file: full port map + geometry (→ HA addendum), measured min-outputs 20/25 (→ HA addendum, kS evidence), 2 calibration-routine specimens (→ HA addendum, M3/R5), the de-facto verb vocabulary + `oscillation` wiggle gap (→ vocabulary doc §3), veer/imbalance triage (→ diagnostics-plan). Code itself: hand-written blocking autons on the defective legacy stack; the active `autonomous()` is 3 lines, the real routine is commented out. Defects: `move_to_pose` computes then **discards** its heading correction; `rotate_to` wraps at ±181°; PID dt passed as `5` (s) in one loop and `0.005` in the next |
| 22 | `shulcd.c.ignore` | **DISC** | Half-copied PROS llemu internals (LVGL 480×240 scaffolding), never referenced, `.ignore`-suffixed |
| 23 | `shulib/chassis/chassis.cpp` | **SUP** → C4 facade + C2 scheduler + M2 calibration path | Defects: right-wheel null check throws "**Left** tracking wheel not initialized" (copy-paste); `moveToLocalPose` is dead scaffolding (unused int-truncated `rotations`, open-loop `move_relative`, `async` ignored). IMU calibrate-retry×5-then-degrade-to-null pattern: v2's analogue is HA-23 boot handling + `HealthMonitor` IMU-after-ready semantics (R1 wires it) |
| 24 | `shulib/chassis/drivetrain.cpp` | **SUP** → `kinematics/` + `math/frame.hpp` (field-centric via typed frames) | **Field-centric rotation bug:** `horizontal` overwritten then used to compute `vertical` — every legacy field-centric drive was wrong (plus int-truncated trig). v2's frame transform is exact and tested |
| 25 | `shulib/chassis/odometry.cpp` | **SALV** (per-wheel health requirements → diagnostics-plan; port/offset provenance → HA addendum; the claim-1 bug evidence → §2) / code **SUP** → `arc_step.hpp` + `pilons_odometry.hpp` + `localizer.hpp` | The new-heading integration bug (§2); the (sL==sR) ÷0 guard present on the *logging* path, absent on the *math* path one page later; the IMU plumbed in but **never read** (wheels-only heading — the root cause of their θ-drift misery); plain `abs()` on float with `<math.h>` (implementation-defined int truncation hazard) |
| 26 | `shulib/chassis/odomUnit.cpp` | **SUP** → `tracking_wheel.hpp` + `hal/rotation.hpp`; provenance **SALV** → HA-16 note | The `position·πd/36000` centidegree conversion is HA-16's "measured elsewhere" — it ran all season on a real robot |
| 27 | `shulib/GUI/gui.cpp.txt` | **DISC** | Non-compiling draft: free function referencing `ui->` with no `ui` in scope; `.txt`-suffixed by the team as abandoned |
| 28 | `shulib/GUI/gui.c.txt` | **DISC** | Abandoned C draft (`pros::delay` inside `.c`, ad-hoc OO-in-C). Alliance-color toggle idea → D-12, already planned |
| 29 | `shulib/GUI/setonhalllogo.c` | **DISC** | 997-line LVGL bitmap of the Seton Hall logo — an asset, not code; regenerable from any source image with LVGL's converter if a D-12-era boot splash ever wants it |
| 30 | `shulib/logger.cpp` | **SUP** → `diag/` clean-room (§2 claim 2 evidence lives here) | The unlinkable `sendDebugMessages()` call, the race, delay-under-mutex, raw-string JSON; the 900-byte chunking (→ diagnostics-plan corroboration note) |
| 31 | `shulib/Pathing/autonomous_commands.c` | **SALV** — characterized in vocabulary doc §4 (dialect B: unit-ambiguous coordinates y≤343 that **cannot** be field inches, per-waypoint speed ramps 41.8→100, no marker rows) / code **DISC** | The *third* duplicate of enum+struct, plus 635 generated rows. Its unit ambiguity is G4 importer requirement #1 |
| 32 | `shulib/pid.cpp` | **SUP** → `control/pid.hpp` | See #10 |
| 33 | `shulib/pose.cpp` | **SUP** → `math/pose2d.hpp` | LemLib-copied implementation (see #11) |
| 34 | `shulib/util.cpp` | **SUP** → `math/angle.hpp` + std | The `angleError` double-sanitize bug lives here (lines 20–21); `avg()` divides by zero on an empty vector |

**Count check: 18 + 16 = 34. Every file classified.** Defect tally across the audit: **11 distinct
live bugs** found in legacy code during this read (slicing, field-centric rotation, angleError,
constexpr NDR, ÷0 heading math, new-heading integration, discarded heading correction, ±181 wrap,
dt=5s, copy-paste error message, unlinkable logger) — none of which exist in v2, all of which a
port would have risked importing. This is the clean-room policy's closing argument.

---

## 5. Decision log

### D1 — Empty port list
Chosen over porting the wheel-health monitor (the only genuinely tempting candidate). **Rejected
porting because:** its correct v2 home is the E-phase estimator-side detector that
`odo_stall_check.hpp` already names as deferred; landing it now creates dead code (no consumer
calls it — violates A1's no-dead-paths rule), pre-empts E-phase design (which owns per-wheel
plausibility as an *estimator* concern, not a log-string concern), and the brief's own landmine
warns "don't port bad code to feel thorough." The requirements — which-wheel attribution, mutual
comparison, structured output — are lodged with the owner in `diagnostics-plan.md`.

### D2 — Legacy constants recorded as *evidence*, not as HA values
The old robot's numbers (±6.5″ offsets, 2.75″ wheels, kS≈1.9 V) describe a **different machine**.
Overwriting HA stand-ins with them would swap one unmeasured guess for another while *looking*
measured — worse than invented, because it reads as settled. **Rejected:** silently updating HA-12/
13/45. **Chosen:** a clearly-scoped reference section whose stated function is to steer R-phase
attention (kS likely nearer 2 V than 1 V) and corroborate magnitudes.

### D3 — `NONE`'s v2 fate is G2's decision, recorded as a gap, not decided here
The brief forbids building G2. C6 could have quietly "recommended" mapping `NONE` → a canonical
`no_op` id; instead the vocabulary doc states the two honest options (an explicit marker id vs
structural absorption into `paths[]` segments) and assigns the decision to G2, with G4 owning the
mapping either way. Deciding vocabulary now, outside the manifest design, would be G2-by-stealth.

### D4 — The CSV's future: git history + a lodged G4 action, not a moved file
**Rejected:** copying `autonomous_commands.csv` into `test/fixtures/` now (creates an unused
fixture three phases early, in C6, a chunk forbidden from touching the build). **Chosen:** the
exact retrieval command recorded in §4 row 19 and in the vocabulary doc, and the action lodged
against G4 (whose brief-time reader will find it in the vocabulary doc's §4 requirements).
Git retains the bytes permanently either way; what C6 adds is the *pointer that someone will read*.

### D5 — Roadmap checkbox flipped with reworded evidence rather than left open or silently reworded
The M2 bullet reads "port `RobotCommands`→`sequence/` seed, `logger.hpp`→`io/Telemetry`…". Two of
its three items were already done under different (better) forms, and the third turned out to be
knowledge-salvage. **Chosen:** flip to `[x]` with an italic evidence note explaining that the
salvage completed in a different form than the 2026-06 wording imagined (per-item mapping), keeping
the original text visible. **Rejected:** editing the bullet text itself (rewriting history) or
leaving it `[ ]` (falsely implying work remains before C7).

### D6 — The stale master-plan §14 clause is flagged, not edited
"Extending the existing `RobotCommands`/`Command` queue" is counterfactual (§2 claim 3). The master
plan is the **why** document with its own locked-decision discipline; C6's mandate is the audit.
The flag lives here and in the vocabulary doc §1, where the M4/G2 brief-writer will hit it.

### D7 — LemLib provenance called out in the audit
`pose.hpp`/`pose.cpp` (and `util.hpp`'s doc style) are copied/derived from LemLib (MIT — legal,
attributed in-file). Recorded because it materially strengthens C7: deletion removes the only
third-party-copied code in the tree, aligning the whole repo with the clean-room/G4-guardrail
posture. Not a reason to delete by itself; a reason the deletion is *cleaner* than expected.

---

## 6. Discoveries flagged for later chunks

| Owner | Discovery |
|---|---|
| **G2** | `NONE` boundary-marker decision (vocabulary §5); registry-not-enum principle documented from legacy's failure; no legacy id demands a canonical-vocabulary addition beyond §14/§16.3's plan |
| **G4** | 7 concrete importer requirements (vocabulary §4), the two dialects, the unit-instability trap (one specimen is *not* in field inches), commands-as-rows→markers conversion, the CSV fixture retrieval action |
| **F2** | The de-facto verb table (vocabulary §3); the `oscillation` seat/settle wiggle as a candidate micro-primitive; confirmation every other hand-written verb maps to a planned §14 primitive |
| **E-phase** (estimator-side odo detector) | Per-wheel stuck *identification* requirements from the legacy mutual-comparison monitor (diagnostics-plan C6 note) |
| **H2 / R-phase** | Veer/imbalance triage as replay-time or bench tooling (diagnostics-plan C6 note); `test_min_output` + `rotation_calibration` procedure specimens (HA addendum) |
| **R1/R3/R5** | The legacy-measured reference table (HA addendum) — esp. kS ≈ 1.9 V evidence against HA-45's 1.0 V placeholder, and the reversed-port/5-motor-side port-map shape for adapter tests |
| **M4 brief-writer** | Master plan §14's "extend the `RobotCommands` queue" clause is counterfactual — build fresh (D6) |
| **C7 (immediate)** | §8's verdict; the CSV pointer; after deletion, broaden the CI PROS-free guard to all of `shulib/` per its in-file comment (the guard's scope note already anticipates this) |

---

## 7. Verification (actually run, outputs as observed)

No code changed, so the bar is "everything that was green is still green, and the guards/gate hold
on the unchanged tree":

```text
$ cmake --build build/test -j$(nproc) && ./build/test/shulib_tests
[doctest] test cases:    659 |    659 passed | 0 failed | 3 skipped
[doctest] assertions: 915570 | 915570 passed | 0 failed |
[doctest] Status: SUCCESS!
```
— identical to the pre-C6 baseline (659 / 915,570 / 3 deliberate skips), as expected for a
docs-only chunk.

```text
$ <ci.yml "No <pros/> in the core" grep, exact scope list>
core is PROS-free
$ <ci.yml "Core never includes the sim plant" grep>
layering holds: core is sim-free
```

```text
$ find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > all_headers.cpp
  (TU includes 102 headers)
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
    -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c all_headers.cpp -o /dev/null -Iinclude
ARM CROSS-COMPILE: CLEAN
```

`git status` shows only the C6 documentation files (this record, the progress log, the vocabulary
doc, and the three planning-doc edits); nothing committed, per the brief.

---

## 8. The verdict C7 acts on

> **`legacy/` — all 34 files under `include/legacy/` and `src/legacy/` — is SAFE TO DELETE,
> unconditionally, with one lodged follow-up that does not gate the deletion.**
>
> The evidence, in one place:
> 1. **Every file is classified** (§4: 18 + 16 = 34; zero unclassified) as superseded, salvaged,
>    or valueless — each with its reason and its v2 pointer.
> 2. **Nothing in v2 references `legacy/`**: no `#include` into it exists from `include/shulib/`
>    or `test/` (the PROS-free + layering guards and the 102-header ARM TU compile without it, §7),
>    and the host suite's 659 cases pass on a tree where `legacy/` contributes nothing.
> 3. **The irreplaceable content has already left the directory**: the command vocabulary and both
>    data-specimen characterizations (`legacy-command-vocabulary.md`), the hard-won constants and
>    calibration procedures (`hardware-assumptions.md` C6 section), and the diagnostic requirements
>    (`diagnostics-plan.md` C6 note) are in documents that survive deletion. The salvage is
>    **knowledge-complete and code-empty by audit** (§3), so there is no ported code whose
>    correctness could depend on the originals.
> 4. **Deletion actively improves the tree**: it removes 11 catalogued live bugs' source text, the
>    only copied third-party code (LemLib `pose.*`), and the ODR-trap duplicate declarations —
>    while git retains every byte at `357d3f2` and earlier.
> 5. **The one follow-up** — G4 lifting `autonomous_commands.csv` into its own fixtures via
>    `git show 357d3f2:src/legacy/autonomous_commands.csv` — is recorded in three places (§4 #19,
>    vocabulary doc §4, §6) and requires nothing from the working tree. It must not delay C7.
>
> C7 may `rm -rf include/legacy src/legacy`, rewire `main.cpp` onto the new core, and broaden the
> CI PROS-free guard to all of `shulib/`.

---

## 9. DoD checklist (brief §Definition of Done)

- [x] All 34 legacy files classified with reasons — §4 (18 + 16, count-checked)
- [x] The three roadmap salvage claims verified against source, not assumed — §2 (claims 1 and 2
      confirmed exactly; claim 3 resolved with the no-executor finding that reshapes it)
- [x] `legacy-command-vocabulary.md` complete and mechanically cross-checked — 7 distinct ids,
      4 sources, scripted counts (re-runnable, preserved in its §6), each id mapped or gap-flagged
      (2 named gaps: `NONE` semantics → G2; seat/settle wiggle → F2)
- [x] Hard-won constants captured — `hardware-assumptions.md` "Legacy-measured reference points"
      (7 rows, scoped as evidence/R-phase provenance per D2)
- [x] Anything ported has tests and is independent of `legacy/` — **vacuous: the port list is
      empty** (§1, §3), which the brief names as a fine outcome when the audit justifies it
- [x] Clear, explicit safe-to-delete statement — §8, evidence-backed, unconditional
- [x] Suite green (659 / 915,570 / 3 skips — unchanged baseline); both CI guards pass; ARM gate
      passes (102 headers) — §7
