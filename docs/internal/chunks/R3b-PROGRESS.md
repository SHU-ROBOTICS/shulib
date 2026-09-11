# R3b — live progress log

> Appended as the work happens (RESUMING.md's rule). If this file ends mid-thought, the pass
> was interrupted exactly there.

## Session 1 — 2026-08-19 — PIECE 3 ONLY (the absent-device ruling, brief §6)

**Scope declaration, first, so an interrupted log cannot over-claim:** this pass executes
**§6 only**. Pieces 1 (motor group) and 2 (IOdometry seam / drive-encoder odometry) have open
decisions gated on R3a Batch-1 measurements that have not come back (brief landmine 6: "If B1
has not come back, do §6 and stop"). They are **untouched** in this pass, deliberately. R3b as
a whole does NOT close here; the DoD items owned by pieces 1–2 (M1 badge, on-robot auton,
HA-18/HA-52/HA-112) remain open.

### 1. Baseline re-established (clean tree, 3a06075, fresh CMake configure)

- Host suite: **1151 cases / 1,523,871 assertions, 0 failed, 3 skipped** — matches the brief's
  §11 baseline runner's re-verification exactly.
- 148 headers in `include/shulib/`, 117 pages in `docs/api/` (counted below before edits).

### 2. Brief correction, verified before starting (do NOT hand-edit mkdocs nav)

Brief §9.2 item 3 says to add the new api pages to `mkdocs.yml`'s nav by hand, alphabetically.
**That instruction is wrong for the current tree**: `mkdocs.yml` carries
`# BEGIN GENERATED API NAV — regenerate with: python3 tools/api_doc_tool.py generate` …
`# END GENERATED API NAV` markers, and `docs/internal/docs-publishing.md` confirms the nav
between them is generated and byte-checked by `check-fresh`. Hand-editing inside generated
markers is the exact failure the brief itself bans for `PROJECT-BRIEFING.md`. So: run
`generate`, then VERIFY the new pages appear between the markers (a page absent from the nav
publishes unreachable with exit code 0 — the DOCS2 measurement).

### 3. Decisions taken this session (with the rejected alternatives)

**D1 — Naming: `AbsentGps` / `AbsentTagSource` / `AbsentVision`.**
Rejected: `NullGps`/`NullTagSource`/`NullVision` for consistency with `NullSink`. The
semantics differ: `NullSink` is a *working sink that discards*; these say *no such device is
installed on this robot*. `.gps = &absentGps` at the composition root reads as a statement
about the robot's configuration. Consistency loses to precision. (Recorded in the headers too.)

**D2 — Header count: THREE headers, one class each**, beside `hal/null_sink.hpp`:
`hal/absent_gps.hpp`, `hal/absent_tag_source.hpp`, `hal/absent_vision.hpp`.
- Rejected: one `hal/absent.hpp` for all three — hides the asymmetry between them (the GPS
  needs no wiring guard; the polled sources do — see D3) and lands below the one-page-per-header
  convention `docs/api/` is built on.
- Rejected: two headers mirroring `gps.hpp`/`vision.hpp` (AbsentTagSource+AbsentVision
  sharing). Defensible — the seams share a header by decision #7 — but the two classes carry
  DIFFERENT wiring rules with different consumers (tag corrector today, M4 vision poller
  later), and a shared header invites reading them as one rule. `null_sink.hpp`'s
  one-class-one-header precedent wins.
- Expected counts: 148 → 151 headers, 117 → 120 api pages.

**D3 — The §6.5 liveness-trap testability shape: a per-seam compile-time wiring rule, one
variable template per polled seam, living in the Absent class's own header.**

The problem: the brief rules "an absent source must not be polled" (a corrector polled over an
`AbsentTagSource` records "camera alive, no tags" about a robot with no camera — a false
diagnostic). But the only composition root, `src/main.cpp`, is in NO gate, so test 6 and
mutation 5 need a host-testable, MUTABLE library expression of the wiring rule.

Shape chosen (in `hal/absent_tag_source.hpp`):
```cpp
template <class TagSourceT>
    requires std::derived_from<TagSourceT, ITagSource>
inline constexpr bool kInstallTagCorrector = true;   // present sources: install and poll
template <>
inline constexpr bool kInstallTagCorrector<AbsentTagSource> = false;  // the §6.5 ruling
```
and its sibling `kInstallVisionPoller` in `hal/absent_vision.hpp` (IVision's consumer is M4's,
but the trap is identical in shape and pre-empting it costs two lines).
Verified before writing: constrained variable template + explicit specialization compiles
clean on host g++ AND arm-none-eabi-g++ (`-std=gnu++20 -Wall -Wextra -Werror`).

This diverges from the executor prompt's recommended shape (one generic
`kIsAbsentDevice<T>` trait + a separate composition helper), deliberately, for these reasons:
1. A single generic primary template needs ONE shared home. With three headers every placement
   forces a false include dependency between sibling headers, and a fourth trait-only header
   lands outside the expected 150/151 header envelope.
2. The composition root never needs the abstract taxonomy "is T absent"; it needs the decision
   "do I install/drive the poller for this source". Collapsing trait+helper into one per-seam
   rule constant IS the recommended "documented composition helper", minus a layer.
3. A generic `kIsAbsentDevice<AbsentGps>` would be true, yet NO wiring decision hangs on it:
   GpsCorrector is pull-based (reads state in propose(), no poll()), permanent no-fix is a
   seam-blessed mode, and test 1 proves installing over AbsentGps is bit-identity. A taxonomy
   bit with no consumer is dead weight; omitting the specialization instead would make the
   trait lie. The per-seam shape lets `absent_gps.hpp` carry NO rule constant and say WHY.
4. Mutation 5 stays exactly as strong: flip `kInstallTagCorrector<AbsentTagSource>` to true →
   test 6 red (checked below when run).
- Also rejected: an overload set `isAbsentDevice(const ITagSource&)`/`(const AbsentTagSource&)`
  — a V5 adapter implementing BOTH ITagSource and IVision (decision #7 expects exactly that)
  makes the unqualified call ambiguous.
- Also rejected: RTTI/dynamic_cast (may be off in the ARM/PROS build; runtime cost for a
  construction-time fact) and any change to the F4-locked seams (`isPresent()` — brief §6.4,
  settled).
Limitation, documented in the header: the rule answers from the DECLARED type. Ask it where
the device member is declared (the composition root does exactly that); through an
`ITagSource*` it answers for the seam, not the device.

**D4 — `AbsentGps::rmsError()` returns `units::Length{1.0e6}` (inches), a named constant.**
- Rejected: `infinity()` — violates IGps's stated finite contract and trips the tree's finite
  guards.
- Rejected: `std::numeric_limits<double>::max()` — finite in name only: the first downstream
  multiply or square (σ_meas = rmsTrustFactor·rms, confidence uses σ²) overflows it to inf,
  manufacturing the exact non-finite the contract bans, one arithmetic step later.
- 1.0e6 inches ≈ 16 miles: astronomically larger than any field (a VEX field diagonal is
  ~204 in), so no plausible gate trusts it, yet (2·1e6)² ≈ 4e12 stays comfortably inside
  double range through every downstream formula in GpsCorrector.

**D5 — Test 1 raises `GpsCorrectorConfig::maxReportedRms` above the D4 constant.**
With the default (6.0 in), a mutated `hasFix()==true` would be declined as
RejectedSensorQuality and the identity test would stay GREEN under mutation 1 — the mutation
table's required red would be fake. Raising the gate makes the test PROVE identity comes from
`hasFix()` alone, not from a downstream quality gate happening to save us — strictly stronger,
and it is what makes mutation 1 actually bite (one fold of a tiny-confidence fix moves the
fused pose by ~1e-11 in, and the test compares BITS, not Approx).

### 4. Work log (appended as it happens)

- 2026-08-19: headers written (`hal/absent_gps.hpp`, `hal/absent_tag_source.hpp`,
  `hal/absent_vision.hpp`), smoke-compiled clean on host g++ AND arm-none-eabi-g++ at the CI
  flag set. Session interrupted by an infrastructure error right here; resumed on verified
  state (headers + this log intact and untracked; nothing committed).
- **Landmine 1 NARROWED (verified by the coordinator, adopted here): `src/main.cpp` IS
  compile-checkable.** The on-robot blocker is LINK-time (soft-float firmware/liblvgl.a), so
  an `arm-none-eabi-g++ -c` of `src/main.cpp` (hard-float, gnu++20, `-Wall -Wextra
  -Wformat=2`) is a real gate for the R3b main.cpp edit. Baseline at HEAD: exit 0 with
  exactly three `-Wunused-function` warnings (`robot()`, `portMapString()`, `shaped()`) on
  the bench build; exit 0 clean with `-DSHULIB_ROBOT_XDRIVE_INVENTED`; `src/bench_r3a.cpp`
  clean in both variants. Because `robot()` is defined, the `Robot` struct IS instantiated,
  so the RobotContext rewiring is genuinely compile-checked. Run all four after the edit.
- D3 (per-seam rule constants), D4 (the 1e6 constant), D5 (raised maxReportedRms so mutation
  1 bites) reviewed and ACCEPTED by the coordinator; D5's standard applies to all five
  mutation checks: a mutation that goes red for a reason other than the one its test is
  meant to catch is a fake red, and gets said so.
- Writing `test/absent_device_test.cpp` next (both headers already cite this filename).

#### 4.1 Tests written and green

- `test/absent_device_test.cpp` (new, 5 cases): bit-identity A/B rig, the IGps invariant
  sweep, the corrector no-op WITH A MAPPED-TAG AMBUSH (the map deliberately contains the one
  tag a fabricated observation would resolve against, placed so such a fix would be ACCEPTED —
  without it, mutation 2 would bounce off the unmapped-id gate and stay green for the wrong
  reason), the byte-indistinguishability demonstration (the evidence §6.5's ruling is
  necessary), and the wiring-rule test over a composition harness that FOLLOWS the library
  rule rather than re-implementing it.
- `test/robot_context_test.cpp` extended, not replaced: the fake-default assertions and all
  three original cases untouched; the null-rejection case gains gps/tags/vision (gps had no
  null check at all before this); one new case constructs the fully-absent context.
- Suite: **1151 → 1157 cases, 1,523,871 → 1,538,107 assertions, 0 failed, 3 skipped.**
  (Counts taken on the dirty working tree; the build-hash-derived assertions shift a little
  on commit — R3a-PROGRESS §12.2's known mechanism.)
- Two rig subtleties recorded because they are load-bearing for the mutations, not style:
  the identity test boots through a WITNESSED not-ready IMU phase so the correctors are first
  consulted on a settled, healthy-dt tick (consulted at tick 1 instead, dt = 0 makes the
  fusion clamp spend the constant-sample AbsentGps's one-and-only fold on a zero nudge), and
  it setPose()s a 5-inch offset INSIDE the fusion policy's 12-inch innovation gate so a
  mutated fix is accepted, not conveniently rejected.

#### 4.2 The five mutation checks — each run, red OBSERVED, restored, suite re-verified green

| # | Mutation | Observed red |
|---|---|---|
| 1 | `AbsentGps::hasFix()` → `true` | **Test 1 (bit-identity): 609 failed** — x/y bits diverge and `isDeadReckoning` flips from the first settled tick (the fold really happened; the widened `maxReportedRms` proved D5's point). Test 2 adds 2000 (`hasFix()==false` sweep). 2609 failed across the two cases. |
| 2 | `AbsentTagSource::tags()` → one fabricated tag (id 7, (24,0,π), conf 1.0) | **Test 3 (corrector no-op): 503 of 505 failed** — the fabricated tag was ACCEPTED against the ambush map (`p.valid` red on essentially every tick, plus the counter checks). |
| 3 | `RobotContext` gps precondition dropped (same-line no-op replacement) | **Test 4 (omission): 1 failed** — `CHECK_THROWS_AS((RobotContext{nullGps}), PreconditionError)` "did NOT throw at all". |
| 4 | `AbsentGps::rmsError()` → `-1.0` | **Test 2 (invariant sweep): 2001 failed** — 2000 × `rmsError() >= 0` + the exact-constant check. Honest note: test 1 stayed GREEN under this mutation, correctly — the no-fix path never reads rmsError; the table's required red is the sweep, and it fired. |
| 5 | `kInstallTagCorrector<AbsentTagSource>` → `true` | **Test 6 (wiring rule): 3 failed** — the cited rule itself, plus both `CHECK_FALSE(noCamera.corrector.has_value())`: the harness, following the flipped LIBRARY answer, installed and drove a corrector over the absent source. The false-liveness wiring is exactly what went red. |

**A near-miss worth recording (the D5 standard catching a fake GREEN, not a fake red):** the
first mutation-1 run reported all-green — from a STALE binary. The build had failed at the
briefing gate (PROJECT-BRIEFING regenerated earlier against the pre-test binary's counts), so
the mutated TU never linked, and the doc-gate failure was invisible to a case-sensitive grep
for "error". Protocol fixed and used for all five: check the build's exit status, keep a
saved green binary so the gate (which runs `build/test/shulib_tests`) never sees a red one
during the restore build, and regenerate docs/api transiently when a mutation shifts doc
line anchors (mutation 2) or a documented initializer (mutation 5) so the observed red is
the TEST's, never a doc gate's. A mutation "verified" without checking that the mutated
binary actually built is worth nothing — this almost happened here.

#### 4.3 src/main.cpp — the composition root, compile-checked on ARM

- Swapped `FakeTagSource`/`FakeVision` → `AbsentTagSource`/`AbsentVision`; both `hal/fake/`
  includes removed (`grep hal/fake src/main.cpp` → 0). The competition binary links no test
  scaffolding.
- The lines 44-45 banner ("Still fake, deliberately") replaced with the absent-device ruling;
  the top-of-file "fakes are gone (vision/tags excepted)" parenthetical corrected too.
- Boot announcement added (§6.4, T5 precedent): one Info line in initialize()'s wired branch —
  "absent devices (deliberate): tags=ABSENT vision=ABSENT …" — the composition root saying
  the degradation out loud, once.
- **GPS DECISION (recorded, per the §6.6 "add an AbsentGps" instruction):** the `Robot`
  struct KEEPS its `ProsGps`. That struct is the preserved INVENTED X-drive graph whose
  HA-111 port map defines a GPS on port 9 (and `portMapString()` prints GPS9 — swapping in an
  AbsentGps would make the two lie about each other), and it is the tree's one live example
  of wiring the real GPS adapter. The BENCH robot has no GPS — but the bench path constructs
  NO RobotContext at all today (initialize() returns early into the read-only R3a bench), and
  building the bench context is exactly pieces 1–2's deliverable (motor group + odometry
  seam, both B1-gated). THAT wiring will spell `.gps = &absentGps`; a comment at the absent
  members says so. So no AbsentGps is wired in main.cpp in this pass, deliberately — wiring
  one into the X-drive graph would have been change for the checkbox, not for the robot.
- Compile-checked (the narrowed landmine 1): `arm-none-eabi-g++ -c` of `src/main.cpp` and
  `src/bench_r3a.cpp`, BOTH variants (bench default and `-DSHULIB_ROBOT_XDRIVE_INVENTED`),
  all exit 0. Warning profile IDENTICAL to HEAD (verified by compiling `git show
  HEAD:src/main.cpp` with the same command): the three baselined `-Wunused-function`s
  (`robot()`, `portMapString()`, `shaped()`) plus one pre-existing third-party
  `liblvgl/core/lv_obj_style.h` `-Wdeprecated-enum-enum-conversion`, present at HEAD and in
  every variant, unrelated to this edit. Zero new warnings. NOT verified: linking and
  running on the brain (the known soft-float firmware blocker, unchanged).

#### 4.4 Documentation pass

- `docs/api/` regenerated: **117 → 120 files** (three new pages: `absent_gps.md`,
  `absent_tag_source.md`, `absent_vision.md`; the parser handled the wiring-rule variable
  templates, giving the specialization its own anchored entry). The generated mkdocs nav
  picked all three up between the markers (verified by grep: `api/absent_*.md` at nav lines
  231–233) — **the §2 brief correction stands: no hand-edit inside the markers.**
- Entities **1,625 → 1,645** across doc-tool headers **115 → 118**. Ungated hand-updates
  done: `docs/README.md` ("All 1,645 of them"), `docs/internal/docs-publishing.md` (118 header
  pages + index + README = 120 files, 1,645 entities — the old sentence's counting was also
  off by construction and is now exact).
- `roadmap.md`: "You are here" gained the R3b piece-3 paragraph (explicitly `[~]`, pieces 1–2
  named open, M1 badge explicitly NOT flipped); the third "What R3b must BUILD" checkbox
  flipped to `[x]` with file/test/count evidence; the live pointer's DOCS2 entity count
  annotated (1,625-then / 1,645-now); the piece-1 bullet's "real 7-motor drive" softened to
  "then-7-motor" with the 8-motor repair noted (measurement text itself untouched).
- `build-order.md`: the "Next:" status block now records R3b `[~]` in flight, §6 landed
  2026-08-19, pieces 1–2 still B1-gated, M1 not flipped. **The brief's §9.5 "fix the stale
  7-motor/five-dead sentence" instruction was ALREADY DONE in the tree** (a "Corrected
  2026-08-18" parenthetical, build-order.md ~line 1589) — nothing to fix; noted rather than
  re-edited.
- `docs/hardware-assumptions.md`: **untouched, deliberately.** Piece 3 settles no HA entry by
  measurement — HA-18/HA-52/HA-112 belong to pieces 1–2 and the bench. No register edit on
  inference.
- `docs/guide/14-what-it-cannot-do-yet.md`: **untouched** — "the library has never driven a
  robot" is still true after piece 3.
- Historical evidence blocks carrying old counts (roadmap DOCS2/E-entries, build-order DOCS2
  entry, changelog 2.1, PROJECT-BRIEFING durable prose) left as written — they record what was
  true when written (the append-never-rewrite convention).

### 5. Final verification (all from the repo root, on the finished working tree)

| Check | Result |
|---|---|
| Full host build (all doc gates in-build) | exit 0 |
| Host suite | **1157 cases / 1,538,107 assertions, 0 failed, 3 skipped** |
| `api_doc_tool.py` self-test / check-coverage / check-fresh / check-examples / check-removability | **5 × PASS** (coverage: 1,645 entities / 118 headers, all documented) |
| `briefing_status.py check` | **PASS** |
| `doc_staleness_audit.py` self-test + audit | **2 × PASS** (0 numeric claims in scope — the intended end state) |
| PROS-free guard / sim-layering guard | both print nothing — **CLEAN** |
| ARM cross-compile gate | **CLEAN, 151 headers** |
| `src/main.cpp` + `src/bench_r3a.cpp` ARM compile (both variants) | exit 0, warning profile identical to HEAD |

### 6. WHAT IS NOT DONE — read this before believing any checkbox

- **Piece 1 (multi-motor-per-side aggregation): NOT STARTED, deliberately.** Gated on R3a
  Batch 1 (per-side gear ratio B1.2, the 8-motor probe re-run). Landmine 6: "If B1 has not
  come back, do §6 and stop." B1 has not come back.
- **Piece 2 (`IOdometry` seam + drive-encoder odometry): NOT STARTED, deliberately.** Gated
  on the same batch (IMU sign convention group, loop rate) and on the §8.5.1 parts-bin
  question (the two never-wired rotation sensors).
- **R3b's DoD is therefore OPEN**: M1's badge unflipped (no V5 number-match exists), no auton
  has run under library steering, HA-18 / HA-52 / HA-112 unsettled (they belong to pieces
  1–2 and the bench), and `kinematics/tank.hpp:80`'s "HAL's business" facility still does
  not exist.
- **`src/main.cpp` is compile-verified on ARM but never LINKED or RUN** — the soft-float
  firmware blocker is unchanged; the boot announcement line has never printed on a brain.
- **The library has still never driven a robot.** `docs/guide/14` remains true and untouched.
- Nothing committed, nothing staged — the working tree carries: 3 new headers, 1 new test
  file, robot_context_test extended, main.cpp rewired, 3 new api pages + regenerated
  README/all-entities/mkdocs-nav/PROJECT-BRIEFING, roadmap + build-order + docs/README +
  docs-publishing updated, and this log.

### 6. Correction, appended (not rewritten): the assertion counts above are DIRTY-tree counts

Verifier's note, 2026-08-19, after the commit made the tree clean. The counts recorded in §4
and §5 above (**1,538,107**) were measured with modified tracked files present, so
`git describe --always --dirty` baked `-dirty` into `SHULIB_BUILD_HASH` — and a test asserts
through that string, which is exactly 6 characters longer. Reconfigured on the committed clean
tree (`v0.1.1-271-geab411b`): **1157 cases / 1,538,101 assertions, 0 failed, 3 skipped.**

That is the number a fresh clone reproduces, so it is the one `docs/roadmap.md` and the
generated `PROJECT-BRIEFING.md` block now carry. R3a-PROGRESS.md §12.2 measured this trap and
states the rule — *"the committed briefing must carry the CLEAN-tree number"* — and it caught
this pass anyway: the first commit shipped the dirty number to both, and it was fixed by
amend. The delta is 6 assertions, in the same direction, for the same reason, a second time.

The measurements above are left as written, per §9.4's append-never-rewrite convention: they
are what was true when taken. This section supersedes them.

---

## Session 2 — 2026-09-10 — PART 0 ONLY (the tester learns the tank chassis + DRIVE)

**Scope declaration, first, so an interrupted log cannot over-claim:** this pass executes
**Part 0 of [`R3b-session2-tank-chassis.md`](R3b-session2-tank-chassis.md) §3 only** — the
per-variant chassis table, MOTOR WATCH generalized with a sign capture, the DRIVE station behind
six gates, the pure stick mapping extracted to a PROS-free header (host-tested, brief test 14),
the `ROBOT=tank` build variant under the src gate (brief test 15), and worksheet Station D.
**Parts 1–3 (`MotorGroup`, `IOdometry`/`DriveEncoderOdometry`, the tank composition root) are
NOT started, deliberately** — the build team is waiting on Part 0 to upload; the rest runs as a
separate pass after the coordinator verifies and commits this one. Nothing here closes R3b, moves
M1's badge, or claims the library drove a robot: the DRIVE station drives the robot through the
library's *adapters*, not its motion stack, and is labelled that way everywhere it appears.
**The tank chassis table ships UNSET** — no port, sign, cartridge or width is invented for
robot two (brief landmine 1). Nothing is committed or staged by this pass.

### 1. Baseline re-established (clean tree, `d92a9fe`, existing `build/test` configure)

- `git describe --always --dirty --abbrev=7` = `v0.1.1-274-gd92a9fe` (clean).
- Host suite: **1157 cases / 1,538,101 assertions, 0 failed, 3 skipped** — matches brief §11
  exactly (and the 101 confirms the on-disk binary carries a clean-tree hash).
- `include/shulib/*.hpp`: **151**; `docs/api/`: **120** files.
- Required reading done in the prompt's order (RESUMING, the Session 2 brief, the original brief
  §9/§10, this log's Session 1, GATE1-PROGRESS §2/§4/§5, the tester end to end, main.cpp, the
  Makefile, the gate tool, ci.yml, the HAL headers, the worksheet).

### 2. Brief checks — what the reading found before writing (things the brief did not say)

- **Test/binary staleness mechanism confirmed in the build files:** `test/CMakeLists.txt`
  `add_dependencies(shulib_tests shulib_doc_gates)` — the doc gates (including
  `briefing_status.py check`, which RUNS the on-disk `build/test/shulib_tests` for its suite
  counts) execute BEFORE the test binary links. A red or count-shifted binary on disk therefore
  fails the gate on the NEXT build and leaves the old binary in place. Mutation protocol for this
  pass (the §4.2 standard, made mechanical): md5 the binary before each build, require build exit
  0 AND a changed md5 before believing any run, and copy a saved green binary over the on-disk one
  before every restore build so the gate never sees red. Script in the session scratchpad.
- **A body-only mutation cannot stale `docs/api/`:** the generated pages carry each entity's
  signature and DECLARING LINE, never the body (`docs/api/controller_conversion.md` inspected). So
  every mutation below is a single-line in-place body edit — no line-count change, no documented
  initializer touched — and any red is the test's, never a doc gate's.
- **Gate 5's premise is documented by the vendored SDK, not assumed:** `include/pros/motors.hpp:51-52`
  — "A reversed motor will reverse the input or output movement functions and movement related
  telemetry in order to produce consistant behavior with non-reversed motors". So after
  construction with a NEGATIVE port, a correctly-signed member reports a velocity of the same sign
  as its command. The DRIVE run measures this (HA-94 onward); the cut-out message says how to read
  the outcome if it turns out false (see D10).
- **Alternating signs inside a five-motor side are the NORMAL pattern for a coupled gear train**,
  not a fault: `docs/hardware-assumptions.md:1868` records last season's real map as
  `L {11,−12,13,−14,−15} / R {16,−17,18,−19,20}`. MOTOR WATCH will show DOWN readings within a
  side on a forward push; DRIVE negates exactly those ports. The worksheet says so.
- **Extracting `shaped()` from `src/main.cpp` changes a documented count:** the Makefile's
  WARNFLAGS comment and the gate tool's header both say THREE `-Wunused-function` warnings
  (`robot()`, `portMapString()`, `shaped()`) are the bench build's evidence of dead wiring. After
  the extraction there are TWO. The gate's assertion is `>= 1` so it holds; both comments are
  updated in this pass so the documented count stays true.
- **`api_doc_tool.py` needs no edit for a new directory:** `grouped_pages()` appends an unknown
  subsystem alphabetically with a title-cased label — so `include/shulib/teleop/` publishes as
  "Teleop" without touching the tool (verified in `SUBSYSTEM_LABELS`' comment and code).
- **The brief §3.4 says "extend the self-test so a non-landing define is caught for `tank` too"**
  — but the CURRENT self-test has NO Makefile-mutation case at all (GATE1's mutations 5a/5b were
  run by hand, GATE1-PROGRESS §7). So the "too" is loose; this pass adds the case for tank AND for
  xdrive as permanent self-test cases (D7), which is what the sentence must have meant.

### 3. Decisions taken up front (alternatives recorded as they are decided)

**D1 — The mapping header lives at `include/shulib/teleop/stick_mapping.hpp`, namespace
`shulib::teleop`.** "Teleop" is T2's own vocabulary (build-order Phase T "Driver control";
register row F13's "T2's teleop layer"), and the header is documented as the seam T2 owns.
- Rejected: `hal/controller_mapping.hpp` — `hal/controller_conversion.hpp:16-17` itself rules
  that deadband is driver-feel POLICY belonging to the teleop layer, never to a conversion; putting
  policy in `hal/` would contradict the header beside it.
- Rejected: `chassis/teleop.hpp` — `chassis/` is the frozen F6 facade's home; a stick mapping is
  a client of `Chassis::drive`, not part of the facade.
- Rejected: `motion/` — the primitives layer; a mapping is not a motion.

**D2 — Three pure layers plus one combined call**: `deadbanded(axis)` → `mapSticks(StickInput)`
→ `DriveRequest{forward,left,yawCcw}` (dimensionless, signs applied, zero when disconnected) →
`toChassisSpeeds(request, maxLinear, maxAngular)`; and `mapSticksToChassisSpeeds(...)` for the
teleop loop. The DRIVE station consumes the MIDDLE layer (it needs volts, not in/s).
- Rejected: one function returning `ChassisSpeeds` only — the tester would have to pass unit
  budgets (1 in/s, 1 rad/s) to recover fractions, coupling volts to in/s through a hack.
- Rejected: a stateful class — the mapping is pure today; slew (stateful) is T2's change, and a
  class now would pre-shape T2's design for no consumer.

**D3 — Bit-identity is PINNED by an oracle, not asserted:** `test/stick_mapping_test.cpp`
carries the pre-change `src/main.cpp` code (`shaped()` + the `ChassisSpeeds` construction)
verbatim as a local function and compares BITS (`std::bit_cast<std::uint64_t>`) across a sweep
including ±0.05, ±0.0499…, −0.0, the rails, and connected/disconnected. "No behaviour change"
is then a measurement (brief §6.2 "nothing about the mapping changes").

**D4 — The tank variant's define is `SHULIB_ROBOT_TANK_2026`.**
- Rejected: `SHULIB_ROBOT_TANK` — the bench bot is ALSO a tank, and `main.cpp` already carries
  `SHULIB_BENCH_TANK`; two "TANK" macros for two different robots is the mirror-in-plain-sight
  class of error. The year names the robot (the brief's own "Robot 2 — the 2026 tank chassis").
- Rejected: `SHULIB_ROBOT_TANK_UNSET` — a status in a name goes stale the day the table is
  filled.

**D5 — `main.cpp`'s internal selector is renamed `SHULIB_BENCH_TANK` → `SHULIB_RUNS_BENCH_TESTER`**
(it now covers two robots: bench and tank both run the tester in Part 0), and a `#error` fires if
both variant defines are set at once (a `make` invocation can only set one, but a stray
`EXTRA_CXXFLAGS` could set the other — fail loudly, GATE1's whole lesson).

**D6 — The gate gains two things beyond the brief's minimum, both for tank's specific blind
spot.** The brief accepts that tank's behavioural detector is the same as bench's (≥1
`-Wunused-function` from `main.cpp`) — which means the warning-count detector CANNOT tell a tank
build from a bench build. GATE1's mutation 5b (a look-alike define that defeats a substring check)
was caught for xdrive ONLY by that warning-count detector; for tank it would be invisible to both.
So: (a) the structural define check is TOKEN-EXACT (the compile line is split on whitespace and
the define must be a whole token; another variant's define must be absent as a token) — this
closes the 5b class structurally for every variant; (b) a **variant identity beacon**: `main.cpp`
defines one string per variant (`shulib-robot-variant=bench|xdrive|tank`) under the same `#if`
chain that selects the wiring, prints it at boot so it is referenced, and the gate asserts the
expected beacon IS in `bin/hot.package.elf` and the other two are NOT — the end-to-end proof that
the `#if` in the SOURCE actually took the branch (a renamed macro is the case no command-line check
can see, and applying the bench table to robot two is exactly HA-111's defect class).
- Rejected: accepting the blind spot as the brief's minimum — "an instruction that silently does
  nothing" is GATE1's defect class, and it is the one class this variant is most exposed to.
- Rejected: `#pragma message` as the beacon — emits a `note:` (a new parser path) and proves the
  compile, not the LINKED package; the ELF-bytes assertion already exists for the hash and is the
  measurement GATE1 itself used.

**D7 — Self-test gains permanent Makefile-mutation cases** (a replace-plant with byte-exact
restore, beside the existing append-plant): tank append DROPPED → structural red; tank
LOOK-ALIKE define → structural (token) red AND beacon red; xdrive append dropped → red. The
brief's test 15 mutation is the first of these, run through `check` by hand as well (§6).

**D8 — Chassis table shape:** one `constexpr ChassisTable kChassis` selected by a single
`#if defined(SHULIB_ROBOT_TANK_2026)` — the "one obvious place" the brief asks for — with
fixed-capacity per-side arrays + counts (0 = unset), a tester-local `Cartridge{Unset,Red,Green,Blue}`
(so an UNKNOWN cartridge is never spelled with a placeholder `MotorGearset` value), IMU port 0 =
unset, a `measured` flag and a provenance string. `describeMissing()` names every unset field on
screen. The bench table is the measured one (L 15–18, R 11–14, blue, IMU 4; R3a-PROGRESS §15–§19).
- Rejected: two arrays per variant behind `#if` (the old shape) — nothing can name what is unset.

**D9 — MOTOR WATCH capture:** a per-port VERDICT from the last run (+1/−1/0) and, separately, the
SIGNS DRIVE consumes, written ONLY by a qualifying whole-robot push (every table port moved — that
is the moved-count == table-count rule of brief §3.3 gate 1 stated per port). After a qualifying
push the panel asks **"WHICH WAY DID YOU PUSH IT? FRONT-FIRST / BACK-FIRST"** and a back-first
push inverts the capture — the same lesson the IMU test in this file learned on 2026-08-19 ("record
the action instead of assuming it"; a wrong front inverts every sign and gate 5 cannot see it
because the members still agree with each other).
- Rejected: overwriting the sign capture on every run — a single-wheel spin after a good push
  would revoke DRIVE for no reason.
- Rejected: assuming the push was forward — the file's own history says operators do the
  opposite of the instruction about half the time.

**D10 — DRIVE station specifics (all constants INVENTED and labelled so on screen):** dead-man
= **L1** (left index finger; thumbs never leave the sticks; physically on the top edge, not near
the sticks); ceiling step = **R1** via `hal::ButtonEdge` (its first hardware use); ceiling 3 → 6 →
9 → 12 V and STOPS at 12 (re-enter for 3 V, which also clears a cut). Gate 5 references: a member
"differs" when its velocity sign is opposite the SIDE'S COMMAND sign (the unambiguous expected
direction), "near zero" when |v| < 0.25 × the side's max |v|, evaluated only while |command| >
1 V and max |v| > 1 rad/s, and CUT only after **250 ms of persistence** — the same window as the
2.4 A over-current cut — so a stick reversal (members coasting the old way while the new command
is opposite) does not false-cut. A WHOLE side disagreeing is reported as "side runs OPPOSITE to
command — the push was backwards or the front is wrong", still a cut. Current and temperature per
port live; controller LCD carries ceiling + dead-man button + cut state, written only on change.
- Rejected: majority-of-mates as the reference — with a whole side inverted it reports nothing.
- Rejected: an instant cut — measured stick reversals would trip it on a healthy drivetrain.

**D11 — Adapter storage and the exit guard:** `std::optional<hal::pros::ProsMotor>` (already
ARM-gated via `motion/motion_scheduler.hpp`) constructed in place inside a `try`, and an RAII
all-stop guard (0 V + Coast on every constructed member) so gate 6 holds on EVERY exit path —
touch, cut, controller loss, precondition throw — not just the happy one.

### 4. Work log (appended as it happens)

- **FACTS RECEIVED from the coordinator mid-pass (2026-09-10, relayed from the build team,
  "read off the robot, not believed"):**
  1. Drive cartridges are **BLUE** — read off a motor. Source: "build team, read off a motor
     2026-09-10". **Applied:** the tank table's cartridge is SET to Blue with that provenance
     string (a reported measurement is allowed to be set; an invention is not — brief landmine 1).
  2. Drive wheels are **2.75 in**. **Not applied anywhere:** the tester uses no wheel diameter
     (MOTOR WATCH reads degrees; DRIVE commands volts). Recorded here for Part 2 / R3d.
  3. The drive is quoted as **"600 rpm"**, which reads as wheel speed, i.e. **direct drive 1:1 —
     UNCONFIRMED** until tooth counts arrive. **Not applied anywhere:** the tester uses no ratio.
     Recorded here, marked unconfirmed; Part 2's plant/geometry question (brief §5.3) waits on it.
  - **Every port stays UNSET.** DRIVE still refuses, naming "LEFT ports, RIGHT ports" as missing
    (and "IMU port" for the IMU station). The stop-after-Part-0 instruction stands.

#### 4.1 The mapping header + test 14 — written, green, and the doc gates' two catches

- `include/shulib/teleop/stick_mapping.hpp` written (D1/D2): `kStickDeadband`, `StickInput`,
  `DriveRequest`, `deadbanded()`, `mapSticks()`, `toChassisSpeeds()`, `mapSticksToChassisSpeeds()`
  — all `constexpr`, PROS-free, every public member `///`-documented. `docs/api/` regenerated:
  **120 → 121 pages** (`stick_mapping.md`), nav spliced between the markers (`mkdocs.yml:283`
  "Stick mapping"), coverage now **1,659 entities / 119 headers** (was 1,645 / 118).
- `test/stick_mapping_test.cpp` written (D3): the ORACLE sweep (the pre-change `main.cpp`
  code verbatim, 51³ axis triples × 3 budget configs × connected/disconnected = 795,906
  bit comparisons, one CHECK per divergence plus a scale assertion), the axis-sign case, the
  axis-ASSIGNMENT case (a swap keeps every single-axis sign right — needed its own case), the
  strict-interior deadband case (threshold passes; +0.0 inside the band), the disconnected case
  (incl. a negative budget, the only input that separates `ChassisSpeeds{}` from `0·budget`),
  and the one-call-equals-composition pin (so the loop's layer and the DRIVE station's layer
  cannot drift). 6 cases, 349 assertions when green.
- **Doc-gate catch 1 (removability):** the first build FAILED at `check-removability` — the
  header banner cited "build-order T2", and a header's banner is reproduced on its public api
  page. Reworded on the same line to "chunk T2" (no line shift), regenerated, passed. Recorded
  because it is exactly the C7 rule working as designed: the gate saw a public→internal link
  before a reviewer could.
- **Doc-gate catch 2 (the briefing's derived counts):** the second build FAILED at
  `briefing_status.py check` — headers 151 → 152 is derived from the tree. Regenerated before the
  build could proceed; the same dance recurs whenever the suite counts move, and the final
  regeneration is recorded in §5. Also observed and explained: the UNCHANGED baseline binary
  reported 1,538,104 (not 101) once the new api page existed — `api_reference_fidelity_test`
  reads `docs/api/` at RUN time, so 3 assertions are tree-dependent, not a stale-binary sign.
- First green build with the new test: **1163 cases / 1,538,459 assertions / 0 failed / 3
  skipped** = 1,538,101 + 6 (`-dirty`, the CONFIGURE_DEPENDS glob reconfigured on the new test
  file: `shulib build hash: v0.1.1-274-gd92a9fe-dirty`) + 3 (fidelity test, new page) + 349
  (this file). New test cases: 1157 → 1163 (+6). The **clean-tree expectation is therefore
  1,538,453** (dirty − 6); §5 says how the briefing handles that.
- A compile error in the test (a mixed `const`/non-`const` pointer initializer list) was caught
  by the protocol script as `build exit=2, binary-bytes-changed=NO` — the stale binary was never
  believed. Fixed with an explicit array.
- A design correction before the campaign: the first draft pinned the three axis signs with
  `static_assert`s, which would turn a sign-flip mutation into a COMPILE failure and hide every
  runtime red. The `static_assert`s now pin only constexpr-ness on inputs no mutation changes;
  the signs are pinned by runtime CHECKs so the observed red carries its inputs.

#### 4.2 Test 14 mutation campaign — each RUN (build exit + binary md5 checked), red OBSERVED verbatim, restored byte-exact, green re-verified

Protocol per mutation (scratchpad `mutcycle.sh`): `sed` the header and PROVE the file changed
(`cmp`), build (`cmake --build`), require exit 0 AND a changed binary md5, run, record the distinct
failing assertion sites with counts, restore the header from the saved copy (`cmp` byte-exact),
copy the saved green binary over the on-disk one so the doc gate never sees red, rebuild, run.

| # | Mutation (as applied, one line, body only) | Built? | Observed RED |
|---|---|---|---|
| M1 | `deadbanded(in.leftY)` → `deadbanded(-in.leftY)` (forward sign) | exit 0, md5 `5c95cbea…` | **4 cases, 351,141 failed**: oracle `:138` × 351,135; sign case `:156`, `:171`; assignment `:176`, `:182`; composition `:248`… — every connected combination with |leftY| ≥ 0.05 diverged |
| M2 | `deadbanded(-in.leftX)` → `deadbanded(in.leftX)` (left sign) | exit 0, md5 `ad7e3f88…` | **3 cases, 351,140 failed**: oracle × 351,135; `:161`, `:163` (LEFT = +left), `:177`, `:183`, `:197` |
| M3 | `deadbanded(-in.rightX)` → `deadbanded(in.rightX)` (yaw sign) | exit 0, md5 `d3127bd6…` | **3 cases, 351,140 failed**: oracle × 351,135; `:166`, `:168` (RIGHT = −yaw), `:178`, `:184`, `:194` |
| M4 | `axis < kStickDeadband` → `axis <= kStickDeadband` (boundary) | exit 0, md5 `c370ed0e…` | **2 cases, 22,954 failed**: oracle × 22,953 (exactly the combinations with an axis AT +0.05 after negation) + `:214` `deadbanded(0.05) == 0.05` |
| M5 | `mapSticks`: `if (!in.connected)` → `if (false && !in.connected)` (first occurrence only) | exit 0, md5 `2b6d7e99…` | **1 case, 3 failed**: `:237`–`:239` — the three `mapSticks` disconnected bit-checks. The oracle stayed GREEN, correctly: the one-call form keeps its own guard, so the red is precisely the layer the DRIVE station consumes and nothing collateral |
| M6 | one-call form: `return math::ChassisSpeeds{}` → `return toChassisSpeeds(mapSticks(in), …)` (0·budget when disconnected) | exit 0, md5 `70d1d2f9…` | **2 cases, 132,652 failed**: oracle × 132,651 = 51³ — the ENTIRE disconnected sweep under the negative-budget config (−0.0 ≠ +0.0 in bits) + `:244` the `neg` check. Exactly the red the negative budget was put in the sweep to produce |
| M7 | `DriveRequest{deadbanded(-in.leftX), deadbanded(in.leftY), …}` (left stick cross-wired) | see the entry appended below | see below |

After every restore: **1163 / 1,538,459 / 0 failed** re-observed (the restore build's md5 is
`86cdc8c4…` each time — the same bytes, a deterministic rebuild of the restored source).
| M7 (appended) | left stick cross-wired: `DriveRequest{deadbanded(-in.leftX), deadbanded(in.leftY), …}` | exit 0, md5 `cb8ac303…` | **3 cases, 391,232 failed**: oracle × 391,221; sign `:156`, `:157`, `:161`, `:163`, `:171`; assignment `:176`, `:177`, `:196`, `:197`, `:200`, `:201` — the assignment case fires on all three of its single-axis probes, which is what it exists for |

**Test 14 campaign complete: 7 mutations, 7 built (exit 0, new md5 each), 7 reds observed, 7
byte-exact restores, 7 greens re-verified. No fake reds** (every failing site is inside
`test/stick_mapping_test.cpp`; no doc gate, no other TU). Header `cmp`-identical to the saved
original after the last restore.

#### 4.3 `src/main.cpp`, the Makefile, the gate tool — the third variant, built and gated

- **`src/main.cpp`:** the `#if` chain gained `SHULIB_ROBOT_TANK_2026` (D4) with an `#error` if
  both variant defines are set (D5); the internal selector is now `SHULIB_RUNS_BENCH_TESTER`
  (bench AND tank boot the tester in Part 0); `kVariantBeacon` per variant (D6), printed in
  `initialize()` so it is linked in; `kTeleopDeadband` and `shaped()` removed and the R1a
  loop calls `shulib::teleop::mapSticksToChassisSpeeds()` — with the axes read ONLY while
  connected, the exact call pattern of the old loop, so device traffic did not change either.
  The boot printf now says which variant and that the tester's DRIVE station is the one
  thing that powers motors.
- **`Makefile`:** `ROBOT=tank` → `override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026`; the
  `$(error)` lists all three; the WARNFLAGS comment corrected from three documented
  `-Wunused-function`s to TWO (`robot()`, `portMapString()` — `shaped()` moved out).
- **`tools/src_build_gate.py`** (D6/D7): `VARIANTS`/`VARIANT_DEFINES`/`VARIANT_BEACONS`
  tables; token-exact define check (split on whitespace; the variant's define must be a
  whole token; every other variant's define must be absent); the beacon assertion on the ELF
  bytes (expected present, the other two absent); the differential now `bench ≥ 1, tank ≥ 1,
  xdrive == 0`; a `_replace` plant (asserts exactly ONE occurrence so a plant that matched
  nothing cannot pass) and three permanent self-test cases: 7 tank append dropped, 8 tank
  look-alike define, 9 xdrive append dropped. Header rewritten to say all of this, with the
  tank-behaves-like-bench note beside GATE1's coupling note and the "two, not three"
  documented count.
- **First measurement, before the gate ran:** `rm -rf bin .d && make ROBOT=tank` → exit 0,
  `bin/hot.package.bin` 22,492 B, two `-Wunused-function` (`portMapString`, `robot`), and
  `strings bin/hot.package.elf` shows `shulib-robot-variant=tank` and no other beacon.
- **Gate `check` on the finished tester — first run FAILED, correctly, on two NEW warnings of
  mine:** `-Wformat-truncation=` at `bench_r3a.cpp:1270` (a 63-byte silent-port list into a
  96-byte summary) and `:1663` (`snprintf("%s")` into an indexed `char[3][24]`, provably
  bounded only by the whole 72-byte array). Both real, both fixed (summary buffer 192;
  `memcpy` of the equal-sized row). The policy's "any other warning in our code fails" did
  exactly what GATE1 built it to do.
- **Gate `check` after the fixes: PASS** — bench 2/2 TUs, 2 allowed, 2 vendor-ignored, 0
  disallowed; xdrive 2/2, 0 allowed; tank 2/2, 2 allowed; hash `v0.1.1-274-gd92a9fe-dirty`
  and the beacon asserted in each. **Self-test: OK, 18 detector cases** (was 10).

#### 4.4 The tester — what changed in `src/bench_r3a.cpp` (1,188 → 2,044 lines)

- **Chassis table (D8):** `Cartridge{Unset,Red,Green,Blue}`, `ChassisTable`, ONE `kChassis`
  under `#if defined(SHULIB_ROBOT_TANK_2026)`: the tank table has `leftCount = rightCount = 0`,
  `cartridge = Blue` (the coordinator's relayed measurement, provenance string carries the
  source/date), `imuPort = 0`, `measured = false`; the bench table is the measured one
  (L 15–18, R 11–14, Blue, IMU 4, `measured = true`, provenance names §9.1/§15–§19 and the
  GREEN-on-brain finding of §20.3). Helpers: `tableHasPorts()`, `tableCount()`,
  `tableSideOf()`, `tableSide()`, `sideLetter()`, `describeMissing()` (names each unset field),
  `portsToString()`, `tableConsistent()` (duplicates, out-of-range, IMU-as-drive-port → refused
  by DRIVE and flagged in the banner). The old `hypothesisedSide()` and the four constants are
  gone; the census, the IMU station (refuses "IMU port UNSET" on tank), the static motor report
  (lists EVERY census motor with side `?` when the table has no ports) and the banner all read
  the table. `grep kLeftPorts|kRightPorts|kLeftCount|kRightCount|hypothesisedSide` → none.
- **MOTOR WATCH (D9):** iterates every census motor (up to 21) in port order, sides from the
  table or `?`; a `Grid`/`gridFor()`/`gridCell()` layout for N (2 columns to 10 cells, 3 beyond;
  pitch and font shrink to fit; the bench bot's 8 reproduces the original x=8/248, pitch 34,
  MEDIUM layout exactly); `WatchCapture g_watch` with `verdict[]` (last run) and `sign[]`
  (written only by a qualifying push: every table port moved, including table ports absent
  from the census counted as silent); the FRONT-FIRST / BACK-FIRST prompt after a qualifying
  push; five distinct outcomes each with its own `lastSummary` that DRIVE quotes when it
  refuses (nothing moved / single wheel / whole-robot signs captured / several moved but table
  has no ports / partial with the silent ports named).
- **DRIVE (D10/D11):** `driveStation()` + `namespace drive` constants and `Member`,
  `stopAll()`, `AllStopGuard`; menu entry 10 `"10 DRIVE (POWERS)"` with `powersMotors = true`
  (RED stripe and RED edge instead of amber; `kMaxMenu` 9 → 10 with a `static_assert` tying
  it to the menu count); the menu header reads "READ-ONLY except 10 DRIVE (powers motors)",
  the splash and the serial banner say the same; the banner prints `robot`, `table`,
  `provenance`, the UNSET list and any contradiction.
- **Also:** `twoButtonPrompt()` extracted for the two new prompts (the IMU test keeps its inline
  copy untouched — it has run on hardware); `bench_r3a.hpp`'s comment corrected (no longer
  "commands no motion").
- **Untouched, deliberately:** the IMU rotate test's body, the SD probe, the loop-rate test,
  the screen ruler — all measured-on-hardware code, and none of it reads the old constants
  except the IMU port, which now comes from the table.

#### 4.5 The DRIVE station's six gates — the written walk-through against the code

The station is PROS-only and cannot run on the host; its verification is (a) the ARM build via
the gate for all three variants (§4.3: PASS) and (b) this walk-through, quoting the guarding
condition of each gate as it stands in `src/bench_r3a.cpp` (line numbers at hand-off).

| Gate (brief §3.3) | Where | The guarding code, verbatim | What it refuses / does |
|---|---|---|---|
| 1 — refuse without a whole-robot sign capture for every table port | `driveStation()` `:1379-1420` | `if (describeMissing(missing, sizeof missing, true))` → `REFUSED: chassis table UNSET -- missing: %s`; `if (!tableConsistent(why, …))` → refused; every `tableSideOf(p) != 0` port must be `E_DEVICE_MOTOR` in the census; `if (!g_watch.signsValid)` → `REFUSED: no sign capture from a WHOLE-ROBOT push this power cycle` quoting `g_watch.lastSummary`; then every table port must have `g_watch.sign[p] != 0` | `g_watch.signsValid` is set ONLY in `motorWatch()` when `qualifies = tableHasPorts() && tableSilent == 0 && tableMoved == tableCount()` — i.e. every table port moved in one push (the moved-count == table-count rule, per port). On `ROBOT=tank` today `describeMissing` yields "LEFT ports, RIGHT ports" and the station returns before anything is constructed |
| 2 — wheels-off-the-ground confirmation before the first volt | `:1429-1441` | `const bool wheelsUp = twoButtonPrompt("ARE THE WHEELS OFF THE GROUND?", …, "YES, WHEELS UP", …, "NO", …)`; `if (!wheelsUp) { … return; }` | No adapter is constructed and no `setVoltage` is reachable before this returns true; NO goes back to the menu with a logged line |
| 3 — dead-man | `:1516-1525` | `const bool deadMan = connected && master.pressed(kDeadMan);` (`kDeadMan = ControllerButton::L1`) … `double vL = 0.0, vR = 0.0; if (deadMan && !cut) { vL = …; vR = …; }` then `setVoltage(units::Voltage{… vL : vR})` to every member every tick | Not held, controller disconnected (`connected` false → `deadMan` false) or cut → 0 V that tick; brake mode is Coast from `stopAll()` at `:1466` and never changed |
| 4 — ceiling starts at 3 V, steps 3→6→9→12, both screens, resets on re-entry | `:1490`, `:1511-1514`, panel `:1606-1612`, LCD `:1640-1650` | `double ceiling = kCeilingStartV;` (3.0, a function-local, so re-entry restarts at 3); `if (ceilingEdge.update(connected && master.pressed(kCeilingUp))) ceiling = std::min(ceiling + kCeilingStepV, kCeilingMaxV);` (R1, `hal::ButtonEdge`, cap 12.0); `vL/vR = std::clamp(ceiling * (…), -ceiling, ceiling)`; the panel prints `CEILING %2.0f V` every 100 ms and the LCD row 0 `DRIVE %2.0fV HOLD L1` on change | Never starts above 3 V; the command can never exceed the ceiling in magnitude |
| 5 — the fighting-motor cut-out + over-current, current and temperature live | `:1538-1602` | per side: `evaluate = std::abs(cmd) > kFightCommandFloorV && vmax > kMovingFloorRadS`; per member: `oppositeSign = (m.v * expected) < 0.0 && std::abs(m.v) > 0.5 * kMovingFloorRadS`, `nearZero = std::abs(m.v) < kNearZeroFraction * vmax`, `m.disagreeTicks = (oppositeSign \|\| nearZero) ? m.disagreeTicks + 1 : 0`, cut when `>= kPersistTicks` (25 × 10 ms = 250 ms); over-current `m.overTicks = m.amps > kCurrentLimitA ? m.overTicks + 1 : 0`, cut at 25 ticks (2.4 A for 250 ms); `if (cut) { stopAll(members, n); emitS(Sev::Bad, "%s", cutWhy); … }`; the loop keeps `if (deadMan && !cut)` false from then on | The whole drive goes to 0 V + Coast at the cut; the port(s) are named on the panel footer (`%.54s` of `cutWhy`), the LCD (`CUT - see brain`) and the serial/SD log with every member's v/A/°C at the cut; `cut` is a function-local, so only re-entering the station clears it. Current (`%4.2fA`) and temperature (`%3.0fC`) per port are on the panel every 100 ms and in the per-port summary at exit |
| 6 — exit → every motor 0 V, coast | `:1445`, `:1500`, `:1672-1673`, `:1367` | `AllStopGuard guard{members, &n};` (constructed before any motor exists; its destructor is `~AllStopGuard() { stopAll(m, *n); }`); `if (… release_count != touch0) break;` then `stopAll(members, n);` explicitly; `stopAll` = `setVoltage(0 V)` + `setBrakeMode(Coast)` on every constructed member | Holds on the touch exit, on a precondition thrown by a `ProsMotor` constructor (the guard runs; the members constructed so far were never powered), and on any other early return after construction |

Mapping and fan-out, as the brief requires: `req = teleop::mapSticks(sticks)` (`:1508`) — the
same pure function the library loop uses — then `vL = ceiling × (forward − yawCcw)`,
`vR = ceiling × (forward + yawCcw)`, both clamped to ±ceiling, and `setVoltage` with `vL` to every
`side < 0` member and `vR` to every `side > 0` member. The negative port is the ONE place a sign
lives: `m.signedPort = g_watch.sign[p] > 0 ? p : -p` (`:1451`) handed to `ProsMotor{signedPort,
gearset}`; nothing else negates (grep: no `-m.v`, no `-vL`, no sign multiplications anywhere in
the station). The cartridge belief is printed at `:1421-1423` BEFORE construction, quoting the
table's provenance.

**Never run on hardware** — stated plainly. The build team runs Station D; the SD log is the
evidence, appended to `R3a-PROGRESS.md` by the coordinator.

#### 4.6 Test 15 (the variant gate) and the gate tool's own detectors — mutations RUN, red OBSERVED, restored

All through the real `make`, sequentially (scratchpad `gate_mutations.sh`); every plant proven
applied (`cmp` against the saved original), every restore `cmp`-exact.

| # | Mutation (as applied) | Observed RED (verbatim) |
|---|---|---|
| T15a — **the brief's test-15 mutation**: Makefile line `override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026` replaced by a comment (ROBOT=tank silently builds bench) | `check` **exit 1**: `[tank] ROBOT=tank but src/bench_r3a.cpp compiled without the whole token -DSHULIB_ROBOT_TANK_2026 — the variant switch is a silent no-op again (§4.1)` (and the same for `src/main.cpp`), PLUS `[tank] ROBOT=tank but the beacon 'shulib-robot-variant=tank' is NOT in bin/hot.package.elf — src/main.cpp's #if chain did not take the tank branch …` and `[tank] … bin/hot.package.elf carries the bench beacon 'shulib-robot-variant=bench' — this is a bench build wearing a tank label`. bench and xdrive stayed green. Restored → `check` PASS |
| T15b — the GATE1-5b shape: the tank define replaced by the look-alike `-DSHULIB_ROBOT_TANK_2026_BROKEN` | `check` **exit 1** with the SAME four lines — the token-exact check refuses the look-alike (a substring check would have passed it), and the beacon says what the binary actually is. Note the warning-count detector stayed silent here, as predicted in D6: tank's 2 `-Wunused-function` are also bench's. Restored → `check` PASS |
| G1 — gate tool: `want_define not in tokens` → `want_define not in cmd` (the substring hole reopened) | `self-test` **exit 1**: `SELF-TEST FAILURE — 8: the token-exact structural check did not fire on the look-alike`. Exactly one case red, the one built for it (the beacon still caught the build, so `not ok` held — the expectation on the token message is what fired). Restored |
| G2 — gate tool: both beacon assertions disabled (`if False and …`) | `self-test` **exit 1**: `SELF-TEST FAILURE — 7: the beacon did not ALSO catch the bench build wearing the tank label` and `… 8: the beacon check did not fire on the look-alike`. Restored |

Post-restore: `self-test` **OK (18 detector cases)**; `git status` shows only the intended edits
to `Makefile` and `tools/src_build_gate.py`. (Afterwards one word in `_summary` was changed from
"beacon asserted" to "beacon checked", because the old wording printed on a failing build too;
the final battery in §5 re-runs both gate commands on that text.)

### 5. Final verification (all from the repo root, on the finished working tree)

| Check | Result |
|---|---|
| Host build (all doc gates in-build) | exit 0 |
| Host suite | **1163 cases / 1,538,459 assertions, 0 failed, 3 skipped** (dirty tree; +6 new cases, +349 assertions from `stick_mapping_test.cpp`, +3 tree-dependent from the fidelity test, +6 from `-dirty`). **Clean-tree expectation: 1,538,453** |
| `api_doc_tool.py` self-test / check-coverage / check-fresh / check-examples / check-removability | **5 × exit 0** (coverage: **1,659 entities / 119 headers**, all documented) |
| `briefing_status.py check` | **exit 0** on this tree — see the note below |
| `doc_staleness_audit.py` self-test + audit | **2 × exit 0** (0 numeric claims in scope) |
| PROS-free guard (ci.yml form) | **CLEAN** — `include/shulib/teleop/stick_mapping.hpp` includes no `<pros/…>` |
| sim-layering guard (ci.yml form) | **CLEAN** |
| ARM header cross-compile (RESUMING §3, anchored sed) | **152 headers, exit 0**, `-Werror` clean |
| `src_build_gate.py self-test` | **OK, 18 detector cases** (10 → 18) |
| `src_build_gate.py check` | **PASS — all THREE variants link** (bench 2 allowed / xdrive 0 / tank 2 `-Wunused-function`, 0 disallowed each, hash `v0.1.1-274-gd92a9fe-dirty` and the beacon asserted in each) |
| `.github/workflows/ci.yml` | parses (`yaml.safe_load`); only comment lines changed |
| C7 removability grep (public docs for `internal/`, `chunks/`, `RESUMING`, `build-order`) | empty |
| `docs/api/` | **121 pages** (120 + `stick_mapping.md`); nav regenerated |
| `git` | 16 modified + 3 new paths; **nothing staged, nothing committed, nothing pushed**; no `MUTATION` residue in any file; the mapping header `cmp`-identical to its saved pre-campaign copy |

**The briefing carries the DIRTY-tree suite number, and this is the documented state, not a
mistake:** `PROJECT-BRIEFING.md`'s generated block was regenerated on this tree (headers 152,
suite 1,163 / 1,538,459) because the briefing gate is a hard dependency of the test binary and
the derived header count moved as soon as the new header existed — there was no way to keep the
clean number AND build. On the committed clean tree the same binary reports **1,538,453**
(`-dirty` is 6 characters, asserted through — R3a-PROGRESS §12.2; this log's Session 1 §6 hit the
identical trap). **Coordinator:** after committing, reconfigure, rebuild, run
`python3 tools/briefing_status.py generate` on the clean tree and amend, exactly as Session 1
did. Writing 453 into the block by hand would be hand-editing generated text.

### 6. WHAT IS NOT DONE — read this before believing any checkbox

- **Parts 1–3 are untouched, deliberately** (brief §2.1, landmine 10): no `hal::MotorGroup`, no
  `localization::IOdometry` / `DriveEncoderOdometry`, no single drive geometry, no `>=` → `==`
  guard change, no plant coupled-member support, no `MotorGroupDisagree`, no tank composition
  root, no TELEOP / BENCH TESTS chooser, no shared teleop-loop function. Tests 1–13 do not exist.
  `ROBOT=tank`'s `initialize()` builds no library graph and its `opcontrol()` runs the tester.
- **The DRIVE station has never run on hardware.** Neither has `ProsMotor` or `ProsController`
  (HA-94 onward stay unsettled). Its verification here is the ARM build for all three variants
  and the §4.5 walk-through; the SD log from Station D is the evidence, and it does not exist
  yet. Every threshold in gate 5 (1 V, 1 rad/s, 25 %, 250 ms, 2.4 A) is INVENTED and says so on
  screen; the register gains rows for them only when R3d/R4 measure them.
- **The tank chassis table is UNSET for ports and the IMU port.** DRIVE refuses on `ROBOT=tank`
  today by design (`REFUSED: chassis table UNSET -- missing: LEFT ports, RIGHT ports`); the IMU
  station refuses `IMU port UNSET`. Only the cartridge is set (BLUE — the build team's reported
  reading, relayed by the coordinator, provenance in the table). Wheel diameter (2.75 in) and the
  "600 rpm"/direct-drive ratio (UNCONFIRMED) are recorded in §4 of this log and nowhere in code.
- **The worksheet's D.1.8 records a limit:** the station has no ground mode — the wheels-up gate
  refuses "NO" — so driving on the floor is a finding for the next revision, not something the
  station lets a helper work around.
- **The library has still never driven a robot.** `docs/guide/14` is untouched and still true.
  `roadmap.md`'s "you are here" and M1's badge, `build-order.md`, and
  `docs/hardware-assumptions.md` are untouched — the first two are the coordinator's
  close-of-chunk call (brief §9), and the register takes only MEASURED facts, of which Part 0
  produced none (the three relayed facts are logged here, not registered).
- **CI was never executed by a runner** (as GATE1 §11 said). The commands are byte-identical to
  the ones run above; the one environmental difference (tagless shallow checkout → bare-hash
  `git describe`) is handled by the tool computing the hash fresh, unchanged from GATE1.
- **Beyond the brief's minimum, for the coordinator to accept or drop** (each with its reasoning
  in §3): the variant identity beacon and the token-exact define check (D6); the FRONT-FIRST /
  BACK-FIRST prompt after a qualifying push (D9); the three permanent Makefile-mutation
  self-test cases (D7); the `twoButtonPrompt()` helper (the IMU test keeps its inline copy);
  comment-only edits to `.github/workflows/ci.yml` and `docs/internal/RESUMING.md` so "both
  variants" does not read as a lie; the worksheet's Station A button table corrected to the
  actual menu (it had listed the pre-MOTOR-WATCH menu).
- **The behavioural (warning-count) detector cannot distinguish `tank` from `bench`** — stated
  in the tool header and measured in T15b, where it stayed silent. The beacon is what covers it.
- Nothing committed, nothing staged, nothing pushed. The working tree carries: 1 new header,
  1 new test file, 1 new api page + regenerated README/all-entities/mkdocs-nav/PROJECT-BRIEFING,
  `main.cpp` / `bench_r3a.cpp` / `bench_r3a.hpp` / `Makefile` / `src_build_gate.py` /
  `ci.yml` edited, `docs/README.md` + `docs-publishing.md` counts, `RESUMING.md`'s variant
  list, the worksheet's Station D, the R3a-PROGRESS cross-reference, and this log.

PART 0 READY FOR VERIFICATION

---

### 7. Coordinator's verification: ONE gap — gate 2 could never drive on the ground (fixed in place)

**Brief correction, recorded as such.** §3.3 gate 2 reads "Confirm wheels-off-the-ground on
screen before the first volt", and this pass implemented it as a PERMANENT refusal — answering
NO returned to the menu, and worksheet D.1.8 told the builder to report the missing ground mode
as a finding. But §3.5's own procedure ends "DRIVE at 3 V wheels up → raise → ground", and the
build team's actual need is to drive the chassis. The two statements were reconciled by the
coordinator into a **two-stage gate 2**, implemented here exactly as specified:

- (a) the FIRST DRIVE run in a power cycle must still be wheels-up, exactly as before;
- (b) at station exit the run is recorded in a process-global beside `g_watch` —
  `DriveRecord g_drive` (`ran`, `cleanWheelsUp`, `lastMode`, `lastClean`, `lastSummary`;
  one power cycle, like the sign capture) — as CLEAN iff no cut, motors driven ≥ 3 s of ticks
  (`kCleanDrivenSeconds`) and the ceiling reached ≥ 6 V (`kCleanCeilingV`);
- (c) answering NO to "ARE THE WHEELS OFF THE GROUND?" now branches: if
  `g_drive.cleanWheelsUp`, a SECOND explicit prompt "ON THE GROUND? clear 3 m all round, a
  second person at the battery" — YES enters **GROUND mode** (ceiling resets to 3 V because it
  is a function-local initialised from `kCeilingStartV`; gates 3–6 untouched; the panel header
  reads `10 DRIVE -- GROUND MODE`, the LCD row 0 reads `GROUND %2.0fV HOLD L1`, the exit summary
  reads `DRIVE (GROUND) ended: …` and the log line names the mode and whether it was clean);
  else it refuses: `REFUSED: ground driving needs a clean wheels-up run at >= 6 V this power
  cycle first.` quoting `g_drive.lastSummary`.

**D12 — what a LATER cut does to the ground permission** (the one point the specification left
open): `cleanWheelsUp` is SET by a clean wheels-up run and CLEARED by any later run that CUT, in
either mode — a cut is new evidence of a fight, and the permission was the claim that there is
none. A later wheels-up run that is merely short or stays at 3 V is not evidence against and
leaves the permission as it was.
- Rejected: sticky-forever ("a clean run exists this power cycle") — the literal reading; it
  would let a robot drive on the ground after the drivetrain had just demonstrated a fight.
- Rejected: last-run-only (permission = the immediately preceding run was clean) — a 3 V
  wheels-up sanity check after a clean 6 V run would lock the ground for no reason.
- One line to change if the coordinator prefers the literal rule (`else if (cut)` in the exit
  block).

Worksheet D.1.8 rewritten to match: ground run 3 V first, then 6 V, then stop; above 6 V only
with the team lead present; the refusal text quoted so a helper knows it is the program, not
them. The menu's DO NOW line says "WHEELS UP first (ground only after a clean 6 V wheels-up
run)". The station's header comment (gate 2) rewritten. `docs/roadmap.md`, `build-order.md`
and `GATE1-COMPLETED.md` untouched (the coordinator's in-progress edits).

#### 7.1 Gate 2 (two-stage) — walk-through against the code (line numbers at hand-off)

| Stage | Where | The guarding code, verbatim | Effect |
|---|---|---|---|
| The record | `struct DriveRecord` `:1088-1095`, `g_drive` beside `g_watch` | fields `ran`, `cleanWheelsUp`, `lastMode`, `lastClean`, `lastSummary`; process-global, one power cycle | Boots `cleanWheelsUp = false`: the FIRST run in any power cycle cannot be a ground run |
| Clean criteria | `:1356-1357`, `:1737` | `kCleanDrivenSeconds = 3.0`, `kCleanCeilingV = 6.0`; at exit `const bool clean = !cut && drivenSeconds >= kCleanDrivenSeconds && ceiling >= kCleanCeilingV;` where `drivenSeconds = drivenTicks * kTickMs / 1000.0` counts only ticks with a non-zero command | "no cut, motors driven for at least 3 s of ticks, ceiling reached at least 6 V" |
| Stage 1 — wheels-up | `:1460-1467` | `DriveMode mode = DriveMode::WheelsUp; const bool wheelsUp = twoButtonPrompt("ARE THE WHEELS OFF THE GROUND?", …, "YES, WHEELS UP", …, "NO", "it is on the floor", …);` | YES → wheels-up mode, exactly as before |
| Stage 2 — refusal | `:1467-1477` | `if (!wheelsUp) { if (!g_drive.cleanWheelsUp) { … emitS(Sev::Bad, "REFUSED: ground driving needs a clean wheels-up run at >= 6 V this power cycle first."); emitf("  DRIVE says: %s", g_drive.lastSummary); … return; }` | Names the reason and quotes the last run's summary; no adapter constructed |
| Stage 2 — second confirmation | `:1478-1489` | `const bool ground = twoButtonPrompt("ON THE GROUND?", "clear 3 m all round, a second person at the battery", "YES, GROUND MODE", "3 V first, then 6 V", "NO", "back to the menu", …); if (!ground) { … return; } mode = DriveMode::Ground;` | Only YES enters ground mode; NO logs "not driven: ground mode was not confirmed" |
| Ground mode = same gates | `:1547` and the loop | `double ceiling = kCeilingStartV;` (function-local → 3 V on every entry, both modes); gates 3–6 are the unchanged code of §4.5, `mode` is never read inside the tick loop | Nothing about the dead-man, the ceiling steps, the cut-out or the exit differs on the ground |
| Both screens say GROUND | `:1536`, `:1712` | `drawHeader(mode == DriveMode::Ground ? "10 DRIVE -- GROUND MODE" : "10 DRIVE -- WHEELS UP");` … `std::snprintf(want[0], …, "%s %2.0fV HOLD L1", lcdMode, ceiling)` with `lcdMode = "GROUND"` in ground mode | Panel header and controller LCD row 0 |
| Exit summary and log name the mode | `:1737-1757` | `g_drive.lastMode = mode; g_drive.lastClean = clean; if (mode == DriveMode::WheelsUp && clean) g_drive.cleanWheelsUp = true; else if (cut) g_drive.cleanWheelsUp = false;` … `emitS(…, "DRIVE (%s) ended: %s", modeWord, …)`; then "this %s run was CLEAN/NOT clean: driven … (need >= 3), ceiling … (need >= 6), cut/no cut" and "ground mode is UNLOCKED/LOCKED …" | The SD log carries the mode, the clean verdict and its three inputs for every run; D12's revoke-on-cut rule is the `else if (cut)` |

`src/bench_r3a.cpp` is 2,123 lines. Gate check after the change: **PASS, all three variants, 0
disallowed** (no new warning from the change).

#### 7.2 Verification re-run after the gate-2 change (all from the repo root)

| Check | Result |
|---|---|
| `src_build_gate.py check` | **PASS — all three variants link**, 0 disallowed in each (bench 2 / xdrive 0 / tank 2 `-Wunused-function`), hash + beacon asserted |
| `src_build_gate.py self-test` | **OK, 18 detector cases** |
| Host build | **stops at the briefing gate, as the coordinator predicted** — see the note below. The test binary's md5 is `86cdc8c4…` before and after (the last green link's binary; no header or test file changed since it was linked, `git status include test` shows only the two new untracked paths) |
| Host suite (that binary) | **1163 cases / 1,538,459 assertions, 0 failed, 3 skipped** (clean-tree expectation 1,538,453) |
| `api_doc_tool.py` self-test / coverage / fresh / examples / removability | **5 × exit 0** (1,659 entities / 119 headers) |
| `doc_staleness_audit.py` self-test + audit | **2 × exit 0** |
| PROS-free guard / sim-layering guard | both **CLEAN** |
| ARM header cross-compile | **152 headers, exit 0** |
| C7 removability grep | empty |
| `git` | nothing staged, nothing committed, HEAD `d92a9fe`; no `MUTATION` residue |

**`briefing_status.py check` — exit 1, NOT regenerated, deliberately.** The coordinator wrote
`docs/internal/chunks/GATE1-COMPLETED.md` into the tree while this pass ran, and the generated
block drifts on exactly that fact and nothing else: `Position: 26 → 27 of 47 chunks complete`,
`INTERRUPTED CHUNK(S): GATE1, R3a, R3b → R3a, R3b`, and `GATE1` joining the completed list —
three changed lines (six diff lines), one cause. The coordinator predicted one line; it is three,
all from the one file, and **the suite line is NOT among them** (the block's 1,163 / 1,538,459
still matches the binary). Regenerating here would fold the coordinator's in-progress edit into
this pass's diff, so it is left for the commit that lands GATE1-COMPLETED.

**Tree at hand-off.** This pass's paths are unchanged from §5 plus the gate-2 edits to
`src/bench_r3a.cpp` and worksheet D.1.8. Also present, NOT this pass's and untouched by it:
`docs/roadmap.md` (modified), `docs/internal/build-order.md` (modified),
`docs/internal/chunks/GATE1-COMPLETED.md` (untracked) — the coordinator's in-progress edits.

**§6 addendum.** Ground mode now exists and has, like the rest of the station, never run on
hardware. D.1.8 is the procedure; its refusal text is quoted there so a helper can tell the
program's "no" from their own mistake. The D12 revoke-on-cut rule is one line if the coordinator
prefers the literal "a clean run exists" reading.

PART 0 READY FOR VERIFICATION

---

### 8. Coordinator's close of Part 0, and the port report (2026-09-10)

**Verified independently and committed as `e16f336`** (amended once so the briefing carries the
CLEAN-tree suite number, **1163 / 1,538,453 / 0 failed / 3 skipped**, measured after a fresh
configure on the committed tree — the dirty-tree 1,538,459 above is the same suite plus the six
`-dirty` characters, per Session 1 §6). Re-run by the coordinator, not taken from §5/§7.2: the
suite, the five `api_doc_tool` gates, both staleness runs, the PROS-free and layering guards, the
ARM header gate (152), `src_build_gate.py` self-test (18) and check (three variants, beacons), and
the six DRIVE gates plus the two-stage gate 2 read against the code. D12 (a cut re-locks ground
mode) ACCEPTED as written. The pace rules that this session's speed earned went into
`RESUMING.md` as `1a15ec0`.

**The port report arrived after the commit** (team lead, from the robot, standing behind it with
its back against him, looking toward the front): **LEFT 11 12 13 14 15, RIGHT 20 19 18 17 16, each
back → front** (11 and 20 rearmost; 15 and 16 frontmost). **The front is therefore the 15/16
end** — implied by the way the report was read rather than declared, so worksheet D.0.6 asks for a
description or photograph of that end. Typed into the tank table (`src/bench_r3a.cpp`, the ONE
place), `.measured` left **false** with a provenance line saying so: the ports were read from the
robot by a person, and station 1's census is what confirms them; flipping the flag before that
would be the over-claim the table's own rule forbids. The IMU stays UNSET (none mounted). Gate
check on the filled table: **PASS, all three variants link**, hash `v0.1.1-276-g1a15ec0-dirty`.
`tableConsistent()` holds by inspection: ten distinct ports in 1..21, no side overlap, no IMU
overlap. Worksheet D.0.6/D.0.7, the roadmap pointer and `build-order.md`'s Next block record the
same facts in the same commit.

**What the next hand at the robot does:** upload `make ROBOT=tank` to slot 3, then Station D —
census (flips `measured` when it shows MOTOR on all ten), the whole-robot push front-first,
DRIVE on blocks at 3 V. Nothing in Part 0 has yet run on hardware.

---

### 9. First hardware session on robot two (2026-09-10, coordinator at the laptop, team lead at the robot)

**The tester ran on robot two's brain. Nothing was DRIVEN.** What follows is measured, with the
evidence quoted; the serial captures themselves live in the session scratchpad, not the repo.

**9.1 Platform, as the kernel banner printed it:** PROS `4.2.2` on VEXos **`1.1.5 (b18)`**;
`arm-none-eabi-g++ 13.2.1` builds. Upload over the brain's USB with `pros upload --slot 3
--after run`; the first upload replaced the brain's onboard PROS library (`Library onboard
doesn't match`, two orphaned slots removed). **`pros terminal` cannot run from the coordinator's
non-interactive shell** (`termios` on stdin); raw reads of the brain's USER CDC port work instead —
the stream is COBS-framed with a `sout` channel tag, and `strings` recovers the text lines whole.
**The port numbers move:** the brain enumerated as `ttyACM0/1` alone and as `ttyACM1/2` beside the
controller; every read must re-resolve them from `pros lsusb`. The brain dropped off USB three
times in the session; the last time was the battery (`12.97 V / 53 %` at station 4 about an hour
earlier) — "the brain died".

**9.2 "The touchscreen isn't working" — measured, and it was not the screen.** With a touch
readout added to the menu loop (every change of `screen_touch_status()` → serial and the header
bar; commit below), the program recorded:
```
touch: status=1 x=364 y=117 press=1 release=0 hit=-1
touch: status=0 x=361 y=117 press=1 release=1 hit=-1
touch: status=1 x=355 y=101 press=2 release=1 hit=3
touch: status=0 x=354 y=101 press=2 release=2 hit=3
>>>>>> 4  BATT/CTRL
battery | RAW 12970 mV  53.0% | CANON 12.970 V
controller master: NOT CONNECTED
```
Taps register and the hit-test works. The first tap released at `y=117` — **in the 4 px gap
between rows 2 and 3** (78–114 / 118–154) — and did nothing; the second hit button 4. **Fixed:**
`hitTest()` now assigns the gaps to the button above/left (integer division of the pitch), so
no point inside the grid is dead, while the header strip and the margin still hit nothing. Also
fixed: the readout's at-rest print passed `hit=-1` unconditionally, which made a tap that landed
during the boot census read as a miss (`T 194,110 r1 h-1` on screen); it now hit-tests too.

**9.3 The program went black when the controller linked — competition state, not touch.** Right
after station 4 the log carries a second PROS boot banner: the program RESTARTED when the
controller linked, and afterwards the screen stayed black. State screens were added to
`disabled()` / `competition_initialize()` / `autonomous()` and the status bits print at boot;
the next run showed, on screen and on serial:
```
[R3A] competition status at boot: 0x0f field=yes disabled=yes auton=yes
[R3A] competition state: FIELD CONTROL CONNECTED (0x0f ...)
```
`0x0f` = CONNECTED | DISABLED | AUTONOMOUS | **SYSTEM** — VEXos reporting a field control
*system* — and it persisted across `pros v5 stop` / `run`. No cable was in the controller's smart
port. **Turning the controller OFF cleared it within seconds**: the tester booted into driver
control, ran its census, and the menu came up. **Then the decisive test:** controller linked,
team lead's hands off it, program started from the laptop → driver control, menu up, stable for
several minutes. **The pretend match came from launching / handling the program from the
controller's own menu** (which can start a program as Run / Timed Run / Match), not from the link
itself. Rule recorded in the worksheet: start the program from the brain or the laptop, never
from the controller; a linked controller on its home screen is fine. (The `0x0f` at boot after a
plain CLI `run` says VEXos keeps the controller-driven mode armed until the controller leaves it.)

**9.4 The census, twice, on robot two:** `RADIO` on port 1; motors on **11 12 13 14 15** (LEFT,
table) and **16 17 18 20** (RIGHT, table) — **`motors found: 9`, port 19 absent**. A finding, not
a table error: the cable is to be re-seated and the census re-run. The table's `measured` flag
therefore stays false, correctly.

**9.5 Not done today:** station 4 with the controller linked in driver control (it read NOT
CONNECTED once, before the link); MOTOR WATCH's push; DRIVE. Ten motors have not yet answered.
Battery dead at session end.

**9.6 Code that rode with this session (commit below, docs in the same commit):** the touch
readout (`touchReadout()`, menu loop), the gap-absorbing `hitTest()`, the competition-state
screens in `src/main.cpp` (`benchStateScreen()`, `competitionBits()`, tester variants only), and
the boot-time status line. Verified: src build gate PASS for all three variants after each
change; no host input changed. Register entries for VEXos 1.1.5, the controller-launch trap and
the coupled-USB port renumbering are owed to `hardware-assumptions.md` with robot two's section.

---

### 10. Second session on robot two, the same evening (2026-09-10, team lead back in the lab)

Fresh battery, the latest build (`v0.1.1-282-g140b9c8`, stamp `Sep 10 2026 22:06:49`) uploaded
over USB and started from the laptop. Boot status **`0x00`** — driver control — with the
controller linked and left on its home screen, exactly as §9.3 predicted.

**10.1 Station 4, first read with the controller linked:**
```
battery | RAW 13161 mV  96.0% | CANON 13.161 V
controller master: CONNECTED
  LeftY=+0.000 LeftX=+0.000 RightX=+0.000 (canonical [-1,1]; PROS raw is +-127)
```
`ProsController::isConnected()` true on hardware for the first time; the axes read 0 at rest.

**10.2 The census, three times.** First `7` motors (11, 15, 19 silent — two more than the
morning's 9, disturbed by the battery swap or the cable work); after re-seating, **`motors
found: 10`**, every table port, plus **`IMU` on port 2** (mounted since the morning) and `RADIO`
on port 1. `measured` flipped true in the table on this evidence; `imuPort = 2`.

**10.3 Two sign captures, identical:** `signed ports for DRIVE: LEFT -11 +12 -13 +14 -15 |
RIGHT +20 -19 +18 -17 +16` — alternating on both sides, the coupled-train signature (bench bot
§17.3/§18.2 again). **Port 18 travelled short on both pushes:** 137 vs ~175 deg, then 171 vs
~210 deg — about 20 % under its side-mates, reproducibly, the same class of anomaly as the bench
bot's p16/p13 (R3a-PROGRESS §20.2). Not slip: it reproduces. Unexplained; watch it under power.
The second capture was needed because the program had been stopped in between — the VEXos
built-in **"Drive"** launcher was on the brain's screen ("nothing moving"): the program that made
the motors fight in the morning. Restarted Bench Tests from the laptop; the capture lives per run.

**10.4 Station 10 DRIVE entered — the first time ten `ProsMotor`s were constructed on hardware.**
```
CARTRIDGE BELIEF: BLUE 600 rpm
signed ports: LEFT -11 +12 -13 +14 -15 | RIGHT +20 -19 +18 -17 +16
mode: WHEELS UP
10 motors constructed through ProsMotor, gearset written and read back OK.
driving, WHEELS UP. Hold L1; R1 raises the ceiling; TOUCH the panel to stop.
ceiling -> 6 V / 9 V / 12 V
```
All six gates passed on hardware; the gearset write/read-back succeeded on all ten; every motor
read `0.0 rad/s`, `0.00–0.07 A`, `30–35 C` at rest; `R1` stepped the ceiling (button reads work);
`L1` held showed `DRIVING (L1 held)` in the footer (dead-man works). **But the commanded volts
stayed `L +0.0 V  R +0.0 V` with L1 held** — no stick deflection reached the mapping at any
screenshot, so **no motor was ever commanded a non-zero voltage and nothing turned.** Two
readings of that fact are possible and the session ended (brain off USB, no "now") before the
decisive check — L1 held AND the left stick pushed fully up, panel photographed — separated
them: (a) the operator was not deflecting the LEFT stick at the instants sampled (the ceiling had
been raised to 12 V before any motion, suggesting R1 was being pressed instead), or (b) the
axis read is wrong on this hardware despite the correct channel mapping (HA-103's untested half).
**The first thing the next session does is that check.** Until then: adapters constructed and
read on hardware — YES; a motor powered by the library's adapters — NOT YET.

**10.5 Not done:** the decisive stick check; any powered motion; the ground run; the tester's
IMU station (needs the rebuild with `imuPort = 2`, which this commit carries — the next upload
has it).

---

## Part 0b — 2026-09-10 (late) — THE DRIVE PROGRAM (brief `R3b-part0b-drive-program.md`)

**Scope declaration, written before any code (Session 1's precedent, so an interrupted log cannot
over-claim):** this section covers ONLY the brief's §1–§5 — the chassis table moved into
`src/chassis_table.hpp` and SIGNED for robot two with the measured signs; MOTOR WATCH's
AGREES/DISAGREES cross-check and `driveStation()` honouring §2; `src/drive_program.cpp` +
`.hpp` (`bench::runDrive()`); the two PROS-free teleop headers `drivetrain_degradation.hpp`
and `coupled_side_monitor.hpp` with host tests and every §4 mutation run; the 3 s brain
chooser in `opcontrol()`; `project.pros` renamed; the documentation pass. **Nothing here has run
on hardware** — the coordinator uploads. Nothing is committed by the executor. Tree at start:
`db20c34` (clean). Baseline claimed by the brief: suite 1163 / 1,538,453 clean-tree
(1,538,459 dirty), 152 headers, gate PASS ×3 — re-measured below before being trusted.

### 1. Baseline re-established (clean tree, `db20c34`)

(appended when the build finishes)

### 2. Brief checks — what the reading found before writing

- **CHANGE OF RULING, received from the coordinator mid-task (team lead, 2026-09-10 late;
  overrides the brief's §1 and the DoD lines that depend on it).** The drive program is a
  SEPARATE PROGRAM in its own slot, not a chooser inside Bench Tests. **Reason, verbatim in
  substance:** the operator picks a program by name from the brain's slot list, and a program
  called Drive that contains the tester behind a timeout is one more thing to explain at a
  field. Consequences, all implemented in this pass: a second build axis
  `make ROBOT=tank PROGRAM=drive` adds `-DSHULIB_PROGRAM_DRIVE` (`PROGRAM ?= tester`, validated
  values `tester` / `drive`, anything else an `$(error)`; `drive` with `ROBOT=bench` or
  `ROBOT=xdrive` is an `$(error)` too — only a signed table can drive); `src/main.cpp`'s
  `opcontrol()` runs `bench::runDrive()` under `SHULIB_PROGRAM_DRIVE` with NO chooser and the
  tester exactly as today otherwise; a fourth beacon `shulib-robot-variant=tank-drive`;
  `tools/src_build_gate.py` builds FOUR entries (bench, xdrive, tank, tank-drive) with
  token-exact define sets, per-build beacons, the behavioural detector (tank-drive like tank),
  and a self-test case for a non-landing `PROGRAM` define, run as a mutation; uploads are
  `pros upload --slot 1 --name "shulib Drive"` for the drive program and `pros upload --slot 3`
  for the tester, both documented in the worksheet's Station E and Daniel's note;
  `project.pros` keeps `"Bench Tests"` (the brief's rename is withdrawn). Everything else in
  the brief stands. The chooser code had not yet been written when the ruling arrived, so
  nothing was thrown away.
- **Beacon substring hazard, found while planning the fourth beacon:** `shulib-robot-variant=tank`
  is a prefix of `shulib-robot-variant=tank-drive`, so the gate's "the other beacons must NOT be
  in the ELF" assertion would have false-failed every tank-drive build. Fixed in the gate by
  matching each beacon WITH its string terminator (`beacon + b"\0"`, which is how a C string
  literal sits in the ELF); documented in the tool header.
- **The old `tableConsistent()` cross-side check was DEAD code:** it asked `tableSideOf()` of a
  left-side port and tested `!= -1`, which can never be true because the left side is searched
  first. A port typed on both sides was never caught. The check is rewritten in
  `src/chassis_table.hpp` (judged by |port|, both directions) and test 4 pins it, including the
  signed case `+13` left / `-13` right.
- **`reportMotorGroup()` indexed `found[]` by the raw table entry.** With a signed table a `-11`
  entry would have indexed the array off its front and asked PROS for a REVERSED reading in a
  station that promises RAW values. It now uses `absPort()`. (Latent until the signs moved into
  the table; fixed in the same pass that made it live.)
- **The brief's §3 "the RAII all-stop guard … runs on that exit too" (a field-disable killing the
  task) is NOT how a task kill works:** PROS deletes the opcontrol task; a deleted FreeRTOS task
  does not unwind its stack, so no destructor runs. VEXos itself disables motor output when the
  field says DISABLED, which is what actually stops the robot. The drive program therefore polls
  `competition_is_disabled()` every tick and EXITS its loop cleanly on it, so the guard does run
  on that path too — and the code comment says which mechanism covers which case rather than
  claiming the guard covers a kill.

### 3. Decisions taken (with the rejected alternatives)

**D0b-1 — The table's signs are the port entries themselves (`{-11, 12, -13, 14, -15}`) plus a
`signsMeasured` flag, not a parallel sign array.** The signed entry is exactly the value
`ProsMotor` takes, so "the one place a sign lives" is a value that is handed on unchanged
(`tableSignedPort()`), never composed. Consequences: every table helper matches by `|port|`
(`absPort()`); a raw PROS read in the tester uses `|port|` too (a negative port would ask PROS
for a REVERSED reading). "Half-signed" is representable only as a `-` entry in a table whose flag
is false, and `tableConsistent()` refuses that; a signed table with an "unmeasured" port is
unrepresentable by construction, which is the point.
- Rejected: a parallel `sign[]` array with 0 = unknown — the drive program would compose
  `sign × port`, a second place where a sign is applied, and the exact thing landmine 1 forbids.

**D0b-2 — `src/chassis_table.hpp` is PROS-free and host-tested by relative include.** The table
is plain data plus pure helpers, so `test/chassis_table_test.cpp` includes `"../src/chassis_table.hpp"`
and breaks the consistency checks with hand-built tables (brief test 4). It stays in `src/` because
it is a per-robot composition fact, not library API (it is not a doc-tool target and not in the ARM
header gate; the src build gate compiles it in every build).
- Rejected: a third header under `include/shulib/` — landmine 4 allows exactly two.
- Rejected: leaving the checks untestable in `src/` — test 4's mutation would have had nowhere to go.

**D0b-3 — The per-side arcade arithmetic (`tankSideVolts`) and the runtime dead-port detector
(`MemberAbsenceDetector`) live in `coupled_side_monitor.hpp`.** The side command IS the monitor's
input and absence IS what the monitor's `present` flag consumes, so both sit with it; the brief's
test 3 needed the arithmetic host-visible, and landmine 4 forbids touching `stick_mapping.hpp`
(the frozen surfaces stay frozen; only the two new headers are added).
- Rejected: `tankSideVolts` in `stick_mapping.hpp` — touching an existing public header.
- Rejected: leaving the absence detection as ad-hoc counters in the PROS-bound loop — untested
  logic in the one place a wrong threshold marks a live port dead.

**D0b-4 — The tester's gate 5 now RUNS THROUGH the shared monitor** (not just "was extracted
from"): `driveStation()` builds a `CoupledSideMonitor` per side and mirrors its per-member ticks
into the panel's colouring; five `static_assert`s pin the monitor's defaults to the station's
documented numbers. One detector, one set of numbers, host-tested; the station's over-current cut
and its permanent-cut policy are unchanged (landmine 6: its gates stay).
- Rejected: keeping the station's inline copy — two copies of a detector that must agree.

**D0b-5 — The degradation policy is applied LITERALLY (`< 3` per side, `> 2` total) and its domain
is stated.** A drivetrain with fewer than three motors per side would be refused on every boot;
the header says so and says why generalising silently would be an invented answer. Nonsense counts
(a side empty, negative, more answering than listed) REFUSE with a reason — the safe direction.

**D0b-6 — A REFUSE at runtime is a permanent 0 V until the field disables, painted; a boot REFUSE
paints and returns.** The brief says "cut to 0 V and paint why"; the panel keeps updating so the
operator sees the reason and the counts, and the loop's disable-poll still exits it cleanly.

**D0b-7 — The drive program polls `competition_is_disabled()` and exits its loop on it** (§2's
finding: a task kill runs no destructors). `opcontrol()` returns; PROS calls it again on re-enable;
the SD sink is a function-local static so re-entry does not truncate the log (the entry count is
logged).

**D0b-8 — Cuts are logged per port only for the first 50** (`kMaxCutsLogged`); after that they are
counted on the panel and LCD but no longer written per tick-of-cut. A permanent fight would
otherwise fill the card at one block per second. The count itself is never capped.

**D0b-9 — The drive program's screen helpers are its own** (a fixed status page, ~60 lines), not a
shared screen library extracted from `bench_r3a.cpp`. The tester's scrolling-log helpers are the
wrong shape for a status page, and extracting them would have refactored hardware-proven code for
no consumer. The palette constants are duplicated and say so.

**D0b-10 — The gate matches beacons WITH their NUL terminator** (§2's prefix hazard), and a build
is a SET of defines (`VARIANT_DEFINES` tuples, `ALL_DEFINES` the universe): every define of the
build present as a whole token, every other define absent. `BUILD_ARGS` carries the make
arguments so `run_build()` never re-implements the Makefile's switches.

### 4. Work log (appended as it happens)

- Baseline re-measured before anything: suite **1163 / 1,538,453 / 0 failed / 3 skipped** on the
  clean tree (`build/test/shulib_tests` md5 `87b6a9e5…`, saved as the green baseline); matches the
  brief's number exactly.
- Two headers written (`drivetrain_degradation.hpp`, `coupled_side_monitor.hpp`) and smoke-compiled
  clean on host g++ and arm-none-eabi-g++ at the CI flag set (`-Wall -Wextra -Wconversion
  -Wsign-conversion -Wshadow -Werror`). `std::abs(double)` avoided in constexpr code (only
  guaranteed constexpr from C++23) — a local `magnitudeOf()`.
- Three tests written (`drivetrain_degradation_test.cpp`, `coupled_side_monitor_test.cpp`,
  `chassis_table_test.cpp`); `<initializer_list>` added after the IDE flagged the range-for over a
  braced list.
- `src/chassis_table.hpp` written; `src/bench_r3a.cpp` rebound to it (table block removed, every
  helper call gains `kChassis`, `reportMotorGroup()` reads by `|port|`, the four "src/bench_r3a.cpp's
  table" strings now name `chassis_table.hpp`); MOTOR WATCH's AGREES/DISAGREES cross-check;
  `driveStation()` gate 1 takes signs from the table when signed (no capture needed), refuses on a
  capture that disagrees naming the port, prints the sign source; construction hands the table entry
  to `ProsMotor` as typed; the arithmetic is `tankSideVolts`; gate 5 runs through the monitor.
  `make ROBOT=tank` exit 0.
- **The Session 1 §4.2 trap, met twice and caught by the md5 check:** the first two host builds
  after the tests were added exited 2 at the doc-gate target (first `check-removability` — the
  monitor header's banner named `R3b-PROGRESS`, a public API page must not; reworded — then the
  briefing gate, dirty-tree drift) and the test binary was NEVER RELINKED (md5 unchanged), so a
  "4 passed" under my filter was four pre-existing cases. Regenerated `docs/api` and the briefing
  transiently; the third build relinked (md5 `5079a29a…`): **1188 cases / 1,540,950 assertions /
  0 failed / 3 skipped** (dirty tree). Saved as the green binary for the mutation cycle.
- Ruling change received (§2); `drive_program.hpp/.cpp` written to it; Makefile `PROGRAM` axis;
  `main.cpp` fourth beacon + `opcontrol()` switch + `#error` on `PROGRAM=drive` without tank;
  `project.pros` left as `"Bench Tests"`. `make ROBOT=tank PROGRAM=drive` exit 0, the two expected
  `-Wunused-function`s from `main.cpp`, `strings` on the hot ELF shows ONLY
  `shulib-robot-variant=tank-drive`. The three Makefile `$(error)` paths exercised with `make -n`:
  `ROBOT=bench PROGRAM=drive`, `ROBOT=xdrive PROGRAM=drive`, `PROGRAM=foo` all stop with the
  intended message; `ROBOT=tank PROGRAM=tester` builds.
- `tools/src_build_gate.py`: four builds, define SETS, `BUILD_ARGS`, NUL-terminated beacons,
  self-test cases 10 (PROGRAM append dropped → structural + both beacon detectors) and 11 (the
  real tank-drive build passes, i.e. the prefix hazard is closed). **`self-test`: OK, 23 detector
  cases (was 18). `check`: PASS, four builds, 3/3 TUs each, xdrive 0 `-Wunused-function` from
  `main.cpp`, bench/tank/tank-drive 2 each, hash asserted, beacons asserted.** Case 10 IS the
  coordinator's "run that mutation": the Makefile's `override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_DRIVE`
  line replaced by a comment, a REAL `make ROBOT=tank PROGRAM=drive`, and the verdict named the
  missing whole token `-DSHULIB_PROGRAM_DRIVE`, the tank-drive beacon NOT in the ELF, and the tank
  beacon present ("a tank build wearing a tank-drive label"); restored byte-exact by the plant
  class's own assertion. (Two vendor-ignored warnings per build, both
  `include/liblvgl/core/lv_obj_style.h:94` via the TUs that include the PROS API headers —
  path-ignored by policy, unchanged by this pass.)
- Docs: roadmap "You are here" gained the Part 0b paragraph; build-order's `**Next:**` block
  rewritten (robot two can be driven through the adapters; the motion stack still has not driven a
  robot; nothing of Part 0b has run on hardware); worksheet Station A (two programs, slot 1 note),
  D (table location, AGREES/DISAGREES rows), **Station E — DRIVE PROGRAM** (the two upload
  commands, the panel row by row, what a WARNING and a CUT mean, the ground-run rule), DO NOT;
  Daniel's note (two programs, both uploads, signed table in `chassis_table.hpp`, IMU port 2,
  step 3 for "shulib Drive"); `docs/README.md` 1,710 entities; `docs-publishing.md` 121 headers /
  123 files / 1,710; `ci.yml` and `RESUMING.md` say four builds; the Makefile's ROBOT comment no
  longer says the tank table "ships UNSET".
- **Prose review (pace rule 4):** grepped `docs/`, `README.md`, the Makefile and the tool header
  for `bench_r3a.cpp`'s table, "ships UNSET", "slot 3", "chooser", "dead-man", "Bench Tests";
  every hit outside the historical logs (R3a/R3b-PROGRESS, the Session 2 brief) was read and either
  fixed (listed above) or is a true statement about the tester, which still has its dead-man and
  its six gates. `hardware-assumptions.md` deliberately untouched: Part 0b settles no HA entry by
  measurement (nothing ran on hardware); the register entries Session 2 §9.6 owes are still owed.
  `docs/guide/14` untouched: still true.

#### 4.1 The drive loop, walked through (brief §4: PROS-bound, so its verification is the ARM build through the gate plus this)

Each guarding condition, quoted from `src/drive_program.cpp` as written:

- **Boot refusals, nothing powered:** `if (!kChassis.signsMeasured) { paintRefusal(...); return; }`
  — an unsigned table cannot drive (the bench variant compiles this TU and would refuse here);
  `if (describeMissing(kChassis, why, sizeof why, true))` and `if (!tableConsistent(kChassis, why,
  sizeof why))` likewise. All three come BEFORE `Member members[kMaxPort]` exists, so there is
  nothing to stop.
- **Construction inside a try per motor:** `try { m.motor.emplace(m.signedPort, gearset); ... }
  catch (const PreconditionError& e) { m.absent = true; m.absentWhy = "refused at boot ..."; emitf("WARNING:
  port %2d (%s, signed %+3d) DEAD AT BOOT -- adapter refused: %s", ...); }` — the loop continues
  (`++n` after the try either way), so a dead port is a member that is present in the table and
  absent on the robot, and the WARNING carries port + side. `m.signedPort = tableSignedPort(kChassis,
  p);  // AS TYPED; never negated here`.
- **The all-stop guard is declared before the first construction:** `AllStopGuard guard{members,
  &n};` immediately after `Member members[kMaxPort]; std::size_t n = 0;`, so a throw out of any later
  line (a controller adapter, an emplace that throws something other than `PreconditionError`)
  still runs `stopAll()` on every motor constructed so far.
- **The degradation call, at boot and again on a runtime absence:** `teleop::DegradationVerdict
  policy = evaluate(members, n);` → `if (policy.verdict == teleop::DriveVerdict::Refuse) {
  paintRefusal(headline, policy.reason, dead); return; }` (the guard stops what was constructed);
  later `if (newlyAbsent) { policy = evaluate(members, n); ... if (policy.verdict ==
  teleop::DriveVerdict::Refuse && !refused) { refused = true; stopAll(members, n); ... } }` and
  every subsequent tick `if (refused) vL = vR = 0.0;`.
- **The 10 ms loop through the shared mapping, no dead-man:** `const teleop::DriveRequest req =
  teleop::mapSticks(sticks);  // all-zero when disconnected` → `const teleop::SideVolts sv =
  teleop::tankSideVolts(req, kMaxDriveV);` with `constexpr double kMaxDriveV = 12.0;` and
  `pros::delay(kTickMs)` at `constexpr int kTickMs = 10;`. There is no button read in the loop.
- **Runtime absence:** `if (m.absence.update(m.motor->faultedReads(), m.v, cmd, matesFastest)) {
  m.absent = true; ... emitf("WARNING: port %2d (%s) went ABSENT at runtime: %s -- dropped from the
  fight detector, still commanded (harmless)", ...); }` — logged once because the detector latches
  and the branch runs only on the transition; `matesFastest` is the fastest of the OTHER present
  members; the member is still in the `setVoltage` loop (`if (members[i].motor)`), which is the
  brief's "its voltage still commanded (harmless)".
- **The fight cut and the over-current cut, 1 s, non-fatal, counted:** both live under `if
  (cutTicksLeft == 0 && !refused)`; the monitor's `verdict.persistedCount > 0` or a member's
  `m.overTicks >= kPersistTicks` sets `cutNow`; then `stopAll(members, n); cutTicksLeft =
  kCutTicks; ++cutCount;` with `constexpr int kCutTicks = 100;` (1 s at 10 ms) and the reason logged
  with a per-port snapshot. Absent members feed the monitor as `present = !m.absent &&
  m.motor.has_value()` and are skipped by the over-current loop.
- **The re-arm:** `if (cutTicksLeft > 0) { --cutTicksLeft; vL = vR = 0.0; if (cutTicksLeft == 0) {
  monitor[0].reset(); monitor[1].reset(); for (...) members[i].overTicks = 0; emitf("re-armed after
  cut #%d (1 s at 0 V)", cutCount); } }` — the streaks start over, so the same fight must persist
  the full 250 ms again before it cuts again.
- **The LCD writes only on change, at most 5 Hz:** `if (tick % kLcdEveryTicks == 0)` with
  `constexpr int kLcdEveryTicks = 20;` (200 ms) and `if (std::strcmp(want[row], lcdRow[row]) != 0)
  { lcd.setLine(row, want[row]); ... }`.
- **Logging with the no-card rule:** `static hal::pros::ProsBlockSink card{"drive_log.txt"};` then
  `emitf("sd logging : %s", card.isOpen() ? "ON -> /usd/drive_log.txt ..." : "OFF -- no card at
  boot; serial only (T5: never stop for a missing card)");` — said once at boot, and `emit()` writes
  to the card only `if (g_card != nullptr && g_card->isOpen())`.
- **Every exit leaves 0 V, coast:** the field-disable poll `if (pros::c::competition_is_disabled())
  break;` is the first statement of the loop body; after the loop `stopAll(members, n);` runs
  explicitly and the guard's destructor runs it again on scope exit. The header states plainly
  that a task KILL runs no destructor and that VEXos's own motor disable is what covers that case.

### 5. Mutations — each run, red OBSERVED verbatim, restored byte-exact, suite re-run green

Protocol (Session 1 §4.2, made mechanical in a scratchpad driver): pristine copies of the three
mutable files saved with their md5s; per mutation — plant (asserting the target occurs exactly
once), regenerate `docs/api` transiently (the API pages render the constexpr bodies, so
`check-fresh` would otherwise block the relink), put the saved GREEN binary back so the briefing
gate sees green, `cmake --build`, **assert exit 0 AND a changed binary md5**, run the target
cases, record the red; restore by copying the pristine file back and **assert the md5 equals the
pristine's**, regenerate, rebuild, re-run the cases green. Green binary at the start of every
cycle: md5 `5079a29a…`. Every mutant below BUILT (a new md5 each time) and every restore was
byte-exact (`1460672…` / `b39abd8…` / `68d3a7a…` after each).

| # | Mutation (file) | Mutant md5 | Observed red (verbatim doctest lines, first few) | After restore |
|---|---|---|---|---|
| M1 | degradation floor `< kMinAnsweringPerSide` → `< kMinAnsweringPerSide - 1` (the brief's `< 3` → `< 2`), both sides | `9472f7a1` | `degradation:*` 4 passed / **1 failed**, 49 assertions / **2 failed**: `drivetrain_degradation_test.cpp:84: CHECK( eval(4, 2, 4, 4).verdict == DriveVerdict::Refuse ) is NOT correct!`, `:86: CHECK( eval(4, 4, 4, 2).verdict == DriveVerdict::Refuse ) is NOT correct!` | 5/5, 49/49 green |
| M2 | degradation cap `> kMaxDeadTotal` → `> kMaxDeadTotal + 1` (`> 2` → `> 3`) | `6ee611d1` | 3 passed / **2 failed**, 49 / **5 failed**: `:65: CHECK( threeTotal.verdict == DriveVerdict::Refuse )`, `:71: CHECK( threeTotalMirror.verdict == … )`, `:99: CHECK( eval(5, 3, 5, 4).verdict == … )`, `:100: CHECK( eval(5, 4, 5, 3).verdict == … )`, `:103: CHECK( eval(6, 3, 6, 6).verdict == … )` all `is NOT correct!` | 5/5, 49/49 green |
| M3 | monitor: drop the persistence (`ticks_[i] >= cfg_.persistTicks` → `>= 1`) | `e397846a` | `side monitor:*` 5 passed / **4 failed**, 82 / **7 failed**: `coupled_side_monitor_test.cpp:91: CHECK( at24.persistedMask == 0 )`, `:92: CHECK( at24.persistedCount == 0 )`, `:110`, `:141`, `:200`, `:205: CHECK( run(mon, 6.0, m, 24).persistedMask == 0 )`, `:257: CHECK( worstPersisted == 0 )` (the stick-reversal case) all `is NOT correct!` | 9/9, 82/82 green |
| M4 | monitor: drop the present-flag check (`if (!v.evaluated \|\| !m.present)` → `if (!v.evaluated)`) | `b1e25fa7` | 8 passed / **1 failed**, 82 / **4 failed**: `:220: CHECK( v.disagreeingMask == 0 )`, `:221: CHECK( v.persistedMask == 0 )`, `:222: CHECK( mon.disagreeTicks(1) == 0 )`, `:223: CHECK( mon.disagreeTicks(4) == 0 )` — the ABSENT-member case: a dead port flagged | 9/9, 82/82 green |
| M5 | monitor: flip the sign test (`(m.velocityRadS * expected) < 0.0` → `> 0.0`) | `fde47886` | **0 passed / 9 failed**, 82 / **46 failed**: from `:69: CHECK( v.disagreeingMask == 0 )` (the all-agree case flags every member) onward | 9/9, 82/82 green |
| M6 | `tankSideVolts`: swap ∓ | `0ac55231` | `side volts:*` **0 passed / 1 failed**, 20 / **14 failed**: `:301: CHECK( ccw.left == 12.0 * (0.5 - 0.25) )`, `:302`, `:303: CHECK( ccw.left == 3.0 )`, `:304: CHECK( ccw.right == 9.0 )`, `:307: CHECK( cw.left == 9.0 )`, `:308`, `:311: CHECK( spin.left == -12.0 )`, `:312: CHECK( spin.right == 12.0 )` all `is NOT correct!` | 1/1, 20/20 green |
| M7 | chassis table: remove the cross-side duplicate check | `6b1c7da4` | `chassis table:*` 7 passed / **1 failed**, 81 / **5 failed**: `chassis_table_test.cpp:145: CHECK_FALSE( consistent(t, why, sizeof why) )`, `:146: CHECK( std::strstr(why, "BOTH sides") != nullptr )`, `:147`, `:150: CHECK_FALSE( consistent(same, …) )`, `:151` all `is NOT correct!` | 8/8, 81/81 green |
| M8 (extra) | absence detector: `faultStreak_ >= persistTicks` → `>` | `5903fb18` | `absence detector:*` 1 passed / **1 failed**, 2253 / **53 failed**: `:341: CHECK( d.update(++faulted, 8.0, 6.0, 8.0) )` (the 25th tick), `:342: CHECK( d.absent() )`, `:343`, `:345 ×50` all `is NOT correct!` | 2/2, 2253/2253 green |
| M-gate | Makefile `override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_DRIVE` replaced by a comment, real `make ROBOT=tank PROGRAM=drive` (self-test case 10) | (ELF) | verdict FAIL naming `whole token -DSHULIB_PROGRAM_DRIVE … silent no-op`, `beacon 'shulib-robot-variant=tank-drive' is NOT in bin/hot.package.elf`, and `carries the tank beacon … a tank build wearing a tank-drive label` — all three asserted by the self-test, which passed (23 cases) | restored by the plant class's byte-exact assertion; `check` PASS ×4 |

**D5-standard notes (a red for the wrong reason is a fake red):**
- **M1's red is real but narrower than the brief's row list suggests.** On a FIVE-per-side train,
  "2 answering" is 3 dead, and the total cap (`> 2`) refuses that row on its own — so the
  `2/5+5/5 REFUSE` row stayed green under M1, correctly, and only the bench bot's four-per-side
  rows (`2/4`) exposed the floor. The per-side floor is independently load-bearing only where
  `expected − 2 < 3`, i.e. four motors or fewer per side; on robot two the cap alone yields the
  same verdicts. The test file's four-per-side rows are therefore not decoration — they are the
  only thing that pins the floor — and this is recorded so nobody later "simplifies" them away.
- M5 red on every case because the flipped test flags AGREEING members; that is the intended
  reason (the sign test), not a compile artefact — the mutant built and the disagree masks are
  what failed.
- Every mutant's red came from the TEST's assertions, never from a doc gate: the transient
  regenerate + green-binary steps kept the gates out of the way, as Session 1 §4.2 prescribes.

### 6. Final verification (all from the repo root, on the finished working tree, HEAD `b8b63f9`)

| Check | Result |
|---|---|
| Host build (all doc gates in-build) | exit 0, binary md5 `5079a29a…` |
| Host suite | **1188 cases / 1,540,950 assertions / 0 failed / 3 skipped** — DIRTY-tree count (`-dirty` in the hash string; the committed briefing must carry the CLEAN-tree number, measured by the coordinator after the commit — Session 1 §6) |
| `api_doc_tool.py` self-test / check-coverage / check-fresh / check-examples / check-removability | **5 × PASS**; coverage **1,710 entities / 121 headers**, all documented; 387 quoted example lines verbatim; removability clean |
| `briefing_status.py check` | PASS — but against a briefing regenerated on the dirty tree (suite line 1,540,950; headers 154). Will read as drift after the commit until regenerated clean: the known caveat |
| `doc_staleness_audit.py` self-test + audit | **2 × PASS** (10 detector cases; 26 live docs, 0 numeric claims) |
| PROS-free guard (ci.yml, verbatim) | CLEAN — `include/shulib` PROS-free outside `hal/pros/`; the two new headers included |
| Sim-layering guard (ci.yml, verbatim) | CLEAN |
| ARM header cross-compile (anchored sed, repo root, `-Werror`) | **CLEAN, 154 headers** (was 152) |
| `src_build_gate.py self-test` | **OK, 23 detector cases** (was 18), each a real build, incl. the PROGRAM-define-dropped mutation |
| `src_build_gate.py check` | **PASS ×4** — bench / xdrive / tank / tank-drive, **3/3 TUs** each (the gate globs `src/`; no change was needed to count three), xdrive **0** `-Wunused-function` from `src/main.cpp`, the others 2, hash asserted, beacons asserted NUL-terminated |
| Makefile validation | `ROBOT=bench PROGRAM=drive`, `ROBOT=xdrive PROGRAM=drive`, `PROGRAM=foo` all `$(error)` with the intended text |
| C7 removability grep of public docs | empty |
| git | nothing staged, nothing committed by the executor; 18 modified + 10 new files (listed by `git status` in the scratchpad log) |

### 7. WHAT IS NOT DONE — read this before believing any checkbox

- **NOTHING HERE HAS RUN ON HARDWARE.** Neither program has been uploaded since this pass; no
  motor has yet turned under the library's adapters; the drive program's panel, LCD, cuts,
  degradation and absence detection have been exercised only on the host (the pure parts) and
  by the ARM build (the loop). The coordinator uploads: `make ROBOT=tank PROGRAM=drive && pros
  upload --slot 1 --name "shulib Drive"`, then `make ROBOT=tank && pros upload --slot 3`.
- **The motion stack has still not driven a robot.** The drive program is adapters and open-loop
  volts; `docs/guide/14` stays as it is; M1's badge has not moved; HA-18/52/112 unsettled.
- **The brief's DoD items that the ruling change withdrew** — the 3 s chooser and the
  `project.pros` rename — are deliberately NOT done; a separate program in slot 1 replaces them
  (§2, and the amended brief at `b8b63f9`).
- **The clean-tree suite number is not known yet** (dirty-tree 1,540,950; the six `-dirty`
  characters shift the hash-derived assertions). The coordinator re-measures after the commit and
  regenerates `PROJECT-BRIEFING.md` (Session 1 §6's rule).
- **`hardware-assumptions.md` untouched** — no measurement happened; the register rows Session 2
  §9.6 owes (VEXos 1.1.5, the controller-launch trap, USB renumbering, the coupled-member
  thresholds as invented) are still owed. The thresholds are now stated once in
  `SideMonitorConfig` and pinned by tests, which is where a register row can point.
- **Robot two's committed table is checked for consistency only at boot** (`tableConsistent()` in
  both programs) and by the src build gate compiling it; the host test re-types the measured
  values rather than including the tank variant (D0b-2). A drift between the header and the
  test's copy would show as a test failure only if someone edits the test, or on the brain at
  boot. Acceptable tonight; R3d replaces typed signs with discovered ones.
- **The degradation policy's domain** (D0b-5): a drivetrain with fewer than three motors per side
  is refused on every boot. Not this robot's problem; stated in the header.
- Parts 1–3 (motor group, odometry seam, composition root) are not started.

**PART 0B READY FOR VERIFICATION**

