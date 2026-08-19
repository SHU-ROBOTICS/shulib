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
