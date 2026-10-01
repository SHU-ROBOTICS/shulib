# DIAG1 — progress log (live)

- **Chunk:** DIAG1 — an implausible odometry tick names its check and its values (Part A only)
- **Brief:** `DIAG1-odometry-tick-names-its-check.md` (committed at `ea58fbc`)
- **Started:** Sat Sep 26 2026 (see the first log entry for the clock)
- **Tree at start:** `ea58fbc`, clean, in the git worktree `.claude/worktrees/diag1` (branch
  `diag1`) — NOT the main working tree. Parallel builders: T3 (teleop/, robot_config.hpp,
  robot.hpp), COMP-a (run_guard.hpp, localizer.hpp re-seat member, robot.hpp, competition/, ui/),
  G1 Part 2c.
- **Executor model:** opus
- **Go:** the team lead, 2026-09-26: "Yes, as recommended" (the brief's header); run by the
  coordinator's workflow. Do not commit; no robot upload; no `pros` commands; no `/dev/ttyACM*`.
- **Scope of this run:** Part A only (brief §1 items 1-6, §2, §3). Part B (the median's non-finite
  handling) is OUT. HA numbers: ONLY HA-160 (new) plus HA-123's amendment. The
  `R3d-PROGRESS.md` correction the brief asks for is IN.

Appended in real time. **Nothing in this chunk has run on a robot.**

---

## Log

### 09:35 — start
- `git status` clean; HEAD `ea58fbc`. Read `docs/internal/RESUMING.md` (the chunk loop, the
  standards, the pace rules) and the brief in full. This log created before any code.

### 09:36-10:05 — required reading; baseline; the capture re-read myself (not taken from the brief)
- **Read in full:** `localization/odometry.hpp`, `drive_encoder_odometry.hpp`, `pilons_odometry.hpp`,
  `diag/health_monitor.hpp`, `diag/fault.hpp`, `diag/line_format.hpp`, `motion/motion.hpp`
  (`tickHealthObservables`), `localization/localizer.hpp` (the C1 forwarder ~:452-458),
  `hal/motor_group.hpp` (the median), `chassis/chassis.hpp` (`drive()`), the scheduler's tick body and
  abort policy, and the three test files the chunk extends (`drive_encoder_odometry_test.cpp`,
  `pilons_odometry_test.cpp`, `health_monitor_test.cpp`) plus `motion_test_rig.hpp`.
- **Confirmed from the tree:** the gate sub-expressions are exactly the brief's (DEO :164-166, Pilons
  :143-150); the ONLY OdoStuck raise site is `HealthMonitor::tick` (:153-162) with the fixed text;
  `FaultLatch::raise` formats into `char buf[176]` (:138-141) — prefix worst case for ODO_STUCK is
  `fault=ODO_STUCK` (15) + ` n=` (3) + 10 digits + ` FIRST` (6) + ` ` (1) = 35, so the detail may be
  at most 175 - 35 = **140** bytes. `appendNum(line, v, 0, 3)` renders a finite value unpadded and
  compacts anything longer than 10 bytes to `%.3g`, so every `<n>` is at most 10 bytes (e.g.
  `-99999.999`, `-9.99e+307`); NaN/+Inf/-Inf are 3-4. `Chassis::drive()` localizes then calls
  `tickHealthObservables` and does NOT run the scheduler's PoseDeltaGuard (so a drive()-only e2e sees
  ODO_STUCK alone); a scheduled motion does run it (IMPLAUSIBLE will ride beside ODO_STUCK there).
- **Baseline (worktree, `ea58fbc` + this log + the regenerated briefing):** the doc gate first failed on
  the STALE briefing (this new `-PROGRESS.md` makes DIAG1 an open chunk) — regenerated with
  `briefing_status.py generate`, then `cmake --build build/test -j6` (the "Linking CXX executable
  shulib_tests" line present) and the suite: **1464 cases, 2,387,533 assertions, 3 skipped, SUCCESS.**
- **The 2026-09-25 capture, re-read from the evidence file myself** (`zcat
  docs/internal/evidence/r3d-session-2026-09-25-teleop.log.gz | strings` → 428,276 lines): episode 1
  is strings line **102381** (`[t=  34.40] ... fault=ODO_STUCK n=1 FIRST implausible odometry tick`),
  episode 2 strings line **110158** (`[t=  60.32] ... n=2`). A record's `v(...)` is the COMMANDED
  speed in the FIELD frame (`term_sink.hpp` :142-147 renders `r.commanded`; `Chassis::drive` stamps
  `robotToField(out.body, pose.heading())`), so for a tank reversing with zero body lateral the
  published heading is `atan2(-vy, -vx)`. Reconstructed (script in my scratchpad, not the tree):
  - **Episode 1:** the last non-zero command is t=34.01 (a gentle forward at ~5.7 in/s, before it
    ~13 in/s); commanded zero for the 0.39 s before the fault. "At rest" (commanded) holds; the
    heading cannot be reconstructed from zero commands.
  - **Episode 2:** reversing from t=59.97 to t=60.31 (0.35 s, strings lines 110053-110155), |v| up
    to 34.0 in/s (vx down to -33.8); the fault fires on the FIRST zero-command record (t=60.32).
    Consecutive-record heading swings in the last 0.09 s: +16.9, -19.7, +17.6, -17.1, +18.5, -18.2,
    +24.1, **-36.9°** (t=60.23-60.31) — the brief's "17-37°" reproduces. **[CORRECTED in review
    round 1: this list skipped one interval — t=60.27→60.28 is -1.4°; the nine intervals t=60.22→60.31
    are +16.9, -19.7, +17.6, -17.1, +18.5, -1.4, -18.2, +24.1, -36.9. "17-37° between consecutive
    records" overstated it; see R3 below.]** 37° in 10 ms is ~3700°/s:
    not a robot. This is an INFERENCE (it assumes zero body lateral and that the stamped heading is
    the published one), and it is recorded as such.
  - So `R3d-PROGRESS.md`'s "Both ODO_STUCK ticks came with commanded speed 0 ... i.e. not from
    driving" is right for episode 1 and WRONG for episode 2 — corrected in this chunk's doc pass.

### 09:49 — the production change (brief §1 items 1-5), existing suite green
- **NEW `include/shulib/diag/odom_tick_audit.hpp`** (a diag leaf: includes only `line_format.hpp` and
  `units/quantity.hpp`): `OdomTravelLabels`, `OdomTickAudit` (three flags, five values, labels,
  `constexpr tripped()`), `kOdomAuditDetailMaxBytes = 140`, `kOdomAuditPrecision = 3`,
  `appendOdomTickAudit(Line&, const OdomTickAudit&)` on `lineformat::appendNum(v, 0, 3)` only.
  **One design choice beyond the brief's letter:** the `why=UNREPORTED` branch lives INSIDE
  `appendOdomTickAudit` (an untripped audit renders ` why=UNREPORTED` and no values), not in
  HealthMonitor. Same text the brief specifies; the reason is that the formatter is then total — no
  caller can produce a malformed ` why= dth=...` with an empty token list. HealthMonitor just calls it.
- **`IOdometry`** gains the pure virtual `lastTickAudit()`; the "exactly these four members" prose is
  rewritten as "these five" with THE FIFTH MEMBER paragraph (why). Both implementations fill
  `audit_` every `update()` from the gate's own sub-expressions and then `implausible_ =
  audit_.tripped()`. DEO: a = dL, b = dR, DriveSides; Pilons: a = RAW `fwdDelta`, b = `latDelta`,
  TrackingWheels. Pilons' stale accessor comment (~:185, "oversized Δθ OR non-finite") and its
  header's TRUST GATE block (listed two reasons; the travel half arrived at DEFECTS1) both fixed.
- **`Localizer::lastOdomTickAudit()`** — ONE forwarder, directly under `lastOdomDeltaImplausible()`;
  no include added (odometry.hpp brings the type), to keep the COMP-a merge local.
- **`HealthMonitor::Observations::odomAudit`** APPENDED last; the rising-edge branch builds the text
  in a `lineformat::Line` only when `odomImplausible` rose (stall alone keeps its old text; both
  together = the valued text + ` +STALL`); the trigger is untouched. `tickHealthObservables` sets
  `.odomAudit = loc.lastOdomTickAudit()` last. The test's `ScriptedOdometry` implements the new member.
- `api_doc_tool.py generate` (149 headers, 2615 entities, all documented), briefing regenerated
  (header count moved), fresh build ("Linking CXX executable shulib_tests" present): **1464 cases,
  2,387,536 assertions, 3 skipped, SUCCESS** — every pre-existing trust-gate test green, which is the
  pin that the verdict did not move. (+3 assertions with no test added. First written here as "the
  doc-fidelity tests walking the API pages" — a GUESS, and wrong. Measured instead: the `ea58fbc`
  tree built in my scratchpad (`git archive`), both binaries run with `-r=xml`, per-test counts
  diffed: the ONLY suite-wide change is `pros_adapter_fence_test.cpp`'s recursive scan of every
  `include/shulib` header (498 -> 501: three checks per header, one new header). The scratchpad tree's
  build-hash test differs by 5 only because a `git archive` has no git identity.)

### 09:59 — the tests: `test/odom_tick_audit_test.cpp` (31 cases), green
- **One new file**, sections A-F (its header lists them): A formatter (goldens, token order, fixed
  non-finite tokens incl. a NEGATIVE NaN, compaction, a 20,000-audit adversarial sweep asserting <= 140
  bytes and printable 7-bit ASCII, the widest number = 10 bytes, the constructed worst case = 137 bytes,
  the real latch carrying it untruncated); B DriveEncoderOdometry (rotation-only exact bits, left-only /
  right-only, both bounds at exact equality and one ulp past, NaN left = NONFINITE+TRAVEL + L=NaN +
  frozen, 1e9 = TRAVEL only + integrated + `1e+09`, DBL_MAX x 2 in/rad = +Inf = NONFINITE+TRAVEL,
  ROT+TRAVEL then a clean tick clearing everything, a **2000-tick sweep against an oracle built from
  the script alone** — verdict == tripped() AND every flag and value, every tick; it exercised rot 95 /
  travel 139 / nonfinite 21 / clean 1768 — and MotorGroups: one member of four jumping 1000 rad is not
  implausible, two jumping gives TRAVEL with L = 500 = the median); C PilonsOdometry (the same set, the
  sweep's non-finite ticks from an OVERFLOW pair since a NaN pod poisons its TrackingWheel for good;
  rot 90 / travel 143 / nonfinite 20 / clean 1770; and **C7: the RAW forward delta** — a 40 in pod
  5 in off centre on a -60 deg tick corrects to ~34.76 in, under the bound, so a centre value on the line
  would not explain the fault); D HealthMonitor (the whole Error line as a golden; 500 changing audits =
  one line with tick 1's values, then a clean tick and a new episode = `n=2` with its own; empty audit =
  `why=UNREPORTED`; flags without the verdict = nothing, 50 ticks; +STALL together, and stall-first then
  implausible = one line with the stall text); E the Localizer forwarder bit for bit (-0.0, a denormal,
  -Inf, a NaN with payload 0xBEEF) and `tickHealthObservables` carrying the field; F end to end on a
  **drive-encoder tank on the host test plant** (robot two's shape; the full Localizer / latch / monitor /
  scheduler / Chassis): `Chassis::drive` at rest with the IMU stepped +100 deg = exactly one ERROR line
  equal to the golden `... why=ROT dth=1.745rad h0=0.000 h1=1.745 L=0.000in R=0.000in`; +/-100 deg
  alternating for 50 ticks = 51 implausible ticks, ONE line, the first tick's values; a MoveToPose with
  the step at 0.5 s = Cancelled + abortFault ODO_STUCK (FAULT_ABORT) before 1.0 s, its line
  `fault=ODO_STUCK n=2 implausible odometry tick why=ROT dth=1.745rad h0=0.000 h1=1.745 L=0.384in
  R=0.384in` (n=2: the scheduler's pose-delta guard raised IMPLAUSIBLE first on the same tick).
- **A test bug found by running it:** B3/C3's first draft stepped 36 then 36 + nextafter(36) from the same
  baseline — `36 + 36.000000000000007` rounds back to exactly 72, so the second tick's travel was 36.0
  and the "one ulp past" case never happened. Rewritten from a ZERO baseline per case, so the travel IS
  the reading.
- Full suite, fresh build: **1495 cases (+31), 2,455,747 assertions, 3 skipped, SUCCESS.**

### 10:07 — the doc pass (while the mutation batch runs; docs only, no header touched mid-batch)
- `docs/guide/11-reading-the-diagnostics.md`: the ODO_STUCK row rewritten (two causes named by the
  line; the trust-gate causes ROT / TRAVEL / NONFINITE; "check the tracking wheels" was WRONG for a
  drive-encoder robot, which cannot run the stall check at all — said so); the fault-lines bullet points
  at a NEW section "Reading an ODO_STUCK line" (the golden line, every token and field, radians with
  1.571 = 90 deg, the default bounds pi/2 and 36 in, NaN/+Inf/-Inf and compaction, UNREPORTED, +STALL,
  one line per episode with the FIRST tick's values); the chapter's own claim "every example is copied
  from output the suite pins byte-for-byte" now names `test/odom_tick_audit_test.cpp` (D1 pins it).
- `docs/guide/12-when-things-go-wrong.md`: ~:127-128 "ODO_STUCK means it *should* have frozen" was
  STALE — only a NONFINITE tick freezes; a ROT/TRAVEL tick is integrated, so the pose can JUMP. Fixed in
  both bullets ("The estimate jumps" / "The pose froze"); the FAULT_ABORT bullet now sends the reader to
  the `why=`; the heading-wrong bullet names `why=ROT` and HA-160. The chapter's "simulation symptoms"
  (:25) was not edited, so not renamed.
- `docs/guide/06-how-things-fail.md` ~:44-52: told ODO_STUCK's story as the stall cross-check ONLY —
  stale since C1 (the trust gate raised it too) and misleading for robot two, where the gate is the
  only trigger. Added the second-trigger paragraph; the edited paragraph's "In simulation" became "On
  the host test plant".
- `docs/diagnostics-plan.md`: a dated note on the legacy which-wheel bullet — the per-tick account is a
  NARROWER thing than the E-phase which-wheel detector, which is still owed (NOT claimed).
- `docs/hardware-assumptions.md`: status count 146 -> 147 with the DIAG1 sentence (HA-160 numbered past
  T3's HA-147..159; "the total is 160 once both are in"); index rows HA-123 (AMENDED) and HA-160 (NEW);
  the HA-123 entry extended to drive sides and its blast radius CORRECTED ("noisy, not dangerous" was
  false while ODO_STUCK aborts by default); the HA-160 entry placed after HA-123 in Group R4 (away from
  where T3 appends), evidence cited by DATE only (C7).
- `docs/changelog.md`: NOT in the brief's list — added anyway, because its own contract is "every
  user-visible change" and `IOdometry::lastTickAudit()` is a new PURE virtual (source-breaking for an
  out-of-tree implementer; not frozen, so no bump). Flagged as a deviation in the report.
- `docs/internal/chunks/R3d-PROGRESS.md`: the 2026-09-25 paragraph corrected in place, the old words
  quoted, with the strings line numbers (102381, 110158, 110053-110155) and the reconstruction.

### 10:09 — found while reading (logged, NOT fixed — out of this chunk's scope)
- **The host test loops' ready-gate divergence** (the brief's named item): `sim_hostile_survival_test.cpp`
  :137 and `sim_hostile_composed_test.cpp` :375 feed `.odomImplausible = h.imu().isReady() &&
  odom.lastDeltaImplausible()`, with a comment calling that "the C1 loop shape" ("boot garbage tripping
  ODO_STUCK would bury real root causes"). The library's own `motion::tickHealthObservables` passes
  `loc.lastOdomDeltaImplausible()` UNGATED. So the loops the hostile suites trust do not match the
  path every program runs: on a robot, a heading swing over pi/2 during the IMU's calibration window
  can raise ODO_STUCK through `drive()` or the idle tick, which no host test models. Not a cause of
  2026-09-25's two faults (t=34 s and t=60 s, the IMU long ready). Owner: the coordinator to route.
- **`TrackingWheel` accepts a non-finite offset and a +Inf diameter** (its only check is
  `wheelDiameter > 0`). Relevant here because it is the ONE way a PilonsOdometry tick can be
  NONFINITE without also being TRAVEL (see the M05a finding below). Not fixed.
- Other ODO_STUCK callers that bypass `tickHealthObservables` exist only in tests (the hostile loops
  above, `health_monitor_test.cpp`); they leave `odomAudit` default, so their lines would read
  `why=UNREPORTED` — honest, and none asserts on the text. `src/` has no direct caller (grep:
  `odomImplausible` / `HealthMonitor` tick sites), so no program needed a change, as the brief said.

### 10:13 — the mutation table (each on a FRESH binary, observed, restored)
Runner (my scratchpad, `diag1-mut/run_mutation.sh`): apply exactly one textual mutation (assert one
occurrence), `touch test/*.cpp`, `cmake --build build/test --target shulib_tests/fast -j6`, REQUIRE the
"Linking CXX executable shulib_tests" line (every run below: `linked=1`), run the whole suite, list the
FAILING cases (parsed from doctest's ERROR blocks, not from MESSAGE headers — the first draft of the
runner counted MESSAGE output and was fixed before any result was recorded), restore from a saved copy
and `cmp` it (every run: restored). Baseline: 1495 cases green.

| # | Mutation (file) | Result | Red cases (DIAG1 ids; others by name) |
|---|---|---|---|
| M01 | rotation/travel flags swapped (DEO) | **RED 11** | B1 B2 B3 B4 B5 B6 B8 B9 F1 F2 F3 |
| M02 | travel re-derived as `|dL| > max || |dR| > max` (DEO) | first run BUILD-FAILED (-Werror unused `travelSane` — not a result); re-run as M02b with `static_cast<void>(travelSane)`: **RED 2** | B4 B8 |
| M03 | L/R swapped in the audit (DEO) | **RED 8** | B2 B3 B4 B5 B6 B7 B8 B9 |
| M04 | audit not reset — flags OR-accumulate across ticks (DEO) | **RED 4** | DriveEncoderOdometry 12, B7 B8 F2 |
| M05a | verdict computed apart, `!finite` dropped (DEO) | SURVIVED in this batch, and called an equivalent mutant (below) — **WRONG, corrected in review round 1**: B10 kills it, RED on a fresh binary | none (then) |
| M05b | verdict computed apart, rotation term dropped (Pilons) | **RED 6** | C1 C3 C8, "Localizer: an implausible odometry tick is reported Degraded", two PilonsOdometry Δθ tests |
| M05c | verdict computed apart, rotation term dropped (DEO) | **RED 7** | DriveEncoderOdometry 12, B1 B3 B8 F1 F2 F3 |
| M05d | verdict computed apart, travel term dropped (DEO) | **RED 4** | DriveEncoderOdometry 12, B3 B8 B9 |
| M06 | Pilons reports the offset-corrected centre value | **RED 2** | C7 C8 |
| M07 | raise outside the rising edge (every bad tick) | **RED 4** | HealthMonitor ODO_STUCK test, D2 D5 F2 |
| M08 | trigger on `odomAudit.tripped()` | **RED 6** | two HealthMonitor tests, D3 D4, composed catastrophic, survival +inf sentinel |
| M09 | `snprintf "%.3f"` in place of appendNum (formatter) | **RED 7** | A3 A4 B4 B5 B6 C4 C5 |
| M10 | compaction removed from `lineformat::appendNum` | **RED 7** | A3 A4 B5 C5, line_format A6, result-line tokens, TermSink compaction |
| M11 | Localizer forwarder returns `{}` | **RED 5** | E1 E2 F1 F2 F3 |
| M12 | `tickHealthObservables` drops `.odomAudit` | **RED 4** | E2 F1 F2 F3 |
| M13a | travel bound `<=` -> `<` (DEO) | **RED 2** | DriveEncoderOdometry 12, B3 |
| M13b | rotation bound `>` -> `>=` (Pilons) | **RED 2** | C3, "PilonsOdometry: Δθ exactly at the gate threshold" |
| M14 | +STALL precedence inverted (stall text wins) | **RED 2** | A4 D5 |
| M15 | Pilons labels DriveSides | **RED 5** | C1 C2 C4 C5 C7 |
| M16 | h0/h1 swapped (DEO) | **RED 3** | B1 F1 F2 |
| M17 | token order ROT/TRAVEL swapped (formatter) | **RED 3** | A2 B7 C6 |
| M18 | UNREPORTED branch removed (formatter) | **RED 2** | A2 D3 |

**[CORRECTED in review round 1 — the proof below is FALSE: it misses an overflow on the way to the
field delta (the drive-encoder sum dL + dR, arcStep's rotation of the pod pair, dTheta * offset). The
real condition, and the tests that now kill M05a, are under "Review findings and their disposition", R1.
The original text stands below as written.]** M05a is an equivalent mutant, and that is a finding worth keeping. In BOTH odometries the
`!finite` term never changes the verdict: the field delta is a bounded function of the travel values
(DEO: `0.5 (dL + dR)`; Pilons: `fwd + dth*offset`, `lat - dth*offset`, with dth always finite since
an `Angle` cannot hold NaN, and arcStep's chord factor and rotation both at most 1), so it can only be
non-finite when a travel value is NaN/Inf or already beyond the (finite, precondition-checked) bound —
i.e. when `travel` is already true. No test of the verdict can catch its removal. The one exception is
a PilonsOdometry built with a NON-FINITE offset (TrackingWheel does not check — logged above), where a
tick could be NONFINITE without TRAVEL. The NONFINITE flag still earns its place: it is the only one
that says position FROZE that tick. Written into `odom_tick_audit.hpp`'s header so no one re-derives it.
Every other brief-listed mutation is RED; M05's "one term dropped" is shown RED for every
non-redundant term (M05b, M05c, M05d).

### 10:27 — post-mutation header notes, the prose review (logged AFTER it ran), the gates
- **Header notes, written only after the batch finished** (the runner edits these files):
  `odom_tick_audit.hpp` gains "A FINDING" (the M05a proof); `pilons_odometry.hpp`'s class doc names the
  fifth member, and its HEADING IS IMU-OWNED item — like `drive_encoder_odometry.hpp`'s — now cites
  **A4 register HA-160** (the register names both files as consumers, so the reconciliation grep finds
  the ID there); **[corrected in review round 1: "like drive_encoder_odometry.hpp's" implied that one
  already existed — it did not; its :25-28 HA-160 citation was ALSO written here, after the batch
  (comment only; `M05d.orig`, saved by the runner at 10:12, lacks it), so the mutation results
  stand]**; `motion.hpp`'s Faults & health list gains `odomAudit` (and `groupMembersDisagreeing`,
  missing since R3b Part 1); `localizer.hpp` :6 "depends on nothing past its four members" -> "five"
  — ONE WORD beyond the one-forwarder instruction, because it became false with this chunk; far from
  the forwarder, flagged for the COMP-a merge.
- **Prose review** (grepped `docs/` and `README.md`, generated `docs/api/` and `docs/internal/` excluded,
  for: odometry.hpp, drive_encoder_odometry, pilons_odometry, health_monitor, localizer.hpp,
  motion/motion.hpp, line_format, odom_tick_audit, IOdometry, "four members", lastDeltaImplausible,
  lastOdomDeltaImplausible, tickHealthObservables, "implausible odometry", "trust gate",
  HealthMonitor::Observations, odomImplausible, maxTickTravel, maxTickRotation, ODO_STUCK; every hit read):
  - FIXED: guide 11, 12, 06 and diagnostics-plan (above); `shulib-v2-master-plan.md` :378 "Trust gate +
    finiteness" listed two checks — the travel half (2026-08-15, verified: `190c758`) was missing; a dated
    parenthesis adds it, the audit and HA-160. `faq.md` :25 said a held value is caught by the
    wheels-spin cross-check — not on a drive-encoder robot (HA-131); added, with the honest limit (a jump
    under 36 in passes). `hardware-assumptions.md`'s LINE citations into files this chunk shifted:
    `health_monitor.hpp:66–69` -> `:93–96` and `:70–71` -> `:98` (both were already stale before this
    chunk), `pilons_odometry.hpp:29–34` -> `:33–39`; `localizer.hpp:81–87`/`:75–78` untouched (above the
    forwarder, unmoved).
  - READ, NOT EDITED: `roadmap.md` :1436-1444 ("the four members the Localizer actually uses") is the dated
    R3b Part 2 delivery record — true of what was built then; the roadmap is the coordinator's.
    `guide/03` :404 and `guide/13` :109 link the pages/headers only; `guide/15` :58, `guide/07` :83,
    `master plan` :860 name ODO_STUCK generically; `faq.md` :179-180's three PROS function names are the
    doc-staleness audit's pre-existing report-only notes.
- **Gates, in the worktree, final state:** fresh full build ("Linking CXX executable shulib_tests"; the doc
  gates inside it: coverage 2615 entities / 149 headers, 408 example lines verbatim, removability clean)
  and the suite **1495 cases, 2,455,747 assertions, 3 skipped, SUCCESS**; `api_doc_tool` self-test OK,
  check-coverage OK, check-fresh exit 0, check-examples OK, check-removability OK; CI greps: PROS-free,
  sim-free; diag stays a leaf (no localization/motion/chassis/sim include); reconciliation direction 1
  empty; ARM header gate (185 headers, arm-none-eabi-g++ -Werror ... softfp) exit 0;
  `src_build_gate.py` self-test OK (33 cases) and check PASS (6 builds link, 0 disallowed warnings);
  `doc_staleness_audit.py` clean + self-test OK; added code string literals: 0 non-ASCII.

### 10:28 — where this run stops (nothing committed; for the coordinator)
- **Deviations from the brief:** (1) `why=UNREPORTED` is rendered inside `appendOdomTickAudit`, not in
  HealthMonitor (same text; the formatter is total). (2) `docs/changelog.md`, `docs/faq.md` and
  `docs/shulib-v2-master-plan.md` edited though not on the brief's list (prose-review findings; the
  changelog because `IOdometry` grew a PURE virtual). (3) `localizer.hpp` carries one word beyond the
  forwarder (:6 "four" -> "five"). (4) The mutation "implausible_ apart from the audit, one term dropped"
  SURVIVES for the `!finite` term — an equivalent mutant, proven above; the other terms are RED.
  **[CORRECTED in review round 1: not equivalent — B10/C9 kill it; see R1 below.]**
  (5) The register's status line reads "7 of 147" (HA-01..146 + HA-160) — it WILL conflict with T3's
  HA-147..159 edit of the same paragraph; resolve to 160.
- **NOT done (out of scope or the coordinator's):** Part B (the median's non-finite handling); a
  `DIAG1-COMPLETED.md`, `build-order.md` / `roadmap.md` / `HANDOFF-remaining-work.md` pointer moves;
  the found-while-reading items (the test loops' IMU-ready gate vs the library's ungated path;
  TrackingWheel's unchecked offset/diameter finiteness); anything on a robot. The briefing is
  regenerated LAST, below, and the coordinator regenerates both generated artifacts at merge.

### 10:45 — review round 1 starts (five findings from the coordinator's reviewer)
- Builder again, same worktree, nothing committed. Each finding re-checked against the tree and the
  capture BEFORE any edit; dispositions in "Review findings and their disposition" below, written
  as each one closes.

### Review findings and their disposition (review round 1, 10:45-11:10)
Each finding re-checked against the tree / the capture before any edit. New mutations ran through
the same runner (`diag1-mut/run_mutation.sh`: one textual change, `touch test/*.cpp`, `/fast`
rebuild, "Linking CXX executable shulib_tests" REQUIRED, whole suite, restore + `cmp`); every run
below `linked=1` and `restored`. Baseline for the batch: 1499 cases, all green.

- **R1 (major) — M05a is NOT an equivalent mutant; the header's proof was false. CONFIRMED, FIXED.**
  Re-derived: with `maxTickTravel = DBL_MAX` (legal — the precondition asks only finite and > 0),
  both drive sides at 1e308 are within the bound (travel false) but `dL + dR` overflows to +Inf
  before the halving, and arcStep gives dx = +Inf, dy = Inf * 0 = NaN — so the real code reports
  `why=NONFINITE` ALONE and M05a reports a clean tick. On tracking wheels, arcStep's rotation of
  (fwd, lat) reaches sqrt(2) * M, and `dTheta * offset` overflows for a huge finite offset.
  - Tests added (`test/odom_tick_audit_test.cpp`): **B10** (DEO, M = DBL_MAX, both sides 1e308:
    implausible, NONFINITE only, position frozen, golden ` why=NONFINITE dth=0.000rad h0=0.000
    h1=0.000 L=1e+308in R=1e+308in`); **C9** (Pilons, two subcases: pods at 0.9 DBL_MAX under
    M = DBL_MAX at a -45 deg heading — at 0 deg the same pods stay finite, the rotation is the
    cause — golden `... h0=-0.785 h1=-0.785 fwd=1.62e+308in lat=1.62e+308in`; and a DBL_MAX
    forward-pod offset on a 1.5 rad tick with no pod travel — golden `... dth=1.500rad h0=0.000
    h1=1.500 fwd=0.000in lat=0.000in`). The non-finite-offset case was NOT pinned (it would pin
    TrackingWheel's missing check as behaviour; it is named in the header instead).
  - Mutations, fresh binaries: **M05a** (DEO, the round-0 spec unchanged) **RED 1: B10**
    (round 0's log kept as `M05a.run.log.round0`). **M05e** (the same drop in Pilons) **RED 1: C9**.
  - `odom_tick_audit.hpp`: the "A FINDING" paragraph REPLACED by the real condition (NONFINITE is
    always also TRAVEL only when M <= DBL_MAX/2 and pi*|offset| + M stays under DBL_MAX/2; overflow
    from M > DBL_MAX/2 on drive encoders, from about DBL_MAX/sqrt(2) on tracking wheels, from
    |offset| > DBL_MAX/|dTheta|, every tick for a non-finite offset), naming B10/C9 and saying the
    first run got it wrong. Round 0's log paragraph, table row and deviation (4) are annotated
    in place as CORRECTED, with the original words kept.
  - `docs/guide/11-reading-the-diagnostics.md`: one sentence — `NONFINITE` alone means in-bound
    readings whose arithmetic overflowed, reachable only with a bound or offset configured near
    the largest double: check the configuration. (No example line added, so the chapter's
    "pinned byte-for-byte" claim is unaffected.)
  - NOT done: tightening the config preconditions (maxTickTravel <= DBL_MAX/2, a finite offset in
    TrackingWheel). A design change for the coordinator — listed for the team lead.

- **R2 (minor) — a held drive port on a GROUPED side never reaches the gate. CONFIRMED, FIXED.**
  Re-checked: `hal/motor_group.hpp` `position()` is `medianOf` over PRESENT members (four-member
  median = mean of the middle two); B9 pins one-of-four jumping 1000 rad as NOT implausible and two
  as TRAVEL with L = 500. `evaluateDisagreement()` samples member `velocity()` only.
  - `docs/faq.md`: the parenthetical rewritten — the gate sees only what the odometry reads; a
    single-motor side's port trips it on a > 36 in wake; a group side's one held member is absorbed
    by the median, and a TRAVEL there means enough members jumped together (two of four);
    `MOTOR_GROUP_DISAGREE` named as the group-level check, compares member velocities, and
    "whether it catches a member whose velocity is held as well is not established".
  - `docs/guide/11-reading-the-diagnostics.md`: the ODO_STUCK row's TRAVEL cause and the `L=`/`R=`
    bullet say the same, without claiming MOTOR_GROUP_DISAGREE catches a held velocity.
  - `docs/hardware-assumptions.md` HA-123 *Source*: "same failure shape" limited to a ONE-motor
    side; the group case (robot two, four a side) and the coupled-side monitor stated as above.
  - `include/shulib/localization/drive_encoder_odometry.hpp` :37-44: the SAME wrong sentence
    ("a drive port that enumerates late, or whose adapter holds a last-good value and then wakes,
    produces exactly the one-tick phantom") predates DIAG1 (R3b Part 2) and is what HA-123 cites —
    fixed there too (rule: a flaw found in an earlier chunk is fixed where it lives). Comment only.
  - Grep for other copies (`waking|wakes|enumerat* late|last-good value|held value` over docs/,
    include/, README, generated api excluded): no other drive-side claim.

- **R3 (minor) — the heading-swing list skipped an interval. CONFIRMED, FIXED.** Recomputed
  `atan2(-vy, -vx)` from `zcat ... | strings` myself (scratchpad script): the nine 10 ms intervals
  t=60.22→60.31 are +16.9, -19.7, +17.6, -17.1, +18.5, **-1.4** (t=60.27→60.28, record header
  lines 110143→110146, v(-32.9,-3.3)→v(-33.0,-2.5)), -18.2, +24.1, -36.9. Round 0's list dropped
  the -1.4; the rest reproduces (episode 2 at 110158, reversing t=59.97-60.31, last command before
  episode 1 at t=34.01).
  - Reworded to "up to 37°; 8 of the last 9 steps at 16.9° or more, alternating in sign": HA-160's
    index row and entry, `drive_encoder_odometry.hpp` :26-29 (also "this robot's" -> "robot
    two's"), R3d-PROGRESS's corrected paragraph (the -1.4 inserted, with a dated note that the
    first correction dropped it), and this log's 09:36 entry (annotated in place).
  - NOT edited: the brief (`DIAG1-odometry-tick-names-its-check.md` :24, :85, "17-37°") — the
    coordinator's committed document; flagged for the team lead.

- **R4 (minor) — contract clauses not pinned. CONFIRMED, FIXED (two pinned, one reworded).**
  - setPose: **B11** (DEO) and **C10** (Pilons) — an implausible ROT+TRAVEL tick, `setPose()`, then
    the audit is bit-identical (new `sameAudit` helper: flags, labels, every value by `sameBits`),
    the verdict still true, `tripped() == lastDeltaImplausible()`, and the teleport happened.
    Mutations **M19a** (DEO `setPose` gains `audit_ = {};`) **RED 1: B11**; **M19b** (Pilons, the
    same) **RED 1: C10**.
  - Before the first update: B1 and C1 now require the audit to EQUAL a default `OdomTickAudit`
    to the bit (C1 had no pre-update check at all). Mutations **M20a** (DEO member initialised
    with `.dTheta = 1.0`) **RED 1: B1**; **M20b** (Pilons initialised with TrackingWheels labels —
    a mutant only the bitwise check can see) **RED 1: C1**.
  - "The ticks after it cost nothing": agreed it cannot be pinned by an output test.
    `health_monitor.hpp` :59-66 now says "each pinned by test (except where said)" and marks the
    cost clause as a property of the code, NOT pinned, with the reason.

- **R5 (minor) — merge conflict unreported; log inaccuracies. CONFIRMED, FIXED.**
  - Merge note ADDED: the HA **index table** will conflict too — HA-160's row sits directly after
    HA-146's, where T3's HA-147..159 rows go. Resolution: T3's rows in order, then HA-160.
  - The 10:27 entry annotated: `drive_encoder_odometry.hpp`'s :25-28 HA-160 citation was ALSO a
    post-batch edit (verified: `diag1-mut/M05d.orig`, saved 10:12:32, lacks it; the only diffs
    against it are that comment and this round's comment edits). Comment-only, so round 0's
    mutation results stand.
  - The missing entry: round 0's briefing WAS regenerated last (file times, per the reviewer:
    briefing 10:28:11 after the log's 10:28:05); its check was not logged then. This round's
    regeneration and check are logged below.

- **Found while re-checking (logged, NOT fixed):**
  - Round 1's own edit shifted `health_monitor.hpp` by three lines: HA-42's citation re-pointed
    `:93–96` -> `:96–99`, HA-44's `:98` -> `:101` (verified against the file).
  - `drive_encoder_odometry.hpp` cited `pilons_odometry.hpp:119` for the `reset()` precedent —
    stale already at `ea58fbc` (it pointed at a precondition); replaced by "in PilonsOdometry's
    constructor" (no line number to rot). R3d-PROGRESS's "`drive_encoder_odometry.hpp` ~:164-166"
    (true at `ea58fbc`) now says "as of `ea58fbc`".
  - `include/shulib/motion/motion.hpp` :224 — a `SHULIB_PRECONDITION` message ("... wheel count —
    ...") carries a non-ASCII em dash. Predates DIAG1 (present at `ea58fbc`); outside this chunk's
    items; for the coordinator.

- **Gates after the fixes** (worktree; see the next entry for the final lines).

### 11:12 — round 1 gates, final (worktree; nothing committed)
- Order: `api_doc_tool.py generate` (after the last header edit) -> fresh `/fast` build ("Linking CXX
  executable shulib_tests" present; the first full build attempt ran the LAST MUTANT's binary
  because the stale-briefing gate stopped the build before relinking — caught by the 1 red case,
  not counted) -> suite -> doc gates -> ARM gate -> `src_build_gate.py` -> briefing regenerated
  LAST (11:10:29) -> full build (doc gates pass, test target up to date) -> suite again ->
  `briefing_status.py check`.
- Suite: **1499 cases (+4: B10, B11, C9, C10), 2,455,785 assertions, 3 skipped, SUCCESS.**
- Doc gates in the build: coverage 2615 entities / 149 headers; 408 example lines verbatim;
  removability clean; briefing block current. `api_doc_tool` self-test OK, check-coverage OK,
  check-fresh exit 0, check-examples OK, check-removability OK. `doc_staleness_audit.py` self-test
  OK (10 cases), audit clean. C7 grep of public docs and README for internal/, chunks/, RESUMING,
  build-order: empty.
- ARM header gate: 185 headers, `arm-none-eabi-g++ -std=gnu++20 ... -Werror -mfloat-abi=softfp`,
  exit 0. CI greps: PROS-free; sim-free. diag/ leaf: DIAG1's headers include no
  localization/motion/chassis/sim (`debug_record.hpp`'s kinematics include predates DIAG1).
  A4 reconciliation direction 1: printed nothing. New code string literals: ASCII (the one
  non-ASCII precondition message in `motion.hpp` predates DIAG1 — above).
- `src_build_gate.py` self-test OK (33 cases); check PASS (6 builds link, 0 disallowed warnings).
- `briefing_status.py check`: "generated block current". The briefing now reads next free HA-161
  (it counts HA-160); T3's HA-147..159 are not in this tree — the coordinator regenerates at merge.

### Merge notes for the coordinator (round 0's list, plus round 1)
1. `docs/hardware-assumptions.md` STATUS paragraph ("147 = HA-01..HA-146 plus HA-160") conflicts
   with T3's edit of the same paragraph — resolve to the combined count (160).
2. **NEW (round 1): the HA INDEX TABLE also conflicts** — HA-160's row sits directly after HA-146's,
   where T3's HA-147..159 rows go. Resolution: T3's rows in order, then HA-160's.
3. `docs/changelog.md` top section and `localizer.hpp` (one forwarder under
   `lastOdomDeltaImplausible()`, plus ":6 four -> five") against COMP-a.
4. Regenerate `docs/api/` and the briefing after the merge.

---

## The coordinator's merge — DIAG1 Part A (2026-09-26, ~13:20–13:40)

Squash-merged from the transport commit `32725d1` (branch `diag1`) onto `cc513a7` (T3 Part 1 and T3
Part 2's brief on top of `ea58fbc`). Conflicts, all documentation: `docs/hardware-assumptions.md` — the
status line is **7 of 155** (T3's 154 + HA-160); DIAG1's status sentence kept, its "numbered past
HA-147..HA-159, which T3 holds in parallel … the total is 160 once both are in" arithmetic replaced by
what is true after T3 (HA-148..152 unused, 155 entries); HA-160's index row after HA-159; its body stays
in Group R4 beside HA-123, its related entry. `docs/changelog.md` — DIAG1's "Changed since 2.2" section
above T3's (newest first). `docs/api/*` and the briefing regenerated (the briefing again after the build —
it quotes build-order's `Next:` line, which the merge edited). `build-order.md` (a DIAG1 PART A LANDED
sentence in the status block; the parallel-build clause) and `roadmap.md` ("Now") by the coordinator.

**Verification (the coordinator):** in the worktree before the merge — every gate green (1499 cases) and
seven hand mutations RED on fresh binaries (the rotation bound's equality flipped; L/R swapped in the
record; the fault line losing its detail; `+STALL` never appended; the audit never reaching the monitor;
tracking wheels reporting h0 as the after-heading; the verdict losing the travel check); the gate code
read line by line (every flag is the gate's own sub-expression copied as it stood, `implausible_ =
audit_.tripped()`, the text built only on the rising edge). On the merged tree: suite **1568 | 1568
passed | 0 failed | 3 skipped**, 2,462,140 assertions; `src_build_gate.py` self-test OK and check PASS
on all six builds; the ARM header gate clean (186 headers); both CI greps empty; `check-fresh` 0;
removability empty; the staleness audit clean; the briefing current after regeneration; two of the
mutations re-run on the merged tree (the detail dropped → 9 cases RED; the travel check dropped → 15
RED), the restored tree 1568/1568.

**Not done:** Part B (the median's non-finite handling) — its own later step. **Nothing has run on a
robot:** the next `ODO_STUCK` on robot two is the first real test of the new line.

---

## Part B

- **Chunk:** DIAG1 Part B — a drive side's median leaves out a member whose reading is not finite
- **Brief:** `DIAG1-partB-median-nonfinite.md` (the team lead's answers at its top: every
  recommendation D0-D9 accepted, 2026-09-26 ~23:10; the Re-check amendments override the body)
- **Tree at start:** `9239538`, clean, in the git worktree `.claude/worktrees/diag1b` (branch
  `diag1b`) — NOT the main working tree. In flight at the same time in other worktrees: comp-t,
  tank1b, SAFE-SAVE Part A, L9 Part A1, H1 Part A1, R5a Part A, T4 Part 2a.
- **Executor model:** opus
- **Go:** the team lead's go for this run (the coordinator's workflow, 2026-09-29). Do not commit; no
  robot upload; no `pros` commands; no `/dev/ttyACM*`. Scratch and probes under
  `.claude/coordinator/scratch/diag1b/` only.
- **Scope:** brief §1 items 1-5 with D1-D5, D7, D8 accepted; D6 and D9 OUT. HA numbers: ONLY HA-165
  (new) plus HA-08's one-sentence amendment.

Appended in real time. **Nothing in this part has run on a robot**, and on robot two the exclusion is
expected never to fire (`ProsMotor` screens a non-finite read to the last good one, HA-08).

### Log

#### 03:38 — start
- `git status` clean; HEAD `9239538`. Read `docs/internal/RESUMING.md` and the brief in full. This
  section created before any code.

#### 03:39-03:47 — baseline; the unchanged tree's median probed myself
- **Baseline (worktree at `9239538` + this section + the regenerated briefing):** `cmake -S test -B
  build/test`, `cmake --build build/test -j8` (the "Linking CXX executable shulib_tests" line
  present), suite **1725 cases | 1725 passed | 0 failed | 3 skipped, 2,648,032 assertions,
  SUCCESS**. The per-test xml report saved to the scratch folder for the "verdict intact" diff.
- **The unchanged tree's `hal::medianInPlace`, probed** (scratch `probe_unchanged.cpp`, against the
  real header; host x86-64, g++ 13.3.0 libstdc++, `-O2`; the robot's arm-none-eabi 13.2.1 library
  was NOT probed). Observed, never asserted in the suite (landmine 1):
  - `{3, NaN, 1, 2}` → NaN; `{NaN, 1, 2, 3, 4}` → 2 (finite median 2.5); `{1, 2, NaN, 3, 4}` →
    NaN, while `{1, 2, 3, 4, NaN}` → 3.
  - `{7, 7, NaN, 7, 7}` → NaN (N9's RED shape); `{7, 7, +Inf, 7, 7}` and `{7, 7, -Inf, 7, 7}` → 7.
  - `{5, Inf, 1, 2, 3}` → 3 (the finite four's median is 2.5); `{1, +Inf, +Inf}` → `+Inf`;
    `{+Inf, -Inf}` → a NEGATIVE NaN (bits `fff8000000000000`).
  - `{DBL_MAX, DBL_MAX}` → `+Inf` (D4's overflow); `{denorm_min, denorm_min}` → `denorm_min`.
  - 21 members with 1-3 NaN, 20,000 trials: seed 1 → 620 right, **18,030 wrong finite**, 1,350
    NaN; seed 2 → 637 right, **18,012 wrong finite**, 1,351 NaN. The brief's ~18,000 / ~1,300
    reproduce.

#### 03:47-03:55 — every file:line the brief cites, re-checked against `9239538` (deltas and how I adapted)
The brief was written against `a01e3a2`; SRC1, T4 Part 1a, TANK1 Part A and the plan docs of
2026-09-28 have landed since. Opened each cited file and line:
- **Library headers — unchanged at the cited lines:** `hal/motor_group.hpp` (`medianInPlace`
  :117-134, the class doc :136-143, `position()`/`velocity()` :207-216, `current()`/`temperature()`
  :218-238, `medianOf` :325-338, `sumCurrent` :340-348, the stale pointers `hal/motor.hpp:47-54`
  at :30 and `hal/motor.hpp:57-65` at :101); `hal/motor.hpp` (the screen text :60-67, the
  ownership text :70-78 — the brief's "now" numbers hold); `hal/pros/motor.hpp` (`faultedReads()`
  :290, the `mutable` caches :378-383); `localization/drive_encoder_odometry.hpp` (:8-10, :37-47,
  :151-157, :158-163, :177-192); `calibration/drivetrain_calibrator.hpp` (item 4 :92-95,
  PushCapture :53-59, `PortObservation::sign` :358-360, `warning()` :746-748, `pushReady()`
  :794-810, `finishPushCapture` :1029-1075, `finishPulse`'s `own`/`others` :1115-1121, the
  scale's spread and median :1251-1261, `pushSummary()` :1435-1451, `sideTravel()` :1530-1537,
  `appendWarning` :1571-1582, `formatPushRecord` :1864-1872). No delta.
- **Tests — unchanged:** `motor_group_test.cpp` test 8's dense-group check :470-475;
  `motor_group_plant_test.cpp` `CoupledMembers` :57-96, `GroupRig` :100-143, `requireIdentical`
  :206-218; `motion_test_rig.hpp` `PlantPacer` :148-170; `odom_tick_audit_test.cpp` B8 :556,
  `DriveEncoderRig` :1193-1222, F3 :1269; `calibration_test_rig.hpp` (robot two's shape, ports
  11-20 + 5, 6; `RunOptions::hook`/`stopWhen`); `sim/drive_plant.hpp` `writeMember`'s
  `case Frozen: return;` :432-433. No delta.
- **`src/` (SRC1 moved them; no `src/` file is edited here):** `src/calibrate_program.cpp` →
  `src/programs/calibrate_program.cpp` — the push summary still :791-793, the LEFT OUT line with
  `warning()` now :866 (brief ~:855-866); `src/teleop_program.cpp` → `src/programs/
  teleop_program.cpp` — the "median reads are over the PRESENT members" log line is now :812-813
  (brief ~:765-771); `src/drive_program.cpp` → `src/programs/drive_program.cpp` — members read
  directly at :475 (brief ~:458/:460); `src/bench_r3a.cpp` → `src/programs/bench_r3a.cpp` :1654
  (unchanged).
- **Docs — shifted:** guide 11's ODO_STUCK table row is now **:240** (brief ~:214), the
  `NONFINITE` bullet **:273-277** (brief ~:247-251), the `L=`/`R=` bullet **:282-288** (brief
  ~:255-261) — T3 Part 2's six lines and later edits moved them; the edits are made at the new
  lines. `hardware-assumptions.md`: HA-08's index row is **:123** (brief ~:115), its blast radius
  **:416-420** (brief ~:404-407), the status paragraph **:39-99** (brief ~:39-80), HA-160's entry
  in Group R4 **:2130-2168** (brief ~:2013-2046), HA-161's index row **:271** — and the next row is
  now **HA-179** (TANK1's), so HA-165's row goes between HA-161 and HA-179 (the brief's numeric-
  order rule, which it wrote for exactly this). HA-162..HA-164 are NOT in this tree (COMP-b's
  and 2c's reservations did not land as entries); the status paragraph says "HA-162..HA-178 are
  not in it" — HA-165 splits that range.
- **The changelog — a structural delta:** the brief says "a new top section, 'Changed since 2.2
  (DIAG1 Part B)'". TANK1 Part A has since bumped the API to **2.3** (`## API 2.3` is the top
  section). Adapted: the section goes at the top of `## API 2.3` as **"Changed since 2.3"** — no
  bump (the brief's reasoning holds: `MotorGroup`, `medianInPlace` and the calibrator are outside
  the Freeze Register). L9 Part A1 (in flight) plans API 2.4: merge note.
- **In-flight overlap — `hal/motor_group.hpp` is T4 Part 2a's too** (runtime absence:
  `markAbsent`, `setMonitorExcluded`, labels; its reads exclude a runtime-absent member). The
  brief predates T4 Part 2's plan. This chunk's edits there are required by the brief (D1, D7,
  D8); kept to `medianInPlace`, `medianOf`, the current/temperature reads, the new counter and
  their comments. **Merge note for the coordinator** (the report repeats it).

#### 03:55-04:18 and 10:37-10:41 — the code (D1-D5, D7, D8), the new test file, the unchanged-tree run
*(Times corrected from the scratch files' stamps: these headings were first written from an
assumed clock. The session did no work between ~04:18 and ~10:37.)*
- **`hal/motor_group.hpp`:** a public `isUsableReading(double)` (= `std::isfinite`) — THE one
  test; `medianInPlace` copies the finite values FORWARD in order, returns a fresh
  `quiet_NaN()` when none is finite, sorts the k finite values and uses `std::midpoint` for an
  even k; `<limits>` and `<numeric>` included. `MotorGroup`: `medianOf` counts each non-finite
  reading into `nonFiniteReads_` (with `isUsableReading`) and hands it to the median, which
  excludes it; `current()` / `totalCurrent()` sum only finite readings (`CurrentSum{amps,
  counted}`) and divide by the COUNTED members; `temperature()` is the max over finite readings,
  no longer seeded from `first()` (each member read once), NaN when none is finite; new `int
  nonFiniteReads() const noexcept` over a `mutable int`. The header's "A READING THAT IS NOT A
  NUMBER" section; the stale `hal/motor.hpp:47-54` and `:57-65` pointers now cite the IMotor
  class doc / destructor doc by name and line.
  **Decision (not in the brief): `totalCurrent()` returns NaN, not 0, when no present member's
  current is finite** — the brief says "the sum over the same members"; an empty sum would read
  0 A, a side drawing nothing, which no member said (ProsMotor's own rule: "never substitute
  zero"). Pinned in N5.
- **`localization/drive_encoder_odometry.hpp`:** comments only (:8-10, :37-47, :151-157); the
  `update()` body untouched (`git diff` shows comment lines only).
- **`calibration/drivetrain_calibrator.hpp`:** private `movedOnPush(d)` =
  `hal::isUsableReading(d) && std::abs(d) >= movedThreshold` — the one predicate
  `finishPushCapture`'s classification and `pushSummary()` use; `finishPushCapture` appends a
  `Warning::NoReading` sentence for each non-finite push delta; `appendWarning(port, which)`
  gained the no-reading sentence (the brief's words, verbatim); `formatPushRecord` prints
  `NO READING` for a non-drive candidate whose reading is not finite; a comment at the scale's
  median (~:1255) says why no guard is needed there; comments at item 4, PushCapture,
  `movedThreshold`, `PortObservation::sign`, `movedNow`, `warning()`, `pushReady()`,
  `pushSummary()`, `formatPushRecord`. No other calibrator logic moved (D9).
  `movedOnPush` uses `hal::isUsableReading` rather than a bare `std::isfinite` — identical
  semantics, and it ties the calibrator's "no reading" to the group's by construction.
- **Syntax + ARM checks** of both headers on the way (host g++ `-Werror`; `arm-none-eabi-g++
  -Werror -Wformat-truncation=2 -mfloat-abi=softfp`): clean.
- **The new file `test/median_nonfinite_test.cpp`**, cases `DIAG1 N1..N10`; `HA-165` in its
  header beside N7. Helpers COPIED (Lcg, sameBits, render, errorLines; `CoupledMembers` with its
  absent-slot parameter dropped; `PlantPacer` as `InjectingPacer`; `requireIdentical`'s checks
  inline). N9 and N10 run each kind in its own SUBCASE (first written as loops; restructured
  when the unchanged-tree run stopped at the first failing kind): doctest ends a case's run at
  a failed REQUIRE, so the unchanged-tree outcomes below were read one sub-run at a time with
  `-sc` filters.
- **After the change:** suite **1735 | 1735 passed | 0 failed | 3 skipped, 2,668,803
  assertions** (1725 + the 10 new cases; +20,771 assertions, all in the new file).
- **"Verdict intact" — measured, not assumed:** the per-test assertion counts (doctest xml)
  of the UNCHANGED tree (original headers + a reduced copy of the new file) against the changed
  tree: **1735 cases, 0 pre-existing cases moved**; the only movement is the 10 new cases. The
  unchanged tree's pre-existing total is 2,648,032 — the baseline's, exactly. So every Part A
  case (`DIAG1 A1..F3`, `B9` included), MotorGroup 7's sweeps and every calibrator case pass with
  the same assertion counts.
- **The unchanged tree, the new cases** (original headers, the new file reduced only by the
  counter subcase and the N9 counter lines, which do not compile there; binary relinked): **all
  10 RED**. Per case: N1 (the first NaN placement), N2 (no-finite not a fresh NaN), N3 (ONLY the
  named `{DBL_MAX, DBL_MAX}` case: +Inf; the 200,000 random all-finite arrays matched), N4 (the
  sparse zero inputs answer `+0` where the dense ones answer `-0`, twice, and through the group),
  N5 (a one-member group with a bad member not a fresh NaN, …), N6 (a bad member made the tick
  implausible), N7 (the step was not ∓0.25), N8 (the side reading ≠ the oracle), N9, N10:
  - **N9 (RED evidence, as the brief predicted):** NaN → `exit Cancelled, abortFault OdoStuck`
    at **pace 60** (the tick after the injection), one `ODO_STUCK` line, the trace cut at 60 of
    the baseline's 180 paces. **+Inf and -Inf → GREEN** (Settled, 180 paces, bit-identical, no
    ODO_STUCK): one infinity of five sorts to an end. They are regression pins, not evidence
    that the case sees the defect. No `MOTOR_GROUP_DISAGREE` in any run (Frozen held the
    member's mid-motion velocity; its mates kept close to it for the 31 ticks) — D6 not reached.
  - **N10 (a) held to the end:** +Inf → `PULSE_CONTRADICTION` at port 13 (movedNow 10,
    drivePortCount 10, sign +1, line `... +inf deg MOVED sign +1`, no warning) — the brief's
    expected wrong-reason refusal, **now observed**; -Inf → the same, sign -1; NaN / -NaN →
    `Done`, drivePortCount 9, LEFT 11/12/14/15, line `... still sign +0`, **warning empty** —
    left out in silence, as the brief said.
  - **N10 (b) recovers after FRONT:** NaN → `NON_DRIVE_MOVED` port 13 (already the net);
    **+Inf → `Done` with port 13 on the LEFT, counted with a push travel of +Inf** (the median
    put it at the end); **-Inf → `Done` with port 13 recorded as `-13` on the RIGHT side** — a
    WRONG description saved without a word: the -Inf gave sign -1, the pulse then drove the
    left train backward, its own encoder "agreed" (own > 0 under the wrong sign), and the yaw
    named it RIGHT. Found by this run; not in the brief. Unreachable on a V5 (HA-08), and closed
    by D5: under the new rule the same run is refused `NON_DRIVE_MOVED` naming port 13.
  - **N10 (c):** the CCW spin's `L` = NaN, the run refused `SPIN_WIDTH_INVALID`.
- **N9 on the changed tree:** all three kinds bit-identical to the baseline (truth, both
  primaries' volts, the left group's position every pace, exit Settled, 180 paces), zero
  ODO_STUCK, zero MOTOR_GROUP_DISAGREE, `abortFault` None; the left group counted **64**
  non-finite readings per run = 31 odometry reads + 31 trace reads + **2 by `OdoStallCheck`**
  (a window closed inside the 31 ticks: it reads each drive motor at the close and again for its
  new baseline) — the brief's "OdoStallCheck gets a finite spin travel", observed.
- **N10 on the changed tree:** (a) every kind: movedNow 9 while watched, pushReady() still
  true, drivePortCount 9, sign 0, `push port 13 travel +inf deg NO READING sign +0` (`-inf`,
  `+nan`, `-nan`), Done with LEFT 11/12/14/15, the warning names port 13, the LEFT and RIGHT
  scales equal the clean run's bit for bit, every other push line unchanged; (b) every kind
  `NON_DRIVE_MOVED` port 13, safe state; (c) Done, 743 poisoned ticks, the CCW record's L/R/W
  and the CW record equal the clean run's bit for bit.

#### 10:44-10:50 — the documentation (while the mutation batch runs; no header or test touched mid-batch)
- **Guide 11** (prose only; no example line added, so its "pinned byte-for-byte" claim is
  untouched): the ODO_STUCK table row (:240) — "a reading that came back NaN/Inf (`NONFINITE`)"
  now says that on a motor-group side it needs NO member to have given a number; the `NONFINITE`
  bullet (:273-283) — a group side reads NaN only when no member is finite, one bad member is
  left out like a missing one, not expected on a V5 (HA-08), nothing run on a robot; the
  `L=`/`R=` bullet (:288-298) — the membership step (a little, and back), `L=NaN` = no member.
- **The register:** HA-165 (Group R4, directly after HA-160, ~:2177; its index row :278, between
  HA-161 and HA-179 — numeric order); HA-08 amended — one sentence in its blast radius, and **AMENDED
  2026-09-29** in its index row; the status paragraph: "7 of 159 settled", a DIAG1 Part B
  sentence, and "HA-162..HA-164 and HA-166..HA-178 are not in it". Evidence cited by date, no
  `docs/internal` path. The per-confidence counts are left as they are (the paragraph already
  says they predate HA-124 and a recount is due).
- **The changelog:** a new top section under `## API 2.3`, "Changed since 2.3 (2026-09-29)"
  (delta: the brief said "since 2.2" — TANK1 bumped to 2.3 since): the behaviour change for
  position/velocity/current/totalCurrent/temperature, the calibrator's `NO READING` + warning,
  the `L=NaNin` line change; CHANGED `medianInPlace`'s post-condition (with a migration line);
  ADDED `nonFiniteReads()` and `isUsableReading()`; no bump (why); host test plant only.
- **Found while documenting — the Done screen cuts the new warning.** `calibrate_panel.hpp` shows
  `warning()` on the Done screen as at most TWO lines of `kSmallCols` = 50 characters
  (`wrapWords<kSmallCols, 2>`, ~:836-842; it does not read `Wrapped::complete`). The brief's
  sentence is 110 characters, so the screen shows "port 13 gave no reading at DONE -- not
  counted as" / "a drive port; if it drives the robot, check it and" and drops "run again"
  (the slipping sentence, 81 characters, fits). The port and "not counted as a drive port"
  survive; the PUSH TABLE line and the serial record carry the whole text. NOT changed here: the
  sentence is the team lead's accepted wording (D5 (b)) and `calibrate_panel.hpp` is out of this
  chunk's scope (brief §3). **For the coordinator / the team lead:** a ≤ 100-character wording
  (e.g. "port 13 gave no reading at DONE -- left out; if it drives the robot, check it and run
  again", 91) or a panel change.

#### Prose review (pace rule 4) — the greps RAN first; each hit read, then judged
Grep over `docs/` and `README.md` (generated `docs/api/` and `docs/internal/` excluded) and over
`include/` + `src/`, for: `median`, `medianInPlace`, `MotorGroup`, `motor_group`, `non-finite`,
`NaN`, `NONFINITE`, `PROS_ERR_F`, `last good`/`last-good`, `ABSENT`, `faultedReads`,
`pushSummary`, `spread`, `drivePortCount`, `movedThreshold` (the per-term file lists are in the
scratch folder's `prose_grep_summary.txt`). The broad terms (`NaN`, `non-finite`, `ABSENT`,
`last good`) hit many unrelated topics (waypoint validation, GPS `hasFix`, absent devices);
every hit that speaks of a motor group, the median, a drive side's reading or the calibrator's
push was read in full:
- **Edited (in scope):** guide 11 :240, :273-283, :288-298; `hardware-assumptions.md` (above);
  `changelog.md` (above); the three headers.
- **Still true, not edited:** `docs/faq.md` :18-36 (held values: a HELD member is finite and
  still absorbed by the median); `hardware-assumptions.md` HA-130 :1140-1162 (the calibrator's
  scale through `hal::medianInPlace`, "cancels EXACTLY" — still the one function) and HA-123
  :2110-2125 (a held/waking member absorbed; landmine 9: no amendment beyond HA-08); guide 12
  :143 (`why=NONFINITE` held position — still true); guide 06 :74-84, :170-178 (adapters hold
  last good; NAN_POSE); guide 07 :178, guide 13 :229, master plan :381, :571 (the adapter idiom);
  `roadmap.md` :276, :1449, :1458 (dated delivery records — read, not edited); `changelog.md`
  :676 (a dated entry); `include/shulib/robot/robot.hpp` :974-976 ("a frozen member is already
  excluded by the group's median" — still true); `include/shulib/sim/drive_plant.hpp` :397-400
  (the Frozen signature "the group's median must ignore" — still true);
  `include/shulib/teleop/coupled_side_monitor.hpp` :19, :84, :208 (PRESENT members of the
  monitor — D6, untouched); `src/programs/calibrate_program.cpp` :752-753, :785, :791-793
  ("MOVED past N deg" — a non-finite reading is not "moved", still true), :876-879 (median
  travel and spread — finite by construction now).
- **Incomplete but not false — merge notes (no `src/` file is edited, brief §4):**
  `src/programs/teleop_program.cpp` :812-813 ("median reads are over the PRESENT members") and
  :847 ("every read is over the PRESENT members") — now also "whose reading is finite";
  `src/programs/calibrate_program.cpp` :866 prints `warning()` only beside a LEFT OUT (decoupled)
  list, so on a run with no decoupled port the no-reading warning reaches the Done screen and the
  PUSH TABLE line but not that summary line — a follow-up for the next owner of that file.

#### 10:44-10:57 — the mutation table (each on a FRESH binary, observed RED, restored)
Runner: scratch `mutate.py` — apply the edit(s) (each pattern asserted to occur exactly once, the
file asserted equal to its saved copy first), `touch test/*.cpp`, `cmake --build build/test
--target shulib_tests/fast -j8`, REQUIRE the "Linking CXX executable shulib_tests" line (every run
below: relinked), run the WHOLE suite with the xml reporter, list the cases whose asserts failed
(from the xml — the first draft parsed "TEST CASE:" headers from the console and counted every
case that prints a MESSAGE; caught on M8's first run, which listed passing EKF cases, and fixed
before any result below was recorded), restore from `scratch/diag1b/mine/`, `cmp` (every run:
restored), touch. Baseline before the batch: 1735 green.

| # | Mutation (file) | Result | RED cases (failed asserts) |
|---|---|---|---|
| M1 | the filter removed — every value sorted (`motor_group.hpp`) | **RED 9** | N1(1) N2(1) N4(3) N5(7) N6(1) N7(16) N8(1) N9(3) N10(3) |
| M2 | only NaN excluded (`!std::isnan`) | **RED 6** | N1 N2 N4 N5(5) N6 N8 |
| M3 | only ±Inf excluded (`!std::isinf`) | **RED 9** | N1 N2 N4(4) N5(3) N6 N7(16) N8 N9(3) N10(3) |
| M4 | no finite value returns 0.0 | **RED 5** | N1 N2 N5(10) N6(20) N8 |
| M5 | no finite value returns `v[0]` | **RED 5** | N1 N2 N5(7) N6(2) N8 |
| M6 | swap-with-last compaction | **RED 1** | N4 (4: :442 twice, :461 through the group) |
| M7 | a last-good cache in the group (a bad read replaced by the member's last finite one) | **RED 4** | N5 at :591 — the spelled-out kill sequence — N6 N7(10) N8 |
| M8 | midpoint back to `0.5 * (a + b)` | **RED 1** | N3 (the `{DBL_MAX, DBL_MAX}` fixed case) |
| M9 | midpoint as `0.5*a + 0.5*b` | **RED 1** | N3(2) (`{denorm_min, denorm_min}`; the random draw too) |
| M10 | the exclusion moved into `MotorGroup::medianOf` only | **RED 4** | N1 N2 N4(2) N10(3) (the calibrator's direct call, (c)) |
| M11 | the exclusion made sticky (a member once bad stays out) | **RED 4** | N5 at :521 (not sticky) N6 N7(8) N8 |
| M12 | `pushSummary()` bypasses the helper (`std::abs(d) >= threshold`) (`drivetrain_calibrator.hpp`) | **RED 1** | N10(4) (movedNow 10) |
| M13 | the classification bypasses the helper | **RED 1** | N10(5) (sign, drivePortCount, the line, Done) |
| M14 | `nonFiniteReads()` misses velocity reads | **RED 1** | N5(28) (the counter subcase) |
| M15 | `temperature()` seeded from the first member again | **RED 1** | N5(4) |
| M16 | `current()` divides by `presentCount()` | **RED 1** | N5 at :619 |
| M17 | the push record's word back to `still` | **RED 1** | N10(4) (the `NO READING` suffix) |
| M18 | the counter counts `std::isnan` only (all three counting sites) | **RED 2** | N5(20) N9(2) (the ±Inf sub-runs counted 0) |
| M19 | the no-reading warning not appended | **RED 1** | N10(4) (`warning()`) |
| M20 *(beyond the brief)* | the SHARED helper loses its finiteness test (both sites revert at once) | **RED 1** | N10(5) |
| M21 *(beyond)* | `totalCurrent()` answers 0 when nothing is finite (the logged decision) | **RED 1** | N5(5) |
| M22 *(beyond)* | the temperature exclusion only for NaN (a +Inf member the hottest again) | **RED 1** | N5(3) |

Every brief-listed mutation M1-M19 is RED on a fresh binary; none survived, so no case needed
strengthening (N4 killed M6 on its first run). Pre-existing cases never went red under any
mutation — the new behaviour is pinned only by the new file, as expected (the old suite feeds
the median finite values only).

#### 10:58-11:25 — the gates, final (worktree; nothing committed)
- Order: `api_doc_tool.py generate` (after the last header state) → briefing regenerated →
  headers and `test/*.cpp` touched → full build (the doc gates inside it pass; "Linking CXX
  executable shulib_tests" present — the fresh binary, not the last mutant's) → suite →
  briefing regenerated again once the suite binary existed → `check`.
- **Suite: 1735 | 1735 passed | 0 failed | 3 skipped, 2,668,803 assertions, SUCCESS** (baseline
  1725 / 2,648,032: +10 cases, +20,771 assertions, all in `test/median_nonfinite_test.cpp`; the
  per-test xml diff shows 0 pre-existing cases moved).
- Doc gates in the build: coverage 2937 entities / 156 headers, all documented; 467 quoted
  example lines verbatim; briefing block current; staleness audit clean. Explicitly:
  `api_doc_tool` self-test OK, check-coverage OK, check-fresh exit 0, check-examples OK,
  check-removability OK ("no public doc references docs/internal/"). `doc_staleness_audit.py`
  self-test OK (10 cases); audit clean (27 live docs). C7 grep of public docs, guide, cookbook
  and README for internal/, chunks/, RESUMING, build-order: empty.
- **ARM header gate:** 192 headers, `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion
  -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp`,
  exit 0 (the one-TU file in the scratch folder, not /tmp).
- **CI greps:** PROS-free (no `pros/` include outside `hal/pros/`): empty; sim-free (no
  `shulib/sim/` include outside `sim/`): empty.
- **`src_build_gate.py`:** self-test OK (189 detector cases); check **PASS — all 6 builds link**
  (bench, xdrive, tank, tank-drive, tank-library, tank-calibrate; 8/8 TUs each, 0 disallowed
  warnings). COMP-b has not merged, so six, not seven.
- **Register reconciliation:** direction 1 (a `PROVISIONAL (A4` label with no HA id) printed
  nothing; direction 2 for HA-165: found in BOTH files its Source names — `motor_group.hpp` (1)
  and `test/median_nonfinite_test.cpp` (4). (`drive_encoder_odometry.hpp` also cites it, in a
  comment.)
- **New string literals:** the no-reading sentence and `NO READING` — 7-bit ASCII; the output
  carries no '%' (N10 asserts `screenSafe(warning())`; the calibrator's case 30 passes).
- **The four `medianInPlace` call sites, confirmed:** `MotorGroup::medianOf`, the calibrator's
  scale (`finishSideAssign`), `pushSummary()` and `sideTravel()` each pass a local scratch array
  and never read it after the call (the scale's spread is computed BEFORE the call, in the fill
  loop). No caller in `src/`.

#### Where this run stops (nothing committed; for the coordinator)
- **Deltas from the brief, each with its reason:**
  1. The changelog section is "Changed since **2.3**" under `## API 2.3`, not "since 2.2" —
     TANK1 Part A bumped the API to 2.3 after the brief was written.
  2. `totalCurrent()` returns NaN (not an empty sum's 0) when no present member's current is
     finite — "never substitute zero", as ProsMotor's screen says. Pinned by N5; M21 kills it.
  3. The helper the brief wished for ("ideally both come from one inline helper") is PUBLIC:
     `hal::isUsableReading(double)` in `motor_group.hpp` — the calibrator's `movedOnPush()` is
     built on it too, so the median, the counter and the push rule share one definition. It is
     documented (the coverage gate) and listed in the changelog as ADDED.
  4. N10 (a)/(b) stop the scripted operator at the first `pushReady()` tick (`stopWhen`),
     inject, run ONE more watched tick (so `status().movedNow` shows the push as the calibrator
     now reads it), then answer DONE and FRONT by hand and let the rig's operator finish. The
     brief's route (the hook, then the rig's own DONE in the same iteration) leaves no tick
     between the injection and DONE, so `movedNow` could not be observed "while the push is
     watched". The clean comparison run takes the same path.
  5. N10 (c) does not locate the finishing tick: its hook poisons port 13's position on EVERY
     CCW spin tick (no `Frozen`, so the plant rewrites it after each tick and no velocity is
     held). Only `sideTravel()` reads a drive shaft's position during a spin, so only the
     finishing tick sees the NaN — proven by the whole run matching the clean one bit for bit
     and by its RED on the unchanged tree (SPIN_WIDTH_INVALID).
  6. N9's and N10's kinds each run in their own SUBCASE (the brief does not say); N9 also records
     the left group's `nonFiniteReads()` as proof the exclusion happened.
  7. The guide, register and changelog lines the brief cites had shifted (T3 Part 2, TANK1):
     edited at their current lines (the re-check entry above).
- **Found while building (not in the brief):** on the unchanged tree a `-Inf` push reading that
  recovers after FRONT gave a SAVED description with port 13 on the WRONG side (`-13`, RIGHT) —
  closed by D5. And the Done screen cuts the new warning's last words (above) — for the team
  lead: reword to ≤ 100 characters, or change the panel.
- **Follow-ups named by the brief and still open:** D6 (the monitor and a non-finite member
  velocity), D9 (the calibrator's live-reading comparisons), `src/programs/calibrate_program.cpp`
  :866 (the serial summary prints `warning()` only beside LEFT OUT), `src/programs/
  teleop_program.cpp` :812-813 and :847 (incomplete, not false).
- **NOT touched (the coordinator's, per the brief):** `docs/roadmap.md`,
  `docs/internal/build-order.md`, `docs/internal/HANDOFF-remaining-work.md`,
  `docs/internal/REMAINING-WORK.md`; `calibrate_panel.hpp`; every `src/` file; the host test
  plant; `ProsMotor`; the coupled-side monitor; guides 06 and 12 (still true).
  `DIAG1-COMPLETED.md` is DRAFTED (both parts) for the coordinator to edit at the merge.

#### 19:01 — RESUMED (a new executor run; `date` read Tue 2026-09-29 19:01:51 EDT)
- The previous run was STOPPED by the team lead's pause at a usage limit (~13:50). Its last writes:
  this log and `DIAG1-COMPLETED.md` at 13:50:54, the briefing at 13:51:00, the suite binary at
  13:51:58 — so it stopped inside the final gates, after "Where this run stops" above.
- State found: `git status` shows the three headers, the docs, the regenerated `docs/api/` pages and
  the briefing modified, `test/median_nonfinite_test.cpp` and `DIAG1-COMPLETED.md` new. **`git diff
  -- src/ Makefile` is EMPTY and nothing under `src/` is untracked**: no gate self-test plant was
  left by the killed run. The scratch copies (`scratch/diag1b/mine/`) of the three headers and the
  test file `cmp` EQUAL to the tree, so no mutation was left applied (the mutation log ends at
  M22 → `done`).
- Plan: do not redo finished work. Re-read the diff myself (the code and the tests, against the
  brief), then rebuild from a touched tree and re-run EVERY gate, and re-run the mutations on
  fresh binaries, logging the numbers here.

#### 19:02-21:39 — the resumed run: the diff re-read, the unchanged tree re-run, every mutation and every gate again
- **The diff re-read against the brief** (the three headers in full, the test file's ten cases, the
  changelog, the register, guide 11, the completion draft): the code does what §1 and D1-D5, D7, D8
  say, and the logged deltas stand. **No code defect found; no code changed.** Four edits, prose
  only:
  1. `hal/motor_group.hpp` :30-32 — the pointer fix had left one sentence broken over three lines
     ("… would then drift / by 1/N of all further / travel …"); re-wrapped.
  2. `calibration/drivetrain_calibrator.hpp` `pushReady()`'s doc (:809-821) — D5's insertion had
     left a one-phrase line (`enough"); and everything at rest`); re-wrapped. A `///` comment, so
     `docs/api/` was regenerated.
  3. `test/median_nonfinite_test.cpp` :5-6 — the header named "M1..M19"; it now names M20..M22 too.
  4. `DIAG1-COMPLETED.md` — Part A's sentence "M05a, was shown NOT to be in review round 1" said
     less than the log: it now says M05a is not an equivalent mutant, B10 kills it, and C9 kills the
     same drop in the tracking-wheel odometry (M05e).
  The scratch backups (`scratch/diag1b/mine/`) were refreshed to these versions before any mutation.
- **The unchanged tree, re-run against the FINAL test file.** The first unchanged-tree run (04:17)
  predates the test file's last edits (saved 10:40), so its record might describe an earlier file.
  Re-run without touching the worktree: the three headers at `9239538` (`git show`) in
  `scratch/diag1b/unchanged_overlay/`, FIRST on the include path (`-H` shows all three resolve
  there), and the final test file reduced ONLY by the counter subcase and N9's three counter lines
  (they do not compile without `nonFiniteReads()`), built with the suite's own flags. **All 10 new
  cases RED** — N1(1) N2(1) N3(1) N4(3) N5(7) N6(1) N7(16) N8(1) N9(3) N10(5) failed asserts. The
  sub-runs, one at a time (`-sc`):
  - N9: **NaN → exit Cancelled, abortFault OdoStuck at pace 60, one ODO_STUCK line** (the RED
    evidence); **+Inf and -Inf → Settled, 180 paces, GREEN** (regression pins, as logged).
  - N10 (a) held: ±Inf → `PULSE_CONTRADICTION` port 13 (movedNow 10, drivePortCount 10, `+inf deg
    MOVED sign +1` / `-inf deg MOVED sign -1`); NaN and -NaN → `Done`, drivePortCount 9, `still
    sign +0`, warning EMPTY.
  - N10 (b) recovers: +Inf → `Done`, port 13 on the LEFT (5); **-Inf → `Done`, port 13 saved as
    `-13` on the RIGHT** (the wrong-side description); NaN → `NON_DRIVE_MOVED` port 13 — this one
    sub-run PASSES on the unchanged tree (for a NaN the net already existed).
  - N10 (c): the CCW spin's `L` NaN, `SPIN_WIDTH_INVALID`, 743 poisoned ticks.
  Every outcome is the one the 04:17 record gives.
- **The §0 probe, re-run** (`scratch/diag1b/probe_unchanged.cpp`, `-O2`) against the overlay's
  original header: every number identical to the 03:39 entry (seed 1: 620 right / 18,030 wrong
  finite / 1,350 NaN; seed 2: 637 / 18,012 / 1,351). The same probe against the CHANGED header:
  every shape defined — `{1, 2, NaN, 3, 4}` → 2.5, `{+Inf, -Inf}` → a positive quiet NaN
  (`7ff8000000000000`), `{DBL_MAX, DBL_MAX}` → `DBL_MAX`, and 21 members with 1-3 NaN → 20,000 of
  20,000 right, both seeds.
- **Checked, because it would silently undo the change on the robot:** no `-ffast-math` or
  `-ffinite-math-only` anywhere in the robot build (`common.mk`'s `MFLAGS` is `-mcpu=cortex-a9
  -mfpu=neon-fp16 -mfloat-abi=hard -Os -g -mthumb`; the `Makefile`'s `EXTRA_CXXFLAGS` is empty;
  a grep of both for `fast-math`, `finite-math` and `Ofast` is empty). Under either flag the
  compiler may fold `std::isfinite` to `true`.
- **Every mutation again, on the final tree** (21:11-21:23; `scratch/diag1b/resume_mutate.py` is
  `mutate.py` writing `resume_mut_*.xml`, so the first batch's files are kept): **all 22 relinked
  and RED, each with the SAME failing cases and the same failed-assert counts as the first batch**
  (compared by script; M8's first run is in its own 10:44 xml: N3, 1 assert). Every file restored
  and `cmp`-equal to its backup.
- **A trap, logged so the next session skips it:** after the batch the suite binary is the LAST
  mutant's (the runner restores the source but does not relink). A plain `cmake --build build/test`
  then FAILS at the doc gate, because the briefing check reads that mutant's RED count. The order
  that works: `--target shulib_tests/fast` (relinked), the suite, `briefing_status.py generate`, and
  then the full build.
- **"Verdict intact", re-measured on the final binary:** the 1725 pre-existing cases carry
  **2,648,032 assertions in both** the unchanged tree's xml and the final one, **0 moved**; the 10
  new cases carry 20,771.
- **The gates, final (after the four edits):**
  - suite **1735 | 1735 passed | 0 failed | 3 skipped, 2,668,803 assertions, SUCCESS**;
  - the full build's doc gates: `api_doc_tool` self-test OK; coverage 2937 entities / 156 headers,
    all documented; 467 quoted example lines verbatim; removability clean; the briefing block
    current; the staleness audit self-test OK (10) and clean (27 live docs);
  - `api_doc_tool` run by hand: self-test, check-coverage, check-fresh (exit 0, no output),
    check-examples, check-removability — all exit 0;
  - `src_build_gate.py` self-test OK (189 detector cases); check **PASS — all 6 builds link**, 0
    disallowed warnings; `git status` of `src/` and the `Makefile` empty before and after;
  - the ARM header gate over all 192 headers (`-Werror … -mfloat-abi=softfp`): exit 0;
  - the CI greps, PROS-free and sim-free: both empty;
  - the C7 grep of the public docs for `internal/`, `chunks/`, `RESUMING`, `build-order`: empty;
  - register reconciliation: direction 1 prints nothing; direction 2 finds `HA-165` in
    `motor_group.hpp` (1) and `test/median_nonfinite_test.cpp` (4);
  - new string literals ASCII; the one with `%` is a format string, and its output carries none;
  - `briefing_status.py` regenerated after the suite and `check` passes. It now counts DIAG1
    COMPLETE (33 of 63), because the drafted `DIAG1-COMPLETED.md` exists.
- **Prose review, re-run BEFORE this line was written:** the brief's fifteen terms, plus
  `nonFiniteReads`, `isUsableReading` and `NO READING`, over `README.md` and `docs/` (generated
  `docs/api/` and `docs/internal/` excluded). The hit files match the 10:44 summary term for term
  once `NaN` and `ABSENT` are grepped case-insensitively, as they were then; the extra files that
  brings in are other words (`dominant`) and other absences (a mechanism's motor, JSON fields, a
  legacy enum). The same fifteen terms over `include/` and `src/` give the 10:44 per-term file
  counts exactly (median 7, MotorGroup 18, non-finite 44, NaN 44, …). Re-read: `docs/faq.md`
  :18-36, guide 06 :170-178, guide 12 :143, guide 10 :160 and :281 and the cookbook :164 (waypoint
  validation, not this topic), the master plan :176-179 and :564 (a filters utility, not this
  median) and :381 (a non-finite tick freezes the pose — still true), `roadmap.md` :276, :1449,
  :1458 (dated records). All still true. The `src/` hits for the median and the present members
  (`teleop_program.cpp` :812-813, :847; `calibrate_program.cpp` :752, :791, :866, :876-879) read as
  the 10:44 entry judged them.
- **Found while verifying (not fixed; for the coordinator):**
  - `warning()` holds 256 bytes, and the no-reading sentences are appended at DONE, before any
    slipping sentence (appended at SideAssign). With two or more no-reading candidates, a later
    slipping sentence is cut. The report's `excludedPorts` still names it, as the header says. It is
    unreachable on a V5 (HA-08), but it belongs with the Done-screen wording item above.
  - `shulib-v2` has moved past this worktree's base since the brief: `ee2e399` (plan docs) and
    `a5bd83f` (TANK1 Part B, no API bump — still 2.3). Its hunks in `docs/changelog.md` (:21, :30),
    guide 11 (+33 lines after :96) and the register (:2061) are all apart from this chunk's, so the
    merge is expected to be clean. That was read from `git diff`, not tried.

#### Where this run stops (resumed run; nothing committed)
Everything the brief asks for is built, and verified twice: the first run (03:38-11:25), then this
run re-read, re-ran and re-measured it (19:01-21:39). The deltas, the found-while-reading items,
the follow-ups and the "not touched" list above stand. This run adds only the four prose edits and
the two found-while-verifying items. **Nothing has run on a robot.**

### DIAG1-B review findings and their disposition (review round, 2026-09-30)

Three reviewers (correctness, tests, docs) returned nineteen findings through the coordinator's
workflow; the team lead's go for this run stands. Each is re-checked here, then fixed at the root or
answered with evidence. Nothing is committed. Appended in real time.

#### 03:46 — RESUMED for the review round (`date` read Wed 2026-09-30 03:46:30 EDT)
- The workflow says a previous fix run for these findings was stopped by the account's usage limit
  (2026-09-29, ~22:20 to ~01:05). **State found — it left nothing to finish or undo:**
  - This log had NO review-round section: its last entry is the resumed run's "Where this run
    stops" (written 21:40 on 09-29).
  - No worktree file is newer than 21:40:53 on 09-29 (`find -newermt`, `build/` and `.git`
    excluded). The suite binary is from 21:36:48, the resumed run's final build.
  - `git diff -- src/ Makefile` is empty, and nothing is untracked under `src/`, `tools/` or the
    `Makefile`: no gate self-test plant was left.
  - The four backups in `scratch/diag1b/mine/` `cmp` EQUAL to the tree: no mutation was left applied.
  - Every scratch file written after 21:49 on 09-29 is a reviewer's (`review-correctness/`,
    `review-docs/`, `review-testgate/` and their `r2/` and `run2/` second runs, the last at ~03:43).
  - So every finding is dispositioned from scratch here; nothing earlier is redone or reverted.
- The binary is relinked from source (the briefing regenerated first) before any suite result is
  trusted. Dedupe: the odd-count wording is in #1 and #14 (one fix); "by a little" is in #1
  (changelog) and #11 (guide 11).

#### 11:26 — RESUMED again (`date` read Wed 2026-09-30 11:26:45 EDT)
- The 03:46 run was stopped too, after the entry above and before any disposition was logged.
  **State found:**
  - Its scratch files are in `scratch/diag1b/fix/`: the briefing regenerated, a full build and a
    green suite (1735 | 1735, 03:53-03:54, BEFORE any edit), then two probes — `fastmath/`
    (04:19, finding #10) and `minority_probe.cpp` (04:20, finding #1, run on the old and the new
    headers).
  - It then edited the three headers (04:21-04:22) and nothing else: no test, no doc. Each edit
    was compared with `scratch/diag1b/mine/` (the backups, which predate it). **Every diff is a
    fix, none a mutation:** `motor_group.hpp` — a line re-wrapped past 100 columns (:32), the
    MEMBERSHIP STEP bullet (#1, #14, #16), a `#error` under `__FINITE_MATH_ONLY__` (#10),
    `isUsableReading()`'s doc (#10), `medianInPlace`'s overflow sentence (#4);
    `drive_encoder_odometry.hpp` — two comments (#1, #16); `drivetrain_calibrator.hpp` —
    `pushTravelRad` written +0.0 for a port with no sign, and its doc (#5), and a second reason
    text for `NON_DRIVE_MOVED` when the port gave no reading at DONE (#15). None was logged,
    built, tested or mutated. They are kept only as a draft: each is re-read below and judged
    with its finding.
  - `git diff -- src/ tools/ Makefile` is empty and nothing is untracked there: no gate
    self-test plant was left. The suite binary (03:53:14) predates the header edits, so it is
    relinked from source before any result is trusted. `docs/api/` (19:04 on 09-29) predates
    the `///` edits, so it is regenerated.
- This run's scratch folder is `scratch/diag1b/fix2/`.

#### 01:26 (10-01) — RESUMED a third time (`date` read Thu 2026-10-01 01:26:21 EDT)
- The 11:26 run was stopped too. It logged nothing after its 11:26 entry, but it worked until
  ~15:16. **State found** (file times, and every file diffed against `scratch/diag1b/mine/`, the
  backups of the reviewed tree):
  - `scratch/diag1b/fix2/`: `docs/api` regenerated, the briefing regenerated and a full build with a
    green suite at 11:32-12:03 (all `test/*.cpp` touched at 12:00:54; `git diff -- test/` is empty,
    so that was a timestamp, not an edit); the reviewers' three probes re-run on the old and the new
    headers at 14:11 (`probes/`: `minorityprobe`, `stallprobe`, `d9probe`); the fast-math probe at
    14:58 (`fastmath/`).
  - Then edits, none built, tested or mutated: `drivetrain_calibrator.hpp` at 14:13 (the 03:46
    draft's `pushTravelRad` and `NON_DRIVE_MOVED` edits, plus the PushCapture "host test plant
    only" clause and the `NonDriveMoved` enum doc), `motor_group.hpp` at 14:58 (the 03:46 draft
    plus "travelled ~20 % short on a hand push … unmeasured under power" and the NONFINITE+TRAVEL
    sentence), and `test/median_nonfinite_test.cpp` at 15:16: N3's second draw past DBL_MAX/2 and
    `{-DBL_MAX, -DBL_MAX}`; N5's 0 A subcase; N7's middle-member and several-left-out subcases;
    N10's `pushTravelRad` check, the reason-text checks in (b), and a new sub-run (d) with two
    no-reading ports. The file header already names **N11 and N12, which do not exist yet** —
    the run stopped while writing them.
  - **Every hunk reads as a fix, none as a mutation.** No mutation tooling ran in `fix2/` (no
    backups, no mutation log there); `git diff -- src/ tools/ Makefile` is empty, nothing is
    untracked but the two new files, and nothing is ignored-and-new under `src/` or `tools/`: no
    gate self-test plant was left.
  - The suite binary (12:01:25) predates every one of those edits: it is relinked from source
    before any result is trusted.
- Plan: judge each of those edits with its finding (keep, change or revert — logged), write N11
  and N12, run every review-round mutation on a fresh binary, re-run M1-M22 on the final tree
  (the code and the tests moved since 09-29's batch), do the docs, then every gate. This run's
  scratch folder is `scratch/diag1b/fix3/`; its mutation backups go to `fix3/backup/` (`mine/` is
  kept as it is: the reviewed tree of 09-29, the reference these diffs were read against).

#### 01:26-08:45 (10-01) — the drafts judged; N11 and N12 written; the first fresh build
*(The clock read 01:26 at the last command before a build and 08:39 at the next one: the session
was suspended between them, not working. Times below are `date` readings.)*
- **The 11:26 run's draft edits, re-read and judged** (each is kept unless said otherwise):
  - `motor_group.hpp` — the MEMBERSHIP STEP bullet (#1, #14, #17): kept, with four precision
    edits of mine: an odd count is "three or more" (one member left out of one is no reading,
    not a step); "between the smallest and the largest present member" → "between the lowest
    and the highest member position", the bound named "the members' SPREAD (highest minus
    lowest)"; "the worst case is one member read alone" → "the one member farthest from the full
    median, read alone" (the extreme order statistic is the worst subset); "under 36 in … (DIAG1
    N7 pins both)" → "a step under maxTickTravel (36 in by default) moves the pose SILENTLY
    (DIAG1 N7 pins that, and TRAVEL past the bound)" — "both" named nothing N7 asserts about the
    old tree. The bounds re-derived by hand: removing x_j from x_1..x_{2m+1} leaves the midpoint
    of (x_{m+1}, x_{m+2}), (x_m, x_{m+1}) or (x_m, x_{m+2}); the step is half a gap, or half the
    DIFFERENCE of the two gaps, never more than half the larger.
  - `motor_group.hpp` — the `#error` under `__FINITE_MATH_ONLY__` and its comment (#10): kept;
    every factual claim in the comment re-measured below before keeping it.
  - `motor_group.hpp` — `medianInPlace`'s overflow sentence (#4): kept. Checked: two values of
    size ≤ DBL_MAX/2 cannot sum past DBL_MAX, so an overflow needs at least one past it, and
    the old half-sum gave +Inf or -Inf by the pair's sign.
  - `drive_encoder_odometry.hpp` — the two comments (#1, #17): kept (comments only; `git diff`
    of the file shows no code line).
  - `drivetrain_calibrator.hpp` — `pushTravelRad = sign != 0 ? sign × delta : +0.0` and its doc
    (#5): kept. One consequence the draft did not log: a STILL port whose finite delta was
    negative used to read `0 × -d = -0.0`; it now reads +0.0. No reader can tell: the only
    reader in the library is the scale (`v[k] = forward * pushTravelRad`, over drive ports
    only, sign ±1), `src/programs/calibrate_program.cpp`:796 prints `status().pushTravelRad`
    (another field), and `drivetrain_calibrator_test.cpp`:384 compares with `== 0.0`. Grep of
    `include/`, `src/`, `test/`, `tools/` and the public docs for `pushTravelRad`: no other
    reader.
  - `drivetrain_calibrator.hpp` — the second `NON_DRIVE_MOVED` reason (#16), the
    `NonDriveMoved` enum doc and the PushCapture "host test plant only" clause (#17): kept.
    The reason keys on `!isUsableReading(report_.ports[j].pushDeltaRad)` — the SAME test the
    push record's NO READING word uses, and `j` runs over non-drive candidates only (`drive_[j]`
    skipped), so the reason and the PUSH TABLE line cannot disagree. No `Refusal` code is
    added (brief §4 OUT). The panel's headline for the code stays "WRONG MOTOR MOVED / a motor
    off the drive turned / check the port shown, RUN AGAIN" (`calibrate_panel.hpp`:295-296 —
    out of scope, brief §3); the reason line under it now tells the true story.
  - `test/median_nonfinite_test.cpp` — N3's second draw, N5's 0 A subcase, N7's two
    subcases, N10's `pushTravelRad` check, (b)'s reason checks and (d): kept, with two fixes
    of mine: (1) **N7 "several left out" asserted "TRAVEL, the pose held" for the 69 case — and
    FAILED on the first fresh build** (`CHECK( -25.5938 == 0 )`). The test was wrong, not the
    code: the gate REPORTS a finite tick and never withholds it — "only a non-finite tick
    FREEZES position" (`drive_encoder_odometry.hpp`:36-38, `update()`:206-207). Rewritten:
    the pose moves by half the step in both cases; the bound decides only whether the tick is
    FLAGGED; the return tick puts the pose back at 0 exactly in both. (2) Its comment called the
    unchanged tree's ±Inf outcome "logged, not asserted: landmine 1" — landmine 1 is NaN's
    undefined behaviour; an infinity sorts. Reworded ("observed on the unchanged tree and
    logged; the suite runs the new header only") — and observed below.
- **Written in this run:**
  - **N11** (#4): verbatim copies of `9239538`'s `sumCurrent`/`current`/`totalCurrent`/
    `temperature` (:221-238, :340-348 there); 200,000 random ALL-FINITE groups (n = 1..21, ~15 %
    holes, whole milliamps and wide-range currents of either sign, ±0.0; 30 % "cold" groups whose
    hottest member is often a zero of either sign) compared bit for bit, plus `nonFiniteReads()`
    staying 0. Self-checking power: it counts the groups a REVERSED sum and a reversed max
    would answer differently and REQUIREs > 1000 of each (observed: **91,702** and **25,035**),
    so a draw that lost the power to see an order change fails instead of passing on nothing.
    Fixed cases: a side of -0.0 A reads +0.0 (n = 1, 2, 5); of +0.0/-0.0 temperatures the FIRST
    is kept.
  - **N12** (#6, #9): (a) robot two's REAL wiring on the rig — `Reversed` on members (0,0),
    (0,2), (0,4), (1,1), (1,3), i.e. `LEFT -11 +12 -13 +14 -15 | RIGHT +20 -19 +18 -17 +16`
    (`src/robots/chassis_table.hpp`:123-125): 5 negative deltas, pushReady() offered (1 ready
    tick), movedNow 10, `status().pushTravelRad` = the oracle's median |delta| = the plain run's
    (29.5385), Done, every signed port and side as the table has them, no warning, the plain
    run's scale bit for bit; (b) pushed BACKWARD (`pushDirection = -1`) and answered BACK-first:
    every delta negative, every sign +1, every sign-corrected travel negative, Done, the plain
    run's description AND scale bit for bit; (c) `movedThreshold` set to the smallest |delta|
    of a first run (the plant is deterministic and the calibrator commands nothing during the
    push, so the second push is the same bits — REQUIREd): 10 of 10 drive ports travelled
    exactly the threshold, drivePortCount 10, Done.
  - The file header's N1..N12 list matches the cases (N11 and N12 continue the N series;
    landmine 8's rule is about Part A's letters).
- **The first fresh builds** (`api_doc_tool generate` → briefing → `cmake -S test -B build/test`
  → touch the headers and `test/*.cpp` → `--target shulib_tests/fast`, "Linking CXX executable
  shulib_tests" present, 0 warnings): after the N7 fix, **suite 1737 | 1737 passed | 0 failed |
  3 skipped, 2,669,029 assertions**. Per-test against the 09-29 final xml: the **1725
  pre-existing cases carry 2,648,032 assertions in both, 0 moved**; N3 7→12, N5 4,983→4,993, N7
  32→58, N10 212→257, N11 14 and N12 126 new.
- **#10 measured** (`scratch/diag1b/fix3/fastmath/`): `arm-none-eabi-g++` 13.2.1 `-Os` compiles
  `std::isfinite(x)` to a `vabs`/`vcmp` sequence, and with `-ffinite-math-only` to `mov r0, #1;
  bx lr`. Host g++ 13.3.0 against the REAL header: `-O2` compiles and gives
  `isUsableReading(NaN)=0`, `{1,2,NaN,3,4} -> 2.5`; `-O2 -ffinite-math-only`, `-Os -ffast-math`
  and `-Ofast` each **fail to compile at `motor_group.hpp`:189's `#error`**; against a copy
  with only the three `#if/#error/#endif` lines removed, the same three flag sets compile and
  give `isUsableReading(NaN)=1`, `{1,2,NaN,3,4} -> nan`. The ARM header gate (192 headers, its
  `-Werror` flags) plus `-ffinite-math-only`: **rc 1 at the `#error`**; without: rc 0, no
  output. `__FINITE_MATH_ONLY__` read with `-dM -E`: 0 plain and 1 under each of the three
  flags, on host g++ 13.3.0, arm-none-eabi-g++ 13.2.1 AND clang 18.1.3 — the comment's "GCC and
  Clang" is measured. A grep of the `Makefile`, `common.mk`, `test/CMakeLists.txt`, `.github/`
  and `tools/*.py` for `fast-math`, `finite-math` and `Ofast`: empty (`common.mk`'s `MFLAGS` is
  `-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -Os -g -mthumb`).

#### 09:15 (10-01) — #18: the sites the 03:47 re-check entry skipped, with their lines now
The 03:47-03:55 entry (09-29) said it re-checked "every file:line the brief cites" and never named
D7's readers or several D5 sites. Opened now, against this tree (the calibrator's and the group's
lines from the clean copies in `scratch/diag1b/fix3/backup/`, since the mutation batch was editing
the worktree's):
- **D7's readers:** the debug records' `wheelCurrent[i] = motors[i]->current()` — `chassis.hpp`
  **:615** (brief ~:581: moved, TANK1 Part A), `move_to_pose.hpp` **:353** (brief ~:236: moved,
  TANK1 Part A), `turn_to.hpp` :235, `drive_brake.hpp` :207, `motion_scheduler.hpp` :1189 (no
  delta); `motion.hpp` `tickHealthObservables` — `maxTemp = std::max(maxTemp, m->temperature())`
  from 0.0 — **:261-263** (brief ~:261-264); `health_monitor.hpp`'s over-temperature raise
  **:228-237** (brief ~:228-233); the scheduler's default abort mask (`OdoStuck` only)
  `motion_scheduler.hpp` :233 (no delta). Read through: `std::max(0.0, NaN)` keeps 0, so a NaN
  group temperature raises nothing, and a +Inf member is now left out, so it raises nothing on its
  own — the changelog's `MOTOR_OVER_TEMP` sentence holds.
- **The median's other readers:** the drive-encoder odometry's construction baseline
  `drive_encoder_odometry.hpp` **:148** (brief ~:140-141); `OdoStallCheck`'s spin travel
  `odo_stall_check.hpp` **:206-211** and its re-baseline **:257-262** (brief ~:206-209, ~:258-262);
  `MoveToPose` handing it `ctx.driveMotors()` `move_to_pose.hpp` :339. `src/` readers unchanged
  from the 03:47 entry.
- **D5's and D9's calibrator sites** (brief, at `a01e3a2`): the NonDriveMoved loop **:898-922**
  (~:870-889), `firstMoving` **:1463** (~:1418-1425), `finishPulse`'s `own`/`others` **:1156-1162**
  (~:1115-1121), the SpinWidthInvalid refusal **:1374** (~:1321-1330), `sideTravel()` **:1591**
  (~:1536); the description's `ipr > 0` rule `drivetrain_description.hpp` **:260** (~:260-261). The
  calibrator's shifts are all this chunk's own insertions above those lines: the file is identical
  at `a01e3a2` and `9239538` (`git diff --stat` of the two, empty — as are the group's header, the
  odometry's, and the four test files this chunk copies helpers from), and untouched on
  `shulib-v2` since.
- **T4 Part 1a and the 2026-09-28 plan docs** (named in the task, not assessed at 03:47):
  `git diff --stat a01e3a2 9239538` touches none of this chunk's three headers or its test file's
  helpers' sources; T4 Part 1a's files (`src/team/robot_two_driver.hpp`, `src/common/
  driver_controls.hpp`, `api_doc_tool`'s example globs) are not this chunk's, and `check-examples`
  passes on them. No delta to adapt to.

#### 09:08-09:29 (10-01) — the mutation batch: 35, each on a fresh binary, observed RED, restored
*(This entry and the three after it were written together at ~09:40; their times are the scratch
files' stamps.)*
Runner: scratch `fix3/mutate_fix3.py` — the 09-29 runner's table (M1-M22) verbatim, plus this
round's thirteen; backups in `fix3/backup/` (`cmp`-equal to the tree before the batch); each pattern
asserted to occur exactly once; `touch test/*.cpp`; `--target shulib_tests/fast` with "Linking CXX
executable shulib_tests" REQUIRED (35 of 35 relinked); the whole suite with the xml reporter; restore;
`cmp`. Baseline before the batch: 1737 green, 2,669,045 assertions. The batch ran 09:08:09-09:27:11,
P4 after it. Afterwards: all four files `cmp`-equal to their backups, no `MUTATION` string in either
header, and the binary relinked green FROM SOURCE (1737 | 1737, 2,669,045) — it was P4's mutant.

| # | Mutation (file) | Result | RED cases (failed asserts) |
|---|---|---|---|
| M1 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 9** | N1(1) N2(1) N4(3) N5(7) N6(1) N7(19) N8(1) N9(3) N10(3) |
| M2 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 7** | N1(1) N2(1) N4(1) N5(5) N6(1) N7(1) N8(1) |
| M3 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 9** | N1(1) N2(1) N4(4) N5(3) N6(1) N7(18) N8(1) N9(3) N10(3) |
| M4 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 5** | N1(1) N2(1) N5(10) N6(20) N8(1) |
| M5 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 5** | N1(1) N2(1) N5(7) N6(2) N8(1) |
| M6 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 1** | N4(4) |
| M7 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 4** | N5(1) N6(1) N7(10) N8(1) |
| M8 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 1** | N3(3) |
| M9 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 1** | N3(3) |
| M10 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 4** | N1(1) N2(1) N4(2) N10(3) |
| M11 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 4** | N5(1) N6(1) N7(13) N8(1) |
| M12 | the 09-29 batch's, verbatim (`drivetrain_calibrator.hpp`) | **RED 1** | N10(5) |
| M13 | the 09-29 batch's, verbatim (`drivetrain_calibrator.hpp`) | **RED 1** | N10(6) |
| M14 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 1** | N5(28) |
| M15 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 1** | N5(4) |
| M16 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 1** | N5(1) |
| M17 | the 09-29 batch's, verbatim (`drivetrain_calibrator.hpp`) | **RED 1** | N10(6) |
| M18 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 2** | N5(20) N9(2) |
| M19 | the 09-29 batch's, verbatim (`drivetrain_calibrator.hpp`) | **RED 1** | N10(5) |
| M20 | the 09-29 batch's, verbatim (`drivetrain_calibrator.hpp`) | **RED 1** | N10(6) |
| M21 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 1** | N5(5) |
| M22 | the 09-29 batch's, verbatim (`motor_group.hpp`) | **RED 1** | N5(3) |
| S1 | `pushSummary()` stores the SIGNED delta (D5's split-out `std::abs` dropped) — #6 (`drivetrain_calibrator.hpp`) | **RED 1** | N12(4) |
| S2 | no separator (space, bar, space) before a second no-reading sentence — #8 (`drivetrain_calibrator.hpp`) | **RED 1** | N10(1) |
| S3 | `movedOnPush()` compares with `>` — #9 (`drivetrain_calibrator.hpp`) | **RED 1** | N12(12) |
| S7 | `totalCurrent()`'s NaN guard keyed on the sum (`amps != 0.0`) — #7 (`motor_group.hpp`) | **RED 2** | N5(5) N11(4) |
| S8 | `current()`'s NaN guard keyed on the sum — #7 (`motor_group.hpp`) | **RED 2** | N5(5) N11(4) |
| MA1 | `sumCurrent()` walks the members in reverse — #4 (`motor_group.hpp`) | **RED 1** | N11(2) |
| MT1 | *(beyond)* `temperature()` walks them in reverse (`motor_group.hpp`) | **RED 1** | N11(3) |
| MS1 | *(beyond)* the sum seeded from the first finite reading, not 0.0 (`motor_group.hpp`) | **RED 1** | N11(8) |
| MW | a different midpoint (`a + 0.5*(b-a)`) past DBL_MAX/2 ONLY — #4 (`motor_group.hpp`) | **RED 1** | N3(1) |
| P1 | `pushTravelRad` back to sign × reading for every port — #5 (`drivetrain_calibrator.hpp`) | **RED 1** | N10(7) |
| P3 | *(beyond)* +0.0 only for a non-finite reading (a still port read backward: -0.0) (`drivetrain_calibrator.hpp`) | **RED 1** | N10(1) |
| P2 | NON_DRIVE_MOVED's reason back to "did not move on the push" for every port — #16 (`drivetrain_calibrator.hpp`) | **RED 1** | N10(6) |
| P4 | *(beyond)* the no-reading reason for EVERY port (`drivetrain_calibrator.hpp`) | **RED 1** | DrivetrainCalibrator 8(1) |

- **Every M1-M22 is RED again on this tree, with the same failing cases as the 09-29 batch**
  (compared by script). Counts grew only where this round's subcases add asserts (M1 N7 16→19, M3 N7
  16→18, M8 N3 1→3, M9 N3 2→3, M11 N7 8→13, M12 N10 4→5, M13 N10 5→6, M17 N10 4→6, M19 N10 4→5,
  M20 N10 5→6), and M2 now also fails N7 (the several-left-out subcase's +Inf members). No case
  dropped out of any RED set.
- **The six that survived the reviewers' runs are RED now:** S1 (N12 (a), (b)), S2 (N10 (d)), S3
  (N12 (c)), S7 and S8 (N5's 0 A subcase and N11's -0.0 cases), MA1 (N11). Their survival records:
  `scratch/diag1b/review-testgate/run2/mutations.log` (S1, S2, S3, S7, S8 "SURVIVED", 1735 ok) and
  `review-correctness/r2/suite_MA1.txt` (1735 | 1735 passed).
- MW is the reviewers' S9 (`a + 0.5*(b-a)`, which N1 and N3 already killed) restricted to values
  past DBL_MAX/2, so that N3's first draw and every fixed case cannot see it: only the new draw
  kills it. P3 dies only in N10 (e), as designed. P4 dies in the PRE-EXISTING calibrator case 8,
  which pins the old reason's "not a drive motor".

#### 09:19-09:32 (10-01) — the probes, on the unchanged headers and the final ones
All in `scratch/diag1b/fix3/`; "old" = the overlay of the three headers at `9239538`, first on the
include path (`-H` confirmed every one resolves there).
- **#1, `probes/minorityprobe`** (members {100, 100.25, 100.5, 80}, three of four set to +Inf, robot
  two's 1.625 in/rad): old — `side L=inf implausible=1 pose x=+0.000 in | why=NONFINITE+TRAVEL …
  L=+Infin`; new — `side L=80 implausible=0 pose x=-16.352 in | why=UNREPORTED`, and the next tick
  back to x=+0.000. The reviewer's numbers, exactly.
- **#2, `probes/stallprobe`** (default `OdoStallCheckConfig`, a still pose): old — all +Inf and all
  -Inf → `spinTravel=inf stalled=TRUE (-> ODO_STUCK)`, {0, +Inf, +Inf} → stalled TRUE; new — all
  +Inf, all -Inf, all NaN → `left.position()=nan spinTravel=nan stalled=false`, {0, +Inf, +Inf} →
  `stalled=false spinTravel=0`.
- **#3, `probes/d9probe`** (port 12's reading bad during its own pulse only): IDENTICAL on old and
  new — `+Inf`: under the floor at 3 V and 6 V, LEFT at 9 V (yaw -6.471 deg, own inf), saved as 12
  on the LEFT; `-Inf`: PULSE_CONTRADICTION at 3 V, yaw +0.000; NaN: under the floor at 3 V and 6 V,
  CONTRADICTION at 9 V. Its second case shows #5 directly: a NO READING port's `pushTravelRad` was
  `inf` (+Inf), `inf` (-1 × -Inf) and `nan` on the old tree, and is `0` for all three now. (D9's
  Decoupled-at-a-still-top-level and Rolled-from-a-+Inf-mate outcomes are read from `finishPulse`
  :1152-1174, not probed.)
- **#6, `s1probe/`** (the reviewer's three runs; two overlays in scratch — the clean headers, and
  the same with S1 applied to a COPY; the worktree untouched): clean — all three `end 5` (Done),
  `pushTravelRad 29.5385`; S1 — robot two's wiring `end 2 readyTicks 0 status pushTravelRad 0`
  (never offered DONE), DONE forced `end 6 refusal PUSH_TOO_SHORT … turned only 0.00 rad`, the
  backward BACK-first push `readyTicks 0 … pushTravelRad -29.5385`. N12's comment states these.
- **#16, `wrap/wrap_probe`**: both NON_DRIVE_MOVED reasons, formatted at ports 5/13/21 and 11/20
  and velocities from 0.123 to -999.999 rad/s, fit `wrapWords<kSmallCols, 3>` complete (longest 129
  and 137 characters), as the calibrator comment says.
- **#10, the robot build:** `make ROBOT=tank PROGRAM=calibrate EXTRA_CXXFLAGS=-ffast-math V=1` on a
  cleared `bin/`/`.d/` (gitignored; the gate clears them the same way): rc 2, stopping at
  `./include/shulib/hal/motor_group.hpp:194:2: error: #error "shulib needs NaN and infinity …"`.
  (`:194` now — five lines below the 08:45 entry's `:189`, after the OdoStallCheck sentence.)
- **The unchanged tree, the FINAL test file** (`unchanged/build_run.sh`: reduced only where the
  counter does not exist — N5's counter subcase, N9's two counter checks and record, N11's one
  counter line — and built with the suite's own flags): **N1-N10 RED** (N3 3, N7 19, N10 6 failed
  asserts — the new subcases add to them) and **N11, N12 GREEN** — regression pins, as expected:
  the unchanged tree's current/temperature reads ARE the old ones, and its push rule took `std::abs`
  at the delta. The new subcases one at a time (`-sc`): N7 "several left out" — `REQUIRE(
  left.position() == lone)` gives `inf == 80` (four +Inf of five reach the middle; the minority
  probe above shows the tick that follows, NONFINITE+TRAVEL with the pose held); N7 "the MIDDLE
  member" — NaN (undefined behaviour on this library: observed only); N10 (d) — port 13 a drive port
  (sign 1), 9 moved; N10 (e) — `pushTravelRad` not +0.0 (it is -0.0); N3's new draw — 5,663
  overflowing pairs give ±Inf where the finite midpoint is wanted, and both named pairs fail.
  **A trap, logged so the next run skips it:** doctest splits `-sc` at commas, so `-sc="current,
  totalCurrent*"` matched NO subcase and "passed"; escaped (`-sc='current\, totalCurrent*'`) the
  subcase runs and is RED at its first checks (the old n = 1 reads), before its 0 A part.

#### 09:13 (10-01) — #15: the merge, re-judged by a trial against `shulib-v2` = `ce95325`
The 21:39 entry's "expected to be clean" was read from `git diff` against `a5bd83f` and is
SUPERSEDED. Trial: `git merge-file` in `scratch/diag1b/fix3/merge/` (base `9239538`, ours the
worktree, theirs `shulib-v2`). **No code file overlaps:** the three headers and the new test file
are untouched on `shulib-v2` since `9239538` (`git diff --name-only 9239538 shulib-v2` over this
chunk's files lists only the documents below). **Four conflict regions:**
- `docs/changelog.md` (1): `shulib-v2` is at **API 2.4** (L9 Part A1, `fa915b0`), and COMP-t put
  "Added and fixed since 2.3 (2026-09-28)" at the top of `## API 2.3`. This chunk's section goes
  at the TOP of `## API 2.4`, above L9's, retitled "Changed since 2.4 (<merge date>)"; its "No
  version bump" reasoning holds unchanged (`version.hpp` stays 2.4).
- `docs/hardware-assumptions.md` (3): the status count → **7 of 162**; the status paragraph keeps
  COMP-t's and L9's sentences ("… with it, 161") and adds this chunk's ("DIAG1 Part B … added ONE —
  HA-165 …; with it, 162"); "The rest of HA-162..HA-176 is not in it" stays true (165 and 167 are
  rows then), and the new "Held numbers" line may keep 165 in its range (its own rule: "a number
  that lands as a row may stay in its range"); the index rows in order HA-161, **HA-165**, HA-167,
  HA-177, HA-178, HA-179. The briefing's next free number stays **HA-187**.
- `docs/guide/11-reading-the-diagnostics.md`: merges clean.
- Regenerate `docs/api/*` and `docs/internal/PROJECT-BRIEFING.md` after the merge (both changed
  on `shulib-v2`).

#### 09:20 (10-01) — prose review for this round (pace rule 4): the greps ran first, every hit read
Over `README.md` and `docs/` (generated `docs/api/` and `docs/internal/` excluded): `pushTravelRad`,
`NON_DRIVE_MOVED`/`NonDriveMoved`, `did not move on the push`, `WRONG MOTOR MOVED`,
`fast-math`/`finite-math`/`Ofast`/`FINITE_MATH`, `membership step`, `HA-165`,
`OdoStallCheck`/`stall check`/`stall cross-check`, `isUsableReading`, `NO READING`,
`medianInPlace`, `nonFiniteReads`, `totalCurrent`. Every hit outside this chunk's own edits is a
stall-check one, each read: `diagnostics-plan.md` :73 and :231-236 (what the check detects, and
that robot two's never reports), `roadmap.md` :256, :1168, :1476-1482 (dated records), the register
HA-52 :173, :617, :731, :1057, :1165 (its thresholds and geometry; HA-131's tautology), guide 11 :240
(the stall line's words), guide 06 :207 (a link list), `README.md` :278 (a directory listing) —
all still true (none says what the check does with a reading that is not a number). Over
`include/` and `src/` for the same phrases: the calibrator's enum, `PortObservation` docs and
:1531 ("every candidate on no side … gets 0 V") and `calibrate_panel.hpp` :296 (the panel words,
logged as a follow-up) — consistent.

#### The nineteen findings — disposition (each re-checked by me; "fixed" means at the root, with its evidence)
Deduped: the odd-count wording is #1's secondary and #14 (one fix); "by a little" is #1 (changelog)
and #11 (guide 11).

1. **HA-165 and the header covered only ONE member left out — REAL, fixed.** Re-checked: with k
   members left out the median of the rest lies between the lowest and highest member position,
   so the step is bounded by the members' spread, and the worst subset is the farthest member read
   alone. The reviewer's probe re-run on both trees (below) reproduces their numbers. Fixed:
   `motor_group.hpp`'s MEMBERSHIP STEP bullet (several left out; the ±Inf class-B change; "an odd
   count (three or more)"); `drive_encoder_odometry.hpp` :45-52 and :166-169; HA-165's claim,
   source, confidence, settle (now the farthest member's distance from the median × ipr, bounded
   by the spread) and blast radius, its index row and the status sentence; the changelog (a
   bullet of its own, no "by a little"); guide 11 (#11). Pinned: N7's two new subcases — the
   MIDDLE member left out (+0.5, the midpoint of its neighbours) and four of five left out at robot
   two's 1.625 in/rad (80 read alone: -33.3125 in, NOT flagged, the pose moves by half; 69:
   -51.1875 in, TRAVEL; both carried back exactly).
2. **OdoStallCheck's verdict changed for an all-±Inf side, unlisted — REAL, documented; the policy
   is a follow-up.** Re-checked in `odo_stall_check.hpp` :206-222 and :257-262, and by the
   reviewer's probe re-run on both trees (below). Not changed in code: `motion/odo_stall_check.hpp`
   is outside the brief's scope and what it should do with an unreadable travel is a design
   choice. Documented: the changelog's BEHAVIOUR CHANGE list (a bullet of its own), the header's
   "NaN WHEN NONE IS FINITE" bullet, and the completion draft (the follow-up, with D6/D9).
3. **The brief's D9 description was wrong for ±Inf — REAL, corrected where the follow-up is
   recorded.** Re-checked by reading `finishPulse` (:1152-1174) and by the reviewer's probe re-run
   on both trees (below): `+Inf` own passes `own > 0`, is Decoupled at a non-turning top level,
   `-Inf` contradicts with no yaw, NaN is Weak then contradicts at a turning level, a `+Inf` mate
   makes `others` Inf (Rolled). `DIAG1-COMPLETED.md`'s D9 item rewritten with all of it.
   `REMAINING-WORK.md` has no D9 row today (grep) — if the coordinator adds one at the merge, it
   should take the completion draft's wording. The brief itself is left as written (a dated plan).
4. **No pin of current/totalCurrent/temperature against the old code — REAL, fixed.** N11
   (verbatim copies, 200,000 random all-finite groups, self-checking power). N3's second draw
   (100,000 even arrays past DBL_MAX/2, both branches REQUIREd). The overflow wording fixed in the
   header and the changelog ("a middle pair whose sum overflowed … +Inf or -Inf").
5. **pushTravelRad NaN for a NO READING port — REAL, fixed in code** (+0.0 for any port with no
   sign) and the doc made exact; the -0.0 → +0.0 side effect for a still port read backward is
   logged above and in the changelog. Pinned: N10 (a), (d), and the new (e).
6. **(major) Dropping pushSummary()'s new std::abs survived — REAL, fixed.** N12 (a) robot two's
   real wiring and (b) a backward push answered BACK-first.
7. **The NaN guard keyed on the sum survived — REAL, fixed.** N5's 0 A subcase.
8. **The " | " before a second no-reading sentence untested — REAL, fixed.** N10 (d), exact text
   (223 characters, inside warning()'s 256 — the cut itself is the logged Done-screen item).
9. **movedOnPush's inclusive threshold untested — REAL, fixed.** N12 (c).
10. **-ffast-math / -ffinite-math-only would silently undo the change — REAL, fixed.** The `#error`
    under `__FINITE_MATH_ONLY__` beside `isUsableReading()`; measured (above); listed in the
    changelog as a CHANGED build constraint with a migration line.
11. **Guide 11's "by a little" — REAL, fixed.** Both "several members jumped together" sentences
    now name the membership step; the shift is sized and its bound stated; HA-165 linked.
12. **Four changelog statements not exactly true — REAL, all four fixed:** (a) "where it read
    `still` (a NaN) or `MOVED` (±Inf)"; (b) "except the one overflow under CHANGED"; (c) "until
    infinities of one sign made up half the members or more (a single one on a side of one or two
    members): the side then read ±Inf, or NaN where both signs met" — more exact than the
    suggested "two of them on a group of three or more", which is false for five (three are
    needed); (d) the last migration line names the two exceptions.
13. **The changelog said the whole sentence is "shown on the Done screen" — REAL, fixed:** it now
    says the screen shows the first two 50-column lines and that the serial summary prints
    `warning()` only beside a LEFT OUT port (so the PUSH TABLE line is what names it there). The
    wording itself is still the team lead's to change (logged 10:44 on 09-29).
14. **The odd-count sentence misdescribed the middle member left out — REAL, fixed** (with #1),
    in the header and the test comment; pinned by N7's middle-member subcase.
15. **"Merge expected to be clean" was stale — REAL; corrected by a trial merge** (below).
16. **NON_DRIVE_MOVED told a no-reading port it "did not move on the push" — REAL, fixed at the
    root in the reason text** (the brief adds no Refusal code and none is added: the same code,
    the reason keyed on the same `isUsableReading(pushDeltaRad)` test as the NO READING word).
    Both reasons fit the Aborted screen's three 50-column lines (measured, every width up to
    -999.999 rad/s). The panel's headline words are `calibrate_panel.hpp`'s (out of scope):
    logged as a follow-up. Pinned: N10 (b); P2 and P4 RED.
17. **Landmine 10 not met in two headers; port 18 stated past HA-130 — REAL, fixed:** the
    PushCapture phase text and the odometry comment now say host test plant only / not expected
    on a V5 (HA-08); `motor_group.hpp` says "travelled ~20 % short on a hand push (HA-130;
    unmeasured under power), so its offset would grow"; HA-165's confidence says the same.
18. **The 03:47 re-check skipped D7's reader sites — REAL, fixed in the log** (the 09:15 entry
    above, every site with its line now; two moved, as the reviewer said).
19. **"Stopped at the team lead's pause" — REAL, fixed** in the completion draft: "stopped at a
    session usage limit (~13:50), inside its final gates".

#### 09:32-09:42 (10-01) — the gates, final (`scratch/diag1b/fix3/gates.sh`; every output in `fix3/gates/`)
Run after the last header, test and public-doc edit. The order the task sets: `api_doc_tool
generate` → the briefing regenerated → `cmake -S test -B build/test` → the three headers and every
`test/*.cpp` touched → the FULL build (its doc gates inside; "Linking CXX executable shulib_tests"
present; 0 warnings) → the suite → the briefing regenerated again → `check` → every check by hand.
- **Suite: 1737 | 1737 passed | 0 failed | 3 skipped, 2,669,045 assertions.** Per test against
  the 09-29 final xml: the **1725 pre-existing cases carry 2,648,032 assertions in both, 0 moved**;
  the N cases: N1 1,577, N2 1,348, N3 12, N4 6, N5 4,993, N6 144, N7 58, N8 10,806, N9 1,656, N10
  273, N11 14, N12 126.
- `briefing_status.py` generate (twice) and `check`: current. `api_doc_tool`: self-test OK;
  check-coverage — 2937 public entities across 156 headers, all documented; check-fresh — exit 0, no
  output; check-examples — 467 quoted lines, all verbatim; check-removability — no public doc
  references `docs/internal/`. `doc_staleness_audit.py`: self-test OK (10 cases); audit clean (27
  live docs).
- `src_build_gate.py`: self-test OK (189 detector cases, each against a real build); check **PASS —
  all 6 builds link**, 0 disallowed warnings (it cleared and rebuilt the `bin/` the `-ffast-math`
  probe build had left).
- The ARM header gate: 192 headers, `-Werror … -mfloat-abi=softfp`, exit 0, no output.
- The CI greps (the workflow's exact commands): PROS-free — empty; sim-free — empty. The C7 grep of
  `README.md`, `docs/*.md`, the guide and the cookbook for `internal/`, `chunks/`, `RESUMING`,
  `build-order`: empty.
- Register reconciliation: direction 1 (`PROVISIONAL (A4` with no HA id) — empty; direction 2 —
  `HA-165` in `motor_group.hpp` (1) and `test/median_nonfinite_test.cpp` (4), the two files its
  Source names.
- New string literals: the second `NON_DRIVE_MOVED` reason (7-bit ASCII; a format string whose
  output carries no `%` — N10 (b) asserts `screenSafe(reason())`) and the `#error` message (ASCII;
  never reaches a screen).
- `git status`: the three headers, the docs, the regenerated `docs/api/` pages and the briefing
  modified; `test/median_nonfinite_test.cpp` and `DIAG1-COMPLETED.md` new; **`src/`, `tools/` and
  the `Makefile` clean**; nothing committed.

#### 09:44 (10-01) — where this run stops (the review round is finished; nothing committed)
- **All nineteen findings are real, and all nineteen are fixed above** — at the root in this
  worktree (code, tests, docs or the log); #2 as its reviewer asked, by documenting the changed
  verdict everywhere it belongs, with the policy question (what `OdoStallCheck` should do with a
  travel it cannot read) left as a named follow-up. Every fix has its evidence: a test that fails
  without it (35 mutations, each RED on a fresh binary), a probe on both trees, or a measurement.
- **Decisions in this round for the coordinator / the team lead** (each reversible, each logged
  with its reason): (1) the `#error` under `__FINITE_MATH_ONLY__` — a new build constraint on
  anyone including `hal/motor_group.hpp`, listed in the changelog as CHANGED with a migration line;
  no build in the repository sets the flags; the alternative was a grep in a gate, which an outside
  team's build would never run. (2) `NON_DRIVE_MOVED`'s second reason text — the same refusal code,
  so no `Refusal` was added (brief §4); the panel's words stay. (3) `pushTravelRad` is +0.0 for
  every port with no sign — including a still port read backward, which was -0.0; no reader can tell.
- **Merge notes** (the 09:13 entry above, against `ce95325`): the changelog section moves to the top of
  `## API 2.4` as "Changed since 2.4"; the register's status becomes 7 of 162, its paragraph gains
  this chunk's sentence beside COMP-t's and L9's, and the index rows go 161, 165, 167, 177, 178,
  179; guide 11 merges clean; regenerate `docs/api/` and the briefing.
- **Still open, unchanged by this round:** the Done screen cutting the no-reading sentence (the
  wording is the team lead's), the program's serial summary printing `warning()` only beside LEFT
  OUT, D6, D9 (now described correctly in the completion draft), the `OdoStallCheck` decision, and
  the `src/` comment follow-ups (`teleop_program.cpp` :812-813, :847).
- **Nothing has run on a robot.**

## The coordinator's merge — DIAG1 Part B (2026-10-01, 09:40–10:20 EDT; DIAG1 COMPLETE)

- **The run:** the build (2026-09-29), three reviews and a fix round, both cut by the account's usage limits
  (2026-09-29 ~22:20 and 2026-09-30 ~04:15) and resumed with continuation notes; the fix round found its own
  stopped predecessor's edits, kept every one but a wrong N7 expectation, and closed all 19 findings
  (35 mutations red). Transport commit `f246319` in the worktree `diag1b`.
- **The merge** onto `ce95325`: the changelog (this part's section moved under `## API 2.4` and re-headed
  "Changed since 2.4 (2026-10-01)" — L9's 2.4 landed first) and the register (status 7 of 162; the sentence
  after COMP-t's and L9's; rows in numeric order) resolved by hand; `docs/api` and the briefing regenerated;
  build-order (the DIAG1 deviations row, the Next line), HANDOFF (DIAG1 COMPLETE) and REMAINING-WORK (row 27,
  the totals, the position — 33 of 63 chunks, DIAG1 the latest) brought to the merged state.
- **Gates on the merged tree:** suite 1,836 | 1,836; src gate self-test (198) and check PASS on all seven
  builds; ARM header gate (193); CI greps; check-fresh, briefing, removability, staleness — all clean; no plant
  (`.claude/coordinator/logs/diag1b_merge_verify.log`).
- **Seven coordinator mutations** (`.claude/coordinator/muts_diag1b.json`), each RED on a fresh binary — the
  list is in `DIAG1-COMPLETED.md`, "The coordinator's merge verification". Committed with this record.
