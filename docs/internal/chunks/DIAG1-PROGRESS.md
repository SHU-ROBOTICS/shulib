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
