# DIAG1 — COMPLETED (Part A landed 2026-09-26; Part B landed 2026-10-01; host- and build-gated; NOT run on a robot)

> Drafted by the Part B executor and completed by the coordinator at the merge (the Part B brief, §3:
> "DIAG1-COMPLETED.md is owed by DIAG1 as a whole, and Part B closes DIAG1"). Written FROM
> [`DIAG1-PROGRESS.md`](DIAG1-PROGRESS.md) — Part A's log, its review round and the coordinator's
> merge record, and Part B's section. Briefs:
> [`DIAG1-odometry-tick-names-its-check.md`](DIAG1-odometry-tick-names-its-check.md) (Part A) and
> [`DIAG1-partB-median-nonfinite.md`](DIAG1-partB-median-nonfinite.md) (Part B). Executor model:
> `opus` for both parts, each in its own worktree (`.claude/worktrees/diag1`, `.claude/worktrees/diag1b`).
> **Nothing in DIAG1 has run on a robot.**

## The one-sentence honest status

An implausible odometry tick now names the check that refused it and the values it read (Part A), and a
drive side read through a motor group now leaves out a member whose reading is not a number — the way it
leaves out a member that is not there — instead of handing `std::sort` a NaN (Part B); both are proven on
the host test plant only, and on robot two the Part B exclusion is expected never to fire, because the V5
motor adapter screens a failed read to the last good one first (HA-08).

## Part A — an implausible odometry tick names its check (landed 2026-09-26)

- **Built:** `diag/odom_tick_audit.hpp` (a diag leaf: `OdomTickAudit` — three flags, five values, the
  labels, `tripped()` — and its renderer, bounded at 140 bytes); `IOdometry::lastTickAudit()` (a fifth
  pure virtual); both odometries fill the audit from their trust gate's own sub-expressions and read the
  verdict FROM it (`implausible_ = audit_.tripped()`), so the verdict and its account cannot drift;
  `Localizer::lastOdomTickAudit()`; `HealthMonitor::Observations::odomAudit` (appended) and the
  rising-edge `ODO_STUCK` line `implausible odometry tick why=… dth=… h0=… h1=… L=… R=…`. HA-160 (the IMU
  heading is continuous tick to tick — invented, and questioned by the 2026-09-25 capture) and HA-123's
  amendment.
- **Tests:** `test/odom_tick_audit_test.cpp` — `DIAG1 A1..F3` (formatter goldens and bounds; both
  odometries' accounts, their boundaries and 2000-tick oracle sweeps; the monitor's rising edge; the
  forwarders; end to end on a drive-encoder tank on the host test plant).
- **Mutations:** 18 in the build run plus the review round's, each on a fresh binary, observed RED and
  restored (the table is in the log, Part A "10:13"; M05a, first called an equivalent mutant, was shown
  in review round 1 NOT to be one: B10 kills it, and C9 kills the same drop in the tracking-wheel
  odometry, M05e).
- **Merged** by the coordinator (squash from `32725d1` onto `cc513a7`): suite 1568 | 1568 passed, every
  gate green, seven coordinator mutations RED on fresh binaries (the merge record in the log).

## Part B — a drive side's median leaves out a non-finite member (built 2026-09-29; review round 2026-09-30 – 10-01)

- **Built** (the team lead's answers: every recommendation D0–D9 accepted, 2026-09-26 ~23:10):
  - `hal::isUsableReading(double)` — THE test (`std::isfinite`). Since the review round, a build that
    sets `-ffinite-math-only`, `-ffast-math` or `-Ofast` is refused by an `#error` beside it: under
    those flags the compiler may fold the test to `true` (measured on the robot's `arm-none-eabi-g++`
    13.2.1), which would put every NaN back into the median.
  - `hal::medianInPlace` copies the finite values forward in order, sorts only those, takes an even
    count's midpoint with `std::midpoint`, and returns a fresh quiet NaN when none is finite (D1-D4).
  - `hal::MotorGroup`: position/velocity through that median; `current()`, `totalCurrent()` and
    `temperature()` over the finite readings, NaN when none (D7); `nonFiniteReads()` (D8).
  - The drivetrain calibrator: ONE private "moved on the push" predicate for its classification and its
    live count; a candidate whose push reading is not finite is not a drive port, and is named — `NO
    READING` in the PUSH TABLE and a sentence in `warning()` (D5). Since the review round: a port with
    no sign has `pushTravelRad` exactly +0.0 (it was sign × reading — a NaN for a port with no
    reading), and `NON_DRIVE_MOVED` on a port that gave no reading at DONE says so, instead of "it did
    not move on the push".
  - HA-165 (the membership step — invented; broadened in the review round to several members left out)
    and HA-08's amendment.
- **Two behaviour changes a reader would not guess** (host test plant only; neither expected on a V5,
  HA-08): a side whose infinite members used to reach the middle (two `+Inf` of three, say) read
  infinite and froze its tick as `NONFINITE+TRAVEL`; it now reads the rest's median and integrates —
  silently under 36 in (HA-165). And `OdoStallCheck`, on a drivetrain with an independent motion source,
  gives no verdict for a window that closes (or is baselined) on a drive side with no finite reading,
  where an all-`±Inf` side read as an infinite spin travel and raised `ODO_STUCK`.
- **Tests:** `test/median_nonfinite_test.cpp` — `DIAG1 N1..N12`: every placement against an independent
  oracle; the fresh NaN; all-finite arrays bit-identical to the old function (200,000, and since the
  review round 100,000 more past DBL_MAX/2); the order of equal values; a non-finite member reads
  exactly as an absent one on every read (and a side at 0 A reads 0, not NaN); through the odometry;
  the membership step exact (one member, the middle one, several); a 1200-tick oracle sweep; end to end
  through the scheduler on the host test plant; the calibrator (one and two ports with no reading, the
  net, a still port's +0.0 travel, the spin); N11 — the group's current and temperature reads against
  verbatim copies of the old ones; N12 — the push over robot two's real wiring, a backward push answered
  BACK-first, and a travel exactly at the threshold. On the unchanged tree N1–N10 are RED — N9's NaN
  sub-run is the evidence the case sees the defect (the `moveTo` aborted on `ODO_STUCK` the tick after
  one member of five went NaN) — and N11–N12 are GREEN: regression pins for the reads and the push rule
  the change rewrote.
- **Mutations:** M1–M19 from the brief plus M20–M22, and the review round's thirteen — S1 S2 S3 S7 S8
  MA1 (six that SURVIVED the reviewers' runs against the earlier tests) and MT1 MS1 MW P1 P2 P3 P4 —
  each on a fresh binary, observed RED, restored; none survived (the tables are in the log, Part B).
  On the unchanged tree the first run also showed a `-Inf` push reading that recovers saving a
  description with the port on the WRONG side — closed by D5.
- **Verified three times.** The build run stopped at a session usage limit (~13:50), inside its final
  gates. A resumed run (19:01–21:39) re-read the diff; it changed no code, only four prose lines. It
  re-ran the unchanged tree against the FINAL test file (all ten RED, every sub-run outcome as first
  logged) and the §0 probe (the same numbers). It re-ran all 22 mutations on the final tree: the same
  RED cases and the same failed-assert counts, every one. It also re-ran every gate. Then three
  reviewers returned nineteen findings, all real. The fix round's first run was stopped by the
  account's usage limit and two resumed runs stopped before finishing; the fourth finished on
  2026-10-01 (the log's "DIAG1-B review findings and their disposition").
- **Verdict intact, measured:** the per-test assertion counts of the 1725 pre-existing cases — 0 moved,
  2,648,032 assertions in the unchanged tree's run and in every run since.
- **Gates in the worktree (2026-10-01, after the review round):** suite **1737 | 1737 passed | 0
  failed | 3 skipped, 2,669,045 assertions** (+12 cases on the 1725 before Part B); the full build's
  doc gates; `src_build_gate.py`
  self-test OK (189) and check PASS (6 builds link, 0 disallowed); the ARM header gate clean (192
  headers) — and refusing the same headers under `-ffinite-math-only` at the `#error`, as a real `make
  ROBOT=tank PROGRAM=calibrate EXTRA_CXXFLAGS=-ffast-math` does; both CI greps empty; every
  `api_doc_tool` check and the staleness audit clean; reconciliation direction 1 empty, HA-165 found
  in both source files.
- **Deviations from the brief** (each logged with its reason): the changelog section is "since 2.3"
  (TANK1 bumped the API; at the merge it goes under 2.4); `totalCurrent()` is NaN, not 0, with no finite
  member; the shared helper is the public `hal::isUsableReading()`; N10's injection path adds one
  watched tick; N10 (c) poisons every CCW spin tick rather than locating the finishing one; and from the
  review round, the `#error` build constraint, the second `NON_DRIVE_MOVED` reason, `pushTravelRad`'s
  +0.0, and N11/N12 beyond the brief's N1..N10.

## The coordinator's merge verification — Part B (2026-10-01, 09:40–10:20 EDT)

Part B's run — the build (2026-09-29), three reviews and a fix round, both interrupted by the account's usage
limits and resumed with continuation notes (19 findings, all real, all fixed; 35 mutations, all red) — was
committed in the worktree as `f246319` and squash-merged onto `ce95325`. Two hand-resolved conflicts: the
changelog (this part's section moved to the top of `## API 2.4` as "Changed since 2.4", since L9 Part A1's 2.4
landed first) and the register (the status line "7 of 162"; this part's sentence after COMP-t's and L9's, the
running total 162; the index rows in numeric order — HA-161, 165, 167, 177, 178). `docs/api` and the briefing
regenerated. On the merged tree: the suite **1,836 | 1,836**, 3 skipped; `src_build_gate.py` self-test OK
(198) and check PASS on all seven builds; the ARM header gate (193 headers); both CI greps; `check-fresh`,
the briefing (self-test and check), removability and the staleness audit — all clean, no plant left. **Seven
coordinator mutations, each on a fresh binary, all RED:** an infinite reading counted as usable (9 cases); an
even count taking the upper middle (8); no usable reading giving 0 instead of NaN (5); the mean current divided
by every member (3); a non-finite temperature not counted (1); a non-finite push travel counted as moved (1);
a port with no sign carrying 0 × its delta, a NaN for a no-reading port (1).

## Found by DIAG1 and not fixed here

- (Part A) the test loops' IMU-ready gate vs the library's ungated path; `TrackingWheel`'s unchecked
  offset/diameter finiteness.
- (Part B, D6) the coupled-side monitor and a non-finite member VELOCITY: a `+Inf` member becomes the
  side's "fastest" and its healthy mates are flagged; a NaN member is never flagged. Its own chunk, after
  T3 Part 2 / T4 Part 2 (the monitor's owners).
- (Part B, D9 — the description CORRECTED in the review round; the brief's D9 text was wrong for ±Inf)
  the calibrator's comparisons against a LIVE reading, unchanged by Part B, and what each does today:
  - the rest check (`firstMoving`) and NonDriveMoved use `std::abs(velocity) > floor`: a NaN velocity
    reads as still, a ±Inf one as moving;
  - the pulse's `own` is sign × (reading − base), so it is NaN only when the base is bad too. A reading
    that goes `+Inf` during the port's OWN pulse gives `own = +Inf`, which PASSES the sign check (`own >
    0`): the side is taken from the yaw and the push sign counts as confirmed by PROS's failure
    sentinel — no refusal, no warning. At the top level with the yaw under the floor the same `+Inf`
    is classified Decoupled and LEFT OUT with the "gear is slipping" warning (a misread of what would
    be PULSE_STILL). `-Inf` trips `own <= -movedThreshold`: PULSE_CONTRADICTION with no yaw at all. A
    NaN is Weak below the floor and PULSE_CONTRADICTION only at a level that turns the robot;
  - `others` (the largest side-mate travel) ignores a NaN (`std::max`) and takes a `+Inf`: a `+Inf`
    side-mate at a top level that does not turn the robot gives PULSE_ROLLED, blaming the PULSED port;
  - the no-reading warning (D5) names a port only when its reading is bad AT DONE; one that goes bad
    after DONE is not named by it.
  Observed on the host test plant (port 12's reading bad during its own pulse only, its base finite),
  identical on the old and the new headers: `+Inf` → Done, port 12 saved on the LEFT; `-Inf` →
  PULSE_CONTRADICTION at 3 V with the yaw at 0; NaN → Weak at 3 V and 6 V, PULSE_CONTRADICTION at 9 V.
  (The Decoupled and Rolled outcomes are read from `finishPulse`, not probed.) Each needs its own
  refusal design — a follow-up together with D6's monitor fix. Not reachable on a
  V5 (HA-08).
- (Part B, review round) `motion::OdoStallCheck` and a spin travel it cannot read: on a drivetrain with
  an independent motion source, a drive side with no finite reading now gives the check NaN, so it
  gives NO verdict for that window, nor the next (its new baseline is the NaN); before, a side all
  `+Inf` or all `-Inf` read as an infinite travel and raised ODO_STUCK. Arguably better (a sentinel is
  not travel), but a loud signal became silence: what the check should do with a travel it cannot read
  is a decision for the D6/D9 follow-up. Robot two's check gives no verdict at all (HA-131).
- (Part B, review round) NON_DRIVE_MOVED on a port that gave no reading at DONE: its reason now says so,
  but the panel's words for the code stay "WRONG MOTOR MOVED / a motor off the drive turned / check
  the port shown, RUN AGAIN" (`calibrate_panel.hpp`, out of Part B's scope), and the Aborted screen does
  not show `warning()`.
- (Part B) the calibrate program's serial summary prints `warning()` only beside a LEFT OUT list, and the
  Done screen shows two 50-character lines of it, which cut the no-reading sentence's last words ("run
  again"). `warning()` itself holds 256 bytes, and the no-reading sentences come first (DONE, before
  SideAssign). So with two or more of them, a slipping port's sentence is cut, although
  `excludedPorts` still names that port. None of it is reachable on a V5 (HA-08).

## What DIAG1 does NOT claim

It does not explain the 2026-09-25 `ODO_STUCK` episodes (Part A makes the next one name its check; Part B
is contract hardening). No threshold was measured. The next `ODO_STUCK` on robot two is the first real
test of the line; nothing here has driven a robot.
