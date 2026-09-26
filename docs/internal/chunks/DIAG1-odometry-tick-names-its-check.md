# DIAG1 — an implausible odometry tick names its check and its values

> **Execution brief.** Written 2026-09-26 (morning) from a read-only research report (the odometry, health,
> fault, IMU and median code, and the 2026-09-25 capture), checked by the coordinator. **The team lead's go
> (2026-09-26): "Yes, as recommended"** — this design; the median's non-finite handling as its OWN later
> step (Part B, excluding non-finite values like ABSENT members); no episode-end summary for now. Built in
> an isolated worktree in parallel with T3, G1 Part 2c and COMP-a. **One executor run. Executor model:
> `opus`. Do not commit.** Live log: `DIAG1-PROGRESS.md`, created FIRST.

## 0. Why
On 2026-09-25 robot two raised `fault=ODO_STUCK ... implausible odometry tick` twice (start 2, t=34.40 and
t=60.32). The line carries no check name and no values, so the cause is unknown — and **ODO_STUCK is the
scheduler's default abort fault** (`motion_scheduler.hpp` ~:233): a repeat during the first autonomous would
end the drive. It must be explainable before the first auton.
- The chain: `drive_encoder_odometry.hpp` ~:164-166 (the gate: non-finite delta, |dTheta| > maxTickRotation,
  |side travel| > maxTickTravel) → `IOdometry::lastDeltaImplausible` → `Localizer::lastOdomDeltaImplausible` →
  `motion::tickHealthObservables` (`motion.hpp` ~:260) → `HealthMonitor::tick` (`health_monitor.hpp`
  ~:153-162), the ONLY OdoStuck raise site, with the fixed text. `PilonsOdometry` has the same gap
  (`pilons_odometry.hpp` ~:143-150; its accessor comment ~:185 is stale).
- **The binding size limit is `FaultLatch`'s `buf[176]`** (`fault.hpp` ~:138-141): after a worst-case
  35-byte prefix the detail may be at most **140 bytes**; `snprintf` truncates silently.
- The research's reading of the capture (an INFERENCE, not a finding): episode 1 (34.40) was at rest;
  **episode 2 (60.32) fired on the first zero-command tick right after ~0.35 s of reversing at up to
  33.8 in/s**, and the published heading swung 17-37° between consecutive 10 ms records just before it — the
  IMU heading path is the prime suspect. `R3d-PROGRESS.md`'s 2026-09-25 entry says both were at rest: correct
  it in this chunk's doc pass.

## 1. Scope — IN (Part A)
1. **`include/shulib/diag/odom_tick_audit.hpp`** (a leaf): `enum class OdomTravelLabels : uint8_t
   {DriveSides, TrackingWheels}`; `struct OdomTickAudit { bool nonFinite, rotation, travel; units::AngleDim
   dTheta, headingBefore, headingAfter; units::Length travelA, travelB; OdomTravelLabels labels; constexpr
   bool tripped() const noexcept; }`; `kOdomAuditDetailMaxBytes = 140`; `appendOdomTickAudit(lineformat::Line&,
   const OdomTickAudit&)` built on `lineformat::appendNum` (the ONE formatter: NaN/+Inf/-Inf tokens,
   compaction) — never `snprintf %f`. ASCII only.
2. **`IOdometry` gains `[[nodiscard]] virtual diag::OdomTickAudit lastTickAudit() const noexcept = 0`** —
   "the trust gate's account of the LAST update(): every tick, not sticky". `odometry.hpp`'s "exactly these
   four members" prose is rewritten to say why a fifth was added. (IOdometry is not in the Freeze Register:
   no version bump; three in-tree implementers incl. the test's `ScriptedOdometry`.)
3. **Both implementations fill the audit every `update()`**, the flags being TODAY'S gate sub-expressions
   copied as they stand — `nonFinite = !finite; rotation = |dTheta| > max; travel = !(|a| <= max && |b| <=
   max)` (so a NaN side reports NONFINITE+TRAVEL, documented) — and then `implausible_ = audit_.tripped()`,
   keeping the VERDICT bit-identical by construction (the existing suite is the pin). DriveEncoderOdometry:
   a = dL, b = dR, DriveSides, headings h0/h1 in radians. Pilons: a = the RAW fwdDelta, b = latDelta,
   TrackingWheels.
4. **`Localizer::lastOdomTickAudit()`** forwards; **`HealthMonitor::Observations` APPENDS `diag::OdomTickAudit
   odomAudit{}` as its LAST field** (designated initializers require declaration order); `tickHealthObservables`
   sets it last.
5. **`HealthMonitor`**: the trigger stays `odomImplausible || odomStalled`, unchanged — the audit is DETAIL
   only, rendered ONLY in the rising-edge branch: `implausible odometry tick` + (tripped ? the rendering :
   ` why=UNREPORTED`); when stalled and implausible rise together: the valued implausible text + ` +STALL`
   (the team lead's "as recommended"). Nothing formatted on non-edge ticks.
6. **The exact text** (a golden test pins it): ` why=` + tokens joined by `+` from {NONFINITE, ROT, TRAVEL}
   in that order + ` dth=<n>rad h0=<n> h1=<n>` + ` L=<n>in R=<n>in` (tracking wheels: ` fwd=<n>in
   lat=<n>in`), `<n>` = `appendNum` precision 3. Example: `fault=ODO_STUCK n=1 FIRST implausible odometry
   tick why=ROT dth=1.745rad h0=0.012 h1=1.757 L=0.000in R=0.000in`. Worst case ≤ 140 bytes.

## 2. Tests (adversarial) and mutations
For BOTH odometries: rotation only (exact dth/h0/h1); travel on the LEFT only and on the RIGHT only (no
channel swap); both bounds at EXACT equality not tripped, one ulp above tripped; NaN left ⇒ NONFINITE+TRAVEL,
L=NaN, position frozen; a finite-but-huge jump ⇒ TRAVEL only, position integrated, value compacted; an
overflow to +Inf ⇒ NONFINITE+TRAVEL; ROT+TRAVEL in one tick names both; a clean tick after an implausible one
clears the flags (not sticky); a ≥1000-tick scripted sweep asserting `lastDeltaImplausible() ==
lastTickAudit().tripped()` every tick; Pilons carries the RAW fwdDelta. Group level: one member of four jumps
1000 rad ⇒ NOT implausible (the median holds); two jump ⇒ TRAVEL with L = the median's value. HealthMonitor:
500 implausible ticks with CHANGING audits ⇒ exactly ONE line with tick 1's values; clean then implausible ⇒
a second line (n=2); empty audit ⇒ `why=UNREPORTED`; flags set but `odomImplausible` false ⇒ no raise; the
+STALL case. Formatter: goldens, deterministic tokens, worst case ≤ 140, 7-bit ASCII. Localizer forwards
bit-for-bit. End to end: `Chassis::drive` at rest with the IMU stepped +100° ⇒ exactly one ERROR line equal
to the golden; alternating ±100° for 50 ticks ⇒ one line; a `MoveToPose` during an IMU step still exits
FAULT_ABORT (unchanged) and its line carries the detail. **Mutations** (fresh binary each; logged): swap the
rotation/travel flags; travel as |d| > max; L/R swapped; audit not reset; `implausible_` computed apart from
the audit with one term dropped; Pilons reports the centre value; the raise outside the rising edge; trigger
on `audit.tripped()`; `snprintf %f` for appendNum; compaction removed; the Localizer forwarder returns `{}`;
`tickHealthObservables` drops `.odomAudit`; an equality boundary flipped; the +STALL precedence inverted.

## 3. Documentation (same change)
`docs/guide/11-reading-the-diagnostics.md` (the ODO_STUCK row gains the trust-gate cause and the why= tokens —
dth in radians, 1.571 = 90°, defaults π/2 and 36 in — and a valued example; its "check the tracking wheels"
advice is wrong for a drive-encoder robot); ch.12 ~:127-128 and ch.06 ~:44-52 read and fixed if stale ("In
simulation" → "on the host test plant" if edited); `docs/diagnostics-plan.md` (the gate now reports which
side and values; do NOT claim the E-phase detector); `docs/hardware-assumptions.md` — HA-123 extended to
drive-encoder sides with its blast radius corrected (a spurious trip ENDS the active motion) and a NEW entry
at the next free number AT COMMIT TIME (T3 is taking HA-147..159 in parallel — use **HA-160**): "the V5 IMU
heading is continuous tick to tick while ready", status "questioned by 2026-09-25 (reconstructed swings of
17-37° per tick in one start); cause unknown"; public docs cite evidence by date, never by a
`docs/internal` path; the `R3d-PROGRESS.md` correction (episode 2 after a hard reverse, with the strings line
numbers); `docs/api` regenerated; the briefing regenerated LAST.

## 4. OUT
The gate's VERDICT (bit-identical); new FaultCodes; DebugRecord / SHUL/2 fields; PoseDeltaGuard in teleop; the
host test loops' ready-gate divergence (log it as found-while-reading); Part B (the median's non-finite
handling — its own later step); any `src/` change (every program raises through `tickHealthObservables`); any
hardware claim.

## 5. Checks
Suite green (counts); every mutation RED on a fresh binary; `src_build_gate.py` self-test + check; the ARM
header gate; CI greps; `api_doc_tool` self-test / coverage / check-fresh / check-examples / removability;
`doc_staleness_audit.py`; briefing regenerated LAST and checked.

## 6. Landmines
1. `Observations` uses designated initializers: APPEND the field at the end or every call site breaks.
2. Copy the gate's sub-expressions as they stand — never re-derive them (the verdict must not move).
3. `FaultLatch` truncates silently at 175 bytes.
4. HA numbering collides with parallel chunks — use HA-160; the coordinator renumbers at merge if needed.
5. No `docs/internal` path in a public doc (C7 removability).
