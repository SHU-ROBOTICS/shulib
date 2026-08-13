# E2 — `GpsCorrector` — progress log

Live log. Appended continuously, never batched. Timestamps are wall clock (`date +%H:%M:%S`).

Branch `shulib-v2`, starting at `73b8e7f` (tree clean, suite 752 cases / 936,895 assertions,
ARM gate clean at 110 headers).

---

## 22:17:06 — start

Chunk E2 begins. This is the first real corrector: everything before it dead-reckoned, and this
is the first code in the library that can tell the estimator it is wrong.

Opening move per the brief: create this log, then required reading in the stated order, then rule
T1/T2/T3 **before** writing the gate.

Two traps already in the register that I am carrying into the reading:

- **HA-07** — `pros::Gps::get_error()` returns **metres**; this library is **inches**. Missing the
  conversion is a factor of 39.37 that never crashes: it either gates everything or trusts
  everything. Must be pinned against a hand-computed absolute, never against another constant
  drawn from the same header.
- **HA-01** — GPS axes are +X East / +Y North; F1's frozen field frame is origin = field centre,
  +X right, +Y away from red. Not automatically the same frame. Must be pinned with an
  independent from-scratch oracle, because a frame error cancels perfectly in any test that
  applies the same conversion to both sides — the exact blindness that bit C1, C3 and C4.

Reading list next.

## 22:18:50 — required reading done, and the first surprise

Read in order: the brief, `i_corrector.hpp`, `correction.hpp`, `gps_conversion.hpp`,
HA-01/07/10/26–31, E1-COMPLETED's T3 section, `degradation.hpp` + `gps_hostility.hpp` +
`fake_gps.hpp`, `debug_record.hpp`, `RESUMING.md`. Also read (not on the list, but load-bearing):
`localizer.hpp`, `complementary_fusion.hpp`, `hal/gps.hpp`, `fake_corrector.hpp`,
`motion_scheduler.hpp`'s `CommandIdStampSink`, `test/CMakeLists.txt` (the four doc gates).

**The surprise, and it changes the shape of the chunk.** The brief hands the corrector
"frame/lever-arm reduction" as its own mess. But three existing contracts say the opposite, and
they say it in writing:

- `hal/gps.hpp:26` — `IGps::pose()` is "the robot-CENTER pose (lever-arm corrected)".
- `gps_conversion.hpp:3-5` — this header is "the ONE place the VEX GPS frame becomes shulib's
  canonical frame (§7: convert exactly once, at the edge)".
- `gps_conversion.hpp:29-34` — "Lever arm removed HERE (`gpsRemoveLeverArm`), **ONE owner** =
  robot config", with an explicit warning that a second subtraction is "inches of silent bias".
- `fake_gps.hpp:3-5` — "Stores a CANONICAL robot-center pose (the VEX-frame conversion is the
  adapter's job)".

So a `GpsCorrector` that converted frames or removed a lever arm would be the **second** owner of
both, and on the real robot (R1 adapter, which is contractually obliged to do it) it would
double-subtract. That is precisely the HA-06 failure the register already warns about.

Ruling to be written up as **T4**. Provisional: the corrector does **not** re-do either; E2's job
on the two traps is to *pin the code that does own them with independent oracles*, and to fix the
hole I found while checking (below), which is Rule 4 work in the layer that owns it.

**The hole (HA-07 has no code and no test).** `gps_conversion.hpp:39-40` states the metres→inches
obligation on `rmsError()` as **prose only**: "rmsError() at the HAL edge MUST scale get_error()
meters→inches (× kMetersToInches)". There is no function in the header that does it, and nothing
in the suite pins it. The single most dangerous conversion in the chunk — factor 39.37, silent —
is currently an instruction to a future adapter author. Next: confirm by grep, then close it where
it belongs (`gps_conversion.hpp`), with a hand-computed absolute.

**The introspection gap (E1's T3, the other half).** Traced the path: corrector → *(nothing)* →
`IFusionPolicy::fuse` → `FusionResult::audit` → `Localizer::lastCorrection()` →
`CommandIdStampSink::emit` → `DebugRecord.gateReason` → blackbox. The Localizer drops
`valid=false` proposals in step 3 (`localizer.hpp:162-173`), so **a corrector-side verdict has no
channel to the record at all**. With correctors registered but none producing a fix, the policy
sees zero proposals and the record says `None` — "no correction proposal this tick" — which is
exactly the quiet off-strip failure the brief calls match-losing. `RejectedNoFix` and
`RejectedHighYawRate` exist in the enum and are documented as "CORRECTOR-side verdicts that E2
fills in"; E2 has to build the channel that carries them.

## 22:26:40 — the rulings, before a line of gate code

### T1 — it is a NORMALIZED INNOVATION, and it will be called that

E2's gate computes

    nu = |z - predicted| / sigmaEff,   sigmaEff = hypot(sigmaMeas, sigmaDeadReckon)

and rejects when `nu > gateSigma`. `sigmaMeas` comes from the device's own `rmsError()` (adaptive
R); `sigmaDeadReckon` grows with the distance the prediction has travelled since this corrector
last had a fix accepted.

**This is not a Mahalanobis distance.** A Mahalanobis distance normalises by the innovation
covariance `S = H P Hᵀ + R`, with `P` *estimated by the filter*. The complementary tier estimates
no `P`; `sigmaDeadReckon` is a hand-written heuristic with an invented growth rate. Reporting the
ratio as `gateMahalanobis` would invite a reader to compare an E2 number with an E4 number as if
they meant the same thing. They do not.

So: `gateMahalanobis` stays **0** on every E2 path, `RejectedMahalanobis` is never raised, and the
append-only enum gains honest spellings.

**Rejected alternative — call it Mahalanobis anyway** (defensible on the grounds that with an
assumed isotropic `S = sigmaEff² I` the scalar ratio *is* the Mahalanobis distance). Rejected
because the assumption is the whole content: E4's number will be earned from a filter, E2's is
asserted by a constant, and one field holding both makes the difference invisible.

**Rejected alternative — a fixed inch threshold** (fully reconstructable from residual alone, no
new constants). Rejected because it cannot widen when the robot has been dead-reckoning for
twenty feet, which is the one moment a correction is worth most. That is gate lockout, and the
complementary tier is already exposed to it through `ComplementaryFusion`'s fixed 12" gate.

### T2 — what "never increases error" is allowed to mean here

Tick-by-tick monotone improvement is not achievable and I will not test for it. **What I will
prove**, on the A2 plant under A3's hostile GPS + IMU, across N seeded runs:

1. **final** position error, corrected ≤ dead-reckoned, per seed;
2. **worst-case** (max over the run) position error, corrected ≤ dead-reckoned, per seed;
3. **boundedness**: the corrected run's error in the second half of the run is not worse than in
   its first half, while dead-reckoning's is — drift stops growing without limit;
4. **per-tick budget honesty**: no tick moves the fused position further from the odometry-only
   prediction than `maxNudgeRate · dt`, ever.

**What I am NOT claiming:** not that every tick improves; not any absolute accuracy number; not
anything about real hardware (every magnitude in the hostile model is invented, HA-26…HA-31);
not that the `< 1°` spec is met — heading is untouched by this chunk.

### T3 — heading stays IMU-owned, and E2 makes that PROVABLE

`providesHeading` is never set. The GPS heading never leaves the corrector: the proposal's
`fieldPose` carries the **predicted (IMU) heading**, not the GPS's, so even a fusion policy that
someday read `fieldPose.heading()` would read the IMU's answer. Tested directly.

**Rejected alternative — pass the GPS heading through in `fieldPose.heading()` "since nothing
reads it".** Rejected: "nothing reads it today" is how a field becomes load-bearing by accident.

### T4 — NEW RULING: the corrector does NOT own frame or lever arm (the conflict from 22:18:50)

The brief assigns "frame/lever-arm reduction" to the corrector; `hal/gps.hpp`,
`hal/gps_conversion.hpp` and `hal/fake/fake_gps.hpp` all assign it to the HAL edge, with
`gps_conversion.hpp` saying **ONE owner** in capitals and naming double-subtraction as the failure.

**RULING: the HAL edge keeps both.** `IGps::pose()` is already canonical, robot-centre, lever-arm
removed; `GpsCorrector` consumes it as such and does no geometry. A corrector that re-did either
would be correct against `FakeGps` (which stores a centre pose) and silently wrong against the R1
adapter (which is contractually obliged to have done it already) — a bug that host tests could
never see, which is the worst kind.

**What E2 therefore owes the two traps is proof, not code:** independent from-scratch oracles
against the conversion functions that *do* own them, at several headings, plus closing the hole
below. This is Rule 4 — fix it in the layer that owns it.

### The HA-07 hole, confirmed

`grep -rn "kMetersToInches" include test` → the constant appears in `gps_conversion.hpp` (the
`gpsSensorPose` position scale) and in `test/gps_conversion_test.cpp`. **There is no function
anywhere that converts `get_error()` metres to inches** — the obligation exists only as prose at
`gps_conversion.hpp:39-40`, addressed to an adapter author who does not exist yet.

Worse: `test/gps_conversion_test.cpp:58,63,66,67,74,79,166,168` pin the position scale **against
`kMetersToInches` imported from the header under test**. That is exactly the shared-oracle
blindness the brief warns about — if the constant were 3.937, every one of those assertions would
still pass. The scale has never been pinned against a hand-computed absolute.

E2 closes both, in `hal/gps_conversion.hpp` where they belong.

### The verdict channel (E1's T3, the other half)

`CorrectionProposal` gains ONE appended field, `GateAudit selfAudit` — the corrector's own account
of a tick, which matters precisely when `valid == false` and the proposal never reaches a fusion
policy. The `Localizer` substitutes it into the tick's audit **only when the policy has no verdict
of its own** (`fr.audit.reason == None`), and names the corrector that produced it in
`AppliedCorrection::source`. Append-only, trailing, defaulted: every existing positional
construction still compiles and means the same thing — the same discipline E1 used to add
`GateAudit` to `FusionResult`.

**Rejected alternative — a new virtual `lastVerdict()` on `ICorrector`.** Rejected: it adds
temporal coupling (valid only immediately after `propose()`) to a seam Phases E–I depend on, when
the value is plainly per-proposal data that the existing call already returns.

**Rejected alternative — let the `Localizer` infer `RejectedNoFix` from "correctors registered,
zero valid proposals".** Rejected: it cannot tell off-strip from a yaw-rate rejection from a
stale sample, and with two correctors (E3) it cannot tell which source was silent.

### `GateReason` appends (all three honest, all append-only, all wire-stable)

    RejectedNormalizedInnovation = 6   E2's gate (T1) — residual too large for the sensor's own sigma
    RejectedStaleFix             = 7   the sample has not changed since it was last folded
    RejectedSensorQuality        = 8   the source's own error estimate is too large to be worth folding

Off-strip keeps the existing `RejectedNoFix = 4`; high yaw rate keeps `RejectedHighYawRate = 5`.
Both were reserved at A1 for exactly this and are documented as corrector-side verdicts.

## 22:39:30 — first accuracy run: 1 accepted fix out of ~570 fresh ones. Instrumented.

Wrote `gps_corrector_test.cpp` (22 cases, green first run), `gps_corrector_blackbox_test.cpp`
(green), then `gps_corrector_accuracy_test.cpp` — which failed with `accepted == 1`.

Did not guess; built a scratch instrumented run. Output:

    accepted=1 noFix=0 stale=2520 qual=0 yaw=57 inno=511
    t= 300 ... gpsPos=(-5.66,-12.09) pred=(-4.67,19.25) truth=(-5.13,-10.00)

**Two things learned, one a test bug and one a real limitation worth recording.**

**(a) My test harness bug.** `scriptedTwist(tick - kSettleTicks)` computes `(tick-300)/100 % 10`,
which is NEGATIVE for `tick < 300` — C++ truncates toward zero — so the robot was driving
through the IMU calibration window, where the Localizer's boot guard deliberately folds nothing.
The estimate froze while the robot drove 30 inches. Out of contract: `localizer.hpp` states it
plainly ("motion commanded before qualityClass leaves Uninitialized is unaccounted"). Fixing the
harness to hold still through calibration, as a real auton does.

**(b) The finding: the corrector's anti-lockout widening cannot rescue an estimate that is more
than 12 inches wrong, because the lockout is not the corrector's.** `ComplementaryFusion` has a
FIXED `innovationGate = 12.0` inches (`complementary_fusion.hpp:37`), applied after the
corrector's own gate. E2's widening can open the corrector's gate as far as it likes; a residual
above 12 inches is rejected one layer down regardless. Confirmed in this run — the estimate sat
29 inches out with a perfectly good GPS in view and never recovered.

This is exactly the "rare catastrophic stubbornness" guide chapter 3 already names. I am NOT
fixing it in E2: the constant belongs to the fusion policy, E4 replaces that policy with an EKF
whose gate is a real Mahalanobis test, and reaching up to retune another layer's threshold from
inside a corrector is precisely the kind of two-owner change this project's Rule 4 exists to
prevent. Recording it as a finding with its evidence, and stating in E2-COMPLETED what is
therefore NOT claimed: E2 bounds drift, it does not recover a grossly wrong estimate.

Also confirmed from the same run: **freshness detection behaves exactly as designed** — 2520
stale ticks against ~570 fresh in 30 s, i.e. ~19 fresh fixes per second against HA-28's modeled
20/s cadence. One measurement, folded once.

## 22:43:10 — T2 measured, and the claim has to shrink. This is the most important finding.

Fixed the harness (hold still through calibration) and re-measured. Then, before touching the
test, ran an 8-seed sweep in a scratch harness over two scenarios — a closed 10 s loop and a
mostly-outbound 60 s path (skills-length, which is what F2's spec is about). Numbers, inches:

    60 s, mostly-outbound, 1246" of path, 963 fixes folded per run
    seed   corrected fin / worst   dead-reckoned fin / worst
      1        0.043 / 0.594            0.314 / 0.546
      2        0.174 / 0.674            0.360 / 0.618
      3        0.771 / 0.771            0.560 / 1.607
      4        0.164 / 0.478            0.205 / 0.605
      5        0.095 / 0.385            0.211 / 0.707
      6        0.310 / 0.751            0.590 / 1.013
      7        0.122 / 0.477            0.205 / 0.702
      8        0.118 / 0.563            0.404 / 0.668

**Dead-reckoning in this simulation is already sub-two-inch over sixty seconds and 1246 inches of
driving.** A3's slip model degrades DRIVEN wheels (spin vs. body motion); the unpowered tracking
wheels read true body travel, so the only real error sources left are IMU heading drift
(±1°/min, HA-20) and encoder quantization. Meanwhile the modeled GPS noise is 0.7"/axis (HA-26),
about 1" radial.

**So the sensor's own noise is the same order as the drift it is correcting**, and the honest
consequence is: the corrector helps, but not on every seed. Final error is better on 7 of 8;
worst-case on 6 of 8. On seed 3 the corrected final error (0.771) is worse than dead-reckoning's
(0.560), even though the corrected WORST case is less than half of it.

**I am not going to scenario-shop until eight of eight pass.** The 30 s closed-loop run does pass
all eight — and picking it for that reason would be exactly the dishonesty the chunk loop exists
to prevent. Instead the claims shrink to what the data supports, and E2-COMPLETED says why:

- **Claim 1 (final error) becomes an AGGREGATE claim** across seeds, with the per-seed table
  reported and the one seed that goes the other way named.
- **Claim 3 (boundedness) is re-specified**, because half-versus-half of a 60 s run measures
  noise here, not trend. The property that actually distinguishes a corrected estimator from a
  dead-reckoning one — and the one guide chapter 3 teaches — is that **odometry cannot remove an
  error it has absorbed and a corrector can**. So: inject a known 6-inch position error, then
  run. The corrected estimate converges back; the dead-reckoned one carries it to the end of the
  run, forever. That is magnitude-independent logic, not a number about this simulation.
- The record states plainly that **whether a GPS corrector is worth folding at all depends on the
  ratio of sensor noise to drift, and BOTH numbers are invented** (HA-26 and HA-20). R4 measures
  them. This simulation is not evidence about a robot.

## 22:52:40 — MUTATION HOLE FOUND: 19 red, 1 GREEN

Ran `docs/internal/verify/verify-e2.sh` (20 mutations, gated on build success). Nineteen went
red. One stayed **green**, and it is a real hole:

    MUTATION: substitution OVERWRITES a real fusion verdict
      localizer.hpp:  if (tickAudit.reason == None && selfAuditSource != nullptr)
                  →   if (selfAuditSource != nullptr)

**Why the suite could not see it.** With ONE corrector the two versions are equivalent:
`selfAuditSource` is only set when a proposal is DECLINED, and a declined proposal means the
policy saw nothing, which means the policy's reason is already `None`. The guard is dead code
until a **second** corrector exists — and then it is load-bearing: corrector A proposes a fix the
policy ACCEPTS while corrector B is off-strip, and the mutated code stamps B's `RejectedNoFix`
over A's `Accepted`. The record would say the estimator dead-reckoned on a tick where it applied
a correction. **That is the exact class of invisible-wrong this chunk's T1 ruling refuses.**

E3 adds the AprilTag corrector, so this would have shipped as a latent bug that only appears one
chunk later, in telemetry, where nobody is looking. Every one of my Localizer-level tests wires
exactly one corrector — the blind spot was structural, not accidental.

Closing it with a two-corrector test that fails alone under the mutation.

## 22:52:10 — MY OWN PROCESS ERROR, and what it cost. Recording it because it is recoverable.

Two harness faults, both mine, both caught by looking at `git diff` instead of trusting the
mutation report:

1. **I ran `git checkout -- include/shulib/localization/localizer.hpp` to undo a mutation.**
   That file held UNCOMMITTED E2 work, so the checkout discarded the chunk's Localizer change,
   not the mutation. The brief forbids `git checkout` for exactly this reason and I did it
   anyway. Restored by re-applying the edit from the transcript.
2. **I piped the mutation script into `head`.** SIGPIPE killed it mid-mutation and left
   `gps_conversion.hpp` on disk with the metres→inches multiply DELETED — the very trap the
   chunk exists to guard. It then reported three mutations as `SKIP — pattern not found`, which
   I had written as a quiet one-line shrug.

**The tell was the second run's totals: 17 RED where the first run had 19 + 1 GREEN.** A drop in
the count of mutations that RAN is the signal; the report itself looked fine.

Hardened `docs/internal/verify/verify-e2.sh`:
- restore is ALWAYS from the byte copy taken before the edit, never `git checkout`, with the
  reason written in the script;
- a `trap` on INT/TERM/PIPE restores the file in flight;
- a SKIP is now LOUD and makes the script exit non-zero — an unfound pattern means a required
  mutation never ran, and "a mutation you reasoned about but did not execute does not count"
  applies to the harness too.

State verified clean afterwards: **794 cases / 1,081,382 assertions, all passing.**

## 22:59:30 — documentation contract discharged

1. Roadmap checkbox `[x]` with file/test/case/assertion evidence and the three scope caveats
   attached to the checkbox itself (not buried in prose).
2. Roadmap "you are here": E2 recorded with what it did AND did not; next pointer → E3.
3. Header design notes: `gps_corrector.hpp` (order of operations, the deliberate
   non-responsibilities per T4, the T1 reasoning, why sigma_dr exists, the double-count guard with
   its honest limitation, the latency reasoning), `correction.hpp` (why `selfAudit` exists and its
   contract), `localizer.hpp` (the substitution rule and why its guard is dead code TODAY).
4. Test evidence: `E2-COMPLETED.md` §6, every mutation named, the hole given its own section.
5. Decisions: §2 (T1–T4) and §5 (D1–D9), each with its rejected alternative.
6. Freeze Register: **E2 freezes nothing**; two append-only vocabularies extended and re-pinned.

Guide, per the named scope: chapter 3 gains the corrector story written as a MENTAL-MODEL change
(drift bounded, not merely smaller) plus four honest limits; chapter 11 gains all nine
`GateReason` spellings with two reading habits and why the Mahalanobis slot stays empty;
chapter 14's "No absolute position correction yet" partly falls and is rewritten as "Position
correction exists; yaw correction does not". No guide code block changed → no example re-quoting
needed, and `check-examples` passes.

Register: HA-61…HA-67 added (table rows + full entries, each with source/confidence/settle/blast
radius), HA-07's entry amended to record that its obligation had no code and no test until E2 and
that the old tests pinned the scale against the constant under test. Reconciliation direction 1
clean.

## 23:00:10 — final verification, all as observed

    suite            794 cases / 1,081,382 assertions, 0 failed, 3 skipped
    GUARD1           PASS      GUARD2  PASS
    ARM gate         PASS (112 headers)
    doc gates        self-test / coverage / fresh / examples / removability — all PASS
    mutations        20 RED, 0 GREEN, 0 build-fail, 0 SKIPPED
    reconciliation   direction 1 clean

Nothing committed, nothing pushed, still on `shulib-v2`. E2 done.
