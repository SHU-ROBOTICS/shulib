# Chunk A2 — COMPLETED (2026-08-01)

> Completion record for [`A2-host-plant.md`](A2-host-plant.md) — the host plant + closed-loop sim
> harness, the roadmap's missing prerequisite. Everything below is **as actually observed** —
> commands run, outputs captured, mutations executed (the live sequence is in
> [`A2-PROGRESS.md`](A2-PROGRESS.md)). Changes are in the working tree, uncommitted, pending review.

---

## 1. What was built

| Piece | File | Role |
|---|---|---|
| `Rng` | `include/shulib/sim/rng.hpp` *(new)* | SplitMix64 — the harness's ONE random source; seeded, portable, pinned to the published reference vectors |
| `DegradationModel` + `GpsTruth` | `include/shulib/sim/degradation.hpp` *(new)* | The nine A3 injection seams, identity-default, each documenting the hostile behaviour it exists for; **empty by scope** |
| `MotorModel` | `include/shulib/sim/motor_model.hpp` *(new)* | Voltage → wheel surface velocity by EXACT inversion of `control::Feedforward` (kS dead band; τ = kA/kV first-order lag); stateless pure `advance()` |
| `TruthState` + `advanceTruth` | `include/shulib/sim/truth_integrator.hpp` *(new)* | Ground-truth pose integration: RK4 on (x, y, θ-UNWRAPPED) — **independent of `arcStep` by construction** (constraint 2) |
| `DrivePlant` | `include/shulib/sim/drive_plant.hpp` *(new)* | The 9-step tick pipeline: motors → seams → model → `IKinematics::forward` → RK4 truth → clock → sensor synthesis into the F4 fakes → lazy `DebugRecord` |
| `SimHarness` + `TruthSample` + `randomBodyTwist` | `include/shulib/sim/scenario.hpp` *(new)* | Full fake robot + `RobotContext` wiring, fixed/variable-dt run loops, matched tracking-wheel factories, memcmp-able truth samples |
| CI guards | `.github/workflows/ci.yml` *(modified)* | `include/shulib/sim` added to the PROS-free scope; **new layering guard**: no core header may include `shulib/sim/` |
| Roadmap fix | `docs/roadmap.md` *(modified)* | The A2 task added under M2 (the incompleteness bug closed, with evidence); "you are here" → A2 done / A3 next |
| Position | `docs/internal/build-order.md` *(modified)* | Current position updated; "There is no host sim" struck through with its honest residual |

**New tests:** `test/sim_motor_model_test.cpp` (8 cases), `test/sim_truth_test.cpp` (8),
`test/sim_plant_test.cpp` (14), `test/sim_closed_loop_test.cpp` (4),
`test/sim_odometry_truth_test.cpp` (5), `test/sim_scenario_test.cpp` (9).
**48 new cases / 25,320 new assertions** (301/522,123 → **349/547,443**).

**The existing fakes were driven, not replaced — with ZERO additive setters.** Every setter the
plant needs already existed (`FakeMotor.setPosition/setVelocity`, `FakeImu.setHeading/setYawRate/
setReady`, `FakeGps.setPose/setRmsError/setHasFix`, `FakeRotation.setPosition`,
`FakeBattery.setVoltage`). F4 is untouched in every sense; the code under test reads only
`RobotContext` and the F4 interfaces.

---

## 2. Decision log (every choice with a viable alternative)

### D1 — Truth integrator: fixed-substep RK4 on UNWRAPPED θ, and the independence tripwire
The one constraint that dominates the chunk (brief §constraint 2). **Rejected:** *reusing
`arcStep`* (any arcStep error would appear identically on both sides of every localization
comparison and cancel invisibly — Phase E would measure nothing); *re-deriving the same closed
form* (independent-looking, but algebraically the identical half-angle expression — a shared blind
spot in different clothes). RK4 shares zero algebraic structure with the chord form.
**The subtle part, discovered while designing the mutation:** arcStep is mathematically EXACT for a
constant twist, so *no agreement test can detect a truth integrator that secretly reuses it* — they
agree to machine precision. The detectable difference is structural: arcStep receives wrapped
`Angle`s (a >π-per-tick rotation aliases before it runs); RK4 on a raw accumulating θ handles it
exactly. Hence the "beyond arcStep's wrap horizon" test (270° in one tick, endpoint from circle
geometry) — proven to red under mutation #1 **while the 405-point agreement sweep stayed green**,
which is the demonstration that the tripwire is load-bearing, not decorative.

### D2 — Wheels→twist SHARED with the frozen F5 kinematics; pose integration independent
The plant calls `IKinematics::forward()` (step 5). **Rejected:** a privately re-derived second
kinematics. Reasons: F5 is frozen and independently oracle-tested (closed-form geometry pins in
`x_drive/tank/matrix` tests); the plant must agree with what motion code computes *by contract*; a
second derivation could silently diverge from the one the motion layer actually uses. The
cancellation argument does NOT apply here the way it does to arcStep: no estimator consumes
`forward()` at M2, and sensor synthesis (step 8) derives from the TRUE POSE, not from kinematics —
so no Phase E estimator-vs-truth comparison can cancel through it. Shared: the frozen contract.
Independent: the pose integrator. Both on purpose, both documented in `drive_plant.hpp`.

### D3 — Motor model: exact steady-state FF inversion + exponential lag, stateless
`v_ss`: dead band |V| ≤ kS → 0, else sign(V)·(|V|−kS)/kV — the exact inverse of
`V = kS·sign(v) + kV·v`, so the defining testable property is *commanding
`Feedforward::calculate(v)` holds the wheel at exactly v* (the relation every C-phase FF+PID loop
depends on; pinned by sweep with kS live). The kA term supplies τ = kA/kV via the exact exponential
update (unconditionally stable at any dt; semigroup property pinned: one step of dt == ten of
dt/10). **Rejected:** *any invented dynamics* (mass/torque/friction — the brief's "explicitly
rejected": unmeasurable without a robot, and a confident wrong plant gets trusted); *Euler-stepped
lag* (step-size-dependent transients would make closed-loop results depend on dt);
*stateful model objects* (a pure `advance(v, V, dt)` is exhaustively testable point-by-point; the
plant owns one double per wheel). Two documented transient simplifications: friction follows the
steady-state sign during the approach (no stiction modeling — that honestly needs R5 data), and
kA = 0 degrades to the memoryless inversion (which is what most logic tests use, making open-loop
distances exactly hand-derivable).

### D4 — Discrete-time semantics: zero-order hold, end-of-tick velocity
Each tick, wheel velocities advance to their end-of-tick values and THAT twist is held constant for
the pose integral. **Rejected:** an exact-average-velocity integral (slightly smoother transients,
but it breaks the "constant twist per tick" model that the truth integrator AND the odometry
derivation both assume — one semantics everywhere beats transient polish). Exact for kA = 0; O(dt)
transient bias under lag, bounded and pinned by the discrete-geometric-series test (which also
checks the gap to the continuous integral stays under the documented bound). Surfaced honestly: the
first version of the lag test compared against the continuous integral and failed by the
discretization bias — the test was fixed to the discrete oracle, and the semantics were promoted
into the header as a documented contract rather than left implicit.

### D5 — Degradation seams: ONE virtual policy, identity defaults, consultation proven by test
Nine hooks (`effectiveVoltage`, `wheelMotionVelocity`, `driveEncoderPosition`,
`trackingEncoderPosition`, `imuHeading`, `imuYawRate`, `imuReady`, `gps`, `batteryVoltage`), each
identity by default and each documenting the A3 behaviour it exists for. Latency is deliberately a
*stateful subclass* concern (hooks receive `(truth, now)` so a model can buffer and delay); loop
jitter is deliberately the *runner's* dt-schedule seam (`runTicksVariable`) — timing hostility is
injected around the plant, never inside it. **Rejected:** per-sensor callback slots (nine ad-hoc
seams with no shared contract); compile-time policy templates (A3 models need runtime state and
test-local subclassing). The trap this design had to close: identity hooks make a plant that
*ignores the policy entirely* pass every test — so the seam-liveness test drives a test-local
hostile stub through all nine hooks with distinctive transforms, and mutation #7 (skip one hook)
proves it reds. A3 is population, not surgery, **by test**.

### D6 — Own SplitMix64 over `<random>`
The standard's engines are portable but its *distributions* are implementation-defined — the same
seed yields different streams on libstdc++ vs libc++. Byte-reproducibility must not depend on a
standard-library version, so the generator AND the value mappings are pinned locally (~20 lines),
and the test vectors were computed by an independent Python reimplementation (matching the
published SplitMix64 sequence), not by the C++ under test. Seed consumers are enumerated in the
header (scenario command draws + future A3 degradation draws) so randomness can never sneak in
elsewhere. **Honest caveat, documented in `drive_plant.hpp`:** run-to-run and same-toolchain
byte-identity is guaranteed and pinned by memcmp; identity across *different libm implementations*
is not (cos/sin/exp may differ in the last ulp between C libraries).

### D7 — Truth exposure: harness-only accessors + a CI layering guard
Truth getters live on `DrivePlant`/`SimHarness` only; no F4 interface exposes them and
`RobotContext` cannot reach them. That structural fact is *held* by a new CI step: any core header
including `shulib/sim/` fails the build. **Rejected:** friend-gated access (complexity without
adding a guarantee); handing tests a separate truth object (the plant must own truth to synthesize
sensors from it — separation would be cosmetic). `TruthState` is deliberately raw doubles rather
than `Pose2d` — a distinct currency that cannot be passed where a HAL value is expected.

### D8 — The plant advances the injected `FakeClock` (single time authority)
`step(dt)` advances physics AND the clock the code under test reads, so they can never skew; the
documented loop shape (controller first, sees time t; step advances to t+dt) gives a `Pid` exactly
one dt between calls — the same shape C1's motion loop will have. **Rejected:** runner-advanced or
test-advanced clocks (two authorities, and every test becomes responsible for keeping them
lock-stepped — the exact bug class injection exists to kill).

### D9 — Encoders read SPIN; body motion and tracking wheels read post-slip MOTION
The slip seam sits *between* them: drive encoders integrate the wheel's spin (pre-slip), body
motion uses `wheelMotionVelocity()` (post-slip), and the unpowered tracking wheels measure actual
motion at their mount point (rigid-body v_point = v + ω×r, derived in the header independently of
`PilonsOdometry`'s inverse). Under A3 slip this reproduces exactly how hardware lies: drive
encoders overcount while the robot undershoots and the tracking wheels tell the truth. The ω×r
signs are pinned by a dedicated test *independent of odometry* — so a synthesis sign error and an
odometry sign error cannot cancel end-to-end (proven: mutation #5 reds both the direct pin AND all
four odometry-vs-truth cases).

### D10 — Tolerances MEASURED, not assumed
A probe measured the truth-integrator error across the exact agreement sweep before the tolerance
was pinned: worst |truth − arcStep| = 3.7e-7 @ 32 substeps, 2.3e-8 @ 64, 9.2e-11 @ 256. The sweep
runs at 256 substeps against 1e-9″ absolute (10× margin), the 4th-order convergence ratio is
asserted directly (e8 < e4/10 against an analytic endpoint), and the plant's default 32 substeps is
orders inside budget at real tick sizes (Δθ ≤ 0.06 rad @ 100 Hz). The probe also surfaced a genuine
numerical finding — see §6.

### D11 — Harness defaults are adversarial; test oracles are hand-derived
`SimHarnessConfig` defaults to NON-zero tracking offsets (−3″, −4.5″) so every default run
exercises the offset-correction math instead of hiding it behind zeros. Every open-loop expectation
in the suite is derived analytically in the test (X-drive √2 geometry, tank differential, ω×r,
geometric series) — never from the plant's own output; `MotorModel` deliberately does NOT expose
its steady-state helper, so tests cannot lazily consult the model they are grading.

---

## 3. Test inventory (what each would catch)

`test/` totals moved **301 cases / 522,123 assertions → 349 / 547,443** (4 pre-existing M0
acceptance stubs still deliberately skipped — see §7 for why A2 does NOT unskip the M2 one).

**sim_motor_model_test.cpp (8)** — the FF-inversion sweep with kS live and non-round gains (a
dropped kS or approximated division reds — mutation #4's home); dead-band boundary incl. exactly-at
kS both signs; τ = kA/kV against the analytic exponential (not the model's own output); the
**semigroup exactness** case (one step == ten chained steps — an Euler-style lag would make every
closed-loop result dt-dependent); reversal decays monotonically THROUGH zero to the analytic
negative steady state; **dt == 0 must not teleport** (the early-out must precede the kA = 0 branch
— a real ordering trap); extreme volts/huge dt finite; precondition rejections (kV ≤ 0, negatives,
NaN gains, negative/NaN dt).

**sim_truth_test.cpp (8)** — straight lines pinned to `robotToField` across headings (handedness);
pure rotation leaves position untouched with θ UNWRAPPED to 9 rad; quarter-circle from circle
geometry (the arc_step keystone, now demanded of the independent integrator); **RK4 4th-order
convergence proven** (e8 < e4/10 vs an analytic endpoint, plus the 1e-9 regime); the **405-point
arcStep-vs-truth agreement sweep** (ω ∈ {0, 1e-12…1e-6 across arcStep's guard, up to |Δθ| = 2.7},
5 headings × 3 vx × 3 vy, 1e-9″ absolute with measured 10× margin) — the DoD's two-sided test of
arcStep; the **independence tripwire** (270°-in-one-tick: truth == circle geometry AND arcStep's
alias provably >1″ elsewhere); full-circle tick returns home with θ = 2π not 0; zero-twist/zero-dt
no-ops + precondition rejections.

**sim_plant_test.cpp (14)** — X-drive open-loop distance vs the hand-derived √2·(V−kS)/kV·T
(mutation #4's required red); lag distance vs the **discrete geometric series** + the bounded gap
to the continuous integral (pins the ZOH contract); tank straight/spin analytic (the plant is not
X-specific — DoD); the 80-trial seeded **round-trip sweep** both drivetrains (toWheels → FF → plant
→ observed twist == command, wheel spins == toWheels; mutation #2's required red), with ranges
derived to stay inside the ±12 V budget; tank strafe honestly ignored; **tracking-wheel ω×r sign
pins independent of PilonsOdometry** (kills end-to-end sign cancellation); IMU/GPS/drive-encoder
synthesis mirrors truth; zero voltage = zero motion (kS > 0 AND kS = 0); saturating 100 V clamped
by the F4 contract to the 12 V analytic distance; **dt == 0 is a true no-op** (state, clock,
sensors, and NO phantom record); one giant 10 s tick lands analytically; symmetric reversal returns
exactly to origin; precondition rejections; initial pose seeds truth AND sensors before tick one.

**sim_closed_loop_test.cpp (4)** — the first closed loops in the project. `Pid` fed by
**PilonsOdometry over synthesized sensors** (truth is only the judge) converges to 24″ and HOLDS
< 0.05″ through the last full second, without lateral smear; the heading loop through the IMU
settles < 0.5° spinning in place; a sign-flipped gain **diverges without bound** (> 100″ and still
growing at 6 s); an overdriven gain (kP = 20,000, past the DERIVED instability threshold — the
discrete loop's eigenvalue product is exactly q = e^{−dt/τ}, so instability needs
dt(1−q)kP > 2(1+q) ⇒ kP ≈ 12,000) **limit-cycles forever**: amplitude 3 orders above the good-gain
hold, repeated crossings, command still slamming the ±60 rail at 10 s. A plant that always
converges models nothing; this one demonstrably doesn't.

**sim_odometry_truth_test.cpp (5)** — the first end-to-end localization proof. `PilonsOdometry`
within 1e-6″ of truth at EVERY tick of a rich 8-segment script (forward/arc/strafe/spin/holonomic/
reversal), heading exactly IMU-owned; 3-seed random 5 s sweeps, same bound; the full `Localizer`
(+`ComplementaryFusion`, empty correctors) rides the same sensors with HONEST quality
(DeadReckon early → Degraded past the 12″ drift horizon with no fixes — the M2 semantics);
**the anti-agreeable proof**: wheels configured 2% fat make odometry read 51.0″ against a true
50.0″ — the analytically predicted lie, and the harness SEES it (if it couldn't, Phase E would
measure nothing); a full 60 s dead-reckon run holds < 1e-5″ worst error (the logic proof — NOT an
F2 accuracy claim; see §7).

**sim_scenario_test.cpp (9)** — SplitMix64 pinned to the independently-computed published reference
vectors (seeds 0 and 42); 11k-draw interval sweeps; **same-seed byte-identical replay** (memcmp on
packed `TruthSample`s — representations, not tolerances) and different-seed divergence (mutation
#3's required red); TermSink watchability (exactly one framed line per tick, `[t=`-stamped from the
record, `[LOC]`-tagged); the A1 cost contract both ways (records populated correctly for a
consuming sink; the **TrapSink probe** — a deliberately contract-violating wantsRecord=false +
counting emit — proves the plant never bypasses `emitRecord`, mutation #6's red); **all nine
degradation seams proven live** via a test-local hostile stub (half-volts, 20% slip, biased
encoders/IMU, dropped readiness, no-fix GPS, sagged battery — each observable asserted, truth
unaffected where it must be); the variable-dt jitter seam integrates its schedule exactly; clock
lockstep incl. `&context().clock() == &clock()`; negative tick counts rejected.

---

## 4. Verification (actually run, outputs as observed)

```text
$ cmake -S test -B build/test && cmake --build build/test && ./build/test/shulib_tests
[doctest] test cases:    349 |    349 passed | 0 failed | 4 skipped
[doctest] assertions: 547443 | 547443 passed | 0 failed |
[doctest] Status: SUCCESS!
```

```text
$ <the ci.yml PROS-free guard grep, scope now incl. include/shulib/sim>
GUARD 1 PASS: core is PROS-free (incl. sim/)
$ <the NEW ci.yml layering guard grep>
GUARD 2 PASS: layering holds, core is sim-free
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # TU includes ALL 69 v2 headers
ARM CROSS-COMPILE: CLEAN (all v2 headers incl. sim/)
```

---

## 5. Mutation checks (each executed: break → build → run → observe red → restore)

| # | Mutation | Required? | Observed result |
|---|---|---|---|
| 1 | Truth integrator reuses `arcStep` | yes | **RED** — 3 cases / 5 assertions: the wrap-horizon tripwire, full-circle-unwrapped, and RK4-convergence (e4 == 0, arcStep exact). **The 405-point agreement sweep stayed GREEN under this mutation** — the observed proof that an agreement test alone can never catch the constraint-2 trap, and the tripwire is load-bearing; 346/349 |
| 2 | Single-wheel sign flip in the plant's wheel→body chain | yes | **RED** — 15 cases / 231 assertions: the round-trip sweep (the required red) + every analytic open-loop case (X-drive AND tank) + closed-loop convergence + seam liveness. Odometry-vs-truth stayed green as designed (it grades estimator-vs-truth; truth-vs-analytic tests own this class); 334/349 |
| 3 | `Rng` ignores its seed, takes wall-clock entropy | yes | **RED** — 2 cases / 6 assertions: same-seed byte-identical replay (memcmp mismatch — the required red) + the SplitMix64 reference pins; 347/349 |
| 4 | Motor model drops the kS term (`v = V/kV`) | yes | **RED** — 14 cases / 181 assertions: the open-loop travel red (required) + FF-inversion sweep, dead band, tank analytics, lag distance, saturation, round-trip, mis-calibration baseline; 335/349 |
| 5 | ω×r sign flipped in tracking-wheel SYNTHESIS | extra | **RED** — 5 cases / 7 assertions: the direct rigid-body sign pin AND all four odometry-vs-truth cases — the feared synthesis↔odometry sign cancellation is impossible; 344/349 |
| 6 | Plant bypasses `emitRecord` (always-build-and-emit) | extra | **RED** — 1 case / 1 assertion: the TrapSink probe (emit() reached 10× with wantsRecord() false). The A1 cost contract is enforced on the plant, not just documented; 348/349 |
| 7 | Plant skips the `effectiveVoltage` degradation hook | extra | **RED** — 1 case / 4 assertions: the seam-liveness case (distance matched the UNdegraded analytic; encoder biases compounded). A dead seam cannot ship — A3 stays population-not-surgery by test; 348/349 |
| 8 | RK4 downgraded to forward Euler (k1 only) | extra | **RED** — 8 cases / 389 assertions: the 4th-order convergence check (ratio ~2, not >10), the agreement sweep, quarter-circle, wrap-horizon, and all four odometry-vs-truth cases. The documented accuracy budget is enforced, not asserted; 341/349 |

After each restoration the full suite was rebuilt and re-run green; final state: no mutation
remnants in `include/` or `test/` (grep verified), suite green **349 / 547,443**.

---

## 6. Discovered about existing code (and about the math)

- **All ten F4 fakes already carry every setter the plant needs** — the "drive the fakes" constraint
  closed with ZERO additive changes to `hal/`. The F4 freeze is untouched in every sense.
- **`arcStep` is exact for a constant twist — which makes constraint 2 subtler than the brief
  states:** independence cannot be *verified* by agreement (an arcStep-reusing truth agrees
  perfectly); it must be verified where the contracts structurally differ (the >π wrap horizon).
  Observed directly under mutation #1.
- **The naive direct-integral SE(2) form is ill-conditioned near ω → 0** — its `(1−cos ω)/ω` term
  loses ~7 digits to cancellation (measured ~1.5e-7 floor at ω ≈ 1e-8). This independently and
  empirically confirms `arc_step.hpp`'s design note that the half-angle `sin(x)/x` form is the
  well-conditioned one. The test oracle is therefore only consulted at large ω.
- **The F4 motor clamp is load-bearing in sweeps:** random twists at 40 in/s demanded ~65 in/s
  wheels ⇒ >12 V ⇒ the FakeMotor clamp engaged and round-trips honestly failed until the sweep was
  budgeted (documented in-test). The plant enforcing voltage reality is a feature the tests now
  respect rather than a surprise.
- **A P velocity-command loop on a first-order lag is unconditionally stable in continuous time**,
  and its discrete eigenvalue product is exactly q = e^{−dt/τ} — instability needs
  dt(1−q)kP > 2(1+q) (kP ≈ 12,000 at τ = 0.3, dt = 10 ms). A first bad-gain attempt at kP = 500
  settled *correctly*; the final test sits at kP = 20,000 with the threshold derived in-comment.
  The plant reproduces textbook discrete-time control behavior quantitatively.
- **Overdriven-gain instability under saturation appears as high-frequency chatter** (~0.05″
  amplitude, rail-to-rail command), not large swings — the lag low-passes the bang-bang. Matches
  real overdriven-P behavior; the test pins the chatter signature.
- **Loop-phase subtlety in harness use:** a consumer that updates odometry *before* each plant step
  trails truth by one tick at run end; the final fold-in is documented in-test. C1's motion loop
  should adopt the same controller-first shape deliberately.
- **`Localizer`'s M2 quality semantics verified end-to-end:** a long dead-reckon run correctly
  degrades (DeadReckon → Degraded past the 12″ drift horizon) — the honest no-corrector story.

## 7. Deliberately left for later chunks

- **The nine degradation seams are empty** → A3 populates them (noise, dropout, slip, quantization,
  latency, sag/brownout, jitter schedules). The seams are proven live and mutation-guarded, so A3
  is population, not surgery.
- **The M0/M2 acceptance stubs in `accuracy_spec_test.cpp` stay skipped, deliberately.** A2 gives
  the [acceptance][M2] 60 s heading stub a system to run against — but with perfect sensors the
  <1° assertion is vacuous (heading error is identically ~0), and unskipping it now would certify
  nothing while claiming F2 evidence. It goes live at A3 when modeled IMU drift makes it a real
  claim. The 60 s *logic* proof (no integration drift) exists in `sim_odometry_truth_test.cpp`.
- **Motor current/temperature synthesis** left at 0 — modeling them needs exactly the invented
  constants the brief forbids; A3 can inject hostile values through the fakes directly, and R6
  back-fits measured behavior.
- **`SimHarness::commandBodyTwist` applies no desaturation and no field rotation** — those are the
  motion layer's jobs (§13 #5 / F1), arriving at C1; the helper mirrors only the wheel-voltage leg.
- **Gains in `DrivePlantConfig` are placeholders by declaration** (right order of magnitude, not
  measurements) — R5 measures, R6 feeds back and re-runs the suite against the calibrated plant.
- **Cross-libm bit-identity** is documented as out of scope for the determinism guarantee (libm
  ulp differences); run-to-run identity is pinned. If Phase E ever needs cross-machine identity,
  a table-free sin/cos would have to be vendored — deferred until a real need exists.
- **The ARM compile gate in CI** → A4 as planned (verified manually again this chunk at 69 headers).

## 8. Freeze Register note (documentation contract #6)

**No freeze.** Two things are becoming de-facto contracts and are flagged for awareness:
- **The `DegradationModel` hook set** — A3 will subclass it; changing hook signatures after A3
  populates them would be surgery. It is header-documented as the A3 seam but NOT frozen; A3 may
  still reshape it freely (it is the first consumer).
- **The scenario loop shape** (controller-first, plant-advances-clock, sensors reflect t+dt after
  `step`) — every closed-loop test in Phases C–F will assume it; C1's `MotionScheduler` should
  adopt the same shape. Documented in `scenario.hpp`; a change after C1 exists would ripple.
Neither is registered as a formal freeze — both have exactly one consumer today, and the
freeze-after-two-consumers rule holds.

## 9. DoD checklist (brief §Definition of Done)

- [x] Open-loop voltage command moves the robot a predictable, analytically-derived distance —
  `sim_plant_test.cpp` X-drive/tank/lag cases (oracles hand-derived in-test; kS live)
- [x] Closed-loop `Pid` holds a position against the plant; a bad gain demonstrably diverges —
  `sim_closed_loop_test.cpp` (converge+hold via the sensor path; sign-flip unbounded; overdriven
  chatter past the derived threshold)
- [x] Plant forward kinematics round-trip against `MatrixKinematics` across a swept input space —
  80 seeded trials, X-drive AND tank, twist AND per-wheel agreement
- [x] Truth integration independent of `arcStep`, and `arcStep` verified against it — RK4 +
  405-point sweep @ 1e-9″ (measured 10× margin) + the wrap-horizon tripwire; mutation #1 red
- [x] `PilonsOdometry` tracks truth within a documented bound over a multi-second run — ≤1e-6″
  every tick over 8 s (and 60 s @ ≤1e-5″); bound's honesty documented (perfect sensors = logic
  proof, real bounds at A3)
- [x] Runs bit-reproducible from a seed — memcmp-identical replay, divergent under a different
  seed; mutation #3 red. (Caveat recorded: run-to-run/same-toolchain; cross-libm excluded)
- [x] Works for both X-drive and tank — open-loop, round-trip, and honest-strafe cases on both
- [x] Degradation seams exist, are documented, and are no-ops — nine identity hooks, each with its
  A3 intent; liveness proven by hostile-stub test; mutation #7 red
- [x] A run is watchable via `TermSink`, and costs nothing with `NullSink` — one framed line per
  tick observed; TrapSink proves emit() unreachable when unwanted; mutation #6 red
- [x] Full host suite green under strict `-Werror`; all v2 headers cross-compile for ARM; CI guard
  passes — **349 cases / 547,443 assertions**; 69/69 headers ARM-clean; both guards pass (incl.
  the new sim-layering guard)
