# Chunk A3 — COMPLETED (2026-08-02)

> Completion record for [`A3-hostile-fakes.md`](A3-hostile-fakes.md) — the hostile fakes: the nine
> A2 degradation seams populated with how V5 hardware actually misbehaves. Everything below is
> **as actually observed** — commands run, outputs captured, mutations executed, defects watched
> surfacing in real time (the live sequence is in [`A3-PROGRESS.md`](A3-PROGRESS.md)). Changes are
> in the working tree, uncommitted, pending review.
>
> **The brief said "A3 that finds nothing has failed." A3 found three real defects in the
> Localizer, a detection hole, and two physics surprises — §3 below is the point of the chunk.**

---

## 1. What was built

| Piece | File | Role |
|---|---|---|
| `ImuHostileModel` | `include/shulib/sim/hostile/imu_hostility.hpp` *(new)* | Calibration window (garbage that MOVES + not-ready), per-boot rate bias → drift (consistent between heading and yawRate, so Phase E can observe it), Gaussian noise, mid-run dropout (frozen-finite) |
| `GpsHostileModel` | `include/shulib/sim/hostile/gps_hostility.hpp` *(new)* | Update decimation (the double-counting hazard), noise DECOUPLED from the claimed rms, off-strip / no-fix windows / dropout (finite stale pose per the IGps contract; origin-until-first-fix so trusting it gets caught), bad-fix confident lies |
| `EncoderHostileModel` | `include/shulib/sim/hostile/encoder_hostility.hpp` *(new)* | Tick-grid quantization (telescoping — cumulative delta error ≤ 1 step, pinned), channel freezes, the **PROS_ERR_F (+∞) sentinel breach** (a deliberate F4-contract violation, injectable on purpose), bump/skid |
| `PowerHostileModel` | `include/shulib/sim/hostile/power_hostility.hpp` *(new)* | Sag ∝ commanded load + discharge, pack ceiling on effective volts, brownout collapse (motors 0 V, run continues), per-wheel thermal ramp with the VEX 50/25/12.5 % droop steps |
| `SlipHostileModel` | `include/shulib/sim/hostile/slip_hostility.hpp` *(new)* | Acceleration-triggered traction loss (engages on hard ramps, releases as accel decays) + declared slip windows with per-wheel masks |
| `LatencyHostileModel` | `include/shulib/sim/hostile/latency_hostility.hpp` *(new)* | Ring-buffered per-channel delay (the stateful subclass `degradation.hpp` promised); startup-stale + bounded-overflow semantics; 1 ns cutoff grace (§3.5) |
| `ChainedDegradation` + `FullHostility` + `JitterSchedule` | `include/shulib/sim/hostile/composed.hpp` *(new)* | Composition = left→right function composition (order pinned by test); the canonical everything-hostile world (power→slip→imu→gps→encoders→latency, documented physical order); the seeded hostile dt schedule with a PRIVATE Rng |
| `HealthMonitor` | `include/shulib/diag/health_monitor.hpp` *(new)* | Pathology→`FaultCode` policy: edge-triggered per EPISODE (anti-spam), boot-window-is-not-a-loss rule, brownout hysteresis + run-latched marker (E1 semantics), raw observables so diag/ stays a leaf; C1 reuses it |
| `FaultCode::MotorOverTemp` | `include/shulib/diag/fault.hpp` *(modified)* | Appended (= 9) via the documented append-only path; wire pins extended |
| `Rng::nextGaussian()` | `include/shulib/sim/rng.hpp` *(modified)* | Pinned Box-Muller mapping (exactly 2 draws/call, log-safe), additive — no existing mapping changed |
| **Localizer fixes** | `include/shulib/localization/localizer.hpp` *(modified)* | The three defects hostility found, fixed at the source (§3.1–3.3): boot guard, settle window (`bootSettleTime`), mid-run-loss = `Degraded` |
| Latency knife-edge fix | `sim/hostile/latency_hostility.hpp` | §3.5 — model-side 1 ns inclusivity grace |
| Seam doc | `include/shulib/sim/degradation.hpp` *(modified)* | Header now points at `sim/hostile/` as the population; the base stays the identity seam |

**New tests:** `test/sim_hostile_imu_test.cpp` (8), `sim_hostile_gps_test.cpp` (9),
`sim_hostile_encoder_test.cpp` (7), `sim_hostile_power_test.cpp` (8), `sim_hostile_slip_test.cpp`
(7), `sim_hostile_latency_test.cpp` (7), `sim_hostile_survival_test.cpp` (10 — the fault-discipline
matrix), `sim_hostile_composed_test.cpp` (7), `health_monitor_test.cpp` (10); plus the
**unskipped-and-implemented `[acceptance][M2]`** in `accuracy_spec_test.cpp`, 5 boot-guard pins in
`localizer_test.cpp`, a `nextGaussian` case in `sim_scenario_test.cpp`, and `MotorOverTemp` pins in
`fault_test.cpp`. **80 newly-running cases / 133,643 new assertions**
(349/547,443 → **429/681,086**; skips 4 → 3 because the M2 stub went live).

---

## 2. The measured numbers (what the user asked hostility to produce)

- **60 s IMU drift, model-level:** worst end error **0.779°** over 8 boots (worst |bias| drawn
  0.773°/min of the ±1°/min provisional bound) — `sim_hostile_imu_test.cpp`, reported by MESSAGE.
- **The M2 `<1°` acceptance, full stack, full composed hostility, 10 boots (seeds 1–10, spanning
  drift draws +0.13…−0.93°/min):** worst **end-of-60s heading error 0.912°** — PASSES the frozen
  1.0° cap. Worst **instantaneous** error **1.065°** — transiently OVER the cap on the near-bound
  boot. **The margin at the pessimistic provisional drift bound is ~zero BY CONSTRUCTION**
  (1°/min × 60 s = 1°): the stack demonstrably adds no heading error of its own; the sensor IS the
  budget. R4's measured drift is the ceiling (build-order says exactly this); Phase E's heading
  correction (E3) is what buys margin. **F2 was not touched.**
- **Composed 14 s full-stack run (everything continuously hostile):** worst position error
  **0.261 in**, end 0.255 in, end heading error 0.007° — bounded, reported, and honest (quality ends
  `Degraded` after ~90 in with no fixes, exactly the M2 semantics).
- **Boot-poisoning attack, pre-fix vs post-fix:** fused drift on a stationary robot during a 2 s
  calibration window: **10.82 in → < 0.05 in** (and the composed run's worst error fell
  3.81 in → 0.261 in when the settle window closed the second leak — the fix is the root-cause fix,
  measured twice).
- **Sentinel breach damage:** ~1.05 in permanent (5 breach ticks + 2 poisoned deltas at 15 in/s),
  error flat afterwards — bounded and characterized, never a NaN.
- **Latency physics:** reported heading trails a 1.5 rad/s spin by exactly ω·L (−0.030 rad @ 20 ms);
  stop-drain phantom ≤ ω·L·|offset| ≈ 0.14 in (§3.6).

---

## 3. Flaws found (the point of the chunk) — each fixed in its owning chunk or explicitly deferred

### 3.1 FOUND + FIXED (Localizer/WS5): boot-window poisoning — observed 10.82 in on a stationary robot
A calibrating IMU emits garbage that MOVES; the odometry offset correction converts each garbage
heading swing into phantom translation (Δθ·offset — up to ~14 in per tick at the default offsets),
and the Localizer folded those deltas while honestly reporting quality 0/Uninitialized. **The flag
was honest; the pose was not** — and with no corrector at M2 the poison was permanent. Fixed IN THE
LOCALIZER (pilons_odometry.hpp explicitly assigns recovery policy to the fusion layer): ticks
before the IMU has EVER been ready fold no odom deltas and no corrector proposals; odometry still
consumes wheel deltas tick-by-tick so the transition sees one tick's travel, not the whole boot's.
Pinned in `localizer_test.cpp` (owner) and `sim_hostile_survival_test.cpp` (attack).

### 3.2 FOUND + FIXED (Localizer/WS5): `isReady()` outruns the heading data path — the composed-model catch
With the calibration window AND sensor latency live together, the ready flag flips true while the
heading STREAM still serves delayed garbage: the first post-transition fold differenced against a
delayed-garbage `prevHeading` and leaked **3.65 in** (probe: the fused error jumps at exactly
t = 2.01, one tick after ready). A one-tick transition guard was necessary but insufficient — the
boundary lives in the stream, displaced by an unknown data-path latency, not at the flag. Fixed
with a **settle window** (`bootSettleTime`, default 0.1 s): after a WITNESSED not-ready phase the
fold stays closed past first-ready; a ready-from-construction boot takes no hold (pinned — the
normal path is unchanged). Consequence promoted to a consumer contract, in-header: **motion before
`qualityClass` leaves `Uninitialized` is unaccounted** — C1's loop waits for a live estimate.
**This defect was reachable ONLY through composition** (window alone: no delay; latency alone: no
garbage) — the reducibility design paying for itself.

### 3.3 FOUND + FIXED (Localizer/WS5): mid-run IMU loss misreported as `Uninitialized`
After a healthy 3 s run, a dropout made `qualityClass()` report `Uninitialized` — a skills gate
would treat a robot that HAD an estimate and lost its heading authority like one still booting
(wrong recovery in both directions). Fixed: never-ready ⇒ `Uninitialized`; was-ready-lost ⇒
`Degraded`; scalar 0 in both. Deltas keep folding on the stale heading (encoders are still good —
the documented best-available choice), pinned in the owner's tests.

### 3.4 FOUND, DEFERRED WITH OWNERS NAMED: a frozen tracking encoder is invisible to the M2 estimator
Zero travel is a perfectly plausible reading, so `lastDeltaImplausible()` never fires and the
estimate walks away from truth at exactly the truth's speed (asserted: error == truth travel since
freeze). **Containment exists and is tested at the loop level** — the wheels-spin-but-no-motion
cross-check feeding `HealthMonitor::odomStalled` → `ODO_STUCK`, exercised in the survival suite in
exactly the shape C1 will own — but the estimator-side detector is **E-phase work** (fault.hpp
already assigns OdoStuck to "the C/E layers") and motion-level containment (watchdog timeout) is
**C1/C2**. Recorded, not papered over.

### 3.5 FOUND + FIXED (A3's own model): fp knife-edge in the latency ring cutoff
With latency an exact multiple of dt (the natural config), `now − L` lands on a stored stamp up to
~1e-17 of arithmetic dust, and the served delay flipped between k and k+1 ticks — deterministic but
config-fragile. Fixed in the model with a 1 ns inclusivity grace (documented in-header); the
exact-delay pins now hold to 1e-12.

### 3.6 FOUND (physics of the stack, pinned as a test): latency turns a clean stop into one final lie
When motion stops, the stale-heading window drains — heading keeps advancing while the wheels
truthfully report zero travel, so the offset correction "removes" a rotation the wheels never saw:
a phantom micro-translation bounded by ω·L·|offset| (~0.14 in at 1.5 rad/s / 20 ms / 4.5 in), after
which the error is exactly frozen. Pinned in `sim_hostile_latency_test.cpp`; this is precisely the
class of surprise E2's latency compensation must handle.

### 3.7 FOUND (modeling insight, recorded): sag only bites at the ceiling
Under true-voltage-control semantics (what V5 firmware approximates), a sagged pack affects motion
only when demand exceeds the sagged ceiling — a 6 V cruise on an 11.6 V pack is untouched. The
power liveness test therefore saturates deliberately; gains tuned at partial throttle are
insensitive to sag, which is worth knowing at R5.

### 3.8 Recorded limitation: truth-side hostility is deterministic
The seed perturbs only the sensor lies; sag/slip/thermal are pure functions of command history.
Under an open-loop script two seeds produce IDENTICAL physics — so the determinism/divergence proof
runs CLOSED-loop (sensor noise reaches the trajectory through the controller). Stochastic
truth-side hostility (random per-tick traction) is a possible future extension; nothing at M2
needs it.

---

## 4. Decision log (every choice with a viable alternative)

### D1 — Per-family model classes + `ChainedDegradation`, not one monolithic config-flag model
Independent injectability = use one model alone; composability = the chain; **reducibility = drop
one model** (ablation is literal, not config spelunking — and §3.2 was then attributed exactly this
way). Rejected: a monolithic class (all pathologies tangled, reduction by flag archaeology);
per-sensor callback slots (re-litigates A2's D5).

### D2 — Defaults are hostile for CONTINUOUS pathologies; EVENT pathologies default off
A default-constructed hostile model that does nothing is a dead seam waiting to ship (liveness
tests + mutation #2/#7 enforce the alternative). But "everything hostile at once" must still
complete a bounded-error run, which permanent dropouts would make vacuous — so drift/noise/
quantization/decimation/sag/accel-slip/latency/calibration are ON at provisional-realistic
magnitudes, and dropouts/sentinels/bad-fix/slip-windows are armed per scenario. Both composed tiers
are tested (realistic bounded-error + catastrophic everything-armed).

### D3 — `HealthMonitor` takes raw observables, lives in diag/, C1 reuses it
Raising is POLICY; the estimators deliberately expose state instead of raising (their headers say
so). A monitor holding `IImu&`/`Localizer&` would invert the diag-is-a-leaf dependency rule
(debug_record.hpp), so it receives plain bools/values the loop already reads. Edge-triggered per
episode with brownout hysteresis: 500 hostile ticks = one fault, not 500 (the §18 anti-spam rule;
mutation #6 proves the gate is load-bearing). GPS no-fix is deliberately NOT a fault (Driving
Skills has no strip — it is a quality state); GPS_GATE_REJECT covers the sensor actively lying.

### D4 — `FaultCode::MotorOverTemp = 9`, appended
Thermal droop is in-scope pathology and motor.hpp already promised a thermal fault; append-only is
the enum's documented evolution path and F9 does not freeze until H1. The wire pins gained a row;
none changed.

### D5 — `Rng::nextGaussian()` (pinned Box-Muller) over uniform noise or `<random>`
Sensor noise is σ-shaped and the tests reason in σ; `<random>`'s normal_distribution is
implementation-defined across stdlibs — the exact reason A2 rejected it for uniforms. The mapping
is pinned locally: exactly two u64 draws per call (no cached spare — draw-count must not depend on
call history), u1 ∈ (0,1] so log never sees 0. Pinned by draw-count alignment, determinism, and
moment tests. (No published reference vectors exist for this mapping — recorded honestly; the
independent-reference discipline A2 used for SplitMix64 doesn't transfer to a derived mapping.)

### D6 — GPS pathologies attack the REAL screening through a TEST-LOCAL naive corrector
E2's `GpsCorrector` is explicitly out of scope (brief: correctors → Phase E). The gullible
`NaiveGpsCorrector` (proposes every fix at face value) is a harness probe in the TrapSink spirit:
it makes the Localizer + ComplementaryFusion screening the thing under attack. Off-strip is proven
as an EQUALITY (fused == corrector-free twin, bit-for-bit), not a bound.

### D7 — The boot guard + settle window shape (§3.1/3.2), and what was rejected
Rejected: quarantining folds on `lastDeltaImplausible()` (a ≤90° boundary garbage draw evades the
flag ~half the time, and it contradicts the documented M2 flagged-but-still-integrated semantics
mid-run); delaying `imuReady` in the MODEL to hide the skew (status flags and data paths genuinely
skew on hardware — the model is right, the stack had to learn it); gating inside PilonsOdometry
(its header assigns recovery policy upward, and a second gate would mask the absence of this one).
The settle applies ONLY after a WITNESSED not-ready phase, so the normal ready-from-construction
path is pinned byte-identical to pre-A3 behavior.

### D8 — Mid-run loss keeps folding deltas (stale heading), reports Degraded + quality 0
The encoders are still good; a stale-heading estimate beats a frozen position for the containment
window until C2's watchdog acts. Rejected: freezing position on loss (discards good encoder data);
keeping Uninitialized (conflates boot with loss — §3.3). Recovery-from-loss re-baselines nothing at
M2 (the recovery tick usually self-flags implausible → Degraded); noted as an E-phase refinement.

### D9 — Latency ring semantics: newest-≥-L-old, startup-stale, bounded overflow, 1 ns grace
A real device's first report is stale, not absent (startup serves the oldest sample); overflow
saturates staleness at the ring span (256 slots = 2.56 s at 10 ms). The grace (§3.5) makes exact
L = k·dt configs serve k ticks robustly. `imuReady` is deliberately NOT delayed — readiness is a
state flag, and keeping it prompt is what exposed §3.2 (delaying it would have hidden a real
hardware hazard).

### D10 — Truth-side hostility is deterministic; randomness lives in the sensors
Sag/slip/thermal are pure functions of command history — reproducible physics with zero
draw-alignment coupling between models; seeded randomness enters through sensor lies and reaches
the physics only via a controller (§3.8). Rejected: rng-perturbed traction (invented constants
with no measurement path until R4, and it buys no M2-relevant coverage).

### D11 — The acceptance test's shape: 10-boot sweep, end-of-run criterion, full composed hostility
Seeds 1–10 with no gaps (the per-boot drift draws span +0.13…−0.93°/min including the near-bound
boot — verified against the pinned SplitMix64 first-draw mapping; representative, not curated).
The F2 criterion asserted is END-of-60s error per boot — the spec's own shape; the worst
instantaneous excursion (1.065°) is REPORTED but not gated on (a criterion the spec does not
state was not invented to fail). Straight legs, no commanded rotation ("straight-line test"),
boot held out per the settle contract.

### D12 — The sentinel breach deliberately violates the F4 finiteness contract
`degradation.hpp` notes the adapter screens PROS_ERR at the edge — but PilonsOdometry carries a
last-resort guard precisely for a breach, and A3's job is to prove that guard is real. So the
breach (+∞ = PROS_ERR_F) is injectable on purpose, documented as modeling a buggy adapter, and
mutation #4 proves the guard is what stands between it and a NaN pose.

---

## 5. Test inventory (what each would catch)

**sim_hostile_imu_test.cpp (8)** — all-zero config = EXACT identity (a hostile model must be
disable-able); calibration garbage MOVES and is far from truth, ready flips at the boundary; drift
== bias·(t−calEnd) exactly with the SAME bias in the reported rate (the Phase-E observability
property); **the 60 s drift number, measured and reported** (floor + ceiling — mutation #2's home);
noise σ pinned statistically; dropout freezes last-emitted forever (finite); same-seed identity /
different-seed different-bias; config rejection.

**sim_hostile_gps_test.cpp (9)** — quiet config passes truth exactly; decimation holds each fix
IDENTICALLY for the cadence (the double-counting hazard made visible); noise σ vs the DECOUPLED
claimed rms; off-strip never-fix with finite origin-stale pose (trusting it gets caught);
no-fix window holds the last sample finite and recovers; dropout permanent; **bad fix = truth +
offset at NORMAL rms with hasFix true** (the confident lie); determinism; rejection.

**sim_hostile_encoder_test.cpp (7)** — on-grid readings with ≤ half-step error; **telescoping**
(10k ticks, cumulative delta error ≤ ONE step — quantization can never random-walk); selective
freezes (incl. the latch-first-seen semantics of a never-read channel); the sentinel window emits
+∞ for exactly its span while the other wheel keeps working; bump/skid = permanent offset;
plant-level: odometry under real tick resolution within 10 tick-steps (~1.75e-3 in) over 8 s —
AND nonzero (liveness); rejection.

**sim_hostile_power_test.cpp (8)** — sag follows load and RECOVERS on stick release; discharge
linear; the ceiling caps both signs below the F4 clamp (brownout pushed away to isolate it);
at/below cutoff → exactly 0 V and a collapsed-but-≥0 reading; the thermal ramp crosses 55/60 °C
with the 50 %/25 % steps engaging and cooling releasing; wheels heat independently; **plant-level
liveness: a default-hostile full-throttle run travels measurably less** (mutation #2-class
protection for this family); rejection.

**sim_hostile_slip_test.cpp (7)** — hook-level accel trigger (baseline/launch/steady/gentle);
independent per-wheel histories; window masks; **the required signature with EXACT accounting:
truth 64.0 in vs 80 in of encoder-implied wheel travel** (overcount, predicted direction) while
tracking odometry rides through at < 1e-6 in (the D9 architecture proof); lagged-plant launch slip
that releases as accel decays; **closed loop converges THROUGH a slip window**; rejection.

**sim_hostile_latency_test.cpp (7)** — exact per-channel delay (1e-12); startup staleness;
zero-latency identity; ω·L heading trail measured through the plant; **bounded-and-then-frozen
odometry corruption incl. the stop-drain phantom** (§3.6); GPS delays the whole triple; rejection.

**sim_hostile_survival_test.cpp (10)** — the fault-discipline matrix (file header: fault raised +
finite pose every tick + bounded damage + run continues, per pathology): boot-garbage (the §3.1
attack, pre/post numbers in §2), mid-run IMU dropout (IMU_LOST once, `Degraded` not
`Uninitialized`), the +∞ sentinel (ODO_STUCK, ~1 in characterized loss, REQUIREd finiteness —
mutation #4's home), the frozen encoder (§3.4 — the estimator-blind assertion IS the record),
off-strip (bit-equal to dead-reckon), out-of-gate lie (GPS_GATE_REJECT, pose unmoved < 0.01 in),
in-gate lie (≤ 8.5 in bounded, heals to < 0.5 in), brownout (latched marker, motors dead, run
completes — mutation #1's home), jitter (LOOP_OVERRUN == the spike count EXACTLY, worst dt = the
5× stall, Pid still converges), thermal droop (MOTOR_OVER_TEMP via the C1-shaped wiring, speed
visibly halved, not dead).

**sim_hostile_composed_test.cpp (7)** — empty chain / identity chain = exact identity; **fold
order pinned** ((3+1)·2 ≠ 3·2+1); default FullHostility degrades every family observably (dead
composed seams cannot ship — mutation #7's home); **the DoD run** (14 s, bounded + MESSAGE-reported,
honest quality); **closed-loop byte-identical replay under full hostility** (truth memcmp AND fused
memcmp — mutation #3's home) with different-seed PHYSICS divergence; ablation attribution (slip and
power: draw-independent exact removals; imu: swept boots 1/3/10 spanning the bias distribution,
in-test comment documents the seed rationale); the catastrophic tier (every event armed at once:
1000 finite-REQUIREd ticks, first fault = Brownout kept, cascade ≥ 3, completed).

**health_monitor_test.cpp (10)** — healthy silence; boot-is-not-a-loss; once-per-episode + re-arm
(mutation #6's home); both ODO_STUCK flavours with their messages; gate-reject episodes; brownout
inclusive threshold + hysteresis band + the never-unlatching marker; over-temp inclusive at 55 °C;
cascade keeps the FIRST fault; reset() semantics; config rejection.

**accuracy_spec_test.cpp** — the `[acceptance][M2]` test, live (§2; D11). **localizer_test.cpp
(+5)** — the owner-side pins for §3.1–3.3 incl. the unchanged-normal-boot guarantee.
**sim_scenario_test.cpp (+1)** — nextGaussian draw-count/determinism/moments.
**fault_test.cpp** — MotorOverTemp wire pins (added, none changed).

---

## 6. Verification (actually run, outputs as observed)

```text
$ cmake --build build/test && ./build/test/shulib_tests
[doctest] test cases:    429 |    429 passed | 0 failed | 3 skipped
[doctest] assertions: 681086 | 681086 passed | 0 failed |
[doctest] Status: SUCCESS!
```
(3 skipped = the two M3 acceptance stubs + the R3 GPS field-cal oracle; the M2 stub is now LIVE.)

```text
$ <the ci.yml PROS-free guard grep>      GUARD 1 PASS: core is PROS-free (incl. sim/hostile + diag/health_monitor)
$ <the ci.yml layering guard grep>       GUARD 2 PASS: layering holds, core is sim-free
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # TU includes ALL 77 v2 headers
ARM CROSS-COMPILE: CLEAN (77 headers)
```

## 7. Mutation checks (each executed: break → build → run → observe red → restore)

| # | Mutation | Required? | Observed result |
|---|---|---|---|
| 1 | `HealthMonitor` drops the Brownout raise | yes (remove a fault-raise) | **RED** — 5 cases / 11 assertions: the brownout/cascade/reset units, the survival brownout attack, the composed catastrophic first-fault check; 424/429 |
| 2 | `ImuHostileModel` drift term made a no-op | yes (degradation no-op) | **RED** — 3 cases / 5 assertions: the exact-drift pin, the 60 s drift-number liveness floor, the composed ablation's worst-of-boots signature; 426/429. A silently dead pathology cannot ship |
| 3 | IMU noise drawn from a shared `static Rng` instead of the seeded stream | yes (break reproducibility) | **RED** — 2 cases / 302 assertions: the composed same-seed replay (BOTH the truth memcmp and the fused-trace memcmp — the closed-loop design is what lets the leak reach the physics) and the imu determinism sweep; 427/429 |
| 4 | `PilonsOdometry` finite guard defeated (`finite := true`) | yes (NaN propagates) | **RED** — 3 cases / 7 assertions: the +∞ sentinel reached the pose and the finiteness REQUIREs ABORTED the survival sentinel case and the composed catastrophic case (FATAL), plus all 5 pilons freeze pins; 426/429 |
| 5 | Localizer boot guard defeated (`foldDeltas := true`) | extra | **RED** — 5 cases / 214 assertions: all owner-side boot pins, the survival boot attack's 200-tick sweep, the composed 14 s bound; 424/429. The flagship fix cannot silently regress |
| 6 | `HealthMonitor` ImuLost episode gate removed (raises every tick) | extra | **RED** — 2 cases / 3 assertions (faultCount 1 vs 50; the cascade count); 427/429 |
| 7 | `ChainedDegradation` skips the imuHeading fold (dead composed seam) | extra | **RED** — 4 cases / 5 assertions: composed liveness, the 14 s hostility-is-live floor, replay divergence, the imu ablation; 425/429 |

After each restoration the full suite was rebuilt and re-run green; final state: no mutation
remnants in `include/` or `test/` (grep verified — the one "MUTATION" hit is A2's pre-existing doc
comment in `truth_integrator.hpp`), suite green **429 / 681,086**.

## 8. Provisional magnitudes — the A4 register's seed content (every one labelled in-header)

Each is a falsifiable claim R4 (or R5/R-phase telemetry) measures; **none is evidence of anything
until then**. Format: claim — where it lives.

1. IMU per-boot drift/rate-bias ≤ 1°/min (pessimistic bound; typical 0.1–0.5) — `imu_hostility.hpp`
2. IMU heading noise σ ≈ 0.05° — same
3. IMU yaw-rate noise σ ≈ 0.5°/s — same
4. IMU calibration takes ≈ 2 s and emits moving garbage while not-ready — same
5. IMU end-to-end heading latency ≈ 10 ms — `latency_hostility.hpp`
6. GPS on-strip position noise σ ≈ 0.7 in/axis — `gps_hostility.hpp`
7. GPS heading noise σ ≈ 1° — same
8. GPS fresh-fix cadence ≈ 50 ms — same
9. GPS claimed rms ≈ 1.0 in when healthy / ≈ 99 in reported at no-fix — same
10. GPS end-to-end latency ≈ 50 ms — `latency_hostility.hpp`
11. Encoder refresh ≈ 1 tick (10 ms) — same (documented; defaulted 0, exercised by test)
12. Drive encoder 900 ticks/rev at the output (GREEN cartridge; ours unverified) — `encoder_hostility.hpp`
13. Rotation sensor 36000 ticks/rev (centidegree) — same
14. Traction breaks above ≈ 80 in/s² wheel accel on our wheels/foam — `slip_hostility.hpp`
15. A slipping wheel still propels ≈ 70 % of its spin — same
16. Pack sag ≈ 0.02 V per commanded volt (≈ 1 V at 4×12 V) — `power_hostility.hpp`
17. Pack discharge ≈ 0.005 V/s under match load — same
18. Brain cuts motor power at ≈ 10.5 V (recover ≈ 10.8 V hysteresis) — same + `health_monitor.hpp`
19. Motor thermal: ≈ 0.0023 °C/(V²s) heating, ≈ 0.01 /s cooling, ambient 25 °C — `power_hostility.hpp`
20. VEX throttle steps 55/60/65 °C → 50/25/12.5 % (shape documented by VEX; onset unmeasured) — same
21. Current limiting (2.5 A stall cap) NOT modeled — needs the load model the honesty boundary forbids; R6 back-fits — same, in-header
22. Loop jitter ±20 %, ≈ 2 % spike probability, ×5 stalls — `composed.hpp` (JitterSchedule)
23. GPS off-strip/no-fix stale pose behaviour (origin here; real device unknown) — `gps_hostility.hpp`
24. `Localizer::bootSettleTime` = 0.1 s adequacy (must cover the real worst data-path latency) — `localizer.hpp`
25. `HealthMonitor` MOTOR_OVER_TEMP threshold 55 °C (= claim 20's first step) — `health_monitor.hpp`

## 9. Deliberately left for later chunks

- **Estimator-side frozen-encoder/stall detection** → E-phase (with C1/C2 watchdog containment);
  the loop-level cross-check SHAPE is tested now (§3.4).
- **Real `GpsCorrector`** (adaptive R, lever arm, latency compensation, yaw-rate rejection) → E2;
  the naive test corrector is a probe, not a preview implementation.
- **Recovery-from-mid-run-IMU-loss re-baselining** → E-phase refinement (D8).
- **Stochastic truth-side hostility** (random traction) → only if a real need appears (D10/§3.8).
- **Automatic motor current/temperature synthesis into the fakes** → unchanged from A2 (no seam;
  tests push `PowerHostileModel::temperatureC()` manually, the same wiring C1 will own).
- **The ARM compile gate in CI** → A4 as planned (verified manually again this chunk, 77 headers).
- **The A4 register itself** → A4; §8 is its seed content.

## 10. Freeze Register note (documentation contract #6)

**No freeze.** Three de-facto notes:
- **The `DegradationModel` hook set** now has eight consumers (the identity base, six hostile
  models, the chain) and is de-facto stable — but it stays sim-internal test infrastructure, NOT a
  cross-team contract, so it is deliberately not registered as a formal freeze; the suite is the
  guard.
- **`FaultCode`** gained `MotorOverTemp = 9` strictly via the documented append-only path (pins
  added, none changed). F9 still freezes at H1.
- **`LocalizerConfig`** gained `bootSettleTime` additively; the `IPoseSource`/`ICorrector`/
  `IFusionPolicy` seams are untouched.

## 11. DoD checklist (brief §Definition of Done)

- [x] **All 9 seams populated with justified hostile behaviours; each independently injectable** —
  effectiveVoltage+batteryVoltage (power), wheelMotionVelocity (slip), drive/tracking encoders
  (encoder), imuHeading/imuYawRate/imuReady (imu), gps (gps), plus latency across sensor channels;
  single-model injection is the per-family test idiom, zeroed configs are exact identities.
- [x] **A composed "everything hostile" model exists and full runs complete under it** — 14 s
  realistic tier (bounded + reported: 0.261 in worst) AND the catastrophic tier (every event armed,
  1000 finite ticks, faults in cause order).
- [x] **Every pathology raises a fault code with a safe fallback — no crash, no NaN in the pose** —
  the 10-case survival matrix + HealthMonitor; finiteness REQUIREd on every tick of every attack;
  mutation #4 proves the guard is load-bearing. Two honest qualifications, recorded: GPS no-fix is
  a quality state BY DESIGN (not a fault — D3), and the frozen-encoder detector is the loop
  cross-check pending E-phase (§3.4).
- [x] **Determinism holds under hostility** — closed-loop same-seed byte-identical (truth memcmp +
  fused memcmp), different-seed physics divergence; mutation #3 red.
- [x] **Every flaw fixed in its owning chunk or explicitly deferred with the chunk named** — §3:
  three Localizer fixes (WS5, owner-side pins), one model fix, two pinned physics findings, two
  named deferrals (E-phase, E2).
- [x] **The M2 `<1°` acceptance stub unskipped; passes with its honest number reported** — worst
  end-of-run 0.912° / 10 boots under full hostility (roadmap carries it as `[~]`: the stack's
  contribution is proven, the field claim waits on R4's drift + Phase E's margin; worst
  instantaneous 1.065° reported). **F2 untouched.**
- [x] **Invented magnitudes labelled provisional in-header and queued for A4** — §8, 25 entries.
- [x] **Full suite green under strict `-Werror`; both CI guards pass; all headers cross-compile for
  ARM** — 429/681,086; both guards; 77/77 headers (§6).
