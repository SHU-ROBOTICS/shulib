# shulib v2 — the complete project briefing

> **Paste this entire file into a new session.** It is the full context transfer: who I am, what the
> library is, everything that exists, how we work, what has bitten us, and what comes next. It is
> maintained as a document, not regenerated ad-hoc — if something here is wrong or stale, fix it here
> and say so.
>
> **Related and NOT duplicated here:** [`RESUMING.md`](RESUMING.md) owns the protocol in its canonical
> form · [`build-order.md`](build-order.md) owns the chunk order and "Current position" ·
> [`../roadmap.md`](../roadmap.md) owns milestones and the Freeze Register ·
> `chunks/*-COMPLETED.md` own the per-chunk records. **Link, don't restate** — any number copied
> between documents is a number that will go stale.

---

## 1. Who I am and what this is

I'm the VEX U programming chair at Seton Hall. **shulib v2** is a holonomic-native autonomous library
for VEX U robots, at `/home/gonzei/projects/shulib`, branch `shulib-v2`.

**The thesis:** LemLib — the library most VEX teams use — is **tank-only**, with no holonomic support
and no sensor fusion. We run X-drive and H-drive robots. shulib wins by being holonomic-native
(translation and rotation decoupled and simultaneous), fusion-based (odometry + IMU + GPS + AprilTags
behind swappable policies), and **usable by people who cannot write C++** — four tiers of progressive
disclosure, each a strict superset of the one below.

**It is a public repo.** shulib is meant to be usable by teams outside SHU, so the standards are not
academic: someone else will read this and trust it.

### What I want from you

Depth and honest pushback. **Tell me when something's a bad idea. Don't agree with me to be
agreeable.** Cut zero corners — always the thorough path over the convenient one. Verify before
claiming, and write your own check rather than trusting a report.

**Go slowly on documentation. I'd rather be slow and right than fast and wrong** — judges and future
team members will read all of it and won't know enough to notice when it's lying to them. **If a
sentence becomes hard to write honestly, that difficulty is the finding.**

Work thoroughly, and **push hard** — but the standards in §7 do not bend for speed.

---

## 2. Read this first, in this order

1. **[`RESUMING.md`](RESUMING.md)** — the working protocol, canonical.
2. **[`HANDOFF-2026-08-13-F2.md`](HANDOFF-2026-08-13-F2.md)** — the most recent chunk handoff.
3. **[`build-order.md`](build-order.md)** — "Current position" (long, worth it), then "The order at a
   glance".
4. Run these and read the output:
   ```sh
   git log --oneline -20
   git status
   ./build/test/shulib_tests | tail -3
   ```

**Baseline you should see: 1018 test cases / 1,522,327 assertions / 3 skipped, all passing.** ARM gate
clean at 124 headers. Tree clean. **25 commits unpushed.**

**If `git status` is dirty, a chunk was interrupted.** Read that chunk's `-PROGRESS.md` — appended in
real time, so it is an honest record of exactly how far the work got — before doing anything else.
**Never start a new chunk on a dirty tree.**

---

## 3. The governing constraint, and it governs everything

**The library has never driven a robot.**

On 2026-08-12 the package was built, uploaded and booted on a V5 brain. It constructed its whole
object graph on ARM and printed its diagnostics banner over USB, including a live `strafeAuthority`
query through the frozen facade. **It drove nothing, because it cannot** — every motor and sensor was
a fake, and the `hal/pros` adapters are R1, unwritten.

That run proves the build/upload/boot path works and that no host-only assumption breaks on ARM. It
proves **nothing** about motion, accuracy or control.

**Do not let "it booted" drift upward into "it works on a robot."** That distinction is currently
defended in six places across the README and the guide. Keep it defended — it is the single easiest
way for this project to start lying.

Everything is validated against the **A2 host plant**: a simulator that converts voltage into motion
behind the unmodified HAL fakes, made hostile at A3 (drift, garbage windows, sentinels, sag, slip,
latency, jitter — reproducible from a seed). It proves **logic, not constants**. **93 registered
hardware assumptions, none settled.**

---

## 4. The architecture — what exists, by layer

```
L0  hal/          the hardware seams (the ONLY thing that differs robot/sim/test)
L1  math/ units/  frames, angles, poses, typed quantities
L2  control/      Pid, Feedforward, TrapezoidProfile, ExitGroup, Watchdog
    kinematics/   IKinematics + X-drive / H-drive / tank presets, desaturate
    localization/ odometry, Localizer, fusion policies, correctors
    motion/       IMotion + primitives + MotionScheduler
    manipulation/ mechanism operations (F1)
    sequence/     the run guard (F2)
    diag/         DebugRecord, sinks, faults, blackbox, run summary
L3  chassis/      Chassis facade (frozen F6) + Routine recipe layer (frozen F10)
                  RobotContext — the composition root, the one object that differs
```

**Key files worth knowing by name:**

| File | What it is |
|---|---|
| `chassis/chassis.hpp` | The public facade. **FROZEN (F6).** Blocking verbs + `drive(speeds, Frame)` |
| `chassis/routine.hpp` | Tier-2 fluent recipe chain. **FROZEN (F10)**, except `then()` |
| `chassis/robot_context.hpp` | Composition root. Not frozen; grows additively |
| `motion/motion.hpp` | `IMotion` + the tick / cancel / wait-for-live contracts + `applyCancelSafeState` |
| `motion/motion_scheduler.hpp` | The **only loop in the tree**. `tick`/`waitUntilSettled`/`waitUntil`/`cancel`/`async`, re-entrancy guards, the fault policy |
| `localization/localizer.hpp` | Fused estimate + categorical quality; heading is IMU-owned |
| `localization/i_fusion_policy.hpp` | The swap point: complementary (default) ↔ EKF |
| `hal/mechanism.hpp` | `IMechanism` (F1) — two virtual members, `applySafeState()` + `name()`, plus a claim token |
| `manipulation/mechanism_op.hpp` | Bounded, tickable mechanism operations |
| `sequence/run_guard.hpp` | The run-scoped deadline + guaranteed end-of-run action (F2) |
| `version.hpp` | API version + the breaking-vs-additive policy. The mechanism behind every freeze |
| `spec/accuracy.hpp` | The **F2 register row** targets (NOT chunk F2 — see §8) |

**Two CI guards, both mutation-proven:** nothing in `include/shulib/` may include `<pros/*>`, and
nothing outside `sim/` may include `shulib/sim/`. **Four build-time doc gates** run at 1% of the
build: undocumented public member, stale generated reference, drifted example, internal link leak.

---

## 5. What is done — 22 chunks

**Phase A — the ground to stand on**
- **A1** `DebugRecord` + `TermSink` + fault discipline. Faults log and recover, never crash.
- **A2** The host plant + closed-loop sim harness. *The simulator is the robot you don't have* — and
  for estimator work it's better, because you possess ground truth.
- **A3** Hostile sensor fakes. Every V5 misbehaviour class injectable and composable. **Found 3 real
  Localizer defects.**
- **A4** Hardware Assumptions Register + the ARM compile gate.

**Phase C — make it move**
- **C1** `IMotion` + primitives. `MoveToPose` runs three **decoupled** per-axis loops — the holonomic
  thesis, mutation-proven. Every motion watchdog-bounded.
- **C2** `MotionScheduler`. One active motion, structurally. Cancel safe state defined (0 V + Brake).
  Fault policy decided (abort on ODO_STUCK only).
- **C3** H-drive + the full pseudo-inverse. **The 15″ H-bot runs the same motion code as the 24″
  X-bot, unmodified.**
- **C4** The `Chassis` facade — one composition root, structural command-id stamping.
- **C5** Per-motion results, session header, run summary. The run is legible end to end.
- **C6** Legacy salvage audit — 34 files classified, port list empty by audit.
- **C7** Cutover: `legacy/` deleted, `make` produces an uploadable package.
- **C8** The 15-chapter manual, with examples compiled and quoted verbatim so CI catches rot.

**Phase D — make it usable**
- **D1** Tier-2 recipe API (`Routine`) — eager fluent chain; the facade held its second consumer with
  zero changes.
- **D2** **F6 freeze** — and the first freeze in the project that means something mechanical.
- **D3** Cookbook + generated API reference + **F10 freeze**. All four doc gates moved to build time.

**Phase E — bound the drift**
- **E1** SD blackbox + flight recorder + estimator introspection. A run is recoverable with no laptop.
- **E2** `GpsCorrector` — drift is bounded, and the claim is sized to the evidence.
- **E3** `AprilTagCorrector` — absolute heading correction.
- **E4** 5-state SE(2) EKF. **It LOST the comparison its own DoD demanded** (0.351″ vs 0.225″, losing
  7 of 8 seeds), so the complementary filter stays the default. **That result was measured against
  invented noise — R4 could flip it.**

**Phase F — sequencing**
- **F1** The `Mechanism` seam + fakes. `IMechanism` earns its existence on **one verb**
  (`applySafeState`) because that is what F2's guard needs across heterogeneous mechanisms.
  Per-mechanism **declared** safe state (a lift holds; an intake coasts), with a physics split: motor
  ops safe on every exit, discrete actuators keep their commanded state on success and safe only on
  cancel — *a clamp whose safe state is "open" would fling its game piece the instant a grab
  succeeded.*
- **F2** The `sequence/` engine + guaranteed end-of-run action. See §11.

---

## 6. The Freeze Register — what is locked and what is not

| Row | Contract | Status |
|---|---|---|
| **F1** | Coordinate frame (origin field-centre, +X right, +Y from red, CCW+) | ✅ LOCKED |
| **F2** | **Accuracy targets** — heading `< 1.0°` hard, ~1.0″ pose, ~0.25″ docked | ✅ LOCKED |
| **F3** | Units & `Angle` semantics (inches, radians, seconds) | ✅ LOCKED |
| **F4** | The 10 runtime HAL interface signatures | ✅ LOCKED |
| **F5** | `IKinematics` contract | ✅ LOCKED |
| **F6** | Public `Chassis` API | ✅ LOCKED (D2) |
| **F7** | `robotProfile` sub-schema in `.vexbot` | 🎯 needs VexBuilder |
| **F8** | `paths[]` + command-id vocabulary | 🎯 needs VexBuilder |
| **F9** | `SHUL/2` telemetry wire | 🎯 M6 |
| **F10** | Public `Routine` API | ✅ LOCKED (D3) — `then()` excluded |
| **F11** | Mechanism seam + operation contract | 🚧 **NOT frozen, by design** |
| **F12** | Sequence engine + run guard | 🚧 **NOT frozen, by design** |

**F11 and F12 say "not frozen" out loud** because D2 learned that silence in that register reads as
"frozen". They freeze after a second real consumer stresses them — the build → second consumer →
freeze path F6 and F10 both took.

**Enforcement is structural.** `test/f6_signature_pin_test.cpp` and
`test/routine_signature_pin_test.cpp` fail the build **naming the row and the member** if a frozen
signature drifts — including a member **losing or gaining** `noexcept`.

---

## 7. How we work — the chunk loop

**One chunk at a time**, in `build-order.md` order.

### 1. Write a detailed brief
`docs/internal/chunks/<CHUNK>-<slug>.md`. **F2's brief is the best template.** It carries:
- why this chunk is here in the order
- what already exists to build on — **read the actual files**, cite `file:line`
- scope: **in**, **out** (naming the chunk that owns it instead), **explicitly rejected**
- the load-bearing constraints, each with its reasoning
- **the tensions to rule on, each requiring a rejected alternative**
- test requirements including the required mutations
- a DoD checklist
- **an explicit documentation scope** — which chapters change, and why
- landmines

**The brief is where the thinking goes.** Every recent brief found a contradiction nobody had noticed
— one where `build-order.md` demanded something a previous chunk had already ruled out, one where the
project's own flagship example had never compiled.

**Where it's cheap, MEASURE BEFORE YOU BRIEF.** F2's brief carried fifteen executable measurements,
and several overturned the design I would otherwise have written into it. shulib is header-only, so a
standalone probe compiles in seconds without touching the build directory:
```sh
g++ -std=gnu++20 -I include -I test -I test/vendor probe.cpp -o probe
```

### 2. Commit the brief

### 3. Run it with Fable
`Agent` tool, `model: "fable"`, `subagent_type: "general-purpose"`, `run_in_background: true`.

The prompt is a skill in itself. It must carry: **required reading in order**; **the non-negotiable
constraints restated inline** (the agent reads the prompt more carefully than the attachment); the
test bar with mutations named; **the verification commands verbatim**; the documentation contract;
**"do not commit, do not push"**; **"create `<CHUNK>-PROGRESS.md` FIRST and append continuously"** (I
watch it with `tail -f`, and it is what makes an interrupted chunk recoverable); and **the traps that
have bitten before, phrased for this chunk specifically**.

### 4. Verify independently — never take the report at face value
**This is the step that makes the process real.**

```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6

grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib && echo GUARD1-FAIL || echo "GUARD1 PASS"
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib && echo GUARD2-FAIL || echo "GUARD2 PASS"

find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo 'int main(){return 0;}' >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
```

`docs/internal/verify/` holds per-chunk harnesses. **Read `verify/README.md` first** — its central
lesson is *a red gate is a question, not a verdict.*

**Then do the part no script can do:** write your own oracle for the chunk's most load-bearing claim,
with **a negative control** — run the same scenario with the feature absent and prove your instrument
can tell the difference. Every time this has been done it found or confirmed something real.

**The harness at `docs/internal/verify/verify-<chunk>.sh` belongs to the REVIEWER.** F1's agent
overwrote the reviewer's harness with its own self-check and destroyed the independence of the audit.
Say so in the chunk prompt.

### 5. Commit
Only after verifying. Conventional style; the body explains the **reasoning** and names honest
partials. Trailer:
```
Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
```
**Do not push** unless asked.

---

## 8. Standards that do not bend

- **Evidence, not vibes.** A checkbox flips only with cited evidence (file + test + counts). `[~]` for
  partial, naming the chunk that owns the rest. **Under-claim before over-claiming.**
- **Tests must try to break the code.** Every test names, **in a comment, the bug it would catch** —
  not what it checks.
- **Mutation testing is mandatory** for load-bearing logic: break it, **rebuild**, run, **observe
  red**, restore. *A mutation you reasoned about but did not execute does not count.* **Gate the
  runner on build success** — a non-compiling mutation read off a stale binary looks green.
  **A mutation that stays GREEN is a hole in the suite and the most valuable thing you can find.**
- **Assertion counts flatter.** 1.5M assertions measures seeds swept, not independent checks. The
  README says so explicitly. Mutation results are the measure this project trusts.
- **Rule 4:** a chunk that finds a flaw in an earlier chunk **fixes it there**, not around it.
- **Clean-room:** re-derive, never port. Re-deriving `arcStep` this way caught a real legacy bug.
- **Documentation is a deliverable**, continuous, and never the thing compressed to save a chunk.
- **Plain English, no slogans.** If a line sounds like a tagline, cut it.
- **`docs/internal/` must stay cleanly removable** — no public doc may link into it. `main` is a squash
  of `release/v2` with that directory dropped, and the docs site publishes from `main`.
- **Constants are provisional.** Anything invented gets an `HA-nn` entry with blast radius and the
  measurement that would settle it. **Next free: HA-94.** Don't tune parameters until a sweep passes —
  that is fitting the test, not the code.

---

## 9. The traps — every one of these has actually happened

### 1. Shared models cancel their own errors (six chunks and counting)
If a test shares a conversion, a camera model, a kinematic model, a motion model **or a clock** with
the code under test, the error **cancels on both sides and the test passes while proving nothing.**
C1, C3, C4, E2, E3 and F2's campaign were each bitten.

The counter: a from-scratch oracle with hand-computed literals, **plus a negative control**. Examples
that earned their keep — E2's oracle found the *existing* test imported the very constant it was
pinning; E3's independent projector confirmed a reversed corner winding returns `valid=true` with
heading 180° wrong; F2's verification measured 0.0 in of post-deadline travel *and* 360 in with the
guard absent, which is what makes the zero mean anything.

### 2. A biased skeptic manufactures a false all-clear
A red-team this session reported **23 findings and 0 survivors** — because the refuting agents were
told to "default to refuted". **Two of the "refuted" findings were real** and both changed a brief.
*A red gate is a question; a green verdict from a skeptic you biased is also a question.* If you use
adversarial verification, make the verifier argue both sides and report confidence, never a verdict.

### 3. Prose goes stale in ways no gate can see
The four doc gates are solid and have **no opinion about whether a sentence is still true.** Found by
*reading*, in single sessions: a chapter claiming 57 assumptions when there were 67; a README claiming
659 tests twenty lines from a claim of 867; an API chapter describing an exception that had been
deleted; a chapter claiming no cookbook existed after D3 shipped one. **Budget reading time, not just
gate time.**

### 4. Frozen means frozen
F6 and F10 change only by major version bump plus a migration note. If a pin fires on a signature,
**stop and report** — that's a breaking change to argue, not an edit to make. Note that the doc
**freshness** gate fires *before* the pins, naming the wrong problem; D3 documented this and it bit
again at F1's verification.

### 5. Name collisions between chunks and freeze rows
Chunk F1/F2/F3/F4 vs **Freeze Register rows F1–F5** (frame, accuracy, units, HAL, kinematics — all
LOCKED). **The collision is in shipped code**: `spec/accuracy.hpp` is titled "the F2 accuracy
targets". **Never edit rows F1–F5.** The verify harnesses gate on this.

### Process failures that actually happened
- A mutation runner **piped into `head`** took a SIGPIPE mid-campaign and left a header with a line
  deleted. Trap `PIPE`, count what you ran, and **never** `git checkout` a file holding uncommitted
  work.
- A **non-compiling mutation nearly read as green** off a stale binary (C4). Gate on build success.
- A **results table was nearly fabricated** before the measurement existed (E4). Write
  `PENDING MEASUREMENT`, measure, then write the number.

---

## 10. What good work looks like here

The best moments in this project have all been **retractions**:
- **E2** measured its accuracy claim, found 7/8 and 6/8 rather than 8/8, discovered a shorter scenario
  that *did* pass 8/8, and **refused to switch to it** as scenario-shopping.
- **E3** reported a headline bug, realised it had **inferred a sign** from an absolute value, and
  **struck its own finding.**
- **E4** reported that **the EKF lost** the comparison its own DoD demanded, did not tune toward the
  expected answer, and made the simpler filter the default on the strength of that measurement.
- **F2** found a green mutation *by planning the campaign* and closed it before running.

**A chunk that reports only wins has not looked hard enough.** Measure first, then claim, and size the
claim to the evidence. A test that disagrees with you is doing its job — consider that your
expectation was wrong before you edit the assertion.

---

## 11. F2 in detail — the most recent chunk, and the one most likely to be over-quoted

`sequence/run_guard.hpp` owns the run-scoped deadline and the **guaranteed end-of-run action**. D-8
(the routine watchdog E1 re-homed) and the end action are **one primitive with two policies**.

It is an `ITickPacer` decorator, **inert until `run()`**. At the caller's `endActionAt` it cuts the
active motion with zero latency, **refuses** every later motion (a latch — cancel-only expiry measured
*inert and marginally counterproductive*), performs cancel-all strictly before the caller's end
action, and enforces an unconditional `hardStopAt` floor.

**The library refuses to know your strategy.** No field coordinate, no park pose, no default lead
time, no default match length. Both instants and the action are caller-supplied. A team that ends
somewhere else, or doesn't park at all, is not fighting the library. `endInMidfield` is F4's, with the
students' own numbers.

**What it cannot do, and this is written in the header and guide ch. 14 rather than buried:** claim
real-brain timing margins (loop rate and PROS latency are invented until R4); preempt pure user code
(no background tasks exist); cut the frozen F10/F6 waits, which pay their full remainder — **a limit
that is tested, not hidden**; or end an unconditional retry loop. And **"zero post-deadline travel" is
a simulator result** — the plant is memoryless, so on a robot with mass the honest claim is *"no new
commanded motion"*.

**Verified by four independent reviewer probes**, each with a negative control: the guarantee held
across three stalls (0.23 in from the caller's pose; 95 in away with no guard), the latch measured
0.0 in against 360 in unguarded, the no-defaults audit translated the manoeuvre 1450 in off any VEX
field and got agreement to 9.1e-12 in, and the C2 fix regressed nothing.

---

## 12. What is left — 21 chunks

| Phase | Chunks | Gate |
|---|---|---|
| **R — Robot arrival** | R1–R6 | **hardware (available now)** |
| **T — Driver control** | T1–T3 | none (T1 folding into R1) |
| **G — No-code authoring** | G1–G4 | G1 ungated; G2–G4 need VexBuilder |
| **H — Ecosystem** | H1–H3 | H2 needs VexBuilder's sim |
| **F′ — Scoring primitives** | F3–F4 | hardware + final mechanism decisions |
| **E′ — Field accuracy** | E5–E6 | hardware + a field |
| **I — Second robot** | I1–I2 | both robots |

**Milestones:** M0 ✅ · M1 🎯 closes at R1–R3 · **M2 open on exactly one clause — a robot that moves**
· M3 at E6 · M4 at F4 · M5 at G4 · M6 at H3 · M7 largely delivered (one honest gap: no beginner has
read the cookbook cold).

---

## 13. THE IMMEDIATE TASK — R1: `hal/pros` adapters + the `IController` seam

**Write the brief first.** R1 is the chunk that puts the library on a robot, and nothing blocks it.

**Scope:**
- **The 9 F4 adapters** — `IClock`, `IMotor`, `IRotation`, `IImu`, `IGps`, `IDistance`, `IOptical`,
  `IBattery`, `ITelemetrySink`. **The only files in the tree permitted to `#include <pros/*>`.** The
  conversion pure-functions (`imu_conversion.hpp`, `gps_conversion.hpp`) are built and red-teamed —
  they wire in here, at the boundary, exactly once.
- **The F1/F11 seams on hardware** — `IMotor` groups behind `MotorMechanism`, ADI digital-out behind
  `IDigitalOut`.
- **`IController` (Phase T's T1, folded in here by my direction)** — axes normalized to `[-1, 1]` and
  buttons as bools **at the adapter edge** (PROS's `-127..127` never reaches the core), new-press edge
  detection, `isConnected()` as a positive validity signal (a controller really does drop mid-match),
  **partner controller from the start** (VEX U runs two drivers; retrofitting a second through a
  single-controller seam is the reshape a seam exists to prevent). An **F4-additive sibling, outside
  that freeze**, exactly as F1's `IDigitalOut` was.
- **`src/main.cpp`'s `TODO(R1)` lines** — each swaps a fake for an adapter and *nothing else changes*.
  Plus the real tick pacer (`pros::Task::delay_until`), the on-robot precondition policy, and session
  header emission.

**The CI guard must be amended** — `hal/pros/*` becomes the one allowed directory. Amend it
deliberately and **prove the amended guard still catches a violation elsewhere.**

**DoD:** every F4 interface has a PROS-backed implementation; the CI guard still passes; the ARM build
compiles them.

**Then R3** (day-one validation — walks the assumptions register top to bottom; **closes M1 and M2's
on-robot clause, open since June**), **then R4** (sensor characterization — replaces A3's invented
noise magnitudes with measured ones; **the highest-information work available**, because E4's headline
result rests on invented numbers).

---

## 14. Sensors — the recommendation, and the question I owe you an answer to

**Seams that exist:** `motor` (encoder + velocity + current + temperature), `rotation`, `imu`, `gps`,
`distance`, `optical`, `vision` (AI Vision: objects *and* AprilTags), `battery`, `clock`,
`digital_out`, `line_display`.

**Seams that DO NOT exist:** **digital input** (limit switches, bumpers), analog input
(potentiometers, line trackers), legacy 3-wire (ultrasonic, optical shaft encoders) — and controller
input until R1.

**For the new competition bots, ranked by accuracy-per-port:**

1. **IMU** — one, centre-mounted, isolated from vibration and away from motors. Heading is the largest
   error term and is IMU-owned by design; the `< 1.0°` target lives or dies here.
2. **Two tracking wheels (forward + lateral) on Rotation Sensors** — unpowered, spring-loaded. These
   are what make odometry survive wheel slip, and X/H drives slip.
3. **GPS** — the only absolute position source; bounded drift over a 60 s run depends on it. **Needs
   the field strip; Driving Skills has none**, so it must degrade gracefully (E2 handles this).
4. **AI Vision** — AprilTags give absolute *heading* (E3 proved the corrector), objects give bearings
   for manipulation. R2 is already scoped for it.
5. **Optical on each scoring mechanism** — the sensor-confirm on every grab/place that §14 calls
   non-negotiable: *never advance on a failed grab.*
6. **Distance** — dock confirm and a cheap wall-reference cross-check.
7. **Limit switches for lift homing** — **only if the lift homes against a stop.**

**THE QUESTION THAT MUST BE ANSWERED BEFORE R1's BRIEF IS FINAL: does the lift home against a limit
switch or against a stall?** F1 built **stall** detection (current + velocity), so current-spike
homing needs no new seam. **A switch needs a digital-input seam that does not exist.** Cheap to add in
R1; expensive to discover at R3 with the robot on the bench.

**For the old tank practice bot: read the device list off the brain** (`pros` terminal) rather than
assuming — I'm told there isn't much on it.

---

## 15. Hardware reality

Available now: a **V5 brain** (boots this code), an **old tank/differential practice bot**, and
sensors.

**What the tank bot validates:** the platform layer — the 9 adapters, IMU drift and calibration
window, GPS noise and latency, encoder refresh, brownout threshold, real loop rate under load, PROS
call latency, the odometry push test, the GPS field-cal axis oracle (`gps_conversion_test.cpp:186`,
currently skipped) — and the **tank** kinematics path.

**What it does NOT validate:** any of the holonomic thesis. No strafe authority, no pseudo-inverse, no
per-axis decoupling, no H-drive geometry. Those register entries stay open until a competition robot
exists, and the register should **say so** rather than imply coverage.

**Two traps:** **never carry gains across chassis** — kS/kV/kA and PID gains are mass-, friction- and
geometry-dependent, and a measured-looking wrong number is worse than an honest placeholder. And
**record anything measured on the practice bot with that provenance**, not as "measured".

**Bench notes:** I'm in `dialout`; the brain enumerates as `/dev/ttyACM0` (system) and `/dev/ttyACM1`
(user). `pros terminal` needs a real TTY, so under automation wrap it
(`script -qec "pros terminal" /dev/null`) — a raw read of `ttyACM1` yields nothing, because the user
port is framed rather than plain serial.

---

## 16. The guardrail

The **library** is built this way. The **competition routines** (Phase F′) and **authored paths**
(Phase G) are strategy **my students must write and be able to defend** — those chunks deliver
primitives and engines and stop short of authoring the season's content. Architect, teach and review;
**don't write their auton.**

This is why F2 ships no park pose and why `buildStack` / `matchLoadCycle` / `endInMidfield` /
`strategyMode` belong to F4, not to the engine.

---

## 17. Release and the docs site

`shulib-v2` → merge into `release/v2` → drop `docs/internal/` → squash that tree onto `main`. The
history shows it: `git log --first-parent main`.

The site publishes from `main` to **docs.shurobotics.com** via GitHub Pages. Publishing from `main` is
deliberate: the development record cannot reach the public site **by construction** rather than by
configuration. `tools/prepare_site.py` refuses to build if an internal doc would be published.

**The rule that must survive:** the publish job runs `python3 tools/api_doc_tool.py check-fresh`
before rendering. Publishing a stale reference is worse than publishing none, because a published
document looks authoritative.

**Pending:** GitHub has not issued the TLS certificate; DNS is correct and the site serves over HTTP.
Tick "Enforce HTTPS" once the cert appears. If Cloudflare's proxy is re-enabled, SSL mode must be
**Full (strict)**, never Flexible.

---

## 18. Open decisions for me

1. **Which sensors are going on the robots, and does the lift home on a switch or a stall?** (§14 —
   decides whether R1 adds a digital-input seam)
2. **Whether to push.** 25 commits unpushed; `main`/`release/v2` are a full release behind and carry
   none of E1–E4, F1, F2 or Phase T. The docs site publishes from `main`, so none of it is public.
3. **R5 timing.** Building `tools/sysid` on the tank bot gives a validated tool and throwaway numbers,
   since gains never transfer across chassis. It may be worth deferring until a competition robot
   exists and slotting G1 or H1 in instead. **This is the one place in the order I want your judgment
   rather than mine.**
