# shulib v2 — session handoff (paste this whole file into a new chat)

I'm the VEX U programming chair at Seton Hall, building **shulib v2**, a holonomic-native autonomous
library for VEX U, at `/home/gonzei/projects/shulib` (branch `shulib-v2`).

**22 of 43 chunks are done. Phases A, C, D, E and F are complete.** The process is mature and it is
the reason this works — read before you build.

---

## Get up to date (in order, do this first)

1. **`docs/internal/RESUMING.md`** — the working protocol. The entry point.
2. **`docs/internal/HANDOFF-2026-08-13-F2.md`** — written for you at the end of F2.
3. **`docs/internal/build-order.md`** — "Current position", then "The order at a glance".
4. Run these and read the output:
   ```sh
   git log --oneline -15
   git status
   ./build/test/shulib_tests | tail -3
   ```

**Baseline you should see: 1018 test cases / 1,522,327 assertions / 3 skipped, all passing.** ARM gate
124 headers. Tree clean. **20 commits unpushed.**

**If `git status` is dirty, a chunk was interrupted** — read that chunk's `-PROGRESS.md` before doing
anything. Never start a new chunk on a dirty tree.

---

## What I want from you

Depth and honest pushback. **Tell me when something's a bad idea. Don't agree with me to be
agreeable.** Cut zero corners — always the thorough path over the convenient one. Verify before
claiming, and write your own check rather than trusting a report.

**Go slowly on documentation.** Judges and future team members will read it and won't know enough to
notice when it's lying. If a sentence becomes hard to write honestly, **that difficulty is the
finding.**

**We follow the established build order** — but it is a living document, not scripture. It has been
amended twice with reasons recorded (C8 added the manual; Phase T added driver control), and
deviations go in its deviations table rather than happening silently. **Push hard, but the standards
below do not bend.**

---

## THE FIRST TASK: R1 — `hal/pros` adapters + the `IController` seam

This is the chunk that puts the library on a robot. **Write the brief first** (see the chunk loop
below). It must cover:

- The **9 F4 adapters**: `IClock`, `IMotor`, `IRotation`, `IImu`, `IGps`, `IDistance`, `IOptical`,
  `IBattery`, `ITelemetrySink`. These are the ONLY files in the tree permitted to `#include <pros/*>`.
  The conversions (`imu_conversion.hpp`, `gps_conversion.hpp`) are already built and red-teamed — they
  wire in here, at the boundary, once.
- **`hal/pros` for the F1/F11 seams too**: `IMotor` groups behind `MotorMechanism`, and ADI digital-out
  behind `IDigitalOut`. F1 built both; R1 is where they meet hardware.
- **The new `IController` seam (Phase T's T1, folded in here by my direction)** — axes normalized to
  `[-1, 1]` and buttons as bools **at the adapter edge** (PROS's `-127..127` never reaches the core),
  new-press edge detection, `isConnected()` as a positive validity signal, **partner controller from
  the start** (VEX U runs two drivers). An F4-additive sibling, **outside** that freeze, exactly as F1's
  `IDigitalOut` was.
- **`src/main.cpp`'s `TODO(R1)` lines** — each swaps a fake for an adapter and nothing else changes.
  Plus the real tick pacer (`pros::Task::delay_until`), the on-robot precondition policy, and session
  header emission.
- **A digital-INPUT seam if we need one** — see the sensor question below.

**R1's DoD:** every F4 interface has a PROS-backed implementation; the CI guard still passes; the ARM
build compiles them.

After R1: **R3** (day-one validation — closes M1 and M2's on-robot clause, open since June), then
**R4** (sensor characterization — the highest-information work available; see below).

---

## Sensors — I asked for a recommendation; here is the state and the question back to you

**Seams that exist today:** `motor` (encoder + velocity + current + temperature), `rotation` (tracking
wheels), `imu`, `gps`, `distance`, `optical` (colour/proximity), `vision` (AI Vision: objects *and*
AprilTags), `battery`, `clock`, `digital_out` (pneumatics), `line_display`.

**Seams that DO NOT exist:** digital **input** (limit switches, bumpers), analog input
(potentiometers, line trackers), legacy 3-wire (ultrasonic, optical shaft encoders), and until R1,
controller input.

**My recommendation for the new competition bots, ranked by accuracy-per-port:**

1. **IMU (one, centre-mounted, isolated from vibration)** — heading is the single largest error term
   and it is IMU-owned by design. Mount it near the rotation centre, away from motors.
2. **Two tracking wheels (forward + lateral) on Rotation Sensors** — unpowered, spring-loaded. These
   are what make odometry survive wheel slip, and the X/H drives will slip.
3. **GPS** — the only absolute position source; bounded drift over a 60 s run depends on it.
   **Note it needs the field strip; Driving Skills has none, so it must degrade gracefully.**
4. **AI Vision** — AprilTags give absolute *heading* (E3 proved the corrector works) and object
   bearings for manipulation. High value, and R2 is already scoped for it.
5. **Optical** on each scoring mechanism — the sensor-confirm on every grab/place that §14 calls
   non-negotiable ("never advance on a failed grab").
6. **Distance** — dock confirm and a cheap wall-reference cross-check.
7. **Limit switches for lift homing** — *only if your lift homes against a stop.* F1 built **stall**
   detection (current + velocity), so current-spike homing needs no new seam. **A switch does.**

**The question I need answered before R1's brief is final: which of these are actually going on the
robots, and does the lift home against a switch or against a stall?** That decides whether R1 adds a
digital-input seam. Cheap now, expensive at R3.

**For the old tank practice bot: read what's on it off the brain** (`pros` device list over the
terminal) rather than assuming — I'm told it isn't much.

**And the trap that matters:** the practice bot validates the **platform** layer (IMU drift and
calibration window, GPS noise and latency, encoder refresh, brownout, real loop rate under load, PROS
call latency, the odometry push test) and the **tank** kinematics path. It exercises **none of the
holonomic thesis** — no strafe authority, no pseudo-inverse, no per-axis decoupling. **Never carry
gains across chassis** (kS/kV/kA and PID are mass-, friction- and geometry-dependent), and record
anything measured on it **with that provenance**, not as "measured".

---

## How we work — the chunk loop

**One chunk at a time.**

1. **Write a detailed brief** → `docs/internal/chunks/<CHUNK>-<slug>.md`. **F2's is the best template.**
   It carries: why this chunk is here; what exists to build on (**read the actual files** — cite
   `file:line`); scope **in / out / explicitly rejected**; load-bearing constraints with reasoning;
   **tensions to rule on, each requiring a rejected alternative**; test requirements including named
   mutations; a DoD checklist; an explicit documentation scope; landmines.
   **The brief is where the thinking goes.** Every recent brief found a contradiction nobody had noticed.
2. **Commit the brief.**
3. **Run it with Fable** — `Agent` tool, `model: "fable"`, `subagent_type: "general-purpose"`,
   `run_in_background: true`. The prompt must restate the non-negotiables **inline** (the agent reads
   the prompt more carefully than the attachment), carry the verification commands verbatim, say
   **"do not commit, do not push"**, and require a `-PROGRESS.md` created FIRST and appended
   continuously (I watch it with `tail -f`; it makes an interrupted chunk recoverable).
4. **Verify independently — never take the report at face value.** This is the step that makes the
   process real.
   ```sh
   cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6

   grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib && echo GUARD1-FAIL || echo "GUARD1 PASS"
   grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib && echo GUARD2-FAIL || echo "GUARD2 PASS"

   find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
   echo 'int main(){return 0;}' >> /tmp/all.cpp
   arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
     -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
   ```
   **NOTE FOR R1: the PROS guard will need its scope amended** — `hal/pros/*` is the one directory
   allowed to include `<pros/*>`. Amend the guard deliberately and prove the amended guard still
   catches a violation elsewhere.
   Then **write your own oracle** for the chunk's most load-bearing claim. Every time I've done this it
   found something.
5. **Commit** — only after verifying. Conventional style; the body explains the **reasoning** and names
   honest partials. Trailer:
   `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`
   **Do not push** unless asked.

---

## Standards that do not bend

- **Evidence, not vibes.** A checkbox flips only with cited evidence (file + test + counts). `[~]` for
  partial, naming the chunk that owns the rest. **Under-claim before over-claiming.**
- **Tests must try to break the code.** Every test names, in a comment, the bug it would catch.
- **Mutation testing is mandatory**: break it, **rebuild**, run, **observe red**, restore. A mutation
  you reasoned about but did not execute does not count. **Gate the runner on build success.**
  **Never pipe it into `head`** (a SIGPIPE once deleted a line from a header). **A mutation that stays
  GREEN is a hole and the most valuable thing you can find.**
- **Rule 4:** a chunk that finds a flaw in an earlier chunk **fixes it there**.
- **Clean-room:** re-derive, never port.
- **Documentation is a deliverable**, continuous, never compressed to save a chunk.
- **Plain English, no slogans.** If a line sounds like a tagline, cut it.
- **`docs/internal/` must stay cleanly removable** — no public doc may link into it.
- **Invented constants get an `HA-nn` entry** with blast radius. **Next free: HA-94.** 93 registered,
  **none settled.**

---

## The five things that will bite you

1. **Shared models cancel their own errors — this has bitten six chunks.** If a test shares a
   conversion, a camera model, a motion model or a *clock* with the code under test, the error cancels
   on both sides and the test passes while proving nothing. The counter is a from-scratch oracle with
   hand-computed literals, **and a negative control** — run the same scenario with the feature absent
   and prove your instrument can tell the difference.
2. **A biased skeptic manufactures a false all-clear.** A red-team this session reported 23 findings
   and **0 survivors** — because I told the refuters to "default to refuted". Two of the "refuted" ones
   were real and both changed a brief. *A red gate is a question; a green verdict from a skeptic you
   biased is also a question.*
3. **Prose goes stale in ways no gate can see.** The four build-time doc gates are solid and have no
   opinion about whether a sentence is still true. Budget reading time, not just gate time.
4. **Frozen means frozen.** F6 (`Chassis`) and F10 (`Routine`) change only by major version bump plus a
   migration note. The pins now catch a member **losing or gaining** `noexcept`. If a pin fires on a
   signature, **stop and report** — that's a breaking change to argue, not an edit.
5. **Name collisions.** Chunk F1/F2/F3/F4 vs **Freeze Register rows F1–F5** (frame, accuracy targets,
   units, HAL, kinematics — all LOCKED). The collision is in shipped code. **Never edit rows F1–F5.**

---

## The governing constraint

**The library has never driven a robot.** A V5 brain booted the package on 2026-08-12 with fake HAL
objects and drove nothing. **Do not let "it booted" drift upward into "it works on a robot."** That
distinction is defended in six places across the README and guide. Keep it defended.

---

## Where things stand

**Done:** A1–A4 (diagnostics, host plant, hostile fakes, assumptions register + ARM gate) · C1–C8
(motion, scheduler, H-drive, `Chassis`, results, salvage, cutover, the manual) · D1–D3 (recipe API,
**F6 freeze**, cookbook + generated docs + **F10 freeze**) · E1–E4 (blackbox, GPS corrector, AprilTag
corrector, EKF) · **F1** (mechanism seam) · **F2** (sequence engine + guaranteed end-of-run action).

**Frozen:** F1 frame · F2 accuracy · F3 units · F4 HAL · F5 kinematics · **F6 `Chassis`** ·
**F10 `Routine`**. **F11** (mechanism seam) and **F12** (sequence engine) exist and are explicitly
**NOT frozen** — stated out loud, because silence in that register reads as "frozen".

**Milestones:** M0 ✅ · M1 🎯 closes at R1–R3 · **M2 open on exactly one clause — a robot that moves** ·
M3 at E6 · M4 at F4 · M5 at G4 · M6 at H3 · M7 largely delivered.

**Remaining 21 chunks:** R1–R6 (hardware) · T1–T3 (driver control — T1 folding into R1) · G1–G4
(VexBuilder) · H1–H3 (ecosystem) · F3–F4 (scoring primitives, hardware + final mechanisms) · E5–E6
(field) · I1–I2 (second robot).

**Recent notable findings, so you don't re-derive them:** F2's guarantee was built on **fifteen
measurements** from executable probes, several of which overturned the obvious design (a
deadline-cancelling pacer unwinds `waitUntilSettled` and *only* that; cancel-at-the-deadline is inert
and marginally counterproductive; driving the end action from the pacer silently lies to the caller).
Four Rule 4 defects were fixed in earlier chunks this session: vacuous version pins, freeze pins blind
to a *gained* `noexcept`, an F1 operation destructor leaving a mechanism claimed and energized, and a
C2 unwind path that was a **use-after-free** with leg-scoped motions.

---

## The guardrail

The **library** is built this way. The **competition routines** (Phase F′) and **authored paths**
(Phase G) are strategy my students must write and be able to defend. Deliver primitives and engines;
stop short of authoring the season's content. Architect, teach and review — don't write their auton.

---

## Open decisions for me

1. **Which sensors are going on the robots, and does the lift home on a switch or a stall?** (decides
   whether R1 adds a digital-input seam)
2. **Whether to push.** 20 commits unpushed; `main`/`release/v2` are a full release behind and carry
   none of E1–E4, F1, F2 or Phase T. The docs site publishes from `main`. The release flow is: merge to
   `release/v2` → drop `docs/internal/` → squash onto `main`.
