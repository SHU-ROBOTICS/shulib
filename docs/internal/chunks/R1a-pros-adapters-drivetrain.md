# Chunk R1a — `hal/pros` adapters: the drivetrain, the driver, and the seam that makes them testable

> **The chunk that puts shulib on a robot.** Nine PROS-backed adapters, two new HAL seams, a host
> shim that makes adapter glue testable for the first time, both CI guards amended and re-proven, and
> `src/main.cpp` rewired so the fakes come out.
>
> **Status:** brief. Not started.
> **Predecessor:** F2 (sequence engine), closed 2026-08-13.
> **Live log:** `R1a-PROGRESS.md` — create it FIRST, append as you go.

---

## 0. What changed about R1 before this brief was written

Four things, each decided by the team lead on 2026-08-13 and each recorded here with the alternative
that was rejected, because three of them contradict what `build-order.md` says today.

| # | build-order.md says | Ruling | Why |
|---|---|---|---|
| 1 | "R1 — the 9 F4 adapters" | **R1 splits into R1a and R1b** | Scope grew to 15 adapters + 2 seams + a shim + `main.cpp` + a bench session. That is 3–4× a chunk. F2 was one header. |
| 2 | DoD is code-only: "every F4 interface has a PROS-backed implementation; the CI guard still passes; the ARM build compiles them" | **DoD keeps that, and adds a recorded bench smoke session** | R1's DoD was written when there was no robot. There is one now. |
| 3 | (silent) | **A digital-input seam lands, in R1b** | The lift-homing question is *still open* — see §1.3. The seam is added on the "cheap now, expensive at R3" argument. |
| 4 | (silent) | **`IController` folds in here** (Phase T's T1) | R1 is the PROS-adapter chunk anyway, and driver control needs that seam. |

**`build-order.md` must be amended by this chunk** — the R1 entry, the `Next:` pointer, the phase
table's chunk count, and the deviations table. A brief that contradicts the order document and does
not fix it leaves two documents disagreeing, which is how the spine rots.

### 0.1 The split, and the argument for where the line falls

**R1a — everything the drivetrain and a driver need.**
`IClock`, `IMotor`, `IRotation`, `IImu`, `IGps`, `IBattery`, `ICharSink`, `ILineDisplay`,
`IController` (new seam), plus the real tick pacer. The shim. Both guard amendments. `main.cpp`.

**R1b — everything a *mechanism* needs.**
`IDistance`, `IOptical`, `IDigitalOut`, `IDigitalIn` (new seam), `IBlockSink`.

The line is not convenience. Every R1b adapter serves a mechanism or a mechanism's confirmation
sensor, and mechanisms are **F3's**, which is gated on the build team's final lift/clamp/intake
decisions — decisions that are not made. Every R1a adapter is needed to move the robot or to let a
person drive it, and nothing external gates that.

**R1b must land before R3.** R3 walks the assumptions register top to bottom, and the register has
entries that only a distance sensor or an optical sensor can settle. R1b is not optional and it is
not "later"; it is the next chunk.

**Rejected:** splitting by *risk* — structural work plus three adapters, then twelve mechanical ones
across two chunks. It is the lower-risk order and it was declined for a real reason: it puts the
robot-moves milestone behind two chunk boundaries instead of one, and M1/M2's on-robot clause has
been open since June.

### 0.2 One correction to the scope list everyone has been quoting

**`ITelemetrySink` does not need a PROS adapter, and the "nine F4 adapters" list is wrong to include
it.** It already has three implementations — `NullSink`, `TermSink`, `SdSink` — and none of them
touch PROS. What is PROS-backed is one layer *below* it: the **device** seams its sinks write
through, `ICharSink` (R1a, USB serial) and `IBlockSink` (R1b, `/usd`). Counting `ITelemetrySink` as an
adapter double-counts `ICharSink` and hides `IBlockSink`.

So the honest count of PROS-backed device adapters in R1a is **nine**: clock, motor, rotation, imu,
gps, battery, char-sink, line-display, controller — plus the tick pacer, which is a `motion::ITickPacer`
rather than a `hal::` interface and is listed separately for that reason.

### 0.3 The question that is still open, and must not be forgotten

**Does the lift home against a limit switch or against a stall?** Asked; the answer is *not decided
yet*, and the ruling was **add the digital-input seam anyway** (R1b).

This brief records the honest shape of that: the seam gets built on an option, not on a consumer.
That is a deliberate departure from F1's standard — `IMechanism` earned its two virtual members on
one verb a real consumer needed, and this seam has no consumer at all. Two things keep it defensible,
and both must be respected in R1b:

1. **The shape is not a guess.** A digital input has one degree of freedom (`bool state()`), and every
   other question — what "pressed" means physically, whether there is a validity channel — is already
   ruled by `IDigitalOut`'s header (`digital_out.hpp:37-44`, `:18-20`): meaning belongs to the
   mechanism that owns the line, and there is no validity signal.
2. **R1b ships the seam, the adapter and the fake — and no homing routine.** A homing operation is
   F3's, and F3 is season content the students author (§16 guardrail). Building the routine here
   would be answering the strategy question by writing code, which is exactly the thing the guardrail
   forbids.

**If the answer comes back "stall", the seam is a small unused sibling.** That is the cost, it is
cheap, and it is stated rather than discovered.

---

## 1. Why this chunk is here in the order

Because **the library has never driven a robot, and this is the only thing standing between it and
doing so.**

On 2026-08-12 the package built, uploaded and booted on a V5. It constructed the whole object graph on
ARM and printed a live `strafeAuthority` query through the frozen facade. It drove nothing, **because
it cannot**: every motor and every sensor in `src/main.cpp` is an in-memory fake, marked `TODO(R1)` at
the exact line an adapter replaces (`src/main.cpp:129-143`). Fourteen `TODO(R1)` markers are
outstanding in shipped code.

Everything above the HAL is done and proven against the A2 plant: three drivetrains, decoupled
per-axis holonomic motion, a fused estimator with two selectable policies, a recipe layer, a
sequencer with a guaranteed end-of-run action, 1,018 test cases. **None of it can reach a motor.**

R1a is also the last chunk before the project's two oldest open clauses can close. R3 closes M1 and
M2's on-robot clause, open since June, and R3 is blocked on R1a and R1b. R4 — which replaces A3's
invented sensor-noise magnitudes with measured ones, and is the highest-information work available
because E4's headline result rests on invented numbers — is blocked behind R3.

---

## 2. What already exists to build on

### 2.1 The seams that must be filled

Every one of these is read code, cited by line.

| Seam | File | The contract R1a must honour |
|---|---|---|
| `IClock` | `hal/clock.hpp:27-28` | seconds, **monotonic non-decreasing**; "the V5's milliseconds are converted to seconds exactly once, in the hal/pros adapter" (`:9-10`) |
| `IMotor` | `hal/motor.hpp:30-69` | volts in, **clamped to ±12 and non-finite REJECTED** (`:8-11`); `position()` **cumulative, non-wrapping** (`:12-14`); current in amperes, "the pros adapter converts mA→A once at the edge" (`:61-62`); temperature a bare double °C |
| `IRotation` | `hal/rotation.hpp:24-28` | cumulative non-wrapping radians; "the hal/pros adapter converts centidegrees→radians and applies the sensor's reversed flag exactly once" (`:4-6`) |
| `IImu` | `hal/imu.hpp:26-38` | canonical CCW radians wrapped (-π,π]; `isReady()` **positive polarity** (`:31-33`) |
| `IGps` | `hal/gps.hpp:30-37` | robot-**centre** pose, lever-arm and frame corrected; when `hasFix()==false` the pose is unspecified **but MUST be finite and MUST NOT throw** (`:27-29`) |
| `IBattery` | `hal/battery.hpp:23-30` | canonical volts, canonical amperes, capacity in **[0,1]** |
| `ICharSink` | `hal/char_sink.hpp:31-32` | verbatim bytes, synchronous, **MUST NOT throw**; one call = one complete line (`:14-16`) |
| `ILineDisplay` | `hal/line_display.hpp:46-48` | 3 rows × 19 cols, **TRUNCATE never wrap** (`:20-21`), MUST NOT throw; geometry is **PROVISIONAL, HA-57, unverified until R1** (`:19-20`) |
| `ITickPacer` | `motion/motion_scheduler.hpp` | the only seam that regains control mid-motion (F2 depends on this) |

### 2.2 The conversions that are already built, red-teamed, and waiting

This is the part of R1a that is *already done*, and it is the reason the chunk is tractable.

- **`hal/imu_conversion.hpp`** — pure, PROS-free, host-tested. Carries an explicit **ADAPTER BINDING
  CONTRACT** (`:15-26`) that R1a must obey clause by clause: bind to `get_rotation()` **not**
  `get_heading()` (HA-03); **never** call `tare`/`tare_rotation`/`set_rotation`/`tare_heading`/
  `set_heading`/`reset` after calibration (HA-05); `bootHeading` applied exactly once, here.
- **`hal/gps_conversion.hpp`** — same shape, and a binding contract that is even more specific
  (`:28-44`): construct `pros::Gps` via the **PORT-ONLY** ctor, never `set_offset()` /
  `initialize_full()` / the offset-taking ctors, because a firmware offset makes `get_position()`
  report the centre and the adapter then **double-subtracts** the lever arm (HA-06); boot-check
  `get_offset() == (0,0)`; screen `PROS_ERR_F` to `hasFix()==false` **before** calling the conversion,
  because feeding a sentinel in **throws by design** (HA-08); call `gpsRmsErrorToCanonical()` for
  metres→inches (HA-07).

  That last one has history worth reading: until E2 the metres→inches obligation was *prose addressed
  to a future adapter author* with no code and no test, and the existing test imported the very
  constant it was pinning. E2 fixed it into a function specifically **"so this is a function the
  adapter CALLS rather than a paragraph the adapter author must remember"** (`gps_conversion.hpp:41-42`).
  R1a is that adapter author. Call the function.

- **`hal/vision_conversion.hpp`** — R2's, not R1a's.

### 2.3 What the fakes tell you the adapters must do

`hal/fake/fake_motor.hpp:18-24` is the reference implementation of the `IMotor` contract, and it does
one thing that is easy to get wrong: on a non-finite voltage it raises `SHULIB_PRECONDITION`. It does
**not** coerce to zero.

This matters more than it looks — see landmine L4.

### 2.4 What `main.cpp` is waiting for

`src/main.cpp:41-51` names four obligations in its own header, and they are R1a's:
adapters for clock/motors/imu/gps/battery/rotation; the tick-boundary pacer replacing
`V5DelayPacer`'s fake-clock advance (`:98-115`); the on-robot precondition policy installed in
`initialize()` before any other task exists; and session-header emission once the PROS Makefile
injects the git build hash the way `test/CMakeLists.txt:36-50` already does.

---

## 3. Measured before briefed

Nine executable measurements, run against the tree at `1f7e281`. Four of them overturned the design
this brief would otherwise have carried. Commands are reproducible; the probe sources are disposable.

### M1 — The PROS SDK headers **fail the ARM compile gate's flags**

```sh
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c probe.cpp -o /dev/null -Iinclude
```
with `probe.cpp` including nine `<pros/*.hpp>` headers → **EXIT 1**, three distinct errors in code we
do not own:

- `include/pros/rtos.hpp:1903` — `-Werror=shadow`, twice (`MutexVarLock`'s ctor params shadow its members)
- `include/pros/motors.hpp:77` — `-Werror=sign-conversion` (`uint8_t` → `int8_t` in a delegating ctor)
- `include/pros/rotation.hpp:58` — the same, in `Rotation`'s

**This is the finding that shapes the chunk.** The ARM gate (`.github/workflows/ci.yml`) globs *every*
header under `include/shulib/` into one TU. Put a PROS-including adapter there and the gate breaks —
and breaks on third-party source, so no amount of care in shulib's own code fixes it. `build-order.md`'s
DoD says "the ARM build compiles them"; as written, **it cannot**.

### M2 / M3 — A two-flag diagnostic fence is sufficient, and `-Wconversion` stays live

Wrapping only the `#include <pros/...>` block in
`#pragma GCC diagnostic push` + `ignored "-Wshadow"` + `ignored "-Wsign-conversion"` + `pop` →
**EXIT 0**, at the gate's exact flags, covering `adi/distance/gps/imu/misc/motors/optical/rotation/rtos/screen/ai_vision`.

`-Wconversion` did **not** need suppressing. The fence is two flags, not three, and not `-Wall`.

### M5 — With the fence, the gate needs **no exclusion at all**

All 124 existing headers **plus** a realistic PROS-including adapter, as one TU, at the gate's exact
flags: **EXIT 0, 125 headers.**

This is the good outcome. D3's most-quoted lesson is *"a gate's exclusion list is where its holes
live"*; here the ARM gate keeps its generated glob, gains the adapters automatically the moment they
exist, and grows no exclusion list to hide in.

### M6 — Negative control: the fence does **not** protect shulib's own code

Planted a `-Wconversion` defect (`unsigned 200u` → `int8_t`) in the adapter body, *after* the
`#pragma GCC diagnostic pop`:

```
error: conversion from 'unsigned int' to 'int8_t' may change value [-Werror=conversion]   EXIT 1
```

Removed it → EXIT 0. **The fence covers the PROS include block and nothing else.** Without this
control, "we added a pragma and it compiled" would be indistinguishable from "we turned the warnings
off for our own adapters," which is the failure that would actually matter.

### M7 — An adapter compiles, links and **runs on the host** against a shim

A hand-written `pros/motors.hpp` shim, `-I shim` ahead of `-I include`, host `g++` at the test build's
full strict flags (`-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror`): the adapter
**compiled, linked, and executed**, and its ±12 V clamp was asserted from a host test binary.

**The adapters are host-testable.** The tree currently assumes they are not — that assumption is why
`build-order.md`'s R1 DoD stops at "it compiles." See §5, T2.

### M8 / M9 — The amended PROS-free guard, and the hole in the cheap version of it

Planted one legal include (`include/shulib/hal/pros/__probe.hpp`) and one illegal one
(`include/shulib/localization/__probe_violation.hpp`). Both candidate amendments caught the violation
and both went green when it was removed. Then planted `include/shulib/localization/pros/__smuggled.hpp`:

| Amendment | Result |
|---|---|
| `grep -rnE --exclude-dir=pros …` | ***MISSED*** — any directory named `pros`, anywhere under `include/shulib/`, is exempt |
| `grep -rnE … \| grep -v '^include/shulib/hal/pros/'` | **CAUGHT** |

The name-based exclusion is a real hole, demonstrated rather than argued. **Use the path-anchored
form.**

### M-extra — the build was RED at HEAD (fixed before this brief, commit `927a6fd`)

`briefing_status.py check` was failing, and the build did not know. Two defects: the generated block
carried HEAD's SHA, the commits-ahead counts and the last-ten commit list — all functions of the
commit being made, so no commit could ever satisfy it; and neither `PROJECT-BRIEFING.md` nor
`briefing_status.py` was in the doc gates' `DEPENDS`, so the check re-ran only when an unrelated
chapter changed. Fixed in the layer that owns it (Rule 4), verified green **after** committing —
the fixed point that provably did not exist before.

---

## 4. The PROS surface, as read from the vendored headers

Every row is a **belief about PROS**, and every belief gets an `HA-nn` entry. Next free is **HA-94**.

| shulib wants | PROS gives | The conversion R1a owns |
|---|---|---|
| `setVoltage(Voltage)` volts | `move_voltage(int32)` **millivolts**, ±12000 | ×1000, round, clamp |
| `IMotor::position()` rad, cumulative | `get_position()` double, **"encoder units"** | see the trap below |
| `IMotor::velocity()` rad/s | `get_actual_velocity()` double, **RPM** | ×2π/60 |
| `IMotor::current()` A | `get_current_draw()` int32, **mA** | ÷1000 |
| `IMotor::temperature()` °C | `get_temperature()` double °C | identity |
| `IRotation::position()` rad cumulative | `get_position()` int32, **centidegrees** | ×π/18000 |
| `IRotation::velocity()` rad/s | `get_velocity()` int32, **centideg/s** | ×π/18000 |
| `IImu::heading()` canonical | `get_rotation()` double, **cumulative CW degrees** | `imuHeadingToCanonical()` |
| `IImu::isReady()` | `is_calibrating()` bool | negate |
| `IGps` pose | `get_position_and_orientation()` → `gps_status_s_t` (metres, CW-from-North degrees) | `gpsToRobotPose()` |
| `IGps::rmsError()` inches | `get_error()` double, **metres** | `gpsRmsErrorToCanonical()` |
| `IBattery::voltage()` V | `battery_get_voltage()` int32 | ÷1000 — **see HA note** |
| `IBattery::current()` A | `battery_get_current()` int32 | ÷1000 — **see HA note** |
| `IBattery::capacity()` [0,1] | `battery_get_capacity()` double, **percent** | ÷100 |
| `IClock::now()` seconds | `millis()` uint32 ms / `micros()` µs | see T6 |
| controller axis [-1,1] | `get_analog()` int32, **−127..127** | ÷127 |

**Sentinels:** `PROS_ERR = INT32_MAX` (`include/pros/error.h:32`), `PROS_ERR_F = INFINITY` (`:36`).

### 4.1 Four traps found by reading the vendored headers

**Trap A — the motor's encoder units are not knowable from the code.**
`pros::Motor`'s constructor defaults are `MotorGears::invalid` and `MotorUnits::invalid`
(`include/pros/motors.hpp:74-75`), which mean *leave the device as it is*. `get_position()` returns
"the absolute position of the motor **in its encoder units**" — degrees, rotations, or raw counts,
whichever the motor's firmware was last told, **possibly by a different program on a different day**.
A motor left in `rotations` returns 1/360 of what the adapter believes. Odometry is then silently
wrong by 360× and nothing crashes.

The adapter **must** set encoder units and gearing explicitly at construction **and read them back**.
Both are also gearset-dependent for velocity, so gearing is not optional either.

**Trap B — the distance sensor's "no object" is an in-band magic number.**
`get_distance()` "**will return 9999 if the sensor can not detect an object**"
(`include/pros/distance.hpp:98-99`). Not `PROS_ERR`. Not a sentinel. A plain integer that converts to
393.66 inches. And `get_confidence()` is "**only available when distance is > 200mm**"
(`:133-135`) — so confidence does not reliably say "no object" either. An adapter that passes 9999
through gives the dock-confirm logic a wall 33 feet away and calls it a reading. *(R1b's, recorded
here because it was found here.)*

**Trap C — `get_digital_new_press()` consumes the event.**
PROS's edge detection is stateful on the controller object and *clears* the press when read. Two
consumers, one loses — and the loser is silent. T1's spec already says shulib does its own edge
detection; this is the reason it must, and the adapter must therefore bind `get_digital()` and
**never** `get_digital_new_press()`. The same applies to `adi::DigitalIn::get_new_press()` in R1b.

**Trap D — the battery's units are not documented in the header at all.**
`battery_get_voltage()` and `battery_get_current()` return `int32_t` and their doc comments say
only "the current voltage of the battery" / "the current current of the battery"
(`include/pros/misc.h:724,742`). No unit. The mV/mA belief comes from PROS's website, not from the
vendored source. That makes it a *weaker* assumption than the others in this table, and it must be
labelled as such and measured on the bench — a battery voltage off by 1000× silently destroys
brownout compensation, which scales every motor command.

---

## 5. The tensions to rule

Each ruling names what it rejects. Four are already decided by the team lead and are recorded as
rulings, not questions.

### T1 — Where the adapters live, and what each guard becomes

**RULED: `include/shulib/hal/pros/*.hpp`, header-only, like the rest of the tree.**

- **PROS-free guard: amended, path-anchored.**
  ```sh
  grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib \
    | grep -v '^include/shulib/hal/pros/'
  ```
  Exempt exactly one path. **Not** `--exclude-dir=pros` — M9 proved that misses
  `include/shulib/localization/pros/anything.hpp`.
- **ARM compile gate: NOT amended.** M5 proved it does not need to be. Its generated glob picks the
  adapters up automatically and they compile clean behind the M3 fence.
- **The fence is two flags.** `#pragma GCC diagnostic push` / `ignored "-Wshadow"` /
  `ignored "-Wsign-conversion"` / the PROS includes / `pop`. Nothing else, and the `pop` comes
  immediately after the includes — M6 is the control that proves it.

**Rejected:** putting adapters outside `include/shulib/` (in `src/` or a sibling tree) so neither
guard changes. It buys nothing — M5 shows the ARM gate is fine — and it costs the header-only shape,
splits the HAL across two trees, and makes `hal/pros` the only part of the library a consumer cannot
get by adding one include directory.

**Rejected:** excluding `hal/pros` from the ARM gate. That is the exclusion list D3 warned about, and
M5 makes it unnecessary.

### T2 — How adapter glue gets tested at all

**RULED: both halves — pure conversions *and* a host `pros/` shim.**

This is the chunk's most important structural decision, so the reasoning is written out.

The adapters are the only code in this tree that 1,522,327 assertions cannot reach. Two available
answers, and taking either one alone reproduces a hole this project has already been bitten by twice:

- **Pure conversions only** (the tree's existing idiom — `imu_conversion.hpp`, `gps_conversion.hpp`).
  Extends cleanly to `motor_conversion.hpp`, `rotation_conversion.hpp`, `controller_conversion.hpp`.
  But it tests the conversion in isolation and says nothing about whether the adapter **calls** it.
  That is **C5's D-5 hole verbatim** ("the D-5 pipeline wiring was invisible to all 915k assertions")
  and **E1's** ("the suite tested that the FORMAT could carry the sink's self-description, never that
  the SINK filled it in"). Two chunks, same shape, both found by mutation.
- **Shim only.** Tests the wiring, but puts the unit arithmetic inside a class that only ever runs
  against our own model of PROS — trap #1, the shared model that cancels its own errors.

So: conversions are pure, host-tested and mutation-tested; the shim proves the glue calls them.

**The honest limit, and it must be written in the shim's own header, not buried here:** the shim
proves the adapter is consistent with **our belief about PROS**. It cannot prove the belief. If the
shim says `get_position()` returns degrees and the adapter converts degrees→radians, the test passes —
and would pass identically if PROS actually returned rotations. **The shim tests the adapter;
hardware tests the shim.** Every belief in it becomes an `HA-nn` entry, and R1a's bench session is
where the load-bearing ones get their first reality check.

**Rejected:** transcribing every shim semantic with the `include/pros/*.hpp` file:line it came from,
so the belief is greppable rather than invented. Genuinely better, and declined for authoring cost;
the `HA-nn` entries carry the same obligation at lower resolution. Recorded so the cheaper choice is
visible as a choice.

### T3 — The shim must be structurally impossible to ship on a robot

**RULED: the shim headers `#error` unless `SHULIB_HOST_PROS_SHIM` is defined**, and only
`test/CMakeLists.txt` defines it.

The failure being prevented: `test/pros_shim/` reaching the PROS Makefile's include path. The robot
binary would then build against in-memory fake motors, upload cleanly, boot cleanly, print a healthy
banner, and **drive nothing**, with every test green. That is a silent catastrophic failure on a
field, and it is exactly the class C5 designed against when it made a missing build hash *loud rather
than plausible*.

Shim placement: `test/pros_shim/pros/*.hpp`, added with `target_include_directories(... BEFORE ...)`
so it shadows the real `include/pros/` in the host build only.

**Rejected:** naming the shim something other than `pros/` and having adapters include an indirection
header. It removes the shadowing hazard but adds a layer between the adapter and the SDK it adapts,
and that layer would itself be untested glue — the very thing T2 exists to eliminate.

### T4 — IMU yaw rate

**RULED (team lead): ship both paths behind a config flag; default to differentiating `get_rotation()`.**

`imu_conversion.hpp:54-60` states the problem: PROS exposes no CW yaw-rate scalar, and
`get_gyro_rate()` returns a raw body-axis struct **whose z sign is undocumented** (HA-04). Its advice —
differentiate `get_rotation()`, which *is* documented CW-positive — was written when there was no
hardware.

Consequences R1a must carry, because a flag is not free:
- Differentiation makes the adapter **stateful** (previous reading + timestamp) and therefore requires
  an `IClock`. `yawRate()` is `const`; the state must be `mutable` with that reasoned in the header.
- Differentiating a quantized angle **amplifies quantization noise** — a real cost against the `< 1°`
  budget, which is heading-owned.
- **Both branches must be tested.** An untaken branch on a robot is where bugs hide, and this is
  precisely why the shim earns its keep: with it, the gyro path is testable without a robot.
- **The bench session must measure `get_gyro_rate().z`'s sign** and register it with tank-bot
  provenance, so the second branch is not dead-and-unvalidated. One measurement on one sensor is not
  proof of portability across IMUs — label it, do not assume it.

**Rejected:** differentiation only (loses a real hardware rate; the header's own advice, written
without hardware). **Rejected:** gyro only (one bench measurement is thin evidence for the default
path of the load-bearing heading quantity).

### T5 — `IController` is an F4-additive sibling, not an F4 amendment

**RULED: new seam `hal/controller.hpp`, explicitly outside the F4 freeze**, the same shape F1 used for
`IDigitalOut` (`digital_out.hpp:22-24`).

Required by T1's spec and non-negotiable:
- **Axes normalized to `[-1, 1]` and buttons as `bool` at the adapter edge.** PROS's `−127..127` never
  reaches the core — the same "convert exactly once, at the edge" rule as IMU and GPS.
- **New-press edge detection above the seam**, from `get_digital()`. Never `get_digital_new_press()` —
  trap C.
- **`isConnected()` as a positive validity signal** (matching `IImu::isReady()` / `IGps::hasFix()`
  polarity, `imu.hpp:31-33`). A controller really does drop mid-match, and the core must be able to
  see the difference between that and "sticks centred."
- **Partner controller from the start.** VEX U runs two drivers. Retrofitting a second through a
  single-controller seam is the reshape a seam exists to prevent.

**Nothing here freezes.** Register row for the new seams says so out loud — D2's lesson that silence
in the register reads as "frozen." The freeze trigger is a second real consumer, which is T2/T3.

### T6 — `IClock`: `millis()` or `micros()`

**Recommendation: `micros()`.** `millis()` quantizes to 1 ms, which is **10% of the 10 ms control
tick**. PID derivative terms and profile timing both divide by `dt`; a 10%-quantized `dt` is a 10%
error in every derivative term, and the host plant has never seen it because `FakeClock` is exact.
`micros()` costs nothing and removes the whole question.

Whichever is chosen, the `IClock` contract is **monotonic non-decreasing** (`clock.hpp:12`). Note that
PROS's counters are unsigned and wrap; at ~49.7 days (`millis`) this is irrelevant to a match and
must still be stated rather than silently assumed.

**Rejected:** `millis()` on the grounds that the tick is 10 ms so millisecond resolution is "enough" —
that conflates the tick period with the measurement of it.

### T7 — What an adapter does with a sentinel on a seam that has no validity channel

`IMotor`, `IRotation` and `IBattery` have **no** `hasFix()`/`isReady()`. `IGps` and `IImu` do. So
`PROS_ERR`/`PROS_ERR_F` from an encoder has nowhere to go in the interface.

The core already assumes this is solved at the edge, in writing, in three places:
`tracking_wheel.hpp:27-28` ("TrackingWheel does no sentinel screening — it trusts the HAL finiteness
contract"), `sim/degradation.hpp:27-29`, `sim/hostile/encoder_hostility.hpp:16,23-25` (whose sentinel
injection **deliberately breaches** the F4 contract to model *a buggy adapter*). A3 built the hostile
case for the failure R1a must not commit.

**Proposed ruling — confirm or overturn while writing the code:** screen at the edge, hold the last
good value, and **raise a fault** so the run can see it. Never propagate a sentinel (breaks the
finiteness contract A3 models as a bug), never silently substitute zero (a zeroed encoder reads as "the
robot stopped", which is exactly the dead-encoder runaway C2's `ODO_STUCK` abort exists to catch —
and substituting zero would make it *undetectable* by making the reading plausible).

**Rejected:** widening the F4 interfaces with validity signals. F4 is **LOCKED**; that is a major
version bump plus a migration note, and the problem does not need it.

### T8 — The on-robot precondition policy

`check.hpp:26` — a handler **must not return**; it throws. The host default turns breaches into red
tests (`:50-52`). `main.cpp:46-48` says the robot policy is "raise the fault code on the latch, then
throw."

The open question R1a must answer with evidence, not assertion: **what happens when that throw
crosses a PROS task boundary?** C2 established that `check.hpp`'s task-boundary catch is real, but
that was reasoned on the host. On the brain an uncaught exception out of a PROS task is not a
controlled failure. Install the policy in `initialize()` **before any other task exists**
(`main.cpp:46-48`) and demonstrate the behaviour on the bench rather than describing it.

---

## 6. Test requirements

The bar is the project's, unchanged: **every test names, in a comment, the bug it would catch** — not
what it checks.

### 6.1 New host tests

- **Conversion tests** for each new pure conversion (`motor_`, `rotation_`, `controller_`), with
  **hand-computed literals** and **no import of the constant under test**. E2's HA-07 finding is the
  reason this sentence exists: the pre-existing test did `using shulib::hal::kMetersToInches;` and then
  asserted against `kMetersToInches`, so a wrong constant satisfied both sides of the `==`.
- **Adapter tests against the shim**, one per adapter, covering at minimum: the conversion is actually
  called; sentinel screening (T7); the ±12 V clamp and the **non-finite precondition** (L4); brake-mode
  mapping both directions; encoder-units/gearing set **and read back** (trap A); the GPS boot-check
  that `get_offset()` is `(0,0)` (HA-06); controller axis normalization and edge detection (trap C);
  `ILineDisplay` truncation at 19 columns (never wrap).
- **Both IMU yaw-rate branches** (T4).
- **A guard test** pinning the diagnostic fence's suppression set to exactly `-Wshadow` and
  `-Wsign-conversion`. Without it the list grows silently, and a fence that ends up ignoring `-Wall` is
  a hole with a pragma in front of it.

### 6.2 Required mutations — the campaign must include these, and each must be **executed**

*A mutation you reasoned about but did not run does not count. Gate the runner on build success — C4
nearly read a non-compiling mutation as green off a stale binary. Trap `PIPE`; never `git checkout` a
file holding uncommitted work (E2 lost a header line that way, and it was caught by the mutation
count dropping, not by the report).*

| # | Mutation | Must go |
|---|---|---|
| M1 | Delete the mV scale in the motor adapter (volts sent as millivolts → 1/1000 of commanded torque) | RED |
| M2 | Delete the ÷1000 in `current()` (mA read as A → stall detection never fires) | RED |
| M3 | Delete the centidegree scale in the rotation adapter | RED |
| M4 | Swap `get_rotation()` for `get_heading()` in the IMU adapter (HA-03 — loses revolution continuity) | RED |
| M5 | Remove `set_encoder_units`/read-back (trap A) | RED |
| M6 | Remove the GPS `get_offset()==(0,0)` boot-check (HA-06 double-subtraction) | RED |
| M7 | Drop `gpsRmsErrorToCanonical()`, pass metres straight through (HA-07, the 39× that E2 built a function to prevent) | RED |
| M8 | Remove sentinel screening on one reader (T7) | RED |
| M9 | Replace the adapter's non-finite precondition with a silent coerce-to-zero (L4) | RED |
| M10 | Bind the controller to `get_digital_new_press()` instead of edge-detecting (trap C) — **needs two consumers to be visible** | RED |
| M11 | Move the `#pragma GCC diagnostic pop` to the end of the file, so the fence covers adapter code | RED (the fence-scope test) |
| M12 | Remove the `#error` from a shim header (T3) | RED |
| M13 | Widen the amended PROS-free guard to `--exclude-dir=pros` and plant `localization/pros/x.hpp` (M9) | RED |
| M14 | Adapter reads the conversion but discards it, returning the raw value (the C5 D-5 / E1 wiring hole) | RED |

**A mutation that stays GREEN is a hole in the suite and the most valuable thing this chunk can find.**
Record every green with its measurement rather than inventing a test to paper over it — E3's recorded
GREEN it could not honestly close is the standard.

### 6.3 The bench smoke session — and what it may **not** claim

Recorded, labelled, and **not** called validation. R3 owns validation.

Do: read every adapter's sensor and print it; spin one motor open-loop and confirm the encoder moves
in the commanded direction; confirm the IMU heading changes sign correctly under a known rotation
(`imu_conversion.hpp:28-31` — a +90° CW spin must **decrease** canonical heading by 90°); measure
`get_gyro_rate().z`'s sign (T4); confirm `ILineDisplay`'s 3×19 geometry against real firmware (HA-57,
unverified since C5); confirm the battery unit scale (trap D); read the device list off the brain
rather than assuming what is on the practice bot.

Do **not**: tune a gain, claim an accuracy number, or record anything as "measured" without
**tank-bot provenance**. kS/kV/kA and PID gains are mass-, friction- and geometry-dependent; a
measured-looking wrong number is worse than an honest placeholder.

**The tank bot validates the platform layer and the tank kinematics path. It validates none of the
holonomic thesis** — no strafe authority, no pseudo-inverse, no per-axis decoupling, no H-drive
geometry. Register entries for those stay open, and must **say** they stay open rather than implying
coverage.

---

## 7. Definition of Done

- [ ] Nine PROS-backed adapters exist under `include/shulib/hal/pros/`, plus the tick pacer
- [ ] `hal/controller.hpp` + `hal/fake/fake_controller.hpp` exist; register row records **not frozen**
- [ ] New pure conversions exist, PROS-free, host-tested with hand-computed literals
- [ ] The host `pros/` shim exists, `#error`s outside the host test build (T3), and its header states
      the "tests the adapter, not the belief" limit in its own words
- [ ] PROS-free guard amended **path-anchored**, and proven still to catch a violation elsewhere
- [ ] ARM compile gate **unamended** and green with the adapters in the glob
- [ ] Full host suite green; every mutation in §6.2 executed, each result recorded RED or GREEN
- [ ] `src/main.cpp`: all fourteen `TODO(R1)` markers resolved or explicitly re-homed to R1b/R3 with
      the reason; `V5DelayPacer` replaced; precondition policy installed; build hash injected by the
      PROS Makefile so the session header is real
- [ ] `make` produces an uploadable package; it uploads and boots
- [ ] Bench smoke session recorded with tank-bot provenance, claiming nothing beyond what it measured
- [ ] `HA-94…` registered for every new belief, all labelled invented/reasoned/measured-on-tank-bot
- [ ] `build-order.md` amended (§0), roadmap checkboxes flipped with cited evidence, `[~]` where partial
- [ ] Documentation contract (§8) discharged
- [ ] **The governing constraint survives:** nothing in this chunk lets "it booted" or "an adapter read
      a sensor" drift into "it works on a robot." That distinction is defended in six places across the
      README and the guide — R1a is the single most likely chunk in the project to erode it.

---

## 8. Documentation scope

Named explicitly, the same way test scope is.

**The standing rule, stated once because R1a is the chunk most likely to break it: EVERY change gets
documented, not only the interesting ones.** Fifteen adapters is fifteen new surfaces, and a chunk
that adds that much and documents only its headline is how a library becomes a thing only its author
can use. "It's just glue" is not an exemption — glue is where the units live.

**Public** (`docs/`, publishes to docs.shurobotics.com):

- **`docs/changelog.md` — NEW, in the mkdocs nav. This is the gap R1a closes.**
  The library has **no changelog**. `version.hpp` moved 2.0 → 2.1 at F1 and the reason is recorded in
  a **code comment** (`version.hpp:52-55`) — so an outside team on 2.0 has no document that tells them
  what changed, whether it affects them, or whether it was additive. `version.hpp` already carries a
  written breaking-vs-additive policy and both freeze rows are pinned in the build; the changelog is
  the reader-facing half that was never written.

  Format: newest first, one section per API version, each entry saying **what changed, whether it is
  breaking or additive, and what a user must do about it** — nothing else. R1a's own entry covers the
  new seams (`IController`), the new adapter tree, and the guard amendment.
  **From R1a on, no chunk closes without its changelog entry.** Add that line to the per-chunk
  documentation contract in `build-order.md` — it is deliverable #7.

- **`docs/faq.md` — NEW, in the mkdocs nav.** The FAQ is for **nuance the reference cannot carry**:
  "what happens when a sensor returns a PROS error mid-run?", "why does `hasFix()` go false in Driving
  Skills?", "why does the library read `get_rotation()` and not `get_heading()`?", "what does 'the shim
  tests the adapter, not the belief' mean if I am porting shulib to non-PROS hardware?"
  It is **not** the changelog and **not** the decision log — it answers *how does this actually
  behave*, which is the question a user has at 11pm before a competition.

- **The adapter surface itself must be documented where a user will look.** Fifteen adapters that
  exist only as headers are fifteen undiscoverable features. At minimum: guide **ch. 07 "Getting set
  up"** gains the real wiring (it currently describes a HAL with no implementation) and **ch. 13
  "Extending the library"** gains the porting story, which is now real rather than hypothetical.
- **Guide ch. 14 "What it cannot do yet"** — the honesty ledger. R1a changes what belongs in it more
  than any chunk since C7. Edit it with more care than any other file here.
- **`docs/hardware-assumptions.md`** — HA-94 onward, one entry per belief about PROS, each with the
  bench measurement that settles it. This is R3's runbook and it grows here.
- **`docs/roadmap.md`** — checkboxes with cited evidence, "you are here", register row for the new
  seams saying **not frozen** out loud.

  **Gate constraints, both of which fail the build:** every ` ```cpp ` line in a public doc must
  appear **verbatim** in a compiled `test/*example*_test.cpp` (`check-examples`) — prose entries are
  free, code samples cost a test file. And no public doc may contain the strings `internal/`,
  `chunks/`, `RESUMING` or `build-order` (`check-removability`).
- **Guide ch. 07 "Getting set up"** and **ch. 13 "Extending the library"** — the HAL chapter is where a
  team porting to different hardware looks, and it currently describes a HAL with no real
  implementation.
- **Guide ch. 14 "What it cannot do yet"** — this chapter is the honesty ledger and R1a changes what
  belongs in it. Edit it with more care than any other file in the chunk.
- **`docs/hardware-assumptions.md`** — HA-94 onward, and R3's runbook grows.
- **`docs/roadmap.md`** — checkboxes with cited evidence, "you are here", register row for the new seams.

**Internal** (`docs/internal/`, dropped at the squash to `main`):
- `R1a-PROGRESS.md`, appended in real time — **created first**, before any code.
- `R1a-COMPLETED.md`, written from the log, not instead of it.
- `build-order.md` amendments per §0.

**Header design notes** are a deliverable, not a courtesy: every adapter header explains **why** —
which PROS call it binds and why that one, which conversion it applies, which `HA-nn` its beliefs are
registered under, and what it deliberately does not do.

---

## 9. Landmines

**L1 — The PROS-free guard is the library's defining property.** "The core depends only on the
PROS-free HAL" is why the same code runs against fakes, the plant, and (at H2) VexBuilder's simulator
"as a structural guarantee, not a feature." R1a amends that guard for the first time. Amend it
deliberately, anchor it to one path, and prove it still bites.

**L2 — `FaultLatch`, `HealthMonitor` and the fault codes already exist.** Do not mint a new
diagnostic vocabulary for adapters. If a sentinel needs to be visible, it goes through the existing
latch.

**L3 — Frozen means frozen.** F4 (the ten HAL interfaces) and F6/F10 are LOCKED. If a signature pin
fires, **stop and report** — that is a breaking change to argue, not an edit to make. And the doc
**freshness** gate fires *before* the pins, naming the wrong problem; D3 documented this and it bit
again at F1's verification. Read the second error, not the first.

**L4 — Do not paper over the non-finite voltage contract.** `IMotor`'s header says a non-finite
voltage is **rejected** (`motor.hpp:8-11`) and `FakeMotor` raises `SHULIB_PRECONDITION`
(`fake_motor.hpp:18-20`). The prototype adapter written for M5 silently coerced non-finite to `0.0` —
which compiles, passes a clamp test, and quietly converts a programming error into a robot that
coasts. It is named here because the mistake was made while measuring for this brief, in a file whose
whole purpose was to be careful.

**L5 — The verify harness at `docs/internal/verify/verify-r1a.sh` belongs to the REVIEWER.** F1's
agent overwrote the reviewer's harness with its own self-check and destroyed the independence of the
audit. Do not create, edit, or overwrite it.

**L6 — Do not commit and do not push.** Verification is a separate step by a separate reader.

**L7 — Assertion counts flatter.** 1.5M measures seeds swept. Mutation results are the measure. A
chunk that reports only wins has not looked hard enough.

**L8 — Chunk/register name collision.** Freeze rows F1–F5 are **not** chunks F1–F5. Row F4 is the HAL
interface signatures — the thing this chunk must not break. **Never edit rows F1–F5.**

---

## 10. What R1a hands to R1b and R3

**To R1b:** the fence pattern, the shim framework, the amended guard, and the adapter idiom — R1b
should be mechanical repetition, not new structure. Plus trap B (the distance sensor's 9999), found
here, owned there.

**To R3:** every `HA-nn` this chunk registers, each with the bench measurement that settles it. R3's
promise is that first contact is a checklist rather than an exploration; R1a's job is to make sure the
checklist is complete and honest, including the entries that say *the tank bot cannot settle this*.
