# R1a Bench Session — first physical contact between shulib and a V5

**Date:** 2026-08-13
**Robot:** the team's **old competition bot** — *not* the "old tank practice bot" the planning
documents describe. See §6; several documents are wrong about what hardware exists.
**Brain firmware:** V5 system `1.1.5-0`, CPU0 F/W `1.1.0-6`, CPU1 SDK `1.1.5-0`
**Toolchain:** PROS CLI 3.5.6, `arm-none-eabi-g++ 13.2.1`
**Transcripts:** captured to the session scratch directory; every number below was read off the
brain over USB serial, not computed.

> **Provenance rule, applied throughout:** every value here is **measured on ONE robot, once**.
> That is enough to replace a guess with an observation. It is *not* proof of portability across
> units, and no entry below is marked otherwise.
>
> **This session validated the PLATFORM LAYER only.** shulib commanded a physical motor for the
> first time and read physical sensors for the first time. **It did not drive a robot**: nothing
> closed a loop, nothing followed a path, no wheel turned under motion control. R3 owns that, and
> it is entirely open.

---

## 1. The headline — every unit conversion is correct

Eight motors, each commanded **+2.0 V for 1.5 s** through the real `ProsMotor` adapter, with the
adapter's canonical value printed beside the raw PROS value so the physical wheel checked shulib's
arithmetic rather than a test checking itself.

| Conversion | Correct | Measured (all 8 motors) | Register |
|---|---|---|---|
| motor degrees → radians | 57.2958 | **57.296** | HA-95 |
| motor RPM → rad/s | 9.5493 | **9.549** | HA-96 |
| motor mA → A | 1000 | **1000.0** | HA-97 |
| volts → millivolts | 2000 mV | device reported **1823–1990 mV** | HA-94 |
| battery raw → volts | — | **13039** raw → 13.04 V | HA-99 |
| battery capacity | — | **91.0** → 0.91 | HA-100 |
| `micros()` over a nominal 1000 ms | 1 000 000 | **999 784** (0.02% low) | HA-101 |

The voltage shortfall is **not** an error: `commandedVoltage()`'s contract is that it reports what
was *applied*, not what was *asked for*, and a motor under load on a sagging battery applies less.
The adapter faithfully reported reality.

**Seven register entries settled** in one session, every one previously a written-down guess.

## 2. The drivetrain, measured twice by independent means

**LEFT = 15, 16, 17, 18 · RIGHT = 11, 12, 14** — all green (18:1, 200 RPM) cartridges.

Established by spinning one side at a time **by hand** with a read-only monitor: during the left
spin, ports 15/16/17/18 moved and 11/12/14 read *exactly zero*; during the right spin, the reverse.
No inference, no ambiguity.

Full device inventory: motors on **1, 2, 3, 5, 11, 12, 13, 14, 15, 16, 17, 18**; **IMU on port 4**;
radio; ADI expander. Twelve motors, seven of them drive.

## 3. Two mechanical faults found — one proven twice

### 3.1 Port 13 is spinning free (confirmed)

| | Movement at 2 V | Current |
|---|---|---|
| 15, 16, 18 | 0.03–0.08 rad (stalled against load) | 0.70–0.95 A |
| 11, 12, 14 | 0.29–2.14 rad | 0.47–0.60 A |
| **13** | **3.93 rad — 4× the fastest** | **0.018 A** |

**Eighteen milliamps.** Every other motor draws 500–950 mA pushing against the drivetrain; 13 spins
faster than all of them while drawing 3% of their current. A motor doing that is connected to
nothing.

Confirmed independently by the hand-spin test, where 13 never moved while its neighbours turned.
**Two unrelated methods, same conclusion: the chain or gear on port 13 is off.** Mechanical repair.

### 3.2 Port 16 under-reports its side-mates (open)

Through a hand spin, m16 read **~80% of what 15, 17 and 18 read**, sample by sample rather than only
in total. Across two separate spins the ratio was ~85% and ~73% — *not* a constant, so it is not a
gearing difference. Most likely mechanical slip. A drive motor that under-reports travel biases
odometry in one direction, quietly. **Not diagnosed; recorded.**

## 4. A real SDK trap, found the hard way

**`registry_get_plugged_type()` is ZERO-indexed (ports 0–20). Every device API — `motor_*`, `imu_*`,
`gps_*` — is ONE-indexed (ports 1–21).** Both conventions in one SDK, documented only in each
function's own comment.

Mixing them produces **plausible wrong answers, never an error**: my inventory probe reported an
IMU one port below where it physically was, then read a motor API on the IMU's real port and got
`ENODEV`. That was reported as "you have a dead motor and a dead IMU." Both devices were fine. The
team lead said "IMU on 4, motor on 5" twice and was right both times.

**The shipped library never had this bug** — grep confirms zero registry calls in any adapter or in
`src/main.cpp`; the adapters are 1-indexed throughout. It was confined to a throwaway probe. It is
recorded because anyone writing a PROS adapter will meet it, and it belongs in the public FAQ.

## 5. What was NOT done, and why

| Runbook step | Status | Reason |
|---|---|---|
| 1 — boot/session header, T8 throw behaviour | **not done** | never booted the shipped `main.cpp`; its invented port map does not match this robot |
| 6 — rotation sensors | **impossible here** | this robot has **no rotation sensors** |
| 7 — IMU heading sign | **not done** | IMU confirmed alive on port 4, but no rotation-against-a-protractor was performed |
| 8 — gyro z sign (T4's mandated measurement) | **not done** | the flagged-undocumented quantity is still undocumented and still guessed |
| 9 — pitch/roll signs | **not done** | — |
| 10 — controller | **blocked** | no controller was paired at any point (`master=0 partner=0`) |
| 11 — LCD 15-vs-19 columns | **not done** | needs the controller |
| 12 — loop cadence | **not done** | needs the shipped stack running |
| 13 — GPS | **impossible here** | this robot has **no GPS** |
| 14 — unplug matrix | **partial, accidental** | a real live disconnect was observed (§7) but not systematically |

**Every heading-related assumption remains open**, which matters more than the count suggests:
heading is IMU-owned and the `< 1.0°` target lives or dies there.

**The holonomic thesis remains entirely unvalidated** — no strafe authority, no pseudo-inverse, no
per-axis decoupling, no H-drive geometry. This is a tank robot; it cannot speak to any of it. Those
register entries stay open and this record says so rather than implying coverage.

## 6. Documents that are wrong about this hardware

- The project briefing describes the available robot as *"an old tank/differential practice bot"*
  with *"I'm told there isn't much on it."* It is the **old competition bot**, with twelve motors, an
  IMU, and an ADI expander.
- It also lists what the tank bot can validate, including **GPS noise/latency, the GPS field-cal axis
  oracle, encoder refresh via tracking wheels, and the odometry push test.** This robot has **no GPS
  and no rotation sensors**, so none of those are settleable on it.

Both corrected alongside this record.

## 7. Process failure worth keeping

Mid-session the **USB cable dropped**. Programs continued running on the brain, uploads reported
success, and serial went silent. I spent a long stretch bisecting shulib, hot/cold linking and the
build system — concluding at one point that linking shulib broke the binary, which a control test
then disproved.

The fact that resolved it was **what the brain screen said** (a black screen with the program name =
running fine). I asked for that far too late.

**The rule:** when a program uploads successfully and produces no output, check that the device is
still enumerated (`ls /dev/ttyACM*`) *before* suspecting the code. Every theory formed during that
stretch was wrong, and one of them — "linking shulib breaks the binary" — would have been a serious
false finding had the control not been run.

*(A smaller one: `cp -a` preserves mtime, so restoring a mutated file leaves make believing it is
up to date and the next run reads a stale binary. Bumping mtime on restore is now required.)*

## 8. Bottom line

The platform layer works on real hardware: shulib's adapters read physical V5 devices and command a
physical motor, and **every unit conversion they perform is correct against a real wheel.**

The library still **has never driven a robot.** Nothing here changes that sentence, and it should
not be edited anywhere in the tree on the strength of this session.
