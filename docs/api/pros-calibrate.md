<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/calibrate.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `calibrate.hpp`

hal/pros/calibrate — the standard "shulib Calibrate" on a V5 brain (chunk CAL1, Part A2a, 2026-10-18): the THIN shell that hands calibration::CalibrateProgram the robot's real devices.

This header declares **1** type (4 members) and **6** free functions.

Extracted from [`include/shulib/hal/pros/calibrate.hpp`](../../include/shulib/hal/pros/calibrate.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsCalibrateIo`](#class-proscalibrateio)
  - [`line`](#proscalibrateio-line)
  - [`competitionStatus`](#proscalibrateio-competitionstatus)
  - [`delayMs`](#proscalibrateio-delayms)
  - [`readShaftDegrees`](#proscalibrateio-readshaftdegrees)
- [`calibrateInitialize`](#calibrateinitialize) — *free function*
- [`calibrateStateScreen`](#calibratestatescreen) — *free function*
- [`calibrateDisabled`](#calibratedisabled) — *free function*
- [`calibrateConnected`](#calibrateconnected) — *free function*
- [`calibrateAutonomous`](#calibrateautonomous) — *free function*
- [`runCalibrate`](#runcalibrate) — *free function*

<a id="class-proscalibrateio"></a>

## `class ProsCalibrateIo`

```cpp
class ProsCalibrateIo final : public calibration::CalibrateIo
```

calibration::CalibrateIo on the brain (header): the serial line, the competition status, a plain wait, and the watch's read through PROS's C API with no motor object.

*class, declared at [`include/shulib/hal/pros/calibrate.hpp:79`](../../include/shulib/hal/pros/calibrate.hpp#L79).*

<a id="proscalibrateio-line"></a>

### `ProsCalibrateIo::line`

```cpp
void line(const char* text) override
```

"[R3D] <text>" on the USB serial port: ONE write() per line, through serialWrapped() (SER1) — a text past one serial line goes out in "[R3D] ... " pieces.

*function, declared at [`include/shulib/hal/pros/calibrate.hpp:83`](../../include/shulib/hal/pros/calibrate.hpp#L83).*

<a id="proscalibrateio-competitionstatus"></a>

### `ProsCalibrateIo::competitionStatus`

```cpp
[[nodiscard]] std::uint8_t competitionStatus() override
```

pros::c::competition_get_status().

*function, declared at [`include/shulib/hal/pros/calibrate.hpp:85`](../../include/shulib/hal/pros/calibrate.hpp#L85).*

<a id="proscalibrateio-delayms"></a>

### `ProsCalibrateIo::delayMs`

```cpp
void delayMs(std::uint32_t ms) override
```

pros::delay(ms).

*function, declared at [`include/shulib/hal/pros/calibrate.hpp:89`](../../include/shulib/hal/pros/calibrate.hpp#L89).*

<a id="proscalibrateio-readshaftdegrees"></a>

### `ProsCalibrateIo::readShaftDegrees`

```cpp
[[nodiscard]] bool readShaftDegrees(int port, double& degrees) override
```

THE WATCH'S READ (header): motor_get_position() on `port` in the units the motor reports, turned into degrees of output shaft — no motor object, nothing written. False when PROS answers PROS_ERR_F (not a motor, or not answering), or a unit or gearing it does not know.

*function, declared at [`include/shulib/hal/pros/calibrate.hpp:93`](../../include/shulib/hal/pros/calibrate.hpp#L93).*

<a id="calibrateinitialize"></a>

## `calibrateInitialize`

```cpp
inline void calibrateInitialize(const char* label = nullptr, const char* buildStamp = __DATE__ " " __TIME__)
```

initialize(): the boot banner (constructs nothing, powers nothing). `label` is a build's beacon, or null; `buildStamp` defaults to this translation unit's own (header: ONE unit).

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:121`](../../include/shulib/hal/pros/calibrate.hpp#L121).*

<a id="calibratestatescreen"></a>

## `calibrateStateScreen`

```cpp
inline void calibrateStateScreen(const char* state, const char* why)
```

disabled(), competition_initialize() and autonomous(): Calibrate's competition-state screen — `state` and `why` are calibrate_screen.hpp's words (checked at compile time to fit).

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:129`](../../include/shulib/hal/pros/calibrate.hpp#L129).*

<a id="calibratedisabled"></a>

## `calibrateDisabled`

```cpp
inline void calibrateDisabled()
```

disabled(): the state screen, DISABLED.

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:136`](../../include/shulib/hal/pros/calibrate.hpp#L136).*

<a id="calibrateconnected"></a>

## `calibrateConnected`

```cpp
inline void calibrateConnected()
```

competition_initialize(): the state screen, FIELD CONNECTED.

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:142`](../../include/shulib/hal/pros/calibrate.hpp#L142).*

<a id="calibrateautonomous"></a>

## `calibrateAutonomous`

```cpp
inline void calibrateAutonomous()
```

autonomous(): the state screen, AUTONOMOUS — no motion here, ever.

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:148`](../../include/shulib/hal/pros/calibrate.hpp#L148).*

<a id="runcalibrate"></a>

## `runCalibrate`

```cpp
inline void runCalibrate(const calibration::CalibrateFacts& facts, const char* buildStamp = __DATE__ " " __TIME__)
```

opcontrol(): the standard Calibrate on this brain, over the robot's real devices (header). Returns when the field disables the robot or starts autonomous (every motor at 0 V + Coast), or when no run can start. `buildStamp` defaults to this translation unit's own.

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:156`](../../include/shulib/hal/pros/calibrate.hpp#L156).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 46 lines, click to expand</summary>

```text

 hal/pros/calibrate — the standard "shulib Calibrate" on a V5 brain (chunk CAL1, Part A2a,
 2026-10-18): the THIN shell that hands calibration::CalibrateProgram the robot's real devices.
 Everything the program decides is in include/shulib/calibration/calibrate_program.hpp and runs on
 the host test plant; this file only composes the PROS adapters and answers CalibrateIo's four
 calls:
   * line()              hal::pros::serialWrapped("R3D", text) — ONE write() per line (SER1: never
                         stdout's shared, unguarded buffer), the "[R3D] " tag every worksheet and
                         every Calibrate capture since R3d has read; a line past one serial line's
                         256 bytes goes out in "[R3D] ... " pieces, one write() each;
   * competitionStatus() pros::c::competition_get_status();
   * delayMs()           pros::delay();
   * readShaftDegrees()  THE WATCH'S READ: three READS of PROS's C API by port —
                         pros::c::motor_get_position(), converted by the units
                         motor_get_encoder_units() reports (counts by the gearing
                         motor_get_gearing() reports: 1800 / 900 / 300 a turn, HA-15). NO motor
                         object of any kind is constructed — neither a pros::Motor nor a ProsMotor,
                         whose constructor WRITES the gearset (T3's rule for a claw: never geared,
                         never set to Coast, never pulsed). The vendored libpros.a's three reads
                         call only PROS's registry check, the port mutex, __errno and one firmware
                         *Get each (disassembled for the CAL1 brief, 2026-10-08) — the belief is
                         HA-187 in docs/hardware-assumptions.md: a read writes nothing to the motor.
                         False when PROS answers PROS_ERR_F, or a unit or gearing it does not know.

 A TEAM'S CALIBRATE is one file that states its facts and calls the five functions below from
 PROS's five entry points (21 lines — the guide's chapter 19, "Calibrating your robot", shows it whole).
 Include this header in ONE translation unit: its default build stamp is that unit's __DATE__ and
 __TIME__, so two units would carry two stamps for one inline function.

 The composition is runCalibrate()'s own frame: the registry, the provider over it (no log copy),
 the SD store (its default root), the screen and the IO — each borrowed by the program for its run.

 THE STACK it takes on PROS's opcontrol task (GCC's -fstack-usage -fcallgraph-info=su over robot one's
 Calibrate, src/programs/calibrate_program.cpp with the robot's flags, 2026-11-17, never on a robot):
 the deepest path GCC can see is 23,552 B — the caller 1,240, this runCalibrate 16,456 (the provider
 alone 14,696), CalibrateProgram::run 1,960, then the DONE path down to the safe save's CRC — BEFORE
 newlib's file and printf frames and VEXos's file calls, which it cannot see. PROS runs opcontrol()
 on a task of TASK_STACK_DEPTH_DEFAULT = 0x2000 words (32 KiB, include/pros/rtos.h:72). The
 calibrator and the run's record live in static storage (CAL2): on this stack they made the path
 31,824 B before CAL2 (a 7,712 B calibrator) and 32,072 B with CAL2's 7,928 B one; a measurement build
 ~140 B over the release stopped with FATAL ERROR on robot one's opcontrol task at the save
 (2026-11-17). The provider is next.

 NOTHING HERE HAS RUN ON A ROBOT in this form. Its host test (test/pros_calibrate_adapter_test.cpp)
 runs it through the host PROS shim, which tests the shell against our BELIEF about PROS, not the
 belief itself (shim_control.hpp says so in full).
```

</details>
