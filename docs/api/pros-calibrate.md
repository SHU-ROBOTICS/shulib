<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/calibrate.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `calibrate.hpp`

hal/pros/calibrate — the standard "shulib Calibrate" on a V5 brain (chunk CAL1, Part A2a, 2026-10-18): the THIN shell that hands calibration::CalibrateProgram the robot's real devices.

This header declares **1** type (4 members) and **6** free functions.

Extracted from [`include/shulib/hal/pros/calibrate.hpp`](../../include/shulib/hal/pros/calibrate.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

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

*class, declared at [`include/shulib/hal/pros/calibrate.hpp:78`](../../include/shulib/hal/pros/calibrate.hpp#L78).*

<a id="proscalibrateio-line"></a>

### `ProsCalibrateIo::line`

```cpp
void line(const char* text) override
```

"[R3D] <text>" on the USB serial port: ONE write() per line, through serialWrapped() (SER1) — a text past one serial line goes out in "[R3D] ... " pieces.

*function, declared at [`include/shulib/hal/pros/calibrate.hpp:82`](../../include/shulib/hal/pros/calibrate.hpp#L82).*

<a id="proscalibrateio-competitionstatus"></a>

### `ProsCalibrateIo::competitionStatus`

```cpp
[[nodiscard]] std::uint8_t competitionStatus() override
```

pros::c::competition_get_status().

*function, declared at [`include/shulib/hal/pros/calibrate.hpp:84`](../../include/shulib/hal/pros/calibrate.hpp#L84).*

<a id="proscalibrateio-delayms"></a>

### `ProsCalibrateIo::delayMs`

```cpp
void delayMs(std::uint32_t ms) override
```

pros::delay(ms).

*function, declared at [`include/shulib/hal/pros/calibrate.hpp:88`](../../include/shulib/hal/pros/calibrate.hpp#L88).*

<a id="proscalibrateio-readshaftdegrees"></a>

### `ProsCalibrateIo::readShaftDegrees`

```cpp
[[nodiscard]] bool readShaftDegrees(int port, double& degrees) override
```

THE WATCH'S READ (header): motor_get_position() on `port` in the units the motor reports, turned into degrees of output shaft — no motor object, nothing written. False when PROS answers PROS_ERR_F (not a motor, or not answering), or a unit or gearing it does not know.

*function, declared at [`include/shulib/hal/pros/calibrate.hpp:92`](../../include/shulib/hal/pros/calibrate.hpp#L92).*

<a id="calibrateinitialize"></a>

## `calibrateInitialize`

```cpp
inline void calibrateInitialize(const char* label = nullptr, const char* buildStamp = __DATE__ " " __TIME__)
```

initialize(): the boot banner (constructs nothing, powers nothing). `label` is a build's beacon, or null; `buildStamp` defaults to this translation unit's own (header: ONE unit).

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:120`](../../include/shulib/hal/pros/calibrate.hpp#L120).*

<a id="calibratestatescreen"></a>

## `calibrateStateScreen`

```cpp
inline void calibrateStateScreen(const char* state, const char* why)
```

disabled(), competition_initialize() and autonomous(): Calibrate's competition-state screen — `state` and `why` are calibrate_screen.hpp's words (checked at compile time to fit).

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:128`](../../include/shulib/hal/pros/calibrate.hpp#L128).*

<a id="calibratedisabled"></a>

## `calibrateDisabled`

```cpp
inline void calibrateDisabled()
```

disabled(): the state screen, DISABLED.

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:135`](../../include/shulib/hal/pros/calibrate.hpp#L135).*

<a id="calibrateconnected"></a>

## `calibrateConnected`

```cpp
inline void calibrateConnected()
```

competition_initialize(): the state screen, FIELD CONNECTED.

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:141`](../../include/shulib/hal/pros/calibrate.hpp#L141).*

<a id="calibrateautonomous"></a>

## `calibrateAutonomous`

```cpp
inline void calibrateAutonomous()
```

autonomous(): the state screen, AUTONOMOUS — no motion here, ever.

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:147`](../../include/shulib/hal/pros/calibrate.hpp#L147).*

<a id="runcalibrate"></a>

## `runCalibrate`

```cpp
inline void runCalibrate(const calibration::CalibrateFacts& facts, const char* buildStamp = __DATE__ " " __TIME__)
```

opcontrol(): the standard Calibrate on this brain, over the robot's real devices (header). Returns when the field disables the robot or starts autonomous (every motor at 0 V + Coast), or when no run can start. `buildStamp` defaults to this translation unit's own.

*free function, declared at [`include/shulib/hal/pros/calibrate.hpp:155`](../../include/shulib/hal/pros/calibrate.hpp#L155).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 45 lines</summary>

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
                         *Get each (disassembled for the CAL1 brief, 2026-10-08) — the belief CAL1
                         registers as HA-187 (in Part A2b): a read writes nothing to the motor.
                         False when PROS answers PROS_ERR_F, or a unit or gearing it does not know.

 A TEAM'S CALIBRATE is one file that states its facts and calls the five functions below from
 PROS's five entry points (21 lines; CAL1's next part adds the guide chapter that shows it whole).
 Include this header in ONE translation unit: its default build stamp is that unit's __DATE__ and
 __TIME__, so two units would carry two stamps for one inline function.

 The composition is runCalibrate()'s own frame: the registry, the provider over it (no log copy),
 the SD store (its default root), the screen and the IO — each borrowed by the program for its run.

 THE STACK it takes on PROS's opcontrol task (measured 2026-10-29 by GCC, never on a robot): the
 provider alone is 7,288 B on ARM and run() holds the 7,712 B calibrator, so the deepest path GCC
 can see in the team's one translation unit (arm-none-eabi-g++ -fstack-usage -fcallgraph-info=su,
 the team project's flags, over the 21-line file) is 22,464 B — runCalibrate 8,712, then
 CalibrateProgram::run 10,272, then the DONE path down to the safe save's CRC — BEFORE newlib's
 file and printf frames and VEXos's file calls, which it cannot see. PROS runs opcontrol() on a task
 of TASK_STACK_DEPTH_DEFAULT = 0x2000 words (32 KiB, include/pros/rtos.h:72): about 70 % of it
 before those frames (R3d's Calibrate, which this replaces in CAL1 Part B: 15,168 B, "about half").
 It fits; it is the margin to watch if the program's frame grows.

 NOTHING HERE HAS RUN ON A ROBOT in this form. Its host test (test/pros_calibrate_adapter_test.cpp)
 runs it through the host PROS shim, which tests the shell against our BELIEF about PROS, not the
 belief itself (shim_control.hpp says so in full).
```

</details>
