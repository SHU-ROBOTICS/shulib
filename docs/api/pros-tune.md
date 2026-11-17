<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/tune.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tune.hpp`

hal/pros/tune — "shulib Tune" on a V5 brain (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's D7 (a), §1.3): the THIN shell that hands mechanism::TuneProgram the robot's real devices.

This header declares **2** types (9 members), **3** free functions, and **1** constant.

Extracted from [`include/shulib/hal/pros/tune.hpp`](../../include/shulib/hal/pros/tune.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kTuneLogFile`](#ktunelogfile) — *constant*
- [`class ProsTuneIo`](#class-prostuneio)
  - [`competitionStatus`](#prostuneio-competitionstatus)
- [`class ProsTuneShell`](#class-prostuneshell)
  - [`ProsTuneShell`](#prostuneshell-prostuneshell)
  - [`ProsTuneShell (overload 2)`](#prostuneshell-prostuneshell-2)
  - [`operator=`](#prostuneshell-operator-eq)
  - [`ProsTuneShell (overload 3)`](#prostuneshell-prostuneshell-3)
  - [`operator= (overload 2)`](#prostuneshell-operator-eq-2)
  - [`~ProsTuneShell`](#prostuneshell-destructor-prostuneshell)
  - [`program`](#prostuneshell-program)
  - [`devices`](#prostuneshell-devices)
- [`tuneShell`](#tuneshell) — *free function*
- [`tuneInitialize`](#tuneinitialize) — *free function*
- [`runTune`](#runtune) — *free function*

<a id="ktunelogfile"></a>

## `kTuneLogFile`

```cpp
inline constexpr const char* kTuneLogFile = "tune_log.txt"
```

The Tune program's SD copy of its log: a BARE name (the provider owns the /usd/ prefix), APPENDED across boots (header).

*constant, declared at [`include/shulib/hal/pros/tune.hpp:57`](../../include/shulib/hal/pros/tune.hpp#L57).*

<a id="class-prostuneio"></a>

## `class ProsTuneIo`

```cpp
class ProsTuneIo final : public mechanism::TuneIo
```

mechanism::TuneIo on the brain: pros::c::competition_get_status().

*class, declared at [`include/shulib/hal/pros/tune.hpp:60`](../../include/shulib/hal/pros/tune.hpp#L60).*

<a id="prostuneio-competitionstatus"></a>

### `ProsTuneIo::competitionStatus`

```cpp
[[nodiscard]] std::uint8_t competitionStatus() override
```

pros::c::competition_get_status().

*function, declared at [`include/shulib/hal/pros/tune.hpp:63`](../../include/shulib/hal/pros/tune.hpp#L63).*

<a id="class-prostuneshell"></a>

## `class ProsTuneShell`

```cpp
class ProsTuneShell
```

Everything "shulib Tune" runs on, in one object (header): the provider with its Appended SD copy, the screen, the master controller, the status, and the program over them. Lives in static storage (tuneShell()); not copyable or movable.

*class, declared at [`include/shulib/hal/pros/tune.hpp:69`](../../include/shulib/hal/pros/tune.hpp#L69).*

<a id="prostuneshell-prostuneshell"></a>

### `ProsTuneShell::ProsTuneShell`

```cpp
ProsTuneShell(const mechanism::MechanismTable& table, const char* buildStamp, const char* copyMountRoot = "/usd/")
```

The shell for the robot's `table` (a constant that outlives it). `copyMountRoot` is the card's mount point (ending in '/'), injectable so a host test writes the same path to a scratch directory; `buildStamp` the build's (the caller's __DATE__ " " __TIME__). Makes NO device.

*function, declared at [`include/shulib/hal/pros/tune.hpp:74`](../../include/shulib/hal/pros/tune.hpp#L74).*

<a id="prostuneshell-prostuneshell-2"></a>

### `ProsTuneShell::ProsTuneShell (overload 2)`

```cpp
ProsTuneShell(const ProsTuneShell&) = delete
```

Not copyable: the program borrows its members.

*function, declared at [`include/shulib/hal/pros/tune.hpp:78`](../../include/shulib/hal/pros/tune.hpp#L78).*

<a id="prostuneshell-operator-eq"></a>

### `ProsTuneShell::operator=`

```cpp
ProsTuneShell& operator=(const ProsTuneShell&) = delete
```

Not copyable: the program borrows its members. Not copy-assignable.

*function, declared at [`include/shulib/hal/pros/tune.hpp:80`](../../include/shulib/hal/pros/tune.hpp#L80).*

<a id="prostuneshell-prostuneshell-3"></a>

### `ProsTuneShell::ProsTuneShell (overload 3)`

```cpp
ProsTuneShell(ProsTuneShell&&) = delete
```

Not copyable: the program borrows its members. Not copy-assignable. Not movable.

*function, declared at [`include/shulib/hal/pros/tune.hpp:82`](../../include/shulib/hal/pros/tune.hpp#L82).*

<a id="prostuneshell-operator-eq-2"></a>

### `ProsTuneShell::operator= (overload 2)`

```cpp
ProsTuneShell& operator=(ProsTuneShell&&) = delete
```

Not copyable: the program borrows its members. Not copy-assignable. Not movable. Not move-assignable.

*function, declared at [`include/shulib/hal/pros/tune.hpp:84`](../../include/shulib/hal/pros/tune.hpp#L84).*

<a id="prostuneshell-destructor-prostuneshell"></a>

### `ProsTuneShell::~ProsTuneShell`

```cpp
~ProsTuneShell() = default
```

Not copyable: the program borrows its members. Not copy-assignable. Not movable. Not move-assignable. Destroys the program, then the devices it borrowed.

*function, declared at [`include/shulib/hal/pros/tune.hpp:86`](../../include/shulib/hal/pros/tune.hpp#L86).*

<a id="prostuneshell-program"></a>

### `ProsTuneShell::program`

```cpp
[[nodiscard]] mechanism::TuneProgram& program() noexcept
```

The program.

*function, declared at [`include/shulib/hal/pros/tune.hpp:89`](../../include/shulib/hal/pros/tune.hpp#L89).*

<a id="prostuneshell-devices"></a>

### `ProsTuneShell::devices`

```cpp
[[nodiscard]] ProsDeviceProvider& devices() noexcept
```

The provider (its SD copy's state: serialCopyOpen(), serialCopyAppendOpen()).

*function, declared at [`include/shulib/hal/pros/tune.hpp:91`](../../include/shulib/hal/pros/tune.hpp#L91).*

<a id="tuneshell"></a>

## `tuneShell`

```cpp
inline ProsTuneShell& tuneShell(const mechanism::MechanismTable& table, const char* buildStamp = __DATE__ " " __TIME__)
```

The program's ONE shell, made on the first call (header) over `table`. Every later call returns the same shell, whatever it is passed.

*free function, declared at [`include/shulib/hal/pros/tune.hpp:103`](../../include/shulib/hal/pros/tune.hpp#L103).*

<a id="tuneinitialize"></a>

## `tuneInitialize`

```cpp
inline void tuneInitialize(const mechanism::MechanismTable& table, const char* label = nullptr, const char* statusWords = nullptr)
```

initialize(): the boot lines — makes NOTHING on the robot. `label` is the build's beacon, or null; `statusWords` the competition status in words, or null for its byte in hex.

*free function, declared at [`include/shulib/hal/pros/tune.hpp:111`](../../include/shulib/hal/pros/tune.hpp#L111).*

<a id="runtune"></a>

## `runTune`

```cpp
inline void runTune(const mechanism::MechanismTable& table)
```

opcontrol(): the program (mechanism::TuneProgram::run()) — returns when VEXos reports the robot DISABLED or autonomous starting, every mechanism released first.

*free function, declared at [`include/shulib/hal/pros/tune.hpp:120`](../../include/shulib/hal/pros/tune.hpp#L120).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 30 lines</summary>

```text

 hal/pros/tune — "shulib Tune" on a V5 brain (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's
 D7 (a), §1.3): the THIN shell that hands mechanism::TuneProgram the robot's real devices.
 Everything the program decides is in include/shulib/mechanism/tune_program.hpp and runs on the host
 test plant; this file only composes the PROS adapters — the brief's list, item by item:
   * THE PROVIDER, with its SD log copy at /usd/tune_log.txt APPENDED across boots (the brief's
     re-check #14: a reboot back into Tune must not erase the last session's COPY — the Competition
     program's mode, hal::pros::ProsFileMode::Append). The copy carries every line the program logs,
     the COPY blocks included; the terminal gets them all too. A missing card leaves the terminal;
   * THE SCREEN (ProsBrainScreen), THE CONTROLLER (the master: every button a STOP, the left stick the
     jog) and THE COMPETITION STATUS (pros::c::competition_get_status(), ProsTuneIo);
   * THE TASK: the program runs on PROS's OPCONTROL task — the standard Calibrate's (hal/pros/calibrate.hpp),
     not a task of its own like the Auton Tester's (a delta from the brief's "the task", logged with the
     run: Tune moves nothing unless driver control runs with no field, so on a disable or the
     start of autonomous it releases every mechanism and returns — PROS calls opcontrol() again later,
     and the program, in this file's STATIC storage, carries on with the same mechanisms and numbers; a
     task PROS deletes mid-tick leaves the service's re-entrancy flag set, which the next entry clears
     (MechanismService::enterDriverControl()); and the host's PROS shim cannot construct a task).

 A ROBOT'S TUNE is two calls from its program file — src/programs/tune_program.cpp:
     tuneInitialize(kMechanisms, label);   // initialize(): the boot lines, nothing made
     runTune(kMechanisms);                 // opcontrol(): the program
 with the build's mechanism table. The first call MAKES the shell (a function-local static, so it is
 made once, on PROS's initialize task, and lives for the program); every later call uses the same one.

 THE STACK: the shell lives in static storage, not on a task's stack — the program's state (its
 samples, its view, the table's copy) is static too.

 NOTHING HERE HAS RUN ON A ROBOT. Its host test (test/tune_program_test.cpp) runs it through the host
 PROS shim, which tests the shell against our BELIEF about PROS, not the belief itself.
```

</details>
