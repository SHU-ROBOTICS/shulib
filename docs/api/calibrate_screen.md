<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/calibrate_screen.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `calibrate_screen.hpp`

calibrate_screen — how the standard Calibrate PAINTS the brain's screen.

This header declares **2** types (10 members), **2** free functions, and **48** constants.

Extracted from [`include/shulib/calibration/calibrate_screen.hpp`](../../include/shulib/calibration/calibrate_screen.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kLiveRepaintSeconds`](#kliverepaintseconds) — *constant*
- [`kBuildTimeCols`](#kbuildtimecols) — *constant*
- [`kColBg`](#kcolbg) — *constant*
- [`kColText`](#kcoltext) — *constant*
- [`kColBlue`](#kcolblue) — *constant*
- [`kColAmber`](#kcolamber) — *constant*
- [`kColGreen`](#kcolgreen) — *constant*
- [`kColRed`](#kcolred) — *constant*
- [`kColGrey`](#kcolgrey) — *constant*
- [`kColGreyFill`](#kcolgreyfill) — *constant*
- [`kColAbortFill`](#kcolabortfill) — *constant*
- [`kColEdge`](#kcoledge) — *constant*
- [`kHeaderStarting`](#kheaderstarting) — *constant*
- [`kHeaderCannot`](#kheadercannot) — *constant*
- [`kHeaderStopped`](#kheaderstopped) — *constant*
- [`kHandsOff`](#khandsoff) — *constant*
- [`kStartingUp`](#kstartingup) — *constant*
- [`kCartridgeUnset`](#kcartridgeunset) — *constant*
- [`kDriveUnset`](#kdriveunset) — *constant*
- [`kNotTank`](#knottank) — *constant*
- [`kFactsBad`](#kfactsbad) — *constant*
- [`kImuCount`](#kimucount) — *constant*
- [`kTooFewMotors`](#ktoofewmotors) — *constant*
- [`kMotorsRefused`](#kmotorsrefused) — *constant*
- [`kImuRefused`](#kimurefused) — *constant*
- [`kProgramFault`](#kprogramfault) — *constant*
- [`kNotDriverControl`](#knotdrivercontrol) — *constant*
- [`kFindingDevices`](#kfindingdevices) — *constant*
- [`kNothingPowered`](#knothingpowered) — *constant*
- [`kDiagnosisTool`](#kdiagnosistool) — *constant*
- [`kMoreOnSerial`](#kmoreonserial) — *constant*
- [`kFaultIdle`](#kfaultidle) — *constant*
- [`kNdc1`](#kndc1) — *constant*
- [`kNdc2`](#kndc2) — *constant*
- [`kNdc3`](#kndc3) — *constant*
- [`kNdc4`](#kndc4) — *constant*
- [`kNdc5`](#kndc5) — *constant*
- [`kState1`](#kstate1) — *constant*
- [`kState3`](#kstate3) — *constant*
- [`kState4`](#kstate4) — *constant*
- [`kState5`](#kstate5) — *constant*
- [`kStateNameDisabled`](#kstatenamedisabled) — *constant*
- [`kStateNameConnected`](#kstatenameconnected) — *constant*
- [`kStateNameAutonomous`](#kstatenameautonomous) — *constant*
- [`kStateWhyDisabled`](#kstatewhydisabled) — *constant*
- [`kStateWhyConnected`](#kstatewhyconnected) — *constant*
- [`kStateWhyAutonomous`](#kstatewhyautonomous) — *constant*
- [`struct Checked`](#struct-checked)
  - [`text`](#checked-text)
  - [`cols`](#checked-cols)
  - [`lines`](#checked-lines)
- [`kAll`](#kall) — *constant*
- [`fits`](#fits) — *free function*
- [`allFit`](#allfit) — *free function*
- [`class CalibratePainter`](#class-calibratepainter)
  - [`CalibratePainter`](#calibratepainter-calibratepainter)
  - [`invalidate`](#calibratepainter-invalidate)
  - [`render`](#calibratepainter-render)
  - [`paintStarting`](#calibratepainter-paintstarting)
  - [`paintCannot`](#calibratepainter-paintcannot)
  - [`paintNotDriverControl`](#calibratepainter-paintnotdrivercontrol)
  - [`paintStateScreen`](#calibratepainter-paintstatescreen)

<a id="kliverepaintseconds"></a>

## `kLiveRepaintSeconds`

```cpp
inline constexpr double kLiveRepaintSeconds = 0.2
```

The least time between two in-place repaints of one line whose TEXT changed and whose colour did not — a live number. 5 Hz: legible, and it bounds PROS's believed per-print leak (header).

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:62`](../../include/shulib/calibration/calibrate_screen.hpp#L62).*

<a id="kbuildtimecols"></a>

## `kBuildTimeCols`

```cpp
inline constexpr std::size_t kBuildTimeCols = 14
```

The characters the header's build-time text may take: "BUILD hh:mm:ss" is exactly this many. A longer text is cut here, so it can never run into the screen's right edge (checked below).

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:66`](../../include/shulib/calibration/calibrate_screen.hpp#L66).*

<a id="kcolbg"></a>

## `kColBg`

```cpp
inline constexpr std::uint32_t kColBg = 0x101418
```

the ground, and dark text on a bright bar

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:75`](../../include/shulib/calibration/calibrate_screen.hpp#L75).*

<a id="kcoltext"></a>

## `kColText`

```cpp
inline constexpr std::uint32_t kColText = 0xF0F0F0
```

ordinary text

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:76`](../../include/shulib/calibration/calibrate_screen.hpp#L76).*

<a id="kcolblue"></a>

## `kColBlue`

```cpp
inline constexpr std::uint32_t kColBlue = 0x1E5AA8
```

the unpowered header; the question boxes

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:77`](../../include/shulib/calibration/calibrate_screen.hpp#L77).*

<a id="kcolamber"></a>

## `kColAmber`

```cpp
inline constexpr std::uint32_t kColAmber = 0xE8B23A
```

powered: stand clear

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:78`](../../include/shulib/calibration/calibrate_screen.hpp#L78).*

<a id="kcolgreen"></a>

## `kColGreen`

```cpp
inline constexpr std::uint32_t kColGreen = 0x39C36E
```

ready, saved, past the floor; DONE's fill

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:79`](../../include/shulib/calibration/calibrate_screen.hpp#L79).*

<a id="kcolred"></a>

## `kColRed`

```cpp
inline constexpr std::uint32_t kColRed = 0xE2564A
```

stopped, not saved; ABORT's edge

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:80`](../../include/shulib/calibration/calibrate_screen.hpp#L80).*

<a id="kcolgrey"></a>

## `kColGrey`

```cpp
inline constexpr std::uint32_t kColGrey = 0x9AA4AE
```

quiet text; a greyed button's label

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:81`](../../include/shulib/calibration/calibrate_screen.hpp#L81).*

<a id="kcolgreyfill"></a>

## `kColGreyFill`

```cpp
inline constexpr std::uint32_t kColGreyFill = 0x3A4450
```

a greyed (arming) button's fill

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:82`](../../include/shulib/calibration/calibrate_screen.hpp#L82).*

<a id="kcolabortfill"></a>

## `kColAbortFill`

```cpp
inline constexpr std::uint32_t kColAbortFill = 0xB0302A
```

ABORT's fill (white on it: 5.6:1)

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:83`](../../include/shulib/calibration/calibrate_screen.hpp#L83).*

<a id="kcoledge"></a>

## `kColEdge`

```cpp
inline constexpr std::uint32_t kColEdge = 0xC8D8F0
```

a live button's edge; Blue text, if ever asked

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:84`](../../include/shulib/calibration/calibrate_screen.hpp#L84).*

<a id="kheaderstarting"></a>

## `kHeaderStarting`

```cpp
inline constexpr std::string_view kHeaderStarting = "CALIBRATE"
```

Header word (kHeaderCols): the first screen, while the census runs.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:223`](../../include/shulib/calibration/calibrate_screen.hpp#L223).*

<a id="kheadercannot"></a>

## `kHeaderCannot`

```cpp
inline constexpr std::string_view kHeaderCannot = "CANNOT START"
```

Header word: a refusal before any run could start.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:225`](../../include/shulib/calibration/calibrate_screen.hpp#L225).*

<a id="kheaderstopped"></a>

## `kHeaderStopped`

```cpp
inline constexpr std::string_view kHeaderStopped = "STOPPED -- MOTORS OFF"
```

Header word: the loop stopped — not driver control, or a program fault.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:227`](../../include/shulib/calibration/calibrate_screen.hpp#L227).*

<a id="khandsoff"></a>

## `kHandsOff`

```cpp
inline constexpr std::string_view kHandsOff = "HANDS OFF"
```

LARGE (kLargeCols): the first screen's first line.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:229`](../../include/shulib/calibration/calibrate_screen.hpp#L229).*

<a id="kstartingup"></a>

## `kStartingUp`

```cpp
inline constexpr std::string_view kStartingUp = "STARTING UP"
```

LARGE: the first screen's second line.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:231`](../../include/shulib/calibration/calibrate_screen.hpp#L231).*

<a id="kcartridgeunset"></a>

## `kCartridgeUnset`

```cpp
inline constexpr std::string_view kCartridgeUnset = "CARTRIDGE UNSET"
```

LARGE: the drive motors' cartridge is not stated.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:233`](../../include/shulib/calibration/calibrate_screen.hpp#L233).*

<a id="kdriveunset"></a>

## `kDriveUnset`

```cpp
inline constexpr std::string_view kDriveUnset = "DRIVE KIND UNSET"
```

LARGE: the drive kind is not stated.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:235`](../../include/shulib/calibration/calibrate_screen.hpp#L235).*

<a id="knottank"></a>

## `kNotTank`

```cpp
inline constexpr std::string_view kNotTank = "NOT A TANK"
```

LARGE: the stated drive kind is not a tank, the one kind Calibrate measures.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:237`](../../include/shulib/calibration/calibrate_screen.hpp#L237).*

<a id="kfactsbad"></a>

## `kFactsBad`

```cpp
inline constexpr std::string_view kFactsBad = "FACTS REFUSED"
```

LARGE: the stated facts contradict themselves (a mechanism port named twice, out of range…).

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:239`](../../include/shulib/calibration/calibrate_screen.hpp#L239).*

<a id="kimucount"></a>

## `kImuCount`

```cpp
inline constexpr std::string_view kImuCount = "IMU COUNT NOT 1"
```

LARGE: the census did not find exactly one IMU.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:241`](../../include/shulib/calibration/calibrate_screen.hpp#L241).*

<a id="ktoofewmotors"></a>

## `kTooFewMotors`

```cpp
inline constexpr std::string_view kTooFewMotors = "TOO FEW MOTORS"
```

LARGE: fewer than two drive candidates.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:243`](../../include/shulib/calibration/calibrate_screen.hpp#L243).*

<a id="kmotorsrefused"></a>

## `kMotorsRefused`

```cpp
inline constexpr std::string_view kMotorsRefused = "MOTORS REFUSED"
```

LARGE: a motor could not be made.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:245`](../../include/shulib/calibration/calibrate_screen.hpp#L245).*

<a id="kimurefused"></a>

## `kImuRefused`

```cpp
inline constexpr std::string_view kImuRefused = "IMU REFUSED"
```

LARGE: the IMU could not be made.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:247`](../../include/shulib/calibration/calibrate_screen.hpp#L247).*

<a id="kprogramfault"></a>

## `kProgramFault`

```cpp
inline constexpr std::string_view kProgramFault = "PROGRAM FAULT"
```

LARGE: something the program did not expect stopped it.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:249`](../../include/shulib/calibration/calibrate_screen.hpp#L249).*

<a id="knotdrivercontrol"></a>

## `kNotDriverControl`

```cpp
inline constexpr std::string_view kNotDriverControl = "NOT DRIVER CONTROL"
```

LARGE: the competition state is not driver control, enabled.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:251`](../../include/shulib/calibration/calibrate_screen.hpp#L251).*

<a id="kfindingdevices"></a>

## `kFindingDevices`

```cpp
inline constexpr std::string_view kFindingDevices = "finding the motors and the IMU"
```

MEDIUM (kMediumCols): the first screen's third line.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:253`](../../include/shulib/calibration/calibrate_screen.hpp#L253).*

<a id="knothingpowered"></a>

## `kNothingPowered`

```cpp
inline constexpr std::string_view kNothingPowered = "nothing is powered -- fix it, then restart it"
```

SMALL (kSmallCols): a refusal's footer.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:255`](../../include/shulib/calibration/calibrate_screen.hpp#L255).*

<a id="kdiagnosistool"></a>

## `kDiagnosisTool`

```cpp
inline constexpr std::string_view kDiagnosisTool = "the census is on the serial log"
```

SMALL: where to look next after a refusal, when its text fitted on the screen.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:257`](../../include/shulib/calibration/calibrate_screen.hpp#L257).*

<a id="kmoreonserial"></a>

## `kMoreOnSerial`

```cpp
inline constexpr std::string_view kMoreOnSerial = "the full text is on the serial log"
```

SMALL: where to look next when a refusal's text did NOT fit on the screen.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:259`](../../include/shulib/calibration/calibrate_screen.hpp#L259).*

<a id="kfaultidle"></a>

## `kFaultIdle`

```cpp
inline constexpr std::string_view kFaultIdle = "every motor is at 0 V, coasting"
```

SMALL: a program fault's footer.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:261`](../../include/shulib/calibration/calibrate_screen.hpp#L261).*

<a id="kndc1"></a>

## `kNdc1`

```cpp
inline constexpr std::string_view kNdc1 = "It powers the motors, so it runs ONLY in"
```

SMALL: the not-driver-control screen, line 1.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:263`](../../include/shulib/calibration/calibrate_screen.hpp#L263).*

<a id="kndc2"></a>

## `kNdc2`

```cpp
inline constexpr std::string_view kNdc2 = "DRIVER CONTROL, enabled. Motors: 0 V, coasting."
```

SMALL: the not-driver-control screen, line 2.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:265`](../../include/shulib/calibration/calibrate_screen.hpp#L265).*

<a id="kndc3"></a>

## `kNdc3`

```cpp
inline constexpr std::string_view kNdc3 = "Unplug any competition switch or field cable,"
```

SMALL: the not-driver-control screen, line 3.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:267`](../../include/shulib/calibration/calibrate_screen.hpp#L267).*

<a id="kndc4"></a>

## `kNdc4`

```cpp
inline constexpr std::string_view kNdc4 = "and NEVER start it from the CONTROLLER's menu."
```

SMALL: the not-driver-control screen, line 4.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:269`](../../include/shulib/calibration/calibrate_screen.hpp#L269).*

<a id="kndc5"></a>

## `kNdc5`

```cpp
inline constexpr std::string_view kNdc5 = "Any result is on the serial log too ([R3D])."
```

SMALL: the not-driver-control screen, line 5.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:271`](../../include/shulib/calibration/calibrate_screen.hpp#L271).*

<a id="kstate1"></a>

## `kState1`

```cpp
inline constexpr std::string_view kState1 = "This program runs ONLY in DRIVER CONTROL, enabled."
```

SMALL: the competition-state screen (disabled(), competition_initialize(), autonomous()), under the state's name.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:274`](../../include/shulib/calibration/calibrate_screen.hpp#L274).*

<a id="kstate3"></a>

## `kState3`

```cpp
inline constexpr std::string_view kState3 = "Look at the CONTROLLER: it names this state."
```

SMALL: the competition-state screen, what to look at.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:276`](../../include/shulib/calibration/calibrate_screen.hpp#L276).*

<a id="kstate4"></a>

## `kState4`

```cpp
inline constexpr std::string_view kState4 = "Unplug any competition switch or field cable"
```

SMALL: the competition-state screen, what to do (first line).

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:278`](../../include/shulib/calibration/calibrate_screen.hpp#L278).*

<a id="kstate5"></a>

## `kState5`

```cpp
inline constexpr std::string_view kState5 = "from the controller. This clears by itself."
```

SMALL: the competition-state screen, what to do (second line).

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:280`](../../include/shulib/calibration/calibrate_screen.hpp#L280).*

<a id="kstatenamedisabled"></a>

## `kStateNameDisabled`

```cpp
inline constexpr std::string_view kStateNameDisabled = "DISABLED"
```

LARGE: the competition-state screen's name for disabled() — what a program passes as paintStateScreen()'s `state`.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:283`](../../include/shulib/calibration/calibrate_screen.hpp#L283).*

<a id="kstatenameconnected"></a>

## `kStateNameConnected`

```cpp
inline constexpr std::string_view kStateNameConnected = "FIELD CONNECTED"
```

LARGE: the name for competition_initialize() (robot two's other programs say "FIELD CONTROL CONNECTED", 23 characters — five past the LARGE budget).

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:286`](../../include/shulib/calibration/calibrate_screen.hpp#L286).*

<a id="kstatenameautonomous"></a>

## `kStateNameAutonomous`

```cpp
inline constexpr std::string_view kStateNameAutonomous = "AUTONOMOUS"
```

LARGE: the name for autonomous().

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:288`](../../include/shulib/calibration/calibrate_screen.hpp#L288).*

<a id="kstatewhydisabled"></a>

## `kStateWhyDisabled`

```cpp
inline constexpr std::string_view kStateWhyDisabled = "VEXos reports the robot DISABLED, so the driver task is not running."
```

SMALL, wrapped onto two lines: why disabled() shows the state screen — paintStateScreen()'s `why`.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:291`](../../include/shulib/calibration/calibrate_screen.hpp#L291).*

<a id="kstatewhyconnected"></a>

## `kStateWhyConnected`

```cpp
inline constexpr std::string_view kStateWhyConnected = "A competition switch or field is connected through the controller."
```

SMALL, wrapped onto two lines: why competition_initialize() does.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:294`](../../include/shulib/calibration/calibrate_screen.hpp#L294).*

<a id="kstatewhyautonomous"></a>

## `kStateWhyAutonomous`

```cpp
inline constexpr std::string_view kStateWhyAutonomous = "No motion here, ever: calibration powers the motors only while a person stands by the " "battery."
```

SMALL, wrapped onto two lines: why autonomous() does.

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:297`](../../include/shulib/calibration/calibrate_screen.hpp#L297).*

<a id="struct-checked"></a>

## `struct Checked`

```cpp
struct Checked
```

One literal and the budget it must fit.

*struct, declared at [`include/shulib/calibration/calibrate_screen.hpp:302`](../../include/shulib/calibration/calibrate_screen.hpp#L302).*

<a id="checked-text"></a>

### `Checked::text`

```cpp
std::string_view text
```

the literal

*field, declared at [`include/shulib/calibration/calibrate_screen.hpp:303`](../../include/shulib/calibration/calibrate_screen.hpp#L303).*

<a id="checked-cols"></a>

### `Checked::cols`

```cpp
std::size_t cols
```

its line's budget

*field, declared at [`include/shulib/calibration/calibrate_screen.hpp:304`](../../include/shulib/calibration/calibrate_screen.hpp#L304).*

<a id="checked-lines"></a>

### `Checked::lines`

```cpp
std::size_t lines = 1
```

1: one line of `cols`. More: wrapped by wrapWords() onto at most this many lines of `cols` (detail::wrapsCompletely() names the one wrapped shape a screen draws).

*field, declared at [`include/shulib/calibration/calibrate_screen.hpp:307`](../../include/shulib/calibration/calibrate_screen.hpp#L307).*

<a id="kall"></a>

## `kAll`

```cpp
inline constexpr Checked kAll[] = { {kHeaderStarting, kHeaderCols}, {kHeaderCannot, kHeaderCols}, {kHeaderStopped, kHeaderCols}, {kHandsOff, kLargeCols}, {kStartingUp, kLargeCols}, {kCartridgeUnset, kLargeCols}, {kDriveUnset, kLargeCols}, {kNotTank, kLargeCols}, {kFactsBad, kLargeCols}, {kImuCount, kLargeCols}, {kTooFewMotors, kLargeCols}, {kMotorsRefused, kLargeCols}, {kImuRefused, kLargeCols}, {kProgramFault, kLargeCols}, {kNotDriverControl, kLargeCols}, {kFindingDevices, kMediumCols}, {kNothingPowered, kSmallCols}, {kDiagnosisTool, kSmallCols}, {kMoreOnSerial, kSmallCols}, {kFaultIdle, kSmallCols}, {kNdc1, kSmallCols}, {kNdc2, kSmallCols}, {kNdc3, kSmallCols}, {kNdc4, kSmallCols}, {kNdc5, kSmallCols}, {kState1, kSmallCols}, {kState3, kSmallCols}, {kState4, kSmallCols}, {kState5, kSmallCols}, {kStateNameDisabled, kLargeCols}, {kStateNameConnected, kLargeCols}, {kStateNameAutonomous, kLargeCols}, {kStateWhyDisabled, kSmallCols, detail::kStateWhyLines}, {kStateWhyConnected, kSmallCols, detail::kStateWhyLines}, {kStateWhyAutonomous, kSmallCols, detail::kStateWhyLines}, }
```

EVERY literal above with its budget — the list the compile-time check walks (and a host test holds this list and the literals above to one another, so a literal cannot be left out).

*constant, declared at [`include/shulib/calibration/calibrate_screen.hpp:312`](../../include/shulib/calibration/calibrate_screen.hpp#L312).*

<a id="fits"></a>

## `fits`

```cpp
[[nodiscard]] constexpr bool fits(const Checked& c) noexcept
```

True when `c` fits: printable ASCII with no '%', and either one line of at most `c.cols` characters or — a wrapped literal — every word on its `c.lines` lines.

*free function, declared at [`include/shulib/calibration/calibrate_screen.hpp:337`](../../include/shulib/calibration/calibrate_screen.hpp#L337).*

<a id="allfit"></a>

## `allFit`

```cpp
template <std::size_t N> [[nodiscard]] constexpr bool allFit(const Checked (&list)[N]) noexcept
```

True when every literal in `list` fits its budget and is printable ASCII with no '%'. This header's static_assert runs it over kAll at compile time, so a budget change that breaks a literal fails the build. (A host test holds that fits() and allFit() refuse what they must — a word one character over, a '%', a non-ASCII byte, a reason that does not wrap, a bad entry anywhere in a list — so a check weakened to pass everything fails to build that test.)

*free function, declared at [`include/shulib/calibration/calibrate_screen.hpp:350`](../../include/shulib/calibration/calibrate_screen.hpp#L350).*

<a id="class-calibratepainter"></a>

## `class CalibratePainter`

```cpp
class CalibratePainter
```

Draws calibration screens on one IBrainScreen: the header bar (with the build TIME, so a stale upload shows at a glance), PanelViews by difference (the header's rules), and the program's own screens — each of which paints over whatever was there and marks the painter's record stale, so the next render() repaints in full. Non-owning: the screen must outlive the painter. Not thread-safe: one task paints.

*class, declared at [`include/shulib/calibration/calibrate_screen.hpp:367`](../../include/shulib/calibration/calibrate_screen.hpp#L367).*

<a id="calibratepainter-calibratepainter"></a>

### `CalibratePainter::CalibratePainter`

```cpp
CalibratePainter(hal::IBrainScreen& screen, const char* buildTime) noexcept
```

Paint on `screen`. `buildTime` is the header's right-hand text — "BUILD hh:mm:ss", cut at kBuildTimeCols without a word on screen — and must outlive the painter (a string literal, or the program's own buffer); null paints none.

*function, declared at [`include/shulib/calibration/calibrate_screen.hpp:372`](../../include/shulib/calibration/calibrate_screen.hpp#L372).*

<a id="calibratepainter-invalidate"></a>

### `CalibratePainter::invalidate`

```cpp
void invalidate() noexcept
```

The next render() is a full repaint (a run was rebuilt, or another screen painted over it).

*function, declared at [`include/shulib/calibration/calibrate_screen.hpp:376`](../../include/shulib/calibration/calibrate_screen.hpp#L376).*

<a id="calibratepainter-render"></a>

### `CalibratePainter::render`

```cpp
void render(const PanelView& v, double now)
```

Bring the screen to `v` at `now` seconds, painting only what changed (the header's rules). `v`'s header word must stay valid while it is on the screen (the panel's words are string literals). Text past its budget — the header word past kHeaderCols, a line past its size's, a button label past MEDIUM's — is cut WITHOUT a word on screen: a view is expected to fit, and the views viewFor() builds are checked to (the file comment says how).

*function, declared at [`include/shulib/calibration/calibrate_screen.hpp:383`](../../include/shulib/calibration/calibrate_screen.hpp#L383).*

<a id="calibratepainter-paintstarting"></a>

### `CalibratePainter::paintStarting`

```cpp
void paintStarting()
```

The first screen, while the census runs and the devices are made.

*function, declared at [`include/shulib/calibration/calibrate_screen.hpp:449`](../../include/shulib/calibration/calibrate_screen.hpp#L449).*

<a id="calibratepainter-paintcannot"></a>

### `CalibratePainter::paintCannot`

```cpp
void paintCannot(std::string_view header, PanelTone tone, std::string_view headline, const char* why, const char* fix, std::string_view footer)
```

A screen the run never gets past: a header of `tone`, a LARGE `headline`, `why` wrapped in MEDIUM (three lines at most), `fix` wrapped in SMALL (two lines), a `footer`, and where to look next — or, when ANY of the five did not fit (a word cut, a line dropped), that the whole text is on the serial log. `header`, `headline` and `footer` are meant to be screen_text's words (checked at compile time); any other text is cut at its budget, and said to be.

*function, declared at [`include/shulib/calibration/calibrate_screen.hpp:467`](../../include/shulib/calibration/calibrate_screen.hpp#L467).*

<a id="calibratepainter-paintnotdrivercontrol"></a>

### `CalibratePainter::paintNotDriverControl`

```cpp
void paintNotDriverControl(const char* bits)
```

The loop's own "not driver control" screen. `bits` is the competition status in words ("status bits 0x.. field=.. disabled=.. auton=.."), one SMALL line — and, when it did not fit that line, a last line saying the full text is on the serial log.

*function, declared at [`include/shulib/calibration/calibrate_screen.hpp:498`](../../include/shulib/calibration/calibrate_screen.hpp#L498).*

<a id="calibratepainter-paintstatescreen"></a>

### `CalibratePainter::paintStateScreen`

```cpp
void paintStateScreen(const char* state, const char* why, const char* bits)
```

The competition-state screen for disabled(), competition_initialize() and autonomous(): the whole screen erased to black, `state` LARGE in amber, the rule, `why` wrapped in SMALL (two lines at most), `bits` (the status in words), and what to do — and, when `state`, `why` or `bits` did not fit, a last line saying the full text is on the serial log. Pass screen_text's words (kStateName*, kStateWhy*): those are checked at compile time to fit. Its colours are those of robot two's other programs' state screen (src/common/state_screens.cpp) and its layout follows that screen's, with two changes: `why` wraps onto two lines (y 100 and 116; that screen prints one, uncut, at 100) and the status bits move down to y 136 (from 130). Its words are re-cut to the budgets, which three of that screen's SMALL lines (53, 53 and 54 characters) and its 23-character LARGE name for competition_initialize() exceed. It paints no header, so a build-time text is not needed.

*function, declared at [`include/shulib/calibration/calibrate_screen.hpp:527`](../../include/shulib/calibration/calibrate_screen.hpp#L527).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 47 lines, click to expand</summary>

```text

 calibrate_screen — how the standard Calibrate PAINTS the brain's screen (chunk CAL1, Part A1,
 2026-10-08). The palette; the painter, which brings the screen to a new calibration::PanelView by
 drawing only the difference; and the program's own screens (starting, cannot start, not driver
 control, a program fault, and the competition-state screen). Everything it draws goes through
 hal::IBrainScreen, so the host suite RUNS it (hal::fake::FakeBrainScreen records every call)
 where until now only what the screen SAYS (calibrate_panel.hpp's viewFor() and TouchGate) was
 host-tested.

 ── Where it comes from ─────────────────────────────────────────────────────────────
 Its rules, its palette and its drawing calls, in their order, are those of the painter in
 "shulib Calibrate" (src/programs/calibrate_program.cpp — `PanelPainter`, its header and button
 helpers and its own screens, written at R3d Part 2F-2 on 2026-09-24), re-expressed through the
 seam. CAL1 Part B (2026-11-05) moved robot two onto the library's Calibrate and deleted that
 program and its painter; this file is the only one. Its words are the program's, with the
 words that named robot two generalized for every team (no slot numbers, no "Bench Tests": a
 team's project may have neither), and a few added for the standard program's refusals.

 ── The painter's rules ─────────────────────────────────────────────────────────────
   * a change of screen, or of any line's place or size, is a full repaint;
   * a line whose COLOUR changed, the header, and any button change are painted the tick they
     happen;
   * a line whose TEXT changed (a live number) is repainted at most every kLiveRepaintSeconds.
     PROS 4.2.2's print_at is BELIEVED to leak its formatted buffer on every call (R3d Part 2F's
     reading of the vendored libpros.a found no free; never measured), so a live number reprinted
     every 10 ms tick would leak megabytes over a few runs; 5 Hz is legible and leaks two orders
     of magnitude less. The screen counts its prints (IBrainScreen::prints()) so a program can
     report the count;
   * an in-place text change never erases: the new text is printed padded with spaces to the old
     text's length, and VEXos draws text pen-on-ERASER (an opaque cell — a hardware finding, Bench
     Tests' menu), so the old characters are covered without a flicker;
   * a button that vanished is erased to the ground; one that is new, moved, re-labelled or
     armed is painted whole.
 Every text budget, line height and position is an ESTIMATE (A4 register HA-136); every literal
 here is checked against its budget at compile time (screen_text::kAll), and every other text is
 cut by the screen's sanitizer (hal::screenSafeLine()), which also turns a '%' into '?'. An own
 screen that had to cut or drop text a CALLER passed (paintCannot(), paintNotDriverControl(),
 paintStateScreen()) says so on its last line: the full text is on the serial log. Two cuts are
 NOT said, because they are clamps that keep the layout whole, not messages: render() cuts a
 view's header word at kHeaderCols, each line at its size's budget and each button label at
 MEDIUM's; and the header, on every screen that has one, cuts the build time at kBuildTimeCols.
 The views viewFor() builds are not expected to reach the first (their literals are checked at
 compile time, and calibrate_panel's host test checks every view its calibration runs reach
 against these budgets); "BUILD hh:mm:ss" is exactly the second's width.

 PROS-free, allocation-free. NOTHING HERE HAS RUN ON A ROBOT in this form; the calls it makes
 are the ones "shulib Calibrate" has made on robot two since 2026-09-24.
```

</details>
