<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/auton_registry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `auton_registry.hpp`

competition::AutonRegistry — the autons a competition program can run, as DATA.

This header declares **9** types (50 members), **3** free functions, **5** constants, and **3** type aliass.

Extracted from [`include/shulib/competition/auton_registry.hpp`](../../include/shulib/competition/auton_registry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxAutons`](#kmaxautons) — *constant*
- [`kMaxAutonNameChars`](#kmaxautonnamechars) — *constant*
- [`kMaxMirroredAutonNameChars`](#kmaxmirroredautonnamechars) — *constant*
- [`kMaxAutonRefusals`](#kmaxautonrefusals) — *constant*
- [`enum class OnRefusal`](#enum-class-onrefusal)
  - [`Throw`](#onrefusal-throw)
  - [`DropEntry`](#onrefusal-dropentry)
- [`kNoAutonName`](#knoautonname) — *constant*
- [`enum class AutonKind`](#enum-class-autonkind)
  - [`Unset`](#autonkind-unset)
  - [`Match`](#autonkind-match)
  - [`Skills`](#autonkind-skills)
- [`autonKindWord`](#autonkindword) — *free function*
- [`struct AutonPeriods`](#struct-autonperiods)
  - [`match`](#autonperiods-match)
  - [`skills`](#autonperiods-skills)
  - [`validate`](#autonperiods-validate)
  - [`periodFor`](#autonperiods-periodfor)
- [`class StartPose`](#class-startpose)
  - [`StartPose`](#startpose-startpose)
  - [`field`](#startpose-field)
  - [`field (overload 2)`](#startpose-field-2)
  - [`relative`](#startpose-relative)
  - [`mode`](#startpose-mode)
  - [`pose`](#startpose-pose)
  - [`isSet`](#startpose-isset)
  - [`sameAs`](#startpose-sameas)
  - [`enum class StartPose::Mode`](#enum-class-startpose-mode)
    - [`Unset`](#startpose-mode-unset)
    - [`Field`](#startpose-mode-field)
    - [`Relative`](#startpose-mode-relative)
- [`startModeWord`](#startmodeword) — *free function*
- [`struct AutonContext`](#struct-autoncontext)
  - [`chassis`](#autoncontext-chassis)
  - [`guard`](#autoncontext-guard)
  - [`telemetry`](#autoncontext-telemetry)
  - [`mechanisms`](#autoncontext-mechanisms)
  - [`entry`](#autoncontext-entry)
  - [`mechanismService`](#autoncontext-mechanismservice)
- [`AutonRoutine`](#autonroutine) — *type alias*
- [`AutonEndAction`](#autonendaction) — *type alias*
- [`struct AutonEntry`](#struct-autonentry)
  - [`name`](#autonentry-name)
  - [`kind`](#autonentry-kind)
  - [`routine`](#autonentry-routine)
  - [`route`](#autonentry-route)
  - [`start`](#autonentry-start)
  - [`endActionAt`](#autonentry-endactionat)
  - [`hardStopAt`](#autonentry-hardstopat)
  - [`endAction`](#autonentry-endaction)
  - [`mirror`](#autonentry-mirror)
  - [`effectiveStart`](#autonentry-effectivestart)
- [`struct AutonVerdict`](#struct-autonverdict)
  - [`ok`](#autonverdict-ok)
  - [`why`](#autonverdict-why)
- [`checkAutonEntry`](#checkautonentry) — *free function*
- [`class AutonRegistry`](#class-autonregistry)
  - [`AutonRegistry`](#autonregistry-autonregistry)
  - [`check`](#autonregistry-check)
  - [`add`](#autonregistry-add)
  - [`onRefusal`](#autonregistry-onrefusal)
  - [`refusedCount`](#autonregistry-refusedcount)
  - [`refusal`](#autonregistry-refusal)
  - [`size`](#autonregistry-size)
  - [`empty`](#autonregistry-empty)
  - [`at`](#autonregistry-at)
  - [`find`](#autonregistry-find)
  - [`periods`](#autonregistry-periods)
  - [`periodFor`](#autonregistry-periodfor)
- [`AutonContext`](#autoncontext) — *type alias*

<a id="kmaxautons"></a>

## `kMaxAutons`

```cpp
inline constexpr std::size_t kMaxAutons = 16
```

The most autons one registry holds. Fixed capacity so registration never allocates; sixteen is several pages of the selector and more than a season has ever needed.

*constant, declared at [`include/shulib/competition/auton_registry.hpp:99`](../../include/shulib/competition/auton_registry.hpp#L99).*

<a id="kmaxautonnamechars"></a>

## `kMaxAutonNameChars`

```cpp
inline constexpr std::size_t kMaxAutonNameChars = ui::kMediumCols - 7
```

The longest auton name: "AUTON: " (7 characters) plus the name must fit ONE medium line (B25).

*constant, declared at [`include/shulib/competition/auton_registry.hpp:102`](../../include/shulib/competition/auton_registry.hpp#L102).*

<a id="kmaxmirroredautonnamechars"></a>

## `kMaxMirroredAutonNameChars`

```cpp
inline constexpr std::size_t kMaxMirroredAutonNameChars = ui::kMediumCols - 11
```

The longest MIRRORED auton name (MIRROR1-A, the brief's D8): its menu button "<name> SKILLS MIR" — the name, " SKILLS" (7) and " MIR" (4) — must fit ONE medium line.

*constant, declared at [`include/shulib/competition/auton_registry.hpp:107`](../../include/shulib/competition/auton_registry.hpp#L107).*

<a id="kmaxautonrefusals"></a>

## `kMaxAutonRefusals`

```cpp
inline constexpr std::size_t kMaxAutonRefusals = 8
```

The most refused entries one registry keeps the sentences of (refusal()); refusedCount() counts past it.

*constant, declared at [`include/shulib/competition/auton_registry.hpp:115`](../../include/shulib/competition/auton_registry.hpp#L115).*

<a id="enum-class-onrefusal"></a>

## `enum class OnRefusal`

```cpp
enum class OnRefusal : std::uint8_t
```

What AutonRegistry::add() does with an entry its check() refuses (file banner: A refused entry).

*enum class, declared at [`include/shulib/competition/auton_registry.hpp:118`](../../include/shulib/competition/auton_registry.hpp#L118).*

<a id="onrefusal-throw"></a>

### `OnRefusal::Throw`

```cpp
Throw = 0
```

throw PreconditionError carrying check()'s sentence (B15; the default)

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:119`](../../include/shulib/competition/auton_registry.hpp#L119).*

<a id="onrefusal-dropentry"></a>

### `OnRefusal::DropEntry`

```cpp
DropEntry = 1
```

register nothing, keep check()'s sentence (refusal()), and return

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:120`](../../include/shulib/competition/auton_registry.hpp#L120).*

<a id="knoautonname"></a>

## `kNoAutonName`

```cpp
inline constexpr std::string_view kNoAutonName = "NONE"
```

The selector's reserved choice. No registered auton may be called this.

*constant, declared at [`include/shulib/competition/auton_registry.hpp:124`](../../include/shulib/competition/auton_registry.hpp#L124).*

<a id="enum-class-autonkind"></a>

## `enum class AutonKind`

```cpp
enum class AutonKind : std::uint8_t
```

What kind of run an auton is — which PERIOD it runs in (B15). Unset is refused.

*enum class, declared at [`include/shulib/competition/auton_registry.hpp:127`](../../include/shulib/competition/auton_registry.hpp#L127).*

<a id="autonkind-unset"></a>

### `AutonKind::Unset`

```cpp
Unset = 0
```

not stated — refused at registration

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:128`](../../include/shulib/competition/auton_registry.hpp#L128).*

<a id="autonkind-match"></a>

### `AutonKind::Match`

```cpp
Match = 1
```

a head-to-head match's autonomous period

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:129`](../../include/shulib/competition/auton_registry.hpp#L129).*

<a id="autonkind-skills"></a>

### `AutonKind::Skills`

```cpp
Skills = 2
```

an autonomous coding skills run

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:130`](../../include/shulib/competition/auton_registry.hpp#L130).*

<a id="autonkindword"></a>

## `autonKindWord`

```cpp
[[nodiscard]] constexpr const char* autonKindWord(AutonKind k) noexcept
```

The kind as a word for screens and logs ("MATCH", "SKILLS"; "UNSET"; "UNKNOWN" for a bad byte).

*free function, declared at [`include/shulib/competition/auton_registry.hpp:134`](../../include/shulib/competition/auton_registry.hpp#L134).*

<a id="struct-autonperiods"></a>

## `struct AutonPeriods`

```cpp
struct AutonPeriods
```

The two periods an auton can run in — the PROGRAM's constants, from the game manual (the COMP brief's C6 records the 2026-27 Override numbers: a 30 s match autonomous, a 60 s skills run). REQUIRED: both default to 0, which validate() refuses, because a library default here would be an invented match length (run_guard.hpp's banner — the HA-51 cautionary tale).

*struct, declared at [`include/shulib/competition/auton_registry.hpp:147`](../../include/shulib/competition/auton_registry.hpp#L147).*

<a id="autonperiods-match"></a>

### `AutonPeriods::match`

```cpp
units::Time match{0.0}
```

The match autonomous period.

*field, declared at [`include/shulib/competition/auton_registry.hpp:149`](../../include/shulib/competition/auton_registry.hpp#L149).*

<a id="autonperiods-skills"></a>

### `AutonPeriods::skills`

```cpp
units::Time skills{0.0}
```

The autonomous skills period.

*field, declared at [`include/shulib/competition/auton_registry.hpp:151`](../../include/shulib/competition/auton_registry.hpp#L151).*

<a id="autonperiods-validate"></a>

### `AutonPeriods::validate`

```cpp
void validate() const
```

Refuse a period that could not mean anything: both finite and > 0.

*function, declared at [`include/shulib/competition/auton_registry.hpp:154`](../../include/shulib/competition/auton_registry.hpp#L154).*

<a id="autonperiods-periodfor"></a>

### `AutonPeriods::periodFor`

```cpp
[[nodiscard]] units::Time periodFor(AutonKind kind) const noexcept
```

The period for `kind` — 0 for Unset (which no valid entry has).

*function, declared at [`include/shulib/competition/auton_registry.hpp:162`](../../include/shulib/competition/auton_registry.hpp#L162).*

<a id="class-startpose"></a>

## `class StartPose`

```cpp
class StartPose
```

Where an auton starts (ruling C2): a FIELD pose, or RELATIVE — wherever the robot is placed becomes (0, 0) facing 0. Either way the auton runner re-seats position AND heading at the instant the auton starts. Default-constructed it is UNSET, which a routine entry may not be.

*class, declared at [`include/shulib/competition/auton_registry.hpp:175`](../../include/shulib/competition/auton_registry.hpp#L175).*

<a id="startpose-startpose"></a>

### `StartPose::StartPose`

```cpp
constexpr StartPose() noexcept = default
```

Unset.

*function, declared at [`include/shulib/competition/auton_registry.hpp:185`](../../include/shulib/competition/auton_registry.hpp#L185).*

<a id="startpose-field"></a>

### `StartPose::field`

```cpp
[[nodiscard]] static StartPose field(units::Length x, units::Length y, math::Angle heading)
```

The robot is placed at field pose (x, y) facing `heading`.

*function, declared at [`include/shulib/competition/auton_registry.hpp:188`](../../include/shulib/competition/auton_registry.hpp#L188).*

<a id="startpose-field-2"></a>

### `StartPose::field (overload 2)`

```cpp
[[nodiscard]] static StartPose field(const math::Pose2d& pose)
```

The same, from a pose.

*function, declared at [`include/shulib/competition/auton_registry.hpp:192`](../../include/shulib/competition/auton_registry.hpp#L192).*

<a id="startpose-relative"></a>

### `StartPose::relative`

```cpp
[[nodiscard]] static StartPose relative()
```

Wherever the robot is placed becomes (0, 0) facing 0.

*function, declared at [`include/shulib/competition/auton_registry.hpp:196`](../../include/shulib/competition/auton_registry.hpp#L196).*

<a id="startpose-mode"></a>

### `StartPose::mode`

```cpp
[[nodiscard]] Mode mode() const noexcept
```

Which kind of start this is.

*function, declared at [`include/shulib/competition/auton_registry.hpp:199`](../../include/shulib/competition/auton_registry.hpp#L199).*

<a id="startpose-pose"></a>

### `StartPose::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const noexcept
```

The pose the runner seats: the field pose, or (0, 0, 0) for RELATIVE (and for Unset).

*function, declared at [`include/shulib/competition/auton_registry.hpp:201`](../../include/shulib/competition/auton_registry.hpp#L201).*

<a id="startpose-isset"></a>

### `StartPose::isSet`

```cpp
[[nodiscard]] bool isSet() const noexcept
```

True unless Unset.

*function, declared at [`include/shulib/competition/auton_registry.hpp:203`](../../include/shulib/competition/auton_registry.hpp#L203).*

<a id="startpose-sameas"></a>

### `StartPose::sameAs`

```cpp
[[nodiscard]] bool sameAs(const StartPose& o) const noexcept
```

Same mode and the same pose (positions within 1e-9 in, headings within 1e-9 rad).

*function, declared at [`include/shulib/competition/auton_registry.hpp:206`](../../include/shulib/competition/auton_registry.hpp#L206).*

<a id="enum-class-startpose-mode"></a>

## `enum class StartPose::Mode`

```cpp
enum class Mode : std::uint8_t
```

Which of the three it is.

*enum class, declared at [`include/shulib/competition/auton_registry.hpp:178`](../../include/shulib/competition/auton_registry.hpp#L178).*

<a id="startpose-mode-unset"></a>

### `StartPose::Mode::Unset`

```cpp
Unset = 0
```

not stated (a route entry may take its route's Start instead)

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:179`](../../include/shulib/competition/auton_registry.hpp#L179).*

<a id="startpose-mode-field"></a>

### `StartPose::Mode::Field`

```cpp
Field = 1
```

a known field pose

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:180`](../../include/shulib/competition/auton_registry.hpp#L180).*

<a id="startpose-mode-relative"></a>

### `StartPose::Mode::Relative`

```cpp
Relative = 2
```

(0, 0) facing 0, wherever the robot is placed

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:181`](../../include/shulib/competition/auton_registry.hpp#L181).*

<a id="startmodeword"></a>

## `startModeWord`

```cpp
[[nodiscard]] constexpr const char* startModeWord(StartPose::Mode m) noexcept
```

The start mode as a word ("FIELD", "RELATIVE", "UNSET").

*free function, declared at [`include/shulib/competition/auton_registry.hpp:219`](../../include/shulib/competition/auton_registry.hpp#L219).*

<a id="struct-autoncontext"></a>

## `struct AutonContext`

```cpp
struct AutonContext
```

What a routine gets to work with while it runs. Built by the runner for one run; every reference is valid for exactly that run.

*struct, declared at [`include/shulib/competition/auton_registry.hpp:232`](../../include/shulib/competition/auton_registry.hpp#L232).*

<a id="autoncontext-chassis"></a>

### `AutonContext::chassis`

```cpp
chassis::Chassis& chassis
```

The facade — every verb an auton is written with.

*field, declared at [`include/shulib/competition/auton_registry.hpp:234`](../../include/shulib/competition/auton_registry.hpp#L234).*

<a id="autoncontext-guard"></a>

### `AutonContext::guard`

```cpp
sequence::RunGuard& guard
```

The run's guard. Its waitFor()/pause() are the only waits the deadline (and a mid-auton disable) can cut: a frozen wait (Chassis::wait, Routine::pause) runs to its own end first.

*field, declared at [`include/shulib/competition/auton_registry.hpp:237`](../../include/shulib/competition/auton_registry.hpp#L237).*

<a id="autoncontext-telemetry"></a>

### `AutonContext::telemetry`

```cpp
hal::ITelemetrySink& telemetry
```

Where the auton's own log lines go.

*field, declared at [`include/shulib/competition/auton_registry.hpp:239`](../../include/shulib/competition/auton_registry.hpp#L239).*

<a id="autoncontext-mechanisms"></a>

### `AutonContext::mechanisms`

```cpp
std::span<hal::IMechanism* const> mechanisms
```

The mechanisms the guard SAFES at the cut and at the end — the Robot's PRESENT ones (B9). For the guard's safing only (B25): an auton reaches its mechanisms through `mechanismService` below (`Routine r{ctx, "name"}`), each by its declaration — never from this list.

*field, declared at [`include/shulib/competition/auton_registry.hpp:244`](../../include/shulib/competition/auton_registry.hpp#L244).*

<a id="autoncontext-entry"></a>

### `AutonContext::entry`

```cpp
const AutonEntry& entry
```

The entry being run.

*field, declared at [`include/shulib/competition/auton_registry.hpp:246`](../../include/shulib/competition/auton_registry.hpp#L246).*

<a id="autoncontext-mechanismservice"></a>

### `AutonContext::mechanismService`

```cpp
mechanism::MechanismService* mechanismService = nullptr
```

The robot's MECHANISM SERVICE (mechanism/service.hpp) — THE LOOKUP an auton reaches its mechanisms through: `Routine r{ctx, "score-three"}` takes it, and every mechanism step finds its mechanism there by its declaration (`r.move(arm["score"], 1.5_s)`). nullptr in a program that runs autons without one (COMP-a's runAuton): a mechanism step then refuses, loudly. (APPENDED at chunk MECH-A4 with a default, so COMP-a's positional `AutonContext{chassis, guard, telemetry, mechanisms, entry}` still compiles.)

*field, declared at [`include/shulib/competition/auton_registry.hpp:253`](../../include/shulib/competition/auton_registry.hpp#L253).*

<a id="autonroutine"></a>

## `AutonRoutine`

```cpp
using AutonRoutine = chassis::RoutineResult (*)(AutonContext&)
```

An auton's routine: runs the auton and returns the Routine's verdict (a bare ExitReason would lose where and why it stopped — routine.hpp's argument).

*type alias, declared at [`include/shulib/competition/auton_registry.hpp:258`](../../include/shulib/competition/auton_registry.hpp#L258).*

<a id="autonendaction"></a>

## `AutonEndAction`

```cpp
using AutonEndAction = bool (*)(AutonContext&)
```

An auton's end action — the park — run by the guard when scoring ends (run_guard.hpp). Returns success. nullptr means the default: `chassis.brake()`, Settled = success.

*type alias, declared at [`include/shulib/competition/auton_registry.hpp:262`](../../include/shulib/competition/auton_registry.hpp#L262).*

<a id="struct-autonentry"></a>

## `struct AutonEntry`

```cpp
struct AutonEntry
```

One registered auton (file banner). Every field without a meaningful default is refused when unset, at registration.

*struct, declared at [`include/shulib/competition/auton_registry.hpp:266`](../../include/shulib/competition/auton_registry.hpp#L266).*

<a id="autonentry-name"></a>

### `AutonEntry::name`

```cpp
const char* name = nullptr
```

Shown on the screen and in the log: at most kMaxAutonNameChars printable ASCII, no '%', unique, never "NONE". BORROWED — a string literal.

*field, declared at [`include/shulib/competition/auton_registry.hpp:269`](../../include/shulib/competition/auton_registry.hpp#L269).*

<a id="autonentry-kind"></a>

### `AutonEntry::kind`

```cpp
AutonKind kind = AutonKind::Unset
```

MATCH or SKILLS: which period it runs in. Unset is refused.

*field, declared at [`include/shulib/competition/auton_registry.hpp:271`](../../include/shulib/competition/auton_registry.hpp#L271).*

<a id="autonentry-routine"></a>

### `AutonEntry::routine`

```cpp
AutonRoutine routine = nullptr
```

The routine to run — EXACTLY one of `routine` / `route` is set.

*field, declared at [`include/shulib/competition/auton_registry.hpp:273`](../../include/shulib/competition/auton_registry.hpp#L273).*

<a id="autonentry-route"></a>

### `AutonEntry::route`

```cpp
const config::Route* route = nullptr
```

The route to run through a runner::PathRunner — BORROWED, must outlive the registry. A route entry needs a runner::CommandRegistry at run time (the runner refuses without one).

*field, declared at [`include/shulib/competition/auton_registry.hpp:276`](../../include/shulib/competition/auton_registry.hpp#L276).*

<a id="autonentry-start"></a>

### `AutonEntry::start`

```cpp
StartPose start{}
```

Where the robot starts (file banner: FIELD or RELATIVE; a route may take its Start's).

*field, declared at [`include/shulib/competition/auton_registry.hpp:278`](../../include/shulib/competition/auton_registry.hpp#L278).*

<a id="autonentry-endactionat"></a>

### `AutonEntry::endActionAt`

```cpp
units::Time endActionAt{0.0}
```

When scoring stops and the end action starts, from the auton's start. 0 = UNSET, refused.

*field, declared at [`include/shulib/competition/auton_registry.hpp:280`](../../include/shulib/competition/auton_registry.hpp#L280).*

<a id="autonentry-hardstopat"></a>

### `AutonEntry::hardStopAt`

```cpp
units::Time hardStopAt{0.0}
```

The unconditional safe floor, from the auton's start. 0 = UNSET, refused; must be >= endActionAt and within the kind's period.

*field, declared at [`include/shulib/competition/auton_registry.hpp:283`](../../include/shulib/competition/auton_registry.hpp#L283).*

<a id="autonentry-endaction"></a>

### `AutonEntry::endAction`

```cpp
AutonEndAction endAction = nullptr
```

The park. nullptr = `chassis.brake()`.

*field, declared at [`include/shulib/competition/auton_registry.hpp:285`](../../include/shulib/competition/auton_registry.hpp#L285).*

<a id="autonentry-mirror"></a>

### `AutonEntry::mirror`

```cpp
const FieldMirror* mirror = nullptr
```

MIRROR1-A: the field mirror this entry runs under — robot two's run of robot one's route — or nullptr, the entry as written (the default). Set by `competition::mirrored(entry, mirror)` (competition/field_mirror.hpp), never by hand; BORROWED (a season constant). A mirrored name is at most kMaxMirroredAutonNameChars. (APPENDED last, so every existing initializer compiles.)

*field, declared at [`include/shulib/competition/auton_registry.hpp:290`](../../include/shulib/competition/auton_registry.hpp#L290).*

<a id="autonentry-effectivestart"></a>

### `AutonEntry::effectiveStart`

```cpp
[[nodiscard]] StartPose effectiveStart() const
```

The start pose the runner seats: `start`, or — for a route entry that left it unset — the route's Start waypoint as a FIELD pose (B16), when that waypoint STATES its heading (HeadingMode::Fixed). Unset when neither says, which includes a route whose Start is `Free`: it states no heading, and the seat will not invent one (file banner, Routes).

*function, declared at [`include/shulib/competition/auton_registry.hpp:296`](../../include/shulib/competition/auton_registry.hpp#L296).*

<a id="struct-autonverdict"></a>

## `struct AutonVerdict`

```cpp
struct AutonVerdict
```

A registration verdict: one bool and the sentence a person reads — naming the auton first.

*struct, declared at [`include/shulib/competition/auton_registry.hpp:308`](../../include/shulib/competition/auton_registry.hpp#L308).*

<a id="autonverdict-ok"></a>

### `AutonVerdict::ok`

```cpp
bool ok = false
```

True when the entry may be registered / run.

*field, declared at [`include/shulib/competition/auton_registry.hpp:310`](../../include/shulib/competition/auton_registry.hpp#L310).*

<a id="autonverdict-why"></a>

### `AutonVerdict::why`

```cpp
char why[192]{}
```

The FIRST thing wrong, beginning "auton '<name>': ". Empty when `ok`.

*field, declared at [`include/shulib/competition/auton_registry.hpp:312`](../../include/shulib/competition/auton_registry.hpp#L312).*

<a id="checkautonentry"></a>

## `checkAutonEntry`

```cpp
[[nodiscard]] inline AutonVerdict checkAutonEntry(const AutonEntry& e, units::Time period) noexcept
```

Can `e` run in a period of `period`? PURE and TOTAL (never throws, never allocates). The checks, in order: the name (present, printable ASCII, no '%', within kMaxAutonNameChars, not "NONE"; a mirrored entry's within kMaxMirroredAutonNameChars); the kind; exactly one of routine/route; a route that is itself valid; the start pose (B16's rules for a route; set, for a routine) and its finiteness; the schedule (stated, finite, endActionAt > 0, hardStopAt >= endActionAt); and hardStopAt within `period`. It does NOT check uniqueness (the registry does) or a route's command registry (the runner does, at run time).

*free function, declared at [`include/shulib/competition/auton_registry.hpp:321`](../../include/shulib/competition/auton_registry.hpp#L321).*

<a id="class-autonregistry"></a>

## `class AutonRegistry`

```cpp
class AutonRegistry
```

The autons a program can run: fixed capacity, filled by explicit add() calls from `initialize()` (file banner), read by the selector and the runner. Single-task while it is being filled; read-only afterwards, so any task may read it once registration is over.

*class, declared at [`include/shulib/competition/auton_registry.hpp:428`](../../include/shulib/competition/auton_registry.hpp#L428).*

<a id="autonregistry-autonregistry"></a>

### `AutonRegistry::AutonRegistry`

```cpp
explicit AutonRegistry(const AutonPeriods& periods, OnRefusal onRefusal = OnRefusal::Throw)
```

`periods` are the PROGRAM's two constants (REQUIRED — AutonPeriods::validate()). `onRefusal`: what add() does with an entry check() refuses (file banner: A refused entry) — throw (the default), or drop just that entry and keep its sentence.

*function, declared at [`include/shulib/competition/auton_registry.hpp:433`](../../include/shulib/competition/auton_registry.hpp#L433).*

<a id="autonregistry-check"></a>

### `AutonRegistry::check`

```cpp
[[nodiscard]] AutonVerdict check(const AutonEntry& e) const noexcept
```

Would `e` be accepted? checkAutonEntry() against its kind's period, then uniqueness and capacity. Pure: registers nothing.

*function, declared at [`include/shulib/competition/auton_registry.hpp:440`](../../include/shulib/competition/auton_registry.hpp#L440).*

<a id="autonregistry-add"></a>

### `AutonRegistry::add`

```cpp
void add(const AutonEntry& e)
```

Register `e` (COPIED; its name and route stay borrowed) when check(e) passes. When it does not: under OnRefusal::Throw (the default) a PreconditionError carrying check()'s sentence, which names the auton, so a program's `initialize()` try block can log it and refuse the boot by name (ruling B15: refusals happen at REGISTRATION, not in a match); under OnRefusal::DropEntry nothing is registered, the sentence is kept (refusal()) and counted (refusedCount()), and add() returns — never throws — so the next entry is registered as if the refused one had never been written (the team lead's ruling of 2026-10-02).

*function, declared at [`include/shulib/competition/auton_registry.hpp:466`](../../include/shulib/competition/auton_registry.hpp#L466).*

<a id="autonregistry-onrefusal"></a>

### `AutonRegistry::onRefusal`

```cpp
[[nodiscard]] OnRefusal onRefusal() const noexcept
```

What add() does with a refused entry (the constructor's choice).

*function, declared at [`include/shulib/competition/auton_registry.hpp:480`](../../include/shulib/competition/auton_registry.hpp#L480).*

<a id="autonregistry-refusedcount"></a>

### `AutonRegistry::refusedCount`

```cpp
[[nodiscard]] std::size_t refusedCount() const noexcept
```

How many entries add() DROPPED (OnRefusal::DropEntry) — every one, kept or not. Always 0 under OnRefusal::Throw, where a refusal throws instead.

*function, declared at [`include/shulib/competition/auton_registry.hpp:483`](../../include/shulib/competition/auton_registry.hpp#L483).*

<a id="autonregistry-refusal"></a>

### `AutonRegistry::refusal`

```cpp
[[nodiscard]] const char* refusal(std::size_t i) const noexcept
```

The sentence of dropped entry `i`, in the order add() met them — check()'s, beginning "auton '<name>': " — for `i` below both refusedCount() and kMaxAutonRefusals; "" otherwise.

*function, declared at [`include/shulib/competition/auton_registry.hpp:486`](../../include/shulib/competition/auton_registry.hpp#L486).*

<a id="autonregistry-size"></a>

### `AutonRegistry::size`

```cpp
[[nodiscard]] std::size_t size() const noexcept
```

How many autons are registered.

*function, declared at [`include/shulib/competition/auton_registry.hpp:491`](../../include/shulib/competition/auton_registry.hpp#L491).*

<a id="autonregistry-empty"></a>

### `AutonRegistry::empty`

```cpp
[[nodiscard]] bool empty() const noexcept
```

True when none are.

*function, declared at [`include/shulib/competition/auton_registry.hpp:493`](../../include/shulib/competition/auton_registry.hpp#L493).*

<a id="autonregistry-at"></a>

### `AutonRegistry::at`

```cpp
[[nodiscard]] const AutonEntry& at(std::size_t index) const
```

The auton at `index` (registration order). Precondition: index < size().

*function, declared at [`include/shulib/competition/auton_registry.hpp:495`](../../include/shulib/competition/auton_registry.hpp#L495).*

<a id="autonregistry-find"></a>

### `AutonRegistry::find`

```cpp
[[nodiscard]] const AutonEntry* find(std::string_view name) const noexcept
```

The auton called `name`, or nullptr.

*function, declared at [`include/shulib/competition/auton_registry.hpp:500`](../../include/shulib/competition/auton_registry.hpp#L500).*

<a id="autonregistry-periods"></a>

### `AutonRegistry::periods`

```cpp
[[nodiscard]] const AutonPeriods& periods() const noexcept
```

The program's two periods, as given.

*function, declared at [`include/shulib/competition/auton_registry.hpp:512`](../../include/shulib/competition/auton_registry.hpp#L512).*

<a id="autonregistry-periodfor"></a>

### `AutonRegistry::periodFor`

```cpp
[[nodiscard]] units::Time periodFor(const AutonEntry& e) const noexcept
```

The period `e` runs in.

*function, declared at [`include/shulib/competition/auton_registry.hpp:514`](../../include/shulib/competition/auton_registry.hpp#L514).*

<a id="autoncontext"></a>

## `AutonContext`

```cpp
using competition::AutonContext
```

What an auton is handed: `RoutineResult scoreThree(AutonContext& ctx)`.

*type alias, declared at [`include/shulib/competition/auton_registry.hpp:533`](../../include/shulib/competition/auton_registry.hpp#L533).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 72 lines, click to expand</summary>

```text

 competition::AutonRegistry — the autons a competition program can run, as DATA (chunk COMP-a,
 2026-09-26; the COMP brief's rulings C2, C3, C6 and amendments B15, B16; OnRefusal::DropEntry at
 T4 Part 3, the team lead's ruling of 2026-10-02 on COMP-b's C5).

 ── What an auton is here ───────────────────────────────────────────────────────────
 An AutonEntry is a row of data the STUDENTS' file registers, from `initialize()`, by an explicit
 call — never at static-initialization time (the file is compiled into every build, and a
 constructor at namespace scope runs before the on-robot precondition handler exists, in an
 order across translation units nobody controls). A row names the auton, says whether it is a
 MATCH auton or a SKILLS run, points at EITHER a routine function OR a config::Route, declares
 where the robot starts, and states its schedule. No field has a default that means something:
 every "unset" is refused, by name, when the row is registered.

 ── The start pose: FIELD or RELATIVE, re-seated when the auton starts (ruling C2) ──
 `StartPose::field(x, y, heading)` — the robot is placed at a known field pose. `StartPose::
 relative()` — wherever the robot is placed becomes (0, 0) facing 0. Either way the auton runner
 (match_runner.hpp) RE-SEATS the pose — position AND heading — at the instant the auton starts,
 which also discards whatever the IMU drifted while the robot waited. The IMU's boot heading is
 never the start heading (hardware-assumptions HA-05, amended at COMP-a).

 ── Routes (B16): the route's Start waypoint IS the start pose ──────────────────────
 A route whose waypoint 0 is a `Start` already says where the robot starts. So a route entry may
 leave `start` unset (it takes the route's), or state the SAME field pose; a DIFFERENT one — or
 RELATIVE — is refused, because the route would setPose() to its own Start one tick after the
 runner re-seated somewhere else. A route with no Start takes the entry's, which must then be set.
 The Start's HEADING is the start heading only when the waypoint STATES it — HeadingMode::Fixed,
 the one mode under which config::Waypoint documents `heading` as read. A `Free` Start says "the
 robot begins at (x, y)" and nothing about which way it faces, so the seat will not read its
 `heading` field (a default 0 there would otherwise be seated silently): such an entry must
 declare its own FIELD start at the Start's (x, y), which supplies the heading, or be refused.
 (A decision-consistency check against F8, COMP-a's review: before B16 no code read a Start's
 heading at all — PathRunner's Start leg seeds position only — so this rule is where the
 field's meaning is settled, and it keeps route.hpp's "read only when Fixed" true.)

 ── The schedule (C6, B15): stated, never defaulted, bounded by the period ──────────
 `endActionAt` / `hardStopAt` are sequence::RunGuardConfig's two instants, measured from the
 auton's start. They default to 0 — UNSET — and an unset or nonsensical schedule is refused
 here, before a match rather than in one. The PERIOD an entry runs in follows from its KIND and
 the PROGRAM's two constants (AutonPeriods — the library states no match length anywhere,
 run_guard.hpp's banner); `hardStopAt` past the period is refused. A margin INSIDE the period is
 recommended (the guide will say so): the period is when the field disables the robot, and a
 hard stop that lands on it is a hard stop that never had a tick to act.

 ── A refused entry: the boot, or just that auton (the team lead's ruling of 2026-10-02) ──
 By default add() THROWS on an entry check() refuses (B15: refusals happen at registration, not in
 a match), so a program's initialize() refuses its boot by name. The team lead ruled otherwise for
 the competition program (COMP-b's review, its C5: "drop just that auton" — an auton bug must not
 cost the match's driver period), and the Auton Tester, which registers through the same function,
 does the same (the coordinator's extension of the ruling, T4 Part 3; the team lead confirmed it
 2026-10-08): a registry built with
 OnRefusal::DropEntry registers NOTHING for a refused entry, keeps check()'s sentence (refusal(),
 which names the auton first), and RETURNS — so the students' registration function goes on to its
 next entry, the refused one is never on the menu (AutonMenu names it there instead), and the
 program logs every sentence. The rules themselves are unchanged: the same check(), the same
 sentences.

 ── Names (B25) ─────────────────────────────────────────────────────────────────────
 A name is shown as "AUTON: <name>" on ONE medium line of the brain screen, so it is at most
 kMaxAutonNameChars = ui::kMediumCols - 7 = 29 printable ASCII characters, no '%', unique, and
 never "NONE" (the selector's own choice). The name pointer is BORROWED — a string literal. A
 MIRRORED entry's (below) is at most kMaxMirroredAutonNameChars = 25: its menu button reads
 "<name> SKILLS MIR" on one medium line.

 ── A mirrored entry (chunk MIRROR1-A, 2026-11-13) ──────────────────────────────────────────────
 `mirror` (APPENDED, default none) marks an entry robot two runs MIRRORED — robot one's route,
 reflected across the field's mirror line. A team never sets it by hand: `competition::mirrored(entry,
 override2026::kSkillsMirror)` (competition/field_mirror.hpp) copies an entry and sets it, and the
 auton runner seats the reflected start and runs the routine inside a chassis::MirroredRun. This file
 only names the type and checks the name budget; field_mirror.hpp says everything else.

 PROS-free, allocation-free (fixed capacity), no std::function. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
