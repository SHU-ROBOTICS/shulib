<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/auton_registry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `auton_registry.hpp`

competition::AutonRegistry — the autons a competition program can run, as DATA.

This header declares **8** types (43 members), **3** free functions, **3** constants, and **2** type aliass.

Extracted from [`include/shulib/competition/auton_registry.hpp`](../../include/shulib/competition/auton_registry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxAutons`](#kmaxautons) — *constant*
- [`kMaxAutonNameChars`](#kmaxautonnamechars) — *constant*
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
  - [`effectiveStart`](#autonentry-effectivestart)
- [`struct AutonVerdict`](#struct-autonverdict)
  - [`ok`](#autonverdict-ok)
  - [`why`](#autonverdict-why)
- [`checkAutonEntry`](#checkautonentry) — *free function*
- [`class AutonRegistry`](#class-autonregistry)
  - [`AutonRegistry`](#autonregistry-autonregistry)
  - [`check`](#autonregistry-check)
  - [`add`](#autonregistry-add)
  - [`size`](#autonregistry-size)
  - [`empty`](#autonregistry-empty)
  - [`at`](#autonregistry-at)
  - [`find`](#autonregistry-find)
  - [`periods`](#autonregistry-periods)
  - [`periodFor`](#autonregistry-periodfor)

<a id="kmaxautons"></a>

## `kMaxAutons`

```cpp
inline constexpr std::size_t kMaxAutons = 16
```

The most autons one registry holds. Fixed capacity so registration never allocates; sixteen is several pages of the selector and more than a season has ever needed.

*constant, declared at [`include/shulib/competition/auton_registry.hpp:76`](../../include/shulib/competition/auton_registry.hpp#L76).*

<a id="kmaxautonnamechars"></a>

## `kMaxAutonNameChars`

```cpp
inline constexpr std::size_t kMaxAutonNameChars = ui::kMediumCols - 7
```

The longest auton name: "AUTON: " (7 characters) plus the name must fit ONE medium line (B25).

*constant, declared at [`include/shulib/competition/auton_registry.hpp:79`](../../include/shulib/competition/auton_registry.hpp#L79).*

<a id="knoautonname"></a>

## `kNoAutonName`

```cpp
inline constexpr std::string_view kNoAutonName = "NONE"
```

The selector's reserved choice. No registered auton may be called this.

*constant, declared at [`include/shulib/competition/auton_registry.hpp:83`](../../include/shulib/competition/auton_registry.hpp#L83).*

<a id="enum-class-autonkind"></a>

## `enum class AutonKind`

```cpp
enum class AutonKind : std::uint8_t
```

What kind of run an auton is — which PERIOD it runs in (B15). Unset is refused.

*enum class, declared at [`include/shulib/competition/auton_registry.hpp:86`](../../include/shulib/competition/auton_registry.hpp#L86).*

<a id="autonkind-unset"></a>

### `AutonKind::Unset`

```cpp
Unset = 0
```

not stated — refused at registration

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:87`](../../include/shulib/competition/auton_registry.hpp#L87).*

<a id="autonkind-match"></a>

### `AutonKind::Match`

```cpp
Match = 1
```

a head-to-head match's autonomous period

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:88`](../../include/shulib/competition/auton_registry.hpp#L88).*

<a id="autonkind-skills"></a>

### `AutonKind::Skills`

```cpp
Skills = 2
```

an autonomous coding skills run

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:89`](../../include/shulib/competition/auton_registry.hpp#L89).*

<a id="autonkindword"></a>

## `autonKindWord`

```cpp
[[nodiscard]] constexpr const char* autonKindWord(AutonKind k) noexcept
```

The kind as a word for screens and logs ("MATCH", "SKILLS"; "UNSET"; "UNKNOWN" for a bad byte).

*free function, declared at [`include/shulib/competition/auton_registry.hpp:93`](../../include/shulib/competition/auton_registry.hpp#L93).*

<a id="struct-autonperiods"></a>

## `struct AutonPeriods`

```cpp
struct AutonPeriods
```

The two periods an auton can run in — the PROGRAM's constants, from the game manual (the COMP brief's C6 records the 2026-27 Override numbers: a 30 s match autonomous, a 60 s skills run). REQUIRED: both default to 0, which validate() refuses, because a library default here would be an invented match length (run_guard.hpp's banner — the HA-51 cautionary tale).

*struct, declared at [`include/shulib/competition/auton_registry.hpp:106`](../../include/shulib/competition/auton_registry.hpp#L106).*

<a id="autonperiods-match"></a>

### `AutonPeriods::match`

```cpp
units::Time match{0.0}
```

The match autonomous period.

*field, declared at [`include/shulib/competition/auton_registry.hpp:108`](../../include/shulib/competition/auton_registry.hpp#L108).*

<a id="autonperiods-skills"></a>

### `AutonPeriods::skills`

```cpp
units::Time skills{0.0}
```

The autonomous skills period.

*field, declared at [`include/shulib/competition/auton_registry.hpp:110`](../../include/shulib/competition/auton_registry.hpp#L110).*

<a id="autonperiods-validate"></a>

### `AutonPeriods::validate`

```cpp
void validate() const
```

Refuse a period that could not mean anything: both finite and > 0.

*function, declared at [`include/shulib/competition/auton_registry.hpp:113`](../../include/shulib/competition/auton_registry.hpp#L113).*

<a id="autonperiods-periodfor"></a>

### `AutonPeriods::periodFor`

```cpp
[[nodiscard]] units::Time periodFor(AutonKind kind) const noexcept
```

The period for `kind` — 0 for Unset (which no valid entry has).

*function, declared at [`include/shulib/competition/auton_registry.hpp:121`](../../include/shulib/competition/auton_registry.hpp#L121).*

<a id="class-startpose"></a>

## `class StartPose`

```cpp
class StartPose
```

Where an auton starts (ruling C2): a FIELD pose, or RELATIVE — wherever the robot is placed becomes (0, 0) facing 0. Either way the auton runner re-seats position AND heading at the instant the auton starts. Default-constructed it is UNSET, which a routine entry may not be.

*class, declared at [`include/shulib/competition/auton_registry.hpp:134`](../../include/shulib/competition/auton_registry.hpp#L134).*

<a id="startpose-startpose"></a>

### `StartPose::StartPose`

```cpp
constexpr StartPose() noexcept = default
```

Unset.

*function, declared at [`include/shulib/competition/auton_registry.hpp:144`](../../include/shulib/competition/auton_registry.hpp#L144).*

<a id="startpose-field"></a>

### `StartPose::field`

```cpp
[[nodiscard]] static StartPose field(units::Length x, units::Length y, math::Angle heading)
```

The robot is placed at field pose (x, y) facing `heading`.

*function, declared at [`include/shulib/competition/auton_registry.hpp:147`](../../include/shulib/competition/auton_registry.hpp#L147).*

<a id="startpose-field-2"></a>

### `StartPose::field (overload 2)`

```cpp
[[nodiscard]] static StartPose field(const math::Pose2d& pose)
```

The same, from a pose.

*function, declared at [`include/shulib/competition/auton_registry.hpp:151`](../../include/shulib/competition/auton_registry.hpp#L151).*

<a id="startpose-relative"></a>

### `StartPose::relative`

```cpp
[[nodiscard]] static StartPose relative()
```

Wherever the robot is placed becomes (0, 0) facing 0.

*function, declared at [`include/shulib/competition/auton_registry.hpp:155`](../../include/shulib/competition/auton_registry.hpp#L155).*

<a id="startpose-mode"></a>

### `StartPose::mode`

```cpp
[[nodiscard]] Mode mode() const noexcept
```

Which kind of start this is.

*function, declared at [`include/shulib/competition/auton_registry.hpp:158`](../../include/shulib/competition/auton_registry.hpp#L158).*

<a id="startpose-pose"></a>

### `StartPose::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const noexcept
```

The pose the runner seats: the field pose, or (0, 0, 0) for RELATIVE (and for Unset).

*function, declared at [`include/shulib/competition/auton_registry.hpp:160`](../../include/shulib/competition/auton_registry.hpp#L160).*

<a id="startpose-isset"></a>

### `StartPose::isSet`

```cpp
[[nodiscard]] bool isSet() const noexcept
```

True unless Unset.

*function, declared at [`include/shulib/competition/auton_registry.hpp:162`](../../include/shulib/competition/auton_registry.hpp#L162).*

<a id="startpose-sameas"></a>

### `StartPose::sameAs`

```cpp
[[nodiscard]] bool sameAs(const StartPose& o) const noexcept
```

Same mode and the same pose (positions within 1e-9 in, headings within 1e-9 rad).

*function, declared at [`include/shulib/competition/auton_registry.hpp:165`](../../include/shulib/competition/auton_registry.hpp#L165).*

<a id="enum-class-startpose-mode"></a>

## `enum class StartPose::Mode`

```cpp
enum class Mode : std::uint8_t
```

Which of the three it is.

*enum class, declared at [`include/shulib/competition/auton_registry.hpp:137`](../../include/shulib/competition/auton_registry.hpp#L137).*

<a id="startpose-mode-unset"></a>

### `StartPose::Mode::Unset`

```cpp
Unset = 0
```

not stated (a route entry may take its route's Start instead)

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:138`](../../include/shulib/competition/auton_registry.hpp#L138).*

<a id="startpose-mode-field"></a>

### `StartPose::Mode::Field`

```cpp
Field = 1
```

a known field pose

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:139`](../../include/shulib/competition/auton_registry.hpp#L139).*

<a id="startpose-mode-relative"></a>

### `StartPose::Mode::Relative`

```cpp
Relative = 2
```

(0, 0) facing 0, wherever the robot is placed

*enumerator, declared at [`include/shulib/competition/auton_registry.hpp:140`](../../include/shulib/competition/auton_registry.hpp#L140).*

<a id="startmodeword"></a>

## `startModeWord`

```cpp
[[nodiscard]] constexpr const char* startModeWord(StartPose::Mode m) noexcept
```

The start mode as a word ("FIELD", "RELATIVE", "UNSET").

*free function, declared at [`include/shulib/competition/auton_registry.hpp:178`](../../include/shulib/competition/auton_registry.hpp#L178).*

<a id="struct-autoncontext"></a>

## `struct AutonContext`

```cpp
struct AutonContext
```

What a routine gets to work with while it runs. Built by the runner for one run; every reference is valid for exactly that run.

*struct, declared at [`include/shulib/competition/auton_registry.hpp:191`](../../include/shulib/competition/auton_registry.hpp#L191).*

<a id="autoncontext-chassis"></a>

### `AutonContext::chassis`

```cpp
chassis::Chassis& chassis
```

The facade — every verb an auton is written with.

*field, declared at [`include/shulib/competition/auton_registry.hpp:193`](../../include/shulib/competition/auton_registry.hpp#L193).*

<a id="autoncontext-guard"></a>

### `AutonContext::guard`

```cpp
sequence::RunGuard& guard
```

The run's guard. Its waitFor()/pause() are the only waits the deadline (and a mid-auton disable) can cut: a frozen wait (Chassis::wait, Routine::pause) runs to its own end first.

*field, declared at [`include/shulib/competition/auton_registry.hpp:196`](../../include/shulib/competition/auton_registry.hpp#L196).*

<a id="autoncontext-telemetry"></a>

### `AutonContext::telemetry`

```cpp
hal::ITelemetrySink& telemetry
```

Where the auton's own log lines go.

*field, declared at [`include/shulib/competition/auton_registry.hpp:198`](../../include/shulib/competition/auton_registry.hpp#L198).*

<a id="autoncontext-mechanisms"></a>

### `AutonContext::mechanisms`

```cpp
std::span<hal::IMechanism* const> mechanisms
```

The mechanisms the guard SAFES at the cut and at the end — the Robot's PRESENT ones (B9). For the guard's safing only (B25): a routine that commands a mechanism looks it up by name from the Robot, not from this list.

*field, declared at [`include/shulib/competition/auton_registry.hpp:202`](../../include/shulib/competition/auton_registry.hpp#L202).*

<a id="autoncontext-entry"></a>

### `AutonContext::entry`

```cpp
const AutonEntry& entry
```

The entry being run.

*field, declared at [`include/shulib/competition/auton_registry.hpp:204`](../../include/shulib/competition/auton_registry.hpp#L204).*

<a id="autonroutine"></a>

## `AutonRoutine`

```cpp
using AutonRoutine = chassis::RoutineResult (*)(AutonContext&)
```

An auton's routine: runs the auton and returns the Routine's verdict (a bare ExitReason would lose where and why it stopped — routine.hpp's argument).

*type alias, declared at [`include/shulib/competition/auton_registry.hpp:209`](../../include/shulib/competition/auton_registry.hpp#L209).*

<a id="autonendaction"></a>

## `AutonEndAction`

```cpp
using AutonEndAction = bool (*)(AutonContext&)
```

An auton's end action — the park — run by the guard when scoring ends (run_guard.hpp). Returns success. nullptr means the default: `chassis.brake()`, Settled = success.

*type alias, declared at [`include/shulib/competition/auton_registry.hpp:213`](../../include/shulib/competition/auton_registry.hpp#L213).*

<a id="struct-autonentry"></a>

## `struct AutonEntry`

```cpp
struct AutonEntry
```

One registered auton (file banner). Every field without a meaningful default is refused when unset, at registration.

*struct, declared at [`include/shulib/competition/auton_registry.hpp:217`](../../include/shulib/competition/auton_registry.hpp#L217).*

<a id="autonentry-name"></a>

### `AutonEntry::name`

```cpp
const char* name = nullptr
```

Shown on the screen and in the log: at most kMaxAutonNameChars printable ASCII, no '%', unique, never "NONE". BORROWED — a string literal.

*field, declared at [`include/shulib/competition/auton_registry.hpp:220`](../../include/shulib/competition/auton_registry.hpp#L220).*

<a id="autonentry-kind"></a>

### `AutonEntry::kind`

```cpp
AutonKind kind = AutonKind::Unset
```

MATCH or SKILLS: which period it runs in. Unset is refused.

*field, declared at [`include/shulib/competition/auton_registry.hpp:222`](../../include/shulib/competition/auton_registry.hpp#L222).*

<a id="autonentry-routine"></a>

### `AutonEntry::routine`

```cpp
AutonRoutine routine = nullptr
```

The routine to run — EXACTLY one of `routine` / `route` is set.

*field, declared at [`include/shulib/competition/auton_registry.hpp:224`](../../include/shulib/competition/auton_registry.hpp#L224).*

<a id="autonentry-route"></a>

### `AutonEntry::route`

```cpp
const config::Route* route = nullptr
```

The route to run through a runner::PathRunner — BORROWED, must outlive the registry. A route entry needs a runner::CommandRegistry at run time (the runner refuses without one).

*field, declared at [`include/shulib/competition/auton_registry.hpp:227`](../../include/shulib/competition/auton_registry.hpp#L227).*

<a id="autonentry-start"></a>

### `AutonEntry::start`

```cpp
StartPose start{}
```

Where the robot starts (file banner: FIELD or RELATIVE; a route may take its Start's).

*field, declared at [`include/shulib/competition/auton_registry.hpp:229`](../../include/shulib/competition/auton_registry.hpp#L229).*

<a id="autonentry-endactionat"></a>

### `AutonEntry::endActionAt`

```cpp
units::Time endActionAt{0.0}
```

When scoring stops and the end action starts, from the auton's start. 0 = UNSET, refused.

*field, declared at [`include/shulib/competition/auton_registry.hpp:231`](../../include/shulib/competition/auton_registry.hpp#L231).*

<a id="autonentry-hardstopat"></a>

### `AutonEntry::hardStopAt`

```cpp
units::Time hardStopAt{0.0}
```

The unconditional safe floor, from the auton's start. 0 = UNSET, refused; must be >= endActionAt and within the kind's period.

*field, declared at [`include/shulib/competition/auton_registry.hpp:234`](../../include/shulib/competition/auton_registry.hpp#L234).*

<a id="autonentry-endaction"></a>

### `AutonEntry::endAction`

```cpp
AutonEndAction endAction = nullptr
```

The park. nullptr = `chassis.brake()`.

*field, declared at [`include/shulib/competition/auton_registry.hpp:236`](../../include/shulib/competition/auton_registry.hpp#L236).*

<a id="autonentry-effectivestart"></a>

### `AutonEntry::effectiveStart`

```cpp
[[nodiscard]] StartPose effectiveStart() const
```

The start pose the runner seats: `start`, or — for a route entry that left it unset — the route's Start waypoint as a FIELD pose (B16), when that waypoint STATES its heading (HeadingMode::Fixed). Unset when neither says, which includes a route whose Start is `Free`: it states no heading, and the seat will not invent one (file banner, Routes).

*function, declared at [`include/shulib/competition/auton_registry.hpp:242`](../../include/shulib/competition/auton_registry.hpp#L242).*

<a id="struct-autonverdict"></a>

## `struct AutonVerdict`

```cpp
struct AutonVerdict
```

A registration verdict: one bool and the sentence a person reads — naming the auton first.

*struct, declared at [`include/shulib/competition/auton_registry.hpp:254`](../../include/shulib/competition/auton_registry.hpp#L254).*

<a id="autonverdict-ok"></a>

### `AutonVerdict::ok`

```cpp
bool ok = false
```

True when the entry may be registered / run.

*field, declared at [`include/shulib/competition/auton_registry.hpp:256`](../../include/shulib/competition/auton_registry.hpp#L256).*

<a id="autonverdict-why"></a>

### `AutonVerdict::why`

```cpp
char why[192]{}
```

The FIRST thing wrong, beginning "auton '<name>': ". Empty when `ok`.

*field, declared at [`include/shulib/competition/auton_registry.hpp:258`](../../include/shulib/competition/auton_registry.hpp#L258).*

<a id="checkautonentry"></a>

## `checkAutonEntry`

```cpp
[[nodiscard]] inline AutonVerdict checkAutonEntry(const AutonEntry& e, units::Time period) noexcept
```

Can `e` run in a period of `period`? PURE and TOTAL (never throws, never allocates). The checks, in order: the name (present, printable ASCII, no '%', within kMaxAutonNameChars, not "NONE"); the kind; exactly one of routine/route; a route that is itself valid; the start pose (B16's rules for a route; set, for a routine) and its finiteness; the schedule (stated, finite, endActionAt > 0, hardStopAt >= endActionAt); and hardStopAt within `period`. It does NOT check uniqueness (the registry does) or a route's command registry (the runner does, at run time).

*free function, declared at [`include/shulib/competition/auton_registry.hpp:267`](../../include/shulib/competition/auton_registry.hpp#L267).*

<a id="class-autonregistry"></a>

## `class AutonRegistry`

```cpp
class AutonRegistry
```

The autons a program can run: fixed capacity, filled by explicit add() calls from `initialize()` (file banner), read by the selector and the runner. Single-task while it is being filled; read-only afterwards, so any task may read it once registration is over.

*class, declared at [`include/shulib/competition/auton_registry.hpp:369`](../../include/shulib/competition/auton_registry.hpp#L369).*

<a id="autonregistry-autonregistry"></a>

### `AutonRegistry::AutonRegistry`

```cpp
explicit AutonRegistry(const AutonPeriods& periods)
```

`periods` are the PROGRAM's two constants (REQUIRED — AutonPeriods::validate()).

*function, declared at [`include/shulib/competition/auton_registry.hpp:372`](../../include/shulib/competition/auton_registry.hpp#L372).*

<a id="autonregistry-check"></a>

### `AutonRegistry::check`

```cpp
[[nodiscard]] AutonVerdict check(const AutonEntry& e) const noexcept
```

Would `e` be accepted? checkAutonEntry() against its kind's period, then uniqueness and capacity. Pure: registers nothing.

*function, declared at [`include/shulib/competition/auton_registry.hpp:376`](../../include/shulib/competition/auton_registry.hpp#L376).*

<a id="autonregistry-add"></a>

### `AutonRegistry::add`

```cpp
void add(const AutonEntry& e)
```

Register `e` (COPIED; its name and route stay borrowed). PRECONDITION: check(e) passes — a refused entry throws PreconditionError carrying check()'s sentence, which names the auton, so a program's `initialize()` try block can log it and refuse the boot by name (ruling B15: refusals happen at REGISTRATION, not in a match).

*function, declared at [`include/shulib/competition/auton_registry.hpp:399`](../../include/shulib/competition/auton_registry.hpp#L399).*

<a id="autonregistry-size"></a>

### `AutonRegistry::size`

```cpp
[[nodiscard]] std::size_t size() const noexcept
```

How many autons are registered.

*function, declared at [`include/shulib/competition/auton_registry.hpp:406`](../../include/shulib/competition/auton_registry.hpp#L406).*

<a id="autonregistry-empty"></a>

### `AutonRegistry::empty`

```cpp
[[nodiscard]] bool empty() const noexcept
```

True when none are.

*function, declared at [`include/shulib/competition/auton_registry.hpp:408`](../../include/shulib/competition/auton_registry.hpp#L408).*

<a id="autonregistry-at"></a>

### `AutonRegistry::at`

```cpp
[[nodiscard]] const AutonEntry& at(std::size_t index) const
```

The auton at `index` (registration order). Precondition: index < size().

*function, declared at [`include/shulib/competition/auton_registry.hpp:410`](../../include/shulib/competition/auton_registry.hpp#L410).*

<a id="autonregistry-find"></a>

### `AutonRegistry::find`

```cpp
[[nodiscard]] const AutonEntry* find(std::string_view name) const noexcept
```

The auton called `name`, or nullptr.

*function, declared at [`include/shulib/competition/auton_registry.hpp:415`](../../include/shulib/competition/auton_registry.hpp#L415).*

<a id="autonregistry-periods"></a>

### `AutonRegistry::periods`

```cpp
[[nodiscard]] const AutonPeriods& periods() const noexcept
```

The program's two periods, as given.

*function, declared at [`include/shulib/competition/auton_registry.hpp:427`](../../include/shulib/competition/auton_registry.hpp#L427).*

<a id="autonregistry-periodfor"></a>

### `AutonRegistry::periodFor`

```cpp
[[nodiscard]] units::Time periodFor(const AutonEntry& e) const noexcept
```

The period `e` runs in.

*function, declared at [`include/shulib/competition/auton_registry.hpp:429`](../../include/shulib/competition/auton_registry.hpp#L429).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 49 lines, click to expand</summary>

```text

 competition::AutonRegistry — the autons a competition program can run, as DATA (chunk COMP-a,
 2026-09-26; the COMP brief's rulings C2, C3, C6 and amendments B15, B16).

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

 ── Names (B25) ─────────────────────────────────────────────────────────────────────
 A name is shown as "AUTON: <name>" on ONE medium line of the brain screen, so it is at most
 kMaxAutonNameChars = ui::kMediumCols - 7 = 29 printable ASCII characters, no '%', unique, and
 never "NONE" (the selector's own choice). The name pointer is BORROWED — a string literal.

 PROS-free, allocation-free (fixed capacity), no std::function. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
