<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/until.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `until.hpp`

mechanism/until.hpp — WHEN A STEP IS DONE: a power step's done-conditions.

This header declares **4** types (29 members), **1** free function, **3** constants, and **3** type aliass.

Extracted from [`include/shulib/mechanism/until.hpp`](../../include/shulib/mechanism/until.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class DoneBy`](#enum-class-doneby)
  - [`None`](#doneby-none)
  - [`Time`](#doneby-time)
  - [`Reached`](#doneby-reached)
  - [`Stalled`](#doneby-stalled)
  - [`Sees`](#doneby-sees)
  - [`StopsSeeing`](#doneby-stopsseeing)
- [`doneByWord`](#donebyword) — *free function*
- [`class Until`](#class-until)
  - [`kSelf`](#until-kself)
  - [`time`](#until-time)
  - [`stalled`](#until-stalled)
  - [`sees`](#until-sees)
  - [`stopsSeeing`](#until-stopsseeing)
  - [`sees (overload 2)`](#until-sees-2)
  - [`stopsSeeing (overload 2)`](#until-stopsseeing-2)
  - [`bound`](#until-bound)
  - [`operator|`](#until-operator-or)
  - [`orAfter`](#until-orafter)
  - [`has`](#until-has)
  - [`seconds`](#until-seconds)
  - [`sensor`](#until-sensor)
  - [`readsOwnSensorToo`](#until-readsownsensortoo)
  - [`conflicting`](#until-conflicting)
  - [`kTime`](#until-ktime)
  - [`kStalled`](#until-kstalled)
  - [`kSees`](#until-ksees)
  - [`kStops`](#until-kstops)
- [`struct SeesPiece`](#struct-seespiece)
  - [`SeesPiece`](#seespiece-seespiece)
  - [`operator()`](#seespiece-operator-call)
- [`struct StopsSeeingPiece`](#struct-stopsseeingpiece)
  - [`StopsSeeingPiece`](#stopsseeingpiece-stopsseeingpiece)
  - [`operator()`](#stopsseeingpiece-operator-call)
- [`seesPiece`](#seespiece) — *constant*
- [`stopsSeeingPiece`](#stopsseeingpiece) — *constant*
- [`stalls`](#stalls) — *constant*
- [`seesPiece (overload 2)`](#seespiece-2) — *type alias*
- [`stopsSeeingPiece (overload 2)`](#stopsseeingpiece-2) — *type alias*
- [`stalls (overload 2)`](#stalls-2) — *type alias*

<a id="enum-class-doneby"></a>

## `enum class DoneBy`

```cpp
enum class DoneBy : std::uint8_t
```

Which done-condition ended a step — the word the step's log line names (`doneByWord`).

*enum class, declared at [`include/shulib/mechanism/until.hpp:56`](../../include/shulib/mechanism/until.hpp#L56).*

<a id="doneby-none"></a>

### `DoneBy::None`

```cpp
None = 0
```

no condition: the bound, a refusal, a cancel

*enumerator, declared at [`include/shulib/mechanism/until.hpp:57`](../../include/shulib/mechanism/until.hpp#L57).*

<a id="doneby-time"></a>

### `DoneBy::Time`

```cpp
Time
```

its time condition (a duration, or `orAfter`'s)

*enumerator, declared at [`include/shulib/mechanism/until.hpp:58`](../../include/shulib/mechanism/until.hpp#L58).*

<a id="doneby-reached"></a>

### `DoneBy::Reached`

```cpp
Reached
```

a position or speed step reached its target (MECH-A3)

*enumerator, declared at [`include/shulib/mechanism/until.hpp:59`](../../include/shulib/mechanism/until.hpp#L59).*

<a id="doneby-stalled"></a>

### `DoneBy::Stalled`

```cpp
Stalled
```

it stalled (`stalls`: SUCCEEDED; a stall signature otherwise: STALLED)

*enumerator, declared at [`include/shulib/mechanism/until.hpp:60`](../../include/shulib/mechanism/until.hpp#L60).*

<a id="doneby-sees"></a>

### `DoneBy::Sees`

```cpp
Sees
```

a piece sensor saw a piece

*enumerator, declared at [`include/shulib/mechanism/until.hpp:61`](../../include/shulib/mechanism/until.hpp#L61).*

<a id="doneby-stopsseeing"></a>

### `DoneBy::StopsSeeing`

```cpp
StopsSeeing
```

a piece sensor stopped seeing a piece

*enumerator, declared at [`include/shulib/mechanism/until.hpp:62`](../../include/shulib/mechanism/until.hpp#L62).*

<a id="donebyword"></a>

## `doneByWord`

```cpp
[[nodiscard]] constexpr const char* doneByWord(DoneBy d) noexcept
```

The words of a `DoneBy`, as the step's log line prints them. Never null; an out-of-range value reads "?".

*free function, declared at [`include/shulib/mechanism/until.hpp:67`](../../include/shulib/mechanism/until.hpp#L67).*

<a id="class-until"></a>

## `class Until`

```cpp
class Until
```

A step's done-conditions (header): any combination of a time, a stall and a piece sensor's seeing or not seeing, ALWAYS beside the step's bound. Combined with `|` (either ends the step) or `orAfter` (a sensor condition, or after a time).

*class, declared at [`include/shulib/mechanism/until.hpp:82`](../../include/shulib/mechanism/until.hpp#L82).*

<a id="until-kself"></a>

### `Until::kSelf`

```cpp
static constexpr const char* kSelf = nullptr
```

"This mechanism": no other mechanism named — the condition reads the step's OWN sensor.

*field, declared at [`include/shulib/mechanism/until.hpp:85`](../../include/shulib/mechanism/until.hpp#L85).*

<a id="until-time"></a>

### `Until::time`

```cpp
[[nodiscard]] static constexpr Until time(units::Time t) noexcept
```

Done when `t` has passed since the step started (a desired duration: SUCCEEDED, by time).

*function, declared at [`include/shulib/mechanism/until.hpp:88`](../../include/shulib/mechanism/until.hpp#L88).*

<a id="until-stalled"></a>

### `Until::stalled`

```cpp
[[nodiscard]] static constexpr Until stalled() noexcept
```

Done when the mechanism STALLS — a squeeze that closes until it is tight: SUCCEEDED, not a fault. Without it, a stated stall signature that trips is the STALLED verdict and a MECHANISM_STALLED fault. Needs the mechanism's stall signature (refused at the step without).

*function, declared at [`include/shulib/mechanism/until.hpp:94`](../../include/shulib/mechanism/until.hpp#L94).*

<a id="until-sees"></a>

### `Until::sees`

```cpp
[[nodiscard]] static constexpr Until sees() noexcept
```

Done when THIS mechanism's piece sensor sees a piece.

*function, declared at [`include/shulib/mechanism/until.hpp:96`](../../include/shulib/mechanism/until.hpp#L96).*

<a id="until-stopsseeing"></a>

### `Until::stopsSeeing`

```cpp
[[nodiscard]] static constexpr Until stopsSeeing() noexcept
```

Done when THIS mechanism's piece sensor stops seeing a piece.

*function, declared at [`include/shulib/mechanism/until.hpp:98`](../../include/shulib/mechanism/until.hpp#L98).*

<a id="until-sees-2"></a>

### `Until::sees (overload 2)`

```cpp
template <class Declared> [[nodiscard]] static constexpr Until sees(const Declared& other) noexcept
```

Done when ANOTHER mechanism's piece sensor sees a piece (the roller runs until the GRIP sees one), named by that mechanism's declaration of any kind and found by its NAME at run time. The words (`seesPiece(grip)`) also refuse, at build, a mechanism that declares no sensor and a declaration with no `.name`. Here, a declaration with no `.name` stores an EMPTY name — never kSelf, which would read the step's own sensor in its place — and the step refuses it.

*function, declared at [`include/shulib/mechanism/until.hpp:105`](../../include/shulib/mechanism/until.hpp#L105).*

<a id="until-stopsseeing-2"></a>

### `Until::stopsSeeing (overload 2)`

```cpp
template <class Declared> [[nodiscard]] static constexpr Until stopsSeeing(const Declared& other) noexcept
```

Done when ANOTHER mechanism's piece sensor stops seeing a piece (`sees(other)`'s rules).

*function, declared at [`include/shulib/mechanism/until.hpp:110`](../../include/shulib/mechanism/until.hpp#L110).*

<a id="until-bound"></a>

### `Until::bound`

```cpp
[[nodiscard]] static constexpr Until bound() noexcept
```

Never done by a condition: only the bound ends the step (TIMED_OUT) — a run that is meant to last its whole time limit while the robot does something else.

*function, declared at [`include/shulib/mechanism/until.hpp:115`](../../include/shulib/mechanism/until.hpp#L115).*

<a id="until-operator-or"></a>

### `Until::operator|`

```cpp
[[nodiscard]] constexpr Until operator|(const Until& o) const noexcept
```

Either condition ends the step. Two TIME conditions keep the EARLIER. The sensor conditions must read ONE mechanism's sensor (header): two different NAMES (`seesPiece(grip) | stopsSeeingPiece(claw)`) are `conflicting()` and the step refuses them; the step's OWN sensor beside a name keeps the name and sets `readsOwnSensorToo()`, which the step resolves (one sensor when it runs on that mechanism, refused otherwise).

*function, declared at [`include/shulib/mechanism/until.hpp:122`](../../include/shulib/mechanism/until.hpp#L122).*

<a id="until-orafter"></a>

### `Until::orAfter`

```cpp
[[nodiscard]] constexpr Until orAfter(units::Time t) const noexcept
```

"... or after `t`, whichever comes first" — the time ends it SUCCEEDED, by time (`seesPiece.orAfter(2.5_s)`: a missed piece does not end the auton). `t` must be shorter than the step's time limit (refused at the step otherwise: the bound would always win).

*function, declared at [`include/shulib/mechanism/until.hpp:157`](../../include/shulib/mechanism/until.hpp#L157).*

<a id="until-has"></a>

### `Until::has`

```cpp
[[nodiscard]] constexpr bool has(std::uint8_t bit) const noexcept
```

Whether the condition `bit` (kTime, kStalled, kSees, kStops) is part of it.

*function, declared at [`include/shulib/mechanism/until.hpp:160`](../../include/shulib/mechanism/until.hpp#L160).*

<a id="until-seconds"></a>

### `Until::seconds`

```cpp
[[nodiscard]] constexpr double seconds() const noexcept
```

The time condition's duration, seconds (0 when there is none).

*function, declared at [`include/shulib/mechanism/until.hpp:162`](../../include/shulib/mechanism/until.hpp#L162).*

<a id="until-sensor"></a>

### `Until::sensor`

```cpp
[[nodiscard]] constexpr const char* sensor() const noexcept
```

The NAME of the mechanism whose piece sensor the condition reads (kSelf = the step's own; "" = a declaration with no `.name`, which no robot's list has).

*function, declared at [`include/shulib/mechanism/until.hpp:165`](../../include/shulib/mechanism/until.hpp#L165).*

<a id="until-readsownsensortoo"></a>

### `Until::readsOwnSensorToo`

```cpp
[[nodiscard]] constexpr bool readsOwnSensorToo() const noexcept
```

True when the condition ALSO reads the step's own sensor beside the one `sensor()` names (`seesPiece | stopsSeeingPiece(intake)`): one sensor if the step runs on that mechanism, two otherwise — the step resolves it (operator|). Always false when `sensor()` is kSelf.

*function, declared at [`include/shulib/mechanism/until.hpp:169`](../../include/shulib/mechanism/until.hpp#L169).*

<a id="until-conflicting"></a>

### `Until::conflicting`

```cpp
[[nodiscard]] constexpr bool conflicting() const noexcept
```

True when `|` joined sensor conditions NAMING two different mechanisms (operator|): the step refuses it rather than silently read one of them.

*function, declared at [`include/shulib/mechanism/until.hpp:172`](../../include/shulib/mechanism/until.hpp#L172).*

<a id="until-ktime"></a>

### `Until::kTime`

```cpp
static constexpr std::uint8_t kTime = 1
```

The time condition's bit.

*field, declared at [`include/shulib/mechanism/until.hpp:175`](../../include/shulib/mechanism/until.hpp#L175).*

<a id="until-kstalled"></a>

### `Until::kStalled`

```cpp
static constexpr std::uint8_t kStalled = 4
```

The stall condition's bit (2 is left free: "reached" is a position step's own, not a condition).

*field, declared at [`include/shulib/mechanism/until.hpp:177`](../../include/shulib/mechanism/until.hpp#L177).*

<a id="until-ksees"></a>

### `Until::kSees`

```cpp
static constexpr std::uint8_t kSees = 8
```

The "sees a piece" condition's bit.

*field, declared at [`include/shulib/mechanism/until.hpp:179`](../../include/shulib/mechanism/until.hpp#L179).*

<a id="until-kstops"></a>

### `Until::kStops`

```cpp
static constexpr std::uint8_t kStops = 16
```

The "stops seeing a piece" condition's bit.

*field, declared at [`include/shulib/mechanism/until.hpp:181`](../../include/shulib/mechanism/until.hpp#L181).*

<a id="struct-seespiece"></a>

## `struct SeesPiece`

```cpp
struct SeesPiece : Until
```

THE WORD `seesPiece`: as itself, the step's OWN piece sensor; called with another mechanism's declaration, THAT one's (`seesPiece(grip)`) — refused AT BUILD when that one declares none.

*struct, declared at [`include/shulib/mechanism/until.hpp:231`](../../include/shulib/mechanism/until.hpp#L231).*

<a id="seespiece-seespiece"></a>

### `SeesPiece::SeesPiece`

```cpp
constexpr SeesPiece() noexcept
```

The step's own sensor.

*function, declared at [`include/shulib/mechanism/until.hpp:233`](../../include/shulib/mechanism/until.hpp#L233).*

<a id="seespiece-operator-call"></a>

### `SeesPiece::operator()`

```cpp
template <class Declared> [[nodiscard]] consteval Until operator()(const Declared& other) const
```

Another mechanism's piece sensor; a declaration with none fails the build, named.

*function, declared at [`include/shulib/mechanism/until.hpp:236`](../../include/shulib/mechanism/until.hpp#L236).*

<a id="struct-stopsseeingpiece"></a>

## `struct StopsSeeingPiece`

```cpp
struct StopsSeeingPiece : Until
```

THE WORD `stopsSeeingPiece`: as `seesPiece`, for "no longer sees a piece".

*struct, declared at [`include/shulib/mechanism/until.hpp:242`](../../include/shulib/mechanism/until.hpp#L242).*

<a id="stopsseeingpiece-stopsseeingpiece"></a>

### `StopsSeeingPiece::StopsSeeingPiece`

```cpp
constexpr StopsSeeingPiece() noexcept
```

The step's own sensor.

*function, declared at [`include/shulib/mechanism/until.hpp:244`](../../include/shulib/mechanism/until.hpp#L244).*

<a id="stopsseeingpiece-operator-call"></a>

### `StopsSeeingPiece::operator()`

```cpp
template <class Declared> [[nodiscard]] consteval Until operator()(const Declared& other) const
```

Another mechanism's piece sensor; a declaration with none fails the build, named.

*function, declared at [`include/shulib/mechanism/until.hpp:247`](../../include/shulib/mechanism/until.hpp#L247).*

<a id="seespiece"></a>

## `seesPiece`

```cpp
inline constexpr SeesPiece seesPiece{}
```

Until the step's piece sensor (or another's, `seesPiece(grip)`) sees a piece.

*constant, declared at [`include/shulib/mechanism/until.hpp:253`](../../include/shulib/mechanism/until.hpp#L253).*

<a id="stopsseeingpiece"></a>

## `stopsSeeingPiece`

```cpp
inline constexpr StopsSeeingPiece stopsSeeingPiece{}
```

Until it stops seeing one.

*constant, declared at [`include/shulib/mechanism/until.hpp:255`](../../include/shulib/mechanism/until.hpp#L255).*

<a id="stalls"></a>

## `stalls`

```cpp
inline constexpr Until stalls = Until::stalled()
```

Until the mechanism stalls (a squeeze: SUCCEEDED).

*constant, declared at [`include/shulib/mechanism/until.hpp:257`](../../include/shulib/mechanism/until.hpp#L257).*

<a id="seespiece-2"></a>

## `seesPiece (overload 2)`

```cpp
using mechanism::seesPiece
```

Until a piece sensor sees a piece.

*type alias, declared at [`include/shulib/mechanism/until.hpp:263`](../../include/shulib/mechanism/until.hpp#L263).*

<a id="stopsseeingpiece-2"></a>

## `stopsSeeingPiece (overload 2)`

```cpp
using mechanism::stopsSeeingPiece
```

Until a piece sensor stops seeing one.

*type alias, declared at [`include/shulib/mechanism/until.hpp:265`](../../include/shulib/mechanism/until.hpp#L265).*

<a id="stalls-2"></a>

## `stalls (overload 2)`

```cpp
using mechanism::stalls
```

Until the mechanism stalls.

*type alias, declared at [`include/shulib/mechanism/until.hpp:267`](../../include/shulib/mechanism/until.hpp#L267).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 44 lines</summary>

```text

 mechanism/until.hpp — WHEN A STEP IS DONE: a power step's done-conditions (the mechanism core,
 chunk MECH-A2, 2026-10-15).

 A step that runs a mechanism ends when ANY of its conditions holds — "this much time has passed",
 "it stalled", "a piece sensor sees a piece", "a piece sensor stops seeing one" — and ALWAYS also at
 its BOUND, the time limit every step states. `Until` never replaces the bound (requirements P4:
 every action is bounded and says how it ended); it only lets a step end sooner, and says which
 condition ended it (`DoneBy`, the word the step's log line names).

     r.run(intake, 12_volt, seesPiece, 3_s)              // until ITS piece sensor sees a piece
     r.run(intake, 12_volt, seesPiece.orAfter(2.5_s), 3_s)  // ... or after 2.5 s: SUCCEEDED by time
     r.run(intake, -6_volt, stopsSeeingPiece, 1_s)       // until it no longer sees one (an eject)
     r.run(roller, 12_volt, seesPiece(grip), 2_s)        // until ANOTHER mechanism's sensor sees one
     r.run(catapult, 6_volt, stalls, 1_s)                // until it stalls: a squeeze, SUCCEEDED

 (`r.run(...)` is the auton's step, MECH-A4; the mechanism service runs the condition,
 mechanism/service.hpp.)

 ── A condition that can never be met is a mistake, said where it is made ────────────
 `seesPiece(arm)` — another mechanism's sensor, on a mechanism that declares none — FAILS THE BUILD
 at the auton's line (detail::that_mechanism_has_no_piece_sensor). The forms the build cannot see
 (`seesPiece` / `stopsSeeingPiece` on the step's OWN mechanism, `stalls` on one with no stall
 signature) are refused at the STEP, before anything is commanded (the service's preconditions):
 the condition and the declaration are two arguments of one run-time call, and the host test of
 every registered auton runs each step against its robot's real list (MECH-A5), so the refusal
 fails the suite before any upload.

 ── What a condition reads ───────────────────────────────────────────────────────────
 A piece sensor is read as POSITIVE evidence (the service's Slot::seen()): YES, NO or UNKNOWN, and
 UNKNOWN — a sensor declared but missing this boot, a distance held near with no confidence, a
 sample taken while its read-fault count moved — satisfies NEITHER `seesPiece` nor
 `stopsSeeingPiece`. A condition that names ANOTHER mechanism stores that mechanism's NAME, not
 its address: the service finds it by name at run time (a declaration copied without `inline` has
 one address per translation unit — the brief's P12). A declaration written without `.name` is
 on no robot's list: the words refuse it at build (A1's `a_mechanism_has_no_name`), and the plain
 `Until::sees(decl)` stores an EMPTY name — never "this mechanism" — which the step refuses.

 ONE mechanism's sensor per condition. Two different NAMES joined by `|` are `conflicting()` at
 once. The step's OWN sensor beside a NAME (`seesPiece | stopsSeeingPiece(intake)`) is one sensor
 when the step runs on that very mechanism and two otherwise — only the step knows which, so the
 condition keeps both facts (`sensor()` and `readsOwnSensorToo()`) and the step decides.

 Plain constexpr data: a bit set, a time, which mechanism's sensor. PROS-free, allocation-free.
```

</details>
