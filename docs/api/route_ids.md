<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/route_ids.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `route_ids.hpp`

mechanism/route_ids.hpp — EVERY MECHANISM'S ACTIONS AS ROUTE IDS, automatically (the mechanism core, chunk MECH-A1, 2026-10-08; the team lead's ruling of 2026-10-08, MECH-PANEL-requirements §4 item 7b: "an arm's presets as arm_stow, arm_score; an intake's i…

This header declares **4** types (23 members), **7** free functions, and **1** constant.

Extracted from [`include/shulib/mechanism/route_ids.hpp`](../../include/shulib/mechanism/route_ids.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class RouteAction`](#enum-class-routeaction)
  - [`GoToPreset`](#routeaction-gotopreset)
  - [`Home`](#routeaction-home)
  - [`SpinUp`](#routeaction-spinup)
  - [`Run`](#routeaction-run)
  - [`RunUntilPiece`](#routeaction-rununtilpiece)
  - [`RunUntilGone`](#routeaction-rununtilgone)
  - [`Stop`](#routeaction-stop)
  - [`SetPosition`](#routeaction-setposition)
- [`kMaxRouteIds`](#kmaxrouteids) — *constant*
- [`struct RouteId`](#struct-routeid)
  - [`id`](#routeid-id)
  - [`mechanism`](#routeid-mechanism)
  - [`action`](#routeid-action)
  - [`index`](#routeid-index)
  - [`argCount`](#routeid-argcount)
  - [`view`](#routeid-view)
- [`enum class RouteIdProblem`](#enum-class-routeidproblem)
  - [`None`](#routeidproblem-none)
  - [`TooLong`](#routeidproblem-toolong)
  - [`Duplicate`](#routeidproblem-duplicate)
  - [`Builtin`](#routeidproblem-builtin)
  - [`TooMany`](#routeidproblem-toomany)
- [`struct RouteIds`](#struct-routeids)
  - [`ids`](#routeids-ids)
  - [`count`](#routeids-count)
  - [`problem`](#routeids-problem)
  - [`offending`](#routeids-offending)
- [`routeIdsOf`](#routeidsof) — *free function*
- [`routeIdProblemText`](#routeidproblemtext) — *free function*
- [`a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name`](#a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name) — *free function*
- [`two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position`](#two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position) — *free function*
- [`a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log`](#a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log) — *free function*
- [`more_route_ids_than_a_route_registry_holds_32`](#more_route_ids_than_a_route_registry_holds_32) — *free function*
- [`refuseRouteIds`](#refuserouteids) — *free function*

<a id="enum-class-routeaction"></a>

## `enum class RouteAction`

```cpp
enum class RouteAction : std::uint8_t
```

What a route id does.

*enum class, declared at [`include/shulib/mechanism/route_ids.hpp:48`](../../include/shulib/mechanism/route_ids.hpp#L48).*

<a id="routeaction-gotopreset"></a>

### `RouteAction::GoToPreset`

```cpp
GoToPreset = 1
```

an arm's or a lift's preset: move there and hold — args [time limit ms]

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:49`](../../include/shulib/mechanism/route_ids.hpp#L49).*

<a id="routeaction-home"></a>

### `RouteAction::Home`

```cpp
Home
```

an arm or a lift: find zero by its method — args [time limit ms]

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:50`](../../include/shulib/mechanism/route_ids.hpp#L50).*

<a id="routeaction-spinup"></a>

### `RouteAction::SpinUp`

```cpp
SpinUp
```

a flywheel's preset: spin up, "at speed", keep spinning — args [time limit ms]

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:51`](../../include/shulib/mechanism/route_ids.hpp#L51).*

<a id="routeaction-run"></a>

### `RouteAction::Run`

```cpp
Run
```

a power mechanism: run at args[0] volts for args[1] ms

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:52`](../../include/shulib/mechanism/route_ids.hpp#L52).*

<a id="routeaction-rununtilpiece"></a>

### `RouteAction::RunUntilPiece`

```cpp
RunUntilPiece
```

a power mechanism: run at args[0] volts until its sensor sees a piece (limit args[1] ms)

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:53`](../../include/shulib/mechanism/route_ids.hpp#L53).*

<a id="routeaction-rununtilgone"></a>

### `RouteAction::RunUntilGone`

```cpp
RunUntilGone
```

a power mechanism: run at args[0] volts until its sensor stops seeing one (limit args[1] ms)

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:54`](../../include/shulib/mechanism/route_ids.hpp#L54).*

<a id="routeaction-stop"></a>

### `RouteAction::Stop`

```cpp
Stop
```

a power mechanism or a flywheel: its safe state

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:55`](../../include/shulib/mechanism/route_ids.hpp#L55).*

<a id="routeaction-setposition"></a>

### `RouteAction::SetPosition`

```cpp
SetPosition
```

a pneumatic's position: its lines now, its verdict after its move time

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:56`](../../include/shulib/mechanism/route_ids.hpp#L56).*

<a id="kmaxrouteids"></a>

## `kMaxRouteIds`

```cpp
inline constexpr std::size_t kMaxRouteIds = 32
```

The most route ids one robot's mechanisms may publish: a route registry's capacity — F8's design constant `runner::kMaxCommands` (32), which a host test pins equal (the runner sits above this layer, so it is not included here). The team's own registrations share it.

*constant, declared at [`include/shulib/mechanism/route_ids.hpp:62`](../../include/shulib/mechanism/route_ids.hpp#L62).*

<a id="struct-routeid"></a>

## `struct RouteId`

```cpp
struct RouteId
```

One published id: its text, whose mechanism (its index in the robot's list), what it does, which preset or position (`index`), and how many args a marker carrying it must give.

*struct, declared at [`include/shulib/mechanism/route_ids.hpp:66`](../../include/shulib/mechanism/route_ids.hpp#L66).*

<a id="routeid-id"></a>

### `RouteId::id`

```cpp
char id[config::kMaxMarkerIdBytes]{}
```

The id text, NUL-terminated (at most 23 characters — F8's rule).

*field, declared at [`include/shulib/mechanism/route_ids.hpp:68`](../../include/shulib/mechanism/route_ids.hpp#L68).*

<a id="routeid-mechanism"></a>

### `RouteId::mechanism`

```cpp
std::uint8_t mechanism = 0
```

The mechanism's index in the robot's list.

*field, declared at [`include/shulib/mechanism/route_ids.hpp:70`](../../include/shulib/mechanism/route_ids.hpp#L70).*

<a id="routeid-action"></a>

### `RouteId::action`

```cpp
RouteAction action = RouteAction::Stop
```

What the id does.

*field, declared at [`include/shulib/mechanism/route_ids.hpp:72`](../../include/shulib/mechanism/route_ids.hpp#L72).*

<a id="routeid-index"></a>

### `RouteId::index`

```cpp
std::uint8_t index = 0
```

GoToPreset / SpinUp: the preset's index; SetPosition: the position's; 0 otherwise.

*field, declared at [`include/shulib/mechanism/route_ids.hpp:74`](../../include/shulib/mechanism/route_ids.hpp#L74).*

<a id="routeid-argcount"></a>

### `RouteId::argCount`

```cpp
std::uint8_t argCount = 0
```

How many args a marker carrying this id gives (the rule's table, header).

*field, declared at [`include/shulib/mechanism/route_ids.hpp:76`](../../include/shulib/mechanism/route_ids.hpp#L76).*

<a id="routeid-view"></a>

### `RouteId::view`

```cpp
[[nodiscard]] constexpr std::string_view view() const noexcept
```

The id text as a view, bounded by its buffer.

*function, declared at [`include/shulib/mechanism/route_ids.hpp:78`](../../include/shulib/mechanism/route_ids.hpp#L78).*

<a id="enum-class-routeidproblem"></a>

## `enum class RouteIdProblem`

```cpp
enum class RouteIdProblem : std::uint8_t
```

Why a robot's route ids cannot be published (None = they can).

*enum class, declared at [`include/shulib/mechanism/route_ids.hpp:84`](../../include/shulib/mechanism/route_ids.hpp#L84).*

<a id="routeidproblem-none"></a>

### `RouteIdProblem::None`

```cpp
None = 0
```

every id is publishable

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:85`](../../include/shulib/mechanism/route_ids.hpp#L85).*

<a id="routeidproblem-toolong"></a>

### `RouteIdProblem::TooLong`

```cpp
TooLong
```

an id would run past 23 characters (or be empty)

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:86`](../../include/shulib/mechanism/route_ids.hpp#L86).*

<a id="routeidproblem-duplicate"></a>

### `RouteIdProblem::Duplicate`

```cpp
Duplicate
```

two ids would be the same

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:87`](../../include/shulib/mechanism/route_ids.hpp#L87).*

<a id="routeidproblem-builtin"></a>

### `RouteIdProblem::Builtin`

```cpp
Builtin
```

an id would be one of the library's own (wait_ms, brake, hold_ms, log)

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:88`](../../include/shulib/mechanism/route_ids.hpp#L88).*

<a id="routeidproblem-toomany"></a>

### `RouteIdProblem::TooMany`

```cpp
TooMany
```

more ids than a route registry holds (kMaxRouteIds)

*enumerator, declared at [`include/shulib/mechanism/route_ids.hpp:89`](../../include/shulib/mechanism/route_ids.hpp#L89).*

<a id="struct-routeids"></a>

## `struct RouteIds`

```cpp
struct RouteIds
```

A robot's route ids, in its list's order — or, at the FIRST problem, what it was and which id.

*struct, declared at [`include/shulib/mechanism/route_ids.hpp:93`](../../include/shulib/mechanism/route_ids.hpp#L93).*

<a id="routeids-ids"></a>

### `RouteIds::ids`

```cpp
RouteId ids[kMaxRouteIds]{}
```

The ids, in order (every one before the problem, if there is one).

*field, declared at [`include/shulib/mechanism/route_ids.hpp:95`](../../include/shulib/mechanism/route_ids.hpp#L95).*

<a id="routeids-count"></a>

### `RouteIds::count`

```cpp
std::uint8_t count = 0
```

How many of `ids` are real.

*field, declared at [`include/shulib/mechanism/route_ids.hpp:97`](../../include/shulib/mechanism/route_ids.hpp#L97).*

<a id="routeids-problem"></a>

### `RouteIds::problem`

```cpp
RouteIdProblem problem = RouteIdProblem::None
```

The first problem, or None.

*field, declared at [`include/shulib/mechanism/route_ids.hpp:99`](../../include/shulib/mechanism/route_ids.hpp#L99).*

<a id="routeids-offending"></a>

### `RouteIds::offending`

```cpp
char offending[48]{}
```

The id that broke the rule, as far as it was built (47 characters at most) — the panel's and a test's words for WHICH id.

*field, declared at [`include/shulib/mechanism/route_ids.hpp:102`](../../include/shulib/mechanism/route_ids.hpp#L102).*

<a id="routeidsof"></a>

## `routeIdsOf`

```cpp
[[nodiscard]] constexpr RouteIds routeIdsOf(std::span<const MechanismSpec> specs) noexcept
```

THE RULE (header): every id a robot's mechanisms publish, in the list's order — or the first problem. A spec of no known kind publishes nothing (specRefusal refuses it first).

*free function, declared at [`include/shulib/mechanism/route_ids.hpp:185`](../../include/shulib/mechanism/route_ids.hpp#L185).*

<a id="routeidproblemtext"></a>

## `routeIdProblemText`

```cpp
[[nodiscard]] constexpr const char* routeIdProblemText(RouteIdProblem p) noexcept
```

The sentence for a problem — the boot's and the panel's words (G3a's validator mirrors it); "" for None.

*free function, declared at [`include/shulib/mechanism/route_ids.hpp:228`](../../include/shulib/mechanism/route_ids.hpp#L228).*

<a id="a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name"></a>

## `a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name`

```cpp
inline void a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name()
```

RouteIdProblem::TooLong's name at the robot's list.

*free function, declared at [`include/shulib/mechanism/route_ids.hpp:248`](../../include/shulib/mechanism/route_ids.hpp#L248).*

<a id="two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position"></a>

## `two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position`

```cpp
inline void two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position()
```

RouteIdProblem::Duplicate's name at the robot's list.

*free function, declared at [`include/shulib/mechanism/route_ids.hpp:250`](../../include/shulib/mechanism/route_ids.hpp#L250).*

<a id="a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log"></a>

## `a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log`

```cpp
inline void a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log()
```

RouteIdProblem::Builtin's name at the robot's list.

*free function, declared at [`include/shulib/mechanism/route_ids.hpp:252`](../../include/shulib/mechanism/route_ids.hpp#L252).*

<a id="more_route_ids_than_a_route_registry_holds_32"></a>

## `more_route_ids_than_a_route_registry_holds_32`

```cpp
inline void more_route_ids_than_a_route_registry_holds_32()
```

RouteIdProblem::TooMany's name at the robot's list.

*free function, declared at [`include/shulib/mechanism/route_ids.hpp:254`](../../include/shulib/mechanism/route_ids.hpp#L254).*

<a id="refuserouteids"></a>

## `refuseRouteIds`

```cpp
consteval void refuseRouteIds(RouteIdProblem p)
```

Stop the build with `p`'s named function (consteval contexts only); None does nothing.

*free function, declared at [`include/shulib/mechanism/route_ids.hpp:258`](../../include/shulib/mechanism/route_ids.hpp#L258).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 34 lines</summary>

```text

 mechanism/route_ids.hpp — EVERY MECHANISM'S ACTIONS AS ROUTE IDS, automatically (the mechanism
 core, chunk MECH-A1, 2026-10-08; the team lead's ruling of 2026-10-08, MECH-PANEL-requirements §4
 item 7b: "an arm's presets as arm_stow, arm_score; an intake's intake_run, intake_until_piece ...
 a coder no longer registers each id by hand").

 ── The ids are DERIVED, never typed ────────────────────────────────────────────────
 From the robot's mechanism list, at BUILD time — the same declarations the autons and the driver
 file name, so an id can never drift from the mechanism it moves. The STEM is the mechanism's
 `.name` in route-id words: lower case, every run of other characters one '_' ("claw pivot 6" ->
 `claw_pivot_6`); a name with no letter or digit in it contributes nothing. The rule, one per kind:

   an ARM or a LIFT   <stem>_<preset> for each preset (move there, hold)       args [time limit ms]
                      <stem>_home when it has a homing method                  args [time limit ms]
   a FLYWHEEL         <stem>_<preset> for each preset (spin up, keep spinning) args [time limit ms]
                      <stem>_stop                                              no args
   a POWER mechanism  <stem>_run                                               args [volts, ms]
                      <stem>_until_piece, <stem>_until_gone (a piece sensor declared)
                                                                               args [volts, time limit ms]
                      <stem>_stop                                              no args
   a PNEUMATIC        <stem>_<position> for each position                     no args (its move time)

 F8's vocabulary rule holds for every id (config::commandIdValid: 1..23 bytes of [a-z0-9_]). An id
 that would break it, two ids that would be the same (a preset named "home" beside a homing
 method; "claw pivot" and "claw_pivot"), an id that is one of the library's four built-ins
 (`wait_ms`, `brake`, `hold_ms`, `log`), or more ids than a route registry holds (32) FAILS THE
 BUILD at the robot's list — mechanism/wiring.hpp's mechanisms() calls refuseRouteIds() — never a
 silently missing id. routeIdsOf() is constexpr, so a host test or a tool can derive any robot's
 ids with no brain, and the robot panel (G3a Part R) mirrors the rule and its four refusals.

 What RUNS an id is runner/mechanism_routes.hpp (the route binder, with the registry's per-id doc —
 MECH-A4); a program registers its robot's ids at boot since MECH-A5. This file is the rule only.

 PROS-free, allocation-free, constexpr.
```

</details>
