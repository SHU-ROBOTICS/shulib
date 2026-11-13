<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/config/route.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `route.hpp`

Route — an autonomous routine as DATA.

This header declares **9** types (45 members), **6** free functions, and **6** constants.

Extracted from [`include/shulib/config/route.hpp`](../../include/shulib/config/route.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxWaypoints`](#kmaxwaypoints) — *constant*
- [`kMaxMarkersPerWaypoint`](#kmaxmarkersperwaypoint) — *constant*
- [`kMaxMarkerArgs`](#kmaxmarkerargs) — *constant*
- [`kMaxMarkerIdBytes`](#kmaxmarkeridbytes) — *constant*
- [`commandIdView`](#commandidview) — *free function*
- [`commandIdValid`](#commandidvalid) — *free function*
- [`struct BuiltinCommand`](#struct-builtincommand)
  - [`id`](#builtincommand-id)
  - [`minArgs`](#builtincommand-minargs)
  - [`maxArgs`](#builtincommand-maxargs)
  - [`firstArgIsDurationMs`](#builtincommand-firstargisdurationms)
  - [`doc`](#builtincommand-doc)
- [`kBuiltinCommands`](#kbuiltincommands) — *constant*
- [`kBuiltinCommandCount`](#kbuiltincommandcount) — *constant*
- [`findBuiltinCommand`](#findbuiltincommand) — *free function*
- [`enum class WaypointMotion`](#enum-class-waypointmotion)
  - [`Start`](#waypointmotion-start)
  - [`MoveToPose`](#waypointmotion-movetopose)
  - [`DriveTo`](#waypointmotion-driveto)
  - [`TurnTo`](#waypointmotion-turnto)
  - [`StrafeTo`](#waypointmotion-strafeto)
  - [`Hold`](#waypointmotion-hold)
- [`waypointMotionWord`](#waypointmotionword) — *free function*
- [`enum class HeadingMode`](#enum-class-headingmode)
  - [`Fixed`](#headingmode-fixed)
  - [`Free`](#headingmode-free)
- [`headingModeWord`](#headingmodeword) — *free function*
- [`struct Constraints`](#struct-constraints)
  - [`maxLinear`](#constraints-maxlinear)
  - [`maxAngular`](#constraints-maxangular)
  - [`timeout`](#constraints-timeout)
- [`struct Marker`](#struct-marker)
  - [`id`](#marker-id)
  - [`args`](#marker-args)
  - [`argCount`](#marker-argcount)
- [`markerIdView`](#markeridview) — *free function*
- [`struct Waypoint`](#struct-waypoint)
  - [`x`](#waypoint-x)
  - [`y`](#waypoint-y)
  - [`heading`](#waypoint-heading)
  - [`headingMode`](#waypoint-headingmode)
  - [`motion`](#waypoint-motion)
  - [`reverse`](#waypoint-reverse)
  - [`constraints`](#waypoint-constraints)
  - [`markers`](#waypoint-markers)
  - [`markerCount`](#waypoint-markercount)
- [`struct Route`](#struct-route)
  - [`name`](#route-name)
  - [`waypoints`](#route-waypoints)
  - [`count`](#route-count)
  - [`valid`](#route-valid)
- [`class IRouteSource`](#class-iroutesource)
  - [`~IRouteSource`](#iroutesource-destructor-iroutesource)
  - [`IRouteSource`](#iroutesource-iroutesource)
  - [`IRouteSource (overload 2)`](#iroutesource-iroutesource-2)
  - [`IRouteSource (overload 3)`](#iroutesource-iroutesource-3)
  - [`operator=`](#iroutesource-operator-eq)
  - [`operator= (overload 2)`](#iroutesource-operator-eq-2)
  - [`route`](#iroutesource-route)
  - [`count`](#iroutesource-count)
  - [`at`](#iroutesource-at)
- [`class StaticRoutes`](#class-staticroutes)
  - [`StaticRoutes`](#staticroutes-staticroutes)
  - [`route`](#staticroutes-route)
  - [`count`](#staticroutes-count)
  - [`at`](#staticroutes-at)

<a id="kmaxwaypoints"></a>

## `kMaxWaypoints`

```cpp
inline constexpr std::size_t kMaxWaypoints = 32
```

The most waypoints one Route may carry. A DESIGN constant (ruling B6): a route longer than this is two routes, which is also how a person should be thinking about it. Not a hardware belief — nothing measured says 32.

*constant, declared at [`include/shulib/config/route.hpp:52`](../../include/shulib/config/route.hpp#L52).*

<a id="kmaxmarkersperwaypoint"></a>

## `kMaxMarkersPerWaypoint`

```cpp
inline constexpr std::size_t kMaxMarkersPerWaypoint = 4
```

The most markers one waypoint may fire. Design constant (B6).

*constant, declared at [`include/shulib/config/route.hpp:55`](../../include/shulib/config/route.hpp#L55).*

<a id="kmaxmarkerargs"></a>

## `kMaxMarkerArgs`

```cpp
inline constexpr std::size_t kMaxMarkerArgs = 2
```

The most typed arguments one marker may carry. Design constant (B6): two covers `{id:"lift_to_level", args:{3}}` and every parametric primitive sketched so far, and a marker that needs a third is a marker that wants a different id.

*constant, declared at [`include/shulib/config/route.hpp:60`](../../include/shulib/config/route.hpp#L60).*

<a id="kmaxmarkeridbytes"></a>

## `kMaxMarkerIdBytes`

```cpp
inline constexpr std::size_t kMaxMarkerIdBytes = 24
```

Bytes of storage one command id occupies, NUL INCLUDED — so the longest id TEXT is 23 bytes. G1 wrote `char id[24]` and never named the number; G2 needs the same bound in three more places (the registry's key buffer, the manifest writer, the id-validity rule), and a capacity spelled as a literal in four files is a capacity that will disagree with itself. Design constant (B6), not a hardware belief: 23 characters is longer than every id anyone has proposed, and short enough that a whole registry of 32 is under a kilobyte.

*constant, declared at [`include/shulib/config/route.hpp:68`](../../include/shulib/config/route.hpp#L68).*

<a id="commandidview"></a>

## `commandIdView`

```cpp
[[nodiscard]] constexpr std::string_view commandIdView(const char* id, std::size_t capacity) noexcept
```

The text of a fixed-width, possibly UNTERMINATED character buffer (a command id, a route name), bounded by the BUFFER rather than by `strlen` — the same hazard `StaticRoutes::route()` guards against, since a generator may fill the last byte. `capacity` is the buffer's size in bytes.

*free function, declared at [`include/shulib/config/route.hpp:74`](../../include/shulib/config/route.hpp#L74).*

<a id="commandidvalid"></a>

## `commandIdValid`

```cpp
[[nodiscard]] constexpr bool commandIdValid(std::string_view id) noexcept
```

Is `id` a legal command id? Non-empty, at most `kMaxMarkerIdBytes - 1` characters, and drawn from `[a-z0-9_]` only.  The charset is deliberately narrow (G2's REJECTED list): an id is the SAME STRING in a C++ source line, in a manifest line, in a `.vexbot` file and on a picker's button, and a vocabulary with no uppercase, no spaces, no punctuation and no non-ASCII is one that never needs escaping on any of those four paths. The length bound is what makes the id storable in the fixed buffer WITH a terminator — a 24-byte id that filled its buffer could be compared but never printed.

*free function, declared at [`include/shulib/config/route.hpp:91`](../../include/shulib/config/route.hpp#L91).*

<a id="struct-builtincommand"></a>

## `struct BuiltinCommand`

```cpp
struct BuiltinCommand
```

One LIBRARY-OWNED command id: a marker a team never registers because the runner itself performs it (G2 ruling R3). The table below is the whole set; a team that registered `brake` to mean something else would break every route another team exported, so `CommandRegistry` refuses one as a precondition violation.  The SPEC lives here, at the data layer, rather than in `runner/`, because an id, its arity and its meaning are facts about the ROUTE VOCABULARY — `Route::valid()` checks them, the manifest publishes them and F8 freezes them; only the DOING of them belongs to the runner. The reverse arrangement would make `config/` depend on `runner/`, which is upside down.

*struct, declared at [`include/shulib/config/route.hpp:113`](../../include/shulib/config/route.hpp#L113).*

<a id="builtincommand-id"></a>

### `BuiltinCommand::id`

```cpp
const char* id
```

The id text, e.g. "wait_ms". Always `commandIdValid()`.

*field, declared at [`include/shulib/config/route.hpp:115`](../../include/shulib/config/route.hpp#L115).*

<a id="builtincommand-minargs"></a>

### `BuiltinCommand::minArgs`

```cpp
std::uint8_t minArgs
```

The fewest typed args the marker must carry.

*field, declared at [`include/shulib/config/route.hpp:117`](../../include/shulib/config/route.hpp#L117).*

<a id="builtincommand-maxargs"></a>

### `BuiltinCommand::maxArgs`

```cpp
std::uint8_t maxArgs
```

The most typed args the marker may carry (<= kMaxMarkerArgs).

*field, declared at [`include/shulib/config/route.hpp:119`](../../include/shulib/config/route.hpp#L119).*

<a id="builtincommand-firstargisdurationms"></a>

### `BuiltinCommand::firstArgIsDurationMs`

```cpp
bool firstArgIsDurationMs
```

True when `args[0]` is a DURATION IN MILLISECONDS and must therefore be > 0 — the facade verbs behind these ids refuse a non-positive duration at the door, so a route carrying one is a routine that throws mid-auton instead of one that is refused before it runs.

*field, declared at [`include/shulib/config/route.hpp:123`](../../include/shulib/config/route.hpp#L123).*

<a id="builtincommand-doc"></a>

### `BuiltinCommand::doc`

```cpp
const char* doc
```

What it does, in one line — published verbatim in the manifest a picker reads.

*field, declared at [`include/shulib/config/route.hpp:125`](../../include/shulib/config/route.hpp#L125).*

<a id="kbuiltincommands"></a>

## `kBuiltinCommands`

```cpp
inline constexpr BuiltinCommand kBuiltinCommands[] = { {"wait_ms", 1, 1, true, "wait, commanding nothing, for args[0] milliseconds (the drive keeps its last state)"}, {"brake", 0, 0, false, "stop the drive (0 V + Brake) and block until the estimate certifies rest"}, {"hold_ms", 1, 1, true, "actively hold the current pose for args[0] milliseconds, driving back disturbance"}, {"log", 0, 2, false, "write one Info line naming this marker and its args"}, }
```

Every built-in id, in the order the manifest publishes them. **APPEND-ONLY**: the order is part of the manifest's byte-exact contract (G2 landmine 5) and the id strings are frozen by F8.

*constant, declared at [`include/shulib/config/route.hpp:130`](../../include/shulib/config/route.hpp#L130).*

<a id="kbuiltincommandcount"></a>

## `kBuiltinCommandCount`

```cpp
inline constexpr std::size_t kBuiltinCommandCount = sizeof kBuiltinCommands / sizeof(BuiltinCommand)
```

How many built-in ids there are — `kBuiltinCommands`'s length, named so a manifest header and a freeze pin can state it without re-deriving it.

*constant, declared at [`include/shulib/config/route.hpp:142`](../../include/shulib/config/route.hpp#L142).*

<a id="findbuiltincommand"></a>

## `findBuiltinCommand`

```cpp
[[nodiscard]] constexpr const BuiltinCommand* findBuiltinCommand(std::string_view id) noexcept
```

The built-in whose id is exactly `id`, or nullptr when `id` is not one. Exact match only: ids are compared whole, never by prefix (`StaticRoutes::route()`'s lesson).

*free function, declared at [`include/shulib/config/route.hpp:146`](../../include/shulib/config/route.hpp#L146).*

<a id="enum-class-waypointmotion"></a>

## `enum class WaypointMotion`

```cpp
enum class WaypointMotion : std::uint8_t
```

What the robot does to get to a waypoint. Every value maps onto a verb the facade has TODAY (ruling B5); the numeric values are explicit and APPEND-ONLY, because a route may be generated by one version of the tools and read by another.

*enum class, declared at [`include/shulib/config/route.hpp:158`](../../include/shulib/config/route.hpp#L158).*

<a id="waypointmotion-start"></a>

### `WaypointMotion::Start`

```cpp
Start = 1
```

Not a motion: "the robot starts HERE". Legal only at index 0 (see Route::valid()); the runner seeds the pose from it rather than driving to it.

*enumerator, declared at [`include/shulib/config/route.hpp:161`](../../include/shulib/config/route.hpp#L161).*

<a id="waypointmotion-movetopose"></a>

### `WaypointMotion::MoveToPose`

```cpp
MoveToPose = 2
```

`Chassis::moveTo(pose)` — translate and rotate simultaneously to the full pose. With `HeadingMode::Free` the commanded heading is the estimate's own, read when the leg starts (G2 ruling R6): "go there, keep facing where I am facing".

*enumerator, declared at [`include/shulib/config/route.hpp:165`](../../include/shulib/config/route.hpp#L165).*

<a id="waypointmotion-driveto"></a>

### `WaypointMotion::DriveTo`

```cpp
DriveTo = 3
```

`Routine::driveTo(x, y)` — move to the point ARRIVING FACING IT; `heading` is ignored and computed from the approach, which is why headingMode must be Free. With `reverse`, it is `Routine::reverseTo(x, y)`: the same point, arriving BACKWARDS.

*enumerator, declared at [`include/shulib/config/route.hpp:169`](../../include/shulib/config/route.hpp#L169).*

<a id="waypointmotion-turnto"></a>

### `WaypointMotion::TurnTo`

```cpp
TurnTo = 4
```

`Chassis::turnTo(heading)` — rotate in place; `x` and `y` are ignored.

*enumerator, declared at [`include/shulib/config/route.hpp:171`](../../include/shulib/config/route.hpp#L171).*

<a id="waypointmotion-strafeto"></a>

### `WaypointMotion::StrafeTo`

```cpp
StrafeTo = 5
```

`Chassis::strafeTo(x, y)` — translate holding the current heading. Honest on tank: an off-line target exits TimedOut rather than pretending (C1's drivetrain honesty).

*enumerator, declared at [`include/shulib/config/route.hpp:174`](../../include/shulib/config/route.hpp#L174).*

<a id="waypointmotion-hold"></a>

### `WaypointMotion::Hold`

```cpp
Hold = 6
```

`Chassis::hold(duration)` — actively hold the pose for `constraints.timeout`, which for this motion is the DURATION and must therefore be > 0.

*enumerator, declared at [`include/shulib/config/route.hpp:177`](../../include/shulib/config/route.hpp#L177).*

<a id="waypointmotionword"></a>

## `waypointMotionWord`

```cpp
[[nodiscard]] inline const char* waypointMotionWord(WaypointMotion m) noexcept
```

The motion as a word, for banners and refusal messages. "UNKNOWN" for a byte outside the enum.

*free function, declared at [`include/shulib/config/route.hpp:181`](../../include/shulib/config/route.hpp#L181).*

<a id="enum-class-headingmode"></a>

## `enum class HeadingMode`

```cpp
enum class HeadingMode : std::uint8_t
```

Whether the waypoint's `heading` field means anything.

*enum class, declared at [`include/shulib/config/route.hpp:194`](../../include/shulib/config/route.hpp#L194).*

<a id="headingmode-fixed"></a>

### `HeadingMode::Fixed`

```cpp
Fixed = 1
```

The robot must arrive at `heading`. The only mode `TurnTo` accepts, and the mode `MoveToPose` takes when the author has a heading in mind.

*enumerator, declared at [`include/shulib/config/route.hpp:197`](../../include/shulib/config/route.hpp#L197).*

<a id="headingmode-free"></a>

### `HeadingMode::Free`

```cpp
Free = 2
```

The heading is the RUNNER's to choose, and `heading` is not read. What it chooses depends on the verb, and each choice is the only honest one that verb has (G2 ruling R6): * `DriveTo` — the approach bearing to (x, y), so the robot arrives facing the point. This is the only mode `DriveTo` accepts. * `MoveToPose` — whatever heading the estimate already has when the leg STARTS ("translate, keep facing where I am facing"). `moveTo` always commands a heading, so there is no "don't care" to pass down and the current one is the one that asks the drivetrain to do the least. * `StrafeTo`, `Hold`, `Start` — the runner reads neither mode; either is accepted. On a `Start` a competition auton entry DOES read it (competition/auton_registry.hpp, since COMP-a): a `Fixed` Start's `heading` is the heading the robot is seated to when the auton starts, and a `Free` Start states none, so the entry must declare one. * `TurnTo` — REFUSED: a turn to no heading is not a motion.

*enumerator, declared at [`include/shulib/config/route.hpp:211`](../../include/shulib/config/route.hpp#L211).*

<a id="headingmodeword"></a>

## `headingModeWord`

```cpp
[[nodiscard]] inline const char* headingModeWord(HeadingMode m) noexcept
```

The heading mode as a word, for messages. "UNKNOWN" for a byte outside the enum.

*free function, declared at [`include/shulib/config/route.hpp:215`](../../include/shulib/config/route.hpp#L215).*

<a id="struct-constraints"></a>

## `struct Constraints`

```cpp
struct Constraints
```

Per-waypoint speed and time budgets. 0 means "use the robot's configured default" — the same sentinel `chassis::MotionOptions` already uses, so a route's constraints map onto it field for field with no translation table.

*struct, declared at [`include/shulib/config/route.hpp:226`](../../include/shulib/config/route.hpp#L226).*

<a id="constraints-maxlinear"></a>

### `Constraints::maxLinear`

```cpp
units::Velocity maxLinear{0.0}
```

Field-frame linear speed budget for this leg, in/s. Finite and >= 0; 0 = the default.

*field, declared at [`include/shulib/config/route.hpp:228`](../../include/shulib/config/route.hpp#L228).*

<a id="constraints-maxangular"></a>

### `Constraints::maxAngular`

```cpp
units::AngularVelocity maxAngular{0.0}
```

Yaw-rate budget for this leg, rad/s. Finite and >= 0; 0 = the default.

*field, declared at [`include/shulib/config/route.hpp:230`](../../include/shulib/config/route.hpp#L230).*

<a id="constraints-timeout"></a>

### `Constraints::timeout`

```cpp
units::Time timeout{0.0}
```

Watchdog bound for this leg, seconds. Finite and >= 0; 0 = the default — EXCEPT on a `Hold`, where this is the hold DURATION and must be > 0 (there is no sensible default duration for "stay here", and `Chassis::hold` refuses a non-positive one anyway).

*field, declared at [`include/shulib/config/route.hpp:234`](../../include/shulib/config/route.hpp#L234).*

<a id="struct-marker"></a>

## `struct Marker`

```cpp
struct Marker
```

One thing to do at a waypoint, named by an ID the student's registry binds to an action.  The id is a fixed-width NUL-terminated buffer rather than a pointer, so a whole Route can be written to a card or emitted by a generator as one object. The arguments are `std::int32_t` and deliberately NOT a variant or a double: every parametric primitive anyone has asked for takes a small integer (a lift level, a count, a side), and a typed integer is the one form that survives serialization, `constexpr` emission and a human reading it in a diff.

*struct, declared at [`include/shulib/config/route.hpp:244`](../../include/shulib/config/route.hpp#L244).*

<a id="marker-id"></a>

### `Marker::id`

```cpp
char id[kMaxMarkerIdBytes]{}
```

The command id, e.g. "intake_in". Non-empty and `commandIdValid()` for any marker inside `Waypoint::markerCount`. An id G2's registry does not know logs a WARN and is SKIPPED (§16.3) — it is not a route validity error, because marker ids are an open vocabulary a team extends. An id outside the CHARSET is a different thing and IS refused: no registry can ever hold it, so it could only ever warn.

*field, declared at [`include/shulib/config/route.hpp:250`](../../include/shulib/config/route.hpp#L250).*

<a id="marker-args"></a>

### `Marker::args`

```cpp
std::int32_t args[kMaxMarkerArgs]{}
```

The typed arguments, e.g. `{3}` for "lift_to_level". Entries beyond `argCount` are 0 and are not read.

*field, declared at [`include/shulib/config/route.hpp:253`](../../include/shulib/config/route.hpp#L253).*

<a id="marker-argcount"></a>

### `Marker::argCount`

```cpp
std::uint8_t argCount = 0
```

How many entries of `args` are real. 0 is the common case.

*field, declared at [`include/shulib/config/route.hpp:255`](../../include/shulib/config/route.hpp#L255).*

<a id="markeridview"></a>

## `markerIdView`

```cpp
[[nodiscard]] constexpr std::string_view markerIdView(const Marker& marker) noexcept
```

A marker's id as text, bounded by the buffer (`commandIdView`'s note). This is the ONE spelling the registry, the runner and the manifest all use, so no consumer can invent a different reading of the same bytes.

*free function, declared at [`include/shulib/config/route.hpp:261`](../../include/shulib/config/route.hpp#L261).*

<a id="struct-waypoint"></a>

## `struct Waypoint`

```cpp
struct Waypoint
```

One step of a route: where, how, how fast, and what to do there.

*struct, declared at [`include/shulib/config/route.hpp:266`](../../include/shulib/config/route.hpp#L266).*

<a id="waypoint-x"></a>

### `Waypoint::x`

```cpp
units::Length x{0.0}
```

Field x, inches. Must be finite. Not read when `motion` is `TurnTo` or `Hold`.

*field, declared at [`include/shulib/config/route.hpp:268`](../../include/shulib/config/route.hpp#L268).*

<a id="waypoint-y"></a>

### `Waypoint::y`

```cpp
units::Length y{0.0}
```

Field y, inches. Must be finite. Not read when `motion` is `TurnTo` or `Hold`.

*field, declared at [`include/shulib/config/route.hpp:270`](../../include/shulib/config/route.hpp#L270).*

<a id="waypoint-heading"></a>

### `Waypoint::heading`

```cpp
math::Angle heading{}
```

Field heading. Must be finite. Read only when `headingMode == Fixed` — on a `Start`, as the heading a competition auton is seated to (HeadingMode's note).

*field, declared at [`include/shulib/config/route.hpp:273`](../../include/shulib/config/route.hpp#L273).*

<a id="waypoint-headingmode"></a>

### `Waypoint::headingMode`

```cpp
HeadingMode headingMode = HeadingMode::Fixed
```

Whether `heading` means anything at this waypoint.

*field, declared at [`include/shulib/config/route.hpp:275`](../../include/shulib/config/route.hpp#L275).*

<a id="waypoint-motion"></a>

### `Waypoint::motion`

```cpp
WaypointMotion motion = WaypointMotion::MoveToPose
```

Which verb takes the robot here.

*field, declared at [`include/shulib/config/route.hpp:277`](../../include/shulib/config/route.hpp#L277).*

<a id="waypoint-reverse"></a>

### `Waypoint::reverse`

```cpp
bool reverse = false
```

Back into this leg's point. Only a `DriveTo` has an approach to reverse (`valid()` refuses it on every other verb): the runner adds π to the bearing, so the robot arrives with its back to the point — exactly `Routine::reverseTo` (L9, API 2.4), pinned bit-identical to it. On a tank — or on any drive whose back must lead the whole leg, since a holonomic drive otherwise turns round in flight — an authored `TurnTo` that puts the robot's back to the point goes first, as `Routine::faceAway` does in a recipe. (Until L9 this comment called the flag advisory and read by no verb; the runner has read it since G2.) Carried, too, because the legacy planner's rows have it and a route that loses it on import has silently changed what a person drew.

*field, declared at [`include/shulib/config/route.hpp:287`](../../include/shulib/config/route.hpp#L287).*

<a id="waypoint-constraints"></a>

### `Waypoint::constraints`

```cpp
Constraints constraints{}
```

This leg's budgets.

*field, declared at [`include/shulib/config/route.hpp:289`](../../include/shulib/config/route.hpp#L289).*

<a id="waypoint-markers"></a>

### `Waypoint::markers`

```cpp
Marker markers[kMaxMarkersPerWaypoint]{}
```

What to fire here. Entries beyond `markerCount` are ignored entirely.

*field, declared at [`include/shulib/config/route.hpp:291`](../../include/shulib/config/route.hpp#L291).*

<a id="waypoint-markercount"></a>

### `Waypoint::markerCount`

```cpp
std::uint8_t markerCount = 0
```

How many entries of `markers` are real. Must be <= kMaxMarkersPerWaypoint.

*field, declared at [`include/shulib/config/route.hpp:293`](../../include/shulib/config/route.hpp#L293).*

<a id="struct-route"></a>

## `struct Route`

```cpp
struct Route
```

A whole autonomous routine, as one value.  Plain data with designated initializers at the call site. Like `RobotConfig`, it does not validate itself on construction — a caller builds one and asks `valid()`, which is what lets a test build a deliberately contradictory route and watch it be refused by name.

*struct, declared at [`include/shulib/config/route.hpp:301`](../../include/shulib/config/route.hpp#L301).*

<a id="route-name"></a>

### `Route::name`

```cpp
char name[32]{}
```

The route's name — the key `IRouteSource::route()` looks up. Must be non-empty: a route nobody can name is a route nobody can run.

*field, declared at [`include/shulib/config/route.hpp:304`](../../include/shulib/config/route.hpp#L304).*

<a id="route-waypoints"></a>

### `Route::waypoints`

```cpp
Waypoint waypoints[kMaxWaypoints]{}
```

The steps, in order.

*field, declared at [`include/shulib/config/route.hpp:306`](../../include/shulib/config/route.hpp#L306).*

<a id="route-count"></a>

### `Route::count`

```cpp
std::uint8_t count = 0
```

How many entries of `waypoints` are real. Must be >= 1 and <= kMaxWaypoints.

*field, declared at [`include/shulib/config/route.hpp:308`](../../include/shulib/config/route.hpp#L308).*

<a id="route-valid"></a>

### `Route::valid`

```cpp
[[nodiscard]] bool valid(char* why = nullptr, std::size_t cap = 0) const noexcept
```

True when nothing in the route contradicts itself. `why` (optional; pass nullptr and 0 to skip) receives the FIRST contradiction found, naming the waypoint index and the field.  The checks are exactly what a mis-drawn path, a bad import or a mis-generated file produces: an empty route name; no waypoints at all, or more than the capacity; a `Start` anywhere but index 0; an unknown motion byte or an unknown heading-mode byte; a NaN or an infinity in x, y or heading; a `TurnTo` whose headingMode is Free (a turn to no heading is not a motion) or a `DriveTo` whose headingMode is Fixed (the verb computes the heading, so a stated one would be silently ignored); a negative or non-finite budget; a `Hold` whose timeout is not > 0 (that field IS the hold duration); more markers than capacity; and an empty marker id.  **Changed at G2 (ruling R6), the one RELAXATION on this list:** a `MoveToPose` with `HeadingMode::Free` used to be refused alongside `TurnTo`, on the reading that both verbs "need a heading". They do — but a `MoveToPose` has an honest one to fall back on that a `TurnTo` does not: the heading the robot already has. "Translate there, keep facing where I am facing" is a thing path authors ask for constantly and it had no spelling in the vocabulary; the runner supplies the estimate's heading at the leg's start. Relaxing a validity rule is the ADDITIVE direction (every route that was valid before is still valid), which is why it is a thing G2 may do at all and why doing it at the F8 freeze rather than after it costs nothing. `TurnTo` keeps the refusal, unchanged.  **Added at G2**, once a runner existed to say what the data can actually ask for — the same principle, applied to four more ways a route can contradict itself: * `reverse` on anything but a `DriveTo`. `DriveTo` is the only verb with an approach direction to reverse (the runner drives to the point facing away from it); on every other verb the flag is READ BY NOTHING, and a stated value that is silently ignored is the quieter of the two bugs — exactly the reasoning that already refuses a `DriveTo` with a `Fixed` heading. * A marker id outside the command-id charset (`commandIdValid`): non-empty, `[a-z0-9_]`, at most `kMaxMarkerIdBytes - 1` bytes. An unknown id is legal data; an UNREGISTRABLE one is not, because `CommandRegistry` refuses it and the runner could therefore only ever warn about it. * A BUILT-IN id with the wrong number of args (`kBuiltinCommands`): `wait_ms` without its duration is not a command the runner can guess a value for. * A built-in whose `args[0]` is a duration and is not > 0: `Chassis::wait`/`hold` refuse a non-positive duration at the door, so this is the `Hold`-timeout rule applied to the two markers that carry the same quantity.  Index 0 is NOT required to be a `Start`: a route that begins with a motion from wherever the robot already is, is a legitimate thing to author (it is what every hand-written auton in this repo's history does), and requiring a Start would force every importer to invent a pose it does not have.  noexcept and total: it allocates nothing, throws nothing, and returns for every input.

*function, declared at [`include/shulib/config/route.hpp:355`](../../include/shulib/config/route.hpp#L355).*

<a id="class-iroutesource"></a>

## `class IRouteSource`

```cpp
class IRouteSource
```

The route seam (master plan decision #10): where a robot's routines come from. A generated header, a card read at boot and an array typed by hand all look the same through it.

*class, declared at [`include/shulib/config/route.hpp:560`](../../include/shulib/config/route.hpp#L560).*

<a id="iroutesource-destructor-iroutesource"></a>

### `IRouteSource::~IRouteSource`

```cpp
virtual ~IRouteSource() = default
```

Polymorphic-base boilerplate: a virtual destructor so a source held as `IRouteSource&`/`*` destroys correctly, with the copy/move set re-defaulted because declaring the destructor suppresses them. The base carries no state.

*function, declared at [`include/shulib/config/route.hpp:565`](../../include/shulib/config/route.hpp#L565).*

<a id="iroutesource-iroutesource"></a>

### `IRouteSource::IRouteSource`

```cpp
IRouteSource() = default
```

Polymorphic-base boilerplate: a virtual destructor so a source held as `IRouteSource&`/`*` destroys correctly, with the copy/move set re-defaulted because declaring the destructor suppresses them. The base carries no state. Default-constructible: the base has nothing to initialize.

*function, declared at [`include/shulib/config/route.hpp:567`](../../include/shulib/config/route.hpp#L567).*

<a id="iroutesource-iroutesource-2"></a>

### `IRouteSource::IRouteSource (overload 2)`

```cpp
IRouteSource(const IRouteSource&) = default
```

Polymorphic-base boilerplate: a virtual destructor so a source held as `IRouteSource&`/`*` destroys correctly, with the copy/move set re-defaulted because declaring the destructor suppresses them. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means.

*function, declared at [`include/shulib/config/route.hpp:569`](../../include/shulib/config/route.hpp#L569).*

<a id="iroutesource-iroutesource-3"></a>

### `IRouteSource::IRouteSource (overload 3)`

```cpp
IRouteSource(IRouteSource&&) = default
```

Polymorphic-base boilerplate: a virtual destructor so a source held as `IRouteSource&`/`*` destroys correctly, with the copy/move set re-defaulted because declaring the destructor suppresses them. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor.

*function, declared at [`include/shulib/config/route.hpp:571`](../../include/shulib/config/route.hpp#L571).*

<a id="iroutesource-operator-eq"></a>

### `IRouteSource::operator=`

```cpp
IRouteSource& operator=(const IRouteSource&) = default
```

Polymorphic-base boilerplate: a virtual destructor so a source held as `IRouteSource&`/`*` destroys correctly, with the copy/move set re-defaulted because declaring the destructor suppresses them. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning.

*function, declared at [`include/shulib/config/route.hpp:573`](../../include/shulib/config/route.hpp#L573).*

<a id="iroutesource-operator-eq-2"></a>

### `IRouteSource::operator= (overload 2)`

```cpp
IRouteSource& operator=(IRouteSource&&) = default
```

Polymorphic-base boilerplate: a virtual destructor so a source held as `IRouteSource&`/`*` destroys correctly, with the copy/move set re-defaulted because declaring the destructor suppresses them. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning. Move-assignable, same reasoning.

*function, declared at [`include/shulib/config/route.hpp:575`](../../include/shulib/config/route.hpp#L575).*

<a id="iroutesource-route"></a>

### `IRouteSource::route`

```cpp
[[nodiscard]] virtual const Route* route(std::string_view name) const noexcept = 0
```

The route called `name`, or NULLPTR when this source has no such route. A null return is the whole error channel on purpose: "the route you asked for is not here" is a normal thing for a program to ask (an auton selector offers what exists), not an exception.

*function, declared at [`include/shulib/config/route.hpp:580`](../../include/shulib/config/route.hpp#L580).*

<a id="iroutesource-count"></a>

### `IRouteSource::count`

```cpp
[[nodiscard]] virtual std::size_t count() const noexcept = 0
```

How many routes this source carries — for an auton selector that lists them.

*function, declared at [`include/shulib/config/route.hpp:583`](../../include/shulib/config/route.hpp#L583).*

<a id="iroutesource-at"></a>

### `IRouteSource::at`

```cpp
[[nodiscard]] virtual const Route& at(std::size_t index) const = 0
```

The route at `index`, which must be < count(). Preconditioned, not bounds-returned: an out-of-range index here is a programming error, unlike a missing NAME.

*function, declared at [`include/shulib/config/route.hpp:587`](../../include/shulib/config/route.hpp#L587).*

<a id="class-staticroutes"></a>

## `class StaticRoutes`

```cpp
class StaticRoutes final : public IRouteSource
```

An `IRouteSource` over a caller's array of routes — the implementation a generated header or a hand-typed table uses.  NON-OWNING: it holds a `std::span`, so the array must outlive the source. That is the opposite choice from `StaticRobotConfig` (which copies) and the reason is size: one config is ~400 bytes and one route is ~5 KB, so copying a table of them into a second buffer would double the largest static object on a brain with 128 KB of user RAM, to protect against a lifetime bug that a `constexpr` table at namespace scope — the only way either producer emits one — cannot have.

*class, declared at [`include/shulib/config/route.hpp:599`](../../include/shulib/config/route.hpp#L599).*

<a id="staticroutes-staticroutes"></a>

### `StaticRoutes::StaticRoutes`

```cpp
explicit StaticRoutes(std::span<const Route> routes) noexcept
```

View `routes`. The array must outlive this source. An empty span is legal: a robot with no routes yet is a robot, and `route()` will answer nullptr for everything.

*function, declared at [`include/shulib/config/route.hpp:603`](../../include/shulib/config/route.hpp#L603).*

<a id="staticroutes-route"></a>

### `StaticRoutes::route`

```cpp
[[nodiscard]] const Route* route(std::string_view name) const noexcept override
```

The route whose `name` matches exactly, or nullptr. Order is preserved and the FIRST match wins — duplicate names are not refused here, because a source is a view of someone else's table and refusing at lookup time would hide the duplicate rather than report it.

*function, declared at [`include/shulib/config/route.hpp:608`](../../include/shulib/config/route.hpp#L608).*

<a id="staticroutes-count"></a>

### `StaticRoutes::count`

```cpp
[[nodiscard]] std::size_t count() const noexcept override
```

How many routes this source views.

*function, declared at [`include/shulib/config/route.hpp:626`](../../include/shulib/config/route.hpp#L626).*

<a id="staticroutes-at"></a>

### `StaticRoutes::at`

```cpp
[[nodiscard]] const Route& at(std::size_t index) const override
```

The route at `index`, in the caller's array order. Precondition: index < count().

*function, declared at [`include/shulib/config/route.hpp:629`](../../include/shulib/config/route.hpp#L629).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 33 lines</summary>

```text

 Route — an autonomous routine as DATA (chunk G1 Part 1, 2026-09-18).

 ── Why this type exists, and why it carries no code ────────────────────────────────
 The team has tried to make autons data three times (a generated C array, a CSV, a
 hard-coded vector — docs/legacy-command-vocabulary.md §1) and what was missing every
 time was the RUNNER, not the format. G2 builds the runner. This header builds the thing
 it runs: a fixed-capacity, allocation-free, constexpr-constructible value that says
 where the robot goes and what it does on the way.

 The legacy planner embedded C++ snippets in its rows (`code_template:"mech.intakeIn();"`).
 That is rejected outright (ruling B5, master plan §16.3): a marker here is an ID plus
 TYPED ARGUMENTS, and binding an id to an action is the student's one line in the command
 registry. Data that can execute is not data — it is a second, unreviewable source file,
 and it is what makes "a non-coder authors the strategy" impossible to keep honest.

 ── The vocabulary is the facade's, plus Start and Hold ─────────────────────────────
 Every Motion below maps onto a verb chassis.hpp already has (ruling B5). A waypoint that
 needs a verb the library does not have is a request for a LIBRARY FEATURE, filed as
 such — never a silent no-op. That is why an unknown Motion byte is INVALID at valid()
 while an unknown MARKER id is not: G2 warns and skips an unknown id (§16.3), because
 marker ids are an open vocabulary a team extends, whereas the motion set is the library's
 own contract and a robot cannot guess what a verb it has never heard of should do.

 ── EXECUTION IS NOT HERE ───────────────────────────────────────────────────────────
 Nothing in this file runs a route. G2's PathRunner walks the waypoints, calls the facade
 verbs and fires the markers through the registry. Part 1 pins the SHAPE and the rules
 that say when the data is lying; that is deliberately all.

 NO HARDWARE CLAIM. No Route has ever been executed at all, on a plant or on a robot.

 PROS-free, sim-free, allocation-free; no std::string, no std::function, no pointers in
 the data types.
```

</details>
