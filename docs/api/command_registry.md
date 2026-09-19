<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/runner/command_registry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `command_registry.hpp`

CommandRegistry — the binding from a marker's ID to the code that performs it.

This header declares **3** types (12 members) and **1** constant.

Extracted from [`include/shulib/runner/command_registry.hpp`](../../include/shulib/runner/command_registry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxCommands`](#kmaxcommands) — *constant*
- [`struct Handler`](#struct-handler)
  - [`manipulation::MechanismOutcome`](#handler-manipulation-mechanismoutcome)
  - [`ctx`](#handler-ctx)
- [`struct CommandEntry`](#struct-commandentry)
  - [`id`](#commandentry-id)
  - [`argCount`](#commandentry-argcount)
  - [`handler`](#commandentry-handler)
  - [`idView`](#commandentry-idview)
- [`class CommandRegistry`](#class-commandregistry)
  - [`CommandRegistry`](#commandregistry-commandregistry)
  - [`on`](#commandregistry-on)
  - [`on (overload 2)`](#commandregistry-on-2)
  - [`find`](#commandregistry-find)
  - [`count`](#commandregistry-count)
  - [`ids`](#commandregistry-ids)

<a id="kmaxcommands"></a>

## `kMaxCommands`

```cpp
inline constexpr std::size_t kMaxCommands = 32
```

The most command ids one registry may hold. A DESIGN constant, on `kMaxWaypoints`'s reasoning (B6): a robot with more than 32 distinct scoring capabilities is not a robot this bound is the problem for. Nothing measured says 32; it is the fixed capacity that keeps the table allocation-free.

*constant, declared at [`include/shulib/runner/command_registry.hpp:69`](../../include/shulib/runner/command_registry.hpp#L69).*

<a id="struct-handler"></a>

## `struct Handler`

```cpp
struct Handler
```

One bound action: a plain function pointer plus the opaque context it is called with — the type-erased, allocation-free pair `std::function` would otherwise be used for (file banner).  `fn` receives the whole `config::Marker`, not just its args, so a handler can read its own id (one function may serve several related ids) and its typed arguments without a second lookup. It returns a `manipulation::MechanismOutcome`: anything other than `Succeeded` stops the route under the runner's error policy, and the vocabulary has no conversion to bool, so a failed grab can never read as success by accident (the T2 guarantee).

*struct, declared at [`include/shulib/runner/command_registry.hpp:79`](../../include/shulib/runner/command_registry.hpp#L79).*

<a id="handler-manipulation-mechanismoutcome"></a>

### `Handler::manipulation::MechanismOutcome`

```cpp
manipulation::MechanismOutcome (*fn)(void* ctx, const config::Marker& marker) = nullptr
```

The action. Never null in a registered Handler (a precondition of `on`).

*function, declared at [`include/shulib/runner/command_registry.hpp:81`](../../include/shulib/runner/command_registry.hpp#L81).*

<a id="handler-ctx"></a>

### `Handler::ctx`

```cpp
void* ctx = nullptr
```

Whatever `fn` needs — the address of the caller's callable for the template `on()` overload, or anything at all for the explicit one. NON-OWNING: the pointee must outlive the registry. May be null when `fn` needs no context.

*field, declared at [`include/shulib/runner/command_registry.hpp:85`](../../include/shulib/runner/command_registry.hpp#L85).*

<a id="struct-commandentry"></a>

## `struct CommandEntry`

```cpp
struct CommandEntry
```

One registration, as the registry stores it: the id, how many typed args a marker carrying it must supply, and the bound action.  The id is a fixed-width NUL-terminated buffer rather than a `std::string_view` into the caller's memory, because a registration commonly reads `on("intake_in", …)` from a string literal but just as legitimately from a buffer a parser filled — and a table of views into somebody else's parse buffer is a dangling read waiting for G3.

*struct, declared at [`include/shulib/runner/command_registry.hpp:95`](../../include/shulib/runner/command_registry.hpp#L95).*

<a id="commandentry-id"></a>

### `CommandEntry::id`

```cpp
char id[config::kMaxMarkerIdBytes]{}
```

The id text, NUL-terminated (`commandIdValid` at registration, so a terminator always fits). Read it through `idView()` rather than directly.

*field, declared at [`include/shulib/runner/command_registry.hpp:98`](../../include/shulib/runner/command_registry.hpp#L98).*

<a id="commandentry-argcount"></a>

### `CommandEntry::argCount`

```cpp
std::uint8_t argCount = 0
```

How many typed args a marker with this id carries — DECLARED at registration, and published in the manifest so a picker can prompt for exactly that many. The library does not enforce it on a marker at run time (an unenforceable claim about someone else's handler); it is the contract the picker and the handler share.

*field, declared at [`include/shulib/runner/command_registry.hpp:103`](../../include/shulib/runner/command_registry.hpp#L103).*

<a id="commandentry-handler"></a>

### `CommandEntry::handler`

```cpp
Handler handler{}
```

The bound action.

*field, declared at [`include/shulib/runner/command_registry.hpp:105`](../../include/shulib/runner/command_registry.hpp#L105).*

<a id="commandentry-idview"></a>

### `CommandEntry::idView`

```cpp
[[nodiscard]] constexpr std::string_view idView() const noexcept
```

The id as text.

*function, declared at [`include/shulib/runner/command_registry.hpp:108`](../../include/shulib/runner/command_registry.hpp#L108).*

<a id="class-commandregistry"></a>

## `class CommandRegistry`

```cpp
class CommandRegistry
```

The id → action table a team fills once at startup, and `PathRunner` reads while a route runs (file banner). Fixed capacity, no heap, no `std::function`; every registration error is a precondition, because registration is code.

*class, declared at [`include/shulib/runner/command_registry.hpp:116`](../../include/shulib/runner/command_registry.hpp#L116).*

<a id="commandregistry-commandregistry"></a>

### `CommandRegistry::CommandRegistry`

```cpp
CommandRegistry() = default
```

An empty registry. The built-in ids (`config::kBuiltinCommands`) are live from here on without appearing in this table at all — they belong to the runner, not to a team.

*function, declared at [`include/shulib/runner/command_registry.hpp:120`](../../include/shulib/runner/command_registry.hpp#L120).*

<a id="commandregistry-on"></a>

### `CommandRegistry::on`

```cpp
void on(std::string_view id, Handler handler, std::uint8_t argCount = 0)
```

Bind `id` to `handler`; `argCount` is how many typed args a marker with this id carries (published in the manifest; 0 for the common case).  Preconditions, every one of them a programming error rather than a datum (file banner): `id` is `config::commandIdValid`; it is not one of `config::kBuiltinCommands`; it is not already registered; `handler.fn` is not null; `argCount <= config::kMaxMarkerArgs`; and the table has room (`kMaxCommands`).

*function, declared at [`include/shulib/runner/command_registry.hpp:129`](../../include/shulib/runner/command_registry.hpp#L129).*

<a id="commandregistry-on-2"></a>

### `CommandRegistry::on (overload 2)`

```cpp
template <typename Callable> void on(std::string_view id, Callable& callable, std::uint8_t argCount = 0)
```

Bind `id` to any callable, adapting its return type exactly as `Routine::then()` does: * `void` — the action always succeeds, * `bool` — `false` fails (reported as `MechanismOutcome::Unconfirmed`: the handler ran and the task did not happen, which is precisely what that verdict means; claiming `TimedOut` or `Stalled` would invent a pathology the `bool` never stated), * `manipulation::MechanismOutcome` — passed through unchanged, so only `Succeeded` continues the route and an `Unconfirmed` grab can never read as success. A callable returning anything else is a compile error naming the three. (`control::ExitReason`, which `then()` also accepts, is deliberately NOT accepted here: mapping a motion verdict onto a mechanism verdict would have to invent a meaning for `Running`, and a handler that wraps a facade verb can return the outcome it means.)  The callable may take the marker or take nothing — `on("lift_to_level", liftTo)` where `liftTo` reads `marker.args[0]`, or `on("intake_in", intakeIn)` where it does not.  NON-OWNING: a pointer to `callable` is stored, so it must outlive the registry. The `Callable&` parameter is what enforces that — a temporary lambda does not bind to it, so the dangling registration cannot be written (file banner).

*function, declared at [`include/shulib/runner/command_registry.hpp:172`](../../include/shulib/runner/command_registry.hpp#L172).*

<a id="commandregistry-find"></a>

### `CommandRegistry::find`

```cpp
[[nodiscard]] const Handler* find(std::string_view id) const noexcept
```

The handler bound to `id`, or NULLPTR when nothing is. A null return is the whole error channel on purpose (`IRouteSource::route()`'s convention): "no handler for this id" is a normal thing for a runner to find, and it answers it with a Warn and a skip.  Built-in ids are NOT in this table and this returns nullptr for them; the runner checks the built-ins first.

*function, declared at [`include/shulib/runner/command_registry.hpp:182`](../../include/shulib/runner/command_registry.hpp#L182).*

<a id="commandregistry-count"></a>

### `CommandRegistry::count`

```cpp
[[nodiscard]] std::size_t count() const noexcept
```

How many ids are registered (built-ins not included — they are never in this table).

*function, declared at [`include/shulib/runner/command_registry.hpp:192`](../../include/shulib/runner/command_registry.hpp#L192).*

<a id="commandregistry-ids"></a>

### `CommandRegistry::ids`

```cpp
[[nodiscard]] std::span<const CommandEntry> ids() const noexcept
```

Every registration, in REGISTRATION ORDER — which is the order the manifest publishes them in, so a diff of two manifests reads as a diff of two `on()` sequences.

*function, declared at [`include/shulib/runner/command_registry.hpp:196`](../../include/shulib/runner/command_registry.hpp#L196).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 48 lines, click to expand</summary>

```text

 CommandRegistry — the binding from a marker's ID to the code that performs it
 (chunk G2, 2026-09-18; master plan §16.3; docs/vexbuilder-integration.md §4.2).

 ── Why this table is the keystone of the no-code story ─────────────────────────────
 A Route (config/route.hpp) says WHERE the robot goes and WHAT to do on the way, and the
 "what" is an ID — never code. The legacy planner embedded C++ snippets in its rows
 (`code_template: "mech.intakeIn();"`), which is rejected outright (ruling B5): data that
 can execute is a second, unreviewable source file. This table is the other half of that
 decision. A coder writes ONE line per capability,

     runner::CommandRegistry registry;
     registry.on("intake_in", intakeIn);      // intakeIn is a callable that OUTLIVES this

 and from then on a non-coder authoring a path in VexBuilder picks "intake_in" from a list
 the library itself publishes (runner/manifest.hpp). The auton is data a non-coder writes;
 the handlers are code the coders maintain.

 ── No std::function, no heap, no std::string ───────────────────────────────────────
 A V5 brain has 128 KB of user RAM and this table may be built during an autonomous
 period, so every allocation is a risk taken for nothing. A Handler is therefore a plain
 function pointer plus an opaque context pointer — the C idiom, chosen because it is the
 only one that is both type-erased and allocation-free. The template `on()` overload
 builds that pair from any callable and stores a NON-OWNING pointer to it: the callable
 must outlive the registry, the same ownership rule `Routine::then()`'s action and
 `StaticRoutes`'s span already carry. That is enforced by the signature, not by a comment
 — the overload takes `Callable&`, so `on("x", []{ ... })` (a temporary lambda) does not
 compile, and the lifetime bug it would cause cannot be written.

 ── A handler runs SYNCHRONOUSLY, on the caller's task, between legs ────────────────
 PathRunner calls a handler from inside run(), after the leg it hangs off has settled, and
 waits for it to return. A handler that blocks for two seconds blocks the route for two
 seconds. That is the right default for "close the clamp, then drive away" and the wrong
 one for "spin the intake for the rest of the match"; asynchronous mechanism operations
 are chunk T3's, and the idiom until then is a handler that STARTS an operation and a
 later marker that confirms it (manipulation/mechanism_op.hpp).

 ── What is a programming error here, and what is data ──────────────────────────────
 Registration is code, so everything wrong with a registration is a PRECONDITION: a
 duplicate id (two handlers for one id is not a merge, it is a typo), a built-in id
 (library-owned — ruling R3), an id outside the vocabulary, a null function pointer, more
 than kMaxCommands. An UNKNOWN id at RUN time is the opposite: it is data, the runner logs
 one Warn and carries on (vexbuilder-integration.md's guarantee), and nothing about it
 reaches this class.

 FROZEN: F8 (docs/roadmap.md), pinned by test/f8_signature_pin_test.cpp.

 PROS-free, sim-free, allocation-free. Single-task, like everything it is called from.
```

</details>
