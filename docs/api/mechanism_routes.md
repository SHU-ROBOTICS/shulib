<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/runner/mechanism_routes.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `mechanism_routes.hpp`

runner/mechanism_routes.hpp — a robot's MECHANISM ROUTE IDS (mechanism/route_ids.hpp), bound to its mechanism service and registered in a route registry in ONE call (the mechanism core, chunk MECH-A4, 2026-10-22; the team lead's ruling of 2026-10-08: "every…

This header declares **1** type (9 members) and **1** constant.

Extracted from [`include/shulib/runner/mechanism_routes.hpp`](../../include/shulib/runner/mechanism_routes.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMechanismRouteDocBytes`](#kmechanismroutedocbytes) — *constant*
- [`class MechanismRoutes`](#class-mechanismroutes)
  - [`MechanismRoutes`](#mechanismroutes-mechanismroutes)
  - [`MechanismRoutes (overload 2)`](#mechanismroutes-mechanismroutes-2)
  - [`operator=`](#mechanismroutes-operator-eq)
  - [`MechanismRoutes (overload 3)`](#mechanismroutes-mechanismroutes-3)
  - [`operator= (overload 2)`](#mechanismroutes-operator-eq-2)
  - [`~MechanismRoutes`](#mechanismroutes-destructor-mechanismroutes)
  - [`registerInto`](#mechanismroutes-registerinto)
  - [`ids`](#mechanismroutes-ids)
  - [`doc`](#mechanismroutes-doc)

<a id="kmechanismroutedocbytes"></a>

## `kMechanismRouteDocBytes`

```cpp
inline constexpr std::size_t kMechanismRouteDocBytes = 112
```

The longest doc one derived id carries, bytes (NUL included) — "intake: run until the piece is gone; args[0] = volts, args[1] = time limit, ms" with a 23-character name fits with room.

*constant, declared at [`include/shulib/runner/mechanism_routes.hpp:62`](../../include/shulib/runner/mechanism_routes.hpp#L62).*

<a id="class-mechanismroutes"></a>

## `class MechanismRoutes`

```cpp
class MechanismRoutes
```

One robot's mechanism route ids, bound to its chassis and mechanism service (file header). ONE lives in the program's static storage beside its registry (the registry keeps pointers into it); register after the service is bound. Non-copyable and non-movable for the same reason.

*class, declared at [`include/shulib/runner/mechanism_routes.hpp:67`](../../include/shulib/runner/mechanism_routes.hpp#L67).*

<a id="mechanismroutes-mechanismroutes"></a>

### `MechanismRoutes::MechanismRoutes`

```cpp
MechanismRoutes(chassis::Chassis& chassis, mechanism::MechanismService& service, const mechanism::MechanismTable& robot, hal::ITelemetrySink& telemetry) noexcept
```

Derive `robot`'s ids (routeIdsOf — the build already refused a list whose ids cannot be published) and write each one's doc. Borrows all four: they must outlive this object (static storage in a program). Registers nothing — registerInto() does.

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:72`](../../include/shulib/runner/mechanism_routes.hpp#L72).*

<a id="mechanismroutes-mechanismroutes-2"></a>

### `MechanismRoutes::MechanismRoutes (overload 2)`

```cpp
MechanismRoutes(const MechanismRoutes&) = delete
```

Non-copyable: the registry holds pointers into this object.

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:82`](../../include/shulib/runner/mechanism_routes.hpp#L82).*

<a id="mechanismroutes-operator-eq"></a>

### `MechanismRoutes::operator=`

```cpp
MechanismRoutes& operator=(const MechanismRoutes&) = delete
```

Non-copyable: the registry holds pointers into this object. Non-copy-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:84`](../../include/shulib/runner/mechanism_routes.hpp#L84).*

<a id="mechanismroutes-mechanismroutes-3"></a>

### `MechanismRoutes::MechanismRoutes (overload 3)`

```cpp
MechanismRoutes(MechanismRoutes&&) = delete
```

Non-copyable: the registry holds pointers into this object. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason).

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:86`](../../include/shulib/runner/mechanism_routes.hpp#L86).*

<a id="mechanismroutes-operator-eq-2"></a>

### `MechanismRoutes::operator= (overload 2)`

```cpp
MechanismRoutes& operator=(MechanismRoutes&&) = delete
```

Non-copyable: the registry holds pointers into this object. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:88`](../../include/shulib/runner/mechanism_routes.hpp#L88).*

<a id="mechanismroutes-destructor-mechanismroutes"></a>

### `MechanismRoutes::~MechanismRoutes`

```cpp
~MechanismRoutes() = default
```

Non-copyable: the registry holds pointers into this object. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason). Nothing to release.

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:90`](../../include/shulib/runner/mechanism_routes.hpp#L90).*

<a id="mechanismroutes-registerinto"></a>

### `MechanismRoutes::registerInto`

```cpp
void registerInto(CommandRegistry& registry)
```

THE ONE CALL: every id into `registry`, in the list's order, each with its doc and its arg count. PRECONDITIONS: this robot's ids can be published (the build refuses a list whose ids cannot, so a hand-built table is the only way here); and the registry's own — room for every id, and none already registered (calling this twice is a duplicate).

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:96`](../../include/shulib/runner/mechanism_routes.hpp#L96).*

<a id="mechanismroutes-ids"></a>

### `MechanismRoutes::ids`

```cpp
[[nodiscard]] const mechanism::RouteIds& ids() const noexcept
```

The ids, as derived (routeIdsOf).

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:105`](../../include/shulib/runner/mechanism_routes.hpp#L105).*

<a id="mechanismroutes-doc"></a>

### `MechanismRoutes::doc`

```cpp
[[nodiscard]] const char* doc(std::size_t i) const
```

Id `i`'s doc, as the manifest publishes it. PRECONDITION: `i` below ids().count.

*function, declared at [`include/shulib/runner/mechanism_routes.hpp:107`](../../include/shulib/runner/mechanism_routes.hpp#L107).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 37 lines</summary>

```text

 runner/mechanism_routes.hpp — a robot's MECHANISM ROUTE IDS (mechanism/route_ids.hpp), bound to its
 mechanism service and registered in a route registry in ONE call (the mechanism core, chunk
 MECH-A4, 2026-10-22; the team lead's ruling of 2026-10-08: "every mechanism publishes its actions as
 route ids AUTOMATICALLY ... a coder no longer registers each id by hand").

 ── One call ─────────────────────────────────────────────────────────────────────────
     static runner::CommandRegistry registry;                 // static storage, a program's
     static runner::MechanismRoutes routes{chassis, service, kRobotTwoMechanisms, sink};
     routes.registerInto(registry);                           // after the service is bound
 Every id the robot's list derives — `arm_score`, `intake_until_piece`, `claw_closed` — is then
 registered, in the list's order, each with a one-line doc the manifest publishes for a route
 editor's marker picker ("claw pivot 6: run; args[0] = volts (-12..12), args[1] = how long, ms").

 ── A marker runs EXACTLY the step an auton's one line runs ─────────────────────────
 Through a chassis::Routine built on the same chassis and service — one verdict policy, not two
 (chassis/routine.hpp's MECH-A4 block): `arm_score [1500]` is `r.move(arm["score"], 1.5_s)`,
 `intake_until_piece [12, 3000]` is `r.run(intake, 12_volt, seesPiece, 3_s)`, `claw_closed` is
 `r.set(claw["closed"])`. The marker BLOCKS at its waypoint until the step's verdict (markers fire
 only after a leg SETTLES — F8's ruling R2 — so a route marker never runs while driving; a step that
 starts mid-leg is a Routine's PARTWAY TRIGGER since STEP1-B, `r.atPercent(60).startMove(...).driveTo(...)`
 (chassis/routine.hpp) — a Routine form only: route markers stay blocking, F8 unchanged). SUCCEEDED — and
 UNCONFIRMED counted done (robot two's sensorless grab, the team lead's Q2 (c)) — continue the
 route; every other verdict returns to the PathRunner, whose policy stops the route (MarkerFailed,
 the verdict in its line). A flywheel's spin and an arm's hold go on after their marker (the service
 keeps them); the run guard's end of run makes every mechanism safe. A long time limit on a marker
 is a long stop at its waypoint (the brief's landmine 23).

 ── A marker's args are DATA a route author wrote ───────────────────────────────────
 So they are checked HERE, never trusted: a time limit is required and above 0 (P4: every action
 bounded; P6: the limit is the route author's, never a library default); a power id's volts are
 within ±12 and not 0. A marker that fails is REFUSED (manipulation::MechanismOutcome::Refused) with
 ONE Warn naming the id and what it needs — `route marker 'arm_score' REFUSED: it needs its time
 limit in ms, above 0 (args[0])` — and the route stops there, nothing commanded.

 Lives in runner/ because it needs both the Routine (chassis/) and the registry; mechanism/ stays
 below chassis/. PROS-free, allocation-free. Nothing here has run on a robot.
```

</details>
