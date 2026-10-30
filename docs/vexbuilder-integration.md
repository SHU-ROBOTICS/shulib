# VexBuilder ↔ shulib — the complete integration contract

> **Who this is for:** whoever is finishing VexBuilder. It is written so you are never blindsided by
> something shulib has not built yet, or by a contract that is still allowed to move under you.
>
> **The short version:** shulib *defines* the contracts, VexBuilder *produces* them — and **shulib has
> not yet built the code that consumes any of them.** That is not a warning about your work, it is the
> single most important scheduling fact on this page, and §1 spells out exactly what does and does not
> exist on each side.
>
> Companion documents: the master plan's "VexBuilder Integration Contract" section owns the *why* and
> the design rationale; `roadmap.md` owns the milestones and the Freeze Register; this document owns
> the **operational detail** — every field, every ask, every ordering constraint, both directions.

---

## 0. Read this before you write a line

**shulib cannot open a `.vexbot` file today. At all.** There is no parser, no config type, no route
type, no path runner, no codegen tool. Everything in the G phase is unbuilt.

So **do not build against a shulib API** — there isn't one to build against yet. Build against the
**contract** in §3–§5. shulib implements the consuming half, and where this document and shulib's
eventual code disagree, **this document is wrong and gets fixed** — the contracts are shulib's to
define, so the burden is ours.

Three consequences worth internalising:

1. **You cannot integration-test against shulib yet.** Your acceptance test is "the file matches the
   schema in §3/§4", not "shulib loaded it". A schema-validation fixture on your side is worth
   building; shulib will ship the same fixtures once G3 exists.
2. **Nothing you emit is wasted if you follow §6's versioning rule.** Additive fields are free.
3. **One of the three seams is now FROZEN — F8** (`paths[]` + the command vocabulary), locked
   2026-09-18 with shulib's path runner; §4 is a contract, not a proposal. F7 is still a draft and
   F9 is not frozen (its format and reader exist since 2026-09-29, host-only — §5.2). §7 is the
   honest list of what can still move, and what we will do to stop it moving under you.

---

## 1. Status of every moving part, both sides

Legend: ✅ exists and works · ⚠️ exists but incomplete · ❌ not built

### VexBuilder's side

| Piece | Status | Note |
|---|---|---|
| `.vexbot` project file, schema v2.0.0 | ✅ | ships today |
| `migrateProject()` 1.1 → 2.0 | ✅ | the additive-migration pattern shulib mirrors |
| `joints[]` | ✅ | shulib ignores it |
| `electrical{motors,sensors,pneumatics}` | ⚠️ | **the arrays are empty because the electrical UI was never built.** This is the biggest single gap: it is the source data for `robotProfile` |
| Explicit drivetrain fields (`kind`/`trackWidth`/`wheelDiameter`) | ❌ | **ask #2** — shulib has a brittle inference fallback and does not want to use it |
| `project.robotProfile` block | ❌ | **ask #5** — schema in §3 |
| `project.paths[]` | ❌ | **ask #1** — schema in §4 |
| Command picker fed by shulib's manifest | ❌ | **ask #4 — UNBLOCKED: the manifest exists and its exact bytes are in §4.2.3** |
| Agent server + `server.json` (`port`, `token`, `pid`) | ✅ | **this exists** — it is the piece the programming chair is taking |
| Rapier physics sim | ❌ | "planned". Everything sim-related is downstream of it |
| `.vbpath` / standalone path planner | 🚫 | **retired by decision.** Paths live in `.vexbot` now — see §4.0 |

### shulib's side

| Piece | Status | Chunk |
|---|---|---|
| `IRobotConfig` / `IRouteSource` / `RobotBuilder` | 🚧 | **G1 Part 1 landed 2026-09-18, host-only** — the in-memory types (`config::RobotConfig`, `config::Route`, both seams) and the builder (`robot::Robot` over an `hal::IDeviceProvider`) exist and drive the host plant identically to a hand-wired graph. *Part 2a (the PROS provider, host-tested only) landed 2026-09-24; Part 2b ("shulib Teleop" built on the builder over the PROS provider — that version not yet run on a robot) landed 2026-09-26; Part 2c landed 2026-09-26, build-gated — `src/main.cpp` is a dispatcher and the invented X-drive a `constexpr RobotConfig` built on the builder, so every program that builds the library's object graph builds it from a config; the codegen that fills a config from a `.vexbot` is G3's. G1 is ungated: it waits on nothing from VexBuilder* |
| `PathRunner` + command-id registry + manifest export | 🚧 | **G2 landed 2026-09-18, host-only — F8 FROZEN.** The runner executes a `Route` through the frozen chassis verbs, the registry binds ids to handlers, and `writeManifest` publishes the vocabulary (§4.2.3). Proven on the host physics plant against a hand-written recipe, bit for bit; *never run on a robot*. The `.vexbot` READER is G3's |
| The robot profile revised (§3.1), its generator, and an optional VS Code robot panel | ❌ | **G3a** — added 2026-09-26, before G3; the profile can change until G3 freezes it |
| `.vexbot` ingestion (into G3a's generator) + SD runtime loader | ❌ | **G3** — freezes F7 |
| The 10-minute guide | ❌ | **G4** — closes the accessibility milestone (the `.shupaths` importer was dropped on 2026-09-26) |
| `SHUL/2` wire protocol | 🚧 | **H1** — freezes F9. **Part A1 landed 2026-09-29, host-only: the format and its reader exist (§5.2); NOT frozen yet.** The writer (the sink) is Part A2; the freeze is Part B. The wire has never crossed a real link. *Ungated: the wire is defined unilaterally* |
| `hal/sim` adapter + record/replay | ❌ | **H2** — needs your Rapier sim |
| On-brain live tuner | ❌ | **H3** |
| **`DebugRecord`** — the record SHUL/2 serialises | ✅ | exists, stable, and already has reserved space for this (§5.2) |
| **Blackbox binary format** — a shipped, versioned, round-trip-tested wire format | ✅ | **the precedent SHUL/2 followed**: its Tick and Summary payloads ARE the blackbox's bytes (§5.2.5) |
| The HAL, motion, localization, sequencing, both auton-authoring tiers | ✅ | done and frozen (but for the pieces the Freeze Register leaves outside: the recipe layer's `then()` and — not yet ruled — its mechanism steps, row F10; the mechanism ids of 4.2.3 are a robot's, never frozen) — this is what a loaded `.vexbot` will drive |

**The honest read:** your side is further along on the file format; our side is further along on the
runtime. The integration is the middle, and neither side has built it.

---

## 2. The three seams, and only three

shulib reads exactly three things from VexBuilder. Everything else in `.vexbot` — parts, holes,
joints, render data — is **ignored by contract**, so you are free to change it without telling us.

| # | Seam | Direction | Carrier | Freeze row |
|---|---|---|---|---|
| 1 | Robot configuration | VexBuilder → shulib | `project.robotProfile` in `.vexbot` | **F7** |
| 2 | Routines + command vocabulary | both | `project.paths[]` in `.vexbot`, plus shulib's id manifest | **F8** |
| 3 | Simulation + telemetry | bidirectional, live | `SHUL/2` over your agent socket | **F9** |

---

## 3. Seam 1 — `project.robotProfile` (F7)

### 3.1 The schema shulib specifies

```
project.robotProfile {
  schemaVersion: int,
  identity   { name, team, season },
  drivetrain {
    kind: "x" | "h" | "tank",
    wheelDiameter: inches,
    trackWidth: inches,          // or geometry{} for non-rectangular
    motors: [ { port: 1..21, reversed: bool, cartridge: "red"|"green"|"blue",
                side: "left"|"right"|"fl"|"fr"|"bl"|"br",
                gearRatio: { motorTeeth: int, wheelTeeth: int } } ]
  },
  odometry {
    wheels: [ { port, reversed, diameter, offset, role: "forward"|"lateral" } ],
    imuPort: int|null,
    gpsPort: int|null
  },
  sensors:    [ { port, type: "distance"|"optical"|"rotation"|"vision"|"imu"|"gps" } ],
  mechanisms: [ { name,                          // unique, at most 23 characters
                  motors: [ { port: 1..21, reversed: bool } ],
                  cartridge: "red"|"green"|"blue",   // REQUIRED when motors is non-empty
                  safeMode: "coast"|"brake"|"hold",  // REQUIRED when motors is non-empty
                  pneumatics: [ "A".."H" ],          // the brain's 3-wire ports; at most 2
                  pneumaticSafe: "low"|"high",       // REQUIRED when pneumatics is non-empty
                  pneumaticStart: "low"|"high" } ],  // REQUIRED when pneumatics is non-empty
                  // motors OR pneumatics in one mechanism, never both
  corrections { x, y, theta }
}
```

### 3.2 Field notes that will save you a rewrite

- **Units are inches and degrees at this boundary.** shulib works internally in inches, radians and
  seconds, and converts once at the edge. Emit degrees; we convert.
- **Ports are 1-indexed (1–21)**, matching every PROS device API. A negative port means reversed in
  PROS's own convention — but **emit `port` and `reversed` as separate fields**, not a signed port.
  shulib translates. *(There is a real trap here: one PROS registry call is 0-indexed while every
  device API is 1-indexed, and mixing them yields plausible wrong answers with no error. Keep the
  boundary explicit and neither side can make that mistake.)*
- **`gearRatio` is new to this schema and it is not optional.** When this was written shulib had **no
  gear-ratio concept anywhere** — a known defect; since then `hal::DriveGeometry` carries a per-side
  motor-to-wheel ratio (R3b) and a mechanism states its gearing as tooth counts (`gears(12, 60)`, the
  mechanism core, 2026-10-08), but nothing reads a `.vexbot` yet. A real team drivetrain has an external reduction
  between the motor cartridge and the wheel, and it may **differ between the left and right sides**.
  Emit it per motor. If a drive is direct, emit `1:1` explicitly rather than omitting the field.
- **`cartridge` is a software declaration, not a measurement.** A V5 motor cannot report its physical
  cartridge; PROS only echoes what you configured. Whatever your UI collects here is trusted
  absolutely, and a wrong value is a silent 3× error in every velocity. Make the UI make it hard to
  get wrong.
- **Every mechanism with motors needs its own `cartridge` and a `safeMode`, and neither has a
  default.** shulib builds each such mechanism from the profile (since chunk T3, 2026-09-26), asking
  for its motors with the MECHANISM's cartridge — a RED claw on a BLUE drive is the normal case, so
  the drivetrain's value is never borrowed. `safeMode` is where the mechanism goes whenever anything
  stops it: `"hold"` keeps a loaded arm or claw where it is, `"coast"` or `"brake"` suit an intake
  that must not sit at stall current. There is no safe choice to default to — a missing value
  defaulting to coast would drop a loaded arm — so a mechanism with motors and either field missing
  is refused by name. Two things that must move separately (two claws, say) are **two
  mechanisms**, never one mechanism with two motors: one mechanism drives all its motors together.
  A mechanism's `motors` may not share a port with the drivetrain or with each other; names must be
  unique (they are how a program finds a mechanism).
- **A pneumatic mechanism needs a `pneumaticSafe` and a `pneumaticStart` level, and neither has a
  default** (since T3's addendum B2, 2026-09-26, shulib builds these too). `"low"` is the line off
  — the solenoid unpowered — and `"high"` is the line on. `pneumaticStart` is what the valve is set
  to **the moment the robot is built**: making a 3-wire line drives it, so there is no "unset" moment
  at boot to hide in. (On a field the robot is built while the brain is disabled; whether the valve
  takes the level then, or only once the robot is enabled, is not yet measured.) `pneumaticSafe` is
  where an end-of-run stop puts it. Whether `"high"` opens a claw or closes it depends on how the
  valve is plumbed, which only the team knows — so a missing value is refused by name, never
  defaulted to `"low"`. **One mechanism is motors OR pneumatics,
  never both** (a claw pivot and the claw's air are two mechanisms); a mixed entry is refused. Emit
  the 3-wire ports as the brain's letters A–H: shulib's config stores them as the numbers 1–8
  (A = 1). A port may appear once across all mechanisms. **Two ports in one mechanism are two
  valves fired together at the same level**, not the two coils of a double-coil solenoid (which
  need opposite levels: since the mechanism core's first part, 2026-10-08, shulib's config can
  state a level PER LINE — `pneumaticStartByLine` / `pneumaticSafeByLine` — and builds such a
  mechanism so its safe state drives each line to its own level; this contract has no field for
  them yet, so do not emit one). Nothing on the brain can
  confirm a solenoid is plugged into a 3-wire port — the UI is the only check there is.
- **`role` on odometry wheels is required**, not inferred. shulib's odometry refuses two wheels of the
  same role, which is the guard that stops a forward and a lateral pod being swapped.
- **`corrections{}`** is the per-robot calibration slice (wheel scale, lever arm, mount offsets). It
  may be absent; shulib defaults it to identity.

### 3.3 Where the data comes from — and the problem

`robotProfile` is meant to be **derived from `electrical{}` plus the explicit drivetrain fields.**
Today `electrical{}` is empty because the UI was never built, so **there is no source data**. That
makes ask #2 and ask #5 effectively one job: the electrical UI is the prerequisite for the profile.

shulib ships an `inferDrivetrain()` fallback that guesses kind/track/diameter from part geometry.
**It is documented as brittle and it is not the contract.** Treat it as a migration aid for old files,
never as a reason to skip the explicit fields.

### 3.4 How shulib will consume it

Two paths, same in-memory types behind `IRobotConfig`:

1. **Codegen (primary).** A host-side tool reads `.vexbot` → emits a typed `robot_config.hpp` with
   both the profile and the routines as `inline constexpr`. No runtime JSON, no SD dependency,
   compile-checked.
2. **SD-card runtime (optional).** shulib reads the `.vexbot` off the SD card at boot, so a non-coder
   re-exports and re-runs **without recompiling**.

Either way the promise is *"VexBuilder file → working robot + auton in two lines."* And the standalone
promise is preserved: a code-fluent team builds the same types directly in C++ with no file at all.

---

## 4. Seam 2 — `project.paths[]` and the command vocabulary (F8)

### 4.0 The decision you need to know about first

**`.shupaths` and the standalone Python path planner are RETIRED.** Paths now live inside `.vexbot`
alongside the robot, so a routine can never fall out of version-sync with the robot it was drawn for.
This deletes the spec's unsolved "re-export when the robot changes" problem outright. Nothing writes
`.shupaths` files again, and nothing reads them: the one-way importer once planned was dropped on
2026-09-26.

### 4.1 The schema — **FROZEN 2026-09-18 (register row F8, chunk G2)**

This section used to be a proposal. It is now a **contract**: G2 built the runner that executes this
vocabulary, and the in-memory types, the enum values and the field names below are pinned by a
compile-time freeze test (`test/f8_signature_pin_test.cpp`) that fails shulib's build if any of them
moves. G3 writes the parser against exactly this and nothing else.

**The names below changed from the proposal, and the changes are listed in 4.1.3.** They changed in
one direction only: *the document converged on the code*. shulib executes `config::Route` today, on
the host plant, and a schema field that did not map onto something the library can actually do would
have been a promise no parser could keep.

```
project.paths: [ {
  schemaVersion: int,            // 1
  name: string,                  // 1..31 bytes; the key an auton selector looks up
  waypoints: [ {                 // 1..32 of them
    x, y,                        // inches, field frame, finite
    heading,                     // DEGREES, finite (read only when headingMode is "fixed")
    headingMode: "fixed" | "free",
    motion: "start" | "move" | "drive" | "turn" | "strafe" | "hold",
    reverse: bool,               // "drive" only
    constraints: { maxLinearSpeed?, maxAngularSpeed?, timeout? },
    markers: [ { id: string, args?: [int, int] } ]   // 0..4 of them
  } ]
} ]
```

#### 4.1.1 What each value MEANS — the verb table

Every `motion` is a verb shulib has **today**; there are no aspirational values in this list, which
is the whole reason the list is short.

| `motion` | What the robot does | `headingMode` | Reads `x`,`y` | Reads `heading` | `reverse` |
|---|---|---|---|---|---|
| `"start"` | Not a motion: *"the robot begins HERE."* Seeds the pose estimate's position. **Legal at index 0 only** | either | yes | when `"fixed"` — the heading a competition auton is seated to; a `"free"` start states none | no |
| `"move"` | Translate and rotate to the full pose, simultaneously | `"fixed"` → arrive at `heading`; `"free"` → keep the heading the robot has when the leg starts | yes | when `"fixed"` | no |
| `"drive"` | Drive to the point, **arriving facing it** — the heading is computed from the approach | **`"free"` only** | yes | no | **yes** — adds 180°, so the robot arrives backwards onto the point. On a tank, author a `"turn"` that puts the robot's back to the point first: a reversed leg that starts facing the point is not steered and times out beside it (measured on the host test plant, 2026-09-29) |
| `"turn"` | Rotate in place to `heading` | **`"fixed"` only** | no | yes | no |
| `"strafe"` | Translate holding the current heading | either | yes | no | no |
| `"hold"` | Actively hold the pose for `constraints.timeout` | either | no | no | no |

**`reverse` on anything but `"drive"` is refused**, not ignored: `"drive"` is the only verb with an
approach direction to reverse, and a stated value that nothing reads is the quieter of the two bugs.

#### 4.1.2 Constraints, units, and the zero sentinel

| Field | Unit | Rule |
|---|---|---|
| `maxLinearSpeed` | **inches per second** | finite, ≥ 0. **0 or absent = the robot's configured default** |
| `maxAngularSpeed` | **radians per second** | finite, ≥ 0. 0 or absent = the default |
| `timeout` | **seconds** | finite, ≥ 0. 0 or absent = the default — **except on `"hold"`, where this field IS the hold duration and must be > 0** |

Units are canonical shulib units and are not negotiable per-file: inches, seconds, radians per second
(`heading` is the one exception — **degrees in the file**, because that is what a person draws with,
converted once at parse). A route whose numbers are not finite is refused with a message naming the
waypoint and the field.

#### 4.1.3 The rename table — proposal → frozen

| Was (proposal) | Is (frozen) | Why |
|---|---|---|
| `headingMode: "locked"` | **`"fixed"`** | the in-memory enumerator is `HeadingMode::Fixed`; one spelling everywhere beats a translation table nobody maintains |
| `headingMode: "tangent"` | **`motion: "drive"`** | "tangent" was a heading MODE describing a MOTION. `"drive"` already means *"arrive facing the point"*, and the runner computes that heading, so the concept survives with a better home |
| `motion: "move"` | `"move"` (unchanged), **plus `"start"`, `"drive"`, `"hold"`** | the proposal had no way to say "begin here", "arrive facing it" or "stay put", all of which every real auton needs |
| `motion: "arc"` | **DROPPED — a named gap, see below** | no facade verb performs an arc, and F8 must not freeze something the library cannot do |
| `markers[].args: { level: 3 }` | **`args: [3]`** — a positional array of **at most 2 integers** | the in-memory type is `int32_t args[2]`. A picker labels the positions from the **manifest**, which is a better place for a label than inside every path file |
| *(unstated)* | `markers[].id` is **1–23 bytes of `[a-z0-9_]`** | 4.2 |
| *(unstated)* | ≤ **32** waypoints, ≤ **4** markers per waypoint | design bounds, frozen so a generator can size its output |

**The `"arc"` gap, stated out loud so it is not mistaken for an oversight.** shulib has no arc verb.
Blended/spline trajectories are a real item on shulib's roadmap, and when one lands, `"arc"` is
**added as a new value** — which is an ADDITIVE change under the versioning contract in §6 and costs
your parser nothing. Do not emit `"arc"` before then; a reader that meets an unknown `motion` refuses
the route by name rather than guessing.

### 4.2 The command-id rule — the keystone of the whole no-code story

The legacy planner embedded **C++ snippets** in path data (`code_template: "mech.intakeIn();"`).
**That is rejected outright.** It couples data to code and it is the exact thing this design exists to
eliminate.

Instead a marker carries an **id**, and the student registers a handler once:

```text
runner::CommandRegistry registry;
registry.on("intake_in", intakeIn);        // intakeIn is a callable that OUTLIVES the registry

runner::PathRunner runner{chassis, registry, telemetry};
const runner::RunReport report = runner.run(*routes.route("left_side_rush"));
```

The auton becomes **data a non-coder authors in VexBuilder, executed by a library the coders maintain.**

#### 4.2.1 The id vocabulary rule — FROZEN

**An id is 1 to 23 characters of `[a-z0-9_]`.** Lower case, digits and underscore; nothing else.
No spaces, no punctuation, no capitals, no non-ASCII. This is deliberate and it is the reason
**nothing in this system ever needs escaping**: the same byte sequence is a C++ string literal, a
manifest line, a `.vexbot` field and a button label, on every platform. An id outside the charset is
refused by shulib at the door, both at registration and when a route is validated — it is not an
"unknown id", because no registry could ever hold it.

#### 4.2.2 The four LIBRARY-OWNED built-in ids — FROZEN

These four always exist, on every robot, with no registration. **A team cannot re-bind them** (doing
so is a hard error in shulib), so a route that uses them means the same thing in every program:

| id | args | Meaning |
|---|---|---|
| `wait_ms` | `[ms]` — exactly one, **> 0** | wait, commanding nothing, for `args[0]` milliseconds |
| `brake` | none | stop the drive (0 V + Brake) and block until the estimate certifies rest |
| `hold_ms` | `[ms]` — exactly one, **> 0** | actively hold the current pose for `args[0]` milliseconds |
| `log` | 0 to 2 | write one Info line naming this marker and its args — the non-coder's print statement |

A built-in carrying the wrong number of args, or a non-positive duration, makes the route invalid
before it runs — deliberately, because the alternative is a routine that throws mid-autonomous.

#### 4.2.3 The manifest — ask #4, and it now EXISTS

shulib owns the canonical id vocabulary and publishes it. `runner::writeManifest(registry, sink)`
emits **one JSON object per line** — not one JSON document, because a line-oriented file is what the
sink's contract carries, what keeps your parser trivial, and what makes two builds diffable:

```
{"manifestSchemaVersion":1,"builtins":4,"commands":2}
{"id":"wait_ms","builtin":true,"minArgs":1,"maxArgs":1,"doc":"wait, commanding nothing, for args[0] milliseconds (the drive keeps its last state)"}
{"id":"brake","builtin":true,"minArgs":0,"maxArgs":0,"doc":"stop the drive (0 V + Brake) and block until the estimate certifies rest"}
{"id":"hold_ms","builtin":true,"minArgs":1,"maxArgs":1,"doc":"actively hold the current pose for args[0] milliseconds, driving back disturbance"}
{"id":"log","builtin":true,"minArgs":0,"maxArgs":2,"doc":"write one Info line naming this marker and its args"}
{"id":"intake_in","builtin":false,"minArgs":0,"maxArgs":0,"doc":""}
{"id":"lift_to_level","builtin":false,"minArgs":1,"maxArgs":1,"doc":""}
```

- Line 1 is the header; every later line is one command, with the **same five fields** every time
  (`doc` is `""` for an id a team registered without one — the library does not know its meaning;
  since the mechanism core's fourth part, 2026-10-29, an id MAY be registered with its one-line doc,
  and every mechanism id below is).
- **Order is part of the contract:** the built-ins first, in the order above, **append-only**; then
  the team's ids in **registration order**. A byte-exact golden test in shulib pins these bytes, so
  a change to the file your picker parses cannot happen quietly.
- `minArgs`/`maxArgs` tell the picker exactly how many argument fields to render.
- Nothing in the file is escaped, because 4.2.1 makes escaping unnecessary (a doc carrying a `"` or a
  `\` is refused when it is registered).

**Every mechanism's actions are ids too — derived, never typed (since the mechanism core's fourth part,
2026-10-29).** A robot's mechanisms are declared once in its robot file, and their ids are DERIVED from
those declarations, one rule per kind — an arm's or a lift's `<name>_<preset>` and `<name>_home`, a
flywheel's `<name>_<preset>` and `<name>_stop`, a power mechanism's `<name>_run`,
`<name>_until_piece` / `<name>_until_gone` (when it has a piece sensor) and `<name>_stop`, a
pneumatic's `<name>_<position>` (the name lower-cased, every run of other characters one `_`: "claw
pivot 6" → `claw_pivot_6`). A program registers all of a robot's ids in one call, each with its doc, so
the manifest carries them like any other registered id.

> **Status: host-tested only — registered by two programs, written to no manifest yet.** Since the
> mechanism core's fifth part (2026-10-29) the competition program and the Auton Tester register robot
> two's six ids at boot (their `ROUTES:` boot line counts them), so a route's markers can fire them;
> no program writes a manifest, so no manifest a robot writes today contains them. The six lines below
> are derived from a test fixture's copy of robot two's declarations, held byte for byte equal to the
> robot file's region by a host test. Where a robot's manifest FILE lives for a picker to read is the
> robot panel's work, not yet built.

Robot two's six, exactly as the manifest writes them (after the header and the built-ins):

```
{"id":"claw_pivot_6_run","builtin":false,"minArgs":2,"maxArgs":2,"doc":"claw pivot 6: run; args[0] = volts (-12..12), args[1] = how long, ms"}
{"id":"claw_pivot_6_stop","builtin":false,"minArgs":0,"maxArgs":0,"doc":"claw pivot 6: stop (its safe state)"}
{"id":"claw_pivot_7_run","builtin":false,"minArgs":2,"maxArgs":2,"doc":"claw pivot 7: run; args[0] = volts (-12..12), args[1] = how long, ms"}
{"id":"claw_pivot_7_stop","builtin":false,"minArgs":0,"maxArgs":0,"doc":"claw pivot 7: stop (its safe state)"}
{"id":"claw_closed","builtin":false,"minArgs":0,"maxArgs":0,"doc":"claw: go to 'closed'"}
{"id":"claw_open","builtin":false,"minArgs":0,"maxArgs":0,"doc":"claw: go to 'open'"}
```

What a picker should know about them:

- **Their args.** An arm's, a lift's or a flywheel's id carries ONE arg: its **time limit in
  milliseconds** (`arm_score [1500]`). A power mechanism's `_run`, `_until_piece` and `_until_gone` carry
  TWO: **volts** (−12..12, not 0) and a **time in milliseconds** (`intake_until_piece [12, 3000]`). A
  `_stop` and a pneumatic's position carry none (a valve's move time is the robot file's). The library
  states no default for any of them: a marker missing one is REFUSED at run time with one warning
  naming the id and what it needs, and the route stops there — so a picker that renders `minArgs`
  fields and refuses an empty or zero one saves a student that run.
- **A mechanism marker BLOCKS** its waypoint until the step's verdict (markers fire only after a leg
  settles, 4.1.1 / ruling R2): a long time limit is a long stop there. SUCCEEDED — and a sensorless grab
  counted done — let the route go on; any other verdict stops it (`MarkerFailed`).
- **The ids are a robot's**, and they change when its robot file does (a preset renamed, a mechanism
  added). Derive the list from the robot it is for; never hardcode one. A robot publishes at most 32
  (the registry's capacity), and its build refuses an id longer than 23 characters, two ids alike, or an
  id that is a built-in.

#### 4.2.4 The unknown-id guarantee, now implemented and tested

**An unknown id logs one warning naming the id and the waypoint index, is counted, and is skipped.
The route continues. It never crashes.** This is no longer a promise — it is `PathRunner`'s
behaviour, with a test that breaks it deliberately and watches it fail. A typo in a picker must not
end a match, and it must not be silent either: the count comes back in `RunReport::unknownIds`, which
is the first number to look at when a route "ran fine" and scored nothing.

### 4.3 The state of the vocabulary today — read this before building the picker

**The manifest now exists (it shipped with G2, 4.2.3).** What the legacy audit found, and where each
legacy id ended up:

| Legacy id | Ever actually emitted? | Fate |
|---|---|---|
| `MOVE_WITH_HEADING` | **yes** — 826 CSV rows + 635 generated + 10 hand-written | covered: it is `motion: "move"` with `headingMode: "fixed"` |
| `NONE` | **yes** — 18 rows, and **absent from both legacy enums** | **ANSWERED — see below. It is not an id and never will be** |
| `PICK_UP`, `PLACE`, `SCOOP`, `RELEASE`, `CLASP` | **no — declared in an enum, never emitted** | planned scoring primitives, not yet built; they will arrive as ordinary registered ids in the manifest. What a robot's MECHANISMS do already arrives that way — each declared mechanism's derived ids (4.2.3), since 2026-10-29 |

**Only one motion id was ever really used.** The five manipulation ids are aspirational, and the
primitives behind them are gated on the build team's final mechanism decisions — which are not made.
**So the vocabulary will be small at first and grow.** Design the picker for a manifest that grows,
and do not hardcode a list — including the built-ins, which are append-only but not fixed in number.

#### 4.3.1 The `NONE` question — ANSWERED: `NONE` is not an id

The 18 `NONE` rows were *segment boundary markers* delimiting 17 move segments. **In this vocabulary
a waypoint IS a segment**, so the boundary has no content left to carry, and shulib will never mint
a command id called `none` (its absence is pinned by the freeze test, so a future built-in cannot
quietly give the legacy spelling a meaning it was ruled out of having).

**What this means for each side:** *(the `.shupaths` importer this paragraph assumed was dropped on
2026-09-26; the rule stands for any importer:)* an importer folds `NONE` rows into waypoint
boundaries and emits no marker for them. Your picker never renders one, and never has to explain to
a student what "NONE" would do. If a `.vexbot` ever arrives carrying `id: "none"`, it is an importer
bug, and it will surface as one warning per occurrence rather than as a mystery.

#### 4.3.2 What shulib has actually executed, stated honestly

A `Route` has been executed **on the host physics plant only** — every verb, both drivetrains, with
the runner's output required to be bit-identical to the same legs written by hand in C++. The library
has never driven a physical robot under its own control. Read §7 before betting a competition on any of this.

---

## 5. Seam 3 — `SHUL/2` over the agent socket (F9)

### 5.1 Discovery and transport — the part that exists

VexBuilder's Tauri agent already writes:

```
~/.local/share/com.gonzei.vexbuilder/agent/server.json   →  { port, token, pid }
```

shulib discovers the socket by reading that file, connects to `port`, and authenticates with `token`.
**This is the one piece of the sim seam that already works**, and it is what makes the rest a plug-in
rather than a redesign.

### 5.2 The payload — the `SHUL/2` v1.0 format (built 2026-09-29, host-only, NOT frozen)

`SHUL/2` is the **versioned, sequenced, checked wire serialisation of `DebugRecord`**, the diagnostic
record defined in shulib's very first chunk. Defining it once and serialising it in four places is why
that chunk came first: **one schema, four sinks** — terminal, SD blackbox, `SHUL/2`, and the tuner — so
bench, field and sim traces are directly comparable.

**What exists, stated exactly.** Since 2026-09-29 shulib has the format (`diag/shul2_format.hpp`: the
encoders, the decoders, the version negotiation and `scanFrame()`, the one function every reader is
built on) and a reader over a whole capture (`diag/shul2_reader.hpp`), proven by the host test suite.
**What does not exist yet:** the writer (the sink a program attaches — the next part of the same chunk),
any program that writes the wire, and the socket end (H2). **These bytes have never crossed a real
link** — no USB port, no socket. **Nothing here is frozen:** register row F9 freezes this wire at the
chunk's last part, and until then any layout below may still move. After it, every frame type grows
only by appending (§5.2.7).

#### 5.2.1 The rules every field follows

- **Binary, not JSON.** Earlier shulib documents called `SHUL/2` "the JSON wire"; it is binary. JSON
  cannot carry NaN or ±Inf at all, a double survives decimal text only with 17 significant digits and a
  correct parser on both ends, and formatting text every tick costs heap and time on the brain.
- **Little-endian, fixed-width, byte by byte.** Every floating-point field is IEEE-754 **binary64**,
  with no narrowing anywhere: a decoded record equals the encoded one field for field, NaN, ±Inf, −0.0
  and denormals included (bit-exact, by test).
- **Canonical units:** inches, radians, seconds, volts, amperes, in/s, rad/s. Headings (the three
  `math::Angle` fields) are wrapped to (−π, π]; a non-finite heading cannot be represented and decodes
  as 0 with a "corrupt" flag raised. Heading errors, residuals and corrections (`errorHeading`,
  `gateResidualHeading`, `correctionDTheta`, and the Summary's `headingMax` / `headingFinal`) are plain
  radians that the decoder neither wraps nor validates: it carries what the writer computed.
  `errorHeading` is always written as the shortest signed error, in (−π, π] — do not unwrap or
  accumulate it.
- **The time base.** Every time on the wire — the Tick's `t`, the Log's `t`, the End's `endTime` — is
  the shulib program's clock reading in seconds, RAW. The Hello's `epochSeconds` is the same clock when
  the session opened; subtract it for session-relative time.
- **A value shulib does not name decodes as its number and is never refused** — a fault code, gate
  reason, motion state or level code from a newer shulib arrives as a number you can print.

#### 5.2.2 The envelope — 14 bytes per frame, the same in EVERY major version

| Offset | Field | Notes |
|---|---|---|
| 0 | `u8` sync0 = `0xFE` | `0xFE` never occurs in valid UTF-8 text |
| 1 | `u8` sync1 = `0x53` | `'S'` |
| 2 | `u8` type | §5.2.3; 0 is never assigned |
| 3 | `u8` flags | 0 in major 1; **ignore unknown bits** |
| 4 | `u32` sequence | per writer, per session (§5.2.6) |
| 8 | `u16` payloadBytes | at most **4096** |
| 10 | payload | `payloadBytes` bytes |
| 10+n | `u32` CRC-32 | over bytes [2, 10+n): type, flags, sequence, length and payload |

**The CRC** is CRC-32/IEEE 802.3 — reflected polynomial `0xEDB88320`, initial value all-ones, final
inversion: exactly `zlib.crc32` (`python3 -c "import zlib; print(hex(zlib.crc32(b'123456789')))"`
prints `0xcbf43926`). **Verify it before you believe a single byte inside the frame.**

**Resync.** After a failed candidate — no sync pair, a length above 4096, or a CRC that does not verify
— resume scanning at the **next byte**, never at the claimed end: a corrupted length would otherwise
throw away every good frame it claimed. A **live** reader must buffer at least 14 + 4096 = 4110 bytes;
a corrupted length stalls it until that many bytes have arrived (about nine tick frames, ~90 ms at
100 Hz), and then the frames inside those bytes are recovered. A reader of a **whole capture** that
meets a candidate running past the end of the capture treats it the same way (resume one byte on): it is
truncation only if no later frame verifies. **When a live stream ends** — the socket closes, the file you
were following ends — what you still hold IS a whole capture: apply that same tail rule to it. Otherwise
a corrupted length in the last 4 KB of a session holds back every frame after it, the End included, for
good, and that session's link loss can never be known. (Applying it after an idle timeout as well is
your policy to choose: it releases a held End sooner, at the cost of dropping a frame whose bytes were
merely late.)

**What reading costs.** Linear in the capture for ordinary damage and for random bytes (a random sync
pair turns up about once in 65,536 positions). The worst case is bytes crafted to pack in as many
candidates claiming an accepted length as a byte pattern can — about three per ten bytes, since a
candidate's own `0x53` and its length's high byte (at most `0x10`) sit where no other candidate can
start — each costing up to 4104 bytes of CRC before it fails: about 1,200 bytes of CRC per input byte,
or roughly 14–23 s per MB of such input with a bitwise CRC on a desktop machine (measured 2026-09-30;
the spread is the machine's other load). A reader on a UI thread should bound its own time; a
table-driven CRC-32 gives the same values faster (shulib's is bitwise, to save the brain's flash).

#### 5.2.3 Frame types, directions, and one writer per capture

| Type | Name | Payload (v1.0) | Direction |
|---|---|---|---|
| 1 | Hello | 256 bytes (§5.2.4) | either — opens a session |
| 2 | Tick | 428 bytes — one `DebugRecord` (§5.2.5) | shulib → peer |
| 3 | Log | 14 bytes + text (§5.2.5) | shulib → peer |
| 4 | Summary | 168 bytes — one run summary (§5.2.5) | shulib → peer |
| 5 | End | 32 bytes (§5.2.5) | either — closes a session |

Hello and End are **session** frames and travel in either direction (a peer's Hello carries role 1).
The ranges govern **data** frames: **2–63** shulib → peer, **64–127** peer → shulib (your simulated
sensor frames, at H2), **128–255** reserved. A verified frame of a type you do not know — 0 included —
is **skipped by its length and counted**, never guessed at.

**One capture carries ONE writer's bytes.** If you record both directions of the socket, keep two
captures: two writers interleaved into one byte stream break the sequence rules.

**What v1 does NOT carry:** per-motion results and path markers as data (both reach you as **Log
text** — §5.3), inbound frames (H2's, in the peer range), or a separate fault-triage frame (a stream
carries every tick, the one that faulted included).

#### 5.2.4 The Hello (v1.0: 256 bytes)

| Offset | Field |
|---|---|
| 0 | `u8[8]` signature `89 53 48 55 4C 00 0D 0A` (`0x89` "SHUL" NUL CR LF) |
| 8 | `u16` versionMajor = 1 |
| 10 | `u16` versionMinor = 0 |
| 12 | `u8` role (0 = the runtime that emits records, 1 = a peer; other values reserved) |
| 13 | `u8` reserved (0) |
| 14 | `u16` flags (0) |
| 16 | `u16` tickPayloadBytes = 428 |
| 18 | `u16` summaryPayloadBytes = 168 |
| 20 | `u16` endPayloadBytes = 32 |
| 22 | `u16` logFixedBytes = 14 |
| 24 | `f64` epochSeconds (the clock at session open, seconds) |
| 32 | `u8` apiMajor (shulib's API version) |
| 33 | `u8` apiMinor |
| 34 | `u16` reserved (0) |
| 36 | `u8[48]` buildHash (text) — **empty means MISSING: show it loudly** |
| 84 | `u8[32]` routineId |
| 116 | `u8[16]` alliance |
| 132 | `u8[16]` side |
| 148 | `u8[96]` portMap |
| 244 | `u8[12]` reserved (zero) |

Text fields are NUL-padded to their width; a field ends at its first NUL (or fills its width). Reserved
bytes and flag bits are written 0 and **never judged** — a later minor may give them only a meaning that
is safe to ignore.

**The signature** exists so that a transport which strips bit 7, truncates at a NUL, or translates
newlines corrupts **every** Hello — so no session ever starts on such a link. Because the CRC catches
that corruption first, a reader reports those links as "no frame verified" (bit 7, NUL) or "frames but
no Hello" (newline translation), not as a bad signature. A bad signature means a Hello whose CRC
verified and whose signature is wrong: a writer bug, or something that is not `SHUL/2` at all.

#### 5.2.5 The payloads

**The Tick** is exactly the SD blackbox's tick record — the same 428 bytes, by construction (one
encoder writes both), so a tick read off a robot's SD card can be re-wrapped as a `SHUL/2` frame without
decoding it.

| Off | Field | Off | Field | Off | Field |
|---|---|---|---|---|---|
| 0 | `f64` t (s) | 112 | `u8` wheelCount (0–8) | 260 | `f64` imuYaw (rad, wrapped) |
| 8 | `f64` dt (s) | 113 | `u8` activeCommandState (MotionState) | 268 | `f64` imuYawRate (rad/s) |
| 16 | `f64` targetPose.x (in) | 114 | `u8` deadReckoning (0/1) | 276 | `f64` quality (0–1) |
| 24 | `f64` targetPose.y (in) | 115 | `u8` qualityClass | 284 | `f64` covarianceTrace |
| 32 | `f64` targetPose.heading (rad, wrapped) | 116 | `u32` activeCommandId (0 = none) | 292 | `f64` gateResidualX (in) |
| 40 | `f64` measuredPose.x (in) | 120 | `u16` fault (FaultCode) | 300 | `f64` gateResidualY (in) |
| 48 | `f64` measuredPose.y (in) | 122 | `u8` gateReason (GateReason) | 308 | `f64` gateResidualHeading (rad) |
| 56 | `f64` measuredPose.heading (rad, wrapped) | 123 | `u8` flags (below) | 316 | `f64` gateMahalanobis |
| 64 | `f64` errorX (in) | 124 | `u32` droppedRecords | 324 | `f64` correctionDx (in) |
| 72 | `f64` errorY (in) | 128 | `u32` droppedLines | 332 | `f64` correctionDy (in) |
| 80 | `f64` errorHeading (rad) | 132 | `f64[8]` wheelVoltage (V) | 340 | `f64` correctionDTheta (rad) |
| 88 | `f64` commanded.vx (in/s) | 196 | `f64[8]` wheelCurrent (A) | 348 | `f64` batteryVoltage (V) |
| 96 | `f64` commanded.vy (in/s) | | | 356 | `f64` batteryCurrent (A) |
| 104 | `f64` commanded.omega (rad/s) | | | 364 | `f64[8]` tickPhase (s, by TickPhase) |

Only the first `wheelCount` entries of the wheel arrays are meaningful; a `wheelCount` above 8 is
corruption (a reader clamps it to 8 and flags the record). `activeCommandId` 0 means no active command.

**The tick flags byte (offset 123):** bit 0 `clampedThisTick` (the per-tick correction budget bound the
nudge), bit 1 `strafeFallbackActive` (an H-drive's turn-then-drive fallback), bit 2 `tankSteerActive` (a
tank's `moveTo` steering back onto its line, added 2026-09-27); bits 3–7 are written 0 and ignored.

**The Summary** is exactly the SD blackbox's summary frame — one run summary, 168 bytes:

| Off | Field | Off | Field |
|---|---|---|---|
| 0 | `i32` motionsStarted | 44 | `f64` worstLoopDt (s) |
| 4 | `i32` motionsSettled | 52 | `f64` firstFaultTime (s) |
| 8 | `i32` motionsTimedOut | 60 | `u32` droppedRecords |
| 12 | `i32` motionsCancelled | 64 | `u32` droppedLines |
| 16 | `i32` motionsAborted | 68 | `u32` blackboxDropped (the summary's own figure) |
| 20 | `i32` gatingRejects | 72 | `f64` batteryStart (V) |
| 24 | `u8` hasHeadingData (0/1) | 80 | `f64` batteryEnd (V) |
| 25 | `u8` brownout (0/1) | 88 | `u8[48]` buildHash (text) |
| 26 | `u16` firstFault (FaultCode) | 136 | `u8[32]` routineId (text) |
| 28 | `f64` headingMax (rad) | 168 | end |
| 36 | `f64` headingFinal (rad) | | |

**The Log** — one leveled, tagged line, 14 fixed bytes and then the text:

| Offset | Field |
|---|---|
| 0 | `u8` level (wire code, below) |
| 1 | `u8` flags (bit 0: tag truncated, bit 1: message truncated; bits 2–7 ignored) |
| 2 | `u8` tagBytes |
| 3 | `u8` reserved |
| 4 | `u16` messageBytes |
| 6 | `f64` t (s, the clock when the line was logged) |
| 14 | the tag, then the message — **verbatim**, never sanitized, never NUL-terminated |

The text always starts at offset 14; fields a later minor adds go **after** the text. `logFixedBytes`
(in the Hello) means every Log byte that is not text — `payloadBytes − tagBytes − messageBytes` — 14 in
v1.0. Sanitizing the text is a display concern: do it where you render, never on the wire.

**The End** — the writer's own account of the session:

| Offset | Field |
|---|---|
| 0 | `u32` framesOffered (the End's own sequence number) |
| 4 | `u32` framesFailed (frames the writer's own device refused — they appear as gaps) |
| 8 | `u32` recordsSeen |
| 12 | `u32` linesSeen |
| 16 | `u32` linesTruncated |
| 20 | `u32` notOpenDropped (items handed over while no session was open) |
| 24 | `f64` endTime (s) |

#### 5.2.6 The value tables

| `FaultCode` (u16) | | `GateReason` (u8) | |
|---|---|---|---|
| 0 | None | 0 | None |
| 1 | Precondition | 1 | Accepted |
| 2 | NanPose | 2 | RejectedInnovation |
| 3 | LoopOverrun | 3 | RejectedMahalanobis |
| 4 | OdoStuck | 4 | RejectedNoFix |
| 5 | ImuLost | 5 | RejectedHighYawRate |
| 6 | GpsGateReject | 6 | RejectedNormalizedInnovation |
| 7 | Brownout | 7 | RejectedStaleFix |
| 8 | MotionTimeout | 8 | RejectedSensorQuality |
| 9 | MotorOverTemp | 9 | RejectedNoTagMapEntry |
| 10 | Implausible | 10 | RejectedTagRange |
| 11 | MechanismStalled | 11 | RejectedObservationAge |
| 12 | MotorGroupDisagree | 12 | CovarianceReinit |

| `MotionState` (`activeCommandState`) | | `qualityClass` | | `TickPhase` (index into `tickPhase`) | |
|---|---|---|---|---|---|
| 0 | Idle | 0 | Uninitialized | 0 | Localization |
| 1 | WaitingForEstimate | 1 | DeadReckon | 1 | Motion |
| 2 | Running | 2 | Corrected | 2 | Health (reserved) |
| 3 | Settled | 3 | Degraded | 3 | Telemetry (reserved) |
| 4 | TimedOut | | | 4 | Scheduler (reserved) |
| 5 | Cancelled | | | 5 | User (reserved); 6–7 spare |

**Log level codes:** Error 0, Warn 1, Info 2, Debug 3, Trace 4.

Every table here is append-only: a value is never renumbered or re-meant, and a new one arrives as a
new number.

#### 5.2.7 Sessions, sequence numbers and versions

- **A session** opens with a Hello at sequence 0 and closes with an End. The writer re-sends its Hello
  periodically (every N frames — a writer policy, not part of the format), so a reader that joins late
  can start at the next Hello; such a **late-joined** session is read normally. A Hello at sequence > 0
  inside a session must repeat that session's Hello byte for byte, or it starts a new session. A Hello
  at sequence 0 always starts a new session — even inside one that never sent its End, which is what a
  writer that restarted (a reset, a brownout) leaves behind.
- **What a reader delivers.** Read a frame only inside an **accepted** session — after its Hello and
  before its End. A Tick, Summary or End must be exactly the width its session's Hello declared, and a
  Log's non-text bytes (`payloadBytes − tagBytes − messageBytes`) must equal the Hello's
  `logFixedBytes`; a frame that disagrees is skipped and counted, never decoded — decoding it would be
  guessing. Frames outside any session (before the first Hello, or after an End and before the next
  Hello) and every frame of a **refused** session (its re-Hellos and its End included, until a Hello that
  differs) are skipped and counted the same way.
- **Sequence numbers** run per writer, per session, modulo 2³². With `d = seq − expected` (mod 2³²):
  `d = 0` in order; `0 < d < 2³¹` means `d` frames were lost; `d ≥ 2³¹` is a stale or duplicate frame
  to skip. A wrap from `0xFFFFFFFF` to 0 is in order; only a **Hello** at 0 starts a session.
- **Link loss** is `max(0, frames lost − End.framesFailed)`: the frames the link lost, as opposed to
  the ones the writer's device refused. It is knowable only for a session read from its sequence-0 Hello
  whose End arrived.
- **What every major version keeps:** the envelope (sync bytes, offsets and widths, the 4096 bound, the
  CRC and what it covers), the Hello's type (1), and the Hello's payload bytes 0–11 (the signature and
  the version). So a reader of any version can always NAME a stream as another major and refuse it,
  rather than seeing only noise. Everything else may change at a major.
- **Versions, from the reader's side:** another **major** is refused whole — read nothing from it. A
  **newer minor** is accepted: decode the prefix of each payload you know and ignore the appended bytes
  (every frame type grows only by appending). An **older minor** is accepted when its declared widths
  equal that minor's documented widths; fields it lacks take their documented defaults. At the **same
  minor**, a declared width that differs is refused — it means a layout changed without a version bump.
  shulib's own reader implements exactly these rules and is tested against a hand-built v1.7 stream (every
  frame 16 bytes longer: read, extras ignored and counted) and a v2.0 stream (refused whole, with the
  v1.0 session after it in the same capture still read). This is §6's promise, made concrete for the
  wire: unknown newer fields are ignored, never fatal; missing fields get safe defaults.

### 5.3 Direction of travel

Bidirectional, once Rapier lands:

- **VexBuilder → shulib:** simulated sensor readings, fed in so the *same unmodified estimator* runs.
- **shulib → VexBuilder:** pose, twist, wheel commands, markers — so you render the ghost robot and
  overlay planned-vs-actual.

**Markers, in v1 of the wire:** there is no marker frame. A path's markers, like each motion's result,
reach you only as **Log text** — the lines the path runner and the motion layer already print, carried
verbatim in Log frames (§5.2.5). Parse them as text for now; a data frame for either is an additive
frame type later (a new number in the 2–63 range), which a v1 reader skips by its length.

Because shulib's core depends only on a PROS-free hardware seam, **"works in the simulator" is a
structural guarantee rather than a feature** — the identical motion and localization code already runs
against real hardware, a host test double, and (next) your sim. No `#ifdef`, no sim-specific branch.

---

## 6. The versioning contract — the promise that protects your work

**Every contract carries `schemaVersion`, and shulib migrates additively.**

- **Unknown newer fields are ignored, never fatal.** You can ship a field before we consume it.
- **Missing fields get safe defaults.** We can ship a field before you emit it.
- shulib negotiates versions and mirrors your own `migrateProject()` pattern.
- **A `.vexbot` made next year still drops into the library.** That is a promise, and it is testable —
  our acceptance test for the ingestion work is literally "a deliberately-newer `schemaVersion` loads
  without fatality."

**Practical consequence for you: additive changes are free. Ship early, ship incrementally.** The only
expensive changes are renames and semantic re-meanings of existing fields — avoid those, and nothing
you build gets thrown away.

---

## 7. What can still move under you — the honest risk list

You asked not to be blindsided. This is the list.

| Risk | Likelihood | What we are doing about it |
|---|---|---|
| **F7 (`robotProfile`) is not frozen.** Field names and nesting can change | **real** | §3's schema is the current best draft. It freezes when shulib's ingestion ships. **Build against §3 and flag anything awkward — early feedback changes the schema cheaply, late feedback does not** |
| ~~**F8 (`paths[]` + ids) is not frozen**~~ | **RETIRED 2026-09-18** | **F8 is frozen.** §4.1's field names, the `motion`/`headingMode` value sets, the capacities and the four built-in ids are pinned by a compile-time test in shulib. What still grows is the **vocabulary of team-registered ids**, which is additive by design and arrives through the manifest |
| ~~**The command-id manifest does not exist**~~ | **RETIRED 2026-09-18** | It exists, with a byte-exact golden test behind it. Its format and a real example are in §4.2.3 |
| **F9 (`SHUL/2`, §5.2) is published but not frozen.** Its layouts and rules can still move | **real, until H1's last part** | The format and shulib's reader exist (host-only, 2026-09-29); the writer comes next, then the freeze. Build a reader against §5.2 now and flag anything awkward — before the freeze a change is cheap. After it, every frame type grows only by appending (§5.2.7) |
| **`gearRatio` is a new field with no shulib reader behind it yet** | certain | shulib had **no gear-ratio concept at all** when this was written; since R3b (2026-09-14) `hal::DriveGeometry` carries a per-side motor-to-wheel ratio, and the self-calibrator measures the effective scale per side — but nothing reads a `.vexbot` yet (G3). Emit it anyway; it is cheap to carry |
| **Per-side gear ratios may be required** | likely | At least one team robot is believed to be geared differently left vs right. §3.1 puts `gearRatio` per motor for this reason — do not hoist it to a single drivetrain-level field |
| ~~**`NONE` boundary-marker representation is undecided**~~ | **RETIRED 2026-09-18** | Decided: **`NONE` is not an id.** A waypoint IS a segment; the importer folds the boundary rows away and the picker never renders one. §4.3.1 |
| **Rapier does not exist, so seam 3 is unexercised** | certain | H1 defines the wire unilaterally and can be built and round-trip tested with no sim at all. The socket work is independent and can proceed now |
| **shulib's sensor-noise and tuning constants are all provisional** | certain | Irrelevant to the file format; relevant if you ever compare sim output to real output. Do not treat shulib's shipped constants as measured |

---

## 8. The ordered work list

### 8.1 VexBuilder side, in dependency order

1. **Agent socket + `server.json` hardening** — *being taken by the programming chair.* Independent of
   everything else; can proceed immediately. Confirm the socket is reachable, the token is honoured,
   and a stale `pid` is detected rather than trusted.
2. **The electrical UI.** ⚠️ **This is the true critical path on your side.** `electrical{}` is empty
   today, and it is the source data for `robotProfile`. Nothing in seam 1 can be emitted until motors,
   sensors and pneumatics can actually be entered.
3. **Explicit drivetrain fields** (`kind`, `trackWidth`, `wheelDiameter`) — small, and it retires the
   brittle inference path.
4. **`project.robotProfile`** per §3.
5. **`project.paths[]`** per §4.1 — unblocks the whole no-code milestone.
6. **Command picker** reading shulib's manifest — **no longer blocked: G2 shipped the manifest.**
   Build against the real format in §4.2.3; the four built-ins are always present, so the picker has
   something to render on a robot whose team has registered nothing yet.
7. **Rapier sim + `SHUL/2` feed** — last, and the only one gated on physics.

### 8.2 shulib side, in dependency order

1. **G1** — the config/route types and builder, against hand-written fixtures. **Ungated, can start
   today, zero VexBuilder dependency.**
2. ~~**G2** — path runner, command registry, **manifest export**~~ **— DONE 2026-09-18 (host-only).**
   Your picker is unblocked; the format is §4.2.3.
3. **G3** — `.vexbot` ingestion + codegen ← *this is what makes your file actually run a robot.*
4. **G4** — legacy importer + the 10-minute guide.
5. **H1** — `SHUL/2` ← *ungated; can be built and tested before Rapier exists.*
6. **H2** — the sim adapter ← the only one that needs Rapier.

### 8.3 The two things that unblock each other soonest

- ~~**shulib G2's manifest** unblocks **VexBuilder's picker.**~~ **Done — the picker is unblocked**
  (§4.2.3 is the format; §4.1 is the schema it writes into).
- **VexBuilder's electrical UI + drivetrain fields** unblock **shulib G3's ingestion.** ← **now the
  single critical path between the two projects.** Everything on shulib's side that can proceed
  without it has proceeded.

---

## 9. Ground rules, restated so neither side has to remember them

1. **shulib defines the contracts; VexBuilder implements them.** Where this document is unclear or
   wrong, that is shulib's bug to fix, and saying so early is cheaper than working around it.
2. **shulib ignores everything outside the three seams.** Parts, holes, joints, render data — change
   them freely.
3. **No authoring and no simulation inside shulib.** No file parsing in the robot's hot path.
4. **shulib must stay standalone-usable.** The file is the on-ramp, never a dependency. Any design that
   makes `.vexbot` mandatory is wrong by construction.
5. **Data, never code.** No C++ snippets in project files, ever. Ids and typed args only.
6. **Additive-only evolution.** New fields are free; renames and re-meanings are not.
