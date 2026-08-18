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
3. **Two of the three seams are not frozen yet** (F7, F8) and one is not written (F9). §7 is the
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
| Command picker fed by shulib's manifest | ❌ | **ask #4** |
| Agent server + `server.json` (`port`, `token`, `pid`) | ✅ | **this exists** — it is the piece the programming chair is taking |
| Rapier physics sim | ❌ | "planned". Everything sim-related is downstream of it |
| `.vbpath` / standalone path planner | 🚫 | **retired by decision.** Paths live in `.vexbot` now — see §4.0 |

### shulib's side

| Piece | Status | Chunk |
|---|---|---|
| `IRobotConfig` / `IRouteSource` / `RobotBuilder` | ❌ | **G1** — *ungated, can start immediately* |
| `PathRunner` + command-id registry + manifest export | ❌ | **G2** — freezes F8 |
| `.vexbot` ingestion + `robot_config.hpp` codegen + SD runtime loader | ❌ | **G3** — freezes F7 |
| `.shupaths` one-way importer + the 10-minute guide | ❌ | **G4** — closes the accessibility milestone |
| `SHUL/2` wire protocol | ❌ | **H1** — freezes F9. *Ungated: the wire is defined unilaterally* |
| `hal/sim` adapter + record/replay | ❌ | **H2** — needs your Rapier sim |
| On-brain live tuner | ❌ | **H3** |
| **`DebugRecord`** — the record SHUL/2 serialises | ✅ | exists, stable, and already has reserved space for this (§5.2) |
| **Blackbox binary format** — a shipped, versioned, round-trip-tested wire format | ✅ | **the working precedent SHUL/2 should follow** (§5.2) |
| The HAL, motion, localization, sequencing, both auton-authoring tiers | ✅ | done and frozen — this is what a loaded `.vexbot` will drive |

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
  mechanisms: [ { name, motors: [ ... ], pneumatics: [ port ] } ],
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
- **`gearRatio` is new to this schema and it is not optional.** shulib currently has **no gear-ratio
  concept anywhere**, which is a known defect; a real team drivetrain has an external reduction
  between the motor cartridge and the wheel, and it may **differ between the left and right sides**.
  Emit it per motor. If a drive is direct, emit `1:1` explicitly rather than omitting the field.
- **`cartridge` is a software declaration, not a measurement.** A V5 motor cannot report its physical
  cartridge; PROS only echoes what you configured. Whatever your UI collects here is trusted
  absolutely, and a wrong value is a silent 3× error in every velocity. Make the UI make it hard to
  get wrong.
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
This deletes the spec's unsolved "re-export when the robot changes" problem outright. A one-way
importer migrates old `.shupaths` files; nothing writes them again.

### 4.1 The schema

```
project.paths: [ {
  schemaVersion: int,
  name: string,
  waypoints: [ {
    x, y,                        // inches, field frame
    heading,                     // degrees
    headingMode: "locked"|"tangent"|"free",
    motion: "move"|"strafe"|"turn"|"arc",
    reverse: bool,
    constraints: { maxLinearSpeed?, maxAngularSpeed?, timeout? },
    markers: [ { id: string, args?: { ... } } ]
  } ]
} ]
```

### 4.2 The command-id rule — the keystone of the whole no-code story

The legacy planner embedded **C++ snippets** in path data (`code_template: "mech.intakeIn();"`).
**That is rejected outright.** It couples data to code and it is the exact thing this design exists to
eliminate.

Instead a marker carries an **id**, and the student registers a handler once:

```text
runner.on("intake_in", []{ intake.in(); });
```

The auton becomes **data a non-coder authors in VexBuilder, executed by a library the coders maintain.**

- **shulib owns the canonical id vocabulary** and exports it as a **manifest** your picker reads
  (ask #4). Ids are not free-form strings the user types.
- **An unknown id logs a warning and is skipped. It never crashes.** Guaranteed by shulib.
- **Markers may carry typed args** (`{id: "lift_to_level", args: {level: 3}}`) so parametric
  primitives do not explode into one id per value.

### 4.3 The state of the vocabulary today — read this before building the picker

The manifest does not exist yet (it ships with G2). What we know from auditing the legacy code:

| Legacy id | Ever actually emitted? | Fate |
|---|---|---|
| `MOVE_WITH_HEADING` | **yes** — 826 CSV rows + 635 generated + 10 hand-written | covered: shulib's core motion verb is a strict superset |
| `NONE` | **yes** — 18 rows, and **absent from both legacy enums** | **an open question, see below** |
| `PICK_UP`, `PLACE`, `SCOOP`, `RELEASE`, `CLASP` | **no — declared in an enum, never emitted** | planned scoring primitives, not yet built |

**Only one motion id was ever really used.** The five manipulation ids are aspirational, and the
primitives behind them are gated on the build team's final mechanism decisions — which are not made.
**So the vocabulary will be small at first and grow.** Design the picker for a manifest that grows,
and do not hardcode a list.

**The `NONE` open question, which is yours as much as ours:** the 18 `NONE` rows are *segment boundary
markers* delimiting 17 move segments. `.vexbot` already has real segment structure, so the boundaries
may not need an id at all. **Decide this together before the picker ships**, or the importer will
produce ids your UI cannot render.

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

### 5.2 The payload — and shulib has already done most of this work

`SHUL/2` is the **versioned, sequenced wire serialisation of `DebugRecord`**, the diagnostic record
defined in shulib's very first chunk. Defining it once and serialising it in four places is why that
chunk came first: **one schema, four sinks** — terminal, SD blackbox, `SHUL/2`, and the tuner — so
bench, field and sim traces are directly comparable.

**You are not starting from nothing.** shulib already ships a **binary blackbox format** that is
versioned, magic-prefixed, self-describing and round-trip tested:

```
 0  u8[4]  magic "SHBB"        12  f64     epochSeconds
 4  u16    formatVersion       20  u32     ringCapacity
 6  u16    headerBytes (256)   24  u32     byteBudget
 8  u16    tickRecordBytes     28  u8[48]  buildHash
10  u16    flags               76  u8[32]  routineId
                              108  u8[16]  alliance / 124 side / 140 portMap
```

**`SHUL/2` should follow this shape**, not invent a second one. And space is already reserved in the
record for exactly this seam: drop counters, and eight tick-phase timing slots of which **two are
deliberately spare** — so adding a new phase later is a vocabulary append, never a wire reshape.

### 5.3 Direction of travel

Bidirectional, once Rapier lands:

- **VexBuilder → shulib:** simulated sensor readings, fed in so the *same unmodified estimator* runs.
- **shulib → VexBuilder:** pose, twist, wheel commands, markers — so you render the ghost robot and
  overlay planned-vs-actual.

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
| **F8 (`paths[]` + ids) is not frozen** | **real** | Same. The *shape* in §4.1 is stable; the **vocabulary** is what will grow |
| **The command-id manifest does not exist** | certain | Ships with shulib's path runner. Until then, treat the §4.3 table as the whole known vocabulary |
| **`gearRatio` is a new field with no shulib support behind it yet** | certain | shulib has **no gear-ratio concept at all** today — a known defect being fixed in the robot phase. Emit it anyway; we need it and it is cheap to carry |
| **Per-side gear ratios may be required** | likely | At least one team robot is believed to be geared differently left vs right. §3.1 puts `gearRatio` per motor for this reason — do not hoist it to a single drivetrain-level field |
| **`NONE` boundary-marker representation is undecided** | certain | §4.3. **Decide jointly before the picker ships** |
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
6. **Command picker** reading shulib's manifest — needs shulib's G2 first; build the UI against a
   hand-written manifest fixture in the meantime.
7. **Rapier sim + `SHUL/2` feed** — last, and the only one gated on physics.

### 8.2 shulib side, in dependency order

1. **G1** — the config/route types and builder, against hand-written fixtures. **Ungated, can start
   today, zero VexBuilder dependency.**
2. **G2** — path runner, command registry, **manifest export** ← *this is what unblocks your picker.*
3. **G3** — `.vexbot` ingestion + codegen ← *this is what makes your file actually run a robot.*
4. **G4** — legacy importer + the 10-minute guide.
5. **H1** — `SHUL/2` ← *ungated; can be built and tested before Rapier exists.*
6. **H2** — the sim adapter ← the only one that needs Rapier.

### 8.3 The two things that unblock each other soonest

- **shulib G2's manifest** unblocks **VexBuilder's picker.**
- **VexBuilder's electrical UI + drivetrain fields** unblock **shulib G3's ingestion.**

Those two can be worked in parallel and neither waits on the other. Do them first.

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
