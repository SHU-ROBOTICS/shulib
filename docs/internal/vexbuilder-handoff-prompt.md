# Handoff prompt — paste this into a chat working on the VexBuilder repo

> Maintained here so it can be re-pasted and kept current as the contracts move. Everything below the
> line is the prompt. It is written to be pasted whole into a session that has the **VexBuilder**
> codebase open and has never seen shulib.

---

You are picking up **VexBuilder**, and your first job is to tell me exactly where it stands and what
has to happen next. Read this whole brief before touching anything, then do the audit in §2 before
writing code.

## 0. What you need to know about the other half

There are **two tools**, and they are separate codebases:

- **VexBuilder** (this repo) — a Tauri / React / Rust + SQLite desktop app. It is the team's
  **single authoring tool**: 3D robot designer, path planner, and a *planned* physics sim (Rapier).
  Everything it makes is saved in one **`.vexbot`** project file.
- **shulib** — the **runtime**, a C++ library that runs on the VEX V5 brain. It ingests a `.vexbot`
  and drives the robot with it. It is a separate repository you do not have access to.

The relationship is fixed and one-directional:

> **shulib DEFINES the contracts. VexBuilder PRODUCES them.**

shulib reads **exactly three things** from VexBuilder. Everything else in `.vexbot` — parts, holes,
joints, render data, anything for the 3D view — is **ignored by contract**, so you can change it
freely without telling anyone.

**The whole point of the integration:** a team member who cannot write C++ designs a robot and draws a
path in VexBuilder, exports one file, and the robot runs it. That is the accessibility promise the
project is partly judged on, and it is the only thing this integration is for.

## 1. The single most important scheduling fact

**shulib cannot open a `.vexbot` file today. At all.**

No parser, no config type, no route type, no path runner, no codegen tool. The entire ingestion layer
is unbuilt on their side. That is not a reason to wait — it is a reason to know that:

1. **You cannot integration-test against shulib.** Your acceptance test is "the emitted file matches
   the schema in §3", not "shulib loaded it".
2. **Build against the CONTRACT below, not against any shulib API.** There isn't one yet.
3. **If this brief is unclear or wrong, that is shulib's bug, not yours.** Say so early — the
   contracts are theirs to define, so the burden of fixing an awkward schema is theirs. Early
   feedback changes a schema cheaply. Late feedback does not.

## 2. YOUR FIRST TASK — audit this repo and report back

**Everything I am about to tell you about VexBuilder's status came from shulib's documentation,
written by the other team, and may be stale or wrong.** Verify it. Where reality differs from this
brief, **the divergence itself is the most valuable thing you can report.**

Answer these from the actual code, citing files and lines:

1. **What schema version does `.vexbot` currently write?** Where is the schema defined, and where is
   it validated (if anywhere)?
2. **`electrical{motors, sensors, pneumatics}` — does it exist in the schema, and is there any UI
   that populates it?** shulib's docs claim the arrays ship **empty because the electrical UI was
   never built.** Is that still true? This is claimed to be the critical path — confirm or refute it.
3. **`migrateProject()`** — what migrations exist, what is the pattern, and does it handle unknown
   newer fields without failing?
4. **Does anything still read or write `.vbpath` or `.shupaths`?** Both are retired by decision
   (§4.0). If code paths remain, list them.
5. **The agent server** — does it run, and does it write
   `~/.local/share/com.gonzei.vexbuilder/agent/server.json` with `{port, token, pid}`? Is the token
   actually checked on connect? What happens with a stale `pid` from a dead process?
6. **Is there any Rapier / physics work started at all**, or is it untouched?
7. **The path planner UI** — what does it currently produce, and in what shape? Does the concept of a
   per-waypoint "marker" or "command" exist in any form yet?
8. **Drivetrain data** — is `kind` / `trackWidth` / `wheelDiameter` stored anywhere explicitly, or
   would it have to be inferred from part geometry today?

**Then give me an in-depth recap:** what exists, what is half-built, what is untouched, and what you
think the real ordering is. Push back on my ordering in §6 if the code tells you something different.

## 3. Seam 1 — `project.robotProfile` (the robot's electrical + geometry description)

shulib needs this block written into `.vexbot`:

```
project.robotProfile {
  schemaVersion: int,
  identity   { name, team, season },
  drivetrain {
    kind: "x" | "h" | "tank",
    wheelDiameter: inches,
    trackWidth: inches,
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

Field rules that will save you a rewrite:

- **Units are inches and degrees at this boundary.** shulib works in inches/radians/seconds
  internally and converts once at the edge. Emit degrees.
- **Ports are 1-indexed (1–21).** Emit `port` and `reversed` as **separate fields** — never a signed
  port number. shulib translates to its own convention.
- **`gearRatio` is required and is PER MOTOR.** Do not hoist it to a drivetrain-level field: at least
  one team robot is believed to be geared **differently on the left and right sides**. If a drive is
  direct, emit `1:1` explicitly rather than omitting it. *(Context: shulib currently has no
  gear-ratio concept anywhere — a known defect they are fixing. Emit it regardless; they need it.)*
- **`cartridge` is a declaration, not a measurement.** A V5 motor physically cannot report its
  cartridge; the API only echoes what software configured. Whatever your UI collects is trusted
  absolutely, and a wrong value is a silent **3× error** in every velocity the robot computes. Make
  the UI hard to get wrong here.
- **`role` on odometry wheels is required, not inferred.** shulib refuses two wheels of the same
  role — that is the guard that stops a forward and a lateral tracking wheel being swapped.
- **`corrections{}`** is per-robot calibration and may be absent; shulib defaults it to identity.

**Where the data comes from — and the problem.** This block is meant to be derived from
`electrical{}` plus explicit drivetrain fields. If `electrical{}` really is empty because the UI was
never built, **there is no source data and the electrical UI is the prerequisite for this entire
seam.** That makes it the first thing to build. Confirm in the audit.

shulib ships an `inferDrivetrain()` fallback that guesses kind/track/diameter from part geometry. It
is documented as **brittle and explicitly not the contract** — a migration aid for old files, never a
reason to skip explicit fields.

## 4. Seam 2 — `project.paths[]` and the command vocabulary

### 4.0 A decision you need to know about

**`.shupaths` and the standalone Python path planner are RETIRED.** Paths now live inside `.vexbot`
alongside the robot, so a routine can never fall out of version-sync with the robot it was drawn for.
This deletes the old "re-export whenever the robot changes" sync problem outright. A one-way importer
migrates old files; nothing writes them again.

### 4.1 Schema

```
project.paths: [ {
  schemaVersion: int,
  name: string,
  waypoints: [ {
    x, y,                        // inches, field frame
    heading,                     // degrees
    headingMode: "locked" | "tangent" | "free",
    motion: "move" | "strafe" | "turn" | "arc",
    reverse: bool,
    constraints: { maxLinearSpeed?, maxAngularSpeed?, timeout? },
    markers: [ { id: string, args?: { ... } } ]
  } ]
} ]
```

### 4.2 The command-id rule — the keystone

The old planner embedded **C++ snippets** in path data (`code_template: "mech.intakeIn();"`).
**That is rejected outright** — it couples data to code and it is the exact thing this design exists
to eliminate.

Instead a marker carries an **id**, and a student registers a handler once in C++:

```
runner.on("intake_in", ...)
```

So the auton is **data a non-coder authors**, executed by a library the coders maintain.

- **shulib owns the canonical id vocabulary** and exports it as a **manifest your picker reads.**
  Ids are never free text the user types.
- **An unknown id logs a warning and is skipped — it never crashes.** Guaranteed by shulib.
- **Markers may carry typed args** (`{id: "lift_to_level", args: {level: 3}}`) so parametric
  primitives do not explode into one id per value.

### 4.3 The state of the vocabulary — read before building the picker

**The manifest does not exist yet.** From auditing the legacy code, this is the entire known
vocabulary:

| Legacy id | Ever actually emitted? | Fate |
|---|---|---|
| `MOVE_WITH_HEADING` | **yes** — 826 rows + 635 generated + 10 hand-written | covered; shulib's motion verb is a strict superset |
| `NONE` | **yes** — 18 rows, and **absent from both legacy enums** | **open question, see below** |
| `PICK_UP`, `PLACE`, `SCOOP`, `RELEASE`, `CLASP` | **no — declared, never emitted** | planned scoring primitives, not yet built |

**Only ONE motion id was ever really used.** The five manipulation ids are aspirational and the
primitives behind them are gated on the build team's final mechanism decisions, which are not made.
**Build the picker for a manifest that GROWS. Hardcode nothing.**

**Joint decision we owe each other:** the 18 `NONE` rows are *segment boundary markers*. `.vexbot`
already has real segment structure, so they may need no id at all. **Decide this before the picker
ships**, or the importer will emit ids your UI cannot render.

## 5. Seam 3 — `SHUL/2` telemetry over the agent socket

**The programming chair is taking this piece personally** — coordinate, do not duplicate.

- **Discovery:** shulib reads `~/.local/share/com.gonzei.vexbuilder/agent/server.json`
  (`{port, token, pid}`), connects, and authenticates with the token. **This already exists** and is
  the one part of the sim seam that works today.
- **Payload:** `SHUL/2` is the versioned, sequenced wire form of shulib's `DebugRecord`. shulib
  already ships a versioned, magic-prefixed, round-trip-tested **binary blackbox format**; SHUL/2
  should follow that shape rather than inventing a second one.
- **Direction:** bidirectional once Rapier lands — VexBuilder feeds simulated sensor readings **in**
  (so the *same unmodified estimator* runs), shulib streams pose / twist / wheel commands / markers
  **out** (so you render a ghost robot and overlay planned-vs-actual).
- shulib's side of the wire (their chunk "H1") is **ungated** — they can build and round-trip test it
  with no sim in existence. So the socket work and the wire work can proceed in parallel, and neither
  waits on Rapier.

## 6. The work list, in dependency order

1. **Agent socket + `server.json` hardening** — *taken by the programming chair.* Independent of
   everything. Reachability, token actually enforced, stale-`pid` detection.
2. **The electrical UI.** ⚠️ Believed to be the true critical path — nothing in seam 1 can be emitted
   without it. **Confirm this in the audit before committing to it.**
3. **Explicit drivetrain fields** (`kind`, `trackWidth`, `wheelDiameter`) — small, and it retires the
   brittle inference path.
4. **`project.robotProfile`** per §3.
5. **`project.paths[]`** per §4.1 — this is what unblocks the whole no-code milestone.
6. **Command picker** fed by shulib's manifest — build the UI against a hand-written manifest fixture
   until theirs exists.
7. **Rapier sim + `SHUL/2` feed** — last, and the only item genuinely gated on physics.

**What shulib owes you, so you know what to expect and when to chase them:**

| They must ship | It unblocks | Blocked on you? |
|---|---|---|
| Config/route types + builder | everything downstream | no — startable now |
| Path runner + **the id manifest** | **your command picker** | no |
| `.vexbot` ingestion + codegen | a file actually running a robot | **yes** — items 2, 3, 4, 5 |
| `SHUL/2` wire | your socket work | no |
| Sim adapter | the sim seam closing | yes — Rapier |

**The two that unblock each other soonest, and neither waits on the other:** their **manifest** →
your picker; your **electrical UI + drivetrain fields** → their ingestion. Start there.

## 7. The versioning contract — the promise that protects your work

**Every contract carries `schemaVersion`, and shulib migrates additively:**

- **Unknown newer fields are ignored, never fatal.** You can ship a field before they consume it.
- **Missing fields get safe defaults.** They can ship support before you emit it.
- Their acceptance test is literally *"a deliberately-newer `schemaVersion` loads without fatality."*
- **A `.vexbot` made next year still drops into the library.** That is a stated promise.

**So: additive changes are free — ship early and incrementally. The only expensive changes are
renames and semantic re-meanings of existing fields.** Avoid those and nothing you build is wasted.

## 8. What can still move under you — the honest risk list

| Risk | Likelihood | Mitigation |
|---|---|---|
| The `robotProfile` schema is **not frozen** — names and nesting can change | **real** | §3 is the current draft. **Flag anything awkward early**; late feedback is what gets expensive |
| The `paths[]` schema is **not frozen** | **real** | The *shape* in §4.1 is stable; the **vocabulary** is what grows |
| The command-id manifest does not exist | certain | §4.3 is the whole known vocabulary until it ships |
| `gearRatio` has no shulib support behind it yet | certain | Emit it anyway — they need it and it is cheap to carry |
| `NONE` boundary-marker representation undecided | certain | Decide jointly before the picker ships |
| Rapier does not exist, so seam 3 is unexercised | certain | The socket and wire work are independent of it |

## 9. Ground rules

1. **shulib defines the contracts; VexBuilder implements them.** Where this brief is wrong, that is
   shulib's bug — report it rather than working around it.
2. **shulib ignores everything outside the three seams.** Parts, holes, joints, render data — yours.
3. **No authoring or simulation inside shulib. No file parsing in the robot's hot path.**
4. **shulib must stay standalone-usable.** The file is the on-ramp, never a dependency. Any design
   that makes `.vexbot` mandatory is wrong by construction.
5. **Data, never code.** No C++ snippets in project files, ever. Ids and typed args only.
6. **Additive-only evolution.**

## 10. What to hand back

1. **The §2 audit**, with file:line citations and every place reality diverges from this brief.
2. **Your recap:** what exists, what is half-built, what is untouched.
3. **Your ordering**, and where you disagree with §6 and why.
4. **A list of every question this brief does not answer** — those are shulib's to resolve, and the
   cheapest time to ask is before you build against a guess.

**Verify before claiming.** If you are unsure whether something works, say so rather than assuming —
a confident wrong status report is more expensive here than an honest gap, because the other side
schedules against it.
