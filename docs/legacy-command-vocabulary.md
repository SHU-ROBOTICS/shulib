# Legacy command vocabulary — mined at C6, the last look before the C7 deletion of `legacy/`

> **What this is:** the complete record of every command id the team ever declared or authored
> routines with in the legacy tree, what each did (or was meant to do), and where each maps in
> shulib v2 today. This is a **requirements input for F3 (the concrete mechanism/scoring
> primitives — an earlier revision said "F2 (mechanism primitives)", which is two mistakes:
> chunk F2 is the sequence engine, and the mechanism seam was F1) and G2 (the
> canonical command-id registry)**, and the two data specimens characterized here are the real
> inputs **G4's `.shupaths` importer** must be able to read.
>
> **Method:** extracted **mechanically** (scripts in §6), from all four places commands existed
> in `legacy/`; the hand-reading only interprets what the scripts found. Nothing below is from
> memory.
>
> **C7 executed the deletion on 2026-08-10.** Every `legacy/` path in this document is now
> historical: the files live in git history, not the working tree. Retrieve any of them with
> `git show 691c656:<path>` (the last commit containing the tree, `shulib-v2` branch) — e.g.
> `git show 691c656:src/legacy/autonomous_commands.csv` for G4's importer specimen.

---

## 1. The four sources, and the headline finding

<!-- staleness-audit: historical-paths — this document's SUBJECT is the deleted legacy
     tree, so it necessarily names files that no longer exist. See the note below. -->

> **Every path in the table below was deleted at the cutover** and none of them exists in
> the tree today. They are named because this document exists to record what was there
> before it went, and the audit that normally flags a dead path is told so explicitly at
> the top of this section rather than being quietly weakened for everyone.
>
> To read any of them, ask git for the commit before the deletion:
> `git log --diff-filter=D -- '*/legacy/*'` finds it, and
> `git show <commit>^:<path>` prints the file.

| Source | Kind | Rows | Distinct ids used |
|---|---|---|---|
| `include/legacy/shulib/RobotCommands/autonomous_commands.h` | C enum (declaration) | — | 6 declared |
| `include/legacy/shulib/RobotCommands/CommandType.hpp` | C++ enum (duplicate declaration) | — | same 6 |
| `src/legacy/shulib/Pathing/autonomous_commands.c` | generated routine data | 635 | **1** (`CMD_MOVE_WITH_HEADING`) |
| `src/legacy/autonomous_commands.csv` | exported routine data | 844 (+header) | **2** (`MOVE_WITH_HEADING` ×826, `NONE` ×18) |
| `src/legacy/main.cpp:448` | hard-coded `CommandData` vector | 11 | 2 (`MOVE_WITH_HEADING` ×10, `NONE` ×1) |

**The headline finding — no executor ever existed.** Verified by grep: `main.cpp`'s
`autonomousCommands` vector is defined at line 448 and **never read**; the `.c` array is
`extern`-declared in two headers (`autonomous_commands.h`, `CommandArray.hpp`) and **no code
consumes it**; `Chassis::commands` (`chassis.hpp:85`) is declared and **never populated**; the one
OO command (`MoveWithHeadingCommand::execute()`) calls a private `moveTo()` whose body is
**empty**. The team built *three parallel data representations* of a routine — C array, CSV,
C++ vector — plus two enum declarations and an OO command class, and none of it ever drove the
robot. Every auton that actually ran was hand-written blocking calls in `main.cpp` (see §4).

That is the strongest possible evidence for G2's design: this team wanted data-driven autons badly
enough to attempt it three ways, and what was missing every time was the **runner**. G2 builds the
runner first-class; VexBuilder supplies the data.

---

## 2. The declared vocabulary — every distinct id, mapped

Seven distinct ids exist across all sources (6 declared + 1 that appears only in data):

| Legacy id | Authored in data? | What it did / meant | v2 home today | Status |
|---|---|---|---|---|
| `MOVE_WITH_HEADING` | **yes** — 826 CSV + 635 `.c` + 10 `main.cpp` rows | Move to `(x, y)` holding a commanded `heading`, at `speed`. The only motion verb the planner emitted; in practice never executed from data (no runner) | **C1 `MoveToPose`** (decoupled x/y/θ — exactly "move with heading") via **C2 `MotionScheduler`**, surfaced as **C4 `Chassis::moveTo`**; per-segment speed = motion-config speed budget / G2 segment constraints | **Covered** — v2's core motion verb is a strict superset (profiled, watchdog-bounded, exit-reasoned) |
| `NONE` | **yes** — 18 CSV rows + 1 `main.cpp` row, **absent from both enums** | Path-segment boundary marker: every one has `speed=0`; the 18 markers (file start, file end, 16 internal) delimit **17** move segments of 12–115 waypoints | **G2** needs an explicit no-op/waypoint-only marker in the canonical vocabulary (or the importer folds boundaries into `paths[]` segment structure, which `.vexbot` already has) | **GAP (named):** G2 must decide boundary-marker representation; G4 must consume `NONE` rows (see §5) |
| `PICK_UP` | **no** — enum only | Intended: acquire a game piece (floor pickup) | Phase F′ **`intakeUntilCapture`** (master plan §14, bottleneck #1) | Covered by plan, not yet built — F-phase |
| `PLACE` | **no** — enum only | Intended: place/score a held piece | Phase F′ **`buildStack`** / `liftToLevel`+place sequence (§14) | Covered by plan, not yet built — F-phase |
| `SCOOP` | **no** — enum only | Intended: scoop-style floor acquisition (mechanism variant of pick-up) | Phase F′ intake family; whether "scoop" is a distinct verb is a **build-team mechanism decision**, not a software one | Covered by plan; flag: don't mint two ids for one mechanism until the mechanism exists |
| `RELEASE` | **no** — enum only | Intended: release a held piece (their robot had a `releaser` motor group — `main.cpp:62`) | Phase F′ `clampActuate(open)` / outtake | Covered by plan, not yet built |
| `CLASP` | **no** — enum only | Intended: clamp/grab (their robot had pneumatic `arm`/`lever` — `main.cpp:57-58`) | Phase F′ **`clampActuate`+`clampConfirm`** (§14) | Covered by plan, not yet built |

**Reading for G2:** the five mechanism ids were *declared intent that data authoring never
reached* — the planner could only emit `MOVE_WITH_HEADING` (+ `NONE`). So the honest legacy-derived
requirement is not "support these five ids"; it is: **(a)** the motion verb + boundary marker are
the proven minimum a planner emits, and **(b)** mechanism ids must be *registerable by the team*
(`runner.on(...)`) precisely because the mechanism set churns season to season — legacy hard-coding
the season's verbs into an enum is the anti-pattern G2's registry replaces.

### Duplicate-declaration hazard (why porting any of this was rejected)

The 6-id enum is declared **twice** (`autonomous_commands.h` as a C `typedef enum`,
`CommandType.hpp` as a C++ `enum`), the payload struct **twice** (`Command` in the C header,
`CommandStruct` in the C++ one), and `Command.hpp` declares an unrelated abstract `Command` *class*
that **name-collides** with the `extern "C"` `Command` *struct* — an ODR trap wired into the
design. None of it is salvageable as code; the *vocabulary* above is the salvage.

---

## 3. The de-facto vocabulary — what hand-written autons actually used

Because the data path never ran, the *real* command vocabulary is in `main.cpp`'s hand-written
routine (the active one and the large commented-out competition routine, lines 836–1045). This is
equally a G2/F2 requirements input — it is what the team reached for when authoring for real:

| Hand-written verb | What it did | v2 home |
|---|---|---|
| `rotate_to(deg)` | Blocking turn-to-heading, coarse PID, min/max power floors | C1 `TurnTo` (profiled, settled-exit) |
| `move_vertical(inches)` | Blocking straight drive by displacement, PID on remaining distance | C1 `MoveToPose` (relative move); the displacement-not-accumulation lesson is designed in (pose-space error) |
| `move_to_pose(pose)` | Turn-to-face then drive (tank strategy) | C1 `MoveToPose` / C3 H-drive fallback does turn-then-drive *structurally* |
| `chassis.setPose(0,0,θ)` after **every** turn | Manual odometry re-anchor — they trusted heading so little they re-zeroed constantly | Superseded by IMU-owned heading (M2) + M3 correctors; a routine should never need mid-run `setPose` |
| `lever.extend()` / `lever.retract()`, `arm` pneumatics | Mechanism actuation | F′ `deployActuator` / `clampActuate` → G2 ids |
| `intake`/`conveyor`/`releaser` `.move(±127)` + timed `limitedCombo`/`limitedComboFull` | Timed open-loop mechanism combos | F′ `intakeUntilCapture` etc. — sensor-confirmed, not timed; G2 typed-args markers |
| `oscillation(n)` | Wiggle forward/back ×n to seat a piece | **Not in any v2 plan by name** — a candidate F′ micro-primitive (seat/settle wiggle); noted as a gap-adjacent observation for F3/F′ (recorded at C6 as "F2", the pre-split chunk name) |
| `timer(ms)` / `pros::delay` between steps | Sequencing by wall-clock delay | C2 `waitUntilSettled` / D1 recipe chaining / M4 Sequencer |
| `test_min_output()`, `rotation_calibration()` | Calibration routines (see hardware-assumptions addendum) | M3 "calibration routines + persistence" — specimens captured at C6 |

Gap summary for the sequencing/primitives chunks (recorded at C6 under the pre-split name "F2"): everything maps onto planned §14 primitives except the **seat/settle wiggle**
(`oscillation`), which is worth a line in F2's brief as a candidate micro-primitive (cheap, and
they used it twice in one routine, under a dedicated task, mid-drive).

---

## 4. The two data specimens — what G4's importer must actually survive

Both files carry the same 5-column shape — `command, x, y, heading, speed` — and they **disagree
about everything else**. G4 must treat these as two dialects of one format:

| Property | `autonomous_commands.csv` (844 rows) | `Pathing/autonomous_commands.c` (635 rows) |
|---|---|---|
| Container | CSV with header row, string command ids | Generated C array, enum command ids |
| Coordinates | x ∈ [3.69, 123.28], y ∈ [14.27, 118.85] — **fits the 144″ field in inches** | x ∈ [13, 151], y ∈ [23, **343**] — **does NOT fit a 144″ field**; unit/space unrecorded (cm? planner canvas px?) |
| Heading | signed degrees, [−177.03, 172.77] (−180..180 convention) | signed degrees, includes values > 90 and < −90; same convention |
| Speed | constant `50` on every move row; `0` on every `NONE` | **per-waypoint ramp**, 41.8 → 100 within each heading-group (the planner emitted speed *profiles*) |
| Segmentation | 18 `NONE` marker rows (speed 0; file start + file end + 16 internal) delimit 17 move segments, 12–115 waypoints each | segmentation implicit in heading/speed-ramp resets; **no marker rows at all** |
| Waypoint density | ~1″ spacing (dense interpolation, not sparse waypoints) | ~3″ spacing, also dense |
| Vocabulary | `MOVE_WITH_HEADING`, `NONE` | `CMD_MOVE_WITH_HEADING` only |

**Importer requirements extracted (G4):**

1. **Never assume inches.** One shipped specimen is in-field-range inches; the sibling specimen is
   not. The importer needs a unit/coordinate-space decision per file — explicit metadata if the
   planner recorded any, else a documented heuristic (e.g. bounds-fit against the field) that
   **flags** rather than silently guesses. This is the same "explicit fields are the contract"
   posture as `inferDrivetrain()` (§16.1).
2. **`NONE` must parse.** It is absent from the legacy enum (the legacy C importer could not even
   have represented its own sibling file — an enum round-trip bug they shipped). Map it to segment
   boundaries in `paths[]`, not to a motion.
3. **Commands arrive as rows interleaved in the waypoint stream**, not as markers attached to
   segments. The importer must convert row-commands → G2 marker model (id on a waypoint).
4. **Speed is per-waypoint and dialect-dependent** (constant vs ramp). Map to G2 segment
   constraints; do not preserve 1:1 (v2 profiles motion itself — imported speeds are constraints,
   not commands).
5. **Dense ~1″ waypoints are the input shape.** 826 waypoints for one routine. `PathRunner`
   consumes sparse waypoints + profiles; the importer should decimate/fit, and say it did.
6. **Expect unconsumed vocabulary.** The five mechanism ids exist in the enum with zero data
   ever authored; a `code_template`-era file may equally carry ids no registry knows. G4 already
   specifies WARN+skip for unknown ids (§16.3) — these specimens confirm it will fire in practice.
7. The 845-row CSV is committed in-tree until C7. **G4 should lift a copy into its own test
   fixtures before/at C7** (flagged in C6-COMPLETED; the git history retains it regardless).

**For G2's manifest:** nothing in the legacy vocabulary demands a new *canonical* id beyond what
§14/§16.3 already plan. The legacy-derived additions to G2's requirements are exactly two:
an explicit **waypoint-only/boundary marker** semantics decision (`NONE`'s successor), and the
**registry-not-enum** principle §2 documents (which G2 already embodies).

---

## 5. Command-id gaps, stated precisely

- **`NONE` (boundary marker):** no v2 representation *decision* yet. Not a missing feature —
  `.vexbot` `paths[]` segments can absorb it structurally — but G2 must make the call and G4 must
  implement the mapping. **Owner: G2 (decision), G4 (mapping).**
- **Seat/settle wiggle (`oscillation`):** used in real routines, in no plan by name.
  **Owner: F2 brief, as a candidate micro-primitive.** Low stakes, cheap to add or reject.
- Everything else declared or authored in `legacy/` has a named v2 home in the tables above —
  built (C1–C4) or planned with an owner (F′ §14, G2/G4, M3/M4).

---

## 6. Mechanical cross-check (re-runnable against git history)

Every count above came from these commands, run at C6 (2026-08-10), from the repo root — at a
tree that still contained `legacy/`. To re-run them today, check out `691c656` (or pipe
`git show 691c656:<path>` into the same commands):

```sh
# Distinct ids + counts, CSV (844 data rows: 826 MOVE_WITH_HEADING + 18 NONE; all rows 5 fields):
awk -F, 'NR>1 {print $1}' src/legacy/autonomous_commands.csv | sort | uniq -c
awk -F, '{print NF}' src/legacy/autonomous_commands.csv | sort | uniq -c   # → "845 5"

# Distinct ids + counts, generated C array (636 = 1 enum decl + 635 rows; others 1 = decl only):
grep -o 'CMD_[A-Z_]*' src/legacy/shulib/Pathing/autonomous_commands.c | sort | uniq -c

# The two enum declarations (identical 6 ids ×2):
grep -h 'CMD_' include/legacy/shulib/RobotCommands/autonomous_commands.h \
              include/legacy/shulib/RobotCommands/CommandType.hpp | tr -d ' ,' | sort | uniq -c

# main.cpp's hard-coded vector (10 MOVE_WITH_HEADING + 1 NONE among the string literals):
grep -o '"[A-Z_]*"' src/legacy/main.cpp | sort | uniq -c

# NONE rows are all speed-0 boundary markers (18 hits, lines 2..845):
grep -n '^NONE' src/legacy/autonomous_commands.csv

# The no-executor proof (each returns declarations/definitions only, zero consumers):
grep -rn "autonomousCommands" src/legacy/main.cpp            # def line 448 only
grep -rn "autonomous_commands\[" src/legacy include/legacy    # extern decls + the .c def only
grep -n  "commands" include/legacy/shulib/chassis/chassis.hpp # member decl only, never touched
```

Coordinate/speed ranges were computed with awk min/max over the data columns (§4 table); segment
lengths with an awk pass over `NONE` positions. The live log of each extraction as it was run is
preserved in the development log on the `shulib-v2` branch.

---

*Produced by chunk C6 (2026-08-10), the final audit of the pre-rebuild tree before its deletion
at C7 — the full 34-file classification and the safe-to-delete verdict are in the development log
on the `shulib-v2` branch. Companion documents: [`hardware-assumptions.md`](hardware-assumptions.md)
(the legacy-measured reference constants, added at C6), [`shulib-v2-master-plan.md`](shulib-v2-master-plan.md)
§16.3 (the registry design this feeds; G2 owns the id registry, G4 the importer).*
