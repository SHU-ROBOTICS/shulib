# R3d Part 2 — persistence, the registry, boot verification, and the programs

> **Execution brief.** Written 2026-09-18 ~05:45 by the coordinator during the autonomous session
> ([session log](SESSION-2026-09-18-autonomous.md)), against Part 1's API as it stood in the
> working tree that morning; the coordinator re-checks the API names against the COMMITTED Part 1
> before this launches and amends §1 if they moved. Parent spec:
> [`R3d-drivetrain-self-calibration.md`](R3d-drivetrain-self-calibration.md) §10 (Part 2), which
> this file expands; its §3 rulings T1, T9 and T10 bind here. Live log: append to
> [`R3d-PROGRESS.md`](R3d-PROGRESS.md) under a **"Part 2"** heading, created FIRST.
> **Executor model: `opus`. Do not commit.**

---

## 0. Why this part, and what "done" buys in the morning

Part 1 built a calibrator that has never left the host. Part 2 gives it a program to run in on the
brain, a place to keep its answer, and a boot that checks the answer against the ports before
trusting it. The morning target (team lead, 2026-09-18): his PC on robot two's REBUILT drivetrain,
and **the library's motion stack driving it for the first time.** The rebuild made the chassis
table's ports and signs unverified; Part 2 is what makes that not matter — the robot describes
itself, the record goes to the card, and "shulib Teleop" boots from the record.

Done = six builds green in the src build gate (`tank-calibrate` added), "shulib Calibrate" and
"shulib Teleop" uploadable, both UNRUN on hardware and saying so, and the worksheet's Station G
written for the team lead to follow alone in the morning.

---

## 1. What exists to build on (read these; Part 1's API is the input)

| Piece | Where | What matters here |
|---|---|---|
| The description | `include/shulib/hal/drivetrain_description.hpp` (Part 1) | `DrivetrainDescription{kind, cartridge, imuPort, left, right, trackWidth, source, provenance[96]}`, `DriveSideDescription{signedPorts[21], count, inchesPerRadian}`, `valid(why, cap)`, `describe(buf, cap)`, `leftGeometry()/rightGeometry()`, enums `DrivetrainKind{Tank=1}`, `MotorCartridge{Red=1,Green=2,Blue=3}`, `DescriptionSource{TypedTable=1,RobotMeasured=2}`; `kMaxSmartPorts = 21`. Fixed width, no pointers — designed for byte-by-byte serialization (spec T10) |
| The calibrator | `include/shulib/calibration/drivetrain_calibrator.hpp` (Part 1) | `DrivetrainCalibrator(candidates, ports, imu, clock, config)` — candidates built with POSITIVE ports; `start(HumanFacts{kind, cartridge, imuPort, provenance})`, `tick()`, `answer(Answer)`, `abort()`, `phase()`, `status()` (`{phase, prompt, progress[96]}`), `reason()`, `result()` (valid in `Done`), `report()` (per-port rows for a panel). `Answer{Done, FrontFirst, BackFirst, Distance24, Distance48, Distance72, Abort}` |
| The table's second source | `src/chassis_table.hpp` `tableToDescription()` + `describeMissingForDescription(t, buf, n)` (Part 1) | robot two's typed table as a description with `source = TypedTable`; INVALID (not merely incomplete) while geometry is UNSET, the cartridge is Unset, the IMU port is 0, or the table is UNSIGNED (both sides empty); the helper names every missing field for the refusal screen |
| The write half of persistence | `hal/block_sink.hpp`, `hal/pros/block_sink.hpp`, `hal/fake/fake_block_sink.hpp`, `test/pros_block_sink_adapter_test.cpp` | Bytes verbatim, bool-valued, noexcept; the adapter OWNS the `/usd/` prefix, takes a BARE name, an injectable `mountRoot` so the host test runs the REAL fopen path against a temp dir; no card ⇒ a refusing sink, never a throw (T5's ruling). **Mirror this exactly for the read half** |
| The record precedent | `diag/blackbox_format.hpp`, `test/blackbox_format_test.cpp` | magic + version + fixed width + little-endian, byte by byte, never a struct memcpy; refuse-don't-misread; a BYTE-EXACT golden, not a round trip (E1's mutation 1b: encoder and decoder moved together stayed green) |
| The registry | `src/bench_r3a.cpp:114-121, :573-620`; `include/pros/apix.h:660` | `pros::c::registry_get_plugged_type(uint8_t idx)` is ZERO-indexed: index i is PORT i+1; returns `v5_device_e_t` (`E_DEVICE_MOTOR`, `E_DEVICE_IMU`, `E_DEVICE_RADIO`, `E_DEVICE_NONE`, …). Not in the host shim (`test/pros_shim/pros/`) — add it, with a shim control to set per-port types |
| The group | `hal/motor_group.hpp` | "Every member is PRESENT by contract … no dead-port tolerance (that is shulib Drive's job until R3d)" — THIS is where that promise is kept; `medianInPlace` is shared with the calibrator (Part 1) |
| The dead-port policy | `teleop/drivetrain_degradation.hpp` | `evaluateDegradation(left, right) → {Drive, DriveDegraded, Refuse}`, ≥ 3 answering per side and ≤ 2 dead in total. Reused by the boot check — one policy, two programs |
| The library program | `src/main.cpp:442-577` (`tanklib::TankRobot`), `:825-847` (the refusal path) | Builds ten `ProsMotor`s from the TABLE's signed entries, one `DriveGeometry` from the table, `TankKinematics{tableTrackWidth}`; refuses at boot on an inconsistent/unsigned/incomplete table; `paintRefusal()` |
| The drive program (the pattern for a tolerant boot and a self-contained panel) | `src/drive_program.cpp:140-200, :320-380` | `Member{port, side, signedPort, optional<ProsMotor>, absent, absentWhy}`; one `try` per adapter; `AllStopGuard`; `paintRefusal(headline, why, dead)`; the LCD/screen painting with `pros::c::screen_*` |
| The build axis | `Makefile:78-115`, `tools/src_build_gate.py:159-176, :362-386` | `PROGRAM` values with `$(error)` on anything else; the gate's `BUILDS`/`DEFINES`/`VARIANT_BEACONS` tables and the beacon-in-ELF check |
| The worksheet | `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` Station F (`:474-594`) | The shape Station G copies: what the screen shows, the first-run rule, YES/NO boxes, "what to send back" |

---

## 2. Scope

### IN

1. **`hal/block_source.hpp` — `IBlockSource`**, the READ sibling of `IBlockSink`: `[[nodiscard]]
   virtual std::size_t readAll(std::span<std::byte> into) noexcept = 0;` — bytes copied into the
   caller's buffer, up to its size; returns the count (0 = nothing: no device, no file, or an empty
   file — `isOpen()` tells them apart); never throws. `hal/fake/fake_block_source.hpp` (a byte
   vector, a `setShortRead(n)` so truncation is testable). `hal/pros/block_source.hpp` —
   `ProsBlockSource(fileName, mountRoot = "/usd/")`, fopen `"rb"`, same bare-name/mount-root/no-card
   rules as the sink, `isOpen()`, `path()`, non-copyable (owns the `FILE*`). Host-tested through the
   shim + a real temp directory, like the sink's test: write with `ProsBlockSink`, read back with
   `ProsBlockSource`, bytes identical; no card ⇒ refusing source.
2. **`hal/device_registry.hpp` — `IDeviceRegistry`**: `enum class DeviceKind : std::uint8_t { None,
   Motor, Imu, Rotation, Distance, Optical, Vision, Gps, Radio, Adi, Other }` and
   `[[nodiscard]] virtual DeviceKind pluggedKind(int port) const noexcept = 0;` for `port` in
   1..21 (out of range ⇒ `None`). `hal/fake/fake_device_registry.hpp` (`setKind(port, kind)`).
   `hal/pros/device_registry.hpp` — `ProsDeviceRegistry` over `registry_get_plugged_type`, the
   `port − 1` conversion in EXACTLY one line with the R1a-shape trap cited (`bench_r3a.cpp:114-121`);
   the shim gains `pros/apix.h` (`v5_device_e_t`, `registry_get_plugged_type`) and a
   `shim::registryState()` control. The tester's own census (`bench_r3a.cpp`) is NOT rewritten
   over the seam in this part (it is a measuring tool with a hardware history; leave it).
3. **`calibration/calibration_record.hpp`** — the on-card format, v1: magic `"SHDC"`, `u16
   version = 1`, `u16 payloadBytes`, the description's fields fixed-width little-endian in
   DECLARATION order — kind u8, cartridge u8, imuPort u8, left (21 × i8 + u8 count + f64 ipr),
   right (the same), f64 trackWidth, source u8, 96 bytes provenance — then a `u32` **CRC-32** (IEEE
   802.3 reflected polynomial `0xEDB88320`, computed bitwise with no table — allocation-free,
   ARM-safe) over everything before it. `kRecordBytes` is a constant the encoder asserts.
   `encodeRecord(desc, std::span<std::byte>) → std::size_t` and `decodeRecord(std::span<const
   std::byte>, DrivetrainDescription&, char* why, cap) → bool`, refusing — naming which — on
   magic, version, length, CRC, and then on `desc.valid()`. Byte-exact golden test (spec T10:
   the E1 precedent, and the reason a CRC is right here where it was wrong there).
4. **`calibration/boot_check.hpp`** — pure: `BootVerdict verifyAgainstRegistry(const
   DrivetrainDescription&, const IDeviceRegistry&)` (the degradation verdict is derived inside,
   from `teleop::evaluateDegradation`). Per recorded drive port: `Present` (a motor),
   `Missing` (`None`), `Swapped` (any other kind); the IMU port must be `Imu`; every registry
   `Motor` not in the description is `New`. Verdict: `Refuse` on any `Swapped`, on a missing
   or non-IMU IMU port, or when `teleop::evaluateDegradation` over Present/Missing says refuse;
   `Degraded` when it says drive-degraded (the missing ports listed); `Ok` otherwise, `New` ports
   always listed as a warning. `reason` names every port in every class. **The honest limit, in
   the header and on the screen:** two drive cables swapped WITH EACH OTHER are invisible here
   (both ports still carry motors) and are caught at the FIRST DRIVE by the coupled-side monitor
   as a fight; so is a re-gearing that keeps the ports. A test asserts the limit (two ports
   exchanged ⇒ `Ok`), because a limit nobody has pinned is a limit someone will forget.
5. **`hal::MotorGroup` — the ABSENT member** (the `motor_group.hpp:72-74` promise): a `nullptr`
   in the members span is ABSENT — skipped in `setVoltage`/`setBrakeMode` fan-out, excluded
   from the median/mean/max reads (over PRESENT members), `present = false` to the monitor,
   `presentCount()`, `absentMask()`; precondition: at least ONE present member; `member(i)` on
   an absent index is a precondition violation. **All-present groups are bit-identical** (the
   whole suite is the pin; the log records the count). The plant test gets an absent-member
   case through the existing rig (one pointer nulled).
6. **The programs** (`src/`, PROS-bound, build-gated, UNRUN):
   - **`PROGRAM=calibrate` → "shulib Calibrate" (slot 4)**, `src/calibrate_program.cpp` +
     `.hpp`, `-DSHULIB_PROGRAM_CALIBRATE`, beacon `shulib-robot-variant=tank-calibrate`, the gate's
     sixth build. Boot: `ProsDeviceRegistry` census → every `Motor` port becomes a `ProsMotor`
     with a POSITIVE port and the cartridge from the chassis table's `cartridge` field (the human
     statement; printed as a belief, refuse if `Unset`); the IMU port DISCOVERED from the registry
     (exactly one `Imu`, else refuse naming the count; the table's `imuPort` printed as a
     cross-check, mismatch = a warning, not a refusal); `ProsImu` on it, `calibrate()` once;
     `ProsClock`, `ProsTickPacer` at 10 ms. `HumanFacts{Tank, cartridge, imuPort, provenance =
     "shulib Calibrate " __DATE__ " " __TIME__}`. The loop ticks the calibrator, polls the touch
     screen, paints at 10 Hz: header (program, build stamp, phase), the `status().prompt` LARGE,
     the `progress` line, the per-port table from `report()` (port, sign, side, the two pushes'
     deltas, colour = state), and a bottom row of touch buttons whose labels change with the
     phase (`DONE` / `FRONT FIRST` `BACK FIRST` / `24 in` `48 in` `72 in` / always `ABORT`).
     Touch is release-edge, the resistive-screen rule (a firm press). On `Done`: encode the
     result, write it through `ProsBlockSink("shulib_drivetrain.bin")`, `flush()`, then READ IT
     BACK through `ProsBlockSource`, decode, compare byte-exact with the encoded buffer — only
     then paint `SAVED` with `describe()`'s line and the file path; a write/flush/read-back
     failure paints `NOT SAVED — <why>` in red and says the result is on screen only. On
     `Aborted`: the reason, red, everything at 0 V + Coast (the calibrator did it; the program
     also runs an `AllStopGuard` on every exit path, the drive program's pattern). Everything
     said on screen is also printed to stdout with a `[R3D]` prefix (the team lead reads the user
     port raw). Competition-state screens as the other programs (`SHULIB_STATE_SCREENS`); the
     calibrator runs ONLY in driver control, enabled. No card ⇒ the run still happens and the
     result is shown; `NOT SAVED — no SD card` in red.
   - **"shulib Teleop" builds from the DESCRIPTION** (`src/main.cpp` `tanklib`): at boot, in
     order — (1) `ProsBlockSource("shulib_drivetrain.bin")` → `decodeRecord` → on success,
     `verifyAgainstRegistry` with `ProsDeviceRegistry`: `Ok`/`Degraded` ⇒ USE IT (`Degraded`
     ⇒ the missing ports become absent group members, item 5, and the panel says `DEGRADED:
     dead 18`); `Refuse` ⇒ refuse at boot naming the ports and saying "recalibrate"; a card
     with NO record ⇒ (2) `tableToDescription(kChassis)`: if `valid()` ⇒ USE IT, with a WARN
     line and a panel row `TYPED TABLE — unverified since the rebuild; run shulib Calibrate` and
     the same registry verification; else (3) refuse naming BOTH ("no record on the card; table
     UNSET: track width, external ratio"). `TankRobot` is refactored to take a
     `const DrivetrainDescription&`: motors from `signedPorts` AS STORED (one place for a sign),
     two `DriveGeometry`s from `leftGeometry()/rightGeometry()`, `TankKinematics{trackWidth}`;
     the session header's port-map string and the panel's `table` row now render the
     DESCRIPTION (`source` word first: `MEASURED` / `TYPED`). A pure, host-tested function decides
     the source: `calibration::chooseBootSource(recordDecoded, recordVerdict, tableValid) →
     {UseRecord, UseTable, Refuse, why}` so the decision table is tested, not eyeballed.
   - `Makefile`: `PROGRAM=calibrate` (tank only, `$(error)` otherwise); comment block updated.
     `tools/src_build_gate.py`: the sixth build in all three tables; its self-test still trips on
     its mutations.
7. Tests (§4), the DoD (§5), the documentation contract (§6).

### OUT (with the owner)

- Running any of it on hardware; every hardware claim; the register's settle lines → **Part 3**.
- The guide chapter "Calibrating a robot" → **Part 3** (written from a followed procedure — the
  guide's rule since C8). Station G (the worksheet) IS in scope here: it is the runbook.
- Runtime death of a member inside the library graph (the drive program's
  `MemberAbsenceDetector` promoted into the group) → a later part; here absence is decided at
  boot.
- Rewriting the tester's census over the registry seam → not now (a measuring tool with a
  hardware history).
- R5/R4 routines in the calibrate program → later chunks (the parent spec's re-framing, pending
  the team lead's word).

### REJECTED

- **A runtime JSON/text record.** Fixed-width binary with a CRC, like the blackbox: no parser, no
  allocation, refuse-don't-misread; a text file invites hand edits that bypass the calibration.
- **Reading the record without verifying the ports.** A card carried from another robot, or a
  robot re-plugged, would boot on a stranger's description; the registry check is cheap and
  runs before any adapter is constructed.
- **Refusing on ANY missing port.** The team lead's rule for the season ("1–2 dead ports
  shouldn't stop driving") is already a pure policy; the library program adopts it rather than
  inventing a stricter one.

---

## 3. Rulings

**P1 — The IMU port is DISCOVERED, not typed.** The registry names the port whose device is an
IMU; that is a fact the robot can read, so the human does not state it (the parent spec's list of
human-stated facts is cartridge, kind, front — the IMU port was in the table only because the
table predates the registry seam). The table's `imuPort` stays as a cross-check and a warning.
Rejected: refusing on a mismatch (a re-plugged IMU is not an error; the calibration is taken
against wherever the IMU IS).

**P2 — The record is verified on the SAME boot that reads it, before any adapter exists, and a
`Swapped` port refuses.** A port that lost its motor may be a dead cable (tolerated by policy) — a
port that now carries a DIFFERENT device kind is a rewired robot, and a rewired robot needs a new
calibration, not a best effort.

**P3 — "shulib Teleop" prefers the record over the table, always.** A `RobotMeasured` description
was produced by moving THIS robot; a `TypedTable` one was typed from a tape on a build that no
longer exists. The order is fixed, not configurable. Rejected: a screen chooser (the team lead's
ruling against choosers inside programs, R3b Part 0b).

**P4 — The absent member is decided at BOOT and is a `nullptr` in the span**, not a flag on
`ProsMotor`: the adapter's constructor already throws on a dead port (HA-98's read-back), so the
composition root catches per port, stores `std::optional<ProsMotor>` (the drive program's
`Member` shape) and hands the group a span with holes. The group's contract changes from "every
member present" to "at least one present", and its header says which reads are over present
members. Rejected: a `NullMotor` object (a fake in a competition binary — R3b §6's rule).

**P5 — Read-back before `SAVED`.** The calibrate program does not paint `SAVED` on a successful
`write()`+`flush()`; it re-opens the file, decodes it and compares bytes. `fflush` is the strongest
persist PROS exposes (HA-122's weak half) and a `SAVED` that was not read back is a claim, not a
measurement.

**P6 — One record file name, in one place.** `calibration::kRecordFileName = "shulib_drivetrain.bin"`
in `calibration_record.hpp`; both programs cite it. A second spelling is how Teleop reads a file
Calibrate never wrote.

---

## 4. Tests (with the mutation each must fail under)

| # | Test | Property | Mutation |
|---|---|---|---|
| 1 | Record golden: the exact bytes of an encoded description (a literal array in the test, hand-derived, not captured from the encoder) | the layout is a document (E1's lesson) | move a field in BOTH encoder and decoder (round trip stays green; the golden goes red) |
| 2 | Decode refusals, each the sole cause: bad magic, version 2, short buffer, long buffer, CRC with one bit flipped in each field class, a decoded description that fails `valid()` (e.g. count 0) | refuse, never misread; each names its reason | drop the CRC check; drop the length check |
| 3 | Round trip through `FakeBlockSink` → `FakeBlockSource`; short read ⇒ refusal naming truncation | the seams carry the record | — |
| 4 | `ProsBlockSource` adapter through the shim + temp dir: bare-name precondition, mount-root join, no card ⇒ refusing (`readAll` 0, `isOpen` false), a real file written by `ProsBlockSink` read back byte-identical, a missing file ⇒ `isOpen` false | the real fopen path | drop the `usd::is_installed` probe |
| 5 | `ProsDeviceRegistry` through the shim: port 1 → index 0, port 21 → index 20, port 0 and 22 ⇒ `None` without touching PROS, every `v5_device_e_t` value mapped | the zero-index conversion lives in one line and is right | `port` instead of `port − 1` |
| 6 | Boot check: all present ⇒ `Ok`; one missing ⇒ `Degraded` listing it; three missing ⇒ `Refuse` (policy); a `Swapped` port ⇒ `Refuse` naming it even with everything else perfect; IMU port carrying a motor ⇒ `Refuse`; a `New` motor ⇒ `Ok` + warning listing it; **two drive ports exchanged ⇒ `Ok`** (the pinned limit) | every class named, the limit honest | drop the swapped check; treat Swapped as Missing |
| 7 | `chooseBootSource` decision table: every row (record ok/degraded/refuse/absent × table valid/invalid) | the boot decision is data | swap the preference |
| 8 | `MotorGroup` absent member: fan-out skips the hole (no call on a null); median over 4 present of 5 equals the 4-member median; `presentCount`, `absentMask`; monitor sees `present=false` for it and never flags it; all-null ⇒ precondition; **all-present groups bit-identical through the plant routine of `motor_group_plant_test` test 7** | the promise at `motor_group.hpp:72` kept without moving anything else | count the absent member in the median; report it present to the monitor |
| 9 | The whole pre-existing suite: assertion count unchanged before/after items 5 and the plant rig change (log both) | bit identity | — |
| 10 | src build gate: six builds; the gate's self-test still red on its mutations; the `tank-calibrate` beacon present in its ELF and absent from the other five | the sixth build is real | — |

Every mutation RUN, observed red, message logged, restored.

---

## 5. DoD — Part 2

- [ ] Items 1–6 of §2 landed; every public member documented; ARM gate green with the new headers
- [ ] `python3 tools/src_build_gate.py self-test && check` — SIX builds green
- [ ] Host suite green; PROS-free and layering guards green (the new `hal/pros/` files are the
      only PROS includes; `calibration/` includes no `sim/`)
- [ ] Tests 1–10 green; every §4 mutation observed red and logged
- [ ] "shulib Calibrate" and "shulib Teleop" both say UNRUN on their boot banner and their screens
- [ ] Station G written; the PROGRESS log complete with the prose-review record
- [ ] **No hardware claim anywhere.**

---

## 6. Documentation, in the same commit

Touched: (1) `docs/api/` regenerated; (2) `docs/hardware-assumptions.md` — new Robot-2 entries
(continuing from Part 1's last number): the registry's zero-index belief as a NAMED entry if the
register does not already carry one (grep `registry` first; if HA-120's family covers it, cite,
do not duplicate); fopen `"rb"` under `/usd/` (a belief until Part 3 reads a file on the brain);
the record's CRC choice as a design note under HA-122; (3) `docs/internal/chunks/R3a-BENCH-WORKSHEET.md`
— **Station G — CALIBRATE, then TELEOP FROM THE RECORD**, in Station F's shape, written for the
team lead alone: F.2.1's census/MOTOR WATCH first (the rebuilt drivetrain), then Calibrate
wheels-up (expect the `pulseYawFloor` refusal — that is the program working), then on the floor
step by step with what the screen shows at each prompt, the tape push distances, the spin
warning (3 m clear, a second person), the `SAVED` line to copy down, a SECOND run for
reproduction, then Teleop from the record with Station F's F.2 checks; the upload commands for
all four programs with `rm -rf bin .d` before each `make`; "what to send back"; (4)
`docs/roadmap.md` "You are here" (one paragraph: Part 2 landed, host-only, six builds, unrun);
(5) `docs/internal/build-order.md` — the R3d status line and the FIRST bold `**Next:**` block
(Part 3 is the bench session, the team lead's morning); (6) `README.md` — "what it is not yet":
one sentence that a calibrate program exists and has not run; (7) `PROJECT-BRIEFING.md`
regenerated. NOT touched, with why in the log: the guide (Part 3), the master plan (moves when it
has run), `docs/faq.md`.

Prose review logged per changed source file (`MotorGroup`, "every member present", "dead-port
tolerance", "until R3d", `chassis table`, `tableTrackWidth`, `tableDriveGeometry`, "refuses at
boot", "shulib Teleop", "three programs", "five builds"): grep `docs/` and `README.md`, read the
hits, fix what is stale — the "three programs / five builds / until R3d" sentences are the ones
most likely to rot.

---

## 7. Landmines

1. **`tableToDescription()` is the ONLY path from the table to the graph now.** `TankRobot` must
   not read `kChassis` for anything the description carries; if it does, a record and a table can
   disagree inside one graph. Grep `kChassis` in `main.cpp`'s `tanklib` after the refactor: the
   session header's port map and the cross-check warning are the only legitimate readers.
2. **The sign is stored inside the port and applied by the adapter, once.** `signedPorts[i]` goes
   to `ProsMotor` AS STORED. Nothing in Part 2 negates anything.
3. **`registry_get_plugged_type` is zero-indexed** and the tester's census comment records the
   day this was learned the hard way. One conversion, one line, one test.
4. **The absent member changes a load-bearing class.** Land it LAST among the library items, with
   the plant pin re-run immediately after; if it does not land clean, REPORT it partial: the boot
   check's `Degraded` then maps to `Refuse` in Teleop (say so on the screen: "dead port 18 — this
   program cannot yet tolerate it; shulib Drive can") and the promise stays open, named.
5. **The screen is resistive** (session-2 §9): release-edge touches, firm presses; a button row a
   thumb can hit; never launch from the controller's menu (HA-128) — the screen says so.
6. **No `fflush` is durable** (HA-122): read back before `SAVED`.
7. **`-Wformat-nonliteral` in the src build**: literal format strings everywhere in `src/`
   (Part 1's `valid()` note).
8. **Do not touch Part 1's calibrator logic.** If a defect is found there, log it and fix it
   THERE (rule: a flaw is fixed where it lives), with its own test and mutation, and say so.
9. **No hardware claim.** The morning is the team lead's.

---

## 8. Verification the coordinator runs (RESUMING §3 + the six-build gate)

The standard block (suite, `src_build_gate self-test && check`, the ARM TU, the two CI greps, the
doc gates, the removability grep), then a read of `boot_check.hpp` in full, `MotorGroup`'s
changed reads, `TankRobot`'s constructor, and the calibrate program's `Done` path (write → flush →
read-back → compare) — and the Station G text against the program's actual strings.
