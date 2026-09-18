# T2 — Input shaping, drive modes, and the feel properties

> **Execution brief**, written 2026-09-18 ~06:10 by the coordinator during the autonomous session
> ([session log](SESSION-2026-09-18-autonomous.md)), tree `76a5819`. Runs **in an isolated
> worktree, in parallel with R3d** — so this chunk touches `include/shulib/teleop/`, its tests and
> the documents named in §6 ONLY; `src/` and the plan documents are integrated by the coordinator
> afterwards (§7). The build-order entry (`build-order.md` "T2 — Input shaping, drive modes, and the
> feel properties") and the roadmap's Phase T rows are the spec; this file carries the design, the
> rulings, the tests and the order. Live log: `docs/internal/chunks/T2-PROGRESS.md`, created FIRST.
> **Executor model: `opus`. Do not commit.**

---

## 0. Why this chunk, why now

Phase T is "the library is the complete stack, driver control included" (build-order Phase T
preamble): a team that needs a second library to drive breaks the one-stop-shop thesis in the
most expensive way — two notions of the drivetrain. T1 (`IController`, the fakes, the PROS
adapter) landed at R1a; the stick mapping was extracted at R3b Session 2 into
`teleop/stick_mapping.hpp` **as the seam T2 owns**, verbatim and deliberately still naive: a
DISCONTINUOUS deadband (output jumps from 0 to 0.05 at the threshold — "the 'twitchy, I can't
place it' complaint", build-order T2), no curve, no slew, body frame only. The team lead's
direction this week: an *extremely user-friendly* library and a main that reads through it. Driver
feel is what a driver meets first, T2 is ungated ("do it when the robot is unavailable" — Stage 3's
fourth row), and its files are disjoint from R3d's — which is why it runs in parallel tonight.

---

## 1. What exists to build on (read these)

| Piece | Where | What matters |
|---|---|---|
| The seam | `include/shulib/teleop/stick_mapping.hpp` | `StickInput{leftY, leftX, rightX, connected}` → `mapSticks()` → `DriveRequest{forward, left, yawCcw}` (dimensionless, signs applied, NEGATE-then-deadband, the sign of zero pinned) → `toChassisSpeeds(request, maxLinear, maxAngular)`; `mapSticksToChassisSpeeds()` is what `src/main.cpp`'s loop calls; `kStickDeadband = 0.05` (HA-112, invented) |
| Its pin | `test/stick_mapping_test.cpp` | bit-identity against a verbatim copy of the R1a code; hand-computed literals; the sign-flip mutations |
| The open-loop consumer | `teleop/coupled_side_monitor.hpp:166-184` `tankSideVolts(request, maxVolts)` | the drive program and the tester turn the SAME `DriveRequest` into per-side volts — shaping must sit BEFORE this fork so both programs feel one robot |
| The manual verb | `chassis/chassis.hpp:385` `drive(ChassisSpeeds, Frame)` (F6, frozen) | pre-empts any motion, advances the localizer, runs the shared command pipeline (desaturation, brownout, strafe clamp); `Frame::Field` during the boot window commands ZERO volts with a Warn; `Frame::Body` always works |
| The loop | `src/main.cpp:420-440` `runTeleopLoop` | read sticks → map → `drive(Body)` → panel → `pacer.pace()`. **Not edited by this chunk** (§7) |
| The controller seam | `hal/controller.hpp` | `axis()` canonical [-1, 1], `pressed()` LEVELS, `ButtonEdge` per consumer (HA-104), `isConnected()` positive |
| Heading, for field-centric | `localization/localizer.hpp` (`estimate()`), `chassis.hpp` (`estimate()`/`estimateState()`) | the ESTIMATED heading and whether the estimate is live (the boot window); `diag::FaultCode::ImuLost` via the health monitor |
| The register entry | `docs/hardware-assumptions.md` HA-112 | "the teleop mapping (stick signs, 0.05 deadband) is drivable — invented; T2 owns the real driver-feel layer" |
| The plant + rigs | `test/motion_test_rig.hpp`, `sim/scenario.hpp`, `test/chassis_drive_test.cpp` | how a `Chassis` is driven tick by tick on the host; `chassis_drive_test` shows the manual verb through the plant |

---

## 2. Scope

### IN

1. **`teleop/input_shaping.hpp`** — pure functions, every one `constexpr`, PROS-free, no state:
   - `continuousDeadband(axis, deadband)`: 0 inside `(−d, +d)`; outside, the surviving range is
     RESCALED so the output leaves zero smoothly: `sign(x)·(|x| − d)/(1 − d)`. Continuous at ±d
     (the value there is exactly 0), monotonic, through (0,0) and (±1,±1).
   - `expoCurve(x, expo)`: `x·(expo·x² + (1 − expo))`, `expo ∈ [0, 1]` — monotonic (derivative
     `3·expo·x² + 1 − expo ≥ 0`), through (0,0) and (±1,±1), `expo = 0` is the identity bit for bit.
   - `shapeAxis(axis, AxisShaping{deadband, expo})` = `expoCurve(continuousDeadband(axis, d), e)`.
   - `AxisShaping` (per axis) and `DriverProfile{forward, left, yaw : AxisShaping; slew : SlewConfig;
     mode : DriveMode}` — "per-driver tuning" is a named profile a program selects; the DEFAULT
     profile reproduces today's feel as closely as a continuous deadband allows and says so
     (deadband 0.05, expo 0, slew off, `ArcadeBody`).
2. **`teleop/slew_limiter.hpp`** — `SlewLimiter{ratePerSecond}` (stateful: one value), `update(target,
   dt) → limited`; `ratePerSecond = 0` (the DEFAULT) means OFF and returns `target` unchanged, bit
   for bit — "slew limiting configurable and OFF by default: it protects the drivetrain and it adds
   lag, and lag is the enemy of feel" (build-order T2). `reset()`.
3. **`teleop/drive_modes.hpp`** — `enum class DriveMode { ArcadeBody, ArcadeField, TankSticks }` and
   the pure mode functions over a `StickInput`:
   - `ArcadeBody`: today's mapping, shaped: forward = leftY, left = −leftX, yaw = −rightX.
   - `TankSticks`: left stick Y = the LEFT side, right stick Y = the RIGHT side (the classic tank
     feel). Mapped INTO the same `DriveRequest`: `forward = (l + r)/2`, `yawCcw = (r − l)/2`,
     `left = 0` — the arcade inverse, so `tankSideVolts()` recovers `l` and `r` exactly (pinned).
     `StickInput` gains `rightY` (additive; `mapSticks` ignores it).
   - `ArcadeField`: the shaped body request's TRANSLATION rotated by `−(heading − zero)` so a
     stick pushed "away from the driver" moves the robot away from the driver whatever way it faces.
     **The heading is passed IN as a number** (with a `bool headingTrusted`), never read from a
     chassis here — the function stays pure. The result is STILL a body-frame request, so the
     caller commands `Frame::Body`: the estimate is never touched, the boot window needs no special
     case, and the fallback below is one branch.
4. **`teleop/field_centric.hpp`** — `FieldCentric` (stateful: the driver's zero offset and the
   fallback latch): `rezero(heading)` (a driver button: "the way I am facing now is away from
   me"), `request(shapedBody, heading, headingTrusted) → {DriveRequest, FieldCentricState}` where
   `FieldCentricState{active, fellBack, reason}` is what a panel shows. **The honest failure mode
   (build-order T2, "do not ship field-centric without it"):** when `headingTrusted` is false (the
   boot window, or IMU_LOST latched — the caller's two inputs) the function returns the BODY
   request unrotated with `fellBack = true` and a reason, and it STAYS fallen back until the driver
   re-zeroes (a latch, not a flicker: controls that silently flip between two frames mid-match are
   worse than controls that stay in one). Nothing here tares the IMU or moves the estimate (HA-05).
5. **`teleop/driver_control.hpp`** — `DriveShaper` (stateful: three `SlewLimiter`s + a
   `FieldCentric`): `shape(StickInput, dt, heading, headingTrusted) → DriveRequest` under a
   `DriverProfile`. ONE call per tick, in this order: mode mapping → per-axis shaping → slew →
   field-centric rotation. Exposes `state()` (`FieldCentricState` + the slew's last outputs) for
   panels. This is the object `runTeleopLoop` will construct and call (§7); it does not read a
   controller or a chassis — numbers in, a request out — so a test drives it without either.
6. **One-tick latency, pinned as a property** (build-order T2): a test through the plant shows a
   stick step at tick k appearing in the commanded volts of tick k (no buffering, no queue).
   Perceived latency is NOT claimed (PROS call cost + loop rate are R4's).
7. **`stick_mapping.hpp` header comment** updated: T2 has landed; the naive deadband stays as
   `deadbanded()` for the bit-identity pin and the legacy callers, marked as superseded by
   `continuousDeadband` — NOT deleted (the drive program's pin depends on it until §7 re-points it).
8. Tests (§4), the DoD (§5), the documentation contract (§6).

### OUT

- **`src/main.cpp` / `src/drive_program.cpp` / `src/bench_r3a.cpp` integration** → the coordinator,
  after R3d Part 2 lands (they share `main.cpp`). §7 says exactly what that integration is.
- **T3** (button bindings, macros, the mechanism seam) → its own chunk.
- **Measured feel** (which curve, which deadband, which slew a driver prefers) → the bench, with a
  driver; every default here is INVENTED and says so.
- **Holonomic strafing feel** (the `left` axis) is mapped and shaped but its authority is the
  kinematics' business (C3's strafe clamp); nothing here special-cases it.

### REJECTED

- **A curve with a dead zone at full deflection** or any non-monotonic "feel" curve: "pushing
  further can slow the robot down" (build-order T2).
- **Slew on by default.** Lag.
- **Field-centric through `Chassis::drive(…, Frame::Field)`.** The facade's field frame is
  correct, but routing through it means the fallback needs the facade's cooperation and the
  boot-window rule silently zeroes the drive; rotating in the shaper and always commanding
  `Frame::Body` keeps the fallback a pure branch and the boot window a non-event. (The facade's
  field verb stays frozen and untouched; autonomous callers may still use it.)
- **Re-zero by taring the IMU or resetting the localizer.** HA-05: a tare under a live
  bootHeading offset silently corrupts the estimate; the driver's zero is the shaper's offset.

---

## 3. Rulings

**S1 — Deadband continuity is the load-bearing property, and the rescale is the fix**, not a
smoothing function: `(|x| − d)/(1 − d)` keeps full deflection at exactly 1 (a driver must be able
to ask for everything) and leaves zero at exactly 0 at the threshold. Rejected: a cubic blend
(more parameters for no felt gain; harder to reason about at the ends).

**S2 — Expo is the one curve shape offered.** It is monotonic for every `expo ∈ [0, 1]`, identity at
0, and it is what every RC transmitter calls "expo", so a driver's intuition transfers. Rejected: a
free polynomial (a non-monotonic one is easy to type).

**S3 — Tank-stick mode maps INTO the arcade request rather than adding a second request type**, so
one `DriveRequest` still feeds both the library pipeline and `tankSideVolts()`; the inverse is
exact and pinned. Rejected: a per-side request (two representations of one command — C6's
catalogued failure).

**S4 — The field-centric fallback is a LATCH cleared only by the driver's re-zero.** See §2 item 4.

**S5 — Order inside `shape()` is mode → shaping → slew → rotation**, and it is a documented
contract: shaping before slew (a slew-limited signal through a curve changes the slew rate the
driver set), rotation last (slew in the driver's frame, not the robot's, so a constant stick under a
spinning robot does not fight the limiter).

**S6 — Every default is INVENTED and tagged `PROVISIONAL (A4: HA-112)`** (the existing entry's
scope widens to the feel layer; no new register number is minted here — see §6).

---

## 4. Tests — `test/input_shaping_test.cpp`, `test/drive_modes_test.cpp`, `test/driver_control_test.cpp`

| # | Test | Property | Mutation (must go red) |
|---|---|---|---|
| 1 | `continuousDeadband`: 0 inside; EXACTLY 0.0 at ±d; monotonic across a fine sweep (each step ≥ previous, both signs); `(±1) → ±1` exactly; `d = 0` is the identity bit for bit | continuity + the ends | naive cut (`x` outside) — the jump at d is caught by the "just above d is tiny" assertion |
| 2 | `expoCurve`: identity at 0 bit for bit; through (0,0), (±1,±1) exactly for every expo on a grid; monotonic on a fine sweep for expo ∈ {0, 0.25, 0.5, 0.75, 1}; odd symmetry | monotonic, the ends | `x·(expo·x² + 1)` (leaves (1,1)) |
| 3 | `SlewLimiter`: rate 0 ⇒ output == target bit for bit; rate r ⇒ |Δ| ≤ r·dt per update, reaches the target in the expected number of ticks (hand-computed); `reset()`; a sign reversal is limited through zero | OFF is off; the limit is the limit | ignore dt; skip the clamp on the negative side |
| 4 | `TankSticks` inverse: for a grid of (l, r), `tankSideVolts(mapTank(l, r), V)` == (V·l, V·r) exactly (no clamp engaged inside the unit square) | S3 | swap the halves |
| 5 | `ArcadeField`: heading 90° CCW, stick UP ⇒ body request LEFT… (hand-derived rotations at 0/90/180/270 and one odd angle); `headingTrusted = false` ⇒ unrotated + `fellBack`; the latch holds after trust returns until `rezero()`; `rezero(h)` makes "stick UP at heading h" mean "body forward" | the honest failure mode | drop the latch (flickers back); rotate by +heading instead of − |
| 6 | `DriveShaper` order: a case where shaping-then-slew and slew-then-shaping differ, pinned to the documented order | S5 | swap the order |
| 7 | Default profile vs today's mapping: on a sweep, the shaped request equals `mapSticks()` everywhere OUTSIDE the deadband to within the rescale (documented, hand-computed bound) and equals it EXACTLY at 0 and ±1 | the default does not surprise a driver who drove yesterday | — (documents the change) |
| 8 | One-tick latency through the plant: a `Chassis` driven by `shape()` each tick; a stick step at tick k changes the commanded wheel volts at tick k | build-order T2's "pinned" | insert a one-tick buffer |
| 9 | Disconnected controller ⇒ all-zero request through every mode, whatever the axes | HA-103 | drop the check in one mode |
| 10 | NaN/inf axes: the pure functions are total (no precondition); the shaper's output is finite for finite inputs and a non-finite input is REJECTED where `Chassis::drive` always rejected it (the test shows the precondition still fires there, not silently before) | the contract's home does not move | — |

Every mutation RUN, observed red, message in the log, restored.

---

## 5. DoD

- [ ] Items 1–7 of §2; every public member `///`-documented; ARM gate green with the new headers
- [ ] Tests 1–10 green; mutations logged red
- [ ] `test/stick_mapping_test.cpp` still green and UNCHANGED (the R1a bit-identity pin is not
      this chunk's to move)
- [ ] The whole suite green; PROS-free and layering guards; `docs/api/` regenerated
- [ ] No `src/` file touched; no plan document touched (§7)
- [ ] `T2-PROGRESS.md` complete: rulings beyond §3, mutations, the prose-review log, the suite count

---

## 6. Documentation — what THIS executor touches, and what it must not

Touch: (1) `docs/api/` regenerated (the freshness gate); (2) `docs/hardware-assumptions.md` —
**edit HA-112's entry text only** (its claim widens to "the feel layer's defaults are drivable":
deadband, expo 0, slew off, the fallback latch; source lines move to `teleop/input_shaping.hpp` and
friends). **Mint NO new HA numbers** — R3d Parts 1–2 are numbering entries in the same file in the
main tree tonight; put any new belief in `T2-PROGRESS.md` under "register entries to add" and the
coordinator numbers them at integration; (3) `docs/guide/14-what-it-cannot-do-yet.md` — the bullet
"`drive()` is a primitive, not a driver-control product" (`:316-318`) becomes true-as-of-T2 (the
shaping exists, on the host; nothing here has been felt by a driver — say that); (4)
`teleop/stick_mapping.hpp`'s header (§2 item 7).

Do NOT touch: `docs/roadmap.md`, `docs/internal/build-order.md`, `PROJECT-BRIEFING.md`, `README.md`,
`src/*`, `docs/guide/10-the-api.md` (the coordinator does these at integration, §7). Record in the
log, per changed source file, the prose review: grep `docs/` and `README.md` for `stick_mapping`,
`deadband`, `HA-112`, `driver feel`, `T2`, `field-centric`, read the hits, list what is stale and
whether you fixed it or left it for integration (the untouchable files).

---

## 7. Integration (the coordinator, after R3d Part 2 lands in `main.cpp`)

`runTeleopLoop` constructs a `DriveShaper` with the default profile and calls `shape()` with
`dt` from the pacer and the heading + trust from the chassis (`estimateState()` live and no
IMU_LOST latched); a controller button (T3 will own bindings; for now one hard-wired `ButtonEdge`)
re-zeroes field-centric; the panel shows `FieldCentricState`. The drive program and the tester's
DRIVE station switch `mapSticks` → `shape()` with a profile whose expo is 0 — their bit-identity pins
move to the new default in the same commit. HA-112's settle line and the roadmap's Phase T rows,
build-order's T2 entry + `Next:` block, `README.md`'s "what it is not yet", the briefing — all at
integration, in the same commit as the `src/` change.

---

## 8. Landmines

1. **The sign of zero.** `stick_mapping.hpp` pins negate-then-deadband so a centred stick yields
   +0.0; keep the new functions returning +0.0 inside the deadband (`0.0`, never `−x·0`), or a
   later bit-identity pin fails for a reason nobody can see.
2. **`expo = 0` and `rate = 0` must be bit-identical no-ops**, not "close": tests 2 and 3 compare bits.
3. **Rotation direction.** The body frame is CCW-positive and the driver's stick vector must be
   rotated by MINUS the (heading − zero) to become a body request. Test 5's odd angle is the
   catch; a 90°-only test passes with the sign wrong.
4. **No `<pros/…>` anywhere in `include/shulib/teleop/`** — the CI grep is exact.
5. **No allocation, no `std::string`, no `std::function`** in these headers (they run on the V5).
6. **Do not "improve" `deadbanded()` or `mapSticks()`** — the R1a pin depends on them verbatim.
7. **No feel claim.** Nothing here has been driven; every default is invented and the headers say so.
