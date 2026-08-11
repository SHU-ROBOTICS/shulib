# Chunk C3 — `HDriveKinematics` + the pseudo-inverse

> **Phase C, chunk 3 of 7.** Predecessors: C1 ✅ (motion primitives), C2 ✅ (`MotionScheduler`).
> This is where the **15″ H-bot runs the same motion code as the 24″ X-bot, unmodified.**

**Workstream:** WS3 (Math & kinematics) · **Milestone:** M2 · **Carries an M1 deferral**

---

## Why this chunk, and why here

Two robots, one library. The X-drive is fully holonomic; the H-drive is a tank base with a single
transverse strafe wheel and **limited strafe authority**. If the motion layer needs to know which robot
it is driving, the abstraction has failed.

**Ordered before the `Chassis` facade on purpose.** C4 builds the facade and D2 freezes it as **F6** —
the contract every auton ever written depends on. Discovering *after* the freeze that the H-drive needs
a different motion contract would be an F6 break. Finding it now is free.

It also settles a question C1 and C2 both left open: **C1 flagged that its strafe-authority
interpretation "awaits C3 confirmation"** (its decision D11), and closed mutation #5 using a
fractional-authority 0.35 drive explicitly described as *"the C3 contract shape."* C3 is where that
assumption meets a real drive whose authority is genuinely less than one.

---

## What already exists

| Thing | Where | Note |
|---|---|---|
| `MatrixKinematics` engine | `kinematics/matrix_kinematics.hpp` | **`forward()` currently requires orthogonal columns** — rank-3 + orthogonality preconditions |
| `xDrive()`, `tank()` presets | `kinematics/x_drive.hpp`, `tank.hpp` | the two drives that must not regress |
| `IKinematics` contract (**F5**) | `kinematics/kinematics.hpp` | `toWheels`/`forward`/`desaturate`/`strafeAuthority()`/`wheelCount` |
| `desaturateUniform` | `kinematics/desaturate.hpp` | over-budget scaling |
| The motion primitives | `motion/` | must run **unmodified** on the H-drive |
| `MotionScheduler` | `motion/motion_scheduler.hpp` | must run a full H-drive routine |
| `DrivePlant` (X-drive + tank today) | `sim/drive_plant.hpp` | needs an H-drive configuration |
| `strafeFallbackActive` | `diag/debug_record.hpp` | **the field already exists** (A1 reserved it) — populate it |
| Hostile models | `sim/hostile/` | the H-drive must survive them too |

**Read first:** `C1-COMPLETED.md` (especially decision **D11** on strafe authority, and §10's handoffs),
`C2-COMPLETED.md` (§ the F6-inherited shapes), `build-order.md` Phase C, `RESUMING.md`,
`hardware-assumptions.md`.

---

## Scope

### In
1. **`HDriveKinematics`** — tank base + transverse strafe wheel
2. **Capped strafe authority** with an **automatic turn-then-drive fallback**, telemetry-visible via
   `strafeFallbackActive`
3. **The pseudo-inverse** — generalize `MatrixKinematics::forward()` to full `(AᵀA)⁻¹Aᵀ` so
   non-orthogonal drives work (the H-drive's strafe wheel is off-centre). *This is the M1 deferral;
   it relaxes a precondition only, so it is **F5-safe**.*
4. **An H-drive plant configuration** so the motion and routine tests can actually run
5. **Confirm or correct C1's strafe-authority interpretation** (its D11)

### Out
- `Chassis` facade → C4 · recipe API → D1 · mechanisms → F
- Mecanum / swerve → Frontier

---

## Design constraints

### 1. The motion layer must not change
C1's primitives and C2's scheduler run on the H-drive **unmodified**. If they need edits, that is a
finding about the abstraction and must be reported loudly — not patched around with a special case.
This is the M2 Definition of Done: *"the same auton runs the H-bot."*

### 2. ⚠️ Turn-then-drive: forbidden as a default, correct as a fallback
C1's brief carried a landmine: *"don't turn-then-drive — if the implementation ever sequences rotation
before translation, the holonomic thesis is lost."* That stands **for a drive that can strafe.**

The H-drive genuinely *cannot* strafe past its authority — this is physics, not a design choice. So a
turn-then-drive fallback is the **correct** behaviour there, and the distinction must be explicit in
the code and its comments: the X-drive never falls back; the H-drive falls back only when the commanded
strafe exceeds what the hardware can deliver. Make the fallback **visible** (`strafeFallbackActive`),
never silent — a robot quietly changing its motion strategy is a robot you cannot debug.

### 3. F5: kinematics never clamps
`toWheels()` still must not clamp internally. `strafeAuthority()` stays a **pure read-only query**.
The motion layer clamps — C1 already implements that. C3's job is to supply an honest authority number
and confirm C1's clamp does the right thing when authority < 1.

### 4. The pseudo-inverse must not regress the orthogonal cases
For X-drive and tank, `(AᵀA)⁻¹Aᵀ` must reduce to exactly what `forward()` computes today. Prove it —
byte-identical or within a documented tolerance, across a swept input space. A regression here breaks
every existing motion test.

### 5. Numerical conditioning is a real risk
`(AᵀA)⁻¹` can be ill-conditioned for near-degenerate geometries. Guard it: reject or flag configurations
whose condition number is poor, rather than silently returning garbage. Document the threshold, and
give it an `HA-nn` entry if it is a chosen constant.

### 6. Strafe authority must be derived, not invented
It is a function of the geometry — wheel placement, radius, motor capability. Derive it. If any part is
a guess, label it `PROVISIONAL (A4: HA-nn)` and register it.

### 7. Respect the standing contracts
A1's cost contract (`emitRecord`; `wantsRecord()`/`emit()` as a pair); injected clock; PROS-free;
strict `-Werror`; both CI guards and the ARM gate.

---

## Test requirements

Hold C1/C2's escalated bar — swept and seeded, every test naming the bug it would catch.

- **No regression** — the pseudo-inverse reproduces current `forward()` for X-drive and tank across a
  swept space; every existing kinematics test still passes untouched
- **H-drive round-trip** — `toWheels` → plant → observed twist, swept, within the authority envelope
- **Strafe authority** — the derived number matches the geometry; a swept authority sweep behaves
- **Beyond authority** — commanded strafe past the cap is clamped by the motion layer, the fallback
  engages, and `strafeFallbackActive` is set. **A silent fallback is a failing test.**
- **C1's primitives unmodified** on the H-drive — each reaches its target and settles
- **C2's scheduler** runs a full routine on the H-drive
- **Routine accuracy on the H-drive** — the same three-way regression as C1 (error vs. move count, vs.
  distance, vs. time). Error must stay **flat in move count**. Report the H-bot's numbers next to the
  X-bot's; they will differ, and the difference is information.
- **Hostile survival** — the H-drive under A3 composed hostility
- **Conditioning** — a near-degenerate geometry is rejected or flagged, not silently wrong
- **Degenerate** — pure strafe at exactly the authority limit; strafe with zero forward; a geometry
  with the strafe wheel exactly on centre (does authority go to a sensible limit?)

**Mutations — go well past four.** C1 found two green holes, C2 found one test that was vacuous.
**A mutation that stays green, or a test that proves nothing, is the most valuable thing you can
find.** Record every result as observed.

---

## Definition of Done

- [ ] `HDriveKinematics` implemented; strafe authority derived from geometry
- [ ] `MatrixKinematics::forward()` generalized to the pseudo-inverse, orthogonal cases proven unchanged
- [ ] Turn-then-drive fallback works, is visible via `strafeFallbackActive`, and is never silent
- [ ] **C1's primitives and C2's scheduler run on the H-drive with no motion-layer changes**
- [ ] H-drive routine accuracy measured, error flat in move count, three-way regression reported
- [ ] C1's D11 strafe-authority interpretation confirmed — or corrected, with the correction explained
- [ ] Ill-conditioned geometries rejected or flagged
- [ ] Any invented constant carries an `HA-nn` entry
- [ ] Suite green under strict `-Werror`; both guards pass; ARM gate passes

---

## Live progress log — required

`docs/internal/chunks/C3-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`), vocabulary
`START` / `DONE` / `MUTATE` / `DECIDE` / `BLOCKED` / `FOUND`.

---

## Documentation contract

All six, plus **`docs/internal/chunks/C3-COMPLETED.md`** at C1/C2 depth (570 and 605 lines), including
a **"What we now know for certain, and what we do not"** section for a reader who was not present.

**F6 is the next chunk but one.** C2 flagged eight shapes the facade will inherit. **Add or amend that
list** with anything C3 reveals — particularly whether the facade needs to expose strafe authority or
the fallback state, since the H-drive is the reason it would.

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **A silent fallback is a debugging nightmare.** Always flag it.
- **Don't special-case the motion layer per drivetrain.** If you need to, that is the finding — report it.
- **Don't regress X-drive or tank.** The pseudo-inverse must be a strict generalization.
- **Don't invent an authority number.** Derive it, or register it as provisional.
- **`(AᵀA)⁻¹` bites.** Guard the conditioning; silent garbage from a near-singular matrix is the worst
  possible failure here.
