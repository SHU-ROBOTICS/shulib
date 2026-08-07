# Chunk C4 — the `Chassis` facade (built, **not** frozen)

> **Phase C, chunk 4 of 7.** Predecessors: C1 ✅, C2 ✅, C3 ✅.
> **This shape becomes F6 — the contract every auton ever written depends on.**

**Workstream:** WS6 / Chassis · **Milestone:** M2 · **Freeze:** F6, at **D2** — *not here*

---

## Why this chunk matters more than its size

`Chassis` is the API a student writes an auton against. Once **F6 freezes at D2**, it changes only by
version bump plus migration — never a silent break. That is the promise that keeps the whole roadmap
true.

**So C4 builds it and deliberately does not freeze it.** D1 (the recipe API) becomes a *second
independent consumer*; only after that does D2 freeze. A contract exercised once has been exercised by
its author.

This is the last chunk where getting the shape wrong is free. Spend the effort here.

---

## The three predecessors' handoffs — all land here

| From | What it asked C4 to do |
|---|---|
| **C1** | Wrap `MotionDeps` + `MotionConfig` + the primitives into the facade |
| **C2** | **Construct the scheduler AND every motion from ONE deps source** — this makes command-id stamping *structural* instead of a convention (C2's stated limitation) |
| **C3** | Accept `HDriveConfig`/`hDrive()` naturally in the builder; expose `strafeAuthority()` passthrough; **no** fallback getter (C3's recommendation) |

**C2 §11 and C3 §11 between them list ~12 shapes F6 will inherit. Work through that list explicitly
and resolve each** — the completion record must show every one either adopted, rejected with a reason,
or deferred to D1/D2 with a named owner. Anything missed becomes permanent at D2.

---

## Scope

### In
- **`Chassis`** with the F6 verbs: `moveTo` · `strafeTo` · `turnTo` · `followTrajectory` ·
  `drive(ChassisSpeeds, Frame)`
- **One construction path** feeding both the scheduler and every motion (C2's handoff)
- **All three drivetrains** — X-drive, tank, H-drive — accepted naturally
- **A direct C++ construction path with no config file** (the standalone promise)
- Resolution of the inherited-shape list

### Out
- **Do not freeze F6.** That is D2's job, after D1.
- Recipe API → D1 · per-motion result formatting → C5 · path *authoring/markers* → G2
- `PathRunner`, command-id registry, `.vexbot` ingestion → Phase G

### `followTrajectory` — define the shape, keep the body honest
F6 freezes the *verb set*, so the signature must exist now. A minimal implementation — chain waypoints
through the scheduler — is legitimate and useful. Marker callbacks, command ids, and `.vexbot` paths
are **G2**. Do not fake them; make the boundary explicit in the header.

---

## Design constraints

### 1. The standalone promise is a locked principle
*"shulib works with nothing else installed; VexBuilder makes it better, never required."* A
code-fluent team must build a working `Chassis` **in plain C++ with no `.vexbot` file at all**. G1's
`RobotBuilder.from(profile)` is an *additional* path, never the only one. Test the file-free path.

### 2. `drive(ChassisSpeeds, Frame)` is explicitly frame-parameterized
Field-relative and body-relative are both first-class, and the caller says which. This is also the
verb a driver-control loop would use for field-centric driving. **Frame confusion is the classic bug
class this project was rebuilt to prevent** — make the parameter impossible to get wrong silently.

### 3. Hard to misuse, per §17 Tier 3
The full API is the expert tier, but it should still make mistakes loud: typed units, no bare doubles
where a dimension exists, preconditions on nonsense inputs. D1's recipe layer will sit on top —
**anything expressible in recipes must remain expressible here**, with no capability lost.

### 4. Do not re-implement the layers below
The facade **delegates**. Motion logic is C1's, scheduling and fault policy are C2's, kinematics are
C3's. If the facade needs behaviour those layers don't provide, that is a finding about them — fix it
there (rule 4), don't duplicate it here.

### 5. Preserve everything the layers guarantee
Through the facade: motions still report `ExitReason`; the watchdog still bounds them; `cancel()` still
reaches the defined safe state; the fault policy still fires; hostility still degrades rather than
diverges; routine error stays flat in move count. **A facade that quietly loses a guarantee is worse
than no facade.** Test each one *through* the facade, not just below it.

### 6. Standing contracts
A1's cost contract (`emitRecord`; `wantsRecord()`/`emit()` as a pair); injected clock; PROS-free;
strict `-Werror`; both CI guards and the ARM gate; any invented constant gets an `HA-nn` entry.

---

## Test requirements

Hold the escalated bar. Every test names the bug it would catch.

- **A complete hand-written auton** runs through the facade on **all three drivetrains**
- **File-free construction** — a working `Chassis` built in plain C++, no config, no VexBuilder
- **Frame correctness** — `drive()` in field vs body frame produces demonstrably different, correct
  motion; sweep headings (a frame bug shows as heading-dependent error)
- **Guarantee preservation** — through the facade: `ExitReason` correct, watchdog bounds, `cancel()`
  safe state reached, `ODO_STUCK` abort policy fires, hostile survival holds
- **Routine accuracy through the facade** matches C1/C2/C3's baselines — error **flat in move count**;
  report all three drivetrains
- **Id stamping is structural** — a motion built through the facade is always stamped; demonstrate
  that C2's convention-only gap is closed
- **`followTrajectory`** — chains waypoints correctly; its documented boundary (no markers yet) holds
- **Misuse** — wrong units rejected at compile time where possible; nonsense inputs rejected loudly
- **Adversarial** — `drive()` called while a motion is active; facade methods during the boot window;
  a facade outliving its context; back-to-back conflicting commands

**Mutations — go well past four.** C1 found two green holes, C2 found a vacuous test, C3 found a
mutation that closed-loop tests structurally cannot catch. **A green mutation or a vacuous test is the
most valuable thing you can find.** Record every result as observed.

---

## Definition of Done

- [ ] `Chassis` with all five F6 verbs, delegating rather than re-implementing
- [ ] One construction path feeds scheduler and motions; **id stamping structural**
- [ ] All three drivetrains work; file-free C++ construction tested
- [ ] Every lower-layer guarantee verified **through** the facade
- [ ] Routine accuracy through the facade matches prior baselines on all three drivetrains
- [ ] Every inherited-shape item from C2 §11 / C3 §11 resolved: adopted, rejected with reason, or
      deferred with an owner
- [ ] **F6 explicitly NOT frozen** — the Freeze Register still shows it pending D2
- [ ] Suite green under strict `-Werror`; both guards pass; ARM gate passes

---

## Live progress log — required

`docs/planning/chunks/C4-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`docs/planning/chunks/C4-COMPLETED.md`** at C1–C3 depth (570/605/654 lines), with a
prominent **"What we now know for certain, and what we do not"** section for a reader who was not
present.

**Dedicate a section to the F6 candidate surface**: the exact verb signatures proposed for freeze, and
for each, why that shape. D1 will stress it and D2 will freeze it — this section is what they read.

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **Don't freeze F6.** D1 must get a chance to find the awkwardness first.
- **Don't lose a guarantee behind the facade.** Test each one through it.
- **Don't re-implement C1/C2/C3.** Delegate; if something's missing, fix it at its source.
- **Don't break the standalone promise.** No config file may ever be required.
- **Don't fake `followTrajectory`.** Minimal and honest beats complete and misleading.
