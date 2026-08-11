# Chunk D1 — the recipe API (Tier 2)

> **Phase D, chunk 1 of 3.** Predecessor: Phase C complete (C1–C8).
> **D1's real job is to be a critical second consumer of the `Chassis` facade before D2 freezes it.**

**Workstream:** WS12 (accessibility) · **Milestone:** M7, pulled forward · **Sets up:** F6 freeze at D2

---

## Why this chunk exists, and why it is *here*

Two reasons, and the first is a correctness argument rather than a convenience one.

**1. It is the second independent consumer.** C4 built the facade and deliberately did **not** freeze
it. A contract exercised only by its author has been tested against the intent that produced it. D1
is the first outside consumer — and if the facade is awkward to build on, that has to surface **now**,
while changing it is still free. After D2 it changes only by version bump and migration.

**Finding awkwardness is success, not failure.** A D1 that reports "the facade was perfect" has
probably not pushed hard enough. C4's completion record §8 names specific tensions it wants probed —
start there.

**2. It is the surface your team actually writes against.** Students must author and defend their own
competition code. The roadmap parked this at M7, behind fusion and mechanisms — which would mean the
whole season gets written against the raw expert API. It is a thin layer over a facade that already
exists; there is no dependency justifying the delay.

---

## What already exists

| Thing | Where |
|---|---|
| `Chassis` facade — `moveTo` / `strafeTo` / `turnTo` / `followTrajectory` / `drive` / `brake` / `hold` | `include/shulib/chassis/chassis.hpp` |
| `MotionScheduler` — `async` / `waitUntilSettled` / `waitUntil` / `cancel` | `motion/motion_scheduler.hpp` |
| `ExitReason`, `CompletedMotion`, fault policy | `control/`, `motion/` |
| Typed units and `Pose2d` | `units/`, `math/` |
| The plant, hostile models, `ChassisRig` | `sim/`, `test/motion_test_rig.hpp` |
| **Guide chapter 09 — reserved and vacant, waiting for this** | `docs/guide/` |
| Working examples, compiled by CI | `test/guide_examples_test.cpp` |

**Read first:** **`C4-COMPLETED.md` §8 (the F6 candidate surface and its named tensions)**, master plan
**§17** (the four accessibility tiers), `C2-COMPLETED.md` §11 and `C3-COMPLETED.md` §11 (inherited
shapes), `docs/internal/guide-maintenance.md` (how to add the guide chapter), `RESUMING.md`.

---

## Scope

### In
1. **The recipe API** — a fluent layer where a complete routine is ~10 readable lines
2. **A critical report on the facade** — every place it was awkward to build on, with a recommendation
   for D2: change it now, or accept it and why
3. **Guide chapter 09** plus compiled examples, per `guide-maintenance.md`

### Out
- **Do not freeze F6.** That is D2, informed by what you find here.
- Mechanisms don't exist yet (F1/F3) — shape the seam for them, don't invent them
- Cookbook and generated API docs → D3 · `.vexbot` authoring → Phase G

---

## Design constraints

### 1. No cliff between tiers — this is the whole idea
§17's tiers are **strict supersets**. Anything expressible in a recipe must remain expressible in the
full API, and dropping from Tier 2 to Tier 3 must be a step, never a rewrite. A student who outgrows
recipes should keep everything they've learned.

Concretely: recipes **delegate**. They add no motion logic. If a recipe needs behaviour the facade
lacks, that is a finding about the facade — report it, don't implement around it.

### 2. Hard to misuse
Typed units (a bare `double` where a length belongs must not compile). Mistakes loud and early.
Sensible defaults so the common case is short, with every default overridable.

### 3. Readable by someone who isn't a programmer *yet*
The test: could a first-year read a 10-line routine and correctly say what the robot will do? Name
things for what they do on the field, not for what they do in the code.

### 4. Errors must not vanish
The facade returns honest `ExitReason`s. A fluent chain makes it *easy* to swallow them — decide
deliberately what a chain does when a step times out or faults (stop? continue? report at the end?),
document it, and test it. **A routine that silently continues after a failed move is worse than one
that stops.**

### 5. Every guarantee survives
Through recipes: watchdog bounds, cancellation to a safe state, the `ODO_STUCK` abort policy,
hostile degradation, routine error flat in move count. Test them *through* the recipe layer.

### 6. Shape the mechanism seam, don't fake it
§17's example is `chassis.moveTo(p).then(intake.in)`. `intake` doesn't exist until F1/F3. Design so a
mechanism action slots in later without reshaping the chain, and say plainly in the header what is
placeholder.

### 7. Standing contracts
A1's cost contract; injected clock; PROS-free; strict `-Werror`; both guards and the ARM gate; any
invented constant gets an `HA-nn` entry.

---

## Test requirements

Every test names the bug it would catch.

- **A complete routine in ~10 lines** runs correctly on **all three drivetrains**
- **No capability lost** — for each facade verb, a recipe reaches the same outcome; where it can't,
  that's a documented, deliberate gap
- **Error handling** — a timing-out step behaves per the documented policy, tested, not assumed
- **Guarantees preserved through recipes** — watchdog, cancel-to-safe-state, `ODO_STUCK` abort,
  hostile survival
- **Accuracy through recipes** matches the C1–C4 baselines (error flat in move count)
- **Misuse rejected** — wrong units fail to compile; nonsense inputs rejected loudly
- **Tier interop** — a routine mixing recipes and direct facade calls works, proving there's no cliff
- **The guide's chapter 09 examples compile** and are quoted verbatim

**Mutations — go well past four.** Every prior chunk found something ordinary tests missed: two green
holes (C1), a vacuous test (C2), a structurally-uncatchable mutation (C3), two green survivors
including fault observables dark in teleop (C4), and dead wiring that 915k assertions couldn't see
(C5). **A green mutation or a vacuous test is the most valuable thing you can find.** Gate the runner
on build success.

---

## Definition of Done

- [ ] Recipe API implemented; a real routine reads in ~10 lines and works on all three drivetrains
- [ ] Delegates only — no motion logic duplicated
- [ ] Error policy for chains decided, documented, tested
- [ ] Every lower-layer guarantee verified **through** the recipe layer
- [ ] No capability lost versus the full API; mixed-tier usage works
- [ ] **A written critique of the facade** — every awkwardness, each with a D2 recommendation
- [ ] Guide chapter 09 written, examples compiled and quoted verbatim
- [ ] **F6 still NOT frozen**; the register still shows it pending D2
- [ ] Suite green; both guards pass; ARM gate passes

---

## Live progress log — required

`docs/internal/chunks/D1-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`D1-COMPLETED.md`** at the depth of C1–C5 (570–654 lines). Give the **facade critique
its own prominent section** — that section is D2's input, and D2 is a freeze that lasts.

**Do not commit. Do not push.**

---

## Landmines

- **Don't be polite about the facade.** Awkwardness you don't report becomes permanent at D2.
- **Don't add motion logic in recipes.** Delegate; report gaps.
- **Don't let a chain swallow a failure.** Decide the policy and test it.
- **Don't invent mechanisms.** Shape the seam, label the placeholder.
- **Don't freeze F6.**
