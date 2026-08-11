# Chunk D2 — the F6 freeze

> **Phase D, chunk 2 of 3.** Predecessor: D1 (the recipe API, the second consumer).
> **This is a freeze. It is the most expensive-to-reverse chunk in the project so far.**

**Workstream:** WS6 · **Milestone:** M2 · **Freezes:** **F6 — the public `Chassis` API**

---

## Why this chunk exists, and why it is *here*

F6's dependency column reads *"Every auton ever written on shulib."* That is not decoration. After
this chunk, the surface changes only by version bump plus migration — so every awkwardness not fixed
here becomes a thing the team lives with for seasons, and every shape frozen carelessly becomes a
thing the library must keep honouring.

The order was built for this moment. C4 deliberately did **not** freeze the facade it built, because
a contract exercised only by its author has been tested against the intent that produced it. D1 then
consumed it from outside and wrote down every place it chafed. **D1-COMPLETED §2 is this chunk's
input packet** — nine items, each already carrying a recommendation. D2's job is to rule on them,
not to rediscover them.

**A freeze is a decision chunk, not a construction chunk** — with two exceptions that are real
engineering (§Scope: the time retype and the signature pin). Resist the urge to build.

---

## What already exists

| Thing | Where |
|---|---|
| **The critique packet — nine items with recommendations** | `D1-COMPLETED.md` §2 |
| The candidate surface + why each shape | `C4-COMPLETED.md` §8 |
| **The inherited-shape ledger, 18 rows** | `C4-COMPLETED.md` §2 |
| C4's decision log (14 decisions) | `C4-COMPLETED.md` §5 |
| D1's decision log (12 decisions) | `D1-COMPLETED.md` §6 |
| The facade | `include/shulib/chassis/chassis.hpp` |
| The recipe layer (new surface, scope question below) | `include/shulib/chassis/routine.hpp` |
| The Freeze Register + its stated semantics | `docs/roadmap.md` §Freeze Register (line ~113) |
| The three "not frozen" notices | guide ch. 09, ch. 10, ch. 14 + `chassis.hpp` header |

**Read first, in this order:** **`D1-COMPLETED.md` §2** (the packet), then **`C4-COMPLETED.md` §8**
(what each shape is for), then **`C4-COMPLETED.md` §2** (the ledger — C4 §10 explicitly instructs D2
to check *every row* before locking), then `chassis.hpp` and `routine.hpp` in full, then the Freeze
Register's own preamble (line 115–117 — it states what a freeze *means*, and D2 must make that
sentence true).

---

## Three things that are wrong right now, which D2 must fix

These were found while writing this brief. They are not optional.

### 1. The register row is narrower than the surface it claims to freeze

The F6 row enumerates five verbs:
`moveTo`/`strafeTo`/`turnTo`/`followTrajectory`/`drive(ChassisSpeeds,Frame)`.

The real candidate surface is **~19 members plus three public types** (`ChassisConfig`,
`MotionOptions`, `TrajectoryResult`) — including `brake`, `hold`, `cancel`, `waitUntil`, `pose`,
`setPose`, `strafeAuthority`, `lastExitReason`, `lastCompleted`, `motionConfig`, and the Tier-3 seam
`deps()`/`scheduler()`. Freezing a row that lists five things, while the header exposes twenty, means
nobody can later say what was frozen. **D2 rewrites the row to enumerate the surface exactly**, or
states explicitly and in the row which members are deliberately outside F6.

### 2. "Frozen" currently has no mechanism behind it

The register says contracts change "only with a `schemaVersion`/API-version bump and a migration
path." **No such mechanism exists** — `grep -rniE "schemaVersion|apiVersion|SHULIB_VERSION|kVersion"`
over `include/` and `src/` returns nothing. F1–F5 are marked LOCKED against a promise with no
machinery.

D2 must decide what a version bump concretely is for a header-only C++ API and **write it down**.
The minimum honest answer is a documented policy (what constitutes a breaking change; where the
version lives; what a migration note looks like). A version constant in a header is cheap and makes
the promise inspectable. F7/F8/F9 are data-schema freezes that will need the same answer, so
whatever D2 establishes is reused three more times — get it right once.

*Scope discipline:* D2 defines the policy for **API** freezes and may add the version constant. It
does **not** design `.vexbot` schema versioning (F7/F8, Phase G) — only note that the policy must
extend there.

### 3. A freeze with no enforcement is a comment

Nothing today would stop a future chunk from quietly changing `moveTo`'s signature. The project's own
pattern is to make guarantees **structural rather than conventional** — C4 closed C2's stamping gap
that way, and it is why command-id stamping cannot be forgotten. **D2 ships a signature-pin test**:
a compile-time assertion of the exact type of every frozen member, so an accidental reshape fails the
build with a message naming F6. This is the chunk's main engineering deliverable and the thing that
makes the freeze real.

---

## Scope

### In

1. **Rule on all nine items in D1-COMPLETED §2** — each ADOPT / REJECT / DEFER-with-named-owner, each
   with reasoning recorded.
2. **Re-check every row of C4-COMPLETED §2's 18-row ledger** (C4 §10's explicit instruction).
3. **The time retype, if adopted** (D1 §2.1) — real code across every call site.
4. **The signature-pin test** (§3 above).
5. **The versioning policy** (§2 above), written down.
6. **Rewrite the F6 register row** to enumerate the actual surface; flip it to LOCKED with the date.
7. **The documentation sweep** — remove the not-frozen notices from guide ch. 09, ch. 10, ch. 14 and
   `chassis.hpp`'s header banner. `guide-maintenance.md`'s D2 row names this sweep; the guide README's
   maintenance list names all three chapters. **All in one commit** — a half-updated freeze claim is
   worse than a stale one.

### Out

- **The recipe cookbook and generated API docs** → D3 (which waits on D2 settling §2.1's spelling).
- **`.vexbot` schema versioning** → F7/F8, Phase G.
- **Mechanisms** → F1/F3. `then()`'s seam shape is D1's; D2 only decides whether it is frozen.
- **New capability of any kind.** If D2 finds itself adding a feature, it has lost the plot — except
  the one candidate addition D1 explicitly recommends considering (`wait`), which is a decision, not
  a discovery.

### Explicitly rejected

- **Freezing by simply flipping the badge.** The row must be accurate and enforced first.
- **Deferring the time decision.** §2.1 is now-or-never by construction; "decide later" *is* deciding
  to keep raw doubles forever, and it should be recorded as such if that is the choice.

---

## The decision docket

Every item gets an explicit ruling in `D2-COMPLETED.md`. No item may be left implicit.

### A — What, exactly, is frozen

| # | Question | Input |
|---|---|---|
| A1 | Does the F6 row enumerate the full surface, or name exclusions? | §1 above |
| A2 | **Does `Routine` join F6?** | D1 §12 — D1 recommends **no** (stays unfrozen through D3 so the cookbook can still shape it). D2 must say so in the row, either way: `Routine` lives in `chassis/`, so silence reads as "frozen". |
| A3 | Are `ChassisConfig` / `MotionOptions` / `TrajectoryResult` frozen? | They appear in frozen signatures; `MotionOptions` is designed additive (C4 §5 D10) |
| A4 | Is the Tier-3 seam (`deps()`/`scheduler()`) inside F6? | C4 §8 argues yes — freezing the facade without it freezes users *out* of the lower layers |

### B — The now-or-never changes

| # | Decision | D1's recommendation |
|---|---|---|
| B1 | **Retype time as `units::Time`** (`MotionOptions.timeoutSeconds`, `hold`, `waitUntil`, and `Routine::pause`) | **Change at D2 or never** (§2.1). `hold(500)` from someone thinking in milliseconds compiles and holds pose for 500 s inside a 15 s match. `units::Time` and `_s`/`_ms` literals already exist and are verified present. Every call site is in-tree today. |
| B2 | **Add `wait(seconds)` as an additive facade verb?** | D1 **leans add** (§2.2): the naive spelling Warn-spams, `hold` energizes the drive, and the working spelling needs Tier-3 plumbing plus a sacrificial timeout. If declined, bless the predicate idiom in F6's docs. |
| B3 | **Adopt `brake` / `hold` into F6** | **Adopt** (§2.6) — every complete D1 routine used both; without them, parking needs the Tier-3 seam, a Tier-2 cliff §17 forbids. |

### C — The accepts, to be confirmed deliberately rather than inherited

| # | Item | D1's recommendation |
|---|---|---|
| C1 | Runtime `abortFaultMask` mutation (C2 §11 flag 6, owner D1) | **Close it** — construction-time only (§2.3) |
| C2 | Branch-on-strafe-fallback (C3 §11 flag 1, owner D1) | **Keep the rejection** (§2.4) — a `waitUntil` predicate cannot see it either, so it is unreachable at every tier, not merely awkward |
| C3 | Tank field-vocabulary sugar (`face`/`driveTo`) | **Accept in the recipe layer; do NOT add to F6** (§2.5) |
| C4 | Three result vocabularies (`ExitReason` / `TrajectoryResult` / `WaitResult`) | **Accept for F6; flag for F2** (§2.7) |
| C5 | C4 §8's four tensions (async, `hold` disturbance radius, per-leg results, per-verb settle tolerance) | All four answered **no change needed** (§2.8) |
| C6 | `maxWheelSpeed` additive path (C3 §11 flag 4) | C4 named it "the freeze review checks the additive path exists" — **check it, structurally** |

---

## Load-bearing constraints

### 1. Under-claim in the register row
A LOCKED badge is read by people deciding what they can rely on. If any part of the surface is
frozen only provisionally, the row says so in words. Prefer a narrower honest freeze to a broad
aspirational one — F6 can be *extended* additively later; it cannot be *narrowed*.

### 2. Additive-only is the whole point of freezing well
Before locking each shape, ask: *what is the additive path if we need more later?* `MotionOptions`
has one by construction (new fields). `followTrajectory` has one (a richer `Trajectory` overload at
G2). If a shape has **no** additive path, that is the strongest possible argument for changing it now.

### 3. A retype is a behaviour-preserving change, and must be proven so
If B1 is adopted: the suite must be **green with identical numbers**, and the accuracy tables must
match D1's to the printed digit. A units change that moves a number is a bug, not a retype. Pin it
the way C3 pinned the pseudo-inverse: bit-identity against the pre-change build.

### 4. `1000` is not a millisecond
If B1 is adopted, the conversion boundary is where the bug lives. `_ms` divides by 1000; a
sign/scale slip there is exactly the class the units system exists to prevent, and it would be
invisible to any test that uses the same literal on both sides. **Pin at least one duration against
a hand-computed absolute** (a 2.5 s timeout must fire at 2.5 s of *simulated clock*, asserted against
the clock, not against another `_s` literal).

### 5. The sweep is atomic
Removing three of four not-frozen notices leaves the library saying two contradictory things. Grep
for every instance and update them together, in the same commit.

### 6. Standing contracts
A1's cost contract; injected clock; PROS-free; strict `-Werror`; both CI guards and the ARM gate;
any invented constant gets an `HA-nn` entry.

---

## Test requirements

Every test names, in a comment, the bug it would catch.

- **The signature pin** — a compile-time assertion of every frozen member's exact type. It must fail
  the build, with a message naming F6, if any signature changes. Prove it: change one signature,
  observe the build break, restore.
- **If B1 lands (time retype):**
  - the suite is green with **identical assertion behaviour** and the accuracy tables unchanged to
    the printed digit (bit-identity against the pre-retype build, C3's method);
  - **misuse is now rejected** — `hold(500)` as a bare number must **not compile**; add the
    compile-rejection pin next to D1's existing `static_assert` concepts;
  - a duration is pinned against an absolute simulated-clock measurement, not another literal
    (constraint 4).
- **If B2 lands (`wait`):** it commands nothing, advances the world, raises no fault, logs no Warn,
  and is watchdog-bounded like every other verb.
- **The documentation sweep is verified mechanically** — grep proves zero "not frozen" instances
  remain across `chassis.hpp`, ch. 09, ch. 10, ch. 14, and the guide README.
- **Guide examples still compile and are still quoted verbatim.** If B1 lands, every guide listing
  containing a timeout changes; the chapters must be re-quoted, not hand-edited. Run the verbatim
  check (compare each ```cpp line in the chapters against `test/guide_examples_test.cpp`).

### Mutations

A decision chunk still has load-bearing logic — the pin, and the retype if it lands. Required:

- Break each frozen signature in turn; the pin must go red. **A pin that does not catch its own
  member is decoration.**
- If B1 lands: mutate the `_ms` conversion factor; mutate one call site's units; observe red.
- **Any mutation that stays GREEN is a hole — log it, close it with a test that goes red alone, and
  give it a prominent place in the record.** Every chunk so far has found one, including D1 (two).
- Gate the runner on build success (C4 §4.6's lesson; D1 tripped it twice).

---

## Definition of Done

- [ ] All nine D1 §2 items ruled on, each ADOPT / REJECT / DEFER-with-owner + reasoning
- [ ] All 18 rows of C4 §2's ledger re-checked and confirmed before locking
- [ ] The F6 register row **rewritten to enumerate the actual surface**, exclusions named
- [ ] **`Routine`'s freeze status stated explicitly** in the row (A2)
- [ ] The versioning policy written down; what a breaking change is, and where the version lives
- [ ] The signature-pin test lands and is proven to catch a real signature change
- [ ] B1 (time) decided; if adopted, retyped everywhere with behaviour proven identical
- [ ] B2 (`wait`) decided; if declined, the predicate idiom is blessed in the docs
- [ ] The not-frozen sweep is complete and grep-verified across all four locations
- [ ] Guide examples recompiled and re-quoted verbatim
- [ ] **F6 flipped to LOCKED with the date** — and the badge is *true*
- [ ] Suite green; both guards pass; ARM gate passes

---

## Live progress log — required

`docs/internal/chunks/D2-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`D2-COMPLETED.md`** at the depth of C1–C5 / D1 (570–654 lines). The **decision docket
with its rulings is the centrepiece** — this record is what someone reads in two years to find out
why the API is shaped the way it is. Record the rejected alternative for every ruling, including the
ones where D1's recommendation was simply accepted: *"accepted D1's recommendation, having checked X"*
is a decision; silence is not.

**Do not commit. Do not push.**

---

## Landmines

- **Don't flip the badge before the row is accurate.** A LOCKED row that under-describes the surface
  is worse than no freeze, because it looks settled.
- **Don't skip the ledger.** C4 wrote "anything missed here becomes permanent at D2." That sentence
  was aimed at this chunk.
- **Don't let "decide later" masquerade as a decision** on B1. Deferring the retype freezes raw
  doubles forever; if that is the call, write it down as the call.
- **Don't freeze `Routine` by accident.** It lives in `chassis/`; silence reads as inclusion.
- **Don't build features.** The only new code is the pin, the version constant, and — if adopted —
  the retype and `wait`.
- **Don't half-sweep the notices.**
- **Don't trust a green suite after a retype.** Identical *numbers* are the bar, not merely green.
