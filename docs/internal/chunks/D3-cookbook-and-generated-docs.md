# Chunk D3 — the recipe cookbook + generated API reference

> **Phase D, chunk 3 of 3 — closes Phase D.** Predecessor: D2 (F6 locked).
> **D3 is `Routine`'s second consumer, and it freezes `Routine` — the same pattern that just
> worked for the facade.**

**Workstream:** WS12 (docs & onboarding) · **Milestone:** M7 · **Freezes:** `Routine` (see §Docket A)

---

## Why this chunk exists, and why it is *here*

Three reasons, and the second is the one the build-order entry misses.

**1. The documentation debt is real and dated.** The roadmap has carried "recipe cookbook" and
"generated API docs published to the team website" since M0. C8 built the manual; D1 added chapter 09.
Neither is a cookbook — a manual teaches concepts in order, a cookbook answers "how do I do the thing
I am trying to do right now" out of order. Those are different documents for different moments.

**2. `Routine` is still unfrozen, deliberately, waiting for this chunk.** D2's F6 row says it out
loud: *"unfrozen until D3's cookbook (its second consumer)."* The C4→D1→D2 sequence worked because a
contract got a critical outside consumer before it froze — and it paid, twice (D1's nine-item critique;
D2's typed-time catch). **D3 is that consumer for `Routine`.** Writing twenty real recipes against the
chain is exactly the exercise that exposes awkward step spellings while changing them is still free.
**Finding awkwardness is success.** A cookbook that reports "the recipe API was perfect" has not
pushed hard enough.

**3. The staleness problem needs an actual mechanism, not a promise.** The guide's verbatim-quoting
rule works and has been proven twice (C8 built it; D2's retype drifted 21 lines and the scan caught
every one). But it only covers *examples*. Nothing today notices when a new public member ships with
no documentation at all. D3 fixes that (§Three artifacts, #3).

---

## What already exists

| Thing | Where |
|---|---|
| The frozen facade — 20 members, 3 types, F6 LOCKED, signature-pinned | `include/shulib/chassis/chassis.hpp` |
| `Routine` — 12 steps, **unfrozen, awaiting this chunk** | `include/shulib/chassis/routine.hpp` |
| The 15-chapter manual; ch. 09 recipes, ch. 10 API-as-prose | `docs/guide/` |
| The verbatim-quoting anti-rot rule + the D2 drift scan | `docs/internal/guide-maintenance.md`, `verify/verify-d2.sh` §9 |
| Compiled examples (`guide-08*`/`09*`/`10*`) | `test/guide_examples_test.cpp` |
| The version + breaking-change policy | `include/shulib/version.hpp` |
| The F6 signature pin — the pattern a doc-coverage gate should copy | `test/f6_signature_pin_test.cpp` |
| D1's facade critique — the model for the `Routine` critique D3 owes | `D1-COMPLETED.md` §2 |

**Read first:** `D2-COMPLETED.md` §2 (how a freeze docket reads) and its §2.D; `D1-COMPLETED.md` §2
(the critique format to imitate) and §6 (the twelve `Routine` decisions — each is a candidate
awkwardness); `routine.hpp` in full; `guide-maintenance.md`; `docs/guide/09-the-recipe-api.md`.

**Raw material check, already done:** `chassis.hpp` carries **120** `///` doc-comment lines and
`routine.hpp` **78**, plus large banner blocks explaining *why*. The headers are already the
reference; they have never been extracted.

---

## The three artifacts, and why it is three and not one

The instinct "generate it so it updates itself" is right, and it is worth being precise about what
generation can and cannot do — because two of these three things cannot be generated, and the third
is the one that makes the whole thing actually hold.

### 1. The API reference — GENERATED, and therefore cannot drift

Extracted from the headers. It **is** the source, reformatted. A new verb with a `///` comment
appears in the reference the next time it runs, with no human step. This is the piece the roadmap has
been calling "generated API docs" and it is straightforwardly right.

### 2. The cookbook — HAND-WRITTEN, with compiled examples quoted verbatim

*"How do I write a left-side auton that scores twice and parks"* is not derivable from signatures.
It is human knowledge about strategy and sequencing, and no generator will ever produce it. What it
**can** inherit is the proven anti-rot mechanism: every code block lives in a compiled test and is
quoted verbatim, so CI turns red when it rots. That mechanism already exists — reuse it, do not
invent a second one.

### 3. The coverage gate — the piece that makes "it updates itself" TRUE

Generation alone does **not** deliver what it promises. A generator faithfully extracts whatever
comments exist; a member added with no `///` comment is silently absent from the reference, and the
reference still looks complete. That is a worse failure than a stale document, because nothing looks
wrong.

**So D3 ships a doc-coverage test**: every public member of the F6 surface (and of `Routine`) must
have documentation, or the build fails naming the undocumented member. This is the same structural
move as D2's signature pin — the project's pattern is *make the guarantee mechanical, not
conventional* — and it is what actually makes the docs self-maintaining. Without it, "generated"
means "generated from whatever someone remembered to write."

---

## Scope

### In

1. **The generated API reference** — a tool + its output, covering the frozen F6 surface and `Routine`
2. **The doc-coverage gate** (§3 above), proven by mutation
3. **The cookbook** — hand-written recipes, examples compiled and quoted verbatim
4. **A written critique of `Routine`** — every awkwardness found writing real recipes, each with a
   recommendation, in the shape of D1-COMPLETED §2
5. **The `Routine` freeze decision** (§Docket A) and, if frozen, its register entry + signature pin

### Out

- **The "first auton in 10 minutes" flow** → G4. It starts in VexBuilder and cannot honestly be
  written until that exists. (The build-order entry already excludes it; it stays excluded.)
- **Rewriting the manual.** The cookbook is a new document. Chapter 10 stays (see Docket B).
- **Actual website hosting** — see the honesty note below.

### Explicitly rejected

- **A hand-maintained API reference.** It is the staleness this project exists to fight, and the
  headers already carry the content.
- **Generating the cookbook.** Strategy prose is not derivable from signatures; a generated cookbook
  would be a signature list wearing a hat.
- **A second anti-rot mechanism.** The verbatim rule works and is proven; extend it, do not compete
  with it.

### An honesty note on "publish to the team website"

The roadmap's DoD says *"docs generate and publish."* **There is no website and no publishing
infrastructure** — the repo has exactly one CI workflow (`ci.yml`) and no Pages configuration.
D3's honest scope is: **generate**, emit **web-portable** output, and **document the publish path**.
Standing up hosting is infrastructure, not documentation, and if D3 does not do it the roadmap
checkbox is `[~]` with the remaining half named. Do not claim published when nothing is published.

---

## The decision docket

Each gets an explicit ruling in `D3-COMPLETED.md`, with its rejected alternative.

### A — the `Routine` freeze (the item the build-order entry omits)

| # | Question |
|---|---|
| A1 | **Does `Routine` freeze at D3?** D2 deferred it here expressly. If yes, it freezes *after* the cookbook has consumed it and its critique is written — never before. If no, name the chunk that owns it and why waiting is better. |
| A2 | **If frozen, where does it live?** Amend the F6 row (it is the same `chassis/` surface, and F6 already names the exclusion), or open a **new register row**? A new row is cleaner if `Routine` might version independently of the facade; an amendment is cleaner if they always move together. Decide, with reasoning. |
| A3 | **Does the signature pin extend to `Routine`?** D2's pin is the enforcement that made F6 real. A freeze without one is a comment. |
| A4 | **What is excluded?** `then()`'s callable contract is a placeholder shape until F1/F3 build mechanisms. Freezing it may over-commit. Say what is in and what is out, explicitly — silence reads as frozen (D2's A2 lesson). |

### B — the documentation architecture

| # | Question |
|---|---|
| B1 | **Generator: Doxygen, or a small custom extractor?** Doxygen is standard and needs no maintenance, but it is a new toolchain dependency for a student team, and its house style fights this repo's plain-English voice. A custom extractor (~200 lines, `python3` — already used by the verify harnesses) matches the header comment style exactly and emits markdown in the guide's voice, but it is code we own. Decide on evidence, not taste. |
| B2 | **What is chapter 10's relationship to the generated reference?** Both document the API. Two documents that can disagree is exactly the rot this chunk exists to prevent. The likely shape is ch. 10 = *how to think about it* (prose, gotchas, worked idioms) and the generated reference = *exactly what exists* (complete, mechanical) — but decide it, state the rule in both places, and make sure neither restates the other. |
| B3 | **Where does the cookbook live** — `docs/guide/` as more chapters, or `docs/cookbook/` as its own document? A cookbook is read out of order and by someone mid-task; a manual is read in order. That argues for separate, but separate means a second place to keep true. |
| B4 | **Is generated output committed to the repo, or built on demand?** Committed output can go stale between runs (and shows up in every diff); build-on-demand means the reference does not exist until someone runs the tool. If committed, CI must verify it is up to date — a regeneration check that fails if the committed output differs from a fresh run. |

---

## Load-bearing constraints

### 1. The cookbook is a critical consumer, not a demo
Its job is to *stress* `Routine`. Write recipes for things a real VEX U auton does — a multi-goal
side routine, a bail-out on a failed grab, a partner-wait, a tank routine, a routine that mixes tiers.
Every place a recipe is awkward to write is a finding. **Report it; do not work around it silently.**

### 2. Do not invent capability to make a recipe read nicely
If a recipe needs something `Routine` or the facade lacks, that is a finding for the critique — and if
it is a *facade* gap, remember F6 is now **locked**: it changes only by major version bump plus
migration note (`version.hpp`). Adding to `Routine` is still free until A1 rules. **Never quietly
widen a frozen surface to make a doc example prettier.**

### 3. Coverage gate before freeze
Order matters: cookbook → critique → coverage gate → *then* the `Routine` freeze decision. Freezing
first would repeat exactly the mistake the C4→D1→D2 order was built to avoid.

### 4. One source of truth per fact
The guide-maintenance rule "link, don't restate" now has teeth: with a generated reference, any
signature or field list restated by hand is a fact that will go stale. Link into the reference.

### 5. Every guarantee still holds through the cookbook's examples
The examples are real compiled routines. They run against the plant like any other test — a cookbook
recipe that cannot actually settle is a lie with a code block around it.

### 6. Standing contracts
A1's cost contract; injected clock; PROS-free **library** (the generator is a *tool*, not library
code — but it must not become a build dependency of the library); strict `-Werror`; both CI guards;
the ARM gate; the removability property (**no public doc may link into `docs/internal/`**); any
invented constant gets an `HA-nn` entry.

---

## Test requirements

Every test names, in a comment, the bug it would catch.

- **The doc-coverage gate** — add a public member with no doc comment; the gate must fail **naming
  it**. Prove it, then restore. A gate that does not catch its own omission is decoration.
- **The generator is deterministic** — running it twice produces identical output (otherwise the
  B4 regeneration check is unusable).
- **The generator does not lie** — spot-pin that the reference's rendered signature for at least a
  few members matches the header exactly. A generator that silently drops a `const`, a default
  argument, or an overload is worse than no generator.
- **Every cookbook example compiles and runs**, quoted verbatim, and the full-guide drift scan
  (all `docs/` markdown, not just `docs/guide/`) reaches **zero**.
- **Recipes achieve what their prose claims** — assert the outcome, and where a duration matters,
  assert against the **simulated clock**, not a sibling literal. (D2's hole #2 was exactly this: a
  `300_s`-for-`300_ms` slip passed a test that checked outcomes but never the clock.)
- **If `Routine` freezes:** its signature pin, proven on several members, including a `noexcept`
  drop on a non-overloaded member — D2's hole #1 was that a cast which *adds* `noexcept` is accepted,
  so an exact-cast pin misses it. Do not re-open that hole.

### Mutations

- Break the coverage gate's own detection (remove a doc comment) — must go red, naming the member.
- Break the generator's signature rendering (drop a qualifier) — the fidelity pin must catch it.
- If `Routine` freezes: break each pinned member in turn.
- **Any mutation that stays GREEN is a hole — log it, close it with a test that fails alone, and
  give it a prominent place in the record.** Every chunk so far found one; D1 found two, D2 found two.
- Gate the runner on build success (D1 tripped this twice; C4 nearly misread a stale binary).

---

## Definition of Done

- [ ] Generated API reference exists, covers the F6 surface + `Routine`, and is deterministic
- [ ] **Doc-coverage gate lands and is proven** to fail on an undocumented public member
- [ ] Generator fidelity pinned (rendered signatures match the headers)
- [ ] Cookbook written; every example compiled, quoted verbatim, drift scan zero
- [ ] **A written critique of `Routine`** — every awkwardness, each with a recommendation
- [ ] **A1 ruled**: `Routine` frozen (with register entry + pin) or deferred with a named owner
- [ ] B1–B4 ruled with reasoning and rejected alternatives
- [ ] Publish path documented; checkbox `[~]` if nothing is actually published, with the rest named
- [ ] The "new reader can write a routine from the cookbook alone" clause addressed **honestly** —
      it needs a real human; if none was available, say so and name it as open (C8 left the same
      clause open with no owner; do not silently inherit that)
- [ ] Suite green; both guards pass; ARM gate passes; removability holds

---

## Live progress log — required

`docs/internal/chunks/D3-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`D3-COMPLETED.md`** at the depth of C1–C5 / D1 / D2 (570–654 lines). The
**`Routine` critique gets its own prominent section** — it is the input to A1's freeze ruling, and a
freeze lasts.

**Do not commit. Do not push.**

---

## Landmines

- **Don't freeze `Routine` before the cookbook has consumed it.** That is the entire reason it is
  still unfrozen.
- **Don't be polite about `Routine`.** Awkwardness you soften becomes permanent at A1.
- **Don't touch the frozen facade to make an example nicer.** F6 changes only by major bump plus
  migration note. If a doc need reveals a real facade gap, that is a *finding*, recorded for a
  future additive minor bump.
- **Don't let the generator and chapter 10 both own the same fact.**
- **Don't claim "published"** when there is no website.
- **Don't assume generation solves staleness.** Without the coverage gate, an undocumented member is
  invisible and the reference still looks complete.
- **Don't let a duration-bearing example assert against a sibling literal.** D2's hole #2.
