# Chunk C7 — the cutover (delete `legacy/`, make the build work, give the repo a front door)

> **Phase C, chunk 7 of 7.** Predecessor: C6 ✅ — which returned an unconditional
> **"`legacy/` is safe to delete."** C7 acts on that sentence.
>
> **This is the only irreversible chunk in the project.**

**Workstream:** WS11 (Tooling/cutover) · **Milestone:** M2 (structural DoD)

---

## What this chunk is

Three things, in this order:

1. **Rewire `main.cpp`** onto the new core so the ARM build compiles again
2. **Delete `src/legacy/` and `include/legacy/`**, and broaden the CI PROS-free guard to all of `shulib/`
3. **Give the repo a real front door** — a README that describes the library as it actually is, plus a
   docs reorganization separating *development process* from *documentation*

`make` has failed since June, when the M0 quarantine moved headers out from under the legacy sources.
C7 is where that ends.

---

## ⚠️ Scope honesty — the line this chunk must not cross

**C7's DoD is "`make` succeeds and produces an uploadable package." It is NOT "it works on a robot."**

There is no robot, and the `hal/pros` adapters that would let the core talk to real motors are **R1**.
So `main.cpp` can wire up the v2 stack and compile, but it cannot drive anything yet.

**Say that plainly everywhere it appears** — in the header, the README, the roadmap, the completion
record. Claiming more would be the first dishonest checkbox in this project, and the whole discipline
here is that a checkbox means something.

Mark the HAL seams `TODO(R1)` explicitly so the gap is visible in the code, not just the docs.

---

## Order of operations — do not reorder

C6's audit is the authority. Deletion is one-way in practice (git keeps it; nobody looks).

1. **Re-read C6's verdict and its 34-file table.** If anything looks unclassified or wrong, stop and
   resolve it before deleting a single file.
2. **Rewire `main.cpp`** and get the ARM build compiling *with `legacy/` still present*, so a failure
   is attributable.
3. **Then delete** `src/legacy/` and `include/legacy/`.
4. **Then broaden the guard** and re-verify everything.

If the build only works *after* deletion, you have learned less than if it worked before.

---

## Scope

### In
- `src/main.cpp` — a v2 entry point wiring the real stack, with `TODO(R1)` HAL seams
- Delete `src/legacy/` + `include/legacy/`
- Broaden the CI PROS-free guard to **all** of `include/shulib/`
- Make `make` succeed and produce an uploadable package
- **`README.md`** — a genuine front door (see below)
- **Docs reorganization** — separate process artifacts from documentation

### Out
- `hal/pros` adapters → **R1** · anything requiring a robot → **R3**
- The in-depth user guide → **C8** (its own chunk, deliberately)
- Recipe API → D1 · F6 freeze → D2

---

## The README — write it for a human

The current README is from February and describes the old library. It is the first thing anyone sees.

**Write it in plain English.** Assume a reader who is a competent programmer but *not* a robotics
specialist, and who has never seen this codebase. Jargon is allowed only after it has been explained
once.

It should answer, in roughly this order:

1. **What is this?** — one honest paragraph. An autonomous stack for VEX U holonomic robots.
2. **What does it actually do today?** — motion, position tracking, diagnostics, three drivetrains.
3. **What is it *not* yet?** — **has never run on a physical robot**; no vision correctors; no no-code
   authoring. Say this clearly and early; it is the single most important fact about the current state.
4. **How do I build and test it?** — the exact commands, copy-pasteable.
5. **What does using it look like?** — a short, real code example that actually compiles.
6. **How is it laid out?** — what lives where and why the layering exists.
7. **Where do I go next?** — the roadmap, and (once C8 lands) the guide.

State the verification honestly and specifically: *659 test cases, ~915,000 assertions, three
drivetrains, simulated hardware faults — all off-robot.* Numbers with their scope beat adjectives.

---

## The docs reorganization

`docs/planning/` currently mixes two very different kinds of file:

| Development process — internal | Project documentation — keep visible |
|---|---|
| `chunks/*` (briefs, `-PROGRESS`, `-COMPLETED`) | `roadmap.md` — its own header says it is *"written to live on the team website"* |
| `RESUMING.md` — the working protocol | `shulib-v2-master-plan.md` — architecture and rationale |
| `build-order.md` — chunk sequencing | `hardware-assumptions.md` — **needed at Phase R; do not lose it** |
| | `diagnostics-plan.md`, `legacy-command-vocabulary.md` |

**Separate them structurally** — e.g. `docs/internal/` for process, `docs/` for documentation — so what
a reader sees is the library and its real docs, not development scaffolding.

**Do not delete anything.** The process record is genuinely valuable and `RESUMING.md` is how future
sessions pick up. Reorganize and label; don't destroy. Update every cross-reference — a reorganization
that leaves dangling links is worse than none.

---

## Design constraints

1. **C6's verdict is the authority for deletion.** Do not re-litigate it; do not exceed it.
2. **`make` must genuinely succeed** — not "compiles except for X." Report the actual artifacts produced.
3. **The guard broadens to all of `include/shulib/`** — and must still pass. `main.cpp` lives in `src/`
   and may include PROS; the core may not.
4. **Nothing may regress.** The full host suite, both guards, and the ARM gate stay green throughout.
5. **Every cross-reference survives the reorganization.** Check them mechanically.
6. **Say what is not true.** Every place the build's status is described, the "never run on hardware"
   caveat travels with it.

---

## Test requirements

- **The host suite is unchanged** — deletion must not touch it: 659 cases / 915,570 assertions
- **Both CI guards pass**, with the PROS guard now covering all of `include/shulib/`
- **The ARM gate passes** at the full header count
- **`make` succeeds** — capture and report the output and the artifacts
- **No dangling references** — no source, test, doc, or build file mentions `legacy/`; verified by grep
- **No broken doc links** after the reorganization; verified mechanically
- **The README's build/test commands are executed exactly as written** and shown to work — a README
  whose commands were never run is a README that is already wrong

---

## Definition of Done

- [ ] `main.cpp` wires the v2 stack with explicit `TODO(R1)` HAL seams
- [ ] `src/legacy/` and `include/legacy/` are **gone**
- [ ] `make` succeeds and produces an uploadable package; artifacts named
- [ ] CI PROS-free guard covers **all** of `include/shulib/` and passes
- [ ] Host suite, both guards, ARM gate all green; no regression
- [ ] Zero references to `legacy/` anywhere; grep-verified
- [ ] README rewritten in plain English, commands executed as written, "never run on hardware" stated plainly
- [ ] Docs reorganized: process separated from documentation, nothing deleted, all links checked
- [ ] **M2's structural clause closes** — the new tree is the only tree
- [ ] The on-robot clause is explicitly recorded as **still open**, owned by R3

---

## Live progress log — required

`docs/planning/chunks/C7-PROGRESS.md` (or its post-reorganization path — say which), appended as work
happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`C7-COMPLETED.md`**. Record the deletion inventory (what went, and the commit anyone
would `git show` to recover it), the reorganization map (old path → new path), and a clear statement
of exactly what `make` now produces and what it does *not* yet do.

**Do not commit. Do not push.** Leave everything in the working tree for review.

---

## Landmines

- **Deleting before the build works** loses your ability to attribute a failure. Order matters.
- **Don't overclaim the build.** Compiling is not running. R3 owns "it works."
- **Don't delete the process record** while reorganizing — relocate it.
- **Don't ship a README whose commands you didn't run.**
- **Don't leave dangling links.** Check mechanically, not by eye.
