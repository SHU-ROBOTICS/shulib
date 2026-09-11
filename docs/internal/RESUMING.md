# Resuming work on shulib v2

> **Read this first in any new session.** It captures the working protocol, which is not derivable
> from the code or the git history. The *plan* lives in [`build-order.md`](build-order.md); this is
> *how the plan gets executed*.

---

## Where documents live (since the C7 reorganization, 2026-08-10)

`docs/planning/` no longer exists. The split, and the rule that keeps it meaningful:

- **`docs/internal/`** — development **process**: this file, `build-order.md`, `chunks/*`
  (briefs, `-PROGRESS` live logs, `-COMPLETED` records), `transcripts/`. This directory is a
  **self-contained, removable unit**: when `shulib-v2` squash-merges to `main`, `docs/internal/`
  is dropped and `main` shows only the library and its documentation.
- **`docs/`** — project **documentation**, public-facing: `roadmap.md`,
  `shulib-v2-master-plan.md`, `hardware-assumptions.md`, `diagnostics-plan.md`,
  `legacy-command-vocabulary.md`.

**The dependency rule (enforced at C7, keep it true):** nothing outside `docs/internal/` may
reference into it — not the README, not any `docs/*.md`. Links out of internal docs into public
ones are fine. Before closing any chunk that touches docs, re-run the C7 removability check
(grep public docs for `internal/`, `chunks/`, `RESUMING`, `build-order` — must be empty).

---

## The three planning documents

| File | Answers |
|---|---|
| [`shulib-v2-master-plan.md`](../shulib-v2-master-plan.md) | **Why** — architecture, locked decisions, conventions, the capability catalog |
| [`roadmap.md`](../roadmap.md) | **What** — every remaining task by milestone + the Freeze Register. Its "you are here" is the status pointer |
| [`RELEASING.md`](RELEASING.md) | **How a release happens** — `shulib-v2` → `release/v2` → `main`, the four invariants, and the three things that used to go wrong. Encoded in `tools/release.py`; read the doc before running it |
| [`build-order.md`](build-order.md) | **In what order, and why that order** — the dependency-ordered chunk list. *(It states its own count; this row used to carry one and it went stale.)* |

`build-order.md` is the working document. Start there.

---

## Where things stand

Check these three, in this order — they are authoritative over anything written here:

```sh
git log --oneline -8                       # what has landed
cat docs/internal/build-order.md           # "Current position" section
ls docs/internal/chunks/                   # *-COMPLETED.md = done; *-PROGRESS.md = was in flight
git status --short                         # uncommitted work = a chunk was interrupted
```

**If `git status` is dirty, a chunk was interrupted mid-flight.** Do not start a new chunk. Read that
chunk's `-PROGRESS.md` (it is appended in real time, so it is an honest record of exactly how far the
work got), then either finish it or verify and commit what is there.

---

## The chunk loop

One chunk at a time, in `build-order.md` order. Each pass:

### 1. Write the brief
`docs/internal/chunks/<CHUNK>-<slug>.md`. Use [`A1-debugrecord-termsink.md`](chunks/A1-debugrecord-termsink.md)
or [`A2-host-plant.md`](chunks/A2-host-plant.md) as the template. A good brief carries:

- Why this chunk is here in the order
- What already exists to build on (read the actual files — briefs get their value from specifics)
- Scope: **in**, **out** (with the chunk that owns it instead), and **explicitly rejected**
- The load-bearing design constraints, each with its reasoning
- Test requirements including the required mutation checks
- The DoD as a checklist
- The documentation contract + the live progress-log requirement
- Landmines

**The brief is where the thinking goes.** Read the code it touches before writing it — the highest-value
lines in both briefs so far came from noticing something specific in an existing header.

### 2. Commit the brief, then run it
Fable executes the chunk (`Agent` tool, `model: fable`, `subagent_type: general-purpose`,
`run_in_background: true` so the user can watch and the session stays responsive).

The prompt must include: required reading, the non-negotiable constraints restated inline (do not rely
on the brief alone for the critical ones), the test bar, the verification commands, the documentation
contract, **"do not commit"**, and **"create the PROGRESS log first and append as you go."**

### 3. Verify independently — never take the report at face value
This is the step that makes the process real. Re-run everything yourself:

```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6
```

Then the CI PROS-free guard (exact command in `.github/workflows/ci.yml`, scope grows per chunk), and
the ARM cross-compile of all v2 headers:

```sh
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo "int main(){return 0;}" >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
```

And the src/ BUILD gate (chunk GATE1, 2026-08-19) — the REAL `make`, compile AND link, every
build (three robot variants since R3b Session 2: `bench`, `xdrive`, `tank`; plus, since R3b
Part 0b, the second axis `ROBOT=tank PROGRAM=drive` — four builds), warning policy in the
tool's header. It deletes and rebuilds `bin/`/`.d/` (both gitignored), so it does not dirty
the tree:

```sh
python3 tools/src_build_gate.py self-test
python3 tools/src_build_gate.py check
```

> **A standing belief was RETIRED here (GATE1, measured 2026-08-19) — do not re-derive it.**
> The on-robot build was long recorded as blocked by a soft-float `firmware/`+`liblvgl.a`
> ABI mismatch and a `gnu++26` default. Both are gone: the `Makefile` pins
> `CXX_STANDARD:=gnu++20` ahead of `common.mk`'s `?=`, the firmware archives are vendored
> in-tree, and `make` COMPILES AND LINKS end to end (exit 0, `bin/hot.package.bin`) at
> apt's `arm-none-eabi-g++ 13.2.1` — locally and in CI. If a session doubts this,
> re-measure with `make`; do not resurrect the old blocker from memory or old notes.
> Variant selection is `make ROBOT=bench` (default) / `make ROBOT=xdrive` / `make ROBOT=tank`
> (the 2026 chassis, since R3b Session 2) — the old documented `CXXFLAGS_EXTRA` flag was a
> measured silent no-op (transposed name; GATE1-PROGRESS §3–4) and no longer appears anywhere.

> **This command was BROKEN here until DOCS1 (2026-08-14), and the failure was silent-ish in the
> worst way.** The `sed` read `s|.*/include/||`, which needs a `/` *before* `include` — so run from
> the repo root as every instruction says, it matched nothing, emitted
> `#include "include/shulib/…"`, and the compiler stopped at the first line with
> `fatal error: No such file or directory`. A session following the canonical protocol would meet a
> fatal error on the project's own verification step and have to guess whether the tree or the
> command was at fault. Fixed to the anchored form (`s|^include/||`), which is what
> `PROJECT-BRIEFING.md` has carried all along — the two documents had drifted, and the working one
> was not this one. `LC_ALL=C` pins the sort so the generated TU is reproducible across locales.

Also confirm: nothing was committed, the DoD items are actually met, the roadmap checkboxes
under-claim honestly (`[~]` for partial), and spot-check the chunk's single most load-bearing
constraint in the source.

### 4. Commit
Only after verifying. Conventional-commit style matching the existing log; the body explains the
*reasoning* and names honest partials. Trailer:

```
Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
```

*(The trailer names the model that coordinated the session. It read `Claude Opus 5 (1M context)`
from 2026-08-01 to 2026-09-10; commits since then carry the line above.)*

**Do not push** unless asked.

---

## Standards that are not negotiable

- **Evidence, not vibes.** A checkbox flips only with cited evidence (file + test + counts).
  **Under-claim before over-claiming** — `[~]` for partial, with the owning chunk named.
- **Tests must try to break the code.** Mutation checks are mandatory for load-bearing logic: break
  it, run it, *observe* red, restore. A mutation not actually run does not count.
- **Documentation is a deliverable, not an afterthought.** Extensive. Headers explain *why*. Every
  decision with a viable alternative gets logged with the alternative and the reason.
- **Clean-room: re-derive, don't port.** Legacy code is reference only. Re-deriving `arcStep` this way
  caught a real legacy bug.
- **A chunk that finds a flaw in an earlier chunk fixes it there**, not around it.

---

## Pace and documentation rules (jal, 2026-09-10)

> *"I want to work a lot slower so that we can more accurately update docs to stay up to date with
> changes being made. I want to actually make this library good."* These rules slow the chunk loop
> down on purpose. The build gates catch what they can see — an undocumented member, a stale generated
> page, a drifted example, a link into `docs/internal/`. Everything else (stale prose that still parses,
> a "you are here" that lags by a chunk, a completion record never written) only stays true if the
> pace leaves room to look. GATE1 landed 2026-08-19 without its completion record or a roadmap
> pointer move, and a `build-order.md` sentence contradicted the paragraph above it for three weeks.
> Speed caused both.

1. **One thing in flight.** One chunk, one executor, no parallel tracks. Verify, update the docs,
   commit — THEN start anything new. Never write a brief and launch it in the same turn.
2. **A brief waits for the team lead's go.** Write it, commit it, show its scope and its
   documentation impact list, and stop. The same for any change to the plan (this file's companions:
   `build-order.md`, `roadmap.md`, the Freeze Register): propose, wait.
3. **Docs move in the same commit as the code, or the commit waits.** No "docs follow-up" commits.
   The commit body lists the documents touched AND the ones deliberately not touched, with why.
4. **Prose review is a logged step, not a hope.** For every changed source file, grep `docs/` and
   `README.md` for its name and its concepts, read the hits, fix what is stale. Record what was
   checked in the PROGRESS log or the commit body. This is the part the gates cannot see.
5. **Status pointers are checked before every commit.** Re-read `roadmap.md`'s "You are here" and
   `build-order.md`'s `**Next:**` block (the briefing tool derives its pointer from the FIRST bold
   `Next:` in that file) and confirm both are true for the tree AFTER the commit. A chunk without its
   `-COMPLETED.md` and its pointer move is not closed.
6. **Report, then wait.** After each step, say what changed and which docs moved, and stop. Do not
   chain the next step into the same turn. "ASAP" means first in line — never skipped verification
   or skipped docs.
7. **Size chunks so the doc pass fits.** If a chunk's documentation impact runs past about five
   documents, split it into parts with their own verification and commit — R3b session 2's Part 0
   is the model.
8. **Session close-out.** Before a session ends: the tree is clean, or the PROGRESS log says exactly
   where work stopped; the four "Where things stand" commands above give a true answer.

## Context on decisions already made

- **This was written when there was no robot at all**, and that constraint drove the whole order:
  hardware work is consolidated into Phase R, and A2's plant exists because it was the only way to
  validate closed-loop behaviour without one. **Since 2026-08-13 a robot has been on the bench**
  (the team's old competition bot — tank, no GPS, no tracking wheels), so the order's premise has
  softened, but the constraint that still governs has not: **the library has never driven one.**
  A2's plant is still the only place closed-loop behaviour has ever been validated.
- **An earlier draft put the hardware bridge before the motion layer** and was reversed — that
  argument needed a robot to validate against. The reversal is recorded in `build-order.md`'s
  deviations table. Don't re-litigate it.
- **A2's truth integrator is deliberately independent of `arcStep`** — if it shared it, any `arcStep`
  error would cancel out and be invisible. This is the subtlest property in Phase A; preserve it.
- **The three additions to the roadmap** (A2 plant, A3 hostile fakes, A4 assumptions register) close a
  real incompleteness bug, not a preference.

## Guardrail

The **library** is built this way. The **competition routines** (Phase F′) and **authored paths**
(Phase G) are strategy that students must author and be able to defend — those chunks deliver
primitives and the engine, and stop short of authoring the season's content.

---

*Created 2026-08-01, during the session that produced `build-order.md` and chunks A1–A2.*
