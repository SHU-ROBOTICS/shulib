# Resuming work on shulib v2

> **Read this first in any new session.** It captures the working protocol, which is not derivable
> from the code or the git history. The *plan* lives in [`build-order.md`](build-order.md); this is
> *how the plan gets executed*.

---

## The three planning documents

| File | Answers |
|---|---|
| [`shulib-v2-master-plan.md`](shulib-v2-master-plan.md) | **Why** — architecture, locked decisions, conventions, the capability catalog |
| [`roadmap.md`](roadmap.md) | **What** — every remaining task by milestone + the Freeze Register. Its "you are here" is the status pointer |
| [`build-order.md`](build-order.md) | **In what order, and why that order** — 39 dependency-ordered chunks |

`build-order.md` is the working document. Start there.

---

## Where things stand

Check these three, in this order — they are authoritative over anything written here:

```sh
git log --oneline -8                       # what has landed
cat docs/planning/build-order.md           # "Current position" section
ls docs/planning/chunks/                   # *-COMPLETED.md = done; *-PROGRESS.md = was in flight
git status --short                         # uncommitted work = a chunk was interrupted
```

**If `git status` is dirty, a chunk was interrupted mid-flight.** Do not start a new chunk. Read that
chunk's `-PROGRESS.md` (it is appended in real time, so it is an honest record of exactly how far the
work got), then either finish it or verify and commit what is there.

---

## The chunk loop

One chunk at a time, in `build-order.md` order. Each pass:

### 1. Write the brief
`docs/planning/chunks/<CHUNK>-<slug>.md`. Use [`A1-debugrecord-termsink.md`](chunks/A1-debugrecord-termsink.md)
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
find include/shulib -name '*.hpp' | sed 's|.*/include/||' | sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo "int main(){return 0;}" >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
```

Also confirm: nothing was committed, the DoD items are actually met, the roadmap checkboxes
under-claim honestly (`[~]` for partial), and spot-check the chunk's single most load-bearing
constraint in the source.

### 4. Commit
Only after verifying. Conventional-commit style matching the existing log; the body explains the
*reasoning* and names honest partials. Trailer:

```
Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
```

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

## Context on decisions already made

- **There is no robot yet**, and won't be for a while. That constraint drives the whole order: all
  hardware work is consolidated into Phase R, and A2's plant exists because it is the only way to
  validate closed-loop behavior without one.
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
