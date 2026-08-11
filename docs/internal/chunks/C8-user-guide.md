# Chunk C8 — the manual

> **Phase C, chunk 8 (added).** Predecessor: C7 ✅ — the cutover is done and the repo has a README.
> C8 writes the thing a new person actually reads.

**Workstream:** WS12 (Docs & onboarding) · **Milestone:** M2/M7

---

## Who this is for, and why that changes everything

**The reader is a new team member who wants to write autonomous routines and is not a robotics
expert.** They may be a first-year student. They may have written some Python and never touched C++.
They are smart and motivated and they do not know what a "pose" is, why a robot "drifts", or what PID
stands for.

**Every choice in this manual serves that person.** If a sentence would stop them, it is wrong — not
because they aren't clever, but because unexplained jargon is a failure of the writing, not the reader.

The second audience is a prospective member being shown what the team builds. The manual should make
them want to join, by being clear rather than by being impressive.

---

## Tone — the hard requirement

Write it the way a patient senior member would explain it sitting next to someone.

- **Plain sentences.** No slogans, no marketing, no "credit where it's due" flourishes. If a line
  sounds like a tagline, delete it.
- **Define every term the first time it appears**, in ordinary words, before using it.
- **Use concrete examples with real numbers.** "The robot thinks it's at (24, 36) but it's actually
  at (25, 35)" beats "estimation error accumulates."
- **Explain *why*, not just *what*.** A reader who understands why field-relative and robot-relative
  differ will never mix them up. A reader who memorised a rule will.
- **Admit difficulty.** "This part is genuinely confusing at first, and here's the thing that makes it
  click" is more useful than pretending it's obvious.
- **Never oversell.** This library has never run on a robot. Say so where it matters.

Assume no prior knowledge of: VEX, robotics, control theory, C++ beyond basic syntax, or this codebase.

---

## Updateable by design — the structural requirement

A manual that rots is worse than none, because people trust it. Build against that:

1. **Many small files, not one giant one.** `docs/guide/`, numbered so order is obvious
   (`01-what-is-this.md`, `02-the-field-and-coordinates.md`, …). A person updating one topic edits
   one file.
2. **Every file opens with a short header:** what it covers, who should read it, and what it assumes
   you already read.
3. **Single source of truth — link, don't restate.** Test counts, the assumptions register, the
   roadmap status: link to them. Any fact copied into the guide is a fact that will go stale.
   Where a number genuinely helps the explanation, mark it as of a date.
4. **⭐ Every code example must compile and be tested.** Put them in a real test file
   (e.g. `test/guide_examples_test.cpp`) and reference them, or extract them so CI proves they build.
   **A code example that isn't compiled is a code example that is already wrong.** This is the single
   most valuable anti-rot measure available, and it's exactly the discipline the rest of this repo runs on.
5. **A maintenance note** at `docs/guide/README.md`: how the guide is organised, where examples live,
   and what to update when the API changes.
6. **Leave room for what's coming.** The recipe API (D1) will add an easier tier. Structure so it
   *adds a file* rather than forcing a rewrite.

---

## Scope — the manual

Roughly this shape. Adjust if something reads better, but cover all of it.

### Part 1 — Orientation
What this library is. What VEX U is. What actually happens in a match and in a skills run. **What the
autonomous problem really is** — a robot alone on a field for 60 seconds with no one steering, needing
to know where it is and get where it's going. Why that's hard.

### Part 2 — Concepts (no code)
The heart of the manual. Plain-English explanations, each earning its place:

- Coordinates and the field — where (0,0) is, which way is +X, why one fixed convention matters
- **Pose** — position plus heading, and why heading is the part that ruins everything when wrong
- **Field-relative vs robot-relative** — the single most common source of confusion; spend real time here
- **Odometry** — how a robot tracks itself by counting wheel rotation, and **why it drifts**
- What each sensor knows and doesn't: tracking wheels, IMU, GPS, distance, optical, camera
- **Holonomic vs tank** — what strafing means, why an X-drive can do things a tank can't, and why the
  H-drive sits in between
- **Kinematics** — turning "go forward while turning left" into four wheel speeds
- **Control loops / PID** — explained without math anxiety, with a real intuition for each term
- **Feedforward** — why you predict *before* you correct
- **Settling** — how a robot knows it has arrived, and why "close enough" needs three conditions
- **How things fail** — drift, wheel slip, brownout, a dead sensor, a jammed mechanism

### Part 3 — Getting set up
Prerequisites. Clone, build, run the tests. **What the test output means.** The project layout and why
it's split that way. What you need before you can put code on a real robot (and what still doesn't exist).

### Part 4 — Your first autonomous routine
A full tutorial: empty file → working routine, **every line explained**, building up in small steps.
Run it against the simulator. Read the output. Then change something, and see what changes.

### Part 5 — The API, written as prose
Not a signature dump. For each `Chassis` verb: what it does, when to use it, what it needs, what it
does when things go wrong, and the gotchas. Cover typed units and why they exist, the motion
primitives, the scheduler, and cancellation.

> **Note:** F6 is not frozen (it freezes at D2). Say so, and keep this part easy to revise.

### Part 6 — Reading the diagnostics
Nobody outside the team knows how to read this output, and it is one of the library's best features.
Line by line: the session header, per-tick lines, per-motion result lines, the run summary. **Every
fault code: what it means, what causes it, what to do about it.**

### Part 7 — When things go wrong
Symptom-first troubleshooting: "my robot doesn't go where I told it", "it stops early", "it
oscillates", "the pose looks wrong", "it won't build". For each: what's likely, how to confirm it
from the diagnostics, and what to change.

### Part 8 — Extending it
For someone ready to go deeper: how the layers fit, the testing discipline and *why* it's strict, how
to add a drivetrain, where to add a motion.

### Part 9 — What it can't do yet
Honest and specific. Never run on a robot. No vision correctors yet. No no-code authoring yet. Link
the assumptions register and the roadmap rather than restating them.

### Glossary
Every term used anywhere in the guide, defined in one ordinary sentence.

---

## Definition of Done

- [ ] `docs/guide/` exists: numbered files, each with a purpose header, plus a maintenance README
- [ ] All nine parts and the glossary are written
- [ ] **Every code example compiles**, verified by the test suite or CI; the guide says where they live
- [ ] No unexplained jargon — every term defined at first use; cross-checked against the glossary
- [ ] No fact duplicated from the roadmap/register/README — linked instead
- [ ] The tutorial in Part 4 was **actually followed start to finish** and works as written
- [ ] The README links to the guide; the guide links back
- [ ] `docs/internal/` remains cleanly removable (C7's property preserved — the guide is public docs)
- [ ] Suite green; both guards pass; ARM gate passes

---

## Live progress log — required

`docs/internal/chunks/C8-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`C8-COMPLETED.md`**. Record the file map, where examples live and how they're
verified, and anything deliberately deferred to D3 (the recipe cookbook, once D1 exists).

**Do not commit. Do not push.**

---

## Landmines

- **Don't write to impress.** Clear beats clever. If a sentence sounds like a tagline, cut it.
- **Don't skip the "why".** Rules without reasons get misapplied the first time reality differs.
- **Don't ship an uncompiled example.** It's already wrong.
- **Don't restate facts that live elsewhere.** Link them, or they go stale.
- **Don't assume knowledge.** The reader may not know what a motor port is.
- **Don't oversell.** Never run on a robot. Say it plainly wherever it matters.
