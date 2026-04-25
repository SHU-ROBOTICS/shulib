# CLAUDE.md

> Entry point for Claude Code on this project. This file should live at
> the project root. Claude Code auto-loads `CLAUDE.md` on session start.

---

## What This Project Is (one paragraph)

A 24-hour rush implementation of a VEX U Programming Skills autonomous routine
for the 2025-2026 Push Back game. Replacing an existing time-based, sensorless
auton (which barely scores) with a sensor-driven routine using a V5 Inertial
Sensor for heading and a V5 AI Vision Sensor (276-8659) for closed-loop target
acquisition. Stack is PROS + LemLib 0.5.x in C++. Competition is imminent.
**Code under time pressure — make it work, not make it elegant.**

---

## How To Use This Doc Set

The full project context lives in numbered docs under the project root.
Read them in this order on first session start:

1. `00_PROJECT_CONTEXT.md` — what we're building, hardware, software stack, constraints
2. `01_STRATEGY.md` — the phased plan (Phases / Steps / Substeps), risk register, fallback ladder
3. `04_DECISIONS.md` — twelve locked decisions with rationale (do not re-litigate without asking)
4. `02_HARDWARE_SETUP.md` — IMU + AI Vision mounting, cabling, AI Vision Utility configuration
5. `03_MODULES.md` — module-by-module documentation including the existing AI Vision wrapper API
6. `05_STATUS.md` — current state of work, what's blocked, what's next

On every subsequent session, read `05_STATUS.md` first — it is the only doc
that changes frequently and reflects current reality.

---

## Working Preferences (CRITICAL)

These come directly from the user. Violating them creates rework and erodes
trust faster than anything else:

1. **Never write code without full context.** Ask questions if any detail is
   ambiguous — the devil is in the details.
2. **One file at a time** when reasonable. For tightly-coupled header+impl
   pairs, treat the pair as one unit.
3. **Full path comment at the top of every code file.** Example:
   `// include/primitives.hpp`.
4. **Disagree with the user when they're going down a wrong path.** Honest
   engineering input over agreement. Several pushbacks have already shaped
   this project (Pi rejection, "don't tune PID the night before comp",
   "don't replace working code until new code is validated").
5. **Keep the user in the loop on decisions.** When a design choice has
   multiple reasonable options, surface it rather than picking silently.
6. **No emojis.** Anywhere.
7. **Quality code principles:** functionality, readability, documentation,
   standards compliance, reusability, maintainability, robustness,
   testability, efficiency, scalability, security.
8. **OOP where appropriate**, not by default.
9. **In planning docs:** little to no actual code. **In implementation:**
   code is the deliverable.
10. **Avoid pre-locking file structure** for unwritten modules. Refer to
    upcoming modules by purpose, not by filename, until they're being
    created.
11. **No ZIP files.** Share files individually.
12. **Phases / Steps / Substeps** for any actionable plan.
13. **If session feels like it lost nuance** (after compaction or context
    drop), ask the user to re-share what's needed before writing code.

---

## Hard "Do Nots"

These have been explicitly rejected and locked in `04_DECISIONS.md`.
Do not propose them without strong new reasoning:

- Don't propose using the Raspberry Pi as a coprocessor (D-001).
- Don't propose full LemLib odometry / `moveToPoint` / pure pursuit (D-002).
- Don't propose deep PID tuning marathons before the competition.
- Don't delete the existing time-based auton until the new system is
  field-validated (D-004).
- Don't propose custom-trained AI vision models — use the pre-trained
  V5RC Push Back model from the AI Vision Utility.
- Don't propose AprilTag-based localization.
- Don't connect the IQ AI Vision Sensor (228-9136) — wrong product.

---

## Architectural Principles (NON-NEGOTIABLE)

Apply to every line of code:

1. **Every vision-guided action has a hard timeout.** No infinite loops.
2. **The routine is a sequence of named, independently-runnable segments.**
3. **Brain screen is a debug log.** Print segment name and key sensor values.
4. **Open-loop where it must be, closed-loop where it can be.** Mechanisms
   stay open-loop time-based. Driving and turning go closed-loop.
5. **No new abstractions today.** This is throwaway code. Do not refactor.
6. **Calibration constants live in one place** (the config module).
7. **Compile early and often.** Don't write 200 lines before first build.
8. **Log timing.** Print elapsed time at segment boundaries.

---

## Quick Project Snapshot

- **Game:** Push Back, VEX U
- **Routine:** Programming Skills, 60 seconds
- **Hardware:** V5 brain, smart motors, AI Vision Sensor (276-8659), IMU (being acquired)
- **Software:** PROS, LemLib 0.5.x, C++
- **Done:** AI Vision wrapper module (`include/ai_vision.hpp`, `src/ai_vision.cpp`)
- **Pending user input:** existing chassis config, IMU port assignment, AI Vision class IDs, scoring strategy details
- **Outstanding work:** config module, IMU integration into chassis, drive primitives, vision behaviors, skills routine, field calibration

For details on any of the above, see the numbered docs.
