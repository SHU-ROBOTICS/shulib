# 01 — Strategy

This document explains the architectural decisions and the phased plan for executing them. For the *why* behind specific decisions, also read `04_DECISIONS.md`.

## Strategy Summary

**AI Vision-first autonomous, IMU for turning, motor encoders for distance, no full odometry, no coprocessor.**

The routine is built around "see → align → approach" loops using the AI Vision Sensor as the primary perception system, with short open-loop segments between vision-guided behaviors. This sidesteps the multi-hour cost of tuning full odometry / pure pursuit / advanced PID, uses only hardware that can be acquired and integrated tonight, and is genuinely achievable in 24 hours.

## Why This And Not Something More Sophisticated

The fundamental problem with the current auton is **drift**. Time-based open-loop driving accumulates error every time the robot turns, every time the surface friction varies, every time the battery sags. By the third or fourth move, the robot is somewhere unpredictable.

This plan solves drift two ways, intentionally avoiding the deep end of the pool:

1. **IMU-based turns** replace time-based turns. A "turn 90 degrees" command becomes "turn until heading hits 90 degrees, then stop." This single change eliminates the largest source of drift.
2. **Vision-guided final approaches** replace blind driving toward targets. Instead of "drive forward 30 inches and hope a block is there," the robot drives until its camera says it's close enough to the target. This is closed-loop — small mechanical or positional errors get corrected by the camera.

The bet here: a simple, well-instrumented routine using two reliable feedback signals beats an ambitious routine with poorly-tuned everything. Under time pressure, this bet wins.

## Architectural Principles (Non-Negotiable)

These rules apply to every line of code we write:

1. **Every vision-guided action has a hard timeout.** No `while(true)` waiting for a detection. If vision sees nothing for N milliseconds, the action exits and the next segment runs. Camera misses must never freeze the routine.
2. **The routine is a sequence of named segments**, each independently runnable and skippable. If segment 3 fails in testing, we can comment it out and run segments 1, 2, 4, 5 cleanly.
3. **Brain screen is a debug log.** Print the current segment name and key sensor values. When something goes wrong on field, you need to see *where* it went wrong without a USB cable.
4. **Open-loop where it must be, closed-loop where it can be.** Mechanism timing (intake spin time, lift hold time) stays open-loop because mechanisms are reliable. Driving and turning go closed-loop.
5. **No new abstractions tonight.** If LemLib gives you a function that does what you need, use it. If it doesn't, write the most direct possible implementation. Refactor never.
6. **Calibration constants live in one place.** Drive scale factor, turn tolerance, vision width thresholds — all named constants in one config file. Tuning at the field means changing values, not hunting through logic.

## Phase 1: Hardware Setup

**Estimated time:** 2-3 hours
**Goal:** Robot has IMU mounted, AI Vision Sensor verified working, all ports documented.

### Step 1.1: Acquire IMU
- Substep 1.1.1: Confirm part is V5 Inertial Sensor. (The V5 part — not the older Cortex IMU.)
- Substep 1.1.2: Pick up the part. Bring a Smart Cable in case the bundled one is short.

### Step 1.2: Mount IMU
- Substep 1.2.1: Mount as flat as possible. Tilt introduces error in heading calculations.
- Substep 1.2.2: Mount close to the robot's center of rotation if practical. Less critical for heading, but helps if you ever extend to position tracking.
- Substep 1.2.3: Avoid mounting directly on or next to a smart motor. Magnetic fields can disturb readings.
- Substep 1.2.4: Use real screws. A loose IMU is a wandering IMU.
- Substep 1.2.5: Plug into a free Smart Port. Document which port number.

### Step 1.3: Mount and Aim AI Vision Sensor
- Substep 1.3.1: Camera height matters — position so the targets you actually want to detect are in frame at the distances where you'll be making vision-guided decisions.
- Substep 1.3.2: Forward-facing only. No pan/tilt — that's a project for another season.
- Substep 1.3.3: Stable mount. A camera that vibrates produces detections that jitter, which makes the centering loop oscillate.
- Substep 1.3.4: Plug into a Smart Port, document the port.

### Step 1.4: Document Port Map
- Substep 1.4.1: Write a port table somewhere visible. Drive motors, intake, mechanism motors, IMU, AI Vision, any other sensors.
- Substep 1.4.2: This goes in code as named constants too — but the physical sticky note saves you in the pit.

### Step 1.5: Configure AI Vision Sensor in the Utility
See `02_HARDWARE_SETUP.md` for detailed steps.

## Phase 2: Software Foundation

**Estimated time:** 2-4 hours
**Goal:** Reusable primitives that the auton routine will compose. Each primitive testable in isolation.

### Step 2.1: Configuration Module (config.hpp/cpp or single header)
- Substep 2.1.1: Centralize port assignments — IMU port, AI Vision port, drivetrain motor ports, mechanism motor ports.
- Substep 2.1.2: Centralize tuning constants — drive scale factor, turn kP, turn tolerance, vision approach width thresholds, default timeouts.
- Substep 2.1.3: Centralize AI Vision class IDs (the IDs you discovered when running the Push Back model in the AI Vision Utility).

### Step 2.2: Chassis Configuration with IMU
- Substep 2.2.1: Modify the existing LemLib chassis configuration to include the IMU on its known port.
- Substep 2.2.2: Verify drivetrain dimensions (wheel diameter, track width, gear ratio) are accurate. Measure with a ruler — do not eyeball.
- Substep 2.2.3: Decision: use LemLib's built-in chassis movement functions, OR roll simpler custom primitives. Recommendation: use LemLib's `turnToHeading` if the angular PID tunes acceptably with default gains; otherwise write a custom proportional-only turn function.

### Step 2.3: Drive Primitives
Each is a small function with a hard timeout. None should ever block forever.

- Substep 2.3.1: **Drive Distance** — drive forward or backward N inches at a given voltage, using motor encoder average to determine when to stop. Should include a heading correction term: read IMU heading, compare to starting heading, apply small differential to motors to keep straight. Has a timeout in milliseconds as hard exit.
- Substep 2.3.2: **Turn To Heading** — turn the robot until IMU heading matches a target value within a tolerance (e.g., 2 degrees). Simple proportional controller. Has tolerance and timeout. This replaces every time-based turn in the existing code.
- Substep 2.3.3: **Brake** — explicit "stop and hold position" call. Smart motors should be set to brake mode for this.

### Step 2.4: AI Vision Wrapper Functions
**ALREADY WRITTEN.** See `03_MODULES.md` and `include/ai_vision.hpp`.

### Step 2.5: Vision-Guided Behaviors
These compose the AI Vision wrapper into actions the auton routine calls.

- Substep 2.5.1: **Vision Center** — turn the robot until the largest detection of a target class is centered (within a tolerance) in the camera frame, OR until a timeout expires. Loop reads detection, computes x offset, applies P-controlled differential drive, checks exit conditions every iteration. Critical: timeout exit must leave the robot stopped.
- Substep 2.5.2: **Vision Approach** — drive forward until the bounding box width of the largest detection of a target class exceeds a threshold (proxy for proximity), OR until timeout, OR until vision loses sight of the object for too long. Width threshold is calibrated at the field.
- Substep 2.5.3: **Vision Acquire** — combination behavior: rotate slowly searching for a target class, when seen hand off to vision center then vision approach. Useful for "find me a block" segments.

### Step 2.6: Logging
- Substep 2.6.1: Implement a `log_segment(name)` function that prints current segment name to brain screen.
- Substep 2.6.2: Print key state (IMU heading, last detection class/size) on a status line that updates throughout the routine.

## Phase 3: Auton Routine Construction

**Estimated time:** 3-5 hours
**Goal:** The actual 60-second skills routine, written as a sequence of segment calls.

### Step 3.1: Translate Scoring Strategy to Segments
- Substep 3.1.1: Decompose the scoring strategy into discrete segments: "drive to first scoring zone," "intake first block," "rotate to face goal," "score," etc.
- Substep 3.1.2: For each segment, decide which primitive(s) it uses: open-loop time, drive distance, turn to heading, vision center, vision approach, mechanism action.
- Substep 3.1.3: Aim for 6-12 segments total. More than that is hard to debug. Fewer probably means individual segments are doing too much.

### Step 3.2: Implement Each Segment
- Substep 3.2.1: One function per segment. Name them after what they accomplish (`scoreFirstBlock`, `returnToStart`), not what they do mechanically (`turn90Right`).
- Substep 3.2.2: Every segment starts with `log_segment("name")` and ends with brake.
- Substep 3.2.3: Inside each segment, sequence primitives. Keep each segment to 5-15 lines of logic max.

### Step 3.3: Mechanism Sequencing
- Substep 3.3.1: Intake/scoring mechanism actions stay open-loop time-based. These mechanisms are reliable enough.
- Substep 3.3.2: Run mechanisms concurrently with drive when possible (e.g., intake spinning while approaching a block). PROS supports tasks/threads if needed; usually a non-blocking start-stop pattern suffices.

### Step 3.4: Compose the Full Routine
- Substep 3.4.1: Top-level `runSkills()` calls segments in order.
- Substep 3.4.2: Add elapsed-time logging at start and at each segment boundary so during testing you can see how long each segment takes.
- Substep 3.4.3: Implement a "panic abort" — if elapsed time exceeds 55 seconds, jump to a final "park safe" segment to avoid mid-action timeout violations.

### Step 3.5: Sanity Pass
- Substep 3.5.1: Read top to bottom. Every timeout sane? Every IMU heading reachable? No infinite loops?
- Substep 3.5.2: Compile. Fix warnings.
- Substep 3.5.3: Deploy to robot, run with wheels off the ground first to verify no immediate crashes.

## Phase 4: Field Testing

**Estimated time:** 1-2 hours (only real field window)
**Goal:** Calibrate constants, observe real-world failure modes, iterate.

### Step 4.1: Sensor Validation
- Substep 4.1.1: Drive the robot in a straight line by hand, check IMU heading stays within a few degrees. Significant drift = remount or replace.
- Substep 4.1.2: From a scoring position, point camera at target. Verify AI Vision detects expected class with stable bounding box dimensions.
- Substep 4.1.3: Move robot to several scoring positions. Identify distance/angle ranges where detection is reliable.

### Step 4.2: Primitive Calibration
- Substep 4.2.1: Run drive-distance for known distance (e.g., 24 inches), measure actual. Adjust scale factor. Repeat.
- Substep 4.2.2: Run turn-to-heading for several target headings (90, 180, 45, -90). Measure overshoot/undershoot. Adjust kP and tolerance.
- Substep 4.2.3: Run vision approach toward a target. Note bounding box width when robot is at desired stopping distance. This is your width threshold.

### Step 4.3: Full Routine Run
- Substep 4.3.1: Run full skills routine. Watch brain screen log to see which segment fails (if any).
- Substep 4.3.2: Note timing — how long does each segment actually take?
- Substep 4.3.3: Note score. Even one successful run is a baseline.

### Step 4.4: Targeted Iteration
- Substep 4.4.1: Identify lowest-hanging failure. Fix it.
- Substep 4.4.2: Re-run. Repeat until field time runs out or routine is stable.
- Substep 4.4.3: When field time ends, note in writing every observed issue you didn't get to fix. These are code-only-iteration targets.

## Phase 5: Code-Only Iteration

**Estimated time:** Whatever's left after field
**Goal:** Address issues that don't strictly require field validation.

### Step 5.1: Defensive Improvements
- Substep 5.1.1: Add fallbacks to vision-guided segments — if vision-approach times out, do a small open-loop forward push.
- Substep 5.1.2: Consider whether IMU re-zeroing at any point in the routine makes sense (usually no — accumulated heading is more useful than re-zero).

### Step 5.2: Polish
- Substep 5.2.1: Clean up brain screen logging so it's actually useful at competition.
- Substep 5.2.2: Comment the final routine in plain English.

### Step 5.3: Sleep
- Substep 5.3.1: A sleep-deprived programmer at competition makes mistakes that cost more than the marginal feature being tuned at 4 AM. Stop coding by some defined hour. Bring the laptop and a printout of the port map to comp.

## Risk Register

| Risk | Likelihood | Mitigation |
|------|-----------|-----------|
| IMU drift over 60s | Low | Recalibrate at routine start. V5 IMU is generally reliable. |
| Vision sees nothing (lighting, occlusion) | Medium | Hard timeouts on every vision action. Time-based fallback for critical scoring segments. |
| Mechanism mistiming | Medium | Keep mechanism timing values as named constants. Easy to tweak between runs. |
| Drive-distance scale drift over battery sag | Low-Medium | Use fresh-ish battery for skills. Don't tune with a 30% battery. |
| Compile errors / silly bugs eating time | High | Compile early, compile often. Don't write 200 lines before first build. |
| Bad calibration on field, no time to retest | Medium | Calibration constants in one file, easy to change between runs. |
| Robot disconnect / cable issue | Low | Pre-comp inspection: tug on every cable. |
| AI Vision Utility not yet configured | Medium | Confirm Push Back model selected and target classes detected before leaving for comp. |

## Fallback Ladder

If the full plan goes sideways, fall back in this order:

1. **Tier 1 — Full plan:** AI Vision-guided segments + IMU turns + drive-distance with heading correction.
2. **Tier 2 — IMU-only:** Skip vision entirely. Use only IMU-based turns and drive-distance. Still a major upgrade over the current time-based auton because turns become reliable.
3. **Tier 3 — Revert:** Restore the original time-based auton. Lose this attempt's points but don't break what works.

Don't be too proud to fall back. The sunk cost of hours coded does not justify a robot that doesn't move on the field.

## What This Plan Explicitly Does NOT Cover

- Tomorrow's match auton (different rules and constraints).
- Driver control improvements.
- Anything cosmetic.
- Refactoring the existing time-based code (treat it as deletable; build new alongside, switch when ready).
