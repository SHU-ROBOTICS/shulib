# VEX U Skills Auton — 24-Hour Rush Plan

## Context Snapshot

- **Game:** Push Back (2025-2026 V5RC / VURC)
- **Goal:** VEX U Programming Skills run, 60 seconds, fully autonomous
- **Stack:** PROS + LemLib on V5 Brain
- **Hardware on hand:** Robot fully built, AI Vision Sensor, V5 brain/motors/controller
- **Hardware acquiring tonight:** V5 Inertial Sensor (IMU)
- **Hardware explicitly NOT in scope:** Tracking wheels, Raspberry Pi, Jetson, secondary battery
- **Field access:** ~1-2 hours tonight, then competition day
- **Current state:** Time-based, sensorless auton that barely scores

---

## Strategy: Why This Plan Exists

The fundamental problem with the current auton is **drift**. Time-based open-loop driving accumulates error every time the robot turns, every time the surface friction varies, every time the battery sags. By the third or fourth move, the robot is somewhere unpredictable.

This plan solves drift two ways, intentionally avoiding the deep end of the pool:

1. **IMU-based turns** replace time-based turns. A "turn 90 degrees" command becomes "turn until heading hits 90 degrees, then stop." This single change eliminates the largest source of drift.
2. **Vision-guided final approaches** replace blind driving toward targets. Instead of "drive forward 30 inches and hope a block is there," the robot drives until its camera says it's close enough to the target. This is closed-loop — small mechanical or positional errors get corrected by the camera.

What we are explicitly **not** doing:
- Full LemLib odometry (no tracking wheels, no time to tune)
- Pure pursuit / motion profiling (overkill for a vision-guided routine)
- Custom-trained vision models (the Push Back pre-trained model exists, use it)
- Pi/Jetson coprocessor (massive setup cost, no benefit for this scope)
- PID tuning marathon (we use IMU-only feedback for turns and vision feedback for approach, which doesn't require tight PID tuning)

The bet here is: a simple, well-instrumented routine using two reliable feedback signals beats an ambitious routine with poorly-tuned everything. This bet is correct under time pressure.

---

## Architectural Principles (Non-Negotiable)

These rules apply to every line of code we write today:

1. **Every vision-guided action has a hard timeout.** No `while(true)` waiting for a detection. If vision sees nothing for N milliseconds, the action exits and the next segment runs. Camera misses must never freeze the routine.
2. **The routine is a sequence of named segments**, each independently runnable and skippable. If segment 3 fails in testing, we can comment it out and run segments 1, 2, 4, 5 cleanly.
3. **Brain screen is a debug log.** Print the current segment name and key sensor values. When something goes wrong on field, you need to see *where* it went wrong without a USB cable.
4. **Open-loop where it must be, closed-loop where it can be.** Mechanism timing (intake spin time, lift hold time) stays open-loop because mechanisms are reliable. Driving and turning go closed-loop.
5. **No new abstractions tonight.** If LemLib gives you a function that does what you need, use it. If it doesn't, write the most direct possible implementation. Refactor never, this is throwaway code.
6. **Calibration constants live in one place.** Drive scale factor, turn tolerance, vision width thresholds — all named constants at the top of one file. Tuning at the field means changing values, not hunting through logic.

---

## Phase 1: Hardware Setup

**Estimated time:** 2-3 hours
**Goal:** Robot has IMU mounted, AI Vision Sensor verified working, all ports documented.

### Step 1.1: Acquire IMU
- Substep 1.1.1: Confirm part is V5 Inertial Sensor (the V5 part, not the older Cortex IMU).
- Substep 1.1.2: Pick up the part. Bring a Smart Cable in case the bundled one is short.

### Step 1.2: Mount IMU
- Substep 1.2.1: Mount as flat as possible. Tilt introduces error in heading calculations.
- Substep 1.2.2: Mount close to the robot's center of rotation if practical. Less critical for heading, but helps if you ever extend to position tracking.
- Substep 1.2.3: Avoid mounting directly on or next to a smart motor. Magnetic fields can disturb the magnetometer (though the V5 IMU mostly relies on gyro/accel, this is good hygiene).
- Substep 1.2.4: Use real screws. A loose IMU is a wandering IMU.
- Substep 1.2.5: Plug into a free Smart Port. Write down which port number on a sticky note attached to the brain.

### Step 1.3: Mount and Aim AI Vision Sensor
- Substep 1.3.1: Camera height matters — position it so the targets you actually want to detect are in frame at the distances where you'll be making vision-guided decisions.
- Substep 1.3.2: Forward-facing only for now. No pan/tilt — that's a project for another season.
- Substep 1.3.3: Stable mount. A camera that vibrates produces detections that jitter, which makes the centering loop oscillate.
- Substep 1.3.4: Plug into a Smart Port, document the port.

### Step 1.4: Document Port Map
- Substep 1.4.1: Write a port table somewhere visible. Drive motors, intake, mechanism motors, IMU, AI Vision, any other sensors.
- Substep 1.4.2: This goes in code as named constants too — but the physical sticky note saves you in the pit.

### Step 1.5: Configure AI Vision Sensor in the Utility
- Substep 1.5.1: Connect the AI Vision Sensor to a computer running VEXcode and open the AI Vision Utility.
- Substep 1.5.2: Enable AI Classification mode in Detection settings.
- Substep 1.5.3: Select the **V5RC Push Back** classification model.
- Substep 1.5.4: In the live preview, verify the model detects the specific game objects your scoring strategy depends on. **This is a critical checkpoint.** If your strategy involves a target class the model doesn't recognize, you must pivot to color signature mode or change the strategy. Find this out tonight, not on the field.
- Substep 1.5.5: Note the class IDs / labels for the objects you care about. You will reference these by ID in code.
- Substep 1.5.6: Adjust exposure / brightness if your venue lighting is meaningfully different from your test environment. Bright gym lights vs. classroom fluorescents matter here.

---

## Phase 2: Software Foundation

**Estimated time:** 2-4 hours
**Goal:** Reusable primitives that the auton routine will compose. Each primitive testable in isolation.

### Step 2.1: Chassis Configuration with IMU
- Substep 2.1.1: Update LemLib chassis configuration to include the IMU on its known port.
- Substep 2.1.2: Set drivetrain dimensions (wheel diameter, track width, gear ratio) based on actual measurements of your robot. Measure with a ruler, do not eyeball.
- Substep 2.1.3: Decide whether to use LemLib's built-in chassis movement functions (e.g., `moveToPoint`) or roll simpler custom primitives. **Recommendation:** use LemLib for what it gives you cheaply (chassis abstraction, basic motor control), but write your own simple turn-to-heading and drive-distance functions if LemLib's require tuning you don't have time for.

### Step 2.2: Drive Primitives
Each of these is a small function with a hard timeout. None should ever block forever.

- Substep 2.2.1: **Drive Distance** — drive forward or backward N inches at a given voltage, using motor encoder average to determine when to stop. Includes a heading correction term: read IMU heading, compare to starting heading, apply small differential to motors to keep straight. Has a timeout in milliseconds as a hard exit.
- Substep 2.2.2: **Turn To Heading** — turn the robot until IMU heading matches a target value within a tolerance (e.g., 2 degrees). Uses a simple proportional controller: `voltage = kP * (target - current)`, clipped to a max. Has a tolerance and a timeout. This is the function that replaces every time-based turn in your current code.
- Substep 2.2.3: **Brake** — explicit "stop and hold position" call. Smart motors should be set to brake mode for this.

### Step 2.3: AI Vision Wrapper Functions
The raw AI Vision API is verbose. Wrap it.

- Substep 2.3.1: **Get Largest Detection of Class** — given a class ID, return the largest detection of that class currently visible (largest bounding box area), or null if none. Largest is a good proxy for closest.
- Substep 2.3.2: **Detection X Offset** — given a detection, return how far left/right of camera center it is, normalized to camera width. Output range roughly -1 to +1.
- Substep 2.3.3: **Detection Width** — given a detection, return its bounding box width in pixels. This is your distance proxy.

### Step 2.4: Vision-Guided Behaviors
These compose the wrappers above into actions the auton routine calls.

- Substep 2.4.1: **Vision Center** — turn the robot until the largest detection of a target class is centered (within a tolerance) in the camera frame, OR until a timeout expires. Loop reads detection, computes x offset, applies P-controlled differential drive, checks exit conditions every iteration. Critical: timeout exit must leave the robot stopped.
- Substep 2.4.2: **Vision Approach** — drive forward until the bounding box width of the largest detection of a target class exceeds a threshold (i.e., we're close enough), OR until timeout, OR until we lose sight of the object for too long. Width threshold is calibrated at the field.
- Substep 2.4.3: **Vision Acquire** — combination behavior: rotate slowly searching for a target class, when seen, hand off to vision center then vision approach. Useful for "find me a block" segments.

### Step 2.5: Logging
- Substep 2.5.1: Implement a `log_segment(name)` function that prints current segment name to brain screen (and optionally to controller screen).
- Substep 2.5.2: Print key state (IMU heading, last detection class/size) on a status line that updates throughout the routine.

---

## Phase 3: Auton Routine Construction

**Estimated time:** 3-5 hours
**Goal:** The actual 60-second skills routine, written as a sequence of segment calls.

### Step 3.1: Translate Scoring Strategy to Segments
- Substep 3.1.1: Take your existing scoring strategy and break it into discrete segments: "drive to first scoring zone," "intake first block," "rotate to face goal," "score," etc.
- Substep 3.1.2: For each segment, decide which primitive(s) it uses: open-loop time, drive distance, turn to heading, vision center, vision approach, mechanism action.
- Substep 3.1.3: Aim for 6-12 segments total. More than that is hard to debug. Fewer probably means individual segments are doing too much.

### Step 3.2: Implement Each Segment
- Substep 3.2.1: One function per segment. Name them after what they accomplish (`scoreFirstBlock`, `returnToStart`, etc.), not what they do mechanically (`turn90Right`).
- Substep 3.2.2: Every segment starts with `log_segment("name")` and ends with brake.
- Substep 3.2.3: Inside each segment, sequence the primitives. Keep each segment short — 5-15 lines of logic max.

### Step 3.3: Mechanism Sequencing
- Substep 3.3.1: Intake/scoring mechanism actions stay open-loop time-based. These mechanisms are reliable and don't need closed-loop control today.
- Substep 3.3.2: Run mechanisms concurrently with drive when possible (e.g., intake spinning while approaching a block). PROS supports tasks/threads if needed, but a simple non-blocking start-stop pattern usually suffices.

### Step 3.4: Compose the Full Routine
- Substep 3.4.1: Top-level `runSkills()` calls segments in order.
- Substep 3.4.2: Add elapsed-time logging at start and at each segment boundary so you can see during testing how long each segment takes.
- Substep 3.4.3: Implement a "panic abort" — if elapsed time exceeds 55 seconds, jump to a final "park safe" segment to avoid mid-action timeout violations.

### Step 3.5: Sanity Pass
- Substep 3.5.1: Read the routine top to bottom. Every timeout sane? Every IMU heading reachable? No infinite loops?
- Substep 3.5.2: Compile. Fix warnings.
- Substep 3.5.3: Deploy to robot, run with the wheels off the ground first to verify no immediate crashes.

---

## Phase 4: Field Testing

**Estimated time:** 1-2 hours (your only real field window)
**Goal:** Calibrate constants, observe real-world failure modes, iterate.

### Step 4.1: Sensor Validation
- Substep 4.1.1: Drive the robot in a straight line by hand, check IMU heading stays within a few degrees. Significant drift = remount or replace.
- Substep 4.1.2: From a scoring position, point camera at target. Verify AI Vision detects the expected class and reports stable bounding box dimensions.
- Substep 4.1.3: Move the robot to several scoring positions, repeat detection check. You're looking for distance/angle ranges where detection is reliable.

### Step 4.2: Primitive Calibration
- Substep 4.2.1: Run drive-distance for a known distance (e.g., 24 inches), measure actual distance traveled. Adjust scale factor. Repeat until close enough.
- Substep 4.2.2: Run turn-to-heading for several target headings (90, 180, 45, -90). Measure overshoot/undershoot. Adjust kP and tolerance until acceptable.
- Substep 4.2.3: Run vision approach toward a target. Note the bounding box width when the robot is at the desired stopping distance. This is your width threshold.

### Step 4.3: Full Routine Run
- Substep 4.3.1: Run the full skills routine. Watch the brain screen log to see which segment fails (if any).
- Substep 4.3.2: Note timing — how long does each segment actually take?
- Substep 4.3.3: Note score. Even one successful run gives you a baseline.

### Step 4.4: Targeted Iteration
- Substep 4.4.1: Identify the lowest-hanging failure. Fix it.
- Substep 4.4.2: Re-run. Repeat until field time runs out or routine is stable.
- Substep 4.4.3: When field time ends, note in writing every observed issue you didn't get to fix. These are your code-only-iteration targets.

---

## Phase 5: Code-Only Iteration

**Estimated time:** Whatever's left after field
**Goal:** Address issues that don't strictly require field validation.

### Step 5.1: Defensive Improvements
- Substep 5.1.1: Add fallbacks to vision-guided segments — if vision-approach times out, do a small open-loop forward push.
- Substep 5.1.2: Add IMU resync — if heading drifts unreasonably, recalibrate (with caveats; mid-routine recalibration is risky, evaluate per case).

### Step 5.2: Polish
- Substep 5.2.1: Clean up brain screen logging so it's actually useful at competition.
- Substep 5.2.2: Comment the final routine in plain English. Future-you will thank present-you.

### Step 5.3: Sleep
- Substep 5.3.1: This is a serious item. A sleep-deprived programmer at competition makes mistakes that cost more than the marginal feature they were tuning at 4 AM. Stop coding by some defined hour. Bring the laptop and a printout of the port map to comp.

---

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

---

## Decisions You Need to Make Tonight

1. **Routine segment ordering.** You said you have a clear scoring strategy — that maps directly here.
2. **Which AI Vision class IDs do which segments target.** Depends on what the Push Back model actually detects vs. what your strategy needs.
3. **Drive-distance vs. vision-approach for each segment.** Rule of thumb: drive-distance for "reposition to known-ish location," vision-approach for "engage with a specific game object." Some segments will mix both — drive-distance to get close, vision-approach to land precisely.
4. **What stays open-loop time-based, if anything.** Mechanism actions probably do. Drive actions probably don't.

---

## What This Plan Explicitly Doesn't Cover

- **Tomorrow's match auton.** This plan is skills-only. Match auton has alliance partners, opposing robots, and different scoring constraints.
- **Driver control improvements.** Out of scope.
- **Anything cosmetic.** Skills score is what matters.
- **Refactoring the existing time-based code.** Treat the existing auton as deletable. Build the new one alongside, switch over when ready.

---

## When This Plan Starts to Smell Bad

If by hour 12 you haven't finished Phase 2 (software foundation), you're behind schedule. Options at that point:

1. Cut vision-approach entirely, use IMU + drive-distance only. Still a major upgrade over time-based.
2. Revert to the original time-based auton, ship it, lose some points but don't break what works.

Don't be too proud to fall back. The sunk cost of hours coded does not justify a robot that doesn't move on the field.
