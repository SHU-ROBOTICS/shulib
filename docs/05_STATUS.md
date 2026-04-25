# 05 — Status and Next Steps

**This document is the most volatile in the doc set.** Update it as work progresses. Future Claude instances should read this every time they re-enter the project — the rest of the docs describe intent, this one describes reality.

---

## Last Updated

At the conversation handoff to Claude Code. Specific items below reflect status at that moment.

---

## Done

- Strategic plan locked. See `01_STRATEGY.md`.
- Decision register established. See `04_DECISIONS.md`.
- AI Vision Sensor part number confirmed: 276-8659 (V5 version, NOT the IQ version 228-9136).
- AI Vision Sensor wrapper module written.
  - Files: `include/ai_vision.hpp`, `src/ai_vision.cpp`.
  - See `03_MODULES.md` for API reference.
  - **Has NOT been tested for compilation in the user's project yet.**
- Hardware acquisition list confirmed: V5 Inertial Sensor (IMU), one Smart Cable, one USB-C cable.
- Configuration workflow for AI Vision Sensor documented in `02_HARDWARE_SETUP.md`.

## In Progress (At Handoff)

- User physically acquiring the V5 Inertial Sensor.
- User configuring AI Vision Sensor via the AI Vision Utility (USB-C → enable AI Classification → select Push Back model → verify target classes detected).
- User gathering existing LemLib chassis configuration code to share.
- User has not yet compiled or tested the AI Vision wrapper module.

## Blocked / Pending User Input

The following work cannot proceed without input from the user:

- **IMU integration into existing chassis config.** Blocked on user sharing the existing chassis configuration code. Without seeing it, any edit would be a guess. Do NOT write this without seeing the existing config.
- **Config module port assignments.** Blocked on user finalizing physical port assignments after mounting the IMU and AI Vision Sensor.
- **AI Vision class IDs.** Blocked on user running the AI Vision Utility against actual target objects and noting the class IDs reported.
- **Skills routine specific segments.** Blocked on user sharing detailed scoring strategy. (User has confirmed a strategy exists but has not described it in detail in conversation yet.)

## Not Started

- Config module (header with port constants and tuning constants).
- Drive primitives module (`turnToHeading`, `driveDistance`, `brake`).
- Vision-guided behaviors module (`visionCenter`, `visionApproach`, `visionAcquire`).
- Logging helper functions.
- Skills auton routine (the actual sequence of segments).
- Field testing and calibration.
- Code-only iteration after field session.

## Open Questions for Next Session

When the user re-engages, here is what should be confirmed before resuming code:

1. **Did the AI Vision wrapper compile in the project?** (Status: untested as of handoff.)
2. **Has the AI Vision Sensor been configured via the utility?**
3. **Has the IMU been physically mounted?**
4. **What ports are assigned to IMU and AI Vision Sensor?**
5. **What does the existing LemLib chassis config look like?** (Code paste needed.)
6. **What AI Vision class IDs do the user's target objects report as?**
7. **What is the user's specific scoring strategy, segment by segment?**
8. **How much field access time is left?**

## Recommended Next Action

Once the user is ready to resume:

1. Confirm the AI Vision wrapper compiles. If not, fix that first.
2. See the existing LemLib chassis config.
3. Write the IMU integration as a one-line-or-so edit to the existing config.
4. Write `config.hpp` (port constants + tuning constants + class IDs).
5. Write the drive primitives module.
6. Write the vision-guided behaviors module.
7. Write the skills routine, segment by segment.
8. Field test and calibrate.

This is the order from `01_STRATEGY.md` Phase 2 onward.

## What Should NOT Happen Next

If you find yourself doing any of these without explicit user re-confirmation, stop and ask:

- Writing code that depends on the existing chassis config without having seen it.
- Picking port numbers without user confirmation.
- Inventing class IDs the user has not yet provided.
- Writing the skills routine without a detailed scoring strategy.
- Proposing to add tracking wheels, a Raspberry Pi, full odometry, or a Jetson. These were rejected. See `04_DECISIONS.md`.

## Known Risks At Handoff

- **AI Vision wrapper untested.** If `pros::AIVision` is not in the user's PROS version (older PROS 3 perhaps), the wrapper will fail to compile. The fix is straightforward but needs to happen before any vision-guided code is written.
- **Field access window is small.** Calibration must be efficient. Constants should be in one place.
- **Sleep deprivation** will degrade code quality as the night progresses. The plan includes "stop coding by some defined hour" as a deliberate item; if Claude is helping at that hour, surface that observation.

## Fallback Plan Reminder

If the full plan is failing during execution:
1. **Tier 1:** Full plan (vision + IMU + drive distance with heading correction).
2. **Tier 2:** IMU-only (skip vision entirely; still a major upgrade over current time-based).
3. **Tier 3:** Revert to current time-based auton.

See `01_STRATEGY.md` Fallback Ladder section for details.
