# 02 — Hardware Setup

Specific hardware setup steps and reference info. For the higher-level phased plan, see `01_STRATEGY.md`.

## Sensor Inventory

| Component | Part Number | Status |
|-----------|------------|--------|
| V5 Robot Brain | (varies) | On robot |
| V5 Smart Motors | (varies) | On robot, drivetrain + mechanisms |
| V5 AI Vision Sensor | 276-8659 | On hand, not yet configured |
| V5 Inertial Sensor (IMU) | 276-2333 | Being acquired tonight |

## Cables Required

### V5 Smart Cable
The standard 4-pin Smart Cable used for all V5 sensors and motors. Comes in six predefined lengths from VEX. The AI Vision Sensor and IMU each need one to connect to a Smart Port on the brain. Length should be appropriate to mounting position — extra slack is fine, too short means rerouting.

### USB-C Cable (for AI Vision Sensor configuration only)
Required for initial configuration of the AI Vision Sensor via the AI Vision Utility on a computer. Any USB-C cable that supports data (not charge-only) works. Once configuration is complete, this cable is not needed during normal operation.

## AI Vision Sensor Configuration

This must be done before the sensor is useful in code. The wrapper module assumes the sensor has been configured with AI Classification mode enabled and the Push Back model selected.

### Step-by-step

1. Connect the AI Vision Sensor to a computer via USB-C cable.
2. Open VEXcode V5 (App-based or Web-based) on the computer.
3. Open the AI Vision Utility.
4. Verify firmware is up to date. If "New Update Available" appears, follow VEX's firmware update procedure first.
5. Enable AI Classification mode in the Detection settings.
6. Select the **V5RC Push Back** classification model.
7. In the live preview, hold the camera up to your target objects and verify they are detected. **Critical checkpoint:** confirm the specific game objects your scoring strategy depends on are recognized by the model. If a target object is not in the model's detection list, the strategy must pivot to color signature mode or change targets — find this out before leaving the configuration step.
8. Note the class IDs / labels reported for each target object. These ID numbers will be referenced in code constants.
9. Adjust exposure / brightness if your venue lighting is different from your testing environment (gym lights vs. classroom can matter).
10. Disconnect USB-C, attach Smart Cable, mount on robot.

### What the wrapper expects
The `ai_vision::Camera` wrapper calls `enable_detection_types(pros::AivisionModeType::objects)` in its `initialize()`, which switches the sensor into AI Object detection mode at runtime. However, this does NOT select the model — the model selection (V5RC Push Back) persists from the AI Vision Utility configuration. You cannot select the model from code; it must be set via the utility.

## IMU Mounting

Notes on physical mounting that affect software behavior:

- **Flat mounting matters.** The IMU's gyroscope is most accurate when mounted flat (parallel to the field surface). A tilted IMU produces heading values that mix yaw, pitch, and roll in ways that confuse autonomous turning logic.
- **Distance from motors matters less than rumored** for the V5 IMU (it relies primarily on gyro/accelerometer rather than magnetometer), but is still good hygiene. Mount where convenient and flat first; "near a motor" is a tertiary concern.
- **Center of rotation, when practical.** Mounting near the robot's center of rotation makes future expansion to position tracking simpler. For pure heading-based turning (what we're doing), the mount location does not matter.
- **Secure mounting only.** Loose IMUs cause heading values to wobble during driving, which corrupts straight-line correction.
- **One Smart Port.** Note the port number; this goes into the config module.

## AI Vision Sensor Mounting

- **Forward-facing.** No pan/tilt rigs. Camera looks the direction the robot drives.
- **Height matters.** Mount so target objects appear in frame at the typical distances where vision-guided decisions are made (e.g., when approaching a scoring zone). Too high or too low and the target leaves the frame at critical moments.
- **Stable mounting.** Vibration causes detections to jitter, which makes the centering control loop oscillate. A rigid mount with no flex is essential.
- **Avoid motor proximity** primarily for cable routing reasons (Smart Cables snagging on rotating parts), not electrical.

## Image Coordinate System (Reference)

Important constants the wrapper code uses:

- Image width: 320 pixels
- Image height: 240 pixels
- Center of frame: (160, 120)
- Origin (0, 0): top-left corner

The wrapper translates the raw API's top-left bounding box coordinates into center coordinates internally, so callers of the wrapper work with center coordinates throughout.

## Power

- The V5 Robot Brain runs on the V5 Battery. Standard.
- All sensors (IMU, AI Vision) draw power through their Smart Cable from the brain. No external power required.
- No secondary battery is in use. (VURC rules permit one secondary battery for additional electronics; we are not using this allowance.)

## Port Map (Template)

Fill in with actual port assignments once decided:

| Port | Component | Notes |
|------|-----------|-------|
| 1    | Drive motor (front-left)? | |
| 2    | Drive motor (front-right)? | |
| 3    | ... | |
| ...  | ... | |
| ?    | V5 IMU | Decided after physical mounting |
| ?    | AI Vision Sensor | Decided after physical mounting |

Maintain this table as a sticky note on the brain or in the config module. Inspectors and pit crews will need it.

## What NOT To Do

- Do not power the AI Vision Sensor through any path except the Smart Cable to the brain.
- Do not connect the AI Vision Sensor to USB-C and the brain Smart Cable simultaneously during competition; configure via USB-C, then unplug and switch to Smart Cable.
- Do not mount the IMU at an angle "to save space." Heading math assumes flat.
- Do not use rubber bands, tape, or zip ties to "temporarily" mount the IMU. Loose = wandering.
- Do not connect the IQ AI Vision Sensor (228-9136) to a V5 brain. Different connector, different API, will not work.
