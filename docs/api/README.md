<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Regenerate: python3 tools/api_doc_tool.py generate -->

# API reference

> **Writing an autonomous routine? You need two of these pages.**
> [`Chassis`](chassis.md) is the facade every routine is written against, and [`Routine`](routine.md) is the fluent recipe layer on top of it. Everything else on this page is the machinery underneath — real, documented, and safe to ignore until you want it.

**Every public entity in every shipped header** — 1,631 of them across 115 headers: types and their members, nested types, free functions, namespace-scope constants and type aliases. Extracted from the headers, so it cannot fall behind the code: anything added to a shipped header appears here the next time the tool runs, and the host test build fails if it has not.

**A public entity with no documentation comment fails the build**, naming itself and its file and line. That gate is what makes "generated" mean "complete" rather than "generated from whatever someone remembered to write".

**What is not here, and why.** Four exclusions, all deliberate:

- **`include/shulib/sim/`** — the host simulator. Test-only, and not by convention: a CI guard fails the build if anything outside `sim/` includes it, so no robot binary can reach it.
- **`hal/fake/` and `localization/fake/`** — the test doubles the suite drives the real seams with. Public by file placement, test fixtures by charter; `test/README.md` is their documentation.
- **Preprocessor macros** (`SHULIB_PRECONDITION`, `SHULIB_TRACE`). A macro has no signature, no access and no type, so there is nothing for an extractor to render without inventing it. Each is explained at length in its own header's design commentary, which every page below reproduces in full — so they are on the site, in prose, but not in the member lists or the index.
- **`protected` members** — one section in the tree, in `motion/move_to_pose.hpp`. This reference documents the surface you *call*; the surface you *subclass* is [guide chapter 13](../guide/13-extending-the-library.md)'s subject.

**Being on this page does not freeze anything.** Most of what follows is unfrozen and expected to move. The Freeze Register in the [roadmap](../roadmap.md) is the only place a contract is locked, and it is enforced by compile-time signature pins, not by this page: changing a frozen signature fails a C++ test that names the register row, while changing anything else here costs one `///` edit and a regeneration. Those are different mechanisms and only the first is a promise.

**What the gate does not check.** It proves every entity *has* a documentation comment. It has no opinion about whether that comment says anything — `/// Sets the voltage.` on `setVoltage` passes. Only a reader catches that, which is why the headers are written to be read and why each page below ends with the header's own design commentary rather than a bare list of signatures.

Prose about *how to think about* the API lives in the [user guide](../guide/README.md) — chapter 10 is the API as prose, and deliberately does not restate signatures. Worked recipes live in the [cookbook](../cookbook/README.md). This page answers "what exactly exists, and what is its exact spelling".

## Pages

### Chassis and routines

| Page | Header | What it is |
|---|---|---|
| [Chassis](chassis.md) | [`chassis/chassis.hpp`](../../include/shulib/chassis/chassis.hpp) | Chassis — the public facade every auton is written against. |
| [Robot context](robot_context.md) | [`chassis/robot_context.hpp`](../../include/shulib/chassis/robot_context.hpp) | RobotContext — the composition root / DI container: "the one object that differs across robot / sim / test". |
| [Routine](routine.md) | [`chassis/routine.hpp`](../../include/shulib/chassis/routine.hpp) | Routine — the Tier-2 recipe layer. |

### Motion

| Page | Header | What it is |
|---|---|---|
| [Command pipeline](command_pipeline.md) | [`motion/command_pipeline.hpp`](../../include/shulib/motion/command_pipeline.hpp) | applyCommandPipeline — the ONE command path from a chassis-speeds demand to energized motors. |
| [Drive brake](drive_brake.md) | [`motion/drive_brake.hpp`](../../include/shulib/motion/drive_brake.hpp) | DriveBrake — stop the drivetrain and confirm it stopped. |
| [Hold pose](hold_pose.md) | [`motion/hold_pose.hpp`](../../include/shulib/motion/hold_pose.hpp) | HoldPose — actively hold a FIELD pose against disturbance. |
| [Motion](motion.md) | [`motion/motion.hpp`](../../include/shulib/motion/motion.hpp) | IMotion — the contract every motion primitive implements. |
| [Motion config](motion_config.md) | [`motion/motion_config.hpp`](../../include/shulib/motion/motion_config.hpp) | MotionConfig — the shared knobs of the C1 motion primitives. |
| [Motion scheduler](motion_scheduler.md) | [`motion/motion_scheduler.hpp`](../../include/shulib/motion/motion_scheduler.hpp) | MotionScheduler — the thing that actually runs a routine. |
| [Move to pose](move_to_pose.md) | [`motion/move_to_pose.hpp`](../../include/shulib/motion/move_to_pose.hpp) | MoveToPose — decoupled per-axis field-pose motion. |
| [Odometry stall check](odo_stall_check.md) | [`motion/odo_stall_check.hpp`](../../include/shulib/motion/odo_stall_check.hpp) | OdoStallCheck — the spin-vs-motion cross-check. |
| [Run reporter](run_reporter.md) | [`motion/run_reporter.hpp`](../../include/shulib/motion/run_reporter.hpp) | RunReporter — the glue that makes a run LEGIBLE end to end (WS13, chunk C5): session header (§18.5) → per-motion result lines (§18.3/§18.4) → run summary (§18.3). |
| [Strafe to](strafe_to.md) | [`motion/strafe_to.hpp`](../../include/shulib/motion/strafe_to.hpp) | StrafeTo — translate to a FIELD (x, y) while HOLDING heading. |
| [Turn to](turn_to.md) | [`motion/turn_to.hpp`](../../include/shulib/motion/turn_to.hpp) | TurnTo — rotate in place to a FIELD heading. |

### Control

| Page | Header | What it is |
|---|---|---|
| [Exit group](exit_group.md) | [`control/exit_group.hpp`](../../include/shulib/control/exit_group.hpp) | ExitReason / ExitGroup — the motion-exit decision. |
| [Feedforward](feedforward.md) | [`control/feedforward.hpp`](../../include/shulib/control/feedforward.hpp) | Feedforward — the kS/kV/kA motor feedforward (master plan §M2): the open-loop voltage to achieve a target velocity + acceleration, so the PID only has to correct the residual. |
| [PID](pid.md) | [`control/pid.hpp`](../../include/shulib/control/pid.hpp) | Pid — a single-axis PID controller. |
| [Settled util](settled_util.md) | [`control/settled_util.hpp`](../../include/shulib/control/settled_util.hpp) | SettledUtil — the motion exit check. |
| [Trapezoid profile](trapezoid_profile.md) | [`control/trapezoid_profile.hpp`](../../include/shulib/control/trapezoid_profile.hpp) | TrapezoidProfile — a trapezoidal motion profile. |
| [Watchdog](watchdog.md) | [`control/watchdog.hpp`](../../include/shulib/control/watchdog.hpp) | Watchdog — a hard timeout primitive. |

### Kinematics

| Page | Header | What it is |
|---|---|---|
| [Desaturate](desaturate.md) | [`kinematics/desaturate.hpp`](../../include/shulib/kinematics/desaturate.hpp) | desaturateUniform — the downstream wheel-speed safety scale. |
| [H drive](h_drive.md) | [`kinematics/h_drive.hpp`](../../include/shulib/kinematics/h_drive.hpp) | hDrive() — the H-drive (tank base + one transverse strafe wheel), as a MatrixKinematics preset (chunk C3; the hybrid backend §13 #15: a holonomic LINEAR drive is a coefficient table, exactly as xDrive() is). |
| [Kinematics](kinematics.md) | [`kinematics/kinematics.hpp`](../../include/shulib/kinematics/kinematics.hpp) | IKinematics — the drivetrain math contract. |
| [Matrix kinematics](matrix_kinematics.md) | [`kinematics/matrix_kinematics.hpp`](../../include/shulib/kinematics/matrix_kinematics.hpp) | MatrixKinematics — the coefficient-matrix engine for FULLY-HOLONOMIC LINEAR drives (the hybrid backend, §13 #15). |
| [Tank](tank.md) | [`kinematics/tank.hpp`](../../include/shulib/kinematics/tank.hpp) | TankKinematics — a 2-wheel differential (skid-steer) drive. |
| [Wheel speeds](wheel_speeds.md) | [`kinematics/wheel_speeds.hpp`](../../include/shulib/kinematics/wheel_speeds.hpp) | WheelSpeeds — per-wheel linear *surface* speeds (in/s), in a drivetrain-defined wheel order (each IKinematics impl documents its own order). |
| [X drive](x_drive.md) | [`kinematics/x_drive.hpp`](../../include/shulib/kinematics/x_drive.hpp) | xDrive() — the symmetric 45° X-drive, as a MatrixKinematics preset (the hybrid backend §13 #15: a holonomic linear drive is just a coefficient table). |

### Localization

| Page | Header | What it is |
|---|---|---|
| [AprilTag corrector](apriltag_corrector.md) | [`localization/apriltag_corrector.hpp`](../../include/shulib/localization/apriltag_corrector.hpp) | AprilTagCorrector — the SECOND real corrector, and the FIRST source in the tree that can tell the estimator which way it is actually pointing. |
| [Arc step](arc_step.md) | [`localization/arc_step.hpp`](../../include/shulib/localization/arc_step.hpp) | arc_step.hpp — the one constant-curvature integration step. |
| [Complementary fusion](complementary_fusion.md) | [`localization/complementary_fusion.hpp`](../../include/shulib/localization/complementary_fusion.hpp) | ComplementaryFusion — the M2 fusion policy. |
| [Correction](correction.md) | [`localization/correction.hpp`](../../include/shulib/localization/correction.hpp) | correction.hpp — the value types the localization fusion seam exchanges. |
| [EKF fusion](ekf_fusion.md) | [`localization/ekf_fusion.hpp`](../../include/shulib/localization/ekf_fusion.hpp) | EkfFusion — the M3 fusion policy: a 5-state SE(2) extended Kalman filter behind the SAME `IFusionPolicy` seam `ComplementaryFusion` has occupied since M2. |
| [GPS corrector](gps_corrector.md) | [`localization/gps_corrector.hpp`](../../include/shulib/localization/gps_corrector.hpp) | GpsCorrector — the FIRST REAL corrector. |
| [ICorrector](i_corrector.md) | [`localization/i_corrector.hpp`](../../include/shulib/localization/i_corrector.hpp) | ICorrector — the WRITE seam: one source of ABSOLUTE position fixes (V5 GPS, AprilTag PnP, LIDAR scan-match). |
| [IFusionPolicy](i_fusion_policy.md) | [`localization/i_fusion_policy.hpp`](../../include/shulib/localization/i_fusion_policy.hpp) | IFusionPolicy — the swap point that lets a complementary filter ship NOW and a 5-state SE(2) EKF drop in LATER behind the same seam. |
| [IPoseSource](i_pose_source.md) | [`localization/i_pose_source.hpp`](../../include/shulib/localization/i_pose_source.hpp) | IPoseSource — the READ seam every pose consumer (motion, alignment, telemetry, skills) depends on. |
| [Localizer](localizer.md) | [`localization/localizer.hpp`](../../include/shulib/localization/localizer.hpp) | Localizer — the fused field-frame estimate. |
| [Pilons odometry](pilons_odometry.md) | [`localization/pilons_odometry.hpp`](../../include/shulib/localization/pilons_odometry.hpp) | PilonsOdometry — tracking-wheel dead-reckoning. |
| [Tag map](tag_map.md) | [`localization/tag_map.hpp`](../../include/shulib/localization/tag_map.hpp) | TagMap — where the AprilTags are on the field, and where each of those numbers CAME FROM. |
| [Tracking wheel](tracking_wheel.md) | [`localization/tracking_wheel.hpp`](../../include/shulib/localization/tracking_wheel.hpp) | TrackingWheel — one unpowered odometry wheel: an `IRotation` sensor + the wheel's diameter + its mounting offset from the tracking center. |

### Manipulation

| Page | Header | What it is |
|---|---|---|
| [Mechanism op](mechanism_op.md) | [`manipulation/mechanism_op.hpp`](../../include/shulib/manipulation/mechanism_op.hpp) | The bounded mechanism operation (chunk F1, WS7/M4): IMechanismOp + the two season-free operations every scoring verb decomposes into — run a motor mechanism until something confirms (RunUntilConfirmed) and fire a discrete actuator, wait for it to physically… |
| [Mechanism outcome](mechanism_outcome.md) | [`manipulation/mechanism_outcome.hpp`](../../include/shulib/manipulation/mechanism_outcome.hpp) | MechanismOutcome — the verdict vocabulary of a bounded mechanism operation. |
| [Stall detector](stall_detector.md) | [`manipulation/stall_detector.hpp`](../../include/shulib/manipulation/stall_detector.hpp) | StallDetector — the jam/stall decision for motor mechanisms. |

### Sequencing

| Page | Header | What it is |
|---|---|---|
| [Run guard](run_guard.md) | [`sequence/run_guard.hpp`](../../include/shulib/sequence/run_guard.hpp) | RunGuard — the run-scoped deadline owner and the guaranteed END-OF-RUN ACTION. |

### Diagnostics

| Page | Header | What it is |
|---|---|---|
| [Blackbox format](blackbox_format.md) | [`diag/blackbox_format.hpp`](../../include/shulib/diag/blackbox_format.hpp) | The SHULIB BLACKBOX on-disk format, v1 — the binary record SdSink writes and BlackboxReader reads. |
| [Blackbox reader](blackbox_reader.md) | [`diag/blackbox_reader.hpp`](../../include/shulib/diag/blackbox_reader.hpp) | BlackboxReader — THE DECODER. It ships in the same chunk as the encoder, because a format nothing can read is not a record: the first time a blackbox genuinely matters is a competition afternoon, and a file that cannot be opened that afternoon is worth exac… |
| [Build info](build_info.md) | [`diag/build_info.hpp`](../../include/shulib/diag/build_info.hpp) | build_info — the git build hash plumbing for the §18.5 session header. |
| [Controller display](controller_display.md) | [`diag/controller_display.hpp`](../../include/shulib/diag/controller_display.hpp) | ControllerFaultDisplay — the D-4 controller-screen content. |
| [Debug record](debug_record.md) | [`diag/debug_record.hpp`](../../include/shulib/diag/debug_record.hpp) | DebugRecord — the per-tick snapshot schema. |
| [Fault](fault.md) | [`diag/fault.hpp`](../../include/shulib/diag/fault.hpp) | Fault discipline (master plan §18.4; WS13, chunk A1) — the stable numeric fault-code enum and the latched first-fault capture. |
| [Finite guard](finite_guard.md) | [`diag/finite_guard.hpp`](../../include/shulib/diag/finite_guard.hpp) | Finite-value invariant guards (master plan §18.4) — the LOG-AND-RECOVER counterpart to SHULIB_PRECONDITION's throw. |
| [Health monitor](health_monitor.md) | [`diag/health_monitor.hpp`](../../include/shulib/diag/health_monitor.hpp) | HealthMonitor — sensor/power pathology → FaultCode, edge-triggered. |
| [Level filter sink](level_filter_sink.md) | [`diag/level_filter_sink.hpp`](../../include/shulib/diag/level_filter_sink.hpp) | LevelFilterSink — per-subsystem log levels. |
| [Line format](line_format.md) | [`diag/line_format.hpp`](../../include/shulib/diag/line_format.hpp) | line_format — the ONE set of §18.3 text-formatting primitives. |
| [Loop monitor](loop_monitor.md) | [`diag/loop_monitor.hpp`](../../include/shulib/diag/loop_monitor.hpp) | LoopMonitor — loop-overrun / tick-timing detection. |
| [Motion result](motion_result.md) | [`diag/motion_result.hpp`](../../include/shulib/diag/motion_result.hpp) | MotionResult — the per-motion result line, as data + one formatter. |
| [Plausibility guard](plausibility_guard.md) | [`diag/plausibility_guard.hpp`](../../include/shulib/diag/plausibility_guard.hpp) | Physical-plausibility invariants (diagnostics-plan D-5; WS13, chunk C5) — FiniteGuard's log-and-recover posture, extended beyond finiteness. |
| [Rate limit sink](rate_limit_sink.md) | [`diag/rate_limit_sink.hpp`](../../include/shulib/diag/rate_limit_sink.hpp) | RateLimitedSink — per-channel rate limiting with COUNTED, REPORTED drops. |
| [Run summary](run_summary.md) | [`diag/run_summary.hpp`](../../include/shulib/diag/run_summary.hpp) | RunSummary — the end-of-run one-screen summary, as DATA. |
| [SD sink](sd_sink.md) | [`diag/sd_sink.hpp`](../../include/shulib/diag/sd_sink.hpp) | SdSink — the BLACKBOX: a binary, versioned, session-stamped record of a run, written to the brain's SD card. |
| [Session info](session_info.md) | [`diag/session_info.hpp`](../../include/shulib/diag/session_info.hpp) | SessionInfo + the §18.5 session header — provenance as the FIRST lines of every run. |
| [Term sink](term_sink.md) | [`diag/term_sink.hpp`](../../include/shulib/diag/term_sink.hpp) | TermSink — the human-readable terminal stream, the PRIMARY dev/debug surface. |
| [Tick attribution](tick_attribution.md) | [`diag/tick_attribution.hpp`](../../include/shulib/diag/tick_attribution.hpp) | TickAttribution — WHO consumed the loop budget. |
| [Trace](trace.md) | [`diag/trace.hpp`](../../include/shulib/diag/trace.hpp) | SHULIB_TRACE — the compile-time TRACE strip. |
| [Triage](triage.md) | [`diag/triage.hpp`](../../include/shulib/diag/triage.hpp) | The D-7 TRIAGE BLOCK — "why did it break", rendered for a human. |

### Math and frames

| Page | Header | What it is |
|---|---|---|
| [Angle](angle.md) | [`math/angle.hpp`](../../include/shulib/math/angle.hpp) | Angle — a heading on SE(2). The one type that owns angle wrapping, so the "degrees into cos/sin" and "359° vs -1°" bug classes are impossible by construction. |
| [Frame](frame.md) | [`math/frame.hpp`](../../include/shulib/math/frame.hpp) | frame.hpp — THE ONE PLACE a frame rotation is allowed. |
| [Pose2d](pose2d.md) | [`math/pose2d.hpp`](../../include/shulib/math/pose2d.hpp) | Pose2d — a rigid-body pose on SE(2): position (x, y) + heading. |
| [Twist2d](twist2d.md) | [`math/twist2d.hpp`](../../include/shulib/math/twist2d.hpp) | Twist2d and ChassisSpeeds — the velocity currencies of the motion stack. |

### Units

| Page | Header | What it is |
|---|---|---|
| [Literals](literals.md) | [`units/literals.hpp`](../../include/shulib/units/literals.hpp) | User-defined literals for shulib units. Each converts to CANONICAL units at the point of writing, so the rest of the code never sees a raw unit again: distance = 24_in; field = 1_tile; dt = 20_ms; v = 12_volt;. |
| [Quantity](quantity.md) | [`units/quantity.hpp`](../../include/shulib/units/quantity.hpp) | Quantity<L, A, T, E, I> — compile-time dimensional analysis. |

### HAL — the hardware seams

| Page | Header | What it is |
|---|---|---|
| [Battery](battery.md) | [`hal/battery.hpp`](../../include/shulib/hal/battery.hpp) | IBattery — the V5 battery (pros::battery) behind the HAL. |
| [Block sink](block_sink.md) | [`hal/block_sink.hpp`](../../include/shulib/hal/block_sink.hpp) | IBlockSink — where BINARY BLOCKS physically go (an SD-card file on the brain, a captured buffer in a test). |
| [Char sink](char_sink.md) | [`hal/char_sink.hpp`](../../include/shulib/hal/char_sink.hpp) | ICharSink — where formatted diagnostic BYTES physically go (a terminal, a captured string in a test, later a serial port). |
| [Clock](clock.md) | [`hal/clock.hpp`](../../include/shulib/hal/clock.hpp) | IClock — the single source of "now" for the whole stack. |
| [Controller](controller.md) | [`hal/controller.hpp`](../../include/shulib/hal/controller.hpp) | IController — the V5 game controller behind the HAL (chunk R1a, absorbing Phase T's T1): the INPUT half of driver control. |
| [Controller conversion](controller_conversion.md) | [`hal/controller_conversion.hpp`](../../include/shulib/hal/controller_conversion.hpp) | Controller canonical conversions — the ONE place the V5 controller's raw stick range becomes shulib's canonical [-1, 1] (§7: "convert exactly once, at the edge"). |
| [Digital in](digital_in.md) | [`hal/digital_in.hpp`](../../include/shulib/hal/digital_in.hpp) | IDigitalIn — a single digital input line behind the HAL (chunk R1b): a limit switch, a bumper, a jumper — any two-state sensor on an ADI port. |
| [Digital out](digital_out.md) | [`hal/digital_out.hpp`](../../include/shulib/hal/digital_out.hpp) | IDigitalOut — a single digital output line behind the HAL (chunk F1, WS7/M4): a pneumatic solenoid on the ADI ports, or any other two-state actuator. |
| [Distance](distance.md) | [`hal/distance.hpp`](../../include/shulib/hal/distance.hpp) | IDistance — a distance / time-of-flight sensor (pros::Distance) behind the HAL. |
| [Distance conversion](distance_conversion.md) | [`hal/distance_conversion.hpp`](../../include/shulib/hal/distance_conversion.hpp) | Distance-sensor canonical conversions — the ONE place the V5 distance sensor's millimeters become shulib's canonical inches and its raw 0–63 confidence becomes [0, 1] (§7: "convert exactly once, at the edge"). |
| [GPS](gps.md) | [`hal/gps.hpp`](../../include/shulib/hal/gps.hpp) | IGps — the VEX GPS behind the HAL, reporting in shulib's CANONICAL frame (the VEX meters / clockwise-from-North convention is converted away in the hal/pros adapter via gps_conversion.hpp). |
| [GPS conversion](gps_conversion.md) | [`hal/gps_conversion.hpp`](../../include/shulib/hal/gps_conversion.hpp) | GPS canonical conversions — the ONE place the VEX GPS frame becomes shulib's canonical frame (§7: "convert exactly once, at the edge"). |
| [IMU](imu.md) | [`hal/imu.hpp`](../../include/shulib/hal/imu.hpp) | IImu — the inertial sensor behind the HAL, reporting in shulib's CANONICAL frame (the V5's clockwise/degrees convention is converted away in the hal/pros adapter via imu_conversion.hpp, so everything above this line is CCW-positive radians). |
| [IMU conversion](imu_conversion.md) | [`hal/imu_conversion.hpp`](../../include/shulib/hal/imu_conversion.hpp) | IMU canonical conversions — the ONE place the V5 inertial sensor's frame becomes shulib's canonical frame (§7: "convert exactly once, at the edge"). |
| [Line display](line_display.md) | [`hal/line_display.hpp`](../../include/shulib/hal/line_display.hpp) | ILineDisplay — where short status LINES physically go (the V5 controller's LCD; a captured fake in tests). |
| [Mechanism](mechanism.md) | [`hal/mechanism.hpp`](../../include/shulib/hal/mechanism.hpp) | The mechanism device seam (chunk F1, WS7/M4): IMechanism + the two concrete compositions every VEX mechanism reduces to at the device level — a group of motors on one shaft (MotorMechanism) and a set of digital lines switching one pneumatic circuit (Pneumat… |
| [Motor](motor.md) | [`hal/motor.hpp`](../../include/shulib/hal/motor.hpp) | IMotor — a single V5 smart motor behind the HAL. |
| [Motor conversion](motor_conversion.md) | [`hal/motor_conversion.hpp`](../../include/shulib/hal/motor_conversion.hpp) | Motor canonical conversions — the ONE place the V5 smart motor's units become shulib's canonical units (§7: "convert exactly once, at the edge"). |
| [Null sink](null_sink.md) | [`hal/null_sink.hpp`](../../include/shulib/hal/null_sink.hpp) | NullSink — the zero-cost default ITelemetrySink (§18.1). |
| [Optical](optical.md) | [`hal/optical.hpp`](../../include/shulib/hal/optical.hpp) | IOptical — a color / optical sensor (pros::Optical) behind the HAL. |
| [Optical conversion](optical_conversion.md) | [`hal/optical_conversion.hpp`](../../include/shulib/hal/optical_conversion.hpp) | Optical-sensor canonical conversions — the ONE place the V5 optical sensor's raw channels become shulib's canonical ranges (§7: "convert exactly once, at the edge"). |
| [Rotation](rotation.md) | [`hal/rotation.hpp`](../../include/shulib/hal/rotation.hpp) | IRotation — a rotation / tracking-wheel sensor (pros::Rotation) behind the HAL. |
| [Rotation conversion](rotation_conversion.md) | [`hal/rotation_conversion.hpp`](../../include/shulib/hal/rotation_conversion.hpp) | Rotation-sensor canonical conversions — the ONE place the V5 rotation sensor's centidegrees become shulib's canonical radians (§7: "convert exactly once, at the edge"). |
| [Telemetry sink](telemetry_sink.md) | [`hal/telemetry_sink.hpp`](../../include/shulib/hal/telemetry_sink.hpp) | ITelemetrySink — the diagnostics output seam. |
| [Vision](vision.md) | [`hal/vision.hpp`](../../include/shulib/hal/vision.hpp) | IVision / ITagSource — the AI Vision seams. |
| [Vision conversion](vision_conversion.md) | [`hal/vision_conversion.hpp`](../../include/shulib/hal/vision_conversion.hpp) | vision_conversion.hpp — the ONE place raw AprilTag image corners become shulib's canonical robot-relative tag pose (§7: convert exactly once, at the edge). |

### HAL — the PROS adapters

| Page | Header | What it is |
|---|---|---|
| [Battery (PROS)](pros-battery.md) | [`hal/pros/battery.hpp`](../../include/shulib/hal/pros/battery.hpp) | ProsBattery — IBattery over the pros::battery namespace (chunk R1a): the brownout-compensation input behind the HAL. |
| [Block sink (PROS)](pros-block_sink.md) | [`hal/pros/block_sink.hpp`](../../include/shulib/hal/pros/block_sink.hpp) | ProsBlockSink — IBlockSink over a PROS FILE* on the V5's SD card (chunk R1b): where the blackbox's binary blocks physically go. |
| [Char sink (PROS)](pros-char_sink.md) | [`hal/pros/char_sink.hpp`](../../include/shulib/hal/pros/char_sink.hpp) | ProsCharSink — ICharSink over the V5's USB serial (chunk R1a): where TermSink's diagnostic bytes physically go on the robot (`pros terminal` displays them). |
| [Clock (PROS)](pros-clock.md) | [`hal/pros/clock.hpp`](../../include/shulib/hal/pros/clock.hpp) | ProsClock — IClock over the V5's real time. |
| [Controller (PROS)](pros-controller.md) | [`hal/pros/controller.hpp`](../../include/shulib/hal/pros/controller.hpp) | ProsController — IController over pros::Controller (chunk R1a): the driver's hands behind the HAL. |
| [Digital in (PROS)](pros-digital_in.md) | [`hal/pros/digital_in.hpp`](../../include/shulib/hal/pros/digital_in.hpp) | ProsDigitalIn — IDigitalIn over pros::adi::DigitalIn (chunk R1b): a limit switch / bumper line behind the HAL. |
| [Digital out (PROS)](pros-digital_out.md) | [`hal/pros/digital_out.hpp`](../../include/shulib/hal/pros/digital_out.hpp) | ProsDigitalOut — IDigitalOut over pros::adi::DigitalOut (chunk R1b): the pneumatic solenoid line behind the HAL (F1's seam, finally on hardware). |
| [Distance (PROS)](pros-distance.md) | [`hal/pros/distance.hpp`](../../include/shulib/hal/pros/distance.hpp) | ProsDistance — IDistance over pros::Distance (chunk R1b): the capture/dock-confirm rangefinder behind the HAL. |
| [GPS (PROS)](pros-gps.md) | [`hal/pros/gps.hpp`](../../include/shulib/hal/pros/gps.hpp) | ProsGps — IGps over pros::Gps (chunk R1a): the absolute-position corrector's sensor behind the HAL. |
| [IMU (PROS)](pros-imu.md) | [`hal/pros/imu.hpp`](../../include/shulib/hal/pros/imu.hpp) | ProsImu — IImu over pros::Imu (chunk R1a): the load-bearing < 1° heading source behind the HAL. |
| [Line display (PROS)](pros-line_display.md) | [`hal/pros/line_display.hpp`](../../include/shulib/hal/pros/line_display.hpp) | ProsLineDisplay — ILineDisplay over the V5 controller's LCD (chunk R1a): where the D-4 status rows physically go. |
| [Motor (PROS)](pros-motor.md) | [`hal/pros/motor.hpp`](../../include/shulib/hal/pros/motor.hpp) | ProsMotor — IMotor over pros::Motor. |
| [Optical (PROS)](pros-optical.md) | [`hal/pros/optical.hpp`](../../include/shulib/hal/pros/optical.hpp) | ProsOptical — IOptical over pros::Optical (chunk R1b): the game-object color/proximity confirmation sensor behind the HAL. |
| [Rotation (PROS)](pros-rotation.md) | [`hal/pros/rotation.hpp`](../../include/shulib/hal/pros/rotation.hpp) | ProsRotation — IRotation over pros::Rotation (chunk R1a): the tracking-wheel pods behind the HAL. |
| [Tick pacer (PROS)](pros-tick_pacer.md) | [`hal/pros/tick_pacer.hpp`](../../include/shulib/hal/pros/tick_pacer.hpp) | ProsTickPacer — motion::ITickPacer over pros::Task::delay_until (chunk R1a): the ONLY seam that regains control mid-motion on the robot, replacing main.cpp's V5DelayPacer (which had to hand-advance a FakeClock). |

### Core

| Page | Header | What it is |
|---|---|---|
| [Check](check.md) | [`core/check.hpp`](../../include/shulib/core/check.hpp) | Precondition checking for shulib core. |

### Spec

| Page | Header | What it is |
|---|---|---|
| [Accuracy](accuracy.md) | [`spec/accuracy.hpp`](../../include/shulib/spec/accuracy.hpp) | accuracy.hpp — the accuracy targets of Freeze Register ROW F2, the LOCKED spec the autonomous is measured against. |

### Top level

| Page | Header | What it is |
|---|---|---|
| [Version](version.md) | [`version.hpp`](../../include/shulib/version.hpp) | The shulib API version — the mechanism behind the Freeze Register's promise. |

## Every public entity, alphabetically

**[The alphabetical index](all-entities.md)** lists all 1,631 of them with a link to each. Nested types appear under their qualified name (`BlackboxReader::Frame::type`), so a member of a nested type is findable by the name you would actually write.

## Where the other documents fit

- The [user guide](../guide/README.md) teaches the ideas in order, and chapter 10 is the API *as prose* — when to reach for a verb, what it does when things go wrong, which gotchas bite. It deliberately does not restate signatures; this reference owns those.
- The [cookbook](../cookbook/README.md) answers "how do I write the routine I am writing right now", with compiled recipes.
- This reference answers "what exactly exists, and what is its exact spelling".
