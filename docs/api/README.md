<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Regenerate: python3 tools/api_doc_tool.py generate -->

# API reference

> **Writing an autonomous routine? You need two of these pages.**
> [`Chassis`](chassis.md) is the facade every routine is written against, and [`Routine`](routine.md) is the fluent recipe layer on top of it. Everything else on this page is the machinery underneath — real, documented, and safe to ignore until you want it.

**Every public entity in every shipped header** — 6,440 of them across 214 headers: types and their members, nested types, free functions, namespace-scope constants and type aliases. Extracted from the headers, so it cannot fall behind the code: anything added to a shipped header appears here the next time the tool runs, and the host test build fails if it has not.

**A public entity with no documentation comment fails the build**, naming itself and its file and line. That gate is what makes "generated" mean "complete" rather than "generated from whatever someone remembered to write".

**What is not here, and why.** Four exclusions, all deliberate:

- **`include/shulib/sim/`** — the host simulator. Test-only, and not by convention: a CI guard fails the build if anything outside `sim/` includes it, so no robot binary can reach it.
- **`hal/fake/` and `localization/fake/`** — the test doubles the suite drives the real seams with. Public by file placement, test fixtures by charter; `test/README.md` is their documentation.
- **Preprocessor macros** (`SHULIB_PRECONDITION`, `SHULIB_TRACE`). A macro has no signature, no access and no type, so there is nothing for an extractor to render without inventing it. Each is explained at length in its own header's design commentary, which every page below reproduces in full — so they are on the site, in prose, but not in the member lists or the index.
- **`protected` members** — 2 sections in the tree, in `mechanism/service.hpp`, `motion/move_to_pose.hpp`. This reference documents the surface you *call*; the surface you *subclass* is [guide chapter 13](../guide/13-extending-the-library.md)'s subject.

**Being on this page does not freeze anything.** Most of what follows is unfrozen and expected to move. The Freeze Register in the [roadmap](../roadmap.md) is the only place a contract is locked, and it is enforced by compile-time signature pins, not by this page: changing a frozen signature fails a C++ test that names the register row, while changing anything else here costs one `///` edit and a regeneration. Those are different mechanisms and only the first is a promise.

**What the gate does not check.** It proves every entity *has* a documentation comment. It has no opinion about whether that comment says anything — `/// Sets the voltage.` on `setVoltage` passes. Only a reader catches that, which is why the headers are written to be read and why each page below ends with the header's own design commentary rather than a bare list of signatures.

Prose about *how to think about* the API lives in [How it works](../guide/README.md) — [The API in words](../guide/10-the-api.md) is the API as prose, and deliberately does not restate signatures. Worked recipes live in [Recipes](../cookbook/README.md). This page answers "what exactly exists, and what is its exact spelling".

## Pages

### Chassis and routines

| Page | Header | What it is |
|---|---|---|
| [Chassis](chassis.md) | [`chassis/chassis.hpp`](../../include/shulib/chassis/chassis.hpp) | Chassis — the public facade every auton is written against. |
| [Robot context](robot_context.md) | [`chassis/robot_context.hpp`](../../include/shulib/chassis/robot_context.hpp) | RobotContext — the composition root / DI container: "the one object that differs across robot / sim / test". |
| [Routine](routine.md) | [`chassis/routine.hpp`](../../include/shulib/chassis/routine.hpp) | Routine — the Tier-2 recipe layer. |
| [Wall reset call](wall_reset_call.md) | [`chassis/wall_reset_call.hpp`](../../include/shulib/chassis/wall_reset_call.hpp) | chassis::resetFromWall — the wall reset's EXPLICIT call (chunk L6-A1, 2026-10-29): re-seat ONE axis of the estimate against a field wall from a side distance sensor, over sensor handles the caller passes. |

### Motion

| Page | Header | What it is |
|---|---|---|
| [Command pipeline](command_pipeline.md) | [`motion/command_pipeline.hpp`](../../include/shulib/motion/command_pipeline.hpp) | applyCommandPipeline — the ONE command path from a chassis-speeds demand to energized motors. |
| [Curve follow](curve_follow.md) | [`motion/curve_follow.hpp`](../../include/shulib/motion/curve_follow.hpp) | CurveFollow — the curve verbs' motion (chunk CURVE1, 2026-11): the robot drives a smooth path through the author's points WITHOUT stopping at them — pure pursuit on the path curve_path.hpp builds, a speed plan that slows before each bend, two guards that st… |
| [Curve path](curve_path.md) | [`motion/curve_path.hpp`](../../include/shulib/motion/curve_path.hpp) | curve_path.hpp — the PURE path of the curve verbs (chunk CURVE1, 2026-11): the smooth path a curveTo / reverseCurveTo drives, built from the robot's live pose and the author's points, its speed plan, and the refusals and warnings a path earns before the rob… |
| [Drive brake](drive_brake.md) | [`motion/drive_brake.hpp`](../../include/shulib/motion/drive_brake.hpp) | DriveBrake — stop the drivetrain and confirm it stopped. |
| [Hold pose](hold_pose.md) | [`motion/hold_pose.hpp`](../../include/shulib/motion/hold_pose.hpp) | HoldPose — actively hold a FIELD pose against disturbance. |
| [Leg progress](leg_progress.md) | [`motion/leg_progress.hpp`](../../include/shulib/motion/leg_progress.hpp) | leg_progress.hpp — how far a drive or a turn has got, as a PARTWAY TRIGGER reads it (chunk STEP1-B, 2026-11-11; the STEP1 brief's D7 (a)). |
| [Motion](motion.md) | [`motion/motion.hpp`](../../include/shulib/motion/motion.hpp) | IMotion — the contract every motion primitive implements. |
| [Motion config](motion_config.md) | [`motion/motion_config.hpp`](../../include/shulib/motion/motion_config.hpp) | MotionConfig — the shared knobs of the C1 motion primitives. |
| [Motion scheduler](motion_scheduler.md) | [`motion/motion_scheduler.hpp`](../../include/shulib/motion/motion_scheduler.hpp) | MotionScheduler — the thing that actually runs a routine. |
| [Move to pose](move_to_pose.md) | [`motion/move_to_pose.hpp`](../../include/shulib/motion/move_to_pose.hpp) | MoveToPose — decoupled per-axis field-pose motion. |
| [Odometry stall check](odo_stall_check.md) | [`motion/odo_stall_check.hpp`](../../include/shulib/motion/odo_stall_check.hpp) | OdoStallCheck — the spin-vs-motion cross-check. |
| [Run reporter](run_reporter.md) | [`motion/run_reporter.hpp`](../../include/shulib/motion/run_reporter.hpp) | RunReporter — the glue that makes a run LEGIBLE end to end (WS13, chunk C5): session header (§18.5) → per-motion result lines (§18.3/§18.4) → run summary (§18.3). |
| [Strafe to](strafe_to.md) | [`motion/strafe_to.hpp`](../../include/shulib/motion/strafe_to.hpp) | StrafeTo — translate to a FIELD (x, y) while HOLDING heading. |
| [Tank steer](tank_steer.md) | [`motion/tank_steer.hpp`](../../include/shulib/motion/tank_steer.hpp) | tank_steer.hpp — the pure geometry of TANK1: where a tank is relative to its target's LINE, and how far MoveToPose bends the heading setpoint to steer it back. |
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
| [Axis](axis.md) | [`localization/axis.hpp`](../../include/shulib/localization/axis.hpp) | Axis — the FIELD axis a wall measures. |
| [Complementary fusion](complementary_fusion.md) | [`localization/complementary_fusion.hpp`](../../include/shulib/localization/complementary_fusion.hpp) | ComplementaryFusion — the M2 fusion policy. |
| [Correction](correction.md) | [`localization/correction.hpp`](../../include/shulib/localization/correction.hpp) | correction.hpp — the value types the localization fusion seam exchanges. |
| [Drive encoder odometry](drive_encoder_odometry.md) | [`localization/drive_encoder_odometry.hpp`](../../include/shulib/localization/drive_encoder_odometry.hpp) | DriveEncoderOdometry — dead reckoning from the DRIVE motors' own encoders and the IMU, for a tank drive with no tracking wheels. |
| [EKF fusion](ekf_fusion.md) | [`localization/ekf_fusion.hpp`](../../include/shulib/localization/ekf_fusion.hpp) | EkfFusion — the M3 fusion policy: a 5-state SE(2) extended Kalman filter behind the SAME `IFusionPolicy` seam `ComplementaryFusion` has occupied since M2. |
| [GPS corrector](gps_corrector.md) | [`localization/gps_corrector.hpp`](../../include/shulib/localization/gps_corrector.hpp) | GpsCorrector — the FIRST REAL corrector. |
| [ICorrector](i_corrector.md) | [`localization/i_corrector.hpp`](../../include/shulib/localization/i_corrector.hpp) | ICorrector — the WRITE seam: one source of ABSOLUTE position fixes (V5 GPS, AprilTag PnP, LIDAR scan-match). |
| [IFusionPolicy](i_fusion_policy.md) | [`localization/i_fusion_policy.hpp`](../../include/shulib/localization/i_fusion_policy.hpp) | IFusionPolicy — the swap point that lets a complementary filter ship NOW and a 5-state SE(2) EKF drop in LATER behind the same seam. |
| [IPoseSource](i_pose_source.md) | [`localization/i_pose_source.hpp`](../../include/shulib/localization/i_pose_source.hpp) | IPoseSource — the READ seam every pose consumer (motion, alignment, telemetry, skills) depends on. |
| [Localizer](localizer.md) | [`localization/localizer.hpp`](../../include/shulib/localization/localizer.hpp) | Localizer — the fused field-frame estimate. |
| [Odometry](odometry.md) | [`localization/odometry.hpp`](../../include/shulib/localization/odometry.hpp) | IOdometry — the dead-reckoning seam the Localizer predicts from. |
| [Override walls](override_walls.md) | [`localization/override_walls.hpp`](../../include/shulib/localization/override_walls.hpp) | The 2026-27 season's field walls for the wall reset — VEX V5RC "Override". |
| [Pilons odometry](pilons_odometry.md) | [`localization/pilons_odometry.hpp`](../../include/shulib/localization/pilons_odometry.hpp) | PilonsOdometry — tracking-wheel dead-reckoning. |
| [Tag map](tag_map.md) | [`localization/tag_map.hpp`](../../include/shulib/localization/tag_map.hpp) | TagMap — where the AprilTags are on the field, and where each of those numbers CAME FROM. |
| [Tracking wheel](tracking_wheel.md) | [`localization/tracking_wheel.hpp`](../../include/shulib/localization/tracking_wheel.hpp) | TrackingWheel — one unpowered odometry wheel: an `IRotation` sensor + the wheel's diameter + its mounting offset from the tracking center. |
| [Wall reset](wall_reset.md) | [`localization/wall_reset.hpp`](../../include/shulib/localization/wall_reset.hpp) | The WALL RESET's pure core (chunk L6-A1, 2026-10-29; the L6 brief, the team lead's answers of 2026-10-22): the geometry that turns one side distance sensor's reading into ONE field coordinate, the gates that REFUSE rather than guess, the verdict, and the on… |

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
| [Crc32](crc32.md) | [`diag/crc32.hpp`](../../include/shulib/diag/crc32.hpp) | CRC-32/IEEE — the one checksum in shulib, in the one place both of its callers can reach. |
| [Debug record](debug_record.md) | [`diag/debug_record.hpp`](../../include/shulib/diag/debug_record.hpp) | DebugRecord — the per-tick snapshot schema. |
| [Fault](fault.md) | [`diag/fault.hpp`](../../include/shulib/diag/fault.hpp) | Fault discipline (master plan §18.4; WS13, chunk A1) — the stable numeric fault-code enum and the latched first-fault capture. |
| [Finite guard](finite_guard.md) | [`diag/finite_guard.hpp`](../../include/shulib/diag/finite_guard.hpp) | Finite-value invariant guards (master plan §18.4) — the LOG-AND-RECOVER counterpart to SHULIB_PRECONDITION's throw. |
| [Health monitor](health_monitor.md) | [`diag/health_monitor.hpp`](../../include/shulib/diag/health_monitor.hpp) | HealthMonitor — sensor/power pathology → FaultCode, edge-triggered. |
| [Level filter sink](level_filter_sink.md) | [`diag/level_filter_sink.hpp`](../../include/shulib/diag/level_filter_sink.hpp) | LevelFilterSink — per-subsystem log levels. |
| [Line format](line_format.md) | [`diag/line_format.hpp`](../../include/shulib/diag/line_format.hpp) | line_format — the ONE set of §18.3 text-formatting primitives. |
| [Log copy sink](log_copy_sink.md) | [`diag/log_copy_sink.hpp`](../../include/shulib/diag/log_copy_sink.hpp) | LogCopySink — a COPY of a sink's leveled messages and run summaries in a second sink, and nothing else (chunk T4 Part 3, the team lead's decisions N5 (a) and A (a), 2026-09-27/28). |
| [Loop monitor](loop_monitor.md) | [`diag/loop_monitor.hpp`](../../include/shulib/diag/loop_monitor.hpp) | LoopMonitor — loop-overrun / tick-timing detection. |
| [Motion result](motion_result.md) | [`diag/motion_result.hpp`](../../include/shulib/diag/motion_result.hpp) | MotionResult — the per-motion result line, as data + one formatter. |
| [Odom tick audit](odom_tick_audit.md) | [`diag/odom_tick_audit.hpp`](../../include/shulib/diag/odom_tick_audit.hpp) | OdomTickAudit — the odometry trust gate's account of ONE tick: which of its checks tripped and the values it tripped on. |
| [Plausibility guard](plausibility_guard.md) | [`diag/plausibility_guard.hpp`](../../include/shulib/diag/plausibility_guard.hpp) | Physical-plausibility invariants (diagnostics-plan D-5; WS13, chunk C5) — FiniteGuard's log-and-recover posture, extended beyond finiteness. |
| [Rate limit sink](rate_limit_sink.md) | [`diag/rate_limit_sink.hpp`](../../include/shulib/diag/rate_limit_sink.hpp) | RateLimitedSink — per-channel rate limiting with COUNTED, REPORTED drops. |
| [Run summary](run_summary.md) | [`diag/run_summary.hpp`](../../include/shulib/diag/run_summary.hpp) | RunSummary — the end-of-run one-screen summary, as DATA. |
| [SD sink](sd_sink.md) | [`diag/sd_sink.hpp`](../../include/shulib/diag/sd_sink.hpp) | SdSink — the BLACKBOX: a binary, versioned, session-stamped record of a run, written to the brain's SD card. |
| [Session info](session_info.md) | [`diag/session_info.hpp`](../../include/shulib/diag/session_info.hpp) | SessionInfo + the §18.5 session header — provenance as the FIRST lines of every run. |
| [Shul2 format](shul2_format.md) | [`diag/shul2_format.hpp`](../../include/shulib/diag/shul2_format.hpp) | SHUL/2 — the telemetry WIRE, format v1.0: DebugRecord streamed, sequenced, checked and versioned. |
| [Shul2 reader](shul2_reader.md) | [`diag/shul2_reader.hpp`](../../include/shulib/diag/shul2_reader.hpp) | Shul2Reader — the SHUL/2 reader over a WHOLE capture. |
| [Term sink](term_sink.md) | [`diag/term_sink.hpp`](../../include/shulib/diag/term_sink.hpp) | TermSink — the human-readable terminal stream, the PRIMARY dev/debug surface. |
| [Tick attribution](tick_attribution.md) | [`diag/tick_attribution.hpp`](../../include/shulib/diag/tick_attribution.hpp) | TickAttribution — WHO consumed the loop budget. |
| [Tick timing stats](tick_timing_stats.md) | [`diag/tick_timing_stats.hpp`](../../include/shulib/diag/tick_timing_stats.hpp) | TickTimingStats — the statistics of a loop's tick-to-tick dt, as DATA. |
| [Trace](trace.md) | [`diag/trace.hpp`](../../include/shulib/diag/trace.hpp) | SHULIB_TRACE — the compile-time TRACE strip. |
| [Triage](triage.md) | [`diag/triage.hpp`](../../include/shulib/diag/triage.hpp) | The D-7 TRIAGE BLOCK — "why did it break", rendered for a human. |

### Math and frames

| Page | Header | What it is |
|---|---|---|
| [Angle](angle.md) | [`math/angle.hpp`](../../include/shulib/math/angle.hpp) | Angle — a heading on SE(2). The one type that owns angle wrapping, so the "degrees into cos/sin" and "359° vs -1°" bug classes are impossible by construction. |
| [Frame](frame.md) | [`math/frame.hpp`](../../include/shulib/math/frame.hpp) | frame.hpp — THE ONE PLACE a frame rotation is allowed. |
| [Pose2d](pose2d.md) | [`math/pose2d.hpp`](../../include/shulib/math/pose2d.hpp) | Pose2d — a rigid-body pose on SE(2): position (x, y) + heading. |
| [Reflection](reflection.md) | [`math/reflection.hpp`](../../include/shulib/math/reflection.hpp) | math::Reflection — the field plane reflected across a line through the origin, or left as it is. |
| [Twist2d](twist2d.md) | [`math/twist2d.hpp`](../../include/shulib/math/twist2d.hpp) | Twist2d and ChassisSpeeds — the velocity currencies of the motion stack. |

### Units

| Page | Header | What it is |
|---|---|---|
| [Literals](literals.md) | [`units/literals.hpp`](../../include/shulib/units/literals.hpp) | User-defined literals for shulib units. Each converts to CANONICAL units at the point of writing, so the rest of the code never sees a raw unit again: distance = 24_in; field = 1_tile; dt = 20_ms; v = 12_volt;. |
| [Quantity](quantity.md) | [`units/quantity.hpp`](../../include/shulib/units/quantity.hpp) | Quantity<L, A, T, E, I> — compile-time dimensional analysis. |

### HAL — the hardware seams

| Page | Header | What it is |
|---|---|---|
| [Absent GPS](absent_gps.md) | [`hal/absent_gps.hpp`](../../include/shulib/hal/absent_gps.hpp) | AbsentGps — the explicit statement that THIS ROBOT HAS NO GPS, as an IGps. |
| [Absent tag source](absent_tag_source.md) | [`hal/absent_tag_source.hpp`](../../include/shulib/hal/absent_tag_source.hpp) | AbsentTagSource — the explicit statement that THIS ROBOT HAS NO APRILTAG SOURCE (no AI Vision sensor, no coprocessor), as an ITagSource. |
| [Absent vision](absent_vision.md) | [`hal/absent_vision.hpp`](../../include/shulib/hal/absent_vision.hpp) | AbsentVision — the explicit statement that THIS ROBOT HAS NO OBJECT/COLOR DETECTION SOURCE, as an IVision. |
| [Battery](battery.md) | [`hal/battery.hpp`](../../include/shulib/hal/battery.hpp) | IBattery — the V5 battery (pros::battery) behind the HAL. |
| [Block char sink](block_char_sink.md) | [`hal/block_char_sink.hpp`](../../include/shulib/hal/block_char_sink.hpp) | BlockCharSink — an ICharSink over an IBlockSink: text lines onto a block device such as the SD card. |
| [Block sink](block_sink.md) | [`hal/block_sink.hpp`](../../include/shulib/hal/block_sink.hpp) | IBlockSink — where BINARY BLOCKS physically go (an SD-card file on the brain, a captured buffer in a test). |
| [Block source](block_source.md) | [`hal/block_source.hpp`](../../include/shulib/hal/block_source.hpp) | IBlockSource — where BINARY BLOCKS come FROM (an SD-card file on the brain, a canned buffer in a test). |
| [Brain screen](brain_screen.md) | [`hal/brain_screen.hpp`](../../include/shulib/hal/brain_screen.hpp) | IBrainScreen — the V5 BRAIN's own touch screen, as a seam. |
| [Char sink](char_sink.md) | [`hal/char_sink.hpp`](../../include/shulib/hal/char_sink.hpp) | ICharSink — where formatted diagnostic BYTES physically go (a terminal, a captured string in a test, later a serial port). |
| [Clock](clock.md) | [`hal/clock.hpp`](../../include/shulib/hal/clock.hpp) | IClock — the single source of "now" for the whole stack. |
| [Controller](controller.md) | [`hal/controller.hpp`](../../include/shulib/hal/controller.hpp) | IController — the V5 game controller behind the HAL (chunk R1a, absorbing Phase T's T1): the INPUT half of driver control. |
| [Controller conversion](controller_conversion.md) | [`hal/controller_conversion.hpp`](../../include/shulib/hal/controller_conversion.hpp) | Controller canonical conversions — the ONE place the V5 controller's raw stick range becomes shulib's canonical [-1, 1] (§7: "convert exactly once, at the edge"). |
| [Device provider](device_provider.md) | [`hal/device_provider.hpp`](../../include/shulib/hal/device_provider.hpp) | IDeviceProvider — devices BY PORT, and the one target-specific seam a builder needs. |
| [Device registry](device_registry.md) | [`hal/device_registry.hpp`](../../include/shulib/hal/device_registry.hpp) | IDeviceRegistry — WHAT IS ACTUALLY PLUGGED INTO EACH SMART PORT. |
| [Digital in](digital_in.md) | [`hal/digital_in.hpp`](../../include/shulib/hal/digital_in.hpp) | IDigitalIn — a single digital input line behind the HAL (chunk R1b): a limit switch, a bumper, a jumper — any two-state sensor on an ADI port. |
| [Digital out](digital_out.md) | [`hal/digital_out.hpp`](../../include/shulib/hal/digital_out.hpp) | IDigitalOut — a single digital output line behind the HAL (chunk F1, WS7/M4): a pneumatic solenoid on the ADI ports, or any other two-state actuator. |
| [Distance](distance.md) | [`hal/distance.hpp`](../../include/shulib/hal/distance.hpp) | IDistance — a distance / time-of-flight sensor (pros::Distance) behind the HAL. |
| [Distance conversion](distance_conversion.md) | [`hal/distance_conversion.hpp`](../../include/shulib/hal/distance_conversion.hpp) | Distance-sensor canonical conversions — the ONE place the V5 distance sensor's millimeters become shulib's canonical inches and its raw 0–63 confidence becomes [0, 1] (§7: "convert exactly once, at the edge"). |
| [Drive geometry](drive_geometry.md) | [`hal/drive_geometry.hpp`](../../include/shulib/hal/drive_geometry.hpp) | DriveGeometry — ONE description of a drive side's geometry, consumed by BOTH the drive-encoder odometry and the stall cross-check. |
| [Drivetrain description](drivetrain_description.md) | [`hal/drivetrain_description.hpp`](../../include/shulib/hal/drivetrain_description.hpp) | DrivetrainDescription — the ONE representation of a drivetrain's discovered facts. |
| [File store](file_store.md) | [`hal/file_store.hpp`](../../include/shulib/hal/file_store.hpp) | IFileStore — the two things the brain's SD card offers that a SAFE save can be built on (chunk SAFE-SAVE Part A, 2026-09-29): read a whole named file, or replace one by truncating it. |
| [GPS](gps.md) | [`hal/gps.hpp`](../../include/shulib/hal/gps.hpp) | IGps — the VEX GPS behind the HAL, reporting in shulib's CANONICAL frame (the VEX meters / clockwise-from-North convention is converted away in the hal/pros adapter via gps_conversion.hpp). |
| [GPS conversion](gps_conversion.md) | [`hal/gps_conversion.hpp`](../../include/shulib/hal/gps_conversion.hpp) | GPS canonical conversions — the ONE place the VEX GPS frame becomes shulib's canonical frame (§7: "convert exactly once, at the edge"). |
| [IMU](imu.md) | [`hal/imu.hpp`](../../include/shulib/hal/imu.hpp) | IImu — the inertial sensor behind the HAL, reporting in shulib's CANONICAL frame (the V5's clockwise/degrees convention is converted away in the hal/pros adapter via imu_conversion.hpp, so everything above this line is CCW-positive radians). |
| [IMU conversion](imu_conversion.md) | [`hal/imu_conversion.hpp`](../../include/shulib/hal/imu_conversion.hpp) | IMU canonical conversions — the ONE place the V5 inertial sensor's frame becomes shulib's canonical frame (§7: "convert exactly once, at the edge"). |
| [Line display](line_display.md) | [`hal/line_display.hpp`](../../include/shulib/hal/line_display.hpp) | ILineDisplay — where short status LINES physically go (the V5 controller's LCD; a captured fake in tests). |
| [Mechanism](mechanism.md) | [`hal/mechanism.hpp`](../../include/shulib/hal/mechanism.hpp) | The mechanism device seam (chunk F1, WS7/M4): IMechanism + the two concrete compositions every VEX mechanism reduces to at the device level — a group of motors on one shaft (MotorMechanism) and a set of digital lines switching one pneumatic circuit (Pneumat… |
| [Motor](motor.md) | [`hal/motor.hpp`](../../include/shulib/hal/motor.hpp) | IMotor — a single V5 smart motor behind the HAL. |
| [Motor conversion](motor_conversion.md) | [`hal/motor_conversion.hpp`](../../include/shulib/hal/motor_conversion.hpp) | Motor canonical conversions — the ONE place the V5 smart motor's units become shulib's canonical units (§7: "convert exactly once, at the edge"). |
| [Motor current limit](motor_current_limit.md) | [`hal/motor_current_limit.hpp`](../../include/shulib/hal/motor_current_limit.hpp) | IMotorCurrentLimit — a V5 motor's OWN current limit, set and read back (chunk MECH-S-A, 2026-11-05; the MECH-S brief's D1 (a)). |
| [Motor group](motor_group.md) | [`hal/motor_group.hpp`](../../include/shulib/hal/motor_group.hpp) | MotorGroup — N physically COUPLED motors behind ONE IMotor. |
| [Null sink](null_sink.md) | [`hal/null_sink.hpp`](../../include/shulib/hal/null_sink.hpp) | NullSink — the zero-cost default ITelemetrySink (§18.1). |
| [Optical](optical.md) | [`hal/optical.hpp`](../../include/shulib/hal/optical.hpp) | IOptical — a color / optical sensor (pros::Optical) behind the HAL. |
| [Optical conversion](optical_conversion.md) | [`hal/optical_conversion.hpp`](../../include/shulib/hal/optical_conversion.hpp) | Optical-sensor canonical conversions — the ONE place the V5 optical sensor's raw channels become shulib's canonical ranges (§7: "convert exactly once, at the edge"). |
| [Read faults](read_faults.md) | [`hal/read_faults.hpp`](../../include/shulib/hal/read_faults.hpp) | IReadFaults — how many reads a SENSOR ADAPTER has SCREENED. |
| [Rotation](rotation.md) | [`hal/rotation.hpp`](../../include/shulib/hal/rotation.hpp) | IRotation — a rotation / tracking-wheel sensor (pros::Rotation) behind the HAL. |
| [Rotation conversion](rotation_conversion.md) | [`hal/rotation_conversion.hpp`](../../include/shulib/hal/rotation_conversion.hpp) | Rotation-sensor canonical conversions — the ONE place the V5 rotation sensor's centidegrees become shulib's canonical radians (§7: "convert exactly once, at the edge"). |
| [Telemetry sink](telemetry_sink.md) | [`hal/telemetry_sink.hpp`](../../include/shulib/hal/telemetry_sink.hpp) | ITelemetrySink — the diagnostics output seam. |
| [Vision](vision.md) | [`hal/vision.hpp`](../../include/shulib/hal/vision.hpp) | IVision / ITagSource — the AI Vision seams. |
| [Vision conversion](vision_conversion.md) | [`hal/vision_conversion.hpp`](../../include/shulib/hal/vision_conversion.hpp) | vision_conversion.hpp — the ONE place raw AprilTag image corners become shulib's canonical robot-relative tag pose (§7: convert exactly once, at the edge). |
| [Vision frames](vision_frames.md) | [`hal/vision_frames.hpp`](../../include/shulib/hal/vision_frames.hpp) | IVisionFrames — the V5 AI Vision camera read a FRAME at a time. |

### HAL — the PROS adapters

| Page | Header | What it is |
|---|---|---|
| [Ai vision (PROS)](pros-ai_vision.md) | [`hal/pros/ai_vision.hpp`](../../include/shulib/hal/pros/ai_vision.hpp) | ProsAiVision — hal::IVisionFrames (and hal::IReadFaults, MECH-A1's channel) over the vendored PROS 4.2.2 AI Vision C API, include/pros/ai_vision.h. |
| [Battery (PROS)](pros-battery.md) | [`hal/pros/battery.hpp`](../../include/shulib/hal/pros/battery.hpp) | ProsBattery — IBattery over the pros::battery namespace (chunk R1a): the brownout-compensation input behind the HAL. |
| [Block sink (PROS)](pros-block_sink.md) | [`hal/pros/block_sink.hpp`](../../include/shulib/hal/pros/block_sink.hpp) | ProsBlockSink — IBlockSink over a PROS FILE* on the V5's SD card (chunk R1b): where the blackbox's binary blocks physically go. |
| [Block source (PROS)](pros-block_source.md) | [`hal/pros/block_source.hpp`](../../include/shulib/hal/pros/block_source.hpp) | ProsBlockSource — IBlockSource over a PROS FILE* on the V5's SD card (chunk R3d Part 2): where the calibration record is read back from. |
| [Brain screen (PROS)](pros-brain_screen.md) | [`hal/pros/brain_screen.hpp`](../../include/shulib/hal/pros/brain_screen.hpp) | ProsBrainScreen — hal::IBrainScreen over PROS's screen API. |
| [Calibrate (PROS)](pros-calibrate.md) | [`hal/pros/calibrate.hpp`](../../include/shulib/hal/pros/calibrate.hpp) | hal/pros/calibrate — the standard "shulib Calibrate" on a V5 brain (chunk CAL1, Part A2a, 2026-10-18): the THIN shell that hands calibration::CalibrateProgram the robot's real devices. |
| [Char sink (PROS)](pros-char_sink.md) | [`hal/pros/char_sink.hpp`](../../include/shulib/hal/pros/char_sink.hpp) | ProsCharSink, serialLine() and serialWrapped() — the V5's USB serial, where TermSink's diagnostic bytes and every program's own lines physically go (`pros terminal` displays them). |
| [Clock (PROS)](pros-clock.md) | [`hal/pros/clock.hpp`](../../include/shulib/hal/pros/clock.hpp) | ProsClock — IClock over the V5's real time. |
| [Controller (PROS)](pros-controller.md) | [`hal/pros/controller.hpp`](../../include/shulib/hal/pros/controller.hpp) | ProsController — IController over pros::Controller (chunk R1a): the driver's hands behind the HAL. |
| [Device provider (PROS)](pros-device_provider.md) | [`hal/pros/device_provider.hpp`](../../include/shulib/hal/pros/device_provider.hpp) | ProsDeviceProvider — hal::IDeviceProvider over the real V5 adapters. |
| [Device registry (PROS)](pros-device_registry.md) | [`hal/pros/device_registry.hpp`](../../include/shulib/hal/pros/device_registry.hpp) | ProsDeviceRegistry — IDeviceRegistry over PROS's smart-port registry. |
| [Digital in (PROS)](pros-digital_in.md) | [`hal/pros/digital_in.hpp`](../../include/shulib/hal/pros/digital_in.hpp) | ProsDigitalIn — IDigitalIn over pros::adi::DigitalIn (chunk R1b): a limit switch / bumper line behind the HAL. |
| [Digital out (PROS)](pros-digital_out.md) | [`hal/pros/digital_out.hpp`](../../include/shulib/hal/pros/digital_out.hpp) | ProsDigitalOut — IDigitalOut over pros::adi::DigitalOut (chunk R1b): the pneumatic solenoid line behind the HAL (F1's seam, finally on hardware). |
| [Distance (PROS)](pros-distance.md) | [`hal/pros/distance.hpp`](../../include/shulib/hal/pros/distance.hpp) | ProsDistance — IDistance over pros::Distance (chunk R1b): the capture/dock-confirm rangefinder behind the HAL. |
| [File store (PROS)](pros-file_store.md) | [`hal/pros/file_store.hpp`](../../include/shulib/hal/pros/file_store.hpp) | ProsFileStore — hal::IFileStore on the V5's SD card (chunk SAFE-SAVE Part A, 2026-09-29): the two operations the safe save builds on — read a file whole, replace it by truncating it — composed from the two adapters that already own the card, one adapter per… |
| [GPS (PROS)](pros-gps.md) | [`hal/pros/gps.hpp`](../../include/shulib/hal/pros/gps.hpp) | ProsGps — IGps over pros::Gps (chunk R1a): the absolute-position corrector's sensor behind the HAL. |
| [IMU (PROS)](pros-imu.md) | [`hal/pros/imu.hpp`](../../include/shulib/hal/pros/imu.hpp) | ProsImu — IImu over pros::Imu (chunk R1a): the load-bearing < 1° heading source behind the HAL. |
| [Line display (PROS)](pros-line_display.md) | [`hal/pros/line_display.hpp`](../../include/shulib/hal/pros/line_display.hpp) | ProsLineDisplay — ILineDisplay over the V5 controller's LCD (chunk R1a): where the D-4 status rows physically go. |
| [Motor (PROS)](pros-motor.md) | [`hal/pros/motor.hpp`](../../include/shulib/hal/pros/motor.hpp) | ProsMotor — IMotor over pros::Motor. |
| [Optical (PROS)](pros-optical.md) | [`hal/pros/optical.hpp`](../../include/shulib/hal/pros/optical.hpp) | ProsOptical — IOptical over pros::Optical (chunk R1b): the game-object color/proximity confirmation sensor behind the HAL. |
| [Rotation (PROS)](pros-rotation.md) | [`hal/pros/rotation.hpp`](../../include/shulib/hal/pros/rotation.hpp) | ProsRotation — IRotation over pros::Rotation (chunk R1a): the tracking-wheel pods behind the HAL. |
| [Tick pacer (PROS)](pros-tick_pacer.md) | [`hal/pros/tick_pacer.hpp`](../../include/shulib/hal/pros/tick_pacer.hpp) | ProsTickPacer — motion::ITickPacer over pros::Task::delay_until (chunk R1a): the ONLY seam that regains control mid-motion on the robot, replacing main.cpp's V5DelayPacer (which had to hand-advance a FakeClock). |
| [Tune (PROS)](pros-tune.md) | [`hal/pros/tune.hpp`](../../include/shulib/hal/pros/tune.hpp) | hal/pros/tune — "shulib Tune" on a V5 brain (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's D7 (a), §1.3): the THIN shell that hands mechanism::TuneProgram the robot's real devices. |

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

### Calibration

| Page | Header | What it is |
|---|---|---|
| [Boot check](boot_check.md) | [`calibration/boot_check.hpp`](../../include/shulib/calibration/boot_check.hpp) | THE BOOT CHECK — is this description still true of THIS robot? |
| [Calibrate panel](calibrate_panel.md) | [`calibration/calibrate_panel.hpp`](../../include/shulib/calibration/calibrate_panel.hpp) | calibrate_panel — the calibration screen as a PURE MODEL. |
| [Calibrate program](calibrate_program.md) | [`calibration/calibrate_program.hpp`](../../include/shulib/calibration/calibrate_program.hpp) | calibrate_program — THE STANDARD "shulib Calibrate" (chunk CAL1, Part A2a, 2026-10-18): the robot measures its own drivetrain, for ANY team, from the few facts only a person can state. |
| [Calibrate screen](calibrate_screen.md) | [`calibration/calibrate_screen.hpp`](../../include/shulib/calibration/calibrate_screen.hpp) | calibrate_screen — how the standard Calibrate PAINTS the brain's screen. |
| [Calibration record](calibration_record.md) | [`calibration/calibration_record.hpp`](../../include/shulib/calibration/calibration_record.hpp) | The CALIBRATION RECORD — a hal::DrivetrainDescription as bytes on the SD card. |
| [Drive characterization](drive_characterization.md) | [`calibration/drive_characterization.hpp`](../../include/shulib/calibration/drive_characterization.hpp) | DriveCharacterization — what the GAINS and REST steps of shulib Calibrate MEASURE, as data. |
| [Drive characterizer](drive_characterizer.md) | [`calibration/drive_characterizer.hpp`](../../include/shulib/calibration/drive_characterizer.hpp) | DriveCharacterizer — shulib Calibrate's GAINS step: the robot drives itself through four short, bounded segments while each drive side's kS, kV and kA are fitted on the brain. |
| [Drivetrain calibrator](drivetrain_calibrator.md) | [`calibration/drivetrain_calibrator.hpp`](../../include/shulib/calibration/drivetrain_calibrator.hpp) | DrivetrainCalibrator — the robot describes its own drivetrain. |
| [Feedforward fit](feedforward_fit.md) | [`calibration/feedforward_fit.hpp`](../../include/shulib/calibration/feedforward_fit.hpp) | FeedforwardFit — kS, kV and kA of ONE drive side, fitted on the brain from that side's own run. |
| [Front check](front_check.md) | [`calibration/front_check.hpp`](../../include/shulib/calibration/front_check.hpp) | front_check — did a new calibration put the robot's FRONT at the other end? |
| [Record store](record_store.md) | [`calibration/record_store.hpp`](../../include/shulib/calibration/record_store.hpp) | The calibration record's SAFE SAVE and its LOAD (chunk SAFE-SAVE Part A, 2026-09-29): a save designed so a failure part-way does not leave the card without a good record — given HA-166 (4) and (5), short of a stated two-fault limit — a load that falls back… |
| [Rest capture](rest_capture.md) | [`calibration/rest_capture.hpp`](../../include/shulib/calibration/rest_capture.hpp) | RestCapture — shulib Calibrate's REST step: the robot sits still, unpowered, for a minute while the IMU's drift and noise, the encoders' noise at rest and the loop's timing are measured. |

### Competition

| Page | Header | What it is |
|---|---|---|
| [Acceptance run](acceptance_run.md) | [`competition/acceptance_run.hpp`](../../include/shulib/competition/acceptance_run.hpp) | competition::acceptanceRun — the LIBRARY's acceptance run: the one routine shulib ships. |
| [Auton menu](auton_menu.md) | [`competition/auton_menu.hpp`](../../include/shulib/competition/auton_menu.hpp) | competition::AutonMenu — the brain-screen auton selector as a PURE MODEL. |
| [Auton registry](auton_registry.md) | [`competition/auton_registry.hpp`](../../include/shulib/competition/auton_registry.hpp) | competition::AutonRegistry — the autons a competition program can run, as DATA. |
| [Auton wall reset](auton_wall_reset.md) | [`competition/auton_wall_reset.hpp`](../../include/shulib/competition/auton_wall_reset.hpp) | competition::resetFromWall — the wall reset's EVERYDAY call, over the auton's context (chunk L6-A2, 2026-11-03; the L6 brief's D4 (a) and its §1.12 — the team lead's answers of 2026-10-22, and his ruling of 2026-11-03 that the wall reset be callable from a… |
| [Field mirror](field_mirror.md) | [`competition/field_mirror.hpp`](../../include/shulib/competition/field_mirror.hpp) | competition::FieldMirror — ONE ROUTE, TWO ROBOTS. |
| [Match runner](match_runner.md) | [`competition/match_runner.hpp`](../../include/shulib/competition/match_runner.hpp) | competition::runAuton — run ONE registered auton under a RunGuard, from its start pose to its guaranteed end. |
| [Override mirror](override_mirror.md) | [`competition/override_mirror.hpp`](../../include/shulib/competition/override_mirror.hpp) | The 2026-27 season's SKILLS MIRROR — VEX V5RC / VEX U "Override". |
| [Toggle visit](toggle_visit.md) | [`competition/toggle_visit.hpp`](../../include/shulib/competition/toggle_visit.hpp) | competition::checkAndFlipToggle — the Toggle flip for a SKILLS run, as ONE route step. |

### Config

| Page | Header | What it is |
|---|---|---|
| [Camera config](camera_config.md) | [`config/camera_config.hpp`](../../include/shulib/config/camera_config.hpp) | CameraConfig — the robot file's AI VISION CAMERA block. |
| [Robot config](robot_config.md) | [`config/robot_config.hpp`](../../include/shulib/config/robot_config.hpp) | RobotConfig — a whole robot, described as DATA. |
| [Route](route.md) | [`config/route.hpp`](../../include/shulib/config/route.hpp) | Route — an autonomous routine as DATA. |
| [Wall sensor config](wall_sensor_config.md) | [`config/wall_sensor_config.hpp`](../../include/shulib/config/wall_sensor_config.hpp) | WallSensorsConfig — the robot file's SIDE DISTANCE SENSORS block. |

### Mechanism

| Page | Header | What it is |
|---|---|---|
| [Devices](devices.md) | [`mechanism/devices.hpp`](../../include/shulib/mechanism/devices.hpp) | mechanism/devices.hpp — the DEVICES one mechanism was built over, and what the build found for each of its ROLES. |
| [Direction check](direction_check.md) | [`mechanism/direction_check.hpp`](../../include/shulib/mechanism/direction_check.hpp) | mechanism/direction_check.hpp — THE DIRECTION CHECK's judgement. |
| [Driver inputs](driver_inputs.md) | [`mechanism/driver_inputs.hpp`](../../include/shulib/mechanism/driver_inputs.hpp) | mechanism/driver_inputs.hpp — THE DRIVER FILE'S MECHANISM CONTROLS: hold-to-run, toggle, preset and the homing controls, each naming its mechanism by its DECLARATION, checked at build against the robot's list, and run THROUGH the mechanism service. |
| [Kinds](kinds.md) | [`mechanism/kinds.hpp`](../../include/shulib/mechanism/kinds.hpp) | mechanism/kinds.hpp — WHAT A MECHANISM IS, the way a team writes it. |
| [Live tuning](live_tuning.md) | [`mechanism/live_tuning.hpp`](../../include/shulib/mechanism/live_tuning.hpp) | mechanism/live_tuning.hpp — LIVE TUNING's library half: a mechanism's numbers changed WHILE THE ROBOT RUNS, inside the same rules as the build, undone, and copied back out as the lines its robot file needs (chunk TUNE1-A, 2026-11-10; the MECH-S/TUNE1 brief'… |
| [Route ids](route_ids.md) | [`mechanism/route_ids.hpp`](../../include/shulib/mechanism/route_ids.hpp) | mechanism/route_ids.hpp — EVERY MECHANISM'S ACTIONS AS ROUTE IDS, automatically (the mechanism core, chunk MECH-A1, 2026-10-08; the team lead's ruling of 2026-10-08, MECH-PANEL-requirements §4 item 7b: "an arm's presets as arm_stow, arm_score; an intake's i… |
| [Service](service.md) | [`mechanism/service.hpp`](../../include/shulib/mechanism/service.hpp) | mechanism/service.hpp — THE MECHANISM SERVICE: one object, ticked once per control tick, that runs every mechanism's behaviour. |
| [Spec](spec.md) | [`mechanism/spec.hpp`](../../include/shulib/mechanism/spec.hpp) | mechanism/spec.hpp — ONE MECHANISM AS DATA, and THE BUILD CHECK. |
| [Tune panel](tune_panel.md) | [`mechanism/tune_panel.hpp`](../../include/shulib/mechanism/tune_panel.hpp) | mechanism/tune_panel.hpp — "shulib Tune"'s brain screen as a PURE MODEL: what each page says, where each button is, which touches mean something (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's D7 (a) and D8 (a), with its re-check amendments #3, #7, #8… |
| [Tune program](tune_program.md) | [`mechanism/tune_program.hpp`](../../include/shulib/mechanism/tune_program.hpp) | mechanism/tune_program.hpp — "shulib Tune", the program that tunes a robot's MECHANISMS on the brain (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's D7 (a) and D8 (a), with its re-check amendments #3, #7, #8, #10 and #14). |
| [Until](until.md) | [`mechanism/until.hpp`](../../include/shulib/mechanism/until.hpp) | mechanism/until.hpp — WHEN A STEP IS DONE: a power step's done-conditions. |
| [Wiring](wiring.md) | [`mechanism/wiring.hpp`](../../include/shulib/mechanism/wiring.hpp) | mechanism/wiring.hpp — THE BINDING (the mechanism core, chunk MECH-A1, 2026-10-08): which port each ROLE is plugged into, a block of the robot file SEPARATE from the declarations, compiled in (P2; the team lead's Q1 (a): the binding lives only in the robot… |

### Robot

| Page | Header | What it is |
|---|---|---|
| [Mechanism parts](mechanism_parts.md) | [`robot/mechanism_parts.hpp`](../../include/shulib/robot/mechanism_parts.hpp) | robot::MechanismParts — every ROLE of every mechanism, built from the config over a device provider. |
| [Robot](robot.md) | [`robot/robot.hpp`](../../include/shulib/robot/robot.hpp) | robot::Robot — a config::RobotConfig, wired. |

### Runner

| Page | Header | What it is |
|---|---|---|
| [Command registry](command_registry.md) | [`runner/command_registry.hpp`](../../include/shulib/runner/command_registry.hpp) | CommandRegistry — the binding from a marker's ID to the code that performs it. |
| [Manifest](manifest.md) | [`runner/manifest.hpp`](../../include/shulib/runner/manifest.hpp) | writeManifest — the command-id vocabulary, published. |
| [Mechanism routes](mechanism_routes.md) | [`runner/mechanism_routes.hpp`](../../include/shulib/runner/mechanism_routes.hpp) | runner/mechanism_routes.hpp — a robot's MECHANISM ROUTE IDS (mechanism/route_ids.hpp), bound to its mechanism service and registered in a route registry in ONE call (the mechanism core, chunk MECH-A4, 2026-10-22; the team lead's ruling of 2026-10-08: "every… |
| [Path runner](path_runner.md) | [`runner/path_runner.hpp`](../../include/shulib/runner/path_runner.hpp) | PathRunner — the engine that EXECUTES a Route. |

### Teleop

| Page | Header | What it is |
|---|---|---|
| [Coupled side monitor](coupled_side_monitor.md) | [`teleop/coupled_side_monitor.hpp`](../../include/shulib/teleop/coupled_side_monitor.hpp) | Coupled-side monitor — the fighting-motor detector for a drivetrain side whose motors are mechanically coupled through one gear train (chunk R3b Part 0b, 2026-09-10), plus the small pure pieces the drive loops need: the per-side arcade arithmetic, and the t… |
| [Drive modes](drive_modes.md) | [`teleop/drive_modes.hpp`](../../include/shulib/teleop/drive_modes.hpp) | Drive modes — the pure mappings from a driver's four stick axes to ONE body-frame DriveRequest. |
| [Drive protection](drive_protection.md) | [`teleop/drive_protection.hpp`](../../include/shulib/teleop/drive_protection.hpp) | DriveProtection — the old "shulib Drive"'s drive protections, in the library's driver loop, for every program that drives through it. |
| [Driver control](driver_control.md) | [`teleop/driver_control.hpp`](../../include/shulib/teleop/driver_control.hpp) | DriveShaper — the whole feel layer in one object and one call per tick. |
| [Drivetrain degradation](drivetrain_degradation.md) | [`teleop/drivetrain_degradation.hpp`](../../include/shulib/teleop/drivetrain_degradation.hpp) | Drivetrain degradation policy — what a drive program does when some of the motors it was built for do not answer. |
| [Field centric](field_centric.md) | [`teleop/field_centric.hpp`](../../include/shulib/teleop/field_centric.hpp) | Field-centric driving, and the honest failure mode without which it must not ship. |
| [Input shaping](input_shaping.md) | [`teleop/input_shaping.hpp`](../../include/shulib/teleop/input_shaping.hpp) | Input shaping — what a driver's raw stick deflection becomes before it is a command. |
| [Mechanism binding](mechanism_binding.md) | [`teleop/mechanism_binding.hpp`](../../include/shulib/teleop/mechanism_binding.hpp) | MechanismBinding — two controller buttons driving one motor mechanism, hold-to-run. |
| [Slew limiter](slew_limiter.md) | [`teleop/slew_limiter.hpp`](../../include/shulib/teleop/slew_limiter.hpp) | SlewLimiter — the optional rate limit on ONE driver axis. |
| [Stick mapping](stick_mapping.md) | [`teleop/stick_mapping.hpp`](../../include/shulib/teleop/stick_mapping.hpp) | Teleop stick mapping — the ONE place a driver's sticks become a body-frame drive request (chunk R3b, Session 2), and the DriveRequest/StickInput vocabulary the whole feel layer is written in. |
| [Teleop loop](teleop_loop.md) | [`teleop/teleop_loop.hpp`](../../include/shulib/teleop/teleop_loop.hpp) | runDriverControl — the library's driver-control loop. |

### Ui

| Page | Header | What it is |
|---|---|---|
| [Touch panel](touch_panel.md) | [`ui/touch_panel.hpp`](../../include/shulib/ui/touch_panel.hpp) | ui::touch_panel — the GENERIC pieces of a brain-screen panel model. |

### Vision

| Page | Header | What it is |
|---|---|---|
| [Camera feed](camera_feed.md) | [`vision/camera_feed.hpp`](../../include/shulib/vision/camera_feed.hpp) | CameraFeed — ONE read of the camera per period, handed to every consumer. |
| [Camera geometry](camera_geometry.md) | [`vision/camera_geometry.hpp`](../../include/shulib/vision/camera_geometry.hpp) | Camera geometry — a detection's box turned into which way it is (BEARING), how high it looks (ELEVATION), how far it is (a GROUND range from where its base meets the floor, a SIZE range from its known real height) and where it is on the FIELD — every one an… |
| [Override elements](override_elements.md) | [`vision/override_elements.hpp`](../../include/shulib/vision/override_elements.hpp) | The 2026-27 Override season's facts the camera code needs — the Toggle's coloured faces, where the Toggles are, the Pin and the Cup, and the class table of VEX's Override AI model. |
| [Piece finder](piece_finder.md) | [`vision/piece_finder.hpp`](../../include/shulib/vision/piece_finder.hpp) | PieceFinder — "take THAT piece": a route names ONE game piece by its TYPE (the AI model's class) and its EXPECTED FIELD POSITION within a TOLERANCE, and the finder answers FOUND, NONE, AMBIGUOUS, NOT IN VIEW or UNKNOWN — NEVER A GUESS. |
| [Toggle tracker](toggle_tracker.md) | [`vision/toggle_tracker.hpp`](../../include/shulib/vision/toggle_tracker.hpp) | ToggleTracker — WHICH COLOUR does the Toggle in front of the robot show: RED, BLUE, YELLOW, or UNKNOWN with the reason. |

## Every public entity, alphabetically

**[The alphabetical index](all-entities.md)** lists all 6,440 of them with a link to each. Nested types appear under their qualified name (`BlackboxReader::Frame::type`), so a member of a nested type is findable by the name you would actually write.

## Where the other documents fit

- [How it works](../guide/README.md) teaches the ideas in order, and [The API in words](../guide/10-the-api.md) is the API *as prose* — when to reach for a verb, what it does when things go wrong, which gotchas bite. It deliberately does not restate signatures; this reference owns those.
- [Recipes](../cookbook/README.md) answers "how do I write the routine I am writing right now", with compiled recipes.
- This reference answers "what exactly exists, and what is its exact spelling".
