<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Regenerate: python3 tools/api_doc_tool.py generate -->

# API reference

Every public member of the two **routine-authoring** surfaces — `Chassis` and `Routine`, both frozen contracts — extracted from the headers. This page is generated, so it cannot fall behind the code: a member added to one of those headers appears here the next time the tool runs, and the host test build fails if it has not. **It is not the whole API** — see "What is not here" below.

**A member of those surfaces with no documentation comment fails the build**, naming itself. That gate is what makes "generated" mean "complete" rather than "generated from whatever someone remembered to write".

**What is not here, and why.** This page covers the two surfaces you write an autonomous *routine* against. shulib's public surface is far larger — roughly 160 public types across a dozen subsystems — and almost none of it is generated here.

Being frozen is *not* the selector, and it is worth saying so plainly: the coordinate frame, the units and angle semantics, the HAL interface signatures and the kinematics contract are all locked contracts too, and none of them is generated here either. The honest reason is narrower — the generator was pointed at the two types a routine author touches, and has not been pointed anywhere else yet.

Shipped, public, and absent:

- **The diagnostics and logging API** — `ITelemetrySink` and `LogLevel`; the sinks (`TermSink`, `SdSink`, `NullSink`, `LevelFilterSink`, `RateLimitedSink`); the records (`DebugRecord`, `MotionResult`, `RunSummary`, `SessionInfo`); the monitors (`FaultLatch`, `FaultCode`, `HealthMonitor`, `LoopMonitor`, `PoseDeltaGuard`); and the blackbox (`BlackboxReader` and its format). This is the largest omission by far, and [guide chapter 11](../guide/11-reading-the-diagnostics.md) is its real documentation — it teaches the transcript line by line, which is how you actually use this layer.
- **Localization** — `GpsCorrector`, `AprilTagCorrector`, `TagMap`, `EkfFusion`, `ComplementaryFusion`, `Localizer` ([chapter 3](../guide/03-knowing-where-you-are.md)).
- **The run guard** — `RunGuard` and its config/report types ([chapter 14](../guide/14-what-it-cannot-do-yet.md) states what "guaranteed" does and does not cover).
- **The seams** — mechanism, controller and digital-input ([chapter 13](../guide/13-extending-the-library.md)), plus the whole `hal/` interface set and its `hal/pros` adapters.

Each of those is documented **in its own header**, and the headers are written to be read — every one opens with why it exists, not just what it does. Until this page grows, that is where the rest of the API lives, and the [user guide](../guide/README.md) is the map to it.

Two real constraints shape when that changes. Adding a header here also puts it under the coverage gate, so **an undocumented public member starts failing the build** — that is the point of the gate, and it is also why expanding is a piece of work rather than a switch. And gating a seam that is deliberately *not* frozen would pin it in place, which is a way of freezing it by accident; the Freeze Register records which seams those are and why they are still open.

## Pages

- [`Chassis` — the frozen facade](chassis.md) — The public surface every autonomous routine is written against. Frozen as register row F6 on 2026-08-12: every member below changes only with a major API-version bump plus a migration note.
- [`Routine` — the recipe layer](routine.md) — The Tier-2 chain: a complete autonomous routine as a sequence of named steps, each delegating to exactly one `Chassis` verb.

## Every public member, alphabetically

| Member | Type | Page |
|---|---|---|
| `Chassis::brake` | `Chassis` | [chassis.md](chassis.md#chassis-brake) |
| `Chassis::cancel` | `Chassis` | [chassis.md](chassis.md#chassis-cancel) |
| `Chassis::Chassis` | `Chassis` | [chassis.md](chassis.md#chassis-chassis) |
| `Chassis::Chassis (overload 2)` | `Chassis` | [chassis.md](chassis.md#chassis-chassis-2) |
| `Chassis::Chassis (overload 3)` | `Chassis` | [chassis.md](chassis.md#chassis-chassis-3) |
| `Chassis::deps` | `Chassis` | [chassis.md](chassis.md#chassis-deps) |
| `Chassis::drive` | `Chassis` | [chassis.md](chassis.md#chassis-drive) |
| `Chassis::followTrajectory` | `Chassis` | [chassis.md](chassis.md#chassis-followtrajectory) |
| `Chassis::followTrajectory (overload 2)` | `Chassis` | [chassis.md](chassis.md#chassis-followtrajectory-2) |
| `Chassis::hold` | `Chassis` | [chassis.md](chassis.md#chassis-hold) |
| `Chassis::lastCompleted` | `Chassis` | [chassis.md](chassis.md#chassis-lastcompleted) |
| `Chassis::lastExitReason` | `Chassis` | [chassis.md](chassis.md#chassis-lastexitreason) |
| `Chassis::motionConfig` | `Chassis` | [chassis.md](chassis.md#chassis-motionconfig) |
| `Chassis::moveTo` | `Chassis` | [chassis.md](chassis.md#chassis-moveto) |
| `Chassis::operator=` | `Chassis` | [chassis.md](chassis.md#chassis-operator-assign) |
| `Chassis::operator= (overload 2)` | `Chassis` | [chassis.md](chassis.md#chassis-operator-assign-2) |
| `Chassis::pose` | `Chassis` | [chassis.md](chassis.md#chassis-pose) |
| `Chassis::scheduler` | `Chassis` | [chassis.md](chassis.md#chassis-scheduler) |
| `Chassis::scheduler (overload 2)` | `Chassis` | [chassis.md](chassis.md#chassis-scheduler-2) |
| `Chassis::setPose` | `Chassis` | [chassis.md](chassis.md#chassis-setpose) |
| `Chassis::strafeAuthority` | `Chassis` | [chassis.md](chassis.md#chassis-strafeauthority) |
| `Chassis::strafeTo` | `Chassis` | [chassis.md](chassis.md#chassis-strafeto) |
| `Chassis::turnTo` | `Chassis` | [chassis.md](chassis.md#chassis-turnto) |
| `Chassis::wait` | `Chassis` | [chassis.md](chassis.md#chassis-wait) |
| `Chassis::waitUntil` | `Chassis` | [chassis.md](chassis.md#chassis-waituntil) |
| `Chassis::~Chassis` | `Chassis` | [chassis.md](chassis.md#chassis-destructor-chassis) |
| `ChassisConfig::motion` | `ChassisConfig` | [chassis.md](chassis.md#chassisconfig-motion) |
| `ChassisConfig::scheduler` | `ChassisConfig` | [chassis.md](chassis.md#chassisconfig-scheduler) |
| `MotionOptions::maxAngularSpeed` | `MotionOptions` | [chassis.md](chassis.md#motionoptions-maxangularspeed) |
| `MotionOptions::maxLinearSpeed` | `MotionOptions` | [chassis.md](chassis.md#motionoptions-maxlinearspeed) |
| `MotionOptions::timeout` | `MotionOptions` | [chassis.md](chassis.md#motionoptions-timeout) |
| `MotionOptions::validate` | `MotionOptions` | [chassis.md](chassis.md#motionoptions-validate) |
| `Routine::brake` | `Routine` | [routine.md](routine.md#routine-brake) |
| `Routine::chassis` | `Routine` | [routine.md](routine.md#routine-chassis) |
| `Routine::driveTo` | `Routine` | [routine.md](routine.md#routine-driveto) |
| `Routine::face` | `Routine` | [routine.md](routine.md#routine-face) |
| `Routine::followTrajectory` | `Routine` | [routine.md](routine.md#routine-followtrajectory) |
| `Routine::followTrajectory (overload 2)` | `Routine` | [routine.md](routine.md#routine-followtrajectory-2) |
| `Routine::hold` | `Routine` | [routine.md](routine.md#routine-hold) |
| `Routine::lastTrajectory` | `Routine` | [routine.md](routine.md#routine-lasttrajectory) |
| `Routine::moveTo` | `Routine` | [routine.md](routine.md#routine-moveto) |
| `Routine::ok` | `Routine` | [routine.md](routine.md#routine-ok) |
| `Routine::operator=` | `Routine` | [routine.md](routine.md#routine-operator-assign) |
| `Routine::operator= (overload 2)` | `Routine` | [routine.md](routine.md#routine-operator-assign-2) |
| `Routine::pause` | `Routine` | [routine.md](routine.md#routine-pause) |
| `Routine::result` | `Routine` | [routine.md](routine.md#routine-result) |
| `Routine::Routine` | `Routine` | [routine.md](routine.md#routine-routine) |
| `Routine::Routine (overload 2)` | `Routine` | [routine.md](routine.md#routine-routine-2) |
| `Routine::Routine (overload 3)` | `Routine` | [routine.md](routine.md#routine-routine-3) |
| `Routine::startAt` | `Routine` | [routine.md](routine.md#routine-startat) |
| `Routine::strafeTo` | `Routine` | [routine.md](routine.md#routine-strafeto) |
| `Routine::then` | `Routine` | [routine.md](routine.md#routine-then) |
| `Routine::turnTo` | `Routine` | [routine.md](routine.md#routine-turnto) |
| `Routine::waitFor` | `Routine` | [routine.md](routine.md#routine-waitfor) |
| `Routine::~Routine` | `Routine` | [routine.md](routine.md#routine-destructor-routine) |
| `RoutineResult::cause` | `RoutineResult` | [routine.md](routine.md#routineresult-cause) |
| `RoutineResult::completed` | `RoutineResult` | [routine.md](routine.md#routineresult-completed) |
| `RoutineResult::exit` | `RoutineResult` | [routine.md](routine.md#routineresult-exit) |
| `RoutineResult::ok` | `RoutineResult` | [routine.md](routine.md#routineresult-ok) |
| `RoutineResult::skipped` | `RoutineResult` | [routine.md](routine.md#routineresult-skipped) |
| `RoutineResult::steps` | `RoutineResult` | [routine.md](routine.md#routineresult-steps) |
| `RoutineResult::stoppedAt` | `RoutineResult` | [routine.md](routine.md#routineresult-stoppedat) |
| `RoutineResult::stoppedName` | `RoutineResult` | [routine.md](routine.md#routineresult-stoppedname) |
| `RoutineStopCause::ActionFailed` | `RoutineStopCause` | [routine.md](routine.md#routinestopcause-actionfailed) |
| `RoutineStopCause::MechanismFailed` | `RoutineStopCause` | [routine.md](routine.md#routinestopcause-mechanismfailed) |
| `RoutineStopCause::MotionFailed` | `RoutineStopCause` | [routine.md](routine.md#routinestopcause-motionfailed) |
| `RoutineStopCause::None` | `RoutineStopCause` | [routine.md](routine.md#routinestopcause-none) |
| `RoutineStopCause::WaitTimedOut` | `RoutineStopCause` | [routine.md](routine.md#routinestopcause-waittimedout) |
| `TrajectoryResult::completedLegs` | `TrajectoryResult` | [chassis.md](chassis.md#trajectoryresult-completedlegs) |
| `TrajectoryResult::exit` | `TrajectoryResult` | [chassis.md](chassis.md#trajectoryresult-exit) |
| `TrajectoryResult::succeeded` | `TrajectoryResult` | [chassis.md](chassis.md#trajectoryresult-succeeded) |
| `TrajectoryResult::totalLegs` | `TrajectoryResult` | [chassis.md](chassis.md#trajectoryresult-totallegs) |

## Where the other documents fit

- The [user guide](../guide/README.md) teaches the ideas in order, and chapter 10 is the API *as prose* — when to reach for a verb, what it does when things go wrong, which gotchas bite. It deliberately does not restate signatures; this reference owns those.
- The [cookbook](../cookbook/README.md) answers "how do I write the routine I am writing right now", with compiled recipes.
- This reference answers "what exactly exists, and what is its exact spelling".
