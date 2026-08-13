<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Regenerate: python3 tools/api_doc_tool.py generate -->

# API reference

Every public member of shulib's autonomous-routine API, extracted from the headers. This page is generated, so it cannot fall behind the code: a member added to a documented header appears here the next time the tool runs, and the host test build fails if it has not.

**A member with no documentation comment fails the build**, naming itself. That gate is what makes "generated" mean "complete" rather than "generated from whatever someone remembered to write".

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
