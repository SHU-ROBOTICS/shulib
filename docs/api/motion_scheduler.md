<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/motion_scheduler.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motion_scheduler.hpp`

MotionScheduler — the thing that actually runs a routine.

This header declares **8** types (82 members) and **1** free function.

Extracted from [`include/shulib/motion/motion_scheduler.hpp`](../../include/shulib/motion/motion_scheduler.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ITickPacer`](#class-itickpacer)
  - [`~ITickPacer`](#itickpacer-destructor-itickpacer)
  - [`ITickPacer`](#itickpacer-itickpacer)
  - [`ITickPacer (overload 2)`](#itickpacer-itickpacer-2)
  - [`ITickPacer (overload 3)`](#itickpacer-itickpacer-3)
  - [`operator=`](#itickpacer-operator-eq)
  - [`operator= (overload 2)`](#itickpacer-operator-eq-2)
  - [`pace`](#itickpacer-pace)
- [`enum class WaitResult`](#enum-class-waitresult)
  - [`Satisfied`](#waitresult-satisfied)
  - [`TimedOut`](#waitresult-timedout)
- [`faultBit`](#faultbit) — *free function*
- [`struct MotionSchedulerConfig`](#struct-motionschedulerconfig)
  - [`abortFaultMask`](#motionschedulerconfig-abortfaultmask)
  - [`loopMonitor`](#motionschedulerconfig-loopmonitor)
  - [`attributionClock`](#motionschedulerconfig-attributionclock)
  - [`plausibility`](#motionschedulerconfig-plausibility)
- [`class CommandIdStampSink`](#class-commandidstampsink)
  - [`CommandIdStampSink`](#commandidstampsink-commandidstampsink)
  - [`log`](#commandidstampsink-log)
  - [`wantsRecord`](#commandidstampsink-wantsrecord)
  - [`emit`](#commandidstampsink-emit)
  - [`summarize`](#commandidstampsink-summarize)
  - [`setActiveId`](#commandidstampsink-setactiveid)
  - [`activeId`](#commandidstampsink-activeid)
  - [`setTickPhases`](#commandidstampsink-settickphases)
  - [`setEstimatorAudit`](#commandidstampsink-setestimatoraudit)
  - [`beginTick`](#commandidstampsink-begintick)
- [`class MotionStatsSink`](#class-motionstatssink)
  - [`MotionStatsSink`](#motionstatssink-motionstatssink)
  - [`log`](#motionstatssink-log)
  - [`wantsRecord`](#motionstatssink-wantsrecord)
  - [`emit`](#motionstatssink-emit)
  - [`summarize`](#motionstatssink-summarize)
  - [`beginMotion`](#motionstatssink-beginmotion)
  - [`hasData`](#motionstatssink-hasdata)
  - [`targetPose`](#motionstatssink-targetpose)
  - [`overshoot`](#motionstatssink-overshoot)
  - [`drift`](#motionstatssink-drift)
- [`struct CompletedMotion`](#struct-completedmotion)
  - [`id`](#completedmotion-id)
  - [`name`](#completedmotion-name)
  - [`exit`](#completedmotion-exit)
  - [`abortFault`](#completedmotion-abortfault)
  - [`startTime`](#completedmotion-starttime)
  - [`endTime`](#completedmotion-endtime)
  - [`preempted`](#completedmotion-preempted)
  - [`finalPose`](#completedmotion-finalpose)
  - [`hasPathData`](#completedmotion-haspathdata)
  - [`targetPose`](#completedmotion-targetpose)
  - [`overshoot`](#completedmotion-overshoot)
  - [`drift`](#completedmotion-drift)
- [`class IMotionObserver`](#class-imotionobserver)
  - [`~IMotionObserver`](#imotionobserver-destructor-imotionobserver)
  - [`IMotionObserver`](#imotionobserver-imotionobserver)
  - [`IMotionObserver (overload 2)`](#imotionobserver-imotionobserver-2)
  - [`IMotionObserver (overload 3)`](#imotionobserver-imotionobserver-3)
  - [`operator=`](#imotionobserver-operator-eq)
  - [`operator= (overload 2)`](#imotionobserver-operator-eq-2)
  - [`onMotionComplete`](#imotionobserver-onmotioncomplete)
- [`class MotionScheduler`](#class-motionscheduler)
  - [`MotionScheduler`](#motionscheduler-motionscheduler)
  - [`MotionScheduler (overload 2)`](#motionscheduler-motionscheduler-2)
  - [`MotionScheduler (overload 3)`](#motionscheduler-motionscheduler-3)
  - [`operator=`](#motionscheduler-operator-eq)
  - [`operator= (overload 2)`](#motionscheduler-operator-eq-2)
  - [`~MotionScheduler`](#motionscheduler-destructor-motionscheduler)
  - [`deps`](#motionscheduler-deps)
  - [`async`](#motionscheduler-async)
  - [`tick`](#motionscheduler-tick)
  - [`waitUntilSettled`](#motionscheduler-waituntilsettled)
  - [`waitUntil`](#motionscheduler-waituntil)
  - [`cancel`](#motionscheduler-cancel)
  - [`hasActiveMotion`](#motionscheduler-hasactivemotion)
  - [`activeCommandId`](#motionscheduler-activecommandid)
  - [`lastExitReason`](#motionscheduler-lastexitreason)
  - [`lastCompleted`](#motionscheduler-lastcompleted)
  - [`motionsStarted`](#motionscheduler-motionsstarted)
  - [`motionsSettled`](#motionscheduler-motionssettled)
  - [`motionsTimedOut`](#motionscheduler-motionstimedout)
  - [`motionsCancelled`](#motionscheduler-motionscancelled)
  - [`motionsAborted`](#motionscheduler-motionsaborted)
  - [`completedCount`](#motionscheduler-completedcount)
  - [`loopMonitor`](#motionscheduler-loopmonitor)
  - [`setBoundaryObserver`](#motionscheduler-setboundaryobserver)
  - [`boundaryObserver`](#motionscheduler-boundaryobserver)
  - [`runHasHeadingData`](#motionscheduler-runhasheadingdata)
  - [`runMaxHeadingDrift`](#motionscheduler-runmaxheadingdrift)
  - [`runFinalHeadingDrift`](#motionscheduler-runfinalheadingdrift)
  - [`attribution`](#motionscheduler-attribution)
  - [`kMaxStalledPaces`](#motionscheduler-kmaxstalledpaces)

<a id="class-itickpacer"></a>

## `class ITickPacer`

```cpp
class ITickPacer
```

The seam through which the WORLD advances between scheduler ticks (header: "who owns the loop"). Host sim: step the A2 plant by the tick dt. Robot: delay to the next tick boundary. pace() MUST eventually advance IClock::now() — every bounded wait depends on time actually passing; a pacer that never advances the clock trips the scheduler's stalled-pace precondition (loudly) rather than hanging.

*class, declared at [`include/shulib/motion/motion_scheduler.hpp:192`](../../include/shulib/motion/motion_scheduler.hpp#L192).*

<a id="itickpacer-destructor-itickpacer"></a>

### `ITickPacer::~ITickPacer`

```cpp
virtual ~ITickPacer() = default
```

Interface boilerplate: a public virtual destructor, with the copy/move set defaulted back in because declaring a destructor suppresses the implicit MOVE constructor and move assignment (the implicit copies survive, merely deprecated — spelling all five keeps the intent explicit rather than inherited). The scheduler holds a pacer by REFERENCE and never copies, moves or destroys one — the pacer is caller-owned and must outlive the scheduler.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:200`](../../include/shulib/motion/motion_scheduler.hpp#L200).*

<a id="itickpacer-itickpacer"></a>

### `ITickPacer::ITickPacer`

```cpp
ITickPacer() = default
```

*Covered by the comment on [`~ITickPacer`](#itickpacer-destructor-itickpacer) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:201`](../../include/shulib/motion/motion_scheduler.hpp#L201).*

<a id="itickpacer-itickpacer-2"></a>

### `ITickPacer::ITickPacer (overload 2)`

```cpp
ITickPacer(const ITickPacer&) = default
```

*Covered by the comment on [`~ITickPacer`](#itickpacer-destructor-itickpacer) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:202`](../../include/shulib/motion/motion_scheduler.hpp#L202).*

<a id="itickpacer-itickpacer-3"></a>

### `ITickPacer::ITickPacer (overload 3)`

```cpp
ITickPacer(ITickPacer&&) = default
```

*Covered by the comment on [`~ITickPacer`](#itickpacer-destructor-itickpacer) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:203`](../../include/shulib/motion/motion_scheduler.hpp#L203).*

<a id="itickpacer-operator-eq"></a>

### `ITickPacer::operator=`

```cpp
ITickPacer& operator=(const ITickPacer&) = default
```

*Covered by the comment on [`~ITickPacer`](#itickpacer-destructor-itickpacer) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:204`](../../include/shulib/motion/motion_scheduler.hpp#L204).*

<a id="itickpacer-operator-eq-2"></a>

### `ITickPacer::operator= (overload 2)`

```cpp
ITickPacer& operator=(ITickPacer&&) = default
```

*Covered by the comment on [`~ITickPacer`](#itickpacer-destructor-itickpacer) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:205`](../../include/shulib/motion/motion_scheduler.hpp#L205).*

<a id="itickpacer-pace"></a>

### `ITickPacer::pace`

```cpp
virtual void pace() = 0
```

Advance the world to the next control-tick instant.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:208`](../../include/shulib/motion/motion_scheduler.hpp#L208).*

<a id="enum-class-waitresult"></a>

## `enum class WaitResult`

```cpp
enum class WaitResult
```

The outcome of waitUntil — a DISTINCT vocabulary from ExitReason on purpose: a predicate satisfying is not a motion settling, and conflating them would let "the wait timed out" read as "the motion timed out".

*enum class, declared at [`include/shulib/motion/motion_scheduler.hpp:214`](../../include/shulib/motion/motion_scheduler.hpp#L214).*

<a id="waitresult-satisfied"></a>

### `WaitResult::Satisfied`

```cpp
Satisfied
```

the predicate became true (possibly true on entry)

*enumerator, declared at [`include/shulib/motion/motion_scheduler.hpp:215`](../../include/shulib/motion/motion_scheduler.hpp#L215).*

<a id="waitresult-timedout"></a>

### `WaitResult::TimedOut`

```cpp
TimedOut
```

the timeout elapsed first — the predicate never held

*enumerator, declared at [`include/shulib/motion/motion_scheduler.hpp:216`](../../include/shulib/motion/motion_scheduler.hpp#L216).*

<a id="faultbit"></a>

## `faultBit`

```cpp
[[nodiscard]] constexpr std::uint32_t faultBit(diag::FaultCode code) noexcept
```

One bit per FaultCode value, for MotionSchedulerConfig::abortFaultMask.

*free function, declared at [`include/shulib/motion/motion_scheduler.hpp:220`](../../include/shulib/motion/motion_scheduler.hpp#L220).*

<a id="struct-motionschedulerconfig"></a>

## `struct MotionSchedulerConfig`

```cpp
struct MotionSchedulerConfig
```

Scheduler policy, COPIED at construction — mutating the caller's struct afterwards changes nothing about a live scheduler. The defaults are the competition posture: abort a motion only when the estimate is lying (ODO_STUCK), tick-time attribution OFF (nullptr = zero clock calls, zero cost), and a generous advisory plausibility envelope that never rewrites a pose. Every pointer here must outlive the scheduler.

*struct, declared at [`include/shulib/motion/motion_scheduler.hpp:229`](../../include/shulib/motion/motion_scheduler.hpp#L229).*

<a id="motionschedulerconfig-abortfaultmask"></a>

### `MotionSchedulerConfig::abortFaultMask`

```cpp
std::uint32_t abortFaultMask = faultBit(diag::FaultCode::OdoStuck)
```

Faults that ABORT the active motion when raised during it (header: "the fault policy"). Default: ODO_STUCK only — the one code that means the estimate is lying. Policy, not physics: configurable by design.

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:233`](../../include/shulib/motion/motion_scheduler.hpp#L233).*

<a id="motionschedulerconfig-loopmonitor"></a>

### `MotionSchedulerConfig::loopMonitor`

```cpp
diag::LoopMonitorConfig loopMonitor{}
```

Scheduler-owned loop timing watchdog (LOOP_OVERRUN). The budget must be strictly greater than the nominal tick period (loop_monitor.hpp).

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:237`](../../include/shulib/motion/motion_scheduler.hpp#L237).*

<a id="motionschedulerconfig-attributionclock"></a>

### `MotionSchedulerConfig::attributionClock`

```cpp
hal::IClock* attributionClock = nullptr
```

D-3 tick-time attribution clock (chunk C5). nullptr = attribution OFF — zero clock calls, zero cost (the A1 contract, structurally). When set, it must be a clock that advances DURING a tick (tick_attribution.hpp says which: real time on the robot — R1 wires it; a scripted fake in tests — the SIM clock only advances between ticks and would attribute all zeros). Must outlive the scheduler.

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:245`](../../include/shulib/motion/motion_scheduler.hpp#L245).*

<a id="motionschedulerconfig-plausibility"></a>

### `MotionSchedulerConfig::plausibility`

```cpp
diag::PlausibilityConfig plausibility{}
```

D-5 pose-delta plausibility envelope (chunk C5): per-tick estimate motion beyond maxSpeed/maxYawRate × margin × dt raises IMPLAUSIBLE (advisory, episode-gated — plausibility_guard.hpp). Defaults are generous physical upper bounds (PROVISIONAL, A4: HA-56).

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:251`](../../include/shulib/motion/motion_scheduler.hpp#L251).*

<a id="class-commandidstampsink"></a>

## `class CommandIdStampSink`

```cpp
class CommandIdStampSink final : public hal::ITelemetrySink
```

ITelemetrySink decorator that stamps DebugRecord.activeCommandId with the scheduler's current id (0 between motions). Stamping at the SINK makes id assignment unforgettable for every record producer — no motion type has to remember to do it. The overwrite is unconditional: this scheduler is THE id assigner (debug_record.hpp), so an incoming nonzero id would be a bug, not information. wantsRecord() forwards to the inner sink — the A1 pair rule — so record population stays skipped when nothing consumes it; the one-record copy in emit() is paid only when a real sink is attached.  Since C5 it also stamps the D-3 tickPhase slots: the scheduler sets the LAST COMPLETED tick's attribution after each tick (records are emitted mid-tick, before this tick's total is knowable — the one-tick lag documented on the schema field). With attribution off the stamp is the quiet all-zeros default. One decorator, one record copy, both stamps.  ── Since E1 it also stamps the ESTIMATOR fields, and the tick's fault ────────── Two holes were found while wiring the blackbox, and both are fixed HERE because this is the layer that owns record population: * Only MoveToPose stamped `correctionDx/Dy/clampedThisTick`; TurnTo, StrafeTo, DriveBrake, HoldPose and the idle record left them at zero, so what the fusion gate did was invisible for most of a run. The §18.2 gating slots (`gateResidual*`, `gateMahalanobis`, `gateReason`, `covarianceTrace`) had no producer at all. * `DebugRecord::fault` — "the fault raised THIS tick" — had NO producer anywhere in the tree. TermSink has rendered ` flt=NAME` since A1 and it could never appear on a real run; the SdSink flight recorder's whole trigger is that field. Both are now stamped from the ONE place every record already passes through, which is the same reasoning that put the command id here. The fault stamp is deliberately CONDITIONAL (unlike the id): a producer that already knows its own fault keeps it. Honest scope: the stamped fault is the most recent fault raised during this tick BEFORE this record was emitted — a fault raised later in the same tick lands on the next record. The FaultLatch remains the authority on the first-fault root cause.

*class, declared at [`include/shulib/motion/motion_scheduler.hpp:286`](../../include/shulib/motion/motion_scheduler.hpp#L286).*

<a id="commandidstampsink-commandidstampsink"></a>

### `CommandIdStampSink::CommandIdStampSink`

```cpp
explicit CommandIdStampSink(hal::ITelemetrySink& inner, const diag::FaultLatch* faults = nullptr) noexcept
```

`faults` (optional) supplies the per-tick fault stamp; nullptr disables it.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:289`](../../include/shulib/motion/motion_scheduler.hpp#L289).*

<a id="commandidstampsink-log"></a>

### `CommandIdStampSink::log`

```cpp
void log(hal::LogLevel level, std::string_view subsystem, std::string_view message) override
```

Pass-through, unstamped: every stamp this decorator applies rides the RECORD channel, so a log line never carries a command id.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:295`](../../include/shulib/motion/motion_scheduler.hpp#L295).*

<a id="commandidstampsink-wantsrecord"></a>

### `CommandIdStampSink::wantsRecord`

```cpp
[[nodiscard]] bool wantsRecord() const noexcept override
```

Forwards the inner sink's answer — the A1 pair rule. A NullSink run therefore still skips record population entirely, and this decorator costs one bool query.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:302`](../../include/shulib/motion/motion_scheduler.hpp#L302).*

<a id="commandidstampsink-emit"></a>

### `CommandIdStampSink::emit`

```cpp
void emit(const diag::DebugRecord& record) override
```

Stamp one record and forward it: the command id (UNCONDITIONALLY — this scheduler is the id assigner, so an incoming nonzero id is a bug, not information), the last completed tick's phase breakdown, the estimator's gate audit, and — only if the producer left it None — the fault raised so far this tick. Costs one DebugRecord copy, paid only when a sink downstream actually wants records.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:309`](../../include/shulib/motion/motion_scheduler.hpp#L309).*

<a id="commandidstampsink-summarize"></a>

### `CommandIdStampSink::summarize`

```cpp
void summarize(const diag::RunSummary& summary) override
```

C5 decorator rule (telemetry_sink.hpp): forward, or the summary dies here.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:331`](../../include/shulib/motion/motion_scheduler.hpp#L331).*

<a id="commandidstampsink-setactiveid"></a>

### `CommandIdStampSink::setActiveId`

```cpp
void setActiveId(std::uint32_t id) noexcept
```

The id every subsequent record is stamped with; 0 means "between motions". The scheduler calls this when it arms a motion and again at its boundary — nothing else should, or records will be attributed to a motion that never emitted them.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:336`](../../include/shulib/motion/motion_scheduler.hpp#L336).*

<a id="commandidstampsink-activeid"></a>

### `CommandIdStampSink::activeId`

```cpp
[[nodiscard]] std::uint32_t activeId() const noexcept
```

Whatever setActiveId() last received; 0 between motions.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:338`](../../include/shulib/motion/motion_scheduler.hpp#L338).*

<a id="commandidstampsink-settickphases"></a>

### `CommandIdStampSink::setTickPhases`

```cpp
void setTickPhases( const std::array<units::Time, static_cast<std::size_t>(diag::kTickPhaseSlots)>& phases) noexcept
```

Install the per-TickPhase time breakdown stamped onto subsequent records. The scheduler passes the LAST COMPLETED tick's numbers, because a record emitted mid-tick cannot know its own tick's total — that is the one-tick lag documented on DebugRecord::tickPhase. All zeros while attribution is off.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:344`](../../include/shulib/motion/motion_scheduler.hpp#L344).*

<a id="commandidstampsink-setestimatoraudit"></a>

### `CommandIdStampSink::setEstimatorAudit`

```cpp
void setEstimatorAudit(const localization::AppliedCorrection& audit) noexcept
```

The estimator's account of the tick just localized (E1). The scheduler calls this right after Localizer::update(), so every record emitted during the tick — motion or idle — carries the same, consistent gate audit.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:353`](../../include/shulib/motion/motion_scheduler.hpp#L353).*

<a id="commandidstampsink-begintick"></a>

### `CommandIdStampSink::beginTick`

```cpp
void beginTick() noexcept
```

Open a new tick for the fault stamp: everything raised from here on belongs to this tick. Cheap (one counter read) and a no-op without a latch.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:359`](../../include/shulib/motion/motion_scheduler.hpp#L359).*

<a id="class-motionstatssink"></a>

## `class MotionStatsSink`

```cpp
class MotionStatsSink final : public hal::ITelemetrySink
```

ITelemetrySink decorator that AGGREGATES the active motion's record stream into the C5 result-line quantities (motion_result.hpp carries their definitions): start pose, target, worst excursion past the target, final heading error. Sits AFTER the id stamp in the scheduler's chain (it discriminates on the stamped id) and forwards everything untouched — a pure observer.  Why derive these from the RECORD STREAM rather than ask the motion: the boundary (CompletedMotion) must not re-derive what the motion already published per tick (brief rule 7), overshoot is inherently a per-tick MAX no boundary snapshot can recover, and the stream is the one place every motion type — including future Tier-3 ones — already reports target/measured/error uniformly. Consequence, stated honestly: with NullSink no records flow (wantsRecord false ⇒ never even built), so hasData() is false and the result line renders "n/a" for the derived fields — you cannot have free result numbers AND zero-cost ticks; the always-real fields (final pose, duration, outcome) come from the boundary itself.  Aggregation rules (each load-bearing, pinned by test): * only records with a nonzero stamped id (idle/teleop records are not the motion's story); * only Running-state ticks and — once Running was seen — the exit-state record (waiting-for-estimate records carry deliberately-zero errors and, for capture-at-live motions, a not-yet-real target: aggregating them would fabricate numbers, the exact lie the brief bans); * target is re-sampled per record (capture-at-live motions publish it from the first live tick; TurnTo/DriveBrake publish a here-anchored target).

*class, declared at [`include/shulib/motion/motion_scheduler.hpp:406`](../../include/shulib/motion/motion_scheduler.hpp#L406).*

<a id="motionstatssink-motionstatssink"></a>

### `MotionStatsSink::MotionStatsSink`

```cpp
explicit MotionStatsSink(hal::ITelemetrySink& inner) noexcept
```

`inner` is NON-OWNING and must outlive this sink; every call is forwarded to it. One of these serves a whole scheduler, not one motion — beginMotion() is what clears the aggregates between motions.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:411`](../../include/shulib/motion/motion_scheduler.hpp#L411).*

<a id="motionstatssink-log"></a>

### `MotionStatsSink::log`

```cpp
void log(hal::LogLevel level, std::string_view subsystem, std::string_view message) override
```

Pass-through: only the record channel carries the quantities this sink derives.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:414`](../../include/shulib/motion/motion_scheduler.hpp#L414).*

<a id="motionstatssink-wantsrecord"></a>

### `MotionStatsSink::wantsRecord`

```cpp
[[nodiscard]] bool wantsRecord() const noexcept override
```

Forwards the inner sink's answer, which is also the honest limit of this sink: behind a sink that wants no records, nothing is ever aggregated, hasData() stays false, and the derived result-line fields render "n/a" rather than a made-up 0.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:422`](../../include/shulib/motion/motion_scheduler.hpp#L422).*

<a id="motionstatssink-emit"></a>

### `MotionStatsSink::emit`

```cpp
void emit(const diag::DebugRecord& record) override
```

Aggregate, then forward the record UNMODIFIED — a pure observer that stamps nothing, so it may sit anywhere after the id stamp it discriminates on.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:426`](../../include/shulib/motion/motion_scheduler.hpp#L426).*

<a id="motionstatssink-summarize"></a>

### `MotionStatsSink::summarize`

```cpp
void summarize(const diag::RunSummary& summary) override
```

Pass-through, per the decorator rule (telemetry_sink.hpp): a decorator that keeps the default no-op body silently eats the run summary.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:433`](../../include/shulib/motion/motion_scheduler.hpp#L433).*

<a id="motionstatssink-beginmotion"></a>

### `MotionStatsSink::beginMotion`

```cpp
void beginMotion() noexcept
```

New motion armed: forget the previous motion's story.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:436`](../../include/shulib/motion/motion_scheduler.hpp#L436).*

<a id="motionstatssink-hasdata"></a>

### `MotionStatsSink::hasData`

```cpp
[[nodiscard]] bool hasData() const noexcept
```

True iff at least one live (Running) record was aggregated.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:444`](../../include/shulib/motion/motion_scheduler.hpp#L444).*

<a id="motionstatssink-targetpose"></a>

### `MotionStatsSink::targetPose`

```cpp
[[nodiscard]] const math::Pose2d& targetPose() const noexcept
```

The motion's published target, RE-SAMPLED from the most recent aggregated record: a capture-at-live motion has no real target until its first live tick, so this is the last target it published, not the one it was constructed with. Read it ONLY when hasData(): beginMotion() does NOT clear it, so between motions it still holds the PREVIOUS motion's target — the scheduler substitutes a default Pose2d itself.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:450`](../../include/shulib/motion/motion_scheduler.hpp#L450).*

<a id="motionstatssink-overshoot"></a>

### `MotionStatsSink::overshoot`

```cpp
[[nodiscard]] units::Length overshoot() const noexcept
```

Overshoot per motion_result.hpp: projection past the target along the start→target direction when the motion HAD a direction; worst wander from the point when it did not (|target − start| < kHoldEpsilonIn).

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:455`](../../include/shulib/motion/motion_scheduler.hpp#L455).*

<a id="motionstatssink-drift"></a>

### `MotionStatsSink::drift`

```cpp
[[nodiscard]] units::AngleDim drift() const noexcept
```

|final heading error| — the last aggregated record's errorHeading.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:465`](../../include/shulib/motion/motion_scheduler.hpp#L465).*

<a id="struct-completedmotion"></a>

## `struct CompletedMotion`

```cpp
struct CompletedMotion
```

One finished motion, as the scheduler saw it — the raw material for the C5 per-motion result line (motion/run_reporter.hpp formats it; this type only records). The C5 fields were ADDED here rather than shadowed in a parallel struct (brief rule 7: CompletedMotion is the one motion-boundary record).

*struct, declared at [`include/shulib/motion/motion_scheduler.hpp:522`](../../include/shulib/motion/motion_scheduler.hpp#L522).*

<a id="completedmotion-id"></a>

### `CompletedMotion::id`

```cpp
std::uint32_t id = 0
```

the activeCommandId it ran under

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:523`](../../include/shulib/motion/motion_scheduler.hpp#L523).*

<a id="completedmotion-name"></a>

### `CompletedMotion::name`

```cpp
const char* name = ""
```

IMotion::name() (stable literal)

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:524`](../../include/shulib/motion/motion_scheduler.hpp#L524).*

<a id="completedmotion-exit"></a>

### `CompletedMotion::exit`

```cpp
control::ExitReason exit = control::ExitReason::Running
```

Running ⇒ "none yet"

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:525`](../../include/shulib/motion/motion_scheduler.hpp#L525).*

<a id="completedmotion-abortfault"></a>

### `CompletedMotion::abortFault`

```cpp
diag::FaultCode abortFault = diag::FaultCode::None
```

None for a settle/timeout/user-cancel; the causal FaultCode when the scheduler's fault policy (or the task-boundary catch) forced the abort.

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:528`](../../include/shulib/motion/motion_scheduler.hpp#L528).*

<a id="completedmotion-starttime"></a>

### `CompletedMotion::startTime`

```cpp
units::Time startTime{}
```

clock at async()

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:529`](../../include/shulib/motion/motion_scheduler.hpp#L529).*

<a id="completedmotion-endtime"></a>

### `CompletedMotion::endTime`

```cpp
units::Time endTime{}
```

clock at the exit/cancel boundary

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:530`](../../include/shulib/motion/motion_scheduler.hpp#L530).*

<a id="completedmotion-preempted"></a>

### `CompletedMotion::preempted`

```cpp
bool preempted = false
```

True iff this Cancelled boundary was a PRE-EMPTION (a newer motion took the slot) — §18.4's SUPERSEDED, distinct from a user cancel.

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:535`](../../include/shulib/motion/motion_scheduler.hpp#L535).*

<a id="completedmotion-finalpose"></a>

### `CompletedMotion::finalPose`

```cpp
math::Pose2d finalPose{}
```

The estimate at the boundary — ALWAYS real (read from the Localizer at finalize, independent of the record stream).

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:538`](../../include/shulib/motion/motion_scheduler.hpp#L538).*

<a id="completedmotion-haspathdata"></a>

### `CompletedMotion::hasPathData`

```cpp
bool hasPathData = false
```

True iff the record stream flowed for a live tick of this motion; the three fields below are only meaningful when it did (MotionStatsSink's honest-scope note — with NullSink they render "n/a", never a lie).

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:542`](../../include/shulib/motion/motion_scheduler.hpp#L542).*

<a id="completedmotion-targetpose"></a>

### `CompletedMotion::targetPose`

```cpp
math::Pose2d targetPose{}
```

the motion's published target (last sampled)

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:543`](../../include/shulib/motion/motion_scheduler.hpp#L543).*

<a id="completedmotion-overshoot"></a>

### `CompletedMotion::overshoot`

```cpp
units::Length overshoot{}
```

worst excursion past the target (see semantics)

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:544`](../../include/shulib/motion/motion_scheduler.hpp#L544).*

<a id="completedmotion-drift"></a>

### `CompletedMotion::drift`

```cpp
units::AngleDim drift{}
```

|final heading error|

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:545`](../../include/shulib/motion/motion_scheduler.hpp#L545).*

<a id="class-imotionobserver"></a>

## `class IMotionObserver`

```cpp
class IMotionObserver
```

Boundary-observer seam (chunk C5): the scheduler calls this SYNCHRONOUSLY at every motion boundary — exit, fault abort, user cancel, pre-empt — right after CompletedMotion is fully recorded. This is what makes the per-motion result line STRUCTURAL (RunReporter implements it): a routine cannot forget to report a boundary, the A1 emitRecord lesson one layer up. Contract: the callback may log through the sinks; it must NOT call any scheduler verb (async/cancel/tick/waits — enforced by precondition: the boundary is not a place to re-plan a routine from). It must not throw.

*class, declared at [`include/shulib/motion/motion_scheduler.hpp:556`](../../include/shulib/motion/motion_scheduler.hpp#L556).*

<a id="imotionobserver-destructor-imotionobserver"></a>

### `IMotionObserver::~IMotionObserver`

```cpp
virtual ~IMotionObserver() = default
```

Interface boilerplate: a public virtual destructor, with the copy/move set defaulted back in because declaring a destructor suppresses the implicit MOVE constructor and move assignment (the implicit copies survive, merely deprecated — spelling all five keeps the intent explicit rather than inherited). Observers attach by RAW POINTER through setBoundaryObserver(); the scheduler never owns one, so an observer must outlive it or be detached first.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:564`](../../include/shulib/motion/motion_scheduler.hpp#L564).*

<a id="imotionobserver-imotionobserver"></a>

### `IMotionObserver::IMotionObserver`

```cpp
IMotionObserver() = default
```

*Covered by the comment on [`~IMotionObserver`](#imotionobserver-destructor-imotionobserver) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:565`](../../include/shulib/motion/motion_scheduler.hpp#L565).*

<a id="imotionobserver-imotionobserver-2"></a>

### `IMotionObserver::IMotionObserver (overload 2)`

```cpp
IMotionObserver(const IMotionObserver&) = default
```

*Covered by the comment on [`~IMotionObserver`](#imotionobserver-destructor-imotionobserver) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:566`](../../include/shulib/motion/motion_scheduler.hpp#L566).*

<a id="imotionobserver-imotionobserver-3"></a>

### `IMotionObserver::IMotionObserver (overload 3)`

```cpp
IMotionObserver(IMotionObserver&&) = default
```

*Covered by the comment on [`~IMotionObserver`](#imotionobserver-destructor-imotionobserver) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:567`](../../include/shulib/motion/motion_scheduler.hpp#L567).*

<a id="imotionobserver-operator-eq"></a>

### `IMotionObserver::operator=`

```cpp
IMotionObserver& operator=(const IMotionObserver&) = default
```

*Covered by the comment on [`~IMotionObserver`](#imotionobserver-destructor-imotionobserver) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:568`](../../include/shulib/motion/motion_scheduler.hpp#L568).*

<a id="imotionobserver-operator-eq-2"></a>

### `IMotionObserver::operator= (overload 2)`

```cpp
IMotionObserver& operator=(IMotionObserver&&) = default
```

*Covered by the comment on [`~IMotionObserver`](#imotionobserver-destructor-imotionobserver) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:569`](../../include/shulib/motion/motion_scheduler.hpp#L569).*

<a id="imotionobserver-onmotioncomplete"></a>

### `IMotionObserver::onMotionComplete`

```cpp
virtual void onMotionComplete(const CompletedMotion& completed) = 0
```

One finished motion, observed at its boundary.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:572`](../../include/shulib/motion/motion_scheduler.hpp#L572).*

<a id="class-motionscheduler"></a>

## `class MotionScheduler`

```cpp
class MotionScheduler
```

The loop that actually runs a routine. Exactly ONE active motion and no queue: starting another PRE-EMPTS the first into the cancel safe state (0 V + Brake, applied synchronously), so there is no tick on which two motions command. It never owns time — the injected ITickPacer advances the world, which is what lets the same scheduler be deterministic in host sim and real on the robot. The verbs are async() to arm, tick() or a blocking wait to make progress, cancel() to stop; cancel() with nothing active is still the panic stop, because a cancel that can be too late is one nobody can rely on. Nothing here can hang: waitUntilSettled() is bounded by the motion's own watchdog, waitUntil() by a required explicit timeout, and a pacer that stops advancing the clock fails loudly rather than spinning. Faults in abortFaultMask abort the MOTION, never the run. Single-task by contract, like everything it composes.

*class, declared at [`include/shulib/motion/motion_scheduler.hpp:586`](../../include/shulib/motion/motion_scheduler.hpp#L586).*

<a id="motionscheduler-motionscheduler"></a>

### `MotionScheduler::MotionScheduler`

```cpp
MotionScheduler(const MotionDeps& deps, ITickPacer& pacer, const MotionSchedulerConfig& config = {})
```

`deps` is the same bundle every motion takes (validated non-null); all pointees — and `pacer` — must outlive the scheduler.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:590`](../../include/shulib/motion/motion_scheduler.hpp#L590).*

<a id="motionscheduler-motionscheduler-2"></a>

### `MotionScheduler::MotionScheduler (overload 2)`

```cpp
MotionScheduler(const MotionScheduler&) = delete
```

Neither copyable nor movable, and not by taste: the context this scheduler hands to motions points at the scheduler's OWN telemetry decorator, so a copy or a move would leave that route aimed at the original object. Construct one where it will live and pass it by reference. Destruction is DEFAULTED and does not cancel — a scheduler destroyed with a motion still armed leaves the drive at its last command, so call cancel() before letting one go out of scope.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:622`](../../include/shulib/motion/motion_scheduler.hpp#L622).*

<a id="motionscheduler-motionscheduler-3"></a>

### `MotionScheduler::MotionScheduler (overload 3)`

```cpp
MotionScheduler(MotionScheduler&&) = delete
```

*Covered by the comment on [`MotionScheduler (overload 2)`](#motionscheduler-motionscheduler-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:623`](../../include/shulib/motion/motion_scheduler.hpp#L623).*

<a id="motionscheduler-operator-eq"></a>

### `MotionScheduler::operator=`

```cpp
MotionScheduler& operator=(const MotionScheduler&) = delete
```

*Covered by the comment on [`MotionScheduler (overload 2)`](#motionscheduler-motionscheduler-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:624`](../../include/shulib/motion/motion_scheduler.hpp#L624).*

<a id="motionscheduler-operator-eq-2"></a>

### `MotionScheduler::operator= (overload 2)`

```cpp
MotionScheduler& operator=(MotionScheduler&&) = delete
```

*Covered by the comment on [`MotionScheduler (overload 2)`](#motionscheduler-motionscheduler-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:625`](../../include/shulib/motion/motion_scheduler.hpp#L625).*

<a id="motionscheduler-destructor-motionscheduler"></a>

### `MotionScheduler::~MotionScheduler`

```cpp
~MotionScheduler() = default
```

*Covered by the comment on [`MotionScheduler (overload 2)`](#motionscheduler-motionscheduler-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:626`](../../include/shulib/motion/motion_scheduler.hpp#L626).*

<a id="motionscheduler-deps"></a>

### `MotionScheduler::deps`

```cpp
[[nodiscard]] const MotionDeps& deps() const noexcept
```

The MotionDeps to construct scheduled motions FROM: identical to the caller's deps except telemetry routes through the id stamp (header: observability). A motion built with raw deps still schedules correctly — its records merely carry id 0. Flagged for F6: the C4 facade must build motions from THIS so the stamping is structural, not remembered.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:633`](../../include/shulib/motion/motion_scheduler.hpp#L633).*

<a id="motionscheduler-async"></a>

### `MotionScheduler::async`

```cpp
void async(IMotion& motion)
```

Start `motion` without blocking: arm it and return — it progresses on subsequent ticks (tick() / the blocking waits). If a motion is active, PRE-EMPT per the pinned semantics (header): the old motion is cancelled into the safe state first; there is no tick on which both command. async(active motion) is a well-defined RESTART (cancel + re-arm). `motion` must outlive its scheduled run. Callable from a waitUntil predicate; NOT from inside a motion tick.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:642`](../../include/shulib/motion/motion_scheduler.hpp#L642).*

<a id="motionscheduler-tick"></a>

### `MotionScheduler::tick`

```cpp
bool tick()
```

One scheduler tick (header: "who owns the loop") — for callers running their own paced loop (the facade's non-blocking mode; teleop polling). Does NOT pace: the caller owns cadence here. Returns whether a motion is still active after the tick. Not callable re-entrantly or from a blocking wait (the wait already owns the loop).

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:674`](../../include/shulib/motion/motion_scheduler.hpp#L674).*

<a id="motionscheduler-waituntilsettled"></a>

### `MotionScheduler::waitUntilSettled`

```cpp
[[nodiscard]] control::ExitReason waitUntilSettled()
```

Block until the active motion exits; returns its ExitReason (Settled / TimedOut / Cancelled — never Running). Bounded WITHOUT a parameter: the motion's own watchdog guarantees exit (C1, mutation-proven), and the stalled-pace guard converts a broken pacer into a loud failure. With no active motion the wait is VACUOUSLY over and returns lastExitReason() immediately (Settled on a virgin scheduler — completedCount() tells a caller nothing actually ran).

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:691`](../../include/shulib/motion/motion_scheduler.hpp#L691).*

<a id="motionscheduler-waituntil"></a>

### `MotionScheduler::waitUntil`

```cpp
template <typename Pred> [[nodiscard]] WaitResult waitUntil(Pred&& pred, double timeoutSeconds)
```

Block until `pred()` holds (checked BEFORE the first tick — true on entry returns immediately) or `timeoutSeconds` elapses, whichever is first; the return says which. The active motion (if any) keeps ticking throughout — this is the marker/callback primitive (G2's PathRunner). timeout is REQUIRED, finite and >= 0 (0 = an honest poll); a timeout logs one Warn line and raises NO fault (header: nothing may hang). `pred` may call async()/cancel() (pre-emption applies); it must not call a blocking verb (precondition).

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:722`](../../include/shulib/motion/motion_scheduler.hpp#L722).*

<a id="motionscheduler-cancel"></a>

### `MotionScheduler::cancel`

```cpp
void cancel()
```

Stop the active motion into the defined safe state (0 V + Brake — motion.hpp), record the Cancelled boundary, and idle the scheduler. With NO active motion this is the PANIC STOP: the safe state is applied to the drive anyway (a cancel that can be "too late" to do anything is a cancel nobody can rely on). Idempotent; callable from a waitUntil predicate AND from a pacer's pace() (the F2 deadline cut — pinned in the re-entrancy banner); NOT from inside a motion tick.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:761`](../../include/shulib/motion/motion_scheduler.hpp#L761).*

<a id="motionscheduler-hasactivemotion"></a>

### `MotionScheduler::hasActiveMotion`

```cpp
[[nodiscard]] bool hasActiveMotion() const noexcept
```

True between async() and that motion's boundary — equivalently activeCommandId() != 0. False again the instant a motion settles, times out, is cancelled or is pre-empted, on the same tick, before any wait returns.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:778`](../../include/shulib/motion/motion_scheduler.hpp#L778).*

<a id="motionscheduler-activecommandid"></a>

### `MotionScheduler::activeCommandId`

```cpp
[[nodiscard]] std::uint32_t activeCommandId() const noexcept
```

The active motion's command id; 0 when none. Ids are 1-based and monotonically increasing for the scheduler's lifetime.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:781`](../../include/shulib/motion/motion_scheduler.hpp#L781).*

<a id="motionscheduler-lastexitreason"></a>

### `MotionScheduler::lastExitReason`

```cpp
[[nodiscard]] control::ExitReason lastExitReason() const noexcept
```

Exit reason of the most recently finished motion. Settled before any motion has finished (the vacuous-wait default — see waitUntilSettled).

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:784`](../../include/shulib/motion/motion_scheduler.hpp#L784).*

<a id="motionscheduler-lastcompleted"></a>

### `MotionScheduler::lastCompleted`

```cpp
[[nodiscard]] const CompletedMotion& lastCompleted() const noexcept
```

The most recent motion boundary in full, overwritten at each one. Default- constructed until a motion finishes, and IN THAT VIRGIN STATE ONLY it disagrees with lastExitReason(): this reads Running ("none yet") where that reads Settled (the vacuous-wait default). Once any motion has reached a boundary the two always agree — finalize() writes both from the same exit reason. completedCount() is what actually says whether anything ran.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:791`](../../include/shulib/motion/motion_scheduler.hpp#L791).*

<a id="motionscheduler-motionsstarted"></a>

### `MotionScheduler::motionsStarted`

```cpp
[[nodiscard]] int motionsStarted() const noexcept
```

async() calls over the scheduler's lifetime — restarts and pre-empting starts included, so this counts STARTS, not distinct motion objects. It equals completedCount() plus one while a motion is active, and equals it exactly when idle.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:795`](../../include/shulib/motion/motion_scheduler.hpp#L795).*

<a id="motionscheduler-motionssettled"></a>

### `MotionScheduler::motionsSettled`

```cpp
[[nodiscard]] int motionsSettled() const noexcept
```

Motions that reached their exit group and stopped there — the only success verdict of the four; the counters around it are all the ways a motion did not finish the job it was given.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:799`](../../include/shulib/motion/motion_scheduler.hpp#L799).*

<a id="motionscheduler-motionstimedout"></a>

### `MotionScheduler::motionsTimedOut`

```cpp
[[nodiscard]] int motionsTimedOut() const noexcept
```

Motions the MOTION's own watchdog ended. A waitUntil() timeout is not counted here and raises no fault — that is a wait giving up, not a motion failing.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:802`](../../include/shulib/motion/motion_scheduler.hpp#L802).*

<a id="motionscheduler-motionscancelled"></a>

### `MotionScheduler::motionsCancelled`

```cpp
[[nodiscard]] int motionsCancelled() const noexcept
```

User/pre-empt cancellations (abortFault == None).

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:804`](../../include/shulib/motion/motion_scheduler.hpp#L804).*

<a id="motionscheduler-motionsaborted"></a>

### `MotionScheduler::motionsAborted`

```cpp
[[nodiscard]] int motionsAborted() const noexcept
```

Fault-policy + task-boundary aborts (abortFault != None).

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:806`](../../include/shulib/motion/motion_scheduler.hpp#L806).*

<a id="motionscheduler-completedcount"></a>

### `MotionScheduler::completedCount`

```cpp
[[nodiscard]] int completedCount() const noexcept
```

Every motion that reached a boundary: settled + timed out + cancelled + aborted, a partition with no double counting. This is the number that tells a caller whether anything actually ran, which lastExitReason() cannot — it reads Settled on a scheduler that has never been given a motion.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:811`](../../include/shulib/motion/motion_scheduler.hpp#L811).*

<a id="motionscheduler-loopmonitor"></a>

### `MotionScheduler::loopMonitor`

```cpp
[[nodiscard]] const diag::LoopMonitor& loopMonitor() const noexcept
```

The scheduler's own tick-timing watchdog, for worstDt() / overrunCount() after a run. The scheduler ticks it once per tick and RE-BASELINES it at every async() and at the top of each blocking wait — that drops only the previous tick's timestamp, so a deliberate gap in which the caller's own code ran between motions is not reported as an overrun. Nothing here ever clears the statistics: worstDt() and overrunCount() are WHOLE-RUN totals, not per-motion ones. A gap between two of the caller's own tick() calls is NOT re-baselined and does count as an overrun.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:821`](../../include/shulib/motion/motion_scheduler.hpp#L821).*

<a id="motionscheduler-setboundaryobserver"></a>

### `MotionScheduler::setBoundaryObserver`

```cpp
void setBoundaryObserver(IMotionObserver* observer) noexcept
```

Attach/replace the boundary observer (nullptr detaches). One observer: the C5 reporter is the intended consumer; fan-out belongs to a composite the caller writes if ever needed. Contract in IMotionObserver.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:828`](../../include/shulib/motion/motion_scheduler.hpp#L828).*

<a id="motionscheduler-boundaryobserver"></a>

### `MotionScheduler::boundaryObserver`

```cpp
[[nodiscard]] IMotionObserver* boundaryObserver() const noexcept
```

The attached observer, or nullptr. NON-OWNING: the scheduler neither deletes it nor extends its lifetime, so detach before the observer dies.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:831`](../../include/shulib/motion/motion_scheduler.hpp#L831).*

<a id="motionscheduler-runhasheadingdata"></a>

### `MotionScheduler::runHasHeadingData`

```cpp
[[nodiscard]] bool runHasHeadingData() const noexcept
```

The run's heading story for the §18.3 summary: max / final of the PER-MOTION BOUNDARY drifts (|final heading error| of each motion that produced path data). Deliberately not mid-tick transients: a 90° turn passes through 90° of "error" by design, and a summary that reported it would bury the real story — how headings LANDED.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:838`](../../include/shulib/motion/motion_scheduler.hpp#L838).*

<a id="motionscheduler-runmaxheadingdrift"></a>

### `MotionScheduler::runMaxHeadingDrift`

```cpp
[[nodiscard]] units::AngleDim runMaxHeadingDrift() const noexcept
```

The largest |final heading error|, in RADIANS, over every motion boundary that produced path data; 0 while runHasHeadingData() is false. BOUNDARY values only — a 90° turn passes through 90° of error by design, and counting that would bury the story this reports. Never reset: one scheduler is one run.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:843`](../../include/shulib/motion/motion_scheduler.hpp#L843).*

<a id="motionscheduler-runfinalheadingdrift"></a>

### `MotionScheduler::runFinalHeadingDrift`

```cpp
[[nodiscard]] units::AngleDim runFinalHeadingDrift() const noexcept
```

|final heading error|, in RADIANS, at the LAST boundary that produced path data — where the run's heading actually LANDED, as opposed to its worst moment. 0 while runHasHeadingData() is false, which is not the same as a run that landed square.

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:849`](../../include/shulib/motion/motion_scheduler.hpp#L849).*

<a id="motionscheduler-attribution"></a>

### `MotionScheduler::attribution`

```cpp
[[nodiscard]] const diag::TickAttribution* attribution() const noexcept
```

The D-3 attribution instrument, when enabled (nullptr when off).

*function, declared at [`include/shulib/motion/motion_scheduler.hpp:854`](../../include/shulib/motion/motion_scheduler.hpp#L854).*

<a id="motionscheduler-kmaxstalledpaces"></a>

### `MotionScheduler::kMaxStalledPaces`

```cpp
static constexpr int kMaxStalledPaces = 100
```

Consecutive pace() calls that may fail to advance the clock before the scheduler declares the pacer broken (header: nothing may hang). Pure logic constant — no hardware claim, hence no register entry.

*field, declared at [`include/shulib/motion/motion_scheduler.hpp:861`](../../include/shulib/motion/motion_scheduler.hpp#L861).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 163 lines, click to expand</summary>

```text

 MotionScheduler — the thing that actually runs a routine (chunk C2, WS6/M2).

 C1 made the library able to execute ONE motion; every routine test hand-rolled
 the loop. This class is that loop, formalized: exactly one active motion at a
 time, started without blocking (async), waited on (waitUntilSettled /
 waitUntil), stopped into a defined safe state (cancel), with the fault policy
 C1 explicitly deferred ("a motion raises but does not self-abort — the
 scheduler owns cancellation"). C4's Chassis facade wraps these verbs; F6
 freezes their shape at D2 — everything here is what that facade will inherit.

 ── Who owns the loop (the A2/C1 controller-first shape, formalized) ────────────────
 One scheduler tick is EXACTLY the loop C1's rig documented:

     localizer.update();          // the estimate advances FIRST (sees time t)
     loopMonitor.tick();          // timing pathology → LOOP_OVERRUN, visibly
     active ? active->tick()      // the motion reads the world and commands
            : idle work;          // no motion: HealthMonitor + an idle record
     <the world advances to t+dt> // via the injected ITickPacer (below)

 The scheduler NEVER owns time. Advancing the world is the pacer's job — in
 host sim that is stepping the A2 plant; on the robot it is delaying to the
 next tick boundary. This is the same inversion as IClock: the scheduler is
 deterministic because it can only observe time, never make it.

 ── One active motion — structural, in two layers ───────────────────────────────────
 (1) The scheduler has ONE active slot and no queue. Starting a motion while
     one is active PRE-EMPTS: the old motion is cancel()led — which puts the
     drivetrain in the cancel safe state AND renders the old object inert
     (post-cancel ticks are contractual no-ops that touch no motor) — and only
     then is the new motion armed. There is no tick on which both command.
     * Rejected REJECT-while-active: a routine that forgot one
       waitUntilSettled() would silently SKIP a move — the robot does the
       wrong thing quietly, the worst failure class.
     * Rejected QUEUE: a queue is F2's Sequence combinator in disguise
       (explicitly out of scope), and a latent queued motion firing seconds
       later is scarier than last-command-wins. Pre-empt matches how a robot
       is commanded: a NEW order supersedes the old one, safely.
 (2) The pre-empted object itself cannot re-command even if a stale caller
     still ticks it — enforcement lives in the MOTION (motion.hpp cancel
     contract), not in scheduler bookkeeping. Direct IMotion use that never
     touches a scheduler remains possible at Tier 3 by design; C4's facade is
     what closes that door for library users.

 ── cancel(): the defined safe state ────────────────────────────────────────────────
 Every cancel path lands in applyCancelSafeState() (motion.hpp): zero volts +
 BrakeMode::Brake, commanded SYNCHRONOUSLY inside the call — no further tick
 is required for the drivetrain to be safe (a cancel that depends on someone
 continuing to tick is a cancel that can leave motors energized). Scheduler
 cancel() with NO active motion still applies the safe state: the panic stop
 must always work. Rationale for brake-not-coast/hold: motion.hpp. Real-world
 braking efficacy is a registered hardware claim (A4: HA-53).

 ── The fault policy (C1's named deferral, decided here) ────────────────────────────
 After each Running tick the scheduler checks whether any fault in
 `abortFaultMask` was raised SINCE THIS MOTION STARTED (per-code raiseCount
 snapshots — a since-clear bitmask cannot see a re-raise, and a dead encoder
 that faulted in motion 1 must still abort motion 2). On a hit: the motion is
 cancelled into the safe state, one Warn line names the causal code, and the
 boundary records it (CompletedMotion.abortFault). The RUN continues — faults
 log and recover, they never crash (fault.hpp); what the next motion does
 about a fault-aborted predecessor is the caller's strategy.

 Default mask = ODO_STUCK only. The reasoning, per code:
   * ODO_STUCK  → ABORT. The spin-vs-motion cross-check says the estimate is
     LYING (dead encoder) or the robot is physically stalled. Continuing to
     servo against a lying estimate is worse than stopping: the controller
     integrates toward a phantom target at full authority. Brake and hand the
     decision back to the routine.
   * IMU_LOST   → CONTINUE. C1 pinned that Degraded does NOT gate: encoders
     are still good, and freezing mid-run strands the robot mid-field (the
     Localizer's own D8 choice). The watchdog + tolerances bound the damage.
   * BROWNOUT   → CONTINUE. A power collapse zeroes effective volts at the
     firmware; the ESTIMATE is still honest. If the pack bounces back,
     finishing the motion is the right outcome; if not, the motion's watchdog
     times it out. Aborting would convert a transient sag into a dead run.
   * GPS_GATE_REJECT → CONTINUE. The gate already did its job — the lying fix
     was rejected and the estimate is protected. Aborting on a defended attack
     would let a flaky sensor end routines.
   * MOTOR_OVER_TEMP / LOOP_OVERRUN → CONTINUE. Degraded authority / timing,
     not a lying estimate; both are visible, bounded, and watchdog-contained.
   * MOTION_TIMEOUT never needs the mask: the motion already exited TimedOut.
 The mask is configurable because this is policy, not physics — a team may
 legitimately choose abort-on-brownout for a strategy that prefers parking.

 ── The task-boundary catch (check.hpp's contract, discharged here) ─────────────────
 core/check.hpp promises: on-robot, a PreconditionError thrown mid-motion is
 "caught by the motion scheduler at the task boundary and converted to a
 FAULT_ABORT exit + a safe drivetrain state". This tick loop is that boundary.
 The catch is TIGHT — PreconditionError only, wrapped around active->tick()
 only (swallowing arbitrary exceptions would hide real bugs; a Localizer
 breach has no motion boundary to unwind to and propagates). If the installed
 handler already raised (the on-robot policy), no second raise happens — the
 faultCount snapshot disambiguates. One bad reading degrades one motion to a
 fault code; it never aborts the auton.

 ── Nothing may hang ────────────────────────────────────────────────────────────────
   * waitUntilSettled() is bounded by the MOTION's watchdog — C1 proved (and
     mutation-guarded) that no motion can fail to exit. No second timeout.
   * waitUntil(pred, timeout) REQUIRES an explicit finite timeout — every
     bound is caller-visible, and there is no invented default constant. The
     return distinguishes Satisfied from TimedOut; a timeout logs one Warn
     line but raises NO fault (a timed-out wait is a legitimate strategy
     branch — "wait for the ring, else move on" — not a pathology).
   * The predicate is checked BEFORE the first tick: true-on-entry returns
     Satisfied having ticked zero times; timeout 0 is an honest poll.
   * A broken pacer that never advances the clock would hang EVERY bound —
     watchdogs read the same frozen clock — so the scheduler counts
     consecutive non-advancing paces and fails the kMaxStalledPaces
     precondition loudly instead of spinning forever.

 ── Re-entrancy (decided and pinned) ────────────────────────────────────────────────
   * async()/cancel() from inside a waitUntil predicate: ALLOWED. Pre-emption
     applies normally; the swap happens between ticks, so the single-command
     invariant holds, and the wait simply continues over the new state.
   * cancel() from inside the pacer's pace(): ALLOWED, and pinned since F2 —
     pace() runs between ticks (inTick_/inBoundary_ both false), so the
     precondition set always admitted it; it was undocumented and untested
     until F2's end-of-run guard came to RELY on it (the deadline cut: a
     cancel from pace() unwinds waitUntilSettled on the same iteration, with
     zero latency — measured). Pinned by test so a later chunk cannot break
     the guard by tightening this precondition. async() from pace() is
     equally precondition-legal and measured to WORK — and is REJECTED as a
     pattern: the hijacked wait keeps looping over the new motion and returns
     ITS verdict as the original caller's (a moveTo that was cut reports
     Settled describing a motion the caller never issued, with zero log
     lines). The guard never uses it; nothing should.
   * A BLOCKING verb (waitUntil / waitUntilSettled / tick) from inside a
     predicate — or from pace(): REJECTED by precondition — disguised
     recursion whose depth is user-data-dependent. Nothing in the G2 marker
     use case needs it; relaxing later is additive, un-forbidding is not.
   * async()/cancel() from inside a motion's tick(): REJECTED by precondition
     — mutating the active slot while active->tick() is on the stack.

 ── Unwind safety of the blocking waits (F2, closing C4's known gap at its root) ───
 A throw through a blocking wait (stalled-pace precondition, a Localizer
 breach, a throwing predicate) used to leave the active motion ARMED and the
 motors at their last command: C4 documented the consequence for
 waitUntilSettled (a verb's stack-owned motion dangling in the slot) and
 patched it in the FACADE with runBlocking's DetachGuard — but the same hole
 was open for direct Tier-3 waits and for waitUntil, where F2 measured the
 worst state in the campaign: 11.4 V under Coast with the slot pointing at a
 dying stack object (Chassis::waitUntil is a bare pass-through). The fix now
 lives HERE, in the loop owner: both waits cancel on unwind (safe state +
 boundary recorded + slot cleared) before the exception propagates. The
 facade's DetachGuard stays — redundant cancels are idempotent, and belt
 plus braces is the right dress code for the panic path.

 ── Observability (what C5 will need; A1's cost contract respected) ─────────────────
 The scheduler assigns activeCommandId (debug_record.hpp: "ids are assigned by
 the motion scheduler") by interposing CommandIdStampSink between the motions
 and the real sink: every record emitted while a motion is active is stamped
 with its id — unforgettably, for every motion type, including future ones.
 Construct motions from scheduler.deps() so their records route through the
 stamp (the C4 facade will make this plumbing automatic; flagged for F6). The
 stamper forwards wantsRecord() — the pair rule — so a NullSink run still
 skips record population entirely. Idle ticks emit a quiet record (no
 invented target) so the stream stays continuous between motions; motion
 boundaries surface as CompletedMotion + per-exit counters, NOT as formatted
 result lines — that formatting is C5's, deliberately not built here.

 Single-task by contract, like everything it composes. Not copyable/movable:
 it holds a self-referential context (the stamped telemetry route).
```

</details>
