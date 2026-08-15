#pragma once
//
// The bounded mechanism operation (chunk F1, WS7/M4): IMechanismOp + the two
// season-free operations every scoring verb decomposes into — run a motor
// mechanism until something confirms (RunUntilConfirmed) and fire a discrete
// actuator, wait for it to physically happen, then check (ActuateAndConfirm).
// F3's concrete primitives (intakeUntilCapture, clampActuate+clampConfirm,
// deployActuator, …) are these two shapes with real sensors and real
// thresholds; F2's combinators interleave, pre-empt and time-bound them. This
// header is the contract both of those chunks are built against.
//
// The shape MIRRORS motion/motion.hpp — deliberately, so nobody maintains two
// near-identical contracts that quietly diverge — and every place the mirror
// STOPS is named here, because an undocumented divergence between two contracts
// this similar is a bug factory:
//
// ── The tick contract (identical to IMotion's) ──────────────────────────────────────
// An operation does NOT own a loop, a task, or a clock cadence. The loop owner
// ticks it: a caller's own loop, or — the documented idiom for blocking use —
// the scheduler's existing wait primitive, with the operation ticked from the
// predicate:
//
//     op.start();
//     chassis.waitUntil([&] { return op.tick() != MechanismOutcome::Running; },
//                       units::Time{2.0});
//     // op.outcome() is the verdict
//
// That one line IS the blocking form. There is deliberately NO blocking helper
// in this layer: the predicate idiom inherits every C2 guard for free (the
// required finite timeout, the stalled-pace precondition, the no-blocking-
// verbs-inside-a-predicate re-entrancy rule), it runs concurrently with an
// active motion (the scheduler keeps ticking it — "intake while driving"), and
// a second loop owner that could deadlock against the first NEVER EXISTS. A
// helper that owned its own pace loop would advance the world underneath a
// scheduler that thinks it is mid-predicate; rather than make that safe in
// every position a caller could put it, the tick form is the only form (the
// C2 stalled-pacer precedent: make the un-ownable loud or make it impossible).
//
// After a non-Running verdict the operation is FINISHED: further tick() calls
// are safe no-ops that return the cached verdict and command nothing (the
// mechanism is already in its safe state). start() fully re-arms — an
// operation object is reusable (retry a grab).
//
// ── The cancel contract (mirrored from motion.hpp, divergences named) ───────────────
// Identical in every clause:
//   * RUNNING: the mechanism is put in its safe state synchronously, the
//     verdict becomes Cancelled, the operation is FINISHED and inert.
//   * Already FINISHED: the safe state is STILL applied (cancel() means "make
//     it safe NOW" and must be idempotent) but the verdict is PRESERVED — an
//     operation that succeeded really did succeed; rewriting history would lie
//     to whoever reads the outcome later (C2's exact ruling for motions).
//   * Never started: complete no-op — an unstarted operation has no
//     relationship to its mechanism yet; commanding it would be the surprise.
//   * cancel() raises NO fault: cancellation is a commanded, normal act.
// The ONE divergence: the safe state applied is the MECHANISM'S DECLARED one
// (mechanism.hpp, T4), not the drivetrain's 0 V + Brake — a loaded lift ends in
// Hold, a jammed intake does not. One definition per mechanism instead of one
// for all is the entire point of T4.
//
// ── Which exits apply the safe state — split by actuation physics (T4) ──────────────
// MOTOR operations end in the declared safe state on EVERY path — Succeeded,
// Stalled, TimedOut, Cancelled: "capture, and the intake stops"; "arrive, and
// the lift holds" (safe mode Hold + 0 V IS the hold). A motor left energized is
// the forgettable-safety-step failure mode A1 exists to prevent, so no exit may
// depend on the caller remembering to stop it. An open-ended behavior — keep
// the intake spinning while driving to the next goal — is deliberately NOT an
// operation: command the mechanism directly (mech.setVoltage(...)) and own the
// stop yourself.
// DISCRETE operations are different, and the difference was caught by writing
// the expected timeline before writing the code: a clamp whose declared safe
// command is "open" would FLING ITS GOAL the instant a grab SUCCEEDED if
// success applied the safe state. A solenoid consumes no energy holding a
// state, so the motor rationale does not transfer; un-commanding a completed
// actuation would undo the very act. So ActuateAndConfirm leaves the COMMANDED
// state in place on Succeeded and Unconfirmed (the completed action persists;
// an unconfirmed one stays put for the caller's retry/undo decision, loudly
// reported), and applies the declared safe state ONLY on cancel() — the
// outside hammer, which is exactly the park-guard path, where the DECLARED
// value is how a team says "clamp stays closed at the buzzer" vs "cylinder
// retracts inside the expansion limit".
//
// ── The watchdog (C1's discipline, one divergence named) ────────────────────────────
// RunUntilConfirmed arms a control::Watchdog in start(); NO code path disarms
// it; a never-confirming world exits TimedOut. ActuateAndConfirm's bound is its
// DEADLINE PAIR (actuation deadline + confirm deadline), both computed ONCE in
// start() from a monotonic clock and never extended — structurally the same
// guarantee (armed at start, nothing disarms it) without a redundant second
// timer that no test could ever see fire. Under an adversarial clock (jumps,
// erratic dt) every path still exits: deadlines are absolute instants, not
// accumulated dt. A FROZEN clock is the loop owner's pathology, not the
// operation's — the scheduler's stalled-pace guard turns it into a loud
// precondition (C2), and an operation cannot hang a loop it does not own.
//
// ── Faults vs verdicts (T6 — the line, drawn) ───────────────────────────────────────
//   * Stalled  → FaultCode::MechanismStalled IS raised. High current + a shaft
//     that will not turn is the robot being unwell (a jam, a bind, a motor
//     cooking toward thermal throttle) — triage material.
//   * TimedOut → NO fault, one Warn line. This DIVERGES from MotionTimeout on
//     purpose, and the argument matters: a motion has authority over its own
//     success — the drive not arriving means the robot is unwell or the
//     estimate lies, so C1 latches it. A mechanism's success routinely depends
//     on the WORLD cooperating: "spin until a ring arrives, 3 s budget" timing
//     out with a healthy, unjammed mechanism means no ring came — strategy,
//     not pathology. Latching that would flood first-fault triage with normal
//     outcomes (the same reasoning as waitUntil's no-fault timeout, which is
//     the precedent followed here). The genuinely-unwell timeout — stuck but
//     not stalling — is indistinguishable from the world not cooperating at
//     this layer; F3's primitives, which know what "should have happened",
//     may mint sharper codes when they can prove them.
//   * Unconfirmed → NO fault, one Warn line: healthy mechanism, failed task.
//   * Succeeded / Cancelled → silent (a normal act needs no line).
//
// ── Where the mirror of IMotion stops (the divergences, in one list) ────────────────
//   * Verdicts are MechanismOutcome, not ExitReason (mechanism_outcome.hpp).
//   * No wait-for-live-estimate contract and no MotionState: an operation
//     never reads the pose estimate, so there is no boot window to wait
//     through and no estimate-derived target to capture.
//   * No per-tick DebugRecord emission and no HealthMonitor tick: the loop
//     owner (scheduler / caller) already owns both; an operation emitting its
//     own records would double-count the tick.
//   * The safe state is per-mechanism (above).
//   * One-operation-per-mechanism is enforced by the mechanism's claim token
//     (mechanism.hpp): start() takes the claim or trips a loud precondition;
//     every exit releases it. C2 enforced one-active-motion with pre-empt
//     policy in the scheduler; the policy layer for mechanisms (cancel the old
//     operation, then start the new one) is F2's, so F1 ships only the
//     structural impossibility of a silent double-drive.

#include <cmath>
#include <type_traits>
#include <utility>

#include "shulib/control/watchdog.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/manipulation/stall_detector.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::manipulation {

/// The dependencies every mechanism operation shares, as NAMED pointers
/// (designated initializers at the call site), validated non-null — the
/// MotionDeps pattern, not a third convention. Deliberately SMALLER than
/// MotionDeps: no localizer (no estimate is read), no kinematics (nothing to
/// desaturate), no HealthMonitor (the loop owner ticks it). All pointees must
/// outlive the operation.
struct MechanismDeps {
    hal::IClock* clock = nullptr;            ///< watchdog / deadlines / stall window
    diag::FaultLatch* faults = nullptr;      ///< run-scoped latch (MechanismStalled)
    hal::ITelemetrySink* telemetry = nullptr;  ///< the Warn lines (TimedOut/Unconfirmed)

    /// Every dependency non-null, or a loud precondition naming the one that is
    /// not. Each operation calls it at CONSTRUCTION, so a forgotten pointer is an
    /// error at the wiring site rather than a null dereference on the first tick.
    void validate() const {
        SHULIB_PRECONDITION(clock != nullptr, "MechanismDeps: clock is null");
        SHULIB_PRECONDITION(faults != nullptr, "MechanismDeps: faults is null");
        SHULIB_PRECONDITION(telemetry != nullptr, "MechanismDeps: telemetry is null");
    }

    /// validate(), then hand out the clock — for a member-initializer list's
    /// FIRST dependency use (MotionDeps' own trick), so a null pointer trips
    /// the precondition instead of being dereferenced.
    [[nodiscard]] hal::IClock& validatedClock() const {
        validate();
        return *clock;
    }
};

/// The operation contract (file banner). Pure-virtual cancel() ON PURPOSE,
/// exactly as IMotion rules: an operation type without a cancellation story is
/// the forgettable-safety-step failure mode; every implementer must state one.
///
/// Since F2 the contract IS a hal::ICancellable: an operation registers itself
/// on the mechanism's claim at start() (tryClaim(*this)), which is how the
/// end-of-run guard — holding only span<hal::IMechanism*> — reaches a live
/// operation and renders it inert at the deadline (mechanism.hpp's claimant-
/// hook banner carries the measured failure this closes). cancel()'s meaning
/// is unchanged; the base only makes it reachable from the hal tier.
class IMechanismOp : public hal::ICancellable {
public:
    /// Interface plumbing: the destructor is virtual through hal::ICancellable, so
    /// an operation may be owned and destroyed through either base, and the rest
    /// are restated because declaring a destructor suppresses the implicit moves.
    /// Defaulting them here is base-class boilerplate, NOT permission to copy an
    /// operation — a concrete operation holds a mechanism claim and a claimant
    /// registration pointing at itself, so both library operations delete all four.
    ~IMechanismOp() override = default;
    IMechanismOp() = default;
    IMechanismOp(const IMechanismOp&) = default;
    IMechanismOp(IMechanismOp&&) = default;
    IMechanismOp& operator=(const IMechanismOp&) = default;
    IMechanismOp& operator=(IMechanismOp&&) = default;

    /// Arm the operation: claim the mechanism (loud precondition if another
    /// operation holds it), reset detectors, arm the watchdog/deadlines.
    /// Re-callable — a finished operation re-arms completely.
    virtual void start() = 0;

    /// One step (see the tick contract). Precondition: start() has been called.
    [[nodiscard]] virtual MechanismOutcome tick() = 0;

    /// Stop from outside (see the cancel contract). Idempotent; never raises;
    /// applies the mechanism's declared safe state whenever started. Overrides
    /// hal::ICancellable — this is the member the claimant hook exposes.
    void cancel() override = 0;

    /// The verdict of the most recent tick (Running before the first, and
    /// before start()).
    [[nodiscard]] virtual MechanismOutcome outcome() const noexcept = 0;

    /// True once start() has been called (stays true after an exit).
    [[nodiscard]] virtual bool started() const noexcept = 0;

    /// Stable short name for logs / step labels (e.g. "grab").
    [[nodiscard]] virtual const char* name() const noexcept = 0;

    /// Convenience: exited with a real verdict (defined from the virtuals so it
    /// can never disagree with them).
    [[nodiscard]] bool finished() const noexcept {
        return started() && outcome() != MechanismOutcome::Running;
    }
};

/// A caller-supplied confirmation that always holds: "completion IS the
/// confirmation" — for fire-and-forget actions (deploy an actuator) where no
/// sensor exists to ask. Using it is a visible, greppable declaration that an
/// action is unverified, which is exactly what an invisible default would hide.
struct AlwaysConfirmed {
    /// Always true, on every call. In ActuateAndConfirm that means "confirmed the
    /// instant the actuation deadline passes"; in RunUntilConfirmed it means the
    /// FIRST tick Succeeds with zero volts ever commanded — which is why this
    /// belongs on discrete actuators and is a mistake on a motor operation.
    [[nodiscard]] bool operator()() const noexcept { return true; }
};

/// RunUntilConfirmed's three numbers, all REQUIRED and all validated at
/// construction — there is no library default for a voltage, a budget or a jam
/// threshold, because every one of them is a fact about one mechanism on one
/// robot. Zero-initializing this struct does not get you a working operation; it
/// gets you a loud precondition.
struct RunUntilConfirmedConfig {
    /// Commanded while running (finite, non-zero — a 0 V "run" is a nonsense
    /// request, refused loudly at construction).
    units::Voltage voltage;
    /// Watchdog budget (finite, > 0). Typed time, the D2 discipline.
    units::Time timeout;
    /// Jam/stall thresholds — REQUIRED, per mechanism, no library defaults
    /// (stall_detector.hpp says why).
    StallConfig stall;
};

/// Run a motor mechanism at a fixed voltage until a caller-supplied
/// confirmation holds. The season-free skeleton of intakeUntilCapture and
/// every "spin until the sensor says so" verb. Possible verdicts: Succeeded /
/// Stalled / TimedOut / Cancelled. `Unconfirmed` is UNREACHABLE here by
/// construction and that is honest: an open-ended run has no notion of "the
/// action completed" separate from its confirmation, so a world that never
/// confirms is a timeout, not a completed-but-unconfirmed act (contrast
/// ActuateAndConfirm, where actuation completing is a fact of its own).
///
/// WHAT CONFIRMS is deliberately a caller-supplied predicate: hue for a
/// Toggle, proximity for a cup, a current signature for a capture — that
/// meaning is season- and robot-specific and belongs above this layer
/// (hal/vision.hpp's opaque-classId house rule, applied to actions). The
/// predicate is TRUSTED: a predicate that lies "confirmed" produces a false
/// Succeeded, which is why F3's primitives must confirm on real sensors —
/// the F1 hostile suite demonstrates the boundary rather than pretending to
/// close it. Checked at the TOP of every tick, before any command (true on
/// entry ⇒ Succeeded with zero volts ever commanded — already holding the
/// ring must not spin the intake; the C2 pred-before-first-tick shape).
template <typename Confirm>
class RunUntilConfirmed final : public IMechanismOp {
    static_assert(std::is_invocable_r_v<bool, Confirm&>,
                  "RunUntilConfirmed: Confirm must be callable as bool()");

public:
    /// `mech` must outlive the operation; `opName` must be a stable literal.
    RunUntilConfirmed(hal::MotorMechanism& mech, const MechanismDeps& deps,
                      const RunUntilConfirmedConfig& config, Confirm confirm,
                      const char* opName = "runUntilConfirmed")
        : watchdog_{validateConfig(config, deps).value(), deps.validatedClock()},
          mech_{&mech},
          deps_{deps},
          cfg_{config},
          confirm_{std::move(confirm)},
          stall_{config.stall},
          name_{opName} {
        SHULIB_PRECONDITION(opName != nullptr, "RunUntilConfirmed: name is null");
    }

    /// F2 (rule-of-three, found building the end-of-run guard): an operation
    /// destroyed MID-FLIGHT — the timeout-mismatch idiom, where the caller's
    /// wait gives up before the op's own watchdog — previously left its
    /// mechanism CLAIMED FOREVER and ENERGIZED at the last commanded voltage:
    /// no code path stopped it, and the stuck claim made the end action's own
    /// operation throw at start(). Destruction now cancels a still-running
    /// operation (declared safe state, claim + claimant released). A FINISHED
    /// operation is deliberately untouched: cancel() would re-apply the safe
    /// state, and on a discrete actuator that un-does the completed action
    /// (the T4 persist rule).
    ~RunUntilConfirmed() override {
        if (started_ && !finished_) {
            cancel();
        }
    }

    /// Non-copyable/non-movable (F2): the claim is a resource and the
    /// mechanism's registered claimant points at THIS object — a copy would
    /// double-release the claim and a move would leave the registration
    /// dangling. Construct where it will live; start() re-arms for retries.
    RunUntilConfirmed(const RunUntilConfirmed&) = delete;
    RunUntilConfirmed(RunUntilConfirmed&&) = delete;
    RunUntilConfirmed& operator=(const RunUntilConfirmed&) = delete;
    RunUntilConfirmed& operator=(RunUntilConfirmed&&) = delete;

    /// Claim the mechanism and register this object as its claimant (loud
    /// precondition if another operation already holds it — cancel that one
    /// first), forget any partial stall window, and arm the watchdog HERE; no code
    /// path disarms it, so a world that never confirms still exits. Re-callable: a
    /// finished operation re-arms completely for a retry, and a claim this object
    /// already holds is kept rather than re-taken. Commands nothing by itself.
    void start() override {
        if (!holdsClaim_) {
            SHULIB_PRECONDITION(mech_->tryClaim(*this),
                                "RunUntilConfirmed::start: mechanism already driven by "
                                "another operation (cancel it first — F2's pre-empt policy)");
            holdsClaim_ = true;
        }
        stall_.reset();
        watchdog_.start();  // armed HERE; no code path disarms it (C1)
        outcome_ = MechanismOutcome::Running;
        started_ = true;
        finished_ = false;
    }

    /// One step, in an order pinned by test: confirmation FIRST (success outranks a
    /// simultaneous stall or timeout), then the stall detector, then the watchdog;
    /// the configured voltage is commanded only when none of the three fired — so a
    /// confirmation already true on entry Succeeds with zero volts ever reaching the
    /// motors. EVERY exit applies the mechanism's declared safe state and releases
    /// the claim, so a caller can never be left owing a stop. Once finished it is a
    /// no-op returning the cached verdict. Precondition: start() has been called.
    [[nodiscard]] MechanismOutcome tick() override {
        SHULIB_PRECONDITION(started_, "RunUntilConfirmed::tick: start() not called");
        if (finished_) {
            return outcome_;  // safe no-op; mechanism already in its safe state
        }
        // Order is deliberate and pinned by test: confirm FIRST (success takes
        // priority over a simultaneous stall/timeout — ExitGroup's rule), then
        // stall (the more specific failure), then the watchdog.
        if (confirm_()) {
            return finish(MechanismOutcome::Succeeded);
        }
        if (stall_.update(deps_.clock->now(), mech_->maxCurrent(), mech_->meanVelocity())) {
            raiseStalled();
            return finish(MechanismOutcome::Stalled);
        }
        if (watchdog_.expired()) {
            warnLine("TIMED_OUT (watchdog) — verdict only, no fault (T6)");
            return finish(MechanismOutcome::TimedOut);
        }
        mech_->setVoltage(cfg_.voltage);
        return MechanismOutcome::Running;
    }

    /// Make the mechanism safe NOW: the declared safe state is applied on every
    /// call made after start(), idempotently, but the VERDICT is written only if
    /// the operation was still running — an operation that Succeeded stays
    /// Succeeded, because rewriting history would lie to whoever reads outcome()
    /// later. Before start() it is a complete no-op. Raises no fault: cancellation
    /// is a commanded, normal act. This is also what the end-of-run guard reaches
    /// through the mechanism's registered claimant.
    void cancel() override {
        if (!started_) {
            return;  // never started: complete no-op (the mirror's third clause)
        }
        // "Make it safe NOW", idempotent — but ONLY on a mechanism this operation still owns,
        // or one nobody owns. A finished operation has already released its claim, so an
        // unguarded call here let a RETAINED, EXITED object safe a mechanism a DIFFERENT live
        // operation had since claimed: op2 running at 12 V dropped to brake + 0 V, silently,
        // no fault, no Warn — and op2 re-commands its voltage next tick but NOT its brake
        // mode, which is the half-safe state run_guard's T6 note names as the reason
        // applySafeState() alone is never trusted. The `!claimed()` arm keeps the documented
        // idempotence for the ordinary case (nobody else took it), which is what the existing
        // "completed verdict is PRESERVED — cancel still re-safes" test pins.
        if (holdsClaim_ || !mech_->claimed()) {
            mech_->applySafeState();
        }
        if (!finished_) {
            releaseClaim();
            outcome_ = MechanismOutcome::Cancelled;
            finished_ = true;
        }
        // already finished: verdict PRESERVED (the mirror's second clause)
    }

    /// The verdict of the most recent tick, cached: Running before the first tick
    /// and before start(), then frozen at whichever of Succeeded / Stalled /
    /// TimedOut / Cancelled ended the operation, until the next start() re-arms.
    [[nodiscard]] MechanismOutcome outcome() const noexcept override { return outcome_; }
    /// True from the first start() onward — including after an exit, and after a
    /// cancel(). Nothing clears it, so it answers "has this ever run?", not "is it
    /// running?" (pair it with outcome() != Running, or just call finished()).
    [[nodiscard]] bool started() const noexcept override { return started_; }
    /// The `opName` handed to the constructor, returned unchanged and not copied.
    /// It is what the MechanismStalled fault detail and the timeout Warn line
    /// quote, so it is the string you grep a transcript for.
    [[nodiscard]] const char* name() const noexcept override { return name_; }

private:
    /// Validate config before the member-initializer list needs it (the
    /// watchdog is constructed first and wants a validated timeout).
    static const units::Time& validateConfig(const RunUntilConfirmedConfig& cfg,
                                             const MechanismDeps& deps) {
        deps.validate();
        SHULIB_PRECONDITION(std::isfinite(cfg.voltage.value()) && cfg.voltage.value() != 0.0,
                            "RunUntilConfirmed: voltage must be finite and non-zero");
        SHULIB_PRECONDITION(std::isfinite(cfg.timeout.value()) && cfg.timeout.value() > 0.0,
                            "RunUntilConfirmed: timeout must be finite and > 0");
        return cfg.timeout;
    }

    MechanismOutcome finish(MechanismOutcome o) {
        mech_->applySafeState();  // EVERY exit path lands here (banner)
        releaseClaim();
        outcome_ = o;
        finished_ = true;
        return o;
    }

    void releaseClaim() noexcept {
        mech_->releaseClaim();
        holdsClaim_ = false;
    }

    void raiseStalled() {
        diag::lineformat::Line d;
        d.appendLiteral("op '");
        d.appendLiteral(name_);
        d.appendLiteral("' mech '");
        d.appendLiteral(mech_->name());
        d.appendLiteral("'");
        deps_.faults->raise(diag::FaultCode::MechanismStalled, "MECH", d.view());
    }

    void warnLine(const char* what) {
        diag::lineformat::Line line;
        line.appendLiteral("'");
        line.appendLiteral(name_);
        line.appendLiteral("' on '");
        line.appendLiteral(mech_->name());
        line.appendLiteral("': ");
        line.appendLiteral(what);
        deps_.telemetry->log(hal::LogLevel::Warn, "MECH", line.view());
    }

    control::Watchdog watchdog_;
    hal::MotorMechanism* mech_;
    MechanismDeps deps_;
    RunUntilConfirmedConfig cfg_;
    Confirm confirm_;
    StallDetector stall_;
    const char* name_;
    MechanismOutcome outcome_ = MechanismOutcome::Running;
    bool started_ = false;
    bool finished_ = false;
    bool holdsClaim_ = false;
};

/// ActuateAndConfirm's schedule: WHAT to command, how long the hardware needs to
/// do it, and how long afterwards proof may take to arrive. Both times are
/// validated finite and >= 0 at construction and become ABSOLUTE instants at
/// start() — they are this operation's watchdog, so no later path can extend
/// them. The physical actuation time is a measured property of the cylinder, not
/// a number the library can supply.
struct ActuateAndConfirmConfig {
    /// The commanded line state (what it means physically is the mechanism's).
    bool target = true;
    /// How long the actuation physically takes before the world can honestly be
    /// asked about it (finite, >= 0). The confirmation is NOT consulted before
    /// this deadline: a clamp's "closed" sensor may still be reporting the
    /// PREVIOUS grab, and confirming on the pre-actuation state is the silent-
    /// success door. Invented values are R4's to measure; register them (HA).
    units::Time actuationTime;
    /// How long after actuation the confirmation may take to arrive (finite,
    /// >= 0; 0 = one check exactly at the actuation deadline). Expiring with
    /// the confirmation still false is the Unconfirmed verdict.
    units::Time confirmWindow;
};

/// Fire a discrete actuator, wait out its physical actuation time, then
/// require a caller-supplied confirmation within a bounded window. The
/// season-free skeleton of clampActuate+clampConfirm and deployActuator — and
/// the operation that makes `Unconfirmed` REAL: the command completed (time
/// passed, the solenoid was told), the mechanism is healthy, and the world
/// reports the thing did not happen. Possible verdicts: Succeeded /
/// Unconfirmed / Cancelled. `Stalled` is unreachable (no current channel
/// exists on a solenoid — digital_out.hpp) and `TimedOut` is unreachable
/// because the deadline pair IS the watchdog (file banner): both instants are
/// fixed at start() and a monotonic clock reaches them. With
/// AlwaysConfirmed, completion is the confirmation (deploy — visibly
/// unverified by declaration).
///
/// ON EXIT (the T4 split, banner): Succeeded and Unconfirmed LEAVE THE
/// COMMANDED STATE IN PLACE — the completed actuation persists (a successful
/// grab must not be un-grabbed by its own success); only cancel() applies the
/// declared safe state.
template <typename Confirm>
class ActuateAndConfirm final : public IMechanismOp {
    static_assert(std::is_invocable_r_v<bool, Confirm&>,
                  "ActuateAndConfirm: Confirm must be callable as bool()");

public:
    /// `mech` must outlive the operation; `opName` must be a stable literal.
    ActuateAndConfirm(hal::PneumaticMechanism& mech, const MechanismDeps& deps,
                      const ActuateAndConfirmConfig& config, Confirm confirm,
                      const char* opName = "actuateAndConfirm")
        : mech_{&mech}, deps_{deps}, cfg_{config}, confirm_{std::move(confirm)}, name_{opName} {
        deps_.validate();
        SHULIB_PRECONDITION(std::isfinite(cfg_.actuationTime.value()) &&
                                cfg_.actuationTime.value() >= 0.0,
                            "ActuateAndConfirm: actuationTime must be finite and >= 0");
        SHULIB_PRECONDITION(std::isfinite(cfg_.confirmWindow.value()) &&
                                cfg_.confirmWindow.value() >= 0.0,
                            "ActuateAndConfirm: confirmWindow must be finite and >= 0");
        SHULIB_PRECONDITION(opName != nullptr, "ActuateAndConfirm: name is null");
    }

    /// Cancel-on-destruction for a MID-FLIGHT operation; a finished one is
    /// untouched (RunUntilConfirmed's destructor note — the T4 persist rule
    /// is the reason the guard is conditional, and it matters MOST here: an
    /// unconditional cancel would force the declared safe state onto every
    /// successfully-grabbed clamp the moment its operation went out of scope).
    ~ActuateAndConfirm() override {
        if (started_ && !finished_) {
            cancel();
        }
    }

    /// Non-copyable/non-movable (F2): same claim-resource reasoning as
    /// RunUntilConfirmed.
    ActuateAndConfirm(const ActuateAndConfirm&) = delete;
    ActuateAndConfirm(ActuateAndConfirm&&) = delete;
    ActuateAndConfirm& operator=(const ActuateAndConfirm&) = delete;
    ActuateAndConfirm& operator=(ActuateAndConfirm&&) = delete;

    /// Claim the mechanism and register this object as its claimant (loud
    /// precondition if another operation already holds it), then compute the
    /// deadline pair ONCE from the clock — actuation, then confirm. Those two
    /// absolute instants ARE this operation's bound: no later path extends them, so
    /// an erratic dt cannot stretch the budget and a clock jump cannot skip it.
    /// Re-callable; a finished operation re-arms for a retry. Commands nothing by
    /// itself — the first tick() is what fires the actuator.
    void start() override {
        if (!holdsClaim_) {
            SHULIB_PRECONDITION(mech_->tryClaim(*this),
                                "ActuateAndConfirm::start: mechanism already driven by "
                                "another operation (cancel it first — F2's pre-empt policy)");
            holdsClaim_ = true;
        }
        // The deadline pair — computed ONCE, from a monotonic clock, never
        // extended by any later code path. This IS the watchdog (banner).
        const units::Time now = deps_.clock->now();
        actDeadline_ = now + cfg_.actuationTime;
        confirmDeadline_ = actDeadline_ + cfg_.confirmWindow;
        outcome_ = MechanismOutcome::Running;
        started_ = true;
        finished_ = false;
    }

    /// One step: re-command the target line state (idempotent — the solenoid holds
    /// it), then ask the confirmation ONLY past the actuation deadline. Before that
    /// instant the operation stays Running WITHOUT consulting the world, because a
    /// clamp's "closed" sensor may still be reporting the previous grab and
    /// confirming on the pre-actuation state is the silent-success door. On exit the
    /// COMMANDED STATE STAYS PUT for both Succeeded and Unconfirmed — a successful
    /// grab must not be un-grabbed by its own success, and an unconfirmed one is
    /// left where it is for the caller's retry/undo decision. The claim is released
    /// on every exit. Once finished it commands nothing. Precondition: start().
    [[nodiscard]] MechanismOutcome tick() override {
        SHULIB_PRECONDITION(started_, "ActuateAndConfirm::tick: start() not called");
        if (finished_) {
            // Inert no-op: commands nothing. The exit already disposed the
            // mechanism (completed state persists; cancel forced safe — banner).
            return outcome_;
        }
        mech_->set(cfg_.target);  // idempotent re-command; the device holds state
        const units::Time now = deps_.clock->now();
        if (now.value() < actDeadline_.value()) {
            return MechanismOutcome::Running;  // mid-actuation: the world can't be asked yet
        }
        if (confirm_()) {
            return finish(MechanismOutcome::Succeeded);
        }
        if (now.value() >= confirmDeadline_.value()) {
            warnLine("UNCONFIRMED — actuation completed, confirmation never arrived; "
                     "verdict only, no fault (T6)");
            return finish(MechanismOutcome::Unconfirmed);
        }
        return MechanismOutcome::Running;  // inside the confirm window
    }

    /// The ONLY path that forces the declared safe state onto this actuator — every
    /// other exit leaves the commanded state in place. Applied on every call made
    /// after start(), idempotently, so it can and does un-do a completed actuation:
    /// that is deliberate, and it is how a team says "the clamp stays closed at the
    /// buzzer" or "the cylinder retracts inside the expansion limit" through the
    /// mechanism's declared safe value. The verdict of an already-finished operation
    /// is preserved; before start() it is a complete no-op; it raises no fault.
    void cancel() override {
        if (!started_) {
            return;  // never started: complete no-op
        }
        // Idempotent — but ONLY on a mechanism this operation still owns, or one nobody owns.
        // Worse here than on the motor side: a finished ActuateAndConfirm deliberately does
        // NOT apply the safe state (finish() leaves a successful actuation standing — the
        // clamp that would FLING its goal), so an unguarded cancel() on a retained, exited
        // object drove the line to the declared safe value and UN-DID an actuation a second,
        // currently-claiming operation had just performed.
        if (holdsClaim_ || !mech_->claimed()) {
            mech_->applySafeState();
        }
        if (!finished_) {
            releaseClaim();
            outcome_ = MechanismOutcome::Cancelled;
            finished_ = true;
        }
        // already finished: verdict PRESERVED
    }

    /// The verdict of the most recent tick, cached: Running before the first tick
    /// and before start(), then frozen at Succeeded, Unconfirmed or Cancelled —
    /// Stalled and TimedOut are unreachable for this operation (no current channel
    /// on a solenoid; the deadline pair is the watchdog).
    [[nodiscard]] MechanismOutcome outcome() const noexcept override { return outcome_; }
    /// True from the first start() onward — including after an exit. Nothing clears
    /// it, so it answers "has this ever run?", not "is it running?".
    [[nodiscard]] bool started() const noexcept override { return started_; }
    /// The `opName` handed to the constructor, returned unchanged and not copied.
    /// The Unconfirmed Warn line quotes it, so it is the string that identifies this
    /// actuation in a transcript.
    [[nodiscard]] const char* name() const noexcept override { return name_; }

private:
    MechanismOutcome finish(MechanismOutcome o) {
        // NO applySafeState here — the commanded state is the completed action
        // and must persist (the T4 split, file banner). cancel() is the only
        // path that forces the declared safe state on a discrete actuator.
        releaseClaim();
        outcome_ = o;
        finished_ = true;
        return o;
    }

    void releaseClaim() noexcept {
        mech_->releaseClaim();
        holdsClaim_ = false;
    }

    void warnLine(const char* what) {
        diag::lineformat::Line line;
        line.appendLiteral("'");
        line.appendLiteral(name_);
        line.appendLiteral("' on '");
        line.appendLiteral(mech_->name());
        line.appendLiteral("': ");
        line.appendLiteral(what);
        deps_.telemetry->log(hal::LogLevel::Warn, "MECH", line.view());
    }

    hal::PneumaticMechanism* mech_;
    MechanismDeps deps_;
    ActuateAndConfirmConfig cfg_;
    Confirm confirm_;
    const char* name_;
    units::Time actDeadline_{0.0};
    units::Time confirmDeadline_{0.0};
    MechanismOutcome outcome_ = MechanismOutcome::Running;
    bool started_ = false;
    bool finished_ = false;
    bool holdsClaim_ = false;
};

}  // namespace shulib::manipulation
