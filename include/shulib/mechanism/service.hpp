#pragma once
//
// mechanism/service.hpp — THE MECHANISM SERVICE: one object, ticked once per control tick, that runs
// every mechanism's behaviour (the mechanism core, chunk MECH-A2, 2026-10-15).
//
// ── Why one ticker ───────────────────────────────────────────────────────────────────
// A mechanism step must keep running while the robot does something else: an intake started before
// a drive must stop the moment its sensor sees a piece MID-drive; a claw step must reach its verdict
// while the robot turns. Nothing in a blocking drive verb ticks a mechanism — the drive's wait is the
// scheduler's, and a student's own loop around it would be a second loop owner. So the behaviours
// live HERE, in one object every control tick reaches through exactly one of two seams:
//   * THE PACER SEAM (autons): the service IS a motion::ITickPacer decorator. A program constructs it
//     around the pacer that advances the world and puts the RunGuard AROUND it — Chassis → RunGuard →
//     MechanismService → (the program's watch) → the tick pacer — so every pace of every blocking verb
//     first lets the guard check its deadlines, then ticks every mechanism, then advances the world
//     (the requirements' order, :116-155).
//   * THE DRIVER LOOP'S STATED PLACE: tickFromDriverLoop(), after the driver's inputs, before the
//     loop's perTick (MECH-A4 wires it into runDriverControl). A pace() that follows such a tick does
//     NOT tick again, so the service ticks EXACTLY ONCE per tick whichever pacer the loop is handed —
//     the raw one, this service, or a guard around it.
//
// ── Each mechanism's SLOT is its claimant ────────────────────────────────────────────
// A slot takes its mechanism's claim (hal/mechanism.hpp) with ITSELF as the registered claimant
// whenever it commands, so the RunGuard's cancel-all — and, since this run, its deadline and abandon
// cut (sequence/run_guard.hpp) — reaches every running behaviour: inert, the declared safe state,
// the claim released, a running step CANCELLED. A mechanism ANOTHER owner has claimed (an F1
// operation a team ticks itself) is never commanded: every step on it is REFUSED and a hold-to-run
// does nothing — "one ticker per operation", never a take-over. A claim RELEASED outside the
// service (no cancel(), nobody took it) ends the slot's behaviour at its next tick, a running step
// CANCELLED: a motor mechanism is put in its safe state, a pneumatic keeps its level. One behaviour
// per mechanism at a time, and the MOST RECENT command wins: a new step, a hold-to-run or a stop
// replaces a running step, which ends CANCELLED, said. `commandSeq()` is bumped by every command —
// how the driver's inputs (MECH-A4) see that something else has spoken since they last did.
//
// ── Piece sensors are POSITIVE evidence (the brief's N8, amended) ────────────────────
// `Slot::seen()` answers YES only on a fresh reading that sees (a distance closer than its threshold
// with confidence above 0; an optical at or above its proximity; a switch at its pressed level), NO
// only on a fresh reading that does not (a distance at or past the threshold — the adapter's honest
// 393.66 in for PROS's 9999 "no object" included; an optical below; a switch released), and UNKNOWN
// otherwise: no sensor built this boot; a distance held NEAR with confidence 0; a reading that is
// not a number (the F4 seams promise finite ones — a NaN must not compare its way to "no piece");
// and ANY sample taken while the sensor's READ-FAULT count moved since the end of the service's
// previous tick (the provider's hal::IReadFaults — a PROS adapter answers a failed read with its
// last good value, and a distance sensor that failed while it read "far" is byte for byte the 9999
// "no object" reading). UNKNOWN satisfies NEITHER `seesPiece` nor `stopsSeeingPiece`. At a step's
// bound: a sensor DECLARED but MISSING this boot → UNCONFIRMED, COUNTED done (the team lead's Q2 (c)
// and N8 (a): a loose cable costs the step's time, not the auton); a sensor that gave no evidence on
// even ONE sample of the step → UNCONFIRMED, NOT counted ("its piece sensor stopped answering": it
// may have missed the very piece the step waited for); otherwise TIMED_OUT. The
// count is compared against the END of the previous tick — never a before/after pair inside one
// read — so two reads of one sensor in one tick agree, and a device that answers again is fresh at
// the next tick (the brief's landmine 22). What it cannot see: an UNPLUGGED 3-wire switch reads a
// level, not PROS_ERR (HA-184).
//
// ── What this run builds, and what it does not ───────────────────────────────────────
// MECH-A2 builds the slot, the two seams, the claimant, the three mode changes, the lookups and the
// POWER and PNEUMATIC behaviours. The POSITION (Arm, Lift) and SPEED (Flywheel) behaviours are
// MECH-A3's: until it lands, bind() REFUSES a list that has one (a precondition naming it — never a
// slot that silently does nothing). The home switch's half of the read-fault rule arrives with
// MECH-A3's homing, its only reader.
//
// ── Re-entrancy (requirements :142-152) ──────────────────────────────────────────────
// A tick never calls a blocking verb, a wait or the scheduler's cancel(), and is never re-entered
// (a precondition). A predicate may READ a slot (`seen()`, `running()`), never tick it. A tick that
// never finished — its task deleted mid-tick by a PROS mode change (HA-161), or a throw out of a
// device — leaves the re-entrancy flag set: the next mode change (enterDriverControl /
// enterAutonomous) clears it.
//
// ── The honest limit ─────────────────────────────────────────────────────────────────
// The service runs ONLY when something ticks it: inside a shulib motion, wait or pause, and every
// driver-loop tick. A student's own busy loop (`pros::delay` in an auton, a then() action that
// blocks) STARVES it, and a starved service leaves each mechanism at its LAST command — not held,
// not limited, not safe (the brief's D11, measured: 11.74 V held for 1.5 s). The run guard says the
// same of itself: nothing preempts pure user code.
//
// ── What a step says ─────────────────────────────────────────────────────────────────
// Every step's verdict is ONE line, tag MECH — Info when SUCCEEDED, Warn otherwise:
//     'intake': SUCCEEDED -- sees a piece in 0.97s
//     'claw': UNCONFIRMED -- time in 0.50s (no piece sensor to confirm it; counted done)
// (the format, field by field: docs/guide/11-reading-the-diagnostics.md). STALLED also raises
// MECHANISM_STALLED, once per stall.
//
// PROS-free, allocation-free: every slot lives in the service's own fixed storage, so a program keeps
// the service in static storage (a deleted task cannot take it with it).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/manipulation/stall_detector.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/until.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"


namespace shulib::mechanism {

/// A step's verdict vocabulary (manipulation/mechanism_outcome.hpp), named here for the service's
/// readers: SUCCEEDED, UNCONFIRMED, TIMED_OUT, CANCELLED, STALLED, ABSENT, NOT_HOMED, REFUSED.
using manipulation::MechanismOutcome;

/// What the most recent step on a mechanism did — what a step reads and logs.
struct StepReport {
    /// The verdict; `Running` while the step is still working.
    MechanismOutcome outcome = MechanismOutcome::Running;
    /// Which condition ended it (`None`: the bound, a refusal or a cancel).
    DoneBy doneBy = DoneBy::None;
    /// UNCONFIRMED only because no sensor exists to confirm it — none declared (a sensorless grab,
    /// the team lead's Q2 (c)), or the declared one MISSING this boot (N8 (a)): the step is counted
    /// DONE and the chain goes on.
    bool countedDone = false;
    /// Seconds from the command to the verdict.
    double elapsed = 0.0;
};

class Slot;
class MechanismService;

namespace detail {
/// What every slot reads from the service that owns it — the clock, the latch, the sink and the
/// robot's slots — held BY the service and pointed at by each slot, so a slot's behaviour is
/// written inside its own class (the house style: no member defined outside its class) without
/// needing the service's complete type.
struct SlotHost {
    hal::IClock* clock = nullptr;
    diag::FaultLatch* faults = nullptr;
    hal::ITelemetrySink* telemetry = nullptr;
    std::array<Slot*, kMaxMechanisms> slots{};
    std::size_t count = 0;
};
}  // namespace detail

/// ONE mechanism's slot (header): its behaviour, its claim, its last verdict, its sensors' evidence.
/// The per-kind slots below derive from it. Lives in the service's storage; non-copyable and
/// non-movable, because the mechanism's claim registers it BY ADDRESS.
class Slot : public hal::ICancellable {
public:
    /// Non-copyable: a copy would leave the mechanism's claim registered to the original.
    Slot(const Slot&) = delete;
    /// Non-copy-assignable (the copy constructor's reason).
    Slot& operator=(const Slot&) = delete;
    /// Non-movable (the copy constructor's reason).
    Slot(Slot&&) = delete;
    /// Non-move-assignable (the copy constructor's reason).
    Slot& operator=(Slot&&) = delete;
    /// Releases nothing itself: the service that owns the slot ends one still holding its claim
    /// before the slot is destroyed (~MechanismService), while the slot is whole.
    ~Slot() override = default;

    /// Which behaviour runs it.
    [[nodiscard]] Kind kind() const noexcept { return spec_->kind; }
    /// The mechanism as data (the robot's list's entry, borrowed for the service's lifetime).
    [[nodiscard]] const MechanismSpec& spec() const noexcept { return *spec_; }
    /// Its name ("claw pivot 6").
    [[nodiscard]] const char* name() const noexcept { return spec_->name; }
    /// Its index in the robot's list.
    [[nodiscard]] std::uint8_t index() const noexcept { return index_; }
    /// True when its actuator was built at boot (false: ABSENT — every step answers ABSENT).
    [[nodiscard]] bool built() const noexcept { return mech_ != nullptr; }
    /// Why it was not built — the boot's sentence naming the port; "" when it was.
    [[nodiscard]] const char* absence() const noexcept { return devices_.absence; }
    /// The most recent verdict: a step's, or the STALLED that ended a hold-to-run (drive() — not a
    /// step, so stepsBegun() does not count it). Before either, a default report (`Running`, with
    /// running() false).
    [[nodiscard]] const StepReport& lastStep() const noexcept { return report_; }
    /// True while a STEP is still working (a behaviour that goes on after its step is not a step).
    [[nodiscard]] bool running() const noexcept {
        return stepsBegun_ > 0 && report_.outcome == MechanismOutcome::Running;
    }
    /// How many steps have begun on this slot (refused and absent ones included; a hold-to-run is not
    /// a step). 0 with lastStep() still `Running`: no verdict yet — the default report.
    [[nodiscard]] std::uint32_t stepsBegun() const noexcept { return stepsBegun_; }
    /// Bumped by EVERY command (a step, a hold-to-run, a stop) and by a stall that ends a hold-to-run:
    /// "the most recent input wins" (Q8) — an input that sees a number it did not write has been
    /// superseded.
    [[nodiscard]] std::uint32_t commandSeq() const noexcept { return seq_; }
    /// True while THIS slot holds its mechanism's claim (it is the registered claimant).
    [[nodiscard]] bool holdsClaim() const noexcept {
        return mech_ != nullptr && mech_->claimed() && mech_->claimant() == this;
    }

    /// The piece sensor's evidence (header): YES and NO only on a fresh reading, UNKNOWN otherwise.
    enum class Seen : std::uint8_t {
        Yes,      ///< a fresh reading that sees a piece
        No,       ///< a fresh reading that does not
        Unknown,  ///< no evidence: no sensor this boot, a near reading with no confidence, a read fault
    };
    /// What the piece sensor says NOW — one sample, read from the device (header: positive evidence,
    /// the read-fault rule). UNKNOWN for a mechanism with no piece sensor built, and for a reading
    /// that is not a number (the F4 sensor seams promise finite readings; an implementer that broke
    /// that promise must not turn "not a number" into "no piece" — a comparison with NaN is false).
    [[nodiscard]] Seen seen() const {
        Seen v = Seen::Unknown;
        if (devices_.pieceDistance != nullptr) {
            const double c = devices_.pieceDistance->confidence();
            const double d = devices_.pieceDistance->distance().value();
            if (!std::isfinite(d) || !std::isfinite(c)) {
                v = Seen::Unknown;  // not a reading
            } else if (d >= spec_->pieceSensor.seesWithinIn) {
                v = Seen::No;  // at or past the threshold — 9999's honest 393.66 in included
            } else if (c > 0.0) {
                v = Seen::Yes;  // near, with confidence
            }  // near with confidence 0: a held value, never evidence
        } else if (devices_.pieceOptical != nullptr) {
            const double p = devices_.pieceOptical->proximity();
            if (std::isfinite(p)) {
                v = p >= spec_->pieceSensor.seesProximity ? Seen::Yes : Seen::No;
            }
        } else if (devices_.pieceSwitch != nullptr) {
            // pressedLevel: the level that means PRESSED (A4 register HA-184 — unmeasured)
            v = devices_.pieceSwitch->state() == spec_->pieceSensor.pressedLevel ? Seen::Yes : Seen::No;
        }
        // THE READ-FAULT RULE (header): a sample taken while the count moved since the end of the last
        // tick was a HELD value — the adapter's last good one — and is never evidence. Read AFTER the
        // sample, so a read that failed just now has already counted.
        if (devices_.pieceFaults != nullptr && devices_.pieceFaults->faultedReads() != pieceFaultsSeen_) {
            return Seen::Unknown;
        }
        return v;
    }
    /// True only on positive evidence of a piece: `seen() == Seen::Yes`.
    [[nodiscard]] bool sees() const { return seen() == Seen::Yes; }
    /// True when the DECLARATION has a piece sensor, whatever this boot found.
    [[nodiscard]] bool declaresPieceSensor() const noexcept {
        return spec_->pieceSensor.kind != SensorKind::None;
    }
    /// True when the piece sensor was built this boot.
    [[nodiscard]] bool hasPieceSensor() const noexcept {
        return devices_.pieceDistance != nullptr || devices_.pieceOptical != nullptr ||
               devices_.pieceSwitch != nullptr;
    }
    /// Declared, but MISSING this boot: its conditions run their bound and end UNCONFIRMED, counted
    /// done (N8 (a)).
    [[nodiscard]] bool pieceSensorMissing() const noexcept { return declaresPieceSensor() && !hasPieceSensor(); }
    /// Where its piece sensor is wired, as a log line says it — "port 9", "3-wire E" — from the
    /// robot's list; "its port unknown to the service" when it was bound from bare specs (or declares
    /// none). Writes at most `cap` bytes, NUL-terminated.
    void pieceSensorPlace(char* out, std::size_t cap) const {
        if (wiring_ != nullptr && wiring_->pieceSensorPort != 0) {
            std::snprintf(out, cap, "port %u", static_cast<unsigned>(wiring_->pieceSensorPort));
        } else if (wiring_ != nullptr && wiring_->pieceSwitch != Line::None) {
            std::snprintf(out, cap, "3-wire %c", lineLetter(wiring_->pieceSwitch));
        } else {
            std::snprintf(out, cap, "its port unknown to the service");
        }
    }

    /// THE GUARD'S REACH (hal::ICancellable): the behaviour goes inert, the declared safe state is
    /// applied, the claim is released, a running step ends CANCELLED. Idempotent. A mechanism ANOTHER
    /// owner has claimed is left alone (it is not this slot's to make safe — the guard reaches that
    /// owner through its own registration).
    void cancel() override {
        if (mech_ == nullptr) {
            return;
        }
        if (running()) {
            finishStep(now(), MechanismOutcome::Cancelled, DoneBy::None);
        }
        stopBehaviour(true);
    }

protected:
    /// `wiring` may be null (a service bound from bare specs: boot lines then name no port);
    /// `actuator` is the mechanism the kind commands (nullptr, or `d.absence` non-empty: ABSENT).
    Slot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t index,
         const Devices& d, hal::IMechanism* actuator)
        : host_{&host}, spec_{&spec}, wiring_{wiring}, index_{index}, devices_{d},
          mech_{d.absence[0] == '\0' ? actuator : nullptr} {
        snapshotReadFaults();
    }

    /// Begin a step bounded by `boundSeconds` at clock time `t`: bump the command number, end a running
    /// step CANCELLED (the most recent wins), drop the current behaviour, take the claim. False — and
    /// the step already ABSENT or REFUSED, said — when it cannot run; nothing is commanded then.
    bool beginStep(double boundSeconds, double t) {
        ++seq_;
        cancelRunningStep(t, "replaced by a newer command");
        resetBehaviour();  // whatever it was doing (a hold-to-run) is replaced too
        active_ = false;
        ++stepsBegun_;
        stepStart_ = t;
        bound_ = boundSeconds;
        report_ = StepReport{};
        if (mech_ == nullptr) {
            char note[160];
            std::snprintf(note, sizeof note, "not built at boot: %.120s", devices_.absence);
            finishStep(t, MechanismOutcome::Absent, DoneBy::None, false, note);
            return false;
        }
        if (mech_->claimed() && !holdsClaim()) {
            finishStep(t, MechanismOutcome::Refused, DoneBy::None, false, "another owner holds its claim");
            return false;
        }
        if (!holdsClaim()) {
            (void)mech_->tryClaim(*this);
        }
        return true;
    }
    /// The verdict: the report, the MECHANISM_STALLED fault for STALLED, and the step's one log line.
    void finishStep(double t, MechanismOutcome o, DoneBy by, bool counted = false, const char* note = nullptr) {
        report_.outcome = o;
        report_.doneBy = by;
        report_.countedDone = counted;
        report_.elapsed = std::max(0.0, t - stepStart_);
        if (o == MechanismOutcome::Stalled) {
            host_->faults->raise(diag::FaultCode::MechanismStalled, "MECH", nameOf(*spec_));
        }
        // Every field BOUNDED (precisions, and the seconds clamped), so the line provably fits: an
        // optimised host build must not see a possible truncation (-Wformat-truncation, an error
        // under -Werror) in a library header. The bracket is at most 2 + 160 + 2 + 12 + 1 = 177
        // characters, so the whole line at most 1 + 23 + 3 + 11 + 4 + 22 + 4 + 8 + 1 + 177 = 254,
        // inside `line`'s 320.
        char why[200];
        why[0] = '\0';
        if (note != nullptr || counted) {
            std::snprintf(why, sizeof why, " (%.160s%s%s)", note != nullptr ? note : "",
                          note != nullptr && counted ? "; " : "", counted ? "counted done" : "");
        }
        const double shown = std::min(report_.elapsed, 99999.99);  // "%.2f": at most 8 characters
        char line[320];
        std::snprintf(line, sizeof line, "'%.23s': %.11s%s%.22s in %.2fs%.180s", spec_->name,
                      manipulation::mechanismOutcomeName(o), by != DoneBy::None ? " -- " : "",
                      by != DoneBy::None ? doneByWord(by) : "", shown, why);
        host_->telemetry->log(o == MechanismOutcome::Succeeded ? hal::LogLevel::Info : hal::LogLevel::Warn, "MECH",
                              line);
    }
    /// End a running step CANCELLED (no-op when none runs).
    void cancelRunningStep(double t, const char* note) {
        if (running()) {
            finishStep(t, MechanismOutcome::Cancelled, DoneBy::None, false, note);
        }
    }
    /// Stop the behaviour: inert, the kind's own state reset, the declared safe state applied when
    /// `applySafe` (only on a mechanism this slot holds or nobody holds), this slot's claim released.
    void stopBehaviour(bool applySafe) {
        active_ = false;
        resetBehaviour();
        if (mech_ == nullptr) {
            return;
        }
        const bool mine = holdsClaim();
        if (applySafe && (mine || !mech_->claimed())) {
            mech_->applySafeState();
        }
        if (mine) {
            mech_->releaseClaim();
        }
    }
    /// The clock, now (seconds).
    [[nodiscard]] double now() const { return host_->clock->now().value(); }
    /// The slot of ANY kind with this NAME on the same robot, or nullptr (the service's slotFor()).
    [[nodiscard]] const Slot* peer(const char* name) const noexcept {
        if (name == nullptr) {
            return nullptr;
        }
        for (std::size_t i = 0; i < host_->count; ++i) {
            if (nameOf(host_->slots[i]->spec()) == std::string_view{name}) {
                return host_->slots[i];
            }
        }
        return nullptr;
    }
    /// The END of every tick: the read-fault count a later sample is compared against.
    void snapshotReadFaults() {
        if (devices_.pieceFaults != nullptr) {
            pieceFaultsSeen_ = devices_.pieceFaults->faultedReads();
        }
    }

    /// One tick of the behaviour at clock time `t` (only while active and holding the claim).
    virtual void tickBehaviour(double t) = 0;
    /// Forget the kind's own behaviour state (a hold-to-run's volts, a step's condition).
    virtual void resetBehaviour() noexcept = 0;
    /// Driver control begins (after the behaviour was stopped): a pneumatic's first entry sends START.
    virtual void onDriverEntry(bool /*firstEntry*/) {}

    friend class MechanismService;

    detail::SlotHost* host_;     ///< the owning service's clock, latch, sink and slots
    const MechanismSpec* spec_;  ///< the robot's list's entry
    const MechanismWiring* wiring_;  ///< its wiring (nullptr when bound from bare specs)
    std::uint8_t index_;         ///< its index in the list
    Devices devices_;            ///< what the parts built for it
    hal::IMechanism* mech_;      ///< the actuator; nullptr = ABSENT
    bool active_ = false;        ///< a behaviour is being ticked
    std::uint32_t seq_ = 0;      ///< commandSeq()
    std::uint32_t stepsBegun_ = 0;  ///< stepsBegun()
    double stepStart_ = 0.0;     ///< when the step (or a hold-to-run) began
    double bound_ = 0.0;         ///< the step's bound, seconds after stepStart_
    StepReport report_{};        ///< lastStep()
    int pieceFaultsSeen_ = 0;    ///< the piece sensor's read-fault count at the end of the last tick
};

/// POWER: run at a voltage until a done-condition, within a bound; the driver's hold-to-run.
class PowerSlot final : public Slot {
public:
    /// Built by the service's bind(); `d.motors` is the actuator (nullptr when ABSENT).
    PowerSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i,
              const Devices& d)
        : Slot{host, spec, wiring, i, d, d.motors} {
        if (spec.power.stall.stated()) {
            stall_.emplace(manipulation::StallConfig{units::Current{spec.power.stall.amps},
                                                     units::AngularVelocity{spec.power.stall.speedRadS},
                                                     units::Time{spec.power.stall.time}});
        }
    }
    /// Start running at `volts` (clamped to the declaration's maxVolts) until `until`, bounded by
    /// `bound`. Returns at once; the service ticks it, and EVERY exit applies the declared safe state
    /// and releases the claim. PRECONDITIONS (nothing is commanded when one fails — a condition that
    /// can never be met is the team's mistake, said where it is made): `volts` finite; `bound` finite
    /// and > 0; a time condition finite, > 0 and at most `bound` (shorter than it beside another
    /// condition: `orAfter`); `seesPiece` / `stopsSeeingPiece` only on a mechanism whose DECLARATION
    /// has a piece sensor, named on this robot; `stalls` only with a stated stall signature; one
    /// mechanism's sensor (`!until.conflicting()`, and the step's own sensor beside a named one only
    /// when the name is this mechanism's — `until.readsOwnSensorToo()`); a named mechanism has a name.
    void run(units::Voltage volts, Until until, units::Time bound) {
        // Every precondition BEFORE anything changes: a refused step commands nothing, takes no claim
        // and bumps no command number (the brief's D7 — a condition that can never be met).
        SHULIB_PRECONDITION(std::isfinite(volts.value()), "r.run(...): the volts must be a finite number");
        SHULIB_PRECONDITION(std::isfinite(bound.value()) && bound.value() > 0.0,
                            "r.run(...): the time limit must be finite and above 0");
        SHULIB_PRECONDITION(!until.conflicting(),
                            "r.run(...): its conditions read two different mechanisms' piece sensors -- a step "
                            "waits on ONE mechanism's sensor");
        const bool sensorCondition = until.has(Until::kSees) || until.has(Until::kStops);
        const Slot* sensor = this;
        if (sensorCondition && until.sensor() != Until::kSelf) {
            SHULIB_PRECONDITION(until.sensor()[0] != '\0',
                                "r.run(...): seesPiece(m) / stopsSeeingPiece(m) names a declaration with no .name "
                                "-- no robot's list has it");
            sensor = peer(until.sensor());
            SHULIB_PRECONDITION(sensor != nullptr,
                                "r.run(...): seesPiece(m) / stopsSeeingPiece(m) names a mechanism this robot's "
                                "list does not have");
            // The step's OWN sensor beside a named one (until.hpp): one sensor only on that mechanism.
            SHULIB_PRECONDITION(!until.readsOwnSensorToo() || sensor == this,
                                "r.run(...): its conditions read two different mechanisms' piece sensors -- a step "
                                "waits on ONE mechanism's sensor");
        }
        SHULIB_PRECONDITION(!sensorCondition || sensor->declaresPieceSensor(),
                            "r.run(...): seesPiece / stopsSeeingPiece on a mechanism that declares NO piece "
                            "sensor -- add .pieceSensor to its declaration, or end the step by time");
        SHULIB_PRECONDITION(!until.has(Until::kStalled) || spec_->power.stall.stated(),
                            "r.run(...): `stalls` on a mechanism with no stall signature -- add .stall to its "
                            "declaration");
        if (until.has(Until::kTime)) {
            SHULIB_PRECONDITION(std::isfinite(until.seconds()) && until.seconds() > 0.0,
                                "r.run(...): a time condition must be finite and above 0");
            const bool alone = !sensorCondition && !until.has(Until::kStalled);
            SHULIB_PRECONDITION(alone ? until.seconds() <= bound.value() : until.seconds() < bound.value(),
                                "r.run(...): its time (orAfter's) must be shorter than the step's time limit -- "
                                "the time limit would always end it first");
        }
        const double t = now();
        if (!beginStep(bound.value(), t)) {
            return;
        }
        const double m = spec_->power.maxVolts;
        volts_ = std::clamp(volts.value(), -m, m);
        until_ = until;
        sensorSlot_ = sensor;
        if (stall_) {
            stall_->reset();
        }
        active_ = true;
    }
    /// Stop: a running step CANCELLED, the declared safe state, the claim released. Does nothing to a
    /// mechanism another owner holds, or one that is ABSENT.
    void stop() {
        if (mech_ == nullptr || (mech_->claimed() && !holdsClaim())) {
            return;
        }
        ++seq_;
        cancelRunningStep(now(), "stopped");
        stopBehaviour(true);
    }
    /// The DRIVER's hold-to-run: `volts` (clamped) every tick until another command — open-ended; the
    /// button's release is the caller's stop(). Call it once or every tick (each call restates the
    /// volts). A stated stall signature watches it (N10): a trip is STALLED, MECHANISM_STALLED, the
    /// safe state, released, and the command number bumped (a held button is superseded until it is
    /// released). With NO signature (allowed on a power mechanism) nothing but the release cuts it —
    /// held against its stop, it sits at the motor's current limit (A4 register HA-154). Does nothing
    /// on a mechanism another owner holds (the T3 rule), or one that is ABSENT. PRECONDITION: `volts`
    /// finite.
    void drive(units::Voltage volts) {
        SHULIB_PRECONDITION(std::isfinite(volts.value()), "PowerSlot::drive: the volts must be a finite number");
        if (mech_ == nullptr || (mech_->claimed() && !holdsClaim())) {
            return;
        }
        ++seq_;
        const double t = now();
        cancelRunningStep(t, "replaced by a newer command");
        if (!holdsClaim()) {
            (void)mech_->tryClaim(*this);
        }
        const double m = spec_->power.maxVolts;
        volts_ = std::clamp(volts.value(), -m, m);
        if (!driving_) {  // a NEW hold-to-run: its stall window and its time start now
            until_ = Until::bound();
            sensorSlot_ = nullptr;
            stepStart_ = t;
            if (stall_) {
                stall_->reset();
            }
        }
        driving_ = true;
        active_ = true;
    }
    /// The voltage its motors were last commanded — the DEVICE readback (0 when ABSENT).
    [[nodiscard]] double commandedVolts() const {
        return devices_.motors != nullptr ? devices_.motors->commandedVoltage().value() : 0.0;
    }

private:
    void tickBehaviour(double t) override {
        hal::MotorMechanism& motors = *devices_.motors;
        if (driving_) {  // the driver's hold-to-run: a STATED stall signature watches it too (N10)
            if (stall_ && stall_->update(units::Time{t}, motors.maxCurrent(), motors.meanVelocity())) {
                ++seq_;  // the held button is superseded until it is released
                stopBehaviour(true);
                finishStep(t, MechanismOutcome::Stalled, DoneBy::Stalled, false,
                           "a stated stall signature tripped while held to run");
                return;
            }
            motors.setVoltage(units::Voltage{volts_});
            return;
        }
        if (!running()) {  // nothing to do (defensive: a step always runs while a non-driving slot is active)
            stopBehaviour(true);
            return;
        }
        const double elapsed = t - stepStart_;
        // 1. THE EVIDENCE first: a piece seen ends the step as "sees a piece" even on the tick its time
        //    is up — the more informative verdict, and both are SUCCEEDED.
        if (until_.has(Until::kSees) || until_.has(Until::kStops)) {
            const Seen s = sensorSlot_->seen();
            ++reads_;
            if (s == Seen::Unknown) {
                ++blind_;
            }
            if (until_.has(Until::kSees) && s == Seen::Yes) {
                stopBehaviour(true);
                finishStep(t, MechanismOutcome::Succeeded, DoneBy::Sees);
                return;
            }
            if (until_.has(Until::kStops) && s == Seen::No) {
                stopBehaviour(true);
                finishStep(t, MechanismOutcome::Succeeded, DoneBy::StopsSeeing);
                return;
            }
        }
        // 2. ITS TIME — checked before the bound with the same arithmetic, so a time equal to the bound
        //    ends it SUCCEEDED by time.
        if (until_.has(Until::kTime) && elapsed >= until_.seconds()) {
            stopBehaviour(true);
            finishStep(t, MechanismOutcome::Succeeded, DoneBy::Time);
            return;
        }
        // 3. A STALL: what `stalls` asked for (SUCCEEDED), or the jam the stated signature watches for.
        if (stall_ && stall_->update(units::Time{t}, motors.maxCurrent(), motors.meanVelocity())) {
            const bool asked = until_.has(Until::kStalled);
            stopBehaviour(true);
            if (asked) {
                finishStep(t, MechanismOutcome::Succeeded, DoneBy::Stalled);
            } else {
                finishStep(t, MechanismOutcome::Stalled, DoneBy::Stalled, false, "a stated stall signature tripped");
            }
            return;
        }
        // 4. THE BOUND.
        if (elapsed >= bound_) {
            verdictAtBound(t);
            return;
        }
        motors.setVoltage(units::Voltage{volts_});
    }
    void resetBehaviour() noexcept override {
        driving_ = false;
        volts_ = 0.0;
        until_ = Until::bound();
        sensorSlot_ = nullptr;
        reads_ = 0;
        blind_ = 0;
    }
    void verdictAtBound(double t) {
        const bool sensorCondition = until_.has(Until::kSees) || until_.has(Until::kStops);
        const Slot* sensor = sensorSlot_;
        const int reads = reads_;
        const int blind = blind_;
        stopBehaviour(true);  // resets the counts: read above
        char note[200];
        if (sensorCondition && sensor->pieceSensorMissing()) {
            // N8 (a): the DECLARED sensor is MISSING this boot — the step ran its time; counted done.
            char where[48];
            sensor->pieceSensorPlace(where, sizeof where);
            if (sensor == this) {
                std::snprintf(note, sizeof note, "its piece sensor is missing this boot, %s", where);
            } else {
                std::snprintf(note, sizeof note, "the piece sensor of '%.23s' is missing this boot, %s",
                              sensor->name(), where);
            }
            finishStep(t, MechanismOutcome::Unconfirmed, DoneBy::Time, true, note);
        } else if (sensorCondition && blind > 0) {
            // A sensor that gave no evidence on some sample of the step might have missed the very piece
            // the step was waiting for: the step cannot say "it never came" — UNCONFIRMED, NOT counted.
            if (sensor == this) {
                std::snprintf(note, sizeof note, "its piece sensor stopped answering: no evidence on %d of %d reads",
                              blind, reads);
            } else {
                std::snprintf(note, sizeof note,
                              "the piece sensor of '%.23s' stopped answering: no evidence on %d of %d reads",
                              sensor->name(), blind, reads);
            }
            finishStep(t, MechanismOutcome::Unconfirmed, DoneBy::None, false, note);
        } else {
            finishStep(t, MechanismOutcome::TimedOut, DoneBy::None);
        }
    }
    double volts_ = 0.0;
    bool driving_ = false;              ///< a hold-to-run is in progress (not a step)
    Until until_ = Until::bound();
    const Slot* sensorSlot_ = nullptr;  ///< whose piece sensor the step's condition reads
    int reads_ = 0;                     ///< the step's piece-sensor samples ...
    int blind_ = 0;                     ///< ... and how many of them were UNKNOWN
    std::optional<manipulation::StallDetector> stall_;
};

/// PNEUMATIC: two or three named positions, each the team's level per line (Q13 (a)).
class PneumaticSlot final : public Slot {
public:
    /// `commandedPosition()` when the lines match none of the positions.
    static constexpr std::uint8_t kNoPosition = 0xFF;

    /// Built by the service's bind(); `d.pneumatic` is the actuator (nullptr when ABSENT).
    PneumaticSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i,
                  const Devices& d)
        : Slot{host, spec, wiring, i, d, d.pneumatic} {}
    /// Go to position `p`: EACH line gets the position's level NOW; the step's verdict comes after the
    /// move time — SUCCEEDED when the position confirms nothing; a `grabs` / `releases` position with
    /// a piece sensor SUCCEEDS when the sensor agrees within the confirm window, else UNCONFIRMED (not
    /// counted); with NO sensor (none declared, or the declared one missing this boot) it is
    /// UNCONFIRMED and COUNTED done (Q2 (c)). The claim is held until the verdict; the level persists
    /// after it. PRECONDITION: `p` is one of its positions.
    void set(std::uint8_t p) {
        SHULIB_PRECONDITION(p < spec_->pneumatic.positionCount, "PneumaticSlot::set: no such position");
        const double t = now();
        if (!beginStep(spec_->pneumatic.actuationTime, t)) {
            return;
        }
        position_ = p;
        writeLevels(p);  // the lines NOW; the verdict after the move time
        active_ = true;
    }
    /// The position its lines are at NOW — read back from the DEVICES (so the end-of-run guard's safe
    /// state shows here too), or kNoPosition when they match none of its positions (or it is ABSENT).
    [[nodiscard]] std::uint8_t commandedPosition() const {
        if (devices_.pneumatic == nullptr) {
            return kNoPosition;
        }
        const PneumaticSpec& q = spec_->pneumatic;
        for (std::uint8_t p = 0; p < q.positionCount; ++p) {
            bool all = true;
            for (std::size_t l = 0; l < spec_->lineCount; ++l) {
                all = all && devices_.pneumatic->commandedLevel(l) == q.positions[p].levels[l];
            }
            if (all) {
                return p;
            }
        }
        return kNoPosition;
    }
    /// The longest a set() takes to reach its verdict: the move time plus the confirm window, seconds.
    [[nodiscard]] double lastStepBound() const noexcept {
        return spec_->pneumatic.actuationTime + spec_->pneumatic.confirmWindow;
    }

private:
    void tickBehaviour(double t) override {
        if (!running()) {
            stopBehaviour(false);  // the level persists; the claim goes
            return;
        }
        const PneumaticSpec& q = spec_->pneumatic;
        const double elapsed = t - stepStart_;
        if (elapsed < q.actuationTime) {
            return;  // mid-move: the world cannot be asked yet
        }
        const Confirm c = q.positions[position_].confirm;
        if (c == Confirm::None) {
            stopBehaviour(false);
            finishStep(t, MechanismOutcome::Succeeded, DoneBy::Time);
            return;
        }
        if (!hasPieceSensor()) {  // Q2 (c) / N8 (a): counted done after the move time, UNCONFIRMED
            char note[96];
            if (declaresPieceSensor()) {
                char where[48];
                pieceSensorPlace(where, sizeof where);
                std::snprintf(note, sizeof note, "its piece sensor is missing this boot, %s", where);
            } else {
                std::snprintf(note, sizeof note, "no piece sensor to confirm it");
            }
            stopBehaviour(false);
            finishStep(t, MechanismOutcome::Unconfirmed, DoneBy::Time, true, note);
            return;
        }
        const Seen s = seen();
        ++reads_;
        if (s == Seen::Unknown) {
            ++blind_;
        }
        if (c == Confirm::Sees && s == Seen::Yes) {
            stopBehaviour(false);
            finishStep(t, MechanismOutcome::Succeeded, DoneBy::Sees);
            return;
        }
        if (c == Confirm::StopsSeeing && s == Seen::No) {
            stopBehaviour(false);
            finishStep(t, MechanismOutcome::Succeeded, DoneBy::StopsSeeing);
            return;
        }
        if (elapsed >= q.actuationTime + q.confirmWindow) {
            const int reads = reads_;
            const int blind = blind_;
            stopBehaviour(false);
            char note[120];
            if (blind > 0) {
                std::snprintf(note, sizeof note, "its piece sensor stopped answering: no evidence on %d of %d reads",
                              blind, reads);
            } else {
                std::snprintf(note, sizeof note, "%s",
                              c == Confirm::Sees ? "its piece sensor did not see a piece"
                                                 : "its piece sensor still sees a piece");
            }
            finishStep(t, MechanismOutcome::Unconfirmed, DoneBy::None, false, note);
        }
    }
    void resetBehaviour() noexcept override {
        reads_ = 0;
        blind_ = 0;
    }
    void onDriverEntry(bool firstEntry) override {
        if (firstEntry && mech_ != nullptr && !mech_->claimed()) {
            writeLevels(spec_->pneumatic.start);  // T4: the START position, on the first entry only
        }
    }
    void writeLevels(std::uint8_t p) {
        bool levels[kMaxLineRoles]{};
        for (std::size_t l = 0; l < spec_->lineCount; ++l) {
            levels[l] = spec_->pneumatic.positions[p].levels[l];
        }
        devices_.pneumatic->setLevels(std::span<const bool>{levels, spec_->lineCount});
    }
    std::uint8_t position_ = 0;  ///< the position the running step went to
    int reads_ = 0;              ///< the confirm window's piece-sensor samples ...
    int blind_ = 0;              ///< ... and how many of them were UNKNOWN
};

/// THE SERVICE (file header): one slot per mechanism of the robot's list, ticked exactly once per
/// control tick through the pacer seam or the driver loop's stated place.
class MechanismService final : public motion::ITickPacer {
public:
    /// `inner` advances the world (the program's watch, or the tick pacer) and must outlive the
    /// service. Unbound, pace() ticks nothing and forwards — a program builds the pacer chain before
    /// the Robot and binds the service after it (landmine 1).
    explicit MechanismService(motion::ITickPacer& inner) noexcept : inner_{&inner} {}
    /// Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration
    /// point at it.
    MechanismService(const MechanismService&) = delete;
    /// Non-copy-assignable (the copy constructor's reason).
    MechanismService& operator=(const MechanismService&) = delete;
    /// Non-movable (the copy constructor's reason).
    MechanismService(MechanismService&&) = delete;
    /// Non-move-assignable (the copy constructor's reason).
    MechanismService& operator=(MechanismService&&) = delete;
    /// A slot still holding its mechanism's claim — a step or a hold-to-run mid-flight — is ended
    /// first: CANCELLED ("its service was destroyed"), the declared safe state, the claim released.
    /// Otherwise the claim would stay registered to a destroyed slot (any later cancel-all would call
    /// into it) and the motor would stay at its last volts — the F1 operations' destructor rule
    /// (manipulation/mechanism_op.hpp). A slot holding no claim is untouched (another owner's claim
    /// is that owner's; a pneumatic's level persists). This is where bind()'s rule matters: the
    /// devices, the clock, the latch and the sink must outlive the service. Nothing escapes: a device
    /// that throws is left where it is, its claim is STILL released (no claimant may outlive the
    /// slot it points at), and the other slots are still ended.
    ~MechanismService() override {
        for (std::size_t i = 0; i < host_.count; ++i) {
            Slot& s = *host_.slots[i];
            if (!s.holdsClaim()) {
                continue;  // nothing of this service's to end
            }
            try {
                s.cancelRunningStep(host_.clock->now().value(), "its service was destroyed");
                s.stopBehaviour(true);
            } catch (...) {  // a destructor must not throw
            }
            if (s.holdsClaim()) {
                s.mech_->releaseClaim();  // the throw skipped stopBehaviour's release: never leave it registered
            }
        }
    }

    /// BOOT: one slot per spec, over its devices — `devices[i]` is `specs[i]`'s, as
    /// robot::MechanismParts built them (the SAME devices the guard's span lists, landmine 20). A
    /// mechanism that was not built is a slot that answers ABSENT. Nothing is commanded. The specs,
    /// the devices' objects, the clock, the latch and the sink must outlive the service.
    /// PRECONDITIONS, each checked before anything is stored: bound ONCE; one Devices per spec, at
    /// most 8; each spec valid (specRefusal) and its name unique; each a POWER or PNEUMATIC mechanism
    /// (position and speed behaviours arrive with MECH-A3); a built mechanism's Devices carries the
    /// actuator of its kind, with the declared number of motors or lines, and an ABSENT one none; a
    /// piece sensor only as declared, of the declared kind; no angle sensor or home switch.
    void bind(std::span<const MechanismSpec> specs, std::span<const Devices> devices, hal::IClock& clock,
              diag::FaultLatch& faults, hal::ITelemetrySink& telemetry) {
        bindImpl(specs, nullptr, devices, clock, faults, telemetry);
    }
    /// BOOT from the ROBOT'S LIST (wiring.hpp's mechanisms(...)): the same, and each slot also knows
    /// its wiring (a missing sensor's line names its port). `robot` must outlive the service.
    void bind(const MechanismTable& robot, std::span<const Devices> devices, hal::IClock& clock,
              diag::FaultLatch& faults, hal::ITelemetrySink& telemetry) {
        bindImpl(robot.specSpan(), robot.wiring, devices, clock, faults, telemetry);
    }
    /// True once bind() has run.
    [[nodiscard]] bool bound() const noexcept { return host_.clock != nullptr; }

    // ── the two ticking seams (header) ────────────────────────────────────────────────
    /// THE PACER SEAM (autons): tick every mechanism once, then advance the world. A pace() right
    /// after tickFromDriverLoop() does NOT tick again (exactly once per tick). Unbound: forwards only.
    void pace() override {
        if (!driverTicked_) {
            tickImpl();
        }
        driverTicked_ = false;
        inner_->pace();
    }
    /// THE DRIVER LOOP'S STATED PLACE (after the driver's inputs, before perTick): tick every mechanism
    /// once; the next pace() then only advances the world.
    void tickFromDriverLoop() {
        tickImpl();
        driverTicked_ = true;
    }

    // ── the mode changes ──────────────────────────────────────────────────────────────
    /// DRIVER CONTROL BEGINS: every slot drops its autonomous-era behaviour. A claim this service
    /// holds is released with the behaviour; a claim ANOTHER owner left (an operation on a task PROS
    /// deleted) is released WITHOUT calling its cancel() — it may have lived on the deleted task's
    /// stack — counted, and said (the team lead's ruling of 2026-09-26, T3 Part 2). A running step
    /// ends CANCELLED. Motor mechanisms are put in their declared safe state; a pneumatic one is sent
    /// its START position on the FIRST entry only (T4's ruling: the claw starts closed — whatever
    /// autonomous left it at — and afterwards keeps what it holds). Clears a re-entrancy flag a
    /// deleted task left.
    void enterDriverControl() {
        inTick_ = false;  // a task deleted mid-tick must not leave the flag set
        driverTicked_ = false;
        if (!bound()) {
            return;
        }
        const double t = host_.clock->now().value();
        for (std::size_t i = 0; i < host_.count; ++i) {
            Slot& s = *host_.slots[i];
            // THE ORDER IS LOAD-BEARING: the leftover claim released FIRST, so stopBehaviour() finds
            // the mechanism unclaimed and puts a MOTOR mechanism in its safe state — a dead operation's
            // last 12 V is stopped too, not only forgotten (T3's rule: "then made safe at entry like
            // any other").
            releaseLeftover(s, "driver control");
            s.cancelRunningStep(t, "driver control began");
            s.stopBehaviour(s.kind() != Kind::Pneumatic);
            s.onDriverEntry(!enteredDriverOnce_);
        }
        enteredDriverOnce_ = true;
    }
    /// AUTONOMOUS BEGINS (the brief's D1): PROS deletes the driver-control task on a mode change
    /// (HA-161), so driver control's own exit may never run — a driver's intake left at 12 V then ran
    /// through the whole auton. The auton runner (MECH-A4's runAuton overload) calls this before the
    /// guard's run: the re-entrancy flag cleared; a claim ANOTHER owner left released WITHOUT its
    /// cancel(), counted, said; every motor mechanism's behaviour stopped (its safe state); a
    /// pneumatic KEEPS its level; a running step CANCELLED. Returns how many leftover claims this call
    /// released.
    int enterAutonomous() {
        inTick_ = false;
        driverTicked_ = false;
        if (!bound()) {
            return 0;
        }
        const int before = leftoverClaimsReleased_;
        const double t = host_.clock->now().value();
        for (std::size_t i = 0; i < host_.count; ++i) {
            Slot& s = *host_.slots[i];
            releaseLeftover(s, "autonomous");  // FIRST: enterDriverControl()'s order, its reason
            s.cancelRunningStep(t, "autonomous began");
            s.stopBehaviour(s.kind() != Kind::Pneumatic);  // a pneumatic keeps its level (HA-157)
        }
        return leftoverClaimsReleased_ - before;
    }
    /// DRIVER CONTROL ENDS (a disable): every MOTOR mechanism's behaviour stops — inert, its declared
    /// safe state, this service's claim released; a PNEUMATIC one keeps its level (a claw must not
    /// drop what it holds — HA-157); a running step ends CANCELLED. A claim another owner took is left
    /// to that owner.
    void leaveDriverControl() {
        driverTicked_ = false;
        if (!bound()) {
            return;
        }
        const double t = host_.clock->now().value();
        for (std::size_t i = 0; i < host_.count; ++i) {
            Slot& s = *host_.slots[i];
            s.cancelRunningStep(t, "driver control ended");
            s.stopBehaviour(s.kind() != Kind::Pneumatic);  // a pneumatic keeps its level (HA-157)
        }
    }

    // ── lookup ───────────────────────────────────────────────────────────────────────
    /// How many mechanisms are bound.
    [[nodiscard]] std::size_t size() const noexcept { return host_.count; }
    /// The slot at `i`. PRECONDITION: `i` below size().
    [[nodiscard]] Slot& slot(std::size_t i) {
        SHULIB_PRECONDITION(i < host_.count, "MechanismService::slot: no mechanism at this index");
        return *host_.slots[i];
    }
    /// The slot at `i`, or nullptr past the robot's mechanisms.
    [[nodiscard]] const Slot* slotPtr(std::size_t i) const noexcept {
        return i < host_.count ? host_.slots[i] : nullptr;
    }
    /// The POWER slot at `i`. PRECONDITION: `i` is a power mechanism.
    [[nodiscard]] PowerSlot& power(std::size_t i) {
        SHULIB_PRECONDITION(i < host_.count && powers_[i].has_value(),
                            "MechanismService::power: the mechanism at this index is not a power mechanism");
        return *powers_[i];
    }
    /// The PNEUMATIC slot at `i`. PRECONDITION: `i` is a pneumatic mechanism.
    [[nodiscard]] PneumaticSlot& pneumatic(std::size_t i) {
        SHULIB_PRECONDITION(i < host_.count && pneumatics_[i].has_value(),
                            "MechanismService::pneumatic: the mechanism at this index is not a pneumatic mechanism");
        return *pneumatics_[i];
    }
    /// A DECLARATION's slot (Q18 (a): the declaration is the handle), found by its KIND (the overload
    /// — it cannot mismatch) and its NAME (unique on a robot: the build refuses two). nullptr when this
    /// robot's list does not have it — a step on it then ends ABSENT, said. By name, never by address:
    /// a declaration written without `inline` has one copy per translation unit (the brief's P12),
    /// and an address lookup would end a step in another file ABSENT, silently. (So a robot two auton
    /// compiled into another robot's build commands a same-named mechanism of the same kind there —
    /// autons are per robot, landmine 6.)
    [[nodiscard]] PowerSlot* find(const mechanism::Power& d) noexcept { return findIn(powers_, d.name.text); }
    /// A pneumatic declaration's slot (find(const Power&)'s rule).
    [[nodiscard]] PneumaticSlot* find(const mechanism::Pneumatic& d) noexcept {
        return findIn(pneumatics_, d.name.text);
    }
    /// A pneumatic GOAL's slot (`claw["closed"]`): also checks the POSITION — returned only when this
    /// robot's mechanism of that name has that position's name at that index.
    [[nodiscard]] PneumaticSlot* find(const PneumaticGoal& g) noexcept {
        PneumaticSlot* p = findIn(pneumatics_, g.mechanismName);
        // kMaxPneumaticPositions first: a bound the compiler can PROVE (an optimised host build warned
        // -Warray-bounds — an error under -Werror — on a constant goal past the array, guarded only by
        // the run-time positionCount)
        if (p == nullptr || g.position >= kMaxPneumaticPositions || g.position >= p->spec().pneumatic.positionCount ||
            g.positionName == nullptr ||
            std::string_view{p->spec().pneumatic.positions[g.position].name} != std::string_view{g.positionName}) {
            return nullptr;
        }
        return p;
    }
    /// The slot of ANY kind with this NAME (another mechanism's piece sensor, the driver's inputs), or
    /// nullptr.
    [[nodiscard]] const Slot* slotFor(const char* name) const noexcept {
        if (name == nullptr) {
            return nullptr;
        }
        for (std::size_t i = 0; i < host_.count; ++i) {
            if (nameOf(host_.slots[i]->spec()) == std::string_view{name}) {
                return host_.slots[i];
            }
        }
        return nullptr;
    }

    /// Every BUILT mechanism, in the list's order — the RunGuard's span (the guard reaches each one's
    /// slot through the claim it registers).
    [[nodiscard]] std::span<hal::IMechanism* const> mechanisms() const noexcept {
        return std::span<hal::IMechanism* const>{built_.data(), builtCount_};
    }
    /// How many ticks have run (an unbound pace() and a re-entered one are not ticks) — a program's
    /// boot proves its pacer chain with it (MECH-A5).
    [[nodiscard]] long ticks() const noexcept { return ticks_; }
    /// How many leftover claims of another owner the mode changes have released, in total.
    [[nodiscard]] int leftoverClaimsReleased() const noexcept { return leftoverClaimsReleased_; }
    /// The clock it was bound with. PRECONDITION: bound.
    [[nodiscard]] hal::IClock& clock() const {
        SHULIB_PRECONDITION(host_.clock != nullptr, "MechanismService::clock: not bound");
        return *host_.clock;
    }
    /// The fault latch it was bound with. PRECONDITION: bound.
    [[nodiscard]] diag::FaultLatch& faults() const {
        SHULIB_PRECONDITION(host_.faults != nullptr, "MechanismService::faults: not bound");
        return *host_.faults;
    }
    /// The telemetry sink it was bound with. PRECONDITION: bound.
    [[nodiscard]] hal::ITelemetrySink& telemetry() const {
        SHULIB_PRECONDITION(host_.telemetry != nullptr, "MechanismService::telemetry: not bound");
        return *host_.telemetry;
    }

private:
    void bindImpl(std::span<const MechanismSpec> specs, const MechanismWiring* wiring,
                  std::span<const Devices> devices, hal::IClock& clock, diag::FaultLatch& faults,
                  hal::ITelemetrySink& telemetry) {
        SHULIB_PRECONDITION(!bound(), "MechanismService::bind: called twice -- bind once, at boot");
        SHULIB_PRECONDITION(specs.size() == devices.size(), "MechanismService::bind: one Devices per spec");
        SHULIB_PRECONDITION(specs.size() <= kMaxMechanisms, "MechanismService::bind: more than 8 mechanisms");
        // EVERY check before anything is stored: a refused bind leaves the service unbound.
        for (std::size_t i = 0; i < specs.size(); ++i) {
            const MechanismSpec& s = specs[i];
            const Devices& d = devices[i];
            SHULIB_PRECONDITION(specRefusal(s) == Refusal::None, specProblem(s));
            SHULIB_PRECONDITION(s.kind == Kind::Power || s.kind == Kind::Pneumatic,
                                "MechanismService::bind: a position (Arm, Lift) or speed (Flywheel) mechanism -- "
                                "their behaviours arrive with MECH-A3; this build runs power and pneumatic "
                                "mechanisms only");
            for (std::size_t j = 0; j < i; ++j) {
                SHULIB_PRECONDITION(nameOf(specs[j]) != nameOf(s),
                                    "MechanismService::bind: two mechanisms share a name");
            }
            SHULIB_PRECONDITION(d.absence != nullptr,
                                "MechanismService::bind: a Devices' absence is null (\"\" = built)");
            const bool built = d.absence[0] == '\0';
            if (built && s.kind == Kind::Power) {
                SHULIB_PRECONDITION(d.motors != nullptr && d.pneumatic == nullptr,
                                    "MechanismService::bind: a built power mechanism's Devices needs its motors, and "
                                    "no pneumatic");
                SHULIB_PRECONDITION(d.motors->motors().size() == s.motorCount,
                                    "MechanismService::bind: the motor group's size is not the declaration's motor "
                                    "count");
            } else if (built) {
                SHULIB_PRECONDITION(d.pneumatic != nullptr && d.motors == nullptr,
                                    "MechanismService::bind: a built pneumatic mechanism's Devices needs its lines, "
                                    "and no motors");
                SHULIB_PRECONDITION(d.pneumatic->lines().size() == s.lineCount,
                                    "MechanismService::bind: the pneumatic's line count is not the wiring's");
            } else {
                SHULIB_PRECONDITION(d.motors == nullptr && d.pneumatic == nullptr,
                                    "MechanismService::bind: an ABSENT mechanism's Devices carries no actuator");
            }
            SHULIB_PRECONDITION(d.angleSensor == nullptr && d.homeSwitch == nullptr,
                                "MechanismService::bind: an angle sensor or a home switch on a power or pneumatic "
                                "mechanism (only a position or speed mechanism declares one)");
            const int pieces = (d.pieceDistance != nullptr ? 1 : 0) + (d.pieceOptical != nullptr ? 1 : 0) +
                               (d.pieceSwitch != nullptr ? 1 : 0);
            const SensorKind k = s.pieceSensor.kind;
            SHULIB_PRECONDITION(
                pieces == 0 || (pieces == 1 && ((k == SensorKind::Distance && d.pieceDistance != nullptr) ||
                                                (k == SensorKind::Optical && d.pieceOptical != nullptr) ||
                                                (k == SensorKind::Switch && d.pieceSwitch != nullptr))),
                "MechanismService::bind: a piece sensor the declaration does not have (or of another kind)");
            SHULIB_PRECONDITION(d.pieceFaults == nullptr || pieces == 1,
                                "MechanismService::bind: a read-fault count for a piece sensor that was not built");
            SHULIB_PRECONDITION(d.homeSwitchFaults == nullptr,
                                "MechanismService::bind: a home switch's read-fault count on a mechanism with no "
                                "home switch");
        }
        host_.clock = &clock;
        host_.faults = &faults;
        host_.telemetry = &telemetry;
        for (std::size_t i = 0; i < specs.size(); ++i) {
            const auto idx = static_cast<std::uint8_t>(i);
            const MechanismWiring* w = wiring != nullptr ? &wiring[i] : nullptr;
            if (specs[i].kind == Kind::Power) {
                host_.slots[i] = &powers_[i].emplace(host_, specs[i], w, idx, devices[i]);
            } else {
                host_.slots[i] = &pneumatics_[i].emplace(host_, specs[i], w, idx, devices[i]);
            }
            if (host_.slots[i]->mech_ != nullptr) {
                built_[builtCount_++] = host_.slots[i]->mech_;
            }
            ++host_.count;
        }
    }
    template <class T>
    T* findIn(std::array<std::optional<T>, kMaxMechanisms>& a, const char* name) noexcept {
        if (name == nullptr) {
            return nullptr;
        }
        for (std::size_t i = 0; i < host_.count; ++i) {
            if (a[i].has_value() && nameOf(host_.slots[i]->spec()) == std::string_view{name}) {
                return &*a[i];
            }
        }
        return nullptr;
    }
    void tickImpl() {
        if (!bound()) {
            return;  // not bound yet: nothing to tick
        }
        SHULIB_PRECONDITION(!inTick_,
                            "MechanismService: a tick began inside another -- re-entered from within a tick, or a "
                            "previous tick never finished (its task deleted mid-tick, or a throw out of it): "
                            "enterDriverControl() / enterAutonomous() clear it");
        inTick_ = true;
        ++ticks_;
        const double t = host_.clock->now().value();
        for (std::size_t i = 0; i < host_.count; ++i) {
            Slot& s = *host_.slots[i];
            if (!s.active_) {
                continue;
            }
            if (s.holdsClaim()) {
                s.tickBehaviour(t);
            } else {
                // Its claim left it outside the service (released without the slot's cancel(), or then
                // taken): never commanded again by this slot. Taken — the new owner owns it, untouched;
                // released and free — a MOTOR mechanism is put SAFE on its way out (a motor left at its
                // last voltage by nobody is the half-safe failure, A2-D2); a PNEUMATIC one KEEPS its
                // level, as its own verdict leaves it — no mode change makes one safe either (driver
                // control's FIRST entry sends its START, nothing else does) — the level persists by
                // design: a claw must not drop what it holds (A2-D17). Said, if a step was running.
                s.cancelRunningStep(t, s.mech_->claimed() ? "its claim was taken by another owner"
                                                          : "its claim was released outside the service");
                s.stopBehaviour(s.kind() != Kind::Pneumatic);  // safe only when nobody holds it
            }
        }
        for (std::size_t i = 0; i < host_.count; ++i) {
            host_.slots[i]->snapshotReadFaults();
        }
        inTick_ = false;
    }
    /// A claim ANOTHER owner left on `s`'s mechanism: released WITHOUT its cancel(), counted, said.
    void releaseLeftover(Slot& s, const char* entering) {
        if (s.mech_ == nullptr || !s.mech_->claimed() || s.holdsClaim()) {
            return;
        }
        s.mech_->releaseClaim();  // NEVER its cancel(): it may have lived on a deleted task's stack
        ++leftoverClaimsReleased_;
        char line[200];
        std::snprintf(line, sizeof line,
                      "'%.23s': a claim left by another owner RELEASED as %s began (its operation was not "
                      "cancelled -- it may have lived on a deleted task)",
                      s.name(), entering);
        host_.telemetry->log(hal::LogLevel::Warn, "MECH", line);
    }

    motion::ITickPacer* inner_;
    detail::SlotHost host_{};  ///< the clock, latch, sink and slots every slot reads (bound() = clock set)
    std::array<std::optional<PowerSlot>, kMaxMechanisms> powers_{};
    std::array<std::optional<PneumaticSlot>, kMaxMechanisms> pneumatics_{};
    std::array<hal::IMechanism*, kMaxMechanisms> built_{};
    std::size_t builtCount_ = 0;
    long ticks_ = 0;
    int leftoverClaimsReleased_ = 0;
    bool inTick_ = false;
    bool enteredDriverOnce_ = false;
    bool driverTicked_ = false;  ///< tickFromDriverLoop() ran since the last pace()
};

}  // namespace shulib::mechanism
