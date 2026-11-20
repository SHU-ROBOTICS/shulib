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
// ── The four behaviours ──────────────────────────────────────────────────────────────
// MECH-A2 built the slot, the two seams, the claimant, the three mode changes, the lookups and the
// POWER and PNEUMATIC behaviours; MECH-A3 (2026-10-15) the POSITION (Arm, Lift) and SPEED (Flywheel)
// ones, so bind() takes every kind a robot's list can hold. Their rules, each pinned by a test:
//   * POSITION. Before it has found its zero a move is NOT_HOMED and a held button jogs it at most
//     `.unhomedVolts` (0: nothing moves it — no limits are known, and no power is invented). It finds
//     its zero with NO motion at bind (rests at its stop; an absolute angle sensor; a home switch
//     already pressed on a FRESH read) or by a homing STEP (push until its switch is pressed, or until
//     it stalls against its stop) — and every (re-)home logs where its zero went, a Warn when a
//     re-home moves it past the "reached" tolerance (homing into a wedged piece sets a wrong zero
//     silently otherwise). A move is a trapezoid profile in OUTPUT degrees (a lift's speed in inches
//     turned into degrees by its steepest slope, so the plan never asks more of the motor than the stated
//     speed there), tracked by P + D with feedforward (kS, kV, kA and gravity: kG·cos(angle −
//     level) for an arm, kG for a lift); when the profile ends a FRESH hold loop (P + I + D, its
//     integral clamped to kHoldIntegralLimitVolts) takes over, so the integral never winds up behind
//     the profile. REACHED is judged in the mechanism's OWN units (a degree tolerance converted at a
//     table's end or a linkage's top is 0), for the settle time: SUCCEEDED, and the HOLD goes on. Out
//     of time: TIMED_OUT, and it HOLDS WHERE IT STOPPED — never pushing on toward a target it could not
//     reach (a blocked arm would sit at stall current). No hold target is ever past a travel
//     limit. A constant target past a limit never compiles; a COMPUTED one is clamped and logged once
//     (the soft stop); a held button pushing toward a limit is taken over where it could no longer stop
//     before it at the smooth move's own acceleration — brought to the limit by the move loop, then
//     HELD AT THE LIMIT by a fresh closed-loop hold for the rest of that push (one soft stop per push),
//     never pushed toward the limit harder than the button pushes.
//   * SPEED. Feedforward (kS + kV·ω) plus P on the speed error, plus an integral that acts only within
//     kSpeedIntegralWindow of the target (from the start it winds up and overshoots) and is clamped to
//     kSpeedIntegralLimitVolts. "At speed" — within the band for its time — is SUCCEEDED, and it KEEPS
//     spinning until stopped; out of time is TIMED_OUT and it still spins (an unreachable wheel spins;
//     a jammed one trips its stall signature).
//   * THE STALL WATCH: a stated stall signature watches EVERY behaviour of a motor kind — a
//     move, the hold after it, a jog, the release's hold, a homing push by switch, a spin before and
//     after "at speed", a power run and a hold-to-run (homing by stall keeps its own signature). A trip
//     is STALLED, MECHANISM_STALLED once, the declared safe state, the claim released; one that ends a
//     behaviour AFTER its step (a hold, a jog, a spin) also bumps the command number — how the driver's
//     inputs (mechanism/driver_inputs.hpp, MECH-A4) see that a held button is superseded until it is released.
// The library's own numbers in all of this are HA-182 — invented, PROVISIONAL — in two groups. The
// starting gains, profile and "reached" (kinds.hpp) are per mechanism: startingValuesLine()
// (wiring.hpp) words them for a boot line (the programs print it since MECH-A5) and a declaration's
// `.tuning` replaces each one. The hold's integral clamp and the speed integral's window and clamp
// (below) are LIBRARY constants: not on the boot line, not `.tuning` fields — only an edit here
// changes them, for every mechanism.
//
// ── Re-entrancy (requirements :142-152) ──────────────────────────────────────────────
// A tick never calls a blocking verb, a wait or the scheduler's cancel(), and is never re-entered
// (a precondition). A predicate may READ a slot (`seen()`, `running()`), never tick it. A tick that
// never finished — its task deleted mid-tick by a PROS mode change (HA-161), or a throw out of a
// device — leaves the re-entrancy flag set: the next mode change (enterDriverControl /
// enterAutonomous) clears it.
//
// ── The honest limit ─────────────────────────────────────────────────────────────────
// The service runs ONLY when something ticks it: inside a shulib motion, wait or pause, and (from
// MECH-A4, which wires it into the driver loop) every driver-loop tick. A student's own busy loop (`pros::delay` in an auton, a then() action that
// blocks) STARVES it, and a starved service leaves each mechanism at its LAST command — not held,
// not limited, not safe (the brief's D11, measured: 11.74 V held for 1.5 s). The run guard says the
// same of itself: nothing preempts pure user code.
//
// ── The CURRENT CAP (chunk MECH-S-A, 2026-11-05; the MECH-S brief's D1 (a), D2 (a), D12 (b)) ─
// A mechanism that STATES `.currentCap` has it written into EVERY one of its motors at bind, through
// each motor's own limit (Devices::currentLimits — hal/motor_current_limit.hpp), and READ BACK; from
// then on the MOTOR enforces it, ticked or not (A4 register HA-219 — how fast, unmeasured). The boot
// says it once, per motor AND in all:
//     'lift': current cap 1.20 A per motor (2 motors: 2.40 A in all) -- set and read back
// and WARNS for each motor that would not take it (the port, the asked and the read-back numbers — the
// mechanism still runs), for a provider that gave no limit to set ("this provider cannot cap a
// motor"), and for each stated stall or homing signature within 0.2 A of the cap (a capped motor may
// read a little under its cap, so such a signature may never trip — A4 register HA-223; the build
// already refuses one at or above it). Every one of these comparisons is made in the DEVICE's unit,
// WHOLE MILLIAMPS (a V5 limit is whole mA): the cap as hal::motorCurrentLimitToMilliamps sends it, the
// read-back rounded to the mA — so a typed 2.4995 A (sent as 2500 mA) reads back as itself — and a
// signature as the FIRST whole-mA reading that trips it (a signature is never sent, so never rounded:
// 0.9996 A and 0.9994 A both trip at 1000 mA). The 0.2 A margin is 200 mA between those two integers,
// inclusive: under a 1.2 A cap, 1.0 A, 0.9996 A and 0.9994 A all warn; 0.999 A does not. A mechanism that states NO cap has each motor's limit READ and
// NOTHING written (D12 (b), until robot two's 10-motor boot is measured — HA-221): a limit that reads
// other than the motor's 2.5 A default gets one Info line naming both possible causes (a cap another
// program left — HA-220 — or VEXos's own sharing — HA-221) and "nothing written".
// ONCE A SECOND, on the service's clock and INSIDE a tick (never its own task — a starved service does
// not re-check, and the motor keeps its cap meanwhile: that is the point of D1), each capped motor's
// limit is read again: one that reads back the motor's DEFAULT (it lost power and came back — HA-220)
// is written again, with one Warn; one that reads ANY OTHER number is a Warn ONCE and is NOT written
// again (something else set it — the library never fights it every second). No verdict changes and
// nothing is refused at boot: the cap is a limit, never a gate. Choosing the number (four currents in
// ORDER — the heaviest hold < the signature < the cap − 0.2 A; the heaviest move's peak < the cap; the
// cap ≤ the string's safe pull) is the guide's (chapter 18); the cap does not limit a carriage's
// momentum at a stop (HA-222).
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

#include "shulib/control/pid.hpp"
#include "shulib/control/trapezoid_profile.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor_conversion.hpp"
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

static_assert(kDevicesMaxMotors == kMaxMotorRoles,
              "mechanism::Devices::currentLimits must hold every motor role a spec can have");

/// How close (A) a stated stall or homing signature may come to the CURRENT CAP before the boot warns
/// (MECH-S-A; A4 register HA-223 — invented: a stalled capped motor is believed to read within ~0.1 A of
/// its cap). A LIBRARY constant, like the integral clamps below.
inline constexpr double kCapSignatureMarginAmps = 0.2;
/// How often (s, on the service's clock, inside a tick) each capped motor's limit is read again
/// (MECH-S-A; HA-220 — a motor that lost power comes back at its default). A library constant.
inline constexpr double kCapRecheckSeconds = 1.0;

/// A step's verdict vocabulary (manipulation/mechanism_outcome.hpp), named here for the service's
/// readers: SUCCEEDED, UNCONFIRMED, TIMED_OUT, CANCELLED, STALLED, ABSENT, NOT_HOMED, REFUSED.
using manipulation::MechanismOutcome;

/// The position HOLD loop's integral clamp, volts: the I-term never adds more than this either way
/// (HA-182 — invented, PROVISIONAL; a LIBRARY constant for every mechanism, not a `.tuning` field and
/// not on the boot line). Enough to carry a gravity mismatch of a few volts; small enough that a hold
/// pushed off its target cannot wind up to full power.
inline constexpr double kHoldIntegralLimitVolts = 3.0;
/// The SPEED loop's integral acts only while the speed error is within this FRACTION of the target
/// (HA-182; a library constant, like the clamps). Accumulated through the whole spin-up it winds up
/// and overshoots (measured on the host test plant: +219 rpm at 3000 with a matched plant, +467 with
/// a lighter wheel).
inline constexpr double kSpeedIntegralWindow = 0.10;
/// The SPEED loop's integral clamp, volts (HA-182; a library constant).
inline constexpr double kSpeedIntegralLimitVolts = 3.0;

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
    /// The HOME switch reads PRESSED now, on a FRESH read: false with no home switch, and false for a
    /// sample taken while the switch's read-fault count moved since the end of the last tick (the
    /// read-fault rule — a held level is never a home: a switch that failed, or that was dead from
    /// boot, holds its last level, and with HA-184 wrong that level could read "pressed" and home the
    /// mechanism wherever it stands). `pressedLevel` is the level that means pressed (HA-184 —
    /// unmeasured). What it cannot see: an UNPLUGGED 3-wire switch reads a level, not a failed read.
    [[nodiscard]] bool homeSwitchPressed() const {
        if (devices_.homeSwitch == nullptr) {
            return false;
        }
        const bool pressed = devices_.homeSwitch->state() == spec_->homeSwitch.pressedLevel;
        if (devices_.homeSwitchFaults != nullptr && devices_.homeSwitchFaults->faultedReads() != homeFaultsSeen_) {
            return false;  // read AFTER the sample, as seen() does: a read that failed just now has counted
        }
        return pressed;
    }
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
        bootCurrentCap();
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
    /// The END of every tick (and bind): the read-fault counts a later sample is compared against —
    /// the piece sensor's and the home switch's.
    void snapshotReadFaults() {
        if (devices_.pieceFaults != nullptr) {
            pieceFaultsSeen_ = devices_.pieceFaults->faultedReads();
        }
        if (devices_.homeSwitchFaults != nullptr) {
            homeFaultsSeen_ = devices_.homeSwitchFaults->faultedReads();
        }
    }
    /// A STATED stall signature tripped while a behaviour went on AFTER its step (a hold, a jog, a
    /// spin): STALLED (said, `why`), MECHANISM_STALLED once, the declared safe state, the claim
    /// released, and the command number bumped — a held button is superseded until it is released
    /// (N10). Not a step: stepsBegun() does not count it (A2's drive() stall is the same).
    void stalledAfterStep(double t, const char* why) {
        ++seq_;
        stopBehaviour(true);
        finishStep(t, MechanismOutcome::Stalled, DoneBy::Stalled, false, why);
    }

    /// THE CURRENT CAP AT BOOT (the header's section): a STATED cap written to every motor and read
    /// back, said once (per motor and in all) or warned per motor; a signature within
    /// kCapSignatureMarginAmps of it warned; NO cap: every motor's limit READ, nothing written, a limit
    /// other than the 2.5 A default said once with both possible causes. A pneumatic or ABSENT
    /// mechanism has no motor: nothing. From the constructor (bind) — and, since TUNE1-A, from a live
    /// re-derive whose stated cap or signature changed (rederiveSlot()).
    void bootCurrentCap() {
        if (mech_ == nullptr || spec_->kind == Kind::Pneumatic) {
            return;
        }
        const double cap = spec_->currentCapAmps;
        const std::size_t n = std::min<std::size_t>(spec_->motorCount, kDevicesMaxMotors);
        nextCapCheck_ = now() + kCapRecheckSeconds;
        char line[240];
        char place[40];
        if (cap <= 0.0) {
            // D12 (b): READ, write nothing; a stray limit is said, never undone (HA-220, HA-221)
            for (std::size_t m = 0; m < n; ++m) {
                const hal::IMotorCurrentLimit* lim = devices_.currentLimits[m];
                if (lim == nullptr) {
                    continue;
                }
                const double reads = lim->currentLimit().value();
                if (milliamps(reads) != defaultMilliamps()) {
                    motorPlace(m, place, sizeof place);
                    std::snprintf(line, sizeof line,
                                  "'%.23s': motor %u (%.32s): its current limit reads %.2f A: a cap left by another "
                                  "program (HA-220), or VEXos's sharing with many motors plugged in (HA-221) -- "
                                  "nothing written",
                                  spec_->name, static_cast<unsigned>(m + 1), place, clampAmps(reads));
                    host_->telemetry->log(hal::LogLevel::Info, "MECH", line);
                }
            }
            return;
        }
        bool allTook = true;
        for (std::size_t m = 0; m < n; ++m) {
            hal::IMotorCurrentLimit* lim = devices_.currentLimits[m];
            motorPlace(m, place, sizeof place);
            if (lim == nullptr) {
                allTook = false;
                capState_[m] = CapState::OtherWarned;  // nothing to re-check on this motor
                std::snprintf(line, sizeof line,
                              "'%.23s': motor %u (%.32s): current cap %.2f A stated, but this provider cannot cap a "
                              "motor -- the motor's own 2.50 A applies",
                              spec_->name, static_cast<unsigned>(m + 1), place, clampAmps(cap));
                host_->telemetry->log(hal::LogLevel::Warn, "MECH", line);
                continue;
            }
            const bool accepted = lim->setCurrentLimit(units::Current{cap});
            const double reads = lim->currentLimit().value();
            if (accepted && milliamps(reads) == capMilliamps()) {
                capState_[m] = CapState::Ok;
                continue;
            }
            allTook = false;
            // a motor still at its default is re-written by the re-check (HA-220); any other number was
            // set by something else and is never fought (D2 (a))
            capState_[m] = milliamps(reads) == defaultMilliamps() ? CapState::LostWarned : CapState::OtherWarned;
            std::snprintf(line, sizeof line,
                          "'%.23s': motor %u (%.32s) did not take its current cap: asked %.2f A, %s, reads back "
                          "%.2f A -- it runs; that motor is not capped as stated",
                          spec_->name, static_cast<unsigned>(m + 1), place, clampAmps(cap),
                          accepted ? "the write was accepted" : "the write was refused", clampAmps(reads));
            host_->telemetry->log(hal::LogLevel::Warn, "MECH", line);
        }
        if (allTook) {
            std::snprintf(line, sizeof line, "'%.23s': current cap %.2f A per motor (%u motor%s: %.2f A in all) -- "
                                             "set and read back",
                          spec_->name, clampAmps(cap), static_cast<unsigned>(n), n == 1 ? "" : "s",
                          clampAmps(cap * static_cast<double>(n)));
            host_->telemetry->log(hal::LogLevel::Info, "MECH", line);
        }
        // every STATED signature must sit MORE than the margin below the cap to be sure to trip (HA-223),
        // judged as the MOTOR reads: the first whole-mA reading that trips the signature against the cap as
        // sent, so 200 mA under warns; the build has already refused one at or above the cap
        const struct {
            const StallSpec* st;
            const char* what;
        } signatures[] = {{&spec_->power.stall, "stall signature"},
                          {&spec_->position.stall, "stall signature"},
                          {&spec_->position.homeStall, "homing signature"},
                          {&spec_->speed.stall, "stall signature"}};
        for (const auto& sig : signatures) {
            if (sig.st->stated() && capMilliamps() - tripMilliamps(sig.st->amps) <= milliamps(kCapSignatureMarginAmps)) {
                std::snprintf(line, sizeof line,
                              "'%.23s': its %s (%.2f A) is within 0.2 A of its current cap (%.2f A): a capped motor "
                              "may read a little under its cap, so it may never trip (HA-223)",
                              spec_->name, sig.what, clampAmps(sig.st->amps), clampAmps(cap));
                host_->telemetry->log(hal::LogLevel::Warn, "MECH", line);
            }
        }
    }
    /// THE ONCE-A-SECOND RE-CHECK (the header's section), from the service's tick at clock time `t`: each
    /// capped motor's limit read; one back at the motor's DEFAULT written again (one Warn per loss); one at
    /// ANY OTHER number warned ONCE and never written. Nothing for a mechanism with no cap.
    void recheckCurrentCap(double t) {
        const double cap = spec_->currentCapAmps;
        if (mech_ == nullptr || spec_->kind == Kind::Pneumatic || cap <= 0.0 || t < nextCapCheck_) {
            return;
        }
        nextCapCheck_ = t + kCapRecheckSeconds;
        const std::size_t n = std::min<std::size_t>(spec_->motorCount, kDevicesMaxMotors);
        char line[240];
        char place[40];
        for (std::size_t m = 0; m < n; ++m) {
            hal::IMotorCurrentLimit* lim = devices_.currentLimits[m];
            if (lim == nullptr) {
                continue;
            }
            const double reads = lim->currentLimit().value();
            const long readsMa = milliamps(reads);
            if (readsMa == capMilliamps()) {
                capState_[m] = CapState::Ok;
                continue;
            }
            motorPlace(m, place, sizeof place);
            if (readsMa == defaultMilliamps()) {
                if (capState_[m] != CapState::LostWarned) {
                    std::snprintf(line, sizeof line,
                                  "'%.23s': motor %u (%.32s): its current limit reads 2.50 A, the motor's default (it "
                                  "lost power? HA-220) -- its cap %.2f A written again",
                                  spec_->name, static_cast<unsigned>(m + 1), place, clampAmps(cap));
                    host_->telemetry->log(hal::LogLevel::Warn, "MECH", line);
                }
                capState_[m] = CapState::LostWarned;
                if (lim->setCurrentLimit(units::Current{cap}) && milliamps(lim->currentLimit().value()) == capMilliamps()) {
                    capState_[m] = CapState::Ok;
                }
                continue;
            }
            if (capState_[m] != CapState::OtherWarned) {
                std::snprintf(line, sizeof line,
                              "'%.23s': motor %u (%.32s): its current limit reads %.2f A, not its cap %.2f A: "
                              "something else set it (another program, or VEXos's sharing -- HA-221) -- not written "
                              "again",
                              spec_->name, static_cast<unsigned>(m + 1), place, clampAmps(reads), clampAmps(cap));
                host_->telemetry->log(hal::LogLevel::Warn, "MECH", line);
                capState_[m] = CapState::OtherWarned;
            }
        }
    }
    /// Where motor `m` (0-based) is wired, as a log line says it — "port 12" — from the robot's list;
    /// "its port unknown to the service" when bound from bare specs.
    void motorPlace(std::size_t m, char* out, std::size_t cap) const {
        if (wiring_ != nullptr && m < kMaxMotorRoles && wiring_->motors[m] != 0) {
            std::snprintf(out, cap, "port %u", static_cast<unsigned>(wiring_->motors[m]));
        } else {
            std::snprintf(out, cap, "its port unknown to the service");
        }
    }
    /// A current in the DEVICE's unit, WHOLE MILLIAMPS (a V5 limit is whole mA — the header's section):
    /// rounded to the nearest mA, as hal::motorCurrentLimitToMilliamps rounds a cap on its way out, so a
    /// read-back (the device's mA / 1000) comes home to exactly the integer the device holds. A number
    /// no motor could report (not finite, below 0, above 1000 A) is -1, equal to no cap and no default.
    [[nodiscard]] static long milliamps(double a) noexcept {
        return (std::isfinite(a) && a >= 0.0 && a <= 1000.0) ? std::lround(a * 1000.0) : -1L;
    }
    /// The FIRST whole-mA reading that trips a signature of `a` A: the least r with r / 1000 >= a, through
    /// hal::motorMilliampsToCanonical — the StallDetector's own `>=` over a motor's reading (a V5 reports
    /// whole mA), so 0.9996 A and 0.9994 A both trip at 1000 mA, not at their nearest mA. A signature is
    /// never SENT, so it is never rounded. A number no signature could be is -1 (as milliamps()).
    [[nodiscard]] static long tripMilliamps(double a) {
        if (!(std::isfinite(a) && a > 0.0 && a <= 1000.0)) {
            return -1L;
        }
        long r = static_cast<long>(std::ceil(a * 1000.0));
        while (hal::motorMilliampsToCanonical(static_cast<double>(r - 1)).value() >= a) {
            --r;
        }
        while (hal::motorMilliampsToCanonical(static_cast<double>(r)).value() < a) {
            ++r;
        }
        return r;
    }
    /// The stated cap as the motor is SENT it, whole mA — hal::motorCurrentLimitToMilliamps, the one A→mA
    /// (ProsMotor sends exactly this). Only for a stated cap: the build has refused any other.
    [[nodiscard]] long capMilliamps() const {
        return hal::motorCurrentLimitToMilliamps(units::Current{spec_->currentCapAmps});
    }
    /// The motor's documented default limit, whole mA (2500 — hal::kDefaultMotorCurrentLimit).
    [[nodiscard]] static long defaultMilliamps() { return hal::motorCurrentLimitToMilliamps(hal::kDefaultMotorCurrentLimit); }
    /// A read-back bounded for "%.2f" (a device may answer anything; the line must provably fit).
    [[nodiscard]] static double clampAmps(double a) noexcept {
        return std::isfinite(a) ? std::clamp(a, -9999.99, 9999.99) : -1.0;
    }

    // ── LIVE TUNING: the slot re-reads its spec (chunk TUNE1-A, 2026-11-05; the MECH-S/TUNE1 brief's
    //    §1.2 — MechanismService::rederive() is the only caller) ─────────────────────────────────
    /// RE-DERIVE what this slot COPIED from its spec at bind (the brief's P6: the PIDs copy their gains,
    /// the stall detectors their signatures, the soft stop its limits — a spec changed under a slot
    /// without this changes nothing, or worse, half of the numbers): the kind's own copies first
    /// (rederiveKind()), then the CURRENT CAP when the cap or a signature changed — a stated cap through
    /// MECH-S-A's own boot path (bootCurrentCap(): written to every motor, read back, said, the 0.2 A
    /// margin re-judged), and a cap REMOVED (live, or by undoing one added live) by writing each motor
    /// back to its 2.5 A default, said once — the limit the Tune program set must not stay in the motor
    /// (the brief's D12 (b)). Everything the slot reads from its spec every tick (the feedforward, the
    /// gravity hold, the tolerance, a top speed at the next move) needs nothing here. An ABSENT slot
    /// re-derives its copies and touches no device.
    void rederiveSlot() {
        const bool signatureChanged = rederiveKind();
        const double cap = spec_->currentCapAmps;
        if (spec_->kind == Kind::Pneumatic || (cap == capInMotors_ && !(signatureChanged && cap > 0.0))) {
            capInMotors_ = cap;
            return;
        }
        const double was = capInMotors_;
        capInMotors_ = cap;
        if (mech_ == nullptr) {
            return;
        }
        if (cap > 0.0) {
            bootCurrentCap();  // MECH-S-A's path: write, read back, say; the margin Warns
            return;
        }
        if (!(was > 0.0)) {
            return;  // no cap before, none now: nothing was ever written
        }
        const std::size_t n = std::min<std::size_t>(spec_->motorCount, kDevicesMaxMotors);
        bool allTook = true;
        char line[240];
        char place[40];
        for (std::size_t m = 0; m < n; ++m) {
            // (capState_ is not touched: the re-check runs only under a stated cap, and a cap stated again
            // goes through bootCurrentCap(), which sets every motor's — TUNE1-A's review, MY10)
            hal::IMotorCurrentLimit* lim = devices_.currentLimits[m];
            if (lim == nullptr) {
                continue;
            }
            const bool accepted = lim->setCurrentLimit(hal::kDefaultMotorCurrentLimit);
            const double reads = lim->currentLimit().value();
            if (accepted && milliamps(reads) == defaultMilliamps()) {
                continue;
            }
            allTook = false;
            motorPlace(m, place, sizeof place);
            std::snprintf(line, sizeof line,
                          "'%.23s': motor %u (%.32s) did not go back to the motor's 2.50 A default after its live "
                          "cap was removed: it reads %.2f A",
                          spec_->name, static_cast<unsigned>(m + 1), place, clampAmps(reads));
            host_->telemetry->log(hal::LogLevel::Warn, "MECH", line);
        }
        if (allTook) {
            std::snprintf(line, sizeof line,
                          "'%.23s': current cap %.2f A removed live -- each motor's limit written back to the "
                          "motor's 2.50 A default",
                          spec_->name, clampAmps(was));
            host_->telemetry->log(hal::LogLevel::Info, "MECH", line);
        }
    }
    /// The kind's own re-derive (rederiveSlot()); true when a STATED stall or homing signature changed
    /// (the cap's 0.2 A margin is then judged again). A pneumatic copies nothing.
    virtual bool rederiveKind() { return false; }
    /// Whether the slot takes its (re-written) spec NOW — MechanismService::rederive() asks it ONCE, before
    /// anything is re-derived, and refuses on false (TUNE1-A's verification round: the decision is one
    /// read of the slot, never a precondition after the spec is written). Only a position slot can refuse
    /// (PositionSlot::takesRange()).
    [[nodiscard]] virtual bool takesSpecNow() const { return true; }
    /// A stall detector made to match `s` (TUNE1-A): built when `wanted` and it has none or one of
    /// another signature, dropped when not `wanted`; an unchanged one is KEPT, its window with it — a
    /// re-derive for a gain must not restart a stall that is half-way to tripping. True when it changed.
    static bool refreshDetector(std::optional<manipulation::StallDetector>& d, const StallSpec& s, bool wanted) {
        if (!wanted) {
            const bool had = d.has_value();
            d.reset();
            return had;
        }
        if (d.has_value() && d->config().currentAtLeast.value() == s.amps &&
            d->config().speedAtMost.value() == s.speedRadS && d->config().persistence.value() == s.time) {
            return false;
        }
        d.emplace(manipulation::StallConfig{units::Current{s.amps}, units::AngularVelocity{s.speedRadS},
                                            units::Time{s.time}});
        return true;
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
    int homeFaultsSeen_ = 0;     ///< the home switch's, likewise
    /// What the cap re-check last found on each motor (MECH-S-A): it decides which reading is said.
    enum class CapState : std::uint8_t {
        Ok,           ///< reads its cap (or no cap is stated)
        LostWarned,   ///< read the motor's DEFAULT: said once per loss, written again each check
        OtherWarned,  ///< read another number (or has no limit to read): said once, never written
    };
    std::array<CapState, kDevicesMaxMotors> capState_{};  ///< per motor, the re-check's last finding
    double nextCapCheck_ = 0.0;  ///< the clock time of the next once-a-second re-check
    /// The cap this slot last put in its motors, A (0: none — the boot wrote nothing): what a live
    /// re-derive compares the spec's against (TUNE1-A). Declared after `spec_`, so it starts at the bind's.
    double capInMotors_ = spec_->currentCapAmps;
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
    /// STEP1-B (the STEP1 brief's D8 (a)): run()'s preconditions, in run()'s order with run()'s
    /// messages — every one of them, and nothing else. run() calls it first; a Routine's partway
    /// trigger (`r.atPercent(60).startRun(...)`) calls it where the trigger is WRITTEN, so a trigger
    /// written wrong throws on its own line, every run, whether or not the drive ever reaches its
    /// mark. It reads only its arguments, this slot's declaration and the robot's list (peer()) —
    /// never run state — so the answer at the trigger's line is the answer when it fires. A check a
    /// later chunk adds to run() goes HERE, and triggers get it too. Returns the slot whose piece
    /// sensor the condition reads (this one, or the named peer).
    [[nodiscard]] const Slot* checkRun(units::Voltage volts, const Until& until, units::Time bound) const {
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
        return sensor;
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
        // and bumps no command number (the brief's D7 — a condition that can never be met). STEP1-B:
        // they are checkRun()'s, so a partway trigger checks the SAME list where it is written.
        const Slot* sensor = checkRun(volts, until, bound);
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
    /// LIVE TUNING (TUNE1-A; Slot::rederiveSlot()): the stall detector matched to the spec's signature;
    /// a hold-to-run in progress clamped to a lowered maxVolts at once (its next drive() would — this
    /// tick must not run past it). A step's volts are never re-read: a re-derive is refused while one runs.
    bool rederiveKind() override {
        const PowerSpec& q = spec_->power;
        const bool changed = refreshDetector(stall_, q.stall, q.stall.stated());
        if (driving_) {
            volts_ = std::clamp(volts_, -q.maxVolts, q.maxVolts);
        }
        return changed;
    }
    double volts_ = 0.0;
    bool driving_ = false;              ///< a hold-to-run is in progress (not a step)
    Until until_ = Until::bound();
    const Slot* sensorSlot_ = nullptr;  ///< whose piece sensor the step's condition reads
    int reads_ = 0;                     ///< the step's piece-sensor samples ...
    int blind_ = 0;                     ///< ... and how many of them were UNKNOWN
    std::optional<manipulation::StallDetector> stall_;
};

/// POSITION (an Arm or a Lift): go to a position smoothly and HOLD it; find its zero; keep inside its
/// travel limits (file header: the position rules). Positions are in the mechanism's units (degrees
/// or inches); the loops run on the OUTPUT angle in degrees, where gravity's cosine lives.
class PositionSlot final : public Slot {
public:
    /// Built by the service's bind(); `d.motors` is the actuator (nullptr when ABSENT). Homing that
    /// needs NO motion happens here, at boot, each logged: rests at its stop; an absolute angle
    /// sensor; a home switch already pressed on a fresh read. The service's clock must be set.
    PositionSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i,
                 const Devices& d)
        : Slot{host, spec, wiring, i, d, d.motors},
          movePid_{control::PidConfig{.kP = spec.position.kP, .kD = spec.position.kD, .outputMin = -12.0,
                                      .outputMax = 12.0},
                   *host.clock},
          holdPid_{control::PidConfig{.kP = spec.position.kP, .kI = spec.position.kI, .kD = spec.position.kD,
                                      .integralLimit = kHoldIntegralLimitVolts, .outputMin = -12.0,
                                      .outputMax = 12.0},
                   *host.clock} {
        const PositionSpec& p = spec.position;
        if (p.stall.stated()) {
            stall_.emplace(manipulation::StallConfig{units::Current{p.stall.amps},
                                                     units::AngularVelocity{p.stall.speedRadS},
                                                     units::Time{p.stall.time}});
        }
        if (p.home == HomeMethod::Stall && p.homeStall.stated()) {
            homeStall_.emplace(manipulation::StallConfig{units::Current{p.homeStall.amps},
                                                         units::AngularVelocity{p.homeStall.speedRadS},
                                                         units::Time{p.homeStall.time}});
        }
        const double a = positionToAngleDeg(p, p.minPosition);
        const double b = positionToAngleDeg(p, p.maxPosition);
        minDeg_ = std::min(a, b);
        maxDeg_ = std::max(a, b);
        // the smooth move's acceleration in OUTPUT degrees/s^2, by the move's own scale (moveTo()) — the
        // soft stop's stopping distance uses it
        accelDeg_ = p.maxAcceleration > 0.0 ? p.maxAcceleration * degPerUnit() : 1.0;
        if (mech_ == nullptr) {
            return;
        }
        // HOMING WITH NO MOTION, at boot
        if (p.home == HomeMethod::RestsAtStop) {
            zeroAt(p.homePosition);
        } else if (p.home == HomeMethod::Absolute) {
            zeroAbsolute();
        } else if (p.home == HomeMethod::Switch && homeSwitchPressed()) {
            zeroAt(p.homePosition);
        }
    }

    /// STEP1-B (the STEP1 brief's D8 (a)): moveTo()'s preconditions, in moveTo()'s order with its
    /// messages — every one of them, and nothing else (checkRun()'s rule: moveTo() calls it first, a
    /// partway trigger calls it where it is written; it reads only its arguments and the declaration).
    void checkMoveTo(double pos, units::Time bound, bool computed = true) const {
        const PositionSpec& p = spec_->position;
        SHULIB_PRECONDITION(std::isfinite(pos), "PositionSlot::moveTo: the target must be a finite number");
        SHULIB_PRECONDITION(std::isfinite(bound.value()) && bound.value() > 0.0,
                            "r.move(...): the time limit must be finite and above 0");
        SHULIB_PRECONDITION(computed || (pos >= p.minPosition && pos <= p.maxPosition),
                            "PositionSlot::moveTo: a CONSTANT target outside the travel limits (the build refuses "
                            "these; a computed one is clamped -- clampedTo(x))");
    }
    /// Move to `pos` (the mechanism's units) within `bound`, then HOLD there (file header). Before it
    /// has found its zero: NOT_HOMED, the safe state, nothing moves. A COMPUTED target past a travel
    /// limit is CLAMPED to it and logged once ("target 175.00 clamped to the travel limit 160.00 (soft
    /// stop)"). PRECONDITIONS (checked before anything changes): `pos` finite; `bound` finite and > 0;
    /// a target that is not `computed` (a preset, `arm.to(95)` — the build checked it) inside the
    /// travel limits.
    void moveTo(double pos, units::Time bound, bool computed = true) {
        const PositionSpec& p = spec_->position;
        checkMoveTo(pos, bound, computed);  // STEP1-B: the preconditions, shared with a partway trigger
        const double t = now();
        if (!beginStep(bound.value(), t)) {
            return;
        }
        if (!homed_) {
            stopBehaviour(true);  // whatever it was doing (a jog before homing) stops, safe
            finishStep(t, MechanismOutcome::NotHomed, DoneBy::None, false, notHomedNote());
            return;
        }
        if (pos < p.minPosition || pos > p.maxPosition) {  // the runtime soft stop: clamp, and say so ONCE
            const double asked = pos;
            pos = std::clamp(pos, p.minPosition, p.maxPosition);
            ++softStops_;
            char line[160];
            std::snprintf(line, sizeof line, "'%.23s': target %.2f clamped to the travel limit %.2f (soft stop)",
                          spec_->name, std::clamp(asked, -99999.0, 99999.0), pos);
            host_->telemetry->log(hal::LogLevel::Warn, "MECH", line);
        }
        targetPos_ = pos;
        fromDeg_ = jointDeg();
        toDeg_ = std::clamp(positionToAngleDeg(p, pos), minDeg_, maxDeg_);
        // The profile in OUTPUT degrees: the declaration's speed and acceleration are in its units,
        // turned into degrees by ONE scale (degPerUnit()) — so the plan never asks more of the motor
        // than the stated speed at the mechanism's steepest slope.
        const double scale = degPerUnit();
        profile_.emplace(toDeg_ - fromDeg_,
                         control::ProfileConstraints{p.maxVelocity * scale, p.maxAcceleration * scale});
        movePid_.reset();
        holdPid_.reset();
        holdLoop_ = false;
        inTolSince_ = -1.0;
        if (stall_) {
            stall_->reset();
        }
        mode_ = Mode::Move;
        active_ = true;
    }
    /// Move to preset `i` (a named value the build checked against the limits). PRECONDITION: `i` is
    /// one of its presets.
    void moveToPreset(std::uint8_t i, units::Time bound) {
        SHULIB_PRECONDITION(i < spec_->position.presetCount, "PositionSlot::moveToPreset: no such preset");
        moveTo(spec_->position.presets[i].value, bound, false);
    }
    /// HOLD WHERE IT IS (gravity included; the target clamped to the travel limits) — a driver's
    /// release of a hold-to-run. Not a step; a running step ends CANCELLED (the most recent command
    /// wins). Before homing there is no position to hold: the declared safe state instead. Does
    /// nothing on a mechanism another owner holds, or one that is ABSENT.
    void holdHere() {
        if (mech_ == nullptr || (mech_->claimed() && !holdsClaim())) {
            return;
        }
        ++seq_;
        const double t = now();
        cancelRunningStep(t, "replaced by a newer command");
        stepStart_ = t;  // not a step: when this behaviour began (a stall's line counts from here)
        beginHold();
    }
    /// STEP1-B (the STEP1 brief's D8 (a)): home()'s preconditions, in home()'s order with its messages
    /// — every one of them, and nothing else (checkRun()'s rule).
    void checkHome(units::Time bound) const {
        const PositionSpec& p = spec_->position;
        SHULIB_PRECONDITION(std::isfinite(bound.value()) && bound.value() > 0.0,
                            "r.home(...): the time limit must be finite and above 0");
        SHULIB_PRECONDITION(p.home != HomeMethod::None,
                            "r.home(...): the mechanism is declared neverHomed -- it has no way to find its zero "
                            "(give it .home = homeBySwitch / homeByStall / restsAtStop / absoluteSensor)");
    }
    /// FIND ITS ZERO by the declaration's method, within `bound`: a homing push (`homeBySwitch`: until
    /// its switch is pressed on a fresh read; `homeByStall`: until its homing signature trips) — then
    /// SUCCEEDED, holding there, its zero logged; out of time TIMED_OUT, the safe state, the zero it
    /// had (if any) kept — a stall-home's line then says the most current its push drew when stopped
    /// (no faster than its homing signature's speed; the number that signature is sized by), or that it
    /// never got that slow. An absolute sensor is READ AGAIN (no motion; SUCCEEDED; a moved zero is
    /// logged). A mechanism that RESTS AT ITS STOP is not re-zeroed: it trusts the pose it had at boot,
    /// never the one it has now — SUCCEEDED with no change, held where it is. A homing push by switch
    /// is watched by the stated stall signature (STALLED: pushed into something with the switch never
    /// pressed). PRECONDITIONS: `bound` finite and > 0; the declaration has a homing method
    /// (`neverHomed` has no way to find its zero).
    void home(units::Time bound) {
        const PositionSpec& p = spec_->position;
        checkHome(bound);  // STEP1-B: the preconditions, shared with a partway trigger
        const double t = now();
        if (!beginStep(bound.value(), t)) {
            return;
        }
        if (p.home == HomeMethod::RestsAtStop || p.home == HomeMethod::Absolute) {
            const char* note = "it rests at its stop at boot: its zero is kept";
            if (p.home == HomeMethod::Absolute) {
                zeroAbsolute();
                note = "its angle sensor read again";
            }
            beginHold();
            finishStep(t, MechanismOutcome::Succeeded, DoneBy::Reached, false, note);
            return;
        }
        if (homeStall_) {
            homeStall_->reset();
        }
        if (stall_) {
            stall_->reset();
        }
        homeStoppedAmps_ = -1.0;  // this push's own record, from nothing: it has not pushed yet
        homePushed_ = false;
        mode_ = Mode::Home;
        active_ = true;
    }
    /// The DRIVER's hold-to-run (`whileHeld`): `volts` every tick until another command — open-ended;
    /// the button's release is the caller's holdHere(). Once homed, a push toward a travel limit is
    /// taken over and HELD AT THE LIMIT for the rest of that push (file header: the soft stop); before
    /// homing it runs at most `.unhomedVolts` either
    /// way, and with none it does nothing (the safe state). 0 V holds where it is (holdHere()). The
    /// stated stall signature watches it. Call it once or every tick (each call restates the
    /// volts). Does nothing on a mechanism another owner holds, or one that is ABSENT. PRECONDITION:
    /// `volts` finite.
    void jog(units::Voltage volts) {
        SHULIB_PRECONDITION(std::isfinite(volts.value()), "PositionSlot::jog: the volts must be a finite number");
        if (mech_ == nullptr || (mech_->claimed() && !holdsClaim())) {
            return;
        }
        if (volts.value() == 0.0) {
            holdHere();
            return;
        }
        ++seq_;
        const double t = now();
        cancelRunningStep(t, "replaced by a newer command");
        if (!homed_ && !(spec_->position.unhomedVolts > 0.0)) {
            stopBehaviour(true);  // no limits known and no power stated for before homing: nothing moves it
            return;
        }
        if (!holdsClaim()) {
            (void)mech_->tryClaim(*this);
        }
        if (mode_ != Mode::Jog || !active_) {  // a NEW hold-to-run: its stall window and its time start now
            resetBehaviour();
            stepStart_ = t;
            if (stall_) {
                stall_->reset();
            }
        }
        jogVolts_ = std::clamp(volts.value(), -12.0, 12.0);
        mode_ = Mode::Jog;
        active_ = true;
    }
    /// LET GO: the declared safe state, the claim released, a running step CANCELLED (a hold is
    /// dropped — under Hold the motor's own brake keeps what it can). Does nothing on a mechanism
    /// another owner holds, or one that is ABSENT.
    void release() {
        if (mech_ == nullptr || (mech_->claimed() && !holdsClaim())) {
            return;
        }
        ++seq_;
        cancelRunningStep(now(), "released");
        stopBehaviour(true);
    }
    /// True once it has found its zero (at boot or by a homing step).
    [[nodiscard]] bool homed() const noexcept { return homed_; }
    /// Where it is now, in its units — meaningful once homed (before, it is the encoders' — or the angle
    /// sensor's — own count through the gearing, with no zero applied). 0 when ABSENT.
    [[nodiscard]] double position() const {
        return mech_ != nullptr ? angleDegToPosition(spec_->position, jointDeg()) : 0.0;
    }
    /// The position the move or the hold aims at, in its units (inside the travel limits).
    [[nodiscard]] double target() const noexcept { return targetPos_; }
    /// How many times the soft stop acted: a computed target clamped, or a push taken over and held at
    /// a limit (once per push, however long it is held there).
    [[nodiscard]] int softStops() const noexcept { return softStops_; }
    /// The OUTPUT angle in degrees (an angle sensor's reading when one is wired, else the motor
    /// encoders through the gearing), with its zero applied once homed. 0 when ABSENT.
    [[nodiscard]] double jointDeg() const { return mech_ != nullptr ? rawJointDeg() + offsetDeg_ : 0.0; }
    /// True while it HOLDS a target (TUNE1-A — what live tuning's "never narrowed past where it is
    /// holding" reads): homed and active in a hold (after a move, a release of a hold-to-run, a homing)
    /// or a held button's soft stop at a limit. Its target is target().
    [[nodiscard]] bool holding() const noexcept {
        return active_ && homed_ && (mode_ == Mode::Hold || (mode_ == Mode::Jog && limitDir_ != 0.0));
    }
    /// True while a HELD BUTTON moves it (TUNE1-A): a hold-to-run in progress that the soft stop has not
    /// taken over at a limit (once it has, holding() is true instead).
    [[nodiscard]] bool heldButtonMoving() const noexcept {
        return active_ && mode_ == Mode::Jog && limitDir_ == 0.0;
    }
    /// Whether a live re-derive may give it `candidate`'s travel limits NOW (TUNE1-A) — live tuning's
    /// check BEFORE it writes the spec (for its reason and sentence) and rederive()'s refusal AFTER (the
    /// decision that counts), through this one function. The where-it-is clause READS THE SENSOR, so the
    /// two calls can disagree when a reading moves between them (a coasting or drifting lift, jitter at the
    /// limit): rederive() then REFUSES — nothing re-derived, nothing said — and live tuning puts the old
    /// spec back and records nothing (TUNE1-A's verification round). False while it HOLDS a target the
    /// new limits leave out (the brief's re-check #7: a hold is never moved — refused instead); while a
    /// held button moves it and the
    /// limits change (the soft stop would meet a push against a limit it was not braking for); and,
    /// homed and neither, when the new limits leave WHERE IT IS further outside them than the limits in
    /// force do (TUNE1-A's review: the next hold — a HOME of a mechanism that rests at its stop, a
    /// holdHere(), a timed-out move — begins at where it is CLAMPED inside the limits, so a limit
    /// narrowed past it there would make that hold a jump with no smooth move: 8 in narrowed away asks
    /// −11 V at once; a widening, or a change that leaves it as far outside as it was, never does).
    /// Compared in OUTPUT DEGREES, the hold's and the soft stop's own unit, so a target set by a move to
    /// exactly the new limit is inside it.
    [[nodiscard]] bool takesRange(const PositionSpec& candidate) const noexcept {
        const double a = positionToAngleDeg(candidate, candidate.minPosition);
        const double b = positionToAngleDeg(candidate, candidate.maxPosition);
        const double lo = std::min(a, b);
        const double hi = std::max(a, b);
        if (holding()) {
            return toDeg_ >= lo && toDeg_ <= hi;
        }
        if (heldButtonMoving()) {
            return lo == minDeg_ && hi == maxDeg_;
        }
        if (!homed_ || mech_ == nullptr) {
            return true;  // no limits act before homing; an ABSENT one never holds
        }
        const double jd = jointDeg();
        const auto outside = [jd](double l, double h) { return std::max(std::max(l - jd, jd - h), 0.0); };
        return !(outside(lo, hi) > outside(minDeg_, maxDeg_));
    }

private:
    /// Its travel limits taken NOW (Slot::takesSpecNow()): takesRange() of the spec it runs — read once,
    /// by MechanismService::rederive(), before anything is re-derived.
    [[nodiscard]] bool takesSpecNow() const override { return takesRange(spec_->position); }
    /// LIVE TUNING (TUNE1-A; Slot::rederiveSlot()): the two loops' gains (BUMPLESS — Pid::setGains keeps
    /// the hold's integral TERM), the stall and homing-stall detectors matched to the spec, and the
    /// travel limits in output degrees and the soft stop's acceleration recomputed. A hold is NEVER moved:
    /// a range narrowed past the held target, or past where it is when it holds nothing (the brief's
    /// re-check #7 — holdAt()'s re-clamp is a jump to a new target with no smooth move: 8 in × 76.4 °/in ×
    /// kP 0.25 asks 153 V), never reaches here — MechanismService::rederive() asked takesSpecNow() just
    /// before and REFUSED it (TUNE1-A's verification round: once a PRECONDITION here, but its where-it-is
    /// clause reads the sensor, and a reading that moved since live tuning's own check threw out of a
    /// refusal-checked change with the spec half-applied).
    bool rederiveKind() override {
        const PositionSpec& p = spec_->position;
        const double a = positionToAngleDeg(p, p.minPosition);
        const double b = positionToAngleDeg(p, p.maxPosition);
        const double lo = std::min(a, b);
        const double hi = std::max(a, b);
        movePid_.setGains(p.kP, 0.0, p.kD);  // the MOVE loop is P + D only (header)
        holdPid_.setGains(p.kP, p.kI, p.kD);
        bool changed = refreshDetector(stall_, p.stall, p.stall.stated());
        changed = refreshDetector(homeStall_, p.homeStall, p.home == HomeMethod::Stall && p.homeStall.stated()) ||
                  changed;
        minDeg_ = lo;
        maxDeg_ = hi;
        accelDeg_ = p.maxAcceleration > 0.0 ? p.maxAcceleration * degPerUnit() : 1.0;
        return changed;
    }
    /// What the slot is doing.
    enum class Mode : std::uint8_t { Idle, Move, Hold, Home, Jog };

    void tickBehaviour(double t) override {
        switch (mode_) {
            case Mode::Idle:
                stopBehaviour(true);  // defensive: an active slot always has a behaviour
                return;
            case Mode::Jog:
                tickJog(t);
                return;
            case Mode::Home:
                tickHome(t);
                return;
            case Mode::Move:
            case Mode::Hold:
                tickMoveOrHold(t);
                return;
        }
    }
    void resetBehaviour() noexcept override {
        mode_ = Mode::Idle;
        holdLoop_ = false;
        limitDir_ = 0.0;
        jogVolts_ = 0.0;
        inTolSince_ = -1.0;
        profile_.reset();
    }
    /// True when the stated stall signature trips on this tick's sample (false with none stated).
    [[nodiscard]] bool stallTripped(double t) {
        return stall_ && stall_->update(units::Time{t}, devices_.motors->maxCurrent(), devices_.motors->meanVelocity());
    }
    void tickMoveOrHold(double t) {
        const PositionSpec& p = spec_->position;
        // THE STALL WATCH (N10) on the move AND the hold after it: a blocked move, or a hold pushed past
        // what it can carry, never sits at stall current.
        if (stallTripped(t)) {
            if (running()) {
                stopBehaviour(true);
                finishStep(t, MechanismOutcome::Stalled, DoneBy::Stalled, false, "a stated stall signature tripped");
            } else {
                stalledAfterStep(t, "a stated stall signature tripped while holding");
            }
            return;
        }
        const double jd = jointDeg();
        const double since = t - stepStart_;
        double set = toDeg_;
        double vel = 0.0;
        double acc = 0.0;
        const bool profiling = mode_ == Mode::Move && profile_.has_value() && !profile_->isDone(since);
        if (profiling) {
            const control::ProfileState s = profile_->sample(since);
            set = fromDeg_ + s.position;
            vel = s.velocity;
            acc = s.acceleration;
        } else if (!holdLoop_) {
            holdPid_.reset();  // the profile is over: a FRESH hold loop (with its integral) takes over
            holdLoop_ = true;
        }
        double u = feedforward(vel, acc, jd) + (holdLoop_ ? holdPid_.update(set, jd) : movePid_.update(set, jd));
        if (holdLoop_ && !profiling && atStopEnd(jd, angleDegToPosition(p, set))) {
            u = restOnStop(u, jd);  // the team lead, 2026-11-20: robot one's cascade stuttered on its bottom stop
        }
        command(u);
        if (mode_ != Mode::Move || !running()) {
            return;
        }
        // REACHED, judged in the mechanism's OWN units, once the profile is done, for the settle time.
        if (!profiling && std::abs(angleDegToPosition(p, jd) - targetPos_) <= p.tolerance) {
            if (inTolSince_ < 0.0) {
                inTolSince_ = t;
            }
            if (t - inTolSince_ >= p.settleTime) {
                mode_ = Mode::Hold;  // arrived: the behaviour goes on as a HOLD
                finishStep(t, MechanismOutcome::Succeeded, DoneBy::Reached);
                return;
            }
        } else {
            inTolSince_ = -1.0;
        }
        if (since >= bound_) {
            // Out of time: HOLD WHERE IT STOPPED (N2), never pushing on toward a target it could not
            // reach — clamped inside the travel limits.
            holdAt(jd);
            finishStep(t, MechanismOutcome::TimedOut, DoneBy::None, false, "it holds where it stopped");
        }
    }
    void tickJog(double t) {
        if (stallTripped(t)) {
            stalledAfterStep(t, "a stated stall signature tripped while held to move");
            return;
        }
        const PositionSpec& p = spec_->position;
        if (!homed_) {  // no limits known: at most the stated volts before homing
            command(std::clamp(jogVolts_, -p.unhomedVolts, p.unhomedVolts));
            return;
        }
        // THE SOFT STOP, CLOSED-LOOP: a push toward a limit that could no longer stop before it at the
        // smooth move's own acceleration — or one at or past it — is taken over for the rest of that
        // push, with the move's own two loops: the MOVE loop (P + D and gravity, no integral to wind up
        // on the way) brings it to the limit, and once it gets there — or once a stop at the move's
        // acceleration would have taken (its speed then over that acceleration) — a FRESH hold loop
        // (P + I + D and gravity) HOLDS it there. It never pushes toward the limit harder than the push
        // itself. One soft stop per push, the hold's integral kept, until the push turns away from the
        // limit or ends.
        const double dir = jogVolts_ > 0.0 ? 1.0 : -1.0;
        if (limitDir_ != 0.0 && limitDir_ != dir) {
            limitDir_ = 0.0;  // the push turned away from the limit it was held at: the driver's again
        }
        const double jd = jointDeg();
        const double jv = jointVelDegS();
        if (limitDir_ == 0.0) {
            const double limit = dir > 0.0 ? maxDeg_ : minDeg_;
            const double toGo = dir * (limit - jd);        // <= 0: at or past the limit
            const double v0 = std::max(0.0, dir * jv);     // its speed TOWARD the limit
            if (toGo > v0 * v0 / (2.0 * accelDeg_)) {      // it can still stop before the limit: the push's
                command(jogVolts_);
                return;
            }
            limitDir_ = dir;
            ++softStops_;
            approachEnd_ = t + v0 / accelDeg_;
            toDeg_ = limit;
            targetPos_ = angleDegToPosition(p, limit);
            movePid_.reset();
            holdLoop_ = false;
        }
        if (!holdLoop_ && (limitDir_ * (jd - toDeg_) >= 0.0 || t >= approachEnd_)) {
            holdPid_.reset();  // at the limit, or the stop's time is up: a FRESH hold loop holds it there
            holdLoop_ = true;
        }
        double u = gravityVolts(jd) + (holdLoop_ ? holdPid_.update(toDeg_, jd) : movePid_.update(toDeg_, jd));
        if (limitDir_ * u > std::abs(jogVolts_)) {
            u = limitDir_ * std::abs(jogVolts_);  // never toward the limit harder than the push itself
        }
        if (holdLoop_ && limitDir_ == static_cast<double>(stopEnd()) && atStopEnd(jd, targetPos_)) {
            u = restOnStop(u, jd);  // the team lead, 2026-11-20: "on the bottom ... it doesnt auto stop so it stalls"
        }
        command(u);
    }
    void tickHome(double t) {
        const PositionSpec& p = spec_->position;
        const bool bySwitch = p.home == HomeMethod::Switch;
        const bool pressed = bySwitch && homeSwitchPressed();
        const units::Current amps = devices_.motors->maxCurrent();
        const units::AngularVelocity vel = devices_.motors->meanVelocity();
        // What the PUSH drew when STOPPED (no faster than its homing signature's speed) — the number a
        // homing signature is sized by. Never the first tick's sample (it shows the behaviour BEFORE the
        // push: a pinned jog's, a loaded hold's), and never one while it moves faster (braking a swing
        // the other way draws more than any stall at the push's volts).
        if (homePushed_ && std::abs(vel.value()) <= p.homeStall.speedRadS) {
            homeStoppedAmps_ = std::max(homeStoppedAmps_, amps.value());
        }
        const bool stalled = !bySwitch && homeStall_ && homeStall_->update(units::Time{t}, amps, vel);
        if (pressed || stalled) {
            zeroAt(p.homePosition);
            beginHold();
            // the HOLD this very tick: never one more tick of the homing push past its zero
            const double jd = jointDeg();
            command(gravityVolts(jd) + holdPid_.update(toDeg_, jd));
            finishStep(t, MechanismOutcome::Succeeded, DoneBy::Reached);
            return;
        }
        if (bySwitch && stallTripped(t)) {  // pushed into something, its switch never pressed
            stopBehaviour(true);
            finishStep(t, MechanismOutcome::Stalled, DoneBy::Stalled, false,
                       "a stated stall signature tripped while homing: its home switch never read pressed");
            return;
        }
        if (t - stepStart_ >= bound_) {
            // a stall-home that never trips says what the push DREW when stopped against what its
            // signature needs — a gentle homing push draws far less than a jam (chapter 18's homing
            // signature rule) — or that it never got that slow at all (it never reached its stop)
            char note[120];
            std::snprintf(note, sizeof note, "it never stalled against its stop: it drew at most %.2f A when stopped, "
                          "its homing signature needs %.2f A", std::clamp(homeStoppedAmps_, 0.0, 99.0),
                          std::clamp(p.homeStall.amps, 0.0, 99.0));
            stopBehaviour(true);
            finishStep(t, MechanismOutcome::TimedOut, DoneBy::None, false,
                       bySwitch                  ? "its home switch never read pressed"
                       : homeStoppedAmps_ < 0.0 ? "it never stalled against its stop: it never slowed to its homing "
                                                   "signature's speed -- did it reach its stop?"
                                                 : note);
            return;
        }
        command(p.homeVolts);
        homePushed_ = true;  // from the next tick on, a sample shows the push
    }
    /// Begin a HOLD where it is (clamped), a fresh hold loop; the claim taken. Before homing: the safe
    /// state instead (no position to hold).
    void beginHold() {
        resetBehaviour();
        if (!homed_) {
            stopBehaviour(true);
            return;
        }
        if (!holdsClaim()) {
            (void)mech_->tryClaim(*this);
        }
        if (stall_) {
            stall_->reset();
        }
        holdAt(jointDeg());
        active_ = true;
    }
    /// Hold at `deg` — CLAMPED inside the travel limits (no hold target is ever past one), a FRESH hold
    /// loop.
    void holdAt(double deg) {
        toDeg_ = std::clamp(deg, minDeg_, maxDeg_);
        targetPos_ = angleDegToPosition(spec_->position, toDeg_);
        holdPid_.reset();
        holdLoop_ = true;
        limitDir_ = 0.0;
        mode_ = Mode::Hold;
    }
    /// Output degrees per unit of the mechanism, for its speed and acceleration: ONE scale, the inverse
    /// of its STEEPEST units per degree (kinds.hpp's detail::unitsPerOutputDeg — 1 for an arm, exact for a
    /// spool, a linkage's level slope, a measured table's steepest segment). A plan in degrees at the
    /// stated speed times this is never faster in units than stated anywhere on the travel, and a
    /// starting-value plan is 75 % of the output's free speed everywhere. (A move's MEAN slope, the
    /// first version's scale, asked up to four times the free speed near a linkage's top.)
    [[nodiscard]] double degPerUnit() const noexcept {
        return 1.0 / std::max(detail::unitsPerOutputDeg(spec_->position), 1e-9);
    }
    /// The feedforward at the plan's speed `vel` (output deg/s) and acceleration `acc` (deg/s^2), at
    /// output angle `jd`, volts: kS in the direction of travel, kV·vel, kA·acc and the gravity hold.
    [[nodiscard]] double feedforward(double vel, double acc, double jd) const noexcept {
        const PositionSpec& p = spec_->position;
        return (vel > 0.0 ? p.kS : vel < 0.0 ? -p.kS : 0.0) + p.kV * vel + p.kA * acc + gravityVolts(jd);
    }
    /// The gravity hold at output angle `jd`, volts: kG·cos(jd − level) (Cosine), kG (Constant), 0.
    [[nodiscard]] double gravityVolts(double jd) const noexcept {
        const PositionSpec& p = spec_->position;
        switch (p.gravity) {
            case Gravity::Cosine:
                return p.kG * std::cos((jd - p.horizontalAt) * kDegToRad);
            case Gravity::Constant:
                return p.kG;
            case Gravity::None:
                return 0.0;
        }
        return 0.0;
    }
    /// -1 when the travel's BOTTOM is the hard stop its homing STALLED on (the one stop a homing proves: a switch may
    /// sit above its stop, a rest-at-start may be a claim), +1 the top, 0 neither (2026-11-20). A hold or a held push
    /// that has ARRIVED there rests on that stop (restOnStop).
    [[nodiscard]] int stopEnd() const noexcept {
        const PositionSpec& p = spec_->position;
        if (p.home != HomeMethod::Stall) {
            return 0;
        }
        return std::abs(p.homePosition - p.minPosition) <= p.tolerance ? -1
             : std::abs(p.homePosition - p.maxPosition) <= p.tolerance ? 1 : 0;
    }
    /// True when it sits AT its stop end (within the tolerance, or past it), held toward `target` there: on a string
    /// lift the stop gives a little, so a target ON the stop is never quite reached -- the team lead, 2026-11-20:
    /// robot one's cascade "stalls rather than stops" at the bottom, its hold pushing on into the stop until its stall
    /// signature cut it, again and again. (Moving or not: braking away from the stop is never limited.)
    [[nodiscard]] bool atStopEnd(double jd, double target) const noexcept {
        const int e = stopEnd();
        if (e == 0) {
            return false;
        }
        const PositionSpec& p = spec_->position;
        const double edge = e < 0 ? p.minPosition : p.maxPosition;
        return e * (target - edge) >= -p.tolerance && e * (angleDegToPosition(p, jd) - edge) >= -p.tolerance;
    }
    /// `u` resting on the stop end. On a BOTTOM stop: nothing at all -- the stop carries the weight, and a string
    /// lift's slack spool, free of its load there, only oscillates under a hold tuned for the load (robot one's log,
    /// 2026-11-20: its motors flipping every 50-100 ms on the bottom). On a top stop: never toward it beyond holding
    /// its own weight; away from it, untouched.
    [[nodiscard]] double restOnStop(double u, double jd) const noexcept {
        const int e = stopEnd();
        if (e < 0) {
            return 0.0;
        }
        const double most = std::max(0.0, e * gravityVolts(jd));  // toward the stop: at most what holds its weight
        return e * u > most ? e * most : u;
    }
    /// EVERY command it sends: the motor's 12 V either way, and DOWN (negative) at most the declaration's
    /// maxDownVolts (API 2.10) — a held button, a move, a preset and the hold alike.
    void command(double volts) {
        const double down = spec_->position.maxDownVolts > 0.0 ? spec_->position.maxDownVolts : 12.0;
        devices_.motors->setVoltage(units::Voltage{std::clamp(volts, -down, 12.0)});
    }
    /// The output angle before the zero: the angle sensor (its declared mounting applied — the port is
    /// never signed for a sensor, mechanism::toMechanismConfig) or the motors through the gearing.
    [[nodiscard]] double rawJointDeg() const {
        if (devices_.angleSensor != nullptr) {
            return (spec_->angleSensor.reversed ? -1.0 : 1.0) * devices_.angleSensor->position().value() / kDegToRad;
        }
        return devices_.motors->meanPosition().value() / kDegToRad / spec_->position.gearRatio;
    }
    [[nodiscard]] double jointVelDegS() const {
        if (devices_.angleSensor != nullptr) {
            return (spec_->angleSensor.reversed ? -1.0 : 1.0) * devices_.angleSensor->velocity().value() / kDegToRad;
        }
        return devices_.motors->meanVelocity().value() / kDegToRad / spec_->position.gearRatio;
    }
    /// ZERO: it is at `pos` now. Logged (D10): the first time, where the zero sits against where its
    /// encoders (or its angle sensor) read zero at boot; again, how far the zero MOVED — a Warn past
    /// the "reached" tolerance (something was in the way, or it slipped).
    void zeroAt(double pos) {
        setOffset(positionToAngleDeg(spec_->position, pos) - rawJointDeg(), nullptr);
    }
    /// ABSOLUTE (HA-186): the sensor reads `absoluteSensorAtZeroDeg` when the mechanism is at its home
    /// position, so the output angle is home + (reading − that reading), the declared mounting applied
    /// — a FIXED offset, never "zero wherever it stands". The reading at power-up is within one turn,
    /// so the turn is chosen that puts the mechanism nearest the middle of its travel (the build
    /// refuses absolute homing over a travel of a whole sensor turn or more, where no turn is right).
    void zeroAbsolute() {
        const PositionSpec& p = spec_->position;
        const double sgn = spec_->angleSensor.reversed ? -1.0 : 1.0;
        double offset = positionToAngleDeg(p, p.homePosition) - sgn * p.absoluteSensorAtZeroDeg;
        const double mid = 0.5 * (minDeg_ + maxDeg_);
        offset += 360.0 * std::round((mid - (rawJointDeg() + offset)) / 360.0);
        char how[96];
        std::snprintf(how, sizeof how, "its angle sensor reads %.1f deg",
                      std::clamp(sgn * rawJointDeg(), -99999.0, 99999.0));
        setOffset(offset, how);
    }
    void setOffset(double offset, const char* absoluteHow) {
        const PositionSpec& p = spec_->position;
        // a logged number to its tenth, never "-0.0" (a sign that rounds away is no sign)
        const auto tenth = [](double x) { return std::round(std::clamp(x, -99999.0, 99999.0) * 10.0) / 10.0 + 0.0; };
        const double was = offsetDeg_;
        const bool again = homed_;
        offsetDeg_ = offset;
        homed_ = true;
        char line[200];
        hal::LogLevel level = hal::LogLevel::Info;
        if (again) {
            // the zero's move, in output degrees, against the tolerance at the home position (both
            // sides: at a table's end one side converts to 0)
            const double shift = was - offset;
            const double homeDeg = positionToAngleDeg(p, p.homePosition);
            const double tolDeg = std::max(std::abs(positionToAngleDeg(p, p.homePosition + p.tolerance) - homeDeg),
                                           std::abs(positionToAngleDeg(p, p.homePosition - p.tolerance) - homeDeg));
            level = std::abs(shift) > tolDeg ? hal::LogLevel::Warn : hal::LogLevel::Info;
            std::snprintf(line, sizeof line, "'%.23s': homed again -- its zero moved %+.1f deg%s", spec_->name,
                          tenth(shift),
                          level == hal::LogLevel::Warn ? " (past its tolerance: was something in the way?)" : "");
        } else if (absoluteHow != nullptr) {
            std::snprintf(line, sizeof line, "'%.23s': homed -- %.60s, so it is at %.2f %s", spec_->name,
                          absoluteHow, std::clamp(position(), -99999.0, 99999.0),
                          p.units == Units::Inches ? "in" : "deg");
        } else {
            std::snprintf(line, sizeof line, "'%.23s': homed -- %+.1f deg from where its %s read zero at boot",
                          spec_->name, tenth(-offset),
                          devices_.angleSensor != nullptr ? "angle sensor" : "encoders");
        }
        host_->telemetry->log(level, "MECH", line);
    }
    [[nodiscard]] const char* notHomedNote() const noexcept {
        switch (spec_->position.home) {
            case HomeMethod::None:
                return "it is declared neverHomed: a move needs a zero";
            case HomeMethod::Switch:
                return "it has not found its zero yet: home it first (homeBySwitch)";
            case HomeMethod::Stall:
                return "it has not found its zero yet: home it first (homeByStall)";
            default:
                return "it has not found its zero";
        }
    }

    static constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    control::Pid movePid_;  ///< the MOVE loop: P + D only — an integral would wind up behind the profile
    control::Pid holdPid_;  ///< the HOLD loop: P + I + D, started FRESH when the profile ends
    std::optional<control::TrapezoidProfile> profile_;
    std::optional<manipulation::StallDetector> stall_;      ///< the stated signature (N10)
    std::optional<manipulation::StallDetector> homeStall_;  ///< homeByStall's own
    Mode mode_ = Mode::Idle;
    bool homed_ = false;
    bool holdLoop_ = false;     ///< the hold loop is the one running
    double limitDir_ = 0.0;     ///< the soft stop holds this push at the top (+1) or bottom (-1) limit; 0 not
    double approachEnd_ = 0.0;  ///< when the soft stop's move-loop approach gives way to the hold (clock s)
    double minDeg_ = 0.0;       ///< the travel limits as output degrees
    double maxDeg_ = 0.0;
    double accelDeg_ = 1.0;     ///< the smooth move's acceleration, output deg/s^2
    double offsetDeg_ = 0.0;    ///< joint = raw + offset
    double fromDeg_ = 0.0;      ///< the move's start, output degrees
    double toDeg_ = 0.0;        ///< the move's or hold's target, output degrees
    double targetPos_ = 0.0;    ///< the same in its units
    double jogVolts_ = 0.0;
    double inTolSince_ = -1.0;  ///< when it last entered the tolerance (< 0: it is outside)
    double homeStoppedAmps_ = -1.0;  ///< the most current the homing push drew when stopped, A (< 0: never stopped)
    int softStops_ = 0;
    bool homePushed_ = false;        ///< the homing push has been commanded (its samples are the push's)
};

/// SPEED (a Flywheel): spin at an RPM of its output and say when it is "at speed" (file header). The
/// step SUCCEEDS at speed; the wheel keeps spinning until stopped.
class SpeedSlot final : public Slot {
public:
    /// Built by the service's bind(); `d.motors` is the actuator (nullptr when ABSENT).
    SpeedSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i,
              const Devices& d)
        : Slot{host, spec, wiring, i, d, d.motors} {
        if (spec.speed.stall.stated()) {
            stall_.emplace(manipulation::StallConfig{units::Current{spec.speed.stall.amps},
                                                     units::AngularVelocity{spec.speed.stall.speedRadS},
                                                     units::Time{spec.speed.stall.time}});
        }
    }
    /// STEP1-B (the STEP1 brief's D8 (a)): spinUp()'s preconditions, in spinUp()'s order with its
    /// messages — every one of them, and nothing else (checkRun()'s rule).
    void checkSpinUp(double rpmGoal, units::Time bound, bool computed = true) const {
        const SpeedSpec& q = spec_->speed;
        SHULIB_PRECONDITION(std::isfinite(rpmGoal), "SpeedSlot::spinUp: the speed must be a finite number");
        SHULIB_PRECONDITION(std::isfinite(bound.value()) && bound.value() > 0.0,
                            "r.spinUp(...): the time limit must be finite and above 0");
        SHULIB_PRECONDITION(computed || std::abs(rpmGoal) <= q.maxRpm,
                            "SpeedSlot::spinUp: a CONSTANT speed past the free speed (the build refuses these; a "
                            "computed one is clamped -- clampedTo(x))");
    }
    /// Spin up to `rpm` (output rpm) within `bound`: SUCCEEDED once "at speed" (within the band for its
    /// time), and it KEEPS spinning; out of time TIMED_OUT, and it still spins (an unreachable speed is
    /// not a fault; a jam trips the stall signature). A COMPUTED speed past the free speed is CLAMPED
    /// to it and logged once. PRECONDITIONS (checked before anything changes): `rpm` finite; `bound`
    /// finite and > 0; a speed that is not `computed` within the free speed.
    void spinUp(double rpmTarget, units::Time bound, bool computed = true) {
        const SpeedSpec& q = spec_->speed;
        double rpmGoal = rpmTarget;
        checkSpinUp(rpmGoal, bound, computed);  // STEP1-B: the preconditions, shared with a partway trigger
        const double t = now();
        if (!beginStep(bound.value(), t)) {
            return;
        }
        if (std::abs(rpmGoal) > q.maxRpm) {
            rpmGoal = std::clamp(rpmGoal, -q.maxRpm, q.maxRpm);
            char line[160];
            std::snprintf(line, sizeof line, "'%.23s': speed %.0f rpm clamped to its free speed %.0f rpm",
                          spec_->name, std::clamp(rpmTarget, -999999.0, 999999.0), rpmGoal);
            host_->telemetry->log(hal::LogLevel::Warn, "MECH", line);
        }
        targetRadS_ = rpmGoal * 2.0 * kPi / 60.0;
        if (stall_) {
            stall_->reset();
        }
        spinning_ = true;
        active_ = true;
    }
    /// Spin up to preset `i`. PRECONDITION: `i` is one of its presets.
    void spinUpPreset(std::uint8_t i, units::Time bound) {
        SHULIB_PRECONDITION(i < spec_->speed.presetCount, "SpeedSlot::spinUpPreset: no such preset");
        spinUp(spec_->speed.presets[i].value, bound, false);
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
    /// Commanded to spin (a toggle's state): true from spinUp() until a stop, a stall or a cancel.
    [[nodiscard]] bool spinning() const noexcept { return spinning_ && active_; }
    /// The output speed now, rpm: the angle sensor's when one is wired, else the motors' through the
    /// gearing. 0 when ABSENT.
    [[nodiscard]] double rpm() const {
        if (devices_.motors == nullptr) {
            return 0.0;
        }
        const double radS = devices_.angleSensor != nullptr
                                ? (spec_->angleSensor.reversed ? -1.0 : 1.0) * devices_.angleSensor->velocity().value()
                                : devices_.motors->meanVelocity().value() / spec_->speed.gearRatio;
        return radS * 60.0 / (2.0 * kPi);
    }
    /// The speed it aims at, rpm (0 when it is not spinning).
    [[nodiscard]] double targetRpm() const noexcept { return spinning() ? targetRadS_ * 60.0 / (2.0 * kPi) : 0.0; }
    /// Within the band for its time, now.
    [[nodiscard]] bool atSpeed() const noexcept { return spinning() && atSpeed_; }

private:
    void tickBehaviour(double t) override {
        const SpeedSpec& q = spec_->speed;
        // THE STALL WATCH (N10), before "at speed" and after it: a JAMMED wheel never sits at stall
        // current (the brief's case H: 30 s at 12 V / 2.5 A before the signature was required).
        if (stall_ && stall_->update(units::Time{t}, devices_.motors->maxCurrent(), devices_.motors->meanVelocity())) {
            if (running()) {
                stopBehaviour(true);
                finishStep(t, MechanismOutcome::Stalled, DoneBy::Stalled, false, "a stated stall signature tripped");
            } else {
                stalledAfterStep(t, "a stated stall signature tripped while spinning");
            }
            return;
        }
        const double e = targetRadS_ - rpm() * 2.0 * kPi / 60.0;
        const double dt = lastT_ < 0.0 ? 0.0 : t - lastT_;
        lastT_ = t;
        // The integral acts only NEAR the target (header); frozen, not cleared, outside the window.
        if (q.kI > 0.0 && std::abs(e) < kSpeedIntegralWindow * std::abs(targetRadS_)) {
            const double cap = kSpeedIntegralLimitVolts / q.kI;
            integral_ = std::clamp(integral_ + e * dt, -cap, cap);
        }
        const double ff = (targetRadS_ > 0.0 ? q.kS : targetRadS_ < 0.0 ? -q.kS : 0.0) + q.kV * targetRadS_;
        devices_.motors->setVoltage(units::Voltage{std::clamp(ff + q.kP * e + q.kI * integral_, -12.0, 12.0)});
        if (std::abs(e) * 60.0 / (2.0 * kPi) <= q.atSpeedBandRpm) {
            if (inBandSince_ < 0.0) {
                inBandSince_ = t;
            }
        } else {
            inBandSince_ = -1.0;
        }
        atSpeed_ = inBandSince_ >= 0.0 && t - inBandSince_ >= q.atSpeedTime;
        if (!running()) {
            return;
        }
        if (atSpeed_) {
            finishStep(t, MechanismOutcome::Succeeded, DoneBy::Reached);  // and it keeps spinning
        } else if (t - stepStart_ >= bound_) {
            finishStep(t, MechanismOutcome::TimedOut, DoneBy::None, false, "it keeps spinning");
        }
    }
    void resetBehaviour() noexcept override {
        spinning_ = false;
        atSpeed_ = false;
        targetRadS_ = 0.0;
        integral_ = 0.0;
        inBandSince_ = -1.0;
        lastT_ = -1.0;
    }
    /// LIVE TUNING (TUNE1-A; Slot::rederiveSlot()): the stall detector matched to the spec's signature,
    /// and the speed integral BUMPLESS across a kI change — Pid::setGains' rule: the TERM kI·∫e survives
    /// (the accumulator rescaled by kI_old / kI_new, then held inside the new clamp; cleared at a new kI
    /// of 0). kS, kV, kP and the band are read from the spec every tick.
    bool rederiveKind() override {
        const SpeedSpec& q = spec_->speed;
        if (q.kI != kIOfIntegral_) {
            integral_ = q.kI > 0.0 ? std::clamp(kIOfIntegral_ * integral_ / q.kI, -kSpeedIntegralLimitVolts / q.kI,
                                                kSpeedIntegralLimitVolts / q.kI)
                                   : 0.0;
            kIOfIntegral_ = q.kI;
        }
        return refreshDetector(stall_, q.stall, q.stall.stated());
    }

    static constexpr double kPi = 3.14159265358979323846;

    double kIOfIntegral_ = spec_->speed.kI;  ///< the kI `integral_` was accumulated under (TUNE1-A)
    std::optional<manipulation::StallDetector> stall_;  ///< the stated signature (N10)
    double targetRadS_ = 0.0;   ///< the target, output rad/s
    double integral_ = 0.0;     ///< the speed error's integral, rad
    double inBandSince_ = -1.0; ///< when it last entered the band (< 0: outside)
    double lastT_ = -1.0;       ///< the previous tick's time (< 0: none yet)
    bool spinning_ = false;
    bool atSpeed_ = false;
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
    /// mechanism that was not built is a slot that answers ABSENT. Nothing is commanded — but a STATED
    /// current cap is WRITTEN into its motors here (a limit, not a command), and every motor kind's
    /// limits are read (the header's CURRENT CAP section). The specs, the devices' objects, the clock,
    /// the latch and the sink must outlive the service.
    /// PRECONDITIONS, each checked before anything is stored: bound ONCE; one Devices per spec, at
    /// most 8; each spec valid (specRefusal) and its name unique; a built mechanism's Devices carries
    /// the actuator of its kind, with the declared number of motors or lines, and an ABSENT one none;
    /// a built position or speed mechanism carries its ANGLE sensor exactly when it declares one, and
    /// a built position mechanism its HOME switch exactly when its homing is by switch (a refused angle
    /// sensor or home switch makes the mechanism ABSENT — robot::MechanismParts); a power or pneumatic
    /// one neither; a piece sensor only as declared, of the declared kind; a current limit only for a
    /// built motor mechanism's own motors (MECH-S-A). Homing that needs no motion happens here
    /// (PositionSlot): each such home is logged.
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
    /// through the whole auton. The auton runner (competition::runAuton's mechanism-service overload,
    /// MECH-A4) calls this before the
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
    /// The POSITION slot at `i` (an Arm or a Lift). PRECONDITION: `i` is a position mechanism.
    [[nodiscard]] PositionSlot& position(std::size_t i) {
        SHULIB_PRECONDITION(i < host_.count && positions_[i].has_value(),
                            "MechanismService::position: the mechanism at this index is not a position mechanism");
        return *positions_[i];
    }
    /// The SPEED slot at `i` (a Flywheel). PRECONDITION: `i` is a speed mechanism.
    [[nodiscard]] SpeedSlot& speed(std::size_t i) {
        SHULIB_PRECONDITION(i < host_.count && speeds_[i].has_value(),
                            "MechanismService::speed: the mechanism at this index is not a speed mechanism");
        return *speeds_[i];
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
    /// An ARM declaration's slot (find(const Power&)'s rule) — a position mechanism in DEGREES (a
    /// same-named LIFT, in inches, is not an arm: nullptr).
    [[nodiscard]] PositionSlot* find(const Arm& d) noexcept { return findPosition(d.name.text, Units::Degrees); }
    /// A LIFT declaration's slot — a position mechanism in INCHES.
    [[nodiscard]] PositionSlot* find(const Lift& d) noexcept { return findPosition(d.name.text, Units::Inches); }
    /// A FLYWHEEL declaration's slot.
    [[nodiscard]] SpeedSlot* find(const mechanism::Flywheel& d) noexcept { return findIn(speeds_, d.name.text); }
    /// A pneumatic declaration's slot (find(const Power&)'s rule).
    [[nodiscard]] PneumaticSlot* find(const mechanism::Pneumatic& d) noexcept {
        return findIn(pneumatics_, d.name.text);
    }
    /// A position GOAL's slot (`arm["score"]`, `lift.to(18_in)`): this robot's position mechanism of
    /// that name and of the goal's UNITS (an Arm's goal never finds a same-named lift) — and, for a
    /// named preset, only when it has that preset at that value (a goal of another robot's same-named
    /// mechanism must not move this one to a stranger's value).
    [[nodiscard]] PositionSlot* find(const PositionGoal& g) noexcept {
        PositionSlot* p = findPosition(g.mechanismName, g.units);
        if (p == nullptr || (g.presetName != nullptr && !hasPreset(p->spec().position.presets,
                                                                    p->spec().position.presetCount, g.presetName,
                                                                    g.value))) {
            return nullptr;
        }
        return p;
    }
    /// A speed GOAL's slot (`flywheel["shoot"]`): this robot's flywheel of that name — and, for a named
    /// preset, only when it has that preset at that speed (find(const PositionGoal&)'s rule).
    [[nodiscard]] SpeedSlot* find(const SpeedGoal& g) noexcept {
        SpeedSlot* p = findIn(speeds_, g.mechanismName);
        if (p == nullptr || (g.presetName != nullptr &&
                             !hasPreset(p->spec().speed.presets, p->spec().speed.presetCount, g.presetName, g.rpm))) {
            return nullptr;
        }
        return p;
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

    // ── LIVE TUNING (chunk TUNE1-A, 2026-11-05; the MECH-S/TUNE1 brief's §1.2) ─────────────────────
    /// RE-READ mechanism `index`'s spec after live tuning changed it IN PLACE (mechanism/live_tuning.hpp
    /// owns the table this service is bound over, so the slot's spec already IS the new one; what it
    /// copied at bind is stale until this runs — the brief's landmine 10). The slot re-derives every
    /// copy (Slot::rederiveSlot(): the PIDs' gains bumpless, the stall detectors, the limits in degrees,
    /// the soft stop's acceleration, the cap re-written when it changed) and ONE Info line says it:
    ///     'lift': retuned live -- kP 0.25 -> 0.3125
    /// (`what`: the change in words, printed as given — live tuning gives each number in its shortest
    /// form; nullptr says "its numbers re-read"). REFUSED — false, nothing
    /// re-derived, nothing said — while a tick runs (from a predicate, a sink, anything a tick calls),
    /// while that slot's STEP runs (a change is applied only BETWEEN steps: the brief's D8), and — a
    /// position mechanism — when the spec's travel limits leave out the target it HOLDS, change while a
    /// held button moves it, or (holding nothing) leave WHERE IT IS further outside them
    /// (PositionSlot::takesRange(), read ONCE here, before anything is re-derived: a hold is never moved —
    /// the brief's re-check #7). A hold, a spin, a held button go on through it and pick the new numbers
    /// up at the next tick. Live tuning checks the same conditions BEFORE it writes the spec (for its
    /// sentence) and puts the old one back if this still refuses — a reading that moved between the two
    /// can make it refuse here. A caller that wrote the spec itself must put the old one back on false:
    /// the slot runs the table it was bound over. PRECONDITIONS: bound; `index` below size(); the slot's
    /// spec passes the build's check (specRefusal) — live tuning's own rules are stricter still
    /// (mechanism::tuningRefusal).
    [[nodiscard]] bool rederive(std::size_t index, const char* what = nullptr) {
        SHULIB_PRECONDITION(bound(), "MechanismService::rederive: not bound");
        SHULIB_PRECONDITION(index < host_.count, "MechanismService::rederive: no mechanism at this index");
        if (!canRederive(index)) {
            return false;
        }
        Slot& s = *host_.slots[index];
        SHULIB_PRECONDITION(specRefusal(s.spec()) == Refusal::None, specProblem(s.spec()));
        if (!s.takesSpecNow()) {
            return false;  // the ONE read of where it is that counts (TUNE1-A's verification round)
        }
        s.rederiveSlot();
        char line[240];
        std::snprintf(line, sizeof line, "'%.23s': retuned live -- %.180s", s.name(),
                      what != nullptr ? what : "its numbers re-read");
        host_.telemetry->log(hal::LogLevel::Info, "MECH", line);
        return true;
    }
    /// Whether rederive(`index`) would run NOW: bound, `index` one of its mechanisms, no tick running,
    /// and no step running on that slot (TUNE1-A).
    [[nodiscard]] bool canRederive(std::size_t index) const noexcept {
        return bound() && index < host_.count && !inTick_ && !host_.slots[index]->running();
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
            for (std::size_t j = 0; j < i; ++j) {
                SHULIB_PRECONDITION(nameOf(specs[j]) != nameOf(s),
                                    "MechanismService::bind: two mechanisms share a name");
            }
            SHULIB_PRECONDITION(d.absence != nullptr,
                                "MechanismService::bind: a Devices' absence is null (\"\" = built)");
            const bool built = d.absence[0] == '\0';
            if (built && s.kind != Kind::Pneumatic) {
                SHULIB_PRECONDITION(d.motors != nullptr && d.pneumatic == nullptr,
                                    "MechanismService::bind: a built motor mechanism's Devices needs its motors, and "
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
            const bool loopKind = s.kind == Kind::Position || s.kind == Kind::Speed;
            SHULIB_PRECONDITION(d.angleSensor == nullptr ||
                                    (built && loopKind && s.angleSensor.kind == SensorKind::Rotation),
                                "MechanismService::bind: an angle sensor the declaration does not have (only a "
                                "built position or speed mechanism that declares one carries it)");
            SHULIB_PRECONDITION(!built || !loopKind || s.angleSensor.kind != SensorKind::Rotation ||
                                    d.angleSensor != nullptr,
                                "MechanismService::bind: a built mechanism that declares an angle sensor has none -- "
                                "its angle is how it knows where it is (a refused one makes it ABSENT)");
            SHULIB_PRECONDITION(d.homeSwitch == nullptr ||
                                    (built && s.kind == Kind::Position && s.homeSwitch.kind == SensorKind::Switch),
                                "MechanismService::bind: a home switch the declaration does not have (only a built "
                                "position mechanism homed by its switch carries one)");
            SHULIB_PRECONDITION(!built || s.kind != Kind::Position || s.homeSwitch.kind != SensorKind::Switch ||
                                    d.homeSwitch != nullptr,
                                "MechanismService::bind: a built mechanism homed by its switch has none (a refused "
                                "one makes it ABSENT)");
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
            SHULIB_PRECONDITION(d.homeSwitchFaults == nullptr || d.homeSwitch != nullptr,
                                "MechanismService::bind: a home switch's read-fault count on a mechanism with no "
                                "home switch");
            for (std::size_t m = 0; m < kDevicesMaxMotors; ++m) {
                SHULIB_PRECONDITION(d.currentLimits[m] == nullptr ||
                                        (built && s.kind != Kind::Pneumatic && m < s.motorCount),
                                    "MechanismService::bind: a current limit for a motor the mechanism does not have "
                                    "(only a built motor mechanism's motors carry one)");
            }
        }
        host_.clock = &clock;
        host_.faults = &faults;
        host_.telemetry = &telemetry;
        for (std::size_t i = 0; i < specs.size(); ++i) {
            const auto idx = static_cast<std::uint8_t>(i);
            const MechanismWiring* w = wiring != nullptr ? &wiring[i] : nullptr;
            switch (specs[i].kind) {
                case Kind::Power:
                    host_.slots[i] = &powers_[i].emplace(host_, specs[i], w, idx, devices[i]);
                    break;
                case Kind::Position:
                    host_.slots[i] = &positions_[i].emplace(host_, specs[i], w, idx, devices[i]);
                    break;
                case Kind::Speed:
                    host_.slots[i] = &speeds_[i].emplace(host_, specs[i], w, idx, devices[i]);
                    break;
                case Kind::Pneumatic:
                    host_.slots[i] = &pneumatics_[i].emplace(host_, specs[i], w, idx, devices[i]);
                    break;
            }
            if (host_.slots[i]->mech_ != nullptr) {
                built_[builtCount_++] = host_.slots[i]->mech_;
            }
            ++host_.count;
        }
    }
    PositionSlot* findPosition(const char* name, Units units) noexcept {
        PositionSlot* p = findIn(positions_, name);
        return p != nullptr && p->spec().position.units == units ? p : nullptr;
    }
    /// True when one of `presets` is called `name` AND holds `value` (a goal copies its declaration's
    /// value, so the same declaration's goal matches exactly).
    static bool hasPreset(const Preset (&presets)[kMaxPresets], std::uint8_t count, const char* name,
                          double value) noexcept {
        for (std::size_t i = 0; i < count && i < kMaxPresets; ++i) {
            if (std::string_view{presets[i].name} == std::string_view{name}) {
                return presets[i].value == value;
            }
        }
        return false;
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
            host_.slots[i]->recheckCurrentCap(t);  // MECH-S-A: once a second, active or not (header)
        }
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
    std::array<std::optional<PositionSlot>, kMaxMechanisms> positions_{};
    std::array<std::optional<SpeedSlot>, kMaxMechanisms> speeds_{};
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
