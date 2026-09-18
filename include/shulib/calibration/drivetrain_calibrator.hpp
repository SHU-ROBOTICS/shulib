#pragma once
//
// DrivetrainCalibrator — the robot describes its own drivetrain (chunk R3d Part 1, 2026-09-17).
//
// ── What it is for ──────────────────────────────────────────────────────────────────
// Almost every number a team is made to type about a drivetrain is discoverable on the robot.
// This state machine discovers them. It is handed every motor the brain found, an IMU and a
// clock, and it produces a hal::DrivetrainDescription: which ports drive, which way each one is
// wired, which side each belongs to, each side's inches per radian, and the effective track
// width. The three facts it CANNOT discover — the cartridge (physically unobservable: the rotor
// turns the same for every insert and the firmware already divides by whatever it was told), the
// drivetrain KIND, and which end is the FRONT — are stated by a person, and the front is stated
// PHYSICALLY, by which end they pushed first.
//
// ── PROVEN ON THE PLANT; NOT YET RUN ON A ROBOT ─────────────────────────────────────
// Every property below is validated against the A2 host plant (sim/drive_plant.hpp) and nothing
// else. No physical drivetrain has ever moved under this class. Where this header says the
// calibrator "measures" something it means "is designed to measure, and does measure on the
// plant". The numbers in CalibratorConfig are INVENTED for a first run — every one of them
// (A4 register HA-136) — and a bench session is what will say which must move.
//
// ── The phases ──────────────────────────────────────────────────────────────────────
//   WaitImu      unpowered   wait for imu.isReady(), bounded.
//   PushCapture  UNPOWERED   the operator pushes the whole robot along its length. Every
//                            candidate whose shaft moved by more than movedThreshold is a DRIVE
//                            port, and the DIRECTION it moved is its SIGN. Then the operator
//                            says which end went first; BACK-first inverts every sign. Ports
//                            that did not move are not drive motors and are never touched again.
//   SideAssign   powered     one member at a time, briefly, at a low ceiling. On a tank, driving
//                            ONE side forward pivots the robot about the other: LEFT forward
//                            turns it CLOCKWISE, which is NEGATIVE yaw in shulib's CCW-positive
//                            frame (hal/imu.hpp), and RIGHT forward is positive — reasoned from
//                            rigid-body kinematics, unmeasured on a robot (A4 register HA-134). The pulse also
//                            CONFIRMS the captured sign — the pulsed member's own encoder must
//                            advance POSITIVE under sign x V — and a contradiction refuses the
//                            run naming the port rather than being averaged away.
//   ScalePush    UNPOWERED   the operator pushes the robot a chosen distance along a tape. Each
//                            side's scale is that distance divided by the MEDIAN of its members'
//                            shaft travel. The push's straightness is checked against the IMU.
//   SpinTest     powered     spin in place both ways; W = (dR - dL) / dtheta_imu per direction,
//                            averaged. On a floor this measures the EFFECTIVE width, which is
//                            LARGER than a taped one because a turning tank scrubs (A4 register
//                            HA-135). The coupled-side monitor watches both sides throughout.
//   Done                     result() is the description; report() is every number behind it.
//
// ── The rules that make it correct, each with the mistake it prevents ───────────────
// 1. ONE PLACE FOR A SIGN, and it moves exactly once. During calibration NO sign is known, so
//    the candidates' adapters are constructed with POSITIVE ports and THIS class applies the
//    captured sign to a member's command and to its reading. The description it produces carries
//    the sign INSIDE the port number, and every later program constructs its adapter from that
//    signed port and never negates again. A calibrator that negated a member whose adapter was
//    ALSO built from a negative port would capture every sign backwards, consistently — invisible
//    on the host, a five-against-four fight on the robot. Hence the precondition: every candidate
//    port handed in must be POSITIVE.
// 2. NO WRAPPED ANGLE TOUCHES SHAFT ARITHMETIC. IMotor::position() is CUMULATIVE (hal/motor.hpp);
//    a math::Angle anywhere in a delta would wrap a two-turn spin to nothing. Shaft travel is
//    units::AngleDim and plain doubles throughout.
// 3. THE IMU'S dtheta IS ACCUMULATED PER TICK through Angle::errorTo, never taken as
//    end-minus-start: a single difference of two wrapped headings cannot see two full turns.
// 4. THE MEDIAN, NOT THE MEAN, for each side's scale — and it is hal::medianInPlace, the SAME
//    function hal::MotorGroup reports position() with. A member that travels short (robot two's
//    port 18, ~20 %, A4 register HA-130) therefore cancels EXACTLY between calibration and run
//    time rather than poisoning the scale or being accepted as the truth of the side.
// 5. THE PUSH IS POWERLESS. Polarity is found with nothing driven, because a pulse on a coupled
//    train whose signs are unknown IS the fight the coupled-side monitor exists to prevent. Only
//    members whose sign is already captured are ever pulsed, one at a time.
// 6. COAST, NEVER BRAKE, on every refusal and every abort. A calibration that stops with a person
//    standing next to the robot must leave the robot pushable.
// 7. A MISSED TICK IS A STOP. Every powered phase re-commands the same volts every tick, so a
//    caller that stops ticking would leave the motors running. If the clock has advanced by more
//    than commandTimeout since the last tick, the calibrator commands 0 V and refuses. (The C2
//    stalled-pacer lesson, applied here.)
//
// Tick-driven and NON-BLOCKING: the caller owns the loop and paces it (10 ms on the robot); the
// operator's answers arrive through answer(), never through a blocking read. Allocation-free
// (every array is sized by hal::kMaxSmartPorts). Raises NO faults — what to do about a refusal is
// the program's policy, exactly as the HAL adapters leave raising to the loop layer. PROS-free
// and sim-free.
//
// ── EVERY RUNTIME STRING IN THIS FILE IS ASCII (amended at R3d Part 2) ──────────────
// Prompts, progress lines and refusals are painted on the V5's screen by
// src/calibrate_program.cpp and read over USB serial. LVGL's built-in fonts carry ASCII and
// nothing else, so a UTF-8 em dash renders as a missing-glyph box on the brain and as
// mojibake in a raw `cat` of the user port — in the one sentence a person gets when the robot
// stops. src/ has used "--" for exactly this reason since R3a; Part 1 wrote these strings
// before they had a screen to land on, and Part 2 converted them (24 of them) when it built
// that screen. The COMMENTS in this file keep their typography: they are read in an editor.
// Nothing else changed with them.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::calibration {

/// Radians per degree — the thresholds below are stated in the degrees a person can picture on a
/// panel and converted once, here, rather than written as decimal radians nobody can check.
inline constexpr double kDegreesToRadians = math::Angle::kPi / 180.0;

/// The reproducibility the calibrator CLAIMS for a side's scale: two complete runs of the same
/// procedure on the same drivetrain must agree within this fraction. PROVISIONAL (A4: HA-136) —
/// pinned against the A2 plant under encoder and IMU hostility; a bench session decides whether a
/// real drivetrain holds it.
inline constexpr double kReproScaleTolerance = 0.02;

/// The reproducibility the calibrator CLAIMS for the effective track width, as a fraction. Looser
/// than the scale's because the spin measurement runs through the floor's scrub, which is the one
/// thing the host plant does not model. PROVISIONAL (A4: HA-136).
inline constexpr double kReproWidthTolerance = 0.03;

/// Where a run has got to. A run ends in exactly one of Done or Aborted.
enum class Phase : std::uint8_t {
    Idle,         ///< constructed, start() not yet called
    WaitImu,      ///< waiting for the IMU to finish its boot calibration
    PushCapture,  ///< UNPOWERED: watching a whole-robot push for drive ports and their signs
    SideAssign,   ///< powered: one brief pulse per drive port, naming its side from the yaw sign
    ScalePush,    ///< UNPOWERED: watching a push along a tape for each side's inches per radian
    SpinTest,     ///< powered: a spin each way for the effective track width
    Done,         ///< result() is valid
    Aborted,      ///< refused or aborted; reason() says why, every motor is at 0 V + Coast
};

/// What the operator's touch means. Everything a person contributes to a calibration arrives
/// through exactly this enum — there is no blocking prompt anywhere in the class.
enum class Answer : std::uint8_t {
    Done,        ///< "I have finished pushing"
    FrontFirst,  ///< the push led with the end the build team calls the FRONT
    BackFirst,   ///< the push led with the other end — every captured sign inverts
    Distance24,  ///< the tape push covered 24 in
    Distance48,  ///< ... 48 in (the default offer: two field tiles)
    Distance72,  ///< ... 72 in
    Abort,       ///< stop now
};

/// The three facts no measurement can supply, stated by a person before the run.
struct HumanFacts {
    /// Which drivetrain this is. Only Tank is supported: a holonomic drivetrain needs a strafe
    /// scale this procedure does not measure, and the calibrator refuses rather than guess.
    hal::DrivetrainKind kind = hal::DrivetrainKind::Tank;
    /// The cartridge in the drive motors — read off a motor, not inferred (see the header).
    hal::MotorCartridge cartridge = hal::MotorCartridge::Blue;
    /// The IMU's smart port, 1..hal::kMaxSmartPorts. Carried into the description so a later boot
    /// can check the device is still there.
    std::uint8_t imuPort = 0;
    /// The build stamp or session note to record as the description's provenance. Copied into the
    /// description's fixed buffer (truncated to fit) — the calibrator keeps no pointer to it past
    /// start().
    const char* provenance = "";
};

/// Every number the procedure uses. NOT ONE OF THEM IS A MEASUREMENT: all are INVENTED for a
/// first run and carried as fields so a bench session can move them without touching code.
/// PROVISIONAL, all of them (A4: HA-136).
struct CalibratorConfig {
    /// Shaft travel over the push above which a candidate counts as a DRIVE port. 5 degrees —
    /// the bench tester's MOTOR WATCH number, invented there and carried here unchanged so the
    /// two tools call the same push the same thing. PROVISIONAL (A4: HA-136).
    units::AngleDim movedThreshold{5.0 * kDegreesToRadians};
    /// |velocity| below which a motor counts as stopped, for the rest windows between pulses.
    /// PROVISIONAL (A4: HA-136).
    units::AngularVelocity restFloor{0.05};
    /// How long everything must stay below restFloor before a pulse may start — so one pulse's
    /// coast-down cannot be read as the next pulse's motion. PROVISIONAL (A4: HA-136).
    units::Time restSettleTime{0.5};
    /// How long to wait for rest before refusing. Without a bound this wait is the one unbounded
    /// loop in the procedure. PROVISIONAL (A4: HA-136).
    units::Time restTimeout{3.0};
    /// The ceiling for a single-member pulse — the bench tester's DRIVE station's first ceiling.
    /// PROVISIONAL (A4: HA-136).
    units::Voltage pulseVolts{3.0};
    /// How long one member is pulsed. PROVISIONAL (A4: HA-136).
    units::Time pulseTime{0.4};
    /// The |dtheta| a pulse must produce before its sign may be read as a side. Below it the robot
    /// is not on the floor (wheels up on blocks turn nothing) and the run refuses saying so.
    /// PROVISIONAL (A4: HA-136).
    units::AngleDim pulseYawFloor{2.0 * kDegreesToRadians};
    /// The |dtheta| a tape push may drift by before it is not a straight push. PROVISIONAL
    /// (A4: HA-136).
    units::AngleDim straightBound{3.0 * kDegreesToRadians};
    /// The least median shaft travel a tape push must produce per side — one full wheel turn.
    /// A shorter push divides the tape distance by a number too small to trust. PROVISIONAL
    /// (A4: HA-136).
    units::AngleDim minPushRad{2.0 * math::Angle::kPi};
    /// The ceiling for the in-place spin. PROVISIONAL (A4: HA-136).
    units::Voltage spinVolts{3.0};
    /// How many full turns each direction of the spin must cover. More turns average more of the
    /// floor and dilute a single scuff; two is the compromise with how long a person will stand
    /// and watch. PROVISIONAL (A4: HA-136).
    double spinTurns = 2.0;
    /// How long one direction of the spin may take before the run refuses. PROVISIONAL
    /// (A4: HA-136).
    units::Time spinTimeout{20.0};
    /// The largest relative disagreement tolerated between the two directions' track widths.
    /// PROVISIONAL (A4: HA-136).
    double spinAgreement = 0.05;
    /// How long to wait for the IMU's boot calibration. PROVISIONAL (A4: HA-136).
    units::Time imuReadyTimeout{5.0};
    /// How far the clock may advance between two tick() calls before a powered phase treats the
    /// gap as a stalled caller, commands 0 V and refuses. PROVISIONAL (A4: HA-136).
    units::Time commandTimeout{0.1};
    /// The coupled-side fight detector's thresholds for the spin. Defaults are the drive
    /// program's and hal::MotorGroup's — one evaluator, one set of numbers (A4: HA-132/133).
    teleop::SideMonitorConfig monitor{};
};

/// What one run concluded about ONE candidate — the panel's table and the log's record.
struct PortObservation {
    /// The candidate's port as handed in: always POSITIVE (the sign is captured, not given).
    std::int8_t port = 0;
    /// The sign captured from the push, already inverted if the push was BACK-first: +1, −1, or 0
    /// for a candidate that did not move and is therefore not a drive motor.
    std::int8_t sign = 0;
    /// −1 LEFT, +1 RIGHT, 0 = not a drive port or not yet pulsed.
    std::int8_t side = 0;
    /// Shaft travel over the SIGN-CAPTURE push, radians, as READ (before the sign is applied) —
    /// the number the sign was read off.
    double pushDeltaRad = 0.0;
    /// Shaft travel over the TAPE push, radians, sign-corrected. Kept separate from
    /// pushDeltaRad so a panel can show both pushes; 0 for a port that is not a drive motor.
    double tapeDeltaRad = 0.0;
    /// The IMU's accumulated dtheta over this port's own pulse, radians. 0 if it was not pulsed.
    double pulseYawRad = 0.0;
    /// This port's own sign-corrected shaft travel over its pulse, radians. Must be > 0.
    double pulseShaftRad = 0.0;
};

/// Every number behind a result — for the panel, for the log, and for a human deciding whether to
/// believe the run. Populated as far as the run got, so it is worth reading after a REFUSAL too.
struct CalibrationReport {
    /// One row per candidate, in the order they were handed in.
    PortObservation ports[hal::kMaxSmartPorts]{};
    /// How many rows of `ports` are real.
    std::uint8_t portCount = 0;
    /// How many of them moved on the push and are therefore drive motors.
    std::uint8_t drivePortCount = 0;
    /// True when the operator answered BACK FIRST and every captured sign was inverted.
    bool backFirst = false;
    /// The tape distance the operator chose, inches.
    double pushDistanceIn = 0.0;
    /// |dtheta| the IMU saw over the tape push, radians — how straight the push actually was.
    double pushStraightnessRad = 0.0;
    /// The LEFT side's median shaft travel over the tape push, radians (the scale's denominator).
    double leftMedianRad = 0.0;
    /// The RIGHT side's median shaft travel over the tape push, radians.
    double rightMedianRad = 0.0;
    /// The LEFT side's measured scale, inches per radian — available as soon as the tape push is
    /// answered, so a panel (and a host test) can read it without waiting for the spin.
    double leftInchesPerRadian = 0.0;
    /// The RIGHT side's measured scale, inches per radian.
    double rightInchesPerRadian = 0.0;
    /// max − min of the LEFT side's members' travel over the tape push, radians: how far apart a
    /// side's motors read. Robot two's port 18 is expected to show here (A4 register HA-130).
    double leftSpreadRad = 0.0;
    /// max − min of the RIGHT side's members' travel over the tape push, radians.
    double rightSpreadRad = 0.0;
    /// The effective track width from the CCW spin, inches.
    double widthCcwIn = 0.0;
    /// The effective track width from the CW spin, inches.
    double widthCwIn = 0.0;
    /// |Wccw − Wcw| / their mean — the agreement the run was judged on.
    double widthDisagreement = 0.0;
};

/// What a panel needs to paint this tick: where the run is, what the operator must DO, and a live
/// line of numbers.
struct CalibratorStatus {
    /// The phase as of the last tick(), answer() or abort().
    Phase phase = Phase::Idle;
    /// The instruction for the person, as a stable string literal — or the refusal, once aborted.
    const char* prompt = "not started";
    /// A live line of this phase's numbers, NUL-terminated and bounded.
    char progress[96]{};
};

/// The tick-driven state machine described in the header. Non-owning: the motors, the IMU, the
/// clock and the arrays the spans view must all outlive it. Owns no loop, allocates nothing,
/// blocks nowhere, raises no faults.
class DrivetrainCalibrator {
public:
    /// `candidates`: every motor the brain found, as adapters built with POSITIVE ports (rule 1 in
    /// the header). `ports`: their physical port numbers, parallel to `candidates`, each in
    /// 1..hal::kMaxSmartPorts and POSITIVE. Preconditions, all of them the caller's contract: the
    /// two spans are the same length; there are at least two candidates and at most
    /// hal::kMaxSmartPorts; no candidate is null; every port is positive, in range, and listed
    /// once.
    DrivetrainCalibrator(std::span<hal::IMotor* const> candidates,
                         std::span<const std::int8_t> ports, hal::IImu& imu, hal::IClock& clock,
                         const CalibratorConfig& config = {})
        : motors_{candidates}, ports_{ports}, imu_{imu}, clock_{clock}, cfg_{config} {
        SHULIB_PRECONDITION(candidates.size() == ports.size(),
                            "DrivetrainCalibrator: one port per candidate motor");
        SHULIB_PRECONDITION(candidates.size() >= 2,
                            "DrivetrainCalibrator: at least two candidate motors are needed");
        SHULIB_PRECONDITION(candidates.size() <= hal::kMaxSmartPorts,
                            "DrivetrainCalibrator: more candidates than the brain has ports");
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            SHULIB_PRECONDITION(candidates[i] != nullptr,
                                "DrivetrainCalibrator: a candidate motor is null");
            SHULIB_PRECONDITION(ports[i] > 0
                                    && ports[i] <= static_cast<std::int8_t>(hal::kMaxSmartPorts),
                                "DrivetrainCalibrator: every candidate port must be POSITIVE and "
                                "in 1..21 -- the calibrator applies the sign, so an adapter built "
                                "with a negative port would apply it twice");
            for (std::size_t j = i + 1; j < candidates.size(); ++j) {
                SHULIB_PRECONDITION(ports[j] != ports[i],
                                    "DrivetrainCalibrator: a port is listed twice");
            }
        }
        SHULIB_PRECONDITION(cfg_.movedThreshold.value() > 0.0
                                && cfg_.restFloor.value() > 0.0
                                && cfg_.restSettleTime.value() >= 0.0
                                && cfg_.restTimeout.value() > cfg_.restSettleTime.value()
                                && cfg_.pulseVolts.value() > 0.0 && cfg_.pulseTime.value() > 0.0
                                && cfg_.pulseYawFloor.value() > 0.0
                                && cfg_.straightBound.value() > 0.0
                                && cfg_.minPushRad.value() > 0.0 && cfg_.spinVolts.value() > 0.0
                                && cfg_.spinTurns > 0.0 && cfg_.spinTimeout.value() > 0.0
                                && cfg_.spinAgreement > 0.0 && cfg_.imuReadyTimeout.value() > 0.0
                                && cfg_.commandTimeout.value() > 0.0,
                            "DrivetrainCalibrator: every config threshold must be positive (and "
                            "restTimeout must exceed restSettleTime)");
        n_ = motors_.size();
        report_.portCount = static_cast<std::uint8_t>(n_);
        for (std::size_t i = 0; i < n_; ++i) {
            report_.ports[i].port = ports_[i];
        }
    }

    /// Begin a run from the three stated facts. Preconditions (caller contract, not run-time
    /// conditions): the calibrator is Idle, the kind is Tank, the cartridge is a real cartridge,
    /// and the IMU port is in range and is not one of the candidate ports. Leaves every candidate
    /// at 0 V + Coast so the robot can be pushed, and enters WaitImu.
    void start(const HumanFacts& facts) {
        SHULIB_PRECONDITION(phase_ == Phase::Idle,
                            "DrivetrainCalibrator::start: a run is already under way");
        SHULIB_PRECONDITION(facts.kind == hal::DrivetrainKind::Tank,
                            "DrivetrainCalibrator::start: only a tank drivetrain is supported "
                            "(a holonomic one needs a strafe scale this procedure never takes)");
        SHULIB_PRECONDITION(facts.cartridge == hal::MotorCartridge::Red
                                || facts.cartridge == hal::MotorCartridge::Green
                                || facts.cartridge == hal::MotorCartridge::Blue,
                            "DrivetrainCalibrator::start: the cartridge must be stated");
        SHULIB_PRECONDITION(facts.imuPort >= 1
                                && facts.imuPort <= static_cast<std::uint8_t>(hal::kMaxSmartPorts),
                            "DrivetrainCalibrator::start: the IMU port must be in 1..21");
        for (std::size_t i = 0; i < n_; ++i) {
            SHULIB_PRECONDITION(ports_[i] != static_cast<std::int8_t>(facts.imuPort),
                                "DrivetrainCalibrator::start: the IMU port is also a candidate "
                                "motor port");
        }
        facts_ = facts;
        safeState();
        enter(Phase::WaitImu, Step::None);
        updateStatus();
    }

    /// One control tick. Cheap and non-blocking; the caller paces it (10 ms on the robot). A no-op
    /// once the run has ended.
    void tick() {
        if (phase_ == Phase::Idle || phase_ == Phase::Done || phase_ == Phase::Aborted) {
            return;
        }
        const units::Time now = clock_.now();
        if (ticked_ && poweredPhase()
            && (now.value() - lastTick_.value()) > cfg_.commandTimeout.value()) {
            // Rule 7: a powered phase holds its command between ticks, so a caller that stops
            // ticking leaves the motors running. Stop them and say so.
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "a tick was missed: %.3f s since the last one, ceiling %.3f s -- "
                          "motors stopped",
                          now.value() - lastTick_.value(), cfg_.commandTimeout.value());
            refuse(msg);
            return;
        }
        lastTick_ = now;
        ticked_ = true;
        accumulateYaw();  // rule 3: per tick, through errorTo — never end minus start

        switch (phase_) {
            case Phase::WaitImu: tickWaitImu(now); break;
            case Phase::PushCapture: tickPushCapture(); break;
            case Phase::SideAssign: tickSideAssign(now); break;
            case Phase::ScalePush: tickScalePush(); break;
            case Phase::SpinTest: tickSpinTest(now); break;
            default: break;
        }
        updateStatus();
    }

    /// The operator's touch. Abort is accepted in any phase; every other answer is accepted only
    /// by the step that is waiting for it, and an answer that arrives out of order REFUSES the run
    /// naming both — a touch that silently did nothing would leave the operator believing a step
    /// had been taken. Precondition: a run is under way (anything but Idle/Done/Aborted), except
    /// for Abort, which is always allowed.
    void answer(Answer a) {
        if (a == Answer::Abort) {
            abort("aborted by the operator");
            return;
        }
        SHULIB_PRECONDITION(phase_ != Phase::Idle && phase_ != Phase::Done
                                && phase_ != Phase::Aborted,
                            "DrivetrainCalibrator::answer: no run is under way");
        switch (step_) {
            case Step::PushWatching:
                if (a == Answer::Done) {
                    finishPushCapture();
                    updateStatus();
                    return;
                }
                break;
            case Step::PushAwaitingFront:
                if (a == Answer::FrontFirst || a == Answer::BackFirst) {
                    applyFront(a == Answer::BackFirst);
                    updateStatus();
                    return;
                }
                break;
            case Step::ScaleWatching:
                if (a == Answer::Done) {
                    step_ = Step::ScaleAwaitingDistance;
                    updateStatus();
                    return;
                }
                break;
            case Step::ScaleAwaitingDistance:
                if (a == Answer::Distance24 || a == Answer::Distance48 || a == Answer::Distance72) {
                    finishScalePush(a == Answer::Distance24   ? 24.0
                                    : a == Answer::Distance48 ? 48.0
                                                              : 72.0);
                    updateStatus();
                    return;
                }
                break;
            default: break;
        }
        char msg[kReasonCap];
        std::snprintf(msg, sizeof msg,
                      "%s arrived while the run was %s -- nothing was recorded",
                      answerWord(a), waitingWord());
        refuse(msg);
    }

    /// Stop now: every candidate to 0 V + Coast (never Brake — rule 6), phase Aborted, `why`
    /// recorded as the reason. Allowed from ANY phase, including Done, where it discards a result
    /// the caller has not copied — call it only when abandoning the run is what is meant.
    void abort(const char* why = "aborted by the operator") {
        safeState();
        std::snprintf(reason_, sizeof reason_, "%s", why != nullptr ? why : "aborted");
        phase_ = Phase::Aborted;
        step_ = Step::None;
        updateStatus();
    }

    /// Where the run has got to.
    [[nodiscard]] Phase phase() const noexcept { return phase_; }

    /// What a panel paints this tick (see CalibratorStatus).
    [[nodiscard]] const CalibratorStatus& status() const noexcept { return status_; }

    /// Why the run was refused or aborted — empty while it is still going or if it finished. Every
    /// refusal names its cause, and the ones that can name a PORT do.
    [[nodiscard]] const char* reason() const noexcept { return reason_; }

    /// The measured description. Precondition: phase() == Done — a refused run has no result, and
    /// asking for one is a caller bug, not a value to be handed back.
    [[nodiscard]] const hal::DrivetrainDescription& result() const {
        SHULIB_PRECONDITION(phase_ == Phase::Done,
                            "DrivetrainCalibrator::result: the run did not finish (phase is not "
                            "Done) -- read reason() instead");
        return result_;
    }

    /// Every number behind the run, filled as far as the run got — worth reading after a refusal.
    [[nodiscard]] const CalibrationReport& report() const noexcept { return report_; }

    /// The config in force.
    [[nodiscard]] const CalibratorConfig& config() const noexcept { return cfg_; }

    /// True when answer(`a`) would be ACCEPTED right now — i.e. when a panel may OFFER it.
    /// Abort is always accepted; everything else only by the step that is waiting for it.
    ///
    /// ── WHY THIS EXISTS (added at R3d Part 2, for the calibrate program's button row) ──
    /// answer() REFUSES an out-of-order answer and ends the run, which is right (a touch that
    /// silently did nothing would leave the operator believing a step had been taken) — but it
    /// makes a wrong BUTTON destructive. A panel therefore has to know which answers are live,
    /// and Phase alone cannot tell it: PushCapture is two steps (watching, then awaiting
    /// FRONT/BACK) and so is ScalePush (watching, then awaiting the distance). Before this
    /// query the only public signal was the PROSE of status().prompt, and prose is not an API.
    /// A caller-side mirror of the step was the alternative — rejected: every consumer would
    /// re-derive it, and one that got it wrong would abort a two-minute calibration on a
    /// mis-tap.
    ///
    /// It is a PURE QUERY over the same step answer() switches on, and nothing about the state
    /// machine moved to add it. It duplicates that switch, so the anti-drift device is a test
    /// rather than a comment: drivetrain_calibrator_test.cpp case 21 drives the run to each
    /// waiting step and asserts, for EVERY Answer, that accepts() agrees with what answer()
    /// actually does on a fresh run.
    [[nodiscard]] bool accepts(Answer a) const noexcept {
        if (a == Answer::Abort) {
            return true;  // abort is accepted in every phase, including Idle and Done
        }
        switch (step_) {
            case Step::PushWatching: return a == Answer::Done;
            case Step::PushAwaitingFront:
                return a == Answer::FrontFirst || a == Answer::BackFirst;
            case Step::ScaleWatching: return a == Answer::Done;
            case Step::ScaleAwaitingDistance:
                return a == Answer::Distance24 || a == Answer::Distance48
                       || a == Answer::Distance72;
            default: return false;
        }
    }

private:
    /// The sub-step inside a phase. Private: a caller steers by Phase and Answer alone.
    enum class Step : std::uint8_t {
        None,
        PushWatching,
        PushAwaitingFront,
        SideRest,
        SidePulse,
        ScaleWatching,
        ScaleAwaitingDistance,
        SpinRestBefore,
        SpinCcw,
        SpinRestMid,
        SpinCw,
        SpinRestAfter,
    };

    // ── phase bodies ───────────────────────────────────────────────────────────────

    void tickWaitImu(units::Time now) {
        if (imu_.isReady()) {
            baselineAll();
            enter(Phase::PushCapture, Step::PushWatching);
            return;
        }
        if (now.value() - phaseStart_.value() > cfg_.imuReadyTimeout.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the IMU never became ready within %.1f s -- is it plugged into port "
                          "%d?",
                          cfg_.imuReadyTimeout.value(), static_cast<int>(facts_.imuPort));
            refuse(msg);
        }
    }

    void tickPushCapture() {
        for (std::size_t i = 0; i < n_; ++i) {
            report_.ports[i].pushDeltaRad = motors_[i]->position().value() - base_[i];
        }
    }

    void tickSideAssign(units::Time now) {
        if (step_ == Step::SideRest) {
            if (atRest()) {
                if (!resting_) {
                    resting_ = true;
                    restSince_ = now;
                }
                if (now.value() - restSince_.value() >= cfg_.restSettleTime.value()) {
                    beginPulse(now);
                }
                return;
            }
            resting_ = false;
            if (now.value() - stepStart_.value() > cfg_.restTimeout.value()) {
                const std::size_t m = firstMoving();
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              "port %d is still turning at %.3f rad/s after %.1f s of waiting "
                              "for rest before pulsing port %d -- a motor that reports motion "
                              "while nothing is driven is a frozen reading or a moving robot",
                              static_cast<int>(m < n_ ? ports_[m] : 0),
                              m < n_ ? motors_[m]->velocity().value() : 0.0,
                              cfg_.restTimeout.value(), static_cast<int>(ports_[pulseIndex_]));
                refuse(msg);
            }
            return;
        }
        // Step::SidePulse — powered, one member, re-commanded every tick (rule 7).
        commandOnly(pulseIndex_, static_cast<double>(sign_[pulseIndex_]) * cfg_.pulseVolts.value());
        for (std::size_t j = 0; j < n_; ++j) {
            // Only the NON-DRIVE candidates must be still: the pulsed member's side-mates are
            // BACK-DRIVEN through the gear train and are expected to turn. A motor that is not on
            // the drivetrain at all turning while one drive port is pulsed means the push capture
            // was wrong about it, or something else is being driven.
            if (j == pulseIndex_ || drive_[j]) {
                continue;
            }
            if (std::abs(motors_[j]->velocity().value()) > cfg_.restFloor.value()) {
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              "port %d moved (%.3f rad/s) while only port %d was pulsed -- it "
                              "did not move on the push, so it is not a drive motor and must "
                              "be still",
                              static_cast<int>(ports_[j]), motors_[j]->velocity().value(),
                              static_cast<int>(ports_[pulseIndex_]));
                refuse(msg);
                return;
            }
        }
        if (now.value() - stepStart_.value() >= cfg_.pulseTime.value()) {
            finishPulse();
        }
    }

    void tickScalePush() {
        for (std::size_t i = 0; i < n_; ++i) {
            if (drive_[i]) {
                report_.ports[i].tapeDeltaRad =
                    static_cast<double>(sign_[i]) * (motors_[i]->position().value() - base_[i]);
            }
        }
    }

    void tickSpinTest(units::Time now) {
        switch (step_) {
            case Step::SpinRestBefore:
            case Step::SpinRestMid:
            case Step::SpinRestAfter: {
                if (atRest()) {
                    if (!resting_) {
                        resting_ = true;
                        restSince_ = now;
                    }
                    if (now.value() - restSince_.value() >= cfg_.restSettleTime.value()) {
                        if (step_ == Step::SpinRestBefore) {
                            beginSpin(now, +1.0, Step::SpinCcw);
                        } else if (step_ == Step::SpinRestMid) {
                            beginSpin(now, -1.0, Step::SpinCw);
                        } else {
                            finishDone();
                        }
                    }
                    return;
                }
                resting_ = false;
                if (now.value() - stepStart_.value() > cfg_.restTimeout.value()) {
                    const std::size_t m = firstMoving();
                    char msg[kReasonCap];
                    std::snprintf(msg, sizeof msg,
                                  "port %d is still turning at %.3f rad/s after %.1f s of "
                                  "waiting for rest around the spin -- a motor that reports "
                                  "motion while nothing is driven is a frozen reading or a "
                                  "moving robot",
                                  static_cast<int>(m < n_ ? ports_[m] : 0),
                                  m < n_ ? motors_[m]->velocity().value() : 0.0,
                                  cfg_.restTimeout.value());
                    refuse(msg);
                }
                return;
            }
            default: break;
        }
        // Spinning: re-command every tick (rule 7), watch both sides for a fight, watch the clock.
        commandSpin();
        if (fightDuringSpin()) {
            return;
        }
        const double target = cfg_.spinTurns * 2.0 * math::Angle::kPi;
        if (spinDirection_ * yaw_ >= target) {
            finishSpin();
            return;
        }
        if (now.value() - stepStart_.value() > cfg_.spinTimeout.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the %s spin reached only %.2f of %.2f turns in %.1f s",
                          spinDirection_ > 0.0 ? "CCW" : "CW",
                          std::abs(yaw_) / (2.0 * math::Angle::kPi), cfg_.spinTurns,
                          cfg_.spinTimeout.value());
            refuse(msg);
        }
    }

    // ── phase transitions ──────────────────────────────────────────────────────────

    /// The push is over: every candidate that moved by more than the threshold is a drive port and
    /// the DIRECTION it moved is its sign (MOTOR WATCH's rule, in the library).
    void finishPushCapture() {
        std::uint8_t moved = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            const double d = motors_[i]->position().value() - base_[i];
            report_.ports[i].pushDeltaRad = d;
            drive_[i] = std::abs(d) >= cfg_.movedThreshold.value();
            sign_[i] = drive_[i] ? static_cast<std::int8_t>(d > 0.0 ? 1 : -1) : std::int8_t{0};
            report_.ports[i].sign = sign_[i];
            if (drive_[i]) {
                ++moved;
            }
        }
        report_.drivePortCount = moved;
        if (moved < 2) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the push moved %d port(s); a drivetrain needs at least 2 -- push the "
                          "WHOLE robot, along its length",
                          static_cast<int>(moved));
            refuse(msg);
            return;
        }
        step_ = Step::PushAwaitingFront;
    }

    /// The front is whatever the operator says it is: a BACK-first push inverts every sign, and no
    /// later step could ever see that it should have (every member would agree with every other,
    /// all the wrong way round).
    void applyFront(bool backFirst) {
        report_.backFirst = backFirst;
        if (backFirst) {
            for (std::size_t i = 0; i < n_; ++i) {
                sign_[i] = static_cast<std::int8_t>(-sign_[i]);
                report_.ports[i].sign = sign_[i];
            }
        }
        pulseIndex_ = 0;
        if (!advanceToNextDrivePort()) {
            refuse("no drive port to pulse");  // unreachable: finishPushCapture demands >= 2
            return;
        }
        enter(Phase::SideAssign, Step::SideRest);
    }

    /// Baseline this member's shaft and the heading, then start driving it.
    void beginPulse(units::Time now) {
        pulseBase_ = motors_[pulseIndex_]->position().value();
        resetYaw();
        stepStart_ = now;
        step_ = Step::SidePulse;
    }

    /// The pulse is over: read the side off the yaw's sign, after confirming the member's own
    /// encoder agreed with the sign the push captured.
    void finishPulse() {
        const std::size_t i = pulseIndex_;
        commandOnly(i, 0.0);  // 0 V + Coast on every candidate
        const double shaft =
            static_cast<double>(sign_[i]) * (motors_[i]->position().value() - pulseBase_);
        report_.ports[i].pulseShaftRad = shaft;
        report_.ports[i].pulseYawRad = yaw_;
        if (shaft <= 0.0) {
            // The push said this member turns one way under a forward push; the pulse drove it
            // that way and it went the other. One of the two readings is wrong, and averaging them
            // away would bake a reversed motor into the description.
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "port %d: driven forward at its captured sign %+d, its OWN encoder "
                          "went %.3f rad -- the push and the pulse contradict each other",
                          static_cast<int>(ports_[i]), static_cast<int>(sign_[i]), shaft);
            refuse(msg);
            return;
        }
        if (std::abs(yaw_) < cfg_.pulseYawFloor.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "port %d: the robot turned only %.2f deg (floor %.2f deg) while it "
                          "was driven -- is the robot on the FLOOR, not up on blocks?",
                          static_cast<int>(ports_[i]), yaw_ / kDegreesToRadians,
                          cfg_.pulseYawFloor.value() / kDegreesToRadians);
            refuse(msg);
            return;
        }
        // Rule: LEFT alone drives the robot CLOCKWISE, which is NEGATIVE yaw in the CCW-positive
        // canonical frame; RIGHT alone is positive (A4 register HA-134 — reasoned, and settled by
        // the FIRST pulse on a real floor, not by anything the host plant can show).
        side_[i] = yaw_ < 0.0 ? std::int8_t{-1} : std::int8_t{+1};
        report_.ports[i].side = side_[i];
        ++pulseIndex_;
        if (advanceToNextDrivePort()) {
            enterStep(Step::SideRest);
            return;
        }
        finishSideAssign();
    }

    /// Every drive port has been pulsed: there must be exactly two non-empty sides.
    void finishSideAssign() {
        leftN_ = 0;
        rightN_ = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            if (!drive_[i]) {
                continue;
            }
            if (side_[i] < 0) {
                leftIdx_[leftN_++] = i;
            } else {
                rightIdx_[rightN_++] = i;
            }
        }
        if (leftN_ == 0 || rightN_ == 0) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "every pulse turned the robot the same way: %u port(s) on one side "
                          "and none on the other -- that is not a two-sided drivetrain",
                          static_cast<unsigned>(leftN_ + rightN_));
            refuse(msg);
            return;
        }
        baselineAll();
        enter(Phase::ScalePush, Step::ScaleWatching);
    }

    /// The tape push is over and the operator has named the distance: check it was straight, then
    /// divide the distance by each side's MEDIAN travel (rule 4).
    void finishScalePush(double distanceIn) {
        report_.pushDistanceIn = distanceIn;
        report_.pushStraightnessRad = yaw_;
        if (std::abs(yaw_) > cfg_.straightBound.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the tape push curved by %.2f deg (bound %.2f deg) -- push it again, "
                          "straight",
                          yaw_ / kDegreesToRadians,
                          cfg_.straightBound.value() / kDegreesToRadians);
            refuse(msg);
            return;
        }
        double median[2] = {0.0, 0.0};
        double spread[2] = {0.0, 0.0};
        const char* names[2] = {"LEFT", "RIGHT"};
        for (int s = 0; s < 2; ++s) {
            const std::size_t* idx = (s == 0) ? leftIdx_ : rightIdx_;
            const std::size_t count = (s == 0) ? leftN_ : rightN_;
            double v[hal::kMaxSmartPorts];
            double lo = 0.0;
            double hi = 0.0;
            for (std::size_t k = 0; k < count; ++k) {
                const std::size_t i = idx[k];
                v[k] = static_cast<double>(sign_[i]) * (motors_[i]->position().value() - base_[i]);
                lo = (k == 0 || v[k] < lo) ? v[k] : lo;
                hi = (k == 0 || v[k] > hi) ? v[k] : hi;
            }
            spread[s] = hi - lo;
            median[s] = hal::medianInPlace(v, count);  // THE median — hal::MotorGroup's own
            if (median[s] <= 0.0) {
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              "%s side's median travel over the push is %.3f rad -- the push "
                              "must roll BOTH sides forward",
                              names[s], median[s]);
                refuse(msg);
                return;
            }
            if (median[s] < cfg_.minPushRad.value()) {
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              "%s side turned only %.2f rad over the push; at least %.2f (one "
                              "wheel turn) is needed to measure a scale -- push it farther",
                              names[s], median[s], cfg_.minPushRad.value());
                refuse(msg);
                return;
            }
        }
        report_.leftMedianRad = median[0];
        report_.rightMedianRad = median[1];
        report_.leftSpreadRad = spread[0];
        report_.rightSpreadRad = spread[1];
        leftIpr_ = distanceIn / median[0];
        rightIpr_ = distanceIn / median[1];
        report_.leftInchesPerRadian = leftIpr_;
        report_.rightInchesPerRadian = rightIpr_;
        enter(Phase::SpinTest, Step::SpinRestBefore);
    }

    /// Baseline the sides and the heading, then start spinning `direction` (+1 = CCW).
    void beginSpin(units::Time now, double direction, Step which) {
        baselineAll();
        resetYaw();
        leftMonitor_.reset();
        rightMonitor_.reset();
        spinDirection_ = direction;
        stepStart_ = now;
        step_ = which;
    }

    /// One direction of the spin is over: W = (dR − dL) / dtheta, with each side's travel taken
    /// through its own median and its own scale. On a real floor the number this produces is the
    /// EFFECTIVE width, LARGER than a tape's, because a turning tank scrubs (A4 register HA-135);
    /// the A2 plant scrubs nothing, so the host recovers the true width exactly and says nothing
    /// about the size of that gap.
    void finishSpin() {
        safeState();  // 0 V + Coast: the robot coasts to a stop, pushable throughout
        const double dL = sideTravel(leftIdx_, leftN_, leftIpr_);
        const double dR = sideTravel(rightIdx_, rightN_, rightIpr_);
        const double w = (dR - dL) / yaw_;
        if (spinDirection_ > 0.0) {
            report_.widthCcwIn = w;
            enterStep(Step::SpinRestMid);
            return;
        }
        report_.widthCwIn = w;
        const double a = report_.widthCcwIn;
        const double b = report_.widthCwIn;
        if (!std::isfinite(a) || !std::isfinite(b) || a <= 0.0 || b <= 0.0) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the spin measured a track width of %.3f in one way and %.3f the "
                          "other -- a width must be positive",
                          a, b);
            refuse(msg);
            return;
        }
        const double mean = 0.5 * (a + b);
        report_.widthDisagreement = std::abs(a - b) / mean;
        if (report_.widthDisagreement > cfg_.spinAgreement) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the two spin directions disagree about the track width by %.1f %% "
                          "(%.3f in vs %.3f in; bound %.1f %%)",
                          100.0 * report_.widthDisagreement, a, b, 100.0 * cfg_.spinAgreement);
            refuse(msg);
            return;
        }
        width_ = mean;
        enterStep(Step::SpinRestAfter);
    }

    /// Assemble the description, check it against its OWN validity rules, and finish.
    void finishDone() {
        safeState();
        result_ = hal::DrivetrainDescription{};
        result_.kind = facts_.kind;
        result_.cartridge = facts_.cartridge;
        result_.imuPort = facts_.imuPort;
        result_.source = hal::DescriptionSource::RobotMeasured;
        fillSide(result_.left, leftIdx_, leftN_, leftIpr_);
        fillSide(result_.right, rightIdx_, rightN_, rightIpr_);
        result_.trackWidth = units::Length{width_};
        std::snprintf(result_.provenance, sizeof result_.provenance,
                      "%.40s | ROBOT-MEASURED: %u ports, tape %.0f in, W %.3f in",
                      facts_.provenance != nullptr ? facts_.provenance : "",
                      static_cast<unsigned>(report_.drivePortCount), report_.pushDistanceIn,
                      width_);
        char why[128];
        if (!result_.valid(why, sizeof why)) {
            // A last line of defence: whatever the procedure concluded, an invalid description
            // must never be handed out as a result.
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the measured description does not hold together: %s",
                          why);
            refuse(msg);
            return;
        }
        phase_ = Phase::Done;
        step_ = Step::None;
    }

    // ── helpers ────────────────────────────────────────────────────────────────────

    /// True in the two phases that command a voltage (rule 7's scope). The rest windows inside
    /// them are included on purpose: a caller that has stopped ticking is a stalled caller whether
    /// or not this particular tick would have driven something, and everything is already at 0 V,
    /// so refusing there costs nothing.
    [[nodiscard]] bool poweredPhase() const noexcept {
        return phase_ == Phase::SideAssign || phase_ == Phase::SpinTest;
    }

    /// The first candidate turning faster than restFloor, or n_ when everything is still — so a
    /// rest that never arrives can NAME the motor that is preventing it instead of leaving a
    /// person to guess which of twelve it is.
    [[nodiscard]] std::size_t firstMoving() const {
        for (std::size_t i = 0; i < n_; ++i) {
            if (std::abs(motors_[i]->velocity().value()) > cfg_.restFloor.value()) {
                return i;
            }
        }
        return n_;
    }

    [[nodiscard]] bool atRest() const {
        for (std::size_t i = 0; i < n_; ++i) {
            if (std::abs(motors_[i]->velocity().value()) > cfg_.restFloor.value()) {
                return false;
            }
        }
        return true;
    }

    void safeState() {
        for (std::size_t i = 0; i < n_; ++i) {
            motors_[i]->setVoltage(units::Voltage{0.0});
            motors_[i]->setBrakeMode(hal::BrakeMode::Coast);  // rule 6: never Brake
        }
    }

    /// Drive candidate `i` at `volts` and every other candidate at 0 V; Coast everywhere.
    void commandOnly(std::size_t i, double volts) {
        for (std::size_t j = 0; j < n_; ++j) {
            motors_[j]->setBrakeMode(hal::BrakeMode::Coast);
            motors_[j]->setVoltage(units::Voltage{j == i ? volts : 0.0});
        }
    }

    /// The spin command: the LEFT side backward and the RIGHT side forward turns the robot CCW
    /// (positive yaw), and the mirror image turns it CW. Each member gets its own captured sign.
    void commandSpin() {
        for (std::size_t j = 0; j < n_; ++j) {
            motors_[j]->setBrakeMode(hal::BrakeMode::Coast);
        }
        for (std::size_t k = 0; k < leftN_; ++k) {
            const std::size_t i = leftIdx_[k];
            motors_[i]->setVoltage(units::Voltage{static_cast<double>(sign_[i]) * -spinDirection_
                                                  * cfg_.spinVolts.value()});
        }
        for (std::size_t k = 0; k < rightN_; ++k) {
            const std::size_t i = rightIdx_[k];
            motors_[i]->setVoltage(units::Voltage{static_cast<double>(sign_[i]) * spinDirection_
                                                  * cfg_.spinVolts.value()});
        }
        for (std::size_t j = 0; j < n_; ++j) {
            if (!drive_[j]) {
                motors_[j]->setVoltage(units::Voltage{0.0});
            }
        }
    }

    /// Run the SHARED coupled-side monitor over both sides for this tick; a PERSISTED disagreement
    /// is a fight (a wrong sign, a stripped gear, a frozen encoder) and refuses the run naming the
    /// port. Returns true when it refused.
    [[nodiscard]] bool fightDuringSpin() {
        const double leftVolts = -spinDirection_ * cfg_.spinVolts.value();
        const double rightVolts = spinDirection_ * cfg_.spinVolts.value();
        for (int s = 0; s < 2; ++s) {
            const std::size_t* idx = (s == 0) ? leftIdx_ : rightIdx_;
            const std::size_t count = (s == 0) ? leftN_ : rightN_;
            teleop::MemberSample samples[hal::kMaxSmartPorts];
            for (std::size_t k = 0; k < count; ++k) {
                const std::size_t i = idx[k];
                // The sign is applied HERE, once (rule 1): the adapters carry positive ports.
                samples[k] = teleop::MemberSample{
                    static_cast<double>(sign_[i]) * motors_[i]->velocity().value(), true};
            }
            teleop::CoupledSideMonitor& monitor = (s == 0) ? leftMonitor_ : rightMonitor_;
            const teleop::SideVerdict v = monitor.update(
                s == 0 ? leftVolts : rightVolts,
                std::span<const teleop::MemberSample>{samples, count});
            if (v.persistedCount > 0) {
                for (std::size_t k = 0; k < count; ++k) {
                    if ((v.persistedMask & (std::uint32_t{1} << k)) != 0u) {
                        char msg[kReasonCap];
                        std::snprintf(msg, sizeof msg,
                                      "port %d fought its %s side-mates for the whole "
                                      "persistence window during the spin -- stop and check "
                                      "that motor",
                                      static_cast<int>(ports_[idx[k]]),
                                      s == 0 ? "LEFT" : "RIGHT");
                        refuse(msg);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /// One side's travel since the last baseline, inches: the MEDIAN of its members' sign-corrected
    /// shaft deltas times that side's scale.
    [[nodiscard]] double sideTravel(const std::size_t* idx, std::size_t count, double ipr) const {
        double v[hal::kMaxSmartPorts];
        for (std::size_t k = 0; k < count; ++k) {
            const std::size_t i = idx[k];
            v[k] = static_cast<double>(sign_[i]) * (motors_[i]->position().value() - base_[i]);
        }
        return hal::medianInPlace(v, count) * ipr;
    }

    void fillSide(hal::DriveSideDescription& side, const std::size_t* idx, std::size_t count,
                  double ipr) const {
        side.count = static_cast<std::uint8_t>(count);
        for (std::size_t k = 0; k < count; ++k) {
            const std::size_t i = idx[k];
            // The SIGN goes INSIDE the port number — the one place it lives from here on.
            side.signedPorts[k] = static_cast<std::int8_t>(sign_[i] * ports_[i]);
        }
        side.inchesPerRadian = units::Length{ipr};
    }

    [[nodiscard]] bool advanceToNextDrivePort() {
        while (pulseIndex_ < n_ && !drive_[pulseIndex_]) {
            ++pulseIndex_;
        }
        return pulseIndex_ < n_;
    }

    void baselineAll() {
        for (std::size_t i = 0; i < n_; ++i) {
            base_[i] = motors_[i]->position().value();
        }
        resetYaw();
    }

    void resetYaw() {
        yaw_ = 0.0;
        prevHeading_ = imu_.heading();
    }

    /// Rule 3: accumulate the shortest signed step every tick, so two full turns are two full
    /// turns and not a wrapped nothing.
    void accumulateYaw() {
        const math::Angle h = imu_.heading();
        yaw_ += prevHeading_.errorTo(h);
        prevHeading_ = h;
    }

    void enter(Phase p, Step s) {
        phase_ = p;
        phaseStart_ = clock_.now();
        enterStep(s);
        if (p != Phase::SideAssign && p != Phase::SpinTest) {
            safeState();  // an unpowered phase leaves the robot pushable
        }
    }

    void enterStep(Step s) {
        step_ = s;
        stepStart_ = clock_.now();
        resting_ = false;
        if (s == Step::SideRest || s == Step::SpinRestBefore || s == Step::SpinRestMid
            || s == Step::SpinRestAfter) {
            safeState();
        }
    }

    /// Refuse the run: record the cause, stop every motor, end in Aborted. Every refusal in this
    /// class goes through here, which is what makes "0 V + Coast on every refusal" a property of
    /// one line rather than of a dozen call sites.
    /// `text` is already formatted. Every call site builds its message with a LITERAL format
    /// string into a local buffer and hands the result here, rather than passing a format through
    /// — the on-robot build gate compiles src/ under -Wformat-nonliteral, and a funnel taking
    /// `const char* format` would switch OFF the compiler's argument checking for every refusal
    /// message in the class, which is exactly where a wrong specifier costs a person the one
    /// sentence they get when the robot stops.
    void refuse(const char* text) {
        std::snprintf(reason_, sizeof reason_, "%s", text);
        safeState();
        phase_ = Phase::Aborted;
        step_ = Step::None;
        updateStatus();
    }

    [[nodiscard]] static const char* answerWord(Answer a) noexcept {
        switch (a) {
            case Answer::Done: return "DONE";
            case Answer::FrontFirst: return "FRONT FIRST";
            case Answer::BackFirst: return "BACK FIRST";
            case Answer::Distance24: return "24 in";
            case Answer::Distance48: return "48 in";
            case Answer::Distance72: return "72 in";
            case Answer::Abort: return "ABORT";
        }
        return "?";
    }

    [[nodiscard]] const char* waitingWord() const noexcept {
        switch (step_) {
            case Step::PushWatching: return "watching the push (waiting for DONE)";
            case Step::PushAwaitingFront: return "waiting for FRONT FIRST or BACK FIRST";
            case Step::ScaleWatching: return "watching the tape push (waiting for DONE)";
            case Step::ScaleAwaitingDistance: return "waiting for the tape distance";
            default: return "driving the motors (no answer is expected)";
        }
    }

    void updateStatus() {
        status_.phase = phase_;
        switch (phase_) {
            case Phase::Idle:
                status_.prompt = "not started";
                status_.progress[0] = '\0';
                return;
            case Phase::WaitImu:
                status_.prompt = "WAIT -- the IMU is calibrating. Keep the robot still.";
                std::snprintf(status_.progress, sizeof status_.progress, "%.1f s of %.1f s",
                              clock_.now().value() - phaseStart_.value(),
                              cfg_.imuReadyTimeout.value());
                return;
            case Phase::PushCapture: {
                if (step_ == Step::PushAwaitingFront) {
                    status_.prompt = "WHICH END WENT FIRST -- the FRONT, or the BACK?";
                    std::snprintf(status_.progress, sizeof status_.progress,
                                  "%u drive port(s) captured",
                                  static_cast<unsigned>(report_.drivePortCount));
                    return;
                }
                status_.prompt = "PUSH THE WHOLE ROBOT along its length, then touch DONE.";
                unsigned movedNow = 0;
                for (std::size_t i = 0; i < n_; ++i) {
                    if (std::abs(report_.ports[i].pushDeltaRad) >= cfg_.movedThreshold.value()) {
                        ++movedNow;
                    }
                }
                std::snprintf(status_.progress, sizeof status_.progress,
                              "%u of %u motor(s) have moved", movedNow,
                              static_cast<unsigned>(n_));
                return;
            }
            case Phase::SideAssign:
                status_.prompt = "STAND CLEAR -- pulsing one motor at a time.";
                std::snprintf(status_.progress, sizeof status_.progress,
                              "port %d: %s, yaw %+.1f deg",
                              static_cast<int>(pulseIndex_ < n_ ? ports_[pulseIndex_] : 0),
                              step_ == Step::SideRest ? "settling" : "pulsing",
                              yaw_ / kDegreesToRadians);
                return;
            case Phase::ScalePush:
                status_.prompt = step_ == Step::ScaleAwaitingDistance
                                     ? "HOW FAR did you push it -- 24, 48 or 72 in?"
                                     : "PUSH THE ROBOT STRAIGHT along the tape, then touch DONE.";
                std::snprintf(status_.progress, sizeof status_.progress,
                              "drift %+.1f deg (bound %.1f deg)", yaw_ / kDegreesToRadians,
                              cfg_.straightBound.value() / kDegreesToRadians);
                return;
            case Phase::SpinTest:
                status_.prompt = "STAND CLEAR -- spinning in place, both ways.";
                std::snprintf(status_.progress, sizeof status_.progress, "%s %.2f of %.2f turns",
                              spinDirection_ > 0.0 ? "CCW" : "CW",
                              std::abs(yaw_) / (2.0 * math::Angle::kPi), cfg_.spinTurns);
                return;
            case Phase::Done:
                status_.prompt = "DONE -- the drivetrain described itself.";
                result_.describe(status_.progress, sizeof status_.progress);
                return;
            case Phase::Aborted:
                status_.prompt = "STOPPED -- every motor is at 0 V and coasting.";
                std::snprintf(status_.progress, sizeof status_.progress, "%.95s", reason_);
                return;
        }
    }

    // ── state ──────────────────────────────────────────────────────────────────────
    std::span<hal::IMotor* const> motors_;
    std::span<const std::int8_t> ports_;
    hal::IImu& imu_;
    hal::IClock& clock_;
    CalibratorConfig cfg_;
    HumanFacts facts_{};

    std::size_t n_ = 0;
    Phase phase_ = Phase::Idle;
    Step step_ = Step::None;
    CalibratorStatus status_{};
    /// How long a refusal may be. Wide enough that no reason is TRUNCATED — checked by the
    /// compiler, which under -Wformat-truncation knows every literal format in this class and
    /// every buffer they are written into. A refusal that lost its last clause would be the one
    /// sentence a person gets, cut off.
    static constexpr std::size_t kReasonCap = 256;
    char reason_[kReasonCap] = "";
    hal::DrivetrainDescription result_{};
    CalibrationReport report_{};

    double base_[hal::kMaxSmartPorts]{};
    std::int8_t sign_[hal::kMaxSmartPorts]{};
    std::int8_t side_[hal::kMaxSmartPorts]{};
    bool drive_[hal::kMaxSmartPorts]{};
    std::size_t leftIdx_[hal::kMaxSmartPorts]{};
    std::size_t rightIdx_[hal::kMaxSmartPorts]{};
    std::size_t leftN_ = 0;
    std::size_t rightN_ = 0;

    units::Time phaseStart_{};
    units::Time stepStart_{};
    units::Time restSince_{};
    units::Time lastTick_{};
    bool resting_ = false;
    bool ticked_ = false;

    math::Angle prevHeading_{};
    double yaw_ = 0.0;

    std::size_t pulseIndex_ = 0;
    double pulseBase_ = 0.0;
    double leftIpr_ = 0.0;
    double rightIpr_ = 0.0;
    double width_ = 0.0;
    double spinDirection_ = 1.0;
    teleop::CoupledSideMonitor leftMonitor_{cfg_.monitor};
    teleop::CoupledSideMonitor rightMonitor_{cfg_.monitor};
};

}  // namespace shulib::calibration
