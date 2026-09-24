// calibration::DrivetrainCalibrator against the A2 plant (chunk R3d Part 1, brief §5 tests 4–17;
// REWRITTEN at R3d Part 2F-1 for the one push, the ramped pulse, the spin guards and the numbers a
// panel needs). NOTHING HERE HAS RUN ON A ROBOT. Every "push" is the harness commanding the
// plant's primaries while the calibrator only observes; every "pulse" and "spin" is the calibrator
// commanding the plant through the coupled-train model.
//
// ── WHY THIS FILE WAS REWRITTEN (read before trusting a green run of it) ──────────────
// Until 2F-1 every case here ran the plant with NO static friction (kS = 0), on purpose, because
// under the plant's coupled-train model one member of five delivers a fifth of its pulse to the
// wheel. On robot two that same single 3 V pulse turned the robot 0.14-0.17 deg and the run
// stopped — and at the tree's usual kS = 1.2 this plant turns by EXACTLY 0.00 deg (case 22 keeps
// that as a regression). The plant also pinned every unpowered wheel, so a one-sided pulse could
// only ever PIVOT. So: the rig now runs at kS = 1.2, the plant can let an idle side COAST along
// (coast-follow, κ), a member can SLIP (MemberFault::Decoupled), and every case STATES its κ.
// THE PLANT'S NUMBERS ARE INVENTED (kS, κ, the slip's drag): these cases prove the LOGIC's
// branches against a plant that can fail the way the robot did — not the robot's physics.
//
// Each case names the bug it catches and the mutation that must turn it red:
//    4  push capture: ten drive ports found, the two REVERSED ones with sign −1, the two motors
//       that never moved excluded                          (threshold → 0; the BackFirst inversion)
//    4b the movedThreshold attacked from BOTH sides on a full 48 in push
//    5  the same push answered BACK FIRST inverts every sign, swaps the sides, keeps the scale
//    6  side assignment: the yaw's sign names the side (at 9 V, the level that turns this plant),
//       a REVERSED member advances POSITIVE on its own encoder     (swap the yaw-sign → side rule)
//    7  a member whose reading contradicts its command is REFUSED, naming the port
//    8  a motor that never moved on the push creeping during a pulse is refused
//    9  wheels up — the heading held still — refuses naming the port and the FLOOR; the yaw floor
//       attacked from both sides                                     (floor → 0)
//   10  the scale from the ONE push is the plant's own radius to 1e-9 — the same number the old
//       two-push procedure produced — and a port that dropped off the bus for part of the push
//       leaves the median untouched while showing in the spread       (median → mean)
//   11  a push that curves is refused at DONE, before anything is powered   (bound → infinity)
//   12  the spin: W both ways within 1e-6 of the true 12.5 in; Done; the result's own geometry
//       drives an odometry that tracks the truth through a figure-eight
//   13  turn scrub makes the EFFECTIVE width LARGER than the true one (T8); four hostile seeds
//       reproduce the scale and the width inside the stated tolerances
//   14  a fight during the spin aborts naming the member, every motor at 0 V + Coast
//   15  timeouts: an IMU that never readies; a spin that starts but never finishes
//   16  a missed tick while spinning stops the motors and refuses
//   17  abort() from every phase leaves the safe state, and result() is then a precondition breach
//   18  the spin-agreement bound from both sides — and its reason says "pct", never '%'
//   19  the caller contract, including 2F-1's new config preconditions
//   21  accepts() agrees with answer() at every waiting step; the retired distances never accepted
//   ── added at 2F-1 ──
//   22  TONIGHT: the old single 3 V pulse at kS 1.2 turns the plant exactly 0.00 deg and is
//       refused STILL naming port 11 — and on the Part 1 rig (kS 0) it would have passed
//   23  the ramp stops at the FIRST level that turns the robot: 3 V at kS 0, 6 V at kS 0.9, 9 V at
//       kS 1.2                                      (never stop early; skip the ramp)
//   24  a DECOUPLED (slipping) member is LEFT OUT with a warning naming it, and the run SAVES a
//       valid description without it               (drop the decoupled classification)
//   25  κ near 1 (robot two is all-omni): κ = 1 and κ = 0.9 at 9 V are the ROLLED refusal naming
//       the port; κ = 0.6 still calibrates at 9 V
//   26  hand motion during a rest window never leaks into a pulse (the yaw-rate rest); a robot
//       turned for longer than restTimeout refuses saying it is still rotating
//                                                   (drop the yaw-rate half of atRest)
//   27  a spin that turns the WRONG way refuses within 15 deg     (drop the wrong-way guard)
//   28  a spin that cannot start steps up after 2 s and refuses 2 s later; one that starts only at
//       the second level finishes there             (drop the start check)
//   29  pushReady() is the finish's own predicates: every "not ready" that the finish can refuse
//       IS refused by it, and at-rest is the one stricter clause   (drop a clause)
//   30  no '%' can reach the screen: the header's source carries no "%%" format, and every
//       refusal this file reaches is ASCII and '%'-free
//   31  the serial record: one line per pushed port, per pulse (every level) and per spin
//   32  RUN AGAIN: a calibrator rebuilt in place carries nothing over from a refused run
//   33  the status NUMBERS a panel reads, during the push, the pulses and the spin

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <optional>
#include <span>
#include <sstream>
#include <string>

#include "calibration_test_rig.hpp"
#include "shulib/calibration/drivetrain_calibrator.hpp"
#include "shulib/control/feedforward.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/hostile/encoder_hostility.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::calibration::Answer;
using shulib::calibration::CalibratorConfig;
using shulib::calibration::DrivetrainCalibrator;
using shulib::calibration::kDegreesToRadians;
using shulib::calibration::kReproScaleTolerance;
using shulib::calibration::kReproWidthTolerance;
using shulib::calibration::Phase;
using shulib::calibration::PulseVerdict;
using shulib::calibration::Refusal;
using shulib::calibration::refusalName;
using shulib::hal::BrakeMode;
using shulib::hal::DrivetrainDescription;
using shulib::hal::IMotor;
using shulib::hal::MotorCartridge;
using shulib::hal::MotorGroup;
using shulib::hal::fake::FakeMotor;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using shulib::sim::MemberFault;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;
using namespace calib_rig;

namespace {

/// What PROS does for a NEGATIVE port, on the host: ONE negation, in the adapter, exactly where a
/// signed port number says a sign lives. Used only AFTER a calibration, to build the object graph
/// a later program would build from the description — the calibrator itself is handed POSITIVE
/// adapters and applies the sign itself (the header's rule 1).
class SignedMotor final : public IMotor {
public:
    SignedMotor(IMotor& inner, int sign) : inner_{&inner}, sign_{static_cast<double>(sign)} {}
    void setVoltage(Voltage v) override { inner_->setVoltage(Voltage{sign_ * v.value()}); }
    [[nodiscard]] Voltage commandedVoltage() const override {
        return Voltage{sign_ * inner_->commandedVoltage().value()};
    }
    void setBrakeMode(BrakeMode m) override { inner_->setBrakeMode(m); }
    [[nodiscard]] BrakeMode brakeMode() const override { return inner_->brakeMode(); }
    [[nodiscard]] shulib::units::AngleDim position() const override {
        return shulib::units::AngleDim{sign_ * inner_->position().value()};
    }
    [[nodiscard]] AngularVelocity velocity() const override {
        return AngularVelocity{sign_ * inner_->velocity().value()};
    }
    [[nodiscard]] shulib::units::Current current() const override { return inner_->current(); }
    [[nodiscard]] double temperature() const override { return inner_->temperature(); }

private:
    IMotor* inner_;
    double sign_;
};

/// A refusal, asserted completely: the code, the port, the safe state, the text rules, and no
/// result to be had.
void requireRefusal(CalibRig& rig, const DrivetrainCalibrator& cal, Refusal code, int port) {
    CHECK(cal.phase() == Phase::Aborted);
    CHECK_MESSAGE(cal.refusal() == code, "refusal ", std::string{refusalName(cal.refusal())},
                  ", expected ", std::string{refusalName(code)}, ": ", std::string{cal.reason()});
    CHECK_MESSAGE(cal.refusalPort() == port, "refusalPort ", cal.refusalPort(), ", expected ",
                  port, ": ", std::string{cal.reason()});
    CHECK(safeStateHolds(rig));
    CHECK(std::strlen(cal.reason()) > 0);
    CHECK_MESSAGE(screenSafe(cal.reason()), std::string{cal.reason()});
    CHECK_THROWS_AS((void)cal.result(), PreconditionError);
}
void requireRefusal(CalibRig& rig, Refusal code, int port) {
    requireRefusal(rig, rig.cal, code, port);
}

/// A degradation model that holds the reported HEADING still (optionally only once armed).
/// Landmine 5 of the R3d brief, stated: this is the HARNESS freezing the IMU, not the plant
/// modelling a robot up on blocks — the plant has no notion of a wheel off the floor. What the
/// calibrator sees is exactly what wheels-up looks like from inside: motors turning, heading
/// unchanged.
struct HeldHeading final : shulib::sim::DegradationModel {
    bool armed = true;
    bool held = false;
    Angle value{};
    [[nodiscard]] Angle imuHeading(Angle truth, Time, shulib::sim::Rng&) override {
        if (!armed) {
            return truth;
        }
        if (!held) {
            value = truth;
            held = true;
        }
        return value;
    }
};

/// An IMU that never finishes calibrating.
struct NeverReady final : shulib::sim::DegradationModel {
    [[nodiscard]] bool imuReady(bool, Time) override { return false; }
};

/// TURN SCRUB, which the plant does not model and T8 is about: while the two wheels are driven in
/// OPPOSITE directions (a spin in place), only `retain` of each wheel's spin reaches the floor, so
/// the encoders over-count the turn and the measured width comes out LARGER than the true one. A
/// straight push scrubs nothing, so the distance scale is unaffected — which is precisely why the
/// effective width and a taped width are different numbers.
struct TurnScrub final : shulib::sim::DegradationModel {
    double retain = 0.95;
    double last[2] = {0.0, 0.0};
    [[nodiscard]] Velocity wheelMotionVelocity(int wheel, Velocity spin, Time,
                                               shulib::sim::Rng&) override {
        last[static_cast<std::size_t>(wheel)] = spin.value();
        const bool opposed = last[0] * last[1] < 0.0 && std::abs(last[0]) > 1.0
                             && std::abs(last[1]) > 1.0;
        return opposed ? Velocity{spin.value() * retain} : spin;
    }
};

/// Scrub that is switched on by the harness for the SECOND spin only — an asymmetry between the
/// two directions, which is what the agreement bound exists to catch.
struct SecondSpinScrub final : shulib::sim::DegradationModel {
    bool active = false;
    double retain = 0.93;
    [[nodiscard]] Velocity wheelMotionVelocity(int, Velocity spin, Time,
                                               shulib::sim::Rng&) override {
        return active ? Velocity{spin.value() * retain} : spin;
    }
};

/// A HAND turning the robot while every motor is still (a robot rotated in the air, or still
/// swinging after the push): the IMU reports `rate` on top of the truth while `now < until`, and
/// the heading carries what that rate added. The motors see nothing — which is exactly why a rest
/// check that reads only the motors cannot see it (the Part 2F brief, §0).
struct HandTurn final : shulib::sim::DegradationModel {
    double rate = 20.0 * kDegreesToRadians;
    double until = -1.0;
    double offset = 0.0;
    double last = 0.0;
    [[nodiscard]] Angle imuHeading(Angle truth, Time now, shulib::sim::Rng&) override {
        const double t = now.value();
        if (t <= until) {
            offset += rate * (t - last);
        }
        last = t;
        return truth + Angle::radians(offset);
    }
    [[nodiscard]] AngularVelocity imuYawRate(AngularVelocity truth, Time now,
                                             shulib::sim::Rng&) override {
        return AngularVelocity{truth.value() + (now.value() < until ? rate : 0.0)};
    }
};

/// Once armed, the IMU reports the robot's rotation MIRRORED about where it stood: the stand-in
/// for sides that are swapped consistently (the spin then turns the robot the way the calibrator
/// believes is backwards). Continuous at arming, so the guard sees a turn, not a jump.
struct MirrorOnceArmed final : shulib::sim::DegradationModel {
    bool armed = false;
    bool pivotSet = false;
    double pivot = 0.0;
    [[nodiscard]] Angle imuHeading(Angle truth, Time, shulib::sim::Rng&) override {
        if (!armed) {
            return truth;
        }
        if (!pivotSet) {
            pivot = truth.radians();
            pivotSet = true;
        }
        return Angle::radians(2.0 * pivot - truth.radians());
    }
    [[nodiscard]] AngularVelocity imuYawRate(AngularVelocity truth, Time,
                                             shulib::sim::Rng&) override {
        return armed ? AngularVelocity{-truth.value()} : truth;
    }
};

/// Once armed, a wheel commanded under `below` volts does not move: a robot whose spin cannot
/// break free at the first spin level but can at the second.
struct SpinStiction final : shulib::sim::DegradationModel {
    bool armed = false;
    double below = 4.0;
    [[nodiscard]] Voltage effectiveVoltage(int, Voltage commanded, Time,
                                           shulib::sim::Rng&) override {
        return armed && std::abs(commanded.value()) < below ? Voltage{0.0} : commanded;
    }
};

/// Build the two MotorGroups a later program would build from a finished description: one signed
/// adapter per port, then a group per side — the same shape src/main.cpp uses.
struct GraphFromDescription {
    std::array<SignedMotor, 10> signedMotors;
    std::array<IMotor*, 5> leftPtrs{};
    std::array<IMotor*, 5> rightPtrs{};
    MotorGroup left;
    MotorGroup right;

    GraphFromDescription(CalibRig& rig, const DrivetrainDescription& d)
        : signedMotors{make(rig, d, 0, 0),  make(rig, d, 0, 1), make(rig, d, 0, 2),
                       make(rig, d, 0, 3),  make(rig, d, 0, 4), make(rig, d, 1, 0),
                       make(rig, d, 1, 1),  make(rig, d, 1, 2), make(rig, d, 1, 3),
                       make(rig, d, 1, 4)},
          left{wire(0, leftPtrs)},
          right{wire(1, rightPtrs)} {}

private:
    static SignedMotor make(CalibRig& rig, const DrivetrainDescription& d, int wheel,
                            std::size_t k) {
        const std::size_t idx = CalibRig::indexOf(wheel, k);
        const int port = rig.ports[idx];
        const std::int8_t signed_ = d.signedPortOf(port);
        REQUIRE(signed_ != 0);
        return SignedMotor{rig.member(wheel, k), signed_ < 0 ? -1 : +1};
    }
    std::span<IMotor* const> wire(int wheel, std::array<IMotor*, 5>& ptrs) {
        for (std::size_t k = 0; k < 5; ++k) {
            ptrs[k] = &signedMotors[static_cast<std::size_t>(wheel) * 5 + k];
        }
        return std::span<IMotor* const>{ptrs.data(), ptrs.size()};
    }
};

[[nodiscard]] double posError(const Pose2d& a, const Pose2d& b) {
    return std::hypot((a.x() - b.x()).value(), (a.y() - b.y()).value());
}

/// Tick a fresh rig through WaitImu (nothing pushed), leaving it watching the push.
void reachPushWatching(CalibRig& rig) {
    rig.cal.start(facts());
    for (int t = 0; t < 100 && rig.cal.phase() != Phase::PushCapture; ++t) {
        rig.cal.tick();
        rig.h.plant().step(Time{kDt});
    }
    REQUIRE(rig.cal.phase() == Phase::PushCapture);
    REQUIRE(rig.cal.accepts(Answer::Done));
}

/// The pulse records of one port, in order.
[[nodiscard]] std::string levelsOf(const DrivetrainCalibrator& cal, int port) {
    std::string out;
    const auto& rep = cal.report();
    for (std::size_t k = 0; k < rep.pulseCount; ++k) {
        if (rep.pulses[k].port == port) {
            char buf[48];
            std::snprintf(buf, sizeof buf, "%.0fV:%s ", rep.pulses[k].levelVolts,
                          shulib::calibration::pulseVerdictWord(rep.pulses[k].verdict));
            out += buf;
        }
    }
    return out;
}

}  // namespace

// ═══ 4. Push capture ════════════════════════════════════════════════════════════════
// Bug caught: MOTOR WATCH's rule not surviving the move into the library — a motor that never
// moved counted as a drive port (the threshold), or a backwards-wired motor's sign lost.
TEST_CASE("DrivetrainCalibrator 4: the one push finds exactly the ten drive ports, signs the two "
          "REVERSED ones −1, and never touches the two motors that did not move") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.setFault(0, 0, MemberFault::Reversed);  // LEFT primary, port 11, wired backwards
    rig.setFault(1, 3, MemberFault::Reversed);  // RIGHT extra, port 19, wired backwards
    rig.cal.start(facts());
    const Phase end = runCalibration(rig, RunOptions{});
    REQUIRE(end == Phase::Done);

    const auto& rep = rig.cal.report();
    CHECK(rep.portCount == 12);
    CHECK(rep.drivePortCount == 10);
    CHECK_FALSE(rep.backFirst);
    CHECK(rep.pushDistanceIn == 48.0);
    for (int w = 0; w < 2; ++w) {
        for (std::size_t k = 0; k < CalibRig::kPerWheel; ++k) {
            const std::size_t i = CalibRig::indexOf(w, k);
            CAPTURE(rig.ports[i]);
            const bool reversed = (w == 0 && k == 0) || (w == 1 && k == 3);
            CHECK(rep.ports[i].sign == (reversed ? -1 : +1));
            CHECK(std::abs(rep.ports[i].pushDeltaRad) > 20.0);  // 48 in on a 1.625 in radius
            // The travel the scale is taken from is sign-corrected: positive, front-first.
            CHECK(rep.ports[i].pushTravelRad == rep.ports[i].sign * rep.ports[i].pushDeltaRad);
            CHECK(rep.ports[i].pushTravelRad > 0.0);
            CHECK_FALSE(rep.ports[i].excluded);
        }
    }
    // The two motors that are not on the drivetrain: no sign, no side, no travel, never pulsed —
    // and they were excluded by the PUSH, not by anybody's list.
    for (std::size_t j = 2 * CalibRig::kPerWheel; j < CalibRig::kCandidates; ++j) {
        CHECK(rep.ports[j].sign == 0);
        CHECK(rep.ports[j].side == 0);
        CHECK(rep.ports[j].pushDeltaRad == 0.0);
        CHECK(rep.ports[j].pushTravelRad == 0.0);
        CHECK(rep.ports[j].verdict == PulseVerdict::None);
        CHECK(rep.ports[j].pulseShaftRad == 0.0);
    }
    // The description carries the signs INSIDE the port numbers.
    const DrivetrainDescription& d = rig.cal.result();
    CHECK(d.signedPortOf(11) == -11);
    CHECK(d.signedPortOf(19) == -19);
    CHECK(d.signedPortOf(12) == 12);
    CHECK(d.signedPortOf(20) == 20);
    CHECK(d.sideOf(5) == 0);  // the idle motors are in no side
    CHECK(d.sideOf(6) == 0);
    CHECK(rig.cal.refusal() == Refusal::None);
    CHECK(std::strlen(rig.cal.warning()) == 0);
}

TEST_CASE("DrivetrainCalibrator 4b: movedThreshold from BOTH sides — the same full push is "
          "captured just under the line and refused just over it") {
    // The 48 in push turns each shaft by 48 / 1.625 = 29.54 rad. Bracket it.
    const double expected = 48.0 / kPlantRadius;
    for (int over = 0; over < 2; ++over) {
        CAPTURE(over);
        CalibratorConfig cfg;
        cfg.movedThreshold =
            shulib::units::AngleDim{over != 0 ? expected * 1.02 : expected * 0.98};
        CalibRig rig{cfg, calibPlant(0.0)};
        rig.cal.start(facts());
        RunOptions opt;
        // Over the line nothing ever counts as moved, so pushReady() never comes true: the
        // operator touches DONE anyway — an old screen's DONE — and the FINISH must refuse.
        opt.waitForReady = false;
        (void)runCalibration(rig, opt);
        if (over != 0) {
            requireRefusal(rig, Refusal::PushTooFew, 0);
            CHECK(reasonMentions(rig.cal, "the push moved 0 port(s)"));
            CHECK(rig.cal.report().pulseCount == 0);  // refused before anything was powered
        } else {
            CHECK(rig.cal.phase() == Phase::Done);
            CHECK(rig.cal.report().drivePortCount == 10);
        }
    }
}

// ═══ 5. The front is what the operator says ═════════════════════════════════════════
// Bug caught: assuming the push direction. A BACK-first push captures every sign backwards, and
// no later step could ever see it — every member would agree with every other, all the wrong way.
TEST_CASE("DrivetrainCalibrator 5: the same push answered BACK FIRST inverts every captured sign, "
          "swaps the sides, and gives the same scale") {
    CalibRig front{CalibratorConfig{}, calibPlant(0.0)};
    front.setFault(0, 0, MemberFault::Reversed);
    front.cal.start(facts());
    REQUIRE(runCalibration(front, RunOptions{}) == Phase::Done);

    CalibRig back{CalibratorConfig{}, calibPlant(0.0)};
    back.setFault(0, 0, MemberFault::Reversed);
    back.cal.start(facts());
    RunOptions opt;
    opt.front = Answer::BackFirst;
    const Phase end = runCalibration(back, opt);

    CHECK(back.cal.report().backFirst);
    for (std::size_t i = 0; i < 2 * CalibRig::kPerWheel; ++i) {
        CAPTURE(back.ports[i]);
        CHECK(back.cal.report().ports[i].sign == -front.cal.report().ports[i].sign);
        CHECK(back.cal.report().ports[i].sign != 0);
        // The robot went BACKWARD in its own frame: the sign-corrected travel is negative.
        CHECK(back.cal.report().ports[i].pushTravelRad < 0.0);
    }
    // A back-first push also swaps which side is which — the robot is the same, the FRONT is the
    // other end, so what was the left side is now the right. With every sign consistently
    // inverted the pulses do not contradict, and the description is a valid description OF A
    // ROBOT FACING THE OTHER WAY.
    REQUIRE(end == Phase::Done);
    CHECK(back.cal.result().signedPortOf(11) == 11);   // was −11 front-first
    CHECK(back.cal.result().signedPortOf(12) == -12);  // was +12
    CHECK(back.cal.result().sideOf(11) == -front.cal.result().sideOf(11));
    CHECK(back.cal.result().sideOf(11) == -back.cal.result().sideOf(20));
    // One push gives the scale either way round: the travel is the same shaft travel.
    CHECK(back.cal.report().leftInchesPerRadian == front.cal.report().rightInchesPerRadian);
    CHECK(back.cal.report().rightInchesPerRadian == front.cal.report().leftInchesPerRadian);
    CHECK(back.cal.result().trackWidth.value()
          == doctest::Approx(front.cal.result().trackWidth.value()).epsilon(1e-9));
}

// ═══ 6. Side assignment from the IMU's yaw sign ═════════════════════════════════════
// Bug caught: the yaw-sign → side rule backwards (every side flipped, and the robot would then
// turn the wrong way for every commanded heading), or a REVERSED member read as a contradiction.
TEST_CASE("DrivetrainCalibrator 6: LEFT pulses turn the robot CLOCKWISE (negative yaw) and RIGHT "
          "pulses counter-clockwise; the sides are the plant's wheels exactly") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.setFault(0, 0, MemberFault::Reversed);
    rig.setFault(1, 3, MemberFault::Reversed);
    rig.cal.start(facts());
    REQUIRE_MESSAGE(runCalibration(rig, RunOptions{}) == Phase::Done, std::string{rig.cal.reason()});

    const auto& rep = rig.cal.report();
    for (std::size_t k = 0; k < CalibRig::kPerWheel; ++k) {
        const std::size_t l = CalibRig::indexOf(0, k);
        const std::size_t r = CalibRig::indexOf(1, k);
        CAPTURE(k);
        CHECK(rep.ports[l].pulseYawRad < 0.0);   // LEFT forward → CW → NEGATIVE canonical yaw
        CHECK(rep.ports[r].pulseYawRad > 0.0);   // RIGHT forward → CCW → positive
        CHECK(rep.ports[l].side == -1);
        CHECK(rep.ports[r].side == +1);
        CHECK(rep.ports[l].verdict == PulseVerdict::Left);
        CHECK(rep.ports[r].verdict == PulseVerdict::Right);
        // At kS 1.2 one member of five needs the TOP level: 3 V and 6 V move nothing.
        CHECK(rep.ports[l].pulseLevelVolts == 9.0);
        CHECK(rep.ports[r].pulseLevelVolts == 9.0);
        // Every pulsed member's OWN encoder advanced POSITIVE under sign × V — including the two
        // wired backwards, which is the whole point of the cross-check.
        CHECK(rep.ports[l].pulseShaftRad > 0.0);
        CHECK(rep.ports[r].pulseShaftRad > 0.0);
    }
    // The sides ARE the plant's wheels: ports 11–15 left, 16–20 right, no leakage either way.
    const DrivetrainDescription& d = rig.cal.result();
    CHECK(d.left.count == 5);
    CHECK(d.right.count == 5);
    for (int p : {11, 12, 13, 14, 15}) {
        CHECK(d.sideOf(p) == -1);
    }
    for (int p : {16, 17, 18, 19, 20}) {
        CHECK(d.sideOf(p) == +1);
    }
}

// ═══ 7. A contradiction is refused, naming the port ═════════════════════════════════
// Bug caught: a member whose reading disagrees with its command being averaged into a side. A
// SignFlipped member reads backwards while its command still drives the train the right way, so
// the push captures −1 for it and the pulse then drives the train BACKWARD — the one case where
// the push and the pulse cannot both be believed.
TEST_CASE("DrivetrainCalibrator 7: a member whose encoder contradicts its own pulse refuses the "
          "run, naming the port") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.setFault(1, 2, MemberFault::SignFlipped);  // port 18 reads backwards, drives forwards
    rig.cal.start(facts());
    (void)runCalibration(rig, RunOptions{});
    requireRefusal(rig, Refusal::PulseContradiction, 18);
    CHECK(reasonMentions(rig.cal, "port 18"));
    CHECK(reasonMentions(rig.cal, "contradict"));
    // The report still carries what was learned before the refusal — a refusal is not an amnesia.
    CHECK(rig.cal.report().drivePortCount == 10);
    CHECK(rig.cal.report().ports[CalibRig::indexOf(1, 2)].sign == -1);
    CHECK(rig.cal.report().ports[CalibRig::indexOf(1, 2)].verdict == PulseVerdict::Contradiction);
    // And it refused AT THE PULSE, before anything downstream was measured: no scale, no width.
    CHECK(rig.cal.report().leftInchesPerRadian == 0.0);
    CHECK(rig.cal.report().rightInchesPerRadian == 0.0);
    CHECK(rig.cal.report().widthCcwIn == 0.0);

    // The same contradiction on a robot that ROLLS (κ = 1) never reaches the yaw floor: the
    // member's own encoder going clearly BACKWARD is a contradiction at any level, not a "rolled".
    // (Port 11 is the flipped one here because on a rolling robot the first port pulsed is where
    // the run ends.)
    CalibRig rolling{CalibratorConfig{}, calibPlant(1.0)};
    rolling.setFault(0, 0, MemberFault::SignFlipped);
    rolling.cal.start(facts());
    (void)runCalibration(rolling, RunOptions{});
    requireRefusal(rolling, Refusal::PulseContradiction, 11);
    const auto& last = rolling.cal.report().pulses[rolling.cal.report().pulseCount - 1];
    CHECK(last.levelVolts == 9.0);
    CHECK(std::abs(last.yawRad) < rolling.cal.config().pulseYawFloor.value());  // it did not turn
    CHECK(last.ownRad <= -rolling.cal.config().movedThreshold.value());          // it went back
}

// ═══ 8. Everything else must be still ═══════════════════════════════════════════════
// Bug caught: a second thing being driven while one member is pulsed — the yaw would then be a
// sum of two motions and the side assignment a coin toss. Note what is NOT checked: the pulsed
// member's own side-mates, which are BACK-DRIVEN through the gear train and must turn.
TEST_CASE("DrivetrainCalibrator 8: a motor that did not move on the push, creeping during a "
          "pulse, refuses the run") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.cal.start(facts());
    RunOptions opt;
    opt.hook = [](CalibRig& r, const Script&) {
        // Only once a PULSE is actually driving something: creeping during the rest window would
        // be caught by the rest check instead, which is a different (also correct) refusal.
        const bool pulsing = std::abs(r.h.plant().trueWheelSpin()[0].value()) > 0.5
                             || std::abs(r.h.plant().trueWheelSpin()[1].value()) > 0.5;
        if (r.cal.phase() == Phase::SideAssign && pulsing) {
            r.idle[1].setVelocity(AngularVelocity{3.0});  // port 6 starts turning
        }
    };
    (void)runCalibration(rig, opt);
    requireRefusal(rig, Refusal::NonDriveMoved, 6);
    CHECK(reasonMentions(rig.cal, "port 6"));
    CHECK(reasonMentions(rig.cal, "not a drive motor"));

    // The CONTROL: the same run with the side-mates turning (which they always do) is fine — the
    // check must not fire on the coupled train it is standing next to.
    CalibRig ok{CalibratorConfig{}, calibPlant(0.0)};
    ok.cal.start(facts());
    CHECK(runCalibration(ok, RunOptions{}) == Phase::Done);
}

// ═══ 9. Wheels up ═══════════════════════════════════════════════════════════════════
// Bug caught: a calibration run on a robot up on blocks producing a confident, meaningless
// description. Every pulse turns a motor and its train and nothing turns the robot, so the side
// assignment would be whatever noise the IMU happened to have. From inside, wheels-up looks
// exactly like a robot that ROLLED (its train moved, the yaw did not) — the refusal says both.
TEST_CASE("DrivetrainCalibrator 9: with the heading held still the run refuses at port 11 saying "
          "the robot did not turn and naming the FLOOR; the yaw floor attacked from both sides") {
    HeldHeading held;
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0), &held};
    rig.cal.start(facts());
    (void)runCalibration(rig, RunOptions{});
    requireRefusal(rig, Refusal::PulseRolled, 11);  // the first port pulsed
    CHECK(reasonMentions(rig.cal, "FLOOR"));
    CHECK(levelsOf(rig.cal, 11) == "3V:under the floor -- next level 6V:under the floor -- next "
                                   "level 9V:ROLLED ");

    // BOTH SIDES of the floor, on the real plant: port 11's 9 V pulse turns the robot by a known
    // amount, so a floor just under it passes and a floor just over it refuses.
    CalibRig measure{CalibratorConfig{}, calibPlant(0.0)};
    measure.cal.start(facts());
    REQUIRE(runCalibration(measure, RunOptions{}) == Phase::Done);
    const double observed =
        std::abs(measure.cal.report().ports[CalibRig::indexOf(0, 0)].pulseYawRad);
    CAPTURE(observed);
    REQUIRE(observed > 0.0);
    for (int over = 0; over < 2; ++over) {
        CAPTURE(over);
        CalibratorConfig cfg;
        cfg.pulseYawFloor = shulib::units::AngleDim{over != 0 ? observed * 1.05 : observed * 0.95};
        CalibRig rig2{cfg, calibPlant(0.0)};
        rig2.cal.start(facts());
        (void)runCalibration(rig2, RunOptions{});
        if (over != 0) {
            requireRefusal(rig2, Refusal::PulseRolled, 11);
        } else {
            CHECK(rig2.cal.phase() == Phase::Done);
        }
    }
}

// ═══ 10. The distance scale, from the ONE push ══════════════════════════════════════
// Bug caught: the MEAN where the median belongs. One member of five reading short drags a mean by
// a fifth of the shortfall and every distance the robot ever drives is wrong by that much,
// silently. The median cannot be moved by one of five.
TEST_CASE("DrivetrainCalibrator 10: each side's scale from the one 48 in push is the plant's own "
          "radius to 1e-9 — what the two-push procedure gave — and a port that dropped off the bus "
          "mid-push leaves the median untouched while showing in the spread") {
    CalibRig clean{CalibratorConfig{}, calibPlant(0.0)};
    clean.cal.start(facts());
    REQUIRE(runCalibration(clean, RunOptions{}) == Phase::Done);
    const auto& cleanRep = clean.cal.report();
    // The number the Part 1 procedure's separate tape push produced on the same plant (its case
    // 10): the plant's own wheel radius. One push, the same scale.
    CHECK(cleanRep.leftInchesPerRadian == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(cleanRep.rightInchesPerRadian == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(clean.cal.result().left.inchesPerRadian.value()
          == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(clean.cal.result().right.inchesPerRadian.value()
          == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(cleanRep.leftSpreadRad == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(std::abs(cleanRep.pushStraightnessRad) < 1e-9);

    // Port 13 drops off the bus for the second half of the push — a frozen reading, like robot
    // two's port 19 that night — and comes back after DONE. Its captured travel is short; one of
    // five, which the median must ignore entirely.
    CalibRig flaky{CalibratorConfig{}, calibPlant(0.0)};
    flaky.cal.start(facts());
    RunOptions opt;
    opt.pauseAtInches = 20.0;  // a moment at rest mid-push, so the frozen reading is a still one
    bool frozen = false;
    bool restored = false;
    opt.hook = [&](CalibRig& r, const Script& s) {
        if (!frozen && s.paused) {
            r.setFault(0, 2, MemberFault::Frozen);
            frozen = true;
        }
        if (frozen && !restored && r.cal.phase() == Phase::SideAssign) {
            r.setFault(0, 2, MemberFault::None);  // the port answers again
            restored = true;
        }
    };
    REQUIRE(runCalibration(flaky, opt) == Phase::Done);
    CHECK(frozen);
    CHECK(restored);
    const auto& rep = flaky.cal.report();
    CHECK(rep.leftInchesPerRadian == doctest::Approx(cleanRep.leftInchesPerRadian).epsilon(1e-9));
    CHECK(rep.rightInchesPerRadian == doctest::Approx(cleanRep.rightInchesPerRadian).epsilon(1e-9));
    // The shortfall is VISIBLE, not hidden: the report shows the side's spread, on the left only.
    CHECK(rep.leftSpreadRad > 10.0);
    CHECK(rep.rightSpreadRad == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(rep.ports[CalibRig::indexOf(0, 2)].pushTravelRad
          < 0.6 * rep.ports[CalibRig::indexOf(0, 1)].pushTravelRad);
}

// ═══ 11. The push must be straight ══════════════════════════════════════════════════
// Bug caught: a curved push divided by a tape distance (the tape measures the straight line; the
// outside wheel measures a longer arc) — and, since the push now also captures the SIGNS, a twist
// that could roll one side backward and invert every sign on it.
TEST_CASE("DrivetrainCalibrator 11: a push that curves is refused at DONE, before anything is "
          "powered") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.cal.start(facts());
    RunOptions opt;
    opt.curve = 0.85;  // the right side rolls slower: the robot arcs
    (void)runCalibration(rig, opt);
    requireRefusal(rig, Refusal::PushCurved, 0);
    CHECK(reasonMentions(rig.cal, "curved"));
    CHECK(reasonMentions(rig.cal, "straight"));
    CHECK(std::abs(rig.cal.report().pushStraightnessRad)
          > rig.cal.config().straightBound.value());
    CHECK(rig.cal.report().pulseCount == 0);  // nothing was ever driven
}

// ═══ 12. The effective track width, and the round trip into the consumer ════════════
// Bug caught: the W formula's sign or its terms (ΔL + ΔR instead of ΔR − ΔL would read a straight
// line as a turn), one direction taken for both, or a description whose numbers do not actually
// work in the odometry they were measured for.
TEST_CASE("DrivetrainCalibrator 12: the spin recovers the plant's true track width both ways, and "
          "the result's own geometry tracks the truth through a figure-eight") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.setFault(0, 0, MemberFault::Reversed);
    rig.setFault(1, 3, MemberFault::Reversed);
    rig.cal.start(facts());
    REQUIRE(runCalibration(rig, RunOptions{}) == Phase::Done);

    const auto& rep = rig.cal.report();
    CHECK(rep.widthCcwIn == doctest::Approx(kTrackWidth).epsilon(1e-6));
    CHECK(rep.widthCwIn == doctest::Approx(kTrackWidth).epsilon(1e-6));
    CHECK(rep.widthCcwIn > 0.0);
    CHECK(rep.widthCwIn > 0.0);
    CHECK(rep.widthDisagreement < 1e-6);
    const DrivetrainDescription d = rig.cal.result();
    CHECK(d.trackWidth.value() == doctest::Approx(kTrackWidth).epsilon(1e-9));
    CHECK(d.trackWidth.value() == doctest::Approx(0.5 * (rep.widthCcwIn + rep.widthCwIn)).epsilon(1e-15));
    CHECK(d.source == shulib::hal::DescriptionSource::RobotMeasured);
    CHECK(d.cartridge == MotorCartridge::Blue);
    CHECK(d.imuPort == 2);
    CHECK(std::strstr(d.provenance, "host test") != nullptr);
    CHECK(std::strstr(d.provenance, "ROBOT-MEASURED: 10 ports, push 48 in") != nullptr);
    char why[160];
    CHECK(d.valid(why, sizeof why));
    CHECK(rig.cal.status().phase == Phase::Done);
    CHECK(std::strstr(rig.cal.status().prompt, "DONE") != nullptr);

    // ── the round trip: build the graph a later program would build, from the description ──
    GraphFromDescription graph{rig, d};
    shulib::localization::DriveEncoderOdometry odom{
        rig.h.imu(),        graph.left,       graph.right, d.leftGeometry(), d.rightGeometry(),
        d.trackWidth,       rig.h.truePose()};
    // The plant's own feedforward, kS included, so a commanded speed is exactly the speed driven.
    shulib::control::Feedforward ff{{.kS = kUsualKs, .kV = kKv, .kA = 0.0}};
    auto commandBody = [&](const ChassisSpeeds& body) {
        const auto wheels = rig.kin.toWheels(body);
        graph.left.setVoltage(ff.calculate(wheels[0]));
        graph.right.setVoltage(ff.calculate(wheels[1]));
    };
    double worst = 0.0;
    for (int tick = 0; tick < 600; ++tick) {
        // A figure-eight: CCW arc, then CW arc, at the same speed.
        const double omega = tick < 300 ? 1.0 : -1.0;
        commandBody(ChassisSpeeds{Velocity{20.0}, Velocity{0.0}, AngularVelocity{omega}});
        rig.h.plant().step(Time{kDt});
        odom.update();
        worst = std::max(worst, posError(odom.pose(), rig.h.truePose()));
        REQUIRE_FALSE(odom.lastDeltaImplausible());
        REQUIRE(std::abs(odom.lastHeadingDisagreement()) < 1e-9);  // the width is right
    }
    MESSAGE("figure-eight worst error under the CALIBRATED geometry: ", worst, " in");
    CHECK(worst < 1e-6);
    CHECK(posError(rig.h.truePose(), Pose2d{}) > 5.0);  // it really drove somewhere
}

// ═══ 13. Under hostility: scrub, and reproducibility ════════════════════════════════
// Bug caught: a calibration that only works on a perfect plant. T8's claim — that the EFFECTIVE
// width from a spin is LARGER than a taped one because a turning tank scrubs — is checked here in
// the only place it can be: a model of scrub the plant itself does not have.
TEST_CASE("DrivetrainCalibrator 13: turn scrub makes the effective width LARGER than the true one, "
          "within 8 %, and four hostile seeds reproduce inside the stated tolerances") {
    TurnScrub scrub;
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0), &scrub};
    rig.cal.start(facts());
    REQUIRE(runCalibration(rig, RunOptions{}) == Phase::Done);
    const double w = rig.cal.result().trackWidth.value();
    MESSAGE("effective width under 5 % turn scrub: ", w, " in (true 12.5)");
    CHECK(w > kTrackWidth);                                  // T8's direction, not just its size
    CHECK(w == doctest::Approx(kTrackWidth / 0.95).epsilon(0.01));
    CHECK(std::abs(w - kTrackWidth) / kTrackWidth < 0.08);   // within 8 % of the tape's number
    // The distance scale is NOT affected: a straight push scrubs nothing. That is exactly why a
    // taped width and an effective width are two different numbers.
    CHECK(rig.cal.report().leftInchesPerRadian == doctest::Approx(kPlantRadius).epsilon(1e-9));

    // ── reproducibility over four hostile seeds (R3d brief §4.4) ─────────────────────
    double scales[4] = {};
    double widths[4] = {};
    for (std::uint64_t seed = 1; seed <= 4; ++seed) {
        CAPTURE(seed);
        shulib::sim::EncoderHostileModel encoders{};
        shulib::sim::ImuHostileModel imu{};
        shulib::sim::ChainedDegradation world{{&encoders, &imu}};
        SimHarnessConfig pcfg = calibPlant(0.0);
        pcfg.plant.seed = seed;
        CalibRig hostile{CalibratorConfig{}, pcfg, &world};
        hostile.cal.start(facts());
        const Phase end = runCalibration(hostile, RunOptions{});
        REQUIRE_MESSAGE(end == Phase::Done, std::string{hostile.cal.reason()});
        const std::size_t k = static_cast<std::size_t>(seed - 1);
        scales[k] = hostile.cal.result().left.inchesPerRadian.value();
        widths[k] = hostile.cal.result().trackWidth.value();
    }
    double scaleLo = scales[0], scaleHi = scales[0], widthLo = widths[0], widthHi = widths[0];
    for (std::size_t k = 1; k < 4; ++k) {
        scaleLo = std::min(scaleLo, scales[k]);
        scaleHi = std::max(scaleHi, scales[k]);
        widthLo = std::min(widthLo, widths[k]);
        widthHi = std::max(widthHi, widths[k]);
    }
    const double scaleSpread = (scaleHi - scaleLo) / scaleLo;
    const double widthSpread = (widthHi - widthLo) / widthLo;
    MESSAGE("reproducibility over 4 hostile seeds: scale spread ", 100.0 * scaleSpread,
            " %, width spread ", 100.0 * widthSpread, " % (bounds ",
            100.0 * kReproScaleTolerance, " / ", 100.0 * kReproWidthTolerance, " %)");
    CHECK(scaleSpread < kReproScaleTolerance);
    CHECK(widthSpread < kReproWidthTolerance);
    // ... and the runs are not the pristine run: the encoder's tick grid moved every scale off the
    // plant's exact radius, and the IMU's per-boot bias draw moved the WIDTH seed by seed.
    CHECK(scaleLo != kPlantRadius);
    CHECK(scaleLo == doctest::Approx(kPlantRadius).epsilon(1e-3));
    CHECK(widthHi > widthLo);
}

// ═══ 14. A fight during the spin ════════════════════════════════════════════════════
// Bug caught: the shared coupled-side monitor not wired in, or a refusal that leaves five motors
// stalled against four. This is the one place a calibration can strip a gear train.
TEST_CASE("DrivetrainCalibrator 14: a member fighting its side during the spin aborts naming the "
          "port, with every motor at 0 V and coasting") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.cal.start(facts());
    RunOptions opt;
    opt.hook = [](CalibRig& r, const Script&) {
        // Healthy through the push and the pulses; a fight begins as the spin does.
        if (r.cal.phase() == Phase::SpinTest) {
            r.setFault(1, 4, MemberFault::SignFlipped);  // port 20
        }
    };
    (void)runCalibration(rig, opt);
    requireRefusal(rig, Refusal::SpinFight, 20);
    CHECK(reasonMentions(rig.cal, "RIGHT"));
    CHECK(reasonMentions(rig.cal, "fought"));
    CHECK(std::strstr(rig.cal.status().progress, "port 20") != nullptr);
}

// ═══ 15. Every unbounded wait has a bound ═══════════════════════════════════════════
// Bug caught: a calibration that hangs — on a robot, with a person standing over it, waiting for
// something that will never happen.
TEST_CASE("DrivetrainCalibrator 15: an IMU that never readies, and a spin that starts but never "
          "finishes, both refuse at their configured bounds") {
    SUBCASE("the IMU never becomes ready") {
        NeverReady never;
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0), &never};
        rig.cal.start(facts());
        for (int t = 0; t < 2000 && rig.cal.phase() == Phase::WaitImu; ++t) {
            rig.cal.tick();
            rig.h.plant().step(Time{kDt});
        }
        requireRefusal(rig, Refusal::ImuNotReady, 0);
        CHECK(reasonMentions(rig.cal, "IMU never became ready"));
        CHECK(reasonMentions(rig.cal, "port 2"));
        // It waited the CONFIGURED time, not less: 5 s at 10 ms is past tick 500.
        CHECK(rig.h.clock().now().value() > rig.cal.config().imuReadyTimeout.value());
    }
    SUBCASE("the spin starts, then stops turning before its turn count") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        rig.cal.start(facts());
        RunOptions opt;
        opt.stopWhen = [](CalibRig& r) {
            return r.cal.status().spinDirection != 0.0 && std::abs(r.cal.status().spinTurns) > 0.1;
        };
        REQUIRE(runCalibration(rig, opt) == Phase::SpinTest);
        // From here the clock advances but the world does not — a robot that jammed against a
        // wall mid-spin. The start check has already passed (it turned 36 deg), so only the
        // spin's own timeout can end this.
        const double spinStart = rig.h.clock().now().value();
        for (int i = 0; i < 4000 && rig.cal.phase() == Phase::SpinTest; ++i) {
            rig.cal.tick();
            rig.h.clock().advance(Time{kDt});
        }
        requireRefusal(rig, Refusal::SpinTimeout, 0);
        CHECK(reasonMentions(rig.cal, "spin reached only"));
        CHECK(rig.h.clock().now().value() - spinStart
              < rig.cal.config().spinTimeout.value() + 0.1);
    }
}

// ═══ 16. A missed tick is a stop ════════════════════════════════════════════════════
// Bug caught: a caller that stops ticking leaving the motors running. The calibrator holds its
// command between ticks, so nothing else would stop them (the C2 stalled-pacer lesson).
TEST_CASE("DrivetrainCalibrator 16: a 0.2 s gap between ticks while spinning stops the motors and "
          "refuses") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.cal.start(facts());
    bool skipped = false;
    RunOptions opt;
    opt.hook = [&skipped](CalibRig& r, const Script&) {
        if (!skipped && r.cal.phase() == Phase::SpinTest
            && std::abs(r.h.trueBodyTwist().omega().value()) > 1.0) {
            skipped = true;
            // The caller went away for 0.2 s — the world kept moving, nothing called tick().
            for (int i = 0; i < 20; ++i) {
                r.h.plant().step(Time{kDt});
            }
        }
    };
    (void)runCalibration(rig, opt);
    CHECK(skipped);
    requireRefusal(rig, Refusal::MissedTick, 0);
    CHECK(reasonMentions(rig.cal, "a tick was missed"));
    CHECK(reasonMentions(rig.cal, "motors stopped"));

    // BOTH SIDES of the bound: a gap just under commandTimeout is not a missed tick.
    CalibRig fine{CalibratorConfig{}, calibPlant(0.0)};
    fine.cal.start(facts());
    bool nudged = false;
    RunOptions ok;
    ok.hook = [&nudged](CalibRig& r, const Script&) {
        if (!nudged && r.cal.phase() == Phase::SpinTest
            && std::abs(r.h.trueBodyTwist().omega().value()) > 1.0) {
            nudged = true;
            for (int i = 0; i < 9; ++i) {  // 9 extra steps + this tick's = 0.10 s, not MORE than it
                r.h.plant().step(Time{kDt});
            }
        }
    };
    CHECK(runCalibration(fine, ok) == Phase::Done);
    CHECK(nudged);
}

// ═══ 17. abort() is unconditional ═══════════════════════════════════════════════════
// Bug caught: a safe state that depends on which phase the run was in. A person reaching for the
// stop button must get the same thing every time: no voltage, no brake, a robot that can be
// pushed clear.
TEST_CASE("DrivetrainCalibrator 17: abort() from every phase leaves the safe state, and result() "
          "is then a precondition breach") {
    // Every phase a run can be in (Part 2F-2 removed the never-entered ScalePush).
    const Phase phases[] = {Phase::WaitImu, Phase::PushCapture, Phase::SideAssign,
                            Phase::SpinTest, Phase::Done};
    for (Phase target : phases) {
        CAPTURE(static_cast<int>(target));
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        rig.cal.start(facts());
        bool aborted = false;
        // Put every motor into the WORST state abort() could find it in — driven, and HOLDING
        // rather than coasting — immediately before the abort. Without this the test proves
        // nothing: at the instant each phase begins, the calibrator has usually just commanded
        // the safe state itself for its own reasons, so an abort() that did nothing at all would
        // still leave the motors safe. (That is exactly what happened on the first pass of Part
        // 1: the "skip the safe state in abort()" mutation stayed GREEN.)
        auto derange = [](CalibRig& r) {
            for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
                r.candidates[i]->setVoltage(Voltage{7.0});
                r.candidates[i]->setBrakeMode(BrakeMode::Hold);
            }
        };
        RunOptions opt;
        opt.hook = [&](CalibRig& r, const Script&) {
            if (!aborted && r.cal.phase() == target) {
                derange(r);
                r.cal.abort("the operator hit STOP");
                aborted = true;
            }
        };
        if (target == Phase::WaitImu) {
            derange(rig);
            rig.cal.abort("the operator hit STOP");  // WaitImu ends on the first tick
            aborted = true;
        } else {
            (void)runCalibration(rig, opt);
            if (target == Phase::Done && !aborted) {
                derange(rig);
                rig.cal.abort("the operator hit STOP");
                aborted = true;
            }
        }
        CHECK(aborted);
        requireRefusal(rig, Refusal::Aborted, 0);
        CHECK(std::strcmp(rig.cal.reason(), "the operator hit STOP") == 0);
    }
    // Idle too — abort before anything started is still the safe state, even from a deranged one.
    CalibRig idle{CalibratorConfig{}, calibPlant(0.0)};
    for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
        idle.candidates[i]->setVoltage(Voltage{7.0});
        idle.candidates[i]->setBrakeMode(BrakeMode::Brake);
    }
    idle.cal.abort();
    requireRefusal(idle, Refusal::Aborted, 0);
    // And the Answer::Abort route is the same route.
    CalibRig viaAnswer{CalibratorConfig{}, calibPlant(0.0)};
    viaAnswer.cal.start(facts());
    for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
        viaAnswer.candidates[i]->setVoltage(Voltage{7.0});
        viaAnswer.candidates[i]->setBrakeMode(BrakeMode::Brake);
    }
    viaAnswer.cal.answer(Answer::Abort);
    requireRefusal(viaAnswer, Refusal::Aborted, 0);
}

// ═══ 18. The spin-agreement bound, from both sides ══════════════════════════════════
// Bug caught: the two directions accepted however far apart they are. Asymmetric scrub biases one
// direction, and averaging two numbers that disagree hides the disagreement instead of reporting
// it. And (2F-1) the reason used to print two percent signs, which PROS re-reads as a format.
TEST_CASE("DrivetrainCalibrator 18: two spin directions that disagree are refused at the bound, "
          "and accepted just outside it; the reason says pct, never the percent sign") {
    for (int tight = 0; tight < 2; ++tight) {
        CAPTURE(tight);
        SecondSpinScrub scrub;
        CalibratorConfig cfg;
        cfg.spinAgreement = tight != 0 ? 0.03 : 0.20;
        CalibRig rig{cfg, calibPlant(0.0), &scrub};
        rig.cal.start(facts());
        RunOptions opt;
        opt.hook = [&scrub](CalibRig& r, const Script&) {
            // Switch the scrub on once the FIRST direction has produced its number.
            scrub.active = r.cal.report().widthCcwIn != 0.0;
        };
        const Phase end = runCalibration(rig, opt);
        const double disagreement = rig.cal.report().widthDisagreement;
        CAPTURE(disagreement);
        CHECK(rig.cal.report().widthCwIn > rig.cal.report().widthCcwIn);
        if (tight != 0) {
            requireRefusal(rig, Refusal::SpinDisagree, 0);
            CHECK(reasonMentions(rig.cal, "disagree about the track width"));
            CHECK(reasonMentions(rig.cal, "pct"));
            CHECK(std::strchr(rig.cal.reason(), '%') == nullptr);
        } else {
            CHECK(end == Phase::Done);
            CHECK(disagreement > 0.03);   // the same disagreement, now inside the bound
            CHECK(disagreement < 0.20);
        }
    }
}

// ═══ 19. Contract breaches ══════════════════════════════════════════════════════════
// Bug caught: the sign applied twice (the header's rule 1 and landmine 1) going unguarded, and a
// config that would make a ramp or a guard meaningless being accepted.
TEST_CASE("DrivetrainCalibrator 19: the caller contract is enforced, including the ramp's and the "
          "rest's new preconditions") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    std::array<std::int8_t, CalibRig::kCandidates> negative = rig.ports;
    negative[3] = static_cast<std::int8_t>(-negative[3]);
    CHECK_THROWS_AS((DrivetrainCalibrator{rig.candidateSpan(),
                                          std::span<const std::int8_t>{negative}, rig.h.imu(),
                                          rig.h.clock()}),
                    PreconditionError);
    std::array<std::int8_t, CalibRig::kCandidates> duplicate = rig.ports;
    duplicate[4] = duplicate[3];
    CHECK_THROWS_AS((DrivetrainCalibrator{rig.candidateSpan(),
                                          std::span<const std::int8_t>{duplicate}, rig.h.imu(),
                                          rig.h.clock()}),
                    PreconditionError);
    // start() refuses facts nobody stated, and an IMU port that is also a motor.
    auto bad = facts();
    bad.imuPort = 13;
    CHECK_THROWS_AS(rig.cal.start(bad), PreconditionError);
    bad = facts();
    bad.cartridge = static_cast<MotorCartridge>(0);
    CHECK_THROWS_AS(rig.cal.start(bad), PreconditionError);
    bad = facts();
    bad.kind = static_cast<shulib::hal::DrivetrainKind>(9);
    CHECK_THROWS_AS(rig.cal.start(bad), PreconditionError);

    // The ramps: at least one level, at most four, strictly increasing, each in (0, 12] V.
    const auto construct = [&rig](const CalibratorConfig& c) {
        DrivetrainCalibrator cal{rig.candidateSpan(), rig.portSpan(), rig.h.imu(), rig.h.clock(),
                                 c};
        (void)cal;
    };
    CHECK_NOTHROW(construct(CalibratorConfig{}));
    CalibratorConfig c;
    c.pulseLevelCount = 0;
    CHECK_THROWS_AS(construct(c), PreconditionError);
    c = CalibratorConfig{};
    c.pulseLevelCount = 5;
    CHECK_THROWS_AS(construct(c), PreconditionError);
    c = CalibratorConfig{};
    c.pulseLevels[1] = Voltage{3.0};  // repeats the first level
    CHECK_THROWS_AS(construct(c), PreconditionError);
    c = CalibratorConfig{};
    c.pulseLevels[2] = Voltage{12.5};  // past the V5's ceiling
    CHECK_THROWS_AS(construct(c), PreconditionError);
    c = CalibratorConfig{};
    c.spinLevels[1] = Voltage{2.0};  // falls
    CHECK_THROWS_AS(construct(c), PreconditionError);
    c = CalibratorConfig{};
    c.spinLevelCount = 1;
    CHECK_NOTHROW(construct(c));
    // The rest's yaw-rate floor is bounded above by the pulse's own floor: a drift slow enough to
    // pass the rest check must not be able to cross the yaw floor on its own during a pulse.
    // 2 deg/s x 0.4 s = 0.8 deg < 2 deg (the default passes); 5.1 deg/s x 0.4 s > 2 deg refuses.
    c = CalibratorConfig{};
    c.restYawRateFloor = AngularVelocity{5.1 * kDegreesToRadians};
    CHECK_THROWS_AS(construct(c), PreconditionError);
    c.restYawRateFloor = AngularVelocity{4.9 * kDegreesToRadians};
    CHECK_NOTHROW(construct(c));
    c = CalibratorConfig{};
    c.spinStartTime = c.spinTimeout;  // a start check that can never fire before the timeout
    CHECK_THROWS_AS(construct(c), PreconditionError);
    c = CalibratorConfig{};
    c.decoupledOwnMin = shulib::units::AngleDim{0.0};
    CHECK_THROWS_AS(construct(c), PreconditionError);

    // An answer that arrives out of order refuses rather than silently doing nothing.
    CalibRig stray{CalibratorConfig{}, calibPlant(0.0)};
    stray.cal.start(facts());
    stray.cal.tick();
    // (Part 2F-2: this used the retired Answer::Distance48; FRONT FIRST while the push is still
    // being watched is the same out-of-order touch on the live surface.)
    REQUIRE(stray.cal.accepts(Answer::Done));  // the IMU is ready: the push is being watched
    stray.cal.answer(Answer::FrontFirst);
    requireRefusal(stray, Refusal::OutOfOrder, 0);
    CHECK(reasonMentions(stray.cal, "FRONT FIRST"));
    CHECK(reasonMentions(stray.cal, "waiting for DONE"));
    CHECK(reasonMentions(stray.cal, "nothing was recorded"));
}

// ── 21 — accepts(): what a panel may OFFER, pinned against what answer() actually DOES ──
//
// ADDED AT R3d PART 2, in Part 1's file, because Part 2 found the gap here (RESUMING: "a chunk
// that finds a flaw in an earlier chunk fixes it there"). answer() REFUSES an out-of-order
// answer and ends the run — right, but it makes a wrong BUTTON destructive. accepts() is the
// query; THIS case is what stops it drifting from answer(), because it never asserts a
// hard-coded expectation: for every waiting step and EVERY Answer it runs the real answer() on a
// FRESH rig and requires the two to agree. (2F-1: the push has two waiting steps now. 2F-2: the
// three retired distance answers are gone from the enum, so the sweep is every Answer there is.)
TEST_CASE("DrivetrainCalibrator 21: accepts() agrees with answer(), at every waiting step") {
    enum class Stop { PushWatching, PushAwaitingFront };
    const Stop stops[] = {Stop::PushWatching, Stop::PushAwaitingFront};
    const Answer answers[] = {Answer::Done, Answer::FrontFirst, Answer::BackFirst, Answer::Abort};

    const auto driveTo = [](CalibRig& rig, Stop stop) {
        rig.cal.start(facts());
        RunOptions opt;
        opt.stopWhen = [stop](CalibRig& r) {
            return stop == Stop::PushWatching ? r.cal.pushReady()
                                              : r.cal.accepts(Answer::FrontFirst);
        };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
    };

    for (const Stop stop : stops) {
        for (const Answer a : answers) {
            CalibRig ask{CalibratorConfig{}, calibPlant(0.0)};
            driveTo(ask, stop);
            const bool offered = ask.cal.accepts(a);

            CalibRig act{CalibratorConfig{}, calibPlant(0.0)};
            driveTo(act, stop);
            act.cal.answer(a);
            const bool refused = act.cal.phase() == Phase::Aborted
                                 && act.cal.refusal() == Refusal::OutOfOrder;
            CHECK_MESSAGE(offered == !refused, "step ", static_cast<int>(stop), ", answer ",
                          static_cast<int>(a), ": accepts() said ", offered,
                          " but answer() ", std::string{refused ? "REFUSED" : "accepted"});
        }
    }

    // Abort is offered everywhere, including where no answer is expected at all.
    CalibRig idle{CalibratorConfig{}, calibPlant(0.0)};
    CHECK(idle.cal.accepts(Answer::Abort));
    CHECK_FALSE(idle.cal.accepts(Answer::Done));          // Idle: nothing is waiting
    CHECK_FALSE(idle.cal.accepts(Answer::FrontFirst));
    CHECK_FALSE(idle.cal.accepts(Answer::BackFirst));
    CHECK_FALSE(idle.cal.pushReady());
}

// ═══ 22. Tonight, on the plant ══════════════════════════════════════════════════════
// The regression that DOCUMENTS 2026-09-24: robot two's calibrator stopped with "port 11: the
// robot turned only 0.17 deg (floor 2.00 deg)". The Part 1 procedure is the ramp with ONE level
// at 3 V; on this plant at the tree's usual kS it turns the robot by EXACTLY nothing, and is now
// classified (nothing moved at the top — its only — level) rather than refused blind. On the Part
// 1 rig's frictionless plant the same pulse passed: that is why the host was green.
TEST_CASE("DrivetrainCalibrator 22: the old single 3 V pulse at kS 1.2 turns the robot exactly "
          "0.00 deg and is refused STILL at port 11 — and on the Part 1 rig it would have passed") {
    CalibratorConfig old;
    old.pulseLevelCount = 1;  // 3 V only: the Part 1 pulse
    CalibRig rig{old, calibPlant(0.0)};
    rig.cal.start(facts());
    (void)runCalibration(rig, RunOptions{});
    requireRefusal(rig, Refusal::PulseStill, 11);
    const auto& rep = rig.cal.report();
    REQUIRE(rep.pulseCount == 1);
    CHECK(rep.pulses[0].port == 11);
    CHECK(rep.pulses[0].levelVolts == 3.0);
    CHECK(rep.pulses[0].yawRad == 0.0);        // exactly: one fifth of 3 V is inside kS = 1.2
    CHECK(rep.pulses[0].ownRad == 0.0);        // not even its own shaft
    CHECK(rep.pulses[0].othersMaxRad == 0.0);
    CHECK(rep.pulses[0].verdict == PulseVerdict::Still);
    CHECK(reasonMentions(rig.cal, "0.00 deg"));
    CHECK(reasonMentions(rig.cal, "at 3.0 V"));

    CalibRig partOneRig{old, calibPlant(0.0, 0.0)};  // kS = 0: the rig the old cases ran on
    partOneRig.cal.start(facts());
    CHECK(runCalibration(partOneRig, RunOptions{.plantKs = 0.0}) == Phase::Done);
    CHECK(partOneRig.cal.report().ports[0].pulseLevelVolts == 3.0);
}

// ═══ 23. The ramp stops at the first level that turns the robot ═════════════════════
// Bug caught: a ramp that always runs to the top (every port jerked at 9 V when 3 V would do), or
// one that skips a level. What level suffices is a property of the PLANT's friction here; on the
// robot, of its gear trains — the serial record says which.
TEST_CASE("DrivetrainCalibrator 23: each port is decided at the FIRST level that clears the yaw "
          "floor — 3 V at kS 0, 6 V at kS 0.9, 9 V at kS 1.2 — and never pulsed again") {
    struct Row {
        double kS;
        const char* levels;  // every port's records, identical for all ten
        double decidedAt;
    };
    const Row rows[] = {
        {0.0, "3V:LEFT ", 3.0},
        {0.9, "3V:under the floor -- next level 6V:LEFT ", 6.0},
        {1.2, "3V:under the floor -- next level 6V:under the floor -- next level 9V:LEFT ", 9.0},
    };
    for (const Row& row : rows) {
        CAPTURE(row.kS);
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0, row.kS)};
        rig.cal.start(facts());
        RunOptions opt;
        opt.plantKs = row.kS;
        REQUIRE(runCalibration(rig, opt) == Phase::Done);
        CHECK(levelsOf(rig.cal, 11) == std::string{row.levels});
        for (std::size_t i = 0; i < 2 * CalibRig::kPerWheel; ++i) {
            CAPTURE(rig.ports[i]);
            CHECK(rig.cal.report().ports[i].pulseLevelVolts == row.decidedAt);
        }
        // 10 drive ports, each with one record per level it needed.
        const std::size_t levelsUsed = row.decidedAt == 3.0 ? 1 : row.decidedAt == 6.0 ? 2 : 3;
        CHECK(rig.cal.report().pulseCount == 10 * levelsUsed);
    }
}

// ═══ 24. A slipping gear is LEFT OUT, and the run goes on ═══════════════════════════
// Bug caught: a robot with one slipping gear that can never be calibrated (refused), or — worse —
// a slipping motor guessed onto a side. Robot two's port 18: it travels short on a push (it is
// captured as a drive port) and its own pulse moved nothing but itself.
TEST_CASE("DrivetrainCalibrator 24: a DECOUPLED member is LEFT OUT with a warning naming it, and "
          "the run SAVES a valid description without it") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0, kUsualKs, /*drag*/ 0.7)};
    rig.setFault(1, 2, MemberFault::Decoupled);  // port 18: 70 % of the train reaches its encoder
    rig.cal.start(facts());
    const Phase end = runCalibration(rig, RunOptions{});
    REQUIRE_MESSAGE(end == Phase::Done, std::string{rig.cal.reason()});
    const auto& rep = rig.cal.report();
    const std::size_t i18 = CalibRig::indexOf(1, 2);
    // It WAS a drive port (the push dragged it), and it is now left out.
    CHECK(rep.ports[i18].sign == 1);
    CHECK(rep.ports[i18].pushTravelRad > 0.6 * rep.ports[CalibRig::indexOf(1, 1)].pushTravelRad);
    CHECK(rep.ports[i18].excluded);
    CHECK(rep.ports[i18].side == 0);
    CHECK(rep.ports[i18].verdict == PulseVerdict::Decoupled);
    CHECK(levelsOf(rig.cal, 18) == "3V:under the floor -- next level 6V:under the floor -- next "
                                   "level 9V:LEFT OUT (decoupled) ");
    CHECK(rep.ports[i18].pulseShaftRad >= rig.cal.config().decoupledOwnMin.value());  // it spun
    CHECK(rep.ports[i18].pulseOthersRad < rig.cal.config().movedThreshold.value());   // alone
    CHECK(rep.excludedCount == 1);
    CHECK(rep.excludedPorts[0] == 18);
    CHECK(rig.cal.status().excludedCount == 1);
    CHECK(std::strstr(rig.cal.warning(),
                      "port 18 turns but does not move the robot -- its gear is slipping; it is "
                      "left out")
          != nullptr);
    CHECK(screenSafe(rig.cal.warning()));
    CHECK(rig.cal.refusal() == Refusal::None);
    // The description the program would SAVE: valid, nine ports, no 18.
    const DrivetrainDescription& d = rig.cal.result();
    char why[160];
    CHECK(d.valid(why, sizeof why));
    CHECK(d.left.count == 5);
    CHECK(d.right.count == 4);
    CHECK(d.sideOf(18) == 0);
    CHECK(d.signedPortOf(18) == 0);
    // The right side's scale is its FOUR healthy members' median — the plant's radius, exactly.
    CHECK(rep.rightInchesPerRadian == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(std::strstr(d.provenance, "9 ports") != nullptr);
    // It was never commanded again: through the whole spin its command stayed 0 V.
    CHECK(rig.member(1, 2).commandedVoltage().value() == 0.0);
}

// ═══ 25. κ near 1 — the robot ROLLS instead of turning ══════════════════════════════
// Bug caught: a pulse that fails on a free-rolling robot and says only "floor". Robot two is
// all-omni. On this plant κ = 1 is pure translation; what κ the robot shows is unmeasured (HA-137).
TEST_CASE("DrivetrainCalibrator 25: at k = 1 and k = 0.9 the 9 V pulse ROLLS the robot and is "
          "refused naming the port; at k = 0.6 it still calibrates") {
    for (const double kappa : {1.0, 0.9}) {
        CAPTURE(kappa);
        CalibRig rig{CalibratorConfig{}, calibPlant(kappa)};
        rig.cal.start(facts());
        (void)runCalibration(rig, RunOptions{});
        requireRefusal(rig, Refusal::PulseRolled, 11);
        CHECK(reasonMentions(rig.cal, "rolled instead of turning"));
        const auto& last = rig.cal.report().pulses[rig.cal.report().pulseCount - 1];
        CHECK(last.levelVolts == 9.0);
        CHECK(last.othersMaxRad >= rig.cal.config().movedThreshold.value());
        CHECK(std::abs(last.yawRad) < rig.cal.config().pulseYawFloor.value());
        if (kappa == 1.0) {
            CHECK(last.yawRad == 0.0);  // exactly: the other side followed all the way
        }
    }
    CalibRig halfRolls{CalibratorConfig{}, calibPlant(0.6)};
    halfRolls.cal.start(facts());
    REQUIRE_MESSAGE(runCalibration(halfRolls, RunOptions{}) == Phase::Done,
                    std::string{halfRolls.cal.reason()});
    CHECK(halfRolls.cal.result().left.count == 5);
    CHECK(halfRolls.cal.result().right.count == 5);
    CHECK(halfRolls.cal.result().sideOf(11) == -1);
}

// ═══ 26. Hand motion during a rest never leaks into a pulse ═════════════════════════
// Bug caught: rest defined by the motors alone (the Part 1 code, against its own design). A robot
// being turned by hand — or still swinging after the push — with its motors stationary counted
// as at rest, and that turn was added into the next pulse's yaw.
TEST_CASE("DrivetrainCalibrator 26: a hand turning the robot at the start of every rest window "
          "delays the pulse instead of leaking into it; one that never stops refuses") {
    HandTurn hand;
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0), &hand};
    rig.cal.start(facts());
    RunOptions opt;
    int windows = 0;
    std::uint32_t lastKey = 0xFFFFFFFFu;
    opt.hook = [&](CalibRig& r, const Script&) {
        const auto& st = r.cal.status();
        if (r.cal.phase() != Phase::SideAssign || st.pulsing) {
            lastKey = 0xFFFFFFFFu;
            return;
        }
        const auto key = static_cast<std::uint32_t>(st.pulseOrdinal) * 100u
                         + static_cast<std::uint32_t>(st.pulseLevelVolts);
        if (key != lastKey) {  // a NEW rest window: the hand turns the robot CCW for 1 s
            lastKey = key;
            hand.until = r.h.clock().now().value() + 1.0;
            ++windows;
        }
    };
    REQUIRE_MESSAGE(runCalibration(rig, opt) == Phase::Done, std::string{rig.cal.reason()});
    CHECK(windows == 30);  // ten ports, three levels each, a hand before every one
    // The sides are the plant's wheels exactly — the hand's CCW turn named none of them.
    const DrivetrainDescription& d = rig.cal.result();
    for (int p : {11, 12, 13, 14, 15}) {
        CHECK(d.sideOf(p) == -1);
    }
    for (int p : {16, 17, 18, 19, 20}) {
        CHECK(d.sideOf(p) == +1);
    }
    // And no pulse's yaw carries the hand: every one is the plant's own 9 V pivot (or nothing).
    const auto& rep = rig.cal.report();
    for (std::size_t k = 0; k < rep.pulseCount; ++k) {
        CAPTURE(k);
        CHECK(std::abs(rep.pulses[k].yawRad) < 0.12);  // the hand alone would add 0.13 rad
    }

    // A hand that never lets go: the rest never comes, and the refusal says why — no motor is
    // named, because none is turning.
    HandTurn forever;
    forever.until = 1e9;
    CalibRig stuck{CalibratorConfig{}, calibPlant(0.0), &forever};
    stuck.cal.start(facts());
    RunOptions pushOnly;
    pushOnly.hook = [&forever](CalibRig& r, const Script&) {
        // Hands off during the push itself (pushReady() needs the yaw rate too): the hand
        // arrives the moment the pulses begin.
        forever.rate = r.cal.phase() == Phase::SideAssign ? 20.0 * kDegreesToRadians : 0.0;
    };
    (void)runCalibration(stuck, pushOnly);
    requireRefusal(stuck, Refusal::RestTimeout, 0);
    CHECK(reasonMentions(stuck.cal, "still rotating"));
    CHECK(reasonMentions(stuck.cal, "before pulsing port 11"));
    CHECK(stuck.cal.report().pulseCount == 0);
}

// ═══ 27. The wrong-way guard ════════════════════════════════════════════════════════
// Bug caught: a spin that turns the WRONG way under power for the whole timeout. Sides swapped
// consistently make no fight inside either side, so the coupled-side monitor never fires.
TEST_CASE("DrivetrainCalibrator 27: a spin that turns the wrong way refuses within 15 deg") {
    MirrorOnceArmed mirror;
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0), &mirror};
    rig.cal.start(facts());
    double thetaAtArm = 0.0;
    RunOptions opt;
    opt.hook = [&](CalibRig& r, const Script&) {
        if (!mirror.armed && r.cal.phase() == Phase::SpinTest) {
            mirror.armed = true;  // from here the IMU says the robot turns the other way
            thetaAtArm = r.h.sample().theta;
        }
    };
    (void)runCalibration(rig, opt);
    requireRefusal(rig, Refusal::SpinWrongWay, 0);
    CHECK(reasonMentions(rig.cal, "WRONG way"));
    CHECK(reasonMentions(rig.cal, "sides are swapped"));
    // How far the robot REALLY turned before the stop: the 15 deg bound plus at most one tick of
    // the spin (1.7 rad/s x 10 ms = 1 deg).
    const double turned = std::abs(rig.h.sample().theta - thetaAtArm) / kDegreesToRadians;
    MESSAGE("true rotation before the wrong-way stop: ", turned, " deg");
    CHECK(turned < 16.5);
    CHECK(turned > 14.0);
    CHECK(rig.cal.report().spinCount == 0);  // it never finished a direction
}

// ═══ 28. The spin's start check and step-up ═════════════════════════════════════════
// Bug caught: a spin that cannot turn the robot stalling at 3 V for the whole 20 s timeout.
TEST_CASE("DrivetrainCalibrator 28: a spin that cannot start steps up after 2 s and refuses 2 s "
          "later; one that starts only at the second level finishes there") {
    SUBCASE("nothing turns: step up at 2 s, refuse at 4 s") {
        HeldHeading held;
        held.armed = false;
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0), &held};
        rig.cal.start(facts());
        double spinBegan = -1.0;
        double steppedAt = -1.0;
        RunOptions opt;
        opt.hook = [&](CalibRig& r, const Script&) {
            if (r.cal.phase() == Phase::SpinTest) {
                held.armed = true;  // the IMU sees no turn from here on
            }
            const auto& st = r.cal.status();
            if (spinBegan < 0.0 && st.spinDirection != 0.0) {
                spinBegan = r.h.clock().now().value();
            }
            if (steppedAt < 0.0 && st.spinLevelVolts == 6.0) {
                steppedAt = r.h.clock().now().value();
            }
        };
        (void)runCalibration(rig, opt);
        requireRefusal(rig, Refusal::SpinNoStart, 0);
        CHECK(reasonMentions(rig.cal, "cannot turn the robot"));
        CHECK(reasonMentions(rig.cal, "6.0 V"));
        REQUIRE(spinBegan > 0.0);
        REQUIRE(steppedAt > 0.0);
        const double stepAfter = steppedAt - spinBegan;
        const double refusedAfter = rig.h.clock().now().value() - spinBegan;
        MESSAGE("stepped up after ", stepAfter, " s; refused after ", refusedAfter, " s");
        CHECK(stepAfter <= 2.0 + 2 * kDt);
        CHECK(stepAfter >= 2.0 - 2 * kDt);
        CHECK(refusedAfter <= 4.0 + 2 * kDt);
    }
    SUBCASE("it breaks free at 6 V: the spin finishes there, both ways") {
        SpinStiction stiction;
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0), &stiction};
        rig.cal.start(facts());
        RunOptions opt;
        opt.hook = [&stiction](CalibRig& r, const Script&) {
            stiction.armed = r.cal.phase() == Phase::SpinTest;
        };
        REQUIRE_MESSAGE(runCalibration(rig, opt) == Phase::Done, std::string{rig.cal.reason()});
        const auto& rep = rig.cal.report();
        REQUIRE(rep.spinCount == 2);
        CHECK(rep.spins[0].levelVolts == 6.0);
        CHECK(rep.spins[1].levelVolts == 6.0);
        CHECK(rep.spins[0].direction == 1.0);
        CHECK(rep.spins[1].direction == -1.0);
        CHECK(rep.spins[0].seconds > 2.0);  // two seconds went on the level that could not turn it
        CHECK(rig.cal.result().trackWidth.value() == doctest::Approx(kTrackWidth).epsilon(1e-6));
    }
}

// ═══ 29. pushReady() is the finish's own predicates ═════════════════════════════════
// Bug caught: a DONE gate that drifts from the DONE finish — a panel offering DONE that the
// finish then refuses (two of robot two's five starts), or hiding DONE when it would work.
TEST_CASE("DrivetrainCalibrator 29: pushReady() says not-ready exactly where the finish refuses, "
          "and at-rest is its one stricter clause") {
    SUBCASE("nothing pushed: not ready; DONE is refused too-few") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        reachPushWatching(rig);
        CHECK_FALSE(rig.cal.pushReady());
        CHECK(rig.cal.status().movedNow == 0);
        CHECK_FALSE(rig.cal.status().pushFarEnough);
        CHECK(rig.cal.status().atRest);
        rig.cal.answer(Answer::Done);
        requireRefusal(rig, Refusal::PushTooFew, 0);
    }
    SUBCASE("one motor turned by hand: not ready; DONE is refused too-few") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        reachPushWatching(rig);
        rig.idle[0].setPosition(shulib::units::AngleDim{40.0});  // one candidate, well past it
        rig.cal.tick();
        CHECK(rig.cal.status().movedNow == 1);
        CHECK_FALSE(rig.cal.pushReady());
        rig.cal.answer(Answer::Done);
        requireRefusal(rig, Refusal::PushTooFew, 0);
        CHECK(reasonMentions(rig.cal, "moved 1 port(s)"));
    }
    SUBCASE("a short push: not ready; DONE is refused too-short, before anything is powered") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        rig.cal.start(facts());
        RunOptions opt;
        opt.pushInches = 4.0;  // 2.5 rad of shaft: every port moved, none a wheel turn
        opt.stopWhen = [](CalibRig& r) {
            return r.cal.status().movedNow == 10 && r.cal.status().atRest;
        };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
        CHECK_FALSE(rig.cal.status().pushFarEnough);
        CHECK_FALSE(rig.cal.pushReady());
        rig.cal.answer(Answer::Done);
        requireRefusal(rig, Refusal::PushTooShort, 0);
        CHECK(rig.cal.report().pulseCount == 0);
    }
    SUBCASE("far enough but still moving: not ready — and the finish ACCEPTS it (the one stricter "
            "clause)") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        rig.cal.start(facts());
        RunOptions opt;
        opt.pushInches = 1000.0;  // never stops on its own
        opt.stopWhen = [](CalibRig& r) { return r.cal.status().pushFarEnough; };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
        rig.pushPrimaries(2.2, 2.2);
        rig.cal.tick();
        rig.h.plant().step(Time{kDt});
        rig.cal.tick();
        CHECK(rig.cal.status().movedNow == 10);
        CHECK(rig.cal.status().pushFarEnough);
        CHECK_FALSE(rig.cal.status().atRest);
        CHECK_FALSE(rig.cal.pushReady());
        rig.cal.answer(Answer::Done);
        CHECK(rig.cal.phase() == Phase::PushCapture);
        CHECK(rig.cal.accepts(Answer::FrontFirst));  // accepted: awaiting FRONT/BACK
    }
    SUBCASE("at rest in the motors but the robot still rotating: not ready") {
        HandTurn hand;
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0), &hand};
        rig.cal.start(facts());
        RunOptions opt;
        opt.stopWhen = [](CalibRig& r) { return r.cal.pushReady(); };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
        REQUIRE(rig.cal.pushReady());
        hand.until = rig.h.clock().now().value() + 5.0;
        rig.h.plant().step(Time{kDt});
        rig.cal.tick();
        CHECK_FALSE(rig.cal.status().atRest);
        CHECK_FALSE(rig.cal.pushReady());
    }
    SUBCASE("ready: DONE goes through") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        rig.cal.start(facts());
        RunOptions opt;
        opt.stopWhen = [](CalibRig& r) { return r.cal.pushReady(); };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
        // pushReady() reads the motors LIVE; status() is the snapshot of the LAST tick, which ran
        // before the plant's final stopping step — one more tick and the two agree.
        rig.cal.tick();
        CHECK(rig.cal.pushReady());
        CHECK(rig.cal.status().movedNow == 10);
        CHECK(rig.cal.status().pushFarEnough);
        CHECK(rig.cal.status().atRest);
        rig.cal.answer(Answer::Done);
        CHECK(rig.cal.phase() == Phase::PushCapture);
        CHECK(rig.cal.accepts(Answer::FrontFirst));
        CHECK_FALSE(rig.cal.pushReady());  // false outside the push-watching step
    }
}

// ═══ 30. No percent sign can reach the screen ═══════════════════════════════════════
// Bug caught: a "%%" in any format string — the only way this class could print a percent sign,
// which PROS re-reads as a format with no argument behind it. The spin-disagreement refusal had
// two until 2F-1. This reads the SOURCE, so a refusal no case reaches is covered too.
TEST_CASE("DrivetrainCalibrator 30: no format in the calibrator can print a percent sign") {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/include/shulib/calibration/drivetrain_calibrator.hpp"};
    REQUIRE(in.good());
    std::stringstream text;
    text << in.rdbuf();
    const std::string src = text.str();
    REQUIRE(src.size() > 10000);
    CHECK(src.find("%%") == std::string::npos);
    // The names every refusal code is logged by are distinct and never the fallback.
    std::string seen;
    for (int r = 0; r <= static_cast<int>(Refusal::InvalidDescription); ++r) {
        const std::string name = refusalName(static_cast<Refusal>(r));
        CAPTURE(r);
        CHECK(name != "?");
        CHECK(seen.find("|" + name + "|") == std::string::npos);
        seen += "|" + name + "|";
    }
}

// ═══ 31. The serial record ══════════════════════════════════════════════════════════
// Bug caught: a run that leaves nothing behind to explain it — robot two's start B stopped at port
// 18 and nobody can say what ports 11–17 did, because the program printed no per-pulse numbers.
TEST_CASE("DrivetrainCalibrator 31: the record holds one line per pushed port, per pulse (every "
          "level) and per spin direction, ASCII and percent-free") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.setFault(0, 0, MemberFault::Reversed);
    rig.cal.start(facts());
    REQUIRE(runCalibration(rig, RunOptions{}) == Phase::Done);
    const auto& rep = rig.cal.report();
    char line[160];
    for (std::size_t i = 0; i < rep.portCount; ++i) {
        (void)shulib::calibration::formatPushRecord(rep.ports[i], line, sizeof line);
        CAPTURE(std::string{line});
        CHECK(screenSafe(line));
        CHECK(std::strstr(line, "push port ") == line);
    }
    (void)shulib::calibration::formatPushRecord(rep.ports[0], line, sizeof line);
    CHECK(std::string{line}.find("push port 11 travel -") == 0);  // reversed: read negative
    CHECK(std::strstr(line, "MOVED sign -1") != nullptr);
    (void)shulib::calibration::formatPushRecord(rep.ports[10], line, sizeof line);
    CHECK(std::strcmp(line, "push port 5 travel +0.0 deg still sign +0") == 0);

    REQUIRE(rep.pulseCount == 30);
    for (std::size_t k = 0; k < rep.pulseCount; ++k) {
        (void)shulib::calibration::formatPulseRecord(rep.pulses[k], rep.drivePortCount, line,
                                                     sizeof line);
        CAPTURE(std::string{line});
        CHECK(screenSafe(line));
    }
    (void)shulib::calibration::formatPulseRecord(rep.pulses[2], rep.drivePortCount, line,
                                                 sizeof line);
    CHECK(std::string{line}.find("pulse port 11 (1 of 10) 9.0 V yaw -") == 0);
    CHECK(std::strstr(line, ": LEFT") != nullptr);
    (void)shulib::calibration::formatPulseRecord(rep.pulses[0], rep.drivePortCount, line,
                                                 sizeof line);
    CHECK(std::strcmp(line, "pulse port 11 (1 of 10) 3.0 V yaw +0.00 deg own +0.0 deg others "
                            "0.0 deg: under the floor -- next level")
          == 0);

    REQUIRE(rep.spinCount == 2);
    for (std::size_t k = 0; k < 2; ++k) {
        (void)shulib::calibration::formatSpinRecord(rep.spins[k], line, sizeof line);
        CAPTURE(std::string{line});
        CHECK(screenSafe(line));
        CHECK(std::strstr(line, "W 12.500 in") != nullptr);
    }
    (void)shulib::calibration::formatSpinRecord(rep.spins[0], line, sizeof line);
    CHECK(std::string{line}.find("spin CCW 3.0 V yaw +7") == 0);  // two turns: +720 deg and a bit
}

// ═══ 32. RUN AGAIN ══════════════════════════════════════════════════════════════════
// Bug caught: state carried from a stopped run into the next — a sign, a side, a refusal code, a
// report row. The header's rule: RUN AGAIN is a NEW calibrator, built in place.
TEST_CASE("DrivetrainCalibrator 32: a calibrator rebuilt in place after a refused run carries "
          "nothing over and finishes") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    std::optional<DrivetrainCalibrator> cal;
    cal.emplace(rig.candidateSpan(), rig.portSpan(), rig.h.imu(), rig.h.clock());
    cal->start(facts());
    // The first run is stopped the way two of robot two's were: DONE touched before any push.
    for (int t = 0; t < 100 && cal->phase() != Phase::PushCapture; ++t) {
        cal->tick();
        rig.h.plant().step(Time{kDt});
    }
    cal->answer(Answer::Done);
    requireRefusal(rig, *cal, Refusal::PushTooFew, 0);

    // RUN AGAIN: exactly what the header tells a program to do.
    cal.emplace(rig.candidateSpan(), rig.portSpan(), rig.h.imu(), rig.h.clock());
    CHECK(cal->phase() == Phase::Idle);
    CHECK(cal->refusal() == Refusal::None);
    CHECK(std::strlen(cal->reason()) == 0);
    CHECK(cal->report().drivePortCount == 0);
    cal->start(facts());
    REQUIRE_MESSAGE(runWith(rig, *cal, RunOptions{}) == Phase::Done, std::string{cal->reason()});
    CHECK(cal->refusal() == Refusal::None);
    CHECK(cal->report().pulseCount == 30);
    CHECK(cal->result().left.count == 5);
    CHECK(cal->result().right.count == 5);
}

// ═══ 33. The numbers a panel reads ══════════════════════════════════════════════════
// Bug caught: status numbers that lag, lie or never move — the panel would then show a count that
// is not the one the gate uses, or a pulse ordinal that skips.
TEST_CASE("DrivetrainCalibrator 33: the status numbers track the push, the pulses and the spin") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.cal.start(facts());
    CHECK(rig.cal.status().candidates == 12);
    int maxOrdinal = 0;
    bool sawPulsing = false;
    bool sawLevel9 = false;
    bool sawCw = false;
    double maxTurns = 0.0;
    double maxMoved = 0.0;
    RunOptions opt;
    opt.hook = [&](CalibRig& r, const Script&) {
        const auto& st = r.cal.status();
        if (r.cal.phase() == Phase::PushCapture && r.cal.accepts(Answer::Done)) {
            maxMoved = std::max(maxMoved, static_cast<double>(st.movedNow));
        }
        if (r.cal.phase() == Phase::SideAssign) {
            CHECK(st.pulseTotal == 10);
            CHECK(st.pulseOrdinal >= 1);
            CHECK(st.pulseOrdinal >= maxOrdinal);  // never goes backward
            maxOrdinal = st.pulseOrdinal;
            CHECK(st.pulsePort == r.ports[static_cast<std::size_t>(st.pulseOrdinal - 1)]);
            sawPulsing = sawPulsing || st.pulsing;
            sawLevel9 = sawLevel9 || st.pulseLevelVolts == 9.0;
            CHECK(static_cast<int>(st.leftCount + st.rightCount) == st.pulseOrdinal - 1);
        }
        if (st.spinDirection < 0.0) {
            sawCw = true;
        }
        maxTurns = std::max(maxTurns, st.spinTurns);
    };
    REQUIRE(runCalibration(rig, opt) == Phase::Done);
    CHECK(maxMoved == 10.0);
    CHECK(maxOrdinal == 10);
    CHECK(sawPulsing);
    CHECK(sawLevel9);
    CHECK(sawCw);
    CHECK(maxTurns > 1.9);
    CHECK(maxTurns < 2.1);
    CHECK(rig.cal.status().leftCount == 5);
    CHECK(rig.cal.status().rightCount == 5);
    CHECK(std::abs(rig.cal.status().driftRad) < 1e-9);  // the push's straightness, as captured
    CHECK(std::strlen(rig.cal.status().prompt) <= 44);
}
