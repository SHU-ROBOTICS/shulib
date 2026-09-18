// calibration::DrivetrainCalibrator against the A2 plant (chunk R3d Part 1, brief §5 tests 4–17).
// NOTHING HERE HAS RUN ON A ROBOT. Every "push" is the harness commanding the plant's primaries
// while the calibrator only observes; every "pulse" and "spin" is the calibrator commanding the
// plant through the coupled-train model. Each case names the bug it catches and the mutation that
// must turn it red:
//    4  push capture: ten drive ports found, the two REVERSED ones with sign −1, the two motors
//       that never moved excluded                          (threshold → 0; the BackFirst inversion)
//    4b the movedThreshold attacked from BOTH sides — a push just over the line is captured, the
//       same push just under it is refused
//    5  the same push answered BACK FIRST inverts every sign
//    6  side assignment: the yaw's sign names the side, the sides are the plant's wheels exactly,
//       and a REVERSED member driven at its captured sign advances POSITIVE on its own encoder
//                                                              (swap the yaw-sign → side rule)
//    7  a member whose reading contradicts its command is REFUSED, naming the port
//                                                              (drop the own-encoder check)
//    8  a motor that never moved on the push creeping during a pulse is refused (drop the check)
//    9  wheels up — the harness holds the heading still — refuses at the yaw floor, saying "floor,
//       not blocks?"; the floor attacked from both sides               (floor → 0)
//   10  the scale: each side's ipr is the plant's own radius to 1e-9; a FROZEN member leaves the
//       median untouched and shows up in the report's spread          (median → mean)
//   11  a push that curves is refused at the straightness bound        (bound → infinity)
//   12  the spin: W both ways within 1e-6 of the true 12.5 in, their mean exact to 1e-9; Done;
//       and the result's own geometry drives a DriveEncoderOdometry that tracks the plant's truth
//       through a figure-eight                     (the W formula's sign; ΔL + ΔR; one direction)
//   13  under hostility: turn scrub makes the EFFECTIVE width LARGER than the true one (T8) and
//       within 8 %; four seeds reproduce the scale and the width inside the stated tolerances
//   14  a fight during the spin aborts naming the member, with every motor at 0 V + Coast
//   15  timeouts: an IMU that never readies; a spin that never turns
//   16  a missed tick while spinning stops the motors and refuses
//   17  abort() from every phase leaves the safe state, and result() is then a precondition breach
//   18  the spin-agreement bound attacked from both sides
//
// The plant's per-wheel STATIC-FRICTION dead band is set to zero (kS = 0) for these runs, and that
// is a deliberate, stated choice: under the coupled-train model a single-member pulse delivers one
// fifth of the pulse voltage to the WHEEL, and the tree's usual kS = 1.2 would swallow it — a
// property of the plant's per-WHEEL dead band, not of the calibrator (five motors pulling one
// train do not face five times one motor's static friction). One case below runs the whole
// procedure with kS = 1.2 and a proportionally higher ceiling, so nothing here depends on kS = 0.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <span>
#include <string>

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
using shulib::calibration::kReproScaleTolerance;
using shulib::calibration::kReproWidthTolerance;
using shulib::calibration::Phase;
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

namespace {

constexpr double kDt = 0.01;
constexpr double kTrackWidth = 12.5;   // the plant's TRUE track width (TankKinematics)
constexpr double kKv = 0.17;           // volts per in/s of wheel surface speed
constexpr double kPlantRadius = 3.25 / 2.0;  // DrivePlantConfig::driveWheelDiameter / 2, 1:1

/// The plant these runs use: memoryless (kA = 0) and with NO static-friction dead band, for the
/// reason in the file header.
[[nodiscard]] SimHarnessConfig calibPlant(double kS = 0.0) {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = kS, .kV = kKv, .kA = 0.0};
    return cfg;
}

/// The human facts a person states before a run.
[[nodiscard]] shulib::calibration::HumanFacts facts() {
    shulib::calibration::HumanFacts f;
    f.kind = shulib::hal::DrivetrainKind::Tank;
    f.cartridge = MotorCartridge::Blue;
    f.imuPort = 2;
    f.provenance = "host test";
    return f;
}

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

/// Robot two's shape on the plant: two wheels, FIVE coupled members each (ports 11–15 and 16–20),
/// plus two motors that are not on the drivetrain at all (ports 5 and 6) — the candidates a
/// registry would hand a real calibrator and which must be excluded by the push, not by a list.
struct CalibRig {
    static constexpr std::size_t kPerWheel = 5;
    static constexpr std::size_t kIdleCount = 2;
    static constexpr std::size_t kCandidates = 2 * kPerWheel + kIdleCount;

    TankKinematics kin{Length{kTrackWidth}};
    SimHarness h;
    std::array<FakeMotor, 2 * (kPerWheel - 1)> extras{};
    std::array<FakeMotor, kIdleCount> idle{};
    std::array<IMotor*, kCandidates> candidates{};
    std::array<std::int8_t, kCandidates> ports{};
    bool primaryReversed[2] = {false, false};
    DrivetrainCalibrator cal;

    explicit CalibRig(const CalibratorConfig& ccfg = {},
                      const SimHarnessConfig& pcfg = calibPlant(),
                      shulib::sim::DegradationModel* degradation = nullptr)
        : h{kin, pcfg, nullptr, degradation},
          cal{wire(), std::span<const std::int8_t>{ports.data(), ports.size()}, h.imu(), h.clock(),
              ccfg} {}

    [[nodiscard]] FakeMotor& member(int wheel, std::size_t k) {
        const auto w = static_cast<std::size_t>(wheel);
        return k == 0 ? h.motor(wheel) : extras[w * (kPerWheel - 1) + (k - 1)];
    }
    /// The candidate INDEX of wheel `wheel`'s member `k` — the index the report and the ports
    /// array are laid out by.
    [[nodiscard]] static std::size_t indexOf(int wheel, std::size_t k) {
        return static_cast<std::size_t>(wheel) * kPerWheel + k;
    }

    void setFault(int wheel, int m, MemberFault f) {
        h.plant().setMemberFault(wheel, m, f);
        if (m == 0 && f == MemberFault::Reversed) {
            primaryReversed[wheel] = true;
        }
    }

    /// A HAND PUSH, modelled: the harness commands each wheel's PRIMARY so the WHEEL rolls at
    /// `leftVolts` / `rightVolts` under the plant's coupled-train mean; the four mates free-wheel
    /// and the calibrator commands nothing at all. A wheel whose primary is wired backwards is
    /// commanded the other way round, because a hand push rolls a wheel forward whatever the
    /// wiring behind it.
    void pushPrimaries(double leftVolts, double rightVolts) {
        const double want[2] = {leftVolts, rightVolts};
        for (int w = 0; w < 2; ++w) {
            const double primary =
                static_cast<double>(kPerWheel) * want[static_cast<std::size_t>(w)];
            h.motor(w).setVoltage(Voltage{primaryReversed[w] ? -primary : primary});
        }
    }
    void stopPush() { pushPrimaries(0.0, 0.0); }

    /// The plant's TRUE forward speed of the robot centre (post-slip) — what a tape measures.
    [[nodiscard]] double trueForwardSpeed() const { return h.trueBodyTwist().vx().value(); }

private:
    /// Fill the candidate/port arrays and tell the plant about the coupled members. Called from
    /// the member-init list, where everything it touches is already constructed and `cal` is not.
    std::span<IMotor* const> wire() {
        const std::int8_t portNumbers[kCandidates] = {11, 12, 13, 14, 15, 16,
                                                      17, 18, 19, 20, 5,  6};
        for (std::size_t i = 0; i < kCandidates; ++i) {
            ports[i] = portNumbers[i];
        }
        for (int w = 0; w < 2; ++w) {
            const auto wi = static_cast<std::size_t>(w);
            std::array<FakeMotor*, kPerWheel - 1> ex{};
            candidates[indexOf(w, 0)] = &h.motor(w);
            for (std::size_t k = 1; k < kPerWheel; ++k) {
                FakeMotor& f = extras[wi * (kPerWheel - 1) + (k - 1)];
                candidates[indexOf(w, k)] = &f;
                ex[k - 1] = &f;
            }
            h.plant().attachCoupledMembers(w, std::span<FakeMotor* const>{ex.data(), ex.size()});
        }
        for (std::size_t j = 0; j < kIdleCount; ++j) {
            candidates[2 * kPerWheel + j] = &idle[j];
        }
        return std::span<IMotor* const>{candidates.data(), candidates.size()};
    }
};

/// How one scripted run behaves.
struct RunOptions {
    double signPushInches = 8.0;   ///< how far the sign-capture push travels
    /// WHEEL volts during the pushes. The harness drives the PRIMARY at five times this to make
    /// the coupled-train mean come out here, and a FakeMotor clamps at 12 V — so anything above
    /// 2.4 would clamp, and (the way this test first went wrong) a CURVED push would silently
    /// straighten out because both sides clamped to the same 12 V.
    double signPushVolts = 2.0;
    Answer front = Answer::FrontFirst;
    double tapeInches = 48.0;      ///< how far the tape push travels, TRUE
    Answer distance = Answer::Distance48;  ///< what the operator answers
    double tapeVolts = 2.0;        ///< wheel volts during the tape push (see signPushVolts)
    double curve = 1.0;            ///< right-side volts relative to left during the tape push
    /// Which way the operator pushes along the tape. The calibrator requires the tape push to
    /// roll both sides FORWARD in the robot's own frame, and a BACK-FIRST sign capture makes
    /// "forward" the other direction on the plant.
    double tapeDirection = 1.0;
    double plantKs = 0.0;          ///< the plant's static-friction dead band, for the exact stop
    int maxTicks = 12000;
    std::function<void(CalibRig&, int)> hook = nullptr;  ///< per-tick fault injection
};

/// Drive a whole calibration: the operator pushes when asked, answers when asked, and stands clear
/// while the calibrator drives. Returns the phase the run ended in.
Phase runCalibration(CalibRig& rig, const RunOptions& opt) {
    int stage = 0;
    double pushed = 0.0;
    // The plant's transfer, both ways: what a wheel command produces, and what command produces a
    // wanted speed. Used to land the tape push on EXACTLY the answered distance.
    const double ks = opt.plantKs;
    auto speedFor = [ks](double volts) {
        return std::abs(volts) <= ks ? 0.0 : (std::abs(volts) - ks) / kKv;
    };
    auto voltsFor = [ks](double speed) { return speed * kKv + ks; };
    for (int t = 0; t < opt.maxTicks; ++t) {
        Phase p = rig.cal.phase();
        if (p == Phase::Done || p == Phase::Aborted) {
            break;
        }
        if (opt.hook) {
            opt.hook(rig, t);
            // A hook may abort the run; what follows must not then command a motor and undo the
            // safe state the abort just established.
            p = rig.cal.phase();
            if (p == Phase::Done || p == Phase::Aborted) {
                break;
            }
        }
        bool pushing = false;
        if (p == Phase::PushCapture) {
            if (stage == 0) {
                if (pushed < opt.signPushInches) {
                    rig.pushPrimaries(opt.signPushVolts, opt.signPushVolts);
                    pushing = true;
                } else {
                    rig.stopPush();
                    stage = 1;
                }
            } else if (stage == 1) {
                rig.cal.answer(Answer::Done);
                stage = 2;
            } else if (stage == 2) {
                rig.cal.answer(opt.front);
                stage = 3;
                pushed = 0.0;
            }
        } else if (p == Phase::ScalePush) {
            if (stage == 3) {
                const double remaining = opt.tapeInches - pushed;
                if (remaining > 1e-10) {
                    double v = opt.tapeVolts;
                    // With a straight push the last tick is trimmed so the TRUE travel lands on
                    // the answered distance exactly — otherwise the scale would be off by the
                    // overshoot and could not be compared with the plant's own radius.
                    if (opt.curve == 1.0 && speedFor(v) * kDt > remaining) {
                        v = voltsFor(remaining / kDt);
                    }
                    rig.pushPrimaries(v * opt.tapeDirection, v * opt.curve * opt.tapeDirection);
                    pushing = true;
                } else {
                    rig.stopPush();
                    stage = 4;
                }
            } else if (stage == 4) {
                rig.cal.answer(Answer::Done);
                stage = 5;
            } else if (stage == 5) {
                rig.cal.answer(opt.distance);
                stage = 6;
            }
        }
        rig.cal.tick();
        rig.h.plant().step(Time{kDt});
        if (pushing) {
            pushed += std::abs(rig.trueForwardSpeed()) * kDt;
        }
    }
    return rig.cal.phase();
}

/// Every candidate at 0 V and coasting — the state every refusal and every abort must leave.
[[nodiscard]] bool safeStateHolds(CalibRig& rig) {
    for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
        if (rig.candidates[i]->commandedVoltage().value() != 0.0) {
            return false;
        }
        if (rig.candidates[i]->brakeMode() != BrakeMode::Coast) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] bool reasonMentions(const DrivetrainCalibrator& cal, const char* text) {
    return std::strstr(cal.reason(), text) != nullptr;
}

/// A degradation model that holds the reported HEADING still. Landmine 5, stated: this is the
/// HARNESS freezing the IMU, not the plant modelling a robot up on blocks — the plant has no
/// notion of a wheel off the floor. What the calibrator sees is exactly what wheels-up looks
/// like from inside: motors turning, heading unchanged.
struct HeldHeading final : shulib::sim::DegradationModel {
    [[nodiscard]] Angle imuHeading(Angle, Time, shulib::sim::Rng&) override { return Angle{}; }
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

}  // namespace

// ═══ 4. Push capture ════════════════════════════════════════════════════════════════
// Bug caught: MOTOR WATCH's rule not surviving the move into the library — a motor that never
// moved counted as a drive port (the threshold), or a backwards-wired motor's sign lost.
TEST_CASE("DrivetrainCalibrator 4: a whole-robot push finds exactly the ten drive ports, signs the "
          "two REVERSED ones −1, and never touches the two motors that did not move") {
    CalibRig rig;
    rig.setFault(0, 0, MemberFault::Reversed);  // LEFT primary, port 11, wired backwards
    rig.setFault(1, 3, MemberFault::Reversed);  // RIGHT extra, port 19, wired backwards
    rig.cal.start(facts());
    const Phase end = runCalibration(rig, RunOptions{});
    REQUIRE(end == Phase::Done);

    const auto& rep = rig.cal.report();
    CHECK(rep.portCount == 12);
    CHECK(rep.drivePortCount == 10);
    CHECK_FALSE(rep.backFirst);
    for (int w = 0; w < 2; ++w) {
        for (std::size_t k = 0; k < CalibRig::kPerWheel; ++k) {
            const std::size_t i = CalibRig::indexOf(w, k);
            CAPTURE(rig.ports[i]);
            const bool reversed = (w == 0 && k == 0) || (w == 1 && k == 3);
            CHECK(rep.ports[i].sign == (reversed ? -1 : +1));
            CHECK(std::abs(rep.ports[i].pushDeltaRad) > 0.5);
        }
    }
    // The two motors that are not on the drivetrain: no sign, no side, no travel — and they were
    // excluded by the PUSH, not by anybody's list.
    for (std::size_t j = 2 * CalibRig::kPerWheel; j < CalibRig::kCandidates; ++j) {
        CHECK(rep.ports[j].sign == 0);
        CHECK(rep.ports[j].side == 0);
        CHECK(rep.ports[j].pushDeltaRad == 0.0);
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
}

TEST_CASE("DrivetrainCalibrator 4b: movedThreshold from BOTH sides — the same push is captured "
          "just under the line and refused just over it") {
    // A 0.2 in push turns each shaft by 0.2 / 1.625 = 0.1231 rad. Bracket it.
    const double expected = 0.2 / kPlantRadius;
    for (int over = 0; over < 2; ++over) {
        CAPTURE(over);
        CalibratorConfig cfg;
        cfg.movedThreshold =
            shulib::units::AngleDim{over != 0 ? expected * 1.05 : expected * 0.95};
        CalibRig rig{cfg};
        rig.cal.start(facts());
        RunOptions opt;
        opt.signPushInches = 0.2;
        opt.signPushVolts = 0.5;
        (void)runCalibration(rig, opt);
        if (over != 0) {
            CHECK(rig.cal.phase() == Phase::Aborted);
            CHECK(reasonMentions(rig.cal, "the push moved 0 port(s)"));
            CHECK(safeStateHolds(rig));
        } else {
            CHECK(rig.cal.phase() == Phase::Done);
            CHECK(rig.cal.report().drivePortCount == 10);
        }
    }
}

// ═══ 5. The front is what the operator says ═════════════════════════════════════════
// Bug caught: assuming the push direction. A BACK-first push captures every sign backwards, and
// no later step could ever see it — every member would agree with every other, all the wrong way.
TEST_CASE("DrivetrainCalibrator 5: the same push answered BACK FIRST inverts every captured sign") {
    CalibRig front;
    front.setFault(0, 0, MemberFault::Reversed);
    front.cal.start(facts());
    REQUIRE(runCalibration(front, RunOptions{}) == Phase::Done);

    CalibRig back;
    back.setFault(0, 0, MemberFault::Reversed);
    back.cal.start(facts());
    RunOptions opt;
    opt.front = Answer::BackFirst;
    // BACK-FIRST makes the robot's own "forward" the other direction on the plant, and the tape
    // push must roll both sides FORWARD in the robot's frame — so the operator pushes the other
    // way. That is not an artifact of the test: it is what "the front is what you say it is"
    // means once it has been said.
    opt.tapeDirection = -1.0;
    const Phase end = runCalibration(back, opt);

    CHECK(back.cal.report().backFirst);
    for (std::size_t i = 0; i < 2 * CalibRig::kPerWheel; ++i) {
        CAPTURE(back.ports[i]);
        CHECK(back.cal.report().ports[i].sign == -front.cal.report().ports[i].sign);
        CHECK(back.cal.report().ports[i].sign != 0);
    }
    // A back-first push also swaps which side is which — the robot is the same, the FRONT is the
    // other end, so what was the left side is now the right. The run is refused only if the
    // pulses then contradict; with every sign consistently inverted they do not, and the
    // description is a valid description OF A ROBOT FACING THE OTHER WAY.
    REQUIRE(end == Phase::Done);
    CHECK(back.cal.result().signedPortOf(11) == 11);   // was −11 front-first
    CHECK(back.cal.result().signedPortOf(12) == -12);  // was +12
    CHECK(back.cal.result().sideOf(11) == -back.cal.result().sideOf(20));
}

// ═══ 6. Side assignment from the IMU's yaw sign ═════════════════════════════════════
// Bug caught: the yaw-sign → side rule backwards (every side flipped, and the robot would then
// turn the wrong way for every commanded heading), or a REVERSED member read as a contradiction.
TEST_CASE("DrivetrainCalibrator 6: LEFT pulses turn the robot CLOCKWISE (negative yaw) and RIGHT "
          "pulses counter-clockwise; the sides are the plant's wheels exactly") {
    CalibRig rig;
    rig.setFault(0, 0, MemberFault::Reversed);
    rig.setFault(1, 3, MemberFault::Reversed);
    rig.cal.start(facts());
    REQUIRE(runCalibration(rig, RunOptions{}) == Phase::Done);

    const auto& rep = rig.cal.report();
    for (std::size_t k = 0; k < CalibRig::kPerWheel; ++k) {
        const std::size_t l = CalibRig::indexOf(0, k);
        const std::size_t r = CalibRig::indexOf(1, k);
        CAPTURE(k);
        CHECK(rep.ports[l].pulseYawRad < 0.0);   // LEFT forward → CW → NEGATIVE canonical yaw
        CHECK(rep.ports[r].pulseYawRad > 0.0);   // RIGHT forward → CCW → positive
        CHECK(rep.ports[l].side == -1);
        CHECK(rep.ports[r].side == +1);
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
    // And the two REVERSED members' pulses turned the robot the SAME way as their side-mates —
    // the sign was applied to the command, once.
    CHECK(rep.ports[CalibRig::indexOf(0, 0)].pulseYawRad < 0.0);
    CHECK(rep.ports[CalibRig::indexOf(1, 3)].pulseYawRad > 0.0);
}

// ═══ 7. A contradiction is refused, naming the port ═════════════════════════════════
// Bug caught: a member whose reading disagrees with its command being averaged into a side. A
// SignFlipped member reads backwards while its command still drives the train the right way, so
// the push captures −1 for it and the pulse then drives the train BACKWARD — the one case where
// the push and the pulse cannot both be believed.
TEST_CASE("DrivetrainCalibrator 7: a member whose encoder contradicts its own pulse refuses the "
          "run, naming the port") {
    CalibRig rig;
    rig.setFault(1, 2, MemberFault::SignFlipped);  // port 18 reads backwards, drives forwards
    rig.cal.start(facts());
    const Phase end = runCalibration(rig, RunOptions{});
    CHECK(end == Phase::Aborted);
    CHECK(reasonMentions(rig.cal, "port 18"));
    CHECK(reasonMentions(rig.cal, "contradict"));
    CHECK(safeStateHolds(rig));
    CHECK_THROWS_AS((void)rig.cal.result(), PreconditionError);
    // The report still carries what was learned before the refusal — a refusal is not an amnesia.
    CHECK(rig.cal.report().drivePortCount == 10);
    CHECK(rig.cal.report().ports[CalibRig::indexOf(1, 2)].sign == -1);
    // And it refused AT THE PULSE, before anything downstream was measured: no scale, no width.
    // (Without the cross-check the contradiction is still caught — by the fight detector, a whole
    // phase later, after a wrong scale has been taken. Refusing early is the point.)
    CHECK(rig.cal.report().leftInchesPerRadian == 0.0);
    CHECK(rig.cal.report().rightInchesPerRadian == 0.0);
    CHECK(rig.cal.report().widthCcwIn == 0.0);
    CHECK(rig.cal.report().pushDistanceIn == 0.0);
}

// ═══ 8. Everything else must be still ═══════════════════════════════════════════════
// Bug caught: a second thing being driven while one member is pulsed — the yaw would then be a
// sum of two motions and the side assignment a coin toss. Note what is NOT checked: the pulsed
// member's own side-mates, which are BACK-DRIVEN through the gear train and must turn.
TEST_CASE("DrivetrainCalibrator 8: a motor that did not move on the push, creeping during a "
          "pulse, refuses the run") {
    CalibRig rig;
    rig.cal.start(facts());
    RunOptions opt;
    opt.hook = [](CalibRig& r, int) {
        // Only once a PULSE is actually driving something: creeping during the rest window would
        // be caught by the rest check instead, which is a different (also correct) refusal.
        const bool pulsing = std::abs(r.h.plant().trueWheelSpin()[0].value()) > 0.5
                             || std::abs(r.h.plant().trueWheelSpin()[1].value()) > 0.5;
        if (r.cal.phase() == Phase::SideAssign && pulsing) {
            r.idle[1].setVelocity(AngularVelocity{3.0});  // port 6 starts turning
        }
    };
    const Phase end = runCalibration(rig, opt);
    CHECK(end == Phase::Aborted);
    CHECK(reasonMentions(rig.cal, "port 6"));
    CHECK(reasonMentions(rig.cal, "not a drive motor"));
    CHECK(safeStateHolds(rig));

    // The CONTROL: the same run with the side-mates turning (which they always do) is fine — the
    // check must not fire on the coupled train it is standing next to.
    CalibRig ok;
    ok.cal.start(facts());
    CHECK(runCalibration(ok, RunOptions{}) == Phase::Done);
}

// ═══ 9. Wheels up ═══════════════════════════════════════════════════════════════════
// Bug caught: a calibration run on a robot up on blocks producing a confident, meaningless
// description. Every pulse turns a motor and nothing turns the robot, so the side assignment
// would be whatever noise the IMU happened to have.
TEST_CASE("DrivetrainCalibrator 9: with the heading held still the run refuses at the yaw floor, "
          "and says floor-not-blocks; the floor attacked from both sides") {
    HeldHeading held;
    CalibRig rig{CalibratorConfig{}, calibPlant(), &held};
    rig.cal.start(facts());
    const Phase end = runCalibration(rig, RunOptions{});
    CHECK(end == Phase::Aborted);
    CHECK(reasonMentions(rig.cal, "FLOOR"));
    CHECK(reasonMentions(rig.cal, "blocks"));
    CHECK(reasonMentions(rig.cal, "port 11"));  // the first port pulsed
    CHECK(safeStateHolds(rig));

    // BOTH SIDES of the floor, on the real plant: one pulse turns the robot by a known amount,
    // so a floor just under it passes and a floor just over it refuses.
    CalibRig measure;
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
        CalibRig rig2{cfg};
        rig2.cal.start(facts());
        (void)runCalibration(rig2, RunOptions{});
        CHECK(rig2.cal.phase() == (over != 0 ? Phase::Aborted : Phase::Done));
        if (over != 0) {
            CHECK(reasonMentions(rig2.cal, "blocks"));
        }
    }
}

// ═══ 10. The distance scale ═════════════════════════════════════════════════════════
// Bug caught: the MEAN where the median belongs. One member of five reading short (a dead port
// holding its last value) drags a mean by a fifth of the shortfall and every distance the robot
// ever drives is wrong by that much, silently. The median cannot be moved by one of five.
TEST_CASE("DrivetrainCalibrator 10: each side's scale is the plant's own radius to 1e-9, and a "
          "FROZEN member leaves the median untouched while showing in the report's spread") {
    CalibRig clean;
    clean.cal.start(facts());
    REQUIRE(runCalibration(clean, RunOptions{}) == Phase::Done);
    const auto& cleanRep = clean.cal.report();
    CHECK(cleanRep.pushDistanceIn == 48.0);
    CHECK(cleanRep.leftInchesPerRadian == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(cleanRep.rightInchesPerRadian == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(clean.cal.result().left.inchesPerRadian.value()
          == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(clean.cal.result().right.inchesPerRadian.value()
          == doctest::Approx(kPlantRadius).epsilon(1e-9));
    CHECK(cleanRep.leftSpreadRad == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(std::abs(cleanRep.pushStraightnessRad) < 1e-9);

    // Now freeze one LEFT member part-way through the tape push: its shaft holds, so its travel
    // comes up short — one of five, which the median must ignore entirely.
    CalibRig frozen;
    frozen.cal.start(facts());
    RunOptions opt;
    opt.hook = [](CalibRig& r, int) {
        if (r.cal.phase() == Phase::ScalePush && r.cal.report().leftInchesPerRadian == 0.0
            && r.trueForwardSpeed() > 1.0) {
            r.setFault(0, 2, MemberFault::Frozen);  // port 13 stops reporting mid-push
        }
    };
    (void)runCalibration(frozen, opt);
    const auto& rep = frozen.cal.report();
    REQUIRE(rep.leftInchesPerRadian > 0.0);  // the scale was reached
    CHECK(rep.leftInchesPerRadian == doctest::Approx(cleanRep.leftInchesPerRadian).epsilon(1e-9));
    CHECK(rep.rightInchesPerRadian == doctest::Approx(cleanRep.rightInchesPerRadian).epsilon(1e-9));
    // The shortfall is VISIBLE, not hidden: the report shows the side's spread, on the left only.
    CHECK(rep.leftSpreadRad > 1.0);
    CHECK(rep.rightSpreadRad == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(rep.ports[CalibRig::indexOf(0, 2)].tapeDeltaRad
          < rep.ports[CalibRig::indexOf(0, 1)].tapeDeltaRad);
    // And the frozen member is CAUGHT one step later rather than quietly riding along. It is
    // caught by the REST check, not by the fight detector, and the reason is worth recording: a
    // plant-frozen member holds its last VELOCITY as well as its last position, so it reports the
    // speed it had when it died, for ever. The drivetrain therefore never comes to rest before the
    // spin, and the refusal names the port. (The fight detector would have caught it too, one step
    // further on; the rest check gets there first, and that is the safer order — nothing is
    // powered while it fires.)
    CHECK(frozen.cal.phase() == Phase::Aborted);
    CHECK(reasonMentions(frozen.cal, "port 13"));
    CHECK(reasonMentions(frozen.cal, "still turning"));
    CHECK(safeStateHolds(frozen));
}

// ═══ 11. The push must be straight ══════════════════════════════════════════════════
// Bug caught: a curved push divided by a tape distance. The tape measures the straight line; the
// wheels measure the arc, and the outside wheel measures a longer one — two different numbers
// silently averaged into one scale.
TEST_CASE("DrivetrainCalibrator 11: a tape push that curves is refused at the straightness bound") {
    CalibRig rig;
    rig.cal.start(facts());
    RunOptions opt;
    opt.curve = 0.85;  // the right side rolls slower: the robot arcs
    const Phase end = runCalibration(rig, opt);
    CHECK(end == Phase::Aborted);
    CHECK(reasonMentions(rig.cal, "curved"));
    CHECK(reasonMentions(rig.cal, "straight"));
    CHECK(safeStateHolds(rig));
    CHECK(std::abs(rig.cal.report().pushStraightnessRad)
          > rig.cal.config().straightBound.value());
}

// ═══ 12. The effective track width, and the round trip into the consumer ════════════
// Bug caught: the W formula's sign or its terms (ΔL + ΔR instead of ΔR − ΔL would read a straight
// line as a turn), one direction taken for both, or a description whose numbers do not actually
// work in the odometry they were measured for.
TEST_CASE("DrivetrainCalibrator 12: the spin recovers the plant's true track width both ways, and "
          "the result's own geometry tracks the truth through a figure-eight") {
    CalibRig rig;
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
    CHECK(std::strstr(d.provenance, "ROBOT-MEASURED") != nullptr);
    char why[160];
    CHECK(d.valid(why, sizeof why));
    CHECK(rig.cal.status().phase == Phase::Done);
    CHECK(std::strstr(rig.cal.status().prompt, "DONE") != nullptr);

    // ── the round trip: build the graph a later program would build, from the description ──
    GraphFromDescription graph{rig, d};
    shulib::localization::DriveEncoderOdometry odom{
        rig.h.imu(),        graph.left,       graph.right, d.leftGeometry(), d.rightGeometry(),
        d.trackWidth,       rig.h.truePose()};
    shulib::control::Feedforward ff{{.kS = 0.0, .kV = kKv, .kA = 0.0}};
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
    CalibRig rig{CalibratorConfig{}, calibPlant(), &scrub};
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

    // ── reproducibility over four hostile seeds (brief §4.4) ─────────────────────────
    double scales[4] = {};
    double widths[4] = {};
    for (std::uint64_t seed = 1; seed <= 4; ++seed) {
        CAPTURE(seed);
        shulib::sim::EncoderHostileModel encoders{};
        shulib::sim::ImuHostileModel imu{};
        shulib::sim::ChainedDegradation world{{&encoders, &imu}};
        SimHarnessConfig pcfg = calibPlant();
        pcfg.plant.seed = seed;
        CalibRig hostile{CalibratorConfig{}, pcfg, &world};
        hostile.cal.start(facts());
        const Phase end = runCalibration(hostile, RunOptions{});
        REQUIRE(end == Phase::Done);
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
    // plant's exact radius, and the IMU's per-boot bias draw moved the WIDTH seed by seed (the
    // scale does not move with the seed because quantization draws no random numbers — it is a
    // deterministic grid, so four seeds give four identical scales, and that is the honest
    // reading of "reproducible", not a test that forgot to vary anything).
    CHECK(scaleLo != kPlantRadius);
    CHECK(scaleLo == doctest::Approx(kPlantRadius).epsilon(1e-3));
    CHECK(widthHi > widthLo);
}

// ═══ 14. A fight during the spin ════════════════════════════════════════════════════
// Bug caught: the shared coupled-side monitor not wired in, or a refusal that leaves five motors
// stalled against four. This is the one place a calibration can strip a gear train.
TEST_CASE("DrivetrainCalibrator 14: a member fighting its side during the spin aborts naming the "
          "port, with every motor at 0 V and coasting") {
    CalibRig rig;
    rig.cal.start(facts());
    RunOptions opt;
    opt.hook = [](CalibRig& r, int) {
        // Healthy through the push and the pulses; a fight begins as the spin does.
        if (r.cal.phase() == Phase::SpinTest) {
            r.setFault(1, 4, MemberFault::SignFlipped);  // port 20
        }
    };
    const Phase end = runCalibration(rig, opt);
    CHECK(end == Phase::Aborted);
    CHECK(reasonMentions(rig.cal, "port 20"));
    CHECK(reasonMentions(rig.cal, "RIGHT"));
    CHECK(reasonMentions(rig.cal, "fought"));
    CHECK(safeStateHolds(rig));
    CHECK(std::strstr(rig.cal.status().progress, "port 20") != nullptr);
    CHECK_THROWS_AS((void)rig.cal.result(), PreconditionError);
}

// ═══ 15. Every unbounded wait has a bound ═══════════════════════════════════════════
// Bug caught: a calibration that hangs — on a robot, with a person standing over it, waiting for
// something that will never happen.
TEST_CASE("DrivetrainCalibrator 15: an IMU that never readies, and a spin that never turns, both "
          "refuse at their configured bounds") {
    SUBCASE("the IMU never becomes ready") {
        NeverReady never;
        CalibRig rig{CalibratorConfig{}, calibPlant(), &never};
        rig.cal.start(facts());
        for (int t = 0; t < 2000 && rig.cal.phase() == Phase::WaitImu; ++t) {
            rig.cal.tick();
            rig.h.plant().step(Time{kDt});
        }
        CHECK(rig.cal.phase() == Phase::Aborted);
        CHECK(reasonMentions(rig.cal, "IMU never became ready"));
        CHECK(reasonMentions(rig.cal, "port 2"));
        CHECK(safeStateHolds(rig));
        // It waited the CONFIGURED time, not less: 5 s at 10 ms is past tick 500.
        CHECK(rig.h.clock().now().value() > rig.cal.config().imuReadyTimeout.value());
    }
    SUBCASE("the spin never reaches its turn count") {
        CalibRig rig;
        rig.cal.start(facts());
        // Run normally until the spin starts, then let TIME pass without the world moving — the
        // clock is advanced directly, which is what a robot pinned against a wall looks like.
        RunOptions opt;
        opt.maxTicks = 4000;
        opt.hook = [](CalibRig&, int) {};
        int t = 0;
        int stage = 0;
        double pushed = 0.0;
        // (a compact inline driver: the shared one runs to completion, and this case must stop)
        while (t++ < 6000 && rig.cal.phase() != Phase::SpinTest
               && rig.cal.phase() != Phase::Aborted && rig.cal.phase() != Phase::Done) {
            const Phase p = rig.cal.phase();
            bool pushing = false;
            if (p == Phase::PushCapture) {
                if (stage == 0) {
                    if (pushed < 8.0) { rig.pushPrimaries(2.0, 2.0); pushing = true; }
                    else { rig.stopPush(); stage = 1; }
                } else if (stage == 1) { rig.cal.answer(Answer::Done); stage = 2; }
                else if (stage == 2) { rig.cal.answer(Answer::FrontFirst); stage = 3; pushed = 0.0; }
            } else if (p == Phase::ScalePush) {
                if (stage == 3) {
                    if (pushed < 48.0) { rig.pushPrimaries(3.0, 3.0); pushing = true; }
                    else { rig.stopPush(); stage = 4; }
                } else if (stage == 4) { rig.cal.answer(Answer::Done); stage = 5; }
                else if (stage == 5) { rig.cal.answer(Answer::Distance48); stage = 6; }
            }
            rig.cal.tick();
            rig.h.plant().step(Time{kDt});
            if (pushing) pushed += rig.trueForwardSpeed() * kDt;
        }
        REQUIRE(rig.cal.phase() == Phase::SpinTest);
        // From here the clock advances but the world does not: the calibrator commands volts and
        // nothing turns.
        for (int i = 0; i < 4000 && rig.cal.phase() == Phase::SpinTest; ++i) {
            rig.cal.tick();
            rig.h.clock().advance(Time{kDt});
        }
        CHECK(rig.cal.phase() == Phase::Aborted);
        CHECK(safeStateHolds(rig));
        // It refused for a REASON with a number in it, at the configured bound.
        CHECK((reasonMentions(rig.cal, "spin reached only")
               || reasonMentions(rig.cal, "never came to rest")));
    }
}

// ═══ 16. A missed tick is a stop ════════════════════════════════════════════════════
// Bug caught: a caller that stops ticking leaving the motors running. The calibrator holds its
// command between ticks, so nothing else would stop them (the C2 stalled-pacer lesson).
TEST_CASE("DrivetrainCalibrator 16: a 0.2 s gap between ticks while spinning stops the motors and "
          "refuses") {
    CalibRig rig;
    rig.cal.start(facts());
    bool skipped = false;
    RunOptions opt;
    opt.hook = [&skipped](CalibRig& r, int) {
        if (!skipped && r.cal.phase() == Phase::SpinTest
            && std::abs(r.h.trueBodyTwist().omega().value()) > 1.0) {
            skipped = true;
            // The caller went away for 0.2 s — the world kept moving, nothing called tick().
            for (int i = 0; i < 20; ++i) {
                r.h.plant().step(Time{kDt});
            }
        }
    };
    const Phase end = runCalibration(rig, opt);
    CHECK(skipped);
    CHECK(end == Phase::Aborted);
    CHECK(reasonMentions(rig.cal, "a tick was missed"));
    CHECK(reasonMentions(rig.cal, "motors stopped"));
    CHECK(safeStateHolds(rig));

    // BOTH SIDES of the bound: a gap just under commandTimeout is not a missed tick.
    CalibRig fine;
    fine.cal.start(facts());
    bool nudged = false;
    RunOptions ok;
    ok.hook = [&nudged](CalibRig& r, int) {
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
    const Phase phases[] = {Phase::WaitImu, Phase::PushCapture, Phase::SideAssign,
                            Phase::ScalePush, Phase::SpinTest, Phase::Done};
    for (Phase target : phases) {
        CAPTURE(static_cast<int>(target));
        CalibRig rig;
        rig.cal.start(facts());
        bool aborted = false;
        // Put every motor into the WORST state abort() could find it in — driven, and HOLDING
        // rather than coasting — immediately before the abort. Without this the test proves
        // nothing: at the instant each phase begins, the calibrator has usually just commanded
        // the safe state itself for its own reasons, so an abort() that did nothing at all would
        // still leave the motors safe. (That is exactly what happened on the first pass here: the
        // "skip the safe state in abort()" mutation stayed GREEN. Recorded in the R3d log.)
        auto derange = [](CalibRig& r) {
            for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
                r.candidates[i]->setVoltage(Voltage{7.0});
                r.candidates[i]->setBrakeMode(BrakeMode::Hold);
            }
        };
        RunOptions opt;
        opt.hook = [&](CalibRig& r, int) {
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
        CHECK(rig.cal.phase() == Phase::Aborted);
        CHECK(std::strcmp(rig.cal.reason(), "the operator hit STOP") == 0);
        CHECK(safeStateHolds(rig));
        CHECK_THROWS_AS((void)rig.cal.result(), PreconditionError);
    }
    // Idle too — abort before anything started is still the safe state, even from a deranged one.
    CalibRig idle;
    for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
        idle.candidates[i]->setVoltage(Voltage{7.0});
        idle.candidates[i]->setBrakeMode(BrakeMode::Brake);
    }
    idle.cal.abort();
    CHECK(idle.cal.phase() == Phase::Aborted);
    CHECK(safeStateHolds(idle));
    // And the Answer::Abort route is the same route.
    CalibRig viaAnswer;
    viaAnswer.cal.start(facts());
    for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
        viaAnswer.candidates[i]->setVoltage(Voltage{7.0});
        viaAnswer.candidates[i]->setBrakeMode(BrakeMode::Brake);
    }
    viaAnswer.cal.answer(Answer::Abort);
    CHECK(viaAnswer.cal.phase() == Phase::Aborted);
    CHECK(safeStateHolds(viaAnswer));

    // A REFUSAL is the other route into the safe state, and unlike an abort it fires while the
    // calibrator itself is driving: the fight detector refuses mid-spin, with ±3 V on ten motors.
    CalibRig fight;
    fight.cal.start(facts());
    bool sawVolts = false;
    RunOptions spinOpt;
    spinOpt.hook = [&sawVolts](CalibRig& r, int) {
        if (r.cal.phase() == Phase::SpinTest) {
            r.setFault(1, 4, MemberFault::SignFlipped);
            if (std::abs(r.candidates[CalibRig::indexOf(0, 0)]->commandedVoltage().value()) > 1.0) {
                sawVolts = true;  // the calibrator really was commanding when it refused
            }
        }
    };
    CHECK(runCalibration(fight, spinOpt) == Phase::Aborted);
    CHECK(sawVolts);
    CHECK(safeStateHolds(fight));
}

// ═══ 18. The spin-agreement bound, from both sides ══════════════════════════════════
// Bug caught: the two directions accepted however far apart they are. Asymmetric scrub biases one
// direction, and averaging two numbers that disagree hides the disagreement instead of reporting
// it.
TEST_CASE("DrivetrainCalibrator 18: two spin directions that disagree are refused at the bound, "
          "and accepted just outside it") {
    for (int tight = 0; tight < 2; ++tight) {
        CAPTURE(tight);
        SecondSpinScrub scrub;
        CalibratorConfig cfg;
        cfg.spinAgreement = tight != 0 ? 0.03 : 0.20;
        CalibRig rig{cfg, calibPlant(), &scrub};
        rig.cal.start(facts());
        RunOptions opt;
        opt.hook = [&scrub](CalibRig& r, int) {
            // Switch the scrub on once the FIRST direction has produced its number.
            scrub.active = r.cal.report().widthCcwIn != 0.0;
        };
        const Phase end = runCalibration(rig, opt);
        const double disagreement = rig.cal.report().widthDisagreement;
        CAPTURE(disagreement);
        CHECK(rig.cal.report().widthCwIn > rig.cal.report().widthCcwIn);
        if (tight != 0) {
            CHECK(end == Phase::Aborted);
            CHECK(reasonMentions(rig.cal, "disagree about the track width"));
            CHECK(safeStateHolds(rig));
        } else {
            CHECK(end == Phase::Done);
            CHECK(disagreement > 0.03);   // the same disagreement, now inside the bound
            CHECK(disagreement < 0.20);
        }
    }
}

// ═══ 19. Contract breaches, and a run with a real static-friction dead band ═════════
// Bug caught: the sign applied twice (the header's rule 1 and landmine 1) going unguarded, and
// anything in the procedure secretly depending on the plant's kS = 0.
TEST_CASE("DrivetrainCalibrator 19: the caller contract is enforced, and the whole procedure runs "
          "with a real static-friction dead band") {
    CalibRig rig;
    std::array<std::int8_t, CalibRig::kCandidates> negative = rig.ports;
    negative[3] = static_cast<std::int8_t>(-negative[3]);
    CHECK_THROWS_AS((DrivetrainCalibrator{std::span<IMotor* const>{rig.candidates},
                                          std::span<const std::int8_t>{negative}, rig.h.imu(),
                                          rig.h.clock()}),
                    PreconditionError);
    std::array<std::int8_t, CalibRig::kCandidates> duplicate = rig.ports;
    duplicate[4] = duplicate[3];
    CHECK_THROWS_AS((DrivetrainCalibrator{std::span<IMotor* const>{rig.candidates},
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

    // An answer that arrives out of order refuses rather than silently doing nothing.
    CalibRig stray;
    stray.cal.start(facts());
    stray.cal.tick();
    stray.cal.answer(Answer::Distance48);  // nobody asked for a distance yet
    CHECK(stray.cal.phase() == Phase::Aborted);
    CHECK(reasonMentions(stray.cal, "48 in"));
    CHECK(reasonMentions(stray.cal, "nothing was recorded"));

    // The whole procedure, on a plant WITH a static-friction dead band (kS = 1.2, the tree's
    // usual): the pulse ceiling is raised so one member of five still clears it. Nothing in the
    // calibrator depends on the frictionless plant the other cases use.
    CalibratorConfig cfg;
    cfg.pulseVolts = Voltage{9.0};
    cfg.spinVolts = Voltage{9.0};
    CalibRig sticky{cfg, calibPlant(1.2)};
    sticky.cal.start(facts());
    RunOptions opt;
    opt.signPushVolts = 2.2;  // wheel volts; the primary carries five times this, under the clamp
    opt.tapeVolts = 2.2;
    opt.plantKs = 1.2;
    REQUIRE(runCalibration(sticky, opt) == Phase::Done);
    CHECK(sticky.cal.result().trackWidth.value() == doctest::Approx(kTrackWidth).epsilon(1e-6));
    CHECK(sticky.cal.report().leftInchesPerRadian
          == doctest::Approx(kPlantRadius).epsilon(1e-6));
}

// ── 21 — accepts(): what a panel may OFFER, pinned against what answer() actually DOES ──
//
// ADDED AT R3d PART 2, in Part 1's file, because Part 2 found the gap here (RESUMING: "a chunk
// that finds a flaw in an earlier chunk fixes it there"). answer() REFUSES an out-of-order
// answer and ends the run — right, but it makes a wrong BUTTON destructive, and Phase alone
// cannot tell a panel which answers are live (PushCapture and ScalePush are two steps each).
// accepts() is the query; THIS case is what stops it drifting from answer(), because it never
// asserts a hard-coded expectation: for every waiting step and EVERY Answer it runs the real
// answer() on a FRESH rig and requires the two to agree. A divergence in either direction —
// an answer offered that would abort, or an answer refused that would have worked — is red.
TEST_CASE("DrivetrainCalibrator 21: accepts() agrees with answer(), at every waiting step") {
    // The four waiting steps, reached by the same scripted operator the other cases use.
    enum class Stop { PushWatching, PushAwaitingFront, ScaleWatching, ScaleAwaitingDistance };
    const Stop stops[] = {Stop::PushWatching, Stop::PushAwaitingFront, Stop::ScaleWatching,
                          Stop::ScaleAwaitingDistance};
    const Answer answers[] = {Answer::Done,       Answer::FrontFirst, Answer::BackFirst,
                              Answer::Distance24, Answer::Distance48, Answer::Distance72,
                              Answer::Abort};

    /// Drive a fresh rig to `stop` and leave it there, un-answered. The push distance is
    /// accumulated ONLY while this script is actually pushing — the SideAssign pulses and the
    /// spin move the robot too, and counting those would make the tape push look finished
    /// before it had started (which is how the first draft of this case aborted).
    const auto driveTo = [](CalibRig& rig, Stop stop) {
        rig.cal.start(facts());
        double pushed = 0.0;
        bool signPushAnswered = false;
        for (int t = 0; t < 20000; ++t) {
            const Phase p = rig.cal.phase();
            REQUIRE_MESSAGE(p != Phase::Aborted, "the scripted operator aborted the run: ",
                            rig.cal.reason());
            bool pushing = false;
            if (p == Phase::PushCapture) {
                if (pushed < 8.0) {
                    rig.pushPrimaries(2.0, 2.0);
                    pushing = true;
                } else {
                    rig.stopPush();
                    if (stop == Stop::PushWatching) {
                        return;  // watching, with a captured push and nothing answered
                    }
                    rig.cal.answer(Answer::Done);
                    if (stop == Stop::PushAwaitingFront) {
                        return;
                    }
                    rig.cal.answer(Answer::FrontFirst);
                    signPushAnswered = true;
                    pushed = 0.0;
                }
            } else if (p == Phase::ScalePush) {
                REQUIRE(signPushAnswered);
                if (pushed < 48.0) {
                    rig.pushPrimaries(2.0, 2.0);
                    pushing = true;
                } else {
                    rig.stopPush();
                    if (stop == Stop::ScaleWatching) {
                        return;
                    }
                    rig.cal.answer(Answer::Done);
                    return;  // ScaleAwaitingDistance
                }
            } else {
                rig.stopPush();  // SideAssign / SpinTest: the calibrator drives, hands off
            }
            rig.cal.tick();
            rig.h.plant().step(Time{kDt});
            if (pushing) {
                pushed += std::abs(rig.trueForwardSpeed()) * kDt;
            }
        }
        FAIL("the scripted operator never reached the requested step");
    };

    for (const Stop stop : stops) {
        for (const Answer a : answers) {
            CalibRig ask;
            driveTo(ask, stop);
            const bool offered = ask.cal.accepts(a);

            CalibRig act;
            driveTo(act, stop);
            act.cal.answer(a);
            // What answer() DID: Abort and a refusal both end in Aborted, so the two are told
            // apart by the reason — a refusal says "nothing was recorded".
            const bool refused = act.cal.phase() == Phase::Aborted
                                 && reasonMentions(act.cal, "nothing was recorded");
            // std::string, not the bare pointer: doctest stringifies a `const char*` as an
            // ADDRESS, and "answer() 0x60024c521210" is a message nobody can act on.
            CHECK_MESSAGE(offered == !refused, "step ", static_cast<int>(stop), ", answer ",
                          static_cast<int>(a), ": accepts() said ", offered,
                          " but answer() ", std::string{refused ? "REFUSED" : "accepted"});
        }
    }

    // Abort is offered everywhere, including where no answer is expected at all.
    CalibRig idle;
    CHECK(idle.cal.accepts(Answer::Abort));
    CHECK_FALSE(idle.cal.accepts(Answer::Done));          // Idle: nothing is waiting
    CHECK_FALSE(idle.cal.accepts(Answer::Distance48));
    CalibRig powered;
    powered.cal.start(facts());
    powered.cal.tick();
    while (powered.cal.phase() == Phase::WaitImu) {
        powered.h.plant().step(Time{kDt});
        powered.cal.tick();
    }
    CHECK(powered.cal.accepts(Answer::Abort));
}
