#pragma once
//
// calibration_test_rig.hpp — the shared host rig for the drivetrain calibrator and its screen
// model (chunk R3d Part 2F-1; lifted out of drivetrain_calibrator_test.cpp so the panel's cases
// drive the SAME scripted operator over the SAME plant rather than a copy of it).
//
// Robot two's shape on the A2 plant: two wheels of FIVE coupled members each (ports 11–15 and
// 16–20) plus two motors off the drivetrain (5 and 6); a scripted operator that pushes the robot
// 48 in, waits for pushReady() like the panel, answers FRONT/BACK and stands clear. At the tree's
// usual static friction (kS = 1.2) and with κ — the plant's coast-follow — stated by every caller,
// because the idle side's behaviour is exactly what the Part 1 rig got wrong (kS = 0, a pinned
// idle wheel). THE PLANT'S NUMBERS ARE INVENTED; this rig proves logic, not robot physics.

#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <span>

#include "doctest.h"
#include "shulib/calibration/drivetrain_calibrator.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

namespace calib_rig {

using shulib::calibration::Answer;
using shulib::calibration::CalibratorConfig;
using shulib::calibration::DrivetrainCalibrator;
using shulib::calibration::Phase;
using shulib::hal::BrakeMode;
using shulib::hal::IMotor;
using shulib::hal::MotorCartridge;
using shulib::hal::fake::FakeMotor;
using shulib::kinematics::TankKinematics;
using shulib::sim::MemberFault;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

inline constexpr double kDt = 0.01;
inline constexpr double kTrackWidth = 12.5;   // the plant's TRUE track width (TankKinematics)
inline constexpr double kKv = 0.17;           // volts per in/s of wheel surface speed
inline constexpr double kPlantRadius = 3.25 / 2.0;  // DrivePlantConfig::driveWheelDiameter / 2, 1:1
/// The tree's usual static-friction dead band (sim_motor_model_test, the hostile suites, the EKF
/// accuracy suite). The Part 1 rig ran at 0 here, which is why its pulse passed and robot two's
/// did not (the file header).
inline constexpr double kUsualKs = 1.2;

/// The plant a case runs on. κ (coast-follow) has NO default: every case states it, because the
/// idle side's behaviour is exactly what the old rig got wrong. kS defaults to the tree's usual.
[[nodiscard]] inline SimHarnessConfig calibPlant(double kappa, double kS = kUsualKs, double drag = 0.0) {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = kS, .kV = kKv, .kA = 0.0};
    cfg.plant.coastFollow = kappa;
    cfg.plant.decoupledDrag = drag;
    return cfg;
}

/// The human facts a person states before a run.
[[nodiscard]] inline shulib::calibration::HumanFacts facts() {
    shulib::calibration::HumanFacts f;
    f.kind = shulib::hal::DrivetrainKind::Tank;
    f.cartridge = MotorCartridge::Blue;
    f.imuPort = 2;
    f.provenance = "host test";
    return f;
}

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
    /// How many members of each wheel still carry their command to the train — five, less any
    /// the harness made Decoupled (the push below needs it).
    std::size_t coupled[2] = {kPerWheel, kPerWheel};
    DrivetrainCalibrator cal;

    CalibRig(const CalibratorConfig& ccfg, const SimHarnessConfig& pcfg,
             shulib::sim::DegradationModel* degradation = nullptr)
        : h{kin, pcfg, nullptr, degradation},
          cal{wire(), portSpan(), h.imu(), h.clock(), ccfg} {}

    [[nodiscard]] std::span<IMotor* const> candidateSpan() {
        return std::span<IMotor* const>{candidates.data(), candidates.size()};
    }
    [[nodiscard]] std::span<const std::int8_t> portSpan() const {
        return std::span<const std::int8_t>{ports.data(), ports.size()};
    }

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
        if (f == MemberFault::Decoupled) {
            --coupled[static_cast<std::size_t>(wheel)];  // (set once per member in this file)
        }
    }

    /// A HAND PUSH, modelled: the harness commands each wheel's PRIMARY so the WHEEL rolls at
    /// `leftVolts` / `rightVolts` under the plant's coupled-train mean; the four mates free-wheel
    /// and the calibrator commands nothing at all. A wheel whose primary is wired backwards is
    /// commanded the other way round, because a hand push rolls a wheel forward whatever the
    /// wiring behind it. The mean is over the members whose command still reaches the train —
    /// a hand pushes both sides alike whatever is slipping behind them, so a wheel with a
    /// Decoupled member gets its primary scaled to match (without this, case 24's push curved
    /// 95 deg: an artifact of pushing BY MOTOR COMMAND, not a property of the robot).
    void pushPrimaries(double leftVolts, double rightVolts) {
        const double want[2] = {leftVolts, rightVolts};
        for (int w = 0; w < 2; ++w) {
            const auto wi = static_cast<std::size_t>(w);
            const double primary = static_cast<double>(coupled[wi]) * want[wi];
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
        return candidateSpan();
    }
};

/// Where the scripted operator is — handed to a case's hook every tick.
struct Script {
    int tick = 0;
    int stage = 0;         ///< 0 pushing, 1 pushed (waiting to touch DONE), 2 DONE touched, 3 hands off
    double pushed = 0.0;   ///< TRUE inches the robot has travelled while the harness pushed
    bool paused = false;   ///< the push is paused (RunOptions::pauseAtInches)
};

/// How one scripted run behaves.
struct RunOptions {
    double pushInches = 48.0;   ///< how far the ONE push travels, TRUE — the config's 48 in
    /// WHEEL volts during the push. The harness drives the PRIMARY at five times this to make the
    /// coupled-train mean come out here, and a FakeMotor clamps at 12 V — so anything above 2.4
    /// would clamp, and (the way this file first went wrong at Part 1) a CURVED push would
    /// silently straighten out because both sides clamped to the same 12 V.
    double pushVolts = 2.2;
    double curve = 1.0;         ///< right-side volts relative to left during the push
    double pushDirection = 1.0; ///< −1 pushes the plant backward
    Answer front = Answer::FrontFirst;
    double plantKs = kUsualKs;  ///< the plant's static-friction dead band, for the exact stop
    /// The scripted operator touches DONE only once pushReady() says so — what the panel does.
    /// false = DONE the moment the harness stops pushing, ready or not (an old screen).
    bool waitForReady = true;
    double pauseAtInches = -1.0;  ///< stop pushing here for pauseTicks ticks (a hook acts then)
    int pauseTicks = 20;
    int maxTicks = 40000;
    std::function<void(CalibRig&, const Script&)> hook = nullptr;  ///< per-tick fault injection
    std::function<bool(CalibRig&)> stopWhen = nullptr;  ///< return early, un-answered
};

/// Drive a whole calibration with `cal` (the rig's own, or one rebuilt in place — case 32): the
/// operator pushes when asked, answers when asked, and stands clear while the calibrator drives.
/// Returns the phase the run ended in (or was stopped in).
inline Phase runWith(CalibRig& rig, DrivetrainCalibrator& cal, const RunOptions& opt) {
    Script s;
    int pauseLeft = 0;
    bool pausedOnce = false;
    // The plant's transfer, both ways: what a wheel command produces, and what command produces a
    // wanted speed. Used to land the push on EXACTLY the configured distance.
    const double ks = opt.plantKs;
    auto speedFor = [ks](double volts) {
        return std::abs(volts) <= ks ? 0.0 : (std::abs(volts) - ks) / kKv;
    };
    auto voltsFor = [ks](double speed) { return speed * kKv + ks; };
    for (s.tick = 0; s.tick < opt.maxTicks; ++s.tick) {
        Phase p = cal.phase();
        if (p == Phase::Done || p == Phase::Aborted) {
            break;
        }
        if (opt.stopWhen && opt.stopWhen(rig)) {
            break;
        }
        if (opt.hook) {
            opt.hook(rig, s);
            // A hook may abort the run; what follows must not then command a motor and undo the
            // safe state the abort just established.
            p = cal.phase();
            if (p == Phase::Done || p == Phase::Aborted) {
                break;
            }
        }
        bool pushing = false;
        if (p == Phase::PushCapture) {
            if (s.stage == 0) {
                const double remaining = opt.pushInches - s.pushed;
                if (!pausedOnce && opt.pauseAtInches >= 0.0 && s.pushed >= opt.pauseAtInches) {
                    pausedOnce = true;
                    pauseLeft = opt.pauseTicks;
                }
                if (pauseLeft > 0) {
                    rig.stopPush();
                    s.paused = true;
                    --pauseLeft;
                } else if (remaining > 1e-10) {
                    s.paused = false;
                    double v = opt.pushVolts;
                    // With a straight push the last tick is trimmed so the TRUE travel lands on
                    // the distance exactly — otherwise the scale would be off by the overshoot and
                    // could not be compared with the plant's own radius.
                    if (opt.curve == 1.0 && speedFor(v) * kDt > remaining) {
                        v = voltsFor(remaining / kDt);
                    }
                    rig.pushPrimaries(v * opt.pushDirection, v * opt.curve * opt.pushDirection);
                    pushing = true;
                } else {
                    s.paused = false;
                    rig.stopPush();
                    s.stage = 1;
                }
            } else if (s.stage == 1) {
                if (!opt.waitForReady || cal.pushReady()) {
                    cal.answer(Answer::Done);
                    s.stage = 2;
                }
            } else if (s.stage == 2) {
                cal.answer(opt.front);
                s.stage = 3;
            }
        }
        cal.tick();
        rig.h.plant().step(Time{kDt});
        if (pushing) {
            s.pushed += std::abs(rig.trueForwardSpeed()) * kDt;
        }
    }
    return cal.phase();
}

inline Phase runCalibration(CalibRig& rig, const RunOptions& opt) { return runWith(rig, rig.cal, opt); }

/// Every candidate at 0 V and coasting — the state every refusal and every abort must leave.
[[nodiscard]] inline bool safeStateHolds(CalibRig& rig) {
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

[[nodiscard]] inline bool reasonMentions(const DrivetrainCalibrator& cal, const char* text) {
    return std::strstr(cal.reason(), text) != nullptr;
}

/// Printable ASCII and no '%' — the two rules for any text that may reach the V5's screen (PROS
/// re-reads screen text as a format; VEXos's fonts carry ASCII only).
[[nodiscard]] inline bool screenSafe(const char* s) {
    for (const char* c = s; *c != '\0'; ++c) {
        const auto u = static_cast<unsigned char>(*c);
        if (u < 0x20 || u > 0x7E || *c == '%') {
            return false;
        }
    }
    return true;
}

}  // namespace calib_rig
