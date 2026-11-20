#pragma once
//
// geared_tank_rig.hpp — robot one's drive on the host test plant (chunk CAL2, 2026-11-17): a tank
// whose sides are each ONE gear train of N motors with BACKLASH, on a chassis too heavy for one
// motor to move. Kinematic, like the A2 plant; every number is INVENTED, shaped on robot one's
// logs (own shaft ~10-14 deg, side-mates 2-6 deg, the other side 0, the yaw a fraction of a
// degree of frame flex). It proves logic, not robot physics.
//
// Per side: the wheels (w), the train (phi, within slackDeg of the wheels) and each member's
// shaft (within backlashDeg of the train). A member alone closes its own gap; past trainKs volts
// it also winds the train to its slack, dragging its mates; the wheels roll only when the side's
// mean volts pass chassisKs. A side with nothing driven springs back to where the push left it.

#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <span>

#include "doctest.h"
#include "shulib/calibration/drivetrain_calibrator.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace geared {

using shulib::calibration::Answer;
using shulib::calibration::CalibratorConfig;
using shulib::calibration::DrivetrainCalibrator;
using shulib::calibration::Phase;
using shulib::hal::IMotor;
using shulib::hal::fake::FakeMotor;

inline constexpr double kDeg = shulib::calibration::kDegreesToRadians;
inline constexpr double kDt = 0.01;
inline constexpr double kRadius = 1.625;  // in, 1:1
inline constexpr double kTrack = 12.5;    // in, the TRUE track width
inline constexpr double kKv = 0.17;       // V per in/s of wheel surface speed

struct TankSpec {
    int perSide = 5;
    bool alternate = true;        ///< mounts + - + - + (robot one's motors face each other)
    double backlashDeg = 10.0;    ///< each member's dead band at its shaft
    double slackDeg = 4.0;        ///< how far the train winds against wheels that hold
    double memberKs = 1.0;        ///< volts a member needs to turn at all
    double trainKs = 4.0;         ///< summed member volts that wind the train (one member at 3 V cannot)
    double chassisKs = 3.0;       ///< a side's MEAN member volts the wheels need (one of five at 12 V: 2.4)
    double flexDegPerDeg = 0.05;  ///< yaw per degree of wind-up, + as if that side drove the robot
    double rateNoiseDegS = 0.0;   ///< instantaneous yaw-rate noise, sign alternating every tick
    double driftDegS = 0.0;       ///< a real, steady turn of the heading (D4's "not at rest")
    bool rightFirst = false;      ///< the FIRST N ports (11..) are the RIGHT side
};

/// Robot one's drive: ports 11..(10+N) LEFT, the next N RIGHT.
struct GearedTank {
    TankSpec spec;
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeImu imu;
    std::array<FakeMotor, 20> motors{};
    std::array<IMotor*, 20> ptrs{};
    std::array<std::int8_t, 20> ports{};
    std::size_t n = 0;
    double w[2] = {0.0, 0.0};
    double phi[2] = {0.0, 0.0};
    double restPhi[2] = {0.0, 0.0};
    std::array<double, 20> th{};
    std::array<double, 20> restTh{};
    double theta = 0.0;
    double lastFlex = 0.0;
    long ticks = 0;
    /// Added to member j's sign-corrected shaft as READ (a fault a case injects), radians.
    std::function<double(const GearedTank&, std::size_t)> bias = nullptr;

    explicit GearedTank(const TankSpec& s) : spec{s} {
        n = static_cast<std::size_t>(2 * s.perSide);
        for (std::size_t j = 0; j < n; ++j) {
            ports[j] = static_cast<std::int8_t>(11 + j);
            ptrs[j] = &motors[j];
        }
        imu.setReady(true);
        write(0.0);
    }

    [[nodiscard]] std::span<IMotor* const> candidates() const { return {ptrs.data(), n}; }
    [[nodiscard]] std::span<const std::int8_t> portSpan() const { return {ports.data(), n}; }
    [[nodiscard]] int side(std::size_t j) const { return (j < n / 2) != spec.rightFirst ? 0 : 1; }
    /// The member being driven alone (a pulse), or n.
    [[nodiscard]] std::size_t pulsed() const {
        std::size_t found = n;
        for (std::size_t j = 0; j < n; ++j) {
            if (motors[j].commandedVoltage().value() != 0.0) {
                if (found < n) {
                    return n;
                }
                found = j;
            }
        }
        return found;
    }
    /// Member j's travel since the push left it, radians (sign-corrected).
    [[nodiscard]] double travel(std::size_t j) const { return th[j] - restTh[j]; }
    [[nodiscard]] double mount(std::size_t j) const {
        return spec.alternate && (j % static_cast<std::size_t>(spec.perSide)) % 2 == 1 ? -1.0 : 1.0;
    }
    [[nodiscard]] double windup(int s) const { return phi[s] - restPhi[s]; }

    /// One tick. `pushInPerS` > 0 is a hand pushing the whole robot forward (nothing driven).
    void step(double pushInPerS = 0.0) {
        const auto before = th;
        const double c = spec.slackDeg * kDeg / 2.0;
        const double b = spec.backlashDeg * kDeg / 2.0;
        double v[2] = {0.0, 0.0};
        for (int s = 0; s < 2; ++s) {
            double sum = 0.0;
            double sumDriven = 0.0;
            bool driven = false;
            for (std::size_t j = 0; j < n; ++j) {
                if (side(j) != s) {
                    continue;
                }
                const double u = mount(j) * motors[j].commandedVoltage().value();
                sum += u;
                driven = driven || std::abs(u) > spec.memberKs;
                sumDriven += std::abs(u) > spec.memberKs ? u : 0.0;
            }
            const double mean = sum / static_cast<double>(spec.perSide);
            if (pushInPerS > 0.0) {
                w[s] += pushInPerS * kDt / kRadius;
                phi[s] = std::clamp(phi[s], w[s] - c, w[s] + c);
                for (std::size_t j = 0; j < n; ++j) {
                    if (side(j) == s) {
                        th[j] = std::clamp(th[j], phi[s] - b, phi[s] + b);
                        restTh[j] = th[j];
                    }
                }
                restPhi[s] = phi[s];
                continue;
            }
            if (!driven) {  // springs back to where the push left it
                phi[s] = restPhi[s];
                for (std::size_t j = 0; j < n; ++j) {
                    if (side(j) == s) {
                        th[j] = restTh[j];
                    }
                }
                continue;
            }
            const double dir = sumDriven > 0.0 ? 1.0 : -1.0;  // the way the driven members push
            if (std::abs(mean) > spec.chassisKs) {
                v[s] = (mean > 0.0 ? 1.0 : -1.0) * (std::abs(mean) - spec.chassisKs) / kKv;
                const double dw = v[s] * kDt / kRadius;
                w[s] += dw;
                restPhi[s] += dw;  // where it springs back to moves with the wheels
                for (std::size_t j = 0; j < n; ++j) {
                    restTh[j] += side(j) == s ? dw : 0.0;
                }
                phi[s] = w[s] + dir * c;
            } else if (std::abs(sumDriven) > spec.trainKs) {
                phi[s] = w[s] + dir * c;
            }
            for (std::size_t j = 0; j < n; ++j) {
                if (side(j) != s) {
                    continue;
                }
                const double u = mount(j) * motors[j].commandedVoltage().value();
                th[j] = std::abs(u) > spec.memberKs ? phi[s] + (u > 0.0 ? b : -b)
                                                    : std::clamp(th[j], phi[s] - b, phi[s] + b);
            }
        }
        const double omega = (v[1] - v[0]) / kTrack;
        theta += omega * kDt + spec.driftDegS * kDeg * kDt;
        clock.advance(shulib::units::Time{kDt});
        for (std::size_t j = 0; j < n; ++j) {
            motors[j].setVelocity(shulib::units::AngularVelocity{mount(j) * (th[j] - before[j]) / kDt});
        }
        ++ticks;
        write(omega);
    }

private:
    void write(double omega) {
        for (std::size_t j = 0; j < n; ++j) {
            motors[j].setPosition(shulib::units::AngleDim{mount(j) * (th[j] + (bias ? bias(*this, j) : 0.0))});
        }
        const double flex = spec.flexDegPerDeg * (windup(1) - windup(0));
        const double noise = (ticks % 2 == 0 ? 1.0 : -1.0) * spec.rateNoiseDegS * kDeg;
        imu.setHeading(shulib::math::Angle::radians(theta + flex));
        imu.setYawRate(shulib::units::AngularVelocity{omega + spec.driftDegS * kDeg + (flex - lastFlex) / kDt + noise});
        lastFlex = flex;
    }
};

/// Robot one's two typed Calibrate thresholds (its ledger: 0.6 deg, 1.0 deg/s).
[[nodiscard]] inline CalibratorConfig robotOneConfig() {
    CalibratorConfig c;
    c.pulseYawFloor = shulib::units::AngleDim{0.6 * kDeg};
    c.restYawRateFloor = shulib::units::AngularVelocity{1.0 * kDeg};
    return c;
}

[[nodiscard]] inline shulib::calibration::HumanFacts facts() {
    shulib::calibration::HumanFacts f;
    f.cartridge = shulib::hal::MotorCartridge::Blue;
    f.imuPort = 2;
    f.provenance = "geared host test";
    return f;
}

/// A whole run: push 48 in, touch DONE once pushReady(), stand clear. `hook` runs every tick.
inline Phase run(GearedTank& t, DrivetrainCalibrator& cal,
                 const std::function<void(GearedTank&, DrivetrainCalibrator&)>& hook = nullptr) {
    cal.start(facts());
    double pushed = 0.0;
    for (int k = 0; k < 60000; ++k) {
        if (cal.phase() == Phase::Done || cal.phase() == Phase::Aborted) {
            break;
        }
        if (hook) {
            hook(t, cal);
        }
        double push = 0.0;
        if (cal.phase() == Phase::PushCapture) {
            if (pushed < 48.0 - 1e-9) {
                push = std::min(10.0, (48.0 - pushed) / kDt);
                pushed += push * kDt;
            } else if (cal.pushReady()) {
                cal.answer(Answer::Done);
            }
        }
        cal.tick();
        t.step(push);
    }
    return cal.phase();
}

}  // namespace geared
