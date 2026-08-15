#pragma once
//
// ProsImu — IImu over pros::Imu (chunk R1a): the load-bearing < 1° heading
// source behind the HAL.
//
// BINDS: pros::Imu::get_rotation() — CUMULATIVE CW degrees, "theoretically
// unbounded" — and NEVER get_heading(), which is bounded [0,360) and loses the
// revolution continuity the cumulative contract assumes (imu_conversion.hpp
// binding contract, clause 1; HA-03). Conversion: imuHeadingToCanonical(),
// which applies bootHeading exactly ONCE, here at the edge (HA-05) — no
// downstream consumer re-applies it.
//
// NEVER CALLS tare/tare_rotation/set_rotation/tare_heading/set_heading —
// each re-zeros the sensor independently of bootHeading and silently
// invalidates the additive offset (HA-05; the < 1° budget has no room for
// it). The ONE sanctioned reset() call is calibrate(), below, which IS the
// calibration — not a post-calibration tare. The fence guard test greps this
// file to keep all of that true structurally.
//
// YAW RATE — BOTH SOURCES SHIP, behind YawRateSource (team-lead ruling, brief
// T4; default = DifferentiateRotation):
//  * DifferentiateRotation: d/dt of get_rotation(), whose CW-positive sign is
//    DOCUMENTED — so the CW→CCW negate in imuYawRateToCanonical() is provably
//    right. Costs: the adapter becomes STATEFUL (previous sample + timestamp,
//    hence the injected IClock and the `mutable` members below — yawRate() is
//    const, and the differentiation cache is part of the READ, not commanded
//    state), and differentiating a quantized angle AMPLIFIES quantization
//    noise — a real cost against the heading-owned < 1° budget, measured at
//    the bench before trusting.
//  * GyroRateZ: get_gyro_rate().z — a real hardware rate, but its SIGN is
//    UNDOCUMENTED (HA-04) and its deg/s unit is itself a belief (HA-109).
//    Assumed CW-positive until the bench measures it (runbook step: spin CW
//    at a steady rate; canonical yaw rate must be negative).
//  Rejected: shipping only one path (either loses a real hardware rate or
//  defaults the load-bearing quantity to a sign nobody has measured).
//
// CALIBRATION: calibrate() starts it (reset(false), HA-108) — call it once
// from initialize(); it never blocks. isReady() is true only after
// calibrate() was called AND is_calibrating() has gone false: an
// never-calibrated IMU emits garbage-that-moves (HA-23), so "I was never
// calibrated" must read as not-ready, not as ready-by-default. POSITIVE
// polarity per imu.hpp:31-33.
//
// SENTINELS (T7): PROS_ERR_F reads (calibrating / unplugged) hold the last
// good value; heading() before any good read returns bootHeading (the best
// available estimate of a robot that has not moved). faultedReads() exposes
// the screen count; the loop's HealthMonitor owns raising IMU_LOST (its
// boot-window rule already handles the calibration phase).
//
// PITCH/ROLL: get_pitch()/get_roll() degrees (-180,180) → math::Angle,
// UNNEGATED — the as-mounted sign convention is unmeasured (HA-110). No
// consumer exists yet (the tip detection these exist for is a master-plan M2
// item); whoever writes the first one consumes MAGNITUDES until the bench
// settles the signs.
//
// HA register: HA-02..HA-05, HA-23, HA-108..HA-110.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/imu.hpp"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdint>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/imu_conversion.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

/// Which hardware quantity yawRate() differentiates or reads (header: the T4
/// ruling and both costs).
enum class YawRateSource {
    DifferentiateRotation,  ///< d/dt of get_rotation() — documented sign (default)
    GyroRateZ,              ///< get_gyro_rate().z — real rate, UNDOCUMENTED sign (HA-04)
};

/// IImu over pros::Imu — the load-bearing heading source. Binds get_rotation() (cumulative
/// CW degrees) and never get_heading(); applies bootHeading exactly ONCE, here at the edge;
/// never tares, because a tare re-zeros the sensor underneath a live bootHeading offset.
/// STATEFUL despite the const readers: each reader screens a non-finite value and returns
/// the last good one, and the default yaw-rate path caches a sample to differentiate — which
/// is why construction takes an IClock. One owner, the loop; not safe to read concurrently.
class ProsImu final : public IImu {
public:
    /// `bootHeading`: the robot's canonical field heading AT calibration — ONE
    /// owner, the robot's start pose (HA-05). `clock`: required by the
    /// differentiation path; must outlive this adapter.
    ProsImu(std::uint8_t port, math::Angle bootHeading, const IClock& clock,
            YawRateSource yawSource = YawRateSource::DifferentiateRotation)
        : sensor_{port},
          bootHeading_{bootHeading},
          clock_{&clock},
          yawSource_{yawSource},
          lastHeading_{bootHeading} {}

    /// Start calibration (reset(false), HA-108). Call ONCE from initialize();
    /// never blocks — the loop waits on isReady() (C1's wait-for-live).
    /// A second call is a precondition violation: re-calibrating mid-run
    /// re-zeros the sensor under a live bootHeading offset (HA-05's hazard).
    void calibrate() {
        SHULIB_PRECONDITION(!calibrateStarted_,
                            "ProsImu::calibrate: already calibrated — a second reset() would "
                            "re-zero the sensor under a live bootHeading (HA-05)");
        calibrateStarted_ = true;
        sensor_.reset(false);
    }

    /// Canonical heading via imuHeadingToCanonical (bootHeading applied ONCE,
    /// here). Screened reads hold last-good; before any good read this is
    /// bootHeading itself (a robot that has not moved).
    [[nodiscard]] math::Angle heading() const override {
        const double degCw = sensor_.get_rotation();
        if (!std::isfinite(degCw)) {
            faultedReads_ += 1;
            return lastHeading_;
        }
        lastHeading_ = imuHeadingToCanonical(degCw, bootHeading_);
        return lastHeading_;
    }

    /// Canonical yaw rate (CCW-positive rad/s) from the YawRateSource fixed at construction.
    /// DifferentiateRotation (the default) differentiates get_rotation() BETWEEN SUCCESSIVE
    /// CALLS, so the interval is however long since you last asked rather than a fixed dt:
    /// the first call after construction returns 0 (one sample is not a derivative), and a
    /// second call at the same clock instant repeats the previous answer instead of dividing
    /// by zero. Call it once per tick. GyroRateZ instead reads the hardware rate, converted
    /// on an UNMEASURED sign assumption (HA-04). Either way a screened read holds last-good.
    [[nodiscard]] units::AngularVelocity yawRate() const override {
        return yawSource_ == YawRateSource::DifferentiateRotation ? differentiatedRate()
                                                                  : gyroRate();
    }

    /// True once calibrate() was called and calibration has finished (header:
    /// never-calibrated must read not-ready). POSITIVE polarity (imu.hpp:31-33).
    [[nodiscard]] bool isReady() const override {
        return calibrateStarted_ && !sensor_.is_calibrating();
    }

    /// Chassis pitch from get_pitch() (degrees, (-180, 180)) as a canonical Angle,
    /// UNNEGATED: the as-mounted sign convention is unmeasured (HA-110), so consume the
    /// MAGNITUDE until the bench settles it. Nothing reads this yet — the tip detection it
    /// exists for is a master-plan M2 item — so the first consumer is also the first chance
    /// to bake in a wrong sign. A non-finite read holds the last good value and counts in
    /// faultedReads(); before any good read, 0.
    [[nodiscard]] math::Angle pitch() const override {
        const double deg = sensor_.get_pitch();
        if (!std::isfinite(deg)) {
            faultedReads_ += 1;
            return lastPitch_;
        }
        lastPitch_ = math::Angle::degrees(deg);  // sign as-mounted: HA-110
        return lastPitch_;
    }

    /// Chassis roll from get_roll() (degrees, (-180, 180)) as a canonical Angle, on the same
    /// terms as pitch(): UNNEGATED because the mounting sign is unmeasured (HA-110), a
    /// non-finite read holds last-good and counts in faultedReads(), and 0 before any good
    /// read. Note that roll and pitch each hold their own last-good value, independently.
    [[nodiscard]] math::Angle roll() const override {
        const double deg = sensor_.get_roll();
        if (!std::isfinite(deg)) {
            faultedReads_ += 1;
            return lastRoll_;
        }
        lastRoll_ = math::Angle::degrees(deg);  // sign as-mounted: HA-110
        return lastRoll_;
    }

    /// How many reads were screened to last-good (T7 observability).
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    [[nodiscard]] units::AngularVelocity differentiatedRate() const {
        const double degCw = sensor_.get_rotation();
        if (!std::isfinite(degCw)) {
            faultedReads_ += 1;
            return lastRate_;
        }
        const units::Time now = clock_->now();
        if (!hasRateSample_) {
            // No history yet: a stationary boot reads 0, honestly — the first
            // derivative needs two samples.
            hasRateSample_ = true;
            lastRotationDeg_ = degCw;
            lastSampleTime_ = now;
            return lastRate_;
        }
        const double dt = (now - lastSampleTime_).value();
        if (dt <= 0.0) {
            return lastRate_;  // same tick: no new information to differentiate
        }
        const double cwDegPerSec = (degCw - lastRotationDeg_) / dt;
        lastRotationDeg_ = degCw;
        lastSampleTime_ = now;
        lastRate_ = imuYawRateToCanonical(cwDegPerSec);
        return lastRate_;
    }

    [[nodiscard]] units::AngularVelocity gyroRate() const {
        const auto raw = sensor_.get_gyro_rate();
        if (!std::isfinite(raw.z)) {
            faultedReads_ += 1;
            return lastRate_;
        }
        // z assumed CW-positive deg/s (HA-04 sign / HA-109 units) — the bench
        // measures both before this branch is trusted on a robot.
        lastRate_ = imuYawRateToCanonical(raw.z);
        return lastRate_;
    }

    ::pros::v5::Imu sensor_;
    math::Angle bootHeading_;
    const IClock* clock_;
    YawRateSource yawSource_;
    bool calibrateStarted_ = false;
    // `mutable`: IImu's readers are const; the T7 hold-last-good screen and
    // the differentiation cache are state OF THE READ (header note).
    mutable math::Angle lastHeading_;
    mutable math::Angle lastPitch_{};
    mutable math::Angle lastRoll_{};
    mutable units::AngularVelocity lastRate_{0.0};
    mutable double lastRotationDeg_ = 0.0;
    mutable units::Time lastSampleTime_{0.0};
    mutable bool hasRateSample_ = false;
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
