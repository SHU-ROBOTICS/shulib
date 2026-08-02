#pragma once
//
// sim::LatencyHostileModel — sensors report the PAST (chunk A3, scope item 7).
// The stateful buffering subclass that degradation.hpp's header note promised
// ("SENSOR LATENCY is a STATEFUL policy: an A3 subclass buffers the true values it
// receives and returns older ones") — the ring buffers live HERE, not in the base.
//
// ── The failure SHAPE (confident), and the MAGNITUDES (provisional) ─────────────────
// Each hook call pushes (now, value) into that channel's ring and returns the newest
// stored value at least `latency` old. The consumer-visible truth this creates: a
// rotating robot's reported heading trails its true heading by ω·L — at 1.5 rad/s
// and 20 ms that is 1.7°, transiently past the entire F2 budget — and the odometry
// pairing of CURRENT wheel travel with STALE heading smears position by ≈ v·ω·L per
// unit time of simultaneous translate+rotate. Both are bounded and both RECOVER when
// the motion stops (latency delays, it does not destroy) — the survival tests pin
// the bound and the recovery, not just the corruption.
//
// Startup semantics: until a value is `latency` old, the OLDEST stored value is
// returned (a real device's first report is stale, not absent). Overflow semantics:
// the ring keeps the newest kCapacity samples per channel; if `latency` exceeds what
// the ring spans at the actual tick rate, staleness saturates at the oldest kept
// sample (bounded, documented — at 10 ms ticks the 256-slot ring spans 2.56 s).
//
// ── PROVISIONAL MAGNITUDES (A4 Hardware Assumptions Register; R4 measures) ─────────
//   * imuLatency = 10 ms — smart-port refresh cadence; the IMU's true end-to-end
//     delay is unmeasured.
//   * gpsLatency = 50 ms — camera capture + solve + transport guess; E2's latency
//     compensation needs the REAL number from R4.
//   * encoder latencies default 0 — device refresh (~10 ms, one tick) is real but
//     defaulted off so the composed model isolates the two latencies with known
//     downstream consumers; tests exercise non-zero encoder latency explicitly.
//
// imuReady is deliberately NOT delayed: readiness is a state flag, not a streamed
// measurement, and delaying it buys no realism the calibration window doesn't
// already model. Determinism: no rng draws; outputs are pure functions of the call
// history. Chain position (composed.hpp): LAST — a real device degrades (noise,
// quantization, freeze) at the source, and the bus delays whatever the device
// produced, so latency wraps the already-degraded stream.

#include <array>
#include <cstddef>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

struct LatencyHostileConfig {
    units::Time imuLatency{0.010};   // PROVISIONAL (A4)
    units::Time gpsLatency{0.050};   // PROVISIONAL (A4)
    units::Time driveEncoderLatency{0.0};     // PROVISIONAL (A4): ~1 tick on hardware
    units::Time trackingEncoderLatency{0.0};  // PROVISIONAL (A4): ~1 tick on hardware
};

class LatencyHostileModel final : public DegradationModel {
public:
    explicit LatencyHostileModel(const LatencyHostileConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(cfg_.imuLatency.value() >= 0.0
                                && cfg_.gpsLatency.value() >= 0.0
                                && cfg_.driveEncoderLatency.value() >= 0.0
                                && cfg_.trackingEncoderLatency.value() >= 0.0,
                            "LatencyHostileModel: latencies must be >= 0");
    }

    [[nodiscard]] math::Angle imuHeading(math::Angle trueHeading, units::Time now,
                                         Rng& /*rng*/) override {
        return heading_.delayed(now.value(), trueHeading, cfg_.imuLatency.value());
    }

    [[nodiscard]] units::AngularVelocity imuYawRate(units::AngularVelocity trueRate,
                                                    units::Time now, Rng& /*rng*/) override {
        return yawRate_.delayed(now.value(), trueRate, cfg_.imuLatency.value());
    }

    [[nodiscard]] GpsTruth gps(const GpsTruth& truth, units::Time now, Rng& /*rng*/) override {
        return gps_.delayed(now.value(), truth, cfg_.gpsLatency.value());
    }

    [[nodiscard]] units::AngleDim driveEncoderPosition(int wheel, units::AngleDim trueShaft,
                                                       units::Time now, Rng& /*rng*/) override {
        return drive_[static_cast<std::size_t>(wheel)].delayed(
            now.value(), trueShaft, cfg_.driveEncoderLatency.value());
    }

    [[nodiscard]] units::AngleDim trackingEncoderPosition(int wheelIndex, units::AngleDim trueShaft,
                                                          units::Time now, Rng& /*rng*/) override {
        return tracking_[static_cast<std::size_t>(wheelIndex)].delayed(
            now.value(), trueShaft, cfg_.trackingEncoderLatency.value());
    }

private:
    /// One channel's history: push, then return the newest sample ≥ latency old
    /// (or the oldest kept — startup/overflow semantics in the header). The cutoff
    /// carries a 1 ns inclusivity grace: with latency an exact multiple of the tick
    /// (L = k·dt, the natural configuration), `now − L` lands on a stored stamp up
    /// to fp rounding dust, and without the grace the served delay would flip
    /// between k and k+1 ticks on ~1e-17 arithmetic noise — deterministic but
    /// config-fragile. 1 ns is 7 orders under any real tick and 8 over the dust.
    template <typename T>
    class Ring {
    public:
        [[nodiscard]] T delayed(double now, const T& value, double latency) {
            t_[head_] = now;
            v_[head_] = value;
            head_ = (head_ + 1) % kCapacity;
            if (size_ < kCapacity) {
                ++size_;
            }
            const double cutoff = now - latency + 1e-9;
            // newest-first scan: the first entry old enough wins
            for (std::size_t k = 1; k <= size_; ++k) {
                const std::size_t i = (head_ + kCapacity - k) % kCapacity;
                if (t_[i] <= cutoff) {
                    return v_[i];
                }
            }
            const std::size_t oldest = (head_ + kCapacity - size_) % kCapacity;
            return v_[oldest];
        }

    private:
        static constexpr std::size_t kCapacity = 256;
        std::array<double, kCapacity> t_{};
        std::array<T, kCapacity> v_{};
        std::size_t head_ = 0;
        std::size_t size_ = 0;
    };

    static constexpr std::size_t kMaxWheels =
        static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels);

    LatencyHostileConfig cfg_;
    Ring<math::Angle> heading_{};
    Ring<units::AngularVelocity> yawRate_{};
    Ring<GpsTruth> gps_{};
    std::array<Ring<units::AngleDim>, kMaxWheels> drive_{};
    std::array<Ring<units::AngleDim>, 4> tracking_{};
};

}  // namespace shulib::sim
