#pragma once
//
// sim::GpsHostileModel — how the V5 GPS actually misbehaves (chunk A3, scope item 4).
// Populates the gps() seam of degradation.hpp.
//
// ── The failure SHAPES modeled (confident), and their MAGNITUDES (provisional) ──────
// 1. UPDATE DECIMATION: the GPS camera produces a NEW fix only every `updatePeriod`;
//    between updates the SAME sample is re-reported. A ~100 Hz consumer therefore
//    sees each fix ~5× — the double-counting hazard Phase E's correctors must
//    survive (folding one measurement N times as if independent).
// 2. POSITION/HEADING NOISE: each fresh sample is truth + Gaussian noise. The
//    reported rmsError is a CONFIG value, deliberately decoupled from the actual
//    noise sigma — a real sensor's self-estimate and its real error are different
//    numbers, and the fusion tier must be graded on surviving that gap.
// 3. NO-FIX — three flavours, all reporting hasFix = false with a FINITE stale pose
//    (the IGps contract: no-fix pose is unspecified-but-finite, callers must check
//    hasFix): `offStrip` (Driving Skills: NO strip exists — false for the whole
//    run), `noFixWindows` (transient occlusion/glare), and `dropoutAt` (device
//    disconnect, permanent). Before any fix exists the stale pose is the ORIGIN —
//    deliberately wrong for most scenarios, so any code that trusts a no-fix pose
//    is dragged toward (0,0) and caught, rather than accidentally handed the truth.
//    (What the REAL device serves off-strip/no-fix is unknown: A4 register HA-31.)
// 4. BAD FIX (event, default OFF): during a `badFixWindows` entry, fresh samples are
//    truth + a constant offset while STILL claiming hasFix = true with the normal
//    rmsError — a plausible, confident lie (strip misread/reflection). This is the
//    attack the fusion innovation gate exists for; damage must be bounded by it.
//
// ── PROVISIONAL MAGNITUDES (A4 Hardware Assumptions Register; R4 measures) ─────────
// Register: docs/planning/hardware-assumptions.md — HA-26..HA-29 (+HA-31 no-fix pose).
//   * noiseSigma = 0.7 in/axis   — field-test folklore for on-strip jitter. (HA-26)
//   * headingNoiseSigma = 1°     — GPS-derived heading is much worse than the IMU's. (HA-27)
//   * updatePeriod = 50 ms       — the GPS camera's cadence guess. (HA-28)
//   * reportedRms = 1.0 in       — what the sensor CLAIMS when healthy. (HA-29)
//   * noFixRms = 99 in           — the off-strip/no-fix error report. (HA-29)
// (GPS LATENCY is deliberately NOT here — LatencyHostileModel delays any sensor,
//  including this one's output, and stacking them in the chain is the composition
//  story. One pathology, one owner.)
//
// Defaults are HOSTILE for the continuous pathologies (decimation + noise ON);
// no-fix windows, off-strip, bad-fix and dropout are events/scenario properties,
// default OFF. All draws (3 Gaussians per FRESH sample: x, y, heading) come from
// the plant's seeded Rng; re-reported samples draw nothing.

#include <cmath>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"  // hostile_detail::kDegToRad
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// A [start, end) time window (seconds, plant-clock time).
struct GpsNoFixWindow {
    units::Time start{};
    units::Time end{};
};

/// A confident lie: during [start, end), fresh fixes are truth + (dx, dy) at the
/// NORMAL reported rms with hasFix true.
struct GpsBadFixWindow {
    units::Time start{};
    units::Time end{};
    units::Length dx{};
    units::Length dy{};
};

struct GpsHostileConfig {
    double noiseSigmaIn = 0.7;                                  // PROVISIONAL (A4: HA-26)
    double headingNoiseSigmaRad = 1.0 * hostile_detail::kDegToRad;  // PROVISIONAL (A4: HA-27)
    units::Time updatePeriod{0.05};                             // PROVISIONAL (A4: HA-28)
    units::Length reportedRms{1.0};                             // PROVISIONAL (A4: HA-29)
    units::Length noFixRms{99.0};                               // PROVISIONAL (A4: HA-29)
    bool offStrip = false;              ///< Driving Skills: no strip for the whole run
    std::vector<GpsNoFixWindow> noFixWindows{};   ///< transient occlusions (events)
    std::vector<GpsBadFixWindow> badFixWindows{};  ///< confident lies (events)
    units::Time dropoutAt{1e18};        ///< EVENT: device disconnect (permanent no-fix)
};

class GpsHostileModel final : public DegradationModel {
public:
    explicit GpsHostileModel(const GpsHostileConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(cfg_.noiseSigmaIn >= 0.0, "GpsHostileModel: noiseSigmaIn must be >= 0");
        SHULIB_PRECONDITION(cfg_.headingNoiseSigmaRad >= 0.0,
                            "GpsHostileModel: headingNoiseSigmaRad must be >= 0");
        SHULIB_PRECONDITION(cfg_.updatePeriod.value() >= 0.0,
                            "GpsHostileModel: updatePeriod must be >= 0");
        for (const GpsNoFixWindow& w : cfg_.noFixWindows) {
            SHULIB_PRECONDITION(w.end.value() >= w.start.value(),
                                "GpsHostileModel: a no-fix window ends before it starts");
        }
        for (const GpsBadFixWindow& w : cfg_.badFixWindows) {
            SHULIB_PRECONDITION(w.end.value() >= w.start.value(),
                                "GpsHostileModel: a bad-fix window ends before it starts");
        }
    }

    [[nodiscard]] GpsTruth gps(const GpsTruth& truth, units::Time now, Rng& rng) override {
        // Permanent / windowed no-fix: stale-but-finite pose, inflated rms, no fix.
        if (cfg_.offStrip || now.value() >= cfg_.dropoutAt.value() || inNoFixWindow(now)) {
            return GpsTruth{staleEmitted_, cfg_.noFixRms, false};
        }

        // Decimation: only sample a FRESH fix when the camera cadence says so.
        if (!hasSample_ || now.value() - lastSampleTime_ >= cfg_.updatePeriod.value()) {
            double x = truth.pose.x().value() + cfg_.noiseSigmaIn * rng.nextGaussian();
            double y = truth.pose.y().value() + cfg_.noiseSigmaIn * rng.nextGaussian();
            const double dh = cfg_.headingNoiseSigmaRad * rng.nextGaussian();
            if (const GpsBadFixWindow* bad = activeBadFix(now)) {
                x += bad->dx.value();  // the confident lie rides the same pipeline
                y += bad->dy.value();
            }
            held_ = math::Pose2d{units::Length{x}, units::Length{y},
                                 truth.pose.heading() + math::Angle::radians(dh)};
            lastSampleTime_ = now.value();
            hasSample_ = true;
        }
        staleEmitted_ = held_;  // what a later no-fix phase will re-report
        return GpsTruth{held_, cfg_.reportedRms, truth.hasFix};
    }

private:
    [[nodiscard]] bool inNoFixWindow(units::Time now) const {
        for (const GpsNoFixWindow& w : cfg_.noFixWindows) {
            if (now.value() >= w.start.value() && now.value() < w.end.value()) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] const GpsBadFixWindow* activeBadFix(units::Time now) const {
        for (const GpsBadFixWindow& w : cfg_.badFixWindows) {
            if (now.value() >= w.start.value() && now.value() < w.end.value()) {
                return &w;
            }
        }
        return nullptr;
    }

    GpsHostileConfig cfg_;
    math::Pose2d held_{};          // the current decimated sample
    math::Pose2d staleEmitted_{};  // last pose actually emitted (origin until a fix exists)
    double lastSampleTime_ = 0.0;
    bool hasSample_ = false;
};

}  // namespace shulib::sim
