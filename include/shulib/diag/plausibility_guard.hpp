#pragma once
//
// Physical-plausibility invariants (diagnostics-plan D-5; WS13, chunk C5) —
// FiniteGuard's log-and-recover posture, extended beyond finiteness. A value can
// be perfectly finite and still be a lie: a pose that teleported 8 inches in one
// 10 ms tick, a commanded speed beyond every configured budget, a wheel volt above
// the battery ceiling. Each such violation raises FaultCode::Implausible (a fault,
// never a crash — fault.hpp's absolute rule) and the run continues on a safe
// fallback. A3 proved this class of guard catches real defects; D-6's flight
// recorder (E1) will trigger on exactly these faults.
//
// The three invariants and where each runs:
//   1. POSE DELTA (PoseDeltaGuard, run by the scheduler each tick): |Δposition| and
//      |Δheading| within the drivetrain's physical maximum × margin × dt. Catches a
//      lying estimate the moment it lies (encoder glitch, fusion bug, a mid-run
//      setPose). Recovery = the fault is ADVISORY: the guard does not rewrite the
//      estimate (the Localizer owns the pose; a diagnostic that mutates the data
//      path is worse than the bug it hunts — principle 4). The watchdogs and the
//      fault policy bound the damage; E-phase correctors may additionally gate on
//      this fault. EPISODE-gated like HealthMonitor: a jump that persists is one
//      episode, not a 100 Hz fault storm; a healthy tick re-arms.
//   2. COMMAND WITHIN CAPABILITY (commandWithinCapability, called by the command
//      pipeline after its own clamps): the final body command must respect the
//      configured budgets. The pipeline just ENFORCED those clamps, which is the
//      point — this is a defense-in-depth self-check, and a violation means the
//      pipeline itself regressed (the class of bug no closed-loop test can see —
//      C4's M21 lesson). Deliberately NOT episode-gated: it has no home for state
//      in a free-function pipeline, and a persistent pipeline regression SHOULD be
//      loud (the latch's saturating tally bounds the damage).
//   3. WHEEL VOLTS CONSISTENT (recoverWheelVoltage, same call site): each commanded
//      volt finite and within the battery ceiling. Recovery is REAL here: a
//      non-finite volt becomes 0 V, an over-ceiling volt is clamped — the bad value
//      never reaches a motor.
//
// The margins: margin (default 1.5) absorbs what the estimate may legitimately do
// beyond commanded physics — fusion nudges (never-snap-clamped, but nonzero),
// discretization, settle chatter. kCommandAuditMargin (1.01) is float headroom
// over an exact clamp. Both are logic constants. The PHYSICAL MAXIMA defaults are
// hardware claims: PROVISIONAL (A4: HA-56) — generous upper bounds no VEX
// drivetrain approaches, so a false positive requires the estimate to be wrong by
// construction; R3/R5 replace them with measured envelopes.

#include <cmath>
#include <cstdio>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// The physical envelope PoseDeltaGuard judges a tick's pose delta against. The defaults are
/// deliberately GENEROUS hardware claims, not a tuned trip point: they sit far above anything a
/// VEX drivetrain reaches, so a false positive requires the estimate to be wrong by
/// construction. That makes this a bug detector, not a performance limit — tightening it toward
/// the real envelope trades that guarantee for sensitivity. Nothing here bounds a COMMAND.
struct PlausibilityConfig {
    /// Physical maximum linear speed the robot could conceivably reach.
    /// PROVISIONAL (A4: HA-56) — a 600 rpm 4" drive tops out near 125 in/s.
    units::Velocity maxSpeed{150.0};
    /// Physical maximum yaw rate. PROVISIONAL (A4: HA-56) — ~3 rev/s is far past
    /// any real chassis.
    units::AngularVelocity maxYawRate{20.0};
    /// Headroom multiplier over the physical maxima (fusion nudges, discretization
    /// — header note). Logic constant. Must be >= 1.
    double margin = 1.5;

    /// Raise a LOUD precondition if any field is unusable (non-finite or non-positive maxima,
    /// margin < 1). Note the polarity — and note what it is NOT: SHULIB_PRECONDITION does not
    /// crash. It throws a catchable PreconditionError, and the on-robot policy raises
    /// FaultCode::Precondition on the latch BEFORE throwing (core/check.hpp). What separates
    /// this from the header's three invariants is RECOVERY, not loudness: they raise
    /// Implausible mid-tick and the run continues on a safe value, while this runs at
    /// CONSTRUCTION — PoseDeltaGuard's ctor, so in practice while the scheduler is being built
    /// at setup — when no motion is in flight for the scheduler's task boundary to convert the
    /// throw into a FAULT_ABORT. It therefore leaves the constructor rather than costing one
    /// motion, which is the intent: a nonsense envelope is a programming error in the setup,
    /// not a runtime anomaly the guard is here to survive. PoseDeltaGuard's constructor already
    /// calls it; call it yourself only when you build a config without one.
    void validate() const {
        SHULIB_PRECONDITION(std::isfinite(maxSpeed.value()) && maxSpeed.value() > 0.0,
                            "PlausibilityConfig: maxSpeed must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(maxYawRate.value()) && maxYawRate.value() > 0.0,
                            "PlausibilityConfig: maxYawRate must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(margin) && margin >= 1.0,
                            "PlausibilityConfig: margin must be finite and >= 1");
    }
};

/// Invariant 1 (header): per-tick pose delta within the physical envelope.
class PoseDeltaGuard {
public:
    /// COPIES `config` and validates it (loud on a nonsense envelope), so later edits to the
    /// caller's config never reach this guard. Starts with NO baseline: the first check() only
    /// records a pose and returns false, because one sample is not yet a delta.
    explicit PoseDeltaGuard(const PlausibilityConfig& config = {}) : cfg_{config} {
        cfg_.validate();
    }

    /// Feed one tick's estimate. `dt` is the measured tick dt; dt <= 0 (the
    /// baseline tick after construction/reset) only re-baselines — there is no
    /// interval to judge. Returns true iff THIS tick's delta is implausible
    /// (raising Implausible on the latch only on a NEW episode — header note).
    bool check(const math::Pose2d& pose, units::Time dt, FaultLatch& faults) {
        if (!hasPrev_ || dt.value() <= 0.0) {
            hasPrev_ = true;
            prev_ = pose;
            return false;
        }
        const double dist = std::hypot(pose.x().value() - prev_.x().value(),
                                       pose.y().value() - prev_.y().value());
        const double dHead = std::abs(pose.heading().errorTo(prev_.heading()));
        const double maxDist = cfg_.maxSpeed.value() * cfg_.margin * dt.value();
        const double maxHead = cfg_.maxYawRate.value() * cfg_.margin * dt.value();
        prev_ = pose;
        const bool bad = dist > maxDist || dHead > maxHead;
        if (bad && !episode_) {
            episode_ = true;
            char buf[112];
            if (dist > maxDist) {
                std::snprintf(buf, sizeof buf, "pose delta %.2fin > max %.2fin (dt=%.4fs)",
                              dist, maxDist, dt.value());
            } else {
                std::snprintf(buf, sizeof buf,
                              "heading delta %.3frad > max %.3frad (dt=%.4fs)", dHead,
                              maxHead, dt.value());
            }
            faults.raise(FaultCode::Implausible, "DIAG", buf);
        } else if (!bad) {
            episode_ = false;  // healthy tick: re-arm (recovery, observable by test)
        }
        return bad;
    }

    /// Forget the baseline before a DELIBERATE teleport (e.g. setPose re-seeding
    /// between runs) so intent is not reported as pathology.
    void reset() noexcept { hasPrev_ = false; episode_ = false; }

private:
    PlausibilityConfig cfg_;
    math::Pose2d prev_{};
    bool hasPrev_ = false;
    bool episode_ = false;
};

/// Float headroom for the pipeline self-checks (invariants 2/3): the values were
/// clamped by the same arithmetic that audits them, so anything past 1% is a real
/// regression, not rounding.
inline constexpr double kCommandAuditMargin = 1.01;

/// Invariant 2 (header): the FINAL body command respects the configured budgets.
/// True = plausible. False = Implausible raised (caller decides recovery; the
/// shipped pipeline's own clamps make reaching false a pipeline regression).
[[nodiscard]] inline bool commandWithinCapability(const math::ChassisSpeeds& body,
                                                  units::Velocity maxLinear,
                                                  units::AngularVelocity maxAngular,
                                                  FaultLatch& faults,
                                                  std::string_view subsystem) noexcept {
    const double vx = body.vx().value();
    const double vy = body.vy().value();
    const double w = body.omega().value();
    const double norm = std::hypot(vx, vy);
    const bool finiteOk = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(w);
    const bool linOk = finiteOk && norm <= maxLinear.value() * kCommandAuditMargin;
    const bool angOk = finiteOk && std::abs(w) <= maxAngular.value() * kCommandAuditMargin;
    if (linOk && angOk) {
        return true;
    }
    char buf[112];
    std::snprintf(buf, sizeof buf,
                  "command outside capability: |v|=%.2f max %.2f, |w|=%.3f max %.3f", norm,
                  maxLinear.value(), std::abs(w), maxAngular.value());
    faults.raise(FaultCode::Implausible, subsystem, buf);
    return false;
}

/// Invariant 3 (header): one wheel volt, made consistent with the battery ceiling.
/// ALWAYS returns a safe value (the FiniteGuard shape): finite in-range volts pass
/// untouched; non-finite → Implausible + 0 V; over-ceiling → Implausible + clamped.
[[nodiscard]] inline units::Voltage recoverWheelVoltage(units::Voltage v,
                                                        units::Voltage ceiling,
                                                        FaultLatch& faults,
                                                        std::string_view subsystem) noexcept {
    const double volts = v.value();
    const double cap = std::abs(ceiling.value()) * kCommandAuditMargin;
    if (std::isfinite(volts) && std::abs(volts) <= cap) {
        return v;
    }
    char buf[80];
    std::snprintf(buf, sizeof buf, "wheel volt %s ceiling %.2fV",
                  std::isfinite(volts) ? "beyond" : "non-finite vs", ceiling.value());
    faults.raise(FaultCode::Implausible, subsystem, buf);
    if (!std::isfinite(volts)) {
        return units::Voltage{0.0};
    }
    return units::Voltage{volts > 0.0 ? cap : -cap};
}

}  // namespace shulib::diag
