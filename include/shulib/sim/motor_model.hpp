#pragma once
//
// sim::MotorModel — voltage → wheel surface velocity, by INVERTING the existing
// feedforward relation (chunk A2). This is the plant's only "dynamics", and its
// scope is a deliberate, load-bearing decision:
//
// ── THE HONESTY BOUNDARY (read this before "improving" the model) ───────────────────
// This plant proves LOGIC, not CONSTANTS. It deliberately models NO mass, NO inertia
// tensor, NO motor torque curve, NO friction coefficient, NO slip physics — those
// parameters are unmeasurable without a robot, and a plant tuned on invented constants
// produces confident, WRONG answers, which is worse than an honestly limited one
// because it gets trusted (build-order § "The missing prerequisite"; A2 brief
// "Explicitly rejected"). Gains tuned against this plant are PROVISIONAL until R5
// (sysid on real hardware) and R6 (measured constants fed back into this plant).
//
// What IS modeled is exactly the relation the control stack already owns:
//
//     V = kS·sign(v) + kV·v + kA·a          (control::Feedforward, M2/WS4)
//
// inverted. At steady state (a = 0):
//
//     |V| <= kS  →  v = 0                   (the static-friction dead band)
//     |V| >  kS  →  v = sign(V)·(|V| − kS)/kV
//
// and the kA term supplies the approach: treating the relation as the first-order ODE
// kA·v̇ = V − kS·sign(v) − kV·v gives an exponential approach to that steady state
// with time constant τ = kA/kV. The per-tick update uses the EXACT solution of the
// linearized ODE (v → v_ss + (v − v_ss)·e^(−dt/τ)), so the step size never affects
// stability — only the honesty boundary above bounds the model's realism.
//
// THE DEFINING, TESTABLE PROPERTY (what "inverting the existing relation" buys):
// commanding Feedforward::calculate(v) holds the wheel at exactly v at steady state.
// That is the property every C-phase feedforward+PID loop depends on, and it is
// pinned by a sweep in test/sim_motor_model_test.cpp — with the kS term live, so
// dropping kS from either side goes red (a required A2 mutation check).
//
// Two documented simplifications (both transient-only; steady state is exact):
//   * During the approach, the friction term follows the STEADY-STATE sign (the sign
//     of v_ss), not the instantaneous sign(v) — the piecewise-sticky crossing of
//     v = 0 is not modeled. A reversal therefore decays smoothly through zero. That
//     is approach REALISM, not physics; modeling stiction honestly needs R5 data.
//   * kA = 0 means an instantaneous jump to v_ss (τ = 0): the model degrades cleanly
//     to the memoryless inversion, which is what most A2 logic tests use because it
//     makes open-loop distances exactly derivable by hand.
//
// STATELESS ON PURPOSE: advance(v, V, dt) is a pure function of its arguments, so the
// plant owns the per-wheel state (one double each) and this class is exhaustively
// testable point-by-point with no hidden history. Units: v is the WHEEL SURFACE speed
// (in/s) — the same currency as kinematics::WheelSpeeds — so kV here is volt·s/in per
// WHEEL, exactly what R5's sysid will measure.

#include <cmath>

#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

class MotorModel {
public:
    /// Gains are the SAME struct sysid produces and control::Feedforward consumes —
    /// one vocabulary, so R6's "feed measured constants back into the plant" is a
    /// config change, not a port. Preconditions: kV > 0 (the inversion divides by
    /// it), kS >= 0, kA >= 0, all finite.
    explicit MotorModel(const control::FeedforwardGains& gains) : g_{gains} {
        SHULIB_PRECONDITION(std::isfinite(g_.kS) && std::isfinite(g_.kV) && std::isfinite(g_.kA),
                            "MotorModel: gains must be finite");
        SHULIB_PRECONDITION(g_.kV > 0.0, "MotorModel: kV must be > 0 (the inversion divides by it)");
        SHULIB_PRECONDITION(g_.kS >= 0.0, "MotorModel: kS must be >= 0");
        SHULIB_PRECONDITION(g_.kA >= 0.0, "MotorModel: kA must be >= 0");
    }

    /// One tick: the wheel's surface velocity after `dt` under `effective` volts,
    /// starting from `current`. Pure — no internal state. dt must be finite and
    /// >= 0; dt == 0 returns `current` unchanged (time did not pass).
    [[nodiscard]] units::Velocity advance(units::Velocity current, units::Voltage effective,
                                          units::Time dt) const {
        SHULIB_PRECONDITION(std::isfinite(dt.value()) && dt.value() >= 0.0,
                            "MotorModel::advance: dt must be finite and >= 0");
        const double v0 = current.value();
        const double vss = steadyState(effective.value());
        if (dt.value() == 0.0) {
            return current;
        }
        if (g_.kA <= 0.0) {
            return units::Velocity{vss};  // memoryless inversion (τ = 0)
        }
        // Exact solution of kA·v̇ = −kV·(v − v_ss): exponential approach, τ = kA/kV.
        // Unconditionally stable for any dt (e^(−dt/τ) ∈ (0, 1]) — no step-size cliff.
        const double decay = std::exp(-dt.value() * g_.kV / g_.kA);
        return units::Velocity{vss + (v0 - vss) * decay};
    }

private:
    /// The exact steady-state inversion of V = kS·sign(v) + kV·v (see header).
    [[nodiscard]] double steadyState(double volts) const noexcept {
        if (std::abs(volts) <= g_.kS) {
            return 0.0;  // inside the static-friction dead band
        }
        const double magnitude = (std::abs(volts) - g_.kS) / g_.kV;
        return (volts > 0.0) ? magnitude : -magnitude;
    }

    control::FeedforwardGains g_;
};

}  // namespace shulib::sim
