#pragma once
//
// sim::DegradationModel — the A3 injection SEAMS, deliberately empty (chunk A2,
// constraint 5: "build the hooks now, a policy/interface the plant consults, so A3
// is population, not surgery — leave them no-ops").
//
// ── What this is, and is NOT ────────────────────────────────────────────────────────
// The DrivePlant consults exactly one DegradationModel at every stage where V5
// hardware can lie: between the commanded voltage and the wheel, between the wheel
// and the floor, and between the physical truth and every sensor reading. THIS base
// class is the identity at every hook — a plant running it is the "perfect robot"
// A2 needs for logic proofs. The A3 hostile models live in sim/hostile/ (one class
// per sensor family + ChainedDegradation/FullHostility for composition); NOTHING
// here implements those behaviours — this file stays the seam definition, and any
// noise math belongs in a sim/hostile/ subclass, never in the base.
//
// ── The seam map (each hook names the A3 behaviour it exists for) ───────────────────
//   effectiveVoltage()      battery sag under load, brownout collapse, current
//                           limiting, thermal droop — the volts that REACH the motor.
//   wheelMotionVelocity()   wheel slip / traction loss: what the spinning wheel
//                           actually contributes to BODY MOTION. The plant reads the
//                           drive encoders from the SPIN (pre-slip) and moves the
//                           body from THIS (post-slip) — so injected slip manifests
//                           exactly as real slip does: encoders overcount while the
//                           robot undershoots, and odometry drifts accordingly.
//   driveEncoderPosition()  encoder quantization (V5 centidegree steps), dropout
//                           (a frozen reading), disconnection (sentinel behaviour —
//                           though NOTE the F4 finiteness contract: the hal/pros
//                           adapter screens PROS_ERR at the edge, so A3 should model
//                           what the CORE would see, which is a frozen/garbage-but-
//                           finite value, not a NaN).
//   trackingEncoderPosition() same, for the unpowered tracking wheels.
//   imuHeading()            per-boot bias, drift rate (~1°/min), noise — the direct
//                           attack on the < 1° budget.
//   imuYawRate()            rate noise/bias (feeds the Localizer's twist omega).
//   imuReady()              IMU dropout / mid-run disconnect (isReady() goes false).
//   gps()                   no-fix (off the Driving-Skills strip), bad-fix, error
//                           inflation, update latency (see the latency note below).
//   batteryVoltage()        the reported pack voltage sagging toward brownout.
//
// ── Two degradations that live elsewhere, on purpose ────────────────────────────────
//   * SENSOR LATENCY is a STATEFUL policy: an A3 subclass buffers the true values it
//     receives and returns older ones. The hooks deliberately receive (truth, now) so
//     a subclass CAN do that; no ring buffer belongs in the base class.
//   * LOOP JITTER / VARIABLE dt is the SCENARIO RUNNER's dt-schedule seam
//     (SimHarness::runTicks takes a per-tick dt provider), not a sensor lie — the
//     plant must integrate whatever dt actually elapsed.
//
// ── Determinism contract (constraint 4) ─────────────────────────────────────────────
// Every hook receives the plant's seeded Rng. An A3 model draws ONLY from it (never
// from wall-clock, never from a private unseeded source), so a hostile run replays
// byte-identically from its seed. The identity model draws nothing.
//
// Ownership/lifetime: non-owning reference held by DrivePlant; single-task, like the
// rest of the harness. Virtual-call cost is irrelevant here — this is host-only test
// infrastructure, never robot code.

#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// The GPS truth triple the gps() hook may degrade (pose is the canonical
/// robot-center pose; rmsError/hasFix mirror the IGps contract).
struct GpsTruth {
    math::Pose2d pose{};
    units::Length rmsError{};
    bool hasFix = true;
};

class DegradationModel {
public:
    virtual ~DegradationModel() = default;
    DegradationModel() = default;
    DegradationModel(const DegradationModel&) = default;
    DegradationModel(DegradationModel&&) = default;
    DegradationModel& operator=(const DegradationModel&) = default;
    DegradationModel& operator=(DegradationModel&&) = default;

    /// A3: sag/brownout/current-limit/thermal droop. Identity: the command arrives intact.
    [[nodiscard]] virtual units::Voltage effectiveVoltage(int /*wheel*/, units::Voltage commanded,
                                                          units::Time /*now*/, Rng& /*rng*/) {
        return commanded;
    }

    /// A3: wheel slip — spin vs. what actually moves the body. Identity: no slip.
    [[nodiscard]] virtual units::Velocity wheelMotionVelocity(int /*wheel*/, units::Velocity spin,
                                                              units::Time /*now*/, Rng& /*rng*/) {
        return spin;
    }

    /// A3: drive-encoder quantization/dropout. Identity: the true shaft angle.
    [[nodiscard]] virtual units::AngleDim driveEncoderPosition(int /*wheel*/, units::AngleDim trueShaft,
                                                               units::Time /*now*/, Rng& /*rng*/) {
        return trueShaft;
    }

    /// A3: tracking-encoder quantization/dropout/slip-on-bump. Identity: truth.
    [[nodiscard]] virtual units::AngleDim trackingEncoderPosition(int /*wheelIndex*/,
                                                                  units::AngleDim trueShaft,
                                                                  units::Time /*now*/, Rng& /*rng*/) {
        return trueShaft;
    }

    /// A3: IMU per-boot bias + drift + noise (the < 1° attack). Identity: truth.
    [[nodiscard]] virtual math::Angle imuHeading(math::Angle trueHeading, units::Time /*now*/,
                                                 Rng& /*rng*/) {
        return trueHeading;
    }

    /// A3: yaw-rate noise/bias. Identity: truth.
    [[nodiscard]] virtual units::AngularVelocity imuYawRate(units::AngularVelocity trueRate,
                                                            units::Time /*now*/, Rng& /*rng*/) {
        return trueRate;
    }

    /// A3: IMU dropout / mid-run disconnect. Identity: stays ready.
    [[nodiscard]] virtual bool imuReady(bool trueReady, units::Time /*now*/) { return trueReady; }

    /// A3: GPS no-fix/bad-fix/latency/error inflation. Identity: the truth triple.
    [[nodiscard]] virtual GpsTruth gps(const GpsTruth& truth, units::Time /*now*/, Rng& /*rng*/) {
        return truth;
    }

    /// A3: battery sag toward brownout. Identity: the configured nominal voltage.
    [[nodiscard]] virtual units::Voltage batteryVoltage(units::Voltage nominal, units::Time /*now*/,
                                                        Rng& /*rng*/) {
        return nominal;
    }
};

}  // namespace shulib::sim
