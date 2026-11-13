#pragma once
//
// sim::ArmPlant and sim::FlywheelPlant — the host test plant's MECHANISMS (the mechanism core:
// MECH-A2 built the scaffolding and its own tests over it; MECH-A3, 2026-10-15, gave the joint the
// two SENSORS a position mechanism homes by — an angle sensor that is ABSOLUTE within one turn at
// power-up, mounted either way round, and a limit SWITCH pressed at the joint's end of travel — and
// runs the position and speed behaviours' tests over it). Say "host test plant", never "simulator":
// nothing here is a measured motor.
//
// ── The model, in the A2 drive plant's own style ─────────────────────────────────────
// NO mass, NO torque curve — the INVERTED feedforward relation the drive plant uses
// (sim/motor_model.hpp), with a gravity LOAD in volts and a first-order approach:
//
//     V_eff  = V_applied − G(θ)          G = kG·cos(θ − level) for an arm, kG for a lift (a LOAD)
//     ω_ss   = sign(V_eff)·(|V_eff| − kS)/kV      (0 inside the static band |V_eff| ≤ kS)
//     ω      → ω_ss with τ = kA/kV                 (the exact exponential step; kA 0 = memoryless)
//
// plus what a mechanism has that a drive wheel does not:
//   * HARD STOPS — the joint cannot pass them; pushing into one reads ZERO speed and stall-grade
//     current (a real stalled motor CREEPS a little — HA-93 — which this plant does not);
//   * an OBSTACLE — a stop that appears mid-run ABOVE the joint (the blocked arm moving up), a WEDGE
//     — one BELOW it (a piece wedged under the arm, MECH-A3), and a JAM — the joint pinned where it is,
//     both ways (a roller jammed on a piece);
//   * the BRAKE MODE at 0 V commanded: Hold drives back toward the angle where the command stopped
//     (a restoring voltage, `holdStiffness` volts per joint radian), Brake damps (`brakeDamping`
//     volts per joint rad/s), Coast is free. Each is the voltage the motor itself APPLIES, clamped to
//     ±12 V like any command — and it draws current like one (an arm held under Hold draws what its
//     hold costs, which a plant that charged only the COMMANDED volts would hide). Both are solved
//     IMPLICITLY — the brake's volts from the speed at the END of the step, the hold's from the angle
//     at the end of it — so neither can ring: an explicit "volts from the last step's speed" made a
//     memoryless joint flip its speed's sign every step once brakeDamping exceeded kV (found by this
//     file's own test, MECH-A2 — a Brake that fell FASTER than Coast), and a stiff Hold on a fast joint
//     the same way once holdStiffness·dt exceeded kV. The steady state is the explicit form's exactly.
// CURRENT: I = |V_applied − kV·ω|·(stallAmps/12), clamped at stallAmps — 2.5 A at a 12 V stall
// (HA-93's shape: the V5 firmware's 2.5 A default limit, `pros/motors.h`).
//
// THE MOTOR'S OWN CURRENT LIMIT (chunk MECH-S-A, 2026-11-05): a joint honours motor 0's
// currentLimit() (hal::IMotorCurrentLimit; every motor of a group is set alike) ONLY when a limit BELOW
// the motor's 2.5 A default is set. Then, within each step, the applied volts are limited so
// |V − kV·ω| ≤ limit·12/stallAmps — the limit's TORQUE, not only its reading — re-solved at the END
// speed of the step, the stops included (the way the 12 V clamp is re-solved), so a joint stopped at a
// hard stop publishes at most the limit and pushes with at most its torque, and the Hold brake at 0 V is
// limited like any command. A load the limit cannot carry even when back-driven falls as fast as −12 V
// drives it (this plant has no mass to fall with — an approximation, said), its reading clamped at the
// limit. AT THE DEFAULT THE LIMITER IS BYPASSED, so every result before MECH-S-A is unchanged (its test
// pins a reversal's samples byte for byte) — including an EXISTING inaccuracy it does not fix: the
// plant clamps only its READING at 2.5 A, so a 12 V push flipped to −12 V drives |V − kV·ω| past 2.5 A's
// torque for a couple of ticks (the MECH-S brief's p_plantlimit: up to 18.26 V). Like every constant
// here the limiter is INVENTED (HA-183); whether a real motor holds its limit like this is HA-219. The
// flywheel plant below has no limiter (no test needs one; a flywheel's cap is the motor's, as for any
// mechanism).
//
// A POWER roller (an intake, a claw pivot) is this joint with no gravity (kG 0) and stops far away:
// the mechanism core's own tests drive power mechanisms through it.
//
// ── The CONTROLLER's constants and the PLANT's are set separately, on purpose ────────
// Every adversarial case mismatches them (the A2 truth-integrator lesson): a test that gives the
// controller and the plant the same kG proves only the algebra. EVERY constant here is INVENTED —
// the inverted-feedforward shape, the kS/kV/kG/τ a test chooses, the Hold stiffness, the Brake
// damping, the current's shape, a hard stop's ZERO speed — and none is a measured motor: HA-183
// (settled by the first robot session that logs a real arm's and a real flywheel's step response
// beside the model's).
//
// ── Readings ─────────────────────────────────────────────────────────────────────────
// Each step() reads motor 0's COMMANDED voltage and brake mode (every motor of a group is commanded
// alike — hal::MotorMechanism's fan-out) and publishes into EVERY fake motor: position (motor
// radians: the joint angle from `encoderZeroDeg`, times the gear ratio), velocity (motor rad/s) and
// current; when one is attached, into a FakeRotation — by default the joint's angle itself; with a
// RotationMount, what HA-186 believes a V5 rotation sensor reads: at attach (power-up) its
// `readsAtJointZeroDeg` plus the joint angle (MINUS it when mounted reversed), wrapped into one turn
// [0, 360), and from there CUMULATIVE, never wrapping again; and, when one is attached, into a
// FakeDigitalIn: a limit switch PRESSED while the joint is at or below (or at or above) its angle,
// written at its pressed level (HA-184's belief is the test's to choose).
// Ground truth (trueDeg(), trueSpeedDegS(), trueRpm()) is for the harness and assertions only —
// shulib/sim/ is unreachable from the library (the CI layering guard).

#include <algorithm>
#include <cmath>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_digital_in.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/motor_current_limit.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// One joint's constants (header: every one INVENTED, HA-183). Angles in JOINT degrees; volts.
struct ArmPlantConfig {
    double gearRatio = 5.0;               ///< MOTOR turns per JOINT turn (> 0)
    double kS = 0.3;                      ///< static band, V (>= 0)
    double kV = 1.2;                      ///< V per joint rad/s (> 0)
    double kA = 0.0;                      ///< V per joint rad/s^2 (>= 0): τ = kA/kV
    double kG = 0.0;                      ///< the gravity load, V: at level (arm) or everywhere (lift)
    bool constantGravity = false;         ///< a lift's load (true) or an arm's cosine (false)
    double levelDeg = 0.0;                ///< the joint angle at which the arm is level
    double minStopDeg = -1.0e6;           ///< the lower hard stop
    double maxStopDeg = 1.0e6;            ///< the upper hard stop (> minStopDeg)
    double stallAmps = 2.5;               ///< the current at a 12 V stall, A (> 0)
    double holdStiffness = 40.0;          ///< Hold at 0 V: restoring volts per joint rad of sag
    double brakeDamping = 3.0;            ///< Brake at 0 V: damping volts per joint rad/s
    double startDeg = 0.0;                ///< where the joint is at construction
    double encoderZeroDeg = 0.0;          ///< the joint angle at which the motor encoders read 0
};

/// How an angle sensor is mounted on the joint (header: HA-186's belief, modelled — never measured):
/// what it reads at power-up when the joint is at 0, and whether it turns the other way round.
struct RotationMount {
    /// Mounted so its reading DEcreases as the joint angle increases.
    bool reversed = false;
    /// What it would read, degrees, with the joint at 0 at power-up (any value: the reading is wrapped
    /// into one turn at attach).
    double readsAtJointZeroDeg = 0.0;
};

/// A limit switch on the joint (header): PRESSED while the joint is at or below `atOrBelowDeg` (or,
/// with `atTop`, at or above it), written at `pressedLevel` (HA-184's belief — the test chooses).
struct SwitchMount {
    /// The joint angle at which it is pressed, degrees.
    double atDeg = 0.0;
    /// Pressed at or ABOVE `atDeg` (the top of the travel) instead of at or below it.
    bool atTop = false;
    /// The level a pressed switch reads (true: pressed reads 1 — HA-184's belief).
    bool pressedLevel = true;
};

/// One joint driven by N fake motors (header). Not copyable: it holds the motors' span.
class ArmPlant {
public:
    /// `motors` (non-empty, non-null) must outlive the plant. PRECONDITIONS: gearRatio, kV and
    /// stallAmps finite and > 0; kS, kA, holdStiffness, brakeDamping finite and >= 0; the stops in
    /// order with startDeg between them.
    ArmPlant(std::span<hal::fake::FakeMotor* const> motors, const ArmPlantConfig& cfg)
        : motors_{motors}, cfg_{cfg}, thetaRad_{cfg.startDeg * kD2R}, heldRad_{cfg.startDeg * kD2R} {
        SHULIB_PRECONDITION(!motors_.empty(), "ArmPlant: no motors");
        for (const hal::fake::FakeMotor* m : motors_) {
            SHULIB_PRECONDITION(m != nullptr, "ArmPlant: a motor is null");
        }
        SHULIB_PRECONDITION(cfg_.gearRatio > 0.0 && cfg_.kV > 0.0 && cfg_.stallAmps > 0.0 &&
                                std::isfinite(cfg_.gearRatio) && std::isfinite(cfg_.kV) &&
                                std::isfinite(cfg_.stallAmps),
                            "ArmPlant: gearRatio, kV and stallAmps must be finite and > 0");
        SHULIB_PRECONDITION(cfg_.kS >= 0.0 && cfg_.kA >= 0.0 && cfg_.holdStiffness >= 0.0 &&
                                cfg_.brakeDamping >= 0.0 && std::isfinite(cfg_.kS) && std::isfinite(cfg_.kA) &&
                                std::isfinite(cfg_.holdStiffness) && std::isfinite(cfg_.brakeDamping),
                            "ArmPlant: kS, kA, holdStiffness and brakeDamping must be finite and >= 0");
        SHULIB_PRECONDITION(cfg_.minStopDeg < cfg_.maxStopDeg && cfg_.startDeg >= cfg_.minStopDeg &&
                                cfg_.startDeg <= cfg_.maxStopDeg,
                            "ArmPlant: the stops must be in order with the start between them");
        publish(0.0);
    }

    /// A game piece: an extra load in volts, through the same cosine as the arm (a lift: constant).
    void addLoadVolts(double extra) noexcept { extraLoad_ = extra; }
    /// A stop that appears ABOVE the joint (the blocked arm moving up); a joint already above it
    /// moves freely.
    void setObstacleDeg(double deg) noexcept {
        obstacleRad_ = deg * kD2R;
        hasObstacle_ = true;
    }
    /// The obstacle goes away.
    void clearObstacle() noexcept { hasObstacle_ = false; }
    /// A stop that appears BELOW the joint (a piece wedged under the arm — what a stall-home would take
    /// for its stop); a joint already below it moves freely.
    void setWedgeDeg(double deg) noexcept {
        wedgeRad_ = deg * kD2R;
        hasWedge_ = true;
    }
    /// The wedged piece is cleared.
    void clearWedge() noexcept { hasWedge_ = false; }
    /// The joint is pinned where it is, both ways (a roller jammed on a piece), or freed.
    void setJammed(bool jammed) noexcept { jammed_ = jammed; }
    /// Publish the joint's absolute angle into `r` too (an angle sensor on the output shaft that
    /// reads the joint angle itself, never wrapped).
    void attachRotation(hal::fake::FakeRotation* r) {
        rotation_ = r;
        mounted_ = false;
        publish(appliedVolts_);
    }
    /// An angle sensor mounted as `m` (header): at THIS moment — the power-up — it reads the joint
    /// angle through its mounting, wrapped into [0, 360); afterwards cumulatively.
    void attachRotation(hal::fake::FakeRotation* r, const RotationMount& m) {
        rotation_ = r;
        mounted_ = true;
        mountSign_ = m.reversed ? -1.0 : 1.0;
        const double atPowerUp = m.readsAtJointZeroDeg + mountSign_ * trueDeg();
        const double wrapped = atPowerUp - 360.0 * std::floor(atPowerUp / 360.0);
        mountBaseRad_ = wrapped * kD2R - mountSign_ * thetaRad_;  // reading = base + sign * theta
        publish(appliedVolts_);
    }
    /// A limit switch on the joint (header), published into `sw` on every step from now on.
    void attachSwitch(hal::fake::FakeDigitalIn* sw, const SwitchMount& m) {
        switch_ = sw;
        switchMount_ = m;
        publish(appliedVolts_);
    }
    /// GROUND TRUTH, joint degrees (assertions only).
    [[nodiscard]] double trueDeg() const noexcept { return thetaRad_ / kD2R; }
    /// GROUND TRUTH, joint degrees per second.
    [[nodiscard]] double trueSpeedDegS() const noexcept { return omega_ / kD2R; }
    /// The voltage the motor APPLIED in the last step (the command, or its brake mode's at 0 V).
    [[nodiscard]] double appliedVolts() const noexcept { return appliedVolts_; }

    /// Advance `dt` seconds (> 0) under the motors' current command and brake mode (header).
    void step(double dt) {
        SHULIB_PRECONDITION(std::isfinite(dt) && dt > 0.0, "ArmPlant::step: dt must be finite and > 0");
        const double v = motors_[0]->commandedVoltage().value();
        const bool commanded = v != 0.0;
        // The applied voltage is a0 − c·ω_next (header: the brake modes are IMPLICIT in the end speed).
        double a0 = v;
        double c = 0.0;
        if (!commanded) {
            switch (motors_[0]->brakeMode()) {
                case hal::BrakeMode::Hold:  // −S·(θ_next − held), θ_next = θ + ω_next·dt
                    a0 = -cfg_.holdStiffness * (thetaRad_ - heldRad_);
                    c = cfg_.holdStiffness * dt;
                    break;
                case hal::BrakeMode::Brake:  // −B·ω_next
                    a0 = 0.0;
                    c = cfg_.brakeDamping;
                    break;
                case hal::BrakeMode::Coast: a0 = 0.0; break;
            }
        }
        if (jammed_) {
            omega_ = 0.0;
            appliedVolts_ = std::clamp(a0, -12.0, 12.0);
            if (limited()) {  // pinned: ω is 0, so the limit is on the volts themselves (header)
                appliedVolts_ = std::clamp(appliedVolts_, -gapVolts(), gapVolts());
            }
            if (commanded) {
                heldRad_ = thetaRad_;  // pinned: where it is IS where the command leaves it
            }
            publish(appliedVolts_);
            return;
        }
        Advance r = advance(a0, c, dt);
        if (limited()) {  // the motor's own limit (header): its torque, re-solved at the end speed
            const double gap = gapVolts();
            const double f = r.applied - cfg_.kV * r.omega;
            if (f > gap || f < -gap) {
                r = limitedAdvance(dt, gap, f > 0.0, r.applied);
            }
        }
        omega_ = r.omega;
        const double applied = r.applied;
        appliedVolts_ = applied;
        thetaRad_ = r.next;
        if (commanded) {
            // A Hold captures where the command STOPPED: the angle at the END of the last commanded
            // step — where the shaft is when the next step's 0 V engages Hold. (Captured before this
            // step's motion, it held one step BEHIND, and a fast joint was driven back a full step of
            // travel the moment it stopped — found by the MECH-A2 review: 17.6° back from 6 V.)
            heldRad_ = thetaRad_;
        }
        publish(applied);
    }

private:
    static constexpr double kD2R = 3.14159265358979323846 / 180.0;
    /// One step's outcome before it is kept: the end speed, the applied volts, the end angle.
    struct Advance {
        double omega;
        double applied;
        double next;
    };
    /// One step from the current state under a0 − c·ω (header): the speed, the 12 V clamp re-solved,
    /// then the hard stops (a joint pushing into one ends it at ZERO speed). Changes nothing.
    [[nodiscard]] Advance advance(double a0, double c, double dt) const {
        Advance r{approach(omega_, a0, c, dt), 0.0, 0.0};
        r.applied = a0 - c * r.omega;
        if (std::abs(r.applied) > 12.0) {  // the motor cannot apply more: the clamped volts, solved again
            r.applied = std::clamp(r.applied, -12.0, 12.0);
            r.omega = approach(omega_, r.applied, 0.0, dt);
        }
        double next = thetaRad_ + r.omega * dt;
        const double loStop = cfg_.minStopDeg * kD2R;
        const double lo = hasWedge_ && thetaRad_ >= wedgeRad_ ? std::max(loStop, wedgeRad_) : loStop;
        const double hiStop = cfg_.maxStopDeg * kD2R;
        const double hi = hasObstacle_ && thetaRad_ <= obstacleRad_ ? std::min(hiStop, obstacleRad_) : hiStop;
        if (next <= lo) {
            next = lo;
            if (r.omega < 0.0) {
                r.omega = 0.0;
            }
        }
        if (next >= hi) {
            next = hi;
            if (r.omega > 0.0) {
                r.omega = 0.0;
            }
        }
        r.next = next;
        return r;
    }
    /// True when motor 0 holds a limit BELOW the motor's 2.5 A default (header: at the default the
    /// limiter is bypassed).
    [[nodiscard]] bool limited() const { return motors_[0]->currentLimit() < hal::kDefaultMotorCurrentLimit; }
    /// The limit as volts of |V − kV·ω| (the current's own scale: stallAmps at 12 V).
    [[nodiscard]] double gapVolts() const {
        return motors_[0]->currentLimit().value() * 12.0 / cfg_.stallAmps;
    }
    /// The step at the volts nearest `applied` whose end-speed torque is inside ±gap (header). V − kV·ω at
    /// the end of a step never decreases as V grows (inside the static band it is V; moving, the exact
    /// step keeps a part of V; into a stop it is V), so the volts that keep it inside are one interval and
    /// a bisection finds its edge. `pushingUp`: the step's torque was ABOVE +gap (else below −gap). A load
    /// the limit cannot carry even at the far end (−12 V or +12 V) gets that end's step (header).
    [[nodiscard]] Advance limitedAdvance(double dt, double gap, bool pushingUp, double applied) const {
        const auto torque = [this](const Advance& a) { return a.applied - cfg_.kV * a.omega; };
        double inside = pushingUp ? -12.0 : 12.0;  // the end that must be inside, if anything is
        double outside = applied;                   // outside, by the caller's test
        Advance best = advance(inside, 0.0, dt);
        const bool reachable = pushingUp ? torque(best) <= gap : torque(best) >= -gap;
        if (!reachable) {
            return best;
        }
        for (int i = 0; i < 60; ++i) {
            const double mid = 0.5 * (inside + outside);
            const Advance m = advance(mid, 0.0, dt);
            const bool ok = pushingUp ? torque(m) <= gap : torque(m) >= -gap;
            if (ok) {
                inside = mid;
                best = m;
            } else {
                outside = mid;
            }
        }
        return best;
    }
    /// The joint speed after `dt` from `w0` when the motor applies a0 − c·ω (c > 0: a brake mode's
    /// implicit part): the steady speed solves kV·ω = a0 − c·ω − G ∓ kS, approached with τ = kA/(kV + c).
    [[nodiscard]] double approach(double w0, double a0, double c, double dt) const {
        const double veff = a0 - gravity();
        const double kv = cfg_.kV + c;
        const double oss = std::abs(veff) <= cfg_.kS ? 0.0 : std::copysign((std::abs(veff) - cfg_.kS) / kv, veff);
        return cfg_.kA <= 0.0 ? oss : oss + (w0 - oss) * std::exp(-dt * kv / cfg_.kA);
    }
    [[nodiscard]] double gravity() const noexcept {
        const double shape = cfg_.constantGravity ? 1.0 : std::cos(thetaRad_ - cfg_.levelDeg * kD2R);
        return (cfg_.kG + extraLoad_) * shape;
    }
    void publish(double applied) {
        const double motorRad = (thetaRad_ - cfg_.encoderZeroDeg * kD2R) * cfg_.gearRatio;
        const double motorRadS = omega_ * cfg_.gearRatio;
        double amps = std::min(cfg_.stallAmps, std::abs(applied - cfg_.kV * omega_) * cfg_.stallAmps / 12.0);
        if (limited()) {  // a limited motor reads at most its limit (header: the load it cannot carry)
            amps = std::min(amps, motors_[0]->currentLimit().value());
        }
        for (hal::fake::FakeMotor* m : motors_) {
            m->setPosition(units::AngleDim{motorRad});
            m->setVelocity(units::AngularVelocity{motorRadS});
            m->setCurrent(units::Current{amps});
        }
        if (rotation_ != nullptr) {
            const double sign = mounted_ ? mountSign_ : 1.0;
            rotation_->setPosition(units::AngleDim{mounted_ ? mountBaseRad_ + sign * thetaRad_ : thetaRad_});
            rotation_->setVelocity(units::AngularVelocity{sign * omega_});
        }
        if (switch_ != nullptr) {
            const double deg = trueDeg();
            const bool pressed = switchMount_.atTop ? deg >= switchMount_.atDeg : deg <= switchMount_.atDeg;
            switch_->setState(pressed ? switchMount_.pressedLevel : !switchMount_.pressedLevel);
        }
    }
    std::span<hal::fake::FakeMotor* const> motors_;
    ArmPlantConfig cfg_;
    double thetaRad_;
    double heldRad_;
    double omega_ = 0.0;
    double appliedVolts_ = 0.0;
    double extraLoad_ = 0.0;
    double obstacleRad_ = 0.0;
    bool hasObstacle_ = false;
    double wedgeRad_ = 0.0;
    bool hasWedge_ = false;
    bool jammed_ = false;
    hal::fake::FakeRotation* rotation_ = nullptr;
    bool mounted_ = false;       ///< attached with a RotationMount
    double mountSign_ = 1.0;     ///< -1 when mounted reversed
    double mountBaseRad_ = 0.0;  ///< the mounted reading = base + sign * theta
    hal::fake::FakeDigitalIn* switch_ = nullptr;
    SwitchMount switchMount_{};
};

/// A flywheel's constants (header: INVENTED, HA-183). Speeds in WHEEL rad/s; volts.
struct FlywheelPlantConfig {
    double gearRatio = 1.0;     ///< MOTOR turns per WHEEL turn (> 0; geared up 6:1 = 1/6)
    double kS = 0.4;            ///< static band, V (>= 0)
    double kV = 0.02;           ///< V per wheel rad/s (> 0)
    double kA = 0.004;          ///< V per wheel rad/s^2 (>= 0): τ = kA/kV (0.2 s here)
    double stallAmps = 2.5;     ///< the current at a 12 V stall, A (> 0)
    double brakeFraction = 0.5; ///< Brake or Hold at 0 V: this fraction of the back-EMF, applied against it (implicit)
};

/// A wheel driven by N fake motors (header): a first-order speed, a shot, a jam.
class FlywheelPlant {
public:
    /// `motors` (non-empty, non-null) must outlive the plant. PRECONDITIONS: gearRatio, kV and
    /// stallAmps finite and > 0; kS, kA and brakeFraction finite and >= 0.
    FlywheelPlant(std::span<hal::fake::FakeMotor* const> motors, const FlywheelPlantConfig& cfg)
        : motors_{motors}, cfg_{cfg} {
        SHULIB_PRECONDITION(!motors_.empty(), "FlywheelPlant: no motors");
        for (const hal::fake::FakeMotor* m : motors_) {
            SHULIB_PRECONDITION(m != nullptr, "FlywheelPlant: a motor is null");
        }
        SHULIB_PRECONDITION(cfg_.gearRatio > 0.0 && cfg_.kV > 0.0 && cfg_.stallAmps > 0.0 &&
                                std::isfinite(cfg_.gearRatio) && std::isfinite(cfg_.kV) &&
                                std::isfinite(cfg_.stallAmps),
                            "FlywheelPlant: gearRatio, kV and stallAmps must be finite and > 0");
        SHULIB_PRECONDITION(cfg_.kS >= 0.0 && cfg_.kA >= 0.0 && cfg_.brakeFraction >= 0.0 &&
                                std::isfinite(cfg_.kS) && std::isfinite(cfg_.kA) && std::isfinite(cfg_.brakeFraction),
                            "FlywheelPlant: kS, kA and brakeFraction must be finite and >= 0");
    }
    /// A shot: the wheel loses `fraction` (0..1) of its speed at once (a piece taking energy).
    void shot(double fraction) noexcept { omega_ *= (1.0 - std::clamp(fraction, 0.0, 1.0)); }
    /// The wheel is jammed (it cannot turn), or freed.
    void setJammed(bool jammed) noexcept { jammed_ = jammed; }
    /// GROUND TRUTH, wheel rpm (assertions only).
    [[nodiscard]] double trueRpm() const noexcept { return omega_ * 60.0 / (2.0 * 3.14159265358979323846); }

    /// Advance `dt` seconds (> 0) under the motors' command and brake mode.
    void step(double dt) {
        SHULIB_PRECONDITION(std::isfinite(dt) && dt > 0.0, "FlywheelPlant::step: dt must be finite and > 0");
        double applied = motors_[0]->commandedVoltage().value();
        // Brake or Hold at 0 V: −brakeFraction·kV·ω_next, IMPLICIT in the end speed (ArmPlant's reason).
        const double c = applied == 0.0 && motors_[0]->brakeMode() != hal::BrakeMode::Coast
                             ? cfg_.brakeFraction * cfg_.kV
                             : 0.0;
        if (jammed_) {
            omega_ = 0.0;
        } else {
            const double kv = cfg_.kV + c;
            const double oss =
                std::abs(applied) <= cfg_.kS ? 0.0 : std::copysign((std::abs(applied) - cfg_.kS) / kv, applied);
            omega_ = cfg_.kA <= 0.0 ? oss : oss + (omega_ - oss) * std::exp(-dt * kv / cfg_.kA);
            applied -= c * omega_;
        }
        motorPos_ += omega_ * cfg_.gearRatio * dt;
        const double amps = std::min(cfg_.stallAmps, std::abs(applied - cfg_.kV * omega_) * cfg_.stallAmps / 12.0);
        for (hal::fake::FakeMotor* m : motors_) {
            m->setPosition(units::AngleDim{motorPos_});
            m->setVelocity(units::AngularVelocity{omega_ * cfg_.gearRatio});
            m->setCurrent(units::Current{amps});
        }
    }

private:
    std::span<hal::fake::FakeMotor* const> motors_;
    FlywheelPlantConfig cfg_;
    double omega_ = 0.0;
    double motorPos_ = 0.0;
    bool jammed_ = false;
};

}  // namespace shulib::sim
