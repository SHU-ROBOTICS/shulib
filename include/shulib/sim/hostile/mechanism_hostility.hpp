#pragma once
//
// Mechanism hostility (chunk F1, per A3's pattern): the misbehaviours a
// mechanism's devices can produce, one pathology per model, injectable at the
// device seam. build-order.md §F1 names the three worlds these compose into —
// A JAMMED INTAKE, AN UNCONFIRMED GRAB, A STALLED LIFT:
//
//   * JammedMotor      — the honest jam: the shaft stops, the current climbs,
//                        the encoder tells the truth about it. A jammed intake
//                        and a stalled lift are ONE device signature with two
//                        narratives (when the window opens and what was
//                        commanded); the config expresses both.
//   * LyingSpinMotor   — the dishonest stall: the shaft truly stops (position
//                        freezes — the truth channel a test asserts against)
//                        but velocity and current keep reporting the healthy
//                        values. This is A3's "the fake must be able to lie"
//                        applied to F1: an operation that trusted its device
//                        would hang or false-succeed here; the watchdog is the
//                        only guard that needs no sensor honesty, and this
//                        model is how a test proves that.
//   * NeverConfirm /   — the confirm-channel hostility: the world where the
//     ConfirmAfter       jaws closed on nothing (never), or where confirmation
//                        arrives late (after t). ConfirmAfter{clock, 0} is the
//                        CONFIRM THAT LIES TRUE — instantly "confirmed" no
//                        matter what happened — used to demonstrate the trust
//                        boundary (mechanism_op.hpp: the predicate is trusted;
//                        choosing real sensors is F3's job).
//   There is deliberately NO "dead air" model for IDigitalOut: a real solenoid
//   has no feedback (digital_out.hpp), so dead air is INVISIBLE at the device
//   and hostility can only appear on the confirm channel — which is exactly
//   NeverConfirm. A model here would claim an observability the hardware does
//   not have.
//
// Composition, the A3 rule: motor models are DECORATORS over hal::IMotor, so a
// composed pathology is nesting (JammedMotor over a latency-shaped wrapper over
// a FakeMotor), and ablation — removing one wrapper — removes exactly that
// pathology. A liveness test pins that an armed jam measurably bites (A3's
// "a dead composed model cannot ship"; the F1 mutation list targets it).
//
// Magnitudes: the jam signature's defaults are INVENTED stand-ins for a V5 11W
// motor's stall behaviour — PROVISIONAL (A4: HA-93); R4 measures a real jam.
// LyingSpinMotor has NO defaults on purpose: "what healthy looked like" is a
// scenario fact the test states, not a physics claim the library should invent.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

struct JammedMotorConfig {
    /// The jam window [start, end): a mid-run jam (intake eats a wedged ring)
    /// or a from-the-start stall (lift commanded into its hard stop) are the
    /// same model with different instants.
    units::Time start{0.0};
    units::Time end{1.0e18};  // effectively forever
    /// Reported current at a full ±12 V command while jammed; scales linearly
    /// with |commanded| (stall current ~ V/R). PROVISIONAL (A4: HA-93).
    units::Current stallCurrentAt12V{2.5};
    /// Residual reported shaft speed while jammed (belt chatter, backlash) —
    /// small but non-zero so a detector keyed on EXACT zero is caught by it.
    /// PROVISIONAL (A4: HA-93).
    units::AngularVelocity creepSpeed{0.05};
};

/// The honest jam (header). Wraps a real/fake motor; passes commands through
/// untouched (a jammed motor still RECEIVES its commands — the driver keeps
/// pushing volts); corrupts the READINGS during the window.
class JammedMotor final : public hal::IMotor {
public:
    /// `inner` and `clock` must outlive the model.
    JammedMotor(hal::IMotor& inner, const hal::IClock& clock,
                const JammedMotorConfig& config = {})
        : inner_{&inner}, clock_{&clock}, cfg_{config} {
        SHULIB_PRECONDITION(cfg_.end.value() > cfg_.start.value(),
                            "JammedMotor: end must be after start");
        SHULIB_PRECONDITION(cfg_.stallCurrentAt12V.value() > 0.0,
                            "JammedMotor: stallCurrentAt12V must be > 0");
        SHULIB_PRECONDITION(cfg_.creepSpeed.value() >= 0.0,
                            "JammedMotor: creepSpeed must be >= 0");
    }

    // Commands pass through untouched.
    void setVoltage(units::Voltage volts) override { inner_->setVoltage(volts); }
    [[nodiscard]] units::Voltage commandedVoltage() const override {
        return inner_->commandedVoltage();
    }
    void setBrakeMode(hal::BrakeMode mode) override { inner_->setBrakeMode(mode); }
    [[nodiscard]] hal::BrakeMode brakeMode() const override { return inner_->brakeMode(); }
    [[nodiscard]] double temperature() const override { return inner_->temperature(); }

    /// Frozen at the value it had when the window opened (the shaft stopped).
    [[nodiscard]] units::AngleDim position() const override {
        if (!jammed()) {
            frozen_ = false;
            return inner_->position();
        }
        if (!frozen_) {
            frozen_ = true;
            frozenPosition_ = inner_->position();
        }
        return frozenPosition_;
    }

    /// Creep (config) with the commanded sign while jammed.
    [[nodiscard]] units::AngularVelocity velocity() const override {
        if (!jammed()) {
            return inner_->velocity();
        }
        const double sign = inner_->commandedVoltage().value() < 0.0 ? -1.0 : 1.0;
        return units::AngularVelocity{sign * cfg_.creepSpeed.value()};
    }

    /// Stall current, scaled by the commanded fraction of full voltage.
    [[nodiscard]] units::Current current() const override {
        if (!jammed()) {
            return inner_->current();
        }
        const double frac =
            std::abs(inner_->commandedVoltage().value()) / hal::kMaxMotorVoltage.value();
        return units::Current{cfg_.stallCurrentAt12V.value() * frac};
    }

    [[nodiscard]] bool jammed() const {
        const double now = clock_->now().value();
        return now >= cfg_.start.value() && now < cfg_.end.value();
    }

private:
    hal::IMotor* inner_;
    const hal::IClock* clock_;
    JammedMotorConfig cfg_;
    mutable bool frozen_ = false;
    mutable units::AngleDim frozenPosition_{0.0};
};

struct LyingSpinMotorConfig {
    /// When the shaft truly stops (position freezes from here on).
    units::Time start{0.0};
    /// The healthy-looking velocity the encoder keeps REPORTING. REQUIRED
    /// scenario fact — no default (header note).
    units::AngularVelocity reportedVelocity;
    /// The nominal free-run current it keeps reporting. REQUIRED — no default.
    units::Current reportedCurrent;
};

/// The dishonest stall (header): truth on the position channel, lies on
/// velocity and current. A stall detector reading this device NEVER trips —
/// correctly, because a detector can only know what the sensors say — so the
/// test that drives an operation over it is the proof that the watchdog, and
/// nothing else, is the no-hang guarantee.
class LyingSpinMotor final : public hal::IMotor {
public:
    /// `inner` and `clock` must outlive the model.
    LyingSpinMotor(hal::IMotor& inner, const hal::IClock& clock,
                   const LyingSpinMotorConfig& config)
        : inner_{&inner}, clock_{&clock}, cfg_{config} {
        SHULIB_PRECONDITION(std::isfinite(cfg_.reportedVelocity.value()),
                            "LyingSpinMotor: reportedVelocity must be finite");
        SHULIB_PRECONDITION(std::isfinite(cfg_.reportedCurrent.value()) &&
                                cfg_.reportedCurrent.value() >= 0.0,
                            "LyingSpinMotor: reportedCurrent must be finite and >= 0");
    }

    void setVoltage(units::Voltage volts) override { inner_->setVoltage(volts); }
    [[nodiscard]] units::Voltage commandedVoltage() const override {
        return inner_->commandedVoltage();
    }
    void setBrakeMode(hal::BrakeMode mode) override { inner_->setBrakeMode(mode); }
    [[nodiscard]] hal::BrakeMode brakeMode() const override { return inner_->brakeMode(); }
    [[nodiscard]] double temperature() const override { return inner_->temperature(); }

    /// TRUTH: the shaft stopped — frozen from the stall instant. This is the
    /// channel a test grades against (the fake's independent truth).
    [[nodiscard]] units::AngleDim position() const override {
        if (!stalled()) {
            frozen_ = false;
            return inner_->position();
        }
        if (!frozen_) {
            frozen_ = true;
            frozenPosition_ = inner_->position();
        }
        return frozenPosition_;
    }

    /// LIE: healthy spin, no matter what the shaft is doing.
    [[nodiscard]] units::AngularVelocity velocity() const override {
        return stalled() ? cfg_.reportedVelocity : inner_->velocity();
    }

    /// LIE: nominal current, no matter the load.
    [[nodiscard]] units::Current current() const override {
        return stalled() ? cfg_.reportedCurrent : inner_->current();
    }

    [[nodiscard]] bool stalled() const { return clock_->now().value() >= cfg_.start.value(); }

private:
    hal::IMotor* inner_;
    const hal::IClock* clock_;
    LyingSpinMotorConfig cfg_;
    mutable bool frozen_ = false;
    mutable units::AngleDim frozenPosition_{0.0};
};

/// The unconfirmed-grab world: confirmation never arrives. Usable directly as
/// a Confirm predicate.
struct NeverConfirm {
    [[nodiscard]] bool operator()() const noexcept { return false; }
};

/// Confirmation arrives at an absolute instant — late (test the window's
/// edges), on time, or with `at = 0` INSTANTLY AND UNCONDITIONALLY, which is
/// the confirm that lies true (header note: the trust-boundary demonstration).
class ConfirmAfter {
public:
    /// `clock` must outlive the predicate.
    ConfirmAfter(const hal::IClock& clock, units::Time at) : clock_{&clock}, at_{at} {}

    [[nodiscard]] bool operator()() const { return clock_->now().value() >= at_.value(); }

private:
    const hal::IClock* clock_;
    units::Time at_;
};

}  // namespace shulib::sim
