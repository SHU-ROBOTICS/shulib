#pragma once
//
// mechanism_test_rig.hpp — shared pieces for the F1 mechanism suites.
//
// The trap these pieces exist to avoid (the one that has bitten five chunks):
// the fake and the operation must NOT share a notion of "done". Nothing here
// knows what an operation considers complete; tests hand-write expected
// timelines from the contract (tick N commanded, tick N+k confirmed, verdict
// at tick N+k) and assert against those literals. RecordingMotor exists so a
// test can assert what was commanded AND IN WHAT ORDER at the bottom of the
// stack — never against a mechanism's or an operation's own record of itself.

#include <string>
#include <vector>

#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/units/quantity.hpp"

namespace mech_rig {

/// An IMotor that records every command as an ordered event list, so ordering
/// claims ("brake mode BEFORE zero volts") and never-commanded claims ("a
/// true-on-entry confirm spins nothing") are assertable rather than argued.
/// Readings are injectable like FakeMotor's.
class RecordingMotor final : public shulib::hal::IMotor {
public:
    struct Event {
        enum class Kind { Voltage, Brake } kind;
        double volts = 0.0;                                       // Kind::Voltage
        shulib::hal::BrakeMode mode = shulib::hal::BrakeMode::Coast;  // Kind::Brake
    };

    void setVoltage(shulib::units::Voltage v) override {
        commanded_ = v;
        events_.push_back({Event::Kind::Voltage, v.value(), shulib::hal::BrakeMode::Coast});
    }
    [[nodiscard]] shulib::units::Voltage commandedVoltage() const override { return commanded_; }
    void setBrakeMode(shulib::hal::BrakeMode m) override {
        mode_ = m;
        events_.push_back({Event::Kind::Brake, 0.0, m});
    }
    [[nodiscard]] shulib::hal::BrakeMode brakeMode() const override { return mode_; }
    [[nodiscard]] shulib::units::AngleDim position() const override { return position_; }
    [[nodiscard]] shulib::units::AngularVelocity velocity() const override { return velocity_; }
    [[nodiscard]] shulib::units::Current current() const override { return current_; }
    [[nodiscard]] double temperature() const override { return 0.0; }

    void setPosition(shulib::units::AngleDim p) { position_ = p; }
    void setVelocity(shulib::units::AngularVelocity v) { velocity_ = v; }
    void setCurrent(shulib::units::Current c) { current_ = c; }

    [[nodiscard]] const std::vector<Event>& events() const noexcept { return events_; }
    [[nodiscard]] int eventCount() const noexcept { return static_cast<int>(events_.size()); }
    /// True if any recorded voltage command was non-zero.
    [[nodiscard]] bool everEnergized() const noexcept {
        for (const Event& e : events_) {
            if (e.kind == Event::Kind::Voltage && e.volts != 0.0) {
                return true;
            }
        }
        return false;
    }

private:
    shulib::units::Voltage commanded_{0.0};
    shulib::hal::BrakeMode mode_ = shulib::hal::BrakeMode::Coast;
    shulib::units::AngleDim position_{0.0};
    shulib::units::AngularVelocity velocity_{0.0};
    shulib::units::Current current_{0.0};
    std::vector<Event> events_;
};

/// Clock + sink + latch + validated MechanismDeps — what every operation test
/// wires first. The clock starts at 0 and the tests advance it by hand.
struct OpRig {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch latch{sink, clock};
    shulib::manipulation::MechanismDeps deps{
        .clock = &clock, .faults = &latch, .telemetry = &sink};
};

/// Drive an operation with a fixed cadence: tick i happens at t = i*dt (the
/// clock advances AFTER each Running tick), so "the verdict lands at tick N"
/// is a hand-checkable literal. Returns the exit verdict and its tick index;
/// maxTicks exhaustion returns Running — itself a failure the caller asserts
/// against (the bounded-loop shape that lets a defeated watchdog read as RED
/// instead of a hung suite).
struct DriveResult {
    shulib::manipulation::MechanismOutcome outcome =
        shulib::manipulation::MechanismOutcome::Running;
    int exitTick = -1;
};
inline DriveResult drive(shulib::manipulation::IMechanismOp& op,
                         shulib::hal::fake::FakeClock& clock, double dt, int maxTicks) {
    for (int i = 0; i < maxTicks; ++i) {
        const auto o = op.tick();
        if (o != shulib::manipulation::MechanismOutcome::Running) {
            return {o, i};
        }
        clock.advance(shulib::units::Time{dt});
    }
    return {shulib::manipulation::MechanismOutcome::Running, maxTicks};
}

/// Warn-level "MECH" lines whose message contains `needle`.
inline int countMechWarns(const shulib::hal::fake::FakeTelemetrySink& sink,
                          const char* needle) {
    int n = 0;
    for (int i = 0; i < sink.size(); ++i) {
        const auto& e = sink.at(i);
        if (e.level == shulib::hal::LogLevel::Warn && e.subsystem == "MECH" &&
            e.message.find(needle) != std::string::npos) {
            ++n;
        }
    }
    return n;
}

}  // namespace mech_rig
