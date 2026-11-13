#pragma once
//
// mechanism_service_rig.hpp — the host rig for the mechanism service's tests (chunk MECH-A2,
// 2026-10-15).
//
// A robot's list is booted through the REAL robot::MechanismParts over the FAKE device provider —
// exactly what a program does with its own provider — and the service is bound over THOSE devices
// (the same objects the RunGuard's span lists, the brief's landmine 20). Below the service sits a
// `World` pacer: each pace advances the FakeClock by `dt` and steps whatever host test plant a test
// registered, so "one control tick" in a test is one svc.pace() (the pacer seam) or one
// svc.tickFromDriverLoop() followed by a world pace.
//
// A piece sensor served by the fake provider starts at FakeDistance's default — 0 in at confidence
// 0, which the evidence rule reads as UNKNOWN (a held near value). A test's "no piece" is the PROS
// adapter's honest no-object reading, 393.66 in (`kNoObjectIn`, the brief's landmine 21).

#include <array>
#include <cstddef>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "shulib/config/robot_config.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/mechanism_parts.hpp"
#include "shulib/units/quantity.hpp"

namespace mech_rig {

/// PROS's 9999 mm "no object", as the distance adapter converts it: the honest far reading.
inline constexpr double kNoObjectIn = 9999.0 / 25.4;

/// The world below the service: each pace advances the clock by `dt` after stepping the plants.
struct World final : shulib::motion::ITickPacer {
    shulib::hal::fake::FakeClock* clock = nullptr;
    double dt = 0.01;
    long paces = 0;
    std::vector<std::function<void(double)>> steps;
    void pace() override {
        ++paces;
        for (auto& f : steps) {
            f(dt);
        }
        clock->advance(shulib::units::Time{dt});
    }
};

/// A pacer that does nothing (the fake provider's, unused by the service's tests).
struct NullPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};

/// A robot's list booted over the fake provider and bound into a service (header). The members are
/// in LIFETIME order: the devices (the provider, the parts) are declared BEFORE the service, so they
/// outlive it — the service's destructor ends a slot still holding its claim, and touches the
/// mechanism to do it (bind()'s rule: the devices outlive the service).
struct Rig {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    NullPacer providerPacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, providerPacer};
    std::array<shulib::config::MechanismConfig, shulib::mechanism::kMaxMechanisms> configs{};
    shulib::robot::MechanismParts parts;
    World world;
    shulib::mechanism::MechanismService svc{world};
    const shulib::mechanism::MechanismTable* table;

    /// `prepare(devices)` runs before the build: refuse a port, set a reading.
    template <class Prepare>
    Rig(const shulib::mechanism::MechanismTable& t, Prepare prepare) : table{&t} {
        world.clock = &clock;
        for (std::size_t i = 0; i < t.count; ++i) {
            configs[i] = shulib::mechanism::toMechanismConfig(t.specs[i], t.wiring[i]);
        }
        prepare(devices);
        parts.build(std::span<const shulib::config::MechanismConfig>{configs.data(), t.count}, devices);
        svc.bind(t, parts.devices(), clock, faults, sink);
    }
    explicit Rig(const shulib::mechanism::MechanismTable& t) : Rig{t, [](shulib::hal::fake::FakeDeviceProvider&) {}} {}

    /// `n` control ticks through the pacer seam.
    void pace(int n = 1) {
        for (int i = 0; i < n; ++i) {
            svc.pace();
        }
    }
    /// Pace until `svc.slot(i)`'s step is over, at most `cap` paces; returns how many ran.
    int paceUntilDone(std::size_t i, int cap = 2000) {
        int n = 0;
        while (svc.slot(i).running() && n < cap) {
            svc.pace();
            ++n;
        }
        return n;
    }
    /// Every MECH-tagged message, in order.
    [[nodiscard]] std::vector<std::string> mechLines() const {
        std::vector<std::string> out;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == "MECH") {
                out.push_back(sink.at(i).message);
            }
        }
        return out;
    }
    /// The last MECH-tagged message ("" when none).
    [[nodiscard]] std::string lastMech() const {
        const std::vector<std::string> v = mechLines();
        return v.empty() ? std::string{} : v.back();
    }
    /// The level of the last MECH-tagged message.
    [[nodiscard]] shulib::hal::LogLevel lastMechLevel() const {
        for (int i = sink.size() - 1; i >= 0; --i) {
            if (sink.at(i).subsystem == "MECH") {
                return sink.at(i).level;
            }
        }
        return shulib::hal::LogLevel::Trace;
    }
};

}  // namespace mech_rig
