#pragma once
//
// mechanism_position_rig.hpp — ONE position or speed mechanism over the host test plant, for the
// position and speed behaviours' tests (chunk MECH-A3, 2026-10-15). Say "host test plant", never
// "simulator": every plant constant here is INVENTED (HA-183).
//
// The rig is the brief's probe rig (P2, P3) made a test fixture: a spec — a robot-file declaration's
// `spec()`, or one built from it with ONE thing changed — its motors as hal::fake::FakeMotor, a
// sim::ArmPlant or sim::FlywheelPlant stepping them, the angle sensor and home switch the plant
// publishes into, and a MechanismService bound over exactly those devices. One control tick is ONE
// `svc.pace()`: the service ticks (the pacer seam), then the world pace steps the plant by `dt` and
// advances the clock — the requirements' order.
//
// THE CONTROLLER'S CONSTANTS AND THE PLANT'S ARE SET SEPARATELY, on purpose: `armPlant()` builds a
// plant from a cartridge, a gearing and a mismatch (its true kV a multiple of the cartridge's, its own
// friction and gravity), never from the spec the controller runs — a test that gave both the same
// numbers would prove only the algebra (the A2 truth-integrator lesson). By default the motor encoders
// read zero where the joint is at construction (the plant's `encoderZeroDeg` — a TEST choice, invented
// like the rest; the service never relies on it: a mechanism that rests at its stop is zeroed at
// whatever its encoders read at bind, and its log line says how far that is from their zero).
//
// Since MECH-S-A (2026-11-05) each rig motor is ALSO the mechanism's current limit (Devices::
// currentLimits — the FakeMotor is an hal::IMotorCurrentLimit), so every position and speed test reads
// its motors' limits at bind exactly as a robot does; the plant limits the joint's torque only when a
// limit below the 2.5 A default is set, so a test that states no cap runs as before. `Wiring::
// noCurrentLimits` boots a rig whose provider gave none (the "this provider cannot cap" boot).
//
// Members are in LIFETIME order: the devices and the plant are declared BEFORE the service, which
// must not outlive them (bind()'s rule; ~MechanismService ends a slot still holding its claim).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_digital_in.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_read_faults.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sim/mechanism_plant.hpp"
#include "shulib/units/quantity.hpp"

namespace pos_rig {

namespace mech = shulib::mechanism;
namespace sim = shulib::sim;
namespace fake = shulib::hal::fake;

inline constexpr double kPi = 3.14159265358979323846;

/// A cartridge's nominal free speed at 12 V, motor rpm (the same 100 / 200 / 600 the starting values
/// derive from — what a REAL motor of that cartridge cannot beat).
[[nodiscard]] inline double cartridgeRpm(shulib::hal::MotorCartridge c) {
    return mech::detail::cartridgeRpm(c);
}

/// THE PLANT for a joint: its true kV is the cartridge's free speed through `gear` times `kvx` (> 1 is
/// a plant SLOWER than its cartridge — losses; < 1 one faster, which a real motor cannot be), its own
/// static band `kS`, τ = 50 ms, its own gravity `kG` (cosine about `levelDeg`, or constant), hard stops
/// at `minStop` / `maxStop`, starting at `startDeg` with its encoders reading zero there.
[[nodiscard]] inline sim::ArmPlantConfig armPlant(double cartRpm, double gear, double kG, double kvx, double kS,
                                                  double startDeg = 0.0, double levelDeg = 45.0,
                                                  double minStop = -2.0, double maxStop = 170.0) {
    sim::ArmPlantConfig c;
    c.gearRatio = gear;
    const double freeJointRadS = cartRpm * 2.0 * kPi / 60.0 / gear;
    c.kS = kS;
    c.kV = 12.0 / freeJointRadS * kvx;
    c.kA = 0.05 * c.kV;
    c.kG = kG;
    c.levelDeg = levelDeg;
    c.minStopDeg = minStop;
    c.maxStopDeg = maxStop;
    c.startDeg = startDeg;
    c.encoderZeroDeg = startDeg;
    return c;
}

/// The world below the service: each pace steps the plant by `dt`, then advances the clock.
struct PlantWorld final : shulib::motion::ITickPacer {
    fake::FakeClock* clock = nullptr;
    double dt = 0.01;
    long paces = 0;
    sim::ArmPlant* arm = nullptr;
    sim::FlywheelPlant* fly = nullptr;
    void pace() override {
        ++paces;
        if (arm != nullptr) {
            arm->step(dt);
        }
        if (fly != nullptr) {
            fly->step(dt);
        }
        clock->advance(shulib::units::Time{dt});
    }
};

/// What a rig wires besides the motors.
struct Wiring {
    /// The plant publishes a limit switch at this joint angle (the spec homes by switch).
    std::optional<sim::SwitchMount> homeSwitch{};
    /// The plant publishes an angle sensor mounted so (the spec declares one).
    std::optional<sim::RotationMount> angle{};
    /// The home switch's read-fault counter is wired (the PROS input's, faked).
    bool homeSwitchFaults = false;
    /// A switch NOT published by the plant: the test drives `homeSwitch` itself (a dead or stuck one).
    bool switchByHand = false;
    /// A hand-driven switch's level at boot (before bind).
    bool switchLevelAtBoot = false;
    /// The home switch's input is FAILING from boot: every read of it counts a screened read.
    bool switchFailingFromBoot = false;
    /// The control tick, seconds.
    double dt = 0.01;
    /// The provider gave NO current limit for the motors (MECH-S-A): Devices::currentLimits all nullptr.
    bool noCurrentLimits = false;
};

/// ONE position mechanism over an ArmPlant (header).
struct ArmRig {
    fake::FakeClock clock;
    fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    std::array<fake::FakeMotor, 4> motors{};
    std::array<shulib::hal::IMotor*, 4> imotors{};
    std::array<fake::FakeMotor*, 4> fakes{};
    fake::FakeRotation rotation;
    fake::FakeDigitalIn homeSwitch;
    fake::FakeReadFaults homeFaults;
    mech::MechanismSpec spec;
    std::optional<shulib::hal::MotorMechanism> mech;
    std::optional<sim::ArmPlant> plant;
    PlantWorld world;
    mech::MechanismService svc{world};

    ArmRig(const mech::MechanismSpec& s, const sim::ArmPlantConfig& pc, const Wiring& w = {}) : spec{s} {
        for (std::size_t i = 0; i < 4; ++i) {
            imotors[i] = &motors[i];
            fakes[i] = &motors[i];
        }
        const std::size_t n = spec.motorCount;
        mech.emplace(std::span<shulib::hal::IMotor* const>{imotors.data(), n},
                     shulib::config::toBrakeMode(spec.safeMode), "rig");
        plant.emplace(std::span<fake::FakeMotor* const>{fakes.data(), n}, pc);
        mech::Devices d{.motors = &*mech};
        for (std::size_t i = 0; i < n && !w.noCurrentLimits; ++i) {
            d.currentLimits[i] = &motors[i];  // MECH-S-A: each motor's own limit (header)
        }
        if (w.angle) {
            plant->attachRotation(&rotation, *w.angle);
            d.angleSensor = &rotation;
        }
        if (w.homeSwitch || w.switchByHand) {
            if (w.homeSwitch) {
                plant->attachSwitch(&homeSwitch, *w.homeSwitch);
            }
            d.homeSwitch = &homeSwitch;
            if (w.switchByHand) {
                homeSwitch.setState(w.switchLevelAtBoot);
            }
            if (w.homeSwitchFaults) {
                d.homeSwitchFaults = &homeFaults;
                homeFaults.failing(w.switchFailingFromBoot);
            }
        }
        world.clock = &clock;
        world.dt = w.dt;
        world.arm = &*plant;
        svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&d, 1}, clock,
                 faults, sink);
    }
    [[nodiscard]] mech::PositionSlot& slot() { return svc.position(0); }
    [[nodiscard]] double trueDeg() const { return plant->trueDeg(); }
    [[nodiscard]] double volts() const { return motors[0].commandedVoltage().value(); }
    [[nodiscard]] double amps() const { return motors[0].current().value(); }
    void tick(int n = 1) {
        for (int i = 0; i < n; ++i) {
            svc.pace();
        }
    }
    /// Pace until the step is over (at most `cap` ticks); returns how many ran.
    int runStep(int cap = 2000) {
        int n = 0;
        while (slot().running() && n < cap) {
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
    /// How many MECH lines contain `text`.
    [[nodiscard]] int mechCount(const std::string& text) const {
        int n = 0;
        for (const std::string& l : mechLines()) {
            n += l.find(text) != std::string::npos ? 1 : 0;
        }
        return n;
    }
    [[nodiscard]] int stallFaults() const { return faults.raiseCount(shulib::diag::FaultCode::MechanismStalled); }
};

/// ONE speed mechanism over a FlywheelPlant (header).
struct FlyRig {
    fake::FakeClock clock;
    fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    std::array<fake::FakeMotor, 4> motors{};
    std::array<shulib::hal::IMotor*, 4> imotors{};
    std::array<fake::FakeMotor*, 4> fakes{};
    mech::MechanismSpec spec;
    std::optional<shulib::hal::MotorMechanism> mech;
    std::optional<sim::FlywheelPlant> plant;
    PlantWorld world;
    mech::MechanismService svc{world};

    FlyRig(const mech::MechanismSpec& s, const sim::FlywheelPlantConfig& pc, double dt = 0.01) : spec{s} {
        for (std::size_t i = 0; i < 4; ++i) {
            imotors[i] = &motors[i];
            fakes[i] = &motors[i];
        }
        const std::size_t n = spec.motorCount;
        mech.emplace(std::span<shulib::hal::IMotor* const>{imotors.data(), n},
                     shulib::config::toBrakeMode(spec.safeMode), "rig");
        plant.emplace(std::span<fake::FakeMotor* const>{fakes.data(), n}, pc);
        mech::Devices d{.motors = &*mech};
        for (std::size_t i = 0; i < n; ++i) {
            d.currentLimits[i] = &motors[i];  // MECH-S-A: each motor's own limit (header)
        }
        world.clock = &clock;
        world.dt = dt;
        world.fly = &*plant;
        svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&d, 1}, clock,
                 faults, sink);
    }
    [[nodiscard]] mech::SpeedSlot& slot() { return svc.speed(0); }
    [[nodiscard]] double volts() const { return motors[0].commandedVoltage().value(); }
    void tick(int n = 1) {
        for (int i = 0; i < n; ++i) {
            svc.pace();
        }
    }
    [[nodiscard]] int stallFaults() const { return faults.raiseCount(shulib::diag::FaultCode::MechanismStalled); }
};

/// THE FLYWHEEL PLANT for a wheel geared `gear` (motor turns per wheel turn) on a cartridge of
/// `cartRpm`: true kV the free speed's times `kvx` (>= 1: no faster than its cartridge), friction `kS`,
/// spin-up τ `tau` seconds (kA = kV·τ).
[[nodiscard]] inline sim::FlywheelPlantConfig flywheelPlant(double cartRpm, double gear, double kvx, double kS,
                                                            double tau) {
    sim::FlywheelPlantConfig c;
    c.gearRatio = gear;
    const double freeWheelRadS = cartRpm / gear * 2.0 * kPi / 60.0;
    c.kS = kS;
    c.kV = 12.0 / freeWheelRadS * kvx;
    c.kA = c.kV * tau;
    return c;
}

}  // namespace pos_rig
