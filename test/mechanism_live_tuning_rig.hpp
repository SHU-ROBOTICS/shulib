#pragma once
//
// mechanism_live_tuning_rig.hpp — a robot's mechanism table run by a MechanismService bound over a
// LiveTuning's COPY, over the host test plant, for live tuning's tests (chunk TUNE1-A, 2026-11-10). Say
// "host test plant", never "simulator": every plant constant here is INVENTED (HA-183).
//
// mechanism_position_rig.hpp's ArmRig / FlyRig bind a service over ONE bare spec; live tuning needs the
// service bound over ITS table (LiveTuning::attach() checks that every slot's spec is the table's), so
// this rig builds each entry's devices the same way — fake motors, each also its own current limit
// (MECH-S-A), an ArmPlant under a position mechanism, a FlywheelPlant under a speed one, bare fake
// motors under a power one — and binds the service over `tuning.table()`. One control tick is ONE
// `svc.pace()` (the service ticks, then the plant steps by `dt` and the clock advances). At most one
// position and one speed mechanism per table (one plant each); a pneumatic entry is ABSENT here.
//
// THE FIXTURES are declared exactly as a robot file writes them (namespace tune_fixture). Two rigs built
// from the same table and plant are TWINS: the tests change one and compare the other, tick for tick.
//
// Members are in LIFETIME order: the devices, the plant and the LiveTuning (whose table the service's
// slots point into) are declared BEFORE the service, which is destroyed first.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechanism_position_rig.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_digital_in.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/live_tuning.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/sim/mechanism_plant.hpp"

namespace tune_fixture {
using namespace shulib::mechanism::words;

// A ONE-MOTOR SPOOL LIFT that rests on its stop: robot one's cascade stands in (its numbers are not known
// yet — the team's answer of 2026-11-05). Its presets stay low, so a range narrowed to 12 in is the
// hold's question, not the build's.
inline constexpr Lift cascade{
    .name = "cascade",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
    .presets = {{"floor", 0_in}, {"low", 6_in}},
};
inline constexpr MechanismTable kCascade = mechanisms(wire(cascade, motors(1)));

// The same lift with a HIGH preset (presets are never live — TA6) and a stated kD (a number the robot file
// STATES — the copy block must keep it).
inline constexpr Lift tallLift{
    .name = "lift",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
    .presets = {{"floor", 0_in}, {"low", 8_in}, {"high", 22_in}},
    .tuning = {.kD = 0.01},
};
inline constexpr MechanismTable kTallLift = mechanisms(wire(tallLift, motors(1)));

// A STALL-HOMED lift whose home is MID-TRAVEL (its push may point either way — the build's rule — so the
// live "never turned around" rule is the only one that sees a turned push), with a current cap.
inline constexpr Lift midHome{
    .name = "mid home lift",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 36),
    .shape = spool(1.25_in),
    .safe = Brake,
    .stall = {.amps = 1.0, .belowMotorRpm = 8, .lasting = 0.3_s},
    .currentCap = {.amps = 1.5},
    .range = {0_in, 20_in},
    .gravity = constantGravity(0.8_volt),
    .home = homeByStall(6_in, -2_volt, {.amps = 0.6, .belowMotorRpm = 4, .lasting = 0.2_s}),
    .presets = {{"stack", 10_in}},
};
inline constexpr MechanismTable kMidHome = mechanisms(wire(midHome, motors(2)));

// An ARM with cosine gravity, stall-homed down onto its stop (every position member a copy line can name).
inline constexpr Arm swing{
    .name = "swing",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = gears(12, 84),
    .safe = Brake,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0, 120},
    .gravity = cosine(1.0_volt, 30),
    .home = homeByStall(0, -3_volt, {.amps = 0.45, .belowMotorRpm = 2, .lasting = 0.2_s}),
    .presets = {{"down", 0}, {"up", 110}},
};
inline constexpr MechanismTable kSwing = mechanisms(wire(swing, motors(3)));

// A FLYWHEEL (robot x's) and a POWER claw pivot WITH a stall signature (robot two's have none).
inline constexpr Flywheel flywheel{
    .name = "flywheel",
    .motors = {Forward},
    .cartridge = Blue,
    .gearing = gears(72, 12),
    .safe = Coast,
    .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
    .atSpeed = within(60, 0.1_s),
    .presets = {{"shoot", 3000}},
};
inline constexpr Power pivot{
    .name = "pivot",
    .motors = {Forward},
    .cartridge = Red,
    .safe = Hold,
    .stall = {.amps = 1.5, .belowMotorRpm = 5, .lasting = 0.3_s},
};
inline constexpr Power roller{.name = "roller", .motors = {Forward}, .cartridge = Blue, .safe = Coast};
inline constexpr MechanismTable kFlyAndPivot = mechanisms(wire(flywheel, motors(4)), wire(pivot, motors(5)),
                                                          wire(roller, motors(6)));

// LONG TYPED NUMBERS on a stall-homed lift (each one's shortest form 17-19 characters), for the copy
// block's longest member line — the homing one. Its name is the longest a homed lift may have: its route
// id `<stem>_home` must fit 23 characters (route_ids.hpp).
inline constexpr Lift longNumbers{
    .name = "eighteen char lift",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 1.0000000000000002, .belowMotorRpm = 5.000000000000001, .lasting = 0.30000000000000004_s},
    .currentCap = {.amps = 2.4000000000000004},
    .range = {0.30000000000000004_in, 23.999999999999996_in},
    .gravity = constantGravity(-1.0000000000000002_volt),
    .home = homeByStall(0.30000000000000004_in, -2.0000000000000004_volt,
                        {.amps = 0.30000000000000004, .belowMotorRpm = 3.0000000000000004, .lasting = 0.20000000000000004_s}),
};
inline constexpr MechanismTable kLongNumbers = mechanisms(wire(longNumbers, motors(7)));
// The LONGEST name a mechanism may have (23 characters): a never-homed lift with no presets publishes no
// route id, so its name may be that long — for the copy block's begin and end lines.
inline constexpr Lift longName{
    .name = "a twenty-three char lft",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = neverHomed,
};
inline constexpr MechanismTable kLongName = mechanisms(wire(longName, motors(8)));
// A SWITCH-HOMED lift with NO GRAVITY (a counterweighted carriage — TUNE1-A's review: no other fixture homes
// by switch or has noGravity). Its home switch is at 12 in, MID-travel (pressed at or above it; the build
// lets that push point either way), so its LOWER limit can be raised past where it holds — the cascade's
// cannot (its home and its "floor" are at 0 in). Run it over liftPlant(0.0) with switchAt(12.0).
inline constexpr Lift switchLift{
    .name = "switch lift",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Coast,  // let go, it coasts: a load can carry it past a limit (the review's "already outside")
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = noGravity,
    .home = homeBySwitch(12_in, 6_volt),
};
inline constexpr MechanismTable kSwitchLift = mechanisms(wire(switchLift, motors(9), homeSwitch('B')));
}  // namespace tune_fixture

namespace tune_rig {

namespace mech = shulib::mechanism;
namespace sim = shulib::sim;
namespace fake = shulib::hal::fake;

/// A sink that keeps every line (a FakeTelemetrySink) and calls `hook` with each — how a test reaches
/// into the middle of a tick (a step's verdict is logged INSIDE the tick that reaches it).
struct HookSink final : shulib::hal::ITelemetrySink {
    fake::FakeTelemetrySink lines;
    std::function<void(std::string_view subsystem, std::string_view message)> hook;
    void log(shulib::hal::LogLevel level, std::string_view subsystem, std::string_view message) override {
        lines.log(level, subsystem, message);
        if (hook) {
            hook(subsystem, message);
        }
    }
};

/// The lift plant every cascade test runs over: a Red direct 1.5 in spool, gravity 1.6 V against the
/// spec's 1.0 V (the hold's integral must carry the difference), its stops at -0.5 in and 24.5 in.
[[nodiscard]] inline sim::ArmPlantConfig liftPlant(double kGPlant = 1.6) {
    const double degPerIn = 360.0 / (3.14159265358979323846 * 1.5);
    sim::ArmPlantConfig c = pos_rig::armPlant(100, 1, kGPlant, 1.0, 0.3, 0.0, 0.0, -0.5 * degPerIn, 24.5 * degPerIn);
    c.constantGravity = true;
    return c;
}

/// The switch-homed lift's home switch: pressed at or above `inches` on the 1.5 in spool (liftPlant's).
[[nodiscard]] inline sim::SwitchMount switchAt(double inches) {
    return sim::SwitchMount{.atDeg = inches * 360.0 / (3.14159265358979323846 * 1.5), .atTop = true};
}

/// A table run by a service bound over a LiveTuning's copy (header).
struct TuneRig {
    fake::FakeClock clock;
    HookSink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    fake::FakeDigitalIn homeSwitch;  ///< the position mechanism's, when it homes by switch (the plant presses it)
    std::array<std::array<fake::FakeMotor, 4>, mech::kMaxMechanisms> motors{};
    std::array<std::array<shulib::hal::IMotor*, 4>, mech::kMaxMechanisms> imotors{};
    std::array<std::array<fake::FakeMotor*, 4>, mech::kMaxMechanisms> fakes{};
    std::array<std::optional<shulib::hal::MotorMechanism>, mech::kMaxMechanisms> mechs{};
    std::array<mech::Devices, mech::kMaxMechanisms> devices{};
    std::optional<sim::ArmPlant> arm;
    std::optional<sim::FlywheelPlant> fly;
    pos_rig::PlantWorld world;
    mech::LiveTuning tuning;
    mech::MechanismService svc{world};

    TuneRig(const mech::MechanismTable& table, const sim::ArmPlantConfig& armCfg = liftPlant(),
            const sim::FlywheelPlantConfig& flyCfg = pos_rig::flywheelPlant(600, 1.0 / 6.0, 1.0, 0.4, 0.2),
            double dt = 0.01, std::optional<sim::SwitchMount> sw = std::nullopt)
        : tuning{table} {
        const std::size_t n = tuning.count();
        for (std::size_t k = 0; k < n; ++k) {
            const mech::MechanismSpec& s = tuning.table().specs[k];
            if (s.kind == mech::Kind::Pneumatic) {
                devices[k] = mech::Devices{.absence = "not built in this rig"};
                continue;
            }
            for (std::size_t m = 0; m < 4; ++m) {
                imotors[k][m] = &motors[k][m];
                fakes[k][m] = &motors[k][m];
            }
            mechs[k].emplace(std::span<shulib::hal::IMotor* const>{imotors[k].data(), s.motorCount},
                             shulib::config::toBrakeMode(s.safeMode), "rig");
            devices[k].motors = &*mechs[k];
            for (std::size_t m = 0; m < s.motorCount; ++m) {
                devices[k].currentLimits[m] = &motors[k][m];
            }
            if (s.kind == mech::Kind::Position) {
                arm.emplace(std::span<fake::FakeMotor* const>{fakes[k].data(), s.motorCount}, armCfg);
                if (s.homeSwitch.kind == mech::SensorKind::Switch) {
                    devices[k].homeSwitch = &homeSwitch;
                    if (sw) {
                        arm->attachSwitch(&homeSwitch, *sw);
                    }
                }
            } else if (s.kind == mech::Kind::Speed) {
                fly.emplace(std::span<fake::FakeMotor* const>{fakes[k].data(), s.motorCount}, flyCfg);
            }
        }
        world.clock = &clock;
        world.dt = dt;
        world.arm = arm ? &*arm : nullptr;
        world.fly = fly ? &*fly : nullptr;
        svc.bind(tuning.table(), std::span<const mech::Devices>{devices.data(), n}, clock, faults, sink);
        tuning.attach(svc);
    }
    void tick(int n = 1) {
        for (int i = 0; i < n; ++i) {
            svc.pace();
        }
    }
    /// Pace until mechanism `k`'s step is over (at most `cap` ticks); returns how many ran.
    int runStep(std::size_t k = 0, int cap = 3000) {
        int n = 0;
        while (svc.slot(k).running() && n < cap) {
            svc.pace();
            ++n;
        }
        return n;
    }
    [[nodiscard]] mech::PositionSlot& pos(std::size_t k = 0) { return svc.position(k); }
    /// Mechanism `k`'s first motor's commanded volts.
    [[nodiscard]] double volts(std::size_t k = 0) const { return motors[k][0].commandedVoltage().value(); }
    /// Mechanism `k`'s first motor's current limit, A.
    [[nodiscard]] double limitAmps(std::size_t k = 0) const { return motors[k][0].currentLimit().value(); }
    /// Every line logged under `tag`, in order.
    [[nodiscard]] std::vector<std::string> linesTagged(std::string_view tag) const {
        std::vector<std::string> out;
        for (int i = 0; i < sink.lines.size(); ++i) {
            if (sink.lines.at(i).subsystem == tag) {
                out.push_back(sink.lines.at(i).message);
            }
        }
        return out;
    }
    /// How many MECH lines contain `text`.
    [[nodiscard]] int mechCount(std::string_view text) const {
        int c = 0;
        for (const std::string& l : linesTagged("MECH")) {
            c += l.find(text) != std::string::npos ? 1 : 0;
        }
        return c;
    }
    [[nodiscard]] int stallFaults() const { return faults.raiseCount(shulib::diag::FaultCode::MechanismStalled); }
};

/// A tuned spec and the robot file's, compared FIELD BY FIELD, bit for bit (each double by its bytes):
/// the name of the first field that differs, or "" when none does. Every member a declaration writes.
[[nodiscard]] inline std::string firstDifference(const mech::MechanismSpec& a, const mech::MechanismSpec& b) {
    const auto same = [](double x, double y) { return std::memcmp(&x, &y, sizeof x) == 0; };
    const auto stall = [&](const mech::StallSpec& x, const mech::StallSpec& y, const std::string& what) -> std::string {
        if (!same(x.amps, y.amps)) {
            return what + ".amps";
        }
        if (!same(x.speedRadS, y.speedRadS)) {
            return what + ".speedRadS";
        }
        if (!same(x.time, y.time)) {
            return what + ".time";
        }
        return "";
    };
    if (std::string{a.name} != std::string{b.name}) {
        return "name";
    }
    if (a.kind != b.kind || a.motorCount != b.motorCount || a.cartridge != b.cartridge || a.safeMode != b.safeMode) {
        return "kind/motors/cartridge/safe";
    }
    if (!same(a.currentCapAmps, b.currentCapAmps)) {
        return "currentCapAmps";
    }
    if (!same(a.power.maxVolts, b.power.maxVolts)) {
        return "power.maxVolts";
    }
    if (std::string d = stall(a.power.stall, b.power.stall, "power.stall"); !d.empty()) {
        return d;
    }
    const mech::PositionSpec& p = a.position;
    const mech::PositionSpec& q = b.position;
    const std::pair<const char*, std::pair<double, double>> pos[] = {
        {"gearRatio", {p.gearRatio, q.gearRatio}}, {"spoolDiameterIn", {p.spoolDiameterIn, q.spoolDiameterIn}},
        {"minPosition", {p.minPosition, q.minPosition}}, {"maxPosition", {p.maxPosition, q.maxPosition}},
        {"maxVelocity", {p.maxVelocity, q.maxVelocity}}, {"maxAcceleration", {p.maxAcceleration, q.maxAcceleration}},
        {"tolerance", {p.tolerance, q.tolerance}}, {"settleTime", {p.settleTime, q.settleTime}},
        {"kP", {p.kP, q.kP}}, {"kI", {p.kI, q.kI}}, {"kD", {p.kD, q.kD}}, {"kS", {p.kS, q.kS}},
        {"kV", {p.kV, q.kV}}, {"kA", {p.kA, q.kA}}, {"kG", {p.kG, q.kG}}, {"horizontalAt", {p.horizontalAt, q.horizontalAt}},
        {"homePosition", {p.homePosition, q.homePosition}}, {"homeVolts", {p.homeVolts, q.homeVolts}},
        {"unhomedVolts", {p.unhomedVolts, q.unhomedVolts}}};
    for (const auto& [name, v] : pos) {
        if (!same(v.first, v.second)) {
            return std::string{"position."} + name;
        }
    }
    if (p.units != q.units || p.geometry != q.geometry || p.gravity != q.gravity || p.home != q.home ||
        p.presetCount != q.presetCount) {
        return "position.units/geometry/gravity/home/presetCount";
    }
    for (std::size_t i = 0; i < mech::kMaxPresets; ++i) {
        if (std::string{p.presets[i].name} != std::string{q.presets[i].name} ||
            !same(p.presets[i].value, q.presets[i].value)) {
            return "position.presets";
        }
    }
    if (std::string d = stall(p.stall, q.stall, "position.stall"); !d.empty()) {
        return d;
    }
    if (std::string d = stall(p.homeStall, q.homeStall, "position.homeStall"); !d.empty()) {
        return d;
    }
    const mech::SpeedSpec& s = a.speed;
    const mech::SpeedSpec& t = b.speed;
    const std::pair<const char*, std::pair<double, double>> spd[] = {
        {"gearRatio", {s.gearRatio, t.gearRatio}}, {"maxRpm", {s.maxRpm, t.maxRpm}}, {"kS", {s.kS, t.kS}},
        {"kV", {s.kV, t.kV}}, {"kA", {s.kA, t.kA}}, {"kP", {s.kP, t.kP}}, {"kI", {s.kI, t.kI}},
        {"atSpeedBandRpm", {s.atSpeedBandRpm, t.atSpeedBandRpm}}, {"atSpeedTime", {s.atSpeedTime, t.atSpeedTime}}};
    for (const auto& [name, v] : spd) {
        if (!same(v.first, v.second)) {
            return std::string{"speed."} + name;
        }
    }
    if (std::string d = stall(s.stall, t.stall, "speed.stall"); !d.empty()) {
        return d;
    }
    return "";
}

}  // namespace tune_rig
