// robot::MechanismParts — every ROLE of every mechanism, built over a device provider (chunk MECH-A1,
// the MECH brief's §1.5 and §3.1 "MechanismParts"; P4).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * P4's SIX CENSUS CASES through the REAL on-robot provider (hal::pros::ProsDeviceProvider over a
//     faked registry and the host PROS shim): every role's state and its boot line VERBATIM — the
//     clean robot; two SAME-kind cables swapped (invisible to the boot: the lines are identical, and
//     the test says so); a rotation and a distance sensor swapped (the wrist ABSENT, the intake runs
//     WITHOUT its piece sensor); a motor where a piece sensor is bound; an unplugged pivot; an
//     actuator port holding an optical sensor (its own angle sensor NOT ASKED — and a counting
//     registry proves its port was never queried).
//   * THE ABSENT RULE (mutation M4: a refused ANGLE sensor treated as degrade-only): an ABSENT
//     mechanism is not built, not in built(), has every device pointer null and its reason in
//     absence(); a degrade-only refusal (a piece sensor) leaves the mechanism built.
//   * The devices handed to the service ARE the ones the provider made, the read-fault counts
//     included (pieceFaults, homeSwitchFaults), and the guard's span lists the SAME mechanisms.
//   * The per-line pneumatic form is built when the config states per-line levels — each line made
//     at ITS start level.
//   * The Robot forwards: its accessors and mechanismParts() agree object for object.

#include "doctest.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "pros/shim_control.hpp"

#include "shulib/config/robot_config.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/mechanism_parts.hpp"
#include "shulib/robot/robot.hpp"

namespace cfgns = shulib::config;
namespace mech = shulib::mechanism;
using shulib::hal::DeviceKind;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::hal::fake::FakeDeviceRegistry;
using shulib::robot::MechanismParts;
using mech::RoleState;

namespace {

/// A registry that forwards to a FakeDeviceRegistry and COUNTS the queries per port — so "not asked"
/// is a fact about the provider, not a word on a line.
class CountingRegistry final : public shulib::hal::IDeviceRegistry {
public:
    explicit CountingRegistry(const FakeDeviceRegistry& inner) : inner_{&inner} {}
    [[nodiscard]] DeviceKind pluggedKind(int port) const noexcept override {
        if (port >= 0 && port < 22) {
            ++asked_[static_cast<std::size_t>(port)];
        }
        return inner_->pluggedKind(port);
    }
    [[nodiscard]] int asked(int port) const { return asked_[static_cast<std::size_t>(port)]; }

private:
    const FakeDeviceRegistry* inner_;
    mutable std::array<int, 22> asked_{};
};

/// P4's clean census: the drive on 11..20, robot z's motors on 1..7, the IMU on 21, the radio on 8, a
/// distance sensor on 9, a rotation sensor on 10.
FakeDeviceRegistry cleanCensus() {
    FakeDeviceRegistry r;
    for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 1, 2, 3, 4, 5, 6, 7}) {
        r.setKind(p, DeviceKind::Motor);
    }
    r.setKind(21, DeviceKind::Imu);
    r.setKind(8, DeviceKind::Radio);
    r.setKind(9, DeviceKind::Distance);
    r.setKind(10, DeviceKind::Rotation);
    return r;
}

/// A list projected into the configs the parts borrow (they must outlive the parts).
template <std::size_t N = cfgns::kMaxMechanisms>
struct Configs {
    std::array<cfgns::MechanismConfig, N> c{};
    std::size_t n = 0;
    explicit Configs(const mech::MechanismTable& t) {
        for (std::size_t i = 0; i < t.count; ++i) {
            c[i] = mech::toMechanismConfig(t.specs[i], t.wiring[i]);
        }
        n = t.count;
    }
    [[nodiscard]] std::span<const cfgns::MechanismConfig> span() const { return {c.data(), n}; }
};

/// Every role's boot line, in order — what a program prints (MECH-A5).
std::vector<std::string> bootLines(const MechanismParts& parts) {
    std::vector<std::string> out;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        for (const mech::RoleFact& r : parts.roles(i)) {
            char line[240];
            mech::roleLine(line, sizeof line, parts.config(i).name, r.role, r.port, static_cast<mech::Line>(r.line),
                           r.state, r.refusal, r.degradeOnly);
            out.emplace_back(line);
        }
    }
    return out;
}

const std::vector<std::string> kCleanLines = {
    "arm: motor 1 -> port 1 ok",
    "lift: motor 1 -> port 2 ok",
    "lift: motor 2 -> port 3 ok",
    "lift: home switch -> 3-wire B not checkable (the brain cannot see a 3-wire device)",
    "flywheel: motor 1 -> port 4 ok",
    "intake: motor 1 -> port 5 ok",
    "intake: piece sensor -> port 9 ok",
    "hood: line 1 -> 3-wire C not checkable (the brain cannot see a 3-wire device)",
    "hood: line 2 -> 3-wire D not checkable (the brain cannot see a 3-wire device)",
    "wrist: motor 1 -> port 6 ok",
    "wrist: angle sensor -> port 10 ok",
    "claw pivot 6: motor 1 -> port 7 ok",
};

/// Boot robot z through the REAL on-robot provider over `census`.
struct ProsBoot {
    CountingRegistry reg;
    shulib::hal::pros::ProsDeviceProvider prov;
    Configs<> configs{mech_fixture::robot_z::kRobotZMechanisms};
    MechanismParts parts;
    explicit ProsBoot(const FakeDeviceRegistry& census) : reg{census}, prov{reg} {
        pros::shim::resetAll();
        parts.build(configs.span(), prov);
    }
};

struct NullPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};

struct FakeWorld {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    NullPacer pacer;
    FakeDeviceProvider devices{clock, battery, sink, pacer};
};

}  // namespace

TEST_CASE("MECH-A1 parts P4.1: the clean robot -- one line per role, every one ok or not checkable") {
    const FakeDeviceRegistry census = cleanCensus();
    ProsBoot b{census};
    CHECK(bootLines(b.parts) == kCleanLines);
    CHECK(b.parts.size() == 7U);
    CHECK(b.parts.built().size() == 7U);
    for (std::size_t i = 0; i < 7; ++i) {
        CAPTURE(i);
        CHECK(std::string{b.parts.absence(i)}.empty());
        CHECK(b.parts.builtAt(i) != nullptr);
        CHECK(b.parts.built()[i] == b.parts.builtAt(i));
    }
    // The devices handed to the service ARE the provider's.
    const auto d = b.parts.devices();
    CHECK(d[0].motors == b.parts.motorAt(0));
    CHECK(d[4].pneumatic == b.parts.pneumaticAt(4));
    CHECK(d[4].motors == nullptr);
    CHECK(d[3].pieceDistance == b.prov.distance(9));
    CHECK(d[3].pieceFaults == b.prov.readFaults(9));
    CHECK(d[3].pieceFaults != nullptr);
    CHECK(d[5].angleSensor == b.prov.rotation(10));
    CHECK(d[1].homeSwitch == b.prov.digitalIn(2));
    CHECK(d[1].homeSwitchFaults == b.prov.lineReadFaults(2));
    CHECK(d[1].homeSwitchFaults != nullptr);
    CHECK(d[0].pieceFaults == nullptr);       // no piece sensor, no count
    CHECK(d[0].homeSwitchFaults == nullptr);
    // The motors were made with the mechanism's OWN signed ports: the mirrored lift motor is -3.
    CHECK(b.parts.config(1).motorPorts[1] == -3);
    CHECK(b.parts.roles(1)[1].port == 3);  // the boot line names the PHYSICAL port
}

TEST_CASE("MECH-A1 parts P4.2: two SAME-KIND cables swapped are INVISIBLE to the boot (the bench page's job)") {
    // The arm's motor and the flywheel's motor trade ports: both are motors, so every line is the
    // clean robot's — this test pins that the boot claims no more than it can see.
    const FakeDeviceRegistry census = cleanCensus();
    ProsBoot b{census};
    CHECK(bootLines(b.parts) == kCleanLines);
}

TEST_CASE("MECH-A1 parts P4.3: a rotation and a distance sensor swapped -- the wrist ABSENT, the intake runs WITHOUT") {
    FakeDeviceRegistry census = cleanCensus();
    census.setKind(9, DeviceKind::Rotation);
    census.setKind(10, DeviceKind::Distance);
    ProsBoot b{census};
    std::vector<std::string> expected = kCleanLines;
    expected[6] = "intake: piece sensor -> port 9 MISSING, it runs WITHOUT it: port 9: a rotation sensor (expected a distance sensor)";
    expected[10] = "wrist: angle sensor -> port 10 ABSENT: port 10: a distance sensor (expected a rotation sensor)";
    CHECK(bootLines(b.parts) == expected);
    // The wrist: ABSENT — not built, not in the guard's span, every device null, its reason kept.
    CHECK(b.parts.builtAt(5) == nullptr);
    CHECK(b.parts.motorAt(5) == nullptr);
    CHECK(std::string{b.parts.absence(5)} == "port 10: a distance sensor (expected a rotation sensor)");
    CHECK(std::string{b.parts.devices()[5].absence} == "port 10: a distance sensor (expected a rotation sensor)");
    CHECK(b.parts.devices()[5].angleSensor == nullptr);
    CHECK(b.parts.built().size() == 6U);
    for (shulib::hal::IMechanism* m : b.parts.built()) {
        CHECK(std::string{m->name()} != "wrist");
    }
    // The intake: BUILT, running WITHOUT its piece sensor (and so with no count).
    CHECK(b.parts.builtAt(3) != nullptr);
    CHECK(std::string{b.parts.absence(3)}.empty());
    CHECK(b.parts.devices()[3].pieceDistance == nullptr);
    CHECK(b.parts.devices()[3].pieceFaults == nullptr);
    CHECK(b.parts.roles(3)[1].state == RoleState::Refused);
    CHECK(b.parts.roles(3)[1].degradeOnly);
}

TEST_CASE("MECH-A1 parts P4.4: a MOTOR where the intake's piece sensor is bound -- it runs WITHOUT one") {
    FakeDeviceRegistry census = cleanCensus();
    census.setKind(9, DeviceKind::Motor);
    ProsBoot b{census};
    std::vector<std::string> expected = kCleanLines;
    expected[6] = "intake: piece sensor -> port 9 MISSING, it runs WITHOUT it: port 9: a motor (expected a distance sensor)";
    CHECK(bootLines(b.parts) == expected);
    CHECK(b.parts.built().size() == 7U);
}

TEST_CASE("MECH-A1 parts P4.5: the claw pivot unplugged -- ABSENT, the provider's sentence kept") {
    FakeDeviceRegistry census = cleanCensus();
    census.setKind(7, DeviceKind::None);
    ProsBoot b{census};
    std::vector<std::string> expected = kCleanLines;
    expected[11] = "claw pivot 6: motor 1 -> port 7 ABSENT: port 7: nothing plugged in (expected a motor)";
    CHECK(bootLines(b.parts) == expected);
    CHECK(b.parts.builtAt(6) == nullptr);
    CHECK(std::string{b.parts.absence(6)} == "port 7: nothing plugged in (expected a motor)");
}

TEST_CASE("MECH-A1 parts P4.6: the wrist's MOTOR port holds an optical sensor -- ABSENT, its angle sensor NEVER asked (M4's net)") {
    FakeDeviceRegistry census = cleanCensus();
    census.setKind(6, DeviceKind::Optical);
    ProsBoot b{census};
    std::vector<std::string> expected = kCleanLines;
    expected[9] = "wrist: motor 1 -> port 6 ABSENT: port 6: an optical sensor (expected a motor)";
    expected[10] = "wrist: angle sensor -> port 10 not asked (the mechanism is already ABSENT)";
    CHECK(bootLines(b.parts) == expected);
    CHECK(b.reg.asked(10) == 0);  // "not asked" is a fact: the registry never heard of port 10
    CHECK(b.reg.asked(6) == 1);
    CHECK(b.parts.builtAt(5) == nullptr);
    CHECK(b.parts.roles(5)[1].state == RoleState::NotAsked);
}

TEST_CASE("MECH-A1 parts: a two-motor lift whose FIRST motor is refused -- its second motor and its switch are NEVER asked") {
    // MECH-A1's review: every refused-motor case had one motor, so "a later motor role is not asked" was
    // never exercised. Robot z's lift is on ports 2 and 3 (its home switch on B); port 3 is no one else's.
    FakeDeviceRegistry census = cleanCensus();
    census.setKind(2, DeviceKind::None);
    ProsBoot b{census};
    std::vector<std::string> expected = kCleanLines;
    expected[1] = "lift: motor 1 -> port 2 ABSENT: port 2: nothing plugged in (expected a motor)";
    expected[2] = "lift: motor 2 -> port 3 not asked (the mechanism is already ABSENT)";
    expected[3] = "lift: home switch -> 3-wire B not asked (the mechanism is already ABSENT)";
    CHECK(bootLines(b.parts) == expected);
    CHECK(b.reg.asked(2) == 1);
    CHECK(b.reg.asked(3) == 0);  // "not asked" is a fact: the registry never heard of port 3
    REQUIRE(b.parts.roles(1).size() == 3U);
    CHECK(b.parts.roles(1)[1].state == RoleState::NotAsked);
    CHECK(b.parts.roles(1)[2].state == RoleState::NotAsked);
    CHECK(b.parts.builtAt(1) == nullptr);
    CHECK(b.parts.devices()[1].homeSwitch == nullptr);
    CHECK(std::string{b.parts.absence(1)} == "port 2: nothing plugged in (expected a motor)");
}

TEST_CASE("MECH-A1 parts: a refused ANGLE sensor makes the mechanism ABSENT even though its motors were made (M4)") {
    FakeDeviceRegistry census = cleanCensus();
    census.setKind(10, DeviceKind::None);
    ProsBoot b{census};
    CHECK(b.parts.builtAt(5) == nullptr);
    CHECK(b.parts.devices()[5].motors == nullptr);
    CHECK(b.parts.roles(5)[0].state == RoleState::Present);  // its motor answered...
    CHECK(b.parts.roles(5)[1].state == RoleState::Refused);  // ... its angle sensor did not
    CHECK_FALSE(b.parts.roles(5)[1].degradeOnly);
    CHECK(std::string{b.parts.absence(5)} == "port 10: nothing plugged in (expected a rotation sensor)");
}

TEST_CASE("MECH-A1 parts: every role of robot y over the HOST provider -- each device and each count wired through") {
    FakeWorld w;
    const Configs<> configs{mech_fixture::robot_y::kRobotYMechanisms};
    MechanismParts parts;
    parts.build(configs.span(), w.devices);
    REQUIRE(parts.size() == 7U);
    CHECK(parts.built().size() == 7U);
    const auto d = parts.devices();
    CHECK(d[0].angleSensor == w.devices.rotation(10));       // the wrist's reversed rotation sensor
    CHECK(d[4].pieceOptical == &w.devices.fakeOptical(9));    // the roller
    CHECK(d[4].pieceFaults == &w.devices.fakeReadFaults(9));
    CHECK(d[4].pieceDistance == nullptr);
    CHECK(d[5].pieceSwitch == &w.devices.fakeDigitalIn(5));   // the catapult's switch on E
    CHECK(d[5].pieceFaults == &w.devices.fakeLineReadFaults(5));
    CHECK(d[6].pneumatic != nullptr);                         // the grip ...
    CHECK(d[6].pieceDistance == &w.devices.fakeDistance(8));  // ... and its distance sensor
    CHECK(d[6].pieceFaults == &w.devices.fakeReadFaults(8));
    // The counts are LIVE: a sensor that stops answering moves the count the service will read.
    REQUIRE(d[4].pieceFaults != nullptr);
    const int before = d[4].pieceFaults->faultedReads();
    w.devices.fakeReadFaults(9).failing(true);
    CHECK(d[4].pieceFaults->faultedReads() > before);
    // The grip's lines were made at its START level ("loose" = Low), its safe level "shut" = High.
    CHECK(w.devices.digitalOutMade(1));
    CHECK_FALSE(w.devices.digitalOutInitialState(1));
    CHECK(d[6].pneumatic->safeCommand());
    CHECK_FALSE(d[6].pneumatic->perLine());
    const std::vector<std::string> lines = bootLines(parts);
    CHECK(lines[lines.size() - 2] == "grip: line 1 -> 3-wire A not checkable (the brain cannot see a 3-wire device)");
    CHECK(lines.back() == "grip: piece sensor -> port 8 ok");
}

TEST_CASE("MECH-A1 parts: a refused HOME switch makes it ABSENT; a refused PIECE switch leaves it running WITHOUT") {
    SUBCASE("the lift's home switch (D12: unreachable on a brain, the fake's refuseLine reaches it)") {
        FakeWorld w;
        w.devices.refuseLine(2);
        const Configs<> configs{mech_fixture::robot_x::kRobotXMechanisms};
        MechanismParts parts;
        parts.build(configs.span(), w.devices);
        CHECK(parts.builtAt(1) == nullptr);
        CHECK(std::string{parts.absence(1)} == "3-wire port B (line 2) has no digital in");
        const std::vector<std::string> lines = bootLines(parts);
        CHECK(lines[3] == "lift: home switch -> 3-wire B ABSENT: 3-wire port B (line 2) has no digital in");
        CHECK(parts.built().size() == 4U);
    }
    SUBCASE("the catapult's piece switch") {
        FakeWorld w;
        w.devices.refuseLine(5);
        const Configs<> configs{mech_fixture::robot_y::kRobotYMechanisms};
        MechanismParts parts;
        parts.build(configs.span(), w.devices);
        CHECK(parts.builtAt(5) != nullptr);
        CHECK(parts.devices()[5].pieceSwitch == nullptr);
        CHECK(parts.devices()[5].pieceFaults == nullptr);
        const std::vector<std::string> lines = bootLines(parts);
        bool found = false;
        for (const std::string& l : lines) {
            found = found || l == "catapult: piece sensor -> 3-wire E MISSING, it runs WITHOUT it: 3-wire port E (line 5) has no digital in";
        }
        CHECK(found);
    }
}

TEST_CASE("MECH-A1 parts: per-line levels build the PER-LINE pneumatic -- each line made at ITS start level") {
    using namespace shulib::mechanism::words;
    static constexpr Pneumatic hood{.name = "hood",
                             .positions = {{"down", {Low, Low}}, {"middle", {High, Low}}, {"up", {High, High}}},
                             .start = "middle", .safe = "down", .moveTime = 0.3_s};
    static constexpr mech::MechanismTable t = mech::mechanisms(wire(hood, lines('C', 'D')));
    FakeWorld w;
    const Configs<> configs{t};
    MechanismParts parts;
    parts.build(configs.span(), w.devices);
    shulib::hal::PneumaticMechanism* p = parts.pneumaticAt(0);
    REQUIRE(p != nullptr);
    CHECK(p->perLine());
    CHECK(w.devices.digitalOutInitialState(3));         // line C made HIGH ("middle")
    CHECK_FALSE(w.devices.digitalOutInitialState(4));   // line D made LOW
    p->applySafeState();                                // "down": Low / Low
    CHECK_FALSE(w.devices.fakeDigitalOut(3).commanded());
    CHECK_FALSE(w.devices.fakeDigitalOut(4).commanded());
    CHECK_FALSE(p->safeCommand());                      // line 0's safe level
    // A safe position that DIFFERS across its lines ("middle" = High / Low) — the case the per-line form
    // exists for (MECH-A1's review: "down" is uniform, so a builder giving every line line 1's level,
    // or a per-line form whose safeCommand() forgot line 0's, would have passed).
    static constexpr Pneumatic safeMiddle{.name = "hood",
                                   .positions = {{"down", {Low, Low}}, {"middle", {High, Low}}, {"up", {High, High}}},
                                   .start = "up", .safe = "middle", .moveTime = 0.3_s};
    static constexpr mech::MechanismTable tm = mech::mechanisms(wire(safeMiddle, lines('C', 'D')));
    FakeWorld wm;
    const Configs<> cm{tm};
    MechanismParts pm;
    pm.build(cm.span(), wm.devices);
    shulib::hal::PneumaticMechanism* hm = pm.pneumaticAt(0);
    REQUIRE(hm != nullptr);
    REQUIRE(hm->perLine());
    CHECK(wm.devices.digitalOutInitialState(3));        // made at "up": High ...
    CHECK(wm.devices.digitalOutInitialState(4));        // ... High
    CHECK(hm->safeLevel(0));
    CHECK_FALSE(hm->safeLevel(1));
    CHECK(hm->safeCommand());                           // line 0's safe level: High
    hm->applySafeState();                               // "middle": line C High, line D LOW
    CHECK(wm.devices.fakeDigitalOut(3).commanded());
    CHECK_FALSE(wm.devices.fakeDigitalOut(4).commanded());
    // Robot x's hood starts and is safe at a UNIFORM position: the single-level form.
    FakeWorld w2;
    const Configs<> x{mech_fixture::robot_x::kRobotXMechanisms};
    MechanismParts px;
    px.build(x.span(), w2.devices);
    REQUIRE(px.pneumaticAt(4) != nullptr);
    CHECK_FALSE(px.pneumaticAt(4)->perLine());
}

TEST_CASE("MECH-A1 parts: built once; bounds; nothing-to-build; out-of-range indices") {
    FakeWorld w;
    std::array<cfgns::MechanismConfig, 1> empty{};
    std::snprintf(empty[0].name, sizeof empty[0].name, "%s", "ghost");
    MechanismParts parts;
    parts.build(empty, w.devices);
    CHECK(parts.builtAt(0) == nullptr);
    CHECK(std::string{parts.absence(0)} == "no motors and no 3-wire lines -- nothing to build");
    CHECK(parts.roles(0).empty());
    CHECK(std::string{parts.absence(1)} == "no mechanism is declared at this index");
    CHECK(parts.builtAt(9) == nullptr);
    CHECK(parts.roles(9).empty());
    CHECK(parts.indexOf("ghost") == 0U);
    CHECK(parts.indexOf("nobody") == 1U);
    CHECK(parts.indexOf(nullptr) == 1U);
    CHECK_THROWS_AS(parts.build(empty, w.devices), shulib::PreconditionError);
    CHECK_THROWS_AS((void)parts.config(1), shulib::PreconditionError);
    std::array<cfgns::MechanismConfig, 9> tooMany{};
    MechanismParts more;
    CHECK_THROWS_AS(more.build(tooMany, w.devices), shulib::PreconditionError);
}

TEST_CASE("MECH-A1 parts: the Robot FORWARDS -- its accessors and mechanismParts() agree object for object") {
    FakeWorld w;
    cfgns::RobotConfig c{};
    std::snprintf(c.identity.name, sizeof c.identity.name, "%s", "x");
    c.drivetrain.kind = cfgns::DrivetrainKind::Tank;
    shulib::hal::DrivetrainDescription& d = c.drivetrain.tank.description;
    d.kind = shulib::hal::DrivetrainKind::Tank;
    d.cartridge = shulib::hal::MotorCartridge::Blue;
    d.imuPort = 21;
    d.left.signedPorts[0] = 11;
    d.left.signedPorts[1] = -12;
    d.left.count = 2;
    d.right.signedPorts[0] = -13;
    d.right.signedPorts[1] = 14;
    d.right.count = 2;
    d.left.inchesPerRadian = shulib::units::Length{1.5};
    d.right.inchesPerRadian = shulib::units::Length{1.5};
    d.trackWidth = shulib::units::Length{12.0};
    d.source = shulib::hal::DescriptionSource::TypedTable;
    c.odometry.kind = cfgns::OdometryKind::DriveEncoders;
    const mech::MechanismTable& t = mech_fixture::robot_y::kRobotYMechanisms;
    for (std::size_t i = 0; i < t.count; ++i) {
        c.mechanisms[i] = mech::toMechanismConfig(t.specs[i], t.wiring[i]);
    }
    c.mechanismCount = t.count;
    REQUIRE(c.valid());
    shulib::robot::Robot robot{c, w.devices};
    const MechanismParts& parts = robot.mechanismParts();
    REQUIRE(robot.mechanismCount() == 7U);
    REQUIRE(parts.size() == 7U);
    CHECK(robot.mechanisms().data() == parts.built().data());
    CHECK(robot.mechanisms().size() == 7U);
    for (std::size_t i = 0; i < 7; ++i) {
        CAPTURE(i);
        CHECK(robot.builtMechanismAt(i) == parts.builtAt(i));
        CHECK(robot.mechanismAt(i) == parts.motorAt(i));
        CHECK(robot.pneumaticMechanismAt(i) == parts.pneumaticAt(i));
        CHECK(robot.mechanismAbsence(i) == parts.absence(i));
        CHECK(&parts.config(i) == &robot.config().mechanisms[i]);  // BORROWED from the Robot's copy
        CHECK(parts.builtAt(i)->name() == robot.config().mechanisms[i].name);
    }
    CHECK(robot.mechanism("wrist") == parts.motorAt(0));
    CHECK(robot.pneumaticMechanism("grip") == parts.pneumaticAt(6));
    CHECK(parts.devices()[0].angleSensor == w.devices.rotation(10));
    CHECK(std::string{robot.mechanismAbsence(7)} == "no mechanism is declared at this index");
}

// ═══ MECH-S-A: each built motor's own CURRENT LIMIT in its Devices (the MECH-S brief's §1.1) ════════

TEST_CASE("MECH-S-A parts: every BUILT motor's current limit is in its Devices, in the motors' order -- none past them, none for a pneumatic") {
    const FakeDeviceRegistry census = cleanCensus();
    ProsBoot b{census};
    const auto d = b.parts.devices();
    // robot z: arm 1, lift 2+3, flywheel 4, intake 5, hood (lines), wrist 6, claw pivot 6 on 7
    const struct {
        std::size_t mech;
        std::uint8_t ports[2];
        std::size_t n;
    } motorKinds[] = {{0, {1, 0}, 1}, {1, {2, 3}, 2}, {2, {4, 0}, 1}, {3, {5, 0}, 1}, {5, {6, 0}, 1}, {6, {7, 0}, 1}};
    for (const auto& k : motorKinds) {
        CAPTURE(k.mech);
        for (std::size_t m = 0; m < 4; ++m) {
            CAPTURE(m);
            if (m < k.n) {
                REQUIRE(d[k.mech].currentLimits[m] != nullptr);
                // the provider's own: the ProsMotor made on that port (BUG CAUGHT: motor 1's limit twice)
                CHECK(d[k.mech].currentLimits[m] == b.prov.currentLimit(k.ports[m]));
            } else {
                CHECK(d[k.mech].currentLimits[m] == nullptr);
            }
        }
    }
    for (std::size_t m = 0; m < 4; ++m) {
        CHECK(d[4].currentLimits[m] == nullptr);  // the hood: lines, no motor
    }
    // the parts WROTE no limit (the service decides — D12 (b)): every motor still at the default
    for (const int p : {1, 2, 3, 4, 5, 6, 7}) {
        CAPTURE(p);
        CHECK(pros::shim::motorState(p).setCurrentLimitCalls == 0);
    }
}

TEST_CASE("MECH-S-A parts: an ABSENT mechanism carries no current limit -- even for a motor that WAS made before the refusal") {
    // the wrist's motor on 6 is made, then its angle sensor on 10 is refused: ABSENT — its Devices are
    // all null, the made motor's limit included (nothing may write a cap to a mechanism that is not run)
    FakeDeviceRegistry census = cleanCensus();
    census.setKind(10, DeviceKind::None);
    ProsBoot b{census};
    REQUIRE(b.parts.builtAt(5) == nullptr);
    REQUIRE(b.prov.currentLimit(6) != nullptr);  // the provider did make it ...
    for (std::size_t m = 0; m < 4; ++m) {
        CHECK(b.parts.devices()[5].currentLimits[m] == nullptr);  // ... the parts hand none on
    }
}
