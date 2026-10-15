// Guide chapter 10's MECHANISMS section, compiled and run (cases guide-10f to guide-10h; chunk
// MECH-A1). Every ```cpp line the chapter shows is a line of this file — tools/api_doc_tool.py's
// check-examples fails the build if one drifts — and each case below checks what the chapter says
// that listing does.

#include "doctest.h"

#include <array>
#include <span>
#include <string>
#include <vector>

#include "shulib/config/robot_config.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/route_ids.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/mechanism_parts.hpp"

namespace guide_robot {
using namespace shulib::mechanism::words;

// ═══ WHAT EACH MECHANISM IS ═════════════════════════════════════════════════════════════
inline constexpr Arm arm{
    .name = "arm",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),         // a 12-tooth gear on the motor drives a 60 on the arm
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
    .range = {0, 160},                // degrees: down on its stop .. just past vertical
    .gravity = cosine(1.6_volt, 45),  // 1.6 V holds it out level; it is level at 45 degrees
    .home = restsAtStop(0),           // it rests on its stop at 0 when the program starts
    .presets = {{"stow", 0}, {"score", 95}, {"high", 140}},
};
inline constexpr Power intake{
    .name = "intake",
    .motors = {Forward},
    .cartridge = Blue,
    .pieceSensor = distanceSensor(2_in),  // it "sees a piece" closer than 2 in
    .safe = Coast,
};
inline constexpr Pneumatic claw{
    .name = "claw",
    .positions = {{"closed", {Low}, grabs}, {"open", {High}}},
    .start = "closed",
    .safe = "closed",
    .moveTime = 0.5_s,
};

// ═══ WHERE EACH ONE IS PLUGGED IN (re-cabling changes only these lines) ═══════════════════
inline constexpr MechanismTable kGuideMechanisms = mechanisms(
    wire(arm, motors(1)),
    wire(intake, motors(5), pieceSensor(9)),
    wire(claw, lines('A')));
}  // namespace guide_robot

namespace {
struct NullPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};
}  // namespace

TEST_CASE("guide-10f: a robot's mechanisms, declared once and wired separately") {
    namespace mech = shulib::mechanism;
    const mech::MechanismTable& t = guide_robot::kGuideMechanisms;
    REQUIRE(t.count == 3);
    // The declaration is WHAT it is; the wiring is WHERE it is plugged in.
    CHECK(t.specs[0].position.gearRatio == 5.0);           // gears(12, 60): five motor turns per arm turn
    CHECK(t.specs[0].position.maxPosition == 160.0);
    CHECK(t.wiring[0].motors[0] == 1);
    CHECK(t.wiring[1].pieceSensorPort == 9);
    CHECK(t.wiring[2].lines[0] == mech::Line::A);
    // The declaration is also the HANDLE: a preset by name, checked at build.
    constexpr mech::PositionGoal score = guide_robot::arm["score"];
    CHECK(score.value == 95.0);
    // The device-level config the robot is built from (the one projection).
    const shulib::config::MechanismConfig c = mech::toMechanismConfig(t.specs[1], t.wiring[1]);
    CHECK(c.motorPorts[0] == 5);
    CHECK(c.distancePort == 9);
}

TEST_CASE("guide-10g: the route ids nobody types") {
    namespace mech = shulib::mechanism;
    const mech::RouteIds ids = mech::routeIdsOf(guide_robot::kGuideMechanisms.specSpan());
    std::vector<std::string> got;
    for (std::size_t i = 0; i < ids.count; ++i) {
        got.emplace_back(ids.ids[i].view());
    }
    CHECK(got == std::vector<std::string>{"arm_stow", "arm_score", "arm_high", "arm_home", "intake_run",
                                          "intake_until_piece", "intake_until_gone", "intake_stop",
                                          "claw_closed", "claw_open"});
}

TEST_CASE("guide-10h: one boot line per role, built over the provider") {
    namespace mech = shulib::mechanism;
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    NullPacer pacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, pacer};
    devices.refusePort(9);  // the intake's distance sensor is unplugged today
    const mech::MechanismTable& t = guide_robot::kGuideMechanisms;
    std::array<shulib::config::MechanismConfig, 3> configs{};
    for (std::size_t i = 0; i < 3; ++i) {
        configs[i] = mech::toMechanismConfig(t.specs[i], t.wiring[i]);
    }
    shulib::robot::MechanismParts parts;
    parts.build(configs, devices);
    std::vector<std::string> lines;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        for (const mech::RoleFact& r : parts.roles(i)) {
            char line[200];
            mech::roleLine(line, sizeof line, parts.config(i).name, r.role, r.port, static_cast<mech::Line>(r.line),
                           r.state, r.refusal, r.degradeOnly);
            lines.emplace_back(line);
        }
    }
    CHECK(lines == std::vector<std::string>{
                       "arm: motor 1 -> port 1 ok",
                       "intake: motor 1 -> port 5 ok",
                       "intake: piece sensor -> port 9 MISSING, it runs WITHOUT it: port 9 has no distance sensor",
                       "claw: line 1 -> 3-wire A not checkable (the brain cannot see a 3-wire device)"});
    CHECK(parts.built().size() == 3U);  // the intake still runs — without its sensor
}
