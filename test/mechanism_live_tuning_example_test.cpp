// Chapter 18's "…or on the robot: live tuning" — its wiring listing, COMPILED (chunk TUNE1-A, 2026-11-10).
// api_doc_tool check-examples holds every ```cpp line of a public chapter to a line of an example test,
// verbatim: these are the chapter's three lines, run over robot two's list (the test fixture's copy of
// it, byte for byte the real one — test/mechanism_table_test.cpp pins the two equal). Its claw pivots
// are fake motors here; its pneumatic claw is ABSENT (nothing in it is live). Over the host test plant's
// fakes; nothing here has run on a robot.

#include "doctest.h"

#include <array>
#include <span>
#include <string>

#include "mechanism_fixtures.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/mechanism/live_tuning.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/motion_scheduler.hpp"

namespace {
using mech_fixture::robot_two::kRobotTwoMechanisms;
namespace mechanism = shulib::mechanism;
namespace fake = shulib::hal::fake;
struct NoWorld final : shulib::motion::ITickPacer {
    void pace() override {}
};
}  // namespace

TEST_CASE("guide-18h: the live-tuning listing -- a COPY of robot two's list, the service over it, attached; a press re-derives") {
    fake::FakeClock clock;
    fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    std::array<fake::FakeMotor, 2> motors{};
    std::array<shulib::hal::IMotor*, 2> imotors{&motors[0], &motors[1]};
    shulib::hal::MotorMechanism pivot6{std::span<shulib::hal::IMotor* const>{&imotors[0], 1}, shulib::hal::BrakeMode::Hold, "claw pivot 6"};
    shulib::hal::MotorMechanism pivot7{std::span<shulib::hal::IMotor* const>{&imotors[1], 1}, shulib::hal::BrakeMode::Hold, "claw pivot 7"};
    std::array<mechanism::Devices, 3> devices{};
    devices[0].motors = &pivot6;
    devices[0].currentLimits[0] = &motors[0];
    devices[1].motors = &pivot7;
    devices[1].currentLimits[0] = &motors[1];
    devices[2].absence = "not built in this test";
    NoWorld world;

    mechanism::LiveTuning tuning{kRobotTwoMechanisms};  // a COPY of the robot file's list
    mechanism::MechanismService service{world};
    service.bind(tuning.table(), devices, clock, faults, sink);  // the service runs the copy
    tuning.attach(service);  // checked: the service runs THIS copy

    CHECK(tuning.step(0, mechanism::Tunable::MaxVolts, -1).ok);
    CHECK(tuning.value(0, mechanism::Tunable::MaxVolts) == 11.95);
    CHECK(kRobotTwoMechanisms.specs[0].power.maxVolts == 12.0);  // the robot file's list is never written
    bool said = false;
    for (int i = 0; i < sink.size(); ++i) {
        said = said || sink.at(i).message == "'claw pivot 6': retuned live -- max volts 12 -> 11.95";
    }
    CHECK(said);
}
