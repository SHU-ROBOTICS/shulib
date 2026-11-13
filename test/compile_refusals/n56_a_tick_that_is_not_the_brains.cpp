// n56_a_tick_that_is_not_the_brains: a driver file that states a `.tickPeriod` of 20 ms — the loop
// runs at the BRAIN's tick, which src/common/driver_file.hpp's DriverFile supplies; a driver file
// states none, and one that states one anyway must state the brain's (T4's sixth check, kept — the
// MECH brief's N6/U6). The equal tick (10 ms) and none compile (test/robot_two_driver_test.cpp).
// Written by MECH-A5; run_refusal.py judges it.
#include "refusal_common.hpp"
#include "common/driver_file.hpp"
#include "shulib/teleop/teleop_loop.hpp"
namespace n56 {
using namespace shulib::mechanism::words;
inline constexpr DriverControls kControls = controls(r2::kRobotTwoMechanisms,
    whileHeld(r2::clawPivot6, Button::R1, Button::R2, 12_volt), whileHeld(r2::clawPivot7, Button::L1, Button::L2, 12_volt),
    toggle(r2::claw, Button::A));
inline constexpr DriverProfile kFeel{.mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{.tickPeriod = 20_ms, .rezeroButton = Button::Y, .releaseBrakeMode = Coast};
inline shulib::bench::DriverFile<kControls, kFeel, kLoop> driver{};
}  // namespace n56
// expect: leave .tickPeriod out of the driver file: the loop runs at the brain's tick
