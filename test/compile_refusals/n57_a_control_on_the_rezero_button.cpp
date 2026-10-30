// n57_a_control_on_the_rezero_button: a driver file whose loop re-zeroes field-centric on A — the
// claw's toggle button (every claw press would also re-zero the field frame). src/common/
// driver_file.hpp's DriverFile refuses it at build (T4's second check, kept — the MECH brief's N6).
// Written by MECH-A5; run_refusal.py judges it.
#include "refusal_common.hpp"
#include "common/driver_file.hpp"
#include "shulib/teleop/teleop_loop.hpp"
namespace n57 {
using namespace shulib::mechanism::words;
inline constexpr DriverControls kControls = controls(r2::kRobotTwoMechanisms,
    whileHeld(r2::clawPivot6, Button::R1, Button::R2, 12_volt), whileHeld(r2::clawPivot7, Button::L1, Button::L2, 12_volt),
    toggle(r2::claw, Button::A));
inline constexpr DriverProfile kFeel{.mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{.rezeroButton = Button::A, .releaseBrakeMode = Coast};
inline shulib::bench::DriverFile<kControls, kFeel, kLoop> driver{};
}  // namespace n57
// expect: a mechanism's button is the loop's re-zero button
