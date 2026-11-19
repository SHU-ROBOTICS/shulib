// rm01_a_feel_out_of_range: a driver file whose forward curve is 1.5 — outside [0, 1] a full push is no longer
// full speed and the curve dips near centre (input_shaping.hpp). src/common/driver_file.hpp's DriverFile refuses
// it at build (its third static_assert: chunk RMP1-B, the RMP1 brief's D10). run_refusal.py judges it.
#include "refusal_common.hpp"
#include "common/driver_file.hpp"
#include "shulib/teleop/teleop_loop.hpp"
namespace rm01 {
using namespace shulib::mechanism::words;
inline constexpr DriverControls kControls = controls(r2::kRobotTwoMechanisms,
    whileHeld(r2::clawPivot6, Button::R1, Button::R2, 12_volt), whileHeld(r2::clawPivot7, Button::L1, Button::L2, 12_volt),
    toggle(r2::claw, Button::A));
inline constexpr DriverProfile kFeel{.forward = {.expo = 1.5}, .mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{.rezeroButton = Button::Y, .releaseBrakeMode = Coast};
inline shulib::bench::DriverFile<kControls, kFeel, kLoop> driver{};
}  // namespace rm01
// expect: the feel is out of range
