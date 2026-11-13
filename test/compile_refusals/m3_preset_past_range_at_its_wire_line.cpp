// m3_preset_past_range_at_its_wire_line: MULTI-LINE: a preset past the range, declared above, refused at its wire(...) line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Arm arm2{
    .name = "arm2",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
    .range = {0, 160},
    .gravity = cosine(1.6_volt, 45),
    .home = restsAtStop(0),
    .presets = {{"stow", 0}, {"score", 195}},
};
inline constexpr MechanismTable mm = mechanisms(
    wire(arm2, motors(1)));
// expect: position_a_preset_is_outside_the_travel_limits
// expect-at: 17
