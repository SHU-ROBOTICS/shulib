// n48b_stall_homed_arm_held_no_way: a STALL-homed arm held with no way to home it in driver control (U5's other homing method).
// Written by MECH-A4's review round; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr Arm sa{.name = "sa", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .safe = Brake, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 120}, .gravity = noGravity, .home = homeByStall(0, -3_volt, {.amps = 0.45, .belowMotorRpm = 2, .lasting = 0.2_s})}; inline constexpr MechanismTable m = mechanisms(wire(sa, motors(1))); inline constexpr DriverControls c = controls(m, whileHeld(sa, Button::R1, Button::R2, 6_volt));
// expect: a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control
// expect-at: 5
