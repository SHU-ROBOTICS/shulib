// s63b_lift_no_shape_tuned: a lift with no .shape but a stated profile (it used to be ACCEPTED, reading 0 in at every angle).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Lift bad{.name = "bad", .motors = {Forward}, .cartridge = Red, .gearing = direct, .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = noGravity, .home = restsAtStop(0_in), .tuning = {.maxSpeed = 5.0, .maxAcceleration = 20.0}};
inline constexpr MechanismTable m = mechanisms(wire(bad, motors(1)));
// expect: position_a_lifts_shape_is_unset_give_it_spool_linkage_or_measured
