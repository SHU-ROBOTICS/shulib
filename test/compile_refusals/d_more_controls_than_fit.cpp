// d_more_controls_than_fit: 25 controls (a driver file holds 24).
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(r2::kRobotTwoMechanisms, autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw), autonOnly(r2::claw));
// expect: more_controls_than_fit
// expect-at: 5
