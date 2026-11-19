// s90_robot_two_flipper_kind_without_its_arm: robot two's flipper typed as a MOTOR ARM while its list declares a
// pneumatic "flipper" (the front toggle) — the build must refuse the disagreement by name (chunk ROB1-BT).
// run_refusal.py judges it.
#include "robots/mechanism_table.hpp"
namespace sb = shulib::bench;
static_assert(sb::flipperKindAgrees(sb::robot_two_facts::FlipperKind::MotorArm, sb::robot_two::kRobotTwoMechanisms));
// expect: the_flippers_typed_kind_does_not_match_its_robots_list
