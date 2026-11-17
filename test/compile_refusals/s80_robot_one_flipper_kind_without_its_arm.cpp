// s80_robot_one_flipper_kind_without_its_arm: the flipper's kind typed as a MOTOR ARM while robot one's list
// declares no Arm named "flipper" (the ROB1 brief's D6). run_refusal.py judges it.
#include "robots/mechanism_table.hpp"
namespace sb = shulib::bench;
static_assert(sb::flipperKindAgrees(sb::robot_one_facts::FlipperKind::MotorArm, sb::robot_one::kRobotOneMechanisms));
// expect: the_flippers_typed_kind_does_not_match_its_robots_list
