// s89_robot_two_undeclared_non_drive_motor: the brain's Devices screen lists a motor on port 11 that is not a
// drive motor, and robot two's list does not declare it — shulib Calibrate would take port 11 for a drive
// candidate and POWER it (the ROB1 brief's R9, T9, GM20b; chunk ROB1-BT). The check src/robots/mechanism_table.hpp
// applies to robot two's real facts, applied here to that list. run_refusal.py judges it.
#include "robots/mechanism_table.hpp"
namespace sb = shulib::bench;
static_assert(sb::nonDriveMotorsDeclared(
    sb::robot_two_facts::PortList{{sb::robot_two_facts::kCascadePorts.value().first,
                                   sb::robot_two_facts::kCascadePorts.value().second, 11},
                                  3},
    sb::robot_two::kRobotTwoMechanisms));
// expect: a_motor_the_devices_screen_lists_is_not_a_drive_motor_and_is_not_declared_in_its_robots_list
