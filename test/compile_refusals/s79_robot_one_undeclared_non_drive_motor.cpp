// s79_robot_one_undeclared_non_drive_motor: the brain's Devices screen lists a motor on port 11 that is not a
// drive motor, and robot one's list does not declare it (ROB1-C2: its declared motors are the cascade's 10 and
// 1 and the side toggles' 9 and 2 — ROB1-A's TU used 10, the cascade's real port since) — shulib Calibrate
// would take port 11 for a drive candidate and POWER it (the ROB1 brief's R9, probe P4, T9, GM20b). The check
// src/robots/mechanism_table.hpp applies to the real ledger, applied here to that list. run_refusal.py judges it.
#include "robots/mechanism_table.hpp"
namespace sb = shulib::bench;
static_assert(sb::robotOneNonDriveMotorsDeclared(
    sb::robot_one_facts::PortList{{sb::robot_one_facts::kCascadePorts.value().first,
                                   sb::robot_one_facts::kCascadePorts.value().second, 11},
                                  3},
    sb::robot_one::kRobotOneMechanisms));
// expect: a_motor_the_devices_screen_lists_is_not_a_drive_motor_and_is_not_declared_in_robot_ones_list
