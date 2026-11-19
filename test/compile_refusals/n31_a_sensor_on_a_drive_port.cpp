// n31_a_sensor_on_a_drive_port: a mechanism's ANGLE SENSOR wired to one of the OLD robot two's DRIVE ports
// (13, its left side), checked beside its signed chassis table (test/retired_robot_two.hpp since ROB1-BT: the
// twin's table types no port) by the robot table file's own drive check (src/robots/mechanism_table.hpp's
// clearOfItsDrive — the static_asserts at its end; MECH-A5). A sensor role, not a motor: the drive check
// reads EVERY smart role (until MECH-A5 it read the motors only — the MECH brief's M29). run_refusal.py judges it.
#include "refusal_common.hpp"
#include "../retired_robot_two.hpp"
#include "robots/mechanism_table.hpp"
inline constexpr MechanismTable m = mechanisms(wire(ry::wrist, motors(1), angleSensor(13)));
static_assert(shulib::bench::clearOfItsDrive(m, retired_robot_two::kChassis));
// expect: a_mechanism_is_wired_to_a_drive_port_or_the_imu_port_of_its_robots_chassis_table
// expect-at: 10
