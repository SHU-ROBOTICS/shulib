// n31_a_sensor_on_a_drive_port: a mechanism's ANGLE SENSOR wired to one of robot two's DRIVE ports
// (13, its left side), checked beside robot two's chassis table by the robot table file's own drive
// check (src/robots/mechanism_table.hpp's clearOfItsDrive — the static_asserts at its end; MECH-A5).
// A sensor role, not a motor: the drive check reads EVERY smart role (until MECH-A5 it read the
// motors only — the MECH brief's M29). Written by MECH-A5; run_refusal.py judges it.
#include "refusal_common.hpp"
#include "robots/mechanism_table.hpp"
inline constexpr MechanismTable m = mechanisms(wire(ry::wrist, motors(1), angleSensor(13)));
static_assert(shulib::bench::clearOfItsDrive(m, shulib::bench::kRobotTwoChassis));
// expect: a_mechanism_is_wired_to_a_drive_port_or_the_imu_port_of_its_robots_chassis_table
// expect-at: 9
