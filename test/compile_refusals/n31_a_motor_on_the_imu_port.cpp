// n31_a_motor_on_the_imu_port: a claw pivot wired to robot two's IMU port (2), checked beside robot
// two's chassis table by the robot table file's own drive check (src/robots/mechanism_table.hpp's
// clearOfItsDrive; MECH-A5). The IMU is the drive's too: the boot would hand its port to a motor
// adapter. Written by MECH-A5; run_refusal.py judges it.
#include "refusal_common.hpp"
#include "robots/mechanism_table.hpp"
inline constexpr MechanismTable m = mechanisms(wire(r2::clawPivot6, motors(2)));
static_assert(shulib::bench::clearOfItsDrive(m, shulib::bench::kRobotTwoChassis));
// expect: a_mechanism_is_wired_to_a_drive_port_or_the_imu_port_of_its_robots_chassis_table
// expect-at: 8
