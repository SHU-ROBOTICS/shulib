#pragma once

#ifdef __cplusplus
#include "robot_config.hpp"

namespace shulib {
namespace robots {

/**
 * @brief TestBot Robot Configuration
 * 
 * Drivetrain: 10-motor tank drive
 * Tracking: 3-wheel odometry (no IMU)
 * 
 * NOTE: This config is optimized for FORWARD motion in opcontrol.
 * The autonomous routines in auton.cpp use direct motor control
 * with different port signs for forward vs turning operations.
 */
inline const RobotConfig TESTBOT = {
    "TestBot",

    DrivetrainConfig{
        {-12, -14, -16, -18, -20},   // Left motors (forward config)
        {11, 13, 15, 17, 19},        // Right motors
        15.0,
        3.25,
        400
    },

    TrackingConfig{
        -8,                           // Left tracking port
        10,                           // Right tracking port
        9,                            // Back tracking port
        2.75,                         // Tracking wheel diameter
        -6.5,                         // Left offset
        6.5,                          // Right offset
        0.0                           // Back offset
    },

    MechanismConfig{
        {-6, 7},                      // Intake ports
        {2, -3, -4, 5},               // Conveyor ports
        {1},                          // Releaser ports
        'B',                          // Pneumatic arm port
        'C',                          // Pneumatic lever port
        false,                        // Arm default state
        false                         // Lever default state
    }
};

}  // namespace robots
}  // namespace shulib

#endif  // __cplusplus