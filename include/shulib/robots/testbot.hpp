#pragma once

#ifdef __cplusplus
#include "robot_config.hpp"

namespace shulib {
namespace robots {

/**
 * @brief TestBot Robot Configuration
 * 
 * Drivetrain: 10-motor tank drive (5 per side)
 * Tracking: 3-wheel odometry (no IMU)
 * 
 * Motor Layout (physically stacked, alternating directions):
 * 
 *   RIGHT SIDE (11-15):         LEFT SIDE (16-20):
 *     Port 11: Forward  → +       Port 16: Backward → -
 *     Port 12: Backward → -       Port 17: Forward  → +
 *     Port 13: Forward  → +       Port 18: Backward → -
 *     Port 14: Backward → -       Port 19: Forward  → +
 *     Port 15: Forward  → +       Port 20: Backward → -
 * 
 * TESTED AND WORKING: 2026-01-21
 */
inline const RobotConfig TESTBOT = {
    "TestBot",

    DrivetrainConfig{
        {-16, 17, -18, 19, -20},       // Left motors
        {11, -12, 13, -14, 15},        // Right motors
        15.0,                           // Track width (inches)
        3.25,                           // Wheel diameter (inches)
        400                             // Motor RPM
    },

    TrackingConfig{
        -8,                             // Left tracking port
        10,                             // Right tracking port
        9,                              // Back tracking port
        2.75,                           // Tracking wheel diameter
        -6.5,                           // Left offset
        6.5,                            // Right offset
        0.0                             // Back offset
    },

    MechanismConfig{
        {-6, 7},                        // Intake ports
        {2, -3, -4, 5},                 // Conveyor ports
        {1},                            // Releaser ports
        'B',                            // Pneumatic arm port
        'C',                            // Pneumatic lever port
        false,                          // Arm default state
        false                           // Lever default state
    }
};

}  // namespace robots
}  // namespace shulib

#endif  // __cplusplus