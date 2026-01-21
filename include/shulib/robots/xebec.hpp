#pragma once

#ifdef __cplusplus

#include "robot_config.hpp"

namespace shulib {
namespace robots {

/**
 * @brief XEBEC Robot Configuration
 * 
 * Drivetrain: 10-motor tank drive
 * Tracking: 3-wheel odometry (no IMU)
 */
inline const RobotConfig XEBEC = {
    "XEBEC",
    DrivetrainConfig{
        {12, -14, 16, -18, 20},
        {11, -13, 15, -17, 19},
        15.0,
        3.25,
        400
    },
    TrackingConfig{
        -8,
        10,
        9,
        2.75,
        -6.5,
        6.5,
        0.0
    },
    MechanismConfig{
        {-6, 7},
        {2, -3, -4, 5},
        {1},
        'B',
        'C',
        false,
        false
    }
};

}  // namespace robots
}  // namespace shulib

#endif  // __cplusplus