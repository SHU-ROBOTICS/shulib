#pragma once

#ifdef __cplusplus

#include "robot_config.hpp"

namespace shulib {
namespace robots {

/**
 * @brief Queens Revenge Robot Configuration
 * 
 * Drivetrain: 10-motor tank drive
 * Tracking: 3-wheel odometry (no IMU)
 */
inline const RobotConfig QUEENS_REVENGE = {
    "Queens Revenge",
    DrivetrainConfig{
        {11, -12, 13, -14, -15},
        {16, -17, 18, -19, 20},
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
        2.5
    },
    MechanismConfig{
        {2, -3},
        {4, -5},
        {-6, 7},
        'B',
        'C',
        false,
        false
    }
};

}  // namespace robots
}  // namespace shulib

#endif  // __cplusplus