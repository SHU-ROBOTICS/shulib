#pragma once

#ifdef __cplusplus

#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>

namespace shulib {

/**
 * @brief Drivetrain configuration
 * Defines motor ports and physical dimensions for the drivetrain
 */
struct DrivetrainConfig {
    std::vector<int8_t> left_ports;   // Negative port = reversed motor
    std::vector<int8_t> right_ports;
    double track_width;               // Distance between left and right wheels (inches)
    double wheel_diameter;            // Drive wheel diameter (inches)
    int rpm;                          // Motor cartridge RPM (100, 200, or 600)
};

/**
 * @brief Tracking wheel configuration
 * Defines rotation sensor ports and physical dimensions for odometry
 */
struct TrackingConfig {
    int8_t left_port;                 // Negative port = reversed sensor
    int8_t right_port;
    int8_t back_port;
    double wheel_diameter;            // Tracking wheel diameter (inches)
    double left_offset;               // Distance from tracking center (inches, negative = left of center)
    double right_offset;              // Distance from tracking center (inches, positive = right of center)
    double back_offset;               // Distance from tracking center (inches)
};

/**
 * @brief Mechanism configuration
 * Defines motor ports for intake, conveyor, and other mechanisms
 */
struct MechanismConfig {
    std::vector<int8_t> intake_ports;
    std::vector<int8_t> conveyor_ports;
    std::vector<int8_t> releaser_ports;
    char pneumatic_arm_port;          // ADI port letter ('A' - 'H')
    char pneumatic_lever_port;        // ADI port letter ('A' - 'H')
    bool arm_default_state;           // true = extended at start, false = retracted
    bool lever_default_state;
};

/**
 * @brief Complete robot configuration
 * Contains all information needed to initialize a robot
 */
struct RobotConfig {
    std::string name;                 // Display name for the robot
    DrivetrainConfig drivetrain;
    TrackingConfig tracking;
    MechanismConfig mechanisms;
};

}  // namespace shulib

#endif  // __cplusplus