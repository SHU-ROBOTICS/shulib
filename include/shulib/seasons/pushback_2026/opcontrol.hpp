#pragma once

#include "shulib/core/chassis.hpp"
#include "shulib/robots/robot_config.hpp"

namespace shulib::seasons::pushback::opcontrol {

/**
 * @brief Run the driver control loop
 * 
 * This function handles all driver input and controls the robot
 * during the driver control period.
 * 
 * @param chassis Reference to the chassis for driving
 * @param config Reference to the robot configuration
 */
void run(Chassis& chassis, const RobotConfig& config);

}  // namespace shulib::seasons::pushback::opcontrol
