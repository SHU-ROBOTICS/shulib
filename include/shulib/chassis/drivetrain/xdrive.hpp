#pragma once

#include "pros/motor_group.hpp"
#include "shulib/chassis/drivetrain.hpp"

namespace shulib {

class XDrive : public Drivetrain {
public:
  XDrive(pros::MotorGroup &frontLeft, pros::MotorGroup &frontRight,
         pros::MotorGroup &backLeft, pros::MotorGroup &backRight,
         float wheelDiameter, float rpm, float horizontalDrift)
      : Drivetrain(wheelDiameter, rpm, horizontalDrift) {
    // Front Left Motor Configuration
    MotorConfig flConfig = {&frontLeft, 1, 1, 1};
    motorConfigs.push_back(flConfig);

    // Front Right Motor Configuration
    MotorConfig frConfig = {&frontRight, -1, 1, -1};
    motorConfigs.push_back(frConfig);

    // Back Left Motor Configuration
    MotorConfig blConfig = {&backLeft, -1, 1, 1};
    motorConfigs.push_back(blConfig);

    // Back Right Motor Configuration
    MotorConfig brConfig = {&frontLeft, 1, 1, -1};
    motorConfigs.push_back(brConfig);
  }
};

} // namespace shulib