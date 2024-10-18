#pragma once

#include "pros/motor_group.hpp"
#include "shulib/chassis/drivetrain.hpp"

namespace shulib {

class TankDrive : public Drivetrain {
public:
  TankDrive(pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors, float trackWidth,
            float wheelDiameter, float rpm, float horizontalDrift)
      : Drivetrain(wheelDiameter, rpm, horizontalDrift),
        trackWidth(trackWidth) {
    // Configure left motors
    MotorConfig leftConfig = {&leftMotors, 0, 1, 1};
    motorConfigs.push_back(leftConfig);

    // Configure right motors
    MotorConfig rightConfig = {&rightMotors, 0, 1, -1};
    motorConfigs.push_back(rightConfig);
  }

private:
  float trackWidth;
};

} // namespace shulib