#pragma once

#include "pros/motor_group.hpp"
#include "shulib/core/drivetrain.hpp"

namespace shulib {

class TankDrive : public Drivetrain {
public:
  TankDrive(pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors, float trackWidth,
            float wheelDiameter, float rpm)
      : Drivetrain(wheelDiameter, rpm, 0),
        trackWidth(trackWidth) {
    // Configure left motors
    MotorConfig leftConfig = {&leftMotors, 0, 1, 1};
    motorConfigs.push_back(leftConfig);

    // Configure right motors
    MotorConfig rightConfig = {&rightMotors, 0, -1, 1};
    motorConfigs.push_back(rightConfig);
  }

  // tostring
  std::string toString() override { return "TankDrive"; }

private:
  float trackWidth;
};

} // namespace shulib