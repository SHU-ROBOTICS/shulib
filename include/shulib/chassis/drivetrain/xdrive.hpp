#pragma once

#include "pros/motor_group.hpp"
#include "shulib/chassis/drivetrain.hpp"
#include <map>
#include <string>

namespace shulib {

class XDrive : public Drivetrain {
public:
  XDrive(pros::MotorGroup &frontLeft, pros::MotorGroup &frontRight,
         pros::MotorGroup &backLeft, pros::MotorGroup &backRight,
         float wheelDiameter, float rpm, float horizontalDrift)
      : Drivetrain(wheelDiameter, rpm, horizontalDrift) {
    // Front Left Motor Configuration
    MotorConfig flConfig = {"front_left", &frontLeft, 1, 1, 1};
    motorConfigs.push_back(flConfig);

    // Front Right Motor Configuration
    MotorConfig frConfig = {"front_right", &frontRight, -1, 1, -1};
    motorConfigs.push_back(frConfig);

    // Back Left Motor Configuration
    MotorConfig blConfig = {"back_left", &backLeft, -1, 1, 1};
    motorConfigs.push_back(blConfig);

    // Back Right Motor Configuration
    MotorConfig brConfig = {"back_right", &backRight, 1, 1, -1};
    motorConfigs.push_back(brConfig);
  }

  // getTemps
  std::map<std::string, double> getTemps();

  // tostring
  std::string toString() { return "XDrive"; }
};

std::map<std::string, double> XDrive::getTemps() {
  std::map<std::string, double> temps;
  for (auto &motorConfig : motorConfigs) {
    temps[motorConfig.name] = motorConfig.motors->get_temperature();
  }
  return temps;
}

} // namespace shulib