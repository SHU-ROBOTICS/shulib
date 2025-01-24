#pragma once

#include "pros/motor_group.hpp"
#include "shulib/chassis/drivetrain.hpp"
#include <map>
#include <string>

namespace shulib {

class TankDrive : public Drivetrain {
public:
  TankDrive(pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors, float trackWidth,
            float wheelDiameter, float rpm, float horizontalDrift)
      : Drivetrain(wheelDiameter, rpm, horizontalDrift),
        trackWidth(trackWidth) {
    // Configure left motors
    MotorConfig leftConfig = {"left", &leftMotors, 0, 1, 1};
    motorConfigs.push_back(leftConfig);

    // Configure right motors
    MotorConfig rightConfig = {"right", &rightMotors, 0, 1, -1};
    motorConfigs.push_back(rightConfig);
  }

  // getTemps
  std::map<std::string, double> getTemps();

  // tostring
  std::string toString() override { return "TankDrive"; }

private:
  float trackWidth;
};

std::map<std::string, double> TankDrive::getTemps() {
  std::map<std::string, double> temps;
  for (auto &motorConfig : motorConfigs) {
    int i = 0;
    for (auto &temp : motorConfig.motors->get_temperature_all()) {
      temps[motorConfig.name + "_" + std::to_string(i)] = temp;
      i++;
    }
  }
  return temps;
}

} // namespace shulib