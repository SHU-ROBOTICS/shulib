#pragma once

#include "pros/motor_group.hpp"
#include <map>
#include <string>

namespace shulib {

class Drivetrain {
public:
  Drivetrain(float wheelDiameter, float rpm, float horizontalDrift)
      : wheelDiameter(wheelDiameter), rpm(rpm),
        horizontalDrift(horizontalDrift) {}

  virtual ~Drivetrain() = default;

  // Generic drive method using motor configurations
  virtual void drive(int horizontal, int vertical, int turn, bool fieldCentric);

  // Generic brake mode setter
  virtual void setBrakeMode(pros::motor_brake_mode_e mode);

  // Generic max voltage setter
  virtual void setMaxVoltage(int voltage);

  // Configuration for each motor group
  struct MotorConfig {
    std::string name;
    pros::MotorGroup *motors;
    float horizontalCoefficient;
    float verticalCoefficient;
    float turnCoefficient;
  };

  // MotorConfig getter
  virtual std::vector<MotorConfig> getMotorConfigs() { return motorConfigs; }

  // getTemps
  std::map<std::string, double> getTemps();

  // tostring
  virtual std::string toString();

protected:
  float wheelDiameter;
  float rpm;
  float horizontalDrift;

  std::vector<MotorConfig> motorConfigs;
};

} // namespace shulib