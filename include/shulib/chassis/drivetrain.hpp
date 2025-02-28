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

  // driving method for differential turns
  virtual void driveCurve(int horizontal, int vertical, int turn, float coeff, bool fieldCentric); 

  float getWheelDiameter(){
    return wheelDiameter;
  }

  // Configuration for each motor group
  struct MotorConfig {
    pros::MotorGroup *motors;
    float horizontalCoefficient;
    float verticalCoefficient;
    float turnCoefficient;
    std::string name;
  };

  // MotorConfig getter
  virtual std::vector<MotorConfig> getMotorConfigs() { return motorConfigs; }

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

