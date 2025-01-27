#include "shulib/chassis/drivetrain.hpp"
#include "shulib/chassis/odometry.hpp"
#include <map>
#include <string>
#include <cmath>


void shulib::Drivetrain::drive(int horizontal, int vertical, int turn,
                               bool fieldCentric) {
    if (fieldCentric) {
        double angle = shulib::getPose().theta;
        double cosA = cos(angle);
        double sinA = sin(angle);
        horizontal = horizontal * cosA - vertical * sinA;
        vertical = horizontal * sinA + vertical * cosA;
    }
  for (const auto &config : motorConfigs) {
    int motorOutput = horizontal * config.horizontalCoefficient +
                      vertical * config.verticalCoefficient +
                      turn * config.turnCoefficient;
    config.motors->move_voltage(std::min(motorOutput*5000, 6000));
  }
}

void shulib::Drivetrain::setBrakeMode(pros::motor_brake_mode_e mode) {
  for (const auto& config : motorConfigs) {
    config.motors->set_brake_mode_all(mode);
  }
}

void shulib::Drivetrain::setMaxVoltage(int voltage) {
  for (const auto& config : motorConfigs) {
    config.motors->set_voltage_limit(voltage);
  }
}


std::map<std::string, double> shulib::Drivetrain::getTemps() {
  std::map<std::string, double> temps;
  for (const auto& config : motorConfigs) {
    int i = 0;
    for (const auto& temp : config.motors->get_voltage_all()) {
      temps[config.name + "_" + std::to_string(i)] = temp;
      i++;
    }
  }
  return temps;
}

std::string shulib::Drivetrain::toString() {
  return "Drivetrain";
}