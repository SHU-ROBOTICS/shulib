#include "shulib/chassis/drivetrain.hpp"
#include "shulib/chassis/odometry.hpp"
#include <cmath>
#include <map>


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
    config.motors->move(motorOutput);
  }
}

void shulib::Drivetrain::driveCurve(int horizontal, int vertical, int turn, int coeff, bool fieldCentric) {
  if (fieldCentric) {
    double angle = shulib::getPose().theta;
    double cosA = cos(angle);
    double sinA = sin(angle);
    horizontal = horizontal * cosA - vertical * sinA;
    vertical = horizontal * sinA + vertical * cosA;
  }

  MotorConfig configL = motorConfigs[0];
  MotorConfig configR = motorConfigs[1];

  int motorOutputA = horizontal * configL.horizontalCoefficient +
  vertical * configL.verticalCoefficient +
  turn * configL.turnCoefficient;
  configL.motors->move(motorOutputA);

  int motorOutputB = motorOutputA * coeff;
  configR.motors->move(motorOutputB);

}

void shulib::Drivetrain::setBrakeMode(pros::motor_brake_mode_e mode) {
  for (const auto& config : motorConfigs) {
    config.motors->set_brake_mode_all(mode);
  }
}

std::map<std::string, double> shulib::Drivetrain::getTemps() {
  std::map<std::string, double> temps;
  for (const auto& config : motorConfigs) {
    int i = 0;
    for (const auto& temp : config.motors->get_temperature_all()) {
      temps[config.name + "_" + std::to_string(i)] = temp;
      i++;
    }
  }
  return temps;
}

std::string shulib::Drivetrain::toString() {
  return "Drivetrain";
}