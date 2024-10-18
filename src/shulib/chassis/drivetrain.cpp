#include "shulib/chassis/drivetrain.hpp"
#include "shulib/chassis/odometry.hpp"
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
    config.motors->move(motorOutput);
  }
}

void shulib::Drivetrain::setBrakeMode(pros::motor_brake_mode_e mode) {
  for (const auto& config : motorConfigs) {
    config.motors->set_brake_mode_all(mode);
  }
}