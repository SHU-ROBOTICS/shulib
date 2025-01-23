#pragma once

#include "pros/imu.hpp"
#include "shulib/chassis/odomUnit.hpp"
#include "shulib/chassis/drivetrain.hpp"
#include "shulib/pose.hpp"
#include "shulib/RobotCommands/CommandStruct.hpp"
#include "shulib/RobotCommands/Command.hpp"

namespace shulib
{

  class OdomSensors
  {
  public:
    OdomSensors(OdomUnit *left, OdomUnit *right, OdomUnit *back,
                pros::Imu *imu);
    OdomUnit *left;
    OdomUnit *right;
    OdomUnit *back;
    pros::Imu *imu;
  };

  class Chassis
  {
  public:
    Chassis(Drivetrain drivetrain, OdomSensors sensors);

    void calibrate(bool calibrateImu = true);

    void setPose(float x, float y, float theta, bool radians = false);

    void setPose(Pose pose, bool radians = false);

    Pose getPose(bool radians = false);

    void setBrakeMode(pros::motor_brake_mode_e mode);

    void drive(int horizontal, int vertical, int turn, bool fieldCentric = false);

    void resetLocalPosition();

    void init();

  protected:
    float distTraveled = 0;

    Drivetrain drivetrain;
    OdomSensors sensors;

  private:
    pros::Mutex mutex;
  };

} // namespace shulib