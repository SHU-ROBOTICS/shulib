#pragma once

#include "pros/imu.hpp"
#include "shulib/chassis/odomUnit.hpp"
#include "shulib/chassis/drivetrain.hpp"
#include "shulib/pose.hpp"
#include "shulib/RobotCommands/CommandStruct.hpp"
#include "shulib/RobotCommands/Command.hpp"

namespace shulib {

class OdomSensors {
public:
  OdomSensors(OdomUnit *left, OdomUnit *right, OdomUnit *back,
              pros::Imu *imu);
  OdomUnit *left;
  OdomUnit *right;
  OdomUnit *back;
  pros::Imu *imu;
};

/**
 * @brief AngularDirection
 *
 * When turning, the user may want to specify the direction the robot should turn in.
 * This enum class has 3 values: CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, and AUTO
 * AUTO will make the robot turn in the shortest direction, and will be the most used value
 */
enum class AngularDirection {
    CW_CLOCKWISE, /** turn clockwise */
    CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
    AUTO /** turn in the direction with the shortest distance to target */
};

class Chassis {
public:
  Chassis(Drivetrain drivetrain, OdomSensors sensors);

  void calibrate(bool calibrateImu = true);

  void setPose(float x, float y, float theta, bool radians = false);

  void setPose(Pose pose, bool radians = false);

  Pose getPose(bool radians = false);
  
  void setBrakeMode(pros::motor_brake_mode_e mode);

  void drive(int horizontal, int vertical, int turn, bool fieldCentric = false);

  void cancelMotion();

  void cancelAllMotions();

  bool isInMotion() const;

  void resetLocalPosition();

  void followPath();

  void addCommand(Command* command);  // Method to register commands

  void executeCommands();             // Execute all registered commands

  void followPath(CommandStruct* commands, size_t commandCount); 

protected:
  bool motionRunning = false;
  bool motionQueued = false;

  float distTraveled = 0;

  //ControllerSettings lateralSettings;
  //ControllerSettings angularSettings;
  Drivetrain drivetrain;
  OdomSensors sensors;
  //DriveCurve *throttleCurve;
  //DriveCurve *steerCurve;

  //ExitCondition lateralLargeExit;
  //ExitCondition lateralSmallExit;
  //ExitCondition angularLargeExit;
  //ExitCondition angularSmallExit;

private:
  pros::Mutex mutex;
  std::vector<Command*> commands;  // Vector to store registered commands
};

} // namespace shulib