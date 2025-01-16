#pragma once

#include "pros/imu.hpp"
#include "shulib/chassis/odomUnit.hpp"
#include "shulib/chassis/drivetrain.hpp"
#include "shulib/pose.hpp"
#include "shulib/RobotCommands/CommandStruct.hpp"
#include "shulib/RobotCommands/Command.hpp"
#include "shulib/pid.hpp"
#include "shulib/exitcondition.hpp"

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

  /**
   * @brief AngularDirection
   *
   * When turning, the user may want to specify the direction the robot should turn in.
   * This enum class has 3 values: CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, and AUTO
   * AUTO will make the robot turn in the shortest direction, and will be the most used value
   */
  enum class AngularDirection
  {
    CW_CLOCKWISE,         /** turn clockwise */
    CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
    AUTO                  /** turn in the direction with the shortest distance to target */
  };

  /**
   * @brief class containing constants for a chassis controller
   */
  class ControllerSettings
  {
  public:
    /**
     * @brief ControllerSettings constructor
     *
     * The constants are stored in a class so that they can be easily passed to the chassis class
     * Set a constant to 0 and it will be ignored
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param antiWindup integral anti windup range. If error is within this range, integral is set to 0
     * @param smallError range of error at which the chassis controller will exit if the error is within this range
     * for an amount of time determined by smallErrorTimeout
     * @param smallErrorTimeout the time the chassis controller will wait before exiting if error is within a
     * certain range determined by smallError
     * @param largeError range of error at which the chassis controller will exit if the error is within this range
     * for an amount of time determined by largeErrorTimeout
     * @param largeErrorTimeout the time the chassis controller will wait before exiting if error is within a
     * certain range determined by largeError
     * @param slew maximum acceleration
     *
     * @b Example
     * @code {.cpp}
     * lemlib::ControllerSettings lateralSettings(10, // proportional gain (kP)
     *                                            0, // integral gain (kI), set to 0 to disable
     *                                            3, // derivative gain (kD), set to 3
     *                                            3, // integral anti windup range, set to 0 to disable
     *                                            1, // small error range, in inches
     *                                            100, // small error range timeout, in milliseconds
     *                                            3, // large error range, in inches
     *                                            500, // large error range timeout, in milliseconds
     *                                            5); // maximum acceleration (slew)
     * @endcode
     */
    ControllerSettings(float kP, float kI, float kD, float windupRange, float smallError, float smallErrorTimeout,
                       float largeError, float largeErrorTimeout, float slew)
        : kP(kP),
          kI(kI),
          kD(kD),
          windupRange(windupRange),
          smallError(smallError),
          smallErrorTimeout(smallErrorTimeout),
          largeError(largeError),
          largeErrorTimeout(largeErrorTimeout),
          slew(slew) {}

    float kP;
    float kI;
    float kD;
    float windupRange;
    float smallError;
    float smallErrorTimeout;
    float largeError;
    float largeErrorTimeout;
    float slew;
  };

  class Chassis
  {
  public:
    Chassis(Drivetrain drivetrain, OdomSensors sensors, ControllerSettings lateralSettings, ControllerSettings angularSettings);

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

    void addCommand(Command *command); // Method to register commands

    void executeCommands(); // Execute all registered commands

    void followPath(CommandStruct *commands, size_t commandCount);

    PID lateralPID;
    PID angularPID;

  protected:
    /**
     * @brief Indicates that this motion is queued and blocks current task until this motion reaches front of queue
     */
    void requestMotionStart();
    /**
     * @brief Dequeues this motion and permits queued task to run
     */
    void endMotion();

    bool motionRunning = false;
    bool motionQueued = false;

    float distTraveled = 0;

    ControllerSettings lateralSettings;
    ControllerSettings angularSettings;
    Drivetrain drivetrain;
    OdomSensors sensors;
    // DriveCurve *throttleCurve;
    // DriveCurve *steerCurve;

    ExitCondition lateralLargeExit;
    ExitCondition lateralSmallExit;
    ExitCondition angularLargeExit;
    ExitCondition angularSmallExit;

  private:
    pros::Mutex mutex;
    std::vector<Command *> commands; // Vector to store registered commands
  };

} // namespace shulib