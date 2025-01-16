#include "shulib/chassis/chassis.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "shulib/RobotCommands/MoveWithHeadingCommand.hpp"
#include "shulib/chassis/odomUnit.hpp"
#include "shulib/chassis/odometry.hpp"
#include "shulib/logger.hpp"
#include "shulib/timer.hpp"
#include "shulib/util.hpp"
#include <math.h>

shulib::OdomSensors::OdomSensors(OdomUnit *left, OdomUnit *right,
                                 OdomUnit *back, pros::Imu *imu)
    : left(left), right(right), back(back), imu(imu) {}

shulib::Chassis::Chassis(Drivetrain drivetrain, OdomSensors sensors, ControllerSettings lateralSettings, ControllerSettings angularSettings)
    : drivetrain(drivetrain), sensors(sensors),
      lateralSettings(lateralSettings),
      angularSettings(angularSettings),
      lateralPID(lateralSettings.kP, lateralSettings.kI, lateralSettings.kD, lateralSettings.windupRange, true), // kP, kI, kD, windupRange, signFlipReset
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, true), // kP, kI, kD, windupRange, signFlipReset
      lateralLargeExit(lateralSettings.largeError, lateralSettings.largeErrorTimeout),
      lateralSmallExit(lateralSettings.smallError, lateralSettings.smallErrorTimeout),
      angularLargeExit(angularSettings.largeError, angularSettings.largeErrorTimeout),
      angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout)
{

  logger().log("Chassis constructor - Linear controller settings:");
  logger().log("  kP:", lateralPID.getKP());
  logger().log("  kI:", lateralPID.getKI());
  logger().log("  kD:", lateralPID.getKD());
  logger().log("Angular controller settings:");
  logger().log("  kP:", angularPID.getKP());
  logger().log("  kI:", angularPID.getKI());
  logger().log("  kD:", angularPID.getKD());
}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(shulib::OdomSensors &sensors)
{
  std::cout << "Calibrating IMU" << std::endl;
  int attempt = 1;
  bool calibrated = false;
  // calibrate inertial, and if calibration fails, then repeat 5 times or until
  // successful
  while (attempt <= 5)
  {
    sensors.imu->reset();
    // wait until IMU is calibrated
    do
      pros::delay(10);
    while (sensors.imu->get_status() != pros::ImuStatus::error &&
           sensors.imu->is_calibrating());
    // exit if imu has been calibrated
    if (!isnanf(sensors.imu->get_heading()) &&
        !isinf(sensors.imu->get_heading()))
    {
      calibrated = true;
      std::cout << "IMU calibrated successfully" << std::endl;
      break;
    }
    // indicate error
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
    std::cout << "IMU failed to calibrate! Attempt #" << attempt << std::endl;
    // shulib::infoSink()->warn("IMU failed to calibrate! Attempt #{}",
    // attempt);
    attempt++;
  }
  // check if calibration attempts were successful
  if (attempt > 5)
  {
    sensors.imu = nullptr;
    // shulib::infoSink()->error("IMU calibration failed, defaulting to tracking
    // "
    //                           "wheels / motor encoders");
  }
}

void shulib::Chassis::calibrate(bool calibrateImu)
{
  // calibrate the IMU if it exists and the user doesn't specify otherwise
  if (sensors.imu != nullptr && calibrateImu)
    calibrateIMU(sensors);
  // initialize odom
  // if sensors are nullptrs, error
  if (sensors.left == nullptr)
    throw std::runtime_error("Left tracking wheel not initialized");
  if (sensors.right == nullptr)
    throw std::runtime_error("Left tracking wheel not initialized");
  if (sensors.back == nullptr)
    throw std::runtime_error("Back tracking wheel not initialized");

  sensors.left->reset();
  sensors.right->reset();
  sensors.back->reset();

  std::cout << "Tracking wheels calibrated!" << std::endl;

  setPose(Pose(0, 0, 0), false);
  setSensors(sensors, drivetrain);
  init();
  // rumble to controller to indicate success
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
  std::cout << "Chassis calibrated!" << std::endl;
}

void shulib::Chassis::setPose(float x, float y, float theta, bool radians)
{
  shulib::setPose(shulib::Pose(x, y, theta), radians);
}

void shulib::Chassis::setPose(shulib::Pose pose, bool radians)
{
  shulib::setPose(pose, radians);
}

shulib::Pose shulib::Chassis::getPose(bool radians)
{
  Pose pose = shulib::getPose(true);
  if (!radians)
    pose.theta = radToDeg(pose.theta);
  return pose;
}

void shulib::Chassis::drive(int horizontal, int vertical, int turn,
                            bool fieldCentric)
{
  drivetrain.drive(horizontal, vertical, turn, fieldCentric);
}

void shulib::Chassis::resetLocalPosition()
{
  float theta = this->getPose().theta;
  shulib::setPose(shulib::Pose(0, 0, theta), false);
}

void shulib::Chassis::setBrakeMode(pros::motor_brake_mode_e mode)
{
  drivetrain.setBrakeMode(mode);
}

// Function to convert degrees to radians
inline float degToRad(float degrees) { return degrees * M_PI / 180.0; }

void shulib::Chassis::requestMotionStart()
{
  if (this->isInMotion())
    this->motionQueued = true;
  else
    this->motionRunning = true; // indicate a motion is running

  // wait until this motion is at front of "queue"
  this->mutex.take(TIMEOUT_MAX);

  // this->motionRunning should be true
  // and this->motionQueued should be false
  // indicating this motion is running
}

void shulib::Chassis::endMotion()
{
  // move the "queue" forward 1
  this->motionRunning = this->motionQueued;
  this->motionQueued = false;

  // permit queued motion to run
  this->mutex.give();
}

void shulib::Chassis::cancelMotion()
{
  this->motionRunning = false;
  pros::delay(10); // give time for motion to stop
}

void shulib::Chassis::cancelAllMotions()
{
  this->motionRunning = false;
  this->motionQueued = false;
  pros::delay(10); // give time for motion to stop
}

bool shulib::Chassis::isInMotion() const { return this->motionRunning; }

void shulib::Chassis::followPath(CommandStruct *commands, size_t commandCount)
{
  for (size_t i = 0; i < commandCount; ++i)
  {
    CommandStruct &cmd = commands[i];

    switch (cmd.command)
    {
    case CMD_MOVE_WITH_HEADING:
    {
      MoveWithHeadingCommand moveCmd(cmd.x, cmd.y, cmd.heading, cmd.speed);
      moveCmd.execute();
      break;
    }
    case CMD_PICK_UP:
    {
      // Implement PickUpCommand similarly
      break;
    }
    case CMD_PLACE:
    {
      // Implement PlaceCommand similarly
      break;
    }
    // Add other cases here...
    default:
      std::cerr << "Unknown command type" << std::endl;
      break;
    }
  }
}

struct TurnToHeadingParams
{
  /** the direction the robot should turn in. AUTO by default */
  shulib::AngularDirection direction = shulib::AngularDirection::AUTO;
  /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
  int maxSpeed = 127;
  /** the minimum speed the robot can turn at. If set to a non-zero value, the `it conditions will switch to less
   * accurate but smoother ones. Value between 0-127. 0 by default */
  int minSpeed = 0;
  /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
};

// void shulib::Chassis::turnToHeading(float theta, int timeout, bool async)
// {
//   TurnToHeadingParams params;
//   params.minSpeed = std::abs(params.minSpeed);
//   this->requestMotionStart();
//   // were all motions cancelled?
//   if (!this->motionRunning)
//     return;
//   // if the function is async, run it in a new task
//   if (async)
//   {
//     pros::Task task([&]()
//                     { turnToHeading(theta, timeout, false); });
//     this->endMotion();
//     pros::delay(10); // delay to give the task time to start
//     return;
//   }
//   float targetTheta;
//   float deltaTheta;
//   float motorPower;
//   float prevMotorPower = 0;
//   float startTheta = getPose().theta;
//   bool settling = false;
//   std::optional<float> prevRawDeltaTheta = std::nullopt;
//   std::optional<float> prevDeltaTheta = std::nullopt;
//   std::uint8_t compState = pros::competition::get_status();
//   distTraveled = 0;
//   Timer timer(timeout);
//   angularLargeExit.reset();
//   angularSmallExit.reset();
//   angularPID.reset();

//   // main loop
//   while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning)
//   {
//     // update variables
//     Pose pose = getPose();

//     // update completion vars
//     distTraveled = fabs(angleError(pose.theta, startTheta, false));

//     targetTheta = theta;

//     // check if settling
//     const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
//     if (prevRawDeltaTheta == std::nullopt)
//       prevRawDeltaTheta = rawDeltaTheta;
//     if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta))
//       settling = true;
//     prevRawDeltaTheta = rawDeltaTheta;

//     // calculate deltaTheta
//     if (settling)
//       deltaTheta = angleError(targetTheta, pose.theta, false);
//     else
//       deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
//     if (prevDeltaTheta == std::nullopt)
//       prevDeltaTheta = deltaTheta;

//     // motion chaining
//     if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange)
//       break;
//     if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta))
//       break;

//     // calculate the speed
//     motorPower = angularPID.update(deltaTheta);
//     angularLargeExit.update(deltaTheta);
//     angularSmallExit.update(deltaTheta);

//     // cap the speed
//     if (motorPower > params.maxSpeed)
//       motorPower = params.maxSpeed;
//     else if (motorPower < -params.maxSpeed)
//       motorPower = -params.maxSpeed;
//     if (fabs(deltaTheta) > 20)
//       motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
//     if (motorPower < 0 && motorPower > -params.minSpeed)
//       motorPower = -params.minSpeed;
//     else if (motorPower > 0 && motorPower < params.minSpeed)
//       motorPower = params.minSpeed;
//     prevMotorPower = motorPower;

//     logger().debug("Turn Motor Power: {"+std::to_string(motorPower)+"}");

//     // move the drivetrain
//     this->drive(0, 0, motorPower);

//     pros::delay(10);
//   }

//   // stop the drivetrain
//   this->drive(0, 0, 0);
//   // set distTraveled to -1 to indicate that the function has finished
//   distTraveled = -1;
//   this->endMotion();
// }

struct MoveToPoseParams
{
  /** whether the robot should move forwards or backwards. True by default */
  bool forwards = true;
  /** how fast the robot will move around corners. Recommended value 2-15. 0
   * means use horizontalDrift set in chassis class. 0 by default. */
  float horizontalDrift = 0.6;
  /** carrot point multiplier. value between 0 and 1. Higher values result in
   * curvier movements. 0.6 by default */
  float lead = 0.6;
  /** the maximum speed the robot can travel at. Value between 0-127. 127 by
   * default */
  float maxSpeed = 127;
  /** the minimum speed the robot can travel at. If set to a non-zero value, the
   * exit conditions will switch to less accurate but smoother ones. Value
   * between 0-127. 0 by default */
  float minSpeed = 0;
  /** distance between the robot and target point where the movement will exit.
   * Only has an effect if minSpeed is non-zero.*/
  float earlyExitRange = 0;
};

// most of this code is from lemlib
// void shulib::Chassis::moveToPose(Pose target, int timeout, bool async)
// {
//   // Take the mutex
//   this->requestMotionStart();

//   // Check if motions were cancelled
//   if (!this->motionRunning)
//     return;

//   // If async, run in new task
//   if (async)
//   {
//     pros::Task task([&]()
//                     { moveToPose(target, timeout, false); });
//     this->endMotion();
//     pros::delay(10); // delay to give the task time to start
//     return;
//   }

//   // Log start of movement
//   logger().log("Starting moveToPose - Target: (", target.x, ", ", target.y, ", ", target.theta, ")");
//   logger().log("PID Settings - Lateral: kP=", lateralPID.getKP(),
//                " kI=", lateralPID.getKI(), " kD=", lateralPID.getKD());
//   logger().log("PID Settings - Angular: kP=", angularPID.getKP(),
//                " kI=", angularPID.getKI(), " kD=", angularPID.getKD());

//   // Reset PIDs and exit conditions
//   lateralPID.reset();
//   lateralLargeExit.reset();
//   lateralSmallExit.reset();
//   angularPID.reset();
//   angularLargeExit.reset();
//   angularSmallExit.reset();

//   // Constants for movement control
//   const float LEAD = 0.6f;             // How far ahead the carrot point should be
//   const float HORIZONTAL_DRIFT = 8.0f; // Higher = less sideways slip
//   const float MAX_SPEED = 100.0f;      // Maximum motor power
//   const float MIN_SPEED = 10.0f;       // Minimum motor power to overcome friction
//   const float SETTLING_DIST = 7.5f;    // Distance to start settling behavior
//   const float EARLY_EXIT_RANGE = 1.0f; // Early exit tolerance

//   // Initialize variables
//   Pose lastPose = getPose(true);
//   Timer timer(timeout);
//   bool close = false;
//   bool lateralSettled = false;
//   bool prevSameSide = false;
//   float prevLateralOut = 0;
//   float prevAngularOut = 0;
//   distTraveled = 0;

//   while (!timer.isDone() &&
//          ((!lateralSettled || (!angularLargeExit.getExit() && !angularSmallExit.getExit())) || !close) &&
//          this->motionRunning)
//   {

//     // Get current pose and update distance traveled
//     const Pose pose = getPose(true);
//     distTraveled += pose.distance(lastPose);
//     lastPose = pose;

//     // Calculate distance to target
//     const float distTarget = pose.distance(target);

//     // Check if close enough to start settling
//     if (distTarget < SETTLING_DIST && !close)
//     {
//       close = true;
//       // Maintain current speed but cap it
//       float currentSpeed = fmax(fabs(prevLateralOut), 60.0f);
//       currentSpeed = fmin(currentSpeed, MAX_SPEED);
//     }

//     // Update lateral settling
//     if (lateralLargeExit.getExit() && lateralSmallExit.getExit())
//     {
//       lateralSettled = true;
//     }

//     // Calculate carrot point
//     Pose carrot = target;
//     if (!close)
//     {
//       carrot = target - Pose(cos(target.theta), sin(target.theta)) * LEAD * distTarget;
//     }

//     // Calculate if robot and carrot are on same side of target line
//     bool robotSide = (pose.y - target.y) * -sin(target.theta) <=
//                      (pose.x - target.x) * cos(target.theta) + EARLY_EXIT_RANGE;
//     bool carrotSide = (carrot.y - target.y) * -sin(target.theta) <=
//                       (carrot.x - target.x) * cos(target.theta) + EARLY_EXIT_RANGE;
//     bool sameSide = robotSide == carrotSide;

//     // Early exit if crossed target line while settling
//     if (!sameSide && prevSameSide && close)
//       break;
//     prevSameSide = sameSide;

//     // Calculate errors
//     float angularError = close ? angleError(pose.theta, target.theta) : angleError(pose.theta, pose.angle(carrot));

//     float lateralError = pose.distance(carrot);
//     if (close)
//     {
//       lateralError *= cos(angleError(pose.theta, pose.angle(carrot)));
//     }
//     else
//     {
//       lateralError *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));
//     }

//     // Update exit conditions
//     lateralSmallExit.update(lateralError);
//     lateralLargeExit.update(lateralError);
//     angularSmallExit.update(radToDeg(angularError));
//     angularLargeExit.update(radToDeg(angularError));

//     // Calculate PID outputs
//     float lateralOut = lateralPID.update(lateralError);
//     float angularOut = angularPID.update(radToDeg(angularError));

//     // Apply speed limits
//     lateralOut = std::clamp(lateralOut, -MAX_SPEED, MAX_SPEED);
//     angularOut = std::clamp(angularOut, -MAX_SPEED, MAX_SPEED);

//     // Calculate curvature and max safe speed
//     float radius = 1.0f / (fabs(getCurvature(pose, carrot)) + 0.0001f);
//     float maxSlipSpeed = sqrt(HORIZONTAL_DRIFT * radius * 9.81f);
//     lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);

//     // Prioritize angular movement
//     float overturn = fabs(angularOut) + fabs(lateralOut) - MAX_SPEED;
//     if (overturn > 0)
//     {
//       lateralOut -= lateralOut > 0 ? overturn : -overturn;
//     }

//     // Apply minimum speed
//     if (fabs(lateralOut) < MIN_SPEED && fabs(lateralOut) > 0)
//     {
//       lateralOut = lateralOut > 0 ? MIN_SPEED : -MIN_SPEED;
//     }

//     // Convert to robot-relative coordinates for holonomic drive
//     float robotRelativeX = (carrot.x - pose.x) * cos(pose.theta) -
//                            (carrot.y - pose.y) * sin(pose.theta);
//     float robotRelativeY = (carrot.x - pose.x) * sin(pose.theta) +
//                            (carrot.y - pose.y) * cos(pose.theta);

//     // Calculate directional components
//     float direction = atan2(robotRelativeY, robotRelativeX);
//     float forwardComponent = cos(direction);
//     float sidewaysComponent = sin(direction);

//     // Apply outputs to motors
//     for (const auto &config : drivetrain.getMotorConfigs())
//     {
//       float motorOutput = lateralOut * (forwardComponent * config.verticalCoefficient +
//                                         sidewaysComponent * config.horizontalCoefficient) +
//                           angularOut * config.turnCoefficient;

//       motorOutput = std::clamp(motorOutput, -MAX_SPEED, MAX_SPEED);
//       config.motors->move(motorOutput);
//     }

//     // Store previous outputs
//     prevLateralOut = lateralOut;
//     prevAngularOut = angularOut;

//     pros::delay(10);
//   }

//   // Stop all motors
//   for (const auto &config : drivetrain.getMotorConfigs())
//   {
//     config.motors->move(0);
//   }

//   logger().success("Movement complete");
//   this->endMotion();
// }