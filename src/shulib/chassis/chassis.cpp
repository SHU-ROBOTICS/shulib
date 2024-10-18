#include <math.h>
#include "pros/imu.hpp"
#include "shulib/util.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/odometry.hpp"
#include "shulib/chassis/odomUnit.hpp"
#include "pros/rtos.hpp"

shulib::OdomSensors::OdomSensors(OdomUnit *left, OdomUnit *right,
                                 OdomUnit *back, pros::Imu *imu)
    : left(left), right(right), back(back), imu(imu) {}

shulib::Chassis::Chassis(Drivetrain drivetrain, OdomSensors sensors)
    : drivetrain(drivetrain), sensors(sensors) {}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(shulib::OdomSensors &sensors) {
  int attempt = 1;
  bool calibrated = false;
  // calibrate inertial, and if calibration fails, then repeat 5 times or until
  // successful
  while (attempt <= 5) {
    sensors.imu->reset();
    // wait until IMU is calibrated
    do
      pros::delay(10);
    while (sensors.imu->get_status() != pros::ImuStatus::error &&
           sensors.imu->is_calibrating());
    // exit if imu has been calibrated
    if (!isnanf(sensors.imu->get_heading()) &&
        !isinf(sensors.imu->get_heading())) {
      calibrated = true;
      break;
    }
    // indicate error
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
    // shulib::infoSink()->warn("IMU failed to calibrate! Attempt #{}",
    // attempt);
    attempt++;
  }
  // check if calibration attempts were successful
  if (attempt > 5) {
    sensors.imu = nullptr;
    // shulib::infoSink()->error("IMU calibration failed, defaulting to tracking
    // "
    //                           "wheels / motor encoders");
  }
}

void shulib::Chassis::calibrate(bool calibrateImu) {
  // calibrate the IMU if it exists and the user doesn't specify otherwise
  if (sensors.imu != nullptr && calibrateImu)
    calibrateIMU(sensors);
  // initialize odom
  // if sensors are nullptrs, error
  if (sensors.left == nullptr)
    throw std::runtime_error("Left tracking wheel not initialized");
  if (sensors.right == nullptr)
    throw std::runtime_error("Left tracking wheel not initialized");

  sensors.left->reset();
  sensors.right->reset();

  if (sensors.back != nullptr)
    sensors.back->reset();
  setSensors(sensors, drivetrain);
  init();
  // rumble to controller to indicate success
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void shulib::Chassis::setPose(float x, float y, float theta, bool radians) {
  shulib::setPose(shulib::Pose(x, y, theta), radians);
}

void shulib::Chassis::setPose(shulib::Pose pose, bool radians) {
    shulib::setPose(pose, radians);
}

shulib::Pose shulib::Chassis::getPose(bool radians) {
    Pose pose = shulib::getPose(true);
    if (!radians) pose.theta = radToDeg(pose.theta);
    return pose;
}

void shulib::Chassis::drive(int horizontal, int vertical, int turn, bool fieldCentric) {
    drivetrain.drive(horizontal, vertical, turn, fieldCentric);
}

void shulib::Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;
    shulib::setPose(shulib::Pose(0, 0, theta), false);
}

void shulib::Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    drivetrain.setBrakeMode(mode);
}