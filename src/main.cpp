#include "main.h"
#include "config.hpp"
#include "shulib/core/chassis.hpp"
#include "shulib/core/drivetrain/tankdrive.hpp"
#include "shulib/core/odometry.hpp"
#include "shulib/core/logger.hpp"
#include "shulib/seasons/pushback_2026/auton.hpp"
#include "shulib/seasons/pushback_2026/opcontrol.hpp"
#include "pros/rotation.hpp"

// ─────────────────────────────────────────────────────────────
// Robot Selection (from config.hpp)
// ─────────────────────────────────────────────────────────────

#if defined(ROBOT_XEBEC)
    #include "shulib/robots/xebec.hpp"
    const auto& ROBOT = shulib::robots::XEBEC;
#elif defined(ROBOT_QUEENS_REVENGE)
    #include "shulib/robots/queens_revenge.hpp"
    const auto& ROBOT = shulib::robots::QUEENS_REVENGE;
#else
    #error "No robot selected! Uncomment a robot in config.hpp"
#endif

// ─────────────────────────────────────────────────────────────
// Global Objects (built from config)
// ─────────────────────────────────────────────────────────────

// Drivetrain motors
pros::MotorGroup leftMotors(ROBOT.drivetrain.left_ports);
pros::MotorGroup rightMotors(ROBOT.drivetrain.right_ports);

// Tracking sensors
pros::Rotation leftRotation(ROBOT.tracking.left_port);
pros::Rotation rightRotation(ROBOT.tracking.right_port);
pros::Rotation backRotation(ROBOT.tracking.back_port);

// Odometry units
shulib::OdomUnit leftOdom(&leftRotation, 
                          ROBOT.tracking.wheel_diameter, 
                          ROBOT.tracking.left_offset);
shulib::OdomUnit rightOdom(&rightRotation, 
                           ROBOT.tracking.wheel_diameter, 
                           ROBOT.tracking.right_offset);
shulib::OdomUnit backOdom(&backRotation, 
                          ROBOT.tracking.wheel_diameter, 
                          ROBOT.tracking.back_offset);

// Drivetrain
shulib::TankDrive drivetrain(leftMotors, rightMotors,
                              ROBOT.drivetrain.track_width,
                              ROBOT.drivetrain.wheel_diameter,
                              ROBOT.drivetrain.rpm);

// Sensors
shulib::OdomSensors sensors(&leftOdom, &rightOdom, &backOdom, nullptr);

// Chassis
shulib::Chassis chassis(drivetrain, sensors);

// ─────────────────────────────────────────────────────────────
// Initialize
// ─────────────────────────────────────────────────────────────

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, ROBOT.name.c_str());

    logger().init();
    logger().log("Initializing " + ROBOT.name);

    shulib::setXCorrectionFactor(1);
    shulib::setYCorrectionFactor(1);
    shulib::setThetaCorrectionFactor(1);

    chassis.calibrate(false);  // false = no IMU
    chassis.setPose(0, 0, 0);

    logger().log("Ready!");
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

    #ifdef ODOM_DISPLAY
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "%s", ROBOT.name.c_str());
            pros::lcd::print(1, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(2, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(3, "Theta: %.2f", chassis.getPose().theta);
            pros::delay(50);
        }
    });
    #endif
}

// ─────────────────────────────────────────────────────────────
// Competition Functions
// ─────────────────────────────────────────────────────────────

void disabled() {}

void competition_initialize() {}

void autonomous() {
    shulib::seasons::pushback::auton::run(chassis, ROBOT);
}

void opcontrol() {
    shulib::seasons::pushback::opcontrol::run(chassis, ROBOT);
}
