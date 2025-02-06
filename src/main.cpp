#include "main.h"
#include "pros/adi.hpp"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/xdrive.hpp"
#include "shulib/logger.hpp"
#include "shulib/chassis/odometry.hpp"
#include "shulib/pid.hpp"
// #include "shulib/GUI/gui.c"

Controller master(CONTROLLER_MASTER);


MotorGroup frontLeft({-18, -19});
MotorGroup frontRight({12, 13});
MotorGroup backLeft({-16, -17});
MotorGroup backRight({15, 14});

// IMU imu(10);
pros::Rotation left(-20);
pros::Rotation right(-11);
pros::Rotation back(7);

// set these to nullptrs instead

shulib::OdomUnit leftOdom(&left, 2.75, -5.875);
shulib::OdomUnit rightOdom(&right, 2.75, 5.875);
shulib::OdomUnit backOdom(&back, 2.75, 4);

shulib::XDrive drivetrain(frontLeft, frontRight, backLeft, backRight, 2.25, 200,
                          2);

shulib::OdomSensors sensors(&leftOdom,  // left odom unit
                            &rightOdom, // right odom unit
                            &backOdom,  // horizontal odom unit
                            nullptr     // inertial sensor
);

shulib::Chassis chassis(drivetrain, sensors);

pros::adi::Pneumatics doinker('H', false);
pros::adi::Pneumatics grabber('G', false);
pros::Motor intake(2);
pros::Motor lift(9);
pros::MotorGroup conveyor({1, -10});
pros::MotorGroup wallStake({3, -8}, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);

pros::IMU imu(4);


void initialize()
{
    lcd::initialize();
    lcd::set_text(0, "Hello, PROS User!");

    logger().init();
    imu.reset();



    logger().log("IMU not calibrated, calibrating...");
    while (imu.is_calibrating())
    {
        pros::delay(100);
    }

    shulib::setXCorrectionFactor(1);
    shulib::setYCorrectionFactor(1.03225806);
    shulib::setThetaCorrectionFactor(0.922861);
    


    logger().log("IMU calibrated!");
    logger().log("IMU pitch: " + std::to_string(imu.get_pitch()));
    logger().log("IMU yaw: " + std::to_string(imu.get_yaw()));
    logger().log("IMU roll: " + std::to_string(imu.get_roll()));

    chassis.calibrate();
    chassis.setPose({36, -60, 0});
}

void disabled() {}
void competition_initialize() {}

double conveyorSpeed = .8;
double intakeSpeed = 1;

void fifteen()
{
    // right: pneumatics
    if (master.get_digital_new_press(DIGITAL_RIGHT))
    {
        grabber.toggle();
    }
    // y : pneumatics #2
    if (master.get_digital_new_press(DIGITAL_Y))
    {
        doinker.toggle();
    }
    // r1 : wall stake setup
    if (master.get_digital_new_press(DIGITAL_R1))
    {
        if (abs(wallStake.get_position()) < 1)
        {
            wallStake.move_absolute(37, 50);
        }
        else
        {
            wallStake.move_absolute(0, 20);
        }
    }
    // r2 : wall stake lift
    if (master.get_digital_new_press(DIGITAL_R2))
    {
        // check if wall stake is at 37 degrees with a tolerance of 1 degree
        if (abs(wallStake.get_position() - 37) < 2)
        {
            wallStake.move_absolute(140, 30);
        }
        else
        {
            wallStake.move_absolute(37, 30);
        }
    }
    // l2: intake, l1: outtake
    if (master.get_digital(DIGITAL_L2))
    {
        intake.move(127 * intakeSpeed);
        conveyor.move(-127 * conveyorSpeed);
    }
    else if (master.get_digital(DIGITAL_L1))
    {
        intake.move(-127 * intakeSpeed);
        conveyor.move(127 * conveyorSpeed);
    }
    else if (master.get_digital(DIGITAL_UP))
    { // up: conveyor up, down: conveyor down
        conveyor.move(127 * conveyorSpeed);
    }
    else if (master.get_digital(DIGITAL_DOWN))
    {
        conveyor.move(-127 * conveyorSpeed);
    }
    else
    {
        intake.move(0);
        conveyor.move(0);
    }

    shulib::logger().updateTelemetry("test", master.get_digital(DIGITAL_B));

    if (master.get_digital(DIGITAL_X))
    {
        chassis.setPose(0, 0, 0);
        shulib::logger().updateTelemetry("test", true);
    }

    if (master.get_digital(DIGITAL_LEFT))
    {
        printf("chassis pose: %f, %f, %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    }
}

void drive_forward(double distance)
{
    Pose target = chassis.getPose();
    target.y += distance;
    logger().updateTelemetry("target", target);
    float error = target.y - chassis.getPose().y;
    PID pid(1, 0, 0);
    while(abs(error) > .125)
    {
        error = target.y - chassis.getPose().y;
        float output = pid.update(error);

        logger().updateTelemetry("error", error);
        logger().updateTelemetry("output", output);

        chassis.drive(0, output, 0);

        pros::delay(10);
    }
    chassis.drive(0, 0, 0);
    pros::delay(100);
    
    error = target.y - chassis.getPose().y;
    while(abs(error) > .05)
    {
        float error = target.y - chassis.getPose().y;
        float output = pid.update(error);

        logger().updateTelemetry("error", error);
        logger().updateTelemetry("output", output);

        chassis.drive(0, output, 0);

        pros::delay(10);
    }
    chassis.drive(0, 0, 0);
}


void rotation_calibration()
{
    logger().log("Starting autonomous");
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    // Reset IMU and chassis pose
    imu.tare();
    chassis.setPose(0, 0, 0);
    pros::delay(500); // Give time for reset

    double correctionFactor = shulib::getThetaCorrectionFactor();
    const double TARGET_ANGLE = 180.0;
    const double TOLERANCE = 2.0;
    const int MAX_ITERATIONS = 3;
    int iterations = 0;

    while (iterations < MAX_ITERATIONS)
    {
        double start_yaw = imu.get_yaw();
        double start_theta = chassis.getPose().theta;

        logger().log("Starting rotation..");
        logger().log("Start yaw: " + std::to_string(start_yaw));
        logger().log("Start theta: " + std::to_string(start_theta));

        chassis.drive(0, 0, 40);
        pros::delay(500);
        // Rotate clockwise
        while (true)
        {
            double current_yaw = imu.get_yaw();
            // Calculate total rotation considering wraparound
            double total_rotation = current_yaw > start_yaw ? current_yaw - start_yaw : TARGET_ANGLE - (start_yaw - current_yaw);


            logger().updateTelemetry("total_rotation", total_rotation);
            logger().updateTelemetry("imu_yaw", imu.get_yaw());
            logger().updateTelemetry("chassis_theta", chassis.getPose().theta);

            if (total_rotation >= TARGET_ANGLE - TOLERANCE)
                break;

            chassis.drive(0, 0, 24); // Increased power for more consistent rotation
            pros::delay(10);
        }

        chassis.drive(0, 0, 0); // Stop rotation
        logger().log("Rotation finished!");
        logger().log("Settling...");
        pros::delay(2000);      // Longer settle time

        // Calculate rotation amounts
        double chassis_rotation = std::abs(chassis.getPose().theta - start_theta);
        double imu_rotation = std::abs(imu.get_yaw() - start_yaw);

        logger().log("End yaw: " + std::to_string(imu.get_yaw()));
        logger().log("End theta: " + std::to_string(chassis.getPose().theta));
        logger().log("");
        logger().log("Chassis rotation: " + std::to_string(chassis_rotation));
        logger().log("IMU rotation: " + std::to_string(imu_rotation));
        logger().log("");
    
        // Update correction factor
        if (chassis_rotation > 1.0)
        { // Prevent division by very small numbers
            double new_correction = imu_rotation / chassis_rotation;
            correctionFactor *= new_correction;
            shulib::setThetaCorrectionFactor(correctionFactor);
            logger().log("Correction factor: " + std::to_string(correctionFactor));
            logger().log("");

            logger().updateTelemetry("chassis_rotation", chassis_rotation);
            logger().updateTelemetry("imu_rotation", imu_rotation);
            logger().updateTelemetry("iteration_correction", new_correction);
            logger().updateTelemetry("cumulative_correction", correctionFactor);
        }

        // Reset for next iteration
        imu.reset();
        while (imu.is_calibrating())
        {
            pros::delay(10);
        }
        chassis.setPose(0, 0, 0);
        pros::delay(100);
        iterations++;

    }

    logger().updateTelemetry("final_correction", correctionFactor);
    logger().log("Correction factor: " + std::to_string(correctionFactor));
}

void log_imu()
{
    while (true)
    {
        logger().updateTelemetry("imu_yaw", imu.get_yaw());
        logger().updateTelemetry("imu_pitch", imu.get_pitch());
        logger().updateTelemetry("imu_roll", imu.get_roll());
        pros::delay(100);
    }
}

void opcontrol()
{
    pros::Task log_imu_task(log_imu);

    wallStake.move_absolute(0, 10);

    while (true)
    {
        chassis.drive(master.get_analog(ANALOG_LEFT_X),
                      master.get_analog(ANALOG_LEFT_Y),
                      master.get_analog(ANALOG_RIGHT_X));

        fifteen();

        pros::delay(20);
    }
}

void movement_calibration()
{
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 0);
    chassis.drive(127, 0, 360);
}

void autonomous()
{
    drive_forward(12);
    pros::delay(1000);
    drive_forward(48);
    pros::delay(1000);
    drive_forward(-12);
    pros::delay(1000);
    drive_forward(-48);
    pros::delay(1000);
    
}