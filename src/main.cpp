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

    shulib::setXCorrectionFactor(1.03225806);
    shulib::setYCorrectionFactor(0.96);
    shulib::setThetaCorrectionFactor(0.922861);

    logger().log("IMU calibrated!");
    logger().log("IMU pitch: " + std::to_string(imu.get_pitch()));
    logger().log("IMU yaw: " + std::to_string(imu.get_yaw()));
    logger().log("IMU roll: " + std::to_string(imu.get_roll()));
    chassis.calibrate();

    chassis.setPose(-66, -50, 10);

    imu.reset();

    logger().log("IMU not calibrated, calibrating...");
    while (imu.is_calibrating())
    {
        pros::delay(100);
    }
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

void rotate_to(double target_angle)
{
    logger().log("Starting rotation to " + std::to_string(target_angle) + " degrees");
    Pose startPose = chassis.getPose();
    double desiredTheta = target_angle;

    logger().log("Start pose - X: " + std::to_string(startPose.x) +
                 " Y: " + std::to_string(startPose.y) +
                 " Theta: " + std::to_string(startPose.theta));

    const double MIN_ROTATION = 10.0; // Minimum rotation power
    const double MAX_ROTATION = 40.0; // Maximum rotation power
    const double ACCEL_RATE = 1.0;    // How fast to ramp up rotation speed
    const double DECEL_ANGLE = 45.0;  // Start slowing down when within this angle

    double error = target_angle - chassis.getPose().theta;
    // Normalize error to [-180, 180]
    while (error > 180)
        error -= 360;
    while (error < -180)
        error += 360;

    int stuckCounter = 0;
    double lastError = error;
    double currentMaxSpeed = MIN_ROTATION; // Start at minimum speed

    // Two-phase control with separate PIDs
    if (fabs(error) > 5)
    {
        // Coarse control phase
        logger().log("Starting coarse rotation (target error < 1.0)");
        PID rotationPID(2.0, 0.01, 0.1); // Conservative gains for rotation

        while (fabs(error) > 1.0)
        {
            Pose currentPose = chassis.getPose();
            error = target_angle - currentPose.theta;
            while (error > 180)
                error -= 360;
            while (error < -180)
                error += 360;

            double rotationOutput = rotationPID.update(error);

            // Ramp up speed gradually
            if (currentMaxSpeed < MAX_ROTATION)
            {
                currentMaxSpeed += ACCEL_RATE;
                if (currentMaxSpeed > MAX_ROTATION)
                    currentMaxSpeed = MAX_ROTATION;
            }

            // Calculate deceleration factor based on angle to target
            double decelFactor = 1.0;
            if (fabs(error) < DECEL_ANGLE)
            {
                decelFactor = fabs(error) / DECEL_ANGLE; // Linear ramp down
                // Ensure we don't go below minimum output
                decelFactor = decelFactor * (currentMaxSpeed - MIN_ROTATION) / currentMaxSpeed + MIN_ROTATION / currentMaxSpeed;
            }

            // Apply speed limits and deceleration
            rotationOutput = std::clamp(rotationOutput, -currentMaxSpeed, currentMaxSpeed);
            rotationOutput *= decelFactor;

            // Ensure minimum power to overcome friction
            if (fabs(rotationOutput) < MIN_ROTATION && fabs(rotationOutput) > 0.1)
            {
                rotationOutput = (rotationOutput > 0) ? MIN_ROTATION : -MIN_ROTATION;
            }

            // Check if we're stuck
            if (fabs(error - lastError) < 0.001)
            {
                stuckCounter++;
                if (stuckCounter > 100)
                {
                    logger().log("WARNING: Possibly stuck - minimal progress detected");
                    logger().log("Current Theta: " + std::to_string(currentPose.theta) +
                                 " Error: " + std::to_string(error));
                    rotationOutput *= 1.5;
                }
            }
            else
            {
                stuckCounter = 0;
            }
            lastError = error;

            if (stuckCounter % 50 == 0)
            {
                logger().log("Coarse Phase - Error: " + std::to_string(error) +
                             " Output: " + std::to_string(rotationOutput) +
                             " Speed: " + std::to_string(currentMaxSpeed));
            }

            chassis.drive(0, 0, rotationOutput);
            pros::delay(10);
        }

        logger().log("Coarse rotation complete. Starting fine rotation");
        chassis.drive(0, 0, 0);
        pros::delay(100);
    }

    // Fine control phase
    logger().log("Starting fine rotation (target error < 0.5)");
    PID fineRotationPID(1.0, 0.02, 0.05); // More conservative gains for fine control

    // Reset for fine control
    stuckCounter = 0;
    lastError = error;
    currentMaxSpeed = MIN_ROTATION; // Reset speed for fine control

    while (fabs(error) > 1)
    {
        Pose currentPose = chassis.getPose();
        error = target_angle - currentPose.theta;
        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;

        double rotationOutput = fineRotationPID.update(error);

        // In fine control, we keep speed limited
        rotationOutput = std::clamp(rotationOutput, -MIN_ROTATION * 1.5, MIN_ROTATION * 1.5);

        // Ensure minimum power
        if (fabs(rotationOutput) < MIN_ROTATION && fabs(rotationOutput) > 0.1)
        {
            rotationOutput = (rotationOutput > 0) ? MIN_ROTATION : -MIN_ROTATION;
        }

        if (fabs(error - lastError) < 0.0005)
        {
            stuckCounter++;
            if (stuckCounter > 100)
            {
                logger().log("WARNING: Possibly stuck in fine control - minimal progress detected");
                logger().log("Current Theta: " + std::to_string(currentPose.theta) +
                             " Error: " + std::to_string(error));
                rotationOutput *= 1.5;
            }
        }
        else
        {
            stuckCounter = 0;
        }
        lastError = error;

        if (stuckCounter % 50 == 0)
        {
            logger().log("Fine Phase - Error: " + std::to_string(error) +
                         " Output: " + std::to_string(rotationOutput));
        }

        chassis.drive(0, 0, rotationOutput);
        pros::delay(10);
    }

    chassis.drive(0, 0, 0);
    logger().log("Rotation complete. Final pose - X: " +
                 std::to_string(chassis.getPose().x) +
                 " Y: " + std::to_string(chassis.getPose().y) +
                 " Theta: " + std::to_string(chassis.getPose().theta));
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
        pros::delay(2000); // Longer settle time

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

void test_min_output()
{
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    int power = 0;
    double start_x = chassis.getPose().x;
    double start_y = chassis.getPose().y;
    double start_theta = chassis.getPose().theta;
    while (true)

    {
        power += 1;
        chassis.drive(power, 0, 0);
        pros::delay(500);

        if (chassis.getPose().x > start_x + .5)
        {
            logger().log("Min output X: " + std::to_string(power));
            chassis.drive(0, 0, 0);
            break;
        }
    }
    power = 0;
    while (true)
    {
        power += 1;
        chassis.drive(0, power, 0);
        pros::delay(500);

        if (chassis.getPose().y > start_y + .5)
        {
            logger().log("Min output Y: " + std::to_string(power));
            chassis.drive(0, 0, 0);
            break;
        }
    }
    power = 0;
    while (true)
    {
        power += 1;
        chassis.drive(0, 0, power);
        pros::delay(500);

        if (chassis.getPose().theta > start_theta + .5)
        {
            logger().log("Min output theta: " + std::to_string(power));
            chassis.drive(0, 0, 0);
            break;
        }
    }
}

void move_to_pose(Pose target_pose, double speedFactor = 1.0)
{
    logger().log("Starting move to pose - Target X: " + std::to_string(target_pose.x) +

                 " Y: " + std::to_string(target_pose.y) +
                 " Theta: " + std::to_string(target_pose.theta));
    logger().updateTelemetry("target", target_pose);

    Pose current_pose = chassis.getPose();
    double distance = current_pose.distance(target_pose);
    double angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
    while (angle > 360)
        angle -= 360;
    while (angle < 0)
        angle += 360;

    logger().log("Angle to target: " + std::to_string(angle));
    double angle_error = angle - current_pose.theta;
    logger().log("Angle error: " + std::to_string(angle_error));

    if (abs(angle_error) > 1)
    {
        logger().log("Rotating to angle: " + std::to_string(angle));
        rotate_to(angle);
    }
    pros::delay(500);
    logger().log("Moving forward");

    const double MIN_OUTPUT = 30.0 * speedFactor;
    const double MAX_OUTPUT = 40.0 * speedFactor;
    const double MAX_ROTATION = 25.0 * speedFactor;
    const double MIN_ROTATION = 10.0 * speedFactor;
    const double ACCEL_RATE = 2.0 * speedFactor;
    const double DECEL_ZONE = 6.0;

    double currentMaxSpeed = MIN_OUTPUT;

    PID linearPID(12, 0.01, 0);
    PID headingPID(3, 0, 0);

    int log_counter = 0;
    while (distance > 1)
    {
        current_pose = chassis.getPose();

        distance = current_pose.distance(target_pose);

        if (distance < 3)
        {
            angle = target_pose.theta;
        }
        else
        {
            angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
        }
        while (angle > 360)
            angle -= 360;
        while (angle < 0)
            angle += 360;
        angle_error = angle - current_pose.theta;
        while (angle_error > 180)
            angle_error -= 360;
        while (angle_error < -180)
            angle_error += 360;

        double forwardOutput = linearPID.update(distance);

        if (currentMaxSpeed < MAX_OUTPUT)
        {
            currentMaxSpeed += ACCEL_RATE;
            if (currentMaxSpeed > MAX_OUTPUT)
                currentMaxSpeed = MAX_OUTPUT;
        }

        double decelFactor = 1.0;
        if (distance < DECEL_ZONE)
        {
            decelFactor = distance / DECEL_ZONE;
            decelFactor = decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed + MIN_OUTPUT / currentMaxSpeed;
        }
        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
        forwardOutput *= decelFactor;

        double angleScaling = std::exp(-fabs(angle_error) / 30.0);
        angleScaling = std::clamp(angleScaling, 0.1, 1.0);
        forwardOutput *= angleScaling;

        double rotationOutput = headingPID.update(angle_error);
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        chassis.drive(0, forwardOutput, rotationOutput);

        log_counter++;
        if (log_counter % 25 == 0)
        {
            logger().log("error_rotation: " + std::to_string(angle_error) + " error_distance: " + std::to_string(distance));
            logger().log("rotation_output: " + std::to_string(rotationOutput) + " forward_output: " + std::to_string(forwardOutput));
        }
        pros::delay(25);
    }
    chassis.drive(0, 0, 0);
    rotate_to(target_pose.theta);
    logger().log("Move to pose complete");
}

void move_vertical(double distance_inches)
{
    logger().log("Starting vertical move - Distance: " + std::to_string(distance_inches) + " inches");

    Pose start_pose = chassis.getPose();
    double initial_theta = start_pose.theta;
    double total_distance_traveled = 0;
    double target_distance = std::abs(distance_inches);

    const double MIN_OUTPUT = 35.0;
    const double MAX_OUTPUT = 50.0;
    const double MAX_ROTATION = 25.0;
    const double ACCEL_RATE = 2.0;
    const double DECEL_ZONE = 2.0;

    double currentMaxSpeed = MIN_OUTPUT;
    double last_y = start_pose.y;

    PID linearPID(12, 0, 0);
    PID headingPID(8, 0.02, 0);

    int log_counter = 0;
    while (total_distance_traveled - .25 < target_distance)
    {
        Pose current_pose = chassis.getPose();

        // Calculate incremental distance traveled
        double dy = std::abs(current_pose.y - last_y);
        total_distance_traveled += dy;
        last_y = current_pose.y;

        double remaining_distance = target_distance - total_distance_traveled;

        // Calculate heading error relative to initial rotation
        double heading_error = initial_theta - current_pose.theta;
        while (heading_error > 180)
            heading_error -= 360;
        while (heading_error < -180)
            heading_error += 360;

        double forwardOutput = linearPID.update(remaining_distance);
        // Invert output if moving backwards
        if (distance_inches < 0)
            forwardOutput = -forwardOutput;

        if (currentMaxSpeed < MAX_OUTPUT)
        {
            currentMaxSpeed += ACCEL_RATE;
            if (currentMaxSpeed > MAX_OUTPUT)
                currentMaxSpeed = MAX_OUTPUT;
        }

        double decelFactor = 1.0;
        if (remaining_distance < DECEL_ZONE)
        {
            decelFactor = remaining_distance / DECEL_ZONE;
            decelFactor = decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed + MIN_OUTPUT / currentMaxSpeed;
        }
        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
        forwardOutput *= decelFactor;

        double rotationOutput = headingPID.update(heading_error);
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        chassis.drive(0, forwardOutput, rotationOutput);

        log_counter++;
        if (log_counter % 25 == 0)
        {
            logger().log("error_heading: " + std::to_string(heading_error) +
                         " distance_traveled: " + std::to_string(total_distance_traveled) +
                         " remaining: " + std::to_string(remaining_distance));
            logger().log("rotation_output: " + std::to_string(rotationOutput) +
                         " forward_output: " + std::to_string(forwardOutput));
        }
        pros::delay(25);
    }

    chassis.drive(0, 0, 0);
    logger().log("Vertical move complete - Total distance traveled: " + std::to_string(total_distance_traveled));
}

void face_point(double x, double y)
{
    logger().log("Starting face_point to coordinates: (" + std::to_string(x) + ", " + std::to_string(y) + ")");

    Pose current_pose = chassis.getPose();
    Pose target_pose(x, y, 0);

    double angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
    while (angle > 360)
        angle -= 360;
    while (angle < 0)
        angle += 360;

    logger().log("Target angle: " + std::to_string(angle));
    rotate_to(angle);
}

void autonomous()
{
    logger().log("Starting autonomous");

    // approach midline
    move_to_pose(Pose(-60, -14, 5));
    pros::delay(500);

    // deploy grabbers
    grabber.set_value(true);
    pros::delay(500);

    // pull back stake and ring
    move_vertical(-12);
    pros::delay(1000);

    // retract grabbers
    grabber.set_value(false);
    pros::delay(1000);

    // rotate to grab stake
    rotate_to(180);
    pros::delay(500);

    chassis.flip();

    face_point(-48, -6);
    pros::delay(500);

    // move to grab stake
    move_vertical(-12);
    pros::delay(1000);

    doinker.set_value(true);
    pros::delay(1000);
}