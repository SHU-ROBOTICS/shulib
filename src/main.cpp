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
    chassis.setPose({-44, -48, 0});
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
    logger().log("Starting drive_forward with target distance: " + std::to_string(distance));
    Pose startPose = chassis.getPose();
    double targetY = startPose.y + distance;
    
    // Calculate target point in 2D space
    double targetX = startPose.x;  // maintain X position
    // Calculate desired heading (angle to target point)
    double dx = targetX - startPose.x;
    double dy = targetY - startPose.y;
    double desiredTheta = std::atan2(dy, dx) * 180.0 / M_PI;  // Convert to degrees
    
    // Normalize to [-180, 180]
    while (desiredTheta > 180) desiredTheta -= 360;
    while (desiredTheta < -180) desiredTheta += 360;

    logger().log("Start pose - X: " + std::to_string(startPose.x) + 
                 " Y: " + std::to_string(startPose.y) + 
                 " Theta: " + std::to_string(startPose.theta));
    logger().log("Target - X: " + std::to_string(targetX) + 
                 " Y: " + std::to_string(targetY) + 
                 " Theta: " + std::to_string(desiredTheta));

    const double MIN_OUTPUT = 30.0;  // Minimum power to move
    const double MAX_OUTPUT = 80.0;  // Maximum power allowed
    const double MAX_ROTATION = 40.0; // Maximum rotation power
    const double MIN_ROTATION = 10.0; // Minimum rotation power
    const double ACCEL_RATE = 2.0;   // How fast to ramp up speed (power units per iteration)
    const double DECEL_ZONE = 6.0;   // Start slowing down when within this distance
    const double ANGLE_TOLERANCE = 5.0; // Degrees of acceptable heading error
    const double SEVERE_ANGLE_ERROR = 20.0; // Degrees of severe heading error
    
    double error = targetY - chassis.getPose().y;
    int stuckCounter = 0;
    double lastError = error;
    double currentMaxSpeed = MIN_OUTPUT;  // Start at minimum speed
    
    // Two-phase control with separate PIDs
    if (fabs(error) > 0.5) {
        // Coarse control phase
        logger().log("Starting coarse control phase (target error < 0.5)");
        PID linearPID(12, 0.01, 0.1);
        // Increased heading gains for better tracking
        PID headingPID(8, 0.02, 0.2);  // More aggressive heading control
        
        while (fabs(error) > 0.5)
        {
            Pose currentPose = chassis.getPose();
            error = targetY - currentPose.y;
            double forwardOutput = linearPID.update(error);

            // Update desired heading based on current position
            dx = targetX - currentPose.x;
            dy = targetY - currentPose.y;
            desiredTheta = std::atan2(dy, dx) * 180.0 / M_PI;
            while (desiredTheta > 180) desiredTheta -= 360;
            while (desiredTheta < -180) desiredTheta += 360;

            // Ramp up speed gradually
            if (currentMaxSpeed < MAX_OUTPUT) {
                currentMaxSpeed += ACCEL_RATE;
                if (currentMaxSpeed > MAX_OUTPUT) currentMaxSpeed = MAX_OUTPUT;
            }

            // Calculate deceleration factor based on distance to target
            double decelFactor = 1.0;
            if (fabs(error) < DECEL_ZONE) {
                decelFactor = fabs(error) / DECEL_ZONE;  // Linear ramp down
                decelFactor = decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed + MIN_OUTPUT / currentMaxSpeed;
            }

            // Apply speed limits and deceleration
            forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
            forwardOutput *= decelFactor;

            double headingError = desiredTheta - currentPose.theta;
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;
            
            // Aggressive speed reduction based on heading error
            double angleErrorFactor = 1.0;
            if (fabs(headingError) > ANGLE_TOLERANCE) {
                // Exponential reduction in speed as angle error increases
                angleErrorFactor = std::exp(-fabs(headingError) / 15.0);
                
                // Almost stop if severely off course
                if (fabs(headingError) > SEVERE_ANGLE_ERROR) {
                    angleErrorFactor = 0.1; // Reduce to 10% speed
                }
            }
            
            forwardOutput *= angleErrorFactor;
            
            // Ensure minimum power to overcome friction
            if (fabs(forwardOutput) < MIN_OUTPUT && fabs(forwardOutput) > 0.1) {
                forwardOutput = (forwardOutput > 0) ? MIN_OUTPUT : -MIN_OUTPUT;
            }
            
            // Scale up heading correction based on drift amount
            double driftFactor = 1.0 + fabs(headingError) / 5.0; // More aggressive correction
            double rotationOutput = headingPID.update(headingError) * driftFactor;
            
            // Limit rotation output
            rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

            // Check if we're stuck
            if (fabs(error - lastError) < 0.001) {
                stuckCounter++;
                if (stuckCounter > 100) {
                    logger().log("WARNING: Possibly stuck - minimal progress detected");
                    logger().log("Current Y: " + std::to_string(currentPose.y) + 
                               " Error: " + std::to_string(error) +
                               " Heading: " + std::to_string(currentPose.theta));
                    forwardOutput *= 1.5;
                }
            } else {
                stuckCounter = 0;
            }
            lastError = error;

            if (stuckCounter % 50 == 0) {
                logger().log("Coarse Phase - Error: " + std::to_string(error) + 
                            " Forward: " + std::to_string(forwardOutput) + 
                            " Speed: " + std::to_string(currentMaxSpeed) +
                            " Heading: " + std::to_string(currentPose.theta) +
                            " Target Heading: " + std::to_string(desiredTheta) +
                            " Angle Factor: " + std::to_string(angleErrorFactor));
            }

            chassis.drive(0, forwardOutput, rotationOutput);
            pros::delay(10);
        }

        logger().log("Coarse control phase complete. Starting fine control phase");
        chassis.drive(0, 0, 0);
        pros::delay(100);
    }

    // Fine control phase
    logger().log("Starting fine control phase (target error < 0.15)");
    PID finePID(8, 0.02, 0.05);
    PID fineHeadingPID(6, 0.01, 0.1);  // Still fairly aggressive heading control
    
    // Reset for fine control
    stuckCounter = 0;
    lastError = error;
    currentMaxSpeed = MIN_OUTPUT;  // Reset speed for fine control
    
    while (fabs(error) > 0.15)
    {
        Pose currentPose = chassis.getPose();
        error = targetY - currentPose.y;
        double forwardOutput = finePID.update(error);

        // Update desired heading
        dx = targetX - currentPose.x;
        dy = targetY - currentPose.y;
        desiredTheta = std::atan2(dy, dx) * 180.0 / M_PI;
        while (desiredTheta > 180) desiredTheta -= 360;
        while (desiredTheta < -180) desiredTheta += 360;

        // In fine control, we keep speed limited
        forwardOutput = std::clamp(forwardOutput, -MIN_OUTPUT * 1.5, MIN_OUTPUT * 1.5);

        double headingError = desiredTheta - currentPose.theta;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        // Aggressive speed reduction in fine control
        double angleErrorFactor = 1.0;
        if (fabs(headingError) > ANGLE_TOLERANCE) {
            angleErrorFactor = std::exp(-fabs(headingError) / 10.0);  // Even more aggressive in fine control
            if (fabs(headingError) > SEVERE_ANGLE_ERROR) {
                angleErrorFactor = 0.1;
            }
        }
        forwardOutput *= angleErrorFactor;
        
        // Scale up heading correction in fine control
        double driftFactor = 1.0 + fabs(headingError) / 3.0; // More aggressive in fine control
        double rotationOutput = fineHeadingPID.update(headingError) * driftFactor;
        
        // Limit rotation output
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION/2, MAX_ROTATION/2);

        if (fabs(error - lastError) < 0.0005) {
            stuckCounter++;
            if (stuckCounter > 100) {
                logger().log("WARNING: Possibly stuck in fine control - minimal progress detected");
                logger().log("Current Y: " + std::to_string(currentPose.y) + 
                           " Error: " + std::to_string(error) +
                           " Heading: " + std::to_string(currentPose.theta));
                forwardOutput *= 1.5;
            }
        } else {
            stuckCounter = 0;
        }
        lastError = error;

        if (stuckCounter % 50 == 0) {
            logger().log("Fine Phase - Error: " + std::to_string(error) + 
                        " Forward: " + std::to_string(forwardOutput) + 
                        " Heading: " + std::to_string(currentPose.theta) +
                        " Target Heading: " + std::to_string(desiredTheta) +
                        " Angle Factor: " + std::to_string(angleErrorFactor));
        }

        chassis.drive(0, forwardOutput, rotationOutput);
        pros::delay(10);
    }

    chassis.drive(0, 0, 0);
    logger().log("Drive forward complete. Final pose - X: " + 
                 std::to_string(chassis.getPose().x) + 
                 " Y: " + std::to_string(chassis.getPose().y) + 
                 " Theta: " + std::to_string(chassis.getPose().theta));
}

void rotate_to(double target_angle)
{
    logger().log("Starting rotation to " + std::to_string(target_angle) + " degrees");
    Pose startPose = chassis.getPose();
    double desiredTheta = target_angle;

    logger().log("Start pose - X: " + std::to_string(startPose.x) + 
                 " Y: " + std::to_string(startPose.y) + 
                 " Theta: " + std::to_string(startPose.theta));

    const double MIN_ROTATION = 10.0;  // Minimum rotation power
    const double MAX_ROTATION = 40.0;  // Maximum rotation power
    const double ACCEL_RATE = 1.0;     // How fast to ramp up rotation speed
    const double DECEL_ANGLE = 45.0;   // Start slowing down when within this angle
    
    double error = target_angle - chassis.getPose().theta;
    // Normalize error to [-180, 180]
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    int stuckCounter = 0;
    double lastError = error;
    double currentMaxSpeed = MIN_ROTATION;  // Start at minimum speed
    
    // Two-phase control with separate PIDs
    if (fabs(error) > 1.0) {
        // Coarse control phase
        logger().log("Starting coarse rotation (target error < 1.0)");
        PID rotationPID(2.0, 0.01, 0.1);  // Conservative gains for rotation
        
        while (fabs(error) > 1.0)
        {
            Pose currentPose = chassis.getPose();
            error = target_angle - currentPose.theta;
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double rotationOutput = rotationPID.update(error);

            // Ramp up speed gradually
            if (currentMaxSpeed < MAX_ROTATION) {
                currentMaxSpeed += ACCEL_RATE;
                if (currentMaxSpeed > MAX_ROTATION) currentMaxSpeed = MAX_ROTATION;
            }

            // Calculate deceleration factor based on angle to target
            double decelFactor = 1.0;
            if (fabs(error) < DECEL_ANGLE) {
                decelFactor = fabs(error) / DECEL_ANGLE;  // Linear ramp down
                // Ensure we don't go below minimum output
                decelFactor = decelFactor * (currentMaxSpeed - MIN_ROTATION) / currentMaxSpeed + MIN_ROTATION / currentMaxSpeed;
            }

            // Apply speed limits and deceleration
            rotationOutput = std::clamp(rotationOutput, -currentMaxSpeed, currentMaxSpeed);
            rotationOutput *= decelFactor;

            // Ensure minimum power to overcome friction
            if (fabs(rotationOutput) < MIN_ROTATION && fabs(rotationOutput) > 0.1) {
                rotationOutput = (rotationOutput > 0) ? MIN_ROTATION : -MIN_ROTATION;
            }

            // Check if we're stuck
            if (fabs(error - lastError) < 0.001) {
                stuckCounter++;
                if (stuckCounter > 100) {
                    logger().log("WARNING: Possibly stuck - minimal progress detected");
                    logger().log("Current Theta: " + std::to_string(currentPose.theta) + 
                               " Error: " + std::to_string(error));
                    rotationOutput *= 1.5;
                }
            } else {
                stuckCounter = 0;
            }
            lastError = error;

            if (stuckCounter % 50 == 0) {
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
    PID fineRotationPID(1.0, 0.02, 0.05);  // More conservative gains for fine control
    
    // Reset for fine control
    stuckCounter = 0;
    lastError = error;
    currentMaxSpeed = MIN_ROTATION;  // Reset speed for fine control
    
    while (fabs(error) > 0.5)
    {
        Pose currentPose = chassis.getPose();
        error = target_angle - currentPose.theta;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double rotationOutput = fineRotationPID.update(error);

        // In fine control, we keep speed limited
        rotationOutput = std::clamp(rotationOutput, -MIN_ROTATION * 1.5, MIN_ROTATION * 1.5);

        // Ensure minimum power
        if (fabs(rotationOutput) < MIN_ROTATION && fabs(rotationOutput) > 0.1) {
            rotationOutput = (rotationOutput > 0) ? MIN_ROTATION : -MIN_ROTATION;
        }

        if (fabs(error - lastError) < 0.0005) {
            stuckCounter++;
            if (stuckCounter > 100) {
                logger().log("WARNING: Possibly stuck in fine control - minimal progress detected");
                logger().log("Current Theta: " + std::to_string(currentPose.theta) + 
                           " Error: " + std::to_string(error));
                rotationOutput *= 1.5;
            }
        } else {
            stuckCounter = 0;
        }
        lastError = error;

        if (stuckCounter % 50 == 0) {
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

void move_to_pose(Pose target_pose)
{
    logger().log("Starting move to pose - Target X: " + std::to_string(target_pose.x) + 
                 " Y: " + std::to_string(target_pose.y) + 
                 " Theta: " + std::to_string(target_pose.theta));

    Pose currentPose = chassis.getPose();
    
    // Calculate distance and angle to target position
    double dx = target_pose.x - currentPose.x;
    double dy = target_pose.y - currentPose.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    double angle_to_target = std::atan2(dy, dx) * 180.0 / M_PI;
    
    // Normalize angle to [-180, 180]
    while (angle_to_target > 180) angle_to_target -= 360;
    while (angle_to_target < -180) angle_to_target += 360;
    
    logger().log("Distance to target: " + std::to_string(distance) + 
                 " Angle to target: " + std::to_string(angle_to_target));
    
    // If we're already very close to the target position
    if (distance < 0.5) {
        // Just rotate to final heading
        rotate_to(target_pose.theta);
        return;
    }
    
    // First, rotate to face the target point
    rotate_to(angle_to_target);
    
    // Movement constants
    const double MIN_OUTPUT = 30.0;
    const double MAX_OUTPUT = 80.0;
    const double MAX_ROTATION = 40.0;
    const double MIN_ROTATION = 10.0;
    const double ACCEL_RATE = 2.0;
    const double DECEL_ZONE = 6.0;
    const double ANGLE_TOLERANCE = 5.0;
    const double SEVERE_ANGLE_ERROR = 20.0;
    
    // Initialize control variables
    int stuckCounter = 0;
    double currentMaxSpeed = MIN_OUTPUT;
    double lastError = distance;
    
    // Coarse movement phase
    logger().log("Starting coarse movement (target error < 0.5)");
    PID linearPID(12, 0.01, 0.1);
    PID headingPID(8, 0.02, 0.2);
    
    while (distance > 0.5)
    {
        currentPose = chassis.getPose();
        dx = target_pose.x - currentPose.x;
        dy = target_pose.y - currentPose.y;
        distance = std::sqrt(dx*dx + dy*dy);
        
        // Recalculate desired heading to target
        angle_to_target = std::atan2(dy, dx) * 180.0 / M_PI;
        while (angle_to_target > 180) angle_to_target -= 360;
        while (angle_to_target < -180) angle_to_target += 360;
        
        double forwardOutput = linearPID.update(distance);
        
        // Ramp up speed gradually
        if (currentMaxSpeed < MAX_OUTPUT) {
            currentMaxSpeed += ACCEL_RATE;
            if (currentMaxSpeed > MAX_OUTPUT) currentMaxSpeed = MAX_OUTPUT;
        }
        
        // Calculate deceleration factor
        double decelFactor = 1.0;
        if (distance < DECEL_ZONE) {
            decelFactor = distance / DECEL_ZONE;
            decelFactor = decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed + MIN_OUTPUT / currentMaxSpeed;
        }
        
        // Apply speed limits and deceleration
        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
        forwardOutput *= decelFactor;
        
        // Calculate heading error and correction
        double headingError = angle_to_target - currentPose.theta;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        // Aggressive speed reduction based on heading error
        double angleErrorFactor = 1.0;
        if (fabs(headingError) > ANGLE_TOLERANCE) {
            angleErrorFactor = std::exp(-fabs(headingError) / 15.0);
            if (fabs(headingError) > SEVERE_ANGLE_ERROR) {
                angleErrorFactor = 0.1;
            }
        }
        
        forwardOutput *= angleErrorFactor;
        
        // Ensure minimum power
        if (fabs(forwardOutput) < MIN_OUTPUT && fabs(forwardOutput) > 0.1) {
            forwardOutput = (forwardOutput > 0) ? MIN_OUTPUT : -MIN_OUTPUT;
        }
        
        // Calculate rotation output
        double driftFactor = 1.0 + fabs(headingError) / 5.0;
        double rotationOutput = headingPID.update(headingError) * driftFactor;
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);
        
        // Stuck detection
        if (fabs(distance - lastError) < 0.001) {
            stuckCounter++;
            if (stuckCounter > 100) {
                logger().log("WARNING: Possibly stuck - minimal progress detected");
                logger().log("Current pose - X: " + std::to_string(currentPose.x) + 
                           " Y: " + std::to_string(currentPose.y) +
                           " Distance: " + std::to_string(distance) +
                           " Heading: " + std::to_string(currentPose.theta));
                forwardOutput *= 1.5;
            }
        } else {
            stuckCounter = 0;
        }
        lastError = distance;
        
        // Logging
        if (stuckCounter % 50 == 0) {
            logger().log("Movement - Distance: " + std::to_string(distance) + 
                        " Forward: " + std::to_string(forwardOutput) + 
                        " Speed: " + std::to_string(currentMaxSpeed) +
                        " Heading: " + std::to_string(currentPose.theta) +
                        " Target Heading: " + std::to_string(angle_to_target) +
                        " Angle Factor: " + std::to_string(angleErrorFactor));
        }
        
        chassis.drive(0, forwardOutput, rotationOutput);
        pros::delay(10);
    }
    
    chassis.drive(0, 0, 0);
    pros::delay(100);
    
    // Finally, rotate to the target heading
    rotate_to(target_pose.theta);
    
    // Log final position and errors
    currentPose = chassis.getPose();
    dx = target_pose.x - currentPose.x;
    dy = target_pose.y - currentPose.y;
    double final_distance_error = std::sqrt(dx*dx + dy*dy);
    double final_angle_error = target_pose.theta - currentPose.theta;
    while (final_angle_error > 180) final_angle_error -= 360;
    while (final_angle_error < -180) final_angle_error += 360;
    
    logger().log("Move to pose complete. Final pose - X: " + std::to_string(currentPose.x) + 
                 " Y: " + std::to_string(currentPose.y) + 
                 " Theta: " + std::to_string(currentPose.theta));
    logger().log("Final errors - Distance: " + std::to_string(final_distance_error) + 
                 " Angle: " + std::to_string(final_angle_error));
}

void autonomous()
{
    logger().log("Starting autonomous");

    // test_min_output();
    // our min output is 30

    chassis.setPose(-44, -48, 0);

    move_to_pose(Pose(-44, 0, 90));  // Move to origin and face 90 degrees
    //move_to_pose(Pose(0, -48, 90));  // Move to origin and face 90 degrees
    // pros::delay(1000);
    // move_to_pose(Pose(-44, -48, 0));  // Return to start
    // pros::delay(1000);
    

    // drive_forward(48);
    // pros::delay(1000);
    // drive_forward(-12);
    // pros::delay(1000);

    // drive_forward(-48);
    // pros::delay(1000);
    
}