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

shulib::OdomUnit leftOdom(&left, 2.75, -6.25);
shulib::OdomUnit rightOdom(&right, 2.75, 6.25);
shulib::OdomUnit backOdom(&back, 2.75, -3);
pros::IMU imu(4);

shulib::XDrive drivetrain(frontLeft, frontRight, backLeft, backRight, 2.25, 200,
                          2);

shulib::OdomSensors sensors(&leftOdom,  // left odom unit
                            &rightOdom, // right odom unit
                            &backOdom,  // horizontal odom unit
                            &imu);      // inertial sensor

shulib::Chassis chassis(drivetrain, sensors);

pros::adi::Pneumatics clamp('H', false);
pros::adi::Pneumatics grabber('G', false);
pros::Motor intake(2);

pros::Motor lift(5);
pros::MotorGroup conveyor({1, -10});
pros::MotorGroup wallStake({3, -8}, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);

bool red = false;


// Constants for game configuration
const int SCREEN_WIDTH = 440;
const int SCREEN_HEIGHT = 200;

lv_obj_t *btn;
lv_obj_t *label;

// Event callback for the button
void btn_event_cb(lv_event_t *e)
{
    red = !red;
    lv_label_set_text_fmt(label, red ? "RED" : "BLUE");
    if (red)
    {
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xff0000), 0);
    }
    else
    {
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x0000ff), 0);
    }
}

void initialize()
{
    lv_obj_t *scr = lv_scr_act();

    // Background setup
    lv_obj_t *bg = lv_obj_create(scr);
    lv_obj_set_size(bg, lv_obj_get_width(scr), lv_obj_get_height(scr));
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x000000), 0);

    // Button setup
    btn = lv_btn_create(bg);
    lv_obj_set_size(btn, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0000ff), 0);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);


    label = lv_label_create(btn);
    lv_obj_set_size(label, lv_obj_get_width(btn), lv_obj_get_height(btn));
    lv_obj_set_pos(label, lv_obj_get_width(btn) / 2, lv_obj_get_height(btn) / 2);
    lv_label_set_text(label, "BLUE");

    // lcd::initialize();
    // lcd::set_text(0, "Hello, PROS User!");

    logger().init();

    shulib::setXCorrectionFactor(1);
    shulib::setYCorrectionFactor(1);
    shulib::setThetaCorrectionFactor(1);

    logger().log("IMU calibrated!");
    logger().log("IMU pitch: " + std::to_string(imu.get_pitch()));
    logger().log("IMU yaw: " + std::to_string(imu.get_yaw()));
    logger().log("IMU roll: " + std::to_string(imu.get_roll()));

    chassis.calibrate();
    // chassis.setPose(0, 0, 0);
    chassis.setPose(-61, -56.5, 180);
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
        clamp.toggle();
    }
    // r1 : wall stake setup
    if (master.get_digital_new_press(DIGITAL_R1))
    {
        if (abs(wallStake.get_position()) < 1)
        {
            wallStake.move_absolute(32, 50);
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
        if (abs(wallStake.get_position() - 32) < 2)
        {
            wallStake.move_absolute(140, 30);
        }
        else
        {
            wallStake.move_absolute(32, 30);
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

    if (master.get_digital(DIGITAL_B))
    {
        lift.move(-127);
    }
    else if (master.get_digital(DIGITAL_X))
    {
        lift.move(127);
    }
    else
    {
        lift.move(0);
    }
}

void opcontrol()

{
    // Set the wall stake to the starting position
    wallStake.move_absolute(0, 10);

    while (true)
    {
        // Control Loop

        // tells the chassis to drive based on the master controller joystick positions
        chassis.drive(master.get_analog(ANALOG_LEFT_X),
                      master.get_analog(ANALOG_LEFT_Y),
                      master.get_analog(ANALOG_RIGHT_X),
                      false);

        // runs if statements for other controls
        fifteen();

        pros::delay(20);
    }
}

void rotate_to(double target_angle, int timeout_ms = 0)
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

    double lastError = error;
    double currentMaxSpeed = MIN_ROTATION; // Start at minimum speed

    uint32_t startTime = pros::millis();

    // Coarse control phase
    logger().log("Starting coarse rotation (target error < 1.0)");
    PID rotationPID(2.0, 0.5, 0.002); // Conservative gains for rotation

    while (fabs(error) > 5.0)
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

        lastError = error;

        if (timeout_ms > 0)
        {
            uint32_t elapsed = pros::millis() - startTime;
            if (elapsed >= (uint32_t)timeout_ms)
            {
                logger().log("Timeout (" + std::to_string(timeout_ms) + "ms) reached; exiting movement loop.");
                break;
            }
        }
        chassis.drive(0, 0, rotationOutput);
        pros::delay(20);
    }

    logger().log("Coarse rotation complete. Starting fine rotation");
    chassis.drive(0, 0, 0);
    pros::delay(100);

    // Fine control phase
    logger().log("Starting fine rotation (target error < 0.5)");
    PID fineRotationPID(1.0, 1, 0.001); // More conservative gains for fine control

    // Reset for fine control
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

        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        // Ensure minimum power
        if (fabs(rotationOutput) < MIN_ROTATION && fabs(rotationOutput) > 0.1)
        {
            rotationOutput = (rotationOutput > 0) ? MIN_ROTATION : -MIN_ROTATION;
        }

        if (timeout_ms > 0)
        {
            uint32_t elapsed = pros::millis() - startTime;
            if (elapsed >= (uint32_t)timeout_ms)
            {
                logger().log("Timeout (" + std::to_string(timeout_ms) + "ms) reached; exiting movement loop.");
                break;
            }
        }
        chassis.drive(0, 0, rotationOutput);
        pros::delay(20);
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

        if (chassis.getPose().x > start_x + .25)
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

        if (chassis.getPose().y > start_y + .25)
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

        if (chassis.getPose().theta > start_theta + 1)
        {
            logger().log("Min output theta: " + std::to_string(power));
            chassis.drive(0, 0, 0);
            break;
        }
    }
}

void move_to_pose(Pose target_pose, int timeout_ms = 0, bool reverse = false)
{
    logger().log("Starting move to pose - Target X: " + std::to_string(target_pose.x) +
                 " Y: " + std::to_string(target_pose.y) +
                 " Theta: " + std::to_string(target_pose.theta));
    logger().updateTelemetry("target", target_pose);

    Pose current_pose = chassis.getPose();
    double distance = current_pose.distance(target_pose);
    double angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
    if (reverse)
    {
        angle += 180;
    }
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

    const double MIN_OUTPUT = 30.0;
    const double MAX_OUTPUT = 40.0;
    const double MAX_ROTATION = 25.0;
    const double MIN_ROTATION = 10.0;
    const double ACCEL_RATE = 2.0;
    const double DECEL_ZONE = 6.0;

    double currentMaxSpeed = MIN_OUTPUT;
    uint32_t startTime = pros::millis();

    PID linearPID(12, 0.5, 0);
    PID headingPID(3, 0, 0);

    int log_counter = 0;
    while (distance > 1)
    {
        current_pose = chassis.getPose();

        distance = current_pose.distance(target_pose);
        double forwardOutput = linearPID.update(distance);
        if (reverse)
        {
            forwardOutput = -forwardOutput;
        }

        if (distance < 3)
        {
            // if the robot is within 3 inches of the target, set the angle to the target angle
            // this prevents the robot from driving in a circle when it is close to the target
            angle = target_pose.theta;
        }
        else
        {
            angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
        }
        if (reverse)
        {
            angle += 180;
        }

        // this is to ensure the angle is between 0 and 360 degrees
        while (angle > 360)
            angle -= 360;
        while (angle < 0)
            angle += 360;
        // then we can calculate the error between the target angle and the current angle
        angle_error = angle - current_pose.theta;
        // this is to ensure the angle error is between -180 and 180 degrees
        // we do this so that the robot can always turn the shortest way to the target
        while (angle_error > 180)
            angle_error -= 360;
        while (angle_error < -180)
            angle_error += 360;

        if (currentMaxSpeed < MAX_OUTPUT)
        {
            // accelerate the robot to the max output
            currentMaxSpeed += ACCEL_RATE;
            if (currentMaxSpeed > MAX_OUTPUT)
                currentMaxSpeed = MAX_OUTPUT;
        }

        double decelFactor = 1.0;
        if (distance < DECEL_ZONE)
        {
            // decelerate the robot to the min output when it's in the deceleration zone
            decelFactor = distance / DECEL_ZONE;
            decelFactor = decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed + MIN_OUTPUT / currentMaxSpeed;
        }

        // clamp the forward output to the current max speed
        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
        // apply the deceleration factor to the forward output
        forwardOutput *= decelFactor;

        // scale the angle error to the forward output
        // this is to ensure the robot turns slower when it is further away from the target,
        // preventing it from moving in the wrong direction
        double angleScaling = std::exp(-fabs(angle_error) / 30.0);
        angleScaling = std::clamp(angleScaling, 0.1, 1.0);
        forwardOutput *= angleScaling;

        double rotationOutput = headingPID.update(angle_error);
        if (reverse)
        {
            rotationOutput = -rotationOutput;
        }
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        // if the timeout is reached, exit the loop
        if (timeout_ms > 0)
        {
            uint32_t elapsed = pros::millis() - startTime;
            if (elapsed >= (uint32_t)timeout_ms)
            {
                logger().log("Timeout (" + std::to_string(timeout_ms) + "ms) reached; exiting movement loop.");
                break;
            }
        }

        chassis.drive(0, forwardOutput, rotationOutput);

        log_counter++;
        if (log_counter % 25 == 0)
        {
            logger().log("error_rotation: " + std::to_string(angle_error) + " error_distance: " + std::to_string(distance));
            logger().log("rotation_output: " + std::to_string(rotationOutput) + " forward_output: " + std::to_string(forwardOutput));
        }
        pros::delay(20);
    }
    chassis.drive(0, 0, 0);
    logger().log("Rotating to target theta: " + std::to_string(target_pose.theta));
    double end_theta = target_pose.theta;
    if (reverse)
    {
        end_theta += 180;
    }
    while (end_theta > 360)
        end_theta -= 360;
    while (end_theta < 0)
        end_theta += 360;
    rotate_to(end_theta, timeout_ms - (pros::millis() - startTime));
    logger().log("Move to pose complete");
}

void move_vertical(double distance_inches, int timeout_ms = 0)
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

    // Record the start time (in milliseconds).
    uint32_t startTime = pros::millis();

    PID linearPID(12, 0, 0);
    PID headingPID(8, 1, 0);

    int log_counter = 0;
    while (total_distance_traveled < target_distance - .25)
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

        if (timeout_ms > 0)
        {
            uint32_t elapsed = pros::millis() - startTime;
            if (elapsed >= (uint32_t)timeout_ms)
            {
                logger().log("Timeout (" + std::to_string(timeout_ms) + "ms) reached; exiting movement loop.");
                break;
            }
            // else {
            //     // Estimate the required speed (in inches per second) to finish on time.
            //     double remainingTimeSec = (timeout_ms - elapsed) / 1000.0;
            //     // Avoid division by zero.
            //     if (remainingTimeSec > 0) {
            //         double requiredSpeed = remaining_distance / remainingTimeSec;
            //         // If our current forward output is above the required speed, clamp it.
            //         if (fabs(forwardOutput) > requiredSpeed)
            //             forwardOutput = copysign(requiredSpeed, forwardOutput);
            //     }
            // }
        }

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
        pros::delay(20);
    }

    chassis.drive(0, 0, 0);
    logger().log("Vertical move complete - Total distance traveled: " + std::to_string(total_distance_traveled));
}

void face_point(double x, double y, int timeout_ms = 0)
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
    rotate_to(angle, timeout_ms);
}

void move_to(Pose target_pose, int timeout_ms = 0, double max_x = 40, double max_y = 40, double max_theta = 25)
{
    // Mirror X coordinates and angles for red alliance
    if (red) {
        target_pose.x = -target_pose.x;
        target_pose.theta = 360 - target_pose.theta;
        while (target_pose.theta >= 360) target_pose.theta -= 360;
        while (target_pose.theta < 0) target_pose.theta += 360;
    }

    logger().log("Starting move_to(" + std::to_string(target_pose.x) + ", " + std::to_string(target_pose.y) + ", " + std::to_string(target_pose.theta) + ")");
    logger().updateTelemetry("target", target_pose);
    Pose current_pose = chassis.getPose();
    uint32_t startTime = pros::millis();

    PID xPID(2, 5, 0);
    PID yPID(2, 5, 0);
    PID thetaPID(3, 2, 0);

    const double MIN_X = 18;
    const double MAX_X = max_x;
    const double THRESHOLD_X = .5;
    const double MIN_Y = 15;
    const double MAX_Y = max_y;
    const double THRESHOLD_Y = .5;
    const double MIN_THETA = 5;
    const double MAX_THETA = max_theta;

    const double THRESHOLD_THETA = 1;

    double error_x = target_pose.x - current_pose.x;
    double error_y = target_pose.y - current_pose.y;
    double error_theta = target_pose.theta - current_pose.theta;

    const double DECEL_ZONE = .35;
    const double DECEL_ZONE_X = error_x * DECEL_ZONE;
    const double DECEL_ZONE_Y = error_y * DECEL_ZONE;
    const double DECEL_ZONE_THETA = error_theta * DECEL_ZONE;

    int log_counter = 0;
    while ((std::abs(error_x) > THRESHOLD_X) || (std::abs(error_y) > THRESHOLD_Y) || (std::abs(error_theta) > THRESHOLD_THETA))
    {
        if (timeout_ms > 0)
        {
            uint32_t elapsed = pros::millis() - startTime;
            if (elapsed >= (uint32_t)timeout_ms)
            {
                logger().log("Timeout (" + std::to_string(timeout_ms) + "ms) reached; exiting movement loop.");
                break;
            }
        }
        double horizontal = 0;
        double vertical = 0;
        double turn = 0;

        current_pose = chassis.getPose();

        error_x = target_pose.x - current_pose.x;
        error_y = target_pose.y - current_pose.y;
        error_theta = target_pose.theta - current_pose.theta;

        log_counter++;

        if (std::abs(error_x) > THRESHOLD_X)
        {
            // update the pid and get the output
            horizontal = xPID.update(error_x);

            // decelerate the robot to the min output when it's in the deceleration zone
            double decelFactor = 1.0;
            if (std::abs(error_x) < DECEL_ZONE_X)

            {
                decelFactor = std::abs(error_x) / DECEL_ZONE_X;
                decelFactor = decelFactor * (horizontal - MIN_X) / horizontal + MIN_X / horizontal;
            }

            // Apply speed limits
            horizontal = std::clamp(horizontal, -MAX_X, MAX_X);

            if (log_counter % 5 == 0)
                logger().log("error_x: " + std::to_string(error_x) + " horizontal: " + std::to_string(horizontal));
        }
        if (std::abs(error_y) > THRESHOLD_Y)
        {
            // update the pid and get the output
            vertical = yPID.update(error_y);

            // decelerate the robot to the min output when it's in the deceleration zone
            double decelFactor = 1.0;
            if (std::abs(error_y) < DECEL_ZONE_Y)
            {
                decelFactor = std::abs(error_y) / DECEL_ZONE_Y;
                decelFactor = decelFactor * (vertical - MIN_Y) / vertical + MIN_Y / vertical;
            }

            // Apply speed limits
            vertical = std::clamp(vertical, -MAX_Y, MAX_Y);

            if (log_counter % 5 == 0)
                logger().log("error_y: " + std::to_string(error_y) + " vertical: " + std::to_string(vertical));
        }
        if (std::abs(error_theta) > THRESHOLD_THETA)
        {
            // update the pid and get the output
            turn = thetaPID.update(error_theta);

            // decelerate the robot to the min output when it's in the deceleration zone
            double decelFactor = 1.0;
            if (std::abs(error_theta) < DECEL_ZONE_THETA)
            {
                decelFactor = std::abs(error_theta) / DECEL_ZONE_THETA;
                decelFactor = decelFactor * (turn - MIN_THETA) / turn + MIN_THETA / turn;
            }

            // Apply speed limits
            turn = std::clamp(turn, -MAX_THETA, MAX_THETA);

            if (log_counter % 5 == 0)
                logger().log("error_theta: " + std::to_string(error_theta) + " turn: " + std::to_string(turn));
        }

        chassis.drive(horizontal, vertical, turn, true);
        pros::delay(20);
    }
    logger().success("move_to complete!");
    chassis.drive(0, 0, 0);
}

void skills()
{
    chassis.setPose(48, -60, 0);

    intake.move(127);
    move_to(Pose(48, -48, 0), 2500, 50, 50, 25);
    intake.move(0);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    move_to(Pose(48, -48, 180), 2500, 50, 50, 25);

    move_to(Pose(48, -20, 180), 2500, 50, 50, 25);
    clamp.set_value(true);

    conveyor.move(-80);
    pros::delay(1000);
    conveyor.move(80);
    pros::delay(250);
    conveyor.move(0);

    clamp.set_value(false);
    pros::delay(250);
    clamp.set_value(true);
    pros::delay(500);

    move_to(Pose(48, -24, 0), 2500, 50, 50, 25);

    pros::Task intake_task = pros::Task([]{
        while (true) {
            intake.move(127);
            conveyor.move(-100);
            pros::delay(50);
            if (conveyor.get_actual_velocity() < 1) {
                conveyor.move(100);
                pros::delay(500);
                conveyor.move(0);
                pros::delay(250);
            }
        }
    });

    move_to(Pose(48, 0, 0), 4000);

    pros::delay(500);
    clamp.set_value(false);
    pros::delay(250);
    clamp.set_value(true);
    pros::delay(250);
    
    move_to(Pose(48, 24, 0), 4000);

    pros::delay(500);
    clamp.set_value(false);
    pros::delay(250);
    clamp.set_value(true);
    pros::delay(250);

    move_to(Pose(48, 48, 0), 4000);
    pros::delay(500);
    intake_task.remove();
    intake.move(0);

    conveyor.move(0);

}


// void autonomous()
// {
//     skills();
// }

void autonomous()
{

    // test_min_output();
    // 25 25 10

    red = false;

    if (red)
    {
        logger().log("Starting autonomous red");
        chassis.setPose(61, -56.5, 180);
    }
    else
    {
        logger().log("Starting autonomous blue");
        chassis.setPose(-61, -56.5, 180);
    }

    // inline lift task, move the lift down until it can't anymore
    pros::Task lift_task = pros::Task([]{
        lift.move(-127);
        pros::delay(250);
        while (lift.get_actual_velocity() > 0)
        {
            pros::delay(100);
        }
        lift.move(0);
    });


    move_to(Pose(-60, -12, 180), 5000, 50, 50, 25);
    pros::delay(250);
    move_to(Pose(-48, -12, 180), 3000, 35, 35, 25);
    pros::delay(250);
    move_to(Pose(-48, -6, 180), 3000, 35, 35, 25);
    pros::delay(250);

    clamp.set_value(true);
    pros::delay(250);
    conveyor.move(-127);
    pros::delay(1000);
    conveyor.move(127);
    pros::delay(250);
    conveyor.move(0);

    clamp.set_value(false);
    pros::delay(500);
    clamp.set_value(true);
    pros::delay(500);

    move_to(Pose(-60, -36, 180), 3000);
    pros::delay(250);
    conveyor.move(-127);
    intake.move(127);
    move_to(Pose(-48, -48, 135), 2000);
    intake.move(0);
    conveyor.move(127);
    pros::delay(250);
    conveyor.move(0);


    move_to(Pose(-48, -48, 90), 3000);
    pros::delay(250);
    move_to(Pose(-55, -60, 45), 3000);

    clamp.set_value(false);
    pros::delay(500);
    
    move_to(Pose(-72, -72, 45), 2000, 75, 75, 25);
    pros::delay(250);


    move_to(Pose(-10, -10, 45), 10000, 75, 75, 25);
    conveyor.move(-127);

    // if conveyor gets stuck, skip
    while (conveyor.get_actual_velocity() < 10)
        pros::delay(100);
    conveyor.move(0);
}