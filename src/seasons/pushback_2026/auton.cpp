#include "shulib/seasons/pushback_2026/auton.hpp"
#include "shulib/core/pid.hpp"
#include "shulib/core/logger.hpp"
#include "shulib/core/util.hpp"
#include "pros/rtos.hpp"
#include "config.hpp"
#include <cmath>
#include <algorithm>

namespace shulib::seasons::pushback::auton {

// ─────────────────────────────────────────────────────────────
// Motion Functions
// ─────────────────────────────────────────────────────────────

void rotateTo(Chassis& chassis, double target_angle) {
    Pose startPose = chassis.getPose();
    logger().log("Starting rotation from " + std::to_string(startPose.theta) +
                 " to " + std::to_string(target_angle) + " degrees");

    const double MIN_ROTATION = 25.0;
    const double MAX_ROTATION = 70.0;

    double error = target_angle - chassis.getPose().theta;
    
    // Normalize error to [-180, 180]
    while (error > 181) error -= 360;
    while (error < -181) error += 360;

    int stuckCounter = 0;
    double lastError = error;
    double currentMaxSpeed = MAX_ROTATION;

    if (fabs(error) > 1.0) {
        PID rotationPID(1, 0.4, 0);

        while (fabs(error) > 1.0) {
            Pose currentPose = chassis.getPose();
            error = target_angle - currentPose.theta;
            
            while (error > 181) error -= 360;
            while (error < -181) error += 360;

            double rotationOutput = rotationPID.update(error, 0.005);
            rotationOutput = std::clamp(rotationOutput, -currentMaxSpeed, currentMaxSpeed);

            // Ensure minimum power to overcome friction
            if (fabs(rotationOutput) < MIN_ROTATION && fabs(rotationOutput) > 0.1) {
                rotationOutput = (rotationOutput > 0) ? MIN_ROTATION : -MIN_ROTATION;
            }

            // Check if stuck
            if (fabs(error - lastError) < 0.001) {
                stuckCounter++;
                if (stuckCounter > 100) {
                    logger().log("WARNING: Possibly stuck");
                    rotationOutput *= 1.5;
                }
            } else {
                stuckCounter = 0;
            }
            lastError = error;

            chassis.drive(0, 0, rotationOutput);
            pros::delay(5);
        }

        chassis.drive(0, 0, 0);
        pros::delay(100);
    }

    logger().log("Rotation complete");
}

void moveVertical(Chassis& chassis, double distance_inches, 
                  Mechanisms* mech, bool intaking, bool conveyor) {
    logger().log("Starting vertical move - Distance: " + std::to_string(distance_inches));

    Pose start_pose = chassis.getPose();
    double initial_theta = start_pose.theta;
    double total_distance_traveled = 0;
    double target_distance = std::abs(distance_inches);

    const double MAX_OUTPUT = 60.0;
    const double MAX_ROTATION = 10.0;

    double currentMaxSpeed = MAX_OUTPUT;
    double last_y = start_pose.y;
    double last_x = start_pose.x;

    PID linearPID(10, 2.5, 0.3);
    PID headingPID(0, 0, 0);

    while (total_distance_traveled < target_distance) {
        Pose current_pose = chassis.getPose();

        double dy = std::abs(current_pose.y - last_y);
        last_y = current_pose.y;

        double dx = std::abs(current_pose.x - last_x);
        last_x = current_pose.x;

        total_distance_traveled += sqrt(pow(dx, 2) + pow(dy, 2));
        double remaining_distance = target_distance - total_distance_traveled;

        double heading_error = initial_theta - current_pose.theta;
        while (heading_error > 180) heading_error -= 360;
        while (heading_error < -180) heading_error += 360;

        double forwardOutput = linearPID.update(remaining_distance, 0.005);
        
        if (distance_inches < 0) {
            forwardOutput = -forwardOutput;
        }

        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);

        double rotationOutput = headingPID.update(heading_error, 0.005);
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        chassis.drive(0, forwardOutput, 0);

        // Run mechanisms if provided
        if (mech != nullptr) {
            if (intaking) mech->intakeIn();
            if (conveyor) mech->conveyorUp();
        }

        pros::delay(5);
    }

    chassis.drive(0, 0, 0);

    if (mech != nullptr) {
        if (intaking) mech->intakeStop();
        if (conveyor) mech->conveyorStop();
    }

    logger().log("Vertical move complete");
}

void moveToPose(Chassis& chassis, Pose target_pose,
                Mechanisms* mech, bool reverse, bool intaking, bool conveyor) {
    logger().log("Starting move to pose - Target: " + std::string(target_pose));

    Pose current_pose = chassis.getPose();
    double distance = current_pose.distance(target_pose);
    double angle = -radToDeg(current_pose.angle(target_pose)) - 270;
    angle = std::fmod(angle + 360, 360);

    double angle_error = angle - current_pose.theta;

    if (fabs(angle_error) > 1) {
        rotateTo(chassis, angle);
    }

    const double MIN_OUTPUT = 20.0;
    const double MAX_OUTPUT = 70.0;
    const double MAX_ROTATION = 30.0;
    const double ACCEL_RATE = 6.0;
    const double DECEL_ZONE = 6.0;

    double currentMaxSpeed = MIN_OUTPUT;
    PID linearPID(12, 0.03, 0);
    PID headingPID(10, 0.005, 0.25);

    while (distance > 1) {
        current_pose = chassis.getPose();
        distance = current_pose.distance(target_pose);

        angle = -radToDeg(current_pose.angle(target_pose)) - 270;
        angle = std::fmod(angle + 360, 360);
        angle_error = angle - current_pose.theta;

        double forwardOutput = linearPID.update(distance, 5);

        if (currentMaxSpeed < MAX_OUTPUT) {
            currentMaxSpeed = std::min(currentMaxSpeed + ACCEL_RATE, MAX_OUTPUT);
        }

        double decelFactor = (distance < DECEL_ZONE) ? (distance / DECEL_ZONE) : 1.0;
        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed) * decelFactor;

        double rotationOutput = headingPID.update(angle_error, 0.005);
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        chassis.drive(0, forwardOutput, 0);

        if (mech != nullptr && intaking) {
            mech->intakeIn();
        }

        pros::delay(5);
    }

    chassis.drive(0, 0, 0);
    logger().log("Move to pose complete");
}

void positionReset(Chassis& chassis) {
    pros::delay(100);
    float theta = chassis.getPose().theta;
    chassis.setPose(0, 0, theta);
    pros::delay(100);
}

// ─────────────────────────────────────────────────────────────
// Autonomous Routines
// ─────────────────────────────────────────────────────────────

void run(Chassis& chassis, const RobotConfig& config) {
    Mechanisms mech(config.mechanisms);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    #if defined(AUTON_SKILLS)
        skills(chassis, mech);
    #elif defined(AUTON_RED_LEFT)
        redLeft(chassis, mech);
    #elif defined(AUTON_RED_RIGHT)
        redRight(chassis, mech);
    #elif defined(AUTON_BLUE_LEFT)
        blueLeft(chassis, mech);
    #elif defined(AUTON_BLUE_RIGHT)
        blueRight(chassis, mech);
    #elif defined(AUTON_TEST)
        test(chassis, mech);
    #else
        logger().log("No autonomous selected!");
    #endif
}

void skills(Chassis& chassis, Mechanisms& mech) {
    logger().log("Running Skills Autonomous");
    chassis.setPose(0, 0, 0);
    
    // Simple test: rotate 90 degrees
    rotateTo(chassis, 90);
    pros::delay(500);
    
    // TODO: Add your skills routine here
}

void redLeft(Chassis& chassis, Mechanisms& mech) {
    logger().log("Running Red Left Autonomous");
    chassis.setPose(0, 0, 0);
    
    // TODO: Add your red left routine here
}

void redRight(Chassis& chassis, Mechanisms& mech) {
    logger().log("Running Red Right Autonomous");
    chassis.setPose(0, 0, 0);
    
    // TODO: Add your red right routine here
}

void blueLeft(Chassis& chassis, Mechanisms& mech) {
    logger().log("Running Blue Left Autonomous");
    chassis.setPose(0, 0, 0);
    
    // TODO: Add your blue left routine here
}

void blueRight(Chassis& chassis, Mechanisms& mech) {
    logger().log("Running Blue Right Autonomous");
    chassis.setPose(0, 0, 0);
    
    // TODO: Add your blue right routine here
}

void test(Chassis& chassis, Mechanisms& mech) {
    logger().log("Running Test Autonomous");
    chassis.setPose(0, 0, 0);
    
    // Simple test routine
    rotateTo(chassis, 90);
    pros::delay(500);
    moveVertical(chassis, 12);
    pros::delay(500);
    rotateTo(chassis, 0);
}

}  // namespace shulib::seasons::pushback::auton
