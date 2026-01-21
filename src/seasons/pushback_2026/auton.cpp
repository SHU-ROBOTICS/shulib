#include "shulib/seasons/pushback_2026/auton.hpp"
#include "shulib/core/pid.hpp"
#include "shulib/core/logger.hpp"
#include "shulib/core/util.hpp"
#include "config.hpp"
#include "pros/rtos.hpp"
#include "pros/motors.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <vector>
#include <string>

namespace shulib::seasons::pushback::auton {

// ─────────────────────────────────────────────────────────────
// Motor Configuration
// Due to inconsistent physical mounting, different operations
// require different port sign configurations:
//
// FORWARD: Left {-12,-14,-16,-18,-20}, Right {11,13,15,17,19}
// TURNING: Left {-12, 14, 16, 18, 20}, Right {11,13,15,17,19}
// ─────────────────────────────────────────────────────────────

// Motor groups for FORWARD motion
pros::MotorGroup leftForward({-12, -14, -16, -18, -20});
pros::MotorGroup rightForward({11, 13, 15, 17, 19});

// Motor groups for TURNING (only left side changes)
pros::MotorGroup leftTurn({-12, 14, 16, 18, 20});
pros::MotorGroup rightTurn({11, 13, 15, 17, 19});

// Helper: Drive forward/backward using FORWARD config
void driveForward(int power) {
    leftForward.move(power);
    rightForward.move(power);
}

// Helper: Stop forward motors
void stopForward() {
    leftForward.move(0);
    rightForward.move(0);
}

// Helper: Turn using TURN config (positive = turn right/clockwise)
void driveTurn(int power) {
    leftTurn.move(power);
    rightTurn.move(-power);
}

// Helper: Stop turn motors
void stopTurn() {
    leftTurn.move(0);
    rightTurn.move(0);
}

// ─────────────────────────────────────────────────────────────
// Motion Functions
// ─────────────────────────────────────────────────────────────

void rotateTo(Chassis& chassis, double target_angle) {
    printf("=== ROTATE TO %.1f deg ===\n", target_angle);
    fflush(stdout);
    
    const double MIN_ROTATION = 30.0;
    const double MAX_ROTATION = 60.0;
    const double TOLERANCE = 3.0;  // degrees

    PID rotationPID(1.2, 0.1, 0.05);

    int iterations = 0;
    const int MAX_ITERATIONS = 600;  // 3 sec timeout
    
    double error;
    do {
        // Get current angle in degrees
        double currentTheta = radToDeg(chassis.getPose().theta);
        error = target_angle - currentTheta;
        
        // Normalize to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double output = rotationPID.update(error, 0.005);
        output = std::clamp(output, -MAX_ROTATION, MAX_ROTATION);

        // Apply minimum power if error is significant
        if (fabs(output) < MIN_ROTATION && fabs(error) > 1.0) {
            output = (error > 0) ? MIN_ROTATION : -MIN_ROTATION;
        }

        // Use TURN motor configuration
        driveTurn(output);
        
        pros::delay(5);
        iterations++;
        
        if (iterations % 100 == 0) {
            printf("  Rotating: current=%.1f target=%.1f error=%.1f\n", 
                   currentTheta, target_angle, error);
            fflush(stdout);
        }
        
    } while (fabs(error) > TOLERANCE && iterations < MAX_ITERATIONS);

    stopTurn();
    
    double finalTheta = radToDeg(chassis.getPose().theta);
    printf("=== ROTATE COMPLETE: %.1f deg (error=%.1f) ===\n", finalTheta, error);
    fflush(stdout);
}

void moveVertical(Chassis& chassis, double distance_inches, 
                  Mechanisms* mech, bool intaking, bool conveyor) {
    printf("=== MOVE VERTICAL %.1f inches ===\n", distance_inches);
    fflush(stdout);

    chassis.setPose(0, 0, 0);  // Reset for distance tracking
    pros::delay(50);

    double target_distance = std::abs(distance_inches);
    int direction = (distance_inches >= 0) ? 1 : -1;
    
    const double MAX_OUTPUT = 60.0;
    const double MIN_OUTPUT = 25.0;

    PID linearPID(8, 1.5, 0.2);
    PID headingPID(2, 0, 0);  // Keep straight

    int iterations = 0;
    const int MAX_ITERATIONS = 1000;  // 5 sec timeout
    
    double totalDistance = 0;
    Pose lastPose = chassis.getPose();
    
    while (totalDistance < target_distance && iterations < MAX_ITERATIONS) {
        Pose currentPose = chassis.getPose();
        
        // Calculate distance traveled since last iteration
        double dx = currentPose.x - lastPose.x;
        double dy = currentPose.y - lastPose.y;
        totalDistance += sqrt(dx*dx + dy*dy);
        lastPose = currentPose;
        
        double remaining = target_distance - totalDistance;
        
        // Linear output
        double linearOutput = linearPID.update(remaining, 0.005);
        linearOutput = std::clamp(linearOutput, -MAX_OUTPUT, MAX_OUTPUT);
        
        if (fabs(linearOutput) < MIN_OUTPUT && remaining > 1.0) {
            linearOutput = (linearOutput >= 0) ? MIN_OUTPUT : -MIN_OUTPUT;
        }
        
        linearOutput *= direction;
        
        // Heading correction (keep straight)
        double headingError = radToDeg(currentPose.theta);
        double headingCorrection = headingPID.update(-headingError, 0.005);
        headingCorrection = std::clamp(headingCorrection, -15.0, 15.0);
        
        // Apply to FORWARD motor configuration with heading correction
        leftForward.move(linearOutput + headingCorrection);
        rightForward.move(linearOutput - headingCorrection);

        if (mech != nullptr) {
            if (intaking) mech->intakeIn();
            if (conveyor) mech->conveyorUp();
        }
        
        pros::delay(5);
        iterations++;
        
        if (iterations % 100 == 0) {
            printf("  Moving: traveled=%.1f remaining=%.1f\n", totalDistance, remaining);
            fflush(stdout);
        }
    }

    stopForward();

    if (mech != nullptr) {
        mech->intakeStop();
        mech->conveyorStop();
    }
    
    Pose finalPose = chassis.getPose();
    printf("=== MOVE COMPLETE: traveled=%.1f, X=%.1f Y=%.1f ===\n", 
           totalDistance, finalPose.x, finalPose.y);
    fflush(stdout);
}

void moveToPose(Chassis& chassis, Pose target_pose,
                Mechanisms* mech, bool reverse, bool intaking, bool conveyor) {
    Pose current = chassis.getPose();
    
    // Calculate angle to target
    double dx = target_pose.x - current.x;
    double dy = target_pose.y - current.y;
    double angleToTarget = radToDeg(atan2(dy, dx));
    
    // Turn to face target
    rotateTo(chassis, angleToTarget);
    pros::delay(100);
    
    // Drive to target
    double distance = sqrt(dx*dx + dy*dy);
    moveVertical(chassis, distance, mech, intaking, conveyor);
    
    // Turn to final heading if specified
    if (target_pose.theta != 0) {
        rotateTo(chassis, target_pose.theta);
    }
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
    printf("\n\n=============================\n");
    printf("AUTONOMOUS STARTING\n");
    printf("Robot: %s\n", config.name.c_str());
    printf("=============================\n\n");
    fflush(stdout);
    
    Mechanisms mech(config.mechanisms);
    
    // Set brake mode on all motors
    leftForward.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rightForward.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    leftTurn.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rightTurn.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

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
        printf("No autonomous selected!\n");
    #endif
    
    printf("\n=============================\n");
    printf("AUTONOMOUS COMPLETE\n");
    printf("=============================\n\n");
    fflush(stdout);
}

void skills(Chassis& chassis, Mechanisms& mech) {
    printf("Running Skills\n");
    fflush(stdout);
    
    chassis.setPose(0, 0, 0);
    
    // Example skills routine
    moveVertical(chassis, 24, nullptr, false, false);
    rotateTo(chassis, 90);
    moveVertical(chassis, 24, nullptr, false, false);
}

void redLeft(Chassis& chassis, Mechanisms& mech) {
    printf("Running Red Left\n");
    chassis.setPose(0, 0, 0);
}

void redRight(Chassis& chassis, Mechanisms& mech) {
    printf("Running Red Right\n");
    chassis.setPose(0, 0, 0);
}

void blueLeft(Chassis& chassis, Mechanisms& mech) {
    printf("Running Blue Left\n");
    chassis.setPose(0, 0, 0);
}

void blueRight(Chassis& chassis, Mechanisms& mech) {
    printf("Running Blue Right\n");
    chassis.setPose(0, 0, 0);
}

void test(Chassis& chassis, Mechanisms& mech) {
    printf("=== DUAL-CONFIG MOTION TEST ===\n");
    printf("Forward uses: L={-12,-14,-16,-18,-20} R={11,13,15,17,19}\n");
    printf("Turning uses: L={-12, 14, 16, 18, 20} R={11,13,15,17,19}\n\n");
    fflush(stdout);
    
    // Test 1: Forward
    printf("TEST 1: Drive forward 24 inches\n");
    fflush(stdout);
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    
    moveVertical(chassis, 24, nullptr, false, false);
    
    Pose p1 = chassis.getPose();
    printf("Result: X=%.1f Y=%.1f Theta=%.1f deg\n\n", p1.x, p1.y, radToDeg(p1.theta));
    fflush(stdout);
    pros::delay(500);
    
    // Test 2: Turn 90
    printf("TEST 2: Turn to 90 degrees\n");
    fflush(stdout);
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    
    rotateTo(chassis, 90);
    
    Pose p2 = chassis.getPose();
    printf("Result: Theta=%.1f deg (expected 90)\n\n", radToDeg(p2.theta));
    fflush(stdout);
    pros::delay(500);
    
    // Test 3: Turn back to 0
    printf("TEST 3: Turn back to 0 degrees\n");
    fflush(stdout);
    
    rotateTo(chassis, 0);
    
    Pose p3 = chassis.getPose();
    printf("Result: Theta=%.1f deg (expected 0)\n\n", radToDeg(p3.theta));
    fflush(stdout);
    pros::delay(500);
    
    // Test 4: Square
    printf("TEST 4: Square pattern\n");
    fflush(stdout);
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    
    for (int i = 0; i < 4; i++) {
        printf("Side %d: forward 12in, turn to %d deg\n", i+1, (i+1)*90);
        fflush(stdout);
        moveVertical(chassis, 12, nullptr, false, false);
        pros::delay(200);
        rotateTo(chassis, (i + 1) * 90);
        pros::delay(200);
    }
    
    Pose pFinal = chassis.getPose();
    printf("\nFinal: X=%.1f Y=%.1f Theta=%.1f deg\n", 
           pFinal.x, pFinal.y, radToDeg(pFinal.theta));
    printf("(Should be near 0,0 if square was accurate)\n\n");
    fflush(stdout);
    
    printf("=== TEST COMPLETE ===\n");
    fflush(stdout);
}

}  // namespace shulib::seasons::pushback::auton