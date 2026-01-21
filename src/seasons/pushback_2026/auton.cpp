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

namespace shulib::seasons::pushback::auton {

// ─────────────────────────────────────────────────────────────
// Motion Functions
// ─────────────────────────────────────────────────────────────

void rotateTo(Chassis& chassis, double target_angle_deg) {
    printf("=== ROTATE TO %.1f deg ===\n", target_angle_deg);
    fflush(stdout);
    
    const double MIN_POWER = 20.0;
    const double MAX_POWER = 50.0;
    const double TOLERANCE = 2.0;
    const int TIMEOUT = 2000;  // Reduced from 3000

    PID turnPID(1.5, 0.01, 0.15);
    
    int elapsed = 0;
    int settleCount = 0;
    int stallCount = 0;
    double lastAngle = chassis.getPose().theta;
    
    while (elapsed < TIMEOUT) {
        double currentAngle = chassis.getPose().theta;
        double error = target_angle_deg - currentAngle;
        
        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        // Check if settled
        if (fabs(error) < TOLERANCE) {
            settleCount++;
            if (settleCount > 5) break;  // 50ms settle time
        } else {
            settleCount = 0;
        }
        
        // Stall detection - if angle hasn't changed much in 300ms, exit
        if (fabs(currentAngle - lastAngle) < 0.5) {
            stallCount++;
            if (stallCount > 30) {
                printf("  (stall detected)\n");
                break;
            }
        } else {
            stallCount = 0;
            lastAngle = currentAngle;
        }
        
        double power = turnPID.update(error, 0.01);
        power = std::clamp(power, -MAX_POWER, MAX_POWER);
        
        // Apply minimum power for errors > 5 degrees
        if (fabs(power) < MIN_POWER && fabs(error) > 5.0) {
            power = (error > 0) ? MIN_POWER : -MIN_POWER;
        }
        
        // Zero power when very close (prevents oscillation)
        if (fabs(error) < TOLERANCE) {
            power = 0;
        }
        
        chassis.drive(0, 0, power);
        
        pros::delay(10);
        elapsed += 10;
    }
    
    chassis.drive(0, 0, 0);
    
    double finalAngle = chassis.getPose().theta;
    printf("=== ROTATE DONE: %.1f deg (target was %.1f) ===\n", finalAngle, target_angle_deg);
    fflush(stdout);
}

void moveVertical(Chassis& chassis, double distance_inches,
                  Mechanisms* mech, bool intaking, bool conveyor) {
    printf("=== MOVE %.1f inches ===\n", distance_inches);
    fflush(stdout);
    
    Pose start = chassis.getPose();
    double target = fabs(distance_inches);
    int direction = (distance_inches >= 0) ? 1 : -1;
    
    const double MAX_POWER = 60.0;
    const double MIN_POWER = 20.0;
    const double TOLERANCE = 1.0;  // Increased from 0.5 for faster exit
    const int TIMEOUT = 4000;
    
    PID drivePID(5.0, 0.05, 0.2);  // Increased P gain
    PID headingPID(1.0, 0, 0);
    
    double startHeading = start.theta;
    
    int elapsed = 0;
    int settleCount = 0;
    int stallCount = 0;
    double lastTraveled = 0;
    
    while (elapsed < TIMEOUT) {
        Pose current = chassis.getPose();
        double dx = current.x - start.x;
        double dy = current.y - start.y;
        double traveled = sqrt(dx*dx + dy*dy);
        double remaining = target - traveled;
        
        // Exit if close enough
        if (remaining < TOLERANCE) {
            settleCount++;
            if (settleCount > 5) break;  // 50ms settle
        } else {
            settleCount = 0;
        }
        
        // Stall detection - if position hasn't changed in 300ms, exit
        if (fabs(traveled - lastTraveled) < 0.1) {
            stallCount++;
            if (stallCount > 30) {
                printf("  (stall detected at %.1f inches)\n", traveled);
                break;
            }
        } else {
            stallCount = 0;
            lastTraveled = traveled;
        }
        
        // Calculate drive power
        double power = drivePID.update(remaining, 0.01);
        power = std::clamp(power, -MAX_POWER, MAX_POWER);
        
        // Apply minimum power for distances > 3 inches
        if (power < MIN_POWER && remaining > 3.0) {
            power = MIN_POWER;
        }
        
        power *= direction;
        
        // Heading correction
        double currentHeading = current.theta;
        double headingError = startHeading - currentHeading;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        double turnCorrection = headingPID.update(headingError, 0.01);
        turnCorrection = std::clamp(turnCorrection, -10.0, 10.0);
        
        chassis.drive(0, power, turnCorrection);
        
        if (mech != nullptr) {
            if (intaking) mech->intakeIn();
            if (conveyor) mech->conveyorUp();
        }
        
        pros::delay(10);
        elapsed += 10;
    }
    
    chassis.drive(0, 0, 0);
    
    if (mech != nullptr) {
        mech->intakeStop();
        mech->conveyorStop();
    }
    
    Pose end = chassis.getPose();
    double edx = end.x - start.x;
    double edy = end.y - start.y;
    double totalTraveled = sqrt(edx*edx + edy*edy);
    printf("=== MOVE DONE: %.1f inches ===\n", totalTraveled);
    fflush(stdout);
}

void moveToPose(Chassis& chassis, Pose target,
                Mechanisms* mech, bool reverse, bool intaking, bool conveyor) {
    printf("=== MOVE TO POSE (%.1f, %.1f) ===\n", target.x, target.y);
    fflush(stdout);
    
    Pose current = chassis.getPose();
    
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double angleToTarget = atan2(dx, dy) * 180.0 / M_PI;
    
    if (reverse) {
        angleToTarget += 180;
        while (angleToTarget > 180) angleToTarget -= 360;
    }
    
    rotateTo(chassis, angleToTarget);
    pros::delay(50);  // Brief pause between turn and drive
    
    double distance = sqrt(dx*dx + dy*dy);
    if (reverse) distance = -distance;
    moveVertical(chassis, distance, mech, intaking, conveyor);
    
    if (target.theta != 0) {
        rotateTo(chassis, target.theta);
    }
    
    printf("=== MOVE TO POSE DONE ===\n");
    fflush(stdout);
}

void positionReset(Chassis& chassis) {
    pros::delay(50);
    float theta = chassis.getPose().theta;
    chassis.setPose(0, 0, theta);
    pros::delay(50);
}

// ─────────────────────────────────────────────────────────────
// Autonomous Selector
// ─────────────────────────────────────────────────────────────

void run(Chassis& chassis, const RobotConfig& config) {
    printf("\n=============================\n");
    printf("AUTONOMOUS: %s\n", config.name.c_str());
    printf("=============================\n\n");
    fflush(stdout);
    
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
        printf("No autonomous selected!\n");
    #endif
    
    printf("\n=============================\n");
    printf("AUTONOMOUS COMPLETE\n");
    printf("=============================\n\n");
    fflush(stdout);
}

// ─────────────────────────────────────────────────────────────
// Autonomous Routines
// ─────────────────────────────────────────────────────────────

void skills(Chassis& chassis, Mechanisms& mech) {
    printf("Running Skills Autonomous\n");
    fflush(stdout);
    
    chassis.setPose(0, 0, 0);
    
    moveVertical(chassis, 24, nullptr, false, false);
    rotateTo(chassis, 90);
    moveVertical(chassis, 24, nullptr, false, false);
    rotateTo(chassis, 180);
    moveVertical(chassis, 24, nullptr, false, false);
    rotateTo(chassis, 270);
    moveVertical(chassis, 24, nullptr, false, false);
    rotateTo(chassis, 0);
}

void redLeft(Chassis& chassis, Mechanisms& mech) {
    printf("Running Red Left Autonomous\n");
    fflush(stdout);
    chassis.setPose(0, 0, 0);
}

void redRight(Chassis& chassis, Mechanisms& mech) {
    printf("Running Red Right Autonomous\n");
    fflush(stdout);
    chassis.setPose(0, 0, 0);
}

void blueLeft(Chassis& chassis, Mechanisms& mech) {
    printf("Running Blue Left Autonomous\n");
    fflush(stdout);
    chassis.setPose(0, 0, 0);
}

void blueRight(Chassis& chassis, Mechanisms& mech) {
    printf("Running Blue Right Autonomous\n");
    fflush(stdout);
    chassis.setPose(0, 0, 0);
}

void test(Chassis& chassis, Mechanisms& mech) {
    printf("=== CHASSIS.DRIVE() TEST ===\n\n");
    fflush(stdout);
    
    chassis.setPose(0, 0, 0);
    pros::delay(100);
    
    // Test 1: Forward 24 inches
    printf("Test 1: Forward 24 inches\n");
    fflush(stdout);
    moveVertical(chassis, 24, nullptr, false, false);
    
    Pose p1 = chassis.getPose();
    printf("Result: X=%.1f Y=%.1f Theta=%.1f\n\n", p1.x, p1.y, p1.theta);
    fflush(stdout);
    pros::delay(200);  // Reduced from 500
    
    // Test 2: Turn 90 degrees
    printf("Test 2: Turn to 90 degrees\n");
    fflush(stdout);
    rotateTo(chassis, 90);
    
    Pose p2 = chassis.getPose();
    printf("Result: Theta=%.1f (expected ~90)\n\n", p2.theta);
    fflush(stdout);
    pros::delay(200);
    
    // Test 3: Forward another 24 inches
    printf("Test 3: Forward 24 inches\n");
    fflush(stdout);
    moveVertical(chassis, 24, nullptr, false, false);
    
    Pose p3 = chassis.getPose();
    printf("Result: X=%.1f Y=%.1f\n\n", p3.x, p3.y);
    fflush(stdout);
    pros::delay(200);
    
    // Test 4: Turn back to 0
    printf("Test 4: Turn to 0 degrees\n");
    fflush(stdout);
    rotateTo(chassis, 0);
    
    Pose p4 = chassis.getPose();
    printf("Result: Theta=%.1f (expected ~0)\n\n", p4.theta);
    fflush(stdout);
    
    printf("=== TEST COMPLETE ===\n");
    fflush(stdout);
}

}  // namespace shulib::seasons::pushback::auton