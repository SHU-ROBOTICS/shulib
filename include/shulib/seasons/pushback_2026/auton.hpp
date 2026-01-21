#pragma once

#include "shulib/core/chassis.hpp"
#include "shulib/core/pose.hpp"
#include "shulib/robots/robot_config.hpp"
#include "shulib/seasons/pushback_2026/mechanisms.hpp"

namespace shulib::seasons::pushback::auton {

/**
 * @brief Run the selected autonomous routine
 * 
 * @param chassis Reference to the chassis
 * @param config Reference to the robot configuration
 */
void run(Chassis& chassis, const RobotConfig& config);

// ─────────────────────────────────────────────────────────────
// Motion Functions
// ─────────────────────────────────────────────────────────────

/**
 * @brief Rotate to a target angle using PID
 * 
 * @param chassis Reference to the chassis
 * @param target_angle Target angle in degrees
 */
void rotateTo(Chassis& chassis, double target_angle);

/**
 * @brief Move forward/backward a specified distance
 * 
 * @param chassis Reference to the chassis
 * @param distance_inches Distance to move (negative = backward)
 * @param mech Optional mechanisms pointer for intaking while moving
 * @param intaking Whether to run intake while moving
 * @param conveyor Whether to run conveyor while moving
 */
void moveVertical(Chassis& chassis, double distance_inches, 
                  Mechanisms* mech = nullptr, 
                  bool intaking = false, 
                  bool conveyor = false);

/**
 * @brief Move to a target pose
 * 
 * @param chassis Reference to the chassis
 * @param target_pose Target pose (x, y, theta)
 * @param mech Optional mechanisms pointer
 * @param reverse Whether to drive in reverse
 * @param intaking Whether to run intake
 * @param conveyor Whether to run conveyor
 */
void moveToPose(Chassis& chassis, Pose target_pose,
                Mechanisms* mech = nullptr,
                bool reverse = false,
                bool intaking = false,
                bool conveyor = false);

/**
 * @brief Reset position to (0, 0, current_theta)
 * 
 * @param chassis Reference to the chassis
 */
void positionReset(Chassis& chassis);

// ─────────────────────────────────────────────────────────────
// Autonomous Routines
// ─────────────────────────────────────────────────────────────

void skills(Chassis& chassis, Mechanisms& mech);
void redLeft(Chassis& chassis, Mechanisms& mech);
void redRight(Chassis& chassis, Mechanisms& mech);
void blueLeft(Chassis& chassis, Mechanisms& mech);
void blueRight(Chassis& chassis, Mechanisms& mech);
void test(Chassis& chassis, Mechanisms& mech);

}  // namespace shulib::seasons::pushback::auton
