#include "shulib/seasons/pushback_2026/mechanisms.hpp"

namespace shulib::seasons::pushback {

Mechanisms::Mechanisms(const MechanismConfig& config) {
    // Convert vector<int8_t> to initializer list for MotorGroup
    intake = std::make_unique<pros::MotorGroup>(config.intake_ports);
    conveyor = std::make_unique<pros::MotorGroup>(config.conveyor_ports);
    releaser = std::make_unique<pros::MotorGroup>(config.releaser_ports);
    
    arm = std::make_unique<pros::adi::Pneumatics>(
        config.pneumatic_arm_port, 
        config.arm_default_state
    );
    lever = std::make_unique<pros::adi::Pneumatics>(
        config.pneumatic_lever_port, 
        config.lever_default_state
    );
}

// ─────────────────────────────────────────────────────────────
// Intake
// ─────────────────────────────────────────────────────────────

void Mechanisms::intakeIn() {
    intake->move(FULL_SPEED);
}

void Mechanisms::intakeOut() {
    intake->move(-FULL_SPEED);
}

void Mechanisms::intakeStop() {
    intake->move(0);
}

void Mechanisms::intakeMove(int velocity) {
    intake->move(velocity);
}

// ─────────────────────────────────────────────────────────────
// Conveyor
// ─────────────────────────────────────────────────────────────

void Mechanisms::conveyorUp() {
    conveyor->move(FULL_SPEED);
}

void Mechanisms::conveyorDown() {
    conveyor->move(-FULL_SPEED);
}

void Mechanisms::conveyorStop() {
    conveyor->move(0);
}

void Mechanisms::conveyorMove(int velocity) {
    conveyor->move(velocity);
}

// ─────────────────────────────────────────────────────────────
// Releaser
// ─────────────────────────────────────────────────────────────

void Mechanisms::releaserForward() {
    releaser->move(FULL_SPEED);
}

void Mechanisms::releaserBackward() {
    releaser->move(-FULL_SPEED);
}

void Mechanisms::releaserStop() {
    releaser->move(0);
}

void Mechanisms::releaserMove(int velocity) {
    releaser->move(velocity);
}

// ─────────────────────────────────────────────────────────────
// Pneumatics
// ─────────────────────────────────────────────────────────────

void Mechanisms::toggleArm() {
    arm->toggle();
}

void Mechanisms::extendArm() {
    arm->extend();
}

void Mechanisms::retractArm() {
    arm->retract();
}

bool Mechanisms::isArmExtended() const {
    return arm->is_extended();
}

void Mechanisms::toggleLever() {
    lever->toggle();
}

void Mechanisms::extendLever() {
    lever->extend();
}

void Mechanisms::retractLever() {
    lever->retract();
}

bool Mechanisms::isLeverExtended() const {
    return lever->is_extended();
}

// ─────────────────────────────────────────────────────────────
// Combos
// ─────────────────────────────────────────────────────────────

void Mechanisms::intakeAndConveyorIn() {
    intakeIn();
    conveyorUp();
}

void Mechanisms::intakeAndConveyorOut() {
    intakeOut();
    conveyorDown();
}

void Mechanisms::stopAll() {
    intakeStop();
    conveyorStop();
    releaserStop();
}

}  // namespace shulib::seasons::pushback
