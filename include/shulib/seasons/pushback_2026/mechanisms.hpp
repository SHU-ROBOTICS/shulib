#pragma once

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
#include "shulib/robots/robot_config.hpp"
#include <memory>
#include <vector>
#include <cstdint>

namespace shulib::seasons::pushback {

class Mechanisms {
public:
    explicit Mechanisms(const MechanismConfig& config);

    // Intake
    void intakeIn();
    void intakeOut();
    void intakeStop();
    void intakeMove(int velocity);

    // Conveyor
    void conveyorUp();
    void conveyorDown();
    void conveyorStop();
    void conveyorMove(int velocity);

    // Releaser
    void releaserForward();
    void releaserBackward();
    void releaserStop();
    void releaserMove(int velocity);

    // Pneumatics
    void toggleArm();
    void extendArm();
    void retractArm();
    bool isArmExtended() const;

    void toggleLever();
    void extendLever();
    void retractLever();
    bool isLeverExtended() const;

    // Combos
    void intakeAndConveyorIn();
    void intakeAndConveyorOut();
    void stopAll();

private:
    std::unique_ptr<pros::MotorGroup> intake;
    std::unique_ptr<pros::MotorGroup> conveyor;
    std::unique_ptr<pros::MotorGroup> releaser;
    std::unique_ptr<pros::adi::Pneumatics> arm;
    std::unique_ptr<pros::adi::Pneumatics> lever;
    
    static const int FULL_SPEED = 127;
};

}  // namespace shulib::seasons::pushback
