#pragma once
#include "pros/motor_group.hpp"
#include "shulib/core/drivetrain.hpp"

namespace shulib {

class TankDrive : public Drivetrain {
public:
    TankDrive(pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors, float trackWidth,
            float wheelDiameter, float rpm)
        : Drivetrain(wheelDiameter, rpm, 0),
        trackWidth(trackWidth) {
        
        MotorConfig leftConfig = {&leftMotors, 0, 1, 1};
        motorConfigs.push_back(leftConfig);
        
        MotorConfig rightConfig = {&rightMotors, 0, 1, -1};
        motorConfigs.push_back(rightConfig);
    }
    
private:
    float trackWidth;
};

}  // namespace shulib
