#include "shulib/pid.hpp"

float shulib::PID::update(float error, float dt) {
    // Reset integral if error changes sign
    if ((error > 0) != (prevError > 0)) {
        integral = 0;
    }
    float derivative = (error - prevError) / dt;
    integral += error * dt;
    prevError = error;
    return kP * error + kI * integral + kD * derivative;
}

        
void shulib::PID::reset() {
    integral = 0;
    prevError = 0;
}