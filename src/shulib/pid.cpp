#include "shulib/pid.hpp"

float shulib::PID::update(float error) {
    float derivative = (error - prevError);
    integral += (error);
    prevError = error;
    return (kP * error) + (kI * integral) + (kD * derivative);
}
        
void shulib::PID::reset() {
    integral = 0;
    prevError = 0;
}