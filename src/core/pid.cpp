#include "shulib/core/pid.hpp"

float shulib::PID::update(float error, float time) {
    float derivative = ((error - prevError) / time);
    integral += (error * time);
    prevError = error;
    return (kP * error) + (kI * integral) + (kD * derivative);
}

void shulib::PID::setKP(float newKP){
    this->kP = newKP;
}

void shulib::PID::setKI(float newKI){
    this->kI = newKI;
}

void shulib::PID::setKD(float newKD){
    this->kD = newKD;
}
        
void shulib::PID::reset() {
    integral = 0;
    prevError = 0;
}