#pragma once

namespace shulib {
class PID {
    public:
        PID(float kP, float kI, float kD): kP(kP), kI(kI), kD(kD){}

        float update(float error);

        void reset();

    protected:
        const float kP;
        const float kI;
        const float kD;

        float integral = 0;
        float prevError = 0;
};
} // namespace shulib