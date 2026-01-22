#pragma once

namespace shulib {
class PID {
    public:
        PID(float kP, float kI, float kD, float kC): kP(kP), kI(kI), kD(kD), kC(kC){}

        float update(float error, float time);

        void setKP(float newKP);
        void setKI(float newKI);
        void setKD(float newKD);

        void reset();

    protected:
        float kP;
        float kI;
        float kD;
        float kC;

        float integral = 0;
        float prevError = 0;
};
} // namespace shulib