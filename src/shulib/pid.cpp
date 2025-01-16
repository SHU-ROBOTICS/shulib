#include "pid.hpp"
#include "util.hpp"
#include "shulib/logger.hpp"

namespace shulib {
PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {
    // logger().log("PID initialized with kP=", kP, " kI=", kI, " kD=", kD);
}

float PID::update(const float error) {
    // calculate integral
    integral += error;
    if (sgn(error) != sgn((prevError)) && signFlipReset) integral = 0;
    if (fabs(error) > windupRange && windupRange != 0) integral = 0;

    // calculate derivative
    derivative = error - prevError;
    prevError = error;

    // calculate each component
    float pTerm = error * kP;
    float iTerm = integral * kI;
    float dTerm = derivative * kD;

    // logger().log("PID update - error:", error, 
    //              " P:", pTerm, "(", kP, "*", error, ")",
    //              " I:", iTerm, "(", kI, "*", integral, ")",
    //              " D:", dTerm, "(", kD, "*", derivative, ")");

    // calculate output
    float output = pTerm + iTerm + dTerm;
    // logger().log("PID output:", output);
    
    return output;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
    derivative = 0;
    logger().log("PID reset");
}
} // namespace shulib