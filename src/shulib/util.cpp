#include <vector>
#include "shulib/pose.hpp"
#include "shulib/util.hpp"

float shulib::slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

constexpr float shulib::sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}

float shulib::avg(std::vector<float> values) {
    float sum = 0;
    for (float value : values) { sum += value; }
    return sum / values.size();
}

float shulib::ema(float current, float previous, float smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}

float shulib::getCurvature(Pose pose, Pose other) {
    // calculate whether the pose is on the left or right side of the circle
    float side = shulib::sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}