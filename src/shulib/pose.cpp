#include "shulib/pose.hpp"
#include <cmath>

shulib::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

shulib::Pose shulib::Pose::operator+(const shulib::Pose& other) const {
    return shulib::Pose(this->x + other.x, this->y + other.y, this->theta);
}

shulib::Pose shulib::Pose::operator-(const shulib::Pose& other) const {
    return shulib::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float shulib::Pose::operator*(const shulib::Pose& other) const { return this->x * other.x + this->y * other.y; }

shulib::Pose shulib::Pose::operator*(const float& other) const {
    return shulib::Pose(this->x * other, this->y * other, this->theta);
}

shulib::Pose shulib::Pose::operator/(const float& other) const {
    return shulib::Pose(this->x / other, this->y / other, this->theta);
}

shulib::Pose shulib::Pose::lerp(shulib::Pose other, float t) const {
    return shulib::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float shulib::Pose::distance(shulib::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float shulib::Pose::angle(shulib::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

shulib::Pose shulib::Pose::rotate(float angle) const {
    return shulib::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string shulib::format_as(const shulib::Pose& pose) {
    // the double brackets become single brackets
    return "shulib::Pose { x: " + std::to_string(pose.x) + ", y: " + std::to_string(pose.y) +
           ", theta: " + std::to_string(pose.theta) + " }";

}