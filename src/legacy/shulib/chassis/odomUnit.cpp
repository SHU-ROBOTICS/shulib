#include "shulib/chassis/odomUnit.hpp"
#include <cmath>

shulib::OdomUnit::OdomUnit(pros::Rotation* sensor, float diameter, float offset) {
    this->sensor = sensor;
    this->diameter = diameter;
    this->offset = offset;
    this->lastPosition = 0;
}

void shulib::OdomUnit::reset() {
    if (this->sensor != nullptr) this->sensor->reset_position();
}

double shulib::OdomUnit::get_travel() {
    if (this->sensor != nullptr) {
        return (float(this->sensor->get_position()) * this->diameter * M_PI / 36000);
    } else {
        return 0;
    }
}

double shulib::OdomUnit::get_travel_delta() {
    double current = this->get_travel();
    double delta = current - this->lastPosition;
    this->lastPosition = current;
    return delta;
}

double shulib::OdomUnit::get_offset() { return this->offset; }