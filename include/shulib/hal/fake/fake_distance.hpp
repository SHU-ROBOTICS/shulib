#pragma once
//
// FakeDistance — a deterministic IDistance for host tests. Injectable distance +
// confidence, so "object present / absent" scenarios are reproducible.

#include "shulib/hal/distance.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

class FakeDistance final : public IDistance {
public:
    [[nodiscard]] units::Length distance() const override { return distance_; }
    [[nodiscard]] double confidence() const override { return confidence_; }

    void setDistance(units::Length d) { distance_ = d; }
    void setConfidence(double c) { confidence_ = c; }

private:
    units::Length distance_{};
    double confidence_ = 0.0;  // default: no usable reading
};

}  // namespace shulib::hal::fake
