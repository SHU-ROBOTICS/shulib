#pragma once
//
// FakeOptical — a deterministic IOptical for host tests. Injectable hue / saturation
// / brightness / proximity, so color-confirmation logic is reproducible.

#include "shulib/hal/optical.hpp"

namespace shulib::hal::fake {

class FakeOptical final : public IOptical {
public:
    [[nodiscard]] double hue() const override { return hue_; }
    [[nodiscard]] double saturation() const override { return saturation_; }
    [[nodiscard]] double brightness() const override { return brightness_; }
    [[nodiscard]] double proximity() const override { return proximity_; }

    void setHue(double h) { hue_ = h; }
    void setSaturation(double s) { saturation_ = s; }
    void setBrightness(double b) { brightness_ = b; }
    void setProximity(double p) { proximity_ = p; }

private:
    double hue_ = 0.0;
    double saturation_ = 0.0;
    double brightness_ = 0.0;
    double proximity_ = 0.0;
};

}  // namespace shulib::hal::fake
