#pragma once
//
// FakeGps — a deterministic IGps for host tests. Stores a CANONICAL robot-center pose
// (the VEX-frame conversion is the adapter's job, tested separately in
// gps_conversion_test) plus error and fix state, so tests can drive the fusion layer
// through on-strip / off-strip / high-error scenarios.

#include "shulib/hal/gps.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

class FakeGps final : public IGps {
public:
    [[nodiscard]] math::Pose2d pose() const override { return pose_; }
    [[nodiscard]] units::Length rmsError() const override { return rmsError_; }
    [[nodiscard]] bool hasFix() const override { return hasFix_; }

    // --- test injection ---
    void setPose(const math::Pose2d& p) { pose_ = p; }
    void setRmsError(units::Length e) { rmsError_ = e; }
    void setHasFix(bool f) { hasFix_ = f; }

private:
    math::Pose2d pose_{};
    units::Length rmsError_{};
    bool hasFix_ = false;  // default: no fix until told otherwise (safe default off-strip)
};

}  // namespace shulib::hal::fake
