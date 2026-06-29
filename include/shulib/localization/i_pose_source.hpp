#pragma once
//
// IPoseSource — the READ seam every pose consumer (motion, alignment, telemetry, skills) depends
// on (master plan §6: "IPoseSource"). The concrete `Localizer` implements it; a future EKF-backed
// localizer, a log-replay source, or a test fake implement the SAME interface, so swapping the
// fusion tier never touches a caller. PROS-free (L2). Heading is IMU-owned; twist() is the matching
// field-frame pose derivative.

#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"

namespace shulib::localization {

class IPoseSource {
public:
    virtual ~IPoseSource() = default;
    IPoseSource() = default;
    IPoseSource(const IPoseSource&) = default;
    IPoseSource(IPoseSource&&) = default;
    IPoseSource& operator=(const IPoseSource&) = default;
    IPoseSource& operator=(IPoseSource&&) = default;

    /// Best current field-frame pose (heading == the IMU heading).
    [[nodiscard]] virtual math::Pose2d pose() const noexcept = 0;

    /// Field-frame velocity estimate (the derivative of the published pose).
    [[nodiscard]] virtual math::Twist2d twist() const noexcept = 0;

    /// Graded trust in the current estimate, in [0,1] — a measurable number, not a vibe.
    [[nodiscard]] virtual double quality() const noexcept = 0;

    /// True when no absolute corrector contributed this tick (running on odom + IMU alone).
    [[nodiscard]] virtual bool isDeadReckoning() const noexcept = 0;
};

}  // namespace shulib::localization
