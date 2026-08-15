#pragma once
//
// ProsGps — IGps over pros::Gps (chunk R1a): the absolute-position corrector's
// sensor behind the HAL.
//
// BINDS: get_position_and_orientation() — ONE atomic status read carrying
// x/y (meters) and yaw (CW-from-North degrees, HA-106), so position and
// heading in a single IGps::pose() come from the SAME device sample rather
// than three reads that could straddle an update. Plus get_error() (meters,
// HA-07) and get_offset() (the HA-06 boot check). Conversion: gpsToRobotPose()
// — sensor meters → canonical robot-CENTER inches, lever arm removed exactly
// once — and gpsRmsErrorToCanonical(), the function E2 built "so this is a
// function the adapter CALLS rather than a paragraph the adapter author must
// remember". This adapter is that author, and it calls both.
//
// CONSTRUCTION — PORT-ONLY, enforced (HA-06): the ctor takes the port and
// NOTHING that could reach the firmware offset. It never calls set_offset(),
// initialize_full(), or the offset-taking ctors: any firmware offset makes
// get_position() report the robot CENTER, and this adapter's own lever-arm
// removal would then subtract the arm TWICE — inches of silent,
// heading-dependent bias. The boot check verifies get_offset() == (0,0):
//  * offset readable and (0,0)  → verified, normal operation;
//  * offset readable and NONZERO at CONSTRUCTION → SHULIB_PRECONDITION (a
//    configured device is a robot-setup error — loud at boot beats
//    double-subtracted at match);
//  * offset UNREADABLE (device absent/calibrating at construction) → the
//    check DEFERS: hasFix() stays false until a later read CAN verify the
//    offset. If the DEFERRED check then finds a NONZERO offset, the device is
//    marked permanently no-fix for the run instead of throwing — pose() and
//    hasFix() are read paths and MUST NOT throw (gps.hpp:27-29); the
//    corrector sees a dead GPS (quality decay), which is honest and safe,
//    where an inches-biased "healthy" GPS is neither. Never trusts an
//    unverified device either way.
//
// SENTINEL SCREENING BEFORE CONVERSION (HA-08): PROS_ERR_F in any status
// field → hasFix() = false for that read, pose() returns the last good pose
// (finite by construction — gps.hpp:27-29: unspecified but finite, callers
// gate on hasFix()). Feeding a sentinel into gpsSensorPose()/
// gpsRmsErrorToCanonical() THROWS by design — that path is the fail-loud
// backstop, not the off-strip path, and this adapter's job is to make it
// unreachable.
//
// WHAT hasFix() MEANS HERE — device-level validity ONLY: the reads are finite
// and the offset is verified. Error-MAGNITUDE gating (is this fix worth
// folding?) deliberately stays in the fusion layer, which already owns it
// with registered thresholds (E2's corrector, HA-61..67) — two layers gating
// on the same number with two thresholds would fight, invisibly.
//
// DELIBERATELY NOT here: no northHeadingDeg guessing (ctor parameter, owner =
// the robot's start-pose authority, HA-09); no lever-arm value (ctor
// parameters, robot config, HA-10); no filtering, no outlier logic (E2's).
//
// HA register: HA-01, HA-06..HA-09, HA-106.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/gps.hpp"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdint>

#include "shulib/core/check.hpp"
#include "shulib/hal/gps.hpp"
#include "shulib/hal/gps_conversion.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

/// IGps over pros::Gps — the absolute-position corrector's sensor, converted to canonical
/// robot-CENTER inches at this seam.
///
/// Construction is PORT-ONLY on purpose, and the constructor VERIFIES it: this adapter owns
/// the lever-arm removal, so a firmware offset configured on the device would make the arm be
/// subtracted twice — inches of silent, heading-dependent bias. A nonzero `get_offset()` at
/// construction is a precondition failure; an unreadable one (device absent or calibrating)
/// defers the check and leaves hasFix() false until a later read can settle it, and if THAT
/// read finds an offset the device is marked no-fix for the whole run rather than throwing
/// from a read path.
///
/// Each reader takes ONE atomic device sample (position and heading from the same status), and
/// screens PROS_ERR_F sentinels to no-fix BEFORE any conversion runs — feeding a sentinel into
/// the conversion helpers throws by design, and keeping that path unreachable is this class's
/// job. While fix-less, pose() and rmsError() hold their last good values.
class ProsGps final : public IGps {
public:
    /// PORT-ONLY device construction (header). `leverArmForward`/`leverArmLeft`:
    /// the sensor's position in the BODY frame (inches — HA-10, robot config).
    /// `northHeadingDeg`: the canonical heading VEX-North points toward (HA-09,
    /// same owner as the robot's start pose).
    ProsGps(std::uint8_t port, units::Length leverArmForward, units::Length leverArmLeft,
            double northHeadingDeg = kGpsDefaultNorthHeadingDeg)
        : sensor_{port},
          leverArmForward_{leverArmForward},
          leverArmLeft_{leverArmLeft},
          northHeadingDeg_{northHeadingDeg} {
        SHULIB_PRECONDITION(std::isfinite(leverArmForward.value())
                                && std::isfinite(leverArmLeft.value()),
                            "ProsGps: lever arm must be finite");
        SHULIB_PRECONDITION(std::isfinite(northHeadingDeg),
                            "ProsGps: northHeadingDeg must be finite");
        verifyOffset(/*bootPhase=*/true);  // the HA-06 boot check (defers if unreadable)
    }

    /// Canonical robot-CENTER pose. Finite always; meaningful only while
    /// hasFix() (gps.hpp:26-30). Never throws.
    [[nodiscard]] math::Pose2d pose() const override {
        refresh();
        return lastPose_;
    }

    /// Canonical inches via gpsRmsErrorToCanonical (HA-07 — the ×39.37 that
    /// was prose until E2). Holds last-good while fix-less.
    [[nodiscard]] units::Length rmsError() const override {
        refresh();
        return lastRmsError_;
    }

    /// Device-level validity ONLY: this read's fields were all finite and the firmware offset
    /// is verified (0,0). It says nothing about whether the fix is ACCURATE enough to fold —
    /// that threshold belongs to the fusion corrector, which already owns it; two layers
    /// gating on rmsError() with two thresholds would fight invisibly. Like pose() and
    /// rmsError(), calling this takes a FRESH device sample, so it is not a free predicate.
    [[nodiscard]] bool hasFix() const override {
        refresh();
        return hasFix_;
    }

    /// How many reads were screened to no-fix (T7 observability).
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    /// The HA-06 boot check (header: the outcomes, and why only the boot
    /// phase may throw — pose()/hasFix() are MUST-NOT-THROW read paths).
    void verifyOffset(bool bootPhase) const {
        if (offsetVerified_ || offsetRejected_) {
            return;
        }
        const auto offset = sensor_.get_offset();
        if (!std::isfinite(offset.x) || !std::isfinite(offset.y)) {
            return;  // unreadable now — defer; hasFix() stays false until verified
        }
        if (offset.x == 0.0 && offset.y == 0.0) {
            offsetVerified_ = true;
            return;
        }
        if (bootPhase) {
            SHULIB_PRECONDITION(false,
                                "ProsGps: firmware offset is configured (get_offset() != (0,0)) — "
                                "this adapter owns the lever arm and would subtract it TWICE. "
                                "Clear the device offset (HA-06).");
        }
        // Deferred discovery: mark the device permanently untrusted for the
        // run rather than throwing from a read path (header).
        offsetRejected_ = true;
    }

    /// One device sample → pose/error/fix, sentinel-screened BEFORE conversion.
    void refresh() const {
        verifyOffset(/*bootPhase=*/false);  // a deferred boot check retries here
        if (!offsetVerified_) {
            // COUNTED. This path used not to be, and it is the one failure the class treats as
            // PERMANENT: once verifyOffset() sets offsetRejected_, the device is no-fix for the
            // whole run, so faultedReads() stayed at 0 forever — the least informative possible
            // answer to "why is the GPS dead?", in precisely the case the header's HA-06
            // discussion cares most about.
            faultedReads_ += 1;
            hasFix_ = false;
            return;
        }
        const auto status = sensor_.get_position_and_orientation();
        const double errMeters = sensor_.get_error();
        if (!std::isfinite(status.x) || !std::isfinite(status.y) || !std::isfinite(status.yaw)
            || !std::isfinite(errMeters)) {
            // Off-strip / calibrating / unplugged (HA-08): screened to no-fix
            // BEFORE any conversion sees a sentinel.
            faultedReads_ += 1;
            hasFix_ = false;
            return;
        }
        lastPose_ = gpsToRobotPose(status.x, status.y, status.yaw, leverArmForward_,
                                   leverArmLeft_, northHeadingDeg_);
        lastRmsError_ = gpsRmsErrorToCanonical(errMeters);
        hasFix_ = true;
    }

    ::pros::v5::Gps sensor_;
    units::Length leverArmForward_;
    units::Length leverArmLeft_;
    double northHeadingDeg_;
    // `mutable`: IGps's readers are const; the screen/hold state is state OF
    // THE READ (T7), and the deferred boot check must be able to complete
    // from a const reader.
    mutable bool offsetVerified_ = false;
    mutable bool offsetRejected_ = false;  ///< deferred check found a configured offset (header)
    mutable bool hasFix_ = false;
    mutable math::Pose2d lastPose_{};  // origin until a first fix — finite by construction
    mutable units::Length lastRmsError_{99.0};  // "no information yet" (HA-29's shape)
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
