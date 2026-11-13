#pragma once
//
// driver_heading.hpp — THE heading rule every driver-control program uses (chunk T4 Part 1a,
// 2026-09-27, in src/common/driver_controls.hpp; MOVED here, unchanged, by the mechanism core's
// MECH-A5, 2026-10-29, when driver_controls.hpp retired — the MECH brief's re-check T6).
//
// ── Why its own file ────────────────────────────────────────────────────────────────
// Two programs turn the sticks by a heading: robot two's driver file (src/common/driver_file.hpp's
// DriverFile::run(), which "shulib Drive" and the competition program run) and the invented X-drive's
// program (src/programs/xdrive_program.cpp), which has no driver file. The rule must have ONE copy —
// the two programs carried one each until T4 Part 1a, and neither program file compiles on the host,
// so a copy there would drift untested — so it lives here, where both include it, and nothing else
// rides along: the X-drive needs the heading, not the mechanism controls driver_controls.hpp held.
//
// PROS-FREE ON PURPOSE: the host suite runs it (test/driver_heading_test.cpp, each of its three terms
// flipped alone over the fake provider). NO HARDWARE CLAIM: no heading it trusted has turned a stick
// on a robot (field-centric is OFF in robot two's driver file).

#include "shulib/diag/fault.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/teleop_loop.hpp"  // teleop::HeadingSample

namespace shulib::bench {

/// THE heading field-centric turns the sticks by (driver_control.hpp's contract), for every
/// driver-control program — the X-drive and every driver file call this one; there is no second
/// copy. The fused heading, TRUSTED only when all three hold:
///   * the estimate is live — the Localizer is past Uninitialized (motion.hpp's wait-for-live
///     contract; Degraded has four causes and says nothing specific about the heading, so it is not
///     the test);
///   * the IMU itself reports ready;
///   * no IMU_LOST fault has been raised this run.
/// Read before the tick's drive(), so it is the previous tick's estimate (10 ms old on a V5).
inline teleop::HeadingSample driverHeading(const robot::Robot& r) {
    const bool trusted =
        r.localizer().qualityClass() != localization::Localizer::Quality::Uninitialized
        && r.context().imu().isReady() && r.faults().raiseCount(diag::FaultCode::ImuLost) == 0;
    return teleop::HeadingSample{.radians = r.localizer().pose().heading().radians(),
                                 .trusted = trusted};
}

}  // namespace shulib::bench
