// The wall reset END TO END on the host test plant (chunk L6-A1; the L6 brief's tests T8 and T16's
// plant half — P2's scenarios, re-based on the F1 field, as tests): a tank on the independent field
// plant, a FIELD start, the explicit call over the plant's side sensors, graded against the plant's
// TRUTH. Nothing here has run on a robot.
//
// THE KNOWN LIMITS are pinned AS SUCH — named "known limit: an object at a plausible distance" — so a
// change either way is seen: no reading gate can tell a tall object at a plausible distance from the
// wall (the brief's landmine 1), and no test may pretend one does.
//
// Geometry (derived, not read off the plant): the left sensor 7.5 in left of the centre, 10.5 in up,
// facing straight out; parked with the lens 7 in from the left wall (x = -70.205 + 7.5 + 7 = -55.7),
// the beam centre lands at the robot's y — y = -38 is inside the left wall's clean band right after a
// seed (the start pieces end at |along| 28.28, the Load Zone begins at 47.10; the widening is 7 tan
// 18 deg = 2.27 + the 3-in margin + 10 % of the travel since y was set).

#include "doctest.h"

#include <cmath>
#include <string>

#include "shulib/chassis/routine.hpp"
#include "shulib/chassis/wall_reset_call.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/sim/field_plant.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "wall_reset_rig.hpp"

namespace {

namespace loc = shulib::localization;
namespace ow = shulib::localization::override2026;
using loc::WallResetVerdict;
using shulib::diag::FaultCode;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sim::FieldBox;
using shulib::sim::FieldModel;
using shulib::sim::SideSensorPlantConfig;
using shulib::units::AngleDim;
using shulib::units::Length;
using shulib::units::Time;
using wall_rig::kD2R;
using wall_rig::kW;
using wall_rig::WallRig;

constexpr double kParkX = -70.205 + 7.5 + 7.0;  // the left lens 7 in from the left wall

Pose2d parkLeft(double y = -38.0, double headingDeg = 90.0) {
    return Pose2d{Length{kParkX}, Length{y}, Angle::degrees(headingDeg)};
}

/// Nose along +x, the RIGHT lens `d` in from the RED wall (y = -70.205): the robot's y.
double yForRightOnRed(double d) { return -70.205 + 7.5 + d; }

double xErr(WallRig& w) { return w.loc().pose().x().value() - w.h().truePose().x().value(); }
double yErr(WallRig& w) { return w.loc().pose().y().value() - w.h().truePose().y().value(); }

/// Move the ESTIMATE's `axis` by `by` inches without touching the truth or the counters (odometry
/// drift, as a test can make it): the Localizer's own one-axis seat.
void drift(WallRig& w, loc::Axis axis, double by) {
    const double now = axis == loc::Axis::X ? w.loc().pose().x().value() : w.loc().pose().y().value();
    w.loc().seatAxis(axis, Length{now + by});
    w.idle(0.05);
}

SideSensorPlantConfig leftAt(double height) {
    SideSensorPlantConfig c = wall_rig::trueLeft();
    c.height = height;
    return c;
}

}  // namespace

// ── P2's scenarios ─────────────────────────────────────────────────────────────────────────────

// S1 — a FIELD start, a leg ALONG the left wall (D4's sequence, as an auton would drive it), the
// estimate drifted 2.5 in: SEATED to within 0.1 in of the truth. RC31: the seat is over 2.25 in, and
// the Routine step after it raises NO IMPLAUSIBLE (every verb's first tick has dt 0).
TEST_CASE("L6 T8 S1 a leg along the left wall, the estimate drifted 2.5 in: SEATED within 0.1 in") {
    WallRig w{Pose2d{Length{kParkX}, Length{-50.0}, Angle::degrees(90.0)}};
    w.idle(0.3);
    w.leg(kParkX, -38.0, 90.0);
    drift(w, loc::Axis::X, 2.5);
    REQUIRE(std::abs(xErr(w) - 2.5) < 0.2);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    REQUIRE(rep.verdict == WallResetVerdict::Seated);
    CHECK(std::abs(xErr(w)) < 0.1);
    CHECK(std::abs(rep.applied.value()) > 2.25);
    CHECK(rep.errorBound.value() <= 1.0);
    // The next step is a Routine verb: no IMPLAUSIBLE from the deliberate jump.
    const int before = w.rig.latch.raiseCount(FaultCode::Implausible);
    shulib::chassis::Routine r{w.chassis, "after the reset"};
    r.pause(Time{0.1});
    CHECK(r.result().ok);
    CHECK(w.rig.latch.raiseCount(FaultCode::Implausible) == before);
}

// RC31's other half, pinned as documented: a Tier-3 loop that ticks the scheduler itself after a
// seat over 2.25 in sees exactly ONE advisory IMPLAUSIBLE (the guard knows only dt).
TEST_CASE("L6 T8 RC31 a caller-paced tick after a seat over 2.25 in raises exactly one IMPLAUSIBLE") {
    WallRig w{parkLeft()};
    w.idle(0.3);
    drift(w, loc::Axis::X, 2.5);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    REQUIRE(rep.verdict == WallResetVerdict::Seated);
    REQUIRE(std::abs(rep.applied.value()) > 2.25);
    const int before = w.rig.latch.raiseCount(FaultCode::Implausible);
    (void)w.chassis.scheduler().tick();
    w.pacer.pace();
    (void)w.chassis.scheduler().tick();
    CHECK(w.rig.latch.raiseCount(FaultCode::Implausible) == before + 1);
}

// Bug caught (the L6-A1 review): a robot STANDING STILL refused because its IMU's heading NOISE was
// read as motion. The host plant's hostile IMU with the register's HA-21 white heading noise (0.05
// deg; and 0.1 deg, twice it), HA-20's drift and HA-22's rate noise ON (its defaults), on this rig's
// tracking wheels — 3 in and 4.5 in off the turning centre, the harness's defaults — seeded as a
// FIELD start is (setPose + reseatHeading). Before the fix:
//   - the turn counter summed every tick's |change|: ~5.6 deg/s of "turning" while nothing turned,
//     so after 45 s the precision gate refused IMPRECISE (the 60-s skills run);
//   - stillness was one tick's finite-difference velocity, and the noise moved the estimate a few
//     thousandths of an inch a tick: 1.1-1.4 in/s of phantom speed, every reset refused MOVING.
// Now the counter stays at a few degrees (HA-20's drift is real turning of the reading) and the
// reset SEATS both right after the seed and after 60 s still.
TEST_CASE("L6 T8 a robot standing still under HA-21 heading noise: the counter stays ~0 and it SEATS, at 0.3 s and 60 s") {
    for (const double sigmaDeg : {0.05, 0.1}) {
        shulib::sim::ImuHostileConfig ic{};
        ic.calibrationEnd = Time{0.0};
        ic.headingNoiseSigmaRad = sigmaDeg * kD2R;
        shulib::sim::ImuHostileModel imu{ic};
        WallRig w{parkLeft(), FieldModel::override2026(), wall_rig::trueLeft(), wall_rig::trueRight(), &imu};
        w.idle(0.1);
        w.loc().setPose(parkLeft());  // the FIELD start's seed, as competition::runAuton makes it
        w.loc().reseatHeading(Angle::degrees(90.0));
        w.idle(0.3);
        INFO("heading noise sigma " << sigmaDeg << " deg");
        drift(w, loc::Axis::X, 1.0);
        const loc::WallResetReport early = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
        CHECK(early.verdict == WallResetVerdict::Seated);
        CHECK(early.speedSeen.value() < 0.25);
        CHECK(std::abs(xErr(w)) < 0.15);
        w.idle(60.0);
        CHECK(w.loc().turnedSinceHeadingSeed().value() < 3.0 * kD2R);
        drift(w, loc::Axis::X, 1.0);
        const loc::WallResetReport late = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
        CHECK(late.verdict == WallResetVerdict::Seated);
        CHECK(late.headingTolerance.value() < 2.06 * kD2R);
        CHECK(late.speedSeen.value() < 0.25);
        CHECK(std::abs(xErr(w)) < 0.15);
    }
}

// S2 — drifted 3.5 in with maxCorrection 3: REFUSED disagrees, the estimate untouched.
TEST_CASE("L6 T8 S2 drifted past maxCorrection: REFUSED disagrees, untouched") {
    WallRig w{parkLeft()};
    w.idle(0.3);
    drift(w, loc::Axis::X, 3.5);
    const double x0 = w.loc().pose().x().value();
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::Disagrees);
    CHECK(w.loc().pose().x().value() == x0);
    CHECK(std::abs(rep.measured.value() - w.h().truePose().x().value()) < 0.1);  // the wall knew
}

// S3 — parked 8 deg off parallel: REFUSED not square.
TEST_CASE("L6 T8 S3 parked 8 deg off parallel: REFUSED not square") {
    WallRig w{parkLeft(-38.0, 98.0)};
    w.idle(0.3);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::NotSquare);
    CHECK(rep.offSquare.value() == doctest::Approx(8.0 * kD2R).epsilon(1e-3));
}

// S4a' — a y reset against the RED wall with the beam on a Loader (x 58.8), the season's keep-outs ON:
// REFUSED keep-out, naming the Loader (the most specific span, though the Load Zone holds it too).
// S4b' — KNOWN LIMIT: the same with NO keep-outs (an own line) SEATS ~3.7 in wrong (the Loader
// protrudes 3.74 in) — what D8's keep-outs exist for.
TEST_CASE("L6 T8 S4a'/S4b' beside a Loader: keep-outs ON refuse; OFF seats 3.7 in wrong (known limit)") {
    const Pose2d at{Length{58.8}, Length{yForRightOnRed(7.0)}, Angle::degrees(0.0)};
    {
        WallRig w{at};
        w.idle(0.3);
        const loc::WallResetReport rep = w.reset(ow::kRedWall, loc::WallSide::Right, 4.0);
        CHECK(rep.verdict == WallResetVerdict::KeepOut);
        CHECK(std::string{rep.detail} == "Loader");
        CHECK(w.sink.last().message.find("Loader at along 58.8") != std::string::npos);
    }
    {
        WallRig w{at};
        w.idle(0.3);
        const loc::FieldWall bare = loc::FieldWall::line("red wall", loc::Axis::Y, Length{-ow::kWallInnerFace.value()}, +1);
        const loc::WallResetReport rep = w.reset(bare, loc::WallSide::Right, 4.0);
        INFO("known limit — an object at a plausible distance: the Loader read as the wall");
        CHECK(rep.verdict == WallResetVerdict::Seated);
        CHECK(std::abs(yErr(w) + 3.74) < 0.15);
    }
}

// S5b / S5c (RC42: a lying CUP, and a lying PIN) — pieces below the beam: SEATED within 0.1 in. At
// 10.5 in up (D10's advice) a lying Cup (3.16 in tall), a lying Pin (1.6 in) and a standing Pin 1 in
// off the wall (6.5 in) are all under the cone's lower edge at a 7-in reading.
TEST_CASE("L6 T8 S5b/S5c pieces under the beam: a lying Cup, a lying Pin, a standing Pin — SEATED") {
    struct Piece {
        const char* name;
        FieldBox box;
    };
    const Piece pieces[] = {
        {"lying Cup", FieldBox{-kW, -kW + 6.5, -39.6, -36.4, 0.0, 3.16, false, false, "lying Cup"}},
        {"lying Pin", FieldBox{-kW, -kW + 6.5, -38.8, -37.2, 0.0, 1.6, false, false, "lying Pin"}},
        {"standing Pin 1 in off", FieldBox{-kW + 1.0, -kW + 2.6, -38.8, -37.2, 0.0, 6.5, false, false, "Pin"}},
    };
    for (const Piece& p : pieces) {
        FieldModel f = FieldModel::override2026();
        f.add(p.box);
        WallRig w{parkLeft(), f};
        w.idle(0.3);
        drift(w, loc::Axis::X, 1.0);
        const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
        INFO(p.name);
        CHECK(rep.verdict == WallResetVerdict::Seated);
        CHECK(std::abs(xErr(w)) < 0.1);
    }
}

// S5d (RC34 — no longer a known limit): a standing Pin 1 in off the wall under a sensor 4 in up is
// refused BEAM LOW before any sample: a 4-in lens has no ceiling at all.
TEST_CASE("L6 T8 S5d a 4-in lens with a standing Pin in front: REFUSED beam low") {
    FieldModel f = FieldModel::override2026();
    f.add(FieldBox{-kW + 1.0, -kW + 2.6, -38.8, -37.2, 0.0, 6.5, false, false, "Pin"});
    WallRig w{parkLeft(), f, leftAt(4.0)};
    w.idle(0.3);
    const double t0 = w.h().clock().now().value();
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::BeamLow);
    CHECK(rep.ceiling.value() == 0.0);
    CHECK(w.h().clock().now().value() == t0);
}

// S6 — a read fault mid-window (from the 4th pace of the window): REFUSED read fault, untouched.
TEST_CASE("L6 T8 S6 a read fault mid-window: REFUSED read fault") {
    WallRig w{parkLeft()};
    w.idle(0.3);
    drift(w, loc::Axis::X, 1.0);
    const double x0 = w.loc().pose().x().value();
    w.pacer.faultLeftAt = w.pacer.paces + 4;
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::ReadFault);
    CHECK(rep.sampleIndex == 4);
    CHECK(w.loc().pose().x().value() == x0);
}

// S7a — the sensor mounted BACKWARDS (its true facing -90 deg, the robot file says +90), open field:
// REFUSED. (P2's floorless model said NO RETURN; this plant models the floor, which the cone's lower
// rays meet ~34 in out — OUT OF RANGE. Either way, refused.)
// S7b — KNOWN LIMIT: backwards, with another robot 5 in to its right: SEATED wrong by the parking
// distance minus 5 in (here 7 - 5 = 2.0 in; P2's 8-in parking gave 2.96).
TEST_CASE("L6 T8 S7a/S7b a sensor mounted backwards: refused in the open; a robot 5 in away seats wrong") {
    SideSensorPlantConfig backwards = wall_rig::trueLeft();
    backwards.left = -7.5;
    backwards.facing = -wall_rig::kPi / 2.0;
    {
        WallRig w{parkLeft(), FieldModel::override2026(), backwards};
        w.leftMount = wall_rig::believedFrom(wall_rig::trueLeft());  // the file says LEFT, +90
        w.idle(0.3);
        const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
        CHECK_FALSE(rep.seated());
        CHECK(rep.verdict == WallResetVerdict::OutOfRange);
    }
    {
        FieldModel f = FieldModel::override2026();
        const double lens = kParkX + 7.5;  // the true lens, on the robot's right
        f.add(FieldBox{lens + 5.0, lens + 20.0, -46.0, -30.0, 0.0, 12.0, false, false, "a robot"});
        WallRig w{parkLeft(), f, backwards};
        w.leftMount = wall_rig::believedFrom(wall_rig::trueLeft());
        w.idle(0.3);
        const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
        INFO("known limit — an object at a plausible distance: a robot read as the wall");
        CHECK(rep.verdict == WallResetVerdict::Seated);
        CHECK(std::abs(xErr(w) + 2.0) < 0.15);
    }
}

// S8 — the left wall's panels CLEAR (see-through): REFUSED (P2's floorless model said NO RETURN; this
// plant's cone meets the floor past the panel — OUT OF RANGE). The clear panel never seats here.
TEST_CASE("L6 T8 S8 a clear wall panel with nothing behind it: REFUSED") {
    FieldModel f = FieldModel::override2026();
    f.seeThroughWall("left wall");
    WallRig w{parkLeft(), f};
    w.idle(0.3);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK_FALSE(rep.seated());
    CHECK(rep.verdict == WallResetVerdict::OutOfRange);
}

// RC37's KNOWN LIMIT: a clear RED wall with something opaque 1 or 2 in behind its face SEATS ~1 / ~2
// in wrong under maxCorrection 3; 3.5 and 4 in behind are refused DISAGREES. (Parked 6 in off, so a
// 4-in gap still reads under the 10.77-in beam ceiling and the agreement gate is the one that bites.)
TEST_CASE("L6 T8 RC37 a clear panel with an object close behind seats wrong by the gap (known limit)") {
    for (const double gap : {1.0, 2.0, 3.5, 4.0}) {
        FieldModel f = FieldModel::override2026();
        f.seeThroughWall("red wall");
        f.add(FieldBox{-45.0, -31.0, -kW - gap - 1.0, -kW - gap, 0.0, 30.0, false, false, "a hand"});
        WallRig w{Pose2d{Length{-38.0}, Length{yForRightOnRed(6.0)}, Angle::degrees(0.0)}, f};
        w.idle(0.3);
        const loc::WallResetReport rep = w.reset(ow::kRedWall, loc::WallSide::Right, 3.0);
        INFO("gap " << gap << " — known limit: an object at a plausible distance behind a clear panel");
        if (gap < 3.0) {
            CHECK(rep.verdict == WallResetVerdict::Seated);
            CHECK(std::abs(yErr(w) - gap) < 0.15);  // the panel's object is FARTHER: y reads high
        } else {
            CHECK(rep.verdict == WallResetVerdict::Disagrees);
        }
    }
}

// S9 — sampled while the robot is still driving (12 in/s commanded): REFUSED moving.
TEST_CASE("L6 T8 S9 sampled while driving: REFUSED moving") {
    WallRig w{parkLeft(-42.0)};  // after 0.3 s at 12 in/s the beam is at ~-38 (the clean band)
    w.idle(0.3);
    w.chassis.drive(shulib::math::ChassisSpeeds{shulib::units::Velocity{12.0}, shulib::units::Velocity{0.0},
                                                shulib::units::AngularVelocity{0.0}},
                    shulib::math::Frame::Body);
    w.idle(0.3);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::Moving);
    CHECK(rep.speedSeen.value() > 1.0);
    w.chassis.cancel();
}

// Bug caught (the resumed fix round's N13): the call handing the sampler a turn rate that is not the
// estimate's (0, or a stale one). A robot turning IN PLACE beside the wall moves its centre nowhere,
// so the displacement rule (rule 5's speed) cannot see it — only the turn rate can. Turning at
// 8 deg/s while sampled: REFUSED moving on the turn rate, the speed alone well under its limit.
TEST_CASE("L6 T8 S9b turning in place while sampled: REFUSED moving on the turn rate") {
    WallRig w{parkLeft()};
    w.idle(0.3);
    w.chassis.drive(shulib::math::ChassisSpeeds{shulib::units::Velocity{0.0}, shulib::units::Velocity{0.0},
                                                shulib::units::AngularVelocity{8.0 * kD2R}},
                    shulib::math::Frame::Body);
    w.idle(0.3);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::Moving);
    CHECK(rep.turnSeen.value() > 3.0 * kD2R);
    CHECK(rep.speedSeen.value() < 1.0);  // the centre barely moved: only the turn rate refuses it
    w.chassis.cancel();
}

// S10a — the beam at the wall's centre (the Toggle), 11.5 in up: REFUSED keep-out.
TEST_CASE("L6 T8 S10a the beam on the Toggle: REFUSED keep-out") {
    WallRig w{parkLeft(0.0), FieldModel::override2026(), leftAt(11.5)};
    w.idle(0.3);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::KeepOut);
    CHECK(std::string{rep.detail} == "Toggle");
}

// S11b — the sensor at the fore-aft centre, the estimate's heading 2 deg off: within 0.05 in.
TEST_CASE("L6 T8 S11b a 2-deg heading error with the sensor at the fore-aft centre: within 0.05 in") {
    WallRig w{parkLeft()};
    w.idle(0.3);
    w.loc().reseatHeading(Angle::degrees(92.0));
    drift(w, loc::Axis::X, 1.0);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    REQUIRE(rep.verdict == WallResetVerdict::Seated);
    CHECK(std::abs(xErr(w)) < 0.05);
}

// S12 — the mount TYPED 0.5 in too far out (8.0 for a true 7.5): SEATED off by the typed error,
// 0.5 +/- 0.05 in — the mount passes through 1:1, which is why Step 0.1 measures it.
TEST_CASE("L6 T8 S12 a mount typed 0.5 in wrong seats 0.5 in off") {
    WallRig w{parkLeft()};
    w.leftMount.left = Length{8.0};
    w.idle(0.3);
    drift(w, loc::Axis::X, 1.0);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    REQUIRE(rep.verdict == WallResetVerdict::Seated);
    CHECK(std::abs(xErr(w) - 0.5) < 0.05);
}

// ── RC34: A12's start pieces in the beam, by the lens's height (Q2's B rows) ─────────────────────

// Bug caught: a beam that can reach a standing piece seating against it. The cluster on the left wall
// at along -23.6 (Cup-Pin-Cup, 6.5 in tall); the cone MEASURED (typed = true, 18 deg) so the piece zone
// is lifted and the beam-height gate alone decides: 8 and 9 in up with the wall 8 in off are past the
// ceiling (3.08 / 6.16) — BEAM LOW; 10.5 and 11 in up (10.77 / 12.31) — SEATED within 0.1 in.
TEST_CASE("L6 T8 RC34 the start cluster in the beam: 8/9 in up refused beam low, 10.5/11 in up seated") {
    for (const double h : {8.0, 9.0, 10.5, 11.0}) {
        WallRig w{Pose2d{Length{-70.205 + 7.5 + 8.0}, Length{-23.6}, Angle::degrees(90.0)},
                  FieldModel::override2026(), leftAt(h)};
        w.leftMount.coneHalfAngle = AngleDim{18.0 * loc::kWallDegToRad};
        w.idle(0.3);
        drift(w, loc::Axis::X, 0.5);
        const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
        INFO("lens " << h << " in up");
        if (h < 10.0) {
            CHECK(rep.verdict == WallResetVerdict::BeamLow);
        } else {
            CHECK(rep.verdict == WallResetVerdict::Seated);
            CHECK(std::abs(xErr(w)) < 0.1);
        }
    }
}

// Bug caught: the piece zone lifted without a measured cone. A REAL cone of 24 deg under a typed
// (unmeasured) 18 deg: the piece zone refuses KEEP-OUT before the beam could meet the cluster; and a
// 3-in lens over the floor is BEAM LOW.
TEST_CASE("L6 T8 RC34 an unmeasured cone keeps the start pieces out; a 3-in lens is beam low") {
    SideSensorPlantConfig wide = leftAt(10.5);
    wide.coneHalfAngle = 24.0 * kD2R;
    WallRig w{Pose2d{Length{kParkX}, Length{-23.6}, Angle::degrees(90.0)}, FieldModel::override2026(), wide};
    w.idle(0.3);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::KeepOut);
    CHECK(std::string{rep.detail} == "start pieces");
    WallRig low{parkLeft(), FieldModel::override2026(), leftAt(3.0)};
    low.idle(0.3);
    CHECK(low.reset(ow::kLeftWall, loc::WallSide::Left, 3.0).verdict == WallResetVerdict::BeamLow);
}

// ── RC41: the Toggle — the body lifts with the measured flag, the brackets never ────────────────

// Bug caught (M39): toggleChecked lifting the BRACKETS too. Lens 10 in up, a true and typed 12-deg
// cone, toggleChecked: at mid-Toggle the reset SEATS within 0.1 in; at a bracket (along 11.9 — the
// brackets hang to 10.6 in, 1 in deep) it is KEPT OUT — lifted, it would read the bracket ~1 in short.
TEST_CASE("L6 T8 RC41 with toggleChecked the Toggle body lifts and its brackets do not") {
    SideSensorPlantConfig c = leftAt(10.0);
    c.coneHalfAngle = 12.0 * kD2R;
    for (const double y : {0.0, 11.9}) {
        WallRig w{parkLeft(y), FieldModel::override2026(), c};
        w.leftMount.coneHalfAngle = AngleDim{12.0 * loc::kWallDegToRad};
        w.leftMount.toggleChecked = true;
        w.idle(0.3);
        drift(w, loc::Axis::X, 0.5);
        const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
        INFO("along " << y);
        if (y == 0.0) {
            CHECK(rep.verdict == WallResetVerdict::Seated);
            CHECK(std::abs(xErr(w)) < 0.1);
        } else {
            CHECK(rep.verdict == WallResetVerdict::KeepOut);
            CHECK(std::string{rep.detail} == "Toggle bracket");
        }
    }
}

// ── RC36: the VEX U Load Zones — kept out, the cone measured or not ─────────────────────────────

TEST_CASE("L6 T8 RC36 a beam inside a Load Zone is kept out, the cone measured or not") {
    for (const bool measured : {false, true}) {
        FieldModel f = FieldModel::override2026();
        f.add(FieldBox{48.4, 51.6, -kW, -kW + 3.15, 0.0, 6.5, false, false, "a Cup"});
        WallRig w{Pose2d{Length{50.0}, Length{yForRightOnRed(7.0)}, Angle::degrees(0.0)}, f};
        if (measured) {
            w.rightMount.coneHalfAngle = AngleDim{18.0 * loc::kWallDegToRad};
        }
        w.idle(0.3);
        const loc::WallResetReport rep = w.reset(ow::kRedWall, loc::WallSide::Right, 3.0);
        INFO("cone measured " << measured);
        CHECK(rep.verdict == WallResetVerdict::KeepOut);
        CHECK(std::string{rep.detail} == "Load Zone");
    }
}

// ── T16, RC33: the along-wall drift term on the plant ───────────────────────────────────────────

// Bug caught (M33): the drift term dropped. Q2's case A, on the RED wall with the Loader keep-outs
// only (the draft's spans — the season wall's Load Zone would refuse here whatever the drift, so the
// case isolates rule 3's widening): the truth's beam at x 55 reads the Loader's side, the estimate's
// along is 4 in short (51). Without the drift term the widening clears the Loader and the reset SEATS
// wrong (by ~0.5-1.3 in here — the Loader's SIDE in the cone's edge rays; Q2 saw 1.57 in its own
// model; a clean seat is within 0.1, S1); after 48 or 24 in of travel the drift term (10 %) refuses
// KEEP-OUT. The truth aims at x 55.2 (the estimate 51.2): the leg ends ~0.2 in short.
TEST_CASE("L6 T16 the along drift: a Loader's estimate 4 in off seats wrong without it, refused with it") {
    const std::array<loc::WallKeepOut, 2> loaders{
        loc::WallKeepOut{Length{-ow::kLoaderCentre.value() - ow::kLoaderHalfWidth.value()},
                         Length{-ow::kLoaderCentre.value() + ow::kLoaderHalfWidth.value()},
                         loc::KeepOutKind::Fixed, "Loader"},
        loc::WallKeepOut{Length{ow::kLoaderCentre.value() - ow::kLoaderHalfWidth.value()},
                         Length{ow::kLoaderCentre.value() + ow::kLoaderHalfWidth.value()},
                         loc::KeepOutKind::Fixed, "Loader"}};
    const loc::FieldWall draftRed =
        loc::FieldWall::line("red wall", loc::Axis::Y, Length{-ow::kWallInnerFace.value()}, +1, loaders);
    for (const double legIn : {48.0, 24.0}) {
        for (const double rate : {0.0, 0.10}) {
            const double yy = yForRightOnRed(7.0);
            WallRig w{Pose2d{Length{55.2 - legIn}, Length{yy}, Angle::degrees(0.0)}};
            w.idle(0.3);
            // The estimate starts 4 in short along x (a placement error), then the leg.
            w.loc().setPose(Pose2d{Length{51.2 - legIn}, Length{yy}, w.loc().pose().heading()});
            w.leg(51.2, yy, 0.0);
            REQUIRE(std::abs(w.h().truePose().x().value() - 55.2) < 0.5);
            REQUIRE(w.loc().travelSinceSeat(loc::Axis::X).value() > legIn - 1.0);
            w.limits.alongDriftRate = rate;
            const loc::WallResetReport rep = w.reset(draftRed, loc::WallSide::Right, 3.0);
            INFO("leg " << legIn << " drift rate " << rate);
            if (rate == 0.0) {
                INFO("truth x " << w.h().truePose().x().value() << " y error " << yErr(w));
                CHECK(rep.verdict == WallResetVerdict::Seated);  // the draft's wrong seat
                CHECK(std::abs(yErr(w)) > 0.5);
            } else {
                CHECK(rep.verdict == WallResetVerdict::KeepOut);
                CHECK(rep.alongUncertainty.value() == doctest::Approx(0.1 * w.loc().travelSinceSeat(loc::Axis::X).value()));
            }
        }
    }
}

// Bug caught (T16): the drift term over-refusing after the along axis was just set. On the red wall's
// clean band (x -38) after a 48-in leg along x the drift (4.8 in) widens the beam onto the start
// pieces and the Load Zone — KEEP-OUT, correctly; a seat of x (as a reset against a left/right wall
// would make) restarts the x counter, and the same red-wall reset then SEATS.
TEST_CASE("L6 T16 a reset right after the perpendicular axis was seated is not over-refused") {
    // Nose -x (heading 180): the LEFT sensor faces -y, toward the red wall.
    const double yLeft = -70.205 + 7.5 + 7.0;
    WallRig w{Pose2d{Length{10.0}, Length{yLeft}, Angle::degrees(180.0)}};
    w.idle(0.3);
    w.leg(-38.0, yLeft, 180.0);
    drift(w, loc::Axis::Y, 1.0);
    const loc::WallResetReport first = w.reset(ow::kRedWall, loc::WallSide::Left, 3.0);
    CHECK(first.verdict == WallResetVerdict::KeepOut);
    w.loc().seatAxis(loc::Axis::X, w.h().truePose().x());  // the perpendicular axis, just seated
    w.idle(0.05);
    const loc::WallResetReport second = w.reset(ow::kRedWall, loc::WallSide::Left, 3.0);
    CHECK(second.verdict == WallResetVerdict::Seated);
    CHECK(std::abs(yErr(w)) < 0.1);
}

// Bug caught (M33, the Toggle case): the drift term dropped beside a Toggle bracket. Lens 10 in up, a
// measured 12-deg cone, toggleChecked: the estimate's along is 5.5 in long (18.0 — clear of the
// bracket by the widening alone), the truth's beam at 12.5 meets the bracket. Without the drift term:
// SEATED ~1 in wrong; after 55 in of travel the drift term refuses KEEP-OUT.
TEST_CASE("L6 T16 the along drift beside a Toggle bracket: seated wrong without it, refused with it") {
    SideSensorPlantConfig c = leftAt(10.0);
    c.coneHalfAngle = 12.0 * kD2R;
    for (const double rate : {0.0, 0.10}) {
        WallRig w{Pose2d{Length{kParkX}, Length{-42.5}, Angle::degrees(90.0)}, FieldModel::override2026(), c};
        w.leftMount.coneHalfAngle = AngleDim{12.0 * loc::kWallDegToRad};
        w.leftMount.toggleChecked = true;
        w.idle(0.3);
        w.loc().setPose(Pose2d{Length{kParkX}, Length{-37.0}, w.loc().pose().heading()});
        w.leg(kParkX, 18.0, 90.0);
        REQUIRE(std::abs(w.h().truePose().y().value() - 12.5) < 0.5);
        drift(w, loc::Axis::X, 0.5);
        w.limits.alongDriftRate = rate;
        const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
        INFO("drift rate " << rate);
        if (rate == 0.0) {
            CHECK(rep.verdict == WallResetVerdict::Seated);
            CHECK(std::abs(xErr(w)) > 0.5);
        } else {
            CHECK(rep.verdict == WallResetVerdict::KeepOut);
        }
    }
}
