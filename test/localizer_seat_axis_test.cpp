// Localizer::seatAxis and the wall reset's two counters (chunk L6-A1; the L6 brief's D2 (a), §1.3,
// tests T6, T7 and T16's counter half). Every case names the bug it would catch.
//
// The rig is the estimator alone over fakes the test drives by hand (localizer_reseat_test's shape):
// the IMU reads what the test says, the forward tracking wheel rolls what the test says — so every
// expectation is arithmetic on inputs the test chose, never the estimator grading itself. EVERY seat
// case runs under BOTH fusion tiers: ComplementaryFusion (the Robot's) and EkfFusion — only the EKF
// keeps its own position state, so only it can see a seat that is not re-based (the brief's M19).

#include "doctest.h"

#include <array>
#include <cmath>
#include <limits>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/localization/axis.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/ekf_fusion.hpp"
#include "shulib/localization/fake/fake_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"

using shulib::PreconditionError;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::localization::Axis;
using shulib::localization::ComplementaryFusion;
using shulib::localization::CorrectionProposal;
using shulib::localization::EkfFusion;
using shulib::localization::ICorrector;
using shulib::localization::IFusionPolicy;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TrackingWheel;
using shulib::localization::fake::FakeCorrector;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr double kDt = 0.01;
constexpr double kDeg = Angle::kPi / 180.0;

/// The estimator over hand-driven fakes: the forward wheel has a 2-inch diameter and zero offset,
/// so `tick(d)` rolls exactly d inches along the robot's heading (the IMU's).
struct Rig {
    FakeClock clk;
    FakeImu imu;
    FakeRotation fwdRot;
    FakeRotation latRot;
    PilonsOdometry odom;
    FakeCorrector corrector;
    std::array<ICorrector*, 1> correctors;
    Localizer loc;
    double travel = 0.0;

    explicit Rig(IFusionPolicy& policy, bool wireCorrector = false)
        : odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
               TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          corrector{"fake-gps"},
          correctors{&corrector},
          loc{clk, imu, odom, policy,
              wireCorrector ? std::span<ICorrector* const>{correctors}
                            : std::span<ICorrector* const>{}} {
        imu.setReady(true);
        imu.setYawRate(AngularVelocity{0.0});
    }

    void tick(double fwdInches = 0.0) {
        clk.advance(Time{kDt});
        travel += fwdInches;
        fwdRot.setPosition(AngleDim{travel});
        loc.update();
    }
};

/// Everything seatAxis promises NOT to touch, captured bit for bit (the doubles compared with ==).
struct Untouched {
    double other = 0.0;
    double heading = 0.0;
    double bias = 0.0;
    double drift = 0.0;
    double quality = 0.0;
    Localizer::Quality cls = Localizer::Quality::Uninitialized;
    double lcDx = 0.0;
    double lcDy = 0.0;
    double lcDtheta = 0.0;
    bool lcGated = false;
    bool lcClamped = false;
    double odomX = 0.0;
    double odomY = 0.0;
    double odomTh = 0.0;
    double otherTravel = 0.0;
    double turned = 0.0;
};

Untouched capture(const Rig& r, Axis seated) {
    Untouched u;
    u.other = seated == Axis::X ? r.loc.pose().y().value() : r.loc.pose().x().value();
    u.heading = r.loc.pose().heading().radians();
    u.bias = r.loc.headingBias().value();
    u.drift = r.loc.distanceSinceCorrection().value();
    u.quality = r.loc.quality();
    u.cls = r.loc.qualityClass();
    u.lcDx = r.loc.lastCorrection().dx.value();
    u.lcDy = r.loc.lastCorrection().dy.value();
    u.lcDtheta = r.loc.lastCorrection().dtheta.value();
    u.lcGated = r.loc.lastCorrection().gated;
    u.lcClamped = r.loc.lastCorrection().clamped;
    u.odomX = r.odom.pose().x().value();
    u.odomY = r.odom.pose().y().value();
    u.odomTh = r.odom.pose().heading().radians();
    u.otherTravel = r.loc.travelSinceSeat(seated == Axis::X ? Axis::Y : Axis::X).value();
    u.turned = r.loc.turnedSinceHeadingSeed().value();
    return u;
}

void requireUntouched(const Untouched& a, const Untouched& b) {
    CHECK(a.other == b.other);
    CHECK(a.heading == b.heading);
    CHECK(a.bias == b.bias);
    CHECK(a.drift == b.drift);  // M17: a seat that clears the drift term (setPose's behaviour)
    CHECK(a.quality == b.quality);
    CHECK(a.cls == b.cls);
    CHECK(a.lcDx == b.lcDx);
    CHECK(a.lcDy == b.lcDy);
    CHECK(a.lcDtheta == b.lcDtheta);
    CHECK(a.lcGated == b.lcGated);
    CHECK(a.lcClamped == b.lcClamped);
    CHECK(a.odomX == b.odomX);  // M18b: the odometry shifted (the dropped setPose forward restored)
    CHECK(a.odomY == b.odomY);
    CHECK(a.odomTh == b.odomTh);
    CHECK(a.otherTravel == b.otherTravel);
    CHECK(a.turned == b.turned);
}

/// A live rig facing `headingDeg`, driven `leg` inches along it (so the drift term is past the
/// 12-in horizon for a 24-in leg: Degraded).
void driveLeg(Rig& r, double headingDeg, double leg) {
    r.imu.setHeading(Angle::degrees(headingDeg));
    for (int i = 0; i < 3; ++i) {
        r.tick();
    }
    r.loc.setPose(Pose2d{Length{-40.0}, Length{-30.0}, Angle::degrees(headingDeg)});
    const int steps = static_cast<int>(leg * 10.0 + 0.5);
    for (int i = 0; i < steps; ++i) {
        r.tick(0.1);
    }
    for (int i = 0; i < 3; ++i) {
        r.tick();  // stand still, so the twist is settled at zero
    }
}

/// The seat case, for one fusion tier and one axis (T6).
void seatOneAxisOnly(IFusionPolicy& policy, Axis axis) {
    Rig r{policy};
    // Drive along the axis NOT seated, so the seated axis's value is untouched by the leg.
    driveLeg(r, axis == Axis::X ? 90.0 : 0.0, 24.0);
    REQUIRE(r.loc.qualityClass() == Localizer::Quality::Degraded);
    REQUIRE(r.loc.distanceSinceCorrection().value() > 12.0);

    const Untouched before = capture(r, axis);
    const double was = axis == Axis::X ? r.loc.pose().x().value() : r.loc.pose().y().value();
    const double v = was - 2.0;  // a 2-in seat (RC1's R1 case)
    r.loc.seatAxis(axis, Length{v});

    // The seated axis is EXACTLY the value, on this tick.
    CHECK((axis == Axis::X ? r.loc.pose().x().value() : r.loc.pose().y().value()) == v);
    // Everything else bit-identical AT THE INSTANT seatAxis returns (RC30's scoping).
    requireUntouched(capture(r, axis), before);
    CHECK(r.loc.travelSinceSeat(axis).value() == 0.0);
    CHECK(r.loc.twist().vx().value() == 0.0);
    CHECK(r.loc.twist().vy().value() == 0.0);

    // The next tick is the dt-0 re-base: Degraded, the drift term's floor 0.2 halved = 0.100; the
    // second tick Degraded 0.200 (one axis measured is not a fix — RC30's per-tick values).
    r.tick();
    CHECK(r.loc.qualityClass() == Localizer::Quality::Degraded);
    CHECK(r.loc.quality() == doctest::Approx(0.100).epsilon(1e-12));
    // No velocity spike: the filter did NOT read the seat as one tick's travel (M19 under the EKF:
    // 1.68 in of a 2-in seat bounced back and the velocity read 167.6 in/s).
    CHECK(std::abs(r.loc.twist().vx().value()) < 1e-9);
    CHECK(std::abs(r.loc.twist().vy().value()) < 1e-9);
    CHECK((axis == Axis::X ? r.loc.pose().x().value() : r.loc.pose().y().value())
          == doctest::Approx(v).epsilon(1e-12));
    r.tick();
    CHECK(r.loc.qualityClass() == Localizer::Quality::Degraded);
    CHECK(r.loc.quality() == doctest::Approx(0.200).epsilon(1e-12));
    for (int i = 0; i < 20; ++i) {
        r.tick();
    }
    CHECK(std::abs(r.loc.twist().vx().value()) < 1e-9);
    CHECK(std::abs(r.loc.twist().vy().value()) < 1e-9);
    // The seat persists: standing still, the axis stays where it was seated (no bounce back).
    CHECK((axis == Axis::X ? r.loc.pose().x().value() : r.loc.pose().y().value())
          == doctest::Approx(v).epsilon(1e-9));
    CHECK(r.loc.distanceSinceCorrection().value() == before.drift);
}

/// T6's "a following 10-in drive moves the estimate by the odometry's 10 in only": two rigs driven
/// identically; one is seated 3 in off, its TWIN is seated to the value it already had (the same
/// dt-0 re-base, no jump). After a 10-in drive ALONG the seated axis the two differ by exactly the
/// seat — the seat moved the estimate and nothing else, and the drive after it is the drive the twin
/// saw (an EKF that read the seat as travel, M19, would not hold the 3 in).
void seatThenDriveAlong(IFusionPolicy& policy, IFusionPolicy& twinPolicy, Axis axis) {
    Rig r{policy};
    Rig twin{twinPolicy};
    driveLeg(r, axis == Axis::X ? 0.0 : 90.0, 6.0);
    driveLeg(twin, axis == Axis::X ? 0.0 : 90.0, 6.0);
    const auto on = [axis](const Rig& g) {
        return axis == Axis::X ? g.loc.pose().x().value() : g.loc.pose().y().value();
    };
    REQUIRE(on(r) == on(twin));
    r.loc.seatAxis(axis, Length{on(r) + 3.0});
    twin.loc.seatAxis(axis, Length{on(twin)});
    for (int i = 0; i < 100; ++i) {
        r.tick(0.1);  // 10 in along the heading = along the seated axis
        twin.tick(0.1);
        CHECK(on(r) - on(twin) == doctest::Approx(3.0).epsilon(1e-9));
    }
    for (int i = 0; i < 3; ++i) {
        r.tick();
        twin.tick();
    }
    CHECK(on(r) - on(twin) == doctest::Approx(3.0).epsilon(1e-9));
}

}  // namespace

// Bug caught: a seat that moves anything but its one axis — the drift term cleared (M17, setPose's
// behaviour: a one-axis measurement passing as a two-axis one), the odometry shifted (M18b), the
// other axis or the heading touched, the quality bumped — or a seat the EKF reads as one tick's
// travel because the next tick was not re-based (M19, red ONLY under EkfFusion).
TEST_CASE("L6 T6 seatAxis(X) moves x and nothing else — ComplementaryFusion") {
    ComplementaryFusion fusion;
    seatOneAxisOnly(fusion, Axis::X);
}
TEST_CASE("L6 T6 seatAxis(X) moves x and nothing else — EkfFusion") {
    EkfFusion fusion;
    seatOneAxisOnly(fusion, Axis::X);
}
TEST_CASE("L6 T6 seatAxis(Y) moves y and nothing else — ComplementaryFusion") {
    ComplementaryFusion fusion;
    seatOneAxisOnly(fusion, Axis::Y);
}
TEST_CASE("L6 T6 seatAxis(Y) moves y and nothing else — EkfFusion") {
    EkfFusion fusion;
    seatOneAxisOnly(fusion, Axis::Y);
}

// Bug caught: a seat the next drive undoes or doubles — the predictor re-based onto the OLD value
// (the odometry's absolute pose re-read), or the seat counted as travel.
TEST_CASE("L6 T6 after a seat, a 10-in drive moves the seated axis by the odometry's 10 in only") {
    ComplementaryFusion c1;
    ComplementaryFusion c1t;
    seatThenDriveAlong(c1, c1t, Axis::X);
    EkfFusion e1;
    EkfFusion e1t;
    seatThenDriveAlong(e1, e1t, Axis::X);
    ComplementaryFusion c2;
    ComplementaryFusion c2t;
    seatThenDriveAlong(c2, c2t, Axis::Y);
    EkfFusion e2;
    EkfFusion e2t;
    seatThenDriveAlong(e2, e2t, Axis::Y);
}

// Bug caught: the D2 (b) overclaim, pinned per tick (RC30) — setPose with the other axis copied
// reads DeadReckon 1.000 from the second tick (the drift term cleared), where the one-axis seat
// stays Degraded 0.200. If seatAxis ever starts behaving like setPose, the first half of T6 turns
// red; this case pins the contrast so a change in setPose is seen too.
TEST_CASE("L6 T6 the contrast: setPose with the other axis copied clears the drift term (D2 (b))") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    driveLeg(r, 90.0, 24.0);
    const Pose2d e = r.loc.pose();
    r.loc.setPose(Pose2d{e.x() - Length{2.0}, e.y(), e.heading()});
    CHECK(r.loc.distanceSinceCorrection().value() == 0.0);
    r.tick();
    CHECK(r.loc.qualityClass() == Localizer::Quality::Degraded);
    CHECK(r.loc.quality() == doctest::Approx(0.500).epsilon(1e-12));
    r.tick();
    CHECK(r.loc.qualityClass() == Localizer::Quality::DeadReckon);
    CHECK(r.loc.quality() == doctest::Approx(1.000).epsilon(1e-12));
}

// Bug caught: a seat computed while the estimate is boot garbage (M20 — the live check removed),
// or a non-finite value written into the fused position.
TEST_CASE("L6 T7 seatAxis refuses a not-live estimate and a non-finite value") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setReady(false);
    r.tick();
    r.tick();
    REQUIRE(r.loc.qualityClass() == Localizer::Quality::Uninitialized);
    CHECK_THROWS_WITH_AS(r.loc.seatAxis(Axis::X, Length{1.0}),
                         doctest::Contains("not live yet"), PreconditionError);
    r.imu.setReady(true);
    for (int i = 0; i < 20; ++i) {
        r.tick();
    }
    REQUIRE(r.loc.qualityClass() != Localizer::Quality::Uninitialized);
    CHECK_THROWS_AS(r.loc.seatAxis(Axis::Y, Length{std::nan("")}), PreconditionError);
    CHECK_THROWS_AS(r.loc.seatAxis(Axis::X, Length{std::numeric_limits<double>::infinity()}), PreconditionError);
    CHECK_NOTHROW(r.loc.seatAxis(Axis::X, Length{1.0}));
}

// Bug caught (T16, RC33): an along-wall counter that does not grow with travel, that a seat of the
// OTHER axis resets (M34 — a reset against the left wall would stop the red wall's keep-outs from
// widening), that setPose does not reset, or that a corrector's fix resets. The counters move in
// 0.5-in STEPS of the odometry's path (the L6-A1 review: jitter must not add up), so a leg reads its
// length to within one step and one tick — never more than the path, never less by more than that.
TEST_CASE("L6 T16 travelSinceSeat: grows by path length, setPose zeroes both, seatAxis its own only") {
    ComplementaryFusion fusion;
    Rig r{fusion, /*wireCorrector=*/true};
    const double step = Localizer::kTravelCounterStepInches;
    CHECK(step == 0.5);
    r.imu.setHeading(Angle::degrees(30.0));
    for (int i = 0; i < 3; ++i) {
        r.tick();
    }
    r.loc.setPose(Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(30.0)});
    CHECK(r.loc.travelSinceSeat(Axis::X).value() == 0.0);
    CHECK(r.loc.travelSinceSeat(Axis::Y).value() == 0.0);
    for (int i = 0; i < 100; ++i) {
        r.tick(0.1);  // 10 in of path, diagonal
    }
    const double x10 = r.loc.travelSinceSeat(Axis::X).value();
    CHECK(x10 <= 10.0 + 1e-9);
    CHECK(x10 >= 10.0 - step - 0.1);
    CHECK(r.loc.travelSinceSeat(Axis::Y).value() == x10);
    CHECK(x10 <= r.loc.distanceSinceCorrection().value() + 1e-9);  // the drift term sums every tick

    r.loc.seatAxis(Axis::X, r.loc.pose().x());
    CHECK(r.loc.travelSinceSeat(Axis::X).value() == 0.0);
    CHECK(r.loc.travelSinceSeat(Axis::Y).value() == x10);  // M34: the OTHER axis's counter untouched
    for (int i = 0; i < 50; ++i) {
        r.tick(0.1);
    }
    const double x5 = r.loc.travelSinceSeat(Axis::X).value();
    CHECK(x5 >= 5.0 - step - 0.1);
    CHECK(x5 <= 5.0 + step + 1e-9);  // the step the seat landed in may count to x: over, never under
    CHECK(r.loc.travelSinceSeat(Axis::Y).value() == doctest::Approx(x10 + x5).epsilon(1e-12));

    const double yBeforeSeat = r.loc.travelSinceSeat(Axis::Y).value();
    REQUIRE(yBeforeSeat > 0.0);
    r.loc.seatAxis(Axis::Y, r.loc.pose().y());
    CHECK(r.loc.travelSinceSeat(Axis::Y).value() == 0.0);
    CHECK(r.loc.travelSinceSeat(Axis::X).value() == x5);

    // A corrector's applied fix scales the drift term but zeroes NEITHER counter (conservative).
    for (int i = 0; i < 30; ++i) {
        r.tick(0.1);
    }
    const double xBefore = r.loc.travelSinceSeat(Axis::X).value();
    const double yBefore = r.loc.travelSinceSeat(Axis::Y).value();
    REQUIRE(yBefore > 0.0);
    const double driftBefore = r.loc.distanceSinceCorrection().value();
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = r.loc.pose();
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    r.corrector.setProposal(p);
    r.tick();
    REQUIRE(r.loc.distanceSinceCorrection().value() < driftBefore);  // the fix WAS applied
    CHECK(r.loc.travelSinceSeat(Axis::X).value() == xBefore);
    CHECK(r.loc.travelSinceSeat(Axis::Y).value() == yBefore);

    r.loc.setPose(r.loc.pose());
    CHECK(r.loc.travelSinceSeat(Axis::X).value() == 0.0);
    CHECK(r.loc.travelSinceSeat(Axis::Y).value() == 0.0);
    // "Since the seed": the step in progress at setPose is dropped — 0.4 in after it reads 0, and
    // 0.6 in after it reads 0.6 (never the pre-seed remainder on top).
    r.tick(0.2);
    r.tick(0.2);
    CHECK(r.loc.travelSinceSeat(Axis::X).value() == 0.0);
    r.tick(0.2);
    CHECK(r.loc.travelSinceSeat(Axis::X).value() == doctest::Approx(0.6).epsilon(1e-9));
}

// Bug caught (the L6-A1 review's finding, carried to the travel counters): a still robot's ODOMETRY
// jitter added up as travel. Back and forth 0.2 in a tick for 20 s is 400 in of per-tick |delta| (the
// drift term, which has always summed every tick, reads exactly that) — but the robot went nowhere,
// and the wall reset's keep-outs must not widen by 40 in. Counted in 0.5-in steps: 0.
TEST_CASE("L6 T16 travelSinceSeat ignores back-and-forth jitter inside its 0.5-in step") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setHeading(Angle::degrees(0.0));
    r.tick();
    r.tick();
    r.loc.setPose(Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(0.0)});
    for (int i = 0; i < 2000; ++i) {
        r.tick((i % 2) == 0 ? 0.2 : -0.2);
    }
    CHECK(r.loc.distanceSinceCorrection().value() == doctest::Approx(400.0).epsilon(1e-9));
    CHECK(r.loc.travelSinceSeat(Axis::X).value() == 0.0);
    CHECK(r.loc.travelSinceSeat(Axis::Y).value() == 0.0);
}

// Bug caught (T16, RC38): a turn counter that sums SIGNED changes (M35 — a left-right wiggle reads
// 0, so a robot that has turned a lot seats as if it had not), that unwraps wrongly at +/-180 deg
// (a 350-degree spin reading 10), or that a heading seed does not restart.
TEST_CASE("L6 T16 turnedSinceHeadingSeed sums |dtheta| through the wrap; seeds restart it") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setHeading(Angle::degrees(170.0));
    for (int i = 0; i < 3; ++i) {
        r.tick();
    }
    r.loc.setPose(Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(170.0)});
    CHECK(r.loc.turnedSinceHeadingSeed().value() == 0.0);
    // A 350-degree spin CCW, 5 degrees a tick, through the +/-180 wrap.
    double h = 170.0;
    for (int i = 0; i < 70; ++i) {
        h += 5.0;
        r.imu.setHeading(Angle::degrees(h));
        r.tick();
    }
    CHECK(r.loc.turnedSinceHeadingSeed().value() == doctest::Approx(350.0 * kDeg).epsilon(1e-9));

    // A left-right wiggle: +10, -10, ten times — 200 degrees turned, net zero.
    r.loc.reseatHeading(r.loc.pose().heading());
    CHECK(r.loc.turnedSinceHeadingSeed().value() == 0.0);
    for (int i = 0; i < 10; ++i) {
        r.imu.setHeading(Angle::degrees(h + 10.0));
        r.tick();
        r.imu.setHeading(Angle::degrees(h));
        r.tick();
    }
    CHECK(r.loc.turnedSinceHeadingSeed().value() == doctest::Approx(200.0 * kDeg).epsilon(1e-9));

    // seatAxis does NOT restart it (only a heading seed does).
    r.loc.seatAxis(Axis::X, r.loc.pose().x());
    CHECK(r.loc.turnedSinceHeadingSeed().value() == doctest::Approx(200.0 * kDeg).epsilon(1e-9));
    r.loc.setPose(r.loc.pose());
    CHECK(r.loc.turnedSinceHeadingSeed().value() == 0.0);
    // From the seed's own reading: a 7-degree turn after it reads 7, not 7 + anything before.
    r.imu.setHeading(Angle::degrees(h + 7.0));
    r.tick();
    CHECK(r.loc.turnedSinceHeadingSeed().value() == doctest::Approx(7.0 * kDeg).epsilon(1e-9));
}

// Bug caught (T16): READING a counter changing the estimate (an accessor with a side effect). Two rigs
// driven identically, one of them read through the new accessors every tick, must agree bit for bit
// — and a boot window (fold closed) adds nothing to either counter. (That the counters change no
// EXISTING observable is the whole suite's evidence, not this case's: every Localizer case of the
// base passes unchanged, and a trace of pose, quality and drift over one drive is bit-identical
// between the base tree and this one — L6-PROGRESS.md records the comparison.)
TEST_CASE("L6 T16 reading the counters has no side effect, and the boot fold keeps them at 0") {
    ComplementaryFusion f1;
    ComplementaryFusion f2;
    Rig a{f1};
    Rig b{f2};
    a.imu.setReady(false);
    b.imu.setReady(false);
    for (int i = 0; i < 5; ++i) {
        a.imu.setHeading(Angle::degrees(10.0 * i));  // boot garbage that moves
        b.imu.setHeading(Angle::degrees(10.0 * i));
        a.tick(0.3);
        b.tick(0.3);
    }
    CHECK(a.loc.turnedSinceHeadingSeed().value() == 0.0);
    CHECK(a.loc.travelSinceSeat(Axis::X).value() == 0.0);
    a.imu.setReady(true);
    b.imu.setReady(true);
    for (int i = 0; i < 200; ++i) {
        const double hd = 40.0 + 0.5 * i;
        a.imu.setHeading(Angle::degrees(hd));
        b.imu.setHeading(Angle::degrees(hd));
        a.tick(0.05);
        b.tick(0.05);
        (void)a.loc.travelSinceSeat(Axis::X);
        (void)a.loc.turnedSinceHeadingSeed();
        CHECK(a.loc.pose().x().value() == b.loc.pose().x().value());
        CHECK(a.loc.pose().y().value() == b.loc.pose().y().value());
        CHECK(a.loc.quality() == b.loc.quality());
    }
    CHECK(a.loc.turnedSinceHeadingSeed().value() > 0.0);
}

// Bug caught (the L6-A1 review): the turn counter adding up a STILL IMU's heading noise. Summing every
// tick's |change| turned the register's HA-21 jitter (~0.05 deg white) into ~5.6 deg of "turning" a
// second, so a robot that stood still long enough was refused imprecise. Counted in 1-degree STEPS
// from an anchor, a jitter of +/-0.4 deg for 20 s reads 0 — and the step's size is pinned: a 0.9-deg
// turn reads 0, a 1.1-deg turn reads 1.1, and a slow 90-deg turn (0.1 deg a tick) reads all but its
// last uncounted step: 88.8-90 deg (a step plus a tick).
TEST_CASE("L6 T16 the turn counter ignores jitter under its 1-deg step and counts a slow turn") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setHeading(Angle::degrees(90.0));
    r.tick();
    r.tick();
    r.loc.reseatHeading(Angle::degrees(90.0));
    for (int i = 0; i < 2000; ++i) {  // 20 s of +/-0.4 deg jitter, every tick a 0.8-deg change
        r.imu.setHeading(Angle::degrees(90.0 + ((i % 2) == 0 ? 0.4 : -0.4)));
        r.tick();
    }
    CHECK(r.loc.turnedSinceHeadingSeed().value() == 0.0);
    CHECK(Localizer::kTurnCounterStepRadians == 1.0 * kDeg);
    // A 0.9-deg turn is under the step: not counted; on to 1.1 deg from the anchor: counted whole.
    r.loc.reseatHeading(r.loc.pose().heading());
    r.imu.setHeading(Angle::degrees(90.0));
    r.tick();
    r.loc.reseatHeading(r.loc.pose().heading());
    r.imu.setHeading(Angle::degrees(90.9));
    r.tick();
    CHECK(r.loc.turnedSinceHeadingSeed().value() == 0.0);
    r.imu.setHeading(Angle::degrees(91.1));
    r.tick();
    CHECK(r.loc.turnedSinceHeadingSeed().value() == doctest::Approx(1.1 * kDeg).epsilon(1e-9));
    // A slow 90-deg turn, 0.1 deg a tick: counted to within one step (plus a tick of rounding).
    r.loc.reseatHeading(r.loc.pose().heading());
    double h = 91.1;
    for (int i = 0; i < 900; ++i) {
        h += 0.1;
        r.imu.setHeading(Angle::degrees(h));
        r.tick();
    }
    CHECK(r.loc.turnedSinceHeadingSeed().value() <= 90.0 * kDeg + 1e-9);
    CHECK(r.loc.turnedSinceHeadingSeed().value() >= 88.8 * kDeg);
}

// Bug caught (the L6-A1 review's L1/L2): the turn counter counting ACROSS the boot boundary — (L1)
// the first tick with the fold open measured against the last BOOT-GARBAGE reading, or (L2) a heading
// seed taken while booting treated as a live anchor. The last closed tick's raw heading is 70 deg from
// the first open tick's; the counter must read exactly 0 after that tick (the robot never turned).
TEST_CASE("L6 T16 the turn counter never counts across the boot boundary, seeded while booting or not") {
    // Find, on a twin, the tick on which the fold first opens (deterministic: the same clock steps).
    const auto firstOpenTick = [] {
        ComplementaryFusion f;
        Rig t{f};
        t.imu.setReady(false);
        for (int i = 0; i < 5; ++i) {
            t.tick();
        }
        t.imu.setReady(true);
        for (int i = 0; i < 100; ++i) {
            t.tick();
            if (t.loc.qualityClass() != Localizer::Quality::Uninitialized) {
                return i;
            }
        }
        return -1;
    }();
    REQUIRE(firstOpenTick > 0);
    for (const bool seedWhileBooting : {false, true}) {
        ComplementaryFusion f;
        Rig r{f};
        r.imu.setReady(false);
        r.imu.setHeading(Angle::degrees(170.0));  // boot garbage
        for (int i = 0; i < 5; ++i) {
            r.tick();
        }
        r.imu.setReady(true);
        for (int i = 0; i < firstOpenTick; ++i) {
            r.imu.setHeading(Angle::degrees((i % 2) == 0 ? 170.0 : 140.0));  // still garbage, moving
            r.tick();
            REQUIRE(r.loc.qualityClass() == Localizer::Quality::Uninitialized);
        }
        if (seedWhileBooting) {
            r.loc.setPose(Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(0.0)});  // on the LAST closed tick
        }
        r.imu.setHeading(Angle::degrees(r.imu.heading().degrees() - 70.0));  // the live reading
        r.tick();
        INFO("seeded while booting: " << seedWhileBooting);
        REQUIRE(r.loc.qualityClass() != Localizer::Quality::Uninitialized);  // the first open tick
        CHECK(r.loc.turnedSinceHeadingSeed().value() == 0.0);
        for (int i = 0; i < 10; ++i) {
            r.tick();  // still: nothing more
        }
        CHECK(r.loc.turnedSinceHeadingSeed().value() == 0.0);
    }
}
