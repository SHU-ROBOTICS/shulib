// Localizer::reseatHeading — the heading twin of setPose (chunk COMP-a; the COMP brief's ruling C2
// and amendment B14). Every case names the bug it would catch.
//
// The rig is the estimator alone over fakes the test drives by hand (the EKF seam suite's shape):
// the IMU reads what the test says, the forward tracking wheel rolls what the test says. So every
// expectation below is arithmetic on inputs the test chose, never the estimator grading itself.

#include "doctest.h"

#include <array>
#include <cmath>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
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
using shulib::localization::ComplementaryFusion;
using shulib::localization::ComplementaryFusionConfig;
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

/// The estimator over hand-driven fakes. The forward wheel has a 1-inch radius (2-inch diameter)
/// and zero offset, so `roll(d)` is exactly d inches of forward travel.
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
          corrector{"fake-tag"},
          correctors{&corrector},
          loc{clk, imu, odom, policy,
              wireCorrector ? std::span<ICorrector* const>{correctors}
                            : std::span<ICorrector* const>{}} {
        imu.setReady(true);
        imu.setYawRate(AngularVelocity{0.0});
    }

    /// One tick, rolling `fwdInches` forward along the robot's CURRENT heading.
    void tick(double fwdInches = 0.0) {
        clk.advance(Time{kDt});
        travel += fwdInches;
        fwdRot.setPosition(AngleDim{travel});
        loc.update();
    }
};

}  // namespace

// Bug caught: a re-seat that does not land — or lands only on this tick's pose and evaporates at
// the next update (the M2 red team's "corrections not accumulating", in the seed's clothes). The
// IMU reads 0 deg while the robot is declared to face 90 deg: after the seat, and after ticks of
// standing still, the published heading must be 90 deg; and the IMU stays the ONLY source of
// heading CHANGE — a 30 deg turn it reports moves the published heading by exactly 30 deg.
TEST_CASE("C2 reseatHeading: the published heading is the seed, and the IMU still moves it") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setHeading(Angle::degrees(0.0));
    for (int i = 0; i < 5; ++i) {
        r.tick();
    }
    REQUIRE(r.loc.qualityClass() != Localizer::Quality::Uninitialized);

    r.loc.reseatHeading(Angle::degrees(90.0));
    CHECK(r.loc.pose().heading().approxEqual(Angle::degrees(90.0), 1e-12));  // this tick
    for (int i = 0; i < 20; ++i) {
        r.tick();
    }
    CHECK(r.loc.pose().heading().approxEqual(Angle::degrees(90.0), 1e-12));  // and it persists
    CHECK(r.loc.headingBias().value() == doctest::Approx(90.0 * kDeg).epsilon(1e-12));

    r.imu.setHeading(Angle::degrees(30.0));  // the robot turns 30 deg left
    r.tick();
    CHECK(r.loc.pose().heading().approxEqual(Angle::degrees(120.0), 1e-12));
}

// Bug caught: a re-seat that fixes the REPORTED heading while the position prediction keeps the
// old one — the odometry rotates each delta by the RAW IMU heading, so without the bias's STEP 2
// re-aim the robot would believe it drove along +x while it drove along +y (the cross-track error
// E3 found, now at 90 deg). Seated at 90 deg with the IMU at 0 deg, 10 in of forward roll must
// land on +y.
TEST_CASE("C2 reseatHeading: position follows the seated heading from the next tick") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setHeading(Angle::degrees(0.0));
    for (int i = 0; i < 3; ++i) {
        r.tick();
    }
    r.loc.setPose(Pose2d{Length{-48.0}, Length{-24.0}, Angle{}});
    r.loc.reseatHeading(Angle::degrees(90.0));
    for (int i = 0; i < 100; ++i) {
        r.tick(0.1);  // 10 in total
    }
    CHECK(r.loc.pose().x().value() == doctest::Approx(-48.0).epsilon(1e-9));
    CHECK(r.loc.pose().y().value() == doctest::Approx(-14.0).epsilon(1e-9));
}

// Bug caught: a re-seat computed from calibration garbage. While the IMU is still booting the
// estimate is Uninitialized, and the seed must be REFUSED loudly (competition::runAuton waits for a
// live estimate before it calls this — B12) rather than baking a moving garbage reading into a
// bias that would be wrong the instant calibration ends.
TEST_CASE("B12 reseatHeading refuses an estimate that is not live yet") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setReady(false);
    r.tick();
    REQUIRE(r.loc.qualityClass() == Localizer::Quality::Uninitialized);
    CHECK_THROWS_AS(r.loc.reseatHeading(Angle::degrees(90.0)), PreconditionError);
    CHECK(r.loc.headingBias().value() == 0.0);  // nothing was written
}

// Bug caught: setPose() discarding the seeded heading. E3 made setPose keep a LEARNED bias; the
// seed lives in the same bias, so a Routine::startAt() or a route's Start leg run after the
// competition runner's re-seat (a route entry does exactly that, one tick later) must not undo it.
TEST_CASE("C2 setPose after a re-seat keeps the seated heading") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setHeading(Angle::degrees(10.0));
    r.tick();
    r.tick();  // live from the SECOND update (the first only establishes the estimate)
    r.loc.reseatHeading(Angle::degrees(-135.0));
    r.tick();
    r.loc.setPose(Pose2d{Length{12.0}, Length{3.0}, Angle::degrees(0.0)});  // heading ignored
    CHECK(r.loc.pose().heading().approxEqual(Angle::degrees(-135.0), 1e-12));
    r.tick();
    CHECK(r.loc.pose().heading().approxEqual(Angle::degrees(-135.0), 1e-12));
}

// Bug caught: a seed that REPLACES the correction path instead of composing with it. After the
// re-seat, a heading-providing corrector that sees the robot 3 deg off must still NUDGE the bias
// — bounded per tick by maxHeadingNudgeRate (never a snap) — and converge onto the absolute
// heading, starting FROM the seed.
TEST_CASE("C2 the seeded bias composes with a later heading correction (bounded, never a snap)") {
    ComplementaryFusionConfig cfg;
    ComplementaryFusion fusion{cfg};
    Rig r{fusion, /*wireCorrector=*/true};
    r.imu.setHeading(Angle::degrees(5.0));
    r.tick();
    r.tick();
    r.loc.reseatHeading(Angle::degrees(90.0));
    r.tick();
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{r.loc.pose().x(), r.loc.pose().y(), Angle::degrees(93.0)};
    p.confidence = 1.0;
    p.positionStdDev = Length{1.0};
    p.providesHeading = true;
    r.corrector.setProposal(p);
    const double maxStep = cfg.maxHeadingNudgeRate.value() * kDt;
    double prev = r.loc.pose().heading().radians();
    double worstStep = 0.0;
    for (int i = 0; i < 400; ++i) {
        r.tick();
        const double now = r.loc.pose().heading().radians();
        worstStep = std::max(worstStep, std::abs(now - prev));
        prev = now;
    }
    CHECK(worstStep <= maxStep + 1e-12);  // never a snap
    CHECK(r.loc.pose().heading().approxEqual(Angle::degrees(93.0), 0.2 * kDeg));  // converged
}

// Bug caught (B14, the load-bearing one): a re-seat that does NOT re-baseline like setPose. The
// EKF sizes its heading process noise from the change in the handed heading; on an ordinary tick
// a 90 deg seed reads as 90 deg of physical rotation and inflates P(theta, theta) — the filter then
// trusts a later heading fix it should have weighed, for the rest of the run. With the re-baseline
// the next tick is a dt-0 RESYNC: no process noise at all, the heading re-based. Compared against
// a twin that was not re-seated and took the same tick.
TEST_CASE("B14 an EkfFusion localizer re-seated by 90 deg shows no heading process-noise spike") {
    EkfFusion seatedFilter;
    EkfFusion twinFilter;
    Rig seated{seatedFilter};
    Rig twin{twinFilter};
    for (Rig* r : {&seated, &twin}) {
        r->imu.setHeading(Angle::degrees(0.0));
        for (int i = 0; i < 50; ++i) {
            r->tick(0.05);
        }
    }
    const double pBefore = seatedFilter.covariance(EkfFusion::kTh, EkfFusion::kTh);
    REQUIRE(pBefore == twinFilter.covariance(EkfFusion::kTh, EkfFusion::kTh));
    const auto resyncsBefore = seatedFilter.resyncCount();

    seated.loc.reseatHeading(Angle::degrees(90.0));
    seated.tick(0.05);
    twin.tick(0.05);

    const double pSeated = seatedFilter.covariance(EkfFusion::kTh, EkfFusion::kTh);
    const double pTwin = twinFilter.covariance(EkfFusion::kTh, EkfFusion::kTh);
    CHECK(seatedFilter.resyncCount() == resyncsBefore + 1);  // the dt-0 re-base happened
    CHECK(pSeated <= pTwin);  // no 90-degree "rotation" was charged as process noise
    // A spike would be at least (headingNoisePerRad * pi/2)^2 above the twin — make the margin
    // explicit so the mutation's size is on the page.
    const double spike = 0.01 * (Angle::kPi / 2.0);
    CHECK(pSeated < pTwin + 0.5 * spike * spike);
    // …and the filter keeps tracking: the published heading is the seed.
    CHECK(seated.loc.pose().heading().approxEqual(Angle::degrees(90.0), 1e-12));
}

// Bug caught (the COMP-a review's finding 5b): a re-seat that re-baselines dt but leaves the OLD
// field-frame velocity in place. The tick after a re-seat is a dt-0 tick, and a dt-0 tick KEEPS
// the previous linear velocity (update()'s twist rule) — so a stale twist measured in the pre-seat
// frame (10 in/s along +x, while the seated robot is driving along +y) would be published for a
// tick as if it belonged to the new one, feeding a motion's velocity term the wrong direction.
// setPose() clears the twist for the same reason; reseatHeading() must too. Built on a MOVING
// estimate, turning as well, so every component of the stale twist is non-zero.
TEST_CASE("C2 reseatHeading clears the twist, and the dt-0 tick after it keeps it clear") {
    ComplementaryFusion fusion;
    Rig r{fusion};
    r.imu.setHeading(Angle::degrees(0.0));
    r.imu.setYawRate(AngularVelocity{0.25});
    for (int i = 0; i < 10; ++i) {
        r.tick(0.1);  // 10 in/s along +x
    }
    REQUIRE(r.loc.qualityClass() != Localizer::Quality::Uninitialized);
    REQUIRE(r.loc.twist().vx().value() == doctest::Approx(10.0).epsilon(1e-9));
    REQUIRE(r.loc.twist().omega().value() == 0.25);

    r.loc.reseatHeading(Angle::degrees(90.0));
    CHECK(r.loc.twist().vx().value() == 0.0);
    CHECK(r.loc.twist().vy().value() == 0.0);
    CHECK(r.loc.twist().omega().value() == 0.0);

    r.tick(0.1);  // the dt-0 re-base tick: linear velocity KEPT (cleared), omega refreshed
    CHECK(r.loc.twist().vx().value() == 0.0);
    CHECK(r.loc.twist().vy().value() == 0.0);
    CHECK(r.loc.twist().omega().value() == 0.25);

    r.tick(0.1);  // an ordinary tick: the velocity is back, along the SEATED heading
    CHECK(std::abs(r.loc.twist().vx().value()) < 1e-9);
    CHECK(r.loc.twist().vy().value() == doctest::Approx(10.0).epsilon(1e-9));
}

// Bug caught (C2's "GPS and tag corrections still compose", the position half — the COMP-a
// review's finding 5c): a seed that breaks the POSITION correction path, or a position-only fix
// that moves the seeded heading. After a 90 deg seat over an IMU reading 5 deg, a GPS-like
// corrector (providesHeading = false) reports the robot 3 in east and 2 in south of the estimate.
// Its heading field says 93 deg — inside every heading gate, so any code that wrongly READ it would
// visibly pull the heading. On both fusion tiers: the position converges onto the fix, and the
// heading stays EXACTLY the seed on every tick — a position fix has no business rotating the
// robot, seeded or not.
TEST_CASE("C2 a position-only (GPS-like) fix after a seat moves the position, never the heading") {
    ComplementaryFusion complementary;
    EkfFusion ekf;
    IFusionPolicy* const policies[] = {&complementary, &ekf};
    for (IFusionPolicy* policy : policies) {
        CAPTURE(policy == &ekf);
        Rig r{*policy, /*wireCorrector=*/true};
        r.imu.setHeading(Angle::degrees(5.0));
        r.tick();
        r.tick();
        r.loc.setPose(Pose2d{Length{-48.0}, Length{-24.0}, Angle{}});
        r.loc.reseatHeading(Angle::degrees(90.0));
        const double seededBias = r.loc.headingBias().value();
        REQUIRE(seededBias == doctest::Approx(85.0 * kDeg).epsilon(1e-12));
        r.tick();

        CorrectionProposal p{};
        p.valid = true;
        p.fieldPose = Pose2d{Length{-45.0}, Length{-26.0}, Angle::degrees(93.0)};  // NOT read
        p.confidence = 1.0;
        p.positionStdDev = Length{1.0};
        p.providesHeading = false;
        r.corrector.setProposal(p);
        bool headingHeld = true;
        for (int i = 0; i < 600; ++i) {
            r.tick();
            headingHeld = headingHeld
                          && r.loc.pose().heading().approxEqual(Angle::degrees(90.0), 1e-12);
        }
        CHECK(headingHeld);                               // on every tick, not just the last
        CHECK(r.loc.headingBias().value() == seededBias);  // the seed untouched, bit for bit
        CHECK(r.loc.pose().x().value() == doctest::Approx(-45.0).epsilon(0.002));
        CHECK(r.loc.pose().y().value() == doctest::Approx(-26.0).epsilon(0.002));
    }
}
