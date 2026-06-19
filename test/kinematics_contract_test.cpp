// Contract-level tests for the kinematics layer (Freeze F5):
//   * WheelSpeeds value-type invariants and bounds (preconditions go RED, not UB),
//   * proof that IKinematics is implementable and usable polymorphically,
//   * a preview of the sanctioned "drop/re-attach the radian" discipline that the
//     real MatrixKinematics will rely on (ω·r ⇄ linear speed happens in exactly
//     one place, here demonstrated on a minimal differential double).
//
// The concrete X-drive / tank math and its round-trip + desaturation properties
// are tested separately as those impls land.

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/kinematics/kinematics.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::kinematics::IKinematics;
using shulib::kinematics::WheelSpeeds;
using shulib::math::ChassisSpeeds;
using shulib::math::Twist2d;
using shulib::units::AngularVelocity;
using shulib::units::Velocity;

// ---------------------------------------------------------------------------
// WheelSpeeds — bounds & invariants. These target the ways an index/count bug
// would silently corrupt state; each must throw (turn a test red) instead.
// ---------------------------------------------------------------------------

TEST_CASE("WheelSpeeds: a fresh set is empty and zeroed") {
    WheelSpeeds ws;
    CHECK(ws.size() == 0);
    CHECK(ws.maxMagnitude().value() == doctest::Approx(0.0));

    WheelSpeeds four{4};
    CHECK(four.size() == 4);
    for (int i = 0; i < 4; ++i) {
        CHECK(four[i].value() == doctest::Approx(0.0));  // explicit() ctor zero-inits
    }
}

TEST_CASE("WheelSpeeds: count must stay within [0, kMaxWheels]") {
    CHECK(WheelSpeeds{WheelSpeeds::kMaxWheels}.size() == WheelSpeeds::kMaxWheels);
    CHECK(WheelSpeeds{0}.size() == 0);
    // Boundary just past the cap, and a negative count, are contract violations.
    CHECK_THROWS_AS(WheelSpeeds{WheelSpeeds::kMaxWheels + 1}, PreconditionError);
    CHECK_THROWS_AS(WheelSpeeds{-1}, PreconditionError);
}

TEST_CASE("WheelSpeeds: index access is bounds-checked on both ends") {
    WheelSpeeds ws{3};
    // valid interior + both boundaries
    CHECK_NOTHROW((void)ws[0]);
    CHECK_NOTHROW((void)ws[2]);
    // just outside each end
    CHECK_THROWS_AS((void)ws[-1], PreconditionError);
    CHECK_THROWS_AS((void)ws[3], PreconditionError);
    CHECK_THROWS_AS(ws.set(-1, Velocity{1.0}), PreconditionError);
    CHECK_THROWS_AS(ws.set(3, Velocity{1.0}), PreconditionError);
}

TEST_CASE("WheelSpeeds: set/get round-trips and preserves units") {
    WheelSpeeds ws{2};
    ws.set(0, Velocity{12.5});
    ws.set(1, Velocity{-4.0});
    CHECK(ws[0].value() == doctest::Approx(12.5));
    CHECK(ws[1].value() == doctest::Approx(-4.0));
}

TEST_CASE("WheelSpeeds: maxMagnitude uses absolute value, not signed max") {
    WheelSpeeds ws{3};
    ws.set(0, Velocity{3.0});
    ws.set(1, Velocity{-5.0});  // largest by magnitude, but most-negative
    ws.set(2, Velocity{1.0});
    // A signed max() would wrongly return 3.0; |−5| must win.
    CHECK(ws.maxMagnitude().value() == doctest::Approx(5.0));
}

TEST_CASE("WheelSpeeds: approxEqual respects size, tolerance and sign") {
    WheelSpeeds a{2};
    a.set(0, Velocity{1.0});
    a.set(1, Velocity{2.0});

    WheelSpeeds b{2};
    b.set(0, Velocity{1.0});
    b.set(1, Velocity{2.0});
    CHECK(a.approxEqual(b));

    // within tolerance
    b.set(1, Velocity{2.0 + 1e-12});
    CHECK(a.approxEqual(b));

    // outside tolerance
    b.set(1, Velocity{2.5});
    CHECK_FALSE(a.approxEqual(b));

    // different size is never equal, even if the shared prefix matches
    WheelSpeeds shorter{1};
    shorter.set(0, Velocity{1.0});
    CHECK_FALSE(a.approxEqual(shorter));
}

// ---------------------------------------------------------------------------
// IKinematics is implementable & polymorphic. A minimal differential double:
// two wheels (0 = left, 1 = right), track width W, no strafe. This previews the
// ONE sanctioned spot where the radian is dropped (ω·r → in/s) on the way to
// wheels and re-attached (in/s → ω) on the way back — the same discipline the
// real MatrixKinematics uses, kept in a single documented place.
// ---------------------------------------------------------------------------

namespace {

class FakeDifferential final : public IKinematics {
public:
    explicit FakeDifferential(double trackWidthInches) : halfTrack_{trackWidthInches / 2.0} {}

    [[nodiscard]] WheelSpeeds toWheels(const ChassisSpeeds& body) const override {
        // SANCTIONED radian-drop: ω [rad/s] · halfTrack [in] → tangential [in/s].
        const double tangential = body.omega().value() * halfTrack_;
        WheelSpeeds w{2};
        w.set(0, Velocity{body.vx().value() - tangential});  // left
        w.set(1, Velocity{body.vx().value() + tangential});  // right
        return w;  // NOTE: no clamping here (§13 #5); vy is intentionally ignored.
    }

    [[nodiscard]] Twist2d forward(const WheelSpeeds& w) const override {
        const double left = w[0].value();
        const double right = w[1].value();
        const double vx = (left + right) / 2.0;
        // SANCTIONED radian-re-attach: (Δspeed / track) [1/s] → ω [rad/s].
        const double omega = ((right - left) / 2.0) / halfTrack_;
        return Twist2d{Velocity{vx}, Velocity{0.0}, AngularVelocity{omega}};
    }

    [[nodiscard]] WheelSpeeds desaturate(const WheelSpeeds& w, Velocity) const override {
        return w;  // the real uniform-scale algorithm is tested with MatrixKinematics
    }

    [[nodiscard]] double strafeAuthority() const override { return 0.0; }
    [[nodiscard]] int wheelCount() const override { return 2; }

private:
    double halfTrack_;
};

}  // namespace

TEST_CASE("IKinematics: a concrete drive is usable through the interface") {
    const FakeDifferential drive{10.0};        // 10" track → halfTrack 5"
    const IKinematics& k = drive;              // exercise the polymorphic seam

    CHECK(k.wheelCount() == 2);
    CHECK(k.strafeAuthority() == doctest::Approx(0.0));

    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{12.0}, Velocity{0.0}, AngularVelocity{1.0}});
    REQUIRE(w.size() == 2);
    CHECK(w[0].value() == doctest::Approx(7.0));   // 12 - 1·5
    CHECK(w[1].value() == doctest::Approx(17.0));  // 12 + 1·5
}

TEST_CASE("IKinematics: forward∘toWheels is identity for an achievable twist") {
    const FakeDifferential drive{10.0};
    const IKinematics& k = drive;

    const ChassisSpeeds cmd{Velocity{8.0}, Velocity{0.0}, AngularVelocity{-1.5}};
    const Twist2d back = k.forward(k.toWheels(cmd));

    CHECK(back.vx().value() == doctest::Approx(8.0));
    CHECK(back.omega().value() == doctest::Approx(-1.5));
    CHECK(back.vy().value() == doctest::Approx(0.0));
}

TEST_CASE("IKinematics: a non-strafing drive ignores commanded vy (cannot strafe)") {
    const FakeDifferential drive{10.0};
    const IKinematics& k = drive;

    const WheelSpeeds noStrafe = k.toWheels(ChassisSpeeds{Velocity{6.0}, Velocity{0.0}, AngularVelocity{0.0}});
    const WheelSpeeds withStrafe = k.toWheels(ChassisSpeeds{Velocity{6.0}, Velocity{9.0}, AngularVelocity{0.0}});
    // vy must not leak into the wheels of a 0-strafe-authority drive.
    CHECK(noStrafe.approxEqual(withStrafe));
}
