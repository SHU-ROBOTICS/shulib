// DIAG1 — an implausible odometry tick names its check and its values. The trust gate's
// account (diag/odom_tick_audit.hpp) from the two odometries, through the Localizer and
// motion::tickHealthObservables, onto HealthMonitor's ODO_STUCK line — and the one thing
// that must NOT move: the gate's verdict. Each case names the bug it catches; the mutation
// table is in the chunk's progress log (DIAG1-PROGRESS.md).
//
//   A  the formatter: goldens, token order, deterministic non-finite tokens, compaction,
//      the 140-byte bound against adversarial values, 7-bit ASCII
//   B  DriveEncoderOdometry fills the audit from its own gate (every check, both sides,
//      both boundaries, NaN / huge / overflow, not sticky, a 2000-tick sweep, the median,
//      the default before the first update, NONFINITE alone from an overflowing side sum,
//      setPose leaving the account alone)
//   C  PilonsOdometry, the same — and the forward value is the RAW pod delta; NONFINITE
//      alone from a rotated pod pair and from a huge offset
//   D  HealthMonitor: the trigger is unchanged, the detail is rising-edge-only, UNREPORTED,
//      the +STALL precedence
//   E  the Localizer forwards bit for bit; tickHealthObservables carries the field
//   F  end to end on a drive-encoder robot on the host test plant: Chassis::drive at rest
//      with the IMU stepped, the alternating step, and a MoveToPose that still aborts
//
// The robot evidence behind the chunk (2026-09-25, robot two): two `implausible odometry
// tick` faults in one start, the line naming nothing. Nothing here has run on a robot.

#include "doctest.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/diag/odom_tick_audit.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/arc_step.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/odometry.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::control::ExitReason;
using shulib::diag::appendOdomTickAudit;
using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::diag::HealthMonitor;
using shulib::diag::kOdomAuditDetailMaxBytes;
using shulib::diag::OdomTickAudit;
using shulib::diag::OdomTravelLabels;
using shulib::diag::lineformat::Line;
using shulib::hal::DriveGeometry;
using shulib::hal::IMotor;
using shulib::hal::LogLevel;
using shulib::hal::MotorGroup;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeMotor;
using shulib::hal::fake::FakeRotation;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::localization::DriveEncoderOdometry;
using shulib::localization::DriveEncoderOdometryConfig;
using shulib::localization::IOdometry;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::PilonsOdometryConfig;
using shulib::localization::TrackingWheel;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Frame;
using shulib::math::Pose2d;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kMax = std::numeric_limits<double>::max();
constexpr double kBound = 36.0;  // the default maxTickTravel, both odometries

/// Render an audit alone (the text after `implausible odometry tick`).
[[nodiscard]] std::string render(const OdomTickAudit& a) {
    Line line;
    appendOdomTickAudit(line, a);
    return std::string{line.view()};
}

/// The whole ODO_STUCK detail HealthMonitor builds for an implausible (± stalled) tick.
[[nodiscard]] std::string detailFor(const OdomTickAudit& a, bool stalled) {
    return "implausible odometry tick" + render(a) + (stalled ? " +STALL" : "");
}

/// Bitwise equality of two doubles (NaN == NaN with the same payload; -0.0 != +0.0).
[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

/// Equal as values, where two NaNs count as equal (an oracle comparison, not a bit one).
[[nodiscard]] bool sameValue(double a, double b) {
    return (std::isnan(a) && std::isnan(b)) || a == b;
}

/// Two audits identical to the bit: every flag, every value (sameBits), the labels.
[[nodiscard]] bool sameAudit(const OdomTickAudit& a, const OdomTickAudit& b) {
    return a.nonFinite == b.nonFinite && a.rotation == b.rotation && a.travel == b.travel
           && sameBits(a.dTheta.value(), b.dTheta.value())
           && sameBits(a.headingBefore.value(), b.headingBefore.value())
           && sameBits(a.headingAfter.value(), b.headingAfter.value())
           && sameBits(a.travelA.value(), b.travelA.value())
           && sameBits(a.travelB.value(), b.travelB.value()) && a.labels == b.labels;
}

[[nodiscard]] bool allPrintableAscii(std::string_view s) {
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        if (c < 0x20U || c > 0x7EU) {
            return false;
        }
    }
    return true;
}

/// The ERROR lines a fault latch's sink received.
[[nodiscard]] std::vector<std::string> errorLines(const FakeTelemetrySink& sink) {
    std::vector<std::string> out;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).level == LogLevel::Error) {
            out.push_back(sink.at(i).message);
        }
    }
    return out;
}

/// 1 in per radian: a shaft reading in radians IS inches of side travel.
const DriveGeometry kUnitGeometry{Length{1.0}, 1.0};

/// Tracking wheels with diameter 2 (radius 1): shaft radians are inches of travel.
TrackingWheel fwdWheel(FakeRotation& r, double leftOffset = 0.0) {
    return TrackingWheel::forward(r, Length{2.0}, Length{leftOffset});
}
TrackingWheel latWheel(FakeRotation& r, double forwardOffset = 0.0) {
    return TrackingWheel::lateral(r, Length{2.0}, Length{forwardOffset});
}

/// A scripted IOdometry whose verdict and account are set independently — so a test can
/// hand the monitor a verdict with no account, or an account with no verdict.
class AuditScript final : public IOdometry {
public:
    void update() override {}
    [[nodiscard]] Pose2d pose() const noexcept override { return Pose2d{}; }
    void setPose(const Pose2d&) override {}
    [[nodiscard]] bool lastDeltaImplausible() const noexcept override { return implausible; }
    [[nodiscard]] OdomTickAudit lastTickAudit() const noexcept override { return audit; }
    bool implausible = false;
    OdomTickAudit audit{};
};

/// A tiny deterministic generator for the sweeps (no dependence on the plant's Rng).
struct Lcg {
    std::uint64_t s;
    explicit Lcg(std::uint64_t seed) : s{seed} {}
    [[nodiscard]] double unit() {  // [0, 1)
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(s >> 11) * (1.0 / 9007199254740992.0);
    }
    [[nodiscard]] double between(double lo, double hi) { return lo + (hi - lo) * unit(); }
};

}  // namespace

// ═══ A. The formatter ════════════════════════════════════════════════════════════════

// Bug caught: any drift in the text a person reads at 2am — a renamed token, a missing unit,
// the angles in degrees, a padded column. The brief's own example is the golden.
TEST_CASE("DIAG1 A1: the rendering matches the goldens, drive sides and tracking wheels") {
    OdomTickAudit a;
    a.rotation = true;
    a.dTheta = AngleDim{1.745};
    a.headingBefore = AngleDim{0.012};
    a.headingAfter = AngleDim{1.757};
    CHECK(render(a) == " why=ROT dth=1.745rad h0=0.012 h1=1.757 L=0.000in R=0.000in");

    OdomTickAudit t;
    t.travel = true;
    t.dTheta = AngleDim{-0.0214};
    t.headingBefore = AngleDim{-3.1};
    t.headingAfter = AngleDim{3.1215};
    t.travelA = Length{40.25};
    t.travelB = Length{-0.5};
    t.labels = OdomTravelLabels::TrackingWheels;
    CHECK(render(t) == " why=TRAVEL dth=-0.021rad h0=-3.100 h1=3.122 fwd=40.250in lat=-0.500in");
}

// Bug caught: tokens in a data-dependent order, a missing '+', a token for an unset flag,
// or an untripped audit printing values nobody reported.
TEST_CASE("DIAG1 A2: the why= tokens — fixed order, '+'-joined, UNREPORTED when none") {
    struct Case {
        bool n, r, t;
        const char* why;
    };
    const Case cases[] = {
        {false, false, false, "UNREPORTED"},
        {true, false, false, "NONFINITE"},
        {false, true, false, "ROT"},
        {false, false, true, "TRAVEL"},
        {true, true, false, "NONFINITE+ROT"},
        {true, false, true, "NONFINITE+TRAVEL"},
        {false, true, true, "ROT+TRAVEL"},
        {true, true, true, "NONFINITE+ROT+TRAVEL"},
    };
    for (const Case& c : cases) {
        CAPTURE(c.why);
        OdomTickAudit a;
        a.nonFinite = c.n;
        a.rotation = c.r;
        a.travel = c.t;
        a.dTheta = AngleDim{0.5};
        const std::string s = render(a);
        CHECK(a.tripped() == (c.n || c.r || c.t));
        if (!a.tripped()) {
            CHECK(s == " why=UNREPORTED");  // and NO values: nobody gave an account
        } else {
            CHECK(s.rfind(std::string{" why="} + c.why + " dth=0.500rad h0=", 0) == 0);
        }
    }
}

// Bug caught: libc's own NaN/Inf spellings ("nan", "-nan(0x8000)", "inf") — which differ
// by platform and would break the one-formatter rule — and a 300-digit value blowing the
// size bound. The same tokens every §18.3 line uses.
TEST_CASE("DIAG1 A3: non-finite values render as fixed tokens; huge values compact") {
    OdomTickAudit a;
    a.nonFinite = true;
    a.travel = true;
    a.travelA = Length{kNaN};
    a.travelB = Length{-kNaN};  // a NEGATIVE NaN must not become "-nan"
    CHECK(render(a) == " why=NONFINITE+TRAVEL dth=0.000rad h0=0.000 h1=0.000 L=NaNin R=NaNin");
    a.travelA = Length{kInf};
    a.travelB = Length{-kInf};
    CHECK(render(a) == " why=NONFINITE+TRAVEL dth=0.000rad h0=0.000 h1=0.000 L=+Infin R=-Infin");

    OdomTickAudit h;
    h.travel = true;
    h.travelA = Length{1e9};           // "1000000000.000" is 14 bytes: compacted
    h.travelB = Length{-123456.789};   // "-123456.789" is 11 bytes: compacted
    CHECK(render(h) == " why=TRAVEL dth=0.000rad h0=0.000 h1=0.000 L=1e+09in R=-1.23e+05in");
    h.travelA = Length{-99999.999};    // exactly 10 bytes: NOT compacted
    h.travelB = Length{kMax};
    CHECK(render(h)
          == " why=TRAVEL dth=0.000rad h0=0.000 h1=0.000 L=-99999.999in R=1.8e+308in");
}

// Bug caught: a detail that FaultLatch silently truncates (its 176-byte buffer) — the one
// failure a golden cannot see, because it only shows with the widest values. Every
// rendering of an adversarial value set, in every flag/label/stall combination, must fit
// in 140 bytes and be printable 7-bit ASCII; the widest must be the computed 137.
TEST_CASE("DIAG1 A4: the worst-case detail fits the 140-byte bound and is 7-bit ASCII") {
    // FaultLatch's arithmetic, stated: `fault=ODO_STUCK` + ` n=` + a 10-digit int + ` FIRST`
    // + the separating space = 35; the buffer holds 175 bytes before its NUL.
    const std::size_t prefix = std::strlen("fault=ODO_STUCK") + std::strlen(" n=") + 10
                               + std::strlen(" FIRST") + 1;
    CHECK(prefix == 35);
    CHECK(prefix + kOdomAuditDetailMaxBytes == 175);

    const double values[] = {-99999.999,  -99999.9995, -kMax, kMax,       -1e308,
                             -9.995e307,  -1.23e-310,  -0.0,  0.0,        kNaN,
                             kInf,        -kInf,       -123456.789, 99999.999, 1.7453292519943295,
                             -3.141592653589793};
    std::size_t widestNumber = 0;
    for (const double v : values) {
        Line l;
        shulib::diag::lineformat::appendNum(l, v, 0, shulib::diag::kOdomAuditPrecision);
        widestNumber = std::max(widestNumber, l.view().size());
    }
    // "-99999.999" is 10; anything longer re-renders as %.3g, whose widest is 10 too
    // ("-9.99e+307"; -DBL_MAX itself is "-1.8e+308").
    CHECK(widestNumber == 10);

    std::size_t worst = 0;
    Lcg rng{0xD1A61};
    const std::size_t nValues = sizeof values / sizeof values[0];
    auto pick = [&] {
        return values[static_cast<std::size_t>(rng.unit() * static_cast<double>(nValues))];
    };
    for (int i = 0; i < 20000; ++i) {
        OdomTickAudit a;
        a.nonFinite = rng.unit() < 0.6;
        a.rotation = rng.unit() < 0.6;
        a.travel = rng.unit() < 0.6;
        a.dTheta = AngleDim{pick()};
        a.headingBefore = AngleDim{pick()};
        a.headingAfter = AngleDim{pick()};
        a.travelA = Length{pick()};
        a.travelB = Length{pick()};
        a.labels = rng.unit() < 0.5 ? OdomTravelLabels::DriveSides
                                    : OdomTravelLabels::TrackingWheels;
        const std::string d = detailFor(a, rng.unit() < 0.5);
        worst = std::max(worst, d.size());
        REQUIRE(d.size() <= kOdomAuditDetailMaxBytes);
        REQUIRE(allPrintableAscii(d));
    }
    // The constructed worst case: every token, five 10-byte numbers, the longer labels,
    // and +STALL. 137 is the header's arithmetic; the sweep must not exceed it either.
    OdomTickAudit w;
    w.nonFinite = w.rotation = w.travel = true;
    w.dTheta = w.headingBefore = w.headingAfter = AngleDim{-99999.999};
    w.travelA = w.travelB = Length{-99999.999};
    w.labels = OdomTravelLabels::TrackingWheels;
    const std::string wd = detailFor(w, true);
    CHECK(wd.size() == 137);
    CHECK(worst <= wd.size());

    // And through the REAL latch: the whole detail reaches the sink, untruncated.
    FakeTelemetrySink sink;
    FakeClock clk;
    FaultLatch latch{sink, clk};
    HealthMonitor mon{latch};
    mon.tick({.odomImplausible = true, .odomStalled = true, .odomAudit = w});
    REQUIRE(sink.size() == 1);
    CHECK(sink.last().message == "fault=ODO_STUCK n=1 FIRST " + wd);
}

// ═══ B. DriveEncoderOdometry fills the audit from its own gate ═══════════════════════

// Bug caught: the rotation flag or its values mis-reported — a wrong heading pair, degrees
// for radians, a travel value invented on a rotation-only tick, the wrong labels.
TEST_CASE("DIAG1 B1: DriveEncoderOdometry, rotation only — exact dth, h0, h1") {
    FakeImu imu;
    FakeMotor left, right;
    imu.setHeading(Angle::degrees(10.0));
    DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry, Length{12.0}};
    // Before the first update: no account — the DEFAULT audit, to the bit (odometry.hpp).
    CHECK_FALSE(odom.lastTickAudit().tripped());
    CHECK(sameAudit(odom.lastTickAudit(), OdomTickAudit{}));
    const Angle h0 = Angle::degrees(10.0);
    const Angle h1 = Angle::degrees(115.0);
    imu.setHeading(h1);
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    CHECK(odom.lastDeltaImplausible());
    CHECK(a.rotation);
    CHECK_FALSE(a.travel);
    CHECK_FALSE(a.nonFinite);
    CHECK(sameBits(a.dTheta.value(), h0.errorTo(h1)));
    CHECK(sameBits(a.headingBefore.value(), h0.radians()));
    CHECK(sameBits(a.headingAfter.value(), h1.radians()));
    CHECK(a.travelA.value() == 0.0);
    CHECK(a.travelB.value() == 0.0);
    CHECK(a.labels == OdomTravelLabels::DriveSides);
    CHECK(render(a) == " why=ROT dth=1.833rad h0=0.175 h1=2.007 L=0.000in R=0.000in");
}

// Bug caught: the two sides swapped into each other's slots — the line would then send a
// person to the wrong side of the robot. Each side alone, both signs.
TEST_CASE("DIAG1 B2: DriveEncoderOdometry, travel on the LEFT only and on the RIGHT only") {
    FakeImu imu;
    FakeMotor left, right;
    DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry, Length{12.0}};

    left.setPosition(AngleDim{40.0});
    odom.update();
    OdomTickAudit a = odom.lastTickAudit();
    CHECK(a.travel);
    CHECK_FALSE(a.rotation);
    CHECK_FALSE(a.nonFinite);
    CHECK(a.travelA.value() == 40.0);
    CHECK(a.travelB.value() == 0.0);
    CHECK(render(a).find(" L=40.000in R=0.000in") != std::string::npos);

    right.setPosition(AngleDim{-45.5});
    odom.update();
    a = odom.lastTickAudit();
    CHECK(a.travel);
    CHECK(a.travelA.value() == 0.0);
    CHECK(a.travelB.value() == -45.5);
    CHECK(render(a).find(" L=0.000in R=-45.500in") != std::string::npos);
}

// Bug caught: an off-by-one at either boundary — `>=` for the strict rotation test, `<` for
// the inclusive travel bound. Exact equality must NOT trip; one ulp past it must.
TEST_CASE("DIAG1 B3: DriveEncoderOdometry, both bounds at exact equality do not trip; one ulp "
          "past them does") {
    // Each case starts from a zero baseline, so the side travel IS the reading, exactly
    // (36 + nextafter(36) rounds back to 72: a second tick cannot carry a one-ulp delta).
    SUBCASE("travel, left side") {
        for (const bool pastIt : {false, true}) {
            CAPTURE(pastIt);
            FakeImu imu;
            FakeMotor left, right;
            DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry,
                                      Length{12.0}};
            const double v = pastIt ? std::nextafter(kBound, kInf) : kBound;
            left.setPosition(AngleDim{v});
            odom.update();
            CHECK(odom.lastTickAudit().travelA.value() == v);
            CHECK(odom.lastTickAudit().travel == pastIt);
            CHECK(odom.lastDeltaImplausible() == pastIt);
        }
    }
    SUBCASE("travel, right side, negative") {
        for (const bool pastIt : {false, true}) {
            CAPTURE(pastIt);
            FakeImu imu;
            FakeMotor left, right;
            DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry,
                                      Length{12.0}};
            const double v = pastIt ? -std::nextafter(kBound, kInf) : -kBound;
            right.setPosition(AngleDim{v});
            odom.update();
            CHECK(odom.lastTickAudit().travelB.value() == v);
            CHECK(odom.lastTickAudit().travel == pastIt);
            CHECK(odom.lastDeltaImplausible() == pastIt);
        }
    }
    SUBCASE("rotation") {
        const double dth = Angle::degrees(0.0).errorTo(Angle::degrees(30.0));
        for (const bool pastIt : {false, true}) {
            CAPTURE(pastIt);
            FakeImu imu;
            FakeMotor left, right;
            DriveEncoderOdometryConfig cfg;
            // At equality the bound IS the tick's |dth|; "one ulp past" puts the bound one
            // ulp BELOW it, so |dth| exceeds the bound by exactly one ulp.
            cfg.maxTickRotation = AngleDim{pastIt ? std::nextafter(dth, 0.0) : dth};
            DriveEncoderOdometry odom{imu,          left,          right, kUnitGeometry,
                                      kUnitGeometry, Length{12.0}, {},    cfg};
            imu.setHeading(Angle::degrees(30.0));
            odom.update();
            CHECK(odom.lastTickAudit().rotation == pastIt);
            CHECK(odom.lastDeltaImplausible() == pastIt);
            CHECK(sameBits(odom.lastTickAudit().dTheta.value(), dth));
        }
    }
}

// Bug caught: a NaN side mis-reported — NONFINITE alone (travel re-derived as |d| > max,
// which is false for NaN), the NaN rendered as libc's spelling, the value lost, or the
// position not frozen.
TEST_CASE("DIAG1 B4: DriveEncoderOdometry, a NaN left side is NONFINITE+TRAVEL, L=NaN, and "
          "position freezes") {
    FakeImu imu;
    FakeMotor left, right;
    DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry, Length{12.0}};
    left.setPosition(AngleDim{3.0});
    right.setPosition(AngleDim{3.0});
    odom.update();
    const Pose2d before = odom.pose();
    left.setPosition(AngleDim{kNaN});
    right.setPosition(AngleDim{4.0});
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    CHECK(a.nonFinite);
    CHECK(a.travel);
    CHECK_FALSE(a.rotation);
    CHECK(std::isnan(a.travelA.value()));
    CHECK(a.travelB.value() == 1.0);
    CHECK(odom.pose().x().value() == before.x().value());  // frozen
    CHECK(odom.pose().y().value() == before.y().value());
    CHECK(render(a) == " why=NONFINITE+TRAVEL dth=0.000rad h0=0.000 h1=0.000 L=NaNin R=1.000in");
}

// Bug caught: a finite-but-huge jump reported as NONFINITE (it is not) or withheld from the
// pose (the gate reports, never withholds), or its value printed as 14 digits.
TEST_CASE("DIAG1 B5: DriveEncoderOdometry, a finite-but-huge jump is TRAVEL only, integrated, "
          "compacted") {
    FakeImu imu;
    FakeMotor left, right;
    DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry, Length{12.0}};
    left.setPosition(AngleDim{1e9});
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    CHECK(a.travel);
    CHECK_FALSE(a.nonFinite);
    CHECK_FALSE(a.rotation);
    CHECK(a.travelA.value() == 1e9);
    CHECK(odom.pose().x().value() == doctest::Approx(0.5e9));  // integrated: reported, not withheld
    CHECK(render(a) == " why=TRAVEL dth=0.000rad h0=0.000 h1=0.000 L=1e+09in R=0.000in");
}

// Bug caught: an arithmetic overflow (a finite shaft reading times the ratio) reported as
// TRAVEL alone, or the +Inf rendered as "inf".
TEST_CASE("DIAG1 B6: DriveEncoderOdometry, an overflow to +Inf is NONFINITE+TRAVEL") {
    FakeImu imu;
    FakeMotor left, right;
    const DriveGeometry twoInPerRad{Length{2.0}, 1.0};
    DriveEncoderOdometry odom{imu, left, right, twoInPerRad, twoInPerRad, Length{12.0}};
    left.setPosition(AngleDim{kMax});  // finite reading; x 2 in/rad overflows
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    CHECK(a.nonFinite);
    CHECK(a.travel);
    CHECK(a.travelA.value() == kInf);
    CHECK(render(a).find(" L=+Infin R=0.000in") != std::string::npos);
}

// Bug caught: a tick that trips two checks naming only one; and a STICKY audit — flags or
// values left over from an earlier tick, so a clean tick would still read as implausible.
TEST_CASE("DIAG1 B7: DriveEncoderOdometry, ROT+TRAVEL in one tick names both; the next clean "
          "tick clears every flag") {
    FakeImu imu;
    FakeMotor left, right;
    DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry, Length{12.0}};
    imu.setHeading(Angle::degrees(100.0));
    right.setPosition(AngleDim{50.0});
    odom.update();
    CHECK(render(odom.lastTickAudit()).rfind(" why=ROT+TRAVEL dth=1.745rad", 0) == 0);
    CHECK(odom.lastTickAudit().travelB.value() == 50.0);

    left.setPosition(AngleDim{1.0});
    right.setPosition(AngleDim{51.0});
    imu.setHeading(Angle::degrees(101.0));
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    CHECK_FALSE(odom.lastDeltaImplausible());
    CHECK_FALSE(a.tripped());
    CHECK_FALSE(a.rotation);
    CHECK_FALSE(a.travel);
    CHECK_FALSE(a.nonFinite);
    CHECK(a.travelA.value() == 1.0);  // THIS tick's values, not the last implausible one's
    CHECK(a.travelB.value() == 1.0);
    CHECK(a.dTheta.value() == doctest::Approx(Angle::degrees(1.0).radians()));
}

// Bug caught: the verdict and the account drifting apart (the verdict computed on its own
// with a term dropped, a flag re-derived differently) — checked on EVERY tick of a 2000-tick
// script that exercises all three checks, against an oracle built from the script alone.
TEST_CASE("DIAG1 B8: DriveEncoderOdometry, 2000 scripted ticks — verdict == tripped() and every "
          "flag and value matches the oracle, every tick") {
    FakeImu imu;
    FakeMotor left, right;
    DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry, Length{12.0}};
    Lcg rng{20260926};
    double pL = 0.0, pR = 0.0, baseL = 0.0, baseR = 0.0, h = 0.0;
    Angle prev{};
    int nRot = 0, nTravel = 0, nNonFinite = 0, nClean = 0;
    for (int tick = 0; tick < 2000; ++tick) {
        CAPTURE(tick);
        const double r = rng.unit();
        h += (r < 0.06) ? rng.between(1.2, 3.0) * (rng.unit() < 0.5 ? -1.0 : 1.0)
                        : rng.between(-0.05, 0.05);
        const double stepL = (r > 0.94) ? rng.between(20.0, 60.0) : rng.between(-0.6, 0.6);
        const double stepR = (r > 0.90 && r < 0.93) ? -rng.between(20.0, 60.0)
                                                    : rng.between(-0.6, 0.6);
        pL += stepL;
        pR += stepR;
        const bool nanTick = (tick % 97) == 50;
        const double readL = nanTick ? kNaN : pL;
        const Angle h1 = Angle::radians(h);
        imu.setHeading(h1);
        left.setPosition(AngleDim{readL});
        right.setPosition(AngleDim{pR});
        odom.update();

        // The oracle, from the script alone (the class's arithmetic, restated).
        const double dL = (readL - baseL) * 1.0;
        const double dR = (pR - baseR) * 1.0;
        if (std::isfinite(readL)) baseL = readL;
        baseR = pR;
        const double dth = prev.errorTo(h1);
        const auto d = shulib::localization::arcStep(
            {Length{0.5 * (dL + dR)}, Length{0.0}}, prev, h1);
        const bool expNonFinite = !(std::isfinite(d.dx.value()) && std::isfinite(d.dy.value()));
        const bool expRot = std::abs(dth) > 0.5 * Angle::kPi;
        const bool expTravel = !(std::abs(dL) <= kBound && std::abs(dR) <= kBound);
        prev = h1;

        const OdomTickAudit a = odom.lastTickAudit();
        REQUIRE(odom.lastDeltaImplausible() == a.tripped());
        REQUIRE(a.nonFinite == expNonFinite);
        REQUIRE(a.rotation == expRot);
        REQUIRE(a.travel == expTravel);
        REQUIRE(sameBits(a.dTheta.value(), dth));
        REQUIRE(sameValue(a.travelA.value(), dL));
        REQUIRE(sameValue(a.travelB.value(), dR));
        nRot += a.rotation ? 1 : 0;
        nTravel += a.travel ? 1 : 0;
        nNonFinite += a.nonFinite ? 1 : 0;
        nClean += a.tripped() ? 0 : 1;
    }
    MESSAGE("DEO sweep: rot ", nRot, " travel ", nTravel, " nonfinite ", nNonFinite, " clean ",
            nClean);
    CHECK(nRot >= 30);  // the sweep exercised every check, and plenty of clean ticks
    CHECK(nTravel >= 30);
    CHECK(nNonFinite >= 15);
    CHECK(nClean >= 1000);
}

// Bug caught: an account that reads a group MEMBER instead of the group's median — one dead
// or waking member of four is NOT an implausible tick (the median holds), and when two jump
// the value on the line must be the median the odometry actually used.
TEST_CASE("DIAG1 B9: over MotorGroups — one member of four jumps: not implausible; two jump: "
          "TRAVEL with L = the median's value") {
    std::array<FakeMotor, 4> l{}, r{};
    std::array<IMotor*, 4> lp{&l[0], &l[1], &l[2], &l[3]};
    std::array<IMotor*, 4> rp{&r[0], &r[1], &r[2], &r[3]};
    MotorGroup left{std::span<IMotor* const>{lp}};
    MotorGroup right{std::span<IMotor* const>{rp}};
    FakeImu imu;
    DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry, Length{12.0}};

    l[2].setPosition(AngleDim{1000.0});  // one of four wakes 1000 rad on
    odom.update();
    CHECK_FALSE(odom.lastDeltaImplausible());
    CHECK_FALSE(odom.lastTickAudit().tripped());
    CHECK(odom.lastTickAudit().travelA.value() == 0.0);  // the median did not move

    l[0].setPosition(AngleDim{1000.0});  // a second one: [0, 0, 1000, 1000] -> median 500
    odom.update();
    CHECK(odom.lastDeltaImplausible());
    CHECK(odom.lastTickAudit().travel);
    CHECK(odom.lastTickAudit().travelA.value() == 500.0);
    CHECK(render(odom.lastTickAudit()).find(" L=500.000in R=0.000in") != std::string::npos);
}

// Bug caught: the verdict computed apart from the account with the `!finite` term dropped
// (review round 1's finding — the first run called that mutation "equivalent", and it is
// not). The config check asks only that maxTickTravel be finite and > 0, so a bound near
// DBL_MAX is legal; with both sides at 1e308 each side is WITHIN it (travel false) but
// dL + dR overflows to +Inf BEFORE the halving, and arcStep turns that into dx = +Inf,
// dy = Inf * 0 = NaN. The one flag that says so is NONFINITE — alone, and the line must say
// NONFINITE with no TRAVEL beside it. Position freezes.
TEST_CASE("DIAG1 B10: DriveEncoderOdometry, both sides within a DBL_MAX bound but their sum "
          "overflows — NONFINITE alone") {
    FakeImu imu;
    FakeMotor left, right;
    DriveEncoderOdometry odom{imu,          left, right, kUnitGeometry, kUnitGeometry,
                              Length{12.0}, Pose2d{Length{3.0}, Length{4.0}, Angle{}},
                              DriveEncoderOdometryConfig{.maxTickTravel = Length{kMax}}};
    left.setPosition(AngleDim{1e308});
    right.setPosition(AngleDim{1e308});
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    REQUIRE(odom.lastDeltaImplausible());
    REQUIRE(a.nonFinite);
    REQUIRE_FALSE(a.travel);  // each side IS within the bound
    REQUIRE_FALSE(a.rotation);
    CHECK(a.travelA.value() == 1e308);
    CHECK(a.travelB.value() == 1e308);
    CHECK(odom.pose().x().value() == 3.0);  // frozen
    CHECK(odom.pose().y().value() == 4.0);
    CHECK(render(a) == " why=NONFINITE dth=0.000rad h0=0.000 h1=0.000 L=1e+308in R=1e+308in");
}

// Bug caught: setPose() clearing (or otherwise touching) the account — odometry.hpp promises
// it leaves the account alone, as it leaves the verdict, so the two cannot disagree between
// a teleport and the next update().
TEST_CASE("DIAG1 B11: DriveEncoderOdometry, setPose() leaves the account and the verdict "
          "alone") {
    FakeImu imu;
    FakeMotor left, right;
    DriveEncoderOdometry odom{imu, left, right, kUnitGeometry, kUnitGeometry, Length{12.0}};
    imu.setHeading(Angle::degrees(100.0));
    right.setPosition(AngleDim{50.0});
    odom.update();
    const OdomTickAudit before = odom.lastTickAudit();
    REQUIRE(before.rotation);
    REQUIRE(before.travel);
    odom.setPose(Pose2d{Length{7.0}, Length{-2.0}, Angle{}});
    CHECK(odom.pose().x().value() == 7.0);  // the teleport happened
    CHECK(odom.lastDeltaImplausible());
    CHECK(sameAudit(odom.lastTickAudit(), before));
    CHECK(odom.lastTickAudit().tripped() == odom.lastDeltaImplausible());
}

// ═══ C. PilonsOdometry, the same account ═════════════════════════════════════════════

// Bug caught: the rotation flag or values mis-reported on the tracking-wheel robot, or the
// line printing drive-side labels for pods.
TEST_CASE("DIAG1 C1: PilonsOdometry, rotation only — exact dth, h0, h1, tracking-wheel labels") {
    FakeImu imu;
    FakeRotation f, l;
    imu.setHeading(Angle::degrees(-20.0));
    PilonsOdometry odom{imu, fwdWheel(f), latWheel(l)};
    // Before the first update: no account — the DEFAULT audit, to the bit (odometry.hpp;
    // its labels are the default DriveSides until a tick fills them).
    CHECK_FALSE(odom.lastTickAudit().tripped());
    CHECK(sameAudit(odom.lastTickAudit(), OdomTickAudit{}));
    const Angle h0 = Angle::degrees(-20.0);
    const Angle h1 = Angle::degrees(-125.0);
    imu.setHeading(h1);
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    CHECK(odom.lastDeltaImplausible());
    CHECK(a.rotation);
    CHECK_FALSE(a.travel);
    CHECK_FALSE(a.nonFinite);
    CHECK(sameBits(a.dTheta.value(), h0.errorTo(h1)));
    CHECK(sameBits(a.headingBefore.value(), h0.radians()));
    CHECK(sameBits(a.headingAfter.value(), h1.radians()));
    CHECK(a.labels == OdomTravelLabels::TrackingWheels);
    CHECK(render(a) == " why=ROT dth=-1.833rad h0=-0.349 h1=-2.182 fwd=0.000in lat=0.000in");
}

// Bug caught: the two pods swapped into each other's slots.
TEST_CASE("DIAG1 C2: PilonsOdometry, travel on the forward pod only and on the lateral pod "
          "only") {
    FakeImu imu;
    FakeRotation f, l;
    PilonsOdometry odom{imu, fwdWheel(f), latWheel(l)};
    f.setPosition(AngleDim{40.0});
    odom.update();
    CHECK(odom.lastTickAudit().travel);
    CHECK(odom.lastTickAudit().travelA.value() == 40.0);
    CHECK(odom.lastTickAudit().travelB.value() == 0.0);
    l.setPosition(AngleDim{-37.0});
    odom.update();
    CHECK(odom.lastTickAudit().travel);
    CHECK(odom.lastTickAudit().travelA.value() == 0.0);
    CHECK(odom.lastTickAudit().travelB.value() == -37.0);
    CHECK(render(odom.lastTickAudit()).find(" fwd=0.000in lat=-37.000in") != std::string::npos);
}

// Bug caught: an off-by-one at either boundary, on the tracking-wheel gate.
TEST_CASE("DIAG1 C3: PilonsOdometry, both bounds at exact equality do not trip; one ulp past "
          "them does") {
    SUBCASE("travel, lateral pod and forward pod") {
        for (const bool pastIt : {false, true}) {
            for (const bool lateral : {false, true}) {
                CAPTURE(pastIt);
                CAPTURE(lateral);
                FakeImu imu;
                FakeRotation f, l;
                PilonsOdometry odom{imu, fwdWheel(f), latWheel(l)};
                const double v = pastIt ? std::nextafter(kBound, kInf) : kBound;
                (lateral ? l : f).setPosition(AngleDim{lateral ? v : -v});
                odom.update();
                CHECK((lateral ? odom.lastTickAudit().travelB.value()
                               : -odom.lastTickAudit().travelA.value())
                      == v);
                CHECK(odom.lastTickAudit().travel == pastIt);
                CHECK(odom.lastDeltaImplausible() == pastIt);
            }
        }
    }
    SUBCASE("rotation") {
        const double dth = Angle::degrees(0.0).errorTo(Angle::degrees(-30.0));
        for (const bool pastIt : {false, true}) {
            CAPTURE(pastIt);
            FakeImu imu;
            FakeRotation f, l;
            const double bound = pastIt ? std::nextafter(std::abs(dth), 0.0) : std::abs(dth);
            PilonsOdometry odom{imu, fwdWheel(f), latWheel(l), Pose2d{},
                                PilonsOdometryConfig{.maxTickRotation = AngleDim{bound}}};
            imu.setHeading(Angle::degrees(-30.0));
            odom.update();
            CHECK(odom.lastTickAudit().rotation == pastIt);
            CHECK(odom.lastDeltaImplausible() == pastIt);
        }
    }
}

// Bug caught: the NaN pod mis-reported (NONFINITE alone), or position not frozen.
TEST_CASE("DIAG1 C4: PilonsOdometry, a NaN forward pod is NONFINITE+TRAVEL and position "
          "freezes") {
    FakeImu imu;
    FakeRotation f, l;
    PilonsOdometry odom{imu, fwdWheel(f), latWheel(l), Pose2d{Length{3.0}, Length{4.0}, Angle{}}};
    f.setPosition(AngleDim{kNaN});
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    CHECK(a.nonFinite);
    CHECK(a.travel);
    CHECK_FALSE(a.rotation);
    CHECK(std::isnan(a.travelA.value()));
    CHECK(odom.pose().x().value() == 3.0);
    CHECK(odom.pose().y().value() == 4.0);
    CHECK(render(a).find(" fwd=NaNin lat=0.000in") != std::string::npos);
}

// Bug caught: a finite-but-huge pod jump reported as non-finite, or withheld; an overflow
// reported as TRAVEL alone.
TEST_CASE("DIAG1 C5: PilonsOdometry, a finite-but-huge jump is TRAVEL only; an overflow is "
          "NONFINITE+TRAVEL") {
    FakeImu imu;
    FakeRotation f, l;
    PilonsOdometry odom{imu, fwdWheel(f), latWheel(l)};
    f.setPosition(AngleDim{1e9});
    odom.update();
    CHECK(odom.lastTickAudit().travel);
    CHECK_FALSE(odom.lastTickAudit().nonFinite);
    CHECK(odom.pose().x().value() == doctest::Approx(1e9));  // integrated
    CHECK(render(odom.lastTickAudit()).find(" fwd=1e+09in") != std::string::npos);

    FakeImu imu2;
    FakeRotation f2, l2;
    // Diameter 4 (radius 2): a finite shaft reading of DBL_MAX overflows the travel to +Inf.
    PilonsOdometry odom2{imu2, TrackingWheel::forward(f2, Length{4.0}, Length{0.0}), latWheel(l2)};
    f2.setPosition(AngleDim{kMax});
    odom2.update();
    CHECK(odom2.lastTickAudit().nonFinite);
    CHECK(odom2.lastTickAudit().travel);
    CHECK(odom2.lastTickAudit().travelA.value() == kInf);
}

// Bug caught: two checks in one tick naming one; a sticky account.
TEST_CASE("DIAG1 C6: PilonsOdometry, ROT+TRAVEL names both; the next clean tick clears them") {
    FakeImu imu;
    FakeRotation f, l;
    PilonsOdometry odom{imu, fwdWheel(f), latWheel(l)};
    imu.setHeading(Angle::degrees(170.0));
    f.setPosition(AngleDim{-80.0});
    odom.update();
    CHECK(render(odom.lastTickAudit()).rfind(" why=ROT+TRAVEL dth=2.967rad", 0) == 0);
    f.setPosition(AngleDim{-79.0});
    imu.setHeading(Angle::degrees(171.0));
    odom.update();
    CHECK_FALSE(odom.lastDeltaImplausible());
    CHECK_FALSE(odom.lastTickAudit().tripped());
    CHECK(odom.lastTickAudit().travelA.value() == 1.0);
}

// Bug caught: the account carrying the offset-CORRECTED centre value instead of the RAW pod
// delta the gate actually bounded. With a forward pod 5 in left of centre and a -60 deg
// tick, a raw 40 in reading corrects to ~34.76 in — under the bound — so a line printing
// the centre value would show a number that does not explain the fault.
TEST_CASE("DIAG1 C7: PilonsOdometry carries the RAW forward delta, not the centre value") {
    FakeImu imu;
    FakeRotation f, l;
    PilonsOdometry odom{imu, fwdWheel(f, 5.0), latWheel(l, -3.0)};
    imu.setHeading(Angle::degrees(-60.0));
    f.setPosition(AngleDim{40.0});
    odom.update();
    const OdomTickAudit a = odom.lastTickAudit();
    const double centre = 40.0 + a.dTheta.value() * 5.0;
    REQUIRE(centre < kBound);  // the centre alone would NOT have tripped the gate
    CHECK(a.travel);
    CHECK_FALSE(a.rotation);
    CHECK(a.travelA.value() == 40.0);
    CHECK(render(a).find(" fwd=40.000in") != std::string::npos);
}

// Bug caught: verdict and account drifting apart on the tracking-wheel odometry. No NaN is
// injected here (a NaN pod poisons its TrackingWheel baseline for good — C4 covers it); the
// non-finite ticks come from an OVERFLOW pair (a pod reading -DBL_MAX, then +DBL_MAX).
TEST_CASE("DIAG1 C8: PilonsOdometry, 2000 scripted ticks — verdict == tripped() and every flag "
          "and value matches the oracle, every tick") {
    FakeImu imu;
    FakeRotation f, l;
    const double bF = 5.0, aL = -3.0;
    PilonsOdometry odom{imu, fwdWheel(f, bF), latWheel(l, aL)};
    Lcg rng{0x9115};
    double sf = 0.0, sl = 0.0, lastF = 0.0, lastL = 0.0, h = 0.0;
    Angle prev{};
    int nRot = 0, nTravel = 0, nNonFinite = 0, nClean = 0;
    for (int tick = 0; tick < 2000; ++tick) {
        CAPTURE(tick);
        const double r = rng.unit();
        h += (r < 0.06) ? rng.between(1.2, 3.0) * (rng.unit() < 0.5 ? -1.0 : 1.0)
                        : rng.between(-0.05, 0.05);
        const int phase = tick % 101;
        if (phase == 60) {
            sf = -kMax;  // finite, enormous: TRAVEL
        } else if (phase == 61) {
            sf = kMax;   // +DBL_MAX - (-DBL_MAX) overflows: NONFINITE+TRAVEL
        } else if (phase == 62) {
            sf = 0.0;    // back: 0 - DBL_MAX, finite: TRAVEL
        } else {
            sf += (r > 0.95) ? rng.between(20.0, 60.0) : rng.between(-0.6, 0.6);
        }
        sl += (r > 0.90 && r < 0.93) ? -rng.between(20.0, 60.0) : rng.between(-0.6, 0.6);
        const Angle h1 = Angle::radians(h);
        imu.setHeading(h1);
        f.setPosition(AngleDim{sf});
        l.setPosition(AngleDim{sl});
        odom.update();

        const double fwd = (sf - lastF) * 1.0;
        const double lat = (sl - lastL) * 1.0;
        lastF = sf;
        lastL = sl;
        const double dth = prev.errorTo(h1);
        const auto d = shulib::localization::arcStep(
            {Length{fwd + dth * bF}, Length{lat - dth * aL}}, prev, h1);
        const bool expNonFinite = !(std::isfinite(d.dx.value()) && std::isfinite(d.dy.value()));
        const bool expRot = std::abs(dth) > 0.5 * Angle::kPi;
        const bool expTravel = !(std::abs(fwd) <= kBound && std::abs(lat) <= kBound);
        prev = h1;

        const OdomTickAudit a = odom.lastTickAudit();
        REQUIRE(odom.lastDeltaImplausible() == a.tripped());
        REQUIRE(a.nonFinite == expNonFinite);
        REQUIRE(a.rotation == expRot);
        REQUIRE(a.travel == expTravel);
        REQUIRE(sameBits(a.dTheta.value(), dth));
        REQUIRE(sameValue(a.travelA.value(), fwd));
        REQUIRE(sameValue(a.travelB.value(), lat));
        nRot += a.rotation ? 1 : 0;
        nTravel += a.travel ? 1 : 0;
        nNonFinite += a.nonFinite ? 1 : 0;
        nClean += a.tripped() ? 0 : 1;
    }
    MESSAGE("Pilons sweep: rot ", nRot, " travel ", nTravel, " nonfinite ", nNonFinite,
            " clean ", nClean);
    CHECK(nRot >= 30);
    CHECK(nTravel >= 30);
    CHECK(nNonFinite >= 15);
    CHECK(nClean >= 1000);
}

// Bug caught: the verdict computed apart from the account with the `!finite` term dropped,
// on the tracking-wheel odometry (review round 1). Two legal configurations reach a
// NONFINITE tick with neither other check tripped:
//  - a DBL_MAX travel bound: both pods at 0.9 DBL_MAX are within it, but arcStep rotates
//    (fwd, lat) by the heading, and at -45 deg the x component is fwd cos + lat sin(45) =
//    ~1.27 DBL_MAX = +Inf (at 0 deg the same pods stay finite — the rotation is the cause);
//  - a huge FINITE wheel offset (TrackingWheel checks only the diameter): the offset
//    correction dTheta * offset overflows on an ordinary 1.5 rad tick with no pod travel.
// Each must read NONFINITE alone, with position frozen.
TEST_CASE("DIAG1 C9: PilonsOdometry, NONFINITE alone — a rotated pod pair past DBL_MAX, and a "
          "huge offset's correction") {
    SUBCASE("both pods within a DBL_MAX bound; the rotation overflows") {
        FakeImu imu;
        FakeRotation f, l;
        imu.setHeading(Angle::degrees(-45.0));
        PilonsOdometry odom{imu, fwdWheel(f), latWheel(l), Pose2d{Length{3.0}, Length{4.0}, Angle{}},
                            PilonsOdometryConfig{.maxTickTravel = Length{kMax}}};
        f.setPosition(AngleDim{0.9 * kMax});
        l.setPosition(AngleDim{0.9 * kMax});
        odom.update();
        const OdomTickAudit a = odom.lastTickAudit();
        REQUIRE(odom.lastDeltaImplausible());
        REQUIRE(a.nonFinite);
        REQUIRE_FALSE(a.travel);
        REQUIRE_FALSE(a.rotation);
        CHECK(odom.pose().x().value() == 3.0);  // frozen
        CHECK(odom.pose().y().value() == 4.0);
        CHECK(render(a)
              == " why=NONFINITE dth=0.000rad h0=-0.785 h1=-0.785 fwd=1.62e+308in "
                 "lat=1.62e+308in");
    }
    SUBCASE("a DBL_MAX forward-pod offset; dTheta * offset overflows") {
        FakeImu imu;
        FakeRotation f, l;
        PilonsOdometry odom{imu, TrackingWheel::forward(f, Length{2.0}, Length{kMax}),
                            latWheel(l), Pose2d{Length{3.0}, Length{4.0}, Angle{}}};
        imu.setHeading(Angle::radians(1.5));  // under the pi/2 rotation bound
        odom.update();
        const OdomTickAudit a = odom.lastTickAudit();
        REQUIRE(odom.lastDeltaImplausible());
        REQUIRE(a.nonFinite);
        REQUIRE_FALSE(a.travel);
        REQUIRE_FALSE(a.rotation);
        CHECK(odom.pose().x().value() == 3.0);  // frozen
        CHECK(odom.pose().y().value() == 4.0);
        CHECK(render(a) == " why=NONFINITE dth=1.500rad h0=0.000 h1=1.500 fwd=0.000in lat=0.000in");
    }
}

// Bug caught: setPose() clearing (or otherwise touching) the account on the tracking-wheel
// odometry.
TEST_CASE("DIAG1 C10: PilonsOdometry, setPose() leaves the account and the verdict alone") {
    FakeImu imu;
    FakeRotation f, l;
    PilonsOdometry odom{imu, fwdWheel(f), latWheel(l)};
    imu.setHeading(Angle::degrees(170.0));
    f.setPosition(AngleDim{-80.0});
    odom.update();
    const OdomTickAudit before = odom.lastTickAudit();
    REQUIRE(before.rotation);
    REQUIRE(before.travel);
    odom.setPose(Pose2d{Length{7.0}, Length{-2.0}, Angle{}});
    CHECK(odom.pose().x().value() == 7.0);  // the teleport happened
    CHECK(odom.lastDeltaImplausible());
    CHECK(sameAudit(odom.lastTickAudit(), before));
    CHECK(odom.lastTickAudit().tripped() == odom.lastDeltaImplausible());
}

// ═══ D. HealthMonitor: the trigger unchanged, the detail on the rising edge only ═════

namespace {
struct MonitorRig {
    FakeTelemetrySink sink;
    FakeClock clock{Time{1.0}};
    FaultLatch latch{sink, clock};
    HealthMonitor monitor{latch};
};

OdomTickAudit rotAudit(double dth, double h0, double h1) {
    OdomTickAudit a;
    a.rotation = true;
    a.dTheta = AngleDim{dth};
    a.headingBefore = AngleDim{h0};
    a.headingAfter = AngleDim{h1};
    return a;
}
}  // namespace

// Bug caught: the fault line itself drifting — the whole Error line is the golden.
TEST_CASE("DIAG1 D1: the ODO_STUCK line is the golden, subsystem LOC, level Error") {
    MonitorRig r;
    r.monitor.tick({.odomImplausible = true, .odomAudit = rotAudit(1.745, 0.012, 1.757)});
    REQUIRE(r.sink.size() == 1);
    CHECK(r.sink.last().level == LogLevel::Error);
    CHECK(r.sink.last().subsystem == "LOC");
    CHECK(r.sink.last().message
          == "fault=ODO_STUCK n=1 FIRST implausible odometry tick why=ROT dth=1.745rad "
             "h0=0.012 h1=1.757 L=0.000in R=0.000in");
}

// Bug caught: the detail formatted (or raised) on every tick of an episode — a 500-line
// storm, or one line carrying the LAST tick's values instead of the first.
TEST_CASE("DIAG1 D2: 500 implausible ticks with changing audits are ONE line with tick 1's "
          "values; a clean tick then a new one is a second line") {
    MonitorRig r;
    for (int i = 0; i < 500; ++i) {
        const double k = static_cast<double>(i);
        r.monitor.tick({.odomImplausible = true,
                        .odomAudit = rotAudit(1.6 + 0.001 * k, 0.1 * k, -0.2 - 0.001 * k)});
    }
    CHECK(r.latch.faultCount() == 1);
    REQUIRE(errorLines(r.sink).size() == 1);
    CHECK(errorLines(r.sink)[0]
          == "fault=ODO_STUCK n=1 FIRST implausible odometry tick why=ROT dth=1.600rad "
             "h0=0.000 h1=-0.200 L=0.000in R=0.000in");

    r.monitor.tick({});  // clean: the episode ends, the edge re-arms
    OdomTickAudit t;
    t.travel = true;
    t.travelB = Length{-41.0};
    r.monitor.tick({.odomImplausible = true, .odomAudit = t});
    REQUIRE(errorLines(r.sink).size() == 2);
    CHECK(errorLines(r.sink)[1]
          == "fault=ODO_STUCK n=2 implausible odometry tick why=TRAVEL dth=0.000rad h0=0.000 "
             "h1=0.000 L=0.000in R=-41.000in");
}

// Bug caught: a verdict with no account rendered as invented values (or as a malformed
// "why= dth=..."): a source that never filled the field must say so.
TEST_CASE("DIAG1 D3: an implausible verdict with an empty audit reads why=UNREPORTED") {
    MonitorRig r;
    r.monitor.tick({.odomImplausible = true});
    REQUIRE(r.sink.size() == 1);
    CHECK(r.sink.last().message
          == "fault=ODO_STUCK n=1 FIRST implausible odometry tick why=UNREPORTED");
}

// Bug caught: the audit promoted to a TRIGGER — the one change that could move the fault.
// A tripped account with the verdict false raises nothing, for a whole episode.
TEST_CASE("DIAG1 D4: flags set but odomImplausible false raises nothing") {
    MonitorRig r;
    OdomTickAudit a = rotAudit(3.0, 0.0, 3.0);
    a.travel = true;
    a.nonFinite = true;
    for (int i = 0; i < 50; ++i) {
        r.monitor.tick({.odomImplausible = false, .odomAudit = a});
    }
    CHECK_FALSE(r.latch.hasFault());
    CHECK(r.sink.empty());
}

// Bug caught: the +STALL precedence inverted (the stall text winning and the values lost),
// the stall-alone text changed, or a second line inside one episode.
TEST_CASE("DIAG1 D5: stall and implausible rising together read the valued text + STALL; "
          "stall alone keeps its text") {
    SUBCASE("together") {
        MonitorRig r;
        r.monitor.tick({.odomImplausible = true,
                        .odomStalled = true,
                        .odomAudit = rotAudit(-2.0, 1.0, -1.0)});
        REQUIRE(r.sink.size() == 1);
        CHECK(r.sink.last().message
              == "fault=ODO_STUCK n=1 FIRST implausible odometry tick why=ROT dth=-2.000rad "
                 "h0=1.000 h1=-1.000 L=0.000in R=0.000in +STALL");
    }
    SUBCASE("stall alone, then implausible inside the same episode") {
        MonitorRig r;
        r.monitor.tick({.odomStalled = true});
        r.monitor.tick({.odomImplausible = true,
                        .odomStalled = true,
                        .odomAudit = rotAudit(2.0, 0.0, 2.0)});
        REQUIRE(r.sink.size() == 1);  // one episode
        CHECK(r.sink.last().message
              == "fault=ODO_STUCK n=1 FIRST wheels spin but odometry reports no motion");
    }
}

// ═══ E. The Localizer forwards; tickHealthObservables carries it ═════════════════════

// Bug caught: a forwarder that returns a default (or a copy of some other state) — the
// monitor would then print UNREPORTED on a robot whose odometry did report.
TEST_CASE("DIAG1 E1: Localizer::lastOdomTickAudit forwards bit for bit") {
    FakeClock clk{Time{2.0}};
    FakeImu imu;
    AuditScript odom;
    shulib::localization::ComplementaryFusion fusion;
    Localizer loc{clk, imu, odom, fusion};
    odom.audit.nonFinite = true;
    odom.audit.travel = true;
    odom.audit.dTheta = AngleDim{-0.0};
    odom.audit.headingBefore = AngleDim{std::numeric_limits<double>::denorm_min()};
    odom.audit.headingAfter = AngleDim{-kInf};
    odom.audit.travelA = Length{std::bit_cast<double>(std::uint64_t{0x7FF800000000BEEFULL})};
    odom.audit.travelB = Length{-1.0e-300};
    odom.audit.labels = OdomTravelLabels::TrackingWheels;
    const OdomTickAudit got = loc.lastOdomTickAudit();
    CHECK(got.nonFinite);
    CHECK_FALSE(got.rotation);
    CHECK(got.travel);
    CHECK(sameBits(got.dTheta.value(), -0.0));
    CHECK(sameBits(got.headingBefore.value(), std::numeric_limits<double>::denorm_min()));
    CHECK(sameBits(got.headingAfter.value(), -kInf));
    CHECK(sameBits(got.travelA.value(), odom.audit.travelA.value()));  // the NaN payload too
    CHECK(sameBits(got.travelB.value(), -1.0e-300));
    CHECK(got.labels == OdomTravelLabels::TrackingWheels);
}

// Bug caught: the one shared health tick dropping the field — every program (drive(), every
// motion, the idle tick) raises through it, so this is where a robot would lose the detail.
TEST_CASE("DIAG1 E2: motion::tickHealthObservables hands the audit to the monitor") {
    const TankKinematics kin{Length{12.0}};
    shulib::sim::SimHarness h{kin, motion_rig::plantConfig()};
    AuditScript odom;
    shulib::localization::ComplementaryFusion fusion;
    Localizer loc{h.clock(), h.imu(), odom, fusion};
    FakeTelemetrySink sink;
    FaultLatch latch{sink, h.clock()};
    HealthMonitor health{latch};
    const shulib::motion::MotionDeps deps{.ctx = &h.context(),
                                          .localizer = &loc,
                                          .kinematics = &kin,
                                          .faults = &latch,
                                          .health = &health};
    odom.implausible = true;
    odom.audit.travel = true;
    odom.audit.travelA = Length{-38.5};
    loc.update();
    shulib::motion::tickHealthObservables(deps, false);
    REQUIRE(sink.size() == 1);
    CHECK(sink.last().message
          == "fault=ODO_STUCK n=1 FIRST implausible odometry tick why=TRAVEL dth=0.000rad "
             "h0=0.000 h1=0.000 L=-38.500in R=0.000in");
}

// ═══ F. End to end: a drive-encoder robot on the host test plant ══════════════════════

namespace {

/// The IMU heading the plant publishes, stepped: truth until `stepAt`, then truth + the
/// offset. `alternateTicks` = 0 holds +100 deg for good; N > 0 alternates +100 / -100 deg
/// for N ticks and then returns to truth (the 2026-09-25 capture's swinging-heading shape,
/// exaggerated past the gate's bound).
struct ImuStep final : shulib::sim::DegradationModel {
    double stepAt;
    int alternateTicks;
    ImuStep(double at, int alternate) : stepAt{at}, alternateTicks{alternate} {}
    [[nodiscard]] Angle imuHeading(Angle trueHeading, Time now,
                                   shulib::sim::Rng& /*rng*/) override {
        if (now.value() < stepAt - 1e-9) {
            return trueHeading;
        }
        if (alternateTicks == 0) {
            return trueHeading + Angle::degrees(100.0);
        }
        const long k = std::lround((now.value() - stepAt) / 0.01);
        if (k >= alternateTicks) {
            return trueHeading;
        }
        return trueHeading + Angle::degrees(k % 2 == 0 ? 100.0 : -100.0);
    }
};

/// Robot two's shape: a tank, NO tracking wheels, DriveEncoderOdometry over the plant's two
/// drive motors, the full Localizer / latch / monitor / scheduler / Chassis stack.
struct DriveEncoderRig {
    TankKinematics kin{Length{12.0}};
    shulib::sim::SimHarness h;
    DriveGeometry geom;
    DriveEncoderOdometry odom;
    shulib::localization::ComplementaryFusion fusion;
    Localizer loc;
    FakeTelemetrySink faultSink;
    FaultLatch latch;
    HealthMonitor health;
    shulib::motion::MotionDeps deps;
    motion_rig::PlantPacer pacer;
    shulib::chassis::Chassis chassis;

    explicit DriveEncoderRig(shulib::sim::DegradationModel* degradation)
        : h{kin, motion_rig::plantConfig(), nullptr, degradation},
          geom{DriveGeometry::fromDiameter(motion_rig::plantConfig().plant.driveWheelDiameter, 1.0)},
          odom{h.imu(), h.motor(0), h.motor(1), geom, geom, Length{12.0}},
          fusion{},
          loc{h.clock(), h.imu(), odom, fusion},
          latch{faultSink, h.clock()},
          health{latch},
          deps{.ctx = &h.context(),
               .localizer = &loc,
               .kinematics = &kin,
               .faults = &latch,
               .health = &health},
          pacer{h},
          chassis{deps, pacer, motion_rig::chassisConfig()} {}
};

}  // namespace

// Bug caught: the detail lost anywhere between the odometry and the log on the real
// composition (the forwarder, the shared health tick, the monitor) — or a second line. At
// rest the travel is exactly zero, so the whole line is a golden.
TEST_CASE("DIAG1 F1: Chassis::drive at rest, IMU stepped +100 deg -> exactly one ERROR line, "
          "equal to the golden") {
    ImuStep step{0.5, 0};
    DriveEncoderRig rig{&step};
    const ChassisSpeeds zero{Velocity{0.0}, Velocity{0.0}, AngularVelocity{0.0}};
    for (int i = 0; i < 150; ++i) {
        rig.chassis.drive(zero, Frame::Body);
        rig.pacer.pace();
    }
    const std::vector<std::string> errs = errorLines(rig.faultSink);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0]
          == "fault=ODO_STUCK n=1 FIRST implausible odometry tick why=ROT dth=1.745rad "
             "h0=0.000 h1=1.745 L=0.000in R=0.000in");
    CHECK(rig.latch.firstFault() == FaultCode::OdoStuck);
}

// Bug caught: an episode of swinging headings (every tick implausible, each with different
// values) producing a line per tick, or a line carrying anything but the FIRST tick.
TEST_CASE("DIAG1 F2: alternating +/-100 deg for 50 ticks at rest -> one line, the first tick's "
          "values") {
    ImuStep swing{0.5, 50};
    DriveEncoderRig rig{&swing};
    const ChassisSpeeds zero{Velocity{0.0}, Velocity{0.0}, AngularVelocity{0.0}};
    int implausibleTicks = 0;
    for (int i = 0; i < 200; ++i) {
        rig.chassis.drive(zero, Frame::Body);
        implausibleTicks += rig.loc.lastOdomDeltaImplausible() ? 1 : 0;
        rig.pacer.pace();
    }
    CHECK(implausibleTicks == 51);  // 50 swings + the return to truth
    const std::vector<std::string> errs = errorLines(rig.faultSink);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0]
          == "fault=ODO_STUCK n=1 FIRST implausible odometry tick why=ROT dth=1.745rad "
             "h0=0.000 h1=1.745 L=0.000in R=0.000in");
}

// Bug caught: the chunk moving the POLICY — a heading step mid-motion must still end the
// motion as FAULT_ABORT on ODO_STUCK (the unchanged default), and now the line says why.
TEST_CASE("DIAG1 F3: a MoveToPose during an IMU step still exits FAULT_ABORT on ODO_STUCK, and "
          "its line carries the detail") {
    ImuStep step{0.5, 0};
    DriveEncoderRig rig{&step};
    shulib::motion::MotionScheduler& sched = rig.chassis.scheduler();
    shulib::motion::MoveToPose m{sched.deps(), Pose2d{Length{40.0}, Length{0.0}, Angle{}},
                                 motion_rig::motionConfig(), 6.0};
    sched.async(m);
    const ExitReason exit = sched.waitUntilSettled();
    // FAULT_ABORT is exactly "Cancelled with a causal fault" (run_reporter.hpp outcomeOf).
    CHECK(exit == ExitReason::Cancelled);
    CHECK(sched.lastCompleted().abortFault == FaultCode::OdoStuck);
    CHECK(rig.h.clock().now().value() < 1.0);  // the policy, at the step — not the watchdog
    std::string odoLine;
    for (const std::string& s : errorLines(rig.faultSink)) {
        if (s.rfind("fault=ODO_STUCK", 0) == 0) {
            REQUIRE(odoLine.empty());  // one ODO_STUCK line
            odoLine = s;
        }
    }
    REQUIRE_FALSE(odoLine.empty());
    MESSAGE("F3 line: ", odoLine);
    CHECK(odoLine.find(" implausible odometry tick why=ROT dth=1.") != std::string::npos);
    CHECK(odoLine.find(" L=") != std::string::npos);
    CHECK(odoLine.find("in R=") != std::string::npos);
    CHECK(odoLine.find("UNREPORTED") == std::string::npos);
    CHECK(odoLine.size() <= 175);
}
