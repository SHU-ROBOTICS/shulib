// DIAG1 Part B — a drive side's median leaves out a member whose reading is not finite.
// hal::medianInPlace (the one median in the tree), hal::MotorGroup's reads and its
// nonFiniteReads() counter, the drive-encoder odometry reading a group, the scheduler on the
// host test plant, and the drivetrain calibrator's "moved on the push" rule. Each case names the
// bug it catches; the mutation tables (the brief's M1..M19 and three more, M20..M22, then the
// review round's, 2026-09-30 — each run on a fresh binary and observed RED) are in the chunk's
// progress log (DIAG1-PROGRESS.md, "## Part B").
//
//   N1  medianInPlace, every placement: one bad value at every index of n = 1..8 and 21, every
//       PAIR for n <= 8, four kinds (NaN, a NaN with its sign bit set, +Inf, -Inf), against an
//       independent oracle (the finite values copied in order, an insertion sort written here)
//   N2  no finite value: a FRESH quiet NaN, never an input's sign or payload ({+Inf, -Inf} too)
//   N3  all finite is the old function, bit for bit, over 200,000 random arrays (signed zeros,
//       subnormals, duplicates, integers, |x| up to DBL_MAX/2) and 100,000 even ones whose middle
//       values are often past DBL_MAX/2 — plus {denorm_min, denorm_min}, and the ONE named
//       exception: a middle pair whose sum overflowed (the old half-sum gave ±Inf; now the finite
//       midpoint), {DBL_MAX, DBL_MAX} and every such pair the draw gives
//   N4  the finite values keep their input order: {+0, NaN, -0, +0} answers what {+0, -0, +0} does
//   N5  a non-finite member IS an absent member: position/velocity every slot, every kind, with a
//       hole beside it; never sticky; NaN when none is finite; the last-good-cache kill
//       sequence; current/totalCurrent/temperature (and a side drawing 0 A is 0, not NaN);
//       nonFiniteReads() counts every exclusion
//   N6  through DriveEncoderOdometry (N = 4 and 5): a bad member is not an implausible tick, L=
//       is the finite median's CHANGE × ipr, position integrates; a whole side bad is
//       NONFINITE+TRAVEL with L=NaN, and the next finite tick integrates across the gap
//   N7  THE MEMBERSHIP STEP, exact (A4 register HA-165): [100, 100.5, 101, 101.5, 80] steps
//       -0.25 and back +0.25; a two-member side [0, 100] steps -50 (TRAVEL) and back, ONE line;
//       several left out — the drifted member read alone steps by up to the spread, silently
//       under 36 in (where +Inf members used to freeze the tick), as TRAVEL past it
//   N8  1000+ scripted ticks: side readings, verdict, every flag and value against an oracle
//   N9  end to end on the host test plant through the scheduler: robot two's shape, five coupled
//       members a side behind two groups over DriveEncoderOdometry; one member NaN / +Inf / -Inf
//       for 31 ticks mid-motion leaves the whole trace bit-identical and raises no ODO_STUCK
//   N10 the calibrator: a candidate with no reading at DONE is not a drive port, is named NO
//       READING and in warning(), has no travel (+0.0, as a still port read backward has), and
//       leaves the LEFT scale bit-identical; two of them, one per side (two sentences); the net
//       when it recovers (NON_DRIVE_MOVED naming it, and saying it gave no reading); the spin's
//       side travel over a NaN member
//   N11 with every reading finite, current(), totalCurrent() and temperature() are the old reads,
//       bit for bit (the order of the sum, the sign of a zero), against verbatim copies
//   N12 the push over robot two's REAL wiring (five of ten ports read backward), a push led with
//       the other end (every delta negative), and a travel exactly AT movedThreshold
//
// NOTHING HERE HAS RUN ON A ROBOT, and on robot two the exclusion is expected never to fire:
// ProsMotor screens a non-finite read to the last good one before a group sees it (A4 register
// HA-08). These cases define what the library does when a reading does get through.

#include "doctest.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "calibration_test_rig.hpp"
#include "motion_test_rig.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/calibration/drivetrain_calibrator.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/diag/odom_tick_audit.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/arc_step.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::control::ExitReason;
using shulib::diag::appendOdomTickAudit;
using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::diag::HealthMonitor;
using shulib::diag::OdomTickAudit;
using shulib::diag::lineformat::Line;
using shulib::hal::DriveGeometry;
using shulib::hal::IMotor;
using shulib::hal::LogLevel;
using shulib::hal::MotorGroup;
using shulib::hal::medianInPlace;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeMotor;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::localization::DriveEncoderOdometry;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Length;
using shulib::units::Time;

namespace {

const double kQNaN = std::numeric_limits<double>::quiet_NaN();
const double kNegNaN = std::copysign(std::numeric_limits<double>::quiet_NaN(), -1.0);
const double kInf = std::numeric_limits<double>::infinity();

/// The four kinds of "not a number" a reading can be. A NaN with its sign bit set is a kind of
/// its own: a test written with a positive NaN only would pass a rule that looked at the sign.
const double kBadKinds[4] = {kQNaN, kNegNaN, kInf, -kInf};
const std::string kBadNames[4] = {"NaN", "-NaN", "+Inf", "-Inf"};

/// Bitwise equality of two doubles (NaN == NaN with the same sign and payload; -0.0 != +0.0).
[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

/// A tiny deterministic generator (copied from odom_tick_audit_test.cpp's `Lcg`).
struct Lcg {
    std::uint64_t s;
    explicit Lcg(std::uint64_t seed) : s{seed} {}
    [[nodiscard]] std::uint64_t bits() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return s;
    }
    [[nodiscard]] double unit() {  // [0, 1)
        return static_cast<double>(bits() >> 11) * (1.0 / 9007199254740992.0);
    }
    [[nodiscard]] double between(double lo, double hi) { return lo + (hi - lo) * unit(); }
    [[nodiscard]] std::size_t below(std::size_t n) {
        return static_cast<std::size_t>(unit() * static_cast<double>(n));
    }
};

/// THE ORACLE for a median over the finite values — written independently of the header: the
/// finite values copied in their order, an insertion sort written here, then the same odd /
/// midpoint rule; a fresh quiet NaN when nothing is finite.
[[nodiscard]] double oracleMedian(const double* in, std::size_t n) {
    std::vector<double> f;
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isnan(in[i]) && !std::isinf(in[i])) {
            f.push_back(in[i]);
        }
    }
    for (std::size_t i = 1; i < f.size(); ++i) {  // insertion sort (stable)
        const double x = f[i];
        std::size_t j = i;
        while (j > 0 && x < f[j - 1]) {
            f[j] = f[j - 1];
            --j;
        }
        f[j] = x;
    }
    const std::size_t k = f.size();
    if (k == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return (k % 2 == 1) ? f[k / 2] : std::midpoint(f[k / 2 - 1], f[k / 2]);
}

/// THE PIN (N3): a verbatim copy of `a01e3a2`'s hal::medianInPlace body (motor_group.hpp
/// :129-134 there), the preconditions aside. Fed FINITE values only — landmine 1: the old
/// function's answers for a NaN are undefined behaviour, observed and logged, never asserted.
[[nodiscard]] double legacyMedianInPlace(double* v, std::size_t n) {
    std::sort(v, v + n);
    return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/// medianInPlace over a COPY (the function reorders its argument).
[[nodiscard]] double medianOfCopy(const double* in, std::size_t n) {
    std::vector<double> v(in, in + n);
    return medianInPlace(v.data(), n);
}

/// Up to 21 fakes, a pointer array over them, and helpers to build a group over the first n
/// with chosen slots made ABSENT (nullptr) — the absent-member rule this chunk's rule copies.
struct Bank {
    static constexpr std::size_t kMax = 21;
    std::array<FakeMotor, kMax> m{};
    [[nodiscard]] std::array<IMotor*, kMax> ptrs(std::size_t n, std::uint32_t holes = 0) {
        std::array<IMotor*, kMax> p{};
        for (std::size_t i = 0; i < n; ++i) {
            p[i] = (holes & (std::uint32_t{1} << i)) != 0u ? nullptr : &m[i];
        }
        return p;
    }
};

/// Distinct finite readings for member i (no zeros, no two alike): position, velocity, current,
/// temperature. Chosen so that adding or removing any one member moves every median.
double posOf(std::size_t i) {
    return 1.0 + 0.5 * static_cast<double>(i) + 0.03125 * static_cast<double>(i % 3);
}
double velOf(std::size_t i) {
    return -7.0 + 1.25 * static_cast<double>(i) + 0.0625 * static_cast<double>(i % 4);
}
double ampsOf(std::size_t i) { return 0.25 + 0.125 * static_cast<double>(i); }
double tempOf(std::size_t i) { return 30.0 + 1.5 * static_cast<double>((i * 7) % 11); }

void fillFinite(Bank& b, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        b.m[i].setPosition(AngleDim{posOf(i)});
        b.m[i].setVelocity(AngularVelocity{velOf(i)});
        b.m[i].setCurrent(Current{ampsOf(i)});
        b.m[i].setTemperature(tempOf(i));
    }
}

/// Render an audit alone (the text after `implausible odometry tick`).
[[nodiscard]] std::string render(const OdomTickAudit& a) {
    Line line;
    appendOdomTickAudit(line, a);
    return std::string{line.view()};
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

/// 1.625 in per radian — robot two's plant radius, and exact in binary, so every product below
/// is exact and can be compared bit for bit.
const DriveGeometry kGeom = DriveGeometry::fromInchesPerRadian(Length{1.625});
const DriveGeometry kUnit = DriveGeometry::fromInchesPerRadian(Length{1.0});

}  // namespace

// ═══ N1-N4. hal::medianInPlace ═══════════════════════════════════════════════════════

// Bug caught: a non-finite value anywhere reaching std::sort (undefined behaviour: NaN, or a
// plausible wrong number decided by the slot), or a filter that knows only one kind of bad value
// (NaN but not ±Inf, or the reverse), or the exclusion living only in MotorGroup so that the
// calibrator's direct calls still sort a NaN.
// Mutations: M1 (no filter), M2 (std::isnan only), M3 (std::isinf only), M10 (exclusion moved
// into MotorGroup::medianOf).
TEST_CASE("DIAG1 N1: medianInPlace leaves out a non-finite value at every index, and every pair "
          "for n <= 8, all four kinds") {
    Lcg rng{0xD1A61B01};
    int singles = 0;
    int pairs = 0;
    for (std::size_t n : {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 21u}) {
        CAPTURE(n);
        // Distinct finite values, never zero (so the answer is one number, not a signed zero).
        double base[21];
        for (std::size_t i = 0; i < n; ++i) {
            base[i] = rng.between(-50.0, 50.0) + 100.0 * static_cast<double>(i % 2 == 0 ? 1 : -1);
        }
        for (std::size_t i = 0; i < n; ++i) {
            for (int kind = 0; kind < 4; ++kind) {
                CAPTURE(i);
                CAPTURE(kBadNames[kind]);
                double v[21];
                std::copy(base, base + n, v);
                v[i] = kBadKinds[kind];
                REQUIRE(sameBits(medianOfCopy(v, n), oracleMedian(v, n)));
                ++singles;
            }
        }
        if (n > 8) {
            continue;
        }
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                for (int ki = 0; ki < 4; ++ki) {
                    for (int kj = 0; kj < 4; ++kj) {
                        CAPTURE(i);
                        CAPTURE(j);
                        CAPTURE(kBadNames[ki]);
                        CAPTURE(kBadNames[kj]);
                        double v[8];
                        std::copy(base, base + n, v);
                        v[i] = kBadKinds[ki];
                        v[j] = kBadKinds[kj];
                        REQUIRE(sameBits(medianOfCopy(v, n), oracleMedian(v, n)));
                        ++pairs;
                    }
                }
            }
        }
    }
    MESSAGE("N1: ", singles, " single placements, ", pairs, " pair placements");
    // Spot values a reader can check by hand — the probe's shapes, now defined.
    const double a[] = {3.0, kQNaN, 1.0, 2.0};
    CHECK(medianOfCopy(a, 4) == 2.0);
    const double b[] = {kQNaN, 1.0, 2.0, 3.0, 4.0};
    CHECK(medianOfCopy(b, 5) == 2.5);
    const double c[] = {1.0, 2.0, kQNaN, 3.0, 4.0};
    CHECK(medianOfCopy(c, 5) == 2.5);
    const double d[] = {5.0, kInf, 1.0, 2.0, 3.0};
    CHECK(medianOfCopy(d, 5) == 2.5);  // one +Inf of five used to be "an outlier" and give 3
    const double e[] = {1.0, kInf, kInf};
    CHECK(medianOfCopy(e, 3) == 1.0);  // two +Inf of three used to REACH the middle: +Inf
}

// Bug caught: a read with nothing finite answering 0 (a side that "did not move" — the dead-
// encoder runaway the stall check exists to catch), answering one of its inputs (so the result's
// sign and payload depend on which bad values arrived — {+Inf, -Inf} used to give a NEGATIVE NaN),
// or passing an infinity through.
// Mutations: M4 (k == 0 returns 0.0), M5 (k == 0 returns v[0]).
TEST_CASE("DIAG1 N2: no finite value gives a FRESH quiet NaN — never an input, never 0") {
    const double fresh = std::numeric_limits<double>::quiet_NaN();
    Lcg rng{0xD1A61B02};
    int draws = 0;
    for (std::size_t n = 1; n <= 21; ++n) {
        for (int t = 0; t < 60; ++t) {
            double v[21];
            for (std::size_t i = 0; i < n; ++i) {
                v[i] = kBadKinds[rng.below(4)];
            }
            CAPTURE(n);
            REQUIRE(sameBits(medianOfCopy(v, n), fresh));
            ++draws;
        }
        // And each kind alone, filling the array — including a first slot that is not a
        // positive NaN, which is what a "return v[0]" would hand back.
        for (int kind = 0; kind < 4; ++kind) {
            double v[21];
            std::fill(v, v + n, kBadKinds[kind]);
            CAPTURE(kBadNames[kind]);
            REQUIRE(sameBits(medianOfCopy(v, n), fresh));
        }
    }
    const double pm[] = {kInf, -kInf};
    CHECK(sameBits(medianOfCopy(pm, 2), fresh));
    const double mp[] = {-kInf, kInf};
    CHECK(sameBits(medianOfCopy(mp, 2), fresh));
    const double one[] = {kNegNaN};
    CHECK(sameBits(medianOfCopy(one, 1), fresh));
    CHECK_FALSE(std::signbit(medianOfCopy(pm, 2)));
    MESSAGE("N2: ", draws, " random all-non-finite arrays");
}

// Bug caught: the all-finite answer — every tick a V5 ever produces through ProsMotor (HA-08) —
// moving by a single bit: a reordered compaction, a hand-rolled half-sum that differs at the
// subnormals, a midpoint that rounds differently — including past DBL_MAX/2, where std::midpoint
// takes its other branches. The legacy function is the pin; where its half-sum OVERFLOWED (to
// +Inf or -Inf), the answer must be the finite midpoint instead.
// Mutations: M8 (midpoint back to 0.5 * (a + b)), M9 (0.5*a + 0.5*b), MW (review round: a
// different midpoint past DBL_MAX/2 only, a + 0.5 * (b - a) — killed only by the second draw).
TEST_CASE("DIAG1 N3: with every value finite the median is the old function, bit for bit — "
          "except the one overflow it had") {
    Lcg rng{0xD1A61B03};
    auto draw = [&rng]() -> double {
        const double r = rng.unit();
        if (r < 0.08) {
            return rng.unit() < 0.5 ? 0.0 : -0.0;
        }
        if (r < 0.14) {  // subnormals, including the smallest
            const double k = static_cast<double>(1 + rng.below(1u << 20));
            return (rng.unit() < 0.5 ? 1.0 : -1.0) * k * std::numeric_limits<double>::denorm_min();
        }
        if (r < 0.17) {
            return (rng.unit() < 0.5 ? 1.0 : -1.0) * std::numeric_limits<double>::denorm_min();
        }
        if (r < 0.35) {  // integers
            return static_cast<double>(static_cast<int>(rng.below(2001)) - 1000);
        }
        if (r < 0.60) {
            return rng.between(-1.0e6, 1.0e6);
        }
        // Full range up to DBL_MAX/2: random sign, exponent 0..0x7FD, random mantissa.
        const std::uint64_t sign = rng.unit() < 0.5 ? 0 : (std::uint64_t{1} << 63);
        const std::uint64_t exp = static_cast<std::uint64_t>(rng.below(0x7FE));
        const std::uint64_t mant = rng.bits() & ((std::uint64_t{1} << 52) - 1);
        return std::bit_cast<double>(sign | (exp << 52) | mant);
    };
    int mismatches = 0;
    std::size_t firstN = 0;
    double firstNew = 0.0;
    double firstOld = 0.0;
    const int kArrays = 200000;
    for (int t = 0; t < kArrays; ++t) {
        const std::size_t n = 1 + rng.below(21);
        double a[21];
        for (std::size_t i = 0; i < n; ++i) {
            // Duplicates: a quarter of the values repeat an earlier one exactly.
            a[i] = (i > 0 && rng.unit() < 0.25) ? a[rng.below(i)] : draw();
        }
        double b[21];
        std::copy(a, a + n, b);
        const double now = medianInPlace(a, n);
        const double old = legacyMedianInPlace(b, n);
        if (!sameBits(now, old)) {
            if (mismatches == 0) {
                firstN = n;
                firstNew = now;
                firstOld = old;
            }
            ++mismatches;
        }
    }
    MESSAGE("N3: ", kArrays, " all-finite arrays compared bit for bit; first mismatch (if any) n=",
            firstN, " new=", firstNew, " old=", firstOld);
    CHECK(mismatches == 0);

    // PAST DBL_MAX/2 (added in the review round, 2026-09-30). The draw above stops at exponent
    // 0x7FD, so every value it gives is at most DBL_MAX/2 and std::midpoint only ever took its
    // first branch, (a + b) / 2. Here: EVEN arrays whose values are often HUGE (exponent 0x7FE, or
    // 0x7FD at the boundary; either sign), mixed with the ordinary draw. Where the old half-sum
    // did not overflow, the answer is the old one bit for bit — the pairs past DBL_MAX/2 included
    // (counted). Where it DID overflow (the middle pair's sum past DBL_MAX: +Inf or -Inf), the
    // answer is the finite midpoint, and the oracle for it is 0.5*a + 0.5*b — both halvings exact
    // (neither value is small when the sum overflows), one rounding, and written without
    // std::midpoint. Both counts are REQUIREd, so a draw that stopped reaching either branch fails
    // here rather than passing by having nothing to compare.
    auto huge = [&rng]() -> double {
        const std::uint64_t sign = rng.unit() < 0.5 ? 0 : (std::uint64_t{1} << 63);
        const std::uint64_t exp = rng.unit() < 0.8 ? 0x7FE : 0x7FD;
        const std::uint64_t mant = rng.bits() & ((std::uint64_t{1} << 52) - 1);
        return std::bit_cast<double>(sign | (exp << 52) | mant);
    };
    int pastHalfSame = 0;
    int overflowed = 0;
    int hugeMismatches = 0;
    const int kHugeArrays = 100000;
    for (int t = 0; t < kHugeArrays; ++t) {
        const std::size_t n = 2 * (1 + rng.below(10));  // 2..20, even: the midpoint is used
        double a[21];
        for (std::size_t i = 0; i < n; ++i) {
            a[i] = rng.unit() < 0.6 ? huge() : draw();
        }
        double b[21];
        double s[21];
        std::copy(a, a + n, b);
        std::copy(a, a + n, s);
        std::sort(s, s + n);
        const double lo = s[n / 2 - 1];
        const double hi = s[n / 2];
        const double now = medianInPlace(a, n);
        const double old = legacyMedianInPlace(b, n);
        if (std::isfinite(old)) {
            hugeMismatches += sameBits(now, old) ? 0 : 1;
            pastHalfSame += (std::abs(lo) > DBL_MAX / 2 || std::abs(hi) > DBL_MAX / 2) ? 1 : 0;
        } else {
            ++overflowed;  // every value finite, so the old answer overflowed in lo + hi
            hugeMismatches += (std::isfinite(now) && sameBits(now, 0.5 * lo + 0.5 * hi)) ? 0 : 1;
        }
    }
    MESSAGE("N3 past DBL_MAX/2: ", kHugeArrays, " even arrays; ", pastHalfSame,
            " non-overflowing middle pairs past DBL_MAX/2 (bit for bit), ", overflowed,
            " overflowing ones (the finite midpoint); mismatches ", hugeMismatches);
    CHECK(hugeMismatches == 0);
    REQUIRE(pastHalfSame > 1000);
    REQUIRE(overflowed > 1000);

    // Fixed cases, independent of the draw.
    const double dm = std::numeric_limits<double>::denorm_min();
    double sub[] = {dm, dm};
    double subOld[] = {dm, dm};
    CHECK(sameBits(medianInPlace(sub, 2), dm));                // 0.5*a + 0.5*b would give 0
    CHECK(sameBits(legacyMedianInPlace(subOld, 2), dm));
    double big[] = {DBL_MAX, DBL_MAX};
    double bigOld[] = {DBL_MAX, DBL_MAX};
    CHECK(sameBits(medianInPlace(big, 2), DBL_MAX));           // THE named exception:
    CHECK(sameBits(legacyMedianInPlace(bigOld, 2), kInf));     // the old half-sum overflowed
    double neg[] = {-DBL_MAX, -DBL_MAX};                       // ... to -Inf as well
    double negOld[] = {-DBL_MAX, -DBL_MAX};
    CHECK(sameBits(medianInPlace(neg, 2), -DBL_MAX));
    CHECK(sameBits(legacyMedianInPlace(negOld, 2), -kInf));
    double mixed[] = {-DBL_MAX, DBL_MAX, 1.0, 2.0};
    CHECK(medianInPlace(mixed, 4) == 1.5);
    double single[] = {-0.0};
    CHECK(sameBits(medianInPlace(single, 1), -0.0));          // n == 1: the bare value
}

// Bug caught: a compaction that reorders the finite values (std::partition, or swap-with-last).
// std::sort is not stable, so among EQUAL values which one lands in the middle depends on the
// order it is handed: a reordering compaction makes a group with a bad member answer
// differently from the same group with that member ABSENT — here, the sign of a zero median.
// Mutation: M6 (swap-with-last compaction).
TEST_CASE("DIAG1 N4: the finite values keep their order — {+0, NaN, -0, +0} is {+0, -0, +0}") {
    struct Case {
        std::vector<double> sparse;
        std::vector<double> dense;
    };
    const std::vector<Case> cases = {
        {{+0.0, kQNaN, -0.0, +0.0}, {+0.0, -0.0, +0.0}},
        {{-0.0, kInf, +0.0, -0.0}, {-0.0, +0.0, -0.0}},
        {{+0.0, -kInf, -0.0, kNegNaN, +0.0}, {+0.0, -0.0, +0.0}},
        {{-0.0, +0.0, kQNaN, -0.0, +0.0, -0.0}, {-0.0, +0.0, -0.0, +0.0, -0.0}},
    };
    for (const Case& c : cases) {
        const double sparse = medianOfCopy(c.sparse.data(), c.sparse.size());
        const double dense = medianOfCopy(c.dense.data(), c.dense.size());
        CAPTURE(c.sparse.size());
        MESSAGE("N4: sparse ", std::signbit(sparse) ? "-0" : "+0", ", dense ",
                std::signbit(dense) ? "-0" : "+0");
        CHECK(sameBits(sparse, dense));
    }
    // The case must be able to tell: on this library the dense {+0, -0, +0} answers -0 (its
    // three-element sort keeps equal values in input order), and a swap-with-last compaction of
    // the sparse input would hand the sort {+0, +0, -0}, answering +0.
    const double d0[] = {+0.0, -0.0, +0.0};
    const double swapped[] = {+0.0, +0.0, -0.0};
    CHECK_FALSE(sameBits(medianOfCopy(d0, 3), medianOfCopy(swapped, 3)));

    // Through the group: a member reading NaN answers exactly what the same slot ABSENT does.
    Bank b;
    b.m[0].setPosition(AngleDim{+0.0});
    b.m[1].setPosition(AngleDim{kQNaN});
    b.m[2].setPosition(AngleDim{-0.0});
    b.m[3].setPosition(AngleDim{+0.0});
    auto pAll = b.ptrs(4);
    auto pHole = b.ptrs(4, 0b0010u);
    MotorGroup g{std::span<IMotor* const>{pAll.data(), 4}};
    MotorGroup h{std::span<IMotor* const>{pHole.data(), 4}};
    CHECK(sameBits(g.position().value(), h.position().value()));
}

// ═══ N5. MotorGroup: a non-finite member is an absent member ═════════════════════════

// Bug caught: any read treating a member with no reading differently from a member that is not
// there — a wrong median, a mean divided by the wrong count, a max seeded from a NaN, a
// last-good cache (the frozen-member drag the median exists to survive), an exclusion that
// latches, a counter that misses a read or a kind.
// Mutations: M7 (last-good cache), M11 (sticky exclusion), M14 (counter misses velocity), M15
// (temperature seeded from a NaN first member), M16 (current divided by presentCount()), M18
// (counter counts std::isnan only); the review round's S7 and S8 (the NaN guard of totalCurrent()
// and current() keyed on the SUM being non-zero instead of on the count).
TEST_CASE("DIAG1 N5: a non-finite member reads exactly as an ABSENT one — every slot, every "
          "kind, every read; never sticky; NaN when none is finite; counted") {
    SUBCASE("position and velocity: every N, every slot, every kind; the hole beside it") {
        int compared = 0;
        for (std::size_t n : {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 21u}) {
            for (std::size_t s = 0; s < n; ++s) {
                for (int kind = 0; kind < 4; ++kind) {
                    CAPTURE(n);
                    CAPTURE(s);
                    CAPTURE(kBadNames[kind]);
                    Bank b;
                    fillFinite(b, n);
                    b.m[s].setPosition(AngleDim{kBadKinds[kind]});
                    b.m[s].setVelocity(AngularVelocity{kBadKinds[kind]});
                    auto pAll = b.ptrs(n);
                    MotorGroup g{std::span<IMotor* const>{pAll.data(), n}};
                    CHECK(g.presentCount() == n);  // no reading is not absence: counts unchanged
                    CHECK(g.absentMask() == 0u);
                    if (n == 1) {
                        // The only member has no reading: a fresh NaN.
                        CHECK(sameBits(g.position().value(), kQNaN));
                        CHECK(sameBits(g.velocity().value(), kQNaN));
                        continue;
                    }
                    auto pHole = b.ptrs(n, std::uint32_t{1} << s);
                    MotorGroup hole{std::span<IMotor* const>{pHole.data(), n}};
                    std::array<IMotor*, Bank::kMax> dp{};
                    std::size_t k = 0;
                    for (std::size_t i = 0; i < n; ++i) {
                        if (i != s) {
                            dp[k++] = &b.m[i];
                        }
                    }
                    MotorGroup dense{std::span<IMotor* const>{dp.data(), k}};
                    REQUIRE(sameBits(g.position().value(), hole.position().value()));
                    REQUIRE(sameBits(g.position().value(), dense.position().value()));
                    REQUIRE(sameBits(g.velocity().value(), hole.velocity().value()));
                    REQUIRE(sameBits(g.velocity().value(), dense.velocity().value()));
                    CHECK(std::isfinite(g.position().value()));

                    // NOT STICKY: the member's next finite reading is counted again — the group
                    // reads exactly as a group of all n with nothing wrong.
                    b.m[s].setPosition(AngleDim{posOf(s)});
                    b.m[s].setVelocity(AngularVelocity{velOf(s)});
                    Bank clean;
                    fillFinite(clean, n);
                    auto pc = clean.ptrs(n);
                    MotorGroup whole{std::span<IMotor* const>{pc.data(), n}};
                    REQUIRE(sameBits(g.position().value(), whole.position().value()));
                    REQUIRE(sameBits(g.velocity().value(), whole.velocity().value()));
                    compared += 6;
                }
            }
        }
        MESSAGE("N5: ", compared, " bitwise group comparisons (position/velocity)");
    }

    SUBCASE("a hole AND a non-finite member together; every present member non-finite is NaN") {
        for (std::size_t n : {3u, 5u, 21u}) {
            for (std::size_t h = 0; h < n; ++h) {
                for (std::size_t s = 0; s < n; ++s) {
                    if (s == h) {
                        continue;
                    }
                    CAPTURE(n);
                    CAPTURE(h);
                    CAPTURE(s);
                    Bank b;
                    fillFinite(b, n);
                    b.m[s].setPosition(AngleDim{kBadKinds[(h + s) % 4]});
                    auto p1 = b.ptrs(n, std::uint32_t{1} << h);
                    auto p2 = b.ptrs(n, (std::uint32_t{1} << h) | (std::uint32_t{1} << s));
                    MotorGroup oneHole{std::span<IMotor* const>{p1.data(), n}};
                    MotorGroup twoHoles{std::span<IMotor* const>{p2.data(), n}};
                    REQUIRE(sameBits(oneHole.position().value(), twoHoles.position().value()));
                    CHECK(oneHole.presentCount() == n - 1);
                    CHECK(oneHole.absentMask() == (std::uint32_t{1} << h));
                }
            }
        }
        // Every PRESENT member non-finite (a hole in slot 1): a fresh NaN, whatever the kinds.
        Bank b;
        fillFinite(b, 5);
        for (std::size_t i = 0; i < 5; ++i) {
            b.m[i].setPosition(AngleDim{kBadKinds[i % 4]});
            b.m[i].setVelocity(AngularVelocity{kBadKinds[(i + 2) % 4]});
        }
        auto p = b.ptrs(5, 0b00010u);
        MotorGroup g{std::span<IMotor* const>{p.data(), 5}};
        CHECK(sameBits(g.position().value(), kQNaN));
        CHECK(sameBits(g.velocity().value(), kQNaN));
    }

    SUBCASE("the last-good-cache kill sequence: a DISTINCTIVE finite value, then no reading") {
        // Read once with the member at 1000 among mates at distinct values (so the 1000 moves the
        // median); then make it non-finite and read again. The answer must be the ABSENT group's.
        // A group that cached the member's last finite reading would still count the 1000. (A
        // fresh group read once with a NaN cannot tell the two apart, and neither can the plant
        // run, where the cached value would equal the mates'.)
        for (std::size_t n : {2u, 3u, 4u, 5u, 21u}) {
            for (std::size_t s = 0; s < n; ++s) {
                for (int kind = 0; kind < 4; ++kind) {
                    CAPTURE(n);
                    CAPTURE(s);
                    CAPTURE(kBadNames[kind]);
                    Bank b;
                    fillFinite(b, n);
                    b.m[s].setPosition(AngleDim{1000.0});
                    b.m[s].setVelocity(AngularVelocity{1000.0});
                    auto p = b.ptrs(n);
                    MotorGroup g{std::span<IMotor* const>{p.data(), n}};
                    const double withIt = g.position().value();
                    (void)g.velocity();
                    b.m[s].setPosition(AngleDim{kBadKinds[kind]});
                    b.m[s].setVelocity(AngularVelocity{kBadKinds[kind]});
                    auto ph = b.ptrs(n, std::uint32_t{1} << s);
                    MotorGroup hole{std::span<IMotor* const>{ph.data(), n}};
                    REQUIRE_FALSE(sameBits(withIt, hole.position().value()));  // it can tell
                    REQUIRE(sameBits(g.position().value(), hole.position().value()));
                    REQUIRE(sameBits(g.velocity().value(), hole.velocity().value()));
                }
            }
        }
    }

    SUBCASE("current, totalCurrent and temperature: the same rule, every N, slot and kind") {
        for (std::size_t n : {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 21u}) {
            for (std::size_t s = 0; s < n; ++s) {
                for (int kind = 0; kind < 4; ++kind) {
                    CAPTURE(n);
                    CAPTURE(s);
                    CAPTURE(kBadNames[kind]);
                    Bank b;
                    fillFinite(b, n);
                    b.m[s].setCurrent(Current{kBadKinds[kind]});
                    b.m[s].setTemperature(kBadKinds[kind]);
                    auto p = b.ptrs(n);
                    MotorGroup g{std::span<IMotor* const>{p.data(), n}};
                    if (n == 1) {
                        CHECK(sameBits(g.current().value(), kQNaN));
                        CHECK(sameBits(g.totalCurrent().value(), kQNaN));
                        CHECK(sameBits(g.temperature(), kQNaN));
                        continue;
                    }
                    auto ph = b.ptrs(n, std::uint32_t{1} << s);
                    MotorGroup hole{std::span<IMotor* const>{ph.data(), n}};
                    REQUIRE(sameBits(g.current().value(), hole.current().value()));
                    REQUIRE(sameBits(g.totalCurrent().value(), hole.totalCurrent().value()));
                    REQUIRE(sameBits(g.temperature(), hole.temperature()));
                    CHECK(std::isfinite(g.current().value()));
                    CHECK(std::isfinite(g.temperature()));
                }
            }
        }
        // The two named cases. A NaN temperature in SLOT 0 used to win (std::max keeps its first
        // argument against a NaN, so the answer depended on the order); a +Inf temperature used
        // to be the hottest member — MOTOR_OVER_TEMP temp=infC.
        Bank b;
        fillFinite(b, 5);
        b.m[0].setTemperature(kQNaN);
        b.m[3].setTemperature(kInf);
        b.m[0].setCurrent(Current{kQNaN});
        auto p = b.ptrs(5);
        MotorGroup g{std::span<IMotor* const>{p.data(), 5}};
        const double hottest = std::max({tempOf(1), tempOf(2), tempOf(4)});
        CHECK(g.temperature() == hottest);
        CHECK(g.current().value() == (ampsOf(1) + ampsOf(2) + ampsOf(3) + ampsOf(4)) / 4.0);
        CHECK(g.current().value() != (ampsOf(1) + ampsOf(2) + ampsOf(3) + ampsOf(4)) / 5.0);
        CHECK(g.totalCurrent().value() == ampsOf(1) + ampsOf(2) + ampsOf(3) + ampsOf(4));
        // Every present member non-finite: NaN for all three (an empty sum is not "0 A").
        for (std::size_t i = 0; i < 5; ++i) {
            b.m[i].setCurrent(Current{kBadKinds[i % 4]});
            b.m[i].setTemperature(kBadKinds[(i + 1) % 4]);
        }
        CHECK(sameBits(g.current().value(), kQNaN));
        CHECK(sameBits(g.totalCurrent().value(), kQNaN));
        CHECK(sameBits(g.temperature(), kQNaN));

        // The other side of that guard (review round, 2026-09-30): a side DRAWING NOTHING is not
        // a side with NO READING. The NaN is decided by how many readings were counted, never by
        // the value they add up to — a group at rest reads exactly +0.0 A, not NaN, and so does a
        // group mixing 0 A members with one member that has no reading. (A guard keyed on the sum
        // would print NaN for every motion record's wheelCurrent while the robot stands still.)
        Bank z;
        for (std::size_t i = 0; i < 5; ++i) {
            z.m[i].setCurrent(Current{0.0});
        }
        auto pz = z.ptrs(5);
        MotorGroup gz{std::span<IMotor* const>{pz.data(), 5}};
        CHECK(sameBits(gz.current().value(), 0.0));
        CHECK(sameBits(gz.totalCurrent().value(), 0.0));
        for (int kind = 0; kind < 4; ++kind) {
            CAPTURE(kBadNames[kind]);
            z.m[2].setCurrent(Current{kBadKinds[kind]});
            CHECK(sameBits(gz.current().value(), 0.0));
            CHECK(sameBits(gz.totalCurrent().value(), 0.0));
        }
    }

    SUBCASE("nonFiniteReads(): exactly one per member reading left out — every read, every kind") {
        for (int kind = 0; kind < 4; ++kind) {
            CAPTURE(kBadNames[kind]);
            const double bad = kBadKinds[kind];
            Bank b;
            fillFinite(b, 5);
            auto p = b.ptrs(5, 0b01000u);  // slot 3 a HOLE: never read, never counted
            MotorGroup g{std::span<IMotor* const>{p.data(), 5}};
            auto readAll = [&g] {
                (void)g.position();
                (void)g.velocity();
                (void)g.current();
                (void)g.totalCurrent();
                (void)g.temperature();
            };
            readAll();
            REQUIRE(g.nonFiniteReads() == 0);  // all finite, a hole: nothing left out

            b.m[2].setPosition(AngleDim{bad});
            (void)g.position();
            CHECK(g.nonFiniteReads() == 1);
            (void)g.velocity();  // its velocity is finite
            CHECK(g.nonFiniteReads() == 1);
            (void)g.position();  // a second read of the same bad member: counted again
            CHECK(g.nonFiniteReads() == 2);

            b.m[2].setVelocity(AngularVelocity{bad});
            (void)g.velocity();
            CHECK(g.nonFiniteReads() == 3);

            b.m[4].setPosition(AngleDim{bad});
            (void)g.position();  // two members left out by one read: two
            CHECK(g.nonFiniteReads() == 5);

            b.m[1].setCurrent(Current{bad});
            (void)g.current();
            CHECK(g.nonFiniteReads() == 6);
            (void)g.totalCurrent();
            CHECK(g.nonFiniteReads() == 7);

            b.m[0].setTemperature(bad);
            (void)g.temperature();
            CHECK(g.nonFiniteReads() == 8);

            // The monitor's samples are not counted (its handling is a separate, later change).
            g.setVoltage(shulib::units::Voltage{8.0});
            (void)g.evaluateDisagreement();
            CHECK(g.nonFiniteReads() == 8);

            // Everything finite again: nothing more is counted — the counter is cumulative, the
            // exclusion is not.
            fillFinite(b, 5);
            readAll();
            CHECK(g.nonFiniteReads() == 8);
            CHECK(g.presentCount() == 4);
            CHECK(g.absentMask() == 0b01000u);
        }
    }
}

// ═══ N6-N8. Through the drive-encoder odometry ══════════════════════════════════════

namespace {

/// Member offsets on a side (radians), exact in binary; sorted they are -0.25, 0, 0.125, 0.375,
/// 0.5. Distinct, so leaving any one out moves the median — the membership step.
constexpr double kOffsets[5] = {0.0, 0.125, -0.25, 0.375, 0.5};

/// Set the first n members of a side to `common` + their offset, with slot `badSlot` (or none, if
/// it is n or more) reading `bad`. Returns the finite median the side should read (the oracle).
double setSide(std::array<FakeMotor, 5>& m, std::size_t n, double common, std::size_t badSlot,
               double bad) {
    double v[5];
    for (std::size_t i = 0; i < n; ++i) {
        v[i] = i == badSlot ? bad : common + kOffsets[i];
        m[i].setPosition(AngleDim{v[i]});
    }
    return oracleMedian(v, n);
}

}  // namespace

// Bug caught: one bad member reaching the odometry — a NaN side frozen for the tick and reported
// NONFINITE+TRAVEL (which ends a motion on ODO_STUCK), a wrong finite side, or the baseline not
// re-arming to the subset median so the NEXT tick carries a phantom. And for a side with no
// reading at all: the verdict must be unchanged — NONFINITE+TRAVEL, frozen, caught up exactly.
TEST_CASE("DIAG1 N6: through DriveEncoderOdometry — one bad member is not an implausible tick; L= "
          "is the finite median's change x ipr; a whole bad side is NONFINITE+TRAVEL") {
    const double ipr = kGeom.inchesPerRadian().value();
    for (std::size_t n : {4u, 5u}) {
        CAPTURE(n);
        std::array<FakeMotor, 5> l{}, r{};
        std::array<IMotor*, 5> lp{&l[0], &l[1], &l[2], &l[3], &l[4]};
        std::array<IMotor*, 5> rp{&r[0], &r[1], &r[2], &r[3], &r[4]};
        MotorGroup left{std::span<IMotor* const>{lp.data(), n}};
        MotorGroup right{std::span<IMotor* const>{rp.data(), n}};
        double baseL = setSide(l, n, 0.0, 99, 0.0);
        double baseR = setSide(r, n, 0.0, 99, 0.0);
        FakeImu imu;
        DriveEncoderOdometry odom{imu, left, right, kGeom, kGeom, Length{12.0}};

        // The script: the side moves 0.5 rad a tick; member 1 reads NaN, then member 1 +Inf,
        // then member 3 -Inf, with clean ticks between; the right side is clean throughout.
        struct Step {
            std::size_t slot;
            double bad;
        };
        const Step script[] = {{99, 0.0}, {1, kQNaN}, {1, kInf}, {99, 0.0}, {3, -kInf},
                               {99, 0.0}, {0, kNegNaN}, {99, 0.0}};
        double common = 0.0;
        int badTicks = 0;
        for (const Step& st : script) {
            common += 0.5;
            CAPTURE(common);
            CAPTURE(st.slot);
            const double medL = setSide(l, n, common, st.slot, st.bad);
            const double medR = setSide(r, n, common, 99, 0.0);
            const Pose2d before = odom.pose();
            odom.update();
            const double dL = (medL - baseL) * ipr;  // a CHANGE, not the median itself
            const double dR = (medR - baseR) * ipr;
            baseL = medL;
            baseR = medR;
            const OdomTickAudit a = odom.lastTickAudit();
            REQUIRE(std::isfinite(medL));
            REQUIRE_FALSE(odom.lastDeltaImplausible());
            REQUIRE_FALSE(a.tripped());
            REQUIRE(sameBits(a.travelA.value(), dL));
            REQUIRE(sameBits(a.travelB.value(), dR));
            // Position integrates — it does not freeze — by exactly the arc the two sides give.
            const auto d = shulib::localization::arcStep(
                {Length{0.5 * (dL + dR)}, Length{0.0}}, Angle{}, Angle{});
            CHECK(sameBits(odom.pose().x().value(), before.x().value() + d.dx.value()));
            CHECK(sameBits(odom.pose().y().value(), before.y().value() + d.dy.value()));
            badTicks += st.slot < n ? 1 : 0;
        }
        CHECK(badTicks == 4);

        // Every member of the LEFT side non-finite: no reading at all — the verdict is today's,
        // NONFINITE+TRAVEL, position frozen, the baseline not re-armed. (This tick the sides
        // move 0.25 rad, so R= is 0.40625 in — no rounding tie in the golden.)
        common += 0.25;
        for (std::size_t i = 0; i < n; ++i) {
            l[i].setPosition(AngleDim{kBadKinds[i % 4]});
        }
        const double medR = setSide(r, n, common, 99, 0.0);
        const Pose2d frozen = odom.pose();
        odom.update();
        const OdomTickAudit a = odom.lastTickAudit();
        CHECK(odom.lastDeltaImplausible());
        CHECK(a.nonFinite);
        CHECK(a.travel);
        CHECK_FALSE(a.rotation);
        CHECK(sameBits(a.travelA.value(), kQNaN * ipr));
        CHECK(a.travelB.value() == 0.40625);
        CHECK(render(a)
              == " why=NONFINITE+TRAVEL dth=0.000rad h0=0.000 h1=0.000 L=NaNin R=0.406in");
        CHECK(odom.pose().x().value() == frozen.x().value());
        CHECK(odom.pose().y().value() == frozen.y().value());
        baseR = medR;
        // The next all-finite tick integrates ACROSS the gap, exactly: 0.25 + 0.5 rad.
        common += 0.5;
        const double medL2 = setSide(l, n, common, 99, 0.0);
        const double medR2 = setSide(r, n, common, 99, 0.0);
        odom.update();
        CHECK_FALSE(odom.lastDeltaImplausible());
        CHECK(sameBits(odom.lastTickAudit().travelA.value(), (medL2 - baseL) * ipr));
        CHECK(odom.lastTickAudit().travelA.value() == 0.75 * ipr);  // both ticks' travel
        CHECK(sameBits(odom.lastTickAudit().travelB.value(), (medR2 - baseR) * ipr));
        baseL = medL2;

        // Every member at +Inf: class C's one visible change — the side reads a fresh NaN, so
        // the line says L=NaNin where it used to say L=+Infin.
        for (std::size_t i = 0; i < n; ++i) {
            l[i].setPosition(AngleDim{kInf});
        }
        odom.update();
        CHECK(odom.lastTickAudit().nonFinite);
        CHECK(render(odom.lastTickAudit()).find(" L=NaNin R=") != std::string::npos);
    }
}

// Bug caught: the membership step (A4 register HA-165) mis-sized or not carried back — the
// median of an odd side with one member left out is the midpoint of the two values then in the
// middle (the middle value and a neighbour, or the middle value's two neighbours when the member
// left out IS the middle one), and the tick the member returns carries the step back, so the two
// ticks' travel is the full-set travel. And a step bigger than the travel bound: reported as
// TRAVEL, ONE fault line. And with SEVERAL left out (review round, 2026-09-30): the rest's median,
// anywhere up to the members' spread — the worst case, one drifted member read alone, passes the
// gate SILENTLY under 36 in (where the same +Inf members used to reach the middle and freeze the
// tick as NONFINITE+TRAVEL), and is TRAVEL past it.
TEST_CASE("DIAG1 N7: the membership step, exact — HA-165's size, its return, and a step past the "
          "bound on a two-member side") {
    SUBCASE("five members, the 80 playing port 18: -0.25 then +0.25; with motion, the sum") {
        for (const double move : {0.0, 0.5}) {
            CAPTURE(move);
            std::array<FakeMotor, 5> l{}, r{};
            std::array<IMotor*, 5> lp{&l[0], &l[1], &l[2], &l[3], &l[4]};
            std::array<IMotor*, 5> rp{&r[0], &r[1], &r[2], &r[3], &r[4]};
            const double pos[5] = {100.0, 100.5, 101.0, 101.5, 80.0};
            for (std::size_t i = 0; i < 5; ++i) {
                l[i].setPosition(AngleDim{pos[i]});
            }
            MotorGroup left{std::span<IMotor* const>{lp}};
            MotorGroup right{std::span<IMotor* const>{rp}};
            REQUIRE(left.position().value() == 100.5);
            FakeImu imu;
            DriveEncoderOdometry odom{imu, left, right, kUnit, kUnit, Length{12.0}};

            // Tick 1: everything moves `move`; member 2 (the 101.0) reads NaN.
            for (std::size_t i = 0; i < 5; ++i) {
                l[i].setPosition(AngleDim{i == 2 ? kQNaN : pos[i] + move});
            }
            odom.update();
            const double step1 = odom.lastTickAudit().travelA.value();
            CHECK(step1 == move - 0.25);  // midpoint(100, 100.5) - 100.5, plus the motion
            CHECK_FALSE(odom.lastDeltaImplausible());
            // Tick 2: everything moves again; member 2 is back.
            for (std::size_t i = 0; i < 5; ++i) {
                l[i].setPosition(AngleDim{pos[i] + 2.0 * move});
            }
            odom.update();
            const double step2 = odom.lastTickAudit().travelA.value();
            CHECK(step2 == move + 0.25);  // the step carried back
            CHECK_FALSE(odom.lastDeltaImplausible());
            CHECK(step1 + step2 == 2.0 * move);  // == the full set's travel over the two ticks
            // HA-165's bound: at most half the larger gap beside the middle value (0.5 each side).
            CHECK(std::abs(step1 - move) <= 0.5 * 0.5);
        }
    }

    SUBCASE("two members [0, 100] at 1 in/rad: member 1 no reading -> L=-50 TRAVEL; one line") {
        for (const std::size_t badSlot : {std::size_t{1}, std::size_t{0}}) {
            CAPTURE(badSlot);
            std::array<FakeMotor, 2> l{}, r{};
            std::array<IMotor*, 2> lp{&l[0], &l[1]};
            std::array<IMotor*, 2> rp{&r[0], &r[1]};
            l[1].setPosition(AngleDim{100.0});
            MotorGroup left{std::span<IMotor* const>{lp}};
            MotorGroup right{std::span<IMotor* const>{rp}};
            FakeImu imu;
            DriveEncoderOdometry odom{imu, left, right, kUnit, kUnit, Length{12.0}};
            FakeTelemetrySink sink;
            FakeClock clock{Time{1.0}};
            FaultLatch latch{sink, clock};
            HealthMonitor monitor{latch};
            auto tick = [&] {
                odom.update();
                monitor.tick({.odomImplausible = odom.lastDeltaImplausible(),
                              .odomAudit = odom.lastTickAudit()});
            };

            l[badSlot].setPosition(AngleDim{kQNaN});
            tick();  // the side falls from 50 to the other member's value
            const std::string sign = badSlot == 1 ? "-" : "";
            CHECK(odom.lastTickAudit().travel);
            CHECK_FALSE(odom.lastTickAudit().nonFinite);
            CHECK(odom.lastTickAudit().travelA.value() == (badSlot == 1 ? -50.0 : 50.0));
            CHECK(render(odom.lastTickAudit())
                  == " why=TRAVEL dth=0.000rad h0=0.000 h1=0.000 L=" + sign + "50.000in R=0.000in");

            l[badSlot].setPosition(AngleDim{badSlot == 1 ? 100.0 : 0.0});
            tick();  // the step back: still implausible — the SAME episode
            CHECK(odom.lastTickAudit().travel);
            CHECK(odom.lastTickAudit().travelA.value() == (badSlot == 1 ? 50.0 : -50.0));
            tick();  // nothing moved: plausible, the episode over
            CHECK_FALSE(odom.lastDeltaImplausible());
            tick();
            const std::vector<std::string> errs = errorLines(sink);
            REQUIRE(errs.size() == 1);  // the rising edge only
            CHECK(errs[0] == "fault=ODO_STUCK n=1 FIRST implausible odometry tick why=TRAVEL "
                             "dth=0.000rad h0=0.000 h1=0.000 L="
                                 + sign + "50.000in R=0.000in");
        }
    }

    SUBCASE("the MIDDLE member left out: the midpoint of its two neighbours") {
        // [80, 100, 100.5, 102, 103]: the middle value is 100.5. Left out, the median is
        // midpoint(100, 102) = 101 — a step of +0.5, within half the larger gap beside the
        // middle value (1.5 / 2 = 0.75), and not the midpoint of the middle value and a
        // neighbour (100.25 or 101.25), which is what the header first said.
        std::array<FakeMotor, 5> m{};
        std::array<IMotor*, 5> p{&m[0], &m[1], &m[2], &m[3], &m[4]};
        const double pos[5] = {103.0, 80.0, 100.5, 100.0, 102.0};
        for (std::size_t i = 0; i < 5; ++i) {
            m[i].setPosition(AngleDim{pos[i]});
        }
        MotorGroup g{std::span<IMotor* const>{p}};
        REQUIRE(g.position().value() == 100.5);
        m[2].setPosition(AngleDim{kQNaN});
        CHECK(g.position().value() == 101.0);
        CHECK(std::abs(g.position().value() - 100.5) <= 0.5 * 1.5);
    }

    SUBCASE("several left out: the drifted member read alone — silent under 36 in, TRAVEL past") {
        // Robot two's radius (1.625 in/rad). Four of five LEFT members read +Inf and the one left
        // is the drifted one (the 80, HA-130's port 18 playing it): the side steps from the full
        // median 100.5 to 80, -20.5 rad, with NOTHING moving — within the members' spread (101.5 -
        // 80 = 21.5 rad), the bound the header gives for several left out. -20.5 x 1.625 =
        // -33.3125 in is under maxTickTravel (36 in), so the tick is PLAUSIBLE and the pose moves
        // by half of it: the silent worst case. On the unchanged tree those four infinities
        // reached the middle ({80, +Inf x4} -> +Inf) and the tick froze as NONFINITE+TRAVEL —
        // defined behaviour there (an infinity sorts), but the suite runs the new header only, so
        // that outcome is observed on the unchanged tree and logged, never asserted here.
        for (const double lone : {80.0, 69.0}) {
            CAPTURE(lone);
            std::array<FakeMotor, 5> l{}, r{};
            std::array<IMotor*, 5> lp{&l[0], &l[1], &l[2], &l[3], &l[4]};
            std::array<IMotor*, 5> rp{&r[0], &r[1], &r[2], &r[3], &r[4]};
            const double pos[5] = {100.0, 100.5, 101.0, 101.5, lone};
            for (std::size_t i = 0; i < 5; ++i) {
                l[i].setPosition(AngleDim{pos[i]});
            }
            MotorGroup left{std::span<IMotor* const>{lp}};
            MotorGroup right{std::span<IMotor* const>{rp}};
            REQUIRE(left.position().value() == 100.5);
            FakeImu imu;
            DriveEncoderOdometry odom{imu, left, right, kGeom, kGeom, Length{12.0}};
            const double ipr = kGeom.inchesPerRadian().value();

            for (std::size_t i = 0; i < 4; ++i) {
                l[i].setPosition(AngleDim{kInf});
            }
            REQUIRE(left.position().value() == lone);  // read ALONE
            odom.update();
            const OdomTickAudit a = odom.lastTickAudit();
            const double step = (lone - 100.5) * ipr;  // exact in binary for both values
            CHECK(sameBits(a.travelA.value(), step));
            CHECK(std::abs(lone - 100.5) <= 101.5 - lone);  // within the present members' spread
            CHECK_FALSE(a.nonFinite);
            // The gate REPORTS a finite tick and never withholds it (the odometry header: only a
            // non-finite tick freezes position), so the pose moves by half the step either way;
            // what the bound decides is whether the tick is FLAGGED.
            CHECK(odom.pose().x().value() == 0.5 * step);
            if (lone == 80.0) {
                // -33.3125 in: under the bound, so NOTHING is flagged — the silent worst case.
                CHECK(step == -33.3125);
                CHECK_FALSE(odom.lastDeltaImplausible());
            } else {
                // 69: -31.5 rad, -51.1875 in — past the bound: TRAVEL, which raises ODO_STUCK
                // and, under the default abort mask, ends the active motion (HA-123).
                CHECK(step == -51.1875);
                CHECK(odom.lastDeltaImplausible());
                CHECK(a.travel);
            }
            // The members return with nothing moved: the step is carried back, exactly, and the
            // pose is where it started. (Past the bound the return tick is TRAVEL too.)
            for (std::size_t i = 0; i < 4; ++i) {
                l[i].setPosition(AngleDim{pos[i]});
            }
            odom.update();
            CHECK(sameBits(odom.lastTickAudit().travelA.value(), -step));
            CHECK(odom.pose().x().value() == 0.0);
            CHECK(odom.lastDeltaImplausible() == (lone != 80.0));
        }
    }
}

// Bug caught: the side readings, the verdict or any audit value drifting from what the gate's
// own expressions give when members come and go — checked every tick of a 1200-tick script that
// mixes clean ticks, one bad member at a random slot, several at once, a whole side with no
// reading, big jumps and big heading steps, against an oracle computed from the script alone.
TEST_CASE("DIAG1 N8: 1200 scripted ticks over two groups of five — side readings, verdict, every "
          "flag and value match the oracle, every tick") {
    std::array<FakeMotor, 5> l{}, r{};
    std::array<IMotor*, 5> lp{&l[0], &l[1], &l[2], &l[3], &l[4]};
    std::array<IMotor*, 5> rp{&r[0], &r[1], &r[2], &r[3], &r[4]};
    MotorGroup left{std::span<IMotor* const>{lp}};
    MotorGroup right{std::span<IMotor* const>{rp}};
    double posL[5];
    double posR[5];
    for (std::size_t i = 0; i < 5; ++i) {
        posL[i] = kOffsets[i];
        posR[i] = -kOffsets[i];
        l[i].setPosition(AngleDim{posL[i]});
        r[i].setPosition(AngleDim{posR[i]});
    }
    FakeImu imu;
    const double ipr = kGeom.inchesPerRadian().value();
    DriveEncoderOdometry odom{imu, left, right, kGeom, kGeom, Length{12.0}};
    double baseL = oracleMedian(posL, 5);
    double baseR = oracleMedian(posR, 5);
    Lcg rng{20260929};
    double h = 0.0;
    Angle prev{};
    int nClean = 0, nOne = 0, nSeveral = 0, nWhole = 0, nRot = 0, nTravel = 0, nNonFinite = 0;
    for (int tick = 0; tick < 1200; ++tick) {
        CAPTURE(tick);
        // The world: both sides roll, each member with a little slip of its own; now and then a
        // big jump (TRAVEL) or a big heading step (ROT).
        const double big = rng.unit();
        const double stepL = big > 0.96 ? rng.between(20.0, 40.0) : rng.between(-0.6, 0.6);
        const double stepR = big < 0.03 ? -rng.between(20.0, 40.0) : rng.between(-0.6, 0.6);
        for (std::size_t i = 0; i < 5; ++i) {
            posL[i] += stepL + rng.between(-0.01, 0.01);
            posR[i] += stepR + rng.between(-0.01, 0.01);
        }
        h += rng.unit() < 0.05 ? rng.between(1.7, 3.0) * (rng.unit() < 0.5 ? -1.0 : 1.0)
                               : rng.between(-0.05, 0.05);
        // The readings: per side, clean, one bad member, several (not all), or none readable.
        double readL[5];
        double readR[5];
        auto readings = [&](const double* truth, double* read, bool countIt) {
            std::copy(truth, truth + 5, read);
            const double c = rng.unit();
            if (c < 0.50) {
                nClean += countIt ? 1 : 0;
            } else if (c < 0.78) {
                read[rng.below(5)] = kBadKinds[rng.below(4)];
                nOne += countIt ? 1 : 0;
            } else if (c < 0.92) {
                const std::size_t k = 2 + rng.below(3);  // 2..4 of 5
                std::size_t idx[5] = {0, 1, 2, 3, 4};
                for (std::size_t j = 0; j < k; ++j) {  // k distinct slots
                    std::swap(idx[j], idx[j + rng.below(5 - j)]);
                    read[idx[j]] = kBadKinds[rng.below(4)];
                }
                nSeveral += countIt ? 1 : 0;
            } else {
                for (std::size_t j = 0; j < 5; ++j) {
                    read[j] = kBadKinds[rng.below(4)];
                }
                nWhole += countIt ? 1 : 0;
            }
        };
        readings(posL, readL, true);
        readings(posR, readR, false);
        for (std::size_t i = 0; i < 5; ++i) {
            l[i].setPosition(AngleDim{readL[i]});
            r[i].setPosition(AngleDim{readR[i]});
        }
        const Angle h1 = Angle::radians(h);
        imu.setHeading(h1);

        // The side readings the odometry will take, against the oracle.
        const double medL = oracleMedian(readL, 5);
        const double medR = oracleMedian(readR, 5);
        REQUIRE(sameBits(left.position().value(), medL));
        REQUIRE(sameBits(right.position().value(), medR));
        odom.update();

        // The oracle: the gate's expressions, copied (drive_encoder_odometry.hpp update()).
        const double dL = (medL - baseL) * ipr;
        const double dR = (medR - baseR) * ipr;
        if (std::isfinite(medL)) baseL = medL;
        if (std::isfinite(medR)) baseR = medR;
        const double dth = prev.errorTo(h1);
        const auto d =
            shulib::localization::arcStep({Length{0.5 * (dL + dR)}, Length{0.0}}, prev, h1);
        const bool expNonFinite = !(std::isfinite(d.dx.value()) && std::isfinite(d.dy.value()));
        const bool expRot = std::abs(dth) > 0.5 * Angle::kPi;
        const bool expTravel = !(std::abs(dL) <= 36.0 && std::abs(dR) <= 36.0);
        prev = h1;

        const OdomTickAudit a = odom.lastTickAudit();
        REQUIRE(odom.lastDeltaImplausible() == a.tripped());
        REQUIRE(a.nonFinite == expNonFinite);
        REQUIRE(a.rotation == expRot);
        REQUIRE(a.travel == expTravel);
        REQUIRE(sameBits(a.dTheta.value(), dth));
        REQUIRE(sameBits(a.travelA.value(), dL));
        REQUIRE(sameBits(a.travelB.value(), dR));
        nRot += a.rotation ? 1 : 0;
        nTravel += a.travel ? 1 : 0;
        nNonFinite += a.nonFinite ? 1 : 0;
    }
    MESSAGE("N8 left side: clean ", nClean, " one-bad ", nOne, " several-bad ", nSeveral,
            " none-readable ", nWhole, "; verdicts: rot ", nRot, " travel ", nTravel,
            " nonfinite ", nNonFinite);
    CHECK(nOne >= 250);
    CHECK(nSeveral >= 100);
    CHECK(nWhole >= 60);
    CHECK(nRot >= 30);
    CHECK(nTravel >= 100);
    CHECK(nNonFinite >= 100);
}

// ═══ N9. End to end on the host test plant, through the scheduler ═══════════════════

namespace {

/// The extra fakes of a two-wheel drive and the two groups over them — COPIED from
/// motor_group_plant_test.cpp's `CoupledMembers` (its absent-slot parameter dropped: not used
/// here). The plant writes one wheel's shaft into every member of that wheel.
struct CoupledMembers {
    static constexpr std::size_t kMaxExtra = 20;
    std::array<FakeMotor, 2 * kMaxExtra> extra{};
    std::array<IMotor*, kMaxExtra + 1> leftPtrs{};
    std::array<IMotor*, kMaxExtra + 1> rightPtrs{};
    std::size_t perWheel;
    MotorGroup left;
    MotorGroup right;

    CoupledMembers(shulib::sim::SimHarness& h, std::size_t membersPerWheel)
        : perWheel{membersPerWheel}, left{wire(h, 0, leftPtrs)}, right{wire(h, 1, rightPtrs)} {}

    /// Member `k` (>= 1) of wheel `w` — the fake the plant's setMemberFault(w, k, …) names.
    [[nodiscard]] FakeMotor& extraMember(int w, std::size_t k) {
        return extra[static_cast<std::size_t>(w) * kMaxExtra + (k - 1)];
    }

private:
    std::span<IMotor* const> wire(shulib::sim::SimHarness& h, int wheel,
                                  std::array<IMotor*, kMaxExtra + 1>& ptrs) {
        ptrs[0] = &h.motor(wheel);
        std::array<FakeMotor*, kMaxExtra> ex{};
        for (std::size_t i = 1; i < perWheel; ++i) {
            FakeMotor& f = extra[static_cast<std::size_t>(wheel) * kMaxExtra + (i - 1)];
            ptrs[i] = &f;
            ex[i - 1] = &f;
        }
        h.plant().attachCoupledMembers(wheel,
                                       std::span<FakeMotor* const>{ex.data(), perWheel - 1});
        return std::span<IMotor* const>{ptrs.data(), perWheel};
    }
};

/// Everything an N9 run produces that must match the baseline bit for bit.
struct N9Trace {
    std::vector<shulib::sim::TruthSample> truth;
    std::vector<double> volts;     // both primaries' commanded volts, per pace
    std::vector<double> groupPos;  // the LEFT group's position(), per pace, after the injection
    ExitReason exit = ExitReason::Running;
    FaultCode abortFault = FaultCode::None;
    int paces = 0;
    int odoStuckLines = 0;
    int disagreeRaises = 0;
    int leftNonFiniteReads = 0;
};

/// PlantPacer (motion_test_rig.hpp :148-170), COPIED, with two additions after each plant step:
/// it records the trace, and it runs the injection — at pace `injectAt` the left wheel's member 2
/// is made Frozen (the plant then never writes it) and its FakeMotor's position is set to `bad`;
/// at pace `restoreAt` the fault is lifted and the next plant step writes the member again.
struct InjectingPacer final : shulib::motion::ITickPacer {
    shulib::sim::SimHarness* h;
    MotorGroup* leftGroup;
    FakeMotor* victim;
    N9Trace* trace;
    int injectAt = -1;
    int restoreAt = -1;
    double bad = 0.0;
    int paces = 0;

    InjectingPacer(shulib::sim::SimHarness& harness, MotorGroup& lg, FakeMotor& v, N9Trace& t)
        : h{&harness}, leftGroup{&lg}, victim{&v}, trace{&t} {}

    void pace() override {
        if (paces >= 5000) {
            throw std::runtime_error("InjectingPacer: pace cap exceeded — a wait failed to end");
        }
        ++paces;
        h->plant().step(Time{0.01});
        if (paces == injectAt) {
            h->plant().setMemberFault(0, 2, shulib::sim::MemberFault::Frozen);
            victim->setPosition(AngleDim{bad});
        }
        if (paces == restoreAt) {
            h->plant().setMemberFault(0, 2, shulib::sim::MemberFault::None);
        }
        trace->truth.push_back(h->sample());
        trace->volts.push_back(h->motor(0).commandedVoltage().value());
        trace->volts.push_back(h->motor(1).commandedVoltage().value());
        trace->groupPos.push_back(leftGroup->position().value());
    }
};

/// Robot two's shape: a tank, five coupled members a wheel, the two groups as the context's
/// drive motors AND as deps.motorGroups (GroupRig's composition, motor_group_plant_test.cpp
/// :100-143), over DriveEncoderOdometry with DIAG1's DriveEncoderRig geometry
/// (odom_tick_audit_test.cpp :1193-1222) — the full Localizer / latch / monitor / Chassis stack.
struct N9Rig {
    TankKinematics kin{Length{12.0}};
    shulib::sim::SimHarness h;
    CoupledMembers m;
    std::array<IMotor*, 2> drive;
    shulib::chassis::RobotContext ctx;
    DriveGeometry geom;
    DriveEncoderOdometry odom;
    shulib::localization::ComplementaryFusion fusion;
    shulib::localization::Localizer loc;
    FakeTelemetrySink faultSink;
    FaultLatch latch;
    HealthMonitor health;
    std::array<MotorGroup*, 2> groups;
    shulib::motion::MotionDeps deps;
    N9Trace trace;
    InjectingPacer pacer;
    shulib::chassis::Chassis chassis;

    N9Rig()
        : h{kin, motion_rig::plantConfig(), nullptr, nullptr},
          m{h, 5},
          drive{&m.left, &m.right},
          ctx{{.clock = &h.clock(),
               .driveMotors = drive,
               .imu = &h.imu(),
               .gps = &h.gps(),
               .battery = &h.battery(),
               .telemetry = &h.context().telemetry(),
               .tags = &h.context().tags(),
               .vision = &h.context().vision()}},
          geom{DriveGeometry::fromDiameter(motion_rig::plantConfig().plant.driveWheelDiameter,
                                           1.0)},
          odom{h.imu(), m.left, m.right, geom, geom, Length{12.0}},
          fusion{},
          loc{h.clock(), h.imu(), odom, fusion},
          latch{faultSink, h.clock()},
          health{latch},
          groups{&m.left, &m.right},
          deps{.ctx = &ctx,
               .localizer = &loc,
               .kinematics = &kin,
               .faults = &latch,
               .health = &health,
               .motorGroups = groups},
          pacer{h, m.left, m.extraMember(0, 2), trace},
          chassis{deps, pacer, motion_rig::chassisConfig()} {}

    /// The motion DIAG1 F3 runs, through the scheduler, under the default abort mask.
    N9Trace& run() {
        shulib::motion::MotionScheduler& sched = chassis.scheduler();
        shulib::motion::MoveToPose mv{sched.deps(), Pose2d{Length{40.0}, Length{0.0}, Angle{}},
                                      motion_rig::motionConfig(), 6.0};
        sched.async(mv);
        trace.exit = sched.waitUntilSettled();
        trace.abortFault = sched.lastCompleted().abortFault;
        trace.paces = pacer.paces;
        for (const std::string& s : errorLines(faultSink)) {
            trace.odoStuckLines += s.rfind("fault=ODO_STUCK", 0) == 0 ? 1 : 0;
        }
        trace.disagreeRaises = latch.raiseCount(FaultCode::MotorGroupDisagree);
        trace.leftNonFiniteReads = m.left.nonFiniteReads();
        return trace;
    }
};

[[nodiscard]] bool sameTruth(const std::vector<shulib::sim::TruthSample>& a,
                             const std::vector<shulib::sim::TruthSample>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    return a.empty()
           || std::memcmp(a.data(), b.data(), a.size() * sizeof(shulib::sim::TruthSample)) == 0;
}

}  // namespace

// Bug caught: one member's non-finite reading ending a motion. On the unchanged tree a NaN in one
// member of five made the side's median NaN ({x, x, NaN, x, x} -> NaN on this library), the tick
// read NONFINITE, ODO_STUCK was raised, and the default abort mask ended the MoveToPose
// (Cancelled, abortFault OdoStuck — the RED run, logged). The two infinite sub-runs were already
// green there (one infinity of five sorts to an end): they are regression pins, not evidence.
TEST_CASE("DIAG1 N9: one member of five reads NaN / +Inf / -Inf for 31 ticks mid-motion — the "
          "scheduled MoveToPose is bit-identical to the clean run, with no ODO_STUCK") {
    N9Rig baseRig;
    const N9Trace& base = baseRig.run();
    MESSAGE("N9 baseline: exit ", static_cast<int>(base.exit), " paces ", base.paces,
            " disagree raises ", base.disagreeRaises);
    REQUIRE(base.exit == ExitReason::Settled);
    REQUIRE(base.odoStuckLines == 0);
    // P0 mid-motion: the member goes bad at pace 60 (0.6 s in) and comes back at pace 90 — the
    // baseline is still driving well after that.
    const int p0 = 60;
    REQUIRE(base.paces > p0 + 60);

    // One sub-run per kind, each on a fresh rig and in its own SUBCASE, so each can be run and
    // read alone (`-sc="+Inf*"`): a failed REQUIRE ends the whole case's run, and one kind's
    // failure would otherwise hide the others' outcomes.
    auto subRun = [&](int kind) {
        CAPTURE(kBadNames[kind]);
        N9Rig rig;
        rig.pacer.injectAt = p0;
        rig.pacer.restoreAt = p0 + 30;
        rig.pacer.bad = kBadKinds[kind];
        const N9Trace& t = rig.run();
        MESSAGE("N9 ", kBadNames[kind], ": exit ", static_cast<int>(t.exit), " abortFault ",
                static_cast<int>(t.abortFault), " paces ", t.paces, " ODO_STUCK lines ",
                t.odoStuckLines, " MOTOR_GROUP_DISAGREE raises ", t.disagreeRaises,
                " left nonFiniteReads ", t.leftNonFiniteReads);
        CHECK(t.odoStuckLines == 0);
        CHECK(t.abortFault == FaultCode::None);
        REQUIRE(t.exit == base.exit);
        REQUIRE(t.paces == base.paces);
        REQUIRE(sameTruth(base.truth, t.truth));
        REQUIRE(base.volts.size() == t.volts.size());
        for (std::size_t i = 0; i < base.volts.size(); ++i) {
            REQUIRE(sameBits(base.volts[i], t.volts[i]));
        }
        REQUIRE(base.groupPos.size() == t.groupPos.size());
        for (std::size_t i = 0; i < base.groupPos.size(); ++i) {
            CAPTURE(i);
            REQUIRE(sameBits(base.groupPos[i], t.groupPos[i]));
        }
        // The exclusion really happened (and was counted), and the member really was bad.
        CHECK(t.leftNonFiniteReads > 0);
        CHECK(base.leftNonFiniteReads == 0);
    };
    SUBCASE("NaN (the RED run on the unchanged tree)") { subRun(0); }
    SUBCASE("+Inf (a regression pin: green on the unchanged tree too)") { subRun(2); }
    SUBCASE("-Inf (a regression pin: green on the unchanged tree too)") { subRun(3); }
}

// ═══ N10. The drivetrain calibrator ═════════════════════════════════════════════════

namespace {

using calib_rig::CalibRig;

/// What one N10 run ended as, and the numbers the case compares.
struct N10Run {
    shulib::calibration::Phase end = shulib::calibration::Phase::Idle;
    int movedWhileWatched = -1;  ///< status().movedNow on the watched tick after the injection
    bool readyAfterInjection = false;
};

/// Robot two's calibration with a scripted operator (calibration_test_rig.hpp), stopped at the
/// FIRST tick pushReady() is true — at rest, the push done, DONE about to be touched. There
/// `inject` runs; then ONE more watched tick (so status() shows the push as the calibrator now
/// reads it); then DONE by hand (the end that led is the front: no WHICH END since CAL1 Part B);
/// then `afterDone`; then the rig's operator
/// finishes the run. A clean run takes the SAME path with no injection, so the two are compared
/// like for like.
N10Run runN10(CalibRig& rig, const std::function<void(CalibRig&)>& inject,
              const std::function<void(CalibRig&)>& afterDone,
              const std::function<void(CalibRig&, const calib_rig::Script&)>& hook = nullptr) {
    using shulib::calibration::Answer;
    using shulib::calibration::Phase;
    N10Run out;
    rig.cal.start(calib_rig::facts());
    calib_rig::RunOptions toReady;
    toReady.stopWhen = [](CalibRig& r) { return r.cal.pushReady(); };
    REQUIRE(calib_rig::runCalibration(rig, toReady) == Phase::PushCapture);
    REQUIRE(rig.cal.pushReady());
    if (inject) {
        inject(rig);
    }
    rig.cal.tick();
    rig.h.plant().step(Time{calib_rig::kDt});
    out.movedWhileWatched = rig.cal.status().movedNow;
    out.readyAfterInjection = rig.cal.pushReady();
    rig.cal.answer(Answer::Done);
    if (afterDone) {
        afterDone(rig);
    }
    calib_rig::RunOptions rest;
    rest.hook = hook;
    out.end = calib_rig::runCalibration(rig, rest);
    return out;
}

/// Port 13 — wheel 0 (LEFT), member 2 — held at `bad` from the first ready tick on.
void freezePort13(CalibRig& rig, double bad) {
    rig.h.plant().setMemberFault(0, 2, shulib::sim::MemberFault::Frozen);
    rig.member(0, 2).setPosition(AngleDim{bad});
}

[[nodiscard]] std::string pushLine(const shulib::calibration::PortObservation& o) {
    char buf[128];
    shulib::calibration::formatPushRecord(o, buf, sizeof buf);
    return std::string{buf};
}

}  // namespace

// Bug caught: a candidate whose push reading is not finite being made a DRIVE port with a sign
// read off PROS's failure sentinel (±Inf passed `std::abs(d) >= threshold`), the push's counts
// and its medians describing different members (the DONE gate's count kept it while the median
// left it out), or the port being dropped IN SILENCE (the old NaN behaviour). And the net under
// the rule: a real drive motor left out that recovers is back-driven by its train and refused
// NON_DRIVE_MOVED naming it; and the spin's side travel over a member with no reading.
// Mutations: M12 (pushSummary bypasses the helper), M13 (the classification bypasses it), M17
// (the push record's word back to `still`), M19 (the no-reading warning not appended); the review
// round's S2 (no " | " before a second no-reading sentence), P1 (a no-reading port's travel back
// to sign x reading, a NaN), P2 (NON_DRIVE_MOVED's reason back to "did not move on the push") and
// P3 (the +0.0 written only for a non-finite reading, so a still port read backward is -0.0).
TEST_CASE("DIAG1 N10: the calibrator — a candidate with no reading at DONE is not a drive port, "
          "is NAMED, and leaves the LEFT scale bit-identical; the net; the spin's side travel") {
    using shulib::calibration::CalibratorConfig;
    using shulib::calibration::Phase;
    using shulib::calibration::Refusal;
    using shulib::calibration::refusalName;
    const std::size_t i13 = CalibRig::indexOf(0, 2);

    // The clean run, along the same path.
    CalibRig clean{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
    const N10Run c = runN10(clean, nullptr, nullptr);
    REQUIRE(c.end == Phase::Done);
    REQUIRE(c.movedWhileWatched == 10);
    REQUIRE(clean.cal.report().drivePortCount == 10);
    REQUIRE(clean.cal.result().left.count == 5);

    SUBCASE("(a) held to the end: +Inf, -Inf, NaN, -NaN") {
        auto held = [&](int kind) {
            CAPTURE(kBadNames[kind]);
            CalibRig rig{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
            const N10Run a = runN10(
                rig, [kind](CalibRig& r) { freezePort13(r, kBadKinds[kind]); }, nullptr);
            const auto& rep = rig.cal.report();
            MESSAGE("N10(a) ", kBadNames[kind], ": end ", static_cast<int>(a.end), " refusal ",
                    std::string{refusalName(rig.cal.refusal())}, " port ", rig.cal.refusalPort(),
                    " movedNow ", a.movedWhileWatched, " drivePortCount ",
                    static_cast<int>(rep.drivePortCount), " sign13 ",
                    static_cast<int>(rep.ports[i13].sign), " warning '",
                    std::string{rig.cal.warning()}, "' line '", pushLine(rep.ports[i13]), "'");
            // While the push is watched the live count leaves it out, and DONE is still offered.
            CHECK(a.movedWhileWatched == 9);
            CHECK(a.readyAfterInjection);
            // At DONE: no sign, not a drive port; the captured count equals the live one.
            CHECK(rep.ports[i13].sign == 0);
            // ... and no travel: exactly +0.0, as for any port that is not a drive motor — not
            // 0 x its reading, which is 0 x +-Inf or 0 x NaN, a NaN (review round, 2026-09-30).
            CHECK(sameBits(rep.ports[i13].pushTravelRad, 0.0));
            CHECK(rep.drivePortCount == 9);
            CHECK(static_cast<int>(rep.drivePortCount) == a.movedWhileWatched);
            // The PUSH TABLE line keeps the port and its reading, and says NO READING.
            const std::string line = pushLine(rep.ports[i13]);
            CHECK(line.rfind("push port 13 travel ", 0) == 0);
            CHECK(line.size() >= 19);
            CHECK(line.substr(line.size() - 19) == " NO READING sign +0");
            // The run finishes without it, and says so.
            REQUIRE_MESSAGE(a.end == Phase::Done, std::string{rig.cal.reason()});
            const auto& d = rig.cal.result();
            REQUIRE(d.left.count == 4);
            CHECK(d.left.signedPorts[0] == 11);
            CHECK(d.left.signedPorts[1] == 12);
            CHECK(d.left.signedPorts[2] == 14);
            CHECK(d.left.signedPorts[3] == 15);
            CHECK(d.right.count == 5);
            CHECK(d.sideOf(13) == 0);
            CHECK(std::string{rig.cal.warning()}
                  == "port 13 gave no reading at DONE -- not counted as a drive port; if it "
                     "drives the robot, check it and run again");
            CHECK(calib_rig::screenSafe(rig.cal.warning()));
            // The LEFT scale is the clean run's, bit for bit (the members read identical values).
            CHECK(sameBits(rep.leftMedianRad, clean.cal.report().leftMedianRad));
            CHECK(sameBits(rep.leftInchesPerRadian, clean.cal.report().leftInchesPerRadian));
            CHECK(sameBits(rep.rightInchesPerRadian, clean.cal.report().rightInchesPerRadian));
            // No other candidate's line changed word.
            for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
                if (i != i13) {
                    CHECK(pushLine(rep.ports[i]) == pushLine(clean.cal.report().ports[i]));
                }
            }
        };
        SUBCASE("+Inf") { held(2); }
        SUBCASE("-Inf") { held(3); }
        SUBCASE("NaN") { held(0); }
        SUBCASE("-NaN") { held(1); }
    }

    SUBCASE("(b) the net: it recovers after FRONT, its train back-drives it, NON_DRIVE_MOVED 13") {
        auto recovers = [&](int kind) {
            CAPTURE(kBadNames[kind]);
            CalibRig rig{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
            const N10Run b = runN10(
                rig, [kind](CalibRig& r) { freezePort13(r, kBadKinds[kind]); },
                [](CalibRig& r) {
                    r.h.plant().setMemberFault(0, 2, shulib::sim::MemberFault::None);
                });
            MESSAGE("N10(b) ", kBadNames[kind], ": end ", static_cast<int>(b.end), " refusal ",
                    std::string{refusalName(rig.cal.refusal())}, " port ", rig.cal.refusalPort(),
                    " left count ",
                    b.end == Phase::Done ? static_cast<int>(rig.cal.result().left.count) : -1,
                    " port 13 as ",
                    b.end == Phase::Done ? static_cast<int>(rig.cal.result().signedPortOf(13)) : 0,
                    " on side ",
                    b.end == Phase::Done ? static_cast<int>(rig.cal.result().sideOf(13)) : 0);
            CHECK(b.end == Phase::Aborted);
            CHECK_MESSAGE(rig.cal.refusal() == Refusal::NonDriveMoved,
                          std::string{refusalName(rig.cal.refusal())}, ": ",
                          std::string{rig.cal.reason()});
            CHECK(rig.cal.refusalPort() == 13);
            CHECK(calib_rig::safeStateHolds(rig));
            // The reason tells the operator what the push record says — it gave NO READING at
            // DONE — never "it did not move on the push", which nobody knows (review round,
            // 2026-09-30); and it fits the Aborted screen's three 50-column lines, as the other
            // NON_DRIVE_MOVED sentence does, so the panel does not cut it.
            CHECK(calib_rig::reasonMentions(rig.cal, "port 13 moved"));
            CHECK(calib_rig::reasonMentions(rig.cal, "it gave no reading at DONE, so it was not "
                                                     "counted as a drive port"));
            CHECK_FALSE(calib_rig::reasonMentions(rig.cal, "did not move on the push"));
            CHECK(calib_rig::screenSafe(rig.cal.reason()));
            CHECK(shulib::calibration::wrapWords<shulib::calibration::kSmallCols, 3>(
                      rig.cal.reason())
                      .complete);
        };
        SUBCASE("+Inf") { recovers(2); }
        SUBCASE("-Inf") { recovers(3); }
        SUBCASE("NaN") { recovers(0); }
    }

    SUBCASE("(d) two with no reading, one per side (ports 13 and 18): two sentences, ' | '") {
        // Review round, 2026-09-30: every case above has ONE no-reading candidate, so the
        // separator before a SECOND no-reading sentence was never run (mutation S2 survived).
        // Two sentences are 223 characters, inside warning()'s 256: the exact text, no cut.
        const std::size_t i18 = CalibRig::indexOf(1, 2);
        CalibRig rig{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
        const N10Run d = runN10(
            rig,
            [](CalibRig& r) {
                freezePort13(r, kInf);
                r.h.plant().setMemberFault(1, 2, shulib::sim::MemberFault::Frozen);
                r.member(1, 2).setPosition(AngleDim{kQNaN});
            },
            nullptr);
        const auto& rep = rig.cal.report();
        REQUIRE(rig.ports[i18] == 18);
        CHECK(d.movedWhileWatched == 8);
        CHECK(rep.drivePortCount == 8);
        for (const std::size_t i : {i13, i18}) {
            CAPTURE(static_cast<int>(rig.ports[i]));
            CHECK(rep.ports[i].sign == 0);
            CHECK(sameBits(rep.ports[i].pushTravelRad, 0.0));
            const std::string line = pushLine(rep.ports[i]);
            CHECK(line.size() >= 19);
            CHECK(line.substr(line.size() - 19) == " NO READING sign +0");
        }
        REQUIRE_MESSAGE(d.end == Phase::Done, std::string{rig.cal.reason()});
        CHECK(std::string{rig.cal.warning()}
              == "port 13 gave no reading at DONE -- not counted as a drive port; if it drives "
                 "the robot, check it and run again | port 18 gave no reading at DONE -- not "
                 "counted as a drive port; if it drives the robot, check it and run again");
        CHECK(calib_rig::screenSafe(rig.cal.warning()));
        const auto& res = rig.cal.result();
        CHECK(res.left.count == 4);
        CHECK(res.right.count == 4);
        CHECK(res.sideOf(13) == 0);
        CHECK(res.sideOf(18) == 0);
    }

    SUBCASE("(e) a STILL candidate read slightly backward: no travel, exactly +0.0, too") {
        // Review round, 2026-09-30: pushTravelRad of a port that is not a drive motor is written
        // +0.0 rather than sign x reading. For a NO READING port that product is a NaN (above);
        // for a still port read a little BACKWARD it is 0 x -d = -0.0. Every still port elsewhere
        // reads exactly +0.0 (an idle FakeMotor at 0), where the two agree, so only this run can
        // tell "+0.0 for every port with no sign" from "+0.0 only when the reading is not finite".
        const std::size_t i5 = 2 * CalibRig::kPerWheel;  // the first idle candidate
        CalibRig rig{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
        REQUIRE(rig.ports[i5] == 5);
        const N10Run e = runN10(
            rig, [](CalibRig& r) { r.idle[0].setPosition(AngleDim{-0.01}); }, nullptr);
        const auto& rep = rig.cal.report();
        CHECK(rep.ports[i5].pushDeltaRad == -0.01);  // under movedThreshold: still
        CHECK(rep.ports[i5].sign == 0);
        CHECK(sameBits(rep.ports[i5].pushTravelRad, 0.0));
        const std::string line = pushLine(rep.ports[i5]);
        CHECK(line.substr(line.size() - 14) == " still sign +0");
        CHECK(e.movedWhileWatched == 10);
        REQUIRE_MESSAGE(e.end == Phase::Done, std::string{rig.cal.reason()});
        CHECK(std::strlen(rig.cal.warning()) == 0);
    }

    SUBCASE("(c) sideTravel(): a LEFT member reads NaN on the tick the CCW spin finishes") {
        // How the hook finds that tick: it does not need to. It poisons port 13's reading on
        // EVERY tick of the CCW spin (the hook runs before the calibrator's tick; the plant's next
        // step writes the member again, so the NaN is seen by exactly one calibrator tick each
        // time). During a spin nothing reads a drive shaft's POSITION except sideTravel(), at the
        // tick the direction finishes — the fight check reads velocities, the rest windows and
        // the status read no position — so the finishing tick is the only one that sees it. The
        // run matching the clean one bit for bit everywhere else is the proof of that sentence.
        CalibRig rig{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
        int poisoned = 0;
        const N10Run s = runN10(rig, nullptr, nullptr,
                                [&poisoned](CalibRig& r, const calib_rig::Script&) {
                                    if (r.cal.status().spinDirection > 0.0) {
                                        r.member(0, 2).setPosition(AngleDim{kQNaN});
                                        ++poisoned;
                                    }
                                });
        const auto& rep = rig.cal.report();
        const auto& crep = clean.cal.report();
        MESSAGE("N10(c): end ", static_cast<int>(s.end), " refusal ",
                std::string{refusalName(rig.cal.refusal())}, " poisoned ticks ", poisoned,
                " spins ", static_cast<int>(rep.spinCount), " L ", rep.spins[0].leftTravelIn,
                " clean L ", crep.spins[0].leftTravelIn);
        CHECK(poisoned > 100);
        REQUIRE(rep.spinCount >= 1);
        CHECK(sameBits(rep.spins[0].leftTravelIn, crep.spins[0].leftTravelIn));
        CHECK(sameBits(rep.spins[0].rightTravelIn, crep.spins[0].rightTravelIn));
        CHECK(sameBits(rep.spins[0].widthIn, crep.spins[0].widthIn));
        REQUIRE_MESSAGE(s.end == Phase::Done, std::string{rig.cal.reason()});
        CHECK(sameBits(rig.cal.result().trackWidth.value(), clean.cal.result().trackWidth.value()));
        CHECK(sameBits(rep.spins[1].leftTravelIn, crep.spins[1].leftTravelIn));
    }
}

// ═══ N11. MotorGroup's current and temperature with every reading finite ═════════════

namespace {

/// THE PIN (N11): verbatim copies of `9239538`'s MotorGroup::sumCurrent(), current(),
/// totalCurrent() and temperature() bodies (motor_group.hpp :221-238 and :340-348 there), as free
/// functions over the same members span: `present` is the class's count of non-null slots, and
/// the temperature's seed is its first() — the first non-null slot. Fed FINITE readings only.
[[nodiscard]] double legacySumCurrent(std::span<IMotor* const> members) {
    double sum = 0.0;
    for (const IMotor* m : members) {
        if (m != nullptr) {
            sum += m->current().value();
        }
    }
    return sum;
}
[[nodiscard]] double legacyCurrent(std::span<IMotor* const> members, std::size_t present) {
    return legacySumCurrent(members) / static_cast<double>(present);
}
[[nodiscard]] double legacyTotalCurrent(std::span<IMotor* const> members) {
    return legacySumCurrent(members);
}
[[nodiscard]] double legacyTemperature(std::span<IMotor* const> members) {
    const IMotor* first = nullptr;
    for (const IMotor* m : members) {
        if (m != nullptr) {
            first = m;
            break;
        }
    }
    double worst = first->temperature();
    for (const IMotor* m : members) {
        if (m != nullptr) {
            worst = std::max(worst, m->temperature());
        }
    }
    return worst;
}

}  // namespace

// Bug caught: the rewritten reads — a count of the finite readings beside the sum, a max no longer
// seeded from first() — answering an ALL-FINITE group (every group a V5 reads through ProsMotor,
// HA-08) differently from before by a single bit: the sum taken in another order (floating-point
// addition is not associative, so the last bit moves), the sum seeded from the first reading
// instead of 0.0 (a side of -0.0 A would read -0.0), the max taken in another order (which of
// +0.0 and -0.0 it keeps depends on that). N5 compares the new reads only with themselves (a bad
// member against a hole); only verbatim copies of the old ones can pin "exactly as before".
// Mutations (review round, 2026-09-30): MA1 (sumCurrent() walks the members in reverse), MT1
// (temperature() walks them in reverse), MS1 (the sum seeded from the first finite reading).
TEST_CASE("DIAG1 N11: with every reading finite, current(), totalCurrent() and temperature() are "
          "the old reads, bit for bit") {
    Lcg rng{0xD1A61B11};
    auto amps = [&rng]() -> double {
        const double r = rng.unit();
        if (r < 0.10) {  // a side at rest: zeros of both signs
            return rng.unit() < 0.5 ? 0.0 : -0.0;
        }
        if (r < 0.45) {  // what PROS hands over: whole milliamps, inexact in binary
            return static_cast<double>(rng.below(2501)) / 1000.0;
        }
        if (r < 0.55) {  // far apart in size, either sign: the order of the sum shows
            return rng.between(-1.0e3, 1.0e3);
        }
        return rng.between(0.0, 2.5);
    };
    auto celsius = [&rng](bool cold) -> double {
        const double r = rng.unit();
        if (cold) {  // the hottest member is often a ZERO, of either sign
            return r < 0.5 ? (rng.unit() < 0.5 ? 0.0 : -0.0)
                           : -static_cast<double>(1 + rng.below(10));
        }
        if (r < 0.30) {  // whole degrees: duplicates are common
            return static_cast<double>(rng.below(81));
        }
        return rng.between(15.0, 90.0);
    };
    const int kGroups = 200000;
    int currentDiffers = 0;
    int totalDiffers = 0;
    int temperatureDiffers = 0;
    int counted = 0;
    int reverseSumDiffers = 0;  // the draw's power: groups a reversed sum would answer differently
    int reverseMaxDiffers = 0;  // ... and a reversed max
    for (int t = 0; t < kGroups; ++t) {
        const std::size_t n = 1 + rng.below(21);
        const bool cold = rng.unit() < 0.3;
        Bank b;
        std::uint32_t holes = 0;
        for (std::size_t i = 0; i < n; ++i) {
            b.m[i].setCurrent(Current{amps()});
            b.m[i].setTemperature(celsius(cold));
            if (rng.unit() < 0.15) {
                holes |= std::uint32_t{1} << i;
            }
        }
        holes &= ~(std::uint32_t{1} << rng.below(n));  // at least one member present
        auto p = b.ptrs(n, holes);
        const std::span<IMotor* const> span{p.data(), n};
        MotorGroup g{span};
        const double oldCurrent = legacyCurrent(span, g.presentCount());
        currentDiffers += sameBits(g.current().value(), oldCurrent) ? 0 : 1;
        totalDiffers += sameBits(g.totalCurrent().value(), legacyTotalCurrent(span)) ? 0 : 1;
        temperatureDiffers += sameBits(g.temperature(), legacyTemperature(span)) ? 0 : 1;
        counted += g.nonFiniteReads();

        double reverseSum = 0.0;
        double reverseMax = 0.0;
        bool seeded = false;
        for (std::size_t i = n; i-- > 0;) {
            if (span[i] != nullptr) {
                reverseSum += span[i]->current().value();
                const double c = span[i]->temperature();
                reverseMax = seeded ? std::max(reverseMax, c) : c;
                seeded = true;
            }
        }
        reverseSumDiffers += sameBits(reverseSum, legacySumCurrent(span)) ? 0 : 1;
        reverseMaxDiffers += sameBits(reverseMax, legacyTemperature(span)) ? 0 : 1;
    }
    MESSAGE("N11: ", kGroups, " all-finite groups; differ from the old reads: current ",
            currentDiffers, ", totalCurrent ", totalDiffers, ", temperature ", temperatureDiffers,
            "; the draw's power: a reversed sum would differ on ", reverseSumDiffers,
            " groups, a reversed max on ", reverseMaxDiffers);
    CHECK(currentDiffers == 0);
    CHECK(totalDiffers == 0);
    CHECK(temperatureDiffers == 0);
    CHECK(counted == 0);  // every reading finite: nothing left out, nothing counted
    // A draw that could no longer tell an order apart would pass by comparing nothing.
    REQUIRE(reverseSumDiffers > 1000);
    REQUIRE(reverseMaxDiffers > 1000);

    // Fixed cases, independent of the draw. A side of -0.0 A reads +0.0 A — the sum starts at
    // 0.0, as it always did — and of two zero temperatures the FIRST member's is kept.
    for (const std::size_t n : {std::size_t{1}, std::size_t{2}, std::size_t{5}}) {
        CAPTURE(n);
        Bank z;
        for (std::size_t i = 0; i < n; ++i) {
            z.m[i].setCurrent(Current{-0.0});
        }
        auto pz = z.ptrs(n);
        MotorGroup gz{std::span<IMotor* const>{pz.data(), n}};
        CHECK(sameBits(gz.current().value(), 0.0));
        CHECK(sameBits(gz.totalCurrent().value(), 0.0));
    }
    for (const double firstZero : {0.0, -0.0}) {
        CAPTURE(firstZero);
        Bank z;
        z.m[0].setTemperature(-5.0);
        z.m[1].setTemperature(firstZero);
        z.m[2].setTemperature(-firstZero);
        auto pz = z.ptrs(3);
        MotorGroup gz{std::span<IMotor* const>{pz.data(), 3}};
        CHECK(sameBits(gz.temperature(), firstZero));
    }
}

// ═══ N12. The push's "moved" rule over robot two's real wiring ════════════════════════

namespace {

/// How one N12 run went: how many watched ticks pushReady() was true on (the scripted operator
/// touches DONE on the first), status().movedNow on that tick, and how the run ended.
struct N12Run {
    shulib::calibration::Phase end = shulib::calibration::Phase::Idle;
    int readyTicks = 0;
    int movedWhenReady = -1;
};

[[nodiscard]] N12Run runN12(CalibRig& rig, calib_rig::RunOptions opt) {
    N12Run out;
    rig.cal.start(calib_rig::facts());
    opt.hook = [&out](CalibRig& r, const calib_rig::Script& s) {
        if (s.stage >= 1 && r.cal.phase() == shulib::calibration::Phase::PushCapture
            && r.cal.pushReady()) {
            if (out.readyTicks == 0) {
                out.movedWhenReady = r.cal.status().movedNow;
            }
            ++out.readyTicks;
        }
    };
    out.end = calib_rig::runCalibration(rig, opt);
    return out;
}

/// The median |push travel| of the drive ports, from the report, through the test's own oracle —
/// what pushSummary() must have shown as status().pushTravelRad while the push was watched.
[[nodiscard]] double medianDriveTravel(const shulib::calibration::CalibrationReport& rep) {
    std::vector<double> v;
    for (std::size_t i = 0; i < rep.portCount; ++i) {
        if (rep.ports[i].sign != 0) {
            v.push_back(std::abs(rep.ports[i].pushDeltaRad));
        }
    }
    return v.empty() ? 0.0 : oracleMedian(v.data(), v.size());
}

/// Robot two's drive ports as its chassis table stores them (src/robots/chassis_table.hpp: the
/// signs MOTOR WATCH measured on 2026-09-10, two front-first pushes).
constexpr int kRobotTwoLeft[5] = {-11, 12, -13, 14, -15};
constexpr int kRobotTwoRight[5] = {20, -19, 18, -17, 16};

}  // namespace

// Bug caught: the DONE gate's median taken over SIGNED push deltas. D5 split pushSummary()'s
// std::abs from its "moved" test (movedOnPush() reads the signed delta; the magnitude is stored
// separately), and dropping that std::abs passed the whole suite, because every push in it reads
// mostly POSITIVE: the calibration rig reverses two members at most (case 4) and no case pushes
// the robot backward. On robot two's real wiring five of the ten drive ports read negative on a
// forward push, so a signed median sits near 0, pushReady() never comes true, and a forced DONE is
// refused PUSH_TOO_SHORT: calibration could never finish on the robot it was written for, with the
// suite green. A push LED WITH THE OTHER END (pushed backward: its leading end is that run's front —
// until CAL1 Part B it was answered BACK-first instead) reads all ten negative. And the rule's
// boundary: a travel exactly AT movedThreshold "reaches" it.
// Mutations (review round, 2026-09-30): S1 (pushSummary() stores the signed delta), S3
// (movedOnPush() compares with '>').
TEST_CASE("DIAG1 N12: the push over robot two's real wiring, a push led with the other end, "
          "and a travel exactly at movedThreshold") {
    using shulib::calibration::Answer;
    using shulib::calibration::CalibratorConfig;
    using shulib::calibration::Phase;
    using shulib::calibration::Refusal;

    // The plain run every comparison below is made against: all members forward, pushed forward.
    CalibRig plain{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
    const N12Run p = runN12(plain, calib_rig::RunOptions{});
    REQUIRE(p.end == Phase::Done);

    SUBCASE("(a) robot two's wiring: five of ten ports read backward on a forward push") {
        CalibRig rig{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
        rig.setFault(0, 0, shulib::sim::MemberFault::Reversed);  // -11
        rig.setFault(0, 2, shulib::sim::MemberFault::Reversed);  // -13
        rig.setFault(0, 4, shulib::sim::MemberFault::Reversed);  // -15
        rig.setFault(1, 1, shulib::sim::MemberFault::Reversed);  // -17
        rig.setFault(1, 3, shulib::sim::MemberFault::Reversed);  // -19
        const N12Run a = runN12(rig, calib_rig::RunOptions{});
        const auto& rep = rig.cal.report();
        int negative = 0;
        for (std::size_t i = 0; i < 2 * CalibRig::kPerWheel; ++i) {
            negative += rep.ports[i].pushDeltaRad < 0.0 ? 1 : 0;
        }
        MESSAGE("N12(a): end ", static_cast<int>(a.end), " readyTicks ", a.readyTicks,
                " movedNow ", a.movedWhenReady, " status pushTravelRad ",
                rig.cal.status().pushTravelRad, " negative deltas ", negative, " refusal ",
                std::string{shulib::calibration::refusalName(rig.cal.refusal())});
        CHECK(negative == 5);  // the case's premise: half the deltas negative
        CHECK(a.readyTicks > 0);  // DONE is OFFERED (the panel waits for pushReady())
        CHECK(a.movedWhenReady == 10);
        // The DONE gate's median is of MAGNITUDES: what the oracle gives from the report, and
        // what the plain run showed.
        CHECK(sameBits(rig.cal.status().pushTravelRad, medianDriveTravel(rep)));
        CHECK(sameBits(rig.cal.status().pushTravelRad, plain.cal.status().pushTravelRad));
        REQUIRE_MESSAGE(a.end == Phase::Done, std::string{rig.cal.reason()});
        // The run finds robot two as its table has it: every sign, every side.
        const auto& d = rig.cal.result();
        for (std::size_t k = 0; k < 5; ++k) {
            CAPTURE(k);
            CHECK(d.signedPortOf(std::abs(kRobotTwoLeft[k])) == kRobotTwoLeft[k]);
            CHECK(d.sideOf(std::abs(kRobotTwoLeft[k])) == -1);
            CHECK(d.signedPortOf(std::abs(kRobotTwoRight[k])) == kRobotTwoRight[k]);
            CHECK(d.sideOf(std::abs(kRobotTwoRight[k])) == +1);
        }
        CHECK(rig.cal.refusal() == Refusal::None);
        CHECK(std::strlen(rig.cal.warning()) == 0);
        // Its scale is the plain run's: the same shaft travel, sign-corrected.
        CHECK(sameBits(rep.leftInchesPerRadian, plain.cal.report().leftInchesPerRadian));
        CHECK(sameBits(rep.rightInchesPerRadian, plain.cal.report().rightInchesPerRadian));
    }

    SUBCASE("(b) pushed BACKWARD -- led with the other end: every delta negative") {
        CalibRig rig{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
        calib_rig::RunOptions opt;
        opt.pushDirection = -1.0;
        const N12Run b = runN12(rig, opt);
        const auto& rep = rig.cal.report();
        MESSAGE("N12(b): end ", static_cast<int>(b.end), " readyTicks ", b.readyTicks,
                " movedNow ", b.movedWhenReady, " status pushTravelRad ",
                rig.cal.status().pushTravelRad, " refusal ",
                std::string{shulib::calibration::refusalName(rig.cal.refusal())});
        CHECK(b.readyTicks > 0);
        CHECK(b.movedWhenReady == 10);
        CHECK(sameBits(rig.cal.status().pushTravelRad, medianDriveTravel(rep)));
        CHECK(rig.cal.status().pushTravelRad > 0.0);
        REQUIRE_MESSAGE(b.end == Phase::Done, std::string{rig.cal.reason()});
        for (std::size_t i = 0; i < 2 * CalibRig::kPerWheel; ++i) {
            CAPTURE(static_cast<int>(rig.ports[i]));
            CHECK(rep.ports[i].pushDeltaRad < 0.0);   // the shaft turned backward ...
            CHECK(rep.ports[i].sign == -1);           // ... which is this run's sign, as captured
            CHECK(rep.ports[i].pushTravelRad > 0.0);  // travel toward THIS run's front: positive
        }
        // The other end led the push, so it is this run's FRONT: the description is the plain
        // run's robot FACING THE OTHER WAY — every port negated and on the other side.
        const auto& d = rig.cal.result();
        const auto& pd = plain.cal.result();
        for (int port = 11; port <= 20; ++port) {
            CAPTURE(port);
            CHECK(d.signedPortOf(port) == -port);
            CHECK(d.signedPortOf(port) == -pd.signedPortOf(port));
            CHECK(d.sideOf(port) == -pd.sideOf(port));
        }
        // And the scales swapped with the sides: the same shaft travel, the other way round.
        CHECK(sameBits(rep.leftInchesPerRadian, plain.cal.report().rightInchesPerRadian));
        CHECK(sameBits(rep.rightInchesPerRadian, plain.cal.report().leftInchesPerRadian));
    }

    SUBCASE("(c) a travel exactly AT movedThreshold has moved: the rule is >=, as documented") {
        // The host test plant is deterministic, so a push's shaft travel can be read off one run
        // and handed to the next as its threshold, bit for bit (the calibrator commands nothing
        // during the push, so the threshold cannot change the push). With the threshold AT the
        // smallest travel, every drive port still counts as moved; under '>' the ports AT it
        // would not, and the run could not describe the robot. (Just under and just over the
        // line: DrivetrainCalibrator 4b, at 0.98x and 1.02x.)
        calib_rig::RunOptions opt;
        opt.waitForReady = false;  // judged by the FINISH, as 4b is, so a '>' rule cannot hang
        CalibRig first{CalibratorConfig{}, calib_rig::calibPlant(0.0)};
        first.cal.start(calib_rig::facts());
        REQUIRE(calib_rig::runCalibration(first, opt) == Phase::Done);
        double at = kInf;
        for (std::size_t i = 0; i < 2 * CalibRig::kPerWheel; ++i) {
            at = std::min(at, std::abs(first.cal.report().ports[i].pushDeltaRad));
        }
        REQUIRE(std::isfinite(at));
        CalibratorConfig cfg;
        cfg.movedThreshold = AngleDim{at};
        CalibRig rig{cfg, calib_rig::calibPlant(0.0)};
        rig.cal.start(calib_rig::facts());
        const Phase end = calib_rig::runCalibration(rig, opt);
        const auto& rep = rig.cal.report();
        int atThreshold = 0;
        for (std::size_t i = 0; i < 2 * CalibRig::kPerWheel; ++i) {
            CAPTURE(static_cast<int>(rig.ports[i]));
            REQUIRE(sameBits(rep.ports[i].pushDeltaRad, first.cal.report().ports[i].pushDeltaRad));
            atThreshold += std::abs(rep.ports[i].pushDeltaRad) == at ? 1 : 0;
            CHECK(rep.ports[i].sign == 1);
        }
        MESSAGE("N12(c): threshold ", at, " rad; ", atThreshold,
                " of 10 drive ports travelled exactly that; drivePortCount ",
                static_cast<int>(rep.drivePortCount), " end ", static_cast<int>(end));
        CHECK(atThreshold >= 1);
        CHECK(rep.drivePortCount == 10);
        REQUIRE_MESSAGE(end == Phase::Done, std::string{rig.cal.reason()});
    }
}
