// IMU1 Part A (2026-10-29) — the IMU's turn rate is read ONCE per control tick, and every reader in
// that tick gets the same value. The brief: docs/internal/chunks/IMU1-imu-rate-once-per-tick.md
// (§2.2's R0 and T1–T11; its re-check amendments R1–R18 override its body). T12 (the clock before
// the device — landmine 4) and L1–L3 (the hold's three limits, as the documents quote them) were
// added at Part A's review, 2026-10-29 — written after the fix; their runs on the unchanged adapter
// are in the progress log.
//
// THE DEFECT these tests reproduce. ProsImu::yawRate() differentiated get_rotation() BETWEEN CALLS,
// and the library reads the rate two or three times a tick (the Localizer; on robot two, every
// motion's record, because its serial sink wants every record; a GPS or tag corrector's gate; the
// calibration classes). On a robot the clock moves microseconds between those reads (ProsClock is
// micros(), HA-101) while the IMU's buffer holds one value for 10 ms (HA-213), so a second read
// answered about 0 or a spike AND moved the differentiation baseline — corrupting the NEXT tick's
// first read, the Localizer's own, which TANK1's yaw observer and DriveBrake use. Every host clock
// stands still inside a tick, so no host test saw it: these tests run the REAL adapter over
// test/imu_buffer_model.hpp (the 10 ms copy and a creeping clock — beliefs HA-213..HA-215, not
// measurements).
//
// REPRODUCTION FIRST (§2.5, landmine 1): every §2.2 test here (R0, T1–T11) was written and run
// against the UNCHANGED adapter before the fix, and the ones the brief calls red TODAY were seen
// red, with their counts in docs/internal/chunks/IMU1-PROGRESS.md. This file names NO symbol the
// fix adds (T3's edge is in literal microseconds — re-check R2), so it builds against either
// adapter; the pin that does name the new constant is test/imu_rate_hold_pin_test.cpp.
//
// Words: "host test plant", never "simulator". NOTHING HERE HAS RUN ON A ROBOT; the copy's period
// (HA-213), its phase (HA-214) and the reads' spacing (HA-215) are beliefs, swept where unknown.

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "calibration_test_rig.hpp"
#include "imu_buffer_model.hpp"
#include "motion_test_rig.hpp"
#include "pros/shim_control.hpp"
#include "shulib/calibration/rest_capture.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/diag/odom_tick_audit.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/imu_conversion.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/hal/pros/clock.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/gps_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/odometry.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/scenario.hpp"

namespace chassis = shulib::chassis;
namespace motion = shulib::motion;
namespace sim = shulib::sim;
namespace units = shulib::units;
namespace loc = shulib::localization;
namespace cal = shulib::calibration;
using shulib::control::ExitReason;
using shulib::hal::IClock;
using shulib::hal::fake::FakeClock;
using shulib::hal::pros::ProsClock;
using shulib::hal::pros::ProsImu;
using shulib::hal::pros::YawRateSource;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;
using imu_model::BufferModel;
using imu_model::CreepClock;
using imu_model::ScopedInstall;
using imu_model::SpyImu;

namespace {

constexpr int kPort = 9;
constexpr double kTick = 0.010;
constexpr double kDegToRad = Angle::kPi / 180.0;
/// The constant turn of the adapter tests: 90 deg/s CW = canonical −π/2 rad/s.
constexpr double kTurnDegCw = 90.0;
constexpr double kTurnCanonical = -kTurnDegCw * kDegToRad;

/// A calibrated, ready ProsImu on `port` over `clock` (the shim starts non-calibrating).
ProsImu readyImu(const IClock& clock, YawRateSource src = YawRateSource::DifferentiateRotation,
                 int port = kPort) {
    ProsImu imu{static_cast<std::uint8_t>(port), Angle{}, clock, src};
    imu.calibrate();
    pros::shim::imuState(port).calibrating = false;
    return imu;
}

/// The shim's microsecond clock moved to an absolute instant (never backward).
void moveToUs(std::uint64_t us) {
    REQUIRE(us >= pros::micros());
    pros::shim::advanceUs(us - pros::micros());
}

/// One adapter run over the model: `ticks` ticks of `readsPerTick` rate reads, a constant turn.
struct TickRun {
    std::vector<double> first;  ///< every tick's FIRST read
    int laterDiffer = 0;        ///< later reads in a tick that differ (bitwise) from its first
    int laterReads = 0;
};
TickRun runTicks(double gap, int readsPerTick, double phase, int ticks) {
    pros::shim::resetAll();
    BufferModel m;
    m.phase = phase;
    m.gap = gap;
    m.truthCwDeg = [](double t) { return kTurnDegCw * t; };
    const CreepClock clk{m};
    const ProsImu imu = readyImu(clk);
    const ScopedInstall installed{m, kPort};
    TickRun r;
    for (int k = 0; k < ticks; ++k) {
        m.beginTick(static_cast<double>(k) * kTick);
        const double first = imu.yawRate().value();
        r.first.push_back(first);
        for (int j = 1; j < readsPerTick; ++j) {
            ++r.laterReads;
            if (imu.yawRate().value() != first) {
                ++r.laterDiffer;
            }
        }
    }
    return r;
}

/// T1's phases for `n` reads a tick (read j lands at j × gap): one before the first read, three
/// between the first and the last, three after — every one strictly between two read instants
/// (on one, HA-214's residual decides the FIRST read; that is not what T1 tests — re-check R12).
std::vector<double> phasesFor(double gap, int n) {
    std::vector<double> v{0.5 * gap};
    for (const double f : {0.25, 0.5, 0.75}) {
        double k = 1.0 + f * static_cast<double>(n - 1);
        if (k == std::floor(k)) {
            k += 0.1;  // never ON a read instant
        }
        v.push_back(k * gap);
    }
    v.push_back((static_cast<double>(n) + 0.5) * gap);
    v.push_back(0.005);
    v.push_back(0.0097);
    return v;
}

bool relClose(double a, double truth, double rel) {
    return std::abs(a - truth) <= rel * std::abs(truth);
}

}  // namespace

// ═══ R0 — the harness's own check: the model is sample-and-hold ════════════════════════════

// Bug caught (mutation M9 — the hook returns the truth AT the read instant, a continuous sensor):
// a model so kind that today's adapter looks nearly right and the reproduction proves nothing
// (landmine 2). Two reads 1 µs apart inside one copy period are IDENTICAL; across a copy they differ
// by exactly one copy's turn. Read through the shim's own get_rotation(), so the hook path counts.
TEST_CASE("IMU1 R0: the host test plant's IMU model holds one value for 10 ms (HA-213) — the "
          "harness's own check") {
    pros::shim::resetAll();
    BufferModel m;
    m.phase = 0.0037;
    m.gap = 1e-6;
    m.truthCwDeg = [](double t) { return kTurnDegCw * t; };
    const ScopedInstall installed{m, kPort};
    const pros::Imu raw{static_cast<std::uint8_t>(kPort)};
    auto& st = pros::shim::imuState(kPort);

    m.beginTick(0.0500);  // copies at 0.0437 and 0.0537
    const double a = raw.get_rotation();
    const double b = raw.get_rotation();
    CHECK(a == b);                                   // inside one period: the SAME value
    CHECK(a == doctest::Approx(kTurnDegCw * 0.0437));  // ... and it is the copy's, not the read's
    m.beginTick(0.0537 - 1.5e-6);                    // reads at 0.5 µs before and after a copy
    const double c = raw.get_rotation();
    const double d = raw.get_rotation();
    CHECK(d - c == doctest::Approx(kTurnDegCw * kTick).epsilon(1e-9));  // exactly one copy's turn
    CHECK(st.rotationReads == 4);
    CHECK(m.deviceReads == 4);
    CHECK(m.creep == doctest::Approx(2e-6));  // the IMU's clock crept 1 µs per device read
}

// ═══ T1 — one answer per tick ═══════════════════════════════════════════════════════════════

// Bug caught: a second read in a tick answering about 0 or a ×(10 ms / gap) spike (probe 1) — the
// record's rate, the gate's, a team's own imu.yawRate() in an auton. Every later read in a tick
// must equal the first BIT FOR BIT, at every copy phase, read spacing and read count.
TEST_CASE("IMU1 T1: every rate read in a tick answers the tick's first read, bit for bit") {
    for (const double gap : {25e-6, 100e-6, 400e-6}) {
        for (const int n : {2, 3, 5}) {
            for (const double phi : phasesFor(gap, n)) {
                CAPTURE(gap);
                CAPTURE(n);
                CAPTURE(phi);
                const TickRun r = runTicks(gap, n, phi, 200);
                CHECK(r.laterReads == 200 * (n - 1));
                CHECK(r.laterDiffer == 0);
            }
        }
    }
}

// ═══ T2 — the first read is the truth, however many reads follow ═══════════════════════════

// Bug caught: a later read moving the differentiation baseline, so the NEXT tick's first read —
// the Localizer's — differentiates over 10 ms minus the tick's span (3 % high at 0.3 ms) or, for a
// copy between two reads, over an interval with no copy in it (0 on every tick). The first read
// must be the truth on every tick and must not depend on how many reads follow it.
TEST_CASE("IMU1 T2: a tick's first rate read is the truth, and extra reads never move it") {
    for (const double gap : {25e-6, 100e-6, 400e-6}) {
        for (const double phi : phasesFor(gap, 5)) {  // off every read instant for 1, 2 and 5 reads
            CAPTURE(gap);
            CAPTURE(phi);
            const TickRun r1 = runTicks(gap, 1, phi, 200);
            const TickRun r2 = runTicks(gap, 2, phi, 200);
            const TickRun r5 = runTicks(gap, 5, phi, 200);
            int off1 = 0;
            int off2 = 0;
            int off5 = 0;
            for (std::size_t k = 1; k < r1.first.size(); ++k) {  // tick 0: the first call answers 0
                off1 += relClose(r1.first[k], kTurnCanonical, 1e-9) ? 0 : 1;
                off2 += relClose(r2.first[k], kTurnCanonical, 1e-9) ? 0 : 1;
                off5 += relClose(r5.first[k], kTurnCanonical, 1e-9) ? 0 : 1;
            }
            CHECK(off1 == 0);
            CHECK(off2 == 0);
            CHECK(off5 == 0);
            CHECK(r2.first == r1.first);  // bit-identical: the extra reads changed nothing
            CHECK(r5.first == r1.first);
        }
    }
}

// ═══ T3 — the hold's edge, in literal microseconds; a held read never extends it ═══════════

// Bug caught (M2: `<` → `<=`; M4: the hold 10 ms): the hold's edge off by a microsecond, or the
// hold longer than half a tick. Answered at t, a read at t + 4 999 µs is HELD (no device read, the
// same value) and a read at t + 5 000 µs RE-SAMPLES (one device read; the rate over 5 ms) — at
// three anchors, one of them not round, because the hold compares WHOLE microseconds (the robot
// clock's own resolution — re-check R3/R13; in double seconds the exact edge would be decided by
// rounding). No new symbol is named here (re-check R2): the pin is its own TU.
TEST_CASE("IMU1 T3 (a): the hold ends exactly 5000 us after the answer, at any anchor") {
    for (const std::uint64_t anchor : {std::uint64_t{0}, std::uint64_t{10'000},
                                       std::uint64_t{1'234'567}}) {
        CAPTURE(anchor);
        pros::shim::resetAll();
        auto& st = pros::shim::imuState(kPort);
        const ProsClock clk;
        const ProsImu imu = readyImu(clk);
        double answered = 0.0;
        if (anchor >= 10'000) {
            moveToUs(anchor - 10'000);
            st.rotationDegCw = 11.0;
            (void)imu.yawRate();  // primes the differentiator a tick before the anchor
            moveToUs(anchor);
            st.rotationDegCw = 12.0;
            answered = imu.yawRate().value();  // 1 deg CW over 10 ms = 100 deg/s CW
            CHECK(answered == doctest::Approx(-100.0 * kDegToRad));
        } else {
            st.rotationDegCw = 12.0;
            answered = imu.yawRate().value();  // the first call: 0 (one sample is not a rate)
            CHECK(answered == 0.0);
        }
        st.rotationDegCw = 13.0;  // the robot turned; a held read must not see it
        moveToUs(anchor + 4'999);
        const int before = st.rotationReads;
        CHECK(imu.yawRate().value() == answered);  // HELD
        CHECK(st.rotationReads == before);         // ... and no device read
        moveToUs(anchor + 5'000);
        const double fresh = imu.yawRate().value();
        CHECK(st.rotationReads == before + 1);  // RE-SAMPLED: one device read
        CHECK(fresh == doctest::Approx(-200.0 * kDegToRad));  // 1 deg CW over 5 ms
    }
}

// Bug caught (M11 — every read, held ones included, restarts the hold): a reader that asks faster
// than every 5 ms would FREEZE the rate for ever (the checker's M11 run: 50 held in a row), and no
// other test asks that fast. Reads every 4 ms for 200 ms: a fresh answer (one device read) on every
// second read at the latest, and every held read equals the latest fresh answer. Only the
// device-read criterion is asserted (re-check R13 (b): a constant turn makes every fresh answer
// the same number, so "no value repeated" would fail a correct hold).
TEST_CASE("IMU1 T3 (b): a held read never extends the hold — a fast reader still gets fresh answers") {
    pros::shim::resetAll();
    auto& st = pros::shim::imuState(kPort);
    const ProsClock clk;
    const ProsImu imu = readyImu(clk);
    int heldRun = 0;
    int longestHeldRun = 0;
    int heldNotLatest = 0;
    int fresh = 0;
    double latestFresh = 0.0;
    for (int i = 0; i <= 50; ++i) {
        moveToUs(static_cast<std::uint64_t>(i) * 4'000);
        st.rotationDegCw = kTurnDegCw * static_cast<double>(pros::micros()) * 1e-6;
        const int before = st.rotationReads;
        const double v = imu.yawRate().value();
        if (st.rotationReads == before + 1) {
            ++fresh;
            latestFresh = v;
            heldRun = 0;
        } else {
            ++heldRun;
            longestHeldRun = std::max(longestHeldRun, heldRun);
            heldNotLatest += v == latestFresh ? 0 : 1;
        }
    }
    CHECK(longestHeldRun <= 1);
    CHECK(heldNotLatest == 0);
    CHECK(fresh >= 26);  // 51 reads, at least every second one fresh
}

// ═══ T4 — a held read touches nothing ═══════════════════════════════════════════════════════

// Bug caught (M5 — a held read still calls get_rotation() and discards it; M8 — the gyro path
// bypasses the hold): four rate reads a tick are ONE device read, on either source, and a dead
// sensor counts ONE screened read a tick, not four.
TEST_CASE("IMU1 T4: four rate reads a tick are one device read, on either source, faulted or not") {
    for (const YawRateSource src : {YawRateSource::DifferentiateRotation, YawRateSource::GyroRateZ}) {
        CAPTURE(static_cast<int>(src));
        pros::shim::resetAll();
        auto& st = pros::shim::imuState(kPort);
        const ProsClock clk;
        const ProsImu imu = readyImu(clk, src);
        for (int k = 0; k < 100; ++k) {
            moveToUs(static_cast<std::uint64_t>(k) * 10'000);
            st.disconnected = k >= 40 && k < 50;  // ten ticks with the sensor gone
            for (int j = 0; j < 4; ++j) {
                pros::shim::advanceUs(25);  // a read takes time on a robot
                st.rotationDegCw = kTurnDegCw * static_cast<double>(pros::micros()) * 1e-6;
                st.gyroZDegPerSecCw = kTurnDegCw;
                (void)imu.yawRate();
            }
        }
        if (src == YawRateSource::DifferentiateRotation) {
            CHECK(st.rotationReads == 100);
            CHECK(st.gyroReads == 0);
        } else {
            CHECK(st.gyroReads == 100);
            CHECK(st.rotationReads == 0);
        }
        CHECK(imu.faultedReads() == 10);  // one per dead tick: a held read is not counted
    }
}

// ═══ T5 — a fault inside a tick ═════════════════════════════════════════════════════════════

// Bug caught (M6 — a faulted read does not open the hold; M7 — a faulted read moves the
// differentiation sample): (a) a tick whose first read is PROS_ERR_F answers the last answer and
// counts ONE fault, and a second read that tick answers the same and touches nothing; (b) the next
// good tick differentiates from the last GOOD sample, 20 ms back — the truth, not double it.
TEST_CASE("IMU1 T5: a faulted first read holds the last answer, counts once, and leaves the sample") {
    pros::shim::resetAll();
    BufferModel m;
    m.phase = 0.005;
    m.gap = 25e-6;
    m.truthCwDeg = [](double t) { return kTurnDegCw * t; };
    const CreepClock clk{m};
    const ProsImu imu = readyImu(clk);
    const ScopedInstall installed{m, kPort};
    auto& st = pros::shim::imuState(kPort);
    double lastAnswer = 0.0;
    for (int k = 0; k < 3; ++k) {
        m.beginTick(static_cast<double>(k) * kTick);
        (void)imu.yawRate();
        lastAnswer = imu.yawRate().value();  // the tick's LAST answer
    }
    // (a) tick 3: the sensor drops.
    st.disconnected = true;
    m.beginTick(3.0 * kTick);
    const int readsBefore = st.rotationReads;
    CHECK(imu.yawRate().value() == lastAnswer);
    CHECK(imu.faultedReads() == 1);
    CHECK(imu.yawRate().value() == lastAnswer);
    CHECK(imu.faultedReads() == 1);                 // the second read counted nothing
    CHECK(st.rotationReads == readsBefore + 1);     // ... and reached no device
    // (b) tick 4: back — 20 ms after the last good sample, two copies of turn.
    st.disconnected = false;
    m.beginTick(4.0 * kTick);
    const double back = imu.yawRate().value();
    CHECK(relClose(back, kTurnCanonical, 1e-9));
    CHECK(imu.yawRate().value() == back);
}

// ═══ T6 — the gyro answers once per tick too ════════════════════════════════════════════════

// Bug caught (M8 — the gyro path bypasses the hold): the hold covers BOTH sources, because a record
// that differs from the Localizer's value is a lie in the evidence column whichever source made it.
TEST_CASE("IMU1 T6: on GyroRateZ a second read in a tick answers the first; the next tick is new") {
    pros::shim::resetAll();
    auto& st = pros::shim::imuState(kPort);
    const ProsClock clk;
    const ProsImu imu = readyImu(clk, YawRateSource::GyroRateZ);
    moveToUs(20'000);
    st.gyroZDegPerSecCw = 45.0;
    const double first = imu.yawRate().value();
    CHECK(first == doctest::Approx(-45.0 * kDegToRad));
    st.gyroZDegPerSecCw = -90.0;  // z changes inside the tick
    pros::shim::advanceUs(25);
    CHECK(imu.yawRate().value() == first);
    CHECK(st.gyroReads == 1);
    moveToUs(30'000);  // the next tick
    CHECK(imu.yawRate().value() == doctest::Approx(90.0 * kDegToRad));
    CHECK(st.gyroReads == 2);
}

// ═══ T7 — nothing else changed: the pre-IMU1 adapter is the oracle ══════════════════════════

namespace {
/// ceb17c0's differentiation, copied as the ORACLE — `ProsImu::differentiatedRate()`,
/// include/shulib/hal/pros/imu.hpp :179-203 at ceb17c0 (unchanged to 2ef1f3f, IMU1's base): a
/// screened read answers last-good and counts; the first good read answers 0; dt <= 0 repeats; else
/// the CW degrees since the last sample over the time since it, negated to canonical. The clock is
/// handed in — on a clock that stands still during a read, the old adapter's "read the clock after
/// the device" and the new "before" are the same instant (re-check R4/R17).
struct PreImu1Differentiator {
    double lastRate = 0.0;
    double lastRotationDeg = 0.0;
    double lastSampleTime = 0.0;
    bool hasRateSample = false;
    int faulted = 0;
    double read(double degCw, double now) {
        if (!std::isfinite(degCw)) {
            faulted += 1;
            return lastRate;
        }
        if (!hasRateSample) {
            hasRateSample = true;
            lastRotationDeg = degCw;
            lastSampleTime = now;
            return lastRate;
        }
        const double dt = (Time{now} - Time{lastSampleTime}).value();
        if (dt <= 0.0) {
            return lastRate;
        }
        const double cwDegPerSec = (degCw - lastRotationDeg) / dt;
        lastRotationDeg = degCw;
        lastSampleTime = now;
        lastRate = shulib::hal::imuYawRateToCanonical(cwDegPerSec).value();
        return lastRate;
    }
};
}  // namespace

// Bug caught (M7 — a faulted read moves the sample; M10 — the first call answers a rate from a zero
// baseline; any arithmetic drift in the rewrite): with ONE read per tick, every read >= 5 ms apart,
// the new adapter must answer BIT-IDENTICALLY to the old one on every tick — over jitter, an
// overrun, two faulted ticks and a calibration window. The first call is at t = 0.37 s with the
// rotation at 12.5 deg CW (both non-zero), so a zero-baseline first answer is a rate, not 0.
TEST_CASE("IMU1 T7: with one read per tick the adapter is bit-identical to the pre-IMU1 adapter") {
    pros::shim::resetAll();
    auto& st = pros::shim::imuState(kPort);
    FakeClock clk;  // stands still during a read (re-check R4/R17)
    const ProsImu imu = readyImu(clk);
    PreImu1Differentiator oracle;
    auto truth = [](double t) {
        const double u = t - 0.37;
        return 12.5 + 40.0 * u + 25.0 * std::sin(3.1 * u);  // CW degrees, 12.5 at the first call
    };
    std::uint32_t lcg = 12345U;
    auto jitter = [&lcg]() {  // ±0.3 ms, deterministic
        lcg = lcg * 1664525U + 1013904223U;
        return (static_cast<double>(lcg >> 8) / 16777216.0 - 0.5) * 0.0006;
    };
    clk.set(Time{0.37});
    int mismatches = 0;
    int ticks = 0;
    for (int k = 0; k < 300; ++k) {
        if (k > 0) {
            const double period = k == 150 ? 0.025 : kTick + jitter();  // tick 150 overran
            clk.advance(Time{period});
        }
        const double t = clk.now().value();
        st.disconnected = k == 60 || k == 200;      // two faulted ticks
        st.calibrating = k >= 100 && k < 105;        // a calibration window
        st.rotationDegCw = truth(t);
        const bool screened = st.disconnected || st.calibrating;
        const double expect = oracle.read(screened ? std::nan("") : st.rotationDegCw, t);
        const double got = imu.yawRate().value();
        if (k == 0) {
            CHECK(got == 0.0);  // the first call: one sample is not a derivative
        }
        mismatches += got == expect ? 0 : 1;
        ++ticks;
    }
    CHECK(ticks == 300);
    CHECK(mismatches == 0);
    CHECK(imu.faultedReads() == oracle.faulted);
    CHECK(oracle.faulted == 7);
}

// ═══ T12 — the clock is read BEFORE the device, and that one instant serves both ════════════
// (added 2026-10-29 at IMU1 Part A's review: the brief's landmine 4 had no test. T7's clock stands
// still during a read and the buffer model's every read takes the same `gap`, so a mutant that
// stamped either instant AFTER the device read was equivalent there — two such mutants passed the
// whole suite. Here the device read itself takes a VARYING time: the shim's micros() advances
// inside get_rotation(), through the rotationAt hook — no symbol the fix adds, so these build
// against either adapter.)

namespace {
std::uint64_t g_readLatencyUs = 0;  ///< how long the NEXT device read takes (T12)
double g_slowRotationDegCw = 0.0;   ///< what it returns
/// The rotationAt hook of T12: the read takes g_readLatencyUs on the robot's clock.
double slowDeviceRead(int /*port*/) {
    pros::shim::advanceUs(g_readLatencyUs);
    return g_slowRotationDegCw;
}
}  // namespace

// Bug caught (M13 — the hold's instant stamped from a second clock read AFTER the device read):
// a fresh read whose device read takes 3 ms would start its hold 3 ms late, so a read 5 000 µs after
// that read STARTED would be held — the hold must run from the answer's instant, read first.
TEST_CASE("IMU1 T12 (a): the hold runs from the instant read BEFORE a slow device read") {
    pros::shim::resetAll();
    auto& st = pros::shim::imuState(kPort);
    const ProsClock clk;
    const ProsImu imu = readyImu(clk);
    st.rotationAt = &slowDeviceRead;
    moveToUs(100'000);
    g_readLatencyUs = 0;
    g_slowRotationDegCw = 10.0;
    (void)imu.yawRate();  // primes the differentiator
    moveToUs(110'000);
    g_readLatencyUs = 3'000;  // this read takes 3 ms
    g_slowRotationDegCw = 11.0;
    const double answered = imu.yawRate().value();  // fresh, its instant 110 000 µs
    CHECK(answered == doctest::Approx(-100.0 * kDegToRad));  // 1 deg CW over 10 ms
    CHECK(pros::micros() == 113'000U);                         // the read really took 3 ms
    g_readLatencyUs = 0;
    const int before = st.rotationReads;
    moveToUs(114'999);  // 4 999 µs after the answer's instant: held
    CHECK(imu.yawRate().value() == answered);
    CHECK(st.rotationReads == before);
    moveToUs(115'000);  // 5 000 µs after it — though only 2 000 µs after its device read ended
    (void)imu.yawRate();
    CHECK(st.rotationReads == before + 1);  // fresh
}

// Bug caught (M12 — the derivative's instant re-read from the clock AFTER the device read, the
// pre-IMU1 order): the read's own latency would enter the rate's interval. Reads 10 ms apart at
// their START, the device read taking 0 and 3 ms alternately, the rotation 1 deg CW further each
// read: the rate is exactly 100 deg/s CW on every read when the instants are the reads' starts.
TEST_CASE("IMU1 T12 (b): a device read's varying latency never enters the rate's interval") {
    pros::shim::resetAll();
    const ProsClock clk;
    const ProsImu imu = readyImu(clk);
    pros::shim::imuState(kPort).rotationAt = &slowDeviceRead;
    int off = 0;
    for (int k = 0; k < 20; ++k) {
        moveToUs(200'000 + static_cast<std::uint64_t>(k) * 10'000);
        g_readLatencyUs = (k % 2 == 0) ? 0 : 3'000;
        g_slowRotationDegCw = static_cast<double>(k);
        const double w = imu.yawRate().value();
        if (k == 0) {
            CHECK(w == 0.0);  // the first call: one sample is not a derivative
        } else if (!(w == doctest::Approx(-100.0 * kDegToRad).epsilon(1e-9))) {
            ++off;
        }
    }
    CHECK(off == 0);
}

// ═══ T8 — the loop: the REAL Chassis / scheduler / Localizer / TANK1's MoveToPose ══════════

namespace {

/// The right side 9 % weak (TANK1's drift case): the steering correction has work to do every tick.
struct WeakRight final : sim::DegradationModel {
    units::Voltage effectiveVoltage(int wheel, units::Voltage cmd, Time, sim::Rng&) override {
        return wheel == 1 ? units::Voltage{cmd.value() * 0.91} : cmd;
    }
};

/// A record sink that wants every record (robot two's TermSink does) and keeps each record's
/// imuYawRate with the tick it was emitted in.
struct RecordTap final : shulib::hal::ITelemetrySink {
    explicit RecordTap(const int* tickCounter) : tick{tickCounter} {}
    void log(shulib::hal::LogLevel, std::string_view, std::string_view) override {}
    [[nodiscard]] bool wantsRecord() const noexcept override { return true; }
    void emit(const shulib::diag::DebugRecord& r) override {
        rates.push_back({*tick, r.imuYawRate.value()});
    }
    const int* tick;
    std::vector<SpyImu::Read> rates;
};

/// TANK1's P14 rig (test/motion_tank_steer_test.cpp :1262-1300) with robot two's IMU wiring: ONE
/// ProsImu over the model serves the context (every record's read) AND the Localizer
/// (device_provider.hpp :318), each through its own spy. The odometry reads the plant's FakeImu.
struct LoopRig;
struct LoopPacer final : motion::ITickPacer {
    void pace() override;
    LoopRig* r = nullptr;
};

struct LoopRig {
    WeakRight world;
    shulib::kinematics::TankKinematics kin{Length{12.0}};
    sim::SimHarness h;
    BufferModel model;
    imu_model::PlantHistory hist;
    CreepClock imuClock{model};
    ProsImu imu;
    int tick = 0;
    SpyImu locImu{imu, &tick};
    SpyImu ctxImu{imu, &tick};
    RecordTap tap{&tick};
    shulib::hal::NullSink none;
    chassis::RobotContext ctx;
    loc::PilonsOdometry odom;
    loc::ComplementaryFusion fusion;
    loc::Localizer localizer;
    shulib::hal::fake::FakeTelemetrySink faultSink;
    shulib::diag::FaultLatch latch;
    shulib::diag::HealthMonitor health;
    motion::MotionDeps deps;
    LoopPacer pacer;
    chassis::Chassis chassis;
    ScopedInstall installed{model, kPort};
    // Per tick (index = paces before it): the plant's time and heading at its start, the
    // Localizer's ω after it, the truth after the step, every commanded volt.
    std::vector<double> tStart, thetaStart, omega, volts;
    std::vector<sim::TruthSample> truth;

    static sim::SimHarnessConfig cfgFor(double kA) {
        sim::SimHarnessConfig c = motion_rig::plantConfig();
        c.plant.wheelFf.kA = kA;  // TANK1's P13 motor lag: 0.02 / 0.03
        return c;
    }
    LoopRig(double kA, double gap, double phase, bool records)
        : h{kin, cfgFor(kA), nullptr, &world},
          imu{static_cast<std::uint8_t>(kPort), Angle{}, imuClock},
          ctx{chassis::RobotContextConfig{
              .clock = &h.clock(),
              .driveMotors = h.context().driveMotors(),
              .imu = &ctxImu,
              .gps = &h.gps(),
              .battery = &h.battery(),
              .telemetry = records ? static_cast<shulib::hal::ITelemetrySink*>(&tap)
                                   : static_cast<shulib::hal::ITelemetrySink*>(&none),
              .tags = &h.context().tags(),
              .vision = &h.context().vision()}},
          odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()},
          localizer{h.clock(), locImu, odom, fusion},
          latch{faultSink, h.clock()},
          health{latch},
          deps{.ctx = &ctx, .localizer = &localizer, .kinematics = &kin, .faults = &latch,
               .health = &health},
          chassis{deps, pacer, motion_rig::chassisConfig()} {
        pacer.r = this;
        model.phase = phase;
        model.gap = gap;
        model.plantClock = &h.clock();
        hist.theta0 = h.plant().truthState().theta;
        hist.record(h.clock().now().value(), hist.theta0, h.trueBodyTwist().omega().value());
        model.truthCwDeg = [this](double c) { return hist.cwDegAt(c); };
        imu.calibrate();
        pros::shim::imuState(kPort).calibrating = false;
        localizer.setPose(motion_rig::plantConfig().plant.initialPose);
    }
    /// The tick in progress at the end of a run (its Localizer ω has no pace to capture it).
    void closeLastTick() {
        tStart.push_back(h.clock().now().value());
        thetaStart.push_back(h.plant().truthState().theta);
        omega.push_back(localizer.twist().omega().value());
    }
};

void LoopPacer::pace() {
    LoopRig& g = *r;
    if (g.truth.size() > 20000) {
        throw std::runtime_error("IMU1 T8: pace cap exceeded");
    }
    g.tStart.push_back(g.h.clock().now().value());
    g.thetaStart.push_back(g.h.plant().truthState().theta);
    g.omega.push_back(g.localizer.twist().omega().value());
    for (int w = 0; w < g.h.motorCount(); ++w) {
        g.volts.push_back(g.h.motor(w).commandedVoltage().value());
    }
    g.h.plant().step(Time{kTick});
    g.truth.push_back(g.h.sample());
    g.model.beginTick();  // the IMU's clock returns to the plant's (landmine 3)
    g.hist.record(g.h.clock().now().value(), g.h.plant().truthState().theta,
                  g.h.trueBodyTwist().omega().value());
    ++g.tick;
}

struct LoopOut {
    bool finished = false;
    std::vector<int> starts;  ///< each motion's first tick (the acceptance run: four)
    std::vector<double> omega, volts, thetaStart;
    std::vector<sim::TruthSample> truth;
    std::vector<SpyImu::Read> locReads, ctxReads, recordRates;
};

LoopOut runLoop(double kA, double gap, double phase, bool acceptance, bool records) {
    pros::shim::resetAll();
    LoopRig r{kA, gap, phase, records};
    LoopOut o;
    o.starts.push_back(0);
    if (acceptance) {
        // competition/acceptance_run.hpp :41-44's four legs, written inline (the brief's §6: its
        // AutonContext grows with MECH), one step at a time so the BOUNDARIES are known.
        chassis::Routine ra{r.chassis, "acceptance-run"};
        ra.moveTo(Pose2d{Length{24.0}, Length{0.0}, Angle{}});
        o.starts.push_back(r.tick);
        ra.turnTo(Angle::degrees(90.0));
        o.starts.push_back(r.tick);
        ra.face(Length{0.0}, Length{0.0});
        o.starts.push_back(r.tick);
        ra.driveTo(Length{0.0}, Length{0.0});
        o.finished = ra.ok();
    } else {
        o.finished =
            r.chassis.moveTo(Pose2d{Length{40.0}, Length{0.0}, Angle{}}) == ExitReason::Settled;
    }
    r.closeLastTick();
    o.omega = r.omega;
    o.volts = r.volts;
    o.thetaStart = r.thetaStart;
    o.truth = r.truth;
    o.locReads = r.locImu.reads;
    o.ctxReads = r.ctxImu.reads;
    o.recordRates = r.tap.rates;
    return o;
}

/// Ticks [0, upTo) of two runs bit-identical: truth, the Localizer's ω, every commanded volt.
bool samePrefix(const LoopOut& a, const LoopOut& b, std::size_t upTo) {
    if (a.truth.size() < upTo || b.truth.size() < upTo) {
        return false;
    }
    const std::size_t nm = a.volts.size() / std::max<std::size_t>(a.truth.size(), 1);
    return std::memcmp(a.truth.data(), b.truth.data(), upTo * sizeof(sim::TruthSample)) == 0
           && std::equal(a.omega.begin(), a.omega.begin() + static_cast<std::ptrdiff_t>(upTo),
                         b.omega.begin())
           && std::equal(a.volts.begin(), a.volts.begin() + static_cast<std::ptrdiff_t>(upTo * nm),
                         b.volts.begin());
}
bool sameRun(const LoopOut& a, const LoopOut& b) {
    return a.finished == b.finished && a.truth.size() == b.truth.size()
           && a.omega.size() == b.omega.size() && a.volts == b.volts
           && samePrefix(a, b, a.truth.size()) && a.omega == b.omega;
}

/// (b): every rate read in a tick — the Localizer's and every record's — equals that tick's first
/// (the Localizer's: update() runs before the motion's record); every record's imuYawRate equals
/// the Localizer's ω that tick. Returns the number of reads that differ.
int readsOffTheTick(const LoopOut& o) {
    int off = 0;
    std::vector<double> firstOf(o.omega.size(), std::nan(""));
    for (const SpyImu::Read& rd : o.locReads) {
        const auto i = static_cast<std::size_t>(rd.tick);
        if (std::isnan(firstOf[i])) {
            firstOf[i] = rd.value;
        } else {
            off += rd.value == firstOf[i] ? 0 : 1;
        }
    }
    for (const SpyImu::Read& rd : o.ctxReads) {
        off += rd.value == firstOf[static_cast<std::size_t>(rd.tick)] ? 0 : 1;
    }
    for (const SpyImu::Read& rd : o.recordRates) {
        off += rd.value == o.omega[static_cast<std::size_t>(rd.tick)] ? 0 : 1;
    }
    return off;
}

/// (c): ticks whose truth turned more than 0.05 rad/s over the last 10 ms and whose Localizer ω is
/// EXACTLY 0 — after the first three ticks of each motion (HA-213's copy lags a motion's start:
/// re-check R12).
int zeroTurningTicks(const LoopOut& o) {
    int zeros = 0;
    for (std::size_t i = 1; i < o.omega.size(); ++i) {
        bool exempt = false;
        for (const int s : o.starts) {
            const auto first = static_cast<std::size_t>(s);
            exempt = exempt || (i >= first && i < first + 3);
        }
        const double turned = (o.thetaStart[i] - o.thetaStart[i - 1]) / kTick;
        if (!exempt && std::abs(turned) > 0.05 && o.omega[i] == 0.0) {
            ++zeros;
        }
    }
    return zeros;
}

/// (d): at each chained-motion boundary the Localizer updates twice in one tick; the second
/// update's ω must be the first's. Returns the boundaries where it is not (or that lack two reads).
int boundaryMismatches(const LoopOut& o) {
    int bad = 0;
    for (std::size_t b = 1; b < o.starts.size(); ++b) {
        std::vector<double> at;
        for (const SpyImu::Read& rd : o.locReads) {
            if (rd.tick == o.starts[b]) {
                at.push_back(rd.value);
            }
        }
        const bool same =
            at.size() >= 2 && std::all_of(at.begin(), at.end(), [&](double v) { return v == at[0]; });
        bad += same ? 0 : 1;
    }
    return bad;
}

}  // namespace

// Bug caught: the record's read damaging the Localizer's own rate (probe 2 — on robot two the
// serial sink wants every record, so this happens EVERY motion tick): with the copy between the
// Localizer's read and the record's, the Localizer read 0 on every turning tick and TANK1's 40 in
// timed out with motor lag; far from it the Localizer read ~3 % high. With the hold: (a) records on
// == records off, bit for bit; (b) every read in a tick is the tick's first, and every record's
// imuYawRate is the Localizer's ω; (c) no exact-0 ω on a turning tick (past HA-213's start lag);
// (d) a boundary's second update() reads the first's ω; (e) every run finishes.
TEST_CASE("IMU1 T8: through the real loop, the record's read changes nothing the robot does") {
    for (const double kA : {0.0, 0.02, 0.03}) {
        for (const double gap : {25e-6, 100e-6}) {
            // in-gap: strictly between the Localizer's rate read (the tick's 2nd device read) and
            // the record's heading read (the 3rd); far: 20 gaps and 4.5 ms. Never ON a read
            // instant (re-check R12); [3, 4) gaps is T8 (a′)'s.
            for (const double phi : {2.25 * gap, 2.5 * gap, 2.75 * gap, 20.0 * gap, 0.0045}) {
                for (const bool acceptance : {false, true}) {
                    CAPTURE(kA);
                    CAPTURE(gap);
                    CAPTURE(phi);
                    CAPTURE(acceptance);
                    const LoopOut on = runLoop(kA, gap, phi, acceptance, true);
                    const LoopOut off = runLoop(kA, gap, phi, acceptance, false);
                    CHECK(sameRun(on, off));                 // (a)
                    CHECK(!on.recordRates.empty());          // the records-on run HAD a reader
                    CHECK(readsOffTheTick(on) == 0);         // (b)
                    CHECK(zeroTurningTicks(on) == 0);        // (c)
                    CHECK(zeroTurningTicks(off) == 0);
                    CHECK(boundaryMismatches(on) == 0);      // (d)
                    CHECK(boundaryMismatches(off) == 0);
                    CHECK(on.finished);                      // (e)
                    CHECK(off.finished);
                }
            }
        }
    }
}

// Bug caught: an over-promise. At a chained-motion boundary the scheduler's second update() reads
// the HEADING one device read later with records on (the record's heading read sits before it);
// a copy in [3, 4) gaps falls between the two, and heading() is deliberately NOT held (landmine 9),
// so the later read sees the newer copy — physical, not a defect (re-check R11). What the hold
// DOES promise there: records on == off up to the first boundary; after it, every rate read in a
// tick is the tick's first, and both runs finish.
TEST_CASE("IMU1 T8 (a'): in the one band (a) cannot promise, on == off up to the first boundary") {
    for (const double kA : {0.0, 0.02, 0.03}) {
        for (const double gap : {25e-6, 100e-6}) {
            for (const double phi : {3.25 * gap, 3.5 * gap, 3.75 * gap}) {
                for (const bool acceptance : {false, true}) {
                    CAPTURE(kA);
                    CAPTURE(gap);
                    CAPTURE(phi);
                    CAPTURE(acceptance);
                    const LoopOut on = runLoop(kA, gap, phi, acceptance, true);
                    const LoopOut off = runLoop(kA, gap, phi, acceptance, false);
                    if (acceptance) {
                        REQUIRE(on.starts.size() == 4);
                        CHECK(samePrefix(on, off, static_cast<std::size_t>(on.starts[1])));
                    } else {
                        CHECK(sameRun(on, off));  // no boundary: identical throughout
                    }
                    CHECK(readsOffTheTick(on) == 0);
                    CHECK(on.finished);
                    CHECK(off.finished);
                }
            }
        }
    }
}

// ═══ T9 — a corrector's gate reads first ════════════════════════════════════════════════════

namespace {

/// An odometry that reports the robot standing on the origin (T9 turns it in place).
struct StillOdometry final : loc::IOdometry {
    void update() override {}
    [[nodiscard]] Pose2d pose() const noexcept override { return Pose2d{}; }
    void setPose(const Pose2d&) override {}
    [[nodiscard]] bool lastDeltaImplausible() const noexcept override { return false; }
    [[nodiscard]] shulib::diag::OdomTickAudit lastTickAudit() const noexcept override { return {}; }
};

/// A corrector that takes TIME after it proposes — the arithmetic between the gate's rate read and
/// the Localizer's (the GPS's latency carry, its gate, the fusion), modelled as `work` seconds on
/// the IMU's creeping clock. On a fresh-fix tick the gate's read comes BEFORE it; on a stale tick
/// the Localizer's read (the tick's first) comes after it — the first read MOVES (re-check R15).
struct WorkSpan final : loc::ICorrector {
    WorkSpan(loc::ICorrector& c, BufferModel& m, double w) : inner{&c}, model{&m}, work{w} {}
    loc::CorrectionProposal propose(const Pose2d& predicted, Time dt) override {
        const loc::CorrectionProposal p = inner->propose(predicted, dt);
        model->creep += work;
        return p;
    }
    [[nodiscard]] const char* name() const noexcept override { return inner->name(); }
    loc::ICorrector* inner;
    BufferModel* model;
    double work;
};

struct GateRun {
    int ticks = 0;
    int gateDisagrees = 0;   ///< fresh ticks where "the gate rejected" != "|the Localizer's ω| > max"
    int gateRejects = 0;
    int freshTicks = 0;
    int freshValueDiffers = 0;  ///< (b): fresh ticks where the gate's value != the Localizer's ω
    int exact = 0, zero = 0, doubled = 0, other = 0;  ///< the Localizer's ω against the truth
};

GateRun runGate(double rateScale, int freshEvery, double phase) {
    pros::shim::resetAll();
    constexpr double kGap = 25e-6;
    constexpr double kWork = 60e-6;
    FakeClock plant;
    BufferModel m;
    m.phase = phase;
    m.gap = kGap;
    m.plantClock = &plant;
    const loc::GpsCorrectorConfig gcfg{};
    const double rateRad = rateScale * gcfg.maxYawRate.value();  // canonical magnitude, turning CW
    const double truth = -rateRad;
    m.truthCwDeg = [rateRad](double t) { return rateRad / kDegToRad * t; };
    const CreepClock clk{m};
    const ProsImu imu = readyImu(clk);
    const ScopedInstall installed{m, kPort};
    int tick = 0;
    SpyImu gateImu{imu, &tick};
    SpyImu locImu{imu, &tick};
    shulib::hal::fake::FakeGps gps;
    gps.setHasFix(true);
    gps.setRmsError(Length{0.5});
    loc::GpsCorrector corrector{plant, gps, gateImu, gcfg};
    WorkSpan timed{corrector, m, kWork};
    std::array<loc::ICorrector*, 1> correctors{&timed};
    StillOdometry odom;
    loc::ComplementaryFusion fusion;
    loc::Localizer localizer{plant, locImu, odom, fusion,
                             std::span<loc::ICorrector* const>{correctors.data(), correctors.size()}};
    GateRun g;
    double nudge = 0.001;
    for (int k = 0; k < 305; ++k) {
        tick = k;
        if (k > 0) {
            plant.advance(Time{kTick});
        }
        m.beginTick();
        const bool fresh = k % freshEvery == 0;
        if (fresh) {
            nudge = -nudge;  // a new sample: the freshness check sees it changed
            gps.setPose(Pose2d{Length{nudge}, Length{0.0}, Angle{}});
        }
        const std::uint32_t rejectsBefore = corrector.yawRateRejects();
        const std::size_t gateReadsBefore = gateImu.reads.size();
        localizer.update();
        const bool rejected = corrector.yawRateRejects() > rejectsBefore;
        const double w = localizer.twist().omega().value();
        g.gateRejects += rejected ? 1 : 0;
        if (gateImu.reads.size() > gateReadsBefore) {  // the gate read the rate: a fresh fix
            ++g.freshTicks;
            g.freshValueDiffers += gateImu.reads.back().value == w ? 0 : 1;
            g.gateDisagrees += rejected == (std::abs(w) > gcfg.maxYawRate.value()) ? 0 : 1;
        }
        if (k >= 5) {
            ++g.ticks;
            if (relClose(w, truth, 0.01)) {
                ++g.exact;
            } else if (w == 0.0) {
                ++g.zero;
            } else if (std::abs(w / truth - 2.0) < 0.05) {
                ++g.doubled;
            } else {
                ++g.other;
            }
        }
    }
    return g;
}

}  // namespace

// Bug caught: the gate and the Localizer disagreeing about one tick's yaw rate — on a corrected
// robot the gate (STEP 3, inside propose()) reads FIRST and the Localizer second, so before IMU1 a
// fix taken mid-spin could be gated on a rate the Localizer never saw. (a) a fresh fix every tick:
// the gate rejects for yaw rate exactly on the ticks whose Localizer ω exceeds maxYawRate.
TEST_CASE("IMU1 T9 (a): the GPS gate rejects for yaw rate exactly when the Localizer's rate is over") {
    for (const double scale : {1.2, 0.8}) {
        for (const double phi : {80e-6, 0.005}) {  // in the gate→Localizer work span, and far
            CAPTURE(scale);
            CAPTURE(phi);
            const GateRun g = runGate(scale, 1, phi);
            CHECK(g.gateDisagrees == 0);
            CHECK(g.freshTicks == 305);
            if (scale > 1.0) {
                CHECK(g.gateRejects >= 300);  // the gate really is reached and really rejects
            } else {
                CHECK(g.gateRejects == 0);
            }
        }
    }
}

// Bug caught: the same disagreement at the realistic cadence — a ~20 Hz sensor against the 100 Hz
// loop (localizer.hpp :302-303), so the gate reads first only on a fresh-fix tick, 1 in 5. On every
// fresh tick the gate's value must be the Localizer's ω. The Localizer's exact / zero / double
// counts are the hold's limit (2) — the tick's FIRST read moving between the gate's position and
// the Localizer's, with the copy between the two (HA-214; re-check R15) — RECORDED, never asserted
// exact (the brief's prototype: 180 / 60 / 60 of 300 in the band; the cure is the Localizer reading
// its rate at the top of update(), the brief's §5, OUT for now). Held to a CEILING since IMU1 Part
// A's review (2026-10-29), because HA-214, HA-179 and the changelog quote them: in the band at most
// one zero and one double per fresh fix counted (ticks 5, 10, … 300: 60) and nothing else off; far
// from it, every tick exact.
TEST_CASE("IMU1 T9 (b): at a fresh fix every 5th tick the gate's rate is the Localizer's on every "
          "fresh tick") {
    for (const double scale : {1.2, 0.8}) {
        for (const double phi : {80e-6, 0.005}) {
            CAPTURE(scale);
            CAPTURE(phi);
            const GateRun g = runGate(scale, 5, phi);
            CHECK(g.freshTicks == 61);
            CHECK(g.freshValueDiffers == 0);
            CHECK(g.gateDisagrees == 0);
            CHECK(g.ticks == 300);
            if (phi < 0.001) {  // in the band: the residual, held to its ceiling
                CHECK(g.zero <= 60);
                CHECK(g.doubled <= 60);
                CHECK(g.other == 0);
            } else {
                CHECK(g.exact == g.ticks);
            }
            MESSAGE("IMU1 T9 (b) residual (limit 2, recorded): scale "
                    << scale << " phi " << phi << " | the Localizer's w of " << g.ticks << ": exact "
                    << g.exact << " zero " << g.zero << " double " << g.doubled << " other "
                    << g.other);
        }
    }
}

// ═══ T10 — Calibrate's REST record ══════════════════════════════════════════════════════════

namespace {

struct RestOut {
    bool done = false;
    double mean = 0.0;
    double sigma = 0.0;
    std::uint32_t samples = 0;
    std::vector<double> captureSamples;  ///< every capture tick's rate sample, in order
};

/// RestCapture over the ProsImu with the robot still and the IMU's rotation drifting 1 deg/s CW
/// (+0.01 deg per copy). A spy keeps every rate read with its tick; a capture tick's sample is its
/// LAST rate read (the start tick reads the settle gate first, then the capture's — :388, :456).
RestOut runRest(double gap, double phase) {
    pros::shim::resetAll();
    shulib::kinematics::TankKinematics kin{Length{12.5}};
    sim::SimHarness h{kin, sim::SimHarnessConfig{}, nullptr, nullptr};
    BufferModel m;
    m.phase = phase;
    m.gap = gap;
    m.plantClock = &h.clock();
    m.truthCwDeg = [](double t) { return 1.0 * t; };
    const CreepClock clk{m};
    ProsImu imu = readyImu(clk);
    const ScopedInstall installed{m, kPort};
    int tick = 0;
    SpyImu spy{imu, &tick};
    const cal::SideMember left[1]{{&h.motor(0), 1, 11}};
    const cal::SideMember right[1]{{&h.motor(1), 1, 16}};
    cal::RestCapture cap{cal::DriveSide{std::span<const cal::SideMember>{left, 1}, Length{1.625}},
                         cal::DriveSide{std::span<const cal::SideMember>{right, 1}, Length{1.625}},
                         Length{12.5}, spy, h.clock(), cal::RestCaptureConfig{}};
    cap.start();
    RestOut o;
    for (; tick < 20000; ++tick) {
        const cal::RestPhase before = cap.phase();
        const std::size_t readsBefore = spy.reads.size();
        cap.tick();
        const cal::RestPhase after = cap.phase();
        const bool captured = (before == cal::RestPhase::Capture || after == cal::RestPhase::Capture)
                              && (after == cal::RestPhase::Capture || after == cal::RestPhase::Done);
        if (captured && spy.reads.size() > readsBefore) {
            o.captureSamples.push_back(spy.reads.back().value);
        }
        if (after == cal::RestPhase::Done || after == cal::RestPhase::Aborted) {
            break;
        }
        h.plant().step(Time{kTick});
        m.beginTick();
    }
    o.done = cap.phase() == cal::RestPhase::Done;
    if (o.done) {
        o.mean = cap.summary().yawRateMean;
        o.sigma = cap.summary().yawRateSigma;
        o.samples = cap.summary().samples;
    }
    return o;
}

}  // namespace

// Bug caught: the REST record's rate σ — HA-22's evidence — inflated by the settle gate's read and
// the capture's first read sharing a tick (rest_capture.hpp :388, :417-426, :456): before IMU1 the
// first sample was a spike of one copy over a gap, σ 4.5e-2 rad/s in-band. (a) φ off the band:
// the mean is the drift within 0.1 % and σ is under 1e-5 rad/s ABSOLUTE — about 1/20 of what ONE
// doubled sample adds (re-check R14).
TEST_CASE("IMU1 T10 (a): a REST capture over the ProsImu records the drift with a clean sigma") {
    const double drift = -1.0 * kDegToRad;
    for (const double gap : {25e-6, 100e-6}) {
        for (const double phi : {2.25 * gap, 2.75 * gap, 0.005}) {
            CAPTURE(gap);
            CAPTURE(phi);
            const RestOut o = runRest(gap, phi);
            REQUIRE(o.done);
            CHECK(o.captureSamples.size() == o.samples);  // the spy found every sample
            CHECK(relClose(o.mean, drift, 1e-3));
            CHECK(o.sigma < 1e-5);
        }
    }
}

// Bug caught: an over-promise, and a regression past the hold's limit (2). When the copy lands
// between the settle read's position (the tick's 1st device read) and the capture read's (the 2nd,
// after heading()), the first FRESH capture read spans 10 ms + gap with TWO copies in it: exactly
// ONE capture sample is about twice the drift, every other is the drift, σ ≤ 3e-4 (re-check R14:
// the hold groups reads by time, so a moved first read is one doubled sample — HA-214's claim).
TEST_CASE("IMU1 T10 (b): in the one band, a REST capture carries exactly one doubled sample") {
    const double drift = -1.0 * kDegToRad;
    for (const double gap : {25e-6, 100e-6}) {
        for (const double phi : {1.25 * gap, 1.75 * gap}) {
            CAPTURE(gap);
            CAPTURE(phi);
            const RestOut o = runRest(gap, phi);
            REQUIRE(o.done);
            CHECK(o.captureSamples.size() == o.samples);
            int doubled = 0;
            int plain = 0;
            for (const double s : o.captureSamples) {
                if (s / drift > 1.9 && s / drift < 2.1) {
                    ++doubled;
                } else if (relClose(s, drift, 0.02)) {
                    ++plain;
                }
            }
            CHECK(doubled == 1);
            CHECK(plain == static_cast<int>(o.captureSamples.size()) - 1);
            CHECK(o.sigma <= 3e-4);
        }
    }
}

// ═══ T11 — the calibrator's rest gate ═══════════════════════════════════════════════════════

// Bug caught: a rest window that completes while the robot is turned by hand. DrivetrainCalibrator's
// tick reads the heading, then the rest window's rate (tickRest → atRest, :1018 → :1477 — the tick's
// FIRST rate read), then updateStatus's (:1740 — the LAST). Before IMU1, with the copy between the
// two, the first read was 0 — rest declared, the window completed, a pulse STARTED while the robot
// turned; far from it the last read was 0 — status().atRest lied (re-check R8). At 3 deg/s (over
// restYawRateFloor's 2 deg/s) with every motor still: no pulse ever starts, the window refuses with
// RestTimeout saying the robot is still rotating, and status().atRest is false once the window holds
// the turn (CAL2: the rest is the heading's average over restSettleTime, so a turn just begun reads
// still for restYawRateFloor / rate of that window -- here 2/3 of it).
TEST_CASE("IMU1 T11: the calibrator never starts a pulse while the robot is turned by hand at 3 deg/s") {
    using calib_rig::CalibRig;
    constexpr double kGapS = 25e-6;
    constexpr int kImuPort = 2;  // calib_rig::facts()'s IMU port
    for (const double phi :
         {0.5 * kGapS, 1.5 * kGapS, 2.5 * kGapS, 3.5 * kGapS, 20.0 * kGapS, 0.005}) {
        CAPTURE(phi);
        pros::shim::resetAll();
        const cal::CalibratorConfig ccfg{};
        CalibRig rig{ccfg, calib_rig::calibPlant(0.0)};
        BufferModel m;
        m.phase = phi;
        m.gap = kGapS;
        m.plantClock = &rig.h.clock();
        // The hand turn starts 2 in before the push ends (under a degree of yaw by DONE, inside the
        // 3 deg straightBound), so the rate is steady by the rest window's first tick — HA-213's
        // copy lags a turn's start. DONE is touched without waiting for pushReady(), whose atRest()
        // a robot turning for a whole window never satisfies (RunOptions::waitForReady).
        double turnFrom = -1.0;
        m.truthCwDeg = [&turnFrom](double t) {
            return turnFrom < 0.0 ? 0.0 : 3.0 * std::max(0.0, t - turnFrom);
        };
        const CreepClock clk{m};
        ProsImu imu = readyImu(clk, YawRateSource::DifferentiateRotation, kImuPort);
        const ScopedInstall installed{m, kImuPort};
        cal::DrivetrainCalibrator dc{rig.candidateSpan(), rig.portSpan(), imu, rig.h.clock(), ccfg};
        dc.start(calib_rig::facts());
        int restTicks = 0;
        int atRestTicks = 0;
        int poweredTicks = 0;
        calib_rig::RunOptions opt;
        opt.waitForReady = false;
        opt.hook = [&](CalibRig& rg, const calib_rig::Script& s) {
            m.beginTick();
            if ((s.pushed >= 46.0 || s.stage >= 1) && turnFrom < 0.0) {
                turnFrom = rg.h.clock().now().value();
            }
            if (dc.phase() == cal::Phase::SideAssign) {  // last tick's outcome, read before this one
                ++restTicks;
                // CAL2: rest averages the heading over restSettleTime, so a turn that has just begun
                // reads still until the window holds enough of it -- never past one window.
                atRestTicks += dc.status().atRest && restTicks > 50 ? 1 : 0;
                for (std::size_t i = 0; i < CalibRig::kCandidates; ++i) {
                    poweredTicks += rg.candidates[i]->commandedVoltage().value() != 0.0 ? 1 : 0;
                }
            }
        };
        const cal::Phase end = calib_rig::runWith(rig, dc, opt);
        CHECK(end == cal::Phase::Aborted);
        CHECK(dc.refusal() == cal::Refusal::RestTimeout);
        CHECK(dc.refusalPort() == 0);
        CHECK(calib_rig::reasonMentions(dc, "still rotating"));
        CHECK(restTicks >= 290);  // the 3 s rest window ran to its timeout
        CHECK(atRestTicks == 0);
        CHECK(poweredTicks == 0);
    }
}

// ═══ L1–L3 — the hold's three limits, as the changelog and the register state them ═══════════
// (added 2026-10-29 at IMU1 Part A's review: these figures were quoted in docs/changelog.md and
// docs/hardware-assumptions.md — HA-213, HA-214, HA-215 — from the IMU1 brief's probes 3 and 3b
// and its re-check's fast-loop and span probes, which ran on a PROTOTYPE of the hold; no suite test
// held them. They are promoted here, on the REAL adapter, so a change to the hold that moves one goes
// red.) These are the hold's documented LIMITS, not its specification: a change that moves a figure
// moves the sentences that quote it in the same commit — and if it cures a limit, the limit's text.
// Every figure is the host test plant's (the 10 ms copy of HA-213 at a chosen phase, a constant
// 90 deg/s CW turn); none has been measured on a robot.

namespace {

/// The Localizer's first read of a tick, classified against the truth (the probes' classes).
struct LimitCounts {
    int n = 0;
    int exact = 0;    ///< within 1 % of the truth
    int zero = 0;     ///< exactly 0
    int low = 0;      ///< under the truth, not 0
    int high = 0;     ///< over the truth
    int doubled = 0;  ///< of `high`: about twice the truth (within 5 %)
    int laterDiffer = 0;  ///< ticks where a later read of the tick answered differently
};

/// One ProsImu over the 10 ms copy at `phase`; tick k's reads at the instants `reads(k)` (seconds;
/// the first is the Localizer's), every read at exactly its instant (the model's `gap` 0 — the
/// schedule IS the case). The first `warmUp` ticks are not counted (the probes' warm-up).
LimitCounts runSchedule(double phase, int ticks, int warmUp,
                        const std::function<std::vector<double>(int)>& reads) {
    pros::shim::resetAll();
    BufferModel m;
    m.phase = phase;
    m.gap = 0.0;
    m.truthCwDeg = [](double t) { return kTurnDegCw * t; };
    const CreepClock clk{m};
    const ProsImu imu = readyImu(clk);
    const ScopedInstall installed{m, kPort};
    LimitCounts c;
    for (int k = 0; k < ticks; ++k) {
        const std::vector<double> at = reads(k);  // called on every tick: a case's draws stay in step
        double first = 0.0;
        bool differ = false;
        for (std::size_t i = 0; i < at.size(); ++i) {
            m.beginTick(at[i]);
            const double v = imu.yawRate().value();
            if (i == 0) {
                first = v;
            } else {
                differ = differ || v != first;
            }
        }
        if (k < warmUp || at.empty()) {
            continue;
        }
        ++c.n;
        c.laterDiffer += differ ? 1 : 0;
        if (relClose(first, kTurnCanonical, 0.01)) {
            ++c.exact;
        } else if (first == 0.0) {
            ++c.zero;
        } else if (std::abs(first) < std::abs(kTurnCanonical)) {
            ++c.low;
        } else {
            ++c.high;
            c.doubled += std::abs(first / kTurnCanonical - 2.0) < 0.05 ? 1 : 0;
        }
    }
    return c;
}

/// The probes' deterministic draws in [0, 1) (an LCG; the same constants, so the same schedules).
struct ProbeDraws {
    unsigned seed;
    double next() {
        seed = seed * 1103515245U + 12345U;
        return static_cast<double>((seed >> 8) & 0xFFFFFFU) / 16777216.0;
    }
};

constexpr double kLocAt = 0.0001;     ///< the Localizer's read, 0.1 ms after the wake (the probes')
constexpr double kRecordAt = 0.0004;  ///< the record's read, 0.4 ms after it

}  // namespace

// Limit (1), a stall or a late wake (HA-215; probe 3b). Stated in the changelog and ProsImu's header
// as: a tick that stalls more than 5 ms between two of its reads gives its later readers a fresh
// answer, and the Localizer's next two ticks read off (one held from that answer, one low) — then
// nothing; a tick that wakes 5 ms or more late is followed by an on-time wake under 5 ms after it,
// which gets the late tick's answer: three of the Localizer's ticks off, where a 4 ms late wake (the
// hold not involved) costs two.
TEST_CASE("IMU1 L1: a stall over 5 ms costs the Localizer two ticks, a wake 5 ms late three — then "
          "nothing") {
    auto onGrid = [](int k) { return static_cast<double>(k) * kTick; };
    for (const double phi : {0.005, 0.003}) {  // a copy far from the reads, and one inside the stall
        CAPTURE(phi);
        const LimitCounts stall = runSchedule(phi, 305, 5, [&](int k) {
            return std::vector<double>{onGrid(k) + kLocAt, onGrid(k) + (k == 100 ? 0.0061 : kRecordAt)};
        });
        CHECK(stall.n == 300);
        CHECK(stall.exact == 298);  // tick 101 held from the stall's fresh read, tick 102 low
        CHECK(stall.high == 1);
        CHECK(stall.low == 1);
        CHECK(stall.laterDiffer == 1);  // only the stalled tick's later reader was answered apart
    }
    for (const double late : {0.004, 0.006}) {
        CAPTURE(late);
        const LimitCounts wake = runSchedule(0.005, 305, 5, [&](int k) {
            const double start = onGrid(k) + (k == 101 ? late : 0.0);
            return std::vector<double>{start + kLocAt, start + kRecordAt};
        });
        CHECK(wake.n == 300);
        CHECK(wake.laterDiffer == 0);
        if (late < 0.005) {  // 4 ms late: tick 102 wakes 6 ms after it — fresh, the hold not involved
            CHECK(wake.exact == 298);
            CHECK(wake.low == 1);
            CHECK(wake.high == 1);
        } else {  // 6 ms late: tick 102 wakes 4 ms after it — HELD, one tick stale
            CHECK(wake.exact == 297);
            CHECK(wake.high == 2);
            CHECK(wake.low == 1);
        }
    }
}

// Limit (2)'s residual, NOT cured (HA-214; probe 3 (2) and (2′)): the copy lands within a read's
// jitter of the tick's ONE rate read (±0.05 ms of wake jitter, the copy AT the Localizer's instant)
// — 73 zero and 73 about-double of 300, the same with one reader or two. The register quotes it.
TEST_CASE("IMU1 L2: a copy at the tick's one rate read gives 73 zero and 73 double of 300, one "
          "reader or two") {
    ProbeDraws draws{12345U};
    std::vector<double> starts(305);
    for (int k = 0; k < 305; ++k) {  // probe 3's case (1) drew first: its draws are skipped in step
        (void)draws.next();
    }
    for (int k = 0; k < 305; ++k) {
        starts[static_cast<std::size_t>(k)] =
            static_cast<double>(k) * kTick + (draws.next() - 0.5) * 0.0001;
    }
    for (const int readers : {2, 1}) {
        CAPTURE(readers);
        const LimitCounts r = runSchedule(kLocAt, 305, 5, [&](int k) {
            const double s = starts[static_cast<std::size_t>(k)];
            return readers == 2 ? std::vector<double>{s + kLocAt, s + kRecordAt}
                                : std::vector<double>{s + kLocAt};
        });
        CHECK(r.n == 300);
        CHECK(r.exact == 154);
        CHECK(r.zero == 73);
        CHECK(r.high == 73);
        CHECK(r.doubled == 73);
        CHECK(r.low == 0);
        CHECK(r.laterDiffer == 0);  // the hold still gives the tick's readers one value
    }
}

// Limit (3), a reader outside the tick, and HA-215's spread reader (the re-check's span probe), at
// three copy phases; and HA-213's caution for a loop of your own (the re-check's fast-loop probe).
// The changelog and the register quote: an off-tick read 5–10 ms after a tick's first costs the
// Localizer two ticks per read (one read every 10 ticks: 60 of 300 off); a later reader at a random
// +4..+7 ms leaves 55 of 300 exact; readers at a steady +4, +6 or +7.5 ms leave all 300 exact (a
// constant turn hides the one-partial-tick lateness of the +6 and +7.5 ms cases); a 4 ms loop reads 0
// on one read in five and never exactly, a 10 ms loop exactly on every read.
TEST_CASE("IMU1 L3: a reader outside the tick costs two Localizer ticks per read; a 4 ms loop reads "
          "0 one read in five") {
    auto onGrid = [](int k) { return static_cast<double>(k) * kTick; };
    for (const double phi : {0.003, 0.0055, 0.008}) {
        CAPTURE(phi);
        const LimitCounts outside = runSchedule(phi, 305, 5, [&](int k) {
            return k % 10 == 3 ? std::vector<double>{onGrid(k) + kLocAt, onGrid(k) + 0.0065}
                               : std::vector<double>{onGrid(k) + kLocAt};
        });
        CHECK(outside.n == 300);
        CHECK(outside.n - outside.exact == 60);  // 30 off-tick reads, two Localizer ticks each
        ProbeDraws draws{7U};
        const LimitCounts spread = runSchedule(phi, 305, 5, [&](int k) {
            return std::vector<double>{onGrid(k) + kLocAt, onGrid(k) + 0.004 + 0.003 * draws.next()};
        });
        CHECK(spread.exact == 55);
        for (const double later : {0.004, 0.006, 0.0075}) {
            CAPTURE(later);
            const LimitCounts steady = runSchedule(phi, 305, 5, [&](int k) {
                return std::vector<double>{onGrid(k) + kLocAt, onGrid(k) + later};
            });
            CHECK(steady.exact == 300);
        }
    }
    for (const double period : {0.004, 0.010}) {
        CAPTURE(period);
        const LimitCounts loop = runSchedule(0.0033, 3000, 20, [&](int i) {
            return std::vector<double>{static_cast<double>(i + 1) * period};
        });
        CHECK(loop.n == 2980);
        if (period < 0.005) {
            CHECK(loop.exact == 0);
            CHECK(loop.zero * 5 == loop.n);  // one read in five
        } else {
            CHECK(loop.exact == 2980);
        }
    }
}
