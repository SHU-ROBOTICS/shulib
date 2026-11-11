// bench_imu_timing_test.cpp — adversarial tests for Bench Tests' STAGE 2c, "the IMU's timing"
// (IMU1 Part B, 2026-11-03; the IMU1 brief's D3 (a), §1 item 5, re-checks R10 and R18).
//
// The stage reads PROS (src/programs/bench_r3a.cpp, imuTimingStage()) and cannot run here. What it
// CONCLUDES is arithmetic in src/programs/imu_timing.hpp — PROS-free on purpose (the bench_r3a.hpp
// precedent) — and that arithmetic decides whether a belief prints HOLDS or WRONG on the robot. So
// every part is tried here: the interval statistics, the phase from the change instants (its unwrap
// and its least-squares drift), the trapezoid integral and its window, and every verdict AT its
// edge (each edge chosen so the double arithmetic lands on it exactly). The realistic inputs come
// from a sample-and-hold model of the IMU (HA-213's belief: the readings copied every period at a
// phase, every read between two copies the same double) polled the way the stage polls — the
// host test plant's model, never a claim about a robot.
//
// What only a SOURCE PIN can guard (the stage's own code, which the host cannot compile) is pinned
// at the end: the stage commands no motor and writes no device, never calls ProsImu::yawRate()
// (R18), pulls in no pacer or Localizer header (R10), prints through SER1's one-call path, runs
// from reportImu after STAGE 2's verdict, and still runs under 7 RUN ALL.
//
// Nothing in this file has run on a V5. The stage's numbers come from worksheet Station N, run by
// the team.

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "programs/imu_timing.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "source_pins.hpp"

namespace {

namespace it = shulib::bench::imu_timing;

using source_pins::bodyOf;
using source_pins::count;
using source_pins::readFile;
using source_pins::stripComments;

// ── the host test plant's model of HA-213: a sample-and-hold copy ───────────────────────

/// The IMU as HA-213 believes it: the rotation (and the gyro's z) copied into the buffer every
/// `periodUs` at instants phaseUs + k·periodUs; a read returns the last copy's values. The robot
/// turns at `rateDegPerS` from t = 0 (rotation = rate · copy instant); `zScale` multiplies the
/// gyro's z (1 = deg/s with get_rotation()'s sign, −1 the opposite sign, π/180 rad/s, 0 another axis).
struct HeldImu {
    double periodUs = 10000.0;
    double phaseUs = 3400.0;
    double rateDegPerS = 90.0;
    double zScale = 1.0;

    /// The instant of the last copy at or before `tUs`.
    [[nodiscard]] double copyAt(std::uint64_t tUs) const {
        const double t = static_cast<double>(tUs);
        return phaseUs + std::floor((t - phaseUs) / periodUs) * periodUs;
    }
    [[nodiscard]] double rotation(std::uint64_t tUs) const {
        return rateDegPerS * copyAt(tUs) * 1e-6;
    }
    [[nodiscard]] double z(std::uint64_t /*tUs*/) const { return zScale * rateDegPerS; }
};

/// The host's stand-in for pros::micros(): a clock that moves only when the model moves it — a
/// device read advances it by the read's cost, a wait sets it to the next wake. The stage's loops
/// (imu_timing.hpp's runCadence / runPhase / runSpans / runGyro) call it exactly as they call
/// pros::micros() on the robot.
struct SimClock {
    std::uint64_t t = 0;
    std::uint64_t operator()() const { return t; }
};

/// The stage's REAL 2c.1 loop (it::runCadence) over the model: each read costs `pollUs`; returns
/// the change instants it kept (room for 4096 — every test here REQUIREs no overflow).
std::vector<std::uint64_t> cadenceInstants(const HeldImu& imu, std::uint64_t startUs,
                                           std::uint64_t durationUs, std::uint64_t pollUs,
                                           it::ChangeWatch& watch) {
    SimClock clock{startUs};
    const auto read = [&] {
        const double v = imu.rotation(clock.t);
        clock.t += pollUs;
        return v;
    };
    std::vector<std::uint64_t> at(4096, 0U);
    const it::CadenceRun run =
        it::runCadence(clock, read, durationUs, watch, std::span<std::uint64_t>(at));
    REQUIRE(run.overflow == 0);
    at.resize(run.stored);
    return at;
}

/// The stage's REAL 2c.2 loop (it::runPhase) over the model: tick i wakes at w0 + i·10 ms
/// (+ jitter[i % n]); each read costs `pollUs`.
std::vector<it::TickPhase> phaseTicks(const HeldImu& imu, std::uint64_t w0, int ticks,
                                      std::uint64_t pollUs, const std::vector<int>& jitterUs = {0}) {
    SimClock clock{0};
    std::size_t i = 0;
    const auto wait = [&] {
        const std::int64_t wake = static_cast<std::int64_t>(w0) + static_cast<std::int64_t>(i) * 10000
                                  + jitterUs[i % jitterUs.size()];
        ++i;
        REQUIRE(wake >= static_cast<std::int64_t>(clock.t));  // delay_until never wakes in the past
        clock.t = static_cast<std::uint64_t>(wake);
    };
    const auto read = [&] {
        const double v = imu.rotation(clock.t);
        clock.t += pollUs;
        return v;
    };
    std::vector<it::TickPhase> out(static_cast<std::size_t>(ticks));
    it::runPhase(clock, wait, read, std::span<it::TickPhase>(out));
    return out;
}

/// The stage's REAL 2c.4 loop (it::runGyro) over the model: the rotation and z read at one
/// instant, then the clock moves `pollUs`.
it::GyroTurn gyroTurn(const HeldImu& imu, std::uint64_t startUs, std::uint64_t durationUs,
                      std::uint64_t pollUs) {
    SimClock clock{startUs};
    const auto rotation = [&] { return imu.rotation(clock.t); };
    const auto z = [&] {
        const double v = imu.z(clock.t);
        clock.t += pollUs;
        return v;
    };
    it::GyroTurn turn;
    it::runGyro(clock, rotation, z, durationUs, turn);
    return turn;
}

it::Cadence cadenceOf(const std::vector<std::uint64_t>& at) {
    static std::vector<std::uint32_t> scratch;
    scratch.assign(at.size() + 1, 0U);
    return it::cadenceFrom(std::span<const std::uint64_t>(at), std::span<std::uint32_t>(scratch));
}

it::Phase phaseOf(const std::vector<it::TickPhase>& ticks) {
    static std::vector<std::uint32_t> scratch;
    scratch.assign(ticks.size() + 1, 0U);
    return it::phaseFrom(std::span<const it::TickPhase>(ticks), it::kCopyPeriodUs,
                         std::span<std::uint32_t>(scratch));
}

it::Summary summaryOf(std::vector<std::uint32_t> v) { return it::summarize(std::span<std::uint32_t>(v)); }

/// A watch that saw `changes` changes on good reads (and a baseline): for the verdict edges.
it::ChangeWatch watchWithChanges(std::size_t changes) {
    it::ChangeWatch w;
    for (std::size_t k = 0; k <= changes; ++k) (void)w.feed(static_cast<double>(k));
    return w;
}

}  // namespace

// ═══ the constants the verdicts rest on ════════════════════════════════════════════════

TEST_CASE("IMU1-B (2c.3): the span verdict's hold IS the library's rate hold — they cannot drift apart") {
    // bench_r3a.cpp static_asserts the same equality; this is the host's copy of that check, so a
    // change to ProsImu::kRateHoldUs fails the host build's test too, not only the robot build.
    static_assert(it::kHoldUs == shulib::hal::pros::ProsImu::kRateHoldUs);
    CHECK(it::kSpanWellUnderUs * 2U == it::kHoldUs);
    CHECK(it::kCopyPeriodUs == 10000U);  // PROS's documented copy (HA-213)
    CHECK(it::kPhaseWindowUs < it::kCopyPeriodUs);  // a poll must end before the next wake
}

TEST_CASE("IMU1-B: every threshold is pinned to the value Station N and the register quote") {
    // The tests below reach most thresholds by NAME, so a changed constant would move them too; the
    // worksheet's N.4 and the register's HA-213/214 Settle quote the numbers. The stage prints them
    // from these constants, so this case is what keeps the printed words, the worksheet and the
    // register one set of numbers (the IMU1-B review, 2026-11-05).
    CHECK(it::kBandUs == 1000U);             // "9 000–11 000 us"
    CHECK(it::kMinIntervals == 20U);         // "at least 20 intervals"
    CHECK(it::kPhaseWindowUs == 9500U);      // "no change within 9500 us"
    CHECK(it::kMinPhaseTicks == 20U);
    CHECK(it::kFixedDriftUsPerS == 10.0);    // "|drift| ... <= 10 us/s"
    CHECK(it::kDriftSigmas == 3.0);          // "3 SE"
    CHECK(it::kSpanWellUnderUs == 2500U);    // "under 2.5 ms"
    CHECK(it::kGyroMinWindowUs == 2000000U); // "at least 2 s"
    CHECK(it::kGyroMinTurnDeg == 30.0);      // "30 degrees"
    CHECK(it::kGyroTolerance == 0.10);       // "within 10 %"
}

// ═══ summarize / usBetween ═════════════════════════════════════════════════════════════

TEST_CASE("IMU1-B summarize: min / LOWER median / max, the median always a measured value") {
    CHECK(summaryOf({}).n == 0);
    CHECK(summaryOf({}).median == 0);
    const it::Summary one = summaryOf({7});
    CHECK((one.n == 1 && one.min == 7 && one.median == 7 && one.max == 7));
    const it::Summary odd = summaryOf({30, 10, 20});
    CHECK((odd.min == 10 && odd.median == 20 && odd.max == 30));
    // An even count: the LOWER middle (2), never the average (2.5 -> 2 or 3 by rounding) or the upper (3).
    const it::Summary even = summaryOf({4, 1, 3, 2});
    CHECK((even.n == 4 && even.min == 1 && even.median == 2 && even.max == 4));
    // A median that only a sort finds: unsorted input with the middle value last.
    const it::Summary unsorted = summaryOf({9, 1, 8, 2, 5});
    CHECK(unsorted.median == 5);
}

TEST_CASE("IMU1-B usBetween: a backwards or equal pair is 0, a huge gap is clamped, exact otherwise") {
    CHECK(it::usBetween(100, 100) == 0);
    CHECK(it::usBetween(100, 99) == 0);  // never a wrapped 2^64 - 1
    CHECK(it::usBetween(100, 10100) == 10000);
    constexpr std::uint64_t kMax32 = std::numeric_limits<std::uint32_t>::max();
    CHECK(it::usBetween(0, kMax32) == kMax32);       // the edge itself is exact
    CHECK(it::usBetween(0, kMax32 + 1) == kMax32);   // one past it is clamped, not wrapped to 0
    CHECK(it::usBetween(5, kMax32 * 3) == kMax32);
}

// ═══ ChangeWatch: what counts as a copy ════════════════════════════════════════════════

TEST_CASE("IMU1-B ChangeWatch: the first good read is a baseline; a change is ANY different double") {
    it::ChangeWatch w;
    CHECK_FALSE(w.feed(12.5));  // the baseline, not a change
    CHECK_FALSE(w.feed(12.5));  // the same copy
    CHECK(w.feed(12.75));
    // No tolerance: the smallest possible move is still a copy that moved the value.
    CHECK(w.feed(std::nextafter(12.75, 100.0)));
    CHECK_FALSE(w.feed(std::nextafter(12.75, 100.0)));
    CHECK(w.polls() == 5);
    CHECK(w.changes() == 2);
    CHECK(w.badReads() == 0);
}

TEST_CASE("IMU1-B ChangeWatch: a PROS_ERR_F read is counted and skipped — never a change, never the value") {
    const double kErr = std::numeric_limits<double>::infinity();  // PROS_ERR_F
    const double kNan = std::numeric_limits<double>::quiet_NaN();
    it::ChangeWatch w;
    CHECK_FALSE(w.feed(kErr));  // no baseline yet, and an error cannot become one
    CHECK_FALSE(w.feed(3.0));   // ... so THIS is the baseline
    CHECK_FALSE(w.feed(kNan));
    CHECK_FALSE(w.feed(3.0));   // the same value across an error: still no change
    CHECK_FALSE(w.feed(-kErr));
    CHECK(w.feed(4.0));         // compared with 3.0, not with the error
    CHECK(w.badReads() == 3);
    CHECK(w.polls() == 6);
    CHECK(w.changes() == 1);
}

// ═══ 2c.1 the cadence (HA-213) ═════════════════════════════════════════════════════════

TEST_CASE("IMU1-B (2c.1): a 10 ms sample-and-hold copy polled as the stage polls -> HA-213 HOLDS") {
    for (const double phase : {0.0, 3400.0, 9999.0}) {
        for (const std::uint64_t poll : {3U, 7U, 41U}) {
            CAPTURE(phase);
            CAPTURE(poll);
            HeldImu imu;
            imu.phaseUs = phase;
            it::ChangeWatch w;
            const auto at = cadenceInstants(imu, 1234567, 2000000, poll, w);
            const it::Cadence c = cadenceOf(at);
            CHECK(c.interval.n >= 198);
            CHECK(c.interval.n <= 200);
            CHECK(c.inBand == c.interval.n);
            CHECK(c.below == 0);
            CHECK(c.above == 0);
            // A poll quantizes each change instant by < one poll, so every interval is within it.
            CHECK(c.interval.min + poll >= 10000);
            CHECK(c.interval.max <= 10000 + poll);
            CHECK(std::fabs(c.meanInBandUs - 10000.0) < static_cast<double>(poll));
            CHECK(it::judgeCadence(w, c.interval) == it::CadenceVerdict::Holds);
        }
    }
}

TEST_CASE("IMU1-B (2c.1): runCadence stamps each change with the instant read BEFORE the device, and counts what it cannot keep") {
    // A 10 ms copy at phase 3400, polled every 7 us from 0: the first poll at or after a copy is the
    // change, and its instant is the clock read before that poll — 3402 for the first copy, never
    // the 3409 a clock read after the device would give.
    HeldImu imu;
    SimClock clock{0};
    const auto read = [&] {
        const double v = imu.rotation(clock.t);
        clock.t += 7;
        return v;
    };
    it::ChangeWatch w;
    std::vector<std::uint64_t> at(5, 0U);  // room for 5 of the ~10 changes in 100 ms
    const it::CadenceRun run = it::runCadence(clock, read, 100000, w, std::span<std::uint64_t>(at));
    CHECK(run.stored == 5);
    CHECK(run.overflow == w.changes() - 5);
    CHECK(w.changes() == 10);  // copies at 3400, 13400, ..., 93400
    for (std::size_t k = 0; k < 5; ++k) {
        const std::uint64_t copy = 3400 + 10000 * k;
        const std::uint64_t firstPoll = (copy + 6) / 7 * 7;  // the first multiple of 7 at or after it
        CAPTURE(k);
        CHECK(at[k] == firstPoll);
    }
    // The loop ran for its duration and no longer: its last poll began before 100 000 us.
    CHECK(clock.t >= 100000);
    CHECK(clock.t < 100000 + 7 + 7);
}

TEST_CASE("IMU1-B (2c.1): a 5 ms copy reads FASTER, a 20 ms one SLOWER, a still robot NO CHANGE") {
    HeldImu fast;
    fast.periodUs = 5000.0;
    it::ChangeWatch w1;
    const it::Cadence cf = cadenceOf(cadenceInstants(fast, 0, 2000000, 7, w1));
    CHECK(cf.interval.median <= 5007);
    CHECK(cf.below == cf.interval.n);
    CHECK(it::judgeCadence(w1, cf.interval) == it::CadenceVerdict::Faster);

    HeldImu slow;
    slow.periodUs = 20000.0;
    it::ChangeWatch w2;
    const it::Cadence cs = cadenceOf(cadenceInstants(slow, 0, 2000000, 7, w2));
    CHECK(cs.above == cs.interval.n);
    CHECK(it::judgeCadence(w2, cs.interval) == it::CadenceVerdict::Slower);

    HeldImu still;
    still.rateDegPerS = 0.0;  // every copy writes the same value: no change is ever seen
    it::ChangeWatch w3;
    const auto none = cadenceInstants(still, 0, 2000000, 7, w3);
    CHECK(none.empty());
    CHECK(w3.changes() == 0);
    CHECK(w3.badReads() == 0);
    CHECK(it::judgeCadence(w3, cadenceOf(none).interval) == it::CadenceVerdict::NoChange);
}

TEST_CASE("IMU1-B (2c.1): a robot held still for part of the loop — the MEDIAN still finds the copy") {
    // 1.2 s turning, 0.8 s still: the still stretch is ONE long interval, so the median holds where a
    // mean of every interval would not.
    std::vector<std::uint64_t> at;
    for (int k = 0; k <= 120; ++k) at.push_back(1000U + static_cast<std::uint64_t>(k) * 10000U);
    at.push_back(at.back() + 800000U);
    const it::Cadence c = cadenceOf(at);
    CHECK(c.interval.n == 121);
    CHECK(c.inBand == 120);
    CHECK(c.above == 1);
    CHECK(c.meanInBandUs == 10000.0);  // the long interval is NOT in the mean
    CHECK(c.interval.median == 10000);
    CHECK(it::judgeCadence(watchWithChanges(122), c.interval) == it::CadenceVerdict::Holds);
}

TEST_CASE("IMU1-B (2c.1): the band's ends are IN the band, one microsecond past them is not") {
    const std::vector<std::uint64_t> at = {0, 9000, 20000, 28999, 40000};  // 9000, 11000, 8999, 11001
    const it::Cadence c = cadenceOf(at);
    REQUIRE(c.interval.n == 4);
    CHECK(c.inBand == 2);  // 9000 and 11000
    CHECK(c.below == 1);   // 8999
    CHECK(c.above == 1);   // 11001
    CHECK(c.meanInBandUs == 10000.0);
}

TEST_CASE("IMU1-B (2c.1): HA-213's verdict at every edge — the reads, the count, and the median's band") {
    const it::ChangeWatch w = watchWithChanges(21);
    it::Summary s;
    CHECK(it::judgeCadence(it::ChangeWatch{}, s) == it::CadenceVerdict::NoReads);  // nothing read at all
    CHECK(it::judgeCadence(watchWithChanges(0), s) == it::CadenceVerdict::NoChange);
    // ONE change is no interval: "too few", never "no change seen" under "changes 1".
    CHECK(it::judgeCadence(watchWithChanges(1), s) == it::CadenceVerdict::TooFew);
    s.n = 1;
    s.median = 10000;
    CHECK(it::judgeCadence(watchWithChanges(2), s) == it::CadenceVerdict::TooFew);
    s.n = it::kMinIntervals - 1;
    CHECK(it::judgeCadence(w, s) == it::CadenceVerdict::TooFew);
    s.n = it::kMinIntervals;
    CHECK(it::judgeCadence(w, s) == it::CadenceVerdict::Holds);
    s.median = 9000;
    CHECK(it::judgeCadence(w, s) == it::CadenceVerdict::Holds);
    s.median = 8999;
    CHECK(it::judgeCadence(w, s) == it::CadenceVerdict::Faster);
    s.median = 11000;
    CHECK(it::judgeCadence(w, s) == it::CadenceVerdict::Holds);
    s.median = 11001;
    CHECK(it::judgeCadence(w, s) == it::CadenceVerdict::Slower);
}

TEST_CASE("IMU1-B (2c.1): an IMU that answers PROS_ERR on EVERY read is NO READS — never 'no change', never a finding") {
    // A cable worked loose after STAGE 2: 400 000 reads, every one PROS_ERR_F. "No change" would send
    // the reader to the busy-read hypothesis about PROS's copy; this is a dead device.
    const double kErr = std::numeric_limits<double>::infinity();
    SimClock clock{0};
    it::ChangeWatch w;
    std::vector<std::uint64_t> at(8, 0U);
    const auto deadRead = [&] {
        clock.t += 5;
        return kErr;
    };
    const it::CadenceRun run = it::runCadence(clock, deadRead, 2000000, w, std::span<std::uint64_t>(at));
    CHECK(w.polls() == 400000);
    CHECK(w.badReads() == w.polls());
    CHECK(run.stored == 0);
    CHECK(it::judgeCadence(w, cadenceOf({}).interval) == it::CadenceVerdict::NoReads);
    // One good read among the errors, and no change: a still robot with a flaky cable is NO CHANGE
    // (the stage then prints the PROS_ERR count beside it and says the run is no finding).
    it::ChangeWatch mixed;
    (void)mixed.feed(kErr);
    (void)mixed.feed(5.0);
    (void)mixed.feed(kErr);
    CHECK(it::judgeCadence(mixed, cadenceOf({}).interval) == it::CadenceVerdict::NoChange);
}

TEST_CASE("IMU1-B (2c.1): fewer than two instants is no interval; a short scratch keeps what fits") {
    CHECK(cadenceOf({}).interval.n == 0);
    CHECK(cadenceOf({5}).interval.n == 0);
    const std::vector<std::uint64_t> at = {0, 10000, 20000, 25000, 45000};
    std::vector<std::uint32_t> scratch(2, 0U);  // room for two of the four intervals
    const it::Cadence c =
        it::cadenceFrom(std::span<const std::uint64_t>(at), std::span<std::uint32_t>(scratch));
    CHECK(c.interval.n == 2);
    CHECK(c.inBand == 2);  // only the first two (10000, 10000) were judged
    CHECK(c.below == 0);
    // An instant that went backwards is a 0 interval (below the band), never a wrapped huge one.
    const it::Cadence back = cadenceOf({100, 50});
    CHECK(back.interval.max == 0);
    CHECK(back.below == 1);
}

// ═══ 2c.2 the phase (HA-214) ═══════════════════════════════════════════════════════════

TEST_CASE("IMU1-B unwrapNear: the step lands in (-P/2, +P/2] — +P/2 kept, -P/2 wrapped up") {
    CHECK(it::unwrapNear(5000, 0, 10000) == 5000);    // +P/2: kept
    CHECK(it::unwrapNear(5001, 0, 10000) == -4999);   // past it: wrapped down
    CHECK(it::unwrapNear(-5000, 0, 10000) == 5000);   // -P/2: wrapped up
    CHECK(it::unwrapNear(-4999, 0, 10000) == -4999);  // inside: kept
    CHECK(it::unwrapNear(100, 9900, 10000) == 10100); // a copy that drifted past the wake comes back 10 ms later
    CHECK(it::unwrapNear(9900, 100, 10000) == -100);
    CHECK(it::unwrapNear(37 + 7 * 10000, 30, 10000) == 37);  // whole periods away
    CHECK(it::unwrapNear(37 - 7 * 10000, 30, 10000) == 37);
    CHECK(it::unwrapNear(1234, 99, 0) == 1234);  // a zero period: unchanged, never a division by 0
}

TEST_CASE("IMU1-B (2c.2): a fixed 10 ms copy -> the offset IS the phase, drift 0, FIXED") {
    for (const double phase : {500.0, 3400.0, 9000.0}) {
        CAPTURE(phase);
        HeldImu imu;
        imu.phaseUs = phase;
        // Wakes on a 10 ms grid starting at 0: the copy lands `phase` after every wake.
        const auto ticks = phaseTicks(imu, 1000000, 100, 3);
        const it::Phase ph = phaseOf(ticks);
        CHECK(ph.ticks == 100);
        CHECK(ph.seen == 100);
        CHECK(ph.missed == 0);
        CHECK(ph.medianUs >= static_cast<std::uint32_t>(phase));
        CHECK(ph.medianUs < static_cast<std::uint32_t>(phase) + 3U);
        CHECK(ph.firstUs == static_cast<std::int32_t>(ph.medianUs));
        CHECK(ph.lastUs == static_cast<std::int32_t>(ph.medianUs));
        REQUIRE(ph.driftValid);
        CHECK(std::fabs(ph.driftUsPerS) < 1e-6);
        CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Fixed);
    }
}

TEST_CASE("IMU1-B (2c.2): a copy 1 us slower than the wake grid drifts 100 us/s — least squares, not endpoints") {
    HeldImu imu;
    imu.periodUs = 10001.0;  // the IMU's clock against the brain's: 100 ppm
    imu.phaseUs = 2000.0;
    // ONE late wake, the last: its offset reads 60 us early. Endpoints alone would make the drift
    // (2039 - 2000) / 0.99 s = ~39 us/s; the least-squares slope over all 100 ticks moves by ~4.
    std::vector<int> jitter(100, 0);
    jitter[99] = 60;
    const auto ticks = phaseTicks(imu, 0, 100, 1, jitter);
    const it::Phase ph = phaseOf(ticks);
    REQUIRE(ph.seen == 100);
    REQUIRE(ph.driftValid);
    CHECK(ph.firstUs == 2000);
    CHECK(ph.lastUs == 2039);
    // Each tick the copy lands 1 us later: +1 us per 10 ms = +100 us/s.
    CHECK(ph.driftUsPerS == doctest::Approx(100.0).epsilon(0.05));
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Drifting);
    CHECK(it::secondsToCrossPeriod(ph.driftUsPerS, it::kCopyPeriodUs)
          == doctest::Approx(100.0).epsilon(0.05));
}

TEST_CASE("IMU1-B (2c.2): a phase that drifts THROUGH the wake is unwrapped — one line, not a jump") {
    // 100 us per tick: from ~9000 us the copy passes the window's end (missed ticks), then the wake
    // itself, and comes back near 0. Unwrapped, the drift is +10 000 us/s throughout; read raw, the
    // jump back to ~0 would make it hugely negative.
    HeldImu imu;
    imu.periodUs = 10100.0;
    imu.phaseUs = 9000.0;
    const auto ticks = phaseTicks(imu, 0, 100, 1);
    const it::Phase ph = phaseOf(ticks);
    CHECK(ph.missed >= 4);   // the copy in [9500, 10000) of a wake: no change inside the window
    CHECK(ph.seen >= 80);
    REQUIRE(ph.driftValid);
    CHECK(ph.driftUsPerS == doctest::Approx(10000.0).epsilon(0.01));
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Drifting);
    // The raw first and last offsets are reported as measured, not unwrapped.
    CHECK(ph.firstUs >= 0);
    CHECK(ph.lastUs >= 0);
    CHECK(ph.lastUs < static_cast<std::int32_t>(it::kPhaseWindowUs));
}

TEST_CASE("IMU1-B (2c.2): a copy AT the wake — a late wake MISSES it (counted), an early one sees it, FIXED") {
    // The copy lands 5 us after the nominal wake; the wakes jitter 0, +8, -3, +12 us. A wake after
    // the copy reads it as its baseline, and the next copy is 10 ms on — past the window: MISSED.
    // A wake before it sees it at 5 or 8 us. Half the ticks missed is the signature of a copy at the
    // wake, and the phase is still fixed.
    HeldImu imu;
    imu.phaseUs = 5.0;
    const auto ticks = phaseTicks(imu, 0, 100, 1, {0, 8, -3, 12});
    const it::Phase ph = phaseOf(ticks);
    CHECK(ph.missed == 50);
    CHECK(ph.seen == 50);
    CHECK(ph.firstUs == 5);
    CHECK(ph.medianUs == 5);
    REQUIRE(ph.driftValid);
    CHECK(std::fabs(ph.driftUsPerS) < 1.0);
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Fixed);
}

TEST_CASE("IMU1-B (2c.2): missed ticks are counted, never guessed, and never enter the median or drift") {
    std::vector<it::TickPhase> ticks;
    for (int i = 0; i < 30; ++i) {
        // Every third tick missed; the seen ones sit at 4000 exactly.
        ticks.push_back(it::TickPhase{static_cast<std::uint64_t>(i) * 10000U, (i % 3 == 0) ? -1 : 4000});
    }
    const it::Phase ph = phaseOf(ticks);
    CHECK(ph.ticks == 30);
    CHECK(ph.missed == 10);
    CHECK(ph.seen == 20);
    CHECK(ph.medianUs == 4000);
    CHECK(ph.firstUs == 4000);
    REQUIRE(ph.driftValid);
    CHECK(ph.driftUsPerS == doctest::Approx(0.0));
    // The ALL-missed robot: nothing seen, no drift, and the verdict says so.
    std::vector<it::TickPhase> none(100);
    const it::Phase pn = phaseOf(none);
    CHECK(pn.seen == 0);
    CHECK(pn.missed == 100);
    CHECK_FALSE(pn.driftValid);
    CHECK(it::judgePhase(pn, it::CadenceVerdict::Holds) == it::PhaseVerdict::NoChange);
}

TEST_CASE("IMU1-B (2c.2): no drift from one tick, or from ticks at one instant") {
    const std::vector<it::TickPhase> one = {{5000, 300}};
    CHECK_FALSE(phaseOf(one).driftValid);
    const std::vector<it::TickPhase> sameInstant = {{5000, 300}, {5000, 400}, {5000, 500}};
    CHECK_FALSE(phaseOf(sameInstant).driftValid);
}

TEST_CASE("IMU1-B (2c.2): HA-214's verdict at every edge — the cadence first, then the count, then 10 us/s") {
    it::Phase ph;
    ph.seen = it::kMinPhaseTicks;
    ph.driftValid = true;
    ph.seValid = true;
    ph.driftSeUsPerS = 0.0;  // a jitter-free line: only the 10 us/s threshold decides here
    ph.driftUsPerS = 0.0;
    // A phase unwrapped at 10 ms means nothing unless 2c.1 found the 10 ms copy.
    for (const auto c : {it::CadenceVerdict::NoChange, it::CadenceVerdict::TooFew,
                         it::CadenceVerdict::Faster, it::CadenceVerdict::Slower}) {
        CHECK(it::judgePhase(ph, c) == it::PhaseVerdict::NoCadence);
    }
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Fixed);
    ph.driftUsPerS = it::kFixedDriftUsPerS;  // exactly 10.0: still fixed
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Fixed);
    ph.driftUsPerS = -it::kFixedDriftUsPerS;
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Fixed);
    ph.driftUsPerS = std::nextafter(it::kFixedDriftUsPerS, 100.0);
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Drifting);
    ph.driftUsPerS = -std::nextafter(it::kFixedDriftUsPerS, 100.0);
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Drifting);
    ph.driftUsPerS = 0.0;
    ph.seen = it::kMinPhaseTicks - 1;
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::TooFew);
    ph.seen = it::kMinPhaseTicks;
    ph.driftValid = false;
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::TooFew);
    ph.driftValid = true;
    ph.seValid = false;  // no standard error: no verdict
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::TooFew);
    ph.seValid = true;
    ph.seen = 0;
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::NoChange);
    CHECK(it::secondsToCrossPeriod(10.0, 10000) == 1000.0);
    CHECK(it::secondsToCrossPeriod(-10.0, 10000) == 1000.0);
    CHECK(it::secondsToCrossPeriod(0.0, 10000) == 0.0);
}

TEST_CASE("IMU1-B (2c.2): the jitter's edge — DRIFTING needs |drift| > 3 SE, FIXED needs 3 SE <= 10 us/s, each edge exact") {
    it::Phase ph;
    ph.seen = it::kMinPhaseTicks;
    ph.driftValid = true;
    ph.seValid = true;
    // DRIFTING's noise edge: 3 x 10.0 == 30.0 exactly in doubles, so a drift of exactly 30 with an SE
    // of 10 is NOT past the noise (no verdict); one ulp more is.
    ph.driftSeUsPerS = 10.0;
    REQUIRE(it::kDriftSigmas * ph.driftSeUsPerS == 30.0);
    ph.driftUsPerS = 30.0;
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::NotSeparable);
    ph.driftUsPerS = std::nextafter(30.0, 100.0);
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Drifting);
    ph.driftUsPerS = -std::nextafter(30.0, 100.0);
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Drifting);
    // A drift past 10 us/s but inside the noise is NOT a drift — the review's false DRIFTING.
    ph.driftUsPerS = 25.0;
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::NotSeparable);
    // FIXED's noise edge: 3 x (10.0 / 3.0) == 10.0 exactly in doubles (REQUIRE'd), so that SE still
    // reads FIXED and the next double up does not.
    ph.driftUsPerS = 0.0;
    ph.driftSeUsPerS = 10.0 / 3.0;
    REQUIRE(it::kDriftSigmas * ph.driftSeUsPerS == it::kFixedDriftUsPerS);
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Fixed);
    ph.driftSeUsPerS = std::nextafter(10.0 / 3.0, 100.0);
    REQUIRE(it::kDriftSigmas * ph.driftSeUsPerS > it::kFixedDriftUsPerS);
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::NotSeparable);
    // And the 10 us/s edge itself with that noise: exactly 10 is not past it — no verdict either way.
    ph.driftUsPerS = it::kFixedDriftUsPerS;
    CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::NotSeparable);
}

namespace {

/// A deterministic 64-bit LCG and Box–Muller: the same draws on every run of the suite.
struct Lcg {
    std::uint64_t s;
    double uniform() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return (static_cast<double>(s >> 11) + 0.5) * (1.0 / 9007199254740992.0);
    }
    double gauss() {
        const double a = uniform();
        const double b = uniform();
        return std::sqrt(-2.0 * std::log(a)) * std::cos(2.0 * 3.14159265358979323846 * b);
    }
};

/// 2c.2 through the REAL loop with LATE wakes drawn as |N(0, sigma)| whole microseconds (a wake is
/// never early — delay_until), a copy at phase 3400 drifting `driftUsPerS` against the wake grid,
/// reads costing 7 us.
it::Phase jitteredPhase(std::uint64_t seed, double sigmaUs, double driftUsPerS) {
    HeldImu imu;
    imu.periodUs = 10000.0 + driftUsPerS * 0.01;  // per 10 ms tick: drift x 0.01 s
    Lcg g{seed * 0x9E3779B97F4A7C15ULL};
    std::vector<int> jitter(100);
    for (int& j : jitter) j = static_cast<int>(std::llround(std::fabs(g.gauss()) * sigmaUs));
    return phaseOf(phaseTicks(imu, 1000000, 100, 7, jitter));
}

}  // namespace

TEST_CASE("IMU1-B (2c.2): a FIXED phase seen through jittery wakes is never called DRIFTING (the review's probe)") {
    // Each offset is measured from the ACTUAL wake, so a late wake shortens it one for one. The first
    // build judged |drift| against 10 us/s alone: at a wake jitter of 50 us a fixed phase read
    // DRIFTING in about a third of runs. Now the drift must also clear 3 standard errors.
    for (const double sigma : {20.0, 30.0, 50.0}) {
        CAPTURE(sigma);
        int drifting = 0;
        int pastThresholdAlone = 0;  // what the first build's rule would have called DRIFTING
        int notSeparable = 0;
        for (std::uint64_t seed = 1; seed <= 100; ++seed) {
            const it::Phase ph = jitteredPhase(seed, sigma, 0.0);
            REQUIRE(ph.seValid);
            const it::PhaseVerdict v = it::judgePhase(ph, it::CadenceVerdict::Holds);
            drifting += v == it::PhaseVerdict::Drifting ? 1 : 0;
            notSeparable += v == it::PhaseVerdict::NotSeparable ? 1 : 0;
            pastThresholdAlone += std::fabs(ph.driftUsPerS) > it::kFixedDriftUsPerS ? 1 : 0;
            // The spread is printed so a reader sees the jitter: it covers the late wakes.
            CHECK(ph.maxUs >= ph.minUs);
            CHECK(ph.residualSdUs > 0.0);
        }
        // 3 SE: ~0.3 % of runs by chance — at most 1 in these 100 deterministic draws.
        CHECK(drifting <= 1);
        if (sigma == 50.0) {
            CHECK(pastThresholdAlone >= 30);  // the first build's rule fails here; this test sees it
            CHECK(notSeparable >= 95);        // and the honest answer is "no verdict"
        }
    }
    // Jitter of a few microseconds — the header's original belief — still reads FIXED, every run.
    for (const double sigma : {2.0, 5.0}) {
        CAPTURE(sigma);
        for (std::uint64_t seed = 1; seed <= 100; ++seed) {
            CHECK(it::judgePhase(jitteredPhase(seed, sigma, 0.0), it::CadenceVerdict::Holds)
                  == it::PhaseVerdict::Fixed);
        }
    }
}

TEST_CASE("IMU1-B (2c.2): a REAL drift through the same jitter still reads DRIFTING, every run") {
    // 100 us/s (a 100 ppm clock against the brain's) under 20 us of wake jitter: the slope's SE is
    // ~4 us/s, far under the drift.
    for (std::uint64_t seed = 1; seed <= 100; ++seed) {
        const it::Phase ph = jitteredPhase(seed, 20.0, 100.0);
        CAPTURE(seed);
        CHECK(ph.driftUsPerS == doctest::Approx(100.0).epsilon(0.15));
        CHECK(it::judgePhase(ph, it::CadenceVerdict::Holds) == it::PhaseVerdict::Drifting);
    }
}

TEST_CASE("IMU1-B (2c.2): the residual SD and the slope's SE are the textbook ones, on a hand-made set") {
    // Five ticks 10 ms apart, offsets 100, 104, 100, 104, 100 (no wrap): the slope is 0, the
    // residuals about the flat line +-2.4/1.6 — Syy = 3 x 1.6^2 + 2 x 2.4^2 = 19.2; s^2 = 19.2 / 3 = 6.4;
    // Sxx = (0.02^2 + 0.01^2) x 2 = 0.001 s^2; SE = sqrt(6.4 / 0.001) = 80 us/s.
    std::vector<it::TickPhase> ticks;
    const std::int32_t offs[] = {100, 104, 100, 104, 100};
    for (int i = 0; i < 5; ++i) ticks.push_back(it::TickPhase{static_cast<std::uint64_t>(i) * 10000U, offs[i]});
    const it::Phase ph = phaseOf(ticks);
    REQUIRE(ph.seValid);
    CHECK(ph.driftUsPerS == doctest::Approx(0.0));
    CHECK(ph.residualSdUs == doctest::Approx(std::sqrt(6.4)));
    CHECK(ph.driftSeUsPerS == doctest::Approx(80.0));
    CHECK(ph.minUs == 100);
    CHECK(ph.maxUs == 104);
    // Two seen ticks: a slope, but no standard error (n - 2 = 0) — and so no verdict.
    const std::vector<it::TickPhase> two = {{0, 100}, {10000, 110}};
    const it::Phase p2 = phaseOf(two);
    CHECK(p2.driftValid);
    CHECK_FALSE(p2.seValid);
    // A perfectly straight line: the residual is 0, never a rounding's negative under the root.
    std::vector<it::TickPhase> line;
    for (int i = 0; i < 50; ++i) line.push_back(it::TickPhase{static_cast<std::uint64_t>(i) * 10000U, 1000 + i});
    const it::Phase pl = phaseOf(line);
    CHECK(pl.driftUsPerS == doctest::Approx(100.0));
    CHECK(std::isfinite(pl.residualSdUs));
    CHECK(pl.residualSdUs < 1e-6);
}

TEST_CASE("IMU1-B (2c.2): a scratch smaller than the seen ticks keeps the first that fit — never a write past it") {
    std::vector<it::TickPhase> ticks;
    for (int i = 0; i < 6; ++i) ticks.push_back(it::TickPhase{static_cast<std::uint64_t>(i) * 10000U, 700 - i * 100});
    // Room for 3 handed over, inside a buffer of 8 whose last five are sentinels: a phase that wrote
    // past its span would change one (and stay inside this allocation, so the test fails cleanly).
    std::vector<std::uint32_t> scratch(8, 0xDEADBEEF);
    const it::Phase ph = it::phaseFrom(std::span<const it::TickPhase>(ticks), it::kCopyPeriodUs,
                                       std::span<std::uint32_t>(scratch).first(3));
    for (std::size_t k = 3; k < scratch.size(); ++k) {
        CAPTURE(k);
        CHECK(scratch[k] == 0xDEADBEEF);
    }
    CHECK(ph.seen == 6);           // every tick still counts for the drift
    CHECK(ph.minUs == 500);        // the kept three: 700, 600, 500
    CHECK(ph.maxUs == 700);
    CHECK(ph.medianUs == 600);
    CHECK(ph.lastUs == 200);       // the last seen tick, kept or not
}

// ═══ 2c.3 the tick span's stand-in (HA-215) ════════════════════════════════════════════

TEST_CASE("IMU1-B (2c.3): HA-215 is judged on the LONGEST span, at every edge") {
    CHECK(it::judgeSpan(summaryOf({})) == it::SpanVerdict::NoTicks);
    CHECK(it::judgeSpan(summaryOf({300, 2499})) == it::SpanVerdict::WellUnder);
    CHECK(it::judgeSpan(summaryOf({300, 2500})) == it::SpanVerdict::Close);
    CHECK(it::judgeSpan(summaryOf({300, 4999})) == it::SpanVerdict::Close);
    CHECK(it::judgeSpan(summaryOf({300, 5000})) == it::SpanVerdict::Over);
    // One long tick among 99 short ones: the hold fails on THAT tick, so the verdict is Over even
    // though the median is a tenth of the hold.
    std::vector<std::uint32_t> spans(99, 400);
    spans.push_back(5200);
    const it::Summary s = summaryOf(spans);
    CHECK(s.median == 400);
    CHECK(it::judgeSpan(s) == it::SpanVerdict::Over);
}

TEST_CASE("IMU1-B (2c.3): runSpans times FIRST IMU read to the instant of the SECOND — the body inside, the second read outside") {
    // Each tick: the wake on a 10 ms grid, each IMU read costing 40 us, the body 300 + i us. The span
    // starts at the clock read before the first IMU read and ends at the clock read before the
    // second: exactly 40 + 300 + i. A span ending after the second read would be 380 + i; one
    // measured backwards would be 0.
    SimClock clock{0};
    std::size_t wakes = 0;
    const auto wait = [&] {
        const std::uint64_t wake = 10000U * (wakes++ + 1);
        REQUIRE(wake >= clock.t);
        clock.t = wake;
    };
    int imuReads = 0;
    const auto readImu = [&] {
        ++imuReads;
        const double v = static_cast<double>(clock.t);  // the read's value: its own instant
        clock.t += 40;
        return v;
    };
    std::vector<std::size_t> bodyTicks;
    std::vector<double> bodyFirst;
    const auto body = [&](std::size_t tick, double first) {
        bodyTicks.push_back(tick);
        bodyFirst.push_back(first);
        clock.t += 300 + tick;
    };
    std::vector<std::uint32_t> spans(5, 0U);
    it::runSpans(clock, wait, readImu, body, std::span<std::uint32_t>(spans));
    CHECK(wakes == 5);
    CHECK(imuReads == 10);  // two IMU reads per tick
    for (std::size_t i = 0; i < 5; ++i) {
        CAPTURE(i);
        CHECK(spans[i] == 340 + i);
        CHECK(bodyTicks[i] == i);
        CHECK(bodyFirst[i] == static_cast<double>(10000U * (i + 1)));  // the FIRST read's value, at the wake
    }
}

// ═══ 2c.4 the gyro beside the rotation (HA-04, HA-109) ═════════════════════════════════

TEST_CASE("IMU1-B (2c.4): z in deg/s with get_rotation()'s sign, a 3 s turn -> HA-04 and HA-109 AGREE") {
    for (const double rate : {90.0, -90.0, 25.0}) {
        for (const double phase : {0.0, 6123.0}) {
            CAPTURE(rate);
            CAPTURE(phase);
            HeldImu imu;
            imu.rateDegPerS = rate;
            imu.phaseUs = phase;
            const it::GyroTurn g = gyroTurn(imu, 500000, 3000000, 7);
            CHECK(g.changes() >= 298);
            CHECK(g.windowUs() >= 2979000);
            // Both angles start and end on a copy: they agree to the poll's few microseconds.
            CHECK(g.integratedZDeg() == doctest::Approx(g.rotationChangeDeg()).epsilon(1e-4));
            CHECK(g.rotationChangeDeg() == doctest::Approx(rate * 2.99).epsilon(0.01));
            CHECK(it::judgeGyro(g) == it::GyroVerdict::Agree);
        }
    }
}

TEST_CASE("IMU1-B (2c.4): z with the OPPOSITE sign -> HA-04 WRONG, HA-109's scale holds") {
    HeldImu imu;
    imu.zScale = -1.0;
    const it::GyroTurn g = gyroTurn(imu, 0, 3000000, 7);
    CHECK(g.integratedZDeg() == doctest::Approx(-g.rotationChangeDeg()).epsilon(1e-4));
    CHECK(it::judgeGyro(g) == it::GyroVerdict::OppositeSign);
}

TEST_CASE("IMU1-B (2c.4): z in RADIANS per second -> HA-109 WRONG, the sign still judged; z another axis -> HA-04 not judged") {
    HeldImu rad;
    rad.zScale = it::kRadPerDeg;
    const it::GyroTurn gr = gyroTurn(rad, 0, 3000000, 7);
    // rad/s about the yaw axis: HA-109 WRONG (units), and the SIGN still judged — a sign does not
    // depend on units (the IMU1-B review: the first build withheld it here too).
    CHECK(it::judgeGyro(gr) == it::GyroVerdict::RadiansSameSign);
    CHECK(it::looksLikeRadians(gr.rotationChangeDeg(), gr.integratedZDeg()));
    HeldImu radNeg;
    radNeg.zScale = -it::kRadPerDeg;
    CHECK(it::judgeGyro(gyroTurn(radNeg, 0, 3000000, 7)) == it::GyroVerdict::RadiansOppositeSign);

    HeldImu flat;
    flat.zScale = 0.0;  // the turn is about another of the IMU's axes
    const it::GyroTurn gf = gyroTurn(flat, 0, 3000000, 7);
    CHECK(gf.integratedZDeg() == 0.0);
    CHECK(it::judgeGyro(gf) == it::GyroVerdict::ScaleOff);  // another axis: HA-04 not judged
    CHECK_FALSE(it::looksLikeRadians(gf.rotationChangeDeg(), gf.integratedZDeg()));
    // A deg/s z that agrees is never mistaken for radians.
    CHECK_FALSE(it::looksLikeRadians(100.0, 100.0));
}

TEST_CASE("IMU1-B (2c.4): a still robot is NO TURN; a short turn TOO SHORT — never a verdict") {
    HeldImu still;
    still.rateDegPerS = 0.0;
    const it::GyroTurn gs = gyroTurn(still, 0, 3000000, 7);
    CHECK(gs.changes() == 0);
    CHECK(gs.windowUs() == 0);
    CHECK(gs.rotationChangeDeg() == 0.0);
    CHECK(gs.integratedZDeg() == 0.0);
    CHECK(it::judgeGyro(gs) == it::GyroVerdict::NoTurn);
    // Turning only for the first 1.5 s of the loop: the window is the turn's, under 2 s.
    HeldImu imu;
    it::GyroTurn g;
    for (std::uint64_t t = 0; t < 3000000; t += 7) {
        const std::uint64_t tt = t < 1500000 ? t : 1500000;  // the rotation freezes at 1.5 s
        g.feed(t, imu.rotation(tt), t < 1500000 ? imu.z(t) : 0.0);
    }
    CHECK(g.windowUs() < 1500000);
    CHECK(it::judgeGyro(g) == it::GyroVerdict::TooShort);
}

TEST_CASE("IMU1-B (2c.4): the integral is the TRAPEZOID between the FIRST and LAST change, exact for a ramp") {
    // z = a + b·t on irregular instants: the trapezoid is exact for a linear integrand, a rectangle
    // rule is not. A large z BEFORE the first change and AFTER the last must not count — the window is
    // the changes', not the loop's.
    const double a = 40.0;
    const double b = 30.0;  // deg/s per s
    const auto zAt = [&](std::uint64_t tUs) { return a + b * static_cast<double>(tUs) * 1e-6; };
    it::GyroTurn g;
    g.feed(0, 0.0, 5000.0);       // the baseline; z here is garbage
    g.feed(100000, 0.0, 5000.0);  // still no change
    // The first change at 0.2 s; polls at irregular steps after it.
    std::uint64_t t = 200000;
    double rot = 1.0;
    g.feed(t, rot, zAt(t));
    std::uint64_t step = 997;
    std::uint64_t lastChangeT = t;
    while (t < 2400000) {
        t += step;
        step = step == 997 ? 13 : 997;
        rot += 0.001;  // every poll a change here: a model of a fast copy, the window is the changes'
        g.feed(t, rot, zAt(t));
        lastChangeT = t;
    }
    const double z0 = a * 0.2 + 0.5 * b * 0.2 * 0.2;
    const double t1 = static_cast<double>(lastChangeT) * 1e-6;
    const double z1 = a * t1 + 0.5 * b * t1 * t1;
    g.feed(lastChangeT + 1000, rot, -9000.0);  // after the last change: no change, garbage z
    CHECK(g.windowUs() == lastChangeT - 200000);
    CHECK(g.integratedZDeg() > 150.0);
    // Exact to rounding: a rectangle rule would be off by ~0.03 deg here, the garbage before the
    // first change by ~500, the garbage after the last by ~9.
    CHECK(std::fabs(g.integratedZDeg() - (z1 - z0)) < 1e-8);
}

TEST_CASE("IMU1-B (2c.4): a dropped poll is counted and bridged — a bad value or a clock gone backwards") {
    it::GyroTurn g;
    const double inf = std::numeric_limits<double>::infinity();
    g.feed(0, 0.0, 10.0);          // the baseline
    g.feed(1000000, 5.0, 10.0);    // the first change: 10 deg integrated so far, BEFORE the window
    g.feed(1500000, 6.0, inf);     // dropped: z is PROS_ERR_F
    g.feed(1600000, inf, 10.0);    // dropped: the rotation is
    g.feed(900000, 7.0, 1.0e9);    // dropped: an instant before the last good poll's — never bridges
    g.feed(2000000, 15.0, 10.0);   // bridged from 1.0 s: 10 deg/s x 1.0 s
    g.feed(2000000, 15.0, 30.0);   // the same microsecond: no area, no change
    g.feed(2500000, 20.0, 30.0);   // 0.5 s at (30 + 30) / 2: 15 deg
    CHECK(g.badReads() == 3);
    CHECK(g.polls() == 8);
    CHECK(g.changes() == 3);
    CHECK(g.windowUs() == 1500000);
    CHECK(g.rotationChangeDeg() == 15.0);
    CHECK(g.integratedZDeg() == doctest::Approx(25.0));
}

TEST_CASE("IMU1-B (2c.4): travel inside the window — rising minus falling IS the window's change") {
    it::GyroTurn g;
    const double rots[] = {0.0, 10.0, 50.0, 30.0, 80.0, 75.0};  // baseline, then five changes
    std::uint64_t t = 0;
    for (const double r : rots) {
        g.feed(t, r, 0.0);
        t += 10000;
    }
    // The step INTO the first change (0 -> 10) is before the window: not travel.
    CHECK(g.risingDeg() == 40.0 + 50.0);
    CHECK(g.fallingDeg() == 20.0 + 5.0);
    CHECK(g.risingDeg() - g.fallingDeg() == g.rotationChangeDeg());
    CHECK(g.rotationChangeDeg() == 65.0);
    CHECK(it::mixedTurn(g.risingDeg(), g.fallingDeg()));  // 25 > 0.2 x 90
    CHECK_FALSE(it::mixedTurn(100.0, 20.0));  // exactly a fifth: not mixed (STAGE 2's rule)
    CHECK(it::mixedTurn(100.0, 20.0001));
    CHECK_FALSE(it::mixedTurn(0.0, 0.0));
}

TEST_CASE("IMU1-B (2c.4): HA-04/HA-109's verdict at every edge — changes, window, turn, 10 percent") {
    using it::GyroVerdict;
    CHECK(it::judgeGyro(1, 3000000, 100.0, 100.0) == GyroVerdict::NoTurn);
    CHECK(it::judgeGyro(2, 1999999, 100.0, 100.0) == GyroVerdict::TooShort);
    CHECK(it::judgeGyro(2, 2000000, 100.0, 100.0) == GyroVerdict::Agree);
    CHECK(it::judgeGyro(2, 2000000, std::nextafter(30.0, 0.0), 30.0) == GyroVerdict::TooSmall);
    CHECK(it::judgeGyro(2, 2000000, 30.0, 30.0) == GyroVerdict::Agree);
    CHECK(it::judgeGyro(2, 2000000, -30.0, -30.0) == GyroVerdict::Agree);
    CHECK(it::judgeGyro(2, 2000000, std::numeric_limits<double>::quiet_NaN(), 30.0)
          == GyroVerdict::TooSmall);
    // The tolerance's ends land EXACTLY (0.10 x 100.0 == 10.0 in doubles) and are inside it.
    CHECK(it::judgeGyro(2, 2000000, 100.0, 110.0) == GyroVerdict::Agree);
    CHECK(it::judgeGyro(2, 2000000, 100.0, 90.0) == GyroVerdict::Agree);
    CHECK(it::judgeGyro(2, 2000000, 100.0, std::nextafter(110.0, 200.0)) == GyroVerdict::ScaleOff);
    CHECK(it::judgeGyro(2, 2000000, 100.0, std::nextafter(90.0, 0.0)) == GyroVerdict::ScaleOff);
    // The sign is judged only once the magnitudes agree.
    CHECK(it::judgeGyro(2, 2000000, 100.0, -110.0) == GyroVerdict::OppositeSign);
    CHECK(it::judgeGyro(2, 2000000, -100.0, 95.0) == GyroVerdict::OppositeSign);
    CHECK(it::judgeGyro(2, 2000000, -100.0, -95.0) == GyroVerdict::Agree);
    CHECK(it::judgeGyro(2, 2000000, 100.0, -50.0) == GyroVerdict::ScaleOff);
    CHECK(it::judgeGyro(2, 2000000, 100.0, std::numeric_limits<double>::infinity())
          == GyroVerdict::ScaleOff);
}

TEST_CASE("IMU1-B (2c.4): a z that CROSSES ZERO between polls — the trapezoid stays exact, and a both-ways turn is judged") {
    // z = -50 + 40 t deg/s (negative, then positive from t = 1.25 s), on uneven stamps: the trapezoid is
    // exact for a linear z whatever its sign, so the area through the crossing is not lost or doubled.
    const auto zAt = [](std::uint64_t tUs) { return -50.0 + 40.0 * static_cast<double>(tUs) * 1e-6; };
    const auto zInt = [](double tS) { return -50.0 * tS + 20.0 * tS * tS; };
    it::GyroTurn g;
    g.feed(0, 0.0, zAt(0));  // the baseline
    std::uint64_t t = 300000;
    double rot = 1.0;
    g.feed(t, rot, zAt(t));  // the first change
    std::uint64_t step = 991;
    while (t < 2700000) {
        t += step;
        step = step == 991 ? 17 : 991;
        rot += 0.001;
        g.feed(t, rot, zAt(t));
    }
    REQUIRE(zAt(300000) < 0.0);  // negative at the first change ...
    REQUIRE(zAt(t) > 0.0);       // ... positive at the last: the window spans the crossing
    const double expected = zInt(static_cast<double>(t) * 1e-6) - zInt(0.3);
    CHECK(std::fabs(g.integratedZDeg() - expected) < 1e-8);

    // A turn that goes one way, then back past its start: z changes sign; the model
    // copies every 10 ms and z IS the rotation's derivative, so the two angles agree — and the
    // verdict is given ("fine here: both angles saw the same motion").
    SimClock clock{0};
    // angle = 100 sin(pi t / 2) + 10 t: up ~110 deg by t ~ 1 s, down through 20 at 2 s, to -70 at 3 s.
    constexpr double kPi = 3.14159265358979323846;
    const auto angleAt = [](double tS) { return 100.0 * std::sin(kPi * tS / 2.0) + 10.0 * tS; };
    const auto rateAt = [](double tS) { return 50.0 * kPi * std::cos(kPi * tS / 2.0) + 10.0; };
    const auto copyAt = [](std::uint64_t tUs) { return static_cast<double>(tUs / 10000U) * 0.01; };
    const auto rotation = [&] { return angleAt(copyAt(clock.t)); };
    const auto z = [&] {
        const double v = rateAt(static_cast<double>(clock.t) * 1e-6);
        clock.t += 5;
        return v;
    };
    it::GyroTurn both;
    it::runGyro(clock, rotation, z, 3000000, both);
    CHECK(both.risingDeg() > 0.0);
    CHECK(both.fallingDeg() > 0.0);
    CHECK(it::mixedTurn(both.risingDeg(), both.fallingDeg()));
    CHECK(both.integratedZDeg() == doctest::Approx(both.rotationChangeDeg()).epsilon(0.01));
    CHECK(it::judgeGyro(both) == it::GyroVerdict::Agree);
}

TEST_CASE("IMU1-B (2c.4): the stage's judge takes the TURN, so its four numbers cannot be swapped; every poll dropped is NO READS") {
    // A turn of 40 deg whose z integrates to 400: swapped (rotation 400, z 40) it would read SCALE OFF
    // through the other door; as it is, the rotation's 40 deg is past the 30 deg minimum and the
    // magnitudes disagree. And 20 deg with z integrating to 200: TOO SMALL as it is; swapped, SCALE OFF.
    it::GyroTurn small;
    small.feed(0, 0.0, 100.0);
    small.feed(1000000, 5.0, 100.0);
    small.feed(3000000, 25.0, 100.0);  // window 2 s, rotation +20, z integrates to +200
    REQUIRE(small.windowUs() == 2000000);
    CHECK(small.rotationChangeDeg() == 20.0);
    CHECK(small.integratedZDeg() == doctest::Approx(200.0));
    CHECK(it::judgeGyro(small) == it::GyroVerdict::TooSmall);
    CHECK(it::judgeGyro(small.changes(), small.windowUs(), small.integratedZDeg(),
                        small.rotationChangeDeg())
          == it::GyroVerdict::ScaleOff);  // what a swap would print — the overload is what prevents it
    // Every poll PROS_ERR_F: NO READS, never NO TURN — a dead IMU is not "you did not turn it".
    it::GyroTurn dead;
    const double inf = std::numeric_limits<double>::infinity();
    for (std::uint64_t t = 0; t < 3000000; t += 1000) dead.feed(t, inf, inf);
    CHECK(dead.badReads() == dead.polls());
    CHECK(it::judgeGyro(dead) == it::GyroVerdict::NoReads);
    CHECK(it::judgeGyro(it::GyroTurn{}) == it::GyroVerdict::NoReads);  // nothing polled at all
    // ONE change on good polls: NO TURN (the stage prints the count beside it).
    it::GyroTurn once;
    once.feed(0, 0.0, 0.0);
    once.feed(1000, 1.0, 0.0);
    CHECK(once.changes() == 1);
    CHECK(it::judgeGyro(once) == it::GyroVerdict::NoTurn);
}

TEST_CASE("IMU1-B (2c.4): a NaN integral is SCALE OFF; radians' edges land EXACTLY and are inside; no turn is never radians") {
    using it::GyroVerdict;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // A NaN integral passes no magnitude check by accident: it is refused first.
    CHECK(it::judgeGyro(2, 2000000, 100.0, nan) == GyroVerdict::ScaleOff);
    CHECK_FALSE(it::looksLikeRadians(100.0, nan));
    CHECK_FALSE(it::looksLikeRadians(0.0, 0.0));
    CHECK_FALSE(it::looksLikeRadians(nan, 1.0));
    // A 45 deg turn: the tolerance's two ends, |z| = r*45 +- 0.10*r*45, land exactly in doubles
    // (REQUIRE'd), are IN (<=), and one ulp outward is out — through judgeGyro too.
    const double turn = 45.0;
    const double centre = it::kRadPerDeg * turn;
    const double tol = it::kGyroTolerance * it::kRadPerDeg * turn;
    const double hi = centre + tol;
    const double lo = centre - tol;
    REQUIRE(std::fabs(std::fabs(hi) - centre) == tol);
    REQUIRE(std::fabs(std::fabs(lo) - centre) == tol);
    CHECK(it::looksLikeRadians(turn, hi));
    CHECK(it::looksLikeRadians(turn, lo));
    CHECK(it::looksLikeRadians(-turn, -hi));
    CHECK_FALSE(it::looksLikeRadians(turn, std::nextafter(hi, 10.0)));
    CHECK_FALSE(it::looksLikeRadians(turn, std::nextafter(lo, 0.0)));
    CHECK(it::judgeGyro(2, 2000000, turn, hi) == GyroVerdict::RadiansSameSign);
    CHECK(it::judgeGyro(2, 2000000, turn, -lo) == GyroVerdict::RadiansOppositeSign);
    CHECK(it::judgeGyro(2, 2000000, -turn, -hi) == GyroVerdict::RadiansSameSign);
    CHECK(it::judgeGyro(2, 2000000, turn, std::nextafter(hi, 10.0)) == GyroVerdict::ScaleOff);
    // Halfway between the two scales is neither: SCALE OFF, the sign not judged.
    CHECK(it::judgeGyro(2, 2000000, turn, 0.5 * turn) == GyroVerdict::ScaleOff);
}

// ═══ source pins: the stage's own code, which the host cannot compile ════════════════════

namespace {

/// `text` with every run of whitespace made one space, so a pin can name a call whatever its line
/// breaks.
std::string squash(const std::string& text) {
    std::string out;
    bool space = false;
    for (const char c : text) {
        if (c == ' ' || c == '\n' || c == '\t' || c == '\r') {
            space = true;
            continue;
        }
        if (space && !out.empty()) out += ' ';
        space = false;
        out += c;
    }
    return out;
}

/// The STAGE 2c code in bench_r3a.cpp, comments stripped and whitespace squashed: the imu2c
/// namespace's body and imuTimingStage()'s.
std::string stageCode() {
    const std::string code = stripComments(readFile("src/programs/bench_r3a.cpp"));
    return squash(bodyOf(code, "namespace imu2c") + "\n" + bodyOf(code, "void imuTimingStage(std::uint8_t port) {"));
}

/// The squashed text of one `case <label>:` of a verdict switch, up to its `break;`.
std::string caseOf(const std::string& code, const std::string& label) {
    const std::size_t at = code.find("case " + label + ":");
    REQUIRE(at != std::string::npos);
    const std::size_t end = code.find("break;", at);
    REQUIRE(end != std::string::npos);
    return code.substr(at, end - at);
}

}  // namespace

TEST_CASE("IMU1-B pins: imu_timing.hpp is PROS-free and library-free (R10: no pacer, no Localizer)") {
    const std::string hdr = stripComments(readFile("src/programs/imu_timing.hpp"));
    CHECK(count(hdr, "pros/") == 0);
    CHECK(count(hdr, "pros::") == 0);
    CHECK(count(hdr, "\"shulib/") == 0);
    CHECK(count(hdr, "<shulib/") == 0);
    // Nor does the bench program gain a pacer or a Localizer through this stage.
    const std::string bench = stripComments(readFile("src/programs/bench_r3a.cpp"));
    CHECK(count(bench, "tick_pacer.hpp") == 0);
    CHECK(count(bench, "localization/") == 0);
    CHECK(count(bench, "#include \"programs/imu_timing.hpp\"") == 1);
}

TEST_CASE("IMU1-B pins: STAGE 2c reads only — no motor command, no device write, never yawRate() (R18)") {
    const std::string s = stageCode();
    REQUIRE(s.size() > 2000);  // the pin read the stage, not an empty match
    for (const char* banned :
         {"motor_move", "motor_brake", "motor_set_", "motor_tare", "setVoltage", "move_voltage",
          "move_velocity", "move_absolute", "move_relative", ".move(", ".brake(", "pros::Motor",
          "ProsMotor", "set_brake_mode", "adi_", "imu_reset", "imu_tare", "imu_set_", ".calibrate(",
          "imu_get_heading", "imu_get_yaw", "yawRate(", "ProsImu imu", "ProsTickPacer",
          "localization::", "#include"}) {
        const std::string shown(banned);
        CAPTURE(shown);
        CHECK(count(s, banned) == 0);
    }
    // ...and it reads exactly what it says, each read NAMED ONCE and handed to the loops: the raw
    // rotation, the raw gyro's z (never x or y — R18), the clock.
    CHECK(count(s, "imu_get_rotation") == 1);
    CHECK(count(s, "const auto rotation = [port] { return pros::c::imu_get_rotation(port); };") == 1);
    CHECK(count(s, "imu_get_gyro_rate") == 1);
    CHECK(count(s, "const auto gyroZ = [port] { return pros::c::imu_get_gyro_rate(port).z; };") == 1);
    CHECK(count(s, "pros::micros()") == 1);
    CHECK(count(s, "std::uint64_t nowUs() { return pros::micros(); }") == 1);
    for (const char* read : {"motor_get_position(raw)", "motor_get_actual_velocity(raw)",
                             "motor_get_current_draw(raw)", "motor_get_voltage(raw)",
                             "battery_get_voltage()"}) {
        const std::string shown(read);
        CAPTURE(shown);
        CHECK(count(s, read) == 1);
    }
    // Paced as STAGE 5 paces: delay_until on the motion tick's period, one pacer per paced part.
    CHECK(count(s, "pros::Task::delay_until(") == 2);
    CHECK(count(s, "[&phasePrev] { pros::Task::delay_until(&phasePrev, imu2c::kTickMs); }") == 1);
    CHECK(count(s, "[&spanPrev] { pros::Task::delay_until(&spanPrev, imu2c::kTickMs); }") == 1);
}

TEST_CASE("IMU1-B pins: STAGE 2c runs the header's loops with the robot's clock and reads, and judges what it collected") {
    // The loops are host-tested above on a simulated clock; on the robot they must get the real
    // clock, the right read, the right duration and the right storage — and each verdict the
    // numbers its own part collected (the IMU1-B review: these could change with the suite green).
    const std::string s = stageCode();
    for (const char* call :
         {"it::runCadence(imu2c::nowUs, rotation, imu2c::kCadenceUs, cadenceWatch, "
          "std::span<std::uint64_t>(imu2c::g_changeAt));",
          "it::runPhase(imu2c::nowUs, [&phasePrev]",
          "rotation, std::span<it::TickPhase>(imu2c::g_ticks));",
          "it::runSpans(imu2c::nowUs, [&spanPrev]",
          "imu2c::standInBody(tick, rot, motorReads, motorErrors); }, std::span<std::uint32_t>(imu2c::g_spans));",
          "it::runGyro(imu2c::nowUs, rotation, gyroZ, imu2c::kGyroUs, turn);",
          "std::span<const std::uint64_t>(imu2c::g_changeAt, cadenceRun.stored)",
          "const it::CadenceVerdict cadVerdict = it::judgeCadence(cadenceWatch, cad.interval);",
          "imu2c::reportCadence(cadenceWatch, cad, cadenceRun.overflow, cadVerdict);",
          "it::phaseFrom(std::span<const it::TickPhase>(imu2c::g_ticks), it::kCopyPeriodUs,",
          "imu2c::reportPhase(ph, it::judgePhase(ph, cadVerdict));",
          "it::summarize(std::span<std::uint32_t>(imu2c::g_spans));",
          "imu2c::reportSpan(spans, motorReads, motorErrors, it::judgeSpan(spans));",
          "imu2c::reportGyro(turn, it::judgeGyro(turn));"}) {
        const std::string shown(call);
        CAPTURE(shown);
        CHECK(count(s, call) == 1);
    }
    // One judge per part, and nowhere else: no second, hand-wired verdict.
    CHECK(count(s, "it::judgeCadence(") == 1);
    CHECK(count(s, "it::judgePhase(") == 1);
    CHECK(count(s, "it::judgeSpan(") == 1);
    CHECK(count(s, "it::judgeGyro(") == 1);
    // The loops run in the order the panel tells the operator: 2c.1, 2c.2, 2c.3, then 2c.4.
    CHECK(s.find("it::runCadence(") < s.find("it::runPhase("));
    CHECK(s.find("it::runPhase(") < s.find("it::runSpans("));
    CHECK(s.find("it::runSpans(") < s.find("it::runGyro("));
}

TEST_CASE("IMU1-B pins: each verdict prints ITS OWN words with ITS OWN severity chip") {
    // The verdict-to-text mapping lives in the stage's four switches; a swapped line or a WRONG shown
    // with a PASS chip would reverse what Station N tells the team. Each case: exactly these emitS
    // lines, in this order, and no other.
    struct Expect {
        const char* label;
        std::vector<const char*> lines;
    };
    const std::vector<Expect> table = {
        {"it::CadenceVerdict::NoReads", {"emitS(Sev::Warn, \"HA-213: every read was PROS_ERR -- no finding.\")"}},
        {"it::CadenceVerdict::NoChange", {"emitS(Sev::Warn, \"HA-213: NO CHANGE SEEN in 2 s"}},
        {"it::CadenceVerdict::TooFew", {"emitS(Sev::Warn, \"HA-213: %lu change(s) only"}},
        {"it::CadenceVerdict::Holds", {"emitS(Sev::Good, \"HA-213 HOLDS"}},
        {"it::CadenceVerdict::Faster", {"emitS(Sev::Bad, \"HA-213 WRONG, FASTER"}},
        {"it::CadenceVerdict::Slower", {"emitS(Sev::Warn, \"HA-213: median %lu us -- SLOWER"}},
        {"it::PhaseVerdict::NoCadence", {"emitS(Sev::Warn, \"HA-214: no verdict -- 2c.1"}},
        {"it::PhaseVerdict::NoChange", {"emitS(Sev::Warn, \"HA-214: no tick saw a change"}},
        {"it::PhaseVerdict::TooFew", {"emitS(Sev::Warn, \"HA-214: only %lu ticks"}},
        {"it::PhaseVerdict::Fixed", {"emitS(Sev::Good, \"HA-214: phase FIXED"}},
        {"it::PhaseVerdict::Drifting", {"emitS(Sev::Warn, \"HA-214: the phase DRIFTS"}},
        {"it::PhaseVerdict::NotSeparable", {"emitS(Sev::Warn, \"HA-214: no verdict -- drift"}},
        {"it::SpanVerdict::NoTicks", {"emitS(Sev::Warn, \"HA-215: nothing was measured."}},
        {"it::SpanVerdict::WellUnder", {"emitS(Sev::Good, \"HA-215: well under the 5 ms hold"}},
        {"it::SpanVerdict::Close", {"emitS(Sev::Warn, \"HA-215: longest stand-in tick"}},
        {"it::SpanVerdict::Over", {"emitS(Sev::Bad, \"HA-215 WRONG here"}},
        {"it::GyroVerdict::NoReads", {"emitS(Sev::Warn, \"HA-04/HA-109: every poll was PROS_ERR"}},
        {"it::GyroVerdict::NoTurn", {"emitS(Sev::Warn, \"HA-04/HA-109: %lu rotation change(s)"}},
        {"it::GyroVerdict::TooShort", {"emitS(Sev::Warn, \"HA-04/HA-109: turn spanned"}},
        {"it::GyroVerdict::TooSmall", {"emitS(Sev::Warn, \"HA-04/HA-109: turned only"}},
        {"it::GyroVerdict::Agree", {"emitS(Sev::Good, \"HA-04 HOLDS", "emitS(Sev::Good, \"HA-109 HOLDS"}},
        {"it::GyroVerdict::OppositeSign", {"emitS(Sev::Bad, \"HA-04 WRONG", "emitS(Sev::Good, \"HA-109's scale holds"}},
        {"it::GyroVerdict::RadiansSameSign", {"emitS(Sev::Bad, \"HA-109 WRONG", "emitS(Sev::Good, \"HA-04 HOLDS"}},
        {"it::GyroVerdict::RadiansOppositeSign", {"emitS(Sev::Bad, \"HA-109 WRONG", "emitS(Sev::Bad, \"HA-04 WRONG"}},
        {"it::GyroVerdict::ScaleOff", {"emitS(Sev::Bad, \"HA-109 WRONG", "emitS(Sev::Warn, \"HA-04: not judged"}},
    };
    const std::string s = stageCode();
    for (const Expect& e : table) {
        const std::string label(e.label);
        CAPTURE(label);
        const std::string body = caseOf(s, e.label);
        CHECK(count(body, "emitS(") == e.lines.size());
        std::size_t from = 0;
        for (const char* line : e.lines) {
            const std::string shown(line);
        CAPTURE(shown);
            const std::size_t at = body.find(line, from);
            CHECK(at != std::string::npos);
            if (at != std::string::npos) from = at + 1;
        }
    }
    // Every enumerator has its case (the ARM build's -Wswitch catches a missing one too).
    CHECK(count(s, "case it::CadenceVerdict::") == 6);
    CHECK(count(s, "case it::PhaseVerdict::") == 6);
    CHECK(count(s, "case it::SpanVerdict::") == 4);
    CHECK(count(s, "case it::GyroVerdict::") == 9);
    // NO CHANGE's "that is the finding" only on a run with no PROS_ERR read.
    const std::string noChange = caseOf(s, "it::CadenceVerdict::NoChange");
    CHECK(noChange.find("if (watch.badReads() == 0) {") < noChange.find("that is the finding"));
    CHECK(noChange.find("that is the finding") < noChange.find("} else {"));
}

TEST_CASE("IMU1-B pins: every number a result block prints is the field its words name, under the branch that prints it") {
    // The per-case pin above guards each verdict's emitS lines, not the data lines around them: the
    // verification round (2026-11-05) printed 1 SE under the words "(3 SE; ...)" and guarded the SE
    // and residual lines dead, each with the suite green (its V13, V14). Station N tells the team to
    // copy these numbers into its table, so every print of the four result blocks is pinned here
    // with its WHOLE argument list, in order, beside every branch head and derived local — and no
    // print, branch or early exit that is not listed can join them.
    struct Block {
        const char* signature;
        std::vector<const char*> items;
    };
    const std::vector<Block> blocks = {
        {"void reportCadence(const it::ChangeWatch& watch,",
         {
         "emitf(\" polls %lu, PROS_ERR %lu, changes %lu (raw deg, PROS's rotation)\", ul(watch.polls()), ul(watch.badReads()), ul(watch.changes()));",
         "if (overflow > 0) {",
         "emitf(\" (%lu more changes than the %lu kept: the first %lu judged)\", ul(overflow), ul(kMaxChanges), ul(kMaxChanges));",
         "emitf(\" change intervals: n %lu min %lu median %lu max %lu us\", ul(cad.interval.n), ul(cad.interval.min), ul(cad.interval.median), ul(cad.interval.max));",
         "emitf(\" in %lu..%lu us: %lu of %lu (below %lu, above %lu); their mean %.1f us\", ul(it::kCopyPeriodUs - it::kBandUs), ul(it::kCopyPeriodUs + it::kBandUs), ul(cad.inBand), ul(cad.interval.n), ul(cad.below), ul(cad.above), cad.meanInBandUs);",
         "emitS(Sev::Warn, \"HA-213: every read was PROS_ERR -- no finding.\");",
         "emitS(Sev::Warn, \"HA-213: NO CHANGE SEEN in 2 s -- turn it and re-run.\");",
         "if (watch.badReads() == 0) {",
         "} else {",
         "emitS(Sev::Warn, \"HA-213: %lu change(s) only -- turn steadily; re-run.\", ul(watch.changes()));",
         "emitf(\" (a verdict needs at least %lu intervals between changes)\", ul(it::kMinIntervals));",
         "emitS(Sev::Good, \"HA-213 HOLDS: a change every ~10 ms (median %lu us).\", ul(cad.interval.median));",
         "emitS(Sev::Bad, \"HA-213 WRONG, FASTER: median %lu us, not ~10000.\", ul(cad.interval.median));",
         "emitS(Sev::Warn, \"HA-213: median %lu us -- SLOWER than 10 ms, or\", ul(cad.interval.median));",
         }},
        {"void reportPhase(const it::Phase& ph, it::PhaseVerdict verdict) {",
         {
         "const double noise = it::kDriftSigmas * ph.driftSeUsPerS;",
         "emitf(\" copy's offset from the wake: first %ld median %lu last %ld us (%lu ticks)\", static_cast<long>(ph.firstUs), ul(ph.medianUs), static_cast<long>(ph.lastUs), ul(ph.seen));",
         "emitf(\" spread of those offsets: min %lu max %lu us\", ul(ph.minUs), ul(ph.maxUs));",
         "emitf(\" no change within %lu us: %lu of %lu ticks\", ul(it::kPhaseWindowUs), ul(ph.missed), ul(ph.ticks));",
         "if (ph.driftValid && ph.seValid) {",
         "emitf(\" drift %+.1f +- %.1f us/s (%.0f SE; least squares, unwrapped at %lu us)\", ph.driftUsPerS, noise, it::kDriftSigmas, ul(it::kCopyPeriodUs));",
         "emitf(\" residual SD %.1f us (the wakes' jitter, mostly)\", ph.residualSdUs);",
         "} else if (ph.driftValid) {",
         "emitf(\" drift %+.1f us/s (no SE: under three ticks saw a change)\", ph.driftUsPerS);",
         "} else {",
         "emitS(Sev::Warn, \"HA-214: no verdict -- 2c.1 did not find the 10 ms copy.\");",
         "emitS(Sev::Warn, \"HA-214: no tick saw a change -- turn it; re-run (or\");",
         "emitS(Sev::Warn, \"HA-214: only %lu ticks saw a change -- re-run.\", ul(ph.seen));",
         "emitS(Sev::Good, \"HA-214: phase FIXED -- the copy lands ~%lu us after\", ul(ph.medianUs));",
         "emitf(\" the wake (|drift| and %.0f SE <= %.0f us/s over 1 s).\", it::kDriftSigmas, it::kFixedDriftUsPerS);",
         "emitS(Sev::Warn, \"HA-214: the phase DRIFTS %+.1f us/s -- it crosses\", ph.driftUsPerS);",
         "emitf(\" a whole 10 ms tick every ~%.0f s: every boot passes the bad band.\", it::secondsToCrossPeriod(ph.driftUsPerS, it::kCopyPeriodUs));",
         "emitS(Sev::Warn, \"HA-214: no verdict -- drift %+.1f +- %.1f us/s:\", ph.driftUsPerS, noise);",
         "emitf(\" the wakes jitter too much to tell %.0f us/s from none\", it::kFixedDriftUsPerS);",
         }},
        {"void reportSpan(const it::Summary& spans,",
         {
         "emitf(\" each: 2 IMU reads, %lu drive motors x 4 reads, the battery, 1 serial line\", ul(kChassis.leftCount + kChassis.rightCount));",
         "emitf(\" span, first IMU read to second: min %lu median %lu max %lu us\", ul(spans.min), ul(spans.median), ul(spans.max));",
         "emitf(\" motor reads that returned PROS_ERR: %lu of %lu\", ul(motorErrors), ul(motorReads));",
         "emitS(Sev::Warn, \"HA-215: nothing was measured.\");",
         "emitS(Sev::Good, \"HA-215: well under the 5 ms hold (max %lu < %lu us);\", ul(spans.max), ul(it::kSpanWellUnderUs));",
         "emitS(Sev::Warn, \"HA-215: longest stand-in tick %lu us -- over half\", ul(spans.max));",
         "emitS(Sev::Bad, \"HA-215 WRONG here: the stand-in spanned %lu us\", ul(spans.max));",
         }},
        {"void reportGyro(const it::GyroTurn& turn, it::GyroVerdict verdict) {",
         {
         "const double d = turn.rotationChangeDeg();",
         "const double z = turn.integratedZDeg();",
         "const double ratio = d != 0.0 ? std::fabs(z / d) : 0.0;",
         "emitf(\" polls %lu, dropped %lu (PROS_ERR), rotation changes %lu\", ul(turn.polls()), ul(turn.badReads()), ul(turn.changes()));",
         "emitf(\" window, first to last change: %.3f s\", static_cast<double>(turn.windowUs()) * 1e-6);",
         "emitf(\" rotation change : %+9.2f deg (raw deg, PROS's rotation -- CW-positive per HA-02)\", d);",
         "emitf(\" gyro z integrated: %+9.2f deg (raw z, integrated -- trapezoid on micros())\", z);",
         "if (d != 0.0) {",
         "emitf(\" z / rotation: %+.4f travel in the window: rising %.1f, falling %.1f deg\", z / d, turn.risingDeg(), turn.fallingDeg());",
         "if (it::mixedTurn(turn.risingDeg(), turn.fallingDeg())) {",
         "emitS(Sev::Warn, \"HA-04/HA-109: every poll was PROS_ERR -- no finding.\");",
         "emitS(Sev::Warn, \"HA-04/HA-109: %lu rotation change(s) -- no turn seen;\", ul(turn.changes()));",
         "emitS(Sev::Warn, \"HA-04/HA-109: turn spanned %.2f s (need >= %.0f s) --\", static_cast<double>(turn.windowUs()) * 1e-6, static_cast<double>(it::kGyroMinWindowUs) * 1e-6);",
         "emitS(Sev::Warn, \"HA-04/HA-109: turned only %.1f deg (need >= %.0f) --\", std::fabs(d), it::kGyroMinTurnDeg);",
         "emitS(Sev::Good, \"HA-04 HOLDS: z has get_rotation()'s sign, so\");",
         "emitS(Sev::Good, \"HA-109 HOLDS: the two agree within 10 percent --\");",
         "emitS(Sev::Bad, \"HA-04 WRONG: z has the OPPOSITE sign to get_rotation()\");",
         "emitS(Sev::Good, \"HA-109's scale holds: within 10 percent.\");",
         "emitS(Sev::Bad, \"HA-109 WRONG: |z| / |rotation| is %.4f, not ~1:\", ratio);",
         "emitS(Sev::Good, \"HA-04 HOLDS: z has get_rotation()'s sign (a sign\");",
         "emitS(Sev::Bad, \"HA-109 WRONG: |z| / |rotation| is %.4f, not ~1:\", ratio);",
         "emitS(Sev::Bad, \"HA-04 WRONG: z has the OPPOSITE sign to get_rotation()\");",
         "emitS(Sev::Bad, \"HA-109 WRONG: |z| / |rotation| is %.4f, not ~1.\", ratio);",
         "emitS(Sev::Warn, \"HA-04: not judged -- z is not about the yaw axis.\");",
         }},
    };
    const std::string code = stripComments(readFile("src/programs/bench_r3a.cpp"));
    for (const Block& b : blocks) {
        const std::string signature(b.signature);
        CAPTURE(signature);
        const std::string body = squash(bodyOf(code, b.signature));
        REQUIRE(body.size() > 300);  // the pin read the function, not an empty match
        std::size_t prints = 0;
        std::size_t ifs = 0;
        std::size_t elses = 0;
        std::size_t locals = 0;
        std::size_t from = 0;
        for (const char* item : b.items) {
            const std::string shown(item);
            CAPTURE(shown);
            const std::size_t at = body.find(shown, from);
            CHECK(at != std::string::npos);
            if (at != std::string::npos) from = at + shown.size();
            if (shown.rfind("emitf(", 0) == 0 || shown.rfind("emitS(", 0) == 0) ++prints;
            if (shown.find("if (") != std::string::npos) ++ifs;
            if (shown.rfind("} else", 0) == 0) ++elses;
            if (shown.rfind("const double ", 0) == 0) ++locals;
        }
        // Nothing unlisted: every print, branch and derived local of the block is one of the above.
        CHECK(count(body, "emitf(") + count(body, "emitS(") == prints);
        CHECK(count(body, "if (") == ifs);
        CHECK(count(body, "else") == elses);
        CHECK(count(body, "const double ") == locals);
        CHECK(count(body, "?") == (locals == 3 ? 1U : 0U));  // reportGyro's ratio, guarded for d == 0
        // (as tokens: the words "returned" and "while a loop" appear in the printed text)
        for (const char* banned : {"return;", "return ", "for (", "while (", "do {", "goto ", "throw "}) {
            const std::string shown(banned);
            CAPTURE(shown);
            CHECK(count(body, banned) == 0);
        }
    }
}

TEST_CASE("IMU1-B pins: STAGE 2c prints only through SER1's one-call path") {
    const std::string s = stageCode();
    for (const char* banned : {"printf(", "puts(", "putchar(", "std::cout", "write(", "fwrite("}) {
        const std::string shown(banned);
        CAPTURE(shown);
        CHECK(count(s, banned) == 0);
    }
    // The stand-in's serial line: ONE serialLine() per stand-in tick, in its body.
    CHECK(count(s, "hal::pros::serialLine(") == 1);
    CHECK(count(bodyOf(stripComments(readFile("src/programs/bench_r3a.cpp")),
                       "void standInBody(std::size_t tick, double rot, std::size_t& motorReads, "
                       "std::size_t& motorErrors) {"),
                "hal::pros::serialLine(")
          == 1);
    CHECK(count(s, "emitS(") >= 10);
}

TEST_CASE("IMU1-B pins: reportImu runs STAGE 2c after STAGE 2's verdict, holds that verdict on the panel, and 7 RUN ALL still runs station 2") {
    const std::string code = stripComments(readFile("src/programs/bench_r3a.cpp"));
    const std::string report = bodyOf(code, "void reportImu(bool present)");
    REQUIRE(count(report, "imuTimingStage(kChassis.imuPort);") == 1);
    const std::size_t call = report.find("imuTimingStage(kChassis.imuPort);");
    // UNCONDITIONAL: the last statement of the try block, straight after STAGE 2's HA-02 verdict
    // (a guarded or dead call — the review's ARM-clean re-cut of MP3 — would keep the text above).
    CHECK(count(squash(report), "dominantCcw ? \"CCW\" : \"CW\"); } imuTimingStage(kChassis.imuPort); } "
                                "catch (const PreconditionError& e) {")
          == 1);
    // After the HA-02 verdict lines, and before the adapter-refused catch (so a refused IMU is
    // not timed).
    CHECK(report.find("HA-02 WRONG") < call);
    CHECK(report.find("HA-02 CONFIRMED") < call);
    CHECK(call < report.find("catch (const PreconditionError& e)"));
    // RUN ALL -> tImu -> reportImu: the stage runs there too, behind its START / SKIP.
    CHECK(count(bodyOf(code, "void tAll(pros::c::v5_device_e_t* f)"), "tImu(f);") == 1);
    CHECK(count(bodyOf(code, "void tImu(pros::c::v5_device_e_t* f)"), "reportImu(") == 1);
    const std::string stage = squash(bodyOf(code, "void imuTimingStage(std::uint8_t port) {"));
    CHECK(count(stage, "twoButtonPrompt(") == 1);
    CHECK(stage.find("twoButtonPrompt(") < stage.find("it::runCadence("));
    // SKIP: the scroll goes back on BEFORE its line, so under 7 RUN ALL every later station still
    // reaches the panel, and nothing is read after it (the verification round, 2026-11-05: a SKIP
    // that left the scroll off survived the suite — its V12). The stage's only return.
    CHECK(count(stage, "if (!start) { g_screenActive = true; drawHeader(\"STAGE 2c IMU TIMING\"); screenClear(); "
                       "emit(\"STAGE 2c SKIPPED by the operator -- nothing measured.\"); return; }")
          == 1);
    CHECK(count(stage, "return;") == 1);
    CHECK(stage.find("const bool start = twoButtonPrompt(") < stage.find("if (!start) {"));
    CHECK(stage.find("if (!start) {") < stage.find("it::runCadence("));
    // STAGE 2's results stay on the panel until a touch: the stage's FIRST act is that wait,
    // before its rule() and its prompt erase the panel (the resumed run's review, 2026-11-03). Two
    // waits in all -- that one and 2c.4's.
    CHECK(count(stage, "waitForTouch();") == 2);
    CHECK(stage.find("waitForTouch();") < stage.find("rule("));
    CHECK(stage.find("rule(") < stage.find("twoButtonPrompt("));
    // The rule and the intro go to serial + SD only — the prompt erases the screen, so on the panel
    // they would only flash (the IMU1-B review, 2026-11-05): the scroll is turned off FIRST.
    CHECK(count(stage, "waitForTouch(); g_screenActive = false; rule(\"STAGE 2c") == 1);
    const std::string intro = stage.substr(stage.find("rule("), stage.find("twoButtonPrompt(") - stage.find("rule("));
    CHECK(count(intro, "g_screenActive = true") == 0);
    // The results reach the panel: the scroll is back on after the last loop and before the first
    // report.
    const std::size_t lastLoop = stage.find("it::runGyro(");
    const std::size_t firstReport = stage.find("imu2c::reportCadence(");
    REQUIRE(lastLoop < firstReport);
    const std::string between = stage.substr(lastLoop, firstReport - lastLoop);
    CHECK(count(between, "g_screenActive = true;") == 1);
    CHECK(count(between, "g_screenActive = false;") == 0);
}
