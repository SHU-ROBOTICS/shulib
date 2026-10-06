// diag::TickTimingStats (chunk R5a Part A, A.5) — the statistics of a loop's dt that shulib
// Calibrate's GAINS and REST steps record, a sibling of LoopMonitor. Each case names the bug it
// catches and the mutation that must turn it red:
//   T1  over a jittered run (the hostile JitterSchedule, fixed seed) the mean and σ equal a two-pass
//       reference to 1e-12 relative, and min, max and both counts EXACTLY
//                                          (count the baseline tick as a dt of 0: red — count, min)
//   T2  the budget is INCLUSIVE — a tick of exactly 15 ms counts, one ulp under does not; "over 2×
//       nominal" is STRICT — exactly 20 ms does not count, one ulp over does
//                                          (budget judged with `>`: red; 2× judged with `>=`: red)
//   T3  the baseline: the first tick returns 0 and measures nothing; rebaseline() skips a deliberate
//       gap and keeps the statistics; reset() forgets them
//   T4  summary().valid() refuses what cannot hold together (a count past `count`, a non-finite
//       time, min > max, a nonzero time with nothing measured)
//   T5  the preconditions (a budget at or under nominal; a non-finite now)
// NOTHING HERE IS A ROBOT'S TIMING: the jitter schedule is the A3 hostile model's INVENTED one
// (A4 register HA-34).

#include "doctest.h"

#include <cmath>
#include <limits>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/diag/tick_timing_stats.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::TickTimingConfig;
using shulib::diag::TickTimingStats;
using shulib::diag::TickTimingSummary;
using shulib::sim::JitterSchedule;
using shulib::units::Time;

namespace {

/// A RELATIVE tolerance. doctest's Approx is relative only for values above 1: its tolerance is
/// epsilon × (scale + max(|a|, |b|)) with a scale of 1 by default, so Approx(0.02).epsilon(1e-6)
/// accepts ±1e-6 — 5e-5 relative — and Approx(0.052).epsilon(0.1) accepts anything within 0.1 of
/// 0.052, twice the target itself (found 2026-10-04: a mutation survived exactly that). Every pin in
/// this file means relative, so each goes through here. The target must be nonzero: at scale 0 an
/// exact 0 never compares equal.
[[nodiscard]] doctest::Approx rel(double value, double epsilon) {
    return doctest::Approx(value).epsilon(epsilon).scale(0.0);
}

/// Feed `times` and return the summary.
TickTimingSummary feed(const std::vector<double>& times, const TickTimingConfig& cfg = {}) {
    TickTimingStats s{cfg};
    for (double t : times) {
        (void)s.tick(Time{t});
    }
    return s.summary();
}

}  // namespace

// ═══ T1. A jittered run against a two-pass reference ════════════════════════════════
// Bug caught: the baseline tick counted as a dt of 0 (a mean pulled down by 1/N, a min of exactly
// 0), a variance by the naive Σx² − (Σx)²/n (cancellation at 10 ms), an off-by-one in either count.
TEST_CASE("TickTimingStats T1: a jittered run's mean and sigma equal a two-pass reference, and "
          "min, max and the counts match exactly") {
    JitterSchedule jitter{11};
    std::vector<double> times;
    double t = 3.0;  // not 0: a clock that started before the stretch being measured
    times.push_back(t);
    for (int i = 0; i < 5000; ++i) {
        t += jitter(i).value();
        times.push_back(t);
    }
    const TickTimingSummary s = feed(times);

    // The reference: the SAME differences the class takes (now − last on the cumulative times),
    // then a mean, then the squared deviations from it — two passes, no running sums.
    std::vector<double> dts;
    for (std::size_t i = 1; i < times.size(); ++i) {
        dts.push_back(times[i] - times[i - 1]);
    }
    double sum = 0.0;
    double mn = dts[0];
    double mx = dts[0];
    unsigned overBudget = 0;
    unsigned overDouble = 0;
    for (double d : dts) {
        sum += d;
        mn = std::min(mn, d);
        mx = std::max(mx, d);
        overBudget += d >= 0.015 ? 1U : 0U;
        overDouble += d > 0.02 ? 1U : 0U;
    }
    const double mean = sum / static_cast<double>(dts.size());
    double ss = 0.0;
    for (double d : dts) {
        ss += (d - mean) * (d - mean);
    }
    const double sigma = std::sqrt(ss / static_cast<double>(dts.size() - 1));

    CHECK(s.count == dts.size());
    CHECK(s.meanS == rel(mean, 1e-12));
    CHECK(s.sigmaS == rel(sigma, 1e-12));
    CHECK(s.minS == mn);  // exact
    CHECK(s.maxS == mx);  // exact
    CHECK(s.overBudget == overBudget);
    CHECK(s.overDouble == overDouble);
    // The schedule really exercises both counts and the spread (±20 % jitter, 2 % spikes at 5×).
    CHECK(overBudget > 0U);
    CHECK(overDouble > 0U);
    CHECK(overBudget < s.count);
    CHECK(mn > 0.0079);
    CHECK(s.valid());
}

// ═══ T2. The two boundaries ═════════════════════════════════════════════════════════
// Bug caught: the budget judged with `>` (a tick that consumed its whole budget passing as on
// time — LoopMonitor's header says why that is a miss), or the 2× count made inclusive.
TEST_CASE("TickTimingStats T2: exactly 15 ms is over budget (inclusive); exactly 20 ms is not "
          "over twice nominal (strict)") {
    SUBCASE("budget, inclusive") {
        CHECK(feed({0.0, 0.015}).overBudget == 1U);  // 0.015 − 0.0 is exactly 0.015
        CHECK(feed({0.0, std::nextafter(0.015, 0.0)}).overBudget == 0U);
    }
    SUBCASE("twice nominal, strict") {
        CHECK(feed({0.0, 0.02}).overDouble == 0U);
        CHECK(feed({0.0, std::nextafter(0.02, 1.0)}).overDouble == 1U);
        CHECK(feed({0.0, 0.02}).overBudget == 1U);  // …and a 20 ms tick is, of course, late
    }
}

// ═══ T3. Baseline, rebaseline, reset ═════════════════════════════════════════════════
// Bug caught: the first tick entering the statistics (a dt measured from 0, or from nothing), a
// deliberate pause measured as one enormous tick, reset() leaving a stale mean.
TEST_CASE("TickTimingStats T3: the baseline tick measures nothing; rebaseline skips a gap and "
          "keeps the statistics; reset forgets them") {
    TickTimingStats s;
    CHECK(s.tick(Time{5.0}).value() == 0.0);
    TickTimingSummary one = s.summary();
    CHECK(one.count == 0U);
    CHECK(one.meanS == 0.0);
    CHECK(one.minS == 0.0);
    CHECK(one.valid());

    CHECK(s.tick(Time{5.01}).value() == rel(0.01, 1e-9));
    CHECK(s.tick(Time{5.02}).value() == rel(0.01, 1e-9));
    CHECK(s.summary().count == 2U);
    CHECK(s.summary().sigmaS == doctest::Approx(0.0).epsilon(1e-12));

    s.rebaseline();
    CHECK(s.tick(Time{60.0}).value() == 0.0);  // the 55 s pause is not a tick
    CHECK(s.summary().count == 2U);
    CHECK(s.summary().maxS < 0.011);
    CHECK(s.tick(Time{60.01}).value() == rel(0.01, 1e-9));
    CHECK(s.summary().count == 3U);

    s.reset();
    CHECK(s.summary().count == 0U);
    CHECK(s.tick(Time{70.0}).value() == 0.0);  // reset forgets the baseline too
    CHECK(s.summary().count == 0U);
    CHECK(s.summary().meanS == 0.0);
}

// ═══ T4. valid() ═════════════════════════════════════════════════════════════════════
TEST_CASE("TickTimingStats T4: a summary that cannot hold together is not valid") {
    TickTimingSummary ok = feed({0.0, 0.01, 0.021, 0.03});
    REQUIRE(ok.valid());
    TickTimingSummary b = ok;
    b.overBudget = b.count + 1;
    CHECK_FALSE(b.valid());
    b = ok;
    b.overDouble = b.count + 1;
    CHECK_FALSE(b.valid());
    b = ok;
    b.meanS = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(b.valid());
    b = ok;
    b.maxS = std::numeric_limits<double>::infinity();
    CHECK_FALSE(b.valid());
    b = ok;
    b.sigmaS = -1e-9;
    CHECK_FALSE(b.valid());
    b = ok;
    b.sigmaS = std::numeric_limits<double>::infinity();  // passes ">= 0"; only isfinite refuses it
    CHECK_FALSE(b.valid());
    b = ok;
    b.minS = b.maxS + 1e-6;
    CHECK_FALSE(b.valid());
    TickTimingSummary empty;
    CHECK(empty.valid());
    empty.meanS = 0.01;  // a time with nothing measured
    CHECK_FALSE(empty.valid());
}

// ═══ T5. Preconditions ═══════════════════════════════════════════════════════════════
TEST_CASE("TickTimingStats T5: a budget at or under nominal, or a non-finite now, is a "
          "precondition breach") {
    CHECK_THROWS_AS(TickTimingStats(TickTimingConfig{Time{0.01}, Time{0.01}}), PreconditionError);
    CHECK_THROWS_AS(TickTimingStats(TickTimingConfig{Time{0.0}, Time{0.015}}), PreconditionError);
    CHECK_THROWS_AS(TickTimingStats(TickTimingConfig{
                        Time{0.01}, Time{std::numeric_limits<double>::infinity()}}),
                    PreconditionError);
    TickTimingStats s;
    CHECK_THROWS_AS((void)s.tick(Time{std::numeric_limits<double>::quiet_NaN()}), PreconditionError);
}
