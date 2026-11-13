// calibration::FeedforwardFit (chunk R5a Part A, A.1) on SYNTHETIC data — the law exactly, the law
// broken on purpose, and the numerics. The characterizer's cases on the host test plant are in
// drive_characterizer_test.cpp. Each case names the bug it catches and the mutation that must turn
// it red:
//   F1  exact data from the discrete law (kS 0.9, kV 0.13, kA 0.02) over the characterizer's four
//       segments → every gain to 1e-9          (drop the γ·sgn(v) column; kA without the logarithm)
//   F2  kA = 0 → kA EXACTLY 0 with kABelowResolution, kS and kV to 1e-9, no NaN
//   F3  α = +1e-12 AND α = −1e-12 → kA exactly 0 and the flag, both times (the brief's R1); the
//       threshold's VALUE is e^−5 (τMin = dt/5), and 10 % either side of it lands either side
//                                              (compare α against 0 instead of alphaMin: red;
//                                               alphaMin at e^−2.5: red)
//   F4  one voltage level only → NotExcited
//   F5  ramp-shaped data only at kA = 0 → NotExcited (the brief's R2); the same ramps with the step
//       segments added → Ok
//   F6  a flipped reading sign → SignMismatch   (remove the β sign check: red)
//   F7  kS < 0 → NonPhysical; α >= 1 → NonPhysical
//   F8  each FitConfig range at its edge: inside passes, one ulp outside refuses; data far outside
//       reaches Implausible through the verdict path
//   F9  TooFewSamples at minSamples − 1, not at minSamples
//   F10 admission: every reason counted exactly, each boundary on the admitted side where the header
//       puts it (dt at ±10 % exactly admitted; |v| at exactly minSpeed admitted; current at exactly
//       the guard excluded); NaN and ±Inf in EVERY field, speeds before and after included
//   F11 each exclusion PROTECTS the fit — off-model rows of each kind added to exact data leave the
//       gains exact                            (admit dead-band / u = 0 / off-nominal-dt / non-finite
//                                               speed rows: red)
//   F12 voltsMin / voltsMax are the admitted samples' least and greatest |volts|, wherever they come
//                                              (first admitted / last admitted: red)
//   ── model mismatch: data the law does NOT describe (the brief's §2.A; A4 register HA-171) ──
//   M1  a current-limited acceleration, its rows flagged at the limit → excluded by the current
//       guard, the gains exact                 (current guard off: red)
//   M2  a free-wheeling coast-down at u = 0, off the law → excluded, the gains exact
//   M3  a reading the law cannot explain (white noise of 5 in/s) → PoorFit, and still PoorFit under
//       a range that excludes its gains        (remove the PoorFit check: red; judge the ranges
//                                               before PoorFit: red)
//   M4  THE DRAG SWEEP (0 to 0.02 V/(in/s)²): the largest quadratic drag the fit still calls Ok, the
//       kS/kV bias it leaves, and the verdict past it — pinned here and written into HA-171. FOUND: a
//       drag NEVER reaches PoorFit (its residual stays under 0.1 in/s); the first refusal is
//       NonPhysical (kS < 0). PoorFit is not a general mismatch detector — and not a drag detector.
//   M5  a first-order-filtered velocity() (τ of 1, 2 and 3 ticks) — the bias pinned, not refused
//   M6  a one-tick command latency — the bias pinned, not refused
//   M7  white noise on the speed reading (σ 0.5 / 1 / 2 in/s, and 0.5 with the current guard
//       excluding the step's opening) — kA low, kS high, every fit Ok: the bias pinned, not refused
//   ── numerics ──
//   N1  10⁵ samples at ~80 in/s with a ~1e-3 in/s residual → the residual RMS equals a two-pass
//       reference to 1e-9 relative             (the normal equations: red — run as a mutation)
//   N2  FitConfig::valid() names each bad field, at the boundaries (minSpeed 0, dtTolerance 0);
//       the fit refuses an invalid config
// EVERY NUMBER HERE IS INVENTED. These cases prove the fit's arithmetic and its honesty about what
// it cannot see; they say nothing about whether a V5 drive side follows the law.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "shulib/calibration/feedforward_fit.hpp"
#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/sim/rng.hpp"

using shulib::PreconditionError;
using shulib::calibration::FeedforwardFit;
using shulib::calibration::FitConfig;
using shulib::calibration::FitExclusion;
using shulib::calibration::FitResult;
using shulib::calibration::FitSample;
using shulib::calibration::FitVerdict;
using shulib::calibration::fitVerdictName;
using shulib::calibration::gainsPlausible;
using shulib::control::FeedforwardGains;
using shulib::sim::Rng;

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

constexpr double kDt = 0.01;

/// The law's steady state: the plant's inversion, dead band included (motor_model.hpp's shape,
/// re-typed here — the fit is checked against the LAW, not against the plant's code).
[[nodiscard]] double steadyState(double u, double kS, double kV) {
    if (std::abs(u) <= kS) {
        return 0.0;
    }
    return (u > 0.0 ? u - kS : u + kS) / kV;
}

/// One tick of the exact law: v → v_ss + (v − v_ss)·e^(−dt·kV/kA); kA = 0 jumps to v_ss.
[[nodiscard]] double exactStep(double v, double u, double kS, double kV, double kA, double dt) {
    const double vss = steadyState(u, kS, kV);
    if (kA <= 0.0) {
        return vss;
    }
    return vss + (v - vss) * std::exp(-dt * kV / kA);
}

/// What one tick of a "robot" does: the next speed from (speed, volts), plus the current it reports.
struct Tick {
    double v = 0.0;
    double current = 1.0;
};
using Law = std::function<Tick(double v, double u)>;
/// What the robot does while stopped (u = 0) — by default the law's own decay.
using StopLaw = std::function<double(double v)>;

/// The characterizer's four segments, simulated: ramp forward at 2 V/s from 0, ramp backward, step
/// +6 V, step −6 V; each until 36 in, 10 V or 8 s; each followed by a stop at u = 0 until the speed
/// is under 0.05 in/s. Returns the true (u, v, current) stream; samples are formed from it.
struct Trace {
    std::vector<double> u;        // u[k] commanded at tick k
    std::vector<double> v;        // v[k] the true speed at tick k (size u.size() + 1)
    std::vector<double> current;  // current read with v[k+1]
};
Trace fourSegments(const Law& law, const StopLaw& stop, bool steps = true, bool ramps = true) {
    Trace tr;
    tr.v.push_back(0.0);
    const int kinds[4] = {0, 1, 2, 3};
    for (int kind : kinds) {
        const bool quasi = kind < 2;
        if ((quasi && !ramps) || (!quasi && !steps)) {
            continue;
        }
        const double sign = (kind == 0 || kind == 2) ? 1.0 : -1.0;
        double pos = 0.0;
        double t = 0.0;
        double v = 0.0;
        tr.v.back() = 0.0;  // each segment starts from rest
        for (;;) {
            const double u = quasi ? sign * 2.0 * t : sign * 6.0;
            if (std::abs(pos) >= 36.0 || t >= 8.0 || (quasi && std::abs(u) >= 10.0)) {
                break;
            }
            const Tick next = law(v, u);
            tr.u.push_back(u);
            tr.current.push_back(next.current);
            tr.v.push_back(next.v);
            pos += next.v * kDt;
            v = next.v;
            t += kDt;
        }
        for (int i = 0; i < 400 && std::abs(v) >= 0.05; ++i) {
            v = stop(v);
            tr.u.push_back(0.0);
            tr.current.push_back(0.2);
            tr.v.push_back(v);
        }
    }
    return tr;
}

/// Samples from a trace, reading the speed through `measured` (the truth by default).
std::vector<FitSample> samplesOf(const Trace& tr, const std::vector<double>* measured = nullptr) {
    const std::vector<double>& v = measured != nullptr ? *measured : tr.v;
    std::vector<FitSample> out;
    for (std::size_t k = 0; k < tr.u.size(); ++k) {
        out.push_back(FitSample{v[k], tr.u[k], v[k + 1], kDt, tr.current[k]});
    }
    return out;
}

FitResult fitOf(const std::vector<FitSample>& samples, const FitConfig& cfg = {}) {
    FeedforwardFit fit{cfg};
    for (const FitSample& s : samples) {
        (void)fit.add(s);
    }
    return fit.result();
}

Law exactLaw(double kS, double kV, double kA) {
    return [=](double v, double u) { return Tick{exactStep(v, u, kS, kV, kA, kDt), 1.0}; };
}
StopLaw exactStop(double kS, double kV, double kA) {
    return [=](double v) { return exactStep(v, 0.0, kS, kV, kA, kDt); };
}

void requireGains(const FitResult& r, double kS, double kV, double kA, double relTol) {
    const std::string verdict = fitVerdictName(r.verdict);
    CAPTURE(verdict);
    REQUIRE(r.verdict == FitVerdict::Ok);
    CHECK(r.gains.kS == rel(kS, relTol));
    CHECK(r.gains.kV == rel(kV, relTol));
    if (kA == 0.0) {
        CHECK(r.gains.kA == 0.0);  // a zero target is exact, never "close"
    } else {
        CHECK(r.gains.kA == rel(kA, relTol));
    }
}

/// Exact data straight from the DISCRETE law with a chosen α (so α can be put where a continuous
/// kA cannot): v' = α·v + β·u + γ·sgn(v), β = (1 − α)/kV, γ = −(1 − α)·kS/kV, over a sweep of volts
/// in both directions that excites every column.
std::vector<FitSample> discreteData(double alpha, double kS, double kV) {
    const double beta = (1.0 - alpha) / kV;
    const double gamma = -(1.0 - alpha) * kS / kV;
    std::vector<FitSample> out;
    for (int dir = -1; dir <= 1; dir += 2) {
        double v = dir * 5.0;
        for (int k = 0; k < 300; ++k) {
            const double u = dir * (2.0 + 8.0 * std::fmod(k * 0.137, 1.0));
            const double next = alpha * v + beta * u + gamma * (v > 0.0 ? 1.0 : -1.0);
            out.push_back(FitSample{v, u, next, kDt, 1.0});
            v = next;
        }
    }
    return out;
}

}  // namespace

// ═══ F1. The exact law over the four segments ════════════════════════════════════════
// Bug caught: a missing friction column (kS lands in the intercept of nothing), kA computed as
// (1 − α)… or any other shortcut for −kV·dt/ln α, a sign slip in kS = −γ/β.
TEST_CASE("FeedforwardFit F1: exact data from the discrete law over the four segments recovers "
          "kS, kV and kA to 1e-9") {
    const double kS = 0.9;
    const double kV = 0.13;
    const double kA = 0.02;
    const Trace tr = fourSegments(exactLaw(kS, kV, kA), exactStop(kS, kV, kA));
    const FitResult r = fitOf(samplesOf(tr));
    requireGains(r, kS, kV, kA, 1e-9);
    CHECK_FALSE(r.kABelowResolution);
    CHECK(r.alpha == rel(std::exp(-kDt * kV / kA), 1e-12));
    CHECK(r.residualRms < 1e-9);
    CHECK(r.samples > 300U);
    CHECK(r.voltsMax == rel(6.0, 1e-12));  // the steps; the ramps stop earlier
    CHECK(r.voltsMin > 0.9);   // nothing inside the dead band is admitted (kS 0.9) …
    CHECK(r.voltsMin < 1.5);   // … and, with the lag, the first admitted volts is a little past it
    // Every stop sample and every dead-band sample was excluded, and counted.
    CHECK(r.excluded[static_cast<std::size_t>(FitExclusion::Stop) - 1] > 50U);
    CHECK(r.excluded[static_cast<std::size_t>(FitExclusion::DeadBand) - 1] > 10U);
}

// ═══ F2. kA = 0 → kA exactly 0 ═══════════════════════════════════════════════════════
TEST_CASE("FeedforwardFit F2: a kA = 0 law gives kA EXACTLY 0, flagged, with kS and kV to 1e-9 "
          "and no NaN") {
    const Trace tr = fourSegments(exactLaw(1.0, 0.16, 0.0), exactStop(1.0, 0.16, 0.0));
    const FitResult r = fitOf(samplesOf(tr));
    requireGains(r, 1.0, 0.16, 0.0, 1e-9);
    CHECK(r.gains.kA == 0.0);  // exactly
    CHECK(r.kABelowResolution);
    CHECK(std::isfinite(r.alpha));
    CHECK(std::abs(r.alpha) < 1e-9);
}

// ═══ F3. α = ±1e-12 — the brief's R1 ═══════════════════════════════════════════════
// Bug caught: kA judged by α's SIGN — at α = +1e-12 that reports kA ≈ −kV·dt/ln(1e-12) ≈ 5·10⁻⁵
// instead of 0, and at −1e-12 a logarithm of a negative number. Which sign a kA = 0 plant fits to
// is floating-point luck, so the resolution threshold must cover both.
TEST_CASE("FeedforwardFit F3: data generated with alpha = +1e-12 and with alpha = -1e-12 both give "
          "kA exactly 0 and the flag") {
    for (double alpha : {1e-12, -1e-12}) {
        CAPTURE(alpha);
        const FitResult r = fitOf(discreteData(alpha, 1.0, 0.16));
        REQUIRE(r.verdict == FitVerdict::Ok);
        CHECK(r.gains.kA == 0.0);
        CHECK(r.kABelowResolution);
        CHECK(r.gains.kV == rel(0.16, 1e-9));
        CHECK(r.gains.kS == rel(1.0, 1e-9));
    }
    // And just ABOVE the resolution threshold kA is reported, not flagged.
    const FitConfig cfg;
    const double alpha = 2.0 * cfg.alphaMin();
    const FitResult r = fitOf(discreteData(alpha, 1.0, 0.16));
    REQUIRE(r.verdict == FitVerdict::Ok);
    CHECK_FALSE(r.kABelowResolution);
    CHECK(r.gains.kA == rel(-0.16 * kDt / std::log(alpha), 1e-9));
    // The threshold's VALUE, not only its use (the R5a review of 2026-10-04: "2 × alphaMin" above is
    // computed by the code under test, so a wrong alphaMin passed it). τMin = dt/5 (HA-170) means
    // alphaMin = e^(−dt/τMin) = e^−5 EXACTLY, and 10 % either side of it lands on either side.
    CHECK(cfg.alphaMin() == std::exp(-5.0));
    const double below = 0.9 * std::exp(-5.0);
    const double above = 1.1 * std::exp(-5.0);
    const FitResult under = fitOf(discreteData(below, 1.0, 0.16));
    REQUIRE(under.verdict == FitVerdict::Ok);
    CHECK(under.kABelowResolution);
    CHECK(under.gains.kA == 0.0);
    const FitResult over = fitOf(discreteData(above, 1.0, 0.16));
    REQUIRE(over.verdict == FitVerdict::Ok);
    CHECK_FALSE(over.kABelowResolution);
    CHECK(over.gains.kA == rel(-0.16 * kDt / std::log(above), 1e-9));
}

// ═══ F4. One voltage level only ══════════════════════════════════════════════════════
TEST_CASE("FeedforwardFit F4: one voltage level only is NotExcited") {
    const Law law = exactLaw(1.0, 0.16, 0.02);
    std::vector<FitSample> s;
    double v = 0.0;
    for (int k = 0; k < 400; ++k) {
        const double next = law(v, 6.0).v;  // 6 V held from rest: the speed climbs, the volts do not
        s.push_back(FitSample{v, 6.0, next, kDt, 1.0});
        v = next;
    }
    const FitResult r = fitOf(s);
    CHECK(r.verdict == FitVerdict::NotExcited);
    CHECK(r.samples > 300U);
}

// ═══ F5. Ramps alone at kA = 0 (R2) ══════════════════════════════════════════════════
// Bug caught: a rank test too lax to see exact collinearity — the fit would then report numbers
// from a singular system. And the converse: the steps DO make the same law identifiable.
TEST_CASE("FeedforwardFit F5: ramp-shaped data alone at kA = 0 is NotExcited; with the step "
          "segments it is Ok") {
    const Law law = exactLaw(1.0, 0.16, 0.0);
    const StopLaw stop = exactStop(1.0, 0.16, 0.0);
    CHECK(fitOf(samplesOf(fourSegments(law, stop, /*steps=*/false))).verdict == FitVerdict::NotExcited);
    CHECK(fitOf(samplesOf(fourSegments(law, stop, /*steps=*/true))).verdict == FitVerdict::Ok);
    // At kA > 0 a ramp alone is identifiable, only worse conditioned (the header: ~40×) — not
    // refused, and still exact on exact data.
    const Law lag = exactLaw(1.0, 0.16, 0.02);
    requireGains(fitOf(samplesOf(fourSegments(lag, exactStop(1.0, 0.16, 0.02), false))), 1.0, 0.16,
                 0.02, 1e-6);
}

// ═══ F6. A flipped reading sign ══════════════════════════════════════════════════════
// Bug caught: the fit returning a NEGATIVE kV (or a huge kS) when the command's and the reading's
// signs disagree — a sign convention broken upstream would then sail through as "gains".
TEST_CASE("FeedforwardFit F6: a reading whose sign is flipped against the command is "
          "SignMismatch") {
    const Trace tr = fourSegments(exactLaw(1.0, 0.16, 0.02), exactStop(1.0, 0.16, 0.02));
    std::vector<double> flipped = tr.v;
    for (double& x : flipped) {
        x = -x;
    }
    const FitResult r = fitOf(samplesOf(tr, &flipped));
    CHECK(r.verdict == FitVerdict::SignMismatch);
    CHECK(r.beta < 0.0);
}

// ═══ F7. NonPhysical ═════════════════════════════════════════════════════════════════
TEST_CASE("FeedforwardFit F7: kS < 0, or alpha >= 1, is NonPhysical") {
    SUBCASE("kS < 0") {
        const FitResult r = fitOf(discreteData(0.9, -0.5, 0.16));
        CHECK(r.verdict == FitVerdict::NonPhysical);
        CHECK(r.gains.kS == rel(-0.5, 1e-9));
    }
    SUBCASE("alpha >= 1") {
        std::vector<FitSample> s;
        for (int dir = -1; dir <= 1; dir += 2) {
            double v = dir * 5.0;
            for (int k = 0; k < 150; ++k) {  // short: a growing law must stay finite
                const double u = dir * (2.0 + 8.0 * std::fmod(k * 0.137, 1.0));
                const double next = 1.01 * v + 0.05 * u - 0.1 * (v > 0.0 ? 1.0 : -1.0);
                s.push_back(FitSample{v, u, next, kDt, 1.0});
                v = next;
            }
        }
        const FitResult r = fitOf(s);
        CHECK(r.verdict == FitVerdict::NonPhysical);
        CHECK(r.alpha == rel(1.01, 1e-9));
    }
}

// ═══ F8. The ranges, at their edges ══════════════════════════════════════════════════
// Bug caught: an edge excluded that the config says is inside (`<` for `<=`), a range tested on the
// wrong quantity (free speed computed without kS), and a range skipped in the verdict path.
TEST_CASE("FeedforwardFit F8: each FitConfig range at its edge is inside, and one ulp outside "
          "refuses; data far outside reaches Implausible") {
    const FeedforwardGains g{.kS = 1.25, .kV = 0.0625, .kA = 0.03125};
    const double freeSpeed = (12.0 - g.kS) / g.kV;  // the same expression the check evaluates
    auto at = [&](auto set) {
        FitConfig c;
        set(c);
        return c;
    };
    char why[160];
    // Each bound placed EXACTLY on the value is inside …
    CHECK(gainsPlausible(g, at([&](FitConfig& c) { c.kSMax = g.kS; }), why, sizeof why));
    CHECK(gainsPlausible(g, at([&](FitConfig& c) { c.kSMin = g.kS; }), why, sizeof why));
    CHECK(gainsPlausible(g, at([&](FitConfig& c) { c.freeSpeedMax = freeSpeed; }), why, sizeof why));
    CHECK(gainsPlausible(g, at([&](FitConfig& c) { c.freeSpeedMin = freeSpeed; }), why, sizeof why));
    CHECK(gainsPlausible(g, at([&](FitConfig& c) { c.kAMax = g.kA; }), why, sizeof why));
    CHECK(gainsPlausible(g, at([&](FitConfig& c) { c.kAMin = g.kA; }), why, sizeof why));
    // … and one ulp past it refuses, naming the range.
    CHECK_FALSE(gainsPlausible(g, at([&](FitConfig& c) { c.kSMax = std::nextafter(g.kS, 0.0); }),
                               why, sizeof why));
    CHECK(std::strstr(why, "kS") != nullptr);
    CHECK_FALSE(gainsPlausible(g, at([&](FitConfig& c) { c.kSMin = std::nextafter(g.kS, 9.0); }),
                               why, sizeof why));
    CHECK_FALSE(gainsPlausible(
        g, at([&](FitConfig& c) { c.freeSpeedMax = std::nextafter(freeSpeed, 0.0); }), why,
        sizeof why));
    CHECK(std::strstr(why, "free speed") != nullptr);
    CHECK_FALSE(gainsPlausible(
        g, at([&](FitConfig& c) { c.freeSpeedMin = std::nextafter(freeSpeed, 1e9); }), why,
        sizeof why));
    CHECK_FALSE(gainsPlausible(g, at([&](FitConfig& c) { c.kAMax = std::nextafter(g.kA, 0.0); }),
                               why, sizeof why));
    CHECK(std::strstr(why, "kA") != nullptr);
    CHECK_FALSE(gainsPlausible(g, at([&](FitConfig& c) { c.kAMin = std::nextafter(g.kA, 1.0); }),
                               why, sizeof why));
    // The defaults' own edges, on the values themselves.
    const FitConfig d;
    CHECK(gainsPlausible(FeedforwardGains{.kS = 4.0, .kV = 0.1, .kA = 0.2}, d, why, sizeof why));
    CHECK_FALSE(gainsPlausible(FeedforwardGains{.kS = std::nextafter(4.0, 5.0), .kV = 0.1, .kA = 0.0},
                               d, why, sizeof why));
    CHECK_FALSE(gainsPlausible(FeedforwardGains{.kS = 1.0, .kV = 0.1, .kA = std::nextafter(0.2, 1.0)},
                               d, why, sizeof why));
    // Through the verdict path: exact data from a law far outside each range.
    auto verdictFor = [](double kS, double kV, double kA) {
        return fitOf(samplesOf(fourSegments(exactLaw(kS, kV, kA), exactStop(kS, kV, kA)))).verdict;
    };
    CHECK(verdictFor(0.5, 0.7, 0.02) == FitVerdict::Implausible);   // free speed 16.4 < 20
    CHECK(verdictFor(1.0, 0.05, 0.02) == FitVerdict::Implausible);  // free speed 220 > 200
    CHECK(verdictFor(5.0, 0.16, 0.02) == FitVerdict::Implausible);  // kS 5 > 4
    CHECK(verdictFor(0.5, 0.16, 0.3) == FitVerdict::Implausible);   // kA 0.3 > 0.2
}

// ═══ F9. TooFewSamples ═══════════════════════════════════════════════════════════════
TEST_CASE("FeedforwardFit F9: minSamples - 1 admitted samples is TooFewSamples, minSamples is not") {
    const std::vector<FitSample> all = discreteData(0.9, 1.0, 0.16);
    FitConfig cfg;
    cfg.minSamples = 100;
    FeedforwardFit fit{cfg};
    std::size_t i = 0;
    while (fit.admitted() < 99U) {
        (void)fit.add(all[i++]);
    }
    CHECK(fit.result().verdict == FitVerdict::TooFewSamples);
    while (fit.admitted() < 100U) {
        (void)fit.add(all[i++]);
    }
    CHECK(fit.result().verdict == FitVerdict::Ok);
}

// ═══ F10. Admission, reason by reason, at the boundaries ═══════════════════════════
// Bug caught: a boundary on the wrong side (the header says: dt at exactly ±tolerance and |v| at
// exactly minSpeed are ADMITTED; a current at exactly the guard is EXCLUDED), a reason counted under
// another, the order changed so a sample is blamed on the wrong thing.
TEST_CASE("FeedforwardFit F10: every exclusion is counted under its own reason, each boundary on "
          "the side the header states") {
    FeedforwardFit fit;
    const FitConfig& c = fit.config();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(fit.add(FitSample{10.0, nan, 10.0, kDt, 1.0}) == FitExclusion::NonFinite);
    CHECK(fit.add(FitSample{10.0, 3.0, 10.0, kDt, inf}) == FitExclusion::NonFinite);
    // Every field, not only volts and current (the R5a review: a NaN or ±Inf SPEED — a side whose
    // every member read non-finite — passed the dead-band compare and reached R with the speed half
    // of this rule removed): NaN and ±Inf before and after, and a non-finite dt.
    CHECK(fit.add(FitSample{nan, 3.0, 10.0, kDt, 1.0}) == FitExclusion::NonFinite);
    CHECK(fit.add(FitSample{10.0, 3.0, nan, kDt, 1.0}) == FitExclusion::NonFinite);
    CHECK(fit.add(FitSample{inf, 3.0, 10.0, kDt, 1.0}) == FitExclusion::NonFinite);
    CHECK(fit.add(FitSample{-inf, 3.0, 10.0, kDt, 1.0}) == FitExclusion::NonFinite);
    CHECK(fit.add(FitSample{10.0, 3.0, inf, kDt, 1.0}) == FitExclusion::NonFinite);
    CHECK(fit.add(FitSample{10.0, 3.0, -inf, kDt, 1.0}) == FitExclusion::NonFinite);
    CHECK(fit.add(FitSample{10.0, 3.0, 10.0, nan, 1.0}) == FitExclusion::NonFinite);
    CHECK(fit.add(FitSample{10.0, 3.0, 10.0, kDt * 1.1000001, 1.0}) == FitExclusion::OffNominalDt);
    CHECK(fit.add(FitSample{10.0, 3.0, 10.0, kDt * 0.8999999, 1.0}) == FitExclusion::OffNominalDt);
    CHECK(fit.add(FitSample{10.0, 3.0, 10.0, 0.0, 1.0}) == FitExclusion::OffNominalDt);
    CHECK(fit.add(FitSample{10.0, 0.0, 9.0, kDt, 1.0}) == FitExclusion::Stop);
    CHECK(fit.add(FitSample{std::nextafter(c.minSpeed, 0.0), 3.0, 10.0, kDt, 1.0})
          == FitExclusion::DeadBand);
    CHECK(fit.add(FitSample{10.0, 3.0, -std::nextafter(c.minSpeed, 0.0), kDt, 1.0})
          == FitExclusion::DeadBand);
    CHECK(fit.add(FitSample{10.0, 3.0, 10.0, kDt, c.currentGuard}) == FitExclusion::OverCurrent);
    // The admitted side of each boundary.
    CHECK(fit.add(FitSample{c.minSpeed, 3.0, c.minSpeed, kDt, 1.0}) == FitExclusion::None);
    CHECK(fit.add(FitSample{10.0, 3.0, 10.0, kDt, std::nextafter(c.currentGuard, 0.0)})
          == FitExclusion::None);
    CHECK(fit.admitted() == 2U);
    CHECK(fit.excluded(FitExclusion::NonFinite) == 9U);
    CHECK(fit.excluded(FitExclusion::OffNominalDt) == 3U);
    CHECK(fit.excluded(FitExclusion::Stop) == 1U);
    CHECK(fit.excluded(FitExclusion::DeadBand) == 2U);
    CHECK(fit.excluded(FitExclusion::OverCurrent) == 1U);
    CHECK(fit.excluded(FitExclusion::None) == 0U);
    const FitResult r = fit.result();
    CHECK(r.excluded[0] == 9U);
    CHECK(r.excluded[4] == 1U);

    // The dt band's edges EXACTLY, in binary-exact numbers (a nominal of 2^-7 s and a tolerance of
    // 1/4, so the band is 2^-7 ± 2^-9 with no rounding anywhere): both edges admitted, one ulp
    // outside either excluded.
    FitConfig exact;
    exact.nominalDt = 0.0078125;
    exact.dtTolerance = 0.25;
    FeedforwardFit edge{exact};
    const double lo = 0.0078125 - 0.001953125;
    const double hi = 0.0078125 + 0.001953125;
    CHECK(edge.add(FitSample{10.0, 3.0, 10.0, lo, 1.0}) == FitExclusion::None);
    CHECK(edge.add(FitSample{10.0, 3.0, 10.0, hi, 1.0}) == FitExclusion::None);
    CHECK(edge.add(FitSample{10.0, 3.0, 10.0, std::nextafter(lo, 0.0), 1.0})
          == FitExclusion::OffNominalDt);
    CHECK(edge.add(FitSample{10.0, 3.0, 10.0, std::nextafter(hi, 1.0), 1.0})
          == FitExclusion::OffNominalDt);
}

// ═══ F11. Each exclusion protects the fit ═══════════════════════════════════════════
// Bug caught: an exclusion rule "simplified" away. Each kind of row the header excludes is added,
// OFF the law, to exact data; the gains must stay exact — which is only true while the rule holds.
TEST_CASE("FeedforwardFit F11: off-law rows of every excluded kind leave exact data's gains exact") {
    const double kS = 1.0;
    const double kV = 0.16;
    const double kA = 0.02;
    std::vector<FitSample> s =
        samplesOf(fourSegments(exactLaw(kS, kV, kA), exactStop(kS, kV, kA)));
    SUBCASE("dead-band rows: a wheel held at 0 while volts rise to kS") {
        for (int k = 0; k < 200; ++k) {
            s.push_back(FitSample{0.0, 0.004 * k, 0.0, kDt, 1.0});
            s.push_back(FitSample{0.5, 0.004 * k, 0.4, kDt, 1.0});
        }
    }
    SUBCASE("u = 0 rows: a coast-down with no friction term, the plant's own shape") {
        for (int k = 0; k < 200; ++k) {
            const double v = 40.0 * std::exp(-0.05 * k);
            s.push_back(FitSample{v, 0.0, v * std::exp(-kDt * kV / kA), kDt, 0.2});
        }
    }
    SUBCASE("off-nominal dt rows: the law over 2 ticks, which a one-tick alpha cannot describe") {
        double v = 5.0;
        for (int k = 0; k < 200; ++k) {
            const double u = 3.0 + 0.03 * k;
            const double next = exactStep(v, u, kS, kV, kA, 2.0 * kDt);
            s.push_back(FitSample{v, u, next, 2.0 * kDt, 1.0});
            v = next;
        }
    }
    SUBCASE("non-finite speed rows: a side whose every member read NaN or Inf") {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        for (int k = 0; k < 50; ++k) {
            s.insert(s.begin() + 40 * k, FitSample{nan, 4.0, 20.0, kDt, 1.0});
            s.push_back(FitSample{20.0, 4.0, k % 2 == 0 ? inf : -inf, kDt, 1.0});
        }
    }
    const FitResult r = fitOf(s);
    requireGains(r, kS, kV, kA, 1e-9);
}

// ═══ F12. The volts range is the ADMITTED samples' least and greatest ═══════════════
// Bug caught (the R5a review of 2026-10-04): voltsMin taken as the first admitted |volts| and
// voltsMax as the last — every characterizer run admits its smallest volts first (just past kS)
// and its largest last (the 6 V StepBackward), so both shortcuts passed F1. HA-171 cites this range
// as "the range the fit admitted". Here neither extreme comes first or last, and an EXCLUDED sample
// beyond each end must not move it.
TEST_CASE("FeedforwardFit F12: voltsMin and voltsMax are the least and greatest |volts| of the "
          "admitted samples, wherever they come, and excluded samples do not move them") {
    std::vector<FitSample> s = discreteData(0.9, 1.0, 0.16);
    s.erase(s.begin(), s.begin() + 3);  // discreteData's first volts (2.0) is its smallest
    {
        // One more exact sample of the law at a mid-range 5 V, so the LAST admitted is not an end.
        const FitSample& last = s.back();
        const double alpha = 0.9;
        const double beta = (1.0 - alpha) / 0.16;
        const double gamma = -(1.0 - alpha) * 1.0 / 0.16;
        const double v = last.speedAfter;
        const double u = v > 0.0 ? 5.0 : -5.0;
        s.push_back(FitSample{v, u, alpha * v + beta * u + gamma * (v > 0.0 ? 1.0 : -1.0), kDt, 1.0});
    }
    double lo = std::numeric_limits<double>::infinity();
    double hi = 0.0;
    for (const FitSample& x : s) {
        lo = std::min(lo, std::abs(x.volts));
        hi = std::max(hi, std::abs(x.volts));
    }
    REQUIRE(std::abs(s.front().volts) != lo);
    REQUIRE(std::abs(s.front().volts) != hi);
    REQUIRE(std::abs(s.back().volts) != lo);
    REQUIRE(std::abs(s.back().volts) != hi);
    // Excluded samples beyond both ends, in the middle of the stream: an over-current 11.5 V and a
    // dead-band 0.3 V.
    s.insert(s.begin() + 100, FitSample{20.0, 11.5, 20.0, kDt, 2.5});
    s.insert(s.begin() + 200, FitSample{0.5, 0.3, 0.4, kDt, 1.0});
    FeedforwardFit fit;
    for (const FitSample& x : s) {
        (void)fit.add(x);
    }
    const FitResult r = fit.result();
    REQUIRE(r.verdict == FitVerdict::Ok);
    CHECK(r.excluded[static_cast<std::size_t>(FitExclusion::OverCurrent) - 1] == 1U);
    CHECK(r.excluded[static_cast<std::size_t>(FitExclusion::DeadBand) - 1] == 1U);
    CHECK(r.voltsMin == lo);  // exact: a copied |volts|, no arithmetic
    CHECK(r.voltsMax == hi);
}

// ═══ M1. A current-limited acceleration ═════════════════════════════════════════════
// Bug caught: the current guard switched off. The step segments' start is where a V5 motor meets
// its current limit (A4 register HA-172): the acceleration is then capped by torque, not by the law,
// and those rows are reported AT the limit. Excluded, the rest of the run is exact.
TEST_CASE("FeedforwardFit M1: a current-limited acceleration, its rows at the limit, is excluded "
          "by the current guard and leaves the gains exact") {
    const double kS = 1.0;
    const double kV = 0.16;
    const double kA = 0.02;
    const double aMax = 150.0;  // in/s² — the steps' opening ticks exceed it
    const Law limited = [=](double v, double u) {
        const double free = exactStep(v, u, kS, kV, kA, kDt);
        if (std::abs(free - v) > aMax * kDt) {
            return Tick{v + (free > v ? aMax : -aMax) * kDt, 2.5};  // AT the limit: 2.5 A reported
        }
        return Tick{free, 1.0};
    };
    const FitResult r = fitOf(samplesOf(fourSegments(limited, exactStop(kS, kV, kA))));
    requireGains(r, kS, kV, kA, 1e-9);
    CHECK(r.excluded[static_cast<std::size_t>(FitExclusion::OverCurrent) - 1] > 10U);
    // Without the guard the same data is NOT the law: it biases or refuses (the mutation's target).
    FitConfig noGuard;
    noGuard.currentGuard = 1e9;
    const FitResult unguarded =
        fitOf(samplesOf(fourSegments(limited, exactStop(kS, kV, kA))), noGuard);
    const double worst = std::max({std::abs(unguarded.gains.kS / kS - 1.0),
                                   std::abs(unguarded.gains.kV / kV - 1.0),
                                   std::abs(unguarded.gains.kA / kA - 1.0)});
    CAPTURE(worst);
    CHECK((unguarded.verdict != FitVerdict::Ok || worst > 1e-3));
}

// ═══ M2. A free-wheeling coast-down ═════════════════════════════════════════════════
TEST_CASE("FeedforwardFit M2: a free-wheeling coast-down at u = 0, off the law, is excluded and "
          "leaves the gains exact") {
    const double kS = 1.0;
    const double kV = 0.16;
    const double kA = 0.02;
    // A V5 at 0 V under Coast may free-wheel (HA-143): here it decays 5× slower than the law's τ.
    const StopLaw freeWheel = [=](double v) { return v * std::exp(-kDt * kV / (5.0 * kA)); };
    const FitResult r = fitOf(samplesOf(fourSegments(exactLaw(kS, kV, kA), freeWheel)));
    requireGains(r, kS, kV, kA, 1e-9);
    CHECK(r.excluded[static_cast<std::size_t>(FitExclusion::Stop) - 1] > 200U);
}

namespace {

/// The law with a quadratic drag, kA·v̇ = u − kS·sgn(v) − kV·v − c·v·|v|, integrated by RK4 over 20
/// sub-steps a tick; the dead band holds a wheel at rest while |u| <= kS.
Law dragLaw(double kS, double kV, double kA, double c) {
    return [=](double v, double u) {
        if (v == 0.0 && std::abs(u) <= kS) {
            return Tick{0.0, 1.0};
        }
        const double fs = v != 0.0 ? (v > 0.0 ? 1.0 : -1.0) : (u > 0.0 ? 1.0 : -1.0);
        auto f = [&](double x) { return (u - kS * fs - kV * x - c * x * std::abs(x)) / kA; };
        const int n = 20;
        const double h = kDt / n;
        double x = v;
        for (int i = 0; i < n; ++i) {
            const double k1 = f(x);
            const double k2 = f(x + 0.5 * h * k1);
            const double k3 = f(x + 0.5 * h * k2);
            const double k4 = f(x + h * k3);
            x += h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        }
        return Tick{x, 1.0};
    };
}

FitResult dragFit(double c) {
    return fitOf(samplesOf(fourSegments(dragLaw(1.0, 0.16, 0.02, c), exactStop(1.0, 0.16, 0.02))));
}

}  // namespace

// ═══ M3. What PoorFit IS for: a reading the law cannot explain ══════════════════════
// Bug caught: the PoorFit check removed — the fit would then hand back gains from data it explains
// no better than noise. The data: the exact law, read through white noise of 5 in/s (a velocity()
// that is mostly jitter), which no choice of (α, β, γ) can follow.
TEST_CASE("FeedforwardFit M3: a speed reading the law cannot explain (white noise of 5 in/s) is "
          "PoorFit") {
    const Trace tr = fourSegments(exactLaw(1.0, 0.16, 0.02), exactStop(1.0, 0.16, 0.02));
    Rng rng{7};
    std::vector<double> noisy = tr.v;
    for (std::size_t k = 1; k < noisy.size(); ++k) {
        noisy[k] += 5.0 * rng.nextGaussian();
    }
    const FitResult r = fitOf(samplesOf(tr, &noisy));
    CAPTURE(r.residualRms);
    CHECK(r.verdict == FitVerdict::PoorFit);
    CHECK(r.residualRms > FitConfig{}.maxResidualRms);
    // PoorFit is judged BEFORE the ranges (the header's reason: "kS out of range" would send a
    // person after the wrong thing). The R5a review of 2026-10-04 found no case held data that was
    // both — swapping the two checks survived. Here the SAME data under a kS range that excludes
    // the gains it fitted: still PoorFit.
    REQUIRE(r.gains.kS > 0.0);
    FitConfig tight;
    tight.kSMax = 0.5 * r.gains.kS;
    REQUIRE_FALSE(gainsPlausible(r.gains, tight, nullptr, 0));  // the range really does exclude it
    CHECK(fitOf(samplesOf(tr, &noisy), tight).verdict == FitVerdict::PoorFit);
}

// ═══ M4. THE DRAG SWEEP — what PoorFit does NOT catch (HA-171) ══════════════════════
// Not a correctness test: a PINNED MEASUREMENT of the fit's blind spot, so no document can claim
// PoorFit catches mismatch in general. A quadratic drag c·v·|v| is swept in steps of 0.0005
// V/(in/s)², from 0 to 0.02. What the sweep found (the R5a progress log has the whole table):
//   * the residual NEVER approaches PoorFit's 3 in/s — it stays under 0.075 in/s up to the first
//     refusal (0.0742 at c = 0.0105) and under 0.1 in/s to the end of the sweep (0.0813 at
//     c = 0.02, where the drag at 30 in/s is 18 V): over the ~0–35 in/s the segments reach, a convex
//     drag is absorbed almost perfectly by a larger kV and a smaller kS;
//   * the fit says Ok up to c = 0.0100, where kV is 2.4× the truth and kS 0.024 instead of 1.0;
//   * the first refusal is NonPhysical (kS < 0), at c = 0.0105 — not PoorFit;
//   * already at c = 0.0005 (a drag of 0.45 V at 30 in/s, ~9 % of kV·v) kS is 9.5 % low and kV
//     10.5 % high, with a residual of 0.015 in/s.
// So the brief's "a quadratic drag must reach PoorFit" does NOT hold for this fit at any residual
// bound a real encoder's noise permits; what catches a drag is S2's residual per voltage band and
// the laptop re-fit (D6), and a full-stick run whose speed is predicted from the gains (HA-171's
// settle path). A change to the fit that moves these numbers must move HA-171 with it.
TEST_CASE("FeedforwardFit M4: the largest quadratic drag the fit still calls Ok, the bias it "
          "leaves, and the verdict past it are pinned") {
    double lastOk = -1.0;
    FitResult atLastOk{};
    FitResult firstRefused{};
    double firstRefusedC = -1.0;
    double worstResidual = 0.0;
    for (int i = 0; i <= 40; ++i) {  // c from 0 to 0.02 — past the first refusal, on purpose
        const double c = 0.0005 * i;
        const FitResult r = dragFit(c);
        worstResidual = std::max(worstResidual, r.residualRms);
        CHECK(r.verdict != FitVerdict::PoorFit);  // never — at any drag the sweep reaches
        if (r.verdict != FitVerdict::Ok) {
            if (firstRefusedC < 0.0) {
                firstRefused = r;
                firstRefusedC = c;
            }
            continue;
        }
        CHECK(firstRefusedC < 0.0);  // Ok only BELOW the first refusal: the verdict is monotone
        lastOk = c;
        atLastOk = r;
    }
    CHECK(worstResidual < 0.1);  // never near PoorFit's 3 in/s
    CHECK(dragFit(0.02).residualRms == rel(0.0813, 1e-2));
    CHECK(lastOk == rel(0.0100, 1e-9));
    CHECK(atLastOk.gains.kS == rel(0.02366, 1e-3));
    CHECK(atLastOk.gains.kV == rel(0.37795, 1e-4));
    CHECK(atLastOk.gains.kA == rel(0.02463, 1e-3));
    CHECK(atLastOk.residualRms == rel(0.0734, 1e-2));
    CHECK(firstRefusedC == rel(0.0105, 1e-9));
    CHECK(firstRefused.verdict == FitVerdict::NonPhysical);
    CHECK(firstRefused.gains.kS < 0.0);
    // The small end, where a real drivetrain's drag would plausibly sit.
    const FitResult small = dragFit(0.0005);
    REQUIRE(small.verdict == FitVerdict::Ok);
    CHECK(small.gains.kS == rel(0.90455, 1e-4));
    CHECK(small.gains.kV == rel(0.17678, 1e-4));
    CHECK(small.gains.kA == rel(0.01983, 1e-3));
    CHECK(small.residualRms == rel(0.0149, 1e-2));
}

// ═══ M5. A first-order-filtered velocity() ══════════════════════════════════════════
// A PINNED MEASUREMENT (HA-171): if velocity() is low-pass filtered by the firmware, the fit does
// not refuse — it biases kA up and kS up, kV slightly down, with a residual far under PoorFit's. At
// a filter of τ = 1 / 2 / 3 ticks: kS +0.8 / +2.0 / +2.1 %, kV −0.25 / −0.6 / −1.0 %, kA +3.3 /
// +8.6 / +19 %. Only the laptop re-fit's comparison of velocity() against differenced position (D6)
// can see this filter.
TEST_CASE("FeedforwardFit M5: a velocity() low-pass filtered by 1, 2 and 3 ticks biases the gains "
          "by the pinned amounts, and is not refused") {
    const double kS = 1.0;
    const double kV = 0.16;
    const double kA = 0.02;
    const Trace tr = fourSegments(exactLaw(kS, kV, kA), exactStop(kS, kV, kA));
    const double pinned[3][3] = {{1.007730, 0.159595, 0.020665},
                                 {1.020299, 0.158986, 0.021723},
                                 {1.020655, 0.158388, 0.023816}};
    for (int ticks = 1; ticks <= 3; ++ticks) {
        CAPTURE(ticks);
        const double a = 1.0 - std::exp(-1.0 / ticks);
        std::vector<double> filtered(tr.v.size());
        filtered[0] = tr.v[0];
        for (std::size_t k = 1; k < tr.v.size(); ++k) {
            filtered[k] = filtered[k - 1] + a * (tr.v[k] - filtered[k - 1]);
        }
        const FitResult r = fitOf(samplesOf(tr, &filtered));
        const auto t = static_cast<std::size_t>(ticks - 1);
        CHECK(r.verdict == FitVerdict::Ok);
        CHECK(r.gains.kS == rel(pinned[t][0], 1e-5));
        CHECK(r.gains.kV == rel(pinned[t][1], 1e-5));
        CHECK(r.gains.kA == rel(pinned[t][2], 1e-4));
        CHECK(r.residualRms < 0.05);
    }
}

// ═══ M6. A one-tick command latency ═════════════════════════════════════════════════
// A PINNED MEASUREMENT (HA-171's "a motor command takes effect within one tick", unregistered until
// R5a): a motor that executes LAST tick's command biases kS +2.8 %, kV −0.4 %, kA −0.6 %, residual
// 0.003 in/s — not refused.
TEST_CASE("FeedforwardFit M6: a one-tick command latency biases the gains by the pinned amounts, "
          "and is not refused") {
    const double kS = 1.0;
    const double kV = 0.16;
    const double kA = 0.02;
    double held = 0.0;  // the command the motor is actually executing: last tick's
    const Law late = [&](double v, double u) {
        const double next = exactStep(v, held, kS, kV, kA, kDt);
        held = u;
        return Tick{next, 1.0};
    };
    const StopLaw lateStop = [&](double v) {
        const double next = exactStep(v, held, kS, kV, kA, kDt);
        held = 0.0;
        return next;
    };
    const FitResult r = fitOf(samplesOf(fourSegments(late, lateStop)));
    CHECK(r.verdict == FitVerdict::Ok);
    CHECK(r.gains.kS == rel(1.028438, 1e-5));
    CHECK(r.gains.kV == rel(0.159315, 1e-5));
    CHECK(r.gains.kA == rel(0.019887, 1e-4));
    CHECK(r.residualRms < 0.01);
}

// ═══ M7. Noise on velocity() — biased gains under an Ok verdict ═════════════════════
// A PINNED MEASUREMENT (HA-171; found by the R5a review of 2026-10-04). Noise on the speed READING
// is noise in a REGRESSOR (errors in variables): it pulls α toward 0, so kA reads LOW and, through
// the friction column, kS reads HIGH — and the residual it leaves is only ~1.3·σ, so maxResidualRms
// (3 in/s) calls a fit Ok up to σ ≈ 2.2 in/s of side-median noise. Over 40 seeds a level, on the
// four segments' exact law (kS 1.0, kV 0.16, kA 0.02), every fit is Ok and the MEAN relative errors
// are those pinned below. Excluding the step's current-limited opening ticks (the current guard,
// HA-172) removes most of the excitation and multiplies the bias THREE- TO FOURFOLD (kS 4.2×, kA
// 3.1× the unguarded run's at the same noise): pinned at σ = 0.5 in/s with a 100 in/s² limit. What
// the fit cannot see here, S2 can: the REST step's encoder speed σ at rest is a floor on the
// reading noise, and this table turns it into a bias.
TEST_CASE("FeedforwardFit M7: white noise on the speed reading biases kA low and kS high under an "
          "Ok verdict, by the pinned amounts") {
    const double kS = 1.0;
    const double kV = 0.16;
    const double kA = 0.02;
    struct Mean {
        int ok = 0;
        double resid = 0.0;
        double e[3] = {0.0, 0.0, 0.0};      ///< mean relative error of kS, kV, kA
        double worst[3] = {0.0, 0.0, 0.0};  ///< largest |relative error|
    };
    auto sweep = [&](const Trace& tr, double sigma) {
        Mean m;
        for (std::uint64_t seed = 1; seed <= 40; ++seed) {
            Rng rng{seed};
            std::vector<double> read = tr.v;
            for (double& x : read) {
                x += sigma * rng.nextGaussian();
            }
            const FitResult r = fitOf(samplesOf(tr, &read));
            if (r.verdict != FitVerdict::Ok) {
                continue;
            }
            ++m.ok;
            m.resid += r.residualRms / 40.0;
            const double e[3] = {r.gains.kS / kS - 1.0, r.gains.kV / kV - 1.0, r.gains.kA / kA - 1.0};
            for (int i = 0; i < 3; ++i) {
                m.e[i] += e[i] / 40.0;
                m.worst[i] = std::max(m.worst[i], std::abs(e[i]));
            }
        }
        std::printf("[R5a M7] sigma %.2f in/s: Ok %d/40, residual %.3f, mean rel err kS %+.4f kV %+.4f "
                    "kA %+.4f, worst kS %.3f kV %.3f kA %.3f\n",
                    sigma, m.ok, m.resid, m.e[0], m.e[1], m.e[2], m.worst[0], m.worst[1], m.worst[2]);
        return m;
    };
    const Trace plain = fourSegments(exactLaw(kS, kV, kA), exactStop(kS, kV, kA));
    const double aMax = 100.0;  // in/s² — the steps' opening ticks exceed it, and report 2.5 A
    const Law limited = [=](double v, double u) {
        const double free = exactStep(v, u, kS, kV, kA, kDt);
        if (std::abs(free - v) > aMax * kDt) {
            return Tick{v + (free > v ? aMax : -aMax) * kDt, 2.5};
        }
        return Tick{free, 1.0};
    };
    const Trace guarded = fourSegments(limited, exactStop(kS, kV, kA));
    const Mean m05 = sweep(plain, 0.5);
    const Mean m10 = sweep(plain, 1.0);
    const Mean m20 = sweep(plain, 2.0);
    const Mean mc = sweep(guarded, 0.5);
    for (const Mean* m : {&m05, &m10, &m20, &mc}) {
        CHECK(m->ok == 40);  // the blind spot: every one of them is Ok
    }
    // MEASURED 2026-10-04 (g++ 13.3, x86-64), pinned to ±0.005 of each mean relative error (the
    // seeds are fixed; the tolerance only absorbs a libm's last bits). Each residual is ~1.3·σ.
    auto pin = [](const Mean& m, double kSe, double kVe, double kAe, double resid) {
        CHECK(std::abs(m.e[0] - kSe) <= 0.005);
        CHECK(std::abs(m.e[1] - kVe) <= 0.005);
        CHECK(std::abs(m.e[2] - kAe) <= 0.005);
        CHECK(m.resid == rel(resid, 0.01));
    };
    pin(m05, +0.0335, +0.0078, -0.1800, 0.674);
    pin(m10, +0.1118, +0.0135, -0.4598, 1.337);
    pin(m20, +0.0585, +0.0541, -0.7240, 2.566);
    pin(mc, +0.1411, -0.0031, -0.5561, 0.663);
    // The current guard's exclusion multiplies the bias three- to fourfold at the same noise
    // (σ = 0.5 in/s): more than 3× in kS, more than 2.5× in kA.
    CHECK(mc.e[0] > 3.0 * m05.e[0]);
    CHECK(mc.e[2] < 2.5 * m05.e[2]);
}

// ═══ N1. The numerics: a tiny residual under large speeds ═══════════════════════════
// Bug caught: the normal equations. Over 10⁵ samples at ~80 in/s, Σv² is ~6·10⁸ while the residual
// sum is ~0.1: RSS = yᵀy − θᵀXᵀy then keeps nothing of it. The reference is two-pass — solve, then
// sum the squared residuals of the fit's own coefficients — which any correct accumulation matches.
TEST_CASE("FeedforwardFit N1: 1e5 samples at ~80 in/s with a ~1e-3 in/s residual report the "
          "two-pass residual RMS to 1e-9") {
    const double alpha = 0.9;
    const double kS = 0.9;
    const double kV = 0.13;
    const double beta = (1.0 - alpha) / kV;
    const double gamma = -(1.0 - alpha) * kS / kV;
    Rng rng{20260929};
    std::vector<FitSample> s;
    double v = 80.0;
    for (int k = 0; k < 100000; ++k) {
        const double u = 9.5 + 2.5 * rng.nextUnit();
        const double next = alpha * v + beta * u + gamma + 1e-3 * rng.nextGaussian();
        s.push_back(FitSample{v, u, next, kDt, 1.0});
        v = next;
    }
    const FitResult r = fitOf(s);
    REQUIRE(r.verdict == FitVerdict::Ok);
    double ss = 0.0;
    for (const FitSample& x : s) {
        const double e = x.speedAfter - (r.alpha * x.speedBefore + r.beta * x.volts + r.gamma);
        ss += e * e;
    }
    const double reference = std::sqrt(ss / static_cast<double>(s.size()));
    CAPTURE(reference);
    CAPTURE(r.residualRms);
    CHECK(r.residualRms == rel(reference, 1e-9));
    CHECK(reference == rel(1e-3, 0.02));  // the noise it was built with
    CHECK(r.alpha == rel(alpha, 1e-4));
    CHECK(r.gains.kV == rel(kV, 1e-3));
    CHECK(r.gains.kS == rel(kS, 1e-3));
}

// ═══ N2. FitConfig::valid() ══════════════════════════════════════════════════════════
TEST_CASE("FeedforwardFit N2: FitConfig::valid() names each bad field, and the fit refuses an "
          "invalid config") {
    char why[128];
    CHECK(FitConfig{}.valid(why, sizeof why));
    auto bad = [&](auto set, const char* field) {
        FitConfig c;
        set(c);
        CHECK_FALSE(c.valid(why, sizeof why));
        CHECK(std::strstr(why, field) != nullptr);
        CHECK_THROWS_AS(FeedforwardFit{c}, PreconditionError);
    };
    bad([](FitConfig& c) { c.nominalDt = 0.0; }, "nominalDt");
    bad([](FitConfig& c) { c.dtTolerance = 1.0; }, "dtTolerance");
    bad([](FitConfig& c) { c.dtTolerance = 0.0; }, "dtTolerance");  // the boundary: no band at all
    bad([](FitConfig& c) { c.minSpeed = -1.0; }, "minSpeed");
    bad([](FitConfig& c) { c.minSpeed = 0.0; }, "minSpeed");  // add()'s sgn() needs |v| >= minSpeed > 0
    bad([](FitConfig& c) { c.currentGuard = 0.0; }, "currentGuard");
    bad([](FitConfig& c) { c.tauMinFraction = 0.0; }, "tauMinFraction");
    bad([](FitConfig& c) { c.minSamples = 2; }, "minSamples");
    bad([](FitConfig& c) { c.maxResidualRms = 0.0; }, "maxResidualRms");
    bad([](FitConfig& c) { c.kSMin = 5.0; }, "kSMin");
    bad([](FitConfig& c) { c.freeSpeedMin = 300.0; }, "freeSpeedMin");
    bad([](FitConfig& c) { c.kAMin = 1.0; }, "kAMin");
    bad([](FitConfig& c) { c.kSMax = std::numeric_limits<double>::infinity(); }, "finite");
}
