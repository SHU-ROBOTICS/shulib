#pragma once
//
// FeedforwardFit — kS, kV and kA of ONE drive side, fitted on the brain from that side's own run
// (chunk R5a Part A, 2026-09-29; the team lead's Q7 ruling of 2026-09-26: gains are measured by the
// robot and kept on the SD card, never typed or committed as constants).
//
// ── PROVEN ON THE HOST TEST PLANT AND ON SYNTHETIC DATA; IT HAS NEVER FITTED A ROBOT ─────
// Every property below is shown on data the test generates or on the A2 host test plant — whose
// motor model IS the law fitted here (sim/motor_model.hpp's honesty boundary), so recovering the
// plant's gains proves the ARITHMETIC, not that a V5 drive side follows this law (A4 register
// HA-171). The model-mismatch cases in feedforward_fit_test.cpp exist for exactly that reason, and
// what they found is recorded rather than smoothed over: PoorFit is NOT a general mismatch detector
// (see "What PoorFit does not catch" below). Every number in FitConfig is INVENTED (HA-170).
//
// ── The model: the DISCRETE form of the feedforward law ────────────────────────────
// control::Feedforward commands V = kS·sgn(v) + kV·v + kA·a. Read as the first-order ODE
// kA·v̇ = u − kS·sgn(v) − kV·v and held at one voltage u for one tick of length dt, its exact
// solution is
//
//     v[k+1] = α·v[k] + β·u[k] + γ·sgn(v[k]),   α = e^(−dt·kV/kA),
//                                               β = (1 − α)/kV,
//                                               γ = −(1 − α)·kS/kV,
//
// which is LINEAR in (α, β, γ). So the fit is a linear least-squares problem over three columns —
// the speed read at a tick, the volts commanded at that tick, the sign of that speed — against the
// speed read at the next tick, and the gains come back as
//
//     kV = (1 − α)/β,    kS = −γ/β,    kA = −kV·dt / ln α.
//
// NO DIFFERENTIATED VELOCITY IS USED ANYWHERE. The obvious alternative — regress u against v and a
// numerically differentiated a — divides the noisiest number on the robot (an encoder's rate) by
// 10 ms. The discrete form needs only two consecutive speed readings. It is also EXACTLY the update
// the A2 plant integrates (motor_model.hpp: v_ss + (v − v_ss)·e^(−dt/τ)), which is why the host can
// check the arithmetic to 1e-9 and why that check says nothing about physics.
//
// ── Numerics: incremental QR by Givens rotations, never the normal equations ──────────
// Each admitted sample is rotated into a 3×3 upper-triangular R and its right-hand side as it
// arrives (the sequential form of Householder/Givens least squares — Gentleman 1973), and the part
// of the sample the three columns cannot explain is squared into the residual sum on the spot. The
// normal equations — accumulate XᵀX and Xᵀy, solve, RSS = yᵀy − θᵀXᵀy — are REJECTED: over one run
// Σv² reaches ~10⁷ (in/s)², and the residual sum is then the difference of two numbers ~10⁹ times
// larger than itself, which leaves nothing of it in a double. The test forces the choice without
// dictating it: 10⁵ samples at ~80 in/s with a ~1e-3 in/s residual must report the residual RMS of a
// two-pass reference to 1e-9 relative (a normal-equations build of this class fails it — run as a
// mutation). Allocation-free and O(1) per sample: three rotations of a four-element row.
//
// ── Identifiability: why the STEP segments exist (R5a's re-check R2 and decision D10) ──────────────
// On a ramp, once the start transient has decayed, v[k] is an affine function of u[k] (the
// feedforward law is linear), so the three columns are COLLINEAR: at kA = 0 exactly — the fit says
// NotExcited — and at kA > 0 about 40× worse conditioned. A held STEP breaks the collinearity: the
// speed then climbs at constant volts. drive_characterizer.hpp therefore runs two step segments
// after the two ramps. "One voltage level only" is NotExcited too (the volts column is then the sign
// column times a constant). NotExcited is judged on R's diagonal: a column whose part NOT explained
// by the columns before it is under kRankTolerance of its own norm carries no information of its own.
//
// ── Admission: which samples the law describes, each exclusion counted by reason ─────────
//   * NonFinite     a speed, volts, dt or current that is not a number. Never on a healthy robot;
//                   refused rather than let one NaN poison R for the rest of the run.
//   * OffNominalDt  dt outside nominalDt·(1 ± dtTolerance). α depends on dt; one α is fitted, so a
//                   late tick is a sample of a DIFFERENT law.
//   * Stop          u = 0 exactly: a segment's stop. For TWO reasons, and the exclusion stays if
//                   either goes away (R5a's landmine 5): on a V5 a motor at 0 V under Coast may
//                   free-wheel (A4 register HA-143) and a Brake stop is braking — the law describes
//                   neither; and on the host test plant the friction term follows the STEADY-STATE
//                   sign (motor_model.hpp), so at u = 0 a moving wheel decays with NO friction term at
//                   all — admitting those samples moves the plant's kS from 1.00 to 0.53 (the
//                   re-check's probe; a mutation here goes red on the plant).
//   * DeadBand      |v[k]| or |v[k+1]| under minSpeed: static friction, where sgn(v) is not the law's
//                   and a stuck wheel reads 0 whatever u is.
//   * OverCurrent   any member of the side at or over currentGuard (read at the tick that closes the
//                   sample): the motor is at its current limit, the one non-linearity the ramp's range
//                   is believed to contain (A4 register HA-172), where torque — not the law — sets the
//                   acceleration.
// What admission CANNOT exclude, because it does not know kS in advance: a sample with 0 < |u| <= kS
// and |v| >= minSpeed also violates the law on the plant (its friction follows the steady-state
// sign, which is 0 inside the dead band). The characterizer's segments never produce one — every
// segment starts from rest and never powers a deceleration. Do not add a powered deceleration
// without re-reading this.
//
// ── Verdicts, in the order they are judged ──────────────────────────────────────────
// TooFewSamples (under minSamples) → NotExcited (rank, above) → SignMismatch (β <= 0: more volts made
// the side SLOWER, so the command and the reading disagree in sign — the sign convention broke, rule
// 1 of drivetrain_calibrator.hpp) → NonPhysical (α >= 1 — the law's decay would be growth — or kS < 0)
// → PoorFit (residual RMS over maxResidualRms) → Implausible (outside FitConfig's ranges) → Ok.
// PoorFit is judged BEFORE the ranges on purpose: when the law does not describe the data its
// numbers are meaningless, and "kS out of range" would send a person after the wrong thing.
// FitVerdict is append-only — its values are persisted and printed by name.
//
// ── kA below resolution is NOT an error ─────────────────────────────────────────────
// When the drive responds within one tick, α is ~0 and ln α is useless: a kA = 0 plant fits α to
// ±1e-13, whose positive side would report kA ≈ 5·10⁻⁵ and whose negative side a NaN. So when
// α < alphaMin = e^(−1/tauMinFraction) — a time constant under tauMinFraction of a tick (τMin = dt/5)
// — kA is reported as EXACTLY 0 with kABelowResolution set. That covers α <= 0 too.
//
// ── What PoorFit does not catch (A4 register HA-171) ────────────────────────────────
// PoorFit judges the residual RMS against maxResidualRms and nothing else. A mismatch whose residual
// stays under that bound is FITTED — a quadratic drag up to the size feedforward_fit_test.cpp's drag
// sweep pins, a velocity() that is low-pass filtered by the firmware, a one-tick command latency,
// and NOISE ON THE SPEED READING — and each leaves a kS/kV/kA bias that test pins and HA-171
// records. The noise is the one every robot has: a noisy speed is noise in a REGRESSOR (errors in
// variables), which pulls α toward 0 — kA reads LOW and kS HIGH — while the residual it leaves is
// only ~1.3·σ, so a side-median noise of σ up to ~2.2 in/s is still Ok. Over 40 seeds on the four
// segments' exact law (M7): σ 0.5 in/s → kS +3 %, kV +1 %, kA −18 % on average; σ 1 in/s → kS
// +11 %, kV +1 %, kA −46 %; σ 2 in/s → kS +6 %, kV +5 %, kA −72 % (single runs up to kS +33 %,
// kV +11 %); and when the current guard removes the step's current-limited opening, which carries
// most of the excitation, the bias grows three- to fourfold (σ 0.5 in/s, a 100 in/s² limit:
// kS +14 % against +3 %, kA −56 % against −18 %). kS and kV are applied; the REST step's encoder speed σ at rest is a floor on σ. The fit
// reports the RESIDUAL so a person can judge; it does not claim the law holds.
//
// kA IS FITTED, RECORDED AND NOT APPLIED: the command pipeline calls the cruise overload
// Feedforward::calculate(v) (command_pipeline.hpp step 7) and WheelSpeeds carries no acceleration, so
// no measured kA reaches a motor (R5a's decisions D2 and D10; A4 register HA-45).
//
// Tick-free and clock-free: a caller (drive_characterizer.hpp) hands in one sample per tick. PROS-free,
// sim-free, allocation-free, raises nothing, and every string it returns is ASCII with no '%' (a
// panel may paint it: PROS re-reads screen text as a format).

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"

namespace shulib::calibration {

/// The relative size under which a column of the fit carries no information of its own: R's
/// diagonal entry for the column (the part the columns before it cannot explain) at or under this
/// fraction of the column's own 2-norm is NotExcited. A NUMERICAL constant, not a guess about a
/// robot: exactly collinear data leaves ~1e-15 here after rounding, and the characterizer's
/// plant runs leave ~1e-2 (a ramp at kA > 0 without its steps, ~1e-3) — 1e-9 sits many decades from
/// both.
inline constexpr double kFitRankTolerance = 1e-9;

/// What a fit concluded. APPEND-ONLY: the values are persisted and printed by name
/// (fitVerdictName()); a new verdict goes at the end.
enum class FitVerdict : std::uint8_t {
    Ok = 0,             ///< the gains below are the fit's, inside every range
    TooFewSamples = 1,  ///< fewer admitted samples than FitConfig::minSamples
    NotExcited = 2,     ///< a column carries no information of its own (header: one level, a ramp alone at kA = 0)
    SignMismatch = 3,   ///< β <= 0: more volts made the side slower — the command's and the reading's signs disagree
    NonPhysical = 4,    ///< α >= 1 (the law's decay would be growth) or kS < 0
    Implausible = 5,    ///< the gains are outside FitConfig's ranges (kS, the implied free speed, kA)
    PoorFit = 6,        ///< the residual RMS is over FitConfig::maxResidualRms — the law does not describe the data
};

/// Why a sample was NOT admitted, or None when it was. The order is the order add() judges in.
enum class FitExclusion : std::uint8_t {
    None = 0,          ///< admitted
    NonFinite = 1,     ///< a field that is not a number
    OffNominalDt = 2,  ///< dt outside nominalDt·(1 ± dtTolerance)
    Stop = 3,          ///< u = 0 exactly — a segment's stop (header: two reasons)
    DeadBand = 4,      ///< |v| before or after under minSpeed
    OverCurrent = 5,   ///< a member at or over currentGuard
};

/// How many exclusion reasons there are (FitExclusion's values past None).
inline constexpr std::size_t kFitExclusionReasons = 5;

/// One tick of one side, as the fit reads it: the side's speed at a tick, the volts commanded AT
/// that tick, the side's speed at the NEXT tick, the time between the two readings, and the largest
/// member current read at the next tick. Speeds are the side's median surface speed in in/s (the
/// characterizer's reading: median over members of sign × velocity() × inches per radian).
struct FitSample {
    double speedBefore = 0.0;  ///< v[k], in/s
    double volts = 0.0;        ///< u[k], volts, as commanded (sign included)
    double speedAfter = 0.0;   ///< v[k+1], in/s
    double dt = 0.0;           ///< seconds from the v[k] reading to the v[k+1] reading
    double maxCurrent = 0.0;   ///< amperes: the largest |current()| of any member, read with v[k+1]
};

/// Every number the fit judges with. NOT ONE IS A MEASUREMENT — all are INVENTED for a first run and
/// carried as fields so an S2 serial log can move them with a logged line. PROVISIONAL, every one
/// (A4: HA-170).
struct FitConfig {
    /// The tick the fitted α belongs to, seconds: shulib's 10 ms loop (HA-32). PROVISIONAL (A4: HA-170).
    double nominalDt = 0.01;
    /// A sample's dt must be within nominalDt·(1 ± this). PROVISIONAL (A4: HA-170).
    double dtTolerance = 0.10;
    /// |speed| under this (in/s) is the static-friction dead band. PROVISIONAL (A4: HA-170).
    double minSpeed = 1.0;
    /// A member current at or over this (A) is the current limit: 2.3 A, under the PROS default
    /// limit of 2.5 A (HA-172). PROVISIONAL (A4: HA-170).
    double currentGuard = 2.3;
    /// τMin as a fraction of nominalDt: a time constant under τMin is below the fit's resolution
    /// and kA is reported as exactly 0 (header). 0.2 = dt/5. PROVISIONAL (A4: HA-170).
    double tauMinFraction = 0.2;
    /// Fewer admitted samples than this is TooFewSamples. PROVISIONAL (A4: HA-170).
    std::uint32_t minSamples = 100;
    /// A residual RMS (in/s) over this is PoorFit. PROVISIONAL (A4: HA-170).
    double maxResidualRms = 3.0;
    /// kS below this (V) is Implausible (below 0 it is NonPhysical first). PROVISIONAL (A4: HA-170).
    double kSMin = 0.0;
    /// kS above this (V) is Implausible. PROVISIONAL (A4: HA-170).
    double kSMax = 4.0;
    /// The implied free speed at 12 V, (12 − kS)/kV in in/s, below this is Implausible.
    /// PROVISIONAL (A4: HA-170).
    double freeSpeedMin = 20.0;
    /// The implied free speed above this (in/s) is Implausible. PROVISIONAL (A4: HA-170).
    double freeSpeedMax = 200.0;
    /// kA below this (V·s²/in) is Implausible. PROVISIONAL (A4: HA-170).
    double kAMin = 0.0;
    /// kA above this (V·s²/in) is Implausible. PROVISIONAL (A4: HA-170).
    double kAMax = 0.2;

    /// The α under which kA is below resolution: e^(−1/tauMinFraction) (header).
    [[nodiscard]] double alphaMin() const noexcept { return std::exp(-1.0 / tauMinFraction); }

    /// True when every field is finite and in its domain: nominalDt, minSpeed, currentGuard,
    /// tauMinFraction and maxResidualRms > 0; dtTolerance in (0, 1); minSamples >= 3 (three unknowns);
    /// each range's low end at or under its high end. Otherwise false, with the first offending field
    /// named in `why` (NUL-terminated, cut to `cap`; `why` may be null).
    [[nodiscard]] bool valid(char* why, std::size_t cap) const {
        auto fail = [&](const char* text) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "%s", text);
            }
            return false;
        };
        const double all[] = {nominalDt,      dtTolerance, minSpeed,     currentGuard,
                              tauMinFraction, maxResidualRms, kSMin,     kSMax,
                              freeSpeedMin,   freeSpeedMax, kAMin,       kAMax};
        for (double v : all) {
            if (!std::isfinite(v)) {
                return fail("FitConfig: every field must be finite");
            }
        }
        // Each check names its field; the first that fails is the one reported.
        struct Rule {
            bool ok;
            const char* text;
        };
        const Rule rules[] = {
            {nominalDt > 0.0, "FitConfig: nominalDt must be > 0"},
            {dtTolerance > 0.0 && dtTolerance < 1.0, "FitConfig: dtTolerance must be in (0, 1)"},
            {minSpeed > 0.0, "FitConfig: minSpeed must be > 0"},
            {currentGuard > 0.0, "FitConfig: currentGuard must be > 0"},
            {tauMinFraction > 0.0, "FitConfig: tauMinFraction must be > 0"},
            {minSamples >= 3, "FitConfig: minSamples must be >= 3 (three unknowns)"},
            {maxResidualRms > 0.0, "FitConfig: maxResidualRms must be > 0"},
            {kSMin <= kSMax, "FitConfig: kSMin must be <= kSMax"},
            {freeSpeedMin <= freeSpeedMax, "FitConfig: freeSpeedMin must be <= freeSpeedMax"},
            {kAMin <= kAMax, "FitConfig: kAMin must be <= kAMax"},
        };
        for (const Rule& r : rules) {
            if (!r.ok) {
                return fail(r.text);
            }
        }
        return true;
    }
};

/// Everything a fit concluded — filled as far as the verdict let it get, so a refused fit still
/// shows its numbers on the serial line.
struct FitResult {
    /// The verdict (FitVerdict). Only Ok means the gains may be used.
    FitVerdict verdict = FitVerdict::TooFewSamples;
    /// kS (V), kV (V·s/in), kA (V·s²/in) — the fit's, when it got as far as computing them (every
    /// verdict from SignMismatch on; zeros before). kA is RECORDED, NOT APPLIED (header).
    control::FeedforwardGains gains{};
    /// True when α was under FitConfig::alphaMin() and kA is therefore reported as exactly 0.
    bool kABelowResolution = false;
    /// How many samples were admitted.
    std::uint32_t samples = 0;
    /// How many were excluded, by reason: index = FitExclusion value − 1.
    std::array<std::uint32_t, kFitExclusionReasons> excluded{};
    /// The root-mean-square of the admitted samples' residuals, in/s (divisor: the sample count).
    double residualRms = 0.0;
    /// The fitted α (0 until a solution exists).
    double alpha = 0.0;
    /// The fitted β.
    double beta = 0.0;
    /// The fitted γ.
    double gamma = 0.0;
    /// The smallest |volts| of any admitted sample — the low end of the range the gains were
    /// fitted over (0 with none admitted).
    double voltsMin = 0.0;
    /// The largest |volts| of any admitted sample — the high end of that range (HA-171: the gains
    /// must hold outside it).
    double voltsMax = 0.0;
};

/// Is `gains` inside FitConfig's ranges — kS in [kSMin, kSMax], the implied free speed at 12 V
/// (hal::kMaxMotorVoltage − kS)/kV in [freeSpeedMin, freeSpeedMax], kA in [kAMin, kAMax]? Every edge
/// is INSIDE. False names the first range missed in `why` (NUL-terminated, cut to `cap`; may be
/// null), with the value — ASCII, no '%'. Precondition: gains.kV > 0 (the fit's verdicts before
/// this one guarantee it).
[[nodiscard]] inline bool gainsPlausible(const control::FeedforwardGains& gains, const FitConfig& cfg,
                                         char* why, std::size_t cap) {
    SHULIB_PRECONDITION(gains.kV > 0.0, "gainsPlausible: kV must be > 0");
    auto say = [&](const char* name, double value, double lo, double hi) {
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap, "%s %.6g is outside [%.6g, %.6g]", name, value, lo, hi);
        }
        return false;
    };
    if (!(gains.kS >= cfg.kSMin && gains.kS <= cfg.kSMax)) {
        return say("kS", gains.kS, cfg.kSMin, cfg.kSMax);
    }
    const double freeSpeed = (hal::kMaxMotorVoltage.value() - gains.kS) / gains.kV;
    if (!(freeSpeed >= cfg.freeSpeedMin && freeSpeed <= cfg.freeSpeedMax)) {
        return say("free speed at 12 V (in/s)", freeSpeed, cfg.freeSpeedMin, cfg.freeSpeedMax);
    }
    if (!(gains.kA >= cfg.kAMin && gains.kA <= cfg.kAMax)) {
        return say("kA", gains.kA, cfg.kAMin, cfg.kAMax);
    }
    return true;
}

/// The online least-squares fit of ONE drive side (header). Feed add() once per tick; read result()
/// whenever — it does not consume anything. Copyable (it is plain numbers); reset() by assigning a
/// new one.
class FeedforwardFit {
public:
    /// Copies `config`. Precondition: config.valid().
    explicit FeedforwardFit(const FitConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(cfg_.valid(nullptr, 0), "FeedforwardFit: the FitConfig is invalid");
    }

    /// Judge one sample and, when it is admitted, rotate it into the fit. Returns FitExclusion::None
    /// when admitted, else the reason it was not (and counts it). Never throws on a bad sample: a
    /// non-finite one is an exclusion, not a precondition, because it comes off a sensor.
    FitExclusion add(const FitSample& s) {
        const FitExclusion why = judge(s);
        if (why != FitExclusion::None) {
            ++excluded_[static_cast<std::size_t>(why) - 1];
            return why;
        }
        const double sgn = s.speedBefore > 0.0 ? 1.0 : -1.0;  // |v| >= minSpeed > 0 here
        double row[3] = {s.speedBefore, s.volts, sgn};
        double y = s.speedAfter;
        for (std::size_t c = 0; c < 3; ++c) {
            colSq_[c] += row[c] * row[c];
        }
        // Rotate the row into R (Givens, column by column); what is left of y afterwards is the
        // part of this sample the columns cannot explain — its square is this sample's share of
        // the residual sum (header: sequential QR).
        for (std::size_t i = 0; i < 3; ++i) {
            if (row[i] == 0.0) {
                continue;
            }
            const double r = std::hypot(r_[i][i], row[i]);
            const double c = r_[i][i] / r;
            const double sn = row[i] / r;
            r_[i][i] = r;
            for (std::size_t j = i + 1; j < 3; ++j) {
                const double t = r_[i][j];
                r_[i][j] = c * t + sn * row[j];
                row[j] = c * row[j] - sn * t;
            }
            const double t = z_[i];
            z_[i] = c * t + sn * y;
            y = c * y - sn * t;
        }
        rss_ += y * y;
        const double mag = std::abs(s.volts);
        if (n_ == 0 || mag < voltsMin_) {
            voltsMin_ = mag;
        }
        if (n_ == 0 || mag > voltsMax_) {
            voltsMax_ = mag;
        }
        ++n_;
        return FitExclusion::None;
    }

    /// The fit as it stands (header: the verdicts in order). Pure — callable any number of times.
    [[nodiscard]] FitResult result() const {
        FitResult out;
        out.samples = n_;
        for (std::size_t i = 0; i < kFitExclusionReasons; ++i) {
            out.excluded[i] = excluded_[i];
        }
        out.voltsMin = voltsMin_;
        out.voltsMax = voltsMax_;
        out.residualRms = n_ > 0 ? std::sqrt(rss_ / static_cast<double>(n_)) : 0.0;
        if (n_ < cfg_.minSamples) {
            out.verdict = FitVerdict::TooFewSamples;
            return out;
        }
        for (std::size_t i = 0; i < 3; ++i) {
            const double norm = std::sqrt(colSq_[i]);
            if (!(norm > 0.0) || !(std::abs(r_[i][i]) > kFitRankTolerance * norm)) {
                out.verdict = FitVerdict::NotExcited;
                return out;
            }
        }
        // Back substitution: R·θ = z.
        double theta[3];
        for (std::size_t k = 3; k-- > 0;) {
            double acc = z_[k];
            for (std::size_t j = k + 1; j < 3; ++j) {
                acc -= r_[k][j] * theta[j];
            }
            theta[k] = acc / r_[k][k];
        }
        out.alpha = theta[0];
        out.beta = theta[1];
        out.gamma = theta[2];
        if (!std::isfinite(out.alpha) || !std::isfinite(out.beta) || !std::isfinite(out.gamma)) {
            out.verdict = FitVerdict::NotExcited;  // unreachable past the rank check; kept honest
            return out;
        }
        if (!(out.beta > 0.0)) {
            out.verdict = FitVerdict::SignMismatch;
            return out;
        }
        out.gains.kV = (1.0 - out.alpha) / out.beta;
        out.gains.kS = -out.gamma / out.beta;
        if (out.alpha < cfg_.alphaMin()) {
            out.gains.kA = 0.0;  // below resolution (header) — covers alpha <= 0
            out.kABelowResolution = true;
        } else if (out.alpha < 1.0) {
            out.gains.kA = -out.gains.kV * cfg_.nominalDt / std::log(out.alpha);
        }
        if (!(out.alpha < 1.0) || !(out.gains.kS >= 0.0)) {
            out.verdict = FitVerdict::NonPhysical;
            return out;
        }
        if (out.residualRms > cfg_.maxResidualRms) {
            out.verdict = FitVerdict::PoorFit;
            return out;
        }
        if (!gainsPlausible(out.gains, cfg_, nullptr, 0)) {
            out.verdict = FitVerdict::Implausible;
            return out;
        }
        out.verdict = FitVerdict::Ok;
        return out;
    }

    /// How many samples have been admitted.
    [[nodiscard]] std::uint32_t admitted() const noexcept { return n_; }

    /// How many samples were excluded for `reason` (0 for FitExclusion::None).
    [[nodiscard]] std::uint32_t excluded(FitExclusion reason) const noexcept {
        return reason == FitExclusion::None ? 0U : excluded_[static_cast<std::size_t>(reason) - 1];
    }

    /// The config in force.
    [[nodiscard]] const FitConfig& config() const noexcept { return cfg_; }

private:
    /// The admission rules of the header, in order.
    [[nodiscard]] FitExclusion judge(const FitSample& s) const noexcept {
        if (!std::isfinite(s.speedBefore) || !std::isfinite(s.volts) || !std::isfinite(s.speedAfter)
            || !std::isfinite(s.dt) || !std::isfinite(s.maxCurrent)) {
            return FitExclusion::NonFinite;
        }
        if (!(std::abs(s.dt - cfg_.nominalDt) <= cfg_.dtTolerance * cfg_.nominalDt)) {
            return FitExclusion::OffNominalDt;
        }
        if (s.volts == 0.0) {
            return FitExclusion::Stop;
        }
        if (std::abs(s.speedBefore) < cfg_.minSpeed || std::abs(s.speedAfter) < cfg_.minSpeed) {
            return FitExclusion::DeadBand;
        }
        if (s.maxCurrent >= cfg_.currentGuard) {
            return FitExclusion::OverCurrent;
        }
        return FitExclusion::None;
    }

    FitConfig cfg_;
    double r_[3][3]{};
    double z_[3]{};
    double colSq_[3]{};
    double rss_ = 0.0;
    std::uint32_t n_ = 0;
    std::uint32_t excluded_[kFitExclusionReasons]{};
    double voltsMin_ = 0.0;
    double voltsMax_ = 0.0;
};

/// A short, stable, ASCII name for a verdict — the serial record's word and a test's message.
[[nodiscard]] constexpr const char* fitVerdictName(FitVerdict v) noexcept {
    switch (v) {
        case FitVerdict::Ok: return "OK";
        case FitVerdict::TooFewSamples: return "TOO_FEW_SAMPLES";
        case FitVerdict::NotExcited: return "NOT_EXCITED";
        case FitVerdict::SignMismatch: return "SIGN_MISMATCH";
        case FitVerdict::NonPhysical: return "NON_PHYSICAL";
        case FitVerdict::Implausible: return "IMPLAUSIBLE";
        case FitVerdict::PoorFit: return "POOR_FIT";
    }
    return "?";
}

/// A short, stable, ASCII name for an exclusion reason — a column of the per-segment serial line.
[[nodiscard]] constexpr const char* fitExclusionName(FitExclusion e) noexcept {
    switch (e) {
        case FitExclusion::None: return "admitted";
        case FitExclusion::NonFinite: return "nonfinite";
        case FitExclusion::OffNominalDt: return "dt";
        case FitExclusion::Stop: return "stop";
        case FitExclusion::DeadBand: return "deadband";
        case FitExclusion::OverCurrent: return "current";
    }
    return "?";
}

}  // namespace shulib::calibration
