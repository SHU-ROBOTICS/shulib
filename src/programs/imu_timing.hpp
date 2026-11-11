#pragma once
// imu_timing.hpp — the PURE half of Bench Tests' STAGE 2c, "the IMU's timing" (IMU1 Part B,
// 2026-11-03; the IMU1 brief's D3 (a), §1 item 5, re-checks R10 and R18).
//
// ═══ WHAT STAGE 2c MEASURES, AND WHY ══════════════════════════════════════════════════
// IMU1 Part A made `ProsImu::yawRate()` keep its answer for 5 ms (`kRateHoldUs`), so every
// reader in one control tick gets one value. That fix, and the residual it leaves, rest on three
// beliefs no robot has shown (the hardware register, A4 register HA-213, HA-214, HA-215), and the
// gyro's own rate — the cure for the residual — waits on two more (HA-04, its sign; HA-109, its
// units). STAGE 2c, button 2's last stage (src/programs/bench_r3a.cpp, imuTimingStage()), is the
// first look at all five, while someone turns the robot by hand:
//   2c.1 CADENCE  — a tight loop reads the raw rotation for 2 s and keeps the instants it
//                  CHANGES; HA-213 holds if the intervals cluster at 10 000 µs (PROS's documented
//                  "copied into the shared memory buffer only at 10ms intervals").
//   2c.2 PHASE    — 100 ticks paced by delay_until(10 ms); each polls the rotation from its wake
//                  until it changes (at most 9.5 ms) and keeps that offset; HA-214's phase is the
//                  offset (first / median / last, and its spread) and its drift per second, with
//                  the drift's own uncertainty — a wake's jitter enters every offset, and a drift
//                  the jitter could make is never called a drift (the IMU1-B review, 2026-11-05).
//   2c.3 SPAN     — a stand-in for a motion tick's body between its two IMU reads, BY PLAN (R10):
//                  the drive motors' reads, the battery and one serial line, 100 paced ticks; a
//                  LOWER bound on a real tick's span (no Localizer arithmetic, no record build, no
//                  motor command), judged against the 5 ms hold — HA-215.
//   2c.4 GYRO     — a tight loop for 3 s: the rotation's change between the FIRST and LAST change
//                  instants it sees, beside the gyro's z integrated (trapezoid on micros() stamps)
//                  over the same window — both RAW PROS degrees, never ProsImu::yawRate(), which
//                  answers canonical CCW rad/s and, held for 5 ms, refreshes at most every 5 ms in a
//                  tight loop (R18). Same sign: z shares get_rotation()'s convention (HA-04); the
//                  two within 10 %: z is deg/s about the yaw axis (HA-109).
//
// ═══ WHY THIS FILE EXISTS ═════════════════════════════════════════════════════════════
// The stage itself reads PROS and cannot run on the host. Everything it CONCLUDES is arithmetic
// on numbers it collected — the interval statistics, the phase from the change instants, the
// trapezoid integral, the verdicts — and that arithmetic is what decides whether a belief prints
// HOLDS or WRONG. So it lives here, PROS-free and allocation-free (the bench_r3a.hpp and
// auton_tester_run.hpp precedent), where the host suite tries to break it
// (test/bench_imu_timing_test.cpp, which also pins this file PROS-free and the stage's own code by
// source). So do the stage's four COLLECTING LOOPS (runCadence, runPhase, runSpans, runGyro, at the
// end): templates over the clock, the wait and the reads, which the stage instantiates with
// pros::micros(), delay_until and the C API's reads, and the host suite with a simulated clock and
// a model of the IMU — so the host drives the stage's real loops, not a copy of them (the IMU1-B
// review, 2026-11-05). It includes nothing of shulib and nothing of PROS, so it pulls neither the
// motion scheduler nor a Localizer into Bench Tests (R10's concern).
//
// ═══ WHAT IT CANNOT KNOW ══════════════════════════════════════════════════════════════
// Nothing here has run on a robot. A verdict is only as good as the hand turn behind it: a robot
// held still shows no copies at all, so "no change" is counted and printed, never guessed
// (the brief's landmine 15), and a short or small turn earns no gyro verdict. An IMU that answers
// PROS_ERR_F on EVERY read (a cable worked loose) is its own verdict, never "no change" — a dead
// device is not a finding about PROS's copy. The thresholds below
// are plain choices, each said where it is declared; the worksheet's Station N reads the printed
// numbers, not only the words.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace shulib::bench::imu_timing {

// ── the numbers the verdicts use ─────────────────────────────────────────────────────

/// PROS's documented copy period (HA-213): the IMU's readings reach the brain's shared buffer every
/// 10 ms. The phase part unwraps its offsets at this period.
inline constexpr std::uint32_t kCopyPeriodUs = 10000;

/// An interval within this of kCopyPeriodUs counts as "one 10 ms copy" (9 000..11 000 µs, ends
/// included), and the cadence verdict asks the MEDIAN interval to lie in the same band. A choice:
/// wide enough for a poll's few microseconds and a copy's jitter, narrow enough that a 5 ms or a
/// 20 ms copy can never pass.
inline constexpr std::uint32_t kBandUs = 1000;

/// The fewest change intervals the cadence verdict judges (a fifth of a second of copies); fewer
/// is "turn it steadily and re-run", never a verdict.
inline constexpr std::size_t kMinIntervals = 20;

/// How long the phase part polls after each wake before it gives the tick up as "no change seen"
/// (the brief's 9.5 ms: the next wake is 10 ms after this one, and a poll must not make it late).
inline constexpr std::uint32_t kPhaseWindowUs = 9500;

/// The fewest phase ticks with a change the phase verdict judges.
inline constexpr std::size_t kMinPhaseTicks = 20;

/// A drift no larger than this (µs of offset per second, either way) reads as a FIXED phase over the
/// stage's one second. A choice: at this drift the phase would take ~1000 s to cross a whole 10 ms
/// tick, longer than any match. The wake jitter is NOT assumed small (it is unmeasured — STAGE 5
/// has never run): see kDriftSigmas.
inline constexpr double kFixedDriftUsPerS = 10.0;

/// How many standard errors of the least-squares slope a drift must clear. Each offset is measured
/// from the ACTUAL wake, so a wake's jitter enters it one for one, and over 1 s of ticks the slope
/// inherits that jitter (its standard error is the residual spread over √Sxx). So: DRIFTING only
/// when |drift| > kFixedDriftUsPerS AND |drift| > kDriftSigmas · SE; FIXED only when
/// |drift| ≤ kFixedDriftUsPerS AND kDriftSigmas · SE ≤ kFixedDriftUsPerS (the data can tell a
/// 10 µs/s drift from none); anything else is "not separable from the wake jitter" — no verdict.
/// A choice: 3 standard errors, a ~0.3 % chance that jitter alone reads as a drift.
inline constexpr double kDriftSigmas = 3.0;

/// `ProsImu::kRateHoldUs` (IMU1 Part A), restated here because this file includes no shulib header;
/// bench_r3a.cpp static_asserts the two equal, so the span verdict moves with the hold or the
/// bench build fails.
inline constexpr std::uint32_t kHoldUs = 5000;

/// The span verdict: a stand-in tick whose LONGEST span is under half the hold is "well under";
/// the other half is left for what the stand-in leaves out (the Localizer's arithmetic, the record's
/// build, the motor commands). A choice, stated on the line the stage prints.
inline constexpr std::uint32_t kSpanWellUnderUs = kHoldUs / 2;

/// The gyro verdict needs the first and last change instants at least this far apart, so the 10 ms
/// copy's quantization at each end is under 0.5 % of the window (R18).
inline constexpr std::uint64_t kGyroMinWindowUs = 2000000;

/// ... and the rotation to have changed by at least this much (raw degrees, either way), so a
/// gyro's bias over the window is a small share of the turn.
inline constexpr double kGyroMinTurnDeg = 30.0;

/// The two angles "agree" (HA-109) when their MAGNITUDES differ by no more than this share of the
/// rotation's change: | |∫z| − |Δrotation| | ≤ 0.10 × |Δrotation| (the brief's "within ~10 %").
inline constexpr double kGyroTolerance = 0.10;

/// One degree in radians: an integrated z within kGyroTolerance of this share of the rotation's
/// change reads as a gyro in RADIANS per second about the yaw axis — judgeGyro's RadiansSameSign /
/// RadiansOppositeSign verdicts: HA-109 WRONG on its units, with the sign (HA-04) still judged.
inline constexpr double kRadPerDeg = 3.14159265358979323846 / 180.0;

// ── a summary of measured microseconds ───────────────────────────────────────────────

/// The count, smallest, LOWER median and largest of a set of microsecond values; all zero when the
/// set is empty. The lower median (element (n − 1) / 2 of the sorted set) is always a value that
/// was measured, never an average of two.
struct Summary {
    std::size_t n = 0;
    std::uint32_t min = 0;
    std::uint32_t median = 0;
    std::uint32_t max = 0;
};

/// Summarizes `values`, SORTING THEM IN PLACE (the stage hands it a scratch copy).
[[nodiscard]] inline Summary summarize(std::span<std::uint32_t> values) noexcept {
    Summary s;
    s.n = values.size();
    if (s.n == 0) return s;
    std::sort(values.begin(), values.end());
    s.min = values.front();
    s.median = values[(s.n - 1) / 2];
    s.max = values.back();
    return s;
}

/// A microsecond difference as the 32-bit value the summaries keep: 0 when `later` is not after
/// `earlier` (micros() is monotonic — HA-101 — so only a broken caller gets here), and the largest
/// uint32 for a gap over ~71 minutes, which no stage loop can produce.
[[nodiscard]] inline std::uint32_t usBetween(std::uint64_t earlier, std::uint64_t later) noexcept {
    if (later <= earlier) return 0;
    const std::uint64_t d = later - earlier;
    return d > std::numeric_limits<std::uint32_t>::max() ? std::numeric_limits<std::uint32_t>::max()
                                                         : static_cast<std::uint32_t>(d);
}

// ── seeing a copy: a change of the raw value ─────────────────────────────────────────

/// Feeds one raw read at a time and says when the value CHANGED. Between two copies every read
/// returns the SAME double (HA-213's sample-and-hold), so a change is an exact inequality — no
/// tolerance, on purpose: a tolerance would hide a copy that moved the value by less. A non-finite
/// read (PROS_ERR_F: unplugged, still calibrating) is counted and skipped — it neither changes nor
/// becomes the value later reads are compared with. The first finite read is the baseline, not a
/// change.
class ChangeWatch {
public:
    /// One read; true when it is a change.
    bool feed(double value) noexcept {
        ++polls_;
        if (!std::isfinite(value)) {
            ++badReads_;
            return false;
        }
        if (!haveBaseline_) {
            haveBaseline_ = true;
            last_ = value;
            return false;
        }
        if (value == last_) return false;  // exact on purpose: the same copy is the same double
        last_ = value;
        ++changes_;
        return true;
    }
    [[nodiscard]] std::size_t polls() const noexcept { return polls_; }
    [[nodiscard]] std::size_t badReads() const noexcept { return badReads_; }
    [[nodiscard]] std::size_t changes() const noexcept { return changes_; }

private:
    std::size_t polls_ = 0;
    std::size_t badReads_ = 0;
    std::size_t changes_ = 0;
    bool haveBaseline_ = false;
    double last_ = 0.0;
};

// ── 2c.1 the cadence (HA-213) ─────────────────────────────────────────────────────────

/// What the change instants of one tight loop say about the copy's cadence.
struct Cadence {
    Summary interval;           ///< over the intervals between consecutive change instants
    std::size_t inBand = 0;     ///< intervals within kBandUs of kCopyPeriodUs (ends included)
    std::size_t below = 0;      ///< intervals shorter than the band
    std::size_t above = 0;      ///< intervals longer than the band (a copy that did not move the value, a pause)
    double meanInBandUs = 0.0;  ///< the in-band intervals' mean — a precise period; 0 when none
};

/// The cadence of `changeAtUs` (the change instants, in the order seen). The intervals are written
/// to `scratch` (and sorted there for the median); only the first scratch.size() of them are kept
/// when it is smaller than changeAtUs.size() − 1, and Summary::n says how many were.
[[nodiscard]] inline Cadence cadenceFrom(std::span<const std::uint64_t> changeAtUs,
                                         std::span<std::uint32_t> scratch) noexcept {
    Cadence c;
    if (changeAtUs.size() < 2) return c;
    const std::size_t n = std::min(changeAtUs.size() - 1, scratch.size());
    double inBandSum = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint32_t iv = usBetween(changeAtUs[i], changeAtUs[i + 1]);
        scratch[i] = iv;
        if (iv < kCopyPeriodUs - kBandUs) {
            ++c.below;
        } else if (iv > kCopyPeriodUs + kBandUs) {
            ++c.above;
        } else {
            ++c.inBand;
            inBandSum += static_cast<double>(iv);
        }
    }
    if (c.inBand > 0) c.meanInBandUs = inBandSum / static_cast<double>(c.inBand);
    c.interval = summarize(scratch.first(n));
    return c;
}

/// HA-213's verdict.
enum class CadenceVerdict {
    NoReads,   ///< not one finite read (every read PROS_ERR_F): the IMU's cable or port — no finding
    NoChange,  ///< no change at all: the robot was still (or no copy reached the loop) — turn it
    TooFew,    ///< under kMinIntervals intervals (ONE change is none): turn it steadily and re-run
    Holds,     ///< the median interval within the band: the 10 ms copy
    Faster,    ///< the median below the band: the copy is faster than 10 ms (IMU1's hold still right)
    Slower,    ///< the median above it: slower than 10 ms — or the robot was not turning most of the time
};

/// HA-213's verdict from the loop's reads (`watch`) and the intervals between its changes. The
/// reads come first: a device that answered nothing but PROS_ERR_F is NoReads, never NoChange.
[[nodiscard]] inline CadenceVerdict judgeCadence(const ChangeWatch& watch,
                                                 const Summary& intervals) noexcept {
    if (watch.badReads() >= watch.polls()) return CadenceVerdict::NoReads;
    if (watch.changes() == 0) return CadenceVerdict::NoChange;
    if (intervals.n < kMinIntervals) return CadenceVerdict::TooFew;
    if (intervals.median < kCopyPeriodUs - kBandUs) return CadenceVerdict::Faster;
    if (intervals.median > kCopyPeriodUs + kBandUs) return CadenceVerdict::Slower;
    return CadenceVerdict::Holds;
}

// ── 2c.2 the phase (HA-214) ───────────────────────────────────────────────────────────

/// One paced tick of the phase part: its wake instant and the offset from it at which the raw
/// rotation first changed; a NEGATIVE offset means no change was seen within kPhaseWindowUs.
struct TickPhase {
    std::uint64_t wakeUs = 0;
    std::int32_t offsetUs = -1;
};

/// `offset` moved by whole periods so that its step from `previous` lies in (−period/2, +period/2]:
/// the copy that drifted past a wake comes back as an offset 10 ms later or earlier, not as a jump.
/// A zero period returns `offset` unchanged.
[[nodiscard]] inline std::int64_t unwrapNear(std::int64_t offset, std::int64_t previous,
                                             std::uint32_t periodUs) noexcept {
    if (periodUs == 0) return offset;
    const std::int64_t p = periodUs;
    std::int64_t step = (offset - previous) % p;  // (−p, p)
    if (2 * step > p) {
        step -= p;
    } else if (2 * step <= -p) {
        step += p;
    }
    return previous + step;
}

/// What the phase part's ticks say.
struct Phase {
    std::size_t ticks = 0;       ///< ticks run
    std::size_t seen = 0;        ///< ticks with a change inside the window
    std::size_t missed = 0;      ///< ticks with none
    std::int32_t firstUs = 0;    ///< the first seen tick's offset (raw, as measured)
    std::uint32_t medianUs = 0;  ///< the kept seen ticks' lower-median offset (raw)
    std::uint32_t minUs = 0;     ///< the kept seen ticks' smallest offset (raw) — the spread's low end
    std::uint32_t maxUs = 0;     ///< ... and largest (raw)
    std::int32_t lastUs = 0;     ///< the last seen tick's offset (raw)
    bool driftValid = false;     ///< two or more seen ticks at different wake instants
    double driftUsPerS = 0.0;    ///< least-squares slope of the UNWRAPPED offset against the wake time
    bool seValid = false;        ///< driftValid with three or more seen ticks (n − 2 ≥ 1)
    double residualSdUs = 0.0;   ///< the offsets' spread about the fitted line (µs) — the wake jitter, mostly
    double driftSeUsPerS = 0.0;  ///< the slope's standard error: residualSdUs / √Sxx
};

/// The phase of `ticks` (in the order run), unwrapped at `periodUs`. The seen ticks' raw offsets go
/// to `scratch` for the median, min and max (the first scratch.size() of them when it is smaller —
/// never a write past it). The drift is a least-squares slope — every seen tick counts, so one
/// jittered wake cannot make it — and its standard error says how much of it the jitter could make.
[[nodiscard]] inline Phase phaseFrom(std::span<const TickPhase> ticks, std::uint32_t periodUs,
                                     std::span<std::uint32_t> scratch) noexcept {
    Phase ph;
    ph.ticks = ticks.size();
    std::size_t kept = 0;
    std::uint64_t wake0 = 0;
    std::int64_t prev = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    for (const TickPhase& t : ticks) {
        if (t.offsetUs < 0) {
            ++ph.missed;
            continue;
        }
        if (ph.seen == 0) {
            ph.firstUs = t.offsetUs;
            wake0 = t.wakeUs;
            prev = t.offsetUs;
        }
        ph.lastUs = t.offsetUs;
        if (kept < scratch.size()) scratch[kept++] = static_cast<std::uint32_t>(t.offsetUs);
        const std::int64_t y = unwrapNear(t.offsetUs, prev, periodUs);
        prev = y;
        sumX += static_cast<double>(usBetween(wake0, t.wakeUs)) * 1e-6;
        sumY += static_cast<double>(y);
        ++ph.seen;
    }
    const Summary offsets = summarize(scratch.first(kept));
    ph.medianUs = offsets.median;
    ph.minUs = offsets.min;
    ph.maxUs = offsets.max;
    if (ph.seen < 2) return ph;
    // The second pass, centred (the numerically quiet form): the same unwrap, replayed.
    const double n = static_cast<double>(ph.seen);
    const double meanX = sumX / n;
    const double meanY = sumY / n;
    double sxx = 0.0;
    double sxy = 0.0;
    double syy = 0.0;
    bool first = true;
    for (const TickPhase& t : ticks) {
        if (t.offsetUs < 0) continue;
        const std::int64_t y = first ? static_cast<std::int64_t>(t.offsetUs)
                                     : unwrapNear(t.offsetUs, prev, periodUs);
        first = false;
        prev = y;
        const double dx = static_cast<double>(usBetween(wake0, t.wakeUs)) * 1e-6 - meanX;
        const double dy = static_cast<double>(y) - meanY;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }
    if (sxx > 0.0) {
        ph.driftValid = true;
        ph.driftUsPerS = sxy / sxx;
        if (ph.seen >= 3) {
            // The residual sum of squares, Syy − Sxy²/Sxx — clamped at 0 against rounding on a
            // perfectly straight line — over n − 2 degrees of freedom.
            const double ssr = std::max(0.0, syy - sxy * sxy / sxx);
            const double variance = ssr / (n - 2.0);
            ph.seValid = true;
            ph.residualSdUs = std::sqrt(variance);
            ph.driftSeUsPerS = std::sqrt(variance / sxx);
        }
    }
    return ph;
}

/// HA-214's verdict.
enum class PhaseVerdict {
    NoCadence,  ///< 2c.1 did not find the 10 ms copy, so offsets unwrapped at 10 ms mean nothing
    NoChange,   ///< no tick saw a change: the robot was still, or the copy lands in the last
                ///< half-millisecond before every wake (or at it)
    TooFew,     ///< under kMinPhaseTicks seen ticks
    Fixed,      ///< |drift| ≤ kFixedDriftUsPerS, and the jitter's kDriftSigmas · SE ≤ it too
    Drifting,   ///< |drift| > kFixedDriftUsPerS and > kDriftSigmas · SE: the copy's phase moves against
                ///< the wake, so every boot passes the bad band
    NotSeparable,  ///< neither: the wakes jitter too much to tell, in one second, a drift from none
};

static_assert(kMinPhaseTicks >= 3, "a judged phase needs a standard error: n - 2 >= 1");

[[nodiscard]] inline PhaseVerdict judgePhase(const Phase& ph, CadenceVerdict cadence) noexcept {
    if (cadence != CadenceVerdict::Holds) return PhaseVerdict::NoCadence;
    if (ph.seen == 0) return PhaseVerdict::NoChange;
    if (ph.seen < kMinPhaseTicks || !ph.driftValid || !ph.seValid) return PhaseVerdict::TooFew;
    const double drift = std::fabs(ph.driftUsPerS);
    const double noise = kDriftSigmas * ph.driftSeUsPerS;
    if (drift > kFixedDriftUsPerS && drift > noise) return PhaseVerdict::Drifting;
    if (drift <= kFixedDriftUsPerS && noise <= kFixedDriftUsPerS) return PhaseVerdict::Fixed;
    return PhaseVerdict::NotSeparable;
}

/// Seconds for a phase drifting at `driftUsPerS` to cross one whole period — how often a drifting
/// boot passes through any band of the copy's phase. 0 for no drift (it never crosses).
[[nodiscard]] inline double secondsToCrossPeriod(double driftUsPerS, std::uint32_t periodUs) noexcept {
    const double d = std::fabs(driftUsPerS);
    return d > 0.0 ? static_cast<double>(periodUs) / d : 0.0;
}

// ── 2c.3 the tick span's stand-in (HA-215) ────────────────────────────────────────────

/// HA-215's reading of the stand-in — on its LONGEST span, because the hold fails on the one tick
/// that runs long, not on the typical one. Never "holds": the stand-in is a lower bound.
enum class SpanVerdict {
    NoTicks,    ///< nothing measured
    WellUnder,  ///< max < kSpanWellUnderUs: room left for what the stand-in leaves out (not proof)
    Close,      ///< kSpanWellUnderUs ≤ max < kHoldUs: a warning — the real tick is longer still
    Over,       ///< max ≥ kHoldUs: even the stand-in spanned the hold — HA-215 is wrong on this brain
};

[[nodiscard]] inline SpanVerdict judgeSpan(const Summary& spans) noexcept {
    if (spans.n == 0) return SpanVerdict::NoTicks;
    if (spans.max < kSpanWellUnderUs) return SpanVerdict::WellUnder;
    if (spans.max < kHoldUs) return SpanVerdict::Close;
    return SpanVerdict::Over;
}

// ── 2c.4 the gyro beside the rotation (HA-04, HA-109) ─────────────────────────────────

/// Feeds the gyro part's tight loop one poll at a time: the instant (micros(), read BEFORE the
/// device, as ProsImu reads its clock — IMU1 Part A), the raw rotation and the raw gyro z. It
/// integrates z by the trapezoid between consecutive good polls, and keeps the rotation and the
/// integral at the FIRST and the LAST change of the rotation — the window R18 compares over, so
/// both angles start and end on a copy. A poll is DROPPED whole, and counted, when either value is
/// non-finite (PROS_ERR_F) or its instant is BEFORE the previous good poll's (micros() is monotonic
/// — HA-101 — so only a broken clock: its z must never bridge a gap); the next good poll's
/// trapezoid spans the gap. Two polls in the same microsecond add no area between them.
class GyroTurn {
public:
    void feed(std::uint64_t tUs, double rotationDeg, double zDegPerS) noexcept {
        ++polls_;
        if (!std::isfinite(rotationDeg) || !std::isfinite(zDegPerS) || (havePrev_ && tUs < prevT_)) {
            ++badReads_;
            return;
        }
        if (havePrev_) {
            integral_ += 0.5 * (prevZ_ + zDegPerS) * static_cast<double>(tUs - prevT_) * 1e-6;
        }
        havePrev_ = true;
        prevT_ = tUs;
        prevZ_ = zDegPerS;
        if (!haveBaseline_) {
            haveBaseline_ = true;
            lastRot_ = rotationDeg;
            return;
        }
        if (rotationDeg == lastRot_) return;  // exact, as ChangeWatch: the same copy is the same double
        const double step = rotationDeg - lastRot_;
        lastRot_ = rotationDeg;
        ++changes_;
        if (changes_ == 1) {
            firstT_ = tUs;
            firstRot_ = rotationDeg;
            firstIntegral_ = integral_;
        } else {
            // Travel INSIDE the window only, so rising − falling is the window's change.
            if (step > 0.0) {
                rising_ += step;
            } else {
                falling_ -= step;
            }
        }
        lastT_ = tUs;
        lastChangeRot_ = rotationDeg;
        lastIntegral_ = integral_;
    }

    [[nodiscard]] std::size_t polls() const noexcept { return polls_; }
    /// Polls dropped: a non-finite value, or an instant before the previous good poll's.
    [[nodiscard]] std::size_t badReads() const noexcept { return badReads_; }
    [[nodiscard]] std::size_t changes() const noexcept { return changes_; }
    /// The last change instant minus the first; 0 with fewer than two changes.
    [[nodiscard]] std::uint64_t windowUs() const noexcept {
        return changes_ >= 2 && lastT_ > firstT_ ? lastT_ - firstT_ : 0;
    }
    /// The raw rotation at the last change minus at the first (raw PROS degrees); 0 with fewer than two.
    [[nodiscard]] double rotationChangeDeg() const noexcept {
        return changes_ >= 2 ? lastChangeRot_ - firstRot_ : 0.0;
    }
    /// ∫ z dt between the same two instants (raw z, integrated: degrees if z is deg/s); 0 with fewer than two.
    [[nodiscard]] double integratedZDeg() const noexcept {
        return changes_ >= 2 ? lastIntegral_ - firstIntegral_ : 0.0;
    }
    /// The rotation's travel inside the window, increasing and decreasing (raw degrees, each ≥ 0).
    [[nodiscard]] double risingDeg() const noexcept { return rising_; }
    [[nodiscard]] double fallingDeg() const noexcept { return falling_; }

private:
    std::size_t polls_ = 0;
    std::size_t badReads_ = 0;
    std::size_t changes_ = 0;
    bool havePrev_ = false;
    std::uint64_t prevT_ = 0;
    double prevZ_ = 0.0;
    double integral_ = 0.0;
    bool haveBaseline_ = false;
    double lastRot_ = 0.0;
    std::uint64_t firstT_ = 0;
    double firstRot_ = 0.0;
    double firstIntegral_ = 0.0;
    std::uint64_t lastT_ = 0;
    double lastChangeRot_ = 0.0;
    double lastIntegral_ = 0.0;
    double rising_ = 0.0;
    double falling_ = 0.0;
};

/// True when |∫z| / |Δrotation| is within kGyroTolerance of one degree's radians — z reads as
/// RADIANS per second about the yaw axis (judgeGyro's Radians* verdicts).
[[nodiscard]] inline bool looksLikeRadians(double rotationChangeDeg, double integratedZDeg) noexcept {
    const double turn = std::fabs(rotationChangeDeg);
    if (!(turn > 0.0) || !std::isfinite(integratedZDeg)) return false;
    return std::fabs(std::fabs(integratedZDeg) - kRadPerDeg * turn) <= kGyroTolerance * kRadPerDeg * turn;
}

/// HA-04's and HA-109's verdict from one turn. HA-109 (units and axis) first, from the MAGNITUDES:
/// deg/s about the yaw axis (within kGyroTolerance), rad/s about it (looksLikeRadians), or neither.
/// HA-04 (the sign) is judged whenever z is about the yaw axis — in deg/s OR rad/s, because a sign
/// does not depend on units — and NOT judged when z is neither: a z on another axis says nothing
/// about the sign ProsImu's GyroRateZ would answer (the IMU1-B review, 2026-11-05, corrected the
/// first build, which withheld the sign in the radians case too).
enum class GyroVerdict {
    NoReads,              ///< not one good poll (every poll dropped): the IMU's cable or port — no finding
    NoTurn,               ///< under two changes: the robot was not turned
    TooShort,             ///< the first and last change under kGyroMinWindowUs apart
    TooSmall,             ///< the rotation changed by under kGyroMinTurnDeg
    Agree,                ///< same sign, magnitudes within kGyroTolerance: HA-04 and HA-109 hold
    OppositeSign,         ///< magnitudes agree, signs opposite: HA-109's scale holds, HA-04 is WRONG
    RadiansSameSign,      ///< z is rad/s: HA-109 WRONG (units); same sign: HA-04 holds
    RadiansOppositeSign,  ///< z is rad/s: HA-109 WRONG (units); opposite sign: HA-04 WRONG
    ScaleOff,             ///< neither scale: HA-109 WRONG (axis, or units); HA-04 not judged
};

[[nodiscard]] inline GyroVerdict judgeGyro(std::size_t changes, std::uint64_t windowUs,
                                           double rotationChangeDeg, double integratedZDeg) noexcept {
    if (changes < 2) return GyroVerdict::NoTurn;
    if (windowUs < kGyroMinWindowUs) return GyroVerdict::TooShort;
    const double turn = std::fabs(rotationChangeDeg);
    if (!(turn >= kGyroMinTurnDeg)) return GyroVerdict::TooSmall;
    // A non-finite integral needs no guard of its own: both scale tests below are `<=` comparisons
    // (and looksLikeRadians refuses it), false for a NaN or an infinity, so it falls to ScaleOff
    // (the IMU1-B review found the first build's explicit guard unreachable by any test).
    const bool sameSign = (integratedZDeg > 0.0) == (rotationChangeDeg > 0.0);
    if (std::fabs(std::fabs(integratedZDeg) - turn) <= kGyroTolerance * turn) {
        return sameSign ? GyroVerdict::Agree : GyroVerdict::OppositeSign;
    }
    if (looksLikeRadians(rotationChangeDeg, integratedZDeg)) {
        return sameSign ? GyroVerdict::RadiansSameSign : GyroVerdict::RadiansOppositeSign;
    }
    return GyroVerdict::ScaleOff;
}

/// The same verdict from the turn itself — what the stage calls, so the four numbers cannot reach
/// the judge in the wrong order — with the reads first: a turn whose every poll was dropped is
/// NoReads, never NoTurn.
[[nodiscard]] inline GyroVerdict judgeGyro(const GyroTurn& turn) noexcept {
    if (turn.badReads() >= turn.polls()) return GyroVerdict::NoReads;
    return judgeGyro(turn.changes(), turn.windowUs(), turn.rotationChangeDeg(), turn.integratedZDeg());
}

/// STAGE 2's rule for a turn that went both ways (bench_r3a.cpp's rotate test): the smaller
/// direction's travel over a fifth of the larger's. Printed, never a reason to withhold the gyro
/// verdict — both angles come from the same motion, whichever way it went.
[[nodiscard]] inline bool mixedTurn(double risingDeg, double fallingDeg) noexcept {
    const double dominant = risingDeg > fallingDeg ? risingDeg : fallingDeg;
    const double minor = risingDeg > fallingDeg ? fallingDeg : risingDeg;
    return minor > 0.2 * dominant;
}

// ── the stage's four collecting loops ────────────────────────────────────────────────
// Templates over what only a robot has: `nowUs()` (pros::micros() there), `waitTick()` (a
// pros::Task::delay_until of the motion tick's period there) and the reads (the C API's
// imu_get_rotation and imu_get_gyro_rate().z there). Every loop reads the clock BEFORE the device,
// as ProsImu reads its clock (IMU1 Part A), and none allocates. The host suite runs these same
// loops on a simulated clock and a model of the IMU.

/// What 2c.1's loop kept: how many change instants fit `changeAt`, and how many more it saw.
struct CadenceRun {
    std::size_t stored = 0;
    std::size_t overflow = 0;
};

/// 2c.1: a tight loop of rotation reads for `durationUs`; each change instant into `changeAt` (the
/// ones past its end counted in `overflow`), every read into `watch`.
template <class NowUs, class ReadRotation>
[[nodiscard]] CadenceRun runCadence(NowUs&& nowUs, ReadRotation&& readRotation, std::uint64_t durationUs,
                                    ChangeWatch& watch, std::span<std::uint64_t> changeAt) {
    CadenceRun run;
    const std::uint64_t start = nowUs();
    for (;;) {
        const std::uint64_t t = nowUs();
        if (t - start >= durationUs) break;
        if (watch.feed(readRotation())) {
            if (run.stored < changeAt.size()) {
                changeAt[run.stored++] = t;
            } else {
                ++run.overflow;
            }
        }
    }
    return run;
}

/// 2c.2: one tick per element of `ticks`. Each waits for its wake, reads the rotation AT the wake
/// (the baseline), then polls until the value CHANGES — at most kPhaseWindowUs after the wake, so
/// the poll ends before the next 10 ms wake — and keeps the wake and the offset (−1: no change).
template <class NowUs, class WaitTick, class ReadRotation>
void runPhase(NowUs&& nowUs, WaitTick&& waitTick, ReadRotation&& readRotation,
              std::span<TickPhase> ticks) {
    for (TickPhase& tick : ticks) {
        waitTick();
        const std::uint64_t wake = nowUs();
        ChangeWatch watch;
        (void)watch.feed(readRotation());  // the value at the wake: the baseline
        std::int32_t offset = -1;
        for (;;) {
            const std::uint32_t since = usBetween(wake, nowUs());
            if (since >= kPhaseWindowUs) break;
            if (watch.feed(readRotation())) {
                offset = static_cast<std::int32_t>(since);
                break;
            }
        }
        tick = TickPhase{wake, offset};
    }
}

/// 2c.3: one stand-in tick per element of `spans`. Each waits for its wake, stamps t0, makes the
/// tick's FIRST IMU read, runs `body(index, thatRead)` (the device reads and the serial line a
/// motion tick makes between its two IMU reads), stamps t1 — the instant of the tick's SECOND IMU
/// read, taken before it as ProsImu does — and makes that read. The span is t1 − t0.
template <class NowUs, class WaitTick, class ReadImu, class Body>
void runSpans(NowUs&& nowUs, WaitTick&& waitTick, ReadImu&& readImu, Body&& body,
              std::span<std::uint32_t> spans) {
    for (std::size_t i = 0; i < spans.size(); ++i) {
        waitTick();
        const std::uint64_t t0 = nowUs();
        const double first = readImu();  // the tick's first IMU read
        body(i, first);
        const std::uint64_t t1 = nowUs();
        (void)readImu();  // the tick's second IMU read: the span ends at its instant
        spans[i] = usBetween(t0, t1);
    }
}

/// 2c.4: a tight loop for `durationUs`, every poll the instant, the raw rotation and the raw z.
template <class NowUs, class ReadRotation, class ReadZ>
void runGyro(NowUs&& nowUs, ReadRotation&& readRotation, ReadZ&& readZ, std::uint64_t durationUs,
             GyroTurn& turn) {
    const std::uint64_t start = nowUs();
    for (;;) {
        const std::uint64_t t = nowUs();
        if (t - start >= durationUs) break;
        const double rotation = readRotation();
        const double z = readZ();
        turn.feed(t, rotation, z);
    }
}

}  // namespace shulib::bench::imu_timing
