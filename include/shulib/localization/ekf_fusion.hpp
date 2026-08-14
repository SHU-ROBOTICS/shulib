#pragma once
//
// EkfFusion — the M3 fusion policy: a 5-state SE(2) extended Kalman filter behind the SAME
// `IFusionPolicy` seam `ComplementaryFusion` has occupied since M2 (master plan §8,
// "complementary → EKF"). It is the first thing in the library that can WEIGH two correctors
// against each other instead of merely bounding the damage when they disagree, and the first
// thing that carries an explicit statement of HOW WRONG IT MIGHT BE.
//
// `ComplementaryFusion` is NOT replaced. It stays as the shipped default and the fallback tier
// (build-order: "the simpler filter is easier to get right and to explain"). Both tiers are
// selectable, both are tested, and the choice is one constructor argument at the call site.
//
// ── WHAT A COVARIANCE BUYS, IN ONE PARAGRAPH ───────────────────────────────────────────────
// The complementary tier answers "how far should I move toward this fix?" with a constant times
// a confidence. It has no way to answer "how wrong am I right now?", so it cannot tell a fix
// that disagrees with a CONFIDENT estimate (probably the sensor is lying) from one that
// disagrees with a LOST estimate (probably the sensor is right). The covariance `P` is that
// missing answer, carried as a number and updated every tick: it GROWS while dead-reckoning, in
// proportion to how far the robot travelled, and SHRINKS every time a fix is folded, in
// proportion to how good that fix claimed to be. Everything else here follows from having it —
// the gate scales with it, the gain is derived from it, two disagreeing sources are combined by
// it, and it is what the estimator publishes when asked how confident it is.
//
// ── THE STATE, AND WHY EACH PIECE IS THERE ────────────────────────────────────────────────
//     x = [ px, py, θ, vx, vy ]
//   px, py   field-frame position, inches
//   θ        heading, radians — see T1 below; this is a BELIEF ABOUT the published heading,
//            not a second, competing heading
//   vx, vy   BODY-frame velocity, in/s (vx forward, vy left)
// Body-frame velocity is what makes the model genuinely nonlinear and what makes θ genuinely
// load-bearing: `p⁻ = p + R(θ)·v·dt` cannot be evaluated without a heading. A field-frame
// velocity state would have made θ decorative.
//
// ── T1 — THE SEAM SAYS A POLICY CANNOT OWN HEADING, AND IT STILL DOESN'T ───────────────────
// `i_fusion_policy.hpp` states that a policy returns the corrected POSITION only, because the
// Localizer re-stamps heading from the IMU afterward. E3 then opened the ONE sanctioned heading
// path: a bounded `FusionResult::headingNudge` that the Localizer folds into a persistent bias
// before composing `imu.heading() + bias` as the last write of the tick.
//
// This filter tracks θ and changes NEITHER of those things. What leaves here is still only
// `{x, y}` plus a bounded increment. Concretely:
//
//   * The θ TIME UPDATE is the IMU's, by construction: `θ⁻ := predicted.heading()` at the top
//     of every tick. The filter does not integrate its own heading and does not hold a rival
//     one. What it estimates is the ERROR in the IMU's answer, and that estimate leaves as an
//     increment, exactly as E3 designed.
//   * That assignment is ALSO what makes the feedback loop safe. `predicted.heading()` already
//     contains the bias built out of this filter's own past nudges. A filter that integrated
//     `Δθ = predicted.heading() − previousPredictedHeading` would count its own correction a
//     SECOND time and overshoot, with the overshoot growing with the gain. Re-basing on the
//     handed heading makes the double-count structurally impossible rather than arithmetically
//     avoided. (Δθ is still computed — with this filter's own last nudge subtracted off — but
//     only to size the process noise on θ, where what is wanted is the PHYSICAL rotation.)
//   * Only a proposal that sets `providesHeading` may move θ. For every other update the θ row
//     of the Kalman gain is ZEROED, so a GPS cannot rotate the robot's idea of the field
//     through a cross-covariance term. That preserves E2's T3 ruling under the swap, and it is
//     the reason the published heading is still bit-identical to the raw IMU on a tree with no
//     heading-providing corrector.
//
// REJECTED — let the EKF own the published heading and delete the re-stamp. Tidier algebra, and
// a structural change to both M2's decision #4 and E3's design: a policy that can return an
// absolute heading can snap one. REJECTED — drop θ and run a 4-state filter with field-frame
// velocity. Simpler, and it destroys the point: with no θ in the state there is no θ VARIANCE,
// so a heading fix cannot be weighed against the filter's own uncertainty, and heading
// arbitration becomes impossible. REJECTED — track θ but emit nothing, leaving heading to the
// complementary tier. Then swapping tiers would silently DELETE E3's heading correction, which
// is a regression wearing a feature's clothes.
//
// ── THE TICK, IN ORDER ────────────────────────────────────────────────────────────────────
// The Localizer hands this policy an already-INTEGRATED dead-reckoned prediction, not a raw
// control input. Both facts below fall out of that.
//
//   A. Re-base θ to `predicted.heading()`; recover this tick's field-frame odometry increment
//      `u = predicted.position − lastReturnedPosition` (exact: the Localizer assigns
//      `fusedX_ = fr.x`, so the policy's own last answer is what the prediction was built on).
//      Add the process noise Q for the interval.
//   B. ODOMETRY UPDATE — `R(θ)ᵀ·u/dt` is a measurement of the BODY-frame velocity, which is
//      literally what the wheels measured (`PilonsOdometry` rotated it out by the same heading
//      θ was just re-based to, so rotating it back is exact). This is the channel through which
//      the wheels enter the filter. It moves the VELOCITY states ONLY: the p and θ rows of its
//      gain are zeroed.
//      WHY, and it is the most important line in this file: the wheels measure how far they
//      TURNED. They say nothing directly about where the robot IS. Folding a relative
//      measurement as though it were an absolute one would shrink the position covariance every
//      tick, and a position covariance that shrinks while dead-reckoning is a filter that
//      becomes CERTAIN as it becomes WRONG — after which no absolute fix can ever pass the
//      gate. That is E2's D2 gate-lockout failure, arrived at from the other side.
//   C. PROPAGATE — `p ← p + R(θ)·v·dt` with the just-updated velocity, and `P ← F P Fᵀ`
//      (all of Q for the interval was added once, in step A).
//      Using the POSTERIOR velocity is deliberate: `u/dt` is the AVERAGE velocity over the
//      interval just ended, so it is the right velocity to carry the position across that same
//      interval. Doing it the other way round (propagate, then update) lags the odometry by a
//      full tick at every change of speed.
//   D. CORRECT — each valid proposal in ascending σ (most trusted first), gated on Mahalanobis
//      distance, applied through a Joseph-form update with the never-snap budget enforced as a
//      GAIN REDUCTION (see below).
//   E. EMIT — the posterior position, the accumulated θ change as `headingNudge`, and the audit.
//
// Consequence worth stating plainly: with no proposals at all, this filter's answer is NOT
// bit-identical to the odometry's, the way the complementary tier's is. It differs by the one
// tick of velocity filtering in steps B/C — measured at under 0.7 inches of cumulative gap over
// two minutes of stop-start driving, bounded by about one tick's travel, and provably NOT
// cumulative (four times the run length does not widen it). `AppliedCorrection::dx/dy` therefore
// reads that small residual on a dead-reckoning tick under this tier, where it reads exactly
// zero under the complementary one. Measured and pinned by test rather than asserted here.
//
// On a CORRECTING tick that residual is charged against the per-tick budget BEFORE any proposal
// is folded (see `foldProposals`), so the tick's total departure is `max(budget, residual)`
// rather than `budget + residual` — without that charge, a persistent correction stream was
// measured inflating the published move to 0.16 inches against a 0.12 inch budget, because a
// position fix teaches the velocity states and that comes back as motion on the next tick.
//
// WHAT THAT MEANS FOR THE §18.2 AUDIT, STATED EXACTLY, BECAUSE IT IS NOT QUITE THE SAME UNDER
// THE TWO TIERS. The never-snap budget bounds the CORRECTION exactly, under both tiers, always
// — `lastCorrectionMagnitude()` is the quantity, and it never exceeds `maxNudgeRate · dt`.
// `AppliedCorrection::dx/dy` is the tick's TOTAL departure from the dead-reckoned prediction,
// which under the complementary tier is the correction and nothing else, and under this tier is
// the correction OR the filtering residual, whichever is larger. Measured over eight seeds of a
// 60-second hostile plant run, the residual reached **0.133 inches against a 0.12 inch budget**
// — 11% over, and only during the hardest direction changes in the script. That excess is not a
// correction: it is the estimate tracking the robot's real motion through a one-tick filter, and
// it is bounded by a fraction of one tick's travel. A reader auditing never-snap from a blackbox
// written under this tier should read `dx/dy` with that 11% allowance, and chapter 11 says so.
//
// ── THE NEVER-SNAP BOUND IS A GAIN REDUCTION, WHICH IS WHY THE JOSEPH FORM IS LOAD-BEARING ─
// Decision #4 says a correction is a bounded nudge, never a snap, and it does not stop applying
// because the filter got cleverer. The obvious implementation — take the Kalman step, then clip
// the state move — makes the filter LIE: `P` would shrink as though the full correction had
// been applied while the state still sat where the clip left it, i.e. it would become confident
// precisely because it was prevented from correcting.
//
// So the bound is applied to the GAIN instead. If the optimal step would move the position by
// more than `maxNudgeRate·dt` (or the heading by more than `maxHeadingNudgeRate·dt`), the gain
// is scaled by the ratio, and the covariance is then updated with THAT gain. The Joseph form
//     P⁺ = (I − K H) P⁻ (I − K H)ᵀ + K R Kᵀ
// is exactly correct for ANY gain, optimal or not — that is its actual virtue, and it is the
// reason it is used here rather than the shorter `(I − K H) P⁻`, which is only valid at the
// optimal gain and silently loses symmetry and positive-definiteness away from it. Every
// deliberately suboptimal gain in this file (the rate clamp, the zeroed heading row, the
// velocity-only odometry update) depends on that property.
//
// ── T4 — THE 12-INCH CEILING IS REPLACED HERE, AND SURVIVES IN THE OTHER TIER ──────────────
// `ComplementaryFusion::innovationGate` is a fixed 12 inches, and E2 recorded live that an
// estimate 29 inches out never recovered with a perfectly good GPS in view. A fixed distance is
// exactly what a covariance replaces. The test here is
//     ν = √( rᵀ S⁻¹ r ),   S = H P Hᵀ + R,   reject if ν > gateSigma
// so the SAME 29-inch fix is REJECTED when the filter is confident (it is far more likely to be
// a reflection than the truth) and ACCEPTED when the filter knows it is badly lost. The
// complementary tier keeps its 12-inch gate unchanged — it has no `P` to normalise by, and E2's
// T1 already ruled that a distance normalised by an assumed constant must not be called a
// Mahalanobis distance.
//
// ── T5 — `gateMahalanobis` BECOMES REAL ───────────────────────────────────────────────────
// `GateAudit::mahalanobis` and `DebugRecord::gateMahalanobis` have been declared and empty since
// A1, and E2 refused to fill them with the ratio it had, because normalising by an ASSUMED
// constant makes the assumption the entire content. The number written here comes from `S`,
// which is `P` (estimated by this filter, tick by tick) plus `R` (the fix's own stated σ), so
// `RejectedMahalanobis` is finally raisable by something that earned it.
// `GateAudit::covarianceTrace` carries `P[px][px] + P[py][py]`, in square inches — the POSITION
// block only, because a trace over a state vector mixing inches, radians and inches-per-second
// is a number with no unit and no meaning. A reader wanting a 1σ radius takes `√(trace/2)`.
//
// ── T2 — "CONSECUTIVE-REJECT RE-INIT" vs §13 #4 "NEVER SNAP" ───────────────────────────────
// build-order.md asks for a consecutive-reject re-init; §13 #4 forbids snapping; a re-init that
// teleports the estimate is a snap. The conflict is real as written, and it dissolves the moment
// "re-init" is read as re-initialising the belief's UNCERTAINTY rather than its VALUE.
//
// RULING: on the trigger, this filter does not move the estimate by so much as a thousandth of
// an inch. It resets the POSITION and VELOCITY covariance to the initial prior and nothing else.
//   * Never-snap holds bit-for-bit — the per-tick rate clamp is untouched and still binds on the
//     re-init tick and every tick after it. No existing never-snap test changed.
//   * The estimator nevertheless RECOVERS, which is the entire reason re-init was asked for:
//     with a large P the Mahalanobis gate opens, the following fixes are accepted with a large
//     gain, and the estimate walks home at up to `maxNudgeRate` instead of never arriving.
//   * It is DECLARED: `GateReason::CovarianceReinit` on that tick, so the event is a word in the
//     decoded blackbox rather than an inference, and `covarianceTrace` jumps on the same tick as
//     an independent numeric witness of the same event.
//   * It is LATCHED and counted (`reinitCount()`, `everReinit()`), and RATE-LIMITED by a
//     cooldown, and it takes a high bar: N CONSECUTIVE gate rejections AND a mean rejected
//     innovation above a floor. Ticks where nothing was proposed neither count nor reset.
// REJECTED — teleport the state onto the rejected fix. That is the snap §13 #4 forbids, and it
// contains a plain self-contradiction: the trigger is the filter saying N times that it does NOT
// trust this fix, so jumping onto it trusts it completely on the strength of having distrusted
// it. REJECTED — no re-init at all: that preserves E2's finding 2 forever, and a robot shoved by
// an opponent never recovers with a perfect fix in view. REJECTED — permanently widen the gate
// after N rejections: the outlier protection is then spent once and gone, where covariance
// inflation self-heals as fixes are folded.
//
// ── HOW PROPOSALS ARE WEIGHED (the capability that justifies the chunk) ────────────────────
// Proposals are folded as SEQUENTIAL Kalman updates in ascending `positionStdDev`, which for
// independent measurements is equivalent to a batch update while still letting each one be gated
// on its own merits (a batch update cannot reject one row). Most-trusted-first is deliberate: a
// good fix tightens P before a doubtful one is tested against it. Two sources that disagree
// therefore settle at the inverse-variance-weighted point between them —
//     x* = (z_A/σ_A² + z_B/σ_B²) / (1/σ_A² + 1/σ_B²)
// — rather than at whichever arrived first, or at the midpoint.
//
// **`confidence` is NOT used as a weight, deliberately.** E2 DERIVES its confidence from σ (it
// is the scalar Kalman gain σ_dr²/(σ_dr² + σ_meas²)), so weighting by both would count the same
// information twice. This tier weights by the stated σ and by nothing else — which is precisely
// the difference between a covariance filter and the gain knob that HA-66 and HA-78 have been
// wearing a covariance's clothes as. `confidence` is still read for ONE purpose: it is passed
// back out as `appliedConfidence`, which is Localizer bookkeeping (how much of the drift
// accumulator an applied fix clears), not fusion weighting.
//
// Heading has no per-proposal σ on `CorrectionProposal`, so heading measurements use one
// configured σ. REJECTED — append a `headingStdDev` field now (E3's handoff suggested it): no
// corrector in the tree can state one, and "nothing reads it today" is exactly how a field
// becomes load-bearing by accident (E2's T3, in reverse). It stays a named handoff for whoever
// adds a corrector that can measure it. REJECTED — scale the heading σ by `confidence`: that
// invents a relationship between a [0,1] trust scalar and a variance, and it would be
// inconsistent with the position channel, which ignores confidence.
//
// ── COST ──────────────────────────────────────────────────────────────────────────────────
// Everything is fixed-size `std::array` on the stack: 5 states, a 5×5 covariance, at most
// `Localizer::kMaxCorrectors` proposals. `fuse()` never allocates and never throws (all
// preconditions are in the constructor; every runtime pathology is screened and counted rather
// than raised). Pinned by test with a replaced global allocator, not asserted here.
//
// ── WHAT IS INVENTED ──────────────────────────────────────────────────────────────────────
// Every noise number below is a GUESS until R4 measures the hardware. They are registered
// HA-83…HA-91 and each carries its tag. The STRUCTURE is what this chunk proves; the NUMBERS
// are fitted on a robot that does not exist yet. No test in this chunk asserts that a constant
// is right — only that a shape is (farther travel ⇒ more uncertainty, a tighter σ ⇒ more pull,
// a confident filter rejects what a lost one accepts).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/i_fusion_policy.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// Tuning for `EkfFusion`. Every value is INVENTED and registered in the A4 hardware-assumptions
/// register; R4 replaces them with measurements. The defaults are deliberately conservative
/// (wide priors, a modest gate) so the filter's failure mode is "slow to trust" rather than
/// "confidently wrong".
struct EkfFusionConfig {
    // ── process noise Q — how fast the belief decays while dead-reckoning ──────────────────
    /// 1σ position error added per inch travelled (2% of travel). This is the term that makes
    /// the gate widen after a long blind stretch, which is what stops the E2/D2 gate lockout.
    /// PROVISIONAL (A4: HA-83).
    double posNoisePerInch = 0.02;
    /// 1σ position error added per second even when standing still — the floor that keeps `P`
    /// strictly positive-definite on a stationary tick. PROVISIONAL (A4: HA-83).
    units::Velocity posNoiseRate{0.5};
    /// 1σ heading error added per radian actually rotated (1% of the rotation) — scale-factor
    /// error in the gyro. PROVISIONAL (A4: HA-84).
    double headingNoisePerRad = 0.01;
    /// 1σ heading error added per second at rest: HA-20's ≈1°/min of raw V5 IMU drift, which is
    /// the assumption the whole heading-correction story rests on. PROVISIONAL (A4: HA-84).
    units::AngularVelocity headingDriftRate{(1.0 / 60.0) * math::Angle::kPi / 180.0};
    /// How much body velocity the drivetrain can gain or lose in one second — the process noise
    /// on the velocity states, i.e. how far the constant-velocity model is allowed to be wrong.
    /// 200 in/s² is roughly a hard VEX drive launch. PROVISIONAL (A4: HA-85).
    units::Acceleration velNoise{200.0};

    // ── the odometry channel R ─────────────────────────────────────────────────────────────
    /// 1σ error on ONE TICK's odometry displacement, independent of distance — encoder
    /// quantization and tracking-wheel jitter. PROVISIONAL (A4: HA-86).
    units::Length odomStdDev{0.01};
    /// …plus this fraction of the tick's travel — slip, which scales with distance.
    /// PROVISIONAL (A4: HA-86).
    double odomStdDevPerInch = 0.02;

    // ── the gate and the measurement noise this tier cannot get from a proposal ────────────
    /// Reject a fix whose Mahalanobis distance exceeds this. 3.0 on a 2-degree-of-freedom
    /// position innovation is a ≈1.1% false-reject rate if the noise model is right.
    /// PROVISIONAL (A4: HA-87).
    double gateSigma = 3.0;
    /// 1σ on an absolute heading measurement, flat: `CorrectionProposal` carries no heading σ,
    /// and inventing a per-proposal relationship would be worse than one honest constant.
    /// PROVISIONAL (A4: HA-88).
    units::AngleDim headingStdDev{2.0 * math::Angle::kPi / 180.0};

    // ── the prior, used at startup, after a discontinuity, and on re-init ──────────────────
    /// "I could be anywhere within a tile." PROVISIONAL (A4: HA-89).
    units::Length initialPosStdDev{24.0};
    /// PROVISIONAL (A4: HA-89).
    units::AngleDim initialHeadingStdDev{30.0 * math::Angle::kPi / 180.0};
    /// PROVISIONAL (A4: HA-89).
    units::Velocity initialVelStdDev{24.0};

    // ── never-snap (§13 #4) — the same contract and the same numbers as the other tier ─────
    /// Max position correction per tick, as a RATE, so the bound is loop-rate independent.
    /// Matches `ComplementaryFusionConfig::maxNudgeRate` on purpose: never-snap must not change
    /// meaning when the tier is swapped.
    units::Velocity maxNudgeRate{12.0};
    /// Max heading-bias change per tick, as a rate. Matches `maxHeadingNudgeRate` (A4: HA-82).
    units::AngularVelocity maxHeadingNudgeRate{10.0 * math::Angle::kPi / 180.0};

    // ── re-init (T2) ───────────────────────────────────────────────────────────────────────
    /// How many CONSECUTIVE gate rejections before the filter is willing to admit it is lost.
    /// At a ~20 Hz fix cadence this is ≈2.5 seconds of a sensor insisting the estimate is wrong.
    /// PROVISIONAL (A4: HA-90).
    int reinitRejectCount = 50;
    /// …and the mean rejected innovation over that run must exceed this, so a burst of
    /// borderline rejections while the filter is very confident cannot trigger it.
    /// PROVISIONAL (A4: HA-90).
    units::Length reinitInnovation{6.0};
    /// Minimum time between re-inits — the rate limit. PROVISIONAL (A4: HA-91).
    units::Time reinitCooldown{5.0};

    /// Above this tick dt, the interval is not a usable prediction step (a loop stall, or the
    /// dt==0 tick the Localizer produces after construction and after `setPose`). The filter
    /// re-bases on the handed prediction instead of integrating garbage. Mirrors
    /// `LocalizerConfig::maxDt`; kept here because a policy cannot see the Localizer's config.
    double maxDt = 0.1;
};

/// A 5-state SE(2) extended Kalman filter implementing `IFusionPolicy`. See the file header for
/// the design and for the T1/T2/T4/T5 rulings.
///
/// STATEFUL, unlike `ComplementaryFusion`. `IFusionPolicy::fuse` never promised statelessness —
/// an EKF cannot be stateless — but nothing said so either, so it is said here: ONE instance
/// belongs to ONE Localizer, is mutated on the control task only, and must outlive it.
class EkfFusion final : public IFusionPolicy {
public:
    /// State dimension. Indices are named below so no bare 0..4 appears in the algebra.
    static constexpr std::size_t kN = 5;
    static constexpr std::size_t kPx = 0;
    static constexpr std::size_t kPy = 1;
    static constexpr std::size_t kTh = 2;
    static constexpr std::size_t kVx = 3;
    static constexpr std::size_t kVy = 4;

    explicit EkfFusion(const EkfFusionConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(config.posNoisePerInch >= 0.0, "EkfFusion: posNoisePerInch must be >= 0");
        SHULIB_PRECONDITION(config.posNoiseRate.value() > 0.0, "EkfFusion: posNoiseRate must be > 0");
        SHULIB_PRECONDITION(config.headingNoisePerRad >= 0.0,
                            "EkfFusion: headingNoisePerRad must be >= 0");
        SHULIB_PRECONDITION(config.headingDriftRate.value() > 0.0,
                            "EkfFusion: headingDriftRate must be > 0");
        SHULIB_PRECONDITION(config.velNoise.value() > 0.0, "EkfFusion: velNoise must be > 0");
        SHULIB_PRECONDITION(config.odomStdDev.value() > 0.0, "EkfFusion: odomStdDev must be > 0");
        SHULIB_PRECONDITION(config.odomStdDevPerInch >= 0.0,
                            "EkfFusion: odomStdDevPerInch must be >= 0");
        SHULIB_PRECONDITION(config.gateSigma > 0.0, "EkfFusion: gateSigma must be > 0");
        SHULIB_PRECONDITION(config.headingStdDev.value() > 0.0,
                            "EkfFusion: headingStdDev must be > 0");
        SHULIB_PRECONDITION(config.initialPosStdDev.value() > 0.0,
                            "EkfFusion: initialPosStdDev must be > 0");
        SHULIB_PRECONDITION(config.initialHeadingStdDev.value() > 0.0,
                            "EkfFusion: initialHeadingStdDev must be > 0");
        SHULIB_PRECONDITION(config.initialVelStdDev.value() > 0.0,
                            "EkfFusion: initialVelStdDev must be > 0");
        SHULIB_PRECONDITION(config.maxNudgeRate.value() >= 0.0,
                            "EkfFusion: maxNudgeRate must be >= 0");
        SHULIB_PRECONDITION(config.maxHeadingNudgeRate.value() >= 0.0,
                            "EkfFusion: maxHeadingNudgeRate must be >= 0");
        SHULIB_PRECONDITION(config.reinitRejectCount > 0, "EkfFusion: reinitRejectCount must be > 0");
        SHULIB_PRECONDITION(config.reinitInnovation.value() > 0.0,
                            "EkfFusion: reinitInnovation must be > 0");
        SHULIB_PRECONDITION(config.reinitCooldown.value() >= 0.0,
                            "EkfFusion: reinitCooldown must be >= 0");
        SHULIB_PRECONDITION(config.maxDt > 0.0, "EkfFusion: maxDt must be > 0");
    }

    [[nodiscard]] FusionResult fuse(const math::Pose2d& predicted,
                                    std::span<const CorrectionProposal> valid,
                                    units::Time dt) override {
        const double px = predicted.x().value();
        const double py = predicted.y().value();
        const double ph = predicted.heading().radians();
        const double h = dt.value();

        // A tick whose inputs are not finite is not a tick this filter can reason about. Return
        // the prediction untouched rather than poisoning five states and twenty-five covariance
        // entries with a NaN that would never wash out (F4's degrade-don't-die posture).
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(ph) || !std::isfinite(h)) {
            ++numericGuardTrips_;
            return passThrough(px, py);
        }

        // ── STEP A(0) — startup and discontinuity ────────────────────────────────────────
        // `dt <= 0` is what the Localizer hands us on the first tick and on the tick after a
        // `setPose()` teleport (it clears `hasLast_`, so the next dt is 0). `dt > maxDt` is a
        // loop stall. In both cases `u/dt` is meaningless, so the filter re-bases onto the
        // handed prediction — which is the odometry's answer, and the odometry is still right
        // about DISPLACEMENT even when the clock is not — and widens its position covariance to
        // say that something discontinuous happened. It applies no correction on such a tick,
        // exactly as the complementary tier applies none when its budget is zero, so the two
        // tiers agree about what a stalled tick means.
        if (!initialized_) {
            initialize(px, py, ph);
            return passThrough(px, py);
        }
        if (!(h > 0.0) || h > cfg_.maxDt) {
            resync(px, py, ph);
            return passThrough(px, py);
        }
        elapsed_ += h;

        // ── STEP A — re-base heading, recover the odometry increment, add Q ──────────────
        // `u` is exact: the Localizer assigns `fusedX_ = fr.x` verbatim, so the prediction it
        // hands back is (this policy's own last answer) + (the odometry's field-frame delta).
        const double ux = px - lastX_;
        const double uy = py - lastY_;
        const double travel = std::hypot(ux, uy);

        // The PHYSICAL rotation over the tick: the change in the handed heading, minus the
        // nudge this filter itself contributed to it last tick. Used only to size Q on θ — θ
        // itself is re-based, not integrated (header, T1).
        const double dThetaRaw = math::Angle::radians(lastHeading_).errorTo(predicted.heading());
        const double dTheta = dThetaRaw - lastHeadingNudge_;
        x_[kTh] = ph;  // the θ time update IS the IMU's

        addProcessNoise(travel, std::abs(dTheta), h);

        // ── STEP B — the odometry velocity update (velocity states only) ────────────────
        odometryUpdate(ux, uy, h, travel);

        // ── STEP C — propagate position with the posterior velocity ─────────────────────
        propagatePosition(h);

        // ── STEP D — fold the absolute fixes ────────────────────────────────────────────
        FusionResult result{};
        foldProposals(valid, h, px, py, result);

        // ── STEP E — emit ───────────────────────────────────────────────────────────────
        result.x = units::Length{x_[kPx]};
        result.y = units::Length{x_[kPy]};
        result.audit.covarianceTrace = positionCovarianceTrace();
        lastX_ = x_[kPx];
        lastY_ = x_[kPy];
        lastHeading_ = x_[kTh];
        lastHeadingNudge_ = result.headingApplied ? result.headingNudge.value() : 0.0;
        return result;
    }

    // ── observability (telemetry and tests; none of this is on the control path) ──────────

    /// `P[px][px] + P[py][py]`, square inches — the POSITION block only (header, T5). A 1σ
    /// radius is `sqrt(trace / 2)`.
    [[nodiscard]] double positionCovarianceTrace() const noexcept {
        return at(P_, kPx, kPx) + at(P_, kPy, kPy);
    }
    /// One covariance entry, for the invariant tests (symmetry, positive-definiteness).
    [[nodiscard]] double covariance(std::size_t i, std::size_t j) const noexcept {
        return at(P_, i, j);
    }
    /// One state entry, indexed by the `kPx`…`kVy` constants.
    [[nodiscard]] double state(std::size_t i) const noexcept { return x_[i]; }
    /// Body-frame velocity estimate, in/s.
    [[nodiscard]] units::Velocity velocityX() const noexcept { return units::Velocity{x_[kVx]}; }
    [[nodiscard]] units::Velocity velocityY() const noexcept { return units::Velocity{x_[kVy]}; }

    /// How many times the covariance has been re-initialised (T2). Latched for the run.
    [[nodiscard]] std::uint32_t reinitCount() const noexcept { return reinitCount_; }
    /// Latched: has this filter ever declared itself lost? Never clears — a run in which the
    /// estimator gave up once is a different run from one in which it did not, forever.
    [[nodiscard]] bool everReinit() const noexcept { return reinitCount_ > 0; }
    /// Consecutive gate rejections right now (resets on any accepted fix).
    [[nodiscard]] int consecutiveRejects() const noexcept { return consecutiveRejects_; }
    /// Ticks on which the filter re-based instead of predicting (first tick, teleport, stall).
    [[nodiscard]] std::uint32_t resyncCount() const noexcept { return resyncCount_; }
    /// Times a non-finite intermediate was caught and the update abandoned. Should be 0.
    [[nodiscard]] std::uint32_t numericGuardTrips() const noexcept { return numericGuardTrips_; }
    /// Fixes accepted by the Mahalanobis gate, and fixes rejected by it.
    [[nodiscard]] std::uint32_t acceptedFixes() const noexcept { return acceptedFixes_; }
    [[nodiscard]] std::uint32_t rejectedFixes() const noexcept { return rejectedFixes_; }

    /// How far the last tick's CORRECTIONS moved the position, summed over the proposals folded
    /// (so it upper-bounds the net move). This — not `AppliedCorrection::dx`, which under this
    /// tier also carries the small velocity-filtering residual from steps B/C — is the quantity
    /// `maxNudgeRate · dt` bounds, and it is what a never-snap test should assert on.
    [[nodiscard]] units::Length lastCorrectionMagnitude() const noexcept {
        return units::Length{lastAppliedPos_};
    }
    /// …and the same for heading: |the increment emitted last tick|, bounded by
    /// `maxHeadingNudgeRate · dt`.
    [[nodiscard]] units::AngleDim lastHeadingCorrectionMagnitude() const noexcept {
        return units::AngleDim{lastAppliedHeading_};
    }

private:
    using Mat = std::array<double, kN * kN>;
    using Vec = std::array<double, kN>;

    [[nodiscard]] static double at(const Mat& m, std::size_t i, std::size_t j) noexcept {
        return m[i * kN + j];
    }
    static double& at(Mat& m, std::size_t i, std::size_t j) noexcept { return m[i * kN + j]; }

    /// The answer on a tick the filter could not act on: hand back the prediction unchanged.
    /// Identical in shape to what the complementary tier returns for a zero budget, so the two
    /// tiers cannot disagree about what "nothing happened" looks like.
    [[nodiscard]] FusionResult passThrough(double px, double py) const {
        FusionResult r{};
        r.x = units::Length{px};
        r.y = units::Length{py};
        r.audit.covarianceTrace = positionCovarianceTrace();
        return r;
    }

    void initialize(double px, double py, double ph) {
        x_ = {px, py, ph, 0.0, 0.0};
        P_.fill(0.0);
        const double sp = cfg_.initialPosStdDev.value();
        const double sh = cfg_.initialHeadingStdDev.value();
        const double sv = cfg_.initialVelStdDev.value();
        at(P_, kPx, kPx) = sp * sp;
        at(P_, kPy, kPy) = sp * sp;
        at(P_, kTh, kTh) = sh * sh;
        at(P_, kVx, kVx) = sv * sv;
        at(P_, kVy, kVy) = sv * sv;
        lastX_ = px;
        lastY_ = py;
        lastHeading_ = ph;
        lastHeadingNudge_ = 0.0;
        initialized_ = true;
        consecutiveRejects_ = 0;
        rejectSum_ = 0.0;
        travelSinceFix_ = 0.0;
        timeSinceFix_ = 0.0;
        rotSinceHeadingFix_ = 0.0;
        timeSinceHeadingFix_ = 0.0;
    }

    /// A discontinuity (teleport / stall): take the odometry's word for the position, forget the
    /// velocity, and widen. The heading covariance is left alone — a stalled loop does not make
    /// the IMU less trustworthy, and `predicted.heading()` is as good on this tick as any other.
    void resync(double px, double py, double ph) {
        ++resyncCount_;
        x_[kPx] = px;
        x_[kPy] = py;
        x_[kTh] = ph;
        x_[kVx] = 0.0;
        x_[kVy] = 0.0;
        const double sp = cfg_.initialPosStdDev.value();
        const double sv = cfg_.initialVelStdDev.value();
        // Position uncertainty is ADDED to (not replaced by): whatever we already doubted is
        // still doubted. Velocity is REPLACED: after a discontinuity the old velocity is not
        // evidence about the new one, and keeping its covariance would keep its cross-terms too.
        at(P_, kPx, kPx) += sp * sp;
        at(P_, kPy, kPy) += sp * sp;
        for (std::size_t i = 0; i < kN; ++i) {
            at(P_, i, kVx) = 0.0;
            at(P_, kVx, i) = 0.0;
            at(P_, i, kVy) = 0.0;
            at(P_, kVy, i) = 0.0;
        }
        at(P_, kVx, kVx) = sv * sv;
        at(P_, kVy, kVy) = sv * sv;
        lastX_ = px;
        lastY_ = py;
        lastHeading_ = ph;
        lastHeadingNudge_ = 0.0;
        travelSinceFix_ = 0.0;  // the widening above already carries the discontinuity
        timeSinceFix_ = 0.0;
    }

    /// Q for one interval. THE point of this function is that the position and heading terms
    /// scale with what actually happened — travel and rotation — not with time alone: an
    /// estimator that stands still barely loses confidence, and one that has driven six feet
    /// blind has lost a lot. That is what widens the gate after a blind stretch, and it is the
    /// mechanism E2's D2 had to hand-build inside a corrector for want of it.
    ///
    /// AND IT IS A SYSTEMATIC GROWTH, NOT A RANDOM WALK — which is a modelling decision, and
    /// the difference is enormous. Adding `(k·Δtravel)²` every tick makes σ grow as the SQUARE
    /// ROOT of distance, because independent per-tick errors partly cancel. Real odometry error
    /// does not cancel: it is dominated by a systematic scale and alignment error (a wheel
    /// diameter measured 1% small is 1% small on every tick, in the same direction), so σ grows
    /// LINEARLY with distance. Measured while building this chunk, with the random-walk form in
    /// place: after 360 inches of dead-reckoning the filter believed it was within half an inch,
    /// and a truthful fix 20 inches away was still rejected. So the increment added here is the
    /// increment of `(k·travelSinceFix)²`, which makes σ = k·travelSinceFix exactly. That is the
    /// same model E2's `driftStdDevPerInch` already uses one layer up, and the two now agree.
    ///
    /// The IMU's drift-per-minute (HA-20) gets the same treatment for the same reason — a gyro
    /// bias is a bias, not a coin flip. The VELOCITY states keep a genuine random walk, because
    /// acceleration really is unpredictable from one tick to the next.
    ///
    /// The accumulators reset when a fix is accepted, exactly as E2's do: an absolute fix
    /// removes the accumulated bias, so the clock on the next one starts again. (How much was
    /// LEARNED from the fix is the covariance update's business, not this one's.)
    void addProcessNoise(double travel, double rotation, double h) {
        const double travelBefore = travelSinceFix_;
        const double timeBefore = timeSinceFix_;
        travelSinceFix_ += travel;
        timeSinceFix_ += h;
        (void)timeBefore;
        const double spBefore = cfg_.posNoisePerInch * travelBefore;
        const double spAfter = cfg_.posNoisePerInch * travelSinceFix_;
        // The TRAVEL term is systematic (see above) and grows linearly. The standing-still
        // FLOOR is not: it stands for small unmodelled disturbances with no preferred
        // direction, so it is a genuine random walk and is added as a variance per tick. The
        // distinction is not pedantry — making the floor systematic too would mean a robot
        // standing perfectly still accumulated 30 inches of position doubt over a match, and
        // would then accept a 30-inch lie as though it had earned it.
        const double dPosVar = spAfter * spAfter - spBefore * spBefore +
                               cfg_.posNoiseRate.value() * h * cfg_.posNoiseRate.value() * h;

        const double rotBefore = rotSinceHeadingFix_;
        const double headTimeBefore = timeSinceHeadingFix_;
        rotSinceHeadingFix_ += rotation;
        timeSinceHeadingFix_ += h;
        const double shBefore = cfg_.headingNoisePerRad * rotBefore +
                                cfg_.headingDriftRate.value() * headTimeBefore;
        const double shAfter = cfg_.headingNoisePerRad * rotSinceHeadingFix_ +
                               cfg_.headingDriftRate.value() * timeSinceHeadingFix_;
        const double dHeadVar = shAfter * shAfter - shBefore * shBefore;

        const double sv = cfg_.velNoise.value() * h;
        at(P_, kPx, kPx) += dPosVar;
        at(P_, kPy, kPy) += dPosVar;
        at(P_, kTh, kTh) += dHeadVar;
        at(P_, kVx, kVx) += sv * sv;
        at(P_, kVy, kVy) += sv * sv;
    }

    /// STEP B. The odometry increment as a measurement of the BODY-frame velocity:
    /// `z = R(θ)ᵀ·u / dt`, `h(x) = [vx, vy]`, so `H`'s θ column is exactly zero.
    ///
    /// THE FRAME CHOICE IS NOT COSMETIC — the field-frame form is wrong here, and measurably so.
    /// Written as `z = u/dt` against `h(x) = R(θ)·v`, the innovation covariance picks up an
    /// `H_θ P_θθ H_θᵀ = |v|²·P_θθ` term; at the 30° prior and 30 in/s that is ≈246 (in/s)², which
    /// swamps the measurement and drives the velocity gain to nearly zero. The filter then
    /// REFUSES TO BELIEVE THE WHEELS BECAUSE IT IS UNSURE WHICH WAY IT IS FACING, and
    /// dead-reckons on a velocity it never updated — measured, before the fix, as an 85-inch gap
    /// against the raw odometry over 30 seconds.
    ///
    /// That sensitivity is fictitious. `u` was produced by `PilonsOdometry` by rotating the
    /// wheels' body-frame displacement through the IMU heading — the same heading θ is re-based
    /// to at the top of the tick. Rotating it back recovers exactly what the wheels measured,
    /// introducing no uncertainty. The real cost of a wrong heading is that the whole increment
    /// is rotated wrongly, and that is accounted for where it belongs: in the θ column of `F`
    /// during step C. The field-frame form counted it a second time, in the wrong place.
    ///
    /// The gain's position and heading rows are zeroed for the separate reason in the file
    /// header: the wheels say nothing directly about location or bearing, and folding a relative
    /// measurement as an absolute one manufactures confidence the filter has not earned.
    /// The θ block is load-bearing and mutation-proven (unblocking it lets a wheel reading
    /// rotate the robot's idea of the field). The POSITION block is structural rather than
    /// numerically critical in this formulation and the difference is measured, not assumed:
    /// unblocking it leaks only through the p–v cross-covariance and moves the position trace
    /// by between 4e-6 and 1.5e-4 relative over a blind trajectory — while writing the update
    /// as an ABSOLUTE position measurement instead, which is what the block is really guarding
    /// against, collapses the covariance and turns 20 tests red. Both are in the harness; the
    /// mild one is recorded there as KNOWN GREEN with its numbers.
    void odometryUpdate(double ux, double uy, double h, double travel) {
        const double c = std::cos(x_[kTh]);
        const double s = std::sin(x_[kTh]);

        std::array<double, 2 * kN> H{};
        H[0 * kN + kVx] = 1.0;
        H[1 * kN + kVy] = 1.0;

        const double zx = (ux * c + uy * s) / h;   // R(θ)ᵀ u / dt
        const double zy = (-ux * s + uy * c) / h;
        const std::array<double, 2> r{zx - x_[kVx], zy - x_[kVy]};
        const double sigmaU = cfg_.odomStdDev.value() + cfg_.odomStdDevPerInch * travel;
        const double rv = (sigmaU / h) * (sigmaU / h);
        const std::array<double, 4> R{rv, 0.0, 0.0, rv};

        // NOT GATED, and that is load-bearing. The Mahalanobis gate exists to refuse an
        // ABSOLUTE FIX that disagrees with the filter; the odometry is not a fix, it is the
        // dead-reckoning input. Gating it was measured, during this chunk, to make the filter
        // stop believing the wheels every time the robot changed speed hard — a 40 in/s launch
        // against a converged velocity covariance is a Mahalanobis distance of about 13 — after
        // which the estimate dead-reckoned on a velocity it had refused to update, and drifted
        // 86 inches over 30 seconds. A gate on the prediction channel is a filter that rejects
        // reality for disagreeing with its model.
        UpdateOutcome ignored{};
        applyUpdate(H.data(), 2, r.data(), R.data(), /*mayMoveHeading=*/false,
                    /*mayMovePosition=*/false, /*posBudget=*/kUnbounded,
                    /*headBudget=*/kUnbounded, /*gate=*/kUnbounded, ignored);
    }

    /// STEP C. `p ← p + R(θ)·v·dt`, a deterministic nonlinear transform of the state, so the
    /// covariance goes through `F P Fᵀ` with F the Jacobian. The θ column of F is where the
    /// "extended" in EKF actually lives: rotating the same body velocity under a different
    /// heading lands somewhere else, and that coupling is what lets a heading fix improve the
    /// position estimate and a position fix improve the heading estimate's siblings.
    void propagatePosition(double h) {
        const double c = std::cos(x_[kTh]);
        const double s = std::sin(x_[kTh]);
        const double vx = x_[kVx];
        const double vy = x_[kVy];

        Mat F{};
        for (std::size_t i = 0; i < kN; ++i) {
            at(F, i, i) = 1.0;
        }
        at(F, kPx, kTh) = -(vx * s + vy * c) * h;
        at(F, kPx, kVx) = c * h;
        at(F, kPx, kVy) = -s * h;
        at(F, kPy, kTh) = (vx * c - vy * s) * h;
        at(F, kPy, kVx) = s * h;
        at(F, kPy, kVy) = c * h;

        x_[kPx] += (vx * c - vy * s) * h;
        x_[kPy] += (vx * s + vy * c) * h;

        Mat FP{};
        multiply(F, P_, FP);
        multiplyTransposed(FP, F, P_);
        symmetrize();
    }

    struct UpdateOutcome {
        bool accepted = false;
        bool numericallyValid = false;
        double mahalanobis = 0.0;
        double dPos = 0.0;          ///< |Δposition| actually applied, inches
        double dHeading = 0.0;      ///< |Δθ| actually applied, radians
        double dHeadingSigned = 0.0;  ///< …and its sign, which is what the nudge carries
        bool clamped = false;
    };

    static constexpr double kUnbounded = 1e300;

    /// STEP D. Fold every valid proposal, most trusted (smallest σ) first, each gated on its own
    /// Mahalanobis distance and each drawing from the tick's remaining never-snap budget.
    void foldProposals(std::span<const CorrectionProposal> valid, double h, double predX,
                       double predY, FusionResult& out) {
        // THE BUDGET IS CHARGED FOR THE WHOLE TICK'S DEPARTURE FROM THE PREDICTION, not just
        // for the corrections. Steps B and C have already moved the position slightly away from
        // the handed prediction (the velocity-filtering residual), and a position fix teaches
        // the velocity states through the p–v cross-covariance, which comes back as MORE
        // movement on the following tick. Left uncharged, a persistent correction stream
        // inflates the published per-tick move by a third — measured at 0.16 inches against a
        // 0.12 inch budget while building this. Charging the residual first makes
        // `AppliedCorrection::dx/dy` — the §18.2 slot that AUDITS never-snap — obey the same
        // bound under this tier as under the complementary one, which is what keeps the
        // blackbox audit meaning the same thing after the swap.
        const double alreadyMoved = std::hypot(x_[kPx] - predX, x_[kPy] - predY);
        double posBudget = std::max(0.0, cfg_.maxNudgeRate.value() * h - alreadyMoved);
        double headBudget = cfg_.maxHeadingNudgeRate.value() * h;

        // Order by ascending positionStdDev. At most kMaxOrder entries; insertion sort on
        // indices, no allocation, deterministic for ties (stable: equal σ keeps arrival order).
        std::array<std::size_t, kMaxOrder> order{};
        std::size_t n = 0;
        for (std::size_t i = 0; i < valid.size() && n < kMaxOrder; ++i) {
            std::size_t j = n++;
            while (j > 0 && valid[order[j - 1]].positionStdDev.value() >
                                valid[i].positionStdDev.value()) {
                order[j] = order[j - 1];
                --j;
            }
            order[j] = i;
        }

        bool haveAudit = false;
        bool haveHeadingAudit = false;
        double headingSum = 0.0;
        double rejectMagSum = 0.0;
        int rejectCount = 0;
        lastAppliedPos_ = 0.0;
        lastAppliedHeading_ = 0.0;

        for (std::size_t k = 0; k < n; ++k) {
            const CorrectionProposal& p = valid[order[k]];
            const double zx = p.fieldPose.x().value();
            const double zy = p.fieldPose.y().value();
            const double sigma = p.positionStdDev.value();

            // ── the position channel ──────────────────────────────────────────────────
            std::array<double, 2 * kN> H{};
            H[0 * kN + kPx] = 1.0;
            H[1 * kN + kPy] = 1.0;
            const std::array<double, 2> r{zx - x_[kPx], zy - x_[kPy]};
            const double rr = sigma * sigma;
            const std::array<double, 4> R{rr, 0.0, 0.0, rr};

            UpdateOutcome o{};
            const bool wellFormed = std::isfinite(zx) && std::isfinite(zy) &&
                                    std::isfinite(sigma) && sigma > 0.0;
            if (wellFormed) {
                applyUpdate(H.data(), 2, r.data(), R.data(), /*mayMoveHeading=*/false,
                            /*mayMovePosition=*/true, posBudget, headBudget, cfg_.gateSigma, o);
            }
            // A malformed proposal fails the gate for the honest reason: the gate accepts only a
            // FINITE distance at or under gateSigma, and a NaN satisfies no inequality. It is
            // reported as a Mahalanobis rejection because that is literally the test it failed.
            if (o.accepted) {
                ++acceptedFixes_;
                out.applied = true;
                out.appliedConfidence = std::max(out.appliedConfidence,
                                                 std::clamp(p.confidence, 0.0, 1.0));
                posBudget = std::max(0.0, posBudget - o.dPos);
                lastAppliedPos_ += o.dPos;
                travelSinceFix_ = 0.0;  // the accumulated systematic bias was corrected
                timeSinceFix_ = 0.0;
                out.clamped = out.clamped || o.clamped;
                if (!haveAudit) {  // the most-trusted accepted fix (we are in ascending σ)
                    out.audit.residualX = units::Length{r[0]};
                    out.audit.residualY = units::Length{r[1]};
                    out.audit.mahalanobis = o.mahalanobis;
                    out.audit.reason = diag::GateReason::Accepted;
                    haveAudit = true;
                }
            } else {
                ++rejectedFixes_;
                out.gated = true;
                ++rejectCount;
                rejectMagSum += std::isfinite(r[0]) && std::isfinite(r[1])
                                    ? std::hypot(r[0], r[1])
                                    : 0.0;
                if (!haveAudit) {  // the first rejection, if nothing has been accepted yet
                    out.audit.residualX = units::Length{r[0]};
                    out.audit.residualY = units::Length{r[1]};
                    out.audit.mahalanobis = o.mahalanobis;
                    out.audit.reason = diag::GateReason::RejectedMahalanobis;
                    haveAudit = true;
                }
            }

            // ── the heading channel, gated INDEPENDENTLY (E3's D4, preserved) ─────────
            if (!p.providesHeading) {
                continue;
            }
            const double innoH =
                math::Angle::radians(x_[kTh]).errorTo(p.fieldPose.heading());
            std::array<double, kN> Hh{};
            Hh[kTh] = 1.0;
            const double sh = cfg_.headingStdDev.value();
            const double Rh = sh * sh;
            UpdateOutcome oh{};
            if (std::isfinite(innoH)) {
                applyUpdate(Hh.data(), 1, &innoH, &Rh, /*mayMoveHeading=*/true,
                            /*mayMovePosition=*/true, posBudget, headBudget, cfg_.gateSigma, oh);
            }
            if (oh.accepted) {
                out.headingApplied = true;
                headingSum += oh.dHeadingSigned;
                headBudget = std::max(0.0, headBudget - oh.dHeading);
                posBudget = std::max(0.0, posBudget - oh.dPos);
                lastAppliedPos_ += oh.dPos;
                lastAppliedHeading_ += oh.dHeading;
                rotSinceHeadingFix_ = 0.0;
                timeSinceHeadingFix_ = 0.0;
                out.headingClamped = out.headingClamped || oh.clamped;
            } else {
                out.headingGated = true;
            }
            if (!haveHeadingAudit) {
                out.audit.residualHeading = units::AngleDim{innoH};
                haveHeadingAudit = true;
            }
        }

        // `headingSum` accumulates ONLY inside the branch that sets `headingApplied`, so the
        // two cannot disagree and no guard is needed here. That was not obvious enough to
        // assume: a guard WAS written, and the mutation harness proved it could never fire —
        // a defensive line no mutation can kill is a line that should not be there.
        out.headingNudge = units::AngleDim{headingSum};

        // ── T2: the re-init bookkeeping ───────────────────────────────────────────────
        if (out.applied) {
            consecutiveRejects_ = 0;
            rejectSum_ = 0.0;
        } else if (rejectCount > 0) {
            consecutiveRejects_ += rejectCount;
            rejectSum_ += rejectMagSum;
        }
        // Ticks on which nothing was proposed neither count nor reset: a corrector going quiet
        // is not evidence either way, and erasing the evidence would mean a source that stutters
        // could never accumulate a case.
        maybeReinit(out);

    }

    void maybeReinit(FusionResult& out) {
        if (consecutiveRejects_ < cfg_.reinitRejectCount) {
            return;
        }
        const double meanInnovation =
            rejectSum_ / static_cast<double>(std::max(1, consecutiveRejects_));
        if (meanInnovation < cfg_.reinitInnovation.value()) {
            return;
        }
        if (reinitCount_ > 0 && (elapsed_ - lastReinitAt_) < cfg_.reinitCooldown.value()) {
            return;  // the rate limit: an estimator that re-inits every tick has not recovered
        }
        // THE RULING (header, T2): the STATE is not touched. Only the belief about how wrong it
        // might be is reset, and only for the states the trigger is evidence about — position,
        // and the velocity that carried it there. θ is left alone: the trigger is a POSITION
        // innovation and says nothing about the IMU.
        const double sp = cfg_.initialPosStdDev.value();
        const double sv = cfg_.initialVelStdDev.value();
        for (std::size_t i = 0; i < kN; ++i) {
            if (i == kTh) {
                continue;
            }
            for (std::size_t j = 0; j < kN; ++j) {
                if (j == kTh) {
                    continue;
                }
                at(P_, i, j) = 0.0;
            }
        }
        at(P_, kPx, kPx) = sp * sp;
        at(P_, kPy, kPy) = sp * sp;
        at(P_, kVx, kVx) = sv * sv;
        at(P_, kVy, kVy) = sv * sv;
        ++reinitCount_;
        lastReinitAt_ = elapsed_;
        consecutiveRejects_ = 0;
        rejectSum_ = 0.0;
        travelSinceFix_ = 0.0;  // P now carries the whole doubt; the accumulator starts over
        timeSinceFix_ = 0.0;
        // DECLARED, not silent. This overwrites the rejection verdict on purpose: on the tick a
        // filter admits it is lost, "I rejected a fix" is the less important half of the story.
        out.audit.reason = diag::GateReason::CovarianceReinit;
    }

    /// One measurement update, for m ∈ {1, 2} rows. Computes the Mahalanobis distance FIRST and
    /// applies nothing if it fails the gate — so a rejected fix leaves the state and the
    /// covariance untouched, which is what makes "outliers cannot inflate the state" a property
    /// of the code rather than a hope.
    ///
    /// `mayMoveHeading` / `mayMovePosition` zero the corresponding gain rows. A zeroed row is a
    /// deliberately SUBOPTIMAL gain, and so is the rate clamp below; the Joseph form is exactly
    /// correct for any gain, which is the whole reason it is used here.
    void applyUpdate(const double* H, std::size_t m, const double* r, const double* R,
                     bool mayMoveHeading, bool mayMovePosition, double posBudget,
                     double headBudget, double gate, UpdateOutcome& out) {
        // PHt (kN x m)
        std::array<double, kN * 2> PHt{};
        for (std::size_t i = 0; i < kN; ++i) {
            for (std::size_t a = 0; a < m; ++a) {
                double sum = 0.0;
                for (std::size_t j = 0; j < kN; ++j) {
                    sum += at(P_, i, j) * H[a * kN + j];
                }
                PHt[i * 2 + a] = sum;
            }
        }
        // S = H PHt + R  (m x m)
        std::array<double, 4> S{};
        for (std::size_t a = 0; a < m; ++a) {
            for (std::size_t b = 0; b < m; ++b) {
                double sum = 0.0;
                for (std::size_t j = 0; j < kN; ++j) {
                    sum += H[a * kN + j] * PHt[j * 2 + b];
                }
                S[a * 2 + b] = sum + R[a * m + b];
            }
        }
        // S⁻¹
        std::array<double, 4> Sinv{};
        if (m == 1) {
            if (!(std::abs(S[0]) > 0.0) || !std::isfinite(S[0])) {
                ++numericGuardTrips_;
                return;
            }
            Sinv[0] = 1.0 / S[0];
        } else {
            const double det = S[0] * S[3] - S[1] * S[2];
            if (!std::isfinite(det) || std::abs(det) < 1e-300) {
                ++numericGuardTrips_;
                return;
            }
            Sinv[0] = S[3] / det;
            Sinv[1] = -S[1] / det;
            Sinv[2] = -S[2] / det;
            Sinv[3] = S[0] / det;
        }
        // ν² = rᵀ S⁻¹ r — THE gate (T4/T5). Written as `!(d2 >= 0 && d2 <= gate²)` so a NaN
        // innovation, a NaN σ or a degenerate S all land on "rejected" rather than sailing
        // through an inverted comparison.
        double d2 = 0.0;
        for (std::size_t a = 0; a < m; ++a) {
            for (std::size_t b = 0; b < m; ++b) {
                d2 += r[a] * Sinv[a * 2 + b] * r[b];
            }
        }
        out.mahalanobis = (std::isfinite(d2) && d2 >= 0.0) ? std::sqrt(d2) : 0.0;
        if (!(std::isfinite(d2) && d2 >= 0.0 && d2 <= gate * gate)) {
            return;  // rejected: nothing is touched
        }

        // K = PHt S⁻¹ (kN x m), with the forbidden rows zeroed.
        std::array<double, kN * 2> K{};
        for (std::size_t i = 0; i < kN; ++i) {
            const bool blocked = (i == kTh && !mayMoveHeading) ||
                                 ((i == kPx || i == kPy) && !mayMovePosition);
            for (std::size_t a = 0; a < m; ++a) {
                if (blocked) {
                    K[i * 2 + a] = 0.0;
                    continue;
                }
                double sum = 0.0;
                for (std::size_t b = 0; b < m; ++b) {
                    sum += PHt[i * 2 + b] * Sinv[b * 2 + a];
                }
                K[i * 2 + a] = sum;
            }
        }
        // δ = K r, and the never-snap bound applied AS A GAIN REDUCTION (header).
        Vec delta{};
        for (std::size_t i = 0; i < kN; ++i) {
            double sum = 0.0;
            for (std::size_t a = 0; a < m; ++a) {
                sum += K[i * 2 + a] * r[a];
            }
            delta[i] = sum;
        }
        const double dPos = std::hypot(delta[kPx], delta[kPy]);
        const double dTh = std::abs(delta[kTh]);
        double scale = 1.0;
        if (dPos > posBudget && dPos > 0.0) {
            scale = std::min(scale, posBudget / dPos);
        }
        if (dTh > headBudget && dTh > 0.0) {
            scale = std::min(scale, headBudget / dTh);
        }
        if (!std::isfinite(scale) || scale < 0.0) {
            ++numericGuardTrips_;
            return;
        }
        if (scale < 1.0) {
            out.clamped = true;
            for (std::size_t i = 0; i < kN * 2; ++i) {
                K[i] *= scale;
            }
            for (std::size_t i = 0; i < kN; ++i) {
                delta[i] *= scale;
            }
        }

        // Joseph: P⁺ = (I − K H) P⁻ (I − K H)ᵀ + K R Kᵀ. Correct for ANY gain — which is
        // exactly what the clamp above and the blocked rows above require.
        Mat A{};
        for (std::size_t i = 0; i < kN; ++i) {
            for (std::size_t j = 0; j < kN; ++j) {
                double sum = (i == j) ? 1.0 : 0.0;
                for (std::size_t a = 0; a < m; ++a) {
                    sum -= K[i * 2 + a] * H[a * kN + j];
                }
                at(A, i, j) = sum;
            }
        }
        Mat AP{};
        multiply(A, P_, AP);
        Mat next{};
        multiplyTransposed(AP, A, next);
        for (std::size_t i = 0; i < kN; ++i) {
            for (std::size_t j = 0; j < kN; ++j) {
                double sum = 0.0;
                for (std::size_t a = 0; a < m; ++a) {
                    for (std::size_t b = 0; b < m; ++b) {
                        sum += K[i * 2 + a] * R[a * m + b] * K[j * 2 + b];
                    }
                }
                at(next, i, j) += sum;
            }
        }
        // Nothing non-finite is ever allowed to enter the state or the covariance: a single NaN
        // in P is permanent and silent, and the estimate must degrade rather than die (F4).
        for (std::size_t i = 0; i < kN; ++i) {
            if (!std::isfinite(delta[i])) {
                ++numericGuardTrips_;
                return;
            }
            for (std::size_t j = 0; j < kN; ++j) {
                if (!std::isfinite(at(next, i, j))) {
                    ++numericGuardTrips_;
                    return;
                }
            }
        }
        P_ = next;
        symmetrize();
        for (std::size_t i = 0; i < kN; ++i) {
            x_[i] += delta[i];
        }
        x_[kTh] = math::Angle::radians(x_[kTh]).radians();  // keep θ in (-π, π]
        out.accepted = true;
        out.numericallyValid = true;
        // Recomputed from the SCALED correction rather than multiplied out, so the budget
        // accounting is exact to the last bit and a never-snap assertion cannot fail on a
        // rounding artefact of the clamp arithmetic.
        out.dPos = std::hypot(delta[kPx], delta[kPy]);
        out.dHeading = std::abs(delta[kTh]);
        out.dHeadingSigned = delta[kTh];
    }

    static void multiply(const Mat& a, const Mat& b, Mat& outM) noexcept {
        for (std::size_t i = 0; i < kN; ++i) {
            for (std::size_t j = 0; j < kN; ++j) {
                double sum = 0.0;
                for (std::size_t k = 0; k < kN; ++k) {
                    sum += a[i * kN + k] * b[k * kN + j];
                }
                outM[i * kN + j] = sum;
            }
        }
    }
    /// outM = a · bᵀ
    static void multiplyTransposed(const Mat& a, const Mat& b, Mat& outM) noexcept {
        for (std::size_t i = 0; i < kN; ++i) {
            for (std::size_t j = 0; j < kN; ++j) {
                double sum = 0.0;
                for (std::size_t k = 0; k < kN; ++k) {
                    sum += a[i * kN + k] * b[j * kN + k];
                }
                outM[i * kN + j] = sum;
            }
        }
    }
    /// A covariance is symmetric by definition; in floating point it drifts. Forcing it back
    /// costs 10 additions per update and removes an entire class of slow-motion failure, in
    /// which the asymmetry grows until `S` is no longer positive and the gate starts accepting
    /// or rejecting for reasons that have nothing to do with the measurement.
    void symmetrize() noexcept {
        for (std::size_t i = 0; i < kN; ++i) {
            for (std::size_t j = i + 1; j < kN; ++j) {
                const double avg = 0.5 * (at(P_, i, j) + at(P_, j, i));
                at(P_, i, j) = avg;
                at(P_, j, i) = avg;
            }
        }
    }

    /// Matches `Localizer::kMaxCorrectors`. Kept as its own constant rather than including
    /// localizer.hpp: a fusion policy must not depend on the orchestrator that owns it.
    static constexpr std::size_t kMaxOrder = 4;

    EkfFusionConfig cfg_;
    Vec x_{};
    Mat P_{};
    bool initialized_ = false;
    double lastX_ = 0.0;
    double lastY_ = 0.0;
    double lastHeading_ = 0.0;
    double lastHeadingNudge_ = 0.0;
    double elapsed_ = 0.0;
    double lastReinitAt_ = 0.0;
    double lastAppliedPos_ = 0.0;      // corrections only, summed over the tick's proposals
    double lastAppliedHeading_ = 0.0;  // |the increment emitted last tick|
    // The systematic-growth accumulators (addProcessNoise). Reset when a fix is accepted.
    double travelSinceFix_ = 0.0;
    double timeSinceFix_ = 0.0;
    double rotSinceHeadingFix_ = 0.0;
    double timeSinceHeadingFix_ = 0.0;
    double rejectSum_ = 0.0;
    int consecutiveRejects_ = 0;
    std::uint32_t reinitCount_ = 0;
    std::uint32_t resyncCount_ = 0;
    std::uint32_t numericGuardTrips_ = 0;
    std::uint32_t acceptedFixes_ = 0;
    std::uint32_t rejectedFixes_ = 0;
};

}  // namespace shulib::localization
