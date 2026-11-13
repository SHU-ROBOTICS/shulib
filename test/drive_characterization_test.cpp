// calibration::DriveCharacterization (chunk R5a Part A, A.2) — the DATA the GAINS and REST steps
// produce and Part B will persist. Each case names the bug it catches:
//   D1  the shape: constexpr-constructible, trivially copyable (no pointer, no allocation — it is
//       written to a card as it stands), and the default is Unset with no section
//   D2  combined() is the field-by-field MEAN (D2 of the brief), kA carried; kVAsymmetry()
//   D3  valid() refuses what cannot hold together, NAMING the field — source, each side's gains
//       (NaN, negative AND +Inf), the kA-below-resolution rule, the volts range (a negative low end
//       too), samples, members, the ramp timing, every rest number (the encoder motion included),
//       the provenance's terminator
//   D4  driftDegPerMin() converts
//   D5  driftTurnBiasBound() and structureRatioDefined()
// NOTHING HERE IS MEASURED: the numbers are the test's.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

#include "shulib/calibration/drive_characterization.hpp"

using shulib::calibration::CharacterizationSource;
using shulib::calibration::DriveCharacterization;
using shulib::calibration::RestSummary;
using shulib::calibration::SideCharacterization;

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

/// A characterization that holds together, both sections present.
DriveCharacterization good() {
    DriveCharacterization c;
    c.source = CharacterizationSource::Measured;
    c.hasGains = true;
    c.hasRest = true;
    auto side = [](double kS, double kV, double kA) {
        SideCharacterization s;
        s.gains = {.kS = kS, .kV = kV, .kA = kA};
        s.residualRms = 0.02;
        s.alpha = 0.92;
        s.voltsMin = 1.1;
        s.voltsMax = 6.0;
        s.samples = 700;
        s.members = 5;
        return s;
    };
    c.left = side(1.0, 0.16, 0.02);
    c.right = side(1.1, 0.168, 0.024);
    c.rampTiming = {.count = 2000, .overBudget = 3, .overDouble = 1, .meanS = 0.01,
                    .sigmaS = 0.0004, .minS = 0.009, .maxS = 0.021};
    c.rest.driftRadPerS = 2e-4;
    c.rest.driftSigmaWhite = 6e-7;
    c.rest.driftSigmaWalk = 1.2e-4;
    c.rest.structureRatio = 0.9;
    c.rest.headingSigmaRad = 8.7e-4;
    c.rest.stepRateSigma = 0.12;
    c.rest.yawRateMean = 2e-4;
    c.rest.yawRateSigma = 8.7e-3;
    c.rest.maxHeadingStepRad = 5e-3;
    c.rest.durationS = 60.0;
    c.rest.speedSigma[0] = 0.01;
    c.rest.speedSigma[1] = 0.012;
    c.rest.encoderTurnRad = -0.004;
    c.rest.encoderTurnMaxRad = 0.006;
    c.rest.encoderMoveMaxIn = 0.05;
    c.rest.samples = 6001;
    c.rest.timing = c.rampTiming;
    std::strcpy(c.provenance, "host test");
    return c;
}

/// valid() on `c`, and the field it names.
bool check(const DriveCharacterization& c, char* why, std::size_t cap) { return c.valid(why, cap); }

}  // namespace

// ═══ D1. The shape ══════════════════════════════════════════════════════════════════
// Bug caught: a pointer, a std::string or a virtual sneaking into a struct Part B writes to a card
// byte by byte and embeds in a constexpr RobotConfig.
static_assert(std::is_trivially_copyable_v<DriveCharacterization>,
              "DriveCharacterization must stay trivially copyable (no pointers, no allocation)");
static_assert(std::is_standard_layout_v<DriveCharacterization>);
constexpr DriveCharacterization kConstexprDefault{};
static_assert(kConstexprDefault.source == CharacterizationSource::Unset);
static_assert(!kConstexprDefault.hasGains && !kConstexprDefault.hasRest);
static_assert(static_cast<int>(CharacterizationSource::Unset) == 0);   // persisted values
static_assert(static_cast<int>(CharacterizationSource::Measured) == 1);

TEST_CASE("DriveCharacterization D1: the default is Unset with no section, and valid") {
    char why[160] = "";
    CHECK(DriveCharacterization{}.valid(why, sizeof why));
    CHECK(DriveCharacterization{}.combined().kV == 0.0);
    CHECK(DriveCharacterization{}.kVAsymmetry() == 0.0);
}

// ═══ D2. combined() and kVAsymmetry() ════════════════════════════════════════════════
// Bug caught: combined = LEFT only (or right), a kA dropped from the mean.
TEST_CASE("DriveCharacterization D2: combined() is the field-by-field mean, kA carried; "
          "kVAsymmetry() is the kV difference over their mean") {
    const DriveCharacterization c = good();
    constexpr DriveCharacterization k{};
    static_assert(k.combined().kS == 0.0);  // combined() is usable at compile time
    CHECK(c.combined().kS == rel(1.05, 1e-12));
    CHECK(c.combined().kV == rel(0.164, 1e-12));
    CHECK(c.combined().kA == rel(0.022, 1e-12));
    CHECK(c.kVAsymmetry() == rel(0.008 / 0.164, 1e-12));
    DriveCharacterization swapped = c;
    std::swap(swapped.left, swapped.right);
    CHECK(swapped.combined().kV == c.combined().kV);
    CHECK(swapped.kVAsymmetry() == c.kVAsymmetry());
}

// ═══ D3. valid(), field by field ════════════════════════════════════════════════════
TEST_CASE("DriveCharacterization D3: valid() refuses each broken field and names it") {
    char why[160];
    REQUIRE(good().valid(why, sizeof why));
    struct Case {
        const char* name;
        void (*breakIt)(DriveCharacterization&);
        const char* field;
    };
    const Case cases[] = {
        {"unknown source", [](DriveCharacterization& c) { c.source = static_cast<CharacterizationSource>(7); }, "source"},
        {"Unset with a section", [](DriveCharacterization& c) { c.source = CharacterizationSource::Unset; }, "source"},
        {"Measured, no section", [](DriveCharacterization& c) { c.hasGains = false; c.hasRest = false; }, "source"},
        {"left kV 0", [](DriveCharacterization& c) { c.left.gains.kV = 0.0; }, "left.gains.kV"},
        {"right kS NaN", [](DriveCharacterization& c) { c.right.gains.kS = std::numeric_limits<double>::quiet_NaN(); }, "right.gains.kS"},
        {"right kS negative", [](DriveCharacterization& c) { c.right.gains.kS = -0.1; }, "right.gains.kS"},
        {"left kA negative", [](DriveCharacterization& c) { c.left.gains.kA = -1e-9; }, "left.gains.kA"},
        // +Inf passes every ">= 0" and "> 0" — only the isfinite half refuses it (the R5a review:
        // each of those halves survived removal; Part B's decoder relies on them).
        {"left kS +Inf", [](DriveCharacterization& c) { c.left.gains.kS = std::numeric_limits<double>::infinity(); }, "left.gains.kS"},
        {"right kV +Inf", [](DriveCharacterization& c) { c.right.gains.kV = std::numeric_limits<double>::infinity(); }, "right.gains.kV"},
        {"left kA +Inf", [](DriveCharacterization& c) { c.left.gains.kA = std::numeric_limits<double>::infinity(); }, "left.gains.kA"},
        {"right residual +Inf", [](DriveCharacterization& c) { c.right.residualRms = std::numeric_limits<double>::infinity(); }, "right.residualRms"},
        {"below resolution, kA not 0", [](DriveCharacterization& c) { c.right.kABelowResolution = true; }, "right.gains.kA"},
        {"residual negative", [](DriveCharacterization& c) { c.left.residualRms = -1.0; }, "left.residualRms"},
        {"alpha infinite", [](DriveCharacterization& c) { c.right.alpha = std::numeric_limits<double>::infinity(); }, "right.alpha"},
        {"voltsMax over 12", [](DriveCharacterization& c) { c.left.voltsMax = 12.5; }, "left.voltsMin/voltsMax"},
        {"voltsMin over voltsMax", [](DriveCharacterization& c) { c.right.voltsMin = 7.0; }, "right.voltsMin/voltsMax"},
        {"voltsMin negative", [](DriveCharacterization& c) { c.left.voltsMin = -0.1; }, "left.voltsMin/voltsMax"},
        {"samples 0", [](DriveCharacterization& c) { c.left.samples = 0; }, "left.samples"},
        {"members 0", [](DriveCharacterization& c) { c.right.members = 0; }, "right.members"},
        {"members 22", [](DriveCharacterization& c) { c.left.members = 22; }, "left.members"},
        {"ramp timing", [](DriveCharacterization& c) { c.rampTiming.overBudget = c.rampTiming.count + 1; }, "rampTiming"},
        {"rest drift NaN", [](DriveCharacterization& c) { c.rest.driftRadPerS = std::numeric_limits<double>::quiet_NaN(); }, "rest"},
        {"rest sigma negative", [](DriveCharacterization& c) { c.rest.headingSigmaRad = -1e-9; }, "rest"},
        {"rest speed sigma negative", [](DriveCharacterization& c) { c.rest.speedSigma[1] = -0.1; }, "rest"},
        {"rest encoder turn NaN", [](DriveCharacterization& c) { c.rest.encoderTurnRad = std::numeric_limits<double>::quiet_NaN(); }, "rest"},
        {"rest largest turn negative", [](DriveCharacterization& c) { c.rest.encoderTurnMaxRad = -1e-9; }, "rest"},
        {"rest largest move negative", [](DriveCharacterization& c) { c.rest.encoderMoveMaxIn = -1e-9; }, "rest"},
        {"rest largest move +Inf", [](DriveCharacterization& c) { c.rest.encoderMoveMaxIn = std::numeric_limits<double>::infinity(); }, "rest"},
        {"rest final turn past the largest", [](DriveCharacterization& c) { c.rest.encoderTurnRad = -2.0 * c.rest.encoderTurnMaxRad; }, "rest.encoderTurnRad"},
        {"rest duration 0", [](DriveCharacterization& c) { c.rest.durationS = 0.0; }, "rest.durationS"},
        {"rest samples 0", [](DriveCharacterization& c) { c.rest.samples = 0; }, "rest.samples"},
        {"rest timing", [](DriveCharacterization& c) { c.rest.timing.minS = 1.0; }, "rest.timing"},
        {"provenance unterminated", [](DriveCharacterization& c) { std::memset(c.provenance, 'x', sizeof c.provenance); }, "provenance"},
    };
    for (const Case& k : cases) {
        CAPTURE(k.name);
        DriveCharacterization c = good();
        k.breakIt(c);
        CHECK_FALSE(check(c, why, sizeof why));
        CHECK(std::strstr(why, k.field) != nullptr);
        for (const char* p = why; *p != '\0'; ++p) {
            CHECK((*p >= 0x20 && *p <= 0x7E && *p != '%'));
        }
    }
    // A section that is ABSENT is not judged: garbage in the rest numbers of a GAINS-only record
    // is fine (Part B's merge keeps sections apart).
    DriveCharacterization gainsOnly = good();
    gainsOnly.hasRest = false;
    gainsOnly.rest.driftRadPerS = std::numeric_limits<double>::quiet_NaN();
    CHECK(gainsOnly.valid(why, sizeof why));
    // kA below resolution with kA exactly 0 is valid.
    DriveCharacterization zeroKa = good();
    zeroKa.left.kABelowResolution = true;
    zeroKa.left.gains.kA = 0.0;
    CHECK(zeroKa.valid(why, sizeof why));
    // `why` may be null.
    DriveCharacterization broken = good();
    broken.left.samples = 0;
    CHECK_FALSE(broken.valid(nullptr, 0));
}

// ═══ D4. driftDegPerMin() ═══════════════════════════════════════════════════════════
TEST_CASE("DriveCharacterization D4: driftDegPerMin converts rad/s to degrees per minute") {
    RestSummary r;
    r.driftRadPerS = 3.14159265358979323846 / 180.0 / 60.0;  // one degree per minute
    CHECK(r.driftDegPerMin() == rel(1.0, 1e-12));
}

// ═══ D5. What a REST summary says about itself ══════════════════════════════════════
// Bug caught (the R5a review of 2026-10-04): a rotation the encoders saw under turnBound biased the
// drift with no trace in the record, and ρ = 0 from a noiseless heading read as random-walk
// evidence. driftTurnBiasBound() is 3·encoderTurnMaxRad/T (rest_capture.hpp derives it; the
// RestCapture cases check it against a real capture); structureRatioDefined() is σ_line > 0.
TEST_CASE("DriveCharacterization D5: driftTurnBiasBound is 3 x the largest encoder turn over the "
          "duration, and the structure ratio is defined only with heading noise") {
    RestSummary r;
    CHECK(r.driftTurnBiasBound() == 0.0);  // durationS 0: no capture, no bound (never a division)
    r.durationS = 60.0;
    CHECK(r.driftTurnBiasBound() == 0.0);  // a robot that never turned
    r.encoderTurnMaxRad = 0.05;
    CHECK(r.driftTurnBiasBound() == rel(3.0 * 0.05 / 60.0, 1e-15));
    CHECK_FALSE(r.structureRatioDefined());
    r.headingSigmaRad = 1e-12;
    CHECK(r.structureRatioDefined());
}
