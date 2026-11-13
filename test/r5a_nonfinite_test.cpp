// R5a Part A x DIAG1 Part B — written by R5a's review round (2026-10-04) as a draft, ADDED AT THE MERGE
// (2026-10-06, the coordinator): these cases need DIAG1 Part B's hal::medianInPlace (a non-finite member
// left out; a fresh NaN when none is finite), which landed after R5a Part A's base (9239538). They were
// seen green on a scratch tree of shulib-v2 (237c4fe, then fab1ebb) with R5a Part A's files overlaid.
//   N1  one member's velocity() reads NaN for the whole GAINS run: left out of its side's median — the
//       run finishes, the gains equal a clean run's
//   N2  a whole side's velocity() reads NaN for 20 ticks while driving: those samples are counted
//       NonFinite on THAT side only, the run is not poisoned, the gains stay exact
//   N3  one member's velocity() reads NaN through a whole REST capture: the capture finishes and its
//       speed σ is finite
//   N4  a whole side's velocity() reads NaN on ONE capture tick: what the REST summary does with it
#include "doctest.h"

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <span>

#include "shulib/calibration/drive_characterizer.hpp"
#include "shulib/calibration/rest_capture.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/sim/scenario.hpp"

using namespace shulib;
using calibration::CharacterizerPhase;
using calibration::DriveCharacterizer;
using calibration::DriveSide;
using calibration::FitExclusion;
using calibration::RestCapture;
using calibration::RestPhase;
using calibration::SideMember;
using hal::fake::FakeMotor;

namespace {
constexpr double kDt = 0.01;
constexpr double kR = 3.25 / 2.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Rig {
    static constexpr std::size_t kPer = 5;
    kinematics::TankKinematics kin{units::Length{12.5}};
    sim::SimHarness h;
    std::array<FakeMotor, 2 * (kPer - 1)> extras{};
    std::array<SideMember, kPer> left{};
    std::array<SideMember, kPer> right{};
    explicit Rig(const sim::SimHarnessConfig& cfg) : h{kin, cfg} {
        for (int w = 0; w < 2; ++w) {
            std::array<FakeMotor*, kPer - 1> ex{};
            for (std::size_t k = 1; k < kPer; ++k) {
                ex[k - 1] = &extras[static_cast<std::size_t>(w) * (kPer - 1) + (k - 1)];
            }
            h.plant().attachCoupledMembers(w, std::span<FakeMotor* const>{ex.data(), ex.size()});
        }
        for (std::size_t k = 0; k < kPer; ++k) {
            left[k] = SideMember{&member(0, k), 1, static_cast<std::int8_t>(11 + k)};
            right[k] = SideMember{&member(1, k), 1, static_cast<std::int8_t>(16 + k)};
        }
    }
    FakeMotor& member(int w, std::size_t k) {
        return k == 0 ? h.motor(w) : extras[static_cast<std::size_t>(w) * (kPer - 1) + (k - 1)];
    }
    DriveSide side(int w) {
        auto& a = w == 0 ? left : right;
        return DriveSide{std::span<const SideMember>{a.data(), a.size()}, units::Length{kR}};
    }
};

sim::SimHarnessConfig gains() {
    sim::SimHarnessConfig c;
    c.plant.wheelFf = {.kS = 1.0, .kV = 0.160, .kA = 0.020};
    c.plant.wheelFfOverride[1] = control::FeedforwardGains{.kS = 1.1, .kV = 0.168, .kA = 0.024};
    return c;
}

/// Run the GAINS step; `poison(rig, ch)` runs after each plant step (it may overwrite readings).
template <typename Poison>
DriveCharacterizer& runGains(Rig& rig, std::optional<DriveCharacterizer>& ch, Poison poison) {
    ch.emplace(rig.side(0), rig.side(1), units::Length{12.5}, rig.h.imu(), rig.h.clock(), rig.h.battery());
    ch->start("draft");
    for (int i = 0; i < 8000; ++i) {
        ch->tick();
        if (ch->phase() == CharacterizerPhase::Done || ch->phase() == CharacterizerPhase::Aborted) {
            break;
        }
        rig.h.plant().step(units::Time{kDt});
        poison(rig, *ch);
    }
    return *ch;
}
}  // namespace

TEST_CASE("R5a x DIAG1 N1: one member's velocity NaN for the whole GAINS run is left out of the median") {
    Rig clean{gains()};
    std::optional<DriveCharacterizer> a;
    runGains(clean, a, [](Rig&, DriveCharacterizer&) {});
    REQUIRE(a->phase() == CharacterizerPhase::Done);
    Rig dirty{gains()};
    std::optional<DriveCharacterizer> b;
    runGains(dirty, b, [](Rig& r, DriveCharacterizer&) { r.member(1, 2).setVelocity(units::AngularVelocity{kNaN}); });
    INFO(b->reason());
    REQUIRE(b->phase() == CharacterizerPhase::Done);
    for (std::size_t s = 0; s < 2; ++s) {
        CHECK(b->report().fit[s].gains.kS == doctest::Approx(a->report().fit[s].gains.kS).epsilon(1e-9).scale(0.0));
        CHECK(b->report().fit[s].gains.kV == doctest::Approx(a->report().fit[s].gains.kV).epsilon(1e-9).scale(0.0));
        CHECK(b->report().fit[s].gains.kA == doctest::Approx(a->report().fit[s].gains.kA).epsilon(1e-9).scale(0.0));
    }
}

TEST_CASE("R5a x DIAG1 N2: a whole side's velocity NaN for 20 driving ticks is counted NonFinite on that "
          "side only, and the gains stay exact") {
    Rig rig{gains()};
    std::optional<DriveCharacterizer> ch;
    int poisoned = 0;
    runGains(rig, ch, [&](Rig& r, DriveCharacterizer& c) {
        if (c.phase() == CharacterizerPhase::Driving && c.status().segment == 2 && poisoned < 20
            && c.lastSample().speed[0] > 10.0) {
            for (std::size_t k = 0; k < Rig::kPer; ++k) {
                r.member(0, k).setVelocity(units::AngularVelocity{kNaN});
            }
            ++poisoned;
        }
    });
    INFO(ch->reason());
    REQUIRE(ch->phase() == CharacterizerPhase::Done);
    const auto nf = static_cast<std::size_t>(FitExclusion::NonFinite) - 1;
    const auto& seg = ch->report().segments[2];
    MESSAGE("LEFT NonFinite " << seg.excluded[0][nf] << ", RIGHT NonFinite " << seg.excluded[1][nf]);
    // 20 ISOLATED NaN readings (the trigger reads the last sample's speed, which a NaN tick makes NaN,
    // so a poisoned tick is always followed by a clean one): each spoils two samples — the one it
    // ends and the one it starts.
    CHECK(seg.excluded[0][nf] == 40U);
    CHECK(seg.excluded[1][nf] == 0U);
    CHECK(ch->report().fit[0].gains.kV == doctest::Approx(0.160).epsilon(1e-6).scale(0.0));
    CHECK(ch->report().fit[1].gains.kV == doctest::Approx(0.168).epsilon(1e-6).scale(0.0));
}

namespace {
template <typename Poison>
RestCapture& runRest(Rig& rig, std::optional<RestCapture>& cap, Poison poison) {
    cap.emplace(rig.side(0), rig.side(1), units::Length{12.5}, rig.h.imu(), rig.h.clock());
    cap->start();
    for (int i = 0; i < 20000; ++i) {
        cap->tick();
        if (cap->phase() == RestPhase::Done || cap->phase() == RestPhase::Aborted) {
            break;
        }
        rig.h.plant().step(units::Time{kDt});
        poison(rig, *cap);
    }
    return *cap;
}
}  // namespace

TEST_CASE("R5a x DIAG1 N3: one member's velocity NaN through a REST capture is left out") {
    Rig rig{sim::SimHarnessConfig{}};
    std::optional<RestCapture> cap;
    runRest(rig, cap, [](Rig& r, RestCapture&) { r.member(0, 1).setVelocity(units::AngularVelocity{kNaN}); });
    INFO(cap->reason());
    REQUIRE(cap->phase() == RestPhase::Done);
    CHECK(std::isfinite(cap->summary().speedSigma[0]));
    char why[160];
    calibration::DriveCharacterization c;
    c.source = calibration::CharacterizationSource::Measured;
    c.hasRest = true;
    c.rest = cap->summary();
    CHECK(c.valid(why, sizeof why));
}

TEST_CASE("R5a x DIAG1 N4: a whole side's velocity NaN on ONE capture tick -- what the summary does") {
    Rig rig{sim::SimHarnessConfig{}};
    std::optional<RestCapture> cap;
    bool done = false;
    runRest(rig, cap, [&](Rig& r, RestCapture& c) {
        const bool on = !done && c.phase() == RestPhase::Capture && c.captureSeconds() > 30.0;
        for (std::size_t k = 0; k < Rig::kPer; ++k) {
            r.member(0, k).setVelocity(units::AngularVelocity{on ? kNaN : 0.0});
        }
        done = done || on;
    });
    INFO(cap->reason());
    REQUIRE(cap->phase() == RestPhase::Done);
    const auto& s = cap->summary();
    MESSAGE("speedSigma LEFT " << s.speedSigma[0] << " RIGHT " << s.speedSigma[1]);
    char why[160] = "";
    calibration::DriveCharacterization c;
    c.source = calibration::CharacterizationSource::Measured;
    c.hasRest = true;
    c.rest = s;
    const bool ok = c.valid(why, sizeof why);
    MESSAGE("summary valid: " << ok << " -- " << why);
    REQUIRE(done);
    // Pinned at the merge (2026-10-06, the coordinator), from the behaviour seen above: the NaN tick is
    // LEFT OUT of the left side's sigma and COUNTED (unreadSides), so every other reading (0) stands --
    // both sides' sigma exactly 0 -- and the summary stays valid; the capture still finishes Done.
    CHECK(s.speedSigma[0] == 0.0);
    CHECK(s.speedSigma[1] == 0.0);
    CHECK(cap->unreadSides() >= 1);
    CHECK(ok);
}
