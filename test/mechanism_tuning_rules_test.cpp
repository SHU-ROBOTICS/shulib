// mechanism/live_tuning.hpp — LIVE TUNING'S RULES (chunk TUNE1-A, 2026-11-10; the MECH-S/TUNE1 brief's D8
// (a), re-check #14, §2.2 TA3, TA8's "no kA", TA9). ADVERSARIAL: each case tries to slip ONE number past
// ONE rule — the build's own check (one case per Refusal a live change can reach), then each live bound
// the build lacks (P9) — and pins that it is refused WITH ITS SENTENCE and that NOTHING changed (the spec
// byte for byte, no undo record, no re-derive line). The step rules are pinned value by value, kA's
// additive first step included. Over the host test plant (HA-183); nothing here has run on a robot.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include "mechanism_fixtures.hpp"
#include "mechanism_live_tuning_rig.hpp"
#include "shulib/mechanism/live_tuning.hpp"

namespace {
namespace mech = shulib::mechanism;
using mech::LiveRefusal;
using mech::Refusal;
using mech::Tunable;
using tune_rig::TuneRig;

/// Apply `v` to number `t` of mechanism `k` and pin a REFUSAL: `why` (and `build` for the build's), its
/// sentence in the Applied, and nothing changed — the spec byte for byte, no undo record, no MECH line.
void refused(TuneRig& r, std::size_t k, Tunable t, double v, LiveRefusal why, Refusal build = Refusal::None) {
    mech::MechanismSpec before;
    std::memcpy(static_cast<void*>(&before), &r.tuning.spec(k), sizeof before);
    const std::size_t depth = r.tuning.undoDepth();
    const int lines = r.sink.lines.size();
    mech::Applied a{};
    CHECK_NOTHROW(a = r.tuning.apply(k, t, v));
    CHECK_FALSE(a.ok);
    CHECK(a.why == why);
    CHECK(a.build == build);
    const char* text = why == LiveRefusal::BuildRefuses ? mech::refusalText(build) : mech::liveRefusalText(why);
    REQUIRE(text != nullptr);
    CHECK(std::string{a.sentence}.find(text) != std::string::npos);
    CHECK(std::string{a.sentence}.find("refused") != std::string::npos);
    CHECK(std::memcmp(&before, &r.tuning.spec(k), sizeof before) == 0);
    CHECK(r.tuning.undoDepth() == depth);
    CHECK(r.sink.lines.size() == lines);  // no "retuned live" line, nothing re-derived
}
/// Apply and pin it ACCEPTED (its re-derive line said).
void accepted(TuneRig& r, std::size_t k, Tunable t, double v) {
    const mech::Applied a = r.tuning.apply(k, t, v);
    INFO(a.sentence);
    CHECK(a.ok);
    CHECK(a.why == LiveRefusal::None);
    CHECK(r.tuning.value(k, t) == v);
}
}  // namespace

TEST_CASE("TA3: every refusal of the BUILD a live change can reach is refused with the build's own sentence, nothing changed") {
    TuneRig fp{tune_fixture::kFlyAndPivot};  // 0 flywheel, 1 pivot (a stated stall), 2 roller
    // a power mechanism: maxVolts, its stall signature, its cap
    refused(fp, 1, Tunable::MaxVolts, 12.05, LiveRefusal::BuildRefuses, Refusal::PowerMaxVoltsIsNotIn012);
    refused(fp, 1, Tunable::MaxVolts, 0.0, LiveRefusal::BuildRefuses, Refusal::PowerMaxVoltsIsNotIn012);
    refused(fp, 1, Tunable::StallAmps, 0.0, LiveRefusal::BuildRefuses, Refusal::AStallSignatureNeedsAmpsAbove0);
    refused(fp, 1, Tunable::StallAmps, 2.5, LiveRefusal::BuildRefuses,
            Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit);
    refused(fp, 1, Tunable::StallRpm, 0.0, LiveRefusal::BuildRefuses, Refusal::AStallSignatureNeedsBelowMotorRpmAbove0);
    refused(fp, 1, Tunable::StallLasting, 0.0, LiveRefusal::BuildRefuses, Refusal::AStallSignatureNeedsLastingAbove0);
    refused(fp, 1, Tunable::CurrentCap, 2.55, LiveRefusal::BuildRefuses, Refusal::ACurrentCapMustBeAbove0AndAtMost25A);
    refused(fp, 1, Tunable::CurrentCap, -0.05, LiveRefusal::BuildRefuses, Refusal::ACurrentCapMustBeAbove0AndAtMost25A);
    refused(fp, 1, Tunable::CurrentCap, 0.0005, LiveRefusal::BuildRefuses, Refusal::ACurrentCapMustBeAtLeastOneMilliamp);
    refused(fp, 1, Tunable::CurrentCap, 1.5, LiveRefusal::BuildRefuses, Refusal::AStallSignatureMustBeBelowTheCurrentCap);
    // a flywheel: its band and its lasting
    refused(fp, 0, Tunable::AtSpeedBand, 0.0, LiveRefusal::BuildRefuses, Refusal::SpeedTheAtSpeedBandMustBe0);
    refused(fp, 0, Tunable::AtSpeedTime, -0.05, LiveRefusal::BuildRefuses, Refusal::SpeedTheAtSpeedLastingIsNegative);
    // a lift: limits, profile, reached, gravity, home and presets
    TuneRig c{tune_fixture::kCascade};
    refused(c, 0, Tunable::RangeMin, 24.0, LiveRefusal::BuildRefuses, Refusal::PositionTravelLimitsMissingOrMinMax);
    refused(c, 0, Tunable::MaxSpeed, 0.0, LiveRefusal::BuildRefuses,
            Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0);
    refused(c, 0, Tunable::MaxAcceleration, -1.0, LiveRefusal::BuildRefuses,
            Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0);
    refused(c, 0, Tunable::Within, 0.0, LiveRefusal::BuildRefuses, Refusal::PositionToleranceMustBe0);
    refused(c, 0, Tunable::Settle, -0.05, LiveRefusal::BuildRefuses, Refusal::PositionTheSettleTimeIsNegative);
    refused(c, 0, Tunable::GravityHold, 12.05, LiveRefusal::BuildRefuses, Refusal::PositionTheGravityHoldIsPast12Volts);
    refused(c, 0, Tunable::RangeMin, 0.25, LiveRefusal::BuildRefuses,
            Refusal::PositionTheHomePositionIsOutsideTheTravelLimits);
    refused(c, 0, Tunable::RangeMax, 5.75, LiveRefusal::BuildRefuses, Refusal::PositionAPresetIsOutsideTheTravelLimits);
    // a stall-homed lift: its push and its homing signature
    TuneRig m{tune_fixture::kMidHome};
    refused(m, 0, Tunable::HomeVolts, 0.0, LiveRefusal::BuildRefuses, Refusal::PositionHomingNeedsHomeVolts);
    refused(m, 0, Tunable::HomeVolts, -12.05, LiveRefusal::BuildRefuses, Refusal::PositionTheHomingPushIsPast12Volts);
    refused(m, 0, Tunable::HomeStallAmps, 0.0, LiveRefusal::BuildRefuses, Refusal::AStallSignatureNeedsAmpsAbove0);
    refused(m, 0, Tunable::HomeStallAmps, 1.5, LiveRefusal::BuildRefuses, Refusal::AStallSignatureMustBeBelowTheCurrentCap);
    refused(m, 0, Tunable::HomeStallRpm, 0.0, LiveRefusal::BuildRefuses, Refusal::AStallSignatureNeedsBelowMotorRpmAbove0);
    refused(m, 0, Tunable::HomeStallLasting, 0.0, LiveRefusal::BuildRefuses, Refusal::AStallSignatureNeedsLastingAbove0);
    // an arm homed onto its LOWER stop: a push turned upward is the BUILD's refusal (it points away)
    TuneRig s{tune_fixture::kSwing};
    refused(s, 0, Tunable::HomeVolts, 3.0, LiveRefusal::BuildRefuses, Refusal::PositionHomingPushesAwayFromItsHomeEnd);
}

TEST_CASE("TA3: each LIVE bound the build lacks (P9) is refused with its sentence, nothing changed") {
    TuneRig c{tune_fixture::kCascade};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    // every number finite (the build accepts a NaN kP — P9)
    CHECK(mech::specRefusal([] {
              mech::MechanismSpec x = tune_fixture::cascade.spec();
              x.position.kP = std::numeric_limits<double>::quiet_NaN();
              return x;
          }()) == Refusal::None);
    refused(c, 0, Tunable::KP, nan, LiveRefusal::NotANumber);
    refused(c, 0, Tunable::KD, inf, LiveRefusal::NotANumber);
    refused(c, 0, Tunable::KA, nan, LiveRefusal::NotANumber);
    // never a value the robot file cannot say: kP, kI, kD, kV above 0; settle never 0
    refused(c, 0, Tunable::KD, 0.0, LiveRefusal::GainNotAbove0);
    refused(c, 0, Tunable::KP, -0.1, LiveRefusal::GainNotAbove0);
    refused(c, 0, Tunable::KI, 0.0, LiveRefusal::GainNotAbove0);
    refused(c, 0, Tunable::KV, 0.0, LiveRefusal::GainNotAbove0);
    refused(c, 0, Tunable::Settle, 0.0, LiveRefusal::SettleIs0);
    // kS and kA not negative (the build accepts both — P9)
    refused(c, 0, Tunable::KS, -0.05, LiveRefusal::FeedforwardNegative);
    refused(c, 0, Tunable::KA, -1e-6, LiveRefusal::FeedforwardNegative);
    // kV at most twice its starting value — the EQUALITY edge lands exactly: accepted at 2x, refused one ulp past
    const double startKv = mech::positionStartingValues(tune_fixture::cascade.spec()).kV;
    const double twice = mech::kLiveKvMostTimesStarting * startKv;
    refused(c, 0, Tunable::KV, 1e9, LiveRefusal::KvPastTwiceStarting);
    refused(c, 0, Tunable::KV, std::nextafter(twice, 1e300), LiveRefusal::KvPastTwiceStarting);
    accepted(c, 0, Tunable::KV, twice);
    // the top speed at most the output's free speed (a Red motor, direct, on a 1.5 in spool: 7.85 in/s)
    const double freeIn = 100.0 * 6.0 / 1.0 * (3.14159265358979323846 * 1.5 / 360.0);
    CHECK(freeIn == doctest::Approx(7.853981633974483));
    refused(c, 0, Tunable::MaxSpeed, 500.0, LiveRefusal::TopSpeedPastFree);
    refused(c, 0, Tunable::MaxSpeed, std::nextafter(freeIn, 1e300), LiveRefusal::TopSpeedPastFree);
    accepted(c, 0, Tunable::MaxSpeed, freeIn);
    // a travel limit only NARROWED: never wider than the robot file's, at either end
    refused(c, 0, Tunable::RangeMin, -0.25, LiveRefusal::RangeWidened);
    refused(c, 0, Tunable::RangeMax, 24.25, LiveRefusal::RangeWidened);
    refused(c, 0, Tunable::RangeMax, std::nextafter(24.0, 1e300), LiveRefusal::RangeWidened);
    accepted(c, 0, Tunable::RangeMax, 23.75);
    accepted(c, 0, Tunable::RangeMax, 24.0);  // back to the file's exactly: allowed
    // a flywheel's gains and kS
    TuneRig fp{tune_fixture::kFlyAndPivot};
    refused(fp, 0, Tunable::KP, 0.0, LiveRefusal::GainNotAbove0);
    refused(fp, 0, Tunable::KI, -0.2, LiveRefusal::GainNotAbove0);
    refused(fp, 0, Tunable::KS, -0.05, LiveRefusal::FeedforwardNegative);
    refused(fp, 0, Tunable::KV, 1e9, LiveRefusal::KvPastTwiceStarting);
    // a homing push turned around, on a lift homed MID-travel (the build lets that push point either way)
    TuneRig m{tune_fixture::kMidHome};
    {
        mech::MechanismSpec x = tune_fixture::midHome.spec();
        x.position.homeVolts = 2.0;
        CHECK(mech::specRefusal(x) == Refusal::None);
    }
    refused(m, 0, Tunable::HomeVolts, 2.0, LiveRefusal::HomingPushTurned);
    accepted(m, 0, Tunable::HomeVolts, -2.5);  // the same direction, harder: fine
    // a stated cap is never removed by a change (0 A would be NO cap)
    refused(m, 0, Tunable::CurrentCap, 0.0, LiveRefusal::CapRemoved);
}

TEST_CASE("TA3: a number that is not live on this mechanism is refused by name, nothing changed") {
    TuneRig fp{tune_fixture::kFlyAndPivot};
    refused(fp, 0, Tunable::KA, 0.01, LiveRefusal::NotLive);        // TA8: a flywheel has no kA
    refused(fp, 0, Tunable::RangeMax, 10.0, LiveRefusal::NotLive);
    refused(fp, 1, Tunable::KP, 0.5, LiveRefusal::NotLive);         // a power mechanism has no gains
    refused(fp, 2, Tunable::StallAmps, 1.0, LiveRefusal::NotLive);  // ... nor a stall it never stated
    TuneRig c{tune_fixture::kCascade};
    refused(c, 0, Tunable::HomeVolts, -1.0, LiveRefusal::NotLive);  // it rests at its stop: no push
    refused(c, 0, Tunable::MaxVolts, 6.0, LiveRefusal::NotLive);
    refused(c, 0, Tunable::AtSpeedBand, 30.0, LiveRefusal::NotLive);
    TuneRig sw{tune_fixture::kSwitchLift};
    refused(sw, 0, Tunable::GravityHold, 0.5, LiveRefusal::NotLive);     // noGravity cannot say a hold
    refused(sw, 0, Tunable::HomeStallAmps, 0.5, LiveRefusal::NotLive);   // a switch home has no homing signature
}

TEST_CASE("TA3: a live bound judges only a number that DIFFERS from the robot file's -- a file the build accepted can be tuned") {
    // a robot file that states a top speed past the free speed (the build accepts it — P9)
    using namespace shulib::mechanism::words;
    static constexpr Lift fast{.name = "fast", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                               .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                               .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in),
                               .tuning = {.maxSpeed = 500}};
    static constexpr mech::MechanismTable table = mech::mechanisms(mech::wire(fast, mech::motors(1)));
    TuneRig r{table};
    accepted(r, 0, Tunable::KP, 0.3125);                                     // tunable at all
    refused(r, 0, Tunable::MaxSpeed, 501.0, LiveRefusal::TopSpeedPastFree);  // ... never FURTHER past free
    accepted(r, 0, Tunable::MaxSpeed, 499.0);                                // ... but back toward it (the review)
    accepted(r, 0, Tunable::MaxSpeed, 7.5);
}

TEST_CASE("TA3 (review): a number the robot file states PAST a live bound steps back TOWARD it -- every [−] accepted, a [+] refused, the edge at the file's own number") {
    using namespace shulib::mechanism::words;
    // a Red direct 1.5 in spool: free 7.854 in/s, starting kV 0.02 (twice: 0.04); the file says 10 in/s, kV 0.1, kS −0.5
    static constexpr Lift past{.name = "past", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                               .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                               .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in),
                               .tuning = {.kS = -0.5, .kV = 0.1, .maxSpeed = 10.0}};
    static constexpr mech::MechanismTable table = mech::mechanisms(mech::wire(past, mech::motors(1)));
    REQUIRE(mech::specRefusal(past.spec()) == Refusal::None);  // the build accepts all three (P9)
    {
        TuneRig r{table};
        refused(r, 0, Tunable::MaxSpeed, std::nextafter(10.0, 1e300), LiveRefusal::TopSpeedPastFree);
        refused(r, 0, Tunable::MaxSpeed, r.tuning.stepped(0, Tunable::MaxSpeed, +1), LiveRefusal::TopSpeedPastFree);
        accepted(r, 0, Tunable::MaxSpeed, std::nextafter(10.0, 0.0));  // one ulp toward the bound
        for (int i = 0; i < 6; ++i) {
            const mech::Applied a = r.tuning.step(0, Tunable::MaxSpeed, -1);
            INFO(a.sentence);
            CHECK(a.ok);
        }
        CHECK(r.tuning.value(0, Tunable::MaxSpeed) < 7.854);  // stepped all the way under the free speed
        accepted(r, 0, Tunable::MaxSpeed, 9.0);  // past free again, but no further than the file's own 10: the build's to judge
        refused(r, 0, Tunable::MaxSpeed, 10.5, LiveRefusal::TopSpeedPastFree);
    }
    {
        TuneRig r{table};
        refused(r, 0, Tunable::KV, r.tuning.stepped(0, Tunable::KV, +1), LiveRefusal::KvPastTwiceStarting);
        const mech::Applied a = r.tuning.step(0, Tunable::KV, -1);
        INFO(a.sentence);
        CHECK(a.ok);
        CHECK(r.tuning.value(0, Tunable::KV) == 0.08);
    }
    {
        TuneRig r{table};
        refused(r, 0, Tunable::KS, r.tuning.stepped(0, Tunable::KS, -1), LiveRefusal::FeedforwardNegative);
        refused(r, 0, Tunable::KS, std::nextafter(-0.5, -1.0), LiveRefusal::FeedforwardNegative);
        const mech::Applied a = r.tuning.step(0, Tunable::KS, +1);
        INFO(a.sentence);
        CHECK(a.ok);
        CHECK(r.tuning.value(0, Tunable::KS) == -0.45);
    }
}

TEST_CASE("tuningRefusal on whole specs: the build's FIRST, then the live bounds; the sentence is the build's own words") {
    const mech::MechanismSpec d = tune_fixture::cascade.spec();
    mech::MechanismSpec t = d;
    CHECK(mech::tuningRefusal(d, t) == LiveRefusal::None);
    CHECK(mech::tuningProblem(d, t) == nullptr);
    t.position.kD = 0.0;             // a live bound ...
    t.position.tolerance = 0.0;      // ... and a build refusal: the build's speaks first
    CHECK(mech::tuningRefusal(d, t) == LiveRefusal::BuildRefuses);
    CHECK(std::string{mech::tuningProblem(d, t)} == mech::refusalText(Refusal::PositionToleranceMustBe0));
    t.position.tolerance = d.position.tolerance;
    CHECK(mech::tuningRefusal(d, t) == LiveRefusal::GainNotAbove0);
    CHECK(std::string{mech::tuningProblem(d, t)} == mech::liveRefusalText(LiveRefusal::GainNotAbove0));
    const mech::MechanismSpec other = tune_fixture::flywheel.spec();
    CHECK(mech::tuningRefusal(d, other) == LiveRefusal::NotLive);  // not one mechanism's kind
}

TEST_CASE("parametersOf: each kind's live set, in the page's order -- a flywheel has no kA, a power mechanism no gains") {
    const auto ids = [](const mech::MechanismSpec& s) {
        std::string out;
        const mech::ParameterList l = mech::parametersOf(s);
        for (std::size_t i = 0; i < l.count; ++i) {
            out += std::string{l.items[i].name} + ",";
        }
        return out;
    };
    CHECK(ids(tune_fixture::cascade.spec()) ==
          "kP,kI,kD,kS,kV,kA,top speed,acceleration,within,settle,gravity hold,stall amps,stall rpm,stall lasting,"
          "current cap,lower limit,upper limit,");
    CHECK(ids(tune_fixture::midHome.spec()) ==
          "kP,kI,kD,kS,kV,kA,top speed,acceleration,within,settle,gravity hold,stall amps,stall rpm,stall lasting,"
          "homing push,homing amps,homing rpm,homing lasting,current cap,lower limit,upper limit,");
    CHECK(ids(tune_fixture::flywheel.spec()) ==
          "kP,kI,kS,kV,at-speed band,at-speed time,stall amps,stall rpm,stall lasting,current cap,");
    CHECK(ids(tune_fixture::pivot.spec()) == "max volts,stall amps,stall rpm,stall lasting,current cap,");
    // homed by SWITCH, with noGravity (the review): the homing push but no homing signature, and no gravity hold
    CHECK(ids(tune_fixture::switchLift.spec()) ==
          "kP,kI,kD,kS,kV,kA,top speed,acceleration,within,settle,stall amps,stall rpm,stall lasting,homing push,"
          "current cap,lower limit,upper limit,");
    CHECK(ids(tune_fixture::roller.spec()) == "max volts,current cap,");
    CHECK(ids(mech_fixture::robot_two::claw.spec()) == "");  // a pneumatic: nothing is live
    CHECK(std::string{mech::parameterOf(Tunable::MaxSpeed).member} == ".tuning.maxSpeed");
    CHECK(std::string{mech::tunableUnit(tune_fixture::cascade.spec(), Tunable::MaxSpeed)} == "in/s");
    CHECK(std::string{mech::tunableUnit(tune_fixture::swing.spec(), Tunable::MaxSpeed)} == "deg/s");
    CHECK(std::string{mech::tunableUnit(tune_fixture::flywheel.spec(), Tunable::KP)} == "V per rad/s");
}

TEST_CASE("a stall's rpm reads back as the robot file wrote it (5, not 5.000000000000001), and sets the spec's rad/s as kinds.hpp does") {
    TuneRig c{tune_fixture::kCascade};
    CHECK(c.tuning.value(0, Tunable::StallRpm) == 5.0);
    TuneRig fp{tune_fixture::kFlyAndPivot};
    CHECK(fp.tuning.value(0, Tunable::StallRpm) == 30.0);
    // a typed rpm with a long tail still maps to EXACTLY the spec's rad/s
    TuneRig l{tune_fixture::kLongNumbers};
    const double rpm = l.tuning.value(0, Tunable::StallRpm);
    CHECK(mech::detail::liveRadS(rpm) == l.tuning.spec(0).position.stall.speedRadS);
    accepted(c, 0, Tunable::StallRpm, 7.5);
    CHECK(c.tuning.spec(0).position.stall.speedRadS == 7.5 * 2.0 * 3.14159265358979323846 / 60.0);
}

TEST_CASE("the step rules: gains x/÷1.25 (FINE 1.05), volts and amps ±0.05, speeds ±5 %, reached, seconds, limits -- each pinned") {
    const mech::MechanismSpec lift = tune_fixture::cascade.spec();
    const mech::MechanismSpec arm = tune_fixture::swing.spec();
    CHECK(mech::steppedValue(lift, Tunable::KP, +1) == 0.3125);
    CHECK(mech::steppedValue(lift, Tunable::KP, -1) == 0.2);
    CHECK(mech::steppedValue(lift, Tunable::KP, +1, true) == 0.2625);
    CHECK(mech::steppedValue(lift, Tunable::KP, -1, true) == 0.238095);  // 0.25/1.05, six figures
    CHECK(mech::steppedValue(lift, Tunable::KP, 0) == 0.25);             // no direction: unchanged
    CHECK(mech::steppedValue(lift, Tunable::GravityHold, +1) == 1.05);
    CHECK(mech::steppedValue(lift, Tunable::GravityHold, -1) == 0.95);
    CHECK(mech::steppedValue(lift, Tunable::StallAmps, -1) == 1.95);
    CHECK(mech::steppedValue(lift, Tunable::StallLasting, +1) == 0.55);
    CHECK(mech::steppedValue(lift, Tunable::Within, +1) == 0.3);   // ±0.05 in on a lift
    CHECK(mech::steppedValue(arm, Tunable::Within, +1) == 2.0);    // ±0.5 deg on an arm
    CHECK(mech::steppedValue(lift, Tunable::RangeMax, -1) == 23.75);
    CHECK(mech::steppedValue(arm, Tunable::RangeMax, -1) == 119.0);
    CHECK(mech::steppedValue(lift, Tunable::StallRpm, +1) == 5.25);
    CHECK(mech::steppedValue(lift, Tunable::Settle, -1) == 0.05);
    // an additive step lands ON the grid: 0.1 - 0.05 - 0.05 is exactly 0, never 1.4e-17
    mech::MechanismSpec s = lift;
    s.position.kS = 0.1;
    s.position.kS = mech::steppedValue(s, Tunable::KS, -1);
    s.position.kS = mech::steppedValue(s, Tunable::KS, -1);
    CHECK(s.position.kS == 0.0);
    CHECK_FALSE(std::signbit(s.position.kS));
    // the cap: none reads as 2.5 A — [−] 2.45, [+] 2.55 (which the build then refuses); a step never removes one
    CHECK(mech::steppedValue(lift, Tunable::CurrentCap, -1) == 2.45);
    CHECK(mech::steppedValue(lift, Tunable::CurrentCap, +1) == 2.55);
    s = lift;
    s.currentCapAmps = 0.05;
    CHECK(mech::steppedValue(s, Tunable::CurrentCap, -1) == 0.05);
    s.currentCapAmps = 0.03;
    CHECK(mech::steppedValue(s, Tunable::CurrentCap, -1) == 0.03);
    // a factor's rounding to six figures at and above a million (its other branch): 1234567.8 -> 1234570
    CHECK(mech::detail::roundSix(1234567.8) == 1234570.0);
    CHECK(mech::detail::roundSix(-98765432.1) == -98765400.0);
    CHECK(mech::detail::roundSix(0.000123456789) == 0.000123457);
    // a number the mechanism does not offer: unchanged
    CHECK(mech::steppedValue(lift, Tunable::MaxVolts, +1) == mech::tunableValue(lift, Tunable::MaxVolts));
}

TEST_CASE("TA9: an Arm's kA at 0 -- [+] sets kV x 0.01 s, the next x1.25; [−] from that first step gives 0, which the file can say") {
    TuneRig r{tune_fixture::kSwing};
    REQUIRE(r.tuning.value(0, Tunable::KA) == 0.0);
    const double kV = r.tuning.value(0, Tunable::KV);
    const double first = mech::detail::roundSix(kV * 0.01);
    CHECK(first > 0.0);
    CHECK(r.tuning.stepped(0, Tunable::KA, -1) == 0.0);  // [−] at 0 stays 0
    REQUIRE(r.tuning.step(0, Tunable::KA, +1).ok);
    CHECK(r.tuning.value(0, Tunable::KA) == first);
    REQUIRE(r.tuning.step(0, Tunable::KA, +1).ok);
    CHECK(r.tuning.value(0, Tunable::KA) == mech::detail::roundSix(first * 1.25));
    REQUIRE(r.tuning.step(0, Tunable::KA, -1).ok);       // ÷1.25: back near the first step, not below it
    CHECK(r.tuning.value(0, Tunable::KA) >= first);
    REQUIRE(r.tuning.apply(0, Tunable::KA, first).ok);
    REQUIRE(r.tuning.step(0, Tunable::KA, -1).ok);       // [−] from the first step: 0
    CHECK(r.tuning.value(0, Tunable::KA) == 0.0);
    // FINE from 0 is the same first step (a factor of 0 is still 0)
    CHECK(r.tuning.stepped(0, Tunable::KA, +1, true) == first);
}

TEST_CASE("setting a number to what it is already is ok and changes nothing (no undo record, no re-derive)") {
    TuneRig r{tune_fixture::kCascade};
    const int lines = r.sink.lines.size();
    const mech::Applied a = r.tuning.apply(0, Tunable::KP, 0.25);
    CHECK(a.ok);
    CHECK(std::string{a.sentence} == "'cascade': kP unchanged (0.25)");
    CHECK(r.tuning.undoDepth() == 0);
    CHECK(r.sink.lines.size() == lines);
    CHECK_FALSE(r.tuning.changed(0));
}
