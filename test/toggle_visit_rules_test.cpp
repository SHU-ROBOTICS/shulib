// AF1S-A1 — the Toggle step's PURE rules (competition/toggle_visit.hpp, vision/override_elements.hpp's Toggle
// table): which Toggle is which Quadrant's (HA-243), the request's refusals (each one, from a valid request
// with ONE field broken, the equality edges landed on exactly), the time formula on the tick grid (T13), the
// look spot's CONTAINMENT rule (T5's 0.01-in edges), the direction toward each wall (D6's sign, R6), the
// verdict -> outcome map (T11) and the words. The end-to-end behaviour is toggle_visit_scenes_test.cpp's.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <set>
#include <string>

#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/competition/toggle_visit.hpp"
#include "toggle_visit_rig.hpp"

namespace ov = shulib::vision::override2026;
namespace sc = shulib::competition;
using shulib::manipulation::MechanismOutcome;
using shulib::units::Length;
using shulib::units::Time;
using shulib::vision::ToggleColour;
using sc::ToggleVisitProblem;
using sc::ToggleVisitVerdict;

namespace {
constexpr double kPi = 3.14159265358979323846;

// A valid request, checked AT BUILD (the consteval form compiles only for a request with no problem).
constexpr sc::ToggleVisitRequest kValid = sc::toggleVisit({
    .toggle = ov::toggles::kRedWall,
    .flipper = tv_fixture::flipArm["flip"],
    .back = tv_fixture::flipArm["stow"],
    .lookFrom = Length{16.0},
    .offBy = Length{6.0},
    .pushFrom = Length{4.0},
    .lookWithin = Time{0.5},
    .flipBound = Time{0.8},
    .backBound = Time{0.8},
    .settle = Time{0.2},
    .driveBound = Time{0.6},
    .margin = Time{0.3},
    .maxImageAge = Time{0.1},
});
static_assert(sc::toggleVisitProblemOf(kValid) == ToggleVisitProblem::None);
static_assert(kValid.maxPushes == 2);
}  // namespace

TEST_CASE("AF1S-A1 Quadrant table (HA-243): right RED, left BLUE, blue wall BLUE, red wall RED — by id, in "
          "kTogglePlaces' order, and mapped to the camera's colours") {
    CHECK(ov::toggleIndex(ov::ToggleId::Unset) == -1);
    CHECK(ov::toggleIndex(ov::toggles::kRightWall) == 0);
    CHECK(ov::toggleIndex(ov::toggles::kLeftWall) == 1);
    CHECK(ov::toggleIndex(ov::toggles::kBlueWall) == 2);
    CHECK(ov::toggleIndex(ov::toggles::kRedWall) == 3);
    CHECK(ov::toggleIndex(static_cast<ov::ToggleId>(5)) == -1);
    // Each id names the wall its table entry is on (the place, not only the index).
    CHECK(ov::kTogglePlaces[ov::toggleIndex(ov::toggles::kRightWall)].x == doctest::Approx(70.2));
    CHECK(ov::kTogglePlaces[ov::toggleIndex(ov::toggles::kLeftWall)].x == doctest::Approx(-70.2));
    CHECK(ov::kTogglePlaces[ov::toggleIndex(ov::toggles::kBlueWall)].y == doctest::Approx(70.2));
    CHECK(ov::kTogglePlaces[ov::toggleIndex(ov::toggles::kRedWall)].y == doctest::Approx(-70.2));
    CHECK(ov::quadrantColour(ov::toggles::kRightWall) == ov::QuadrantColour::Red);
    CHECK(ov::quadrantColour(ov::toggles::kLeftWall) == ov::QuadrantColour::Blue);
    CHECK(ov::quadrantColour(ov::toggles::kBlueWall) == ov::QuadrantColour::Blue);
    CHECK(ov::quadrantColour(ov::toggles::kRedWall) == ov::QuadrantColour::Red);
    CHECK(ov::quadrantColour(ov::ToggleId::Unset) == ov::QuadrantColour::Unset);
    // Both entries of the map, and "ours" per Toggle.
    CHECK(sc::toggleColourOf(ov::QuadrantColour::Red) == ToggleColour::Red);
    CHECK(sc::toggleColourOf(ov::QuadrantColour::Blue) == ToggleColour::Blue);
    CHECK(sc::toggleColourOf(ov::QuadrantColour::Unset) == ToggleColour::Unknown);
    CHECK(sc::oursFor(ov::toggles::kRightWall) == ToggleColour::Red);
    CHECK(sc::oursFor(ov::toggles::kLeftWall) == ToggleColour::Blue);
    CHECK(sc::oursFor(ov::toggles::kBlueWall) == ToggleColour::Blue);
    CHECK(sc::oursFor(ov::toggles::kRedWall) == ToggleColour::Red);
    CHECK(std::string{ov::toggleName(ov::toggles::kRedWall)} == "red wall Toggle");
    CHECK(std::string{ov::toggleName(ov::toggles::kRightWall)} == "right wall Toggle");
    CHECK(std::string{ov::toggleName(ov::ToggleId::Unset)} == "unnamed Toggle");
}

TEST_CASE("AF1S-A1 the direction TOWARD each wall (R6: minus the inward normal) — right 0, blue +90, left 180, "
          "red -90") {
    CHECK(sc::towardWallRadians(ov::toggles::kRightWall) == doctest::Approx(0.0));
    CHECK(sc::towardWallRadians(ov::toggles::kBlueWall) == doctest::Approx(kPi / 2.0));
    CHECK(std::fabs(sc::towardWallRadians(ov::toggles::kLeftWall)) == doctest::Approx(kPi));
    CHECK(sc::towardWallRadians(ov::toggles::kRedWall) == doctest::Approx(-kPi / 2.0));
}

TEST_CASE("AF1S-A1 the request: every refusal from a valid request with ONE field broken — the equality edges "
          "landed on exactly") {
    auto problem = [](auto edit) {
        sc::ToggleVisitRequest r = kValid;
        edit(r);
        return sc::toggleVisitProblemOf(r);
    };
    using P = ToggleVisitProblem;
    CHECK(problem([](auto& r) { r.toggle = ov::ToggleId::Unset; }) == P::ToggleUnset);
    CHECK(problem([](auto& r) { r.toggle = static_cast<ov::ToggleId>(9); }) == P::ToggleUnset);
    CHECK(problem([](auto& r) { r.settle = Time{std::nan("")}; }) == P::NotFinite);
    CHECK(problem([](auto& r) { r.offBy = Length{HUGE_VAL}; }) == P::NotFinite);
    CHECK(problem([](auto& r) { r.lookFrom = Length{0.0}; }) == P::LookFromUnset);
    CHECK(problem([](auto& r) { r.offBy = Length{0.0}; }) == P::OffByUnset);
    CHECK(problem([](auto& r) { r.pushFrom = Length{0.0}; }) == P::PushFromUnset);
    // pushFrom == lookFrom (EXACTLY) is refused; a hair nearer is not this refusal.
    CHECK(problem([](auto& r) { r.pushFrom = Length{16.0}; }) == P::PushNotNearer);
    CHECK(problem([](auto& r) { r.pushFrom = Length{17.0}; }) == P::PushNotNearer);
    CHECK(problem([](auto& r) { r.pushFrom = Length{15.999}; }) == P::None);
    // offBy == lookFrom (EXACTLY) is refused; a hair less is not.
    CHECK(problem([](auto& r) { r.offBy = Length{16.0}; }) == P::OffByTooLarge);
    CHECK(problem([](auto& r) { r.offBy = Length{15.999}; }) == P::None);
    CHECK(problem([](auto& r) { r.lookWithin = Time{0.0}; }) == P::LookWithinUnset);
    CHECK(problem([](auto& r) { r.flipBound = Time{0.0}; }) == P::FlipBoundUnset);
    CHECK(problem([](auto& r) { r.backBound = Time{0.0}; }) == P::BackBoundUnset);
    CHECK(problem([](auto& r) { r.settle = Time{0.0}; }) == P::SettleUnset);
    CHECK(problem([](auto& r) { r.driveBound = Time{0.0}; }) == P::DriveBoundUnset);
    CHECK(problem([](auto& r) { r.margin = Time{0.0}; }) == P::MarginUnset);
    CHECK(problem([](auto& r) { r.maxImageAge = Time{0.0}; }) == P::MaxImageAgeUnset);
    CHECK(problem([](auto& r) { r.maxPushes = 0; }) == P::MaxPushes);
    CHECK(problem([](auto& r) { r.maxPushes = 3; }) == P::MaxPushes);
    CHECK(problem([](auto& r) { r.maxPushes = 1; }) == P::None);
    // The flipper pair: two kinds, two mechanisms, the same goal.
    CHECK(problem([](auto& r) { r.back = sc::FlipperGoal{}; }) == P::FlipperKindsDiffer);
    CHECK(problem([](auto& r) { r.back = tv_fixture::flipPiston["stow"]; }) == P::FlipperKindsDiffer);
    CHECK(problem([](auto& r) { r.back = tv_fixture::flipLift["stow"]; }) == P::TwoMechanisms);
    CHECK(problem([](auto& r) { r.back = tv_fixture::flipArm["flip"]; }) == P::SameGoalTwice);
    CHECK(problem([](auto& r) { r.back = tv_fixture::flipArm.to(90); }) == P::SameGoalTwice);  // same value
    CHECK(problem([](auto& r) {
              r.flipper = tv_fixture::flipPiston["flip"];
              r.back = tv_fixture::flipPiston["flip"];
          }) == P::SameGoalTwice);
    CHECK(problem([](auto& r) {
              r.flipper = tv_fixture::flipPiston["flip"];
              r.back = tv_fixture::flipPiston["stow"];
          }) == P::None);
    // A PASSIVE flipper may leave flipBound and backBound at 0 — never negative.
    CHECK(problem([](auto& r) {
              r.flipper = sc::FlipperGoal{};
              r.back = sc::FlipperGoal{};
              r.flipBound = Time{0.0};
              r.backBound = Time{0.0};
          }) == P::None);
    CHECK(problem([](auto& r) {
              r.flipper = sc::FlipperGoal{};
              r.back = sc::FlipperGoal{};
              r.flipBound = Time{-0.1};
          }) == P::FlipBoundUnset);
    // Every problem has a sentence; None has none.
    CHECK(sc::toggleVisitProblemSentence(P::None) == nullptr);
    for (int p = 1; p <= static_cast<int>(P::SameGoalTwice); ++p) {
        CAPTURE(p);
        const char* s = sc::toggleVisitProblemSentence(static_cast<P>(p));
        REQUIRE(s != nullptr);
        CHECK(std::strlen(s) > 10);
    }
    CHECK(sc::toggleVisitRequestProblem(kValid) == nullptr);
}

TEST_CASE("AF1S-A1 FlipperGoal converts IMPLICITLY from a goal (R9); default-constructed it is PASSIVE") {
    const sc::FlipperGoal a = tv_fixture::flipArm["flip"];
    CHECK(a.kind == sc::FlipperGoal::Kind::Position);
    CHECK(std::string{a.mechanismName()} == "flip arm");
    CHECK(a.position.value == doctest::Approx(90.0));
    const sc::FlipperGoal p = tv_fixture::flipPiston["flip"];
    CHECK(p.kind == sc::FlipperGoal::Kind::Pneumatic);
    CHECK(std::string{p.mechanismName()} == "flip piston");
    const sc::FlipperGoal none{};
    CHECK(none.passive());
    CHECK(std::string{none.mechanismName()}.empty());
}

TEST_CASE("AF1S-A1 T13 worstCaseSeconds == need(maxPushes): every bound rounded UP to the 10-ms tick, two ticks a "
          "push, the drive in AND out, and the drive in's frozen HOLD (maxImageAge) — on the grid and off it") {
    sc::ToggleVisitRequest r = kValid;
    // On the grid: 2 x (0.6 + 0.1 + 0.8 + 0.8 + 0.2 + 0.6 + 0.5 + 0.02) + 0.3 = 7.54 (0.1: maxImageAge, the hold).
    CHECK(sc::worstCaseSeconds(r) == doctest::Approx(7.54).epsilon(1e-9));
    CHECK(sc::toggleVisitNeedSeconds(r, 1) == doctest::Approx(3.92).epsilon(1e-9));
    // Off the grid: each rounded UP — 0.601 -> 0.61 (twice), 0.333 -> 0.34, 0.8001 -> 0.81, 0.101 -> 0.11,
    // 0.2 stays.
    r.driveBound = Time{0.601};
    r.flipBound = Time{0.333};
    r.backBound = Time{0.8001};
    r.lookWithin = Time{0.5};
    r.maxImageAge = Time{0.101};
    r.maxPushes = 1;
    const double one = 0.61 + 0.11 + 0.34 + 0.81 + 0.2 + 0.61 + 0.5 + 0.02;
    CHECK(sc::worstCaseSeconds(r) == doctest::Approx(one + 0.3).epsilon(1e-9));
    CHECK(sc::toggleVisitCeilTick(0.8) == doctest::Approx(0.8).epsilon(1e-12));  // 0.8/0.01 is 80.000...01
    CHECK(sc::toggleVisitCeilTick(0.801) == doctest::Approx(0.81).epsilon(1e-12));
    CHECK(sc::toggleVisitCeilTick(0.0) == doctest::Approx(0.0));
}

TEST_CASE("AF1S-A1 T5 the look spot is CONTAINMENT (R2): the whole range interval inside [lookFrom - offBy, "
          "lookFrom + offBy] — its ends ON the window accepted, 0.01 in outside refused, an overlap refused") {
    const double win = 25.0;  // degrees
    shulib::vision::ToggleView v{};
    v.colour = ToggleColour::Yellow;
    v.bearing = shulib::vision::Interval{true, -0.05, 0.05};
    auto at = [&](double lo, double hi) {
        v.range = shulib::vision::Interval{true, lo, hi};
        return sc::bandAtLookSpotProblem(v, kValid, win) == nullptr;
    };
    CHECK(at(10.0, 22.0));         // exactly the window (16 +- 6)
    CHECK(at(12.0, 20.0));
    CHECK_FALSE(at(9.99, 20.0));   // 0.01 in short
    CHECK_FALSE(at(12.0, 22.01));  // 0.01 in long
    CHECK_FALSE(at(16.7, 29.8));   // P7's robot 8 in too far: it OVERLAPS the window — refused
    CHECK_FALSE(at(4.0, 30.0));    // wider than the window, around it
    v.range = shulib::vision::Interval{true, 12.0, HUGE_VAL};
    CHECK(sc::bandAtLookSpotProblem(v, kValid, win) != nullptr);  // an unbounded far end
    v.range = shulib::vision::Interval{false, 12.0, 20.0};
    CHECK(sc::bandAtLookSpotProblem(v, kValid, win) != nullptr);  // nothing measured
    // The bearing interval must be wholly inside the window too.
    v.range = shulib::vision::Interval{true, 12.0, 20.0};
    const double w = win * shulib::vision::kDegToRad;
    v.bearing = shulib::vision::Interval{true, -w, w};
    CHECK(sc::bandAtLookSpotProblem(v, kValid, win) == nullptr);
    v.bearing = shulib::vision::Interval{true, -w, w + 1e-6};
    CHECK(sc::bandAtLookSpotProblem(v, kValid, win) != nullptr);
    v.bearing = shulib::vision::Interval{false, 0.0, 0.0};
    CHECK(sc::bandAtLookSpotProblem(v, kValid, win) != nullptr);
}

TEST_CASE("AF1S-A1 T11 outcome(): SUCCEEDED for the two successes, CANCELLED for CUT, STALLED for a stuck flipper "
          "or a short visit after a STALLED push, TIMED_OUT for the time verdicts, UNCONFIRMED otherwise") {
    sc::ToggleVisitReport r{};
    auto out = [&](ToggleVisitVerdict v) {
        r.verdict = v;
        return r.outcome();
    };
    CHECK(out(ToggleVisitVerdict::Unset) == MechanismOutcome::Unconfirmed);
    CHECK_FALSE(r.ok());
    CHECK(out(ToggleVisitVerdict::AlreadyOurs) == MechanismOutcome::Succeeded);
    CHECK(r.ok());
    CHECK(out(ToggleVisitVerdict::Flipped) == MechanismOutcome::Succeeded);
    CHECK(r.ok());
    CHECK(out(ToggleVisitVerdict::Cut) == MechanismOutcome::Cancelled);
    CHECK(out(ToggleVisitVerdict::FlipperStuck) == MechanismOutcome::Stalled);
    CHECK(out(ToggleVisitVerdict::NotEnoughTime) == MechanismOutcome::TimedOut);
    CHECK(out(ToggleVisitVerdict::NoTimeForSecondPush) == MechanismOutcome::TimedOut);
    CHECK(out(ToggleVisitVerdict::RetractedForTime) == MechanismOutcome::TimedOut);
    for (const ToggleVisitVerdict v :
         {ToggleVisitVerdict::NotSkills, ToggleVisitVerdict::NoCamera, ToggleVisitVerdict::FlipperNotReady,
          ToggleVisitVerdict::WrongWall, ToggleVisitVerdict::UnknownBefore, ToggleVisitVerdict::NotAtSpot,
          ToggleVisitVerdict::NoChange, ToggleVisitVerdict::StillNotOurs, ToggleVisitVerdict::UnknownAfter,
          ToggleVisitVerdict::OursAfterShortStroke, ToggleVisitVerdict::DriveFailed, ToggleVisitVerdict::SeenNotOurs,
          ToggleVisitVerdict::FrozenBeforePush, ToggleVisitVerdict::SpotsTooClose /* AF1S-A2 */}) {
        CAPTURE(static_cast<int>(v));
        CHECK(out(v) == MechanismOutcome::Unconfirmed);
        CHECK_FALSE(r.ok());
    }
    // A short visit after a STALLED push is STALLED; OURS AFTER A SHORT STROKE stays UNCONFIRMED.
    r.pushes = 1;
    r.push[0].outcome = MechanismOutcome::Stalled;
    CHECK(out(ToggleVisitVerdict::NoChange) == MechanismOutcome::Stalled);
    CHECK(out(ToggleVisitVerdict::StillNotOurs) == MechanismOutcome::Stalled);
    CHECK(out(ToggleVisitVerdict::UnknownAfter) == MechanismOutcome::Stalled);
    CHECK(out(ToggleVisitVerdict::OursAfterShortStroke) == MechanismOutcome::Unconfirmed);
    r.push[0].outcome = MechanismOutcome::TimedOut;
    CHECK(out(ToggleVisitVerdict::NoChange) == MechanismOutcome::Unconfirmed);
}

TEST_CASE("AF1S-A1 the verdict words: every verdict has its own, ASCII, none empty") {
    // The LAST verdict (AF1S-A2 appended SpotsTooClose; a verdict appended later moves this bound with it).
    constexpr ToggleVisitVerdict kLast = ToggleVisitVerdict::SpotsTooClose;
    std::set<std::string> seen;
    for (int v = 0; v <= static_cast<int>(kLast); ++v) {
        const std::string w = sc::toggleVisitWord(static_cast<ToggleVisitVerdict>(v));
        CAPTURE(v);
        CHECK_FALSE(w.empty());
        CHECK(w != "UNKNOWN");
        for (const char c : w) {
            CHECK(static_cast<unsigned char>(c) < 0x80);
        }
        seen.insert(w);
    }
    CHECK(seen.size() == static_cast<std::size_t>(kLast) + 1);
    CHECK(std::string{sc::toggleVisitWord(kLast)} == "SPOTS TOO CLOSE");
    CHECK(std::string{sc::toggleVisitWord(static_cast<ToggleVisitVerdict>(static_cast<int>(kLast) + 1))} == "UNKNOWN");
}

// ═══ MIRROR1-A T7: the Toggle step in a MIRRORED auton (the MIRROR1 brief's §2.2, D4) ═══════════════════

// Bug caught: MM6 (the Toggle images crossed, right <-> left — robot two would look for robot one's Toggle
// on the wrong wall and be refused WRONG WALL, or flip the wrong one); a mirrored step that keeps robot
// one's Toggle (WRONG WALL from robot two's spot); a swap that changes "ours". The SAME request — robot
// one's red wall Toggle — on robot one parked at the red wall (entry as written) and on robot two parked
// at the right wall (entry MIRRORED): both pass rule 1's wall check with a FIELD start, both flip, "ours"
// is RED both times, and robot two's report and [TGL] line name the RIGHT wall's Toggle. The same request
// on robot two's spot WITHOUT the mirror is WRONG WALL. Through mirroredToggle, the four images keep
// their Quadrant colours (the static_asserts in override_mirror.hpp check toggleImage at build; this
// checks the function the step calls). AND (a review finding, 2026-11-13) robot two's visit once more
// with the Chassis's mirrored run OPEN around it — the configuration runAuton gives a mirrored auton's
// step: the step drives and reads BELOW the front door (the scheduler, the localizer), so its verdict,
// its strokes and its [TGL] line are byte-identical; a step that drove or read through a Chassis verb
// (chassis.pose(), chassis.moveTo) would be reflected a SECOND time and turn this red.
TEST_CASE("MIRROR1 T7 robot one's red wall Toggle, mirrored, is robot two's right wall Toggle: same colour, flipped") {
    using namespace tv_rig;
    namespace om = sc::override2026;
    const sc::ToggleVisitRequest rq = armRequest(ov::toggles::kRedWall);
    const sc::AutonEntry asWritten = Rig::entry(sc::AutonKind::Skills, sc::StartPose::field(parkedAt(ov::toggles::kRedWall, 16.0)));
    const sc::AutonEntry mirroredEntry = sc::mirrored(asWritten, om::kSkillsMirror);
    {
        Rig one{scene(Form::Arm, ov::toggles::kRedWall)};
        const sc::ToggleVisitReport rep = one.visit(rq, asWritten);
        CHECK(rep.toggle == ov::toggles::kRedWall);
        CHECK(rep.ours == ToggleColour::Red);
        CHECK(rep.wallChecked);
        CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
    }
    {
        Rig two{scene(Form::Arm, ov::toggles::kRightWall)};
        const sc::ToggleVisitReport rep = two.visit(rq, mirroredEntry);
        CHECK(rep.toggle == ov::toggles::kRightWall);
        CHECK(rep.ours == ToggleColour::Red);
        CHECK(rep.wallChecked);
        CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        REQUIRE(two.lines("TGL").size() == 1U);
        CHECK(two.lines("TGL")[0].rfind("right wall Toggle: ", 0) == 0);
        // the same visit with the mirrored run open (runAuton's configuration)
        Rig scoped{scene(Form::Arm, ov::toggles::kRightWall)};
        sc::ToggleVisitReport inRun{};
        {
            const shulib::chassis::MirroredRun run{scoped.chassis, om::kSkillsMirror.reflection};
            inRun = scoped.visit(rq, mirroredEntry);
        }
        CHECK(inRun.toggle == ov::toggles::kRightWall);
        CHECK(inRun.ours == ToggleColour::Red);
        CHECK(inRun.wallChecked);
        CHECK(inRun.verdict == ToggleVisitVerdict::Flipped);
        CHECK(scoped.toggle.strokes() == two.toggle.strokes());
        REQUIRE(scoped.lines("TGL").size() == 1U);
        CHECK(scoped.lines("TGL")[0] == two.lines("TGL")[0]);
    }
    {
        Rig two{scene(Form::Arm, ov::toggles::kRightWall)};
        const sc::ToggleVisitReport rep = two.visit(rq, asWritten);  // robot one's Toggle, unmirrored
        CHECK(rep.verdict == ToggleVisitVerdict::WrongWall);
        CHECK(two.toggle.strokes() == 0);
    }
    for (const ov::ToggleId id : {ov::toggles::kRightWall, ov::toggles::kLeftWall, ov::toggles::kBlueWall,
                                  ov::toggles::kRedWall}) {
        CAPTURE(static_cast<int>(id));
        const ov::ToggleId img = om::mirroredToggle(mirroredEntry, id);
        CHECK(img != id);
        CHECK(ov::quadrantColour(img) == ov::quadrantColour(id));
        CHECK(om::mirroredToggle(mirroredEntry, img) == id);  // the image's image
        CHECK(om::mirroredToggle(asWritten, id) == id);       // as written: unchanged
    }
    CHECK(om::mirroredToggle(mirroredEntry, ov::toggles::kRedWall) == ov::toggles::kRightWall);
    CHECK(om::mirroredToggle(mirroredEntry, ov::toggles::kLeftWall) == ov::toggles::kBlueWall);
}
