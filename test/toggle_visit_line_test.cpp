// The Toggle visit's ONE LOG LINE, byte for byte (chunk AF1S-A1, its review round of 2026-11-11 — guide 11's
// `TGL` section). Every line is the step's OWN output from a scene on the host test plant
// (test/toggle_visit_rig.hpp), so a format change, a word change or a number that drifts turns this file
// red. GUIDE 11 QUOTES these twelve lines word for word, each after a terminal `[t=...]` prefix (which
// test/term_sink_test.cpp pins) and its level tag: change a line here and in guide 11 together — the
// second case fails until the guide is re-quoted. Every number is a host-plant placeholder; nothing here
// has run on a robot.

#include "doctest.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "toggle_visit_rig.hpp"

using namespace tv_rig;
using sc::ToggleVisitVerdict;

namespace {

// Guide 11's twelve TGL lines, in its order.
constexpr const char* kGuideLines[] = {
    "right wall Toggle: YELLOW -> RED after 2 pushes, 9.5s (ours RED), wall not checked (relative start), seating "
    "not checked",
    "red wall Toggle: already RED (ours), nothing pushed, 0.2s, wall not checked (relative start), seating not checked",
    "right wall Toggle: not at the spot -- band at 17.2-30.8 in, look spot 16.0 +- 6.0 in -- nothing pushed",
    "right wall Toggle: UNKNOWN before (another colour directly above) -- nothing pushed, wall not checked (relative "
    "start)",
    "right wall Toggle: wrong wall -- heading 60 deg, its wall is at 0 deg -- nothing pushed",
    "right wall Toggle: not now -- 6.3s left, needs 13.1s -- nothing pushed",
    "right wall Toggle: NO CHANGE -- still YELLOW after 1 push (sticky, or a camera frozen since before), wall not "
    "checked (relative start)",
    "right wall Toggle: RED after a SHORT stroke (push STALLED) -- maybe stuck part-way, not counted",
    "right wall Toggle: FLIPPER STUCK -- 'flip arm' did not reach its back position (STALLED); the robot stays put",
    "right wall Toggle: retracted for time after 1 push -- 0.1s left, the robot is off it",
    "right wall Toggle: the drive in did not arrive (timed out) after 0 pushes",
    "right wall Toggle: camera frozen? seen on the drive in -- push 1 not made, YELLOW before, the robot drove back "
    "out",
};
constexpr std::size_t kLines = sizeof kGuideLines / sizeof kGuideLines[0];

Options faces(Options o, std::uint8_t start, std::uint8_t next, std::uint8_t third) {
    o.toggle.fieldId = start;
    o.toggle.bottomId = next;
    o.toggle.outerId = third;
    return o;
}

// The one TGL line a scene wrote, and its level.
std::string tgl(const Rig& r, shulib::hal::LogLevel& level) {
    const std::vector<std::string> l = r.lines("TGL");
    REQUIRE(l.size() == 1U);
    level = r.lastLevel("TGL");
    return l[0];
}

}  // namespace

// Bug caught: any verdict's words or numbers drifting from guide 11's lines (the AF1S-A1 review: the guide's TGL
// examples were written by hand and pinned by nothing; this case also caught the wrong-wall line printing the right
// wall's direction as "-0 deg").
TEST_CASE("AF1S-A1 guide 11's twelve TGL lines, byte for byte, each from a scene on the host test plant") {
    shulib::hal::LogLevel lv{};
    {
        Rig r{scene(Form::Arm)};
        REQUIRE(r.visit(armRequest(), Rig::entry()).verdict == ToggleVisitVerdict::Flipped);
        CHECK(tgl(r, lv) == std::string{kGuideLines[0]});
        CHECK(lv == shulib::hal::LogLevel::Info);
    }
    {
        Rig r{faces(scene(Form::Arm, ov::toggles::kRedWall), kRed, kYellow, kBlue)};
        REQUIRE(r.visit(armRequest(ov::toggles::kRedWall), Rig::entry()).verdict == ToggleVisitVerdict::AlreadyOurs);
        CHECK(tgl(r, lv) == std::string{kGuideLines[1]});
        CHECK(lv == shulib::hal::LogLevel::Info);
    }
    {
        Options o = scene(Form::Arm);
        o.lookFrom = 24.0;
        Rig r{o};
        REQUIRE(r.visit(armRequest(), Rig::entry()).verdict == ToggleVisitVerdict::NotAtSpot);
        CHECK(tgl(r, lv) == std::string{kGuideLines[2]});
        CHECK(lv == shulib::hal::LogLevel::Warn);
    }
    {
        Options o = scene(Form::Arm);
        o.headingDeg = 8.0;
        Rig r{o};
        REQUIRE(r.visit(armRequest(), Rig::entry()).verdict == ToggleVisitVerdict::UnknownBefore);
        CHECK(tgl(r, lv) == std::string{kGuideLines[3]});
    }
    {
        Options o = scene(Form::Arm);
        o.headingDeg = 60.0;
        Rig r{o};
        REQUIRE(r.visit(armRequest(), Rig::entry(sc::AutonKind::Skills, sc::StartPose::field(r.truth()))).verdict
                == ToggleVisitVerdict::WrongWall);
        CHECK(tgl(r, lv) == std::string{kGuideLines[4]});
    }
    {
        Rig r{scene(Form::Arm)};
        REQUIRE(r.visit(armRequest(), Rig::entry(), 6.5).verdict == ToggleVisitVerdict::NotEnoughTime);
        CHECK(tgl(r, lv) == std::string{kGuideLines[5]});
    }
    {
        Options o = scene(Form::Arm);
        o.toggle.mode = sim::ToggleMode::Sticky;
        Rig r{o};
        REQUIRE(r.visit(armRequest(), Rig::entry()).verdict == ToggleVisitVerdict::NoChange);
        CHECK(tgl(r, lv) == std::string{kGuideLines[6]});
    }
    {
        Rig r{faces(scene(Form::Arm), kYellow, kRed, kBlue)};
        r.armPlant.setObstacleDeg(70.0);
        REQUIRE(r.visit(armRequest(), Rig::entry()).verdict == ToggleVisitVerdict::OursAfterShortStroke);
        CHECK(tgl(r, lv) == std::string{kGuideLines[7]});
    }
    {
        Rig r{scene(Form::Arm)};
        bool wedged = false;
        r.world.onPace = [&] {
            if (!wedged && r.armPlant.trueDeg() > 85.0) {
                wedged = true;
                r.armPlant.setWedgeDeg(50.0);
            }
        };
        REQUIRE(r.visit(armRequest(), Rig::entry()).verdict == ToggleVisitVerdict::FlipperStuck);
        CHECK(tgl(r, lv) == std::string{kGuideLines[8]});
    }
    {
        Options o = scene(Form::Arm);
        o.toggle.touchIn = 15.0;
        Rig r{o};
        sc::ToggleVisitRequest q = armRequest();
        q.settle = Time{0.6};
        q.margin = Time{0.1};
        bool jumped = false;
        r.world.onPace = [&] {
            if (!jumped && r.guard.running() && r.armPlant.trueDeg() >= 35.0) {
                jumped = true;
                r.rig.h.clock().advance(Time{r.guard.remaining().value() - 2.0});
            }
        };
        REQUIRE(r.visit(q, Rig::entry()).verdict == ToggleVisitVerdict::RetractedForTime);
        CHECK(tgl(r, lv) == std::string{kGuideLines[9]});
    }
    {
        Rig r{scene(Form::Arm)};
        const int m0 = r.chassis.scheduler().motionsStarted();
        r.world.onPace = [&] {
            if (r.chassis.scheduler().hasActiveMotion() && r.chassis.scheduler().motionsStarted() - m0 == 1) {
                r.stopDrive();
            }
        };
        REQUIRE(r.visit(armRequest(), Rig::entry()).verdict == ToggleVisitVerdict::DriveFailed);
        CHECK(tgl(r, lv) == std::string{kGuideLines[10]});
    }
    {
        Rig r{scene(Form::Arm)};
        bool frozen = false;
        r.world.onPace = [&] {
            if (!frozen && r.chassis.scheduler().hasActiveMotion()) {
                frozen = true;
                r.camPlant.setState(sim::CameraState::Frozen);
            }
        };
        REQUIRE(r.visit(armRequest(), Rig::entry()).verdict == ToggleVisitVerdict::FrozenBeforePush);
        CHECK(tgl(r, lv) == std::string{kGuideLines[11]});
        CHECK(lv == shulib::hal::LogLevel::Warn);
    }
}

// Bug caught (the AF1S-A1 review): guide 11's TGL examples drifting from the code with nothing turning red. The
// chapter's `TGL` section must quote EXACTLY the twelve lines above, in order, each after a `[t=...]` prefix and
// its level tag (none for Info, `[WARN]` otherwise).
TEST_CASE("AF1S-A1 guide 11 quotes exactly the twelve pinned TGL lines") {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/docs/guide/11-reading-the-diagnostics.md"};
    REQUIRE(in.good());
    std::stringstream buf;
    buf << in.rdbuf();
    const std::string text = buf.str();
    const std::size_t from = text.find("## The Toggle visit (TGL)");
    REQUIRE(from != std::string::npos);
    const std::size_t to = text.find("\n## ", from + 1);
    std::vector<std::string> quoted;
    std::istringstream lines{text.substr(from, to == std::string::npos ? std::string::npos : to - from)};
    for (std::string l; std::getline(lines, l);) {
        const std::size_t tag = l.find("[TGL] ");
        if (l.rfind("[t=", 0) == 0 && tag != std::string::npos) {
            quoted.push_back(l.substr(tag + 6));
        }
    }
    REQUIRE(quoted.size() == kLines);
    for (std::size_t i = 0; i < kLines; ++i) {
        INFO("guide line " << i);
        CHECK(quoted[i] == std::string{kGuideLines[i]});
    }
}
