// route_preview_test.cpp — THE ROUTE PREVIEW (chunk RPV1-A — A0 and B1 together, 2026-11-12; the RPV1
// brief's §2.2). ADVERSARIAL: each case tries to break ONE rule of test/route_preview_run.hpp and names
// it. Every case runs the preview's own code (runPreview, previewEntry, foldRun) — the program's main
// only forwards argv. Say "host test plant" (the project's one name for it): nothing here has run on a robot.
//
// Most cases run FIXTURE autons (test/route_preview_fixtures.hpp) through fixture "robots": robot two
// (the rig's default fakes, robot two's real list) with fixture autons registered instead of the team's,
// and robot x (test/mechanism_fixtures.hpp, the fixture robot with a lift — RPV1-B1) on FakesByKind. The
// team's own autons are run by name too (RP3, RP10, RP35), robot one's on its own list (RP25).

#include "doctest.h"

#include <sys/resource.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "auton_tester_rig.hpp"
#include "fakes_by_kind.hpp"
#include "mechanism_fixtures.hpp"
#include "route_preview_fixtures.hpp"
#include "route_preview_hash.hpp"
#include "route_preview_model.hpp"
#include "route_preview_render.hpp"
#include "route_preview_run.hpp"
#include "source_pins.hpp"
#include "team/autons.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sim/degradation.hpp"

namespace {

namespace rp = route_preview;
namespace comp = shulib::competition;
using rp_fixture::entry;
using rp_fixture::field;

// ── fixture robots ─────────────────────────────────────────────────────────────────────────────────

void registerTwoFixtures(comp::AutonRegistry& reg) {
    using comp::AutonKind;
    const comp::StartPose rel = comp::StartPose::relative();
    reg.add(entry("seven", AutonKind::Match, &rp_fixture::sevenSteps, field(-48, -48, 0), 27, 29));
    reg.add(entry("times out", AutonKind::Match, &rp_fixture::stepThreeTimesOut, field(-48, -48, 0), 27, 29));
    reg.add(entry("forever", AutonKind::Skills, &rp_fixture::loopsForever, field(-48, -48, 0), 57, 59));
    reg.add(entry("background", AutonKind::Match, &rp_fixture::backgroundPivot, rel, 27, 29));
    reg.add(entry("thrower", AutonKind::Match, &rp_fixture::throwsAfterOne, rel, 27, 29));
    reg.add(entry("far", AutonKind::Match, &rp_fixture::twoHundredInches, rel, 27, 29));
    reg.add(entry("wall", AutonKind::Match, &rp_fixture::wallResetRefused, field(-48, -48, 0), 27, 29));
    reg.add(entry("helper", AutonKind::Match, &rp_fixture::mainWithHelper, field(-48, -48, 0), 27, 29));
    reg.add(entry("away", AutonKind::Match, &rp_fixture::awayFromRedWall, field(-40, -62.7, 90.1), 27, 29));
    reg.add(entry("loader", AutonKind::Match, &rp_fixture::backToLoader<6>, field(-58.77, -48, 90), 27, 29));
    reg.add(entry("loader deep", AutonKind::Match, &rp_fixture::backToLoader<21>, field(-58.77, -48, 90), 27, 29));
    reg.add(entry("hug", AutonKind::Match, &rp_fixture::hugsRedWall, field(-48, -70.2047 + 7.5 + 0.1, 0), 27, 29));
    reg.add(entry("a<b & \"c\"", AutonKind::Match, &rp_fixture::sevenSteps, field(-48, -48, 0), 27, 29));
    reg.add(entry("skills long", AutonKind::Skills, &rp_fixture::skillsLong, field(-48, -48, 0), 57, 59));
    // REFUSED by the registry (DROP mode): a stop past the 60 s period, and no start.
    reg.add(entry("too long", AutonKind::Skills, &rp_fixture::sevenSteps, field(-48, -48, 0), 58, 61));
    reg.add(entry("no start", AutonKind::Match, &rp_fixture::sevenSteps, comp::StartPose{}, 27, 29));
}

/// The review's cases (RPV1-A's review, 2026-11-13) — a second fixture file for robot two: a registry holds
/// 16 autons (competition::kMaxAutons), and the first one is full.
void registerTwoReview(comp::AutonRegistry& reg) {
    using comp::AutonKind;
    const comp::StartPose rel = comp::StartPose::relative();
    reg.add(entry("drive claw", AutonKind::Match, &rp_fixture::driveThenClaw, field(-48, -48, 0), 27, 29));
    reg.add(entry("curve", AutonKind::Match, &rp_fixture::curveThenDrive, field(-48, -48, 0), 27, 29));
    reg.add(entry("silent", AutonKind::Skills, &rp_fixture::loopsSilently, field(-48, -48, 0), 57, 59));
    reg.add(entry("throw after cut", AutonKind::Match, &rp_fixture::throwsAfterTheCut, field(-48, -48, 0), 1, 3));
    reg.add(entry("forty", AutonKind::Match, &rp_fixture::fortyInches, rel, 27, 29));
    comp::AutonEntry ownPark = entry("own park", AutonKind::Match, &rp_fixture::driveThenOpen, field(-48, -48, 0), 27, 29);
    ownPark.endAction = &rp_fixture::parkClosingClaw;
    reg.add(ownPark);
    // The verification round's (2026-11-13): where a run threw, beyond a drive (RP9b (e)-(i)); two starts
    // on one pace (RP28).
    reg.add(entry("pivot throw", AutonKind::Match, &rp_fixture::driveThenBlockingPivot, field(-48, -48, 0), 27, 29));
    reg.add(entry("claw pause", AutonKind::Match, &rp_fixture::driveClawPause, field(-48, -48, 0), 27, 29));
    reg.add(entry("bg pause", AutonKind::Match, &rp_fixture::backgroundPivotDrivePause, field(-48, -48, 0), 27, 29));
    reg.add(entry("bg blocking", AutonKind::Match, &rp_fixture::backgroundPivotThenBlockingPivot, field(-48, -48, 0),
                  27, 29));
    reg.add(entry("two starts", AutonKind::Match, &rp_fixture::twoStartsOnePace, field(-48, -48, 0), 27, 29));
}

void registerNineRefused(comp::AutonRegistry& reg) {
    static const char* const kNames[9] = {"r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"};
    for (const char* n : kNames) {
        reg.add(entry(n, comp::AutonKind::Skills, &rp_fixture::sevenSteps, field(-48, -48, 0), 58, 61));
    }
    reg.add(entry("seven", comp::AutonKind::Match, &rp_fixture::sevenSteps, field(-48, -48, 0), 27, 29));
}

void registerXFixtures(comp::AutonRegistry& reg) {
    const comp::StartPose rel = comp::StartPose::relative();
    reg.add(entry("lift", comp::AutonKind::Match, &rp_fixture::liftUpAndDown, rel, 27, 29));
    reg.add(entry("intake bg", comp::AutonKind::Match, &rp_fixture::backgroundIntakeTimesOut, rel, 27, 29));
    reg.add(entry("fly", comp::AutonKind::Match, &rp_fixture::spinUpFlywheel, rel, 27, 29));
}

void registerYFixtures(comp::AutonRegistry& reg) {
    const comp::StartPose rel = comp::StartPose::relative();
    reg.add(entry("y", comp::AutonKind::Match, &rp_fixture::robotYSwingAndTilt, rel, 27, 29));
    reg.add(entry("dr4b", comp::AutonKind::Match, &rp_fixture::robotYDr4b, rel, 27, 29));
    reg.add(entry("pieces", comp::AutonKind::Match, &rp_fixture::robotYPieceSensors, rel, 27, 29));
}

const rp::PreviewRobot kTwoFixtures{
    .key = "two",
    .title = "robot two",
    .table = &mech_fixture::robot_two::kRobotTwoMechanisms,  // the claw robot (ROB1-BT)
    .registerAutons = &registerTwoFixtures,
    .autonsFile = "test/route_preview_fixtures.hpp",
    .defaultRig = true,
    .provisional = std::span<const rp::Provisional>{rp::kRobotTwoProvisional},
};
const rp::PreviewRobot kTwoReview{
    .key = "review",
    .title = "robot two",
    .table = &mech_fixture::robot_two::kRobotTwoMechanisms,  // the claw robot (ROB1-BT)
    .registerAutons = &registerTwoReview,
    .autonsFile = "test/route_preview_fixtures.hpp (the review's)",
    .defaultRig = true,
    .provisional = std::span<const rp::Provisional>{rp::kRobotTwoProvisional},
};
const rp::PreviewRobot kNineRefused{
    .key = "nine",
    .title = "robot two",
    .table = &mech_fixture::robot_two::kRobotTwoMechanisms,  // the claw robot (ROB1-BT)
    .registerAutons = &registerNineRefused,
    .autonsFile = "a fixture with nine refusals",
    .defaultRig = true,
    .provisional = std::span<const rp::Provisional>{rp::kRobotTwoProvisional},
};
const rp::PreviewRobot kRobotX{
    .key = "x",
    .title = "robot x",
    .table = &mech_fixture::robot_x::kRobotXMechanisms,
    .registerAutons = &registerXFixtures,
    .autonsFile = "test/route_preview_fixtures.hpp",
    .defaultRig = false,
    .provisional = {},
};
/// Robot y (test/mechanism_fixtures.hpp): a STALL-homed arm, a linkage lift, a MEASURED-shape lift — the
/// shapes robot one's cascade has (stall-homed, measured), before robot one's own list lands.
const rp::PreviewRobot kRobotY{
    .key = "y",
    .title = "robot y",
    .table = &mech_fixture::robot_y::kRobotYMechanisms,
    .registerAutons = &registerYFixtures,
    .autonsFile = "test/route_preview_fixtures.hpp",
    .defaultRig = false,
    .provisional = {},
};
/// Robot one is the program's own (rp::kRobotOne): its real list, autons file and ledger.
const rp::PreviewRobot kFixtureRobots[] = {kTwoFixtures, kRobotX, rp::kRobotOne, kNineRefused, kRobotY, kTwoReview};

/// A scratch directory for pages (under the build tree, never the repository).
std::string scratchDir() {
    const std::filesystem::path p = std::filesystem::path{SHULIB_SOURCE_DIR} / "build" / "route_preview_test";
    std::filesystem::create_directories(p);
    return p.string();
}

struct Ran {
    int code = -1;
    std::string out;
    std::string err;
    std::string page;
    bool pageWritten = false;
    std::string path;
};

/// runPreview on `args` (an --out under the scratch directory is added unless `noOut`).
Ran run(std::vector<std::string> args, std::span<const rp::PreviewRobot> robots = kFixtureRobots,
        const std::string& file = "page.html", bool noOut = false) {
    Ran r;
    r.path = scratchDir() + "/" + file;
    std::filesystem::remove(r.path);
    if (!noOut) {
        args.push_back("--out");
        args.push_back(r.path);
    }
    std::vector<std::string_view> views(args.begin(), args.end());
    std::ostringstream out;
    std::ostringstream err;
    r.code = rp::runPreview(views, out, err, robots);
    r.out = out.str();
    r.err = err.str();
    std::ifstream f{r.path, std::ios::binary};
    if (f) {
        r.pageWritten = true;
        r.page.assign(std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{});
    }
    return r;
}

/// The fixture registry for `robot`.
comp::AutonRegistry registryOf(const rp::PreviewRobot& robot) {
    comp::AutonRegistry reg{shulib::bench::kAutonPeriods, comp::OnRefusal::DropEntry};
    robot.registerAutons(reg);
    return reg;
}
std::size_t indexOf(const comp::AutonRegistry& reg, std::string_view name) {
    for (std::size_t i = 0; i < reg.size(); ++i) {
        if (std::string_view{reg.at(i).name} == name) {
            return i;
        }
    }
    FAIL("no fixture auton named ", name);
    return 0;
}
rp::Model preview(const rp::PreviewRobot& robot, std::string_view name, const rp::PreviewOptions& opt = {}) {
    const comp::AutonRegistry reg = registryOf(robot);
    return rp::previewEntry(robot, reg, indexOf(reg, name), opt);
}

/// The IMU reads truth + `offsetDeg` (autons_test.cpp's OffsetImu).
struct OffsetImu final : shulib::sim::DegradationModel {
    explicit OffsetImu(double offset) : offsetDeg{offset} {}
    [[nodiscard]] shulib::math::Angle imuHeading(shulib::math::Angle truth, shulib::units::Time,
                                                 shulib::sim::Rng&) override {
        return truth + shulib::math::Angle::degrees(offsetDeg);
    }
    double offsetDeg;
};

/// An INDEPENDENT tap below the recorder: the truth after every pace.
struct Tap final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    const shulib::sim::SimHarness* h = nullptr;
    const motion_rig::PlantPacer* plant = nullptr;
    std::vector<shulib::sim::TruthSample> samples;
    std::vector<double> plantDistance;  ///< the plant pacer's own summed distance after each pace
    void pace() override {
        inner->pace();
        samples.push_back(h->sample());
        plantDistance.push_back(plant != nullptr ? plant->distance : -1.0);
    }
};

/// The from-below steps of `m` that are not the end action's.
std::vector<rp::Step> routineSteps(const rp::Model& m) {
    std::vector<rp::Step> out;
    for (const rp::Step& s : m.steps) {
        if (!s.endAction) {
            out.push_back(s);
        }
    }
    return out;
}

/// The points of the polyline with id="path" in `page`.
std::vector<std::pair<double, double>> pathPoints(const std::string& page) {
    std::vector<std::pair<double, double>> pts;
    const std::string key = "id=\"path\" class=\"path\" points=\"";
    const std::size_t at = page.find(key);
    if (at == std::string::npos) {
        return pts;
    }
    const std::size_t end = page.find('"', at + key.size());
    std::istringstream in{page.substr(at + key.size(), end - at - key.size())};
    std::string pair;
    while (in >> pair) {
        const std::size_t comma = pair.find(',');
        pts.emplace_back(std::stod(pair.substr(0, comma)), std::stod(pair.substr(comma + 1)));
    }
    return pts;
}

std::string unescape(std::string s) {
    const std::pair<const char*, const char*> kMap[] = {
        {"&lt;", "<"}, {"&gt;", ">"}, {"&quot;", "\""}, {"&#39;", "'"}, {"&amp;", "&"}};
    for (const auto& [from, to] : kMap) {
        for (std::size_t p = s.find(from); p != std::string::npos; p = s.find(from, p + std::strlen(to))) {
            s.replace(p, std::strlen(from), to);
        }
    }
    return s;
}

}  // namespace

// RP1 — the picture's path IS the plant's poses. Bug caught: a path drawn from anything but the
// plant's truth after each pace — the estimate (RM2), a sample taken one tick stale (RM1), a y axis
// not flipped (RM3). Checked three ways against things the preview does not compute: an INDEPENDENT
// tap below the recorder, the plant pacer's own summed distance, and the page's polyline parsed back
// and compared with the field transform written out here (5 px/in, +Y up, 74 in to the edge).
TEST_CASE("route preview RP1: every drawn sample is the plant's truth, from an independent tap and the page") {
    for (const char* name : {"seven", "skills long"}) {
        CAPTURE(name);
        Tap tap;
        rp::PreviewOptions opt;
        opt.onBuilt = [&](tester_rig::TesterStack& s, shulib::motion::ITickPacer*& recorderInner) {
            tap.inner = recorderInner;
            tap.h = &s.rig.h;
            tap.plant = &s.plant;
            recorderInner = &tap;
        };
        const comp::AutonRegistry reg = registryOf(kTwoFixtures);
        const rp::Model m = rp::previewEntry(kTwoFixtures, reg, indexOf(reg, name), opt);
        REQUIRE(m.samples.size() > 10);
        REQUIRE(tap.samples.size() >= m.samples.size());
        // The recorder's samples are the LAST m.samples.size() paces, one for one, bit for bit.
        const std::size_t off = tap.samples.size() - m.samples.size();
        for (std::size_t i = 0; i < m.samples.size(); ++i) {
            const shulib::sim::TruthSample& t = tap.samples[off + i];
            if (!(m.samples[i].x == t.x && m.samples[i].y == t.y && m.samples[i].theta == t.theta &&
                  m.samples[i].t + m.armedAt == t.t)) {
                FAIL("sample ", i, " is not the tap's: (", m.samples[i].x, ", ", m.samples[i].y, ") vs (", t.x, ", ",
                     t.y, ")");
            }
        }
        // The summed drawn path equals the PLANT PACER's own distance (the brief's P1: exactly) — nothing
        // moved before the first drawn sample, and the drawn samples add the same steps in the same order.
        double drawn = 0.0;
        for (std::size_t i = 1; i < m.samples.size(); ++i) {
            drawn += std::hypot(m.samples[i].x - m.samples[i - 1].x, m.samples[i].y - m.samples[i - 1].y);
        }
        CHECK(tap.plantDistance[off] == 0.0);
        CHECK(drawn == tap.plantDistance.back());
        CHECK(drawn > 30.0);
        // The page's polyline, parsed back, is every sample through the field transform (written here).
        const std::string page = rp::renderHtml(m);
        const auto pts = pathPoints(page);
        REQUIRE(pts.size() == m.samples.size());
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const double px = (m.samples[i].x + 74.0) * 5.0;
            const double py = (74.0 - m.samples[i].y) * 5.0;
            if (std::abs(pts[i].first - px) > 0.01 || std::abs(pts[i].second - py) > 0.01) {
                FAIL("page point ", i, " (", pts[i].first, ", ", pts[i].second, ") is not the sample's (", px, ", ", py,
                     ")");
            }
        }
    }
}

// RP1 (the estimate) — under an IMU that reads 5 degrees off, and a RELATIVE start placed on the field,
// the drawn path is the TRUTH: it ends where the plant ends (on the field, at y near -8), while the
// estimate the robot itself held ends near (40, 0) in its own frame. A path drawn from the estimate
// (RM2) ends 40 in to the right of where the plant is.
TEST_CASE("route preview RP1: the path is the truth, not the estimate, under an IMU that reads 5 degrees off") {
    OffsetImu imu{5.0};
    Tap tap;
    rp::PreviewOptions opt;
    opt.place = true;
    opt.placeX = -48.0;
    opt.placeY = -48.0;
    opt.placeDeg = 90.0;
    opt.degradation = &imu;
    opt.onBuilt = [&](tester_rig::TesterStack& s, shulib::motion::ITickPacer*& inner) {
        tap.inner = inner;
        tap.h = &s.rig.h;
        inner = &tap;
    };
    comp::AutonRegistry team{shulib::bench::kAutonPeriods, comp::OnRefusal::DropEntry};
    shulib::bench::registerAutons(team);
    const rp::Model m = rp::previewEntry(rp::kRobotTwo, team, indexOf(team, "40in forward"), opt);
    REQUIRE(!m.samples.empty());
    const rp::Sample& end = m.samples.back();
    CHECK(end.x == tap.samples.back().x);
    CHECK(end.y == tap.samples.back().y);
    CHECK(end.y > -15.0);              // the truth: 40 in up the field from y = -48
    CHECK(std::abs(end.ex - end.x) > 30.0);  // the estimate is elsewhere (its own frame)
    const auto pts = pathPoints(rp::renderHtml(m));
    REQUIRE(!pts.empty());
    CHECK(std::abs(pts.back().first - (end.x + 74.0) * 5.0) < 0.01);
    CHECK(std::abs(pts.back().second - (74.0 - end.y) * 5.0) < 0.01);
}

// RP2 — a FIELD start is drawn at the declared start. Bug caught: the plant left at the rig's origin
// (RM4) — the brief's P5b: the estimate then ends "right" while the truth is 48 in away, a picture that
// is wrong and looks right.
TEST_CASE("route preview RP2: a FIELD start is drawn where it is declared, and the estimate agrees at the end") {
    const rp::Model m = preview(kTwoFixtures, "seven");
    REQUIRE(!m.samples.empty());
    CHECK(std::abs(m.samples.front().x + 48.0) < 1e-9);
    CHECK(std::abs(m.samples.front().y + 48.0) < 1e-9);
    CHECK(std::abs(m.samples.front().theta) < 1e-9);
    CHECK(std::hypot(m.samples.back().x - m.samples.back().ex, m.samples.back().y - m.samples.back().ey) < 0.5);
    CHECK(m.samples.back().y > -30.0);  // it went up the field to y = -24
    const auto pts = pathPoints(rp::renderHtml(m));
    REQUIRE(!pts.empty());
    CHECK(std::abs(pts.front().first - (-48.0 + 74.0) * 5.0) < 0.01);
    CHECK(std::abs(pts.front().second - (74.0 + 48.0) * 5.0) < 0.01);
}

// RP3 — a RELATIVE start is drawn at --place: the team's 40 in auton placed at (-48, -48, 90) ends at
// the placement composed with the unplaced run's end (the plant is a rigid transform of itself: equal to
// 1e-6), which is (-48.00, -8.23) to 0.01 (the brief's P5c). Without --place the red "CENTRE" line is
// on the page and on stdout. Bug caught: --place's heading applied with the wrong sign (RM5) — the
// robot would be drawn driving DOWN the field.
TEST_CASE("route preview RP3: a RELATIVE start is drawn at --place; without it, the page says CENTRE in red") {
    comp::AutonRegistry team{shulib::bench::kAutonPeriods, comp::OnRefusal::DropEntry};
    shulib::bench::registerAutons(team);
    const std::size_t i = indexOf(team, "40in forward");
    const rp::Model bare = rp::previewEntry(rp::kRobotTwo, team, i, {});
    rp::PreviewOptions opt;
    opt.place = true;
    opt.placeX = -48.0;
    opt.placeY = -48.0;
    opt.placeDeg = 90.0;
    const rp::Model placed = rp::previewEntry(rp::kRobotTwo, team, i, opt);
    REQUIRE(!bare.samples.empty());
    REQUIRE(!placed.samples.empty());
    // placement ∘ bare end: rotate (x, y) by 90 degrees, then translate.
    const double ex = -48.0 - bare.samples.back().y;
    const double ey = -48.0 + bare.samples.back().x;
    CHECK(std::abs(placed.samples.back().x - ex) < 1e-6);
    CHECK(std::abs(placed.samples.back().y - ey) < 1e-6);
    CHECK(std::abs(placed.samples.back().x + 48.00) < 0.01);
    CHECK(std::abs(placed.samples.back().y + 8.23) < 0.01);
    CHECK_FALSE(placed.placedAtCentre);
    CHECK(bare.placedAtCentre);
    const Ran withPlace = run({"40in forward", "--place", "-48,-48,90"}, std::span<const rp::PreviewRobot>{rp::kRobots});
    const Ran without = run({"40in forward"}, std::span<const rp::PreviewRobot>{rp::kRobots});
    CHECK(withPlace.code == 0);
    CHECK(without.code == 0);
    CHECK(without.page.find("class=\"notice\">This auton has a RELATIVE start and no --place was given") !=
          std::string::npos);
    CHECK(without.out.find("!! This auton has a RELATIVE start and no --place was given") != std::string::npos);
    CHECK(withPlace.page.find("no --place was given") == std::string::npos);
    CHECK(withPlace.page.find("RELATIVE, placed with --place at (-48.0, -48.0) 90.0") != std::string::npos);
}

// RP6 — a failed step is drawn as failed. A chain whose step 3 (a drive with a 0.3 s limit) times out:
// step 3 is red where it happened, its sentence is the Routine's stop line WORD FOR WORD, steps 4 and 5
// are only in the grey "skipped" lines (never on the path — nothing ran them), the word is ROUTINE
// STOPPED, and step 1 begins at T+0.00 (every T+ from the arming — RM9 would start it at T+3.5).
TEST_CASE("route preview RP6: a timed-out step is red with the library's own sentence; the skipped ones are not drawn") {
    const rp::Model m = preview(kTwoFixtures, "times out");
    CHECK(m.word == "ROUTINE STOPPED");
    const auto steps = routineSteps(m);
    REQUIRE(steps.size() == 3);
    CHECK(steps[0].number == 1);
    CHECK(std::abs(steps[0].t0) < 1e-9);
    CHECK(steps[0].mark == rp::Mark::Ok);
    CHECK(steps[1].kind == rp::StepKind::Turn);
    CHECK(steps[2].number == 3);
    CHECK(steps[2].verdict == "TIMED_OUT");
    CHECK(steps[2].mark == rp::Mark::Failed);
    CHECK(std::abs((steps[2].t1 - steps[2].t0) - 0.3) < 0.02);
    // The stop line, verbatim, red; the two skip lines, grey.
    int stops = 0;
    int skips = 0;
    for (const rp::LogLine& l : m.routineLines) {
        if (l.text.rfind("routine 'times-out' STOPPED at step 3 (moveTo): ", 0) == 0) {
            ++stops;
            CHECK(l.mark == rp::Mark::Failed);
            CHECK(l.s >= steps[2].s1 - 1);
        }
        if (l.text.find(") skipped — stopped at step 3") != std::string::npos) {
            ++skips;
            CHECK(l.mark == rp::Mark::Grey);
        }
    }
    CHECK(stops == 1);
    CHECK(skips == 2);
    // Nothing on the path after step 3 but the park's brake.
    for (const rp::Step& s : m.steps) {
        CHECK((s.number <= 3 || s.endAction));
        CHECK(s.mark != rp::Mark::Grey);
    }
    const std::string page = rp::renderHtml(m);
    CHECK(page.find("[RTN] routine &#39;times-out&#39; STOPPED at step 3 (moveTo): ") != std::string::npos);
}

// RP8 — a refused start. (a) A skills auton whose stop is 61 s and (b) one with no start are REFUSED by
// the registry, which then never registers them: they are found BY NAME among its sentences and drawn
// as a REFUSED page with the registry's own sentence, no path, exit 1. (c) runAuton's own refusal (a
// ROUTE auton with no command registry — unreachable from a team file, so a fixture through the same
// code): the fold draws it REFUSED, nothing moved. Bug caught: a registry built to THROW (RM10) — the
// preview would die on the team's first bad entry instead of drawing it.
TEST_CASE("route preview RP8: refused autons are drawn REFUSED with the program's own sentence, exit 1") {
    Ran a;
    REQUIRE_NOTHROW(a = run({"too long"}));  // a registry built to THROW would end here (RM10)
    CHECK(a.code == 1);
    REQUIRE(a.pageWritten);
    CHECK(a.page.find("auton &#39;too long&#39;: hardStopAt 61.00 s is past its SKILLS period of 60.00 s -- the "
                      "field disables the robot first") != std::string::npos);
    CHECK(a.page.find("REFUSED") != std::string::npos);
    CHECK(a.page.find("id=\"path\"") == std::string::npos);
    Ran b;
    REQUIRE_NOTHROW(b = run({"no start"}));
    CHECK(b.code == 1);
    CHECK(b.page.find("auton &#39;no start&#39;: has no start pose -- declare FIELD(x, y, heading) or RELATIVE") !=
          std::string::npos);
    // (c) runAuton's own refusal, through foldRun: an entry handed a period its stop does not fit.
    tester_rig::TesterStack s{rp::standInDrive()};
    const comp::AutonEntry route = entry("refused here", comp::AutonKind::Match, &rp_fixture::sevenSteps,
                                         comp::StartPose::relative(), 27, 29);
    shulib::bench::TesterRunRecord rec{};
    rec.run = comp::runAuton(s.chassis, s.guard, route, &s.commands, s.sink, s.mechanisms, shulib::units::Time{20.0});
    REQUIRE(rec.run.outcome == comp::AutonRunOutcome::Refused);
    rp::Recorder recorder;
    rp::MotionLog log;
    rp::Model m = rp::askedModel(rp::kRobotTwo, route, 30.0, {});
    rp::foldRun(m, rp::kRobotTwo, recorder, log, s.sink, &rec);
    CHECK(m.refused);
    CHECK(m.word == "REFUSED");
    CHECK(m.why == "auton 'refused here': hardStopAt 29.00 s is past its MATCH period of 20.00 s -- the field "
                   "disables the robot first");
    CHECK(m.samples.empty());
    CHECK(m.exitCode == 1);
}

// RP9 — an auton that throws: PROGRAM ERROR with its own words, the path up to the throw, "after step
// 1", exit 1. Bug caught: a throw drawn as FINISHED (RM11).
TEST_CASE("route preview RP9: an auton that throws is a PROGRAM ERROR with its text and the path to the throw") {
    const Ran r = run({"thrower", "--place", "0,0,0"});
    CHECK(r.code == 1);
    CHECK(r.out.find("PROGRAM ERROR") != std::string::npos);
    CHECK(r.out.find("THREW: a student's helper threw: no such stack") != std::string::npos);
    CHECK(r.page.find("FINISHED") == std::string::npos);
    const rp::Model m = preview(kTwoFixtures, "thrower");
    CHECK(m.threw);
    CHECK(m.word == "PROGRAM ERROR");
    REQUIRE(!m.samples.empty());
    CHECK(std::abs(m.samples.back().x - 20.0) < 0.6);  // the 20 in drive ran; nothing after it
    bool after = false;
    for (const std::string& n : m.notices) {
        after = after || n == "PROGRAM ERROR after step 1 (counted from below)";
    }
    CHECK(after);
    // The summary says WHEN it threw (the last pace's time), never a "scoring ended" the guard never
    // reported (it reads T+0.00 on every throw — the review's finding).
    char when[32];
    std::snprintf(when, sizeof when, "%.2f", m.samples.back().t);
    CHECK(m.samples.back().t > 0.5);
    CHECK(r.out.find("PROGRAM ERROR \xe2\x80\x94 it threw at T+" + std::string{when} + " s of the 30 s period") !=
          std::string::npos);
    CHECK(r.out.find("scoring ended") == std::string::npos);
    CHECK(r.page.find("scoring ended") == std::string::npos);
}

// RP10 — an unknown name: exit 2, every registered name listed, NO file written; a name differing only
// in case is unknown too. Bug caught: a nearest or case-folded match (RM12) — the preview would draw a
// DIFFERENT auton than the one asked for.
TEST_CASE("route preview RP10: an unknown or wrongly-cased name draws nothing and lists the real names") {
    const auto robots = std::span<const rp::PreviewRobot>{rp::kRobots};
    for (const char* wrong : {"skils", "40IN FORWARD", "40in forward ", "40in"}) {
        CAPTURE(wrong);
        const Ran r = run({wrong}, robots);
        CHECK(r.code == 2);
        CHECK_FALSE(r.pageWritten);
        CHECK(r.err.find("no auton named '" + std::string{wrong} + "' in src/team/autons.cpp") != std::string::npos);
        CHECK(r.err.find("'40in forward'") != std::string::npos);
        CHECK(r.err.find("'TEST 24in turn back'") != std::string::npos);
    }
    const Ran listed = run({"--list"}, robots, "page.html", true);
    CHECK(listed.code == 0);
    CHECK(listed.out.find("'40in forward'") != std::string::npos);
    const Ran none = run({}, robots, "page.html", true);
    CHECK(none.code == 2);
    const Ran badFlag = run({"40in forward", "--colour"}, robots);
    CHECK(badFlag.code == 2);
    CHECK_FALSE(badFlag.pageWritten);
}

// RP11 — going THROUGH a wall. A 200 in drive from the field's centre: THROUGH at the FIRST sample whose
// outline's CORNER passes the right wall's face (70.2047 in) by more than 1 in — not when the centre
// does (7.5 in later — RM13), not at mere contact, not at a wall drawn at 72 (RM14). Everything after it
// is faded, and the command fails.
TEST_CASE("route preview RP11: a drive through a wall is THROUGH at the first corner past face + 1 in, exit 1") {
    const rp::Model m = preview(kTwoFixtures, "far");
    REQUIRE(m.through);
    const double face = shulib::sim::FieldModel::kInnerFace;
    CHECK(std::abs(face - 70.2047) < 1e-4);
    const auto cornerPast = [&](std::size_t s) {
        const rp::Outline o = rp::outlineAt(m.samples[s].x, m.samples[s].y, m.samples[s].theta);
        double worst = -1e9;
        for (double x : o.cx) {
            worst = std::max(worst, x - face);
        }
        return worst;
    };
    REQUIRE(m.throughAt > 0);
    CHECK(cornerPast(m.throughAt) > 1.0);
    CHECK(cornerPast(m.throughAt - 1) <= 1.0);
    CHECK(m.samples[m.throughAt].x < face);  // the centre is still inside: it is the OUTLINE that went through
    const rp::FieldNote& last = m.field.back();
    CHECK(last.kind == rp::Contact::Through);
    CHECK(last.element == "right wall");
    CHECK(last.step == 1);
    CHECK(std::abs(last.depth - cornerPast(m.throughAt)) < 1e-9);
    // The TOUCH came first, at the first sample a corner reached the face.
    bool touched = false;
    for (const rp::FieldNote& f : m.field) {
        if (f.kind == rp::Contact::Touch && f.element == "right wall") {
            touched = true;
            CHECK(cornerPast(f.s) > 0.0);
            CHECK(cornerPast(f.s - 1) <= 0.0);
        }
    }
    CHECK(touched);
    CHECK(m.exitCode == 1);
    const std::string page = rp::renderHtml(m);
    CHECK(page.find("class=\"faded\"") != std::string::npos);
    CHECK(page.find("THROUGH: the host test plant has no walls; on the robot it hits the right wall here") !=
          std::string::npos);
    const Ran r = run({"far"});
    CHECK(r.code == 1);
}

// RP13 — mechanism steps are drawn where they BEGAN, with their verdict word from the MECH line, and a
// grab nothing confirms (UNCONFIRMED, counted done) in its OWN colour, never green. The from-below
// numbering skips the pause and the then(): drive 1, claw 2, pivot 3, turn 4, drive 5, claw 6.
TEST_CASE("route preview RP13: mechanism steps are marked where they began, UNCONFIRMED in its own colour") {
    const rp::Model m = preview(kTwoFixtures, "seven");
    CHECK(m.word == "FINISHED");
    const auto steps = routineSteps(m);
    REQUIRE(steps.size() == 6);
    const char* const kinds[6] = {"drive", "claw", "claw pivot 6", "turn", "drive", "claw"};
    for (std::size_t i = 0; i < 6; ++i) {
        CHECK(rp::stepLabel(steps[i]) == kinds[i]);
        CHECK(steps[i].number == static_cast<int>(i) + 1);
    }
    CHECK(steps[1].verdict == "SUCCEEDED");
    CHECK(steps[1].mark == rp::Mark::Ok);
    CHECK(steps[1].sentence.rfind("'claw': SUCCEEDED", 0) == 0);
    CHECK(steps[2].verdict == "SUCCEEDED");
    CHECK(steps[5].verdict == "UNCONFIRMED");
    CHECK(steps[5].mark == rp::Mark::Unconfirmed);
    CHECK(steps[5].sentence.find("counted done") != std::string::npos);
    // The claw began where the first drive ended (the robot still), 0.3 s after it (the pause).
    const rp::Sample& driveEnd = m.samples[steps[0].s1];
    const rp::Sample& clawAt = m.samples[steps[1].s0];
    CHECK(std::hypot(driveEnd.x - clawAt.x, driveEnd.y - clawAt.y) < 0.05);
    CHECK(std::abs((steps[1].t0 - steps[0].t1) - 0.3) < 0.02);
    CHECK(std::abs((steps[1].t1 - steps[1].t0) - 0.5) < 0.02);  // the claw's declared 0.5 s
    CHECK_FALSE(steps[1].background);
    const std::string page = rp::renderHtml(m);
    CHECK(page.find("class=\"mech unconfirmed hatch\"") != std::string::npos);
}

// RP14 — a wall reset that refuses (no sensor on the tester rig) is AMBER, "no sensor", never green —
// though its then() step counts done and the run FINISHES. Bug caught: the mark taken from the step's
// count (RM15).
TEST_CASE("route preview RP14: a refused wall reset is amber with its own line, though the step counts done") {
    const rp::Model m = preview(kTwoFixtures, "wall");
    CHECK(m.word == "FINISHED");
    REQUIRE(m.wallLines.size() == 1);
    CHECK(m.wallLines[0].text.find("REFUSED") != std::string::npos);
    CHECK(m.wallLines[0].text.find("no sensor") != std::string::npos);
    CHECK(m.wallLines[0].mark == rp::Mark::Amber);
    const std::string page = rp::renderHtml(m);
    CHECK(page.find("class=\"wallmark amber\"") != std::string::npos);
    CHECK(page.find("class=\"wallmark ok\"") == std::string::npos);
    // It is drawn where the robot was when it refused: after the first drive (x = -38).
    CHECK(std::abs(m.samples[m.wallLines[0].s].x + 38.0) < 0.6);
}

// RP15 — the caption, the robot line and the stamp line are on EVERY page, and nothing on any page
// reaches outside it: no "http", no src= or href=, and never the s-word the project retired for the host test
// plant (spelled in two halves below, so this file does not carry it either). Bug caught: the
// caption dropped (RM16).
TEST_CASE("route preview RP15: every page carries the caption and the robot line, and nothing from outside") {
    for (const char* name : {"seven", "times out", "far", "wall", "thrower", "too long", "background"}) {
        CAPTURE(name);
        const Ran r = run({name, "--place", "0,0,0"});
        REQUIRE(r.pageWritten);
        CHECK(r.page.find("IDEAL host test plant, not the robot: no wheel slip, no battery sag, motors that answer at "
                          "once.") != std::string::npos);
        CHECK(r.page.find("STAND-IN drive") != std::string::npos);
        CHECK(r.page.find("commit not stamped") != std::string::npos);
        CHECK(r.page.find("http") == std::string::npos);
        CHECK(r.page.find("src=") == std::string::npos);
        CHECK(r.page.find("href=") == std::string::npos);
        std::string lower = r.page;
        for (char& c : lower) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        CHECK(lower.find(std::string{"simul"} + "ator") == std::string::npos);
        CHECK(r.out.find("IDEAL host test plant") != std::string::npos);
        // The outline's two numbers are the ones the check uses (printed from its constants).
        char outline[160];
        std::snprintf(outline, sizeof outline, "The robot is drawn as a %g in square centred on the point the plant "
                      "tracks, with %g in of allowance before an overlap counts as going THROUGH", rp::kPreviewFootprintIn,
                      rp::kThroughAllowanceIn);
        CHECK(r.page.find(outline) != std::string::npos);
    }
}

// RP16 — the stdout table equals the page's table, cell by cell (both drawn from tableRows of one Model).
TEST_CASE("route preview RP16: the stdout table and the page's table are the same cells") {
    const Ran r = run({"seven"});
    REQUIRE(r.pageWritten);
    std::vector<std::vector<std::string>> text;
    std::istringstream in{r.out};
    std::string line;
    bool inTable = false;
    while (std::getline(in, line)) {
        if (line.rfind("#", 0) == 0 && line.find(" | step") != std::string::npos) {
            inTable = true;
        }
        if (!inTable) {
            continue;
        }
        if (line.find(" | ") == std::string::npos) {
            break;
        }
        std::vector<std::string> cells;
        std::size_t from = 0;
        for (std::size_t at = line.find(" | "); ; at = line.find(" | ", from)) {
            std::string c = line.substr(from, at == std::string::npos ? std::string::npos : at - from);
            while (!c.empty() && c.back() == ' ') {
                c.pop_back();
            }
            cells.push_back(c);
            if (at == std::string::npos) {
                break;
            }
            from = at + 3;
        }
        text.push_back(cells);
    }
    std::vector<std::vector<std::string>> html;
    const std::size_t t0 = r.page.find("<table id=\"steps\">");
    const std::size_t t1 = r.page.find("</table>", t0);
    REQUIRE(t0 != std::string::npos);
    const std::string table = r.page.substr(t0, t1 - t0);
    for (std::size_t at = table.find("<tr"); at != std::string::npos; at = table.find("<tr", at + 3)) {
        const std::size_t end = table.find("</tr>", at);
        const std::string row = table.substr(at, end - at);
        std::vector<std::string> cells;
        for (std::size_t c = row.find("<t"); c != std::string::npos; c = row.find("<t", c + 2)) {
            if (row.compare(c, 3, "<tr") == 0) {
                continue;
            }
            const std::size_t open = row.find('>', c) + 1;
            const std::size_t close = row.find("</t", open);
            cells.push_back(unescape(row.substr(open, close - open)));
            c = close;
        }
        html.push_back(cells);
    }
    REQUIRE(text.size() == 8);  // the header, six steps, the park
    CHECK(text == html);
}

// RP17 — determinism: two runs of the same tree write byte-identical pages. Bug caught: a wall-clock
// stamp in the page (RM17).
TEST_CASE("route preview RP17: two runs of the same auton write the same bytes") {
    const Ran a = run({"seven"}, kFixtureRobots, "a.html");
    const Ran b = run({"seven"}, kFixtureRobots, "b.html");
    REQUIRE(a.pageWritten);
    REQUIRE(b.pageWritten);
    CHECK(a.page.size() > 1000);
    CHECK(a.page == b.page);
    CHECK(a.out.substr(0, a.out.find("page: ")) == b.out.substr(0, b.out.find("page: ")));
}

// RP18 — escaping: an auton named `a<b & "c"` (printable ASCII the registry accepts) appears escaped
// everywhere on the page; no raw `<b`. Bug caught: no HTML escaping (RM18).
TEST_CASE("route preview RP18: a name with markup in it is escaped on the page") {
    const Ran r = run({"a<b & \"c\""}, kFixtureRobots, "esc.html");
    CHECK(r.code == 0);
    REQUIRE(r.pageWritten);
    CHECK(r.page.find("a&lt;b &amp; &quot;c&quot;") != std::string::npos);
    CHECK(r.page.find("a<b") == std::string::npos);
    CHECK(r.page.find("\"c\"") == std::string::npos);
}

// RP27 — the loop with no pace: C1's `for (;;) { moveTo(A); reverseTo(B); }` on a 57/59 s skills schedule
// ends through the NO-PACE STOP — not std::bad_alloc, not the launcher's timeout: the page exists, says
// "the auton keeps looping without letting time pass", the process exits 1, in under 5 s and 200 MB.
// The stop ends the process (std::_Exit), so runPreview runs in a FORKED child here, with its own
// 2 GiB address-space bound and a 60 s alarm. Bug caught: the stop disabled (RM22) — the child then
// grows to its bound and the library reports std::bad_alloc as a PROGRAM ERROR, or the alarm kills it.
TEST_CASE("route preview RP27: a loop that never lets time pass ends through the no-pace stop, page written, exit 1") {
    const std::string path = scratchDir() + "/forever.html";
    std::filesystem::remove(path);
    std::fflush(nullptr);  // nothing buffered in this process may be written twice by the child
    const auto t0 = std::chrono::steady_clock::now();
    const pid_t child = fork();
    REQUIRE(child >= 0);
    if (child == 0) {
        const rlimit as{2ULL << 30, 2ULL << 30};
        setrlimit(RLIMIT_AS, &as);
        alarm(60);
        // The child never returns into the test runner: whatever happens, it ends here.
        try {
            const std::string_view args[] = {"forever", "--out", path};
            std::ostringstream out;
            std::ostringstream err;
            const int code = rp::runPreview(args, out, err, kFixtureRobots);
            std::_Exit(code == 1 ? 77 : 78);  // returned instead of the stop: never 1
        } catch (...) {
            std::_Exit(79);  // threw (std::bad_alloc, when the stop does not fire): never 1
        }
    }
    int status = 0;
    rusage use{};
    REQUIRE(wait4(child, &status, 0, &use) == child);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    REQUIRE(WIFEXITED(status));
    CHECK(WEXITSTATUS(status) == 1);
    CHECK(secs < 5.0);
    CHECK(use.ru_maxrss < 200L * 1024L);  // KiB on Linux
    std::ifstream f{path, std::ios::binary};
    REQUIRE(f);
    const std::string page{std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
    CHECK(page.find("the auton keeps looping without letting time pass; on the robot it would sit there until the "
                    "field disables it, and its park would never run") != std::string::npos);
    CHECK(page.find("LOOPS WITHOUT LETTING TIME PASS") != std::string::npos);
    CHECK(page.find("bad_alloc") == std::string::npos);
    // "after step N": N is the step the routine's own STOPPED line names.
    const std::size_t stop = page.find("STOPPED at step ");
    REQUIRE(stop != std::string::npos);
    const std::string n = page.substr(stop + 16, page.find(' ', stop + 16) - stop - 16);
    CHECK(page.find("after step " + n + " the auton keeps looping") != std::string::npos);
    CHECK(page.find("The looping lines (the first 200 of ") != std::string::npos);
    // The flood is the looping section's, never the page's log of what the library said (PROGRESS Δ11):
    // that log holds the run's own lines only — a real 60 s run logs about 40.
    const std::size_t logAt = page.find("<h2>Every line the library logged from the start of the auton</h2><pre>");
    REQUIRE(logAt != std::string::npos);
    const std::string log = page.substr(logAt, page.find("</pre>", logAt) - logAt);
    std::size_t logLines = 0;
    for (const char c : log) {
        logLines += c == '\n' ? 1U : 0U;
    }
    CHECK(logLines < 300);
}

// RP28 — a background mechanism's verdict is ITS OWN, attributed by the mechanism's NAME: C2's
// startRun(pivot 6, 12 V, 1 s), a 40 in drive, waitUntilDone. The pivot's SUCCEEDED is on the pivot's
// span (from where it began to its T+1.0 verdict, mid-drive); the drive carries NO mechanism verdict
// and stays green. The failing form (B1, robot x's intake run until it sees a piece, which never comes)
// draws ITS span red, the drive still green. Bug caught: MECH lines attributed to the step open when
// they arrive (RM21).
TEST_CASE("route preview RP28: a background mechanism's verdict is on its own span, never on the open drive") {
    rp::PreviewOptions opt;
    opt.place = true;
    opt.placeX = -48.0;
    opt.placeY = -48.0;
    const rp::Model m = preview(kTwoFixtures, "background", opt);
    CHECK(m.word == "FINISHED");
    const auto steps = routineSteps(m);
    REQUIRE(steps.size() == 2);
    CHECK(steps[0].what == "claw pivot 6");
    CHECK(steps[0].number == 1);
    CHECK(steps[0].verdict == "SUCCEEDED");
    CHECK(steps[0].background);
    CHECK(std::abs(steps[0].t1 - 1.0) < 0.03);
    CHECK(m.samples[steps[0].s1].x > m.samples[steps[0].s0].x + 5.0);  // it ended mid-drive
    CHECK(steps[1].kind == rp::StepKind::Drive);
    CHECK(steps[1].mark == rp::Mark::Ok);
    CHECK(steps[1].sentence.empty());
    CHECK(steps[1].t1 > steps[0].t1);  // the drive was open when the verdict arrived
    CHECK(rp::renderHtml(m).find("class=\"span ok\"") != std::string::npos);

    const rp::Model x = preview(kRobotX, "intake bg", opt);
    const auto xs = routineSteps(x);
    REQUIRE(xs.size() == 2);
    CHECK(xs[0].what == "intake");
    CHECK(xs[0].verdict == "TIMED_OUT");
    CHECK(xs[0].mark == rp::Mark::Failed);
    CHECK(xs[0].background);
    CHECK(xs[1].kind == rp::StepKind::Drive);
    CHECK(xs[1].mark == rp::Mark::Ok);
    CHECK(xs[1].sentence.empty());
    CHECK(rp::renderHtml(x).find("class=\"span failed\"") != std::string::npos);

    // Two start forms on ONE pace (the verification round): pivot 7 started, then pivot 6, with nothing
    // paced between — from below they began at the same instant, and they are numbered in robot two's LIST
    // order (pivot 6 first), not the code's; guide 16 says so. Each still gets its own verdict, by name.
    const rp::Model two = preview(kTwoReview, "two starts");
    const auto ts = routineSteps(two);
    REQUIRE(ts.size() == 3);
    CHECK(ts[0].what == "claw pivot 6");
    CHECK(ts[0].number == 1);
    CHECK(ts[1].what == "claw pivot 7");
    CHECK(ts[1].number == 2);
    CHECK(ts[0].s0 == ts[1].s0);
    CHECK(ts[2].kind == rp::StepKind::Drive);
    CHECK(ts[2].number == 3);
    CHECK(ts[0].verdict == "SUCCEEDED");
    CHECK(ts[1].verdict == "SUCCEEDED");
    CHECK(ts[0].background);
    CHECK(ts[1].background);
}

// RP29 — a TOUCH is not a failure. (a) A FIELD start with its back on the red wall at y = -62.7 turned
// 0.1 degrees: its corner reaches 0.008 in past the face — a TOUCH at T+0.00, exit 0. (b) Backing up to
// a Loader, 0.6 in into it: "touches the Loader", exit 0. (c) The same 2.1 in in: THROUGH (a Loader is
// solid), exit 1. (d) Along the red wall 0.1 in inside its face: no note at all. Bug caught: every
// overlap counted as THROUGH (RM23).
TEST_CASE("route preview RP29: touching a wall or a Loader is an amber note; only going through fails") {
    const rp::Model a = preview(kTwoFixtures, "away");
    REQUIRE(!a.field.empty());
    CHECK(a.field[0].kind == rp::Contact::Touch);
    CHECK(a.field[0].element == "red wall");
    CHECK(a.field[0].s == 0);
    CHECK(a.field[0].t == doctest::Approx(0.0));
    CHECK(a.field[0].depth > 0.0);
    CHECK(a.field[0].depth < 0.02);
    CHECK_FALSE(a.through);
    CHECK(a.exitCode == 0);
    CHECK(run({"away"}).code == 0);

    const rp::Model b = preview(kTwoFixtures, "loader");
    CHECK_FALSE(b.through);
    bool loader = false;
    for (const rp::FieldNote& f : b.field) {
        loader = loader || (f.kind == rp::Contact::Touch && f.element == "Loader");
    }
    CHECK(loader);
    CHECK(b.exitCode == 0);
    const Ran bRun = run({"loader"});
    CHECK(bRun.code == 0);
    CHECK(bRun.out.find("touches the Loader at T+") != std::string::npos);

    const rp::Model c = preview(kTwoFixtures, "loader deep");
    REQUIRE(c.through);
    CHECK(c.field.back().element == "Loader");
    CHECK(c.field.back().depth > 1.0);
    CHECK(c.exitCode == 1);

    const rp::Model d = preview(kTwoFixtures, "hug");
    CHECK(d.field.empty());
    CHECK_FALSE(d.through);
    CHECK(d.exitCode == 0);
    // It really hugged the wall: its lowest corner stayed within 0.2 in of the face, never past it.
    double nearest = 1e9;
    for (const rp::Sample& s : d.samples) {
        const rp::Outline o = rp::outlineAt(s.x, s.y, s.theta);
        for (double y : o.cy) {
            nearest = std::min(nearest, y + shulib::sim::FieldModel::kInnerFace);
        }
    }
    CHECK(nearest > 0.0);
    CHECK(nearest < 0.2);
}

// RP31 — two routines: a helper builds its OWN Routine "score", which stops at ITS step 2 ('intake' is
// not on robot two). The page quotes the helper's line with its name ("routine 'score' STOPPED at step
// 2"), and the main routine's line keeps the main routine's numbering ("routine 'main' STOPPED at step 2
// (score)"). Bug caught: the routine's name dropped from the quoted lines, so the two numberings read as
// one (RM25, re-cut for A0: the numbers on these lines are the library's).
TEST_CASE("route preview RP31: a helper's own routine keeps its own name and numbering on the page") {
    const rp::Model m = preview(kTwoFixtures, "helper");
    bool helper = false;
    bool main = false;
    for (const rp::LogLine& l : m.routineLines) {
        helper = helper || l.text.rfind("routine 'score' STOPPED at step 2 (intake): ", 0) == 0;
        main = main || l.text.rfind("routine 'main' STOPPED at step 2 (score): ", 0) == 0;
    }
    CHECK(helper);
    CHECK(main);
    const std::string page = rp::renderHtml(m);
    CHECK(page.find("[RTN] routine &#39;score&#39; STOPPED at step 2 (intake)") != std::string::npos);
    CHECK(page.find("[RTN] routine &#39;main&#39; STOPPED at step 2 (score)") != std::string::npos);
    // The top line says why the route stopped there (the brief's landmine 16).
    bool notice = false;
    for (const std::string& n : m.notices) {
        notice = notice || n.find("a step on any other mechanism ('intake' here) stops the route") != std::string::npos;
    }
    CHECK(notice);
}

// RP32 — a refused entry is found by its name (exit 1, not "no auton named", exit 2); with nine refused
// entries the ninth's name is beyond what the registry keeps, and the preview says so (exit 2).
TEST_CASE("route preview RP32: a refused auton is found by name; beyond the registry's eight, the preview says so") {
    const Ran found = run({"too long"});
    CHECK(found.code == 1);
    CHECK(found.err.find("no auton named") == std::string::npos);
    // A PART of a refused name is no name at all (the brief's D7: never a nearest match).
    for (const char* part : {"long", "too", "oo long"}) {
        CAPTURE(part);
        const Ran p = run({part});
        CHECK(p.code == 2);
        CHECK_FALSE(p.pageWritten);
        CHECK(p.err.find("no auton named '" + std::string{part} + "'") != std::string::npos);
    }
    const Ran eighth = run({"r8", "--robot", "nine"});
    CHECK(eighth.code == 1);
    CHECK(eighth.page.find("auton &#39;r8&#39;: hardStopAt 61.00 s") != std::string::npos);
    const Ran ninth = run({"r9", "--robot", "nine"});
    CHECK(ninth.code == 2);
    CHECK_FALSE(ninth.pageWritten);
    CHECK(ninth.err.find("no auton named 'r9' \xe2\x80\x94 and 1 more autons were refused than the registry keeps the "
                         "names of; fix the refusals listed first") != std::string::npos);
}

// RP33 — the PLACEHOLDER note: every claw step carries "PLACEHOLDER number ... (HA-185)"; no pivot step
// does; and the preview's table of provisional numbers is EXACTLY the robot file's `// PROVISIONAL
// (HA-...)` comments, read out of src/robots/mechanism_table.hpp by the declaration they sit in — so the
// table cannot drift from the file.
TEST_CASE("route preview RP33: placeholder numbers are hatched, and the table equals the robot file's comments") {
    const rp::Model m = preview(kTwoFixtures, "seven");
    int claws = 0;
    for (const rp::Step& s : m.steps) {
        if (s.what == "claw") {
            ++claws;
            CHECK(s.placeholder == "PLACEHOLDER number \xe2\x80\x94 this time is the declaration's 0.5 s (HA-185); "
                                   "Station J times the real one");
        } else {
            CHECK(s.placeholder.empty());
        }
    }
    CHECK(claws == 2);
    // The robot file's PROVISIONAL comments, by declaration name, inside the claw robot's region (the old robot
    // two's, kept as a fixture since ROB1-BT).
    const std::string file = source_pins::readFile("test/mechanism_fixtures.hpp");
    const std::size_t from = file.find("\n// [robot two mechanisms]\n");
    const std::size_t to = file.find("\n// [/robot two mechanisms]\n");
    REQUIRE(from != std::string::npos);
    REQUIRE(to != std::string::npos);
    const std::string region = file.substr(from, to - from);
    std::map<std::string, std::string> inFile;
    for (std::size_t at = region.find("PROVISIONAL (HA-"); at != std::string::npos;
         at = region.find("PROVISIONAL (HA-", at + 1)) {
        const std::size_t nameAt = region.rfind(".name = \"", at);
        REQUIRE(nameAt != std::string::npos);
        const std::size_t q = nameAt + 9;
        const std::string name = region.substr(q, region.find('"', q) - q);
        const std::size_t ha = at + std::strlen("PROVISIONAL (");
        inFile[name] = region.substr(ha, region.find(')', ha) - ha);
    }
    std::map<std::string, std::string> inTool;
    for (const rp::Provisional& p : rp::kRobotTwoProvisional) {
        inTool[p.mechanism] = p.ha;
    }
    CHECK(inFile == inTool);
    CHECK(inFile.size() == 1);
}

// RP33b (ROB1-BT) — robot two's preview hatches each piston whose stroke time is OWED in its facts file, with
// that number. Bug caught: robot two (every fact owed) drawn as if measured — the brief's landmine 15; a
// number in the table that is not the ledger's; a piston left out.
TEST_CASE("route preview RP33b: robot two's owed piston times are hatched, each the ledger's own number") {
    namespace two = shulib::bench::robot_two_facts;
    struct Owed {
        const char* mechanism;
        const decltype(two::kClawStrokeTime)& fact;
    };
    const Owed owed[] = {{"claw", two::kClawStrokeTime},
                         {"flipper", two::kFlipperStrokeTime},
                         {"ramp", two::kRampStrokeTime}};
    std::map<std::string, std::string> want;
    for (const Owed& o : owed) {
        if (!o.fact.typed) {
            char number[16];
            std::snprintf(number, sizeof number, "%.1f s", o.fact.value().value());
            want[o.mechanism] = number;
        }
    }
    std::map<std::string, std::string> got;
    for (const rp::Provisional& p : rp::kRobotTwo.provisional) {
        got[p.mechanism] = p.number;
    }
    CHECK(got == want);
    CHECK(rp::kRobotTwo.provisional.empty());  // robot one's chosen times, the same build (2026-11-20), as robot one's
}

// RP34 — the RUN-TIME stamp: --stamp and --autons-hash are printed on the page and on stdout; without
// them the page says "commit not stamped". Bug caught: the stamp taken from SHULIB_BUILD_HASH (RM24) —
// set at CONFIGURE time, it can read clean for an edited autons file.
TEST_CASE("route preview RP34: the stamp and the autons hash are printed as given; unstamped says so") {
    const Ran s = run({"seven", "--stamp", "abc1234-dirty", "--autons-hash", "0123456789ab"});
    CHECK(s.page.find("commit abc1234-dirty \xc2\xb7 autons file 0123456789ab") != std::string::npos);
    CHECK(s.out.find("commit abc1234-dirty \xc2\xb7 autons file 0123456789ab") != std::string::npos);
    CHECK(s.page.find("not stamped") == std::string::npos);
    const Ran u = run({"seven"});
    CHECK(u.page.find("commit not stamped") != std::string::npos);
}

// ── RPV1-B1: any robot's own mechanisms ────────────────────────────────────────────────────────────

// RP24 + RP35's second half — robot x's LIFT on FakesByKind moves in its DECLARED time (the trapezoid
// its declaration's starting speed and acceleration predict, computed here from the declaration with
// the library's own conversion only for degrees ↔ inches), SUCCEEDS, and every lift step carries the
// hatched "PLACEHOLDER numbers" note (its numbers are the library's starting values, HA-182). Bug caught:
// a plant not built from the declaration (a slower motor, the wrong gearing) — the time would differ.
TEST_CASE("route preview RP24: a placeholder lift on fakes by kind moves in its declared time, hatched") {
    const rp::Model m = preview(kRobotX, "lift");
    CHECK(m.word == "FINISHED");
    const auto steps = routineSteps(m);
    REQUIRE(steps.size() == 3);
    const shulib::mechanism::MechanismSpec& lift = mech_fixture::robot_x::kRobotXMechanisms.specs[1];
    REQUIRE(std::string{shulib::mechanism::nameOf(lift)} == "lift");
    // The declaration's own profile: Red (100 rpm) direct on a 1.5 in spool, at 75 % of free speed,
    // reached in 0.25 s; 0 -> 22 in, then the 0.1 s "reached" window.
    const double freeInS = 100.0 / 60.0 * 3.14159265358979323846 * 1.5;
    const double v = 0.75 * freeInS;
    const double a = v / 0.25;
    const double d = 22.0;
    const double profile = d >= v * v / a ? d / v + v / a : 2.0 * std::sqrt(d / a);
    for (const std::size_t i : {std::size_t{0}, std::size_t{2}}) {
        CAPTURE(i);
        CHECK(steps[i].what == "lift");
        CHECK(steps[i].verdict == "SUCCEEDED");
        CHECK(steps[i].placeholder.find("PLACEHOLDER numbers") != std::string::npos);
        CHECK(steps[i].placeholder.find("HA-182") != std::string::npos);
        const double took = steps[i].t1 - steps[i].t0;
        CHECK(took >= profile);
        CHECK(took <= profile + 0.35);
    }
    CHECK(steps[1].kind == rp::StepKind::Drive);
    CHECK(steps[1].placeholder.empty());
    CHECK(rp::renderHtml(m).find("hatch") != std::string::npos);
}

// RP26 — --robot picks the robot's OWN list (robot x here; robot one's own is RP25, at the end):
// robot x's page names its five mechanisms, and its lift auton runs; robot two's rig, given a step on
// 'lift', stops there ("is not on this robot") — the list really changed with the flag.
TEST_CASE("route preview RP26: --robot binds that robot's own mechanism list") {
    const Ran x = run({"lift", "--robot", "x"});
    CHECK(x.code == 0);
    CHECK(x.page.find("Previewing ROBOT X&#39;s mechanisms (arm, lift, flywheel, intake, hood) on fakes built from "
                      "their declarations&#39; own numbers") != std::string::npos);
    const comp::AutonRegistry reg = registryOf(kRobotX);
    const rp::Model onTwo = rp::previewEntry(kTwoFixtures, reg, indexOf(reg, "lift"), {});
    CHECK(onTwo.word == "ROUTINE STOPPED");
    bool absent = false;
    for (const rp::Step& s : onTwo.steps) {
        absent = absent || s.sentence.find("'lift' is not on this robot") != std::string::npos;
    }
    CHECK(absent);
}

// RP35 — the rig's default is unchanged (RPV1-B1 changed TesterStack IN PLACE): built with the new
// parameter left at its default, the team's two autons (as they were registered then — MATCH, 27 / 29 s:
// since ROB1-BT the team's file registers them as SKILLS, so RP35 registers that day's two itself, below)
// and the long skills fixture run with the truth
// stream and the sink IDENTICAL to fingerprints taken over the BASE tree before the parameter existed
// (scratch probe base_hash_probe.cpp at d33d25d, RPV1-PROGRESS.md) — the fourth run steps EVERY one of robot
// two's mechanisms, so a default device that changed but is never used cannot hide (the first form of this
// case, three runs that never touch pivot 7, let a default with pivot 7 ABSENT through: mutation RB1, logged).
// Bug caught: any change to the default list, devices or chain.
TEST_CASE("route preview RP35: the tester rig's default runs exactly as before RPV1-B1, bit for bit") {
    struct Want {
        const char* name;
        std::uint64_t truth;
        std::uint64_t sink;
        long paces;
    };
    const Want kWant[] = {{"40in forward", 0xb777dc5fd6086413ULL, 0xa9f6e719ff29c0cdULL, 494},
                          {"TEST 24in turn back", 0x9834ef8786009c0aULL, 0x05467b932ab6683eULL, 858},
                          {"skills long", 0x4182eaa13ccb3663ULL, 0xfdcb43740365e43dULL, 6050},
                          {"every mechanism", 0xad9b850f2b3bc459ULL, 0x633cf69544aee734ULL, 629}};
    // The team's two autons AS THEY WERE (src/team/autons.cpp before ROB1-BT): the 40 in and the acceptance
    // run, each a MATCH entry on 27 / 29 s — the same routine and step names, so the same bytes.
    comp::AutonRegistry team{shulib::bench::kAutonPeriods, comp::OnRefusal::DropEntry};
    team.add(entry("40in forward", comp::AutonKind::Match,
                   [](comp::AutonContext& ctx) {
                       shulib::chassis::Routine r{ctx, "40in-forward"};
                       r.moveTo(shulib::math::Pose2d{shulib::units::Length{40.0}, shulib::units::Length{0.0},
                                                     shulib::math::Angle::degrees(0.0)});
                       return r.result();
                   },
                   comp::StartPose::relative(), 27.0, 29.0));
    team.add(comp::acceptanceRunEntry(comp::AutonKind::Match, shulib::units::Time{27.0}, shulib::units::Time{29.0}));
    comp::AutonRegistry sk{shulib::bench::kAutonPeriods, comp::OnRefusal::DropEntry};
    sk.add(entry("skills long", comp::AutonKind::Skills, &rp_fixture::skillsLong, field(-48, -48, 0), 57.0, 59.0));
    sk.add(entry("every mechanism", comp::AutonKind::Match, &rp_fixture::everyMechanism, comp::StartPose::relative(),
                 27.0, 29.0));
    for (const Want& w : kWant) {
        CAPTURE(w.name);
        const bool fixture = std::string_view{w.name} == "skills long" || std::string_view{w.name} == "every mechanism";
        const comp::AutonRegistry& reg = fixture ? sk : team;
        tester_rig::TesterStack s{rp::standInDrive()};
        // The bound slots themselves: robot two's three, in its order, every one BUILT (the base tree's).
        REQUIRE(s.mechanisms.size() == 3);
        const char* const kNames[3] = {"claw pivot 6", "claw pivot 7", "claw"};
        for (std::size_t i = 0; i < 3; ++i) {
            CHECK(std::string_view{s.mechanisms.slotPtr(i)->name()} == kNames[i]);
            CHECK(s.mechanisms.slotPtr(i)->built());
        }
        rp_hash::TruthTap tap;
        tap.inner = s.probe.inner;
        tap.h = &s.rig.h;
        s.probe.inner = &tap;
        (void)s.run(reg, static_cast<int>(indexOf(reg, w.name)));
        CHECK(tap.paces == w.paces);
        CHECK(tap.fnv.h == w.truth);
        CHECK(rp_hash::sinkHash(s.sink) == w.sink);
    }
}

// FakesByKind's plants are the DECLARATION's: the ArmPlant constants for robot x's lift are its gearing,
// its kV and kA through degrees -> radians, its gravity, its travel as hard stops, its home as the start.
TEST_CASE("route preview B1: FakesByKind builds each plant from the declaration's own numbers") {
    const shulib::mechanism::MechanismSpec& lift = mech_fixture::robot_x::kRobotXMechanisms.specs[1];
    const shulib::sim::ArmPlantConfig c = tester_rig::FakesByKind::armPlantFrom(lift);
    CHECK(c.gearRatio == lift.position.gearRatio);
    CHECK(c.kV == doctest::Approx(lift.position.kV * 180.0 / 3.14159265358979323846));
    CHECK(c.kG == lift.position.kG);
    CHECK(c.constantGravity);
    CHECK(c.minStopDeg == doctest::Approx(0.0));
    CHECK(c.maxStopDeg == doctest::Approx(24.0 / (3.14159265358979323846 * 1.5) * 360.0));
    CHECK(c.startDeg == doctest::Approx(0.0));
    tester_rig::FakesByKind fakes{mech_fixture::robot_x::kRobotXMechanisms};
    CHECK(fakes.arms().size() == 2);  // the arm and the lift
    CHECK(fakes.parts().devices().size() == 5);
    for (const shulib::mechanism::Devices& d : fakes.parts().devices()) {
        CHECK(std::string_view{d.absence}.empty());
    }
}

// ── RPV1-A's review (2026-11-13): each case below closes a finding or a mutation that survived ───────

namespace {

/// A pose as an independent formatter writes it (never the renderer's poseText).
std::string poseOf(const rp::Sample& p) {
    double deg = std::remainder(p.theta * 180.0 / 3.14159265358979323846, 360.0);
    if (std::abs(deg) < 0.05) {
        deg = 0.0;
    }
    char b[96];
    std::snprintf(b, sizeof b, "(%.1f, %.1f) %.1f\xc2\xb0", p.x + 0.0, p.y + 0.0, deg);
    return b;
}
/// `v` to two places, independently of the renderer.
std::string two(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.2f", std::abs(v) < 0.005 ? 0.0 : v);
    return b;
}
/// The SVG group of step `id` in `page` (from its <g ...> to its </g>).
std::string groupOf(const std::string& page, const std::string& id) {
    const std::size_t at = page.find("data-step=\"" + id + "\">");
    const std::size_t g = at == std::string::npos ? std::string::npos : page.rfind("<g ", at);
    if (g == std::string::npos) {
        return "";
    }
    return page.substr(g, page.find("</g>", at) - g);
}
/// A Model of hand-made samples (T+ 0.01 s apart) for THE FIELD CHECK alone.
rp::Model handPath(const std::vector<std::array<double, 3>>& poses) {
    rp::Model m;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        m.samples.push_back(rp::Sample{0.01 * static_cast<double>(i), poses[i][0], poses[i][1], poses[i][2], 0.0, 0.0});
    }
    return m;
}
/// The field model's box named `name` whose centre is nearest (x, y).
const shulib::sim::FieldBox& boxNear(const char* name, double x, double y) {
    const shulib::sim::FieldBox* best = nullptr;
    double bestD = 1e18;
    for (const shulib::sim::FieldBox& b : rp::previewField().boxes()) {
        const double d = std::hypot(0.5 * (b.xMin + b.xMax) - x, 0.5 * (b.yMin + b.yMax) - y);
        if (std::string_view{b.name} == name && d < bestD) {
            best = &b;
            bestD = d;
        }
    }
    REQUIRE(best != nullptr);
    return *best;
}
const rp::Step* stepNamed(const rp::Model& m, std::string_view what, int nth = 0) {
    for (const rp::Step& s : m.steps) {
        if (s.what == what && nth-- == 0) {
            return &s;
        }
    }
    return nullptr;
}

/// An IMU that throws from inside the plant's pace once the auton has run 50 paces: a throw INSIDE a
/// motion (the auton's own code never gets one in there; the world below the recorder can).
struct ThrowingImu final : shulib::sim::DegradationModel {
    const shulib::sequence::RunGuard* guard = nullptr;
    int live = 0;
    [[nodiscard]] shulib::math::Angle imuHeading(shulib::math::Angle truth, shulib::units::Time,
                                                 shulib::sim::Rng&) override {
        if (guard != nullptr && guard->running() && ++live > 50) {
            throw std::runtime_error("the IMU's cable came out");
        }
        return truth;
    }
};

/// A pacer layer put ABOVE the recorder (between the rig's probe and it) that throws once the auton has
/// run 50 paces: a throw from the code above the plant while a motion runs, which never enters the
/// recorder's pace.
struct ThrowingLayer final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    const shulib::sequence::RunGuard* guard = nullptr;
    int live = 0;
    void pace() override {
        if (guard != nullptr && guard->running() && ++live > 50) {
            throw std::runtime_error("a layer above the plant threw");
        }
        inner->pace();
    }
};

/// A pacer layer ABOVE the recorder that throws on the 4th pace of the END ACTION (it watches the sink for
/// the guard's "end-of-run action starting" line): a throw inside the park's own brake.
struct ThrowInParkLayer final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    const shulib::hal::fake::FakeTelemetrySink* sink = nullptr;
    int seen = 0;
    int parkPaces = -1;
    void pace() override {
        for (; sink != nullptr && seen < sink->size(); ++seen) {
            if (sink->at(seen).message.rfind("end-of-run action starting", 0) == 0) {
                parkPaces = 0;
            }
        }
        if (parkPaces >= 0 && ++parkPaces > 3) {
            throw std::runtime_error("a layer threw during the park");
        }
        inner->pace();
    }
};

/// A pacer layer that throws once the robot has STOPPED DRIVING: on the `afterIdle`-th pace with no motion
/// running after one was seen (with `still`, also no mechanism step running), while the guard is live.
/// `above` puts it between the rig's probe and the recorder (its throw never enters the recorder's pace);
/// `below` puts it in the recorder's `inner` slot (its throw comes from inside the recorder's pace).
struct ThrowWhenStoppedLayer final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    const shulib::sequence::RunGuard* guard = nullptr;
    const shulib::motion::MotionScheduler* sched = nullptr;
    const shulib::mechanism::MechanismService* svc = nullptr;
    int afterIdle = 5;
    bool still = false;
    bool sawMotion = false;
    int idle = 0;
    void pace() override {
        if (guard->running()) {
            const bool moving = sched->hasActiveMotion();
            sawMotion = sawMotion || moving;
            bool busy = false;
            for (std::size_t i = 0; still && i < svc->size(); ++i) {
                busy = busy || svc->slotPtr(i)->running();
            }
            if (sawMotion && !moving && !busy && ++idle >= afterIdle) {
                throw std::runtime_error("a layer threw once the robot stopped driving");
            }
        }
        inner->pace();
    }
    void watch(const tester_rig::TesterStack& s) {
        guard = &s.guard;
        sched = &s.chassis.scheduler();
        svc = &s.mechanisms;
    }
    [[nodiscard]] rp::PreviewOptions above() {
        rp::PreviewOptions o;
        o.onBuilt = [this](tester_rig::TesterStack& s, shulib::motion::ITickPacer*&) {
            watch(s);
            inner = s.probe.inner;  // the recorder
            s.probe.inner = this;
        };
        return o;
    }
    [[nodiscard]] rp::PreviewOptions below() {
        rp::PreviewOptions o;
        o.onBuilt = [this](tester_rig::TesterStack& s, shulib::motion::ITickPacer*& recorderInner) {
            watch(s);
            inner = recorderInner;  // the plant's pacer
            recorderInner = this;
        };
        return o;
    }
};

/// The one PROGRAM ERROR notice of `m` ("" when there is none; "TWO" when there are two).
std::string programError(const rp::Model& m) {
    std::string found;
    for (const std::string& n : m.notices) {
        if (n.rfind("PROGRAM ERROR", 0) == 0) {
            found = found.empty() ? n : std::string{"TWO"};
        }
    }
    return found;
}

}  // namespace

// RP36 — the END ACTION is decided by ORDER, not by time. A step that fails AT ONCE stops the routine with
// no time passing, so it begins at the very instant scoring ends; a half-tick window called it the park
// (number 0, "park" in the table, the page and the library's words). Each path a step can come from:
// (a) the MECH line of a mechanism not on this robot, after two drives (RP31's helper) — step 3; (b) the
// same as the auton's FIRST step (robot x's lift auton on robot two) — step 1; (c) a slot's step that is
// refused at once (robot y's never-homed tilt, after its swing's home and move) — step 3; (d) and the
// other side: an entry's OWN end action that commands the claw — that claw and its brake ARE the park.
TEST_CASE("route preview RP36: a step that fails at once keeps its number; only the end action's own steps are the park") {
    const rp::Model h = preview(kTwoFixtures, "helper");
    const rp::Step* intake = stepNamed(h, "intake");
    REQUIRE(intake != nullptr);
    CHECK(intake->verdict == "ABSENT");
    CHECK_FALSE(intake->endAction);
    CHECK(intake->number == 3);  // main's drive 1, the helper's drive 2, its intake 3
    int parks = 0;
    for (const rp::Step& s : h.steps) {
        if (s.endAction) {
            ++parks;
            CHECK(s.kind == rp::StepKind::Brake);
            CHECK(s.number == 0);
        }
    }
    CHECK(parks == 1);
    bool row = false;
    for (const auto& r : rp::tableRows(h)) {
        row = row || (r[0] == "3" && r[1] == "intake" && r[6] == "ABSENT");
    }
    CHECK(row);
    const std::string page = rp::renderHtml(h);
    CHECK(page.find("<tr data-step=\"3\"><td>3</td><td>intake</td>") != std::string::npos);
    CHECK(page.find("step 3: &#39;intake&#39; is not on this robot (its wiring list does not have it): the step did "
                    "nothing") != std::string::npos);
    CHECK(page.find("data-step=\"park\"><td>park</td><td>intake") == std::string::npos);
    CHECK(rp::renderText(h).find("step 3: 'intake' is not on this robot") != std::string::npos);

    const comp::AutonRegistry xReg = registryOf(kRobotX);
    const rp::Model onTwo = rp::previewEntry(kTwoFixtures, xReg, indexOf(xReg, "lift"), {});
    REQUIRE(!onTwo.steps.empty());
    CHECK(onTwo.steps[0].what == "lift");
    CHECK(onTwo.steps[0].verdict == "ABSENT");
    CHECK_FALSE(onTwo.steps[0].endAction);
    CHECK(onTwo.steps[0].number == 1);

    const rp::Model y = preview(kRobotY, "y");
    CHECK(y.word == "ROUTINE STOPPED");
    const rp::Step* tilt = stepNamed(y, "tilt");
    REQUIRE(tilt != nullptr);
    CHECK(tilt->verdict == "NOT_HOMED");
    CHECK_FALSE(tilt->endAction);
    CHECK(tilt->number == 3);

    const rp::Model o = preview(kTwoReview, "own park");
    CHECK(o.word == "FINISHED");
    const rp::Step* open = stepNamed(o, "claw", 0);
    const rp::Step* closed = stepNamed(o, "claw", 1);
    REQUIRE(open != nullptr);
    REQUIRE(closed != nullptr);
    CHECK_FALSE(open->endAction);
    CHECK(open->number == 2);
    CHECK(closed->endAction);
    CHECK(closed->number == 0);
    int ownParks = 0;
    for (const rp::Step& s : o.steps) {
        ownParks += s.endAction ? 1 : 0;
    }
    CHECK(ownParks == 2);  // the end action's claw and its brake
}

// RP9b — WHERE a run threw. (a) A drive the deadline cut, then the auton's own code throws once it sees the
// cut: AFTER step 1, never "inside" it (the cut drive is CANCELLED, drawn amber as cut). (b) A throw from
// INSIDE a pace while a 40 in drive is open (an IMU that throws, below the recorder): INSIDE step 1, the
// drive cut short where it was. (c) The same from ABOVE the recorder: still inside step 1. (d) A throw during
// the end action's brake: inside the park. The verification round's: (e), (f) a BLOCKING mechanism step,
// thrown into from above and from below: inside it; (g) a pause, from below: "during a wait", from above:
// "after"; (h) a mechanism the robot drove during, still running: never "inside" it; (i) a blocking step
// while another mechanism runs (no drive): not in the background, inside it.
TEST_CASE("route preview RP9b: a throw after a cut step is AFTER it; a throw inside a drive is INSIDE it") {
    const rp::Model a = preview(kTwoReview, "throw after cut");
    CHECK(a.threw);
    CHECK(a.why == "the auton threw after the cut");
    const auto as = routineSteps(a);
    REQUIRE(as.size() == 1);
    CHECK(as[0].verdict == "CANCELLED");
    CHECK(as[0].mark == rp::Mark::Cut);
    bool after = false;
    for (const std::string& n : a.notices) {
        after = after || n == "PROGRAM ERROR after step 1 (counted from below)";
        CHECK(n.find("inside") == std::string::npos);
    }
    CHECK(after);

    ThrowingImu imu;
    rp::PreviewOptions opt;
    opt.degradation = &imu;
    opt.onBuilt = [&imu](tester_rig::TesterStack& s, shulib::motion::ITickPacer*&) { imu.guard = &s.guard; };
    const rp::Model b = preview(kTwoReview, "forty", opt);
    CHECK(b.threw);
    CHECK(b.why == "the IMU's cable came out");
    bool inside = false;
    for (const std::string& n : b.notices) {
        inside = inside || n == "PROGRAM ERROR inside step 1 (counted from below)";
    }
    CHECK(inside);
    REQUIRE(!b.samples.empty());
    CHECK(b.samples.back().x > 0.5);
    CHECK(b.samples.back().x < 30.0);  // cut short: the drive was 40 in

    // (c) A throw from ABOVE the recorder (a pacer layer between the rig's probe and it) while the same
    // drive runs: it never enters the recorder's pace, and it is still INSIDE step 1 — the drive's
    // boundary came while the exception unwound.
    ThrowingLayer layer;
    rp::PreviewOptions above;
    above.onBuilt = [&layer](tester_rig::TesterStack& s, shulib::motion::ITickPacer*&) {
        layer.guard = &s.guard;
        layer.inner = s.probe.inner;  // the recorder
        s.probe.inner = &layer;
    };
    const rp::Model c = preview(kTwoReview, "forty", above);
    CHECK(c.threw);
    CHECK(c.why == "a layer above the plant threw");
    const auto cs = routineSteps(c);
    REQUIRE(cs.size() == 1);
    CHECK(cs[0].openAtThrow);
    CHECK(cs[0].verdict == "CANCELLED");
    bool insideC = false;
    for (const std::string& n : c.notices) {
        insideC = insideC || n == "PROGRAM ERROR inside step 1 (counted from below)";
    }
    CHECK(insideC);
    REQUIRE(!c.samples.empty());
    CHECK(c.samples.back().x > 0.5);
    CHECK(c.samples.back().x < 30.0);
    // ... and the cut drive of (a) was NOT cut by the throw: the deadline cut it first.
    CHECK_FALSE(as[0].openAtThrow);

    // (d) The same during the END ACTION's brake: inside THE PARK, never "inside step N" (the brake is not
    // the routine's) — and the drive, which settled before the park began, was not cut by it.
    ThrowInParkLayer park;
    rp::PreviewOptions inPark;
    inPark.onBuilt = [&park](tester_rig::TesterStack& s, shulib::motion::ITickPacer*&) {
        park.sink = &s.sink;
        park.inner = s.probe.inner;  // the recorder
        s.probe.inner = &park;
    };
    const rp::Model d = preview(kTwoReview, "forty", inPark);
    CHECK(d.threw);
    CHECK(d.why == "a layer threw during the park");
    const rp::Step* brake = nullptr;
    for (const rp::Step& s : d.steps) {
        brake = s.endAction && s.kind == rp::StepKind::Brake ? &s : brake;
    }
    REQUIRE(brake != nullptr);
    CHECK(brake->openAtThrow);
    const auto ds = routineSteps(d);
    REQUIRE(ds.size() == 1);
    CHECK(ds[0].verdict == "SETTLED");
    CHECK_FALSE(ds[0].openAtThrow);
    bool insidePark = false;
    for (const std::string& n : d.notices) {
        insidePark = insidePark || n == "PROGRAM ERROR inside the park (the end action)";
    }
    CHECK(insidePark);

    // (e), (f) A BLOCKING mechanism step that is running when the run throws — from ABOVE the recorder and
    // from BELOW it — is the step it threw inside: pivot 6's CANCELLED line comes during the unwind (the
    // guard's cancel-all). Before the verification round both said "after step 2" or "during a wait".
    for (const bool fromBelow : {false, true}) {
        CAPTURE(fromBelow);
        ThrowWhenStoppedLayer stopper;
        const rp::Model e = preview(kTwoReview, "pivot throw", fromBelow ? stopper.below() : stopper.above());
        CHECK(e.threw);
        CHECK(e.why == "a layer threw once the robot stopped driving");
        CHECK(programError(e) == "PROGRAM ERROR inside step 2 (counted from below)");
        const auto es = routineSteps(e);
        REQUIRE(es.size() == 2);  // the last drive never began
        CHECK(es[0].kind == rp::StepKind::Drive);
        CHECK(es[0].verdict == "SETTLED");
        CHECK_FALSE(es[0].openAtThrow);
        CHECK(es[1].what == "claw pivot 6");
        CHECK(es[1].verdict == "CANCELLED");
        CHECK_FALSE(es[1].background);
        CHECK(es[1].openAtThrow);
        CHECK(es[1].t1 - es[1].t0 < 0.5);  // cut 5 paces in, not at its 3 s
    }

    // (g) A throw in a PAUSE, with no step running, from BELOW the recorder (inside its pace): "during a
    // wait" — and the claw, which SUCCEEDED before the pause, was not where it threw. From ABOVE, the
    // throw never enters a pace: "after step 2" (header: WHERE A RUN THREW).
    {
        ThrowWhenStoppedLayer stopper;
        stopper.still = true;
        const rp::Model g = preview(kTwoReview, "claw pause", stopper.below());
        CHECK(g.threw);
        CHECK(programError(g) == "PROGRAM ERROR during a wait, after step 2 (counted from below)");
        const auto gs = routineSteps(g);
        REQUIRE(gs.size() == 2);
        CHECK(gs[1].what == "claw");
        CHECK(gs[1].verdict == "SUCCEEDED");
        CHECK_FALSE(gs[1].openAtThrow);
        CHECK_FALSE(gs[0].openAtThrow);
    }
    {
        ThrowWhenStoppedLayer stopper;
        stopper.still = true;
        const rp::Model g = preview(kTwoReview, "claw pause", stopper.above());
        CHECK(g.threw);
        CHECK(programError(g) == "PROGRAM ERROR after step 2 (counted from below)");
    }

    // (h) A mechanism the robot DROVE during (started in the background) is still running when a pause
    // throws: the unwind ends it, but the code had moved on past it — "during a wait, after step 2", never
    // inside pivot 7.
    {
        ThrowWhenStoppedLayer stopper;
        const rp::Model h = preview(kTwoReview, "bg pause", stopper.below());
        CHECK(h.threw);
        CHECK(programError(h) == "PROGRAM ERROR during a wait, after step 2 (counted from below)");
        const auto hs = routineSteps(h);
        REQUIRE(hs.size() == 2);
        CHECK(hs[0].what == "claw pivot 7");
        CHECK(hs[0].verdict == "CANCELLED");
        CHECK(hs[0].background);
        CHECK_FALSE(hs[0].openAtThrow);
        CHECK(hs[1].kind == rp::StepKind::Drive);
    }

    // (i) A blocking pivot 6 while pivot 7 (started earlier) still runs, with no drive during either: pivot 6
    // is NOT in the background (the robot did not drive — another mechanism running is not driving), and
    // the throw is inside it, step 3. Pivot 7, started with no drive during it, also reads as ended inside
    // (the header's limit: from below that is a blocking step); the LATER-begun of the two is where it threw.
    {
        ThrowWhenStoppedLayer stopper;
        stopper.afterIdle = 30;  // past the 0.2 s pause: 0.1 s into pivot 6
        const rp::Model i = preview(kTwoReview, "bg blocking", stopper.below());
        CHECK(i.threw);
        CHECK(programError(i) == "PROGRAM ERROR inside step 3 (counted from below)");
        const auto is = routineSteps(i);
        REQUIRE(is.size() == 3);
        CHECK(is[1].what == "claw pivot 7");
        CHECK(is[2].what == "claw pivot 6");
        CHECK_FALSE(is[1].background);
        CHECK_FALSE(is[2].background);
        CHECK(is[1].openAtThrow);
        CHECK(is[2].openAtThrow);
        CHECK(is[2].verdict == "CANCELLED");
        REQUIRE(rp::tableRows(i).size() == i.steps.size() + 1);  // a header row, then one per step
        for (const auto& row : rp::tableRows(i)) {
            for (const std::string& cell : row) {
                CHECK(cell.find("(in the background)") == std::string::npos);
            }
        }
    }
}

// RP37 — every flag takes its value after '=' too (the guide writes `--place=-48,-48,90`; PROGRESS Δ10 says
// the program takes either), and gives the same run; --place refuses junk, a NaN or an infinity (nothing
// drawn); a FIELD auton given --place says it is ignored and is drawn at its FIELD start.
TEST_CASE("route preview RP37: flags take '=' values, --place refuses junk, a FIELD auton ignores --place aloud") {
    const Ran glued = run({"forty", "--robot=review", "--place=-48,-48,90"});
    const Ran spaced = run({"forty", "--robot", "review", "--place", "-48,-48,90"}, kFixtureRobots, "page2.html");
    CHECK(glued.code == 0);
    CHECK(spaced.code == 0);
    CHECK(glued.page.find("RELATIVE, placed with --place at (-48.0, -48.0) 90.0") != std::string::npos);
    CHECK(glued.out.substr(0, glued.out.find("page: ")) == spaced.out.substr(0, spaced.out.find("page: ")));
    const std::string gluedOut = scratchDir() + "/glued-out.html";
    std::filesystem::remove(gluedOut);
    const Ran rx = run({"lift", "--robot=x", "--out=" + gluedOut}, kFixtureRobots, "unused.html", true);
    CHECK(rx.code == 0);
    CHECK(std::filesystem::exists(gluedOut));
    CHECK(rx.out.find("Previewing ROBOT X's mechanisms") != std::string::npos);
    CHECK(run({"--list=yes"}, kFixtureRobots, "page.html", true).code == 2);
    for (const char* bad : {"1,2,3x", "nan,0,0", "0,inf,0", "1,2", "a,b,c", ""}) {
        CAPTURE(bad);
        const Ran r = run({"forty", "--robot", "review", "--place", bad});
        CHECK(r.code == 2);
        CHECK_FALSE(r.pageWritten);
        CHECK(r.err.find("--place takes X,Y,DEG") != std::string::npos);
    }
    const Ran f = run({"seven", "--place", "0,0,0"});
    CHECK(f.code == 0);
    CHECK(f.out.find("!! --place is ignored: this auton declares a FIELD start, and it is drawn there.") !=
          std::string::npos);
    CHECK(f.page.find("started at its FIELD start (-48.0, -48.0) 0.0") != std::string::npos);
}

// RP38 — the tool's OWN failure is exit 3, never 2 (2 is a usage error: a script must tell a typo from a
// broken tool): a page that cannot be written, and a preview that throws (a robot handed to it with no
// mechanism list — runPreview throws, runProgram says so). And a run KILLED from outside leaves no page that
// looks like its own: the old page is removed before the run (the silent loop, ended by its alarm, in a
// forked child bounded like RP27's). And the no-pace stop's page that cannot be written is 3 too, never
// the drawn loop's 1 (the verification round; a forked child like RP27's).
TEST_CASE("route preview RP38: the tool's own failure is exit 3; a killed run leaves no old page behind") {
    const std::string notADir = scratchDir() + "/not-a-dir";
    {
        std::ofstream f{notADir};
        f << "a file, not a directory";
    }
    {
        const std::string under = notADir + "/page.html";
        const std::string_view args[] = {"seven", "--out", under};
        std::ostringstream out;
        std::ostringstream err;
        CHECK(rp::runPreview(args, out, err, kFixtureRobots) == rp::kExitToolFailed);
        CHECK(err.str().find("could not write the page") != std::string::npos);
    }
    const rp::PreviewRobot broken{
        .key = "broken",
        .title = "robot broken",
        .table = nullptr,
        .registerAutons = &registerXFixtures,
        .autonsFile = "a robot with no list",
        .defaultRig = false,
        .provisional = {},
    };
    const rp::PreviewRobot brokenOnly[] = {broken};
    {
        const std::string out = scratchDir() + "/broken.html";
        const std::string_view args[] = {"lift", "--out", out};
        std::ostringstream o;
        std::ostringstream e;
        CHECK_THROWS_AS((void)rp::runPreview(args, o, e, brokenOnly), shulib::PreconditionError);
        std::ostringstream o2;
        std::ostringstream e2;
        CHECK(rp::runProgram(args, o2, e2, brokenOnly) == rp::kExitToolFailed);
        CHECK(e2.str().find("route preview: the preview itself failed: route preview: a robot with no mechanism list") !=
              std::string::npos);
        CHECK(rp::kExitToolFailed != rp::kExitUsage);
        rp::PreviewRobot noAutons = kRobotX;
        noAutons.registerAutons = nullptr;
        const rp::PreviewRobot noAutonsOnly[] = {noAutons};
        std::ostringstream o3;
        std::ostringstream e3;
        CHECK(rp::runProgram(args, o3, e3, noAutonsOnly) == rp::kExitToolFailed);
        CHECK(e3.str().find("a robot with no autons file") != std::string::npos);
    }
    // The killed run.
    const std::string path = scratchDir() + "/silent.html";
    {
        std::ofstream f{path};
        f << "THE LAST RUN'S PAGE";
    }
    std::fflush(nullptr);
    const pid_t child = fork();
    REQUIRE(child >= 0);
    if (child == 0) {
        std::signal(SIGALRM, SIG_DFL);
        const rlimit as{2ULL << 30, 2ULL << 30};
        setrlimit(RLIMIT_AS, &as);
        alarm(1);
        try {
            const std::string_view args[] = {"silent", "--robot", "review", "--out", path};
            std::ostringstream out;
            std::ostringstream err;
            (void)rp::runPreview(args, out, err, kFixtureRobots);
            std::_Exit(78);  // the silent loop never returns: never here
        } catch (...) {
            std::_Exit(79);
        }
    }
    int status = 0;
    REQUIRE(waitpid(child, &status, 0) == child);
    REQUIRE(WIFSIGNALED(status));
    CHECK(WTERMSIG(status) == SIGALRM);
    CHECK_FALSE(std::filesystem::exists(path));
    CHECK_FALSE(std::filesystem::exists(path + ".part"));

    // The NO-PACE STOP whose page cannot be written (the verification round): RP27's looping auton with its
    // page under a FILE exits 3 — the tool's failure — never the drawn loop's 1, and says so on stderr (the
    // child's stderr goes to a file read here; the stop ends the process, so nothing else can carry it).
    const std::string loopErr = scratchDir() + "/forever-unwritable.err";
    std::filesystem::remove(loopErr);
    std::fflush(nullptr);
    const pid_t looper = fork();
    REQUIRE(looper >= 0);
    if (looper == 0) {
        const rlimit as{2ULL << 30, 2ULL << 30};
        setrlimit(RLIMIT_AS, &as);
        alarm(60);
        std::FILE* e = std::freopen(loopErr.c_str(), "w", stderr);
        if (e == nullptr) {
            std::_Exit(80);
        }
        try {
            const std::string under = notADir + "/forever.html";
            const std::string_view args[] = {"forever", "--out", under};
            std::ostringstream out;
            const int code = rp::runPreview(args, out, std::cerr, kFixtureRobots);
            std::_Exit(code == 3 ? 77 : 78);  // returned instead of the stop: never 3
        } catch (...) {
            std::_Exit(79);
        }
    }
    int loopStatus = 0;
    REQUIRE(waitpid(looper, &loopStatus, 0) == looper);
    REQUIRE(WIFEXITED(loopStatus));
    CHECK(WEXITSTATUS(loopStatus) == rp::kExitToolFailed);
    CHECK_FALSE(std::filesystem::exists(notADir + "/forever.html"));
    std::ifstream ef{loopErr};
    REQUIRE(ef);
    const std::string loopErrText{std::istreambuf_iterator<char>{ef}, std::istreambuf_iterator<char>{}};
    CHECK(loopErrText.find("route preview: could not write the page to " + notADir + "/forever.html") !=
          std::string::npos);
}

// RP39 — the page and the table SAY what the Model holds. RP16 holds the two tables equal, but both come from
// one tableRows(): a wrong cell would be wrong in both. So here each thing is checked against a value
// computed INDEPENDENTLY: a row's cells (an own formatter), the failed step's colour in its own SVG group, a
// step's number where it began (its pixel, from the transform written out here) with its times, the outlines,
// the timeline's park and stop lines (720 px over the period from x = 10), the summary's period, the
// THROUGH and TOUCH rings, the faded part from the THROUGH on, and the failed mechanism's sentence on stdout.
TEST_CASE("route preview RP39: the page and the table say what the Model holds, cell by cell and mark by mark") {
    const rp::Model m = preview(kTwoFixtures, "seven");
    const auto rows = rp::tableRows(m);
    REQUIRE(rows.size() == 8);
    const rp::Step& d = m.steps[0];
    const std::vector<std::string> want = {"1", "drive", poseOf(m.samples[d.s0]), poseOf(m.samples[d.s1]),
                                           two(d.t0), two(d.t1), "SETTLED"};
    CHECK(rows[1] == want);
    CHECK(rows[1][2] == "(-48.0, -48.0) 0.0\xc2\xb0");
    CHECK(std::stod(rows[1][4]) < std::stod(rows[1][5]));  // T+ start before T+ end
    CHECK(std::abs(std::stod(rows[1][3].substr(1)) + 24.0) < 0.6);  // it ENDED near x = -24: "to" is the end
    const std::string page = rp::renderHtml(m);
    // Each motion step: an outline where it began and one where it ended.
    std::size_t motions = 0;
    for (const rp::Step& s : m.steps) {
        motions += s.kind != rp::StepKind::Mechanism ? 1U : 0U;
    }
    std::size_t starts = 0;
    std::size_t ends = 0;
    for (std::size_t at = page.find("<polygon class=\"outline"); at != std::string::npos;
         at = page.find("<polygon class=\"outline", at + 1)) {
        (page.compare(at, 28, "<polygon class=\"outline end\"") == 0 ? ends : starts) += 1;
    }
    CHECK(starts == motions);
    CHECK(ends == motions);
    // Step 1's number where it began (+6, -6 px from the sample), its times in the tooltip.
    const rp::Sample& s0 = m.samples[d.s0];
    char num[96];
    std::snprintf(num, sizeof num, "<text class=\"num\" x=\"%.2f\" y=\"%.2f\">1<title>step 1 drive: T+%s ",
                  (s0.x + 74.0) * 5.0 + 6.0, (74.0 - s0.y) * 5.0 - 6.0, two(d.t0).c_str());
    CHECK(page.find(num) != std::string::npos);
    CHECK(page.find("T+" + two(d.t1) + " s, SETTLED") != std::string::npos);
    // The timeline: the axis over the period, park at 27 s and the hard stop at 29 s of 30.
    CHECK(page.find("<svg id=\"timeline\"") != std::string::npos);
    CHECK(page.find("<line class=\"park\" x1=\"658.0\"") != std::string::npos);      // 10 + 27/30 * 720
    CHECK(page.find("<line class=\"hardstop\" x1=\"706.0\"") != std::string::npos);  // 10 + 29/30 * 720
    CHECK(page.find(">30 s</text>") != std::string::npos);
    CHECK(page.find("scoring ended T+" + two(m.scoringEnded) + " s of the 30 s period") != std::string::npos);
    CHECK(rp::renderText(m).find("scoring ended T+" + two(m.scoringEnded) + " s of the 30 s period") !=
          std::string::npos);

    // A failed step is red ON THE FIELD, in its own group; an ok one green in its.
    const std::string t = rp::renderHtml(preview(kTwoFixtures, "times out"));
    CHECK(groupOf(t, "3").find("<polyline class=\"seg failed\"") != std::string::npos);
    CHECK(groupOf(t, "1").find("<polyline class=\"seg ok\"") != std::string::npos);
    CHECK(groupOf(t, "1").find("seg failed") == std::string::npos);

    // THROUGH: the red ring at the THROUGH sample; the faded part starts THERE and runs to the end.
    const rp::Model far = preview(kTwoFixtures, "far");
    REQUIRE(far.through);
    const std::string fp = rp::renderHtml(far);
    const rp::Sample& at = far.samples[far.throughAt];
    char ring[96];
    std::snprintf(ring, sizeof ring, "<circle class=\"through\" cx=\"%.2f\" cy=\"%.2f\"", (at.x + 74.0) * 5.0,
                  (74.0 - at.y) * 5.0);
    CHECK(fp.find(ring) != std::string::npos);
    const std::string fadedKey = "<polyline class=\"faded\" points=\"";
    const std::size_t fa = fp.find(fadedKey);
    REQUIRE(fa != std::string::npos);
    const std::string fadedPts = fp.substr(fa + fadedKey.size(), fp.find('"', fa + fadedKey.size()) - fa - fadedKey.size());
    char firstPt[64];
    std::snprintf(firstPt, sizeof firstPt, "%.2f,%.2f ", (at.x + 74.0) * 5.0, (74.0 - at.y) * 5.0);
    CHECK(fadedPts.rfind(firstPt, 0) == 0);
    CHECK(static_cast<std::size_t>(std::count(fadedPts.begin(), fadedPts.end(), ' ')) + 1 ==
          far.samples.size() - far.throughAt);
    // TOUCH: the amber ring.
    CHECK(rp::renderHtml(preview(kTwoFixtures, "loader")).find("<circle class=\"touch\"") != std::string::npos);

    // A failed mechanism step's own sentence on stdout (robot x's intake, TIMED_OUT in the background).
    rp::PreviewOptions opt;
    opt.place = true;
    const rp::Model x = preview(kRobotX, "intake bg", opt);
    const auto xs = routineSteps(x);
    REQUIRE(!xs.empty());
    REQUIRE(!xs[0].sentence.empty());
    CHECK(rp::renderText(x).find("step 1: " + xs[0].sentence) != std::string::npos);
    // ... and its span's two marks: the diamond "started", the dot where its verdict arrived.
    const std::string xp = rp::renderHtml(x);
    CHECK(xp.find("step 1 intake: started here in the background, T+0.00 s; its verdict TIMED_OUT arrived at T+") !=
          std::string::npos);
    CHECK(xp.find("<circle class=\"mechend failed\"") != std::string::npos);
}

// RP40 — THE FIELD CHECK, case by case on hand-made paths (no run): every wall at its INNER face (a corner
// 0.5 in past it a TOUCH, 1.5 in past a THROUGH — the left and blue walls too, which no route case
// approaches); the equality edge exactly (a corner EXACTLY 1 in past is a TOUCH, the next double past it a
// THROUGH); a square turned 45 degrees whose BOUNDING BOX overlaps a Loader but whose square does not — no
// note at all (the outline's own axes, not only the world's); and the start pieces — one GREY note per piece
// for the whole run, even driven over twice.
TEST_CASE("route preview RP40: the field check, case by case on hand-made paths") {
    const double face = shulib::sim::FieldModel::kInnerFace;
    struct Wall {
        const char* name;
        double x;
        double y;
        double nx;  // the outward normal
        double ny;
    };
    const Wall walls[] = {{"left wall", -1.0, 45.0, -1.0, 0.0}, {"right wall", 1.0, 45.0, 1.0, 0.0},
                          {"red wall", 40.0, -1.0, 0.0, -1.0}, {"blue wall", 40.0, 1.0, 0.0, 1.0}};
    for (const Wall& w : walls) {
        for (const double past : {0.5, 1.5}) {
            CAPTURE(w.name);
            CAPTURE(past);
            const double cx = w.nx != 0.0 ? w.nx * (face - 7.5 + past) : w.x;
            const double cy = w.ny != 0.0 ? w.ny * (face - 7.5 + past) : w.y;
            rp::Model m = handPath({{0.0, 0.0, 0.0}, {cx, cy, 0.0}});
            rp::fieldCheck(m);
            REQUIRE(m.field.size() == 1);
            CHECK(m.field[0].element == w.name);
            CHECK(m.field[0].kind == (past > 1.0 ? rp::Contact::Through : rp::Contact::Touch));
            CHECK(m.field[0].depth == doctest::Approx(past));
            CHECK(m.through == (past > 1.0));
        }
    }
    // The equality edge, landed on EXACTLY.
    const shulib::sim::FieldBox& right = boxNear("right wall", 71.0, 0.0);
    double x = right.xMin + 1.0 - 7.5;
    for (int i = 0; i < 1000 && rp::wallPenetration(rp::outlineAt(x, 45.0, 0.0), right) != 1.0; ++i) {
        x = rp::wallPenetration(rp::outlineAt(x, 45.0, 0.0), right) < 1.0 ? std::nextafter(x, 1e9)
                                                                           : std::nextafter(x, -1e9);
    }
    REQUIRE(rp::wallPenetration(rp::outlineAt(x, 45.0, 0.0), right) == 1.0);
    rp::Model edge = handPath({{0.0, 45.0, 0.0}, {x, 45.0, 0.0}});
    rp::fieldCheck(edge);
    REQUIRE(edge.field.size() == 1);
    CHECK(edge.field[0].kind == rp::Contact::Touch);
    CHECK(edge.field[0].depth == 1.0);
    double over = std::nextafter(x, 1e9);
    while (rp::wallPenetration(rp::outlineAt(over, 45.0, 0.0), right) <= 1.0) {
        over = std::nextafter(over, 1e9);
    }
    rp::Model past = handPath({{0.0, 45.0, 0.0}, {over, 45.0, 0.0}});
    rp::fieldCheck(past);
    REQUIRE(past.field.size() == 1);
    CHECK(past.field[0].kind == rp::Contact::Through);
    // A 45-degree square near the red-left Loader: its bounding box overlaps the Loader, its square does not.
    const shulib::sim::FieldBox& loader = boxNear("Loader", -58.0, -68.0);
    const double h = 7.5 * std::sqrt(2.0);
    const double lcx = -50.0;
    const double lcy = -59.0;
    REQUIRE(lcx - h < loader.xMax);                    // the bounding boxes overlap ...
    REQUIRE(lcy - h < loader.yMax);
    REQUIRE(lcx + lcy - h > loader.xMax + loader.yMax);  // ... the square's edge clears the Loader's corner
    rp::Model turned = handPath({{-40.0, -50.0, 0.7853981633974483}, {lcx, lcy, 0.7853981633974483}});
    rp::fieldCheck(turned);
    CHECK(turned.field.empty());
    CHECK_FALSE(turned.through);
    // The start pieces along the red wall: there and back again, 0.1 in inside the wall's face.
    const double yHug = -face + 7.5 + 0.1;
    std::vector<std::array<double, 3>> poses;
    for (double px = -45.0; px <= -21.0; px += 0.5) {
        poses.push_back({px, yHug, 0.0});
    }
    for (double px = -21.0; px >= -45.0; px -= 0.5) {
        poses.push_back({px, yHug, 0.0});
    }
    rp::Model pieces = handPath(poses);
    rp::fieldCheck(pieces);
    int cups = 0;
    int pins = 0;
    for (const rp::FieldNote& f : pieces.field) {
        CHECK(f.kind == rp::Contact::Piece);
        cups += f.element == "start Cup" ? 1 : 0;
        pins += f.element == "start Pin" ? 1 : 0;
    }
    CHECK(cups == 2);
    CHECK(pins == 1);
    CHECK(pieces.field.size() == 3);
    const std::string page = rp::renderHtml(pieces);
    CHECK(page.find("drives over a start Cup at T+") != std::string::npos);
}

// RP41 — the EQUAL edge of "the robot drove while it ran": a claw straight after a drive (no pause) begins
// on the very pace the drive ended. That is not "in the background" (the drive is over), and it is drawn as
// a plain mark, never a span. The case lands on the edge exactly: drive.s1 == claw.s0.
TEST_CASE("route preview RP41: a claw straight after a drive is not in the background (the equal edge, landed on)") {
    const rp::Model m = preview(kTwoReview, "drive claw");
    const auto steps = routineSteps(m);
    REQUIRE(steps.size() == 2);
    CHECK(steps[0].kind == rp::StepKind::Drive);
    CHECK(steps[1].what == "claw");
    REQUIRE(steps[0].s1 == steps[1].s0);
    CHECK_FALSE(steps[1].background);
    const auto rows = rp::tableRows(m);
    CHECK(rows[2][6].find("(in the background)") == std::string::npos);
    CHECK(rp::renderHtml(m).find("class=\"span ") == std::string::npos);
}

// RP42 — a CURVE (CURVE1-A) is ONE step, "curve", numbered like a drive, drawn from the plant's own samples
// (never a redrawn spline: its seg is exactly the samples between its start and end, through the transform
// written out here), with its end target's ×; and the path really curves (a sample more than 1 in off the
// straight line from its start to its end).
TEST_CASE("route preview RP42: a curve is one step, drawn from the plant's samples, its x at its end target") {
    const rp::Model m = preview(kTwoReview, "curve");
    CHECK(m.word == "FINISHED");
    const auto steps = routineSteps(m);
    REQUIRE(steps.size() == 2);
    const rp::Step& c = steps[0];
    CHECK(c.kind == rp::StepKind::Curve);
    CHECK(rp::stepLabel(c) == "curve");
    CHECK(c.what == "CurveTo");
    CHECK(c.number == 1);
    CHECK(c.verdict == "SETTLED");
    CHECK(steps[1].kind == rp::StepKind::Drive);
    CHECK(steps[1].number == 2);
    int curves = 0;
    for (const rp::Step& s : m.steps) {
        curves += s.kind == rp::StepKind::Curve ? 1 : 0;
    }
    CHECK(curves == 1);
    REQUIRE(c.hasTarget);
    CHECK(c.tx == doctest::Approx(0.0));
    CHECK(c.ty == doctest::Approx(-24.0));
    const std::string page = rp::renderHtml(m);
    const std::string g = groupOf(page, "1");
    std::string pts;
    for (std::size_t i = c.s0; i <= c.s1; ++i) {
        char b[48];
        std::snprintf(b, sizeof b, "%s%.2f,%.2f", i == c.s0 ? "" : " ", (m.samples[i].x + 74.0) * 5.0 + 0.0,
                      (74.0 - m.samples[i].y) * 5.0 + 0.0);
        pts += b;
    }
    CHECK(g.find("<polyline class=\"seg ok\" points=\"" + pts + "\"/>") != std::string::npos);
    char aim[96];
    std::snprintf(aim, sizeof aim, "<text class=\"aim\" x=\"%.2f\" y=\"%.2f\"", (c.tx + 74.0) * 5.0,
                  (74.0 - c.ty) * 5.0 + 4.0);
    CHECK(g.find(aim) != std::string::npos);
    const rp::Sample& a = m.samples[c.s0];
    const rp::Sample& b = m.samples[c.s1];
    double off = 0.0;
    for (std::size_t i = c.s0; i <= c.s1; ++i) {
        const double cross = (b.x - a.x) * (m.samples[i].y - a.y) - (b.y - a.y) * (m.samples[i].x - a.x);
        off = std::max(off, std::abs(cross) / std::hypot(b.x - a.x, b.y - a.y));
    }
    CHECK(off > 1.0);
}

// RP43 — a step the deadline CUT is amber (Cut), never red: the long skills fixture's last steps, cut at its
// park time — a drive cut mid-way, or a claw cut while it moved, whichever the cut lands on.
TEST_CASE("route preview RP43: the step the deadline cuts is drawn as cut, never as failed") {
    const rp::Model m = preview(kTwoFixtures, "skills long");
    CHECK(m.word == "CUT BY DEADLINE");
    int cut = 0;
    for (const rp::Step& s : routineSteps(m)) {
        if (s.verdict == "CANCELLED") {
            ++cut;
            CHECK(s.mark == rp::Mark::Cut);
        }
        CHECK(s.mark != rp::Mark::Failed);
    }
    CHECK(cut >= 1);
}

// RP44 — the PLACEHOLDER rule is ANY starting value, not all: a lift that states only its kP is still on the
// library's starting values elsewhere (hatched); one that states kP, kV, its speed and its tolerance is not.
// A flywheel that states no kA spins up on the host test plant's ASSUMED inertia, and says so.
namespace rp44 {
using namespace shulib::mechanism::words;
using namespace shulib::units::literals;
inline constexpr Lift kpOnly{.name = "lift",
                             .motors = {Forward},
                             .cartridge = Red,
                             .gearing = direct,
                             .shape = spool(1.5_in),
                             .safe = Hold,
                             .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                             .range = {0_in, 24_in},
                             .gravity = constantGravity(1.0_volt),
                             .home = restsAtStop(0_in),
                             .presets = {{"floor", 0_in}, {"high", 22_in}},
                             .tuning = {.kP = 0.4}};
inline constexpr Lift tuned{.name = "lift",
                            .motors = {Forward},
                            .cartridge = Red,
                            .gearing = direct,
                            .shape = spool(1.5_in),
                            .safe = Hold,
                            .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                            .range = {0_in, 24_in},
                            .gravity = constantGravity(1.0_volt),
                            .home = restsAtStop(0_in),
                            .presets = {{"floor", 0_in}, {"high", 22_in}},
                            .tuning = {.kP = 0.4, .kV = 0.03, .maxSpeed = 10.0, .within = 0.3}};
inline constexpr Flywheel tunedWheel{.name = "wheel",
                                     .motors = {Forward},
                                     .cartridge = Blue,
                                     .gearing = gears(72, 12),
                                     .safe = Coast,
                                     .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
                                     .atSpeed = within(60, 0.1_s),
                                     .presets = {{"shoot", 3000}},
                                     .tuning = {.kV = 0.033, .kP = 0.04}};
inline constexpr MechanismTable kKpOnly = mechanisms(wire(kpOnly, motors(2)));
inline constexpr MechanismTable kTuned = mechanisms(wire(tuned, motors(2)));
inline constexpr MechanismTable kTunedWheel = mechanisms(wire(tunedWheel, motors(4)));
}  // namespace rp44

TEST_CASE("route preview RP44: a placeholder is ANY starting value; a flywheel with no kA says its inertia is assumed") {
    rp::PreviewRobot r{.key = "p", .title = "robot p", .table = &rp44::kKpOnly};
    CHECK(rp::placeholderFor(r, "lift").find("PLACEHOLDER numbers") != std::string::npos);
    r.table = &rp44::kTuned;
    CHECK(rp::placeholderFor(r, "lift").empty());
    r.table = &mech_fixture::robot_x::kRobotXMechanisms;
    const std::string fly = rp::placeholderFor(r, "flywheel");
    // Robot x's flywheel states no tuning: on the starting values AND the assumed inertia — both named.
    CHECK(fly == "PLACEHOLDER numbers \xe2\x80\x94 this time is what its declaration predicts, on the library's "
                 "starting values (HA-182), with an ASSUMED 0.2 s wheel time constant (a flywheel's declaration has "
                 "no number for it); 'shulib Tune' sets the real ones on the robot");
    // A flywheel that states its kP and kV is off the starting values, but its inertia is still assumed.
    r.table = &rp44::kTunedWheel;
    const std::string tunedFly = rp::placeholderFor(r, "wheel");
    CHECK(tunedFly.find("with an ASSUMED 0.2 s wheel time constant") != std::string::npos);
    CHECK(tunedFly.find("HA-182") == std::string::npos);
    // On the default rig (robot two's own fakes) no inertia is assumed by the preview.
    r.defaultRig = true;
    CHECK(rp::placeholderFor(r, "wheel").empty());
}

// B1 — robot x's FLYWHEEL on fakes by kind spins up to "shoot" (3000 rpm, within 60 rpm for 0.1 s) and
// SUCCEEDS — never the false TIMED_OUT of a wheel with no inertia (the review's finding: kA 0 is "no
// feedforward", not "no inertia"). Its time is bounded by the assumed time constant alone: a first-order
// wheel on its feedforward reaches 60 rpm of 3000 in τ·ln(3000/60), the proportional term only shortens it.
// Bug caught: a plant weaker than declared (it never reaches 3000), a plant never stepped, no inertia.
TEST_CASE("route preview B1: robot x's flywheel on fakes by kind spins up in the time its assumed inertia allows") {
    rp::PreviewOptions opt;
    opt.place = true;
    const rp::Model m = preview(kRobotX, "fly", opt);
    CHECK(m.word == "FINISHED");
    const auto steps = routineSteps(m);
    REQUIRE(steps.size() == 1);
    CHECK(steps[0].what == "flywheel");
    CHECK(steps[0].verdict == "SUCCEEDED");
    const double took = steps[0].t1 - steps[0].t0;
    CHECK(took >= 0.1);
    CHECK(took <= tester_rig::kAssumedWheelTauS * std::log(3000.0 / 60.0) + 0.1 + 0.05);
    CHECK(steps[0].placeholder.find("ASSUMED 0.2 s wheel time constant") != std::string::npos);
}

// B1 — the plant constants FakesByKind takes from a declaration, every one: an arm whose home is NOT at its
// joint's zero (so the encoders' zero must be the home), its level angle, a STATED kS and kA (used as given),
// and robot x's lift and flywheel with NO kA (the assumed time constants, as kA = τ·kV).
namespace rpb1 {
using namespace shulib::mechanism::words;
using namespace shulib::units::literals;
inline constexpr Arm raised{.name = "raised",
                            .motors = {Forward},
                            .cartridge = Green,
                            .gearing = gears(12, 60),
                            .safe = Hold,
                            .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
                            .range = {30, 150},
                            .gravity = cosine(1.6_volt, 45),
                            .home = restsAtStop(30),
                            .presets = {{"rest", 30}, {"up", 140}},
                            .tuning = {.kS = 0.2, .kA = 0.001}};
inline constexpr MechanismTable kRaised = mechanisms(wire(raised, motors(1)));
}  // namespace rpb1

TEST_CASE("route preview B1: FakesByKind takes every plant constant from the declaration, or says it assumed it") {
    constexpr double kDeg = 180.0 / 3.14159265358979323846;
    const shulib::mechanism::MechanismSpec& a = rpb1::kRaised.specs[0];
    const shulib::sim::ArmPlantConfig c = tester_rig::FakesByKind::armPlantFrom(a);
    CHECK(c.kS == doctest::Approx(0.2));
    CHECK(c.kA == doctest::Approx(0.001 * kDeg));
    CHECK(c.levelDeg == doctest::Approx(45.0));
    CHECK_FALSE(c.constantGravity);
    CHECK(c.minStopDeg == doctest::Approx(30.0));
    CHECK(c.maxStopDeg == doctest::Approx(150.0));
    CHECK(c.startDeg == doctest::Approx(30.0));
    CHECK(c.encoderZeroDeg == doctest::Approx(30.0));  // the encoders read 0 where the program started
    const shulib::mechanism::MechanismSpec& lift = mech_fixture::robot_x::kRobotXMechanisms.specs[1];
    const shulib::sim::ArmPlantConfig l = tester_rig::FakesByKind::armPlantFrom(lift);
    CHECK(lift.position.kA == 0.0);
    CHECK(l.kA == doctest::Approx(tester_rig::kAssumedJointTauS * l.kV));
    const shulib::mechanism::MechanismSpec& fw = mech_fixture::robot_x::kRobotXMechanisms.specs[2];
    const shulib::sim::FlywheelPlantConfig f = tester_rig::FakesByKind::flywheelPlantFrom(fw);
    CHECK(f.gearRatio == doctest::Approx(1.0 / 6.0));
    CHECK(f.kV == fw.speed.kV);
    CHECK(f.kS == fw.speed.kS);
    CHECK(fw.speed.kA == 0.0);
    CHECK(f.kA == doctest::Approx(tester_rig::kAssumedWheelTauS * f.kV));

    // The numbers the words STATE are these constants (the verification round): guide 16 ("Which robot"
    // and the Dashed bullet) and fakes_by_kind.hpp's banner (INERTIA) quote both assumed time constants, so
    // a constant changed without its sentences — or a sentence without its constant — fails HERE, not on a
    // reader (the outline sentence's rule, D9). Each text is read as prose: comment markers dropped,
    // whitespace squeezed, so a reflow is not a failure.
    const auto prose = [](const std::string& text) {
        std::string out;
        std::istringstream in{text};
        std::string line;
        while (std::getline(in, line)) {
            const std::size_t at = line.find_first_not_of(" \t");
            std::string body = at == std::string::npos ? std::string{} : line.substr(at);
            if (body.rfind("//", 0) == 0) {
                body = body.substr(2);
            }
            out += " " + body;
        }
        std::string squeezed;
        for (const char ch : out) {
            const bool space = ch == ' ' || ch == '\t';
            if (!(space && (squeezed.empty() || squeezed.back() == ' '))) {
                squeezed += space ? ' ' : ch;
            }
        }
        return squeezed;
    };
    const auto shortest = [](double v) {  // the shortest text that reads back as v: never a rounded one
        std::array<char, 32> buf{};
        const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
        return std::string(buf.data(), res.ptr);
    };
    const std::string joint = shortest(tester_rig::kAssumedJointTauS);
    const std::string wheel = shortest(tester_rig::kAssumedWheelTauS);
    const std::string guide = prose(source_pins::readFile("docs/guide/16-testing-an-auton-on-the-bench.md"));
    CHECK(guide.find("the fake assumes it (" + joint + " s for an arm or lift, " + wheel + " s for a flywheel)") !=
          std::string::npos);
    CHECK(guide.find("a flywheel spins up on an ASSUMED " + wheel + " s time constant") != std::string::npos);
    const std::string banner = prose(source_pins::readFile("test/fakes_by_kind.hpp"));
    CHECK(banner.find("kAssumedJointTauS (" + joint + " s;") != std::string::npos);
    CHECK(banner.find("kAssumedWheelTauS (" + wheel + " s;") != std::string::npos);
}

// B1 — the shapes robot one's cascade has, before robot one's list lands: robot y's STALL-homed swing homes
// (by its stall signature, on the plant's hard stop) and moves; its linkage lift moves; its three piece
// sensors never see a piece (an optical one, a switch, a distance one); and a cascade-like
// lift declared HERE (stall-homed, MEASURED shape, a current cap — the shape ROB1-A's placeholder declaration
// has) homes and reaches a preset in the time its declared profile predicts.
namespace rpc {
using namespace shulib::mechanism::words;
using namespace shulib::units::literals;
inline constexpr HeightAt kRows[] = {{0_in, 0}, {8_in, 360}, {16_in, 720}, {24_in, 1080}, {40_in, 1800}};
inline constexpr Lift cascade{
    .name = "cascade",
    .motors = {Forward, Reversed},
    .cartridge = Red,
    .gearing = gears(12, 60),
    .shape = measured(kRows),
    .safe = Hold,
    .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.3_s},
    .currentCap = {.amps = 1.5},
    .range = {0_in, 40_in},
    .gravity = constantGravity(1.0_volt),
    .home = homeByStall(0_in, -6_volt, {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.3_s}),
    .presets = {{"floor", 0_in}, {"L1", 8_in}},
    .unhomedVolts = 4_volt,
};
inline constexpr MechanismTable kTable = mechanisms(wire(cascade, motors(2, 3)));
inline shulib::chassis::RoutineResult homeAndLift(comp::AutonContext& ctx) {
    shulib::chassis::Routine r{ctx, "cascade"};
    r.home(cascade, 4_s);
    r.move(cascade["L1"], 8_s);
    return r.result();
}
inline void reg(comp::AutonRegistry& g) {
    g.add(entry("cascade", comp::AutonKind::Match, &homeAndLift, comp::StartPose::relative(), 27, 29));
}
}  // namespace rpc

TEST_CASE("route preview B1: a stall-homed arm, a linkage lift and a measured cascade-like lift run on fakes by kind") {
    rp::PreviewOptions opt;
    opt.place = true;
    const rp::Model y = preview(kRobotY, "y", opt);
    const auto ys = routineSteps(y);
    REQUIRE(ys.size() == 3);
    CHECK(ys[0].what == "swing");
    CHECK(ys[0].verdict == "SUCCEEDED");  // homed by its stall signature
    CHECK(ys[1].verdict == "SUCCEEDED");
    const rp::Model d = preview(kRobotY, "dr4b", opt);
    REQUIRE(routineSteps(d).size() == 1);
    CHECK(routineSteps(d)[0].verdict == "SUCCEEDED");
    // The piece sensors (file banner of test/fakes_by_kind.hpp: they never see a piece): an optical one and
    // a limit switch, each run "until it sees a piece" for 0.3 s, time out; a distance one confirming a
    // grab leaves it UNCONFIRMED. A fake that saw a piece would turn any of the three green.
    const rp::Model p = preview(kRobotY, "pieces", opt);
    const rp::Step* roller = stepNamed(p, "roller");
    const rp::Step* catapult = stepNamed(p, "catapult");
    const rp::Step* grip = stepNamed(p, "grip");
    REQUIRE(roller != nullptr);
    REQUIRE(catapult != nullptr);
    REQUIRE(grip != nullptr);
    CHECK(roller->verdict == "TIMED_OUT");
    CHECK(catapult->verdict == "TIMED_OUT");
    CHECK(grip->verdict == "UNCONFIRMED");
    const rp::PreviewRobot c{.key = "c", .title = "robot c", .table = &rpc::kTable, .registerAutons = &rpc::reg,
                             .autonsFile = "test/route_preview_test.cpp"};
    const rp::Model m = preview(c, "cascade", opt);
    CHECK(m.word == "FINISHED");
    const auto cs = routineSteps(m);
    REQUIRE(cs.size() == 2);
    CHECK(cs[0].verdict == "SUCCEEDED");
    CHECK(cs[1].verdict == "SUCCEEDED");
    // The declared profile: Red (100 rpm) through 12:60 is 120 deg/s at the drum; the measured rows are
    // 45 deg per inch, so 75 % of free speed is 2 in/s, reached in 0.25 s; 0 -> 8 in.
    const double v = 0.75 * 120.0 / 45.0;
    const double acc = v / 0.25;
    const double profile = 8.0 / v + v / acc;
    const double took = cs[1].t1 - cs[1].t0;
    CHECK(took >= profile);
    CHECK(took <= profile + 0.5);
}

// ── RPV1-S1: robot one ─────────────────────────────────────────────────────────────────────────────

namespace rp1 {
using namespace shulib::units::literals;
namespace facts = shulib::bench::robot_one_facts;
namespace mech = shulib::mechanism;
/// Halfway up its travel: inside the range whatever the ledger says (no level assumed — there may be none).
inline constexpr shulib::units::Length kMid = (facts::cascadeBottom() + facts::cascadeTop()) / 2.0;
inline shulib::chassis::RoutineResult homeMoveHome(comp::AutonContext& ctx) {
    shulib::chassis::Routine r{ctx, "cascade"};
    r.home(shulib::bench::robot_one::cascade, 4_s);
    r.move(shulib::bench::robot_one::cascade.to(kMid), 30_s);
    r.home(shulib::bench::robot_one::cascade, 40_s);
    return r.result();
}
inline void reg(comp::AutonRegistry& g) {
    g.add(entry("cascade", comp::AutonKind::Skills, &homeMoveHome, comp::StartPose::relative(), 57, 59));
    g.add(entry("too long", comp::AutonKind::Skills, &homeMoveHome, field(-48, -48, 0), 58, 61));
}
/// Robot one exactly as the program knows it, with this file's autons in place of the team's.
rp::PreviewRobot withAutons(rp::RegisterFn fn) {
    rp::PreviewRobot r = rp::kRobotOne;
    r.registerAutons = fn;
    r.autonsFile = "test/route_preview_test.cpp";
    return r;
}
std::size_t occurrences(const std::string& s, const std::string& what) {
    std::size_t n = 0;
    for (std::size_t p = s.find(what); p != std::string::npos; p = s.find(what, p + what.size())) {
        ++n;
    }
    return n;
}
std::string slurp(const std::filesystem::path& p) {
    std::ifstream f{p, std::ios::binary};
    return {std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
}
/// Runs in `dir` until it goes out of scope (a page with no --out lands under it, never in the repository).
struct InDir {
    std::filesystem::path was = std::filesystem::current_path();
    explicit InDir(const std::filesystem::path& dir) { std::filesystem::current_path(dir); }
    ~InDir() { std::filesystem::current_path(was); }
    InDir(const InDir&) = delete;
    InDir& operator=(const InDir&) = delete;
};
/// Robot one's page and table list exactly the ledger's OWED facts, each once, under the header line.
void checkMarks(const Ran& r) {
    REQUIRE(r.pageWritten);
    std::size_t owed = 0;
    for (const facts::FactSummary& f : facts::kRobotOneFacts) {
        CAPTURE(f.name);
        CHECK((r.page.find("<li>" + std::string{rp::esc(f.name)} + "</li>") != std::string::npos) == !f.typed);
        CHECK((r.out.find("  [PLACEHOLDER] " + std::string{f.name} + "\n") != std::string::npos) == !f.typed);
        owed += f.typed ? 0 : 1;
    }
    CHECK(occurrences(r.out, "\n  [PLACEHOLDER] ") == owed);
    const std::size_t block = r.page.find("<div class=\"owed\"");
    CHECK((block != std::string::npos) == (owed > 0));
    if (block != std::string::npos) {
        CHECK(occurrences(r.page.substr(block, r.page.find("</div>", block) - block), "<li>") == owed);
        const std::string head = "PLACEHOLDER numbers: " + std::to_string(owed) +
                                 " of this robot's facts are still OWED in src/robots/robot_one_facts.hpp";
        CHECK(r.out.find(head) != std::string::npos);
        CHECK(unescape(r.page).find(head) != std::string::npos);
    }
}
}  // namespace rp1

TEST_CASE("route preview RP25: robot one runs its own autons on its own list, every OWED fact marked") {
    const auto robots = std::span<const rp::PreviewRobot>{rp::kRobots};
    CHECK(rp::kRobotOne.ledger.data() == std::data(rp1::facts::kRobotOneFacts));  // the ledger itself, never a copy
    CHECK(rp::kRobotOne.ledger.size() == std::size(rp1::facts::kRobotOneFacts));
    comp::AutonRegistry reg{shulib::bench::kAutonPeriods, comp::OnRefusal::DropEntry};
    shulib::bench::registerRobotOneAutons(reg);  // robot one's file itself, not through the preview
    REQUIRE(reg.size() >= 1);
    std::string names;
    for (std::size_t i = 0; i < shulib::bench::robot_one::kRobotOneMechanisms.count; ++i) {
        names += (i == 0 ? "" : ", ") +
                 std::string{rp1::mech::nameOf(shulib::bench::robot_one::kRobotOneMechanisms.specs[i])};
    }
    const std::string line = "Previewing ROBOT ONE's mechanisms (" + names + ") on fakes built from their declarations";
    for (std::size_t i = 0; i < reg.size(); ++i) {
        const std::string name = reg.at(i).name;
        CAPTURE(name);
        const Ran r = run({name, "--robot", "one", "--place=-48,-48,90"}, robots, "one.html");
        CHECK((r.code == rp::kExitDrawn || r.code == rp::kExitProblem));  // drawn; never usage or the tool's failure
        CHECK(r.out.find(line) != std::string::npos);
        const comp::AutonEntry& e = reg.at(i);  // its kind and schedule: robot two's same-named copies differ
        CHECK(r.out.find("auton '" + name + "' (" + comp::autonKindWord(e.kind) + ", park T+" +
                         rp::fixed(e.endActionAt.value(), 2) + " s, stop T+" + rp::fixed(e.hardStopAt.value(), 2)) !=
              std::string::npos);
        rp1::checkMarks(r);
    }
    const Ran list = run({"--list", "--robot", "one"}, robots, "one.html", true);
    CHECK(list.code == 0);
    CHECK(list.out.find("robot one's autons (src/team/robot_one_autons.cpp):") != std::string::npos);
    for (std::size_t i = 0; i < reg.size(); ++i) {
        CHECK(list.out.find("  '" + std::string{reg.at(i).name} + "'\n") != std::string::npos);
    }
    const Ran unknown = run({"no auton has this name", "--robot", "one"}, robots, "one.html");
    CHECK(unknown.code == 2);
    CHECK_FALSE(unknown.pageWritten);
    CHECK(unknown.err.find("in src/team/robot_one_autons.cpp") != std::string::npos);
    const Ran bad = run({"40in forward", "--robot", "three"}, robots);
    CHECK(bad.code == 2);
    CHECK(bad.err.find("no robot 'three'") != std::string::npos);
    // No --out: the robots share auton names, so robot one's page goes under one/; robot two's is unchanged.
    const std::filesystem::path dir = std::filesystem::path{scratchDir()} / "default-paths";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const rp1::InDir in{dir};
    const Ran two = run({"40in forward"}, robots, "unused.html", true);
    const Ran one = run({"40in forward", "--robot", "one"}, robots, "unused.html", true);
    CHECK(two.code == rp::kExitDrawn);
    CHECK(one.code == rp::kExitDrawn);
    const std::string twoPage = rp1::slurp(dir / "build/route_preview/40in forward.html");
    CHECK(unescape(twoPage).find("ROBOT TWO's mechanisms") != std::string::npos);
    CHECK(unescape(rp1::slurp(dir / "build/route_preview/one/40in forward.html")).find("ROBOT ONE's mechanisms") !=
          std::string::npos);
    // robot two (ROB1-BT): its ledger's OWED facts are its placeholders too, named from its own file
    const auto twoOwed = static_cast<std::size_t>(shulib::bench::robot_two_facts::kRobotTwoOwedCount);
    CHECK(twoOwed > 0);
    CHECK(rp::owedFacts(rp::kRobotTwo).size() == twoOwed);
    CHECK(two.out.find("PLACEHOLDER numbers: " + std::to_string(twoOwed) +
                       " of this robot's facts are still OWED in src/robots/robot_two_facts.hpp") != std::string::npos);
    CHECK(twoPage.find("class=\"owed\"") != std::string::npos);
    const std::size_t stamp = two.out.find("\ncommit ");
    REQUIRE(stamp != std::string::npos);
    CHECK(two.out.compare(two.out.find('\n', stamp + 1), 2, "\n\n") != 0);  // no empty line where the block goes
}

TEST_CASE("route preview RP25b: robot one's cascade homes and moves on fakes by kind, marks on every page") {
    const rp::PreviewRobot one = rp1::withAutons(&rp1::reg);
    const rp::PreviewRobot robots[] = {one};
    const Ran r = run({"cascade", "--robot", "one", "--place=-48,-48,90"}, robots, "one-cascade.html");
    CHECK(r.code == rp::kExitDrawn);
    rp1::checkMarks(r);
    rp::PreviewOptions opt;
    opt.place = true;
    const rp::Model m = preview(one, "cascade", opt);
    CHECK(m.word == "FINISHED");
    const auto cs = routineSteps(m);
    REQUIRE(cs.size() == 3);
    for (const rp::Step& s : cs) {
        CHECK(s.what == "cascade");
        CHECK(s.verdict == "SUCCEEDED");
    }
    const rp1::mech::MechanismTable& t = shulib::bench::robot_one::kRobotOneMechanisms;
    const auto* found = std::find_if(t.specs, t.specs + t.count, [](const rp1::mech::MechanismSpec& s) {
        return std::string_view{rp1::mech::nameOf(s)} == "cascade";
    });
    REQUIRE(found != t.specs + t.count);
    const rp1::mech::MechanismSpec& c = *found;
    // The move: the declared trapezoid from the bottom stop to halfway, plus the "reached" window.
    const double d = rp1::kMid.value() - rp1::facts::cascadeBottom().value();
    const double v = c.position.maxVelocity;
    const double a = c.position.maxAcceleration;
    const double profile = d >= v * v / a ? d / v + v / a : 2.0 * std::sqrt(d / a);
    const double took = cs[1].t1 - cs[1].t0;
    CHECK(took >= profile);
    CHECK(took <= profile + 0.5);
    CHECK(cs[2].t1 - cs[2].t0 > cs[0].t1 - cs[0].t0);  // homed from above: it travels down before it stalls
    // HA-182: every step is a PLACEHOLDER while any of the cascade's numbers is still the library's starting value.
    const rp1::mech::PositionSpec lib = rp1::mech::positionStartingValues(c);
    const bool starting = c.position.kP == lib.kP || c.position.kV == lib.kV ||
                          c.position.maxVelocity == lib.maxVelocity || c.position.tolerance == lib.tolerance;
    for (const rp::Step& s : cs) {
        CHECK((s.placeholder.find("HA-182") != std::string::npos) == starting);
    }
    CHECK(rp1::occurrences(r.out, "SUCCEEDED [PLACEHOLDER]") == (starting ? cs.size() : 0));
    const Ran refused = run({"too long", "--robot", "one"}, robots, "one-refused.html");
    CHECK(refused.code == rp::kExitProblem);
    CHECK(refused.out.find("REFUSED") != std::string::npos);
    rp1::checkMarks(refused);
    const comp::AutonRegistry reg = registryOf(one);
    const rp::Model onTwo = rp::previewEntry(kTwoFixtures, reg, indexOf(reg, "cascade"), opt);
    CHECK(onTwo.word == "ROUTINE STOPPED");
    CHECK(stepNamed(onTwo, "cascade") != nullptr);
    CHECK(stepNamed(onTwo, "cascade")->sentence.find("'cascade' is not on this robot") != std::string::npos);
}

TEST_CASE("route preview RP25c: only the ledger's OWED facts are listed, in its order; none, no block") {
    using shulib::bench::robot_one_facts::FactSummary;
    using shulib::bench::robot_one_facts::Stage;
    const FactSummary mixed[] = {{"typed A", Stage::A, true}, {"owed B", Stage::B, false},
                                 {"typed rec", Stage::Recorded, true}, {"owed rec", Stage::Recorded, false}};
    const FactSummary oneOwed[] = {{"typed A", Stage::A, true}, {"owed only", Stage::B, false}};
    const FactSummary allTyped[] = {{"typed A", Stage::A, true}};
    rp::PreviewRobot r = rp1::withAutons(&rp1::reg);
    r.ledger = rp::Ledger{mixed};
    CHECK(rp::owedFacts(r) == std::vector<std::string>{"owed B", "owed rec"});
    rp::PreviewOptions opt;
    opt.place = true;
    const rp::Model m = preview(r, "cascade", opt);
    const std::string text = rp::renderText(m);
    const std::string page = rp::renderHtml(m);
    CHECK(text.find("typed A") == std::string::npos);
    CHECK(page.find("typed rec") == std::string::npos);
    CHECK(text.find("  [PLACEHOLDER] owed B\n") < text.find("  [PLACEHOLDER] owed rec\n"));
    CHECK(text.find("  [PLACEHOLDER] owed rec\n") != std::string::npos);
    CHECK(page.find("<li>owed B</li>") < page.find("<li>owed rec</li>"));
    CHECK(page.find("<li>owed rec</li>") != std::string::npos);
    CHECK(page.find("2 of this robot&#39;s facts are still OWED") != std::string::npos);
    r.ledger = rp::Ledger{oneOwed};
    const rp::Model single = preview(r, "cascade", opt);
    CHECK(rp::renderText(single).find("PLACEHOLDER numbers: 1 of this robot") != std::string::npos);
    CHECK(rp::renderText(single).find("  [PLACEHOLDER] owed only\n") != std::string::npos);
    CHECK(rp::renderHtml(single).find("<li>owed only</li>") != std::string::npos);
    r.ledger = rp::Ledger{allTyped};
    CHECK(rp::owedFacts(r).empty());
    const rp::Model none = preview(r, "cascade", opt);
    CHECK(rp::renderText(none).find("PLACEHOLDER numbers:") == std::string::npos);
    CHECK(rp::renderHtml(none).find("class=\"owed\"") == std::string::npos);
}
