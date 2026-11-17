// A MIRRORED AUTON through competition::runAuton — MIRROR1-A's T5, the runner half of T4, T8's COMP and
// [MIR] lines, and T10 (the MIRROR1 brief's §2.2). Everything here is the host test plant; nothing here
// has run on a robot.
//
// Every run is the program's chain (test/mirrored_pair_check.hpp's Robot: Chassis -> RunGuard ->
// MechanismService over robot two's real list -> the recording world, with the route registry). Robot
// one runs an entry as written from its own start; robot two runs the same entry MIRRORED from the
// reflected start. The reflection the test compares with is its OWN (rf(): (x, y, h) -> (-y, -x, -90 deg
// - h); rr(): (x, y, h) -> (x, -y, -h)), written out by hand.
//
// Each case attacks ONE rule and names the bug it would catch.

#include "doctest.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mirror_unchanged_runs.hpp"
#include "mirrored_pair_check.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/reflection.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib::units::literals;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::competition::AutonContext;
using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::AutonRegistry;
using shulib::competition::AutonRunResult;
using shulib::competition::FieldMirror;
using shulib::competition::mirrored;
using shulib::competition::StartPose;
using shulib::competition::override2026::kSkillsMirror;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::math::Reflection;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr double kPi = 3.14159265358979323846;

Pose2d rf(const Pose2d& p) {
    return Pose2d{Length{-p.y().value()}, Length{-p.x().value()}, Angle::radians(-kPi / 2.0 - p.heading().radians())};
}
Pose2d rr(const Pose2d& p) {
    return Pose2d{p.x(), Length{-p.y().value()}, Angle::radians(-p.heading().radians())};
}

const shulib::kinematics::TankKinematics& tank() {
    static const shulib::kinematics::TankKinematics k{Length{12.0}};
    return k;
}

const Pose2d kStart{-36_in, -55.7_in, 0_deg};  // the brief's worked start (robot one, red wall's Quadrant)

// What a routine saw at its first step: the localizer (the REAL field) and chassis.pose() (the route's).
Pose2d gReal{};
Pose2d gSeen{};
bool gIsMirrored = false;

RoutineResult seatAndDrive(AutonContext& ctx) {
    Routine r{ctx, "seat"};
    r.then([&ctx] {
        gReal = ctx.chassis.deps().localizer->pose();
        gSeen = ctx.chassis.pose();
        gIsMirrored = shulib::competition::isMirrored(ctx);
    });
    r.driveTo(-20_in, -55.7_in);
    r.turnTo(60_deg);
    r.moveTo(Pose2d{-14_in, -44_in, 60_deg});
    return r.result();
}

/// A routine the deadline CUTS (a long hold past endActionAt) — so the park runs.
RoutineResult driveThenHoldLong(AutonContext& ctx) {
    Routine r{ctx, "cut"};
    r.driveTo(-20_in, -55.7_in);
    r.hold(30_s);
    return r.result();
}

bool parkMove(AutonContext& ctx) {
    return ctx.chassis.moveTo(Pose2d{-8_in, -55.7_in, 0_deg}, {.timeout = 3_s}) == shulib::control::ExitReason::Settled;
}

AutonEntry entry(const char* name, shulib::competition::AutonRoutine routine, StartPose start,
                 double endActionAt = 55.0, double hardStopAt = 58.0) {
    AutonEntry e{};
    e.name = name;
    e.kind = AutonKind::Skills;
    e.routine = routine;
    e.start = start;
    e.endActionAt = Time{endActionAt};
    e.hardStopAt = Time{hardStopAt};
    return e;
}

std::unique_ptr<mirror_pair::Robot> robot(const Pose2d& trueStart) {
    return std::make_unique<mirror_pair::Robot>(tank(), trueStart, 1.0);
}

/// The COMP lines of a track, in order.
std::vector<std::string> comp(const mirror_pair::Track& t) {
    std::vector<std::string> out;
    for (const std::string& l : t.lines) {
        if (l.rfind("[COMP] ", 0) == 0) {
            out.push_back(l.substr(7));
        }
    }
    return out;
}

double maxPathGap(const mirror_pair::Track& one, const mirror_pair::Track& two, Pose2d (*map)(const Pose2d&),
                  bool& samePaces) {
    samePaces = one.paces.size() == two.paces.size();
    double m = 0.0;
    const std::size_t n = std::min(one.paces.size(), two.paces.size());
    for (std::size_t i = 0; i < n; ++i) {
        const Pose2d w = map(one.paces[i].truth);
        m = std::max(m, std::hypot((w.x() - two.paces[i].truth.x()).value(), (w.y() - two.paces[i].truth.y()).value()));
        m = std::max(m, std::fabs(w.heading().errorTo(two.paces[i].truth.heading())));
    }
    return m;
}

bool sameBits(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }

}  // namespace

// ═══ T10 NOTHING ELSE CHANGES ═════════════════════════════════════════════════════════════════════

// Bug caught: an UNMIRRORED run moved by one bit anywhere — MM12 (the identity "applied": a heading
// re-wrapped, a -0.0 normalised) included; the route writes a -0.0 heading and position on purpose.
// Three autons OWNED BY THE FIXTURE (a copy of the base's 40 in auton, the library's acceptance run, an
// every-verb FIELD auton) over the fixture's own mechanism list, through runAuton, on a 9 % weak-side
// tank and an X-drive: records, log lines, the truth after every pace. Nothing here is the team's, so a
// student's edit to src/team or src/robots never moves it (test/mirror_unchanged_runs.hpp's banner). The
// literals were MEASURED with that header compiled against a clean archive of MIRROR1-A's merge base
// 5f6633b (MIRROR1-PROGRESS.md: first before the library changed; again for the library-owned fixture,
// the same two numbers).
TEST_CASE("MIRROR1 T10 an unmirrored run through runAuton is bit-identical to the merge base (tank and X-drive)") {
    const shulib::kinematics::TankKinematics t{Length{12.0}};
    const shulib::kinematics::MatrixKinematics xd = shulib::kinematics::xDrive(Length{7.0});
    CHECK(mirror_t10::runnerHash(t, 0.91) == 0x0fff1e8756c26e85ULL);
    CHECK(mirror_t10::runnerHash(xd, 1.0) == 0x61e0c4294783616aULL);
}

// ═══ T5 the runner ════════════════════════════════════════════════════════════════════════════════

// Bug caught: MM10 (the runner seats the UNREFLECTED start), a re-seat of the position without the
// heading, pose() not reflected back at the route's first step, isMirrored(ctx) lying, a "-0.0" seat. A
// mirrored FIELD entry seats (-y, -x, -90 deg - h) — position AND heading, on the localizer — while the
// route reads robot one's own start back; its two COMP lines name the mirror and both poses.
TEST_CASE("MIRROR1 T5 a mirrored FIELD entry seats robot two's real start; the route still reads robot one's") {
    const AutonEntry e = mirrored(entry("skills", &seatAndDrive, StartPose::field(kStart)), kSkillsMirror);
    auto two = robot(rf(kStart));
    gIsMirrored = false;
    const mirror_pair::Track& t = two->run(e, 60_s);
    REQUIRE(t.result.started());
    CHECK(t.result.mirrored);
    CHECK(gIsMirrored);
    // the seat: (55.7, 36, -90) — positions EXACT, heading wrapped
    CHECK(t.result.seated.x().value() == 55.7);
    CHECK(t.result.seated.y().value() == 36.0);
    CHECK(t.result.seated.heading().approxEqual(Angle::degrees(-90.0), 1e-12));
    CHECK(gReal.x().value() == 55.7);
    CHECK(gReal.y().value() == 36.0);
    CHECK(gReal.heading().approxEqual(Angle::degrees(-90.0), 1e-9));
    CHECK(gSeen.approxEqual(kStart, Length{1e-9}, 1e-9));  // the route's own numbers
    const std::vector<std::string> lines = comp(t);
    REQUIRE(lines.size() == 4U);
    CHECK(lines[0] == "auton 'skills' (SKILLS, period 60.0s, park T+55.00s, stop T+58.00s, start FIELD MIRRORED): arming");
    CHECK(lines[1]
          == "auton 'skills': re-seated FIELD MIRRORED (55.70, 36.00) in facing -90.0 deg; the IMU read -90.0 deg, "
             "live after 0.00s");
    CHECK(lines[2] == "auton 'skills': MIRRORED across the skills diagonal -- robot one's start (-36.00, -55.70) facing 0.0 deg");
    CHECK(two->chassis.reflection().isIdentity());  // closed before runAuton returned
}

// Bug caught: the park (the end action) run OUTSIDE the mirrored run — MM14: robot two would drive to
// robot one's park, across the field. A routine CUT at its deadline, then the park; robot two's whole
// path, park included, is robot one's reflection to 1e-9.
TEST_CASE("MIRROR1 T5 the park runs mirrored: a cut routine, then the end action, all reflected") {
    AutonEntry e = entry("cut", &driveThenHoldLong, StartPose::field(kStart), 4.0, 8.0);
    e.endAction = &parkMove;
    auto one = robot(kStart);
    auto two = robot(rf(kStart));
    const mirror_pair::Track& a = one->run(e, 60_s);
    const mirror_pair::Track& b = two->run(mirrored(e, kSkillsMirror), 60_s);
    CHECK(a.result.guard.scoringCut);
    CHECK(b.result.guard.scoringCut);
    CHECK(a.result.guard.endActionSucceeded);
    CHECK(b.result.guard.endActionSucceeded);
    bool same = false;
    CHECK(maxPathGap(a, b, rf, same) <= 1e-9);
    CHECK(same);
    // robot two's park ended at the reflection of (-8, -55.7): (55.7, 8)
    const Pose2d end = b.paces.back().truth;
    CHECK(std::hypot(end.x().value() - 55.7, end.y().value() - 8.0) < 1.0);
}

namespace {
RoutineResult relativeRoute(AutonContext& ctx) {
    Routine r{ctx, "relative"};
    r.moveTo(Pose2d{24_in, 12_in, 30_deg});
    r.turnTo(-45_deg);
    r.driveTo(36_in, 0_in);
    return r.result();
}
}  // namespace

// Bug caught: MM11 (a RELATIVE mirrored entry reflected across the field's diagonal — the robot frame's
// origin is wherever the robot stands, so the field's line means nothing there). RELATIVE seats (0, 0, 0)
// — never "-0.0" — and mirrors in the robot's own frame, (x, y, h) -> (x, -y, -h): with robot two
// placed at the field image of robot one's spot, its TRUE path is robot one's field reflection (P1 D).
TEST_CASE("MIRROR1 T5 a mirrored RELATIVE entry seats (0, 0, 0) and mirrors in the robot's own frame") {
    const AutonEntry e = entry("relative", &relativeRoute, StartPose::relative(), 25.0, 28.0);
    auto one = robot(kStart);
    auto two = robot(rf(kStart));  // the drivers place robot two at the image of robot one's spot
    const mirror_pair::Track& a = one->run(e, 60_s);
    const mirror_pair::Track& b = two->run(mirrored(e, kSkillsMirror), 60_s);
    CHECK(b.result.mirrored);
    for (const double c : {b.result.seated.x().value(), b.result.seated.y().value(), b.result.seated.heading().radians()}) {
        CHECK(c == 0.0);
        CHECK_FALSE(std::signbit(c));
    }
    bool same = false;
    CHECK(maxPathGap(a, b, rf, same) <= 1e-9);
    CHECK(same);
    // in its own frame robot two's estimate ends at robot one's, every left a right
    const Pose2d endOne = one->chassis.deps().localizer->pose();
    const Pose2d endTwo = two->chassis.deps().localizer->pose();
    CHECK(endTwo.approxEqual(rr(endOne), Length{1e-9}, 1e-9));
    const std::vector<std::string> lines = comp(b);
    REQUIRE(lines.size() >= 3U);
    CHECK(lines[1].rfind("auton 'relative': re-seated RELATIVE MIRRORED (0.00, 0.00) in facing 0.0 deg;", 0) == 0);
    CHECK(lines[2] == "auton 'relative': MIRRORED in the robot's own frame -- robot one's start (0.00, 0.00) facing 0.0 deg");
}

// Bug caught: a ROUTE entry (F8, through runner::PathRunner) not mirrored — its Start re-seeded unreflected
// one tick after the reflected re-seat, or its legs driven in robot one's numbers. PathRunner drives only
// through the Chassis, so the whole route comes out reflected.
TEST_CASE("MIRROR1 T5 a ROUTE entry runs mirrored through PathRunner") {
    using shulib::config::HeadingMode;
    using shulib::config::WaypointMotion;
    static shulib::config::Route route{};
    std::strncpy(route.name, "mirror route", sizeof route.name - 1);
    route.waypoints[0] = {.x = kStart.x(), .y = kStart.y(), .heading = kStart.heading(), .headingMode = HeadingMode::Fixed,
                          .motion = WaypointMotion::Start};
    route.waypoints[1] = {.x = -20_in, .y = -55.7_in, .heading = 0_deg, .headingMode = HeadingMode::Fixed,
                          .motion = WaypointMotion::MoveToPose};
    route.waypoints[2] = {.x = -20_in, .y = -40_in, .heading = 0_deg, .headingMode = HeadingMode::Free,
                          .motion = WaypointMotion::DriveTo};
    route.waypoints[3] = {.x = 0_in, .y = 0_in, .heading = 135_deg, .headingMode = HeadingMode::Fixed,
                          .motion = WaypointMotion::TurnTo};
    route.count = 4;
    REQUIRE(route.valid(nullptr, 0));
    AutonEntry e{};
    e.name = "route";
    e.kind = AutonKind::Skills;
    e.route = &route;
    e.endActionAt = 55_s;
    e.hardStopAt = 58_s;
    auto one = robot(kStart);
    auto two = robot(rf(kStart));
    const mirror_pair::Track& a = one->run(e, 60_s);
    const mirror_pair::Track& b = two->run(mirrored(e, kSkillsMirror), 60_s);
    REQUIRE(a.result.outcome == shulib::competition::AutonRunOutcome::RouteRan);
    REQUIRE(b.result.outcome == shulib::competition::AutonRunOutcome::RouteRan);
    CHECK(b.result.mirrored);
    bool same = false;
    CHECK(maxPathGap(a, b, rf, same) <= 1e-9);
    CHECK(same);
    CHECK(a.paces.size() > 200U);
}

// Bug caught: MM9 (mirrored() not toggling back — robot one's own entry run reflected): mirrored(mirrored(e,
// k), k) is e, and runs BYTE-IDENTICAL to it — every log line and every pace's truth, bit for bit.
TEST_CASE("MIRROR1 T5 mirrored(mirrored(e, k), k) is e and runs byte-identical to it") {
    const AutonEntry e = entry("skills", &seatAndDrive, StartPose::field(kStart));
    const AutonEntry twice = mirrored(mirrored(e, kSkillsMirror), kSkillsMirror);
    CHECK(twice.mirror == nullptr);
    auto a = robot(kStart);
    auto b = robot(kStart);
    const mirror_pair::Track& ta = a->run(e, 60_s);
    const mirror_pair::Track& tb = b->run(twice, 60_s);
    CHECK(ta.lines == tb.lines);
    REQUIRE(ta.paces.size() == tb.paces.size());
    bool bits = true;
    for (std::size_t i = 0; i < ta.paces.size(); ++i) {
        bits = bits && sameBits(ta.paces[i].truth.x().value(), tb.paces[i].truth.x().value())
               && sameBits(ta.paces[i].truth.y().value(), tb.paces[i].truth.y().value())
               && sameBits(ta.paces[i].truth.heading().radians(), tb.paces[i].truth.heading().radians());
    }
    CHECK(bits);
}

// Bug caught: two mirrors stacked silently (D5 (2)); an identity "mirror" accepted. Mirrored by a
// DIFFERENT reflection is a precondition naming it; the same reflection from another object toggles back.
TEST_CASE("MIRROR1 T5 mirrored(): a second, DIFFERENT mirror refused by name; the identity refused") {
    static constexpr FieldMirror kOther{.name = "the other diagonal",
                                        .reflection = Reflection::across(Reflection::Line::Diagonal),
                                        .walls = shulib::localization::override2026::kWalls};
    static constexpr FieldMirror kSameLine{.name = "a copy of the skills diagonal",
                                           .reflection = Reflection::across(Reflection::Line::AntiDiagonal),
                                           .walls = shulib::localization::override2026::kWalls};
    static constexpr FieldMirror kNone{.name = "none", .reflection = Reflection{}, .walls = {}};
    const AutonEntry e = entry("skills", &seatAndDrive, StartPose::field(kStart));
    const AutonEntry once = mirrored(e, kSkillsMirror);
    CHECK(once.mirror == &kSkillsMirror);
    CHECK_THROWS_WITH_AS((void)mirrored(once, kOther),
                         "competition::mirrored: this auton is already mirrored by a DIFFERENT mirror -- mirror it "
                         "once (mirroring twice by the same mirror gives the route back as written)",
                         shulib::PreconditionError);
    CHECK(mirrored(once, kSameLine).mirror == nullptr);
    CHECK_THROWS_AS((void)mirrored(e, kNone), shulib::PreconditionError);
    // A mirror with no name (a review finding, 2026-11-13 — the name half of the precondition removed
    // survived): the second COMP line prints the mirror's name ("%.32s" of a null pointer is undefined).
    static constexpr FieldMirror kNameless{.name = "",
                                           .reflection = Reflection::across(Reflection::Line::AntiDiagonal),
                                           .walls = shulib::localization::override2026::kWalls};
    static constexpr FieldMirror kNullName{.name = nullptr,
                                           .reflection = Reflection::across(Reflection::Line::AntiDiagonal),
                                           .walls = shulib::localization::override2026::kWalls};
    CHECK_THROWS_WITH_AS((void)mirrored(e, kNameless),
                         "competition::mirrored: the FieldMirror must reflect and have a name -- use a season "
                         "constant (override2026::kSkillsMirror)",
                         shulib::PreconditionError);
    CHECK_THROWS_WITH_AS((void)mirrored(e, kNullName),
                         "competition::mirrored: the FieldMirror must reflect and have a name -- use a season "
                         "constant (override2026::kSkillsMirror)",
                         shulib::PreconditionError);
}

namespace {
/// Can mirrored() be handed a mirror of type M (an lvalue for `M&`, a temporary for a plain `M`)?
template <class M>
concept MirroredAccepts = requires(const AutonEntry& e, M&& m) { mirrored(e, static_cast<M&&>(m)); };
}  // namespace

// Bug caught (a review finding, 2026-11-13): mirrored() handed a TEMPORARY FieldMirror — the entry keeps
// its ADDRESS, so the registry, the runner, the wall reset and the Toggle step would read a dead object
// on the robot; -Wall -Wextra -Werror said nothing. The deleted overload refuses it AT COMPILE TIME; this
// test's own static_asserts are the assertion (the build of this file fails, naming them, if the
// deletion is lost). A named mirror — the season constant, or a const one of the team's own — still binds.
TEST_CASE("MIRROR1 T5 mirrored() refuses a TEMPORARY FieldMirror at compile time; a named one binds") {
    static_assert(MirroredAccepts<const FieldMirror&>, "a named const FieldMirror must bind");
    static_assert(MirroredAccepts<FieldMirror&>, "a named FieldMirror must bind");
    static_assert(!MirroredAccepts<FieldMirror>, "a temporary FieldMirror must be refused (it would dangle)");
    static_assert(!MirroredAccepts<const FieldMirror>, "a const temporary FieldMirror must be refused too");
    CHECK(mirrored(entry("skills", &seatAndDrive, StartPose::field(kStart)), kSkillsMirror).mirror == &kSkillsMirror);
}

// Bug caught: a mirrored name too long for its "<name> SKILLS MIR" button (D5 (3), D8) — refused BY THE
// REGISTRY'S OWN PATH (so a DropEntry registry drops just that auton and names it); 25 characters fit;
// the same 26-character name UNmirrored is still fine (29 is its limit).
TEST_CASE("MIRROR1 T5 a 26-character mirrored name is refused by name at registration; 25 is accepted") {
    const char* const n26 = "abcdefghijklmnopqrstuvwxyz";  // 26
    const char* const n25 = "abcdefghijklmnopqrstuvwxy";   // 25
    REQUIRE(std::strlen(n26) == 26U);
    const shulib::competition::AutonPeriods periods{.match = 30_s, .skills = 60_s};
    AutonRegistry reg{periods};
    CHECK_THROWS_WITH_AS(reg.add(mirrored(entry(n26, &seatAndDrive, StartPose::field(kStart)), kSkillsMirror)),
                         "auton 'abcdefghijklmnopqrstuvwxyz': a MIRRORED auton's name must be at most 25 characters "
                         "(its menu button reads '<name> SKILLS MIR' on one screen line)",
                         shulib::PreconditionError);
    reg.add(mirrored(entry(n25, &seatAndDrive, StartPose::field(kStart)), kSkillsMirror));
    reg.add(entry(n26, &seatAndDrive, StartPose::field(kStart)));
    CHECK(reg.size() == 2U);
    AutonRegistry drop{periods, shulib::competition::OnRefusal::DropEntry};
    drop.add(mirrored(entry(n26, &seatAndDrive, StartPose::field(kStart)), kSkillsMirror));
    CHECK(drop.size() == 0U);
    CHECK(std::string{drop.refusal(0)}.find("MIRRORED auton's name") != std::string::npos);
}

// ═══ T4 the runner's scope: closed on every way out ════════════════════════════════════════════════

namespace {
RoutineResult throwsMidway(AutonContext& ctx) {
    Routine r{ctx, "throws"};
    r.driveTo(-20_in, -55.7_in);
    r.then([] { throw std::runtime_error("a routine threw"); });
    return r.result();
}
bool parkThrows(AutonContext& ctx) {
    (void)ctx.chassis.moveTo(Pose2d{-24_in, -48_in, 30_deg}, {.timeout = 2_s});
    throw std::runtime_error("the park threw");
}
RoutineResult abandonsMidway(AutonContext& ctx) {
    Routine r{ctx, "abandons"};
    r.driveTo(-20_in, -55.7_in);
    r.then([&ctx] { ctx.guard.requestAbandon(); });
    r.hold(5_s);
    return r.result();
}
RoutineResult unmirroredFollowUp(AutonContext& ctx) {
    Routine r{ctx, "follow-up"};
    r.moveTo(Pose2d{30_in, 4_in, 8_deg});
    return r.result();
}

/// After a mirrored run ended its way: the mirror is OFF, and an UNMIRRORED RELATIVE run on the same robot
/// seats (0, 0, 0) unmirrored, commands its written target (every record of its leg aims at (30, 4, 8 deg)
/// bit for bit — a leaked mirror would aim at a reflection: (30, -4, -8 deg) in the robot's frame, (-4,
/// -30, -98 deg) across the diagonal) and ends there.
void followUpIsUnmirrored(mirror_pair::Robot& r) {
    CHECK(r.chassis.reflection().isIdentity());
    const int from = r.sink.recordCount();
    const AutonEntry u = entry("follow-up", &unmirroredFollowUp, StartPose::relative(), 25.0, 28.0);
    const AutonRunResult res = shulib::competition::runAuton(r.chassis, r.guard, u, &r.commands, r.sink, r.svc, 60_s);
    CHECK_FALSE(res.mirrored);
    CHECK(res.routine.ok);
    int aimed = 0;
    std::uint32_t leg = 0;  // the follow-up's first motion: its moveTo
    for (int i = from; i < r.sink.recordCount(); ++i) {
        const auto& rec = r.sink.recordAt(i);
        if (leg == 0 && rec.activeCommandId != 0) {
            leg = rec.activeCommandId;
        }
        if (leg != 0 && rec.activeCommandId == leg) {
            ++aimed;
            CHECK(sameBits(rec.targetPose.x().value(), 30.0));
            CHECK(sameBits(rec.targetPose.y().value(), 4.0));
            CHECK(sameBits(rec.targetPose.heading().radians(), Angle::degrees(8.0).radians()));
        }
    }
    CHECK(aimed > 50);
    const Pose2d end = r.chassis.pose();
    CHECK(std::hypot(end.x().value() - 30.0, end.y().value() - 4.0) < 1.0);
}
}  // namespace

// Bug caught: MM3 (the scope's destructor emptied) and any way out of runAuton that leaves the mirror ON —
// the next, unmirrored auton (or driver control) would steer reflected. A normal end, the deadline's cut
// (the park inside the scope), an abandon, a throw from the routine and a throw from the end action: after
// each, the mirror is off and an unmirrored follow-up run is unmirrored.
TEST_CASE("MIRROR1 T4 the runner closes the mirrored run on every way out") {
    SUBCASE("a normal end") {
        auto r = robot(rf(kStart));
        const AutonRunResult res = shulib::competition::runAuton(
            r->chassis, r->guard, mirrored(entry("normal", &seatAndDrive, StartPose::field(kStart)), kSkillsMirror),
            &r->commands, r->sink, r->svc, 60_s);
        CHECK(res.routine.ok);
        followUpIsUnmirrored(*r);
    }
    SUBCASE("the deadline's cut, then the park") {
        auto r = robot(rf(kStart));
        AutonEntry e = entry("cut", &driveThenHoldLong, StartPose::field(kStart), 4.0, 8.0);
        e.endAction = &parkMove;
        const AutonRunResult res = shulib::competition::runAuton(r->chassis, r->guard, mirrored(e, kSkillsMirror),
                                                                 &r->commands, r->sink, r->svc, 60_s);
        CHECK(res.guard.scoringCut);
        CHECK(res.guard.endActionSucceeded);
        followUpIsUnmirrored(*r);
    }
    SUBCASE("an abandon") {
        auto r = robot(rf(kStart));
        const AutonRunResult res = shulib::competition::runAuton(
            r->chassis, r->guard, mirrored(entry("abandon", &abandonsMidway, StartPose::field(kStart)), kSkillsMirror),
            &r->commands, r->sink, r->svc, 60_s);
        CHECK(res.guard.abandoned);
        followUpIsUnmirrored(*r);
    }
    SUBCASE("a throw from the routine") {
        auto r = robot(rf(kStart));
        CHECK_THROWS_AS((void)shulib::competition::runAuton(
                            r->chassis, r->guard,
                            mirrored(entry("throw", &throwsMidway, StartPose::field(kStart)), kSkillsMirror),
                            &r->commands, r->sink, r->svc, 60_s),
                        std::runtime_error);
        followUpIsUnmirrored(*r);
    }
    SUBCASE("a throw from the end action") {
        auto r = robot(rf(kStart));
        AutonEntry e = entry("park throws", &seatAndDrive, StartPose::field(kStart));
        e.endAction = &parkThrows;
        CHECK_THROWS_AS((void)shulib::competition::runAuton(r->chassis, r->guard, mirrored(e, kSkillsMirror),
                                                            &r->commands, r->sink, r->svc, 60_s),
                        std::runtime_error);
        followUpIsUnmirrored(*r);
    }
}

// Bug caught: runAuton running an UNMIRRORED auton on a Chassis whose mirror leaked (it would run
// reflected): refused, a precondition naming it.
TEST_CASE("MIRROR1 T4 runAuton refuses a chassis whose mirror is still on") {
    auto r = robot(kStart);
    const shulib::chassis::MirroredRun leaked{r->chassis, kSkillsMirror.reflection};
    CHECK_THROWS_WITH_AS((void)shulib::competition::runAuton(r->chassis, r->guard,
                                                             entry("plain", &seatAndDrive, StartPose::field(kStart)),
                                                             &r->commands, r->sink, r->svc, 60_s),
                         "runAuton: the chassis is still in a mirrored run -- a chassis::MirroredRun outlived its auton",
                         shulib::PreconditionError);
}

// ═══ T8 the lines: the worst cases, measured; ASCII; inside the buffers and the terminal's cut ═══════

namespace {
RoutineResult stillRoutine(AutonContext& ctx) {
    Routine r{ctx, "still"};
    r.pause(50_ms);
    return r.result();
}
bool ascii(const std::string& s) {
    for (const char c : s) {
        if (static_cast<unsigned char>(c) > 126U || static_cast<unsigned char>(c) < 32U) {
            return false;
        }
    }
    return true;
}
}  // namespace

// Bug caught (the brief's R8): a mirrored COMP line cut by the runner's 160-byte buffer or the terminal's
// 200-byte message cut — the mirror's name or robot one's start lost at the worst case — or a non-ASCII
// dash in a log string. The worst case, MEASURED through the real runner: a 25-character name (the
// mirrored maximum), robot one at (-9999.99, -9999.99) facing 89.9 deg (robot two: (9999.99, 9999.99)
// facing -179.9), the IMU reading -179.9. Each line is whole (its last words present) and under 160.
TEST_CASE("MIRROR1 T8 the mirrored COMP lines at their worst case: whole, ASCII, under 160 and under 200") {
    const Pose2d far{Length{-9999.99}, Length{-9999.99}, Angle::degrees(89.9)};
    const AutonEntry e = mirrored(entry("abcdefghijklmnopqrstuvwxy", &stillRoutine, StartPose::field(far)), kSkillsMirror);
    auto r = robot(rf(far));
    const mirror_pair::Track& t = r->run(e, 60_s);
    const std::vector<std::string> lines = comp(t);
    REQUIRE(lines.size() == 4U);
    for (const std::string& l : lines) {
        MESSAGE(l.size() << ": " << l);
        CHECK(ascii(l));
        CHECK(l.size() < 160U);
    }
    CHECK(lines[1].find("re-seated FIELD MIRRORED (9999.99, 9999.99) in facing -179.9 deg; the IMU read -179.9 deg, "
                        "live after ")
          != std::string::npos);
    CHECK(lines[1].substr(lines[1].size() - 1) == "s");
    CHECK(lines[2]
          == "auton 'abcdefghijklmnopqrstuvwxy': MIRRORED across the skills diagonal -- robot one's start (-9999.99, "
             "-9999.99) facing 89.9 deg");
}

// Bug caught: the [MIR] half-turn line cut or non-ASCII at its longest verb ("followTrajectory leg").
TEST_CASE("MIRROR1 T8 the [MIR] lines: ASCII, under the terminal's 200-byte cut, the longest verb whole") {
    auto r = robot(rf(kStart));
    {
        const shulib::chassis::MirroredRun run{r->chassis, kSkillsMirror.reflection};
        // robot one's heading 0 -> a leg to 180: exactly a half turn for robot two too
        (void)r->chassis.followTrajectory({Pose2d{-30_in, -55.7_in, 180_deg}}, {.timeout = 0.2_s});
    }
    std::string mir;
    for (int i = 0; i < r->sink.size(); ++i) {
        if (r->sink.at(i).subsystem == "MIR") {
            mir = r->sink.at(i).message;
        }
    }
    MESSAGE(mir.size() << ": " << mir);
    CHECK(mir
          == "mirrored followTrajectory leg turns 180.0 deg: within 1 deg of a half turn, robot two may sweep the "
             "same way as robot one, not the mirror way -- turn first, in two 90 deg turns, where it matters");
    CHECK(mir.size() == 194U);  // the longest verb and the leg's advice: under the 200-byte cut
    CHECK(ascii(mir));
    CHECK(mir.substr(mir.size() - 16) == "where it matters");
}

// Bug caught: the menu's "<name> SKILLS MIR" label cut, or "MIR" on an unmirrored button (T8).
TEST_CASE("MIRROR1 T8 the menu: '<name> SKILLS MIR' at 25 characters fits; an unmirrored label is unchanged") {
    const shulib::competition::AutonPeriods periods{.match = 30_s, .skills = 60_s};
    AutonRegistry reg{periods};
    reg.add(mirrored(entry("abcdefghijklmnopqrstuvwxy", &seatAndDrive, StartPose::field(kStart)), kSkillsMirror));
    reg.add(entry("plain skills", &seatAndDrive, StartPose::field(kStart)));
    shulib::competition::AutonMenu menu{reg, 0};
    const shulib::competition::MenuView v = menu.view(10.0);
    std::vector<std::string> labels;
    for (int i = 0; i < v.buttonCount; ++i) {
        labels.emplace_back(v.buttons[i].label);
    }
    CHECK(std::find(labels.begin(), labels.end(), "abcdefghijklmnopqrstuvwxy SKILLS MIR") != labels.end());
    CHECK(std::find(labels.begin(), labels.end(), "plain skills SKILLS") != labels.end());
    CHECK(std::string{"abcdefghijklmnopqrstuvwxy SKILLS MIR"}.size() == shulib::ui::kMediumCols);
}
