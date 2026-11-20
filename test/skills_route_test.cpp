// THE SHARED ROUTES (src/team/skills_route.{hpp,cpp}; chunk ROB1-BT, 2026-11-16, named routes RMP1-C — the route
// maker's contract, the RMP1 brief's §6.3). Say "host test plant", never "simulator". Nothing here has run on a robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * THE CONTRACT: exactly ONE marked region, its two marker lines each alone on its line, holding the route
//     maker's output for the EMPTY project (the skills route, no steps) byte for byte; no #include or `using` inside
//     it; the plumbing it needs outside it (the includes, the namespace, the literals, `using namespace this_robot;`,
//     teamRoutes() and the registration both autons files call).
//   * BOTH ROBOTS RUN IT: robot one registers it as written, robot two MIRRORED — each runs on the host test
//     plant through the Auton Tester's own sequence: 0 steps, finished, safe.
//   * THE BUILDING ROBOT'S NAMES: `cascade`, `claw`, ... and `facts::lv(i)` are THIS build's robot's (the host is
//     the bench build, whose autons are robot two's), and every one of them is chosen by kRobotId — never one
//     robot's declaration on both sides.

#include "doctest.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <regex>
#include <string>
#include <string_view>

#include "auton_tester_rig.hpp"
#include "fakes_by_kind.hpp"
#include "programs/auton_tester_run.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/kinematics/tank.hpp"
#include "source_pins.hpp"
#include "team/autons.hpp"
#include "team/robot_one_autons.hpp"
#include "team/skills_route.hpp"

namespace sb = shulib::bench;

namespace {

/// §6.3's EMPTY region, byte for byte (the route maker's exporter is held equal to it — the RMP1 brief's GL0).
constexpr std::string_view kEmptyRegion =
    "// ═══ ROUTE MAKER — route \"skills\" — BEGIN ═══════════════════════════════════════════════\n"
    "// Made in the route maker from src/team/route_maker.json. Change the route there and export again:\n"
    "// the route maker will not overwrite a hand edit between BEGIN and END.\n"
    "// To make this hand-written code instead, delete the BEGIN and END lines.\n"
    "namespace {\n"
    "// The route \"skills\", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across\n"
    "// the field's diagonal (the list at the bottom says so).\n"
    "chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {\n"
    "    chassis::Routine r{ctx, \"skills\"};\n"
    "    // NO STEPS YET: the team's route is made in the route maker and written here.\n"
    "    return r.result();\n"
    "}\n"
    "\n"
    "// Its entry.\n"
    "competition::AutonEntry skillsRoute() {\n"
    "    competition::AutonEntry e{};\n"
    "    e.name = \"skills\";                                // what the brain's menu shows\n"
    "    e.kind = competition::AutonKind::Skills;          // the 60 s skills run\n"
    "    e.routine = &skillsSteps;                         // the steps above\n"
    "    e.start = competition::StartPose::relative();     // NO START YET: the route maker writes robot one's start\n"
    "    e.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action (a brake) starts\n"
    "    e.hardStopAt = 59_s;   // everything stops, 1 s before the run's 60 s\n"
    "    return e;\n"
    "}\n"
    "}  // namespace\n"
    "\n"
    "// EVERY ROUTE, in the order the brain's menu lists them (addTeamRoutes, below the END line, registers them).\n"
    "// Robot one registers each as written; robot two MIRRORS one marked Mirror::ForRobotTwo. Together::Yes: both\n"
    "// robots run it at the SAME time, so the host suite checks they never come too close.\n"
    "const TeamRoute kTeamRoutes[] = {\n"
    "    {&skillsRoute, Mirror::ForRobotTwo, Together::NotAnswered},  // \"skills\"\n"
    "};\n"
    "// ═══ ROUTE MAKER — route \"skills\" — END ═════════════════════════════════════════════════\n";

const std::string kBegin = "// ═══ ROUTE MAKER — route \"skills\" — BEGIN ";
const std::string kEnd = "// ═══ ROUTE MAKER — route \"skills\" — END ";

/// The 12 in stand-in tank the tester rig drives (the robots' drive is Calibrate's to measure).
const shulib::kinematics::TankKinematics& tank() {
    static const shulib::kinematics::TankKinematics k{shulib::units::Length{12.0}};
    return k;
}

}  // namespace

TEST_CASE("skills route (§6.3): ONE marked region, each marker alone on its line, holding the EMPTY route byte for byte") {
    // BUG CAUGHT: a second region or a missing marker (the route maker would refuse the file, or write into the
    // wrong place); a marker sharing its line with code; the starter region drifting from the exporter's empty
    // output (the route maker would take the starter for a HAND EDIT and refuse to write).
    const std::string file = source_pins::readFile("src/team/skills_route.cpp");
    CHECK(source_pins::count(file, kBegin) == 1);
    CHECK(source_pins::count(file, kEnd) == 1);
    const std::size_t b = file.find(kBegin);
    const std::size_t e = file.find('\n', file.find(kEnd)) + 1;
    REQUIRE(b != std::string::npos);
    REQUIRE(b < e);
    CHECK((b == 0 || file[b - 1] == '\n'));  // each marker starts its line ...
    CHECK(file.substr(b, e - b) == kEmptyRegion);  // ... and the region (both markers whole) is the contract's
    const std::string region = file.substr(b, e - b);
    CHECK(region.find("#include") == std::string::npos);
    CHECK(region.find("using ") == std::string::npos);
}

TEST_CASE("skills route (§6.3): the plumbing OUTSIDE the region -- the includes the route maker's code needs, the namespace, the literals, the building robot's names") {
    // BUG CAUGHT: an include the route maker relies on missing (its first export would not compile), the region
    // outside shulib::bench, or the names resolving to ONE robot's declarations on both robots.
    const std::string file = source_pins::readFile("src/team/skills_route.cpp");
    const std::string outside = file.substr(0, file.find(kBegin)) + file.substr(file.find('\n', file.find(kEnd)) + 1);
    for (const char* inc : {"#include \"robots/mechanism_table.hpp\"", "#include \"shulib/chassis/routine.hpp\"",
                            "#include \"shulib/competition/auton_registry.hpp\"",
                            "#include \"shulib/competition/auton_wall_reset.hpp\"",
                            "#include \"shulib/localization/override_walls.hpp\"",
                            "#include \"shulib/sequence/run_guard.hpp\"", "#include \"shulib/units/literals.hpp\"",
                            "#include \"shulib/math/angle.hpp\"", "#include \"shulib/competition/field_mirror.hpp\"",
                            "#include \"shulib/competition/override_mirror.hpp\""}) {
        CAPTURE(inc);
        CHECK(source_pins::count(outside, inc) == 1);
    }
    const std::string code = source_pins::squash(source_pins::stripComments(outside));
    CHECK(source_pins::count(code, "namespace shulib::bench { using namespace units::literals; using namespace this_robot;") == 1);
    CHECK(code.find("robot_one") == std::string::npos);  // a file both robots compile names neither robot
    CHECK(code.find("robot_two") == std::string::npos);
    // RMP1-C D2: the list is the region's; the registration is the plumbing's — robot two mirrors only a marked route
    CHECK(source_pins::count(code, "std::span<const TeamRoute> teamRoutes() { return kTeamRoutes; }") == 1);
    CHECK(source_pins::count(code, "const bool mirror = robot == TeamRobot::Two && route.mirror == Mirror::ForRobotTwo; "
                                   "registry.add(mirror ? competition::mirrored(e, competition::override2026::kSkillsMirror) : e);") == 1);
    CHECK(source_pins::count(code, "registerRoutes(registry, kTeamRoutes, robot);") == 1);
}

TEST_CASE("skills route: both robots run it on the host test plant -- robot one as written, robot two MIRRORED -- 0 steps, finished, safe") {
    // BUG CAUGHT: the shared route refused at registration on one robot (a schedule past 60 s, a mirrored name
    // over 25 characters), or failing to run through the tester's own sequence; robot one's copy mirrored.
    for (const bool robotTwo : {false, true}) {
        CAPTURE(robotTwo);
        shulib::competition::AutonRegistry reg{sb::kAutonPeriods};
        if (robotTwo) {
            sb::registerAutons(reg);
        } else {
            sb::registerRobotOneAutons(reg);
        }
        int at = -1;
        for (std::size_t i = 0; i < reg.size(); ++i) {
            at = std::string_view{reg.at(i).name} == "skills" ? static_cast<int>(i) : at;
        }
        REQUIRE(at >= 0);
        CHECK((reg.at(static_cast<std::size_t>(at)).mirror != nullptr) == robotTwo);
        tester_rig::TesterStack s{tank()};
        const sb::TesterRunRecord& r = s.run(reg, at);
        MESSAGE("skills route on robot ", robotTwo ? "two" : "one", ": ", std::string{sb::testerResultWord(r)}, ", steps ",
                r.run.routine.steps, ", ok ", r.run.routine.ok);
        CHECK(std::string_view{sb::testerResultWord(r)} == "FINISHED");
        CHECK(r.run.routine.steps == 0);
        CHECK(r.run.routine.ok);
        CHECK(r.run.guard.endActionRan);
        CHECK(s.driveSafe());
    }
    REQUIRE(sb::teamRoutes().size() == 1);
    CHECK(std::string_view{sb::teamRoutes()[0].entry().name} == "skills");
    CHECK(sb::teamRoutes()[0].mirror == sb::Mirror::ForRobotTwo);
    CHECK(sb::teamRoutes()[0].together == sb::Together::NotAnswered);  // the starter: nobody has answered yet
    CHECK(sb::teamRoutes()[0].entry().hardStopAt.value() == 59.0);
}

TEST_CASE("skills route: `cascade`, `claw`, ... and `facts::lv(i)` are the BUILDING robot's -- each chosen by kRobotId, robot one's on robot one, robot two's elsewhere") {
    // BUG CAUGHT (the route maker's landmine 10): robot two's package running robot ONE's cascade (another
    // robot's ports and facts) because a name in the selection points at one robot on both sides, or a
    // mechanism of the robots' region with no name there (the route maker's first step on it would not compile).
    namespace tr = sb::this_robot;
    CHECK_FALSE(tr::kRobotOneBuild);  // the host is the bench build
    CHECK(&tr::cascade == &sb::robot_two::cascade);
    CHECK(&tr::rightToggle == &sb::robot_two::rightToggle);
    CHECK(&tr::leftToggle == &sb::robot_two::leftToggle);
    CHECK(&tr::claw == &sb::robot_two::claw);
    CHECK(&tr::flipper == &sb::robot_two::flipper);
    CHECK(&tr::ramp == &sb::robot_two::ramp);
    for (std::size_t i = 0; i < 6; ++i) {
        CAPTURE(i);
        CHECK(&tr::facts::lv(i) == &sb::robot_two_facts::lv(i));
    }
    // ...and in the text, EVERY declaration of robot one's region — read from the region, not listed here, so a
    // seventh mechanism added to both regions without a name here fails — has its name, chosen BOTH ways.
    const std::string code =
        source_pins::squash(source_pins::stripComments(source_pins::readFile("src/robots/mechanism_table.hpp")));
    const std::size_t from = code.find("namespace facts = robot_one_facts;");
    const std::size_t to = code.find("inline constexpr MechanismTable kRobotOneMechanisms", from);
    REQUIRE(from != std::string::npos);
    REQUIRE(to != std::string::npos);
    const std::string region = code.substr(from, to - from);
    const std::regex decl{R"(inline constexpr (?:Lift|Arm|Flywheel|Power|Pneumatic) (\w+)\{)"};
    int names = 0;
    for (auto it = std::sregex_iterator(region.begin(), region.end(), decl); it != std::sregex_iterator(); ++it) {
        ++names;
        const std::string name = (*it)[1];
        CAPTURE(name);
        const std::string pick = "& " + name + " = kRobotOneBuild ? robot_one::" + name + " : robot_two::" + name + ";";
        CHECK(source_pins::count(code, pick) == 1);
    }
    CHECK(names == 6);  // the cascade, two side toggles, three pistons (robot one's list)
    CHECK(source_pins::count(code, "return kRobotOneBuild ? robot_one_facts::lv(i) : robot_two_facts::lv(i);") == 1);
    // The host is the bench build, so it cannot see robot one's side: the choice itself is pinned by its text,
    // and each robot's ARM build checks its own side (the two static_asserts, pinned here so neither goes).
    CHECK(source_pins::count(code, "inline constexpr bool kRobotOneBuild = kRobotId == RobotId::RobotOne;") == 1);
    CHECK(source_pins::count(code, "static_assert(kRobotId != RobotId::RobotOne || (&this_robot::cascade == "
                                   "&robot_one::cascade && &this_robot::facts::lv(0) == &robot_one_facts::lv(0)),") == 1);
    CHECK(source_pins::count(code, "static_assert(kRobotId != RobotId::RobotTwo || (&this_robot::cascade == "
                                   "&robot_two::cascade && &this_robot::facts::lv(0) == &robot_two_facts::lv(0)),") == 1);
}

TEST_CASE("skills route: `using namespace this_robot;` also gives the mechanism words a route's steps use (stalls, seesPiece, High, within ...)") {
    // BUG CAUGHT: a route the students write by hand (or the route maker exports) using `stalls` or `High` —
    // words robot_one_autons.cpp has through `using namespace robot_one;` — failing to compile in both robots'
    // builds, because the shared file's one `using` brought the names but not the words.
    using namespace sb::this_robot;
    [[maybe_unused]] const auto& s = stalls;
    [[maybe_unused]] const auto& p = seesPiece;
    [[maybe_unused]] const bool high = High;
    CHECK(high == shulib::mechanism::words::High);
    const std::string code =
        source_pins::squash(source_pins::stripComments(source_pins::readFile("src/robots/mechanism_table.hpp")));
    CHECK(source_pins::count(code, "namespace this_robot { using namespace mechanism::words;") == 1);
}

// ═══ THE SIDE TOGGLES UNDER THE MIRROR (ROB1-BT's review round, 2026-11-16) ═════════════════════════
// A mirror swaps a robot's sides: what robot one's RIGHT toggle reaches, mirrored robot two meets on its
// LEFT. The library does not swap a mechanism (field_mirror.hpp: "The ROBOT is not mirrored"), so the
// shared route picks the side itself — the line guide 17 and cookbook 9 give:
//     r.run(competition::isMirrored(ctx) ? leftToggle : rightToggle, 12_volt, 0.3_s);
namespace {
using namespace shulib::units::literals;

/// Peak volts each smart port's fake motor was commanded during a run (read every pace).
struct VoltSpy final : tester_rig::MechanismWorldLayer {
    tester_rig::FakesByKind* fakes = nullptr;
    double peak[22]{};
    void pace() override {
        for (std::uint8_t p = 1; p <= 21; ++p) {
            const double v = fakes->provider().fakeMotor(p).commandedVoltage().value();
            peak[p] = std::max(peak[p], v < 0.0 ? -v : v);
        }
        fakes->inner = inner;
        fakes->pace();
    }
};

/// The motor port `table` wires the mechanism named `name` to (its first motor role).
int portOf(const shulib::mechanism::MechanismTable& table, std::string_view name) {
    for (std::size_t i = 0; i < table.count; ++i) {
        if (std::string_view{table.specs[i].name} == name) {
            return table.wiring[i].motors[0];
        }
    }
    return 0;
}

shulib::chassis::RoutineResult pickedSide(shulib::competition::AutonContext& ctx) {
    using namespace sb::this_robot;
    shulib::chassis::Routine r{ctx, "side"};
    r.run(shulib::competition::isMirrored(ctx) ? leftToggle : rightToggle, 12_volt, 0.3_s);
    return r.result();
}
shulib::chassis::RoutineResult rightOnly(shulib::competition::AutonContext& ctx) {
    using namespace sb::this_robot;
    shulib::chassis::Routine r{ctx, "side"};
    r.run(rightToggle, 12_volt, 0.3_s);
    return r.result();
}

/// Run `routine` on `table`'s robot (mirrored or not) and return each port's peak volts.
VoltSpy runSide(const shulib::mechanism::MechanismTable& table, shulib::competition::AutonRoutine routine,
                bool mirrored) {
    shulib::competition::AutonEntry e{};
    e.name = "side";
    e.kind = shulib::competition::AutonKind::Skills;
    e.routine = routine;
    e.start = shulib::competition::StartPose::relative();
    e.endActionAt = 57_s;
    e.hardStopAt = 59_s;
    shulib::competition::AutonRegistry reg{sb::kAutonPeriods};
    reg.add(mirrored ? shulib::competition::mirrored(e, shulib::competition::override2026::kSkillsMirror) : e);
    tester_rig::FakesByKind fakes{table};
    VoltSpy spy;
    spy.fakes = &fakes;
    tester_rig::RigMechanisms binding = fakes.binding();
    binding.world = &spy;
    tester_rig::TesterStack s{tank(), 1, 1.0, motion_rig::plantConfig(), nullptr, 50, false, binding};
    const sb::TesterRunRecord& r = s.run(reg, 0);
    CHECK(std::string_view{sb::testerResultWord(r)} == "FINISHED");
    CHECK(r.run.routine.ok);
    return spy;
}
}  // namespace

TEST_CASE("skills route: the mirror swaps the robot's SIDES, not its mechanisms -- robot two's mirrored run "
          "drives its LEFT toggle only when the route picks it with isMirrored(ctx)") {
    // BUG CAUGHT: the documented line not reaching robot two's LEFT toggle port (isMirrored(ctx) false inside
    // robot two's mirrored run, or the step finding robot two's toggles by the wrong name); robot one's run
    // picking its left toggle. And it pins the library's stated rule — a plain `rightToggle` step runs robot
    // two's RIGHT toggle under the mirror — so a change to that rule meets this test and the guide's line.
    const auto& one = sb::robot_one::kRobotOneMechanisms;
    const auto& two = sb::robot_two::kRobotTwoMechanisms;
    const int oneRight = portOf(one, "right toggle");
    const int oneLeft = portOf(one, "left toggle");
    const int twoRight = portOf(two, "right toggle");
    const int twoLeft = portOf(two, "left toggle");
    REQUIRE(oneRight != 0);
    REQUIRE(oneLeft != 0);
    REQUIRE(oneRight != oneLeft);
    REQUIRE(twoRight != 0);
    REQUIRE(twoLeft != 0);
    REQUIRE(twoRight != twoLeft);
    const VoltSpy robotOne = runSide(one, &pickedSide, false);
    CHECK(robotOne.peak[oneRight] > 0.0);
    CHECK(robotOne.peak[oneLeft] == 0.0);
    const VoltSpy robotTwo = runSide(two, &pickedSide, true);
    CHECK(robotTwo.peak[twoLeft] > 0.0);
    CHECK(robotTwo.peak[twoRight] == 0.0);
    // without the pick, the mirror leaves the mechanism alone: robot two's RIGHT toggle runs
    const VoltSpy plain = runSide(two, &rightOnly, true);
    CHECK(plain.peak[twoRight] > 0.0);
    CHECK(plain.peak[twoLeft] == 0.0);
}
