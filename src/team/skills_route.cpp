// skills_route.cpp — THE TEAM'S ROUTES, shared by both robots (chunk ROB1-BT, 2026-11-16; named routes: RMP1-C).
//
// Written ONCE, in ROBOT ONE's numbers: the skills route and every other route the team names. Both robots' builds
// compile this file and register the list at the bottom (addTeamRoutes): robot one each route as written
// (src/team/robot_one_autons.cpp), robot two each route marked Mirror::ForRobotTwo MIRRORED across the field's
// diagonal (src/team/autons.cpp). `cascade`, `claw`, `facts::lv(i)` ... below are the BUILDING robot's own (the
// `using namespace this_robot;` line; src/robots/mechanism_table.hpp).
//
// A MIRROR SWAPS THE ROBOT'S SIDES: what robot one's RIGHT toggle does, robot two does with its LEFT one.
// So a side-toggle step picks the side:  r.run(competition::isMirrored(ctx) ? leftToggle : rightToggle, ...);
//
// THE ROUTES ARE THE STUDENTS' (the guardrail G4). They are made in the route maker, which writes ONLY between
// the BEGIN and END lines below; to write them by hand instead, delete those two lines. Everything outside
// them is the plumbing the routes need — leave it.
//
// NOTHING HERE HAS RUN ON A ROBOT. The host suite runs every route on the host test plant on both robots
// (test/skills_route_test.cpp, test/autons_test.cpp).

#include "team/skills_route.hpp"

#include "robots/mechanism_table.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/auton_wall_reset.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/literals.hpp"

namespace shulib::bench {

using namespace units::literals;  // 57_s, 24_in, 90_deg, 6_volt
using namespace this_robot;       // cascade, claw, flipper, ramp, the toggles, facts::lv(i): THIS build's robot's

// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════
// Made in the route maker from src/team/route_maker.json. Change the route there and export again:
// the route maker will not overwrite a hand edit between BEGIN and END.
// To make this hand-written code instead, delete the BEGIN and END lines.
namespace {
// The route "skills", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (the list at the bottom says so).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};
    // NO STEPS YET: the team's route is made in the route maker and written here.
    return r.result();
}

// Its entry.
competition::AutonEntry skillsRoute() {
    competition::AutonEntry e{};
    e.name = "skills";                                // what the brain's menu shows
    e.kind = competition::AutonKind::Skills;          // the 60 s skills run
    e.routine = &skillsSteps;                         // the steps above
    e.start = competition::StartPose::relative();     // NO START YET: the route maker writes robot one's start
    e.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action (a brake) starts
    e.hardStopAt = 59_s;   // everything stops, 1 s before the run's 60 s
    return e;
}
}  // namespace

// EVERY ROUTE, in the order the brain's menu lists them (addTeamRoutes, below the END line, registers them).
// Robot one registers each as written; robot two MIRRORS one marked Mirror::ForRobotTwo. Together::Yes: both
// robots run it at the SAME time, so the host suite checks they never come too close.
const TeamRoute kTeamRoutes[] = {
    {&skillsRoute, Mirror::ForRobotTwo, Together::NotAnswered},  // "skills"
};
// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════

// ── THE PLUMBING both autons files call (not the route builder's: it writes only between BEGIN and END) ──────────

std::span<const TeamRoute> teamRoutes() { return kTeamRoutes; }

void registerRoutes(competition::AutonRegistry& registry, std::span<const TeamRoute> routes, TeamRobot robot) {
    for (const TeamRoute& route : routes) {
        const competition::AutonEntry e = route.entry();
        const bool mirror = robot == TeamRobot::Two && route.mirror == Mirror::ForRobotTwo;
        registry.add(mirror ? competition::mirrored(e, competition::override2026::kSkillsMirror) : e);
    }
}

void addTeamRoutes(competition::AutonRegistry& registry, TeamRobot robot) { registerRoutes(registry, kTeamRoutes, robot); }

}  // namespace shulib::bench
