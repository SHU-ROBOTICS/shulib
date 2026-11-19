// skills_route.cpp — THE TEAM'S SKILLS ROUTE, shared by both robots (chunk ROB1-BT, 2026-11-16).
//
// Written ONCE, in ROBOT ONE's numbers. Both robots' builds compile this file: robot one registers the
// route as written (src/team/robot_one_autons.cpp), robot two MIRRORED across the field's diagonal
// (src/team/autons.cpp). `cascade`, `claw`, `facts::lv(i)` ... below are the BUILDING robot's own (the
// `using namespace this_robot;` line; src/robots/mechanism_table.hpp).
//
// A MIRROR SWAPS THE ROBOT'S SIDES: what robot one's RIGHT toggle does, robot two does with its LEFT one.
// So a side-toggle step picks the side:  r.run(competition::isMirrored(ctx) ? leftToggle : rightToggle, ...);
//
// THE ROUTE IS THE STUDENTS' (the guardrail G4). It is made in the route maker, which writes ONLY between
// the BEGIN and END lines below; to write it by hand instead, delete those two lines. Everything outside
// them is the plumbing the route needs — leave it.
//
// NOTHING HERE HAS RUN ON A ROBOT. The host suite runs this route on the host test plant on both robots
// (test/skills_route_test.cpp).

#include "team/skills_route.hpp"

#include "robots/mechanism_table.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/auton_wall_reset.hpp"
#include "shulib/competition/field_mirror.hpp"
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
// The skills route, in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across
// the field's diagonal (robot two's autons file says so in one line).
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "skills"};
    // NO STEPS YET: the team's route is made in the route maker and written here.
    return r.result();
}
}  // namespace

// The entry both robots register.
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
// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════

}  // namespace shulib::bench
