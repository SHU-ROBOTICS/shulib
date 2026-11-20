// route_maker_goldens.cpp — WG1's compiled twins (chunk RMP1-A2e; the RMP1 brief A.2): every golden's EXPORTED C++
// (test/goldens/route_maker/*.cpp, the page's own exporter's output) compiled into the TEST module and run the way
// the runner runs the same route's lines — RPV1's previewEntry on the same robot — so the two records can be
// compared byte for byte. The test module only (tools/route_maker/wasm/, SHULIB_ROUTE_MAKER_TEST_STEPS); the
// names resolve as in the shared skills-route file (§6.3 item 2), as test/route_maker_golden_test.cpp does it.

#include <cstdio>
#include <cstring>

#include "mechanism_fixtures.hpp"
#include "route_maker_runner.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/literals.hpp"
#include "team/skills_route.hpp"

#define RM_ON(robot)                         \
    using namespace shulib;                  \
    using namespace shulib::units::literals; \
    using namespace robot;                   \
    using shulib::bench::TeamRoute;          \
    using shulib::bench::Mirror;             \
    using shulib::bench::Together;

namespace rm_gl0 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl0_empty.cpp"
}
namespace rm_gl1 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl1_drive_turn_first.cpp"
}
namespace rm_gl1b { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl1b_drive_off_line.cpp"
}
namespace rm_gl2 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl2_reverse_and_turns.cpp"
}
namespace rm_gl3 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl3_curves.cpp"
}
namespace rm_gl3b { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl3b_curve_16_points.cpp"
}
namespace rm_gl4 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl4_mechanisms.cpp"
}
namespace rm_gl12s { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl12_fail_stop.cpp"
}
namespace rm_gl12c { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl12_fail_carry_on.cpp"
}
namespace rm_gl12k { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl12_fail_skip.cpp"
}
namespace rm_gl13 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl13_wait_past_the_cut.cpp"
}
namespace rm_gl18a { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl18a_move_never_homed.cpp"
}
namespace rm_gl18b { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl18b_move_while_homing.cpp"
}
namespace rm_gl18c { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl18c_home_then_move.cpp"
}
namespace rm_gl18d { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl18d_home_while_driving.cpp"
}
namespace rm_gl19 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl19_caps.cpp"
}
namespace rm_wg8 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/wg8_distinct_numbers.cpp"
}
namespace rm_gl20 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl20_named_routes.cpp"
}
namespace rm_gl21 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl21_lift_heights.cpp"
}
namespace rm_gl22 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl22_small_moves.cpp"
}
namespace rm_gl23 { RM_ON(mech_fixture::robot_x)
#include "goldens/route_maker/gl23_pair_near_the_diagonal.cpp"
}
namespace rm_smoke_one { RM_ON(shulib::bench::robot_one)
#include "goldens/route_maker/smoke_robot_one.cpp"
}
namespace rm_smoke_two { RM_ON(shulib::bench::robot_two)
#include "goldens/route_maker/smoke_robot_two.cpp"
}
#undef RM_ON

namespace {

struct Golden {
    const char* name;  ///< the golden's file name, without .cpp
    const char* robot; ///< the robot its route is for
    shulib::competition::AutonEntry (*entry)();
};
// Every golden in test/goldens/route_maker/ (the WG1 test holds this list equal to the directory).
constexpr Golden kGoldens[] = {
    {"gl0_empty", "x", &rm_gl0::skillsRoute},
    {"gl1_drive_turn_first", "x", &rm_gl1::skillsRoute},
    {"gl1b_drive_off_line", "x", &rm_gl1b::skillsRoute},
    {"gl2_reverse_and_turns", "x", &rm_gl2::skillsRoute},
    {"gl3_curves", "x", &rm_gl3::skillsRoute},
    {"gl3b_curve_16_points", "x", &rm_gl3b::skillsRoute},
    {"gl4_mechanisms", "x", &rm_gl4::skillsRoute},
    {"gl12_fail_stop", "x", &rm_gl12s::skillsRoute},
    {"gl12_fail_carry_on", "x", &rm_gl12c::skillsRoute},
    {"gl12_fail_skip", "x", &rm_gl12k::skillsRoute},
    {"gl13_wait_past_the_cut", "x", &rm_gl13::skillsRoute},
    {"gl18a_move_never_homed", "x", &rm_gl18a::skillsRoute},
    {"gl18b_move_while_homing", "x", &rm_gl18b::skillsRoute},
    {"gl18c_home_then_move", "x", &rm_gl18c::skillsRoute},
    {"gl18d_home_while_driving", "x", &rm_gl18d::skillsRoute},
    {"gl19_caps", "x", &rm_gl19::skillsRoute},
    {"wg8_distinct_numbers", "x", &rm_wg8::skillsRoute},
    {"gl20_named_routes", "x", &rm_gl20::skillsRoute},  // its FIRST route (the lines' route)
    {"gl21_lift_heights", "x", &rm_gl21::skillsRoute},
    {"gl22_small_moves", "x", &rm_gl22::skillsRoute},
    {"gl23_pair_near_the_diagonal", "x", &rm_gl23::skillsRoute},
    {"smoke_robot_one", "one", &rm_smoke_one::skillsRoute},
    {"smoke_robot_two", "two", &rm_smoke_two::skillsRoute},
};

shulib::competition::AutonEntry (*gEntry)() = nullptr;
void registerGolden(shulib::competition::AutonRegistry& reg) { reg.add(gEntry()); }

}  // namespace

/// Run golden `name`'s compiled export and write its record (pageSteps null: no runner dated it).
int runGolden(const char* name) {
    for (const Golden& g : kGoldens) {
        if (std::strcmp(g.name, name) == 0) {
            gEntry = g.entry;
            const route_preview::PreviewRobot robot = route_maker::previewRobot(g.robot, &registerGolden);
            shulib::competition::AutonRegistry reg{shulib::bench::kAutonPeriods,
                                                   shulib::competition::OnRefusal::DropEntry};
            robot.registerAutons(reg);
            const route_preview::Model m = route_preview::previewEntry(robot, reg, 0, {});
            route_maker::emit(route_maker::record(m, nullptr, nullptr));
            return m.exitCode;
        }
    }
    std::fprintf(stderr, "shulib_route_maker: no golden \"%s\"\n", name);
    return 2;
}

/// Every golden's name and robot, one per line.
void listGoldens() {
    for (const Golden& g : kGoldens) {
        std::printf("%s %s\n", g.name, g.robot);
    }
}
