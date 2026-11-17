// THE ROUTE BUILDER'S CONTROLS TAB AND DRIVING FEEL, judged by the C++ (chunk RMP1-B; the RMP1 brief §2.2's
// RMP1-B list). Say "host test plant", never "simulator". Host only: nothing here has run on a robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * FE1: the page's copy of the library's stick shaping drifting from the C++ — the suite re-makes the 9 630-point
//     table and it must equal test/goldens/route_maker/feel_curve.txt byte for byte (node checks the page against
//     the same file, bit for bit).
//   * FE2: the page and AxisShaping::valid() disagreeing at a boundary — feel_valid_rows.hpp is one static_assert per
//     row, each answer the PAGE's (goldens.mjs writes it from checkFeel's rule).
//   * CT21 / FE3: a configuration the page passes that the build refuses — each controls_*.hpp is the page's own
//     export on a fixture robot or on robot one's real declarations, compiled through DriverFile.
//   The refusals' half (CT1–CT31: the page refuses, the build refuses with the SAME name) is the compile-refusal
//   harness's rm_ct_*.cpp (`ctest -R compile_refusal_rm`).

#include "doctest.h"

#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>

#include "common/driver_file.hpp"
#include "mechanism_fixtures.hpp"
#include "route_maker_feel.hpp"
#include "shulib/teleop/teleop_loop.hpp"

#include "goldens/route_maker/feel_valid_rows.hpp"
#include "goldens/route_maker/controls_x_every_form.hpp"
#include "goldens/route_maker/controls_y_homes_and_holds.hpp"
#include "goldens/route_maker/controls_y_pneumatic_positions.hpp"
#include "robots/mechanism_table.hpp"
#include "goldens/route_maker/controls_one_every_form.hpp"
#include "goldens/route_maker/controls_one_holds_only.hpp"
#include "goldens/route_maker/controls_one_builders_map.hpp"

namespace {
std::string goldenText(const char* name) {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/test/goldens/route_maker/" + name, std::ios::binary};
    REQUIRE_MESSAGE(in.good(), "cannot open test/goldens/route_maker/" << name);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}
}  // namespace

TEST_CASE("RMP1-B FE1b: the feel's EDGES golden is what the library's shapeAxis makes now -- each dead zone's edge and its neighbours, -0, the limits") {
    // BUG CAUGHT (the second review): a dead zone compared with < for <=, or a -0 turned +0, which the 9 630-point
    // grid never lands on.
    INFO("test/goldens/route_maker/feel_edges.txt is stale: re-make it with `build/test/shulib_route_maker feel-edges`, then read the diff");
    CHECK(goldenText("feel_edges.txt") == route_maker::feelEdgeTable());
}

TEST_CASE("RMP1-B FE1: the feel golden is what the library's shapeAxis makes now, all 9 630 points") {
    // BUG CAUGHT: input_shaping.hpp's formula changing while the page's copy (and its golden) does not.
    const std::string want = route_maker::feelCurveTable();
    INFO("test/goldens/route_maker/feel_curve.txt is stale: re-make it with `build/test/shulib_route_maker feel`, then read the diff");
    CHECK(goldenText("feel_curve.txt") == want);
    std::size_t lines = 0;
    for (const char c : want) {
        lines += c == '\n' ? 1U : 0U;
    }
    CHECK(lines == 9630);
}

TEST_CASE("RMP1-B CT21 / FE3: the page's exports on the fixture robots build through DriverFile") {
    // Compiling this file IS the check (each golden instantiates DriverFile); the counts pin that every control
    // the page wrote reached the list.
    namespace gx = mech_fixture::robot_x::rm_golden_x_every_form;
    namespace gy1 = mech_fixture::robot_y::rm_golden_y_homes_and_holds;
    namespace gy2 = mech_fixture::robot_y::rm_golden_y_pneumatic_positions;
    CHECK(gx::kControls.count == gx::kCount);
    CHECK(gy1::kControls.count == gy1::kCount);
    CHECK(gy2::kControls.count == gy2::kCount);
    // the feel the page wrote is the feel the library gets (tank sticks, the forward curve, the ramp)
    CHECK(gx::kFeel.mode == shulib::teleop::DriveMode::TankSticks);
    CHECK(gx::kFeel.forward.deadband == 0.08);
    CHECK(gx::kFeel.forward.expo == 0.6);
    CHECK(gx::kFeel.yaw.expo == 0.3);
    CHECK(gx::kFeel.slew.ratePerSecond == 4.0);
    CHECK(gx::kLoop.releaseBrakeMode == shulib::hal::BrakeMode::Brake);
    CHECK(gy1::kFeel.mode == shulib::teleop::DriveMode::ArcadeField);
    CHECK(gy1::kLoop.releaseBrakeMode == shulib::hal::BrakeMode::Hold);
}

TEST_CASE("RMP1-B CT21: the page's exports on ROBOT ONE's real declarations build through DriverFile") {
    // BUG CAUGHT (the review): CT21 compiled only on fixture robots, so a form robot one's kinds take (a piston's
    // position by press, a home on a button, a two-button hold on a side toggle, autons only) could stop building
    // unseen; and guide 21 quotes the builder's map from controls_one_builders_map.hpp, held to the file by CT22.
    namespace e = shulib::bench::robot_one::rm_golden_one_every_form;
    namespace h = shulib::bench::robot_one::rm_golden_one_holds_only;
    namespace m = shulib::bench::robot_one::rm_golden_one_builders_map;
    CHECK(e::kControls.count == e::kCount);
    CHECK(h::kControls.count == h::kCount);
    CHECK(m::kControls.count == m::kCount);
    CHECK(e::kLoop.releaseBrakeMode == shulib::hal::BrakeMode::Brake);
}
