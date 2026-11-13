// t06_toggle_visit_settle_unset: settle left UNSET (every number is the team's).
// The AF1S brief's T10: a Toggle visit request built through competition::toggleVisit(...) as a constant is
// checked AT BUILD. run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/competition/toggle_visit.hpp"
namespace cv = shulib::competition;
namespace ov = shulib::vision::override2026;
inline constexpr cv::ToggleVisitRequest bad = cv::toggleVisit({
    .toggle = ov::toggles::kRedWall,
    .flipper = rx::arm["score"],
    .back = rx::arm["stow"],
    .lookFrom = 16_in,
    .offBy = 6_in,
    .pushFrom = 4_in,
    .lookWithin = 0.5_s,
    .flipBound = 0.8_s,
    .backBound = 0.8_s,
    .driveBound = 0.6_s,
    .margin = 0.3_s,
    .maxImageAge = 0.1_s,
});
// expect: a_toggle_visit_number_is_unset_every_one_is_the_teams_from_step_0
