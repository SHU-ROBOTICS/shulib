// t04_toggle_visit_push_spot_not_nearer: the push spot at the look spot (the look after could not see a frozen camera).
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
    .pushFrom = 16_in,
    .lookWithin = 0.5_s,
    .flipBound = 0.8_s,
    .backBound = 0.8_s,
    .settle = 0.2_s,
    .driveBound = 0.6_s,
    .margin = 0.3_s,
    .maxImageAge = 0.1_s,
});
// expect: the_push_spot_must_be_nearer_than_the_look_spot
