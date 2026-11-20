#pragma once
//
// mirrored_pair_numbers.hpp — the pair check's two numbers (test/mirrored_pair_check.hpp), on their own so the route
// builder's description (test/route_maker_describe.hpp) bakes the SAME numbers into the page (RMP1-C D5): the page's
// too-close check and the suite's never disagree, and neither is retyped.

namespace mirror_pair {

/// The least gap the two robots' squares may come to (inches): 3 in a side of the diagonal. A named
/// constant the team may change. PROVISIONAL (A4: HA-260) — a chosen margin, unmeasured.
inline constexpr double kMirrorGapIn = 6.0;
/// The robots' square footprint (inches): both are 15 in — held equal to route_preview::kPreviewFootprintIn by a
/// static_assert in mirrored_pair_test.cpp.
inline constexpr double kRobotSquareIn = 15.0;

}  // namespace mirror_pair
