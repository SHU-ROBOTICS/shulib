#pragma once
//
// screen_touch.hpp — ONE read of the brain's touch screen, reduced PROS-free: the release-edge
// sample every panel model takes (ui::TouchSample) plus PROS's press count (moved here at chunk
// COMP-b's first step, 2026-09-30 — the COMP brief's decision ND3 (b)).
//
// "shulib Auton Tester" (COMP-t, 2026-09-28) wrote this reduction as testerTouchFrom(), beside its
// abort watch in src/programs/auton_abort_watch.hpp. The competition program needs the same read
// for its auton selector — and may not include the tester's watch header (the COMP-t brief's §4:
// COMP-b does not reuse the tester's watch) — so the reduction moved here, under a neutral name,
// and the tester's testerTouchFrom() became a wrapper over it that adds the one fact only the
// tester uses (a finger on the glass now). The tester's TesterTouch stays the tester's.
//
// A TEMPLATE so the host suite runs it on a struct with PROS's own field names
// (test/auton_tester_rig.hpp's FakeProsTouch) and a program on the real screen_touch_status_s_t;
// `touchErrorStatus` is pros::E_TOUCH_ERROR — an error read, every field -1, whose counts mean
// nothing (ui::ReleaseEdge ignores it, and never lets it move the release count).
//
// PROS-free, header-only; host-tested in test/guarded_run_test.cpp and, through the tester's
// wrapper, test/auton_abort_watch_test.cpp (6b). NOTHING HERE HAS RUN ON A ROBOT.

#include <cstdint>

#include "shulib/ui/touch_panel.hpp"

namespace shulib::bench {

/// One touch read, reduced: the panels' release-edge sample and PROS's press count.
struct ScreenTouch {
    ui::TouchSample sample{};     ///< error, releaseCount, x, y -- a panel's release edge
    std::int32_t pressCount = 0;  ///< PROS's press_count: +1 when a finger LANDS (A4: HA-178 (a))
};

/// PROS's screen_touch_status_s_t, reduced (the one place its five fields are read for the Auton
/// Tester and the competition program; shulib Calibrate and the bench tests read the status
/// themselves). `touchErrorStatus` is pros::E_TOUCH_ERROR.
template <class ProsTouch>
[[nodiscard]] constexpr ScreenTouch screenTouchFrom(const ProsTouch& t,
                                                    int touchErrorStatus) noexcept {
    ScreenTouch r{};
    r.sample.error = static_cast<int>(t.touch_status) == touchErrorStatus;
    r.sample.releaseCount = t.release_count;
    r.sample.x = t.x;
    r.sample.y = t.y;
    r.pressCount = t.press_count;
    return r;
}

}  // namespace shulib::bench
