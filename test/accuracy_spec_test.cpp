// F2 accuracy-spec encoding (master plan §7 / Freeze F2).
//
// Two real checks plus the skipped acceptance STUBS:
//  * a guard that pins the locked target values (a tripwire — editing a target
//    reds this test, forcing an explicit acknowledgement that F2 is changing),
//  * internal-consistency invariants the target set must satisfy,
//  * skipped placeholders naming the M2/M3 system-level acceptance tests, kept
//    skipped (no Localizer/motion yet) so the suite stays green and honest.

#include "doctest.h"
#include "shulib/spec/accuracy.hpp"
#include "shulib/units/quantity.hpp"

using namespace shulib::spec;

TEST_CASE("F2 spec guard: the locked target values (a tripwire on the Freeze)") {
    CHECK(kHeadingErrorMaxDeg            == doctest::Approx(1.0));
    CHECK(kPositionErrorEndOfRun.value() == doctest::Approx(1.0));
    CHECK(kRepeatability.value()         == doctest::Approx(0.75));
    CHECK(kDockedPositionError.value()   == doctest::Approx(0.25));
}

TEST_CASE("F2 spec: internal-consistency invariants the target set must satisfy") {
    CHECK(kDockedPositionError < kPositionErrorEndOfRun);   // docking is tighter than dead-reckon
    CHECK(kRepeatability       < kPositionErrorEndOfRun);   // repeatability tighter than absolute
    CHECK(kDockedHeadingTypicalDeg < kHeadingErrorMaxDeg);  // the aspiration beats the hard cap
    CHECK(kHeadingErrorMaxDeg  > 0.0);
}

// ----- Acceptance-test STUBS — go live as the systems land (kept skipped now) -----

TEST_CASE("[acceptance][M2] dead-reckon heading holds over a 60s straight-line test"
          * doctest::skip()) {
    // TODO(M2): with Localizer(odom + IMU-owned heading), run 60s;
    // assert |heading error| < kHeadingErrorMaxDeg.
}

TEST_CASE("[acceptance][M3] end-of-60s fused pose within F2 targets"
          * doctest::skip()) {
    // TODO(M3): full fused run (GPS + AprilTag) with contact + spins;
    // assert position <= kPositionErrorEndOfRun AND |heading| < kHeadingErrorMaxDeg.
}

TEST_CASE("[acceptance][M3] vision docking nests a 1.6in pin within kDockedPositionError"
          * doctest::skip()) {
    // TODO(M3): DockToGoal closed-loop visual-servo;
    // assert final alignment <= kDockedPositionError AND |heading| < kHeadingErrorMaxDeg.
}
