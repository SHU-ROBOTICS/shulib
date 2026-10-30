// IMU1 Part A (2026-10-29), T3 (c) — the pin: ProsImu's rate hold is HALF the control tick.
//
// ProsImu::yawRate() keeps its answer for kRateHoldUs so every read of one tick gets the tick's
// first answer and the next tick's first read is fresh (hal/pros/imu.hpp's header). That only
// works while the hold is half the tick: a longer hold would hold the NEXT tick's first read from
// this one (a frozen rate), a much shorter one would let a tick's later reads re-sample. So the
// hold is pinned to the tick, not to a number: a change to ProsTickPacer::kTickMs (a 5 ms tick, a
// 20 ms one) moves the hold or fails here. Integers both sides, the conversion written out — the
// tick is milliseconds, the hold microseconds (the brief's landmine 13). What the pin cannot test
// is the belief under it — that a tick's reads fall within 5 ms of its first and no reader reads
// between ticks (A4 register HA-215).
//
// Its OWN translation unit because it names ProsImu::kRateHoldUs, which the adapter did not have
// before IMU1: the reproduction-first build (the tree with the pre-IMU1 adapter) is the tree
// WITHOUT this one file — test/CMakeLists.txt globs *_test.cpp — while every other IMU1 test
// (test/imu_rate_once_per_tick_test.cpp) builds against either adapter (the brief's re-check R2).

#include "doctest.h"

#include <cstdint>

#include "pros/shim_control.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"

using shulib::hal::pros::ProsImu;
using shulib::hal::pros::ProsTickPacer;

// Bug caught (mutation M4 — the hold 5 ms → 10 ms): the hold no longer half the tick, so the next
// tick's first read is held from this one; or the tick changed and the hold did not follow it.
TEST_CASE("IMU1 T3 (c): ProsImu's rate hold is exactly half the control tick, in integers") {
    const std::uint32_t holdUs = ProsImu::kRateHoldUs;
    const std::uint32_t tickMs = ProsTickPacer::kTickMs;
    CHECK(holdUs * 2U == tickMs * 1000U);
}
