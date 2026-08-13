#pragma once
//
// shim_control — the test-facing switchboard for the host PROS shim. NOT a
// real PROS header: adapters never include this; tests do, to program device
// state and reset the world between cases.
//
// ═══ THE SHIM'S HONEST LIMIT, in full (T2's ruling, stated where the shim
// lives rather than buried in a brief) ═══════════════════════════════════════
// This shim tests the ADAPTER against our BELIEF about PROS; it cannot test
// the belief. If the shim says get_position() returns degrees and the adapter
// converts degrees→radians, the test passes — and would pass identically if
// PROS actually returned rotations. A shared model cancels its own errors.
// Three things keep that honest:
//   1. Every semantic in this shim was transcribed from the VENDORED PROS
//      headers' own doc comments (file:line cited in each shim header), never
//      invented to make an adapter pass.
//   2. Every belief is a registered entry in docs/hardware-assumptions.md
//      (HA-94 onward for R1a), labelled by confidence, with the bench
//      measurement that settles it.
//   3. The shim's defaults are ADVERSARIAL where the trap is real (motor
//      ports default to rotations/red — the state "a different program on a
//      different day" leaves), so the shared-model cancellation at least
//      cannot hide the leave-device-as-is class of bug.
// The shim tests the adapter; HARDWARE tests the shim. The R1a bench runbook
// (docs/internal) walks the load-bearing beliefs on a real V5.

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

namespace pros::shim {

/// Reset EVERY shim device to its (deliberately adversarial) defaults — call
/// at the top of each adapter test case so no state leaks between cases.
inline void resetAll() {
    resetTime();
    resetMotors();
    resetRotations();
    resetImus();
    resetGps();
    resetControllers();
    resetBattery();
}

}  // namespace pros::shim
