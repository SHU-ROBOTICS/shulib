#pragma once
//
// HOST SHIM for <pros/error.h> — the PROS sentinel values, verbatim from the
// vendored include/pros/error.h:24-36 so adapter screening logic sees the exact
// bit patterns the real SDK returns.
//
// HONEST LIMIT (T2/T3, stated in the shim's own words): this shim tests the
// adapter against OUR BELIEF about PROS; it cannot test the belief. If the
// belief is wrong here, it is wrong in the adapter's test in the same way, and
// the test passes anyway. Hardware tests the belief — every semantic modeled in
// this tree is a registered entry in docs/hardware-assumptions.md and the R1a
// bench runbook is where the load-bearing ones meet reality.

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM. If you are seeing this from the PROS \
Makefile, the shim has leaked into the robot include path: the binary would build against \
in-memory fake motors, upload cleanly, boot cleanly, and drive NOTHING."
#endif

#include <cmath>
#include <cstdint>

// Verbatim beliefs (vendored error.h:24,28,32,36):
#define PROS_ERR_BYTE (INT8_MAX)
#define PROS_ERR_2_BYTE (INT16_MAX)
#define PROS_ERR (INT32_MAX)
#define PROS_ERR_F (INFINITY)
