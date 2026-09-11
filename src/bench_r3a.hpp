#pragma once
//
// R3a's bench validation entry point (chunk R3a, §4.2 item 1), extended at R3b
// Session 2 with the per-variant chassis table and the DRIVE station.
//
// Declared in its own header so src/main.cpp can dispatch to it under the
// compile-time robot selector WITHOUT the X-drive object graph ever being
// constructed — that graph's ports are invented (HA-111) and every adapter
// ctor read-back would throw at boot on the bench robot. Both tester variants
// (ROBOT=bench, the measured bench bot; ROBOT=tank, the 2026 chassis whose
// table carries MEASURED SIGNS since R3b Part 0b) boot into this as "Bench
// Tests" -- unless built with PROGRAM=drive, which boots src/drive_program.cpp
// ("shulib Drive") instead. The chassis table both share is src/chassis_table.hpp.
//
// This lives in src/, not include/shulib/, deliberately: it makes raw
// <pros/*> calls to print RAW value beside CANONICAL value, which is the
// whole point of the chunk, and include/shulib/ is PROS-free by CI guard.

namespace shulib::bench {

/// The whole bench session: device census, per-device raw-vs-canonical report,
/// loop-rate measurement, a live motor watch that captures signs — and, since R3b
/// Session 2, ONE station (10 DRIVE) that POWERS the drive motors through the
/// hal/pros adapters behind six safety gates. Every other station commands no
/// motion. Never returns.
void runR3a();

}  // namespace shulib::bench
