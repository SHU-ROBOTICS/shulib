#pragma once
//
// R3a's bench validation entry point (chunk R3a, §4.2 item 1).
//
// Declared in its own header so src/main.cpp can dispatch to it under the
// compile-time robot selector WITHOUT the X-drive object graph ever being
// constructed — that graph's ports are invented (HA-111) and every adapter
// ctor read-back would throw at boot on the bench robot.
//
// This lives in src/, not include/shulib/, deliberately: it makes raw
// <pros/*> calls to print RAW value beside CANONICAL value, which is the
// whole point of the chunk, and include/shulib/ is PROS-free by CI guard.

namespace shulib::bench {

/// The whole R3a session: device census, per-device raw-vs-canonical report,
/// loop-rate measurement, then a live monitor. READ-ONLY — commands no motion.
void runR3a();

}  // namespace shulib::bench
