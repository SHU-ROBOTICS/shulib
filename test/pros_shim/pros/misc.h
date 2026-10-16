#pragma once
//
// HOST SHIM for the C competition API of <pros/misc.h> (chunk CAL1, Part A1, 2026-10-08) — the
// four reads a program makes of the field's state: competition_get_status() and the three
// is_disabled / is_autonomous / is_connected tests. The status is programmable
// (pros::shim::competitionStatus()), and it starts at 0: driver control, enabled, no field.
//
// The enum's names and values are transcribed from the vendored include/pros/misc.h :45-50
// (COMPETITION_DISABLED 1 << 0, COMPETITION_AUTONOMOUS 1 << 1, COMPETITION_CONNECTED 1 << 2,
// COMPETITION_SYSTEM 1 << 3 — in the vendored file's order), at file scope as there; the functions
// sit in pros::c, as there. The real misc.h cannot be included on the host beside this shim's
// misc.hpp (their controller enums collide), so this file carries only the competition calls.
//
// Reset with pros::shim::resetCompetition(): pros::shim::resetAll() (shim_control.hpp) does not
// reach this file, which is newer than it.
//
// HONEST LIMIT: this shim tests a program against OUR BELIEF about PROS; it cannot test the
// belief. Hardware tests the belief (shim_control.hpp says it in full).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <cstdint>

typedef enum {
    COMPETITION_DISABLED = 1 << 0,
    COMPETITION_CONNECTED = 1 << 2,
    COMPETITION_AUTONOMOUS = 1 << 1,
    COMPETITION_SYSTEM = 1 << 3,
} competition_status;

namespace pros::shim {

/// The field's status bits as the next read returns them (0 = driver control, enabled, no field).
inline std::uint8_t& competitionStatus() {
    static std::uint8_t s = 0;
    return s;
}
/// How many times any of the four reads below was called.
inline int& competitionReads() {
    static int n = 0;
    return n;
}
inline void resetCompetition() {
    competitionStatus() = 0;
    competitionReads() = 0;
}

}  // namespace pros::shim

namespace pros::c {

inline std::uint8_t competition_get_status() {
    ::pros::shim::competitionReads() += 1;
    return ::pros::shim::competitionStatus();
}
inline std::uint8_t competition_is_disabled() {
    ::pros::shim::competitionReads() += 1;
    return (::pros::shim::competitionStatus() & COMPETITION_DISABLED) != 0 ? 1 : 0;
}
inline std::uint8_t competition_is_autonomous() {
    ::pros::shim::competitionReads() += 1;
    return (::pros::shim::competitionStatus() & COMPETITION_AUTONOMOUS) != 0 ? 1 : 0;
}
inline std::uint8_t competition_is_connected() {
    ::pros::shim::competitionReads() += 1;
    return (::pros::shim::competitionStatus() & COMPETITION_CONNECTED) != 0 ? 1 : 0;
}

}  // namespace pros::c
