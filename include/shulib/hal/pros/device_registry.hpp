#pragma once
//
// ProsDeviceRegistry — IDeviceRegistry over PROS's smart-port registry (chunk R3d Part 2).
// The one place in the LIBRARY that converts a PHYSICAL port number into PROS's zero-indexed
// registry index.
//
// BINDS:
//  * ::pros::c::registry_get_plugged_type(uint8_t index) — apix.h:640-660, returning
//    v5_device_e_t. "Returns the type of the device plugged into the ZERO-INDEXED port …
//    The V5 port number from 0-20." (HA-138)
//
// ═══ THE R1a-SHAPED TRAP, AND WHY THE CONVERSION IS ONE LINE ════════════════════════
// The registry is ZERO-INDEXED and NOTHING ELSE IN PROS IS — every motor, sensor and
// adapter in this tree takes a physical 1..21 port. src/programs/bench_r3a.cpp's registry comment (the
// one just above `kMaxRegistryIndex`) records the day that cost: the tester scanned 1..21 as
// if the index were physical, which shifted every reading by one port and invented a device
// at index 21 (physical 22, which does not exist). It produced a confident, completely wrong
// port map — and HA-120 had already predicted exactly that, because the 2026-08-13 "ADI
// expander" sighting came from index 21.
// The same two-conventions-in-one-API shape as HA-122's /usd/ prefix.
//
// So in the LIBRARY the conversion appears EXACTLY ONCE, on the line marked below, and
// test/pros_device_registry_adapter_test.cpp pins it FROM BOTH ENDS: physical port 1 must
// read registry index 0, and physical port 21 must read index 20. A test that only checked
// one end would pass with an off-by-one in the other direction. (src/programs/bench_r3a.cpp's census
// keeps its own copy, deliberately — see the note at the conversion line.)
//
// Out-of-range ports (0, 22, negatives) answer None WITHOUT calling PROS at all. That is not
// defensive decoration: `registry_get_plugged_type` takes a uint8_t, so a caller's 22 would
// be passed straight through and read whatever index 22 holds, and a negative would convert
// to a large index. The seam's contract (hal/device_registry.hpp) makes "not a port" mean
// "empty", and the screen is what makes that true.
//
// ═══ WHAT THIS ADAPTER DOES NOT DO ═════════════════════════════════════════════════
// It does not cache. A registry read is a table lookup in the PROS daemon's own state, and
// a cached census is a census that can be wrong about a cable pulled thirty seconds ago —
// which is precisely the situation the boot check exists to catch. Callers that need a
// consistent snapshot take one themselves, in one pass, and say so.
//
// It does not construct anything. That is the whole point: discovering what is plugged in
// must be cheaper than opening it (hal/device_registry.hpp's header).
//
// HA register: HA-138 (the zero-index mapping), and HA-120 for the sighting that predicted
// it. Host-tested through test/pros_shim/pros/apix.h, which transcribes the vendored
// v5_device_e_t values and the zero-indexed contract — and, like every shim, tests the
// adapter against our BELIEF about PROS rather than the belief itself.

// ── THE FENCE, and the ONE adapter that needs a third flag ──────────────────────────
// The tree's rule since R1a: the suppression set is EXACTLY {-Wshadow, -Wsign-conversion},
// closed BEFORE any shulib code, and test/pros_adapter_fence_test.cpp enforces it
// structurally — a list that grows is warnings turned off for our own code with a pragma in
// front of them (R1a mutation M11). A speculative `-Wconversion` was added here while this
// file was being written and the fence test caught it the same hour, which is the guard
// working.
//
// -Wdeprecated-declarations is the one documented exception, and it exists because THIS is
// the only adapter that must include `pros/apix.h`: `registry_get_plugged_type` is declared
// nowhere else. apix.h pulls `api.h`, which pulls `pros/llemu.hpp`, whose `pros::lcd`
// namespace is `[[deprecated]]` unless liblvgl's own llemu is in the include path — so the
// ARM header gate (-Werror, every header compiled) fails on VENDORED code this tree does not
// use and cannot edit. It is the same category the src build gate already ignores BY PATH
// (include/pros/), applied here by flag because the header gate has no path filter. The
// fence test allows it only in a file that includes apix.h, and still demands the two
// mandatory flags.
// Rejected alternatives: `pros::c::get_plugged_type` from the narrow `pros/device.h` (a
// DIFFERENT indexing convention — 1-based — and the zero-indexed call is the one with the
// hardware history, bench_r3a.cpp's comment above `kMaxRegistryIndex`); re-declaring the
// function ourselves (a second source of truth for a vendored signature, and one that would
// drift SILENTLY); defining
// PROS's `_PROS_KERNEL_SUPPRESS_LLEMU_WARNING` from a library header (reaching into another
// project's internals, and order-dependent on which header a TU includes first).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "pros/apix.h"
#pragma GCC diagnostic pop

#include <cstdint>

#include "shulib/hal/device_registry.hpp"

namespace shulib::hal::pros {

/// IDeviceRegistry over `pros::c::registry_get_plugged_type`. Stateless, cacheless and
/// constructible anywhere; the ONE line that turns a physical port into PROS's zero-indexed
/// registry index lives in pluggedKind() and is pinned from both ends by the adapter test
/// (header: the 2026-08-18 off-by-one that invented a device at index 21).
class ProsDeviceRegistry final : public IDeviceRegistry {
public:
    /// Nothing to open, nothing to own — the registry is the brain's own table.
    ProsDeviceRegistry() = default;

    /// What PHYSICAL port `port` (1..kMaxSmartPorts) carries, right now. A port outside that
    /// range answers None WITHOUT touching PROS (header: uint8_t truncation would otherwise
    /// read a neighbouring or nonexistent index).
    [[nodiscard]] DeviceKind pluggedKind(int port) const noexcept override {
        if (!isSmartPort(port)) {
            return DeviceKind::None;
        }
        // ── THE ONE CONVERSION IN THE LIBRARY. Physical port p is registry index p − 1
        //    (apix.h; HA-138). It exists nowhere else under include/shulib, and nothing that
        //    consumes this seam re-derives it. There IS a second copy in the repository, and it
        //    is deliberate: src/programs/bench_r3a.cpp's census (`const int port = idx + 1;`) keeps its
        //    own, because it is a measuring tool with a hardware history and the R3d brief
        //    ruled it out of scope to rewrite over this seam. If either ever moves, the other
        //    must too — and the tester's comment above `kMaxRegistryIndex` in bench_r3a.cpp says
        //    the same thing from its side.
        const auto index = static_cast<std::uint8_t>(port - 1);
        return kindOf(::pros::c::registry_get_plugged_type(index));
    }

    /// PROS's device type as this library's DeviceKind. Every value of `v5_device_e_t` is
    /// named here on purpose — a device kind PROS adds later lands in `Other`, never in
    /// `Motor`, and the adapter test walks the whole enum so that stays true.
    [[nodiscard]] static DeviceKind kindOf(::pros::c::v5_device_e_t t) noexcept {
        switch (t) {
            case ::pros::c::E_DEVICE_NONE: return DeviceKind::None;
            case ::pros::c::E_DEVICE_MOTOR: return DeviceKind::Motor;
            case ::pros::c::E_DEVICE_ROTATION: return DeviceKind::Rotation;
            case ::pros::c::E_DEVICE_IMU: return DeviceKind::Imu;
            case ::pros::c::E_DEVICE_DISTANCE: return DeviceKind::Distance;
            case ::pros::c::E_DEVICE_RADIO: return DeviceKind::Radio;
            case ::pros::c::E_DEVICE_VISION: return DeviceKind::Vision;
            case ::pros::c::E_DEVICE_ADI: return DeviceKind::Adi;
            case ::pros::c::E_DEVICE_OPTICAL: return DeviceKind::Optical;
            case ::pros::c::E_DEVICE_GPS: return DeviceKind::Gps;
            case ::pros::c::E_DEVICE_AIVISION: return DeviceKind::Vision;
            case ::pros::c::E_DEVICE_SERIAL: return DeviceKind::Other;
            case ::pros::c::E_DEVICE_UNDEFINED: return DeviceKind::Other;
            default: break;
        }
        return DeviceKind::Other;
    }
};

}  // namespace shulib::hal::pros
