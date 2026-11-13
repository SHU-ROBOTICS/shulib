// Adapter tests for ProsDeviceRegistry THROUGH THE HOST SHIM (chunk R3d Part 2).
//
// ═══ WHAT ONLY THIS FILE CAN PROVE ══════════════════════════════════════════════════
// That the PHYSICAL-port → ZERO-INDEXED-registry conversion is right, and right in BOTH
// DIRECTIONS. src/programs/bench_r3a.cpp's registry comment (just above `kMaxRegistryIndex`) records the
// day the absence of this test cost a session: the tester scanned 1..21 as if the index were
// physical, shifted every reading by one port, and invented a device at index 21 (physical
// 22, which does not exist) — a confident, completely wrong port map that HA-120 had already
// predicted.
//
// A test that only asserted "port 1 reads a motor" would pass with the conversion MISSING as
// long as the fixture happened to put a motor on both. So the fixture below is deliberately
// asymmetric — a DIFFERENT device on every interesting index — and the assertions run from
// both ends of the range: physical 1 → index 0, physical 21 → index 20. The shim also
// records the last index it was asked for, so the conversion is asserted DIRECTLY and not
// only through its consequences.
//
// NO HARDWARE CLAIM: this tests the adapter against our BELIEF about PROS (HA-138). The
// belief is settled beside a brain's own Devices screen — the worksheet's Station G census.

#include "doctest.h"

#include <array>
#include <cstdint>
#include <string>

#include "pros/shim_control.hpp"

#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/pros/device_registry.hpp"

using shulib::hal::DeviceKind;
using shulib::hal::deviceKindWord;
using shulib::hal::isSmartPort;
using shulib::hal::kMaxSmartPorts;
using shulib::hal::pros::ProsDeviceRegistry;

TEST_CASE("ProsDeviceRegistry: PHYSICAL port p reads registry index p-1 — from BOTH ends") {
    // BUG CAUGHT (brief §4 test 5's mutation: `port` instead of `port − 1`). The fixture puts
    // a MOTOR at index 0 and a GPS at index 20, i.e. on physical ports 1 and 21 — the two
    // ends of the range, where an off-by-one in either direction shows up as a wrong answer
    // or as a read past the end.
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(1, pros::c::E_DEVICE_MOTOR);
    pros::shim::setPluggedAtPort(21, pros::c::E_DEVICE_GPS);

    const ProsDeviceRegistry registry;

    CHECK(registry.pluggedKind(1) == DeviceKind::Motor);
    CHECK(pros::shim::registryState().lastIndex == 0);   // the conversion, asserted DIRECTLY

    CHECK(registry.pluggedKind(21) == DeviceKind::Gps);
    CHECK(pros::shim::registryState().lastIndex == 20);

    // And the ports either side of those, which an off-by-one would have hit instead.
    CHECK(registry.pluggedKind(2) == DeviceKind::None);
    CHECK(pros::shim::registryState().lastIndex == 1);
    CHECK(registry.pluggedKind(20) == DeviceKind::None);
    CHECK(pros::shim::registryState().lastIndex == 19);
}

TEST_CASE("ProsDeviceRegistry: an ASYMMETRIC census — every physical port answers its own device") {
    // A conversion that was off by one in a way the two-port test missed would scramble this
    // whole map. Each port carries a DIFFERENT device on purpose: a shift of one lands on a
    // neighbour that cannot be mistaken for the right answer.
    pros::shim::resetAll();
    struct Row { int port; pros::c::v5_device_e_t raw; DeviceKind kind; };
    const Row rows[] = {
        {1, pros::c::E_DEVICE_RADIO, DeviceKind::Radio},
        {2, pros::c::E_DEVICE_IMU, DeviceKind::Imu},
        {3, pros::c::E_DEVICE_ROTATION, DeviceKind::Rotation},
        {5, pros::c::E_DEVICE_DISTANCE, DeviceKind::Distance},
        {8, pros::c::E_DEVICE_OPTICAL, DeviceKind::Optical},
        {11, pros::c::E_DEVICE_MOTOR, DeviceKind::Motor},
        {12, pros::c::E_DEVICE_MOTOR, DeviceKind::Motor},
        {4, pros::c::E_DEVICE_AIVISION, DeviceKind::AiVision},  // VIS1-A1: beside the older one
        {16, pros::c::E_DEVICE_VISION, DeviceKind::Vision},
        {19, pros::c::E_DEVICE_ADI, DeviceKind::Adi},
        {21, pros::c::E_DEVICE_GPS, DeviceKind::Gps},
    };
    for (const Row& r : rows) {
        pros::shim::setPluggedAtPort(r.port, r.raw);
    }
    const ProsDeviceRegistry registry;
    for (const Row& r : rows) {
        CHECK_MESSAGE(registry.pluggedKind(r.port) == r.kind, "physical port ", r.port);
    }
    // Every other port is EMPTY — the adversarial default, and the proof that nothing was
    // smeared across neighbours.
    for (int p = 1; p <= static_cast<int>(kMaxSmartPorts); ++p) {
        bool listed = false;
        for (const Row& r : rows) {
            listed = listed || r.port == p;
        }
        if (!listed) {
            CHECK_MESSAGE(registry.pluggedKind(p) == DeviceKind::None, "physical port ", p);
        }
    }
}

TEST_CASE("ProsDeviceRegistry: ports 0 and 22 answer None WITHOUT touching PROS") {
    // BUG CAUGHT: an out-of-range port passed through to a uint8_t parameter. 22 would read
    // index 22 (past the documented 0-20 and the very thing that invented the phantom ADI
    // expander on 2026-08-18); a negative would convert to a huge index. The screen must
    // happen in OUR code, before the call — so the assertion is that the platform was never
    // asked at all, not merely that the answer was None.
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(1, pros::c::E_DEVICE_MOTOR);
    const ProsDeviceRegistry registry;
    const int before = pros::shim::registryState().queries;

    CHECK(registry.pluggedKind(0) == DeviceKind::None);
    CHECK(registry.pluggedKind(22) == DeviceKind::None);
    CHECK(registry.pluggedKind(-1) == DeviceKind::None);
    CHECK(registry.pluggedKind(1000) == DeviceKind::None);
    CHECK(registry.pluggedKind(-1000) == DeviceKind::None);
    CHECK(pros::shim::registryState().queries == before);  // PROS was never asked
    CHECK(pros::shim::registryState().lastIndex == -1);

    // The range predicate the seam, the fake and the adapter all share.
    CHECK_FALSE(isSmartPort(0));
    CHECK(isSmartPort(1));
    CHECK(isSmartPort(static_cast<int>(kMaxSmartPorts)));
    CHECK_FALSE(isSmartPort(static_cast<int>(kMaxSmartPorts) + 1));
}

TEST_CASE("ProsDeviceRegistry: EVERY v5_device_e_t value is mapped, and none becomes a Motor by accident") {
    // A device kind PROS adds later must land in Other, never in Motor: "shulib Calibrate"
    // builds its candidate list from the ports that answer Motor, and a mis-mapped device
    // would be powered as a drive motor. Walking the whole enum here is what keeps the
    // adapter's switch honest when the vendored header is updated.
    struct Row { pros::c::v5_device_e_t raw; DeviceKind kind; const char* word; };
    const Row rows[] = {
        {pros::c::E_DEVICE_NONE, DeviceKind::None, "EMPTY"},
        {pros::c::E_DEVICE_MOTOR, DeviceKind::Motor, "MOTOR"},
        {pros::c::E_DEVICE_ROTATION, DeviceKind::Rotation, "ROTATION"},
        {pros::c::E_DEVICE_IMU, DeviceKind::Imu, "IMU"},
        {pros::c::E_DEVICE_DISTANCE, DeviceKind::Distance, "DISTANCE"},
        {pros::c::E_DEVICE_RADIO, DeviceKind::Radio, "RADIO"},
        {pros::c::E_DEVICE_VISION, DeviceKind::Vision, "VISION"},
        {pros::c::E_DEVICE_ADI, DeviceKind::Adi, "ADI"},
        {pros::c::E_DEVICE_OPTICAL, DeviceKind::Optical, "OPTICAL"},
        {pros::c::E_DEVICE_GPS, DeviceKind::Gps, "GPS"},
        // VIS1-A1 (D4): the AI Vision sensor is its OWN kind — until then it answered Vision, the
        // older sensor's kind, so no registry check could refuse an older Vision sensor on the camera's
        // port. The older sensor (E_DEVICE_VISION, above) still answers Vision / "VISION".
        {pros::c::E_DEVICE_AIVISION, DeviceKind::AiVision, "AI-VISION"},
        {pros::c::E_DEVICE_SERIAL, DeviceKind::Other, "OTHER"},
        {pros::c::E_DEVICE_UNDEFINED, DeviceKind::Other, "OTHER"},
    };
    for (const Row& r : rows) {
        CHECK_MESSAGE(ProsDeviceRegistry::kindOf(r.raw) == r.kind,
                      "v5_device_e_t value ", static_cast<int>(r.raw));
        CHECK(std::string{deviceKindWord(ProsDeviceRegistry::kindOf(r.raw))} == r.word);
    }
    // A value PROS does not currently define (the "new device kind next season" case).
    CHECK(ProsDeviceRegistry::kindOf(static_cast<pros::c::v5_device_e_t>(33))
          == DeviceKind::Other);
    CHECK(ProsDeviceRegistry::kindOf(static_cast<pros::c::v5_device_e_t>(1))
          == DeviceKind::Other);

    // The VALUES themselves, transcribed from the vendored pros/device.h — they are NOT
    // consecutive, and an adapter that assumed they were would map every device wrong.
    CHECK(static_cast<int>(pros::c::E_DEVICE_MOTOR) == 2);
    CHECK(static_cast<int>(pros::c::E_DEVICE_IMU) == 6);
    CHECK(static_cast<int>(pros::c::E_DEVICE_GPS) == 20);
    CHECK(static_cast<int>(pros::c::E_DEVICE_UNDEFINED) == 255);
}

TEST_CASE("ProsDeviceRegistry: robot two's census, read through the seam") {
    // The shape Station G's first step produces: ten motors on 11-15 / 16-20, an IMU on 2,
    // the radio on 1 (src/robots/chassis_table.hpp's provenance). This is what "shulib Calibrate"
    // builds its candidate list from, and what the boot check compares a record against.
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(1, pros::c::E_DEVICE_RADIO);
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
    for (int p = 11; p <= 20; ++p) {
        pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
    }
    const ProsDeviceRegistry registry;

    int motors = 0;
    int imus = 0;
    int imuPort = 0;
    for (int p = 1; p <= static_cast<int>(kMaxSmartPorts); ++p) {
        const DeviceKind k = registry.pluggedKind(p);
        if (k == DeviceKind::Motor) {
            ++motors;
        }
        if (k == DeviceKind::Imu) {
            ++imus;
            imuPort = p;
        }
    }
    CHECK(motors == 10);
    CHECK(imus == 1);
    CHECK(imuPort == 2);           // DISCOVERED, not typed (ruling P1)
    // ONE query per port and not one more: the census is a single pass, and nothing in the
    // adapter re-reads or caches behind the caller's back.
    CHECK(pros::shim::registryState().queries == static_cast<int>(kMaxSmartPorts));
    CHECK(registry.pluggedKind(1) == DeviceKind::Radio);  // and the radio is not a motor
}
