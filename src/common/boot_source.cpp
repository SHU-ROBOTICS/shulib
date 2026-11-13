// boot_source.cpp — the PROS shim of the boot source: the card, the registry, this build's typed
// table, and the serial (G1 Part 2b, 2026-09-26). Every DECISION is in boot_source.hpp
// (decideBootSourceFrom(), readRecord(), bootConfigFrom()), PROS-free and host-tested
// (test/boot_source_test.cpp); this file only supplies what a host cannot: the SD card, the brain's
// port registry and the per-build kChassis. Together they are src/main.cpp's R3d Part 2 boot block
// (its lines ~983-1059 at b622d9b) plus its record/table cross-check (~1118-1140), MOVED with every
// decision unchanged: the answer is RETURNED (a BootOutcome) instead of being written into
// main.cpp's globals, and a refusal is handed back for the caller to paint instead of being painted
// here. The order of the side effects is the old block's: open the card, read it, close it; THEN
// ask the registry — all before any adapter exists.
//
// Compiled in all seven builds, like every .cpp under src/ (the brief's A12): nothing here reads a build
// switch (only src/build_selection.hpp's one chain does — G1 Part 2c final), and in the builds whose
// program never calls decideBootSource() the linker's --gc-sections drops it. In the bench build
// src/robots/chassis_table.hpp's kChassis is the bench bot's table and in the xdrive build it is
// kNoChassis, which is harmless because no program in those builds asks for a boot source.
//
// Every bounded copy in the header carries a precision the compiler can check (-Wformat-truncation),
// an error in the src build gate. Each line goes to the serial as ONE write() (SER1, HA-189):
// hal::pros::serialWrapped(), never printf — stdout's buffer is shared by every task, unguarded, and
// a boot line runs to kBootLineBytes - 1 = 511 characters, so it WRAPS into pieces of at most
// hal::pros::kSerialLineBytes, one write() each, and is never cut.

#include "common/boot_source.hpp"

#include "robots/chassis_table.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/pros/block_source.hpp"
#include "shulib/hal/pros/char_sink.hpp"
#include "shulib/hal/pros/device_registry.hpp"

namespace shulib::bench {

BootOutcome decideBootSource(const char* tag) {
    // The card: opened, read and CLOSED before the registry is asked anything.
    RecordRead record{};
    {
        hal::pros::ProsBlockSource card{calibration::kRecordFileName};
        record = readRecord(card, card.path());
    }
    const hal::pros::ProsDeviceRegistry registry;
    // tableToDescription() is the ONLY path from the table into a graph; nothing reads kChassis
    // for a number anywhere else in a boot.
    const hal::DrivetrainDescription fromTable = tableToDescription(kChassis);
    char missing[160] = "";
    (void)describeMissingForDescription(kChassis, missing, sizeof missing);

    // kMechanisms (src/robots/mechanism_table.hpp, through boot_source.hpp): robot two's claws in the
    // ROBOT=tank builds, none elsewhere — converted into the config here (T3 Part 2).
    BootOutcome out = decideBootSourceFrom(
        record, registry, fromTable, missing, kMechanisms, kChassis.robot,
        [tag](const char* line) { hal::pros::serialWrapped(tag, line); });
    return out;
}

}  // namespace shulib::bench
