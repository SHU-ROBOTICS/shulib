// THE BOOT CHECK — a description against the ports, and the boot's source decision
// (chunk R3d Part 2, brief §4 tests 6 and 7).
//
// NO HARDWARE CLAIM: every registry answer here comes from hal::fake::FakeDeviceRegistry.
// Nothing in this file has read a physical smart port.
//
// The case that matters most is the LAST one: two drive ports EXCHANGED still read Ok,
// because both still carry motors. That is the honest limit of a port census, it is stated
// in the header and on the screen, and it is PINNED here — a limit nobody has pinned is a
// limit somebody will forget, and the day somebody "fixes" the check without understanding
// it, this test is what tells them what they broke.

#include "doctest.h"

#include <cstdint>
#include <string>

#include "shulib/calibration/boot_check.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/units/quantity.hpp"

using shulib::calibration::BootDecision;
using shulib::calibration::BootSource;
using shulib::calibration::bootDecisionWord;
using shulib::calibration::BootVerdict;
using shulib::calibration::chooseBootSource;
using shulib::calibration::PortClass;
using shulib::calibration::portClassWord;
using shulib::calibration::verifyAgainstRegistry;
using shulib::hal::DescriptionSource;
using shulib::hal::DeviceKind;
using shulib::hal::DrivetrainDescription;
using shulib::hal::DrivetrainKind;
using shulib::hal::MotorCartridge;
using shulib::hal::fake::FakeDeviceRegistry;
using shulib::units::Length;

namespace {

/// Robot two's measured drivetrain, as a RobotMeasured description: LEFT -11 +12 -13 +14 -15,
/// RIGHT +20 -19 +18 -17 +16, IMU on port 2 (src/robots/chassis_table.hpp's provenance).
DrivetrainDescription robotTwo() {
    DrivetrainDescription d{};
    d.kind = DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 2;
    const std::int8_t left[5] = {-11, 12, -13, 14, -15};
    const std::int8_t right[5] = {20, -19, 18, -17, 16};
    for (int i = 0; i < 5; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 5;
    d.right.count = 5;
    d.left.inchesPerRadian = Length{1.375};
    d.right.inchesPerRadian = Length{1.375};
    d.trackWidth = Length{13.7};
    d.source = DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "boot check test");
    return d;
}

/// The registry as the robot the description describes: motors on all ten drive ports, the
/// IMU on 2, the radio on 1.
FakeDeviceRegistry healthyRegistry() {
    FakeDeviceRegistry r;
    for (int p = 11; p <= 20; ++p) {
        r.setKind(p, DeviceKind::Motor);
    }
    r.setKind(2, DeviceKind::Imu);
    r.setKind(1, DeviceKind::Radio);
    return r;
}

}  // namespace

TEST_CASE("BootCheck 6a: everything present ⇒ Ok, and the row per port says so") {
    const DrivetrainDescription d = robotTwo();
    const FakeDeviceRegistry registry = healthyRegistry();
    const BootVerdict v = verifyAgainstRegistry(d, registry);

    CHECK(v.decision == BootDecision::Ok);
    CHECK(v.usable());
    CHECK(static_cast<int>(v.portCount) == 10);
    CHECK(static_cast<int>(v.missingCount) == 0);
    CHECK(static_cast<int>(v.swappedCount) == 0);
    CHECK(static_cast<int>(v.newCount) == 0);
    CHECK(v.imuOk);
    CHECK(v.imuFound == DeviceKind::Imu);
    CHECK(static_cast<int>(v.imuPort) == 2);
    CHECK(v.degradation.verdict == shulib::teleop::DriveVerdict::Drive);
    CHECK(std::string{v.reason}.find("OK: all 10 recorded drive ports") != std::string::npos);
    // The limit is said even when everything is fine — an operator reading a green screen is
    // the one who most needs to know what it does NOT cover.
    CHECK(std::string{v.reason}.find("NOT CHECKED") != std::string::npos);
    CHECK(std::string{bootDecisionWord(v.decision)} == "OK");

    // LEFT rows first, then RIGHT, in the description's own order, signs AS STORED.
    CHECK(v.ports[0].signedPort == -11);
    CHECK(static_cast<int>(v.ports[0].port) == 11);
    CHECK(v.ports[0].side == -1);
    CHECK(v.ports[0].state == PortClass::Present);
    CHECK(v.ports[5].signedPort == 20);
    CHECK(v.ports[5].side == +1);
    CHECK(std::string{portClassWord(v.ports[5].state)} == "PRESENT");
    CHECK_FALSE(v.isMissing(11));
}

TEST_CASE("BootCheck 6b: ONE missing port ⇒ Degraded, listing it") {
    const DrivetrainDescription d = robotTwo();
    FakeDeviceRegistry registry = healthyRegistry();
    registry.setKind(18, DeviceKind::None);  // a dead cable on the RIGHT side

    const BootVerdict v = verifyAgainstRegistry(d, registry);
    CHECK(v.decision == BootDecision::Degraded);
    CHECK(v.usable());
    CHECK(static_cast<int>(v.missingCount) == 1);
    CHECK(static_cast<int>(v.swappedCount) == 0);
    CHECK(v.degradation.verdict == shulib::teleop::DriveVerdict::DriveDegraded);
    CHECK(v.degradation.rightDead == 1);
    CHECK(v.degradation.leftDead == 0);
    CHECK(std::string{v.reason}.find("DEGRADED: dead 18") != std::string::npos);
    CHECK(v.isMissing(18));
    CHECK_FALSE(v.isMissing(17));
    CHECK_FALSE(v.isMissing(99));  // a port nobody listed is not "missing"
}

TEST_CASE("BootCheck 6c: TWO missing ⇒ still Degraded; THREE ⇒ Refuse (the shared policy)") {
    // The numbers are teleop::evaluateDegradation's, not this file's — one policy, two
    // programs. If they ever diverge, "shulib Drive" and "shulib Teleop" would disagree about
    // the same robot on the same day, which is how a driver stops trusting either.
    const DrivetrainDescription d = robotTwo();

    SUBCASE("two dead, one per side — drives") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(13, DeviceKind::None);
        registry.setKind(18, DeviceKind::None);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Degraded);
        CHECK(static_cast<int>(v.missingCount) == 2);
        CHECK(std::string{v.reason}.find("13") != std::string::npos);
        CHECK(std::string{v.reason}.find("18") != std::string::npos);
    }

    SUBCASE("two dead on ONE side — 3 of 5 still drives") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(11, DeviceKind::None);
        registry.setKind(12, DeviceKind::None);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Degraded);
        CHECK(v.degradation.leftDead == 2);
    }

    SUBCASE("three dead anywhere — the total cap refuses") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(11, DeviceKind::None);
        registry.setKind(12, DeviceKind::None);
        registry.setKind(20, DeviceKind::None);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK_FALSE(v.usable());
        CHECK(std::string{v.reason}.find("more than 2 motors dead") != std::string::npos);
        CHECK(std::string{v.reason}.find("11") != std::string::npos);
    }

    SUBCASE("three dead on ONE side — the per-side floor refuses first") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(11, DeviceKind::None);
        registry.setKind(12, DeviceKind::None);
        registry.setKind(13, DeviceKind::None);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK(std::string{v.reason}.find("LEFT side has fewer than 3") != std::string::npos);
    }
}

TEST_CASE("BootCheck 6d: a SWAPPED port REFUSES, even with everything else perfect (ruling P2)") {
    // A port that LOST its motor may be a dead cable (tolerated). A port that now carries a
    // DIFFERENT KIND of device is a rewired robot, and a rewired robot needs a new
    // calibration, not a best effort.
    const DrivetrainDescription d = robotTwo();

    SUBCASE("a rotation sensor where a drive motor was") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(14, DeviceKind::Rotation);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK(static_cast<int>(v.swappedCount) == 1);
        CHECK(static_cast<int>(v.missingCount) == 0);   // NOT counted as missing
        CHECK(v.ports[3].state == PortClass::Swapped);  // LEFT row 4 = port 14
        CHECK(v.ports[3].found == DeviceKind::Rotation);
        CHECK(std::string{v.reason}.find("14(ROTATION)") != std::string::npos);
        CHECK(std::string{v.reason}.find("rewired") != std::string::npos);
        // A SWAPPED port does not count as ANSWERING either — there is no motor there to
        // drive — so the shared policy on its own would have said DRIVE_DEGRADED, i.e. "carry
        // on without it". The swap check WINS over that, and must: "a motor is missing" and
        // "this robot was rewired" are different findings with different right answers.
        CHECK(v.degradation.verdict == shulib::teleop::DriveVerdict::DriveDegraded);
        CHECK(v.degradation.leftDead == 1);
    }

    SUBCASE("a swap refuses even when it is the ONLY thing wrong and the policy is happy") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(16, DeviceKind::Distance);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK(std::string{v.reason}.find("16(DISTANCE)") != std::string::npos);
    }

    SUBCASE("a swap wins over a missing port — the more serious finding is the one reported") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(13, DeviceKind::None);
        registry.setKind(19, DeviceKind::Optical);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK(static_cast<int>(v.missingCount) == 1);
        CHECK(static_cast<int>(v.swappedCount) == 1);
        CHECK(std::string{v.reason}.find("19(OPTICAL)") != std::string::npos);
    }
}

TEST_CASE("BootCheck 6e: the IMU port must carry an IMU") {
    const DrivetrainDescription d = robotTwo();

    SUBCASE("a motor on the IMU port") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(2, DeviceKind::Motor);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK_FALSE(v.imuOk);
        CHECK(v.imuFound == DeviceKind::Motor);
        CHECK(std::string{v.reason}.find("IMU on port 2") != std::string::npos);
        CHECK(std::string{v.reason}.find("MOTOR") != std::string::npos);
        // ...and that motor is ALSO reported as a NEW motor the calibration never saw.
        CHECK(static_cast<int>(v.newCount) == 1);
        CHECK(static_cast<int>(v.newPorts[0]) == 2);
    }

    SUBCASE("an EMPTY IMU port") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(2, DeviceKind::None);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK(std::string{v.reason}.find("now reads EMPTY") != std::string::npos);
    }

    SUBCASE("a SWAPPED drive port is reported BEFORE a bad IMU — the rewiring is the headline") {
        FakeDeviceRegistry registry = healthyRegistry();
        registry.setKind(2, DeviceKind::None);
        registry.setKind(15, DeviceKind::Gps);
        const BootVerdict v = verifyAgainstRegistry(d, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK(std::string{v.reason}.find("15(GPS)") != std::string::npos);
    }
}

TEST_CASE("BootCheck 6f: a NEW motor is a WARNING, never a refusal") {
    // An extra motor is not a drivetrain fault (an intake, a lift). But a DRIVE motor added
    // since the calibration would sit uncommanded and fight its side-mates, so it is always
    // listed.
    const DrivetrainDescription d = robotTwo();
    FakeDeviceRegistry registry = healthyRegistry();
    registry.setKind(7, DeviceKind::Motor);
    registry.setKind(8, DeviceKind::Motor);

    const BootVerdict v = verifyAgainstRegistry(d, registry);
    CHECK(v.decision == BootDecision::Ok);
    CHECK(static_cast<int>(v.newCount) == 2);
    CHECK(static_cast<int>(v.newPorts[0]) == 7);
    CHECK(static_cast<int>(v.newPorts[1]) == 8);
    CHECK(std::string{v.reason}.find("NEW motors not in the calibration: 7 8")
          != std::string::npos);
    CHECK(static_cast<int>(v.portCount) == 10);  // and they are NOT drive ports
}

TEST_CASE("BootCheck 6f': a DECLARED mechanism motor (a claw pivot, T3 Part 2) is named with the "
          "caller's words, never as a NEW motor; the decision and the rest of the prose are "
          "untouched") {
    // BUG CAUGHT: a declared claw reported as an unknown motor ("NEW motors ... 6 7" — the boot
    // line the T3 brief says must name them as claw pivots); a declared port that is EMPTY listed
    // anyway; a declared port that is a DRIVE port pulled out of its drive row (the collision is
    // the program's refusal to make, and the drive row must stay); the old prose changed for a
    // caller that declares nothing.
    const DrivetrainDescription d = robotTwo();
    FakeDeviceRegistry registry = healthyRegistry();
    registry.setKind(6, DeviceKind::Motor);
    registry.setKind(7, DeviceKind::Motor);
    registry.setKind(8, DeviceKind::Motor);  // an undeclared extra motor: still NEW
    const int claws[] = {6, 7, 9, 12};       // 9 is EMPTY; 12 is a drive port
    const shulib::calibration::DeclaredMotorPorts declared{
        .ports = claws, .label = "claw pivots (declared, not drive motors)"};

    const BootVerdict v = verifyAgainstRegistry(d, registry, declared);
    CHECK(v.decision == BootDecision::Ok);
    REQUIRE(static_cast<int>(v.newCount) == 1);
    CHECK(static_cast<int>(v.newPorts[0]) == 8);
    REQUIRE(static_cast<int>(v.declaredCount) == 2);
    CHECK(static_cast<int>(v.declaredPorts[0]) == 6);
    CHECK(static_cast<int>(v.declaredPorts[1]) == 7);
    CHECK(static_cast<int>(v.portCount) == 10);  // port 12 kept its drive row
    CHECK(std::string{v.reason}
          == "OK: all 10 recorded drive ports carry motors, IMU on port 2. NEW motors not in the "
             "calibration: 8; claw pivots (declared, not drive motors): 6 7. NOT CHECKED: two "
             "drive cables swapped WITH EACH OTHER look identical here -- the first drive catches "
             "that");
    // Only declared motors, no NEW one: "none" first, then the declared clause.
    registry.setKind(8, DeviceKind::None);
    const BootVerdict onlyClaws = verifyAgainstRegistry(d, registry, declared);
    CHECK(std::string{onlyClaws.reason}.find(
              "NEW motors not in the calibration: none; claw pivots (declared, not drive motors): "
              "6 7. NOT CHECKED")
          != std::string::npos);
    // DEGRADED carries the same clause at its end.
    registry.setKind(13, DeviceKind::None);
    const BootVerdict degraded = verifyAgainstRegistry(d, registry, declared);
    CHECK(degraded.decision == BootDecision::Degraded);
    CHECK(std::string{degraded.reason}
          == "DEGRADED: dead 13 -- driving without them (left dead 1, right dead 0). NEW motors "
             "not in the calibration: none; claw pivots (declared, not drive motors): 6 7");
    // The two-argument form, and a declared set whose ports carry no motor, say exactly the old
    // words: a caller that declares nothing sees no change.
    registry.setKind(13, DeviceKind::Motor);
    registry.setKind(8, DeviceKind::Motor);
    const BootVerdict old = verifyAgainstRegistry(d, registry);
    CHECK(static_cast<int>(old.declaredCount) == 0);
    CHECK(static_cast<int>(old.newCount) == 3);
    CHECK(std::string{old.reason}.find("NEW motors not in the calibration: 6 7 8. NOT CHECKED")
          != std::string::npos);
    const int empty[] = {9, 10};
    const BootVerdict none =
        verifyAgainstRegistry(d, registry, {.ports = empty, .label = "claw pivots"});
    CHECK(std::string{none.reason} == std::string{old.reason});
}

TEST_CASE("BootCheck 6g: an INVALID description refuses before a single port is read") {
    // Defensive: every caller checks valid() first. A refusal here is still better than a
    // graph built from a description that contradicts itself — and it must not be an
    // exception, because this runs at boot on a robot.
    DrivetrainDescription d = robotTwo();
    d.trackWidth = Length{0.0};  // UNSET
    const FakeDeviceRegistry registry = healthyRegistry();
    const BootVerdict v = verifyAgainstRegistry(d, registry);
    CHECK(v.decision == BootDecision::Refuse);
    CHECK(static_cast<int>(v.portCount) == 0);
    CHECK(registry.queries() == 0);  // nothing was read
    CHECK(std::string{v.reason}.find("contradicts itself") != std::string::npos);
    CHECK(std::string{v.reason}.find("track width") != std::string::npos);
}

TEST_CASE("BootCheck 6h: THE PINNED LIMIT — two drive ports EXCHANGED still read Ok") {
    // ═══ THIS IS NOT A BUG. IT IS THE BOUNDARY OF WHAT A PORT CENSUS CAN KNOW. ═══
    // Unplug port 12's cable and port 13's cable and swap them. Both ports still carry a
    // motor; every row classes PRESENT; the verdict is Ok. The description then drives port
    // 12's motor with port 13's recorded SIGN — and on robot two those signs are OPPOSITE
    // (+12 and -13), so the two members fight each other on the first command.
    //
    // What catches it: teleop::CoupledSideMonitor, at the FIRST DRIVE, within 250 ms, naming
    // the port (MOTOR_GROUP_DISAGREE). What must never happen: somebody "tightening" this
    // check until it claims to catch it. This test is the tripwire on that.
    const DrivetrainDescription d = robotTwo();
    const FakeDeviceRegistry registry = healthyRegistry();  // a swap is INVISIBLE to it
    const BootVerdict v = verifyAgainstRegistry(d, registry);

    CHECK(v.decision == BootDecision::Ok);
    CHECK(static_cast<int>(v.swappedCount) == 0);
    CHECK(std::string{v.reason}.find("swapped WITH EACH OTHER") != std::string::npos);
    CHECK(std::string{v.reason}.find("first drive catches that") != std::string::npos);

    // And the same for a RE-GEARING that keeps the ports: nothing about the census changes,
    // so the recorded scale is silently wrong until a tape says otherwise.
    DrivetrainDescription regeared = d;
    regeared.left.inchesPerRadian = Length{2.75};   // someone changed the sprockets
    regeared.right.inchesPerRadian = Length{2.75};
    const BootVerdict w = verifyAgainstRegistry(regeared, registry);
    CHECK(w.decision == BootDecision::Ok);
}

TEST_CASE("BootCheck 6i: a bench-bot-shaped drivetrain (4 per side) goes through the same policy") {
    // The policy's domain note (teleop/drivetrain_degradation.hpp): 3 of 4 drives, 2 of 4
    // refuses. The boot check must not have opinions of its own about the count.
    DrivetrainDescription d{};
    d.kind = DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 4;
    const std::int8_t left[4] = {15, 16, 17, 18};
    const std::int8_t right[4] = {11, 12, 13, 14};
    for (int i = 0; i < 4; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 4;
    d.right.count = 4;
    d.left.inchesPerRadian = Length{1.375};
    d.right.inchesPerRadian = Length{1.375};
    d.trackWidth = Length{12.0};
    d.source = DescriptionSource::RobotMeasured;

    FakeDeviceRegistry registry;
    for (int p = 11; p <= 18; ++p) {
        registry.setKind(p, DeviceKind::Motor);
    }
    registry.setKind(4, DeviceKind::Imu);
    CHECK(verifyAgainstRegistry(d, registry).decision == BootDecision::Ok);

    registry.setKind(15, DeviceKind::None);
    CHECK(verifyAgainstRegistry(d, registry).decision == BootDecision::Degraded);  // 3 of 4
    registry.setKind(16, DeviceKind::None);
    CHECK(verifyAgainstRegistry(d, registry).decision == BootDecision::Refuse);    // 2 of 4
}

TEST_CASE("BootCheck 7: chooseBootSource — EVERY row of the decision table") {
    // BUG CAUGHT (brief §4 test 7's mutation: swap the preference). The boot decision is data,
    // not an `if` chain somebody eyeballed: all eight rows are asserted, and each is the
    // reason a robot boots from one description rather than another.
    struct Row {
        bool recordDecoded;
        BootDecision recordVerdict;
        bool tableValid;
        BootSource want;
        const char* mustSay;
    };
    const Row rows[] = {
        // A decoded record ALWAYS wins, whatever the table says (ruling P3).
        {true, BootDecision::Ok, true, BootSource::UseRecord, "MEASURED"},
        {true, BootDecision::Ok, false, BootSource::UseRecord, "MEASURED"},
        {true, BootDecision::Degraded, true, BootSource::UseRecord, "some ports are dead"},
        {true, BootDecision::Degraded, false, BootSource::UseRecord, "some ports are dead"},
        // A record that FAILED its port check refuses — it never falls back to the OLDER
        // typed table, whatever state that table is in.
        {true, BootDecision::Refuse, true, BootSource::Refuse, "rewired"},
        {true, BootDecision::Refuse, false, BootSource::Refuse, "rewired"},
        // No record: the table, if it can describe a drivetrain at all.
        {false, BootDecision::Ok, true, BootSource::UseTable, "TYPED TABLE"},
        {false, BootDecision::Ok, false, BootSource::Refuse, "typed table cannot describe"},
    };
    for (const Row& r : rows) {
        const auto choice = chooseBootSource(r.recordDecoded, r.recordVerdict, r.tableValid);
        const std::string why{choice.why};
        CHECK_MESSAGE(choice.source == r.want, "record=", r.recordDecoded,
                      " verdict=", static_cast<int>(r.recordVerdict),
                      " table=", r.tableValid);
        CHECK_MESSAGE(why.find(r.mustSay) != std::string::npos, "why was: ", why);
        CHECK_FALSE(why.empty());
    }

    // The verdict byte is IGNORED when no record was decoded — so a caller that leaves it at
    // its default cannot accidentally produce a different answer.
    CHECK(chooseBootSource(false, BootDecision::Refuse, true).source == BootSource::UseTable);
    CHECK(chooseBootSource(false, BootDecision::Degraded, true).source == BootSource::UseTable);
    CHECK(chooseBootSource(false, BootDecision::Refuse, false).source == BootSource::Refuse);

    // It is constexpr, so the table is decided at compile time and a future `if` chain that
    // reached for a clock or a global would not compile.
    static_assert(chooseBootSource(true, BootDecision::Ok, false).source
                  == BootSource::UseRecord);
    static_assert(chooseBootSource(true, BootDecision::Refuse, true).source
                  == BootSource::Refuse);
    static_assert(chooseBootSource(false, BootDecision::Ok, true).source == BootSource::UseTable);
    static_assert(chooseBootSource(false, BootDecision::Ok, false).source == BootSource::Refuse);
}

TEST_CASE("BootCheck 7b: the record and the table, end to end through the chooser") {
    // The shape the boot takes (src/common/boot_source.hpp's decideBootSourceFrom() since G1 Part 2b;
    // src/main.cpp's before):
    // verify, then choose. Written here so the two halves are tested together, because that
    // join is where a program usually gets it wrong.
    const DrivetrainDescription record = robotTwo();
    FakeDeviceRegistry registry = healthyRegistry();

    // 1. a healthy robot with a record: the record, verified.
    {
        const BootVerdict v = verifyAgainstRegistry(record, registry);
        const auto choice = chooseBootSource(true, v.decision, /*tableValid=*/true);
        CHECK(choice.source == BootSource::UseRecord);
    }
    // 2. the same robot with one dead cable: still the record, minus that member.
    registry.setKind(18, DeviceKind::None);
    {
        const BootVerdict v = verifyAgainstRegistry(record, registry);
        CHECK(v.decision == BootDecision::Degraded);
        CHECK(v.isMissing(18));
        CHECK(chooseBootSource(true, v.decision, true).source == BootSource::UseRecord);
    }
    // 3. the card carried over from another robot (its ports carry other things): REFUSE,
    //    and NOT a quiet fall-back to this robot's typed table.
    registry.setKind(18, DeviceKind::Motor);
    registry.setKind(17, DeviceKind::Rotation);
    {
        const BootVerdict v = verifyAgainstRegistry(record, registry);
        CHECK(v.decision == BootDecision::Refuse);
        CHECK(chooseBootSource(true, v.decision, true).source == BootSource::Refuse);
    }
}
