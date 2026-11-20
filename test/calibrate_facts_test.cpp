// src/robots/calibrate_facts.hpp — robot two's (and every robot's) facts for the standard Calibrate,
// DERIVED from its two tables (chunk CAL1 Part B, 2026-11-05; the CAL1 brief's §2.B tests 2 and 4).
// PROS-free: the host suite reads every robot's tables by name. NOTHING HERE HAS RUN ON A ROBOT.
//
// Each case names the bug it catches:
//   F1  robot two's derived facts ARE the literal facts CAL1 A2b's goldens were made with — Tank,
//       BLUE, {6 "claw pivot 6", 7 "claw pivot 7"} by the table's names, the robot's name, IMU 2,
//       no typed drive port — field by field (the goldens then prove it end to end)
//                                              (a mechanism entry dropped; the cartridge cleared)
//   F2  the mechanism half for EVERY robot and for hand-built lists: one fact per MOTOR ROLE, in
//       mechanism::declaredMotorPorts()' order, named by its mechanism (a two-motor lift gives two
//       facts with ONE name); a 3-wire line or a smart SENSOR port never a fact; the same ports
//       isDeclaredMechanismPort() answers for, port by port; factsProblem() passes for each
//   F3  the bench bot and the invented X-drive: their facts carry what their tables say — the
//       bench bot BLUE with no mechanism, the X-drive an UNSET cartridge (refused at Calibrate,
//       which the Makefile builds for neither)
//   F4  THE TYPED PORTS (D6 (a), as the check of D6 (a) corrected it): for every RobotId the typed
//       sides are tableToDescription(t)'s bit for bit when it is valid() — the test the boot makes
//       before it drives from a table — and EMPTY otherwise; today every robot's is empty, each
//       with its reason asserted (so a table that becomes valid turns this red, and its author
//       rewords the worksheet's Station G robot-two sentence); robot two's table MADE valid
//       carries LEFT -11 12 -13 14 -15 | RIGHT 20 -19 18 -17 16; each fact undone again gives none
//                    (the mapping ignores valid(); swaps the sides; is dropped)
//   F5  the glue (src/programs/calibrate_program.cpp), by its source: runCalibrate() hands the
//       standard Calibrate withTypedDriveFrom(kCalibrateFacts, kChassis) — on robot two the mapping
//       gives nothing, so ONLY this pin sees it dropped — and the glue makes no PROS call of its
//       own, writes no file, prints nothing; kCalibrateFacts is built over the build's own two
//       tables; src/main.cpp's Calibrate cases call its five entries, EACH IN ITS OWN entry
//       function (F5b checks the brace matcher that cuts main.cpp into them)
//                                              (the glue runs kCalibrateFacts bare; main.cpp's
//                                               Calibrate disabled() back on benchStateScreen;
//                                               two Calibrate cases swapped between functions —
//                                               the CAL1 Part B review round's R2/R3; the facts
//                                               constant built over another list — R1)
//   F6  mechanismFactsAre(): true for every right mapping (each robot, both fixtures, the empty
//       list), false for every wrong one — no list against robot two's, another robot's, a port
//       off, the order swapped, an entry added or dropped, a stray last entry, a copied or null
//       name; the static_assert on kCalibrateFacts is this function on every ARM build
//                                              (R1: facts built over NO mechanism list)
//   F7  (ROB1-BT) THE RECORD NAME, END TO END, for both robots: the glue's facts
//       (withTypedDriveFrom(calibrateFactsFor(..., the robot's words' name), ...), the program's
//       own two calls) carry the robot's name; Calibrate saves a record that names it; the SAME
//       robot's boot passes step 0 on those bytes and the other robot's refuses them
//                                              (the glue dropping the name — both robots would
//                                               save nameless records and refuse every one)

#include "doctest.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "calibrate_program_rig.hpp"
#include "common/boot_source.hpp"
#include "common/robot_words.hpp"
#include "mechanism_fixtures.hpp"
#include "retired_robot_two.hpp"
#include "robots/calibrate_facts.hpp"
#include "robots/chassis_table.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/calibration/calibrate_program.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/hal/fake/fake_block_source.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/mechanism/wiring.hpp"

#ifndef SHULIB_SOURCE_DIR
#error "SHULIB_SOURCE_DIR must be defined by the build (test/CMakeLists.txt)"
#endif

namespace cal = shulib::calibration;
namespace mech = shulib::mechanism;
using shulib::bench::calibrateFactsFor;
using shulib::bench::ChassisTable;
using shulib::bench::chassisTableFor;
using shulib::bench::MechanismTable;
using shulib::bench::mechanismTableFor;
using shulib::bench::RobotId;
using shulib::bench::tableToDescription;
using shulib::bench::withTypedDriveFrom;

namespace {

constexpr RobotId kEveryRobot[] = {RobotId::BenchBot, RobotId::RobotTwo, RobotId::InventedXDrive};

/// The facts' mechanism list as {port, name} pairs, up to the first unused entry — and the count of
/// entries USED anywhere (an entry after a gap would be a hole in the list).
struct Listed {
    std::vector<std::pair<int, std::string>> pairs;
    std::size_t usedAnywhere = 0;
};
Listed listed(const cal::CalibrateFacts& f) {
    Listed l;
    bool gap = false;
    for (const cal::MechanismPort& m : f.mechanisms) {
        if (m.port != 0 || m.name != nullptr) {
            ++l.usedAnywhere;
            if (!gap) {
                l.pairs.emplace_back(m.port, m.name != nullptr ? m.name : "<null>");
            }
        } else {
            gap = true;
        }
    }
    return l;
}

std::vector<int> typedSide(const std::int8_t (&side)[shulib::hal::kMaxSmartPorts]) {
    std::vector<int> v;
    for (const std::int8_t p : side) {
        if (p != 0) {
            v.push_back(p);
        }
    }
    return v;
}

std::vector<int> descSide(const shulib::hal::DriveSideDescription& s) {
    std::vector<int> v;
    for (std::size_t i = 0; i < s.count; ++i) {
        v.push_back(s.signedPorts[i]);
    }
    return v;
}

bool noTyped(const cal::CalibrateFacts& f) { return !cal::hasTypedDrive(f); }

/// The facts' typed sides against the description the boot would drive from, bit for bit: the
/// sides when it is valid(), nothing otherwise.
void checkTypedParity(const ChassisTable& t, const char* what) {
    CAPTURE(what);
    const cal::CalibrateFacts f = withTypedDriveFrom(cal::CalibrateFacts{}, t);
    const shulib::hal::DrivetrainDescription d = tableToDescription(t);
    if (d.valid()) {
        CHECK(typedSide(f.typedDrive.left) == descSide(d.left));
        CHECK(typedSide(f.typedDrive.right) == descSide(d.right));
        // bit for bit, slot by slot (no zero inside the list, nothing after it)
        for (std::size_t i = 0; i < shulib::hal::kMaxSmartPorts; ++i) {
            CHECK(f.typedDrive.left[i] == (i < d.left.count ? d.left.signedPorts[i] : 0));
            CHECK(f.typedDrive.right[i] == (i < d.right.count ? d.right.signedPorts[i] : 0));
        }
    } else {
        CHECK(noTyped(f));
    }
    CHECK(cal::factsProblem(f).why == nullptr);
}

std::string readRepo(const char* rel) {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/" + rel};
    REQUIRE_MESSAGE(in.good(), "cannot open ", rel);
    std::stringstream s;
    s << in.rdbuf();
    return s.str();
}

/// `src` with every comment removed and every run of whitespace squeezed to one space (string
/// literals kept whole) — so a pin matches however the line is wrapped, and a commented-out
/// statement does not count.
std::string codeOnly(const std::string& src) {
    std::string out;
    bool space = false;
    std::size_t i = 0;
    const auto put = [&](char c) {
        if (space && !out.empty()) {
            out.push_back(' ');
        }
        space = false;
        out.push_back(c);
    };
    while (i < src.size()) {
        const char c = src[i];
        const char n = i + 1 < src.size() ? src[i + 1] : '\0';
        if (c == '/' && n == '/') {
            while (i < src.size() && src[i] != '\n') {
                ++i;
            }
            space = true;
            continue;
        }
        if (c == '/' && n == '*') {
            i += 2;
            while (i + 1 < src.size() && !(src[i] == '*' && src[i + 1] == '/')) {
                ++i;
            }
            i += 2;
            space = true;
            continue;
        }
        if (c == '"') {
            put(c);
            ++i;
            while (i < src.size() && src[i] != '"') {
                if (src[i] == '\\' && i + 1 < src.size()) {
                    out.push_back(src[i++]);
                }
                out.push_back(src[i++]);
            }
            if (i < src.size()) {
                out.push_back(src[i++]);
            }
            continue;
        }
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            space = true;
            ++i;
            continue;
        }
        put(c);
        ++i;
    }
    return out;
}

/// The body of the ONE function whose signature `open` (ending in its "{") opens in `code` (codeOnly()'d
/// text): from that brace to its matching one, braces inside string literals skipped. Empty when the
/// signature is absent, appears more than once, or never closes.
std::string functionBody(const std::string& code, const std::string& open) {
    const std::size_t at = code.find(open);
    if (at == std::string::npos || code.find(open, at + 1) != std::string::npos) {
        return {};
    }
    const std::size_t start = at + open.size() - 1;
    int depth = 0;
    bool inString = false;
    for (std::size_t i = start; i < code.size(); ++i) {
        const char c = code[i];
        if (inString) {
            if (c == '\\') {
                ++i;
            } else if (c == '"') {
                inString = false;
            }
            continue;
        }
        if (c == '"') {
            inString = true;
        } else if (c == '{') {
            ++depth;
        } else if (c == '}' && --depth == 0) {
            return code.substr(start, i - start + 1);
        }
    }
    return {};
}

std::size_t occurrences(const std::string& hay, const std::string& needle) {
    std::size_t n = 0;
    for (std::size_t at = hay.find(needle); at != std::string::npos; at = hay.find(needle, at + 1)) {
        ++n;
    }
    return n;
}

}  // namespace

TEST_CASE("CalibrateFacts F1: the OLD robot two's derived facts (the claw robot, test/retired_robot_two.hpp) ARE the literal facts the goldens were made with") {
    const cal::CalibrateFacts f = withTypedDriveFrom(
        calibrateFactsFor(retired_robot_two::kChassis, retired_robot_two::kMechanisms),
        retired_robot_two::kChassis);
    REQUIRE(f.drive.has_value());
    CHECK(*f.drive == shulib::config::DrivetrainKind::Tank);
    REQUIRE(f.cartridge.has_value());
    CHECK(*f.cartridge == shulib::hal::MotorCartridge::Blue);
    const Listed l = listed(f);
    CHECK(l.pairs == std::vector<std::pair<int, std::string>>{{6, "claw pivot 6"}, {7, "claw pivot 7"}});
    CHECK(l.usedAnywhere == 2);
    // the names are the LIST's own storage, not copies (one fact, one place)
    CHECK(f.mechanisms[0].name == retired_robot_two::kMechanisms.specs[0].name);
    CHECK(f.mechanisms[1].name == retired_robot_two::kMechanisms.specs[1].name);
    REQUIRE(f.robot != nullptr);
    CHECK(std::string{f.robot} == "2026 TANK CHASSIS (robot two)");
    CHECK(f.robot == retired_robot_two::kChassis.robot);
    CHECK(f.imuPortStated == 2);
    CHECK(noTyped(f));  // its table is not a description the boot can drive from (F4)
    CHECK(cal::factsProblem(f).why == nullptr);
    // the build's own constant (the host is the bench build) passed its static_assert, and is the
    // bench bot's — the selection, not a robot named here
    CHECK(shulib::bench::kCalibrateFacts.robot == shulib::bench::kChassis.robot);
}

TEST_CASE("CalibrateFacts F2: one fact per MOTOR ROLE, in declaredMotorPorts()' order, named by its "
          "mechanism; a line or a sensor port is never a fact") {
    // robot y: a two-motor DR4B (3, 4), an angle sensor on 10, piece sensors on 9 and 8 (and a 3-wire
    // one, E), a pneumatic grip on line A
    const MechanismTable& y = mech_fixture::robot_y::kRobotYMechanisms;
    const cal::CalibrateFacts fy = calibrateFactsFor(retired_robot_two::kChassis, y);
    const Listed ly = listed(fy);
    CHECK(ly.pairs == std::vector<std::pair<int, std::string>>{{1, "wrist"}, {2, "swing"}, {3, "dr4b"}, {4, "dr4b"},
                                                                 {5, "tilt"}, {6, "roller"}, {7, "catapult"}});
    CHECK(ly.usedAnywhere == 7);
    // robot x: a two-motor lift (2, 3), a home switch and a pneumatic on lines — never facts
    const MechanismTable& x = mech_fixture::robot_x::kRobotXMechanisms;
    const Listed lx = listed(calibrateFactsFor(retired_robot_two::kChassis, x));
    CHECK(lx.pairs == std::vector<std::pair<int, std::string>>{{1, "arm"}, {2, "lift"}, {3, "lift"}, {4, "flywheel"},
                                                                 {5, "intake"}});
    // the ports are EXACTLY declaredMotorPorts()', and isDeclaredMechanismPort() agrees port by port,
    // for every robot and both fixtures
    std::vector<const MechanismTable*> lists = {&x, &y};
    for (const RobotId r : kEveryRobot) {
        lists.push_back(&mechanismTableFor(r));
    }
    for (const MechanismTable* m : lists) {
        const cal::CalibrateFacts f = calibrateFactsFor(retired_robot_two::kChassis, *m);
        int ports[32]{};
        const std::size_t n = mech::declaredMotorPorts(*m, std::span<int>{ports});
        const Listed l = listed(f);
        REQUIRE(l.pairs.size() == n);
        CHECK(l.usedAnywhere == n);
        for (std::size_t i = 0; i < n; ++i) {
            CAPTURE(i);
            CHECK(l.pairs[i].first == ports[i]);
        }
        for (int p = 0; p <= 22; ++p) {
            CAPTURE(p);
            CHECK((cal::declaredMechanism(f, p) != nullptr) == shulib::bench::isDeclaredMechanismPort(*m, p));
        }
        CHECK(cal::factsProblem(f).why == nullptr);
    }
    // robot two's pneumatic claw is on 3-wire line A: smart port 1 is not a mechanism
    const cal::CalibrateFacts two =
        calibrateFactsFor(retired_robot_two::kChassis, retired_robot_two::kMechanisms);
    CHECK(cal::declaredMechanism(two, 1) == nullptr);
    // a pneumatic-only list gives no fact at all
    const MechanismTable onlyClaw = [] {
        MechanismTable t = retired_robot_two::kMechanisms;
        t.specs[0] = t.specs[2];
        t.wiring[0] = t.wiring[2];
        t.count = 1;
        return t;
    }();
    CHECK(listed(calibrateFactsFor(retired_robot_two::kChassis, onlyClaw)).usedAnywhere == 0);
}

TEST_CASE("CalibrateFacts F6: mechanismFactsAre() -- the build's compile-time check that kCalibrateFacts lists "
          "EXACTLY its own list's motor ports -- holds for every right mapping and fails every wrong one") {
    using shulib::bench::mechanismFactsAre;
    const MechanismTable& two = retired_robot_two::kMechanisms;
    const MechanismTable& y = mech_fixture::robot_y::kRobotYMechanisms;
    const MechanismTable& x = mech_fixture::robot_x::kRobotXMechanisms;
    const MechanismTable none{};
    const ChassisTable& c = retired_robot_two::kChassis;
    // every right mapping: each robot's list and both fixtures, the empty list too
    std::vector<const MechanismTable*> lists = {&two, &x, &y, &none};
    for (const RobotId r : kEveryRobot) {
        lists.push_back(&mechanismTableFor(r));
    }
    for (const MechanismTable* m : lists) {
        CHECK(mechanismFactsAre(calibrateFactsFor(c, *m), *m));
    }
    // the review round's surviving mutant R1: facts built over NO list, checked against robot two's
    // (the static_assert's view on the tank-calibrate build) -- and the reverse
    CHECK_FALSE(mechanismFactsAre(calibrateFactsFor(c, none), two));
    CHECK_FALSE(mechanismFactsAre(calibrateFactsFor(c, two), none));
    // another robot's list: other ports (robot y has a 6 and a 7 too, at other entries)
    CHECK_FALSE(mechanismFactsAre(calibrateFactsFor(c, y), two));
    CHECK_FALSE(mechanismFactsAre(calibrateFactsFor(c, two), y));
    const cal::CalibrateFacts good = calibrateFactsFor(c, two);
    REQUIRE(mechanismFactsAre(good, two));
    {  // one port off
        cal::CalibrateFacts f = good;
        f.mechanisms[1].port = 8;
        CHECK_FALSE(mechanismFactsAre(f, two));
    }
    {  // the two in the other order: the same SET of ports, the wrong order
        cal::CalibrateFacts f = good;
        std::swap(f.mechanisms[0].port, f.mechanisms[1].port);
        CHECK_FALSE(mechanismFactsAre(f, two));
    }
    {  // one entry too many: the count is part of the claim (the entry just past the list)
        cal::CalibrateFacts f = good;
        f.mechanisms[2] = {9, two.specs[0].name};
        CHECK_FALSE(mechanismFactsAre(f, two));
    }
    {  // a name with no port past the list, and a stray port with no name: each still an entry
        cal::CalibrateFacts f = good;
        f.mechanisms[2].name = two.specs[0].name;
        CHECK_FALSE(mechanismFactsAre(f, two));
        f = good;
        f.mechanisms[cal::kMaxMechanismPorts - 1].port = 21;  // the LAST entry, the loop's edge
        CHECK_FALSE(mechanismFactsAre(f, two));
    }
    {  // one entry dropped
        cal::CalibrateFacts f = good;
        f.mechanisms[1] = {};
        CHECK_FALSE(mechanismFactsAre(f, two));
    }
    {  // a name that is not the list's own storage (an equal string elsewhere), and a null name
        static const char kCopy[] = "claw pivot 7";
        cal::CalibrateFacts f = good;
        f.mechanisms[1].name = kCopy;
        CHECK_FALSE(mechanismFactsAre(f, two));
        f.mechanisms[1].name = nullptr;
        CHECK_FALSE(mechanismFactsAre(f, two));
    }
    // and the build's own constant passed it against the build's own list (the host's: empty)
    CHECK(mechanismFactsAre(shulib::bench::kCalibrateFacts, shulib::bench::kMechanisms));
    static_assert(mechanismFactsAre(calibrateFactsFor(retired_robot_two::kChassis, two), two));
    static_assert(!mechanismFactsAre(calibrateFactsFor(retired_robot_two::kChassis, MechanismTable{}), two));
}

TEST_CASE("CalibrateFacts F3: the bench bot and the invented X-drive carry what their tables say") {
    const cal::CalibrateFacts bench =
        calibrateFactsFor(chassisTableFor(RobotId::BenchBot), mechanismTableFor(RobotId::BenchBot));
    REQUIRE(bench.drive.has_value());
    CHECK(*bench.drive == shulib::config::DrivetrainKind::Tank);
    REQUIRE(bench.cartridge.has_value());
    CHECK(*bench.cartridge == shulib::hal::MotorCartridge::Blue);
    CHECK(listed(bench).usedAnywhere == 0);
    CHECK(bench.imuPortStated == 4);
    const cal::CalibrateFacts xd =
        calibrateFactsFor(chassisTableFor(RobotId::InventedXDrive), mechanismTableFor(RobotId::InventedXDrive));
    CHECK_FALSE(xd.cartridge.has_value());  // UNSET stays unset: the program refuses it by name
    CHECK(listed(xd).usedAnywhere == 0);
    CHECK(xd.imuPortStated == 0);
    // every cartridge maps to its own, and UNSET to none
    CHECK(shulib::bench::cartridgeFact(shulib::bench::Cartridge::Red) == shulib::hal::MotorCartridge::Red);
    CHECK(shulib::bench::cartridgeFact(shulib::bench::Cartridge::Green) == shulib::hal::MotorCartridge::Green);
    CHECK(shulib::bench::cartridgeFact(shulib::bench::Cartridge::Blue) == shulib::hal::MotorCartridge::Blue);
    CHECK_FALSE(shulib::bench::cartridgeFact(shulib::bench::Cartridge::Unset).has_value());
    for (const RobotId r : kEveryRobot) {
        CAPTURE(static_cast<int>(r));
        CHECK(cal::factsProblem(calibrateFactsFor(chassisTableFor(r), mechanismTableFor(r))).why == nullptr);
    }
}

TEST_CASE("CalibrateFacts F4: the typed ports are the boot's own description when it is valid(), and none "
          "otherwise -- every robot today has none, each for its stated reason") {
    // Today: no robot's table is a description the boot can drive from. Each reason is asserted, so
    // a table that BECOMES valid turns this red — and whoever does it rewords the worksheet's Station G
    // robot-two sentence in the same change (the CAL1 brief §3.2).
    struct Today {
        RobotId robot;
        const char* why;
    };
    for (const Today& t : {Today{RobotId::RobotTwo, "IMU port 0"},  // the twin's: robot one's cartridge (2026-11-20), the rest UNSET
                           Today{RobotId::BenchBot, "LEFT side has no ports"},
                           Today{RobotId::InventedXDrive, "cartridge byte 0"}}) {
        CAPTURE(static_cast<int>(t.robot));
        const ChassisTable& c = chassisTableFor(t.robot);
        char why[160];
        CHECK_FALSE(tableToDescription(c).valid(why, sizeof why));
        CAPTURE(why);
        CHECK(std::strstr(why, t.why) != nullptr);
        const cal::CalibrateFacts f = withTypedDriveFrom(calibrateFactsFor(c, mechanismTableFor(t.robot)), c);
        CHECK(noTyped(f));
        checkTypedParity(c, "today's table");
    }
    // Robot two's table MADE valid (the track width and ratio typed): its signed ports, as typed.
    ChassisTable valid = retired_robot_two::kChassis;
    valid.trackWidthIn = 12.5;
    valid.externalRatio = 1.0;
    REQUIRE(tableToDescription(valid).valid());
    const cal::CalibrateFacts typed = withTypedDriveFrom(cal::CalibrateFacts{}, valid);
    CHECK(typedSide(typed.typedDrive.left) == std::vector<int>{-11, 12, -13, 14, -15});
    CHECK(typedSide(typed.typedDrive.right) == std::vector<int>{20, -19, 18, -17, 16});
    checkTypedParity(valid, "made valid");
    // The mapping keeps every other fact as it was handed in.
    const cal::CalibrateFacts base =
        calibrateFactsFor(retired_robot_two::kChassis, retired_robot_two::kMechanisms);
    const cal::CalibrateFacts withT = withTypedDriveFrom(base, valid);
    CHECK(listed(withT).pairs == listed(base).pairs);
    CHECK(withT.robot == base.robot);
    CHECK(withT.imuPortStated == base.imuPortStated);
    CHECK(withT.cartridge == base.cartridge);
    CHECK(withT.recordName == base.recordName);  // ROB1-BT: the record name too (F7 proves it end to end)
    CHECK(cal::factsProblem(withT).why == nullptr);
    // ... and a stale typed list handed in is REPLACED, never kept: an invalid table clears it.
    CHECK(noTyped(withTypedDriveFrom(withT, retired_robot_two::kChassis)));
    // Each fact undone again, one at a time: no typed port.
    struct Undo {
        const char* what;
        void (*apply)(ChassisTable&);
    };
    const Undo undos[] = {
        {"track width unset", [](ChassisTable& t) { t.trackWidthIn = 0.0; }},
        {"ratio unset", [](ChassisTable& t) { t.externalRatio = 0.0; }},
        {"wheel unset", [](ChassisTable& t) { t.wheelDiameterIn = 0.0; }},
        {"IMU unset", [](ChassisTable& t) { t.imuPort = 0; }},
        {"IMU on a drive port", [](ChassisTable& t) { t.imuPort = 12; }},
        {"cartridge unset", [](ChassisTable& t) { t.cartridge = shulib::bench::Cartridge::Unset; }},
        {"unsigned", [](ChassisTable& t) { t.signsMeasured = false; }},
        {"a port on both sides", [](ChassisTable& t) { t.right[0] = 11; }},
    };
    for (const Undo& u : undos) {
        CAPTURE(u.what);
        ChassisTable t = valid;
        u.apply(t);
        CHECK_FALSE(tableToDescription(t).valid());
        CHECK(noTyped(withTypedDriveFrom(cal::CalibrateFacts{}, t)));
        checkTypedParity(t, u.what);
    }
}

TEST_CASE("CalibrateFacts F5: the glue hands the standard Calibrate withTypedDriveFrom(kCalibrateFacts, "
          "kChassis), makes no PROS call of its own; main.cpp's Calibrate cases are its five") {
    const std::string glue = codeOnly(readRepo("src/programs/calibrate_program.cpp"));
    CHECK(occurrences(glue, "void runCalibrate() { const calibration::CalibrateFacts facts = "
                            "withTypedDriveFrom(kCalibrateFacts, kChassis); "
                            "hal::pros::runCalibrate(facts, kBuildStamp); }") == 1);
    CHECK(occurrences(glue, "kCalibrateFacts") == 1);  // never handed in bare anywhere else
    CHECK(occurrences(glue, "hal::pros::runCalibrate(") == 1);
    CHECK(occurrences(glue, "void calibrateInitialize(const char* variantBeacon, const char* ) "
                            "{ hal::pros::calibrateInitialize(variantBeacon, kBuildStamp); }") == 1);
    CHECK(occurrences(glue, "void calibrateDisabled() { hal::pros::calibrateDisabled(); }") == 1);
    CHECK(occurrences(glue, "void calibrateConnected() { hal::pros::calibrateConnected(); }") == 1);
    CHECK(occurrences(glue, "void calibrateAutonomous() { hal::pros::calibrateAutonomous(); }") == 1);
    // nothing of its own on the robot: every PROS name is the library shell's (hal::pros::), no file,
    // no print
    for (const char* banned : {"ProsBlockSink", "ProsBlockSource", "ProsFileStore", "ProsMotor", "printf",
                               "serialLine", "#include \"pros/", "#include \"main.h\""}) {
        CAPTURE(banned);
        CHECK(glue.find(banned) == std::string::npos);
    }
    CHECK(occurrences(glue, "pros::") == occurrences(glue, "hal::pros::"));
    CHECK(occurrences(glue, "#include \"robots/calibrate_facts.hpp\"") == 1);
    CHECK(occurrences(glue, "#include \"shulib/hal/pros/calibrate.hpp\"") == 1);

    // the build's facts are built over the build's OWN two tables (the one chain's selection) -- on
    // the host the mechanism list is empty, so this pin and mechanismFactsAre()'s static_assert on
    // the ARM builds (F6) are what see a facts constant built over any other list (review round, R1)
    const std::string facts = codeOnly(readRepo("src/robots/calibrate_facts.hpp"));
    CHECK(occurrences(facts, "inline constexpr calibration::CalibrateFacts kCalibrateFacts = "
                             "calibrateFactsFor(kChassis, kMechanisms, ThisRobotWords::value.recordName);") == 1);
    CHECK(occurrences(facts, "kCalibrateFacts =") == 1);
    CHECK(occurrences(facts, "static_assert(mechanismFactsAre(kCalibrateFacts, kMechanisms),") == 1);

    // src/main.cpp: Calibrate's five entries, each in ITS OWN PROS entry function (two swapped
    // between functions kept every name present -- the review round's R2 and R3), its three state
    // screens the library's, never benchStateScreen
    const std::string main = codeOnly(readRepo("src/main.cpp"));
    struct Entry {
        const char* function;
        const char* call;
    };
    const Entry entries[] = {
        {"void initialize() {", "shulib::bench::calibrateInitialize("},
        {"void disabled() {", "shulib::bench::calibrateDisabled();"},
        {"void competition_initialize() {", "shulib::bench::calibrateConnected();"},
        {"void autonomous() {", "shulib::bench::calibrateAutonomous();"},
        {"void opcontrol() {", "shulib::bench::runCalibrate();"},
    };
    for (const Entry& e : entries) {
        CAPTURE(e.function);
        const std::string body = functionBody(main, e.function);
        REQUIRE_FALSE(body.empty());
        CHECK(occurrences(body, std::string{"case Program::Calibrate: "} + e.call) == 1);
        CHECK(occurrences(main, e.call) == 1);  // and nowhere else in the file
        // no other Calibrate entry in this function: each function makes its one call
        for (const Entry& other : entries) {
            if (&other != &e) {
                CHECK(occurrences(body, other.call) == 0);
            }
        }
    }
    CHECK(occurrences(main, "case Program::Calibrate: shulib::bench::benchStateScreen(") == 0);
    CHECK(occurrences(main, "runCalibrate(") == 1);
}

TEST_CASE("CalibrateFacts F5b: functionBody() -- the brace matcher F5's main.cpp pins stand on") {
    const std::string src = codeOnly("void a() { if (x) { f(); } else { g(\"}\"); } } void b() { h(); }");
    CHECK(functionBody(src, "void a() {") == "{ if (x) { f(); } else { g(\"}\"); } }");
    CHECK(functionBody(src, "void b() {") == "{ h(); }");
    CHECK(functionBody(src, "void c() {").empty());                 // absent
    CHECK(functionBody(codeOnly("void a() { { }"), "void a() {").empty());  // never closed
    CHECK(functionBody(codeOnly("void a() {} void a() {}"), "void a() {").empty());  // twice: ambiguous
}

TEST_CASE("CalibrateFacts F-CFG: robot one's 10-motor drive and its twin carry the lowered pulse floor; the bench "
          "bot the calibrator's defaults; a floor a pulse's drift can reach is refused before anything is powered") {
    namespace sb = shulib::bench;
    const shulib::calibration::CalibrateFacts one = sb::calibrateFactsFor(sb::kRobotOneChassis, sb::robot_one::kRobotOneMechanisms);
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    CHECK(one.calibrator.pulseYawFloor.value() == doctest::Approx(0.6 * kDeg));
    CHECK(one.calibrator.restYawRateFloor.value() == doctest::Approx(1.0 * kDeg));
    CHECK(shulib::calibration::factsProblem(one).why == nullptr);
    const shulib::calibration::CalibrateFacts two = sb::calibrateFactsFor(sb::kRobotTwoChassis, sb::robot_two::kRobotTwoMechanisms);
    CHECK(two.calibrator.pulseYawFloor.value() == doctest::Approx(0.6 * kDeg));  // the twin: the same drive
    CHECK(two.calibrator.restYawRateFloor.value() == doctest::Approx(1.0 * kDeg));
    const shulib::calibration::CalibratorConfig d{};
    const shulib::calibration::CalibrateFacts bench = sb::calibrateFactsFor(sb::kBenchBotChassis, mechanismTableFor(RobotId::BenchBot));
    CHECK(bench.calibrator.pulseYawFloor.value() == doctest::Approx(d.pulseYawFloor.value()));
    CHECK(bench.calibrator.restYawRateFloor.value() == doctest::Approx(d.restYawRateFloor.value()));
    // 1.0 deg/s for a 0.4 s pulse is 0.4 deg of drift: a 0.3 deg floor could be crossed by drift alone
    shulib::calibration::CalibrateFacts bad = one;
    bad.calibrator.pulseYawFloor = shulib::units::AngleDim{0.3 * kDeg};
    REQUIRE(shulib::calibration::factsProblem(bad).why != nullptr);
    CHECK(std::string{shulib::calibration::factsProblem(bad).why}.find("pulse floor") != std::string::npos);
}

// ═══ F7 — THE RECORD NAME, END TO END (chunk ROB1-BT) ═══════════════════════════════════════════════
namespace {
/// Robot `R`'s Calibrate facts as its program makes them (the two calls of calibrate_program.cpp over
/// kCalibrateFacts' own expression), its saved record's bytes, and the first boot line `B`'s boot prints
/// over those bytes.
template <RobotId R>
cal::CalibrateFacts gluedFacts() {
    return withTypedDriveFrom(
        calibrateFactsFor(chassisTableFor(R), mechanismTableFor(R), shulib::bench::WordsOf<R>::value.recordName),
        chassisTableFor(R));
}
std::vector<std::byte> savedRecordNaming(const char* recordName) {
    cal_prog_rig::Scenario sc;
    cal_prog_rig::Rig rig{sc};
    cal::CalibrateFacts f = cal_prog_rig::robotTwoFacts();  // the golden rig's own robot (its ports, its claws)
    f.recordName = recordName;                              // ...saving the name the glue handed in
    rig.run(f);
    REQUIRE_FALSE(rig.timedOut);
    return rig.fileBytes(cal::kRecordFileName);
}
template <RobotId B>
std::string firstBootLine(const std::vector<std::byte>& bytes) {
    shulib::hal::fake::FakeBlockSource card{bytes};
    const shulib::bench::RecordRead record = shulib::bench::readRecord(card, "/usd/shulib_drivetrain.bin");
    REQUIRE(record.decoded);
    shulib::hal::fake::FakeDeviceRegistry census;
    std::vector<std::string> lines;
    (void)shulib::bench::decideBootSourceFrom(record, census, tableToDescription(chassisTableFor(B)), "x",
                                              mechanismTableFor(B), "robot",
                                              [&lines](const char* line) { lines.emplace_back(line); },
                                              shulib::bench::WordsOf<B>{});
    REQUIRE_FALSE(lines.empty());
    return lines.front();
}
}  // namespace

TEST_CASE("CalibrateFacts F7 (ROB1-BT): each robot's Calibrate saves a record naming THAT robot, its own boot "
          "passes step 0 on it, the other robot's refuses it") {
    const cal::CalibrateFacts one = gluedFacts<RobotId::RobotOne>();
    const cal::CalibrateFacts two = gluedFacts<RobotId::RobotTwo>();
    REQUIRE(one.recordName != nullptr);
    REQUIRE(two.recordName != nullptr);
    CHECK(std::string{one.recordName} == shulib::bench::robotRecordName(RobotId::RobotOne));
    CHECK(std::string{two.recordName} == shulib::bench::robotRecordName(RobotId::RobotTwo));
    CHECK(cal::factsProblem(one).why == nullptr);
    CHECK(cal::factsProblem(two).why == nullptr);
    const std::vector<std::byte> oneCard = savedRecordNaming(one.recordName);
    const std::vector<std::byte> twoCard = savedRecordNaming(two.recordName);
    // its own robot reaches step 1 (the port check) — step 0 passed
    CHECK(firstBootLine<RobotId::RobotOne>(oneCard).rfind("calibration record: DECODED, port check", 0) == 0);
    CHECK(firstBootLine<RobotId::RobotTwo>(twoCard).rfind("calibration record: DECODED, port check", 0) == 0);
    // the other robot refuses it by name
    CHECK(firstBootLine<RobotId::RobotTwo>(oneCard)
          == "calibration record: DECODED, REFUSED -- this card's record is robot=robot-one's; this is "
             "robot=robot-two -- run shulib Calibrate (slot 4) on THIS robot");
    CHECK(firstBootLine<RobotId::RobotOne>(twoCard)
          == "calibration record: DECODED, REFUSED -- this card's record is robot=robot-two's; this is "
             "robot=robot-one -- run shulib Calibrate (slot 4) on THIS robot");
}
