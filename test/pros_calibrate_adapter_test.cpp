// hal::pros's standard Calibrate shell (include/shulib/hal/pros/calibrate.hpp, chunk CAL1 Part A2a —
// the brief's §2.A test 8b) through the host PROS shim. NOTHING HERE HAS RUN ON A ROBOT.
//
// HONEST LIMIT (the shim's own, shim_control.hpp): these cases test the shell against our BELIEF
// about PROS — the vendored headers' C reads, competition calls and serial descriptor, transcribed
// into the shim — not the belief itself. The belief that a C read WRITES nothing to the motor is the
// one CAL1 registers as HA-187 (in Part A2b); hardware settles it.
//
// Each case names the bug it catches:
//   PC1 the WATCH's read: motor_get_position() turned into output-shaft degrees by the units
//       motor_get_encoder_units() reports — degrees, rotations, and counts by the gearing
//       motor_get_gearing() reports (red 1800, green 900, blue 300 a turn); false for a port that
//       does not answer (PROS_ERR_F), an unknown unit, or counts with no gearing — EACH of the
//       three guards reached ALONE (a dropout on one read while the other two answer: the shim's
//       failPositionReads / failUnitsReads / failGearingReads, the second review round) — and NOT
//       ONE write to any port, nor a motor object of any kind (the shim's gearing counter is what a
//       pros::Motor{port, gearset} would move)
//   PC2 line(): ONE write() per line on the serial descriptor, `[R3D] ` + the text + one newline —
//       a line past one serial line wraps into `[R3D] ... ` pieces, one write each (SER1); a null
//       text is an empty line, never a crash
//   PC3 competitionStatus() is PROS's byte; delayMs() is pros::delay()
//   PC4 calibrateInitialize(): the boot banner, verbatim, one write per line, constructing nothing
//   PC5 calibrateDisabled() / Connected() / Autonomous(): the state screen through the PROS
//       screen adapter (the name, the reason, no "cut" line — the library's words fit) and ONE
//       serial line each
//   PC6 runCalibrate(): the shell's own composition runs the library program over the real PROS
//       adapters — a refusal (the drive kind unset) painted and printed, every serial line one
//       tagged write, nothing asked of a motor
//   PC7 a source pin: the shell names no pros::Motor, pros::v5::Motor or ProsMotor in code, and its
//       line() is one serialWrapped() call — no printf, puts, fflush or std::cout

#include "doctest.h"

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/shim_control.hpp"
#include "shulib/calibration/calibrate_screen.hpp"
#include "shulib/hal/pros/calibrate.hpp"

namespace {

using shulib::hal::pros::ProsCalibrateIo;

/// fd 1 pointed at a SOCK_DGRAM socket for the object's life, restored by restore() or the
/// destructor: each write() on fd 1 becomes ONE datagram, so "one write per line" is countable.
/// (A copy of SER1's test helper, test/serial_line_test.cpp — that file is SER1's; no existing test
/// TU is edited.) The sending end is NON-BLOCKING: an over-full queue makes write() fail, which a
/// test sees as a missing line, never a hang. ASSERT NOTHING while it is live — doctest reports on
/// fd 1, into the socket; collect, restore, then CHECK.
class StdoutCapture {
public:
    StdoutCapture() {
        std::cout.flush();
        (void)std::fflush(stdout);
        if (::socketpair(AF_UNIX, SOCK_DGRAM, 0, sv_) != 0) {
            return;
        }
        const int flags = ::fcntl(sv_[0], F_GETFL);
        if (flags < 0 || ::fcntl(sv_[0], F_SETFL, flags | O_NONBLOCK) != 0) {
            return;
        }
        saved_ = ::dup(STDOUT_FILENO);
        ok_ = saved_ >= 0 && ::dup2(sv_[0], STDOUT_FILENO) == STDOUT_FILENO;
    }
    StdoutCapture(const StdoutCapture&) = delete;
    StdoutCapture& operator=(const StdoutCapture&) = delete;
    ~StdoutCapture() {
        restore();
        for (const int fd : sv_) {
            if (fd >= 0) {
                ::close(fd);
            }
        }
    }
    [[nodiscard]] bool ok() const { return ok_; }
    void restore() {
        if (saved_ >= 0) {
            (void)::dup2(saved_, STDOUT_FILENO);
            ::close(saved_);
            saved_ = -1;
        }
    }
    [[nodiscard]] std::vector<std::string> datagrams() const {
        std::vector<std::string> out;
        std::vector<char> buf(65536);
        for (;;) {
            const ssize_t n = ::recv(sv_[1], buf.data(), buf.size(), MSG_DONTWAIT);
            if (n < 0) {
                break;
            }
            out.emplace_back(buf.data(), static_cast<std::size_t>(n));
        }
        return out;
    }

private:
    int sv_[2] = {-1, -1};
    int saved_ = -1;
    bool ok_ = false;
};

/// What `fn` wrote on fd 1, one string per write() call; `ok` false if fd 1 could not be captured
/// (the case then FAILS: a capture that saw nothing proves nothing).
struct Captured {
    bool ok = false;
    std::vector<std::string> writes;
};
template <class Fn>
Captured capture(Fn&& fn) {
    Captured c;
    StdoutCapture cap;
    c.ok = cap.ok();
    if (c.ok) {
        fn();
        cap.restore();
        c.writes = cap.datagrams();
    }
    return c;
}

/// Every write is ONE tagged line: `[R3D] `, the text, exactly one newline, at the end.
void checkTaggedLines(const Captured& c) {
    REQUIRE(c.ok);
    for (const std::string& w : c.writes) {
        CAPTURE(w);
        CHECK(w.rfind("[R3D] ", 0) == 0);
        CHECK(w.find('\n') == w.size() - 1);
        CHECK(w.size() <= shulib::hal::pros::kSerialLineBytes);
    }
}

/// The shell's state before each case: every shim device reset, the C-read counters and the
/// screen's record too (resetAll() does not reach those: A1's hand-over, shim_control.hpp).
void freshShim() {
    pros::shim::resetAll();
    pros::shim::resetMotorCReads();
    pros::shim::resetScreen();
    pros::shim::resetCompetition();
}

/// Every write counter of every motor port is 0 (no motor configured, commanded or braked).
void checkNoMotorWrite() {
    for (int p = 1; p <= 21; ++p) {
        CAPTURE(p);
        const auto& s = pros::shim::motorState(p);
        CHECK(s.setGearingCalls == 0);
        CHECK(s.setEncoderUnitsCalls == 0);
        CHECK(s.moveVoltageCalls == 0);
        CHECK(s.setBrakeModeCalls == 0);
        CHECK(s.brakeCalls == 0);
    }
}

bool drewOnShim(const std::string& part) {
    for (const auto& l : pros::shim::screenState().log) {
        if (l.find(part) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("ProsCalibrateIo PC1: the watch's read -- degrees from the units and gearing the motor reports, "
          "false when it cannot be read, and NOT ONE write") {
    freshShim();
    ProsCalibrateIo io;
    auto& m = pros::shim::motorState(6);
    m.positionOutputDeg = 90.0;
    double deg = -1.0;
    // degrees: the reading as it is
    m.units = pros::MotorUnits::degrees;
    REQUIRE(io.readShaftDegrees(6, deg));
    CHECK(deg == doctest::Approx(90.0));
    CHECK(pros::shim::motorCReads()[6].position == 1);
    CHECK(pros::shim::motorCReads()[6].units == 1);
    CHECK(pros::shim::motorCReads()[6].gearing == 0);  // degrees need no gearing
    // rotations: × 360
    m.units = pros::MotorUnits::rotations;
    REQUIRE(io.readShaftDegrees(6, deg));
    CHECK(deg == doctest::Approx(90.0));
    // counts, by the REPORTED gearing — each cartridge, so a fixed 900 a turn cannot pass
    m.units = pros::MotorUnits::counts;
    for (const auto g : {pros::MotorGears::red, pros::MotorGears::green, pros::MotorGears::blue}) {
        CAPTURE(static_cast<int>(g));
        m.gearing = g;
        deg = -1.0;
        REQUIRE(io.readShaftDegrees(6, deg));
        CHECK(deg == doctest::Approx(90.0));
    }
    CHECK(pros::shim::motorCReads()[6].gearing == 3);
    // a different angle, so a read that ignored the device could not pass
    m.positionOutputDeg = -1692.4;
    m.gearing = pros::MotorGears::blue;
    REQUIRE(io.readShaftDegrees(6, deg));
    CHECK(deg == doctest::Approx(-1692.4));
    // cannot be read: false, and `degrees` is not trusted
    m.gearing = pros::MotorGears::invalid;
    CHECK_FALSE(io.readShaftDegrees(6, deg));  // counts with no gearing: PROS_ERR_F
    m.gearing = pros::MotorGears::blue;
    m.units = pros::MotorUnits::invalid;
    CHECK_FALSE(io.readShaftDegrees(6, deg));
    m.units = pros::MotorUnits::degrees;
    m.disconnected = true;
    CHECK_FALSE(io.readShaftDegrees(6, deg));  // no motor answering
    m.disconnected = false;
    // Each of the shell's three guards ALONE (the review round: the three cases above all stop at
    // the first guard — the shim's position read fails in each — so any one guard deleted passed).
    // A dropout on ONE read while the other two answer, each consumed by that one call:
    //   (1) the position read fails while the units read DEGREES: the PROS_ERR_F guard, alone
    m.units = pros::MotorUnits::degrees;
    m.failPositionReads = 1;
    CHECK_FALSE(io.readShaftDegrees(6, deg));
    CHECK(m.failPositionReads == 0);
    //   (2) a finite count read whose gearing read fails: the "no counts per turn" guard, alone
    m.units = pros::MotorUnits::counts;
    m.gearing = pros::MotorGears::blue;
    m.failGearingReads = 1;
    CHECK_FALSE(io.readShaftDegrees(6, deg));
    CHECK(m.failGearingReads == 0);
    //   (3) a finite reading whose units read fails: the unknown-unit guard, alone
    m.units = pros::MotorUnits::degrees;
    m.failUnitsReads = 1;
    CHECK_FALSE(io.readShaftDegrees(6, deg));
    CHECK(m.failUnitsReads == 0);
    // ... and each dropout was ONE read: the next read of the same port answers
    deg = 0.0;
    REQUIRE(io.readShaftDegrees(6, deg));
    CHECK(deg == doctest::Approx(-1692.4));
    // NOT ONE write, anywhere — no motor object configured a gearing, a unit, a voltage or a brake
    checkNoMotorWrite();
    CHECK(m.positionOutputDeg == -1692.4);
}

TEST_CASE("ProsCalibrateIo PC2: line() is ONE tagged write per line; a long line wraps into tagged pieces, "
          "one write each; a null text is an empty line") {
    freshShim();
    ProsCalibrateIo io;
    const std::string longText(400, 'w');
    const Captured c = capture([&io, &longText] {
        io.line("front check: SAME FRONT");
        io.line(nullptr);
        io.line("");
        io.line(longText.c_str());
    });
    checkTaggedLines(c);
    REQUIRE(c.writes.size() >= 5);
    CHECK(c.writes[0] == "[R3D] front check: SAME FRONT\n");
    CHECK(c.writes[1] == "[R3D] \n");
    CHECK(c.writes[2] == "[R3D] \n");
    // the long line: pieces, the later ones "[R3D] ... ", and every 'w' carried exactly once
    std::string carried;
    for (std::size_t i = 3; i < c.writes.size(); ++i) {
        const std::string head = i == 3 ? "[R3D] " : "[R3D] ... ";
        CAPTURE(i);
        REQUIRE(c.writes[i].rfind(head, 0) == 0);
        carried += c.writes[i].substr(head.size(), c.writes[i].size() - head.size() - 1);
    }
    CHECK(carried == longText);
    CHECK(c.writes.size() == 5);  // 400 characters: two pieces of at most 256 bytes
}

TEST_CASE("ProsCalibrateIo PC3: competitionStatus() is PROS's byte; delayMs() waits on PROS's clock") {
    freshShim();
    ProsCalibrateIo io;
    pros::shim::competitionStatus() = COMPETITION_DISABLED | COMPETITION_CONNECTED;
    CHECK(io.competitionStatus()
          == (shulib::calibration::kCompetitionDisabled | shulib::calibration::kCompetitionConnected));
    pros::shim::competitionStatus() = COMPETITION_AUTONOMOUS;
    CHECK(io.competitionStatus() == shulib::calibration::kCompetitionAutonomous);
    const std::uint32_t t0 = pros::millis();
    io.delayMs(250);
    CHECK(pros::millis() - t0 == 250);
}

TEST_CASE("hal::pros::calibrateInitialize PC4: the boot banner, verbatim, one tagged write per line, and "
          "nothing constructed") {
    freshShim();
    pros::shim::competitionStatus() = COMPETITION_CONNECTED;
    const Captured c = capture(
        [] { shulib::hal::pros::calibrateInitialize("shulib-robot-variant=tank-calibrate", "Oct 22 2026 01:02:03"); });
    checkTaggedLines(c);
    const std::vector<std::string> want = {
        "[R3D] booted: shulib CALIBRATE build [shulib-robot-variant=tank-calibrate].\n",
        "[R3D] the robot MEASURES ITS OWN DRIVETRAIN -- which ports drive, which way each\n",
        "[R3D] is wired, which side each is on, inches per radian per side, and the\n",
        "[R3D] EFFECTIVE track width -- then writes them to /usd/shulib_drivetrain.bin (a verified backup "
        "shulib_drivetrain.bak first).\n",
        "[R3D] It POWERS THE MOTORS. The end that LEADS the push is the FRONT. Every threshold\n",
        "[R3D] in it is INVENTED (A4 register HA-136).\n",
        "[R3D] nothing is constructed at boot: the motors are found when driver control starts.\n",
        "[R3D] BUILD STAMP Oct 22 2026 01:02:03\n",
        "[R3D] competition status at boot: 0x04 field=yes disabled=no auton=no -- calibration runs only in "
        "DRIVER CONTROL, enabled; any other state stops the run and says so.\n",
    };
    CHECK(c.writes == want);
    // no label: said so
    const Captured n = capture([] { shulib::hal::pros::calibrateInitialize(nullptr, "x"); });
    REQUIRE(n.ok);
    REQUIRE_FALSE(n.writes.empty());
    CHECK(n.writes[0] == "[R3D] booted: shulib CALIBRATE build [no label].\n");
    checkNoMotorWrite();
    CHECK(pros::shim::screenState().log.empty());  // the banner paints nothing
}

TEST_CASE("hal::pros::calibrate state screens PC5: DISABLED, FIELD CONNECTED and AUTONOMOUS -- painted "
          "whole through the PROS screen adapter, and ONE serial line each") {
    struct Case {
        void (*fn)();
        std::uint8_t status;
        const char* name;
        const char* line;
    };
    const Case cases[] = {
        {&shulib::hal::pros::calibrateDisabled, COMPETITION_DISABLED, "DISABLED",
         "[R3D] competition state: DISABLED (0x01 field=no disabled=yes auton=no) -- VEXos reports the robot "
         "DISABLED, so the driver task is not running.\n"},
        {&shulib::hal::pros::calibrateConnected, COMPETITION_CONNECTED | COMPETITION_DISABLED, "FIELD CONNECTED",
         "[R3D] competition state: FIELD CONNECTED (0x05 field=yes disabled=yes auton=no) -- A competition "
         "switch or field is connected through the controller.\n"},
        {&shulib::hal::pros::calibrateAutonomous, COMPETITION_AUTONOMOUS, "AUTONOMOUS",
         "[R3D] competition state: AUTONOMOUS (0x02 field=no disabled=no auton=yes) -- No motion here, ever: "
         "calibration powers the motors only while a person stands by the battery.\n"},
    };
    for (const Case& k : cases) {
        CAPTURE(std::string{k.name});
        freshShim();
        pros::shim::competitionStatus() = k.status;
        const Captured c = capture([&k] { k.fn(); });
        checkTaggedLines(c);
        REQUIRE(c.writes.size() == 1);
        CHECK(c.writes[0] == k.line);
        CHECK(drewOnShim("'" + std::string{k.name}));
        CHECK_FALSE(drewOnShim("the full text is on the serial log"));  // the library's words fit
        checkNoMotorWrite();
    }
}

TEST_CASE("hal::pros::runCalibrate PC6: the shell's own composition runs the library program over the real "
          "PROS adapters -- a refusal painted and printed, every line one tagged write, no motor asked "
          "for") {
    freshShim();
    pros::shim::setPluggedAtPort(11, pros::c::E_DEVICE_MOTOR);
    pros::shim::setPluggedAtPort(12, pros::c::E_DEVICE_MOTOR);
    shulib::calibration::CalibrateFacts facts{};
    facts.cartridge = shulib::hal::MotorCartridge::Blue;  // the drive kind left UNSET
    const Captured c = capture([&facts] { shulib::hal::pros::runCalibrate(facts, "Oct 22 2026 01:02:03"); });
    checkTaggedLines(c);
    auto has = [&c](const std::string& line) {
        for (const auto& w : c.writes) {
            if (w == line) {
                return true;
            }
        }
        return false;
    };
    CHECK(has("[R3D] BUILD STAMP: Oct 22 2026 01:02:03   <-- must match the build you just uploaded\n"));
    CHECK(has("[R3D] card       : NO SD CARD in the brain -- nothing can be saved, and the FRONT is not checked\n"));
    CHECK(has("[R3D]     11  MOTOR\n"));
    CHECK(has("[R3D]   2 motor(s), 0 IMU(s)\n"));
    CHECK(has("[R3D] REFUSED: DRIVE KIND UNSET -- the drive kind is not stated\n"));
    REQUIRE_FALSE(c.writes.empty());
    CHECK(c.writes.back() == "[R3D]   state it in the robot's facts (Tank), rebuild, upload\n");
    CHECK(drewOnShim("'DRIVE KIND UNSET'"));
    CHECK(drewOnShim("'CANNOT START"));
    checkNoMotorWrite();  // refused before the provider was asked for anything
}

TEST_CASE("hal/pros/calibrate.hpp PC7: a source pin -- no motor object in code, and line() is one "
          "serialWrapped() call with no stdio printer") {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/include/shulib/hal/pros/calibrate.hpp"};
    REQUIRE(in.good());
    const std::string src((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::string code;  // // comments dropped
    for (std::size_t i = 0; i < src.size(); ++i) {
        if (src[i] == '/' && i + 1 < src.size() && src[i + 1] == '/') {
            while (i < src.size() && src[i] != '\n') {
                ++i;
            }
        }
        if (i < src.size()) {
            code += src[i];
        }
    }
    auto count = [&code](const std::string& s) {
        int n = 0;
        for (std::size_t at = code.find(s); at != std::string::npos; at = code.find(s, at + 1)) {
            ++n;
        }
        return n;
    };
    CHECK(count("pros::Motor") == 0);
    CHECK(count("v5::Motor") == 0);
    CHECK(count("ProsMotor") == 0);
    CHECK(count("Motor{") == 0);
    CHECK(count("Motor(") == 0);
    CHECK(count("serialWrapped(\"R3D\"") == 1);
    CHECK(count("printf") == 0);
    CHECK(count("puts(") == 0);
    CHECK(count("fflush") == 0);
    CHECK(count("std::cout") == 0);
    // the watch's three reads are the C API's, by port
    CHECK(count("::pros::c::motor_get_position(") == 1);
    CHECK(count("::pros::c::motor_get_encoder_units(") == 1);
    CHECK(count("::pros::c::motor_get_gearing(") == 1);
}
