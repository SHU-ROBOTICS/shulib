// Our robot's Calibrate. We state only what the robot cannot measure about itself;
// shulib Calibrate finds everything else (which ports drive, their directions, the sides,
// the FRONT -- the end you push first -- the track width and the wheel scale).
#include "main.h"
#include "shulib/hal/pros/calibrate.hpp"

using namespace shulib;

constexpr calibration::CalibrateFacts kOurRobot{
    .drive = config::DrivetrainKind::Tank,
    .cartridge = hal::MotorCartridge::Blue,  // the insert colour of the DRIVE motors
    // EVERY motor that is not a drive motor. A motor left out is a drive candidate,
    // and one the push turns gets powered (3, 6, 9 V) like a drive motor.
    .mechanisms = {{6, "left claw"}, {7, "right claw"}},
};

void initialize() { hal::pros::calibrateInitialize(); }
void disabled() { hal::pros::calibrateDisabled(); }
void competition_initialize() { hal::pros::calibrateConnected(); }
void autonomous() { hal::pros::calibrateAutonomous(); }
void opcontrol() { hal::pros::runCalibrate(kOurRobot); }

// ═══════════════════════════════════════════════════════════════════════════════════════
// calibrate_team_file_example_test.cpp — THE TEAM'S CALIBRATE, the 21 lines above VERBATIM (chunk
// CAL1, Part A2b, 2026-10-30; the CAL1 brief's §1.1 and §2.A test 11). They are a whole PROS program
// file: its `#include "main.h"` resolves, on the host, to test/pros_shim/main.h (PROS's five entry
// points with C linkage, exactly as the project's include/main.h declares them), and its five
// functions are DEFINED here — no other translation unit of the suite defines them. The guide's
// chapter "Calibrating your robot" quotes them, and `api_doc_tool.py check-examples` holds every
// quoted line to a line of THIS file (its name matches test/*example*_test.cpp): a chapter line that
// drifts from the compiled file fails the build.
//
// Below the program, the cases: the facts are usable (factsProblem() at compile time and at run time)
// and read back as stated, with NO typed drive ports (the file is unchanged by the team lead's D6
// (a)); the chapter's two OPTIONAL typed-drive-port spellings — the one line in a chassis table's own
// spelling, and typedDrivePortsFrom() over a real `inline constexpr config::RobotConfig` tank — each
// in its own facts, each usable and each the same ports; and the five entry points RUN through the
// host PROS shim: initialize() prints the boot banner, the three state entries paint the
// competition-state screen, and opcontrol() runs the library's program — on a brain with nothing
// plugged in, a refusal before anything is made. Nothing here has run on a robot.

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#include "doctest.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/shim_control.hpp"
#include "shulib/config/robot_config.hpp"

namespace {

// The chapter's typed-drive-port spellings, each in facts of its own (the team file above is
// unchanged by them). First, the one line, in a chassis table's own spelling:
constexpr calibration::CalibrateFacts kOurRobotTyped{
    .drive = config::DrivetrainKind::Tank,
    .cartridge = hal::MotorCartridge::Blue,  // the insert colour of the DRIVE motors
    .mechanisms = {{6, "left claw"}, {7, "right claw"}},
    .typedDrive = {.left = {-11, 12, -13, 14, -15}, .right = {20, -19, 18, -17, 16}},
};

// Second, from a team's drive program typed as a config::RobotConfig tank (the `kRobot` must be a
// constant expression, because the facts are `constexpr`):
inline constexpr config::RobotConfig kRobot{
    .identity = {.name = "our tank"},
    .drivetrain = {.kind = config::DrivetrainKind::Tank,
                   .tank = {.description = {.kind = hal::DrivetrainKind::Tank,
                                            .cartridge = hal::MotorCartridge::Blue,
                                            .imuPort = 2,
                                            .left = {.signedPorts = {-11, 12, -13, 14, -15},
                                                     .count = 5,
                                                     .inchesPerRadian = units::Length{1.625}},
                                            .right = {.signedPorts = {20, -19, 18, -17, 16},
                                                      .count = 5,
                                                      .inchesPerRadian = units::Length{1.625}},
                                            .trackWidth = units::Length{12.5}}}},
};
constexpr calibration::CalibrateFacts kOurRobotFromConfig{
    .drive = config::DrivetrainKind::Tank,
    .cartridge = hal::MotorCartridge::Blue,  // the insert colour of the DRIVE motors
    .mechanisms = {{6, "left claw"}, {7, "right claw"}},
    .typedDrive = calibration::typedDrivePortsFrom(kRobot.drivetrain.tank.description),
};

// The facts, at COMPILE time — what a team's own static_assert would hold.
static_assert(calibration::factsProblem(kOurRobot).why == nullptr);
static_assert(!calibration::hasTypedDrive(kOurRobot));  // the 21 lines state no typed drive ports
static_assert(calibration::factsProblem(kOurRobotTyped).why == nullptr);
static_assert(calibration::hasTypedDrive(kOurRobotTyped));
static_assert(calibration::factsProblem(kOurRobotFromConfig).why == nullptr);
static_assert(calibration::hasTypedDrive(kOurRobotFromConfig));

/// fd 1 into a file for the life of the object (the shell's serial line is ONE write() on fd 1);
/// ASSERT NOTHING while it is live — doctest reports on fd 1. `text()` after `restore()`.
class StdoutToFile {
public:
    StdoutToFile()
        : path_{(std::filesystem::temp_directory_path()
                 / ("shulib_team_file_" + std::to_string(::getpid()) + ".out"))
                    .string()} {
        std::cout.flush();
        (void)std::fflush(stdout);
        const int fd = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            return;
        }
        saved_ = ::dup(STDOUT_FILENO);
        ok_ = saved_ >= 0 && ::dup2(fd, STDOUT_FILENO) == STDOUT_FILENO;
        ::close(fd);
    }
    StdoutToFile(const StdoutToFile&) = delete;
    StdoutToFile& operator=(const StdoutToFile&) = delete;
    ~StdoutToFile() {
        restore();
        std::filesystem::remove(path_);
    }
    [[nodiscard]] bool ok() const { return ok_; }
    void restore() {
        if (saved_ >= 0) {
            (void)::dup2(saved_, STDOUT_FILENO);
            ::close(saved_);
            saved_ = -1;
        }
    }
    [[nodiscard]] std::string text() const {
        std::ifstream f{path_, std::ios::binary};
        return std::string{std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
    }

private:
    std::string path_;
    int saved_ = -1;
    bool ok_ = false;
};

void resetShim() {
    pros::shim::resetAll();
    pros::shim::resetScreen();
    pros::shim::resetCompetition();
    pros::shim::resetMotorCReads();
}

bool drew(const std::string& part) {
    for (const auto& l : pros::shim::screenState().log) {
        if (l.find(part) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("team file: the 21 lines' facts are usable and read back as stated, with NO typed drive ports") {
    const calibration::FactsProblem p = calibration::factsProblem(kOurRobot);
    CHECK(p.why == nullptr);
    REQUIRE(kOurRobot.drive.has_value());
    CHECK(*kOurRobot.drive == config::DrivetrainKind::Tank);
    REQUIRE(kOurRobot.cartridge.has_value());
    CHECK(*kOurRobot.cartridge == hal::MotorCartridge::Blue);
    CHECK(kOurRobot.mechanisms[0].port == 6);
    CHECK(std::string{kOurRobot.mechanisms[0].name} == "left claw");
    CHECK(kOurRobot.mechanisms[1].port == 7);
    CHECK(std::string{kOurRobot.mechanisms[1].name} == "right claw");
    for (std::size_t i = 2; i < calibration::kMaxMechanismPorts; ++i) {
        CAPTURE(i);
        CHECK(kOurRobot.mechanisms[i].port == 0);
        CHECK(kOurRobot.mechanisms[i].name == nullptr);
    }
    CHECK(kOurRobot.robot == nullptr);       // the optional name, not stated
    CHECK(kOurRobot.imuPortStated == 0);     // the optional cross-check, not stated
    CHECK_FALSE(calibration::hasTypedDrive(kOurRobot));
}

TEST_CASE("team file: the chapter's two typed-drive-port spellings -- the one line and typedDrivePortsFrom() "
          "over a real RobotConfig tank -- are usable and state the SAME ports, in order") {
    const hal::DrivetrainDescription a = calibration::typedDriveDescription(kOurRobotTyped);
    const hal::DrivetrainDescription b = calibration::typedDriveDescription(kOurRobotFromConfig);
    const int left[] = {-11, 12, -13, 14, -15};
    const int right[] = {20, -19, 18, -17, 16};
    REQUIRE(a.left.count == 5);
    REQUIRE(a.right.count == 5);
    REQUIRE(b.left.count == 5);
    REQUIRE(b.right.count == 5);
    for (std::size_t i = 0; i < 5; ++i) {
        CAPTURE(i);
        CHECK(a.left.signedPorts[i] == left[i]);
        CHECK(a.right.signedPorts[i] == right[i]);
        CHECK(b.left.signedPorts[i] == left[i]);
        CHECK(b.right.signedPorts[i] == right[i]);
    }
    // kRobot is a real, complete tank config — not a bare description
    char why[160] = "";
    const bool valid = kRobot.valid(why, sizeof why);
    INFO("why: " << why);
    CHECK(valid);
    // everything else in the typed facts is the 21 lines' own
    CHECK(kOurRobotTyped.mechanisms[0].port == kOurRobot.mechanisms[0].port);
    CHECK(kOurRobotFromConfig.mechanisms[1].port == kOurRobot.mechanisms[1].port);
    CHECK(*kOurRobotFromConfig.cartridge == *kOurRobot.cartridge);
}

TEST_CASE("team file: the five entry points RUN through the host PROS shim -- the banner, the three state "
          "screens, and opcontrol()'s library program, which refuses on a brain with nothing plugged in") {
    resetShim();
    std::string banner;
    std::string states;
    std::string run;
    bool ok = true;
    {
        StdoutToFile out;
        ok = ok && out.ok();
        initialize();
        out.restore();
        banner = out.text();
    }
    {
        StdoutToFile out;
        ok = ok && out.ok();
        disabled();
        competition_initialize();
        autonomous();
        out.restore();
        states = out.text();
    }
    {
        resetShim();  // no SD card (the shim's default): nothing on this machine is read or written
        StdoutToFile out;
        ok = ok && out.ok();
        opcontrol();  // nothing plugged in: refused before anything is made, and it returns
        out.restore();
        run = out.text();
    }
    REQUIRE(ok);
    // initialize(): the library's boot banner, under its default label
    CHECK(banner.find("[R3D] booted: shulib CALIBRATE build [no label].\n") == 0);
    // the three state entries: one line each, their state's name
    CHECK(states.find("[R3D] competition state: DISABLED") != std::string::npos);
    CHECK(states.find("[R3D] competition state: FIELD CONNECTED") != std::string::npos);
    CHECK(states.find("[R3D] competition state: AUTONOMOUS") != std::string::npos);
    // opcontrol(): the standard Calibrate itself — the census of an empty brain, then its refusal
    CHECK(run.find("[R3D] #  shulib CALIBRATE -- the robot MEASURES ITS OWN DRIVETRAIN\n") != std::string::npos);
    CHECK(run.find("[R3D]   0 motor(s), 0 IMU(s)\n") != std::string::npos);
    CHECK(run.find("[R3D] REFUSED: ") != std::string::npos);
    CHECK(run.find("the registry reports 0 IMUs; this procedure needs exactly one") != std::string::npos);
    // ONE run of the program (CAL1-A2b's review, 2026-10-30: finding each line passed a runCalibrate()
    // that ran the program twice): its banner once, its refusal once, and nothing printed after it
    // but the refusal's fix line
    const auto occurrences = [&run](const std::string& part) {
        int n = 0;
        for (auto at = run.find(part); at != std::string::npos; at = run.find(part, at + part.size())) {
            ++n;
        }
        return n;
    };
    CHECK(occurrences("#  shulib CALIBRATE -- the robot MEASURES ITS OWN DRIVETRAIN\n") == 1);
    CHECK(occurrences("[R3D] REFUSED: ") == 1);
    const auto refused = run.find("[R3D] REFUSED: ");
    REQUIRE(refused != std::string::npos);
    const auto fixLine = run.find('\n', refused);
    REQUIRE(fixLine != std::string::npos);
    CHECK(run.find('\n', fixLine + 1) == run.size() - 1);  // the fix line is the run's last
    CHECK(drew("'CANNOT START"));
    // nothing was asked of a motor on any port
    for (int p = 1; p <= 21; ++p) {
        CAPTURE(p);
        CHECK(pros::shim::motorState(p).setGearingCalls == 0);
        CHECK(pros::shim::motorState(p).moveVoltageCalls == 0);
    }
    resetShim();
}
