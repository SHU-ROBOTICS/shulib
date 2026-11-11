#pragma once
//
// calibrate_golden_rig.hpp — the standard Calibrate RUN THROUGH ITS REAL PROS SHELL on the A2 host
// test plant (chunk CAL1, Part A2b, 2026-10-30; the CAL1 brief's §2.A test 10 — its probe harness,
// brief-CAL1 d6a/v3/harness4.cpp, made project code). test/calibrate_golden_test.cpp holds the
// scenarios and the goldens they must equal; this file is the machine they run on.
//
// ── What runs, and through what ─────────────────────────────────────────────────────
// calibration::CalibrateProgram composed EXACTLY as hal::pros::runCalibrate() composes it — the
// PROS shell's own ProsDeviceRegistry, ProsDeviceProvider over it, ProsBrainScreen and
// ProsCalibrateIo — except that the SD card is ProsFileStore{<a directory>} (the adapter's own
// host-test constructor) instead of the brain's /usd/. Before it, hal::pros::calibrateInitialize()
// prints the boot banner, as PROS calls initialize() before opcontrol(). Every motor, the IMU, the
// registry, the card's files, the screen, the touch, the competition status and the clock are the
// host PROS SHIM's (test/pros_shim/), so the shell's adapters run as they would on the brain — over
// our BELIEF about PROS, which the shim transcribes and hardware settles (shim_control.hpp).
//
// ── The robot ───────────────────────────────────────────────────────────────────────
// Robot two's shape on the plant (the probe's wire(), and test/calibrate_program_rig.hpp's): two
// wheels of FIVE coupled members — LEFT 11..15 (11 the primary), RIGHT 16..20 (16 the primary),
// members 11, 13, 15, 17 and 19 wired backwards; two claw pivots on 6 and 7 the plant never moves
// (unless a scenario turns one); the IMU on 2, ready 2 s after its reset; the radio on 1. The world
// is stepped 10 ms at a time from the shim clock's ADVANCE HOOK — every pros::delay() and every
// Task::delay_until() the program's pacer makes — so the plant moves exactly as the program's own
// waits let time pass. The ends are named by the PLANT: pushDir +1 leads with the end the plant
// drives forward (the end robot two's TYPED table calls front; robot two's real camera end is the
// other — the CAL1 brief §8).
//
// ── The operator reads the SCREEN, never the program ────────────────────────────────
// It learns what is drawn from the shim screen's draw log as each draw happens (the header word at
// (8, 4); each button rectangle's latest fill), pushes when a PUSH header shows, and taps a button
// only once it is drawn LIVE, after a 0.3 s reaction — so a button the program never shows, or shows
// greyed, can never be pressed. Its push commands each wheel's PRIMARY so the coupled train's mean is
// 2.2 V, 48 in of TRUE travel, then 0 V (the probe's push).
//
// ── What it captures ────────────────────────────────────────────────────────────────
// The program's serial lines (fd 1, redirected to a file for the run: the shell's line() is ONE
// write() per line there, SER1), the shim screen's draw log, the card's two files, the C reads the
// WATCH made per port (the shim's own counters — the CAL1 brief's test 8b "2 per push", through the
// real shell), every port's configuration and command counters, and every port's largest commanded
// voltage. Nothing here has run on a robot.
//
// THE PLANT'S NUMBERS ARE INVENTED (the probe's and calibration_test_rig.hpp's): this proves the
// program and its shell, not robot physics. Host-test infrastructure; it allocates.

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "pros/apix.h"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/shim_control.hpp"
#include "robots/calibrate_facts.hpp"
#include "shulib/calibration/calibrate_program.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/record_store.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/pros/calibrate.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

namespace cal_golden {

using shulib::hal::fake::FakeMotor;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDt = 0.01;
inline constexpr double kKv = 0.17;    // volts per in/s of wheel surface (the probe's)
inline constexpr double kKs = 1.2;     // the tree's usual static friction
inline constexpr double kTrack = 12.5;  // the plant's TRUE track width, in
/// The build stamp: the probe's, so these goldens compare with the brief's line for line.
inline constexpr const char* kStamp = "Oct  6 2026 12:00:00";
/// The boot banner's label: robot two's build beacon, as the probe's glue passed it.
inline constexpr const char* kLabel = "shulib-robot-variant=tank-calibrate";
/// The operator's reaction: a button is tapped once it has been drawn live this long, s.
inline constexpr double kReactSec = 0.3;
/// Shim seconds before the rig gives up on a program that never returns (the test then FAILS).
inline constexpr double kDeadlineSec = 1200.0;

/// A held screen's answers.
inline constexpr int kLeftBox = 1;   ///< USE NEW (FRONT FLIPPED) or SAVE (MECHANISM PORT TURNED)
inline constexpr int kRunAgain = 2;  ///< RUN AGAIN
inline constexpr int kDisable = 3;   ///< the field disables the robot 1 s after the screen shows

/// What a scenario puts on the card before the run.
enum class Seed {
    None,            ///< an empty card
    FrontLedRecord,  ///< `one`'s saved record as BOTH files (taken from `one`'s golden)
    Garbage,         ///< both files 180 bytes of 'x' (files that open and do not decode)
    BackupGarbage,   ///< only the backup, 180 bytes of 'x'
    EmptyRecordFile, ///< only the record file, EMPTY (0 bytes: a crash mid-write)
};

/// One scenario: the probe's own knobs (harness4.cpp's parse()), by the same names.
struct Scenario {
    std::string name;
    int runs = 1;                           ///< runs to drive; then the field disables the robot
    double pushDir[4] = {1, 1, 1, 1};       ///< +1 the plant's front leads; -1 the other end
    double curve[4] = {1, 1, 1, 1};         ///< right volts / left volts during the push
    bool decouple18 = false;                ///< robot two's slipping port 18 (LEFT OUT)
    bool card = true;                       ///< an SD card in the brain
    bool imu = true;                        ///< an IMU on port 2
    double disableAfterPulses = -1.0;       ///< the field disables N s after PULSES first shows
    int clawTurnsPort = 0;                  ///< a declared port the push turns
    int heldChoice[4] = {0, 0, 0, 0};       ///< per run, on a HELD screen: kLeftBox, kRunAgain, kDisable
    bool swapBundles = false;               ///< LEFT and RIGHT drive cable bundles swapped at the brain
    bool undeclared = false;                ///< undeclared motors: 8 still, 9 an intake the push turns
    bool typo12 = false;                    ///< the facts declare drive port 12 (claw pivot 7's typo)
    /// The typed drive ports the facts carry: 0 none; 2 FLIPPED against `one`'s record; 3 a 5/5 tie;
    /// 4 no port in common; 8 robot two's typed table (made valid, the probe's case 8); 9 a partial
    /// match (12's sign wrong, 16 not typed, 9 typed and not a drive motor).
    int typedCase = 0;
    Seed seed = Seed::None;
};

/// Robot two's facts AS ITS GLUE DERIVES THEM (CAL1 Part B, 2026-11-05;
/// src/programs/calibrate_program.cpp's own composition, src/robots/calibrate_facts.hpp):
/// withTypedDriveFrom(calibrateFactsFor(robot two's chassis table, its mechanism list), the
/// table) — Tank, BLUE, the two claw pivots by their names, the robot's name, the typed IMU port as
/// a cross-check, and NO typed drive ports (its table is not a description the boot can drive
/// from). Until Part B they were written here literally; every golden stayed UNCHANGED when they
/// became derived (CAL1-PROGRESS.md, Part B), and test/calibrate_facts_test.cpp pins the derived
/// facts against those literals. Then the scenario's changes.
[[nodiscard]] inline shulib::calibration::CalibrateFacts factsFor(const Scenario& sc) {
    shulib::calibration::CalibrateFacts f = shulib::bench::withTypedDriveFrom(
        shulib::bench::calibrateFactsFor(shulib::bench::kRobotTwoChassis,
                                         shulib::bench::robot_two::kRobotTwoMechanisms),
        shulib::bench::kRobotTwoChassis);
    if (sc.typo12) {
        f.mechanisms[1].port = 12;  // claw pivot 7 typed as 12: a DRIVE port declared a mechanism
    }
    const auto setTyped = [&f](std::initializer_list<int> l, std::initializer_list<int> r) {
        f.typedDrive = {};
        std::size_t i = 0;
        for (const int p : l) {
            f.typedDrive.left[i++] = static_cast<std::int8_t>(p);
        }
        i = 0;
        for (const int p : r) {
            f.typedDrive.right[i++] = static_cast<std::int8_t>(p);
        }
    };
    switch (sc.typedCase) {
        case 2: setTyped({-20, 19, -18, 17, -16}, {11, -12, 13, -14, 15}); break;
        case 3: setTyped({-11, 12, -13, -16, 17, -20}, {-14, 15, 18, -19}); break;
        case 4: setTyped({-3, 4}, {5, -8}); break;
        case 8: setTyped({-11, 12, -13, 14, -15}, {20, -19, 18, -17, 16}); break;
        case 9: setTyped({-11, -12, -13, 14, -15}, {20, -19, 18, -17, 9}); break;
        default: break;
    }
    return f;
}

/// What a run left behind (header: what it captures).
struct Result {
    std::string serial;  ///< the serial lines, fd 1's bytes, NORMALIZED (cardRoot → "<card>/", the hash)
    std::vector<std::string> screen;  ///< the shim screen's draw log
    std::optional<std::vector<char>> bin;  ///< the record file, if the card holds one
    std::optional<std::vector<char>> bak;  ///< the backup, if the card holds one
    bool captured = false;        ///< fd 1 was redirected and restored (else the serial is not the run's)
    bool timedOut = false;        ///< the rig's deadline ended the run (the test FAILS on it)
    double finishedAt = 0.0;      ///< shim seconds when the program returned
    int runsDriven = 1;           ///< the operator's run count
    pros::shim::MotorCReadCounts cReads[22]{};  ///< the WATCH's C reads, per port
    pros::shim::MotorPortState motors[22]{};    ///< every port's configuration and command counters
    double maxAbsMv[22]{};        ///< every port's largest commanded |mV|
    int poweredSteps[22]{};       ///< 10 ms steps with a non-zero command, per port
    /// What a FORWARD command on the record file does to the plant (forwardScore()): "R of T", "none"
    /// when the card holds no record file, or "does not decode".
    std::string forward;
};

/// The rig (header). One at a time: the shim is global, and so are the two hooks it calls.
class Rig {
public:
    Rig(const Scenario& sc, std::filesystem::path workDir) : sc_{sc}, dir_{std::move(workDir)} {
        // Which wheel each drive port turns, and which members are wired backwards — the plant's
        // wiring, the swap's bundles included (wire() builds the plant from the same tables).
        for (int wheel = 0; wheel < 2; ++wheel) {
            const int base = (wheel == 0) != sc_.swapBundles ? 11 : 16;
            for (int k = 0; k < 5; ++k) {
                wheelOf_[static_cast<std::size_t>(base + k)] = wheel;
            }
        }
        for (const auto& r : kReversed) {
            const int base = (r[0] == 0) != sc_.swapBundles ? 11 : 16;
            revOf_[static_cast<std::size_t>(base + r[1])] = true;
        }
    }
    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;

    /// The card directory as the program sees it ("…/card/").
    [[nodiscard]] std::string cardRoot() const { return (dir_ / "card").string() + "/"; }

    /// Run the scenario (header) with `facts`, seeding the card first with `seedBin` / `seedBak`
    /// (null = leave that file absent).
    Result run(const shulib::calibration::CalibrateFacts& facts, const std::vector<char>* seedBin,
               const std::vector<char>* seedBak) {
        namespace fs = std::filesystem;
        Result out;
        fs::remove_all(dir_);
        fs::create_directories(dir_ / "card");
        const auto put = [this](const char* name, const std::vector<char>* bytes) {
            if (bytes == nullptr) {
                return;
            }
            std::ofstream f{(dir_ / "card" / name).string(), std::ios::binary};
            f.write(bytes->data(), static_cast<std::streamsize>(bytes->size()));
        };
        put(shulib::calibration::kRecordFileName, seedBin);
        put(shulib::calibration::kRecordBackupFileName, seedBak);

        resetShim();
        World world{plantConfig(sc_.decouple18 ? 0.7 : 0.0)};
        world_ = &world;
        wire(world);
        current() = this;
        pros::shim::screenState().observer = &Rig::observeThunk;
        pros::shim::advanceHook() = &Rig::advanceThunk;

        const std::string serialPath = (dir_ / "serial.out").string();
        out.captured = redirectStdout(serialPath);
        if (out.captured) {
            shulib::hal::pros::calibrateInitialize(kLabel, kStamp);
            // runCalibrate()'s composition, the card's root aside (header; the golden test pins
            // the shell's own five against this).
            shulib::hal::pros::ProsDeviceRegistry registry;
            shulib::hal::pros::ProsDeviceProvider devices{registry};
            const std::string root = cardRoot();
            shulib::hal::pros::ProsFileStore card{root.c_str()};
            shulib::hal::pros::ProsBrainScreen screen;
            shulib::hal::pros::ProsCalibrateIo io;
            shulib::calibration::CalibrateProgram program{
                facts, {devices, registry, card, screen, io}, kStamp};
            program.run();
            restoreStdout();
        }
        pros::shim::advanceHook() = nullptr;
        pros::shim::screenState().observer = nullptr;
        current() = nullptr;
        world_ = nullptr;

        out.timedOut = timedOut_;
        out.finishedAt = static_cast<double>(pros::shim::timeState().nowUs) * 1e-6;
        out.runsDriven = op_.run + 1;
        out.screen = pros::shim::screenState().log;
        for (std::size_t p = 0; p < 22; ++p) {
            out.cReads[p] = pros::shim::motorCReads()[p];
            out.motors[p] = pros::shim::motorPorts()[p];
            out.maxAbsMv[p] = maxMv_[p];
            out.poweredSteps[p] = poweredSteps_[p];
        }
        out.bin = readFile(dir_ / "card" / shulib::calibration::kRecordFileName);
        out.bak = readFile(dir_ / "card" / shulib::calibration::kRecordBackupFileName);
        out.forward = "none";
        if (out.bin) {
            shulib::hal::DrivetrainDescription d{};
            const auto* b = reinterpret_cast<const std::byte*>(out.bin->data());
            if (shulib::calibration::decodeRecord(std::span<const std::byte>{b, out.bin->size()}, d)) {
                int total = 0;
                const int right = forwardScore(d, total);
                out.forward = std::to_string(right) + " of " + std::to_string(total);
            } else {
                out.forward = "does not decode";
            }
        }
        if (out.captured) {
            std::ifstream f{serialPath, std::ios::binary};
            out.serial.assign(std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{});
            out.serial = normalize(out.serial, cardRoot());
        }
        resetShim();  // leave nothing behind for the next case
        fs::remove_all(dir_);
        return out;
    }

    /// How a FORWARD command on `d` (every drive port at +1 through its signed port) turns each
    /// motor: how many drive their OWN side's wheel forward, of `total` — the probe's score, against
    /// this rig's wiring (the swap scenario's bundles included).
    [[nodiscard]] int forwardScore(const shulib::hal::DrivetrainDescription& d, int& total) const {
        int right = 0;
        total = 0;
        for (int side = 0; side < 2; ++side) {
            const auto& s = side == 0 ? d.left : d.right;
            for (std::size_t i = 0; i < s.count; ++i) {
                const int sp = s.signedPorts[i];
                const int p = sp < 0 ? -sp : sp;
                const int dir = (sp < 0 ? -1 : 1) * (revOf_[static_cast<std::size_t>(p)] ? -1 : 1);
                ++total;
                if (wheelOf_[static_cast<std::size_t>(p)] == side && dir > 0) {
                    ++right;
                }
            }
        }
        return right;
    }

private:
    /// The members wired backwards, {wheel, member}: robot two's alternating coupled-train signs
    /// (11, 13, 15 on the LEFT wheel, 17, 19 on the RIGHT — before any swap).
    static constexpr int kReversed[5][2] = {{0, 0}, {0, 2}, {0, 4}, {1, 1}, {1, 3}};

    struct World {
        shulib::kinematics::TankKinematics kin{Length{kTrack}};
        shulib::sim::SimHarness h;
        std::array<FakeMotor, 8> extras{};
        FakeMotor* byPort[22]{};
        bool primaryReversed[2] = {false, false};
        std::size_t coupled[2] = {5, 5};
        double cumHeading = 0.0;
        double lastHeading = 0.0;
        explicit World(const shulib::sim::SimHarnessConfig& cfg) : h{kin, cfg} {}
    };

    struct Operator {
        int run = 0;  // 0-based
        bool pushing = false;
        bool pushed = false;
        double pushedIn = 0.0;
        double seenAt = -1.0;  // when the current target button was first seen live
        std::string waitingFor;
        bool finished = false;
        double pushCmd[2] = {0.0, 0.0};
        double pulsesSeenAt = -1.0;
    };

    static Rig*& current() {
        static Rig* r = nullptr;
        return r;
    }
    static void observeThunk(const std::string& line) {
        if (current() != nullptr) {
            current()->observe(line);
        }
    }
    static void advanceThunk(std::uint64_t /*fromUs*/, std::uint64_t toUs) {
        if (current() != nullptr) {
            current()->advance(toUs);
        }
    }

    static void resetShim() {
        pros::shim::resetAll();
        pros::shim::resetScreen();
        pros::shim::resetCompetition();
        pros::shim::resetMotorCReads();
    }

    static shulib::sim::SimHarnessConfig plantConfig(double drag) {
        shulib::sim::SimHarnessConfig cfg;
        cfg.plant.wheelFf = {.kS = kKs, .kV = kKv, .kA = 0.0};
        cfg.plant.coastFollow = 0.0;
        cfg.plant.decoupledDrag = drag;
        return cfg;
    }

    /// The probe's wire(), on the shim's registry (header: the robot).
    void wire(World& w) {
        using pros::c::E_DEVICE_IMU;
        using pros::c::E_DEVICE_MOTOR;
        using pros::c::E_DEVICE_RADIO;
        pros::shim::competitionStatus() = 0;
        pros::shim::usdState().installed = sc_.card;
        pros::shim::setPluggedAtPort(1, E_DEVICE_RADIO);
        if (sc_.imu) {
            pros::shim::setPluggedAtPort(2, E_DEVICE_IMU);
        }
        for (const int p : {6, 7}) {
            pros::shim::setPluggedAtPort(p, E_DEVICE_MOTOR);
        }
        if (sc_.undeclared) {
            for (const int p : {8, 9}) {
                pros::shim::setPluggedAtPort(p, E_DEVICE_MOTOR);
            }
        }
        for (int p = 11; p <= 20; ++p) {
            pros::shim::setPluggedAtPort(p, E_DEVICE_MOTOR);
        }
        for (int wheel = 0; wheel < 2; ++wheel) {
            std::array<FakeMotor*, 4> ex{};
            const int base = (wheel == 0) != sc_.swapBundles ? 11 : 16;
            w.byPort[base] = &w.h.motor(wheel);
            for (int k = 1; k < 5; ++k) {
                FakeMotor& f = w.extras[static_cast<std::size_t>(wheel * 4 + (k - 1))];
                ex[static_cast<std::size_t>(k - 1)] = &f;
                w.byPort[base + k] = &f;
            }
            w.h.plant().attachCoupledMembers(wheel, std::span<FakeMotor* const>{ex.data(), ex.size()});
        }
        for (const auto& r : kReversed) {
            w.h.plant().setMemberFault(r[0], r[1], shulib::sim::MemberFault::Reversed);
            if (r[1] == 0) {
                w.primaryReversed[r[0]] = true;
            }
        }
        if (sc_.decouple18) {
            w.h.plant().setMemberFault(1, 2, shulib::sim::MemberFault::Decoupled);
            --w.coupled[1];
        }
    }

    // ── the screen, as the operator sees it ─────────────────────────────────────────
    void observe(const std::string& line) {
        if (line.rfind("print M 8 4 ", 0) == 0) {
            const auto q = line.find('\'');
            header_ = line.substr(q + 1, line.size() - q - 2);
            return;
        }
        if (line.rfind("fill ", 0) == 0) {
            const auto p = line.find(" pen=");
            fill_[line.substr(5, p - 5)] = line.substr(p + 5);
            return;
        }
        if (line.rfind("erase ", 0) == 0) {
            int x0 = 0;
            int y0 = 0;
            int x1 = 0;
            int y1 = 0;
            if (std::sscanf(line.c_str() + 6, "%d %d %d %d", &x0, &y0, &x1, &y1) == 4) {
                for (auto it = fill_.begin(); it != fill_.end();) {
                    int a = 0;
                    int b = 0;
                    int c = 0;
                    int d = 0;
                    (void)std::sscanf(it->first.c_str(), "%d %d %d %d", &a, &b, &c, &d);
                    if (a >= x0 && c <= x1 && b >= y0 && d <= y1) {
                        it = fill_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            return;
        }
        if (line.rfind("erase-all", 0) == 0) {
            fill_.clear();
            header_.clear();
        }
    }
    [[nodiscard]] bool shows(const char* rect, const char* colour) const {
        const auto it = fill_.find(rect);
        return it != fill_.end() && it->second == colour;
    }

    // The button rectangles (calibrate_panel.hpp) and the live fills — the probe's.
    static constexpr const char* kDoneRect = "6 180 300 234";
    static constexpr const char* kFrontRect = "6 72 234 158";
    static constexpr const char* kBackRect = "246 72 474 158";
    static constexpr const char* kRunAgainRect = "246 112 474 158";
    static constexpr const char* kGreen = "39c36e";
    static constexpr const char* kBlue = "1e5aa8";

    static void tapRect(const char* rect) {
        int a = 0;
        int b = 0;
        int c = 0;
        int d = 0;
        (void)std::sscanf(rect, "%d %d %d %d", &a, &b, &c, &d);
        pros::shim::screenTap(static_cast<std::int16_t>((a + c) / 2),
                              static_cast<std::int16_t>((b + d) / 2));
    }

    // ── the operator ─────────────────────────────────────────────────────────────────
    static double speedFor(double v) { return std::abs(v) <= kKs ? 0.0 : (std::abs(v) - kKs) / kKv; }
    static double voltsFor(double speed) { return speed * kKv + kKs; }

    void newRun() {
        ++op_.run;
        op_.pushing = false;
        op_.pushed = false;
        op_.pushedIn = 0.0;
        op_.seenAt = -1.0;
        op_.waitingFor.clear();
    }

    /// Tap `rect` once it has been seen live for kReactSec (a person's reaction; the arming delay
    /// is the program's own business, honoured because a greyed button is never live).
    bool tapWhenLive(const char* rect, const char* colour, const std::string& name, double now) {
        if (!shows(rect, colour)) {
            if (op_.waitingFor == name) {
                op_.seenAt = -1.0;
            }
            return false;
        }
        if (op_.waitingFor != name) {
            op_.waitingFor = name;
            op_.seenAt = now;
        }
        if (now - op_.seenAt >= kReactSec) {
            tapRect(rect);
            op_.waitingFor.clear();
            op_.seenAt = -1.0;
            return true;
        }
        return false;
    }

    void disable() {
        pros::shim::competitionStatus() = COMPETITION_DISABLED;
        op_.finished = true;
    }

    void operate(double now) {
        if (op_.finished) {
            return;
        }
        if (op_.pulsesSeenAt < 0.0 && header_.find("PULSES") != std::string::npos) {
            op_.pulsesSeenAt = now;
        }
        if (sc_.disableAfterPulses > 0.0 && op_.pulsesSeenAt >= 0.0
            && now >= op_.pulsesSeenAt + sc_.disableAfterPulses) {
            disable();
            return;
        }
        const int r = op_.run < 4 ? op_.run : 3;
        const bool pushScreen = header_.find("PUSH") != std::string::npos;
        // 1. the push
        op_.pushCmd[0] = op_.pushCmd[1] = 0.0;
        if (pushScreen && !op_.pushed) {
            const double remaining = 48.0 - op_.pushedIn;
            if (remaining > 1e-10) {
                double v = 2.2;
                if (sc_.curve[r] == 1.0 && speedFor(v) * kDt > remaining) {
                    v = voltsFor(remaining / kDt);
                }
                const double want[2] = {v * sc_.pushDir[r], v * sc_.curve[r] * sc_.pushDir[r]};
                for (int w = 0; w < 2; ++w) {
                    const double primary = static_cast<double>(world_->coupled[w]) * want[w];
                    op_.pushCmd[w] = world_->primaryReversed[w] ? -primary : primary;
                }
                op_.pushing = true;
            } else {
                op_.pushing = false;
                op_.pushed = true;
            }
        } else {
            op_.pushing = false;
        }
        // 2. the taps
        if (op_.pushed && tapWhenLive(kDoneRect, kGreen, "DONE", now)) {
            return;
        }
        const bool held = header_.find("FLIPPED") != std::string::npos
                          || header_.find("MECHANISM PORT TURNED") != std::string::npos;
        if (held) {
            const int c = sc_.heldChoice[r];
            if (c == kDisable) {
                if (op_.waitingFor != "HELDEND") {
                    op_.waitingFor = "HELDEND";
                    op_.seenAt = now;
                } else if (now - op_.seenAt >= 1.0) {
                    disable();
                }
                return;
            }
            const std::string key = header_ + std::to_string(c);
            if (c == kLeftBox && tapWhenLive(kFrontRect, kBlue, key, now)) {
                return;
            }
            if (c == kRunAgain && tapWhenLive(kBackRect, kBlue, key, now)) {
                newRun();
            }
            return;
        }
        if (shows(kRunAgainRect, kBlue)) {
            if (op_.run + 1 < sc_.runs) {
                if (tapWhenLive(kRunAgainRect, kBlue, "RUNAGAIN", now)) {
                    newRun();
                }
            } else if (op_.waitingFor != "END") {
                // the last run is over: the field disables the robot one second later
                op_.waitingFor = "END";
                op_.seenAt = now;
            } else if (now - op_.seenAt >= 1.0) {
                disable();
            }
        }
    }

    static shulib::hal::BrakeMode brakeOf(pros::v5::MotorBrake b) {
        switch (b) {
            case pros::v5::MotorBrake::brake: return shulib::hal::BrakeMode::Brake;
            case pros::v5::MotorBrake::hold: return shulib::hal::BrakeMode::Hold;
            default: return shulib::hal::BrakeMode::Coast;
        }
    }

    // ── the world step, every 10 ms of shim time ────────────────────────────────────
    void stepWorld(std::uint64_t nowUs) {
        World& w = *world_;
        const double now = static_cast<double>(nowUs) * 1e-6;
        operate(now);
        for (int p = 11; p <= 20; ++p) {
            auto& s = pros::shim::motorState(p);
            FakeMotor* m = w.byPort[p];
            double v = s.control == pros::shim::MotorControl::Voltage ? s.lastVoltageMv / 1000.0 : 0.0;
            const int wheel = wheelOf_[static_cast<std::size_t>(p)];
            if (op_.pushing && m == &w.h.motor(wheel)) {
                v = op_.pushCmd[wheel];
            }
            m->setVoltage(Voltage{v});
            m->setBrakeMode(brakeOf(s.control == pros::shim::MotorControl::Stopped ? s.stoppedUnder
                                                                                     : s.brake));
        }
        const bool wasPushing = op_.pushing;
        w.h.plant().step(Time{kDt});
        if (wasPushing) {
            op_.pushedIn += std::abs(w.h.trueBodyTwist().vx().value()) * kDt;
        }
        for (int p = 11; p <= 20; ++p) {
            auto& s = pros::shim::motorState(p);
            s.positionOutputDeg = w.byPort[p]->position().value() * 180.0 / kPi;
            s.velocityRpm = w.byPort[p]->velocity().value() * 60.0 / (2.0 * kPi);
        }
        for (int p = 1; p <= 21; ++p) {  // every port's largest commanded |V|, and its powered steps
            const auto& st = pros::shim::motorState(p);
            const double mv = st.control == pros::shim::MotorControl::Voltage
                                  ? std::abs(static_cast<double>(st.lastVoltageMv))
                                  : 0.0;
            if (mv > maxMv_[p]) {
                maxMv_[p] = mv;
            }
            if (mv != 0.0) {
                ++poweredSteps_[p];
            }
        }
        // the undeclared intake on 9: the push turns it (0.6 of the left wheel's shaft) and it spins
        // FREELY when powered (the drive's kS/kV, no load); 8 never moves
        if (sc_.undeclared) {
            auto& s = pros::shim::motorState(9);
            const double mv = s.control == pros::shim::MotorControl::Voltage ? s.lastVoltageMv : 0.0;
            if (wasPushing) {
                s.positionOutputDeg +=
                    0.6 * (w.byPort[12]->position().value() * 180.0 / kPi - lastLeftDeg_);
            }
            const double volts = mv / 1000.0;
            const double spd = speedFor(volts) * (volts < 0 ? -1.0 : 1.0);  // rad/s of shaft
            s.positionOutputDeg += spd * kDt * 180.0 / kPi;
            s.velocityRpm = spd * 60.0 / (2.0 * kPi);
        }
        lastLeftDeg_ = w.byPort[12]->position().value() * 180.0 / kPi;
        // a declared port the push turns: it follows the left wheel's travel
        if (sc_.clawTurnsPort > 0 && wasPushing) {
            pros::shim::motorState(sc_.clawTurnsPort).positionOutputDeg =
                w.byPort[12]->position().value() * 180.0 / kPi;
        }
        // the IMU: cumulative clockwise degrees, from the plant's TRUE heading
        const double hd = w.h.truePose().heading().radians();
        double d = hd - w.lastHeading;
        while (d > kPi) {
            d -= 2.0 * kPi;
        }
        while (d < -kPi) {
            d += 2.0 * kPi;
        }
        w.cumHeading += d;
        w.lastHeading = hd;
        auto& imu = pros::shim::imuState(2);
        imu.rotationDegCw = -w.cumHeading * 180.0 / kPi;
        imu.gyroZDegPerSecCw = -w.h.trueBodyTwist().omega().value() * 180.0 / kPi;
        if (imu.resetCalls != imuResets_) {
            imuResets_ = imu.resetCalls;
            imuResetAtUs_ = nowUs;
        }
        if (imu.calibrating && nowUs - imuResetAtUs_ >= 2000000u) {
            imu.calibrating = false;
        }
        if (now > kDeadlineSec && !timedOut_) {
            timedOut_ = true;
            disable();  // the program must still end: the test then FAILS on timedOut
        }
    }

    void advance(std::uint64_t toUs) {
        while (steppedUs_ + 10000u <= toUs) {
            steppedUs_ += 10000u;
            stepWorld(steppedUs_);
        }
    }

    // ── fd 1, for the run ────────────────────────────────────────────────────────────
    bool redirectStdout(const std::string& path) {
        std::cout.flush();
        (void)std::fflush(stdout);
        const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            return false;
        }
        savedStdout_ = ::dup(STDOUT_FILENO);
        const bool ok = savedStdout_ >= 0 && ::dup2(fd, STDOUT_FILENO) == STDOUT_FILENO;
        ::close(fd);
        if (!ok && savedStdout_ >= 0) {
            ::close(savedStdout_);
            savedStdout_ = -1;
        }
        return ok;
    }
    void restoreStdout() {
        if (savedStdout_ >= 0) {
            (void)::dup2(savedStdout_, STDOUT_FILENO);
            ::close(savedStdout_);
            savedStdout_ = -1;
        }
    }

    static std::optional<std::vector<char>> readFile(const std::filesystem::path& p) {
        std::ifstream f{p.string(), std::ios::binary};
        if (!f) {
            return std::nullopt;
        }
        return std::vector<char>{std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
    }

    /// The serial as a golden holds it: the card's directory (this machine's) as "<card>/", and the
    /// build's git hash (this checkout's) as "<hash>" — the only two things that are not the
    /// program's.
    static std::string normalize(std::string s, const std::string& root) {
        for (std::size_t at = s.find(root); at != std::string::npos; at = s.find(root, at)) {
            s.replace(at, root.size(), "<card>/");
        }
        const std::string tag = "build hash : ";
        for (std::size_t at = s.find(tag); at != std::string::npos; at = s.find(tag, at)) {
            const std::size_t from = at + tag.size();
            const std::size_t end = s.find('\n', from);
            s.replace(from, (end == std::string::npos ? s.size() : end) - from, "<hash>");
            at = from;
        }
        return s;
    }

    Scenario sc_;
    std::filesystem::path dir_;
    World* world_ = nullptr;
    Operator op_{};
    std::string header_;
    std::map<std::string, std::string> fill_;
    int wheelOf_[22]{};
    bool revOf_[22]{};
    std::uint64_t steppedUs_ = 0;
    double maxMv_[22]{};
    int poweredSteps_[22]{};
    double lastLeftDeg_ = 0.0;
    std::uint64_t imuResetAtUs_ = 0;
    int imuResets_ = 0;
    bool timedOut_ = false;
    int savedStdout_ = -1;
};

}  // namespace cal_golden
