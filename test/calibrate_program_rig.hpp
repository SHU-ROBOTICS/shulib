#pragma once
//
// calibrate_program_rig.hpp — the host rig for the standard Calibrate (chunk CAL1, Part A2a,
// 2026-10-18): calibration::CalibrateProgram RUN WHOLE on the A2 host test plant, through the
// library's own seams and their fakes — never through the PROS shim (that is A2b's golden test).
//
// ── What it is ──────────────────────────────────────────────────────────────────────
// Robot two's shape on the plant (the CAL1 brief's probe harness, `wire()`): two wheels of FIVE
// coupled members each — LEFT ports 11..15 (11 the primary), RIGHT ports 16..20 (16 the primary) —
// with members 11, 13, 15, 17 and 19 wired backwards (MemberFault::Reversed: robot two's alternating
// coupled-train signs); two claw pivots on 6 and 7 (motors the registry reports, never bridged to the
// plant: they move only when a scenario turns one); the IMU on 2 (the plant's, ready 2 s after the
// program starts — the IMU warm-up the program's first screen waits out); the radio on 1. The program
// sees all of it through:
//   * a FakeDeviceProvider whose ports are BOUND to RecordingMotors over the plant's motors (so the
//     program's commands reach the plant, and every command is counted per port), wrapped in a
//     CountingProvider that counts EVERY device request of every kind — "nothing was asked for" is
//     then a number;
//   * a FakeDeviceRegistry (the census), a FakeFileStore (the card; every operation logged), a
//     FakeBrainScreen (every draw recorded) and a FakeIo (the serial lines, the competition status,
//     the waits, and the WATCH's read of a declared port, counted per port);
//   * a pacer that steps the plant 10 ms per pace() and runs the scripted OPERATOR first.
//
// ── The operator reads the SCREEN, never the program ────────────────────────────────
// It learns what is on the screen from the FakeBrainScreen's draw calls as they happen (the header
// word drawn at (8, 4); each button rectangle's latest fill), exactly as the probe harness read the
// shim's: it pushes when a PUSH header shows, taps DONE only once DONE is drawn LIVE (green), taps a
// held screen's box or RUN AGAIN only once drawn live (blue), each after a 0.3 s reaction — so a
// button the program never shows, or shows greyed, can never be pressed. The push is the probe's: the
// harness commands each wheel's PRIMARY so the coupled train's mean is 2.2 V (signed by the
// primary's wiring and the push direction), 48 in of TRUE travel, then 0 V.
//
// ── The ends are named by the PLANT, not by robot two's camera ──────────────────────
// pushDir +1 leads with the end the plant drives forward (the end robot two's TYPED table calls its
// front, the 15/16 end; robot two's real camera end is the other — the CAL1 brief §8). A front-led
// push here therefore saves LEFT -11 +12 -13 +14 -15 | RIGHT +16 -17 +18 -19 +20.
//
// ── What a scenario can change ──────────────────────────────────────────────────────
// The runs and each run's push (direction, curve); each held screen's answer (left box, RUN AGAIN,
// or the field disabling); a slipping port 18 (LEFT OUT); no card; no IMU; the field disabling mid
// pulses; a declared claw the push turns; the LEFT and RIGHT cable bundles swapped at the brain;
// undeclared motors on 8 (still) and 9 (turned by the push, free-spinning or loaded); one side's
// REPORTED shaft scaled (unequal side scales, CAL1 re-check R14); a motor that throws on its first
// powered command (a PROGRAM FAULT); a declared claw that moves an EXACT angle (the watch's edge); a
// declared port the watch cannot read, or reads as NaN; claws whose readings start far from zero (an
// encoder holds what the last program left), and a declared port whose push-start read alone is
// refused (A2a's second resumption, T5k); a double tap on a held screen's left box. Since the
// review round (A2a.R): BOTH claws turned by the push, a different answer for every MECHANISM PORT
// TURNED question after a run's first (one question per port), the status the end of driver
// control sets (a disable, autonomous, with or without the "connected" bit), and the IMU's heading
// HELD (wheels up: the pulses turn trains, never the robot). Since the SECOND review round
// (2026-10-29): a field or switch connected from the start (the "connected" bit), a declared claw
// that settles during the IMU warm-up (before the push's baseline), a declared port whose DONE read
// alone is refused, a PROGRAM FAULT mid-spin (the motors before it POWERED), and one slow tick (the
// loop's worst gap). Since the THIRD verification round (2026-10-29): the PROGRAM's clock read the
// brain's way (ProsClock's micros() x 1e-6 over the rig's steps — exact at every quarter second,
// where the plant's FakeClock, adding 0.01 a step, is a hair off), and an operator who waits on the
// PUSH screen before pushing; every serial line carries the program clock's instant (FakeIo::lineAt).
// Tests reach the devices directly for the rest — a census edited, a port the
// provider refuses, a hook as the IMU is asked for, as each wait starts, as each touch is read (the
// program's screen is a forwarding seam over the FakeBrainScreen), or as a motor's planted fault
// fires. The card
// is seeded by the TEST, with explicit bytes — a seeded scenario can never run on an empty card by
// accident (the CAL1 check of D6 (a), C2).
//
// THE PLANT'S NUMBERS ARE INVENTED (calibration_test_rig.hpp's): this rig proves the program's
// logic, not robot physics. Host-test infrastructure; it allocates (std::vector, std::string).

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "shulib/calibration/calibrate_program.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/record_store.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/brain_screen.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_brain_screen.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_file_store.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

namespace cal_prog_rig {

using shulib::calibration::CalibrateFacts;
using shulib::hal::BrakeMode;
using shulib::hal::DeviceKind;
using shulib::hal::IMotor;
using shulib::hal::MotorCartridge;
using shulib::hal::fake::FakeMotor;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDt = 0.01;
inline constexpr double kKv = 0.17;    // volts per in/s of wheel surface (calibration_test_rig.hpp's)
inline constexpr double kKs = 1.2;     // the tree's usual static friction
inline constexpr double kTrack = 12.5;  // the plant's TRUE track width, in
/// The plant's wheel radius (DrivePlantConfig::driveWheelDiameter 3.25 in, 1:1): the TRUE inches per
/// radian of every drive shaft.
inline constexpr double kTrueIpr = 3.25 / 2.0;
/// Robot two's measured side-scale ratio (1.36703 / 1.25442, the CAL1 re-check R14): the RIGHT
/// side's reported shaft is scaled by it in the unequal-scales scenario.
inline constexpr double kRobotTwoScaleRatio = 1.36703 / 1.25442;
/// The build stamp every rig run passes.
inline constexpr const char* kStamp = "Oct 18 2026 12:34:56";
/// A held screen's answers.
inline constexpr int kLeftBox = 1;   ///< USE NEW (FRONT FLIPPED) or SAVE (MECHANISM PORT TURNED)
inline constexpr int kRunAgain = 2;  ///< RUN AGAIN
inline constexpr int kDisable = 3;   ///< the field disables the robot 1 s after the screen shows

/// Robot two's facts, written LITERALLY (what CAL1 Part B derives from its tables: Tank, BLUE, the
/// two claw pivots by the table's names, the robot's name, the typed IMU port as a cross-check).
[[nodiscard]] inline CalibrateFacts robotTwoFacts() {
    CalibrateFacts f{};
    f.drive = shulib::config::DrivetrainKind::Tank;
    f.cartridge = MotorCartridge::Blue;
    f.mechanisms[0] = {6, "claw pivot 6"};
    f.mechanisms[1] = {7, "claw pivot 7"};
    f.robot = "2026 TANK CHASSIS (robot two)";
    f.imuPortStated = 2;
    return f;
}

/// What one scenario does (header).
struct Scenario {
    int runs = 1;                               ///< runs to drive; then the field disables the robot
    double pushDir[4] = {1.0, 1.0, 1.0, 1.0};   ///< +1 the plant's front leads; -1 the other end
    double curve[4] = {1.0, 1.0, 1.0, 1.0};     ///< right volts / left volts during the push
    int heldFlip[4] = {0, 0, 0, 0};             ///< FRONT FLIPPED's answer per run (kLeftBox, …)
    int heldMech[4] = {0, 0, 0, 0};             ///< MECHANISM PORT TURNED's answer per run
    /// The answer to every MECHANISM PORT TURNED question after a run's FIRST (one per declared port
    /// that turned), per run; 0 = heldMech's.
    int heldMechNext[4] = {0, 0, 0, 0};
    bool decouple18 = false;                    ///< robot two's slipping port 18 (LEFT OUT)
    bool card = true;                           ///< an SD card in the brain
    bool imu = true;                            ///< an IMU on port 2
    double disableAfterPulses = -1.0;           ///< the field disables N s after PULSES shows
    int clawTurns = 0;                          ///< a declared port the push turns (6 or 7)
    int clawTurnsToo = 0;                       ///< a second claw the push turns (6 or 7), as clawTurns
    /// With clawTurns: the claw's reading moves by EXACTLY this many degrees once the push begins
    /// (an exact edge for the watch's threshold), instead of following the wheel. < 0 = follow.
    double clawExactDeg = -1.0;
    /// The claws' readings when the program starts, degrees ({port 6, port 7}): a claw's encoder
    /// holds whatever the last program left, so the watch must judge TRAVEL since the push began,
    /// never the reading. clawExactDeg is travel from here.
    double clawStartDeg[2] = {0.0, 0.0};
    /// The watch's FIRST read on this port (the push's baseline) is refused; every later read
    /// answers — so a port whose baseline failed is seen answering at DONE.
    int unreadableAtPushPort = 0;
    bool swapBundles = false;                 ///< LEFT and RIGHT drive cable bundles swapped
    bool undeclared8 = false;                   ///< an undeclared motor on 8 that never moves
    bool undeclared9 = false;                   ///< an undeclared motor on 9 the push turns
    bool loaded9 = false;                       ///< ... and that cannot spin when powered
    bool clawsPlugged = true;                   ///< the registry reports motors on 6 and 7
    double rightScale = 1.0;                    ///< the RIGHT side's reported shaft, scaled
    int faultPort = 0;                          ///< throws on its first non-zero command
    int unreadablePort = 0;                     ///< the watch's read refused on this port
    int nanPort = 0;                            ///< the watch's read answers NaN on this port
    bool doubleTapLeftBox = false;              ///< a second tap 0.1 s after each left-box tap
    double touchErrorsUntil = -1.0;             ///< every touch read errors until this time (s)
    double disableAt = -1.0;                    ///< the field disables the robot at this time (s)
    /// The competition status every end of driver control sets (disableAt, disableAfterPulses,
    /// kDisable, the last run's end): a disable by default; 0x02 / 0x06 autonomous, 0x05 a field's
    /// disable with its "connected" bit.
    std::uint8_t endStatus = shulib::calibration::kCompetitionDisabled;
    /// The IMU's heading HELD from its first read, its yaw rate 0 (wheels up: from inside, a pulse
    /// that turns its train and not the robot ROLLED).
    bool headingHeld = false;
    /// The competition status when the program starts: 0 = no field or switch; 0x04 = a field or
    /// switch CONNECTED, enabled in driver control (every scenario before the second review round
    /// started from 0, so a program that ended driver control on ANY bit passed). endStatus is
    /// then 0x05 — the same field's disable.
    std::uint8_t startStatus = 0;
    /// A declared claw (6 or 7) whose reading moves +30 deg at t = 1.0 s — during the IMU warm-up,
    /// before any push (a claw settling under gravity): no turn WITH the push, since the watch's
    /// baseline is taken as the push begins.
    int clawSettlesBeforePush = 0;
    /// The watch's FIRST read on this port (the push's baseline) answers; every later read (DONE's)
    /// is refused — the mirror of unreadableAtPushPort.
    int unreadableAtDonePort = 0;
    /// Throws on its first non-zero command once the SPIN screen shows: a PROGRAM FAULT mid-spin,
    /// every motor commanded before it that tick POWERED (the calibrator powers the LEFT side, then
    /// the RIGHT, in their capture order — so 20, the last, leaves nine powered).
    int faultPortInSpin = 0;
    /// One tick that takes (1 + stallSteps) x 10 ms of plant time: the first pace at or after
    /// stallAt seconds (a slow tick, for the loop's worst-gap line). < 0 = none.
    double stallAt = -1.0;
    int stallSteps = 0;                         ///< the slow tick's EXTRA 10 ms steps
    double deadline = 900.0;                    ///< sim seconds before the rig gives up (timedOut)
    /// The PROGRAM's clock the brain's way: the rig's step count in microseconds plus this offset,
    /// scaled x1e-6 exactly as ProsClock scales pros::micros() — instead of the plant's FakeClock,
    /// which ADDS 0.01 every step and so sits a hair off most decimal instants (why no push-live gap
    /// on the plant's clock ever lands exactly on 0.25 s or 5 s). Under it every instant that is
    /// exact in binary IS exact (double(1e-6) is within 4.6e-17 of 1e-6, under half an ulp of any
    /// product, so n x 1e-6 rounds to n/1e6 whenever n/1e6 is a double): every quarter second, and
    /// so every gap between two quarter seconds. The plant, the IMU's warm-up and the operator keep
    /// the plant's clock. < 0 = the program reads the plant's clock (every scenario before the third
    /// verification round of 2026-10-29).
    std::int64_t exactClockOffsetUs = -1;
    /// The operator waits this long on the PUSH screen, hands off, before pushing, s (0 = at once).
    double pushWaitSec = 0.0;
};

/// The program's clock under Scenario::exactClockOffsetUs (header): ProsClock's formula over the
/// rig's step count — (steps x 10,000 us + the offset) x 1e-6, monotonic as the steps are.
class ExactClock final : public shulib::hal::IClock {
public:
    ExactClock(const std::uint64_t& steppedUs, std::uint64_t offsetUs)
        : steppedUs_{&steppedUs}, offsetUs_{offsetUs} {}
    [[nodiscard]] Time now() const override {
        return Time{static_cast<double>(*steppedUs_ + offsetUs_) * 1e-6};
    }

private:
    const std::uint64_t* steppedUs_;
    std::uint64_t offsetUs_;
};

/// A motor that forwards to the plant's FakeMotor and COUNTS every command the program sends — so
/// "never powered" and "pulsed at 3, 6 and 9 V" are measured, not assumed. Its REPORTED shaft can
/// be scaled (unequal side scales), and it can throw once on its first non-zero command (a planted
/// PROGRAM FAULT).
class RecordingMotor final : public IMotor {
public:
    explicit RecordingMotor(FakeMotor& inner, double scale = 1.0) : inner_{&inner}, scale_{scale} {}

    void setVoltage(Voltage volts) override {
        ++voltageCalls;
        const double v = volts.value();
        if (v != 0.0) {
            if (throwOnFirstPower && !thrown) {
                thrown = true;
                if (beforeThrow) {
                    beforeThrow();  // a test reads the world at the instant of the fault
                }
                SHULIB_PRECONDITION(false, "rig: a planted fault on the first powered command");
            }
            ++nonZeroCalls;
            const double a = std::abs(v);
            maxAbs = a > maxAbs ? a : maxAbs;
            const double rounded = std::round(a * 100.0) / 100.0;
            bool known = false;
            for (const double l : levels) {
                known = known || l == rounded;
            }
            if (!known) {
                levels.push_back(rounded);
            }
        }
        inner_->setVoltage(volts);
    }
    [[nodiscard]] Voltage commandedVoltage() const override { return inner_->commandedVoltage(); }
    void setBrakeMode(BrakeMode mode) override {
        ++brakeCalls;
        if (mode != BrakeMode::Coast) {
            ++nonCoastCalls;
        }
        inner_->setBrakeMode(mode);
    }
    [[nodiscard]] BrakeMode brakeMode() const override { return inner_->brakeMode(); }
    [[nodiscard]] shulib::units::AngleDim position() const override {
        return shulib::units::AngleDim{inner_->position().value() * scale_};
    }
    [[nodiscard]] shulib::units::AngularVelocity velocity() const override {
        return shulib::units::AngularVelocity{inner_->velocity().value() * scale_};
    }
    [[nodiscard]] shulib::units::Current current() const override { return inner_->current(); }
    [[nodiscard]] double temperature() const override { return inner_->temperature(); }

    int voltageCalls = 0;         ///< every setVoltage()
    int nonZeroCalls = 0;         ///< every setVoltage() that was not 0 V
    double maxAbs = 0.0;          ///< the largest |V| commanded
    std::vector<double> levels;   ///< each distinct non-zero |V|, rounded to 0.01, in order
    int brakeCalls = 0;           ///< every setBrakeMode()
    int nonCoastCalls = 0;        ///< every setBrakeMode() that was not Coast
    bool throwOnFirstPower = false;
    bool thrown = false;
    std::function<void()> beforeThrow;  ///< called as the planted fault fires, before it throws

private:
    FakeMotor* inner_;
    double scale_;
};

/// The plant's IMU, not READY until 2 s of rig time — the warm-up the program's IMU screen waits
/// out (ProsDeviceProvider starts the calibration as it makes the IMU; HA-23 says ~2 s).
class WarmupImu final : public shulib::hal::IImu {
public:
    WarmupImu(shulib::hal::IImu& inner, shulib::hal::IClock& clock) : inner_{&inner}, clock_{&clock} {}
    [[nodiscard]] shulib::math::Angle heading() const override {
        if (!hold) {
            return inner_->heading();
        }
        if (!latched_) {
            held_ = inner_->heading();
            latched_ = true;
        }
        return held_;
    }
    [[nodiscard]] shulib::units::AngularVelocity yawRate() const override {
        return hold ? shulib::units::AngularVelocity{0.0} : inner_->yawRate();
    }
    [[nodiscard]] bool isReady() const override {
        return clock_->now().value() >= 2.0 && inner_->isReady();
    }
    [[nodiscard]] shulib::math::Angle pitch() const override { return inner_->pitch(); }
    [[nodiscard]] shulib::math::Angle roll() const override { return inner_->roll(); }

    bool hold = false;  ///< the heading held from its first read, the yaw rate 0 (wheels up)

private:
    shulib::hal::IImu* inner_;
    shulib::hal::IClock* clock_;
    mutable bool latched_ = false;
    mutable shulib::math::Angle held_{};
};

/// An IDeviceProvider that forwards to a FakeDeviceProvider and COUNTS every device request, of
/// every kind, served or refused — so "the refusal came before ANY device was asked for" is one
/// number.
class CountingProvider final : public shulib::hal::IDeviceProvider {
public:
    explicit CountingProvider(shulib::hal::IDeviceProvider& inner) : inner_{&inner} {}
    IMotor* motor(std::int8_t signedPort, MotorCartridge cartridge) override {
        ++requests;
        ++motorRequests;
        return inner_->motor(signedPort, cartridge);
    }
    shulib::hal::IImu* imu(std::uint8_t port) override {
        if (onImu) {
            onImu();  // BEFORE the IMU is made: a test reads the world as the program asks for it
        }
        ++requests;
        ++imuRequests;
        return inner_->imu(port);
    }
    shulib::hal::IGps* gps(std::uint8_t port, Length f, Length l) override {
        ++requests;
        return inner_->gps(port, f, l);
    }
    shulib::hal::IRotation* rotation(std::uint8_t port) override {
        ++requests;
        return inner_->rotation(port);
    }
    shulib::hal::IDigitalOut* digitalOut(std::uint8_t line, bool initialState) override {
        ++requests;
        return inner_->digitalOut(line, initialState);
    }
    shulib::hal::IDistance* distance(std::uint8_t port) override {
        ++requests;
        return inner_->distance(port);
    }
    shulib::hal::IOptical* optical(std::uint8_t port) override {
        ++requests;
        return inner_->optical(port);
    }
    shulib::hal::IDigitalIn* digitalIn(std::uint8_t line) override {
        ++requests;
        return inner_->digitalIn(line);
    }
    const shulib::hal::IReadFaults* readFaults(std::uint8_t port) const noexcept override {
        return inner_->readFaults(port);
    }
    const shulib::hal::IReadFaults* lineReadFaults(std::uint8_t line) const noexcept override {
        return inner_->lineReadFaults(line);
    }
    shulib::hal::IClock& clock() override { return clockOverride != nullptr ? *clockOverride : inner_->clock(); }
    shulib::hal::IBattery& battery() override { return inner_->battery(); }
    shulib::hal::ITelemetrySink& telemetry() override { return inner_->telemetry(); }
    shulib::motion::ITickPacer& pacer() override { return inner_->pacer(); }
    const char* lastRefusal() const noexcept override { return inner_->lastRefusal(); }

    int requests = 0;       ///< every device request of every kind
    int motorRequests = 0;  ///< motor() calls
    int imuRequests = 0;    ///< imu() calls
    std::function<void()> onImu;  ///< called as each imu() request arrives, before it is served
    /// The clock the program is given, when set (Scenario::exactClockOffsetUs); else the inner one's.
    shulib::hal::IClock* clockOverride = nullptr;

private:
    shulib::hal::IDeviceProvider* inner_;
};

class Rig;

/// The rig's CalibrateIo: the serial lines kept in order, the competition status the operator
/// sets, waits that move the plant, and the watch's read counted per port.
class FakeIo final : public shulib::calibration::CalibrateIo {
public:
    explicit FakeIo(Rig& rig) : rig_{&rig} {}
    void line(const char* text) override;
    std::uint8_t competitionStatus() override;
    void delayMs(std::uint32_t ms) override;
    bool readShaftDegrees(int port, double& degrees) override;

    std::vector<std::string> lines;         ///< every serial line, untagged, in order
    /// When each line was written: the PROGRAM's clock then (what the provider's clock() answers —
    /// within a tick, the `now` the program read at the tick's start), one per line, in order.
    std::vector<double> lineAt;
    std::array<int, 22> watchReads{};       ///< readShaftDegrees() calls, by port
    std::uint8_t status = 0;                ///< what competitionStatus() answers
    /// Called as each delayMs() starts, with its length, BEFORE the plant moves — so a test reads
    /// the world as a wait begins (the PROGRAM FAULT's idle waits 250 ms at a time).
    std::function<void(std::uint32_t)> onDelay;

private:
    Rig* rig_;
};

/// The program's screen: every call forwarded to the rig's FakeBrainScreen (whose log the tests
/// read), and `onTouchRead` called as each touch read arrives, BEFORE it is served — so a test can
/// change the world between the paint and the read, as the robot can move between the two on a
/// brain.
class SeamScreen final : public shulib::hal::IBrainScreen {
public:
    explicit SeamScreen(shulib::hal::fake::FakeBrainScreen& inner) : inner_{&inner} {}
    void fillRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                  std::uint32_t rgb) override {
        inner_->fillRect(x0, y0, x1, y1, rgb);
    }
    void outlineRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                     std::uint32_t rgb) override {
        inner_->outlineRect(x0, y0, x1, y1, rgb);
    }
    void eraseRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                   std::uint32_t rgb) override {
        inner_->eraseRect(x0, y0, x1, y1, rgb);
    }
    void eraseAll(std::uint32_t rgb) override { inner_->eraseAll(rgb); }
    void print(shulib::hal::ScreenTextSize size, std::int16_t x, std::int16_t y, std::uint32_t pen,
               std::uint32_t eraser, const char* text, std::size_t cols,
               std::size_t padTo = 0) override {
        inner_->print(size, x, y, pen, eraser, text, cols, padTo);
    }
    [[nodiscard]] shulib::hal::ScreenTouch touch() override {
        if (onTouchRead) {
            onTouchRead();
        }
        return inner_->touch();
    }
    [[nodiscard]] std::uint32_t prints() const override { return inner_->prints(); }

    std::function<void()> onTouchRead;  ///< called as each touch read arrives, before it is served

private:
    shulib::hal::fake::FakeBrainScreen* inner_;
};

/// The rig's pacer: the operator, then one 10 ms plant step.
class RigPacer final : public shulib::motion::ITickPacer {
public:
    explicit RigPacer(Rig& rig) : rig_{&rig} {}
    void pace() override;

private:
    Rig* rig_;
};

/// The whole rig (header). Construct it over a Scenario, seed the card if the test needs to, then
/// run() the program with the facts.
class Rig {
public:
    explicit Rig(const Scenario& sc) : sc_{sc}, h_{kin_, plantConfig(sc)}, imu_{h_.imu(), h_.clock()} {
        imu_.hold = sc_.headingHeld;
        io.status = sc_.startStatus;
        wire();
        provider_.bindImu(2, imu_);
        registry.setKind(1, DeviceKind::Radio);
        if (sc_.imu) {
            registry.setKind(2, DeviceKind::Imu);
        }
        if (sc_.clawsPlugged) {
            registry.setKind(6, DeviceKind::Motor);
            registry.setKind(7, DeviceKind::Motor);
        }
        clawDeg_[6] = sc_.clawStartDeg[0];
        clawDeg_[7] = sc_.clawStartDeg[1];
        if (sc_.undeclared8) {
            registry.setKind(8, DeviceKind::Motor);
        }
        if (sc_.undeclared9) {
            registry.setKind(9, DeviceKind::Motor);
        }
        for (int p = 11; p <= 20; ++p) {
            registry.setKind(p, DeviceKind::Motor);
        }
        card.setCardPresent(sc_.card);
        if (sc_.touchErrorsUntil > 0.0) {
            screen.setTouchError(true);  // from the first read on, until operate() clears it
        }
        if (sc_.faultPort != 0) {
            rec(sc_.faultPort).throwOnFirstPower = true;
        }
        screen.observe(&Rig::observeThunk, this);
        if (sc_.exactClockOffsetUs >= 0) {
            programClock_.emplace(steppedUs_, static_cast<std::uint64_t>(sc_.exactClockOffsetUs));
            counting.clockOverride = &*programClock_;
        }
    }
    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;

    /// Run the program over this rig with `facts`, to its end (the field disabling it, a refusal, or
    /// the rig's deadline).
    void run(const CalibrateFacts& facts) {
        shulib::calibration::CalibrateProgram program{
            facts, {counting, registry, card, seam, io}, kStamp};
        program.run();
        returnedAt = now();
    }

    /// A program over this rig's devices with `facts` and `stamp`, CONSTRUCTED but not run — for a
    /// case that needs two programs alive at once (each must keep its own build time).
    [[nodiscard]] shulib::calibration::CalibrateProgram program(const CalibrateFacts& facts,
                                                                 const char* stamp) {
        return shulib::calibration::CalibrateProgram{facts, {counting, registry, card, seam, io}, stamp};
    }

    /// The FakeDeviceProvider under the counting one (its per-port request counts and cartridges).
    [[nodiscard]] shulib::hal::fake::FakeDeviceProvider& provider() { return provider_; }

    /// The RecordingMotor on `port` (every motor port the rig wires has one).
    [[nodiscard]] RecordingMotor& rec(int port) {
        SHULIB_PRECONDITION(port >= 1 && port <= 21 && rec_[static_cast<std::size_t>(port)],
                            "rig: no motor on that port");
        return *rec_[static_cast<std::size_t>(port)];
    }
    /// Whether the rig wires a motor on `port`.
    [[nodiscard]] bool hasMotor(int port) const {
        return port >= 1 && port <= 21 && rec_[static_cast<std::size_t>(port)].has_value();
    }

    /// How a FORWARD command on `d` (every drive port at +1 through its signed port) turns each
    /// motor: the number that drive their OWN side's wheel forward, of all `d` lists — the probe's
    /// score, against this rig's wiring.
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

    /// Set the plant motor on `port`'s velocity to `radPerSec` until the next plant step overwrites
    /// it (a drive member) — the robot jostled between two reads.
    void jostle(int port, double radPerSec) {
        SHULIB_PRECONDITION(port >= 1 && port <= 21 && byPort_[static_cast<std::size_t>(port)] != nullptr,
                            "rig: no motor on that port");
        byPort_[static_cast<std::size_t>(port)]->setVelocity(shulib::units::AngularVelocity{radPerSec});
    }

    /// The record file's bytes on the card, or empty.
    [[nodiscard]] std::vector<std::byte> fileBytes(const char* name) const {
        const auto f = card.file(name);
        return f ? *f : std::vector<std::byte>{};
    }
    /// The record file decoded, when it decodes.
    [[nodiscard]] std::optional<shulib::hal::DrivetrainDescription> savedRecord() const {
        const auto bytes = fileBytes(shulib::calibration::kRecordFileName);
        shulib::hal::DrivetrainDescription d{};
        if (bytes.empty()
            || !shulib::calibration::decodeRecord(std::span<const std::byte>{bytes.data(), bytes.size()}, d)) {
            return std::nullopt;
        }
        return d;
    }

    /// Replace operations in the card's log from operation `from` on.
    [[nodiscard]] int replacesFrom(std::size_t from) const {
        int n = 0;
        const auto& log = card.log();
        for (std::size_t i = from; i < log.size(); ++i) {
            n += log[i].op == shulib::hal::fake::FakeFileOp::Replace ? 1 : 0;
        }
        return n;
    }

    /// The header word the screen shows NOW (the last one drawn).
    [[nodiscard]] const std::string& header() const { return header_; }
    /// Every header word drawn, in order, without repeats in a row.
    [[nodiscard]] const std::vector<std::string>& headers() const { return headers_; }

    /// Seconds of rig time so far.
    [[nodiscard]] double now() const { return static_cast<double>(steppedUs_) * 1e-6; }
    /// What the PROGRAM's clock reads now (the plant's, or Scenario::exactClockOffsetUs's), s.
    [[nodiscard]] double programNow() { return counting.clock().now().value(); }

    // ── the devices the program runs on (public: the tests read them) ─────────────────
    shulib::hal::fake::FakeDeviceRegistry registry;
    shulib::hal::fake::FakeFileStore card{"/usd/"};
    shulib::hal::fake::FakeBrainScreen screen;
    SeamScreen seam{screen};  ///< what the program is given: `screen`, with a hook on each touch read
    FakeIo io{*this};

    /// One header change: the word, the card's operation count then, and the time.
    struct HeaderMark {
        std::string header;
        std::size_t cardOps = 0;
        double at = 0.0;
    };
    std::vector<HeaderMark> marks;  ///< every header change, in order
    /// Every tap the operator made: what it meant to tap (the screen's word and the answer, "DONE",
    /// "RUNAGAIN", or "REFLEX" for the reflexive second tap), the card's operation count then, and
    /// the time — so "nothing was written until the operator touched it" is a count between marks.
    std::vector<HeaderMark> taps;
    /// The first tap whose name is `name`, or null.
    [[nodiscard]] const HeaderMark* tap(const std::string& name) const {
        for (const HeaderMark& m : taps) {
            if (m.header == name) {
                return &m;
            }
        }
        return nullptr;
    }

    bool timedOut = false;        ///< the rig's deadline ended the run (a test FAILS on it)
    double disabledAt = -1.0;     ///< when the field disabled the robot (s), or -1
    double returnedAt = -1.0;     ///< when run() returned (s)
    int runsStarted = 1;          ///< the operator's run number, 1-based
    int leftBoxTaps = 0;          ///< taps the operator made on a held screen's left box
    double pushedInches[4] = {};  ///< TRUE inches each run's push travelled

private:
    friend class FakeIo;
    friend class RigPacer;

    static shulib::sim::SimHarnessConfig plantConfig(const Scenario& sc) {
        shulib::sim::SimHarnessConfig cfg;
        cfg.plant.wheelFf = {.kS = kKs, .kV = kKv, .kA = 0.0};
        cfg.plant.coastFollow = 0.0;
        cfg.plant.decoupledDrag = sc.decouple18 ? 0.7 : 0.0;
        return cfg;
    }

    /// The probe harness's wire(): two wheels of five coupled members, the alternating wiring,
    /// the bundles swapped when asked; claws and undeclared motors off the drivetrain.
    void wire() {
        for (int wheel = 0; wheel < 2; ++wheel) {
            std::array<FakeMotor*, 4> ex{};
            const int base = (wheel == 0) != sc_.swapBundles ? 11 : 16;
            for (int k = 0; k < 5; ++k) {
                wheelOf_[static_cast<std::size_t>(base + k)] = wheel;
            }
            const double scale = wheel == 1 ? sc_.rightScale : 1.0;
            byPort_[static_cast<std::size_t>(base)] = &h_.motor(wheel);
            for (int k = 1; k < 5; ++k) {
                FakeMotor& f = extras_[static_cast<std::size_t>(wheel * 4 + (k - 1))];
                ex[static_cast<std::size_t>(k - 1)] = &f;
                byPort_[static_cast<std::size_t>(base + k)] = &f;
            }
            for (int k = 0; k < 5; ++k) {
                const auto p = static_cast<std::size_t>(base + k);
                rec_[p].emplace(*byPort_[p], scale);
                provider_.bindMotor(static_cast<std::uint8_t>(p), *rec_[p]);
            }
            h_.plant().attachCoupledMembers(wheel, std::span<FakeMotor* const>{ex.data(), ex.size()});
        }
        const int reversed[][2] = {{0, 0}, {0, 2}, {0, 4}, {1, 1}, {1, 3}};
        for (const auto& r : reversed) {
            h_.plant().setMemberFault(r[0], r[1], shulib::sim::MemberFault::Reversed);
            if (r[1] == 0) {
                primaryReversed_[r[0]] = true;
            }
            const int base = (r[0] == 0) != sc_.swapBundles ? 11 : 16;
            revOf_[static_cast<std::size_t>(base + r[1])] = true;
        }
        if (sc_.decouple18) {
            // Port 18 is the RIGHT wheel's member 2 (16 is member 0), wherever the bundles are.
            h_.plant().setMemberFault(wheelOf_[18], 2, shulib::sim::MemberFault::Decoupled);
            --coupled_[static_cast<std::size_t>(wheelOf_[18])];
        }
        const int offPlant[] = {6, 7, 8, 9};
        for (std::size_t i = 0; i < 4; ++i) {
            const auto p = static_cast<std::size_t>(offPlant[i]);
            byPort_[p] = &offPlant_[i];
            rec_[p].emplace(offPlant_[i], 1.0);
            provider_.bindMotor(static_cast<std::uint8_t>(p), *rec_[p]);
        }
    }

    // ── the operator's view of the screen ───────────────────────────────────────────
    static void observeThunk(const std::string& line, void* ctx) {
        static_cast<Rig*>(ctx)->observe(line);
    }
    void observe(const std::string& line) {
        if (line.rfind("print M 8 4 ", 0) == 0) {
            const auto q = line.find('\'');
            std::string w = line.substr(q + 1, line.size() - q - 2);
            while (!w.empty() && w.back() == ' ') {
                w.pop_back();
            }
            header_ = w;
            if (sc_.faultPortInSpin != 0 && !spinFaultArmed_ && w.find("SPIN") != std::string::npos) {
                spinFaultArmed_ = true;
                rec(sc_.faultPortInSpin).throwOnFirstPower = true;  // its next non-zero command throws
            }
            if (headers_.empty() || headers_.back() != w) {
                headers_.push_back(w);
                marks.push_back({w, card.log().size(), now()});
            }
            return;
        }
        if (line.rfind("fill ", 0) == 0) {
            const auto p = line.find(" pen=");
            fills_.push_back({line.substr(5, p - 5), line.substr(p + 5)});
            return;
        }
        if (line.rfind("erase ", 0) == 0) {
            int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
            if (std::sscanf(line.c_str() + 6, "%d %d %d %d", &x0, &y0, &x1, &y1) == 4) {
                std::vector<std::pair<std::string, std::string>> kept;
                for (const auto& f : fills_) {
                    int a = 0, b = 0, c = 0, d = 0;
                    (void)std::sscanf(f.first.c_str(), "%d %d %d %d", &a, &b, &c, &d);
                    if (!(a >= x0 && c <= x1 && b >= y0 && d <= y1)) {
                        kept.push_back(f);
                    }
                }
                fills_ = kept;
            }
            return;
        }
        if (line.rfind("erase-all", 0) == 0) {
            fills_.clear();
            header_.clear();
        }
    }
    /// True when the latest fill of `rect` is `colour` — a button drawn LIVE in that colour.
    [[nodiscard]] bool shows(const char* rect, const char* colour) const {
        for (auto it = fills_.rbegin(); it != fills_.rend(); ++it) {
            if (it->first == rect) {
                return it->second == colour;
            }
        }
        return false;
    }

    // ── the operator ────────────────────────────────────────────────────────────────
    static constexpr const char* kDoneRect = "6 180 300 234";
    static constexpr const char* kLeftRect = "6 72 234 158";
    static constexpr const char* kRightRect = "246 72 474 158";
    static constexpr const char* kRunAgainRect = "246 112 474 158";
    static constexpr const char* kGreen = "39c36e";
    static constexpr const char* kBlue = "1e5aa8";
    static constexpr double kReact = 0.3;

    void tapRect(const char* rect) {
        int a = 0, b = 0, c = 0, d = 0;
        (void)std::sscanf(rect, "%d %d %d %d", &a, &b, &c, &d);
        screen.tap((a + c) / 2, (b + d) / 2);
    }
    /// Tap `rect` once it has been seen LIVE in `colour` for the reaction time.
    bool tapWhenLive(const char* rect, const char* colour, const std::string& name, double t) {
        if (!shows(rect, colour)) {
            if (waitingFor_ == name) {
                seenAt_ = -1.0;
                waitingFor_.clear();
            }
            return false;
        }
        if (waitingFor_ != name) {
            waitingFor_ = name;
            seenAt_ = t;
        }
        if (t - seenAt_ >= kReact) {
            taps.push_back({name, card.log().size(), t});
            tapRect(rect);
            waitingFor_.clear();
            seenAt_ = -1.0;
            return true;
        }
        return false;
    }
    void newRun() {
        ++run_;
        ++runsStarted;
        pushed_ = false;
        pushing_ = false;
        pushSeenAt_ = -1.0;
        mechQ_ = 0;
        waitingFor_.clear();
        seenAt_ = -1.0;
    }
    void disableNow() {
        io.status = sc_.endStatus;
        if (disabledAt < 0.0) {
            disabledAt = now();
        }
        finished_ = true;
    }

    void operate(double t) {
        if (sc_.touchErrorsUntil > 0.0) {
            screen.setTouchError(t < sc_.touchErrorsUntil);
        }
        if (sc_.disableAt > 0.0 && t >= sc_.disableAt && !finished_) {
            disableNow();
            return;
        }
        if (pendingSecondTapAt_ >= 0.0 && t >= pendingSecondTapAt_) {
            taps.push_back({"REFLEX", card.log().size(), t});
            tapRect(kLeftRect);  // the reflexive second tap: wherever the left box now is
            pendingSecondTapAt_ = -1.0;
        }
        if (finished_) {
            return;
        }
        const int r = run_ < 4 ? run_ : 3;
        if (pulsesSeenAt_ < 0.0 && header_.find("PULSES") != std::string::npos) {
            pulsesSeenAt_ = t;
        }
        if (sc_.disableAfterPulses > 0.0 && pulsesSeenAt_ >= 0.0
            && t >= pulsesSeenAt_ + sc_.disableAfterPulses) {
            disableNow();
            return;
        }
        // 1. the push
        pushCmd_[0] = pushCmd_[1] = 0.0;
        const bool pushScreen = header_.find("PUSH") != std::string::npos;
        const bool wasPushing = pushing_;
        pushing_ = false;
        if (pushScreen && pushSeenAt_ < 0.0) {
            pushSeenAt_ = t;  // the operator first sees this run's PUSH screen
        }
        if (pushScreen && !pushed_ && t - pushSeenAt_ >= sc_.pushWaitSec) {
            const double remaining = 48.0 - pushedInches[r];
            if (remaining > 1e-10) {
                double v = 2.2;  // wheel volts: the plant's kV is volts per in/s of surface
                if (sc_.curve[r] == 1.0 && speedFor(v) * kDt > remaining) {
                    v = voltsFor(remaining / kDt);  // the last step lands on 48 in exactly
                }
                const double want[2] = {v * sc_.pushDir[r], v * sc_.curve[r] * sc_.pushDir[r]};
                for (int w = 0; w < 2; ++w) {
                    const double primary = static_cast<double>(coupled_[static_cast<std::size_t>(w)]) * want[w];
                    pushCmd_[w] = primaryReversed_[w] ? -primary : primary;
                }
                pushing_ = true;
            } else {
                pushed_ = true;
            }
        }
        if (pushing_) {
            h_.motor(0).setVoltage(Voltage{pushCmd_[0]});
            h_.motor(1).setVoltage(Voltage{pushCmd_[1]});
        } else if (wasPushing) {
            h_.motor(0).setVoltage(Voltage{0.0});  // the hands come off: the harness's command ends
            h_.motor(1).setVoltage(Voltage{0.0});
        }
        // 2. the taps
        if (pushed_ && tapWhenLive(kDoneRect, kGreen, "DONE", t)) {
            return;
        }
        const bool flippedScreen = header_ == "FRONT FLIPPED";
        const bool mechScreen = header_ == "MECHANISM PORT TURNED";
        if (flippedScreen || mechScreen) {
            int c = flippedScreen ? sc_.heldFlip[r] : sc_.heldMech[r];
            if (mechScreen && mechQ_ > 0 && sc_.heldMechNext[r] != 0) {
                c = sc_.heldMechNext[r];
            }
            // A run's later MECHANISM PORT TURNED questions are named "#2", "#3", ... (the first
            // keeps its plain name).
            const std::string key = header_ + std::to_string(c)
                                    + (mechScreen && mechQ_ > 0 ? "#" + std::to_string(mechQ_ + 1) : "");
            if (c == kDisable) {
                if (waitingFor_ != "HELDEND") {
                    waitingFor_ = "HELDEND";
                    seenAt_ = t;
                } else if (t - seenAt_ >= 1.0) {
                    disableNow();
                }
                return;
            }
            if (c == kLeftBox && tapWhenLive(kLeftRect, kBlue, key, t)) {
                ++leftBoxTaps;
                if (mechScreen) {
                    ++mechQ_;  // the next MECHANISM PORT TURNED is the next port's question
                }
                if (sc_.doubleTapLeftBox) {
                    pendingSecondTapAt_ = t + 0.1;
                }
                return;
            }
            if (c == kRunAgain && tapWhenLive(kRightRect, kBlue, key, t)) {
                newRun();
            }
            return;
        }
        if (shows(kRunAgainRect, kBlue)) {
            if (run_ + 1 < sc_.runs) {
                if (tapWhenLive(kRunAgainRect, kBlue, "RUNAGAIN", t)) {
                    newRun();
                }
            } else if (waitingFor_ != "END") {
                waitingFor_ = "END";
                seenAt_ = t;
            } else if (t - seenAt_ >= 1.0) {
                disableNow();
            }
        }
    }

    /// The plant's steady speed at `v` volts (in/s of wheel surface — and, for port 9's roller
    /// model, rad/s of its shaft: a model, as the probe's was).
    [[nodiscard]] static double speedFor(double v) {
        return std::abs(v) <= kKs ? 0.0 : (std::abs(v) - kKs) / kKv;
    }
    [[nodiscard]] static double voltsFor(double speed) { return speed * kKv + kKs; }

    /// One 10 ms step of the world: the operator, the plant, the off-plant motors.
    void step() {
        steppedUs_ += 10000u;
        const double t = now();
        operate(t);
        if (sc_.clawSettlesBeforePush != 0 && !clawSettled_ && t >= 1.0) {
            clawSettled_ = true;  // during the IMU warm-up: before the push's baseline is read
            clawDeg_[static_cast<std::size_t>(sc_.clawSettlesBeforePush)] += 30.0;
        }
        const double leftBefore = byPort_[12]->position().value();
        h_.plant().step(Time{kDt});
        const double leftDelta = byPort_[12]->position().value() - leftBefore;
        if (pushing_) {
            pushedInches[run_ < 4 ? run_ : 3] += std::abs(h_.trueBodyTwist().vx().value()) * kDt;
            if (sc_.clawTurns != 0 && sc_.clawExactDeg >= 0.0) {
                // travel from the claw's start (0.0 + x is x exactly, so T5g's edge stays exact)
                clawDeg_[static_cast<std::size_t>(sc_.clawTurns)] =
                    (sc_.clawTurns == 6 ? sc_.clawStartDeg[0]
                                        : sc_.clawTurns == 7 ? sc_.clawStartDeg[1] : 0.0)
                    + sc_.clawExactDeg;
            } else if (sc_.clawTurns != 0) {
                clawDeg_[static_cast<std::size_t>(sc_.clawTurns)] += leftDelta * 180.0 / kPi;
            }
            if (sc_.clawTurnsToo != 0) {
                clawDeg_[static_cast<std::size_t>(sc_.clawTurnsToo)] += leftDelta * 180.0 / kPi;
            }
        }
        // Port 9: an intake roller the push turns (0.6 of the left wheel's shaft) and that spins
        // FREELY when powered (the drive's kS/kV, no load) — or not at all when LOADED.
        FakeMotor& nine = offPlant_[3];
        if (sc_.undeclared9) {
            double pos = nine.position().value();
            if (pushing_) {
                pos += 0.6 * leftDelta;
            }
            const double volts = nine.commandedVoltage().value();
            const double spd = sc_.loaded9 ? 0.0 : speedFor(volts) * (volts < 0.0 ? -1.0 : 1.0);
            pos += spd * kDt;
            nine.setPosition(shulib::units::AngleDim{pos});
            nine.setVelocity(shulib::units::AngularVelocity{spd});
        }
        if (t > sc_.deadline) {
            timedOut = true;
            disableNow();
            // Whatever the scenario's end status, a timed-out run must END: the plainest disable
            // there is (0x01 alone), so a program that honoured one bit, or only an exact status,
            // still returns — and the test fails on timedOut instead of hanging the suite.
            io.status = shulib::calibration::kCompetitionDisabled;
        }
    }
    /// One pace: a 10 ms step — and, once, the scenario's slow tick.
    void pace() {
        step();
        if (sc_.stallAt >= 0.0 && !stalled_ && now() >= sc_.stallAt) {
            stalled_ = true;
            for (int k = 0; k < sc_.stallSteps; ++k) {
                step();
            }
        }
    }
    void delay(std::uint32_t ms) {
        pendingUs_ += static_cast<std::uint64_t>(ms) * 1000u;
        while (pendingUs_ >= 10000u) {
            pendingUs_ -= 10000u;
            step();
        }
    }

    Scenario sc_;
    shulib::kinematics::TankKinematics kin_{Length{kTrack}};
    shulib::sim::SimHarness h_;
    std::array<FakeMotor, 8> extras_{};
    std::array<FakeMotor, 4> offPlant_{};  // ports 6, 7, 8, 9
    std::array<FakeMotor*, 22> byPort_{};
    std::array<std::optional<RecordingMotor>, 22> rec_{};
    std::array<int, 22> wheelOf_{};
    std::array<bool, 22> revOf_{};
    bool primaryReversed_[2] = {false, false};
    std::size_t coupled_[2] = {5, 5};
    std::array<double, 22> clawDeg_{};
    std::array<int, 22> shaftReads_{};  // the watch's reads per port (unreadableAtPushPort)
    WarmupImu imu_;
    shulib::hal::NullSink telemetry_;
    RigPacer pacer_{*this};
    shulib::hal::fake::FakeDeviceProvider provider_{h_.clock(), h_.battery(), telemetry_, pacer_};

public:
    CountingProvider counting{provider_};  ///< what the program is given: counts every request

private:
    // the operator's state
    std::string header_;
    std::vector<std::string> headers_;
    std::vector<std::pair<std::string, std::string>> fills_;
    int run_ = 0;
    bool pushing_ = false;
    bool pushed_ = false;
    bool finished_ = false;
    double pushCmd_[2] = {0.0, 0.0};
    std::string waitingFor_;
    double seenAt_ = -1.0;
    double pulsesSeenAt_ = -1.0;
    double pendingSecondTapAt_ = -1.0;
    int mechQ_ = 0;  // MECHANISM PORT TURNED questions answered with the left box this run
    bool spinFaultArmed_ = false;
    bool clawSettled_ = false;
    bool stalled_ = false;
    std::uint64_t steppedUs_ = 0;
    std::uint64_t pendingUs_ = 0;
    double pushSeenAt_ = -1.0;  // when the operator first saw this run's PUSH screen (rig time)
    std::optional<ExactClock> programClock_;  // the program's clock under exactClockOffsetUs

    bool readShaft(int port, double& degrees) {
        if (port == sc_.unreadablePort) {
            return false;
        }
        if (port >= 1 && port <= 21 && port == sc_.unreadableAtPushPort
            && shaftReads_[static_cast<std::size_t>(port)]++ == 0) {
            return false;  // the push's baseline read refused; every later read answers
        }
        if (port >= 1 && port <= 21 && port == sc_.unreadableAtDonePort
            && shaftReads_[static_cast<std::size_t>(port)]++ >= 1) {
            return false;  // the baseline read answered; every later read (DONE's) refused
        }
        if (port == sc_.nanPort) {
            degrees = std::nan("");
            return true;  // a read that "worked" and is not a number
        }
        if (port == 6 || port == 7) {
            if (!sc_.clawsPlugged) {
                return false;  // no motor on the port: PROS answers PROS_ERR_F, the shell false
            }
            degrees = clawDeg_[static_cast<std::size_t>(port)];
            return true;
        }
        if (port >= 1 && port <= 21 && byPort_[static_cast<std::size_t>(port)] != nullptr) {
            degrees = byPort_[static_cast<std::size_t>(port)]->position().value() * 180.0 / kPi;
            return true;
        }
        return false;
    }
};

inline void FakeIo::line(const char* text) {
    lines.emplace_back(text != nullptr ? text : "");
    lineAt.push_back(rig_->programNow());
}
inline std::uint8_t FakeIo::competitionStatus() { return status; }
inline void FakeIo::delayMs(std::uint32_t ms) {
    if (onDelay) {
        onDelay(ms);
    }
    rig_->delay(ms);
}
inline bool FakeIo::readShaftDegrees(int port, double& degrees) {
    if (port >= 1 && port <= 21) {
        ++watchReads[static_cast<std::size_t>(port)];
    }
    return rig_->readShaft(port, degrees);
}
inline void RigPacer::pace() { rig_->pace(); }

// ── reading the serial record ────────────────────────────────────────────────────────

/// The index of the first line equal to `text` at or after `from`, or npos.
[[nodiscard]] inline std::size_t findLine(const std::vector<std::string>& lines, const std::string& text,
                                          std::size_t from = 0) {
    for (std::size_t i = from; i < lines.size(); ++i) {
        if (lines[i] == text) {
            return i;
        }
    }
    return std::string::npos;
}
/// The index of the first line beginning with `prefix` at or after `from`, or npos.
[[nodiscard]] inline std::size_t findPrefix(const std::vector<std::string>& lines,
                                            const std::string& prefix, std::size_t from = 0) {
    for (std::size_t i = from; i < lines.size(); ++i) {
        if (lines[i].rfind(prefix, 0) == 0) {
            return i;
        }
    }
    return std::string::npos;
}
/// How many lines begin with `prefix`.
[[nodiscard]] inline int countPrefix(const std::vector<std::string>& lines, const std::string& prefix) {
    int n = 0;
    for (const auto& l : lines) {
        n += l.rfind(prefix, 0) == 0 ? 1 : 0;
    }
    return n;
}
/// How many lines contain `part`.
[[nodiscard]] inline int countContaining(const std::vector<std::string>& lines, const std::string& part) {
    int n = 0;
    for (const auto& l : lines) {
        n += l.find(part) != std::string::npos ? 1 : 0;
    }
    return n;
}

/// A description's two sides as the serial log writes them: "LEFT -11 +12 | RIGHT +16 -17".
[[nodiscard]] inline std::string sidesOf(const shulib::hal::DrivetrainDescription& d) {
    std::string s = "LEFT";
    for (std::size_t i = 0; i < d.left.count; ++i) {
        char b[8];
        std::snprintf(b, sizeof b, " %+d", static_cast<int>(d.left.signedPorts[i]));
        s += b;
    }
    s += " | RIGHT";
    for (std::size_t i = 0; i < d.right.count; ++i) {
        char b[8];
        std::snprintf(b, sizeof b, " %+d", static_cast<int>(d.right.signedPorts[i]));
        s += b;
    }
    return s;
}

/// A description as encodeRecord() writes it.
[[nodiscard]] inline std::vector<std::byte> encoded(const shulib::hal::DrivetrainDescription& d) {
    std::vector<std::byte> b(shulib::calibration::kRecordBytes);
    const std::size_t n = shulib::calibration::encodeRecord(d, std::span<std::byte>{b.data(), b.size()});
    b.resize(n);
    return b;
}

}  // namespace cal_prog_rig
