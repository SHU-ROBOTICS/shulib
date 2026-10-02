// teleop::DriveProtection inside teleop::runDriverControl — the old "shulib Drive"'s protections, in
// the library, for every program (chunk T4 Part 2, brief §3 "Part 2", tests P2-1 … P2-22; P2-23 and
// P2-24, and the lettered cases after 2026-09-30, from the review of that day; P2-19e and P2-20d from
// the verification of 2026-10-01).
//
// ═══ HOW THE CASES ARE DRIVEN ════════════════════════════════════════════════════════
// Every case drives a whole robot::Robot — robot two's SHAPE: four drive members a side, the
// record's port numbers (L -16 +17 -18 -20, R +11 -12 +13 +15), drive-encoder odometry — through
// runDriverControl with a FakeController, exactly as a program does. Two worlds:
//   * THE HOST TEST PLANT (World::Mode::Plant): the A2 plant, each side's four members hung on its
//     coupled train, member faults from DrivePlant::setMemberFault (Frozen, Reversed, SignFlipped,
//     Decoupled). THE PLANT SYNTHESIZES NO CURRENT: every current below is set by hand with
//     FakeMotor::setCurrent. And it has no mass: its train reverses in one tick and can never hit
//     a wall.
//   * HAND-STEPPED (World::Mode::Hand): no plant is stepped. The test writes every member's next
//     position and velocity into its FakeMotor itself, at the top of each tick, from a scripted table
//     or from HandSide — a small train with a lag, BLUE's 300 counts per output turn with a per-member
//     encoder phase, a velocity report lagging and truncated to whole output RPM, an optional wall
//     and an optional slipping member: the T4 re-check's slip_probe model (2026-09-28), run here
//     through the REAL loop, the REAL CoupledSideMonitor and the REAL FrozenReadingDetector. It is
//     how the cases the plant cannot shape — a wall, a coast-down, a slipping member, a reading that
//     fights and then freezes — are made.
// Each case names the bug it catches; the mutation that turns it red is in T4-PROGRESS "Part 2a".
//
// NO HARDWARE CLAIM. Nothing here has run on a robot.

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/digital_out.hpp"
#include "shulib/hal/fake/fake_controller.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/absent_gps.hpp"
#include "shulib/hal/absent_tag_source.hpp"
#include "shulib/hal/absent_vision.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/teleop_loop.hpp"

namespace cfgns = shulib::config;
using motion_rig::motionConfig;
using motion_rig::plantConfig;
using shulib::PreconditionError;
using shulib::diag::FaultCode;
using shulib::hal::ControllerAxis;
using shulib::hal::ControllerButton;
using shulib::hal::DescriptionSource;
using shulib::hal::DrivetrainDescription;
using shulib::hal::IMotor;
using shulib::hal::MotorCartridge;
using shulib::hal::MotorGroup;
using shulib::hal::fake::FakeMotor;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::math::ChassisSpeeds;
using shulib::sim::MemberFault;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::teleop::DriveProtection;
using shulib::teleop::DriveProtectionConfig;
using shulib::teleop::DriveProtectionState;
using shulib::teleop::DriverLoopConfig;
using shulib::teleop::DriverProfile;
using shulib::teleop::DriveShaper;
using shulib::teleop::DriveShaperState;
using shulib::teleop::DriveVerdict;
using shulib::teleop::HeadingSample;
using shulib::teleop::ProtectionMode;
using shulib::teleop::runDriverControl;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr std::int8_t kLeft[4] = {-16, 17, -18, -20};  // robot two's record of 2026-09-25
constexpr std::int8_t kRight[4] = {11, -12, 13, 15};
constexpr int kLeftPort[4] = {16, 17, 18, 20};
constexpr int kRightPort[4] = {11, 12, 13, 15};
constexpr std::uint8_t kImuPort = 2;
constexpr double kTrackWidth = 12.0;
constexpr double kWheelDiameter = 4.0;  // inches per radian 2.0, as the description says

const shulib::kinematics::TankKinematics kTank{Length{kTrackWidth}};

/// Robot two's shape: four per side, drive-encoder odometry, the plant-matching motion gains.
cfgns::RobotConfig robotTwoShape() {
    DrivetrainDescription d{.cartridge = MotorCartridge::Blue,
                            .imuPort = kImuPort,
                            .trackWidth = Length{kTrackWidth},
                            .source = DescriptionSource::RobotMeasured,
                            .provenance = "T4 Part 2 host fixture: never measured on a robot"};
    for (std::size_t i = 0; i < 4; ++i) {
        d.left.signedPorts[i] = kLeft[i];
        d.right.signedPorts[i] = kRight[i];
    }
    d.left.count = 4;
    d.right.count = 4;
    d.left.inchesPerRadian = Length{0.5 * kWheelDiameter};
    d.right.inchesPerRadian = Length{0.5 * kWheelDiameter};
    cfgns::RobotConfig c{.identity = {.name = "Robot Two", .team = "1234A", .season = "Push Back"},
                         .drivetrain = {.kind = cfgns::DrivetrainKind::Tank,
                                        .tank = {.description = d}},
                         .odometry = {.kind = cfgns::OdometryKind::DriveEncoders}};
    c.tuning.motion = motionConfig();
    return c;
}

SimHarnessConfig worldConfig() {
    SimHarnessConfig cfg = plantConfig();
    cfg.plant.driveWheelDiameter = Length{kWheelDiameter};
    return cfg;
}

constexpr DriverLoopConfig kCfg{.tickPeriod = Time{0.01}, .rezeroButton = ControllerButton::Y};

/// What one tick left behind, read in perTick (after drive() and the protections, before pace()).
struct TickLog {
    std::array<double, 8> mv{};  ///< every drive member's commanded volts: LEFT 0..3, RIGHT 0..3
    DriveProtectionState st{};
    std::uint32_t runtimeAbsent[2]{};
    std::uint32_t excluded[2]{};
    std::uint32_t disagreeing[2]{};  ///< the group monitor's THIS-tick disagreeing mask
    int persisted[2]{};
};

/// The pacer: the plant steps (Plant mode), or only the clock advances (Hand mode). Capped, so a
/// loop that never sees its disable turns a case red instead of hanging the suite.
struct StepPacer final : shulib::motion::ITickPacer {
    StepPacer(SimHarness& harness, bool plantMode) : h{&harness}, plant{plantMode} {}
    void pace() override {
        if (++paces > 200000) {
            throw std::runtime_error("StepPacer: pace cap exceeded");
        }
        if (plant) {
            h->plant().step(Time{0.01});
        } else {
            h->clock().advance(Time{0.01});
        }
    }
    SimHarness* h;
    bool plant;
    int paces = 0;
};

/// A robot two-shaped Robot over the fake provider, driven through runDriverControl.
struct World {
    enum class Mode { Plant, Hand };

    explicit World(Mode m = Mode::Plant, const cfgns::RobotConfig& c = robotTwoShape(),
                   const SimHarnessConfig& plantCfg = worldConfig())
        : mode{m}, h{kTank, plantCfg}, pacer{h, m == Mode::Plant},
          devices{h.clock(), h.battery(), sink, pacer} {
        h.battery().setVoltage(shulib::units::Voltage{12.8});
        wire(c);
        robot.emplace(c, devices);
    }

    /// The FakeMotor behind LEFT (side 0) or RIGHT (side 1) slot `slot`.
    FakeMotor& member(int side, int slot) {
        if (mode == Mode::Plant && slot == 0) {
            return h.motor(side);
        }
        const int port = side == 0 ? kLeftPort[slot] : kRightPort[slot];
        return devices.fakeMotor(static_cast<std::uint8_t>(port));
    }
    MotorGroup& group(int side) {
        return *robot->chassis().deps().motorGroups[static_cast<std::size_t>(side)];
    }
    /// Every drive member's current, the same value.
    void allAmps(double a) {
        for (int s = 0; s < 2; ++s) {
            for (int i = 0; i < 4; ++i) member(s, i).setCurrent(shulib::units::Current{a});
        }
    }
    void stick(double forward, double turn = 0.0) {
        pad.setAxis(ControllerAxis::LeftY, forward);
        pad.setAxis(ControllerAxis::RightX, turn);
    }

    /// Run `ticks` ticks of driver control; `script(k)` runs at the TOP of tick k (inside the
    /// disabled predicate, which the loop polls first). Returns one TickLog per completed tick.
    template <typename Script>
    std::vector<TickLog> run(int ticks, Script&& script, const DriverLoopConfig& cfg = kCfg) {
        std::vector<TickLog> log;
        int k = 0;
        runDriverControl(
            robot->chassis(), pad, pacer, shaper, cfg,
            [&] {
                if (k >= ticks) return true;
                script(k);
                return false;
            },
            [] { return HeadingSample{.radians = 0.0, .trusted = true}; },
            [&](const ChassisSpeeds&, bool, const DriveShaperState&,
                const DriveProtectionState& st) {
                TickLog t;
                for (int s = 0; s < 2; ++s) {
                    for (int i = 0; i < 4; ++i) {
                        t.mv[static_cast<std::size_t>(s * 4 + i)] =
                            member(s, i).commandedVoltage().value();
                    }
                    t.runtimeAbsent[s] = group(s).runtimeAbsentMask();
                    t.excluded[s] = group(s).monitorExcludedMask();
                    t.disagreeing[s] = group(s).lastVerdict().disagreeingMask;
                    t.persisted[s] = group(s).lastVerdict().persistedCount;
                }
                t.st = st;
                log.push_back(t);
                ++k;
            });
        return log;
    }

    Mode mode;
    SimHarness h;
    FakeTelemetrySink sink;
    StepPacer pacer;
    shulib::hal::fake::FakeDeviceProvider devices;
    shulib::hal::fake::FakeController pad;
    DriveShaper shaper{DriverProfile{}};
    std::optional<shulib::robot::Robot> robot;

private:
    void wire(const cfgns::RobotConfig& c) {
        const DrivetrainDescription& d = c.drivetrain.tank.description;
        const shulib::hal::DriveSideDescription* sides[2] = {&d.left, &d.right};
        for (int wheel = 0; wheel < 2; ++wheel) {
            const shulib::hal::DriveSideDescription& side = *sides[wheel];
            if (mode == Mode::Hand) {
                continue;  // every drive port is the provider's own fake, written by the test
            }
            devices.bindMotor(static_cast<std::uint8_t>(
                                  DrivetrainDescription::portMagnitude(side.signedPorts[0])),
                              h.motor(wheel));
            std::array<FakeMotor*, 4> extras{};
            std::size_t n = 0;
            for (std::size_t i = 1; i < side.count; ++i) {
                extras[n++] = &devices.fakeMotor(static_cast<std::uint8_t>(
                    DrivetrainDescription::portMagnitude(side.signedPorts[i])));
            }
            h.plant().attachCoupledMembers(wheel, std::span<FakeMotor* const>{extras.data(), n});
        }
        devices.bindImu(c.drivetrain.imuPort(), h.imu());
    }
};

/// The TEL lines in `sink` from index `from` on, in order.
std::vector<std::string> telLines(const FakeTelemetrySink& sink, int from = 0) {
    std::vector<std::string> out;
    for (int i = from; i < sink.size(); ++i) {
        if (sink.at(i).subsystem == "TEL") out.push_back(sink.at(i).message);
    }
    return out;
}

int countContaining(const std::vector<std::string>& lines, const std::string& needle) {
    int n = 0;
    for (const std::string& l : lines) {
        if (l.find(needle) != std::string::npos) ++n;
    }
    return n;
}

/// The ticks on which a cut STARTED (its deciding tick: cutCount rose).
std::vector<int> cutTicks(const std::vector<TickLog>& log) {
    std::vector<int> out;
    int last = 0;
    for (std::size_t k = 0; k < log.size(); ++k) {
        if (log[k].st.cutCount > last) out.push_back(static_cast<int>(k));
        last = log[k].st.cutCount;
    }
    return out;
}

/// The first tick on which LEFT (side 0) slot `slot` was marked ABSENT, or -1.
int absentTick(const std::vector<TickLog>& log, int side, int slot) {
    for (std::size_t k = 0; k < log.size(); ++k) {
        if ((log[k].runtimeAbsent[side] & (std::uint32_t{1} << slot)) != 0) {
            return static_cast<int>(k);
        }
    }
    return -1;
}

/// The first tick on which side `side` slot `slot` was excluded from the monitor (FROZEN), or -1.
int frozenTick(const std::vector<TickLog>& log, int side, int slot) {
    for (std::size_t k = 0; k < log.size(); ++k) {
        if ((log[k].excluded[side] & (std::uint32_t{1} << slot)) != 0) return static_cast<int>(k);
    }
    return -1;
}

bool allDriveZero(const TickLog& t) {
    for (const double v : t.mv) {
        if (v != 0.0) return false;
    }
    return true;
}

/// The longest run of consecutive ticks a member other than `except` (side, slot) was FROZEN.
int longestLiveFrozenRun(const std::vector<TickLog>& log, int exceptSide = -1,
                         int exceptSlot = -1) {
    int best = 0;
    int run[2][4] = {};
    for (const TickLog& t : log) {
        for (int s = 0; s < 2; ++s) {
            for (int i = 0; i < 4; ++i) {
                const bool f = (t.excluded[s] & (std::uint32_t{1} << i)) != 0;
                run[s][i] = f ? run[s][i] + 1 : 0;
                if (!(s == exceptSide && i == exceptSlot)) best = std::max(best, run[s][i]);
            }
        }
    }
    return best;
}

// ═══ The hand-stepped train (the re-check's slip_probe model, 2026-09-28) ══════════════════
constexpr double kDt = 0.01;
constexpr double kPi = 3.14159265358979323846;
constexpr double kCount = 2.0 * kPi / 300.0;  // BLUE: 300 counts per output turn
constexpr double kRpm = 2.0 * kPi / 60.0;

/// One side of four coupled members, stepped by hand from the side's commanded volts.
struct HandSide {
    double tauTrain = 0.08;    ///< the train's lag under a command, s
    double tauCoast = 0.08;    ///< the train's time constant with the command at 0, s
    double tauV = 0.08;        ///< the velocity report's lag, s
    std::array<double, 4> tauVOf{};  ///< one member's OWN report lag, s (0: `tauV`)
    double phaseCounts = 1.0;  ///< the encoder phase spread across the four members, counts
    double radPerVolt = 5.0;
    int slip = -1;             ///< the member whose gear slips (its own free spin), or -1
    double slipDrag = 0.0;
    bool wall = false;         ///< the train held at 0 (a wall), the slipping member excepted
    int dead = -1;             ///< the member whose readings are held (a dead port), or -1
    // ONE stalled member that does not read like its stalled mates (the review of 2026-09-30;
    // each applies only once the wall has held the train more than 5 ticks, never to `slip`):
    int flicker = -1;          ///< its whole-RPM report reads 1 RPM on odd ticks instead of 0, or -1
    int dither = -1;           ///< its encoder toggles one count every 3 ticks (a humming train), or -1
    int wallTicks = 0;         ///< consecutive ticks the wall has held the train
    int tick = 0;              ///< write() calls so far: the loop's tick, in handTick
    double w = 0.0, theta = 0.0, wSlip = 0.0, thSlip = 0.0;
    std::array<double, 4> vf{};
    std::array<double, 4> lastPos{};
    std::array<double, 4> lastVel{};

    void step(double cmd) {
        const double target = radPerVolt * cmd;
        if (wall) {
            w = 0.0;
            ++wallTicks;
        } else {
            w += (target - w) * (kDt / (cmd == 0.0 ? tauCoast : tauTrain));
            wallTicks = 0;
        }
        theta += w * kDt;
        wSlip += (target - wSlip) * (kDt / 0.05);
        thSlip += wSlip * kDt + slipDrag * w * kDt;
        for (std::size_t i = 0; i < 4; ++i) {
            const double wi = static_cast<int>(i) == slip ? wSlip + slipDrag * w : w;
            vf[i] += (wi - vf[i]) * (kDt / (tauVOf[i] > 0.0 ? tauVOf[i] : tauV));
        }
    }
    /// Write each member's reading — quantized position, whole-RPM velocity — into its fake.
    void write(World& world, int side) {
        for (std::size_t i = 0; i < 4; ++i) {
            const int m = static_cast<int>(i);
            if (m != dead) {
                const double ph = phaseCounts * kCount * static_cast<double>(i) / 4.0;
                const double th = m == slip ? thSlip : theta;
                lastPos[i] = std::floor((th + ph) / kCount) * kCount;
                lastVel[i] = std::trunc(vf[i] / kRpm) * kRpm;
                const bool stalled = wall && m != slip && wallTicks > 5;
                if (stalled && m == dither && (tick / 3) % 2 == 1) lastPos[i] += kCount;
                if (stalled && m == flicker && lastVel[i] == 0.0 && tick % 2 == 1) lastVel[i] = kRpm;
            }
            FakeMotor& fm = world.member(side, m);
            fm.setPosition(shulib::units::AngleDim{lastPos[i]});
            fm.setVelocity(shulib::units::AngularVelocity{lastVel[i]});
        }
        ++tick;
    }
};

/// Step both hand sides from what the loop commanded last tick, and write their readings.
void handTick(World& w, HandSide& left, HandSide& right) {
    left.step(w.group(0).commandedVoltage().value());
    right.step(w.group(1).commandedVoltage().value());
    left.write(w, 0);
    right.write(w, 1);
}

}  // namespace

// ═══ P2-1 healthy ═══════════════════════════════════════════════════════════════════════
TEST_CASE("T4 P2-1: 20 s of full-stick sweeps on a healthy robot, every member at 1 A (each side's "
          "TOTAL 4 A, over the 2.4 A limit) — zero cuts, zero FROZEN, zero absent, nothing said but "
          "one peak line a side and the disable summary") {
    // BUG CAUGHT: a protection that bites a healthy robot — the over-current judged on the side's
    // total (4 A > 2.4 A), a live member called FROZEN at a reversal, a cut on a stick transient.
    World w;
    w.allAmps(1.0);
    const auto log = w.run(2000, [&](int k) {
        const int p = (k / 250) % 8;  // forward, stop, turn left, stop, reverse, stop, turn right, stop
        const double f = p == 0 ? 1.0 : p == 4 ? -1.0 : 0.0;
        const double t = p == 2 ? 1.0 : p == 6 ? -1.0 : 0.0;
        w.stick(f, t);
    });
    REQUIRE(log.size() == 2000U);
    CHECK(log.back().st.cutCount == 0);
    CHECK(log.back().st.overCurrentWarnings == 0);
    CHECK(log.back().st.mode == ProtectionMode::Driving);
    int excluded = 0;
    for (const TickLog& t : log) {
        excluded += (t.excluded[0] != 0 || t.excluded[1] != 0) ? 1 : 0;
        CHECK(t.runtimeAbsent[0] == 0u);
        CHECK(t.runtimeAbsent[1] == 0u);
        CHECK(t.st.frozen[0] == '\0');
    }
    CHECK(excluded == 0);
    CHECK(log[100].mv[0] > 10.0);  // it really drove
    const auto lines = telLines(w.sink);
    REQUIRE(lines.size() == 4U);  // LEFT and RIGHT peak lines (the first stretch), 2 summaries
    CHECK(lines[0] == "drive: LEFT peak 1.00 A (port 16) over a 2.5 s drive, no cut");
    CHECK(lines[1] == "drive: RIGHT peak 1.00 A (port 11) over a 2.5 s drive, no cut");
    CHECK(lines[2].rfind("drive at the disable: LEFT, cuts 0 | port 16 1.00 A", 0) == 0);
    CHECK(lines[3].rfind("drive at the disable: RIGHT, cuts 0 | port 11 1.00 A", 0) == 0);
    CHECK(w.robot->faults().raiseCount(FaultCode::MotorGroupDisagree) == 0);
}

// ═══ P2-2 / P2-14 a FIGHT, and the four-argument perTick ═══════════════════════════════
TEST_CASE("T4 P2-2 + P2-14: a Reversed LEFT member — the monitor persists on its 25th tick and THAT "
          "tick writes 0 V to every drive motor; exactly 0.0 V for the 100 ticks after; driving again "
          "on the 101st; cut #2 exactly 25 moving ticks later; the state counts down") {
    // BUG CAUGHT: a cut that starts a tick late (no 0 V write on the deciding tick — drive() had
    // already sent the stick's volts), a cut that hands drive() the stick command, a cut length off
    // by one, streaks not reset at the re-arm, a fourth perTick argument never passed.
    World w;
    w.h.plant().setMemberFault(0, 2, MemberFault::Reversed);  // LEFT slot 2 = port 18
    const auto log = w.run(400, [&](int) { w.stick(1.0); });
    const std::vector<int> cuts = cutTicks(log);
    REQUIRE(cuts.size() >= 2U);
    // Tick 0 reads the robot at rest (the monitor needs 1 rad/s); tick 1 is the first moving read,
    // so the 25th disagreeing tick is tick 25.
    CHECK(cuts[0] == 25);
    CHECK(log[24].mv[0] > 1.0);          // driving the tick before
    CHECK(allDriveZero(log[25]));        // the deciding tick itself ends at 0 V
    for (int k = 26; k <= 125; ++k) {
        CAPTURE(k);
        CHECK(allDriveZero(log[static_cast<std::size_t>(k)]));
        CHECK(log[static_cast<std::size_t>(k)].st.cutTicksLeft == 125 - k);
        // In force through tick 124; tick 125 is the 100th zero tick AND the re-arm (its end).
        CHECK(log[static_cast<std::size_t>(k)].st.mode
              == (k < 125 ? ProtectionMode::Cut : ProtectionMode::Driving));
    }
    CHECK(log[25].st.cutTicksLeft == 100);
    CHECK(log[25].st.mode == ProtectionMode::Cut);
    CHECK(log[126].mv[0] > 1.0);  // the stick drives again on the 101st tick after the cut
    CHECK(log[126].st.mode == ProtectionMode::Driving);
    // Cut #2: the plant reads the restart one tick late (tick 127 is the first moving read), so the
    // streak's 25th tick is 151 — exactly the first cut's shape, started over.
    CHECK(cuts[1] == 151);
    CHECK(log[151].st.cutCount == 2);
    const auto lines = telLines(w.sink);
    CHECK(lines[0] == "CUT: FIGHT on LEFT -- port(s) 18 -- both sides 0 V for 1 s, cut #1, then re-arm");
    CHECK(countContaining(lines, "re-armed after cut #1 (1 s at 0 V)") == 1);
    CHECK(countContaining(lines, "  at the cut: LEFT port 18 ") == static_cast<int>(cuts.size()));
    CHECK(countContaining(lines, "  at the cut: ") == 8 * static_cast<int>(cuts.size()));
    CHECK(std::string{log[26].st.lastCut} == "CUT: FIGHT on LEFT -- port(s) 18");
    CHECK(log.back().st.cutCount == static_cast<int>(cuts.size()));
}

TEST_CASE("T4 P2-14: a THREE-argument perTick compiles and sees exactly what it saw before") {
    // BUG CAUGHT: the fourth argument breaking the three-argument form every program uses.
    std::vector<double> three;
    std::vector<double> four;
    for (int form = 0; form < 2; ++form) {
        World w;
        int k = 0;
        const auto disabled = [&] {
            if (k >= 60) return true;
            w.stick(k < 30 ? 1.0 : -0.5, k > 40 ? 0.3 : 0.0);
            return false;
        };
        const auto heading = [] { return HeadingSample{.radians = 0.0, .trusted = true}; };
        if (form == 0) {
            runDriverControl(w.robot->chassis(), w.pad, w.pacer, w.shaper, kCfg, disabled, heading,
                             [&](const ChassisSpeeds& c, bool connected, const DriveShaperState&) {
                                 three.push_back(c.vx().value() + c.omega().value()
                                                 + (connected ? 1000.0 : 0.0));
                                 ++k;
                             });
        } else {
            runDriverControl(w.robot->chassis(), w.pad, w.pacer, w.shaper, kCfg, disabled, heading,
                             [&](const ChassisSpeeds& c, bool connected, const DriveShaperState&,
                                 const DriveProtectionState&) {
                                 four.push_back(c.vx().value() + c.omega().value()
                                                + (connected ? 1000.0 : 0.0));
                                 ++k;
                             });
        }
    }
    REQUIRE(three.size() == 60U);
    CHECK(three == four);
}

TEST_CASE("T4 P2-14b: perTick's `command` is the DRIVER'S request, also while a cut sends the drive "
          "zero — in both forms of perTick (teleop_loop.hpp's documented contract)") {
    // BUG CAUGHT (the review of 2026-09-30, S16): the panel handed the zero command during a cut —
    // a panel that shows the stick would read "released" while the driver is holding it, and could
    // not tell a cut from a let-go stick.
    for (int form = 0; form < 2; ++form) {
        CAPTURE(form);
        World w;
        w.h.plant().setMemberFault(0, 2, MemberFault::Reversed);  // a fight: cut at tick 25
        std::vector<double> vx;
        std::vector<double> volts;  // LEFT slot 0's commanded volts, the same tick
        std::vector<ProtectionMode> modes;
        int k = 0;
        const auto disabled = [&] {
            if (k >= 60) return true;
            w.stick(1.0);
            return false;
        };
        const auto heading = [] { return HeadingSample{.radians = 0.0, .trusted = true}; };
        if (form == 0) {
            runDriverControl(w.robot->chassis(), w.pad, w.pacer, w.shaper, kCfg, disabled, heading,
                             [&](const ChassisSpeeds& c, bool, const DriveShaperState&,
                                 const DriveProtectionState& st) {
                                 vx.push_back(c.vx().value());
                                 volts.push_back(w.member(0, 0).commandedVoltage().value());
                                 modes.push_back(st.mode);
                                 ++k;
                             });
        } else {
            runDriverControl(w.robot->chassis(), w.pad, w.pacer, w.shaper, kCfg, disabled, heading,
                             [&](const ChassisSpeeds& c, bool, const DriveShaperState&) {
                                 vx.push_back(c.vx().value());
                                 volts.push_back(w.member(0, 0).commandedVoltage().value());
                                 ++k;
                             });
        }
        REQUIRE(vx.size() == 60U);
        CHECK(vx[20] > 0.0);
        CHECK(volts[20] > 1.0);
        for (int t = 25; t < 60; ++t) {
            CAPTURE(t);
            CHECK(volts[static_cast<std::size_t>(t)] == 0.0);  // the drive IS cut …
            CHECK(vx[static_cast<std::size_t>(t)] == vx[20]);  // … and the panel still sees the stick
        }
        if (form == 0) {
            CHECK(modes[30] == ProtectionMode::Cut);
        }
    }
}

// ═══ P2-3 SignFlipped on the RIGHT ═════════════════════════════════════════════════════════
TEST_CASE("T4 P2-3: a SignFlipped RIGHT member is cut on exactly as a LEFT one — a protection that "
          "judged only the first group would never cut") {
    World w;
    w.h.plant().setMemberFault(1, 1, MemberFault::SignFlipped);  // RIGHT slot 1 = port 12
    const auto log = w.run(200, [&](int) { w.stick(1.0); });
    const std::vector<int> cuts = cutTicks(log);
    REQUIRE_FALSE(cuts.empty());
    CHECK(cuts[0] == 25);
    CHECK(allDriveZero(log[25]));
    CHECK(telLines(w.sink)[0]
          == "CUT: FIGHT on RIGHT -- port(s) 12 -- both sides 0 V for 1 s, cut #1, then re-arm");
}

TEST_CASE("T4 P2-2b: BOTH sides fight from the same tick — ONE cut, and its line names the FIRST group "
          "that persisted (LEFT, the Robot's order)") {
    // BUG CAUGHT (the review of 2026-09-30, S17): the fight's group chosen as the LAST one that
    // persisted — a cut both sides called at once would name RIGHT, and a pit crew reading the log
    // would look at the wrong side first. One cut either way: the check runs once a tick.
    World w;
    w.h.plant().setMemberFault(0, 2, MemberFault::Reversed);  // LEFT slot 2 = port 18
    w.h.plant().setMemberFault(1, 1, MemberFault::Reversed);  // RIGHT slot 1 = port 12
    const auto log = w.run(60, [&](int) { w.stick(1.0); });
    REQUIRE(log.size() == 60U);
    CHECK(log[25].persisted[0] > 0);  // both monitors persisted on the deciding tick
    CHECK(log[25].persisted[1] > 0);
    const std::vector<int> cuts = cutTicks(log);
    REQUIRE(cuts.size() == 1U);
    CHECK(cuts[0] == 25);
    CHECK(telLines(w.sink)[0]
          == "CUT: FIGHT on LEFT -- port(s) 18 -- both sides 0 V for 1 s, cut #1, then re-arm");
}

TEST_CASE("T4 P2-2c: EVERY member of a side reading opposite to its command — the WHOLE-SIDE words, "
          "every port named") {
    // BUG CAUGHT (the review of 2026-09-30, S19): the whole-side words never chosen — a side whose
    // reads are all backwards (a side wired or configured the wrong way round) reported as a fight
    // of four ports, instead of the one fault it is.
    World w;
    for (int i = 0; i < 4; ++i) w.h.plant().setMemberFault(0, i, MemberFault::SignFlipped);
    const auto log = w.run(60, [&](int) { w.stick(1.0); });
    const std::vector<int> cuts = cutTicks(log);
    REQUIRE_FALSE(cuts.empty());
    CHECK(cuts[0] == 25);
    CHECK(log[25].persisted[0] == 4);
    CHECK(telLines(w.sink)[0]
          == "CUT: whole LEFT side OPPOSITE/stalled vs command (ports 16 17 18 20) -- both sides 0 V "
             "for 1 s, cut #1, then re-arm");
    CHECK(std::string{log[26].st.lastCut}
          == "CUT: whole LEFT side OPPOSITE/stalled vs command (ports 16 17 18 20)");
}

// ═══ P2-4 over-current, at its boundaries ════════════════════════════════════════════════
TEST_CASE("T4 P2-4: over-current at its boundaries — 2.41 A for 24 ticks no, 25 yes; 2.40 A never; "
          "one tick at 2.39 A starts it over; the RIGHT side the same; an ABSENT member never; a "
          "released stick never; with the switch OFF, warned and counted, never cut") {
    // BUG CAUGHT: `>=` for `>`; the window 24; a streak not reset by a dip; the protections judged
    // on the first group only; an absent member's current counted; the command floor ignored; the
    // switch ignored.
    SUBCASE("2.41 A for 24 ticks: no cut; for 25: a cut on the 25th") {
        for (const int n : {24, 25}) {
            World w;
            const auto log = w.run(80, [&](int k) {
                w.stick(1.0);
                w.member(0, 1).setCurrent(shulib::units::Current{k >= 10 && k < 10 + n ? 2.41 : 1.0});
            });
            CAPTURE(n);
            const std::vector<int> cuts = cutTicks(log);
            if (n == 24) {
                CHECK(cuts.empty());
            } else {
                REQUIRE(cuts.size() == 1U);
                CHECK(cuts[0] == 34);
                CHECK(telLines(w.sink)[0]
                      == "CUT: port 17 (LEFT) over 2.4 A for 250 ms -- both sides 0 V for 1 s, cut "
                         "#1, then re-arm");
            }
        }
    }
    SUBCASE("2.40 A for 1000 ticks: never (the rule is strictly above)") {
        World w;
        const auto log = w.run(1000, [&](int) {
            w.stick(1.0);
            w.member(0, 1).setCurrent(shulib::units::Current{2.40});
        });
        CHECK(cutTicks(log).empty());
    }
    SUBCASE("one tick at 2.39 A starts the streak over; a re-arm starts it over too") {
        World w;
        const auto log = w.run(300, [&](int k) {
            w.stick(1.0);
            w.member(0, 1).setCurrent(shulib::units::Current{k >= 10 && k != 30 ? 2.41 : 1.0});
        });
        const std::vector<int> cuts = cutTicks(log);
        REQUIRE(cuts.size() >= 2U);
        CHECK(cuts[0] == 55);   // 31 .. 55 is the unbroken 25
        CHECK(cuts[1] == 180);  // the re-arm at 155; 156 .. 180 is the next unbroken 25
    }
    SUBCASE("the 25-tick case on the RIGHT side: a cut") {
        World w;
        const auto log = w.run(80, [&](int k) {
            w.stick(1.0);
            w.member(1, 3).setCurrent(shulib::units::Current{k >= 10 ? 2.41 : 1.0});
        });
        const std::vector<int> cuts = cutTicks(log);
        REQUIRE(cuts.size() == 1U);
        CHECK(cuts[0] == 34);
        CHECK(telLines(w.sink)[0].rfind("CUT: port 15 (RIGHT) over 2.4 A", 0) == 0);
    }
    SUBCASE("the 24- and 25-tick cases driving in REVERSE (stick -1): no cut, then a cut on the "
            "25th — the command floor is judged on |volts|") {
        // BUG CAUGHT (the review of 2026-09-30, S1): the floor judged on the SIGNED command, so a
        // side driven backward — a robot pushed while reversing — never counts at all.
        for (const int n : {24, 25}) {
            World w;
            const auto log = w.run(80, [&](int k) {
                w.stick(-1.0);
                w.member(0, 1).setCurrent(shulib::units::Current{k >= 10 && k < 10 + n ? 2.41 : 1.0});
            });
            CAPTURE(n);
            REQUIRE(log[20].mv[0] < -1.0);  // it really drove backward
            const std::vector<int> cuts = cutTicks(log);
            if (n == 24) {
                CHECK(cuts.empty());
            } else {
                REQUIRE(cuts.size() == 1U);
                CHECK(cuts[0] == 34);
                CHECK(telLines(w.sink)[0].rfind("CUT: port 17 (LEFT) over 2.4 A for 250 ms", 0) == 0);
            }
        }
    }
    SUBCASE("a runtime-ABSENT member at 2.5 A never cuts") {
        World w;
        REQUIRE(w.group(0).markAbsent(1));
        const auto log = w.run(1000, [&](int) {
            w.stick(1.0);
            w.member(0, 1).setCurrent(shulib::units::Current{2.5});
        });
        CHECK(cutTicks(log).empty());
        CHECK(log.back().st.mode == ProtectionMode::Degraded);
    }
    SUBCASE("2.5 A with the stick RELEASED (the side at 0 V) for 1000 ticks never cuts") {
        World w;
        const auto log = w.run(1000, [&](int) {
            w.stick(0.0);
            w.member(0, 1).setCurrent(shulib::units::Current{2.5});
        });
        CHECK(cutTicks(log).empty());
    }
    SUBCASE("overCurrentCuts = false: a WARNING per episode, counted, never a cut") {
        World w;
        DriverLoopConfig cfg = kCfg;
        cfg.protection.overCurrentCuts = false;
        const auto log = w.run(
            400,
            [&](int k) {
                w.stick(1.0);
                const bool on = (k >= 10 && k < 110) || (k >= 200 && k < 300);
                w.member(0, 1).setCurrent(shulib::units::Current{on ? 2.5 : 1.0});
            },
            cfg);
        CHECK(cutTicks(log).empty());
        CHECK(log.back().st.overCurrentWarnings == 2);
        CHECK(log[33].st.overCurrentWarnings == 0);
        CHECK(log[34].st.overCurrentWarnings == 1);
        const auto lines = telLines(w.sink);
        CHECK(countContaining(lines, "WARNING: port 17 (LEFT) over 2.4 A for 250 ms -- over-current "
                                     "cuts are OFF (DriveProtectionConfig::overCurrentCuts): not "
                                     "cut, warning #")
              == 2);
        for (const TickLog& t : log) CHECK(t.mv[0] > 1.0);  // it never stopped driving
    }
}

TEST_CASE("T4 P2-4c: the command floor is STRICT — a side commanded exactly 1 V counts no over-current "
          "and starts no driven stretch; a hair above 1 V does both") {
    // BUG CAUGHT (the review of 2026-09-30, S3): the floor made inclusive — a side held AT the fight
    // monitor's 1 V floor, where the monitor does not judge either, counting toward a cut, or
    // printing a peak line for a stretch nothing drove. Through DriveProtection's public calls (a
    // program with a loop of its own), because the loop's command pipeline never lands on exactly
    // 1.0 V: each tick sets both groups' command, then runs the two checks around where drive() is.
    World w;
    DriveProtection p{w.robot->chassis(), DriveProtectionConfig{}, Time{0.01}};
    p.enter();
    w.member(0, 1).setCurrent(shulib::units::Current{2.5});  // port 17, over the limit throughout
    const auto tickAt = [&](double volts) {
        w.group(0).setVoltage(shulib::units::Voltage{volts});
        w.group(1).setVoltage(shulib::units::Voltage{volts});
        p.beforeDrive();
        p.afterDrive();
    };
    for (int k = 0; k < 100; ++k) tickAt(1.0);
    tickAt(0.0);
    CHECK(p.state().cutCount == 0);
    CHECK(telLines(w.sink).empty());  // no peak line: nothing was driven ABOVE the floor
    constexpr double kHair = 1.0 + 1e-9;
    for (int k = 0; k < 24; ++k) tickAt(kHair);
    CHECK(p.state().cutCount == 0);
    tickAt(kHair);
    CHECK(p.state().cutCount == 1);  // the 25th tick above the floor
    CHECK(telLines(w.sink)[0].rfind("CUT: port 17 (LEFT) over 2.4 A for 250 ms", 0) == 0);
}

// ═══ P2-4b a FROZEN current ══════════════════════════════════════════════════════════════
TEST_CASE("T4 P2-4b: a member that dies at the current limit during a push cuts ONCE — no cut while "
          "the robot stands (the command floor), none when it drives away (FROZEN, then ABSENT)") {
    // BUG CAUGHT (the 2026-09-27 re-check's finding 2): a dead port's held 2.5 A cutting a standing
    // robot every 1.25 s for the rest of the match; a FROZEN member's stale current counted.
    World w;
    constexpr int kDrive = 325;  // the drive away, at half stick
    const auto log = w.run(420, [&](int k) {
        if (k < 25) {
            w.stick(1.0);
            w.allAmps(2.5);  // the push: every member at the limit
        } else {
            w.stick(k >= kDrive ? 0.5 : 0.0);
            if (k == 30) {
                w.h.plant().setMemberFault(0, 1, MemberFault::Frozen);  // port 17 dies in the cut
                w.allAmps(0.6);
                w.member(0, 1).setCurrent(shulib::units::Current{2.5});  // its current held
            }
        }
    });
    const std::vector<int> cuts = cutTicks(log);
    REQUIRE(cuts.size() == 1U);  // the push's own cut, and nothing after it
    CHECK(cuts[0] == 24);
    // Frozen at rest: nothing is SUSPECT until a moving mate passes 1 rad/s — the first moving read
    // after the drive away (the plant reads a step one tick late).
    CHECK(frozenTick(log, 0, 1) == kDrive + 1);
    CHECK(absentTick(log, 0, 1) == kDrive + 1 + 24);
    CHECK(log.back().st.mode == ProtectionMode::Degraded);
    // What a panel reads (the review of 2026-09-30, S8 / S9b — P2-14 pinned only the mode and the
    // cut's count): the FROZEN member by its port on each of its 25 FROZEN ticks — the marking tick
    // included — and on no other; each side's highest current this entry; the group count; and
    // the MID-RUN verdict once it is dead.
    const int f = frozenTick(log, 0, 1);
    REQUIRE(f > 0);
    for (int k = 0; k < static_cast<int>(log.size()); ++k) {
        CAPTURE(k);
        const bool frozenNow = k >= f && k <= f + 24;
        CHECK(std::string{log[static_cast<std::size_t>(k)].st.frozen} == (frozenNow ? "17" : ""));
    }
    const DriveProtectionState& last = log.back().st;
    CHECK(std::string{last.dead} == "17");
    CHECK(last.groupCount == 2U);
    CHECK(last.peakAmps[0] == 2.5);  // the push, every member at 2.5 A
    CHECK(last.peakAmps[1] == 2.5);
    CHECK(last.verdict.verdict == DriveVerdict::DriveDegraded);
    CHECK(last.verdict.leftDead == 1);
    CHECK(last.verdict.rightDead == 0);
    CHECK(last.verdict.totalDead == 1);
    CHECK(log[static_cast<std::size_t>(f) - 1].st.verdict.verdict == DriveVerdict::Drive);
}

// ═══ P2-5 a member frozen AT REST, then driven ═════════════════════════════════════════════
TEST_CASE("T4 P2-5: a member frozen at rest is FROZEN on the first tick a moving mate reads above "
          "1 rad/s and ABSENT 24 ticks later — one line, no cut, no MOTOR_GROUP_DISAGREE; out of the "
          "reads, still commanded") {
    // BUG CAUGHT: markAbsent leaving the member in the monitor (a DISAGREE raised), or in the
    // median, or no longer commanded; a death at rest never seen (the mates branch).
    World w;
    const auto log = w.run(250, [&](int k) {
        if (k == 10) w.h.plant().setMemberFault(0, 2, MemberFault::Frozen);  // port 18, at rest
        w.stick(k >= 100 ? 1.0 : 0.0);
    });
    CHECK(frozenTick(log, 0, 2) == 101);
    CHECK(absentTick(log, 0, 2) == 125);
    CHECK(cutTicks(log).empty());
    CHECK(w.robot->faults().raiseCount(FaultCode::MotorGroupDisagree) == 0);
    const auto lines = telLines(w.sink);
    REQUIRE(countContaining(lines, "went ABSENT") == 1);
    CHECK(lines[0]
          == "WARNING: port 18 (LEFT) went ABSENT at runtime: reads frozen for 250 ms while its side "
             "answered -- dropped from the fight detector, still commanded (harmless)");
    CHECK(lines[1]
          == "degradation re-evaluated: DRIVE_DEGRADED (left dead 1, right dead 0, total 1) -- "
             "dead: 18");
    CHECK(std::string{log.back().st.dead} == "18");
    // The panel's state on the way (the review of 2026-09-30): FROZEN names the port from its first
    // FROZEN tick; the verdict turns DriveDegraded on the marking tick, not before.
    CHECK(std::string{log[100].st.frozen}.empty());
    CHECK(std::string{log[101].st.frozen} == "18");
    CHECK(std::string{log[100].st.dead}.empty());
    CHECK(log[124].st.verdict.verdict == DriveVerdict::Drive);
    CHECK(log[125].st.verdict.verdict == DriveVerdict::DriveDegraded);
    CHECK(log[125].st.verdict.leftDead == 1);
    CHECK(log[125].st.verdict.totalDead == 1);
    // Out of the reads: the group's median is the three live members'.
    MotorGroup& left = w.group(0);
    CHECK(left.presentCount() == 3);
    CHECK(left.position().value() == w.member(0, 0).position().value());
    CHECK(left.position().value() != w.member(0, 2).position().value());
    // Still commanded: its fake got the side's volts on the last tick.
    CHECK(log.back().mv[2] == log.back().mv[0]);
    CHECK(log.back().mv[2] > 1.0);
}

TEST_CASE("T4 P2-5b: the same death at rest, the robot then driven BACKWARD — FROZEN on the first "
          "moving read, ABSENT 24 ticks later, no cut: the mates' speed is judged by its magnitude") {
    // BUG CAUGHT (the review of 2026-09-30, F7 — every case above drove forward): the mates'
    // fastest judged SIGNED, so mates turning backward are no evidence the train turns — the port
    // that died at rest is never seen when the robot backs away, stays in the monitor with its
    // stale 0, and the monitor CUTS on it (N2 (a)'s race, lost) with MOTOR_GROUP_DISAGREE raised.
    World w;
    const auto log = w.run(250, [&](int k) {
        if (k == 10) w.h.plant().setMemberFault(0, 2, MemberFault::Frozen);  // port 18, at rest
        w.stick(k >= 100 ? -1.0 : 0.0);
    });
    REQUIRE(log[150].mv[0] < -1.0);  // it really drove backward
    CHECK(frozenTick(log, 0, 2) == 101);
    CHECK(absentTick(log, 0, 2) == 125);
    CHECK(cutTicks(log).empty());
    CHECK(w.robot->faults().raiseCount(FaultCode::MotorGroupDisagree) == 0);
}

TEST_CASE("T4 P2-5c: what a FROZEN member's current is NOT part of — the panel's peakAmps and the push "
          "check's peak line both leave it out, whatever it reads, on every one of its FROZEN ticks") {
    // BUG CAUGHT (the review of 2026-09-30's S9, which no case reached, and its twin in the peak
    // line's driven stretch): a FROZEN member's current counted in a peak — DriveProtectionState's
    // documented "over its present members that were not FROZEN", and the stretch's own rule. A
    // DEAD port's held current only repeats a value it gave before it was FROZEN (A4 register
    // HA-175 (a)), so a peak could not show the difference; a LIVE member wrongly FROZEN at a wall
    // reads a real current that can climb while it is FROZEN, and that is the case built here:
    // P2-5's world (port 18 dies at rest at tick 10, the robot drives from 100: FROZEN 101-124,
    // ABSENT at 125), every member at 1.0 A, and port 18 reading 2.2 A on its FROZEN ticks 102-124
    // only — under the 2.4 A limit, so nothing is cut.
    World w;
    w.allAmps(1.0);
    const auto log = w.run(260, [&](int k) {
        if (k == 10) w.h.plant().setMemberFault(0, 2, MemberFault::Frozen);  // port 18, at rest
        w.stick(k >= 100 && k < 200 ? 1.0 : 0.0);
        w.member(0, 2).setCurrent(shulib::units::Current{k >= 102 && k <= 124 ? 2.2 : 1.0});
    });
    REQUIRE(frozenTick(log, 0, 2) == 101);
    REQUIRE(absentTick(log, 0, 2) == 125);
    CHECK(cutTicks(log).empty());
    for (int k = 101; k <= 124; ++k) {
        CAPTURE(k);
        CHECK(std::string{log[static_cast<std::size_t>(k)].st.frozen} == "18");
        CHECK(log[static_cast<std::size_t>(k)].st.peakAmps[0] == 1.0);
    }
    CHECK(log.back().st.peakAmps[0] == 1.0);
    CHECK(log.back().st.peakAmps[1] == 1.0);
    const auto lines = telLines(w.sink);
    CHECK(countContaining(lines, "drive: LEFT peak 1.00 A (port 16) over a 1.0 s drive, no cut") == 1);
    CHECK(countContaining(lines, "drive: LEFT peak 2.20 A") == 0);
}

// ═══ P2-6 THE RACE ════════════════════════════════════════════════════════════════════════
TEST_CASE("T4 P2-6: a member dies TURNING and the stick reverses on the same tick — ZERO cuts, its "
          "monitor streak never above 5, ABSENT 24 ticks after it was first FROZEN (the reversal + "
          "29)") {
    // BUG CAUGHT (the first proposal, and the brief's mutations "SUSPECT after persistTicks reads"
    // and "the frozen-reading rule dropped"): the fight monitor cutting on a dead member's stale +v
    // before the absence rule marks it.
    World w;
    constexpr int kF = 100;
    const auto log = w.run(250, [&](int k) {
        if (k == kF) w.h.plant().setMemberFault(0, 1, MemberFault::Frozen);
        w.stick(k >= kF ? -1.0 : 1.0);
    });
    CHECK(cutTicks(log).empty());
    int streak = 0;
    int maxStreak = 0;
    for (const TickLog& t : log) {
        streak = (t.disagreeing[0] & 0b0010u) != 0 ? streak + 1 : 0;
        maxStreak = std::max(maxStreak, streak);
    }
    CHECK(maxStreak <= 5);
    CHECK(frozenTick(log, 0, 1) == kF + 5);
    CHECK(absentTick(log, 0, 1) == kF + 29);
    CHECK(w.robot->faults().raiseCount(FaultCode::MotorGroupDisagree) == 0);
}

// ═══ P2-6b THE ORDER (hand-stepped) ═════════════════════════════════════════════════════════
TEST_CASE("T4 P2-6b: a member that FIGHTS for 20 ticks, then freezes: at its 5th repeated read its "
          "monitor streak would reach 25 — judged BEFORE drive() it is FROZEN and out of the monitor "
          "that tick: no cut, ABSENT 24 ticks later") {
    // BUG CAUGHT (the brief's mutation "the FROZEN exclusion applied AFTER drive()"): the monitor
    // judging the member present on the tick the rule catches it — a 1 s cut on a dead port.
    World w{World::Mode::Hand};
    HandSide right;
    std::array<double, 4> pos{};
    constexpr double kV = 20.0;
    constexpr int kFight = 10;  // ticks 10..29 fight; from tick 30 member 0's readings are held
    const auto log = w.run(120, [&](int k) {
        w.stick(1.0);
        right.step(w.group(1).commandedVoltage().value());
        right.write(w, 1);
        for (int i = 0; i < 4; ++i) {
            FakeMotor& m = w.member(0, i);
            if (i == 0 && k >= kFight + 20) continue;  // dead: holds its last fighting reading
            const double v = (i == 0 && k >= kFight) ? -kV : (k >= 1 ? kV : 0.0);
            pos[static_cast<std::size_t>(i)] += v * kDt;
            m.setPosition(shulib::units::AngleDim{pos[static_cast<std::size_t>(i)]});
            m.setVelocity(shulib::units::AngularVelocity{v});
        }
    });
    CHECK(cutTicks(log).empty());
    // The streak: ticks 10..33 disagreeing (24), 34 would have been the 25th.
    CHECK((log[33].disagreeing[0] & 1u) != 0);
    CHECK(frozenTick(log, 0, 0) == 34);
    CHECK((log[34].disagreeing[0] & 1u) == 0);
    CHECK(absentTick(log, 0, 0) == 34 + 24);
}

// ═══ P2-6c after a cut, reversed at the re-arm ════════════════════════════════════════════
TEST_CASE("T4 P2-6c: a member frozen at +v two ticks before an over-current cut is ABSENT DURING the "
          "cut (its own stale speed makes it SUSPECT; its standing mates are not), and a reversal at "
          "the re-arm cuts nothing") {
    // BUG CAUGHT (the brief's mutation "the mates' fastest counting a member whose position
    // repeats"): the dead member's stale 30 rad/s making every STANDING mate SUSPECT during the cut
    // — the side outvotes it, it is never marked, and the reversal at the re-arm reads it as a fight.
    World w;
    const auto log = w.run(260, [&](int k) {
        w.stick(k >= 125 ? -1.0 : 1.0);  // the reversal at the re-arm
        w.member(0, 3).setCurrent(shulib::units::Current{k < 25 ? 2.5 : 1.0});  // port 20: the cut
        if (k == 22) w.h.plant().setMemberFault(0, 1, MemberFault::Frozen);    // port 17 dies
    });
    const std::vector<int> cuts = cutTicks(log);
    REQUIRE(cuts.size() == 1U);
    CHECK(cuts[0] == 24);
    CHECK(frozenTick(log, 0, 1) == 27);
    CHECK(absentTick(log, 0, 1) == 51);  // during the cut (24 .. 124)
    CHECK(log[51].st.mode == ProtectionMode::Cut);
    CHECK(w.robot->faults().raiseCount(FaultCode::MotorGroupDisagree) == 0);
}

// ═══ P2-6d a deceleration through zero (hand-stepped) ══════════════════════════════════════
TEST_CASE("T4 P2-6d: a member held at +v while its mates decelerate linearly through zero (under "
          "1 rad/s for 9 ticks) is FROZEN from its 5th repeat on EVERY tick after — its own stale "
          "speed keeps it SUSPECT — zero cuts, ABSENT at the pinned tick; and the same MIRRORED "
          "(held at -v, the robot backing and then driving forward)") {
    // BUG CAUGHT (the brief's mutation "SUSPECT only through the mates"): the member leaving FROZEN
    // at the turnaround, the latch moving, and the monitor (with its stale speed as the side's
    // fastest) judging it again. The MIRROR (the review of 2026-09-30, F8): the own-speed branch
    // judged on the SIGNED speed — a member that died turning backward is then SUSPECT only through
    // its mates, so it leaves FROZEN while they pass through zero, exactly the hole this case
    // closes forward.
    for (const double s : {1.0, -1.0}) {
        CAPTURE(s);
        World w{World::Mode::Hand};
        HandSide right;
        std::array<double, 4> pos{};
        const double kV = 3.0 * s;
        const auto log = w.run(120, [&](int k) {
            w.stick(k >= 10 ? -s : s);
            right.step(w.group(1).commandedVoltage().value());
            right.write(w, 1);
            const double mates = k < 10   ? kV
                                 : k < 40 ? kV * (1.0 - 2.0 * static_cast<double>(k - 10) / 30.0)
                                          : -kV;
            for (int i = 0; i < 4; ++i) {
                if (i == 0 && k >= 10) continue;  // held at its last reading: 3 rad/s, signed
                const double v = (k == 0) ? 0.0 : (i == 0 ? kV : mates);
                pos[static_cast<std::size_t>(i)] += v * kDt;
                w.member(0, i).setPosition(shulib::units::AngleDim{pos[static_cast<std::size_t>(i)]});
                w.member(0, i).setVelocity(shulib::units::AngularVelocity{v});
            }
        });
        CHECK(cutTicks(log).empty());
        const int first = frozenTick(log, 0, 0);
        CHECK(first == 14);
        const int latched = absentTick(log, 0, 0);
        CHECK(latched == 38);
        for (int k = first; k < latched && k >= 0; ++k) {
            CAPTURE(k);
            CHECK((log[static_cast<std::size_t>(k)].excluded[0] & 1u) != 0);  // on EVERY tick
        }
    }
}

// ═══ P2-7 the floor, per N7 (c) ═════════════════════════════════════════════════════════════
TEST_CASE("T4 P2-7 (N7 (c)): two deaths on ONE side DRIVE on, degraded, with no REFUSE; a third, on the "
          "other side, REFUSES — exactly 0 V from that tick until the disable, ONE refusal — and the "
          "next entry is REFUSED from tick 1") {
    // BUG CAUGHT (the brief's mutations "the per-side floor applied mid-run under N7 (c)" and "the
    // REFUSE not re-applied at entry"): robot two stopped for the match at its second death on one
    // side; a refusal that a disable and a restart of driver control quietly lifts.
    World w;
    const auto log = w.run(300, [&](int k) {
        if (k == 5) {
            w.h.plant().setMemberFault(0, 1, MemberFault::Frozen);
            w.h.plant().setMemberFault(0, 2, MemberFault::Frozen);
        }
        if (k == 150) w.h.plant().setMemberFault(1, 3, MemberFault::Frozen);  // port 15
        w.stick(k >= 20 ? 1.0 : 0.0);
    });
    const int twoDead = absentTick(log, 0, 1);
    CHECK(twoDead == 45);                    // FROZEN from 21 (a tie: 2 vs 2), latched at 45
    CHECK(absentTick(log, 0, 2) == twoDead);
    CHECK(log[static_cast<std::size_t>(twoDead)].st.mode == ProtectionMode::Degraded);
    CHECK(log[static_cast<std::size_t>(twoDead) + 10].mv[0] > 1.0);  // it drives on
    const int third = absentTick(log, 1, 3);
    REQUIRE(third > 150);
    CHECK(log[static_cast<std::size_t>(third) - 1].mv[4] > 1.0);
    for (std::size_t k = static_cast<std::size_t>(third); k < log.size(); ++k) {
        CAPTURE(k);
        CHECK(allDriveZero(log[k]));
        CHECK(log[k].st.mode == ProtectionMode::Refused);
    }
    const auto lines = telLines(w.sink);
    CHECK(countContaining(lines, "REFUSED") == 1);
    CHECK(countContaining(lines, "REFUSED: more than 2 motors dead in total -- 0 V for the rest of "
                                 "the run, every later start of driver control included; restart the "
                                 "program once the robot is fixed")
          == 1);
    CHECK(std::string{log.back().st.dead} == "17 18 15");
    // The next entry: REFUSED at entry, 0 V from tick 1, whatever the stick says.
    const int before = w.sink.size();
    const auto again = w.run(30, [&](int) { w.stick(1.0); });
    for (const TickLog& t : again) {
        CHECK(allDriveZero(t));
        CHECK(t.st.mode == ProtectionMode::Refused);
    }
    const auto entry = telLines(w.sink, before);
    REQUIRE(entry.size() >= 2U);
    CHECK(entry[0]
          == "REFUSED at the start of driver control: more than 2 motors dead in total (left dead 2, "
             "right dead 1, total 3) -- 0 V for the rest of the run");
    CHECK(entry[1] == "  dead: 17 18 15 -- restart the program once the robot is fixed");
}

TEST_CASE("T4 P2-7b: a third death that latches DURING a cut REFUSES at once — 0 V from that tick, no "
          "re-arm ever, and the panel reads no cut left (Refused, cutTicksLeft 0)") {
    // BUG CAUGHT (the review of 2026-09-30, S7): a refusal that leaves the cut's countdown in the
    // state — a panel reading "cut, 0.7 s left" on a robot that will not drive again this run.
    // P2-6c's world (port 20 over current until the cut at 24; port 17 dies at +v at 22 and is
    // marked during the cut, at 51) with RIGHT ports 11 and 12 already dead: port 17 is the third.
    World w;
    REQUIRE(w.group(1).markAbsent(0));
    REQUIRE(w.group(1).markAbsent(1));
    const auto log = w.run(200, [&](int k) {
        w.stick(1.0);
        w.member(0, 3).setCurrent(shulib::units::Current{k < 25 ? 2.5 : 1.0});  // port 20: the cut
        if (k == 22) w.h.plant().setMemberFault(0, 1, MemberFault::Frozen);    // port 17 dies
    });
    const std::vector<int> cuts = cutTicks(log);
    REQUIRE(cuts.size() == 1U);
    CHECK(cuts[0] == 24);
    const int third = absentTick(log, 0, 1);
    CHECK(third == 51);
    REQUIRE(third > 25);
    CHECK(log[static_cast<std::size_t>(third) - 1].st.mode == ProtectionMode::Cut);
    CHECK(log[static_cast<std::size_t>(third) - 1].st.cutTicksLeft == 100 - (third - 1 - 24));
    for (std::size_t k = static_cast<std::size_t>(third); k < log.size(); ++k) {
        CAPTURE(k);
        CHECK(log[k].st.mode == ProtectionMode::Refused);
        CHECK(log[k].st.cutTicksLeft == 0);
        CHECK(allDriveZero(log[k]));
    }
    const auto lines = telLines(w.sink);
    CHECK(countContaining(lines, "REFUSED: more than 2 motors dead in total") == 1);
    CHECK(countContaining(lines, "re-armed") == 0);  // a refusal is not a cut: it never re-arms
    CHECK(std::string{log.back().st.dead} == "17 11 12");
}

// ═══ P2-8 port 18's slip ══════════════════════════════════════════════════════════════════
TEST_CASE("T4 P2-8: a slipping member (the plant's Decoupled at drag 0.8, HA-130's port 18) over 20 s "
          "of sweeps is never absent and never cut") {
    // BUG CAUGHT: every rule that says "a mate turns, so this one should" tried against the robot's
    // one known slipping member.
    World w;
    w.h.plant().setMemberFault(0, 2, MemberFault::Decoupled);
    const auto log = w.run(2000, [&](int k) {
        const int p = (k / 150) % 4;
        w.stick(p == 0 ? 1.0 : p == 1 ? 0.5 : p == 2 ? -1.0 : -0.25, p == 1 ? 0.4 : 0.0);
    });
    CHECK(cutTicks(log).empty());
    CHECK(log.back().runtimeAbsent[0] == 0u);
    CHECK(longestLiveFrozenRun(log) == 0);
}

// ═══ P2-9 the median when a member drops ═══════════════════════════════════════════════════
TEST_CASE("T4 P2-9: a member marked absent mid-run moves its side's median by no more than the gap "
          "between the two middle members, and the odometry raises no ODO_STUCK") {
    // BUG CAUGHT: a jump in the side's reported position at the mark — a phantom tick of travel the
    // odometry would integrate (if this goes red, the fix is a continuity offset in MotorGroup at
    // the change, never a looser gate).
    // Two members slip (drag 0.8, HA-130's shape), decoupled at different ticks so they read
    // apart; the dying member ends up BELOW both live readings. At the mark the middle two of the
    // four are the live member and the nearer slipper — so the median really moves.
    SimHarnessConfig slipCfg = worldConfig();
    slipCfg.plant.decoupledDrag = 0.8;
    World w{World::Mode::Plant, robotTwoShape(), slipCfg};
    double beforeMedian = 0.0;
    double gap = 0.0;
    double afterMedian = 0.0;
    int markTick = -1;
    int k = 0;
    std::vector<double> held(4);
    runDriverControl(
        w.robot->chassis(), w.pad, w.pacer, w.shaper, kCfg,
        [&] {
            if (k >= 300) return true;
            if (k == 10) w.h.plant().setMemberFault(0, 3, MemberFault::Decoupled);
            if (k == 60) w.h.plant().setMemberFault(0, 2, MemberFault::Decoupled);
            if (k == 100) w.h.plant().setMemberFault(0, 1, MemberFault::Frozen);
            w.stick(1.0);
            // The four readings this tick (the dying member's included), before the protections run.
            for (int i = 0; i < 4; ++i) held[static_cast<std::size_t>(i)] = w.member(0, i).position().value();
            return false;
        },
        [] { return HeadingSample{.radians = 0.0, .trusted = true}; },
        [&](const ChassisSpeeds&, bool, const DriveShaperState&) {
            if (markTick < 0 && w.group(0).runtimeAbsentMask() != 0) {
                markTick = k;
                std::vector<double> s = held;
                std::sort(s.begin(), s.end());
                beforeMedian = 0.5 * (s[1] + s[2]);
                gap = s[2] - s[1];
                afterMedian = w.group(0).position().value();
            }
            ++k;
        });
    REQUIRE(markTick > 100);
    CHECK(gap > 0.0);                                       // the middle two really differ …
    CHECK(std::abs(afterMedian - beforeMedian) > 0.0);      // … so the median really moves …
    CHECK(std::abs(afterMedian - beforeMedian) <= gap + 1e-12);  // … and by no more than that gap
    CHECK(w.robot->faults().raiseCount(FaultCode::OdoStuck) == 0);
}

// ═══ P2-11 a disable during a cut; P2-15 a controller drop during a cut ═════════════════════
TEST_CASE("T4 P2-11: a disable DURING a cut returns at once through cancel(); the next entry starts "
          "with no cut and a count of 0 — and the absence marks are kept") {
    // BUG CAUGHT: a cut in force delaying the disable; a cut, or its count, surviving into the next
    // entry (the object is per call); the marks lost at a disable (they live in the groups).
    World w;
    w.h.plant().setMemberFault(0, 2, MemberFault::Reversed);
    REQUIRE(w.group(1).markAbsent(0));  // a member already marked (port 11)
    const auto log = w.run(40, [&](int) { w.stick(1.0); });  // cut at 25; disabled at 40, mid-cut
    REQUIRE(log.size() == 40U);
    CHECK(log.back().st.mode == ProtectionMode::Cut);
    for (int s = 0; s < 2; ++s) {
        for (int i = 0; i < 4; ++i) {
            CHECK(w.member(s, i).commandedVoltage().value() == 0.0);
            CHECK(w.member(s, i).brakeMode() == shulib::hal::BrakeMode::Brake);  // cancel()'s stop
        }
    }
    // The protections never READ a runtime-ABSENT member (the 2026-09-28 re-check's item 10; BUG
    // CAUGHT by the review of 2026-09-30, S5 / S6): at the cut its line says ABSENT with no reading,
    // and the disable summary calls it DEAD, with the peaks it had up to its mark — none this entry.
    const auto first = telLines(w.sink);
    CHECK(countContaining(first, "  at the cut: RIGHT port 11 ABSENT (not read)") == 1);
    CHECK(countContaining(first, "  at the cut: RIGHT port 11 v=") == 0);
    CHECK(countContaining(first, " | port 11 DEAD (to 0.00 A 0 C)") == 1);
    w.h.plant().setMemberFault(0, 2, MemberFault::None);
    const auto again = w.run(10, [&](int) { w.stick(1.0); });
    CHECK(again[0].st.cutCount == 0);
    CHECK(again[0].st.mode == ProtectionMode::Degraded);  // the kept mark
    CHECK(again[0].mv[0] > 1.0);                           // driving from tick 0
    CHECK(w.group(1).runtimeAbsentMask() == 1u);
}

TEST_CASE("T4 P2-15: the controller disconnected during a cut — still exactly 0 V, the cut ends once, "
          "nothing counted twice") {
    World w;
    w.h.plant().setMemberFault(0, 2, MemberFault::Reversed);
    const auto log = w.run(140, [&](int k) {  // the fight would cut again at 151
        w.stick(1.0);
        w.pad.setConnected(!(k >= 40 && k < 90));
    });
    for (int k = 25; k <= 125; ++k) CHECK(allDriveZero(log[static_cast<std::size_t>(k)]));
    CHECK(log[125].st.cutCount == 1);
    CHECK(countContaining(telLines(w.sink), "re-armed after cut #1") == 1);
    CHECK(countContaining(telLines(w.sink), "cut #2") == 0);
}

// ═══ P2-12 the log bound ════════════════════════════════════════════════════════════════════
TEST_CASE("T4 P2-12: 60 cuts give 50 logged blocks and the notice exactly once; the count reads 60") {
    // BUG CAUGHT: the bound off by one; a notice every cut after the bound; a count that stops at
    // the bound.
    World w;
    w.h.plant().setMemberFault(0, 2, MemberFault::Reversed);
    // A cut every 126 ticks from tick 25 (P2-2's shape): the 60th at 25 + 59 x 126 = 7459, the 61st
    // would be at 7585.
    const auto log = w.run(7500, [&](int) { w.stick(1.0); });
    CHECK(log.back().st.cutCount == 60);
    const auto lines = telLines(w.sink);
    CHECK(countContaining(lines, "-- both sides 0 V for 1 s, cut #") == 50);
    CHECK(countContaining(lines, "cut #50, then re-arm") == 1);
    CHECK(countContaining(lines, "cut #51, then re-arm") == 0);
    CHECK(countContaining(lines, "  at the cut: ") == 50 * 8);
    CHECK(countContaining(lines, "(further cuts are counted but no longer logged per port)") == 1);
}

// ═══ P2-13 the X-drive (no groups) ═══════════════════════════════════════════════════════════
namespace {

/// The invented X-drive — four BARE wheels, no motor groups — on the host test plant, driven
/// through runDriverControl. `script(k)` runs at the top of tick k; one state is kept per tick.
struct XDriveRig {
    XDriveRig()
        : xkin{shulib::kinematics::xDrive(Length{7.0})}, h{xkin, plantConfig()}, pacer{h, true},
          devices{h.clock(), h.battery(), sink, pacer} {
        cfgns::RobotConfig c{
            .identity = {.name = "Robot One", .team = "1234A", .season = "Push Back"},
            .drivetrain = {.kind = cfgns::DrivetrainKind::XDrive,
                           .xdrive = {.ports = {1, 2, 3, 4},
                                      .cartridge = MotorCartridge::Green,
                                      .imuPort = 10,
                                      .driveRadius = Length{7.0}}},
            .odometry = {.kind = cfgns::OdometryKind::TrackingWheels,
                         .wheels = {.forward = {.port = 6, .wheelDiameter = Length{2.0}, .offset = Length{-3.0}},
                                    .lateral = {.port = 7, .wheelDiameter = Length{2.0}, .offset = Length{-4.5}}}}};
        c.tuning.motion = motionConfig();
        for (int i = 0; i < 4; ++i) devices.bindMotor(static_cast<std::uint8_t>(i + 1), h.motor(i));
        devices.bindImu(10, h.imu());
        devices.bindRotation(6, h.forwardEncoder());
        devices.bindRotation(7, h.lateralEncoder());
        robot.emplace(c, devices);
    }
    template <typename Script>
    std::vector<DriveProtectionState> run(int ticks, Script&& script) {
        std::vector<DriveProtectionState> states;
        int k = 0;
        runDriverControl(
            robot->chassis(), pad, pacer, shaper, kCfg,
            [&] {
                if (k >= ticks) return true;
                script(k);
                return false;
            },
            [] { return HeadingSample{.radians = 0.0, .trusted = true}; },
            [&](const ChassisSpeeds&, bool, const DriveShaperState&, const DriveProtectionState& st) {
                states.push_back(st);
                ++k;
            });
        return states;
    }
    shulib::kinematics::MatrixKinematics xkin;
    SimHarness h;
    FakeTelemetrySink sink;
    StepPacer pacer;
    shulib::hal::fake::FakeDeviceProvider devices;
    std::optional<shulib::robot::Robot> robot;
    shulib::hal::fake::FakeController pad;
    DriveShaper shaper{DriverProfile{}};
};

/// The ticks on which a cut STARTED, from a per-tick state list.
std::vector<int> cutTicksOf(const std::vector<DriveProtectionState>& states) {
    std::vector<int> out;
    int last = 0;
    for (std::size_t t = 0; t < states.size(); ++t) {
        if (states[t].cutCount > last) out.push_back(static_cast<int>(t));
        last = states[t].cutCount;
    }
    return out;
}

}  // namespace

TEST_CASE("T4 P2-13: the invented X-drive (four bare wheels, no groups) — over-current cuts per wheel, "
          "named by wheel index, counted only while that wheel is commanded above 1 V; no absence or "
          "fight path; nothing crashes") {
    XDriveRig x;
    REQUIRE(x.robot->motorGroupCount() == 0U);
    const auto states = x.run(200, [&](int k) {
        x.pad.setAxis(ControllerAxis::LeftY, k < 100 ? 1.0 : 0.0);
        x.h.motor(2).setCurrent(shulib::units::Current{2.5});  // wheel 2, all along
    });
    REQUIRE(states.size() == 200U);
    CHECK(states[23].cutCount == 0);
    CHECK(states[24].cutCount == 1);
    CHECK(states.back().cutCount == 1);  // released from 100: never counted again at 0 V
    CHECK(states.back().groupCount == 0U);
    const auto lines = telLines(x.sink);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0]
          == "CUT: wheel 2 over 2.4 A for 250 ms -- every drive motor 0 V for 1 s, cut #1, then "
             "re-arm");
    CHECK(countContaining(lines, "  at the cut: wheel ") == 4);
    CHECK(countContaining(lines, "ABSENT") == 0);
    CHECK(countContaining(lines, "drive at the disable") == 0);  // no groups, no summary
}

TEST_CASE("T4 P2-13b: an X-drive wheel still over current after the re-arm, the stick HELD — a NEW "
          "250 ms streak starts at the re-arm: cut #2 at the re-arm + 25, never on the first tick "
          "after it") {
    // BUG CAUGHT (the review of 2026-09-30, S4): the bare motors' streaks not cleared at the re-arm
    // — P2-13 lets go of the stick inside its cut, so its streak zeroes by itself; held, a wheel
    // still over the limit would re-cut on the first tick after every re-arm, 1 s in every 1.01 s.
    XDriveRig x;
    const auto states = x.run(300, [&](int) {
        x.pad.setAxis(ControllerAxis::LeftY, 1.0);  // held through the re-arm
        x.h.motor(2).setCurrent(shulib::units::Current{2.5});
    });
    const std::vector<int> cuts = cutTicksOf(states);
    REQUIRE(cuts.size() >= 2U);
    CHECK(cuts[0] == 24);
    CHECK(cuts[1] == 24 + 100 + 25);
    CHECK(countContaining(telLines(x.sink), "re-armed after cut #1 (1 s at 0 V)") == 1);
}

// ═══ P2-16 the disable summary; P2-16b the peak line ═════════════════════════════════════════
TEST_CASE("T4 P2-16: the disable summary — one line a side, each member's peak current and "
          "temperature this entry, and the cut count") {
    World w;
    const auto log = w.run(50, [&](int k) {
        w.stick(1.0);
        for (int i = 0; i < 4; ++i) {
            w.member(0, i).setCurrent(shulib::units::Current{k == 20 ? 2.0 + 0.1 * i : 1.0});
            w.member(0, i).setTemperature(k == 30 ? 40.0 + i : 30.0);
        }
    });
    const auto lines = telLines(w.sink);
    REQUIRE(lines.size() >= 2U);
    CHECK(lines[lines.size() - 2]
          == "drive at the disable: LEFT, cuts 0 | port 16 2.00 A 40 C | port 17 2.10 A 41 C | port "
             "18 2.20 A 42 C | port 20 2.30 A 43 C");
    CHECK(lines.back()
          == "drive at the disable: RIGHT, cuts 0 | port 11 0.00 A 0 C | port 12 0.00 A 0 C | port 13 "
             "0.00 A 0 C | port 15 0.00 A 0 C");
}

TEST_CASE("T4 P2-16b: the PEAK line — a 3 s push at 2.30 A, let go: ONE line naming the port, before "
          "any disable; a smaller push: none; a larger: one more; a push that cuts: the line names "
          "the cut") {
    // BUG CAUGHT (the brief's mutation "the peak line only on the disable path"): a push check whose
    // only record is a line the loop may never print (the team has no competition switch).
    World w;
    std::vector<std::string> atEnd;
    const auto log = w.run(1400, [&](int k) {
        const bool push = (k < 300) || (k >= 400 && k < 600) || (k >= 700 && k < 900)
                          || (k >= 1000 && k < 1100);
        w.stick(push ? 1.0 : 0.0);
        const double amps = k < 300 ? 2.30 : k < 600 ? 2.10 : k < 900 ? 2.35 : 2.50;
        w.member(0, 1).setCurrent(shulib::units::Current{push ? amps : 0.2});
        if (k == 1399) atEnd = telLines(w.sink);  // everything said before the disable
    });
    REQUIRE(atEnd.size() == 4U + 8U + 1U);  // three peak lines, the cut + 8 at-the-cut, the re-arm
    CHECK(atEnd[0] == "drive: LEFT peak 2.30 A (port 17) over a 3.0 s drive, no cut");
    CHECK(atEnd[1] == "drive: LEFT peak 2.35 A (port 17) over a 2.0 s drive, no cut");
    CHECK(atEnd[2].rfind("CUT: port 17 (LEFT) over 2.4 A for 250 ms", 0) == 0);
    CHECK(atEnd[11] == "drive: LEFT peak 2.50 A (port 17) over a 0.2 s drive, cut #1");
    CHECK(atEnd[12] == "re-armed after cut #1 (1 s at 0 V)");
}

TEST_CASE("T4 P2-16c: the peak line for a push made BACKING UP — a 3 s reverse push at 2.30 A, let go: "
          "ONE line, the same as forward") {
    // BUG CAUGHT (the review of 2026-09-30, S2): a driven stretch judged on the SIGNED command — a
    // push made in reverse (a robot backing into a goal or an opponent) leaves no peak line at all,
    // and that line is the push check's record.
    World w;
    std::vector<std::string> atEnd;
    (void)w.run(400, [&](int k) {
        w.stick(k < 300 ? -1.0 : 0.0);
        w.member(0, 1).setCurrent(shulib::units::Current{k < 300 ? 2.30 : 0.2});
        if (k == 399) atEnd = telLines(w.sink);
    });
    // ONE line: RIGHT's members read the plant's 0 A throughout, and a stretch whose highest current
    // is 0 A names no member, so it prints nothing (P2-16b's forward pushes the same).
    REQUIRE(atEnd.size() == 1U);
    CHECK(atEnd[0] == "drive: LEFT peak 2.30 A (port 17) over a 3.0 s drive, no cut");
}

// ═══ P2-17 labels ════════════════════════════════════════════════════════════════════════════
TEST_CASE("T4 P2-17: a builder-made group names its members by PORT, in the description's order") {
    World w;
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(w.group(0).label(i) == kLeftPort[i]);
        CHECK(w.group(1).label(i) == kRightPort[i]);
    }
}

TEST_CASE("T4 P2-17b: Robot::describe() names a drive port marked ABSENT mid-run, from then on — the "
          "banner a program prints says what the protections decided") {
    // BUG CAUGHT (the review of 2026-09-30, R1): describe() naming the BOOT holes only — robot.hpp's
    // own doc says a runtime mark is named too, and a banner that still read "all present" after
    // the protections dropped port 18 would send a pit crew looking in the wrong place.
    World w;
    char before[400];
    w.robot->describe(before, sizeof before);
    CHECK(std::string{before}.find("DEGRADED: absent") == std::string::npos);
    REQUIRE(w.group(0).markAbsent(2));  // port 18
    REQUIRE(w.group(1).markAbsent(2));  // port 13
    char after[400];
    w.robot->describe(after, sizeof after);
    CHECK(std::string{after}.find(" | DEGRADED: absent 18,13 | ") != std::string::npos);
}

namespace {

/// A pacer that only advances the clock it is given.
struct ClockPacer final : shulib::motion::ITickPacer {
    explicit ClockPacer(shulib::hal::fake::FakeClock& c) : clock{&c} {}
    void pace() override { clock->advance(Time{0.01}); }
    shulib::hal::fake::FakeClock* clock;
};

/// A HAND-MADE tank — two MotorGroups of two FakeMotors each, built WITHOUT labels — over the
/// library's own pieces, the way chassis.hpp's standalone recipe wires a Chassis.
struct UnlabelledTank {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeImu imu;
    shulib::hal::AbsentGps gps;
    shulib::hal::AbsentTagSource tags;
    shulib::hal::AbsentVision vision;
    FakeTelemetrySink sink;
    std::array<FakeMotor, 4> m{};
    std::array<IMotor*, 2> leftPtrs{&m[0], &m[1]};
    std::array<IMotor*, 2> rightPtrs{&m[2], &m[3]};
    MotorGroup left{std::span<IMotor* const>{leftPtrs}};
    MotorGroup right{std::span<IMotor* const>{rightPtrs}};
    std::array<IMotor*, 2> drive{&left, &right};
    std::array<MotorGroup*, 2> groups{&left, &right};
    shulib::chassis::RobotContext ctx{{.clock = &clock,
                                       .driveMotors = drive,
                                       .imu = &imu,
                                       .gps = &gps,
                                       .battery = &battery,
                                       .telemetry = &sink,
                                       .tags = &tags,
                                       .vision = &vision}};
    shulib::localization::DriveEncoderOdometry odom{imu, left, right, shulib::hal::DriveGeometry{},
                                                    shulib::hal::DriveGeometry{},
                                                    Length{kTrackWidth}};
    shulib::localization::ComplementaryFusion fusion{};
    shulib::localization::Localizer loc{clock, imu, odom, fusion};
    shulib::diag::FaultLatch latch{sink, clock};
    shulib::diag::HealthMonitor health{latch};
    shulib::motion::MotionDeps deps{.ctx = &ctx,
                                    .localizer = &loc,
                                    .kinematics = &kTank,
                                    .faults = &latch,
                                    .health = &health,
                                    .motorGroups = groups};
    ClockPacer pacer{clock};
    shulib::chassis::Chassis chassis{deps, pacer, motion_rig::chassisConfig()};
    UnlabelledTank() { battery.setVoltage(shulib::units::Voltage{12.8}); }
};

}  // namespace

TEST_CASE("T4 P2-17: a hand-made group with NO labels names a slot \"member i\" — in the cut line "
          "and in every `at the cut:` line") {
    // BUG CAUGHT: a label read past an empty span (a crash, or a garbage port number in a pit crew's
    // log), or a line that names nothing at all.
    UnlabelledTank t;
    REQUIRE(t.left.label(0) == 0);
    shulib::hal::fake::FakeController pad;
    DriveShaper shaper{DriverProfile{}};
    std::array<double, 4> pos{};
    int k = 0;
    runDriverControl(
        t.chassis, pad, t.pacer, shaper, kCfg,
        [&] {
            if (k >= 40) return true;
            pad.setAxis(ControllerAxis::LeftY, 1.0);
            for (std::size_t i = 0; i < 4; ++i) {
                const double v = k == 0 ? 0.0 : (i == 1 ? -20.0 : 20.0);  // member 1 of LEFT fights
                pos[i] += v * kDt;
                t.m[i].setPosition(shulib::units::AngleDim{pos[i]});
                t.m[i].setVelocity(shulib::units::AngularVelocity{v});
            }
            return false;
        },
        [] { return HeadingSample{.radians = 0.0, .trusted = true}; },
        [&](const ChassisSpeeds&, bool, const DriveShaperState&) { ++k; });
    const auto lines = telLines(t.sink);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0] == "CUT: FIGHT on LEFT -- member(s) 1 -- both sides 0 V for 1 s, cut #1, then "
                      "re-arm");
    CHECK(countContaining(lines, "  at the cut: LEFT member 0 v=") == 1);
    CHECK(countContaining(lines, "  at the cut: LEFT member 1 v=") == 1);
    CHECK(countContaining(lines, "  at the cut: RIGHT member 1 v=") == 1);
}

// ═══ P2-19 THE WALL; P2-19c the coast-down; P2-21/21b a slipping member (hand-stepped) ═══════
TEST_CASE("T4 P2-19: a live train that stops at a wall under full stick, its reports lagging "
          "ALIKE, marks NOBODY — no FROZEN, no ABSENT, no DEGRADED line, no cut") {
    // BUG CAUGHT (the brief's mutation "the repeat on position alone", M29): a live robot at a wall
    // losing live members — every member alike is never a death. Under decision D (a) the
    // velocity-identical repeat is what keeps this case clean: the stalled members' decaying reports
    // keep changing together, so none of them repeats while a mate still reports motion, and the
    // brief's "FROZEN without the other-member guard" (M5) leaves this case GREEN. The guard's own
    // evidence is P2-21 / P2-21b here and P2-10b's whole-side case (T4-PROGRESS, delta 7).
    World w{World::Mode::Hand};
    HandSide left;
    HandSide right;
    const auto log = w.run(700, [&](int k) {
        w.stick(k < 500 ? 1.0 : 0.0);
        left.wall = right.wall = (k >= 200 && k < 500);
        handTick(w, left, right);
    });
    CHECK(cutTicks(log).empty());
    CHECK(longestLiveFrozenRun(log) == 0);
    CHECK(log.back().runtimeAbsent[0] == 0u);
    CHECK(log.back().runtimeAbsent[1] == 0u);
    CHECK(countContaining(telLines(w.sink), "DEGRADED") == 0);
    CHECK(log[150].mv[0] > 1.0);  // it drove before the wall
}

TEST_CASE("T4 P2-19c: the coast-down — full stick, let go, the train coasting (150 ms) with each "
          "member's encoder a quarter count apart and each report lagging 150 ms: every FROZEN run at "
          "most 3 ticks (none here), nobody ABSENT, no cut") {
    // BUG CAUGHT (the brief's mutation "the repeat on position alone"): the coast-down's FROZEN
    // spells (up to 12 ticks under item 4 as first written).
    World w{World::Mode::Hand};
    HandSide left;
    HandSide right;
    left.tauCoast = right.tauCoast = 0.15;
    left.tauV = right.tauV = 0.15;
    const auto log = w.run(700, [&](int k) {
        w.stick(k < 200 ? 1.0 : 0.0);
        handTick(w, left, right);
    });
    CHECK(cutTicks(log).empty());
    const int longest = longestLiveFrozenRun(log);
    CHECK(longest <= 3);
    CHECK(longest == 0);  // pinned: the probe's G4 at this point read 0
    CHECK(log.back().runtimeAbsent[0] == 0u);
}

TEST_CASE("T4 P2-21: a SLIPPING member at a wall (drag 0 and 0.8) — the three stalled LIVE members are "
          "never marked: no ABSENT, no DEGRADED, no REFUSE; the monitor's fight cuts during the push "
          "are allowed and pinned") {
    // BUG CAUGHT (the 2026-09-28 re-check's finding 14; the brief's mutation "FROZEN guard 'some other
    // member not SUSPECT'"): one slipping member (robot two's port 18) and a stalled train marking the
    // THREE live members — three dead, the drive REFUSED for the rest of the match.
    for (const double drag : {0.0, 0.8}) {
        World w{World::Mode::Hand};
        HandSide left;
        HandSide right;
        left.slip = 2;  // port 18
        left.slipDrag = drag;
        const auto log = w.run(700, [&](int k) {
            w.stick(k < 500 ? 1.0 : 0.0);
            left.wall = right.wall = (k >= 200 && k < 500);
            handTick(w, left, right);
        });
        CAPTURE(drag);
        CHECK(log.back().runtimeAbsent[0] == 0u);
        CHECK(log.back().runtimeAbsent[1] == 0u);
        CHECK(countContaining(telLines(w.sink), "DEGRADED") == 0);
        CHECK(countContaining(telLines(w.sink), "REFUSED") == 0);
        CHECK(cutTicks(log).size() == 3U);  // the slip_probe's count for G4
        CHECK(longestLiveFrozenRun(log) == 0);
    }
}

TEST_CASE("T4 P2-21b: the slipping member COASTING in a cut (a 150 ms report lag) — its position stops "
          "while its lagging report is still above 0.5 rad/s; it is never FROZEN and never ABSENT") {
    // BUG CAUGHT (the brief's mutation "the repeat on position alone"): the live slipping member
    // itself latched during a cut (the re-check's S1c).
    World w{World::Mode::Hand};
    HandSide left;
    HandSide right;
    left.slip = 2;
    left.tauV = right.tauV = 0.15;
    const auto log = w.run(700, [&](int k) {
        w.stick(k < 500 ? 1.0 : 0.0);
        left.wall = right.wall = (k >= 200 && k < 500);
        handTick(w, left, right);
    });
    CHECK(log.back().runtimeAbsent[0] == 0u);
    CHECK(longestLiveFrozenRun(log) == 0);
    CHECK(countContaining(telLines(w.sink), "REFUSED") == 0);
}

TEST_CASE("T4 P2-21c: THE KNOWN GAP (decision D's accepted cost, the re-check's S10) — a LEFT side "
          "already down to two, whose slipping member runs while its last live mate stalls at a wall: a "
          "tie, so the stalled LIVE member is marked, the third dead in total, and the drive REFUSES") {
    // PINNED ON PURPOSE, not a bug caught: the documented cost (A4 register HA-175; guide 12's "a motor
    // reported dead that is not") must stay true, and a change to the guard must be a decision, not an
    // accident — under a strict-majority guard (decision D (d)) this case marks nobody and this test
    // goes red, and its cost (a 2-of-4 side's next REAL death never marked) comes with it.
    World w{World::Mode::Hand};
    HandSide left;
    HandSide right;
    left.slip = 2;                       // port 18 slips (HA-130)
    REQUIRE(w.group(0).markAbsent(0));   // ports 16 and 17 already dead — an earlier start's marks
    REQUIRE(w.group(0).markAbsent(1));
    const auto log = w.run(700, [&](int k) {
        w.stick(k < 500 ? 1.0 : 0.0);
        left.wall = right.wall = (k >= 200 && k < 500);
        handTick(w, left, right);
    });
    const auto lines = telLines(w.sink);
    CHECK(countContaining(lines, "driver control starts DEGRADED (left dead 2, right dead 0, total 2)")
          == 1);
    CHECK(log[150].mv[0] > 1.0);  // it drove, degraded, before the wall
    const int marked = absentTick(log, 0, 3);  // port 20: live, stalled at the wall
    CAPTURE(marked);
    CHECK(marked > 200);
    CHECK(marked < 500);
    CHECK(marked == 360);  // pinned: 1.6 s into the push, observed 2026-09-29
    CHECK((log.back().runtimeAbsent[0] & 0b0100u) == 0u);  // the slipping member is never marked
    CHECK(log.back().runtimeAbsent[1] == 0u);               // nor anyone on the RIGHT, at the same wall
    CHECK(countContaining(lines, "WARNING: port 20 (LEFT) went ABSENT at runtime") == 1);
    CHECK(countContaining(lines, "REFUSED: more than 2 motors dead in total") == 1);
    REQUIRE(marked > 0);
    for (std::size_t k = static_cast<std::size_t>(marked); k < log.size(); ++k) {
        CAPTURE(k);
        CHECK(allDriveZero(log[k]));
        CHECK(log[k].st.mode == ProtectionMode::Refused);
    }
}

namespace {

/// One stalled member that does not read like its stalled mates (the review of 2026-09-30; the
/// verification of 2026-10-01 added the lags and the stick): its report flickering by 1 RPM, its
/// encoder toggling a count every 3 ticks, or its report settling sooner than its mates' — its own
/// lag `fastTau` beside the LEFT side's `matesTau`. -1 = none of that kind. Every slot is LEFT's;
/// the RIGHT side reads alike, every report at HandSide's 80 ms.
struct OddMember {
    int flicker = -1;
    int dither = -1;
    int fastLag = -1;
    double fastTau = 0.03;   ///< the `fastLag` member's own report lag, s
    double matesTau = 0.08;  ///< every other LEFT member's report lag (HandSide::tauV), s
    double stick = 1.0;      ///< the stick held through the push
};

/// P2-19's wall (a push at `odd.stick`, the train held from tick 200 to 499, let go at 500) with
/// `odd` on the LEFT side and, when `slip` >= 0, that LEFT member slipping (P2-21's port 18).
std::vector<TickLog> oddWall(World& w, const OddMember& odd, int slip) {
    HandSide left;
    HandSide right;
    left.slip = slip;
    left.flicker = odd.flicker;
    left.dither = odd.dither;
    left.tauV = odd.matesTau;
    if (odd.fastLag >= 0) left.tauVOf[static_cast<std::size_t>(odd.fastLag)] = odd.fastTau;
    return w.run(700, [&](int k) {
        w.stick(k < 500 ? odd.stick : 0.0);
        left.wall = right.wall = (k >= 200 && k < 500);
        handTick(w, left, right);
    });
}

}  // namespace

TEST_CASE("T4 P2-19d: a PLAIN wall (no slipping member) with ONE stalled member that does not read like "
          "its mates — a report flickering by 1 RPM, an encoder toggling a count, a report settling "
          "sooner by too little — marks NOBODY and cuts nothing; each longest live FROZEN spell is "
          "pinned (up to 24 ticks, one short of the mark)") {
    // BUG CAUGHT: a rule that one odd reading on a stalled LIVE train pushes into a mark. At a plain
    // wall every ENCODER stops with the train, but its mates' REPORTS do not stop at once: a lagging
    // report keeps reading above 1 rad/s for a while, and the mates' fastest counts any mate whose
    // position OR report still changes. So a member whose report settles first repeats, is SUSPECT
    // alone, the guard holds (one against three), and it is FROZEN until its mates' reports fall
    // under 1 rad/s: 9 ticks at 30 ms beside 80 ms (the review of 2026-09-30's "NO slip" row), 17 at
    // 30 ms beside 100 ms, 24 at 50 ms beside 150 ms — one short of the mark — and 20 at HALF stick
    // at 30 ms beside 120 ms, a spread that marks at full stick (the train stops from a lower speed,
    // so the mates' reports fall under the floor sooner). A wider spread reaches 25 and marks a live
    // motor: P2-19e, that gap pinned. Each spell is pinned so a rule change that lengthens it is seen
    // before it reaches 25; the 24-tick row is the latch's own boundary at a plain wall.
    struct Row {
        const char* name;
        OddMember odd;
        int longest;
    };
    const Row rows[] = {
        {"port 20's report flickers 0/1 RPM", {.flicker = 3}, 0},
        {"port 16's encoder toggles a count every 3 ticks", {.dither = 0}, 0},
        {"port 16's report lags 30 ms, its mates' 80 ms", {.fastLag = 0}, 9},
        {"port 16's report lags 30 ms, its mates' 100 ms", {.fastLag = 0, .matesTau = 0.10}, 17},
        {"port 16's report lags 50 ms, its mates' 150 ms (one short)",
         {.fastLag = 0, .fastTau = 0.05, .matesTau = 0.15}, 24},
        {"half stick: port 16's report lags 30 ms, its mates' 120 ms",
         {.fastLag = 0, .matesTau = 0.12, .stick = 0.5}, 20},
    };
    for (const Row& row : rows) {
        const std::string rowName{row.name};  // CAPTURE prints a const char* as an address
        CAPTURE(rowName);
        World w{World::Mode::Hand};
        const auto log = oddWall(w, row.odd, -1);
        CHECK(log.back().runtimeAbsent[0] == 0u);
        CHECK(log.back().runtimeAbsent[1] == 0u);
        CHECK(countContaining(telLines(w.sink), "went ABSENT") == 0);
        CHECK(cutTicks(log).empty());
        CHECK(longestLiveFrozenRun(log) == row.longest);
        CHECK(log[150].mv[0] > 1.0);  // it drove before the wall
    }
}

TEST_CASE("T4 P2-19e: THE REPORT-LAG GAP (found by the verification of 2026-10-01; the team lead's to "
          "rule on) — a PLAIN wall, NO slipping member: ONE stalled member whose report settles enough "
          "sooner than its mates' is marked ABSENT, a live motor, and its side drives on DEGRADED") {
    // PINNED ON PURPOSE, not a bug caught (as P2-21c and P2-21d). At a wall every encoder stops, but
    // the mates' lagging REPORTS still read above 1 rad/s, and they are the evidence that the train
    // turns (the mates' fastest counts a mate whose position OR report still changes). The member
    // whose report reaches 0 first repeats, is SUSPECT alone, and the guard holds (one against
    // three); when its mates' reports stay above 1 rad/s for 29 ticks after its own first reads 0 —
    // 5 reads to SUSPECT, then 25 FROZEN ticks — it is marked: a 30 ms report beside 120 ms ones (the
    // verification's row), 10 ms beside 100 ms, and 30 ms beside 150 ms at HALF stick (the same
    // 30-beside-120 marks nobody at half stick: P2-19d). No slipping gear is involved, so fixing
    // port 18's does not close this; beside it the bar is lower (P2-21d's third row: 30 beside 80).
    // A4 register HA-175 states the gap; a change to the guard or its evidence must be a ruling, not
    // an accident, and this case goes red with it, to be re-pinned to the ruled behaviour.
    struct Row {
        const char* name;
        OddMember odd;
        int at;
    };
    const Row rows[] = {
        {"port 16's report lags 30 ms, its mates' 120 ms", {.fastLag = 0, .matesTau = 0.12}, 244},
        {"port 16's report lags 10 ms, its mates' 100 ms",
         {.fastLag = 0, .fastTau = 0.01, .matesTau = 0.10}, 229},
        {"half stick: port 16's report lags 30 ms, its mates' 150 ms",
         {.fastLag = 0, .matesTau = 0.15, .stick = 0.5}, 242},
    };
    for (const Row& row : rows) {
        const std::string rowName{row.name};  // CAPTURE prints a const char* as an address
        CAPTURE(rowName);
        World w{World::Mode::Hand};
        const auto log = oddWall(w, row.odd, -1);
        CHECK(absentTick(log, 0, 0) == row.at);
        CHECK(log.back().runtimeAbsent[0] == 0b0001u);  // port 16 only
        CHECK(log.back().runtimeAbsent[1] == 0u);       // the RIGHT, at the same wall, reads alike
        const auto lines = telLines(w.sink);
        CHECK(countContaining(lines, "WARNING: port 16 (LEFT) went ABSENT at runtime") == 1);
        CHECK(countContaining(lines, "went ABSENT") == 1);
        CHECK(countContaining(lines, "REFUSED") == 0);  // one dead: it drives on, DEGRADED
        CHECK(cutTicks(log).empty());
        CHECK(log.back().st.mode == ProtectionMode::Degraded);
        CHECK(log[150].mv[0] > 1.0);  // it drove before the wall
        CHECK(log[400].mv[0] > 1.0);  // marked, and still commanded: a mark drops reads, not drive
    }
}

TEST_CASE("T4 P2-21d: THE WIDER GAP (found by the review of 2026-09-30; the team lead's to rule on) — "
          "port 18 slipping at a wall beside ONE stalled member that does not read like its mates: LIVE "
          "members are marked, and a second push REFUSES a side of four live motors") {
    // PINNED ON PURPOSE, not a bug caught (as P2-21c): decision D (a)'s guard is a COUNT of suspect
    // members against answering ones, and at a wall beside a slipping member it holds only while
    // every stalled member reads alike (P2-21). One stalled member whose readings keep changing
    // counts as ANSWERING — a 2-2 tie, and two live members are FROZEN and marked; one whose report
    // settles sooner than its mates' is SUSPECT ALONE — marked, at a 30-beside-80 ms spread here,
    // where the slipping member's encoder keeps the evidence alive (with no slipping member the same
    // lone suspect takes a wider spread: P2-19d, P2-19e). A4 register HA-175, guide 12 and
    // coupled_side_monitor.hpp state this wider gap; a change to the guard must be a ruling, not an
    // accident, and this case goes red with it, to be re-pinned to the ruled behaviour.
    struct Row {
        const char* name;
        OddMember odd;
        std::uint32_t marked;
        int at;
    };
    const Row rows[] = {
        {"port 20's report flickers: 16 and 17 marked (a tie)", {.flicker = 3}, 0b0011u, 360},
        {"port 16's encoder toggles a count: 17 and 20 marked (a tie)", {.dither = 0}, 0b1010u, 360},
        {"port 16's report settles first: 16 marked (suspect alone)", {.fastLag = 0}, 0b0001u, 244},
    };
    for (const Row& row : rows) {
        const std::string rowName{row.name};  // CAPTURE prints a const char* as an address
        CAPTURE(rowName);
        World w{World::Mode::Hand};
        const auto log = oddWall(w, row.odd, 2);
        CHECK(log.back().runtimeAbsent[0] == row.marked);
        CHECK(log.back().runtimeAbsent[1] == 0u);  // the RIGHT, at the same wall, reads alike
        int marks = 0;
        for (int i = 0; i < 4; ++i) {
            if ((row.marked & (std::uint32_t{1} << i)) != 0) {
                CAPTURE(i);
                CHECK(absentTick(log, 0, i) == row.at);
                ++marks;
            }
        }
        const auto lines = telLines(w.sink);
        CHECK(countContaining(lines, "went ABSENT") == marks);
        CHECK(countContaining(lines, "REFUSED") == 0);  // two dead at most: it drives on, DEGRADED
        CHECK(log.back().st.mode == ProtectionMode::Degraded);
        CHECK(absentTick(log, 0, 2) == -1);  // the slipping member itself is never marked
    }
    // TWO pushes, port 20 flickering in the first only: the first marks 16 and 17 (above); the
    // second, every stalled member now reading alike, is port 20 against the slipping port 18 — a
    // 1-1 tie — and marks port 20: the third dead, REFUSED for the rest of the run.
    World w{World::Mode::Hand};
    HandSide left;
    HandSide right;
    left.slip = 2;
    left.flicker = 3;
    const auto log = w.run(1400, [&](int k) {
        w.stick((k < 500 || (k >= 700 && k < 1200)) ? 1.0 : 0.0);
        left.wall = right.wall = (k >= 200 && k < 500) || (k >= 900 && k < 1200);
        if (k == 900) left.flicker = -1;
        handTick(w, left, right);
    });
    CHECK(absentTick(log, 0, 0) == 360);
    CHECK(absentTick(log, 0, 1) == 360);
    CHECK(absentTick(log, 0, 3) == 1060);
    CHECK(absentTick(log, 0, 2) == -1);
    CHECK(log.back().runtimeAbsent[1] == 0u);
    REQUIRE(absentTick(log, 0, 3) > 0);
    for (std::size_t k = static_cast<std::size_t>(absentTick(log, 0, 3)); k < log.size(); ++k) {
        CAPTURE(k);
        CHECK(log[k].st.mode == ProtectionMode::Refused);
        CHECK(allDriveZero(log[k]));
    }
    CHECK(countContaining(telLines(w.sink), "REFUSED: more than 2 motors dead in total") == 1);
}

// ═══ P2-20 the config's entry check ══════════════════════════════════════════════════════════
namespace {

/// An IMotor that COUNTS every command reaching it (FakeMotor has no write count; the brief's §5
/// forbids changing it).
struct CountingMotor final : IMotor {
    FakeMotor inner;
    int voltageWrites = 0;
    int brakeSets = 0;
    void setVoltage(shulib::units::Voltage v) override {
        ++voltageWrites;
        inner.setVoltage(v);
    }
    [[nodiscard]] shulib::units::Voltage commandedVoltage() const override {
        return inner.commandedVoltage();
    }
    void setBrakeMode(shulib::hal::BrakeMode m) override {
        ++brakeSets;
        inner.setBrakeMode(m);
    }
    [[nodiscard]] shulib::hal::BrakeMode brakeMode() const override { return inner.brakeMode(); }
    [[nodiscard]] shulib::units::AngleDim position() const override { return inner.position(); }
    [[nodiscard]] shulib::units::AngularVelocity velocity() const override {
        return inner.velocity();
    }
    [[nodiscard]] shulib::units::Current current() const override { return inner.current(); }
    [[nodiscard]] double temperature() const override { return inner.temperature(); }
};

/// A 3-wire output that counts its writes.
struct CountingLine final : shulib::hal::IDigitalOut {
    bool level = false;
    int writes = 0;
    void set(bool value) override {
        ++writes;
        level = value;
    }
    [[nodiscard]] bool commanded() const override { return level; }
};

}  // namespace

TEST_CASE("T4 P2-20: a bad DriverLoopConfig::protection is a PreconditionError before tick 1, and "
          "NOTHING is commanded — no drive or pivot volts, no brake mode, no line write, and a leftover "
          "claim still held") {
    // BUG CAUGHT (the brief's mutations "the config's entry check dropped" and "the config check
    // after the claim release"): a nonsense protection config running a match, or its refusal coming
    // after the loop has already released a claim, made the pivots safe, entered the toggles and set
    // the release mode. The review of 2026-09-30 (S10-S13) found four of the check's clauses reached
    // by no row, each at its boundary: overCurrentTicks 0 (every member then "over" on every tick:
    // the drive cut after every re-arm), maxCutsLogged 0, frozenSuspectReads 0 (the detector's own
    // precondition would throw AFTER the entry's commands — the failure this case exists for), and an
    // overCurrentAmps of 0, below 0 or infinite. Each row names the field its refusal must name.
    struct Bad {
        const char* field;
        void (*apply)(DriveProtectionConfig&);
    };
    const Bad table[] = {
        {"overCurrentAmps",
         [](DriveProtectionConfig& p) { p.overCurrentAmps = std::numeric_limits<double>::quiet_NaN(); }},
        {"overCurrentAmps", [](DriveProtectionConfig& p) { p.overCurrentAmps = 0.0; }},
        {"overCurrentAmps", [](DriveProtectionConfig& p) { p.overCurrentAmps = -1.0; }},
        {"overCurrentAmps",
         [](DriveProtectionConfig& p) { p.overCurrentAmps = std::numeric_limits<double>::infinity(); }},
        {"overCurrentTicks", [](DriveProtectionConfig& p) { p.overCurrentTicks = 0; }},
        {"overCurrentTicks", [](DriveProtectionConfig& p) { p.overCurrentTicks = -1; }},
        {"cutTicks", [](DriveProtectionConfig& p) { p.cutTicks = 0; }},
        {"maxCutsLogged", [](DriveProtectionConfig& p) { p.maxCutsLogged = 0; }},
        {"frozenSuspectReads", [](DriveProtectionConfig& p) { p.frozenSuspectReads = 0; }},
        {"frozenSuspectReads", [](DriveProtectionConfig& p) { p.frozenSuspectReads = 25; }},
    };
    const auto bad = [&table](std::size_t which) {
        DriverLoopConfig cfg = kCfg;
        table[which].apply(cfg.protection);
        return cfg;
    };
    for (std::size_t which = 0; which < std::size(table); ++which) {
        CAPTURE(which);
        const std::string field{table[which].field};  // printed, not its address
        CAPTURE(field);
        cfgns::RobotConfig c = robotTwoShape();
        c.mechanisms[0] = {.name = "claw pivot 6",
                           .motorPorts = {6},
                           .motorCount = 1,
                           .cartridge = MotorCartridge::Red,
                           .safeMode = cfgns::SafeBrakeMode::Hold};
        c.mechanisms[1] = {.name = "claw",
                           .pneumaticPorts = {1},
                           .pneumaticCount = 1,
                           .pneumaticSafe = cfgns::PneumaticLevel::Low,
                           .pneumaticStart = cfgns::PneumaticLevel::Low};
        c.mechanismCount = 2;
        SimHarness h{kTank, worldConfig()};
        FakeTelemetrySink sink;
        StepPacer pacer{h, false};
        shulib::hal::fake::FakeDeviceProvider devices{h.clock(), h.battery(), sink, pacer};
        std::array<CountingMotor, 9> motors{};
        const int ports[9] = {16, 17, 18, 20, 11, 12, 13, 15, 6};
        for (std::size_t i = 0; i < 9; ++i) {
            devices.bindMotor(static_cast<std::uint8_t>(ports[i]), motors[i]);
        }
        CountingLine line;
        devices.bindDigitalOut(1, line);
        shulib::robot::Robot robot{c, devices};
        REQUIRE(robot.mechanismAt(0) != nullptr);
        REQUIRE(robot.pneumaticMechanismAt(1) != nullptr);
        shulib::teleop::MechanismBinding pivot{*robot.mechanismAt(0), ControllerButton::R1,
                                               ControllerButton::R2, shulib::units::Voltage{12.0}};
        shulib::teleop::ToggleBinding claw{*robot.pneumaticMechanismAt(1), ControllerButton::A,
                                           false};
        shulib::teleop::MechanismBinding* holds[1] = {&pivot};
        shulib::teleop::ToggleBinding* toggles[1] = {&claw};
        REQUIRE(robot.mechanismAt(0)->tryClaim());  // a leftover claim
        for (CountingMotor& m : motors) {
            m.voltageWrites = 0;
            m.brakeSets = 0;
        }
        line.writes = 0;
        shulib::hal::fake::FakeController pad;
        DriveShaper shaper{DriverProfile{}};
        int ticks = 0;
        bool threw = false;
        std::string message;
        try {
            runDriverControl(robot.chassis(), pad, pacer, shaper, bad(which),
                             std::span<shulib::teleop::MechanismBinding* const>{holds},
                             std::span<shulib::teleop::ToggleBinding* const>{toggles},
                             [&] { return ticks++ > 5; }, [] { return HeadingSample{}; },
                             [](const ChassisSpeeds&, bool, const DriveShaperState&) {});
        } catch (const PreconditionError& e) {
            threw = true;
            message = e.what();
        }
        CHECK(threw);
        CHECK(message.find(std::string{"DriveProtectionConfig: "} + table[which].field)
              != std::string::npos);
        CHECK(ticks == 0);
        for (const CountingMotor& m : motors) {
            CHECK(m.voltageWrites == 0);
            CHECK(m.brakeSets == 0);
        }
        CHECK(line.writes == 0);
        CHECK(robot.mechanismAt(0)->claimed());
    }
}

TEST_CASE("T4 P2-20b: validProtectionConfig — more groups than the protections cover, and a null group, "
          "are refused with their reasons; kMaxProtectedGroups groups pass; a null or zero-sized "
          "`why` is safe") {
    // BUG CAUGHT (the review of 2026-09-30, S14 / S15): the capacity clause dropped — a fifth group
    // would index the protections' fixed per-group state past its end — and the null-group clause
    // dropped (a null read in the loop's own check). A robot::Robot builds at most two groups and
    // MotionDeps refuses a null one, so neither is reachable through the World: the check is called
    // directly, as runDriverControl calls it for a chassis a program wired by hand.
    std::array<FakeMotor, 5> m{};
    std::array<IMotor*, 1> p0{&m[0]};
    std::array<IMotor*, 1> p1{&m[1]};
    std::array<IMotor*, 1> p2{&m[2]};
    std::array<IMotor*, 1> p3{&m[3]};
    std::array<IMotor*, 1> p4{&m[4]};
    MotorGroup g0{std::span<IMotor* const>{p0}};
    MotorGroup g1{std::span<IMotor* const>{p1}};
    MotorGroup g2{std::span<IMotor* const>{p2}};
    MotorGroup g3{std::span<IMotor* const>{p3}};
    MotorGroup g4{std::span<IMotor* const>{p4}};
    std::array<MotorGroup*, 5> five{&g0, &g1, &g2, &g3, &g4};
    const std::span<MotorGroup* const> all{five};
    const DriveProtectionConfig cfg{};
    char why[200] = "";
    CHECK_FALSE(shulib::teleop::validProtectionConfig(cfg, all, why, sizeof why));
    CHECK(std::string{why}.find("kMaxProtectedGroups") != std::string::npos);
    CHECK(shulib::teleop::validProtectionConfig(cfg, all.first(shulib::teleop::kMaxProtectedGroups),
                                                why, sizeof why));
    std::array<MotorGroup*, 2> withNull{&g0, nullptr};
    const std::span<MotorGroup* const> nulls{withNull};
    CHECK_FALSE(shulib::teleop::validProtectionConfig(cfg, nulls, why, sizeof why));
    CHECK(std::string{why} == "DriveProtection: a motorGroups entry is null");
    CHECK_FALSE(shulib::teleop::validProtectionConfig(cfg, nulls, nullptr, 0));
    char tiny[4] = "xyz";
    CHECK_FALSE(shulib::teleop::validProtectionConfig(cfg, nulls, tiny, 0));
    CHECK(std::string{tiny} == "xyz");  // cap 0: nothing written
    CHECK_FALSE(shulib::teleop::validProtectionConfig(cfg, nulls, tiny, sizeof tiny));
    CHECK(std::string{tiny} == "Dri");  // cut to fit, NUL-terminated
}

TEST_CASE("T4 P2-20c: a tick under 1 ms, and one over 1000 s — both of which runDriverControl has always "
          "accepted — RUN: nothing is refused after the entry has commanded something, and every "
          "duration prints in true units") {
    // BUG CAUGHT (the review of 2026-09-30, two lenses): DriveProtection refusing a tick the loop's
    // own precondition accepts — it required a whole millisecond, and no more than 1000 s — THROWN
    // after the claim release, the entry safe state and the release mode had already commanded the
    // robot (P2-20's invariant broken), and a regression for a tick the loop ran before the
    // protections existed. No program uses such a tick (a V5's is 10 ms); the contract is the
    // loop's, and it holds for every finite tick above zero. The pacer here steps the plant 10 ms
    // whatever the stated tick: the protections count TICKS, so the cut lands on the same tick, and
    // only the durations their lines print follow the stated period.
    struct Row {
        double seconds;
        const char* cutLine;
        const char* rearmLine;
    };
    const Row rows[] = {
        {0.0005,
         "CUT: port 17 (LEFT) over 2.4 A for 12.5 ms -- both sides 0 V for 50 ms, cut #1, then re-arm",
         "re-armed after cut #1 (50 ms at 0 V)"},
        {2000.0,
         "CUT: port 17 (LEFT) over 2.4 A for 50000 s -- both sides 0 V for 200000 s, cut #1, then "
         "re-arm",
         "re-armed after cut #1 (200000 s at 0 V)"},
    };
    for (const Row& row : rows) {
        CAPTURE(row.seconds);
        World w;
        DriverLoopConfig cfg = kCfg;
        cfg.tickPeriod = Time{row.seconds};
        std::vector<TickLog> log;
        CHECK_NOTHROW(log = w.run(
                          1200,
                          [&](int k) {
                              w.stick(1.0);
                              const bool over = k >= 1000 && k < 1100;  // ends inside the cut
                              w.member(0, 1).setCurrent(shulib::units::Current{over ? 2.5 : 1.0});
                          },
                          cfg));
        REQUIRE(log.size() == 1200U);
        REQUIRE(log[999].mv[0] > 1.0);  // driven above the floor before the over-current starts
        const std::vector<int> cuts = cutTicks(log);
        REQUIRE(cuts.size() == 1U);
        CHECK(cuts[0] == 1024);  // 25 ticks: the window counts ticks, whatever their length
        const auto lines = telLines(w.sink);
        REQUIRE_FALSE(lines.empty());
        CHECK(lines[0] == row.cutLine);
        CHECK(countContaining(lines, row.rearmLine) == 1);
    }
}

TEST_CASE("T4 P2-20d: DriveProtection built DIRECTLY with a tick that is zero, negative, NaN or infinite "
          "is a PreconditionError naming the tick period, and a refused construction touches nothing; "
          "the smallest and the largest finite positive ticks are accepted") {
    // BUG CAUGHT (the verification of 2026-10-01, its mutation E1: DriveProtection's own tick
    // precondition dropped, which survived the whole suite): P2-20c pins only the ACCEPTING side, and
    // through runDriverControl, whose identical check runs first, a bad tick never reaches the
    // class. The class is public for a program with its own loop; without its check, such a program
    // builds it with a 0, negative or NaN tick and every duration it prints reads "0 ms", "-250 ms"
    // or "nan ms" instead of a refusal.
    struct Row {
        const char* name;
        double seconds;
    };
    const Row rows[] = {
        {"zero", 0.0},
        {"negative zero", -0.0},
        {"negative", -0.01},
        {"NaN", std::numeric_limits<double>::quiet_NaN()},
        {"+infinity", std::numeric_limits<double>::infinity()},
        {"-infinity", -std::numeric_limits<double>::infinity()},
    };
    for (const Row& row : rows) {
        const std::string rowName{row.name};  // CAPTURE prints a const char* as an address
        CAPTURE(rowName);
        World w;
        w.group(0).setMonitorExcluded(0b0110u);  // a mask the destructor would clear if one ran
        const int linesBefore = w.sink.size();
        bool threw = false;
        std::string message;
        try {
            const DriveProtection p{w.robot->chassis(), DriveProtectionConfig{}, Time{row.seconds}};
        } catch (const PreconditionError& e) {
            threw = true;
            message = e.what();
        }
        CHECK(threw);
        CHECK(message.find("DriveProtection: the tick period must be finite and > 0")
              != std::string::npos);
        CHECK(w.group(0).monitorExcludedMask() == 0b0110u);  // refused: no object, no destructor
        CHECK(w.sink.size() == linesBefore);                 // and nothing said
    }
    // Exactly "finite and > 0" — the loop's own check — at both ends of the doubles.
    const double accepted[] = {std::numeric_limits<double>::denorm_min(),
                               std::numeric_limits<double>::max()};
    for (const double s : accepted) {
        CAPTURE(s);
        World w;
        CHECK_NOTHROW(DriveProtection{w.robot->chassis(), DriveProtectionConfig{}, Time{s}});
    }
}

// ═══ P2-22 the exclusion mask never outlives driver control ═══════════════════════════════════
TEST_CASE("T4 P2-22: a member FROZEN when driver control ends is back in its monitor afterwards — on "
          "the disable return AND on a throw — so a turnTo in the next autonomous, with it still "
          "reading frozen, raises MOTOR_GROUP_DISAGREE") {
    // BUG CAUGHT (the brief's mutation "no mask clear on the disable return / in the destructor"): a
    // stale exclusion that hides a fight from every motion for the rest of the run.
    SUBCASE("the disable return") {
        World w;
        const auto log = w.run(115, [&](int k) {
            if (k == 100) w.h.plant().setMemberFault(0, 1, MemberFault::Frozen);
            w.stick(1.0);
        });
        REQUIRE(frozenTick(log, 0, 1) == 105);
        REQUIRE((log.back().excluded[0] & 0b0010u) != 0);  // FROZEN on the last tick
        CHECK(w.group(0).monitorExcludedMask() == 0u);
        CHECK(w.group(1).monitorExcludedMask() == 0u);
        CHECK(w.group(0).runtimeAbsentMask() == 0u);  // not marked: it never reached 25
        (void)w.robot->chassis().turnTo(shulib::math::Angle::degrees(90.0),
                                        {.timeout = Time{3.0}});
        CHECK(w.robot->faults().raiseCount(FaultCode::MotorGroupDisagree) >= 1);
    }
    SUBCASE("a throw out of the loop (the destructor)") {
        World w;
        int k = 0;
        CHECK_THROWS_AS(
            runDriverControl(
                w.robot->chassis(), w.pad, w.pacer, w.shaper, kCfg,
                [&] {
                    if (k == 100) w.h.plant().setMemberFault(0, 1, MemberFault::Frozen);
                    w.stick(1.0);
                    return false;
                },
                [] { return HeadingSample{.radians = 0.0, .trusted = true}; },
                [&](const ChassisSpeeds&, bool, const DriveShaperState&) {
                    if (k == 114) {
                        REQUIRE(w.group(0).monitorExcludedMask() == 0b0010u);
                        SHULIB_PRECONDITION(false, "P2-22: a throw out of driver control");
                    }
                    ++k;
                }),
            PreconditionError);
        CHECK(w.group(0).monitorExcludedMask() == 0u);
        (void)w.robot->chassis().turnTo(shulib::math::Angle::degrees(90.0),
                                        {.timeout = Time{3.0}});
        CHECK(w.robot->faults().raiseCount(FaultCode::MotorGroupDisagree) >= 1);
    }
}

TEST_CASE("T4 P2-22b: enter() clears an exclusion mask it finds — DriveProtection's own contract, for a "
          "program that runs it in a loop of its own") {
    // BUG CAUGHT (the mutation "no mask clear at enter()"): runDriverControl cannot show it — its
    // first beforeDrive() re-sets every mask before the first drive() — but DriveProtection is public
    // for a program with its own loop, which may drive between enter() and its first beforeDrive(),
    // and a mask left by a task PROS deleted mid-tick (no destructor ran) would hide a fight there.
    World w;
    w.group(0).setMonitorExcluded(0b0110u);
    w.group(1).setMonitorExcluded(0b0001u);
    REQUIRE(w.group(0).monitorExcludedMask() == 0b0110u);
    REQUIRE(w.group(1).monitorExcludedMask() == 0b0001u);
    DriveProtection p{w.robot->chassis(), DriveProtectionConfig{}, Time{0.01}};
    CHECK(w.group(0).monitorExcludedMask() == 0b0110u);  // construction touches nothing
    p.enter();
    CHECK(w.group(0).monitorExcludedMask() == 0u);
    CHECK(w.group(1).monitorExcludedMask() == 0u);
    CHECK(p.state().mode == ProtectionMode::Driving);
}

// ═══ P2-23 a port dead AT BOOT; P2-24 a chassis with four groups ═════════════════════════════
TEST_CASE("T4 P2-23: a robot with a drive port DEAD AT BOOT (a hole in its group) driven through the "
          "protections — DEGRADED at entry naming it, a fight cut that logs it ABSENT (dead at boot) "
          "without reading it, the disable summary saying so, and nothing thrown") {
    // BUG CAUGHT (the review of 2026-09-30, S21 / S22 / S23 — no case built its Robot with a boot
    // hole): the protections READING a hole, where MotorGroup::member(i) is a precondition THROW that
    // mid-match would end driver control (landmine 17) — on the first tick's frozen-reading pass, or
    // at the first cut — and the dead list naming runtime marks only (T4-PROGRESS delta 2: a boot
    // hole is dead too, and a panel's "DEGRADED" beside an empty list would say nothing is).
    World w;
    const int absent[1] = {18};
    w.robot.reset();
    w.robot.emplace(robotTwoShape(), w.devices,
                    shulib::robot::BuildOptions{.absentDrivePorts = std::span<const int>{absent}});
    REQUIRE(w.group(0).absentMask() == 0b0100u);
    w.h.plant().setMemberFault(1, 1, MemberFault::Reversed);  // RIGHT port 12 fights
    std::vector<TickLog> log;
    CHECK_NOTHROW(log = w.run(60, [&](int) { w.stick(1.0); }));
    REQUIRE(log.size() == 60U);
    CHECK(cutTicks(log).size() == 1U);
    CHECK(log[0].st.mode == ProtectionMode::Degraded);
    CHECK(std::string{log[0].st.dead} == "18");
    CHECK(log[0].st.verdict.verdict == DriveVerdict::DriveDegraded);
    const auto lines = telLines(w.sink);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0]
          == "driver control starts DEGRADED (left dead 1, right dead 0, total 1) -- dead: 18");
    CHECK(countContaining(lines, "  at the cut: LEFT port 18 ABSENT (dead at boot)") == 1);
    CHECK(countContaining(lines, " | port 18 dead at boot") == 1);
}

namespace {

/// A hand-made HOLONOMIC chassis with a MotorGroup per wheel — four groups of two FakeMotors, built
/// without labels — over the library's own pieces, the way chassis.hpp's standalone recipe wires
/// one. No Robot builds such a chassis; kMaxProtectedGroups (4) says the protections cover it.
struct FourGroupRig {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeImu imu;
    shulib::hal::AbsentGps gps;
    shulib::hal::AbsentTagSource tags;
    shulib::hal::AbsentVision vision;
    FakeTelemetrySink sink;
    std::array<FakeMotor, 8> m{};
    std::array<IMotor*, 2> p0{&m[0], &m[1]};
    std::array<IMotor*, 2> p1{&m[2], &m[3]};
    std::array<IMotor*, 2> p2{&m[4], &m[5]};
    std::array<IMotor*, 2> p3{&m[6], &m[7]};
    MotorGroup g0{std::span<IMotor* const>{p0}};
    MotorGroup g1{std::span<IMotor* const>{p1}};
    MotorGroup g2{std::span<IMotor* const>{p2}};
    MotorGroup g3{std::span<IMotor* const>{p3}};
    std::array<IMotor*, 4> drive{&g0, &g1, &g2, &g3};
    std::array<MotorGroup*, 4> groups{&g0, &g1, &g2, &g3};
    shulib::kinematics::MatrixKinematics xkin = shulib::kinematics::xDrive(Length{7.0});
    shulib::chassis::RobotContext ctx{{.clock = &clock,
                                       .driveMotors = drive,
                                       .imu = &imu,
                                       .gps = &gps,
                                       .battery = &battery,
                                       .telemetry = &sink,
                                       .tags = &tags,
                                       .vision = &vision}};
    shulib::localization::DriveEncoderOdometry odom{imu, g0, g1, shulib::hal::DriveGeometry{},
                                                    shulib::hal::DriveGeometry{},
                                                    Length{kTrackWidth}};
    shulib::localization::ComplementaryFusion fusion{};
    shulib::localization::Localizer loc{clock, imu, odom, fusion};
    shulib::diag::FaultLatch latch{sink, clock};
    shulib::diag::HealthMonitor health{latch};
    shulib::motion::MotionDeps deps{.ctx = &ctx,
                                    .localizer = &loc,
                                    .kinematics = &xkin,
                                    .faults = &latch,
                                    .health = &health,
                                    .motorGroups = groups};
    ClockPacer pacer{clock};
    shulib::chassis::Chassis chassis{deps, pacer, motion_rig::chassisConfig()};
    FourGroupRig() { battery.setVoltage(shulib::units::Voltage{12.8}); }
};

}  // namespace

TEST_CASE("T4 P2-24: a chassis with FOUR groups (a group per wheel) — the census counts every group: two "
          "dead in groups 2 and 3 are DEGRADED, a third anywhere REFUSES from the next entry") {
    // BUG CAUGHT (the review of 2026-09-30, S25): the census counting groups 0 and 1 only — on a
    // chassis with more groups (kMaxProtectedGroups says four are covered), a death in a third or
    // fourth group would never count toward "more than two dead in total".
    FourGroupRig r;
    REQUIRE(r.g2.markAbsent(0));
    REQUIRE(r.g3.markAbsent(1));
    {
        DriveProtection p{r.chassis, DriveProtectionConfig{}, Time{0.01}};
        p.enter();
        CHECK(p.state().groupCount == 4U);
        CHECK(p.state().mode == ProtectionMode::Degraded);
        CHECK(p.state().verdict.verdict == DriveVerdict::DriveDegraded);
        CHECK(p.state().verdict.leftDead == 0);   // group 0 is one "side" …
        CHECK(p.state().verdict.rightDead == 2);  // … every other group the other
        CHECK(p.state().verdict.totalDead == 2);
    }
    REQUIRE(r.g1.markAbsent(0));
    DriveProtection q{r.chassis, DriveProtectionConfig{}, Time{0.01}};
    q.enter();
    CHECK(q.state().mode == ProtectionMode::Refused);
    CHECK(q.state().verdict.totalDead == 3);
    CHECK_FALSE(q.commandAllowed());
    const auto lines = telLines(r.sink);
    CHECK(countContaining(lines, "driver control starts DEGRADED (left dead 0, right dead 2, total 2)")
          == 1);
    CHECK(countContaining(lines, "REFUSED at the start of driver control: more than 2 motors dead in "
                                 "total (left dead 0, right dead 3, total 3)")
          == 1);
}

// ═══ The source pins: who gets the protections from the default ═════════════════════════════
namespace {
std::string readSource(const std::string& relative) {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/" + relative};
    REQUIRE_MESSAGE(in.good(), relative);
    std::stringstream s;
    s << in.rdbuf();
    return s.str();
}
}  // namespace

TEST_CASE("T4 Part 2 source pins: \"shulib Teleop\"'s loopCfg, the driver file's kLoop and the "
          "X-drive's kLoopConfig name no `protection` — so all three get the library's defaults (no host "
          "test compiles Teleop's or the X-drive's TU)") {
    // BUG CAUGHT: a program that quietly turns the protections down, or off, in its own config — the
    // protections are the library's, not a program's.
    const std::string teleop = readSource("src/programs/teleop_program.cpp");
    const std::size_t at = teleop.find("const teleop::DriverLoopConfig loopCfg{");
    REQUIRE(at != std::string::npos);
    const std::size_t end = teleop.find("};", at);
    REQUIRE(end != std::string::npos);
    const std::string block = teleop.substr(at, end - at);
    CHECK(block.find("tickPeriod") != std::string::npos);
    CHECK(block.find("protection") == std::string::npos);
    CHECK(teleop.find(".protection") == std::string::npos);
    CHECK(teleop.find("loopCfg, st.claws.holds()") != std::string::npos);  // the call that uses it
    const std::string driver = readSource("src/team/robot_two_driver.hpp");
    const std::size_t k = driver.find("static constexpr teleop::DriverLoopConfig kLoop{");
    REQUIRE(k != std::string::npos);
    const std::string kLoop = driver.substr(k, driver.find("};", k) - k);
    CHECK(kLoop.find("protection") == std::string::npos);
    // The invented X-drive (its loop's header names it as a caller that gets the defaults).
    const std::string xdrive = readSource("src/programs/xdrive_program.cpp");
    const std::size_t x = xdrive.find("constexpr teleop::DriverLoopConfig kLoopConfig{");
    REQUIRE(x != std::string::npos);
    const std::string kLoopConfig = xdrive.substr(x, xdrive.find("};", x) - x);
    CHECK(kLoopConfig.find("tickPeriod") != std::string::npos);
    CHECK(kLoopConfig.find("protection") == std::string::npos);
    CHECK(xdrive.find(".protection") == std::string::npos);
}
