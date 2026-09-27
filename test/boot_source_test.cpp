// boot_source_test.cpp — G1 Part 2b's test 6: the boot source feeds the builder faithfully
// (src/common/boot_source.hpp, included from src/ the way chassis_table_test.cpp includes
// src/robots/chassis_table.hpp — the host build compiles test/*.cpp only, never a .cpp under src/).
//
// As the brief's amendment A11 strengthened it, and as the Part 2b reviews sharpened it:
//   * the config the core builds is valid, carries the description's SOURCE, the drive-encoder
//     odometry, no GPS — and `tuning` equal to chassis::ChassisConfig{} FIELD BY FIELD, written
//     out longhand on purpose (there is no operator==, and a memcmp would compare padding: the
//     structs hold bools, a uint32 mask and a pointer — test/blackbox_format_test.cpp's
//     precedent). A MEMBER-COUNT pin on every struct in that comparison makes a new field fail
//     here until the comparison lists it — a size pin alone could not: a bool after
//     OdoStallCheckConfig::independentMotionSource, or a uint32 after
//     MotionSchedulerConfig::abortFaultMask, lands in padding and leaves every size unchanged
//     (the second review round's finding). The size pins stay, for a changed member TYPE;
//   * the per-side scales and the track width reach the built Robot's drive-encoder odometry
//     EXACTLY — `==`, never Approx: the record stores f64 and fromInchesPerRadian is x 1.0, so an
//     exact check is possible, and Approx would let a float round trip (~3e-8 relative) survive;
//   * robot::Robot::portMap() equals the 2026-09-25 capture's `[SES] ports` line, from a
//     description rebuilt from what that capture printed — a literal copied from the evidence,
//     never a string computed by the code under test;
//   * a missing port gives EXACTLY that port in absent() (not merely one port — on robot two all
//     eight ports are drive ports, so a size-only check would pass a core that recorded the
//     wrong one), validate() passes with the core's own buildOptions(), and the Robot builds
//     degraded around it without ever asking the provider for it;
//   * THE WHOLE BOOT DECISION (6k-6r, the second review round): decideBootSourceFrom() over a
//     FakeBlockSource holding an ENCODED record and a FakeDeviceRegistry reproduces the robot's
//     own 2026-09-25 decision lines word for word, and every other row of the decision table —
//     a degraded record, a refused record that must NOT fall back to the table, no card with a
//     usable table, no card with an unusable one, an open but empty file, a truncated record,
//     a table that fails its own port check — with the lines each prints, the source chosen, the
//     description built from, and the refusal's words. Every one reads the config and the
//     builder's options through the outcome's `built` member, exactly as the program does.
//
// EVIDENCE: docs/internal/evidence/r3d-session-2026-09-25-teleop.log.gz, read with
// `zcat … | strings` — the first boot block, lines 76-106 (the line numbers below are that
// command's). NO HARDWARE CLAIM: every registry answer here is a FakeDeviceRegistry's, every
// device a FakeDeviceProvider's.

#include "doctest.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "common/boot_source.hpp"
#include "shulib/calibration/boot_check.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/control/feedforward.hpp"
#include "shulib/control/settled_util.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/loop_monitor.hpp"
#include "shulib/diag/plausibility_guard.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_block_source.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/units/quantity.hpp"

namespace {

using shulib::PreconditionError;
using shulib::bench::BootConfig;
using shulib::bench::bootConfigFrom;
using shulib::bench::BootOutcome;
using shulib::bench::countPortDisagreements;
using shulib::bench::decideBootSourceFrom;
using shulib::bench::kNoMechanisms;
using shulib::bench::readRecord;
using shulib::bench::RecordRead;
using shulib::calibration::BootDecision;
using shulib::calibration::BootSource;
using shulib::calibration::BootVerdict;
using shulib::calibration::verifyAgainstRegistry;
using shulib::hal::DescriptionSource;
using shulib::hal::DeviceKind;
using shulib::hal::DrivetrainDescription;
using shulib::hal::DrivetrainKind;
using shulib::hal::MotorCartridge;
using shulib::hal::fake::FakeBattery;
using shulib::hal::fake::FakeBlockSource;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::hal::fake::FakeDeviceRegistry;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::units::Length;
namespace cfgns = shulib::config;

/// The name src/robots/chassis_table.hpp gives robot two, which the teleop program passes as the robot's
/// identity (kChassis.robot in the ROBOT=tank build).
constexpr std::string_view kRobotTwoName = "2026 TANK CHASSIS (robot two)";

// ── robot two's calibration record, REBUILT FROM THE 2026-09-25 CAPTURE ─────────────────
//   :94  [SES] ports MEASURED L -16 +17 -18 -20 R +11 -12 +13 +15 IMU2 ipr 1.2544/1.3670 TW12.01
//   :95  ... MEASURED tank BLUE 600 rpm: L4/R4 ports, ipr L1.25442 R1.36703, W 12.006 in
//   :97  description provenance: shulib Calibrate Sep 24 2026 17:42:16 | ROBOT-MEASURED: 8 ports,
//        push 48 in, W 12.006 in
// The signed ports are as the record STORES them (the port map prints them in stored order); the
// scales and the width are the 5- and 3-decimal values line 95 printed. The record's own doubles
// may carry more digits than the log shows; what this test pins is that whatever the description
// carries reaches the graph unchanged.
constexpr std::int8_t kRecordLeft[4] = {-16, 17, -18, -20};
constexpr std::int8_t kRecordRight[4] = {11, -12, 13, 15};
constexpr double kLeftIpr = 1.25442;
constexpr double kRightIpr = 1.36703;
constexpr double kTrackWidth = 12.006;
constexpr const char* kProvenance =
    "shulib Calibrate Sep 24 2026 17:42:16 | ROBOT-MEASURED: 8 ports, push 48 in, W 12.006 in";

DrivetrainDescription robotTwoRecord() {
    DrivetrainDescription d{};
    d.kind = DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 2;
    for (std::size_t i = 0; i < 4; ++i) {
        d.left.signedPorts[i] = kRecordLeft[i];
        d.right.signedPorts[i] = kRecordRight[i];
    }
    d.left.count = 4;
    d.right.count = 4;
    d.left.inchesPerRadian = Length{kLeftIpr};
    d.right.inchesPerRadian = Length{kRightIpr};
    d.trackWidth = Length{kTrackWidth};
    d.source = DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "%s", kProvenance);
    return d;
}

/// Robot two's port census as the 2026-09-25 boot saw it: motors on the eight recorded ports and
/// on 14 ("NEW motors not in the calibration: 14"), nothing on 19, the IMU on 2, the radio on 1,
/// the AI Vision camera on 3.
FakeDeviceRegistry robotTwoCensus() {
    FakeDeviceRegistry reg;
    for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 20}) {
        reg.setKind(p, DeviceKind::Motor);
    }
    reg.setKind(1, DeviceKind::Radio);
    reg.setKind(2, DeviceKind::Imu);
    reg.setKind(3, DeviceKind::Vision);
    return reg;
}

/// Robot two's TYPED table as src/robots/chassis_table.hpp carries it for the ROBOT=tank builds
/// (kRobotTwoChassis; its front is the 15/16 end), typed here by hand as the description the pure
/// core is handed. (Until G1 Part 2c's final run the host could not see robot two's table at all —
/// it was defined only under the tank define; test/chassis_table_test.cpp now pins the real table,
/// its name and its missing-geometry line to the same literals this file re-types.)
DrivetrainDescription robotTwoTableSigns() {
    constexpr std::int8_t left[5] = {-11, 12, -13, 14, -15};
    constexpr std::int8_t right[5] = {20, -19, 18, -17, 16};
    DrivetrainDescription d{};
    d.kind = DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 2;
    for (std::size_t i = 0; i < 5; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 5;
    d.right.count = 5;
    d.source = DescriptionSource::TypedTable;
    return d;
}

/// A host world with no plant: the provider's own fakes on every port, a clock, a battery, a sink
/// and a pacer that is never paced (nothing here drives).
struct NoopPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};
struct World {
    FakeClock clock{};
    FakeBattery battery{};
    FakeTelemetrySink sink{};
    NoopPacer pacer{};
    FakeDeviceProvider devices{clock, battery, sink, pacer};
};

/// Every field of a description, longhand — including the unused tail of each port array, which
/// a core that copied `count` entries into a fresh struct would leave zero and a core that copied
/// garbage would not.
void requireSameDescription(const DrivetrainDescription& got, const DrivetrainDescription& want) {
    REQUIRE(got.kind == want.kind);
    REQUIRE(got.cartridge == want.cartridge);
    REQUIRE(got.imuPort == want.imuPort);
    REQUIRE(got.source == want.source);
    REQUIRE(got.left.count == want.left.count);
    REQUIRE(got.right.count == want.right.count);
    for (std::size_t i = 0; i < shulib::hal::kMaxSmartPorts; ++i) {
        REQUIRE(got.left.signedPorts[i] == want.left.signedPorts[i]);
        REQUIRE(got.right.signedPorts[i] == want.right.signedPorts[i]);
    }
    REQUIRE(got.left.inchesPerRadian.value() == want.left.inchesPerRadian.value());    // exact
    REQUIRE(got.right.inchesPerRadian.value() == want.right.inchesPerRadian.value());  // exact
    REQUIRE(got.trackWidth.value() == want.trackWidth.value());                        // exact
    REQUIRE(std::strcmp(got.provenance, want.provenance) == 0);
}

// ── Every struct requireDefaultTuning() walks, pinned by its MEMBER COUNT ────────────
// A NEW field in any of them must fail HERE until the longhand comparison below lists it —
// otherwise a core that set it would survive. The count is what brace-initialisation accepts:
// `T{f, f, …}` compiles with up to as many initialisers as T has members, and a field that only
// fills padding still adds one (sizeof cannot see that: see the file header). AnyField converts
// to any member type, so no brace elision splits a nested aggregate into its leaves.
struct AnyField {
    template <class T>
    operator T() const;  // declared only: used in unevaluated brace-initialisation below
};
template <class T, std::size_t... I>
constexpr bool bracesAccept(std::index_sequence<I...> /*unused*/) {
    return requires { T{((void)I, AnyField{})...}; };
}
template <class T, std::size_t N = 0>
constexpr std::size_t memberCount() {
    if constexpr (bracesAccept<T>(std::make_index_sequence<N + 1>{})) {
        return memberCount<T, N + 1>();
    } else {
        return N;
    }
}
static_assert(memberCount<shulib::chassis::ChassisConfig>() == 2,
              "ChassisConfig gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::motion::MotionConfig>() == 12,
              "MotionConfig gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::control::FeedforwardGains>() == 3,
              "FeedforwardGains gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::motion::AxisGains>() == 4,
              "AxisGains gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::control::SettleConfig>() == 3,
              "SettleConfig gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::motion::OdoStallCheckConfig>() == 6,
              "OdoStallCheckConfig gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::hal::DriveGeometry>() == 2,
              "DriveGeometry gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::motion::MotionSchedulerConfig>() == 4,
              "MotionSchedulerConfig gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::diag::LoopMonitorConfig>() == 1,
              "LoopMonitorConfig gained a member: compare it in requireDefaultTuning()");
static_assert(memberCount<shulib::diag::PlausibilityConfig>() == 3,
              "PlausibilityConfig gained a member: compare it in requireDefaultTuning()");
// And the sizes, measured on the host (x86-64/LP64): a member whose TYPE changed.
static_assert(sizeof(shulib::motion::MotionConfig) == 368,
              "MotionConfig changed size: check requireDefaultTuning() still compares it all");
static_assert(sizeof(shulib::motion::MotionSchedulerConfig) == 48,
              "MotionSchedulerConfig changed size: check requireDefaultTuning() still compares it all");
static_assert(sizeof(shulib::chassis::ChassisConfig) == 416,
              "ChassisConfig changed size: check requireDefaultTuning() still compares it all");

/// `t` equals chassis::ChassisConfig{} in EVERY leaf, written out longhand (file header).
void requireDefaultTuning(const shulib::chassis::ChassisConfig& t) {
    const shulib::chassis::ChassisConfig ref{};
    const shulib::motion::MotionConfig& m = t.motion;
    const shulib::motion::MotionConfig& r = ref.motion;
    REQUIRE(m.wheelFf.kS == r.wheelFf.kS);
    REQUIRE(m.wheelFf.kV == r.wheelFf.kV);
    REQUIRE(m.wheelFf.kA == r.wheelFf.kA);
    const shulib::motion::AxisGains* gm[2] = {&m.translation, &m.heading};
    const shulib::motion::AxisGains* gr[2] = {&r.translation, &r.heading};
    for (int i = 0; i < 2; ++i) {
        REQUIRE(gm[i]->kP == gr[i]->kP);
        REQUIRE(gm[i]->kI == gr[i]->kI);
        REQUIRE(gm[i]->kD == gr[i]->kD);
        REQUIRE(gm[i]->integralLimit == gr[i]->integralLimit);  // +inf == +inf
    }
    REQUIRE(m.maxLinearSpeed.value() == r.maxLinearSpeed.value());
    REQUIRE(m.maxAngularSpeed.value() == r.maxAngularSpeed.value());
    REQUIRE(m.maxWheelSpeed.value() == r.maxWheelSpeed.value());
    const shulib::control::SettleConfig* sm[3] = {&m.translationSettle, &m.headingSettle,
                                                  &m.brakeSettle};
    const shulib::control::SettleConfig* sr[3] = {&r.translationSettle, &r.headingSettle,
                                                  &r.brakeSettle};
    for (int i = 0; i < 3; ++i) {
        REQUIRE(sm[i]->maxError == sr[i]->maxError);
        REQUIRE(sm[i]->maxErrorRate == sr[i]->maxErrorRate);
        REQUIRE(sm[i]->settleTime == sr[i]->settleTime);
    }
    REQUIRE(m.defaultTimeout == r.defaultTimeout);
    REQUIRE(m.rotationRadius.value() == r.rotationRadius.value());
    // The stall check: the field a port of main.cpp's tankChassisConfig() into the core would set
    // (setAllWheels + independentMotionSource) — robot::Robot::effectiveTuning() overwrites both
    // for a tank, so ONLY this check can see it.
    REQUIRE(m.stall.window == r.stall.window);
    REQUIRE(m.stall.minSpinTravel.value() == r.stall.minSpinTravel.value());
    REQUIRE(m.stall.motionRatio == r.stall.motionRatio);
    REQUIRE(m.stall.wheels.size() == r.stall.wheels.size());
    for (std::size_t i = 0; i < m.stall.wheels.size(); ++i) {
        REQUIRE(m.stall.wheels[i].wheelRadius.value() == r.stall.wheels[i].wheelRadius.value());
        REQUIRE(m.stall.wheels[i].motorToWheelRatio == r.stall.wheels[i].motorToWheelRatio);
    }
    REQUIRE(m.stall.rotationRadius.value() == r.stall.rotationRadius.value());
    REQUIRE(m.stall.independentMotionSource == r.stall.independentMotionSource);
    // The scheduler's fault policy, loop budget, attribution clock and plausibility envelope.
    REQUIRE(t.scheduler.abortFaultMask == ref.scheduler.abortFaultMask);
    REQUIRE(t.scheduler.loopMonitor.budget.value() == ref.scheduler.loopMonitor.budget.value());
    REQUIRE(t.scheduler.attributionClock == ref.scheduler.attributionClock);
    REQUIRE(t.scheduler.plausibility.maxSpeed.value()
            == ref.scheduler.plausibility.maxSpeed.value());
    REQUIRE(t.scheduler.plausibility.maxYawRate.value()
            == ref.scheduler.plausibility.maxYawRate.value());
    REQUIRE(t.scheduler.plausibility.margin == ref.scheduler.plausibility.margin);
}

}  // namespace

TEST_CASE("G1 2b test 6a: the census rebuilt from the capture reproduces the 2026-09-25 port check") {
    // :83-84 calibration record: DECODED, port check OK -- <this reason, whole>. If this fixture
    // did not reproduce the robot's own verdict, nothing below would be about robot two.
    const FakeDeviceRegistry reg = robotTwoCensus();
    const BootVerdict v = verifyAgainstRegistry(robotTwoRecord(), reg);
    REQUIRE(v.decision == BootDecision::Ok);
    CHECK(std::string_view{v.reason}
          == "OK: all 8 recorded drive ports carry motors, IMU on port 2. NEW motors not in the "
             "calibration: 14. NOT CHECKED: two drive cables swapped WITH EACH OTHER look identical "
             "here -- the first drive catches that");
}

TEST_CASE("G1 2b test 6b: the pure core carries robot two's record into a RobotConfig faithfully") {
    const DrivetrainDescription record = robotTwoRecord();
    const BootVerdict v = verifyAgainstRegistry(record, robotTwoCensus());
    REQUIRE(v.decision == BootDecision::Ok);
    const BootConfig boot = bootConfigFrom(record, v, kRobotTwoName, kNoMechanisms);
    const cfgns::RobotConfig& c = boot.config;

    char why[192] = "";
    REQUIRE_MESSAGE(c.valid(why, sizeof why), why);
    CHECK(std::string_view{c.identity.name} == kRobotTwoName);
    CHECK(c.identity.team[0] == '\0');
    CHECK(c.identity.season[0] == '\0');
    REQUIRE(c.drivetrain.kind == cfgns::DrivetrainKind::Tank);
    requireSameDescription(c.drivetrain.tank.description, record);  // source preserved: MEASURED
    CHECK(c.odometry.kind == cfgns::OdometryKind::DriveEncoders);
    CHECK_FALSE(c.sensors.gps.present);
    CHECK(c.mechanismCount == 0);
    requireDefaultTuning(c.tuning);
    // Nothing is absent on an Ok verdict, and the options say so.
    CHECK(boot.absent().empty());
    CHECK(boot.buildOptions().absentDrivePorts.empty());
}

TEST_CASE("G1 2b test 6c: robot::Robot::portMap() equals the 2026-09-25 capture's [SES] ports line") {
    const DrivetrainDescription record = robotTwoRecord();
    const BootConfig boot = bootConfigFrom(record, verifyAgainstRegistry(record, robotTwoCensus()),
                                           kRobotTwoName, kNoMechanisms);
    // :94 — copied from the evidence, not computed by the code under test.
    constexpr std::string_view kCapturedPortMap =
        "MEASURED L -16 +17 -18 -20 R +11 -12 +13 +15 IMU2 ipr 1.2544/1.3670 TW12.01";
    static_assert(kCapturedPortMap.size() < shulib::diag::kMaxPortMapBytes,
                  "the golden must fit the session header's field for equality to mean anything");
    World w;
    REQUIRE(shulib::robot::validate(boot.config, w.devices, boot.buildOptions()).ok);
    std::optional<shulib::robot::Robot> robot;
    robot.emplace(boot.config, w.devices, boot.buildOptions());
    char map[shulib::diag::kMaxPortMapBytes];
    robot->portMap(map, sizeof map);
    CHECK(std::string_view{map} == kCapturedPortMap);
    // The NEW boot line the program prints as `robot::Robot: <this>` (193 bytes with its prefix,
    // one TermSink message): typed by hand from robot.hpp's format, so a change to the banner shows
    // up here and in the Part 2b banner diff at once.
    char banner[256];
    robot->describe(banner, sizeof banner);
    CHECK(std::string_view{banner}
          == "2026 TANK CHASSIS (robot two) [] tank MEASURED | L -16,17,-18,-20 R 11,-12,13,15 | "
             "ipr L1.25442 R1.36703 W 12.006 in | IMU2 | drive-encoders | GPS ABSENT tags ABSENT "
             "vision ABSENT");
    // :95 — the description's own banner line, which the program prints beside robot.describe().
    char described[128];
    boot.config.drivetrain.tank.description.describe(described, sizeof described);
    CHECK(std::string_view{described}
          == "MEASURED tank BLUE 600 rpm: L4/R4 ports, ipr L1.25442 R1.36703, W 12.006 in");
}

TEST_CASE("G1 2b test 6d: the built Robot carries both scales and the width EXACTLY, and the "
          "provisional tuning the capture printed") {
    const DrivetrainDescription record = robotTwoRecord();
    const BootConfig boot = bootConfigFrom(record, verifyAgainstRegistry(record, robotTwoCensus()),
                                           kRobotTwoName, kNoMechanisms);
    World w;
    std::optional<shulib::robot::Robot> robot;
    robot.emplace(boot.config, w.devices, boot.buildOptions());
    const shulib::localization::DriveEncoderOdometry* odom = robot->driveOdometry();
    REQUIRE(odom != nullptr);
    // == on purpose (file header): Approx's default epsilon would pass a float round trip.
    CHECK(odom->leftInchesPerRadian().value() == kLeftIpr);
    CHECK(odom->rightInchesPerRadian().value() == kRightIpr);
    CHECK(odom->trackWidth().value() == kTrackWidth);
    // :96 — "4+4 slots behind two MotorGroups, 4+4 PRESENT".
    const auto groups = robot->chassis().deps().motorGroups;
    REQUIRE(groups.size() == 2);
    CHECK(groups[0]->memberCount() == 4);
    CHECK(groups[1]->memberCount() == 4);
    CHECK(groups[0]->presentCount() == 4);
    CHECK(groups[1]->presentCount() == 4);
    // :105 — the facade line's budgets and gains, formatted exactly as the program formats them.
    const shulib::motion::MotionConfig& mc = robot->chassis().motionConfig();
    char facade[160];
    std::snprintf(facade, sizeof facade,
                  "budgets lin %.0f in/s ang %.2f rad/s wheel %.0f in/s; ff kS %.2f kV %.4f kA %.4f",
                  mc.maxLinearSpeed.value(), mc.maxAngularSpeed.value(), mc.maxWheelSpeed.value(),
                  mc.wheelFf.kS, mc.wheelFf.kV, mc.wheelFf.kA);
    CHECK(std::string_view{facade}
          == "budgets lin 60 in/s ang 6.00 rad/s wheel 60 in/s; ff kS 1.00 kV 0.1714 kA 0.0000");
    CHECK(mc.wheelFf.kV == 12.0 / 70.0);
    // The stall check told the truth by the BUILDER (effectiveTuning), not by the core: no
    // independent source, every wheel slot the LEFT geometry (canDetectStall=false in :105).
    CHECK_FALSE(mc.stall.independentMotionSource);
    for (const shulib::hal::DriveGeometry& g : mc.stall.wheels) {
        CHECK(g.inchesPerRadian().value() == kLeftIpr);
    }
    CHECK(robot->chassis().strafeAuthority() == 0.0);
    CHECK_FALSE(robot->hasGpsCorrector());
}

TEST_CASE("G1 2b test 6e: one missing RIGHT port (13) -- exactly that port is absent, validate() "
          "passes with the core's options, and the Robot builds degraded around it") {
    const DrivetrainDescription record = robotTwoRecord();
    FakeDeviceRegistry reg = robotTwoCensus();
    reg.setKind(13, DeviceKind::None);  // the right side's third port, +13 as stored
    const BootVerdict v = verifyAgainstRegistry(record, reg);
    REQUIRE(v.decision == BootDecision::Degraded);
    CHECK(std::string_view{v.reason}.starts_with("DEGRADED: dead 13 -- driving without them"));
    const BootConfig boot = bootConfigFrom(record, v, kRobotTwoName, kNoMechanisms);

    // WHICH port, not merely how many: every port on robot two is a drive port, so a core that
    // recorded the wrong one would still give size 1 and still pass validate().
    REQUIRE(boot.absent().size() == 1);
    CHECK(boot.absent()[0] == 13);
    const shulib::robot::BuildOptions opts = boot.buildOptions();
    REQUIRE(opts.absentDrivePorts.size() == 1);
    CHECK(opts.absentDrivePorts[0] == 13);

    World w;
    w.devices.refusePort(13);  // as the brain would: the registry says the port is EMPTY
    const shulib::robot::BuildVerdict bv = shulib::robot::validate(boot.config, w.devices, opts);
    REQUIRE_MESSAGE(bv.ok, bv.why);
    std::optional<shulib::robot::Robot> robot;
    robot.emplace(boot.config, w.devices, boot.buildOptions());
    const auto groups = robot->chassis().deps().motorGroups;
    CHECK(groups[0]->presentCount() == 4);
    CHECK(groups[0]->memberCount() == 4);
    CHECK(groups[1]->presentCount() == 3);
    CHECK(groups[1]->memberCount() == 4);
    CHECK(w.devices.motorRequestCount(13) == 0);  // never asked for: named once, by the boot check
    char banner[256];
    robot->describe(banner, sizeof banner);
    CHECK(std::strstr(banner, "DEGRADED: absent 13") != nullptr);
}

TEST_CASE("G1 2b test 6f: one missing port per side, NEGATIVE entries -- magnitudes, LEFT then "
          "RIGHT in the description's order") {
    const DrivetrainDescription record = robotTwoRecord();
    FakeDeviceRegistry reg = robotTwoCensus();
    reg.setKind(18, DeviceKind::None);  // left, stored -18
    reg.setKind(12, DeviceKind::None);  // right, stored -12
    const BootVerdict v = verifyAgainstRegistry(record, reg);
    REQUIRE(v.decision == BootDecision::Degraded);
    const BootConfig boot = bootConfigFrom(record, v, kRobotTwoName, kNoMechanisms);
    REQUIRE(boot.absent().size() == 2);
    CHECK(boot.absent()[0] == 18);
    CHECK(boot.absent()[1] == 12);
    World w;
    w.devices.refusePort(18);
    w.devices.refusePort(12);
    const shulib::robot::BuildVerdict bv =
        shulib::robot::validate(boot.config, w.devices, boot.buildOptions());
    REQUIRE_MESSAGE(bv.ok, bv.why);
}

TEST_CASE("G1 2b test 6g: a REFUSED port check is refused by the core -- nothing is built from it") {
    const DrivetrainDescription record = robotTwoRecord();
    FakeDeviceRegistry reg = robotTwoCensus();
    reg.setKind(2, DeviceKind::None);  // the IMU unplugged: the boot check refuses first (HA-141)
    const BootVerdict v = verifyAgainstRegistry(record, reg);
    REQUIRE(v.decision == BootDecision::Refuse);
    CHECK_THROWS_AS((void)bootConfigFrom(record, v, kRobotTwoName, kNoMechanisms), PreconditionError);
}

TEST_CASE("G1 2b test 6h: a TYPED description stays TYPED -- the source is carried, not assumed") {
    DrivetrainDescription typed = robotTwoTableSigns();
    typed.left.inchesPerRadian = Length{1.375};  // a 2.75 in wheel, direct drive
    typed.right.inchesPerRadian = Length{1.375};
    typed.trackWidth = Length{12.5};
    std::snprintf(typed.provenance, sizeof typed.provenance, "%s", "TYPED TABLE (test)");
    FakeDeviceRegistry reg;
    for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 19, 20}) {
        reg.setKind(p, DeviceKind::Motor);
    }
    reg.setKind(2, DeviceKind::Imu);
    const BootVerdict v = verifyAgainstRegistry(typed, reg);
    REQUIRE(v.decision == BootDecision::Ok);
    const BootConfig boot = bootConfigFrom(typed, v, kRobotTwoName, kNoMechanisms);
    requireSameDescription(boot.config.drivetrain.tank.description, typed);
    CHECK(boot.config.drivetrain.tank.description.source == DescriptionSource::TypedTable);
    char map[shulib::diag::kMaxPortMapBytes];
    shulib::robot::writePortMap(boot.config, map, sizeof map);
    CHECK(std::string_view{map}
          == "TYPED L -11 +12 -13 +14 -15 R +20 -19 +18 -17 +16 IMU2 ipr 1.3750/1.3750 TW12.50");
}

TEST_CASE("G1 2b test 6i: the record/table cross-check reads 10 on 2026-09-25's record, as the robot "
          "did") {
    // :100 [WARN][R3B] CROSS-CHECK: the MEASURED record and the typed chassis table disagree about
    // 10 port(s). — the record's front is the table's back (every shared port's sign flips), and
    // 14 and 19 are listed only by the table.
    const DrivetrainDescription record = robotTwoRecord();
    const DrivetrainDescription table = robotTwoTableSigns();
    CHECK(countPortDisagreements(record, table) == 10);
    CHECK(countPortDisagreements(table, record) == 10);
    CHECK(countPortDisagreements(record, record) == 0);
    DrivetrainDescription oneFlipped = record;
    oneFlipped.right.signedPorts[2] = -13;
    CHECK(countPortDisagreements(record, oneFlipped) == 1);
    // The ends of the port range count too: port 1 and port 21, each listed by one side only.
    DrivetrainDescription ends = record;
    ends.right.signedPorts[4] = 21;
    ends.right.count = 5;
    ends.left.signedPorts[4] = -1;
    ends.left.count = 5;
    CHECK(countPortDisagreements(record, ends) == 2);
}

TEST_CASE("G1 2b test 6j: the robot's name is bounded to the identity field, never overflowing") {
    const DrivetrainDescription record = robotTwoRecord();
    const BootVerdict v = verifyAgainstRegistry(record, robotTwoCensus());
    const std::string_view longName = "a name much longer than the thirty-one characters that fit";
    const BootConfig boot = bootConfigFrom(record, v, longName, kNoMechanisms);
    const std::size_t cap = sizeof boot.config.identity.name;
    CHECK(std::strlen(boot.config.identity.name) == cap - 1);
    CHECK(std::string_view{boot.config.identity.name} == longName.substr(0, cap - 1));
    const BootConfig empty = bootConfigFrom(record, v, std::string_view{}, kNoMechanisms);
    CHECK(empty.config.identity.name[0] == '\0');
}

// ═══ 6k-6r: THE WHOLE BOOT DECISION, run against the fakes (the second review round) ════════
// decideBootSourceFrom() is everything src/common/boot_source.cpp's decideBootSource() decides; the .cpp
// only opens the card, makes the registry, turns kChassis into a description and prints. So these
// cases hand it what the brain would: a FakeBlockSource holding an ENCODED record (or refusing,
// or empty, or short), a FakeDeviceRegistry census, and a table description — and read back the
// lines it prints, the source it chose, the description it built from and the refusal's words.
namespace {

constexpr const char* kCardPath = "/usd/shulib_drivetrain.bin";  // ProsBlockSource's path()
/// describeMissingForDescription(kChassis) in the ROBOT=tank build, as the robot printed it (:85).
constexpr const char* kTankTableMissing = "track width, external ratio (motor->wheel)";

std::vector<std::byte> encoded(const DrivetrainDescription& d) {
    std::vector<std::byte> bytes(shulib::calibration::kRecordBytes);
    REQUIRE(shulib::calibration::encodeRecord(d, bytes) == shulib::calibration::kRecordBytes);
    return bytes;
}

/// Robot two's typed table with geometry typed in — a table that CAN describe a drivetrain
/// (the committed one cannot: robotTwoTableSigns() leaves both scales and the width 0).
DrivetrainDescription usableTable() {
    DrivetrainDescription t = robotTwoTableSigns();
    t.left.inchesPerRadian = Length{1.375};
    t.right.inchesPerRadian = Length{1.375};
    t.trackWidth = Length{12.5};
    std::snprintf(t.provenance, sizeof t.provenance, "%s", "TYPED TABLE (test)");
    return t;
}

/// The census a usable TABLE matches: motors on all ten of its ports, the IMU on 2.
FakeDeviceRegistry tableCensus() {
    FakeDeviceRegistry reg;
    for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 19, 20}) {
        reg.setKind(p, DeviceKind::Motor);
    }
    reg.setKind(2, DeviceKind::Imu);
    return reg;
}

struct Boot {
    BootOutcome out;
    std::vector<std::string> lines;
};

/// One boot: read the card, decide, capture every line in order.
/// `mechanisms` defaults to none: the G1 2b cases pin the boot as it was before the claws (and
/// test 6s onward pass robot two's REAL table).
Boot runBoot(FakeBlockSource& card, const FakeDeviceRegistry& registry,
             const DrivetrainDescription& table,
             const shulib::bench::MechanismTable& mechanisms = shulib::bench::kNoMechanisms) {
    Boot b;
    const RecordRead record = readRecord(card, kCardPath);
    b.out = decideBootSourceFrom(record, registry, table, kTankTableMissing, mechanisms,
                                 kRobotTwoName,
                                 [&b](const char* line) { b.lines.emplace_back(line); });
    return b;
}

// The robot's own sentences, which several cases share.
constexpr std::string_view kNoCardWhy =
    "no calibration record: /usd/shulib_drivetrain.bin could not be opened (no SD card, or the "
    "robot has never been calibrated)";
constexpr std::string_view kTankTableLine =  // :85
    "typed table: cannot describe a drivetrain -- UNSET: track width, external ratio "
    "(motor->wheel) (LEFT inches per radian is 0 -- must be finite and > 0 (0 = UNSET))";
constexpr std::string_view kTableWhy = "LEFT inches per radian is 0 -- must be finite and > 0 (0 = UNSET)";
constexpr std::string_view kSourceTableLine =
    "boot source: TYPED TABLE -- no calibration record on the card -- falling back to the TYPED "
    "TABLE, which is unverified since the rebuild; run shulib Calibrate";

}  // namespace

TEST_CASE("G1 2b test 6k: the whole boot decision on robot two's card and census reproduces the "
          "2026-09-25 boot's decision lines, word for word") {
    FakeBlockSource card{encoded(robotTwoRecord())};
    const Boot b = runBoot(card, robotTwoCensus(), robotTwoTableSigns());
    CHECK(card.readCalls() == 1);  // the card is read ONCE
    REQUIRE(b.lines.size() == 4);
    // :83-84, :85, :86 and :100 of the capture, without their "[R3B] " / "[WARN][R3B] " prefixes.
    CHECK(b.lines[0]
          == "calibration record: DECODED, port check OK -- OK: all 8 recorded drive ports carry "
             "motors, IMU on port 2. NEW motors not in the calibration: 14. NOT CHECKED: two drive "
             "cables swapped WITH EACH OTHER look identical here -- the first drive catches that");
    CHECK(b.lines[1] == kTankTableLine);
    CHECK(b.lines[2]
          == "boot source: RECORD -- MEASURED calibration record on the card, verified against the "
             "ports");
    CHECK(b.lines[3]
          == "CROSS-CHECK: the MEASURED record and the typed chassis table disagree about 10 "
             "port(s). The record wins (it moved THIS robot); the table is stale and should be "
             "updated or deleted.");
    REQUIRE(b.out.ok);
    CHECK(b.out.source == BootSource::UseRecord);
    CHECK(b.out.verdict.decision == BootDecision::Ok);
    CHECK(b.out.crossCheckDiffering == 10);
    CHECK(b.out.headline[0] == '\0');
    CHECK(b.out.why[0] == '\0');
    // Built from the RECORD, every field, through the program's own door.
    requireSameDescription(b.out.built.config.drivetrain.tank.description, robotTwoRecord());
    CHECK(std::string_view{b.out.built.config.identity.name} == kRobotTwoName);
    CHECK(b.out.built.absent().empty());
    World w;
    REQUIRE(shulib::robot::validate(b.out.built.config, w.devices, b.out.built.buildOptions()).ok);

    // A table that AGREES with the record prints no cross-check line at all.
    FakeBlockSource again{encoded(robotTwoRecord())};
    const Boot same = runBoot(again, robotTwoCensus(), robotTwoRecord());
    REQUIRE(same.out.ok);
    CHECK(same.out.crossCheckDiffering == 0);
    REQUIRE(same.lines.size() == 2);  // the record line and `boot source:` — the record is valid
    CHECK(same.lines[1].starts_with("boot source: RECORD -- "));
}

TEST_CASE("G1 2b test 6l: a DEGRADED record (port 13 dead) boots degraded, and the outcome's `built` "
          "carries exactly port 13 to the builder") {
    FakeBlockSource card{encoded(robotTwoRecord())};
    FakeDeviceRegistry reg = robotTwoCensus();
    reg.setKind(13, DeviceKind::None);
    const Boot b = runBoot(card, reg, robotTwoTableSigns());
    REQUIRE(b.lines.size() == 4);
    CHECK(b.lines[0]
          == "calibration record: DECODED, port check DEGRADED -- DEGRADED: dead 13 -- driving "
             "without them (left dead 0, right dead 1). NEW motors not in the calibration: 14");
    CHECK(b.lines[2]
          == "boot source: RECORD -- MEASURED calibration record on the card; some ports are dead "
             "and the drivetrain is built without them");
    CHECK(b.lines[3].starts_with("CROSS-CHECK: "));
    REQUIRE(b.out.ok);
    CHECK(b.out.source == BootSource::UseRecord);
    CHECK(b.out.verdict.decision == BootDecision::Degraded);
    // The program's spelling, and the only door: `built`.
    REQUIRE(b.out.built.absent().size() == 1);
    CHECK(b.out.built.absent()[0] == 13);
    const shulib::robot::BuildOptions opts = b.out.built.buildOptions();
    REQUIRE(opts.absentDrivePorts.size() == 1);
    CHECK(opts.absentDrivePorts[0] == 13);
    World w;
    w.devices.refusePort(13);
    const shulib::robot::BuildVerdict bv =
        shulib::robot::validate(b.out.built.config, w.devices, b.out.built.buildOptions());
    REQUIRE_MESSAGE(bv.ok, bv.why);
    std::optional<shulib::robot::Robot> robot;
    robot.emplace(b.out.built.config, w.devices, b.out.built.buildOptions());
    CHECK(robot->chassis().deps().motorGroups[1]->presentCount() == 3);
    CHECK(w.devices.motorRequestCount(13) == 0);
}

TEST_CASE("G1 2b test 6m: a record that FAILS its port check is refused, and never falls back to a "
          "usable table") {
    SUBCASE("the IMU unplugged") {
        FakeBlockSource card{encoded(robotTwoRecord())};
        FakeDeviceRegistry reg = tableCensus();  // a census the usable table WOULD pass …
        reg.setKind(2, DeviceKind::None);        // … but for the IMU, which both need
        const Boot b = runBoot(card, reg, usableTable());
        CHECK(std::string_view{b.out.verdict.reason}
              == "REFUSE: the calibration was taken against an IMU on port 2 and that port now "
                 "reads EMPTY; run shulib Calibrate");
        REQUIRE(b.lines.size() == 2);  // no table line (it is valid), no warning, no cross-check
        CHECK(b.lines[0]
              == std::string{"calibration record: DECODED, port check REFUSE -- "}
                     + b.out.verdict.reason);
        CHECK(b.lines[1]
              == "boot source: REFUSE -- the calibration record does not match this robot's ports "
                 "-- it was rewired, or the card came from another robot; run shulib Calibrate");
        CHECK_FALSE(b.out.ok);
        CHECK(b.out.source == BootSource::Refuse);
        CHECK(std::string_view{b.out.headline} == "NO USABLE DRIVETRAIN DESCRIPTION");
        // "%.80s | %.100s": the record's why is EMPTY once it decoded, so the refusal's reason is
        // " | " and the verdict's first 100 characters (the whole sentence is the line above).
        CHECK(std::string_view{b.out.why}
              == " | " + std::string{b.out.verdict.reason}.substr(0, 100));
        CHECK(b.out.built.config.drivetrain.tank.description.left.count == 0);  // nothing built
    }
    SUBCASE("a port that now carries a DIFFERENT device (the worksheet's F.0 row)") {
        FakeBlockSource card{encoded(robotTwoRecord())};
        FakeDeviceRegistry reg = robotTwoCensus();
        reg.setKind(13, DeviceKind::Imu);
        const Boot b = runBoot(card, reg, usableTable());
        const std::string reason{b.out.verdict.reason};
        CHECK(reason
              == "REFUSE: port(s) 13(IMU) carry a DIFFERENT device than the calibration recorded "
                 "-- this robot was rewired; run shulib Calibrate");
        CHECK_FALSE(b.out.ok);
        CHECK(b.out.source == BootSource::Refuse);
        // The screen's reason stops at 100 characters of it — "... -- this robot was rew" — and
        // the whole sentence is only on the `calibration record: DECODED, port check REFUSE`
        // line (R3a-BENCH-WORKSHEET F.0 says so).
        CHECK(std::string_view{b.out.why} == " | " + reason.substr(0, 100));
        CHECK(std::string_view{b.out.why}.ends_with("this robot was rew"));
    }
}

TEST_CASE("G1 2b test 6n: no card and a usable table -- the TABLE is chosen, re-verified, built from, "
          "and no cross-check runs") {
    FakeBlockSource card;
    card.setOpen(false);
    const Boot b = runBoot(card, tableCensus(), usableTable());
    CHECK(card.readCalls() == 0);  // a card that did not open is never read
    REQUIRE(b.lines.size() == 3);
    CHECK(b.lines[0] == "calibration record: NOT USED -- " + std::string{kNoCardWhy});
    CHECK(b.lines[1] == kSourceTableLine);
    CHECK(b.lines[2]
          == "WARNING: driving from the TYPED TABLE -- unverified since the rebuild; run shulib "
             "Calibrate (slot 4). Port check OK -- OK: all 10 recorded drive ports carry motors, "
             "IMU on port 2. NEW motors not in the calibration: none. NOT CHECKED: two drive cables "
             "swapped WITH EACH OTHER look identical here -- the first drive catches that");
    REQUIRE(b.out.ok);
    CHECK(b.out.source == BootSource::UseTable);
    CHECK(b.out.verdict.portCount == 10);  // the TABLE's port check, not a stale record's
    CHECK(b.out.crossCheckDiffering == 0);
    requireSameDescription(b.out.built.config.drivetrain.tank.description, usableTable());
    CHECK(b.out.built.config.drivetrain.tank.description.source == DescriptionSource::TypedTable);
}

TEST_CASE("G1 2b test 6o: no card and a table that cannot describe a drivetrain -- refused, naming "
          "BOTH reasons") {
    FakeBlockSource card;
    card.setOpen(false);
    const Boot b = runBoot(card, robotTwoCensus(), robotTwoTableSigns());
    REQUIRE(b.lines.size() == 3);
    CHECK(b.lines[0] == "calibration record: NOT USED -- " + std::string{kNoCardWhy});
    CHECK(b.lines[1] == kTankTableLine);
    CHECK(b.lines[2]
          == "boot source: REFUSE -- no calibration record on the card, and the typed table "
             "cannot describe a drivetrain either; run shulib Calibrate");
    CHECK_FALSE(b.out.ok);
    CHECK(std::string_view{b.out.headline} == "NO USABLE DRIVETRAIN DESCRIPTION");
    CHECK(std::string_view{b.out.why}
          == std::string{kNoCardWhy.substr(0, 80)} + " | " + std::string{kTableWhy});
}

TEST_CASE("G1 2b test 6p: an OPEN but EMPTY record file says so (ruling G1-2b-R7) and decides as "
          "no record") {
    FakeBlockSource card;  // open, zero bytes
    const Boot b = runBoot(card, tableCensus(), usableTable());
    CHECK(card.readCalls() == 1);
    REQUIRE(b.lines.size() == 3);
    CHECK(b.lines[0]
          == "calibration record: NOT USED -- the calibration record /usd/shulib_drivetrain.bin is "
             "EMPTY (0 bytes read) -- run shulib Calibrate");
    CHECK(b.lines[1] == kSourceTableLine);
    REQUIRE(b.out.ok);
    CHECK(b.out.source == BootSource::UseTable);
}

TEST_CASE("G1 2b test 6q: a TRUNCATED record is refused by the decoder, by name, and the boot falls "
          "back to a usable table") {
    FakeBlockSource card{encoded(robotTwoRecord())};
    card.setShortRead(100);  // a card yanked mid-write
    const Boot b = runBoot(card, tableCensus(), usableTable());
    REQUIRE(b.lines.size() == 3);
    CHECK(b.lines[0].starts_with("calibration record: NOT USED -- record is 100 bytes"));
    CHECK(b.lines[1] == kSourceTableLine);
    REQUIRE(b.out.ok);
    CHECK(b.out.source == BootSource::UseTable);
    requireSameDescription(b.out.built.config.drivetrain.tank.description, usableTable());
}

TEST_CASE("G1 2b test 6r: a usable table that FAILS its own port check is refused, with the "
          "warning printed first") {
    FakeBlockSource card;
    card.setOpen(false);
    FakeDeviceRegistry reg = tableCensus();
    reg.setKind(19, DeviceKind::Distance);  // the table's port 19 now carries a sensor
    const Boot b = runBoot(card, reg, usableTable());
    REQUIRE(b.lines.size() == 3);
    CHECK(b.lines[1] == kSourceTableLine);
    CHECK(b.lines[2]
          == std::string{"WARNING: driving from the TYPED TABLE -- unverified since the rebuild; "
                         "run shulib Calibrate (slot 4). Port check REFUSE -- "}
                 + b.out.verdict.reason);
    CHECK(std::string_view{b.out.verdict.reason}.starts_with("REFUSE: port(s) 19(DISTANCE)"));
    CHECK_FALSE(b.out.ok);
    CHECK(b.out.source == BootSource::UseTable);  // the table WAS chosen; its check refused it
    CHECK(std::string_view{b.out.headline} == "the TYPED TABLE does not match the ports");
    CHECK(std::string_view{b.out.why} == std::string{b.out.verdict.reason}.substr(0, 191));
}

// ═══ 6s-6v: THE MECHANISMS INTO THE CONFIG (T3 Part 2) ═══════════════════════════════════════
// src/common/boot_source.hpp's pure core converts src/robots/mechanism_table.hpp — the REAL table, robot two's
// kRobotTwoMechanisms, defined in every build — into config::MechanismConfig, field by field, in
// table order; the port check names the declared motor ports as claw pivots; and a claw port that
// the chosen description lists as a drive port refuses the boot. NO HARDWARE CLAIM: nobody has
// seen a motor on port 6 or 7 (A4 register HA-153), and the census below is a FakeDeviceRegistry.
namespace {

using shulib::bench::Cartridge;
using shulib::bench::kRobotTwoMechanisms;
using shulib::bench::MechanismTable;
using shulib::bench::PneumaticPosition;

/// Robot two's census with the claws plugged in: the 2026-09-25 census plus MOTOR on 6 and 7.
FakeDeviceRegistry robotTwoCensusWithClaws() {
    FakeDeviceRegistry reg = robotTwoCensus();
    reg.setKind(6, DeviceKind::Motor);
    reg.setKind(7, DeviceKind::Motor);
    return reg;
}

/// Every field of a converted MOTOR mechanism, longhand — including the fields a motor mechanism
/// must leave UNSET (its lines and levels) and the unused tail of its port array.
void requireMotorMechanism(const cfgns::MechanismConfig& m, std::string_view name, int signedPort,
                           MotorCartridge cartridge, cfgns::SafeBrakeMode safe) {
    CHECK(std::string_view{m.name} == name);
    CHECK(static_cast<int>(m.motorPorts[0]) == signedPort);
    for (std::size_t i = 1; i < cfgns::kMaxMotorsPerMechanism; ++i) {
        CHECK(m.motorPorts[i] == 0);
    }
    CHECK(static_cast<int>(m.motorCount) == 1);
    CHECK(m.cartridge == cartridge);
    CHECK(m.safeMode == safe);
    CHECK(static_cast<int>(m.pneumaticCount) == 0);
    CHECK(m.pneumaticPorts[0] == 0);
    CHECK(m.pneumaticPorts[1] == 0);
    CHECK(static_cast<int>(m.pneumaticSafe) == 0);   // UNSET: not a pneumatic mechanism
    CHECK(static_cast<int>(m.pneumaticStart) == 0);
}

/// Every field of a converted PNEUMATIC mechanism, longhand.
void requirePneumaticMechanism(const cfgns::MechanismConfig& m, std::string_view name, int line,
                               cfgns::PneumaticLevel start, cfgns::PneumaticLevel safe) {
    CHECK(std::string_view{m.name} == name);
    CHECK(static_cast<int>(m.pneumaticPorts[0]) == line);
    CHECK(m.pneumaticPorts[1] == 0);
    CHECK(static_cast<int>(m.pneumaticCount) == 1);
    CHECK(m.pneumaticStart == start);
    CHECK(m.pneumaticSafe == safe);
    CHECK(static_cast<int>(m.motorCount) == 0);
    CHECK(m.motorPorts[0] == 0);
    CHECK(static_cast<int>(m.cartridge) == 0);  // UNSET: not a motor mechanism
    CHECK(static_cast<int>(m.safeMode) == 0);
}

/// A table whose every field is DISTINGUISHABLE — the fields robot two's table cannot tell apart
/// (its two pivots share a cartridge and a mode; its claw's start and safe are both CLOSED, and its
/// polarity makes CLOSED the LOW level, which is also what a CLOSED position read as a bool gives).
constexpr MechanismTable kDistinguishable = {
    .motors = {{.name = "lift",
                .signedPort = -9,
                .cartridge = Cartridge::Green,
                .safeMode = shulib::hal::BrakeMode::Brake,
                .forward = shulib::hal::ControllerButton::Up,
                .reverse = shulib::hal::ControllerButton::Down,
                .volts = 8.0,
                .provenance = "test"},
               {.name = "intake",
                .signedPort = 10,
                .cartridge = Cartridge::Blue,
                .safeMode = shulib::hal::BrakeMode::Coast,
                .forward = shulib::hal::ControllerButton::Left,
                .reverse = shulib::hal::ControllerButton::Right,
                .volts = 6.0,
                .provenance = "test"}},
    .motorCount = 2,
    .pneumatics = {{.name = "clamp",
                    .adiLine = 3,
                    .openCommand = false,  // LOW opens: CLOSED is HIGH
                    .start = PneumaticPosition::Closed,
                    .safe = PneumaticPosition::Open,
                    .toggle = shulib::hal::ControllerButton::X,
                    .provenance = "test"},
                   {.name = "wing",
                    .adiLine = 8,
                    .openCommand = true,
                    .start = PneumaticPosition::Open,
                    .safe = PneumaticPosition::Closed,
                    .toggle = shulib::hal::ControllerButton::B,
                    .provenance = "test"}},
    .pneumaticCount = 2,
};

}  // namespace

TEST_CASE("T3 P2 test 6s: the pure core converts robot two's REAL mechanism table into the config "
          "exactly — the pivots as single-motor mechanisms, the claw as a pneumatic one whose START "
          "and SAFE are LEVELS, in table order — and every field is converted, nothing defaulted") {
    // BUG CAUGHT: a pivot converted with Coast (the value-initialised mode: it drops a loaded claw),
    // the drive's cartridge, a lost sign; the claw's POSITION handed over as its level, or its
    // SAFE level as its START; the two pivots in one mechanism; the order of the kinds swapped (the
    // bindings bind by config index); a mechanism dropped or a name cut.
    const DrivetrainDescription record = robotTwoRecord();
    const BootVerdict v = verifyAgainstRegistry(record, robotTwoCensusWithClaws());
    REQUIRE(v.usable());
    const BootConfig boot = bootConfigFrom(record, v, kRobotTwoName, kRobotTwoMechanisms);
    const cfgns::RobotConfig& c = boot.config;
    char why[192] = "";
    REQUIRE_MESSAGE(c.valid(why, sizeof why), why);
    REQUIRE(static_cast<int>(c.mechanismCount) == 3);
    requireMotorMechanism(c.mechanisms[0], "claw pivot 6", +6, MotorCartridge::Red,
                          cfgns::SafeBrakeMode::Hold);
    requireMotorMechanism(c.mechanisms[1], "claw pivot 7", +7, MotorCartridge::Red,
                          cfgns::SafeBrakeMode::Hold);
    // The claw: 3-wire A; CLOSED at the start and CLOSED when forced safe — as LEVELS through the
    // one polarity constant (kClawOpenCommand: the level that OPENS it, UNVERIFIED — HA-159).
    const cfgns::PneumaticLevel closed = shulib::bench::kClawOpenCommand
                                             ? cfgns::PneumaticLevel::Low
                                             : cfgns::PneumaticLevel::High;
    requirePneumaticMechanism(c.mechanisms[2], "claw", 1, closed, closed);
    CHECK(c.mechanisms[2].pneumaticStart
          == shulib::bench::toPneumaticLevel(
              shulib::bench::pneumaticStartCommand(kRobotTwoMechanisms.pneumatics[0])));
    CHECK(shulib::bench::mechanismConfigIndexOfPneumatic(kRobotTwoMechanisms, 0) == 2U);
    // The drivetrain is untouched by the mechanisms (6b's pins, again, with them in).
    requireSameDescription(c.drivetrain.tank.description, record);
    requireDefaultTuning(c.tuning);

    SUBCASE("a table whose every field differs: each lands in its own place") {
        const BootConfig b = bootConfigFrom(record, v, kRobotTwoName, kDistinguishable);
        const cfgns::RobotConfig& d = b.config;
        REQUIRE_MESSAGE(d.valid(why, sizeof why), why);
        REQUIRE(static_cast<int>(d.mechanismCount) == 4);
        requireMotorMechanism(d.mechanisms[0], "lift", -9, MotorCartridge::Green,
                              cfgns::SafeBrakeMode::Brake);
        requireMotorMechanism(d.mechanisms[1], "intake", 10, MotorCartridge::Blue,
                              cfgns::SafeBrakeMode::Coast);
        // clamp: LOW opens, so CLOSED (start) is HIGH and OPEN (safe) is LOW.
        requirePneumaticMechanism(d.mechanisms[2], "clamp", 3, cfgns::PneumaticLevel::High,
                                  cfgns::PneumaticLevel::Low);
        // wing: HIGH opens, so OPEN (start) is HIGH and CLOSED (safe) is LOW.
        requirePneumaticMechanism(d.mechanisms[3], "wing", 8, cfgns::PneumaticLevel::High,
                                  cfgns::PneumaticLevel::Low);
        CHECK(shulib::bench::mechanismConfigIndexOfPneumatic(kDistinguishable, 1) == 3U);
    }
    SUBCASE("no mechanisms: none, and the config stays what G1 Part 2b built") {
        const BootConfig none = bootConfigFrom(record, v, kRobotTwoName, kNoMechanisms);
        CHECK(static_cast<int>(none.config.mechanismCount) == 0);
    }
}

TEST_CASE("T3 P2 test 6t: a table that does not convert WHOLE is named by mechanismTableProblem() "
          "— a name one character too long, more mechanisms than a RobotConfig holds, an unset "
          "field — and the boot refuses by that sentence; bootConfigFrom() will not build from it") {
    // BUG CAUGHT: a name cut to fit (the Robot's lookups, the boot lines and the bindings would name
    // another mechanism than the table); a ninth mechanism silently dropped; an unset optional read.
    CHECK(shulib::bench::mechanismTableProblem(kRobotTwoMechanisms) == nullptr);
    CHECK(shulib::bench::mechanismTableProblem(kNoMechanisms) == nullptr);
    CHECK(shulib::bench::mechanismTableProblem(kDistinguishable) == nullptr);

    MechanismTable longName = kRobotTwoMechanisms;
    longName.motors[0].name = "abcdefghijklmnopqrstuvw";  // 23: fits with its NUL
    longName.pneumatics[0].name = "ABCDEFGHIJKLMNOPQRSTUVW";
    CHECK(shulib::bench::mechanismTableProblem(longName) == nullptr);
    {
        // …and is carried WHOLE, both kinds — copyMechanismName()'s bound at its edge (review
        // round 1, finding 5: only mechanismTableProblem() was pinned at 23/24).
        const BootVerdict fits = verifyAgainstRegistry(robotTwoRecord(), robotTwoCensus());
        const BootConfig b = bootConfigFrom(robotTwoRecord(), fits, kRobotTwoName, longName);
        CHECK(std::string_view{b.config.mechanisms[0].name} == "abcdefghijklmnopqrstuvw");
        CHECK(std::string_view{b.config.mechanisms[2].name} == "ABCDEFGHIJKLMNOPQRSTUVW");
    }
    longName.pneumatics[0].name = kRobotTwoMechanisms.pneumatics[0].name;
    longName.motors[0].name = "abcdefghijklmnopqrstuvwx";  // 24: does not
    CHECK(std::string_view{shulib::bench::mechanismTableProblem(longName)}
          == "a mechanism name is longer than 23 characters (a RobotConfig name field)");
    MechanismTable longClaw = kRobotTwoMechanisms;
    longClaw.pneumatics[0].name = "abcdefghijklmnopqrstuvwx";
    CHECK(shulib::bench::mechanismTableProblem(longClaw) != nullptr);

    MechanismTable nine = kDistinguishable;  // 2 motors + 7 pneumatics = 9 > 8
    for (std::size_t j = 2; j < 7; ++j) {
        nine.pneumatics[j] = kDistinguishable.pneumatics[0];
        nine.pneumatics[j].adiLine = static_cast<std::uint8_t>(j + 1);
    }
    nine.pneumaticCount = 7;
    CHECK(std::string_view{shulib::bench::mechanismTableProblem(nine)}
          == "the mechanism table declares more mechanisms than a RobotConfig holds (8)");
    MechanismTable eight = nine;
    eight.pneumaticCount = 6;  // 2 + 6 = 8: fits exactly
    CHECK(shulib::bench::mechanismTableProblem(eight) == nullptr);

    MechanismTable unset = kRobotTwoMechanisms;
    unset.pneumatics[0].openCommand = std::nullopt;
    CHECK(std::string_view{shulib::bench::mechanismTableProblem(unset)}
          == "a declared mechanism has an UNSET field (a value is never invented)");
    MechanismTable tooMany = kRobotTwoMechanisms;
    tooMany.motorCount = 7;
    CHECK(std::string_view{shulib::bench::mechanismTableProblem(tooMany)}
          == "the mechanism table declares more entries than its arrays hold");

    // The boot refuses by the sentence, printed as it is decided, AFTER the source lines.
    FakeBlockSource card{encoded(robotTwoRecord())};
    const Boot b = runBoot(card, robotTwoCensus(), robotTwoTableSigns(), longName);
    REQUIRE(b.lines.size() == 5U);
    CHECK(b.lines[4]
          == "mechanism table: REFUSED -- a mechanism name is longer than 23 characters (a "
             "RobotConfig name field) (src/robots/mechanism_table.hpp)");
    CHECK_FALSE(b.out.ok);
    CHECK(std::string_view{b.out.headline} == "the MECHANISM TABLE does not convert");
    CHECK(std::string_view{b.out.why}
          == "a mechanism name is longer than 23 characters (a RobotConfig name field)");
    const BootVerdict v = verifyAgainstRegistry(robotTwoRecord(), robotTwoCensus());
    CHECK_THROWS_AS((void)bootConfigFrom(robotTwoRecord(), v, kRobotTwoName, longName),
                    PreconditionError);
}

TEST_CASE("T3 P2 test 6u: the whole boot with robot two's claws plugged in — the port check names "
          "6 and 7 as CLAW PIVOTS, never as NEW motors, the record is chosen, and the config carries "
          "the three mechanisms") {
    // BUG CAUGHT: the boot line calling a declared claw an unknown motor (the T3 brief: "names
    // declared claw ports as claw pivots, not unknown motors"); the program not passing its table
    // to the port check, or to the config.
    FakeBlockSource card{encoded(robotTwoRecord())};
    const Boot b = runBoot(card, robotTwoCensusWithClaws(), robotTwoTableSigns(),
                           kRobotTwoMechanisms);
    REQUIRE(b.lines.size() == 4U);
    CHECK(b.lines[0]
          == "calibration record: DECODED, port check OK -- OK: all 8 recorded drive ports carry "
             "motors, IMU on port 2. NEW motors not in the calibration: 14; claw pivots (declared, "
             "not drive motors): 6 7. NOT CHECKED: two drive cables swapped WITH EACH OTHER look "
             "identical here -- the first drive catches that");
    REQUIRE(b.out.ok);
    CHECK(b.out.source == BootSource::UseRecord);
    CHECK(static_cast<int>(b.out.verdict.newCount) == 1);
    CHECK(static_cast<int>(b.out.verdict.declaredCount) == 2);
    CHECK(static_cast<int>(b.out.built.config.mechanismCount) == 3);
    // The claws unplugged: the same boot, and the port check says nothing about them (the Robot's
    // build reports them ABSENT — test/teleop_claws_test.cpp).
    FakeBlockSource again{encoded(robotTwoRecord())};
    const Boot unplugged = runBoot(again, robotTwoCensus(), robotTwoTableSigns(),
                                   kRobotTwoMechanisms);
    CHECK(unplugged.lines[0].find("NEW motors not in the calibration: 14. NOT CHECKED")
          != std::string::npos);
    CHECK(unplugged.out.ok);
    CHECK(static_cast<int>(unplugged.out.built.config.mechanismCount) == 3);
    // Without the program's table (a program that forgot it), 6 and 7 would be NEW motors — the
    // difference this case exists to pin.
    FakeBlockSource bare{encoded(robotTwoRecord())};
    const Boot noTable = runBoot(bare, robotTwoCensusWithClaws(), robotTwoTableSigns());
    CHECK(noTable.lines[0].find("NEW motors not in the calibration: 6 7 14. NOT CHECKED")
          != std::string::npos);
    // A pivot FLIPPED in the table (Station J's fix for a claw that moves the wrong way: +6 -> -6)
    // is still port 6 to the port check — the declared ports are MAGNITUDES.
    MechanismTable flipped = kRobotTwoMechanisms;
    flipped.motors[0].signedPort = -6;
    FakeBlockSource again2{encoded(robotTwoRecord())};
    const Boot flippedBoot = runBoot(again2, robotTwoCensusWithClaws(), robotTwoTableSigns(), flipped);
    CHECK(flippedBoot.lines[0].find("NEW motors not in the calibration: 14; claw pivots (declared, "
                                    "not drive motors): 6 7. NOT CHECKED")
          != std::string::npos);
    REQUIRE(flippedBoot.out.ok);
    CHECK(static_cast<int>(flippedBoot.out.built.config.mechanisms[0].motorPorts[0]) == -6);
}

TEST_CASE("T3 P2 test 6v: a claw port that the RECORD lists as a DRIVE port REFUSES the boot, by "
          "name — \"recalibrate\" — and nothing is built") {
    // BUG CAUGHT: a record taken before Calibrate skipped the claws (it took port 6 as a drive
    // motor) booting anyway — robot two would drive the claw as a wheel and bind R1 to a drive
    // motor; the refusal worded as a port collision that does not say what to do.
    DrivetrainDescription tookTheClaw = robotTwoRecord();
    tookTheClaw.right.signedPorts[3] = 6;  // RIGHT +11 -12 +13 +6 (a claw measured as a wheel)
    FakeBlockSource card{encoded(tookTheClaw)};
    FakeDeviceRegistry census = robotTwoCensusWithClaws();
    census.setKind(15, DeviceKind::None);
    const Boot b = runBoot(card, census, robotTwoTableSigns(), kRobotTwoMechanisms);
    REQUIRE_FALSE(b.lines.empty());
    CHECK(b.lines.back()
          == "mechanisms: REFUSED -- port 6 is both a claw (claw pivot 6) and a drive motor in the "
             "record -- recalibrate: shulib Calibrate (slot 4) never takes a declared claw port");
    CHECK_FALSE(b.out.ok);
    CHECK(b.out.source == BootSource::UseRecord);  // the record WAS chosen; the claws refused it
    CHECK(std::string_view{b.out.headline} == "a CLAW port is a DRIVE port in the record");
    CHECK(std::string_view{b.out.why}
          == "port 6 is both a claw (claw pivot 6) and a drive motor in the record -- recalibrate: "
             "shulib Calibrate (slot 4) never takes a declared claw port");
    CHECK(static_cast<int>(b.out.built.config.mechanismCount) == 0);  // nothing was built
    CHECK(b.out.built.config.drivetrain.tank.description.left.count == 0);
    // The same with the pivot FLIPPED in the table (-6): the collision is on the port's MAGNITUDE.
    MechanismTable flipped = kRobotTwoMechanisms;
    flipped.motors[0].signedPort = -6;
    FakeBlockSource card2{encoded(tookTheClaw)};
    const Boot f = runBoot(card2, census, robotTwoTableSigns(), flipped);
    CHECK_FALSE(f.out.ok);
    CHECK(std::string_view{f.out.headline} == "a CLAW port is a DRIVE port in the record");
    CHECK(std::string_view{f.out.why}.starts_with("port 6 is both a claw (claw pivot 6)"));
    // The typed table says the same about ITSELF (src/robots/chassis_table.hpp and the mechanism table
    // disagree) — a build's pair cannot, by the mechanism table's check 8; a hand-built one can.
    FakeBlockSource none;
    none.setOpen(false);
    DrivetrainDescription table = usableTable();
    table.left.signedPorts[1] = 7;  // the table's LEFT 12 retyped as 7
    FakeDeviceRegistry tableCensusWithClaws = tableCensus();
    tableCensusWithClaws.setKind(7, DeviceKind::Motor);
    tableCensusWithClaws.setKind(6, DeviceKind::Motor);
    const Boot t = runBoot(none, tableCensusWithClaws, table, kRobotTwoMechanisms);
    CHECK_FALSE(t.out.ok);
    CHECK(std::string_view{t.out.headline} == "a CLAW port is a DRIVE port in the typed table");
    CHECK(std::string_view{t.out.why}
          == "port 7 is both a claw (claw pivot 7) and a drive motor in the typed table -- "
             "src/robots/chassis_table.hpp and src/robots/mechanism_table.hpp disagree");
}

TEST_CASE("T3 P2 test 6w: a boot with NO record falls back to the TYPED TABLE — and its port check "
          "names 6 and 7 as CLAW PIVOTS too, never as NEW motors") {
    // BUG CAUGHT (review round 1, finding 2): the declared ports passed on the RECORD's port check
    // only. decideBootSourceFrom() calls verifyAgainstRegistry() in TWO places — the record's check
    // and the typed table's re-check — and 6u reaches only the first; 6v's typed-table part ends in
    // the collision refusal and never reads the check's words. Without this case the table branch
    // could drop `declared` and a robot with no SD card would print `NEW motors not in the
    // calibration: 6 7` for its own claws — what the Part 2 brief §1 item 1 forbids.
    FakeBlockSource none;
    none.setOpen(false);
    FakeDeviceRegistry census = tableCensus();
    census.setKind(6, DeviceKind::Motor);
    census.setKind(7, DeviceKind::Motor);
    const Boot b = runBoot(none, census, usableTable(), kRobotTwoMechanisms);
    REQUIRE(b.lines.size() == 3U);
    CHECK(b.lines[0] == "calibration record: NOT USED -- " + std::string{kNoCardWhy});
    CHECK(b.lines[1] == kSourceTableLine);
    CHECK(b.lines[2]
          == "WARNING: driving from the TYPED TABLE -- unverified since the rebuild; run shulib "
             "Calibrate (slot 4). Port check OK -- OK: all 10 recorded drive ports carry motors, "
             "IMU on port 2. NEW motors not in the calibration: none; claw pivots (declared, not "
             "drive motors): 6 7. NOT CHECKED: two drive cables swapped WITH EACH OTHER look "
             "identical here -- the first drive catches that");
    REQUIRE(b.out.ok);
    CHECK(b.out.source == BootSource::UseTable);
    CHECK(static_cast<int>(b.out.verdict.newCount) == 0);
    CHECK(static_cast<int>(b.out.verdict.declaredCount) == 2);
    CHECK(static_cast<int>(b.out.built.config.mechanismCount) == 3);
    // The same boot WITHOUT the program's table: 6 and 7 are NEW — the difference the case pins.
    FakeBlockSource none2;
    none2.setOpen(false);
    const Boot bare = runBoot(none2, census, usableTable());
    REQUIRE(bare.lines.size() == 3U);
    CHECK(bare.lines[2].find("NEW motors not in the calibration: 6 7. NOT CHECKED")
          != std::string::npos);
}
