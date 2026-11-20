// calibrate_coupling_test.cpp — CAL2: Calibrate finds the drive's sides by COUPLING, and a steady
// DONE (the CAL2 brief, §3). Robot one's drive on geared_tank_rig.hpp: two gear trains with
// backlash on a chassis one motor cannot move. Every number is the host test plant's.

#include <cmath>
#include <cstring>
#include <string>

#include "calibration_test_rig.hpp"
#include "doctest.h"
#include "geared_tank_rig.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/core/check.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/sim/scenario.hpp"

namespace {

using geared::GearedTank;
using geared::kDeg;
using geared::TankSpec;
using shulib::calibration::CalibratorConfig;
using shulib::calibration::DrivetrainCalibrator;
using shulib::calibration::Phase;
using shulib::calibration::PulseVerdict;
using shulib::calibration::Refusal;
using shulib::calibration::refusalName;

/// Coupling switched off: the yaw alone decides every port — the calibrator before CAL2.
CalibratorConfig yawOnly(CalibratorConfig c = {}) {
    c.followThreshold = shulib::units::AngleDim{360.0 * kDeg};
    return c;
}

std::string why(const DrivetrainCalibrator& cal) {
    return std::string{refusalName(cal.refusal())} + ": " + cal.reason();
}

void requireDone(const DrivetrainCalibrator& cal) { REQUIRE_MESSAGE(cal.phase() == Phase::Done, why(cal)); }

void requireStopped(const DrivetrainCalibrator& cal, Refusal code, int port) {
    REQUIRE(cal.phase() == Phase::Aborted);
    CHECK_MESSAGE(cal.refusal() == code, why(cal));
    CHECK_MESSAGE(cal.refusalPort() == port, why(cal));
}

/// A whole run that also says whether the identification turn ever started.
Phase runWatchingTurn(GearedTank& t, DrivetrainCalibrator& cal, bool& turned) {
    turned = false;
    return geared::run(t, cal, [&turned](GearedTank&, DrivetrainCalibrator& c) { turned = turned || c.status().identifying; });
}

/// While port `p` (candidate index) is pulsed, member `j` reads `factor` times the train's travel
/// instead of its own (0 = it does not see the train; -1 = it reads backward).
std::function<double(const GearedTank&, std::size_t)> seeAs(std::size_t p, std::size_t j, double factor) {
    return [p, j, factor](const GearedTank& t, std::size_t k) {
        return k == j && t.pulsed() == p ? (factor - 1.0) * t.travel(k) : 0.0;
    };
}

}  // namespace

// ═══ C1. Robot one: two groups, the right sides, few pulses ═════════════════════════
// Bug caught: sides read off the yaw on a robot that does not turn (robot one, 2026-11-17).
TEST_CASE("CAL2 C1: robot one's geared, heavy drive -- the yaw alone stops PULSE_STILL; coupling "
          "finds two groups, the right sides and the width, never past 6 V") {
    for (const bool robotOne : {false, true}) {
        CAPTURE(robotOne);
        const CalibratorConfig base = robotOne ? geared::robotOneConfig() : CalibratorConfig{};
        {
            GearedTank t{TankSpec{}};
            DrivetrainCalibrator old{t.candidates(), t.portSpan(), t.imu, t.clock, yawOnly(base)};
            (void)geared::run(t, old);
            requireStopped(old, Refusal::PulseStill, 11);
            CHECK(old.report().pulses[old.report().pulseCount - 1].levelVolts == 12.0);
            CHECK(std::strstr(old.reason(), "at most 4.0 deg") != nullptr);  // D5: what was measured
        }
        GearedTank t{TankSpec{}};
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, base};
        (void)geared::run(t, cal);
        requireDone(cal);
        const auto& rep = cal.report();
        CHECK(rep.groupCount == 2);
        CHECK(rep.pulseCount == 12);  // 11 and 16 at 3 and 6 V; every mate once, at 6 V
        for (std::size_t k = 0; k < rep.pulseCount; ++k) {
            CHECK(rep.pulses[k].levelVolts <= 6.0);
        }
        CHECK(rep.groupSide[0] == -1);
        CHECK(rep.groupSide[1] == 1);
        CHECK(rep.identifyYawRad < 0.0);
        const auto& d = cal.result();
        CHECK(d.left.count == 5);
        CHECK(d.right.count == 5);
        for (int p = 11; p <= 20; ++p) {
            CAPTURE(p);
            CHECK(d.sideOf(p) == (p <= 15 ? -1 : 1));
            CHECK(d.signedPortOf(p) == ((p - 11) % 5 % 2 == 1 ? -p : p));  // + - + - + per side
        }
        CHECK(d.trackWidth.value() == doctest::Approx(geared::kTrack).epsilon(shulib::calibration::kReproWidthTolerance));
    }
}

// ═══ C2. The first level the backlash lets through ══════════════════════════════════
TEST_CASE("CAL2 C2: backlash wider than the 3 V pulse -- no follower at 3 V, four at 6 V, and the "
          "ramp stops there; the serial line names them") {
    GearedTank t{TankSpec{}};
    DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock};
    char line[160];
    std::string first;
    std::string second;
    (void)geared::run(t, cal, [&](GearedTank&, DrivetrainCalibrator& c) {
        const auto& r = c.report();
        if (r.pulseCount == 1 && first.empty()) {
            shulib::calibration::formatFollowerRecord(r.pulses[0], r, true, line, sizeof line);
            first = line;
        }
        if (r.pulseCount == 2 && second.empty()) {
            shulib::calibration::formatFollowerRecord(r.pulses[1], r, true, line, sizeof line);
            second = line;
        }
    });
    requireDone(cal);
    const auto& p = cal.report().pulses;
    CHECK(p[0].levelVolts == 3.0);
    CHECK(p[0].verdict == PulseVerdict::Weak);
    CHECK(p[0].followers == 0u);
    CHECK(p[0].ownRad == doctest::Approx(10.0 * kDeg));  // its own gap, nothing more
    CHECK(p[1].levelVolts == 6.0);
    CHECK(p[1].verdict == PulseVerdict::Followed);
    CHECK(p[1].followers == 0b11110u);
    CHECK(p[2].port == 12);  // a mate confirms it drives its train, from its group's level
    CHECK(p[2].levelVolts == 6.0);
    CHECK(p[2].followers == 0b11101u);
    CHECK(first == "pulse port 11 3.0 V no followers");
    CHECK(second == "pulse port 11 6.0 V followers 12 13 14 15 (+4 +4 +4 +4 deg)");
    shulib::calibration::formatFollowerRecord(p[1], cal.report(), false, line, sizeof line);
    CHECK(std::string{line} == "pulse port 11 6.0 V followers 12 13 14 15");
    shulib::calibration::formatGroupsRecord(cal.report(), line, sizeof line);
    CHECK(std::string{line} == "groups: A 11 12 13 14 15 | B 16 17 18 19 20 | by yaw: none");
    shulib::calibration::formatIdentifyRecord(cal.report(), line, sizeof line);
    CHECK(std::string{line}.find("A is LEFT, B is RIGHT") != std::string::npos);
}

// ═══ C3. A drive not geared together runs as it did ═════════════════════════════════
// Bug caught: coupling read from a robot that MOVED — its other side rolls along, and every port
// would "follow" (the plant's κ; test 25 calibrates at κ = 0.6 today).
TEST_CASE("CAL2 C3: where no port follows, or the whole robot moved, the yaw decides exactly as "
          "before -- the same pulses, the same description") {
    SUBCASE("one motor per side") {
        auto runOne = [](const CalibratorConfig& c, shulib::calibration::CalibrationReport& rep,
                         shulib::hal::DrivetrainDescription& out) {
            shulib::kinematics::TankKinematics kin{shulib::units::Length{calib_rig::kTrackWidth}};
            shulib::sim::SimHarness h{kin, calib_rig::calibPlant(0.0)};
            shulib::hal::IMotor* m[2] = {&h.motor(0), &h.motor(1)};
            const std::int8_t ports[2] = {11, 16};
            DrivetrainCalibrator cal{std::span<shulib::hal::IMotor* const>{m, 2},
                                     std::span<const std::int8_t>{ports, 2}, h.imu(), h.clock(), c};
            cal.start(calib_rig::facts());
            double pushed = 0.0;
            for (int k = 0; k < 40000 && cal.phase() != Phase::Done && cal.phase() != Phase::Aborted; ++k) {
                if (cal.phase() == Phase::PushCapture) {
                    const double v = pushed < 48.0 ? 2.2 : 0.0;
                    h.motor(0).setVoltage(shulib::units::Voltage{v});
                    h.motor(1).setVoltage(shulib::units::Voltage{v});
                    if (pushed >= 48.0 && cal.pushReady()) {
                        cal.answer(shulib::calibration::Answer::Done);
                    }
                }
                cal.tick();
                h.plant().step(shulib::units::Time{calib_rig::kDt});
                pushed += cal.phase() == Phase::PushCapture ? std::abs(h.trueBodyTwist().vx().value()) * calib_rig::kDt : 0.0;
            }
            requireDone(cal);
            rep = cal.report();
            out = cal.result();
        };
        shulib::calibration::CalibrationReport a{};
        shulib::calibration::CalibrationReport b{};
        shulib::hal::DrivetrainDescription da{};
        shulib::hal::DrivetrainDescription db{};
        runOne(CalibratorConfig{}, a, da);
        runOne(yawOnly(), b, db);
        CHECK(a.groupCount == 0);
        REQUIRE(a.pulseCount == b.pulseCount);
        for (std::size_t k = 0; k < a.pulseCount; ++k) {
            CHECK(a.pulses[k].verdict == b.pulses[k].verdict);
            CHECK(a.pulses[k].yawRad == b.pulses[k].yawRad);
        }
        CHECK(da.sideOf(11) == -1);
        CHECK(da.sideOf(16) == 1);
        CHECK(da.trackWidth.value() == db.trackWidth.value());
    }
    SUBCASE("robot two's old drive at k = 0.6: the other side rolls along") {
        calib_rig::CalibRig a{CalibratorConfig{}, calib_rig::calibPlant(0.6)};
        calib_rig::CalibRig b{yawOnly(), calib_rig::calibPlant(0.6)};
        a.cal.start(calib_rig::facts());
        b.cal.start(calib_rig::facts());
        (void)calib_rig::runCalibration(a, calib_rig::RunOptions{});
        (void)calib_rig::runCalibration(b, calib_rig::RunOptions{});
        requireDone(a.cal);
        requireDone(b.cal);
        CHECK(a.cal.report().groupCount == 0);
        REQUIRE(a.cal.report().pulseCount == b.cal.report().pulseCount);
        int moved = 0;
        for (std::size_t k = 0; k < a.cal.report().pulseCount; ++k) {
            const auto& r = a.cal.report().pulses[k];
            CHECK(r.verdict == b.cal.report().pulses[k].verdict);
            if (r.followers != 0u) {  // D5: the record says who turned with it, the robot having moved
                ++moved;
                char line[160];
                shulib::calibration::formatFollowerRecord(r, a.cal.report(), false, line, sizeof line);
                CHECK(std::string{line}.find(" V turned with it (no group): ") != std::string::npos);
            }
        }
        CHECK(moved >= 4);
        CHECK(a.cal.result().trackWidth.value() == b.cal.result().trackWidth.value());
    }
}

// ═══ C4. The group union, and the refusals by name ═══════════════════════════════════
TEST_CASE("CAL2 C4: a port that saw no train joins the group its own pulse moves; both groups, a "
          "third group and a reversed follower are refused naming the port") {
    SUBCASE("port 20 misses port 16's pulse, then joins B through port 17's") {
        GearedTank t{TankSpec{}};
        t.bias = seeAs(5, 9, 0.0);
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock};
        (void)geared::run(t, cal);
        requireDone(cal);
        CHECK(cal.report().groupCount == 2);
        CHECK(cal.report().ports[9].group == 2);
        CHECK(cal.result().sideOf(20) == 1);
    }
    SUBCASE("port 20's pulse also moves port 12 (group A): both groups") {
        GearedTank t{TankSpec{}};
        const auto miss = seeAs(5, 9, 0.0);
        t.bias = [miss](const GearedTank& g, std::size_t k) {
            return k == 1 && g.pulsed() == 9 && g.travel(6) > kDeg ? 3.0 * kDeg : miss(g, k);
        };
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock};
        (void)geared::run(t, cal);
        requireStopped(cal, Refusal::SideConflict, 20);
        CHECK(std::strstr(cal.reason(), "BOTH groups") != nullptr);
        CHECK(calib_rig::screenSafe(cal.reason()));
    }
    SUBCASE("a third group") {
        GearedTank t{TankSpec{}};
        t.bias = [](const GearedTank& g, std::size_t k) {
            const std::size_t p = g.pulsed();
            const bool hide = (p < 3 && (k == 3 || k == 4)) || ((p == 3 || p == 4) && k < 3);
            return hide ? -g.travel(k) : 0.0;
        };
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock};
        (void)geared::run(t, cal);
        requireStopped(cal, Refusal::SideConflict, 16);
        CHECK(std::strstr(cal.reason(), "THIRD group") != nullptr);
    }
    SUBCASE("followers against their sign: the first is named") {
        GearedTank t{TankSpec{}};
        const auto a = seeAs(0, 2, -1.0);
        const auto b = seeAs(0, 3, -1.0);
        t.bias = [a, b](const GearedTank& g, std::size_t k) { return a(g, k) + b(g, k); };
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock};
        (void)geared::run(t, cal);
        requireStopped(cal, Refusal::FollowerReversed, 13);
        CHECK(std::strstr(cal.reason(), "AGAINST its captured sign") != nullptr);
    }
    SUBCASE("the only port that turned went against its sign") {
        GearedTank t{TankSpec{}};
        t.bias = [](const GearedTank& g, std::size_t k) {  // 12 reads backward, 13-15 see nothing
            return g.pulsed() == 0 && k >= 1 && k <= 4 ? (k == 1 ? -2.0 : -1.0) * g.travel(k) : 0.0;
        };
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock};
        (void)geared::run(t, cal);
        requireStopped(cal, Refusal::FollowerReversed, 12);
        CHECK(cal.report().pulseCount == 2);  // 3 V nothing, 6 V refused: no ramp to 12 V
    }
}

// ═══ C5. The spin names the groups, both ways ═══════════════════════════════════════
// Bug caught: a group's side taken from a pulse's yaw, which on robot one is frame flex — here
// flex that points the WRONG way past robot one's 0.6 deg floor.
TEST_CASE("CAL2 C5: the identification turn names A LEFT or A RIGHT whichever way the robot is "
          "wired, ignores a pulse's flex, and refuses a robot it cannot turn") {
    for (const bool rightFirst : {false, true}) {
        CAPTURE(rightFirst);
        TankSpec s;
        s.rightFirst = rightFirst;
        s.flexDegPerDeg = -0.2;  // 4 deg of wind-up: 0.8 deg of yaw, the WRONG way
        GearedTank t{s};
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, geared::robotOneConfig()};
        (void)geared::run(t, cal);
        requireDone(cal);
        CHECK(std::abs(cal.report().pulses[1].yawRad) >= cal.config().pulseYawFloor.value());
        CHECK(cal.report().groupSide[0] == (rightFirst ? 1 : -1));
        CHECK(cal.report().groupSide[1] == (rightFirst ? -1 : 1));
        CHECK((cal.report().identifyYawRad > 0.0) == rightFirst);
        CHECK(cal.result().sideOf(11) == (rightFirst ? 1 : -1));
        CHECK(cal.result().sideOf(20) == (rightFirst ? -1 : 1));
    }
    TankSpec pinned;
    pinned.chassisKs = 13.0;  // no level rolls the wheels
    GearedTank t{pinned};
    DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock};
    (void)geared::run(t, cal);
    requireStopped(cal, Refusal::SpinNoStart, 0);
    CHECK(std::strstr(cal.reason(), "group A forward") != nullptr);
    CHECK(cal.report().groupSide[0] == 0);
}

// ═══ C6. D4 — rest from the heading, not the jumpy instant rate ═════════════════════
// Bug caught: robot one's DONE flicker (2026-11-17): an instant rate of ~1 deg/s against a floor
// of 1.0 made READY flip every update, and every release landed on a not-ready tick.
TEST_CASE("CAL2 C6: an instant rate of +-1.2 deg/s on a steady heading keeps DONE ready and "
          "pressable on every tick; a real 2 deg/s turn never rests") {
    using namespace shulib::calibration;
    TankSpec s;
    s.rateNoiseDegS = 1.2;
    GearedTank t{s};
    DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, geared::robotOneConfig()};
    cal.start(geared::facts());
    TouchGate gate{0};
    std::int32_t releases = 0;
    double pushed = 0.0;
    int readyTicks = 0;
    int pressed = 0;
    for (int k = 0; k < 2000 && readyTicks < 200; ++k) {
        double push = 0.0;
        if (cal.phase() == Phase::PushCapture && pushed < 48.0 - 1e-9) {
            push = std::min(10.0, (48.0 - pushed) / geared::kDt);
            pushed += push * geared::kDt;
        }
        cal.tick();
        t.step(push);
        const PanelView v = gate.view(cal, t.clock.now().value());
        if (pushed < 48.0 - 1e-9 || readyTicks == 0) {
            readyTicks += cal.pushReady() ? 1 : 0;
            continue;
        }
        CHECK(cal.pushReady());
        ++readyTicks;
        const PanelButton* done = nullptr;
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            done = v.buttons[b].id == PanelButtonId::Done ? &v.buttons[b] : done;
        }
        REQUIRE(done != nullptr);
        if (!done->armed) {
            CHECK(pressed == 0);  // once armed, it stays armed
            continue;
        }
        const TouchSample tap{false, ++releases, static_cast<std::int16_t>((done->rect.x0 + done->rect.x1) / 2),
                              static_cast<std::int16_t>((done->rect.y0 + done->rect.y1) / 2)};
        const TouchResult r = gate.touch(tap, v, cal);
        CHECK(r.action == TouchAction::Answer);
        pressed += r.action == TouchAction::Answer ? 1 : 0;
    }
    CHECK(readyTicks >= 200);
    CHECK(pressed >= 100);

    s.rateNoiseDegS = 0.0;
    s.driftDegS = 2.0;  // a real turn, from the start: the window always holds it
    GearedTank turning{s};
    DrivetrainCalibrator moving{turning.candidates(), turning.portSpan(), turning.imu, turning.clock,
                                geared::robotOneConfig()};
    moving.start(geared::facts());
    pushed = 0.0;
    int ready = 0;
    for (int k = 0; k < 1500; ++k) {
        double push = 0.0;
        if (pushed < 48.0 - 1e-9) {
            push = std::min(10.0, (48.0 - pushed) / geared::kDt);
            pushed += push * geared::kDt;
        }
        moving.tick();
        turning.step(push);
        ready += moving.pushReady() ? 1 : 0;
    }
    CHECK(ready == 0);
    CHECK_FALSE(moving.status().atRest);

    for (const double drift : {0.95, 1.05}) {  // robot one's floor is 1.0 deg/s: just under, just over
        CAPTURE(drift);
        s.driftDegS = drift;
        GearedTank near{s};
        DrivetrainCalibrator c{near.candidates(), near.portSpan(), near.imu, near.clock, geared::robotOneConfig()};
        c.start(geared::facts());
        pushed = 0.0;
        ready = 0;
        for (int k = 0; k < 1500; ++k) {
            double push = 0.0;
            if (pushed < 48.0 - 1e-9) {
                push = std::min(10.0, (48.0 - pushed) / geared::kDt);
                pushed += push * geared::kDt;
            }
            c.tick();
            near.step(push);
            ready += pushed >= 48.0 - 1e-9 && c.pushReady() ? 1 : 0;
        }
        CHECK((ready > 1000) == (drift < 1.0));
        CHECK((ready == 0) == (drift > 1.0));
    }
}

// ═══ C7. The new config's contract ═══════════════════════════════════════════════════
TEST_CASE("CAL2 C7: followThreshold, identifyYaw and the rest window must be positive; the default "
          "ramp is 3 / 6 / 9 / 12 V") {
    const CalibratorConfig d{};
    CHECK(d.pulseLevelCount == 4);
    CHECK(d.pulseLevels[3].value() == 12.0);
    CHECK(d.followThreshold.value() == doctest::Approx(2.0 * kDeg));
    CHECK(d.identifyYaw.value() == doctest::Approx(5.0 * kDeg));
    GearedTank t{TankSpec{}};
    for (int which = 0; which < 3; ++which) {
        CalibratorConfig c;
        if (which == 0) {
            c.followThreshold = shulib::units::AngleDim{0.0};
        } else if (which == 1) {
            c.identifyYaw = shulib::units::AngleDim{-1.0};
        } else {
            c.restSettleTime = shulib::units::Time{0.0};
        }
        CHECK_THROWS_AS(DrivetrainCalibrator(t.candidates(), t.portSpan(), t.imu, t.clock, c),
                        shulib::PreconditionError);
    }
}

// ═══ C8. One group, and ports the yaw decided (D3's mixed rule) ═══════════════════════
TEST_CASE("CAL2 C8: one group plus ports the yaw decided -- the turn drives the group forward and "
          "them back; a yaw side that disagrees with the turn is refused naming the port") {
    for (const double flex : {0.2, -0.2}) {
        CAPTURE(flex);
        TankSpec s;
        s.flexDegPerDeg = flex;  // 0.8 deg past robot one's 0.6 floor: the right way, then the wrong way
        GearedTank t{s};
        t.bias = [](const GearedTank& g, std::size_t k) {  // the RIGHT mates never show a pulse
            const std::size_t p = g.pulsed();
            return p >= 5 && p < 10 && k >= 5 && k != p ? -g.travel(k) : 0.0;
        };
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, geared::robotOneConfig()};
        (void)geared::run(t, cal);
        CHECK(cal.report().groupCount == 1);
        CHECK(cal.report().groupSide[1] == 0);
        CHECK(cal.report().ports[5].verdict == (flex > 0.0 ? PulseVerdict::Right : PulseVerdict::Left));
        if (flex > 0.0) {
            requireDone(cal);
            CHECK(cal.report().groupSide[0] == -1);
            CHECK(cal.result().sideOf(11) == -1);
            CHECK(cal.result().sideOf(16) == 1);
        } else {
            requireStopped(cal, Refusal::SideConflict, 16);
            CHECK(std::strstr(cal.reason(), "the spin put it on the RIGHT side") != nullptr);
        }
    }
}

// ═══ C9-C11. D3's mixed rule, both ways, and one group with no other side ═════════════════
TEST_CASE("CAL2 C9: one group {11 12 13}, the yaw puts 14 15 LEFT and 16-20 RIGHT -- refused naming "
          "port 16 before any turn is powered") {
    TankSpec s;
    s.flexDegPerDeg = 0.2;  // 0.8 deg of yaw: past robot one's 0.6 floor, so the yaw decides 14-20
    GearedTank t{s};
    t.bias = [](const GearedTank& g, std::size_t k) {
        const std::size_t p = g.pulsed();
        const bool hide = (p < 3 && (k == 3 || k == 4)) || ((p == 3 || p == 4) && k < 5 && k != p)
                          || (p >= 5 && p < 10 && k >= 5 && k != p);
        return hide ? -g.travel(k) : 0.0;
    };
    DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, geared::robotOneConfig()};
    bool turned = false;
    (void)runWatchingTurn(t, cal, turned);
    CHECK(cal.report().groupCount == 1);
    CHECK(cal.report().ports[3].verdict == PulseVerdict::Left);
    CHECK(cal.report().ports[5].verdict == PulseVerdict::Right);
    requireStopped(cal, Refusal::SideConflict, 16);
    CHECK(std::strstr(cal.reason(), "one group, and the yaw put port 16 on the other side") != nullptr);
    CHECK_FALSE(turned);  // never 11-13 forward against 14-15 backward in one train
}

TEST_CASE("CAL2 C10: two groups plus port 15, which only the yaw saw -- it keeps its yaw side (LEFT) "
          "and the run finishes") {
    TankSpec s;
    s.flexDegPerDeg = 0.2;
    GearedTank t{s};
    t.bias = [](const GearedTank& g, std::size_t k) {
        const std::size_t p = g.pulsed();
        return (p < 4 && k == 4) || (p == 4 && k < 5 && k != p) ? -g.travel(k) : 0.0;
    };
    DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, geared::robotOneConfig()};
    (void)geared::run(t, cal);
    requireDone(cal);
    CHECK(cal.report().groupCount == 2);
    CHECK(cal.report().ports[4].group == 0);
    CHECK(cal.report().ports[4].verdict == PulseVerdict::Left);
    CHECK(cal.report().groupSide[0] == -1);
    CHECK(cal.report().groupSide[1] == 1);
    CHECK(cal.result().sideOf(15) == -1);
    CHECK(cal.result().left.count == 5);
}

TEST_CASE("CAL2 C11: one group of every drive port and no other side -- ONE_SIDED before any turn") {
    GearedTank t{TankSpec{}};
    t.bias = [](const GearedTank& g, std::size_t k) {  // 16-19 read as if port 11's train; 20 stays
        return g.pulsed() == 0 && k >= 5 && k < 9 ? g.travel(1) - g.travel(k) : 0.0;
    };
    DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock};
    bool turned = false;
    (void)runWatchingTurn(t, cal, turned);
    requireStopped(cal, Refusal::OneSided, 0);
    CHECK(cal.report().groupCount == 1);
    CHECK_FALSE(turned);
}

// ═══ C12. A mate whose own pulse drags too little to count ════════════════════════════
// Bug caught (review, 2026-11-19): robot one's mates moved theirs 1.2-4.8 deg (11-17 log l.5503-5506,
// l.5367-5375). A grouped port's confirmation that moved no mate past followThreshold was refused
// PULSE_STILL, or, with its shaft past 20 deg, LEFT OUT as slipping.
TEST_CASE("CAL2 C12: a grouped port that turns its own shaft but drags its mates under the threshold "
          "stays in its group (KEPT); one that turns nothing stops PULSE_STILL; a gear that spins free is "
          "LEFT OUT") {
    struct Case {
        double extraOwnDeg;
        bool slipping;
    };
    for (const Case c : {Case{0.0, false}, Case{10.0, false}, Case{300.0, true}}) {
        CAPTURE(c.extraOwnDeg);
        GearedTank t{TankSpec{}};
        t.bias = [c](const GearedTank& g, std::size_t k) {  // port 12 pulsed: mates read 30 pct
            if (g.pulsed() != 1) {
                return 0.0;
            }
            return k != 1 ? -0.7 * g.travel(k) : c.extraOwnDeg * kDeg;
        };
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, geared::robotOneConfig()};
        (void)geared::run(t, cal);
        requireDone(cal);
        const auto& rep = cal.report();
        CHECK(rep.groupCount == 2);
        CHECK(rep.ports[1].group == 1);
        CHECK(rep.ports[1].pulseLevelVolts == 12.0);  // 6, 9, then 12 V: no mate past 2 deg
        CHECK(rep.ports[1].verdict == (c.slipping ? PulseVerdict::Decoupled : PulseVerdict::Kept));
        CHECK(rep.excludedCount == (c.slipping ? 1u : 0u));
        CHECK(cal.result().sideOf(12) == (c.slipping ? 0 : -1));
        CHECK(cal.result().left.count == (c.slipping ? 4 : 5));
        CHECK(cal.result().right.count == 5);
    }
    // Bug caught (verifier, 2026-11-19): KEPT took a grouped port whose own shaft never turned (a dead
    // or jammed motor). Its own shaft must turn movedThreshold (5 deg); robot one's turned 8.4-24.
    for (const double ownDeg : {0.0, 4.0, 6.0}) {
        CAPTURE(ownDeg);
        GearedTank t{TankSpec{}};
        t.bias = [ownDeg](const GearedTank& g, std::size_t k) {  // port 12 pulsed: no mate moves
            return g.pulsed() != 1 ? 0.0 : -g.travel(k) + (k == 1 ? ownDeg * kDeg : 0.0);
        };
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, geared::robotOneConfig()};
        (void)geared::run(t, cal);
        CHECK(cal.report().ports[1].pulseLevelVolts == 12.0);
        if (ownDeg < 5.0) {
            requireStopped(cal, Refusal::PulseStill, 12);
        } else {
            requireDone(cal);
            CHECK(cal.report().ports[1].verdict == PulseVerdict::Kept);
        }
    }
}

// ═══ C13. A light geared robot: the pulse turns it and its idle side rolls part-way ═════
// Bugs caught (review and verifier, 2026-11-19): two of the idle side's five read the roll (backlash
// differs per member), three stayed. Rolled BACK: refused FOLLOWER_REVERSED. Dragged FORWARD: the two
// joined the pulsed port's group, every port ended in one group, ONE_SIDED. The yaw alone finishes.
TEST_CASE("CAL2 C13: ports a turn rolls back are moved, not reversed; ports it drags forward are no "
          "mates of a group that turned the other way -- the run finishes with the right sides") {
    struct Case {
        double roll;  // -1 back, +1 forward
        bool coupling;
    };
    for (const Case c : {Case{-1.0, true}, Case{-1.0, false}, Case{1.0, true}, Case{1.0, false}}) {
        const double roll = c.roll;
        const bool coupling = c.coupling;
        CAPTURE(roll);
        CAPTURE(coupling);
        TankSpec s;
        s.chassisKs = 0.5;  // one motor of five at 3 V turns the robot about 1.3 deg
        GearedTank t{s};
        double theta0 = 0.0;
        std::size_t was = 99;
        t.bias = [&theta0, &was, roll](const GearedTank& g, std::size_t k) {
            const std::size_t p = g.pulsed();
            if (p != was) {
                was = p;
                theta0 = g.theta;
            }
            const std::size_t first = g.side(k) == 0 ? 0 : g.n / 2;
            if (p >= g.n || g.side(k) == g.side(p) || (k != first && k != first + 1)) {
                return 0.0;
            }
            return roll * (geared::kTrack / 2.0) * std::abs(g.theta - theta0) / geared::kRadius;
        };
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock,
                                 coupling ? geared::robotOneConfig() : yawOnly(geared::robotOneConfig())};
        (void)geared::run(t, cal);
        requireDone(cal);
        for (int p = 11; p <= 20; ++p) {
            CAPTURE(p);
            CHECK(cal.result().sideOf(p) == (p <= 15 ? -1 : 1));
        }
        if (coupling && roll > 0.0) {  // 11's pulse dragged 16 17 into A; their own pulses take them out
            CHECK(cal.report().groupCount == 2);
            CHECK(cal.report().ports[5].group == 2);
            CHECK(cal.report().ports[6].group == 2);
        }
    }
}

// ═══ C14. A yaw under the floor is no direction ══════════════════════════════════════════════
// Bug caught: robot one's port 11 turned it -0.48 deg at 6 V, under its 0.6 floor (11-17 log l.5700).
// Read as a direction, a group's first pulse or a mate's confirmation that reads +0.2 deg against
// -0.8 would take its mates out of group A.
TEST_CASE("CAL2 C14: port 11's or port 12's 6 V pulse reads +0.2 deg (under the floor), the others "
          "-0.8 deg -- all five stay in A") {
    for (const std::size_t odd : {std::size_t{0}, std::size_t{1}}) {
        CAPTURE(odd);
        TankSpec s;
        s.flexDegPerDeg = 0.2;  // 0.8 deg of yaw per pulse from 6 V, past robot one's 0.6 floor
        GearedTank t{s};
        DrivetrainCalibrator cal{t.candidates(), t.portSpan(), t.imu, t.clock, geared::robotOneConfig()};
        (void)geared::run(t, cal, [odd](GearedTank& g, DrivetrainCalibrator&) {
            if (g.pulsed() == odd && std::abs(g.motors[odd].commandedVoltage().value()) > 4.5) {  // +1 deg
                const double flex = g.spec.flexDegPerDeg * (g.windup(1) - g.windup(0));
                g.imu.setHeading(shulib::math::Angle::radians(g.theta + flex + 1.0 * kDeg));
            }
        });
        requireDone(cal);
        const auto& rep = cal.report();
        double yaw[2] = {0.0, 0.0};
        for (std::size_t k = 0; k < rep.pulseCount; ++k) {  // each port's deciding (last) pulse
            yaw[0] = rep.pulses[k].port == 11 ? rep.pulses[k].yawRad : yaw[0];
            yaw[1] = rep.pulses[k].port == 12 ? rep.pulses[k].yawRad : yaw[1];
        }
        CHECK(yaw[odd] > 0.0);
        CHECK(yaw[odd] < 0.6 * kDeg);
        CHECK(yaw[1 - odd] < -0.6 * kDeg);
        CHECK(rep.groupCount == 2);
        for (std::size_t j = 0; j < 5; ++j) {
            CAPTURE(j);
            CHECK(rep.ports[j].group == 1);
        }
        CHECK(cal.result().sideOf(12) == -1);
    }
}
