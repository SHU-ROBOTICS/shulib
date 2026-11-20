// Adversarial tests for Bench Tests' STATION 13 MECHANISMS (the mechanism core's MECH-A6, 2026-10-30) — its
// PURE parts, src/programs/bench_r3a.hpp's mech_station: the six gates, the direction check's page (jog ->
// observation -> mechanism::judgeDirection -> the question -> the verdict; a pneumatic's fire ->
// judgePneumatic), the readings in declared units, the switch's three states, the piece sensor's evidence
// and the RECORD lines (the MECH brief's §1.14 and §3.6).
//
// THE RIG: the station runs over the library's REAL robot::MechanismParts built on the fake device provider
// from the fixtures' robots (test/mechanism_fixtures.hpp — robot two's claws byte for byte as the real
// table; robot x's arm, lift, flywheel, intake and hood; robot y's wrist with a reversed angle sensor), so
// every command the station sends lands on a hal::fake device the test reads back. The clock is whole
// milliseconds (`ms / 1000.0`, as bench_r3a.cpp's `pros::millis() / 1000.0`), so every time edge below is
// landed on EXACTLY. Every case names the bug it catches. Nothing here has run on a robot: the page's
// drawing, the controller and the raw temperature read are bench_r3a.cpp's, and only a bench session
// checks them (the worksheet's Station P).
//
// THE REVIEW ROUND (2026-11-03): the cases from "station 13 re-entry" on were written for the three reviewers'
// findings — each names the finding's bug, and each was seen RED on a mutant of the header (MECH-PROGRESS.md,
// "MECH-A6 review findings and their disposition"). The direction check now ANNOUNCES first: A says "ONLY
// '<name>' should move" and drives NOTHING for kAnnounceMs, then the 0.4 s "+" jog (Bench::announce()). The
// last case PINS bench_r3a.cpp's own safety steps (the order of its prompts, the build and the guard), which
// no host test can run.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "programs/bench_r3a.hpp"
#include "robots/mechanism_table.hpp"
#include "source_pins.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/motion/motion_scheduler.hpp"

// The review round's own declarations — each the smallest robot that reaches a path the fixtures' robots do not:
// a power mechanism whose team LOWERED its maxVolts, a pneumatic whose START drives its line High, and a lift
// with EVERY reading role (four motors, an angle sensor, a home switch). Outside shulib, as the fixtures are.
namespace a6_fixture {
using namespace shulib::mechanism::words;
inline constexpr Power gentle{.name = "gentle", .motors = {Forward}, .cartridge = Red, .safe = Hold, .maxVolts = 6_volt};
inline constexpr Pneumatic latch{.name = "latch", .positions = {{"down", {Low}}, {"up", {High}}}, .start = "up",
                                 .safe = "down", .moveTime = 0.3_s};
inline constexpr MechanismTable kGentle = mechanisms(wire(gentle, motors(1)), wire(latch, lines('A')));
inline constexpr Lift bigLift{.name = "big lift", .motors = {Forward, Reversed, Forward, Reversed}, .cartridge = Red,
                              .gearing = direct, .shape = spool(1.5_in), .angleSensor = rotationSensor(), .safe = Hold,
                              .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in},
                              .gravity = constantGravity(1.0_volt), .home = homeBySwitch(0_in, -3_volt),
                              .presets = {{"floor", 0_in}}};
inline constexpr MechanismTable kBigLift =
    mechanisms(wire(bigLift, motors(1, 2, 3, 4), angleSensor(10), homeSwitch('B')));
}  // namespace a6_fixture

namespace ms = shulib::bench::mech_station;
namespace mech = shulib::mechanism;
using shulib::hal::BrakeMode;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Length;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct NullPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};

/// An IMotor that COUNTS every command — bound to a mechanism the station must never touch.
class CountingMotor final : public shulib::hal::IMotor {
public:
    void setVoltage(shulib::units::Voltage v) override {
        ++commands;
        last = v.value();
    }
    [[nodiscard]] shulib::units::Voltage commandedVoltage() const override { return shulib::units::Voltage{last}; }
    void setBrakeMode(BrakeMode m) override {
        ++commands;
        mode = m;
    }
    [[nodiscard]] BrakeMode brakeMode() const override { return mode; }
    [[nodiscard]] AngleDim position() const override { return AngleDim{0.0}; }
    [[nodiscard]] AngularVelocity velocity() const override { return AngularVelocity{0.0}; }
    [[nodiscard]] Current current() const override { return Current{0.0}; }
    [[nodiscard]] double temperature() const override { return 30.0; }
    int commands = 0;
    double last = 0.0;
    BrakeMode mode = BrakeMode::Coast;
};

/// The station over the REAL MechanismParts on the fake provider (header).
struct Bench {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    NullPacer pacer;
    shulib::hal::fake::FakeDeviceProvider dev{clock, battery, sink, pacer};
    shulib::config::MechanismConfig cfg[mech::kMaxMechanisms]{};
    shulib::robot::MechanismParts parts;
    const mech::MechanismTable& list;
    std::optional<ms::Station> st;
    std::uint32_t nowMs = 100000;
    std::vector<std::string> log;

    /// `startMs`: the clock at the first tick (a small one lands a stall window's edge where absolute
    /// seconds would round it — the review's R12).
    explicit Bench(const mech::MechanismTable& l, std::uint32_t startMs = 100000) : list{l}, nowMs{startMs} {
        st.emplace(list, parts);
    }
    /// The stage-1 YES: the parts built (on the first CLEAR of a power cycle), then the clear marked.
    void build() {
        const std::size_t n = ms::projectConfigs(list, std::span<shulib::config::MechanismConfig>{cfg});
        parts.build(std::span<const shulib::config::MechanismConfig>{cfg, n}, dev);
        st->markCleared();
    }
    /// One tick at the current time, then 10 ms pass.
    void tick(const ms::Pad& p) {
        st->tick(p, nowMs);
        drain();
        nowMs += static_cast<std::uint32_t>(ms::kTickMs);
    }
    void drain() {
        for (std::size_t k = 0; k < st->eventCount(); ++k) log.emplace_back(st->event(k));
        st->clearEvents();
    }
    [[nodiscard]] bool logged(std::string_view what) const {
        for (const std::string& l : log) {
            if (l.find(what) != std::string::npos) return true;
        }
        return false;
    }
    void select(std::size_t i) {
        while (st->selected() != i) st->next();
        drain();
    }
    /// Leave and re-enter station 13: a NEW Station over the SAME parts (bench_r3a.cpp builds them once per
    /// power cycle), and its CLEAR answered YES.
    void reenter() {
        st->exit();
        drain();
        st.emplace(list, parts);
        st->markCleared();
        drain();
    }
    /// The direction check's ANNOUNCEMENT on the shown (armed, resting) motor mechanism: L1 + A's press at t0,
    /// then every tick to t0 + 0.99 s — nothing driven, the page saying what should move. The NEXT tick (t0 +
    /// 1.00 s) starts the "+" jog.
    void announce() {
        tick(held());
        tick(held(false, false, true));
        REQUIRE(st->phase() == ms::Phase::Announcing);
        for (std::uint32_t k = 1; k < ms::kAnnounceMs / static_cast<std::uint32_t>(ms::kTickMs); ++k) {
            REQUIRE_FALSE(st->live(st->selected()));
            tick(held(false, false, true));
        }
        REQUIRE(st->phase() == ms::Phase::Announcing);
        REQUIRE_FALSE(st->live(st->selected()));
    }
    /// Gate 2's stage 2 for the shown motor mechanism: L1, then UP's press asks; YES (L1 held) arms; UP let go.
    void arm() {
        tick(held());
        tick(held(true));
        REQUIRE(st->phase() == ms::Phase::Confirm);
        st->confirm(true, true);
        drain();
        tick(held());
        REQUIRE(st->armed(st->selected()));
    }
    static ms::Pad held(bool up = false, bool down = false, bool a = false, bool r1 = false) {
        return ms::Pad{.connected = true, .deadMan = true, .raise = r1, .plus = up, .minus = down, .check = a};
    }
    static ms::Pad loose(bool up = false) {
        return ms::Pad{.connected = true, .deadMan = false, .raise = false, .plus = up, .minus = false, .check = false};
    }
    shulib::hal::fake::FakeMotor& motor(std::uint8_t port) { return dev.fakeMotor(port); }
    double volts(std::uint8_t port) { return dev.fakeMotor(port).commandedVoltage().value(); }
};

const mech::MechanismTable& robotTwo() { return mech_fixture::robot_two::kRobotTwoMechanisms; }
const mech::MechanismTable& robotX() { return mech_fixture::robot_x::kRobotXMechanisms; }
const mech::MechanismTable& robotY() { return mech_fixture::robot_y::kRobotYMechanisms; }

/// EVERY panel line of mechanism `i` (as many as bench_r3a.cpp asks for: kMaxPanelLines) with every raw
/// temperature `temp`.
std::vector<std::string> panel(Bench& b, std::size_t i, double temp = 31.0) {
    char out[ms::Station::kMaxPanelLines][ms::Station::kPanelBytes]{};
    const std::size_t n = b.st->panelLines(i, [temp](std::uint8_t) { return temp; }, out, ms::Station::kMaxPanelLines);
    return std::vector<std::string>(out, out + n);
}
/// Mechanism `i`'s RECORD lines with every raw temperature `temp`.
std::vector<std::string> record(Bench& b, int rec, std::size_t i, double temp = 30.0) {
    std::vector<std::string> out;
    b.st->record(rec, i, [temp](std::uint8_t) { return temp; }, [&](const char* l) { out.emplace_back(l); });
    return out;
}
bool has(const std::vector<std::string>& lines, std::string_view what) {
    for (const std::string& l : lines) {
        if (l.find(what) != std::string::npos) return true;
    }
    return false;
}

}  // namespace

// ═══ GATE 2, STAGE 1 — nothing is made or powered before the CLEAR ═══════════════════════════════

TEST_CASE("station 13 gate 2 stage 1: before the CLEAR nothing is made and every press is refused; the CLEAR "
          "names the line the build DRIVES") {
    // BUG CAUGHT: the station building (and so DRIVING robot two's claw line, HA-119) before anybody said
    // the robot was clear, a press acting before the CLEAR, or a CLEAR prompt that hides the line it drives.
    Bench b{robotTwo()};
    CHECK_FALSE(b.st->cleared());
    // line A not asked for: the Station alone never makes the parts — bench_r3a.cpp builds them only after
    // this CLEAR's YES, an order only its source shows (pinned by the last case, "pin: station 13's own ...")
    CHECK(b.dev.digitalOutRequestCount(1) == 0);
    const ms::PromptText t = b.st->clearText();
    CHECK(std::string{t.title} == "STATION 13 POWERS MECHANISMS");
    CHECK(std::string{t.foot1} == "making a line DRIVES it: 'claw' -> 'closed'");
    CHECK(std::string{t.sub} == "every path clear, hands away? (3 declared)");
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    CHECK(b.st->phase() == ms::Phase::Idle);  // no confirm asked: the CLEAR comes first
    CHECK(b.logged("REFUSED: the station's CLEAR question was not answered YES"));
    CHECK_FALSE(b.st->built(0));
    CHECK(std::string{b.st->absence(0)} == "not made yet: answer the station's CLEAR question first");
    // the CLEAR: the build drives line A at its START level ('closed' = Low), once
    b.build();
    CHECK(b.st->cleared());
    CHECK(b.dev.digitalOutMade(1));
    CHECK_FALSE(b.dev.digitalOutInitialState(1));
    CHECK(b.st->built(0));
    CHECK(b.st->built(2));
    // robot x: its three-position hood's start; a list of motors only drives nothing
    const Bench x{robotX()};
    CHECK(std::string{x.st->clearText().foot1} == "making a line DRIVES it: 'hood' -> 'down'");
    static constexpr mech::MechanismTable kArmOnly = mech::mechanisms(mech::wire(mech_fixture::robot_x::arm, mech::motors(1)));
    const Bench a{kArmOnly};
    CHECK(std::string{a.st->clearText().foot1} == "making a motor mechanism moves nothing");
}

// ═══ GATE 2, STAGE 2 — no volt before the confirm ════════════════════════════════════════════════

TEST_CASE("station 13 gate 2 stage 2: a motor mechanism gets NO volt before its confirm; YES only arms it, and "
          "it moves on a FRESH press") {
    // BUG CAUGHT (the brief's spot check): a volt before the two-stage confirm — UP acting on an unconfirmed
    // pivot, NO arming it, or a YES that drives at once because UP was still held through the prompt.
    Bench b{robotTwo()};
    b.build();
    b.tick(Bench::held());
    b.tick(Bench::held(true));  // L1 held, UP pressed: the confirm, never a volt
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    CHECK(b.volts(6) == 0.0);
    CHECK_FALSE(b.st->live(0));
    const ms::PromptText t = b.st->confirmText();
    CHECK(std::string{t.title} == "POWER 'claw pivot 6'?");
    CHECK(std::string{t.sub} == "ONLY it may move: path clear, hands away?");
    for (int k = 0; k < 5; ++k) b.tick(Bench::held(true));  // ticks while a confirm waits power nothing
    CHECK(b.volts(6) == 0.0);
    b.st->confirm(false, true);  // NO
    b.drain();
    CHECK_FALSE(b.st->armed(0));
    CHECK(b.st->phase() == ms::Phase::Idle);
    b.tick(Bench::held(true));  // UP still held: no new press, so no new question and no volt
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK(b.volts(6) == 0.0);
    b.tick(Bench::held());
    b.tick(Bench::held(true));  // a fresh press asks again
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    b.st->confirm(true, true);  // YES: armed, still nothing moves
    b.drain();
    CHECK(b.st->armed(0));
    CHECK(b.logged("ARMED 'claw pivot 6'"));
    b.tick(Bench::held(true));  // UP held THROUGH the prompt: not a press
    CHECK(b.volts(6) == 0.0);
    b.tick(Bench::held());
    b.tick(Bench::held(true));  // the fresh press: the jog, at the ceiling
    CHECK(b.volts(6) == 3.0);
    CHECK(b.st->live(0));
    CHECK(b.volts(7) == 0.0);  // ONLY the shown mechanism is ever powered
    // arming is per ENTRY: a new station (re-entering) asks again
    b.st.emplace(b.list, b.parts);
    b.st->markCleared();
    CHECK_FALSE(b.st->armed(0));
}

// ═══ GATE 3 — the dead-man ═══════════════════════════════════════════════════════════════════════

TEST_CASE("station 13 gate 3: UP without L1 commands nothing; letting go of L1 makes the mechanism safe THAT "
          "tick; a controller drop does too; a mechanism never powered is never commanded") {
    // BUG CAUGHT (M30): the dead-man ignored — a jog running without L1, or still running a tick after L1 is
    // let go, or after the controller drops; or the release "safing" a mechanism the station never touched.
    Bench b{robotTwo()};
    CountingMotor untouched;
    b.dev.bindMotor(7, untouched);  // claw pivot 7: the station must never command it here
    b.build();
    b.arm();
    b.tick(Bench::loose(true));  // UP pressed WITHOUT L1
    CHECK(b.volts(6) == 0.0);
    CHECK_FALSE(b.st->live(0));
    b.tick(Bench::loose());
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    REQUIRE(b.volts(6) == 3.0);
    b.motor(6).setBrakeMode(BrakeMode::Coast);  // so the safe state is seen being applied
    b.tick(Bench::loose(true));  // L1 let go, UP still held: safe THIS tick
    CHECK(b.volts(6) == 0.0);
    CHECK(b.motor(6).brakeMode() == BrakeMode::Hold);  // the declaration's safe state
    CHECK_FALSE(b.st->live(0));
    CHECK(b.logged("L1 released: every powered mechanism safe"));
    // a controller drop with every button "held" (a dropped controller reads nothing)
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    REQUIRE(b.volts(6) == 3.0);
    b.tick(ms::Pad{.connected = false, .deadMan = true, .raise = false, .plus = true, .minus = false, .check = false});
    CHECK(b.volts(6) == 0.0);
    CHECK(b.logged("controller DROPPED: every powered mechanism safe"));
    // a release with nothing powered says nothing and commands nothing
    const std::size_t lines = b.log.size();
    b.tick(Bench::held());
    b.tick(Bench::loose());
    CHECK(b.log.size() == lines);
    CHECK(untouched.commands == 0);
}

// ═══ GATE 4 — the ceiling ════════════════════════════════════════════════════════════════════════

TEST_CASE("station 13 gate 4: the ceiling STARTS at 3 V and R1's press steps it 3 -> 6 -> 9 -> 12, never above") {
    // BUG CAUGHT (M31): a ceiling starting at 12 V (a first jog at full power into an unmeasured claw), a step
    // per tick R1 is held instead of per press, or a ceiling past 12 V.
    Bench b{robotTwo()};
    b.build();
    CHECK(b.st->ceiling() == 3.0);
    b.arm();
    b.tick(Bench::held(true));
    CHECK(b.volts(6) == 3.0);  // the first jog: EXACTLY the starting ceiling
    b.tick(Bench::held(true, false, false, true));  // R1 pressed mid-jog
    CHECK(b.st->ceiling() == 6.0);
    CHECK(b.volts(6) == 6.0);
    for (int k = 0; k < 10; ++k) b.tick(Bench::held(true, false, false, true));  // held: one step only
    CHECK(b.st->ceiling() == 6.0);
    b.tick(Bench::held());
    for (const double want : {9.0, 12.0, 12.0, 12.0}) {
        b.tick(Bench::held(false, false, false, true));
        b.tick(Bench::held());
        CHECK(b.st->ceiling() == want);
    }
    CHECK(b.logged("ceiling 3 V -> 6 V"));
    CHECK(b.logged("ceiling 12 V -> 12 V (the most)"));
    b.tick(Bench::held(false, true));  // DOWN: the minus jog at the ceiling
    CHECK(b.volts(6) == -12.0);
}

// ═══ GATE 5 — the cut-outs ═══════════════════════════════════════════════════════════════════════

TEST_CASE("station 13 gate 5: two motors that FIGHT are cut at 250 ms exactly; every motor alike against the "
          "command is a back-drive, never a fight") {
    // BUG CAUGHT: a fight left running (a mirrored lift motor declared the wrong way stalls against its mate),
    // a cut a tick early or late, or a sagging lift — both motors dragged down alike — cut as a "fight".
    Bench b{robotX()};
    b.build();
    b.select(1);  // the lift: motors on ports 2 and 3 (3 mirrored)
    b.arm();
    b.motor(2).setVelocity(AngularVelocity{3.0});
    b.motor(3).setVelocity(AngularVelocity{-3.0});  // dragged against its command by its mate
    b.tick(Bench::held(true));  // drive tick 1
    for (int k = 2; k <= 24; ++k) b.tick(Bench::held(true));
    CHECK_FALSE(b.st->cut(1));  // 24 ticks: 240 ms
    CHECK(b.volts(2) == 3.0);
    b.tick(Bench::held(true));  // the 25th: 250 ms
    REQUIRE(b.st->cut(1));
    CHECK(std::string{b.st->cutWhy(1)} == "its motor(s) 2 disagree with their mates for 250 ms (a FIGHT)");
    CHECK(b.volts(2) == 0.0);
    CHECK(b.volts(3) == 0.0);
    CHECK(b.motor(2).brakeMode() == BrakeMode::Hold);
    CHECK(b.logged("CUT 'lift': its motor(s) 2 disagree"));
    // it stays cut until the station is re-entered
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    CHECK(b.volts(2) == 0.0);
    CHECK(b.logged("REFUSED 'lift': CUT ("));
    // both motors alike against the command: a back-drive — not cut by the fight rule
    Bench c{robotX()};
    c.build();
    c.select(1);
    c.arm();
    c.motor(2).setVelocity(AngularVelocity{-3.0});
    c.motor(3).setVelocity(AngularVelocity{-3.0});
    for (int k = 0; k < 100; ++k) c.tick(Bench::held(true));
    CHECK_FALSE(c.st->cut(1));
    CHECK(c.volts(2) == 3.0);
}

TEST_CASE("station 13 gate 5: any one motor ABOVE 2.4 A for 250 ms is cut; at 2.4 A it never is") {
    // BUG CAUGHT: a jammed pivot left at stall current (robot two's pivots state no stall signature — this
    // rule and the jog's time are all that cut them), a cut on the limit itself, or a window off by a tick.
    Bench b{robotTwo()};
    b.build();
    b.arm();
    b.motor(6).setCurrent(Current{2.4});  // AT the limit: never "above"
    for (int k = 0; k < 200; ++k) b.tick(Bench::held(true));
    CHECK_FALSE(b.st->cut(0));
    b.tick(Bench::held());
    b.motor(6).setCurrent(Current{2.41});
    b.tick(Bench::held(true));
    for (int k = 2; k <= 24; ++k) b.tick(Bench::held(true));
    CHECK_FALSE(b.st->cut(0));
    b.tick(Bench::held(true));
    REQUIRE(b.st->cut(0));
    CHECK(std::string{b.st->cutWhy(0)} == "motor 1 (port 6) above 2.4 A for 250 ms");
    CHECK(b.volts(6) == 0.0);
}

TEST_CASE("station 13 gate 5: the mechanism's OWN stall signature cuts a jog at its stated time; a mechanism "
          "with none is never cut by one") {
    // BUG CAUGHT (M48): the stall signature not watched by the station — an arm jogged into its stop sits at
    // its jam current (2.0 A, under the 2.4 A cut) until the 3 s jog limit; or a window that trips early.
    Bench b{robotX()};
    b.build();
    b.arm();  // the arm: .stall = {2.0 A, below 10 motor rpm, 0.5 s}
    b.motor(1).setCurrent(Current{2.0});
    b.motor(1).setVelocity(AngularVelocity{0.0});
    b.tick(Bench::held(true));  // t0: the window opens
    for (int k = 1; k <= 49; ++k) b.tick(Bench::held(true));  // up to t0 + 0.49 s
    CHECK_FALSE(b.st->cut(0));
    CHECK(b.volts(1) == 3.0);
    b.tick(Bench::held(true));  // t0 + 0.50 s: EXACTLY its lasting
    REQUIRE(b.st->cut(0));
    CHECK(std::string{b.st->cutWhy(0)} ==
          "its stall signature (2.00 A or more, slower than 1.0 rad/s at the motor, for 0.50 s)");
    CHECK(b.volts(1) == 0.0);
    // robot two's pivot states none: the same 2.0 A, stopped, runs on to the jog's own limit
    Bench c{robotTwo()};
    c.build();
    c.arm();
    c.motor(6).setCurrent(Current{2.0});
    for (int k = 0; k < 290; ++k) c.tick(Bench::held(true));
    CHECK_FALSE(c.st->cut(0));
}

TEST_CASE("station 13 gate 5: a jog held for its stated maximum, 3 s, is cut — at 3.00 s exactly, not 2.99") {
    // BUG CAUGHT: a held button running a mechanism for ever (robot two's pivots have no signature), or the
    // limit measured from the wrong moment.
    Bench b{robotTwo()};
    b.build();
    b.arm();
    b.motor(6).setCurrent(Current{0.3});
    b.tick(Bench::held(true));  // the press: t = start
    for (int k = 1; k <= 299; ++k) b.tick(Bench::held(true));  // start + 2.99 s
    CHECK_FALSE(b.st->cut(0));
    CHECK(b.volts(6) == 3.0);
    b.tick(Bench::held(true));  // start + 3.00 s
    REQUIRE(b.st->cut(0));
    CHECK(std::string{b.st->cutWhy(0)} == "a jog held for its stated maximum, 3.0 s (HA-182)");
    CHECK(b.volts(6) == 0.0);
    // a release and a new press start a new jog (and a new limit) — on a mechanism that was not cut
    Bench c{robotTwo()};
    c.build();
    c.arm();
    for (int k = 0; k < 200; ++k) c.tick(Bench::held(true));
    c.tick(Bench::held());
    for (int k = 0; k < 200; ++k) c.tick(Bench::held(true));
    CHECK_FALSE(c.st->cut(0));
}

// ═══ GATE 1 — only a built mechanism ═════════════════════════════════════════════════════════════

TEST_CASE("station 13 gate 1: an ABSENT mechanism offers nothing powered, and its page says the provider's "
          "sentence") {
    // BUG CAUGHT: a press on an unbuilt mechanism reaching a null device, a confirm asked for a mechanism
    // that cannot move, or a page that hides why it is absent.
    Bench b{robotTwo()};
    b.dev.refusePort(7);
    b.build();
    CHECK_FALSE(b.st->built(1));
    b.select(1);
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    CHECK(b.st->phase() == ms::Phase::Idle);  // no confirm for what cannot be powered
    CHECK(b.logged("REFUSED 'claw pivot 7': it was not built -- "));
    const std::vector<std::string> p = panel(b, 1);
    REQUIRE(p.size() >= 2);
    CHECK(p[0] == "ABSENT -- nothing on this page is powered:");
    CHECK(p[1] == "port 7 has no motor");  // the provider's sentence, naming the port
    CHECK(b.logged("REFUSED 'claw pivot 7': it was not built -- port 7 has no motor"));
    CHECK(std::string{b.st->absence(1)} == std::string{b.parts.absence(1)});
}

// ═══ GATE 6 — the exit ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("station 13 gate 6: exit() makes the powered mechanism safe — a fired claw back to 'closed' — and never "
          "touches one it did not power; it is idempotent") {
    // BUG CAUGHT: DONE (or the guard on another way out) leaving a pivot at its jog volts or the claw open, or
    // the exit "safing" an untouched mechanism (a command nobody asked for).
    Bench b{robotTwo()};
    CountingMotor untouched;
    b.dev.bindMotor(7, untouched);
    b.build();
    b.arm();
    b.tick(Bench::held(true));
    REQUIRE(b.volts(6) == 3.0);
    b.st->exit();
    CHECK(b.volts(6) == 0.0);
    CHECK(b.motor(6).brakeMode() == BrakeMode::Hold);
    CHECK_FALSE(b.st->live(0));
    b.st->exit();  // twice: harmless
    CHECK(untouched.commands == 0);
    // the claw: fired open, then the exit (L1 still held) puts it back to its safe position
    Bench c{robotTwo()};
    c.build();
    c.select(2);
    c.tick(Bench::held());
    c.tick(Bench::held(false, false, true));
    c.st->confirm(true, true);
    REQUIRE(c.dev.fakeDigitalOut(1).commanded());
    c.st->exit();
    CHECK_FALSE(c.dev.fakeDigitalOut(1).commanded());
    CHECK(c.st->phase() == ms::Phase::Idle);
}

// ═══ THE PNEUMATIC — every fire confirmed ════════════════════════════════════════════════════════

TEST_CASE("station 13: a valve is NEVER fired without its confirm; each fire names its position; letting go of "
          "L1 returns it to its safe position") {
    // BUG CAUGHT (M32): a valve fired without its confirm — the claw snapping on a hand — or fired when L1 was
    // let go during the prompt, or a fire whose prompt names the wrong position.
    Bench b{robotTwo()};
    b.build();
    shulib::hal::fake::FakeDigitalOut& lineA = b.dev.fakeDigitalOut(1);
    const int madeSets = lineA.setCount();  // the build's one drive
    b.select(2);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));  // L1 + A on the claw
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    CHECK(lineA.setCount() == madeSets);
    CHECK_FALSE(lineA.commanded());
    const ms::PromptText t = b.st->confirmText();
    CHECK(std::string{t.title} == "FIRE 'claw' -> 'open'?");
    CHECK(std::string{t.foot1} == "ONLY 'claw' should move. Letting go of L1");
    CHECK(std::string{t.foot2} == "returns it to 'closed' (its safe position).");
    b.st->confirm(true, false);  // YES, but L1 was let go during the prompt
    b.drain();
    CHECK(lineA.setCount() == madeSets);
    CHECK(b.logged("NOT fired: 'claw' -- L1 was not held when YES was touched"));
    b.tick(Bench::held());
    b.tick(Bench::held(true));  // UP fires too (the next position)
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    b.st->confirm(false, true);  // NO
    b.drain();
    CHECK(lineA.setCount() == madeSets);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    b.st->confirm(true, true);  // YES with L1 held: FIRED
    b.drain();
    CHECK(lineA.commanded());  // open = High (the table's guess, HA-159)
    CHECK(b.st->live(2));
    CHECK(b.st->phase() == ms::Phase::Asking);
    CHECK(b.logged("FIRED 'claw' -> 'open' (L1 held; letting go returns it to 'closed')"));
    const int firedSets = lineA.setCount();
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));  // a FRESH press of A while the question waits: no new fire asked
    CHECK(b.st->phase() == ms::Phase::Asking);
    b.tick(Bench::held());
    b.tick(Bench::held(true));  // and of UP
    CHECK(b.st->phase() == ms::Phase::Asking);
    CHECK(lineA.setCount() == firedSets);
    CHECK(lineA.commanded());
    b.tick(Bench::loose());  // L1 let go: back to 'closed' THIS tick
    CHECK_FALSE(lineA.commanded());
    CHECK_FALSE(b.st->live(2));
}

TEST_CASE("station 13 direction check, a PNEUMATIC: 'which position did it go to?' — its positions and 'nothing "
          "/ something else' as buttons -> judgePneumatic") {
    // BUG CAUGHT: the page's answers driving judgePneumatic with the wrong commanded position, a swapped answer
    // order, or no way to answer "nothing moved" (a valve on another line: the boot cannot see it).
    const auto fireOpen = [](Bench& b) {
        b.tick(Bench::held());
        b.tick(Bench::held(false, false, true));
        b.st->confirm(true, true);
        b.drain();
        REQUIRE(b.st->phase() == ms::Phase::Asking);
    };
    const mech::MechanismSpec claw = mech_fixture::robot_two::claw.spec();
    const mech::MechanismWiring& w = robotTwo().wiring[2];
    Bench b{robotTwo()};
    b.build();
    b.select(2);
    fireOpen(b);
    REQUIRE(b.st->answerCount() == 3);
    char l1[16], l2[16];
    b.st->answerWords(0, l1, l2);
    CHECK(std::string{l1} == "closed");
    b.st->answerWords(1, l1, l2);
    CHECK(std::string{l1} == "open");
    b.st->answerWords(2, l1, l2);
    CHECK(std::string{l1} == "NOTHING /");
    CHECK(std::string{l2} == "ELSE MOVED");
    char q[256];
    b.st->questionText(q, sizeof q);
    CHECK(std::string{q} == "ONLY 'claw' should have moved. Told 'open': which position did it go to?");
    CHECK(b.st->verdictCount() == 0);
    b.st->answer(1);  // it went 'open'
    b.drain();
    REQUIRE(b.st->hasVerdict());
    CHECK(b.st->verdictCount() == 1);  // the page shows each NEW verdict
    CHECK(b.st->verdict().finding == mech::DirectionFinding::Ok);
    CHECK(std::string{b.st->verdict().fix} == "'claw': 'open' confirmed");
    CHECK(b.logged("CHECK 'claw': answered 'open' -> OK: 'claw': 'open' confirmed"));
    for (const auto& [slot, saw, finding] :
         {std::tuple{0, std::uint8_t{0}, mech::DirectionFinding::MechanismReversed},
          std::tuple{2, mech::kNothingOrSomethingElseMoved, mech::DirectionFinding::CableOnAnotherMechanism}}) {
        CAPTURE(slot);
        b.tick(Bench::loose());
        fireOpen(b);
        b.st->answer(slot);
        CHECK(b.st->verdict().finding == finding);
        CHECK(std::string{b.st->verdict().fix} == std::string{mech::judgePneumatic(claw, w, 1, saw).fix});
    }
    // robot x's hood: three positions -> four answers; its first fire goes to 'middle' = C high, D low
    Bench h{robotX()};
    h.build();
    h.select(4);
    h.tick(Bench::held());
    h.tick(Bench::held(false, false, true));
    CHECK(std::string{h.st->confirmText().title} == "FIRE 'hood' -> 'middle'?");
    h.st->confirm(true, true);
    CHECK(h.dev.fakeDigitalOut(3).commanded());
    CHECK_FALSE(h.dev.fakeDigitalOut(4).commanded());
    CHECK(h.st->answerCount() == 4);
    h.st->answerWords(3, l1, l2);
    CHECK(std::string{l1} == "NOTHING /");
    h.st->answer(4);  // past the last answer: ignored
    CHECK(h.st->phase() == ms::Phase::Asking);
    h.st->answer(0);  // it went 'down'
    CHECK(h.st->verdict().finding == mech::DirectionFinding::MechanismReversed);
}

// ═══ THE DIRECTION CHECK — a motor mechanism ════════════════════════════════════════════════════

namespace {
/// Run a motor mechanism's check on `b`'s shown mechanism: L1 + A, its announcement (nothing moves), then
/// ticks until the jog ends, each jog tick after the first moving every motor in `ports` by `stepRad` and
/// drawing `amps`.
void runCheck(Bench& b, std::initializer_list<std::uint8_t> ports, double stepRad, double amps,
              int maxTicks = 60) {
    for (std::uint8_t p : ports) b.motor(p).setCurrent(Current{amps});
    b.announce();
    b.tick(Bench::held(false, false, true));  // t0 + 1.00 s: the jog's first tick, its baseline
    REQUIRE(b.st->phase() == ms::Phase::Jogging);
    for (int k = 0; k < maxTicks && b.st->phase() == ms::Phase::Jogging; ++k) {
        for (std::uint8_t p : ports) b.motor(p).setPosition(AngleDim{b.motor(p).position().value() + stepRad});
        b.tick(Bench::held(false, false, true));
    }
}
}  // namespace

TEST_CASE("station 13 direction check, a MOTOR mechanism: 'ONLY <name> should move' said first, then one + jog at "
          "the ceiling for 0.40 s -> the observation -> judgeDirection -> the three answers -> the verdict") {
    // BUG CAUGHT: a check jog at the wrong volts or for the wrong time, an observation that is not what the
    // encoders saw, the three answers wired to the wrong PersonSaw, or a verdict that is not the judgement's.
    const mech::MechanismSpec arm = mech_fixture::robot_x::arm.spec();
    const mech::MechanismWiring& w = robotX().wiring[0];
    Bench b{robotX()};
    b.build();
    b.arm();
    b.motor(1).setCurrent(Current{0.3});
    b.announce();  // A at t0: the page names what should move and drives nothing for 1.0 s
    CHECK(b.logged("CHECK 'arm': ONLY 'arm' should move -- watch it: a + jog at 3.0 V for 0.40 s in 1.0 s"));
    CHECK(b.volts(1) == 0.0);
    b.tick(Bench::held(false, false, true));  // t0 + 1.00 s: the jog at the ceiling
    REQUIRE(b.st->phase() == ms::Phase::Jogging);
    CHECK(b.volts(1) == 3.0);
    CHECK(b.logged("CHECK 'arm': the + jog at 3.0 V"));
    for (int k = 1; k <= 39; ++k) {  // t0 + 0.01 .. t0 + 0.39: still jogging
        b.motor(1).setPosition(AngleDim{b.motor(1).position().value() + 0.02});
        b.tick(Bench::held(false, false, true));
    }
    CHECK(b.st->phase() == ms::Phase::Jogging);
    CHECK(b.volts(1) == 3.0);
    b.motor(1).setPosition(AngleDim{b.motor(1).position().value() + 0.02});
    b.tick(Bench::held(false, false, true));  // t0 + 0.40: the jog ends, the arm safe, the question
    REQUIRE(b.st->phase() == ms::Phase::Asking);
    CHECK(b.volts(1) == 0.0);
    CHECK(b.motor(1).brakeMode() == BrakeMode::Hold);
    CHECK(b.st->observation().commandVolts == 3.0);
    CHECK(b.st->observation().motorDeltaRad[0] == doctest::Approx(0.80).epsilon(1e-12));
    CHECK(b.st->observation().motorPeakAmps[0] == 0.3);
    CHECK(b.logged("the encoders found nothing wrong -- asking which way it moved"));
    char q[256];
    b.st->questionText(q, sizeof q);
    CHECK(std::string{q} == "ONLY 'arm' should have moved: did it go its + way, the OTHER way, or did it NOT move "
                            "(or something ELSE moved)?");
    char l1[16], l2[16];
    b.st->answerWords(1, l1, l2);
    CHECK(std::string{l1} == "THE OTHER");
    CHECK(std::string{l2} == "WAY");
    b.st->answer(1);  // the OTHER way
    b.drain();
    mech::DirectionObservation o = b.st->observation();
    REQUIRE(b.st->verdict().finding == mech::DirectionFinding::MechanismReversed);
    o.person = mech::PersonSaw::TheOtherWay;
    CHECK(std::string{b.st->verdict().fix} == std::string{mech::judgeDirection(arm, w, o, ms::kMinMoveRad, ms::kDrawAmps).fix});
    CHECK(std::string_view{b.st->verdict().fix}.find("test/mechanism_fixtures.hpp:38") != std::string_view::npos);
    CHECK(b.logged("CHECK 'arm': answered 'the other way' -> REVERSED: 'arm' moved the OTHER way"));
    // the other two answers, each on a fresh check
    const std::pair<int, mech::DirectionFinding> rest[] = {{0, mech::DirectionFinding::Ok},
                                                           {2, mech::DirectionFinding::CableOnAnotherMechanism}};
    for (const auto& [slot, finding] : rest) {
        CAPTURE(slot);
        runCheck(b, {1}, 0.02, 0.3);
        REQUIRE(b.st->phase() == ms::Phase::Asking);
        b.st->answer(slot);
        CHECK(b.st->verdict().finding == finding);
    }
    CHECK(std::string_view{b.st->verdict().fix}.find("test/mechanism_fixtures.hpp:93") != std::string_view::npos);
    CHECK(std::string{b.st->verdict().fix} ==
          std::string{mech::judgeDirection(arm, w,
                                           mech::DirectionObservation{.commandVolts = 3.0, .motorDeltaRad = {0.8},
                                                                      .motorPeakAmps = {0.3},
                                                                      .person = mech::PersonSaw::NothingOrSomethingElseMoved},
                                           ms::kMinMoveRad, ms::kDrawAmps)
                          .fix});
    // the check's volts follow the ceiling at its start
    b.tick(Bench::held(false, false, false, true));
    b.announce();
    CHECK(b.st->observation().commandVolts == 6.0);
    b.tick(Bench::held(false, false, true));
    CHECK(b.volts(1) == 6.0);
}

TEST_CASE("station 13 direction check: what the encoders see is reported AT ONCE — a fight (its cut names the "
          "motor), a back-drive, a jam, a motor that never answered, a sensor against its motors") {
    // BUG CAUGHT: the page asking the person about a fault the encoders already show, a fight's 250 ms cut
    // throwing the check away instead of naming the motor, or the angle sensor read without its mounting.
    {  // a FIGHT on the lift: motor 2 dragged backwards; the cut at 250 ms ends the jog and the judgement names it
        Bench b{robotX()};
        b.build();
        b.select(1);
        b.arm();
        b.announce();
        b.motor(2).setVelocity(AngularVelocity{3.0});
        b.motor(3).setVelocity(AngularVelocity{-3.0});
        b.tick(Bench::held(false, false, true));
        for (int k = 0; k < 60 && b.st->phase() == ms::Phase::Jogging; ++k) {
            b.motor(2).setPosition(AngleDim{b.motor(2).position().value() + 0.03});
            b.motor(3).setPosition(AngleDim{b.motor(3).position().value() - 0.03});
            b.tick(Bench::held(false, false, true));
        }
        REQUIRE(b.st->hasVerdict());
        CHECK(b.st->phase() == ms::Phase::Idle);
        CHECK(b.st->cut(1));
        CHECK(b.st->verdict().finding == mech::DirectionFinding::MotorFights);
        CHECK(b.st->verdictCount() == 1);  // a NEW verdict: the page shows it at once (R21)
        CHECK(b.st->verdict().role == 1);
        CHECK(b.logged("CHECK 'lift' (the jog was CUT) -> A MOTOR FIGHTS: 'lift' motor 2 turns AGAINST its command"));
    }
    {  // a BACK-DRIVE: the one-motor arm turned against its command — no line named
        Bench b{robotX()};
        b.build();
        b.arm();
        runCheck(b, {1}, -0.02, 0.3);
        REQUIRE(b.st->hasVerdict());
        CHECK(b.st->verdict().finding == mech::DirectionFinding::BackDriven);
        CHECK(b.st->verdictCount() == 1);  // a NEW verdict: the page shows it at once (R21)
        CHECK(std::string_view{b.st->verdict().fix}.find("jog it harder, or from another pose") != std::string_view::npos);
        CHECK_FALSE(b.st->cut(0));  // one motor: no "fight" cut — the judgement speaks
    }
    {  // a JAM: still, drawing at least the page's 0.40 A — and AT it exactly (the equality edge)
        Bench b{robotX()};
        b.build();
        b.arm();
        runCheck(b, {1}, 0.0, ms::kDrawAmps);
        REQUIRE(b.st->hasVerdict());
        CHECK(b.st->verdict().finding == mech::DirectionFinding::FightOrJam);
        CHECK(b.st->verdictCount() == 1);  // a NEW verdict: the page shows it at once (R21)
    }
    {  // still and NOT drawing: its cable, its wire line
        Bench b{robotX()};
        b.build();
        b.arm();
        runCheck(b, {1}, 0.0, 0.39);
        REQUIRE(b.st->hasVerdict());
        CHECK(b.st->verdict().finding == mech::DirectionFinding::DidNotMove);
        CHECK(b.st->verdictCount() == 1);  // a NEW verdict: the page shows it at once (R21)
    }
    {  // an encoder change EXACTLY at the page's "did not move" angle counts as moved (the equality edge)
        Bench b{robotX()};
        b.build();
        b.arm();
        b.motor(1).setCurrent(Current{0.1});
        b.announce();
        b.tick(Bench::held(false, false, true));
        for (int k = 0; k < 60 && b.st->phase() == ms::Phase::Jogging; ++k) {
            b.motor(1).setPosition(AngleDim{ms::kMinMoveRad});
            b.tick(Bench::held(false, false, true));
        }
        CHECK(b.st->observation().motorDeltaRad[0] == ms::kMinMoveRad);
        CHECK(b.st->phase() == ms::Phase::Asking);
    }
    {  // robot y's wrist: its sensor is mounted Reversed; the raw reading rising reads AGAINST the motors
        Bench b{robotY()};
        b.build();
        b.arm();
        b.motor(1).setCurrent(Current{0.3});
        b.announce();
        b.tick(Bench::held(false, false, true));
        for (int k = 0; k < 60 && b.st->phase() == ms::Phase::Jogging; ++k) {
            b.motor(1).setPosition(AngleDim{b.motor(1).position().value() + 0.02});
            b.dev.fakeRotation(10).setPosition(AngleDim{b.dev.fakeRotation(10).position().value() + 0.01});
            b.tick(Bench::held(false, false, true));
        }
        REQUIRE(b.st->hasVerdict());
        CHECK(b.st->verdict().finding == mech::DirectionFinding::SensorReversed);
        CHECK(b.st->verdictCount() == 1);  // a NEW verdict: the page shows it at once (R21)
        CHECK(b.st->observation().sensorDeltaRad < 0.0);  // as the declaration mounts it
        CHECK(b.st->observation().hasAngleSensor);
    }
}

TEST_CASE("station 13 direction check: L1 let go mid-jog or mid-announcement, NEXT, or the exit abandon it — "
          "nothing judged, the mechanism safe, no jog after") {
    // BUG CAUGHT: a half-jog judged as if it were whole (a short jog reads as "did not move"), the jog left
    // running when the page moves on, or an announcement that still jogs after L1 was let go.
    Bench b{robotX()};
    b.build();
    b.arm();
    b.announce();
    b.tick(Bench::held(false, false, true));  // the jog
    REQUIRE(b.st->phase() == ms::Phase::Jogging);
    for (int k = 0; k < 10; ++k) b.tick(Bench::held(false, false, true));
    b.tick(Bench::loose());
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK_FALSE(b.st->hasVerdict());
    CHECK(b.volts(1) == 0.0);
    CHECK(b.logged("L1 was let go before the check's jog ended -- nothing judged"));
    // let go DURING the announcement: abandoned before any volt, and L1 held again starts nothing
    b.log.clear();
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    REQUIRE(b.st->phase() == ms::Phase::Announcing);
    for (int k = 0; k < 50; ++k) b.tick(Bench::held(false, false, true));
    b.tick(ms::Pad{.connected = true, .deadMan = false, .raise = false, .plus = false, .minus = false, .check = true});
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK(b.logged("L1 was let go before the check's jog ended -- nothing judged"));
    for (int k = 0; k < 120; ++k) b.tick(Bench::held(false, false, true));  // A still held: no new check
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK(b.volts(1) == 0.0);
    // the controller DROPPED during the announcement
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    REQUIRE(b.st->phase() == ms::Phase::Announcing);
    b.tick(ms::Pad{.connected = false, .deadMan = false, .raise = false, .plus = false, .minus = false, .check = false});
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK(b.logged("the controller DROPPED during the check -- nothing judged"));
    // NEXT during the announcement, and during the jog
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    REQUIRE(b.st->phase() == ms::Phase::Announcing);
    b.st->next();  // to the lift
    b.drain();
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK(b.logged("CHECK 'arm': NEXT pressed"));
    b.select(0);
    b.announce();
    b.tick(Bench::held(false, false, true));
    REQUIRE(b.st->phase() == ms::Phase::Jogging);
    b.st->next();  // to the lift
    b.drain();
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK(b.volts(1) == 0.0);
    CHECK(b.st->selected() == 1);
    CHECK_FALSE(b.st->hasVerdict());
    // the exit during the announcement
    b.select(0);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    REQUIRE(b.st->phase() == ms::Phase::Announcing);
    b.st->exit();
    b.drain();
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK(b.logged("the station was left during the check -- nothing judged"));
    CHECK(b.volts(1) == 0.0);
    // a question dropped (B) judges nothing
    Bench c{robotX()};
    c.build();
    c.arm();
    runCheck(c, {1}, 0.02, 0.3);
    REQUIRE(c.st->phase() == ms::Phase::Asking);
    c.st->cancelQuestion();
    c.drain();
    CHECK(c.st->phase() == ms::Phase::Idle);
    CHECK_FALSE(c.st->hasVerdict());
    CHECK(c.logged("question dropped -- nothing judged"));
}

// ═══ THE READINGS — in declared units, from power-up ════════════════════════════════════════════

TEST_CASE("station 13 readings (U1): a 12:60 arm turned one output turn reads 360 deg; a 1.5 in spool lift one "
          "spool turn reads pi x 1.5 in; a flywheel reads its output rpm; a wrong gearing reads a wrong angle") {
    // BUG CAUGHT: the gearing applied the wrong way round (the N11 slip: 1/5 instead of 5), a spool read as
    // its diameter instead of its circumference, a reading not from the declared home, or rpm through the
    // gearing upside down.
    const mech::MechanismSpec arm = mech_fixture::robot_x::arm.spec();
    const ms::UnitsReading a = ms::unitsReading(arm, 5.0 * 2.0 * kPi, 0.0);  // 5 motor turns = 1 output turn
    CHECK(a.has);
    CHECK(a.value == doctest::Approx(360.0).epsilon(1e-12));
    CHECK(a.outputDeg == doctest::Approx(360.0).epsilon(1e-12));
    CHECK(std::string{a.unit} == "deg");
    char t[96];
    ms::unitsText(t, sizeof t, arm, a);
    CHECK(std::string{t} == "output +360.00 deg since power-up (+360.0 deg)");
    const mech::MechanismSpec lift = mech_fixture::robot_x::lift.spec();
    const ms::UnitsReading l = ms::unitsReading(lift, 2.0 * kPi, 0.0);  // direct: one motor turn = one spool turn
    CHECK(l.value == doctest::Approx(kPi * 1.5).epsilon(1e-12));
    CHECK(std::string{l.unit} == "in");
    ms::unitsText(t, sizeof t, lift, l);
    CHECK(std::string{t} == "output +360.00 deg since power-up (+4.71 in)");
    const mech::MechanismSpec fly = mech_fixture::robot_x::flywheel.spec();  // 72:12 — geared UP six times
    const ms::UnitsReading f = ms::unitsReading(fly, 0.0, 600.0 * 2.0 * kPi / 60.0);  // the motor at 600 rpm
    CHECK(f.value == doctest::Approx(3600.0).epsilon(1e-12));
    ms::unitsText(t, sizeof t, fly, f);
    CHECK(std::string{t} == "output +3600 rpm");
    // the power-up pose is the declared HOME: robot y's dr4b rests at 0 in — a linkage, so its height runs
    // through the sine from there, and at power-up it reads its home exactly
    const mech::MechanismSpec dr4b = mech_fixture::robot_y::dr4b.spec();
    CHECK(ms::unitsReading(dr4b, 0.0, 0.0).value == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(ms::unitsReading(dr4b, 5.0 * 2.0 * kPi * 10.0 / 360.0, 0.0).value ==
          doctest::Approx(mech::angleDegToPosition(dr4b.position,
                                                   mech::positionToAngleDeg(dr4b.position, 0.0) + 10.0)));
    // a power and a pneumatic declare no units
    CHECK_FALSE(ms::unitsReading(mech_fixture::robot_x::intake.spec(), 1.0, 1.0).has);
    CHECK_FALSE(ms::unitsReading(mech_fixture::robot_x::hood.spec(), 1.0, 1.0).has);
    // THROUGH THE PAGE: the arm's motor turned 5 turns by hand reads 360 deg; robot y's 12:36 wrist, turned
    // the same, reads 600 — a gearing typed wrong is a wrong angle on the screen
    Bench b{robotX()};
    b.build();
    b.motor(1).setPosition(AngleDim{5.0 * 2.0 * kPi});
    CHECK(has(panel(b, 0), "output +360.00 deg since power-up (+360.0 deg)"));
    CHECK(has(panel(b, 0), "motor 1 (port 1) +1800.0 deg 0.00 A 31C"));
    Bench y{robotY()};
    y.build();
    y.motor(1).setPosition(AngleDim{5.0 * 2.0 * kPi});
    CHECK(has(panel(y, 0), "output +600.00 deg since power-up (+600.0 deg)"));
}

TEST_CASE("station 13 readings: a motor's temperature is 'unknown' when its read failed — never the adapter's "
          "held 20 C") {
    // BUG CAUGHT: a failed temperature read shown as a cool motor (the adapter seeds 20 C and holds the last
    // good value, so the page must read the RAW value and say "unknown").
    char t[16];
    ms::temperatureText(t, sizeof t, std::numeric_limits<double>::infinity());  // PROS_ERR_F
    CHECK(std::string{t} == "unknown");
    ms::temperatureText(t, sizeof t, std::numeric_limits<double>::quiet_NaN());
    CHECK(std::string{t} == "unknown");
    ms::temperatureText(t, sizeof t, 31.4);
    CHECK(std::string{t} == "31C");
    Bench b{robotTwo()};
    b.build();
    b.motor(6).setTemperature(20.0);  // what a held adapter would say
    CHECK(has(panel(b, 0, std::numeric_limits<double>::infinity()), "motor 1 (port 6) +0.0 deg 0.00 A unknown"));
    CHECK_FALSE(has(panel(b, 0, std::numeric_limits<double>::infinity()), "20C"));
}

TEST_CASE("station 13 readings: a limit switch PRESSED, RELEASED and FAILING are three different lines (HA-184's "
          "settle); the RECORD carries the raw level") {
    // BUG CAUGHT: a switch shown only as "pressed / not pressed" (an unplugged line would hide inside one), a
    // failed read shown as a level, or a failing switch remembered as failing after it answers again.
    char s[96];
    ms::switchText(s, sizeof s, "home sw", 'B', true, true, false);
    CHECK(std::string{s} == "home sw B: reads 1 = PRESSED");
    ms::switchText(s, sizeof s, "home sw", 'B', false, true, false);
    CHECK(std::string{s} == "home sw B: reads 0 = released");
    ms::switchText(s, sizeof s, "home sw", 'B', true, true, true);
    CHECK(std::string{s} == "home sw B: NOT ANSWERING (a read failed)");
    ms::switchText(s, sizeof s, "piece sw", 'E', false, false, false);  // a pressedLevel of 0: 0 is PRESSED
    CHECK(std::string{s} == "piece sw E: reads 0 = PRESSED");
    Bench b{robotX()};  // the lift's home switch on 3-wire B
    b.build();
    b.dev.fakeDigitalIn(2).setState(true);
    CHECK(has(panel(b, 1), "home sw B: reads 1 = PRESSED"));
    CHECK(has(panel(b, 1), "  RECORD it pressed, released, and cable OUT"));  // the screen ASKS for all three
    CHECK_FALSE(has(panel(b, 0), "cable OUT"));  // the arm has no switch: nothing asked
    b.dev.fakeDigitalIn(2).setState(false);
    CHECK(has(panel(b, 1), "home sw B: reads 0 = released"));
    b.dev.fakeLineReadFaults(2).failing(true);
    CHECK(has(panel(b, 1), "home sw B: NOT ANSWERING (a read failed)"));
    b.dev.fakeLineReadFaults(2).failing(false);
    (void)panel(b, 1);  // the paint after the failure stopped catches up with the count
    CHECK(has(panel(b, 1), "home sw B: reads 0 = released"));
    std::vector<std::string> rec;
    b.st->record(4, 1, [](std::uint8_t) { return 31.0; }, [&](const char* l) { rec.emplace_back(l); });
    CHECK(has(rec, "[R3A] mech REC 4 'lift' home switch 3-wire B reads 0 (pressedLevel 1; HA-184), read faults "));
    CHECK(has(rec, "-- RECORD it pressed, released and with its cable OUT"));
}

TEST_CASE("station 13 readings: the piece sensor's evidence at its edges — the service's positive-evidence rule") {
    // BUG CAUGHT: the page disagreeing with what an auton step would conclude — a distance AT its threshold read
    // as a piece, a near reading with no confidence read as one, a failing sensor read as "no piece".
    CHECK(ms::distanceEvidence(2.0, 0.9, 2.0, false) == ms::Evidence::NoPiece);  // AT the threshold
    CHECK(ms::distanceEvidence(1.99, 0.9, 2.0, false) == ms::Evidence::Sees);
    CHECK(ms::distanceEvidence(1.0, 0.0, 2.0, false) == ms::Evidence::NoEvidence);  // near, confidence 0
    CHECK(ms::distanceEvidence(std::numeric_limits<double>::quiet_NaN(), 0.9, 2.0, false) == ms::Evidence::NoEvidence);
    CHECK(ms::distanceEvidence(1.0, 0.9, 2.0, true) == ms::Evidence::NotAnswering);
    CHECK(ms::opticalEvidence(0.5, 0.5, false) == ms::Evidence::Sees);  // AT the threshold sees (>=)
    CHECK(ms::opticalEvidence(0.49, 0.5, false) == ms::Evidence::NoPiece);
    CHECK(ms::opticalEvidence(0.9, 0.5, true) == ms::Evidence::NotAnswering);
    Bench b{robotX()};  // the intake's distance sensor on port 9, "sees" within 2 in
    b.build();
    b.dev.fakeDistance(9).setDistance(Length{393.66});
    b.dev.fakeDistance(9).setConfidence(1.0);
    CHECK(has(panel(b, 3), "piece p9: 393.66in conf 1.00 -> no piece (within 2.00in)"));
    b.dev.fakeDistance(9).setDistance(Length{1.0});
    b.dev.fakeDistance(9).setConfidence(0.8);
    CHECK(has(panel(b, 3), "-> SEES a piece"));
    b.dev.fakeReadFaults(9).failing(true);
    CHECK(has(panel(b, 3), "-> NOT ANSWERING"));
    Bench m{robotX()};
    m.dev.refusePort(9);
    m.build();
    CHECK(has(panel(m, 3), "piece p9: MISSING this boot -- it runs without one"));
}

TEST_CASE("station 13 readings: the angle sensor raw and as mounted (HA-186's reading); a pneumatic's lines and "
          "position") {
    // BUG CAUGHT: an angle sensor shown without its mounting (the team reads absoluteSensor's number off it),
    // or a claw page that does not say which level each line holds.
    Bench y{robotY()};
    y.build();
    y.dev.fakeRotation(10).setPosition(AngleDim{kPi / 2.0});
    CHECK(has(panel(y, 0), "angle p10: raw +90.0deg, as mounted -90.0deg"));
    CHECK(has(panel(y, 0), "  RECORD it at 2 marks: power-cycled, restarted"));  // HA-186's two readings
    Bench b{robotTwo()};
    b.build();
    const std::vector<std::string> p = panel(b, 2);
    CHECK(has(p, "line A: commanded LOW"));
    CHECK(has(p, "position: 'closed'"));
}

// ═══ RECORD ══════════════════════════════════════════════════════════════════════════════════════

TEST_CASE("station 13 RECORD: `[R3A] mech REC` lines with every number in full, and the last verdict for its "
          "mechanism") {
    // BUG CAUGHT: a RECORD that drops the motor's port, the units' basis (the gearing and the home), or the
    // verdict the session is about.
    Bench b{robotX()};
    b.build();
    b.motor(1).setPosition(AngleDim{5.0 * 2.0 * kPi});
    std::vector<std::string> rec;
    const auto into = [&](const char* l) { rec.emplace_back(l); };
    b.st->record(2, 0, [](std::uint8_t) { return std::numeric_limits<double>::infinity(); }, into);
    REQUIRE(rec.size() == 3);
    CHECK(rec[0] == "[R3A] mech REC 2 'arm' ARM built, not armed, ceiling 3 V");
    CHECK(rec[1] == "[R3A] mech REC 2 'arm' motor 1 port 1 pos +31.416 rad (+1800.0 deg) vel +0.000 rad/s cur 0.000 A "
                    "temp unknown");
    CHECK(rec[2] == "[R3A] mech REC 2 'arm' ARM +360.0000 deg from power-up (output +360.00 deg through 5.0000 motor "
                    "turns per output turn; the power-up pose taken as its home 0.0000)");
    b.arm();
    runCheck(b, {1}, 0.02, 0.3);
    b.st->answer(0);
    rec.clear();
    b.st->record(3, 0, [](std::uint8_t) { return 30.0; }, into);
    CHECK(rec.front() == "[R3A] mech REC 3 'arm' ARM built, armed, ceiling 3 V");
    CHECK(rec.back() == "[R3A] mech REC 3 'arm' last check: OK (its + way): 'arm': direction confirmed");
    rec.clear();
    b.st->record(3, 1, [](std::uint8_t) { return 30.0; }, into);  // the lift: no verdict of its own
    CHECK_FALSE(has(rec, "last check"));
    Bench a{robotTwo()};
    a.dev.refusePort(6);
    a.build();
    rec.clear();
    a.st->record(1, 0, [](std::uint8_t) { return 30.0; }, into);
    REQUIRE(rec.size() == 1);
    CHECK(rec[0].starts_with("[R3A] mech REC 1 'claw pivot 6' POWER ABSENT: "));
}

// ═══ THE BUILD, NEXT, AND THE WRAP ═══════════════════════════════════════════════════════════════

TEST_CASE("station 13 builds what the programs build: projectConfigs is toMechanismConfig entry for entry — robot "
          "two on ports 6 and 7 and 3-wire A") {
    // BUG CAUGHT: the station building its own idea of a mechanism (a sign dropped, a line moved) instead of the
    // one projection every program boots from.
    shulib::config::MechanismConfig cfg[mech::kMaxMechanisms]{};
    const mech::MechanismTable& t = robotTwo();
    REQUIRE(ms::projectConfigs(t, std::span<shulib::config::MechanismConfig>{cfg}) == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        const shulib::config::MechanismConfig want = mech::toMechanismConfig(t.specs[i], t.wiring[i]);
        CHECK(std::string{cfg[i].name} == std::string{want.name});
        CHECK(cfg[i].motorCount == want.motorCount);
        CHECK(cfg[i].motorPorts[0] == want.motorPorts[0]);
        CHECK(cfg[i].pneumaticCount == want.pneumaticCount);
        CHECK(cfg[i].pneumaticPorts[0] == want.pneumaticPorts[0]);
    }
    CHECK(cfg[0].motorPorts[0] == 6);
    CHECK(cfg[1].motorPorts[0] == 7);
    CHECK(cfg[2].pneumaticPorts[0] == 1);
    shulib::config::MechanismConfig two[2]{};
    CHECK(ms::projectConfigs(robotX(), std::span<shulib::config::MechanismConfig>{two}) == 2);  // bounded
    CHECK(std::string{two[1].name} == "lift");
    // robot x's mirrored lift motor: the projection's NEGATIVE port reaches the provider
    Bench b{robotX()};
    b.build();
    CHECK(b.dev.motorSignedPort(3) == -3);
}

TEST_CASE("station 13 NEXT: one mechanism at a time — leaving a powered one makes it safe; the page wraps round") {
    // BUG CAUGHT: two mechanisms powered at once (the shown one changed while the last kept its volts), or a NEXT
    // past the end showing nothing.
    Bench b{robotTwo()};
    b.build();
    b.arm();
    b.tick(Bench::held(true));
    REQUIRE(b.st->live(0));
    b.st->next();
    b.drain();
    CHECK(b.volts(6) == 0.0);
    CHECK_FALSE(b.st->live(0));
    CHECK(b.st->selected() == 1);
    CHECK(b.logged("[R3A] mech page: 'claw pivot 7' (POWER)"));
    b.tick(Bench::held(true));  // still held: pivot 7 is not armed, and no new press — nothing
    CHECK(b.volts(7) == 0.0);
    b.st->next();
    b.st->next();
    CHECK(b.st->selected() == 0);
}

TEST_CASE("station 13 wrapText: a verdict's sentence on the panel — no line past its width, no word lost") {
    // BUG CAUGHT: the panel cutting the robot-file line out of a verdict (the one thing the team must read).
    const char* text = "'arm' moved the OTHER way: in the declaration at test/mechanism_fixtures.hpp:38, .motors: "
                       "swap EVERY Forward and Reversed (and the angle sensor's), then rebuild and upload";
    char out[8][96]{};
    const std::size_t n = ms::wrapText(text, 50, out, 8);
    REQUIRE(n >= 3);
    std::string joined;
    for (std::size_t i = 0; i < n; ++i) {
        CHECK(std::string{out[i]}.size() <= 50);
        joined += (i == 0 ? "" : " ") + std::string{out[i]};
    }
    CHECK(joined == text);
    CHECK(ms::wrapText(text, 50, out, 2) == 2);  // bounded by its lines
    CHECK(ms::wrapText(nullptr, 50, out, 8) == 0);
    char narrow[2][8]{};
    CHECK(ms::wrapText("abcdefghijklmnop", 7, narrow, 2) == 2);  // a word longer than a line is cut
    CHECK(std::string{narrow[0]} == "abcdefg");
}

TEST_CASE("station 13's prompts: every line of robot two's prompts fits where twoButtonPrompt prints it") {
    // BUG CAUGHT: a confirm whose decisive words fall off the panel's right edge ("FIRE 'claw' -> 'op"),
    // which bench_r3a.cpp would cut at the budget — on robot two's REAL names, nothing may be cut.
    const auto fits = [](const ms::PromptText& t) {
        CHECK(std::string_view{t.title}.size() <= 36);
        CHECK(std::string_view{t.sub}.size() <= 50);
        CHECK(std::string_view{t.yesBig}.size() <= 14);
        CHECK(std::string_view{t.yesSmall}.size() <= 22);
        CHECK(std::string_view{t.noBig}.size() <= 14);
        CHECK(std::string_view{t.noSmall}.size() <= 22);
        CHECK(std::string_view{t.foot1}.size() <= 50);
        CHECK(std::string_view{t.foot2}.size() <= 50);
    };
    Bench b{robotTwo()};
    fits(b.st->clearText());
    b.build();
    fits(b.st->clearText());  // a later entry's CLEAR (the parts made): its own words
    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        b.select(i);
        b.tick(Bench::held());
        b.tick(Bench::held(false, false, true));
        REQUIRE(b.st->phase() == ms::Phase::Confirm);
        fits(b.st->confirmText());
        b.st->confirm(false, true);
        b.tick(Bench::held());
    }
    b.select(0);  // FIRST (NEXT puts the ceiling back to 3 V), then the ceiling to 12 V: the widest foot line
    b.tick(Bench::held(false, false, false, true));
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, false, true));
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, false, true));
    b.tick(Bench::held());
    REQUIRE(b.st->ceiling() == 12.0);
    b.tick(Bench::held(false, false, true));
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    CHECK(std::string{b.st->confirmText().foot2} == "12 V now, R1 for more");
    fits(b.st->confirmText());
}

TEST_CASE("station 13: nothing resumes by itself — an arrow still held when the controller comes back, or after a "
          "check, is not a press") {
    // BUG CAUGHT: a jog that restarts with nobody pressing anything — the controller reconnecting while L1 and UP
    // are still held, or a hold-to-run jog picking up again once a check (started mid-jog) is over.
    Bench b{robotTwo()};
    b.build();
    b.arm();
    b.tick(Bench::held(true));
    REQUIRE(b.volts(6) == 3.0);
    b.tick(ms::Pad{.connected = false, .deadMan = false, .raise = false, .plus = false, .minus = false, .check = false});
    CHECK(b.volts(6) == 0.0);
    for (int k = 0; k < 5; ++k) b.tick(Bench::held(true));  // back, L1 and UP held all along
    CHECK(b.volts(6) == 0.0);
    CHECK_FALSE(b.st->live(0));
    b.tick(Bench::held());
    b.tick(Bench::held(true));  // a fresh press, seen
    CHECK(b.volts(6) == 3.0);
    // a check, with UP pressed DURING it and held through to its end: after it, UP is not a press
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    REQUIRE(b.st->phase() == ms::Phase::Announcing);
    for (int k = 0; k < 200 && b.st->phase() != ms::Phase::Asking && b.st->phase() != ms::Phase::Idle; ++k) {
        b.motor(6).setPosition(AngleDim{b.motor(6).position().value() + (b.st->phase() == ms::Phase::Jogging ? 0.02 : 0.0)});
        b.tick(Bench::held(true, false, true));
    }
    REQUIRE(b.st->phase() == ms::Phase::Asking);
    b.st->answer(0);
    REQUIRE(b.st->phase() == ms::Phase::Idle);
    for (int k = 0; k < 5; ++k) b.tick(Bench::held(true));  // UP still held: not a press
    CHECK(b.volts(6) == 0.0);
    CHECK_FALSE(b.st->live(0));
}

TEST_CASE("station 13: L1 let go DURING a confirm prompt is the dead-man's release — a claw fired before goes "
          "back to safe at once") {
    // BUG CAUGHT: the prompt blocking the ticks hides the release — a claw fired 'open', a second fire asked,
    // L1 let go while the prompt waits: the claw must not stay open with nobody holding the dead-man.
    Bench b{robotTwo()};
    b.build();
    b.select(2);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    b.st->confirm(true, true);  // fired 'open', L1 held
    REQUIRE(b.dev.fakeDigitalOut(1).commanded());
    b.st->answer(1);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));  // the next fire ('closed') asked, L1 still held
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    b.st->confirm(true, false);  // ... and L1 let go while the prompt waited
    b.drain();
    CHECK_FALSE(b.dev.fakeDigitalOut(1).commanded());  // back to 'closed', its safe position
    CHECK_FALSE(b.st->live(2));
    CHECK(b.logged("L1 released during the prompt: every powered mechanism safe"));
    CHECK(b.logged("NOT fired: 'claw' -- L1 was not held when YES was touched"));
}

// ═══ THE REVIEW ROUND (2026-11-03) — each case names its finding (MECH-PROGRESS.md, R-1 .. R-31) ═══════════

namespace {
/// Every log line `b` wrote, then every mechanism's RECORD lines.
std::vector<std::string> everyLine(Bench& b) {
    b.drain();
    std::vector<std::string> out = b.log;
    for (std::size_t i = 0; i < b.st->count(); ++i) {
        const std::vector<std::string> r = record(b, 9, i);
        out.insert(out.end(), r.begin(), r.end());
    }
    return out;
}
/// The texts the log's pieces were split from: a line holding " (cont.) " continues the one before it.
std::vector<std::string> rejoin(const std::vector<std::string>& lines) {
    std::vector<std::string> out;
    for (const std::string& l : lines) {
        const std::size_t c = l.find(" (cont.) ");
        if (c != std::string::npos && !out.empty()) {
            out.back() += " " + l.substr(c + 9);
        } else {
            out.push_back(l);
        }
    }
    return out;
}
std::size_t holding(const std::vector<std::string>& lines, std::string_view what) {
    return static_cast<std::size_t>(
        std::count_if(lines.begin(), lines.end(), [&](const std::string& l) { return l.find(what) != std::string::npos; }));
}
}  // namespace

TEST_CASE("station 13 re-entry (R-1): a pneumatic's position is READ BACK from its lines — where an earlier entry "
          "left it, not its start; the first FIRE names the OTHER position; a readback matching none is said") {
    // BUG CAUGHT: every entry made a new Station believing each pneumatic sat at its START. Robot y's grip
    // (start 'loose', safe 'shut') left 'shut' by the last entry's dead-man read "position: 'loose'" beside
    // "line A: commanded HIGH"; its first FIRE went to 'shut', where it already was; and "nothing moved" was
    // judged CABLE ELSEWHERE on a correct robot file.
    Bench y{robotY()};
    y.build();
    shulib::hal::fake::FakeDigitalOut& lineA = y.dev.fakeDigitalOut(1);
    CHECK_FALSE(lineA.commanded());  // made at its start: 'loose' = Low
    CHECK(has(panel(y, 6), "position: 'loose'"));
    y.select(6);
    y.tick(Bench::held());
    y.tick(Bench::held(false, false, true));
    REQUIRE(y.st->phase() == ms::Phase::Confirm);
    CHECK(std::string{y.st->confirmText().title} == "FIRE 'grip' -> 'shut'?");
    y.st->confirm(true, true);
    y.drain();
    REQUIRE(lineA.commanded());  // 'shut' = High
    y.st->answer(0);
    y.tick(Bench::loose());  // the dead-man: its SAFE position, 'shut' — the line stays High
    REQUIRE(lineA.commanded());
    y.reenter();
    CHECK(y.logged("'grip' holds 'shut' (an earlier entry left it there), not its start 'loose'"));
    const std::vector<std::string> p = panel(y, 6);
    CHECK(has(p, "line A: commanded HIGH"));
    CHECK(has(p, "position: 'shut'"));
    CHECK_FALSE(has(p, "'loose'"));
    CHECK(has(record(y, 1, 6), "[R3A] mech REC 1 'grip' position 'shut'"));
    y.select(6);
    y.tick(Bench::held());
    y.tick(Bench::held(false, false, true));
    REQUIRE(y.st->phase() == ms::Phase::Confirm);
    CHECK(std::string{y.st->confirmText().title} == "FIRE 'grip' -> 'loose'?");  // the OTHER position
    y.st->confirm(true, true);
    y.drain();
    CHECK_FALSE(lineA.commanded());  // it was told to MOVE: 'loose' = Low
    y.st->answer(1);                 // ... and it went 'loose'
    CHECK(y.st->verdict().finding == mech::DirectionFinding::Ok);
    // a start position, re-entered: nothing said (robot two's claw left at 'closed', its start)
    Bench b{robotTwo()};
    b.build();
    b.reenter();
    CHECK_FALSE(b.logged("an earlier entry left it there"));
    CHECK(has(panel(b, 2), "position: 'closed'"));
    // a readback matching NO declared position — robot x's hood with C Low and D High (it declares LL, HL, HH):
    // the page says so, and its first fire goes to its SAFE position, 'down'
    Bench h{robotX()};
    h.build();
    h.dev.fakeDigitalOut(4).set(true);
    h.reenter();
    CHECK(h.logged("'hood': its lines match NO declared position -- its first fire goes to 'down'"));
    CHECK(has(panel(h, 4), "position: UNKNOWN (no declared one matches)"));
    CHECK(has(record(h, 1, 4), "[R3A] mech REC 1 'hood' position UNKNOWN: its lines match no declared position"));
    h.select(4);
    h.tick(Bench::held());
    h.tick(Bench::held(false, false, true));
    REQUIRE(h.st->phase() == ms::Phase::Confirm);
    CHECK(std::string{h.st->confirmText().title} == "FIRE 'hood' -> 'down'?");
    h.st->confirm(true, true);
    h.drain();
    CHECK_FALSE(h.dev.fakeDigitalOut(3).commanded());
    CHECK_FALSE(h.dev.fakeDigitalOut(4).commanded());
    CHECK(has(panel(h, 4), "position: 'down' (FIRED)"));  // known again
    h.tick(Bench::loose());
    CHECK(has(panel(h, 4), "position: 'down'"));
}

TEST_CASE("station 13's CLEAR (R-9): its YES says what it MOVES — a pneumatic whose START is High strokes when "
          "made; a later entry makes nothing and says so") {
    // BUG CAUGHT: "nothing moves yet" printed under a YES whose build drives a High start line (the cylinder
    // strokes on that touch), or a later entry's CLEAR claiming to drive lines it never drives.
    Bench g{a6_fixture::kGentle};
    ms::PromptText t = g.st->clearText();
    CHECK(std::string{t.yesSmall} == "a cylinder MOVES");
    CHECK(std::string{t.foot1} == "making a line DRIVES it: 'latch' -> 'up' MOVES");
    g.build();
    CHECK(g.dev.digitalOutInitialState(1));  // it did: 'up' = High
    t = g.st->clearText();
    CHECK(std::string{t.yesSmall} == "nothing moves on YES");
    CHECK(std::string{t.foot1} == "made earlier this power cycle: no line is driven");
    Bench b{robotTwo()};  // the claw starts Low: its first YES moves nothing, and names the line it drives
    t = b.st->clearText();
    CHECK(std::string{t.yesSmall} == "nothing moves yet");
    CHECK(std::string{t.foot1} == "making a line DRIVES it: 'claw' -> 'closed'");
    Bench x{robotX()};  // the hood starts LL: no cylinder moves
    CHECK(std::string{x.st->clearText().yesSmall} == "nothing moves yet");
    for (const ms::PromptText& c : {Bench{a6_fixture::kGentle}.st->clearText(), g.st->clearText()}) {
        CHECK(std::string_view{c.yesSmall}.size() <= 22);
        CHECK(std::string_view{c.foot1}.size() <= 50);
    }
}

TEST_CASE("station 13 gate 4 and a power mechanism's maxVolts (R-2): the jog and the check NEVER past it, whatever "
          "the ceiling — and the page, the confirm and RECORD say so") {
    // BUG CAUGHT: R1 taking a mechanism whose team lowered maxVolts to 6 V ("the most any command may ask",
    // which the mechanism service always obeys) to 12 V — on the very page that powers it first.
    Bench g{a6_fixture::kGentle};
    g.build();
    g.tick(Bench::held());
    g.tick(Bench::held(true));
    REQUIRE(g.st->phase() == ms::Phase::Confirm);
    CHECK(std::string{g.st->confirmText().foot2} == "3 V now, its most 6 V");
    g.st->confirm(true, true);
    g.tick(Bench::held());
    char gl[96];
    g.st->limitText(0, gl, sizeof gl);
    CHECK(std::string{gl} == "3 V now (R1 for more)");
    g.tick(Bench::held(true));
    CHECK(g.volts(1) == 3.0);  // under its maxVolts: the ceiling itself
    g.tick(Bench::held());
    for (const double ceiling : {6.0, 9.0, 12.0}) {
        CAPTURE(ceiling);
        g.tick(Bench::held(false, false, false, true));
        g.tick(Bench::held());
        REQUIRE(g.st->ceiling() == ceiling);
        g.tick(Bench::held(true));
        CHECK(g.volts(1) == 6.0);
        CHECK(g.st->commanded(0) == 6.0);
        g.tick(Bench::held());
        g.tick(Bench::held(false, true));
        CHECK(g.volts(1) == -6.0);
        g.tick(Bench::held());
        g.st->limitText(0, gl, sizeof gl);
        CHECK(std::string{gl} == "6 V now (the most)");  // R1 gives it no more past its maxVolts
    }
    CHECK(g.logged("ceiling 9 V -> 12 V; 'gentle' at most 6.0 V (its maxVolts)"));
    CHECK(has(record(g, 1, 0), "[R3A] mech REC 1 'gentle' POWER built, armed, ceiling 12 V (its maxVolts 6.0 V: the jog "
                               "and the check at most 6.0 V)"));
    runCheck(g, {1}, 0.02, 0.3);
    CHECK(g.st->observation().commandVolts == 6.0);
    CHECK(g.logged("CHECK 'gentle': the + jog at 6.0 V"));
    // robot two's pivots keep the default 12 V: nothing capped (the gate-4 case reaches 12 V)
    Bench b{robotTwo()};
    b.build();
    b.st->limitText(0, gl, sizeof gl);
    CHECK(std::string{gl} == "3 V now (R1 for more)");
    CHECK_FALSE(has(record(b, 1, 0), "maxVolts"));
}

TEST_CASE("station 13 gate 4 per mechanism (R-4): NEXT puts the ceiling back to 3 V — one raised for a heavy pivot "
          "never carries over to the next one's FIRST jog or check") {
    // BUG CAUGHT: pivot 6 raised to 6 V (the worksheet's BACK-DRIVEN advice), NEXT, and pivot 7's first-ever
    // jog and direction check at 6 V — gate 4's gentle first motion lost after the first mechanism.
    Bench b{robotTwo()};
    b.build();
    b.arm();
    b.tick(Bench::held(false, false, false, true));
    b.tick(Bench::held());
    REQUIRE(b.st->ceiling() == 6.0);
    b.st->next();
    b.drain();
    CHECK(b.st->ceiling() == 3.0);
    CHECK(b.logged("ceiling 6 V -> 3 V: every mechanism starts at 3 V (R1 raises it)"));
    b.arm();
    b.tick(Bench::held(true));
    CHECK(b.volts(7) == 3.0);  // pivot 7's FIRST jog
    b.tick(Bench::held());
    b.announce();
    CHECK(b.st->observation().commandVolts == 3.0);  // ... and its first check
    b.log.clear();
    b.st->next();  // already at 3 V: nothing said about the ceiling
    b.drain();
    CHECK_FALSE(b.logged("every mechanism starts at"));
}

TEST_CASE("station 13 keeps EVERY mechanism's latest verdict this entry (R-5): RECORD carries each, an abandoned "
          "check leaves the one before, and any finding is the menu's FAIL") {
    // BUG CAUGHT: one verdict per Station — pivot 6 REVERSED, then pivot 7 OK, and the RECORD, the exit summary
    // and the menu's dot all lost pivot 6's finding (the worksheet's normal visit is exactly this path).
    Bench b{robotTwo()};
    b.build();
    b.arm();
    runCheck(b, {6}, 0.02, 0.3);
    REQUIRE(b.st->phase() == ms::Phase::Asking);
    b.st->answer(1);  // pivot 6: the OTHER way
    REQUIRE(b.st->verdict().finding == mech::DirectionFinding::MechanismReversed);
    b.st->next();
    b.drain();
    b.arm();
    runCheck(b, {7}, 0.02, 0.3);
    b.st->answer(0);  // pivot 7: its + way
    CHECK(b.st->verdict().finding == mech::DirectionFinding::Ok);
    REQUIRE(b.st->hasVerdictFor(0));
    CHECK(b.st->verdictFor(0).finding == mech::DirectionFinding::MechanismReversed);
    CHECK(std::string{b.st->answeredFor(0)} == "the other way");
    REQUIRE(b.st->hasVerdictFor(1));
    CHECK(b.st->verdictFor(1).finding == mech::DirectionFinding::Ok);
    CHECK_FALSE(b.st->hasVerdictFor(2));
    CHECK(b.st->anyVerdict());
    CHECK(b.st->anyFinding());  // pivot 6's finding still stands: the FAIL dot
    CHECK(has(record(b, 5, 0), "[R3A] mech REC 5 'claw pivot 6' last check: REVERSED (the other way): 'claw pivot 6' "
                               "moved the OTHER way"));
    CHECK(has(record(b, 5, 1), "[R3A] mech REC 5 'claw pivot 7' last check: OK (its + way): 'claw pivot 7': "
                               "direction confirmed"));
    CHECK_FALSE(has(record(b, 5, 2), "last check"));
    // pivot 6 checked again, OK: ITS latest is OK, and no finding remains
    b.select(0);
    runCheck(b, {6}, 0.02, 0.3);
    b.st->answer(0);
    CHECK(b.st->verdictFor(0).finding == mech::DirectionFinding::Ok);
    CHECK_FALSE(b.st->anyFinding());
    // an ABANDONED check leaves its mechanism's last verdict standing (only "the last check" is cleared)
    b.announce();
    b.tick(Bench::loose());
    CHECK_FALSE(b.st->hasVerdict());
    CHECK(b.st->hasVerdictFor(0));
    CHECK(b.st->verdictFor(0).finding == mech::DirectionFinding::Ok);
    // the claw's fire and answer are its own verdict
    b.select(2);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    b.st->confirm(true, true);
    b.st->answer(2);  // nothing / else moved
    CHECK(b.st->verdictFor(2).finding == mech::DirectionFinding::CableOnAnotherMechanism);
    CHECK(std::string{b.st->answeredFor(2)} == "nothing / else");
    CHECK(b.st->anyFinding());
    // a new entry starts with none
    b.reenter();
    CHECK_FALSE(b.st->anyVerdict());
    CHECK_FALSE(b.st->anyFinding());
}

TEST_CASE("station 13 (R-6): a confirm CLOSED by L1 let go — bench_r3a.cpp's promptHeld answers 'no, not held' the "
          "moment it is let go — grants nothing and makes safe what was powered") {
    // BUG CAUGHT: a release during the prompt arming a motor mechanism, or firing a valve, or leaving a claw fired
    // before the prompt open with nobody on the dead-man.
    Bench b{robotTwo()};
    b.build();
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    b.st->confirm(true, false);  // YES touched after L1 was let go
    b.drain();
    CHECK_FALSE(b.st->armed(0));
    CHECK(b.logged("not armed: 'claw pivot 6' -- L1 was let go during the prompt; hold L1, press again"));
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    CHECK(b.volts(6) == 0.0);  // asked again, never driven
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    b.st->confirm(false, false);  // the prompt closed by the release
    CHECK_FALSE(b.st->armed(0));
    b.select(2);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    b.st->confirm(true, true);  // fired 'open'
    REQUIRE(b.dev.fakeDigitalOut(1).commanded());
    b.st->answer(1);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));  // the next fire ('closed') asked
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    b.log.clear();
    b.st->confirm(false, false);  // ... and closed by the release
    b.drain();
    CHECK_FALSE(b.dev.fakeDigitalOut(1).commanded());
    CHECK_FALSE(b.st->live(2));
    CHECK(b.logged("L1 released during the prompt: every powered mechanism safe"));
    CHECK(b.logged("not fired: 'claw' -> 'closed' -- L1 was let go during the prompt"));
}

TEST_CASE("station 13 direction check starts from REST (R-7, R-30): A during a jog is refused and the jog runs on; "
          "the page says 'ONLY <name> should move' and drives NOTHING for 1.0 s; a motor still moving in its last "
          "0.4 s abandons it") {
    // BUG CAUGHT: A mid-jog reversing a pivot from -3 V to +3 V in one tick with the baseline taken while it
    // moved (the coast judged as the jog: a false BACK-DRIVEN); the "ONLY <name> should move" warning said in
    // the same tick as the jog (gone from the panel before anybody read it); a coast judged as a check; the
    // settle check watching motor 1 only, so a lift's second motor still coasting got its jog anyway.
    Bench b{robotTwo()};
    b.build();
    b.arm();
    b.tick(Bench::held(false, true));  // DOWN: the minus jog
    REQUIRE(b.volts(6) == -3.0);
    b.tick(Bench::held(false, true, true));  // A pressed mid-jog
    CHECK(b.st->phase() == ms::Phase::Idle);
    CHECK(b.volts(6) == -3.0);  // the jog runs on: never reversed in one tick
    CHECK(b.logged("REFUSED A: 'claw pivot 6' is being jogged -- let go of the arrow first; a check starts from rest"));
    b.tick(Bench::held(false, false, true));  // DOWN let go, A still held: safe, and no check
    CHECK(b.volts(6) == 0.0);
    CHECK(b.st->phase() == ms::Phase::Idle);
    // the announcement: said first, nothing driven for exactly kAnnounceMs
    b.tick(Bench::held());
    const std::uint32_t t0 = b.nowMs;
    b.tick(Bench::held(false, false, true));
    REQUIRE(b.st->phase() == ms::Phase::Announcing);
    char a[ms::Station::kTextBytes];
    b.st->announceText(a, sizeof a, t0);
    CHECK(std::string{a} == "CHECK 'claw pivot 6': ONLY 'claw pivot 6' should move -- watch it. A + jog at 3.0 V for "
                            "0.40 s starts in 1.0 s; keep L1 held.");
    b.st->announceText(a, sizeof a, t0 + 500);
    CHECK(std::string_view{a}.find("starts in 0.5 s") != std::string_view::npos);
    char g[96];
    b.st->doNowText(0, Bench::held(), t0, g, sizeof g);
    CHECK(std::string{g} == "Watch it: it moves + in 1.0 s -- keep L1 held");
    for (std::uint32_t k = 1; k < ms::kAnnounceMs / 10; ++k) {
        CHECK(b.volts(6) == 0.0);
        b.tick(Bench::held(false, false, true));
    }
    CHECK(b.st->phase() == ms::Phase::Announcing);
    CHECK(b.volts(6) == 0.0);
    CHECK(b.nowMs == t0 + ms::kAnnounceMs);
    b.tick(Bench::held(false, false, true));  // t0 + 1.00 s: the jog
    CHECK(b.st->phase() == ms::Phase::Jogging);
    CHECK(b.volts(6) == 3.0);
    b.st->doNowText(0, Bench::held(), b.nowMs, g, sizeof g);
    CHECK(std::string{g} == "Checking which way it moves -- keep L1 held");
    // THE SETTLE WINDOW is the announcement's last 0.4 s, sampled at t0 + 0.60 s exactly
    const auto settle = [](double moveRad, std::uint32_t beforeTickAt) {
        Bench c{robotX()};
        c.build();
        c.arm();
        c.tick(Bench::held());
        const std::uint32_t s0 = c.nowMs;
        c.tick(Bench::held(false, false, true));
        while (c.nowMs < s0 + ms::kAnnounceMs + 10 && c.st->phase() == ms::Phase::Announcing) {
            if (c.nowMs == s0 + beforeTickAt) c.motor(1).setPosition(AngleDim{c.motor(1).position().value() + moveRad});
            c.tick(Bench::held(false, false, true));
        }
        return std::pair{c.st->phase(), c.logged("still MOVING when its jog was due")};
    };
    // moved EXACTLY kMinMoveRad inside the window: still moving — abandoned, nothing judged (the equality edge)
    CHECK(settle(ms::kMinMoveRad, ms::kAnnounceMs) == std::pair{ms::Phase::Idle, true});
    CHECK(settle(ms::kMinMoveRad, ms::kAnnounceMs - ms::kCheckJogMs + 10) == std::pair{ms::Phase::Idle, true});
    // just under it: the jog
    CHECK(settle(ms::kMinMoveRad - 1e-6, ms::kAnnounceMs) == std::pair{ms::Phase::Jogging, false});
    // a whole radian BEFORE the window (taken into its sample at t0 + 0.60 s): the jog
    CHECK(settle(1.0, ms::kAnnounceMs - ms::kCheckJogMs) == std::pair{ms::Phase::Jogging, false});
    Bench c{robotX()};
    c.build();
    c.arm();
    c.announce();
    c.motor(1).setPosition(AngleDim{0.5});
    c.tick(Bench::held(false, false, true));
    CHECK(c.logged("CHECK 'arm': still MOVING when its jog was due (motor 1 turned +0.500 rad in the last 0.40 s) -- "
                   "let it settle, then press A again; nothing judged"));
    CHECK(c.volts(1) == 0.0);
    CHECK_FALSE(c.st->hasVerdict());
    // EVERY motor is watched, not motor 1 alone (the verification round's V19): robot x's LIFT (motors on ports 2
    // and 3, 3 mirrored), its SECOND motor alone still moving. Port 3 starts at kMinMoveRad, not 0, so a sample
    // that skipped motor 2 (its stored angle left at 0) would read it as moved in the "under" case too; and the
    // jump to 2 × kMinMoveRad lands the equality edge EXACTLY (checked first, not assumed).
    const double start = ms::kMinMoveRad;
    const double atEdge = 2.0 * ms::kMinMoveRad;
    REQUIRE(atEdge - start == ms::kMinMoveRad);
    const auto settleSecond = [start](double target) {
        Bench l{robotX()};
        l.build();
        l.select(1);  // the lift
        l.arm();
        l.motor(3).setPosition(AngleDim{start});
        l.tick(Bench::held());
        const std::uint32_t s0 = l.nowMs;
        l.tick(Bench::held(false, false, true));
        REQUIRE(l.st->phase() == ms::Phase::Announcing);
        double v2 = 0.0;
        double v3 = 0.0;
        while (l.nowMs < s0 + ms::kAnnounceMs + 10 && l.st->phase() == ms::Phase::Announcing) {
            if (l.nowMs == s0 + ms::kAnnounceMs) l.motor(3).setPosition(AngleDim{target});  // motor 2 ONLY
            v2 = std::max(v2, std::abs(l.volts(2)));
            v3 = std::max(v3, std::abs(l.volts(3)));
            l.tick(Bench::held(false, false, true));
        }
        CHECK(l.motor(2).position().value() == 0.0);  // motor 1 never moved
        return std::tuple{l.st->phase(), l.log, v2 + v3 + std::abs(l.volts(2)) + std::abs(l.volts(3)),
                          l.st->hasVerdict()};
    };
    {  // motor 2 turned EXACTLY kMinMoveRad in the window: abandoned, naming motor 2, nothing driven or judged
        const auto [phase, log, volts, verdict] = settleSecond(atEdge);
        CHECK(phase == ms::Phase::Idle);
        CHECK(volts == 0.0);
        CHECK_FALSE(verdict);
        bool named = false;
        for (const std::string& l : log) {
            if (l.find("CHECK 'lift': still MOVING when its jog was due (motor 2 turned +0.035 rad in the last "
                       "0.40 s) -- let it settle, then press A again; nothing judged") != std::string::npos) {
                named = true;
            }
            CHECK(l.find("(motor 1 turned") == std::string::npos);
        }
        CHECK(named);
    }
    {  // 1 µrad under it: the jog
        const auto [phase, log, volts, verdict] = settleSecond(atEdge - 1e-6);
        CHECK(phase == ms::Phase::Jogging);
        CHECK(volts > 0.0);  // the + jog started on the lift
        for (const std::string& l : log) CHECK(l.find("still MOVING") == std::string::npos);
    }
}

TEST_CASE("station 13's live page PAGES its readings (R-8): a 4-motor lift with an angle sensor and a home switch "
          "has 14 lines (its guide first), more than the panel's kShownLines — its switch and HA-184's question on the "
          "last page, never cut") {
    // BUG CAUGHT: bench_r3a.cpp asking panelLines() for its 7 panel lines only, so such a lift's home switch and
    // its "RECORD it pressed, released, and cable OUT" were cut off the page without a word.
    CHECK(ms::readingsPage(7, 7, 0).pages == 1);  // n == shown: one page, all of them, no footer
    CHECK(ms::readingsPage(7, 7, 0).count == 7);
    CHECK(ms::readingsPage(7, 7, 3).first == 0);
    const ms::ReadingsPage p0 = ms::readingsPage(8, 7, 0);  // ONE more: two pages of six and a footer
    CHECK(p0.pages == 2);
    CHECK(p0.first == 0);
    CHECK(p0.count == 6);
    const ms::ReadingsPage p1 = ms::readingsPage(8, 7, 1);
    CHECK(p1.page == 1);
    CHECK(p1.first == 6);
    CHECK(p1.count == 2);
    CHECK(ms::readingsPage(8, 7, 2).page == 0);  // the tap wraps
    const ms::ReadingsPage p3 = ms::readingsPage(13, 7, 2);
    CHECK(p3.pages == 3);
    CHECK(p3.first == 12);
    CHECK(p3.count == 1);
    CHECK(ms::readingsPage(3, 1, 5).count == 1);  // no room for a footer: the first line, unpaged
    char f[96];
    ms::pageFooter(f, sizeof f, p1);
    CHECK(std::string{f} == "-- page 2/2: TAP the readings for more --");
    CHECK(std::string_view{f}.size() <= ms::Station::kPanelColumns);
    Bench b{a6_fixture::kBigLift};
    b.build();
    REQUIRE(b.st->built(0));
    const std::vector<std::string> all = panel(b, 0);
    REQUIRE(all.size() == 14);  // S13UX D3: the guide first (a heading, its bottom stop, level 'floor', its top stop)
    CHECK(all[0] == "To measure it, RECORD at each step (1 of 3):");
    CHECK(all[8] == "output +0.00 deg since power-up (+0.00 in)");
    CHECK(all[9] == "angle p10: raw +0.0deg, as mounted +0.0deg");
    CHECK(all[12] == "home sw B: reads 0 = released");
    CHECK(all[13] == "  RECORD it pressed, released, and cable OUT");
    for (const std::string& l : all) CHECK(l.size() <= ms::Station::kPanelColumns);
    const ms::ReadingsPage first = ms::readingsPage(all.size(), ms::Station::kShownLines, 0);
    const ms::ReadingsPage second = ms::readingsPage(all.size(), ms::Station::kShownLines, 1);
    const ms::ReadingsPage third = ms::readingsPage(all.size(), ms::Station::kShownLines, 2);
    CHECK(first.pages == 3);
    CHECK(first.count + 1 == ms::Station::kShownLines);  // the panel's last line is the footer
    CHECK(second.first == first.count);
    CHECK(third.first + third.count == all.size());  // the switch and its question: the last page
    CHECK(all.size() <= ms::Station::kMaxPanelLines);
}

TEST_CASE("station 13 readings (R-10, R-29): the angle sensor's line names the number absoluteSensor(at, reads) "
          "TAKES — the RAW reading in [0, 360), never the 'as mounted' one") {
    // BUG CAUGHT: a page (and guide) leaving "raw" and "as mounted" side by side with nothing to say which one the
    // declaration takes — a team with a Reversed sensor typing the mounted number gets a refusal or a zero off by
    // twice the reading.
    CHECK(ms::absoluteReadsDeg(90.0) == 90.0);
    CHECK(ms::absoluteReadsDeg(-90.0) == 270.0);
    CHECK(ms::absoluteReadsDeg(450.0) == 90.0);
    CHECK(ms::absoluteReadsDeg(359.96) == 0.0);  // rounds to 360.0: a whole turn is 0
    CHECK(ms::absoluteReadsDeg(359.94) == 359.9);
    CHECK(ms::absoluteReadsDeg(-0.04) == 0.0);
    CHECK_FALSE(std::signbit(ms::absoluteReadsDeg(-0.04)));  // never "-0.0"
    CHECK(std::isnan(ms::absoluteReadsDeg(std::numeric_limits<double>::quiet_NaN())));
    // robot y's wrist declares absoluteSensor(0, 180) with its sensor mounted REVERSED; at its home the sensor
    // reads 180 RAW (test/mechanism_position_test.cpp homes it there through the mechanism service). The page
    // says 180.0 — the declaration's own number — while "as mounted" says -180.
    REQUIRE(mech_fixture::robot_y::wrist.spec().position.absoluteSensorAtZeroDeg == 180.0);
    Bench y{robotY()};
    y.build();
    y.dev.fakeRotation(10).setPosition(AngleDim{kPi});
    std::vector<std::string> p = panel(y, 0);
    CHECK(has(p, "angle p10: raw +180.0deg, as mounted -180.0deg"));
    CHECK(has(p, "  absoluteSensor(at, reads): reads = 180.0"));
    y.dev.fakeRotation(10).setPosition(AngleDim{-kPi / 2.0});  // raw -90: reads 270, as mounted +90
    p = panel(y, 0);
    CHECK(has(p, "  absoluteSensor(at, reads): reads = 270.0"));
    CHECK(has(record(y, 1, 0), "as mounted +90.00 deg; absoluteSensor(at, reads) takes reads = 270.0 (the RAW "
                               "reading, 0..360)"));
}

TEST_CASE("station 13 readings (R-25): a PIECE switch is a limit switch too — the page asks for it pressed, released "
          "and unplugged, and RECORD carries its raw level and pressedLevel (HA-184's settle)") {
    // BUG CAUGHT: HA-184's three readings asked of a HOME switch only, while a piece switch (robot y's catapult)
    // showed one level with no question and a RECORD line without its pressedLevel.
    Bench y{robotY()};  // the catapult's limit switch on 3-wire E
    y.build();
    y.dev.fakeDigitalIn(5).setState(true);
    const std::vector<std::string> p = panel(y, 5);
    CHECK(has(p, "piece sw E: reads 1 = PRESSED"));
    CHECK(has(p, "  RECORD it pressed, released, and cable OUT"));
    CHECK(has(record(y, 2, 5), "[R3A] mech REC 2 'catapult' piece switch 3-wire E reads 1 (pressedLevel 1; HA-184), "
                               "read faults 0 -- RECORD it pressed, released and with its cable OUT"));
    y.dev.fakeDigitalIn(5).setState(false);
    CHECK(has(record(y, 3, 5), "piece switch 3-wire E reads 0 (pressedLevel 1; HA-184)"));
    // a distance or optical piece sensor is no switch: nothing asked
    CHECK_FALSE(has(panel(y, 4), "cable OUT"));
    Bench x{robotX()};
    x.build();
    CHECK_FALSE(has(panel(x, 3), "cable OUT"));
    CHECK(has(record(x, 1, 3), "[R3A] mech REC 1 'intake' piece p9: "));
}

TEST_CASE("station 13's log (R-3): EVERY line is ONE serial write — 255 characters at most, each tagged '[R3A] mech' "
          "— on robot two's REAL table, each verdict's sentence whole across its pieces, the path whole on one line") {
    // BUG CAUGHT: a CHECK or REC line carrying the judgement's sentence after its prefix — 8 of robot two's over
    // 255 characters — went out as TWO serial writes, the second '... ' with no [R3A] tag, so the worksheet's
    // `grep '\[R3A\] mech'` dropped the very part that names the line to edit. The host's path to the real table
    // is LONGER than the brain's ./src/robots/mechanism_table.hpp, so the host is the harder case.
    const mech::MechanismTable& real = mech_fixture::robot_two::kRobotTwoMechanisms;  // the claw robot (ROB1-BT)
    std::size_t pieces = 0;
    std::size_t audited = 0;
    const auto audit = [&](Bench& b) {
        REQUIRE(b.st->hasVerdict());
        const std::string fix = b.st->verdict().fix;
        const std::vector<std::string> lines = everyLine(b);
        for (const std::string& l : lines) {
            CAPTURE(l);
            CHECK(l.size() <= ms::Station::kLineChars);
            CHECK(l.starts_with("[R3A] mech "));
            if (l.find(" (cont.) ") != std::string::npos) ++pieces;
        }
        CHECK(holding(rejoin(lines), fix) >= 2);  // whole: in its CHECK line and in its RECORD line
        const std::size_t at = fix.find("mechanism_table.hpp:");
        if (at != std::string::npos) {  // the robot-file path, whole on ONE line
            const std::size_t from = fix.rfind(' ', at) + 1;
            const std::string path = fix.substr(from, fix.find_first_of(" ,", at) - from);
            CAPTURE(path);
            CHECK(holding(lines, path) >= 2);
        }
        ++audited;
    };
    const auto pivot = [&](double stepRad, double amps, int answer) {
        Bench b{real};
        b.build();
        b.arm();
        runCheck(b, {6}, stepRad, amps);
        if (answer >= 0) b.st->answer(answer);
        audit(b);
    };
    pivot(0.02, 0.3, 0);   // OK
    pivot(0.02, 0.3, 1);   // REVERSED
    pivot(0.02, 0.3, 2);   // CABLE ELSEWHERE
    pivot(-0.02, 0.3, -1);  // BACK-DRIVEN
    pivot(0.0, 0.1, -1);   // DID NOT MOVE
    pivot(0.0, 1.0, -1);   // FIGHT OR JAM
    for (const int answer : {0, 1, 2}) {  // the claw: REVERSED, OK, CABLE ELSEWHERE
        Bench b{real};
        b.build();
        b.select(2);
        b.tick(Bench::held());
        b.tick(Bench::held(false, false, true));
        b.st->confirm(true, true);
        b.st->answer(answer);
        audit(b);
    }
    CHECK(audited == 9);
    CHECK(pieces > 0);  // the long ones WERE split: this case reaches the path it pins
    // the logPieces rule itself, at its edges
    std::vector<std::string> out;
    const auto into = [&](const char* l) { out.emplace_back(l); };
    const std::string exact(255, 'x');
    ms::logPieces(exact.c_str(), "[R3A] mech (cont.) ", 255, into);
    CHECK(out.size() == 1);  // EXACTLY 255: one line
    out.clear();
    const std::string over = std::string(200, 'a') + " " + std::string(55, 'b');  // 256
    ms::logPieces(over.c_str(), "[R3A] mech (cont.) ", 255, into);
    REQUIRE(out.size() == 2);
    CHECK(out[0] == std::string(200, 'a'));
    CHECK(out[1] == "[R3A] mech (cont.) " + std::string(55, 'b'));
    out.clear();
    const std::string word(300, 'w');  // a word longer than a line is cut, nothing lost
    ms::logPieces(word.c_str(), "[R3A] mech (cont.) ", 255, into);
    REQUIRE(out.size() == 2);
    CHECK(out[0].size() == 255);
    CHECK(out[1] == "[R3A] mech (cont.) " + std::string(45, 'w'));
    out.clear();
    ms::logPieces(nullptr, "[R3A] mech (cont.) ", 255, into);
    CHECK(out.empty());
}

TEST_CASE("station 13 gate 5's hold-to-run (R-11): letting go of the ARROW with L1 still held makes it safe THAT "
          "tick") {
    // BUG CAUGHT (the review's mutant R1b, which survived the whole suite): a pivot that keeps running at the
    // ceiling after UP is let go — until L1 is let go or the 3 s cut — while the worksheet says it stops.
    Bench b{robotTwo()};
    b.build();
    b.arm();
    b.tick(Bench::held(true));
    REQUIRE(b.volts(6) == 3.0);
    b.motor(6).setBrakeMode(BrakeMode::Coast);
    b.tick(Bench::held());  // UP let go, L1 held
    CHECK(b.volts(6) == 0.0);
    CHECK_FALSE(b.st->live(0));
    CHECK(b.motor(6).brakeMode() == BrakeMode::Hold);
    CHECK(b.logged("[R3A] mech 'claw pivot 6': jog released -- safe"));
    // EXACTLY that line: its queue slot last held the longer ARMED line, and nothing of it may show through
    CHECK(b.log.back() == "[R3A] mech 'claw pivot 6': jog released -- safe");
    for (int k = 0; k < 10; ++k) b.tick(Bench::held());
    CHECK(b.volts(6) == 0.0);
    b.tick(Bench::held(false, true));
    REQUIRE(b.volts(6) == -3.0);
    b.tick(Bench::held());  // DOWN let go
    CHECK(b.volts(6) == 0.0);
    CHECK_FALSE(b.st->live(0));
}

TEST_CASE("station 13 (R-12): nothing is powered while the direction check's QUESTION waits — a FRESH press of UP, "
          "DOWN or A does nothing, on a motor mechanism and on the claw") {
    // BUG CAUGHT (the review's mutant R2): the question's guard gone — a fresh UP during a motor check's question
    // jogs the mechanism with the question on screen; a fresh A on the claw's replaces it with a new FIRE.
    Bench b{robotX()};
    b.build();
    b.arm();
    runCheck(b, {1}, 0.02, 0.3);
    REQUIRE(b.st->phase() == ms::Phase::Asking);
    for (const ms::Pad press : {Bench::held(true), Bench::held(false, true), Bench::held(false, false, true)}) {
        b.tick(Bench::held());
        b.tick(press);
        CHECK(b.st->phase() == ms::Phase::Asking);
        CHECK(b.volts(1) == 0.0);
        CHECK_FALSE(b.st->live(0));
    }
    Bench c{robotTwo()};
    c.build();
    c.select(2);
    c.tick(Bench::held());
    c.tick(Bench::held(false, false, true));
    c.st->confirm(true, true);
    REQUIRE(c.st->phase() == ms::Phase::Asking);
    const int sets = c.dev.fakeDigitalOut(1).setCount();
    for (const ms::Pad press : {Bench::held(true), Bench::held(false, false, true)}) {
        c.tick(Bench::held());
        c.tick(press);
        CHECK(c.st->phase() == ms::Phase::Asking);
    }
    CHECK(c.dev.fakeDigitalOut(1).setCount() == sets);
}

TEST_CASE("station 13 direction check's observation (R-13): measured from the jog's START (not power-up), the angle "
          "sensor from ITS start, and the PEAK current, not the last") {
    // BUG CAUGHT (the review's mutants R3, R4, R5): every check had started at 0 with a constant current. On a
    // robot the pivot is swung by hand and jogged first (a still pivot read as "moved"), an absolute sensor never
    // starts at 0 (its whole reading taken as the change), and a jam spikes then eases ("did not move").
    {  // R4: the motor at 1.0 rad before the check, still in it, 0.1 A: DID NOT MOVE, a change of exactly 0
        Bench b{robotX()};
        b.build();
        b.motor(1).setPosition(AngleDim{1.0});
        b.arm();
        runCheck(b, {1}, 0.0, 0.1);
        REQUIRE(b.st->hasVerdict());
        CHECK(b.st->verdict().finding == mech::DirectionFinding::DidNotMove);
        CHECK(b.st->observation().motorDeltaRad[0] == 0.0);
    }
    {  // R5: robot y's wrist sensor at 1.0 rad first; its raw falling 0.2 rad (Reversed: +0.2 as mounted) with the motor
        Bench b{robotY()};
        b.build();
        b.dev.fakeRotation(10).setPosition(AngleDim{1.0});
        b.arm();
        b.motor(1).setCurrent(Current{0.3});
        b.announce();
        b.tick(Bench::held(false, false, true));
        for (int k = 0; k < 60 && b.st->phase() == ms::Phase::Jogging; ++k) {
            b.motor(1).setPosition(AngleDim{b.motor(1).position().value() + 0.02});
            b.dev.fakeRotation(10).setPosition(AngleDim{b.dev.fakeRotation(10).position().value() - 0.005});
            b.tick(Bench::held(false, false, true));
        }
        CHECK(b.st->phase() == ms::Phase::Asking);
        CHECK(b.st->observation().sensorDeltaRad > 0.0);
        CHECK(b.st->observation().sensorDeltaRad < 0.5);
        CHECK(b.st->observation().sensorDeltaRad == doctest::Approx(0.2).epsilon(1e-9));
    }
    {  // R3: 1.0 A for the jog's first 10 ticks, then 0 A, still: FIGHT OR JAM on the PEAK
        Bench b{robotX()};
        b.build();
        b.arm();
        b.motor(1).setCurrent(Current{1.0});
        b.announce();
        b.tick(Bench::held(false, false, true));
        for (int k = 0; k < 60 && b.st->phase() == ms::Phase::Jogging; ++k) {
            if (k == 9) b.motor(1).setCurrent(Current{0.0});
            b.tick(Bench::held(false, false, true));
        }
        REQUIRE(b.st->hasVerdict());
        CHECK(b.st->observation().motorPeakAmps[0] == 1.0);
        CHECK(b.st->verdict().finding == mech::DirectionFinding::FightOrJam);
    }
}

TEST_CASE("station 13 (R-14): nothing resumes by itself through NEXT — a held arrow never re-drives the mechanism "
          "NEXT lands on, alone on its list or armed earlier") {
    // BUG CAUGHT (the review's mutant R19): NEXT not ending the hold-to-run — on a one-mechanism list NEXT wraps to
    // the same arm and the next tick drives it again at the ceiling with UP still held and no fresh press.
    static constexpr mech::MechanismTable kArmOnly = mech::mechanisms(mech::wire(mech_fixture::robot_x::arm, mech::motors(1)));
    Bench a{kArmOnly};
    a.build();
    a.arm();
    a.tick(Bench::held(true));
    REQUIRE(a.volts(1) == 3.0);
    a.st->next();  // wraps to itself
    a.drain();
    REQUIRE(a.st->selected() == 0);
    CHECK(a.volts(1) == 0.0);
    for (int k = 0; k < 5; ++k) a.tick(Bench::held(true));
    CHECK(a.volts(1) == 0.0);
    CHECK_FALSE(a.st->live(0));
    Bench b{robotTwo()};  // both pivots armed; UP held on 6 through NEXT onto 7
    b.build();
    b.select(1);
    b.arm();
    b.select(0);
    b.arm();
    b.tick(Bench::held(true));
    REQUIRE(b.volts(6) == 3.0);
    b.st->next();
    b.drain();
    REQUIRE(b.st->selected() == 1);
    REQUIRE(b.st->armed(1));
    for (int k = 0; k < 5; ++k) b.tick(Bench::held(true));
    CHECK(b.volts(7) == 0.0);
    CHECK(b.volts(6) == 0.0);
    b.tick(Bench::held());
    b.tick(Bench::held(true));  // a FRESH press: now it moves
    CHECK(b.volts(7) == 3.0);
}

TEST_CASE("station 13's answer buttons (R-15): each slot's LABEL is the answer that slot is JUDGED as") {
    // BUG CAUGHT (the review's mutant R11): two labels swapped — 'ITS + WAY' drawn on the button judged "not moved /
    // else moved", so a correct pivot answered truthfully gets CABLE ELSEWHERE naming its wire line.
    const struct {
        int slot;
        const char* line1;
        const char* line2;
        mech::DirectionFinding finding;
    } kSlots[] = {{0, "ITS + WAY", "", mech::DirectionFinding::Ok},
                  {1, "THE OTHER", "WAY", mech::DirectionFinding::MechanismReversed},
                  {2, "NOT MOVED /", "ELSE MOVED", mech::DirectionFinding::CableOnAnotherMechanism}};
    for (const auto& s : kSlots) {
        CAPTURE(s.slot);
        Bench b{robotX()};
        b.build();
        b.arm();
        runCheck(b, {1}, 0.02, 0.3);
        REQUIRE(b.st->answerCount() == 3);
        char l1[16], l2[16];
        b.st->answerWords(s.slot, l1, l2);
        CHECK(std::string{l1} == s.line1);
        CHECK(std::string{l2} == s.line2);
        b.st->answer(s.slot);
        CHECK(b.st->verdict().finding == s.finding);
    }
}

TEST_CASE("station 13 gate 5's over-current (R-16): ANY one motor — a mirrored lift's SECOND — above 2.4 A for 250 "
          "CONSECUTIVE ms; a dip starts the window over") {
    // BUG CAUGHT (the review's mutants R7, R6): only motor 1 watched (a lift's second motor at 3 A never cut), or
    // short spikes added up across a dip into a false cut.
    Bench b{robotX()};
    b.build();
    b.select(1);  // the lift: ports 2 and 3
    b.arm();
    b.motor(3).setCurrent(Current{2.41});
    b.tick(Bench::held(true));
    for (int k = 2; k <= 24; ++k) b.tick(Bench::held(true));
    CHECK_FALSE(b.st->cut(1));
    b.tick(Bench::held(true));  // the 25th
    REQUIRE(b.st->cut(1));
    CHECK(std::string{b.st->cutWhy(1)} == "motor 2 (port 3) above 2.4 A for 250 ms");
    Bench c{robotTwo()};  // 24 ticks above, one below, 24 above: never 25 in a row
    c.build();
    c.arm();
    c.motor(6).setCurrent(Current{2.41});
    c.tick(Bench::held(true));
    for (int k = 2; k <= 24; ++k) c.tick(Bench::held(true));
    c.motor(6).setCurrent(Current{0.0});
    c.tick(Bench::held(true));
    c.motor(6).setCurrent(Current{2.41});
    for (int k = 1; k <= 24; ++k) c.tick(Bench::held(true));
    CHECK_FALSE(c.st->cut(0));
    c.tick(Bench::held(true));  // the 25th in a row since the dip
    CHECK(c.st->cut(0));
}

TEST_CASE("station 13 readings (R-18): each motor's temperature is read on ITS OWN port, on the panel and in RECORD") {
    // BUG CAUGHT (the review's mutants R13, R14): the role number passed for the port — on the brain another port's
    // read ('unknown', or another motor's number).
    Bench b{robotTwo()};
    b.build();
    const auto only6 = [](std::uint8_t port) { return port == 6 ? 31.0 : std::numeric_limits<double>::infinity(); };
    char out[ms::Station::kMaxPanelLines][ms::Station::kPanelBytes]{};
    std::size_t n = b.st->panelLines(0, only6, out, ms::Station::kMaxPanelLines);
    REQUIRE(n >= 1);
    CHECK(std::string{out[1]} == "motor 1 (port 6) +0.0 deg 0.00 A 31C");  // after its guide line (S13UX D3)
    n = b.st->panelLines(1, only6, out, ms::Station::kMaxPanelLines);
    REQUIRE(n >= 1);
    CHECK(std::string{out[1]} == "motor 1 (port 7) +0.0 deg 0.00 A unknown");
    std::vector<std::string> rec;
    b.st->record(1, 0, only6, [&](const char* l) { rec.emplace_back(l); });
    CHECK(has(rec, "motor 1 port 6 pos +0.000 rad (+0.0 deg) vel +0.000 rad/s cur 0.000 A temp 31C"));
}

TEST_CASE("station 13 RECORD (R-19): the angle sensor's line, a CUT and a pneumatic's levels, each in full") {
    // BUG CAUGHT (the review's mutants R15, R17, R18): 'as mounted' ignoring the mounting in the line HA-186's
    // settle is read from, a RECORD that drops the cut and why, or a pneumatic line's level inverted.
    Bench y{robotY()};
    y.build();
    y.dev.fakeRotation(10).setPosition(AngleDim{kPi / 2.0});
    CHECK(has(record(y, 1, 0), "[R3A] mech REC 1 'wrist' angle sensor port 10 raw +1.5708 rad (+90.00 deg) as mounted "
                               "-90.00 deg; absoluteSensor(at, reads) takes reads = 90.0"));
    Bench b{robotTwo()};
    b.build();
    b.arm();
    for (int k = 0; k <= 300; ++k) b.tick(Bench::held(true));
    REQUIRE(b.st->cut(0));
    std::vector<std::string> rec = record(b, 2, 0);
    REQUIRE_FALSE(rec.empty());
    CHECK(rec[0] == "[R3A] mech REC 2 'claw pivot 6' POWER built, armed, ceiling 3 V, CUT: a jog held for its stated "
                    "maximum, 3.0 s (HA-182)");
    rec = record(b, 3, 2);
    CHECK(has(rec, "[R3A] mech REC 3 'claw' line 1 3-wire A commanded LOW"));
    CHECK(has(rec, "[R3A] mech REC 3 'claw' position 'closed'"));
    b.select(2);
    b.tick(Bench::held());
    b.tick(Bench::held(false, false, true));
    b.st->confirm(true, true);
    rec = record(b, 4, 2);
    CHECK(has(rec, "[R3A] mech REC 4 'claw' line 1 3-wire A commanded HIGH"));
    CHECK(has(rec, "[R3A] mech REC 4 'claw' position 'open' (fired, L1 held)"));
}

TEST_CASE("station 13 readings (R-20): a sensor that FAILED during an earlier entry is not blamed on the next entry's "
          "first paint — the read-fault watch is seeded at the CLEAR") {
    // BUG CAUGHT (the review's mutants R9, R10): the parts — and their fault counts — live for the power cycle, so
    // an unseeded watch reads every earlier failure as "moved" on the first paint: NOT ANSWERING for a switch or a
    // sensor answering now.
    Bench b{robotX()};
    b.build();
    b.dev.fakeLineReadFaults(2).failing(true);  // the lift's home switch
    b.dev.fakeReadFaults(9).failing(true);      // the intake's distance sensor
    b.dev.fakeDistance(9).setDistance(Length{10.0});
    b.dev.fakeDistance(9).setConfidence(1.0);
    REQUIRE(has(panel(b, 1), "home sw B: NOT ANSWERING (a read failed)"));
    REQUIRE(has(panel(b, 3), "-> NOT ANSWERING"));
    b.dev.fakeLineReadFaults(2).failing(false);
    b.dev.fakeReadFaults(9).failing(false);
    b.reenter();
    CHECK(has(panel(b, 1), "home sw B: reads 0 = released"));  // the FIRST paint of the new entry
    CHECK(has(panel(b, 3), "-> no piece"));
}

TEST_CASE("station 13 direction check (R-21): 'did not move' is pinned from BELOW too — 0.0349 rad is DID NOT MOVE "
          "(HA-182's 0.035 rad)") {
    // BUG CAUGHT (the review's mutant R8): the threshold lowered to 0.030 rad, unseen by the equality-edge case alone.
    Bench b{robotX()};
    b.build();
    b.arm();
    b.motor(1).setCurrent(Current{0.1});
    b.announce();
    b.tick(Bench::held(false, false, true));
    for (int k = 0; k < 60 && b.st->phase() == ms::Phase::Jogging; ++k) {
        b.motor(1).setPosition(AngleDim{0.0349});
        b.tick(Bench::held(false, false, true));
    }
    REQUIRE(b.st->hasVerdict());
    CHECK(b.st->observation().motorDeltaRad[0] == 0.0349);
    CHECK(b.st->verdict().finding == mech::DirectionFinding::DidNotMove);
}

TEST_CASE("station 13 gate 5's stall window (R-22): its edge is exact at ANY clock — a press at 70 ms cuts at 570 ms") {
    // BUG CAUGHT (the review's mutant R12): the stall clock in absolute seconds — (0.57 - 0.07) is just under 0.5 in
    // floating point, so the cut came a tick late; every other case started at 100000 ms, where it happens not to.
    Bench b{robotX(), 40};
    b.build();
    b.arm();  // ticks at 40, 50 and 60
    b.motor(1).setCurrent(Current{2.0});
    b.motor(1).setVelocity(AngularVelocity{0.0});
    REQUIRE(b.nowMs == 70);
    b.tick(Bench::held(true));  // 70 ms: the window opens
    for (int k = 1; k <= 49; ++k) b.tick(Bench::held(true));
    CHECK_FALSE(b.st->cut(0));
    REQUIRE(b.nowMs == 570);
    b.tick(Bench::held(true));  // 570 ms: EXACTLY its lasting
    CHECK(b.st->cut(0));
}

TEST_CASE("station 13's verdict on the panel: its finding, its sentence, and 'nothing is saved'") {
    // BUG CAUGHT: the verdict's panel text drifting from the judgement it shows.
    char v[ms::Station::kTextBytes];
    Bench c{robotX()};
    c.build();
    c.arm();
    runCheck(c, {1}, 0.02, 0.3);
    c.st->answer(1);
    c.st->verdictPanelText(v, sizeof v);
    CHECK(std::string_view{v}.starts_with("REVERSED: 'arm' moved the OTHER way"));
    CHECK(std::string_view{v}.ends_with(" -- edit the robot file, rebuild, upload (nothing is saved)"));
}

// ═══ S13UX — station 13 made plain (2026-11-19): D2 the "what to do now" line, D3 the lift's guide and RECORD's
// big line, D4 the jog-time cut that L1 re-arms, D5 the declared current cap ═══════════════════════════════════

namespace {
/// D2's line for mechanism `i` at `now` (the bench's clock when 0).
std::string doNow(Bench& b, std::size_t i, const ms::Pad& pad, std::uint32_t now = 0) {
    char out[96];
    b.st->doNowText(i, pad, now != 0 ? now : b.nowMs, out, sizeof out);
    return out;
}
/// Hold UP with L1 for `ms` (the press included), then let go of UP (L1 held).
void jogFor(Bench& b, std::uint32_t ms, bool up = true) {
    for (std::uint32_t t = 0; t < ms; t += static_cast<std::uint32_t>(ms::kTickMs)) b.tick(Bench::held(up, !up));
    b.tick(Bench::held());
}
/// RECORD as bench_r3a.cpp does it: every mechanism's lines, then recorded().
std::vector<std::string> recordAll(Bench& b, int rec) {
    std::vector<std::string> out;
    for (std::size_t i = 0; i < b.st->count(); ++i) {
        b.st->record(rec, i, [](std::uint8_t) { return 30.0; }, [&](const char* l) { out.emplace_back(l); });
    }
    b.st->recorded(rec);
    b.drain();
    return out;
}
const mech::MechanismTable& robotOne() { return shulib::bench::robot_one::kRobotOneMechanisms; }
/// The latest `mech guide:` log line ("" when none).
std::string lastGuide(const Bench& b) {
    for (auto l = b.log.rbegin(); l != b.log.rend(); ++l) {
        if (l->find("mech guide:") != std::string::npos) return *l;
    }
    return "";
}
}  // namespace

TEST_CASE("S13UX D2: ONE 'what to do now' line for every state — no link, a field link, L1 up, L1 held, a confirm, "
          "a jog with its seconds, a jog that did not move, each cut — every line within the panel's 50 columns") {
    // BUG CAUGHT (robot one, 2026-11-17): a page that never said the controller was the problem, that L1 must be
    // held, that a 3 V jog did not move, or what a cut needs — "I'm confused what it's asking me".
    std::vector<std::string> said;
    const auto line = [&](Bench& b, std::size_t i, const ms::Pad& pad, std::uint32_t now = 0) {
        said.push_back(doNow(b, i, pad, now));
        return said.back();
    };
    Bench u{robotTwo()};  // nothing made yet
    CHECK(line(u, 0, Bench::held()) == "Answer the CLEAR question first");
    Bench b{robotTwo()};
    b.dev.refusePort(7);
    b.build();
    CHECK(line(b, 0, ms::Pad{}) == "Pair the controller to the brain: it has no link");
    // a field link: the station still WORKS (a DISABLED one ends it — the boot's state screen), so it is said only
    // when nothing else is: never over "hold L1", a jog, a confirm or a cut (each checked below)
    ms::Pad field = Bench::held();
    field.field = true;
    CHECK(line(b, 0, field) == "Controller says FIELD: unplug the field cable");
    field.deadMan = false;
    CHECK(line(b, 0, field) == "Hold L1 to power the claw pivot 6");
    CHECK(line(b, 0, ms::Pad{.field = true}) == "Pair the controller to the brain: it has no link");
    CHECK(line(b, 1, Bench::held()) == "Not found on the brain: check its cable, or NEXT");
    CHECK(line(b, 0, Bench::loose()) == "Hold L1 to power the claw pivot 6");
    CHECK(line(b, 0, Bench::held()) == "UP / DOWN jog it; A checks which way (L1 held)");
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    REQUIRE(b.st->phase() == ms::Phase::Confirm);
    CHECK(line(b, 0, Bench::held(true)) == "Touch YES on the screen -- keep L1 held");
    field.deadMan = true;
    CHECK(line(b, 0, field) == "Touch YES on the screen -- keep L1 held");
    b.st->confirm(true, true);
    // UP still held through the YES moves nothing (only a fresh press does): the line says so
    CHECK(line(b, 0, Bench::held(true)) == "Let go of UP, then press it again");
    CHECK(line(b, 0, Bench::held(false, true)) == "Let go of DOWN, then press it again");
    b.tick(Bench::held());
    const std::uint32_t t = b.nowMs;
    b.tick(Bench::held(true));  // the press: the jog's clock starts
    CHECK(line(b, 0, Bench::held(true), t + 1800) == "Moving UP -- 1.8 s of 3 s (let go to stop)");
    field.plus = true;
    CHECK(line(b, 0, field, t + 1800) == "Moving UP -- 1.8 s of 3 s (let go to stop)");
    field.plus = false;
    CHECK(line(b, 0, Bench::held(true), t + 1899) == "Moving UP -- 1.8 s of 3 s (let go to stop)");
    b.tick(Bench::held());
    const std::uint32_t d = b.nowMs;
    b.tick(Bench::held(false, true));
    CHECK(line(b, 0, Bench::held(false, true), d + 400) == "Moving DOWN -- 0.4 s of 3 s (let go to stop)");
    b.tick(Bench::held());
    // the pneumatic claw: its own words
    b.select(2);
    CHECK(line(b, 2, Bench::loose()) == "Hold L1 to fire the claw");
    CHECK(line(b, 2, Bench::held()) == "UP fires it to 'open' (keep L1 held)");
    CHECK(line(b, 2, Bench::held(false, true)) == "DOWN does nothing here: UP fires it");  // it fires on UP only
    CHECK(line(b, 2, Bench::held(true, true)) == "Let go of UP, then press it again");
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    b.st->confirm(true, true);
    b.drain();
    REQUIRE(b.st->phase() == ms::Phase::Asking);
    CHECK(line(b, 2, Bench::held()) == "Where did it go? Touch your answer below");
    b.st->answer(1);
    CHECK(line(b, 2, Bench::held()) == "UP fires it again; let go of L1 to make it safe");
    // a motor check's question, the announcement, a lift's own words
    Bench x{robotX()};
    x.build();
    x.select(1);
    x.arm();
    CHECK(line(x, 1, Bench::held()) == "UP raises, DOWN lowers; A checks its way (L1 held)");
    x.select(0);
    x.arm();
    x.tick(Bench::held());
    const std::uint32_t a0 = x.nowMs;
    x.tick(Bench::held(false, false, true));
    CHECK(line(x, 0, Bench::held(), a0 + 400) == "Watch it: it moves + in 0.6 s -- keep L1 held");
    for (int k = 0; k < 99; ++k) x.tick(Bench::held(false, false, true));
    x.tick(Bench::held(false, false, true));
    REQUIRE(x.st->phase() == ms::Phase::Jogging);
    CHECK(line(x, 0, Bench::held()) == "Checking which way it moves -- keep L1 held");
    for (int k = 0; k < 60 && x.st->phase() == ms::Phase::Jogging; ++k) {
        x.motor(1).setPosition(AngleDim{x.motor(1).position().value() + 0.02});
        x.motor(1).setCurrent(Current{0.3});
        x.tick(Bench::held(false, false, true));
    }
    REQUIRE(x.st->phase() == ms::Phase::Asking);
    CHECK(line(x, 0, Bench::held()) == "Which way did it move? Touch your answer below");
    for (const std::string& l : said) {
        CAPTURE(l);
        CHECK(l.size() <= ms::Station::kPanelColumns);
    }
}

TEST_CASE("S13UX D2: a jog that did NOT move — R1 offered only under a cap that holds and never to a power "
          "mechanism, worded for a stop; judged after 0.4 s or more; gone when R1, NEXT or the next jog changes it") {
    // BUG CAUGHT: the cascade barely lifting at 3 V and the page silent about R1; a tap called a failure; the advice
    // left standing at a new ceiling; and (the review, 2026-11-19) "press R1" with the cap NOT SET (guide 21: keep
    // 3 V), on a mechanism with no cap, on a toggle against its stop, or as an order at a stop.
    Bench b{robotOne()};  // the cascade: a lift capped at 2.0 A, two motors
    b.build();
    b.arm();
    REQUIRE(b.st->capHeld(0));
    jogFor(b, 100);  // a tap: 0.1 s, nothing moved — not judged
    CHECK(doNow(b, 0, Bench::held()) == "UP raises, DOWN lowers; A checks its way (L1 held)");
    jogFor(b, 500);  // half a second at 3 V, every motor still
    CHECK(doNow(b, 0, Bench::held()) == "Did not move at 3 V. Not at a stop? R1 for 6 V");
    CHECK(b.logged("'cascade': the jog did NOT move it at 3.0 V"));
    CHECK(doNow(b, 0, Bench::loose()) == "Hold L1 to power the cascade");  // L1 first, always
    b.tick(Bench::held(false, false, false, true));  // R1: 6 V
    b.tick(Bench::held());
    CHECK(doNow(b, 0, Bench::held()) == "UP raises, DOWN lowers; A checks its way (L1 held)");
    jogFor(b, 500, false);  // DOWN at 6 V, still
    CHECK(doNow(b, 0, Bench::held()) == "Did not move at 6 V. Not at a stop? R1 for 9 V");
    // NEXT round to it again: the ceiling back at 3 V, the 6 V finding gone
    for (std::size_t k = 0; k < b.st->count(); ++k) b.st->next();
    REQUIRE(b.st->selected() == 0);
    CHECK(doNow(b, 0, Bench::held()) == "UP raises, DOWN lowers; A checks its way (L1 held)");
    // ONE of its two motors turning is a jog that MOVED (the most-moved motor decides)
    b.tick(Bench::held(true));
    for (int k = 0; k < 50; ++k) {
        b.motor(10).setPosition(AngleDim{b.motor(10).position().value() + 0.02});
        b.tick(Bench::held(true));
    }
    b.tick(Bench::held());
    CHECK(doNow(b, 0, Bench::held()) == "UP raises, DOWN lowers; A checks its way (L1 held)");
    // a 3 s jog that never moved, cut by the jog cap (robot one's own case): judged too, said after L1's re-arm
    b.tick(Bench::held(true));
    for (int k = 0; k < 300; ++k) b.tick(Bench::held(true));
    REQUIRE(b.st->cutKind(0) == ms::CutKind::JogTime);
    b.tick(Bench::loose());
    b.tick(Bench::held());
    REQUIRE_FALSE(b.st->cut(0));
    CHECK(doNow(b, 0, Bench::held()) == "Did not move at 3 V. Not at a stop? R1 for 6 V");
    // at the most: no R1 to offer
    for (int k = 0; k < 3; ++k) {
        b.tick(Bench::held(false, false, false, true));
        b.tick(Bench::held());
    }
    REQUIRE(b.st->ceiling() == 12.0);
    b.tick(Bench::held(true));
    for (int k = 0; k < 60; ++k) b.tick(Bench::held(true));
    b.tick(Bench::loose(true));  // L1 let go mid-jog ends it too
    CHECK(doNow(b, 0, Bench::held()) == "Did not move at 12 V: at a stop, or check cables");
    // a side toggle (a POWER mechanism, capped): never R1 — "run it briefly, never into its stop"
    b.select(2);
    b.arm();
    REQUIRE(b.st->capHeld(2));
    jogFor(b, 500);
    CHECK(doNow(b, 2, Bench::held()) == "Did not move at 3 V: at a stop, or check cables");
    // the cap NOT SET (refused, or a motor that holds a DIFFERENT number): keep 3 V (guide 21)
    for (const bool refuse : {true, false}) {
        CAPTURE(refuse);
        Bench n{robotOne()};
        n.build();
        if (refuse) {
            n.motor(1).refuseCurrentLimit(true);
        } else {
            n.motor(1).currentLimitTakesInstead(1.0);
        }
        n.arm();
        REQUIRE_FALSE(n.st->capHeld(0));
        jogFor(n, 500);
        CHECK(doNow(n, 0, Bench::held()) == "Did not move at 3 V, cap NOT SET: check cables");
    }
    // NO cap stated — robot x's arm and lift, the claw robot's pivot, a gentle power one: never R1
    Bench x{robotX()};
    x.build();
    for (const std::size_t i : {std::size_t{0}, std::size_t{1}}) {
        CAPTURE(i);
        x.select(i);
        x.arm();
        jogFor(x, 500);
        CHECK(doNow(x, i, Bench::held()) == "Did not move at 3 V: at a stop, or check cables");
    }
    Bench u{robotTwo()};
    u.build();
    u.arm();
    jogFor(u, 500);
    CHECK(doNow(u, 0, Bench::held()) == "Did not move at 3 V: at a stop, or check cables");
    Bench g{a6_fixture::kGentle};
    g.build();
    g.arm();
    jogFor(g, 500);
    CHECK(doNow(g, 0, Bench::held()) == "Did not move at 3 V: at a stop, or check cables");
    for (Bench* bp : {&b, &x, &u, &g}) {
        for (std::size_t i = 0; i < bp->st->count(); ++i) CHECK(doNow(*bp, i, Bench::held()).size() <= 50);
    }
}

TEST_CASE("S13UX D2: a check abandoned because it was STILL MOVING says so on the panel — until the next press or "
          "NEXT") {
    // BUG CAUGHT (the review, 2026-11-19): only the log said "let it settle, then press A again"; the panel went
    // from "Watch it" to the plain line with no verdict and nothing saying why.
    const auto sag = [](Bench& b) {
        b.tick(Bench::held());
        b.tick(Bench::held(false, false, true));  // A: the announcement
        REQUIRE(b.st->phase() == ms::Phase::Announcing);
        for (int k = 0; k < 120 && b.st->phase() == ms::Phase::Announcing; ++k) {
            b.motor(10).setPosition(AngleDim{b.motor(10).position().value() + 0.01});
            b.tick(Bench::held(false, false, true));
        }
        REQUIRE(b.st->phase() == ms::Phase::Idle);
        b.tick(Bench::held());
    };
    Bench b{robotOne()};
    b.build();
    b.arm();
    sag(b);
    CHECK(b.logged("still MOVING when its jog was due"));
    CHECK_FALSE(b.st->hasVerdict());
    CHECK(doNow(b, 0, Bench::held()) == "It was still moving: wait, then press A again");
    CHECK(doNow(b, 0, Bench::loose()) == "Hold L1 to power the cascade");
    CHECK(doNow(b, 1, Bench::held()) == "UP / DOWN jog it; A checks which way (L1 held)");  // its own page only
    jogFor(b, 100);  // the next press clears it
    CHECK(doNow(b, 0, Bench::held()) == "UP raises, DOWN lowers; A checks its way (L1 held)");
    sag(b);
    REQUIRE(doNow(b, 0, Bench::held()) == "It was still moving: wait, then press A again");
    for (std::size_t k = 0; k < b.st->count(); ++k) b.st->next();  // so does NEXT
    CHECK(doNow(b, 0, Bench::held()) == "UP raises, DOWN lowers; A checks its way (L1 held)");
}

TEST_CASE("S13UX D2/D4: each cut says what cut it and the ONE way on — the 3 s jog cap: L1 let go and held; a fault: "
          "DONE, then open 13 again") {
    // BUG CAUGHT: every cut sending the operator out of the station (robot one: "we need to restart this"), or a
    // fault cut worded as if L1 cleared it.
    Bench j{robotTwo()};
    j.build();
    j.arm();
    j.motor(6).setPosition(AngleDim{0.0});
    for (int k = 0; k <= 300; ++k) {
        j.motor(6).setPosition(AngleDim{j.motor(6).position().value() + 0.01});
        j.tick(Bench::held(true));
    }
    REQUIRE(j.st->cut(0));
    CHECK(j.st->cutKind(0) == ms::CutKind::JogTime);
    CHECK(doNow(j, 0, Bench::held(true)) == "Stopped at 3 s: let go of L1, then hold it again");
    CHECK(j.logged("CUT 'claw pivot 6': a jog held for its stated maximum, 3.0 s (HA-182) -- let go of L1 and hold it "
                   "again to go on"));
    // a FIGHT (robot x's mirrored lift)
    Bench f{robotX()};
    f.build();
    f.select(1);
    f.arm();
    f.motor(2).setVelocity(AngularVelocity{3.0});
    f.motor(3).setVelocity(AngularVelocity{-3.0});
    for (int k = 0; k < 25; ++k) f.tick(Bench::held(true));
    REQUIRE(f.st->cut(1));
    CHECK(f.st->cutKind(1) == ms::CutKind::Fight);
    CHECK(doNow(f, 1, Bench::held()) == "Its motors FIGHT: DONE, then open 13 again");
    // the current
    Bench c{robotTwo()};
    c.build();
    c.arm();
    c.motor(6).setCurrent(Current{2.41});
    for (int k = 0; k < 25; ++k) c.tick(Bench::held(true));
    REQUIRE(c.st->cut(0));
    CHECK(c.st->cutKind(0) == ms::CutKind::Current);
    CHECK(doNow(c, 0, Bench::held()) == "Drew too much current: DONE, then open 13 again");
    // the stall signature (robot x's arm states one)
    Bench s{robotX()};
    s.build();
    s.arm();
    s.motor(1).setCurrent(Current{2.3});
    for (int k = 0; k < 200 && !s.st->cut(0); ++k) s.tick(Bench::held(true));
    REQUIRE(s.st->cut(0));
    CHECK(s.st->cutKind(0) == ms::CutKind::Stall);
    CHECK(doNow(s, 0, Bench::held()) == "It STALLED: DONE, then open 13 again");
    // a LIFT's stall is most likely a stop (guide 21 step 3): RECORD first, then DONE
    Bench l{robotOne()};
    l.build();
    l.arm();
    l.motor(10).setCurrent(Current{1.9});
    l.motor(1).setCurrent(Current{1.9});
    for (int k = 0; k < 100 && !l.st->cut(0); ++k) l.tick(Bench::held(true));
    REQUIRE(l.st->cutKind(0) == ms::CutKind::Stall);
    CHECK(doNow(l, 0, Bench::held()) == "It STALLED: at a stop? RECORD, then DONE, open 13");
    (void)recordAll(l, 1);  // RECORDed: the line no longer invites another RECORD (a second one is that step AGAIN)
    CHECK(doNow(l, 0, Bench::held()) == "RECORDed: now DONE, then open 13 again");
    for (std::size_t k = 0; k < l.st->count(); ++k) l.st->next();
    CHECK(doNow(l, 0, Bench::held()) == "RECORDed: now DONE, then open 13 again");
    (void)recordAll(s, 1);  // robot x's arm is no lift: DONE, as before
    CHECK(doNow(s, 0, Bench::held()) == "It STALLED: DONE, then open 13 again");
    // a RECORD before a LATER cut is not this cut's RECORD (robot one's 3 s cut, RECORDed, re-armed, then a stall)
    Bench r{robotOne()};
    r.build();
    r.arm();
    for (int k = 0; k <= 300; ++k) r.tick(Bench::held(true));
    REQUIRE(r.st->cutKind(0) == ms::CutKind::JogTime);
    (void)recordAll(r, 1);
    CHECK(doNow(r, 0, Bench::held()) == "Stopped at 3 s: let go of L1, then hold it again");
    r.tick(Bench::loose());
    r.tick(Bench::held());
    REQUIRE_FALSE(r.st->cut(0));
    r.motor(10).setCurrent(Current{1.9});
    r.motor(1).setCurrent(Current{1.9});
    for (int k = 0; k < 100 && !r.st->cut(0); ++k) r.tick(Bench::held(true));
    REQUIRE(r.st->cutKind(0) == ms::CutKind::Stall);
    CHECK(doNow(r, 0, Bench::held()) == "It STALLED: at a stop? RECORD, then DONE, open 13");
    // a check CUT mid-jog that still asks: the question first (the row shows only its answers)
    Bench q{robotOne()};
    q.build();
    q.arm();
    q.announce();
    for (int k = 0; k < 60 && (k == 0 || q.st->phase() == ms::Phase::Jogging); ++k) {
        if (k < 5) {
            q.motor(10).setPosition(AngleDim{q.motor(10).position().value() + 0.02});
            q.motor(1).setPosition(AngleDim{q.motor(1).position().value() + 0.02});
        }
        q.motor(10).setCurrent(Current{k == 0 ? 0.0 : 1.9});
        q.motor(1).setCurrent(Current{k == 0 ? 0.0 : 1.9});
        q.tick(Bench::held(false, false, true));
    }
    REQUIRE(q.st->phase() == ms::Phase::Asking);
    REQUIRE(q.st->cutKind(0) == ms::CutKind::Stall);
    CHECK(doNow(q, 0, Bench::held()) == "Which way did it move? Touch your answer below");
    q.st->answer(0);
    q.drain();
    CHECK(doNow(q, 0, Bench::held()) == "It STALLED: at a stop? RECORD, then DONE, open 13");
    // a field link never hides a cut
    ms::Pad fl = Bench::held();
    fl.field = true;
    CHECK(doNow(j, 0, fl) == "Stopped at 3 s: let go of L1, then hold it again");
    for (Bench* b : {&j, &f, &c, &s, &l, &q, &r}) {
        for (std::size_t i = 0; i < b->st->count(); ++i) CHECK(doNow(*b, i, Bench::held()).size() <= 50);
    }
}

TEST_CASE("S13UX D4: a jog-time cut RE-ARMS when L1 is let go and held again — never with L1 held throughout, never "
          "through a controller drop, never by itself; a FAULT cut never re-arms that way") {
    // BUG CAUGHT: robot one, 2026-11-17 — after a 3 s jog the operator had to leave the station and re-enter, and
    // nobody knew; and the opposite danger: a fight or a stall cleared by a finger on L1.
    const auto jogCut = [](Bench& b) {
        b.arm();
        b.tick(Bench::held(true));
        for (int k = 0; k < 300; ++k) b.tick(Bench::held(true));
        REQUIRE(b.st->cut(0));
        REQUIRE(b.st->cutKind(0) == ms::CutKind::JogTime);
    };
    Bench b{robotTwo()};
    b.build();
    jogCut(b);
    // L1 HELD THROUGHOUT: arrows let go and pressed again, for 10 s — still cut, nothing driven
    for (int k = 0; k < 500; ++k) b.tick(Bench::held((k / 50) % 2 == 1));
    CHECK(b.st->cut(0));
    CHECK(b.volts(6) == 0.0);
    CHECK(b.logged("REFUSED 'claw pivot 6': CUT (a jog held for its stated maximum, 3.0 s (HA-182)) -- let go of L1 "
                   "and hold it again"));
    // a controller DROP and back with L1 held: not a deliberate let-go
    b.tick(ms::Pad{});
    b.tick(ms::Pad{});
    b.tick(Bench::held());
    CHECK(b.st->cut(0));
    // let go (connected) — still cut until it is HELD again
    b.tick(Bench::loose(true));
    CHECK(b.st->cut(0));
    CHECK(doNow(b, 0, Bench::loose()) == "Stopped at 3 s: let go of L1, then hold it again");
    // held again, UP still held from before: re-armed, but nothing resumes by itself
    b.tick(Bench::held(true));
    CHECK_FALSE(b.st->cut(0));
    CHECK(b.st->cutKind(0) == ms::CutKind::None);
    CHECK(std::string{b.st->cutWhy(0)}.empty());
    CHECK(b.logged("'claw pivot 6': L1 let go and held again -- its 3 s stop is cleared"));
    CHECK(doNow(b, 0, Bench::held(true)) == "Let go of UP, then press it again");
    for (int k = 0; k < 20; ++k) b.tick(Bench::held(true));
    CHECK(b.volts(6) == 0.0);
    CHECK_FALSE(b.st->live(0));
    // a fresh press jogs again — still armed (the confirm is per entry), its 3 s from this press
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    CHECK(b.volts(6) == 3.0);
    for (int k = 0; k < 299; ++k) b.tick(Bench::held(true));
    CHECK_FALSE(b.st->cut(0));
    b.tick(Bench::held(true));
    CHECK(b.st->cut(0));  // and the cap still cuts at 3 s

    // A FAULT: a fight, an over-current — L1 let go and held again changes nothing
    Bench f{robotX()};
    f.build();
    f.select(1);
    f.arm();
    f.motor(2).setVelocity(AngularVelocity{3.0});
    f.motor(3).setVelocity(AngularVelocity{-3.0});
    for (int k = 0; k < 25; ++k) f.tick(Bench::held(true));
    REQUIRE(f.st->cut(1));
    f.motor(2).setVelocity(AngularVelocity{0.0});
    f.motor(3).setVelocity(AngularVelocity{0.0});
    for (int k = 0; k < 3; ++k) {
        f.tick(Bench::loose());
        f.tick(Bench::held());
    }
    f.tick(Bench::held(true));
    CHECK(f.st->cut(1));
    CHECK(f.volts(2) == 0.0);
    CHECK(f.logged("REFUSED 'lift': CUT (its motor(s) 2 disagree with their mates for 250 ms (a FIGHT)) -- leave and "
                   "re-enter station 13"));
    Bench c{robotTwo()};
    c.build();
    c.arm();
    c.motor(6).setCurrent(Current{2.41});
    for (int k = 0; k < 25; ++k) c.tick(Bench::held(true));
    REQUIRE(c.st->cutKind(0) == ms::CutKind::Current);
    c.motor(6).setCurrent(Current{0.0});
    c.tick(Bench::loose());
    c.tick(Bench::held());
    c.tick(Bench::held(true));
    CHECK(c.st->cut(0));
    CHECK(c.volts(6) == 0.0);
    // re-entering the station is still the way on from a fault
    c.reenter();
    CHECK_FALSE(c.st->cut(0));
}

TEST_CASE("S13UX D3: a lift's page is its measuring guide — robot one's cascade (no levels): its bottom stop, then "
          "its top stop, the current step marked and moved on by a RECORD after a jog") {
    // BUG CAUGHT: a page that never says what to RECORD next ("the screen is confusing"), a step that does not move
    // on, or one that runs past the end; and (the verifier, 2026-11-19) a second RECORD with nothing jogged between
    // labelled the NEXT step — kept for the rest of the run.
    Bench b{robotOne()};
    b.build();
    REQUIRE(b.st->built(0));
    CHECK(b.st->guideSteps(0) == 2);
    std::vector<std::string> p = panel(b, 0);
    REQUIRE(p.size() >= 3);
    CHECK(p[0] == "To measure it, RECORD at each step (1 of 2):");
    CHECK(p[1] == "> 1. Lower it onto its bottom stop, then RECORD");
    CHECK(p[2] == "  2. Top stop: UP at 3-6 V till it stops; RECORD");
    (void)recordAll(b, 1);
    CHECK(lastGuide(b) == "[R3A] mech guide: REC 1 = 'cascade' step 1 of 2, its bottom stop");
    p = panel(b, 0);
    CHECK(p[0] == "To measure it, RECORD at each step (2 of 2):");
    CHECK(p[1] == "  1. its bottom stop: RECORDed");
    CHECK(p[2] == "> 2. Top stop: UP at 3-6 V till it stops; RECORD");
    // nothing jogged since (robot one, 2026-11-17: two RECORDs, nothing between): that step AGAIN, not the next
    (void)recordAll(b, 2);
    CHECK(lastGuide(b) == "[R3A] mech guide: REC 2 = 'cascade' step 1 of 2, its bottom stop -- AGAIN (not jogged since)");
    b.arm();  // arming powers nothing
    (void)recordAll(b, 3);
    CHECK(lastGuide(b) == "[R3A] mech guide: REC 3 = 'cascade' step 1 of 2, its bottom stop -- AGAIN (not jogged since)");
    CHECK(b.st->guideStep(0) == 1);
    jogFor(b, 100);  // a jog: the next RECORD is the next step
    (void)recordAll(b, 4);
    CHECK(lastGuide(b) == "[R3A] mech guide: REC 4 = 'cascade' step 2 of 2, its top stop");
    p = panel(b, 0);
    CHECK(p[0] == "Measured: all 2 steps RECORDed");
    CHECK(p[2] == "  2. its top stop: RECORDed");
    (void)recordAll(b, 5);
    CHECK(lastGuide(b) == "[R3A] mech guide: REC 5 = 'cascade' step 2 of 2, its top stop -- AGAIN (not jogged since)");
    jogFor(b, 100);
    (void)recordAll(b, 6);  // past the end: nothing moves on, nothing is said
    CHECK(b.st->guideStep(0) == 2);
    CHECK_FALSE(b.logged("REC 6 = 'cascade'"));
    // a RECORD with ANOTHER mechanism shown moves the cascade's guide on not at all
    Bench o{robotOne()};
    o.build();
    o.select(1);
    (void)recordAll(o, 1);
    CHECK(o.st->guideStep(0) == 0);
    // the other kinds: one line each, first on their page
    CHECK(panel(o, 1)[0] == "Run it briefly, never into its stop: which way?");
    CHECK(panel(o, 3)[0] == "Fire it once (UP) and answer where it went");
    for (std::size_t i = 0; i < o.st->count(); ++i) {
        for (const std::string& l : panel(o, i)) CHECK(l.size() <= ms::Station::kPanelColumns);
    }
    // an ARM (a position mechanism in degrees) is no lift: no guide, the plain jog line
    Bench x{robotX()};
    x.build();
    CHECK(x.st->guideSteps(0) == 0);
    CHECK(panel(x, 0)[0].starts_with("motor 1 (port 1)"));
    CHECK(doNow(x, 0, Bench::held()) == "UP / DOWN jog it; A checks which way (L1 held)");
}

TEST_CASE("S13UX D3: the guide moves on only for a RECORD that READ the lift, and its place (the step, and whether it "
          "was jogged since) carries over to the next entry while Bench Tests runs") {
    // BUG CAUGHT (the review, 2026-11-19): an ABSENT cascade's RECORD logged as its bottom stop; a stall at a stop
    // sending the operator out (DONE, open 13 again) and back to "> 1. Lower it onto its bottom stop", the next
    // RECORD then labelled step 1.
    Bench a{robotOne()};
    a.dev.refusePort(10);
    a.build();
    REQUIRE_FALSE(a.st->built(0));
    (void)recordAll(a, 1);
    CHECK(a.st->guideStep(0) == 0);
    CHECK_FALSE(a.logged("mech guide:"));
    CHECK(std::string{a.st->recordText()} == "REC 1: cascade in the log");
    // carried over: bench_r3a.cpp keeps guideMark() at the exit and hands it back after the CLEAR (pinned below)
    Bench b{robotOne()};
    b.build();
    (void)recordAll(b, 1);
    const ms::GuideMark kept = b.st->guideMark(0);
    REQUIRE(kept.step == 1);
    CHECK_FALSE(kept.jogged);
    b.reenter();
    CHECK(b.st->guideStep(0) == 0);  // a new Station starts at step 1 ...
    b.st->resumeGuide(0, kept);      // ... until the page hands its place back
    std::vector<std::string> p = panel(b, 0);
    CHECK(p[1] == "  1. its bottom stop: RECORDed");
    CHECK(p[2] == "> 2. Top stop: UP at 3-6 V till it stops; RECORD");
    (void)recordAll(b, 1);  // nothing jogged since step 1's RECORD, in either entry: step 1 again
    CHECK(lastGuide(b) == "[R3A] mech guide: REC 1 = 'cascade' step 1 of 2, its bottom stop -- AGAIN (not jogged since)");
    b.arm();
    jogFor(b, 100);  // jogged in this entry, RECORDed in the next (a fault cut between): a NEW step
    const ms::GuideMark moved = b.st->guideMark(0);
    REQUIRE(moved.step == 1);
    REQUIRE(moved.jogged);
    b.reenter();
    b.st->resumeGuide(0, moved);
    (void)recordAll(b, 1);  // this entry's REC 1 IS the top stop
    CHECK(lastGuide(b) == "[R3A] mech guide: REC 1 = 'cascade' step 2 of 2, its top stop");
    b.st->resumeGuide(0, {99, false});
    CHECK(b.st->guideStep(0) == 2);  // never past its last step
    b.st->resumeGuide(1, {3, true});
    CHECK(b.st->guideStep(1) == 0);  // a toggle has no guide
    b.st->resumeGuide(99, {1, true});  // no such mechanism: nothing
}

namespace s13ux_fixture {
using namespace shulib::mechanism::words;
// A lift with six levels (one name 15 characters long): the guide's widest case. A lift with a long name:
// the big line's (its route ids, <name>_home, fit 23).
inline constexpr Lift tall{.name = "tall", .motors = {Forward}, .cartridge = Blue, .gearing = direct,
                           .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.3_s},
                           .currentCap = {.amps = 2.4}, .range = {0_in, 40_in}, .gravity = constantGravity(1.0_volt),
                           .home = restsAtStop(0_in),
                           .presets = {{"pin", 4_in}, {"cup", 8_in}, {"abcdefghijklmno", 12_in}, {"d", 16_in},
                                       {"e", 20_in}, {"f", 24_in}}};
inline constexpr MechanismTable kTall = mechanisms(wire(tall, motors(1)));
inline constexpr Lift longName{.name = "far side cascade", .motors = {Forward}, .cartridge = Blue,
                               .gearing = direct, .shape = spool(1.5_in), .safe = Hold,
                               .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.3_s}, .range = {0_in, 40_in},
                               .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr MechanismTable kLongName = mechanisms(wire(longName, motors(1)));
// The widest lift: four motors, an angle sensor, a home switch AND six levels (kMaxPanelLines' worst case).
inline constexpr Lift wide{.name = "wide lift", .motors = {Forward, Reversed, Forward, Reversed}, .cartridge = Red,
                           .gearing = direct, .shape = spool(1.5_in), .angleSensor = rotationSensor(), .safe = Hold,
                           .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in},
                           .gravity = constantGravity(1.0_volt), .home = homeBySwitch(0_in, -3_volt),
                           .presets = {{"a", 2_in}, {"b", 4_in}, {"c", 6_in}, {"d", 8_in}, {"e", 10_in}, {"f", 12_in}}};
inline constexpr MechanismTable kWide = mechanisms(wire(wide, motors(1, 2, 3, 4), angleSensor(10), homeSwitch('B')));
// Capped power mechanisms whose maxVolts print wide: 4 significant digits, the smallest legal, nearly 12.
inline constexpr Power odd{.name = "odd", .motors = {Forward}, .cartridge = Green, .safe = Brake,
                           .currentCap = {.amps = 2.5}, .maxVolts = 9.125_volt};
inline constexpr Power wee{.name = "wee", .motors = {Forward}, .cartridge = Green, .safe = Brake,
                           .currentCap = {.amps = 2.5}, .maxVolts = 0.0001234_volt};
inline constexpr Power nearly{.name = "nearly", .motors = {Forward}, .cartridge = Green, .safe = Brake,
                              .currentCap = {.amps = 2.5}, .maxVolts = 11.96_volt};
inline constexpr MechanismTable kOdd = mechanisms(wire(odd, motors(5)), wire(wee, motors(6)), wire(nearly, motors(7)));
}  // namespace s13ux_fixture

TEST_CASE("S13UX D2: DOWN on a pneumatic says it does nothing — robot one's ramp ('up' / 'down') fires on UP only") {
    // BUG CAUGHT (the verifier, 2026-11-19): the ramp's line `UP fires it to 'down'` invites DOWN, and each DOWN
    // press then said `Let go of DOWN, then press it again` — a dead end: DOWN never fires a pneumatic.
    Bench b{robotOne()};
    b.build();
    b.select(5);
    REQUIRE(std::string{b.st->spec(5).name} == "ramp");
    CHECK(doNow(b, 5, Bench::held()) == "UP fires it to 'down' (keep L1 held)");
    for (int k = 0; k < 3; ++k) {
        b.tick(Bench::held());
        b.tick(Bench::held(false, true));  // a fresh DOWN press
        CHECK(b.st->phase() == ms::Phase::Idle);
        CHECK_FALSE(b.st->live(5));
        CHECK(doNow(b, 5, Bench::held(false, true)) == "DOWN does nothing here: UP fires it");
    }
    CHECK(doNow(b, 0, Bench::held(false, true)) == "Let go of DOWN, then press it again");  // a motor one's DOWN jogs
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    CHECK(b.st->phase() == ms::Phase::Confirm);  // UP asks to fire it
}

TEST_CASE("S13UX D2: the volts fit the panel for every legal maxVolts — 9.125 V, 0.0001234 V, 11.96 V: the did-not-move "
          "lines and the status line within 50 columns, whole volts bare, others to 0.1 V as the log prints them") {
    // BUG CAUGHT (the verifier, 2026-11-19): `Did not move at 9.125 V: at a stop, or check cables` — 51 columns.
    Bench b{s13ux_fixture::kOdd};
    b.build();
    const std::string shown[] = {"9.1", "0.0", "12.0"};
    const auto limit = [&](std::size_t i) {
        char l[96];
        b.st->limitText(i, l, sizeof l);
        return std::string{l};
    };
    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        b.select(i);
        b.arm();
        for (int k = 0; k < 4; ++k) {  // R1 to 12 V: the most is its maxVolts
            b.tick(Bench::held(false, false, false, true));
            b.tick(Bench::held());
        }
        jogFor(b, 500);
        const std::string v = shown[i];
        const std::string still = doNow(b, i, Bench::held());
        CHECK(still == "Did not move at " + v + " V: at a stop, or check cables");
        CHECK(limit(i) == v + " V now (the most); cap 2.5 A");
        b.motor(static_cast<std::uint8_t>(5 + i)).refuseCurrentLimit(true);
        jogFor(b, 500);
        const std::string unset = doNow(b, i, Bench::held());
        CHECK(unset == "Did not move at " + v + " V, cap NOT SET: check cables");
        CHECK(limit(i) == v + " V now; cap 2.5 A NOT SET: keep 3 V");
        for (const std::string& l : {still, unset, limit(i)}) CHECK(l.size() <= ms::Station::kPanelColumns);
    }
}

TEST_CASE("S13UX D3: a lift WITH levels — each level its own step between the stops, every line within 50 columns") {
    // BUG CAUGHT: the levels left out of the guide (a row AT each level is what makes each level exact — HA-254),
    // or a long level name pushing the line off the panel.
    Bench b{s13ux_fixture::kTall};
    b.build();
    CHECK(b.st->guideSteps(0) == 8);
    std::vector<std::string> p = panel(b, 0);
    REQUIRE(p.size() >= 9);
    CHECK(p[0] == "To measure it, RECORD at each step (1 of 8):");
    CHECK(p[2] == "  2. Jog to level 'pin', tape it, RECORD");
    CHECK(p[4] == "  4. Jog to level 'abcdefghijklm', tape it, RECORD");
    CHECK(p[8] == "  8. Top stop: UP at 3-6 V till it stops; RECORD");
    b.arm();
    for (int r = 1; r <= 3; ++r) {
        (void)recordAll(b, r);
        jogFor(b, 100);  // each step after the first is reached by a jog
    }
    p = panel(b, 0);
    CHECK(p[3] == "  3. level 'cup': RECORDed");
    CHECK(p[4] == "> 4. Jog to level 'abcdefghijklm', tape it, RECORD");
    CHECK(b.logged("[R3A] mech guide: REC 3 = 'tall' step 3 of 8, level 'cup'"));
    for (const std::string& l : p) CHECK(l.size() <= ms::Station::kPanelColumns);
    CHECK(p.size() <= ms::Station::kMaxPanelLines);
    // the widest lift: its guide AND every role's line — none cut (its home switch's question is the last)
    Bench w{s13ux_fixture::kWide};
    w.build();
    const std::vector<std::string> all = panel(w, 0);
    CHECK(all.size() == 19);
    CHECK(all.back() == "  RECORD it pressed, released, and cable OUT");
    for (const std::string& l : all) CHECK(l.size() <= ms::Station::kPanelColumns);
}

TEST_CASE("S13UX D3: RECORD's big line is what RECORD wrote for the SHOWN mechanism — its output degrees EQUAL to the "
          "serial line's, the name cut before the number, gone when anything moves or NEXT") {
    // BUG CAUGHT: robot one — the number the coordinator types (the cascade's `output … deg`) was only in a serial
    // line among dozens; a big line that rounds it differently, reads the inches, or shows another mechanism's.
    Bench b{robotOne()};
    b.build();
    b.motor(10).setPosition(AngleDim{65.6});
    b.motor(1).setPosition(AngleDim{65.6});
    const std::vector<std::string> rec = recordAll(b, 2);
    std::string serial;
    for (const std::string& l : rec) {
        if (l.starts_with("[R3A] mech REC 2 'cascade' LIFT ") && l.find("(output ") != std::string::npos) serial = l;
    }
    REQUIRE_FALSE(serial.empty());
    const std::size_t at = serial.find("(output ") + 8;
    const std::string number = serial.substr(at, serial.find(" deg", at) - at);
    CHECK(number != "+0.00");
    REQUIRE(b.st->recordShown());
    CHECK(std::string{b.st->recordText()} == "REC 2: cascade output " + number + " deg");
    CHECK(std::string_view{b.st->recordText()}.size() <= ms::Station::kTitleColumns);
    // the reading line shows the SAME digits before the RECORD too
    CHECK(has(panel(b, 0), "output " + number + " deg since power-up"));
    // the name is cut, never the number
    Bench t{s13ux_fixture::kLongName};
    t.build();
    t.motor(1).setPosition(AngleDim{-1745.3});  // about -99998.6 deg: the widest number a reading line holds
    std::string far;
    for (const std::string& l : recordAll(t, 12)) {
        const std::size_t o = l.find("(output ");
        if (o != std::string::npos) far = l.substr(o + 8, l.find(" deg", o) - o - 8);
    }
    REQUIRE(far.starts_with("-9999"));
    CHECK(std::string{t.st->recordText()} == "REC 12: far sid output " + far + " deg");  // the name cut to 7
    CHECK(std::string_view{t.st->recordText()}.size() <= ms::Station::kTitleColumns);
    // anything powered ends it; so does NEXT
    b.arm();
    CHECK(b.st->recordShown());  // arming moves nothing
    b.tick(Bench::held(true));
    CHECK_FALSE(b.st->recordShown());
    b.tick(Bench::held());
    (void)recordAll(b, 3);
    REQUIRE(b.st->recordShown());
    b.st->next();
    CHECK_FALSE(b.st->recordShown());
    // the other kinds: a pneumatic's position, a power mechanism written to the log
    (void)recordAll(b, 4);  // 'right toggle' shown
    CHECK(std::string{b.st->recordText()} == "REC 4: right toggle in the log");
    b.select(3);
    (void)recordAll(b, 5);
    CHECK(std::string{b.st->recordText()} == "REC 5: claw at 'closed'");
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    b.st->confirm(true, true);  // a FIRE ends it too
    b.drain();
    CHECK_FALSE(b.st->recordShown());
    REQUIRE(b.st->live(3));
    char l[96];
    b.st->limitText(3, l, sizeof l);
    CHECK(std::string{l} == "Let go of L1: it goes back to 'closed'");  // its SAFE position, not where it is
    // a flywheel: the serial line's own rpm digits
    Bench f{robotX()};
    f.build();
    f.select(2);
    REQUIRE(std::string{f.st->spec(2).name} == "flywheel");
    f.motor(4).setVelocity(AngularVelocity{60.7357});
    std::string rpm;
    for (const std::string& r : recordAll(f, 3)) {
        const std::size_t o = r.find("'flywheel' FLYWHEEL ");
        if (o != std::string::npos) rpm = r.substr(o + 20, r.find(" rpm", o) - o - 20);
    }
    REQUIRE(rpm.find('.') != std::string::npos);
    CHECK(std::string{f.st->recordText()}.ends_with(" output " + rpm + " rpm"));
    CHECK(std::string_view{f.st->recordText()}.size() <= ms::Station::kTitleColumns);
}

TEST_CASE("S13UX D5: the DECLARED current cap is in each motor while station 13 powers it — the cascade's and the "
          "toggles'; nothing written before, nothing for a mechanism without one, an absent one untouched") {
    // BUG CAUGHT: guide 21's "never press R1 on the cascade" — station 13 ran the cascade UNCAPPED (2.5 A) while
    // every program caps it at 2.0 A; or a cap written into a motor nobody powers.
    namespace facts = shulib::bench::robot_one_facts;
    Bench b{robotOne()};
    b.dev.refusePort(9);  // the right toggle unplugged
    b.build();
    REQUIRE_FALSE(b.st->built(1));
    for (const std::uint8_t port : std::initializer_list<std::uint8_t>{10, 1, 2}) {
        CAPTURE(port);
        CHECK(b.motor(port).currentLimitWrites() == 0);  // nothing written before anything is powered
        CHECK(b.motor(port).currentLimit().value() == 2.5);
    }
    CHECK(b.st->statesCap(0));
    char l[96];
    b.st->limitText(0, l, sizeof l);
    CHECK(std::string{l} == "3 V now (R1 for more); cap 2.0 A when powered");
    b.arm();  // the cascade: its cap before its first volt
    CHECK(b.motor(10).currentLimit().value() == doctest::Approx(facts::kCascadeCapAmps.value()));
    CHECK(b.motor(1).currentLimit().value() == doctest::Approx(facts::kCascadeCapAmps.value()));
    CHECK(b.st->capHeld(0));
    CHECK(b.logged("'cascade': current cap 2.00 A per motor -- set and read back"));
    b.st->limitText(0, l, sizeof l);
    CHECK(std::string{l} == "3 V now (R1 for more); cap 2.0 A");
    // a motor that lost power comes back at 2.5 A: the next jog writes the cap again
    b.motor(1).losePower();
    b.tick(Bench::held(true));
    CHECK(b.motor(1).currentLimit().value() == doctest::Approx(facts::kCascadeCapAmps.value()));
    CHECK(b.volts(10) != 0.0);
    b.tick(Bench::held());
    // the left toggle (port 2): its own cap
    b.select(2);
    b.arm();
    CHECK(b.motor(2).currentLimit().value() == doctest::Approx(facts::kSideToggleCapAmps.value()));
    // the ABSENT right toggle: never written, never powered
    b.select(1);
    b.tick(Bench::held());
    b.tick(Bench::held(true));
    CHECK(b.motor(9).currentLimitWrites() == 0);
    CHECK(b.volts(9) == 0.0);
    // the confirm says the cap
    Bench c{robotOne()};
    c.build();
    c.tick(Bench::held());
    c.tick(Bench::held(true));
    REQUIRE(c.st->phase() == ms::Phase::Confirm);
    CHECK(std::string{c.st->confirmText().foot2} == "3 V now, R1 for more, capped at 2.0 A");
    // a motor that will not take it: said, and the mechanism still runs (a cap is a limit, never a gate)
    c.motor(1).refuseCurrentLimit(true);
    c.st->confirm(true, true);
    c.drain();
    CHECK_FALSE(c.st->capHeld(0));
    CHECK(c.logged("'cascade': motor 2 (port 1) did NOT take its current cap 2.00 A (reads 2.50 A) -- it runs uncapped"));
    char m[96];
    c.st->limitText(0, m, sizeof m);
    CHECK(std::string{m} == "3 V now; cap 2.0 A NOT SET: keep 3 V");
    c.tick(Bench::held());
    c.tick(Bench::held(true));
    CHECK(c.volts(10) == 3.0);
    // a mechanism that states NO cap (the claw robot's pivots): nothing written, ever
    Bench n{robotTwo()};
    n.build();
    CHECK_FALSE(n.st->statesCap(0));
    n.arm();
    jogFor(n, 200);
    CHECK(n.motor(6).currentLimitWrites() == 0);
    n.st->limitText(0, m, sizeof m);
    CHECK(std::string{m} == "3 V now (R1 for more)");
    // the direction check writes it again too (a motor that lost power, HA-220)
    Bench k{robotOne()};
    k.build();
    k.arm();
    k.motor(1).losePower();
    k.tick(Bench::held());
    k.tick(Bench::held(false, false, true));
    REQUIRE(k.st->phase() == ms::Phase::Announcing);
    CHECK(k.motor(1).currentLimit().value() == doctest::Approx(facts::kCascadeCapAmps.value()));
    // a motor that takes a DIFFERENT number: NOT SET (only the read-back can tell; guide 21 then keeps 3 V)
    Bench d{robotOne()};
    d.build();
    d.motor(1).currentLimitTakesInstead(1.0);
    d.arm();
    CHECK_FALSE(d.st->capHeld(0));
    d.st->limitText(0, m, sizeof m);
    CHECK(std::string{m} == "3 V now; cap 2.0 A NOT SET: keep 3 V");
}

TEST_CASE("pin (S13UX): bench_r3a.cpp feeds the field link into the Station, places and paints D2's line, RECORD's "
          "big line and the status line, calls recorded() after RECORD, paints no log line, and keeps a lift's guide "
          "while Bench Tests runs") {
    // BUG CAUGHT: what only bench_r3a.cpp does — a D2 line never painted, a field link never read (robot one's FIELD
    // CONNECTED controller, 2026-11-17), RECORD's big line never set, or the engineering log lines back on the panel.
    const std::string code = source_pins::stripComments(source_pins::readFile("src/programs/bench_r3a.cpp"));
    const std::string body =
        source_pins::squeeze(source_pins::bodyOf(code, "void mechanismsStation(pros::c::v5_device_e_t*)"));
    REQUIRE_FALSE(body.empty());
    const auto at = [&](const std::string& what) {
        CHECK_MESSAGE(source_pins::count(body, what) == 1, "mechanismsStation must hold exactly one: ", what);
        return body.find(what);
    };
    const std::size_t field = at("pad.field = (pros::c::competition_get_status() & COMPETITION_CONNECTED) != 0;");
    const std::size_t tick = at("st.tick(pad, now);");
    CHECK(field < tick);
    at("doNow.place(pros::E_TEXT_SMALL, kDoNowY, kSmall);");
    const std::size_t doNow = at("st.doNowText(i, pad, now, d, sizeof d);");
    CHECK(doNow < at("doNow.set(d, "));
    at("if (st.recordShown()) { title.set(st.recordText(), kColGood);");
    at("if (st.recordShown()) { status.set(statusText, statusPen); } else {");
    const std::size_t rec = at("st.record(records, i, mech13::rawTemperature, [&](const char* l) {");
    const std::size_t recorded = at("st.recorded(records);");
    CHECK(rec < recorded);
    const std::size_t limit = at("st.limitText(i, l, sizeof l);");
    CHECK(limit < at("status.set(l, "));
    // the engineering lines to the log only; a refused role still marks the menu's dot
    at("mech13::logOnly(mechanismDeclarationLine(list, i).text);");
    at("if (f.state == mechanism::RoleState::Refused) { setVerdict(Sev::Warn); }");
    // a lift's place in its guide kept while Bench Tests runs: handed back after the CLEAR, kept at the exit
    const std::size_t marked = at("st.markCleared();");
    const std::size_t resume = at("for (std::size_t i = 0; i < st.count(); ++i) st.resumeGuide(i, b.guide[i]);");
    const std::size_t exit = at("st.exit();");
    const std::size_t keep = at("for (std::size_t i = 0; i < st.count(); ++i) b.guide[i] = st.guideMark(i);");
    CHECK(marked < resume);
    CHECK(resume < doNow);
    CHECK(exit < keep);
    // the Station's events go to serial + SD only: never copied onto the status line
    CHECK(source_pins::count(body, "emit(st.event(k));") == 2);  // the loop's drain and the exit's
    CHECK(source_pins::count(body, "statusText, sizeof statusText, \"%.54s\"") == 0);
    CHECK(source_pins::count(body, "st.gatesText(") == 0);
}

TEST_CASE("pin (R-23, R-6, R-8, R-5): station 13's own safety steps in bench_r3a.cpp — the guard before the first "
          "prompt, the parts made only after the CLEAR, the dead-man watched INSIDE the confirm and L1 read after it, "
          "the page at the Station's kShownLines, every verdict on the menu's dot") {
    // BUG CAUGHT (the tests review): what only bench_r3a.cpp does had no check beyond compiling — a build moved
    // before the CLEAR (robot two's claw line DRIVEN before anybody said the robot was clear), a confirm fed the
    // pre-prompt L1, a confirm prompt waiting on a touch alone (the claw left fired with nobody on the dead-man),
    // the guard declared after the first way out. The host cannot compile that file (pros/screen.h): it is read.
    const std::string code = source_pins::stripComments(source_pins::readFile("src/programs/bench_r3a.cpp"));
    const std::string body =
        source_pins::squeeze(source_pins::bodyOf(code, "void mechanismsStation(pros::c::v5_device_e_t*)"));
    REQUIRE_FALSE(body.empty());
    const auto at = [&](const std::string& what) {
        CHECK_MESSAGE(source_pins::count(body, what) == 1, "mechanismsStation must hold exactly one: ", what);
        return body.find(what);
    };
    const std::size_t guard = at("mech13::ExitGuard guard{&st};");
    const std::size_t clear = at("if (!mech13::prompt(st.clearText())) {");
    const std::size_t build = at(".parts.build(");  // ONE build, whatever the object is spelled
    const std::size_t marked = at("st.markCleared();");
    const std::size_t ask = at("const mech13::Held answer = mech13::promptHeld(st.confirmText(), master);");
    const std::size_t heldNow = at("const bool heldNow = answer != mech13::Held::LetGo && master.isConnected() && "
                                   "master.pressed(hal::ControllerButton::L1);");
    const std::size_t confirm = at("st.confirm(answer == mech13::Held::Yes, heldNow);");
    CHECK(guard < clear);
    CHECK(clear < build);
    CHECK(build < marked);
    CHECK(ask < heldNow);
    CHECK(heldNow < confirm);
    CHECK(source_pins::count(body, "mech13::prompt(st.confirmText())") == 0);  // never the touch-only prompt
    at("constexpr std::size_t kShown = ms::Station::kShownLines;");
    at("all = st.panelLines(i, mech13::rawTemperature, shown, ms::Station::kMaxPanelLines);");
    at("const ms::ReadingsPage page = ms::readingsPage(all, kShown, readPage);");
    at("setVerdict(st.anyFinding() ? Sev::Bad : Sev::Good);");
    // the exit summary's sentences in pieces, uncapped (a fixed line budget dropped a long one's tail)
    at("ms::logPieces(v, nullptr, static_cast<std::size_t>(kScreenCols) - 2, [](const char* l) { emitf(\"  %s\", l); });");
    CHECK(source_pins::count(body, "ms::wrapText(v, static_cast<std::size_t>(kScreenCols) - 2") == 0);
    // promptHeld: the dead-man polled INSIDE the wait, before a touch is taken, closing it as LetGo
    const std::string held = source_pins::squeeze(
        source_pins::bodyOf(code, "Held promptHeld(const ms::PromptText& t, hal::pros::ProsController& master)"));
    const std::size_t loop = held.find("for (;;) {");
    const std::size_t poll = held.find("if (!master.isConnected() || !master.pressed(hal::ControllerButton::L1)) {");
    const std::size_t letGo = held.find("return Held::LetGo;");
    const std::size_t touch = held.find("const pros::screen_touch_status_s_t s = pros::c::screen_touch_status();");
    REQUIRE(loop != std::string::npos);
    REQUIRE(poll != std::string::npos);
    REQUIRE(letGo != std::string::npos);
    REQUIRE(touch != std::string::npos);
    CHECK(loop < poll);
    CHECK(poll < letGo);
    CHECK(letGo < touch);
}
