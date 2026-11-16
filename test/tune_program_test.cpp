// Adversarial tests for "shulib Tune" — mechanism::TuneProgram (include/shulib/mechanism/tune_program.hpp)
// and its PROS shell (include/shulib/hal/pros/tune.hpp) — chunk TUNE1-B1, 2026-11-10. The MECH-S/TUNE1
// brief's §2.2 TB2–TB5 and TB7–TB11 (TB1 is test/tune_panel_test.cpp; TB6 and TB12 are the src build
// gate's self-test). Each case tries to break ONE rule.
//
// THE BENCH: the program over hal::fake::FakeDeviceProvider, a FakeBrainScreen (a tap is a release the
// program reads), a FakeController (every button a STOP) and a status byte — and the HOST TEST PLANT
// (sim::ArmPlant / FlywheelPlant, its constants INVENTED — HA-183) stepping the provider's own fake motors
// on the mechanisms' ports. One program.step() is one control tick: the program reads, acts, paints, ticks
// its service, and the provider's pacer steps the plant and advances the clock. Say "host test plant",
// never "simulator". NOTHING HERE HAS RUN ON A ROBOT.

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechanism_live_tuning_rig.hpp"
#include "mechanism_position_rig.hpp"
#include "pros/shim_control.hpp"
#include "robots/chassis_table.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/control/trapezoid_profile.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_brain_screen.hpp"
#include "shulib/hal/fake/fake_controller.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/pros/tune.hpp"
#include "shulib/mechanism/tune_program.hpp"

namespace {

namespace mech = shulib::mechanism;
namespace fake = shulib::hal::fake;
namespace sim = shulib::sim;
using mech::TuneButton;
using mech::TunePage;
using shulib::hal::ControllerButton;
using shulib::manipulation::MechanismOutcome;

constexpr double kDegPerIn = 360.0 / (3.14159265358979323846 * 1.5);  // liftPlant's 1.5 in spool

// ── the tables the tests run ──────────────────────────────────────────────────────────
using namespace shulib::mechanism::words;

// a PNEUMATIC claw beside the cascade: TB8's "a pneumatic line included"
inline constexpr Pneumatic benchClaw{
    .name = "claw",
    .positions = {{"closed", {Low}}, {"open", {High}}},
    .start = "closed",
    .safe = "closed",
    .moveTime = 0.5_s,
};
inline constexpr MechanismTable kCascadeAndClaw =
    mechanisms(wire(tune_fixture::cascade, motors(1)), wire(benchClaw, lines('A')));

// a cascade with presets far apart (TB4's moves) and a SLOW one (TB9's 10 s cap)
inline constexpr Lift tallCascade{
    .name = "tall cascade",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
    .presets = {{"floor", 0_in}, {"stack 3", 18_in}},
};
inline constexpr MechanismTable kTallCascade = mechanisms(wire(tallCascade, motors(1)));

// TB11: a lift whose stall signature (0.6 A, lasting 2 s — longer than the report's second) sits UNDER an
// ordinary hold with a piece on it; and a cap of 0.6 A (under the move's peak) for the cap warning
inline constexpr Lift lowSignature{
    .name = "lowsig lift",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 0.6, .belowMotorRpm = 5, .lasting = 2.0_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
    .presets = {{"floor", 0_in}, {"mid", 12_in}},
};
inline constexpr MechanismTable kLowSignature = mechanisms(wire(lowSignature, motors(1)));
inline constexpr Lift cappedLift{
    .name = "capped lift",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 0.45, .belowMotorRpm = 5, .lasting = 0.5_s},
    .currentCap = {.amps = 0.6},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
    .presets = {{"floor", 0_in}, {"mid", 12_in}},
};
inline constexpr MechanismTable kCappedLift = mechanisms(wire(cappedLift, motors(1)));

// TB10: two mechanisms with motors — the cascade (NO cap) and a capped (1.5 A) stall-homed lift
inline constexpr MechanismTable kTwoLifts =
    mechanisms(wire(tune_fixture::cascade, motors(1)), wire(tune_fixture::midHome, motors(2)));

/// The competition status byte, programmable.
struct StatusIo final : mech::TuneIo {
    std::uint8_t status = 0;
    [[nodiscard]] std::uint8_t competitionStatus() override { return status; }
};

/// One sample the TEST records itself, from the PLANT and the fake motors — never from the program.
struct Rec {
    double t = 0.0;
    double x = 0.0;
    double amps = 0.0;
    bool running = false;
};

/// The program over fakes and the host test plant (header).
struct Bench {
    fake::FakeClock clock;
    fake::FakeBattery battery;
    fake::FakeTelemetrySink sink;
    pos_rig::PlantWorld world;
    fake::FakeDeviceProvider provider{clock, battery, sink, world};
    fake::FakeBrainScreen screen;
    fake::FakeController controller;
    StatusIo io;
    std::array<fake::FakeMotor*, 4> armMotors{};
    std::array<fake::FakeMotor*, 4> flyMotors{};
    std::optional<sim::ArmPlant> arm;
    std::optional<sim::FlywheelPlant> fly;
    std::size_t armIndex = 99;
    double zeroDeg = 0.0;          ///< the plant's joint when READY bound the service (a rests-at-stop zero)
    double startInches = 0.0;      ///< the plant, in inches from that zero, as the last test started
    mech::TuneProgram program;
    // the test's own recording
    bool recording = false;
    std::vector<Rec> rec;

    explicit Bench(const mech::MechanismTable& t, const sim::ArmPlantConfig& armCfg = tune_rig::liftPlant())
        : program{t, mech::TuneDevices{provider, screen, controller, io}, "Nov 10 2026 23:00:00"} {
        for (std::size_t i = 0; i < t.count; ++i) {
            const mech::MechanismSpec& s = t.specs[i];
            if (s.kind == mech::Kind::Position && !arm) {
                for (std::size_t m = 0; m < s.motorCount; ++m) {
                    armMotors[m] = &provider.fakeMotor(static_cast<std::uint8_t>(t.wiring[i].motors[m]));
                }
                arm.emplace(std::span<fake::FakeMotor* const>{armMotors.data(), s.motorCount}, armCfg);
                armIndex = i;
            } else if (s.kind == mech::Kind::Speed && !fly) {
                for (std::size_t m = 0; m < s.motorCount; ++m) {
                    flyMotors[m] = &provider.fakeMotor(static_cast<std::uint8_t>(t.wiring[i].motors[m]));
                }
                fly.emplace(std::span<fake::FakeMotor* const>{flyMotors.data(), s.motorCount},
                            pos_rig::flywheelPlant(600, 1.0 / 6.0, 1.0, 0.4, 0.2));
            }
        }
        world.clock = &clock;
        world.dt = 0.01;
        world.arm = arm ? &*arm : nullptr;
        world.fly = fly ? &*fly : nullptr;
        controller.setConnected(true);
    }

    /// The plant's position in inches from where it rested when READY bound the service (a lift that rests
    /// at its stop is zeroed where it is then) — computed from the PLANT, never from the program.
    [[nodiscard]] double plantInches() const { return (arm->trueDeg() - zeroDeg) / kDegPerIn; }
    /// The most current any of the position mechanism's motors draws (the fake motors, as the plant set).
    [[nodiscard]] double plantAmps() const {
        double a = 0.0;
        for (fake::FakeMotor* m : armMotors) {
            if (m != nullptr) {
                a = std::max(a, m->current().value());
            }
        }
        return a;
    }

    /// One control tick. Returns the program's answer (false: driver control ended).
    bool step() {
        const bool more = program.step();
        if (recording && arm) {
            rec.push_back(Rec{clock.now().value(), plantInches(), plantAmps(),
                              program.service().slot(armIndex).running()});
        }
        return more;
    }
    void steps(int n) {
        for (int i = 0; i < n; ++i) {
            REQUIRE(step());
        }
    }
    [[nodiscard]] const mech::TuneButtonView* button(TuneButton id) const { return program.drawn().button(id); }
    /// Step until `id` is drawn ARMED (the arming delay after a layout change).
    void waitArmed(TuneButton id) {
        for (int i = 0; i < 200; ++i) {
            const mech::TuneButtonView* b = button(id);
            if (b != nullptr && b->armed) {
                return;
            }
            REQUIRE(step());
        }
        FAIL("button never armed");
    }
    /// A tap in the middle of `id`, then the tick that reads it.
    void tap(TuneButton id) {
        waitArmed(id);
        tapAnyway(id);
    }
    /// A tap in the middle of `id` as drawn now, armed or not, then one tick.
    void tapAnyway(TuneButton id) {
        const mech::TuneButtonView* b = button(id);
        REQUIRE(b != nullptr);
        screen.tap((b->rect.x0 + b->rect.x1) / 2, (b->rect.y0 + b->rect.y1) / 2);
        REQUIRE(step());
    }
    void ready() {
        steps(1);
        waitArmed(TuneButton::Ready);
        if (arm) {
            zeroDeg = arm->trueDeg();  // the service binds inside the next tick, before the plant steps
        }
        tapAnyway(TuneButton::Ready);
        REQUIRE(program.page() == TunePage::Tune);
    }
    /// Select the shown mechanism's number named `name` (NEXT until it is).
    void select(const char* name) {
        for (int i = 0; i < 30; ++i) {
            const mech::ParameterList l = mech::parametersOf(program.tuning().spec(program.shown()));
            if (std::strcmp(l.items[program.selected()].name, name) == 0) {
                return;
            }
            tap(TuneButton::NextParam);
        }
        FAIL("no such number");
    }
    /// Start TEST A or B, through stage 2's GO when it is asked; recording from the tick that starts it.
    void startTest(TuneButton which) {
        const bool confirmed = program.confirmed(program.shown());
        if (confirmed) {
            waitArmed(which);
        } else {
            tap(which);
            REQUIRE(program.page() == TunePage::Confirm);
            waitArmed(TuneButton::Go);
        }
        recording = true;
        rec.clear();
        if (arm) {
            startInches = plantInches();  // where it is as the test's tick begins
        }
        tapAnyway(confirmed ? which : TuneButton::Go);
        REQUIRE(program.busy());
    }
    /// Tick until the test (or HOME) is over.
    void finish(int cap = 2000) {
        for (int i = 0; i < cap && program.busy(); ++i) {
            REQUIRE(step());
        }
        recording = false;
        REQUIRE_FALSE(program.busy());
    }
    [[nodiscard]] int linesContaining(std::string_view tag, std::string_view text) const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == tag && sink.at(i).message.find(text) != std::string::npos) {
                ++n;
            }
        }
        return n;
    }
};

/// The report, computed by the TEST from its own recording (the brief's TB4: "an independent computation
/// over the recorded samples"): the move is every sample up to the first one taken with the step no longer
/// running; the hold, the rest.
struct Independent {
    double overshoot = 0.0;
    double settle = -1.0;
    double peak = 0.0;
    double finalError = 0.0;
    double hold = -1.0;
};
Independent independently(const std::vector<Rec>& r, double start, double target, double within, bool succeeded) {
    Independent out{};
    std::size_t verdict = r.size();
    for (std::size_t i = 0; i < r.size(); ++i) {
        if (!r[i].running) {
            verdict = i;
            break;
        }
    }
    REQUIRE(verdict < r.size());
    const double sign = target >= start ? 1.0 : -1.0;
    double firstInside = -1.0;
    for (std::size_t i = 0; i <= verdict; ++i) {
        out.peak = std::max(out.peak, r[i].amps);
        out.overshoot = std::max(out.overshoot, (r[i].x - target) * sign);
        if (firstInside < 0.0 && std::fabs(r[i].x - target) <= within) {
            firstInside = r[i].t;
        }
    }
    out.finalError = r[verdict].x - target;
    if (succeeded && firstInside >= 0.0) {
        out.settle = r[verdict].t - firstInside;
    }
    if (succeeded && verdict + 1 < r.size()) {
        double sum = 0.0;
        for (std::size_t i = verdict + 1; i < r.size(); ++i) {
            sum += r[i].amps;
        }
        out.hold = sum / static_cast<double>(r.size() - verdict - 1);
    }
    return out;
}

[[nodiscard]] bool anyRequest(const fake::FakeDeviceProvider& p) {
    for (std::uint8_t port = 1; port <= 21; ++port) {
        if (p.motorRequestCount(port) != 0) {
            return true;
        }
    }
    for (std::uint8_t line = 1; line <= 8; ++line) {
        if (p.digitalOutRequestCount(line) != 0 || p.digitalInMade(line)) {
            return true;
        }
    }
    return false;
}

}  // namespace

// ═══ TB8 — the clear confirm (re-check #7) ═══════════════════════════════════════════════

TEST_CASE("Tune TB8: NOTHING is made before stage 1 (READY) -- the request log is EMPTY, a pneumatic line "
          "included; READY makes exactly the declared roles (the claw's line once); the FIRST motion waits "
          "for GO, BACK moves nothing, and GO is asked once per mechanism") {
    // BUG CAUGHT (MB7): the mechanisms made at construction or at the first tick — robot two's pneumatic
    // claw would MOVE at boot (HA-119) before anyone said the robot was clear.
    Bench b{kCascadeAndClaw};
    b.steps(120);
    CHECK(b.program.page() == TunePage::Clear);
    CHECK_FALSE(b.program.made());
    CHECK_FALSE(anyRequest(b.provider));
    // a tap OUTSIDE READY does nothing
    b.screen.tap(470, 10);
    b.steps(1);
    CHECK_FALSE(anyRequest(b.provider));
    b.tap(TuneButton::Ready);
    CHECK(b.program.made());
    CHECK(b.provider.motorRequestCount(1) >= 1);
    CHECK(b.provider.digitalOutRequestCount(1) == 1);  // line A, made once (at its start level)
    for (std::uint8_t port = 2; port <= 21; ++port) {
        CHECK(b.provider.motorRequestCount(port) == 0);
    }
    // stage 2: the first TEST asks; BACK moves nothing
    const std::uint32_t before = b.program.service().slot(0).stepsBegun();
    b.tap(TuneButton::TestB);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.steps(10);
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    b.tap(TuneButton::Back);
    CHECK(b.program.page() == TunePage::Tune);
    CHECK(b.program.service().slot(0).stepsBegun() == before);
    b.steps(20);
    CHECK(std::fabs(b.plantInches()) < 0.05);
    // asked again (BACK confirmed nothing), then GO starts it
    b.tap(TuneButton::TestB);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.tap(TuneButton::Go);
    CHECK(b.program.busy());
    CHECK(b.program.service().slot(0).stepsBegun() == before + 1);
    b.finish();
    CHECK(b.program.page() == TunePage::Report);
    b.tap(TuneButton::Back);
    // the second motion in the session is NOT asked again
    b.tap(TuneButton::TestA);
    CHECK(b.program.page() == TunePage::Tune);
    CHECK(b.program.busy());
}

// ═══ TB2 — ANY controller button stops every mechanism within one tick ═══════════════════

TEST_CASE("Tune TB2: EACH of the twelve controller buttons, pressed mid-TEST, releases the mechanism to its "
          "safe state in the SAME tick and reports the test CANCELLED") {
    // BUG CAUGHT (MB1): one button left out of the STOP — a student's thumb on it, the lift keeps going.
    for (int k = 0; k <= static_cast<int>(ControllerButton::A); ++k) {
        CAPTURE(k);
        Bench b{kTallCascade};
        b.ready();
        b.startTest(TuneButton::TestB);  // up toward 18 in
        b.steps(30);
        REQUIRE(b.program.service().slot(0).running());
        REQUIRE(std::fabs(b.provider.fakeMotor(1).commandedVoltage().value()) > 1.0);
        b.controller.setPressed(static_cast<ControllerButton>(k), true);
        REQUIRE(b.step());  // ONE tick
        CHECK(b.program.service().slot(0).lastStep().outcome == MechanismOutcome::Cancelled);
        CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
        CHECK(b.provider.fakeMotor(1).brakeMode() == shulib::hal::BrakeMode::Hold);  // its declared safe
        CHECK_FALSE(b.program.busy());
        CHECK(b.program.page() == TunePage::Report);
        CHECK(std::string_view{b.program.report().verdict} == "CANCELLED");
        CHECK(std::string_view{b.program.report().stoppedBy} == "a controller button");
        // held, it keeps everything stopped, and a TEST is refused while it is held
        b.steps(5);
        b.tap(TuneButton::Back);
        b.tap(TuneButton::TestA);
        CHECK_FALSE(b.program.busy());
        CHECK(std::string_view{b.program.status()}.find("a controller button is held") != std::string_view::npos);
    }
}

TEST_CASE("Tune TB2b: a button also stops a HOLD after a test and the stick's jog, and the page's STOP does "
          "the same") {
    Bench b{kTallCascade};
    b.ready();
    b.startTest(TuneButton::TestB);
    b.finish();
    b.tap(TuneButton::Back);
    REQUIRE(b.program.service().position(0).holding());
    b.controller.setPressed(ControllerButton::Down, true);
    REQUIRE(b.step());
    CHECK_FALSE(b.program.service().position(0).holding());
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    b.controller.setPressed(ControllerButton::Down, false);
    b.steps(2);
    // the stick jogs it (confirmed already); a button stops the jog at once
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.8);
    b.steps(3);
    CHECK(b.program.service().position(0).heldButtonMoving());
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() > 1.0);
    b.controller.setPressed(ControllerButton::R1, true);
    REQUIRE(b.step());
    CHECK_FALSE(b.program.service().position(0).heldButtonMoving());
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    // and while the button is held the stick moves nothing
    b.steps(10);
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    b.controller.setPressed(ControllerButton::R1, false);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(2);
    // the page's STOP
    b.startTest(TuneButton::TestA);
    b.steps(5);
    b.tapAnyway(TuneButton::Stop);
    CHECK(std::string_view{b.program.report().stoppedBy} == "STOP on the screen");
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
}

// ═══ TB7 — the abort set and the refusals ════════════════════════════════════════════════

TEST_CASE("Tune TB7: the controller DISCONNECTING mid-TEST releases it in the same tick; so does VEXos's "
          "DISABLED bit, and then the loop returns") {
    // BUG CAUGHT (MB6): a dropped controller ignored — the STOP has no hands and the lift keeps moving.
    {
        Bench b{kTallCascade};
        b.ready();
        b.startTest(TuneButton::TestB);
        b.steps(25);
        REQUIRE(b.program.service().slot(0).running());
        b.controller.setConnected(false);
        REQUIRE(b.step());
        CHECK(b.program.service().slot(0).lastStep().outcome == MechanismOutcome::Cancelled);
        CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
        CHECK(std::string_view{b.program.report().stoppedBy} == "the controller disconnected");
    }
    {
        Bench b{kTallCascade};
        b.ready();
        b.startTest(TuneButton::TestB);
        b.steps(25);
        b.io.status = mech::kTuneCompetitionDisabled;
        CHECK_FALSE(b.program.step());  // driver control ended: run() returns
        CHECK(b.program.service().slot(0).lastStep().outcome == MechanismOutcome::Cancelled);
        CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
        CHECK(b.linesContaining("TUNEPG", "VEXos reports the robot DISABLED -- every mechanism released; Tune waits") ==
              1);
        CHECK(std::string_view{b.program.report().stoppedBy} == "VEXos reports the robot DISABLED");
        // autonomous starting ends it too
        b.io.status = mech::kTuneCompetitionAutonomous;
        CHECK_FALSE(b.program.step());
        // driver control back: it carries on (the same mechanisms, the same page)
        b.io.status = 0;
        b.program.enter();
        CHECK(b.step());
        CHECK(b.program.made());
    }
}

TEST_CASE("Tune TB7b: TEST, HOME and STRING CHECK are REFUSED with no controller and with a field or "
          "switch -- each with its sentence -- and nothing moves") {
    Bench b{tune_fixture::kMidHome};  // a stall-homed lift: HOME exists
    b.ready();
    struct Case {
        TuneButton button;
        const char* word;
    };
    const Case cases[] = {{TuneButton::TestA, "TEST A"}, {TuneButton::Home, "HOME"},
                          {TuneButton::StringCheck, "STRING CHECK"}};
    for (const Case& c : cases) {
        CAPTURE(c.word);
        const std::uint32_t begun = b.program.service().slot(0).stepsBegun();
        b.controller.setConnected(false);
        b.steps(2);
        b.tap(c.button);
        std::string expected = std::string{"'mid home lift': "} + c.word +
                               " refused -- no controller is connected (it carries the STOP)";
        CHECK(std::string_view{b.program.status()} == expected);
        CHECK(b.program.page() == TunePage::Tune);
        b.controller.setConnected(true);
        b.io.status = mech::kTuneCompetitionConnected;
        b.steps(2);
        b.tap(c.button);
        expected = std::string{"'mid home lift': "} + c.word +
                   " refused -- a field or competition switch is plugged in (Tune runs at the bench only)";
        CHECK(std::string_view{b.program.status()} == expected);
        CHECK(b.program.page() == TunePage::Tune);
        b.io.status = 0;
        CHECK(b.program.service().slot(0).stepsBegun() == begun);
        CHECK(b.provider.fakeMotor(2).commandedVoltage().value() == 0.0);
        b.steps(2);
    }
    // not homed: a TEST is refused by its own sentence (HOME is the way)
    b.tap(TuneButton::TestA);
    CHECK(std::string_view{b.program.status()} ==
          "'mid home lift': TEST A refused -- it has not found its zero: press HOME first");
}

// ═══ TB3 — a refused change ═══════════════════════════════════════════════════════════════

TEST_CASE("Tune TB3: a refused change shows its sentence and changes NOTHING; during a test every button "
          "but STOP is greyed and a tap on [+] changes nothing") {
    Bench b{kTallCascade};
    b.ready();
    b.select("settle");
    mech::MechanismSpec before;
    std::memcpy(static_cast<void*>(&before), &b.program.tuning().spec(0), sizeof before);
    // settle 0.1 s: two [-] presses reach 0, which the robot file cannot say
    b.tap(TuneButton::Minus);
    CHECK(b.program.tuning().spec(0).position.settleTime == doctest::Approx(0.05));
    mech::MechanismSpec after1;
    std::memcpy(static_cast<void*>(&after1), &b.program.tuning().spec(0), sizeof after1);
    b.tap(TuneButton::Minus);
    CHECK(std::string_view{b.program.status()}.find(mech::liveRefusalText(mech::LiveRefusal::SettleIs0)) !=
          std::string_view::npos);
    CHECK(std::memcmp(&b.program.tuning().spec(0), &after1, sizeof after1) == 0);
    // the refused sentence is ON the screen (the status lines)
    bool shown = false;
    for (std::size_t i = 0; i < b.program.drawn().lineCount; ++i) {
        shown = shown || std::string_view{b.program.drawn().lines[i].text}.find("refused") != std::string_view::npos ||
                std::string_view{b.program.drawn().lines[i].text}.find("settle") != std::string_view::npos;
    }
    CHECK(shown);
    // during a test: greyed, and a tap on [+] is not read
    b.select("kP");
    const double kP = b.program.tuning().spec(0).position.kP;
    b.startTest(TuneButton::TestB);
    b.steps(2);
    for (std::size_t i = 0; i < b.program.drawn().buttonCount; ++i) {
        const mech::TuneButtonView& v = b.program.drawn().buttons[i];
        CAPTURE(v.label);
        CHECK(v.armed == (v.id == TuneButton::Stop));
    }
    b.tapAnyway(TuneButton::Plus);
    CHECK(b.program.tuning().spec(0).position.kP == kP);
    CHECK(b.program.busy());
    (void)before;
}

// ═══ TB4 — the report against an independent computation ════════════════════════════════

TEST_CASE("Tune TB4: a TEST move's report equals an independent computation over the test's OWN samples "
          "(from the plant), on three plant mismatches") {
    // BUG CAUGHT (MB3): the overshoot measured from the START instead of the target (it would read about the
    // whole travel); a settle, peak, final error or hold read from the wrong samples.
    struct Plant {
        const char* what;
        sim::ArmPlantConfig cfg;
    };
    sim::ArmPlantConfig slow = tune_rig::liftPlant(1.6);
    slow.kV *= 1.4;  // losses: a slower lift than its cartridge
    sim::ArmPlantConfig light = tune_rig::liftPlant(0.3);
    light.kA *= 3.0;  // a heavy carriage on a light gravity: it coasts past
    const Plant plants[] = {{"gravity 1.6 V (spec 1.0)", tune_rig::liftPlant(1.6)},
                            {"40 % slower", slow},
                            {"light and heavy-inertia", light}};
    bool anyOvershoot = false;
    for (const Plant& p : plants) {
        for (TuneButton which : {TuneButton::TestB, TuneButton::TestA}) {
            CAPTURE(p.what);
            CAPTURE(static_cast<int>(which));
            Bench b{kTallCascade, p.cfg};
            b.ready();
            if (which == TuneButton::TestA) {
                // go up first, so TEST A comes DOWN to the floor
                b.startTest(TuneButton::TestB);
                b.finish();
                b.tap(TuneButton::Back);
            }
            const double target = which == TuneButton::TestB ? 18.0 : 0.0;
            b.startTest(which);
            const double start = b.startInches;
            b.finish();
            const mech::TuneReport& r = b.program.report();
            const bool ok = std::string_view{r.verdict} == "SUCCEEDED";
            const Independent want = independently(b.rec, start, target, kTallCascade.specs[0].position.tolerance, ok);
            CHECK(r.target == target);
            CHECK(r.overshoot == doctest::Approx(want.overshoot).epsilon(1e-9).scale(1.0));
            CHECK(r.peakAmps == doctest::Approx(want.peak).epsilon(1e-12));
            CHECK(r.finalError == doctest::Approx(want.finalError).epsilon(1e-9).scale(1.0));
            CHECK(r.settle == doctest::Approx(want.settle).epsilon(1e-9));
            CHECK(r.holdAmps == doctest::Approx(want.hold).epsilon(1e-9));
            CHECK(r.seconds == b.program.service().slot(0).lastStep().elapsed);
            // the cascade states NO cap: never a cap warning (R01: `capAmps >= 0` would warn on every test)
            CHECK(r.capAmps == 0.0);
            CHECK_FALSE(r.warnCapNotAbovePeak);
            CHECK(b.linesContaining("TUNEPG", "the cap limited the move") == 0);
            CHECK(b.program.moveSamples().size() + b.program.holdSamples().size() == b.rec.size());
            if (ok) {
                CHECK(b.program.holdSamples().size() == 100);  // the second after it settled, at 10 ms
            }
            anyOvershoot = anyOvershoot || want.overshoot > 1e-3;
        }
    }
    CHECK(anyOvershoot);  // the comparison has seen a real overshoot, not only zeros
}

// ═══ TB9 — a TEST move's time limit ══════════════════════════════════════════════════════

TEST_CASE("Tune TB9: a TEST move's limit is the plan's time x 2 + settle + 0.5 s, at most 10 s; a lift that "
          "cannot reach the target reports TIMED_OUT AT that limit") {
    SUBCASE("the formula, against the test's own plan") {
        Bench b{kTallCascade};
        b.ready();
        const mech::MechanismSpec& s = b.program.tuning().spec(0);
        b.startTest(TuneButton::TestB);
        const double fromDeg = b.startInches * kDegPerIn;  // the joint as the test's tick began (the plant's)
        const double scale = kDegPerIn;  // a spool: exactly its degrees per inch
        const shulib::control::TrapezoidProfile plan{
            18.0 * kDegPerIn - fromDeg, {s.position.maxVelocity * scale, s.position.maxAcceleration * scale}};
        const double want = std::min(10.0, 2.0 * plan.duration() + s.position.settleTime + 0.5);
        CHECK(b.program.lastBound() == doctest::Approx(want).epsilon(1e-9));
        CHECK(b.program.lastBound() < 10.0);
    }
    SUBCASE("blocked at 10 in: TIMED_OUT at the limit, within one tick") {
        sim::ArmPlantConfig blocked = tune_rig::liftPlant();
        blocked.maxStopDeg = 10.0 * kDegPerIn;
        blocked.stallAmps = 2.0;  // a blocked motor draws at most 2.0 A here ...
        Bench b{kTallCascade, blocked};
        b.ready();
        // ... and its stall signature is raised above that live, so the LIMIT ends the move, not a STALL
        REQUIRE(b.program.tuning().apply(0, mech::Tunable::StallAmps, 2.4).ok);
        b.startTest(TuneButton::TestB);
        b.finish();
        const mech::TuneReport& r = b.program.report();
        CHECK(std::string_view{r.verdict} == "TIMED_OUT");
        CHECK(r.seconds >= r.bound - 1e-9);
        CHECK(r.seconds < r.bound + 0.01 + 1e-9);
        CHECK(r.holdAmps < 0.0);
    }
    SUBCASE("a slow live top speed: the limit is EXACTLY 10 s") {
        Bench b{kTallCascade};
        b.ready();
        // 0.5 in/s over 18 in: a 36 s plan -> capped
        REQUIRE(b.program.tuning().apply(0, mech::Tunable::MaxSpeed, 0.5).ok);
        b.startTest(TuneButton::TestB);
        CHECK(b.program.lastBound() == 10.0);
    }
}

// ═══ TB11 — the hold current and its warning ════════════════════════════════════════════

TEST_CASE("Tune TB11: the report's HOLD current, and a WARNING when the stall signature is not above it "
          "(a piece on the plant raising the hold) -- and none on a light hold; the cap warning likewise") {
    {
        Bench b{kLowSignature};
        b.ready();
        b.arm->addLoadVolts(1.6);  // a game piece on the carriage
        b.startTest(TuneButton::TestB);
        b.finish();
        const mech::TuneReport& r = b.program.report();
        REQUIRE(std::string_view{r.verdict} == "SUCCEEDED");
        CHECK(r.holdAmps > 0.6);
        CHECK(r.warnStallNotAboveHold);
        CHECK(b.linesContaining("TUNEPG", "is not above its hold") == 1);
        b.steps(1);  // the REPORT page is painted on the tick after the one that finished the test
        REQUIRE(b.program.drawn().page == TunePage::Report);
        bool onPage = false;
        for (std::size_t i = 0; i < b.program.drawn().lineCount; ++i) {
            onPage = onPage || std::string_view{b.program.drawn().lines[i].text}.rfind("WARNING: stall", 0) == 0;
        }
        CHECK(onPage);
    }
    {
        Bench b{kLowSignature};
        b.ready();
        b.startTest(TuneButton::TestB);
        b.finish();
        const mech::TuneReport& r = b.program.report();
        REQUIRE(std::string_view{r.verdict} == "SUCCEEDED");
        CHECK(r.holdAmps >= 0.0);
        CHECK(r.holdAmps < 0.6);
        CHECK_FALSE(r.warnStallNotAboveHold);
        CHECK(b.linesContaining("TUNEPG", "is not above its hold") == 0);
    }
    {
        Bench b{kCappedLift};
        b.ready();
        b.startTest(TuneButton::TestB);
        b.finish();
        const mech::TuneReport& r = b.program.report();
        CHECK(r.capAmps == 0.6);
        CHECK(r.peakAmps >= 0.6 - 1e-9);  // the plant holds the current at the cap while it accelerates
        CHECK(r.warnCapNotAbovePeak);
        CHECK(b.linesContaining("TUNEPG", "the cap limited the move") == 1);
    }
}

// ═══ TB5 — never the drive ═════════════════════════════════════════════════════════════════

TEST_CASE("Tune TB5: over ROBOT TWO's own table the provider is asked ONLY for its mechanisms' ports -- "
          "never a drive motor, never the IMU -- through READY, a TEST, HOME's refusal, COPY and EXIT") {
    // BUG CAUGHT (MB4): the program building the Robot (or any drive motor) — a robot on blocks is not assumed.
    Bench b{shulib::bench::robot_two::kRobotTwoMechanisms};
    b.ready();
    b.tap(TuneButton::TestA);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.tap(TuneButton::Go);
    b.finish();
    b.tap(TuneButton::Back);
    b.tap(TuneButton::Copy);
    b.tap(TuneButton::NextMech);
    b.tap(TuneButton::NextMech);
    b.tap(TuneButton::Exit);
    for (std::uint8_t port = 1; port <= 21; ++port) {
        CAPTURE(static_cast<int>(port));
        const bool mechanism = port == 6 || port == 7;
        if (mechanism) {
            CHECK(b.provider.motorRequestCount(port) >= 1);
        } else {
            CHECK(b.provider.motorRequestCount(port) == 0);
        }
    }
    CHECK(b.provider.digitalOutRequestCount(1) == 1);
    for (std::uint8_t line = 2; line <= 8; ++line) {
        CHECK(b.provider.digitalOutRequestCount(line) == 0);
    }
    // robot two's drive ports, by name: none asked
    const auto& chassis = shulib::bench::kRobotTwoChassis;
    for (std::size_t i = 0; i < chassis.leftCount; ++i) {
        CHECK(b.provider.motorRequestCount(static_cast<std::uint8_t>(shulib::bench::absPort(chassis.left[i]))) == 0);
    }
    for (std::size_t i = 0; i < chassis.rightCount; ++i) {
        CHECK(b.provider.motorRequestCount(static_cast<std::uint8_t>(shulib::bench::absPort(chassis.right[i]))) == 0);
    }
}

// ═══ TB10 — every limit Tune changed is written back ═════════════════════════════════════

TEST_CASE("Tune TB10: RESET, LEAVING the mechanism's page and EXIT each write back every current cap Tune "
          "changed -- the robot file's cap, or the motor's 2.5 A default when it states none") {
    // BUG CAUGHT (MB8): EXIT leaves a changed limit in the motor — the next program (stating no cap)
    // writes nothing, and the claw runs a match at 1 A (HA-220).
    const auto capOf = [](Bench& b, std::uint8_t port) { return b.provider.fakeMotor(port).currentLimit().value(); };
    SUBCASE("EXIT: a cap ADDED to a lift whose file states none, and a STATED cap lowered") {
        Bench b{kTwoLifts};
        b.ready();
        // the cascade states no cap: one added live (above its 2.0 A stall signature — set outright here;
        // on the page a [-] from none starts at 2.45 A and a [+] from none, 2.55 A, is refused: the review
        // round's case "a cap the robot file does not state STARTS with [-]"), and the other lift's stated
        // 1.5 A lowered — both while the cascade's page is shown (leaving a page writes its cap back on its
        // own: the next subcase)
        REQUIRE(b.program.tuning().apply(0, mech::Tunable::CurrentCap, 2.2).ok);
        REQUIRE(capOf(b, 1) == doctest::Approx(2.2));
        REQUIRE(b.program.tuning().apply(1, mech::Tunable::CurrentCap, 1.45).ok);
        REQUIRE(capOf(b, 2) == doctest::Approx(1.45));
        const int writes1 = b.provider.fakeMotor(1).currentLimitWrites();
        const int writes2 = b.provider.fakeMotor(2).currentLimitWrites();
        b.tap(TuneButton::Exit);
        CHECK(b.program.page() == TunePage::Exited);
        CHECK(capOf(b, 1) == 2.5);  // the motor's default: the file states none
        CHECK(b.provider.fakeMotor(1).currentLimitWrites() > writes1);
        CHECK(b.program.tuning().spec(0).currentCapAmps == 0.0);
        CHECK(capOf(b, 2) == doctest::Approx(1.5));
        CHECK(b.provider.fakeMotor(2).currentLimitWrites() > writes2);
        CHECK(b.program.tuning().spec(1).currentCapAmps == 1.5);
        CHECK(b.linesContaining("TUNEPG", "EXIT: 2 current caps written back") == 1);
    }
    SUBCASE("leaving the page, a STATED cap") {
        Bench b{kTwoLifts};
        b.ready();
        b.tap(TuneButton::NextMech);
        REQUIRE(b.program.shown() == 1);
        b.select("current cap");
        b.tap(TuneButton::Minus);
        REQUIRE(b.program.tuning().spec(1).currentCapAmps == doctest::Approx(1.45));
        REQUIRE(capOf(b, 2) == doctest::Approx(1.45));
        b.tap(TuneButton::PrevMech);
        CHECK(b.program.shown() == 0);
        CHECK(capOf(b, 2) == doctest::Approx(1.5));
        CHECK(b.program.tuning().spec(1).currentCapAmps == 1.5);
        CHECK(b.program.capsWrittenBack() == 1);
    }
    SUBCASE("RESET") {
        Bench b{kTwoLifts};
        b.ready();
        REQUIRE(b.program.tuning().apply(0, mech::Tunable::CurrentCap, 2.2).ok);
        REQUIRE(capOf(b, 1) < 2.5);
        b.tap(TuneButton::Reset);
        CHECK(capOf(b, 1) == 2.5);
        CHECK_FALSE(b.program.tuning().changed(0));
    }
}

TEST_CASE("Tune TB10b: the SD copy is /usd/tune_log.txt APPENDED -- two boots' COPY blocks are both in the "
          "file (through the PROS shell over the host shim)") {
    namespace fs = std::filesystem;
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const fs::path dir = fs::temp_directory_path() / "shulib_tune1b1_logcopy";
    fs::create_directories(dir);
    const std::string root = dir.string() + "/";
    fs::remove(dir / "tune_log.txt");
    for (int boot = 1; boot <= 2; ++boot) {
        shulib::hal::pros::ProsTuneShell shell{tune_fixture::kCascade, "stamp", root.c_str()};
        REQUIRE(shell.devices().serialCopyOpen());
        CHECK(shell.devices().serialCopyAppendOpen() == (boot == 1 ? shulib::hal::pros::AppendOpen::Created
                                                                    : shulib::hal::pros::AppendOpen::Appended));
        REQUIRE(shell.program().tuning().count() == 1);
        shell.program().bootBanner(boot == 1 ? "boot-one" : "boot-two", "0x00");
        (void)shell.program().copy(0);
    }
    std::ifstream in(dir / "tune_log.txt", std::ios::binary);
    const std::string onCard((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const std::size_t one = onCard.find("boot-one");
    const std::size_t two = onCard.find("boot-two");
    CHECK(one != std::string::npos);
    CHECK(two != std::string::npos);
    CHECK(one < two);
    // both boots' COPY blocks (begin and end lines), in order
    std::size_t blocks = 0;
    for (std::size_t at = onCard.find("copy ends"); at != std::string::npos; at = onCard.find("copy ends", at + 1)) {
        ++blocks;
    }
    CHECK(blocks == 2);
    CHECK(onCard.find("'cascade': copy begins") > one);
}

// ═══ COPY, EXIT, the stick (the program's flow) ══════════════════════════════════════════

TEST_CASE("Tune: COPY logs the mechanism's [TUNE] block through the provider's sink and says how many "
          "lines; an untouched mechanism copies 0 lines") {
    Bench b{kTallCascade};
    b.ready();
    b.tap(TuneButton::Copy);
    CHECK(std::string_view{b.program.status()} == "'tall cascade': COPY -- 0 lines: its numbers are the robot file's");
    CHECK(b.linesContaining("TUNE", "copy ends -- 0 lines") == 1);
    b.select("kP");
    b.tap(TuneButton::Plus);
    b.tap(TuneButton::Copy);
    CHECK(std::string_view{b.program.status()} ==
          "'tall cascade': COPY -- 1 line to the terminal and /usd/tune_log.txt: put them in the robot file");
    CHECK(b.linesContaining("TUNE", ".tuning.kP = 0.3125") == 1);
    CHECK_FALSE(b.program.uncopied(0));
    // the page shows the sentence WITHOUT the shown mechanism's name (the title line has it), whole (a
    // changed line is repainted at most every 0.1 s: 12 ticks)
    b.steps(12);
    CHECK(b.program.drawn().lines[std::size_t{6}].text == std::string{"COPY -- 1 line to the terminal and"});
    CHECK(b.program.drawn().lines[std::size_t{7}].text == std::string{"/usd/tune_log.txt: put them in the robot file"});
}

TEST_CASE("Tune: the stick waits for stage 2, jogs only past its deadband, holds where it is when let go; a "
          "tap ends a jog first, and no TEST starts while the stick is pushed") {
    // BUG CAUGHT: a jog left running when a tap changed the page or the mechanism (the stick's release would
    // then act on another mechanism, or cancel a test).
    Bench b{kTallCascade};
    b.ready();
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.1);  // inside the deadband: nothing
    b.steps(5);
    CHECK(b.program.page() == TunePage::Tune);
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.5);
    b.steps(1);
    CHECK(b.program.page() == TunePage::Confirm);  // its first motion asks
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.tap(TuneButton::Go);
    const double from = b.plantInches();
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(2);
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == doctest::Approx(mech::kTuneJogMostVolts));
    b.steps(60);
    CHECK(b.program.service().position(0).heldButtonMoving());
    CHECK(b.plantInches() > from + 0.5);
    // a TEST tapped while the stick moves it is refused — releasing the stick later must never cancel a
    // test the stick did not start
    b.tapAnyway(TuneButton::TestA);
    CHECK_FALSE(b.program.busy());
    CHECK(std::string_view{b.program.status()} ==
          "'tall cascade': TEST A refused -- the stick is pushed (it moves the mechanism): let go of it first");
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(2);
    CHECK(b.program.service().position(0).holding());
    // a range change while the stick moves it is refused by live tuning (rule 3)
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.5);
    b.steps(2);
    const mech::Applied a = b.program.tuning().apply(0, mech::Tunable::RangeMax, 23.0);
    CHECK_FALSE(a.ok);
    CHECK(a.why == mech::LiveRefusal::HeldButtonMoving);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(2);
}

TEST_CASE("Tune: EXIT releases everything and says so; TUNE AGAIN returns to the page") {
    Bench b{kTallCascade};
    b.ready();
    b.startTest(TuneButton::TestB);
    b.finish();
    b.tap(TuneButton::Back);
    REQUIRE(b.program.service().position(0).holding());
    b.tap(TuneButton::Exit);
    CHECK(b.program.page() == TunePage::Exited);
    CHECK_FALSE(b.program.service().position(0).holding());
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    b.tap(TuneButton::TuneAgain);
    CHECK(b.program.page() == TunePage::Tune);
}

TEST_CASE("Tune: a tap on > while the stick jogs a mechanism ends THAT jog first -- the old mechanism holds, "
          "the new one does not move, and a NEW push asks about it") {
    // BUG CAUGHT: the jog left running on the mechanism the page left (its stick release would then stop
    // the NEW mechanism, and the old one would go on at its last volts).
    Bench b{kTwoLifts};
    b.ready();
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(1);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.tap(TuneButton::Go);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(20);
    REQUIRE(b.program.service().position(0).heldButtonMoving());
    b.tap(TuneButton::NextMech);
    CHECK(b.program.shown() == 1);
    CHECK_FALSE(b.program.service().position(0).heldButtonMoving());
    CHECK(b.program.service().position(0).holding());
    // the stick, still pushed, does NOT move 'mid home lift' (its first motion is not confirmed) — and asks
    // only on a NEW push
    b.steps(5);
    CHECK(b.program.page() == TunePage::Tune);
    CHECK(b.provider.fakeMotor(2).commandedVoltage().value() == 0.0);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(1);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(1);
    CHECK(b.program.page() == TunePage::Confirm);
    CHECK(b.provider.fakeMotor(2).commandedVoltage().value() == 0.0);
}

// ═══ The hold that ENDS inside the report's second (delta 8; V1's case) ════════════════════

namespace {
// a lift whose stall signature (0.6 A, lasting 0.3 s — SHORTER than the report's second) sits under an
// ordinary hold with a piece on it: the signature trips on the hold itself
inline constexpr Lift shortSignature{
    .name = "short sig lift",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 0.6, .belowMotorRpm = 5, .lasting = 0.3_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
    .presets = {{"floor", 0_in}, {"mid", 12_in}},
};
inline constexpr MechanismTable kShortSignature = mechanisms(wire(shortSignature, motors(1)));
// a cascade homed by stalling gently DOWN into its bottom (robot one's plan, 2026-11-10): HOME run to SUCCEEDED
inline constexpr Lift stallHomed{
    .name = "stall homed lift",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = homeByStall(0_in, -4_volt, {.amps = 0.5, .belowMotorRpm = 5, .lasting = 0.2_s}),
    .presets = {{"floor", 0_in}, {"mid", 12_in}},
};
inline constexpr MechanismTable kStallHomed = mechanisms(wire(stallHomed, motors(1)));
}  // namespace

TEST_CASE("Tune: a hold that ENDS inside the second after the move settled (its stall signature tripping on "
          "the hold) is reported as such -- hold n/a, the stall warning, its own line -- never as a hold") {
    // BUG CAUGHT: the mean of a cut hold reported as the hold (a partial second, ending at 0 V, reads LOW —
    // under the signature it just tripped) with no warning: the student keeps a signature an ordinary hold trips.
    Bench b{kShortSignature};
    b.ready();
    b.arm->addLoadVolts(1.6);  // a game piece on the carriage
    b.startTest(TuneButton::TestB);
    b.finish();
    const mech::TuneReport& r = b.program.report();
    REQUIRE(std::string_view{r.verdict} == "SUCCEEDED");  // the MOVE succeeded; its hold did not last
    CHECK(r.holdAmps < 0.0);
    CHECK(r.warnStallNotAboveHold);
    CHECK(b.program.service().slot(0).lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(b.linesContaining("TUNEPG", "WARNING -- its hold ENDED inside the second after it settled") == 1);
    CHECK(b.linesContaining("TUNEPG", "is not above its hold") == 0);  // not the mean's warning: there is none
    CHECK(r.holdEnded);
    CHECK_FALSE(r.holdStopped);
    // the PAGE says it too — never "did not settle" under SUCCEEDED, never the -1 as a hold
    b.steps(1);
    REQUIRE(b.program.drawn().page == TunePage::Report);
    std::string page;
    for (std::size_t i = 0; i < b.program.drawn().lineCount; ++i) {
        page += b.program.drawn().lines[i].text;
        page += '\n';
    }
    INFO(page);
    CHECK(page.find("SUCCEEDED") != std::string::npos);
    CHECK(page.find("; hold ENDED (the stall tripped on it)\n") != std::string::npos);
    CHECK(page.find("WARNING: stall 0.60 A trips on the hold: raise it\n") != std::string::npos);
    CHECK(page.find("-1.00") == std::string::npos);
    CHECK(page.find("did not settle") == std::string::npos);
}

// ═══ Guide 11's TUNEPG lines are the program's own (§3.4: "a test pins it") ═══════════════

namespace {
/// A guide chapter with every run of whitespace folded to one space (a line the guide wraps still matches).
std::string guideText(const std::string& file) {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/docs/guide/" + file};
    REQUIRE(in.good());
    const std::string raw{std::istreambuf_iterator<char>{in}, std::istreambuf_iterator<char>{}};
    std::string out;
    bool space = false;
    for (const char c : raw) {
        const bool ws = c == ' ' || c == '\n' || c == '\t';
        if (ws && !space) {
            out += ' ';
        } else if (!ws) {
            out += c;
        }
        space = ws;
    }
    return out;
}
/// A line's SHAPE: a quoted mechanism name ('lift': ...) -> 'N', a bracketed beacon -> [B], every number -> #. Everything
/// else — every word, every separator, every unit, a plural's s — must be the program's, character for
/// character.
std::string shapeOf(std::string_view line) {
    std::string out;
    std::size_t i = 0;
    const auto digit = [](char c) { return c >= '0' && c <= '9'; };
    while (i < line.size()) {
        const char c = line[i];
        if (c == '\'' && (i == 0 || line[i - 1] == ' ') && line.find("':", i + 1) != std::string_view::npos) {
            out += "'N'";  // a name opens at a word's start and closes at "':" (never an apostrophe: "file's")
            i = line.find("':", i + 1) + 1;
        } else if (c == '[' && line.find(']', i + 1) != std::string_view::npos) {
            out += "[B]";
            i = line.find(']', i + 1) + 1;
        } else if (digit(c) || ((c == '+' || c == '-') && i + 1 < line.size() && digit(line[i + 1]))) {
            ++i;
            while (i < line.size() && (digit(line[i]) || (line[i] == '.' && i + 1 < line.size() && digit(line[i + 1])))) {
                ++i;
            }
            out += '#';
        } else {
            out += c;
            ++i;
        }
    }
    return out;
}
void collect(const Bench& b, std::vector<std::string>& into) {
    for (int i = 0; i < b.sink.size(); ++i) {
        if (b.sink.at(i).subsystem == mech::kTuneTag) {
            into.push_back(b.sink.at(i).message);
        }
    }
}
}  // namespace

TEST_CASE("Tune: EVERY TUNEPG line guide 11 quotes is one the program logs -- the same words, separators, "
          "units and plurals, numbers and names aside -- and every STOP cause it names is the program's") {
    // BUG CAUGHT: guide 11 teaching a line the program does not print (a reworded log line, a guide example
    // typed by hand) — a student at the terminal looks for words that never come.
    std::vector<std::string> logged;
    {  // the boot and READY (two mechanisms: the plural)
        Bench b{kCascadeAndClaw};
        b.program.bootBanner("tank-tune", "0x00");
        b.ready();
        collect(b, logged);
    }
    {  // a change, TEST B to 'stack 3' and its report, the refusal, a cap written back, every STOP cause, EXIT
        Bench b{kTallCascade};
        b.ready();
        b.select("kP");
        b.tap(TuneButton::Plus);
        b.startTest(TuneButton::TestB);
        b.finish();
        b.tap(TuneButton::Back);
        b.controller.setPressed(ControllerButton::B, true);
        b.steps(2);
        b.controller.setPressed(ControllerButton::B, false);
        b.steps(2);
        b.controller.setConnected(false);
        b.steps(2);
        b.tap(TuneButton::TestA);
        b.controller.setConnected(true);
        b.io.status = mech::kTuneCompetitionConnected;
        b.steps(2);
        b.io.status = 0;
        b.steps(2);
        b.tapAnyway(TuneButton::Stop);
        REQUIRE(b.program.tuning().apply(0, mech::Tunable::CurrentCap, 2.2).ok);
        b.tap(TuneButton::Exit);
        b.tap(TuneButton::TuneAgain);
        b.io.status = mech::kTuneCompetitionDisabled;
        CHECK_FALSE(b.step());
        collect(b, logged);
    }
    {  // autonomous starting (a field's): every mechanism released, the loop returns
        Bench b{kTallCascade};
        b.ready();
        b.io.status = mech::kTuneCompetitionAutonomous;
        CHECK_FALSE(b.step());
        collect(b, logged);
    }
    for (const MechanismTable* t : {&kLowSignature, &kCappedLift, &kShortSignature}) {  // the three warnings
        Bench b{*t};
        b.ready();
        if (t != &kCappedLift) {
            b.arm->addLoadVolts(1.6);
        }
        b.startTest(TuneButton::TestB);
        b.finish();
        collect(b, logged);
    }
    {  // HOME, run to its verdict (a stall-homed lift)
        Bench b{kStallHomed};
        b.ready();
        b.tap(TuneButton::Home);
        REQUIRE(b.program.page() == TunePage::Confirm);
        b.tap(TuneButton::Go);
        b.finish();
        collect(b, logged);
    }
    {  // a cap going back as its page is left (COPIED first), an UNDO waiting on it, a READY refused
        Bench b{kTwoLifts};
        b.ready();
        b.tap(TuneButton::NextMech);
        b.select("current cap");
        b.tap(TuneButton::Minus);
        b.tap(TuneButton::PrevMech);
        b.tap(TuneButton::Undo);
        collect(b, logged);
        Bench r{kCascadeAndClaw};
        r.steps(1);
        r.waitArmed(TuneButton::Ready);
        r.controller.setConnected(false);
        r.tapAnyway(TuneButton::Ready);
        collect(r, logged);
    }
    {  // EXIT with a cap that cannot go back by itself: COPIED, then RESET
        Bench b{kCappedLift};
        b.ready();
        REQUIRE(b.program.tuning().apply(0, mech::Tunable::CurrentCap, 1.0).ok);
        REQUIRE(b.program.tuning().apply(0, mech::Tunable::StallAmps, 0.8).ok);
        b.tap(TuneButton::Exit);
        collect(b, logged);
    }
    std::vector<std::string> shapes;
    for (const std::string& l : logged) {
        shapes.push_back(shapeOf(l));
    }
    const auto loggedWhole = [&](std::string_view guideLine) {
        const std::string want = shapeOf(guideLine);
        return std::find(shapes.begin(), shapes.end(), want) != shapes.end();
    };
    const auto loggedStarting = [&](std::string_view guidePrefix) {
        const std::string want = shapeOf(guidePrefix);
        return std::any_of(shapes.begin(), shapes.end(), [&](const std::string& s) { return s.rfind(want, 0) == 0; });
    };
    const std::string g11 = guideText("11-reading-the-diagnostics.md");
    const char* whole[] = {
        "booted: shulib TUNE build [<beacon>] -- live tuning of this robot's 3 mechanisms; NOTHING is made until "
        "READY on the screen",
        "READY: 3 of 3 mechanisms made (no drive motor is ever made here); each one's first motion waits for GO",
        "'lift': kP 0.25 -> 0.3125",
        "'lift': TEST B -- to stack 3 18.000 in, time limit 7.09 s",
        "'lift': TEST B to 18.000 in: SUCCEEDED in 3.21 s (limit 7.09 s)",
        "'lift': report -- overshoot 0.0214 in, settle 0.25 s, peak 1.62 A, final error +0.0039 in, hold 0.41 A "
        "(mean over the second after it settled; -1 = n/a)",
        "'lift': WARNING -- its stall signature 0.60 A is not above its hold 0.85 A: an ordinary hold can trip it "
        "(STALLED)",
        "'lift': WARNING -- its current cap 0.60 A is not above the move's peak 0.60 A: the cap limited the move",
        "'lift': HOME SUCCEEDED in 2.31 s, its push's peak 0.82 A -- its zero is found",
        "STOP -- a controller button: every mechanism released (its declared safe state)",
        "'lift': TEST A refused -- no controller is connected (it carries the STOP)",
        "EXIT: 1 current cap written back; every mechanism released -- you may stop the program now",
        "'lift': its cap goes back to the robot file's, so its changed numbers are COPIED first (below)",
        "'lift': UNDO waits on another page -- the next undo would change its current cap (only its own page may "
        "change a cap): show it (< >) and UNDO there",
        "READY refused -- no controller is connected (it carries the STOP); nothing was made",
        "EXIT: 1 mechanism's changed numbers COPIED above -- put them into the robot file by hand",
        "EXIT: 'lift': RESET to the robot file's numbers -- a live signature is at or above the file's cap (its "
        "numbers were COPIED above)",
    };
    for (const char* line : whole) {
        INFO(std::string{line});
        CHECK(g11.find(std::string{"`"} + line + "`") != std::string::npos);
        CHECK(loggedWhole(line));
    }
    // the two the guide cuts with "…": their words up to the cut
    const std::pair<const char*, const char*> cut[] = {
        {"'lift': WARNING -- its hold ENDED inside the second after it settled …",
         "'lift': WARNING -- its hold ENDED inside the second after it settled "},
        {"'lift': its current cap back to the robot file's (…) -- …", "'lift': its current cap back to the robot file's ("},
    };
    for (const auto& [quoted, prefix] : cut) {
        INFO(std::string{quoted});
        CHECK(g11.find(std::string{"`"} + quoted + "`") != std::string::npos);
        CHECK(loggedStarting(prefix));
    }
    // every STOP cause the guide names, as the program words it
    const char* causes[] = {"the controller disconnected", "a field or switch was plugged in", "STOP on the screen",
                            "EXIT", "VEXos reports the robot DISABLED", "autonomous started"};
    for (const char* cause : causes) {
        INFO(std::string{cause});
        CHECK(g11.find(std::string{"`"} + cause + "`") != std::string::npos);
        CHECK(loggedWhole(std::string{"STOP -- "} + cause + ": every mechanism released (its declared safe state)"));
    }
}

// ═══ The review round (2026-11-11): each case below was a reviewer's finding — a rule no test held ══════

namespace {
// two lifts that rest on their stops (both homed at READY): the stick carried across a mechanism change
inline constexpr Lift secondLift{
    .name = "second lift",
    .motors = {Forward},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
    .presets = {{"floor", 0_in}, {"top", 20_in}},
};
inline constexpr MechanismTable kTwoHomedLifts =
    mechanisms(wire(tune_fixture::cascade, motors(1)), wire(secondLift, motors(2)));
// a power mechanism whose most volts is BELOW the stick's 6 V
inline constexpr Power weakPivot{
    .name = "weak pivot", .motors = {Forward}, .cartridge = Red, .safe = Hold, .maxVolts = 4_volt};
inline constexpr MechanismTable kWeakPivot = mechanisms(wire(weakPivot, motors(3)));

/// Every line the page last drew, one per line.
std::string drawnText(const Bench& b) {
    std::string out;
    for (std::size_t i = 0; i < b.program.drawn().lineCount; ++i) {
        out += b.program.drawn().lines[i].text;
        out += '\n';
    }
    return out;
}
/// The index of the first logged line (any tag) containing `text`, or -1.
int firstLine(const Bench& b, std::string_view text) {
    for (int i = 0; i < b.sink.size(); ++i) {
        if (b.sink.at(i).message.find(text) != std::string::npos) {
            return i;
        }
    }
    return -1;
}
/// Confirm the shown mechanism's first motion through the STICK (push, GO, let go) — moving nothing.
void confirmByStick(Bench& b) {
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(1);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.tap(TuneButton::Go);
    REQUIRE(b.program.confirmed(b.program.shown()));
}
}  // namespace

// ── the report's pure parts at their edges ──────────────────────────────────────────────────

TEST_CASE("Tune review: tuneTargets for EVERY kind -- two presets, one preset (a quarter and three quarters of "
          "the range; half and nine tenths of the top speed), and a power run's + and - its most volts") {
    // BUG CAUGHT (R12, R15): TEST B on a claw spinning the same way as TEST A; a lift with one preset tested
    // to the wrong place.
    const mech::TuneTargets two = mech::tuneTargets(kTallCascade.specs[0]);
    CHECK(two.a == 0.0);
    CHECK(two.b == 18.0);
    CHECK(std::string_view{two.aName} == "floor");
    CHECK(std::string_view{two.bName} == "stack 3");
    const mech::MechanismSpec& one = tune_fixture::kMidHome.specs[0];  // range 0..20 in, one preset
    REQUIRE(one.position.presetCount == 1);
    const mech::TuneTargets q = mech::tuneTargets(one);
    CHECK(q.a == 5.0);
    CHECK(q.b == 15.0);
    CHECK(std::string_view{q.aName} == "1/4 range");
    CHECK(std::string_view{q.bName} == "3/4 range");
    const mech::MechanismSpec& fly = tune_fixture::kFlyAndPivot.specs[0];  // one preset
    REQUIRE(fly.speed.presetCount == 1);
    const mech::TuneTargets f = mech::tuneTargets(fly);
    CHECK(f.a == 0.5 * fly.speed.maxRpm);
    CHECK(f.b == 0.9 * fly.speed.maxRpm);
    const mech::TuneTargets p = mech::tuneTargets(shulib::bench::robot_two::kRobotTwoMechanisms.specs[0]);
    CHECK(p.a == 12.0);
    CHECK(p.b == -12.0);
    const mech::TuneTargets w = mech::tuneTargets(kWeakPivot.specs[0]);
    CHECK(w.a == 4.0);
    CHECK(w.b == -4.0);
}

TEST_CASE("Tune review: the report's warnings at their EQUALITY edges -- a hold EXACTLY at the signature warns; "
          "no signature or no cap never warns, whatever the currents") {
    // BUG CAUGHT (R03): `stall < hold` — a hold exactly at the signature trips it and was not warned; (R02 /
    // R01): a 0 (none) signature or cap warned on every test.
    std::vector<mech::TuneSample> move{{0.01, 0.0, 0.5}, {0.02, 12.0, 0.8}};
    std::vector<mech::TuneSample> hold(100, mech::TuneSample{0.5, 12.0, 0.5});
    mech::TuneMoveFacts f{};
    f.test = mech::TuneTest::B;
    f.kind = mech::Kind::Position;
    f.target = 12.0;
    f.within = 0.1;
    f.outcome = MechanismOutcome::Succeeded;
    f.stallAmps = 0.5;  // the hold's mean, EXACTLY
    mech::TuneReport r = mech::tuneReportFrom(move, hold, f);
    REQUIRE(r.holdAmps == 0.5);
    CHECK(r.warnStallNotAboveHold);
    f.stallAmps = 0.5000001;  // just above: no warning
    CHECK_FALSE(mech::tuneReportFrom(move, hold, f).warnStallNotAboveHold);
    f.stallAmps = 0.0;  // states none
    CHECK_FALSE(mech::tuneReportFrom(move, hold, f).warnStallNotAboveHold);
    f.capAmps = 0.8;  // the peak, EXACTLY
    CHECK(mech::tuneReportFrom(move, hold, f).warnCapNotAbovePeak);
    f.capAmps = 0.0;  // states none
    CHECK_FALSE(mech::tuneReportFrom(move, hold, f).warnCapNotAbovePeak);
    // a speed mechanism with no signature at speed: never the stall warning
    f.kind = mech::Kind::Speed;
    CHECK_FALSE(mech::tuneReportFrom(move, hold, f).warnStallNotAboveHold);
}

// ── the stick ───────────────────────────────────────────────────────────────────────────────

TEST_CASE("Tune review: the stick's deadband EXACTLY -- at 0.15 nothing moves, just past it the shown lift "
          "jogs") {
    // BUG CAUGHT (R10): any deadband between 0.1 and 0.5 passed the stick test.
    Bench b{kTallCascade};
    b.ready();
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, mech::kTuneJogDeadband);
    b.steps(5);
    CHECK(b.program.page() == TunePage::Tune);  // not even stage 2's question
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, mech::kTuneJogDeadband + 1e-4);
    b.steps(1);
    CHECK(b.program.page() == TunePage::Confirm);
}

TEST_CASE("Tune review: on a POWER mechanism (robot two's claw pivot) the stick drives it at the stick's 6 V and "
          "STOPS it in the tick it is let go; on one whose most is 4 V, at 4 V") {
    // BUG CAUGHT (R06, R13, R07): the claw driven on after the stick was let go (PowerSlot::drive is
    // open-ended); the stick doing nothing on a claw; the stick past a mechanism's own most volts.
    Bench b{shulib::bench::robot_two::kRobotTwoMechanisms};
    b.ready();
    REQUIRE(std::string_view{b.program.tuning().spec(0).name} == "claw pivot 6");
    confirmByStick(b);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(5);
    CHECK(b.provider.fakeMotor(6).commandedVoltage().value() == doctest::Approx(mech::kTuneJogMostVolts));
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, -0.5);  // still pushed (no new edge): it follows
    b.steps(1);
    CHECK(b.provider.fakeMotor(6).commandedVoltage().value() == doctest::Approx(-0.5 * mech::kTuneJogMostVolts));
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    REQUIRE(b.step());  // ONE tick
    CHECK(b.provider.fakeMotor(6).commandedVoltage().value() == 0.0);
    b.steps(10);
    CHECK(b.provider.fakeMotor(6).commandedVoltage().value() == 0.0);
    CHECK(b.provider.fakeMotor(7).commandedVoltage().value() == 0.0);  // the other pivot never moved

    Bench w{kWeakPivot};
    w.ready();
    confirmByStick(w);
    w.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    w.steps(3);
    CHECK(w.provider.fakeMotor(3).commandedVoltage().value() == doctest::Approx(4.0));
    // HALF the stick is half of ITS most (2 V), not half of the stick's 6 V clamped to 4 (3 V): the stick's
    // full travel spans the mechanism's own range (R07 — at full deflection the service's clamp hides it)
    w.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.5);
    w.steps(2);
    CHECK(w.provider.fakeMotor(3).commandedVoltage().value() == doctest::Approx(2.0));
}

TEST_CASE("Tune review: the stick moves NOTHING on the EXITED, REPORT and CONFIRM pages -- and a stick held "
          "through a page change moves nothing until it is let go and pushed again") {
    // BUG CAUGHT (R23): after EXIT ("you may stop the program now") a stick push jogged the lift at 6 V.
    Bench b{kTallCascade};
    b.ready();
    confirmByStick(b);
    b.tap(TuneButton::Exit);
    REQUIRE(b.program.page() == TunePage::Exited);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(20);
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    // still held through TUNE AGAIN: nothing until it is centred and pushed again
    b.tap(TuneButton::TuneAgain);
    REQUIRE(b.program.page() == TunePage::Tune);
    b.steps(10);
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(1);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(2);
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == doctest::Approx(mech::kTuneJogMostVolts));
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(2);
    // REPORT: pushed while the page shows a report
    b.startTest(TuneButton::TestB);
    b.finish();
    REQUIRE(b.program.page() == TunePage::Report);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, -1.0);
    b.steps(20);
    CHECK(b.program.service().position(0).holding());
    CHECK_FALSE(b.program.service().position(0).heldButtonMoving());
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    // CONFIRM: a fresh table, the first TEST's question up, the stick pushed
    Bench c{kTallCascade};
    c.ready();
    c.tap(TuneButton::TestA);
    REQUIRE(c.program.page() == TunePage::Confirm);
    c.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    c.steps(20);
    CHECK(c.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    CHECK(c.program.page() == TunePage::Confirm);
}

TEST_CASE("Tune review: a stick HELD while > changes the mechanism does not move the newly shown one, even when "
          "its first motion is confirmed -- only a fresh push does") {
    // BUG CAUGHT: the jog carried onto the next (confirmed) lift with no centre-then-push — a different lift
    // moving under the student's thumb while their eyes are on the screen.
    Bench b{kTwoHomedLifts};
    b.ready();
    b.tap(TuneButton::NextMech);
    confirmByStick(b);  // 'second lift' confirmed
    b.tap(TuneButton::PrevMech);
    confirmByStick(b);  // 'cascade' confirmed
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.5);
    b.steps(3);
    REQUIRE(b.program.service().position(0).heldButtonMoving());
    b.tap(TuneButton::NextMech);  // the stick STILL at 0.5
    REQUIRE(b.program.shown() == 1);
    b.steps(10);
    CHECK(b.provider.fakeMotor(2).commandedVoltage().value() == 0.0);
    CHECK_FALSE(b.program.service().position(1).heldButtonMoving());
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(1);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.5);
    b.steps(2);
    CHECK(b.program.service().position(1).heldButtonMoving());
    CHECK(b.provider.fakeMotor(2).commandedVoltage().value() > 1.0);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(2);
}

// ── READY ───────────────────────────────────────────────────────────────────────────────────

TEST_CASE("Tune review: READY makes NOTHING with no controller, with a field or switch, or with a button held -- "
          "READY greyed with why; and a touch read on the tick the controller drops is refused too") {
    // BUG CAUGHT: READY made the mechanisms (robot two's claw MOVES, HA-119) with no controller to stop it.
    struct Case {
        const char* what;
        bool connected;
        std::uint8_t status;
        bool held;
        const char* why;
    };
    const Case cases[] = {
        {"no controller", false, 0, false, "No controller: connect it first (it is the STOP)."},
        {"a field", true, mech::kTuneCompetitionConnected, false, "Field or switch plugged in: unplug it first."},
        {"a button held", true, 0, true, "A controller button is held: let go first."}};
    for (const Case& c : cases) {
        CAPTURE(c.what);
        Bench b{kCascadeAndClaw};
        b.controller.setConnected(c.connected);
        b.io.status = c.status;
        b.controller.setPressed(ControllerButton::X, c.held);
        b.steps(80);
        REQUIRE(b.button(TuneButton::Ready) != nullptr);
        CHECK_FALSE(b.button(TuneButton::Ready)->armed);
        CHECK(drawnText(b).find(c.why) != std::string::npos);
        b.tapAnyway(TuneButton::Ready);
        CHECK_FALSE(b.program.made());
        CHECK(b.program.page() == TunePage::Clear);
        CHECK_FALSE(anyRequest(b.provider));
    }
    // the backstop: READY drawn ARMED, the controller dropping on the very tick the touch is read
    Bench b{kCascadeAndClaw};
    b.steps(1);
    b.waitArmed(TuneButton::Ready);
    b.controller.setConnected(false);
    b.tapAnyway(TuneButton::Ready);
    CHECK_FALSE(b.program.made());
    CHECK(b.provider.digitalOutRequestCount(1) == 0);
    CHECK(std::string_view{b.program.status()} ==
          "READY refused -- no controller is connected (it carries the STOP); nothing was made");
    CHECK(b.linesContaining("TUNEPG", "READY refused -- no controller is connected") == 1);
}

// ── TEST, AGAIN, STOP on the REPORT page ───────────────────────────────────────────────────────

TEST_CASE("Tune review: AGAIN repeats the SAME test; the REPORT page's STOP (the TUNE page's cell) stops the hold "
          "and starts nothing") {
    // BUG CAUGHT (R18): AGAIN always TEST A; a reflexive STOP tap after a test landing on AGAIN.
    Bench b{kTallCascade};
    b.ready();
    b.startTest(TuneButton::TestB);
    b.finish();
    REQUIRE(b.program.page() == TunePage::Report);
    REQUIRE(b.program.report().test == mech::TuneTest::B);
    b.tap(TuneButton::Back);
    b.startTest(TuneButton::TestA);  // down to the floor, so a TEST B again must travel up
    b.finish();
    b.tap(TuneButton::Back);
    b.startTest(TuneButton::TestB);
    b.finish();
    b.tap(TuneButton::Again);
    REQUIRE(b.program.busy());
    b.finish();
    CHECK(b.program.report().test == mech::TuneTest::B);
    CHECK(b.program.report().target == 18.0);
    REQUIRE(b.program.page() == TunePage::Report);
    REQUIRE(b.program.service().position(0).holding());
    // STOP at the TUNE page's STOP spot, on the report's FIRST frame (no arming wait)
    const std::uint32_t begun = b.program.service().slot(0).stepsBegun();
    b.steps(1);
    const mech::TuneButtonView* stop = b.button(TuneButton::Stop);
    REQUIRE(stop != nullptr);
    CHECK(stop->rect.x0 == 109);
    CHECK(stop->rect.x1 == 349);
    b.tapAnyway(TuneButton::Stop);
    CHECK_FALSE(b.program.service().position(0).holding());
    CHECK_FALSE(b.program.busy());
    CHECK(b.program.service().slot(0).stepsBegun() == begun);
    CHECK(b.provider.fakeMotor(1).commandedVoltage().value() == 0.0);
    CHECK(b.program.page() == TunePage::Tune);
    CHECK(b.linesContaining("TUNEPG", "STOP -- STOP on the screen: every mechanism released") == 1);
}

TEST_CASE("Tune review: a STOP inside the hold second reports the hold n/a -- never a partial mean -- and says "
          "so on the page") {
    // BUG CAUGHT (R09): a STOP in the hold second reported the partial second's mean (0.29 A) as the hold.
    Bench b{kTallCascade};
    b.ready();
    b.startTest(TuneButton::TestB);
    for (int i = 0; i < 2000 && b.program.service().slot(0).running(); ++i) {
        REQUIRE(b.step());
    }
    b.steps(20);  // inside the hold second
    REQUIRE(b.program.busy());
    REQUIRE(b.program.page() == TunePage::Tune);  // not reported yet
    b.controller.setPressed(ControllerButton::A, true);
    REQUIRE(b.step());
    const mech::TuneReport& r = b.program.report();
    CHECK(std::string_view{r.verdict} == "SUCCEEDED");
    CHECK(r.holdAmps < 0.0);
    CHECK(r.holdStopped);
    CHECK_FALSE(r.holdEnded);
    CHECK_FALSE(r.warnStallNotAboveHold);
    CHECK(std::string_view{r.stoppedBy} == "a controller button");
    b.steps(1);
    const std::string page = drawnText(b);
    INFO(page);
    CHECK(page.find("; hold n/a: stopped in its second\n") != std::string::npos);
    CHECK(page.find("-1.00") == std::string::npos);
}

TEST_CASE("Tune verify: a STOP AFTER the stall tripped on the hold keeps the hold ENDED and its WARNING on the "
          "page -- the page and the log say the same, and the stop is still said") {
    // BUG CAUGHT (verify V1): the stop took priority — the page said "hold n/a: stopped in its second" with no
    // warning while the log said "its hold ENDED ... raise the signature": the page hid the one finding the
    // hold second exists for, from a student whose reflex on seeing the lift let go is to press a button.
    Bench b{kShortSignature};
    b.ready();
    b.arm->addLoadVolts(1.6);  // a game piece on the carriage
    b.startTest(TuneButton::TestB);
    for (int i = 0; i < 2000 && b.program.service().slot(0).running(); ++i) {
        REQUIRE(b.step());
    }
    REQUIRE(b.program.busy());  // the hold second
    int ticks = 0;
    while (b.program.busy() && b.program.service().slot(0).lastStep().outcome == MechanismOutcome::Succeeded &&
           ticks < 200) {
        REQUIRE(b.step());
        ++ticks;
    }
    REQUIRE(b.program.service().slot(0).lastStep().outcome == MechanismOutcome::Stalled);  // tripped ON the hold
    b.steps(20);  // 0.2 s: a reflex
    REQUIRE(b.program.busy());  // still inside the second
    b.controller.setPressed(ControllerButton::A, true);
    REQUIRE(b.step());
    const mech::TuneReport& r = b.program.report();
    CHECK(std::string_view{r.verdict} == "SUCCEEDED");
    CHECK(r.holdEnded);
    CHECK_FALSE(r.holdStopped);
    CHECK(r.warnStallNotAboveHold);
    CHECK(r.holdAmps < 0.0);
    CHECK(std::string_view{r.stoppedBy} == "a controller button");
    CHECK(b.linesContaining("TUNEPG", "WARNING -- its hold ENDED inside the second after it settled") == 1);
    b.controller.setPressed(ControllerButton::A, false);
    b.steps(1);
    const std::string page = drawnText(b);
    INFO(page);
    CHECK(page.find("; hold ENDED (the stall tripped on it)\n") != std::string::npos);
    CHECK(page.find("WARNING: stall 0.60 A trips on the hold: raise it\n") != std::string::npos);
    CHECK(page.find("stopped by a controller button\n") != std::string::npos);
    CHECK(page.find("hold n/a") == std::string::npos);
}

TEST_CASE("Tune review: TEST A and TEST B on robot two's claw pivot run it FORWARD then BACKWARD at its most "
          "volts and SUCCEED; the REPORT shows no settle and no hold for a power run") {
    // BUG CAUGHT (R12): TEST B spinning the claw the same way as TEST A; a power run's report saying
    // "never settled" and "no hold: it did not settle" under SUCCEEDED.
    Bench b{shulib::bench::robot_two::kRobotTwoMechanisms};
    b.ready();
    b.tap(TuneButton::TestA);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.tap(TuneButton::Go);
    REQUIRE(b.program.busy());
    b.steps(3);
    CHECK(b.provider.fakeMotor(6).commandedVoltage().value() == doctest::Approx(12.0));
    b.finish();
    CHECK(std::string_view{b.program.report().verdict} == "SUCCEEDED");
    CHECK(b.provider.fakeMotor(6).commandedVoltage().value() == 0.0);
    b.steps(1);
    const std::string page = drawnText(b);
    INFO(page);
    CHECK(page.find("settle") == std::string::npos);
    CHECK(page.find("hold") == std::string::npos);
    CHECK(page.find("\npeak 0.00 A\n") != std::string::npos);  // the claw's fake motor draws nothing
    b.tap(TuneButton::Back);
    b.startTest(TuneButton::TestB);
    b.steps(3);
    CHECK(b.provider.fakeMotor(6).commandedVoltage().value() == doctest::Approx(-12.0));
    b.finish();
    CHECK(std::string_view{b.program.report().verdict} == "SUCCEEDED");
    CHECK(b.program.report().target == -12.0);
}

TEST_CASE("Tune review: a FLYWHEEL's TEST spins to its target, reports the second at speed, and is STOPPED "
          "after it") {
    // BUG CAUGHT (R08): a flywheel left spinning after its report.
    Bench b{tune_fixture::kFlyAndPivot};
    b.ready();
    REQUIRE(b.program.tuning().spec(0).kind == mech::Kind::Speed);
    b.tap(TuneButton::TestA);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.tap(TuneButton::Go);
    REQUIRE(b.program.busy());
    b.steps(5);
    CHECK(b.provider.fakeMotor(4).commandedVoltage().value() > 1.0);
    b.finish();
    const mech::TuneReport& r = b.program.report();
    CHECK(r.kind == mech::Kind::Speed);
    CHECK(r.target == 0.5 * b.program.tuning().spec(0).speed.maxRpm);
    CHECK(std::string_view{r.verdict} == "SUCCEEDED");
    CHECK(r.holdAmps >= 0.0);
    CHECK(b.program.holdSamples().size() == 100);
    CHECK(b.provider.fakeMotor(4).commandedVoltage().value() == 0.0);
    CHECK_FALSE(b.program.service().speed(0).spinning());
}

// ── HOME ──────────────────────────────────────────────────────────────────────────────────────

TEST_CASE("Tune review: HOME from MID-TRAVEL on a stall-homed cascade -- the logged push peak IS the most current "
          "the motors drew (the test's own recording), its zero found; then TEST B and COPY in the same session") {
    // BUG CAUGHT (R05): every HOME logging "its push's peak 0.00 A" — the number a homing signature is sized
    // by, the cascade's tuning the week of Nov 14.
    Bench b{kStallHomed};
    b.ready();
    b.tap(TuneButton::Home);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.tap(TuneButton::Go);
    b.finish();
    REQUIRE(b.program.service().position(0).homed());
    b.startTest(TuneButton::TestB);  // up to 'mid', 12 in
    b.finish();
    REQUIRE(std::string_view{b.program.report().verdict} == "SUCCEEDED");
    b.tap(TuneButton::Back);
    REQUIRE(b.plantInches() > 10.0);
    // HOME again, from 12 in: the push travels DOWN onto the stop
    b.waitArmed(TuneButton::Home);
    b.recording = true;
    b.rec.clear();
    b.tapAnyway(TuneButton::Home);
    REQUIRE(b.program.busy());
    b.finish();
    CHECK(b.program.service().position(0).homed());
    CHECK(std::fabs(b.plantInches()) < 0.5);
    double most = 0.0;
    for (const Rec& r : b.rec) {
        most = std::max(most, r.amps);
    }
    CHECK(most > 0.5);  // a real push into the stop (its homing signature is 0.5 A)
    char want[96];
    std::snprintf(want, sizeof want, "'stall homed lift': HOME SUCCEEDED in ");
    CHECK(b.linesContaining("TUNEPG", want) == 2);
    std::snprintf(want, sizeof want, ", its push's peak %.2f A -- its zero is found", most);
    INFO(std::string{want});
    CHECK(b.linesContaining("TUNEPG", want) >= 1);
    // TEST and COPY in the same session
    b.select("kP");
    b.tap(TuneButton::Plus);
    b.startTest(TuneButton::TestB);
    b.finish();
    CHECK(std::string_view{b.program.report().verdict} == "SUCCEEDED");
    b.tap(TuneButton::Back);
    b.tap(TuneButton::Copy);
    CHECK(b.linesContaining("TUNE", "'stall homed lift': copy ends -- 1 line") == 1);
}

TEST_CASE("Tune review: a HOME whose push never trips its signature is TIMED_OUT AT kTuneHomeSeconds, within a "
          "tick") {
    // BUG CAUGHT (R16): HOME's limit cut to 1 s survived (the only HOME run never travelled).
    Bench b{tune_fixture::kMidHome};  // its -2 V push draws ~0.42 A on the host test plant, under its 0.6 A
    b.ready();
    b.tap(TuneButton::Home);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.tap(TuneButton::Go);
    b.finish(3000);
    const shulib::mechanism::StepReport& r = b.program.service().slot(0).lastStep();
    CHECK(r.outcome == MechanismOutcome::TimedOut);
    CHECK(r.elapsed >= mech::kTuneHomeSeconds - 1e-9);
    CHECK(r.elapsed <= mech::kTuneHomeSeconds + 0.01 + 1e-9);
    CHECK_FALSE(b.program.service().position(0).homed());
    CHECK(b.linesContaining("TUNEPG", "'mid home lift': HOME TIMED_OUT in 10.0") == 1);
}

// ── COPY, the caps, EXIT ────────────────────────────────────────────────────────────────────

TEST_CASE("Tune review: COPY writes ONE block for EVERY mechanism changed (D9 (a)), not only the shown one's") {
    // BUG CAUGHT: a student who tuned two mechanisms copies one and loses the other's numbers.
    Bench b{kTwoHomedLifts};
    b.ready();
    b.select("kP");
    b.tap(TuneButton::Plus);
    b.tap(TuneButton::NextMech);
    b.select("kP");
    b.tap(TuneButton::Minus);
    b.tap(TuneButton::PrevMech);
    REQUIRE(b.program.shown() == 0);
    b.tap(TuneButton::Copy);
    CHECK(b.linesContaining("TUNE", "'cascade': copy begins") == 1);
    CHECK(b.linesContaining("TUNE", "'second lift': copy begins") == 1);
    CHECK(std::string_view{b.program.status()} ==
          "COPY: 2 lines from 2 mechanisms to the terminal and /usd/tune_log.txt: put them in the robot file");
    CHECK_FALSE(b.program.uncopied(0));
    CHECK_FALSE(b.program.uncopied(1));
}

TEST_CASE("Tune review: LEAVING a page puts its tuned cap back -- after COPYING its numbers first when no COPY "
          "carries them (the tuned cap is in the log, not lost); a COPY made already is not repeated") {
    // BUG CAUGHT: › reverted the tuned cap in the live numbers and the COPY made after it carried the
    // FILE's cap — the robot file kept the old cap and nothing said so.
    Bench b{kTwoLifts};
    b.ready();
    b.tap(TuneButton::NextMech);
    b.select("current cap");
    b.tap(TuneButton::Minus);
    REQUIRE(b.program.tuning().spec(1).currentCapAmps == doctest::Approx(1.45));
    b.tap(TuneButton::PrevMech);
    CHECK(b.program.shown() == 0);
    CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.5));
    const int capLine = firstLine(b, ".currentCap = {.amps = 1.45}");
    const int back = firstLine(b, "'mid home lift': its current cap back to the robot file's");
    INFO(capLine);
    CHECK(capLine >= 0);
    CHECK(back > capLine);  // COPIED before it went back
    CHECK(std::string_view{b.program.status()} ==
          "'mid home lift': its cap is the file's 1.50 A again; its numbers were COPIED first");
    // COPY first, then leave: the cap goes back with NO second block
    b.tap(TuneButton::NextMech);
    b.select("current cap");
    b.tap(TuneButton::Minus);
    b.tap(TuneButton::Copy);
    const int blocks = b.linesContaining("TUNE", "'mid home lift': copy begins");
    b.tap(TuneButton::PrevMech);
    CHECK(b.linesContaining("TUNE", "'mid home lift': copy begins") == blocks);
    CHECK(std::string_view{b.program.status()} == "'mid home lift': its cap is the file's 1.50 A again");
}

TEST_CASE("Tune review: a page whose cap CANNOT go back (a live signature at or above the file's cap) is not left "
          "-- the cap stays, its numbers are COPIED, and the page's two lines carry the whole instruction") {
    // BUG CAUGHT (R24): the page moved on with a raised cap left in a motor no page showed; the sentence cut
    // at "refu" on the screen, its instruction never seen.
    Bench b{kTwoLifts};
    b.ready();
    b.tap(TuneButton::NextMech);
    REQUIRE(b.program.shown() == 1);
    REQUIRE(b.program.tuning().apply(1, mech::Tunable::CurrentCap, 2.2).ok);
    REQUIRE(b.program.tuning().apply(1, mech::Tunable::StallAmps, 1.8).ok);  // above the file's 1.5 A cap
    b.tap(TuneButton::PrevMech);
    CHECK(b.program.shown() == 1);
    CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(2.2));
    CHECK(b.program.tuning().spec(1).currentCapAmps == 2.2);
    CHECK(b.linesContaining("TUNE", ".currentCap = {.amps = 2.2}") == 1);  // COPIED
    CHECK(std::string_view{b.program.status()} ==
          "'mid home lift': can't leave: a signature >= the file's cap 1.50 A -- lower it, or RESET (your numbers "
          "are COPIED)");
    b.steps(12);  // a changed line is repainted at most every 0.1 s
    const std::string page = drawnText(b);
    INFO(page);
    CHECK(page.find("can't leave: a signature >= the file's cap 1.50 A\n-- lower it, or RESET (your numbers are "
                    "COPIED)\n") != std::string::npos);
    CHECK(b.linesContaining("TUNEPG", "lower that signature, or RESET it") == 1);
}

TEST_CASE("Tune review: EXIT with a cap that CANNOT go back alone COPIES the mechanism's numbers, RESETS it (the "
          "motor's limit is the file's) and says so -- on the EXITED page and in TUNE AGAIN's sentence") {
    // BUG CAUGHT (R11, and EXIT silently discarding a session): EXIT left 2.2 A in the motor (HA-220), or
    // RESET every live number with nothing on the screen, and TUNE AGAIN said "your live numbers are kept".
    Bench b{kCappedLift};
    b.ready();
    REQUIRE(b.program.tuning().apply(0, mech::Tunable::CurrentCap, 1.0).ok);
    REQUIRE(b.program.tuning().apply(0, mech::Tunable::StallAmps, 0.8).ok);  // above the file's 0.6 A cap
    b.select("kP");
    b.tap(TuneButton::Plus);
    const double kP = b.program.tuning().spec(0).position.kP;
    REQUIRE(kP != kCappedLift.specs[0].position.kP);
    b.tap(TuneButton::Exit);
    REQUIRE(b.program.page() == TunePage::Exited);
    CHECK(b.provider.fakeMotor(1).currentLimit().value() == doctest::Approx(0.6));
    CHECK_FALSE(b.program.tuning().changed(0));  // RESET
    CHECK(b.program.copiedAtExit() == 1);
    CHECK(b.program.resetAtExit() == 1);
    // the tuned numbers are in the log, BEFORE the reset
    char kPLine[64];
    std::snprintf(kPLine, sizeof kPLine, ".tuning.kP = %g", kP);
    const int copied = firstLine(b, kPLine);
    const int reset = firstLine(b, "EXIT: 'capped lift': RESET to the robot file's numbers");
    INFO(std::string{kPLine});
    CHECK(copied >= 0);
    CHECK(reset > copied);
    CHECK(b.linesContaining("TUNE", ".currentCap = {.amps = 1}") == 1);
    CHECK(b.linesContaining("TUNEPG", "EXIT: 1 mechanism's changed numbers COPIED above") == 1);
    b.steps(1);
    const std::string page = drawnText(b);
    INFO(page);
    CHECK(page.find("1 RESET to the file: a signature >= its cap.") != std::string::npos);
    CHECK(page.find("EXIT COPIED 1 mechanism's changed numbers to") != std::string::npos);
    b.tap(TuneButton::TuneAgain);
    CHECK(std::string_view{b.program.status()} ==
          "TUNE AGAIN: live numbers kept, but 1 mechanism was RESET at EXIT (each one's block is in the log)");
    // the same on a mechanism NOT shown at EXIT (mid home lift, shown: the cascade)
    Bench t{kTwoLifts};
    t.ready();
    REQUIRE(t.program.tuning().apply(1, mech::Tunable::CurrentCap, 2.2).ok);
    REQUIRE(t.program.tuning().apply(1, mech::Tunable::StallAmps, 1.8).ok);
    t.tap(TuneButton::Exit);
    CHECK(t.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.5));
    CHECK(t.program.tuning().spec(1).currentCapAmps == 1.5);
    CHECK(t.program.resetAtExit() == 1);
}

TEST_CASE("Tune review: EXIT COPIES every mechanism changed since its last COPY (the live numbers are not kept), "
          "and only those") {
    Bench b{kTwoHomedLifts};
    b.ready();
    b.select("kP");
    b.tap(TuneButton::Plus);
    b.tap(TuneButton::Copy);  // the cascade copied ...
    b.tap(TuneButton::NextMech);
    b.select("kP");
    b.tap(TuneButton::Plus);  // ... the second lift not
    const int cascadeBlocks = b.linesContaining("TUNE", "'cascade': copy begins");
    b.tap(TuneButton::Exit);
    CHECK(b.program.copiedAtExit() == 1);
    CHECK(b.linesContaining("TUNE", "'cascade': copy begins") == cascadeBlocks);
    CHECK(b.linesContaining("TUNE", "'second lift': copy begins") == 1);
    CHECK(b.program.resetAtExit() == 0);
    b.tap(TuneButton::TuneAgain);
    CHECK(std::string_view{b.program.status()} ==
          "TUNE AGAIN: your live numbers are kept; the caps went back to the file's");
    CHECK(b.program.tuning().changed(1));  // kept
}

TEST_CASE("Tune verify: an UNDO whose next undo would change ANOTHER mechanism's cap (the one put back as its "
          "page was left) WAITS -- nothing changes, the page says so whole; on that mechanism's page it runs, and "
          "then the shown page's own changes undo") {
    // BUG CAUGHT (review C5): one UNDO on the next page re-wrote 1.45 A into 'mid home lift''s motor while the
    // page said "any controller button STOPS every mechanism" — and a stop from the brain carried it into the
    // next program (HA-220).
    // BUG CAUGHT (verify V2): the first fix undid the write-back and re-applied it — the depth never fell
    // ("UNDO 3" for ever), the cascade's own kP change could never be undone from its page, every press
    // logged two lines, and the screen cut the explanation at "(cha".
    Bench b{kTwoLifts};
    b.ready();
    b.select("kP");
    b.tap(TuneButton::Plus);  // the cascade's own change, made FIRST
    REQUIRE(b.program.tuning().spec(0).position.kP != kTwoLifts.specs[0].position.kP);
    b.tap(TuneButton::NextMech);
    b.select("current cap");
    b.tap(TuneButton::Minus);
    REQUIRE(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.45));
    b.tap(TuneButton::PrevMech);  // its cap goes back (COPIED first): one more undo record
    REQUIRE(b.program.shown() == 0);
    REQUIRE(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.5));
    const std::size_t depth = b.program.tuning().undoDepth();
    REQUIRE(depth == 3);
    CHECK(b.program.undoTrackedDepth() == depth);
    const int waitsLogged = b.linesContaining("TUNEPG", "UNDO waits on another page");
    for (int k = 1; k <= 3; ++k) {  // as many presses as a student makes: NOTHING changes
        b.tap(TuneButton::Undo);
        CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.5));
        CHECK(b.program.tuning().spec(1).currentCapAmps == 1.5);
        CHECK(b.program.tuning().undoDepth() == depth);
        CHECK(b.program.undoTrackedDepth() == depth);
        CHECK(b.linesContaining("TUNEPG", "UNDO waits on another page") == waitsLogged + k);  // ONE line a press
    }
    CHECK(b.linesContaining("TUNEPG",
                            "'mid home lift': UNDO waits on another page -- the next undo would change its current "
                            "cap (only its own page may change a cap): show it (< >) and UNDO there") == 3);
    CHECK(std::string_view{b.program.status()} == "UNDO waits: next is 'mid home lift''s cap -- UNDO it on its page");
    b.steps(12);  // a changed line is repainted at most every 0.1 s
    const std::string page = drawnText(b);
    INFO(page);
    CHECK(page.find("\nUNDO waits: next is 'mid home lift''s cap -- UNDO\nit on its page\n") != std::string::npos);
    CHECK(page.find("cap changed") == std::string::npos);  // no cap is changed anywhere: none was re-written
    {  // with the LONGEST name a mechanism can have (23 characters) the sentence is still whole on the two lines
        const std::string longest = "UNDO waits: next is '" + std::string(23, 'w') + "''s cap -- UNDO it on its page";
        char s1[shulib::ui::kSmallCols + 1];
        char s2[shulib::ui::kSmallCols + 1];
        mech::detail::tuneWrap(longest.c_str(), s1, s2, shulib::ui::kSmallCols);
        CHECK(std::string{s1} + " " + s2 == longest);
    }
    // on ITS page the same UNDO runs: the cap put back is taken back (shown — the safety line says so) ...
    b.tap(TuneButton::NextMech);
    b.tap(TuneButton::Undo);
    CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.45));
    CHECK(b.program.tuning().undoDepth() == depth - 1);
    b.steps(12);
    CHECK(std::string_view{b.program.drawn().lines[std::size_t{2}].text} == "cap changed: EXIT or RESET before you stop");
    b.tap(TuneButton::Undo);  // ... and the [-] itself
    CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.5));
    CHECK(b.program.tuning().undoDepth() == depth - 2);
    // back on the cascade's page (nothing to put back), its own kP change undoes
    b.tap(TuneButton::PrevMech);
    REQUIRE(b.program.shown() == 0);
    b.tap(TuneButton::Undo);
    CHECK(b.program.tuning().spec(0).position.kP == kTwoLifts.specs[0].position.kP);
    CHECK(b.program.tuning().undoDepth() == 0);
    CHECK(b.program.undoTrackedDepth() == 0);
    // a cap changed on a mechanism the page does not show (the library's apply, outside the program): the
    // safety line watches EVERY mechanism's cap
    REQUIRE(b.program.tuning().apply(1, mech::Tunable::CurrentCap, 1.4).ok);
    b.steps(12);
    CHECK(std::string_view{b.program.drawn().lines[std::size_t{2}].text} == "cap changed: EXIT or RESET before you stop");
}

TEST_CASE("Tune verify: an UNDO on another page that would undo a mechanism's RESET of a RAISED cap (its signature "
          "above the file's cap) WAITS -- the motor keeps the file's cap; on its own page it runs, and that page "
          "is then not left") {
    // BUG CAUGHT (verify V5): the UNDO brought back 'mid home lift''s 2.2 A cap AND its 1.8 A signature, the
    // re-apply of the file's cap was refused (the signature), and motor 2 kept 2.2 A with the page saying
    // "it stays the file's" — a stop from the brain then carried 2.2 A into the next program (HA-220).
    Bench b{kTwoLifts};
    b.ready();
    b.tap(TuneButton::NextMech);
    REQUIRE(b.program.shown() == 1);
    b.select("current cap");  // by presses only: the cap up to 2.2 A, then the signature up to 1.8 A
    for (int k = 0; k < 40 && b.program.tuning().spec(1).currentCapAmps < 2.2 - 1e-9; ++k) {
        b.tap(TuneButton::Plus);
    }
    REQUIRE(b.program.tuning().spec(1).currentCapAmps == doctest::Approx(2.2));
    b.select("stall amps");
    for (int k = 0; k < 40 && mech::tunableValue(b.program.tuning().spec(1), mech::Tunable::StallAmps) < 1.8 - 1e-9;
         ++k) {
        b.tap(TuneButton::Plus);
    }
    REQUIRE(mech::tunableValue(b.program.tuning().spec(1), mech::Tunable::StallAmps) >= 1.5);  // >= the file's cap
    b.tap(TuneButton::Reset);  // the remedy the page offers
    REQUIRE_FALSE(b.program.tuning().changed(1));
    b.tap(TuneButton::PrevMech);
    REQUIRE(b.program.shown() == 0);
    REQUIRE(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.5));
    const std::size_t depth = b.program.tuning().undoDepth();
    b.tap(TuneButton::Undo);
    CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.5));
    CHECK_FALSE(b.program.tuning().changed(1));
    CHECK(b.program.tuning().undoDepth() == depth);
    CHECK(b.program.undoTrackedDepth() == depth);
    CHECK(std::string_view{b.program.status()} == "UNDO waits: next is 'mid home lift''s cap -- UNDO it on its page");
    b.steps(12);
    CHECK(std::string_view{b.program.drawn().lines[std::size_t{2}].text} ==
          "any controller button STOPS every mechanism");
    // on its own page the RESET undoes (the raised cap SHOWN), and the page then is not left
    b.tap(TuneButton::NextMech);
    b.tap(TuneButton::Undo);
    CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(2.2));
    b.tap(TuneButton::PrevMech);
    CHECK(b.program.shown() == 1);
    CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(2.2));
    CHECK(b.program.undoTrackedDepth() == b.program.tuning().undoDepth());
}

TEST_CASE("Tune verify: UNDO on one page still takes back ANOTHER mechanism's change that is not its cap (D8 (a): "
          "UNDO is global, the sentence names it) -- only a cap waits") {
    // BUG CAUGHT (by construction): a wait on EVERY record of another mechanism would make UNDO page-local,
    // against D8 (a), and strand every change made before a ‹ ›.
    Bench b{kTwoLifts};
    b.ready();
    b.select("kP");
    b.tap(TuneButton::Plus);
    b.tap(TuneButton::NextMech);
    REQUIRE(b.program.shown() == 1);
    b.tap(TuneButton::Undo);
    CHECK(b.program.tuning().spec(0).position.kP == kTwoLifts.specs[0].position.kP);
    CHECK(b.program.tuning().undoDepth() == 0);
    CHECK(b.program.undoTrackedDepth() == 0);
    CHECK(std::string_view{b.program.status()}.rfind("'cascade': undo: kP ", 0) == 0);
}

TEST_CASE("Tune verify: the program tracks WHOSE every undo record is, in step with live tuning's own -- past "
          "its 16 the OLDEST falls off (never the newest), and a refusal records nothing") {
    // BUG CAUGHT (by construction): a record kept out of step names the wrong mechanism, and UNDO then runs
    // into another mechanism's cap — here, 1.45 A written into a motor no page shows.
    Bench b{kTwoLifts};
    b.ready();
    const auto inStep = [&] { return b.program.undoTrackedDepth() == b.program.tuning().undoDepth(); };
    b.select("kP");
    for (int k = 0; k < 14; ++k) {  // the cascade: 14 records (its cap: the file's "none" throughout)
        b.tap(TuneButton::Plus);
        REQUIRE(inStep());
    }
    b.select("current cap");
    b.tap(TuneButton::Plus);  // the cascade states no cap: [+] from none is refused — no record
    CHECK(inStep());
    CHECK(b.program.tuning().undoDepth() == 14);
    b.tap(TuneButton::NextMech);
    b.select("current cap");
    b.tap(TuneButton::Minus);  // 1.5 -> 1.45
    b.tap(TuneButton::Plus);   // 1.45 -> 1.5: the file's again, but undoing it brings 1.45 back
    REQUIRE(b.program.tuning().spec(1).currentCapAmps == 1.5);
    REQUIRE(b.program.tuning().undoDepth() == mech::kLiveUndoDepth);  // full
    REQUIRE(inStep());
    b.tap(TuneButton::PrevMech);  // its cap is the file's: nothing to put back, no record
    REQUIRE(b.program.shown() == 0);
    CHECK(b.program.tuning().undoDepth() == mech::kLiveUndoDepth);
    b.tap(TuneButton::Undo);  // its [+] would bring 1.45 A back into motor 2: waits
    CHECK(std::string_view{b.program.status()} == "UNDO waits: next is 'mid home lift''s cap -- UNDO it on its page");
    b.select("kP");
    b.tap(TuneButton::Plus);  // a 17th record: the OLDEST (the cascade's first kP) falls off
    REQUIRE(inStep());
    b.tap(TuneButton::Undo);  // that kP: the cascade's own
    REQUIRE(inStep());
    b.tap(TuneButton::Undo);  // the top is 'mid home lift''s [+] again: waits — motor 2 keeps the file's cap
    CHECK(b.provider.fakeMotor(2).currentLimit().value() == doctest::Approx(1.5));
    CHECK(b.program.tuning().spec(1).currentCapAmps == 1.5);
    CHECK(std::string_view{b.program.status()} == "UNDO waits: next is 'mid home lift''s cap -- UNDO it on its page");
    // on its page both of its records undo, then the cascade's 13 left, each on the cascade's page
    b.tap(TuneButton::NextMech);
    b.tap(TuneButton::Undo);
    b.tap(TuneButton::Undo);
    CHECK(b.program.tuning().spec(1).currentCapAmps == 1.5);
    b.tap(TuneButton::PrevMech);
    int undone = 0;
    while (b.program.tuning().undoDepth() > 0 && undone < 40) {
        b.tap(TuneButton::Undo);
        ++undone;
        REQUIRE(inStep());
    }
    CHECK(undone == 13);  // every one ran: all are the cascade's own
    CHECK(b.program.tuning().changed(0));  // its first kP fell off the 16: it stays (RESET reaches it)
}

TEST_CASE("Tune review: a cap the robot file does not state STARTS with [-] at 2.45 A (the motor's 2.5 A less a "
          "step); [+] from none is refused; the page says how") {
    // BUG CAUGHT (a document's, and the run log's finding for TUNE1-A): guide 18 said a cap cannot be started
    // from the page — a [+] from none "is 0.05 A". It is 2.55 A (refused); [-] gives 2.45 A.
    Bench b{kTallCascade};  // states no cap; its stall signature is 2.0 A
    b.ready();
    b.select("current cap");
    b.steps(12);  // a changed line is repainted at most every 0.1 s
    CHECK(drawnText(b).find("  file: none; - starts a cap at 2.45 A\n") != std::string::npos);
    b.tap(TuneButton::Plus);
    CHECK(b.program.tuning().spec(0).currentCapAmps == 0.0);
    CHECK(std::string_view{b.program.status()}.find("refused") != std::string_view::npos);
    b.tap(TuneButton::Minus);
    CHECK(b.program.tuning().spec(0).currentCapAmps == doctest::Approx(2.45));
    CHECK(b.provider.fakeMotor(1).currentLimit().value() == doctest::Approx(2.45));
}

// ═══ ROB1-A's integration onto TUNE1-B1 (2026-11-13): robot one's cascade in "shulib Tune" ═══════════════
// Robot one joined the Makefile's MECHANISM_ROBOTS (its cascade is a mechanism table), so `make ROBOT=one
// PROGRAM=tune` builds the day its facts are typed. This runs the SAME program over robot one's OWN table —
// the cascade on the facts file's placeholders (src/robots/robot_one_facts.hpp) — on the host test plant.

namespace {
/// Robot one's cascade on the host test plant, from its DECLARATION (test/robot_one_tables_test.cpp's
/// cascadePlant(), restated): the facts file's cartridge (its free speed), gearing and rows (the stops at the
/// bottom and top rows' output angles), gravity below the hold volts; started at `startDeg`, its encoders
/// reading ZERO there — where it is is unknown until it homes.
sim::ArmPlantConfig robotOneCascadePlant(double startDeg) {
    namespace facts = shulib::bench::robot_one_facts;
    const double gear = static_cast<double>(facts::kCascadeTeeth.value().spool) /
                        static_cast<double>(facts::kCascadeTeeth.value().motor);
    sim::ArmPlantConfig pc = pos_rig::armPlant(pos_rig::cartridgeRpm(facts::kCascadeCartridge.value()), gear,
                                               0.8 * facts::kCascadeHoldVolts.value(), 1.0, 0.3, startDeg);
    pc.constantGravity = true;
    pc.minStopDeg = facts::kCascadeRows.value().row[0].degrees;
    pc.maxStopDeg = facts::kCascadeRows.value().row[facts::kCascadeRowCount - 1].degrees;
    pc.encoderZeroDeg = startDeg;
    return pc;
}
}  // namespace

namespace {
/// Robot one's cascade: the OUTPUT angle (the rows' frame, the plant's) at height `in`, straight between the
/// facts file's rows, and the local degrees per inch there.
struct RowPoint {
    double deg;
    double degPerIn;
};
RowPoint robotOneRowPoint(double in) {
    namespace facts = shulib::bench::robot_one_facts;
    const auto& row = facts::kCascadeRows.value().row;
    std::size_t i = 1;
    while (i + 1 < facts::kCascadeRowCount && in > row[i].height.value()) {
        ++i;
    }
    const double h0 = row[i - 1].height.value();
    const double h1 = row[i].height.value();
    const double slope = (row[i].degrees - row[i - 1].degrees) / (h1 - h0);
    return RowPoint{row[i - 1].degrees + slope * (in - h0), slope};
}
}  // namespace

TEST_CASE("Tune (ROB1-A's integration; ROB1-C2's six mechanisms): over ROBOT ONE's own table the provider is asked "
          "ONLY for its DECLARED ports; HOME pushes the cascade DOWN onto the bottom row with the facts file's cap in "
          "both motors; TEST A brings it DOWN to its first target from above; TEST B drives it UP to its second and it "
          "settles there") {
    // BUG CAUGHT: robot one put in MECHANISM_ROBOTS with a table Tune cannot run — a cascade whose homing push,
    // signature or targets the program refuses or never reaches would build `make ROBOT=one PROGRAM=tune` the
    // day the facts are typed and tune nothing; Tune writing a cap that is not the facts file's (the review of
    // the integration: the peak current alone could not tell — the limit IN THE MOTORS is checked); or Tune
    // asking for a port robot one's list does not declare (robot one's drive is unknown until Calibrate, so ANY
    // other port asked is a motor nobody declared). Since ROB1-C2 the cascade has NO levels (manual for now), so
    // Tune's targets are a quarter and three quarters of its range (TuneTargets) — read from the program's own
    // rule, never assumed.
    namespace facts = shulib::bench::robot_one_facts;
    const mech::MechanismTable& t = shulib::bench::robot_one::kRobotOneMechanisms;
    REQUIRE(t.count == 6);
    REQUIRE(std::string_view{t.specs[0].name} == "cascade");
    const std::uint8_t portA = static_cast<std::uint8_t>(facts::kCascadePorts.value().first);
    const std::uint8_t portB = static_cast<std::uint8_t>(facts::kCascadePorts.value().second);
    const double capAmps = facts::kCascadeCapAmps.value();
    const auto& row = facts::kCascadeRows.value().row;
    Bench b{t, robotOneCascadePlant(300.0)};  // above the bottom stop
    b.ready();
    CHECK(std::string_view{b.program.tuning().spec(b.program.shown()).name} == "cascade");
    CHECK_FALSE(b.program.service().position(0).homed());
    // the facts file's cap is IN BOTH MOTORS before anything moves
    CHECK(b.provider.fakeMotor(portA).currentLimit().value() == doctest::Approx(capAmps));
    CHECK(b.provider.fakeMotor(portB).currentLimit().value() == doctest::Approx(capAmps));
    // HOME: stage 2 asks first (its first motion), then DOWN into the bottom stop
    b.tap(TuneButton::Home);
    REQUIRE(b.program.page() == TunePage::Confirm);
    b.tap(TuneButton::Go);
    double lowest = 0.0;
    double worst = 0.0;
    for (int i = 0; i < 2000 && b.program.busy(); ++i) {
        REQUIRE(b.step());
        lowest = std::min(lowest, b.armMotors[0]->commandedVoltage().value());
        worst = std::max(worst, b.plantAmps());
    }
    REQUIRE_FALSE(b.program.busy());
    REQUIRE(b.program.service().position(0).homed());
    CHECK(b.program.service().position(0).position() == doctest::Approx(facts::cascadeBottom().value()));
    CHECK(b.arm->trueDeg() == doctest::Approx(row[0].degrees).epsilon(0.001));
    CHECK(lowest < 0.0);                                             // it pushed DOWN
    CHECK(lowest >= facts::kCascadeHomingPushVolts.value() - 1e-9);  // no harder than the facts file's push
    CHECK(worst <= capAmps + 1e-9);                                  // (the plant reads at most the limit)
    CHECK(b.provider.fakeMotor(portA).currentLimit().value() == doctest::Approx(capAmps));  // still the cap
    CHECK(b.provider.fakeMotor(portB).currentLimit().value() == doctest::Approx(capAmps));
    CHECK(b.linesContaining("TUNEPG", "'cascade': HOME SUCCEEDED") >= 1);
    // The targets, by the program's own rule: the first and last NAMED level, or with fewer than two named a
    // quarter and three quarters of the range (ROB1-C2: none named).
    std::size_t named = 0;
    while (named < shulib::mechanism::kMaxPresets && facts::lv(named).name != nullptr) {
        ++named;
    }
    const mech::TuneTargets targets = mech::tuneTargets(t.specs[0]);
    const double bottom = facts::cascadeBottom().value();
    const double top = facts::cascadeTop().value();
    if (named >= 2) {
        CHECK(std::string_view{targets.aName} == facts::lv(0).name);
        CHECK(std::string_view{targets.bName} == facts::lv(named - 1).name);
    } else {
        CHECK(std::string_view{targets.aName} == "1/4 range");
        CHECK(std::string_view{targets.bName} == "3/4 range");
        CHECK(targets.a == doctest::Approx(bottom + 0.25 * (top - bottom)));
        CHECK(targets.b == doctest::Approx(bottom + 0.75 * (top - bottom)));
    }
    // Where each target is, by the PLANT (never the program): its output angle by the rows, within half an inch.
    const RowPoint aAt = robotOneRowPoint(targets.a);
    const RowPoint bAt = robotOneRowPoint(targets.b);
    // TEST A from ABOVE its target (HOME left it on the floor, so first the stick jogs it up — a TEST A from
    // below would be a move UP, not the move down this case exists for)
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 1.0);
    b.steps(300);
    b.controller.setAxis(shulib::hal::ControllerAxis::LeftY, 0.0);
    b.steps(20);
    const double aFrom = b.arm->trueDeg();
    MESSAGE("robot one's TEST A from " << aFrom << " deg to " << aAt.deg << " deg");
    REQUIRE(aFrom - aAt.deg > 2.0 * aAt.degPerIn);  // a real move down, not a settle in place
    b.startTest(TuneButton::TestA);
    b.finish();
    MESSAGE("robot one's TEST A: " << std::string{b.program.report().verdict} << ", plant at " << b.arm->trueDeg());
    CHECK(std::string_view{b.program.report().verdict} == "SUCCEEDED");
    CHECK(std::abs(b.arm->trueDeg() - aAt.deg) <= 0.5 * aAt.degPerIn);
    b.tap(TuneButton::Back);
    // TEST B, up to its target. Its time limit is the plan x 2 + settle + 0.5 s, AT MOST 10 s
    // (kTuneTestMostSeconds). Independently of the program: the live top speed is at most the output's free
    // speed, so the plan takes at least distance / free speed. (ROB1-A's red 12:60 placeholder gearing needed
    // ~9 s and ended TIMED_OUT once; the builder's blue direct drive moves the output at 3600 deg/s free —
    // either way a TIMED_OUT must HOLD where it stopped, and a second TEST B finishes it.)
    const double freeDegPerS = pos_rig::cartridgeRpm(facts::kCascadeCartridge.value()) * 6.0 *
                               static_cast<double>(facts::kCascadeTeeth.value().motor) /
                               static_cast<double>(facts::kCascadeTeeth.value().spool);
    const double leastPlanS = (bAt.deg - b.arm->trueDeg()) / freeDegPerS;
    const double bound = mech::tunePositionBound(t.specs[0], b.arm->trueDeg(), targets.b);  // the plant's frame IS the rows'
    if (mech::kTuneTestPlanFactor * leastPlanS + mech::kTuneTestExtraSeconds >= mech::kTuneTestMostSeconds) {
        CHECK(bound == mech::kTuneTestMostSeconds);  // the cap, by arithmetic the program does not do
    }
    double endDeg = b.arm->trueDeg();
    bool reached = false;
    for (int attempt = 1; attempt <= 3 && !reached; ++attempt) {
        CAPTURE(attempt);
        const double fromDeg = b.arm->trueDeg();
        CHECK(std::abs(fromDeg - endDeg) <= 0.5 * bAt.degPerIn);  // a TIMED_OUT HOLDS where it stopped (never drops)
        b.startTest(TuneButton::TestB);
        const double t0 = b.clock.now().value();
        b.finish();
        const double tookS = b.clock.now().value() - t0;
        const std::string verdict{b.program.report().verdict};
        endDeg = b.arm->trueDeg();
        MESSAGE("robot one's TEST B (" << attempt << "): " << verdict << " after " << tookS << " s (limit " << bound
                                       << " s for the first), plant from " << fromDeg << " to " << endDeg << " deg of "
                                       << bAt.deg);
        CHECK(endDeg - fromDeg > 0.0);  // DRIVEN UP toward it, not left where it was
        if (verdict == "SUCCEEDED") {
            reached = true;
            CHECK(std::abs(endDeg - bAt.deg) <= 0.5 * bAt.degPerIn);
        } else {
            REQUIRE(verdict == "TIMED_OUT");  // nothing else: no stall, no refusal, no cancel
            CHECK(tookS >= mech::kTuneTestMostSeconds - 0.05);  // it had the whole cap, never less
            CHECK(bAt.deg - endDeg > 0.5 * bAt.degPerIn);        // and was really short of the target
        }
        b.tap(TuneButton::Back);
    }
    CHECK(reached);
    // ...and it STAYS there (the hold after the TEST), a second later
    b.steps(100);
    CHECK(std::abs(b.arm->trueDeg() - bAt.deg) <= 0.5 * bAt.degPerIn);
    b.tap(TuneButton::Exit);
    // ONLY robot one's DECLARED ports were ever asked for — the cascade's two, the side toggles' two and the
    // three pistons' lines — no other smart port, no other 3-wire line.
    for (std::uint8_t port = 1; port <= 21; ++port) {
        CAPTURE(static_cast<int>(port));
        const bool declared = shulib::bench::isDeclaredMechanismPort(t, port);
        if (port == portA || port == portB) {
            CHECK(b.provider.motorRequestCount(port) >= 1);
        }
        if (!declared) {
            CHECK(b.provider.motorRequestCount(port) == 0);
        }
    }
    for (std::uint8_t line = 1; line <= 8; ++line) {
        CAPTURE(static_cast<int>(line));
        const char letter = static_cast<char>('A' + line - 1);
        const bool declared = letter == facts::kClawLine.value() || letter == facts::kFlipperLine.value() ||
                              letter == facts::kRampLine.value();
        if (!declared) {
            CHECK(b.provider.digitalOutRequestCount(line) == 0);
        }
    }
}
