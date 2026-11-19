#pragma once
//
// competition_rig.hpp — "shulib Competition"'s pacer chain and auton request on the A2 host test
// plant, for test/competition_watch_test.cpp and test/competition_run_test.cpp (chunk COMP-b,
// 2026-09-30).
//
// The chain is the program's (src/programs/competition_program.cpp) with the plant in place of the
// provider's 10 ms pacer and a PROBE layer under the watch:
//
//     Chassis -> RunGuard -> MechanismService (REAL) -> CompetitionWatch (REAL) -> ProbeLayer -> PlantPacer
//
// The mechanism service and the route registry are the program's (the mechanism core's MECH-A5): bound
// over THE CLAW ROBOT (the old robot two, a fixture since ROB1-BT) on fakes (tester_rig::RobotTwoFakes — the
// pivots and the claw, each counting its safe-state applications), the route ids registered at
// construction — test/auton_tester_rig.hpp's banner says why.
//
// The watch reads a SCRIPTED status byte (what competition_get_status() returns on the robot); a
// test changes it at a time of its choosing through `onRead`, which the status source calls on
// every read with the robot clock. The request is served through the SAME functions the program's
// auton task calls (runAutonRequest / serveAutonRequest), with the team's real registry
// (src/team/autons.cpp), a real competition::AutonMenu, the handshake and the flags.
//
// Graded on PLANT GROUND TRUTH (PlantPacer::distance) and on the drive motors' commanded state.
// Host test plant numbers: instant motors, a tank that never skids. Nothing here has run on a
// robot (worksheet Station K).

#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "auton_tester_rig.hpp"
#include "motion_test_rig.hpp"
#include "common/state_screens.hpp"
#include "programs/competition_run.hpp"
#include "programs/competition_watch.hpp"
#include "team/autons.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/mechanism_routes.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/ui/touch_panel.hpp"
#include "shulib/units/quantity.hpp"

namespace comp_rig {

using shulib::units::Length;
using shulib::units::Time;

/// The status byte of an ENABLED autonomous period (a field: connected + autonomous).
inline constexpr std::uint8_t kAutonEnabled =
    shulib::bench::kStatusConnected | shulib::bench::kStatusAutonomous;
/// The same field, DISABLED (between periods, or a field disable mid-auton).
inline constexpr std::uint8_t kDisabledOnField =
    shulib::bench::kStatusConnected | shulib::bench::kStatusDisabled;
/// Driver control on a field (connected, enabled, not autonomous).
inline constexpr std::uint8_t kDriverOnField = shulib::bench::kStatusConnected;

/// The watch's status source (file banner). `onRead(now)` runs first on every read.
struct ScriptedStatus final : shulib::bench::ICompetitionStatus {
    std::uint8_t status = kAutonEnabled;
    shulib::hal::IClock* clock = nullptr;
    std::function<void(double)> onRead;
    int reads = 0;
    std::uint8_t read() override {
        ++reads;
        if (onRead) {
            onRead(clock->now().value());
        }
        return status;
    }
};

/// The probe layer (file banner): records, and may throw — as the tester rig's, over this watch.
struct ProbeLayer final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    shulib::sim::SimHarness* h = nullptr;
    motion_rig::PlantPacer* plant = nullptr;
    shulib::sequence::RunGuard* guard = nullptr;
    shulib::bench::CompetitionWatch* watch = nullptr;
    int paces = 0;
    int throwAfter = -1;  ///< throw on this pace (0-based count), -1 never
    tester_rig::ProbeThrow throwKind = tester_rig::ProbeThrow::Precondition;
    int energizedWhileIdle = 0;   ///< paces with a drive motor != 0 V and no run live
    double travelAtLatch = -1.0;  ///< plant travel when the watch was first seen latched
    int pacesAfterLatch = 0;      ///< paces the world advanced after the latch was first seen
    std::function<void()> onPace;  ///< a case's own look at the world, every pace (before the inner pace)

    void pace() override {
        if (throwAfter >= 0 && paces >= throwAfter) {
            throwAfter = -1;
            switch (throwKind) {
                case tester_rig::ProbeThrow::Precondition:
                    throw shulib::PreconditionError("probe: a pacer layer threw mid-auton");
                case tester_rig::ProbeThrow::Standard:
                    throw std::runtime_error("probe: a std::runtime_error mid-auton");
                case tester_rig::ProbeThrow::Unknown:
                    throw 7;
            }
        }
        ++paces;
        const bool running = guard != nullptr && guard->running();
        if (!running) {
            for (const shulib::hal::IMotor* m : h->context().driveMotors()) {
                if (m->commandedVoltage().value() != 0.0) {
                    ++energizedWhileIdle;
                    break;
                }
            }
        }
        if (watch != nullptr && watch->cause() != shulib::bench::CompetitionCause::None) {
            if (travelAtLatch < 0.0) {
                travelAtLatch = plant->distance;
            } else {
                ++pacesAfterLatch;
            }
        }
        if (onPace) {
            onPace();
        }
        inner->pace();
    }
};

/// What the request told the screen (the status page), and what had happened to the robot when it
/// was told (RC3: the status page comes BEFORE runAuton -- nothing moved, nothing armed yet).
struct RecordingObserver final : shulib::bench::ICompetitionRunObserver {
    int calls = 0;
    int entry = -2;
    shulib::bench::CompetitionSkip skip = shulib::bench::CompetitionSkip::None;
    bool throwOnStart = false;  ///< the painter throws (RC1 (a): a throw OUTSIDE runAuton)
    tester_rig::ProbeThrow throwKind = tester_rig::ProbeThrow::Standard;
    const shulib::competition::AutonMenu* menu = nullptr;  ///< the stack's menu
    const motion_rig::PlantPacer* plant = nullptr;          ///< the stack's plant
    const shulib::sequence::RunGuard* guard = nullptr;      ///< the stack's guard
    const shulib::bench::CompetitionWatch* watch = nullptr;  ///< the stack's watch
    bool lockedAtStart = false;        ///< the choice was LOCKED when the status page was told
    double distanceAtStart = -1.0;     ///< plant travel when it was told
    bool guardRunningAtStart = true;   ///< the guard was running when it was told
    bool watchArmedAtStart = true;     ///< the watch was armed when it was told
    void starting(const shulib::bench::CompetitionRunRecord& rec) override {
        ++calls;
        entry = rec.entry;
        skip = rec.skip;
        lockedAtStart = menu != nullptr && menu->locked();
        distanceAtStart = plant != nullptr ? plant->distance : -1.0;
        guardRunningAtStart = guard != nullptr && guard->running();
        watchArmedAtStart = watch != nullptr && watch->armed();
        if (throwOnStart) {
            switch (throwKind) {
                case tester_rig::ProbeThrow::Precondition:
                    throw shulib::PreconditionError("probe: the status painter threw");
                case tester_rig::ProbeThrow::Standard:
                    throw std::runtime_error("probe: the status painter threw a runtime_error");
                case tester_rig::ProbeThrow::Unknown:
                    throw 7;
            }
        }
    }
};

/// A telemetry sink that forwards to a FakeTelemetrySink and can be made to THROW on the Nth log()
/// (0-based) — a broken sink on the request path (RC1 (a)).
struct ThrowingSink final : shulib::hal::ITelemetrySink {
    shulib::hal::fake::FakeTelemetrySink inner;
    int calls = 0;
    int throwAt = -1;
    void log(shulib::hal::LogLevel level, std::string_view subsystem,
             std::string_view message) override {
        const int n = calls++;
        if (throwAt >= 0 && n >= throwAt) {
            throw std::runtime_error("probe: the log sink threw");
        }
        inner.log(level, subsystem, message);
    }
};

/// The whole stack (file banner). `kin` must outlive it. The world ticks idle for `idleTicks`
/// first (the robot on the field before autonomous: the estimate live, nothing commanded). The
/// menu is built over the team's registry; `choose(i)` picks entry i through a real touch.
struct CompetitionStack {
    tester_rig::WeakSide weak;
    shulib::hal::fake::FakeTelemetrySink sink;
    motion_rig::MotionRig rig;
    motion_rig::PlantPacer plant;
    ProbeLayer probe;
    ScriptedStatus status;
    shulib::bench::CompetitionWatch watch;
    tester_rig::RobotTwoFakes two;                   // before the service: bind()'s lifetime rule
    shulib::mechanism::MechanismService mechanisms;  // the guard's inner pacer; the watch is its
    shulib::sequence::RunGuard guard;
    shulib::chassis::Chassis chassis;
    shulib::runner::MechanismRoutes routes;          // robot two's six route ids ...
    shulib::runner::CommandRegistry commands;        // ... registered here (holds pointers into routes)
    shulib::competition::AutonRegistry registry{shulib::bench::kAutonPeriods};
    std::optional<shulib::competition::AutonMenu> menu;
    shulib::bench::AutonHandshake handshake;
    shulib::bench::CompetitionFlags flags;
    RecordingObserver obs;
    shulib::bench::CompetitionRunRecord rec{};
    ThrowingSink* telemetryOverride = nullptr;  ///< when set, the parts' telemetry
    std::int32_t releases = 0;
    double menuClock = 0.0;  ///< the menu's own clock for choose(): only ever moves forward

    explicit CompetitionStack(const shulib::kinematics::IKinematics& kin, int weakWheel = 1,
                              double weakScale = 1.0, int idleTicks = 50,
                              void (*extraAutons)(shulib::competition::AutonRegistry&) = nullptr,
                              bool absentPivot7 = false)
        : weak{},
          rig{kin, motion_rig::plantConfig(), &sink, &weak},
          plant{rig.h},
          watch{probe, status},
          two{absentPivot7},
          mechanisms{watch},
          guard{mechanisms},
          chassis{rig.deps, guard, motion_rig::chassisConfig()},
          routes{chassis, mechanisms, mech_fixture::robot_two::kRobotTwoMechanisms, sink} {
        // THE PROGRAM'S BOOT ORDER: the service bound after the chassis exists, then the route ids.
        mechanisms.bind(mech_fixture::robot_two::kRobotTwoMechanisms, two.devs, rig.h.clock(), rig.latch, sink);
        routes.registerInto(commands);
        weak.wheel = weakWheel;
        weak.scale = weakScale;
        probe.inner = &plant;
        probe.h = &rig.h;
        probe.plant = &plant;
        probe.guard = &guard;
        probe.watch = &watch;
        status.clock = &rig.h.clock();
        watch.attachGuard(guard);
        if (extraAutons != nullptr) {
            extraAutons(registry);
        }
        shulib::bench::registerAutons(registry);
        menu.emplace(registry, releases);
        obs.menu = &*menu;
        obs.plant = &plant;
        obs.guard = &guard;
        obs.watch = &watch;
        flags.bootFromRecord = true;
        for (int i = 0; i < idleTicks; ++i) {
            chassis.wait(Time{0.01});
        }
    }

    [[nodiscard]] double now() { return rig.h.clock().now().value(); }

    /// Choose registry entry `index` (-1: NONE) through real touches on the menu's drawn frames,
    /// each after its arming delay — the way a person chooses before the match: NEXT / PREV until
    /// the entry's page is up (two autons per page), then its button. The menu keeps its own clock
    /// (`menuClock`), which only moves forward, so every new layout's arming delay is honoured.
    void choose(int index) {
        using shulib::competition::MenuButtonId;
        const auto tap = [this](const shulib::competition::MenuView& drawn,
                                const shulib::competition::MenuButton& btn) {
            shulib::ui::TouchSample s{};
            s.releaseCount = ++releases;
            s.x = static_cast<std::int16_t>((btn.rect.x0 + btn.rect.x1) / 2);
            s.y = static_cast<std::int16_t>((btn.rect.y0 + btn.rect.y1) / 2);
            (void)menu->touch(s, drawn);
        };
        const int wantPage =
            index < 0 ? menu->page()
                      : index / static_cast<int>(shulib::competition::kMenuEntriesPerPage);
        for (int flips = 0; flips < 64; ++flips) {
            (void)menu->view(menuClock);
            menuClock += 1.0;
            const shulib::competition::MenuView v = menu->view(menuClock);  // armed
            const MenuButtonId step = v.page < wantPage   ? MenuButtonId::Next
                                      : v.page > wantPage ? MenuButtonId::Prev
                                                          : MenuButtonId::Entry;
            for (std::uint8_t b = 0; b < v.buttonCount; ++b) {
                const shulib::competition::MenuButton& btn = v.buttons[b];
                const bool match =
                    step != MenuButtonId::Entry
                        ? btn.id == step
                        : (index < 0 ? btn.id == MenuButtonId::None
                                     : (btn.id == MenuButtonId::Entry && btn.entry == index));
                if (match) {
                    tap(v, btn);
                    if (step == MenuButtonId::Entry) {
                        REQUIRE(menu->choice() == index);
                        return;
                    }
                    break;
                }
            }
        }
        FAIL("choose(): no button for entry ", index, " on any page the menu pages to");
    }

    shulib::bench::CompetitionParts parts() {
        return shulib::bench::CompetitionParts{
            chassis,
            guard,
            watch,
            *menu,
            registry,
            telemetryOverride != nullptr ? static_cast<shulib::hal::ITelemetrySink&>(*telemetryOverride)
                                         : static_cast<shulib::hal::ITelemetrySink&>(sink),
            mechanisms,
            &commands,
            handshake,
            flags};
    }

    /// One request as autonomous() makes it and the auton task serves it (serve only).
    const shulib::bench::CompetitionRunRecord& serve() {
        const std::uint32_t n = handshake.request(now());
        std::uint32_t request = 0;
        double at = 0.0;
        REQUIRE(handshake.pending(handshake.finished(), request, at));
        REQUIRE(request == n);
        shulib::bench::CompetitionParts p = parts();
        return shulib::bench::serveAutonRequest(p, rec, request, at, static_cast<int>(request), obs);
    }

    /// Robot two's PRESENT mechanisms each at its safe state (a pivot at 0 V under Hold, the claw's line
    /// LOW — closed); an ABSENT pivot is not asked.
    [[nodiscard]] bool mechanismsSafe() const {
        const bool seven = two.devs[1].motors == nullptr ||
                           (two.m7.commandedVoltage().value() == 0.0 && two.m7.brakeMode() == shulib::hal::BrakeMode::Hold);
        return two.m6.commandedVoltage().value() == 0.0 && two.m6.brakeMode() == shulib::hal::BrakeMode::Hold &&
               seven && !two.lineA.commanded();
    }

    /// Every drive motor at 0 V under Brake (the guard's and the program-error stop's safe state).
    [[nodiscard]] bool driveSafe() {
        for (const shulib::hal::IMotor* m : rig.h.context().driveMotors()) {
            if (m->commandedVoltage().value() != 0.0
                || m->brakeMode() != shulib::hal::BrakeMode::Brake) {
                return false;
            }
        }
        return true;
    }
};

}  // namespace comp_rig
