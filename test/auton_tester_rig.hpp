#pragma once
//
// auton_tester_rig.hpp — the Auton Tester's pacer chain on the A2 host test plant, for
// test/autons_test.cpp and test/auton_tester_run_test.cpp (chunk COMP-t, 2026-09-28).
//
// The chain is the program's (src/programs/auton_tester_program.cpp) with the plant in place of
// the provider's 10 ms pacer and a PROBE layer in place of the painter:
//
//     Chassis -> RunGuard -> AbortWatch (REAL) -> ProbeLayer -> PlantPacer (the plant steps)
//
// The watch's inputs go through the SAME pure reductions the program uses — testerTouchFrom() on a
// struct with PROS's field names, abortInputsFrom() over a FakeController — so every run test also
// exercises the PROS mapping. A test scripts the world by time through `onRead`, which the inputs
// call on every read with the robot clock.
//
// The probe layer sits where the painter sits (below the watch, above the tick pacer): it sees
// every pace AFTER the guard's check and the watch's read and BEFORE the world advances, so it can
// record (a) whether any drive motor was commanded while no run was live, (b) the plant's travel at
// the pace the watch latched, and (c) throw on a chosen pace, as a broken layer would.
//
// Graded on PLANT GROUND TRUTH (h.truePose(), PlantPacer::distance), independent of the estimate
// the motions read. Host test plant numbers: instant motors, a tank that never skids.

#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>

#include "motion_test_rig.hpp"
#include "programs/auton_abort_watch.hpp"
#include "programs/auton_tester_run.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_controller.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/units/quantity.hpp"

namespace tester_rig {

using shulib::units::Length;
using shulib::units::Time;

/// PROS's screen_touch_status_s_t, by its field names and types (include/pros/screen.h), so the
/// program's reduction runs here unchanged.
enum FakeTouchStatus { kTouchReleased = 0, kTouchPressed, kTouchHeld, kTouchError };
/// See FakeTouchStatus.
struct FakeProsTouch {
    FakeTouchStatus touch_status = kTouchReleased;  // NOLINT: PROS's name
    std::int16_t x = 0;
    std::int16_t y = 0;
    std::int32_t press_count = 0;    // NOLINT: PROS's name
    std::int32_t release_count = 0;  // NOLINT: PROS's name
};

/// One side's volts scaled: wheel 0 = LEFT, 1 = RIGHT (the tank's kinematic order). A weak side
/// turns a tank toward it — the drift TANK1 steers out, and robot two's ~9 % (R3d, 2026-09-25).
struct WeakSide final : shulib::sim::DegradationModel {
    int wheel = 1;
    double scale = 1.0;
    shulib::units::Voltage effectiveVoltage(int w, shulib::units::Voltage commanded,
                                            shulib::units::Time,
                                            shulib::sim::Rng&) override {
        return w == wheel ? shulib::units::Voltage{commanded.value() * scale} : commanded;
    }
};

/// The watch's inputs: the controller, the touch struct and the disabled flag, reduced exactly as
/// the program reduces PROS's (abortInputsFrom(testerTouchFrom(...))). `onRead(now)` runs first on
/// every read, so a test changes the world at a time of its choosing.
struct ScriptedInputs final : shulib::bench::IAbortInputs {
    shulib::hal::fake::FakeController pad;
    FakeProsTouch touch{};
    bool disabled = false;
    shulib::hal::IClock* clock = nullptr;
    std::function<void(double)> onRead;
    int reads = 0;

    shulib::bench::AbortInputs read() override {
        ++reads;
        if (onRead) {
            onRead(clock->now().value());
        }
        return shulib::bench::abortInputsFrom(
            shulib::bench::testerTouchFrom(touch, static_cast<int>(kTouchError)), pad, disabled);
    }
    /// A finger lands on the glass (press_count rises; HA-178 (a)'s belief).
    void press(std::int16_t x = 240, std::int16_t y = 60) {
        touch.touch_status = kTouchPressed;
        touch.x = x;
        touch.y = y;
        ++touch.press_count;
    }
    /// The finger lifts (release_count rises).
    void release() {
        touch.touch_status = kTouchReleased;
        ++touch.release_count;
    }
};

/// What the probe layer throws (each of runTesterOnce's three handlers has its own case).
enum class ProbeThrow { Precondition, Standard, Unknown };

/// The painter's place in the chain (file banner): records, and may throw.
struct ProbeLayer final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    shulib::sim::SimHarness* h = nullptr;
    motion_rig::PlantPacer* plant = nullptr;
    shulib::sequence::RunGuard* guard = nullptr;
    shulib::bench::AbortWatch* watch = nullptr;
    int paces = 0;
    int throwAfter = -1;                ///< throw on this pace (0-based count), -1 never
    ProbeThrow throwKind = ProbeThrow::Precondition;  ///< what it throws
    bool energizeBeforeThrow = false;   ///< drive every motor at 6 V, then throw (a broken layer
                                        ///< that left the drive live: only the panic stop saves it)
    int energizedWhileIdle = 0;         ///< paces with a drive motor != 0 V and no run live
    double travelAtLatch = -1.0;        ///< plant travel when the watch was first seen latched
    double travelAtGuardArm = -1.0;     ///< plant travel on the first pace the guard was running

    void pace() override {
        if (throwAfter >= 0 && paces >= throwAfter) {
            throwAfter = -1;
            if (energizeBeforeThrow) {
                for (shulib::hal::IMotor* m : h->context().driveMotors()) {
                    m->setVoltage(shulib::units::Voltage{6.0});
                }
            }
            switch (throwKind) {
                case ProbeThrow::Precondition:
                    throw shulib::PreconditionError("probe: a pacer layer threw in the countdown");
                case ProbeThrow::Standard:
                    throw std::runtime_error("probe: a std::runtime_error in the countdown");
                case ProbeThrow::Unknown:
                    throw 7;  // not a std::exception at all: only catch (...) can take it
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
        } else if (travelAtGuardArm < 0.0) {
            travelAtGuardArm = plant->distance;
        }
        if (travelAtLatch < 0.0 && watch != nullptr
            && watch->cause() != shulib::bench::AbortCause::None) {
            travelAtLatch = plant->distance;
        }
        inner->pace();
    }
};

/// What runTesterOnce told the screen, and when (plus the plant's travel at runStarted).
struct RecordingObserver final : shulib::bench::ITesterRunObserver {
    motion_rig::PlantPacer* plant = nullptr;
    shulib::sim::SimHarness* h = nullptr;  ///< for throwAtRun's live drive
    double countdownAt = -1.0;
    double runAt = -1.0;
    double travelAtRun = -1.0;
    int countdownCalls = 0;
    int runCalls = 0;
    /// runStarted() leaves every drive motor at 6 V under Coast, then throws `throwKind`: a throw
    /// OUTSIDE every scheduler wait with the drive live -- the one case where runTesterOnce's own
    /// panic stop is the only thing that stops the robot (the scheduler's F2 unwind covers a throw
    /// through a wait, the guard's covers one through runAuton).
    bool throwAtRun = false;
    ProbeThrow throwKind = ProbeThrow::Precondition;
    void countdownStarted(double now) override {
        countdownAt = now;
        ++countdownCalls;
    }
    void runStarted(double now) override {
        runAt = now;
        travelAtRun = plant->distance;
        ++runCalls;
        if (throwAtRun) {
            throwAtRun = false;
            for (shulib::hal::IMotor* m : h->context().driveMotors()) {
                m->setBrakeMode(shulib::hal::BrakeMode::Coast);
                m->setVoltage(shulib::units::Voltage{6.0});
            }
            switch (throwKind) {
                case ProbeThrow::Precondition:
                    throw shulib::PreconditionError("probe: the observer threw at runStarted");
                case ProbeThrow::Standard:
                    throw std::runtime_error("probe: a std::runtime_error at runStarted");
                case ProbeThrow::Unknown:
                    throw 7;
            }
        }
    }
};

/// The whole stack (file banner). `kin` must outlive it. The world is ticked idle for
/// `idleTicks` first (the tester's READY page: the estimate live, nothing commanded).
struct TesterStack {
    WeakSide weak;
    shulib::hal::fake::FakeTelemetrySink sink;
    motion_rig::MotionRig rig;
    motion_rig::PlantPacer plant;
    ProbeLayer probe;
    ScriptedInputs inputs;
    shulib::bench::AbortWatch watch;
    shulib::sequence::RunGuard guard;
    shulib::chassis::Chassis chassis;
    RecordingObserver obs;
    shulib::bench::TesterRunRecord rec{};

    explicit TesterStack(const shulib::kinematics::IKinematics& kin, int weakWheel = 1,
                         double weakScale = 1.0,
                         const shulib::sim::SimHarnessConfig& cfg = motion_rig::plantConfig(),
                         shulib::sim::DegradationModel* degradation = nullptr, int idleTicks = 50)
        : weak{},
          rig{kin, cfg, &sink, degradation != nullptr ? degradation : &weak},
          plant{rig.h},
          watch{probe, inputs},
          guard{watch},
          chassis{rig.deps, guard, motion_rig::chassisConfig()} {
        weak.wheel = weakWheel;
        weak.scale = weakScale;
        probe.inner = &plant;
        probe.h = &rig.h;
        probe.plant = &plant;
        probe.guard = &guard;
        probe.watch = &watch;
        inputs.clock = &rig.h.clock();
        inputs.pad.setConnected(true);
        obs.plant = &plant;
        obs.h = &rig.h;
        watch.attachGuard(guard);
        for (int i = 0; i < idleTicks; ++i) {
            chassis.wait(Time{0.01});
        }
    }

    [[nodiscard]] double now() { return rig.h.clock().now().value(); }

    /// One run of registry entry `index`, as the program runs it: RUN's release read is NOW
    /// (its press count is the abort's touch baseline).
    const shulib::bench::TesterRunRecord& run(const shulib::competition::AutonRegistry& reg,
                                              int index, int number = 1,
                                              std::span<shulib::hal::IMechanism* const> present =
                                                  {}) {
        return runFrom(inputs.touch.press_count, reg, index, number, present);
    }

    /// One run whose RUN release was read EARLIER, with press count `runPressCount` (a finger
    /// may have landed since).
    const shulib::bench::TesterRunRecord& runFrom(std::int32_t runPressCount,
                                                  const shulib::competition::AutonRegistry& reg,
                                                  int index, int number = 1,
                                                  std::span<shulib::hal::IMechanism* const>
                                                      present = {}) {
        return shulib::bench::runTesterOnce(rec, number, chassis, guard, watch, reg, index, sink,
                                            present, obs, runPressCount);
    }

    /// Every drive motor at 0 V under Brake (the guard's and the panic stop's safe state).
    [[nodiscard]] bool driveSafe() {
        for (const shulib::hal::IMotor* m : rig.h.context().driveMotors()) {
            if (m->commandedVoltage().value() != 0.0
                || m->brakeMode() != shulib::hal::BrakeMode::Brake) {
                return false;
            }
        }
        return true;
    }

    /// How many motion tick records (a motion was active) since record `from`, and how many of
    /// them carry TSTR (TANK1's steering flag).
    void tstrCount(int from, int& motionRecords, int& tstr) {
        motionRecords = 0;
        tstr = 0;
        for (int i = from; i < sink.recordCount(); ++i) {
            const shulib::diag::DebugRecord& r = sink.recordAt(i);
            if (r.activeCommandState == 0) {
                continue;
            }
            ++motionRecords;
            tstr += r.tankSteerActive ? 1 : 0;
        }
    }
};

}  // namespace tester_rig
