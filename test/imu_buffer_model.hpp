#pragma once
//
// imu_buffer_model.hpp — the host test plant's model of the V5 IMU's 10 ms data copy, for the
// IMU1 tests (chunk IMU1 Part A, 2026-10-29; test/imu_rate_once_per_tick_test.cpp). It is the
// probe the IMU1 brief reproduced the defect with (its §0.4, probes 1 and 2), promoted into the
// suite.
//
// IT MODELS BELIEFS, NOT MEASUREMENTS — each registered in docs/hardware-assumptions.md:
//  * HA-213 — PROS copies the IMU's readings into the brain's shared buffer every 10 ms (the
//    vendored include/pros/imu.hpp, set_data_rate's doc: "values are copied into the shared
//    memory buffer only at 10ms intervals"), so every read between two copies returns the SAME
//    rotation: SAMPLE-AND-HOLD. Never observed on our brains.
//  * HA-214 — the copy's PHASE against the loop's wake is unknown (fixed per boot, or drifting),
//    so every test that uses this model sweeps it, and keeps it strictly OFF every read instant
//    (on a read instant the copy lands "at" the read and the answer is decided by rounding).
//  * HA-215 — a device read takes TIME, so two reads in one tick are microseconds apart on a robot
//    (`ProsClock` is `micros()`, HA-101): the clock this model hands the adapter CREEPS by `gap` per
//    device read and returns to the plant's time at each tick. `gap` is a guess, so it is swept.
//
// WHY THE MODEL EXISTS: every host clock (FakeClock, the PROS shim's micros()) stands still inside
// a tick, so a second read of ProsImu::yawRate() in a tick happened at the SAME instant on the host
// and repeated the first answer — the defect IMU1 fixes could not be seen by any host test until
// the clock moved inside a tick and the sensor held its value between copies. The model must be
// UNKIND (the IMU1 brief's landmine 2): a continuous sensor would make the old adapter look almost
// right. The R0 test proves it holds (and mutation M9, the copy made continuous, turns R0 red).
//
// THE CREEP IS THE IMU'S CLOCK ONLY (landmine 3): the plant's clock, the scheduler's and the
// Localizer's never move inside a tick — moving them would change every motion's dt and break a
// records-on vs records-off comparison for the wrong reason.
//
// Host only. Installed through the PROS host shim's `ImuPortState::rotationAt` hook
// (test/pros_shim/pros/imu.hpp), a plain function pointer — hence the one ACTIVE model below.
// One model at a time; `ScopedInstall` removes it again. Nothing here has run on a robot.

#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

#include "pros/shim_control.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace imu_model {

/// The copy period HA-213 believes in (10 ms) — the same number as the control tick by PROS's
/// default, which is what lets ONE read per tick see exactly one copy per interval.
inline constexpr double kCopyPeriodS = 0.010;

/// The IMU's shared buffer, modelled (file header). `truthCwDeg(t)` is the robot's TRUE cumulative
/// CW rotation at time `t` (seconds, the plant's epoch) — from a formula in the adapter tests, from
/// the plant's state history in the loop tests (PlantHistory, below).
struct BufferModel {
    double phase = 0.0;  ///< copies land at phase + j × 10 ms (s) — HA-214, swept by every caller
    double gap = 25e-6;  ///< seconds one device read takes — HA-215's guess, swept
    double creep = 0.0;  ///< seconds of device reads so far in this tick (gap × reads)
    /// The tick's start when no plant clock is attached (the adapter tests set it per tick).
    double tickStart = 0.0;
    /// When set, the tick's start is this clock's now() — the plant's, which stands still in a tick.
    const shulib::hal::IClock* plantClock = nullptr;
    std::function<double(double)> truthCwDeg;
    int deviceReads = 0;  ///< every get_rotation() that reached the model

    /// The plant's time at this tick.
    [[nodiscard]] double base() const {
        return plantClock != nullptr ? plantClock->now().value() : tickStart;
    }
    /// The IMU's clock: the plant's time plus the device reads made so far this tick.
    [[nodiscard]] double now() const { return base() + creep; }
    /// The instant of the last copy at or before `t` — the buffer's content is the truth THERE.
    [[nodiscard]] double lastCopyAt(double t) const {
        return phase + std::floor((t - phase) / kCopyPeriodS) * kCopyPeriodS;
    }
    /// One device read: it takes `gap` (the clock creeps FIRST — the read completes `gap` after
    /// the previous one), then returns the rotation at the last copy before its instant.
    double read() {
        creep += gap;
        ++deviceReads;
        return truthCwDeg(lastCopyAt(now()));
    }
    /// A new tick at `t` (the adapter tests): the IMU's clock returns to the tick's start.
    void beginTick(double t) {
        tickStart = t;
        creep = 0.0;
    }
    /// A new tick on the attached plant clock (the loop tests' pacer calls it after stepping).
    void beginTick() { creep = 0.0; }
};

/// The ONE model the shim hook reads (the hook is a plain function pointer).
inline BufferModel*& activeModel() {
    static BufferModel* m = nullptr;
    return m;
}

/// The shim hook: `ImuPortState::rotationAt`.
inline double hookedRotation(int /*port*/) { return activeModel()->read(); }

/// Installs `model` as port `port`'s rotation for the guard's lifetime; removes it after (the
/// shim's resetAll() also clears the hook).
struct ScopedInstall {
    ScopedInstall(BufferModel& model, int port) : port_{port} {
        activeModel() = &model;
        pros::shim::imuState(port_).rotationAt = &hookedRotation;
    }
    ~ScopedInstall() {
        pros::shim::imuState(port_).rotationAt = nullptr;
        activeModel() = nullptr;
    }
    ScopedInstall(const ScopedInstall&) = delete;
    ScopedInstall& operator=(const ScopedInstall&) = delete;

private:
    int port_;
};

/// The IClock the adapter under test reads: the model's creeping clock (file header).
struct CreepClock final : shulib::hal::IClock {
    explicit CreepClock(const BufferModel& m) : model{&m} {}
    [[nodiscard]] shulib::units::Time now() const override {
        return shulib::units::Time{model->now()};
    }
    const BufferModel* model;
};

/// The truth of the loop tests: the plant's heading recorded once per tick (after each step),
/// read back at a copy instant as the last record at or before it, advanced at that record's
/// rate — so a copy taken mid-tick sees the plant moving, not frozen at the tick's start.
struct PlantHistory {
    struct Snap {
        double t;
        double theta;  ///< canonical CCW radians
        double omega;  ///< canonical CCW rad/s
    };
    std::vector<Snap> snaps;
    double theta0 = 0.0;  ///< the heading the ProsImu's bootHeading stands for

    void record(double t, double theta, double omega) { snaps.push_back({t, theta, omega}); }
    /// The TRUE cumulative CW degrees at `c` (the IMU's convention: CW-positive, from boot).
    [[nodiscard]] double cwDegAt(double c) const {
        std::size_t i = snaps.size();
        while (i > 1 && snaps[i - 1].t > c + 1e-12) {
            --i;
        }
        const Snap& s = snaps[i - 1];
        const double theta = s.theta + s.omega * (c - s.t);
        return -(theta - theta0) * 180.0 / 3.14159265358979323846;
    }
};

/// An IImu that forwards to another and RECORDS every yawRate() answer with the tick it was asked
/// in (`*tick`, the caller's counter) — so a test can tell the Localizer's reads from a record's or
/// a gate's while one ProsImu serves them all (robot two wires one IMU to every reader).
struct SpyImu final : shulib::hal::IImu {
    SpyImu(const shulib::hal::IImu& inner, const int* tickCounter) : in{&inner}, tick{tickCounter} {}
    [[nodiscard]] shulib::math::Angle heading() const override { return in->heading(); }
    [[nodiscard]] shulib::units::AngularVelocity yawRate() const override {
        const shulib::units::AngularVelocity v = in->yawRate();
        reads.push_back({tick != nullptr ? *tick : 0, v.value()});
        return v;
    }
    [[nodiscard]] bool isReady() const override { return in->isReady(); }
    [[nodiscard]] shulib::math::Angle pitch() const override { return in->pitch(); }
    [[nodiscard]] shulib::math::Angle roll() const override { return in->roll(); }

    struct Read {
        int tick;
        double value;
    };
    const shulib::hal::IImu* in;
    const int* tick;
    mutable std::vector<Read> reads;
};

}  // namespace imu_model
