#pragma once
//
// RobotContext — the composition root / DI container: "the one object that differs across
// robot / sim / test" (master plan §5). Every layer above L0 (Chassis, Localizer, motion,
// skills) reads hardware ONLY through here, so swapping the HAL implementations
// (hal/fake ↔ hal/pros ↔ hal/sim) swaps the whole robot without touching a line of motion
// code — which is exactly the M1 Definition of Done.
//
// L3, PROS-free: it sees only the L0 interfaces. NOT part of the F4 freeze — it grows
// additively as M3/M4 consumers need more sensors (distance / optical / tracking-wheel
// arrays), so adding fields here later is safe.
//
// Built from a RobotContextConfig of NAMED pointers (designated initializers at the call
// site); the constructor validates them all non-null, then the accessors hand out
// references so consumers write `ctx.clock().now()` rather than juggling pointers.

#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/battery.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/gps.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/hal/vision.hpp"

namespace shulib::chassis {

/// The HAL handles a robot is built from. Pointers so fields are NAMED at the call site
/// (designated initializers) and so the RobotContext can validate them non-null.
struct RobotContextConfig {
    hal::IClock* clock = nullptr;
    std::span<hal::IMotor* const> driveMotors;  // in kinematic wheel order
    hal::IImu* imu = nullptr;
    hal::IGps* gps = nullptr;
    hal::IBattery* battery = nullptr;
    hal::ITelemetrySink* telemetry = nullptr;
    hal::ITagSource* tags = nullptr;
    hal::IVision* vision = nullptr;
};

class RobotContext {
public:
    explicit RobotContext(const RobotContextConfig& cfg) : cfg_{cfg} {
        SHULIB_PRECONDITION(cfg_.clock != nullptr, "RobotContext: clock is null");
        SHULIB_PRECONDITION(!cfg_.driveMotors.empty(), "RobotContext: driveMotors is empty");
        for (const hal::IMotor* m : cfg_.driveMotors) {
            SHULIB_PRECONDITION(m != nullptr, "RobotContext: a drive motor is null");
        }
        SHULIB_PRECONDITION(cfg_.imu != nullptr, "RobotContext: imu is null");
        SHULIB_PRECONDITION(cfg_.gps != nullptr, "RobotContext: gps is null");
        SHULIB_PRECONDITION(cfg_.battery != nullptr, "RobotContext: battery is null");
        SHULIB_PRECONDITION(cfg_.telemetry != nullptr, "RobotContext: telemetry is null");
        SHULIB_PRECONDITION(cfg_.tags != nullptr, "RobotContext: tags is null");
        SHULIB_PRECONDITION(cfg_.vision != nullptr, "RobotContext: vision is null");
    }

    [[nodiscard]] hal::IClock& clock() const { return *cfg_.clock; }
    [[nodiscard]] std::span<hal::IMotor* const> driveMotors() const { return cfg_.driveMotors; }
    [[nodiscard]] hal::IImu& imu() const { return *cfg_.imu; }
    [[nodiscard]] hal::IGps& gps() const { return *cfg_.gps; }
    [[nodiscard]] hal::IBattery& battery() const { return *cfg_.battery; }
    [[nodiscard]] hal::ITelemetrySink& telemetry() const { return *cfg_.telemetry; }
    [[nodiscard]] hal::ITagSource& tags() const { return *cfg_.tags; }
    [[nodiscard]] hal::IVision& vision() const { return *cfg_.vision; }

private:
    RobotContextConfig cfg_;
};

}  // namespace shulib::chassis
