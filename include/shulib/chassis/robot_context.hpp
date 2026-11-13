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
// site); the constructor validates every REQUIRED one non-null, then the accessors hand out
// references so consumers write `ctx.clock().now()` rather than juggling pointers.
//
// ── The first OPTIONAL slot: the wall sensors (chunk L6-A2, 2026-11-03) ─────────────
// `walls` is the one slot the constructor does NOT require: nullptr means "this robot declares no
// wall sensors", and the wall reset then refuses NO SENSOR — so every existing caller (the host
// harness, the rigs, the Robot before L6-A2) compiles and runs unchanged. It is a pointer handed
// out as a pointer (`wallSensors()`), never a reference, because null is a real answer here. This
// context only FORWARD-DECLARES its type and never dereferences it (it sees only the L0 interfaces;
// the type lives in localization/wall_reset.hpp). The pointer is NON-CONST: the everyday wall reset
// files each outcome in the slot's log. motion::MotionScheduler's shadow context carries it field by
// field like every other slot — a slot not carried there is nullptr in every `chassis.deps().ctx`.

#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/battery.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/gps.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/hal/vision.hpp"

namespace shulib::localization {
/// Forward declaration only (localization/wall_reset.hpp defines it): this context stores the OPTIONAL
/// wall-sensor slot's pointer and never reads through it, so it includes no localization header.
struct WallSensors;
}  // namespace shulib::localization

namespace shulib::chassis {

/// The HAL handles a robot is built from. Pointers so fields are NAMED at the call site
/// (designated initializers) and so the RobotContext can validate them non-null.
struct RobotContextConfig {
    hal::IClock* clock = nullptr;             ///< the single source of now; seconds, monotonic
    /// The drive motors, in kinematic wheel order: element i is commanded with wheel speed i of
    /// the installed IKinematics, so a wrong order drives the robot the wrong way in silence.
    /// ONE entry per kinematic wheel — N physical motors on a side go behind one
    /// hal::MotorGroup (R3b Part 1). This context checks only that the span is non-empty and
    /// all-non-null; motion::MotionDeps::validate() is where the count is checked, and it
    /// demands EQUALITY with the kinematics' wheel count. This is a non-owning VIEW: the
    /// pointer ARRAY must outlive the context.
    std::span<hal::IMotor* const> driveMotors;
    hal::IImu* imu = nullptr;                 ///< heading and yaw rate, canonical CCW radians
    hal::IGps* gps = nullptr;                 ///< absolute fix; off-strip its hasFix() reads false
    hal::IBattery* battery = nullptr;         ///< volts: the pipeline's ceiling, and the run bookends
    hal::ITelemetrySink* telemetry = nullptr; ///< every log line and DebugRecord; NullSink = off
    hal::ITagSource* tags = nullptr;          ///< AprilTags as body-frame poses (the M3 corrector)
    hal::IVision* vision = nullptr;           ///< object bearings for M4 manipulation targeting
    /// OPTIONAL (L6-A2, appended): the robot's side distance sensors, their limits and the reset log —
    /// nullptr = this robot declares no wall sensors, and the wall reset refuses NO SENSOR. The ONE
    /// slot the constructor does NOT require. Non-const: the everyday wall reset files its outcome in
    /// the slot's log. Must outlive the context when set.
    localization::WallSensors* walls = nullptr;
};

/// The composition root: the ONE object that differs between the real robot, the simulator and a
/// host test. Every layer above the HAL reaches hardware only through it, so exchanging the HAL
/// implementations exchanges the whole robot without touching a line of motion code.
///
/// NON-OWNING throughout. It copies the config's pointers and span; it never adopts, allocates or
/// destroys anything, so every pointee — and the array the driveMotors span views — must outlive
/// the context. It also caches nothing: each accessor hands back the live handle, so a reading is
/// only ever as fresh as the caller's own call.
class RobotContext {
public:
    /// Validates the whole config up front — every REQUIRED handle non-null and driveMotors
    /// non-empty (`walls` is optional and never checked) — through SHULIB_PRECONDITION, so a mis-wired robot fails at construction naming the handle
    /// it is missing, instead of dereferencing null halfway through an auton. The count of drive
    /// motors is checked here only for emptiness; motion::MotionDeps::validate() checks it for
    /// EQUALITY against the kinematics' wheel count (the context never sees the kinematics).
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

    /// The run's clock. A reference, never null — the constructor already proved that, which is
    /// why consumers write `ctx.clock().now()` and never test a pointer.
    [[nodiscard]] hal::IClock& clock() const { return *cfg_.clock; }
    /// The drive motors, in kinematic wheel order. A VIEW of the caller's array, so it is only
    /// as alive as that array; guaranteed non-empty here, and guaranteed to EQUAL the wheel
    /// count of the installed kinematics once a motion::MotionDeps has validated the pairing.
    [[nodiscard]] std::span<hal::IMotor* const> driveMotors() const { return cfg_.driveMotors; }
    /// The IMU, already canonical: CCW-positive radians, +X = 0. Nothing above this call converts
    /// an angle. Gate trust on isReady() at boot — a calibrating IMU reports garbage that moves.
    [[nodiscard]] hal::IImu& imu() const { return *cfg_.imu; }
    /// The GPS. Check hasFix() before believing pose(): off-strip the pose is unspecified (still
    /// finite), and Driving Skills has no strip at all, so this seam is silent for a whole run.
    [[nodiscard]] hal::IGps& gps() const { return *cfg_.gps; }
    /// The battery. Read live at each use — the command pipeline takes its voltage ceiling from
    /// it every tick and the run summary samples it at both ends; nothing here caches a volt.
    [[nodiscard]] hal::IBattery& battery() const { return *cfg_.battery; }
    /// The one diagnostics sink for this robot. Which sink is installed is what decides whether
    /// tracing costs anything: with NullSink the per-tick record is never even populated.
    [[nodiscard]] hal::ITelemetrySink& telemetry() const { return *cfg_.telemetry; }
    /// The AprilTag source — V5 AI Vision or a coprocessor, indistinguishable from here. Tags
    /// arrive as body-frame poses with no timestamp; staleness is the corrector's problem.
    [[nodiscard]] hal::ITagSource& tags() const { return *cfg_.tags; }
    /// The object/colour detection source. A separate seam from tags() so one adapter can serve
    /// both, or either alone, without a consumer of one depending on the other.
    [[nodiscard]] hal::IVision& vision() const { return *cfg_.vision; }
    /// The robot's side distance sensors (L6-A2) — nullptr when this robot declares none (the one
    /// OPTIONAL slot; the wall reset then refuses NO SENSOR). A pointer, as handed in: this context
    /// never dereferences it.
    [[nodiscard]] localization::WallSensors* wallSensors() const noexcept { return cfg_.walls; }

private:
    RobotContextConfig cfg_;
};

}  // namespace shulib::chassis
