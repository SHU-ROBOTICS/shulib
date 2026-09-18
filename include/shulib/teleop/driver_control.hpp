#pragma once
//
// DriveShaper — the whole feel layer in one object and one call per tick
// (chunk T2). Sticks and a dt go in, a body-frame DriveRequest comes out.
//
// THE ORDER IS THE CONTRACT: mode -> shaping -> slew -> rotation.
//
//   1. MODE (drive_modes.hpp)      raw sticks -> a DriveRequest, signs applied,
//                                  nothing shaped. A disconnected controller is
//                                  all zero here (HA-103).
//   2. SHAPING (input_shaping.hpp) per-axis continuous deadband, then expo.
//   3. SLEW (slew_limiter.hpp)     per-axis rate limit. OFF by default.
//   4. ROTATION (field_centric.hpp) field-centric's minus-heading rotation, or
//                                  its latched fallback. Only for ArcadeField.
//
// Neither adjacent pair commutes, and both orderings were chosen rather than
// stumbled into:
//
//   * SHAPING BEFORE SLEW. A slew limit is specified in units of the signal it
//     limits, per second. Put a curve AFTER the limiter and the rate the driver
//     asked for is no longer the rate the drivetrain sees — expo would scale it
//     by a factor that depends on where the stick is. Shaping first means
//     "1.5 per second" means 1.5 of the commanded request per second, always.
//   * ROTATION LAST. The slew limit belongs in the DRIVER's frame. Limit after
//     rotating and a robot spinning under a perfectly still stick generates a
//     rotating target that the limiter then fights, turning "hold the stick
//     still" into a drifting, heading-dependent command. Rotating last leaves
//     the limiter looking at exactly what the driver's hands are doing.
//
// ONE CALL PER TICK, NO BUFFER, NO QUEUE, NO ALLOCATION. `shape()` computes
// this tick's answer from this tick's inputs; the only state it carries between
// ticks is the three limiters' last outputs and field-centric's zero and latch,
// which is the state a rate limit and a latch ARE. That is what makes the
// one-tick-latency property true by CONSTRUCTION rather than by measurement:
// a stick step at tick k reaches the commanded motor volts at tick k, pinned
// through the host plant in test/driver_control_test.cpp. PERCEIVED latency is
// NOT claimed — real input-to-motion latency is PROS call cost plus loop rate,
// and both are unmeasured.
//
// NUMBERS IN, A REQUEST OUT. This object never reads a controller and never
// reads a chassis. The caller builds a StickInput from its IController and
// passes the heading and its trust verdict as a number and a bool. A host test
// therefore drives the whole feel layer without either, and the two drivers VEX
// U runs are two DriveShapers rather than an argument anywhere.
//
// A DISCONNECTED CONTROLLER SHORT-CIRCUITS EVERYTHING, including the limiters,
// which are reset to zero rather than ramped down. See shape().
//
// EVERY DEFAULT IS INVENTED — PROVISIONAL (A4: HA-112). No driver has felt any
// of this. PROS-free, allocation-free, no std::string, no std::function.

#include "shulib/teleop/drive_modes.hpp"
#include "shulib/teleop/field_centric.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/slew_limiter.hpp"
#include "shulib/teleop/stick_mapping.hpp"

namespace shulib::teleop {

/// Everything a panel needs to show about the feel layer after a tick, and everything a test
/// needs to see inside it. Copied out by value; nothing here is a handle into the shaper.
struct DriveShaperState {
    /// Field-centric's indication for the tick just run — `NotSelected` while the profile's
    /// mode is a body-frame one (field_centric.hpp).
    FieldCentricState fieldCentric{};
    /// The three slew limiters' outputs after the tick just run — i.e. the request as it stood
    /// AFTER shaping and slew and BEFORE any field-centric rotation. Equal to the returned
    /// request whenever field-centric is not rotating, which is how a panel can show the cost
    /// of the rate limit without a second computation.
    DriveRequest slewOutput{};
    /// True when the tick just run saw a disconnected controller and zeroed everything
    /// (HA-103). Distinct from "the sticks were centred", which is the whole reason
    /// `isConnected()` exists.
    bool controllerAbsent = false;
};

/// The composed feel layer: one driver's profile, three rate limiters and one field-centric
/// mapper, called once per tick.
///
/// Construct one per driver, hold it for the life of the program, and call `shape()` in the
/// teleop loop. It is copyable (it is just numbers), but a copy carries the limiters' history,
/// so copy it to snapshot a driver's tuning, not to hand control over.
class DriveShaper {
public:
    /// A shaper running `profile` — by default the shipped profile: deadband 0.05, no expo, no
    /// slew, ArcadeBody. The profile is COPIED, so later edits to the caller's object never
    /// reach a live shaper; `setProfile()` is the way to change one.
    explicit constexpr DriveShaper(const DriverProfile& profile = {}) noexcept
        : profile_{profile},
          forward_{profile.slew},
          left_{profile.slew},
          yaw_{profile.slew} {}

    /// One tick: sticks and elapsed time in, the body-frame request to command out.
    ///
    /// The order is mode -> shaping -> slew -> rotation (see the file header for why each step
    /// sits where it does). The returned request is always in the ROBOT's frame, so the caller
    /// commands `Frame::Body` whatever the mode — field-centric is handled here and never by
    /// the facade.
    ///
    /// `dtSeconds` is the time since the previous call, from the caller's pacer. It is only
    /// read by the rate limiters, so with the shipped (slew-off) profile its value cannot
    /// change the answer at all. A non-positive or NaN dt freezes the limiters rather than
    /// letting them take an unbounded step.
    ///
    /// `headingRadians` / `headingTrusted` are the robot's estimated heading and the caller's
    /// verdict on it (the estimate is live and no IMU_LOST fault is latched). Both are ignored
    /// unless the profile's mode is `ArcadeField`.
    ///
    /// A DISCONNECTED CONTROLLER (HA-103) returns the all-zero request AND resets the three
    /// limiters to zero. The limiters are reset rather than ramped because a controller that
    /// drops mid-match must stop the robot NOW: a rate limit exists to protect the drivetrain
    /// from a driver, not to keep a robot rolling after its driver is gone. Field-centric's
    /// latch is deliberately NOT disturbed — a lost radio link says nothing about the heading,
    /// and clearing the latch behind the driver's back is exactly the flicker the latch exists
    /// to prevent.
    [[nodiscard]] DriveRequest shape(const StickInput& in, double dtSeconds,
                                     double headingRadians, bool headingTrusted) noexcept {
        if (!in.connected) {
            forward_.reset();
            left_.reset();
            yaw_.reset();
            state_ = DriveShaperState{.fieldCentric = fieldCentric_.state(),
                                      .slewOutput = DriveRequest{},
                                      .controllerAbsent = true};
            return DriveRequest{};
        }

        // 1. mode: raw sticks -> a request, signs applied, unshaped.
        const DriveRequest mapped = modeRequest(profile_.mode, in);
        // 2. shaping: continuous deadband, then expo, per axis.
        const DriveRequest shaped = shapeRequest(mapped, profile_);
        // 3. slew: per-axis rate limit, in the DRIVER's frame (off by default).
        const DriveRequest limited{forward_.update(shaped.forward, dtSeconds),
                                   left_.update(shaped.left, dtSeconds),
                                   yaw_.update(shaped.yawCcw, dtSeconds)};
        // 4. rotation: field-centric, or nothing at all.
        if (profile_.mode != DriveMode::ArcadeField) {
            state_ = DriveShaperState{.fieldCentric = FieldCentricState{},
                                      .slewOutput = limited,
                                      .controllerAbsent = false};
            return limited;
        }
        const FieldCentric::Result fc = fieldCentric_.request(limited, headingRadians,
                                                              headingTrusted);
        state_ = DriveShaperState{.fieldCentric = fc.state,
                                  .slewOutput = limited,
                                  .controllerAbsent = false};
        return fc.request;
    }

    /// What the last `shape()` call did — for a panel, and for a test that wants to see inside
    /// the pipeline. All-default before the first tick.
    [[nodiscard]] constexpr const DriveShaperState& state() const noexcept { return state_; }

    /// "The way the robot is facing now is away from me" — the driver's field-centric button.
    /// Sets the zero and clears the fallback latch (field_centric.hpp::rezero). Harmless in a
    /// body-frame mode, where it simply pre-arms the zero for a later mode switch.
    constexpr void rezero(double headingRadians) noexcept {
        fieldCentric_.rezero(headingRadians);
    }

    /// The profile in force. Returned by const reference; the shaper owns its copy.
    [[nodiscard]] constexpr const DriverProfile& profile() const noexcept { return profile_; }

    /// Swap in another driver's tuning. The limiters are rebuilt at the new rate and RESET —
    /// carrying a limiter's history across a rate change would make the first tick after the
    /// switch obey neither rate. Field-centric's zero and latch survive, because the driver's
    /// notion of "away from me" is not a tuning parameter.
    constexpr void setProfile(const DriverProfile& profile) noexcept {
        profile_ = profile;
        forward_ = SlewLimiter{profile.slew};
        left_ = SlewLimiter{profile.slew};
        yaw_ = SlewLimiter{profile.slew};
        state_ = DriveShaperState{};
    }

    /// Forget every tick that came before: limiters to zero, field-centric's zero and latch
    /// cleared, reported state blank. What a program calls when control changes hands or teleop
    /// restarts — never something a match does on its own.
    constexpr void reset() noexcept {
        forward_.reset();
        left_.reset();
        yaw_.reset();
        fieldCentric_.reset();
        state_ = DriveShaperState{};
    }

    /// Field-centric's own state, for a caller that wants the latch between ticks (the
    /// per-tick indication is in `state().fieldCentric`).
    [[nodiscard]] constexpr const FieldCentric& fieldCentric() const noexcept {
        return fieldCentric_;
    }

private:
    DriverProfile profile_{};
    SlewLimiter forward_{};
    SlewLimiter left_{};
    SlewLimiter yaw_{};
    FieldCentric fieldCentric_{};
    DriveShaperState state_{};
};

/// The whole feel layer plus the speed budgets, in one call — the teleop loop's line. Shapes
/// the sticks and scales the result into the body-frame `ChassisSpeeds` that
/// `Chassis::drive(…, Frame::Body)` takes.
///
/// The frame is ALWAYS Body, including in field-centric mode: the rotation already happened
/// inside `shape()`. `maxLinear` / `maxAngular` are the caller's MotionConfig budgets (HA-50,
/// provisional), passed in rather than read here so nothing in this header needs a chassis.
[[nodiscard]] inline math::ChassisSpeeds shapeToChassisSpeeds(DriveShaper& shaper,
                                                              const StickInput& in,
                                                              double dtSeconds,
                                                              double headingRadians,
                                                              bool headingTrusted,
                                                              units::Velocity maxLinear,
                                                              units::AngularVelocity maxAngular) {
    return toChassisSpeeds(shaper.shape(in, dtSeconds, headingRadians, headingTrusted), maxLinear,
                           maxAngular);
}

}  // namespace shulib::teleop
