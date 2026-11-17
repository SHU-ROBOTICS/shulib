#pragma once
//
// fakes_by_kind.hpp — ANY robot's mechanism list on host fakes, by kind, for the Auton Tester's rig
// (test/auton_tester_rig.hpp's RigMechanisms) — chunk RPV1-B1, 2026-11-12 (the RPV1 brief's §1.2).
// Say "host test plant" — the project's one name for it: every plant here is host-only, and none is measured.
//
// ── What it builds, and from what ────────────────────────────────────────────────────────────
// The list is booted the way a program boots it: every mechanism's config derived from its
// declaration (mechanism::toMechanismConfig), built by the library's REAL robot::MechanismParts over
// the FAKE device provider (test/mechanism_service_rig.hpp's recipe — reuse, not a fork). Then, per kind:
//   * POWER — the provider's fake motors; nothing reads them back (a Power mechanism has no model).
//   * PNEUMATIC — the provider's fake lines, made at the declaration's start levels by the build.
//   * ARM / LIFT (Position) — the fake motors stepped by a sim::ArmPlant whose constants are the
//     DECLARATION'S OWN numbers: its gearing, the velocity feedforward the declaration runs with (kV —
//     the library's starting value unless `.tuning` states it), its friction (kS), its gravity term (kG,
//     cosine or constant, level angle), its travel limits as hard stops, and its home position as where
//     it starts. So it moves on the declaration's own numbers: its time is the declared profile plus the
//     controller's "reached" window, NOT exact (RP24 holds the lift inside [profile, profile + 0.35 s]).
//     The plant's job here is to show the timing the declaration implies, NOT to test the controller
//     (the mechanism suites do that against deliberately MISMATCHED plants — test/mechanism_auton_rig.hpp).
//     A home switch follows the plant (pressed at the home end, at the declared pressed level); an angle
//     sensor reads the joint through its declared mounting (an absolute one reads its stated value at
//     the home position).
//   * FLYWHEEL (Speed) — a sim::FlywheelPlant from the declaration's gearing, kV and kS, likewise.
//
// ── INERTIA: one ASSUMPTION, where the declaration is silent (RPV1-A's review, 2026-11-13) ─────────
// A declaration's kA is the controller's ACCELERATION FEEDFORWARD, and the library's starting value is
// 0 — "no feedforward", NOT "no inertia". A plant built with kA = 0 has none: it reaches the speed its
// voltage asks for in the same tick. On that unphysical plant a loop that reads the speed one tick late
// overshoots every tick — robot x's flywheel (kP / kV = 1.57) alternated 12 V and 6.86 V and never
// came within 60 rpm of 3000 (a FALSE red TIMED_OUT), and so did a Blue direct arm. No real mechanism
// does that. So when the declaration states no kA, the plant is given the host test plant's own time
// constant for that kind — kAssumedJointTauS (0.05 s; test/mechanism_auton_rig.hpp's lift) for an arm
// or lift, kAssumedWheelTauS (0.2 s; that rig's flywheel and sim::FlywheelPlantConfig's default) for a
// flywheel — kA = τ·kV. A STATED kA (an Arm's or a Lift's `.tuning`) is the team's own claim about the
// mechanism and is used as given. A Flywheel's words have no kA to state (mechanism::SpeedTuning holds
// kS, kV, kP and kI), so every declared flywheel runs on the assumed τ; only a SpeedSpec built by hand
// can carry its own. A flywheel's spin-up time is mostly that τ, so its steps carry the PLACEHOLDER
// note naming it (route_preview_run.hpp's placeholderFor); an arm's time is mostly its declared profile.
//   * PIECE SENSORS — never see a piece: the host test plant has no pieces (a distance sensor reads
//     PROS's honest "no object", an optical one proximity 0, a switch released). A step that waits for
//     a piece therefore ends as it would on a robot with an empty intake — the preview says so.
//
// Every pace steps every plant by one motion tick (0.01 s), then the plant pacer below (`inner`) —
// the order test/mechanism_auton_rig.hpp's XWorld uses. Host only (allocates).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "auton_tester_rig.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/robot/mechanism_parts.hpp"
#include "shulib/sim/mechanism_plant.hpp"

namespace tester_rig {

/// PROS's 9999 mm "no object", as the distance adapter converts it: the honest far reading.
inline constexpr double kFakesNoObjectIn = 9999.0 / 25.4;
/// An arm's or lift's time constant, s, when its declaration states no kA (file banner: INERTIA).
inline constexpr double kAssumedJointTauS = 0.05;
/// A flywheel's time constant, s, when its declaration states no kA (file banner: INERTIA).
inline constexpr double kAssumedWheelTauS = 0.2;

/// ANY robot's mechanisms on fakes, by kind (file banner). Non-copyable, non-movable: the built
/// mechanisms and the plants hold spans into this object.
class FakesByKind final : public MechanismWorldLayer {
public:
    /// Boot `table` (file banner). `table` must outlive this object.
    explicit FakesByKind(const shulib::mechanism::MechanismTable& table) : table_{&table} {
        namespace mech = shulib::mechanism;
        const std::size_t n = std::min<std::size_t>(table.count, mech::kMaxMechanisms);
        for (std::size_t i = 0; i < n; ++i) {
            const mech::MechanismSpec& s = table.specs[i];
            const mech::MechanismWiring& w = table.wiring[i];
            configs_[i] = mech::toMechanismConfig(s, w);
            std::size_t motors = 0;
            for (std::size_t m = 0; m < s.motorCount && m < mech::kMaxMotorRoles; ++m) {
                if (w.motors[m] != 0) {
                    motorPtrs_[i][motors++] = &provider_.fakeMotor(w.motors[m]);
                }
            }
            const std::span<shulib::hal::fake::FakeMotor* const> group{motorPtrs_[i].data(), motors};
            if (s.kind == mech::Kind::Position && motors > 0) {
                arms_.push_back(std::make_unique<shulib::sim::ArmPlant>(group, armPlantFrom(s)));
                shulib::sim::ArmPlant& plant = *arms_.back();
                const double homeDeg = startDegOf(s);
                if (w.homeSwitch != mech::Line::None) {
                    plant.attachSwitch(&provider_.fakeDigitalIn(static_cast<std::uint8_t>(w.homeSwitch)),
                                       shulib::sim::SwitchMount{.atDeg = homeAtLowEnd(s) ? homeDeg + kSwitchBandDeg
                                                                                         : homeDeg - kSwitchBandDeg,
                                                                .atTop = !homeAtLowEnd(s),
                                                                .pressedLevel = s.homeSwitch.pressedLevel});
                }
                if (w.angleSensor != 0) {
                    const double sign = s.angleSensor.reversed ? -1.0 : 1.0;
                    const double atZero = s.position.home == mech::HomeMethod::Absolute
                                              ? s.position.absoluteSensorAtZeroDeg - sign * homeDeg
                                              : 0.0;
                    plant.attachRotation(&provider_.fakeRotation(w.angleSensor),
                                         shulib::sim::RotationMount{.reversed = s.angleSensor.reversed,
                                                                    .readsAtJointZeroDeg = atZero});
                }
            } else if (s.kind == mech::Kind::Speed && motors > 0) {
                wheels_.push_back(std::make_unique<shulib::sim::FlywheelPlant>(group, flywheelPlantFrom(s)));
            }
            // Piece sensors never see a piece (file banner).
            if (w.pieceSensorPort != 0 && s.pieceSensor.kind == mech::SensorKind::Distance) {
                provider_.fakeDistance(w.pieceSensorPort).setConfidence(1.0);
                provider_.fakeDistance(w.pieceSensorPort).setDistance(shulib::units::Length{kFakesNoObjectIn});
            } else if (w.pieceSensorPort != 0 && s.pieceSensor.kind == mech::SensorKind::Optical) {
                provider_.fakeOptical(w.pieceSensorPort).setProximity(0.0);
            }
            if (w.pieceSwitch != mech::Line::None) {
                provider_.fakeDigitalIn(static_cast<std::uint8_t>(w.pieceSwitch)).setState(!s.pieceSensor.pressedLevel);
            }
        }
        parts_.build(std::span<const shulib::config::MechanismConfig>{configs_.data(), n}, provider_);
    }
    FakesByKind(const FakesByKind&) = delete;             ///< spans into this object (class note)
    FakesByKind& operator=(const FakesByKind&) = delete;  ///< spans into this object (class note)
    FakesByKind(FakesByKind&&) = delete;                  ///< spans into this object (class note)
    FakesByKind& operator=(FakesByKind&&) = delete;       ///< spans into this object (class note)
    ~FakesByKind() override = default;

    /// Every plant one motion tick, then the plant pacer (file banner).
    void pace() override {
        for (auto& a : arms_) {
            a->step(kTickSeconds);
        }
        for (auto& f : wheels_) {
            f->step(kTickSeconds);
        }
        inner->pace();
    }

    /// The binding a TesterStack takes (its devices, in the list's order, and this layer).
    [[nodiscard]] RigMechanisms binding() noexcept {
        return RigMechanisms{.table = table_, .devices = parts_.devices(), .world = this};
    }
    /// The built parts (a test reads a mechanism's motor or line through them).
    [[nodiscard]] const shulib::robot::MechanismParts& parts() const noexcept { return parts_; }
    /// The provider (a test reads a fake device by its port).
    [[nodiscard]] shulib::hal::fake::FakeDeviceProvider& provider() noexcept { return provider_; }
    /// The position plants, in the list's order of the Position mechanisms.
    [[nodiscard]] const std::vector<std::unique_ptr<shulib::sim::ArmPlant>>& arms() const noexcept { return arms_; }

    /// The ArmPlant constants a Position declaration implies (file banner): its own numbers.
    [[nodiscard]] static shulib::sim::ArmPlantConfig armPlantFrom(const shulib::mechanism::MechanismSpec& s) {
        namespace mech = shulib::mechanism;
        const mech::PositionSpec& p = s.position;
        shulib::sim::ArmPlantConfig c;
        c.gearRatio = p.gearRatio;
        c.kS = p.kS;
        c.kV = p.kV * kRadToDeg;  // V per output deg/s -> V per joint rad/s
        // V per output deg/s^2 -> V per joint rad/s^2; none stated: the assumed time constant (banner: INERTIA).
        c.kA = p.kA > 0.0 ? p.kA * kRadToDeg : kAssumedJointTauS * c.kV;
        c.kG = p.kG;
        c.constantGravity = p.gravity == mech::Gravity::Constant;
        c.levelDeg = p.horizontalAt;
        if (p.gravity == mech::Gravity::None) {
            c.kG = 0.0;
        }
        const double a = mech::positionToAngleDeg(p, p.minPosition);
        const double b = mech::positionToAngleDeg(p, p.maxPosition);
        c.minStopDeg = std::min(a, b);
        c.maxStopDeg = std::max(a, b);
        c.startDeg = startDegOf(s);
        c.encoderZeroDeg = c.startDeg;  // the encoders read 0 where the program started
        return c;
    }
    /// The FlywheelPlant constants a Speed declaration implies (file banner).
    [[nodiscard]] static shulib::sim::FlywheelPlantConfig flywheelPlantFrom(const shulib::mechanism::MechanismSpec& s) {
        shulib::sim::FlywheelPlantConfig c;
        c.gearRatio = s.speed.gearRatio;
        c.kS = s.speed.kS;
        c.kV = s.speed.kV;
        c.kA = s.speed.kA > 0.0 ? s.speed.kA : kAssumedWheelTauS * c.kV;  // none stated: banner, INERTIA
        return c;
    }
    /// Where a Position mechanism starts (joint degrees): its home position, inside its stops.
    [[nodiscard]] static double startDegOf(const shulib::mechanism::MechanismSpec& s) {
        namespace mech = shulib::mechanism;
        const mech::PositionSpec& p = s.position;
        const double a = mech::positionToAngleDeg(p, p.minPosition);
        const double b = mech::positionToAngleDeg(p, p.maxPosition);
        return std::clamp(mech::positionToAngleDeg(p, p.homePosition), std::min(a, b), std::max(a, b));
    }
    /// True when the home position is at the LOW end of the joint's travel (a switch pressed below it).
    [[nodiscard]] static bool homeAtLowEnd(const shulib::mechanism::MechanismSpec& s) {
        namespace mech = shulib::mechanism;
        const mech::PositionSpec& p = s.position;
        const double a = mech::positionToAngleDeg(p, p.minPosition);
        const double b = mech::positionToAngleDeg(p, p.maxPosition);
        const double h = startDegOf(s);
        return std::abs(h - std::min(a, b)) <= std::abs(h - std::max(a, b));
    }

private:
    static constexpr double kTickSeconds = 0.01;
    static constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    /// How far from the home end the switch still reads pressed (joint degrees).
    static constexpr double kSwitchBandDeg = 0.5;

    const shulib::mechanism::MechanismTable* table_;
    shulib::hal::fake::FakeClock clock_;
    shulib::hal::fake::FakeBattery battery_;
    shulib::hal::fake::FakeTelemetrySink providerSink_;
    struct NullPacer final : shulib::motion::ITickPacer {
        void pace() override {}
    } providerPacer_;
    shulib::hal::fake::FakeDeviceProvider provider_{clock_, battery_, providerSink_, providerPacer_};
    std::array<shulib::config::MechanismConfig, shulib::mechanism::kMaxMechanisms> configs_{};
    std::array<std::array<shulib::hal::fake::FakeMotor*, shulib::mechanism::kMaxMotorRoles>,
               shulib::mechanism::kMaxMechanisms>
        motorPtrs_{};
    std::vector<std::unique_ptr<shulib::sim::ArmPlant>> arms_;
    std::vector<std::unique_ptr<shulib::sim::FlywheelPlant>> wheels_;
    shulib::robot::MechanismParts parts_;
};

}  // namespace tester_rig
