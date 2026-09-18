#pragma once
//
// DriveGeometry — ONE description of a drive side's geometry, consumed by BOTH the
// drive-encoder odometry and the stall cross-check (chunk R3b Part 2, 2026-09-14).
//
// ── Why one type, and why it lives here ─────────────────────────────────────────────
// Converting a drive motor's shaft radians into inches of wheel travel needs two facts
// about the mechanism: the wheel's radius and the external gearing between the motor's
// output shaft and the wheel. Two consumers need that conversion — the drive-encoder
// odometry (localization/drive_encoder_odometry.hpp) and the spin-vs-motion stall check
// (motion/odo_stall_check.hpp) — and until this chunk each had a different answer: the
// stall check carried ONE scalar `wheelRadius` for every drive motor (with a comment A29
// predicted would be wrong on a geared robot), and the odometry did not exist. The R3b
// brief's ruling (first-closed-loop §4.1, session-2 §5.3): the per-side ratio lives WITH
// THE MOTORS, never in F5 kinematics (which answers "how fast should each side's wheel
// travel", symmetric and frozen), and it is represented ONCE, instantiated once per side
// at the composition root, and handed to both consumers. This header is that one type,
// in hal/ because it describes the physical motor-to-wheel mechanism the adapters sit on
// and because both consumers can include hal/ without a layering exception.
//
// Asymmetric sides are representable — two objects, one per side — because a real
// drivetrain has been seen with "the right side geared down a touch for traction"
// (the bench bot's legacy source, recorded in the development log on the shulib-v2
// branch). Robot two is BELIEVED symmetric; belief is not a reason to make
// asymmetry unrepresentable.
//
// The defaults are the tree's STAND-IN geometry (3.25 in wheel, 1:1 — A4 register HA-14),
// carried here unchanged from OdoStallCheckConfig so a config that names no geometry
// behaves exactly as before. Robot two's measured values arrive through the chassis
// table (src/chassis_table.hpp), never through these defaults.
//
// ── TWO CONSTRUCTORS, TWO PROVENANCES — and what wheelRadius then means ─────────────
// fromDiameter(d, ratio) is what a HUMAN measures: a ruler across the tread and a pair of
// tooth counts, two separable facts.
//
// fromInchesPerRadian(k) is what a ROBOT measures (chunk R3d Part 1, T5): pushing the
// chassis a known distance along a tape and dividing by the shaft radians it turned yields
// ONE number, and nothing in that measurement can separate the radius from the ratio. So
// the calibrated constructor stores k in `wheelRadius` with `motorToWheelRatio = 1.0`, and
// — stated here because it is the one surprising thing about this type — **a CALIBRATED
// geometry's `wheelRadius` field carries the PRODUCT, not a radius.** Both consumers
// (drive_encoder_odometry.hpp, odo_stall_check.hpp) read only inchesPerRadian(), so neither
// can tell the difference and neither needs to; only a PANEL does, and a panel showing a
// calibrated description must print "ipr", never "radius".
// Rejected: a third field or a tagged variant on a type two consumers already hold BY VALUE
// (a shape change that buys no consumer anything); and storing a guessed diameter with a
// ratio derived to match — that is HA-111's invented-constant defect class in a new coat,
// two made-up factors standing in for one measured number.

#include <cmath>

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// One drive side's motor-to-wheel geometry: wheel radius and external motor→wheel ratio,
/// yielding inches of wheel travel per radian of motor output-shaft rotation. Plain data
/// (designated initializers at the call site), validated by whoever consumes it
/// (valid()). Instantiate one per side at the composition root and hand the SAME object to
/// the odometry and the stall check — one representation, two consumers.
struct DriveGeometry {
    /// Wheel RADIUS (inches) — EXCEPT in a geometry built by fromInchesPerRadian(), where this
    /// field carries the whole radius × ratio PRODUCT and the ratio beside it is 1.0 (see the
    /// header's "two constructors, two provenances"). Stand-in default: half of 3.25 in
    /// (A4: HA-14).
    units::Length wheelRadius{3.25 / 2.0};
    /// External gearing, motor output shaft → wheel: wheel revolutions per motor revolution
    /// (1.0 = direct drive; 0.6 = a 5:3 reduction; the cartridge is NOT included — the
    /// adapter already reports OUTPUT-shaft rotation). Stand-in default 1:1 (A4: HA-14).
    double motorToWheelRatio = 1.0;

    /// Inches of wheel travel per radian of motor output-shaft rotation: radius × ratio.
    [[nodiscard]] constexpr units::Length inchesPerRadian() const noexcept {
        return units::Length{wheelRadius.value() * motorToWheelRatio};
    }

    /// A geometry built from what a human measures: the wheel DIAMETER (a ruler across the
    /// tread) and the ratio (tooth counts, or 1.0 for direct drive).
    [[nodiscard]] static constexpr DriveGeometry fromDiameter(units::Length wheelDiameter,
                                                              double ratio = 1.0) noexcept {
        return DriveGeometry{units::Length{0.5 * wheelDiameter.value()}, ratio};
    }

    /// A geometry built from what the ROBOT measures (chunk R3d Part 1): inches of wheel travel
    /// per radian of motor output-shaft rotation, straight from a push of a known distance along
    /// a tape. The radius and the ratio are NOT separable from that measurement and are not
    /// invented here: the product is stored in `wheelRadius` with a ratio of exactly 1.0, so
    /// inchesPerRadian() returns it unchanged (× 1.0 is exact). A calibrated geometry's
    /// `wheelRadius` is therefore NOT a radius — see the header.
    [[nodiscard]] static constexpr DriveGeometry fromInchesPerRadian(
        units::Length inchesPerRadian) noexcept {
        return DriveGeometry{inchesPerRadian, 1.0};
    }

    /// Both fields finite and strictly positive — what a consumer's precondition checks.
    /// UNSET (0) is invalid on purpose: a geometry nobody measured must refuse, never
    /// silently scale travel by zero.
    [[nodiscard]] bool valid() const noexcept {
        return std::isfinite(wheelRadius.value()) && wheelRadius.value() > 0.0
               && std::isfinite(motorToWheelRatio) && motorToWheelRatio > 0.0;
    }
};

}  // namespace shulib::hal
