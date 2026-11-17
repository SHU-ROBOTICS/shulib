#pragma once
//
// math::Reflection — the field plane reflected across a line through the origin, or left as it is (chunk
// MIRROR1-A, 2026-11-13; the MIRROR1 brief's §1.2 and D1/D7). PURE: it includes only math/ and units/
// (SHULIB_PRECONDITION reaches it through math/angle.hpp, which carries core/check.hpp for every math
// header). Not in the locked math/frame.hpp, whose banner allows "new vocabulary only".
//
// ── Why only lines at a multiple of 45 degrees ───────────────────────────────────────────────────
// A reflection across the line through the origin at heading a maps a point by the matrix
//     [ cos 2a   sin 2a ]
//     [ sin 2a  -cos 2a ]
// and a heading h to 2a - h. At a multiple of 45 degrees, 2a is a multiple of 90 degrees, so every
// coefficient is EXACTLY 0 or +-1: a position is reflected by swapping and negating coordinates — no
// rounding at all — and a reflected reflection gives the position back EXACTLY. Those are the only
// four lines a VEX field's symmetry uses (the two midlines and the two diagonals); any other line is a
// precondition, refused by acrossLineAt(). The four, in the field frame (F1: +x right as seen from the
// red station, +y away from it, heading 0 along +x, counterclockwise positive):
//     XAxis         y = 0      (x, y, h) -> ( x, -y,       -h)   — also the ROBOT-frame mirror (D7)
//     Diagonal      y = x      (x, y, h) -> ( y,  x,  90 deg - h)
//     YAxis         x = 0      (x, y, h) -> (-x,  y, 180 deg - h)
//     AntiDiagonal  y = -x     (x, y, h) -> (-y, -x, -90 deg - h)  — Override's SKILLS mirror (D1)
//
// ── What it reflects ─────────────────────────────────────────────────────────────────────────────
// A point, a Pose2d (position and heading), an Angle (2a - h, re-wrapped into F3's (-pi, pi]), and a
// ChassisSpeeds in either math::Frame: a FIELD velocity's (vx, vy) as a point and its turn rate negated;
// a BODY velocity as (vx, -vy, -omega) whatever the line — the mirror image of a robot drives the same
// speed forward, the same speed to its other side, and turns the other way.
//
// ── Two rules that keep its output safe to print and to compare ─────────────────────────────────
//  * NEVER -0.0. Reflecting negates, and -(0.0) prints "-0.0" — robot one's x = 0 must not reach robot
//    two's screen as "y = -0.0". Every component a non-identity reflection produces is normalised with
//    `+ 0.0` (in round-to-nearest, -0.0 + 0.0 is +0.0 and every other value is unchanged): the idiom
//    competition/toggle_visit.hpp's towardWallRadians already uses. A reflected reflection therefore
//    returns positions EQUAL to the original (an input -0.0 comes back +0.0, which compares equal), not
//    bit-identical.
//  * THE IDENTITY IS NOT APPLIED. apply() on the identity hands back its input untouched — and a caller
//    that must stay bit-identical when nothing is mirrored (the Chassis's verbs) branches on
//    isIdentity() and never calls apply() at all: re-wrapping a heading through Angle::radians can move
//    its last bit (the MIRROR1 brief's landmine 6).
// Headings follow F3 untouched: the result is wrapped into (-pi, pi], an exact -pi stored as +pi.
//
// PROS-free, allocation-free, header-only. Nothing here has run on a robot.

#include <cmath>
#include <cstdint>

#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::math {

/// The field plane reflected across a line through the origin at a multiple of 45 degrees, or the
/// identity (the default) — the file banner's table. Exact in position, idempotent in pairs (a
/// reflection applied twice gives positions back equal), never produces -0.0.
class Reflection {
public:
    /// The line the reflection is across (the file banner's table); None = the identity.
    enum class Line : std::uint8_t {
        None = 0,          ///< the identity: nothing is reflected
        XAxis = 1,         ///< y = 0: (x, -y, -h) — also the robot-frame mirror ("every left a right")
        Diagonal = 2,      ///< y = x: (y, x, 90 deg - h)
        YAxis = 3,         ///< x = 0: (-x, y, 180 deg - h)
        AntiDiagonal = 4,  ///< y = -x: (-y, -x, -90 deg - h) — Override's skills mirror
    };

    /// A field point (inches) — what a reflection maps a position to.
    struct Point {
        units::Length x{};  ///< field x, inches
        units::Length y{};  ///< field y, inches
    };

    /// The identity.
    constexpr Reflection() noexcept = default;

    /// The identity, by name.
    [[nodiscard]] static constexpr Reflection identity() noexcept { return Reflection{}; }

    /// The reflection across `line` (Line::None is the identity).
    [[nodiscard]] static constexpr Reflection across(Line line) noexcept { return Reflection{line}; }

    /// The reflection across the line through the origin at `heading` (either direction along it:
    /// -45 deg and 135 deg are the same line). PRECONDITION: `heading` is a multiple of 45 degrees
    /// (within 1e-9 deg) — any other line would need irrational coefficients and lose exactness.
    [[nodiscard]] static Reflection acrossLineAt(Angle heading) {
        const double deg = heading.degrees();
        const double k = std::round(deg / 45.0);
        SHULIB_PRECONDITION(std::abs(deg - 45.0 * k) <= 1e-9,
                            "math::Reflection::acrossLineAt: the line must lie at a multiple of 45 deg (a "
                            "midline or a diagonal of the field) -- any other line is not exact");
        const int quarter = ((static_cast<int>(k) % 4) + 4) % 4;  // 0, 45, 90, 135 deg
        switch (quarter) {
            case 0: return across(Line::XAxis);
            case 1: return across(Line::Diagonal);
            case 2: return across(Line::YAxis);
            default: return across(Line::AntiDiagonal);
        }
    }

    /// True for the identity (nothing is reflected).
    [[nodiscard]] constexpr bool isIdentity() const noexcept { return line_ == Line::None; }
    /// The line (Line::None for the identity).
    [[nodiscard]] constexpr Line line() const noexcept { return line_; }

    /// Two reflections are equal when they are across the same line.
    [[nodiscard]] friend constexpr bool operator==(const Reflection& a, const Reflection& b) noexcept {
        return a.line_ == b.line_;
    }

    /// The point (x, y) reflected — EXACT (the coefficients are 0 / +-1), never -0.0. The identity
    /// returns it untouched.
    [[nodiscard]] constexpr Point apply(units::Length x, units::Length y) const noexcept {
        const double px = x.value();
        const double py = y.value();
        switch (line_) {
            case Line::None: return Point{x, y};
            case Line::XAxis: return point(px + 0.0, -py + 0.0);
            case Line::Diagonal: return point(py + 0.0, px + 0.0);
            case Line::YAxis: return point(-px + 0.0, py + 0.0);
            case Line::AntiDiagonal: return point(-py + 0.0, -px + 0.0);
        }
        return Point{x, y};
    }
    /// The point `p` reflected (as apply(x, y)).
    [[nodiscard]] constexpr Point apply(const Point& p) const noexcept { return apply(p.x, p.y); }

    /// The heading `h` reflected: 2a - h for the line at heading a, wrapped into (-pi, pi] (F3), never
    /// -0.0. The identity returns it untouched.
    [[nodiscard]] Angle apply(Angle h) const {
        if (line_ == Line::None) {
            return h;
        }
        // remainder() brings 2a - h into [-pi, pi] (|2a - h| <= 3pi/2 here); + 0.0 turns a -0.0 into 0.0;
        // Angle::radians then folds an exact -pi up to +pi, F3's one rule.
        const double r = std::remainder(twoAlpha() - h.radians(), 2.0 * Angle::kPi) + 0.0;
        return Angle::radians(r);
    }

    /// The pose `p` reflected: its position and its heading. The identity returns it untouched.
    [[nodiscard]] Pose2d apply(const Pose2d& p) const {
        if (line_ == Line::None) {
            return p;
        }
        const Point q = apply(p.x(), p.y());
        return Pose2d{q.x, q.y, apply(p.heading())};
    }

    /// A chassis velocity in `frame` reflected (the file banner): FIELD — (vx, vy) as a point, the turn
    /// rate negated; BODY — (vx, -vy, -omega) for every line. Never -0.0. The identity returns it
    /// untouched.
    [[nodiscard]] ChassisSpeeds apply(const ChassisSpeeds& s, Frame frame) const noexcept {
        if (line_ == Line::None) {
            return s;
        }
        const units::AngularVelocity omega{-s.omega().value() + 0.0};
        if (frame == Frame::Body) {
            return ChassisSpeeds{units::Velocity{s.vx().value() + 0.0}, units::Velocity{-s.vy().value() + 0.0},
                                 omega};
        }
        const Point v = apply(units::Length{s.vx().value()}, units::Length{s.vy().value()});
        return ChassisSpeeds{units::Velocity{v.x.value()}, units::Velocity{v.y.value()}, omega};
    }

private:
    constexpr explicit Reflection(Line line) noexcept : line_{line} {}

    [[nodiscard]] static constexpr Point point(double x, double y) noexcept {
        return Point{units::Length{x}, units::Length{y}};
    }

    /// Twice the line's heading, radians, for the heading rule 2a - h.
    [[nodiscard]] constexpr double twoAlpha() const noexcept {
        switch (line_) {
            case Line::None:
            case Line::XAxis: return 0.0;
            case Line::Diagonal: return Angle::kPi / 2.0;
            case Line::YAxis: return Angle::kPi;
            case Line::AntiDiagonal: return -Angle::kPi / 2.0;
        }
        return 0.0;
    }

    Line line_ = Line::None;
};

}  // namespace shulib::math
