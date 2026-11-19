#pragma once
//
// competition::FieldMirror — ONE ROUTE, TWO ROBOTS (chunk MIRROR1-A, 2026-11-13; the MIRROR1 brief's
// D1–D11, the team lead's GO of 2026-11-13: "do what is best they should play onto one side when they
// need to when mirrored, do what you need to do").
//
// ── What it is for ───────────────────────────────────────────────────────────────────────────────
// In a VEX U skills run both robots start on red's side, one in each red Quadrant (the red wall's and
// the right wall's), and those two Quadrants are mirror images of each other across the field's
// diagonal (competition/override_mirror.hpp says which line and why, from the game manual). So the
// students write the skills route ONCE, in robot one's numbers, and robot two's autons file registers
// the same entry MIRRORED, in one line:
//
//     registry.add(skillsRoute());                                                    // robot one
//     registry.add(competition::mirrored(skillsRoute(), override2026::kSkillsMirror));  // robot two
//
// `mirrored()` copies the entry and marks it; the auton runner (match_runner.hpp) does the rest when it
// runs that entry: it seats robot two at the REFLECTED start, and opens a chassis::MirroredRun around
// the routine (or route) AND its end action, so every field number the route writes is reflected at
// the Chassis's front door (chassis.hpp's MIRRORED RUN) and every pose the route reads comes back in
// robot one's numbers. The wall reset (auton_wall_reset.hpp) and the Toggle step (toggle_visit.hpp)
// work below that door; under the entry's mirror they swap the wall (its image), the sensor side (LEFT
// <-> RIGHT) and the Toggle (its image) themselves. Sizes — timeouts, speeds, tolerances, a trigger's
// percent / inches / degrees, mechanism goals — are not field numbers and do not change.
//
// ── The start mode decides WHICH reflection the run uses (the brief's D7) ──────────────────────────
//   FIELD start     the FieldMirror's own reflection (Override: (x, y, h) -> (-y, -x, -90 deg - h)).
//   RELATIVE start  the ROBOT-frame mirror (x, y, h) -> (x, -y, -h): "every left a right". A relative
//                   auton's numbers are in the frame of wherever the robot was placed; when robot two
//                   is placed at the field-mirror image of robot one's spot, the field's mirror IS this
//                   flip. The library cannot see where the drivers put the robot.
//
// ── BOTH ROBOTS RUN AT THE SAME TIME (the brief's D11) ─────────────────────────────────────────────
// A reflection leaves every point of its line where it is: whenever robot one's robot reaches the
// diagonal, robot two's reaches the same spot at the same moment. In a pure mirror the gap between the
// two robots is exactly twice robot one's clearance from the diagonal, whatever their timing — so a
// mirrored route keeps robot one's WHOLE robot at least 3 in off the diagonal (then the robots stay
// at least 6 in apart), and anything ON the diagonal (the centre goal, the two yellow Pins of the
// skills setup on it, the red/right corner's Load Zone) is for ONE robot, chosen in the shared route:
//
//     if (!competition::isMirrored(ctx)) { /* robot one only: the centre goal */ }
//
// The host suite runs every mirrored entry robot two registers TOGETHER with its unmirrored self and
// fails, naming the moment and the motions, when their 15 in squares come within 6 in
// (test/mirrored_pair_check.hpp; A4 register HA-260). On a TANK it also fails one step that drives
// AND turns about 180 deg, naming the steps to write instead: turn first, then drive (MIRROR1-B).
//
// ── What the mirror cannot carry (told, never guessed) ──────────────────────────────────────────────
// The ROBOT is not mirrored: a part mounted off its centre line (a side claw, an off-centre camera,
// the two side sensors at different distances ahead of the centre) meets the field on the other side
// of the reflected path (A4: HA-259); its own left/right drive differences are not mirrored either
// (HA-260). The field is not symmetric everywhere: Override's Loaders and the Load Zones' colours do
// not mirror (override_mirror.hpp). The camera's field inputs (a vision::PieceTarget, the pose handed
// to the piece finder or the Toggle tracker) are plain numbers the library cannot tell from any other:
// they are the REAL field — feed them the localizer's pose (`ctx.chassis.deps().localizer->pose()`),
// never `ctx.chassis.pose()` inside a mirrored run (robot one's numbers), and send a real-field fix to
// a Chassis verb through `ctx.chassis.reflection().apply(...)` first.
//
// ── Refused by name (the brief's D5) ───────────────────────────────────────────────────────────────
//   * an entry mirrored by two DIFFERENT mirrors — a precondition at mirrored() (twice by the same
//     mirror gives the entry back as written: mirrored() TOGGLES);
//   * a mirrored entry whose name is over kMaxMirroredAutonNameChars (25) — refused at registration by
//     the registry's own check (its menu button reads "<name> SKILLS MIR" on one line);
//   * the EXPLICIT wall reset (chassis::resetFromWall over a sensor handle) inside a mirrored run — a
//     precondition naming the everyday form, which swaps the sensor side.
// A refusal at run time can only happen when its line runs, because ONE function serves both robots:
// the host suite runs every auton of robot two's registry on the host test plant, mirrored ones
// included, before the robot ever does.
//
// PROS-free, allocation-free, header-only. NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>
#include <cstdint>
#include <span>

#include "shulib/competition/auton_registry.hpp"
#include "shulib/core/check.hpp"
#include "shulib/localization/axis.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/reflection.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::competition {

/// A field's mirror (the file banner): its name for the log, its reflection, and the season's walls —
/// the wall reset's wall is mapped to the season wall at its reflected line. A SEASON CONSTANT with
/// static storage (an entry keeps a pointer to it): `override2026::kSkillsMirror`.
struct FieldMirror {
    /// Its name in the log ("the skills diagonal") — a stable literal, ASCII.
    const char* name = "";
    /// The reflection of the FIELD frame (never the identity — mirrored() refuses one).
    math::Reflection reflection{};
    /// The season's walls (localization::override2026::kWalls): the image of a season wall is the
    /// season wall at its reflected line, with ITS real name and keep-outs.
    std::span<const localization::FieldWall> walls{};
};

/// The sensor on the OTHER side: LEFT <-> RIGHT. Every reflection swaps a robot's sides — robot one's
/// right sensor along a wall is robot two's left sensor along the image wall.
[[nodiscard]] constexpr localization::WallSide mirroredSide(localization::WallSide side) noexcept {
    return side == localization::WallSide::Left ? localization::WallSide::Right : localization::WallSide::Left;
}

namespace detail {
/// |v|, usable in a constant expression (std::abs is constexpr only from C++23).
[[nodiscard]] constexpr double mirrorAbs(double v) noexcept { return v < 0.0 ? -v : v; }
}  // namespace detail

/// `wall` reflected GEOMETRICALLY by `r`: its line `axis = coord` mapped to the image line, the field
/// on the image side, every keep-out span mapped along the image wall (order, kind and name kept). A
/// wall's name and frame are kept as they are. (mirroredWall, below, is what the wall reset uses.)
[[nodiscard]] constexpr localization::FieldWall reflectedWall(const math::Reflection& r,
                                                              const localization::FieldWall& wall) noexcept {
    using localization::Axis;
    localization::FieldWall out = wall;
    if (r.isIdentity()) {
        return out;
    }
    // The wall's normal e_axis and its along direction e_other, each through the reflection: a
    // reflection with 0 / +-1 coefficients maps each unit axis onto +-(an axis).
    const math::Reflection::Point n = wall.axis == Axis::X ? r.apply(units::Length{1.0}, units::Length{0.0})
                                                           : r.apply(units::Length{0.0}, units::Length{1.0});
    const math::Reflection::Point t = wall.axis == Axis::X ? r.apply(units::Length{0.0}, units::Length{1.0})
                                                           : r.apply(units::Length{1.0}, units::Length{0.0});
    const bool normalOnX = n.x.value() != 0.0;
    const double ns = normalOnX ? n.x.value() : n.y.value();  // +1 or -1
    const double ts = normalOnX ? t.y.value() : t.x.value();  // +1 or -1
    out.axis = normalOnX ? Axis::X : Axis::Y;
    out.coord = units::Length{ns * wall.coord.value() + 0.0};
    out.inward = static_cast<std::int8_t>(ns > 0.0 ? wall.inward : -wall.inward);
    for (std::size_t i = 0; i < wall.keepOutCount && i < localization::kMaxWallKeepOuts; ++i) {
        const localization::WallKeepOut& k = wall.keepOut[i];
        out.keepOut[i].from = units::Length{(ts > 0.0 ? k.from.value() : -k.to.value()) + 0.0};
        out.keepOut[i].to = units::Length{(ts > 0.0 ? k.to.value() : -k.from.value()) + 0.0};
    }
    return out;
}

/// The index in `mirror.walls` of the season wall at `wall`'s reflected line (the same axis, the same
/// inner face within 1e-6 in, the field on the same side), or -1 when none is — an author's own line,
/// or a FIELD wall the season does not have.
[[nodiscard]] constexpr int seasonImageIndex(const FieldMirror& mirror, const localization::FieldWall& wall) noexcept {
    const localization::FieldWall img = reflectedWall(mirror.reflection, wall);
    for (std::size_t i = 0; i < mirror.walls.size(); ++i) {
        const localization::FieldWall& s = mirror.walls[i];
        if (s.axis == img.axis && s.inward == img.inward && detail::mirrorAbs(s.coord.value() - img.coord.value()) <= 1e-6) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

/// THE WALL RESET'S WALL under `mirror` (the file banner): a SEASON wall (frame Field) maps to the
/// season wall at its reflected line — with that wall's REAL name and keep-outs (Override's red wall
/// carries Loaders the right wall does not: the image is the right wall as it is, not the red wall's
/// spans moved); an author's OWN line (frame Own), or a Field wall the season lacks, is reflected
/// geometrically (reflectedWall), its name kept.
[[nodiscard]] constexpr localization::FieldWall mirroredWall(const FieldMirror& mirror,
                                                             const localization::FieldWall& wall) noexcept {
    if (wall.frame == localization::WallFrame::Field) {
        const int i = seasonImageIndex(mirror, wall);
        if (i >= 0) {
            return mirror.walls[static_cast<std::size_t>(i)];
        }
    }
    return reflectedWall(mirror.reflection, wall);
}

/// `entry` MIRRORED by `mirror` (the file banner) — a copy that robot two's autons file registers:
/// `registry.add(competition::mirrored(skillsRoute(), override2026::kSkillsMirror));`. It TOGGLES: an
/// entry already mirrored by the same reflection comes back as written. `mirror` is BORROWED (a season
/// constant with static storage). PRECONDITIONS (a programming error, thrown): `mirror` reflects (not
/// the identity) and has a name; the entry is not already mirrored by a DIFFERENT reflection (D5).
/// Nothing else is checked here: the registry's own check refuses a mirrored name over
/// kMaxMirroredAutonNameChars when the entry is registered. The ROBOT is not mirrored — a part off its
/// centre line meets the field on the other side of the reflected path (A4 register HA-259).
[[nodiscard]] inline AutonEntry mirrored(const AutonEntry& entry, const FieldMirror& mirror) {
    SHULIB_PRECONDITION(!mirror.reflection.isIdentity() && mirror.name != nullptr && mirror.name[0] != '\0',
                        "competition::mirrored: the FieldMirror must reflect and have a name -- use a season "
                        "constant (override2026::kSkillsMirror)");
    AutonEntry out = entry;
    if (entry.mirror == nullptr) {
        out.mirror = &mirror;
        return out;
    }
    SHULIB_PRECONDITION(entry.mirror->reflection == mirror.reflection,
                        "competition::mirrored: this auton is already mirrored by a DIFFERENT mirror -- mirror "
                        "it once (mirroring twice by the same mirror gives the route back as written)");
    out.mirror = nullptr;  // twice by the same mirror: the route as written
    return out;
}

/// REFUSED AT COMPILE TIME: mirrored() handed a TEMPORARY FieldMirror. The entry keeps the mirror's
/// ADDRESS for its whole life (the registry holds it; the runner, the wall reset and the Toggle step
/// read it at run time), and a temporary is gone by the end of the line — a dangling pointer no
/// warning catches. Hand it the season constant, which has static storage:
/// `mirrored(skillsRoute(), override2026::kSkillsMirror)`. (Deleting the const rvalue form catches a
/// non-const temporary too: it binds there before it would bind to the const reference.)
AutonEntry mirrored(const AutonEntry& entry, const FieldMirror&& mirror) = delete;

/// True while `ctx` runs a MIRRORED entry — robot two's run of the shared route. The shared route's
/// way to send ONE robot to what lies on the field's mirror line (the file banner; the brief's D11):
/// `if (!competition::isMirrored(ctx)) { /* robot one only */ }`.
[[nodiscard]] inline bool isMirrored(const AutonContext& ctx) noexcept { return ctx.entry.mirror != nullptr; }

/// The reflection a run of `entry` uses (the file banner's table): the identity for an entry as
/// written; for a mirrored one, its FieldMirror's reflection under a FIELD start and the ROBOT-frame
/// mirror (math::Reflection::Line::XAxis) under a RELATIVE one. An UNSET start (refused at
/// registration) answers the identity.
[[nodiscard]] inline math::Reflection runReflection(const AutonEntry& entry) {
    if (entry.mirror == nullptr) {
        return math::Reflection{};
    }
    switch (entry.effectiveStart().mode()) {
        case StartPose::Mode::Field: return entry.mirror->reflection;
        case StartPose::Mode::Relative: return math::Reflection::across(math::Reflection::Line::XAxis);
        case StartPose::Mode::Unset: break;
    }
    return math::Reflection{};
}

/// Where a run of `entry` SEATS the robot — the entry's effective start, reflected by runReflection():
/// robot two's REAL start for a mirrored entry (where the drivers must place it), the start as written
/// otherwise.
[[nodiscard]] inline math::Pose2d seatedStart(const AutonEntry& entry) {
    const math::Pose2d asWritten = entry.effectiveStart().pose();
    const math::Reflection r = runReflection(entry);
    return r.isIdentity() ? asWritten : r.apply(asWritten);
}

}  // namespace shulib::competition
