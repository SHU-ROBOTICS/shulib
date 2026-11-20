#pragma once
//
// override_field_obstacles.hpp — the Override field's SOLID structures on the floor, for the route builder (chunk
// RMP1-H, 2026-11-19). Host only: route_maker_describe.hpp writes them into robots.json's `field.obstacles`, and the
// page checks every step's swept square against them. Nothing a robot program compiles.
//
// THE SOURCE: the VEX V5RC Override Game Manual, Appendix A — v2.0 (2026-09-03; its PDF pages A7 "Goal
// Specifications", A9 "Loader Specifications", A10 "Field Element Locations") and v2.1 (the online manual, read
// 2026-11-19: the same three drawings, released 2026-04-27, every number below the same). Each number is the
// drawing's millimetres. The drawings' audience view has (0, 0) at the bottom-left inner corner and the red station
// on its LEFT; F1 (localization/override_walls.hpp) is that view turned: x = 70.205 - a_y, y = a_x - 70.205 (inches).
//
// SOLID = standing on the floor, fixed, and a robot cannot drive through it: the nine Goals (the cones are placed ON
// them) and the four Loaders. NOT here: the Pins and Cups (game pieces: they move), the Toggles (on the wall's top,
// their brackets 10.6 in up: override_walls.hpp), the tape (flat).

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "shulib/localization/override_walls.hpp"

namespace route_maker::obstacles {

/// Millimetres to inches.
inline constexpr double kPerMm = 1.0 / 25.4;
/// The field's centre on A10's axes: 1783.2 mm.
inline constexpr double kCentreMm = 1783.2;
/// A goal's centre, the near offset from a centre line: A10 2381.3 - 1783.2 mm (23.547 in).
inline constexpr double kGoalNear = (2381.3 - kCentreMm) * kPerMm;
/// A goal's centre, the far offset: A10 2979.3 - 1783.2 mm (47.091 in).
inline constexpr double kGoalFar = (2979.3 - kCentreMm) * kPerMm;
/// Half a goal's base, flat to flat: A7 142.5 mm (5.61 in), dimensioned on the red goal; the black (5.77 in tall) and
/// the centre (8.77 in tall) goals' bases are drawn the same width on that sheet (332 and 330 px to the red's 332, v2.1).
inline constexpr double kGoalHalfWidth = 142.5 / 2.0 * kPerMm;
/// How far a robot's front may reach over a red or blue goal's base: above its 10 mm lip the body narrows from the
/// base's 142.5 mm to 88.8 mm at its 82.5 mm top (A7), so (142.5 - 88.8) / 2 mm (1.057 in). The black and the centre
/// goals stand full width up to 64 mm and 140.2 mm (A7), the Loaders straight (A9): nothing to reach over (0).
inline constexpr double kColourGoalOverhang = (142.5 - 88.8) / 2.0 * kPerMm;
/// A goal base's corners, from its centre: A7 R81.9 mm (3.22 in) — "Goals are octagonal" (the glossary).
inline constexpr double kGoalCornerRadius = 81.9 * kPerMm;
/// The Loaders' centres along the red and blue walls: A10 3275.8 - 1783.2 mm (58.764 in).
inline constexpr double kLoaderAlong = (3275.8 - kCentreMm) * kPerMm;
/// A Loader's width along its wall: A9 102.1 mm (4.02 in).
inline constexpr double kLoaderWidth = 102.1 * kPerMm;
/// How far a Loader stands into the field from the wall's inner face: A9 and A10 95 mm (3.74 in).
inline constexpr double kLoaderDepth = 95.0 * kPerMm;

/// The walls' inner face (the page's walls: override_walls.hpp, A10's 1783.2 mm) — where a Loader stands.
inline constexpr double kFace = shulib::localization::override2026::kWallInnerFace.value();

/// What a structure is.
enum class Kind { Goal, Loader };

/// One solid structure: a goal's centre, or the middle of a Loader's face on its wall with the way into the field.
struct Obstacle {
    const char* id;    ///< stable, for a step's "touches this on purpose"
    const char* name;  ///< the page's words: "the <name>"
    Kind kind;
    double x;          ///< F1, inches
    double y;          ///< F1, inches
    double inY;        ///< a Loader: +1 on the red wall, -1 on the blue (into the field); a goal: 0
    double overhang;   ///< inches a robot's front may reach over the footprint, touching it on purpose (above)
};

/// Where each element's numbers come from (the description carries it beside the element).
[[nodiscard]] constexpr const char* sourceOf(Kind k) noexcept {
    return k == Kind::Goal ? "Game Manual Appendix A (v2.0 = v2.1): Field Element Locations (A10), Goal Specifications (A7)"
                           : "Game Manual Appendix A (v2.0 = v2.1): Field Element Locations (A10), Loader Specifications (A9)";
}

/// The thirteen, in F1. Goals named by colour and their nearest wall; Loaders by wall and side (as seen from red).
inline constexpr std::array<Obstacle, 13> kObstacles{{
    {"centre-goal", "centre goal", Kind::Goal, 0.0, 0.0, 0.0, 0.0},                                  // A10 (70.20, 70.20) black, tall
    {"red-goal-right-wall", "red goal by the right wall", Kind::Goal, kGoalFar, -kGoalNear, 0.0, kColourGoalOverhang},   // A10 (46.66, 23.11)
    {"red-goal-red-wall", "red goal by the red wall", Kind::Goal, kGoalNear, -kGoalFar, 0.0, kColourGoalOverhang},       // A10 (23.11, 46.66)
    {"blue-goal-left-wall", "blue goal by the left wall", Kind::Goal, -kGoalFar, kGoalNear, 0.0, kColourGoalOverhang},   // A10 (93.75, 117.30)
    {"blue-goal-blue-wall", "blue goal by the blue wall", Kind::Goal, -kGoalNear, kGoalFar, 0.0, kColourGoalOverhang},   // A10 (117.30, 93.75)
    {"black-goal-left-wall", "black goal by the left wall", Kind::Goal, -kGoalFar, -kGoalNear, 0.0, 0.0},  // A10 (46.66, 117.30)
    {"black-goal-red-wall", "black goal by the red wall", Kind::Goal, -kGoalNear, -kGoalFar, 0.0, 0.0},    // A10 (23.11, 93.75)
    {"black-goal-blue-wall", "black goal by the blue wall", Kind::Goal, kGoalNear, kGoalFar, 0.0, 0.0},    // A10 (117.30, 46.66)
    {"black-goal-right-wall", "black goal by the right wall", Kind::Goal, kGoalFar, kGoalNear, 0.0, 0.0},  // A10 (93.75, 23.11)
    {"loader-red-wall-left", "left Loader on the red wall", Kind::Loader, -kLoaderAlong, -kFace, 1.0, 0.0},   // A10 (0, 128.97)
    {"loader-red-wall-right", "right Loader on the red wall", Kind::Loader, kLoaderAlong, -kFace, 1.0, 0.0},  // A10 (0, 11.44)
    {"loader-blue-wall-left", "left Loader on the blue wall", Kind::Loader, -kLoaderAlong, kFace, -1.0, 0.0},  // A10 (140.41, 128.97)
    {"loader-blue-wall-right", "right Loader on the blue wall", Kind::Loader, kLoaderAlong, kFace, -1.0, 0.0}, // A10 (140.41, 11.44)
}};

/// A number on the description's grid (0.001 in).
[[nodiscard]] inline double grid(double v) {
    const double r = std::round(v * 1000.0) / 1000.0;
    return r == 0.0 ? 0.0 : r;  // never -0
}

/// The footprint, counterclockwise, on the 0.001 in grid: a goal's base is its square of flats cut by the R3.22 corner
/// circle (each corner arc four chords); a Loader's, its rectangle from the wall's face into the field.
[[nodiscard]] inline std::vector<std::array<double, 2>> footprint(const Obstacle& o) {
    std::vector<std::array<double, 2>> out;
    if (o.kind == Kind::Loader) {
        const double h = kLoaderWidth / 2.0;
        const double far = o.y + o.inY * kLoaderDepth;
        const double lo = std::fmin(o.y, far);
        const double hi = std::fmax(o.y, far);
        const std::array<std::array<double, 2>, 4> box{{{o.x - h, lo}, {o.x + h, lo}, {o.x + h, hi}, {o.x - h, hi}}};
        for (const auto& p : box) {
            out.push_back({grid(p[0]), grid(p[1])});
        }
        return out;
    }
    const double hw = kGoalHalfWidth;
    const double r = kGoalCornerRadius;
    const double flat = std::sqrt(r * r - hw * hw);  // half a flat's length
    const double a0 = std::atan2(flat, hw);
    const double quarter = std::acos(-1.0) / 2.0;
    for (int side = 0; side < 4; ++side) {
        const double phi = quarter * side;
        const double c = std::cos(phi);
        const double s = std::sin(phi);
        const auto at = [&](double lx, double ly) { out.push_back({grid(o.x + lx * c - ly * s), grid(o.y + lx * s + ly * c)}); };
        at(hw, -flat);
        at(hw, flat);
        for (int j = 1; j <= 3; ++j) {
            const double t = a0 + (quarter - 2.0 * a0) * j / 4.0;
            at(r * std::cos(t), r * std::sin(t));
        }
    }
    return out;
}

}  // namespace route_maker::obstacles
