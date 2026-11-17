#pragma once
//
// The 2026-27 season's SKILLS MIRROR — VEX V5RC / VEX U "Override" (chunk MIRROR1-A, 2026-11-13; the
// MIRROR1 brief's D1, its §0.3 and its probe P3). Season data: next season adds its own header, and
// this one stays as the record (VIS1's and L6's rule: season data in a season header, cited).
//
// ── Which mirror, from the game manual (v2.0 read; v2.1's <SG1> and <VURS1>-<VURS3> re-read 2026-11-13) ─
//   * In a VEX U skills run BOTH robots start "in legal starting positions for the red Alliance"
//     (<VURS3>), and "All other portions of rule <SG1> apply": each robot not sharing a Quadrant with
//     another robot (<SG1>d) and on its alliance's side of the Autonomous Line (<SG1>f). The
//     Autonomous Line is the double diagonal tape, along F1's y = x; red's side holds exactly two
//     Quadrants — the RED wall's and the RIGHT wall's (as seen from the red station). So robot one
//     starts in one and robot two in the other.
//   * Appendix A's field drawing (A10) makes those two Quadrants mirror images across the diagonal from
//     the corner where the red wall meets the right wall to the corner where the blue wall meets the
//     left wall — F1's line y = -x: (x, y, heading) -> (-y, -x, -90 deg - heading). Under it all 9 goals
//     land on a goal of the same colour, all 4 Toggles on a Toggle of the same Quadrant colour (red wall
//     <-> right wall, left wall <-> blue wall), red's side stays red, and the skills setup's Pins
//     (Figure VURS1) land on Pins of the same colours. Every other candidate fails on the manual's own
//     numbers (the brief's probe P3): the left/right mirror across x = 0 keeps 1 of 9 goal colours, the
//     red/blue mirror across y = 0 keeps 1 of 9, the half turn keeps 5 of 9 and turns red's side blue
//     (it is the HEAD-TO-HEAD red <-> blue swap — not this), and the other diagonal turns red's side
//     blue.
//
// ── What it does NOT carry (manual facts — told, never refused: the library cannot see intent) ────
//   * The LOADERS: all four sit on the red and blue walls (localization/override_walls.hpp); their
//     images land on the right and left walls, where there are none (0 of 4). A step at a Loader has no
//     mirror — Match Loads go only through red Loaders (<RSC2>d).
//   * The LOAD ZONES' colours: the red/left corner's red zone lands on the blue/right corner's blue
//     zone (2 of 4 keep their colour). A park in that zone does not mirror.
//   * AprilTag NUMBERS: tags 1 <-> 4 and 2 <-> 3 swap (only the centre's 0 keeps its number).
//   * ON THE LINE ITSELF — the centre goal (0, 0), two yellow Pins of the skills setup (about (23.5,
//     -23.5) and (47, -47) in, Figure VURS1 read by pixel), and the red/right corner's Load Zone with the
//     red Loader at (58.76, -70.2) beside it — each is its own image: robot two reaches it at the same
//     moment as robot one. ONE robot only (competition::isMirrored(ctx), field_mirror.hpp).
//   * Field elements may sit up to +-1.0 in off nominal (<T5>a): a mirrored target can land up to ~2 in
//     from robot two's element where robot one's was tuned to its own. A4 register HA-258.
//
// PROS-free, allocation-free, constexpr data. NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>

#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/math/reflection.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/override_elements.hpp"

namespace shulib::competition::override2026 {

/// Override's SKILLS mirror (the file banner): the reflection across F1's y = -x — the diagonal from
/// the red/right corner to the blue/left corner — that maps the red wall's Quadrant onto the right
/// wall's. Robot two's autons file: `registry.add(competition::mirrored(skillsRoute(), kSkillsMirror));`.
/// A4 register HA-258 (the event's field set up to Appendix A and Figure VURS1).
inline constexpr FieldMirror kSkillsMirror{
    .name = "the skills diagonal",
    .reflection = math::Reflection::across(math::Reflection::Line::AntiDiagonal),
    .walls = localization::override2026::kWalls,
};

/// The Toggle at the image of `id`'s Toggle under `r` — found GEOMETRICALLY from the season's places
/// (vision::override2026::kTogglePlaces), so it cannot be typed wrong; Unset for an Unset id or an image
/// that is no Toggle's place. The identity returns `id`.
[[nodiscard]] constexpr vision::override2026::ToggleId toggleImage(const math::Reflection& r,
                                                                   vision::override2026::ToggleId id) noexcept {
    namespace vo = vision::override2026;
    const int i = vo::toggleIndex(id);
    if (i < 0) {
        return vo::ToggleId::Unset;
    }
    if (r.isIdentity()) {
        return id;
    }
    const vo::TogglePlace& p = vo::kTogglePlaces[i];
    const math::Reflection::Point q = r.apply(units::Length{p.x}, units::Length{p.y});
    for (int j = 0; j < 4; ++j) {
        const vo::TogglePlace& c = vo::kTogglePlaces[j];
        if (detail::mirrorAbs(c.x - q.x.value()) <= 1e-6 && detail::mirrorAbs(c.y - q.y.value()) <= 1e-6) {
            return static_cast<vo::ToggleId>(j + 1);
        }
    }
    return vo::ToggleId::Unset;
}

/// THE TOGGLE STEP'S TOGGLE for `entry` (competition/toggle_visit.hpp): `id` as written, or — for a
/// mirrored entry — its image under the entry's FieldMirror (the Toggle is a FIELD fact, whatever the
/// start mode: robot two placed at the image of robot one's spot faces the image Toggle). Its Quadrant
/// colour is the same (the static_asserts below), so "ours" does not change.
[[nodiscard]] inline vision::override2026::ToggleId mirroredToggle(const AutonEntry& entry,
                                                                   vision::override2026::ToggleId id) noexcept {
    return entry.mirror == nullptr ? id : toggleImage(entry.mirror->reflection, id);
}

// ── The season's data closes under the mirror — checked where it is written ──────────────────────────
// kWalls is left, right, red, blue (override_walls.hpp): the red wall's image is the right wall and the
// left wall's the blue wall, and back.
static_assert(seasonImageIndex(kSkillsMirror, localization::override2026::kLeftWall) == 3,
              "the skills mirror maps the LEFT wall onto the BLUE wall");
static_assert(seasonImageIndex(kSkillsMirror, localization::override2026::kRightWall) == 2,
              "the skills mirror maps the RIGHT wall onto the RED wall");
static_assert(seasonImageIndex(kSkillsMirror, localization::override2026::kRedWall) == 1,
              "the skills mirror maps the RED wall onto the RIGHT wall");
static_assert(seasonImageIndex(kSkillsMirror, localization::override2026::kBlueWall) == 0,
              "the skills mirror maps the BLUE wall onto the LEFT wall");
// Each Toggle onto a Toggle of the SAME Quadrant colour (AF1S-A1's table, HA-243): right <-> red wall
// (RED), left <-> blue wall (BLUE).
static_assert(toggleImage(kSkillsMirror.reflection, vision::override2026::ToggleId::RightWall)
                  == vision::override2026::ToggleId::RedWall
              && toggleImage(kSkillsMirror.reflection, vision::override2026::ToggleId::RedWall)
                     == vision::override2026::ToggleId::RightWall
              && toggleImage(kSkillsMirror.reflection, vision::override2026::ToggleId::LeftWall)
                     == vision::override2026::ToggleId::BlueWall
              && toggleImage(kSkillsMirror.reflection, vision::override2026::ToggleId::BlueWall)
                     == vision::override2026::ToggleId::LeftWall,
              "the skills mirror maps right <-> red wall and left <-> blue wall Toggles");
static_assert(vision::override2026::quadrantColour(
                  toggleImage(kSkillsMirror.reflection, vision::override2026::ToggleId::RightWall))
                  == vision::override2026::quadrantColour(vision::override2026::ToggleId::RightWall)
              && vision::override2026::quadrantColour(
                     toggleImage(kSkillsMirror.reflection, vision::override2026::ToggleId::RedWall))
                     == vision::override2026::quadrantColour(vision::override2026::ToggleId::RedWall)
              && vision::override2026::quadrantColour(
                     toggleImage(kSkillsMirror.reflection, vision::override2026::ToggleId::LeftWall))
                     == vision::override2026::quadrantColour(vision::override2026::ToggleId::LeftWall)
              && vision::override2026::quadrantColour(
                     toggleImage(kSkillsMirror.reflection, vision::override2026::ToggleId::BlueWall))
                     == vision::override2026::quadrantColour(vision::override2026::ToggleId::BlueWall),
              "every Toggle's image under the skills mirror keeps its Quadrant colour (HA-243)");

}  // namespace shulib::competition::override2026
