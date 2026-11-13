#pragma once
//
// The 2026-27 Override season's facts the camera code needs — the Toggle's coloured faces, where the
// Toggles are, the Pin and the Cup, and the class table of VEX's Override AI model (chunk VIS1-A1,
// 2026-10-22). DATA ONLY: which Toggle to flip, which piece to take and when are the team's (the
// competition code — the project's guardrail); nothing here decides anything. One header per season:
// next season adds its own, and this one stays as the record.
//
// THE SOURCE: the VEX V5RC Override Game Manual v2.0 (2026-09-03) — its Appendix A drawings (pA8 "Toggle
// Specifications", pA10 "Field Element Locations", pA13 / pA14 the perimeter walls, pA15 the Toggles'
// start orientation) and its glossary (Pin, Cup, Toggle). v2.1 was scheduled to take effect 2026-10-15;
// its PDF answered HTTP 404 on 2026-10-15, 2026-10-18 and 2026-10-22, so v2.0 is what was read. A manual
// change to any number here is a one-header change with its citation — never a silent edit.
//
// Every number is cited. Where a number is ARITHMETIC from a drawing rather than a stated figure, it
// says so and names its register entry: the Toggle's face geometry is A4 register HA-201 (reasoned).
//
// ── The Toggle's cross-section, read from pA8's end view ────────────────────────────
// A rounded triangle on an axle 13.20 in above the floor: 51.6 mm (2.03 in) from the apex to the FLAT
// bottom face, the axle 30.2 mm (1.19 in) below the apex, the corners heavily rounded; three coloured
// faces (pA15). By the shape's 120-degree symmetry each corner's middle — where one colour meets the next
// — lies 30.2 mm from the axle, so two colour boundaries are 30.2 x sqrt 3 = 52.3 mm apart: the
// glossary's "each face of the triangle is approximately 2.05 in (52.2 mm) wide". So the field-facing
// face runs from the apex (13.20 + 1.19 = 14.39 in up, over the axle) down to its lower boundary 15.1 mm
// (0.594 in) below the axle — 12.61 in up — and 26.2 mm (1.03 in) toward the field: it LEANS BACK. The
// FLAT bottom face is lower still, 21.4 mm (0.84 in) below the axle — 12.36 in up — joined to the field
// face's lower boundary by the rounded corner. (VIS1's brief, before its re-check, read 12.36 as the
// field face's lower edge; the re-check took 12.61 from the glossary; the drawing, read at VIS1-A1,
// shows both hold — 12.36 is the bottom face, not the field face's edge.)

#include <cstdint>

namespace shulib::vision::override2026 {

/// The Toggle's FIELD-FACING face — the one that counts "when viewed from inside the Field" (the
/// glossary) — as a leaning band, and the flat bottom face under it (header). A4 register HA-201.
struct ToggleFace {
    /// The face's lower colour boundary above the floor, LOW end, inches: the arithmetic's 12.61, less
    /// the drawing's tolerance and the boundary's spread over the rounded corner (HA-201).
    double bottomIn = 12.55;
    /// The face's lower colour boundary above the floor, HIGH end, inches (HA-201).
    double bottomHiIn = 12.65;
    /// The apex's horizontal set-back behind the lower boundary, at least, inches (the arithmetic's 1.03,
    /// less the corner's spread — HA-201).
    double setBackLoIn = 0.9;
    /// The apex's horizontal set-back behind the lower boundary, at most, inches (HA-201).
    double setBackHiIn = 1.2;
    /// The face's upper edge (the apex) above the floor, inches: the axle's 13.20 + 30.2 mm (pA8;
    /// arithmetic, HA-201).
    double topIn = 14.39;
    /// The FLAT bottom face's height above the floor, inches: the apex's 14.39 less pA8's 2.03 — about a
    /// quarter inch BELOW the field face's lower boundary (header; HA-201).
    double flatBottomIn = 12.36;
    /// The Toggle's length, inches: the glossary's "25.8 in (656.2 mm) long" (pA8's 25.99 includes the
    /// end caps).
    double lengthIn = 25.8;
    /// The perimeter wall's top above the floor, inches: 11.50 (the metal wall, pA14; the portable wall
    /// is 11.54, pA13).
    double wallTopIn = 11.5;
};
/// The Toggle's face geometry, as the manual and the arithmetic above give it (HA-201).
inline constexpr ToggleFace kToggleFace{};

/// Where one Toggle is: the wall's inner face at its centre, in F1's field frame (origin at the field's
/// centre), and the unit vector INTO the field. How far the face sits from the wall's inner face
/// horizontally is UNVERIFIED (HA-201) — a consumer's position gate allows for it.
struct TogglePlace {
    double x = 0.0;    ///< the wall's inner face at its centre: field x, inches
    double y = 0.0;    ///< the wall's inner face at its centre: field y, inches
    double inX = 0.0;  ///< the unit vector into the field: x
    double inY = 0.0;  ///< the unit vector into the field: y
};
/// The four Toggles, one centred on each wall: pA10 puts each on its wall's centreline, 70.20 in from the
/// field's centre (the field 140.41 in across).
inline constexpr TogglePlace kTogglePlaces[4] = {
    {70.2, 0.0, -1.0, 0.0}, {-70.2, 0.0, 1.0, 0.0}, {0.0, 70.2, 0.0, -1.0}, {0.0, -70.2, 0.0, 1.0}};

// ── Which Toggle, and its Quadrant's colour (chunk AF1S-A1, 2026-11-11) ──────────────
// In a SKILLS run a Toggle helps only when it shows ITS OWN QUADRANT's colour: v2.0 <RSC3>a-iii-1
// ("Toggles only confer Ownership of Pins when the Toggle matches the color of its Quadrant") — the
// v2.1 change log (read 2026-11-10) re-words the rule for "Pin Half" and keeps it in substance. Which
// Quadrant each Toggle belongs to is read from v2.0 Figure Q-1 (Red 1 against the red alliance's wall,
// Red 2 against the wall on the red station's RIGHT; Blue 1 against the blue alliance's wall, Blue 2
// against the red station's LEFT) with pA15's start orientation (each Toggle yellow into the field,
// its Quadrant's colour facing OUT). In F1 (+x to the right as seen from the red station, the red wall
// at y = -70.2): RIGHT wall RED, LEFT wall BLUE, BLUE wall BLUE, RED wall RED. A READING of two drawings,
// not a measurement: A4 register HA-243 (reasoned), settled on the event's field. DATA ONLY — its own
// small colour type, not vision::ToggleColour (that lives in toggle_tracker.hpp, which includes this
// header: a circular include otherwise); competition/toggle_visit.hpp maps one to the other.

/// WHICH Toggle — in kTogglePlaces' order (index = the id less one). Unset (0) names none.
enum class ToggleId : std::uint8_t {
    Unset = 0,  ///< no Toggle named (refused by every consumer)
    RightWall,  ///< the right wall's (x = +70.2) — kTogglePlaces[0]
    LeftWall,   ///< the left wall's (x = -70.2) — kTogglePlaces[1]
    BlueWall,   ///< the blue alliance's wall's (y = +70.2) — kTogglePlaces[2]
    RedWall,    ///< the red alliance's wall's (y = -70.2) — kTogglePlaces[3]
};

/// The named handles a route writes (`toggles::kRedWall`).
namespace toggles {
/// The right wall's Toggle (x = +70.2) — a RED Quadrant's (HA-243).
inline constexpr ToggleId kRightWall = ToggleId::RightWall;
/// The left wall's Toggle (x = -70.2) — a BLUE Quadrant's (HA-243).
inline constexpr ToggleId kLeftWall = ToggleId::LeftWall;
/// The blue alliance's wall's Toggle (y = +70.2) — a BLUE Quadrant's (HA-243).
inline constexpr ToggleId kBlueWall = ToggleId::BlueWall;
/// The red alliance's wall's Toggle (y = -70.2) — a RED Quadrant's (HA-243).
inline constexpr ToggleId kRedWall = ToggleId::RedWall;
}  // namespace toggles

/// A Quadrant's colour — this header's own (header: the circular include).
enum class QuadrantColour : std::uint8_t {
    Unset = 0,  ///< no colour (an Unset Toggle's)
    Red,        ///< a red Quadrant
    Blue,       ///< a blue Quadrant
};

/// Each Toggle's Quadrant colour, in kTogglePlaces' order: right RED, left BLUE, blue wall BLUE, red
/// wall RED (header; A4 register HA-243 — reasoned from Figure Q-1 and pA15, unmeasured).
inline constexpr QuadrantColour kToggleQuadrantColour[4] = {QuadrantColour::Red, QuadrantColour::Blue,
                                                           QuadrantColour::Blue, QuadrantColour::Red};

/// The index of `id` in kTogglePlaces (0..3), or -1 for Unset or a byte outside the enum.
[[nodiscard]] constexpr int toggleIndex(ToggleId id) noexcept {
    const int i = static_cast<int>(id) - 1;
    return i >= 0 && i < 4 ? i : -1;
}

/// The Quadrant colour of `id`'s Toggle — Unset for Unset or a byte outside the enum.
[[nodiscard]] constexpr QuadrantColour quadrantColour(ToggleId id) noexcept {
    const int i = toggleIndex(id);
    return i < 0 ? QuadrantColour::Unset : kToggleQuadrantColour[i];
}

/// The Toggle's name for a log line: "right wall Toggle", "left wall Toggle", "blue wall Toggle", "red
/// wall Toggle"; "unnamed Toggle" for Unset or a byte outside the enum.
[[nodiscard]] constexpr const char* toggleName(ToggleId id) noexcept {
    switch (id) {
        case ToggleId::RightWall: return "right wall Toggle";
        case ToggleId::LeftWall: return "left wall Toggle";
        case ToggleId::BlueWall: return "blue wall Toggle";
        case ToggleId::RedWall: return "red wall Toggle";
        case ToggleId::Unset: break;
    }
    return "unnamed Toggle";
}

/// A Scoring Object's size (the glossary).
struct PieceSize {
    double diameterIn = 0.0;  ///< across, inches
    double heightIn = 0.0;    ///< standing, inches
};
/// A Pin: "approximately 1.6 in (40 mm) in diameter and 6.5 in (165 mm) tall" (the glossary).
inline constexpr PieceSize kPinSize{1.6, 6.5};
/// A Cup: "approximately 3.15 in (80 mm) in diameter and 6.5 in (164.5 mm) tall" (the glossary).
inline constexpr PieceSize kCupSize{3.15, 6.5};

/// The Override AI model's classes, as api.vex.com lists them for VEXcode (read 2026-10-15: "Override:
/// 0 redBluePin, 1 blueYellowPin, 2 redYellowPin, 3 yellowPin, 4 cup"). Whether a PROS program sees
/// these names for these ids is A4 register HA-197 — the adapter checks them BY NAME before an id means
/// anything (hal::ModelCheck).
enum class ModelClass : std::uint8_t {
    RedBluePin = 0,     ///< a Pin, half red and half blue
    BlueYellowPin = 1,  ///< a Pin, half blue and half yellow
    RedYellowPin = 2,   ///< a Pin, half red and half yellow
    YellowPin = 3,      ///< a Pin, all yellow
    Cup = 4,            ///< a Cup
};
/// How many classes the table names.
inline constexpr std::uint8_t kModelClassCount = 5;

/// The table a robot file hands the adapter (`hal::AiVisionSettings::expectedClasses`): the camera must
/// name exactly these, in this order, before an Object id is believed (A4 register HA-197). VEX's
/// spelling; the camera's own is read at the bench — a different one fails SAFE (every object withheld,
/// the boot line naming the first name that differed).
inline constexpr const char* kOverrideClassNames[kModelClassCount] = {
    "redBluePin", "blueYellowPin", "redYellowPin", "yellowPin", "cup"};

/// The class's name as VEX spells it (`kOverrideClassNames[id]`), or nullptr for an id outside the table.
[[nodiscard]] constexpr const char* modelClassName(std::uint8_t id) noexcept {
    return id < kModelClassCount ? kOverrideClassNames[id] : nullptr;
}

/// The class's physical size — a Pin's for ids 0..3, the Cup's for 4 — or nullptr for an id outside the
/// table.
[[nodiscard]] constexpr const PieceSize* modelClassSize(std::uint8_t id) noexcept {
    if (id < static_cast<std::uint8_t>(ModelClass::Cup)) {
        return &kPinSize;
    }
    return id == static_cast<std::uint8_t>(ModelClass::Cup) ? &kCupSize : nullptr;
}

}  // namespace shulib::vision::override2026
