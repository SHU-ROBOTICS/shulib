#pragma once
//
// TagMap — where the AprilTags are on the field, and where each of those numbers CAME FROM
// (master plan §8; WS5, chunk E3, tension T2).
//
// ── WHY THIS TYPE EXISTS AT ALL ─────────────────────────────────────────────────────────────
// A tag observation says "there is a tag with id 7 at relative pose P". That is useless on its
// own. It becomes an absolute robot pose only against knowledge of where tag 7 IS — and that
// knowledge is INPUT, not something the library can derive, measure, or reasonably guess.
//
// It is also the single most dangerous input in the whole localization stack, for a reason worth
// stating in full: sensor noise averages out and a fusion filter is built to absorb it, but a
// WRONG TAG POSE DOES NOT AVERAGE OUT. A map entry two inches off produces a corrector that is
// confidently two inches wrong, every time it sees that tag, with a small residual and a high
// confidence — i.e. it looks exactly like a healthy fix. It will also fight the GPS corrector,
// and the fusion policy has no way to tell which of the two is lying. There is no gate width
// that fixes this and no amount of filtering that reveals it.
//
// ── WHAT SHULIB DOES NOT SHIP, AND WHY ──────────────────────────────────────────────────────
// **There is NO built-in VEX field tag map in this library, and adding one would be a mistake
// until somebody can cite a published table.** Nobody on this project has a game-manual table of
// AprilTag field poses in hand; a plausible-looking default map would be invented geometry
// wearing the clothes of a specification, and every team that forgot to override it would be
// silently localizing against fiction. So the map is empty until a caller fills it, an empty map
// makes the corrector decline with `RejectedNoTagMapEntry`, and that is a loud, diagnosable
// state rather than a quiet wrong one. Obtaining the real layout is R3's job (A4: HA-68).
//
// ── PROVENANCE IS MANDATORY, BY CONSTRUCTION ────────────────────────────────────────────────
// `add()` REFUSES an entry that does not say where its numbers came from. Not a convention, not
// a lint: a precondition. `TagProvenance::Invented` is a completely legitimate answer — a guess
// LABELED as a guess is exactly what the A4 register exists to protect (see its own preamble) —
// but "I did not say" is not, because the difference between a specified pose and an invented
// one is invisible in the arithmetic and total in the consequences.
//
// `anyInvented()` exists so a run can be honest about it in telemetry: an estimator anchored to
// made-up field geometry should not read the same as one anchored to a measured field.
//
// ── THE FRAME ───────────────────────────────────────────────────────────────────────────────
// `fieldPose` is in the canonical field frame (F1: +X, +Y, CCW-positive), and its HEADING is the
// direction the tag's OUTWARD NORMAL points — the direction it "faces", i.e. toward a robot
// looking at it. A tag flat on the wall at x = 70 that a robot at the origin can read is facing
// 180 degrees. This matches hal/vision_conversion.hpp's reduction exactly, on purpose: that
// function reports `poseInRobot.heading()` with the same meaning, so the two compose without a
// convention change in between. (A convention change in between is how the sign errors get in.)
//
// Fixed capacity, no allocation, no clock, no HAL: pure data plus one rigid-body inversion.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// Where a tag's field pose came from. There is no default: see the header note.
enum class TagProvenance : std::uint8_t {
    Unspecified = 0,  ///< NOT ACCEPTED by add() — the whole point of the type
    Specified = 1,    ///< from a published field/game specification; `source` cites it
    Measured = 2,     ///< measured on the actual competition field; `source` says how
    Invented = 3,     ///< a number somebody made up; `source` says why. Legitimate, if labeled.
};

/// One tag's placement on the field, with its provenance attached inseparably.
struct TagPlacement {
    int id = -1;
    /// Field pose; heading = the direction the tag's outward normal points (header note).
    math::Pose2d fieldPose{};
    TagProvenance provenance = TagProvenance::Unspecified;
    /// The citation, the measurement method, or the reason this is a guess. Must be non-empty.
    /// A static string literal: this type stores the pointer, it does not own the text.
    const char* source = nullptr;
};

class TagMap {
public:
    /// Enough for a VEX field's worth of tags with room to spare. Fixed so the lookup on the
    /// control path never allocates and never rehashes.
    static constexpr std::size_t kMaxTags = 16;

    /// Register a tag. Refuses, loudly and at setup time (never mid-match), an entry with no
    /// provenance, no source text, a negative id, a non-finite pose, or a duplicate id — a
    /// duplicate is the mistake most likely to survive review, because the second entry simply
    /// never wins a lookup and the map still "works".
    void add(const TagPlacement& placement) {
        SHULIB_PRECONDITION(placement.id >= 0, "TagMap: tag id must be >= 0");
        SHULIB_PRECONDITION(placement.provenance != TagProvenance::Unspecified,
                            "TagMap: a tag pose must say where it came from (TagProvenance)");
        SHULIB_PRECONDITION(placement.source != nullptr && placement.source[0] != '\0',
                            "TagMap: a tag pose must carry a non-empty source citation");
        SHULIB_PRECONDITION(std::isfinite(placement.fieldPose.x().value()) &&
                                std::isfinite(placement.fieldPose.y().value()),
                            "TagMap: tag field pose must be finite");
        SHULIB_PRECONDITION(find(placement.id) == nullptr, "TagMap: duplicate tag id");
        SHULIB_PRECONDITION(count_ < kMaxTags, "TagMap: too many tags");
        entries_[count_++] = placement;
    }

    /// The placement for `id`, or nullptr if this map does not know that tag. Linear over at
    /// most kMaxTags entries: no allocation, no branching on data the caller cannot see.
    [[nodiscard]] const TagPlacement* find(int id) const noexcept {
        for (std::size_t k = 0; k < count_; ++k) {
            if (entries_[k].id == id) {
                return &entries_[k];
            }
        }
        return nullptr;
    }

    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    [[nodiscard]] bool empty() const noexcept { return count_ == 0; }

    /// True if ANY registered tag pose is an invented number. A run anchored to invented field
    /// geometry must not read the same as one anchored to a measured field (header note).
    [[nodiscard]] bool anyInvented() const noexcept {
        for (std::size_t k = 0; k < count_; ++k) {
            if (entries_[k].provenance == TagProvenance::Invented) {
                return true;
            }
        }
        return false;
    }

    /// THE INVERSION. Given where a tag IS on the field and where it appears RELATIVE to the
    /// robot, where must the robot be?
    ///
    /// The forward composition is `tagField = robot ∘ tagInRobot`:
    ///     Tx = Rx + rx·cos(Rθ) − ry·sin(Rθ)
    ///     Ty = Ry + rx·sin(Rθ) + ry·cos(Rθ)
    ///     Tθ = Rθ + rθ
    /// so, solving for the robot:
    ///     Rθ = Tθ − rθ                     (wrap-correct; math::Angle owns that)
    ///     Rx = Tx − (rx·cos(Rθ) − ry·sin(Rθ))
    ///     Ry = Ty − (rx·sin(Rθ) + ry·cos(Rθ))
    ///
    /// Note the ORDER: the heading must be solved FIRST, because the position term is rotated by
    /// the ROBOT's heading, not the tag's. Those two coincide exactly when rθ == 0, so a suite
    /// that only ever tested a tag "facing the same way as the robot" could not tell them apart
    /// — which is why every case in tag_map_test.cpp uses a non-zero relative heading, and none
    /// uses the origin or heading 0.
    ///
    /// Static and pure: it is the tag map's arithmetic, not the corrector's, so it can be tested
    /// (and mutated) without constructing a corrector at all.
    [[nodiscard]] static math::Pose2d robotPoseFromTag(const math::Pose2d& tagField,
                                                       const math::Pose2d& tagInRobot) {
        const math::Angle robotHeading =
            math::Angle::radians(tagField.heading().radians() - tagInRobot.heading().radians());
        const double c = std::cos(robotHeading.radians());
        const double s = std::sin(robotHeading.radians());
        const double rx = tagInRobot.x().value();
        const double ry = tagInRobot.y().value();
        return math::Pose2d{units::Length{tagField.x().value() - (rx * c - ry * s)},
                            units::Length{tagField.y().value() - (rx * s + ry * c)},
                            robotHeading};
    }

private:
    TagPlacement entries_[kMaxTags]{};
    std::size_t count_ = 0;
};

}  // namespace shulib::localization
