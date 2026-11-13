#pragma once
//
// PieceFinder — "take THAT piece": a route names ONE game piece by its TYPE (the AI model's class) and its
// EXPECTED FIELD POSITION within a TOLERANCE, and the finder answers FOUND, NONE, AMBIGUOUS, NOT IN VIEW
// or UNKNOWN — NEVER A GUESS (chunk VIS1-A2, 2026-11-03). It never returns the nearest of several, and
// never one it is not sure is inside the tolerance: "not touch one vs touch another in a route" (the team
// lead, 2026-10-14) is decided by the ROUTE's expected position, and the finder says only whether the
// piece the route named is there.
//
// ── The five verdicts ───────────────────────────────────────────────────────────────
//  * FOUND — exactly one candidate of the named type and posture CERTAINLY inside the tolerance (its whole
//    error disc inside the tolerance disc), none on the border, the same one held for `holdSeconds`.
//  * NONE — nothing of that type and posture inside or on the border (it may be gone, or hidden; "wrong
//    posture inside" when a candidate of the type stood there in another posture).
//  * AMBIGUOUS — two inside, one on the border, one of the type it cannot measure whose BEARING could be
//    inside, or its own error larger than the tolerance ("drive closer, or measure the mount tighter":
//    with a mount measured to +-1 degree, two Pins a foot apart are AMBIGUOUS from about 4 ft, by design).
//  * NOT IN VIEW — the expected place (its tolerance disc, at the floor AND at a Pin's height) is not
//    certainly inside the picture, for every camera of the tolerance box. Tested on the eight corners of
//    an octagon drawn AROUND the disc: the picture's certain region on a plane is convex (the image's
//    four edges and "in front of the lens", each a half-plane, for each camera), so those eight inside
//    puts the whole disc inside — a few points ON the disc would not.
//  * UNKNOWN — no target set (or one pieceTargetProblem() refuses: nothing named is nothing known missing),
//    no usable read, none within `aliveSeconds`, the LATEST read not usable (a fault breaks the
//    picture: it is never "nothing there"), FOUND not held yet, the read POSSIBLY FULL (24 or more — a
//    piece may be among the detections never sent), or the camera's AI model not confirmed BY NAME (A4
//    register HA-197: an Object id means nothing until the names match, and a read whose Objects were
//    withheld is never evidence that nothing is there).
//
// ── Where a piece is, and how it stands (the box: A4 register HA-200) ───────────────
// A candidate's BEARING comes from its box's centre; its RANGE from the row where its box's bottom meets
// the floor (the GROUND range — the piece's near rim, its centre one radius farther). Its POSTURE:
//  * a box cut by the image's edge is UNSURE (a box cut ONLY by the bottom edge is bounded: its base is
//    nearer than the floor seen at the picture's bottom row — `nearerThanIn`);
//  * wider than `lyingAspect` it is LYING (its range from its near edge, widened by its length);
//  * narrower than `standingAspect` it stands — ON THE FLOOR when SOME camera of the tolerance box makes
//    the ground range of its box bottom agree with the range its known height implies (the SIZE range, one
//    camera at a time: the pitch error is common to both) AND NO camera of the box could see it raised
//    `raisedMinIn` or more (the team's number: the lowest a raised piece sits on this field — a Goal's
//    top, a stack). Agreeing with the floor is not enough on its own: far out, or from a high lens, a Pin
//    on a Goal agrees with the floor through a camera at the far edge of the tolerance box, and its floor
//    fix then misses it by more than its radius (VIS1-A2's tests: 73 of 2,720 such sightings from a 10-in
//    lens, 639 of 2,772 from an 18-in one) — so the raise is excluded first, shown from the whole tolerance
//    box's EXACT intervals: a raise of z0 puts the bottom row's floor point a / (a - z0) times farther than
//    the near rim, for a lens a high;
//    AND the box is a Pin STANDING, not one LYING toward the lens (VIS1-A2's review: a Pin lying near
//    end-on has a box as tall and narrow as a standing one's; it read ON THE FLOOR with a fix far from it —
//    so a box narrower than `standingAspect` is never taken as upright on its aspect alone): its WIDTH agrees,
//    on the same camera, with the piece's diameter at the range that camera agreed on (a Pin lying end-on
//    on a Goal's top fails this: its box far too wide for a Pin that far out), and NO camera of the box could
//    see it lying on the floor — a lying piece's highest point is at most a diameter up and at most its
//    length's diagonal beyond its nearest point, so the box's top row would meet the plane a diameter up no
//    farther than the ground range plus that diagonal (tested camera by camera at the box's corners: for one
//    camera the two rows' depressions differ by a pitch-free angle, so the pitch's extremes decide whenever
//    the diagonal squared is at least D(a - D) for a lens a high — a Pin under any lens below 29 in; above
//    that, the whole box's extremes);
//  * RAISED (on a Goal, in a Cup, on a stack) when NO camera can put it on the floor — its box bottom too
//    high for its size —, also shown from the whole box's exact intervals (camera_geometry.hpp: the corners
//    are not every camera, so "none" is never concluded from the corners alone);
//  * anything else is UNSURE — a Pin the stated tolerances cannot place on the floor or above it, or one that
//    could be lying toward the lens (close in, from a high lens, a standing Pin's box and a Pin lying end-on
//    look alike: UNSURE there is the honest answer).
// The size range of a real Pin reads NEARER than its near rim when the lens is above the Pin's top (its
// box takes in the top face's far rim — camera_geometry.hpp's sizeRange()): by at most 2r(a - H)/H for a
// Pin of radius r and height H under a lens a high (in the camera's own depth; off the axis, divided by
// the cosine of the bearing), which the agreement allows for. A FIX is computed only for a posture the
// evidence decides: ON THE FLOOR from both its ground range and its size range (each a bound on the near
// rim once the floor is certain — the intersection), RAISED from its size range alone (only when every
// camera's box bottom looks down, so the lens is above its base), LYING from its ground range.
// The route says which posture it wants; a candidate of the type in another posture is never taken.
//
// ── What the fix is a bound on ──────────────────────────────────────────────────────
// A field position with a radius (camera_geometry.hpp's fieldFix()): the pose the CALLER states, plus the
// camera's intervals. The pose must be the robot's at the IMAGE's time (A4 register HA-192): a robot that
// moves between the image and the read is off by its speed times the image's age, so a consumer passes
// vision::boundAtImage() with the MEASURED age — and while the age is unmeasured, takes a piece only with
// the robot stopped.
// KNOWN GAP (VIS1-A2's open finding A2-F1): off the camera's axis a standing piece's TOP is nearer the lens,
// so its cross-section is the widest in pixels and sets the box's centre column — the piece's axis at the
// box's TOP row. sightPiece() takes the bearing at the box's MIDDLE row and the ground range at its BOTTOM
// row, where that column points a little farther out: both can sit just outside the true piece. On the
// host plant: a Pin 31 degrees off the axis, 11 in out, a camera measured to 0.01 in / 0.01 degree and its
// box's bottom edge a pixel low — the fix missed it by 0.02 in past its radius. With the stated
// tolerances of a real mount the radius absorbs it (no miss in ~26,000 sightings with the host plant's box
// edges jittered by 1 px; at its full 2 px plus the integer rounding a box edge can sit up to ~3 px off, past
// kBoxEdgePx, and standing floor Pins' fixes then missed by up to 1.8 in — A4 register HA-202's bound is on
// the REPORTED edge). The fix (the bearing over the box's rows, the floor range from the row's depth) is not
// built yet: no program uses the finder before drive-to-piece.
//
// Every number the finder uses is the TEAM's (PieceFinderConfig, from the bench) or the ROUTE's
// (PieceTarget, the students'): none has a library default, and an UNSET one is refused. PROS-free,
// sim-free, allocation-free; single-task. NO HARDWARE CLAIM: every rule here has run against the host
// camera model (sim/camera_plant.hpp) and nothing else.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "shulib/config/camera_config.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/vision/camera_geometry.hpp"
#include "shulib/vision/override_elements.hpp"

namespace shulib::vision {

/// How a piece stands.
enum class Posture : std::uint8_t {
    Unsure = 0,  ///< cut by the image, merged with another, maybe lying toward the lens, or neither on the floor nor raised for certain
    OnFloor,     ///< standing on the floor: some camera puts its base, height and width at one range; no camera sees it raised or lying
    Raised,      ///< standing with its base above the floor (on a Goal, in a Cup, on a stack): no camera explains it on the floor
    Lying,       ///< on its side: its box wider than `lyingAspect`
};

/// The posture as one ASCII word: "UNSURE", "ON-FLOOR", "RAISED", "LYING".
[[nodiscard]] constexpr const char* postureWord(Posture p) noexcept {
    switch (p) {
        case Posture::Unsure: return "UNSURE";
        case Posture::OnFloor: return "ON-FLOOR";
        case Posture::Raised: return "RAISED";
        case Posture::Lying: return "LYING";
    }
    return "UNSURE";
}

/// How much NEARER than its near rim a standing piece's size range can read, inches: its box takes in the
/// top face's far rim when the lens is above its top. For a piece of `heightIn` and `diameterIn` under a
/// lens `lensHeightIn` high it is at most 2r(a - H)/H (r the radius, a the lens height, H the height — the
/// supremum over every distance; 0 when the lens is not above the top).
[[nodiscard]] constexpr double topFaceAllowanceIn(double heightIn, double diameterIn, double lensHeightIn) noexcept {
    if (!(heightIn > 0.0) || !(lensHeightIn > heightIn)) {
        return 0.0;
    }
    return diameterIn * (lensHeightIn - heightIn) / heightIn;
}

/// One Object detection, interpreted (sightPiece()).
struct PieceSighting {
    std::uint8_t classId = 0;           ///< the model's class (meaningful only once the model's names match)
    Posture posture = Posture::Unsure;  ///< how it stands
    Interval bearing{};                 ///< body frame, radians, left positive — at the IMAGE's time
    /// The lens's horizontal distance to where the box's bottom row meets the floor, inches (valid only
    /// below the horizon).
    Interval groundRange{};
    /// The lens's horizontal distance to an upright thing of the class's height filling the box, inches
    /// (unclipped boxes only).
    Interval sizeRange{};
    FieldFix fix{};                     ///< where its CENTRE is (OnFloor, Raised, Lying); invalid otherwise
    /// A box cut ONLY by the image's BOTTOM edge cannot be measured, but its base is nearer than the floor
    /// seen at the image's bottom row: the lens's horizontal distance to its centre is under this, inches
    /// (0 = no bound).
    double nearerThanIn = 0.0;
    hal::Detection box{};               ///< the detection as read
};

/// What a route asks for. Every field is the ROUTE's (the students'): none is a library default.
struct PieceTarget {
    std::uint32_t classMask = 0;      ///< bit i = the model's class i is acceptable; 0 = UNSET
    double x = 0.0;                   ///< the expected field position's x (F1), inches
    double y = 0.0;                   ///< the expected field position's y (F1), inches
    double toleranceIn = 0.0;         ///< how far from (x, y) it may be, inches; 0 = UNSET
    Posture want = Posture::OnFloor;  ///< the posture the route takes (never Unsure)
};

/// The verdict (header).
enum class PieceVerdict : std::uint8_t {
    Unknown = 0,  ///< no target, no fresh usable read, the latest read not usable, FOUND not held yet, the camera full, or the model not confirmed
    Found,        ///< exactly one candidate certainly inside, none on the border — held
    None,         ///< nothing of the type and posture inside or on the border
    Ambiguous,    ///< two inside, one on the border, one unmeasurable, or its error larger than the tolerance
    NotInView,    ///< the expected place is not certainly inside the picture
};

/// The verdict as ASCII words: "UNKNOWN", "FOUND", "NONE", "AMBIGUOUS", "NOT IN VIEW".
[[nodiscard]] constexpr const char* pieceVerdictWords(PieceVerdict v) noexcept {
    switch (v) {
        case PieceVerdict::Unknown: return "UNKNOWN";
        case PieceVerdict::Found: return "FOUND";
        case PieceVerdict::None: return "NONE";
        case PieceVerdict::Ambiguous: return "AMBIGUOUS";
        case PieceVerdict::NotInView: return "NOT IN VIEW";
    }
    return "UNKNOWN";
}

/// Why — the detail behind the verdict.
enum class PieceWhy : std::uint8_t {
    Ok = 0,                    ///< FOUND
    NoRead,                    ///< no usable read yet
    CameraSilent,              ///< no usable read within aliveSeconds
    NotHeldYet,                ///< FOUND, but not yet for holdSeconds
    TwoInside,                 ///< two candidates certainly inside
    OnTheBorder,               ///< a candidate could be inside and could be outside
    ErrorLargerThanTolerance,  ///< the candidates' own error is wider than the tolerance: drive closer, or measure the mount tighter
    WrongPostureInside,        ///< a candidate of the type stands there in another posture
    OutOfView,                 ///< the expected place is not certainly in the picture
    NothingThere,              ///< nothing of the type there
    CameraFull,                ///< the read was POSSIBLY FULL: a piece may be among the detections never sent
    UnmeasurableInside,        ///< a candidate of the type it cannot measure could be inside
    ModelNotConfirmed,         ///< the camera's AI model is not confirmed BY NAME: its ids mean nothing
    ReadNotUsable,             ///< the latest read was a fault, torn or not configured: no evidence either way
    NoTarget,                  ///< no target set (or one pieceTargetProblem() refuses): it knows of no piece to look for
};

/// The reason in a few ASCII words.
[[nodiscard]] constexpr const char* pieceWhyWords(PieceWhy w) noexcept {
    switch (w) {
        case PieceWhy::Ok: return "ok";
        case PieceWhy::NoRead: return "no read yet";
        case PieceWhy::CameraSilent: return "camera silent";
        case PieceWhy::NotHeldYet: return "not held yet";
        case PieceWhy::TwoInside: return "two inside";
        case PieceWhy::OnTheBorder: return "on the border";
        case PieceWhy::ErrorLargerThanTolerance: return "error larger than the tolerance";
        case PieceWhy::WrongPostureInside: return "wrong posture inside";
        case PieceWhy::OutOfView: return "out of view";
        case PieceWhy::NothingThere: return "nothing there";
        case PieceWhy::CameraFull: return "camera full";
        case PieceWhy::UnmeasurableInside: return "unmeasurable inside";
        case PieceWhy::ModelNotConfirmed: return "model not confirmed";
        case PieceWhy::ReadNotUsable: return "read not usable";
        case PieceWhy::NoTarget: return "no target";
    }
    return "?";
}

/// Every number the finder uses that is not the route's — the TEAM's, from the bench. Defaults are UNSET and
/// refused: 0 for the numbers that must be positive, -1 for the two whose 0 means something (`minScore`'s
/// "every score", `viewMarginPx`'s "no margin").
struct PieceFinderConfig {
    double holdSeconds = 0.0;     ///< FOUND must hold this long (the same candidate), seconds
    double aliveSeconds = 0.0;    ///< a usable read at most this old, seconds
    double minScore = -1.0;       ///< an Object scored below this is ignored (the raw scale: A4 register HA-198); 0 = every score; -1 = UNSET
    double lyingAspect = 0.0;     ///< a box wider over tall than this is LYING
    double standingAspect = 0.0;  ///< a box narrower over tall than this may stand (the posture rules decide; between the two: UNSURE)
    double viewMarginPx = -1.0;   ///< the expected place must be at least this far inside the picture, pixels; 0 = none; -1 = UNSET
    /// The LOWEST a raised piece's base sits above the floor on this field (a Goal's top, a stack), inches
    /// — the team's, read from the game manual's field drawings (a fact of the field, not of the bench): a
    /// standing piece is ON THE FLOOR only when no camera of the tolerance box could see it raised this much
    /// or more. A smaller number refuses more (UNSURE); a larger one than the field's lowest raise lets a
    /// raised piece read as a floor one.
    double raisedMinIn = 0.0;
};

/// The first problem with a finder config, or nullptr: a number that is not finite; the hold or alive time
/// UNSET; the standing aspect UNSET or the lying one not above it; the score floor or the view margin UNSET
/// (negative); the lowest raise UNSET.
[[nodiscard]] constexpr const char* pieceFinderConfigProblem(const PieceFinderConfig& c) noexcept {
    const double numbers[] = {c.holdSeconds,    c.aliveSeconds,   c.minScore,   c.lyingAspect,
                              c.standingAspect, c.viewMarginPx,   c.raisedMinIn};
    for (const double n : numbers) {
        if (!config::finiteNumber(n)) {
            return "a finder number is not a finite number";
        }
    }
    if (!(c.holdSeconds > 0.0)) {
        return "holdSeconds UNSET -- how long FOUND must hold";
    }
    if (!(c.aliveSeconds > 0.0)) {
        return "aliveSeconds UNSET -- how old the newest usable read may be";
    }
    if (!(c.standingAspect > 0.0) || !(c.lyingAspect > c.standingAspect)) {
        return "standing / lying aspect UNSET or reversed";
    }
    if (c.minScore < 0.0) {
        return "minScore UNSET (negative) -- the lowest model score believed, 0 for every score";
    }
    if (c.viewMarginPx < 0.0) {
        return "viewMarginPx UNSET (negative) -- how far inside the picture the place must be, 0 for none";
    }
    if (!(c.raisedMinIn > 0.0)) {
        return "raisedMinIn UNSET -- the lowest a raised piece sits on this field (a Goal's top), inches";
    }
    return nullptr;
}

/// The first problem with a route's target, or nullptr: no class named, or one the season's model does not
/// name; a position or tolerance that is not finite; the tolerance UNSET; an UNSURE posture asked for.
[[nodiscard]] constexpr const char* pieceTargetProblem(const PieceTarget& t) noexcept {
    if (t.classMask == 0U) {
        return "no class named";
    }
    if ((t.classMask >> override2026::kModelClassCount) != 0U) {
        return "a class the season's AI model does not name";
    }
    if (!config::finiteNumber(t.x) || !config::finiteNumber(t.y) || !config::finiteNumber(t.toleranceIn)) {
        return "the expected position or the tolerance is not a finite number";
    }
    if (!(t.toleranceIn > 0.0)) {
        return "tolerance UNSET";
    }
    if (t.want == Posture::Unsure) {
        return "a route cannot ask for an UNSURE piece";
    }
    return nullptr;
}

/// The slack, inches, the finder allows between the ground and the size ranges of ONE camera for what the
/// model leaves out (a box's corners off the axis's vertical plane).
inline constexpr double kPostureSlackIn = 0.25;

namespace detail {

/// Could an UPRIGHT cylinder `diameterIn` across and `heightIn` tall, standing on the floor with its centre
/// at a horizontal lens distance in [`centreLo`, `centreHi`], fill a box `widthPx` wide through camera `c`,
/// within each edge's +-kBoxEdgePx? Bounds, never estimates: the piece's every point lies between the depths
/// `zMin` and `zMax` along this camera's axis, and laterally within its centre's offset and a radius; its
/// widest cross-section's two side points share one depth, so the true width is at least D fx / zMax, and no
/// two of its points are farther apart in the picture than the farthest-out one at zMin and the nearest-in
/// one at zMax. The centre's angle off this camera's axis is bounded by the box's own corners (the piece is
/// inside its box; for a fixed column the angle is monotone in the row). True when it cannot judge (a depth
/// not in front of the lens, or an angle past 80 degrees) — it never refuses on what it cannot bound.
[[nodiscard]] inline bool uprightWidthFits(const CameraCorner& c, const hal::Detection& d, double diameterIn,
                                           double heightIn, double centreLo, double centreHi) noexcept {
    const double r = diameterIn / 2.0;
    double aMin = 0.0;
    double aMax = 0.0;
    bool first = true;
    const double l = d.left - kBoxEdgePx;
    const double rt = d.left + d.width + kBoxEdgePx;
    for (const double u : {l, rt}) {
        for (const double v : {d.top - kBoxEdgePx, d.top + d.height + kBoxEdgePx}) {
            double b = 0.0;
            double e = 0.0;
            CameraModel::ray(c, u, v, b, e);
            const double a = std::fabs(std::remainder(b - c.yaw, 2.0 * math::Angle::kPi));
            aMin = first ? a : std::fmin(aMin, a);
            aMax = first ? a : std::fmax(aMax, a);
            first = false;
        }
    }
    if (l <= c.cx && rt >= c.cx) {
        aMin = 0.0;  // the box spans this camera's axis column: a point of it is straight ahead
    }
    if (!(aMax < 80.0 * kDegToRad)) {
        return true;
    }
    const double cp = std::cos(c.pitch);
    const double sp = std::sin(c.pitch);
    const double zLow = std::fmin(-c.h * sp, (heightIn - c.h) * sp);
    const double zHigh = std::fmax(-c.h * sp, (heightIn - c.h) * sp);
    const double zMin = (centreLo * std::cos(aMax) - r) * cp + zLow;
    const double zMax = (centreHi * std::cos(aMin) + r) * cp + zHigh;
    if (!(zMin > 0.0) || !std::isfinite(zMax)) {
        return true;
    }
    const double lateral = centreHi * std::sin(aMax);  // the centre's farthest lateral offset
    const double wMin = c.fx * diameterIn / zMax;
    const double wMax = c.fx * ((lateral + r) / zMin - (lateral >= r ? (lateral - r) / zMax : (lateral - r) / zMin));
    return d.width + 2.0 * kBoxEdgePx >= wMin && d.width - 2.0 * kBoxEdgePx <= wMax;
}

}  // namespace detail

/// Interpret one Object detection, seen from `robot` (the caller's pose and its stated bound) — pure
/// (header: the posture rules, the fix).
[[nodiscard]] inline PieceSighting sightPiece(const hal::Detection& d, const CameraModel& m, const PoseBound& robot,
                                              const PieceFinderConfig& cfg) noexcept {
    PieceSighting s{};
    s.classId = d.id;
    s.box = d;
    const override2026::PieceSize* size = override2026::modelClassSize(d.id);
    if (d.kind != hal::DetectionKind::Object || size == nullptr || d.width <= 0 || d.height <= 0) {
        return s;
    }
    const double u = d.left + d.width / 2.0;
    const double vBottom = d.top + d.height;
    const double rad = size->diameterIn / 2.0;
    s.bearing = m.bearing(u, d.top + d.height / 2.0);
    if (!s.bearing.valid) {
        return s;
    }
    // Off the camera's axis a depth along it is shorter than the horizontal range by the bearing's cosine
    // (header): the bearing's farthest end, the mount's yaw and its tolerance, capped well short of 90.
    const config::CameraPlacement& p = m.placement();
    const double offAxis = std::fmin(80.0 * kDegToRad, std::fmax(std::fabs(s.bearing.lo), std::fabs(s.bearing.hi))
                                                           + (std::fabs(p.yawDeg) + p.yawToleranceDeg) * kDegToRad);
    const double sec = 1.0 / std::cos(offAxis);
    if (d.clipped()) {  // UNSURE: part of it is outside the image
        const bool bottomOnly = d.top + d.height >= hal::kVisionImageHeightPx - 1 && d.top > 1 && d.left > 1
                                && d.left + d.width < hal::kVisionImageWidthPx - 1;
        if (bottomOnly) {
            // Its base is below the picture: its near rim is nearer than the floor seen at the image's bottom
            // row — the LAST row a reported box can end on, less its edge's jitter (the farthest the base can
            // be), at the column farthest from the axis, for every camera of the box.
            double bound = 0.0;
            bool ok = true;
            m.eachCorner(/*useHeight=*/true, [&](const CameraCorner& c) {
                for (const double du : {-kBoxEdgePx, kBoxEdgePx}) {
                    double g = 0.0;
                    if (!CameraModel::rangeToPlane(c, u + du, hal::kVisionImageHeightPx - 1 - kBoxEdgePx, 0.0, 2.0,
                                                   g)) {
                        ok = false;
                        return;
                    }
                    bound = std::fmax(bound, g);
                }
            });
            if (ok) {
                s.nearerThanIn = bound + rad * sec;
            }
        }
        return s;
    }
    const double aspect = static_cast<double>(d.width) / d.height;
    if (aspect >= cfg.lyingAspect) {
        s.posture = Posture::Lying;
        s.groundRange = m.groundRange(u, vBottom);  // a lying piece touches the floor
        if (s.groundRange.valid) {
            // Its box bottom is its nearest end's floor contact; its centre is beyond that, by at most half its
            // length and a radius (whichever way it lies), divided off the axis by the bearing's cosine.
            Interval r = s.groundRange;
            r.hi += (size->heightIn / 2.0 + rad) * sec;
            s.fix = m.fieldFix(robot, s.bearing, r);
        }
        return s;
    }
    if (aspect > cfg.standingAspect) {
        return s;  // UNSURE: end-on, tilted, or two merged
    }
    s.groundRange = m.groundRange(u, vBottom);
    s.sizeRange = m.sizeRange(size->heightIn, u, d.top + d.height / 2.0, d.height);
    if (!s.sizeRange.valid) {
        return s;
    }
    const double lensHi = p.height.value() + p.heightTolerance.value();
    const double allowanceMax = topFaceAllowanceIn(size->heightIn, size->diameterIn, lensHi) * sec;
    // ON THE FLOOR, part 1 — ONE camera at a time (A4 register HA-200: the box encloses the whole piece, its
    // bottom row the base's near rim): the size range reads the rim, or nearer by up to the top face's
    // allowance; they agree when that camera's two ranges meet.
    bool agrees = false;
    m.eachCorner(/*useHeight=*/true, [&](const CameraCorner& c) {
        if (agrees) {
            return;
        }
        const Interval g = m.groundRangeAt(c, u, vBottom);
        const Interval z = m.sizeRangeAt(c, size->heightIn, u, d.top + d.height / 2.0, d.height);
        if (!g.valid || !z.valid) {
            return;
        }
        const double allowance = topFaceAllowanceIn(size->heightIn, size->diameterIn, c.h) * sec;
        if (z.lo - kPostureSlackIn <= g.hi && z.hi + allowance + kPostureSlackIn >= g.lo) {
            // ... and its WIDTH: the box as wide as an upright piece at that camera's ranges (header) — the near
            // rim between the two ranges' common ends, the centre a radius beyond it, padded by a radius more
            // each way.
            const double rimLo = std::fmax(g.lo, z.lo - kPostureSlackIn);
            const double rimHi = std::fmin(g.hi, z.hi + allowance + kPostureSlackIn);
            if (detail::uprightWidthFits(c, d, size->diameterIn, size->heightIn, rimLo,
                                         rimHi + size->diameterIn * sec)) {
                agrees = true;
            }
        }
    });
    // ON THE FLOOR, part 3 — no camera could see it LYING on the floor (header). A lying piece's lowest
    // point in the picture is no farther than the ground range G, and its highest is at most its diameter D
    // up and at most K = hypot(length, D) beyond that, so that row's ray meets the plane D up no farther than
    // G + K. Tested camera by camera, at the tolerance box's corners: G at its farthest (the bottom row a jitter
    // up, either jittered column or the principal point's), the top row's meeting at its nearest. Exact at the
    // corners for the pitch: for one camera the two rows' depressions differ by a pitch-free angle, and the
    // depression a lying top could have rises SLOWER than the top row's own whenever K^2 >= D(a - D) for a
    // lens a high (the log, A2's review) — so a camera between two pitches cannot be the one that fits. The
    // height enters G + K - (the meeting) monotonically, the rows too. When that inequality fails (a lens
    // high over a fat piece), the whole box's extremes are used instead — never a tighter test.
    const double lieK = std::hypot(size->heightIn, size->diameterIn);
    const bool perCamera = lieK * lieK >= size->diameterIn * (lensHi - size->diameterIn);
    double farthestGround = 0.0;
    double nearestTop = std::numeric_limits<double>::infinity();
    bool lieFits = !s.groundRange.valid;
    m.eachCorner(/*useHeight=*/true, [&](const CameraCorner& c) {
        const double cols[3] = {u - kBoxEdgePx, u + kBoxEdgePx, c.cx};
        const int colCount = std::fabs(u - c.cx) < kBoxEdgePx ? 3 : 2;  // the principal point's column, when reached
        double gFar = 0.0;
        double topNear = std::numeric_limits<double>::infinity();
        for (int k = 0; k < colCount; ++k) {
            for (const double dv : {-kBoxEdgePx, kBoxEdgePx}) {
                double g = 0.0;
                if (CameraModel::rangeToPlane(c, cols[k], vBottom + dv, 0.0, 0.0, g)) {
                    gFar = std::fmax(gFar, g);
                } else {
                    gFar = std::numeric_limits<double>::infinity();  // a bottom row that meets no floor
                }
                if (CameraModel::rangeToPlane(c, cols[k], d.top + dv, size->diameterIn, 0.0, g)) {
                    topNear = std::fmin(topNear, g);
                }
            }
        }
        farthestGround = std::fmax(farthestGround, gFar);
        nearestTop = std::fmin(nearestTop, topNear);
        if (!(gFar + lieK + kPostureSlackIn < topNear)) {
            lieFits = true;
        }
    });
    const bool noLie = perCamera ? !lieFits : farthestGround + lieK + kPostureSlackIn < nearestTop;
    // ON THE FLOOR, part 2 — no camera could see it raised `raisedMinIn` or more (header), from the whole
    // box: a raise of z0 >= raisedMinIn puts the floor point at least lensHi / (lensHi - raisedMinIn) times
    // the near rim, which is at least the size range's near end (a raise at or above every lens would not
    // look down at all, and this box's bottom does).
    const bool noRaise = cfg.raisedMinIn >= lensHi
                         || (s.groundRange.valid
                             && s.groundRange.hi + kPostureSlackIn
                                    < s.sizeRange.lo * lensHi / (lensHi - cfg.raisedMinIn));
    if (agrees && noRaise && noLie && s.groundRange.valid) {
        // Its centre: one radius beyond the near rim (off the axis, up to r / cos further), the rim bounded
        // by the ground range AND — the floor now certain — by the size range (and its allowance).
        const double lo = std::fmax(s.groundRange.lo, s.sizeRange.lo) + rad;
        const double hi = std::fmin(s.groundRange.hi + rad * sec, s.sizeRange.hi + allowanceMax + rad * sec);
        if (lo <= hi) {
            s.posture = Posture::OnFloor;
            s.fix = m.fieldFix(robot, s.bearing, Interval{true, lo, hi});
        }
        return s;
    }
    if (agrees) {
        return s;  // UNSURE: the floor fits, and so might a raise
    }
    // RAISED — NO camera of the box puts it on the floor, shown from the whole box's EXACT intervals: the
    // farthest the size range lets its near rim be is still nearer than the nearest the ground range lets its
    // box bottom be. The ground range's nearest, over every camera: its corners (the ground range is monotone
    // in each input) and the principal point's column, any ray that descends at all (one that does not meets
    // no floor: farther than anything).
    const double farthestRim = s.sizeRange.hi + allowanceMax;
    double nearestGround = std::numeric_limits<double>::infinity();
    bool allDescend = true;
    m.eachCorner(/*useHeight=*/true, [&](const CameraCorner& c) {
        const double cols[3] = {u - kBoxEdgePx, u + kBoxEdgePx, c.cx};
        const int colCount = std::fabs(u - c.cx) < kBoxEdgePx ? 3 : 2;  // the principal point's column, when reached
        for (int k = 0; k < colCount; ++k) {
            for (const double dv : {-kBoxEdgePx, kBoxEdgePx}) {
                double g = 0.0;
                if (CameraModel::rangeToPlane(c, cols[k], vBottom + dv, 0.0, 0.0, g)) {
                    nearestGround = std::fmin(nearestGround, g);
                } else {
                    allDescend = false;
                }
            }
        }
    });
    if (farthestRim + kPostureSlackIn < nearestGround) {
        s.posture = Posture::Raised;
        // Its centre, only when every camera's box bottom looks down (the lens above its base, so the size
        // range bounds its near rim — header): the rim (the size range, up to the allowance farther) plus
        // one radius.
        if (allDescend) {
            s.fix = m.fieldFix(robot, s.bearing, Interval{true, s.sizeRange.lo + rad, farthestRim + rad * sec});
        }
    }
    return s;
}

/// The finder (header): set the route's target, feed it reads, ask it at any time. Single-task.
class PieceFinder {
public:
    /// The most sightings one read holds (the camera's 24).
    static constexpr std::size_t kMax = hal::kMaxVisionDetections;

    /// The answer at a moment.
    struct Result {
        PieceVerdict verdict = PieceVerdict::Unknown;  ///< the verdict
        PieceWhy why = PieceWhy::NoRead;               ///< the detail behind it
        PieceSighting sighting{};                      ///< the one FOUND (meaningful only then)
        int inside = 0;                                ///< candidates certainly inside the tolerance
        int border = 0;                                ///< candidates on its border, or unmeasurable and maybe inside
    };

    /// PRECONDITION: pieceFinderConfigProblem(cfg) == nullptr, and `model` built from a mount
    /// cameraModelProblem() accepts — a consumer asks both first and refuses to construct, naming them.
    PieceFinder(const PieceFinderConfig& cfg, const CameraModel& model) noexcept : cfg_{cfg}, m_{model} {}

    /// The route's target (pieceTargetProblem(t) must be nullptr: one it refuses is no target — every answer
    /// UNKNOWN "no target" until a valid one is set). A new target restarts the hold.
    void setTarget(const PieceTarget& t) noexcept {
        target_ = t;
        hasTarget_ = pieceTargetProblem(t) == nullptr;
        holdSince_ = -1.0;
    }

    /// One read, seen from `robot` — the caller's pose and its stated bound, AT THE IMAGE's time (header).
    void update(const hal::VisionFrame& f, const PoseBound& robot) noexcept {
        robot_ = robot;
        lastUsable_ = f.usable();
        if (!lastUsable_) {
            n_ = 0;
            holdSince_ = -1.0;  // positive evidence only: a fault is no picture at all, never "nothing there"
            return;
        }
        haveOk_ = true;
        lastOk_ = f.readAt.value();
        truncated_ = f.truncated;
        // The model check BY NAME (A4 register HA-197): withheld ids are no evidence of absence.
        modelOk_ = f.model == hal::ModelCheck::Matches;
        n_ = 0;
        if (!modelOk_ || truncated_) {
            holdSince_ = -1.0;
            return;
        }
        const std::size_t count = f.count < kMax ? f.count : kMax;
        for (std::size_t i = 0; i < count; ++i) {
            const hal::Detection& d = f.detections[i];
            if (d.kind != hal::DetectionKind::Object || d.score < cfg_.minScore) {
                continue;
            }
            seen_[n_++] = sightPiece(d, m_, robot, cfg_);
        }
        // The hold: FOUND must keep finding the same candidate (its fix within the two radii).
        const Result r = decide();
        if (r.verdict == PieceVerdict::Found) {
            const bool same = holdSince_ >= 0.0
                              && std::hypot(r.sighting.fix.x - heldX_, r.sighting.fix.y - heldY_)
                                     <= r.sighting.fix.radius + heldR_;
            if (!same) {
                holdSince_ = lastOk_;
            }
            heldX_ = r.sighting.fix.x;
            heldY_ = r.sighting.fix.y;
            heldR_ = r.sighting.fix.radius;
        } else {
            holdSince_ = -1.0;
        }
    }

    /// The answer at `nowSeconds` (the clock the reads are stamped with).
    [[nodiscard]] Result answer(double nowSeconds) const noexcept {
        Result r{};
        if (!hasTarget_) {
            r.why = PieceWhy::NoTarget;  // never NONE: with nothing named, nothing is known to be missing
            return r;
        }
        if (!haveOk_) {
            return r;  // UNKNOWN, no read yet
        }
        if (!(nowSeconds - lastOk_ <= cfg_.aliveSeconds)) {
            r.why = PieceWhy::CameraSilent;
            return r;
        }
        if (!lastUsable_) {
            r.why = PieceWhy::ReadNotUsable;
            return r;
        }
        if (!modelOk_) {
            r.why = PieceWhy::ModelNotConfirmed;
            return r;
        }
        if (truncated_) {
            r.why = PieceWhy::CameraFull;
            return r;
        }
        r = decide();
        if (r.verdict == PieceVerdict::Found && (holdSince_ < 0.0 || lastOk_ - holdSince_ < cfg_.holdSeconds)) {
            r.verdict = PieceVerdict::Unknown;
            r.why = PieceWhy::NotHeldYet;
        }
        return r;
    }

private:
    [[nodiscard]] Result decide() const noexcept {
        const PieceTarget& t = target_;
        Result r{};
        const double th = robot_.pose.heading().radians();
        const double ch = std::cos(th);
        const double sh = std::sin(th);
        // NOT IN VIEW: the expected place — its tolerance disc, widened by the pose's and the mount's position
        // tolerances and by the heading tolerance at that distance — must be certainly in view, at the floor
        // AND at a Pin's height, for every camera of the box: the eight corners of the octagon AROUND that
        // disc (header: the certain region is convex).
        const double dx0 = t.x - robot_.pose.x().value();
        const double dy0 = t.y - robot_.pose.y().value();
        const double slack = t.toleranceIn + robot_.positionTolerance.value() + m_.placement().positionTolerance.value()
                             + std::hypot(dx0, dy0) * robot_.headingToleranceDeg * kDegToRad;
        if (!std::isfinite(slack)) {
            r.verdict = PieceVerdict::NotInView;
            r.why = PieceWhy::OutOfView;
            return r;
        }
        const double corner = slack / std::cos(math::Angle::kPi / 8.0);  // the octagon's circumradius
        for (int k = 0; k < 8; ++k) {
            const double a = k * math::Angle::kPi / 4.0;
            const double dx = dx0 + corner * std::cos(a);
            const double dy = dy0 + corner * std::sin(a);
            const double fwd = ch * dx + sh * dy;
            const double lft = -sh * dx + ch * dy;
            if (!m_.inViewForCertain(fwd, lft, 0.0, cfg_.viewMarginPx)
                || !m_.inViewForCertain(fwd, lft, override2026::kPinSize.heightIn, cfg_.viewMarginPx)) {
                r.verdict = PieceVerdict::NotInView;
                r.why = PieceWhy::OutOfView;
                return r;
            }
        }
        bool wrongPostureInside = false;
        bool tooVague = true;
        bool any = false;
        bool unmeasurable = false;
        std::size_t found = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            const PieceSighting& s = seen_[i];
            if (s.classId >= 32U || ((t.classMask >> s.classId) & 1U) == 0U) {
                continue;
            }
            if (!s.fix.valid) {
                // It cannot be measured: if its BEARING could point inside the tolerance it cannot be ruled
                // out — it counts on the border (AMBIGUOUS), never silently dropped; unless a bottom-cut box
                // is certainly NEARER than the tolerance disc.
                if (s.nearerThanIn > 0.0 && s.nearerThanIn < nearestOfDisc()) {
                    continue;
                }
                if (s.bearing.valid && bearingCouldBeInside(s.bearing)) {
                    ++r.border;
                    unmeasurable = true;
                }
                continue;
            }
            const double dist = std::hypot(s.fix.x - t.x, s.fix.y - t.y);
            if (dist - s.fix.radius > t.toleranceIn) {
                continue;  // certainly outside
            }
            if (s.posture != t.want) {
                wrongPostureInside = true;
                continue;
            }
            any = true;
            if (s.fix.radius < t.toleranceIn) {
                tooVague = false;
            }
            if (dist + s.fix.radius <= t.toleranceIn) {
                ++r.inside;
                found = i;
            } else {
                ++r.border;
            }
        }
        if (r.inside == 1 && r.border == 0) {
            r.verdict = PieceVerdict::Found;
            r.why = PieceWhy::Ok;
            r.sighting = seen_[found];
        } else if (r.inside == 0 && r.border == 0) {
            r.verdict = PieceVerdict::None;
            r.why = wrongPostureInside ? PieceWhy::WrongPostureInside : PieceWhy::NothingThere;
        } else {
            r.verdict = PieceVerdict::Ambiguous;
            r.why = r.inside >= 2              ? PieceWhy::TwoInside
                    : unmeasurable            ? PieceWhy::UnmeasurableInside
                    : (any && tooVague)       ? PieceWhy::ErrorLargerThanTolerance
                                              : PieceWhy::OnTheBorder;
        }
        return r;
    }

    // The lens's horizontal distance to the NEAREST point of the target's tolerance disc, inches (0 inside it)
    // — a bound from below: the pose's position tolerance and the lens's swing over its heading tolerance
    // taken off.
    [[nodiscard]] double nearestOfDisc() const noexcept {
        const PieceTarget& t = target_;
        const double th = robot_.pose.heading().radians();
        const double ch = std::cos(th);
        const double sh = std::sin(th);
        const double dx = t.x - robot_.pose.x().value();
        const double dy = t.y - robot_.pose.y().value();
        const double fwd = ch * dx + sh * dy - m_.placement().forward.value();
        const double lft = -sh * dx + ch * dy - m_.placement().left.value();
        const double lensSwing = std::hypot(m_.placement().forward.value(), m_.placement().left.value())
                                 * robot_.headingToleranceDeg * kDegToRad;
        const double nearest = std::hypot(fwd, lft) - t.toleranceIn - robot_.positionTolerance.value() - lensSwing
                               - m_.placement().positionTolerance.value();
        return std::isfinite(nearest) ? std::fmax(0.0, nearest) : 0.0;
    }

    // Could a thing at body bearing interval `b` lie inside the target's tolerance disc? The disc, seen from
    // the lens, spans a cone of bearings (widened by the heading tolerance); an overlap means it could.
    [[nodiscard]] bool bearingCouldBeInside(const Interval& b) const noexcept {
        const PieceTarget& t = target_;
        const double th = robot_.pose.heading().radians();
        const double ch = std::cos(th);
        const double sh = std::sin(th);
        const double dx = t.x - robot_.pose.x().value();
        const double dy = t.y - robot_.pose.y().value();
        const double fwd = ch * dx + sh * dy - m_.placement().forward.value();
        const double lft = -sh * dx + ch * dy - m_.placement().left.value();
        const double dist = std::hypot(fwd, lft);
        const double slack = t.toleranceIn + robot_.positionTolerance.value() + m_.placement().positionTolerance.value();
        if (!(dist > slack)) {
            return true;  // the lens is inside the disc (or a number is not finite): it could be anywhere
        }
        const double centre = std::atan2(lft, fwd);
        const double half = std::asin(slack / dist) + robot_.headingToleranceDeg * kDegToRad;
        return !(b.hi < centre - half) && !(b.lo > centre + half);
    }

    PieceFinderConfig cfg_;
    CameraModel m_;
    PieceTarget target_{};
    bool hasTarget_ = false;
    PoseBound robot_{};
    PieceSighting seen_[kMax]{};
    std::size_t n_ = 0;
    bool haveOk_ = false;
    double lastOk_ = 0.0;
    double holdSince_ = -1.0;
    double heldX_ = 0.0;
    double heldY_ = 0.0;
    double heldR_ = 0.0;
    bool truncated_ = false;
    bool modelOk_ = false;
    bool lastUsable_ = false;
};

}  // namespace shulib::vision
