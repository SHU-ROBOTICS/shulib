#pragma once
//
// ToggleTracker — WHICH COLOUR does the Toggle in front of the robot show: RED, BLUE, YELLOW, or UNKNOWN
// with the reason (chunk VIS1-A2, 2026-11-03). It reports a colour ONLY when it is sure: the same band, of
// the same colour, passing every gate, on every usable read for at least `holdSeconds`, with a usable
// read within `aliveSeconds` of now. "The same band": read to read, its direction from the lens (the
// heading plus its bearing) and its range overlap the last read's, widened only by what the lens's travel
// since then explains, and (with the position gate on) it names the same Toggle — another band of the same
// colour taking over restarts the hold. Anything else is UNKNOWN, and UNKNOWN never triggers anything (the
// library's evidence rule: a consumer acts only on a colour). It reports COLOURS, never "ours": which
// colour a robot wants is its alliance's (the consumer's), not this header's.
//
// ── What a colour MEANS ─────────────────────────────────────────────────────────────
// The colour of the face the camera SEES where a seated Toggle's field face could be, within the stated
// tolerances — NOT the game's "set" (fully seated, a face parallel with its mounts, AND no robot
// touching it; otherwise neutral). A Toggle stuck mid-roll shows its DESTINATION face from below and can
// read as that colour, and the camera's measure of a face's lean is never fine enough to tell a seated
// Toggle from a rolled one at any mount and range VIS1's host camera model tried. So this header exposes
// no "seated" flag and no lean: seating and contact are the consumer's to establish, AT THE PUSH POINT,
// with a sensor that sees colour there (an Optical at the flipper — a distance sensor says something is
// there, never which colour). The camera's colour is the FAR-RANGE cue.
//
// ── The gates, in order (each turns a lookalike away) ───────────────────────────────
//  1. KIND AND ID: a Colour detection with one of the three Toggle descriptor ids; anything else is
//     ignored.
//  2. SHAPE: a long, flat band — width over height at least `minAspect` (a standing Pin's half is tall
//     and thin; a Cup is not a band) — and taller than 2 x kBoxEdgePx (a library rule, not a team
//     number: a band within its edges' jitter cannot be measured; without it the bottom face's far-off
//     sliver read as a far face).
//  3. CUT: a band touching the image's top or bottom is a face leaving the frame — "cut by the image",
//     never a colour from a partial face. A band cut at a SIDE has no measurable length, so it is judged
//     only when it spans the image's centre column (a real face close in, wider than the picture, does);
//     otherwise it is "cut" too (a short lookalike reaching in from the edge — a 4.5-in licence plate,
//     a lying Pin — never reaches the length gate it would fail).
//  4. ONE CAMERA EXPLAINS IT (camera_geometry.hpp, tested CORNER BY CORNER): the band's bottom row on
//     the face's lower edge (an interval, HA-201) and its top row on the apex, at ranges that put the apex
//     0.9–1.2 in behind the lower edge (the face LEANS BACK — a vertical plate does not), AND (a band not
//     cut at a side) a width that is `minLengthFraction..maxLengthFraction` of the Toggle's 25.8 in at
//     that range. Never by overlapping two separately widened intervals: the pitch error is common to
//     both edges of one box.
//  5. NOT THE BOTTOM FACE: the Toggle's BOTTOM face (a different colour) is visible from a camera below
//     it and, close in, mimics the field face. A band some corner explains as the bottom face (both
//     edges on the bottom's plane, 1–3 in deep) is kept only when the place its field face would occupy,
//     directly above, is certainly in the image AND holds no other Toggle-coloured band; a band there
//     makes this one the bottom face (dropped); that place out of frame — UNKNOWN "maybe the bottom face".
//     WHAT THIS RESTS ON: an empty place is taken to mean "no field face there", so a read on which the
//     field face's descriptor MISSES while its bottom face's fires lets the bottom face through — on the
//     host camera model, the field face's detection removed, the bottom face's colour read sure in about one
//     parking in ten (VIS1-A2's review); a field face detected but cut short by up to 6 px, never. Refusing
//     every band some corner explains as a bottom face would refuse the real face at almost every distance
//     from a lens below it (measured: a 6-in lens never read a colour) and STILL let a missed face's bottom
//     through (16 of 600 parkings: a bottom face no corner explains as one is judged as a face). So the
//     field face's descriptor must fire on every read on which its bottom face's does — A4 register
//     HA-199, settled at the bench.
//  6. WHERE: its bearing interval wholly inside +-`bearingWindowDeg`, and its range interval OVERLAPPING
//     [`minRangeIn`, `maxRangeIn`] — the window drops bands that are certainly elsewhere (another wall's
//     Toggle); the view carries both intervals, so a consumer can demand more.
//  7. POSITION (optional): with a pose bound tighter than `maxPoseToleranceIn`, the band's field position
//     (fix radius at most `maxFixRadiusIn`) must lie on a Toggle's face line (override2026::kTogglePlaces,
//     within `positionToleranceIn`); `toggleIndex` says which.
// And, over the whole read:
//  * Two DIFFERENT colours passing at once — UNKNOWN (never "pick one"); the same colour twice — the
//    nearer band.
//  * A passing band beside a band of ANOTHER Toggle colour the gates could not judge (cut by the image, or
//    maybe a bottom face) whose bearing touches the window — UNKNOWN: that band may be the real face.
//  * A band with a box of ANOTHER Toggle colour directly ABOVE it (columns overlapping; the other's top
//    above the band's and its bottom within 2 x kBoxEdgePx of the band's top, or lower; whatever ITS gates
//    said) — UNKNOWN. A seated field face has nothing of another Toggle colour directly above it (its
//    bottom face is BELOW it); a bottom face does (its field face), and so does a Toggle mid-roll showing
//    two faces. Without it, a robot TURNED 15 degrees to a RED Toggle 30 in away read the YELLOW bottom
//    face "sure": off the optical axis a horizontal edge projects SLANTED, the field face's box grows
//    taller than the face, gate 4 refuses it — and the thin bottom face under it passed. The cost: close
//    in, the colour holds only when the robot is SQUARE to the Toggle (VIS1-A2's tests, 8-, 10- and 11-in
//    lenses turned in 5-degree steps: at 14 in only square; at 20 in square, an 11-in lens to 5 degrees one
//    way; at 30 in from square only (8 in) to 10–15 degrees (11 in); at 40 in 5 (8 in) to 15; at 48 in
//    10 (8 in) to 15; never past 15 — the R7 test prints the table). Square up; never remove this to "get
//    coverage back".
//  * A read reporting 24 or more (`VisionFrame::truncated`, POSSIBLY FULL) — UNKNOWN "camera full": the
//    band that contradicts a colour may be among the detections never sent.
//
// ── Freshness is TIME, and a frozen camera is caught only while the camera MOVES ─────
// PROS gives no frame counter (A4 register HA-192), so a camera that keeps answering with its last frame
// reads exactly like a still scene. While the CAMERA moves — its LENS (the robot's pose through the
// mount, never the robot's centre: a robot turning in place hardly moves its centre) travels more than
// `frozenMoveIn`, OR the robot turns more than `frozenTurnDeg` — a band whose box has not moved by a
// single pixel is UNKNOWN "camera frozen?"; the first read whose box moves again clears it and RESTARTS
// the hold. Parked — exactly where a consumer checks a push — nothing can tell a frozen camera from a
// still scene: that check is never the camera's.
//
// ── What the tracker cannot do (the host camera model's lookalikes, stated) ─────────
// A colour band the gates cannot tell from a face is reported as one (A4 register HA-199). With the
// position gate off, red bands of the wrong height or length close to a wall pass (VIS1-A2's tests: 70 of
// 540 lookalike bands 30 in in front of a wall), and so does a red LED strip on the wall's top seen from
// 20 in or closer. The position gate, with a pose the team trusts, turns those away (0 of the 540; the strip
// at every distance) — they are not where a Toggle's face is. It CANNOT turn away a red strip about a
// Toggle's length lying ON a Toggle's face line: a vertical plate there reads RED from about 22 in out with
// the gate off, and with it on from 22 to 30 in (named as that Toggle; farther, its position too vague for
// an 8-in fix). That is the limit a consumer plans for. A band's rows
// are taken as the face's apex and lower edge within kBoxEdgePx (A4 register HA-202: glare on the
// back-leaning face, rounded corners). And the image is OLDER than its read (HA-192): the bearing in a
// view is the robot's at the IMAGE's time — a consumer pairing it with the pose widens the bound
// (vision::boundAtImage).
//
// Every threshold is the TEAM's, from the bench: none has a library default, and an UNSET config is
// refused (toggleTrackerConfigProblem). PROS-free, sim-free, allocation-free; single-task. NO HARDWARE
// CLAIM: every rule here has run against the host camera model (sim/camera_plant.hpp, written with its
// own projection) and nothing else.

#include <algorithm>
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

/// A Toggle face's colour, or UNKNOWN.
enum class ToggleColour : std::uint8_t {
    Unknown = 0,  ///< no colour is sure (the view's `why` says why)
    Red,          ///< the red face
    Blue,         ///< the blue face
    Yellow,       ///< the yellow face
};

/// The colour as one ASCII word for a log line or a bench page: "RED", "BLUE", "YELLOW", "UNKNOWN".
[[nodiscard]] constexpr const char* toggleColourWord(ToggleColour c) noexcept {
    switch (c) {
        case ToggleColour::Red: return "RED";
        case ToggleColour::Blue: return "BLUE";
        case ToggleColour::Yellow: return "YELLOW";
        case ToggleColour::Unknown: return "UNKNOWN";
    }
    return "UNKNOWN";  // a byte outside the enum: never invents a colour
}

/// Why the tracker says what it says.
enum class ToggleWhy : std::uint8_t {
    Sure = 0,                ///< a colour, held for holdSeconds and alive
    NoRead,                  ///< no usable read yet
    CameraSilent,            ///< no usable read within aliveSeconds (unplugged, faulting, not configured, or not fed)
    ReadNotUsable,           ///< the latest read was a fault, torn or not configured: the hold restarts
    CameraFull,              ///< the latest read was POSSIBLY FULL (24 or more): a contradicting band may be missing
    NoBand,                  ///< no Toggle-coloured band passed the gates
    CutByImage,              ///< a Toggle-coloured band touches the image's edge (too close, or a partial band)
    MaybeBottomFace,         ///< the only band could be the Toggle's BOTTOM face (too close to see the field face)
    TwoColours,              ///< two different colours passed at once
    NotHeldYet,              ///< the colour has not held for holdSeconds
    CameraFrozen,            ///< the camera moved or turned and the band's box did not move at all
    ConflictingPartialBand,  ///< a band passed beside a band of ANOTHER colour the gates could not judge
    ColourAbove,             ///< a box of ANOTHER Toggle colour sits directly ABOVE the band
};

/// The reason in a few ASCII words, for a log line or a bench page.
[[nodiscard]] constexpr const char* toggleWhyWords(ToggleWhy w) noexcept {
    switch (w) {
        case ToggleWhy::Sure: return "sure";
        case ToggleWhy::NoRead: return "no read yet";
        case ToggleWhy::CameraSilent: return "camera silent";
        case ToggleWhy::ReadNotUsable: return "read not usable";
        case ToggleWhy::CameraFull: return "camera full";
        case ToggleWhy::NoBand: return "no band";
        case ToggleWhy::CutByImage: return "cut by the image";
        case ToggleWhy::MaybeBottomFace: return "maybe the bottom face";
        case ToggleWhy::TwoColours: return "two colours";
        case ToggleWhy::NotHeldYet: return "not held yet";
        case ToggleWhy::CameraFrozen: return "camera frozen?";
        case ToggleWhy::ConflictingPartialBand: return "another colour beside it, cut or maybe a bottom face";
        case ToggleWhy::ColourAbove: return "another colour directly above";
    }
    return "?";
}

/// The robot's three colour descriptor ids for the Toggle's faces — the colours the team taught the camera
/// (config::CameraConfig's settings). 0 = UNSET.
struct ToggleColourIds {
    std::uint8_t red = 0;     ///< the red face's descriptor id, 1..7
    std::uint8_t blue = 0;    ///< the blue face's descriptor id, 1..7
    std::uint8_t yellow = 0;  ///< the yellow face's descriptor id, 1..7
};

/// The bottom face's depth, seen as a band on the bottom's plane: its far edge at least this much farther
/// than its near edge, inches (the manual's ~2.05-in face, its rounded corners — generous; A4 register
/// HA-201).
inline constexpr double kBottomDepthLoIn = 1.0;
/// The bottom face's depth: at most this, inches (HA-201).
inline constexpr double kBottomDepthHiIn = 3.0;

/// Every number the tracker uses — the TEAM's, from the bench. Defaults are UNSET (0) and refused.
struct ToggleTrackerConfig {
    ToggleColourIds ids{};           ///< the three descriptor ids (distinct, each 1..7)
    double holdSeconds = 0.0;        ///< T: the same colour this long, on usable reads, before it is reported
    double aliveSeconds = 0.0;       ///< X: a usable read at most this old, or the view is "camera silent"
    double minAspect = 0.0;          ///< a band's width over its height, at least this (more than 1)
    double minLengthFraction = 0.0;  ///< a band not cut at a side: implied length at least this x 25.8 in
    double maxLengthFraction = 0.0;  ///< a band not cut at a side: implied length at most this x 25.8 in
    double minRangeIn = 0.0;         ///< the range window's near end: lens to the face, horizontal, inches
    double maxRangeIn = 0.0;         ///< the range window's far end, inches
    double bearingWindowDeg = 0.0;   ///< a band's whole bearing interval inside +-this, degrees
    double frozenMoveIn = 0.0;       ///< the LENS moved this far and the box did not move: "camera frozen?"
    double frozenTurnDeg = 0.0;      ///< the robot TURNED this far and the box did not move: "camera frozen?"
    /// The POSITION gate: a band counts only where a Toggle IS (override2026::kTogglePlaces), within this
    /// (plus the band's own fix radius) of the face's line, inches. 0 = the gate is OFF (every view says
    /// `positionChecked == false`) — the team turns it on only with a pose it trusts.
    double positionToleranceIn = 0.0;
    /// With the position gate on: it is skipped (the view says unchecked) for a read whose pose bound is
    /// looser than this, inches.
    double maxPoseToleranceIn = 0.0;
    /// With the position gate on: a band whose field position is vaguer than this (its fix radius) fails
    /// it — a vague position is no evidence of WHERE, inches.
    double maxFixRadiusIn = 0.0;
};

/// The first problem with a tracker config, or nullptr. Every number must be finite; the ids set,
/// distinct and 1..7; the hold and alive times, the window's ends, the bearing window and the two frozen
/// thresholds positive; `minAspect` above 1; the length fractions positive and ordered; the range window
/// ordered; the position tolerance not negative — and, with it on, the pose bound it trusts and the
/// vaguest position it accepts set.
[[nodiscard]] constexpr const char* toggleTrackerConfigProblem(const ToggleTrackerConfig& c) noexcept {
    if (c.ids.red == 0 || c.ids.blue == 0 || c.ids.yellow == 0) {
        return "a Toggle colour id is UNSET -- the descriptor ids the team taught the camera";
    }
    if (c.ids.red > hal::kMaxColourDescriptors || c.ids.blue > hal::kMaxColourDescriptors
        || c.ids.yellow > hal::kMaxColourDescriptors) {
        return "a Toggle colour id is outside 1..7";
    }
    if (c.ids.red == c.ids.blue || c.ids.red == c.ids.yellow || c.ids.blue == c.ids.yellow) {
        return "two Toggle colours share one descriptor id";
    }
    const double numbers[] = {c.holdSeconds,       c.aliveSeconds,      c.minAspect,         c.minLengthFraction,
                              c.maxLengthFraction, c.minRangeIn,        c.maxRangeIn,        c.bearingWindowDeg,
                              c.frozenMoveIn,      c.frozenTurnDeg,     c.positionToleranceIn, c.maxPoseToleranceIn,
                              c.maxFixRadiusIn};
    for (const double n : numbers) {
        if (!config::finiteNumber(n)) {
            return "a tracker number is not a finite number";
        }
    }
    if (!(c.holdSeconds > 0.0)) {
        return "holdSeconds UNSET -- how long a colour must hold before it is reported";
    }
    if (!(c.aliveSeconds > 0.0)) {
        return "aliveSeconds UNSET -- how old the newest usable read may be";
    }
    if (!(c.minAspect > 1.0)) {
        return "minAspect UNSET or not above 1 (a band is wider than tall)";
    }
    if (!(c.minLengthFraction > 0.0) || !(c.maxLengthFraction > c.minLengthFraction)) {
        return "length fractions UNSET or reversed";
    }
    if (!(c.minRangeIn > 0.0) || !(c.maxRangeIn > c.minRangeIn)) {
        return "range window UNSET or reversed";
    }
    if (!(c.bearingWindowDeg > 0.0)) {
        return "bearing window UNSET";
    }
    if (!(c.frozenMoveIn > 0.0)) {
        return "frozenMoveIn UNSET -- how far the lens moves before an unmoved box means a frozen camera";
    }
    if (!(c.frozenTurnDeg > 0.0)) {
        return "frozenTurnDeg UNSET -- how far the robot turns before an unmoved box means a frozen camera";
    }
    if (c.positionToleranceIn < 0.0) {
        return "position tolerance negative";
    }
    if (c.positionToleranceIn > 0.0 && !(c.maxPoseToleranceIn > 0.0)) {
        return "the position gate is on but the pose bound it trusts is UNSET";
    }
    if (c.positionToleranceIn > 0.0 && !(c.maxFixRadiusIn > 0.0)) {
        return "the position gate is on but the vaguest position it accepts is UNSET";
    }
    return nullptr;
}

/// How far a band got through the library's own gates (2's height rule, 3 and 4 — no team threshold
/// except, in the tracker, the length fractions).
enum class ToggleFaceGate : std::uint8_t {
    Explained = 0,  ///< some corner of the tolerance box explains it as a Toggle's field face
    TooThin,        ///< no taller than 2 x kBoxEdgePx (or an empty box): it cannot be measured
    Cut,            ///< it touches the image's top or bottom, or is cut at a side without spanning the centre
    NotExplained,   ///< no corner explains its two edges (and, when judged, its length) as a face
};

/// What the library measures of one colour band as a Toggle's field face — numbers, never a verdict that
/// needs a threshold the team has not typed (Bench Tests prints them beside the camera's raw records).
struct ToggleFaceMeasure {
    ToggleFaceGate gate = ToggleFaceGate::NotExplained;  ///< how far it got
    double aspect = 0.0;            ///< width over height (0 for an empty box)
    bool sideClipped = false;       ///< cut at the image's left or right edge (its length unmeasurable)
    /// Explained: the lens's horizontal distance to the face's LOWER EDGE, inches — a bound over EVERY camera
    /// of the tolerance box that could explain it, not only the corners that do (invalid otherwise; its far
    /// end infinite when a ray at some corner cannot reach its plane and no length bounds it).
    Interval range{};
    /// Explained and not side-clipped: the band's length at those ranges (its width +-2 x kBoxEdgePx over the
    /// focal lengths' extremes), inches (the Toggle is 25.8).
    Interval impliedLength{};
};

namespace detail {

/// The library's gates 2 (height), 3 and 4 for one band, corner by corner; with `judgeLength`, a corner
/// explains a band not cut at a side only when its length is `minFraction..maxFraction` of the Toggle's.
[[nodiscard]] inline ToggleFaceMeasure measureToggleFace(const hal::Detection& d, const CameraModel& m,
                                                         bool judgeLength, double minFraction,
                                                         double maxFraction) noexcept {
    ToggleFaceMeasure out{};
    const double w = d.width;
    const double h = d.height;
    if (!(w > 0.0) || !(h > 0.0)) {
        out.gate = ToggleFaceGate::TooThin;
        return out;
    }
    out.aspect = w / h;
    // A COLOUR band's top and bottom rows are taken as the face's apex and lower edge within kBoxEdgePx —
    // glare on a back-leaning face and rounded corners included (A4 register HA-202). A band no taller
    // than its two edges' jitter cannot be measured.
    if (h <= 2.0 * kBoxEdgePx) {
        out.gate = ToggleFaceGate::TooThin;
        return out;
    }
    if (d.top <= 1 || d.top + d.height >= hal::kVisionImageHeightPx - 1) {
        out.gate = ToggleFaceGate::Cut;
        return out;
    }
    out.sideClipped = d.left <= 1 || d.left + d.width >= hal::kVisionImageWidthPx - 1;
    if (out.sideClipped
        && !(d.left <= hal::kVisionImageWidthPx / 2 && d.left + d.width >= hal::kVisionImageWidthPx / 2)) {
        out.gate = ToggleFaceGate::Cut;
        return out;
    }
    const double u = d.left + w / 2.0;
    const override2026::ToggleFace& face = override2026::kToggleFace;
    const double length = face.lengthIn;
    const double wLo = std::fmax(1.0, w - 2.0 * kBoxEdgePx);
    const double wHi = w + 2.0 * kBoxEdgePx;
    const bool judge = !out.sideClipped && judgeLength;
    // Whether SOME corner explains it is decided corner by corner (below). The RANGE reported is a bound over
    // EVERY camera of the tolerance box that could explain it, not the hull of the corners that do (VIS1-A2's
    // review: an 11-in lens 64 in out, the true range outside the explaining corners' hull). Each edge's range
    // to its plane is monotone in each of the camera's numbers, so its extremes over the box are at corners.
    // When the two edges move TOGETHER in every number — the lens below both planes or above both (the pitch,
    // the height and fx then move both rays alike) and all four rows on one side of the principal row (fy
    // too) — the edge agreement's two ends are monotone as well, and the bound is their extremes over EVERY
    // corner whose rays meet the planes, explaining or not; otherwise each edge's extremes are taken apart.
    // The length's terms depend on fx alone: their extremes are fx's. A corner whose ray cannot reach its
    // plane (within half a degree of level, or pointing away) has cameras next to it whose range is as large
    // as you like: the far end is then only what the length allows (or none).
    const double lensLo = m.placement().height.value() - m.placement().heightTolerance.value();
    const double lensHi = m.placement().height.value() + m.placement().heightTolerance.value();
    const double rowA = d.top - kBoxEdgePx - m.nominal().cy;
    const double rowB = d.top + h + kBoxEdgePx - m.nominal().cy;
    const bool together = (lensHi < face.bottomIn || lensLo > face.topIn) && rowA * rowB > 0.0;
    double edgeLoMin = std::numeric_limits<double>::infinity();
    double edgeHiMax = 0.0;
    double rbMin = std::numeric_limits<double>::infinity();
    double rbMax = 0.0;
    double rtMin = std::numeric_limits<double>::infinity();
    double rtMax = 0.0;
    double fxMin = std::numeric_limits<double>::infinity();
    double fxMax = 0.0;
    bool bottomUnbounded = false;
    bool topUnbounded = false;
    bool explained = false;
    m.eachCorner(/*useHeight=*/true, [&](const CameraCorner& c) {
        fxMin = std::fmin(fxMin, c.fx);
        fxMax = std::fmax(fxMax, c.fx);
        double rt1 = 0.0;
        double rt2 = 0.0;
        const bool t1 = CameraModel::rangeToPlane(c, u, d.top - kBoxEdgePx, face.topIn, 0.5, rt1);
        const bool t2 = CameraModel::rangeToPlane(c, u, d.top + kBoxEdgePx, face.topIn, 0.5, rt2);
        for (const double r : {t1 ? rt1 : -1.0, t2 ? rt2 : -1.0}) {
            if (r < 0.0) {
                topUnbounded = true;
            } else {
                rtMin = std::fmin(rtMin, r);
                rtMax = std::fmax(rtMax, r);
            }
        }
        for (const double bottomPlane : {face.bottomIn, face.bottomHiIn}) {  // the lower edge's interval (HA-201)
            // Within ONE corner each edge's jitter is independent: each edge's range is an interval over its
            // own +-kBoxEdgePx (monotone in the row).
            double rb1 = 0.0;
            double rb2 = 0.0;
            const bool b1 = CameraModel::rangeToPlane(c, u, d.top + h - kBoxEdgePx, bottomPlane, 0.5, rb1);
            const bool b2 = CameraModel::rangeToPlane(c, u, d.top + h + kBoxEdgePx, bottomPlane, 0.5, rb2);
            for (const double r : {b1 ? rb1 : -1.0, b2 ? rb2 : -1.0}) {
                if (r < 0.0) {
                    bottomUnbounded = true;
                } else {
                    rbMin = std::fmin(rbMin, r);
                    rbMax = std::fmax(rbMax, r);
                }
            }
            if (!b1 || !b2 || !t1 || !t2) {
                continue;
            }
            // EDGE AGREEMENT: the lower edge's ranges that put the apex setBackLo..setBackHi behind it.
            double lo = std::fmax(std::fmin(rb1, rb2), std::fmin(rt1, rt2) - face.setBackHiIn - 0.25);
            double hi = std::fmin(std::fmax(rb1, rb2), std::fmax(rt1, rt2) - face.setBackLoIn + 0.25);
            edgeLoMin = std::fmin(edgeLoMin, lo);
            edgeHiMax = std::fmax(edgeHiMax, hi);
            if (lo > hi) {
                continue;
            }
            if (judge) {  // LENGTH: of those, the ranges where it is a Toggle's
                lo = std::fmax(lo, minFraction * length * c.fx / wHi);
                hi = std::fmin(hi, maxFraction * length * c.fx / wLo);
                if (lo > hi) {
                    continue;
                }
            }
            explained = true;
        }
    });
    if (explained) {
        const double inf = std::numeric_limits<double>::infinity();
        double lo = together ? edgeLoMin : std::fmax(rbMin, rtMin - face.setBackHiIn - 0.25);
        double hi = std::fmin(bottomUnbounded ? inf : rbMax, topUnbounded ? inf : rtMax - face.setBackLoIn + 0.25);
        if (together && !bottomUnbounded && !topUnbounded) {
            hi = std::fmin(hi, edgeHiMax);
        }
        if (judge) {
            lo = std::fmax(lo, minFraction * length * fxMin / wHi);
            hi = std::fmin(hi, maxFraction * length * fxMax / wLo);
        }
        out.range = Interval{true, lo, hi};
        if (!out.sideClipped) {
            out.impliedLength = Interval{true, wLo * lo / fxMax, wHi * hi / fxMin};
        }
    }
    out.gate = out.range.valid ? ToggleFaceGate::Explained : ToggleFaceGate::NotExplained;
    return out;
}

}  // namespace detail

/// What the library measures of ONE colour band as a Toggle's field face, with NO team threshold: whether
/// it can be measured, whether the image cuts it, the range at which some corner of the tolerance box
/// explains its two edges as the face's lower edge and apex, and the length that implies — a pure helper
/// for a bench page, which PRINTS what the tracker measures beside the camera's raw record (the tracker
/// itself also judges the length against the team's fractions).
[[nodiscard]] inline ToggleFaceMeasure explainAsToggleFace(const hal::Detection& d, const CameraModel& m) noexcept {
    return detail::measureToggleFace(d, m, /*judgeLength=*/false, 0.0, 0.0);
}

/// What the tracker reports.
struct ToggleView {
    ToggleColour colour = ToggleColour::Unknown;  ///< a colour only when sure
    ToggleWhy why = ToggleWhy::NoRead;            ///< why
    double heldSeconds = 0.0;                     ///< how long the current colour has held on usable reads, seconds
    /// The band's bearing interval (body frame, radians, left positive) — the robot's at the IMAGE's time
    /// (A4 register HA-192); valid when a band passed.
    Interval bearing{};
    /// The lens's horizontal distance to the face's lower edge, inches — a bound over EVERY camera of the
    /// tolerance box that explains the band; its far end infinite when a camera next to the box's corners
    /// cannot reach the lower edge's plane and the length does not bound it.
    Interval range{};
    bool positionChecked = false;                 ///< the band was found where a Toggle is (the position gate ran)
    int toggleIndex = -1;                         ///< which of override2026::kTogglePlaces, when checked; -1 otherwise
};

/// The tracker (header). Feed it every read (update); ask it at any time (view). Single-task.
class ToggleTracker {
public:
    /// PRECONDITION: toggleTrackerConfigProblem(cfg) == nullptr, and `model` built from a mount
    /// cameraModelProblem() accepts — a consumer asks both first and refuses to construct, naming them.
    ToggleTracker(const ToggleTrackerConfig& cfg, const CameraModel& model) noexcept : cfg_{cfg}, model_{model} {}

    /// One read, seen from `robotBound` — the caller's pose and its stated bound. The pose's MOTION is
    /// used for the frozen check (the lens's travel and the robot's turn); the whole bound for the
    /// optional position gate.
    void update(const hal::VisionFrame& f, const PoseBound& robotBound) noexcept {
        const math::Pose2d& robot = robotBound.pose;
        // The LENS's field position and the robot's heading: a robot turning in place barely moves its
        // centre, but its camera sweeps the scene — both are compared.
        const double th = robot.heading().radians();
        const double fwd = model_.placement().forward.value();
        const double lft = model_.placement().left.value();
        const double lensX = robot.x().value() + std::cos(th) * fwd - std::sin(th) * lft;
        const double lensY = robot.y().value() + std::sin(th) * fwd + std::cos(th) * lft;
        const bool usePosition = cfg_.positionToleranceIn > 0.0
                                 && robotBound.positionTolerance.value() <= cfg_.maxPoseToleranceIn;
        last_ = ToggleView{};
        if (!f.usable()) {  // positive evidence only: a fault breaks the hold
            holding_ = ToggleColour::Unknown;
            last_.why = ToggleWhy::ReadNotUsable;
            return;
        }
        lastOk_ = f.readAt.value();
        haveOk_ = true;
        if (f.truncated) {  // possibly full: a contradicting band may be among the ones never sent
            holding_ = ToggleColour::Unknown;
            last_.why = ToggleWhy::CameraFull;
            return;
        }
        const std::size_t n = std::min<std::size_t>(f.count, hal::kMaxVisionDetections);
        ToggleColour front = ToggleColour::Unknown;
        double frontRange = 0.0;
        bool cut = false;
        bool two = false;
        bool maybeBottom = false;
        unsigned partial = 0U;  // the colours of bands the gates could not judge, inside the window
        int frontPlace = -1;
        hal::Detection frontBox{};
        Interval fb{};
        Interval fr{};
        for (std::size_t i = 0; i < n; ++i) {
            const hal::Detection& d = f.detections[i];
            const ToggleColour c = colourOf(d);
            if (c == ToggleColour::Unknown) {
                continue;
            }
            Interval b{};
            Interval r{};
            Gate g = gate(d, b, r);
            if (g == Gate::Cut
                && touchesWindow(model_.bearing(d.left + d.width / 2.0, d.top + d.height / 2.0))) {
                partial |= bitOf(c);
            }
            if (g == Gate::Pass) {
                const Bottom bf = bottomFace(d, f, n);
                if (bf == Bottom::IsBottom) {
                    continue;  // the band above it is the face, judged on its own
                }
                if (bf == Bottom::CannotTell) {
                    maybeBottom = true;
                    if (touchesWindow(b)) {
                        partial |= bitOf(c);
                    }
                    continue;
                }
            }
            int place = -1;
            if (g == Gate::Pass && usePosition) {
                place = whereIs(robotBound, b, r);
                if (place < 0) {
                    g = Gate::Position;
                }
            }
            if (g == Gate::Cut) {
                cut = true;
            }
            if (g != Gate::Pass) {
                continue;
            }
            if (front != ToggleColour::Unknown && c != front) {
                two = true;  // never "pick one"
            }
            if (front == ToggleColour::Unknown || r.mid() < frontRange) {  // the same colour twice: the nearer
                front = c;
                frontRange = r.mid();
                frontPlace = place;
                frontBox = d;
                fb = b;
                fr = r;
            }
        }
        if (two) {
            holding_ = ToggleColour::Unknown;
            last_.why = ToggleWhy::TwoColours;
            return;
        }
        if (front != ToggleColour::Unknown && (partial & ~bitOf(front)) != 0U) {
            holding_ = ToggleColour::Unknown;
            last_.why = ToggleWhy::ConflictingPartialBand;
            return;
        }
        if (front != ToggleColour::Unknown && colourAbove(f, n, frontBox, front)) {
            holding_ = ToggleColour::Unknown;
            last_.why = ToggleWhy::ColourAbove;
            return;
        }
        if (front == ToggleColour::Unknown) {
            holding_ = ToggleColour::Unknown;
            last_.why = maybeBottom ? ToggleWhy::MaybeBottomFace : cut ? ToggleWhy::CutByImage : ToggleWhy::NoBand;
            return;
        }
        if (front != holding_ || !sameBand(fb, fr, frontPlace, th, lensX, lensY)) {
            // a new colour — or the same colour from ANOTHER band (another Toggle, a lookalike that became the
            // nearer one): the hold is the band's, not the colour's
            holding_ = front;
            holdSince_ = f.readAt.value();
            holdBox_ = frontBox;
            holdLensX_ = lensX;
            holdLensY_ = lensY;
            holdHeading_ = th;
            frozen_ = false;
        } else if (sameBox(frontBox, holdBox_)) {
            const double turned = std::fabs(std::remainder(th - holdHeading_, 2.0 * math::Angle::kPi));
            if (std::hypot(lensX - holdLensX_, lensY - holdLensY_) > cfg_.frozenMoveIn
                || turned > cfg_.frozenTurnDeg * kDegToRad) {
                frozen_ = true;  // the camera moved or turned, and the band did not move by a pixel
            }
        } else {  // the box moved
            if (frozen_) {  // a camera that RECOVERS is believed again — after a NEW hold
                frozen_ = false;
                holdSince_ = f.readAt.value();
            }
            holdBox_ = frontBox;
            holdLensX_ = lensX;
            holdLensY_ = lensY;
            holdHeading_ = th;
        }
        bandHeading_ = th;
        bandBearing_ = fb;
        bandRange_ = fr;
        bandPlace_ = frontPlace;
        bandLensX_ = lensX;
        bandLensY_ = lensY;
        last_.why = ToggleWhy::Sure;
        last_.bearing = fb;
        last_.range = fr;
        last_.colour = front;
        last_.positionChecked = usePosition;
        last_.toggleIndex = frontPlace;
    }

    /// What it believes at `nowSeconds` (the clock the reads are stamped with).
    [[nodiscard]] ToggleView view(double nowSeconds) const noexcept {
        ToggleView v{};
        if (!haveOk_) {
            v.why = ToggleWhy::NoRead;
            return v;
        }
        if (!(nowSeconds - lastOk_ <= cfg_.aliveSeconds)) {
            v.why = ToggleWhy::CameraSilent;
            return v;
        }
        v = last_;
        if (holding_ == ToggleColour::Unknown) {
            v.colour = ToggleColour::Unknown;
            return v;
        }
        if (frozen_) {
            v.colour = ToggleColour::Unknown;
            v.why = ToggleWhy::CameraFrozen;
            v.heldSeconds = 0.0;
            v.positionChecked = false;
            v.toggleIndex = -1;
            return v;
        }
        v.heldSeconds = lastOk_ - holdSince_;
        if (v.heldSeconds < cfg_.holdSeconds) {
            v.colour = ToggleColour::Unknown;
            v.why = ToggleWhy::NotHeldYet;
            return v;
        }
        v.colour = holding_;
        v.why = ToggleWhy::Sure;
        return v;
    }

private:
    enum class Gate : std::uint8_t { Pass, Shape, Cut, Explain, Where, Position };
    enum class Bottom : std::uint8_t { NotBottom, IsBottom, CannotTell };

    // Which Toggle a band at (bearing, range) can be — the index into kTogglePlaces, or -1.
    [[nodiscard]] int whereIs(const PoseBound& rb, const Interval& b, const Interval& r) const noexcept {
        const FieldFix fix = model_.fieldFix(rb, b, r);
        if (!fix.valid || !(fix.radius <= cfg_.maxFixRadiusIn)) {
            return -1;
        }
        const double half = override2026::kToggleFace.lengthIn / 2.0;
        const double slack = fix.radius + cfg_.positionToleranceIn;
        for (int i = 0; i < 4; ++i) {
            const override2026::TogglePlace& p = override2026::kTogglePlaces[i];
            const double dx = fix.x - p.x;
            const double dy = fix.y - p.y;
            const double across = dx * p.inX + dy * p.inY;  // into the field: + ; behind the wall: -
            const double along = -dx * p.inY + dy * p.inX;
            if (std::fabs(across) <= slack && std::fabs(along) <= half + slack) {
                return i;
            }
        }
        return -1;
    }

    [[nodiscard]] ToggleColour colourOf(const hal::Detection& d) const noexcept {
        if (d.kind != hal::DetectionKind::Colour) {
            return ToggleColour::Unknown;
        }
        if (d.id == cfg_.ids.red) {
            return ToggleColour::Red;
        }
        if (d.id == cfg_.ids.blue) {
            return ToggleColour::Blue;
        }
        if (d.id == cfg_.ids.yellow) {
            return ToggleColour::Yellow;
        }
        return ToggleColour::Unknown;
    }

    // Gate 5 (header). For each corner that explains `d` as the BOTTOM face (both edges on the bottom's
    // plane, the far edge kBottomDepthLoIn..kBottomDepthHiIn farther): where would its field face then be —
    // directly above, its apex? Out of the frame for any such corner: CannotTell; a Toggle-coloured band
    // sitting there: IsBottom; otherwise this corner rules it out. The planes: the lower edge's interval
    // (HA-201). NOT the flat bottom's own height (`flatBottomIn`, a quarter inch lower): the bottom
    // colour's region runs from the lower edge round the corner to the flat bottom, the depth window
    // (1..3 in) covers that, and a third plane only cost range — against a Toggle drawn that way, a lens
    // near the lower edge lost the colour at most distances, with no wrong colour gained or prevented
    // (VIS1-A2's tests: the real-shape Toggle; D9's table).
    [[nodiscard]] Bottom bottomFace(const hal::Detection& d, const hal::VisionFrame& f, std::size_t n) const noexcept {
        const override2026::ToggleFace& face = override2026::kToggleFace;
        const double u = d.left + d.width / 2.0;
        bool cannot = false;
        bool explained = false;
        model_.eachCorner(/*useHeight=*/true, [&](const CameraCorner& c) {
            for (const double bottomPlane : {face.bottomIn, face.bottomHiIn}) {
                double n1 = 0.0;
                double n2 = 0.0;
                double f1 = 0.0;
                double f2 = 0.0;
                if (!CameraModel::rangeToPlane(c, u, d.top - kBoxEdgePx, bottomPlane, 0.5, n1)
                    || !CameraModel::rangeToPlane(c, u, d.top + kBoxEdgePx, bottomPlane, 0.5, n2)
                    || !CameraModel::rangeToPlane(c, u, d.top + d.height - kBoxEdgePx, bottomPlane, 0.5, f1)
                    || !CameraModel::rangeToPlane(c, u, d.top + d.height + kBoxEdgePx, bottomPlane, 0.5, f2)) {
                    continue;
                }
                const double nLo = std::fmin(n1, n2);
                const double nHi = std::fmax(n1, n2);
                const double fLo = std::fmin(f1, f2);
                const double fHi = std::fmax(f1, f2);
                if (fHi - nLo < kBottomDepthLoIn || fLo - nHi > kBottomDepthHiIn) {
                    continue;
                }
                explained = true;
                // The apex of the face this would be the bottom of: nHi + the set-back, at the apex's height.
                double b = 0.0;
                double e = 0.0;
                CameraModel::ray(c, u, d.top, b, e);
                const double r = nHi + face.setBackHiIn;
                const double fwd = model_.placement().forward.value() + r * std::cos(b);
                const double lft = model_.placement().left.value() + r * std::sin(b);
                double pu = 0.0;
                double pv = 0.0;
                if (!model_.project(c, fwd, lft, face.topIn, pu, pv) || pv < 2.0 * kBoxEdgePx) {
                    cannot = true;  // the face it would belong to is out of the frame
                }
            }
        });
        if (!explained) {
            return Bottom::NotBottom;
        }
        // A Toggle-coloured band directly above it (columns overlapping, its bottom at d's top)?
        for (std::size_t i = 0; i < n; ++i) {
            const hal::Detection& o = f.detections[i];
            if (&o == &d || colourOf(o) == ToggleColour::Unknown) {
                continue;
            }
            const int overlap = std::min(o.left + o.width, d.left + d.width) - std::max(o.left, d.left);
            const int bottomOfO = o.top + o.height;
            // above it, touching or overlapping its upper half (the two faces share an edge; the boxes'
            // jitter can overlap them by a few pixels)
            if (overlap * 2 >= std::min(o.width, d.width) && o.top < d.top
                && bottomOfO >= d.top - 3.0 * kBoxEdgePx && bottomOfO <= d.top + d.height / 2) {
                return Bottom::IsBottom;
            }
        }
        return cannot ? Bottom::CannotTell : Bottom::NotBottom;
    }

    static unsigned bitOf(ToggleColour c) noexcept { return 1U << static_cast<unsigned>(c); }

    // Is a box of ANOTHER Toggle colour — whatever its gates said — directly ABOVE `band`: their columns
    // overlap, its top above band's top, its bottom row within 2 x kBoxEdgePx of band's top, or lower?
    [[nodiscard]] bool colourAbove(const hal::VisionFrame& f, std::size_t n, const hal::Detection& band,
                                   ToggleColour c) const noexcept {
        for (std::size_t i = 0; i < n; ++i) {
            const hal::Detection& d = f.detections[i];
            const ToggleColour oc = colourOf(d);
            if (oc == ToggleColour::Unknown || oc == c) {
                continue;
            }
            const bool columns = d.left < band.left + band.width && band.left < d.left + d.width;
            const bool above = d.top < band.top && d.top + d.height >= band.top - 2.0 * kBoxEdgePx;
            if (columns && above) {
                return true;
            }
        }
        return false;
    }

    // Any part of a bearing interval inside the window (the partial-band veto is conservative: overlap, not
    // containment). An invalid interval counts as inside — nothing says it is elsewhere.
    [[nodiscard]] bool touchesWindow(const Interval& b) const noexcept {
        const double win = cfg_.bearingWindowDeg * kDegToRad;
        return !b.valid || (b.hi >= -win && b.lo <= win);
    }

    // Is the band at (bearing `b`, range `r`, place `place`), seen from heading `th` with the lens at
    // (lensX, lensY), the band the hold was on at the previous read? Its FIELD direction from the lens (the
    // heading plus the bearing) must overlap the previous one's, widened by the angle the lens's travel since
    // then can turn a point at the nearer of the two ranges' near ends; its range must overlap the previous
    // one's, widened by that travel; and, with the position gate on, it must be the same Toggle place.
    [[nodiscard]] bool sameBand(const Interval& b, const Interval& r, int place, double th, double lensX,
                                double lensY) const noexcept {
        if (place != bandPlace_) {
            return false;
        }
        const double travel = std::hypot(lensX - bandLensX_, lensY - bandLensY_);
        const double nearest = std::fmin(r.lo, bandRange_.lo);
        const double swing = travel < nearest ? std::asin(travel / nearest) : math::Angle::kPi;
        const double apart = std::fabs(std::remainder((th + b.mid()) - (bandHeading_ + bandBearing_.mid()),
                                                      2.0 * math::Angle::kPi));
        if (!(apart <= (b.hi - b.lo) / 2.0 + (bandBearing_.hi - bandBearing_.lo) / 2.0 + swing)) {
            return false;
        }
        return !(r.lo > bandRange_.hi + travel) && !(r.hi < bandRange_.lo - travel);
    }

    static bool sameBox(const hal::Detection& a, const hal::Detection& b) noexcept {
        return a.left == b.left && a.top == b.top && a.width == b.width && a.height == b.height;
    }

    // Gates 2–4 and 6 for one band (header).
    Gate gate(const hal::Detection& d, Interval& bearing, Interval& range) const noexcept {
        if (d.width <= 0 || d.height <= 0) {
            return Gate::Shape;
        }
        if (static_cast<double>(d.width) / d.height < cfg_.minAspect) {
            return Gate::Shape;  // a side clip only shortens the width: still a floor
        }
        const ToggleFaceMeasure m =
            detail::measureToggleFace(d, model_, /*judgeLength=*/true, cfg_.minLengthFraction, cfg_.maxLengthFraction);
        switch (m.gate) {
            case ToggleFaceGate::TooThin: return Gate::Shape;
            case ToggleFaceGate::Cut: return Gate::Cut;
            case ToggleFaceGate::NotExplained: return Gate::Explain;
            case ToggleFaceGate::Explained: break;
        }
        range = m.range;
        bearing = model_.bearing(d.left + d.width / 2.0, d.top + d.height / 2.0);
        const double win = cfg_.bearingWindowDeg * kDegToRad;
        if (range.hi < cfg_.minRangeIn || range.lo > cfg_.maxRangeIn || !bearing.valid || bearing.lo < -win
            || bearing.hi > win) {
            return Gate::Where;
        }
        return Gate::Pass;
    }

    ToggleTrackerConfig cfg_;
    CameraModel model_;
    ToggleColour holding_ = ToggleColour::Unknown;
    double holdSince_ = 0.0;
    double lastOk_ = 0.0;
    bool haveOk_ = false;
    bool frozen_ = false;
    hal::Detection holdBox_{};
    double holdLensX_ = 0.0;
    double holdLensY_ = 0.0;
    double holdHeading_ = 0.0;
    double bandHeading_ = 0.0;   // the held band at the previous read: the robot's heading,
    Interval bandBearing_{};     // its bearing,
    Interval bandRange_{};       // its range,
    int bandPlace_ = -1;         // its Toggle place (-1: unchecked or none),
    double bandLensX_ = 0.0;     // and the lens's field position
    double bandLensY_ = 0.0;
    ToggleView last_{};
};

}  // namespace shulib::vision
