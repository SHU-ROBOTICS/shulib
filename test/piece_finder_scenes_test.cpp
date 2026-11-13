// vision::PieceFinder against SCENES from the INDEPENDENT host camera model (chunk VIS1-A2; the VIS1 brief's
// §1.13 and §2.2 "the finder" — its probes P5, P6's piece cases, P7 and the re-check's R6.1/R6.2, plus
// property sweeps written for this run).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   The library believes a lens 10 in up pitched 10 degrees down; the host camera model sees through a TRUE
//   camera moved inside the stated tolerances (camera_scene_rig.hpp's pieceTruth: 0.2 in, 0.8 degree,
//   0.7 degree of yaw, the field of view 72.8 x 64 against 74 x 63).
//   * The team lead's "2 or 3 yellow pins": AMBIGUOUS from afar, FOUND (the middle one) once close —
//     never a neighbour (F1); the named one taken -> NONE (F2); on a Goal -> RAISED, never taken by a
//     floor route (F3); lying -> never FOUND by a floor route (F4); beside the robot -> NOT IN VIEW (F5);
//     a loose pose -> AMBIGUOUS (F6); two in one tolerance -> AMBIGUOUS (F7); dead and torn cameras (F8);
//     the wrong class (F9).
//   * A second Pin cut by the image's bottom, certainly nearer than the disc, does not block FOUND (P1a);
//     an unmeasurable box AT the place makes it AMBIGUOUS (P1b); a POSSIBLY FULL read is UNKNOWN (P2); a
//     place whose top or floor leaves the frame is NOT IN VIEW (P3, P3b); withheld Objects are UNKNOWN,
//     never NONE (P7).
//   * PROPERTIES, swept across the picture through true cameras at the edges of the tolerance box: a
//     Pin standing on the FLOOR is never RAISED (the top face's allowance — the size range reads NEARER
//     than its near rim when the lens is above its top), and its fix holds its true centre; a Pin on a
//     Goal is never ON THE FLOOR, and when RAISED its fix holds its true centre; a Pin cut by the
//     image's bottom is never nearer than its `nearerThanIn` says.
//   * The image's age (R6.1, R6.2): turning 180 deg/s, or driving 60 in/s past a Pin — un-widened, the fix
//     misses the true Pin at the longer lags (the finding, pinned); widened by boundAtImage(), never.

#include "doctest.h"

#include <cmath>
#include <cstdint>

#include "camera_scene_rig.hpp"

namespace {

namespace hal = camrig::hal;
namespace sim = camrig::sim;
namespace vision = camrig::vision;
using camrig::Time;
using sim::CameraPlant;
using sim::CameraState;
using vision::PieceFinder;
using vision::PieceTarget;
using vision::PieceVerdict;
using vision::PieceWhy;
using vision::Posture;

constexpr double kPi = camrig::kPi;

vision::CameraModel pieceModel() { return camrig::model(10.0, -10.0); }

// The robot at the origin facing +x: `secs` of reads.
PieceFinder::Result lookFromOrigin(CameraPlant& plant, PieceFinder& f, double& t, double secs) {
    return camrig::look(plant, f, 0.0, 1.0, t, secs);
}

}  // namespace

// ─── P5: the team lead's scenes ───────────────────────────────────────────────────────────────────

TEST_CASE("piece scenes F1: three yellow Pins a foot apart, the route names the middle one within 6 in (half the "
          "spacing): AMBIGUOUS from afar, FOUND -- the middle one, never a neighbour -- once close, NOT IN VIEW "
          "closer still") {
    CameraPlant plant{camrig::pieceTruth(), 3};
    plant.setJitterPx(1.0);
    for (const double y : {-12.0, 0.0, 12.0}) {
        plant.addElement(camrig::pin(60.0, y));
    }
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(60.0, 0.0, 6.0));
    double t = 0.0;
    bool sawFound = false;
    bool sawAmbiguousFar = false;
    double nearestFound = 99.0;
    double farthestFound = -1.0;
    for (double rx = 0.0; rx <= 40.0; rx += 1.0) {
        CAPTURE(rx);
        const PieceFinder::Result r = camrig::look(plant, f, rx, 1.0, t, 0.25);
        CHECK(r.verdict != PieceVerdict::None);
        if (r.verdict == PieceVerdict::Found) {
            CHECK(std::fabs(r.sighting.fix.y) < 6.0);  // the middle one, never a neighbour
            CHECK(std::hypot(r.sighting.fix.x - 60.0, r.sighting.fix.y) <= r.sighting.fix.radius);
            sawFound = true;
            nearestFound = std::fmin(nearestFound, 60.0 - rx - camrig::kLensFwd);
            farthestFound = std::fmax(farthestFound, 60.0 - rx - camrig::kLensFwd);
        }
        if (rx <= 8.0 && r.verdict == PieceVerdict::Ambiguous) {
            sawAmbiguousFar = true;
        }
    }
    MESSAGE("F1: FOUND (the middle Pin) with the lens " << nearestFound << " to " << farthestFound << " in from it");
    CHECK(sawFound);
    CHECK(sawAmbiguousFar);
}

TEST_CASE("piece scenes F2: the named Pin already taken, its neighbours left: NONE, never a neighbour") {
    CameraPlant plant{camrig::pieceTruth(), 5};
    for (const double y : {-12.0, 12.0}) {
        plant.addElement(camrig::pin(60.0, y));
    }
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(60.0));
    double t = 0.0;
    const PieceFinder::Result r = camrig::look(plant, f, 30.0, 1.0, t, 0.3);
    CHECK(r.verdict == PieceVerdict::None);
    CHECK(r.why == PieceWhy::NothingThere);
}

TEST_CASE("piece scenes F3: the named place holds a yellow Pin ON A GOAL (base 5.8 in up): RAISED -- NONE 'wrong "
          "posture inside' for a floor route, FOUND for a raised one") {
    for (const Posture want : {Posture::OnFloor, Posture::Raised}) {
        CAPTURE(static_cast<int>(want));
        CameraPlant plant{camrig::pieceTruth(), 7};
        plant.addElement(camrig::pin(60.0, 0.0, 5.8));
        PieceFinder f{camrig::fcfg(), pieceModel()};
        f.setTarget(camrig::yellowPinAt(60.0, 0.0, 6.0, want));
        double t = 0.0;
        const PieceFinder::Result r = camrig::look(plant, f, 30.0, 1.0, t, 0.3);
        if (want == Posture::OnFloor) {
            CHECK(r.verdict == PieceVerdict::None);
            CHECK(r.why == PieceWhy::WrongPostureInside);
        } else {
            CHECK(r.verdict == PieceVerdict::Found);
            CHECK(r.sighting.posture == Posture::Raised);
        }
    }
}

TEST_CASE("piece scenes F4: a LYING Pin (across the view; the host model's lying Pin runs from (56, 0) to (56, 6.5)) "
          "is LYING at every distance, its fix holding its true centre -- a floor route never takes it, a lying "
          "route never answers NONE") {
    const double cx = 56.0;
    const double cy = 3.25;  // the lying Pin's centre: half its length along +y
    int lyingFound = 0;
    for (const Posture want : {Posture::OnFloor, Posture::Lying}) {
        CAPTURE(static_cast<int>(want));
        CameraPlant plant{camrig::pieceTruth(), 9};
        sim::Element e = camrig::pin(56.0, 0.0, 0.0, true);
        e.lyingHeadingRad = kPi / 2.0;
        plant.addElement(e);
        PieceFinder f{camrig::fcfg(), pieceModel()};
        f.setTarget(camrig::yellowPinAt(cx, cy, 8.0, want));
        double t = 0.0;
        for (double rx = 0.0; rx <= 34.0; rx += 2.0) {
            CAPTURE(rx);
            const PieceFinder::Result r = camrig::look(plant, f, rx, 1.0, t, 0.25);
            const hal::VisionFrame fr = plant.read(rx, 0.0, 0.0, Time{t});
            REQUIRE(fr.count == 1);
            const vision::PieceSighting s =
                vision::sightPiece(fr.detections[0], pieceModel(), camrig::poseAt(rx, 0.0, 0.0), camrig::fcfg());
            CHECK(s.posture == Posture::Lying);
            CHECK(std::hypot(s.fix.x - cx, s.fix.y - cy) <= s.fix.radius);
            if (want == Posture::OnFloor) {
                CHECK(r.verdict != PieceVerdict::Found);
                if (r.verdict == PieceVerdict::None) {
                    CHECK(r.why == PieceWhy::WrongPostureInside);
                }
            } else {
                CHECK(r.verdict != PieceVerdict::None);
                lyingFound += r.verdict == PieceVerdict::Found ? 1 : 0;
            }
        }
    }
    MESSAGE("F4: the lying route FOUND it at " << lyingFound << " of 18 positions (its fix is wide: whichever way "
                                               << "it lies, the centre is up to half its length beyond its near end)");
}

TEST_CASE("piece scenes F5: the named place beside the robot (40 in to its left): NOT IN VIEW, never NONE") {
    CameraPlant plant{camrig::pieceTruth(), 11};
    plant.addElement(camrig::pin(60.0, 0.0));
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(60.0, 40.0));
    double t = 0.0;
    const PieceFinder::Result r = camrig::look(plant, f, 30.0, 1.0, t, 0.3);
    CHECK(r.verdict == PieceVerdict::NotInView);
    CHECK(r.why == PieceWhy::OutOfView);
}

TEST_CASE("piece scenes F6: a LOOSE pose (+-6 in): AMBIGUOUS, the error larger than the tolerance") {
    CameraPlant plant{camrig::pieceTruth(), 13};
    plant.addElement(camrig::pin(60.0, 0.0));
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(60.0));
    double t = 0.0;
    const PieceFinder::Result r = camrig::look(plant, f, 20.0, 6.0, t, 0.3);
    CHECK(r.verdict == PieceVerdict::Ambiguous);
    CHECK(r.why == PieceWhy::ErrorLargerThanTolerance);
}

TEST_CASE("piece scenes F7: two Pins 3 in apart inside one 8-in tolerance: AMBIGUOUS, never one of them") {
    CameraPlant plant{camrig::pieceTruth(), 17};
    plant.addElement(camrig::pin(60.0, -1.5));
    plant.addElement(camrig::pin(60.0, 1.5));
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(60.0, 0.0, 8.0));
    double t = 0.0;
    const PieceFinder::Result r = camrig::look(plant, f, 24.0, 0.5, t, 0.3);
    CHECK(r.verdict == PieceVerdict::Ambiguous);
    CHECK(r.inside + r.border >= 2);
}

TEST_CASE("piece scenes F8: a torn read is no picture (UNKNOWN 'read not usable', never NONE) and restarts the "
          "hold; an unplugged camera goes silent") {
    CameraPlant plant{camrig::pieceTruth(), 19};
    plant.addElement(camrig::pin(60.0, 0.0));
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(60.0, 0.0, 6.0));
    double t = 0.0;
    REQUIRE(camrig::look(plant, f, 30.0, 1.0, t, 0.3).verdict == PieceVerdict::Found);
    plant.setState(CameraState::Torn);
    const PieceFinder::Result torn = camrig::look(plant, f, 30.0, 1.0, t, 0.033);
    CHECK(torn.verdict == PieceVerdict::Unknown);
    CHECK(torn.why == PieceWhy::ReadNotUsable);
    plant.setState(CameraState::Normal);
    const PieceFinder::Result again = camrig::look(plant, f, 30.0, 1.0, t, 0.066);
    CHECK(again.verdict == PieceVerdict::Unknown);
    CHECK(again.why == PieceWhy::NotHeldYet);
    CHECK(camrig::look(plant, f, 30.0, 1.0, t, 0.2).verdict == PieceVerdict::Found);
    plant.setState(CameraState::Unplugged);
    const PieceFinder::Result first = camrig::look(plant, f, 30.0, 1.0, t, 0.033);
    CHECK(first.verdict == PieceVerdict::Unknown);
    CHECK(first.why == PieceWhy::ReadNotUsable);
    const PieceFinder::Result r = camrig::look(plant, f, 30.0, 1.0, t, 0.2);
    CHECK(r.verdict == PieceVerdict::Unknown);
    CHECK(r.why == PieceWhy::CameraSilent);
}

TEST_CASE("piece scenes F9: a CUP stands where the route names a yellow Pin: NONE (the class decides the type)") {
    CameraPlant plant{camrig::pieceTruth(), 23};
    sim::Element cup = camrig::pin(60.0, 0.0);
    cup.classId = camrig::kCup;
    cup.diameter = vision::override2026::kCupSize.diameterIn;
    plant.addElement(cup);
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(60.0, 0.0, 6.0));
    double t = 0.0;
    const PieceFinder::Result r = camrig::look(plant, f, 30.0, 1.0, t, 0.3);
    CHECK(r.verdict == PieceVerdict::None);
    // the same Cup, named by a route that takes a Cup: FOUND
    PieceFinder g{camrig::fcfg(), pieceModel()};
    PieceTarget cupTarget = camrig::yellowPinAt(60.0, 0.0, 6.0);
    cupTarget.classMask = 1U << camrig::kCup;
    g.setTarget(cupTarget);
    double t2 = 0.0;
    CHECK(camrig::look(plant, g, 30.0, 1.0, t2, 0.3).verdict == PieceVerdict::Found);
}

// ─── P6's piece cases, P7 ─────────────────────────────────────────────────────────────────────────

TEST_CASE("piece scenes P1a: a second yellow Pin right in front, CUT by the image's bottom (its base below the "
          "frame): bounded NEARER than the target's disc, so the named Pin is still FOUND") {
    CameraPlant plant{camrig::pieceTruth(), 4};
    plant.addElement(camrig::pin(30.0, 0.0));
    plant.addElement(camrig::pin(14.5, 0.6));  // the lens 7.5 in away: its base is below the picture
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(30.0, 0.0, 6.0));
    double t = 0.0;
    const PieceFinder::Result r = lookFromOrigin(plant, f, t, 0.4);
    CHECK(r.verdict == PieceVerdict::Found);
    CHECK(r.sighting.fix.x > 25.0);
}

TEST_CASE("piece scenes P1b: an UNMEASURABLE box of the right class AT the named place (two Pins merged into one "
          "wide box): AMBIGUOUS 'unmeasurable inside', never NONE and never FOUND") {
    CameraPlant plant{camrig::pieceTruth(), 4};
    sim::Element merged = camrig::pin(30.0, 0.0);
    merged.diameter = 7.0;  // ~1.1 wide per tall: between standing and lying -> UNSURE
    plant.addElement(merged);
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(30.0));
    double t = 0.0;
    const PieceFinder::Result r = lookFromOrigin(plant, f, t, 0.4);
    CHECK(r.verdict == PieceVerdict::Ambiguous);
    CHECK(r.why == PieceWhy::UnmeasurableInside);
}

TEST_CASE("piece scenes P2: a POSSIBLY FULL read: UNKNOWN 'camera full', even with the named Pin alone in its "
          "place") {
    CameraPlant plant{camrig::pieceTruth(), 4};
    plant.addElement(camrig::pin(40.0, 0.0));
    for (int i = 0; i < 23; ++i) {  // 23 more pieces, each firing two colour halves: past 24 detections
        sim::Element e = camrig::pin(80.0 + i, -30.0 + 2.5 * i);
        e.lowerColourId = 1;
        e.upperColourId = 3;
        plant.addElement(e);
    }
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(40.0));
    double t = 0.0;
    const PieceFinder::Result r = lookFromOrigin(plant, f, t, 0.3);
    CHECK(r.verdict == PieceVerdict::Unknown);
    CHECK(r.why == PieceWhy::CameraFull);
}

TEST_CASE("piece scenes P3: the named Pin so close its TOP leaves the frame: NOT IN VIEW, never NONE") {
    CameraPlant plant{camrig::pieceTruth(), 4};
    plant.addElement(camrig::pin(14.0, 0.0));
    PieceFinder f{camrig::fcfg(), pieceModel()};
    f.setTarget(camrig::yellowPinAt(14.0, 0.0, 3.0));
    double t = 0.0;
    CHECK(lookFromOrigin(plant, f, t, 0.3).verdict == PieceVerdict::NotInView);
}

TEST_CASE("piece scenes P3b: a camera 5 in up pitched DOWN 30 degrees -- the named Pin's FLOOR is in view, its TOP "
          "is not: NOT IN VIEW (the top is checked too), never NONE or AMBIGUOUS") {
    sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd;
    tc.height = 5.0;
    tc.pitchDeg = -30.0;
    CameraPlant plant{tc, 4};
    plant.addElement(camrig::pin(17.0, 0.0));  // the lens 10 in away
    PieceFinder f{camrig::fcfg(), camrig::model(5.0, -30.0)};
    f.setTarget(camrig::yellowPinAt(17.0, 0.0, 2.0));
    double t = 0.0;
    CHECK(lookFromOrigin(plant, f, t, 0.3).verdict == PieceVerdict::NotInView);
}

TEST_CASE("piece scenes P7: a read whose Objects were WITHHELD (the model not confirmed by name) is UNKNOWN to the "
          "finder -- never NONE -- for every check that is not Matches") {
    for (const hal::ModelCheck mc : {hal::ModelCheck::Different, hal::ModelCheck::Unreadable,
                                     hal::ModelCheck::NotAsked}) {
        CAPTURE(static_cast<int>(mc));
        CameraPlant plant{camrig::pieceTruth(), 4};
        plant.addElement(camrig::pin(40.0, 0.0));
        plant.setModel(mc);
        PieceFinder f{camrig::fcfg(), pieceModel()};
        f.setTarget(camrig::yellowPinAt(40.0));
        double t = 0.0;
        const PieceFinder::Result r = lookFromOrigin(plant, f, t, 0.3);
        CHECK(r.verdict == PieceVerdict::Unknown);
        CHECK(r.why == PieceWhy::ModelNotConfirmed);
    }
}

// ─── properties across the picture ────────────────────────────────────────────────────────────────

namespace {

// The true cameras the properties look through: each at an edge of the believed tolerance box (+-0.25 in,
// +-1 degree, the focal lengths within 5 %), the lens `h` up pitched `pitch`.
sim::TrueCamera edgeTruth(double h, double pitch, int k) {
    sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd + ((k & 1) != 0 ? 0.2 : -0.2);
    tc.height = h + ((k & 2) != 0 ? 0.24 : -0.24);
    tc.pitchDeg = pitch + ((k & 4) != 0 ? 0.95 : -0.95);
    tc.yawDeg = (k & 8) != 0 ? 0.95 : -0.95;
    // fx and fy each 4.5 % off, in their own directions (a smaller field of view is a longer focal length)
    const double fxScale = (k & 16) != 0 ? 1.045 : 0.955;
    const double fyScale = (k & 32) != 0 ? 1.045 : 0.955;
    tc.hfovDeg = 2.0 * std::atan(std::tan(37.0 * kPi / 180.0) / fxScale) * 180.0 / kPi;
    tc.vfovDeg = 2.0 * std::atan(std::tan(31.5 * kPi / 180.0) / fyScale) * 180.0 / kPi;
    return tc;
}

struct Sweep {
    int seen = 0;
    int onFloor = 0;
    int raised = 0;
    int lying = 0;
    int unsure = 0;
    int fixMissed = 0;
};

// One Pin at a time, base `baseZ` up, seen through each of 64 true cameras from a robot at the origin
// facing +x: every unclipped sighting's posture tallied, and every fix checked against the true centre.
Sweep sweepPins(double h, double pitch, double baseZ) {
    Sweep sw{};
    const vision::CameraModel m = camrig::model(h, pitch);
    const vision::PieceFinderConfig cfg = camrig::fcfg();
    const vision::PoseBound pb = camrig::poseAt(0.0, 0.0, 0.0, 0.5, 0.5);
    for (int k = 0; k < 64; k += 3) {
        for (double range = 14.0; range <= 66.0; range += 4.0) {
            for (double bearDeg = -32.0; bearDeg <= 32.0; bearDeg += 8.0) {
                const double px = camrig::kLensFwd + range * std::cos(bearDeg * kPi / 180.0);
                const double py = range * std::sin(bearDeg * kPi / 180.0);
                CameraPlant plant{edgeTruth(h, pitch, k), static_cast<std::uint32_t>(k + 1)};
                plant.setJitterPx(1.0);
                plant.addElement(camrig::pin(px, py, baseZ));
                const hal::VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
                for (std::uint8_t i = 0; i < fr.count; ++i) {
                    const hal::Detection& d = fr.detections[i];
                    if (d.kind != hal::DetectionKind::Object || d.clipped()) {
                        continue;
                    }
                    ++sw.seen;
                    const vision::PieceSighting s = vision::sightPiece(d, m, pb, cfg);
                    switch (s.posture) {
                        case Posture::OnFloor: ++sw.onFloor; break;
                        case Posture::Raised: ++sw.raised; break;
                        case Posture::Lying: ++sw.lying; break;
                        case Posture::Unsure: ++sw.unsure; break;
                    }
                    if (s.fix.valid && std::hypot(s.fix.x - px, s.fix.y - py) > s.fix.radius) {
                        ++sw.fixMissed;
                        MESSAGE("fix missed: lens " << h << " pitch " << pitch << " base " << baseZ << " k " << k
                                                    << " range " << range << " bearing " << bearDeg << " posture "
                                                    << static_cast<int>(s.posture) << " err "
                                                    << std::hypot(s.fix.x - px, s.fix.y - py) << " radius "
                                                    << s.fix.radius);
                    }
                }
            }
        }
    }
    return sw;
}

}  // namespace

TEST_CASE("piece scenes PROPERTY: a Pin standing on the FLOOR, across the picture, through true cameras at the "
          "edges of the tolerance box, lenses below AND above its top: never RAISED, and every fix holds its "
          "true centre") {
    struct Mount {
        double h;
        double pitch;
    };
    for (const Mount mt : {Mount{6.0, 0.0}, Mount{10.0, -10.0}, Mount{10.0, 0.0}, Mount{14.0, -15.0},
                           Mount{18.0, -20.0}}) {
        CAPTURE(mt.h);
        CAPTURE(mt.pitch);
        const Sweep sw = sweepPins(mt.h, mt.pitch, 0.0);
        MESSAGE("floor Pins, lens " << mt.h << " in pitch " << mt.pitch << ": seen " << sw.seen << ", ON-FLOOR "
                                    << sw.onFloor << ", RAISED " << sw.raised << ", UNSURE " << sw.unsure
                                    << ", LYING " << sw.lying);
        CHECK(sw.seen > 200);
        CHECK(sw.raised == 0);
        CHECK(sw.lying == 0);
        CHECK(sw.fixMissed == 0);
        if (mt.h <= 10.0) {
            CHECK(sw.onFloor * 20 >= sw.seen * 19);  // D9's mounts: nearly every one placed on the floor
        } else {
            CHECK(sw.onFloor > 0);  // a high lens cannot rule out a Goal far out: UNSURE, the honest cost
        }
    }
}

TEST_CASE("piece scenes PROPERTY: a Pin on a GOAL (base 5.8 in up), across the picture, through true cameras at "
          "the edges of the tolerance box: never ON THE FLOOR, and every RAISED fix holds its true centre") {
    struct Mount {
        double h;
        double pitch;
    };
    for (const Mount mt : {Mount{10.0, -10.0}, Mount{10.0, 0.0}, Mount{14.0, -15.0}, Mount{18.0, -20.0}}) {
        CAPTURE(mt.h);
        CAPTURE(mt.pitch);
        const Sweep sw = sweepPins(mt.h, mt.pitch, 5.8);
        MESSAGE("Pins on a Goal, lens " << mt.h << " in pitch " << mt.pitch << ": seen " << sw.seen << ", RAISED "
                                        << sw.raised << ", ON-FLOOR " << sw.onFloor << ", UNSURE " << sw.unsure);
        CHECK(sw.seen > 200);
        CHECK(sw.onFloor == 0);
        CHECK(sw.fixMissed == 0);
        CHECK(sw.raised * 2 >= sw.seen);
    }
}

TEST_CASE("piece scenes PROPERTY: a Pin LYING on the floor at every heading, across the picture, through true "
          "cameras at the edges of the tolerance box, five mounts: never 'standing on the floor', and every fix "
          "holds its true centre (VIS1-A2's review: a Pin lying near END-ON read ON-FLOOR, its fix far from it)") {
    struct Mount {
        double h;
        double pitch;
    };
    for (const Mount mt : {Mount{6.0, 0.0}, Mount{10.0, 0.0}, Mount{10.0, -10.0}, Mount{14.0, -15.0},
                           Mount{18.0, -20.0}}) {
        CAPTURE(mt.h);
        CAPTURE(mt.pitch);
        const vision::CameraModel m = camrig::model(mt.h, mt.pitch);
        const vision::PoseBound pb = camrig::poseAt(0.0, 0.0, 0.0, 0.5, 0.5);
        int seen = 0;
        int onFloor = 0;
        int missed = 0;
        for (int k = 0; k < 64; k += 3) {
            for (double range = 8.0; range <= 64.0; range += 4.0) {
                for (double bearDeg = -32.0; bearDeg <= 32.0; bearDeg += 8.0) {
                    const double px = camrig::kLensFwd + range * std::cos(bearDeg * kPi / 180.0);
                    const double py = range * std::sin(bearDeg * kPi / 180.0);
                    CameraPlant plant{edgeTruth(mt.h, mt.pitch, k), static_cast<std::uint32_t>(k + 5)};
                    plant.setJitterPx(1.0);
                    sim::Element e = camrig::pin(px, py, 0.0, true);
                    e.lyingHeadingRad = 0.3 * k;  // every heading, END-ON among them
                    plant.addElement(e);
                    const hal::VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
                    for (std::uint8_t i = 0; i < fr.count; ++i) {
                        const hal::Detection& d = fr.detections[i];
                        if (d.kind != hal::DetectionKind::Object || d.clipped()) {
                            continue;
                        }
                        ++seen;
                        const vision::PieceSighting s = vision::sightPiece(d, m, pb, camrig::fcfg());
                        onFloor += s.posture == Posture::OnFloor ? 1 : 0;
                        const double cx = px + std::cos(e.lyingHeadingRad) * 3.25;  // its centre: half a length on
                        const double cy = py + std::sin(e.lyingHeadingRad) * 3.25;
                        if (s.fix.valid && std::hypot(s.fix.x - cx, s.fix.y - cy) > s.fix.radius) {
                            ++missed;
                        }
                    }
                }
            }
        }
        MESSAGE("lying Pins, lens " << mt.h << " in pitch " << mt.pitch << ": seen " << seen << ", ON-FLOOR "
                                    << onFloor << ", a fix missing its centre " << missed);
        CHECK(seen > 200);
        CHECK(onFloor == 0);
        CHECK(missed == 0);
    }
}

TEST_CASE("piece scenes PROPERTY: a Pin cut ONLY by the image's bottom edge is never nearer than its "
          "nearerThanIn says (the bound that lets a nearer Pin be ignored)") {
    const vision::CameraModel m = pieceModel();
    int bounded = 0;
    for (int k = 0; k < 64; k += 3) {
        for (double range = 3.0; range <= 14.0; range += 0.5) {
            for (double bearDeg = -24.0; bearDeg <= 24.0; bearDeg += 6.0) {
                const double px = camrig::kLensFwd + range * std::cos(bearDeg * kPi / 180.0);
                const double py = range * std::sin(bearDeg * kPi / 180.0);
                CameraPlant plant{edgeTruth(10.0, -10.0, k), static_cast<std::uint32_t>(k + 7)};
                plant.setJitterPx(1.0);
                plant.addElement(camrig::pin(px, py));
                const hal::VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
                for (std::uint8_t i = 0; i < fr.count; ++i) {
                    const vision::PieceSighting s =
                        vision::sightPiece(fr.detections[i], m, camrig::poseAt(0.0, 0.0, 0.0), camrig::fcfg());
                    if (!(s.nearerThanIn > 0.0)) {
                        continue;
                    }
                    ++bounded;
                    CHECK(s.posture == Posture::Unsure);
                    CHECK_FALSE(s.fix.valid);
                    CHECK(range < s.nearerThanIn);  // the lens to the Pin's centre, horizontally
                }
            }
        }
    }
    MESSAGE("bottom-cut Pins bounded: " << bounded);
    CHECK(bounded > 50);
}

// ─── R6.1 / R6.2: the image's age ─────────────────────────────────────────────────────────────────

namespace {

// One lagged read: the host model sees the robot where it WAS `lag` s before; the finder is handed the pose
// at read completion, widened by boundAtImage() or not. False when the Pin was not measured.
bool lagged(double lag, double speedInPerS, double turnDegPerS, bool widen, double pinY, double& err,
            double& radius) {
    CameraPlant plant{camrig::truth(10.0), 21};
    plant.setJitterPx(1.0);
    sim::Element pinEl = camrig::pin(36.0, pinY);
    plant.addElement(pinEl);
    const vision::CameraModel m = camrig::model(10.0);
    const double hThen = -turnDegPerS * kPi / 180.0 * lag;
    const double xThen = -speedInPerS * lag;
    const hal::VisionFrame f = plant.read(xThen, 0.0, hThen, Time{1.0});
    vision::PoseBound b = camrig::poseAt(0.0, 0.0, 0.0);
    if (widen) {
        b = vision::boundAtImage(b, speedInPerS, turnDegPerS, lag);
    }
    vision::PieceFinderConfig fc = camrig::fcfg();
    fc.standingAspect = 0.6;
    fc.lyingAspect = 2.0;
    for (std::uint8_t i = 0; i < f.count; ++i) {
        if (f.detections[i].kind != hal::DetectionKind::Object) {
            continue;
        }
        const vision::PieceSighting s = vision::sightPiece(f.detections[i], m, b, fc);
        if (!s.fix.valid) {
            return false;
        }
        err = std::hypot(s.fix.x - pinEl.x, s.fix.y - pinEl.y);
        radius = s.fix.radius;
        return true;
    }
    return false;
}

}  // namespace

TEST_CASE("piece scenes R6.1: turning 180 deg/s -- the bound at READ completion misses the true Pin once the image "
          "is ~66 ms old (the finding, pinned); the bound AT THE IMAGE (boundAtImage) always holds it") {
    int missedUnwidened = 0;
    for (const double lagMs : {0.0, 33.0, 66.0, 100.0, 150.0}) {
        CAPTURE(lagMs);
        double e0 = 0.0;
        double r0 = 0.0;
        double e1 = 0.0;
        double r1 = 0.0;
        REQUIRE(lagged(lagMs / 1000.0, 0.0, 180.0, false, 0.0, e0, r0));
        REQUIRE(lagged(lagMs / 1000.0, 0.0, 180.0, true, 0.0, e1, r1));
        missedUnwidened += e0 > r0 ? 1 : 0;
        CHECK(e1 <= r1);
    }
    CHECK(missedUnwidened >= 2);
}

TEST_CASE("piece scenes R6.2: driving 60 in/s past a Pin 36 in ahead and 14 in aside -- un-widened, the fix misses "
          "it at the longer lags; widened by boundAtImage, it holds it at every lag") {
    int missedUnwidened = 0;
    for (const double lagMs : {33.0, 66.0, 100.0, 150.0}) {
        CAPTURE(lagMs);
        double e0 = 0.0;
        double r0 = 0.0;
        double e1 = 0.0;
        double r1 = 0.0;
        REQUIRE(lagged(lagMs / 1000.0, 60.0, 0.0, false, 14.0, e0, r0));
        REQUIRE(lagged(lagMs / 1000.0, 60.0, 0.0, true, 14.0, e1, r1));
        missedUnwidened += e0 > r0 ? 1 : 0;
        CHECK(e1 <= r1);
    }
    CHECK(missedUnwidened >= 1);
}

TEST_CASE("piece scenes: two more bounds LANDED (VIS1-A2's review, found by a scene sweep diffed against each "
          "surviving mutant) -- a Pin on a 3.5-in support through a true camera at the tolerance box's edge stays "
          "UNSURE because each corner's top-face allowance is THAT corner's lens height's (the highest lens's "
          "would let it agree with the floor: ON THE FLOOR with a fix 12 in past it); a Pin LYING across the view "
          "close to a 6-in lens keeps its centre inside its fix because the fix's near end is the ground range's "
          "own (the lowest ray of a lying Pin meets the floor NEARER than the line it rests on)") {
    {
        CameraPlant plant{edgeTruth(8.0, 0.0, 54), 7};
        const double px = camrig::kLensFwd + 45.0;
        plant.addElement(camrig::pin(px, 0.0, 3.5));
        const hal::VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
        REQUIRE(fr.count == 1);
        REQUIRE_FALSE(fr.detections[0].clipped());
        const vision::PieceSighting s = vision::sightPiece(fr.detections[0], camrig::model(8.0, 0.0),
                                                           camrig::poseAt(0.0, 0.0, 0.0, 0.0, 0.0), camrig::fcfg());
        CHECK(s.posture == Posture::Unsure);
    }
    {
        CameraPlant plant{edgeTruth(6.0, 0.0, 0), 1};
        const double bear = -24.0 * kPi / 180.0;
        const double px = camrig::kLensFwd + 10.0 * std::cos(bear);
        const double py = 10.0 * std::sin(bear);
        sim::Element e = camrig::pin(px, py, 0.0, true);
        e.lyingHeadingRad = kPi / 2.0;  // across the view: its centre half a length to the left of (px, py)
        plant.addElement(e);
        const hal::VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
        REQUIRE(fr.count == 1);
        REQUIRE_FALSE(fr.detections[0].clipped());
        const vision::PieceSighting s = vision::sightPiece(fr.detections[0], camrig::model(6.0, 0.0),
                                                           camrig::poseAt(0.0, 0.0, 0.0, 0.0, 0.0), camrig::fcfg());
        REQUIRE(s.posture == Posture::Lying);
        REQUIRE(s.fix.valid);
        CHECK(std::hypot(s.fix.x - px, s.fix.y - (py + 3.25)) <= s.fix.radius);
    }
}
