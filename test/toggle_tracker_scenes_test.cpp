// vision::ToggleTracker against SCENES from the INDEPENDENT host camera model (chunk VIS1-A2; the VIS1
// brief's §1.12 and §2.2 "the tracker" — its probes P4, P6, P7, P8, P8b, P4b and the re-check's R2, R4,
// R5, R6.4, R7 and S15b, each turned from a printout into CHECKs).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   The library believes the TYPED mount; the host camera model (sim/camera_plant.hpp, its own
//   projection — it includes nothing from vision/) sees through a TRUE camera moved inside the stated
//   tolerances (camera_scene_rig.hpp). So a gate that only works for the believed camera fails here.
//   * Each gate against its lookalike: a plate at the face's height (LENGTH), a wall-top LED strip and a
//     bumper-height band (ONE CAMERA EXPLAINS IT — elevation), a block cut at the image's side (CUT /
//     SHAPE), a Toggle off to the side (the bearing WINDOW's containment, S15b), a short strip mid-field
//     (the POSITION gate), a vertical plate on the face's line (the face LEANS BACK — T10).
//   * The bottom face: the WHOLE Toggle approached by low cameras — never the bottom face's colour (T1,
//     T8), with the truth an equilateral prism the library does not believe exactly (P8/P8b), and TURNED
//     -30..+30 degrees (R7: the defect the colour-above veto fixes).
//   * Freshness: frozen while driving (S6), frozen while TURNING IN PLACE (R2.S3), the LENS's travel
//     (R2.S3b), recovery RESTARTS the hold (T7), a torn read restarts it (S10), unplugged and starved
//     consumers are silent (S7, S14), a read at the camera's cap is "camera full" (R2.S4, T2).
//   * Characterisations, printed and pinned so a change is SEEN: the mount table (R5: a 10-in level lens
//     first reports at >= 60 in, a 13.25-in one at <= 44, the 12.25-in one never — D9's blind band), a
//     Toggle rolled -120..+120 (R4: never claimed set; the mid-roll colour count printed), the lookalike
//     sweep (P4b: the position gate's count pinned at 0), side-clipped short bands (R2.S8: 0 RED).
//   * A lagged image while turning with the position gate on: never a wrong colour or Toggle (R6.4).
//
// The numbers here are the PROBES' (camera_scene_rig.hpp) — the team's own come from the bench.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <string>

#include "camera_scene_rig.hpp"

namespace {

using camrig::hal::Detection;
using camrig::hal::VisionFrame;
using camrig::kFace;
using camrig::kLensFwd;
using camrig::kPi;
using camrig::kWallX;
using camrig::sim::CameraPlant;
using camrig::sim::CameraState;
using camrig::sim::ColourQuad;
using camrig::sim::P3;
using camrig::Time;
using camrig::vision::ToggleColour;
using camrig::vision::ToggleTracker;
using camrig::vision::ToggleTrackerConfig;
using camrig::vision::ToggleView;
using camrig::vision::ToggleWhy;

// A flat quad from (x, y0..y1) at heights z0..z1 (vertical), colour `id`.
ColourQuad verticalQuad(double x, double y0, double y1, double z0, double z1, std::uint8_t id) {
    ColourQuad q;
    q.c = {P3{x, y0, z0}, P3{x, y1, z0}, P3{x, y1, z1}, P3{x, y0, z1}};
    q.colourId = id;
    return q;
}

// `reads` reads from (rx, 0) facing +x, with the default bound; the view at the end.
ToggleView still(CameraPlant& plant, ToggleTracker& tr, double rx, int reads, double& t) {
    return camrig::readsAt(plant, tr, rx, 0.0, 0.0, camrig::poseAt(rx, 0.0, 0.0), reads, t);
}

// The two truths a scene is run against: the face the library believes at the LOW end of its lower edge
// (12.55, set back 1.2), and the equilateral prism the arithmetic gives (12.61, 1.02) — P8/P8b.
struct FaceTruth {
    double bottom;
    double setBack;
};
constexpr FaceTruth kTruths[] = {{12.55, 1.2}, {12.61, 1.02}};

}  // namespace

// ─── P4 / P8b: the tracker's gates against the host camera model ──────────────────────────────────

TEST_CASE("toggle scenes S1: approaching a RED face (8-in level camera): RED from afar, never another colour, "
          "and 'cut by the image' once the face leaves the frame") {
    for (const FaceTruth tr0 : kTruths) {
        CAPTURE(tr0.bottom);
        CameraPlant plant{camrig::truth(8.0), 7};
        plant.setJitterPx(1.0);
        plant.addQuad(camrig::face(1, 0.0, tr0.bottom, tr0.setBack));
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
        double t = 0.0;
        double firstSure = -1.0;
        double lastSure = -1.0;
        for (double d = 72.0; d >= 1.0; d -= 1.0) {
            const ToggleView v = camrig::park(plant, tr, d, t, 0.2);
            CAPTURE(d);
            if (v.colour == ToggleColour::Red) {
                if (firstSure < 0.0) {
                    firstSure = d;
                }
                lastSure = d;
            }
            CHECK(v.colour != ToggleColour::Blue);
            CHECK(v.colour != ToggleColour::Yellow);
        }
        MESSAGE("S1 (lower edge " << tr0.bottom << "): first RED at " << firstSure << " in, last at " << lastSure);
        CHECK(firstSure >= 48.0);
        CHECK(lastSure >= 8.0);
        CHECK(lastSure <= 14.0);  // the face leaves an 8-in level camera's frame close in (F1)
        // the REASON at the frame's edge is a clean "cut", never "no band" (a consumer learns "too close")
        double t2 = 0.0;
        ToggleTracker fresh{camrig::tcfg(), camrig::model(8.0)};
        CHECK(camrig::park(plant, fresh, 9.0, t2, 0.3).why == ToggleWhy::CutByImage);
    }
}

TEST_CASE("toggle scenes S2: a 6-in level camera has the BLUE face at 30 in and loses it at 3 in -- UNKNOWN, "
          "never a colour from a partial face") {
    CameraPlant plant{camrig::truth(6.0), 3};
    plant.addQuad(camrig::face(2));
    ToggleTracker tr{camrig::tcfg(), camrig::model(6.0)};
    double t = 0.0;
    CHECK(camrig::park(plant, tr, 30.0, t, 0.3).colour == ToggleColour::Blue);
    const ToggleView near = camrig::park(plant, tr, 3.0, t, 0.3);
    CHECK(near.colour == ToggleColour::Unknown);
}

TEST_CASE("toggle scenes S3: a red/yellow Pin 24 in ahead (its halves fire the RED and YELLOW descriptors) in "
          "front of a BLUE face: BLUE -- a Pin's half is tall, never a band") {
    for (const FaceTruth tr0 : kTruths) {
        CAPTURE(tr0.bottom);
        CameraPlant plant{camrig::truth(8.0), 5};
        plant.addQuad(camrig::face(2, 0.0, tr0.bottom, tr0.setBack));
        camrig::sim::Element pinEl{};
        pinEl.classId = 2;  // redYellowPin
        pinEl.x = kWallX - 40.0 + 24.0 - kLensFwd;
        pinEl.lowerColourId = 1;
        pinEl.upperColourId = 3;
        plant.addElement(pinEl);
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
        double t = 0.0;
        CHECK(camrig::park(plant, tr, 40.0, t, 0.4).colour == ToggleColour::Blue);
    }
}

TEST_CASE("toggle scenes S4: the WORST plate -- red, 10 x 2 in, at the face's own height, 24 in ahead: the "
          "LENGTH gate turns it away; with that gate opened it reads RED (the trap the gate exists for)") {
    for (const bool lengthGate : {true, false}) {
        CAPTURE(lengthGate);
        CameraPlant plant{camrig::truth(8.0), 9};
        const double px = 24.0 + kLensFwd;  // the robot at x = 0
        plant.addQuad(verticalQuad(px, -5.0, 5.0, 12.4, 14.4, 1));
        ToggleTrackerConfig c = camrig::tcfg();
        if (!lengthGate) {
            c.minLengthFraction = 0.01;
            c.maxLengthFraction = 10.0;
        }
        ToggleTracker tr{c, camrig::model(8.0)};
        double t = 0.0;
        const ToggleView v = still(plant, tr, 0.0, 12, t);
        CHECK(v.colour == (lengthGate ? ToggleColour::Unknown : ToggleColour::Red));
    }
}

TEST_CASE("toggle scenes S5: a red LED strip on the wall top (11.0..11.5 in) under a YELLOW face: never RED") {
    CameraPlant plant{camrig::truth(8.0), 11};
    plant.addQuad(camrig::face(3));
    plant.addQuad(verticalQuad(kWallX + 0.1, -20.0, 20.0, 11.0, 11.5, 1));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    for (const double d : {48.0, 30.0, 20.0}) {
        CAPTURE(d);
        CHECK(camrig::park(plant, tr, d, t, 0.4).colour != ToggleColour::Red);
    }
}

TEST_CASE("toggle scenes S5b: the red wall-top strip ALONE (the STATED LIMIT, pinned so a change is seen) -- "
          "with the position gate OFF: UNKNOWN at 48 and 30 in, RED at 20 and 14 in (some camera of the tolerance "
          "box explains its two edges as a face); with the gate ON (S13's numbers): UNKNOWN at every distance -- "
          "the strip's fix is not on a Toggle's face line, so the gate turns it away") {
    for (const bool gate : {false, true}) {
        for (const double d : {48.0, 30.0, 20.0, 14.0}) {
            CAPTURE(gate);
            CAPTURE(d);
            CameraPlant plant{camrig::truth(8.0), 31};
            plant.addQuad(verticalQuad(kWallX + 0.1, -20.0, 20.0, 11.0, 11.5, 1));
            ToggleTrackerConfig c = camrig::tcfg();
            if (gate) {
                c.positionToleranceIn = 3.0;
                c.maxPoseToleranceIn = 6.0;
                c.maxFixRadiusIn = 8.0;
            }
            ToggleTracker tr{c, camrig::model(8.0)};
            double t = 0.0;
            const ToggleColour expected = (gate || d > 25.0) ? ToggleColour::Unknown : ToggleColour::Red;
            CHECK(camrig::park(plant, tr, d, t, 0.4).colour == expected);
        }
    }
}

TEST_CASE("toggle scenes S6: a camera that FREEZES while the robot drives 10 in: UNKNOWN 'camera frozen?'") {
    CameraPlant plant{camrig::truth(8.0), 13};
    plant.addQuad(camrig::face(1));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    REQUIRE(camrig::park(plant, tr, 40.0, t, 0.3).colour == ToggleColour::Red);
    plant.setState(CameraState::Frozen);
    ToggleView v{};
    for (double d = 40.0; d >= 30.0; d -= 1.0) {
        v = camrig::park(plant, tr, d, t, 0.05);
    }
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::CameraFrozen);
}

TEST_CASE("toggle scenes S7: an UNPLUGGED camera -- UNKNOWN at once (read not usable), and 'camera silent' "
          "once its last usable read is older than aliveSeconds") {
    CameraPlant plant{camrig::truth(8.0), 17};
    plant.addQuad(camrig::face(1));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    REQUIRE(camrig::park(plant, tr, 40.0, t, 0.3).colour == ToggleColour::Red);
    plant.setState(CameraState::Unplugged);
    const ToggleView first = camrig::park(plant, tr, 40.0, t, 0.033);
    CHECK(first.colour == ToggleColour::Unknown);
    CHECK(first.why == ToggleWhy::ReadNotUsable);
    const ToggleView later = camrig::park(plant, tr, 40.0, t, 0.2);
    CHECK(later.colour == ToggleColour::Unknown);
    CHECK(later.why == ToggleWhy::CameraSilent);
}

TEST_CASE("toggle scenes S8: the face changes RED -> BLUE: never RED after the change; BLUE only after the hold") {
    CameraPlant plant{camrig::truth(8.0), 19};
    plant.addQuad(camrig::face(1));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    REQUIRE(camrig::park(plant, tr, 30.0, t, 0.3).colour == ToggleColour::Red);
    plant.clear();
    plant.addQuad(camrig::face(2));
    const ToggleView v1 = camrig::park(plant, tr, 30.0, t, 0.07);
    CHECK(v1.colour == ToggleColour::Unknown);
    CHECK(v1.why == ToggleWhy::NotHeldYet);
    CHECK(camrig::park(plant, tr, 30.0, t, 0.2).colour == ToggleColour::Blue);
}

TEST_CASE("toggle scenes S9: two faces of DIFFERENT colours both passing: UNKNOWN 'two colours', never one") {
    CameraPlant plant{camrig::truth(8.0), 23};
    plant.addQuad(camrig::face(1, -8.0));
    plant.addQuad(camrig::face(2, 8.0));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    const ToggleView v = camrig::park(plant, tr, 48.0, t, 0.3);
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::TwoColours);
}

TEST_CASE("toggle scenes S10: a TORN read in the middle of a hold restarts it (positive evidence only)") {
    CameraPlant plant{camrig::truth(8.0), 29};
    plant.addQuad(camrig::face(3));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    (void)camrig::park(plant, tr, 30.0, t, 0.1);
    plant.setState(CameraState::Torn);
    const ToggleView torn = camrig::park(plant, tr, 30.0, t, 0.033);
    CHECK(torn.why == ToggleWhy::ReadNotUsable);
    plant.setState(CameraState::Normal);
    CHECK(camrig::park(plant, tr, 30.0, t, 0.1).colour == ToggleColour::Unknown);  // 0.1 s < the 0.15-s hold
    CHECK(camrig::park(plant, tr, 30.0, t, 0.1).colour == ToggleColour::Yellow);
}

TEST_CASE("toggle scenes S11: a short red strip (6 x 1 in, 9..10 in up) 18 in ahead of a robot 30 in off the "
          "Toggle's line: the shape, length and explain gates do NOT stop it (the stated limit, pinned so a fix "
          "is noticed); the POSITION gate does") {
    for (const bool positionGate : {false, true}) {
        CAPTURE(positionGate);
        CameraPlant plant{camrig::truth(8.0), 37};
        const double sx = 18.0 + kLensFwd;
        plant.addQuad(verticalQuad(sx, 27.0, 33.0, 9.0, 10.0, 1));
        ToggleTrackerConfig c = camrig::tcfg();
        if (positionGate) {
            c.positionToleranceIn = 3.0;
            c.maxPoseToleranceIn = 6.0;
            c.maxFixRadiusIn = 8.0;
        }
        ToggleTracker tr{c, camrig::model(8.0)};
        double t = 0.0;
        const ToggleView v = camrig::readsAt(plant, tr, 0.0, 30.0, 0.0, camrig::poseAt(0.0, 30.0, 0.0, 2.0, 2.0), 12, t);
        CHECK(v.colour == (positionGate ? ToggleColour::Unknown : ToggleColour::Red));
    }
}

TEST_CASE("toggle scenes S12: a 3 x 2 in red block at the face's height, half out of the frame's left side, a "
          "40-degree window: UNKNOWN (a side-clipped band not spanning the centre is cut)") {
    CameraPlant plant{camrig::truth(8.0), 41};
    const double bx = 20.0 + kLensFwd;
    const double edgeY = 20.0 * std::tan(37.0 * kPi / 180.0);  // the view's left edge at 20 in
    plant.addQuad(verticalQuad(bx, edgeY - 1.5, edgeY + 1.5, 12.4, 14.4, 1));
    ToggleTrackerConfig c = camrig::tcfg(40.0);
    ToggleTracker tr{c, camrig::model(8.0)};
    double t = 0.0;
    CHECK(still(plant, tr, 0.0, 12, t).colour == ToggleColour::Unknown);
}

TEST_CASE("toggle scenes S13: the real face with the POSITION gate on (pose +-1 in / 1 deg, fix radius <= 8 in): "
          "RED where it is believed, always Toggle 0 (the +x wall's)") {
    for (const FaceTruth tr0 : kTruths) {
        CAPTURE(tr0.bottom);
        CameraPlant plant{camrig::truth(8.0), 43};
        plant.addQuad(camrig::face(1, 0.0, tr0.bottom, tr0.setBack));
        ToggleTrackerConfig c = camrig::tcfg();
        c.positionToleranceIn = 3.0;
        c.maxPoseToleranceIn = 6.0;
        c.maxFixRadiusIn = 8.0;
        ToggleTracker tr{c, camrig::model(8.0)};
        double t = 0.0;
        double first = -1.0;
        for (double d = 72.0; d >= 6.0; d -= 2.0) {
            CAPTURE(d);
            const ToggleView v = camrig::park(plant, tr, d, t, 0.25);
            if (v.colour == ToggleColour::Red) {
                if (first < 0.0) {
                    first = d;
                }
                CHECK(v.positionChecked);
                CHECK(v.toggleIndex == 0);
            }
            CHECK(v.colour != ToggleColour::Blue);
        }
        CHECK(first >= 24.0);
    }
}

TEST_CASE("toggle scenes S14: a starved consumer -- RED held, then no update for 0.5 s: 'camera silent'") {
    CameraPlant plant{camrig::truth(8.0), 47};
    plant.addQuad(camrig::face(1));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    REQUIRE(camrig::park(plant, tr, 30.0, t, 0.3).colour == ToggleColour::Red);
    const ToggleView v = tr.view(t + 0.5);
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::CameraSilent);
}

TEST_CASE("toggle scenes S15: a Toggle 31 degrees aside (inside the camera's view, outside the 25-degree "
          "window): UNKNOWN") {
    CameraPlant plant{camrig::truth(8.0), 53};
    plant.addQuad(camrig::face(1, 22.0));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    CHECK(camrig::park(plant, tr, 36.0, t, 0.3).colour == ToggleColour::Unknown);
}

TEST_CASE("toggle scenes S15b: a WHOLE, uncut face -- its centre's bearing outside the 25-degree window: UNKNOWN; "
          "inside it: RED (the window's CONTAINMENT, which S15's side-clipped face no longer reaches)") {
    int outside = 0;
    int outsideRefused = 0;
    int inside = 0;
    int insideRed = 0;
    for (double d = 48.0; d <= 72.0; d += 6.0) {
        for (double y = 18.0; y <= 36.0; y += 2.0) {
            CameraPlant plant{camrig::truth(8.0), 61};
            plant.addQuad(camrig::face(1, y));
            const double rx = camrig::robotXFor(d);
            const VisionFrame f0 = plant.read(rx, 0.0, 0.0, Time{0.0});
            bool whole = false;
            for (std::uint8_t i = 0; i < f0.count; ++i) {
                if (!f0.detections[i].clipped()) {
                    whole = true;
                }
            }
            if (!whole) {
                continue;
            }
            const double centreDeg = std::atan2(y, d) * 180.0 / kPi;
            ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
            double t = 0.0;
            const ToggleView v = camrig::park(plant, tr, d, t, 0.4);
            if (centreDeg >= 27.5) {  // outside 25 by more than the yaw and optics uncertainty
                ++outside;
                outsideRefused += v.colour == ToggleColour::Unknown ? 1 : 0;
            } else if (centreDeg <= 22.0) {
                ++inside;
                insideRed += v.colour == ToggleColour::Red ? 1 : 0;
            }
        }
    }
    MESSAGE("S15b: whole faces OUTSIDE the window " << outside << " (refused " << outsideRefused << "), INSIDE "
                                                     << inside << " (RED " << insideRed << ")");
    CHECK(outside >= 3);
    CHECK(outsideRefused == outside);
    CHECK(inside >= 3);
    CHECK(insideRed == inside);
}

TEST_CASE("toggle scenes S16: a red band 9..11 in high, 10 in long, 14 in ahead: never RED (the explain gate)") {
    CameraPlant plant{camrig::truth(8.0), 5};
    const double x = 14.0 + kLensFwd;
    plant.addQuad(verticalQuad(x, -5.0, 5.0, 9.0, 11.0, 1));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    CHECK(still(plant, tr, 0.0, 10, t).colour == ToggleColour::Unknown);
}

// ─── P6 / P8: the bottom face, a full camera, the per-corner gate, a camera pitched up ────────────

TEST_CASE("toggle scenes T1: the WHOLE Toggle (field face RED, bottom face YELLOW) approached by an 8-in level "
          "camera: never YELLOW -- the bottom band is never taken for the face -- and RED most of the way") {
    for (const FaceTruth tr0 : kTruths) {
        CAPTURE(tr0.bottom);
        CameraPlant plant{camrig::truth(8.0), 11};
        plant.setJitterPx(1.0);
        camrig::addToggle(plant, 1, 3, tr0.bottom, tr0.setBack);
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
        double t = 0.0;
        int red = 0;
        for (double d = 72.0; d >= 4.0; d -= 2.0) {
            CAPTURE(d);
            const ToggleView v = camrig::park(plant, tr, d, t, 0.2);
            CHECK(v.colour != ToggleColour::Yellow);
            CHECK(v.colour != ToggleColour::Blue);
            red += v.colour == ToggleColour::Red ? 1 : 0;
        }
        CHECK(red > 10);
    }
}

TEST_CASE("toggle scenes T2: a POSSIBLY FULL read (more than 24 detections) with a clean RED face: UNKNOWN "
          "'camera full'") {
    CameraPlant plant{camrig::truth(8.0), 3};
    camrig::addToggle(plant, 1, 3);
    for (int i = 0; i < 23; ++i) {  // pin halves far away fill the camera
        camrig::sim::Element e{};
        e.classId = 0;
        e.x = 52.0 + 0.3 * i;
        e.y = -14.0 + 1.25 * i;
        e.lowerColourId = 1;
        e.upperColourId = 2;
        plant.addElement(e);
    }
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    const ToggleView v = camrig::park(plant, tr, 30.0, t, 0.3);
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::CameraFull);
}

TEST_CASE("toggle scenes T3: after a FAULT read the reason says so -- never a stale 'sure'") {
    CameraPlant plant{camrig::truth(8.0), 3};
    camrig::addToggle(plant, 1, 3);
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    REQUIRE(camrig::park(plant, tr, 30.0, t, 0.4).colour == ToggleColour::Red);
    plant.setState(CameraState::Torn);
    const ToggleView v = camrig::park(plant, tr, 30.0, t, 0.033);
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::ReadNotUsable);
}

TEST_CASE("toggle scenes T4: the per-corner gate -- a red band 9..10 in high, 4 in long, 14 in ahead (an "
          "opponent's plate at bumper height): never a Toggle") {
    CameraPlant plant{camrig::truth(8.0), 5};
    const double rx = kWallX - 30.0 - kLensFwd - 14.0;
    plant.addQuad(verticalQuad(rx + 14.0 + kLensFwd, -2.0, 2.0, 9.0, 10.0, 1));
    ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
    double t = 0.0;
    const ToggleView v = camrig::readsAt(plant, tr, rx, 0.0, 0.0, camrig::poseAt(rx, 0.0, 0.0, 2.0, 2.0), 10, t);
    CHECK(v.colour == ToggleColour::Unknown);
}

TEST_CASE("toggle scenes T5: an 8-in camera pitched UP 15 degrees keeps the face closer in than a level one") {
    double levelLast = -1.0;
    double upLast = -1.0;
    for (const double pitch : {0.0, 15.0}) {
        CameraPlant plant{camrig::truth(8.0, pitch), 9};
        plant.setJitterPx(1.0);
        camrig::addToggle(plant, 2, 3);
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0, pitch)};
        double t = 0.0;
        double last = -1.0;
        for (double d = 48.0; d >= 1.0; d -= 1.0) {
            if (camrig::park(plant, tr, d, t, 0.2).colour == ToggleColour::Blue) {
                last = d;
            }
        }
        (pitch > 0.0 ? upLast : levelLast) = last;
    }
    MESSAGE("T5: last BLUE level " << levelLast << " in, pitched up 15 " << upLast << " in");
    CHECK(levelLast > 0.0);
    CHECK(upLast > 0.0);
    CHECK(upLast < levelLast);
}

TEST_CASE("toggle scenes T6: the mount advice -- 12.75 in level, 10 in pitched up 10, 8 in level: never the "
          "bottom face's YELLOW at any distance from 48 to 1 in") {
    struct Mount {
        double h;
        double pitch;
    };
    for (const Mount mt : {Mount{12.75, 0.0}, Mount{10.0, 10.0}, Mount{8.0, 0.0}}) {
        CAPTURE(mt.h);
        CameraPlant plant{camrig::truth(mt.h, mt.pitch), 13};
        plant.setJitterPx(1.0);
        camrig::addToggle(plant, 1, 3);
        ToggleTracker tr{camrig::tcfg(), camrig::model(mt.h, mt.pitch)};
        double t = 0.0;
        int red = 0;
        for (double d = 48.0; d >= 1.0; d -= 0.5) {
            const ToggleView v = camrig::park(plant, tr, d, t, 0.2);
            CHECK(v.colour != ToggleColour::Yellow);
            red += v.colour == ToggleColour::Red ? 1 : 0;
        }
        CHECK(red > 0);
    }
}

// ─── P7: the latch, the bottom face's sliver, a camera near the lower edge, a vertical plate ──────

TEST_CASE("toggle scenes T7: a camera that FREEZES while the robot drives, then RECOVERS: 'camera frozen?', then "
          "the colour again only after a NEW hold on moving boxes (never the old hold carried over)") {
    CameraPlant plant{camrig::truth(10.0), 7};
    camrig::addToggle(plant, 1, 3);
    ToggleTracker tr{camrig::tcfg(), camrig::model(10.0)};
    double t = 0.0;
    double lens = 60.0;
    auto step = [&](double dLens) {
        lens -= dLens;
        const double rx = camrig::robotXFor(lens);
        t += 0.033;
        tr.update(plant.read(rx, 0.0, 0.0, Time{t}), camrig::poseAt(rx, 0.0, 0.0));
        return tr.view(t);
    };
    ToggleView v{};
    for (int i = 0; i < 10; ++i) {
        v = step(0.5);  // driving in at ~15 in/s
    }
    CHECK(v.colour == ToggleColour::Red);
    plant.setState(CameraState::Frozen);
    for (int i = 0; i < 12; ++i) {
        v = step(0.5);  // 6 in of travel, the picture never changes
    }
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::CameraFrozen);
    plant.setState(CameraState::Normal);
    int recovered = -1;
    for (int i = 0; i < 20; ++i) {
        v = step(0.5);
        if (recovered < 0 && v.colour == ToggleColour::Red) {
            recovered = i;
        }
    }
    // The first moved box (index 0) clears the latch and restarts the hold there: held 4 x 0.033 = 0.132 s
    // at index 4 (< 0.15), 0.165 s at index 5 — RED first at index 5, exactly. A carried-over hold would
    // answer RED at index 0; a hold restarted one read late, at index 6.
    CHECK(recovered == 5);
    CHECK(v.colour == ToggleColour::Red);
}

TEST_CASE("toggle scenes T8: the bottom face's thin sliver never costs the face -- RED unbroken from 48 to 14 "
          "in (the lower edge at the belief's low end) and from 56 to 14 in (the equilateral prism)") {
    struct Run {
        double bottom;
        double setBack;
        double farIn;
    };
    for (const Run run : {Run{12.55, 1.2, 48.0}, Run{12.61, 1.02, 56.0}}) {
        CAPTURE(run.bottom);
        CameraPlant plant{camrig::truth(8.0), 11};
        plant.setJitterPx(1.0);
        camrig::addToggle(plant, 1, 3, run.bottom, run.setBack);
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
        double t = 0.0;
        for (double d = 72.0; d >= 4.0; d -= 2.0) {
            CAPTURE(d);
            const ToggleView v = camrig::park(plant, tr, d, t, 0.2);
            CHECK(v.colour != ToggleColour::Yellow);
            if (d <= run.farIn && d >= 14.0) {
                CHECK(v.colour == ToggleColour::Red);
            }
        }
    }
}

TEST_CASE("toggle scenes T9: a camera 12.75 in up (just above the lower edge), the truth the equilateral prism: "
          "RED holds in to 4 in or closer -- the library must allow the lower edge's whole interval") {
    CameraPlant plant{camrig::truth(12.75), 5};
    plant.setJitterPx(1.0);
    camrig::addToggle(plant, 1, 3, 12.61, 1.02);
    ToggleTracker tr{camrig::tcfg(), camrig::model(12.75)};
    double t = 0.0;
    double lastRed = -1.0;
    for (double d = 30.0; d >= 1.0; d -= 0.5) {
        const ToggleView v = camrig::park(plant, tr, d, t, 0.2);
        CHECK(v.colour != ToggleColour::Yellow);
        if (v.colour == ToggleColour::Red) {
            lastRed = d;
        }
    }
    CHECK(lastRed > 0.0);
    CHECK(lastRed <= 4.0);
}

TEST_CASE("toggle scenes T10: a VERTICAL red plate at the face's heights (12.4..14.4 in, 26 in long) on the "
          "face's line: never a colour from 14 in or closer -- the face LEANS BACK, a plate does not") {
    int redClose = 0;
    int redFar = 0;
    for (double d = 8.0; d <= 24.0; d += 2.0) {
        CameraPlant plant{camrig::truth(8.0), 9};
        plant.setJitterPx(1.0);
        plant.addQuad(verticalQuad(kWallX, -13.0, 13.0, 12.4, 14.4, 1));
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
        double t = 0.0;
        if (camrig::park(plant, tr, d, t, 0.3).colour == ToggleColour::Red) {
            (d <= 14.0 ? redClose : redFar) += 1;
        }
    }
    MESSAGE("T10: the vertical plate read RED at " << redFar << " distances from 16 to 24 in (the stated limit)");
    CHECK(redClose == 0);
}

TEST_CASE("toggle scenes T10b: the same vertical red plate ON THE FACE'S OWN LINE with the POSITION gate ON (S13's "
          "numbers) -- the STATED LIMIT the gate cannot help with, pinned: RED at 22..30 in, named Toggle 0 (it IS "
          "where a Toggle's face is); UNKNOWN at 20 in and closer, and from 32 in (its fix vaguer than 8 in)") {
    for (double d = 8.0; d <= 40.0; d += 2.0) {
        CAPTURE(d);
        CameraPlant plant{camrig::truth(8.0), 9};
        plant.setJitterPx(1.0);
        plant.addQuad(verticalQuad(kWallX, -13.0, 13.0, 12.4, 14.4, 1));
        ToggleTrackerConfig c = camrig::tcfg();
        c.positionToleranceIn = 3.0;
        c.maxPoseToleranceIn = 6.0;
        c.maxFixRadiusIn = 8.0;
        ToggleTracker tr{c, camrig::model(8.0)};
        double t = 0.0;
        const ToggleView v = camrig::park(plant, tr, d, t, 0.3);
        if (d >= 22.0 && d <= 30.0) {
            CHECK(v.colour == ToggleColour::Red);
            CHECK(v.positionChecked);
            CHECK(v.toggleIndex == 0);
        } else {
            CHECK(v.colour == ToggleColour::Unknown);
        }
    }
}

// ─── the re-check's R2: frozen while turning, the partial-band veto, the cap, side clips ──────────

TEST_CASE("toggle scenes R2.S3: frozen while TURNING IN PLACE -- 'camera frozen?' by the third 1-degree step "
          "(past frozenTurnDeg 2); a LIVE camera turning slowly is never called frozen") {
    for (const bool frozen : {true, false}) {
        CAPTURE(frozen);
        CameraPlant plant{camrig::truth(8.0), 3};
        plant.setJitterPx(frozen ? 1.0 : 0.0);  // live WITHOUT jitter: only the turn moves the box
        camrig::addToggle(plant, 1, 3);
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
        const double rx = camrig::robotXFor(30.0);
        double t = 0.0;
        REQUIRE(still(plant, tr, rx, 10, t).colour == ToggleColour::Red);
        if (frozen) {
            plant.setState(CameraState::Frozen);
        }
        double heading = 0.0;
        int frozenAt = -1;
        int frozenWhileLive = 0;
        for (int i = 0; i < 8; ++i) {
            heading += kPi / 180.0;
            t += 0.033;
            tr.update(plant.read(rx, 0.0, heading, Time{t}), camrig::poseAt(rx, 0.0, heading));
            if (tr.view(t).why == ToggleWhy::CameraFrozen) {
                if (frozen && frozenAt < 0) {
                    frozenAt = i;
                }
                frozenWhileLive += frozen ? 0 : 1;
            }
        }
        if (frozen) {
            CHECK(frozenAt == 2);  // 3 degrees turned > 2: the third step, never earlier, never later
        } else {
            CHECK(frozenWhileLive == 0);
        }
    }
}

TEST_CASE("toggle scenes R2.S3b: the frozen check follows the LENS -- turning in place with the turn check set to "
          "45 degrees, the lens 7 in forward travels past frozenMoveIn (2 in) while the centre stays put") {
    CameraPlant plant{camrig::truth(8.0), 3};
    plant.setJitterPx(1.0);
    camrig::addToggle(plant, 1, 3);
    ToggleTrackerConfig c = camrig::tcfg();
    c.frozenTurnDeg = 45.0;
    ToggleTracker tr{c, camrig::model(8.0)};
    const double rx = camrig::robotXFor(30.0);
    double t = 0.0;
    REQUIRE(still(plant, tr, rx, 10, t).colour == ToggleColour::Red);
    plant.setState(CameraState::Frozen);
    double heading = 0.0;
    int frozenAt = -1;
    for (int i = 1; i <= 30; ++i) {
        heading += kPi / 180.0;
        t += 0.033;
        tr.update(plant.read(rx, 0.0, heading, Time{t}), camrig::poseAt(rx, 0.0, heading));
        if (frozenAt < 0 && tr.view(t).why == ToggleWhy::CameraFrozen) {
            frozenAt = i;
        }
    }
    // The lens's chord after n degrees is 2 x 7 x sin(n/2): past 2 in first at n = 17 (16.4 degrees).
    CHECK(frozenAt == 17);
}

TEST_CASE("toggle scenes R2.S7: a passing RED face beside a BLUE band CUT by the image in the window: UNKNOWN "
          "'another colour beside it'; beside a cut RED band: RED") {
    for (const std::uint8_t cutId : {std::uint8_t{2}, std::uint8_t{1}}) {
        CAPTURE(static_cast<int>(cutId));
        CameraPlant plant{camrig::truth(8.0), 5};
        camrig::addToggle(plant, 1, 0);
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
        const double rx = camrig::robotXFor(30.0);
        double t = 0.0;
        ToggleView v{};
        for (int i = 0; i < 10; ++i) {
            t += 0.033;
            VisionFrame f = plant.read(rx, 0.0, 0.0, Time{t});
            f.detections[f.count++] = camrig::colourBox(cutId, 20, 0, 280, 40);
            f.reportedCount += 1;
            tr.update(f, camrig::poseAt(rx, 0.0, 0.0));
            v = tr.view(t);
        }
        if (cutId == 2) {
            CHECK(v.colour == ToggleColour::Unknown);
            CHECK(v.why == ToggleWhy::ConflictingPartialBand);
        } else {
            CHECK(v.colour == ToggleColour::Red);
        }
    }
}

TEST_CASE("toggle scenes R2.S7b: a passing RED band beside a BLUE band the bottom-face check CANNOT judge, in the "
          "window, with NO column overlap (the colour-above veto cannot fire): UNKNOWN, every one") {
    const camrig::vision::CameraModel m = camrig::model(8.0);
    CameraPlant far{camrig::truth(8.0), 5};
    far.addQuad(camrig::face(1, -14.0));
    const double rx = camrig::robotXFor(50.0);
    const VisionFrame ff = far.read(rx, 0.0, 0.0, Time{0.0});
    REQUIRE(ff.count == 1);
    const Detection red = ff.detections[0];
    int candidates = 0;
    int vetoed = 0;
    for (int top = 2; top <= 70; top += 2) {
        for (int h = 6; h <= 24; h += 2) {
            const Detection blue = camrig::colourBox(2, 0, top, red.left - 2, h);
            if (blue.left + blue.width < camrig::hal::kVisionImageWidthPx / 2) {
                continue;  // it must span the centre column to be judged at all (S8)
            }
            VisionFrame f = camrig::frameOf(&blue, 1, 0.0);
            ToggleTracker alone{camrig::tcfg(), m};
            alone.update(f, camrig::poseAt(rx, 0.0, 0.0));
            if (alone.view(0.0).why != ToggleWhy::MaybeBottomFace) {
                continue;
            }
            ++candidates;
            f.detections[f.count++] = red;
            f.reportedCount = 2;
            ToggleTracker both{camrig::tcfg(), m};
            double t = 0.0;
            ToggleView v{};
            for (int i = 0; i < 8; ++i) {
                t += 0.033;
                f.readAt = Time{t};
                both.update(f, camrig::poseAt(rx, 0.0, 0.0));
                v = both.view(t);
            }
            vetoed += (v.colour == ToggleColour::Unknown && v.why == ToggleWhy::ConflictingPartialBand) ? 1 : 0;
        }
    }
    MESSAGE("R2.S7b: BLUE bands the bottom-face check cannot judge " << candidates << ", vetoed " << vetoed);
    CHECK(candidates >= 1);
    CHECK(vetoed == candidates);
}

TEST_CASE("toggle scenes R2.S4: the host model marks a read of EXACTLY 24 possibly full and the tracker says "
          "'camera full'; 23 is complete and the colour holds") {
    const double rx = camrig::robotXFor(30.0);
    for (const int want : {23, 24}) {
        CAPTURE(want);
        CameraPlant plant{camrig::truth(8.0), 5};
        camrig::addToggle(plant, 1, 0);
        for (int k = 0; k < 40; ++k) {  // small yellow squares on the floor ahead: one detection each
            if (plant.read(rx, 0.0, 0.0, Time{0.0}).reportedCount >= want) {
                break;
            }
            const double x = rx + 20.0 + 1.5 * (k % 8);
            const double y = -6.0 + 3.0 * (k / 8);
            ColourQuad q;
            q.c = {P3{x, y, 0.0}, P3{x, y + 1.0, 0.0}, P3{x, y + 1.0, 1.5}, P3{x, y, 1.5}};
            q.colourId = 3;
            plant.addQuad(q);
        }
        ToggleTracker tr{camrig::tcfg(), camrig::model(8.0)};
        double t = 0.0;
        ToggleView v{};
        VisionFrame g{};
        for (int i = 0; i < 10; ++i) {
            t += 0.033;
            g = plant.read(rx, 0.0, 0.0, Time{t});
            tr.update(g, camrig::poseAt(rx, 0.0, 0.0));
            v = tr.view(t);
        }
        REQUIRE(g.reportedCount == want);
        CHECK(g.truncated == (want == 24));
        if (want == 24) {
            CHECK(v.colour == ToggleColour::Unknown);
            CHECK(v.why == ToggleWhy::CameraFull);
        } else {
            CHECK(v.colour == ToggleColour::Red);
        }
    }
}

TEST_CASE("toggle scenes R2.S8: side-clipped SHORT red bands (4, 4.5 and 6.5 in -- legal plates, a lying Pin) "
          "never read RED, with a 35-degree window") {
    int total = 0;
    int red = 0;
    for (const double camH : {8.0, 10.0}) {
        for (const double len : {4.0, 4.5, 6.5}) {
            for (const double low : {12.2, 12.6}) {
                for (double range = 10.0; range <= 40.0; range += 3.0) {
                    for (double lat = -40.0; lat <= 40.0; lat += 2.0) {
                        camrig::sim::TrueCamera tc{};
                        tc.forward = kLensFwd;
                        tc.height = camH;
                        CameraPlant plant{tc, 3};
                        plant.setJitterPx(1.0);
                        const double x = kLensFwd + range;
                        ColourQuad q;
                        q.c = {P3{x, lat - len / 2.0, low}, P3{x, lat + len / 2.0, low},
                               P3{x + 0.5, lat + len / 2.0, low + 2.0}, P3{x + 0.5, lat - len / 2.0, low + 2.0}};
                        q.colourId = 1;
                        plant.addQuad(q);
                        const VisionFrame f0 = plant.read(0.0, 0.0, 0.0, Time{0.0});
                        bool side = false;
                        for (std::uint8_t i = 0; i < f0.count; ++i) {
                            const Detection& d = f0.detections[i];
                            side = side || d.left <= 1 || d.left + d.width >= 319;
                        }
                        if (!side) {
                            continue;
                        }
                        ++total;
                        ToggleTracker tr{camrig::tcfg(35.0), camrig::model(camH)};
                        double t = 0.0;
                        red += still(plant, tr, 0.0, 8, t).colour == ToggleColour::Red ? 1 : 0;
                    }
                }
            }
        }
    }
    MESSAGE("R2.S8: side-clipped short red bands " << total << ", read RED " << red);
    CHECK(total > 500);
    CHECK(red == 0);
}

// ─── the re-check's characterisations: R4 rolled, R5 mounts, R7 turned, P4b lookalikes ────────────

TEST_CASE("toggle scenes R4: a Toggle rolled -120..+120 degrees (CHARACTERISED): seated, never another face's "
          "colour; a full flip, never another face's colour; the mid-roll colour count printed") {
    int midRollColour = 0;
    int midRollCases = 0;
    int wrongSeated = 0;
    int wrongFlipped = 0;
    for (const double camH : {8.0, 10.0, 12.0}) {
        for (const double lens : {8.0, 14.0, 20.0, 30.0}) {
            for (double roll = -120.0; roll <= 120.0; roll += 10.0) {
                CameraPlant plant{camrig::truth(camH), 13};
                plant.setJitterPx(1.0);
                const double lift = 0.35 * std::fabs(std::sin(roll * kPi / 120.0));
                plant.addRolledToggle(kWallX, 0.0, -1.0, 0.0, kFace.lengthIn, 13.20, 1.19, roll, lift, 1, 2, 3);
                ToggleTracker tr{camrig::tcfg(), camrig::model(camH)};
                double t = 0.0;
                const ToggleView v = still(plant, tr, camrig::robotXFor(lens), 12, t);
                const double a = std::fabs(roll);
                if (a >= 20.0 && a <= 100.0) {
                    ++midRollCases;
                    midRollColour += v.colour != ToggleColour::Unknown ? 1 : 0;
                }
                const bool known = v.colour != ToggleColour::Unknown;
                wrongSeated += (roll == 0.0 && known && v.colour != ToggleColour::Red) ? 1 : 0;
                wrongFlipped += (roll == 120.0 && known && v.colour != ToggleColour::Blue) ? 1 : 0;
                wrongFlipped += (roll == -120.0 && known && v.colour != ToggleColour::Yellow) ? 1 : 0;
            }
        }
    }
    MESSAGE("R4: a colour reported for a Toggle 20..100 degrees from seated in " << midRollColour << " of "
                                                                                 << midRollCases << " cases");
    CHECK(midRollCases == 216);
    CHECK(wrongSeated == 0);
    CHECK(wrongFlipped == 0);
}

namespace {

struct Cover {
    double farIn = -1.0;
    double nearIn = -1.0;
    int red = 0;
    int wrong = 0;
    int n = 0;
};

// R5: park square to a seated Toggle (RED face, YELLOW bottom) every 2 in from 80 to 4 in.
Cover mountSweep(double camH, double pitch, double bottom, double setBack) {
    Cover cv{};
    for (double d = 80.0; d >= 4.0; d -= 2.0) {
        CameraPlant plant{camrig::truth(camH, pitch), 11};
        plant.setJitterPx(1.0);
        camrig::addToggle(plant, 1, 3, bottom, setBack);
        ToggleTracker tr{camrig::tcfg(), camrig::model(camH, pitch)};
        double t = 0.0;
        const ToggleView v = still(plant, tr, camrig::robotXFor(d), 10, t);
        ++cv.n;
        if (v.colour == ToggleColour::Red) {
            if (cv.red == 0) {
                cv.farIn = d;
            }
            cv.nearIn = d;
            ++cv.red;
        } else if (v.colour != ToggleColour::Unknown) {
            ++cv.wrong;
        }
    }
    return cv;
}

}  // namespace

TEST_CASE("toggle scenes R5: D9's mounts scored by the tracker (CHARACTERISED) -- never a wrong colour; a 10-in "
          "level lens first reports at >= 60 in, a 13.25-in one at <= 44 in, the 12.25-in one never (the blind "
          "band)") {
    struct Mount {
        double h;
        double p;
    };
    const Mount mounts[] = {{6.0, 0.0},   {8.0, 0.0},   {8.0, 10.0},  {10.0, 0.0}, {10.0, 10.0}, {11.0, 0.0},
                            {11.5, 0.0},  {11.75, 0.0}, {12.0, 0.0},  {12.25, 0.0}, {12.5, 0.0}, {12.75, 0.0},
                            {13.0, 0.0},  {13.25, 0.0}, {14.0, 0.0},  {12.0, 10.0}};
    const FaceTruth truths[] = {{12.55, 1.2}, {12.61, 1.02}, {12.65, 1.0}, {12.36, 1.2}};  // 12.36: out of belief
    int wrong = 0;
    double far10 = 1e9;
    double near10 = -1.0;
    double far1325 = -1.0;
    int blind1225 = 0;
    double near12Up = -1.0;
    for (const Mount mt : mounts) {
        for (const FaceTruth tf : truths) {
            const Cover c = mountSweep(mt.h, mt.p, tf.bottom, tf.setBack);
            wrong += c.wrong;
            MESSAGE("R5 lens " << mt.h << " in, pitch " << mt.p << ", lower edge " << tf.bottom << ": RED from "
                               << c.farIn << " to " << c.nearIn << " in, at " << c.red << " of " << c.n);
            if (tf.bottom > 12.5) {  // inside the belief
                if (mt.h == 10.0 && mt.p == 0.0) {
                    far10 = std::fmin(far10, c.farIn);
                    near10 = std::fmax(near10, c.nearIn);
                }
                if (mt.h == 13.25 && mt.p == 0.0) {
                    far1325 = std::fmax(far1325, c.farIn);
                }
                if (mt.h == 12.25 && mt.p == 0.0) {
                    blind1225 += c.red == 0 ? 1 : 0;
                }
                if (mt.h == 12.0 && mt.p == 10.0) {
                    near12Up = std::fmax(near12Up, c.nearIn);
                }
            }
        }
    }
    CHECK(wrong == 0);
    CHECK(far10 >= 60.0);     // D9 (a): below the face, the far range kept ...
    CHECK(near10 > 0.0);
    CHECK(near10 <= 12.0);    // ... down to ~12 in
    CHECK(far1325 > 0.0);
    CHECK(far1325 <= 44.0);   // above the face the far range is lost (D9's trade)
    CHECK(blind1225 == 3);    // the blind band
    CHECK(near12Up > 0.0);
    CHECK(near12Up <= 12.0);  // D9: pitching a 12-in lens up 10 degrees rescues it close in
}

TEST_CASE("toggle scenes the REAL shape: the bottom colour runs from the face's lower boundary (12.61) round the "
          "rounded corner to the FLAT bottom's far end (12.36, 2.05 in back) -- lenses 6..12.25 in, square (80..4 "
          "in) and turned (-30..+30 degrees at 14..48 in), both bottom colours: never a wrong colour") {
    struct Mount {
        double h;
        double p;
    };
    const Mount mounts[] = {{6.0, 0.0},  {8.0, 0.0},   {8.0, 10.0}, {10.0, 0.0}, {10.0, 10.0}, {11.0, 0.0},
                            {11.5, 0.0}, {11.75, 0.0}, {12.0, 0.0}, {12.0, 10.0}, {12.25, 0.0}};
    auto addRealShape = [](CameraPlant& plant, std::uint8_t bottomId) {
        ColourQuad f = CameraPlant::toggleFace(kWallX, 0.0, -1.0, 0.0, kFace.lengthIn, 12.61, kFace.topIn, 1.02, 1);
        const double dh = kFace.topIn - 12.61;
        const double n = std::hypot(dh, 1.02);
        f.oneSided = true;
        f.normal = {-dh / n, 0.0, 1.02 / n};
        plant.addQuad(f);
        const double half = kFace.lengthIn / 2.0;
        ColourQuad b;  // c[0] on the FLAT bottom: the one-sided test is "below 12.36"
        b.c = {P3{kWallX + 2.05, -half, kFace.flatBottomIn}, P3{kWallX + 2.05, half, kFace.flatBottomIn},
               P3{kWallX, half, 12.61}, P3{kWallX, -half, 12.61}};
        b.colourId = bottomId;
        b.oneSided = true;
        b.normal = {0.0, 0.0, -1.0};
        plant.addQuad(b);
    };
    int wrong = 0;
    int red = 0;
    int cases = 0;
    double near12Up = 99.0;
    for (const Mount mt : mounts) {
        for (const std::uint8_t bottomId : {std::uint8_t{2}, std::uint8_t{3}}) {
            for (double d = 80.0; d >= 4.0; d -= 2.0) {
                CameraPlant plant{camrig::truth(mt.h, mt.p), 11};
                plant.setJitterPx(1.0);
                addRealShape(plant, bottomId);
                ToggleTracker tr{camrig::tcfg(), camrig::model(mt.h, mt.p)};
                double t = 0.0;
                const ToggleView v = still(plant, tr, camrig::robotXFor(d), 10, t);
                ++cases;
                wrong += (v.colour != ToggleColour::Unknown && v.colour != ToggleColour::Red) ? 1 : 0;
                red += v.colour == ToggleColour::Red ? 1 : 0;
                if (mt.h == 12.0 && mt.p == 10.0 && v.colour == ToggleColour::Red) {
                    near12Up = std::fmin(near12Up, d);
                }
            }
            for (const double lens : {14.0, 20.0, 30.0, 40.0, 48.0}) {
                for (double hd = -30.0; hd <= 30.0; hd += 5.0) {
                    CameraPlant plant{camrig::truth(mt.h, mt.p), 9};
                    plant.setJitterPx(1.0);
                    addRealShape(plant, bottomId);
                    ToggleTracker tr{camrig::tcfg(), camrig::model(mt.h, mt.p)};
                    const double h = hd * kPi / 180.0;
                    const double rx = camrig::robotXFor(lens);
                    double t = 0.0;
                    const ToggleView v = camrig::readsAt(plant, tr, rx, 0.0, h, camrig::poseAt(rx, 0.0, h), 10, t);
                    ++cases;
                    wrong += (v.colour != ToggleColour::Unknown && v.colour != ToggleColour::Red) ? 1 : 0;
                    red += v.colour == ToggleColour::Red ? 1 : 0;
                }
            }
        }
    }
    MESSAGE("the real shape: " << cases << " parkings, RED " << red << ", a wrong colour " << wrong
                               << "; a 12-in lens pitched up 10 degrees RED in to " << near12Up << " in");
    CHECK(cases == 11 * 2 * (39 + 65));
    CHECK(wrong == 0);
    CHECK(near12Up <= 12.0);  // the gate's bottom planes cost no range here (a flat-bottom plane cost 28 in)
}

TEST_CASE("toggle scenes R7: a seated Toggle with the robot TURNED -30..+30 degrees (cameras 8, 10, 11 in; the "
          "bottom face BLUE or YELLOW; 14..48 in): never a colour but the field face's") {
    int cases = 0;
    int wrong = 0;
    int sure = 0;
    int squareMissed = 0;
    int past15 = 0;
    for (const double camH : {8.0, 10.0, 11.0}) {
        for (const std::uint8_t bottomId : {std::uint8_t{2}, std::uint8_t{3}}) {
            for (const double lens : {14.0, 20.0, 30.0, 40.0, 48.0}) {
                std::string coverage;  // the brief's §2.2: the coverage printed, camera by camera
                for (double hd = -30.0; hd <= 30.0; hd += 5.0) {
                    CameraPlant plant{camrig::truth(camH), 9};
                    plant.setJitterPx(1.0);
                    camrig::addToggle(plant, 1, bottomId);
                    ToggleTracker tr{camrig::tcfg(), camrig::model(camH)};
                    const double h = hd * kPi / 180.0;
                    const double rx = camrig::robotXFor(lens);
                    double t = 0.0;
                    const ToggleView v = camrig::readsAt(plant, tr, rx, 0.0, h, camrig::poseAt(rx, 0.0, h), 10, t);
                    ++cases;
                    wrong += (v.colour != ToggleColour::Unknown && v.colour != ToggleColour::Red) ? 1 : 0;
                    sure += v.colour == ToggleColour::Red ? 1 : 0;
                    if (v.colour == ToggleColour::Red) {
                        coverage += " " + std::to_string(static_cast<int>(hd));
                        past15 += std::fabs(hd) > 15.0 ? 1 : 0;
                    } else if (hd == 0.0) {
                        ++squareMissed;
                    }
                }
                MESSAGE("R7 coverage: lens " << camH << " in, bottom face " << static_cast<int>(bottomId) << ", "
                                             << lens << " in out: RED at" << (coverage.empty() ? " none" : coverage)
                                             << " degrees");
            }
        }
    }
    MESSAGE("R7: turned-Toggle cases " << cases << ", RED " << sure << ", a wrong colour " << wrong);
    CHECK(cases == 390);
    CHECK(wrong == 0);
    CHECK(sure > 100);
    // pinned so a change is seen (guide 19 states this table): square, every camera and distance reads RED;
    // turned past 15 degrees, none does
    CHECK(squareMissed == 0);
    CHECK(past15 == 0);
}

TEST_CASE("toggle scenes P4b: 540 lookalike bands 30 in in front of a wall (CHARACTERISED) -- the count that "
          "reads RED printed per window; with the POSITION gate on (pose +-2 in / 2 deg), none") {
    const camrig::vision::CameraModel m = camrig::model(8.0);
    int passes96 = 0;
    int passes36 = 0;
    int passesPos = 0;
    int total = 0;
    for (const double z0 : {4.0, 6.0, 9.0, 10.0, 11.0, 12.4, 15.0, 16.0, 18.0}) {
        for (const double th : {1.0, 2.0}) {
            for (const double len : {4.0, 6.0, 10.0, 16.0, 26.0}) {
                for (const double range : {10.0, 14.0, 18.0, 24.0, 36.0, 48.0}) {
                    ++total;
                    const bool face = z0 == 12.4 && th == 2.0 && len == 26.0;
                    for (const bool posGate : {false, true}) {
                        for (const double maxR : {36.0, 96.0}) {
                            CameraPlant plant{camrig::truth(8.0), 5};
                            const double rx = kWallX - 30.0 - kLensFwd - range;
                            plant.addQuad(verticalQuad(rx + range + kLensFwd, -len / 2.0, len / 2.0, z0, z0 + th, 1));
                            ToggleTrackerConfig c = camrig::tcfg();
                            c.maxRangeIn = maxR;
                            if (posGate) {
                                c.positionToleranceIn = 3.0;
                                c.maxPoseToleranceIn = 6.0;
                                c.maxFixRadiusIn = 8.0;
                            }
                            ToggleTracker tr{c, m};
                            double t = 0.0;
                            const ToggleView v = camrig::readsAt(plant, tr, rx, 0.0, 0.0,
                                                                 camrig::poseAt(rx, 0.0, 0.0, 2.0, 2.0), 10, t);
                            if (v.colour != ToggleColour::Red || face) {
                                continue;
                            }
                            if (posGate) {
                                passesPos += maxR == 36.0 ? 1 : 0;
                            } else {
                                (maxR == 96.0 ? passes96 : passes36) += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    MESSAGE("P4b: lookalike bands that read RED -- " << passes96 << " of " << total << " (96-in window), " << passes36
                                                    << " (36-in window), " << passesPos << " (36-in window + the "
                                                    << "position gate)");
    CHECK(total == 540);
    CHECK(passesPos == 0);
    CHECK(passes36 <= passes96);  // a tighter window can only refuse more
}

// ─── R6.4: a lagged image while turning, the position gate on ─────────────────────────────────────

TEST_CASE("toggle scenes R6.4: a lagged image while turning 60 deg/s with the POSITION gate on -- never a wrong "
          "colour or Toggle; lag can only cost a refusal") {
    ToggleTrackerConfig c = camrig::tcfg();
    c.holdSeconds = 0.10;
    c.positionToleranceIn = 3.0;
    c.maxPoseToleranceIn = 6.0;
    c.maxFixRadiusIn = 12.0;
    int redAtZeroLag = 0;
    for (const double lagMs : {0.0, 66.0, 100.0, 150.0}) {
        CAPTURE(lagMs);
        CameraPlant plant{camrig::truth(10.0), 5};
        plant.setJitterPx(1.0);
        camrig::addToggle(plant, 1, 3);
        ToggleTracker tr{c, camrig::model(10.0)};
        const double rx = camrig::robotXFor(30.0);
        double t = 0.0;
        double h = -10.0 * kPi / 180.0;  // sweeping -10 .. +10 degrees at 60 deg/s
        int red = 0;
        int wrong = 0;
        for (int i = 0; i < 10; ++i) {
            t += 0.033;
            h += 60.0 * kPi / 180.0 * 0.033;
            const double hThen = h - 60.0 * kPi / 180.0 * lagMs / 1000.0;
            tr.update(plant.read(rx, 0.0, hThen, Time{t}), camrig::poseAt(rx, 0.0, h));
            const ToggleView v = tr.view(t);
            if (v.colour == ToggleColour::Red) {
                ++red;
                wrong += v.toggleIndex != 0 ? 1 : 0;
            } else if (v.colour != ToggleColour::Unknown) {
                ++wrong;
            }
        }
        if (lagMs == 0.0) {
            redAtZeroLag = red;
        }
        CHECK(wrong == 0);
    }
    CHECK(redAtZeroLag > 0);  // a scene where the colour IS reported without lag
}

// ─── VIS1-A2's review: what gate 5 depends on, and the view's range as a bound ─────────────────────

namespace {

// The Toggle's REAL lower shape (as "the real shape" above): the field face's colour boundary at 12.61 in, its
// apex 1.02 in behind; the bottom colour from that boundary round the corner to the flat bottom 2.05 in back.
void addRealShape(CameraPlant& plant, std::uint8_t bottomId) {
    ColourQuad f = CameraPlant::toggleFace(kWallX, 0.0, -1.0, 0.0, kFace.lengthIn, 12.61, kFace.topIn, 1.02, 1);
    const double dh = kFace.topIn - 12.61;
    const double n = std::hypot(dh, 1.02);
    f.oneSided = true;
    f.normal = {-dh / n, 0.0, 1.02 / n};
    plant.addQuad(f);
    const double half = kFace.lengthIn / 2.0;
    ColourQuad b;
    b.c = {P3{kWallX + 2.05, -half, kFace.flatBottomIn}, P3{kWallX + 2.05, half, kFace.flatBottomIn},
           P3{kWallX, half, 12.61}, P3{kWallX, -half, 12.61}};
    b.colourId = bottomId;
    b.oneSided = true;
    b.normal = {0.0, 0.0, -1.0};
    plant.addQuad(b);
}

}  // namespace

TEST_CASE("toggle scenes the MISSED field face (the STATED DEPENDENCY, pinned): gate 5 keeps a band that could be "
          "the bottom face when the place its field face would occupy is in the picture and EMPTY -- so the "
          "tracker's safety rests on the field face's descriptor firing whenever its bottom face's does (A4 "
          "register HA-199). The RED detection removed: the bottom face's colour reads sure; the RED band's lower "
          "edge cut short by up to 6 px (within the jitter and beyond): never") {
    struct Mount {
        double h;
        double pitch;
    };
    int removedWrong = 0;
    int shortWrong = 0;
    int parkings = 0;
    for (const Mount mt : {Mount{6.0, 0.0}, Mount{8.0, 0.0}, Mount{10.0, 0.0}, Mount{11.0, 0.0}, Mount{12.0, 10.0}}) {
        for (const bool realShape : {false, true}) {
            for (double d = 80.0; d >= 4.0; d -= 4.0) {
                for (const double hd : {0.0, -10.0, 10.0}) {
                    for (const int cut : {-1, 0, 2, 4, 6}) {  // -1: the RED detection removed
                        CameraPlant plant{camrig::truth(mt.h, mt.pitch), 11};
                        plant.setJitterPx(1.0);
                        if (realShape) {
                            addRealShape(plant, 3);
                        } else {
                            camrig::addToggle(plant, 1, 3);
                        }
                        ToggleTracker tr{camrig::tcfg(), camrig::model(mt.h, mt.pitch)};
                        double t = 0.0;
                        const double rx = camrig::robotXFor(d);
                        const double h = hd * kPi / 180.0;
                        for (int i = 0; i < 10; ++i) {
                            t += 0.033;
                            VisionFrame f = plant.read(rx, 0.0, h, Time{t});
                            VisionFrame g = f;
                            g.count = 0;
                            for (std::uint8_t k = 0; k < f.count; ++k) {
                                Detection dd = f.detections[k];
                                if (dd.id == 1) {
                                    if (cut < 0) {
                                        continue;
                                    }
                                    dd.height = static_cast<std::int16_t>(dd.height > cut + 1 ? dd.height - cut : 1);
                                }
                                g.detections[g.count++] = dd;
                            }
                            g.reportedCount = g.count;
                            tr.update(g, camrig::poseAt(rx, 0.0, h));
                        }
                        const ToggleView v = tr.view(t);
                        const bool wrong = v.colour != ToggleColour::Unknown && v.colour != ToggleColour::Red;
                        if (cut < 0) {
                            ++parkings;
                            removedWrong += wrong ? 1 : 0;
                        } else {
                            shortWrong += wrong ? 1 : 0;
                        }
                    }
                }
            }
        }
    }
    MESSAGE("the missed field face: " << parkings << " parkings, the bottom face's YELLOW read sure in "
                                      << removedWrong << "; the RED band cut short by 0..6 px: a wrong colour in "
                                      << shortWrong);
    CHECK(removedWrong > 0);  // the dependency is real: a fix that removes it is SEEN here
    CHECK(shortWrong == 0);
}

TEST_CASE("toggle scenes the view's RANGE is a BOUND on the lens's true distance to the face's lower edge -- every "
          "camera of the tolerance box, not only the corners that explain the band (an 11-in level lens 64 in out: "
          "the corners that explain it said 35..51 in)") {
    int views = 0;
    int missed = 0;
    // lenses below the face (6, 8, 11 in) AND inside its band (13, 14 in: the edges' rays then point opposite
    // ways, so the near end cannot come from one corner's edge agreement alone — mutation NR2), pitched -10..10
    for (const double h : {6.0, 8.0, 11.0, 13.0, 14.0}) {
        for (const double pitch : {-10.0, 0.0, 10.0}) {
            for (const FaceTruth ft : {FaceTruth{12.55, 0.9}, FaceTruth{12.61, 1.03}, FaceTruth{12.65, 1.2}}) {
                for (double d = 6.0; d <= 90.0; d += 2.0) {
                    CameraPlant plant{camrig::truth(h, pitch), 1};
                    plant.setJitterPx(1.0);
                    camrig::addToggle(plant, 1, 3, ft.bottom, ft.setBack);
                    ToggleTracker tr{camrig::tcfg(), camrig::model(h, pitch)};
                    double t = 0.0;
                    const ToggleView v = camrig::park(plant, tr, d, t, 0.25);
                    if (v.colour != ToggleColour::Red) {
                        continue;
                    }
                    ++views;
                    const double trueRange = d - 0.2;  // the true lens is 0.2 in forward of the believed one
                    if (!(v.range.lo <= trueRange && trueRange <= v.range.hi)) {
                        ++missed;
                        MESSAGE("range missed: lens " << h << " pitch " << pitch << " lower edge " << ft.bottom
                                                      << " at " << d << " in: [" << v.range.lo << ", "
                                                      << v.range.hi << "]");
                    }
                }
            }
        }
    }
    // lenses just BELOW the face's lower edge (11.5, 12 in), pitched up a little, far out: some corner's ray
    // cannot reach the edge's plane, and the far end must then be INFINITE (mutation NR1)
    for (const double h : {11.5, 12.0}) {
        for (const double pitch : {2.0, 10.0}) {
            for (double d = 30.0; d <= 120.0; d += 2.0) {
                CameraPlant plant{camrig::truth(h, pitch), 1};
                plant.setJitterPx(1.0);
                camrig::addToggle(plant, 1, 3, 12.61, 1.03);
                ToggleTracker tr{camrig::tcfg(), camrig::model(h, pitch)};
                double t = 0.0;
                const ToggleView v = camrig::park(plant, tr, d, t, 0.25);
                if (v.colour != ToggleColour::Red) {
                    continue;
                }
                ++views;
                const double trueRange = d - 0.2;
                if (!(v.range.lo <= trueRange && trueRange <= v.range.hi)) {
                    ++missed;
                    MESSAGE("range missed: lens " << h << " pitch " << pitch << " at " << d << " in: ["
                                                  << v.range.lo << ", " << v.range.hi << "]");
                }
            }
        }
    }
    MESSAGE("the view's range: " << views << " sure views, the true distance outside the interval in " << missed);
    CHECK(views > 500);
    CHECK(missed == 0);
    {  // the review's case
        CameraPlant plant{camrig::truth(11.0), 1};
        plant.setJitterPx(1.0);
        camrig::addToggle(plant, 1, 3, 12.55, 0.9);
        ToggleTracker tr{camrig::tcfg(), camrig::model(11.0)};
        double t = 0.0;
        const ToggleView v = camrig::park(plant, tr, 64.0, t, 0.25);
        REQUIRE(v.colour == ToggleColour::Red);
        CHECK(v.range.lo <= 63.8);
        CHECK(v.range.hi >= 63.8);
    }
}
