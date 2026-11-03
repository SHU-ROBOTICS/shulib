// sim::CameraPlant — the HOST CAMERA MODEL's own anchors and failure modes (chunk VIS1-A1, the VIS1
// brief's §1.11 and §2.2 "the host camera model").
//
// Its anchors are its OWN, computed by hand here and never through vision/ (the A2 rule: the model and
// the library must not share a projection, or one sign error cancels in every test).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * Straight ahead at the lens's height lands on the image's centre; UP is a smaller row; LEFT is a
//     smaller column; the robot's heading and the camera's pitch turn the view the right way.
//   * A one-sided quad (a Toggle's face) is invisible from behind.
//   * FROZEN returns the last frame RE-STAMPED (a new time, a new poll count — indistinguishable from a
//     still scene); UNPLUGGED is a ReadFault with no detections; TORN is a PartialFault.
//   * setModel(Different) WITHHOLDS every Object, counted — the colours kept; 23 detections are
//     complete, 24 and 25 POSSIBLY FULL with 24 held (as the adapter).
//   * addRolledToggle: seated, the field face's lower vertex lies ON the wall line and its apex at the
//     axle + the circumradius; +120 degrees brings the bottom face into the field position, -120 the
//     outer face.
//   * A scene past its capacity throws (a test's mistake), the jitter is deterministic per seed.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "shulib/hal/vision_frames.hpp"
#include "shulib/sim/camera_plant.hpp"
#include "shulib/units/quantity.hpp"

namespace hal = shulib::hal;
namespace sim = shulib::sim;
using shulib::units::Time;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// A small square quad facing the camera (normal -X), centred at (x, y, z), `half` inches across.
sim::ColourQuad square(double x, double y, double z, double half, std::uint8_t id) {
    sim::ColourQuad q;
    q.c = {sim::P3{x, y - half, z - half}, sim::P3{x, y + half, z - half}, sim::P3{x, y + half, z + half},
           sim::P3{x, y - half, z + half}};
    q.colourId = id;
    return q;
}

double centreU(const hal::Detection& d) { return d.left + d.width / 2.0; }
double centreV(const hal::Detection& d) { return d.top + d.height / 2.0; }

sim::TrueCamera level(double height) {
    sim::TrueCamera t{};
    t.forward = 0.0;
    t.height = height;
    return t;
}

}  // namespace

TEST_CASE("CameraPlant anchors: straight ahead at the lens's height is the centre; UP is a smaller row; "
          "LEFT a smaller column") {
    sim::CameraPlant plant{level(10.0)};
    plant.addQuad(square(30.0, 0.0, 10.0, 0.5, 1));
    hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(centreU(f.detections[0]) == doctest::Approx(160.0).epsilon(0.005));
    CHECK(centreV(f.detections[0]) == doctest::Approx(120.0).epsilon(0.005));
    CHECK(f.detections[0].kind == hal::DetectionKind::Colour);
    CHECK(f.detections[0].id == 1);

    const double fy = 120.0 / std::tan(31.5 * kPi / 180.0);
    const double fx = 160.0 / std::tan(37.0 * kPi / 180.0);
    plant.clear();
    plant.addQuad(square(30.0, 0.0, 15.0, 0.5, 1));  // 5 in ABOVE the lens
    f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(centreV(f.detections[0]) < 120.0);
    CHECK(centreV(f.detections[0]) == doctest::Approx(120.0 - fy * 5.0 / 30.0).epsilon(0.01));
    plant.clear();
    plant.addQuad(square(30.0, 6.0, 10.0, 0.5, 1));  // 6 in LEFT
    f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(centreU(f.detections[0]) < 160.0);
    CHECK(centreU(f.detections[0]) == doctest::Approx(160.0 - fx * 6.0 / 30.0).epsilon(0.01));
}

TEST_CASE("CameraPlant anchors: the robot's heading and the camera's mount turn the view") {
    // Facing +Y (90 degrees), a quad on the +Y axis is straight ahead.
    sim::CameraPlant plant{level(10.0)};
    sim::ColourQuad q = square(0.0, 30.0, 10.0, 0.5, 2);
    q.c = {sim::P3{-0.5, 30.0, 9.5}, sim::P3{0.5, 30.0, 9.5}, sim::P3{0.5, 30.0, 10.5}, sim::P3{-0.5, 30.0, 10.5}};
    plant.addQuad(q);
    hal::VisionFrame f = plant.read(0.0, 0.0, 0.5 * kPi, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(centreU(f.detections[0]) == doctest::Approx(160.0).epsilon(0.005));
    // Facing +X it is out of view (90 degrees left, past 37).
    CHECK(plant.read(0.0, 0.0, 0.0, Time{0.0}).count == 0);
    // The lens forward of the tracking centre: a robot at x = -7 with the lens 7 in forward sees what
    // a lens at the origin sees.
    sim::TrueCamera fwd = level(10.0);
    fwd.forward = 7.0;
    sim::CameraPlant p2{fwd};
    p2.addQuad(square(30.0, 0.0, 10.0, 0.5, 1));
    f = p2.read(-7.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(centreU(f.detections[0]) == doctest::Approx(160.0).epsilon(0.005));
    CHECK(centreV(f.detections[0]) == doctest::Approx(120.0).epsilon(0.005));
    // Pitched UP 10 degrees: a point at the lens's height straight ahead appears BELOW the centre.
    sim::TrueCamera up = level(10.0);
    up.pitchDeg = 10.0;
    sim::CameraPlant p3{up};
    p3.addQuad(square(30.0, 0.0, 10.0, 0.5, 1));
    f = p3.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    const double fy = 120.0 / std::tan(31.5 * kPi / 180.0);
    CHECK(centreV(f.detections[0]) == doctest::Approx(120.0 + fy * std::tan(10.0 * kPi / 180.0)).epsilon(0.01));
    // Yawed 20 degrees LEFT: a point straight ahead of the ROBOT appears RIGHT of the centre.
    sim::TrueCamera yawed = level(10.0);
    yawed.yawDeg = 20.0;
    sim::CameraPlant p4{yawed};
    p4.addQuad(square(30.0, 0.0, 10.0, 0.5, 1));
    f = p4.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(centreU(f.detections[0]) > 160.0);
}

TEST_CASE("CameraPlant: a one-sided quad is invisible from behind; a box is clipped to the image; behind "
          "the lens is not seen") {
    sim::CameraPlant plant{level(10.0)};
    sim::ColourQuad face = square(30.0, 0.0, 10.0, 0.5, 1);
    face.oneSided = true;
    face.normal = {-1.0, 0.0, 0.0};  // facing the camera at the origin
    plant.addQuad(face);
    CHECK(plant.read(0.0, 0.0, 0.0, Time{0.0}).count == 1);
    plant.clear();
    face.normal = {1.0, 0.0, 0.0};  // facing away
    plant.addQuad(face);
    CHECK(plant.read(0.0, 0.0, 0.0, Time{0.0}).count == 0);
    plant.clear();
    plant.addQuad(square(-30.0, 0.0, 10.0, 0.5, 1));  // behind
    CHECK(plant.read(0.0, 0.0, 0.0, Time{0.0}).count == 0);
    plant.clear();
    plant.addQuad(square(10.0, 0.0, 10.0, 8.0, 1));  // bigger than the view: clipped to it
    const hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(f.detections[0].left == 0);
    CHECK(f.detections[0].top == 0);
    CHECK(f.detections[0].left + f.detections[0].width == 319);
    CHECK(f.detections[0].top + f.detections[0].height == 239);
    CHECK(f.detections[0].clipped());
    plant.clear();
    sim::ColourQuad invisible = square(30.0, 0.0, 10.0, 0.5, 0);  // fires no descriptor
    plant.addQuad(invisible);
    CHECK(plant.read(0.0, 0.0, 0.0, Time{0.0}).count == 0);
}

TEST_CASE("CameraPlant: FROZEN is the last frame RE-STAMPED; UNPLUGGED a ReadFault with nothing; TORN a "
          "PartialFault") {
    sim::CameraPlant plant{level(10.0)};
    plant.addQuad(square(30.0, 0.0, 10.0, 0.5, 1));
    const hal::VisionFrame a = plant.read(0.0, 0.0, 0.0, Time{1.0});
    REQUIRE(a.count == 1);
    CHECK(a.readSeq == 1U);
    plant.setState(sim::CameraState::Frozen);
    // The robot TURNS 30 degrees: a live camera would see the quad move; a frozen one does not.
    const hal::VisionFrame b = plant.read(0.0, 0.0, 30.0 * kPi / 180.0, Time{1.5});
    CHECK(b.usable());
    CHECK(b.count == 1);
    CHECK(b.detections[0].left == a.detections[0].left);
    CHECK(b.readAt.value() == doctest::Approx(1.5));  // re-stamped
    CHECK(b.readSeq == 2U);
    plant.setState(sim::CameraState::Normal);
    const hal::VisionFrame live = plant.read(0.0, 0.0, 30.0 * kPi / 180.0, Time{2.0});
    REQUIRE(live.count == 1);
    CHECK(live.detections[0].left > a.detections[0].left + 50);  // turned left: the quad moved right

    plant.setState(sim::CameraState::Unplugged);
    const hal::VisionFrame u = plant.read(0.0, 0.0, 0.0, Time{3.0});
    CHECK(u.status == hal::FrameStatus::ReadFault);
    CHECK(u.count == 0);
    CHECK(u.readAt.value() == doctest::Approx(3.0));
    plant.setState(sim::CameraState::Torn);
    const hal::VisionFrame t = plant.read(0.0, 0.0, 0.0, Time{4.0});
    CHECK(t.status == hal::FrameStatus::PartialFault);
    CHECK(t.skipped == 1);
    CHECK_FALSE(t.usable());

    // A camera frozen before it ever read reads the scene (there is no last frame to repeat).
    sim::CameraPlant fresh{level(10.0)};
    fresh.addQuad(square(30.0, 0.0, 10.0, 0.5, 1));
    fresh.setState(sim::CameraState::Frozen);
    CHECK(fresh.read(0.0, 0.0, 0.0, Time{0.0}).count == 1);
}

TEST_CASE("CameraPlant: a model check other than Matches WITHHOLDS every Object, counted; the colours "
          "are kept") {
    sim::CameraPlant plant{level(10.0)};
    sim::Element pin{};
    pin.classId = 3;
    pin.x = 30.0;
    pin.lowerColourId = 3;  // a yellow Pin's halves fire the yellow descriptor
    pin.upperColourId = 3;
    plant.addElement(pin);
    hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    CHECK(f.model == hal::ModelCheck::Matches);
    CHECK(f.count == 3);  // the object and its two halves
    CHECK(f.objectsWithheld == 0);
    CHECK(f.detections[0].kind == hal::DetectionKind::Object);
    CHECK(f.detections[0].id == 3);
    CHECK(f.detections[0].score == 90);
    for (const hal::ModelCheck m : {hal::ModelCheck::Different, hal::ModelCheck::Unreadable,
                                    hal::ModelCheck::NotAsked}) {
        plant.setModel(m);
        f = plant.read(0.0, 0.0, 0.0, Time{0.0});
        CHECK(f.model == m);
        CHECK(f.objectsWithheld == 1);
        CHECK(f.count == 2);
        CHECK(f.reportedCount == 3);  // the camera still SAW it
        for (std::uint8_t i = 0; i < f.count; ++i) {
            CHECK(f.detections[i].kind == hal::DetectionKind::Colour);
        }
    }
}

TEST_CASE("CameraPlant: an element's score reaches the frame WHOLE -- the camera's raw range, 0..65535, "
          "as the seam carries it") {
    // The seam's score holds the camera's whole raw uint16 (VIS1-A1's review: a score narrowed to int16 read
    // 65535 as -1, the "not an Object" value). The host test plant must emit that whole range too, or no
    // scene could put a high score in front of a consumer. The value goes in through the plant's OWN type
    // (a cast that is a no-op while that type holds the range), so a narrowed field fails these CHECKs,
    // not the build.
    CHECK(std::numeric_limits<decltype(sim::Element::score)>::max() >= 65535);
    for (const std::int32_t raw : {0, 32767, 32768, 65535}) {
        CAPTURE(raw);
        sim::CameraPlant plant{level(10.0)};
        sim::Element pin{};
        pin.classId = 3;
        pin.x = 30.0;
        pin.score = static_cast<decltype(pin.score)>(raw);
        plant.addElement(pin);
        const hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
        REQUIRE(f.count == 1);
        CHECK(f.detections[0].kind == hal::DetectionKind::Object);
        CHECK(f.detections[0].score == raw);
    }
}

TEST_CASE("CameraPlant: 23 detections are complete; 24 and 25 are POSSIBLY FULL, 24 held (as the "
          "adapter)") {
    for (const int n : {23, 24, 25}) {
        CAPTURE(n);
        sim::CameraPlant plant{level(10.0)};
        for (int i = 0; i < n; ++i) {
            plant.addQuad(square(60.0, -20.0 + 1.7 * i, 10.0, 0.3, 1));
        }
        const hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
        CHECK(f.reportedCount == n);
        CHECK(f.truncated == (n >= 24));
        CHECK(f.count == (n < 24 ? n : 24));
        CHECK(f.usable());
    }
}

TEST_CASE("CameraPlant: every n-th detection dropped; the jitter deterministic per seed") {
    sim::CameraPlant plant{level(10.0)};
    for (int i = 0; i < 6; ++i) {
        plant.addQuad(square(60.0, -10.0 + 4.0 * i, 10.0, 0.5, 1));
    }
    plant.setDropEvery(3);
    hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    CHECK(f.count == 4);  // the 3rd and the 6th dropped
    CHECK(f.reportedCount == 4);
    plant.setDropEvery(0);
    CHECK(plant.read(0.0, 0.0, 0.0, Time{0.0}).count == 6);

    sim::CameraPlant a{level(10.0), 7};
    sim::CameraPlant b{level(10.0), 7};
    sim::CameraPlant c{level(10.0), 8};
    for (sim::CameraPlant* p : {&a, &b, &c}) {
        p->setJitterPx(2.0);
        p->addQuad(square(30.0, 0.0, 10.0, 2.0, 1));
    }
    bool differs = false;
    for (int k = 0; k < 10; ++k) {
        const hal::VisionFrame fa = a.read(0.0, 0.0, 0.0, Time{0.0});
        const hal::VisionFrame fb = b.read(0.0, 0.0, 0.0, Time{0.0});
        const hal::VisionFrame fc = c.read(0.0, 0.0, 0.0, Time{0.0});
        CHECK(fa.detections[0].left == fb.detections[0].left);
        CHECK(fa.detections[0].width == fb.detections[0].width);
        differs = differs || fa.detections[0].left != fc.detections[0].left
                  || fa.detections[0].width != fc.detections[0].width;
    }
    CHECK(differs);
}

TEST_CASE("CameraPlant: a lying Pin's box is wider than tall; a standing one's taller than wide; a raised "
          "one sits higher") {
    sim::CameraPlant plant{level(10.0)};
    sim::Element standing{};
    standing.x = 30.0;
    plant.addElement(standing);
    hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    const hal::Detection upright = f.detections[0];
    CHECK(upright.height > upright.width);
    plant.clear();
    sim::Element lying = standing;
    lying.lying = true;
    lying.lyingHeadingRad = 0.5 * kPi;  // across the view
    plant.addElement(lying);
    f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(f.detections[0].width > f.detections[0].height);
    plant.clear();
    sim::Element raised = standing;
    raised.baseZ = 3.0;
    plant.addElement(raised);
    f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(f.detections[0].top + f.detections[0].height < upright.top + upright.height);
}

TEST_CASE("CameraPlant addRolledToggle: seated, the field face's lower vertex is ON the wall line and its "
          "apex at the axle + the circumradius; +-120 degrees brings the next face into the field") {
    // The wall's inner face at x = 70.2, the field toward -X.
    const double wx = 70.2;
    const double axle = 13.20;
    const double r = 1.19;
    sim::CameraPlant plant{level(10.0)};
    plant.addRolledToggle(wx, 0.0, -1.0, 0.0, 25.8, axle, r, 0.0, 0.0, 1, 2, 3);
    REQUIRE(plant.quadCount() == 3U);
    const sim::ColourQuad& field = plant.quad(0);
    const sim::ColourQuad& bottom = plant.quad(1);
    const sim::ColourQuad& outer = plant.quad(2);
    CHECK(field.colourId == 1);
    CHECK(bottom.colourId == 2);
    CHECK(outer.colourId == 3);
    // The field face's LOWER edge (corners 0 and 1) on the wall line, at the axle - R/2.
    for (int k = 0; k < 2; ++k) {
        CAPTURE(k);
        CHECK(field.c[static_cast<std::size_t>(k)].x == doctest::Approx(wx));
        CHECK(field.c[static_cast<std::size_t>(k)].z == doctest::Approx(axle - 0.5 * r));
    }
    // Its UPPER edge (corners 2 and 3) is the apex: the axle + R, R cos 30 behind the wall line.
    for (int k = 2; k < 4; ++k) {
        CAPTURE(k);
        CHECK(field.c[static_cast<std::size_t>(k)].z == doctest::Approx(axle + r));
        CHECK(field.c[static_cast<std::size_t>(k)].x == doctest::Approx(wx + r * std::cos(kPi / 6.0)));
    }
    // 25.8 in long along the wall (y).
    CHECK(std::fabs(field.c[1].y - field.c[0].y) == doctest::Approx(25.8));
    // Faces the field and leans back: the normal points -X (into the field) and up.
    CHECK(field.normal.x < 0.0);
    CHECK(field.normal.z > 0.0);
    // The bottom face is horizontal and faces down.
    CHECK(bottom.c[0].z == doctest::Approx(bottom.c[2].z));
    CHECK(bottom.normal.z == doctest::Approx(-1.0));
    // The outer face faces out of the field.
    CHECK(outer.normal.x > 0.0);
    // Each face is ~2.06 in wide (vertex to vertex: R sqrt 3).
    CHECK(std::hypot(field.c[3].x - field.c[0].x, field.c[3].z - field.c[0].z) == doctest::Approx(r * std::sqrt(3.0)));

    // Rolled +120: the BOTTOM face now sits where the field face was.
    sim::CameraPlant plus{level(10.0)};
    plus.addRolledToggle(wx, 0.0, -1.0, 0.0, 25.8, axle, r, 120.0, 0.0, 1, 2, 3);
    const sim::ColourQuad& nowField = plus.quad(1);  // the bottom face's quad
    for (std::size_t k = 0; k < 4; ++k) {
        CAPTURE(k);
        // Same corner set as the seated field face (possibly in another order along the face).
        bool found = false;
        for (std::size_t j = 0; j < 4; ++j) {
            found = found || (std::fabs(nowField.c[k].x - field.c[j].x) < 1e-9
                              && std::fabs(nowField.c[k].y - field.c[j].y) < 1e-9
                              && std::fabs(nowField.c[k].z - field.c[j].z) < 1e-9);
        }
        CHECK(found);
    }
    CHECK(nowField.normal.x < 0.0);
    // Rolled -120: the OUTER face does.
    sim::CameraPlant minus{level(10.0)};
    minus.addRolledToggle(wx, 0.0, -1.0, 0.0, 25.8, axle, r, -120.0, 0.0, 1, 2, 3);
    CHECK(minus.quad(2).normal.x < 0.0);
    CHECK(minus.quad(2).normal.z > 0.0);
    // A lift raises everything by it.
    sim::CameraPlant lifted{level(10.0)};
    lifted.addRolledToggle(wx, 0.0, -1.0, 0.0, 25.8, axle, r, 0.0, 0.5, 1, 2, 3);
    CHECK(lifted.quad(0).c[0].z == doctest::Approx(axle - 0.5 * r + 0.5));
}

TEST_CASE("CameraPlant addToggle: the seated field face leans back from the wall line, the bottom face "
          "under it; a scene past its capacity throws") {
    sim::CameraPlant plant{level(10.0)};
    plant.addToggle(70.2, 0.0, -1.0, 0.0, 25.8, 12.61, 14.39, 1.03, 1, 3);
    REQUIRE(plant.quadCount() == 2U);
    CHECK(plant.quad(0).c[0].x == doctest::Approx(70.2));
    CHECK(plant.quad(0).c[0].z == doctest::Approx(12.61));
    CHECK(plant.quad(0).c[2].x == doctest::Approx(70.2 + 1.03));
    CHECK(plant.quad(0).c[2].z == doctest::Approx(14.39));
    CHECK(plant.quad(1).c[2].z == doctest::Approx(12.61));
    CHECK(plant.quad(1).normal.z == doctest::Approx(-1.0));
    // From a 10-in camera 30 in out, square: the field face is seen, and its bottom face (from below) too.
    const hal::VisionFrame f = plant.read(70.2 - 30.0, 0.0, 0.0, Time{0.0});
    CHECK(f.count == 2);

    sim::CameraPlant full{level(10.0)};
    for (std::size_t i = 0; i < sim::CameraPlant::kMaxQuads; ++i) {
        full.addQuad(square(30.0, 0.0, 10.0, 0.1, 1));
    }
    CHECK_THROWS_AS(full.addQuad(square(30.0, 0.0, 10.0, 0.1, 1)), std::length_error);
    for (std::size_t i = 0; i < sim::CameraPlant::kMaxElements; ++i) {
        full.addElement(sim::Element{});
    }
    CHECK_THROWS_AS(full.addElement(sim::Element{}), std::length_error);
    CHECK_THROWS_AS((void)full.quad(sim::CameraPlant::kMaxQuads), std::out_of_range);
}
