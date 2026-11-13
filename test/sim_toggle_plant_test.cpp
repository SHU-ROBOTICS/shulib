// AF1S-A1 — the HOST TOGGLE itself (sim/toggle_plant.hpp; the AF1S brief's T12). The step's end-to-end tests
// use this plant as their ORACLE (its true face and scoredColour()), so the plant is tested on its own first:
// one roll per full stroke, none on a short stroke or from too far, two in over-roll, none when sticky (and the
// arm stopped where it meets it), the passive stroke and its re-arm, the seat time, a jammed Toggle never
// seated, scoredColour() yellow while touched or unseated, and the camera model shown the TRUE faces after each
// change. Say "host test plant": every number here is the test's.

#include "doctest.h"

#include <cstdint>

#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/sim/camera_plant.hpp"
#include "shulib/sim/mechanism_plant.hpp"
#include "shulib/sim/toggle_plant.hpp"
#include "shulib/units/quantity.hpp"

using shulib::sim::CameraPlant;
using shulib::sim::TogglePlant;
using shulib::sim::TogglePlantConfig;
using shulib::sim::ToggleMode;

namespace {
constexpr std::uint8_t kR = 1;
constexpr std::uint8_t kB = 2;
constexpr std::uint8_t kY = 3;

TogglePlantConfig redSide(ToggleMode m = ToggleMode::Normal) {
    TogglePlantConfig c{};
    c.fieldId = kY;
    c.bottomId = kB;
    c.outerId = kR;
    c.yellowId = kY;
    c.mode = m;
    return c;
}

// Sweep the flipper 0 -> `peak` -> 0 at 300 deg/s with the robot's front `front` in from the wall, then rest 1 s.
void stroke(TogglePlant& t, double peak, double front) {
    double deg = 0.0;
    while (deg < peak) {
        deg = deg + 3.0 > peak ? peak : deg + 3.0;
        t.step(0.01, deg, front);
    }
    while (deg > 0.0) {
        deg = deg - 3.0 < 0.0 ? 0.0 : deg - 3.0;
        t.step(0.01, deg, front);
    }
    for (int i = 0; i < 100; ++i) {
        t.step(0.01, 0.0, front);
    }
}

// The colour ids a camera 16 in from the +x wall (lens 10 in up, square) sees in one read.
unsigned seenIds(TogglePlant& t) {
    shulib::sim::TrueCamera cam{};
    cam.forward = 7.0;
    cam.height = 10.0;
    CameraPlant plant{cam, 1};
    t.draw(plant);
    const shulib::hal::VisionFrame f = plant.read(70.2 - 16.0 - 7.0, 0.0, 0.0, shulib::units::Time{1.0});
    unsigned ids = 0;
    for (std::int32_t i = 0; i < f.count; ++i) {
        ids |= 1U << f.detections[i].id;
    }
    return ids;
}
}  // namespace

TEST_CASE("AF1S-A1 T12 host Toggle: ONE face per full stroke (field <- bottom), seated only after the seat time") {
    TogglePlant t{redSide()};
    CHECK(t.fieldFace() == kY);
    CHECK(t.seated());
    CHECK(t.scoredColour() == kY);
    // Mid-roll: not seated, scored yellow.
    double deg = 0.0;
    while (deg < 90.0) {
        deg += 3.0;
        t.step(0.01, deg, 4.0);
    }
    CHECK(t.strokes() == 1);
    CHECK_FALSE(t.seated());
    CHECK(t.scoredColour() == kY);
    stroke(t, 0.0, 4.0);  // back down and rest
    CHECK(t.seated());
    CHECK(t.fieldFace() == kB);  // the bottom face came up into the field
    CHECK(t.facesRolled() == 1);
    // The second stroke brings the outer face (red) — the red side's Y -> B -> R.
    stroke(t, 90.0, 4.0);
    CHECK(t.fieldFace() == kR);
    CHECK(t.scoredColour() == kR);
    CHECK(t.strokes() == 2);
}

TEST_CASE("AF1S-A1 T12 host Toggle: a SHORT stroke (peak under rollDeg) and an AIR push (front past reachIn) roll "
          "nothing; a stroke only re-arms below rearmDeg") {
    TogglePlant t{redSide()};
    stroke(t, 55.0, 4.0);  // past contact (30), short of 60
    CHECK(t.strokes() == 0);
    CHECK(t.fieldFace() == kY);
    stroke(t, 90.0, 6.5);  // reach is 6
    CHECK(t.strokes() == 0);
    // Up past 60, down only to 20 (not re-armed), up again: ONE roll.
    for (double d = 0.0; d <= 90.0; d += 3.0) {
        t.step(0.01, d, 4.0);
    }
    for (double d = 90.0; d >= 20.0; d -= 3.0) {
        t.step(0.01, d, 4.0);
    }
    for (double d = 20.0; d <= 90.0; d += 3.0) {
        t.step(0.01, d, 4.0);
    }
    CHECK(t.strokes() == 1);
}

TEST_CASE("AF1S-A1 T12 host Toggle: OVER-ROLL rolls two faces; STICKY rolls none and stops an attached arm where "
          "it meets the Toggle; rollNow() is our other robot's stroke") {
    TogglePlant over{redSide(ToggleMode::OverRoll)};
    stroke(over, 90.0, 4.0);
    CHECK(over.fieldFace() == kR);
    CHECK(over.facesRolled() == 2);

    shulib::hal::fake::FakeMotor m;
    shulib::hal::fake::FakeMotor* ms[1]{&m};
    shulib::sim::ArmPlantConfig ac{};
    ac.minStopDeg = -2.0;
    ac.maxStopDeg = 125.0;
    shulib::sim::ArmPlant arm{ms, ac};
    TogglePlant sticky{redSide(ToggleMode::Sticky)};
    sticky.attachArm(arm);
    m.setVoltage(shulib::units::Voltage{12.0});
    for (int i = 0; i < 200; ++i) {
        arm.step(0.01);
        sticky.step(0.01, arm.trueDeg(), 4.0);
    }
    CHECK(arm.trueDeg() <= 31.0);  // stopped just past contactDeg
    CHECK(arm.trueDeg() >= 25.0);
    CHECK(sticky.fieldFace() == kY);
    CHECK(sticky.touched());  // the flipper is on it
    // Off the Toggle (front past reach): the plant clears ITS obstacle, and the arm moves on.
    for (int i = 0; i < 200; ++i) {
        arm.step(0.01);
        sticky.step(0.01, arm.trueDeg(), 10.0);
    }
    CHECK(arm.trueDeg() > 60.0);

    TogglePlant other{redSide()};
    other.rollNow();
    for (int i = 0; i < 50; ++i) {
        other.step(0.01, 0.0, 30.0);
    }
    CHECK(other.fieldFace() == kB);
}

TEST_CASE("AF1S-A1 T12 host Toggle: PASSIVE — the front coming within passiveRollIn is the stroke; it re-arms only "
          "past passiveRollIn + passiveRearmIn") {
    TogglePlantConfig c = redSide();
    c.passive = true;
    TogglePlant t{c};
    for (double f = 16.0; f >= 4.0; f -= 0.1) {
        t.step(0.01, 0.0, f);
    }
    CHECK(t.strokes() == 1);
    CHECK(t.touched());  // the bar is on it
    for (double f = 4.0; f <= 7.5; f += 0.1) {  // out to 7.5 (< 5 + 3): not re-armed
        t.step(0.01, 0.0, f);
    }
    for (double f = 7.5; f >= 4.0; f -= 0.1) {
        t.step(0.01, 0.0, f);
    }
    CHECK(t.strokes() == 1);
    for (double f = 4.0; f <= 16.0; f += 0.1) {
        t.step(0.01, 0.0, f);
    }
    CHECK_FALSE(t.touched());
    for (double f = 16.0; f >= 4.0; f -= 0.1) {
        t.step(0.01, 0.0, f);
    }
    CHECK(t.strokes() == 2);
    for (int i = 0; i < 50; ++i) {
        t.step(0.01, 0.0, 16.0);
    }
    CHECK(t.fieldFace() == kR);
}

TEST_CASE("AF1S-A1 T12 host Toggle: JAMMED part-way it never seats, and scores YELLOW whatever the camera sees") {
    TogglePlantConfig c = redSide(ToggleMode::Jammed);
    c.jamDeg = 70.0;
    TogglePlant t{c};
    stroke(t, 90.0, 4.0);
    for (int i = 0; i < 300; ++i) {
        t.step(0.01, 0.0, 16.0);
    }
    CHECK(t.jammed());
    CHECK_FALSE(t.seated());
    CHECK(t.scoredColour() == kY);
    CHECK(t.fieldFace() == kY);  // the faces never rotated
    stroke(t, 90.0, 4.0);        // a jammed Toggle takes no new roll
    CHECK(t.facesRolled() == 0);
}

TEST_CASE("AF1S-A1 T12 host Toggle: scoredColour() is YELLOW while anything of the robot touches it (<SC4>b) — the "
          "flipper past contactDeg within reach, or the front within touchIn") {
    TogglePlantConfig c = redSide();
    c.fieldId = kR;
    c.bottomId = kY;
    c.outerId = kB;
    TogglePlant t{c};
    t.step(0.01, 0.0, 16.0);
    CHECK(t.scoredColour() == kR);
    t.step(0.01, 35.0, 5.0);  // the flipper on it
    CHECK(t.touched());
    CHECK(t.scoredColour() == kY);
    t.step(0.01, 0.0, 1.0);  // the front on it
    CHECK(t.scoredColour() == kY);
    t.step(0.01, 35.0, 8.0);  // the flipper out, but beyond reach
    CHECK_FALSE(t.touched());
    CHECK(t.scoredColour() == kR);
}

TEST_CASE("AF1S-A1 T12 host Toggle: contacts while its SEATED face is ours are counted (T1's oracle)") {
    TogglePlantConfig c = redSide();
    c.fieldId = kR;
    c.bottomId = kY;
    c.outerId = kB;
    TogglePlant t{c};
    t.setOurs(kR);
    stroke(t, 40.0, 4.0);  // touches, short of a roll
    CHECK(t.flipperContactsWhileOurs() == 1);
    TogglePlant y{redSide()};
    y.setOurs(kR);
    stroke(y, 40.0, 4.0);
    CHECK(y.flipperContactsWhileOurs() == 0);
}

TEST_CASE("AF1S-A1 T12 host Toggle: the camera model is shown the TRUE faces after each change") {
    TogglePlant t{redSide()};
    CHECK((seenIds(t) & (1U << kY)) != 0U);
    CHECK((seenIds(t) & (1U << kB)) != 0U);  // the bottom face, seen from below
    CHECK((seenIds(t) & (1U << kR)) == 0U);  // the outer face faces out of the field
    stroke(t, 90.0, 4.0);
    CHECK(t.fieldFace() == kB);
    CHECK((seenIds(t) & (1U << kB)) != 0U);
    CHECK((seenIds(t) & (1U << kR)) != 0U);  // red is the bottom face now
    CHECK((seenIds(t) & (1U << kY)) == 0U);  // yellow went out of the field
}
