// vision::override2026 — the 2026-27 Override season's data the camera code carries (chunk VIS1-A1, the
// VIS1 brief's §1.10). DATA, so the tests pin it two ways: every number against the manual's own figure
// (Game Manual v2.0 — v2.1's PDF answered 404 on 2026-10-22), and every INFERRED number against the
// arithmetic that produced it, re-derived here from the drawing's figures (pA8's end view: the axle
// 13.20 in up, the apex 30.2 mm above it, 51.6 mm from the apex to the flat bottom face; the glossary's
// 52.2-mm faces) — so a hand-edit of one number that breaks the geometry it came from goes red.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "shulib/vision/override_elements.hpp"

namespace ov = shulib::vision::override2026;

namespace {
constexpr double kMmPerIn = 25.4;
constexpr double kAxleIn = 13.20;        // pA8 side view
constexpr double kApexAboveAxleMm = 30.2;  // pA8 end view
constexpr double kApexToFlatMm = 51.6;     // pA8 end view (a VERTICAL dimension)
constexpr double kFaceWidthMm = 52.2;      // the glossary: "each face ... approximately 2.05 in (52.2 mm) wide"
}  // namespace

TEST_CASE("VIS1-A1 Override data: the Toggle's face heights follow from pA8 and the glossary") {
    const ov::ToggleFace& f = ov::kToggleFace;
    // The apex: the axle + 30.2 mm.
    CHECK(f.topIn == doctest::Approx(kAxleIn + kApexAboveAxleMm / kMmPerIn).epsilon(0.001));
    // The flat bottom face: the apex less the 51.6-mm end view.
    CHECK(f.flatBottomIn == doctest::Approx(f.topIn - kApexToFlatMm / kMmPerIn).epsilon(0.001));
    // The colour boundaries: corner middles 30.2 mm from the axle, 120 degrees apart — 52.3 mm apart, the
    // glossary's 52.2 within a millimetre (the reading is consistent).
    const double chordMm = kApexAboveAxleMm * std::sqrt(3.0);
    CHECK(std::fabs(chordMm - kFaceWidthMm) < 1.0);
    // The field face's lower boundary: 30.2 x sin 30 mm below the axle; its set-back 30.2 x cos 30 mm.
    const double lowerEdgeIn = kAxleIn - kApexAboveAxleMm * 0.5 / kMmPerIn;
    const double setBackIn = kApexAboveAxleMm * std::cos(30.0 * 3.14159265358979323846 / 180.0) / kMmPerIn;
    CHECK(lowerEdgeIn == doctest::Approx(12.61).epsilon(0.001));
    CHECK(setBackIn == doctest::Approx(1.03).epsilon(0.002));
    // The carried INTERVALS hold the arithmetic and are narrow (the brief's re-check S10: [12.55, 12.65]
    // and [0.9, 1.2] — a widening past them is a decision, not an edit).
    CHECK(f.bottomIn <= lowerEdgeIn);
    CHECK(f.bottomHiIn >= lowerEdgeIn);
    CHECK(f.bottomIn == doctest::Approx(12.55));
    CHECK(f.bottomHiIn == doctest::Approx(12.65));
    CHECK(f.setBackLoIn <= setBackIn);
    CHECK(f.setBackHiIn >= setBackIn);
    CHECK(f.setBackLoIn == doctest::Approx(0.9));
    CHECK(f.setBackHiIn == doctest::Approx(1.2));
    // The flat bottom face is BELOW the field face's lower boundary (VIS1-A1's reading of the drawing).
    CHECK(f.flatBottomIn < f.bottomIn);
    CHECK(f.bottomIn - f.flatBottomIn == doctest::Approx(0.19).epsilon(0.1));
    CHECK(f.topIn > f.bottomHiIn);
    // The length and the wall.
    CHECK(f.lengthIn == doctest::Approx(25.8));
    CHECK(f.wallTopIn == doctest::Approx(11.5));
    CHECK(f.wallTopIn < f.flatBottomIn);  // the whole Toggle sits above the wall's top
}

TEST_CASE("VIS1-A1 Override data: one Toggle centred on each wall, 70.20 in out, facing INTO the field") {
    int seen = 0;
    for (const ov::TogglePlace& p : ov::kTogglePlaces) {
        ++seen;
        CHECK(std::hypot(p.x, p.y) == doctest::Approx(70.2));
        CHECK(std::hypot(p.inX, p.inY) == doctest::Approx(1.0));
        // The inward vector points back at the field's centre.
        CHECK(p.inX * p.x + p.inY * p.y == doctest::Approx(-70.2));
        // Centred on its wall: one coordinate is zero.
        CHECK(p.x * p.y == doctest::Approx(0.0));
    }
    CHECK(seen == 4);
    // All four walls, no wall twice.
    CHECK(ov::kTogglePlaces[0].x == doctest::Approx(70.2));
    CHECK(ov::kTogglePlaces[1].x == doctest::Approx(-70.2));
    CHECK(ov::kTogglePlaces[2].y == doctest::Approx(70.2));
    CHECK(ov::kTogglePlaces[3].y == doctest::Approx(-70.2));
}

TEST_CASE("VIS1-A1 Override data: the model's class table, its names and sizes") {
    REQUIRE(ov::kModelClassCount == 5);
    const char* expected[5] = {"redBluePin", "blueYellowPin", "redYellowPin", "yellowPin", "cup"};
    for (std::uint8_t i = 0; i < 5; ++i) {
        CAPTURE(static_cast<int>(i));
        CHECK(std::string{ov::kOverrideClassNames[i]} == expected[i]);
        CHECK(std::string{ov::modelClassName(i)} == expected[i]);
        CHECK(ov::modelClassName(i) == ov::kOverrideClassNames[i]);  // one table, never two
        // Every name fits the camera's 20-byte buffer with its NUL.
        CHECK(std::strlen(ov::kOverrideClassNames[i]) < 20U);
    }
    CHECK(ov::modelClassName(5) == nullptr);
    CHECK(ov::modelClassName(255) == nullptr);
    CHECK(static_cast<int>(ov::ModelClass::RedBluePin) == 0);
    CHECK(static_cast<int>(ov::ModelClass::YellowPin) == 3);
    CHECK(static_cast<int>(ov::ModelClass::Cup) == 4);
    for (std::uint8_t i = 0; i < 4; ++i) {
        CHECK(ov::modelClassSize(i) == &ov::kPinSize);
    }
    CHECK(ov::modelClassSize(4) == &ov::kCupSize);
    CHECK(ov::modelClassSize(5) == nullptr);
    CHECK(ov::kPinSize.diameterIn == doctest::Approx(1.6));
    CHECK(ov::kPinSize.heightIn == doctest::Approx(6.5));
    CHECK(ov::kCupSize.diameterIn == doctest::Approx(3.15));
    CHECK(ov::kCupSize.heightIn == doctest::Approx(6.5));
}
