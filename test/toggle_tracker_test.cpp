// vision::ToggleTracker on HAND-MADE frames — every rule's edge landed on exactly (chunk VIS1-A2; the VIS1
// brief's §1.12 and §2.2 "the tracker ... plus unit cases on hand-made frames"). The scenes against the host
// camera model are toggle_tracker_scenes_test.cpp.
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * EVERY config refusal by its own words, and the first accepted value beside each refused one (an id of
//     0 and 8 refused, 1 and 7 accepted; minAspect 1 refused, the next double above it accepted; equal
//     length fractions or range ends refused; the position gate's two companions).
//   * The words: every colour and reason ASCII, distinct, and a byte outside the enum never a colour.
//   * explainAsToggleFace(): the minimum height LANDED ON (2 x kBoxEdgePx is too thin, one more pixel is
//     measured); the CUT rows landed on (top 1 cut, 2 not; bottom 238 cut, 237 not); a side clip that spans
//     the centre column is measured (no length), one that does not is cut; the true range and 25.8 in held.
//   * The tracker's rules on frames built by hand from one real face's box: positive evidence only (every
//     non-Ok status, a POSSIBLY FULL read); the hold and the alive time LANDED ON (equal is enough, the next
//     double is not); kind and id; the aspect floor landed on; the range window's OVERLAP and the bearing
//     window's CONTAINMENT each landed on; two colours; the same colour twice (the nearer); the colour-above
//     veto's reach landed on (2 x kBoxEdgePx above: vetoed; one pixel more: not; columns that only touch:
//     not); a cut band of another colour beside a passing one (in the window: vetoed; far outside: not);
//     the frozen check's move and turn thresholds landed on (equal is not frozen, the next double is), and
//     recovery restarting the hold; the position gate (a loose pose skips it, a tight one names the Toggle,
//     a vague fix fails it).

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "camera_scene_rig.hpp"

namespace {

namespace hal = camrig::hal;
namespace vision = camrig::vision;
using camrig::Time;
using hal::Detection;
using hal::VisionFrame;
using vision::ToggleColour;
using vision::ToggleFaceGate;
using vision::ToggleTracker;
using vision::ToggleTrackerConfig;
using vision::ToggleView;
using vision::ToggleWhy;

// The one real face every hand-made frame starts from: a RED field face 30 in from the lens of an 8-in level
// camera, seen by a TRUE camera EQUAL to the believed one, no jitter (so the hand edits below are the only
// differences). The robot sits at robotXFor(30), heading 0.
Detection realFace() {
    camrig::sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd;
    tc.height = 8.0;
    camrig::sim::CameraPlant plant{tc, 1};
    plant.addQuad(camrig::face(1));
    const VisionFrame f = plant.read(camrig::robotXFor(30.0), 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    return f.detections[0];
}

double faceRobotX() { return camrig::robotXFor(30.0); }

vision::PoseBound atFace(double dx = 0.0, double headingRad = 0.0) {
    return camrig::poseAt(faceRobotX() + dx, 0.0, headingRad);
}

VisionFrame frame(std::initializer_list<Detection> ds, double at) {
    VisionFrame f{};
    f.status = hal::FrameStatus::Ok;
    f.model = hal::ModelCheck::Matches;
    f.readAt = Time{at};
    for (const Detection& d : ds) {
        f.detections[f.count++] = d;
    }
    f.reportedCount = f.count;
    return f;
}

// Feed the same frame `n` times, `dt` apart from `t0`, from the face pose; the view at the last read.
ToggleView feed(ToggleTracker& tr, std::initializer_list<Detection> ds, int n, double t0 = 1.0, double dt = 0.0625) {
    double t = t0;
    for (int i = 0; i < n; ++i) {
        tr.update(frame(ds, t), atFace());
        t += dt;
    }
    return tr.view(t - dt);
}

vision::CameraModel model8() { return camrig::model(8.0); }

ToggleTrackerConfig cfg() {
    ToggleTrackerConfig c = camrig::tcfg();
    c.holdSeconds = 0.25;   // binary-exact, so the hold's edge can be landed on
    c.aliveSeconds = 0.125;
    return c;
}

bool ascii(const char* s) {
    for (; *s != '\0'; ++s) {
        if (static_cast<unsigned char>(*s) >= 0x80U || static_cast<unsigned char>(*s) < 0x20U) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("toggle tracker: every config refusal by its words; the first accepted value beside each refused one") {
    const ToggleTrackerConfig good = camrig::tcfg();
    REQUIRE(vision::toggleTrackerConfigProblem(good) == nullptr);
    CHECK(vision::toggleTrackerConfigProblem(ToggleTrackerConfig{}) != nullptr);  // every default refused
    auto refusal = [&](void (*edit)(ToggleTrackerConfig&)) {
        ToggleTrackerConfig c = good;
        edit(c);
        const char* p = vision::toggleTrackerConfigProblem(c);
        return p == nullptr ? std::string{"ACCEPTED"} : std::string{p};
    };
    const std::string unsetId = "a Toggle colour id is UNSET -- the descriptor ids the team taught the camera";
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids.red = 0; }) == unsetId);
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids.blue = 0; }) == unsetId);
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids.yellow = 0; }) == unsetId);
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids.red = 8; }) == "a Toggle colour id is outside 1..7");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids.yellow = 8; }) == "a Toggle colour id is outside 1..7");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids = {7, 1, 2}; }) == "ACCEPTED");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids = {2, 2, 3}; }) == "two Toggle colours share one descriptor id");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids = {1, 2, 1}; }) == "two Toggle colours share one descriptor id");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.ids = {1, 3, 3}; }) == "two Toggle colours share one descriptor id");
    // every number non-finite, one at a time
    const std::string notFinite = "a tracker number is not a finite number";
    double ToggleTrackerConfig::*const numbers[] = {
        &ToggleTrackerConfig::holdSeconds,         &ToggleTrackerConfig::aliveSeconds,
        &ToggleTrackerConfig::minAspect,           &ToggleTrackerConfig::minLengthFraction,
        &ToggleTrackerConfig::maxLengthFraction,   &ToggleTrackerConfig::minRangeIn,
        &ToggleTrackerConfig::maxRangeIn,          &ToggleTrackerConfig::bearingWindowDeg,
        &ToggleTrackerConfig::frozenMoveIn,        &ToggleTrackerConfig::frozenTurnDeg,
        &ToggleTrackerConfig::positionToleranceIn, &ToggleTrackerConfig::maxPoseToleranceIn,
        &ToggleTrackerConfig::maxFixRadiusIn};
    for (double ToggleTrackerConfig::*const m : numbers) {
        for (const double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
            ToggleTrackerConfig c = good;
            c.*m = bad;
            const char* p = vision::toggleTrackerConfigProblem(c);
            REQUIRE(p != nullptr);
            CHECK(std::string{p} == notFinite);
        }
    }
    CHECK(refusal([](ToggleTrackerConfig& c) { c.holdSeconds = 0.0; }) ==
          "holdSeconds UNSET -- how long a colour must hold before it is reported");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.aliveSeconds = -1.0; }) ==
          "aliveSeconds UNSET -- how old the newest usable read may be");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.minAspect = 1.0; }) ==
          "minAspect UNSET or not above 1 (a band is wider than tall)");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.minAspect = std::nextafter(1.0, 2.0); }) == "ACCEPTED");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.minLengthFraction = 0.0; }) == "length fractions UNSET or reversed");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.maxLengthFraction = c.minLengthFraction; }) ==
          "length fractions UNSET or reversed");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.minRangeIn = 0.0; }) == "range window UNSET or reversed");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.maxRangeIn = c.minRangeIn; }) == "range window UNSET or reversed");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.bearingWindowDeg = 0.0; }) == "bearing window UNSET");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.frozenMoveIn = 0.0; }) ==
          "frozenMoveIn UNSET -- how far the lens moves before an unmoved box means a frozen camera");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.frozenTurnDeg = 0.0; }) ==
          "frozenTurnDeg UNSET -- how far the robot turns before an unmoved box means a frozen camera");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.positionToleranceIn = -0.5; }) == "position tolerance negative");
    CHECK(refusal([](ToggleTrackerConfig& c) { c.positionToleranceIn = 3.0; }) ==
          "the position gate is on but the pose bound it trusts is UNSET");
    CHECK(refusal([](ToggleTrackerConfig& c) {
              c.positionToleranceIn = 3.0;
              c.maxPoseToleranceIn = 6.0;
          }) == "the position gate is on but the vaguest position it accepts is UNSET");
    CHECK(refusal([](ToggleTrackerConfig& c) {
              c.positionToleranceIn = 3.0;
              c.maxPoseToleranceIn = 6.0;
              c.maxFixRadiusIn = 8.0;
          }) == "ACCEPTED");
}

TEST_CASE("toggle tracker: the words -- every colour and reason ASCII and distinct; a byte outside the enum is "
          "never a colour") {
    const ToggleColour colours[] = {ToggleColour::Unknown, ToggleColour::Red, ToggleColour::Blue,
                                    ToggleColour::Yellow};
    const char* expected[] = {"UNKNOWN", "RED", "BLUE", "YELLOW"};
    for (int i = 0; i < 4; ++i) {
        CHECK(std::string{vision::toggleColourWord(colours[i])} == expected[i]);
    }
    CHECK(std::string{vision::toggleColourWord(static_cast<ToggleColour>(9))} == "UNKNOWN");
    for (int a = 0; a <= static_cast<int>(ToggleWhy::ColourAbove); ++a) {
        const char* wa = vision::toggleWhyWords(static_cast<ToggleWhy>(a));
        CHECK(ascii(wa));
        CHECK(std::string{wa} != "?");
        for (int b = a + 1; b <= static_cast<int>(ToggleWhy::ColourAbove); ++b) {
            CHECK(std::string{wa} != vision::toggleWhyWords(static_cast<ToggleWhy>(b)));
        }
    }
    CHECK(std::string{vision::toggleWhyWords(static_cast<ToggleWhy>(200))} == "?");
}

TEST_CASE("toggle tracker: explainAsToggleFace -- the true range and 25.8 in held; the minimum height and the cut "
          "rows LANDED ON; a side clip measured only when it spans the centre column") {
    const vision::CameraModel m = model8();
    const Detection d = realFace();
    const vision::ToggleFaceMeasure ok = vision::explainAsToggleFace(d, m);
    REQUIRE(ok.gate == ToggleFaceGate::Explained);
    CHECK_FALSE(ok.sideClipped);
    CHECK(ok.range.contains(30.0));                       // the lens to the lower edge, horizontally
    CHECK(ok.impliedLength.contains(camrig::kFace.lengthIn));
    CHECK(ok.aspect == doctest::Approx(static_cast<double>(d.width) / d.height));
    // the minimum height: 2 x kBoxEdgePx (4 px) is too thin; 5 px is measured (explained or not)
    Detection thin = d;
    thin.height = static_cast<std::int16_t>(2.0 * vision::kBoxEdgePx);
    CHECK(vision::explainAsToggleFace(thin, m).gate == ToggleFaceGate::TooThin);
    thin.height = static_cast<std::int16_t>(thin.height + 1);
    CHECK(vision::explainAsToggleFace(thin, m).gate != ToggleFaceGate::TooThin);
    Detection empty = d;
    empty.width = 0;
    CHECK(vision::explainAsToggleFace(empty, m).gate == ToggleFaceGate::TooThin);
    // the cut rows, within one pixel of the picture's 0..240 extent: a top at row 1 is cut, row 2 is not; a
    // bottom edge (top + height) at 239 is cut, 238 is not
    Detection top = d;
    top.top = 1;
    CHECK(vision::explainAsToggleFace(top, m).gate == ToggleFaceGate::Cut);
    top.top = 2;
    CHECK(vision::explainAsToggleFace(top, m).gate != ToggleFaceGate::Cut);
    Detection bottom = d;
    bottom.top = static_cast<std::int16_t>(239 - bottom.height);
    CHECK(vision::explainAsToggleFace(bottom, m).gate == ToggleFaceGate::Cut);
    bottom.top = static_cast<std::int16_t>(238 - bottom.height);
    CHECK(vision::explainAsToggleFace(bottom, m).gate != ToggleFaceGate::Cut);
    // a side clip: from the left edge to the centre column -> measured, no length; one pixel short -> cut
    Detection side = d;
    side.left = 0;
    side.width = 160;
    const vision::ToggleFaceMeasure spans = vision::explainAsToggleFace(side, m);
    CHECK(spans.sideClipped);
    CHECK(spans.gate != ToggleFaceGate::Cut);
    CHECK_FALSE(spans.impliedLength.valid);
    side.width = 159;
    CHECK(vision::explainAsToggleFace(side, m).gate == ToggleFaceGate::Cut);
    Detection right = d;  // reaching the right edge (left + width >= 319), spanning the centre from the right
    right.left = 160;
    right.width = 159;
    CHECK(vision::explainAsToggleFace(right, m).sideClipped);
    CHECK(vision::explainAsToggleFace(right, m).gate != ToggleFaceGate::Cut);
    right.left = 161;
    right.width = 158;
    CHECK(vision::explainAsToggleFace(right, m).gate == ToggleFaceGate::Cut);
}

TEST_CASE("toggle tracker: positive evidence only -- no read, each non-Ok status, and a POSSIBLY FULL read are "
          "never a colour, and each breaks the hold") {
    const Detection d = realFace();
    ToggleTracker tr{cfg(), model8()};
    CHECK(tr.view(0.0).why == ToggleWhy::NoRead);
    REQUIRE(feed(tr, {d}, 5).colour == ToggleColour::Red);  // 0.25 s held
    for (const hal::FrameStatus s : {hal::FrameStatus::ReadFault, hal::FrameStatus::PartialFault,
                                     hal::FrameStatus::NotConfigured, hal::FrameStatus::NotRead}) {
        CAPTURE(static_cast<int>(s));
        ToggleTracker t2{cfg(), model8()};
        REQUIRE(feed(t2, {d}, 5).colour == ToggleColour::Red);
        VisionFrame bad = frame({d}, 1.3125);
        bad.status = s;  // the detection is still in the array: a non-Ok read is ignored whole
        t2.update(bad, atFace());
        const ToggleView v = t2.view(1.3125);
        CHECK(v.colour == ToggleColour::Unknown);
        CHECK(v.why == ToggleWhy::ReadNotUsable);
        // the next good read starts a NEW hold
        t2.update(frame({d}, 1.375), atFace());
        CHECK(t2.view(1.375).why == ToggleWhy::NotHeldYet);
    }
    ToggleTracker t3{cfg(), model8()};
    REQUIRE(feed(t3, {d}, 5).colour == ToggleColour::Red);
    VisionFrame full = frame({d}, 1.3125);
    full.truncated = true;
    t3.update(full, atFace());
    CHECK(t3.view(1.3125).why == ToggleWhy::CameraFull);
    CHECK(t3.view(1.3125).colour == ToggleColour::Unknown);
    t3.update(frame({d}, 1.375), atFace());
    CHECK(t3.view(1.375).why == ToggleWhy::NotHeldYet);
}

TEST_CASE("toggle tracker: the hold and the alive time LANDED ON -- held exactly holdSeconds is sure, a read "
          "exactly aliveSeconds old is alive; the next double past either is not") {
    const Detection d = realFace();
    ToggleTracker tr{cfg(), model8()};
    tr.update(frame({d}, 1.0), atFace());
    tr.update(frame({d}, std::nextafter(1.25, 0.0)), atFace());
    CHECK(tr.view(1.25).why == ToggleWhy::NotHeldYet);  // a hair under 0.25 s
    tr.update(frame({d}, 1.25), atFace());
    ToggleView v = tr.view(1.25);
    CHECK(v.colour == ToggleColour::Red);  // exactly 0.25 s
    CHECK(v.heldSeconds == 0.25);
    CHECK(tr.view(1.375).colour == ToggleColour::Red);  // the read exactly 0.125 s old
    v = tr.view(std::nextafter(1.375, 2.0));
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::CameraSilent);
    CHECK(tr.view(std::numeric_limits<double>::quiet_NaN()).why == ToggleWhy::CameraSilent);  // never "alive"
}

TEST_CASE("toggle tracker: KIND and ID -- a Code or an Object carrying the red id, and a colour id no Toggle uses, "
          "are ignored") {
    Detection code = realFace();
    code.kind = hal::DetectionKind::Code;
    Detection object = realFace();
    object.kind = hal::DetectionKind::Object;
    Detection other = realFace();
    other.id = 4;
    for (const Detection& d : {code, object, other}) {
        ToggleTracker tr{cfg(), model8()};
        const ToggleView v = feed(tr, {d}, 6);
        CHECK(v.colour == ToggleColour::Unknown);
        CHECK(v.why == ToggleWhy::NoBand);
    }
    // the same box with each Toggle id reads that colour
    const ToggleColour expect[] = {ToggleColour::Red, ToggleColour::Blue, ToggleColour::Yellow};
    for (std::uint8_t id = 1; id <= 3; ++id) {
        Detection d = realFace();
        d.id = id;
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {d}, 6).colour == expect[id - 1]);
    }
}

TEST_CASE("toggle tracker: the SHAPE floor LANDED ON -- width over height exactly minAspect passes, the next "
          "double above it does not") {
    const Detection d = realFace();
    const double aspect = static_cast<double>(d.width) / d.height;
    ToggleTrackerConfig at = cfg();
    at.minAspect = aspect;
    ToggleTracker t1{at, model8()};
    CHECK(feed(t1, {d}, 6).colour == ToggleColour::Red);
    ToggleTrackerConfig over = cfg();
    over.minAspect = std::nextafter(aspect, 100.0);
    ToggleTracker t2{over, model8()};
    const ToggleView v = feed(t2, {d}, 6);
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::NoBand);
}

TEST_CASE("toggle tracker: WHERE -- the range window is an OVERLAP and the bearing window a CONTAINMENT, each "
          "LANDED ON") {
    const Detection d = realFace();
    ToggleTracker probe{cfg(), model8()};
    const ToggleView ref = feed(probe, {d}, 6);
    REQUIRE(ref.colour == ToggleColour::Red);
    REQUIRE(ref.range.valid);
    REQUIRE(ref.bearing.valid);
    // the range: a window ending exactly at the band's NEAR end still overlaps it; the next double below does not
    ToggleTrackerConfig r1 = cfg();
    r1.maxRangeIn = ref.range.lo;
    ToggleTracker t1{r1, model8()};
    CHECK(feed(t1, {d}, 6).colour == ToggleColour::Red);
    ToggleTrackerConfig r2 = cfg();
    r2.maxRangeIn = std::nextafter(ref.range.lo, 0.0);
    ToggleTracker t2{r2, model8()};
    CHECK(feed(t2, {d}, 6).why == ToggleWhy::NoBand);
    // a window starting exactly at the band's FAR end overlaps it; the next double above does not
    ToggleTrackerConfig r3 = cfg();
    r3.minRangeIn = ref.range.hi;
    r3.maxRangeIn = 200.0;
    ToggleTracker t3{r3, model8()};
    CHECK(feed(t3, {d}, 6).colour == ToggleColour::Red);
    ToggleTrackerConfig r4 = r3;
    r4.minRangeIn = std::nextafter(ref.range.hi, 200.0);
    ToggleTracker t4{r4, model8()};
    CHECK(feed(t4, {d}, 6).why == ToggleWhy::NoBand);
    // the bearing: the smallest window (degrees) whose radians reach the band's farthest end contains it; the
    // next double below does not
    const double edge = std::fmax(std::fabs(ref.bearing.lo), std::fabs(ref.bearing.hi));
    double winDeg = edge / vision::kDegToRad;
    while (winDeg * vision::kDegToRad < edge) {
        winDeg = std::nextafter(winDeg, 90.0);
    }
    while (std::nextafter(winDeg, 0.0) * vision::kDegToRad >= edge) {
        winDeg = std::nextafter(winDeg, 0.0);
    }
    ToggleTrackerConfig b1 = cfg();
    b1.bearingWindowDeg = winDeg;
    ToggleTracker t5{b1, model8()};
    CHECK(feed(t5, {d}, 6).colour == ToggleColour::Red);
    ToggleTrackerConfig b2 = cfg();
    b2.bearingWindowDeg = std::nextafter(winDeg, 0.0);
    ToggleTracker t6{b2, model8()};
    CHECK(feed(t6, {d}, 6).why == ToggleWhy::NoBand);
}

TEST_CASE("toggle tracker: two different colours passing -> 'two colours'; the same colour twice -> that colour, "
          "the view carrying the NEARER band") {
    const Detection near = realFace();
    // the same face 48 in out, from the host model, for a second (farther) band
    camrig::sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd;
    tc.height = 8.0;
    camrig::sim::CameraPlant plant{tc, 1};
    plant.addQuad(camrig::face(1, 0.0));
    const VisionFrame farFrame = plant.read(camrig::robotXFor(48.0), 0.0, 0.0, Time{0.0});
    REQUIRE(farFrame.count == 1);
    const Detection far = farFrame.detections[0];
    // (the tracker judges a band from the picture alone: the pose only feeds the frozen check and the
    // position gate, which is off here)
    ToggleTracker solo{cfg(), model8()};
    const ToggleView farAlone = feed(solo, {far}, 6);
    REQUIRE(farAlone.colour == ToggleColour::Red);
    ToggleTracker same{cfg(), model8()};
    const ToggleView v = feed(same, {far, near}, 6);
    CHECK(v.colour == ToggleColour::Red);
    CHECK(v.range.mid() < farAlone.range.mid());  // the nearer band's range, whichever came first
    ToggleTracker same2{cfg(), model8()};
    CHECK(feed(same2, {near, far}, 6).range.mid() == doctest::Approx(v.range.mid()));
    Detection blueFar = far;
    blueFar.id = 2;
    ToggleTracker two{cfg(), model8()};
    const ToggleView t = feed(two, {near, blueFar}, 6);
    CHECK(t.colour == ToggleColour::Unknown);
    CHECK(t.why == ToggleWhy::TwoColours);
}

TEST_CASE("toggle tracker: the COLOUR-ABOVE veto's reach LANDED ON -- another colour's box whose bottom is "
          "2 x kBoxEdgePx above the band's top vetoes it; one pixel higher does not; columns that only touch do "
          "not; the band's own colour above never does") {
    const Detection d = realFace();
    auto above = [&](std::uint8_t id, int gapPx, int left, int width) {
        Detection o = hal::Detection{};
        o.kind = hal::DetectionKind::Colour;
        o.id = id;
        o.left = static_cast<std::int16_t>(left);
        o.width = static_cast<std::int16_t>(width);
        o.height = 3;  // too thin to be judged itself: only the veto can see it
        o.top = static_cast<std::int16_t>(d.top - gapPx - o.height);
        return o;
    };
    const int reach = static_cast<int>(2.0 * vision::kBoxEdgePx);
    {
        ToggleTracker tr{cfg(), model8()};
        const ToggleView v = feed(tr, {d, above(3, reach, d.left, d.width)}, 6);
        CHECK(v.colour == ToggleColour::Unknown);
        CHECK(v.why == ToggleWhy::ColourAbove);
    }
    {
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {d, above(3, reach + 1, d.left, d.width)}, 6).colour == ToggleColour::Red);
    }
    {  // overlapping the band's top rows (bottom BELOW the band's top) still vetoes: "or lower"
        ToggleTracker tr{cfg(), model8()};
        Detection o = above(2, 0, d.left + 10, 20);
        o.top = static_cast<std::int16_t>(d.top - 1);
        o.height = 6;
        CHECK(feed(tr, {d, o}, 6).why == ToggleWhy::ColourAbove);
    }
    {  // columns that only TOUCH (its left at the band's right edge) do not overlap
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {d, above(3, 0, d.left + d.width, 30)}, 6).colour == ToggleColour::Red);
    }
    {  // one column of overlap is enough
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {d, above(3, 0, d.left + d.width - 1, 30)}, 6).why == ToggleWhy::ColourAbove);
    }
    {  // the band's OWN colour above never vetoes
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {d, above(1, 0, d.left, d.width)}, 6).colour == ToggleColour::Red);
    }
    {  // a non-Toggle colour above never vetoes
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {d, above(5, 0, d.left, d.width)}, 6).colour == ToggleColour::Red);
    }
}

TEST_CASE("toggle tracker: a passing band beside a CUT band of another colour -- vetoed when the cut band's "
          "bearing touches the window, not when it is certainly outside; beside a cut band of its own colour, "
          "never") {
    const Detection d = realFace();
    Detection cutInside = camrig::colourBox(2, 20, 0, 280, 40);  // touching the image's top, across the middle
    ToggleTracker t1{cfg(), model8()};
    const ToggleView v1 = feed(t1, {d, cutInside}, 6);
    CHECK(v1.colour == ToggleColour::Unknown);
    CHECK(v1.why == ToggleWhy::ConflictingPartialBand);
    // the same cut band far to the right of a 10-degree window: certainly elsewhere
    ToggleTrackerConfig narrow = cfg();
    narrow.bearingWindowDeg = 10.0;
    const Detection cutOutside = camrig::colourBox(2, 290, 0, 25, 40);
    ToggleTracker t2{narrow, model8()};
    CHECK(feed(t2, {d, cutOutside}, 6).colour == ToggleColour::Red);
    Detection cutOwn = cutInside;
    cutOwn.id = 1;
    ToggleTracker t3{cfg(), model8()};
    CHECK(feed(t3, {d, cutOwn}, 6).colour == ToggleColour::Red);
    // with nothing passing, the reasons in their order: maybe the bottom face, then cut, then no band
    ToggleTracker t4{cfg(), model8()};
    CHECK(feed(t4, {cutInside}, 2).why == ToggleWhy::CutByImage);
}

TEST_CASE("toggle tracker: the FROZEN check's thresholds LANDED ON -- the lens moved exactly frozenMoveIn or the "
          "robot turned exactly frozenTurnDeg is not frozen, the next double is; recovery restarts the hold") {
    const Detection d = realFace();
    ToggleTrackerConfig c = cfg();
    c.frozenMoveIn = 2.0;
    c.frozenTurnDeg = 2.0;
    const double x0 = 10.0;  // a pose the frozen check reads; the box never moves
    // The lens at the tracking centre, so the pose's next double IS the lens's (7 in forward, x0 + 2 + 7
    // rounds the pose's last bit away). The picture is judged on its own: the mount's forward offset feeds
    // only the frozen check and the position gate.
    auto centredModel = [] {
        camrig::config::CameraPlacement p = camrig::believed(8.0);
        p.forward = camrig::Length{0.0};
        return vision::CameraModel{vision::kAiVisionOptics, p};
    };
    {
        ToggleTracker tr{c, centredModel()};
        tr.update(frame({d}, 1.0), camrig::poseAt(x0, 0.0, 0.0));
        tr.update(frame({d}, 1.25), camrig::poseAt(x0 + 2.0, 0.0, 0.0));
        CHECK(tr.view(1.25).colour == ToggleColour::Red);  // moved exactly 2.0: not frozen
        tr.update(frame({d}, 1.3125), camrig::poseAt(std::nextafter(x0 + 2.0, 100.0), 0.0, 0.0));
        const ToggleView v = tr.view(1.3125);
        CHECK(v.colour == ToggleColour::Unknown);
        CHECK(v.why == ToggleWhy::CameraFrozen);
        CHECK_FALSE(v.positionChecked);
        // RECOVERY: the first moved box clears it and restarts the hold
        Detection moved = d;
        moved.left = static_cast<std::int16_t>(moved.left + 1);
        tr.update(frame({moved}, 1.375), camrig::poseAt(x0 + 3.0, 0.0, 0.0));
        CHECK(tr.view(1.375).why == ToggleWhy::NotHeldYet);
        tr.update(frame({moved}, 1.5625), camrig::poseAt(x0 + 3.0, 0.0, 0.0));
        CHECK(tr.view(1.5625).why == ToggleWhy::NotHeldYet);  // 0.1875 s since recovery
        tr.update(frame({moved}, 1.625), camrig::poseAt(x0 + 3.0, 0.0, 0.0));
        CHECK(tr.view(1.625).colour == ToggleColour::Red);  // 0.25 s since recovery, exactly
    }
    {
        const double turn = c.frozenTurnDeg * vision::kDegToRad;
        // with the lens 7 in forward a turn moves it too: centred, the turn check alone decides
        ToggleTracker centred{c, centredModel()};
        centred.update(frame({d}, 1.0), camrig::poseAt(x0, 0.0, 0.0));
        centred.update(frame({d}, 1.25), camrig::poseAt(x0, 0.0, turn));
        CHECK(centred.view(1.25).colour == ToggleColour::Red);  // turned exactly 2 degrees: not frozen
        centred.update(frame({d}, 1.3125), camrig::poseAt(x0, 0.0, std::nextafter(turn, 1.0)));
        CHECK(centred.view(1.3125).why == ToggleWhy::CameraFrozen);
        // the turn the other way, across the +-pi seam, is measured the short way round
        ToggleTracker seam{c, centredModel()};
        seam.update(frame({d}, 1.0), camrig::poseAt(x0, 0.0, camrig::kPi - 0.01));
        seam.update(frame({d}, 1.25), camrig::poseAt(x0, 0.0, -camrig::kPi + 0.01));  // 0.02 rad, not 6.26
        CHECK(seam.view(1.25).colour == ToggleColour::Red);
    }
    {  // a parked robot with a still box is never "frozen" -- nothing can tell
        ToggleTracker tr{c, model8()};
        CHECK(feed(tr, {d}, 20).colour == ToggleColour::Red);
    }
}

TEST_CASE("toggle tracker: the POSITION gate -- a loose pose skips it (unchecked), a tight one names the +x wall's "
          "Toggle, a vague fix or a band off every Toggle's line fails it") {
    const Detection d = realFace();
    ToggleTrackerConfig c = cfg();
    c.positionToleranceIn = 3.0;
    c.maxPoseToleranceIn = 6.0;
    c.maxFixRadiusIn = 12.0;  // this band's fix at 30 in is ~9.2 in wide (the edge agreement's range interval)
    {
        ToggleTracker tr{c, model8()};
        double t = 1.0;
        for (int i = 0; i < 6; ++i, t += 0.0625) {
            tr.update(frame({d}, t), camrig::poseAt(faceRobotX(), 0.0, 0.0, 1.0, 1.0));
        }
        const ToggleView v = tr.view(t - 0.0625);
        CHECK(v.colour == ToggleColour::Red);
        CHECK(v.positionChecked);
        CHECK(v.toggleIndex == 0);
    }
    {  // the pose bound looser than maxPoseToleranceIn: the gate is skipped, the colour still reported
        ToggleTracker tr{c, model8()};
        double t = 1.0;
        for (int i = 0; i < 6; ++i, t += 0.0625) {
            tr.update(frame({d}, t), camrig::poseAt(faceRobotX(), 0.0, 0.0, 6.5, 1.0));
        }
        const ToggleView v = tr.view(t - 0.0625);
        CHECK(v.colour == ToggleColour::Red);
        CHECK_FALSE(v.positionChecked);
        CHECK(v.toggleIndex == -1);
    }
    {  // the same picture from a pose 40 in off the Toggle's line: refused
        ToggleTracker tr{c, model8()};
        double t = 1.0;
        for (int i = 0; i < 6; ++i, t += 0.0625) {
            tr.update(frame({d}, t), camrig::poseAt(faceRobotX(), 40.0, 0.0, 1.0, 1.0));
        }
        CHECK(tr.view(t - 0.0625).colour == ToggleColour::Unknown);
    }
    {  // a fix vaguer than maxFixRadiusIn: refused
        ToggleTrackerConfig vague = c;
        vague.maxFixRadiusIn = 0.5;
        ToggleTracker tr{vague, model8()};
        double t = 1.0;
        for (int i = 0; i < 6; ++i, t += 0.0625) {
            tr.update(frame({d}, t), camrig::poseAt(faceRobotX(), 0.0, 0.0, 1.0, 1.0));
        }
        CHECK(tr.view(t - 0.0625).colour == ToggleColour::Unknown);
    }
}

// ─── VIS1-A2's review: the vetoes during a hold, gate 5's edges, the position gate's edges, the frozen view,
// the read's count, the length gate's edges, the same band ───────────────────────────────────────────

namespace {

// The real face 48 in out (the same host model as realFace()), for a second, farther band.
Detection farFace() {
    camrig::sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd;
    tc.height = 8.0;
    camrig::sim::CameraPlant plant{tc, 1};
    plant.addQuad(camrig::face(1, 0.0));
    const VisionFrame f = plant.read(camrig::robotXFor(48.0), 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    return f.detections[0];
}

// Gate 5's corner arithmetic, written again from its header (an oracle, not the tracker's code): for each
// corner and each lower-edge plane, is `d` explained as the BOTTOM face (with and without the depth's floor),
// and the lowest row its field face's apex would project to.
struct BottomOracle {
    bool explained = false;        // some corner, the depth 1..3 in
    bool explainedNoFloor = false;  // some corner, the depth at most 3 in
    double minApexRow = 1e9;        // over the corners that explain it (depth 1..3)
    double minApexRowNoFloor = 1e9;
};

BottomOracle bottomOracle(const vision::CameraModel& m, const Detection& d) {
    BottomOracle o{};
    const vision::override2026::ToggleFace& face = vision::override2026::kToggleFace;
    const double u = d.left + d.width / 2.0;
    const double e = vision::kBoxEdgePx;
    m.eachCorner(true, [&](const vision::CameraCorner& c) {
        for (const double plane : {face.bottomIn, face.bottomHiIn}) {
            double n1 = 0.0;
            double n2 = 0.0;
            double f1 = 0.0;
            double f2 = 0.0;
            if (!vision::CameraModel::rangeToPlane(c, u, d.top - e, plane, 0.5, n1)
                || !vision::CameraModel::rangeToPlane(c, u, d.top + e, plane, 0.5, n2)
                || !vision::CameraModel::rangeToPlane(c, u, d.top + d.height - e, plane, 0.5, f1)
                || !vision::CameraModel::rangeToPlane(c, u, d.top + d.height + e, plane, 0.5, f2)) {
                continue;
            }
            const double depthMax = std::fmax(f1, f2) - std::fmin(n1, n2);
            const double depthMin = std::fmin(f1, f2) - std::fmax(n1, n2);
            if (depthMin > vision::kBottomDepthHiIn) {
                continue;
            }
            double b = 0.0;
            double el = 0.0;
            vision::CameraModel::ray(c, u, d.top, b, el);
            const double r = std::fmax(n1, n2) + face.setBackHiIn;
            double pu = 0.0;
            double pv = -1e9;
            if (!m.project(c, m.placement().forward.value() + r * std::cos(b),
                           m.placement().left.value() + r * std::sin(b), face.topIn, pu, pv)) {
                pv = -1e9;
            }
            o.explainedNoFloor = true;
            o.minApexRowNoFloor = std::fmin(o.minApexRowNoFloor, pv);
            if (depthMax >= vision::kBottomDepthLoIn) {
                o.explained = true;
                o.minApexRow = std::fmin(o.minApexRow, pv);
            }
        }
    });
    return o;
}

}  // namespace

TEST_CASE("toggle tracker: a VETO read in the middle of a SURE hold breaks it at that read -- two colours, a cut "
          "band of another colour in the window, another colour directly above -- and the next clean read is "
          "'not held yet' (positive evidence only)") {
    const Detection d = realFace();
    Detection blueFar = farFace();
    blueFar.id = 2;
    const Detection cutBlue = camrig::colourBox(2, 20, 0, 280, 40);  // touching the image's top, across the middle
    Detection yellowAbove{};
    yellowAbove.kind = hal::DetectionKind::Colour;
    yellowAbove.id = 3;
    yellowAbove.left = d.left;
    yellowAbove.width = d.width;
    yellowAbove.height = 3;
    yellowAbove.top = static_cast<std::int16_t>(d.top - 3);
    auto broken = [&](const Detection& extra, ToggleWhy why) {
        ToggleTracker tr{cfg(), model8()};
        double t = 1.0;
        for (int i = 0; i < 5; ++i, t += 0.0625) {
            tr.update(frame({d}, t), atFace());
        }
        REQUIRE(tr.view(t - 0.0625).colour == ToggleColour::Red);  // held exactly 0.25 s: sure
        tr.update(frame({d, extra}, t), atFace());
        const ToggleView v = tr.view(t);
        CHECK(v.colour == ToggleColour::Unknown);
        CHECK(v.why == why);
        t += 0.0625;
        tr.update(frame({d}, t), atFace());
        const ToggleView after = tr.view(t);
        CHECK(after.colour == ToggleColour::Unknown);
        CHECK(after.why == ToggleWhy::NotHeldYet);
        CHECK(after.heldSeconds == 0.0);
    };
    broken(blueFar, ToggleWhy::TwoColours);
    broken(cutBlue, ToggleWhy::ConflictingPartialBand);
    broken(yellowAbove, ToggleWhy::ColourAbove);
}

TEST_CASE("toggle tracker: a CUT band of another colour that passes the SHAPE gate -- inside a 10-degree window "
          "it vetoes, certainly outside it does not; with nothing passing the reasons come in their order: maybe "
          "the bottom face, then cut, then no band") {
    const Detection d = realFace();
    ToggleTrackerConfig narrow = cfg();
    narrow.bearingWindowDeg = 10.0;
    // 30 x 8 (aspect 3.75 >= minAspect 3): it reaches the CUT gate, so only its bearing decides
    const Detection cutOutside = camrig::colourBox(2, 290, 0, 30, 8);
    const Detection cutInside = camrig::colourBox(2, 145, 0, 30, 8);
    REQUIRE(model8().bearing(305.0, 4.0).hi < -10.0 * vision::kDegToRad);  // certainly right of the window
    REQUIRE(model8().bearing(160.0, 4.0).lo < 0.0);
    {
        ToggleTracker tr{narrow, model8()};
        CHECK(feed(tr, {d, cutOutside}, 6).colour == ToggleColour::Red);
    }
    {
        ToggleTracker tr{narrow, model8()};
        const ToggleView v = feed(tr, {d, cutInside}, 6);
        CHECK(v.colour == ToggleColour::Unknown);
        CHECK(v.why == ToggleWhy::ConflictingPartialBand);
    }
    // the order: a band that may be the bottom face (an 8-in level lens, the real face 10 in out: its field
    // face's place above it is out of the frame) beside a cut band -> 'maybe the bottom face'; the cut band
    // alone -> 'cut'; nothing -> 'no band'
    const Detection maybeBottom = camrig::colourBox(1, 0, 24, 319, 13);
    {
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {maybeBottom}, 2).why == ToggleWhy::MaybeBottomFace);
    }
    {
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {maybeBottom, cutInside}, 2).why == ToggleWhy::MaybeBottomFace);
    }
    {
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {cutInside}, 2).why == ToggleWhy::CutByImage);
    }
    {
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {}, 2).why == ToggleWhy::NoBand);
    }
}

TEST_CASE("toggle tracker: gate 5's edges LANDED ON -- the box above's bottom row at the band's middle row "
          "(integer) makes it the bottom face, one row lower does not; its reach 3 x kBoxEdgePx above the band's "
          "top does, one row higher does not; half the narrower box's columns overlapping does, one fewer does "
          "not; the field face's apex row just inside 2 x kBoxEdgePx of the top is 'out of the frame'; the "
          "depth's 1-in floor decides a band some corner would otherwise call a bottom face") {
    // A band some corner explains as the bottom face, its field face's place out of the frame: alone it is
    // 'maybe the bottom face'; a Toggle-coloured box directly above makes it the bottom face (dropped, so with
    // nothing else: 'no band'). The box above is yellow and too thin or too square to be judged itself.
    const Detection b = camrig::colourBox(1, 0, 8, 319, 23);
    auto why = [&](const Detection& above) {
        ToggleTracker tr{cfg(), model8()};
        return feed(tr, {b, above}, 2).why;
    };
    {
        ToggleTracker tr{cfg(), model8()};
        REQUIRE(feed(tr, {b}, 2).why == ToggleWhy::MaybeBottomFace);
    }
    const int mid = b.top + b.height / 2;  // 19
    CHECK(why(camrig::colourBox(3, 100, mid - 15, 40, 15)) == ToggleWhy::NoBand);  // its bottom AT row 19
    CHECK(why(camrig::colourBox(3, 100, mid - 14, 40, 15)) == ToggleWhy::MaybeBottomFace);  // row 20
    const int reach = b.top - static_cast<int>(3.0 * vision::kBoxEdgePx);  // row 2
    CHECK(why(camrig::colourBox(3, 100, 0, 40, reach)) == ToggleWhy::NoBand);
    CHECK(why(camrig::colourBox(3, 100, 0, 40, reach - 1)) == ToggleWhy::MaybeBottomFace);
    // columns: a 40-wide box hanging off the right, overlapping 20 columns (half of the narrower) or 19
    CHECK(why(camrig::colourBox(3, 299, 0, 40, reach)) == ToggleWhy::NoBand);
    CHECK(why(camrig::colourBox(3, 300, 0, 40, reach)) == ToggleWhy::MaybeBottomFace);
    // the apex row: a band whose explaining corners put its field face's apex at row 0..4 (inside the frame,
    // but within 2 x kBoxEdgePx of its top) cannot tell
    const Detection nearTop = camrig::colourBox(1, 0, 24, 319, 13);
    const BottomOracle o1 = bottomOracle(model8(), nearTop);
    REQUIRE(o1.explained);
    REQUIRE(o1.minApexRow >= 0.0);
    REQUIRE(o1.minApexRow < 2.0 * vision::kBoxEdgePx);
    {
        ToggleTracker tr{cfg(), model8()};
        CHECK(feed(tr, {nearTop}, 2).why == ToggleWhy::MaybeBottomFace);
    }
    // the depth's floor: a camera pitched UP 10 degrees, a band no corner explains as the bottom face (its
    // depth on the bottom's plane under 1 in at every corner that could) — though one would without the floor,
    // with the field face's place out of the frame. It passes, and becomes sure.
    const vision::CameraModel up10 = camrig::model(8.0, 10.0);
    const Detection shallow = camrig::colourBox(1, 0, 2, 319, 7);
    const BottomOracle o2 = bottomOracle(up10, shallow);
    REQUIRE_FALSE(o2.explained);
    REQUIRE(o2.explainedNoFloor);
    REQUIRE(o2.minApexRowNoFloor < 2.0 * vision::kBoxEdgePx);
    {
        ToggleTracker tr{cfg(), up10};
        const ToggleView v = feed(tr, {shallow}, 6);
        CHECK(v.colour == ToggleColour::Red);
    }
}

TEST_CASE("toggle tracker: the POSITION gate at EACH of the four Toggle places names that place; its edges LANDED "
          "ON -- the pose bound exactly maxPoseToleranceIn checks, the next double skips; a tolerance of 0 is the "
          "gate OFF; a fix off the face's line by more than the tolerance but within it plus the fix's radius "
          "passes, and past the face's end by less than that slack too") {
    ToggleTrackerConfig c = cfg();
    c.positionToleranceIn = 3.0;
    c.maxPoseToleranceIn = 6.0;
    c.maxFixRadiusIn = 30.0;
    camrig::sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd;
    tc.height = 8.0;
    for (int i = 0; i < 4; ++i) {
        CAPTURE(i);
        const vision::override2026::TogglePlace& p = vision::override2026::kTogglePlaces[i];
        camrig::sim::CameraPlant plant{tc, 1};
        plant.addQuad(camrig::sim::CameraPlant::toggleFace(p.x, p.y, p.inX, p.inY, camrig::kFace.lengthIn,
                                                           camrig::kFace.bottomIn, camrig::kFace.topIn, 1.2, 1));
        const double rx = p.x + p.inX * (30.0 + camrig::kLensFwd);
        const double ry = p.y + p.inY * (30.0 + camrig::kLensFwd);
        const double heading = std::atan2(-p.inY, -p.inX);  // facing the wall
        ToggleTracker tr{c, model8()};
        double t = 1.0;
        for (int k = 0; k < 6; ++k, t += 0.0625) {
            tr.update(plant.read(rx, ry, heading, Time{t}), camrig::poseAt(rx, ry, heading, 1.0, 1.0));
        }
        const ToggleView v = tr.view(t - 0.0625);
        CHECK(v.colour == ToggleColour::Red);
        CHECK(v.positionChecked);
        CHECK(v.toggleIndex == i);
    }
    const Detection d = realFace();
    auto viewFrom = [&](const ToggleTrackerConfig& cc, double dx, double dy, double posTol) {
        ToggleTracker tr{cc, model8()};
        double t = 1.0;
        for (int k = 0; k < 6; ++k, t += 0.0625) {
            tr.update(frame({d}, t), camrig::poseAt(faceRobotX() + dx, dy, 0.0, posTol, 1.0));
        }
        return tr.view(t - 0.0625);
    };
    {  // the pose bound: exactly maxPoseToleranceIn is trusted, the next double is not
        const ToggleView at = viewFrom(c, 0.0, 0.0, 6.0);
        CHECK(at.positionChecked);
        CHECK(at.toggleIndex == 0);
        const ToggleView past = viewFrom(c, 0.0, 0.0, std::nextafter(6.0, 7.0));
        CHECK(past.colour == ToggleColour::Red);
        CHECK_FALSE(past.positionChecked);
        CHECK(past.toggleIndex == -1);
    }
    {  // a tolerance of 0 is the gate OFF, whatever its companions say
        ToggleTrackerConfig off = c;
        off.positionToleranceIn = 0.0;
        const ToggleView v = viewFrom(off, 0.0, 0.0, 1.0);
        CHECK(v.colour == ToggleColour::Red);
        CHECK_FALSE(v.positionChecked);
        CHECK(v.toggleIndex == -1);
    }
    // the slack: where this picture puts the face from the true pose, then the pose moved so the fix sits
    // between the tolerance and the tolerance plus the fix's radius off the line (across), or that far past
    // the face's end (along)
    const ToggleView ref = viewFrom(c, 0.0, 0.0, 1.0);
    REQUIRE(ref.toggleIndex == 0);
    const vision::FieldFix fix = model8().fieldFix(camrig::poseAt(faceRobotX(), 0.0, 0.0, 1.0, 1.0), ref.bearing,
                                                   ref.range);
    REQUIRE(fix.valid);
    REQUIRE(fix.radius > 1.0);
    {  // the fix's radius LANDED ON: exactly maxFixRadiusIn is placed; the next double down is too vague
        ToggleTrackerConfig atR = c;
        atR.maxFixRadiusIn = fix.radius;
        const ToggleView at = viewFrom(atR, 0.0, 0.0, 1.0);
        CHECK(at.colour == ToggleColour::Red);
        CHECK(at.positionChecked);
        CHECK(at.toggleIndex == 0);
        atR.maxFixRadiusIn = std::nextafter(fix.radius, 0.0);
        CHECK(viewFrom(atR, 0.0, 0.0, 1.0).colour == ToggleColour::Unknown);
    }
    const vision::override2026::TogglePlace& p0 = vision::override2026::kTogglePlaces[0];
    const double across0 = (fix.x - p0.x) * p0.inX + (fix.y - p0.y) * p0.inY;
    const double along0 = -(fix.x - p0.x) * p0.inY + (fix.y - p0.y) * p0.inX;
    const double between = c.positionToleranceIn + 0.5 * fix.radius;
    {  // across: the robot moved +x by dx moves the fix +x, across by -dx
        const double dx = across0 + between;
        const ToggleView v = viewFrom(c, dx, 0.0, 1.0);
        CHECK(v.colour == ToggleColour::Red);
        CHECK(v.toggleIndex == 0);
        const ToggleView beyond = viewFrom(c, across0 + c.positionToleranceIn + 1.5 * fix.radius, 0.0, 1.0);
        CHECK(beyond.colour == ToggleColour::Unknown);
    }
    {  // along: the robot moved +y by dy moves the fix +y, along by -dy (inX = -1)
        const double half = vision::override2026::kToggleFace.lengthIn / 2.0;
        const double slack = fix.radius + c.positionToleranceIn;
        const double dy = along0 + half + 0.5 * slack;
        const ToggleView v = viewFrom(c, 0.0, dy, 1.0);
        CHECK(v.colour == ToggleColour::Red);
        CHECK(v.toggleIndex == 0);
        const ToggleView beyond = viewFrom(c, 0.0, along0 + half + 1.5 * slack, 1.0);
        CHECK(beyond.colour == ToggleColour::Unknown);
    }
}

TEST_CASE("toggle tracker: a FROZEN view with the position gate ON says unchecked, no Toggle and no time held") {
    const Detection d = realFace();
    ToggleTrackerConfig c = cfg();
    c.positionToleranceIn = 3.0;
    c.maxPoseToleranceIn = 6.0;
    c.maxFixRadiusIn = 30.0;
    ToggleTracker tr{c, model8()};
    double t = 1.0;
    for (int k = 0; k < 5; ++k, t += 0.0625) {
        tr.update(frame({d}, t), atFace());
    }
    const ToggleView sure = tr.view(t - 0.0625);
    REQUIRE(sure.colour == ToggleColour::Red);
    REQUIRE(sure.positionChecked);
    REQUIRE(sure.toggleIndex == 0);
    REQUIRE(sure.heldSeconds == 0.25);
    tr.update(frame({d}, t), atFace(3.0));  // the lens moved 3 in, the box did not move by a pixel
    const ToggleView v = tr.view(t);
    CHECK(v.colour == ToggleColour::Unknown);
    CHECK(v.why == ToggleWhy::CameraFrozen);
    CHECK_FALSE(v.positionChecked);
    CHECK(v.toggleIndex == -1);
    CHECK(v.heldSeconds == 0.0);
}

TEST_CASE("toggle tracker: a frame whose count says MORE than the array holds reads only the array -- a passing "
          "band of another colour just past it is never seen") {
    struct Padded {
        VisionFrame f{};
        Detection past[2]{};
    };
    Padded p{};
    // the detection just past the array must sit right after it, or this test shows nothing
    REQUIRE(reinterpret_cast<const char*>(&p.past[0])
            == reinterpret_cast<const char*>(p.f.detections + hal::kMaxVisionDetections));
    p.f.status = hal::FrameStatus::Ok;
    p.f.model = hal::ModelCheck::Matches;
    p.f.detections[0] = realFace();
    p.past[0] = farFace();
    p.past[0].id = 2;  // a passing BLUE band: seen, it would make the read 'two colours'
    p.f.count = static_cast<std::uint8_t>(hal::kMaxVisionDetections + 1);
    p.f.reportedCount = p.f.count;
    ToggleTracker tr{cfg(), model8()};
    double t = 1.0;
    for (int k = 0; k < 5; ++k, t += 0.0625) {
        p.f.readAt = Time{t};
        tr.update(p.f, atFace());
    }
    CHECK(tr.view(t - 0.0625).colour == ToggleColour::Red);
}

namespace {

// The edge agreement per corner, written again from the header (an oracle): for each corner whose four rays
// meet their planes, the lower edge's ranges [lo, hi] that put the apex 0.9..1.2 in behind it — before the
// length — with that corner's fx.
struct CornerEdge {
    double lo = 0.0;
    double hi = 0.0;
    double fx = 0.0;
};

int cornerEdges(const vision::CameraModel& m, const Detection& d, CornerEdge (&out)[64]) {
    const vision::override2026::ToggleFace& face = vision::override2026::kToggleFace;
    const double u = d.left + d.width / 2.0;
    const double e = vision::kBoxEdgePx;
    int n = 0;
    m.eachCorner(true, [&](const vision::CameraCorner& c) {
        for (const double plane : {face.bottomIn, face.bottomHiIn}) {
            double b1 = 0.0;
            double b2 = 0.0;
            double t1 = 0.0;
            double t2 = 0.0;
            if (!vision::CameraModel::rangeToPlane(c, u, d.top + d.height - e, plane, 0.5, b1)
                || !vision::CameraModel::rangeToPlane(c, u, d.top + d.height + e, plane, 0.5, b2)
                || !vision::CameraModel::rangeToPlane(c, u, d.top - e, face.topIn, 0.5, t1)
                || !vision::CameraModel::rangeToPlane(c, u, d.top + e, face.topIn, 0.5, t2)) {
                continue;
            }
            const double lo = std::fmax(std::fmin(b1, b2), std::fmin(t1, t2) - face.setBackHiIn - 0.25);
            const double hi = std::fmin(std::fmax(b1, b2), std::fmax(t1, t2) - face.setBackLoIn + 0.25);
            if (lo <= hi && n < 64) {
                out[n++] = CornerEdge{lo, hi, c.fx};
            }
        }
    });
    return n;
}

}  // namespace

TEST_CASE("toggle tracker: the LENGTH gate's edges LANDED ON at the jittered widths -- the largest minimum "
          "fraction and the smallest maximum one that still pass, from the edge agreement and the box's width "
          "+-2 x kBoxEdgePx; one double past each refuses; the implied length is the range times the jittered "
          "widths over the focal lengths") {
    const Detection d = realFace();
    CornerEdge ce[64];
    const int n = cornerEdges(model8(), d, ce);
    REQUIRE(n > 0);
    const double length = vision::override2026::kToggleFace.lengthIn;
    const double w = d.width;
    const double wLo = std::fmax(1.0, w - 2.0 * vision::kBoxEdgePx);
    const double wHi = w + 2.0 * vision::kBoxEdgePx;
    // the MAXIMUM fraction: a corner explains it when lo <= f L fx / wLo (the tracker's own expression, in its
    // order); the smallest f any corner accepts
    auto maxOk = [&](double f) {
        for (int i = 0; i < n; ++i) {
            if (ce[i].lo <= std::fmin(ce[i].hi, f * length * ce[i].fx / wLo)) {
                return true;
            }
        }
        return false;
    };
    double fMax = 4.0;
    for (double step = 1.0; step > 1e-15; step /= 2.0) {
        while (fMax - step > 0.02 && maxOk(fMax - step)) {
            fMax -= step;
        }
    }
    while (maxOk(std::nextafter(fMax, 0.0))) {
        fMax = std::nextafter(fMax, 0.0);
    }
    REQUIRE(maxOk(fMax));
    auto colourWith = [&](double minF, double maxF) {
        ToggleTrackerConfig c = cfg();
        c.minLengthFraction = minF;
        c.maxLengthFraction = maxF;
        ToggleTracker tr{c, model8()};
        return feed(tr, {d}, 6).colour;
    };
    CHECK(colourWith(0.01, fMax) == ToggleColour::Red);
    CHECK(colourWith(0.01, std::nextafter(fMax, 0.0)) == ToggleColour::Unknown);
    // the MINIMUM fraction: a corner explains it when f L fx / wHi <= hi; the largest f any corner accepts
    auto minOk = [&](double f) {
        for (int i = 0; i < n; ++i) {
            if (std::fmax(ce[i].lo, f * length * ce[i].fx / wHi) <= ce[i].hi) {
                return true;
            }
        }
        return false;
    };
    double fMin = 0.02;
    for (double step = 1.0; step > 1e-15; step /= 2.0) {
        while (fMin + step < 3.9 && minOk(fMin + step)) {
            fMin += step;
        }
    }
    while (minOk(std::nextafter(fMin, 9.0))) {
        fMin = std::nextafter(fMin, 9.0);
    }
    REQUIRE(minOk(fMin));
    CHECK(colourWith(fMin, 4.0) == ToggleColour::Red);
    CHECK(colourWith(std::nextafter(fMin, 9.0), 4.0) == ToggleColour::Unknown);
    // the implied length (no length judged): the reported range times the jittered widths over the focal
    // lengths' extremes
    const vision::ToggleFaceMeasure mm = vision::explainAsToggleFace(d, model8());
    REQUIRE(mm.gate == ToggleFaceGate::Explained);
    double fxLo = 1e9;
    double fxHi = 0.0;
    model8().eachCorner(true, [&](const vision::CameraCorner& c) {
        fxLo = std::fmin(fxLo, c.fx);
        fxHi = std::fmax(fxHi, c.fx);
    });
    CHECK(mm.impliedLength.lo == wLo * mm.range.lo / fxHi);
    CHECK(mm.impliedLength.hi == wHi * mm.range.hi / fxLo);
}

TEST_CASE("toggle tracker: the hold is the BAND's -- a nearer band of the same colour, in another direction, "
          "taking over mid-hold restarts it ('not held yet'), and holds again from there") {
    const Detection near = realFace();
    Detection far = farFace();  // 48 in out, moved 60 px right: about 16 degrees off, its range overlapping
    far.left = static_cast<std::int16_t>(far.left + 60);
    ToggleTracker tr{cfg(), model8()};
    double t = 1.0;
    for (int k = 0; k < 5; ++k, t += 0.0625) {
        tr.update(frame({far}, t), atFace());
    }
    REQUIRE(tr.view(t - 0.0625).colour == ToggleColour::Red);
    const double farMid = tr.view(t - 0.0625).range.mid();
    tr.update(frame({far, near}, t), atFace());  // the nearer band (30 in, not 48) is now the front
    const ToggleView jumped = tr.view(t);
    CHECK(jumped.colour == ToggleColour::Unknown);
    CHECK(jumped.why == ToggleWhy::NotHeldYet);
    CHECK(jumped.range.mid() < farMid);
    for (int k = 0; k < 4; ++k) {
        t += 0.0625;
        tr.update(frame({far, near}, t), atFace());
    }
    const ToggleView again = tr.view(t);
    CHECK(again.colour == ToggleColour::Red);
    CHECK(again.heldSeconds == 0.25);
}

TEST_CASE("toggle tracker: the same band, decided by RANGE -- through a camera measured to 0.01 in / 0.01 degree, "
          "a nearer band straight ahead (the same bearing, a range that does not overlap) taking over restarts the "
          "hold") {
    camrig::config::CameraPlacement p = camrig::believed(8.0);
    p.positionTolerance = camrig::Length{0.01};
    p.heightTolerance = camrig::Length{0.01};
    p.yawToleranceDeg = 0.01;
    p.pitchToleranceDeg = 0.01;
    vision::CameraOptics o = vision::kAiVisionOptics;
    o.fovTolerance = 0.001;
    const vision::CameraModel tight{o, p};
    const Detection near = realFace();
    const Detection far = farFace();
    ToggleTracker tr{cfg(), tight};
    double t = 1.0;
    for (int k = 0; k < 5; ++k, t += 0.0625) {
        tr.update(frame({far}, t), atFace());
    }
    const ToggleView held = tr.view(t - 0.0625);
    REQUIRE(held.colour == ToggleColour::Red);
    tr.update(frame({far, near}, t), atFace());
    const ToggleView jumped = tr.view(t);
    REQUIRE(jumped.range.hi < held.range.lo);  // the ranges apart
    REQUIRE_FALSE(jumped.bearing.hi < held.bearing.lo);  // the bearings overlapping: the range decides
    REQUIRE_FALSE(held.bearing.hi < jumped.bearing.lo);
    CHECK(jumped.why == ToggleWhy::NotHeldYet);
}

TEST_CASE("toggle tracker: the same band, decided by PLACE -- with the position gate on, a read taken after the lens "
          "moved far (its travel widens the direction and the range to anything) whose band names ANOTHER Toggle "
          "restarts the hold") {
    ToggleTrackerConfig c = cfg();
    c.positionToleranceIn = 3.0;
    c.maxPoseToleranceIn = 6.0;
    c.maxFixRadiusIn = 30.0;
    camrig::sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd;
    tc.height = 8.0;
    camrig::sim::CameraPlant plant{tc, 1};
    for (const int i : {0, 1}) {
        const vision::override2026::TogglePlace& p = vision::override2026::kTogglePlaces[i];
        plant.addQuad(camrig::sim::CameraPlant::toggleFace(p.x, p.y, p.inX, p.inY, camrig::kFace.lengthIn,
                                                           camrig::kFace.bottomIn, camrig::kFace.topIn, 1.2, 1));
    }
    auto readFacing = [&](ToggleTracker& tr, int i, double lensToFace, double t) {
        const vision::override2026::TogglePlace& p = vision::override2026::kTogglePlaces[i];
        const double rx = p.x + p.inX * (lensToFace + camrig::kLensFwd);
        const double ry = p.y + p.inY * (lensToFace + camrig::kLensFwd);
        const double heading = std::atan2(-p.inY, -p.inX);
        tr.update(plant.read(rx, ry, heading, Time{t}), camrig::poseAt(rx, ry, heading, 1.0, 1.0));
    };
    ToggleTracker tr{c, model8()};
    double t = 1.0;
    for (int k = 0; k < 5; ++k, t += 0.0625) {
        readFacing(tr, 0, 30.0, t);
    }
    const ToggleView held = tr.view(t - 0.0625);
    REQUIRE(held.colour == ToggleColour::Red);
    REQUIRE(held.toggleIndex == 0);
    readFacing(tr, 1, 36.0, t);  // the other wall's Toggle, a different distance (its box is not the held one's)
    const ToggleView moved = tr.view(t);
    CHECK(moved.colour == ToggleColour::Unknown);
    CHECK(moved.why == ToggleWhy::NotHeldYet);
    CHECK(moved.toggleIndex == 1);
}

TEST_CASE("toggle tracker: the same band WHILE MOVING -- a camera measured to 0.01 in / 0.01 degree strafing past a "
          "Toggle 24 in away, 2 in a read: the band's direction from the lens moves more than its bearing interval "
          "is wide from read to read, but the lens's travel explains it, so the hold continues and the colour "
          "becomes sure") {
    camrig::config::CameraPlacement p = camrig::believed(8.0);
    p.positionTolerance = camrig::Length{0.01};
    p.heightTolerance = camrig::Length{0.01};
    p.yawToleranceDeg = 0.01;
    p.pitchToleranceDeg = 0.01;
    vision::CameraOptics o = vision::kAiVisionOptics;
    o.fovTolerance = 0.001;
    const vision::CameraModel tight{o, p};
    camrig::sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd;
    tc.height = 8.0;
    camrig::sim::CameraPlant plant{tc, 1};
    camrig::addToggle(plant, 1, 3);
    ToggleTracker tr{cfg(), tight};
    const double rx = camrig::robotXFor(24.0);
    double t = 1.0;
    vision::Interval prev{};
    bool apart = false;
    for (int k = 0; k < 6; ++k, t += 0.0625) {
        const double ry = -5.0 + 2.0 * k;
        tr.update(plant.read(rx, ry, 0.0, Time{t}), camrig::poseAt(rx, ry, 0.0, 0.01, 0.01));
        const ToggleView v = tr.view(t);
        REQUIRE(v.bearing.valid);
        if (prev.valid && (v.bearing.hi < prev.lo || v.bearing.lo > prev.hi)) {
            apart = true;  // the heading is 0 throughout: the direction is the bearing
        }
        prev = v.bearing;
    }
    REQUIRE(apart);  // without the lens's travel the two reads could not be the same band's
    const ToggleView v = tr.view(t - 0.0625);
    CHECK(v.colour == ToggleColour::Red);
    CHECK(v.heldSeconds >= 0.25);
}

TEST_CASE("toggle tracker: the frozen flag -- a NEW colour after a frozen view starts a fresh hold (not 'frozen'); "
          "a box that keeps its corner but changes SIZE has moved (not frozen)") {
    const Detection d = realFace();
    ToggleTrackerConfig c = cfg();
    auto centredModel = [] {
        camrig::config::CameraPlacement p = camrig::believed(8.0);
        p.forward = camrig::Length{0.0};
        return vision::CameraModel{vision::kAiVisionOptics, p};
    };
    {
        ToggleTracker tr{c, centredModel()};
        tr.update(frame({d}, 1.0), camrig::poseAt(10.0, 0.0, 0.0));
        tr.update(frame({d}, 1.0625), camrig::poseAt(13.0, 0.0, 0.0));
        REQUIRE(tr.view(1.0625).why == ToggleWhy::CameraFrozen);
        Detection blue = d;
        blue.id = 2;
        blue.left = static_cast<std::int16_t>(blue.left + 2);
        tr.update(frame({blue}, 1.125), camrig::poseAt(14.0, 0.0, 0.0));
        CHECK(tr.view(1.125).why == ToggleWhy::NotHeldYet);
    }
    {
        ToggleTracker tr{c, centredModel()};
        tr.update(frame({d}, 1.0), camrig::poseAt(10.0, 0.0, 0.0));
        Detection taller = d;
        taller.height = static_cast<std::int16_t>(taller.height + 1);
        tr.update(frame({taller}, 1.0625), camrig::poseAt(13.0, 0.0, 0.0));
        CHECK(tr.view(1.0625).why != ToggleWhy::CameraFrozen);
        Detection wider = taller;
        wider.width = static_cast<std::int16_t>(wider.width + 1);
        tr.update(frame({wider}, 1.125), camrig::poseAt(16.0, 0.0, 0.0));
        CHECK(tr.view(1.125).why != ToggleWhy::CameraFrozen);
    }
}
