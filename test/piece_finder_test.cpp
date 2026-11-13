// vision::PieceFinder and sightPiece() on HAND-MADE frames — every rule's edge landed on exactly (chunk VIS1-A2;
// the VIS1 brief's §1.13 and §2.2 "the finder ... every target and config refusal by name"). The scenes
// against the host camera model are piece_finder_scenes_test.cpp.
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * EVERY config and target refusal by its own words (the lowest raise among them — VIS1-A2's addition),
//     and the first accepted value beside each refused one (a class past the model's five; the posture
//     UNSURE asked for).
//   * The words, ASCII and distinct; the top face's allowance at its edges.
//   * sightPiece(): only an Object of a class the season names is interpreted; a clipped box is UNSURE (a
//     side clip carries no bound); the aspect thresholds LANDED ON (exactly lyingAspect is LYING, exactly
//     standingAspect stands; the next double the other way does not).
//   * The finder: positive evidence only — EVERY non-Ok status is UNKNOWN "read not usable", never NONE (a
//     defect the stopped run's copy carried: the fault emptied the sightings and answer() said NONE); a model
//     check other than Matches; a POSSIBLY FULL read; the alive time and the hold LANDED ON; the score
//     floor landed on; INSIDE / ON THE BORDER / OUTSIDE each landed on from the sighting's own fix (equal is
//     inside; the next double below is the border; outside is strict); the error larger than the tolerance;
//     two inside; an unmeasurable box whose bearing could point inside (AMBIGUOUS) and one that cannot
//     (NONE); the wrong posture; the class mask; the hold restarted by a different candidate and by a new
//     target.
//   * NOT IN VIEW from the WHOLE disc: a place whose centre and four axis points are certainly in view but
//     whose disc pokes out of the picture diagonally is NOT IN VIEW (the five-point test said otherwise).
//   * A TIGHT camera (mount to 0.01 in / 0.01 degree, optics to 0.1 %): its intervals little more than the box
//     edge's jitter, so each far-end term of a fix is visible — the floor fix's radius past the ground range
//     (MA12), the RAISED fix's top-face allowance past the size range (MA14), the LYING fix's half length past
//     its nearest point for a Pin lying DIAGONALLY (MA15). Each mutant survived every looser scene until its
//     case; each case pins that its scene IS on the edge (the true centre past the mutant's far end).

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "camera_scene_rig.hpp"

namespace {

namespace hal = camrig::hal;
namespace sim = camrig::sim;
namespace vision = camrig::vision;
using camrig::Time;
using hal::Detection;
using hal::VisionFrame;
using vision::PieceFinder;
using vision::PieceFinderConfig;
using vision::PieceTarget;
using vision::PieceVerdict;
using vision::PieceWhy;
using vision::Posture;

constexpr double kPi = camrig::kPi;

vision::CameraModel fm() { return camrig::model(10.0, -10.0); }

// The TRUE camera EQUAL to the believed one (10 in up, pitched 10 down, 74 x 63), no jitter: a hand edit is
// then the only difference between the frames below.
sim::TrueCamera exactTruth() {
    sim::TrueCamera tc{};
    tc.forward = camrig::kLensFwd;
    tc.height = 10.0;
    tc.pitchDeg = -10.0;
    return tc;
}

// The box of a yellow Pin standing at (x, y) — or lying ACROSS the view, from (x, y) to (x, y + 6.5) — seen
// from the robot at the origin facing +x.
Detection pinBox(double x, double y, bool lying = false, double baseZ = 0.0) {
    sim::CameraPlant plant{exactTruth(), 1};
    sim::Element e = camrig::pin(x, y, baseZ, lying);
    e.lyingHeadingRad = kPi / 2.0;
    plant.addElement(e);
    const VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    return f.detections[0];
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

PieceFinderConfig cfg() {
    PieceFinderConfig c = camrig::fcfg();
    c.holdSeconds = 0.25;  // binary-exact edges
    c.aliveSeconds = 0.125;
    return c;
}

vision::PoseBound origin() { return camrig::poseAt(0.0, 0.0, 0.0, 0.5, 0.5); }

// The Pin the finder's cases name: 30 in ahead (the lens 23 in from it — near enough to be FOUND within 6 in).
constexpr double kPinX = 30.0;
PieceTarget named() { return camrig::yellowPinAt(kPinX, 0.0, 6.0); }

// Feed `n` copies of a frame from the origin, 1/16 s apart from t0; the answer at the last read.
PieceFinder::Result feed(PieceFinder& f, std::initializer_list<Detection> ds, int n, double t0 = 1.0) {
    double t = t0;
    for (int i = 0; i < n; ++i) {
        f.update(frame(ds, t), origin());
        t += 0.0625;
    }
    return f.answer(t - 0.0625);
}

// A camera measured on the bench: its mount to 0.01 in and 0.01 degree, its optics to 0.1 % (HA-194) — the
// intervals then little more than the box edge's jitter, so a bound's every term is visible.
vision::CameraModel tightModel(double h, double pitch) {
    camrig::config::CameraPlacement p = camrig::believed(h, pitch);
    p.positionTolerance = camrig::Length{0.01};
    p.heightTolerance = camrig::Length{0.01};
    p.yawToleranceDeg = 0.01;
    p.pitchToleranceDeg = 0.01;
    vision::CameraOptics o = vision::kAiVisionOptics;
    o.fovTolerance = 0.001;
    return vision::CameraModel{o, p};
}

// The true camera EQUAL to tightModel(h, pitch)'s.
sim::TrueCamera tightTruth(double h, double pitch) {
    sim::TrueCamera tc = exactTruth();
    tc.height = h;
    tc.pitchDeg = pitch;
    return tc;
}

// One Element through the tight camera from the robot at the origin facing +x: its one detection.
Detection tightBox(const sim::Element& e, double h, double pitch) {
    sim::CameraPlant plant{tightTruth(h, pitch), 1};
    plant.addElement(e);
    const VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    REQUIRE_FALSE(f.detections[0].clipped());
    return f.detections[0];
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

TEST_CASE("piece finder: every config refusal by its words; the first accepted value beside each refused one") {
    const PieceFinderConfig good = camrig::fcfg();
    REQUIRE(vision::pieceFinderConfigProblem(good) == nullptr);
    CHECK(vision::pieceFinderConfigProblem(PieceFinderConfig{}) != nullptr);
    double PieceFinderConfig::*const numbers[] = {&PieceFinderConfig::holdSeconds,  &PieceFinderConfig::aliveSeconds,
                                                  &PieceFinderConfig::minScore,     &PieceFinderConfig::lyingAspect,
                                                  &PieceFinderConfig::standingAspect, &PieceFinderConfig::viewMarginPx,
                                                  &PieceFinderConfig::raisedMinIn};
    for (double PieceFinderConfig::*const m : numbers) {
        for (const double bad : {std::numeric_limits<double>::quiet_NaN(), -std::numeric_limits<double>::infinity()}) {
            PieceFinderConfig c = good;
            c.*m = bad;
            const char* p = vision::pieceFinderConfigProblem(c);
            REQUIRE(p != nullptr);
            CHECK(std::string{p} == "a finder number is not a finite number");
        }
    }
    auto words = [&](void (*edit)(PieceFinderConfig&)) {
        PieceFinderConfig c = good;
        edit(c);
        const char* p = vision::pieceFinderConfigProblem(c);
        return p == nullptr ? std::string{"ACCEPTED"} : std::string{p};
    };
    CHECK(words([](PieceFinderConfig& c) { c.holdSeconds = 0.0; }) == "holdSeconds UNSET -- how long FOUND must hold");
    CHECK(words([](PieceFinderConfig& c) { c.aliveSeconds = 0.0; }) ==
          "aliveSeconds UNSET -- how old the newest usable read may be");
    CHECK(words([](PieceFinderConfig& c) { c.standingAspect = 0.0; }) == "standing / lying aspect UNSET or reversed");
    CHECK(words([](PieceFinderConfig& c) { c.lyingAspect = c.standingAspect; }) ==
          "standing / lying aspect UNSET or reversed");
    CHECK(words([](PieceFinderConfig& c) { c.lyingAspect = std::nextafter(c.standingAspect, 9.0); }) == "ACCEPTED");
    // the two numbers whose 0 means something ("every score", "no margin"): UNSET is -1, the default (A2's review)
    CHECK(PieceFinderConfig{}.minScore == -1.0);
    CHECK(PieceFinderConfig{}.viewMarginPx == -1.0);
    CHECK(words([](PieceFinderConfig& c) { c.minScore = PieceFinderConfig{}.minScore; }) ==
          "minScore UNSET (negative) -- the lowest model score believed, 0 for every score");
    CHECK(words([](PieceFinderConfig& c) { c.minScore = -std::numeric_limits<double>::denorm_min(); }) ==
          "minScore UNSET (negative) -- the lowest model score believed, 0 for every score");
    CHECK(words([](PieceFinderConfig& c) { c.minScore = 0.0; }) == "ACCEPTED");
    CHECK(words([](PieceFinderConfig& c) { c.viewMarginPx = PieceFinderConfig{}.viewMarginPx; }) ==
          "viewMarginPx UNSET (negative) -- how far inside the picture the place must be, 0 for none");
    CHECK(words([](PieceFinderConfig& c) { c.viewMarginPx = -std::numeric_limits<double>::denorm_min(); }) ==
          "viewMarginPx UNSET (negative) -- how far inside the picture the place must be, 0 for none");
    CHECK(words([](PieceFinderConfig& c) { c.viewMarginPx = 0.0; }) == "ACCEPTED");
    CHECK(words([](PieceFinderConfig& c) { c.raisedMinIn = 0.0; }) ==
          "raisedMinIn UNSET -- the lowest a raised piece sits on this field (a Goal's top), inches");
    CHECK(words([](PieceFinderConfig& c) { c.raisedMinIn = -5.0; }) ==
          "raisedMinIn UNSET -- the lowest a raised piece sits on this field (a Goal's top), inches");
}

TEST_CASE("piece finder: every target refusal by its words -- no class, a class past the model's five, a position "
          "or tolerance not finite, the tolerance UNSET, an UNSURE posture") {
    const PieceTarget good = named();
    REQUIRE(vision::pieceTargetProblem(good) == nullptr);
    CHECK(vision::pieceTargetProblem(PieceTarget{}) != nullptr);
    auto words = [&](void (*edit)(PieceTarget&)) {
        PieceTarget t = good;
        edit(t);
        const char* p = vision::pieceTargetProblem(t);
        return p == nullptr ? std::string{"ACCEPTED"} : std::string{p};
    };
    CHECK(words([](PieceTarget& t) { t.classMask = 0U; }) == "no class named");
    CHECK(words([](PieceTarget& t) { t.classMask = 1U << 5; }) == "a class the season's AI model does not name");
    CHECK(words([](PieceTarget& t) { t.classMask = 1U << 31; }) == "a class the season's AI model does not name");
    CHECK(words([](PieceTarget& t) { t.classMask = 0x1FU; }) == "ACCEPTED");  // all five classes
    CHECK(words([](PieceTarget& t) { t.x = std::numeric_limits<double>::quiet_NaN(); }) ==
          "the expected position or the tolerance is not a finite number");
    CHECK(words([](PieceTarget& t) { t.y = std::numeric_limits<double>::infinity(); }) ==
          "the expected position or the tolerance is not a finite number");
    CHECK(words([](PieceTarget& t) { t.toleranceIn = std::numeric_limits<double>::infinity(); }) ==
          "the expected position or the tolerance is not a finite number");
    CHECK(words([](PieceTarget& t) { t.toleranceIn = 0.0; }) == "tolerance UNSET");
    CHECK(words([](PieceTarget& t) { t.want = Posture::Unsure; }) == "a route cannot ask for an UNSURE piece");
    for (const Posture p : {Posture::OnFloor, Posture::Raised, Posture::Lying}) {
        PieceTarget t = good;
        t.want = p;
        CHECK(vision::pieceTargetProblem(t) == nullptr);
    }
}

TEST_CASE("piece finder: the words ASCII and distinct; a byte outside each enum never a verdict or a posture; the "
          "top face's allowance at its edges") {
    for (int a = 0; a <= static_cast<int>(PieceWhy::ReadNotUsable); ++a) {
        const char* wa = vision::pieceWhyWords(static_cast<PieceWhy>(a));
        CHECK(ascii(wa));
        CHECK(std::string{wa} != "?");
        for (int b = a + 1; b <= static_cast<int>(PieceWhy::ReadNotUsable); ++b) {
            CHECK(std::string{wa} != vision::pieceWhyWords(static_cast<PieceWhy>(b)));
        }
    }
    CHECK(std::string{vision::pieceWhyWords(static_cast<PieceWhy>(99))} == "?");
    const char* verdicts[] = {"UNKNOWN", "FOUND", "NONE", "AMBIGUOUS", "NOT IN VIEW"};
    for (int v = 0; v <= static_cast<int>(PieceVerdict::NotInView); ++v) {
        CHECK(std::string{vision::pieceVerdictWords(static_cast<PieceVerdict>(v))} == verdicts[v]);
    }
    CHECK(std::string{vision::pieceVerdictWords(static_cast<PieceVerdict>(9))} == "UNKNOWN");
    const char* postures[] = {"UNSURE", "ON-FLOOR", "RAISED", "LYING"};
    for (int p = 0; p <= static_cast<int>(Posture::Lying); ++p) {
        CHECK(std::string{vision::postureWord(static_cast<Posture>(p))} == postures[p]);
    }
    CHECK(std::string{vision::postureWord(static_cast<Posture>(9))} == "UNSURE");
    // 2r(a - H)/H: a 1.6-in Pin 6.5 tall under a 10-in lens
    CHECK(vision::topFaceAllowanceIn(6.5, 1.6, 10.0) == doctest::Approx(1.6 * 3.5 / 6.5));
    CHECK(vision::topFaceAllowanceIn(6.5, 1.6, 6.5) == 0.0);  // the lens AT the top: its far rim is not above
    CHECK(vision::topFaceAllowanceIn(6.5, 1.6, std::nextafter(6.5, 9.0)) > 0.0);
    CHECK(vision::topFaceAllowanceIn(6.5, 1.6, 4.0) == 0.0);
    CHECK(vision::topFaceAllowanceIn(0.0, 1.6, 10.0) == 0.0);
}

TEST_CASE("piece finder: sightPiece interprets only an Object of a class the season names; a clipped box is UNSURE; "
          "the aspect thresholds LANDED ON") {
    const vision::CameraModel m = fm();
    const Detection d = pinBox(kPinX, 0.0);
    const vision::PieceSighting on = vision::sightPiece(d, m, origin(), cfg());
    REQUIRE(on.posture == Posture::OnFloor);
    CHECK(std::hypot(on.fix.x - kPinX, on.fix.y) <= on.fix.radius);
    Detection colour = d;
    colour.kind = hal::DetectionKind::Colour;
    Detection unknownClass = d;
    unknownClass.id = 5;
    Detection empty = d;
    empty.height = 0;
    for (const Detection& x : {colour, unknownClass, empty}) {
        const vision::PieceSighting s = vision::sightPiece(x, m, origin(), cfg());
        CHECK(s.posture == Posture::Unsure);
        CHECK_FALSE(s.fix.valid);
        CHECK_FALSE(s.bearing.valid);
    }
    Detection side = d;
    side.left = 0;  // cut by the picture's left edge: UNSURE, with no bound
    const vision::PieceSighting ss = vision::sightPiece(side, m, origin(), cfg());
    CHECK(ss.posture == Posture::Unsure);
    CHECK_FALSE(ss.fix.valid);
    CHECK(ss.nearerThanIn == 0.0);
    // the aspect: exactly lyingAspect is LYING; the next double above it is not
    const double aspect = static_cast<double>(d.width) / d.height;
    PieceFinderConfig lie = cfg();
    lie.standingAspect = aspect / 2.0;
    lie.lyingAspect = aspect;
    CHECK(vision::sightPiece(d, m, origin(), lie).posture == Posture::Lying);
    lie.lyingAspect = std::nextafter(aspect, 9.0);
    CHECK(vision::sightPiece(d, m, origin(), lie).posture == Posture::Unsure);  // between the two
    // exactly standingAspect stands; the next double below it does not
    PieceFinderConfig stand = cfg();
    stand.standingAspect = aspect;
    CHECK(vision::sightPiece(d, m, origin(), stand).posture == Posture::OnFloor);
    stand.standingAspect = std::nextafter(aspect, 0.0);
    CHECK(vision::sightPiece(d, m, origin(), stand).posture == Posture::Unsure);
}

TEST_CASE("piece finder: positive evidence only -- EVERY non-Ok status answers UNKNOWN 'read not usable', never NONE; "
          "a model check other than Matches, 'model not confirmed'; a POSSIBLY FULL read, 'camera full'") {
    const Detection d = pinBox(kPinX, 0.0);
    {
        PieceFinder f{cfg(), fm()};
        f.setTarget(named());
        CHECK(f.answer(0.0).why == PieceWhy::NoRead);
    }
    for (const hal::FrameStatus s : {hal::FrameStatus::ReadFault, hal::FrameStatus::PartialFault,
                                     hal::FrameStatus::NotConfigured, hal::FrameStatus::NotRead}) {
        CAPTURE(static_cast<int>(s));
        PieceFinder f{cfg(), fm()};
        f.setTarget(named());
        REQUIRE(feed(f, {d}, 5).verdict == PieceVerdict::Found);
        VisionFrame bad = frame({d}, 1.3125);
        bad.status = s;
        f.update(bad, origin());
        const PieceFinder::Result r = f.answer(1.3125);
        CHECK(r.verdict == PieceVerdict::Unknown);
        CHECK(r.why == PieceWhy::ReadNotUsable);
        // an EMPTY scene after the fault is NONE again — only a usable read is evidence either way
        f.update(frame({}, 1.375), origin());
        CHECK(f.answer(1.375).verdict == PieceVerdict::None);
    }
    for (const hal::ModelCheck mc : {hal::ModelCheck::NotAsked, hal::ModelCheck::Different,
                                     hal::ModelCheck::Unreadable}) {
        PieceFinder f{cfg(), fm()};
        f.setTarget(named());
        VisionFrame w = frame({}, 1.0);  // the adapter withheld the Pin: no detection, and the check says why
        w.model = mc;
        w.objectsWithheld = 1;
        f.update(w, origin());
        CHECK(f.answer(1.0).verdict == PieceVerdict::Unknown);
        CHECK(f.answer(1.0).why == PieceWhy::ModelNotConfirmed);
    }
    PieceFinder f{cfg(), fm()};
    f.setTarget(named());
    VisionFrame full = frame({d}, 1.0);
    full.truncated = true;
    f.update(full, origin());
    CHECK(f.answer(1.0).verdict == PieceVerdict::Unknown);
    CHECK(f.answer(1.0).why == PieceWhy::CameraFull);
}

TEST_CASE("piece finder: the alive time and the hold LANDED ON -- FOUND held exactly holdSeconds, a read exactly "
          "aliveSeconds old; the next double past either is not") {
    const Detection d = pinBox(kPinX, 0.0);
    PieceFinder f{cfg(), fm()};
    f.setTarget(named());
    f.update(frame({d}, 1.0), origin());
    f.update(frame({d}, std::nextafter(1.25, 0.0)), origin());
    CHECK(f.answer(1.25).why == PieceWhy::NotHeldYet);
    f.update(frame({d}, 1.25), origin());
    CHECK(f.answer(1.25).verdict == PieceVerdict::Found);
    CHECK(f.answer(1.375).verdict == PieceVerdict::Found);
    const PieceFinder::Result silent = f.answer(std::nextafter(1.375, 2.0));
    CHECK(silent.verdict == PieceVerdict::Unknown);
    CHECK(silent.why == PieceWhy::CameraSilent);
    CHECK(f.answer(std::numeric_limits<double>::quiet_NaN()).why == PieceWhy::CameraSilent);
    // a NEW target restarts the hold
    f.setTarget(camrig::yellowPinAt(kPinX, 0.0, 5.5));
    f.update(frame({d}, 1.3125), origin());
    CHECK(f.answer(1.3125).why == PieceWhy::NotHeldYet);
}

TEST_CASE("piece finder: the score floor LANDED ON -- a score equal to minScore is kept, one below is ignored") {
    Detection d = pinBox(kPinX, 0.0);
    PieceFinderConfig c = cfg();
    c.minScore = 50.0;
    d.score = 50;
    PieceFinder keep{c, fm()};
    keep.setTarget(named());
    CHECK(feed(keep, {d}, 6).verdict == PieceVerdict::Found);
    d.score = 49;
    PieceFinder drop{c, fm()};
    drop.setTarget(named());
    const PieceFinder::Result r = feed(drop, {d}, 6);
    CHECK(r.verdict == PieceVerdict::None);
    CHECK(r.why == PieceWhy::NothingThere);
}

TEST_CASE("piece finder: INSIDE, ON THE BORDER and OUTSIDE each LANDED ON from the sighting's own fix -- its whole "
          "error disc inside the tolerance is inside (equal included); the next double smaller is the border; "
          "certainly outside is strict") {
    const Detection d = pinBox(kPinX, 0.0);
    const vision::PieceSighting s = vision::sightPiece(d, fm(), origin(), cfg());
    REQUIRE(s.fix.valid);
    REQUIRE(s.fix.radius > 1.0);
    // the target 3 in beyond the fix's centre (along x): dist as the finder computes it
    PieceTarget t = camrig::yellowPinAt(s.fix.x + 3.0, s.fix.y);
    const double dist = std::hypot(s.fix.x - t.x, s.fix.y - t.y);
    t.toleranceIn = dist + s.fix.radius;
    {
        PieceFinder f{cfg(), fm()};
        f.setTarget(t);
        const PieceFinder::Result r = feed(f, {d}, 6);
        CHECK(r.verdict == PieceVerdict::Found);
        CHECK(r.inside == 1);
        CHECK(r.border == 0);
    }
    {
        PieceTarget b = t;
        b.toleranceIn = std::nextafter(dist + s.fix.radius, 0.0);
        PieceFinder f{cfg(), fm()};
        f.setTarget(b);
        const PieceFinder::Result r = feed(f, {d}, 6);
        CHECK(r.verdict == PieceVerdict::Ambiguous);
        CHECK(r.why == PieceWhy::OnTheBorder);
        CHECK(r.border == 1);
    }
    // the target far enough that the fix could only just reach it: dist - radius == tolerance is the border;
    // the next double smaller tolerance puts the candidate certainly outside -> NONE
    PieceTarget o = camrig::yellowPinAt(s.fix.x + s.fix.radius + 2.0, s.fix.y);
    const double distO = std::hypot(s.fix.x - o.x, s.fix.y - o.y);
    o.toleranceIn = distO - s.fix.radius;
    REQUIRE(o.toleranceIn > 1.0);
    {
        PieceFinder f{cfg(), fm()};
        f.setTarget(o);
        const PieceFinder::Result r = feed(f, {d}, 6);
        CHECK(r.verdict == PieceVerdict::Ambiguous);
        CHECK(r.border == 1);
    }
    {
        PieceTarget out = o;
        out.toleranceIn = std::nextafter(distO - s.fix.radius, 0.0);
        PieceFinder f{cfg(), fm()};
        f.setTarget(out);
        const PieceFinder::Result r = feed(f, {d}, 6);
        CHECK(r.verdict == PieceVerdict::None);
        CHECK(r.why == PieceWhy::NothingThere);
    }
    {  // a tolerance smaller than the fix's own radius: AMBIGUOUS, the error larger than the tolerance
        PieceTarget v = camrig::yellowPinAt(s.fix.x, s.fix.y, s.fix.radius * 0.9);
        PieceFinder f{cfg(), fm()};
        f.setTarget(v);
        const PieceFinder::Result r = feed(f, {d}, 6);
        CHECK(r.verdict == PieceVerdict::Ambiguous);
        CHECK(r.why == PieceWhy::ErrorLargerThanTolerance);
    }
    {  // the reason's edge LANDED ON (VIS1-A2's review, RF5): the place 0.01 in off the fix's centre, the
       // tolerance EXACTLY the fix's radius -> still 'the error larger than the tolerance'; one double more ->
       // 'on the border' (the radius strictly under the tolerance is what makes it a border case)
        PieceTarget eq = camrig::yellowPinAt(s.fix.x + 0.01, s.fix.y, s.fix.radius);
        PieceFinder f{cfg(), fm()};
        f.setTarget(eq);
        const PieceFinder::Result r = feed(f, {d}, 6);
        CHECK(r.verdict == PieceVerdict::Ambiguous);
        CHECK(r.why == PieceWhy::ErrorLargerThanTolerance);
        eq.toleranceIn = std::nextafter(s.fix.radius, 1e9);
        PieceFinder g{cfg(), fm()};
        g.setTarget(eq);
        const PieceFinder::Result r2 = feed(g, {d}, 6);
        CHECK(r2.verdict == PieceVerdict::Ambiguous);
        CHECK(r2.why == PieceWhy::OnTheBorder);
    }
}

TEST_CASE("piece finder: two inside -> AMBIGUOUS 'two inside', never one of them; the class mask decides the type") {
    const Detection a = pinBox(40.0, -1.0);
    const Detection b = pinBox(40.0, 1.0);
    PieceFinder f{cfg(), fm()};
    f.setTarget(camrig::yellowPinAt(40.0, 0.0, 12.0));
    const PieceFinder::Result r = feed(f, {a, b}, 6);
    CHECK(r.verdict == PieceVerdict::Ambiguous);
    CHECK(r.why == PieceWhy::TwoInside);
    CHECK(r.inside == 2);
    PieceTarget redBlue = camrig::yellowPinAt(40.0, 0.0, 12.0);
    redBlue.classMask = 1U << 0;  // redBluePin: the yellow Pins are not it
    PieceFinder g{cfg(), fm()};
    g.setTarget(redBlue);
    const PieceFinder::Result n = feed(g, {a, b}, 6);
    CHECK(n.verdict == PieceVerdict::None);
    CHECK(n.why == PieceWhy::NothingThere);
    PieceTarget either = redBlue;
    either.classMask |= 1U << camrig::kYellowPin;
    PieceFinder h{cfg(), fm()};
    h.setTarget(either);
    CHECK(feed(h, {a, b}, 6).why == PieceWhy::TwoInside);
}

TEST_CASE("piece finder: an UNMEASURABLE box of the class whose bearing could point inside makes it AMBIGUOUS "
          "'unmeasurable inside' -- never dropped; one whose bearing cannot is ignored (NONE)") {
    Detection wide = pinBox(kPinX, 0.0);
    wide.width = static_cast<std::int16_t>(wide.height);  // aspect 1: between standing 0.8 and lying 1.5
    wide.left = static_cast<std::int16_t>(160 - wide.width / 2);
    PieceFinder f{cfg(), fm()};
    f.setTarget(named());
    const PieceFinder::Result r = feed(f, {wide}, 6);
    CHECK(r.verdict == PieceVerdict::Ambiguous);
    CHECK(r.why == PieceWhy::UnmeasurableInside);
    Detection aside = wide;
    aside.left = 290;  // ~33 degrees right: outside the disc's cone of bearings
    PieceFinder g{cfg(), fm()};
    g.setTarget(named());
    const PieceFinder::Result n = feed(g, {aside}, 6);
    CHECK(n.verdict == PieceVerdict::None);
}

TEST_CASE("piece finder: a candidate of the type in ANOTHER posture at the place -> NONE 'wrong posture inside'; the "
          "route that wants that posture takes it") {
    const Detection lying = pinBox(40.0, -3.25, true);  // across the view, its centre on the axis at x = 40
    PieceFinder f{cfg(), fm()};
    f.setTarget(camrig::yellowPinAt(40.0, 0.0, 10.0));
    const PieceFinder::Result r = feed(f, {lying}, 6);
    CHECK(r.verdict == PieceVerdict::None);
    CHECK(r.why == PieceWhy::WrongPostureInside);
    PieceFinder g{cfg(), fm()};
    g.setTarget(camrig::yellowPinAt(40.0, 0.0, 10.0, Posture::Lying));
    const PieceFinder::Result l = feed(g, {lying}, 6);
    CHECK(l.verdict == PieceVerdict::Found);
    CHECK(l.sighting.posture == Posture::Lying);
}

TEST_CASE("piece finder: the hold follows ONE candidate -- a fix that jumps by more than the two radii restarts it") {
    const Detection left = pinBox(40.0, 8.0);
    const Detection right = pinBox(40.0, -8.0);
    const vision::PieceSighting sl = vision::sightPiece(left, fm(), origin(), cfg());
    const vision::PieceSighting sr = vision::sightPiece(right, fm(), origin(), cfg());
    REQUIRE(sl.fix.valid);
    REQUIRE(sr.fix.valid);
    // the jump is more than the two radii (else "the same candidate" — the hold's own rule)
    REQUIRE(std::hypot(sl.fix.x - sr.fix.x, sl.fix.y - sr.fix.y) > sl.fix.radius + sr.fix.radius);
    PieceTarget t = camrig::yellowPinAt(40.0, 0.0);
    t.toleranceIn = std::fmax(std::hypot(sl.fix.x - t.x, sl.fix.y - t.y) + sl.fix.radius,
                              std::hypot(sr.fix.x - t.x, sr.fix.y - t.y) + sr.fix.radius);
    PieceFinder f{cfg(), fm()};
    f.setTarget(t);
    REQUIRE(feed(f, {left}, 6).verdict == PieceVerdict::Found);
    f.update(frame({right}, 2.0), origin());
    CHECK(f.answer(2.0).why == PieceWhy::NotHeldYet);  // another Pin, alone inside: a NEW hold
    f.update(frame({right}, 2.25), origin());
    CHECK(f.answer(2.25).verdict == PieceVerdict::Found);
    CHECK(f.answer(2.25).sighting.fix.y < 0.0);
}

TEST_CASE("piece finder: NOT IN VIEW from the WHOLE disc -- a place whose centre and four axis points are certainly "
          "in view but whose disc pokes out of the picture diagonally is NOT IN VIEW") {
    const vision::CameraModel m = fm();
    const vision::PoseBound pb = origin();
    const PieceFinderConfig c = cfg();
    const double tol = 6.0;
    const double fwdC = 40.0;
    double found = -1.0;
    for (double lft = 0.0; lft <= 40.0 && found < 0.0; lft += 0.05) {
        const double slack = tol + pb.positionTolerance.value() + m.placement().positionTolerance.value()
                             + std::hypot(fwdC, lft) * pb.headingToleranceDeg * vision::kDegToRad;
        auto in = [&](double f, double l) {
            return m.inViewForCertain(f, l, 0.0, c.viewMarginPx)
                   && m.inViewForCertain(f, l, vision::override2026::kPinSize.heightIn, c.viewMarginPx);
        };
        const bool fivePoints = in(fwdC, lft) && in(fwdC + slack, lft) && in(fwdC - slack, lft)
                                && in(fwdC, lft + slack) && in(fwdC, lft - slack);
        bool disc = true;
        for (int k = 0; k < 360; ++k) {
            const double a = k * kPi / 180.0;
            disc = disc && in(fwdC + slack * std::cos(a), lft + slack * std::sin(a));
        }
        if (fivePoints && !disc) {
            found = lft;
        }
    }
    REQUIRE(found > 0.0);
    PieceFinder f{c, m};
    f.setTarget(camrig::yellowPinAt(fwdC, found, tol));
    f.update(frame({}, 1.0), pb);
    const PieceFinder::Result r = f.answer(1.0);
    CHECK(r.verdict == PieceVerdict::NotInView);
    CHECK(r.why == PieceWhy::OutOfView);
    // the same disc moved back onto the axis is in view: an empty picture there is NONE
    PieceFinder g{c, m};
    g.setTarget(camrig::yellowPinAt(fwdC, 0.0, tol));
    g.update(frame({}, 1.0), pb);
    CHECK(g.answer(1.0).verdict == PieceVerdict::None);
}

TEST_CASE("piece finder: a TIGHT mount -- the floor fix's far end carries the radius beyond the ground range's far "
          "end (its centre is a radius past the rim the ground range bounds), or a well-measured camera loses the Pin") {
    // 12 in up, pitched down 25 (tightModel): the ground range is little more than the box edge's jitter
    // (0.35 in wide at 10 in, 0.9 in at 20 in), so a Pin's centre — 0.8 in past its rim — lies 0.3–0.6 in
    // BEYOND the ground range's far end (pinned below, so the scene stays on that edge) and only the radius
    // added there holds it. Past 20 in the margin is under a quarter radius.
    constexpr double kH = 12.0;
    constexpr double kPitch = -25.0;
    const vision::CameraModel m = tightModel(kH, kPitch);
    const vision::PoseBound pb = camrig::poseAt(0.0, 0.0, 0.0, 0.05, 0.05);
    const double r = vision::override2026::kPinSize.diameterIn / 2.0;
    int sightings = 0;
    for (double range = 10.0; range <= 20.0; range += 2.0) {
        for (const double bearDeg : {-10.0, 0.0, 10.0}) {
            const double px = camrig::kLensFwd + range * std::cos(bearDeg * kPi / 180.0);
            const double py = range * std::sin(bearDeg * kPi / 180.0);
            const vision::PieceSighting s = vision::sightPiece(tightBox(camrig::pin(px, py), kH, kPitch), m, pb, cfg());
            CAPTURE(range);
            CAPTURE(bearDeg);
            REQUIRE(s.groundRange.valid);
            // the scene really is the edge: the centre (a radius past the rim) is beyond the ground range
            CHECK(s.groundRange.hi < range);
            CHECK(range - s.groundRange.hi > 0.25 * r);
            CHECK(s.posture == Posture::OnFloor);
            REQUIRE(s.fix.valid);
            CHECK(std::hypot(s.fix.x - px, s.fix.y - py) <= s.fix.radius);
            ++sightings;
        }
    }
    CHECK(sightings == 18);
}

TEST_CASE("piece finder: a TIGHT mount over a Pin on a GOAL -- the RAISED fix's far end carries the top face's "
          "allowance (the size range reads NEARER than the near rim by the top face's depth)") {
    constexpr double kH = 18.0;
    constexpr double kPitch = -30.0;
    constexpr double kGoalTopIn = 5.8;  // the scenes' Goal (F3)
    // 18 in up, pitched down 30, over a Pin standing on a Goal (its top 12.3 in up, below the lens): the box
    // takes in the top face, so the size range reads its near rim NEARER by up to the top face's allowance
    // (topFaceAllowanceIn) — the centre lies 0.4–0.6 in past the size range's far end plus a radius from 10 to
    // 14 in (pinned below; farther out the margin closes), and only the allowance added there holds it.
    const vision::CameraModel m = tightModel(kH, kPitch);
    const vision::PoseBound pb = camrig::poseAt(0.0, 0.0, 0.0, 0.05, 0.05);
    const double r = vision::override2026::kPinSize.diameterIn / 2.0;
    int sightings = 0;
    for (double range = 10.0; range <= 14.0; range += 2.0) {
        for (const double bearDeg : {-10.0, 0.0, 10.0}) {
            const double px = camrig::kLensFwd + range * std::cos(bearDeg * kPi / 180.0);
            const double py = range * std::sin(bearDeg * kPi / 180.0);
            const vision::PieceSighting s =
                vision::sightPiece(tightBox(camrig::pin(px, py, kGoalTopIn), kH, kPitch), m, pb, cfg());
            CAPTURE(range);
            CAPTURE(bearDeg);
            REQUIRE(s.sizeRange.valid);
            // the scene really is the edge: the centre is beyond the size range's far end plus a radius
            CHECK(range - (s.sizeRange.hi + r) > 0.25 * r);
            CHECK(s.posture == Posture::Raised);
            REQUIRE(s.fix.valid);
            CHECK(std::hypot(s.fix.x - px, s.fix.y - py) <= s.fix.radius);
            ++sightings;
        }
    }
    CHECK(sightings == 9);
}

TEST_CASE("piece finder: a TIGHT mount over a Pin lying DIAGONALLY -- the LYING fix's far end carries half a length "
          "(its centre is up to half a length past its nearest point, not just a radius)") {
    // 6 in up, pitched down 10, a Pin lying at 45 degrees to the view with its NEAR end 12..27 in out (the
    // host model's lying Pin runs from its (x, y) along its heading): wide and flat enough to be LYING, and
    // its centre 1.0–1.3 in past the ground range's far end plus a radius (pinned below) — every lying scene
    // before this one lay ACROSS the view, where a radius is all the centre is past its nearest point.
    constexpr double kH = 6.0;
    constexpr double kPitch = -10.0;
    const vision::CameraModel m = tightModel(kH, kPitch);
    const vision::PoseBound pb = camrig::poseAt(0.0, 0.0, 0.0, 0.05, 0.05);
    const double r = vision::override2026::kPinSize.diameterIn / 2.0;
    const double half = vision::override2026::kPinSize.heightIn / 2.0;
    int sightings = 0;
    for (double range = 12.0; range <= 27.0; range += 3.0) {
        for (const double headDeg : {-45.0, 45.0}) {
            const double hd = headDeg * kPi / 180.0;
            const double ex = camrig::kLensFwd + range;  // the near end
            const double ey = 0.0;
            sim::Element e = camrig::pin(ex, ey, 0.0, true);
            e.lyingHeadingRad = hd;
            const double cx = ex + half * std::cos(hd);
            const double cy = ey + half * std::sin(hd);
            const vision::PieceSighting s = vision::sightPiece(tightBox(e, kH, kPitch), m, pb, cfg());
            CAPTURE(range);
            CAPTURE(headDeg);
            const double centreRange = std::hypot(cx - camrig::kLensFwd, cy);
            REQUIRE(s.groundRange.valid);
            // the scene really is the edge: the centre is beyond the ground range's far end plus a radius
            CHECK(centreRange - (s.groundRange.hi + r) > 0.25 * r);
            CHECK(s.posture == Posture::Lying);
            REQUIRE(s.fix.valid);
            CHECK(std::hypot(s.fix.x - cx, s.fix.y - cy) <= s.fix.radius);
            ++sightings;
        }
    }
    CHECK(sightings == 12);
}

// ─── VIS1-A2's review ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("piece finder: with NO target set -- or one pieceTargetProblem() refuses -- every answer is UNKNOWN 'no "
          "target', never NONE (nothing named is nothing known missing); a valid target then answers as usual") {
    const Detection pin = pinBox(kPinX, 0.0);
    PieceFinder f{cfg(), fm()};
    const PieceFinder::Result none = feed(f, {pin}, 6);
    CHECK(none.verdict == PieceVerdict::Unknown);
    CHECK(none.why == PieceWhy::NoTarget);
    CHECK(f.answer(-1.0).why == PieceWhy::NoTarget);  // before anything else, even an old read
    PieceTarget bad = named();
    bad.toleranceIn = 0.0;  // UNSET: refused
    REQUIRE(vision::pieceTargetProblem(bad) != nullptr);
    f.setTarget(bad);
    CHECK(feed(f, {pin}, 6).why == PieceWhy::NoTarget);
    f.setTarget(named());
    const PieceFinder::Result r = feed(f, {pin}, 6, 3.0);
    CHECK(r.verdict == PieceVerdict::Found);
    PieceFinder fresh{cfg(), fm()};
    CHECK(fresh.answer(0.0).why == PieceWhy::NoTarget);
    CHECK(std::string{vision::pieceWhyWords(PieceWhy::NoTarget)} == "no target");
}

TEST_CASE("piece finder: a Pin LYING near END-ON is never 'standing on the floor' -- on the floor (a high lens: "
          "its box as tall and narrow as a standing Pin's, the lying exclusion) and on a Goal's top (its box far "
          "too WIDE for a Pin at the range its height implies, the width check)") {
    // on the floor, its axis 5 degrees off the line of sight, an 18-in lens pitched down 20 (the review's case:
    // a 15 x 37 box read ON-FLOOR with a fix 3.5 in short of its centre)
    const vision::CameraModel high = camrig::model(18.0, -20.0);
    int floorLying = 0;
    for (double range = 14.0; range <= 50.0; range += 2.0) {
        for (const double off : {0.0, 0.09, -0.09, 0.2}) {
            sim::TrueCamera tc = exactTruth();
            tc.height = 18.0;
            tc.pitchDeg = -20.0;
            sim::CameraPlant plant{tc, 1};
            sim::Element e = camrig::pin(camrig::kLensFwd + range, 0.0, 0.0, true);
            e.lyingHeadingRad = off;
            plant.addElement(e);
            const VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
            for (std::uint8_t i = 0; i < fr.count; ++i) {
                if (fr.detections[i].clipped()) {
                    continue;
                }
                ++floorLying;
                CAPTURE(range);
                CAPTURE(off);
                CHECK(vision::sightPiece(fr.detections[i], high, origin(), cfg()).posture != Posture::OnFloor);
            }
        }
    }
    CHECK(floorLying > 40);
    // on a Goal's top (5.8 in), end-on, a 10-in lens pitched down 10 (the review's 33 x 47 box read ON-FLOOR,
    // its fix 15.6 in from the Pin, and a route for a standing floor Pin there told FOUND)
    int raisedLying = 0;
    for (double range = 10.0; range <= 30.0; range += 1.0) {
        sim::CameraPlant plant{exactTruth(), 3};
        sim::Element e = camrig::pin(camrig::kLensFwd + range, 0.0, 5.8, true);
        e.lyingHeadingRad = 0.0;
        plant.addElement(e);
        const VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
        if (fr.count != 1 || fr.detections[0].clipped()) {
            continue;
        }
        ++raisedLying;
        CAPTURE(range);
        const vision::PieceSighting s = vision::sightPiece(fr.detections[0], fm(), origin(), cfg());
        CHECK(s.posture != Posture::OnFloor);
    }
    CHECK(raisedLying > 10);
    // and the same rules keep a STANDING floor Pin standing at those mounts (its box's width IS a Pin's)
    for (double range = 20.0; range <= 50.0; range += 5.0) {
        sim::TrueCamera tc = exactTruth();
        tc.height = 18.0;
        tc.pitchDeg = -20.0;
        sim::CameraPlant plant{tc, 1};
        plant.addElement(camrig::pin(camrig::kLensFwd + range, 0.0));
        const VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
        REQUIRE(fr.count == 1);
        CAPTURE(range);
        const vision::PieceSighting s = vision::sightPiece(fr.detections[0], high, origin(), cfg());
        CHECK(s.posture != Posture::Raised);
        CHECK(s.posture != Posture::Lying);
    }
}

TEST_CASE("piece finder: a box CUT by the image's bottom is dropped only when CERTAINLY nearer than the disc -- each "
          "term the disc's nearest point takes off (the tolerance, the pose's position tolerance, the lens's swing "
          "over the heading tolerance, the mount's position tolerance) LANDED ON: the place moved out by less than "
          "that term keeps the box (AMBIGUOUS), where a bound without it would drop it (NONE)") {
    const Detection cut = pinBox(14.0, 0.0);  // its base below the picture, its top in it
    REQUIRE(cut.clipped());
    const vision::PieceSighting s = vision::sightPiece(cut, fm(), origin(), cfg());
    REQUIRE(s.nearerThanIn > 0.0);
    const double tol = 3.0;
    const double posTol = origin().positionTolerance.value();
    const double swing = std::hypot(fm().placement().forward.value(), fm().placement().left.value())
                         * origin().headingToleranceDeg * vision::kDegToRad;
    const double mount = fm().placement().positionTolerance.value();
    const double all = tol + posTol + swing + mount;
    for (const double term : {tol, posTol, swing, mount}) {
        CAPTURE(term);
        // the place straight ahead, its nearest point (all terms off) just NEARER than the box's bound by
        // less than this one term
        const double lensToPlace = s.nearerThanIn + all - std::fmin(term, 0.4) / 2.0;
        PieceFinder f{cfg(), fm()};
        f.setTarget(camrig::yellowPinAt(fm().placement().forward.value() + lensToPlace, 0.0, tol));
        const PieceFinder::Result r = feed(f, {cut}, 6);
        CHECK(r.verdict == PieceVerdict::Ambiguous);
        CHECK(r.why == PieceWhy::UnmeasurableInside);
    }
    // and its bound: the farthest floor point at the last row a box can end on, less its edge's jitter, at
    // EITHER jittered column, for every camera, plus a radius off the axis (the header's rule, written again)
    {
        const Detection side = camrig::objectBox(3, 230, 150, 30, 90);  // off the axis, cut only by the bottom
        REQUIRE(side.top + side.height >= hal::kVisionImageHeightPx - 1);
        const vision::PieceSighting ss = vision::sightPiece(side, fm(), origin(), cfg());
        const double u = side.left + side.width / 2.0;
        const camrig::config::CameraPlacement p = fm().placement();
        const double offAxis =
            std::fmin(80.0 * vision::kDegToRad, std::fmax(std::fabs(ss.bearing.lo), std::fabs(ss.bearing.hi))
                                                    + (std::fabs(p.yawDeg) + p.yawToleranceDeg) * vision::kDegToRad);
        const double sec = 1.0 / std::cos(offAxis);
        double bound = 0.0;
        fm().eachCorner(true, [&](const vision::CameraCorner& c) {
            for (const double du : {-vision::kBoxEdgePx, vision::kBoxEdgePx}) {
                double g = 0.0;
                REQUIRE(vision::CameraModel::rangeToPlane(c, u + du, hal::kVisionImageHeightPx - 1 - vision::kBoxEdgePx,
                                                          0.0, 2.0, g));
                bound = std::fmax(bound, g);
            }
        });
        CHECK(ss.nearerThanIn == bound + vision::override2026::kPinSize.diameterIn / 2.0 * sec);
        // cut at the LEFT as well: no bound at all
        const Detection corner = camrig::objectBox(3, 0, 150, 30, 90);
        CHECK(vision::sightPiece(corner, fm(), origin(), cfg()).nearerThanIn == 0.0);
    }
}

namespace {

// bearingCouldBeInside(), written again from its header (an oracle): the place's disc, seen from the lens, as a
// cone of bearings widened by the heading tolerance; `posTolTerm` and `headTerm` scale its two terms.
bool coneHolds(const vision::Interval& b, double fwd, double lft, double tol, double posTol, double mountTol,
               double headTolRad) {
    const double dist = std::hypot(fwd, lft);
    const double slack = tol + posTol + mountTol;
    const double centre = std::atan2(lft, fwd);
    const double half = std::asin(slack / dist) + headTolRad;
    return !(b.hi < centre - half) && !(b.lo > centre + half);
}

}  // namespace

TEST_CASE("piece finder: an UNMEASURABLE box counts while its bearing could point inside -- the cone's edge LANDED "
          "between the bound with and without each of its terms (the heading tolerance, the pose's position "
          "tolerance); a lens INSIDE the place's disc counts anything (a camera pitched steeply down)") {
    const Detection squat = camrig::objectBox(3, 150, 100, 30, 30);  // aspect 1: neither standing nor lying
    const vision::PieceSighting s = vision::sightPiece(squat, fm(), origin(), cfg());
    REQUIRE(s.posture == Posture::Unsure);
    REQUIRE_FALSE(s.fix.valid);
    REQUIRE(s.bearing.valid);
    const double tol = 3.0;
    const double R = 30.0;  // lens to the place
    const double posTol = origin().positionTolerance.value();
    const double mountTol = fm().placement().positionTolerance.value();
    const double head = origin().headingToleranceDeg * vision::kDegToRad;
    // the place to the LEFT of the box's bearing: its cone's right edge at the box's left end, less half of
    // what one term adds
    const double halfAll = std::asin((tol + posTol + mountTol) / R) + head;
    const double halfNoHead = std::asin((tol + posTol + mountTol) / R);
    const double halfNoPos = std::asin((tol + mountTol) / R) + head;
    for (const double lessBy : {halfAll - halfNoHead, halfAll - halfNoPos}) {
        CAPTURE(lessBy);
        const double phi = s.bearing.hi + halfAll - lessBy / 2.0;
        const double fwd = R * std::cos(phi);
        const double lft = R * std::sin(phi);
        REQUIRE(coneHolds(s.bearing, fwd, lft, tol, posTol, mountTol, head));
        PieceFinder f{cfg(), fm()};
        f.setTarget(camrig::yellowPinAt(fm().placement().forward.value() + fwd, lft, tol));
        const PieceFinder::Result r = feed(f, {squat}, 6);
        CHECK(r.verdict == PieceVerdict::Ambiguous);
        CHECK(r.why == PieceWhy::UnmeasurableInside);
        // a little farther out (the whole term past the edge): it cannot be inside
        PieceFinder g{cfg(), fm()};
        const double phiOut = s.bearing.hi + halfAll + 0.002;
        g.setTarget(camrig::yellowPinAt(fm().placement().forward.value() + R * std::cos(phiOut), R * std::sin(phiOut),
                                        tol));
        CHECK(feed(g, {squat}, 6).verdict == PieceVerdict::None);
    }
    // the lens INSIDE the place's disc: a camera 10 in up pitched 70 degrees down, the place 1 in ahead of the
    // lens within 0.5 in (the lens within the tolerance, the pose's and the mount's, 1.25 in): a box far to the
    // left could be anywhere
    const vision::CameraModel steep = camrig::model(10.0, -70.0);
    PieceFinder f{cfg(), steep};
    f.setTarget(camrig::yellowPinAt(steep.placement().forward.value() + 1.0, 0.0, 0.5));
    const PieceFinder::Result r = feed(f, {camrig::objectBox(3, 20, 100, 30, 30)}, 6);
    CHECK(r.verdict == PieceVerdict::Ambiguous);
    CHECK(r.why == PieceWhy::UnmeasurableInside);
}

TEST_CASE("piece finder: NOT IN VIEW checks the FLOOR as well as a Pin's height (a place whose floor is below the "
          "picture, a Pin's top in it), and widens the place by the heading tolerance at its distance (LANDED "
          "between the two)") {
    {  // 12 in from the lens: the floor seen from about 11.3 in out, the place's near edge nearer
        PieceFinder f{cfg(), fm()};
        f.setTarget(camrig::yellowPinAt(fm().placement().forward.value() + 12.0, 0.0, 1.0));
        const PieceFinder::Result r = feed(f, {}, 2);
        CHECK(r.verdict == PieceVerdict::NotInView);
    }
    // the heading's slack: a place 30 in out moved sideways until it leaves the certain view WITH the slack but
    // not without (the octagon test written again: every corner, at the floor and at a Pin's height)
    auto inView = [&](double ty, double slackHead) {
        const double dx0 = 37.0;
        const double dy0 = ty;
        const double slack = 3.0 + origin().positionTolerance.value() + fm().placement().positionTolerance.value()
                             + slackHead * std::hypot(dx0, dy0) * origin().headingToleranceDeg * vision::kDegToRad;
        const double corner = slack / std::cos(camrig::kPi / 8.0);
        for (int k = 0; k < 8; ++k) {
            const double a = k * camrig::kPi / 4.0;
            const double fwd = dx0 + corner * std::cos(a);
            const double lft = dy0 + corner * std::sin(a);
            if (!fm().inViewForCertain(fwd, lft, 0.0, cfg().viewMarginPx)
                || !fm().inViewForCertain(fwd, lft, vision::override2026::kPinSize.heightIn, cfg().viewMarginPx)) {
                return false;
            }
        }
        return true;
    };
    double y = 0.0;
    while (inView(y + 0.01, 1.0)) {
        y += 0.01;
    }
    y += 0.01;  // the first sideways place OUT of the certain view with the slack
    REQUIRE_FALSE(inView(y, 1.0));
    REQUIRE(inView(y, 0.0));  // without the heading's slack it would be in view
    PieceFinder f{cfg(), fm()};
    f.setTarget(camrig::yellowPinAt(37.0, y, 3.0));
    CHECK(feed(f, {}, 2).verdict == PieceVerdict::NotInView);
}

TEST_CASE("piece finder: ON THE FLOOR needs the raise excluded -- the lowest raise LANDED where its slack and the "
          "lens's height tolerance decide (UNSURE), and a lowest raise at or above the lens excludes nothing it "
          "could see (ON-FLOOR)") {
    const Detection d = pinBox(kPinX, 0.0);
    const vision::PieceSighting base = vision::sightPiece(d, fm(), origin(), cfg());
    REQUIRE(base.posture == Posture::OnFloor);
    const double g = base.groundRange.hi;
    const double z = base.sizeRange.lo;
    REQUIRE(z < g);
    const double lensHi = fm().placement().height.value() + fm().placement().heightTolerance.value();
    const double lensNom = fm().placement().height.value();
    auto posture = [&](double raisedMin) {
        PieceFinderConfig c = cfg();
        c.raisedMinIn = raisedMin;
        return vision::sightPiece(d, fm(), origin(), c).posture;
    };
    // the slack: z a / (a - r) = g + slack / 2  ->  UNSURE (it needs g + slack below it)
    const double rSlack = lensHi * (1.0 - z / (g + vision::kPostureSlackIn / 2.0));
    CHECK(posture(rSlack) == Posture::Unsure);
    // the lens's height tolerance: the raise r with z a / (a - r) = g + slack between the believed lens and the
    // highest one -> UNSURE (from the highest lens the raise still fits)
    const double rLo = lensHi * (1.0 - z / (g + vision::kPostureSlackIn));
    const double rHi = lensNom * (1.0 - z / (g + vision::kPostureSlackIn));
    REQUIRE(rHi < rLo);
    CHECK(posture(0.5 * (rLo + rHi)) == Posture::Unsure);
    // a lowest raise above both (the raise it must exclude higher, so farther): excluded, ON-FLOOR
    CHECK(posture(0.5 * (rLo + lensHi)) == Posture::OnFloor);
    // a lowest raise AT the highest lens (or above): no raise it could see -> ON-FLOOR; one just under it is a
    // raise a camera there could see from below its level
    CHECK(posture(lensHi) == Posture::OnFloor);
    CHECK(posture(lensHi + 1.0) == Posture::OnFloor);
}

TEST_CASE("piece finder: RAISED with the lens BELOW the support has no fix (its box bottom looks up for some "
          "camera); with the lens just under the Pin's top, the RAISED fix's near end still holds the centre") {
    {  // a 6-in level lens, a Pin on a support 8 in up, 30 in out
        sim::TrueCamera tc = exactTruth();
        tc.height = 6.0;
        tc.pitchDeg = 0.0;
        sim::CameraPlant plant{tc, 1};
        plant.addElement(camrig::pin(30.0, 0.0, 8.0));
        const VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
        REQUIRE(fr.count == 1);
        const vision::PieceSighting s = vision::sightPiece(fr.detections[0], camrig::model(6.0), origin(), cfg());
        CHECK(s.posture == Posture::Raised);
        CHECK_FALSE(s.fix.valid);
    }
    // a camera measured to 0.01 in / 0.01 degree, 12 in up (under the Goal Pin's 12.3-in top: no top-face
    // allowance), pitched 10 down: the size range bounds the near rim from below tightly
    int raised = 0;
    for (double x = 20.0; x <= 40.0; x += 1.0) {
        sim::Element e = camrig::pin(x, 0.0, 5.8);
        const Detection d = tightBox(e, 12.0, -10.0);
        const vision::PieceSighting s = vision::sightPiece(d, tightModel(12.0, -10.0), origin(), cfg());
        if (s.posture != Posture::Raised) {
            continue;
        }
        ++raised;
        CAPTURE(x);
        REQUIRE(s.fix.valid);
        CHECK(std::hypot(s.fix.x - x, s.fix.y) <= s.fix.radius);
    }
    CHECK(raised > 5);
}

TEST_CASE("piece finder: the fix's near ends LANDED (VIS1-A2's review, found by a scene sweep diffed against each "
          "surviving mutant) -- RAISED: the size range's near end plus ONE radius, no more (a Pin on a Goal 8 in "
          "from an 8-in level lens measured to 0.01 in: its centre 0.2 in inside a 0.39-in fix); ON THE FLOOR: "
          "only when the rim's two bounds INTERSECT (a box whose size range ends before its ground range begins "
          "agreed only through the posture slack -> UNSURE, never a floor piece without a fix); and COVERAGE: the "
          "width check allows each box edge its jitter (a 6-px-wide standing Pin 52 in out still stands)") {
    {
        const double px = camrig::kLensFwd + 8.0 * std::cos(-6.0 * camrig::kPi / 180.0);
        const double py = 8.0 * std::sin(-6.0 * camrig::kPi / 180.0);
        const Detection d = tightBox(camrig::pin(px, py, 5.8), 8.0, 0.0);
        const vision::PieceSighting s =
            vision::sightPiece(d, tightModel(8.0, 0.0), camrig::poseAt(0.0, 0.0, 0.0, 0.0, 0.0), cfg());
        REQUIRE(s.posture == Posture::Raised);
        REQUIRE(s.fix.valid);
        CHECK(std::hypot(s.fix.x - px, s.fix.y - py) <= s.fix.radius);
    }
    {  // a Pin on a 2-in support, 44 in from a 10-in level lens measured to 0.01 in (below `raisedMinIn`, so
       // the raise exclusion passes it: only the intersection stands between it and ON THE FLOOR)
        const double px = camrig::kLensFwd + 44.0 * std::cos(6.0 * camrig::kPi / 180.0);
        const double py = 44.0 * std::sin(6.0 * camrig::kPi / 180.0);
        const Detection d = tightBox(camrig::pin(px, py, 2.0), 10.0, 0.0);
        const vision::PieceSighting s =
            vision::sightPiece(d, tightModel(10.0, 0.0), camrig::poseAt(0.0, 0.0, 0.0, 0.0, 0.0), cfg());
        REQUIRE(s.groundRange.valid);
        REQUIRE(s.sizeRange.valid);
        REQUIRE(s.sizeRange.hi < s.groundRange.lo);  // the two bounds on the near rim do not meet
        CHECK(s.posture == Posture::Unsure);
        CHECK_FALSE(s.fix.valid);
    }
    {  // COVERAGE: the width check allows each box edge its jitter — a standing Pin 52 in from a 6-in level lens
       // measured to 0.01 in, its box edges jittered by 1 px (6 px wide), still stands, its fix holding it
        const double px = camrig::kLensFwd + 52.0;
        sim::CameraPlant plant{tightTruth(6.0, 0.0), 32};
        plant.setJitterPx(1.0);
        plant.addElement(camrig::pin(px, 0.0));
        const VisionFrame fr = plant.read(0.0, 0.0, 0.0, Time{0.0});
        REQUIRE(fr.count == 1);
        const vision::PieceSighting s = vision::sightPiece(fr.detections[0], tightModel(6.0, 0.0),
                                                           camrig::poseAt(0.0, 0.0, 0.0, 0.0, 0.0), cfg());
        CHECK(s.posture == Posture::OnFloor);
        CHECK((s.fix.valid && std::hypot(s.fix.x - px, s.fix.y) <= s.fix.radius));
    }
}

TEST_CASE("piece finder: a POSSIBLY FULL read, or one whose model is not confirmed, in the middle of a FOUND hold "
          "breaks it -- the next clean read is 'not held yet' (positive evidence only)") {
    const Detection pin = pinBox(kPinX, 0.0);
    for (const bool truncated : {true, false}) {
        CAPTURE(truncated);
        PieceFinder f{cfg(), fm()};
        f.setTarget(named());
        REQUIRE(feed(f, {pin}, 5).verdict == PieceVerdict::Found);  // held exactly 0.25 s
        VisionFrame bad = frame({pin}, 1.3125);
        if (truncated) {
            bad.truncated = true;
        } else {
            bad.model = hal::ModelCheck::Different;
        }
        f.update(bad, origin());
        CHECK(f.answer(1.3125).verdict == PieceVerdict::Unknown);
        f.update(frame({pin}, 1.375), origin());
        const PieceFinder::Result r = f.answer(1.375);
        CHECK(r.verdict == PieceVerdict::Unknown);
        CHECK(r.why == PieceWhy::NotHeldYet);
    }
}

TEST_CASE("piece finder: the LYING exclusion's two margins LANDED on hand-made boxes (VIS1-A2's review: found by a "
          "search over every narrow box through four cameras, diffed against each surviving mutant) -- the lying "
          "Pin's reach is its DIAGONAL (length and diameter), and the box's bottom row moves by its jitter: a box "
          "that either margin alone decides is never 'standing on the floor'") {
    const vision::PoseBound exact = camrig::poseAt(0.0, 0.0, 0.0, 0.0, 0.0);
    for (const int w : {3, 4}) {  // a tiny, narrow box through a camera measured to 0.01 in, 10 in up pitched -10
        CAPTURE(w);
        const Detection d = camrig::objectBox(camrig::kYellowPin, 146, 92, w, 6);
        CHECK(vision::sightPiece(d, tightModel(10.0, -10.0), exact, cfg()).posture != Posture::OnFloor);
    }
    for (int w = 3; w <= 7; ++w) {  // at the picture's left, through the stated tolerances
        CAPTURE(w);
        const Detection d = camrig::objectBox(camrig::kYellowPin, 2, 98, w, 9);
        REQUIRE_FALSE(d.clipped());
        CHECK(vision::sightPiece(d, fm(), exact, cfg()).posture != Posture::OnFloor);
    }
}

TEST_CASE("piece finder: RAISED's nearest ground includes the PRINCIPAL COLUMN when the box spans it (along a row the "
          "floor is nearest straight ahead) -- two hand-made boxes centred on it that the column keeps from RAISED "
          "(and the RAISED test's slack too: drop either and they read RAISED) (VIS1-A2's review, found by a search "
          "over every narrow box)") {
    const vision::PoseBound exact = camrig::poseAt(0.0, 0.0, 0.0, 0.0, 0.0);
    for (const int w : {27, 29}) {
        CAPTURE(w);
        const Detection d = camrig::objectBox(camrig::kYellowPin, 146, 98, w, 87);
        REQUIRE(std::fabs(d.left + d.width / 2.0 - fm().nominal().cx) < vision::kBoxEdgePx);
        CHECK(vision::sightPiece(d, fm(), exact, cfg()).posture != Posture::Raised);
    }
}

TEST_CASE("piece finder: ON THE FLOOR only when the rim's two bounds INTERSECT -- the MIRRORED side: a box whose size "
          "range BEGINS after its ground range ends (too short for a Pin standing where its bottom row is) is UNSURE "
          "with no fix, never a floor piece placed by the ground range alone (VIS1-A2's verification round, found "
          "by a search over every box through four camera models)") {
    // Through cameras measured to 0.01 in. The other side (a size range that ENDS before the ground range
    // begins: a Pin on a 2-in support) is the "fix's near ends LANDED" case above.
    struct Box {
        double h, pitch;
        int left, top, width, height;
    };
    for (const Box& b : {Box{10.0, -10.0, 110, 108, 10, 24}, Box{10.0, -10.0, 110, 108, 11, 24},
                         Box{18.0, -20.0, 110, 100, 12, 18}}) {
        CAPTURE(b.h);
        CAPTURE(b.width);
        const Detection d = camrig::objectBox(camrig::kYellowPin, b.left, b.top, b.width, b.height);
        REQUIRE_FALSE(d.clipped());
        const vision::PieceSighting s =
            vision::sightPiece(d, tightModel(b.h, b.pitch), camrig::poseAt(0.0, 0.0, 0.0, 0.0, 0.0), cfg());
        REQUIRE(s.groundRange.valid);
        REQUIRE(s.sizeRange.valid);
        REQUIRE(s.sizeRange.lo > s.groundRange.hi);  // the two bounds on the near rim do not meet
        CHECK(s.posture == Posture::Unsure);
        CHECK_FALSE(s.fix.valid);
    }
}
