// The wall reset's ONE LOG LINE, byte for byte (chunk L6-A1; the L6 brief's T9 "carrying its
// numbers", and guide 11's `WALL` section — added by the L6-A1 review). Every verdict's line is pinned
// as an exact string, built by `localization::appendWallResetLine` from a report the library's OWN
// pure functions computed (preCheck / describeWallReset / evaluateWallReset on the season's left
// wall) wherever the verdict can come from them, and from a hand-built report only for the verdicts
// only a call can reach (the sensor, the mount, liveness, motion, the everyday form's three).
//
// GUIDE 11 QUOTES the six lines of "L6 T9 guide 11's six WALL lines" word for word (without the
// terminal's `[t=...]` prefix, which test/term_sink_test.cpp pins) — and, since MIRROR1-A, a seventh: a
// mirrored auton's line (kGuideMirroredLine, pinned by "MIRROR1 T8 ..."): change a line's format and this
// file AND guide 11 change together. Nothing here has run on a robot.

#include "doctest.h"

#include <fstream>
#include <sstream>
#include <span>
#include <string>
#include <vector>

#include "shulib/diag/line_format.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace {

namespace loc = shulib::localization;
namespace ow = shulib::localization::override2026;
using loc::WallResetVerdict;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::math::Twist2d;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

constexpr double kWall = 70.205;  // the left wall's inner face, x = -70.205

/// The mount a robot file types for a left sensor 7.5 in out at the fore-aft centre, 10.5 in up.
loc::SideMount leftMount() {
    loc::SideMount m{};
    m.measured = true;
    m.forward = Length{0.0};
    m.left = Length{7.5};
    m.height = Length{10.5};
    m.facing = AngleDim{90.0 * loc::kWallDegToRad};
    return m;
}

loc::WallResetRequest leftRequest(double maxCorrection = 3.0) {
    loc::WallResetRequest r{};
    r.wall = ow::kLeftWall;
    r.sensor = loc::WallSide::Left;
    r.maxCorrection = Length{maxCorrection};
    return r;
}

/// The robot with its left lens `reading` in from the left wall (the estimate), at `y`, nose `deg`.
Pose2d parked(double reading, double y = -38.0, double deg = 90.0) {
    return Pose2d{Length{-kWall + 7.5 + reading}, Length{y}, Angle::degrees(deg)};
}

/// A window of readings 0.01 s apart, confidence `conf`.
std::vector<loc::WallSample> window(std::initializer_list<double> readings, double conf = 1.0) {
    std::vector<loc::WallSample> s;
    double t = 0.0;
    for (const double r : readings) {
        s.push_back(loc::WallSample{Length{r}, conf, false, Time{t}});
        t += 0.01;
    }
    return s;
}

/// The library's verdict on a window, from the estimate at `pose` (the explicit call's rules 5-9).
loc::WallResetReport judge(const std::vector<loc::WallSample>& s, const Pose2d& pose,
                           const loc::WallResetRequest& rq, const loc::WallResetLimits& l = {},
                           const Twist2d& worst = Twist2d{}) {
    return loc::evaluateWallReset(std::span<const loc::WallSample>{s.data(), s.size()}, pose, worst,
                                  leftMount(), rq, l, loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
}

/// A pre-sampling refusal's report, as the call builds it.
loc::WallResetReport refuseBefore(const Pose2d& pose, const loc::WallResetRequest& rq,
                                  const loc::WallResetLimits& l = {}) {
    loc::WallResetReport rep =
        loc::describeWallReset(pose, leftMount(), rq, l, loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
    rep.verdict = loc::preCheck(pose, leftMount(), rq, l, Length{0.0});
    return rep;
}

/// The line, as the call writes it (the message: no `[t=...]` prefix). `note`: a mirrored auton's note
/// (MIRROR1-A), written inside the sensor's parenthesis.
std::string line(const loc::WallResetReport& rep, const loc::WallResetRequest& rq = leftRequest(),
                 const loc::WallResetLimits& l = {}, const char* note = nullptr) {
    shulib::diag::lineformat::Line out;
    loc::appendWallResetLine(out, rep, rq, leftMount(), l, note);
    return std::string{out.view()};
}

/// MIRROR1-A: guide 11's seventh WALL line — robot two's reset in a MIRRORED auton: robot one's route asked
/// for the blue wall with its RIGHT sensor, so robot two reads the left wall (the blue wall's image across
/// the skills diagonal) with its LEFT sensor, and the line says so inside the sensor's parenthesis.
const char* const kGuideMirroredLine =
    "left wall x SEATED (LEFT sensor, mirrored from blue wall RIGHT): -53.30 -> -55.71 (moved -2.41, limit 3.00) | "
    "median 6.99 in spread 0.25 n 16 | bound 0.87 | faults ok";

/// Guide 11's six example lines (the messages, after the terminal's `[t=...]` prefix), in its order.
const char* const kGuideLines[6] = {
    "left wall x SEATED (LEFT sensor): -53.30 -> -55.71 (moved -2.41, limit 3.00) | median 6.99 in spread 0.25 n "
    "16 | bound 0.87 | faults ok",
    "left wall x REFUSED disagrees (LEFT sensor): wall says -55.72, estimate -52.20, limit 3.00 | median 6.98 in "
    "spread 0.25 n 16",
    "left wall x REFUSED keep-out (LEFT sensor): Load Zone at along -46.0 (widened 5.3 = beam 2.3 + margin 3.0 + "
    "drift 0.0) | clean -41.8..-33.6",
    "left wall x REFUSED not square (LEFT sensor): beam 7.0 deg off square (limit 5.0)",
    "left wall x REFUSED beam low (LEFT sensor): expected 11.00 in past the ceiling 10.77 in (lens 10.50 in up, cone "
    "18.0 deg)",
    "left wall x REFUSED bad request (LEFT sensor): maxCorrection UNSET: say how far this wall may move x",
};

/// Sixteen readings: a median of 6.99 and a spread of 0.25.
std::vector<loc::WallSample> sixteen(double centre) {
    return window({centre - 0.13, centre - 0.1, centre - 0.08, centre - 0.06, centre - 0.04, centre - 0.02,
                   centre - 0.01, centre, centre, centre + 0.01, centre + 0.02, centre + 0.04, centre + 0.06,
                   centre + 0.08, centre + 0.1, centre + 0.12});
}

}  // namespace

// Bug caught (the L6-A1 review — S21, S21b): the SEATED line printing `moved` with the wrong sign, or
// the answer where the estimate before belongs; any verdict's numbers or words drifting from guide
// 11's lines. Each line below is guide 11's, exactly.
TEST_CASE("L6 T9 guide 11's six WALL lines, byte for byte") {
    const loc::WallResetReport seated = judge(sixteen(6.99), parked(9.4), leftRequest());
    REQUIRE(seated.verdict == WallResetVerdict::Seated);
    CHECK(line(seated) == std::string{kGuideLines[0]});

    const loc::WallResetReport disagrees = judge(sixteen(6.98), parked(10.5), leftRequest());
    REQUIRE(disagrees.verdict == WallResetVerdict::Disagrees);
    CHECK(line(disagrees) == std::string{kGuideLines[1]});

    const loc::WallResetReport keepOut = refuseBefore(parked(7.0, -46.0), leftRequest());
    REQUIRE(keepOut.verdict == WallResetVerdict::KeepOut);
    CHECK(line(keepOut) == std::string{kGuideLines[2]});

    const loc::WallResetReport notSquare = refuseBefore(parked(7.0, -38.0, 97.0), leftRequest());
    REQUIRE(notSquare.verdict == WallResetVerdict::NotSquare);
    CHECK(line(notSquare) == std::string{kGuideLines[3]});

    const loc::WallResetReport beamLow = refuseBefore(parked(11.0), leftRequest());
    REQUIRE(beamLow.verdict == WallResetVerdict::BeamLow);
    CHECK(line(beamLow) == std::string{kGuideLines[4]});

    const loc::WallResetReport bad = refuseBefore(parked(7.0), leftRequest(0.0));
    REQUIRE(bad.verdict == WallResetVerdict::BadRequest);
    CHECK(line(bad, leftRequest(0.0)) == std::string{kGuideLines[5]});
}

// Bug caught (the L6-A1 review): a refusal line's TAIL unpinned — a ReadFault line that ended "faults
// ok" (contradicting the verdict it reports), a NoReturn / LowConfidence / OutOfRange line that called
// a sensor with no read-fault count "ok". Every other verdict's line, exactly: "faults moved" on a
// ReadFault, "faults ok" / "faults n/a" on the per-sample refusals, and the request's name leading.
TEST_CASE("L6 T9 every verdict's line, byte for byte; the faults tail per verdict") {
    const loc::WallResetRequest rq = leftRequest();
    const Pose2d pose = parked(7.0);
    // The per-sample refusals, from the evaluator.
    std::vector<loc::WallSample> s = window({7.0, 7.0, 7.0, 7.0, 7.0});
    s[3].faultMoved = true;
    loc::WallResetReport rf = judge(s, pose, rq);
    REQUIRE(rf.verdict == WallResetVerdict::ReadFault);
    CHECK(line(rf) == "left wall x REFUSED read fault (LEFT sensor): sample 3 read while the port faulted | faults moved");

    s = window({7.0, 7.0, 7.0, 7.0, 7.0});
    s[2].confidence = 0.0;
    s[2].distance = Length{393.66};
    loc::WallResetReport nr = judge(s, pose, rq);
    REQUIRE(nr.verdict == WallResetVerdict::NoReturn);
    CHECK(line(nr)
          == "left wall x REFUSED no return (LEFT sensor): sample 2 confidence 0.00 (limit 0.50, distance 393.66 in) "
             "| faults ok");
    nr.faultChannel = false;
    CHECK(line(nr)
          == "left wall x REFUSED no return (LEFT sensor): sample 2 confidence 0.00 (limit 0.50, distance 393.66 in) "
             "| faults n/a");

    s = window({7.0, 7.0, 7.0, 7.0, 7.0});
    s[1].confidence = 0.375;
    const loc::WallResetReport lc = judge(s, pose, rq);
    REQUIRE(lc.verdict == WallResetVerdict::LowConfidence);
    CHECK(line(lc)
          == "left wall x REFUSED low confidence (LEFT sensor): sample 1 confidence 0.38 (limit 0.50, distance 7.00 "
             "in) | faults ok");

    s = window({7.0, 7.0, 7.0, 1.5, 7.0});
    loc::WallResetReport oor = judge(s, pose, rq);
    REQUIRE(oor.verdict == WallResetVerdict::OutOfRange);
    CHECK(line(oor)
          == "left wall x REFUSED out of range (LEFT sensor): sample 3 read 1.50 in (window 2.00..16.00) | faults ok");
    oor.faultChannel = false;
    CHECK(line(oor)
          == "left wall x REFUSED out of range (LEFT sensor): sample 3 read 1.50 in (window 2.00..16.00) | faults n/a");

    const loc::WallResetReport moving = judge(window({7.0, 7.0, 7.0, 7.0, 7.0}), pose, rq, {},
                                              Twist2d{Velocity{1.5}, Velocity{0.0}, AngularVelocity{0.0}});
    REQUIRE(moving.verdict == WallResetVerdict::Moving);
    CHECK(line(moving)
          == "left wall x REFUSED moving (LEFT sensor): speed 1.50 in/s, turn 0.0 deg/s (limit 1.00 in/s, 3.0 deg/s)");

    const loc::WallResetReport few = judge(window({7.0, 7.0, 7.0}), pose, rq);
    REQUIRE(few.verdict == WallResetVerdict::TooFewSamples);
    CHECK(line(few) == "left wall x REFUSED too few samples (LEFT sensor): n 3 over 0.02 s (need 5 over 0.15 s)");

    const loc::WallResetReport spread = judge(window({7.0, 7.25, 7.75, 7.0, 7.5, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0,
                                                      7.0, 7.0, 7.0}),
                                              pose, rq);
    REQUIRE(spread.verdict == WallResetVerdict::Spread);
    CHECK(line(spread)
          == "left wall x REFUSED spread (LEFT sensor): spread 0.75 in (limit 0.50) | median 7.00 in spread 0.75 n 16");

    loc::WallResetLimits tight{};
    tight.errorBoundMax = Length{0.5};
    const loc::WallResetReport imprecise = judge(window({7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0,
                                                         7.0, 7.0, 7.0, 7.0}),
                                                 pose, rq, tight);
    REQUIRE(imprecise.verdict == WallResetVerdict::Imprecise);
    CHECK(line(imprecise, rq, tight)
          == "left wall x REFUSED imprecise (LEFT sensor): bound 0.87 in (limit 0.50; heading tolerance 2.0 deg) | "
             "median 7.00 in spread 0.00 n 16");

    // The call-only verdicts (hand-built reports: no pure function reaches them).
    loc::WallResetReport callOnly{};
    callOnly.axis = loc::Axis::X;
    callOnly.side = loc::WallSide::Left;
    callOnly.verdict = WallResetVerdict::SensorAbsent;
    callOnly.detail = "port 4: no device";
    CHECK(line(callOnly) == "left wall x REFUSED sensor absent (LEFT sensor): port 4: no device");
    callOnly.detail = "";
    callOnly.verdict = WallResetVerdict::MountUnset;
    CHECK(line(callOnly) == "left wall x REFUSED mount unset (LEFT sensor): measure the mount and type it with measured = true");
    callOnly.verdict = WallResetVerdict::NotLive;
    CHECK(line(callOnly) == "left wall x REFUSED not live (LEFT sensor): the estimate is not live yet");
    callOnly.verdict = WallResetVerdict::MotionActive;
    CHECK(line(callOnly) == "left wall x REFUSED motion active (LEFT sensor): a motion is still armed");
    callOnly.verdict = WallResetVerdict::NoSensor;
    CHECK(line(callOnly) == "left wall x REFUSED no sensor (LEFT sensor): this robot declares no wall sensors");
    callOnly.verdict = WallResetVerdict::NotFieldStart;
    // L6-A2 (finding 23 of the L6-A1 review): the line names the start mode, so RELATIVE and UNSET read
    // apart; a hand-built report with no mode still says what is wrong.
    callOnly.detail = "RELATIVE";
    CHECK(line(callOnly) == "left wall x REFUSED not a field start (LEFT sensor): start RELATIVE -- the season's "
                            "walls are FIELD coordinates: declare a FIELD start");
    callOnly.detail = "UNSET";
    CHECK(line(callOnly) == "left wall x REFUSED not a field start (LEFT sensor): start UNSET -- the season's "
                            "walls are FIELD coordinates: declare a FIELD start");
    callOnly.detail = "";
    CHECK(line(callOnly) == "left wall x REFUSED not a field start (LEFT sensor): start not FIELD -- the "
                            "season's walls are FIELD coordinates: declare a FIELD start");
    callOnly.verdict = WallResetVerdict::Cut;
    // L6-A2 (a review finding of 2026-11-05): with samples taken the cut landed IN the window; with none
    // the run had already ended when the window was to begin — the call came too late. The two forms
    // split between 0 and 1 sample, so each side of that edge is checked ON it (the fresh verifier's
    // V11, 2026-11-10: `samples > 1` survived when this side sat at 7).
    callOnly.samples = 1;
    CHECK(line(callOnly) == "left wall x REFUSED cut (LEFT sensor): the run's deadline or a disable landed in the window; nothing changed");
    callOnly.samples = 0;
    CHECK(line(callOnly) == "left wall x REFUSED cut (LEFT sensor): the run's deadline or a disable came before the window began; nothing changed");

    // A named request leads its line; the right sensor's word.
    loc::WallResetRequest named = rq;
    named.name = "after the Loader";
    named.sensor = loc::WallSide::Right;
    loc::WallResetReport mu = callOnly;
    mu.verdict = WallResetVerdict::MountUnset;
    mu.side = loc::WallSide::Right;
    CHECK(line(mu, named)
          == "after the Loader: left wall x REFUSED mount unset (RIGHT sensor): measure the mount and type it with "
             "measured = true");
}

// Bug caught (the L6-A1 review): guide 11's WALL examples drifting from the code with nothing turning
// red (they were copied once from a scratch probe). The chapter's `WALL` section must quote EXACTLY
// the six lines above, in order, each after a terminal `[t=...]` prefix — so a change to a line's
// format changes this file, and this case then fails until the guide is re-quoted from it.
TEST_CASE("L6 T9 guide 11 quotes exactly the six pinned WALL lines (and MIRROR1-A's mirrored seventh)") {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/docs/guide/11-reading-the-diagnostics.md"};
    REQUIRE(in.good());
    std::stringstream buf;
    buf << in.rdbuf();
    const std::string text = buf.str();
    const std::size_t from = text.find("## The wall reset (WALL)");
    const std::size_t to = text.find("\n## ", from + 1);
    REQUIRE(from != std::string::npos);
    REQUIRE(to != std::string::npos);
    std::vector<std::string> quoted;
    std::istringstream lines{text.substr(from, to - from)};
    for (std::string l; std::getline(lines, l);) {
        const std::size_t tag = l.find("[WALL] ");
        if (l.rfind("[t=", 0) == 0 && tag != std::string::npos) {
            quoted.push_back(l.substr(tag + 7));
        }
    }
    REQUIRE(quoted.size() == 7U);
    for (std::size_t i = 0; i < 6U; ++i) {
        INFO("guide line " << i);
        CHECK(quoted[i] == std::string{kGuideLines[i]});
    }
    CHECK(quoted[6] == std::string{kGuideMirroredLine});
}

// MIRROR1-A T8 (the MIRROR1 brief's D8, R8): a MIRRORED auton's WALL line. Bug caught: the note dropped,
// put where the terminal's 200-byte message cut can reach it (after the numbers), or written into an
// UNMIRRORED line (every line above stays byte for byte as L6 pinned it); a note that breaks the
// parenthesis. The line, byte for byte; and at its WORST case — a 48-character request name, a
// 32-character wall name (both the sanitizer's caps), the longest verdict word, robot one's 32-character
// wall named in the note — the note and its closing parenthesis still sit inside the first 200 bytes.
TEST_CASE("MIRROR1 T8 the mirrored WALL line: the note inside the sensor's parenthesis, inside the 200-byte cut") {
    const loc::WallResetReport seated = judge(sixteen(6.99), parked(9.4), leftRequest());
    REQUIRE(seated.verdict == WallResetVerdict::Seated);
    CHECK(line(seated, leftRequest(), {}, "mirrored from blue wall RIGHT") == std::string{kGuideMirroredLine});
    CHECK(line(seated, leftRequest(), {}, "") == std::string{kGuideLines[0]});       // an empty note: no note
    CHECK(line(seated, leftRequest(), {}, nullptr) == std::string{kGuideLines[0]});  // the default: no note
    // the worst case: every name at its cap, every verdict word
    loc::WallResetRequest rq = leftRequest();
    rq.name = "nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn";       // 48
    rq.wall.name = "wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww";                    // 32
    const std::string note = "mirrored from wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww RIGHT";  // the everyday form's
    std::size_t worst = 0;
    for (int v = 0; v <= static_cast<int>(WallResetVerdict::Cut); ++v) {
        loc::WallResetReport rep = seated;
        rep.verdict = static_cast<WallResetVerdict>(v);
        rep.detail = "";
        const std::string l = line(rep, rq, {}, note.c_str());
        const std::size_t at = l.find("(LEFT sensor, " + note + "): ");
        CAPTURE(l);
        REQUIRE(at != std::string::npos);
        const std::size_t end = at + std::string{"(LEFT sensor, "}.size() + note.size() + 1;  // through ')'
        worst = std::max(worst, end);
        CHECK(end <= 200U);
        for (const char c : l) {
            CHECK(static_cast<unsigned char>(c) < 0x80);
        }
    }
    MESSAGE("the mirrored note's closing parenthesis at its worst: byte " << worst << " of the 200 the terminal keeps");
}
