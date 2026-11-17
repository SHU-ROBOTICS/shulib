#pragma once
//
// mirrored_pair_check.hpp — THE PAIR CHECK (chunk MIRROR1-A, 2026-11-13; the MIRROR1 brief's D11 (a) 2,
// T9, T11; A4 register HA-260). Everything here is the host test plant; nothing here has run on a robot.
//
// ── Why it exists ────────────────────────────────────────────────────────────────────────────────
// In a VEX U skills run BOTH robots run at once (<VURS2>). A mirrored route's robot two is robot one's
// reflection at the SAME moment, and a reflection leaves its line where it is: wherever robot one's robot
// reaches the diagonal, robot two's reaches the same spot. So the suite runs every mirrored entry robot
// two registers TOGETHER with its unmirrored self — robot one's run (the entry as written, from its own
// start) and robot two's (the entry as registered, from the reflected start), each through
// competition::runAuton on its own host test plant — and compares them PACE BY PACE: it FAILS, naming
// the moment, both positions and the motion each robot was in (robot one's with its target, in the
// route's own numbers), when their 15 in squares come closer than kMirrorGapIn (6 in). In a pure mirror the gap is exactly twice robot one's clearance from the line, so
// this IS the guide's rule "keep robot one's whole robot 3 in off the diagonal"; after an isMirrored(ctx)
// branch the two runs differ in time, and the check holds them apart at the PLANNED timing only — a real
// robot's timing drifts (host test plant, a 9 % weak side: 0.44 s over 21 s, the brief's P1 B).
//
// ── One function, shared ─────────────────────────────────────────────────────────────────────────
// checkMirroredPair() returns a VERDICT; it never asserts. The suite asserts it for robot two's registry
// (test/autons_test.cpp, every mirrored entry); test/mirrored_pair_test.cpp asserts it on planted routes
// (a planted crash is a PASSING test of the check); a route preview (RPV1's follow-up) or a route maker
// can call the same function so the picture and the suite never disagree. A RELATIVE mirrored entry is
// SKIPPED, by name: its field positions are wherever the drivers place the robot.
//
// ── The robots ───────────────────────────────────────────────────────────────────────────────────
// Each run is the program's chain — Chassis -> RunGuard -> MechanismService (robot two's REAL list, on
// fakes) -> the recording world — with the route registry, on the drivetrain the caller names (robot
// two's stand-in: a 12 in tank) and an optional weak side (both robots the SAME build, so both get it).
// The square is the robots' 15 in footprint centred on the tracking centre, turned with the heading —
// RPV1's kPreviewFootprintIn ("both robots are 15 in", rpv1-a/test/route_preview_model.hpp, not merged at
// this writing): the second of RPV1-A and MIRROR1-A to merge makes the two one constant. That the real
// robot fits inside a 15 in square CENTRED on its tracking centre is a belief (A4 register HA-260): a
// footprint sitting d in ahead of or behind the tracking centre still mirrors, but the check then
// misjudges each robot's clearance by up to d — out of the 3 in a side.
//
// ── Naming the step ──────────────────────────────────────────────────────────────────────────────
// The Routine exposes no step index to an outside observer (and F10, the Routine, is untouched), so the
// check names the scheduler's DRIVE MOTION: "motion 8 (CurveTo ending at (12.0, -18.0))". The number
// counts drive motions only — each driveTo, reverseTo, face, faceAway, turnTo, moveTo, strafeTo, curve,
// hold, brake and each followTrajectory LEG is one; a then() or a Toggle step counts the drives it runs;
// a wall reset, a pause, a waitFor or a mechanism step is none — so it is NOT the "step N" the Routine's
// own lines print. Robot one's TARGET (from the motion's own records: the point a drive goes to, a
// curve's end, a turn's heading) is in the route's own numbers, so a student finds the line by it.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "auton_tester_rig.hpp"
#include "motion_test_rig.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/kinematics.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/mechanism_routes.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

namespace mirror_pair {

/// The least gap the two robots' squares may come to (inches): 3 in a side of the diagonal. A named
/// constant the team may change. PROVISIONAL (A4: HA-260) — a chosen margin, unmeasured.
inline constexpr double kMirrorGapIn = 6.0;
/// The robots' square footprint (inches): both are 15 in — held equal to route_preview::kPreviewFootprintIn by a
/// static_assert in mirrored_pair_test.cpp.
inline constexpr double kRobotSquareIn = 15.0;

/// One pace of one robot's run.
struct Pace {
    double t = 0.0;              ///< seconds since the run's guard armed
    shulib::math::Pose2d truth;  ///< the plant's TRUE pose after the pace
    std::uint32_t motion = 0;    ///< the scheduler's active command id (0: between motions)
    std::uint32_t lastMotion = 0;  ///< the last motion that ENDED (its id), for "between motions"
    bool hasTarget = false;            ///< `target` read from the active motion's own latest record
    shulib::math::Pose2d target{};     ///< that record's targetPose (the field the run drives in)
};

/// One robot's run.
struct Track {
    std::vector<Pace> paces;
    std::vector<std::pair<std::uint32_t, std::string>> names;  ///< motion id -> its name (at its end)
    shulib::competition::AutonRunResult result{};
    std::vector<std::string> lines;  ///< every log line, "[SUB] message"

    [[nodiscard]] std::string nameOf(std::uint32_t id) const {
        for (const auto& n : names) {
            if (n.first == id) {
                return n.second;
            }
        }
        return "?";
    }
    /// "motion 8 (CurveTo)" — with `withTarget`, the motion's target too: "motion 8 (CurveTo ending at
    /// (12.0, -18.0))", "motion 3 (MoveToPose to (-12.0, -40.0))", "motion 2 (TurnTo to 90.0 deg)" (a hold
    /// or a brake has none) — or "between motions, after motion 7 (MoveToPose)" (file banner: naming the
    /// step).
    [[nodiscard]] std::string stepAt(std::size_t i, bool withTarget = false) const {
        const Pace& p = paces[std::min(i, paces.size() - 1)];
        char out[160];
        if (i >= paces.size()) {
            std::snprintf(out, sizeof out, "its run over (stopped)");
        } else if (p.motion != 0) {
            const std::string name = nameOf(p.motion);
            char target[64] = "";
            if (withTarget && p.hasTarget) {
                const double x = p.target.x().value();
                const double y = p.target.y().value();
                if (name == "TurnTo") {
                    std::snprintf(target, sizeof target, " to %.1f deg", p.target.heading().degrees());
                } else if (name == "CurveTo" || name == "ReverseCurveTo") {
                    std::snprintf(target, sizeof target, " ending at (%.1f, %.1f)", x, y);
                } else if (name == "MoveToPose" || name == "StrafeTo") {
                    std::snprintf(target, sizeof target, " to (%.1f, %.1f)", x, y);
                }
            }
            std::snprintf(out, sizeof out, "motion %u (%s%s)", static_cast<unsigned>(p.motion), name.c_str(), target);
        } else if (p.lastMotion != 0) {
            std::snprintf(out, sizeof out, "between motions, after motion %u (%s)", static_cast<unsigned>(p.lastMotion),
                          nameOf(p.lastMotion).c_str());
        } else {
            std::snprintf(out, sizeof out, "before its first motion");
        }
        return out;
    }
};

/// The program's chain under one robot (file banner), recording every pace into a Track.
struct Robot {
    struct World final : shulib::motion::ITickPacer {
        Robot* r = nullptr;
        long paces = 0;
        void pace() override {
            if (++paces > 400000) {
                throw std::runtime_error("mirror_pair: pace cap exceeded -- a wait failed to end");
            }
            r->rig.h.plant().step(shulib::units::Time{0.01});
            if (r->armedAt < 0.0) {
                return;  // the idle ticks before the run
            }
            Pace p;
            p.t = r->rig.h.clock().now().value() - r->armedAt;
            p.truth = r->rig.h.truePose();
            p.motion = r->chassis.scheduler().activeCommandId();
            // the active motion's own latest LIVE record (file banner: naming the step) — its target (a
            // record from before its first live tick may carry none). Other emitters' records (id 0)
            // interleave with the motion's, so the newest few are searched.
            if (p.motion != 0) {
                const int n = r->sink.recordCount();
                for (int k = n - 1; k >= 0 && k >= n - 8; --k) {
                    const shulib::diag::DebugRecord& rec = r->sink.recordAt(k);
                    if (rec.activeCommandId == p.motion
                        && rec.activeCommandState == static_cast<std::uint8_t>(shulib::motion::MotionState::Running)) {
                        p.hasTarget = true;
                        p.target = rec.targetPose;
                        break;
                    }
                }
            }
            const shulib::motion::CompletedMotion& c = r->chassis.lastCompleted();
            p.lastMotion = c.id;
            if (c.id != 0 && (r->track.names.empty() || r->track.names.back().first != c.id)) {
                r->track.names.emplace_back(c.id, c.name);
            }
            r->track.paces.push_back(p);
        }
    };
    static shulib::sim::SimHarnessConfig config(const shulib::math::Pose2d& start) {
        shulib::sim::SimHarnessConfig c = motion_rig::plantConfig();
        c.plant.initialPose = start;
        return c;
    }

    tester_rig::WeakSide weak;
    shulib::hal::fake::FakeTelemetrySink sink;
    motion_rig::MotionRig rig;
    World world;
    tester_rig::RobotTwoFakes two;
    shulib::mechanism::MechanismService svc;
    shulib::sequence::RunGuard guard;
    shulib::chassis::Chassis chassis;
    shulib::runner::MechanismRoutes routes;
    shulib::runner::CommandRegistry commands;
    Track track;
    double armedAt = -1.0;

    Robot(const shulib::kinematics::IKinematics& kin, const shulib::math::Pose2d& trueStart, double weakScale)
        : rig{kin, config(trueStart), &sink, &weak},
          svc{world},
          guard{svc},
          chassis{rig.deps, guard, motion_rig::chassisConfig()},
          routes{chassis, svc, shulib::bench::robot_two::kRobotTwoMechanisms, sink} {
        weak.wheel = 1;  // the right side (tester_rig::WeakSide)
        weak.scale = weakScale;
        world.r = this;
        svc.bind(shulib::bench::robot_two::kRobotTwoMechanisms, two.devs, rig.h.clock(), rig.latch, sink);
        routes.registerInto(commands);
        for (int i = 0; i < 30; ++i) {
            chassis.wait(shulib::units::Time{0.01});  // the estimate live, nothing commanded
        }
    }

    /// Run `entry` once through competition::runAuton; the track starts the pace the guard arms.
    Track& run(const shulib::competition::AutonEntry& entry, shulib::units::Time period) {
        armedAt = rig.h.clock().now().value();  // runAuton arms its guard on its first instruction
        track.result = shulib::competition::runAuton(chassis, guard, entry, &commands, sink, svc, period);
        for (int i = 0; i < sink.size(); ++i) {
            track.lines.push_back("[" + sink.at(i).subsystem + "] " + sink.at(i).message);
        }
        return track;
    }
};

// ── the squares ──────────────────────────────────────────────────────────────────────────────────

struct V2 {
    double x = 0.0;
    double y = 0.0;
};

/// The 15 in square of a robot at `p` (centre, heading), corners counterclockwise.
inline void square(const shulib::math::Pose2d& p, double side, V2 out[4]) {
    const double h = side / 2.0;
    const double c = std::cos(p.heading().radians());
    const double s = std::sin(p.heading().radians());
    const double xs[4] = {h, -h, -h, h};
    const double ys[4] = {h, h, -h, -h};
    for (int i = 0; i < 4; ++i) {
        out[i] = V2{p.x().value() + c * xs[i] - s * ys[i], p.y().value() + s * xs[i] + c * ys[i]};
    }
}
inline double segDist(V2 p, V2 a, V2 b) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double wx = p.x - a.x;
    const double wy = p.y - a.y;
    double t = (vx * wx + vy * wy) / (vx * vx + vy * vy);
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    return std::hypot(wx - t * vx, wy - t * vy);
}
/// Do two convex quads overlap (or touch)? The separating-axis test over both squares' edge normals.
inline bool overlap(const V2 a[4], const V2 b[4]) {
    for (int k = 0; k < 2; ++k) {
        const V2* q = k == 0 ? a : b;
        for (int i = 0; i < 2; ++i) {
            const double nx = -(q[i + 1].y - q[i].y);
            const double ny = q[i + 1].x - q[i].x;
            double amin = 1e300;
            double amax = -1e300;
            double bmin = 1e300;
            double bmax = -1e300;
            for (int j = 0; j < 4; ++j) {
                const double da = a[j].x * nx + a[j].y * ny;
                const double db = b[j].x * nx + b[j].y * ny;
                amin = std::min(amin, da);
                amax = std::max(amax, da);
                bmin = std::min(bmin, db);
                bmax = std::max(bmax, db);
            }
            if (amax < bmin || bmax < amin) {
                return false;
            }
        }
    }
    return true;
}
/// The gap between the two robots' squares (inches): 0 when they overlap or touch.
inline double squareGap(const shulib::math::Pose2d& p, const shulib::math::Pose2d& q, double side = kRobotSquareIn) {
    V2 a[4];
    V2 b[4];
    square(p, side, a);
    square(q, side, b);
    if (overlap(a, b)) {
        return 0.0;
    }
    double d = 1e300;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            d = std::min(d, segDist(a[i], b[j], b[(j + 1) % 4]));
            d = std::min(d, segDist(b[i], a[j], a[(j + 1) % 4]));
        }
    }
    return d;
}

// ── the verdict ──────────────────────────────────────────────────────────────────────────────────

/// What the pair check found. Never an assertion: the caller decides.
struct PairVerdict {
    bool checked = false;     ///< false: SKIPPED (a RELATIVE mirrored entry), `sentence` says why
    bool ok = true;           ///< checked and never under the gap
    double leastGapIn = 1e300;  ///< the least gap over the whole run (inches)
    double leastGapAt = 0.0;    ///< when (seconds since the guard armed)
    std::size_t paces = 0;      ///< paces compared
    // the FIRST pace under the gap, when not ok
    double failAt = 0.0;
    double failGapIn = 0.0;
    shulib::math::Pose2d one{};
    shulib::math::Pose2d two{};
    std::string stepOne;
    std::string stepTwo;
    std::string sentence;  ///< the line a person reads (the failure, the skip, or the pass)
};

/// Compare two tracks pace by pace (file banner). A robot whose run is over stays where it stopped.
inline PairVerdict comparePair(const char* name, const Track& one, const Track& two, double gapIn = kMirrorGapIn,
                               double side = kRobotSquareIn) {
    PairVerdict v;
    v.checked = true;
    const std::size_t n = std::max(one.paces.size(), two.paces.size());
    v.paces = n;
    for (std::size_t i = 0; i < n && !one.paces.empty() && !two.paces.empty(); ++i) {
        const Pace& a = one.paces[std::min(i, one.paces.size() - 1)];
        const Pace& b = two.paces[std::min(i, two.paces.size() - 1)];
        const double g = squareGap(a.truth, b.truth, side);
        const double t = i < one.paces.size() ? a.t : b.t;
        if (g < v.leastGapIn) {
            v.leastGapIn = g;
            v.leastGapAt = t;
        }
        if (v.ok && g < gapIn) {
            v.ok = false;
            v.failAt = t;
            v.failGapIn = g;
            v.one = a.truth;
            v.two = b.truth;
            v.stepOne = one.stepAt(i, /*withTarget=*/true);  // robot one's target: the route's numbers
            v.stepTwo = two.stepAt(i);
        }
    }
    char line[512];
    if (!v.ok) {
        std::snprintf(line, sizeof line,
                      "auton '%s': the two robots come within %.1f in (%.2f in apart) at %.2f s -- robot one at "
                      "(%.1f, %.1f) in %s, robot two at (%.1f, %.1f) in %s. Keep robot one's whole robot %.1f in off "
                      "the mirror line, or send ONE robot there with isMirrored(ctx).",
                      name, gapIn, v.failGapIn, v.failAt, v.one.x().value(), v.one.y().value(), v.stepOne.c_str(),
                      v.two.x().value(), v.two.y().value(), v.stepTwo.c_str(), gapIn / 2.0);
    } else {
        std::snprintf(line, sizeof line, "auton '%s': the two robots stay at least %.2f in apart (least at %.2f s)",
                      name, v.leastGapIn, v.leastGapAt);
    }
    v.sentence = line;
    return v;
}

/// The options of one pair check.
struct PairOptions {
    const shulib::kinematics::IKinematics* kin = nullptr;  ///< both robots' drivetrain (the same build)
    double weakScale = 1.0;                                ///< a weak right side on BOTH (1.0: none)
    shulib::units::Time period{60.0};                      ///< the period the entry runs in
    double gapIn = kMirrorGapIn;
};

/// Robot one's and robot two's runs of the MIRRORED `entry`, compared (file banner). Robot one runs the
/// entry as written (mirrored() toggled back) from its own start; robot two runs `entry` from the
/// reflected start (competition::seatedStart). A RELATIVE (or unmirrored) entry: SKIPPED, by name.
/// `tracks` (optional) receives both runs.
inline PairVerdict checkMirroredPair(const shulib::competition::AutonEntry& entry, const PairOptions& o,
                                     std::pair<Track, Track>* tracks = nullptr) {
    PairVerdict v;
    const char* name = entry.name != nullptr ? entry.name : "(unnamed)";
    char line[256];
    if (entry.mirror == nullptr) {
        std::snprintf(line, sizeof line, "auton '%s': not a mirrored entry -- no pair to check", name);
        v.sentence = line;
        return v;
    }
    if (entry.effectiveStart().mode() != shulib::competition::StartPose::Mode::Field) {
        std::snprintf(line, sizeof line,
                      "auton '%s': a RELATIVE mirrored entry -- its field positions are wherever the drivers place "
                      "the robots, so the pair check is SKIPPED",
                      name);
        v.sentence = line;
        return v;
    }
    const shulib::competition::AutonEntry asWritten = shulib::competition::mirrored(entry, *entry.mirror);
    auto one = std::make_unique<Robot>(*o.kin, asWritten.effectiveStart().pose(), o.weakScale);
    auto two = std::make_unique<Robot>(*o.kin, shulib::competition::seatedStart(entry), o.weakScale);
    const Track& a = one->run(asWritten, o.period);
    const Track& b = two->run(entry, o.period);
    v = comparePair(name, a, b, o.gapIn);
    if (tracks != nullptr) {
        *tracks = {a, b};
    }
    return v;
}

/// Every MIRRORED entry of `reg` through checkMirroredPair (T9) — one verdict each, in the registry's
/// order; the unmirrored entries are left out.
inline std::vector<PairVerdict> checkEveryMirroredEntry(const shulib::competition::AutonRegistry& reg,
                                                        const shulib::kinematics::IKinematics& kin) {
    std::vector<PairVerdict> out;
    for (std::size_t i = 0; i < reg.size(); ++i) {
        const shulib::competition::AutonEntry& e = reg.at(i);
        if (e.mirror == nullptr) {
            continue;
        }
        PairOptions o;
        o.kin = &kin;
        o.period = reg.periodFor(e);
        out.push_back(checkMirroredPair(e, o));
    }
    return out;
}

}  // namespace mirror_pair
