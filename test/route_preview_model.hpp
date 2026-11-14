#pragma once
//
// route_preview_model.hpp — what the ROUTE PREVIEW knows about one run, and THE FIELD CHECK (chunk
// RPV1-A, 2026-11-12; the RPV1 brief's §1.1). Host only (allocates). Say "host test plant" (the
// project's one name for it).
//
// The preview (test/route_preview_run.hpp) runs one auton through the Auton Tester's own sequence on
// the host test plant and folds what it recorded into a Model; the renderer (test/
// route_preview_render.hpp) draws the Model as the page and the stdout table. This header holds the
// Model and the one piece of geometry the preview judges with — the robot's OUTLINE against the field
// — so the renderer depends on neither the rig nor the library's run.
//
// ── The step numbers count from BELOW (the brief's D2 (a)) ──────────────────────────────────────
// Today the library says nothing when a step succeeds. From outside, the preview sees every motion
// (the scheduler's boundary observer: drives, curves, strafes, turns, brakes and holds) and every mechanism step
// (a slot's step count rising, or the MECH line of a step on a mechanism the robot does not have), but
// not a pause, a wait (waitFor, waitUntilDone), a then(...) action or a startAt — each is a step of the
// Routine that starts no motion and raises no count. So the Model numbers the motions and mechanism
// steps in the order they began, and quotes the Routine's own RTN lines word for word — those carry
// the code's REAL step numbers. The page says so at the top.
//
// ── THE FIELD CHECK (the brief's R1, D6, D12 (a)) ─────────────────────────────────────────────────
// The host test plant has no walls: a 200 in drive goes straight through one. So the preview checks
// the robot's OUTLINE — a 15 in square (kPreviewFootprintIn) centred on the point the plant tracks,
// turned with its heading — on every sample against the field the host test plant's field model
// draws (sim::FieldModel::override2026(): the plant's OWN copy of the manual's numbers):
//   * THROUGH — a corner more than kThroughAllowanceIn (1 in) past a wall's inner face, or the square
//     more than that deep in a LOADER (solid, standing on the floor: the robot cannot pass it): the
//     first such sample is the run's THROUGH; everything after it is drawn faded; the command fails.
//   * TOUCH — any overlap within the allowance with a wall's face or a Loader, or any overlap with a
//     Toggle's body or brackets: an amber note per element per contact. Squaring on a wall and
//     driving up to a Loader are normal, so a TOUCH never fails anything.
//   * a START PIECE — a grey note per first contact (the plant does not push pieces).
// The square is a drawing assumption, said on every page: the real frame may be smaller, its tracked
// point may not be its middle, and a claw or lift that reaches past it is not checked.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "shulib/sim/field_plant.hpp"

namespace route_preview {

/// The robot's outline for the field check: a square this many inches a side (D12 (a): the team
/// lead's 2026-10-08 word, both robots are 15 in).
inline constexpr double kPreviewFootprintIn = 15.0;
/// How far past a wall's face (or into a Loader) the outline may go before it counts as THROUGH.
inline constexpr double kThroughAllowanceIn = 1.0;
/// More log lines than this with no pace between them: the auton loops without letting time pass
/// (a real 60 s run logs about 40; the brief's R3).
inline constexpr std::size_t kNoPaceLineLimit = 10000;
/// How many of the looping lines the page keeps.
inline constexpr std::size_t kLoopLinesKept = 200;

/// How a thing is drawn: its colour on the page and its word in the table.
enum class Mark : std::uint8_t {
    Ok,           ///< green: succeeded
    Unconfirmed,  ///< teal: counted done, but nothing confirmed it (UNCONFIRMED)
    Failed,       ///< red: failed, refused, timed out — the library's sentence beside it
    Cut,          ///< amber: cut short (by the deadline, a throw, or a newer command)
    Amber,        ///< amber: refused but the route went on (a wall reset), or a TOUCH
    Grey,         ///< grey: never ran (skipped), or no verdict
};

/// The mark's word ("ok", "FAILED", ...), for the table and the CSS class.
[[nodiscard]] inline const char* markWord(Mark m) noexcept {
    switch (m) {
        case Mark::Ok: return "ok";
        case Mark::Unconfirmed: return "unconfirmed";
        case Mark::Failed: return "failed";
        case Mark::Cut: return "cut";
        case Mark::Amber: return "amber";
        case Mark::Grey: return "grey";
    }
    return "grey";
}

/// One recorded pace: the plant's TRUE pose (never the estimate) and the estimate beside it, at T+
/// seconds from the auton's arming.
struct Sample {
    double t = 0.0;      ///< T+ seconds (clock − armedAt)
    double x = 0.0;      ///< true field x, in
    double y = 0.0;      ///< true field y, in
    double theta = 0.0;  ///< true heading, rad (unwrapped)
    double ex = 0.0;     ///< the estimate's x, in (the robot's own belief — not drawn as the path)
    double ey = 0.0;     ///< the estimate's y, in
};

/// What kind of step, from below.
enum class StepKind : std::uint8_t { Drive, Turn, Curve, Brake, Hold, Strafe, Mechanism };

/// The kind as the page's word.
[[nodiscard]] inline const char* stepKindWord(StepKind k) noexcept {
    switch (k) {
        case StepKind::Drive: return "drive";
        case StepKind::Turn: return "turn";
        case StepKind::Curve: return "curve";
        case StepKind::Brake: return "brake";
        case StepKind::Hold: return "hold";
        case StepKind::Strafe: return "strafe";
        case StepKind::Mechanism: return "mechanism";
    }
    return "step";
}

/// One step seen from below (header): a motion, or one mechanism step.
struct Step {
    int number = 0;                       ///< the from-below count (1-based); 0 for the end action's
    StepKind kind = StepKind::Drive;      ///< what it is
    std::string what;                     ///< the motion's name ("MoveToPose") or the mechanism's
    double t0 = 0.0;                      ///< T+ when it began
    double t1 = 0.0;                      ///< T+ when it ended (its verdict arrived)
    std::size_t s0 = 0;                   ///< the sample it began at
    std::size_t s1 = 0;                   ///< the sample it ended at
    bool hasTarget = false;               ///< a motion that published a target
    double tx = 0.0;                      ///< the target (field in, rad), where a drive was AIMED
    double ty = 0.0;                      ///< (target y)
    double tth = 0.0;                     ///< (target heading)
    std::string verdict;                  ///< "SETTLED", "TIMED_OUT", "SUCCEEDED", ...
    Mark mark = Mark::Grey;               ///< how it is drawn
    std::string sentence;                 ///< the library's own line (a MECH verdict), verbatim
    bool endAction = false;               ///< ran in the end action (the park), not the routine
    bool background = false;              ///< a mechanism step the robot drove during (its own span)
    bool openAtThrow = false;             ///< the step the run threw INSIDE (ended by the unwind; route_preview_run.hpp)
    std::string placeholder;              ///< the PLACEHOLDER note, or ""
};

/// One library log line the page quotes, dated to the pace it was logged at.
struct LogLine {
    double t = 0.0;          ///< T+ of the sample it dates to
    std::size_t s = 0;       ///< that sample
    std::string subsystem;   ///< "RTN", "MECH", "WALL", "COMP", ...
    std::string text;        ///< verbatim
    Mark mark = Mark::Grey;  ///< how it is drawn
};

/// What a field contact is.
enum class Contact : std::uint8_t { Through, Touch, Piece };

/// One field contact (header: THE FIELD CHECK).
struct FieldNote {
    Contact kind = Contact::Touch;  ///< THROUGH, TOUCH or a start piece
    std::string element;            ///< "left wall", "Loader", "Toggle bracket", "start Cup", ...
    double t = 0.0;                 ///< T+ of the first sample of the contact
    std::size_t s = 0;              ///< that sample
    int step = 0;                   ///< the from-below step open then (0: between steps)
    double depth = 0.0;             ///< how far in, in
};

/// Everything the page and the table are drawn from (header).
struct Model {
    // ── what was asked ──
    std::string auton;          ///< the auton's name, as registered
    std::string robotKey;       ///< "two", "one", "x"
    std::string robotLine;      ///< which robot, which drive, what is pretend (the page's first line)
    std::string kindWord;       ///< "SKILLS" / "MATCH"
    double period = 0.0;        ///< the period it runs in, s
    double park = 0.0;          ///< endActionAt, s
    double stop = 0.0;          ///< hardStopAt, s
    std::string startLine;      ///< where it started, in words
    bool placedAtCentre = false;  ///< a RELATIVE start with no --place
    std::string stamp;          ///< the commit, or "" (not stamped)
    std::string autonsHash;     ///< the autons file's short hash, or ""
    std::vector<std::string> notices;  ///< red/amber lines at the top
    // ── what happened ──
    bool refused = false;       ///< nothing ran: the registry or runAuton refused it
    std::string word;           ///< the tester's result word (FINISHED, CUT BY DEADLINE, ...)
    std::string why;            ///< the refusal's or the throw's sentence
    bool threw = false;         ///< the run threw (PROGRAM ERROR)
    bool looped = false;        ///< it looped without letting time pass (the no-pace stop)
    std::size_t loopLines = 0;  ///< how many lines arrived with no pace
    std::vector<std::string> loopHead;  ///< the first kLoopLinesKept of them
    std::string loopSentence;   ///< "after step N the auton keeps looping ..."
    bool cut = false;           ///< the deadline cut scoring
    double scoringEnded = 0.0;  ///< T+ scoring ended
    double endActionEnded = 0.0;  ///< T+ the end action ended (0: never ran)
    double armedAt = 0.0;         ///< the robot clock when the guard armed (every T+ is from here)
    std::vector<Sample> samples;  ///< from the arming on
    std::vector<Step> steps;      ///< from below, in the order they began
    std::vector<LogLine> routineLines;  ///< every RTN line, verbatim
    std::vector<LogLine> wallLines;     ///< every WALL line, verbatim
    std::vector<LogLine> allLines;      ///< every line logged from the arming on (the page's log)
    std::vector<FieldNote> field;       ///< THE FIELD CHECK's notes, in time order
    bool through = false;               ///< a THROUGH happened ...
    std::size_t throughAt = 0;          ///< ... at this sample
    int exitCode = 0;                   ///< 0 drawn and nothing wrong; 1 refused/threw/THROUGH/looped
};

// ── THE FIELD CHECK (header) ───────────────────────────────────────────────────────────────────

/// The outline's four corners at (x, y, theta): front-left, front-right, back-right, back-left.
struct Outline {
    double cx[4]{};  ///< corner x
    double cy[4]{};  ///< corner y
};
/// The square of side kPreviewFootprintIn centred on (x, y), turned by theta (rad).
[[nodiscard]] inline Outline outlineAt(double x, double y, double theta) noexcept {
    const double h = kPreviewFootprintIn / 2.0;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    // forward u = (c, s), left v = (-s, c)
    const double fx[4] = {h, h, -h, -h};
    const double ly[4] = {h, -h, -h, h};
    Outline o;
    for (int i = 0; i < 4; ++i) {
        o.cx[i] = x + fx[i] * c - ly[i] * s;
        o.cy[i] = y + fx[i] * s + ly[i] * c;
    }
    return o;
}

/// How far a corner of `o` reaches past the inner face of the wall box `b` (0 when none does) — the
/// face is the box's side toward the field's centre.
[[nodiscard]] inline double wallPenetration(const Outline& o, const shulib::sim::FieldBox& b) noexcept {
    double worst = 0.0;
    const bool xWall = b.yMax - b.yMin > b.xMax - b.xMin;  // a long box along y is an x wall (left/right)
    for (int i = 0; i < 4; ++i) {
        double pen = 0.0;
        if (xWall) {
            pen = b.xMax <= 0.0 ? b.xMax - o.cx[i] : o.cx[i] - b.xMin;
        } else {
            pen = b.yMax <= 0.0 ? b.yMax - o.cy[i] : o.cy[i] - b.yMin;
        }
        worst = std::max(worst, pen);
    }
    return worst;
}

/// How deep the outline `o` and the box `b` overlap in the floor plane (the separating-axis depth:
/// the least overlap over the four axes; 0 when they are apart or only touch).
[[nodiscard]] inline double boxDepth(const Outline& o, const shulib::sim::FieldBox& b) noexcept {
    const double bx[4] = {b.xMin, b.xMax, b.xMax, b.xMin};
    const double by[4] = {b.yMin, b.yMin, b.yMax, b.yMax};
    double axes[4][2] = {{1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}, {0.0, 0.0}};
    const double ex = o.cx[1] - o.cx[0];
    const double ey = o.cy[1] - o.cy[0];
    const double len = std::hypot(ex, ey);
    axes[2][0] = ex / len;
    axes[2][1] = ey / len;
    axes[3][0] = -axes[2][1];
    axes[3][1] = axes[2][0];
    double depth = 1e18;
    for (const auto& a : axes) {
        double oMin = 1e18;
        double oMax = -1e18;
        double bMin = 1e18;
        double bMax = -1e18;
        for (int i = 0; i < 4; ++i) {
            const double po = o.cx[i] * a[0] + o.cy[i] * a[1];
            const double pb = bx[i] * a[0] + by[i] * a[1];
            oMin = std::min(oMin, po);
            oMax = std::max(oMax, po);
            bMin = std::min(bMin, pb);
            bMax = std::max(bMax, pb);
        }
        const double overlap = std::min(oMax, bMax) - std::max(oMin, bMin);
        if (overlap <= 0.0) {
            return 0.0;
        }
        depth = std::min(depth, overlap);
    }
    return depth;
}

/// What one box is to the field check.
enum class BoxRole : std::uint8_t { Wall, Solid, Touch, Piece };
/// A box's role by its field-model name (header: walls and Loaders can be gone THROUGH; Toggles only
/// touched; start pieces are grey notes).
[[nodiscard]] inline BoxRole boxRole(const shulib::sim::FieldBox& b) noexcept {
    const std::string n{b.name};
    if (b.wall) {
        return BoxRole::Wall;
    }
    if (n == "Loader") {
        return BoxRole::Solid;
    }
    if (n.rfind("start", 0) == 0) {
        return BoxRole::Piece;
    }
    return BoxRole::Touch;
}

/// The field the check judges against (the host test plant's own: sim::FieldModel::override2026()).
[[nodiscard]] inline const shulib::sim::FieldModel& previewField() {
    static const shulib::sim::FieldModel f = shulib::sim::FieldModel::override2026();
    return f;
}

/// THE FIELD CHECK over `m.samples` (header): fills `m.field`, `m.through` and `m.throughAt`, each
/// note carrying the from-below step open at its sample. Stops looking at the first THROUGH (the
/// robot would not get past it).
inline void fieldCheck(Model& m) {
    const auto& boxes = previewField().boxes();
    std::vector<char> inContact(boxes.size(), 0);
    std::vector<char> pieceSeen(boxes.size(), 0);
    m.field.clear();
    m.through = false;
    m.throughAt = 0;
    const auto stepAt = [&m](std::size_t s) {
        int open = 0;
        for (const Step& st : m.steps) {
            if (st.kind != StepKind::Mechanism && s >= st.s0 && s <= st.s1) {
                open = st.number;
            }
        }
        return open;
    };
    for (std::size_t s = 0; s < m.samples.size(); ++s) {
        const Sample& p = m.samples[s];
        const Outline o = outlineAt(p.x, p.y, p.theta);
        for (std::size_t b = 0; b < boxes.size(); ++b) {
            const BoxRole role = boxRole(boxes[b]);
            const double depth = role == BoxRole::Wall ? wallPenetration(o, boxes[b]) : boxDepth(o, boxes[b]);
            if ((role == BoxRole::Wall || role == BoxRole::Solid) && depth > kThroughAllowanceIn) {
                m.field.push_back(FieldNote{Contact::Through, boxes[b].name, p.t, s, stepAt(s), depth});
                m.through = true;
                m.throughAt = s;
                return;
            }
            const bool touching = depth > 0.0;
            if (touching && inContact[b] == 0) {
                if (role == BoxRole::Piece) {
                    if (pieceSeen[b] == 0) {
                        m.field.push_back(FieldNote{Contact::Piece, boxes[b].name, p.t, s, stepAt(s), depth});
                        pieceSeen[b] = 1;
                    }
                } else {
                    m.field.push_back(FieldNote{Contact::Touch, boxes[b].name, p.t, s, stepAt(s), depth});
                }
            }
            inContact[b] = touching ? 1 : 0;
        }
    }
}

}  // namespace route_preview
