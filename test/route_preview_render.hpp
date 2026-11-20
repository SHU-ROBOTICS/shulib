#pragma once
//
// route_preview_render.hpp — the ROUTE PREVIEW's two outputs, both drawn from ONE Model (chunk RPV1-A,
// 2026-11-12; the RPV1 brief's §1.1): the stdout table (renderText) and the one offline page
// (renderHtml, with the field picture renderSvg inline). Host only. Say "host test plant" (the project's
// one name for it).
//
// ── The rules the page keeps ─────────────────────────────────────────────────────────────────────
//   * THE PATH IS THE PLANT'S POSES, nothing else: every recorded sample, mapped by fieldPx() (5 px
//     per inch, +Y UP — the red station at the bottom), never a redrawn line or spline (a curve is
//     drawn from the poses the plant drove). The whole path is ONE polyline, id "path"; each motion
//     step is drawn over it in its colour, from its own samples.
//   * NOTHING FROM OUTSIDE THE PAGE: no external URL, no script or style fetched, no wall-clock time
//     (two runs of the same tree write the same bytes), every string from the program HTML-escaped.
//   * THE TABLE IS ONE TABLE: tableRows() makes the cells; renderText pads them for the terminal and
//     renderHtml puts them in <td>s, so the two cannot disagree.
//   * THE CAPTION AND THE ROBOT LINE are on every page, the PLACEHOLDER notes on every step whose
//     time comes from a number nobody has measured, and every fact the robot's ledger still OWES.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "route_preview_model.hpp"
#include "shulib/localization/override_walls.hpp"

namespace route_preview {

/// The caption every page carries (the brief's D5 (a)).
inline constexpr const char* kCaption =
    "IDEAL host test plant, not the robot: no wheel slip, no battery sag, motors that answer at once. "
    "It shows what the library would do with this code on a perfect robot. The times are not a promise "
    "either way \xe2\x80\x94 time it on the robot.";
/// The step-numbering line (D2 (a); until the library listener, RPV1-A2): what the preview can see from
/// below (route_preview_model.hpp's banner) and what it cannot.
inline constexpr const char* kNumberingLine =
    "Numbers count every motion (drives, curves, strafes, turns, brakes and holds) and every mechanism step; "
    "pauses, waits (waitFor, waitUntilDone), then(...) steps and startAt are not counted \xe2\x80\x94 the "
    "library's own lines below give your code's step numbers.";
/// What the field drawing holds.
inline constexpr const char* kFieldLine =
    "Drawn: the walls, Toggles, Loaders, Load Zones and start pieces \xe2\x80\x94 the field parts the host "
    "test plant's field model has. Anything else on the field, the goals included, is not drawn or checked "
    "here (the route builder checks the goals), and the host test plant has no walls, pieces or "
    "other robots: it never stops or pushes anything.";

/// The page's pixels per inch.
inline constexpr double kPxPerIn = 5.0;
/// Half the drawn field, inches (the walls' outer side is at 72.2).
inline constexpr double kHalfFieldIn = 74.0;

/// A field point (in) as page pixels: x right, +Y UP (header).
struct Px {
    double x;  ///< pixels from the left
    double y;  ///< pixels from the top
};
/// The field transform (header).
[[nodiscard]] inline Px fieldPx(double x, double y) noexcept {
    return Px{(x + kHalfFieldIn) * kPxPerIn, (kHalfFieldIn - y) * kPxPerIn};
}

/// HTML-escape `s` (& < > " ').
[[nodiscard]] inline std::string esc(std::string_view s) {
    std::string out;
    out.reserve(s.size());
    for (const char c : s) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&#39;"; break;
            default: out += c;
        }
    }
    return out;
}

/// `v` with `decimals` places ("-0.00" written "0.00", so a run never differs by a sign of zero).
[[nodiscard]] inline std::string fixed(double v, int decimals) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*f", decimals, v);
    std::string s{b};
    if (s.size() > 1 && s[0] == '-' && s.find_first_not_of("-0.") == std::string::npos) {
        s.erase(0, 1);
    }
    return s;
}

/// `v` with at most two places and no trailing zeros ("15", "1", "0.5").
[[nodiscard]] inline std::string trimmed(double v) {
    std::string s = fixed(v, 2);
    while (s.find('.') != std::string::npos && (s.back() == '0' || s.back() == '.')) {
        const bool dot = s.back() == '.';
        s.pop_back();
        if (dot) {
            break;
        }
    }
    return s;
}

/// What the outline is (D12 (a)), its two numbers printed from the constants the check uses.
[[nodiscard]] inline std::string outlineLine() {
    return "The robot is drawn as a " + trimmed(kPreviewFootprintIn) +
           " in square centred on the point the plant tracks, with " + trimmed(kThroughAllowanceIn) +
           " in of allowance before an overlap counts as going THROUGH a wall or a Loader. The square is a "
           "drawing assumption \xe2\x80\x94 the real frame may be smaller, its tracked point may not be its "
           "middle, and a claw or lift reaching past it is not checked.";
}

/// A pose as the table writes it: "(x, y) h°".
[[nodiscard]] inline std::string poseText(double x, double y, double theta) {
    double deg = std::remainder(theta * 180.0 / 3.14159265358979323846, 360.0);
    return "(" + fixed(x, 1) + ", " + fixed(y, 1) + ") " + fixed(deg, 1) + "\xc2\xb0";
}

/// The step's label: "drive", "turn", or the mechanism's name.
[[nodiscard]] inline std::string stepLabel(const Step& s) {
    return s.kind == StepKind::Mechanism ? s.what : std::string{stepKindWord(s.kind)};
}

/// THE TABLE (header): the header row, then one row per step. Both outputs draw these cells.
[[nodiscard]] inline std::vector<std::vector<std::string>> tableRows(const Model& m) {
    std::vector<std::vector<std::string>> rows;
    rows.push_back({"#", "step", "from", "to", "T+ start", "T+ end", "result"});
    for (const Step& s : m.steps) {
        const Sample* a = s.s0 < m.samples.size() ? &m.samples[s.s0] : nullptr;
        const Sample* b = s.s1 < m.samples.size() ? &m.samples[s.s1] : nullptr;
        std::string result = s.verdict;
        if (s.background) {
            result += " (in the background)";
        }
        if (!s.placeholder.empty()) {
            result += " [PLACEHOLDER]";
        }
        rows.push_back({s.endAction ? std::string{"park"} : std::to_string(s.number), stepLabel(s),
                        a != nullptr ? poseText(a->x, a->y, a->theta) : std::string{"-"},
                        b != nullptr ? poseText(b->x, b->y, b->theta) : std::string{"-"}, fixed(s.t0, 2),
                        fixed(s.t1, 2), result});
    }
    return rows;
}

/// The run's one-line summary ("auton 'x' (SKILLS, park T+57.00 s, stop T+59.00 s): FINISHED").
[[nodiscard]] inline std::string summaryLine(const Model& m) {
    if (m.period <= 0.0) {  // refused by the registry: never registered, so it has no schedule to show
        return "auton '" + m.auton + "': " + m.word;
    }
    std::string s = "auton '" + m.auton + "' (" + m.kindWord + ", park T+" + fixed(m.park, 2) + " s, stop T+" +
                    fixed(m.stop, 2) + " s): " + m.word;
    if (m.threw && !m.samples.empty()) {
        // A throw ends the run before the guard reports when scoring ended: the time is the last pace's.
        s += " \xe2\x80\x94 it threw at T+" + fixed(m.samples.back().t, 2) + " s of the " + fixed(m.period, 0) +
             " s period";
    } else if (!m.refused && !m.samples.empty() && !m.looped) {
        s += " \xe2\x80\x94 scoring ended T+" + fixed(m.scoringEnded, 2) + " s of the " + fixed(m.period, 0) +
             " s period";
    }
    if (m.through && !m.field.empty()) {
        s += " \xe2\x80\x94 but it goes THROUGH the " + m.field.back().element + " at T+" + fixed(m.field.back().t, 2) +
             " s";
    }
    return s;
}

/// The stamp line ("commit abc1234 \xc2\xb7 autons file 0123456789ab", or "commit not stamped").
[[nodiscard]] inline std::string stampLine(const Model& m) {
    std::string s = m.stamp.empty() ? std::string{"commit not stamped"} : "commit " + m.stamp;
    if (!m.autonsHash.empty()) {
        s += " \xc2\xb7 autons file " + m.autonsHash;
    }
    return s;
}

/// The line over the robot's OWED facts, or "" when it owes none.
[[nodiscard]] inline std::string owedLine(const Model& m) {
    if (m.owed.empty()) {
        return "";
    }
    return "PLACEHOLDER numbers: " + std::to_string(m.owed.size()) + " of this robot's facts are still OWED in " +
           m.ledgerFile + " \xe2\x80\x94 wherever the preview needs one, it uses the placeholder until it is typed:";
}

/// A field note in words.
[[nodiscard]] inline std::string fieldNoteText(const FieldNote& f) {
    const std::string at = " at T+" + fixed(f.t, 2) + " s" +
                           (f.step > 0 ? " in step " + std::to_string(f.step) : std::string{" between steps"});
    switch (f.kind) {
        case Contact::Through:
            return "THROUGH: the host test plant has no walls; on the robot it hits the " + f.element + " here" + at +
                   " (" + fixed(f.depth, 2) + " in past its face \xe2\x80\x94 more than the " +
                   trimmed(kThroughAllowanceIn) + " in allowance). The robot would not get past this point.";
        case Contact::Touch:
            return "touches the " + f.element + at + " \xe2\x80\x94 the plant has no " + f.element +
                   "; touching is normal, not a failure";
        case Contact::Piece:
            return "drives over a " + f.element + at + " (the plant does not push pieces)";
    }
    return "";
}

/// THE STDOUT TABLE (header): the summary, the robot line, the notices, the table (cells padded, " | "
/// between), the library's RTN and WALL lines, the field notes, the caption.
[[nodiscard]] inline std::string renderText(const Model& m) {
    std::string out;
    out += "ROUTE PREVIEW \xe2\x80\x94 " + summaryLine(m) + "\n";
    out += m.robotLine + "\n";
    out += stampLine(m) + "\n";
    if (!m.owed.empty()) {
        out += owedLine(m) + "\n";
        for (const std::string& f : m.owed) {
            out += "  [PLACEHOLDER] " + f + "\n";
        }
    }
    for (const std::string& n : m.notices) {
        out += "!! " + n + "\n";
    }
    if (!m.why.empty()) {
        out += (m.threw ? "THREW: " : "WHY: ") + m.why + "\n";
    }
    if (m.looped) {
        out += "LOOP: " + m.loopSentence + "\n";
    }
    if (!m.refused) {
        out += kNumberingLine;
        out += "\n";
        const auto rows = tableRows(m);
        std::vector<std::size_t> width(rows[0].size(), 0);
        for (const auto& r : rows) {
            for (std::size_t c = 0; c < r.size(); ++c) {
                std::size_t w = 0;
                for (const char ch : r[c]) {
                    w += (static_cast<unsigned char>(ch) & 0xC0) != 0x80 ? 1 : 0;  // count UTF-8 characters
                }
                width[c] = std::max(width[c], w);
            }
        }
        for (const auto& r : rows) {
            std::string line;
            for (std::size_t c = 0; c < r.size(); ++c) {
                std::size_t w = 0;
                for (const char ch : r[c]) {
                    w += (static_cast<unsigned char>(ch) & 0xC0) != 0x80 ? 1 : 0;
                }
                line += r[c];
                if (c + 1 < r.size()) {
                    line += std::string(width[c] - w, ' ') + " | ";
                }
            }
            out += line + "\n";
        }
    }
    for (const Step& s : m.steps) {
        if (!s.sentence.empty() && s.mark != Mark::Ok) {
            out += "step " + (s.endAction ? std::string{"park"} : std::to_string(s.number)) + ": " + s.sentence + "\n";
        }
        if (!s.placeholder.empty()) {
            out += "step " + (s.endAction ? std::string{"park"} : std::to_string(s.number)) + ": " + s.placeholder +
                   "\n";
        }
    }
    for (const LogLine& l : m.routineLines) {
        out += "T+" + fixed(l.t, 2) + " [RTN] " + l.text + "\n";
    }
    for (const LogLine& l : m.wallLines) {
        out += "T+" + fixed(l.t, 2) + " [WALL] " + l.text + "\n";
    }
    for (const FieldNote& f : m.field) {
        out += fieldNoteText(f) + "\n";
    }
    out += kCaption;
    out += "\n";
    return out;
}

/// "x,y" of a sample, 2 decimals.
[[nodiscard]] inline std::string ptText(double x, double y) {
    const Px p = fieldPx(x, y);
    return fixed(p.x, 2) + "," + fixed(p.y, 2);
}

/// The polyline points of samples [a, b] (inclusive, clamped).
[[nodiscard]] inline std::string points(const Model& m, std::size_t a, std::size_t b) {
    std::string out;
    if (m.samples.empty()) {
        return out;
    }
    b = std::min(b, m.samples.size() - 1);
    for (std::size_t i = a; i <= b; ++i) {
        if (!out.empty()) {
            out += ' ';
        }
        out += ptText(m.samples[i].x, m.samples[i].y);
    }
    return out;
}

/// The outline at sample `s` as an SVG polygon, its front edge marked; `title` is its tooltip.
[[nodiscard]] inline std::string outlineSvg(const Sample& p, const char* cls, const std::string& title) {
    const Outline o = outlineAt(p.x, p.y, p.theta);
    std::string pts;
    for (int i = 0; i < 4; ++i) {
        pts += (i == 0 ? "" : " ") + ptText(o.cx[i], o.cy[i]);
    }
    std::string out = "<polygon class=\"" + std::string{cls} + "\" points=\"" + pts + "\"><title>" + esc(title) +
                      "</title></polygon>";
    out += "<polyline class=\"front\" points=\"" + ptText(o.cx[0], o.cy[0]) + " " + ptText(o.cx[1], o.cy[1]) + "\"/>";
    return out;
}

/// THE FIELD PICTURE (header).
[[nodiscard]] inline std::string renderSvg(const Model& m) {
    const double size = 2.0 * kHalfFieldIn * kPxPerIn;
    std::string out = "<svg id=\"field\" viewBox=\"0 0 " + fixed(size, 0) + " " + fixed(size, 0) + "\" width=\"" +
                      fixed(size, 0) + "\" height=\"" + fixed(size, 0) + "\" role=\"img\">";
    out += "<rect class=\"floor\" x=\"0\" y=\"0\" width=\"" + fixed(size, 0) + "\" height=\"" + fixed(size, 0) + "\"/>";
    // The Load Zones: kLoadZoneDepth from each corner along both walls (the library's season header).
    {
        const double w = shulib::sim::FieldModel::kInnerFace;
        const double z = shulib::localization::override2026::kLoadZoneDepth.value();
        const double corners[4][2] = {{-w, -w}, {w - z, -w}, {w - z, w - z}, {-w, w - z}};
        for (const auto& c : corners) {
            const Px p = fieldPx(c[0], c[1] + z);
            out += "<rect class=\"zone\" x=\"" + fixed(p.x, 2) + "\" y=\"" + fixed(p.y, 2) + "\" width=\"" +
                   fixed(z * kPxPerIn, 2) + "\" height=\"" + fixed(z * kPxPerIn, 2) + "\"/>";
        }
    }
    // The field model's boxes (walls, Toggles, Loaders, start pieces).
    for (const shulib::sim::FieldBox& b : previewField().boxes()) {
        const Px p = fieldPx(b.xMin, b.yMax);
        const BoxRole role = boxRole(b);
        const char* cls = role == BoxRole::Wall    ? "wall"
                          : role == BoxRole::Solid ? "loader"
                          : role == BoxRole::Piece ? "piece"
                                                   : "toggle";
        out += "<rect class=\"" + std::string{cls} + "\" x=\"" + fixed(p.x, 2) + "\" y=\"" + fixed(p.y, 2) +
               "\" width=\"" + fixed((b.xMax - b.xMin) * kPxPerIn, 2) + "\" height=\"" +
               fixed((b.yMax - b.yMin) * kPxPerIn, 2) + "\"><title>" + esc(b.name) + "</title></rect>";
    }
    out += "<text class=\"label\" x=\"" + fixed(size / 2.0, 0) + "\" y=\"" + fixed(size - 4.0, 0) +
           "\" text-anchor=\"middle\">red station</text>";
    if (!m.samples.empty()) {
        const std::size_t last = m.through ? m.throughAt : m.samples.size() - 1;
        // THE PATH: every sample, one polyline (faded after a THROUGH).
        out += "<polyline id=\"path\" class=\"path\" points=\"" + points(m, 0, m.samples.size() - 1) + "\"/>";
        if (m.through && m.throughAt + 1 < m.samples.size()) {
            out += "<polyline class=\"faded\" points=\"" + points(m, m.throughAt, m.samples.size() - 1) + "\"/>";
        }
        // Each motion step in its colour, its outlines at both ends, its aim; every step's number where it
        // began. Numbers that would sit on one another (a route that comes back to a spot, or a drive that
        // ends where the next step begins) share ONE label there: the first few numbers, "+N" for the
        // rest, and every one of them with its times and its result in the label's tooltip. Where a step
        // ENDED is the tooltip of its end outline (a motion) or of its end mark (a mechanism started in the
        // background) — a second visible number there would sit on the next step's.
        struct Label {
            Px at;
            std::vector<std::string> ids;
            std::vector<std::string> tips;
        };
        std::vector<Label> labels;
        const auto labelAt = [&labels](Px a, const std::string& id, const std::string& tip) {
            for (Label& l : labels) {
                if (std::abs(l.at.x - a.x) < 4.0 && std::abs(l.at.y - a.y) < 4.0) {
                    l.ids.push_back(id);
                    l.tips.push_back(tip);
                    return;
                }
            }
            labels.push_back(Label{a, {id}, {tip}});
        };
        for (const Step& s : m.steps) {
            const std::string id = s.endAction ? std::string{"park"} : std::to_string(s.number);
            const std::string name = (s.endAction ? "park " : "step " + id + " ") + stepLabel(s);
            const std::string span = "T+" + fixed(s.t0, 2) + " \xe2\x86\x92 T+" + fixed(s.t1, 2) + " s";
            const bool faded = m.through && s.s0 > last;
            const std::string grp = std::string{"<g class=\"step"} + (faded ? " dim" : "") + "\" data-step=\"" + id + "\">";
            out += grp;
            if (s.kind != StepKind::Mechanism) {
                // After a THROUGH the step goes on faded: the robot would not get there.
                const std::size_t solidTo = m.through ? std::min(s.s1, m.throughAt) : s.s1;
                if (s.s0 <= solidTo) {
                    out += "<polyline class=\"seg " + std::string{markWord(s.mark)} + "\" points=\"" +
                           points(m, s.s0, solidTo) + "\"/>";
                }
                if (solidTo < s.s1) {
                    out += "<polyline class=\"seg dim " + std::string{markWord(s.mark)} + "\" points=\"" +
                           points(m, std::max(s.s0, solidTo), s.s1) + "\"/>";
                }
                out += outlineSvg(m.samples[std::min(s.s0, m.samples.size() - 1)], "outline",
                                  name + " began here, T+" + fixed(s.t0, 2) + " s");
                out += outlineSvg(m.samples[std::min(s.s1, m.samples.size() - 1)], "outline end",
                                  name + " ended here, T+" + fixed(s.t1, 2) + " s: " + s.verdict);
                if (s.hasTarget && (s.kind == StepKind::Drive || s.kind == StepKind::Curve || s.kind == StepKind::Strafe)) {
                    const Px t = fieldPx(s.tx, s.ty);
                    out += "<text class=\"aim\" x=\"" + fixed(t.x, 2) + "\" y=\"" + fixed(t.y + 4.0, 2) +
                           "\" text-anchor=\"middle\">\xc3\x97</text>";
                }
                labelAt(fieldPx(m.samples[std::min(s.s0, m.samples.size() - 1)].x,
                                m.samples[std::min(s.s0, m.samples.size() - 1)].y),
                        id, name + ": " + span + ", " + s.verdict);
            } else {
                // A mechanism step: a diamond where it began. One the robot drove during (started in the
                // background) is its own span, the diamond its "started" mark and a dot where its verdict
                // arrived (the brief's R4).
                const bool spanned = s.background && s.s1 > s.s0;
                if (spanned) {
                    out += "<polyline class=\"span " + std::string{markWord(s.mark)} + "\" points=\"" +
                           points(m, s.s0, s.s1) + "\"/>";
                }
                const Px a = fieldPx(m.samples[std::min(s.s0, m.samples.size() - 1)].x,
                                     m.samples[std::min(s.s0, m.samples.size() - 1)].y);
                const std::string tip = spanned ? name + ": started here in the background, T+" + fixed(s.t0, 2) +
                                                      " s; its verdict " + s.verdict + " arrived at T+" +
                                                      fixed(s.t1, 2) + " s"
                                                : name + ": " + span + ", " + s.verdict;
                out += "<rect class=\"mech " + std::string{markWord(s.mark)} +
                       (s.placeholder.empty() ? "" : " hatch") + "\" x=\"" + fixed(a.x - 5.0, 2) + "\" y=\"" +
                       fixed(a.y - 5.0, 2) + "\" width=\"10\" height=\"10\" transform=\"rotate(45 " + fixed(a.x, 2) +
                       " " + fixed(a.y, 2) + ")\"><title>" + esc(tip) + "</title></rect>";
                if (spanned) {
                    const Px e = fieldPx(m.samples[std::min(s.s1, m.samples.size() - 1)].x,
                                         m.samples[std::min(s.s1, m.samples.size() - 1)].y);
                    out += "<circle class=\"mechend " + std::string{markWord(s.mark)} + "\" cx=\"" + fixed(e.x, 2) +
                           "\" cy=\"" + fixed(e.y, 2) + "\" r=\"4\"><title>" +
                           esc(name + ": " + s.verdict + " here, T+" + fixed(s.t1, 2) + " s") + "</title></circle>";
                }
                labelAt(a, id, tip);
            }
            out += "</g>";
        }
        for (const Label& l : labels) {
            std::string shown;
            std::string all;
            for (std::size_t i = 0; i < l.ids.size(); ++i) {
                if (i < 4) {
                    shown += (i == 0 ? "" : ",") + l.ids[i];
                }
                all += (i == 0 ? "" : " \xc2\xb7 ") + l.tips[i];
            }
            if (l.ids.size() > 4) {
                shown += " +" + std::to_string(l.ids.size() - 4);
            }
            out += "<text class=\"num\" x=\"" + fixed(l.at.x + 6.0, 2) + "\" y=\"" + fixed(l.at.y - 6.0, 2) + "\">" +
                   esc(shown) + "<title>" + esc(all) + "</title></text>";
        }
        // Wall resets, the library's stop lines, the field notes.
        for (const LogLine& l : m.wallLines) {
            const Px a = fieldPx(m.samples[std::min(l.s, m.samples.size() - 1)].x,
                                 m.samples[std::min(l.s, m.samples.size() - 1)].y);
            out += "<circle class=\"wallmark " + std::string{markWord(l.mark)} + "\" cx=\"" + fixed(a.x, 2) + "\" cy=\"" +
                   fixed(a.y, 2) + "\" r=\"7\"><title>" + esc(l.text) + "</title></circle>";
        }
        for (const LogLine& l : m.routineLines) {
            if (l.mark != Mark::Failed) {
                continue;
            }
            const Px a = fieldPx(m.samples[std::min(l.s, m.samples.size() - 1)].x,
                                 m.samples[std::min(l.s, m.samples.size() - 1)].y);
            out += "<text class=\"stop\" x=\"" + fixed(a.x, 2) + "\" y=\"" + fixed(a.y + 6.0, 2) +
                   "\" text-anchor=\"middle\">\xe2\x9c\x95<title>" + esc(l.text) + "</title></text>";
        }
        for (const FieldNote& f : m.field) {
            const Px a = fieldPx(m.samples[f.s].x, m.samples[f.s].y);
            const char* cls = f.kind == Contact::Through ? "through" : f.kind == Contact::Touch ? "touch" : "pnote";
            out += "<circle class=\"" + std::string{cls} + "\" cx=\"" + fixed(a.x, 2) + "\" cy=\"" + fixed(a.y, 2) +
                   "\" r=\"" + (f.kind == Contact::Through ? "12" : "6") + "\"><title>" + esc(fieldNoteText(f)) +
                   "</title></circle>";
        }
    }
    out += "</svg>";
    return out;
}

/// The timeline: 0 to the period, each step a bar, the park and stop times marked.
[[nodiscard]] inline std::string renderTimeline(const Model& m) {
    const double w = 740.0;
    const double period = m.period > 0.0 ? m.period : 60.0;
    const auto xt = [&](double t) { return std::clamp(t, 0.0, period) / period * (w - 20.0) + 10.0; };
    std::string out = "<svg id=\"timeline\" viewBox=\"0 0 740 60\" width=\"740\" height=\"60\" role=\"img\">";
    out += "<line class=\"axis\" x1=\"10\" y1=\"40\" x2=\"730\" y2=\"40\"/>";
    for (int s = 0; s <= static_cast<int>(period); s += 10) {
        const double x = xt(s);
        out += "<text class=\"tick\" x=\"" + fixed(x, 1) + "\" y=\"56\" text-anchor=\"middle\">" + std::to_string(s) +
               " s</text>";
    }
    for (const Step& s : m.steps) {
        const double a = xt(s.t0);
        const double b = std::max(xt(s.t1), a + 1.0);
        out += "<rect class=\"bar " + std::string{markWord(s.mark)} + "\" x=\"" + fixed(a, 1) + "\" y=\"" +
               (s.kind == StepKind::Mechanism ? "22" : "28") + "\" width=\"" + fixed(b - a, 1) + "\" height=\"8\"/>";
    }
    out += "<line class=\"park\" x1=\"" + fixed(xt(m.park), 1) + "\" y1=\"14\" x2=\"" + fixed(xt(m.park), 1) +
           "\" y2=\"44\"/><text class=\"tick\" x=\"" + fixed(xt(m.park), 1) + "\" y=\"11\" text-anchor=\"middle\">park " +
           fixed(m.park, 0) + " s</text>";
    out += "<line class=\"hardstop\" x1=\"" + fixed(xt(m.stop), 1) + "\" y1=\"14\" x2=\"" + fixed(xt(m.stop), 1) +
           "\" y2=\"44\"/>";
    out += "</svg>";
    return out;
}

/// THE PAGE (header): one self-contained HTML document.
[[nodiscard]] inline std::string renderHtml(const Model& m) {
    std::string h;
    h += "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">";
    h += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    h += "<title>Route preview \xe2\x80\x94 " + esc(m.auton) + "</title><style>";
    h += "body{font:14px/1.45 system-ui,sans-serif;margin:16px;background:#fff;color:#1d1d1f;max-width:1200px}"
         "h1{font-size:20px;margin:0 0 4px}p{margin:4px 0}.robot{font-weight:600}"
         ".notice{color:#b3261e;font-weight:600}.caption{background:#fff6d6;border:1px solid #e0c060;padding:6px 8px}"
         ".wrap{overflow-x:auto}table{border-collapse:collapse;margin:8px 0}td,th{border:1px solid #ccc;padding:2px 6px;"
         "text-align:left;font-variant-numeric:tabular-nums}tr.hl td{background:#e8f0fe}"
         ".floor{fill:#eceff1}.wall{fill:#455a64}.toggle{fill:#90a4ae}.loader{fill:#6d4c41}.piece{fill:#bdbdbd}"
         ".zone{fill:none;stroke:#78909c;stroke-dasharray:4 3}.label,.tick{font-size:11px;fill:#455a64}"
         ".path{fill:none;stroke:#9e9e9e;stroke-width:1.5}.faded{fill:none;stroke:#fff;stroke-width:4;opacity:.7}"
         ".seg{fill:none;stroke-width:3}.span{fill:none;stroke-width:9;opacity:.35}"
         ".ok{stroke:#2e7d32;fill:#2e7d32}.unconfirmed{stroke:#00897b;fill:#00897b}.failed{stroke:#c62828;fill:#c62828}"
         ".cut,.amber{stroke:#ef8f00;fill:#ef8f00}.grey{stroke:#9e9e9e;fill:#9e9e9e}"
         ".outline{fill:none;stroke:#37474f;stroke-width:.8;opacity:.5}.outline.end{opacity:.8}"
         ".front{fill:none;stroke:#1565c0;stroke-width:2}.num{font-size:11px;font-weight:700;fill:#1565c0}"
         ".aim{font-size:14px;fill:#1565c0}.mech{stroke:#000;stroke-width:.6}.hatch{stroke-dasharray:2 2;stroke-width:2}"
         ".wallmark{fill:none;stroke-width:3}.stop{font-size:16px;fill:#c62828;font-weight:700}"
         ".through{fill:none;stroke:#c62828;stroke-width:3}.touch{fill:none;stroke:#ef8f00;stroke-width:2}"
         ".pnote{fill:none;stroke:#757575;stroke-width:1.5}.dim{opacity:.25}.mechend{stroke:#000;stroke-width:.6}"
         ".bar.ok{fill:#2e7d32}.axis,.park,.hardstop{stroke:#455a64}.hardstop{stroke:#c62828}"
         "g.step.hl .seg{stroke-width:6}li.failed{color:#b3261e}li.amber,li.cut{color:#9a5b00}li.grey{color:#757575}"
         "code{font-size:12px}.legend{font-size:12px;color:#455a64}.k-ok{color:#2e7d32}.k-failed{color:#c62828}"
         ".k-amber{color:#ef8f00}.k-unconfirmed{color:#00897b}li.unconfirmed{color:#00695c}";
    h += "</style></head><body>";
    h += "<h1>Route preview: " + esc(summaryLine(m)) + "</h1>";
    h += "<p class=\"robot\">" + esc(m.robotLine) + "</p>";
    h += "<p class=\"caption\">" + esc(kCaption) + "</p>";
    h += "<p>" + esc(stampLine(m)) + " \xc2\xb7 started " + esc(m.startLine) + "</p>";
    if (!m.owed.empty()) {
        h += "<div class=\"owed\" style=\"color:#9a5b00;border:1px dashed #ef8f00;padding:2px 8px\"><p>" +
             esc(owedLine(m)) + "</p><ul>";
        for (const std::string& f : m.owed) {
            h += "<li>" + esc(f) + "</li>";
        }
        h += "</ul></div>";
    }
    for (const std::string& n : m.notices) {
        h += "<p class=\"notice\">" + esc(n) + "</p>";
    }
    if (!m.why.empty()) {
        h += "<p class=\"notice\">" + std::string{m.threw ? "THREW: " : "WHY: "} + esc(m.why) + "</p>";
    }
    if (m.looped) {
        h += "<p class=\"notice\">" + esc(m.loopSentence) + "</p>";
    }
    if (!m.refused) {
        h += "<p>" + esc(kNumberingLine) + "</p>";
    }
    h += "<div class=\"wrap\">" + renderSvg(m) + "</div>";
    h += "<p class=\"legend\"><b class=\"k-ok\">green</b> did what it was asked \xc2\xb7 <b class=\"k-failed\">red</b> "
         "failed or refused (the library's sentence below) \xc2\xb7 <b class=\"k-amber\">amber</b> cut short (by the "
         "deadline, or by a throw: the PROGRAM ERROR line says where it happened), a refused wall reset, or a TOUCH \xc2\xb7 <b class=\"k-unconfirmed\">teal</b> counted done, unconfirmed \xc2\xb7 "
         "dashed diamond: a PLACEHOLDER number \xc2\xb7 diamond: a mechanism step \xc2\xb7 square: the robot's outline, "
         "front edge blue \xc2\xb7 \xc3\x97: where a drive was aimed \xc2\xb7 thick stretch: a mechanism running while the "
         "robot drove, a dot where its verdict arrived \xc2\xb7 hover a number or a mark for its times \xc2\xb7 red ring: THROUGH \xc2\xb7 amber ring: TOUCH</p>";
    h += "<div class=\"wrap\">" + renderTimeline(m) + "</div>";
    if (!m.refused) {
        h += "<div class=\"wrap\"><table id=\"steps\">";
        const auto rows = tableRows(m);
        for (std::size_t r = 0; r < rows.size(); ++r) {
            const bool head = r == 0;
            h += head ? "<tr>" : "<tr data-step=\"" + esc(rows[r][0]) + "\">";
            for (const std::string& c : rows[r]) {
                h += (head ? "<th>" : "<td>") + esc(c) + (head ? "</th>" : "</td>");
            }
            h += "</tr>";
        }
        h += "</table></div>";
    }
    h += "<h2>What the library said</h2><ul>";
    for (const Step& s : m.steps) {
        const std::string id = s.endAction ? std::string{"park"} : std::to_string(s.number);
        if (!s.sentence.empty() && s.mark != Mark::Ok) {
            h += "<li class=\"" + std::string{markWord(s.mark)} + "\">step " + esc(id) + ": " + esc(s.sentence) + "</li>";
        }
        if (!s.placeholder.empty()) {
            h += "<li class=\"amber\">step " + esc(id) + ": " + esc(s.placeholder) + "</li>";
        }
    }
    for (const LogLine& l : m.routineLines) {
        h += "<li class=\"" + std::string{markWord(l.mark)} + "\">T+" + fixed(l.t, 2) + " [RTN] " + esc(l.text) + "</li>";
    }
    for (const LogLine& l : m.wallLines) {
        h += "<li class=\"" + std::string{markWord(l.mark)} + "\">T+" + fixed(l.t, 2) + " [WALL] " + esc(l.text) +
             "</li>";
    }
    for (const FieldNote& f : m.field) {
        const char* cls = f.kind == Contact::Through ? "failed" : f.kind == Contact::Touch ? "amber" : "grey";
        h += "<li class=\"" + std::string{cls} + "\">" + esc(fieldNoteText(f)) + "</li>";
    }
    h += "</ul>";
    if (m.looped && !m.loopHead.empty()) {
        h += "<h2>The looping lines (the first " + std::to_string(m.loopHead.size()) + " of " +
             std::to_string(m.loopLines) + ")</h2><pre>";
        for (const std::string& l : m.loopHead) {
            h += esc(l) + "\n";
        }
        h += "</pre>";
    }
    h += "<p>" + esc(outlineLine()) + "</p><p>" + esc(kFieldLine) + "</p>";
    h += "<h2>Every line the library logged from the start of the auton</h2><pre>";
    for (const LogLine& l : m.allLines) {
        h += "T+" + fixed(l.t, 2) + " [" + esc(l.subsystem) + "] " + esc(l.text) + "\n";
    }
    h += "</pre>";
    // Hover a row: its step is drawn thicker (a few lines, nothing fetched).
    h += "<script>(function(){var t=document.getElementById('steps');if(!t)return;"
         "t.addEventListener('mouseover',function(e){var r=e.target.closest('tr');if(!r||!r.dataset.step)return;"
         "document.querySelectorAll('.hl').forEach(function(x){x.classList.remove('hl')});r.classList.add('hl');"
         "document.querySelectorAll('g.step[data-step=\"'+r.dataset.step+'\"]').forEach(function(g){g.classList.add('hl')});});"
         "})();</script>";
    h += "</body></html>\n";
    return h;
}

}  // namespace route_preview
