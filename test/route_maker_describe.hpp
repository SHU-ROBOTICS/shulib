#pragma once
//
// route_maker_describe.hpp — ONE ROBOT'S DESCRIPTION for the route builder, as JSON, from its REAL tables
// (chunk RMP1-A1, 2026-11-14; the RMP1 brief §1.1 "The C++ side", A.3.2 (b); [R6], [R8], [R16]). Host only:
// nothing a robot program compiles.
//
// The route builder (tools/route_maker/, a web page) cannot read C++, so the robot's facts reach it through
// this file: every mechanism of the robot's own list (its name, its declaration's C++ name, its kind, its
// homing method, its unhomed volts (RMP1-B's controls), its presets or positions — a preset that is a level of robot one's facts ledger is marked
// with its index, so the export names it `m[facts::lv(i).name]`), the drive the page plans on (a tank, the
// library's own tank-steering window and curve rules — MotionConfig's defaults, the numbers a Chassis is
// built with when nothing overrides them), the field's numbers (localization/override_walls.hpp,
// vision/override_elements.hpp) and the API version; since RMP1-C also each arm's and lift's home height, the
// brain menu's name budgets and size, the other autons' names, the pair check's two numbers (mirrored_pair_numbers.hpp) and the skills mirror;
// since RMP1-H the field's solid structures (override_field_obstacles.hpp: the goals and the Loaders, each footprint and overhang cited). A number the tables do not state is LEFT OUT, never
// filled from the host test plant's stand-in drive [R16].
//
// The declaration's C++ name (what the export writes — `clawPivot6`, not its `.name` "claw pivot 6") is
// read from the SOURCE LINE the declaration recorded (mechanism::MechanismSpec::declaredFile/Line: the
// `.name = ...` line, std::source_location) — the identifier after `constexpr <Kind>` on that line or the
// three before it. No path is written into the description (RG1: the published page must not carry a
// laptop's or a runner's directory).
//
// Used by test/route_maker_main.cpp (`shulib_route_maker describe --robot one|two|x|y|z`, run in CI to make
// the page's robots.json) and by test/route_maker_golden_test.cpp (the committed fixture descriptions the
// page's node tests read must equal what this writes).

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "mirrored_pair_numbers.hpp"
#include "override_field_obstacles.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/motion/curve_path.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/version.hpp"
#include "shulib/vision/override_elements.hpp"

namespace route_maker {

/// The autons both autons files register BESIDE the route builder's routes (the 40 in, the acceptance run), by name:
/// no route may take one. Typed here (this program does not compile src/team/); the suite holds it equal to both
/// robots' real lists (route_maker_golden_test.cpp, "RMP1-C: the other autons").
inline constexpr std::array<const char*, 2> kOtherAutonNames{"40in forward", shulib::competition::kAcceptanceRunName};

/// A double as the shortest text that reads back to the same double (std::to_chars), so the page's
/// numbers are the C++'s, bit for bit.
[[nodiscard]] inline std::string num(double v) {
    if (!std::isfinite(v)) {
        return "null";
    }
    if (v == 0.0) {
        return "0";  // never "-0"
    }
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof buf, v);
    return std::string{buf, r.ptr};
}

/// A JSON string literal (the names here are printable ASCII; anything else is escaped).
[[nodiscard]] inline std::string str(std::string_view s) {
    std::string out = "\"";
    for (const char c : s) {
        const auto u = static_cast<unsigned char>(c);
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (u < 0x20 || u > 0x7e) {
            char esc[8];
            const auto r = std::to_chars(esc, esc + sizeof esc, static_cast<unsigned>(u), 16);
            out += "\\u00";
            if (r.ptr - esc < 2) {
                out += '0';
            }
            out.append(esc, r.ptr);
        } else {
            out += c;
        }
    }
    return out + "\"";
}

/// The C++ identifier a declaration was written under, read from its recorded source line (file banner):
/// the word after `constexpr <Kind>` on `line` or one of the three lines before it. Empty when the file
/// cannot be read or the line is not a declaration of that shape (a hand-built table).
[[nodiscard]] inline std::string identAt(const char* file, std::uint32_t line) {
    if (file == nullptr || line == 0) {
        return {};
    }
#ifdef SHULIB_SOURCE_DIR
    // A name the build made relative (W10's prefix map) is this build's tree's, never the working directory's.
    std::ifstream in{file[0] == '/' ? std::string{file} : std::string{SHULIB_SOURCE_DIR} + "/" + file};
#else
    std::ifstream in{file};
#endif
    if (!in) {
        return {};
    }
    std::vector<std::string> lines;
    std::string l;
    while (std::getline(in, l)) {
        lines.push_back(l);
        if (lines.size() >= line) {
            break;
        }
    }
    if (lines.size() < line) {
        return {};
    }
    auto isWord = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_'; };
    for (std::uint32_t back = 0; back <= 3 && back < line; ++back) {
        const std::string& s = lines[line - 1 - back];
        const std::size_t at = s.find("constexpr ");
        if (at == std::string::npos) {
            continue;
        }
        std::size_t i = at + 10;
        auto skipSpace = [&] { while (i < s.size() && s[i] == ' ') { ++i; } };
        auto word = [&] {
            const std::size_t b = i;
            while (i < s.size() && (isWord(s[i]) || s[i] == ':')) {
                ++i;
            }
            return s.substr(b, i - b);
        };
        skipSpace();
        const std::string kind = word();
        skipSpace();
        const std::string ident = word();
        if (!kind.empty() && !ident.empty() && ident.find(':') == std::string::npos) {
            return ident;
        }
    }
    return {};
}

/// The words for a homing method (the page's, and RC25's input).
[[nodiscard]] constexpr const char* homingWord(shulib::mechanism::HomeMethod h) noexcept {
    using shulib::mechanism::HomeMethod;
    switch (h) {
        case HomeMethod::None: return "never";
        case HomeMethod::RestsAtStop: return "restsAtStop";
        case HomeMethod::Switch: return "switch";
        case HomeMethod::Stall: return "stall";
        case HomeMethod::Absolute: return "absolute";
    }
    return "unset";
}

/// A preset name's text (up to its NUL, at most its array).
template <std::size_t N>
[[nodiscard]] inline std::string_view textOf(const char (&s)[N]) {
    std::size_t n = 0;
    while (n < N && s[n] != '\0') {
        ++n;
    }
    return std::string_view{s, n};
}

/// What a robot's description is made from.
struct RobotSource {
    const char* id;                                  ///< "one", "two", "x", "y", "z"
    const char* name;                                ///< "robot one" ...
    const shulib::mechanism::MechanismTable* table;  ///< its REAL list
    bool tank;                                       ///< no strafe authority (both 2026 robots: HA-252)
    /// Robot one's facts ledger: level i's name (nullptr past the last) — so a preset that IS a ledger
    /// level is written `m[facts::lv(i).name]`. Empty for a robot with no ledger.
    std::function<const char*(std::size_t)> ledgerLevel{};
    /// The ledger's levels are still OWED (placeholders, not measured): each is described `"owed": true`, and the
    /// page shows "not measured yet" in place of its height — a placeholder is never shown as a fact.
    bool ledgerOwed = false;
    /// RMP1-O: the claw's grab spot, inches inside the front edge (the facts ledger's kClawGrabInsetIn); none: no claw.
    std::optional<double> clawInsetIn{};
};

/// The square the page draws for every robot (both 2026 robots are 15 in).
inline constexpr double kFootprintIn = 15;

/// The description (file banner), one JSON object, keys in a fixed order, ending in a newline.
[[nodiscard]] inline std::string describe(const RobotSource& r) {
    namespace m = shulib::mechanism;
    const shulib::motion::MotionConfig cfg{};
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    std::string j = "{\n";
    j += "  \"id\": " + str(r.id) + ",\n";
    j += "  \"name\": " + str(r.name) + ",\n";
    j += "  \"api\": " + str(shulib::kApiVersionString) + ",\n";
    j += "  \"autonNameMaxChars\": " + std::to_string(shulib::competition::kMaxAutonNameChars) + ",\n";
    // RMP1-C: a mirrored name's budget, the menu's size, the pair check's numbers and the skills mirror (cos 2a, sin 2a
    // of its line's heading a: the image of the point (1, 0))
    j += "  \"mirroredAutonNameMaxChars\": " + std::to_string(shulib::competition::kMaxMirroredAutonNameChars) + ",\n";
    j += "  \"maxAutons\": " + std::to_string(shulib::competition::kMaxAutons) + ",\n";
    j += "  \"otherAutons\": [";
    for (std::size_t i = 0; i < kOtherAutonNames.size(); ++i) {
        j += (i == 0 ? "" : ", ") + str(kOtherAutonNames[i]);
    }
    j += "],\n";
    j += "  \"pairCheck\": {\"gapIn\": " + num(mirror_pair::kMirrorGapIn) + ", \"squareIn\": " + num(mirror_pair::kRobotSquareIn) + "},\n";
    {
        using namespace shulib::units::literals;
        const auto image = shulib::competition::override2026::kSkillsMirror.reflection.apply(1.0_in, 0.0_in);
        j += "  \"skillsMirror\": {\"cos2a\": " + num(image.x.value()) + ", \"sin2a\": " + num(image.y.value()) + "},\n";
    }
    j += "  \"drive\": {\"tank\": " + std::string{r.tank ? "true" : "false"} + ", \"footprintIn\": " + num(kFootprintIn) + ", \"maxSteerDeg\": "
         + num(cfg.tankSteer.maxSteer * kRadToDeg) + ", \"curve\": {\"maxStartOffDeg\": " + num(cfg.curve.maxStartOff * kRadToDeg)
         + ", \"lookAheadIn\": " + num(cfg.curve.lookAhead) + ", \"minSpacingIn\": " + num(shulib::motion::kCurveMinSpacing)
         + ", \"maxPoints\": " + std::to_string(shulib::motion::kMaxCurvePoints) + "}},\n";
    // RMP1-O: the claw's spot from the centre (forward, left), inches. It assumes the tracking centre is the square's
    // middle, 7.5 in behind the front edge (HA-260: unmeasured) -- d in off, every pin placed by the cross misses by d
    if (r.clawInsetIn) {
        j += "  \"clawSpot\": {\"forwardIn\": " + num(kFootprintIn / 2 - *r.clawInsetIn) + ", \"sideIn\": 0},\n";
    }
    j += "  \"mechanisms\": [";
    const auto& t = *r.table;
    for (std::size_t i = 0; i < t.count; ++i) {
        const m::MechanismSpec& s = t.specs[i];
        const std::string ident = identAt(s.declaredFile, s.declaredLine);
        j += i == 0 ? "\n" : ",\n";
        j += "    {\"name\": " + str(m::nameOf(s)) + ", \"ident\": " + (ident.empty() ? std::string{"null"} : str(ident));
        switch (s.kind) {
            case m::Kind::Position: {
                const bool arm = s.position.units == m::Units::Degrees;
                j += ", \"kind\": " + str(arm ? "arm" : "lift") + ", \"units\": " + str(arm ? "deg" : "in");
                j += ", \"homing\": " + str(homingWord(s.position.home));
                if (s.position.home != m::HomeMethod::None) {
                    j += ", \"homeAt\": " + num(s.position.homePosition);  // RMP1-C: a home's planned height
                }
                j += ", \"range\": [" + num(s.position.minPosition) + ", " + num(s.position.maxPosition) + "]";
                j += ", \"unhomedVolts\": " + num(s.position.unhomedVolts);  // RMP1-B: the controls' U5 rules
                j += ", \"presets\": [";
                for (std::size_t p = 0; p < s.position.presetCount; ++p) {
                    const std::string_view pn = textOf(s.position.presets[p].name);
                    j += p == 0 ? "" : ", ";
                    j += "{\"name\": " + str(pn) + ", \"value\": " + num(s.position.presets[p].value);
                    if (r.ledgerLevel) {
                        const char* lv = r.ledgerLevel(p);
                        if (lv != nullptr && std::string_view{lv} == pn) {
                            j += ", \"ledger\": " + std::to_string(p);
                            if (r.ledgerOwed) {
                                j += ", \"owed\": true";
                            }
                        }
                    }
                    j += "}";
                }
                j += "]";
                break;
            }
            case m::Kind::Speed: {
                j += ", \"kind\": \"flywheel\", \"presets\": [";
                for (std::size_t p = 0; p < s.speed.presetCount; ++p) {
                    j += p == 0 ? "" : ", ";
                    j += "{\"name\": " + str(textOf(s.speed.presets[p].name)) + ", \"value\": " + num(s.speed.presets[p].value) + "}";
                }
                j += "]";
                break;
            }
            case m::Kind::Power:
                j += ", \"kind\": \"power\", \"pieceSensor\": "
                     + std::string{s.pieceSensor.kind != m::SensorKind::None ? "true" : "false"};
                break;
            case m::Kind::Pneumatic: {
                j += ", \"kind\": \"pneumatic\", \"positions\": [";
                for (std::size_t p = 0; p < s.pneumatic.positionCount; ++p) {
                    j += p == 0 ? "" : ", ";
                    j += str(textOf(s.pneumatic.positions[p].name));
                }
                j += "]";
                break;
            }
        }
        j += "}";
    }
    j += t.count ? "\n  ],\n" : "],\n";
    namespace walls = shulib::localization::override2026;
    namespace el = shulib::vision::override2026;
    j += "  \"field\": {\"wallInnerFaceIn\": " + num(walls::kWallInnerFace.value()) + ", \"centreGoal\": {\"x\": 0, \"y\": 0}, \"toggles\": [";
    const char* toggleNames[4] = {"right wall", "left wall", "blue wall", "red wall"};
    for (std::size_t i = 0; i < 4; ++i) {
        j += i == 0 ? "" : ", ";
        j += "{\"name\": " + str(toggleNames[i]) + ", \"x\": " + num(el::kTogglePlaces[i].x) + ", \"y\": " + num(el::kTogglePlaces[i].y) + "}";
    }
    // RMP1-H: the solid structures (override_field_obstacles.hpp), each with its source and its footprint
    j += "], \"obstacles\": [";
    for (std::size_t i = 0; i < route_maker::obstacles::kObstacles.size(); ++i) {
        const auto& o = route_maker::obstacles::kObstacles[i];
        const bool goal = o.kind == route_maker::obstacles::Kind::Goal;
        j += (i == 0 ? "\n    " : ",\n    ") + std::string{"{\"id\": "} + str(o.id) + ", \"name\": " + str(o.name) + ", \"kind\": "
             + str(goal ? "goal" : "Loader") + ", \"x\": " + num(route_maker::obstacles::grid(o.x)) + ", \"y\": "
             + num(route_maker::obstacles::grid(o.y)) + ", \"overhangIn\": " + num(route_maker::obstacles::grid(o.overhang)) + ", \"source\": " + str(route_maker::obstacles::sourceOf(o.kind)) + ", \"footprint\": [";
        const auto f = route_maker::obstacles::footprint(o);
        for (std::size_t k = 0; k < f.size(); ++k) {
            j += (k == 0 ? "[" : ", [") + num(f[k][0]) + ", " + num(f[k][1]) + "]";
        }
        j += "]}";
    }
    j += "\n  ]}\n}\n";
    return j;
}

}  // namespace route_maker
