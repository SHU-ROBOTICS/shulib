#pragma once
//
// route_maker_runner.hpp — THE RUNNER (chunk RMP1-A2e, 2026-11-16; the RMP1 brief A.2, A.3.10, §1.1): the
// route builder's route, as LINES (tools/route_maker/src/model/route_lines.mjs), run through the REAL library
// on the host test plant, and its record written as JSON (schema 1 — tools/route_maker/README.md). Host only.
// One source for the native `shulib_route_maker preview` and the WebAssembly module (W9).
//
// THE SAME CALLS AS THE EXPORT (tools/route_maker/src/model/export.mjs), in its order: label, onFail, the
// call; a turn-first drive is face (faceAway) then the drive, the onFail before each; a wait is the run
// guard's pause inside then(...), named "wait <s> s"; a mechanism found by its `.name` in the robot's own
// table and a preset by its name — the goal the compile-time `m["name"]` builds. Every number is read the
// way its C++ literal is: `55.7_in` is static_cast<double>(55.7L), `-55.7_in` that literal negated, a turn
// cap's `270.5` a plain double. Every value is read into a named variable BEFORE a call (W8: the order of
// a call's arguments is unspecified, and GCC and Clang differ). A line the export could not have written is
// refused by name, as the compiler would refuse the C++.
//
// THE RUN is RPV1's previewEntry, unchanged. Through its hooks: onBuilt hands over the clock, so every page
// step is dated where it began and ended (exact attribution, A.3.10), and installs the mechanisms' tap below
// the recorder (runEntry, RMP1-E); onLoop hands over the record of a
// route that loops without letting time pass — RPV1 then ends the call with std::_Exit(1): natively the
// process, in WebAssembly only callMain (P-W1f).

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "route_maker_describe.hpp"
#include "route_maker_robots.hpp"
#include "route_preview_run.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/version.hpp"

namespace route_maker {

namespace mech = shulib::mechanism;
namespace comp = shulib::competition;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;

/// The lines' version this runner reads (route_lines.mjs's LINES_VERSION).
inline constexpr int kLinesVersion = 1;

/// A line the runner cannot run (the message names it): the program's usage error, never a run.
struct LinesError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// ── the numbers, read as their C++ literals are ─────────────────────────────────────────────────────────

/// The text is what numbers.mjs's fmt prints: an optional '-', digits, an optional '.digits'.
[[nodiscard]] inline bool isNumberText(std::string_view t) {
    std::size_t i = t.starts_with('-') ? 1 : 0;
    const std::size_t digits = i;
    while (i < t.size() && t[i] >= '0' && t[i] <= '9') {
        ++i;
    }
    if (i == digits) {
        return false;
    }
    if (i < t.size() && t[i] == '.') {
        const std::size_t frac = ++i;
        while (i < t.size() && t[i] >= '0' && t[i] <= '9') {
            ++i;
        }
        if (i == frac) {
            return false;
        }
    }
    return i == t.size();
}
/// The magnitude of a unit literal (`55.7_in`: the long double 55.7L, then the operator's cast).
[[nodiscard]] inline double literalMagnitude(const std::string& t) {
    return static_cast<double>(std::strtold(t.c_str() + (t[0] == '-' ? 1 : 0), nullptr));
}
/// A quantity `t_unit` as the C++ reads it: the literal, negated when written with a '-'.
template <class Q>
[[nodiscard]] Q quantity(const std::string& t) {
    const Q q{literalMagnitude(t)};
    return t[0] == '-' ? -q : q;
}
[[nodiscard]] inline shulib::math::Angle degreesLiteral(const std::string& t) {
    const shulib::math::Angle a = shulib::math::Angle::degrees(literalMagnitude(t));
    return t[0] == '-' ? -a : a;
}

// ── the parsed route ───────────────────────────────────────────────────────────────────────────────────

/// One step's words (key -> value, names %-decoded) and its curve points (each "x,y" or "x,y,h").
struct StepWords {
    std::string kind;
    std::map<std::string, std::string> kv;
    std::vector<std::vector<std::string>> points;
    int mech = -1;  ///< its mechanism's index in the robot's table
    [[nodiscard]] bool has(const char* k) const { return kv.count(k) != 0; }
    [[nodiscard]] const std::string& at(const char* k) const { return kv.at(k); }
};

/// The route the lines describe.
struct Route {
    std::string robot;
    std::string name;
    std::string park;
    std::string stop;
    bool haveStart = false;
    bool relative = true;
    std::string sx, sy, sh;  ///< a FIELD start's numbers
    std::vector<StepWords> steps;
};

/// A name's %XX decoded.
[[nodiscard]] inline std::string decoded(const std::string& s) {
    std::string out;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '%') {
            out += s[i];
            continue;
        }
        if (i + 2 >= s.size() || std::isxdigit(static_cast<unsigned char>(s[i + 1])) == 0 ||
            std::isxdigit(static_cast<unsigned char>(s[i + 2])) == 0) {
            throw LinesError{"a bad %-escape in \"" + s + "\""};
        }
        out += static_cast<char>(std::strtol(s.substr(i + 1, 2).c_str(), nullptr, 16));
        i += 2;
    }
    return out;
}

/// What each step kind carries (beside label, fail and skipTo, which every step may).
struct KindRule {
    const char* kind;
    const char* need;  ///< its required keys, space-separated
    bool motion;       ///< takes the motion options (timeout speed turn tolIn tolDeg)
};
inline constexpr KindRule kKinds[] = {
    {"driveTo", "x y face", true},   {"reverseTo", "x y face", true}, {"turnTo", "h", true},
    {"face", "x y", true},           {"faceAway", "x y", true},       {"curveTo", "", true},
    {"reverseCurveTo", "", true},    {"move", "mech preset bound", false},
    {"startMove", "mech preset bound", false}, {"moveTo", "mech to bound", false},
    {"startMoveTo", "mech to bound", false}, {"set", "mech pos", false},
    {"run", "mech volts for", false}, {"startRun", "mech volts for", false},
    {"home", "mech bound", false},   {"startHome", "mech bound", false},
    {"waitUntilDone", "mech bound", false}, {"wait", "s", false},
#ifdef SHULIB_ROUTE_MAKER_TEST_STEPS
    {"testLoop", "", false},  // WG7's fixture (RPV1's looping case): only the test module accepts it
#endif
};
[[nodiscard]] inline const KindRule* ruleOf(std::string_view kind) {
    for (const KindRule& k : kKinds) {
        if (kind == k.kind) {
            return &k;
        }
    }
    return nullptr;
}

/// The index of the mechanism named `name` in `t` (a declared one), or a refusal.
[[nodiscard]] inline int mechanismNamed(const mech::MechanismTable& t, const std::string& name) {
    for (std::size_t i = 0; i < t.count; ++i) {
        if (mech::nameOf(t.specs[i]) == name && t.declared[i] != nullptr) {
            return static_cast<int>(i);
        }
    }
    throw LinesError{"this robot has no mechanism \"" + name + "\""};
}

inline void checkMechanismStep(const mech::MechanismTable& t, const StepWords& s);

/// `s` split at `sep` (empty pieces dropped between spaces).
[[nodiscard]] inline std::vector<std::string> split(std::string_view s, char sep) {
    std::vector<std::string> out;
    for (std::size_t from = 0; from <= s.size();) {
        const std::size_t to = std::min(s.find(sep, from), s.size());
        if (to > from || sep != ' ') {
            out.emplace_back(s.substr(from, to - from));
        }
        from = to + 1;
    }
    return out;
}

/// A number word's text, refused unless it is one.
[[nodiscard]] inline const std::string& numberAt(const StepWords& w, const char* k) {
    if (!w.has(k) || !isNumberText(w.at(k))) {
        throw LinesError{std::string{"\""} + k + "\" needs a number"};
    }
    return w.at(k);
}

/// One line's words after its head: `key=value`, a repeated `p=x,y[,h]` collecting curve points.
[[nodiscard]] inline StepWords wordsOf(const std::vector<std::string>& words) {
    StepWords w;
    for (std::size_t i = 1; i < words.size(); ++i) {
        const std::size_t eq = words[i].find('=');
        if (eq == std::string::npos || eq == 0) {
            throw LinesError{"\"" + words[i] + "\" is not key=value"};
        }
        const std::string k = words[i].substr(0, eq);
        const std::string v = words[i].substr(eq + 1);
        if (k == "p") {
            const std::vector<std::string> p = split(v, ',');
            if ((p.size() != 2 && p.size() != 3) || !std::all_of(p.begin(), p.end(), [](const auto& c) { return isNumberText(c); })) {
                throw LinesError{"a curve point is x,y or x,y,heading: \"" + v + "\""};
            }
            w.points.push_back(p);
        } else if (!w.kv.emplace(k, v).second) {
            throw LinesError{"\"" + k + "\" given twice"};
        }
    }
    return w;
}

/// A step's words checked against its kind's rule and the robot's table (what the compiler checks in the export).
[[nodiscard]] inline StepWords stepOf(StepWords w, const mech::MechanismTable& t) {
    w.kind = w.has("do") ? w.at("do") : "";
    w.kv.erase("do");
    const KindRule* rule = ruleOf(w.kind);
    if (rule == nullptr) {
        throw LinesError{"the engine cannot run a \"" + w.kind + "\" step"};
    }
    std::size_t known = 0;
    for (const std::string& k : split(rule->need, ' ')) {
        if (!w.has(k.c_str())) {
            throw LinesError{"a " + w.kind + " step needs \"" + k + "\""};
        }
        ++known;
    }
    for (const char* k : {"label", "fail", "skipTo"}) {
        known += w.has(k) ? 1U : 0U;
    }
    for (const char* k : {"timeout", "speed", "turn", "tolIn", "tolDeg"}) {
        if (w.has(k) && rule->motion) {
            (void)numberAt(w, k);
            ++known;
        }
    }
    if (known != w.kv.size()) {
        throw LinesError{"a " + w.kind + " step does not take one of its words"};
    }
    for (const char* k : {"x", "y", "h", "to", "bound", "volts", "for", "s"}) {
        if (w.has(k)) {
            (void)numberAt(w, k);
        }
    }
    if ((w.has("fail") && (w.at("fail") != "carryOn" || w.has("skipTo"))) ||
        (w.has("face") && w.at("face") != "0" && w.at("face") != "1")) {
        throw LinesError{"fail is carryOn (or skipTo=<label>, not both); face is 0 or 1"};
    }
    if ((w.kind == "curveTo" || w.kind == "reverseCurveTo") == w.points.empty()) {
        throw LinesError{"a curve, and only a curve, has points"};
    }
    for (const char* k : {"mech", "preset", "pos", "label", "skipTo"}) {
        if (w.has(k)) {
            w.kv[k] = decoded(w.at(k));
        }
    }
    if (w.has("mech")) {
        w.mech = mechanismNamed(t, w.at("mech"));
        checkMechanismStep(t, w);
    }
    return w;
}

/// Parse the lines (route_lines.mjs's format), every step checked before anything runs: what the compiler would
/// refuse in the export, the runner refuses by name.
[[nodiscard]] inline Route parseLines(const std::string& text) {
    Route r;
    std::optional<RobotSource> src;
    const std::vector<std::string> lines = split(text, '\n');
    for (std::size_t n = 0; n < lines.size(); ++n) {
        const std::vector<std::string> words = split(lines[n], ' ');
        const std::string head = words.empty() ? "" : words[0];
        try {
            if (n == 0) {
                if (lines[0] != "shulib-route " + std::to_string(kLinesVersion)) {
                    throw LinesError{"not route lines of version " + std::to_string(kLinesVersion)};
                }
            } else if (head == "route" && !src) {
                StepWords w = wordsOf(words);
                src = robotSource(w.has("robot") ? w.at("robot") : "");
                if (!src || !w.has("name") || w.kv.size() != 4) {
                    throw LinesError{"a route line is: route robot=one|two|x|y|z name=.. park=.. stop=.."};
                }
                r.robot = src->id;
                r.name = decoded(w.at("name"));
                r.park = numberAt(w, "park");
                r.stop = numberAt(w, "stop");
            } else if (head == "start" && src && !r.haveStart) {
                r.haveStart = true;
                if (lines[n] == "start relative") {
                    continue;
                }
                StepWords w = wordsOf(words);
                r.relative = false;
                r.sx = numberAt(w, "x");
                r.sy = numberAt(w, "y");
                r.sh = numberAt(w, "h");
                if (w.kv.size() != 3) {
                    throw LinesError{"a start is: start x=.. y=.. h=.. (or: start relative)"};
                }
            } else if (head == "step" && r.haveStart) {
                r.steps.push_back(stepOf(wordsOf(words), *src->table));
            } else if (!lines[n].empty()) {
                throw LinesError{"expected " + std::string{!src ? "the route line" : !r.haveStart ? "a start" : "a step"}};
            }
        } catch (const LinesError& e) {
            throw LinesError{"line " + std::to_string(n + 1) + ": " + e.what()};
        }
    }
    if (!r.haveStart) {
        throw LinesError{"the lines end before the route's start"};
    }
    return r;
}

// ── one run's state (the routine is a plain function pointer; one preview at a time) ──────────────────────

/// Where one page step began and ended, its brain's step numbers and its verdict (A.3.10).
struct PageStep {
    double t0 = -1.0;  ///< the clock when it began (-1: never reached)
    double t1 = -1.0;  ///< the clock when it ended (-1: it never returned)
    int libFrom = 0;   ///< the brain's log's first step number for it
    int libTo = 0;     ///< ... and its last
    std::string verdict = "notReached";
};
/// THE MECHANISMS (RMP1-E): each one's state every pace, read from the library's own slots below the recorder, so
/// it records when the recorder does (once at the start, once after each pace). A lift's or an arm's position in its
/// units; a pneumatic's position index (-1: none); a power mechanism's or a flywheel's direction (-1, 0, 1).
struct MechTap final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    const mech::MechanismService* svc = nullptr;
    std::vector<std::vector<double>> at;  ///< per slot, per pace
    void record() {
        at.resize(svc->size());
        for (std::size_t i = 0; i < svc->size(); ++i) {
            at[i].push_back(stateOf(*svc->slotPtr(i)));
        }
    }
    void pace() override {
        inner->pace();
        record();
    }
    [[nodiscard]] static double stateOf(const mech::Slot& s) {
        const auto sign = [](double v) { return v > 0.0 ? 1.0 : v < 0.0 ? -1.0 : 0.0; };
        switch (s.kind()) {
            case mech::Kind::Position: return static_cast<const mech::PositionSlot&>(s).position();
            case mech::Kind::Power: return sign(static_cast<const mech::PowerSlot&>(s).commandedVolts());
            case mech::Kind::Speed: return sign(static_cast<const mech::SpeedSlot&>(s).targetRpm());
            case mech::Kind::Pneumatic: {
                const std::uint8_t p = static_cast<const mech::PneumaticSlot&>(s).commandedPosition();
                return p == mech::PneumaticSlot::kNoPosition ? -1.0 : p;
            }
        }
        return 0.0;
    }
};

struct RunState {
    Route route;
    const mech::MechanismTable* table = nullptr;
    tester_rig::TesterStack* stack = nullptr;
    std::vector<PageStep> page;
    std::deque<std::string> keep;  ///< the names a Routine stores by pointer, alive for the run
    MechTap tap;
};
inline RunState& state() {
    static RunState s;
    return s;
}

// ── the calls (export.mjs's, word for word) ──────────────────────────────────────────────────────────────

[[nodiscard]] inline shulib::chassis::MotionOptions optionsOf(const StepWords& s) {
    shulib::chassis::MotionOptions o{};
    if (s.has("timeout")) {
        o.timeout = quantity<shulib::units::Time>(s.at("timeout"));
    }
    if (s.has("speed")) {  // `N_in / 1_s`
        o.maxLinearSpeed = quantity<shulib::units::Length>(s.at("speed")) / shulib::units::Time{1.0};
    }
    if (s.has("turn")) {  // `units::AngularVelocity{N * math::Angle::kPi / 180.0}`: N a plain C++ number
        const double n = std::strtod(s.at("turn").c_str(), nullptr);
        o.maxAngularSpeed = shulib::units::AngularVelocity{n * shulib::math::Angle::kPi / 180.0};
    }
    if (s.has("tolIn")) {
        o.positionTolerance = quantity<shulib::units::Length>(s.at("tolIn"));
    }
    if (s.has("tolDeg")) {
        o.headingTolerance = degreesLiteral(s.at("tolDeg"));
    }
    return o;
}

/// The declaration at table index `i`, as kind D (nullptr when it is another kind).
template <class D>
[[nodiscard]] const D* declAs(const mech::MechanismTable& t, int i) {
    const mech::MechanismSpec& s = t.specs[static_cast<std::size_t>(i)];
    bool ok = false;
    if constexpr (std::is_same_v<D, mech::Arm>) {
        ok = s.kind == mech::Kind::Position && s.position.units == mech::Units::Degrees;
    } else if constexpr (std::is_same_v<D, mech::Lift>) {
        ok = s.kind == mech::Kind::Position && s.position.units == mech::Units::Inches;
    } else if constexpr (std::is_same_v<D, mech::Flywheel>) {
        ok = s.kind == mech::Kind::Speed;
    } else if constexpr (std::is_same_v<D, mech::Power>) {
        ok = s.kind == mech::Kind::Power;
    } else {
        ok = s.kind == mech::Kind::Pneumatic;
    }
    return ok ? static_cast<const D*>(t.declared[static_cast<std::size_t>(i)]) : nullptr;
}

/// `m["preset"]` for an arm or a lift: the goal its consteval operator[] builds.
[[nodiscard]] inline mech::PositionGoal presetGoal(const mech::MechanismTable& t, const StepWords& s) {
    const std::string& want = s.at("preset");
    if (const mech::Arm* a = declAs<mech::Arm>(t, s.mech)) {
        for (const mech::DegreePreset& p : a->presets) {
            if (p.name != nullptr && want == p.name) {
                return mech::PositionGoal{a, a->name.text, p.name, p.value, false, mech::Units::Degrees};
            }
        }
    } else if (const mech::Lift* l = declAs<mech::Lift>(t, s.mech)) {
        for (const mech::InchPreset& p : l->presets) {
            if (p.name != nullptr && want == p.name) {
                return mech::PositionGoal{l, l->name.text, p.name, p.value.value(), false, mech::Units::Inches};
            }
        }
    } else {
        throw LinesError{"\"" + s.at("mech") + "\" has no presets: it is not an arm or a lift"};
    }
    throw LinesError{"no_such_preset_on_this_mechanism: \"" + s.at("mech") + "\" has no preset \"" + want + "\""};
}

/// `m.to(<number>)` (RMP1-C D7): an arm's plain degrees (`arm.to(120)`, a double literal) or a lift's inches
/// (`lift.to(18_in)`) — the goal its consteval to() builds, refused past the travel as the build refuses it.
[[nodiscard]] inline mech::PositionGoal toGoal(const mech::MechanismTable& t, const StepWords& s) {
    const std::string& v = s.at("to");
    const auto refuse = [&] {
        throw LinesError{"target_is_outside_the_travel_limits: \"" + s.at("mech") + "\" cannot go to " + v};
    };
    if (const mech::Arm* a = declAs<mech::Arm>(t, s.mech)) {
        const double deg = std::strtod(v.c_str(), nullptr);
        if (!(deg >= a->range.min && deg <= a->range.max)) {
            refuse();
        }
        return mech::PositionGoal{a, a->name.text, nullptr, deg, false, mech::Units::Degrees};
    }
    if (const mech::Lift* l = declAs<mech::Lift>(t, s.mech)) {
        const double in = quantity<shulib::units::Length>(v).value();
        if (!(in >= l->range.min.value() && in <= l->range.max.value())) {
            refuse();
        }
        return mech::PositionGoal{l, l->name.text, nullptr, in, false, mech::Units::Inches};
    }
    throw LinesError{"\"" + s.at("mech") + "\" is not an arm or a lift: it has no height to go to"};
}

/// `claw["position"]`.
[[nodiscard]] inline mech::PneumaticGoal positionGoal(const mech::MechanismTable& t, const StepWords& s) {
    const mech::Pneumatic* p = declAs<mech::Pneumatic>(t, s.mech);
    if (p == nullptr) {
        throw LinesError{"\"" + s.at("mech") + "\" is not a pneumatic"};
    }
    for (std::size_t i = 0; i < p->positions.count; ++i) {
        if (p->positions.p[i].name != nullptr && s.at("pos") == p->positions.p[i].name) {
            return mech::PneumaticGoal{p, p->name.text, static_cast<std::uint8_t>(i), p->positions.p[i].name};
        }
    }
    throw LinesError{"no_such_position_on_this_mechanism: \"" + s.at("mech") + "\" has no position \"" + s.at("pos") +
                     "\""};
}

/// Call `f` with the arm or lift at `s.mech` (home, startHome), or refuse.
template <class F>
void withArmOrLift(const mech::MechanismTable& t, const StepWords& s, F&& f) {
    if (const mech::Arm* a = declAs<mech::Arm>(t, s.mech)) {
        f(*a);
    } else if (const mech::Lift* l = declAs<mech::Lift>(t, s.mech)) {
        f(*l);
    } else {
        throw LinesError{"\"" + s.at("mech") + "\" is not an arm or a lift: it has no home"};
    }
}

/// What the compiler checks in the export's mechanism step: the kind fits the call, the preset or position exists.
inline void checkMechanismStep(const mech::MechanismTable& t, const StepWords& s) {
    const std::string& k = s.kind;
    if (k == "move" || k == "startMove") {
        (void)presetGoal(t, s);
    } else if (k == "moveTo" || k == "startMoveTo") {
        (void)toGoal(t, s);
    } else if (k == "set") {
        (void)positionGoal(t, s);
    } else if ((k == "run" || k == "startRun") && declAs<mech::Power>(t, s.mech) == nullptr) {
        throw LinesError{"\"" + s.at("mech") + "\" is not a power mechanism: it cannot run for a time"};
    } else if (k == "home" || k == "startHome") {
        withArmOrLift(t, s, [](const auto&) {});
    }
}

/// The step's own library calls (after its label and onFail; checked by checkMechanismStep).
inline void callStep(Routine& r, comp::AutonContext& ctx, const StepWords& s, const mech::MechanismTable& t) {
    using shulib::units::Length;
    using shulib::units::Time;
    using shulib::units::Voltage;
    const std::string& k = s.kind;
    if (k == "driveTo" || k == "reverseTo" || k == "face" || k == "faceAway") {
        const Length x = quantity<Length>(s.at("x"));
        const Length y = quantity<Length>(s.at("y"));
        const shulib::chassis::MotionOptions o = optionsOf(s);
        if (k == "face") {
            r.face(x, y, o);
        } else if (k == "faceAway") {
            r.faceAway(x, y, o);
        } else if (k == "driveTo") {
            r.driveTo(x, y, o);
        } else {
            r.reverseTo(x, y, o);
        }
    } else if (k == "turnTo") {
        const shulib::math::Angle h = degreesLiteral(s.at("h"));
        r.turnTo(h, optionsOf(s));
    } else if (k == "curveTo" || k == "reverseCurveTo") {
        std::vector<shulib::motion::CurvePoint> pts;
        for (const std::vector<std::string>& p : s.points) {
            const Length x = quantity<Length>(p[0]);
            const Length y = quantity<Length>(p[1]);
            if (p.size() == 3) {
                const shulib::math::Angle h = degreesLiteral(p[2]);
                pts.emplace_back(x, y, h);
            } else {
                pts.emplace_back(x, y);
            }
        }
        const std::span<const shulib::motion::CurvePoint> span{pts};
        k == "curveTo" ? r.curveTo(span, optionsOf(s)) : r.reverseCurveTo(span, optionsOf(s));
    } else if (k == "move" || k == "startMove") {
        const mech::PositionGoal g = presetGoal(t, s);
        const Time bound = quantity<Time>(s.at("bound"));
        k == "move" ? r.move(g, bound) : r.startMove(g, bound);
    } else if (k == "moveTo" || k == "startMoveTo") {
        const mech::PositionGoal g = toGoal(t, s);
        const Time bound = quantity<Time>(s.at("bound"));
        k == "moveTo" ? r.move(g, bound) : r.startMove(g, bound);
    } else if (k == "set") {
        r.set(positionGoal(t, s));
    } else if (k == "run" || k == "startRun") {
        const mech::Power* p = declAs<mech::Power>(t, s.mech);
        const Voltage v = quantity<Voltage>(s.at("volts"));
        const Time f = quantity<Time>(s.at("for"));
        k == "run" ? r.run(*p, v, f) : r.startRun(*p, v, f);
    } else if (k == "home" || k == "startHome") {
        const Time bound = quantity<Time>(s.at("bound"));
        withArmOrLift(t, s, [&](const auto& m) { k == "home" ? r.home(m, bound) : r.startHome(m, bound); });
    } else if (k == "waitUntilDone") {
        const Time bound = quantity<Time>(s.at("bound"));
        if (const auto* a = declAs<mech::Arm>(t, s.mech)) {
            r.waitUntilDone(*a, bound);
        } else if (const auto* l = declAs<mech::Lift>(t, s.mech)) {
            r.waitUntilDone(*l, bound);
        } else if (const auto* f = declAs<mech::Flywheel>(t, s.mech)) {
            r.waitUntilDone(*f, bound);
        } else if (const auto* p = declAs<mech::Power>(t, s.mech)) {
            r.waitUntilDone(*p, bound);
        } else {
            r.waitUntilDone(*declAs<mech::Pneumatic>(t, s.mech), bound);
        }
    } else if (k == "wait") {
        const Time sec = quantity<Time>(s.at("s"));
        const char* name = state().keep.emplace_back("wait " + s.at("s") + " s").c_str();
        r.then([&ctx, sec] { return ctx.guard.pause(sec) == shulib::sequence::GuardedWaitResult::Satisfied; }, name);
#ifdef SHULIB_ROUTE_MAKER_TEST_STEPS
    } else if (k == "testLoop") {  // RPV1's looping case: a drive that times out stops the route, then forever
        r.driveTo(Length{60.0}, Length{60.0}, {.timeout = Time{0.05}});
        for (;;) {
            r.driveTo(Length{0.0}, Length{0.0});
        }
#endif
    }
}

/// How the page step that ran `calls` library calls between results `b` and `a` ended.
[[nodiscard]] inline std::string verdictOf(const RoutineResult& b, const RoutineResult& a, int calls,
                                           const shulib::sequence::RunGuard& guard) {
    if (b.stoppedAt == 0 && a.stoppedAt != 0) {
        return guard.expired() ? "cut" : "failed";
    }
    if (a.failedOn > b.failedOn) {
        return "failedWentOn";
    }
    if (a.jumped - b.jumped == calls) {
        return "jumped";
    }
    return a.completed - b.completed == calls ? "ok" : "skipped";
}

/// THE ROUTINE the runner registers: every page step, dated and judged.
inline RoutineResult runRoute(comp::AutonContext& ctx) {
    RunState& g = state();
    Routine r{ctx, g.route.name.c_str()};
    for (std::size_t i = 0; i < g.route.steps.size(); ++i) {
        const StepWords& s = g.route.steps[i];
        PageStep& p = g.page[i];
        const bool twoCalls = (s.kind == "driveTo" || s.kind == "reverseTo") && s.at("face") == "1";
        const RoutineResult before = r.result();
        p.t0 = g.stack->now();
        try {
            if (s.has("label")) {
                r.label(g.keep.emplace_back(s.at("label")).c_str());
            }
            const auto onFail = [&] {
                if (s.has("fail")) {
                    r.onFail(shulib::chassis::carryOn);
                } else if (s.has("skipTo")) {
                    r.onFail(shulib::chassis::skipTo(g.keep.emplace_back(s.at("skipTo")).c_str()));
                }
            };
            onFail();
            if (twoCalls) {
                const shulib::units::Length x = quantity<shulib::units::Length>(s.at("x"));
                const shulib::units::Length y = quantity<shulib::units::Length>(s.at("y"));
                s.kind == "driveTo" ? r.face(x, y) : r.faceAway(x, y);
                onFail();
            }
            callStep(r, ctx, s, *g.table);
        } catch (...) {
            p.t1 = g.stack->now();
            p.verdict = "threw";
            throw;  // as the export: the tester catches it (PROGRAM ERROR)
        }
        const RoutineResult after = r.result();
        p.t1 = g.stack->now();
        p.libFrom = before.steps + 1;
        p.libTo = after.steps;
        p.verdict = verdictOf(before, after, twoCalls ? 2 : 1, ctx.guard);
    }
    return r.result();
}

inline void registerRoute(comp::AutonRegistry& reg) {
    const Route& rt = state().route;
    comp::AutonEntry e{};
    e.name = rt.name.c_str();
    e.kind = comp::AutonKind::Skills;
    e.routine = &runRoute;
    if (!rt.relative) {
        const auto x = quantity<shulib::units::Length>(rt.sx);
        const auto y = quantity<shulib::units::Length>(rt.sy);
        const auto h = degreesLiteral(rt.sh);
        e.start = comp::StartPose::field(x, y, h);
    } else {
        e.start = comp::StartPose::relative();
    }
    e.endActionAt = quantity<shulib::units::Time>(rt.park);
    e.hardStopAt = quantity<shulib::units::Time>(rt.stop);
    reg.add(e);
}

// ── the robots (RPV1's own rows for one and two; the fixture robots on fakes by kind) ──────────────────────

/// The preview row for robot `id`, its autons replaced by `reg`.
[[nodiscard]] inline route_preview::PreviewRobot previewRobot(const std::string& id, route_preview::RegisterFn reg) {
    route_preview::PreviewRobot p{};
    if (id == "one") {
        p = route_preview::kRobotOne;
    } else if (id == "two") {
        p = route_preview::kRobotTwo;
    } else {
        const std::optional<RobotSource> s = robotSource(id);
        p = route_preview::PreviewRobot{.key = s->id, .title = s->name, .table = s->table};
    }
    p.registerAutons = reg;
    p.autonsFile = "the route builder";
    return p;
}

/// RPV1's previewEntry (entry 0) with the mechanisms tapped (RMP1-E); a test's own onBuilt runs after, given the
/// layer below the tap (the fakes' world).
[[nodiscard]] inline route_preview::Model runEntry(const route_preview::PreviewRobot& robot, const comp::AutonRegistry& reg,
                                                   route_preview::PreviewOptions opt = {}) {
    opt.onBuilt = [then = opt.onBuilt](tester_rig::TesterStack& s, shulib::motion::ITickPacer*& below) {
        MechTap& tap = state().tap;
        state().stack = &s;
        tap = MechTap{};
        tap.inner = below;
        tap.svc = &s.mechanisms;
        below = &tap;
        tap.record();
        if (then) {
            then(s, tap.inner);
        }
    };
    return route_preview::previewEntry(robot, reg, 0, opt);
}

// ── THE RECORD (schema 1 — tools/route_maker/README.md) ─────────────────────────────────────────────────────

/// A JSON string: UTF-8 kept as it is; '"', '\\' and control characters escaped.
[[nodiscard]] inline std::string jstr(std::string_view s) {
    std::string o = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') {
            o += '\\';
            o += c;
        } else if (static_cast<unsigned char>(c) < 0x20) {
            char e[8];
            std::snprintf(e, sizeof e, "\\u%04x", static_cast<unsigned>(c));
            o += e;
        } else {
            o += c;
        }
    }
    return o + "\"";
}
[[nodiscard]] inline const char* jbool(bool b) { return b ? "true" : "false"; }

/// The model sample at robot-clock time `clock` (foldRun's own rule for a motion's start and end).
[[nodiscard]] inline std::size_t sampleAt(const route_preview::Model& m, double clock) {
    std::size_t i = 0;
    while (i + 1 < m.samples.size() && m.samples[i].t + m.armedAt < clock - 0.005) {
        ++i;
    }
    return i;
}

/// The record's `mechanisms` (RMP1-E): each one in `robot`'s list, in its order, and its state at every sample —
/// `at[k]` is `samples[k]`'s pace (the record's samples are the recorder's last `n`; so are the tap's).
[[nodiscard]] inline std::string mechanismsOf(const route_preview::PreviewRobot& robot, const MechTap* tap, std::size_t n) {
    const mech::MechanismTable& t = *robot.table;
    std::string o = "[";
    for (std::size_t i = 0; i < t.count && i < mech::kMaxMechanisms; ++i) {
        const mech::MechanismSpec& s = t.specs[i];
        std::string kind = s.kind == mech::Kind::Power ? "power" : s.kind == mech::Kind::Speed ? "flywheel" : "pneumatic";
        std::string more;
        if (s.kind == mech::Kind::Position) {
            const bool lift = s.position.units == mech::Units::Inches;
            kind = lift ? "lift" : "arm";
            more = ", \"units\": " + jstr(lift ? "in" : "deg") + ", \"bottom\": " + num(s.position.minPosition) +
                   ", \"top\": " + num(s.position.maxPosition);
        } else if (s.kind == mech::Kind::Pneumatic) {
            more = ", \"positions\": [";
            for (std::size_t p = 0; p < s.pneumatic.positionCount; ++p) {
                more += (p == 0 ? "" : ", ") + jstr(s.pneumatic.positions[p].name);
            }
            more += "]";
        }
        std::string at = "[";
        if (tap != nullptr && i < tap->at.size() && tap->at[i].size() >= n) {
            const std::vector<double>& v = tap->at[i];
            for (std::size_t k = v.size() - n; k < v.size(); ++k) {
                at += (k == v.size() - n ? "" : ", ") + num(v[k]);
            }
        }
        o += std::string{i == 0 ? "" : ",\n  "} + "{\"name\": " + jstr(mech::nameOf(s)) + ", \"kind\": " + jstr(kind) +
             more + ", \"placeholder\": " + jstr(route_preview::placeholderFor(robot, std::string{mech::nameOf(s)})) + ", \"at\": " +
             at + "]}";
    }
    return o + "]";
}

/// The record, one top-level member per line; `page` null for a run with no page steps (WG1's compiled twin).
/// `mechs` is mechanismsOf's text.
[[nodiscard]] inline std::string record(const route_preview::Model& m, const std::vector<PageStep>* page,
                                        const std::vector<StepWords>* kinds, const std::string& mechs) {
    std::string o = "{\"schema\": 1,\n";
    const auto member = [&o](const char* k, const std::string& v, bool last = false) {
        o += std::string{"\""} + k + "\": " + v + (last ? "\n" : ",\n");
    };
    const auto list = [](const std::vector<std::string>& v) {
        std::string s = "[";
        for (std::size_t i = 0; i < v.size(); ++i) {
            s += (i == 0 ? "" : ", ") + jstr(v[i]);
        }
        return s + "]";
    };
    member("api", jstr(shulib::kApiVersionString));
    member("robot", jstr(m.robotKey));
    member("robotLine", jstr(m.robotLine));
    member("owed", list(m.owed));
    member("ledgerFile", jstr(m.ledgerFile));
    member("auton", jstr(m.auton));
    member("kind", jstr(m.kindWord));
    member("period", num(m.period));
    member("park", num(m.park));
    member("stop", num(m.stop));
    member("start", jstr(m.startLine));
    member("notices", list(m.notices));
    member("word", jstr(m.word));
    member("refused", jbool(m.refused));
    member("why", jstr(m.why));
    member("threw", jbool(m.threw));
    member("looped", jbool(m.looped));
    member("loopSentence", jstr(m.loopSentence));
    member("loopHead", list(m.loopHead));
    member("cut", jbool(m.cut));
    member("scoringEnded", num(m.scoringEnded));
    member("endActionEnded", num(m.endActionEnded));
    member("exit", std::to_string(m.exitCode));
    std::string ps = "null";
    if (page != nullptr) {
        ps = "[";
        for (std::size_t i = 0; i < page->size(); ++i) {
            const PageStep& p = (*page)[i];
            const bool reached = p.t0 >= 0.0 && !m.samples.empty();
            const std::size_t first = reached ? sampleAt(m, p.t0) : 0;
            const std::size_t last = !reached ? 0 : p.t1 >= 0.0 ? sampleAt(m, p.t1) : m.samples.size() - 1;
            const std::string verdict = reached && p.t1 < 0.0 ? (m.looped ? "looping" : "notReached") : p.verdict;
            ps += std::string{i == 0 ? "" : ", "} + "{\"n\": " + std::to_string(i + 1) + ", \"do\": " +
                  jstr(kinds != nullptr ? (*kinds)[i].kind : "") + ", \"verdict\": " + jstr(verdict) +
                  ", \"first\": " + (reached ? std::to_string(first) : "null") +
                  ", \"last\": " + (reached ? std::to_string(last) : "null") +
                  ", \"lib\": " + (p.libFrom > 0 && p.libTo >= p.libFrom
                                       ? "[" + std::to_string(p.libFrom) + ", " + std::to_string(p.libTo) + "]"
                                       : "null") +
                  "}";
        }
        ps += "]";
    }
    member("pageSteps", ps);
    std::string st = "[";
    for (std::size_t i = 0; i < m.steps.size(); ++i) {
        const route_preview::Step& s = m.steps[i];
        st += std::string{i == 0 ? "" : ",\n  "} + "{\"n\": " + std::to_string(s.number) + ", \"kind\": " +
              jstr(route_preview::stepKindWord(s.kind)) + ", \"what\": " + jstr(s.what) + ", \"s0\": " +
              std::to_string(s.s0) + ", \"s1\": " + std::to_string(s.s1) + ", \"t0\": " + num(s.t0) + ", \"t1\": " +
              num(s.t1) + ", \"verdict\": " + jstr(s.verdict) + ", \"mark\": " + jstr(route_preview::markWord(s.mark)) +
              ", \"sentence\": " + jstr(s.sentence) + ", \"endAction\": " + jbool(s.endAction) + ", \"background\": " +
              jbool(s.background) + ", \"openAtThrow\": " + jbool(s.openAtThrow) + ", \"placeholder\": " +
              jstr(s.placeholder) + ", \"target\": " +
              (s.hasTarget ? "[" + num(s.tx) + ", " + num(s.ty) + ", " + num(s.tth) + "]" : std::string{"null"}) + "}";
    }
    member("steps", st + "]");
    std::string ls = "[";
    for (std::size_t i = 0; i < m.allLines.size(); ++i) {
        const route_preview::LogLine& l = m.allLines[i];
        ls += std::string{i == 0 ? "" : ",\n  "} + "{\"s\": " + std::to_string(l.s) + ", \"t\": " + num(l.t) +
              ", \"sub\": " + jstr(l.subsystem) + ", \"text\": " + jstr(l.text) + ", \"mark\": " +
              jstr(route_preview::markWord(l.mark)) + "}";
    }
    member("lines", ls + "]");
    std::string fs = "[";
    for (std::size_t i = 0; i < m.field.size(); ++i) {
        const route_preview::FieldNote& f = m.field[i];
        const char* kind = f.kind == route_preview::Contact::Through ? "THROUGH"
                           : f.kind == route_preview::Contact::Touch ? "TOUCH" : "PIECE";
        fs += std::string{i == 0 ? "" : ", "} + "{\"kind\": " + jstr(kind) + ", \"element\": " + jstr(f.element) +
              ", \"s\": " + std::to_string(f.s) + ", \"t\": " + num(f.t) + ", \"step\": " + std::to_string(f.step) +
              ", \"depth\": " + num(f.depth) + "}";
    }
    member("field", fs + "]");
    member("through", jbool(m.through));
    member("throughAt", std::to_string(m.throughAt));
    member("mechanisms", mechs);
    std::string ss = "[";
    for (std::size_t i = 0; i < m.samples.size(); ++i) {
        const route_preview::Sample& s = m.samples[i];
        ss += std::string{i == 0 ? "" : ",\n  "} + "[" + num(s.t) + ", " + num(s.x) + ", " + num(s.y) + ", " +
              num(s.theta) + "]";
    }
    member("samples", ss + "]", true);
    return o + "}\n";
}

/// Write `text` to stdout whole.
inline void emit(const std::string& text) {
    std::fwrite(text.data(), 1, text.size(), stdout);
    std::fflush(stdout);
}

/// The route parsed into state() run on its robot (a test calls it as preview does): the Model, the page steps and
/// the mechanisms' tap left in state(). A registry that refuses the entry gives RPV1's refused Model, nothing run.
inline route_preview::Model runParsed(const route_preview::PreviewRobot& robot, const route_preview::PreviewOptions& opt) {
    RunState& g = state();
    g.table = robot.table;
    g.page.assign(g.route.steps.size(), PageStep{});
    comp::AutonRegistry reg{shulib::bench::kAutonPeriods, comp::OnRefusal::DropEntry};
    robot.registerAutons(reg);
    if (reg.size() != 1) {  // the registry refused the entry (its sentence is the reason)
        return route_preview::refusedModel(robot, g.route.name, reg.refusedCount() > 0 ? reg.refusal(0) : "refused", {});
    }
    return runEntry(robot, reg, opt);
}

/// The record of `m` (run by runParsed on `robot`).
[[nodiscard]] inline std::string recordOf(const route_preview::Model& m, const route_preview::PreviewRobot& robot) {
    RunState& g = state();
    return record(m, &g.page, &g.route.steps, mechanismsOf(robot, &g.tap, m.samples.size()));
}

/// PREVIEW: run the lines' route and write its record to stdout. 0 drawn, 1 a problem (refused, threw, went
/// THROUGH, looped — the record says), 2 lines it cannot run (said on stderr, no record).
inline int preview(const std::string& lines) {
    RunState& g = state();
    g = RunState{};
    try {
        g.route = parseLines(lines);
    } catch (const LinesError& e) {
        std::fprintf(stderr, "shulib_route_maker preview: %s\n", e.what());
        return 2;
    }
    const route_preview::PreviewRobot robot = previewRobot(g.route.robot, &registerRoute);
    route_preview::PreviewOptions opt;
    opt.onLoop = [&robot](const route_preview::Model& lm) {
        emit(recordOf(lm, robot));
        return true;
    };
    const route_preview::Model m = runParsed(robot, opt);
    emit(recordOf(m, robot));
    return m.exitCode;
}

}  // namespace route_maker
