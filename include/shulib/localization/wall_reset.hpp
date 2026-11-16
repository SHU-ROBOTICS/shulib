#pragma once
//
// The WALL RESET's pure core (chunk L6-A1, 2026-10-29; the L6 brief, the team lead's answers of
// 2026-10-22): the geometry that turns one side distance sensor's reading into ONE field coordinate,
// the gates that REFUSE rather than guess, the verdict, and the one log line. Nothing here moves the
// estimate or reads a clock: the calls that do are `chassis::resetFromWall` (the explicit form over
// sensor handles — chassis/wall_reset_call.hpp) and `competition::resetFromWall` (the everyday form
// over an auton's context — competition/auton_wall_reset.hpp, L6-A2). Both run THESE functions, so no gate
// is written twice.
//
// ── What a wall reset is ─────────────────────────────────────────────────────────────────────────
// A robot parked beside a field wall, nose along it and square to it, with a side sensor facing it:
// the sensor's range, from the sensor's MEASURED place on the robot, says how far the robot's centre
// is from that wall — ONE coordinate (x against the left/right walls, y against the red/blue walls).
// The reset replaces that one coordinate of the estimate (`Localizer::seatAxis`) and nothing else:
// not the other coordinate, not the heading, not the estimate's trust in itself (the drift term).
//
// ── The rules, in the order they run (the verdict names the FIRST that fails) ───────────────────
//   1. Setup — a sensor, its mount measured, the estimate live, NO motion active, the request sane
//      (the call checks the first four; `wallResetProblem` the last, naming the field).
//   2. Square — by the estimate's heading and the typed facing, the beam within `squareWithin` of
//      the wall's normal; a beam pointing AWAY is NotSquare.
//   3. Keep-out — the beam centre's coordinate ALONG the wall, widened by the beam's footprint, a
//      margin and how far the estimate may have drifted along the wall since that axis was set
//      (`alongDriftRate` × `Localizer::travelSinceSeat`), overlaps no active keep-out span.
//   4. Beam height — the expected range is at most the CEILING: past it, the cone's lower edge dips
//      below a standing game piece (6.5 in) + a margin, so the nearest surface could be a Pin or a Cup.
//   5. Stillness — on every tick, the estimate's speed over the trailing `window` (its displacement
//      over that time) within `stillSpeed`, and its turn rate |ω| within `stillTurn`.
//   6. Each sample — no read fault between its two reads, a real return (confidence > 0), enough
//      confidence, and the bias-corrected range inside [rangeMin, rangeMax].
//   7. Enough, consistent, under the ceiling — ≥ minSamples over ≥ window; spread ≤ spreadMax; the
//      MEDIAN's range ≤ the ceiling too (rule 4 judged the estimate; this judges what was seen).
//   8. Precise — the worst-case error of the answer (`axisErrorBound`) ≤ errorBoundMax.
//   9. Agrees — |answer − estimate| ≤ the author's maxCorrection. Then the axis is SEATED.
// Rules 1-4 read no sensor and cost no time; a refusal leaves the estimate exactly as it was.
//
// ── What no gate here can catch (never claim otherwise — guide 14 lists it too) ─────────────────
// Something TALLER than a game piece at a plausible distance in the beam passes every check: a robot
// beside ours, a nested stack against the wall, a sensor wired to the other side's port, something
// close behind a CLEAR wall panel. The defences are the author's (a tight maxCorrection; parked where
// the route says), the keep-outs, the beam-height gate and the mount. A SINGLE standing Pin or Cup is
// refused by rule 4 — so long as the typed height and cone are true (A4 register HA-205).
//
// ── Every number that is a belief carries its register id ──────────────────────────────────────
// The sensor's accuracy (A4 register HA-204), its cone and nearest-surface behaviour (HA-205), the
// wall returning a reading (HA-206), the update interval (HA-209), and every gate default (HA-211).
// The season's walls and keep-outs are localization/override_walls.hpp's (HA-207, HA-208, HA-212).
//
// PURE (the master plan's §5 layering): no chassis/, motion/, sequence/ or competition/ include — the
// Localizer includes the leaf axis.hpp, and the calls sit at the layers that own their arguments.
// Allocation-free (fixed std::arrays), no exceptions on a refusal (a refusal is a VALUE);
// SHULIB_PRECONDITION only for programming errors (a non-finite reading handed to the geometry, too
// many keep-outs handed to FieldWall::line). The angle defaults are spelled `N * kWallDegToRad`
// because the `_deg` literal is not constexpr and a robot file's block must sit in a constexpr table.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/distance.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/localization/axis.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// Degrees to radians, as a constexpr factor: the wall reset's angle defaults are written
/// `5.0 * kWallDegToRad` because the `_deg` literal builds a non-constexpr math::Angle.
inline constexpr double kWallDegToRad = math::Angle::kPi / 180.0;

/// VEX's published accuracy of the V5 Distance Sensor below 200 mm — "approximately ±15 mm" — in
/// inches. With kPublishedFarAccuracyFraction it gives the accuracy the reset assumes until a
/// measured `SideMount::rangeAccuracy` replaces it: max(15 mm, 5 % of the reading) at EVERY range,
/// never a step at 200 mm (a step would put a cliff in the precision gate). A4 register HA-204.
inline constexpr double kPublishedCloseAccuracyInches = 15.0 / 25.4;

/// VEX's published accuracy above 200 mm — "about 5 %" of the reading. A4 register HA-204.
inline constexpr double kPublishedFarAccuracyFraction = 0.05;

/// Which of the robot's two side sensors a reset reads (one per side, facing straight out — the
/// team lead's 2026-10-08 ruling for both robots).
enum class WallSide : std::uint8_t {
    /// The sensor on the robot's LEFT side (facing +90 degrees from forward, typically).
    Left,
    /// The sensor on the robot's RIGHT side (facing -90 degrees from forward, typically).
    Right
};

/// The side's word in the log line: "LEFT" or "RIGHT" (printed "(LEFT sensor)" — a sensor, never a
/// direction). Exhaustive, no default.
[[nodiscard]] constexpr const char* wallSideWord(WallSide side) noexcept {
    switch (side) {
        case WallSide::Left:
            return "LEFT";
        case WallSide::Right:
            return "RIGHT";
    }
    return "?";
}

/// Whose coordinates a wall is in.
enum class WallFrame : std::uint8_t {
    /// A SEASON wall in the field frame (F1) — localization::override2026's. Meaningful only when the
    /// estimate is in field coordinates, i.e. the auton declared a FIELD start.
    Field,
    /// An author's OWN line (`FieldWall::line`), in whatever frame the author's code uses, with the
    /// author's own keep-outs or none — the explicit, Tier-3 form's.
    Own
};

/// What lifts a keep-out span (rule 3).
enum class KeepOutKind : std::uint8_t {
    /// Never lifted: the Toggle's brackets, the Loaders, the corners, the VEX U Load Zones.
    Fixed,
    /// The Toggle's BODY: lifted per side only by the mount's MEASURED `toggleChecked` (that sensor,
    /// at its height, was seen to read the wall beneath a real Toggle on a field).
    Toggle,
    /// Where single game pieces are KNOWN to stand (the start pieces): applies while the mount's cone
    /// is UNMEASURED and lifts once a measured `coneHalfAngle` is typed, because the beam-height gate
    /// (rule 4) then rests on a measured cone.
    PieceZone
};

/// One stretch of a wall a beam must not land on, as coordinates ALONG the wall (`from` <= `to`;
/// the along axis is the one the wall does not measure).
struct WallKeepOut {
    /// The span's lower end along the wall (inches).
    units::Length from{};
    /// The span's upper end along the wall (inches); >= `from`.
    units::Length to{};
    /// What lifts it (KeepOutKind).
    KeepOutKind kind = KeepOutKind::Fixed;
    /// The span's name in the log line ("Loader", "Load Zone", "Toggle", ...) — a stable literal.
    const char* why = "";
};

/// The most keep-out spans one wall carries (the red and blue walls need 11).
inline constexpr std::size_t kMaxWallKeepOuts = 12;

/// A wall: the line `axis = coord`, with the field on its `inward` side (+1: the field is at larger
/// coordinates, as for the left wall at x = -70.2; -1: at smaller), and the spans along it a beam must
/// not land on.
struct FieldWall {
    /// The wall's name in the log line ("left wall") — a stable literal.
    const char* name = "";
    /// The axis this wall measures.
    Axis axis = Axis::X;
    /// The wall's inner face, as a coordinate on `axis` (inches).
    units::Length coord{};
    /// +1 or -1: the side of the line the field (and the robot) is on. Anything else is refused
    /// BadRequest.
    std::int8_t inward = +1;
    /// Whose coordinates the wall is in (WallFrame).
    WallFrame frame = WallFrame::Field;
    /// The keep-out spans; the first `keepOutCount` are used.
    std::array<WallKeepOut, kMaxWallKeepOuts> keepOut{};
    /// How many of `keepOut` are used.
    std::uint8_t keepOutCount = 0;

    /// An author's OWN wall line (frame `Own`) — for hand-wired Tier-3 code and tests that know
    /// their frame: the line `axis = coord`, the field on the `inward` side, and the author's
    /// keep-outs (none by default — a line with none has none: the reset then cannot tell a Loader
    /// from the wall). More than kMaxWallKeepOuts spans is a programming error (a precondition).
    [[nodiscard]] static constexpr FieldWall line(const char* name, Axis axis, units::Length coord,
                                                  std::int8_t inward,
                                                  std::span<const WallKeepOut> keepOuts = {}) {
        SHULIB_PRECONDITION(keepOuts.size() <= kMaxWallKeepOuts,
                            "FieldWall::line: more keep-out spans than kMaxWallKeepOuts");
        FieldWall w{};
        w.name = name;
        w.axis = axis;
        w.coord = coord;
        w.inward = inward;
        w.frame = WallFrame::Own;
        for (std::size_t i = 0; i < keepOuts.size(); ++i) {
            w.keepOut[i] = keepOuts[i];
        }
        w.keepOutCount = static_cast<std::uint8_t>(keepOuts.size());
        return w;
    }

    /// The used keep-out spans, as a view into this wall.
    [[nodiscard]] constexpr std::span<const WallKeepOut> keepOuts() const noexcept {
        const std::size_t n = keepOutCount <= kMaxWallKeepOuts ? keepOutCount : kMaxWallKeepOuts;
        return std::span<const WallKeepOut>{keepOut.data(), n};
    }
};

/// One side sensor's place on the robot, in the BODY frame, MEASURED with a tape on the robot — every
/// number with its tolerance, and a provenance string. `measured == false` (the default) makes every
/// reset with this sensor REFUSE `MountUnset`; a placeholder "so the reset works" is the defect this
/// project has paid for before.
struct SideMount {
    /// False = UNSET: every reset with this sensor refuses MountUnset (never the robot).
    bool measured = false;
    /// The lens's offset FORWARD of the robot's centre (the turning centre on a tank), inches. The
    /// fore-aft centre (0) is load-bearing for precision: the axis error from a heading error is this
    /// offset times the heading error.
    units::Length forward{};
    /// The lens's offset to the LEFT of the robot's centre, inches (negative = right).
    units::Length left{};
    /// The lens centre's height above the floor, inches — decides the beam-height ceiling (rule 4).
    units::Length height{};
    /// The beam's direction, counter-clockwise from forward: `90.0 * kWallDegToRad` = straight out the
    /// left side, `-90.0 * kWallDegToRad` the right.
    units::AngleDim facing{};
    /// How well `forward` and `left` were measured (inches, +/-). PROVISIONAL default (A4: HA-211).
    units::Length positionTolerance{0.25};
    /// How square the sensor was found (+/-). PROVISIONAL default (A4: HA-211).
    units::AngleDim facingTolerance{2.0 * kWallDegToRad};
    /// Subtracted from every reading; 0 = none measured. Typed ONLY when this sensor's readings at
    /// taped distances from a wall show a consistent offset (A4 register HA-204).
    units::Length rangeBias{};
    /// This sensor's +/- accuracy, measured from readings at taped distances from a wall; 0 =
    /// unmeasured, and the reset assumes VEX's published max(15 mm, 5 % of the reading) — A4
    /// register HA-204.
    units::Length rangeAccuracy{};
    /// The beam's half-angle, measured (how far to the side of the beam a target is still seen);
    /// 0 = UNMEASURED: the limits' 18 degrees is used and the start-piece zones stay kept out (A4
    /// register HA-205).
    units::AngleDim coneHalfAngle{};
    /// True only when this sensor, at its height, was seen to read the WALL beneath a real Toggle's
    /// BODY on a field — then the Toggle body's keep-out lifts for it (its brackets never do — A4
    /// register HA-208).
    bool toggleChecked = false;
    /// Who measured what, how and when — a stable literal; printed by nothing, read by people.
    const char* provenance = "";
};

/// The sensor-physics and geometry gates — PROVISIONAL library defaults (A4: HA-211), overridable per
/// robot. The ONE policy number, maxCorrection, is not here: the author states it in every request.
struct WallResetLimits {
    /// The shortest bias-corrected reading accepted (inches): VEX's 20-mm floor with room for the
    /// unpublished close-range behaviour. PROVISIONAL (A4: HA-211).
    units::Length rangeMin{2.0};
    /// The longest (inches): where the published accuracy alone (5 % = 0.8 in) leaves no room under
    /// the 1.0-in precision gate. PROVISIONAL (A4: HA-211).
    units::Length rangeMax{16.0};
    /// The beam's largest angle off the wall's normal (rule 2). PROVISIONAL (A4: HA-211).
    units::AngleDim squareWithin{5.0 * kWallDegToRad};
    /// The window's largest max - min reading (rule 7). PROVISIONAL (A4: HA-211).
    units::Length spreadMax{0.5};
    /// The fewest samples a window needs (rule 7); at most kMaxWallSamples. PROVISIONAL (A4: HA-211).
    int minSamples = 5;
    /// The shortest window (rule 7) — the TIME is the real guard: reads inside one sensor update
    /// repeat (A4 register HA-209). Also the span rule 5 measures the speed over. Above 0 and at
    /// most kMaxWallWindowSeconds. PROVISIONAL (A4: HA-211).
    units::Time window{0.15};
    /// The longest a window may run — the explicit form's frozen-wait lateness bound (at most 1 s).
    /// PROVISIONAL (A4: HA-211).
    units::Time windowMax{0.5};
    /// The lowest confidence a sample may carry (rule 6). Synthetic below 200 mm: the adapter reports
    /// 1.0 there. PROVISIONAL (A4: HA-211).
    double minConfidence = 0.5;
    /// The fastest the estimate may move during the window (rule 5): its displacement over the
    /// trailing `window`, divided by that time — never one tick's finite difference, which turns a
    /// still IMU's heading noise into phantom speed on offset tracking wheels.
    /// PROVISIONAL (A4: HA-211).
    units::Velocity stillSpeed{1.0};
    /// The fastest it may turn during the window (rule 5), on every tick (the IMU's own rate).
    /// PROVISIONAL (A4: HA-211).
    units::AngularVelocity stillTurn{3.0 * kWallDegToRad};
    /// The largest worst-case error of the answer (rule 8) — F2's ~1.0-in pose target. PROVISIONAL
    /// (A4: HA-211).
    units::Length errorBoundMax{1.0};
    /// The keep-out margin (rule 3): start-pose placement + a field element's place + the beam
    /// centre's aim, summed (~2.1) and rounded up. PROVISIONAL (A4: HA-211).
    units::Length keepOutMargin{3.0};
    /// The along-wall drift rate (rule 3): the keep-outs widen by this times the travel since the
    /// along axis was last set (robot two's saved scale reads ~8 % short). PROVISIONAL (A4: HA-211).
    double alongDriftRate = 0.10;
    /// The beam half-angle while the mount's is unmeasured (rules 3, 4) — an unofficial figure
    /// (A4 register HA-205). PROVISIONAL (A4: HA-211).
    units::AngleDim coneHalfAngle{18.0 * kWallDegToRad};
    /// The tallest single game piece the ceiling keeps the cone above (inches): a standing Pin or Cup
    /// (A4 register HA-212). PROVISIONAL (A4: HA-211).
    units::Length pieceTop{6.5};
    /// The clearance above that piece (inches). PROVISIONAL (A4: HA-211).
    units::Length beamMargin{0.5};
    /// The heading tolerance's floor (rule 8): HA-20's drift over a run, doubled. PROVISIONAL
    /// (A4: HA-211).
    units::AngleDim headingFloor{2.0 * kWallDegToRad};
    /// The heading tolerance's growth per radian turned since the heading was set: HA-84's 1 %,
    /// doubled. PROVISIONAL (A4: HA-211).
    double headingPerTurn = 0.02;
};

/// One reset, as an auton states it: the wall, the side, how far it may move the coordinate, and
/// whether a refusal fails the step.
struct WallResetRequest {
    /// The wall to reset against — a season wall (`localization::override2026::kLeftWall`, ...) or an
    /// author's own line (`FieldWall::line`).
    FieldWall wall{};
    /// Which side's sensor faces it.
    WallSide sensor = WallSide::Left;
    /// How far this wall may move the coordinate (inches). NO DEFAULT: 0 is UNSET and refused by name
    /// (`maxCorrection UNSET: say how far this wall may move x`) — the largest drift the author has
    /// taped at this point plus ~1 in, never near the gap to anything a wrong seat could mistake for
    /// the wall.
    units::Length maxCorrection{0.0};
    /// True: a refusal FAILS the step (through `then()` the chain stops). False: a refusal leaves the
    /// estimate as it was and the auton goes on.
    bool required = false;
    /// Optional: leads the log line when set — a stable literal.
    const char* name = nullptr;
};

/// One sample of the window, as the sampler took it.
struct WallSample {
    /// The sensor's distance() reading — RAW (the mount's bias is removed by the gates, never here).
    units::Length distance{};
    /// The sensor's confidence() reading, in [0, 1].
    double confidence = 0.0;
    /// The read-fault count moved across the sample's two device reads: the values may be HELD (a
    /// faulted port's adapter returns its last good reading), not measured — never evidence.
    bool faultMoved = false;
    /// When it was taken, from the window's first sample.
    units::Time at{};
};

/// The window's sample buffer: fixed, allocation-free. At the 10-ms tick the 0.5-s windowMax is 50
/// samples; a faster pacer fills it and the sampler STOPS at capacity — it never wraps or overruns.
inline constexpr std::size_t kMaxWallSamples = 64;

/// The longest `WallResetLimits::window` a reset accepts (seconds). The 64-sample buffer spans 63
/// ticks — 0.63 s at the project's 10-ms control tick — and a `window` longer than the buffer can
/// span could never be met: every reset would end TOO FEW SAMPLES. 0.6 s keeps a tick of slack;
/// past it the request is refused BadRequest, naming the field (found by the L6-A1 review).
inline constexpr double kMaxWallWindowSeconds = 0.6;

/// A reset's verdict. APPEND-ONLY from L6-A1's landing (2026-10-29): a value is never renumbered or
/// reused. `wallResetVerdictWord` gives each its word; guide 11 gives each its remedy.
enum class WallResetVerdict : std::uint8_t {
    /// The axis was re-seated.
    Seated,
    /// The robot declares no wall sensors (the everyday form: the robot's context has no wall-sensor
    /// slot).
    NoSensor,
    /// The named side's sensor does not exist (the provider refused it); the detail is the reason.
    SensorAbsent,
    /// The side's mount was never measured (`SideMount::measured` false).
    MountUnset,
    /// The estimate is not live yet (the IMU still booting).
    NotLive,
    /// A motion is still armed (a Tier-3 `async()`): the reset never moves the estimate under one.
    MotionActive,
    /// A request, mount or limit field is unusable; the detail names it.
    BadRequest,
    /// The auton's start is not a FIELD start (the everyday form); the detail is the start mode's word.
    NotFieldStart,
    /// The beam is not within `squareWithin` of the wall's normal (or faces away from it).
    NotSquare,
    /// The beam lands on (or its widened footprint overlaps) a keep-out span; the detail names it.
    KeepOut,
    /// The expected range, or the median's, is past the beam-height ceiling.
    BeamLow,
    /// The estimate moved or turned during the window.
    Moving,
    /// A sample was read while the port faulted.
    ReadFault,
    /// A sample had no return (confidence 0 — PROS's 9999, or a held value).
    NoReturn,
    /// A sample's confidence was under `minConfidence`.
    LowConfidence,
    /// A sample's bias-corrected range was outside [rangeMin, rangeMax].
    OutOfRange,
    /// The window had fewer than `minSamples`, or ran shorter than `window`.
    TooFewSamples,
    /// The window's readings spread more than `spreadMax`.
    Spread,
    /// The answer's worst-case error is over `errorBoundMax`.
    Imprecise,
    /// The answer differs from the estimate by more than `maxCorrection`.
    Disagrees,
    /// The run's deadline or a disable landed in the window — or had already come when it was to begin
    /// (`samples` 0; the line says which) — the everyday form.
    Cut
};

/// The verdict's word in the log line ("SEATED", "REFUSED keep-out", ...). Exhaustive with no
/// `default:` — a new verdict without a word does not compile (-Wswitch -Werror).
[[nodiscard]] constexpr const char* wallResetVerdictWord(WallResetVerdict v) noexcept {
    switch (v) {
        case WallResetVerdict::Seated:
            return "SEATED";
        case WallResetVerdict::NoSensor:
            return "REFUSED no sensor";
        case WallResetVerdict::SensorAbsent:
            return "REFUSED sensor absent";
        case WallResetVerdict::MountUnset:
            return "REFUSED mount unset";
        case WallResetVerdict::NotLive:
            return "REFUSED not live";
        case WallResetVerdict::MotionActive:
            return "REFUSED motion active";
        case WallResetVerdict::BadRequest:
            return "REFUSED bad request";
        case WallResetVerdict::NotFieldStart:
            return "REFUSED not a field start";
        case WallResetVerdict::NotSquare:
            return "REFUSED not square";
        case WallResetVerdict::KeepOut:
            return "REFUSED keep-out";
        case WallResetVerdict::BeamLow:
            return "REFUSED beam low";
        case WallResetVerdict::Moving:
            return "REFUSED moving";
        case WallResetVerdict::ReadFault:
            return "REFUSED read fault";
        case WallResetVerdict::NoReturn:
            return "REFUSED no return";
        case WallResetVerdict::LowConfidence:
            return "REFUSED low confidence";
        case WallResetVerdict::OutOfRange:
            return "REFUSED out of range";
        case WallResetVerdict::TooFewSamples:
            return "REFUSED too few samples";
        case WallResetVerdict::Spread:
            return "REFUSED spread";
        case WallResetVerdict::Imprecise:
            return "REFUSED imprecise";
        case WallResetVerdict::Disagrees:
            return "REFUSED disagrees";
        case WallResetVerdict::Cut:
            return "REFUSED cut";
    }
    return "?";
}

/// What a reset did, with every number its log line prints. A VALUE: a refusal is a report, never an
/// exception. Fields a verdict did not reach stay at their defaults.
struct WallResetReport {
    /// The verdict (the FIRST rule that failed, or Seated).
    WallResetVerdict verdict = WallResetVerdict::NoSensor;
    /// The axis the wall measures.
    Axis axis = Axis::X;
    /// The side whose sensor was read.
    WallSide side = WallSide::Left;
    /// The request's `required` (D3): a refusal then fails the step.
    bool required = false;
    /// The estimate's coordinate on `axis` when the reset judged it.
    units::Length before{};
    /// The wall's answer for that coordinate (filled once a median exists).
    units::Length measured{};
    /// How far the coordinate was moved: measured - before when SEATED, 0 otherwise.
    units::Length applied{};
    /// The request's maxCorrection, echoed for the line.
    units::Length maxCorrection{};
    /// The window's MEDIAN reading — raw, as the sensor gave it.
    units::Length median{};
    /// The window's max - min reading.
    units::Length spread{};
    /// How many samples the window took.
    int samples = 0;
    /// How long the window ran (first sample to last).
    units::Time windowSpan{};
    /// The beam's angle off the wall's normal, by the estimate's heading (rule 2).
    units::AngleDim offSquare{};
    /// The beam-centre range to the wall the ESTIMATE implies (rules 3, 4).
    units::Length expectedRange{};
    /// The beam centre's coordinate along the wall, by the estimate (rule 3).
    units::Length along{};
    /// The beam's footprint half-width along the wall at the expected range (rule 3).
    units::Length beamHalfWidth{};
    /// The keep-out margin used (rule 3).
    units::Length keepOutMargin{};
    /// The along-wall drift allowance used: alongDriftRate x travelSinceSeat (rule 3).
    units::Length alongUncertainty{};
    /// The beam half-angle used (the mount's measured one, or the limits').
    units::AngleDim cone{};
    /// The beam-height ceiling (rule 4).
    units::Length ceiling{};
    /// The heading tolerance used by the precision gate (rule 8).
    units::AngleDim headingTolerance{};
    /// The answer's worst-case error (rule 8; filled once a median exists).
    units::Length errorBound{};
    /// The fastest the estimate moved during the window.
    units::Velocity speedSeen{};
    /// The fastest it turned during the window.
    units::AngularVelocity turnSeen{};
    /// The refusing sample's index (rule 6), or -1.
    int sampleIndex = -1;
    /// The refusing sample's distance reading (rule 6).
    units::Length sampleDistance{};
    /// The refusing sample's confidence (rule 6).
    double sampleConfidence = 0.0;
    /// BadRequest: the field's message; KeepOut: the span's name; SensorAbsent: the absence sentence;
    /// NotFieldStart: the start mode's word ("RELATIVE", "UNSET" — L6-A2). A stable literal (or the
    /// Robot's copied sentence), "" when none.
    const char* detail = "";
    /// KeepOut: a clean band for the beam centre was found along the wall.
    bool cleanFound = false;
    /// KeepOut: the nearest clean band's lower end along the wall.
    units::Length cleanFrom{};
    /// KeepOut: the nearest clean band's upper end along the wall.
    units::Length cleanTo{};
    /// False: the sensor has no read-fault count ("faults n/a" — never "faults ok").
    bool faultChannel = true;

    /// The axis was re-seated.
    [[nodiscard]] constexpr bool seated() const noexcept { return verdict == WallResetVerdict::Seated; }
    /// The auton may go on: SEATED, or REFUSED and not `required` (D3) — the value a `then()` returns.
    [[nodiscard]] constexpr bool continues() const noexcept { return seated() || !required; }
};

/// One side as a reset reads it: the sensor, its read-fault count, its mount, and why it is absent.
/// Built by the caller (the explicit form) or by robot::Robot from the robot file's `walls` block
/// (config/wall_sensor_config.hpp).
struct WallSensor {
    /// The sensor; nullptr = absent (the reset refuses SensorAbsent with `absence`).
    hal::IDistance* sensor = nullptr;
    /// Its read-fault count; nullptr = none (a fake, or a provider that never screens) — accepted,
    /// and the line says "faults n/a".
    const hal::IReadFaults* faults = nullptr;
    /// Its measured mount.
    SideMount mount{};
    /// Why it is absent, when `sensor` is nullptr — a stable literal or the Robot's copied sentence.
    const char* absence = "not declared";
};

/// A robot's running tally of its resets (the Auton Tester shows it to the people testing, from
/// L6-B) — counts and the last outcome. `record()` is how a call files one.
struct WallResetLog {
    /// Resets SEATED so far.
    int seated = 0;
    /// Resets REFUSED so far.
    int refused = 0;
    /// The last outcome (a default report until the first).
    WallResetReport last{};

    /// File one outcome: counted SEATED or REFUSED, and kept as `last`.
    void record(const WallResetReport& report) noexcept {
        if (report.seated()) {
            ++seated;
        } else {
            ++refused;
        }
        last = report;
    }
};

/// Both sides, the robot's limits and its log — the OPTIONAL slot the robot's context carries
/// (`chassis::RobotContextConfig::walls`), which the everyday form finds through `chassis.deps().ctx`.
struct WallSensors {
    /// The LEFT side's sensor.
    WallSensor left{};
    /// The RIGHT side's sensor.
    WallSensor right{};
    /// The robot's gate limits.
    WallResetLimits limits{};
    /// The robot's running tally.
    WallResetLog log{};

    /// The named side.
    [[nodiscard]] constexpr const WallSensor& side(WallSide which) const noexcept {
        return which == WallSide::Left ? left : right;
    }
};

// ═══ the geometry (pure) ═══════════════════════════════════════════════════════════════════════

namespace detail {

/// The robot centre's coordinate on `axis` that a beam-centre RANGE `range` implies, for a robot at
/// heading `theta` with the lens at (`forward`, `left`) facing `facing` (radians), against the line
/// `axis = coord`. The beam leaves the lens along heading + facing and meets the line where its
/// `axis` coordinate equals `coord`; the inward sign is not needed here (rule 2 has already
/// required the beam to point AT the wall).
[[nodiscard]] inline double axisFromRange(double range, double theta, double forward, double left,
                                          double facing, Axis axis, double coord) noexcept {
    const double beam = theta + facing;
    if (axis == Axis::X) {
        const double leverX = forward * std::cos(theta) - left * std::sin(theta);
        return coord - range * std::cos(beam) - leverX;
    }
    const double leverY = forward * std::sin(theta) + left * std::cos(theta);
    return coord - range * std::sin(beam) - leverY;
}

/// A pose's coordinate on `axis`, as a plain double.
[[nodiscard]] constexpr double poseOn(const math::Pose2d& pose, Axis axis) noexcept {
    return axis == Axis::X ? pose.x().value() : pose.y().value();
}

}  // namespace detail

/// PURE: the accuracy the reset assumes for a bias-corrected RANGE: the mount's measured
/// `rangeAccuracy` when typed, else VEX's published max(15 mm, 5 % of the range) — at every range,
/// never a step at 200 mm. A4 register HA-204.
[[nodiscard]] inline units::Length readingAccuracy(units::Length range, const SideMount& mount) noexcept {
    if (mount.rangeAccuracy.value() > 0.0) {
        return mount.rangeAccuracy;
    }
    return units::Length{std::max(kPublishedCloseAccuracyInches,
                                  kPublishedFarAccuracyFraction * std::abs(range.value()))};
}

/// PURE: the robot centre's coordinate a READING implies (the beam-centre model; the mount's bias
/// removed). `reading` is the sensor's raw distance and must be finite (a precondition).
[[nodiscard]] inline units::Length axisFromReading(units::Length reading, const math::Pose2d& pose,
                                                   const SideMount& mount, const FieldWall& wall) {
    SHULIB_PRECONDITION(std::isfinite(reading.value()),
                        "axisFromReading: the reading must be finite");
    const double range = reading.value() - mount.rangeBias.value();
    return units::Length{detail::axisFromRange(range, pose.heading().radians(), mount.forward.value(),
                                               mount.left.value(), mount.facing.value(), wall.axis,
                                               wall.coord.value())};
}

/// PURE: the worst-case error of axisFromReading's answer — over the CORNERS of the tolerance box
/// (the reading +/- its accuracy, `forward` and `left` +/- the mount's positionTolerance, `facing`
/// +/- its facingTolerance, the heading +/- `headingTolerance`), each corner run through the exact
/// geometry, not a linearisation. A non-finite tolerance (a NaN typed into the mount, or handed in
/// as `headingTolerance`) gives +infinity — the worst answer, so the precision gate refuses —
/// never a NaN silently dropped by a max (found by the L6-A1 review). A4 register HA-204 (the
/// accuracy), HA-211 (the tolerances).
[[nodiscard]] inline units::Length axisErrorBound(units::Length reading, const math::Pose2d& pose,
                                                  const SideMount& mount, const FieldWall& wall,
                                                  units::AngleDim headingTolerance) {
    SHULIB_PRECONDITION(std::isfinite(reading.value()),
                        "axisErrorBound: the reading must be finite");
    const double range = reading.value() - mount.rangeBias.value();
    const double acc = readingAccuracy(units::Length{range}, mount).value();
    const double theta = pose.heading().radians();
    const double f = mount.forward.value();
    const double l = mount.left.value();
    const double phi = mount.facing.value();
    const double pt = std::abs(mount.positionTolerance.value());
    const double ft = std::abs(mount.facingTolerance.value());
    const double ht = std::abs(headingTolerance.value());
    const double coord = wall.coord.value();
    const double nominal = detail::axisFromRange(range, theta, f, l, phi, wall.axis, coord);
    double worst = 0.0;
    for (unsigned corner = 0; corner < 32U; ++corner) {
        const double r = range + ((corner & 1U) != 0U ? acc : -acc);
        const double fc = f + ((corner & 2U) != 0U ? pt : -pt);
        const double lc = l + ((corner & 4U) != 0U ? pt : -pt);
        const double pc = phi + ((corner & 8U) != 0U ? ft : -ft);
        const double tc = theta + ((corner & 16U) != 0U ? ht : -ht);
        const double v = detail::axisFromRange(r, tc, fc, lc, pc, wall.axis, coord);
        const double err = std::abs(v - nominal);
        if (!std::isfinite(err)) {
            return units::Length{std::numeric_limits<double>::infinity()};
        }
        worst = std::max(worst, err);
    }
    return units::Length{worst};
}

/// PURE: the heading tolerance the precision gate assumes: `headingFloor` + `headingPerTurn` x the
/// |rotation| since the heading was last seeded (`Localizer::turnedSinceHeadingSeed`). A4 register
/// HA-211 (the floor from HA-20, the growth from HA-84).
[[nodiscard]] inline units::AngleDim headingTolerance(units::AngleDim turnedSinceHeadingSeed,
                                                      const WallResetLimits& limits) noexcept {
    return units::AngleDim{limits.headingFloor.value()
                           + limits.headingPerTurn * std::abs(turnedSinceHeadingSeed.value())};
}

/// PURE: the beam half-angle a reset uses — the mount's MEASURED one when typed (> 0), else the
/// limits' (A4 register HA-205).
[[nodiscard]] constexpr units::AngleDim effectiveCone(const SideMount& mount,
                                                     const WallResetLimits& limits) noexcept {
    return mount.coneHalfAngle.value() > 0.0 ? mount.coneHalfAngle : limits.coneHalfAngle;
}

/// PURE: the farthest range whose cone stays above `pieceTop` + `beamMargin` all the way to the
/// wall, from the TYPED height and cone: (height - pieceTop - beamMargin) / tan(cone), and 0 when
/// the lens is not above that line at all. At or under it, no single standing game piece can be the
/// nearest surface in the cone — nor the floor, which the cone meets only beyond height / tan(cone).
[[nodiscard]] inline units::Length beamCeiling(const SideMount& mount, const WallResetLimits& limits) noexcept {
    const double rise = mount.height.value() - limits.pieceTop.value() - limits.beamMargin.value();
    if (!(rise > 0.0)) {
        return units::Length{0.0};
    }
    return units::Length{rise / std::tan(effectiveCone(mount, limits).value())};
}

/// PURE: the first thing wrong with a request, its mount or the limits, as the message the line
/// prints ("maxCorrection UNSET: say how far this wall may move x") — or nullptr when everything is
/// usable. Rule 1's last check (BadRequest).
[[nodiscard]] inline const char* wallResetProblem(const SideMount& mount, const WallResetRequest& request,
                                                  const WallResetLimits& limits) noexcept {
    const auto finite = [](double v) { return std::isfinite(v); };
    const double mc = request.maxCorrection.value();
    if (mc == 0.0) {
        return request.wall.axis == Axis::X ? "maxCorrection UNSET: say how far this wall may move x"
                                            : "maxCorrection UNSET: say how far this wall may move y";
    }
    if (!finite(mc) || !(mc > 0.0)) {
        return "maxCorrection must be a finite distance above 0";
    }
    const FieldWall& w = request.wall;
    if (w.inward != 1 && w.inward != -1) {
        return "wall.inward must be +1 or -1";
    }
    if (!finite(w.coord.value())) {
        return "wall.coord is not finite";
    }
    if (w.keepOutCount > kMaxWallKeepOuts) {
        return "wall.keepOutCount is past kMaxWallKeepOuts";
    }
    for (const WallKeepOut& k : w.keepOuts()) {
        if (!finite(k.from.value()) || !finite(k.to.value()) || !(k.from.value() <= k.to.value())) {
            return "a wall keep-out span is reversed or not finite";
        }
    }
    if (!finite(mount.forward.value()) || !finite(mount.left.value()) || !finite(mount.height.value())
        || !finite(mount.facing.value()) || !finite(mount.rangeBias.value())) {
        return "the mount has a value that is not finite";
    }
    if (!finite(mount.positionTolerance.value()) || !(mount.positionTolerance.value() >= 0.0)
        || !finite(mount.facingTolerance.value()) || !(mount.facingTolerance.value() >= 0.0)
        || !finite(mount.rangeAccuracy.value()) || !(mount.rangeAccuracy.value() >= 0.0)) {
        return "the mount's tolerances and accuracy must be finite and >= 0";
    }
    const double mountCone = mount.coneHalfAngle.value();
    if (!finite(mountCone) || (mountCone != 0.0 && !(mountCone >= 3.0 * kWallDegToRad && mountCone <= 30.0 * kWallDegToRad))) {
        return "mount.coneHalfAngle must be 0 (unmeasured) or 3..30 degrees";
    }
    // Angle edges compared in RADIANS against `N * kWallDegToRad` — the very value a robot file
    // types — so a limit typed exactly at an edge is accepted exactly (a radians -> degrees round
    // trip is not exact: 30 degrees comes back 29.999999999999996).
    const double square = limits.squareWithin.value();
    const double cone = limits.coneHalfAngle.value();
    if (!(limits.rangeMin.value() > 0.0) || !(limits.rangeMax.value() > limits.rangeMin.value())
        || !finite(limits.rangeMax.value())) {
        return "limits.rangeMin/rangeMax must satisfy 0 < rangeMin < rangeMax";
    }
    if (!(square > 0.0 && square <= 20.0 * kWallDegToRad)) {
        return "limits.squareWithin must be above 0 and at most 20 degrees";
    }
    if (!(cone >= 3.0 * kWallDegToRad && cone <= 30.0 * kWallDegToRad)) {
        return "limits.coneHalfAngle must be 3..30 degrees";
    }
    if (!(limits.spreadMax.value() > 0.0) || !finite(limits.spreadMax.value())) {
        return "limits.spreadMax must be finite and above 0";
    }
    if (limits.minSamples < 1 || limits.minSamples > static_cast<int>(kMaxWallSamples)) {
        return "limits.minSamples must be 1..64";
    }
    if (!(limits.window.value() > 0.0) || !(limits.windowMax.value() >= limits.window.value())
        || !(limits.windowMax.value() <= 1.0)) {
        return "limits.window/windowMax must satisfy 0 < window <= windowMax <= 1 s";
    }
    if (!(limits.window.value() <= kMaxWallWindowSeconds)) {
        return "limits.window must be at most 0.6 s (the 64-sample buffer spans 0.63 s at 10 ms)";
    }
    if (!(limits.minConfidence >= 0.0 && limits.minConfidence <= 1.0)) {
        return "limits.minConfidence must be 0..1";
    }
    if (!(limits.stillSpeed.value() > 0.0) || !finite(limits.stillSpeed.value())
        || !(limits.stillTurn.value() > 0.0) || !finite(limits.stillTurn.value())) {
        return "limits.stillSpeed/stillTurn must be finite and above 0";
    }
    if (!(limits.errorBoundMax.value() > 0.0) || !finite(limits.errorBoundMax.value())) {
        return "limits.errorBoundMax must be finite and above 0";
    }
    if (!(limits.keepOutMargin.value() >= 0.0) || !finite(limits.keepOutMargin.value())
        || !(limits.alongDriftRate >= 0.0) || !finite(limits.alongDriftRate)) {
        return "limits.keepOutMargin/alongDriftRate must be finite and >= 0";
    }
    if (!(limits.pieceTop.value() >= 0.0) || !finite(limits.pieceTop.value())
        || !(limits.beamMargin.value() >= 0.0) || !finite(limits.beamMargin.value())) {
        return "limits.pieceTop/beamMargin must be finite and >= 0";
    }
    if (!(limits.headingFloor.value() >= 0.0) || !finite(limits.headingFloor.value())
        || !(limits.headingPerTurn >= 0.0) || !finite(limits.headingPerTurn)) {
        return "limits.headingFloor/headingPerTurn must be finite and >= 0";
    }
    return nullptr;
}

/// Where a beam lands, by the ESTIMATE — everything rules 2-4 judge, computed once (PURE).
struct WallAim {
    /// The beam's angle off the wall's outward normal, in [0, pi]: above 90 degrees it faces away.
    units::AngleDim offSquare{};
    /// The beam-centre range to the wall the estimate implies (<= 0 if the estimate puts the lens
    /// past the wall; only meaningful while the beam points at it).
    units::Length expectedRange{};
    /// The beam centre's coordinate along the wall.
    units::Length along{};
    /// The footprint's half-width along the wall at the expected range: the LARGER side of the
    /// exact cone footprint (a skewed beam's footprint is lopsided), applied both ways.
    units::Length beamHalfWidth{};
    /// The keep-out margin (the limits').
    units::Length margin{};
    /// The along-wall drift allowance (alongDriftRate x the travel since the along axis was set).
    units::Length drift{};
    /// beamHalfWidth + margin + drift: how far either side of `along` a keep-out must stay clear.
    units::Length widening{};
    /// The beam half-angle used (effectiveCone).
    units::AngleDim cone{};
    /// The beam-height ceiling (beamCeiling).
    units::Length ceiling{};
    /// The first ACTIVE keep-out the widened footprint overlaps (its `why`), or nullptr.
    const char* keepOutWhy = nullptr;
    /// A clean band for the beam centre exists along the wall (only computed when `keepOutWhy` is set).
    bool cleanFound = false;
    /// The nearest clean band's lower end (may be -infinity on an author's own line).
    units::Length cleanFrom{};
    /// The nearest clean band's upper end (may be +infinity on an author's own line).
    units::Length cleanTo{};
};

/// PURE: whether a keep-out span applies for this mount (KeepOutKind's rules).
[[nodiscard]] constexpr bool keepOutActive(const WallKeepOut& span, const SideMount& mount) noexcept {
    switch (span.kind) {
        case KeepOutKind::Fixed:
            return true;
        case KeepOutKind::Toggle:
            return !mount.toggleChecked;
        case KeepOutKind::PieceZone:
            return !(mount.coneHalfAngle.value() > 0.0);
    }
    return true;
}

/// PURE: where the beam lands by the estimate `pose`, and what it would land on (rules 2-4's
/// inputs). Requires a usable request (`wallResetProblem` returned nullptr) — with an `inward`
/// other than +/-1 the normal is undefined.
[[nodiscard]] inline WallAim aimAt(const math::Pose2d& pose, const SideMount& mount, const FieldWall& wall,
                                   const WallResetLimits& limits, units::Length alongUncertainty) {
    WallAim aim{};
    const double theta = pose.heading().radians();
    const double beam = theta + mount.facing.value();
    const double bx = std::cos(beam);
    const double by = std::sin(beam);
    const double inward = static_cast<double>(wall.inward);
    // Outward normal n = -inward * e_axis; along unit t = +e_other.
    const double bAxis = wall.axis == Axis::X ? bx : by;
    const double bAlong = wall.axis == Axis::X ? by : bx;
    const double cosOff = -inward * bAxis;
    const double alpha = std::atan2(bAlong, cosOff);  // signed, toward +along
    aim.offSquare = units::AngleDim{std::abs(alpha)};
    // The lens, in the field (or the author's) frame.
    const double lx = pose.x().value() + mount.forward.value() * std::cos(theta)
                      - mount.left.value() * std::sin(theta);
    const double ly = pose.y().value() + mount.forward.value() * std::sin(theta)
                      + mount.left.value() * std::cos(theta);
    const double lensAxis = wall.axis == Axis::X ? lx : ly;
    const double lensAlong = wall.axis == Axis::X ? ly : lx;
    const double perpendicular = inward * (lensAxis - wall.coord.value());  // > 0: on the field side
    const double cone = effectiveCone(mount, limits).value();
    aim.cone = units::AngleDim{cone};
    aim.ceiling = beamCeiling(mount, limits);
    aim.margin = limits.keepOutMargin;
    aim.drift = units::Length{std::max(0.0, alongUncertainty.value())};
    if (cosOff > 1e-9 && std::abs(alpha) + cone < 1.5) {
        const double range = perpendicular / cosOff;
        aim.expectedRange = units::Length{range};
        const double d = std::max(0.0, perpendicular);
        const double tc = std::tan(alpha);
        aim.along = units::Length{lensAlong + perpendicular * tc};
        const double upper = d * (std::tan(alpha + cone) - tc);
        const double lower = d * (tc - std::tan(alpha - cone));
        aim.beamHalfWidth = units::Length{std::max(upper, lower)};
    } else {
        aim.expectedRange = units::Length{cosOff > 1e-9 ? perpendicular / cosOff : 0.0};
        aim.along = units::Length{lensAlong};
        aim.beamHalfWidth = units::Length{0.0};
    }
    const double w = aim.beamHalfWidth.value() + aim.margin.value() + aim.drift.value();
    aim.widening = units::Length{w};
    const double a = aim.along.value();
    for (const WallKeepOut& k : wall.keepOuts()) {
        if (!keepOutActive(k, mount)) {
            continue;
        }
        if (a - w <= k.to.value() && a + w >= k.from.value()) {
            aim.keepOutWhy = k.why;
            break;
        }
    }
    if (aim.keepOutWhy != nullptr) {
        // The nearest clean band for the beam centre: the gaps between the active spans, each grown
        // by the widening. A season wall's ends are closed by its corners, so only gaps BETWEEN
        // blocked stretches count there; an author's own line may be open at either end.
        std::array<double, kMaxWallKeepOuts> lo{};
        std::array<double, kMaxWallKeepOuts> hi{};
        std::size_t n = 0;
        for (const WallKeepOut& k : wall.keepOuts()) {
            if (keepOutActive(k, mount)) {
                lo[n] = k.from.value() - w;
                hi[n] = k.to.value() + w;
                ++n;
            }
        }
        // Insertion sort by lower end (n <= 12), then merge.
        for (std::size_t i = 1; i < n; ++i) {
            for (std::size_t j = i; j > 0 && lo[j - 1] > lo[j]; --j) {
                std::swap(lo[j - 1], lo[j]);
                std::swap(hi[j - 1], hi[j]);
            }
        }
        std::size_t m = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if (m > 0 && lo[i] <= hi[m - 1]) {
                hi[m - 1] = std::max(hi[m - 1], hi[i]);
            } else {
                lo[m] = lo[i];
                hi[m] = hi[i];
                ++m;
            }
        }
        const double inf = std::numeric_limits<double>::infinity();
        double bestDist = inf;
        const auto consider = [&](double gLo, double gHi) {
            if (!(gLo < gHi)) {
                return;
            }
            const double dist = a < gLo ? gLo - a : (a > gHi ? a - gHi : 0.0);
            if (dist < bestDist) {
                bestDist = dist;
                aim.cleanFound = true;
                aim.cleanFrom = units::Length{gLo};
                aim.cleanTo = units::Length{gHi};
            }
        };
        for (std::size_t i = 0; i + 1 < m; ++i) {
            consider(hi[i], lo[i + 1]);
        }
        if (wall.frame == WallFrame::Own && m > 0) {
            consider(-inf, lo[0]);
            consider(hi[m - 1], inf);
        }
    }
    return aim;
}

/// PURE: the gates that need no reading, in order — BadRequest (wallResetProblem), NotSquare,
/// KeepOut, BeamLow; Seated means "go on and sample". (The call checks the sensor, the mount's
/// `measured`, liveness and motion first.)
[[nodiscard]] inline WallResetVerdict preCheck(const math::Pose2d& pose, const SideMount& mount,
                                               const WallResetRequest& request,
                                               const WallResetLimits& limits,
                                               units::Length alongUncertainty) {
    if (wallResetProblem(mount, request, limits) != nullptr) {
        return WallResetVerdict::BadRequest;
    }
    const WallAim aim = aimAt(pose, mount, request.wall, limits, alongUncertainty);
    if (!(aim.offSquare.value() <= limits.squareWithin.value())) {
        return WallResetVerdict::NotSquare;
    }
    if (aim.keepOutWhy != nullptr) {
        return WallResetVerdict::KeepOut;
    }
    if (!(aim.expectedRange.value() <= aim.ceiling.value())) {
        return WallResetVerdict::BeamLow;
    }
    return WallResetVerdict::Seated;
}

/// PURE: rule 6 for ONE sample — ReadFault, NoReturn, LowConfidence, OutOfRange, or Seated when the
/// sample is usable. The sampler stops the window at the first sample this refuses; the evaluator
/// names it.
[[nodiscard]] inline WallResetVerdict sampleVerdict(const WallSample& sample, const SideMount& mount,
                                                    const WallResetLimits& limits) noexcept {
    if (sample.faultMoved) {
        return WallResetVerdict::ReadFault;
    }
    if (!(sample.confidence > 0.0)) {
        return WallResetVerdict::NoReturn;
    }
    if (sample.confidence < limits.minConfidence) {
        return WallResetVerdict::LowConfidence;
    }
    const double range = sample.distance.value() - mount.rangeBias.value();
    if (!(range >= limits.rangeMin.value() && range <= limits.rangeMax.value())) {
        return WallResetVerdict::OutOfRange;
    }
    return WallResetVerdict::Seated;
}

/// PURE: whether a window's worst twist is still (rule 5): the norm of (vx, vy) within `stillSpeed`
/// and |omega| within `stillTurn`. A non-finite twist is not still.
[[nodiscard]] inline bool wallStill(const math::Twist2d& worst, const WallResetLimits& limits) noexcept {
    const double speed = std::hypot(worst.vx().value(), worst.vy().value());
    const double turn = std::abs(worst.omega().value());
    return speed <= limits.stillSpeed.value() && turn <= limits.stillTurn.value();
}

/// PURE: a report with every number the estimate alone decides — the axis, the side, `before`, the
/// aim's numbers, the heading tolerance, the limit echoed — and verdict Seated ("nothing refused
/// yet"). The call fills a pre-sampling refusal's report from it; evaluateWallReset starts from it.
[[nodiscard]] inline WallResetReport describeWallReset(const math::Pose2d& pose, const SideMount& mount,
                                                       const WallResetRequest& request,
                                                       const WallResetLimits& limits,
                                                       units::AngleDim headingTol,
                                                       units::Length alongUncertainty) {
    WallResetReport rep{};
    rep.verdict = WallResetVerdict::Seated;
    rep.axis = request.wall.axis;
    rep.side = request.sensor;
    rep.required = request.required;
    rep.maxCorrection = request.maxCorrection;
    rep.before = units::Length{detail::poseOn(pose, request.wall.axis)};
    rep.headingTolerance = headingTol;
    const char* problem = wallResetProblem(mount, request, limits);
    if (problem != nullptr) {
        rep.detail = problem;
        return rep;
    }
    const WallAim aim = aimAt(pose, mount, request.wall, limits, alongUncertainty);
    rep.offSquare = aim.offSquare;
    rep.expectedRange = aim.expectedRange;
    rep.along = aim.along;
    rep.beamHalfWidth = aim.beamHalfWidth;
    rep.keepOutMargin = aim.margin;
    rep.alongUncertainty = aim.drift;
    rep.cone = aim.cone;
    rep.ceiling = aim.ceiling;
    if (aim.keepOutWhy != nullptr) {
        rep.detail = aim.keepOutWhy;
        rep.cleanFound = aim.cleanFound;
        rep.cleanFrom = aim.cleanFrom;
        rep.cleanTo = aim.cleanTo;
    }
    return rep;
}

/// PURE: the whole verdict from a window of samples — rules 5-9 (run preCheck first: this does not
/// re-judge rules 2-4, but an unusable request is refused BadRequest here too, never judged).
/// `maxTwist` is the window's worst twist: its linear part's norm (WallSampler's speed over the
/// trailing `window`) and |omega| each the largest seen. `pose` is the estimate the answer is
/// compared with (and seated over). Fills `measured`, `applied` (SEATED only), `median`,
/// `spread`, `errorBound` and the refusing sample's numbers. Changes nothing.
[[nodiscard]] inline WallResetReport evaluateWallReset(std::span<const WallSample> samples,
                                                       const math::Pose2d& pose,
                                                       const math::Twist2d& maxTwist,
                                                       const SideMount& mount,
                                                       const WallResetRequest& request,
                                                       const WallResetLimits& limits,
                                                       units::AngleDim headingTol,
                                                       units::Length alongUncertainty) {
    WallResetReport rep = describeWallReset(pose, mount, request, limits, headingTol, alongUncertainty);
    if (wallResetProblem(mount, request, limits) != nullptr) {
        rep.verdict = WallResetVerdict::BadRequest;  // describeWallReset put the message in `detail`
        return rep;
    }
    rep.detail = "";
    rep.cleanFound = false;
    const std::size_t n = std::min(samples.size(), kMaxWallSamples);
    rep.samples = static_cast<int>(n);
    if (n > 0) {
        rep.windowSpan = samples[n - 1].at - samples[0].at;
    }
    rep.speedSeen = units::Velocity{std::hypot(maxTwist.vx().value(), maxTwist.vy().value())};
    rep.turnSeen = units::AngularVelocity{std::abs(maxTwist.omega().value())};
    // Rule 5 — stillness over the whole window.
    if (!wallStill(maxTwist, limits)) {
        rep.verdict = WallResetVerdict::Moving;
        return rep;
    }
    // Rule 6 — every sample.
    for (std::size_t i = 0; i < n; ++i) {
        const WallResetVerdict v = sampleVerdict(samples[i], mount, limits);
        if (v != WallResetVerdict::Seated) {
            rep.verdict = v;
            rep.sampleIndex = static_cast<int>(i);
            rep.sampleDistance = samples[i].distance;
            rep.sampleConfidence = samples[i].confidence;
            return rep;
        }
    }
    // Rule 7 — enough, consistent, and the median under the ceiling.
    if (n < static_cast<std::size_t>(limits.minSamples) || !(rep.windowSpan.value() >= limits.window.value())) {
        rep.verdict = WallResetVerdict::TooFewSamples;
        return rep;
    }
    std::array<double, kMaxWallSamples> sorted{};
    for (std::size_t i = 0; i < n; ++i) {
        sorted[i] = samples[i].distance.value();
    }
    std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(n));
    rep.spread = units::Length{sorted[n - 1] - sorted[0]};
    const double median = (n % 2U) == 1U ? sorted[n / 2U] : 0.5 * (sorted[n / 2U - 1U] + sorted[n / 2U]);
    rep.median = units::Length{median};
    if (!(rep.spread.value() <= limits.spreadMax.value())) {
        rep.verdict = WallResetVerdict::Spread;
        return rep;
    }
    rep.measured = axisFromReading(rep.median, pose, mount, request.wall);
    rep.errorBound = axisErrorBound(rep.median, pose, mount, request.wall, headingTol);
    if (!(median - mount.rangeBias.value() <= rep.ceiling.value())) {
        rep.verdict = WallResetVerdict::BeamLow;
        return rep;
    }
    // Rule 8 — precise enough.
    if (!(rep.errorBound.value() <= limits.errorBoundMax.value())) {
        rep.verdict = WallResetVerdict::Imprecise;
        return rep;
    }
    // Rule 9 — agrees with the estimate within the author's bound.
    const double diff = rep.measured.value() - rep.before.value();
    if (!(std::abs(diff) <= request.maxCorrection.value())) {
        rep.verdict = WallResetVerdict::Disagrees;
        return rep;
    }
    rep.verdict = WallResetVerdict::Seated;
    rep.applied = units::Length{diff};
    return rep;
}

// ═══ the sampler (one tick at a time; no clock, no wait — the call owns those) ══════════════════

/// The window's sampler: one `step()` per control tick, each judging stillness from the estimate
/// and taking ONE sample — the read-fault count read before the sample's first device read and
/// after its second (`distance()` then `confidence()` are TWO device reads; both must be
/// fault-free). It says the window is DONE at the first tick that refuses (moving, or a sample rule
/// 6 refuses), once the window has `minSamples` over `window`, at `windowMax`, or at the buffer's
/// capacity — never later. The explicit call drives it from `Chassis::waitUntil`'s predicate; the
/// everyday form from the run guard's wait. The wait's own ticks keep the estimate current.
///
/// RULE 5's SPEED is the estimate's DISPLACEMENT over the trailing `window`, per that time: on each
/// tick, the distance from the pose of the latest earlier tick at least `window` back (or the
/// window's first tick, while the window is younger than that), divided by the time between them,
/// floored at `window`. A robot moving at v reads v once the window is `window` old, and a push
/// lasting `window` is seen; WHY NOT ONE TICK'S VELOCITY (the L6-A1 review): the estimate's
/// per-tick linear velocity is a finite difference of the fused pose, and on a robot whose
/// tracking wheels sit off the turning centre a still IMU's heading noise (the register's HA-21,
/// ~0.05 degrees) moves that pose a few thousandths of an inch each tick — 1.1-1.4 in/s of PHANTOM
/// speed on the host test plant's wheel offsets, so every reset was refused MOVING while nothing
/// moved. That jitter does not add up over `window` (it is noise about a fixed point), so the
/// displacement reads ~0.08 in/s on average there instead (0.04-0.17 in/s over 600 still resets,
/// ten seeds). It grows in step with the noise: ~0.33 in/s (up to ~0.7) at four times HA-21's,
/// and it reaches the 1-in/s limit only near six times (2 of 600 still resets refused MOVING).
/// The turn rate is judged on every tick from the estimate's own `omega` (the IMU's rate), as
/// before. A NaN pose or turn rate counts as +infinity — the window ends MOVING at that tick
/// (std::max alone would drop a NaN and judge the tick still).
class WallSampler {
public:
    /// Borrows `sensor` and `limits` (both must outlive the sampler); `sensor.sensor` must be set and
    /// the limits usable (wallResetProblem() returned nullptr — `window` above 0).
    WallSampler(const WallSensor& sensor, const WallResetLimits& limits) noexcept
        : sensor_{&sensor}, limits_{&limits} {}

    /// One tick at time `now`, the estimate at `pose` and turning at `turnRate`: judge stillness,
    /// take one sample. Returns true when the window is DONE (the caller's wait predicate returns
    /// this).
    bool step(units::Time now, const math::Pose2d& pose, units::AngularVelocity turnRate) {
        if (!started_) {
            started_ = true;
            t0_ = now.value();
        }
        const double elapsed = now.value() - t0_;
        if (ticks_ < kMaxWallTicks) {
            tickAt_[ticks_] = elapsed;
            tickX_[ticks_] = pose.x().value();
            tickY_[ticks_] = pose.y().value();
            ++ticks_;
        }
        // Rule 5's speed: the displacement since the latest stored tick at least `window` back.
        const double span = limits_->window.value();
        while (ref_ + 1 < ticks_ && elapsed - tickAt_[ref_ + 1] >= span) {
            ++ref_;
        }
        const double moved = std::hypot(pose.x().value() - tickX_[ref_], pose.y().value() - tickY_[ref_]);
        double speed = moved / std::max(elapsed - tickAt_[ref_], span);
        double turn = std::abs(turnRate.value());
        if (!std::isfinite(speed)) {
            speed = std::numeric_limits<double>::infinity();
        }
        if (!std::isfinite(turn)) {
            turn = std::numeric_limits<double>::infinity();
        }
        maxSpeed_ = std::max(maxSpeed_, speed);
        maxTurn_ = std::max(maxTurn_, turn);
        if (!wallStill(maxTwist(), *limits_)) {
            return true;  // Moving: the window is over at the first moving tick
        }
        if (n_ < kMaxWallSamples) {
            const hal::IReadFaults* faults = sensor_->faults;
            const int c0 = faults != nullptr ? faults->faultedReads() : 0;
            const units::Length d = sensor_->sensor->distance();
            const double c = sensor_->sensor->confidence();
            const int c1 = faults != nullptr ? faults->faultedReads() : 0;
            samples_[n_] = WallSample{d, c, c1 != c0, units::Time{elapsed}};
            ++n_;
            if (sampleVerdict(samples_[n_ - 1], sensor_->mount, *limits_) != WallResetVerdict::Seated) {
                return true;  // rule 6 refuses: no later sample can change the verdict
            }
        }
        const bool enough = elapsed >= limits_->window.value()
                            && n_ >= static_cast<std::size_t>(limits_->minSamples);
        return enough || elapsed >= limits_->windowMax.value() || n_ >= kMaxWallSamples;
    }

    /// The samples taken so far.
    [[nodiscard]] std::span<const WallSample> samples() const noexcept {
        return std::span<const WallSample>{samples_.data(), n_};
    }

    /// The window's worst twist so far: vx carries the largest speed judged (rule 5's: the
    /// displacement over the trailing `window`), vy is 0, omega the largest |turn rate| — the form
    /// evaluateWallReset takes.
    [[nodiscard]] math::Twist2d maxTwist() const noexcept {
        return math::Twist2d{units::Velocity{maxSpeed_}, units::Velocity{0.0},
                             units::AngularVelocity{maxTurn_}};
    }

private:
    // One more than the samples: a tick can be judged MOVING without taking a sample.
    static constexpr std::size_t kMaxWallTicks = kMaxWallSamples + 1;

    const WallSensor* sensor_;
    const WallResetLimits* limits_;
    std::array<WallSample, kMaxWallSamples> samples_{};
    std::size_t n_ = 0;
    std::array<double, kMaxWallTicks> tickAt_{};
    std::array<double, kMaxWallTicks> tickX_{};
    std::array<double, kMaxWallTicks> tickY_{};
    std::size_t ticks_ = 0;
    std::size_t ref_ = 0;
    bool started_ = false;
    double t0_ = 0.0;
    double maxSpeed_ = 0.0;
    double maxTurn_ = 0.0;
};

// ═══ the log line (one per call) ═══════════════════════════════════════════════════════════════

/// PURE: append the reset's ONE log line (subsystem "WALL"; ASCII; within a Line's 384 bytes) —
/// `[<name>: ]<wall> <axis> <WORD> (<SIDE> sensor): <the verdict's numbers>`, e.g.
/// `left wall x SEATED (LEFT sensor): -52.21 -> -54.66 (moved -2.45, limit 3.00) | median 8.05 in
/// spread 0.26 n 10 | bound 0.62 | faults ok`. Caller-supplied text (the names, an absence sentence)
/// goes through the Line's sanitizer. MIRROR1-A: `sideNote` (default none — every line byte-identical
/// to before) is written INSIDE the sensor's parenthesis, `(LEFT sensor, <sideNote>)`, near the line's
/// start, so the terminal's 200-byte message cut never drops it: the everyday call passes "mirrored
/// from red wall RIGHT" for a mirrored auton's reset (competition/auton_wall_reset.hpp).
inline void appendWallResetLine(diag::lineformat::Line& line, const WallResetReport& rep,
                                const WallResetRequest& request, const SideMount& mount,
                                const WallResetLimits& limits, const char* sideNote = nullptr) {
    namespace lf = diag::lineformat;
    const auto num = [&line](double v, int prec) { lf::appendNum(line, v, 0, prec); };
    const auto deg = [](units::AngleDim a) { return a.value() / kWallDegToRad; };
    const auto window = [&] {
        line.appendLiteral(" | median ");
        num(rep.median.value(), 2);
        line.appendLiteral(" in spread ");
        num(rep.spread.value(), 2);
        line.appendLiteral(" n ");
        lf::appendUnsigned(line, static_cast<unsigned long>(rep.samples < 0 ? 0 : rep.samples));
    };
    // The faults tail, on the lines whose samples were screened: "ok" = the read-fault count did
    // not move across any sample judged (a ReadFault line says "faults moved" instead); "n/a" = the
    // sensor has no count to read — accepted, but never called ok.
    const auto faults = [&] {
        line.appendLiteral(rep.faultChannel ? " | faults ok" : " | faults n/a");
    };
    if (request.name != nullptr && request.name[0] != '\0') {
        line.appendSanitized(request.name, 48);
        line.appendLiteral(": ");
    }
    line.appendSanitized(request.wall.name != nullptr ? request.wall.name : "", 32);
    line.appendLiteral(" ");
    line.appendLiteral(axisLetter(rep.axis));
    line.appendLiteral(" ");
    line.appendLiteral(wallResetVerdictWord(rep.verdict));
    line.appendLiteral(" (");
    line.appendLiteral(wallSideWord(rep.side));
    line.appendLiteral(" sensor");
    if (sideNote != nullptr && sideNote[0] != '\0') {
        line.appendLiteral(", ");
        line.appendSanitized(sideNote, 64);
    }
    line.appendLiteral("): ");
    switch (rep.verdict) {
        case WallResetVerdict::Seated:
            num(rep.before.value(), 2);
            line.appendLiteral(" -> ");
            num(rep.measured.value(), 2);
            line.appendLiteral(" (moved ");
            num(rep.applied.value(), 2);
            line.appendLiteral(", limit ");
            num(rep.maxCorrection.value(), 2);
            line.appendLiteral(")");
            window();
            line.appendLiteral(" | bound ");
            num(rep.errorBound.value(), 2);
            faults();
            break;
        case WallResetVerdict::NoSensor:
            line.appendLiteral("this robot declares no wall sensors");
            break;
        case WallResetVerdict::SensorAbsent:
            line.appendSanitized(rep.detail != nullptr ? rep.detail : "", 120);
            break;
        case WallResetVerdict::MountUnset:
            line.appendLiteral("measure the mount and type it with measured = true");
            break;
        case WallResetVerdict::NotLive:
            line.appendLiteral("the estimate is not live yet");
            break;
        case WallResetVerdict::MotionActive:
            line.appendLiteral("a motion is still armed");
            break;
        case WallResetVerdict::BadRequest:
            line.appendSanitized(rep.detail != nullptr ? rep.detail : "", 120);
            break;
        case WallResetVerdict::NotFieldStart:
            // The start mode's word, so RELATIVE and UNSET read apart.
            line.appendLiteral("start ");
            line.appendSanitized(rep.detail != nullptr && rep.detail[0] != '\0' ? rep.detail : "not FIELD", 16);
            line.appendLiteral(" -- the season's walls are FIELD coordinates: declare a FIELD start");
            break;
        case WallResetVerdict::NotSquare:
            line.appendLiteral("beam ");
            num(deg(rep.offSquare), 1);
            line.appendLiteral(" deg off square (limit ");
            num(deg(limits.squareWithin), 1);
            line.appendLiteral(")");
            break;
        case WallResetVerdict::KeepOut:
            line.appendSanitized(rep.detail != nullptr ? rep.detail : "", 32);
            line.appendLiteral(" at along ");
            num(rep.along.value(), 1);
            line.appendLiteral(" (widened ");
            num(rep.beamHalfWidth.value() + rep.keepOutMargin.value() + rep.alongUncertainty.value(), 1);
            line.appendLiteral(" = beam ");
            num(rep.beamHalfWidth.value(), 1);
            line.appendLiteral(" + margin ");
            num(rep.keepOutMargin.value(), 1);
            line.appendLiteral(" + drift ");
            num(rep.alongUncertainty.value(), 1);
            line.appendLiteral(") | clean ");
            if (rep.cleanFound) {
                num(rep.cleanFrom.value(), 1);
                line.appendLiteral("..");
                num(rep.cleanTo.value(), 1);
            } else {
                line.appendLiteral("none");
            }
            break;
        case WallResetVerdict::BeamLow:
            if (rep.samples > 0) {
                line.appendLiteral("median ");
                num(rep.median.value() - mount.rangeBias.value(), 2);
            } else {
                line.appendLiteral("expected ");
                num(rep.expectedRange.value(), 2);
            }
            line.appendLiteral(" in past the ceiling ");
            num(rep.ceiling.value(), 2);
            line.appendLiteral(" in (lens ");
            num(mount.height.value(), 2);
            line.appendLiteral(" in up, cone ");
            num(deg(rep.cone), 1);
            line.appendLiteral(" deg)");
            break;
        case WallResetVerdict::Moving:
            line.appendLiteral("speed ");
            num(rep.speedSeen.value(), 2);
            line.appendLiteral(" in/s, turn ");
            num(rep.turnSeen.value() / kWallDegToRad, 1);
            line.appendLiteral(" deg/s (limit ");
            num(limits.stillSpeed.value(), 2);
            line.appendLiteral(" in/s, ");
            num(limits.stillTurn.value() / kWallDegToRad, 1);
            line.appendLiteral(" deg/s)");
            break;
        case WallResetVerdict::ReadFault:
            line.appendLiteral("sample ");
            lf::appendUnsigned(line, static_cast<unsigned long>(rep.sampleIndex < 0 ? 0 : rep.sampleIndex));
            line.appendLiteral(" read while the port faulted | faults moved");
            break;
        case WallResetVerdict::NoReturn:
        case WallResetVerdict::LowConfidence:
            line.appendLiteral("sample ");
            lf::appendUnsigned(line, static_cast<unsigned long>(rep.sampleIndex < 0 ? 0 : rep.sampleIndex));
            line.appendLiteral(" confidence ");
            num(rep.sampleConfidence, 2);
            line.appendLiteral(" (limit ");
            num(limits.minConfidence, 2);
            line.appendLiteral(", distance ");
            num(rep.sampleDistance.value(), 2);
            line.appendLiteral(" in)");
            faults();
            break;
        case WallResetVerdict::OutOfRange:
            line.appendLiteral("sample ");
            lf::appendUnsigned(line, static_cast<unsigned long>(rep.sampleIndex < 0 ? 0 : rep.sampleIndex));
            line.appendLiteral(" read ");
            num(rep.sampleDistance.value() - mount.rangeBias.value(), 2);
            line.appendLiteral(" in (window ");
            num(limits.rangeMin.value(), 2);
            line.appendLiteral("..");
            num(limits.rangeMax.value(), 2);
            line.appendLiteral(")");
            faults();
            break;
        case WallResetVerdict::TooFewSamples:
            line.appendLiteral("n ");
            lf::appendUnsigned(line, static_cast<unsigned long>(rep.samples < 0 ? 0 : rep.samples));
            line.appendLiteral(" over ");
            num(rep.windowSpan.value(), 2);
            line.appendLiteral(" s (need ");
            lf::appendUnsigned(line, static_cast<unsigned long>(limits.minSamples < 0 ? 0 : limits.minSamples));
            line.appendLiteral(" over ");
            num(limits.window.value(), 2);
            line.appendLiteral(" s)");
            break;
        case WallResetVerdict::Spread:
            line.appendLiteral("spread ");
            num(rep.spread.value(), 2);
            line.appendLiteral(" in (limit ");
            num(limits.spreadMax.value(), 2);
            line.appendLiteral(")");
            window();
            break;
        case WallResetVerdict::Imprecise:
            line.appendLiteral("bound ");
            num(rep.errorBound.value(), 2);
            line.appendLiteral(" in (limit ");
            num(limits.errorBoundMax.value(), 2);
            line.appendLiteral("; heading tolerance ");
            num(deg(rep.headingTolerance), 1);
            line.appendLiteral(" deg)");
            window();
            break;
        case WallResetVerdict::Disagrees:
            line.appendLiteral("wall says ");
            num(rep.measured.value(), 2);
            line.appendLiteral(", estimate ");
            num(rep.before.value(), 2);
            line.appendLiteral(", limit ");
            num(rep.maxCorrection.value(), 2);
            window();
            break;
        case WallResetVerdict::Cut:
            // No sample: the run had already ended (a deadline passed, or a disable) when the window was
            // to begin — the call came too late, which is not the window's fault.
            line.appendLiteral(rep.samples > 0
                                   ? "the run's deadline or a disable landed in the window; nothing changed"
                                   : "the run's deadline or a disable came before the window began; nothing "
                                     "changed");
            break;
    }
}

}  // namespace shulib::localization
