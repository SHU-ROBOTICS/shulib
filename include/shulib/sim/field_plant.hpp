#pragma once
//
// sim::FieldModel and sim::SideSensorPlant — the host test plant's FIELD and its SIDE DISTANCE
// SENSORS (chunk L6-A1, the wall reset, 2026-10-29). Say "host test plant", never "simulator":
// nothing here is a measured sensor or a measured field.
//
// ── Why it exists, and the one rule it keeps (the A2 rule) ───────────────────────────────────
// The wall reset turns a side sensor's reading into one field coordinate. A test of that geometry
// that used the LIBRARY's own geometry to make the reading would grade the code against itself: a
// sign error would sit in both and cancel. So this plant is INDEPENDENT: its own copy of the field's
// drawing numbers (never the library's season-walls header — a test asserts the two agree to 0.01 in),
// its own 3-D ray cast against boxes with heights, and the sensor's TRUE mount, set separately from
// the mount the robot file BELIEVES (every adversarial case mismatches them). It includes nothing
// from the library's localization layer (the L6 brief's landmine 9; its §7 greps this file).
//
// ── The field (FieldModel) ───────────────────────────────────────────────────────────────────
// Axis-aligned boxes in the field frame F1 (x right from the red station, y away from it, z up from
// the floor), each opaque or SEE-THROUGH (a clear wall panel, the A4 register's HA-206), plus the
// floor (z = 0). `override2026()` builds the season's field from its OWN numbers (VEX V5RC
// "Override" manual v2.0, Appendix A):
//   * the four perimeter walls, inner faces at +/-70.2047 in (A10: 3566.4 mm wall to wall), 11.5 in
//     tall (A14 11.50, A13 11.54 — the plant uses 11.5), each carrying the GPS code strip's band;
//   * each Toggle's BODY on top of its wall (z 11.5..14), overhanging the inner face by 1.0 in (the
//     overhang is read by eye — HA-208's invented part), its two BRACKETS at |along| 10.9..12.995 in
//     hanging to 10.6 in above the floor, 1.0 in deep into the field (the depth is not drawn — the L6
//     brief's Q2 used the same 1 in);
//   * the Loaders on the RED and BLUE walls only (y = -/+70.2), 4.02 in wide, protruding 3.74 in,
//     centred 58.77 in either side of x = 0 (A9/A10), drawn 20 in tall (their real height does not
//     matter to a beam under the wall top);
//   * A12's start pieces on EVERY wall — Cup-Pin-Cup clusters standing against the wall (each piece
//     centred 1.58 in off it) at 43.50 / 46.66 / 49.82 and 90.59 / 93.75 / 96.91 in from the corner, a
//     Cup 3.15 in across, a Pin 1.6 in, both 6.5 in tall (modelled as square boxes that wide).
// A test adds whatever else it needs (`add`): a Pin standing or lying, a Cup, another robot, an object
// behind a clear panel; `clearPerimeter()` / `seeThroughWall()` change the walls.
//
// ── The sensor (SideSensorPlant) ─────────────────────────────────────────────────────────────
// A cone of rays (default +/-18 degrees, a 7 x 7 grid kept inside the circular cone; 1 = the centre
// ray only) from the TRUE lens pose, returning the NEAREST opaque hit — the register's HA-205 belief
// — or, in `blend` mode, the MEAN range over the cone's hits (the belief's alternative: VEX says the
// sensor integrates over its view). Beyond 78.74 in (2000 mm) a ray sees nothing; a cone that sees
// nothing publishes PROS's 9999 mm (393.66 in) with confidence 0. On top: the code strip's per-colour
// range bias and confidence scale (default 0 / 1 — the plant's walls read true until the brief's Step
// 0.3 gives numbers); a fixed bias; seeded Gaussian noise (sim/rng.hpp); an update interval with a hold
// between updates and a k-step latency delay line (HA-209's two unknowns); drop-outs; a FROZEN mode
// (the published value stops changing); and read faults through a hal::fake::FakeReadFaults (while
// faulted the plant does what ProsDistance does on PROS_ERR: the last good distance, confidence 0, the
// count moving at every read). Each `step(truth)` takes the harness's TRUE pose (ground truth for
// SYNTHESIS only — the drive plant's own rule) and publishes into a hal::fake::FakeDistance (F4,
// unmodified). Deterministic for a given seed; allocates (host only — never on the V5).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "shulib/hal/fake/fake_distance.hpp"
#include "shulib/hal/fake/fake_read_faults.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// One box of the host field plant (inches, field frame F1, z up from the floor).
struct FieldBox {
    /// The box's x extent.
    double xMin = 0.0;
    /// The box's x extent (upper end).
    double xMax = 0.0;
    /// The box's y extent.
    double yMin = 0.0;
    /// The box's y extent (upper end).
    double yMax = 0.0;
    /// The box's z extent (from the floor).
    double zMin = 0.0;
    /// The box's z extent (its top).
    double zMax = 0.0;
    /// A clear panel: rays pass through it (HA-206's contrary evidence).
    bool seeThrough = false;
    /// A perimeter wall: its inner face carries the GPS code strip's band.
    bool wall = false;
    /// A label for a test's failure message ("left wall", "Loader", "Pin").
    const char* name = "";
};

/// What a ray hit.
struct RayHit {
    /// Whether anything (a box or the floor) was hit within the range asked.
    bool hit = false;
    /// The distance along the ray to the hit (inches).
    double range = std::numeric_limits<double>::infinity();
    /// The hit point's height above the floor (inches).
    double z = 0.0;
    /// The hit point's coordinate along the wall it hit (for the code strip's colour).
    double along = 0.0;
    /// The box hit, or nullptr for the floor.
    const FieldBox* box = nullptr;
};

/// The host field plant's field: boxes and the floor (file header).
class FieldModel {
public:
    /// The field's inner wall face from the centre — the plant's OWN copy of A10's number:
    /// 3566.4 mm wall to wall, halved, in inches (70.2047). Never read from the library.
    static constexpr double kInnerFace = 3566.4 / 2.0 / 25.4;
    /// The perimeter walls' height (A14 11.50 in; A13 11.54 — the plant uses 11.5).
    static constexpr double kWallHeight = 11.5;
    /// The GPS code strip's band on every wall face (A8 at 300 dpi; the A4 register's HA-206).
    static constexpr double kStripLow = 9.06;
    /// The code strip band's top.
    static constexpr double kStripHigh = 11.44;
    /// The Toggle's whole length along its wall (A8: 25.99 in, caps included).
    static constexpr double kToggleLength = 25.99;
    /// Where a Toggle's body ends and its brackets begin, along the wall (A8; HA-208).
    static constexpr double kToggleBodyHalf = 10.9;
    /// How far the brackets hang: their bottom, above the floor (A8; HA-208).
    static constexpr double kBracketBottom = 10.6;
    /// How far the brackets and the body overhang reach into the field (not drawn; 1.0 in).
    static constexpr double kToggleDepth = 1.0;
    /// The Loaders' centres along the red and blue walls (A9/A10: 58.77 in either side of centre).
    static constexpr double kLoaderCentre = 58.77;
    /// A Loader's width along its wall (A9).
    static constexpr double kLoaderWidth = 4.02;
    /// How far a Loader protrudes into the field (A9).
    static constexpr double kLoaderDepth = 3.74;
    /// A Cup's diameter (the manual's 3.15 in) and a Pin's (1.6 in); both 6.5 in tall.
    static constexpr double kCupDiameter = 3.15;
    /// A Pin's diameter.
    static constexpr double kPinDiameter = 1.6;
    /// A standing piece's height (the manual: 6.5 in).
    static constexpr double kPieceHeight = 6.5;
    /// The start pieces' centres, off their wall (A12).
    static constexpr double kStartOffWall = 1.58;

    /// An empty field: the floor only.
    FieldModel() = default;

    /// The season's field (file header), every wall opaque.
    [[nodiscard]] static FieldModel override2026() {
        FieldModel f;
        const double w = kInnerFace;
        const double t = 2.0;  // wall thickness (only the inner face matters)
        f.add({-w - t, -w, -w - t, w + t, 0.0, kWallHeight, false, true, "left wall"});
        f.add({w, w + t, -w - t, w + t, 0.0, kWallHeight, false, true, "right wall"});
        f.add({-w - t, w + t, -w - t, -w, 0.0, kWallHeight, false, true, "red wall"});
        f.add({-w - t, w + t, w, w + t, 0.0, kWallHeight, false, true, "blue wall"});
        const double half = kToggleLength / 2.0;
        // Each wall: (axis 0 = an x wall at coord, 1 = a y wall), its inner face, inward sign.
        struct Side {
            int axis;
            double coord;
            double inward;
            bool loaders;
        };
        const Side sides[4] = {{0, -w, +1.0, false}, {0, w, -1.0, false}, {1, -w, +1.0, true},
                               {1, w, -1.0, true}};
        for (const Side& s : sides) {
            // A box spanning `a0..a1` along the wall, from the face `d0..d1` into the field.
            const auto box = [&](double a0, double a1, double d0, double d1, double z0, double z1,
                                 const char* name) {
                const double p0 = s.coord + s.inward * d0;
                const double p1 = s.coord + s.inward * d1;
                const double lo = std::min(p0, p1);
                const double hi = std::max(p0, p1);
                if (s.axis == 0) {
                    f.add({lo, hi, a0, a1, z0, z1, false, false, name});
                } else {
                    f.add({a0, a1, lo, hi, z0, z1, false, false, name});
                }
            };
            box(-half, half, -t, kToggleDepth, kWallHeight, kWallHeight + 2.5, "Toggle body");
            box(-half, -kToggleBodyHalf, 0.0, kToggleDepth, kBracketBottom, kWallHeight, "Toggle bracket");
            box(kToggleBodyHalf, half, 0.0, kToggleDepth, kBracketBottom, kWallHeight, "Toggle bracket");
            if (s.loaders) {
                for (const double c : {-kLoaderCentre, kLoaderCentre}) {
                    box(c - kLoaderWidth / 2.0, c + kLoaderWidth / 2.0, 0.0, kLoaderDepth, 0.0, 20.0, "Loader");
                }
            }
            // A12's clusters, measured from the corner at -w along the wall.
            const double fromCorner[6] = {43.50, 46.66, 49.82, 90.59, 93.75, 96.91};
            for (int i = 0; i < 6; ++i) {
                const bool cup = (i % 3) != 1;
                const double dia = cup ? kCupDiameter : kPinDiameter;
                const double a = -w + fromCorner[i];
                box(a - dia / 2.0, a + dia / 2.0, kStartOffWall - dia / 2.0, kStartOffWall + dia / 2.0, 0.0,
                    kPieceHeight, cup ? "start Cup" : "start Pin");
            }
        }
        return f;
    }

    /// Add a box (a piece, a robot, an object behind a panel).
    void add(const FieldBox& box) { boxes_.push_back(box); }

    /// Remove every box whose name starts with `prefix` (e.g. "start" — the start pieces moved away).
    void removeNamed(const char* prefix) {
        const auto starts = [prefix](const FieldBox& b) {
            const char* p = prefix;
            const char* n = b.name;
            while (*p != '\0') {
                if (*n != *p) {
                    return false;
                }
                ++p;
                ++n;
            }
            return true;
        };
        boxes_.erase(std::remove_if(boxes_.begin(), boxes_.end(), starts), boxes_.end());
    }

    /// Make the named perimeter wall a CLEAR panel (rays pass through it).
    void seeThroughWall(const char* name) {
        for (FieldBox& b : boxes_) {
            if (b.wall && namesEqual(b.name, name)) {
                b.seeThrough = true;
            }
        }
    }

    /// The boxes, in the order added.
    [[nodiscard]] const std::vector<FieldBox>& boxes() const noexcept { return boxes_; }

    /// Cast one ray from (ox, oy, oz) along the UNIT direction (dx, dy, dz): the nearest opaque box
    /// face or the floor within `maxRange`, or no hit. A box the origin is inside is ignored (a
    /// sensor sitting in its own robot's box).
    [[nodiscard]] RayHit cast(double ox, double oy, double oz, double dx, double dy, double dz,
                              double maxRange) const {
        RayHit best{};
        for (const FieldBox& b : boxes_) {
            if (b.seeThrough) {
                continue;
            }
            double tEnter = 0.0;
            if (!slab(ox, oy, oz, dx, dy, dz, b, tEnter)) {
                continue;
            }
            if (tEnter > 0.0 && tEnter <= maxRange && tEnter < best.range) {
                best.hit = true;
                best.range = tEnter;
                best.box = &b;
                best.z = oz + dz * tEnter;
                // The coordinate along the wall: for a box long in y it is y, else x.
                const double hx = ox + dx * tEnter;
                const double hy = oy + dy * tEnter;
                best.along = (b.yMax - b.yMin) > (b.xMax - b.xMin) ? hy : hx;
            }
        }
        if (dz < 0.0 && oz > 0.0) {
            const double tFloor = -oz / dz;
            if (tFloor <= maxRange && tFloor < best.range) {
                best.hit = true;
                best.range = tFloor;
                best.box = nullptr;
                best.z = 0.0;
                best.along = 0.0;
            }
        }
        return best;
    }

private:
    [[nodiscard]] static bool namesEqual(const char* a, const char* b) noexcept {
        while (*a != '\0' && *a == *b) {
            ++a;
            ++b;
        }
        return *a == *b;
    }

    /// The slab test: the ray enters box `b` at `tEnter` (> 0) — false when it misses, or when the
    /// origin is inside the box.
    [[nodiscard]] static bool slab(double ox, double oy, double oz, double dx, double dy, double dz,
                                   const FieldBox& b, double& tEnter) noexcept {
        double t0 = -std::numeric_limits<double>::infinity();
        double t1 = std::numeric_limits<double>::infinity();
        const double o[3] = {ox, oy, oz};
        const double d[3] = {dx, dy, dz};
        const double lo[3] = {b.xMin, b.yMin, b.zMin};
        const double hi[3] = {b.xMax, b.yMax, b.zMax};
        for (int i = 0; i < 3; ++i) {
            if (std::abs(d[i]) < 1e-12) {
                if (o[i] < lo[i] || o[i] > hi[i]) {
                    return false;
                }
                continue;
            }
            double a = (lo[i] - o[i]) / d[i];
            double c = (hi[i] - o[i]) / d[i];
            if (a > c) {
                std::swap(a, c);
            }
            t0 = std::max(t0, a);
            t1 = std::min(t1, c);
            if (t0 > t1) {
                return false;
            }
        }
        if (t0 <= 0.0) {
            return false;  // the origin is inside the box (or the box is behind the ray)
        }
        tEnter = t0;
        return true;
    }

    std::vector<FieldBox> boxes_;
};

/// The GPS code strip's effect on a ray that hits a wall face inside its band (9.06-11.44 in):
/// a range bias and a confidence scale per colour. The squares alternate every `squareSize` inches
/// along the wall (even index white, odd black). Default: no effect — the plant's walls read true
/// until the L6 brief's Step 0.3 measures one (HA-206).
struct CodeStripModel {
    /// Inches added to a ray's range on a WHITE square.
    double whiteBias = 0.0;
    /// Inches added on a BLACK square.
    double blackBias = 0.0;
    /// The confidence scale on a white square.
    double whiteConfidence = 1.0;
    /// The confidence scale on a black square.
    double blackConfidence = 1.0;
    /// The squares' size along the wall (inches).
    double squareSize = 1.0;
};

/// The side sensor's TRUE mount and behaviour (file header) — set separately from the mount the
/// robot file BELIEVES.
struct SideSensorPlantConfig {
    /// The lens's TRUE offset forward of the robot's centre (inches).
    double forward = 0.0;
    /// Its TRUE offset to the left (inches; negative = right).
    double left = 7.5;
    /// Its TRUE height above the floor (inches).
    double height = 10.5;
    /// Its TRUE facing, radians CCW from forward (+pi/2 = straight out the left side).
    double facing = 1.5707963267948966;
    /// The TRUE beam half-angle, radians (HA-205: +/-18 degrees short-range, unofficial).
    double coneHalfAngle = 18.0 * 3.14159265358979323846 / 180.0;
    /// Rays per axis of the grid (odd; 1 = the centre ray only, a "ray-centre sensor").
    int raysPerAxis = 7;
    /// false: the NEAREST hit in the cone (HA-205's belief); true: the MEAN range over the hits.
    bool blend = false;
    /// The confidence a solid return publishes (scaled by the code strip).
    double confidence = 1.0;
    /// Inches added to every TRUE range.
    double bias = 0.0;
    /// Gaussian noise's sigma on the range (inches); 0 = noiseless.
    double noiseSigma = 0.0;
    /// Steps between sensor updates; the published value HOLDS in between (HA-209). >= 1.
    int updateEvery = 1;
    /// Steps of latency: the published value is the measurement this many steps old (HA-209).
    int latencySteps = 0;
    /// The chance per update of a drop-out (published as no object).
    double dropoutProbability = 0.0;
    /// The seed for the noise and the drop-outs.
    std::uint64_t seed = 1;
    /// The code strip's per-colour effect.
    CodeStripModel strip{};
};

/// One side distance sensor on the host test plant (file header): it reads the FieldModel from the
/// TRUE pose and mount and publishes into a FakeDistance; its FakeReadFaults is the read-fault count.
class SideSensorPlant {
public:
    /// The no-object range: PROS's 9999 mm, converted (393.66 in), published with confidence 0.
    static constexpr double kNoObject = 9999.0 / 25.4;
    /// The farthest a ray sees: 2000 mm (VEX's published range).
    static constexpr double kMaxRange = 2000.0 / 25.4;

    /// Borrows `field` (it must outlive the plant).
    SideSensorPlant(const FieldModel& field, const SideSensorPlantConfig& config)
        : field_{&field}, cfg_{config}, rng_{config.seed} {
        out_.setDistance(units::Length{kNoObject});
        out_.setConfidence(0.0);
    }

    /// One step at the robot's TRUE pose: measure (on an update step), delay, publish.
    void step(const math::Pose2d& truth) {
        Measurement m = held_;
        const bool update = (steps_ % static_cast<long>(std::max(1, cfg_.updateEvery))) == 0;
        ++steps_;
        if (update && !frozen_) {
            m = measure(truth);
            held_ = m;
        }
        delay_.push_back(m);
        const std::size_t keep = static_cast<std::size_t>(std::max(0, cfg_.latencySteps)) + 1U;
        while (delay_.size() > keep) {
            delay_.erase(delay_.begin());
        }
        const Measurement pub = delay_.front();
        if (faulted_) {
            // ProsDistance on PROS_ERR: the last good distance, confidence 0, the count moving.
            out_.setDistance(units::Length{lastGood_});
            out_.setConfidence(0.0);
            return;
        }
        if (frozen_ && hasPublished_) {
            return;  // FROZEN: the published value stops changing
        }
        out_.setDistance(units::Length{pub.range});
        out_.setConfidence(pub.confidence);
        lastGood_ = pub.range;
        hasPublished_ = true;
    }

    /// One fresh measurement at `truth` (the cone, the strip, the bias, and — when configured — the
    /// noise and the drop-outs), WITHOUT publishing it or touching the hold and the delay line. A
    /// test's oracle uses a noiseless config.
    [[nodiscard]] double rangeAt(const math::Pose2d& truth) { return measure(truth).range; }

    /// The sensor as the library reads it.
    [[nodiscard]] hal::fake::FakeDistance& sensor() noexcept { return out_; }
    /// Its read-fault count.
    [[nodiscard]] hal::fake::FakeReadFaults& faults() noexcept { return faults_; }
    /// Read faults on (the device stops answering) or off.
    void fault(bool on) noexcept {
        faulted_ = on;
        faults_.failing(on);
    }
    /// FROZEN on: the published value stops changing (a sensor that stopped updating).
    void freeze(bool on) noexcept { frozen_ = on; }
    /// The config, for a test to change between steps (the mount, the bias, the strip).
    [[nodiscard]] SideSensorPlantConfig& config() noexcept { return cfg_; }

private:
    struct Measurement {
        double range = kNoObject;
        double confidence = 0.0;
    };

    [[nodiscard]] Measurement measure(const math::Pose2d& truth) {
        const double th = truth.heading().radians();
        const double c = std::cos(th);
        const double s = std::sin(th);
        const double lx = truth.x().value() + cfg_.forward * c - cfg_.left * s;
        const double ly = truth.y().value() + cfg_.forward * s + cfg_.left * c;
        const double lz = cfg_.height;
        const double centre = th + cfg_.facing;
        const int n = std::max(1, cfg_.raysPerAxis);
        const double cone = cfg_.coneHalfAngle;
        double nearest = std::numeric_limits<double>::infinity();
        double nearestScale = 1.0;
        double sum = 0.0;
        double sumScale = 0.0;
        int hits = 0;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                const double a = n == 1 ? 0.0 : cone * (2.0 * i / (n - 1) - 1.0);
                const double e = n == 1 ? 0.0 : cone * (2.0 * j / (n - 1) - 1.0);
                if (a * a + e * e > cone * cone * (1.0 + 1e-9)) {
                    continue;  // outside the circular cone
                }
                const double az = centre + a;
                const double dx = std::cos(e) * std::cos(az);
                const double dy = std::cos(e) * std::sin(az);
                const double dz = std::sin(e);
                const RayHit h = field_->cast(lx, ly, lz, dx, dy, dz, kMaxRange);
                if (!h.hit) {
                    continue;
                }
                double range = h.range;
                double scale = 1.0;
                if (h.box != nullptr && h.box->wall && h.z >= FieldModel::kStripLow
                    && h.z <= FieldModel::kStripHigh) {
                    const double sq = cfg_.strip.squareSize > 0.0 ? cfg_.strip.squareSize : 1.0;
                    const long idx = static_cast<long>(std::floor(h.along / sq));
                    const bool black = (idx % 2) != 0;
                    range += black ? cfg_.strip.blackBias : cfg_.strip.whiteBias;
                    scale = black ? cfg_.strip.blackConfidence : cfg_.strip.whiteConfidence;
                }
                ++hits;
                sum += range;
                sumScale += scale;
                if (range < nearest) {
                    nearest = range;
                    nearestScale = scale;
                }
            }
        }
        Measurement m{};
        const bool drop = cfg_.dropoutProbability > 0.0 && rng_.nextUnit() < cfg_.dropoutProbability;
        if (hits == 0 || drop) {
            return m;  // no object: 9999, confidence 0
        }
        double range = cfg_.blend ? sum / hits : nearest;
        const double scale = cfg_.blend ? sumScale / hits : nearestScale;
        range += cfg_.bias;
        if (cfg_.noiseSigma > 0.0) {
            range += cfg_.noiseSigma * rng_.nextGaussian();
        }
        m.range = range;
        m.confidence = std::clamp(cfg_.confidence * scale, 0.0, 1.0);
        return m;
    }

    const FieldModel* field_;
    SideSensorPlantConfig cfg_;
    Rng rng_;
    hal::fake::FakeDistance out_;
    hal::fake::FakeReadFaults faults_;
    std::vector<Measurement> delay_;
    Measurement held_{};
    long steps_ = 0;
    double lastGood_ = kNoObject;
    bool faulted_ = false;
    bool frozen_ = false;
    bool hasPublished_ = false;
};

}  // namespace shulib::sim
