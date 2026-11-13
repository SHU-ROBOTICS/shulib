#pragma once
//
// sim::CameraPlant — the HOST CAMERA MODEL (chunk VIS1-A1, 2026-10-22). Say "host test plant" and "the
// host camera model", never "simulator": nothing here is a measured camera.
//
// A scene of things on the field — coloured QUADS (a Toggle's faces, a licence plate, an LED strip, a
// cloth) and model ELEMENTS (Pins and Cups: standing, lying, or raised on a Goal; each half able to fire
// a colour) — seen through a TRUE camera (its own optics and mount, which a test sets DIFFERENT from what
// the library believes, inside the stated tolerances) from a TRUE robot pose, and turned into
// hal::VisionFrame reads with the camera's failure modes: box-edge jitter (seeded, deterministic),
// every n-th detection dropped, a camera that stops answering (a ReadFault), one that FREEZES (the last
// frame again, re-stamped — indistinguishable from a still scene, A4 register HA-192), a torn read (a
// PartialFault), a model check other than Matches (every Object withheld and counted, as the adapter
// does), and a read of 24 or more detections marked "possibly full" (`truncated`, as the adapter does).
//
// ── INDEPENDENCE (the A2 rule: the truth integrator never shares arcStep) ───────────
// This file projects with its OWN vector code in the FIELD frame and includes NOTHING from vision/. If it
// shared the library's projection (vision/camera_geometry.hpp), a sign error there would cancel in every
// test; written apart, a flipped yaw or an image read upside down shows up (VIS1's mutations MG1, MG2).
// What it cannot catch: a belief both share — +v is DOWN (A4 register HA-193), the field of view
// (A4 register HA-194: its default true camera takes VEX's 74 x 63 degrees). The robot settles those.
//
// ── NOT MODELLED (stated, not hidden) ───────────────────────────────────────────────
// Occlusion (everything is seen whole or clipped by the image's edge, never hidden by another thing — and
// there is NO WALL, which on a real field may hide a Toggle's bottom face from a low camera: the scenes
// are the worst case); lighting; motion blur; the colour classifier itself (a quad IS its descriptor's
// colour, and a lookalike fires a descriptor because the test says it does — A4 register HA-199).
// addToggle()'s bottom face is drawn at the field face's lower-edge height; the real flat bottom face sits
// about a quarter inch lower (vision/override_elements.hpp's flatBottomIn, A4 register HA-201) — an
// approximation the colour tracker's tests must not lean on to the quarter inch.
//
// Test-only: shulib/sim/ is unreachable from the library (the CI layering guard), and outside the API
// reference (test/README.md documents it for test authors).

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "shulib/hal/vision_frames.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// A 3-D point in the FIELD frame (F1's x and y; z up from the floor), inches.
struct P3 {
    double x = 0.0;  ///< field x, inches
    double y = 0.0;  ///< field y, inches
    double z = 0.0;  ///< up from the floor, inches
};

/// The TRUE camera: its field of view and its mount (body frame, from the robot's tracking centre).
struct TrueCamera {
    double hfovDeg = 74.0;   ///< the horizontal field of view, degrees
    double vfovDeg = 63.0;   ///< the vertical field of view, degrees
    double forward = 0.0;    ///< the lens forward of the tracking centre, inches
    double left = 0.0;       ///< the lens left of the tracking centre, inches
    double height = 6.0;     ///< the lens above the floor, inches
    double yawDeg = 0.0;     ///< the optical axis counter-clockwise from the robot's +X, degrees
    double pitchDeg = 0.0;   ///< the optical axis above the horizontal, degrees
};

/// One coloured QUAD in the scene: four corners and the colour descriptor id the camera reports for it
/// (0 = the camera does not fire on it).
struct ColourQuad {
    std::array<P3, 4> c{};       ///< the corners, in order around the quad
    std::uint8_t colourId = 0;   ///< the descriptor it fires; 0 = none
    bool oneSided = false;       ///< seen only from the side `normal` points to (a Toggle's face)
    P3 normal{};                 ///< the outward normal when oneSided
};

/// One model element (a Pin or a Cup): an upright cylinder — or a lying one, its axis horizontal.
struct Element {
    std::uint8_t classId = 0;     ///< the model's class id
    double x = 0.0;               ///< the base's centre: field x, inches
    double y = 0.0;               ///< the base's centre: field y, inches
    double baseZ = 0.0;           ///< the base's height: 0 on the floor, a Goal's top when raised
    double diameter = 1.6;        ///< inches (a Pin's)
    double height = 6.5;          ///< inches (a Pin's)
    bool lying = false;           ///< on its side, its axis along lyingHeadingRad
    double lyingHeadingRad = 0.0; ///< the axis's direction when lying, radians
    std::int32_t score = 90;      ///< its detection's raw score — 0..65535, the seam's range (HA-198)
    std::uint8_t lowerColourId = 0;  ///< the colour its LOWER half fires (0 = none) — a Pin's halves are lookalikes
    std::uint8_t upperColourId = 0;  ///< the colour its UPPER half fires (0 = none)
};

/// How the camera answers this read.
enum class CameraState : std::uint8_t {
    Normal,     ///< the scene, as seen
    Unplugged,  ///< a ReadFault with no detections
    Frozen,     ///< the last frame again, re-stamped (a fresh camera with no last frame reads normally)
    Torn,       ///< the scene, but a PartialFault with one skipped object
};

/// The host camera model (header). Deterministic: its jitter comes from a seeded generator.
class CameraPlant {
public:
    /// The most quads a scene holds.
    static constexpr std::size_t kMaxQuads = 48;
    /// The most elements a scene holds.
    static constexpr std::size_t kMaxElements = 32;

    /// A scene seen through `cam`, its jitter seeded with `seed`.
    explicit CameraPlant(const TrueCamera& cam, std::uint32_t seed = 1) : cam_{cam}, rng_{seed} {}

    /// Add a quad. More than kMaxQuads is a TEST's mistake, refused loudly (std::length_error).
    void addQuad(const ColourQuad& q) {
        if (nq_ >= kMaxQuads) {
            throw std::length_error("CameraPlant::addQuad: more than kMaxQuads");
        }
        quads_[nq_++] = q;
    }
    /// Add an element; more than kMaxElements is refused the same way.
    void addElement(const Element& e) {
        if (ne_ >= kMaxElements) {
            throw std::length_error("CameraPlant::addElement: more than kMaxElements");
        }
        elements_[ne_++] = e;
    }
    /// Empty the scene (the camera, its state and its last frame are kept).
    void clear() noexcept {
        nq_ = 0;
        ne_ = 0;
    }
    /// Each box edge moves by a uniform amount in [-j, +j] pixels (0 = none).
    void setJitterPx(double j) noexcept { jitter_ = j; }
    /// Drop every n-th detection, counted across a read (0 = never).
    void setDropEvery(int n) noexcept { dropEvery_ = n; }
    /// How the camera answers from now on.
    void setState(CameraState s) noexcept { state_ = s; }
    /// The model check the frames report (Matches — the Override model on the camera — by default);
    /// anything else WITHHOLDS every Object detection, counted, as the adapter does.
    void setModel(hal::ModelCheck m) noexcept { model_ = m; }
    /// The true camera, to move it between reads.
    [[nodiscard]] TrueCamera& camera() noexcept { return cam_; }
    /// How many quads the scene holds.
    [[nodiscard]] std::size_t quadCount() const noexcept { return nq_; }
    /// Quad `i` (0..quadCount()-1) — for a test that checks a scene's own geometry.
    [[nodiscard]] const ColourQuad& quad(std::size_t i) const { return quads_.at(i); }

    /// A Toggle's field-facing face as a quad: the wall's inner face runs through (wx, wy) with the field
    /// on the side (inX, inY) (a unit vector into the field); the face's lower edge sits `bottom` up at
    /// the wall line, its upper edge `top` up and `setBack` further from the field; `length` along the
    /// wall; it fires `colourId`.
    [[nodiscard]] static ColourQuad toggleFace(double wx, double wy, double inX, double inY, double length,
                                               double bottom, double top, double setBack,
                                               std::uint8_t colourId) {
        const double ax = -inY;  // along the wall
        const double ay = inX;
        const double h = length / 2.0;
        ColourQuad q;
        q.c[0] = {wx - ax * h, wy - ay * h, bottom};
        q.c[1] = {wx + ax * h, wy + ay * h, bottom};
        q.c[2] = {wx + ax * h - inX * setBack, wy + ay * h - inY * setBack, top};
        q.c[3] = {wx - ax * h - inX * setBack, wy - ay * h - inY * setBack, top};
        q.colourId = colourId;
        return q;
    }

    /// A seated Toggle: its FIELD-FACING face (`fieldColour`, leaning back, seen only from the field
    /// side) AND its BOTTOM face (`bottomColour`, horizontal at `bottom`, `bottomWidth` deep, seen only
    /// from BELOW — a camera under the lower edge sees it, close in, as a second band under the first).
    void addToggle(double wx, double wy, double inX, double inY, double length, double bottom,
                   double top, double setBack, std::uint8_t fieldColour, std::uint8_t bottomColour,
                   double bottomWidth = 2.05) {
        ColourQuad f = toggleFace(wx, wy, inX, inY, length, bottom, top, setBack, fieldColour);
        const double dh = top - bottom;
        const double n = std::hypot(dh, setBack);
        f.oneSided = true;
        f.normal = {inX * dh / n, inY * dh / n, setBack / n};
        addQuad(f);
        const double ax = -inY;
        const double ay = inX;
        const double h = length / 2.0;
        ColourQuad b;
        b.c[0] = {wx - ax * h, wy - ay * h, bottom};
        b.c[1] = {wx + ax * h, wy + ay * h, bottom};
        b.c[2] = {wx + ax * h - inX * bottomWidth, wy + ay * h - inY * bottomWidth, bottom};
        b.c[3] = {wx - ax * h - inX * bottomWidth, wy - ay * h - inY * bottomWidth, bottom};
        b.colourId = bottomColour;
        b.oneSided = true;
        b.normal = {0.0, 0.0, -1.0};
        addQuad(b);
    }

    /// A Toggle ROLLED `rollDeg` about its axle — or stuck there: its three coloured faces as one-sided
    /// quads of a SHARP equilateral prism of circumradius `circumR` about an axle `axleZ` up (pA8: the
    /// apex 1.19 in above the 13.20-in axle; the prism's vertices are the faces' colour boundaries, so
    /// each face is about 2.06 in wide — the glossary's "approximately 2.05"). Seated (roll 0) the field
    /// face's lower vertex sits on the wall's inner-face line through (wx, wy) and the apex behind it;
    /// +roll turns the apex AWAY from the field (the bottom face comes up into the field at +120), -roll
    /// toward it (the outer face comes down into the field at -120). `lift` raises the axle (a Toggle
    /// riding up in its slots mid-roll). Adds three quads: the field, bottom and outer faces, in order.
    void addRolledToggle(double wx, double wy, double inX, double inY, double length, double axleZ,
                         double circumR, double rollDeg, double lift, std::uint8_t fieldColour,
                         std::uint8_t bottomColour, std::uint8_t outerColour) {
        const double ax = -inY;
        const double ay = inX;
        const double h = length / 2.0;
        const double c30 = std::cos(30.0 * kD);
        // The axle, behind the wall line by R cos 30 (so the seated field-bottom vertex is ON it).
        const double cxw = wx - inX * circumR * c30;
        const double cyw = wy - inY * circumR * c30;
        const double zc = axleZ + lift;
        // A vertex at angle a0 (degrees from INTO the field, counter-clockwise toward up), rolled.
        auto vtx = [&](double a0, double& d, double& z) {
            const double a = (a0 + rollDeg) * kD;  // + roll: the apex (90) turns toward 90+ = away
            d = circumR * std::cos(a);
            z = zc + circumR * std::sin(a);
        };
        double dA = 0.0;
        double zA = 0.0;
        double dF = 0.0;
        double zF = 0.0;
        double dO = 0.0;
        double zO = 0.0;
        vtx(90.0, dA, zA);   // the apex
        vtx(-30.0, dF, zF);  // the field-bottom vertex
        vtx(210.0, dO, zO);  // the outer-bottom vertex
        auto face = [&](double d1, double z1, double d2, double z2, std::uint8_t id) {
            ColourQuad q;
            q.c[0] = {cxw + inX * d1 - ax * h, cyw + inY * d1 - ay * h, z1};
            q.c[1] = {cxw + inX * d1 + ax * h, cyw + inY * d1 + ay * h, z1};
            q.c[2] = {cxw + inX * d2 + ax * h, cyw + inY * d2 + ay * h, z2};
            q.c[3] = {cxw + inX * d2 - ax * h, cyw + inY * d2 - ay * h, z2};
            q.colourId = id;
            q.oneSided = true;
            const double md = 0.5 * (d1 + d2);
            const double mz = 0.5 * (z1 + z2) - zc;
            const double n = std::hypot(md, mz);
            q.normal = {inX * md / n, inY * md / n, mz / n};  // outward: the axle to the face's middle
            addQuad(q);
        };
        face(dF, zF, dA, zA, fieldColour);   // seated: faces the field, leaning back
        face(dO, zO, dF, zF, bottomColour);  // seated: horizontal, underneath
        face(dA, zA, dO, zO, outerColour);   // seated: faces out of the field
    }

    /// One read, as the camera would answer it from the TRUE pose (rx, ry inches; rh radians), stamped
    /// `now`.
    [[nodiscard]] hal::VisionFrame read(double rx, double ry, double rh, units::Time now) {
        ++reads_;
        if (state_ == CameraState::Unplugged) {
            hal::VisionFrame f{};
            f.status = hal::FrameStatus::ReadFault;
            f.readAt = now;
            f.readSeq = reads_;
            return f;
        }
        if (state_ == CameraState::Frozen && haveLast_) {
            hal::VisionFrame f = last_;
            f.readAt = now;
            f.readSeq = reads_;
            return f;
        }
        hal::VisionFrame f{};
        setUpView(rx, ry, rh);
        int k = 0;
        for (std::size_t i = 0; i < nq_; ++i) {
            if (quads_[i].colourId == 0) {
                continue;
            }
            if (quads_[i].oneSided) {  // seen only from the side its normal points to
                const P3& q0 = quads_[i].c[0];
                const P3& nn = quads_[i].normal;
                if ((C_.x - q0.x) * nn.x + (C_.y - q0.y) * nn.y + (C_.z - q0.z) * nn.z <= 0.0) {
                    continue;
                }
            }
            hal::Detection d{};
            if (!boxOf(quads_[i].c.data(), 4, d)) {
                continue;
            }
            d.kind = hal::DetectionKind::Colour;
            d.id = quads_[i].colourId;
            if (keep(++k)) {
                push(f, d);
            }
        }
        for (std::size_t i = 0; i < ne_; ++i) {
            const Element& e = elements_[i];
            P3 pts[16];
            cylinder(e, 0.0, e.height, pts);
            hal::Detection d{};
            if (boxOf(pts, 16, d)) {
                d.kind = hal::DetectionKind::Object;
                d.id = e.classId;
                d.score = e.score;
                if (keep(++k)) {
                    if (model_ == hal::ModelCheck::Matches) {
                        push(f, d);
                    } else {
                        ++f.objectsWithheld;
                        f.reportedCount += 1;
                    }
                }
            }
            const std::uint8_t halves[2] = {e.lowerColourId, e.upperColourId};
            for (int half = 0; half < 2; ++half) {
                if (halves[half] == 0) {
                    continue;
                }
                cylinder(e, half * e.height / 2.0, (half + 1) * e.height / 2.0, pts);
                hal::Detection hd{};
                if (boxOf(pts, 16, hd)) {
                    hd.kind = hal::DetectionKind::Colour;
                    hd.id = halves[half];
                    if (keep(++k)) {
                        push(f, hd);
                    }
                }
            }
        }
        f.status = hal::FrameStatus::Ok;
        f.model = model_;
        // "Possibly full" at the cap, not only above it — as the adapter does.
        f.truncated = f.reportedCount >= static_cast<std::int32_t>(hal::kMaxVisionDetections);
        if (state_ == CameraState::Torn) {
            f.status = hal::FrameStatus::PartialFault;
            f.skipped = 1;
        }
        f.readAt = now;
        f.readSeq = reads_;
        last_ = f;
        haveLast_ = true;
        return f;
    }

private:
    void setUpView(double rx, double ry, double rh) {
        const double ch = std::cos(rh);
        const double sh = std::sin(rh);
        C_ = {rx + ch * cam_.forward - sh * cam_.left, ry + sh * cam_.forward + ch * cam_.left,
              cam_.height};
        const double yaw = rh + cam_.yawDeg * kD;
        const double p = cam_.pitchDeg * kD;
        axis_ = {std::cos(p) * std::cos(yaw), std::cos(p) * std::sin(yaw), std::sin(p)};
        right_ = {std::sin(yaw), -std::cos(yaw), 0.0};
        // down = axis x right (right x down = axis: a right-handed image frame). The first draft wrote
        // right x axis — which is UP — and the hand anchor (a point above the axis lands on a SMALLER
        // row) caught it.
        down_ = {axis_.y * right_.z - axis_.z * right_.y, axis_.z * right_.x - axis_.x * right_.z,
                 axis_.x * right_.y - axis_.y * right_.x};
        fx_ = 160.0 / std::tan(cam_.hfovDeg * kD / 2.0);
        fy_ = 120.0 / std::tan(cam_.vfovDeg * kD / 2.0);
    }

    // The bounding box of `pts` in the image, clipped to it; false when a point is behind the lens or
    // nothing is in view.
    bool boxOf(const P3* pts, int n, hal::Detection& d) {
        double umin = 1e9;
        double umax = -1e9;
        double vmin = 1e9;
        double vmax = -1e9;
        for (int i = 0; i < n; ++i) {
            const P3 v{pts[i].x - C_.x, pts[i].y - C_.y, pts[i].z - C_.z};
            const double depth = v.x * axis_.x + v.y * axis_.y + v.z * axis_.z;
            if (depth < 0.5) {
                return false;  // behind or at the lens: not seen (no partial-depth clip)
            }
            const double u = 160.0 + fx_ * (v.x * right_.x + v.y * right_.y + v.z * right_.z) / depth;
            const double vv = 120.0 + fy_ * (v.x * down_.x + v.y * down_.y + v.z * down_.z) / depth;
            umin = std::fmin(umin, u);
            umax = std::fmax(umax, u);
            vmin = std::fmin(vmin, vv);
            vmax = std::fmax(vmax, vv);
        }
        umin = std::fmax(umin + jit(), 0.0);
        umax = std::fmin(umax + jit(), 319.0);
        vmin = std::fmax(vmin + jit(), 0.0);
        vmax = std::fmin(vmax + jit(), 239.0);
        if (umax - umin < 1.0 || vmax - vmin < 1.0) {
            return false;
        }
        d.left = static_cast<std::int16_t>(std::lround(umin));
        d.top = static_cast<std::int16_t>(std::lround(vmin));
        d.width = static_cast<std::int16_t>(std::lround(umax - umin));
        d.height = static_cast<std::int16_t>(std::lround(vmax - vmin));
        return true;
    }

    // Sixteen points on the cylinder between `z0` and `z1` along its axis (eight around each end).
    void cylinder(const Element& e, double z0, double z1, P3* out) const {
        const double r = e.diameter / 2.0;
        for (int i = 0; i < 8; ++i) {
            const double a = i * kPi / 4.0;
            if (!e.lying) {
                out[2 * i] = {e.x + r * std::cos(a), e.y + r * std::sin(a), e.baseZ + z0};
                out[2 * i + 1] = {e.x + r * std::cos(a), e.y + r * std::sin(a), e.baseZ + z1};
            } else {  // the axis along lyingHeading, resting on its side: its centre line at baseZ + r
                const double ux = std::cos(e.lyingHeadingRad);
                const double uy = std::sin(e.lyingHeadingRad);
                const double ox = -uy * r * std::cos(a);
                const double oy = ux * r * std::cos(a);
                const double oz = r + r * std::sin(a);
                out[2 * i] = {e.x + ux * z0 + ox, e.y + uy * z0 + oy, e.baseZ + oz};
                out[2 * i + 1] = {e.x + ux * z1 + ox, e.y + uy * z1 + oy, e.baseZ + oz};
            }
        }
    }

    double jit() {  // uniform in [-jitter, +jitter], deterministic
        rng_ = rng_ * 1664525U + 1013904223U;
        const double u = (rng_ >> 8) / 16777216.0;
        return jitter_ * (2.0 * u - 1.0);
    }
    [[nodiscard]] bool keep(int k) const noexcept { return dropEvery_ <= 0 || k % dropEvery_ != 0; }
    static void push(hal::VisionFrame& f, const hal::Detection& d) noexcept {
        if (f.count < hal::kMaxVisionDetections) {
            f.detections[f.count++] = d;
        }
        f.reportedCount += 1;
    }

    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kD = kPi / 180.0;
    TrueCamera cam_;
    std::uint32_t rng_;
    std::array<ColourQuad, kMaxQuads> quads_{};
    std::array<Element, kMaxElements> elements_{};
    std::size_t nq_ = 0;
    std::size_t ne_ = 0;
    double jitter_ = 0.0;
    int dropEvery_ = 0;
    CameraState state_ = CameraState::Normal;
    hal::ModelCheck model_ = hal::ModelCheck::Matches;
    P3 C_{};
    P3 axis_{};
    P3 right_{};
    P3 down_{};
    double fx_ = 0.0;
    double fy_ = 0.0;
    std::uint32_t reads_ = 0;
    hal::VisionFrame last_{};
    bool haveLast_ = false;
};

}  // namespace shulib::sim
