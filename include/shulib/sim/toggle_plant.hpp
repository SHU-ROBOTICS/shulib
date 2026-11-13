#pragma once
//
// sim::TogglePlant — the HOST TOGGLE (chunk AF1S-A1, 2026-11-11; AF1's planned plant, the AF1 brief's
// §1.7, re-pointed from an Optical sensor to the camera). Say "host test plant", never "simulator": every
// number here is a TEST's choice — no Toggle has been measured (A4 register HA-231: one stroke rolls one
// face, the roll order; HA-232: it seats within the settle time after a full stroke).
//
// ── What it models ──────────────────────────────────────────────────────────────────────────────
// A Toggle on one wall with THREE coloured faces: the FIELD face (what the camera and the game read),
// the BOTTOM face, and the OUTER face. A STROKE rolls it one face the way a push from the field side
// does (the apex turns AWAY from the field; the bottom face comes up into the field — CameraPlant::
// addRolledToggle's +roll): field <- bottom <- outer <- field. Which face is the bottom at the start IS
// the roll order (the research's red-side Toggle: yellow field, BLUE bottom, red outer — Y -> B -> R;
// the blue side the mirror; the test picks).
//  * A MOTOR or PNEUMATIC flipper strokes it when its angle (the test hands the plant the flipper's TRUE
//    angle every step — sim::ArmPlant's trueDeg(), or a cylinder's extension mapped to degrees) passes
//    `rollDeg` OUTWARD while the robot's front is within `reachIn` of the wall. A stroke that peaks
//    short of `rollDeg` rolls nothing; one from farther than `reachIn` pushes air. The flipper re-arms
//    once it is back below `rearmDeg`.
//  * A PASSIVE flipper (a fixed bar) strokes it when the robot's front comes within `passiveRollIn` of
//    the wall; it re-arms once the front is back beyond `passiveRollIn + passiveRearmIn`.
//  * A roll takes `seatSeconds`: the face turns 0 -> 120 degrees (drawn mid-roll through
//    addRolledToggle) and only then SEATS. OVER-ROLL: two faces (0 -> 240). STICKY: nothing rolls, and
//    an attached flipper is stopped where it meets the Toggle (just past `contactDeg`, ArmPlant::setObstacleDeg) —
//    the push STALLS. JAMMED: the roll stops at `jamDeg` and never seats (the camera reads the face
//    it was rolling to — the AF1S brief's probe P5(c) — while the game counts it yellow).
//  * CONTACT (<SC4>b): a flipper at or past `contactDeg` within `reachIn`, a passive bar within
//    `passiveRollIn`, or the robot's front within `touchIn`.
//  * scoredColour(): the field face only when SEATED and UNTOUCHED, else yellow (<SC4>).
//  * rollNow(): an outside stroke — our OTHER robot flipping it at a scripted time (<VURS2>).
//  * flipperContactsWhileOurs(): how many times a flipper (or bar) came into contact while the TRUE
//    seated field face was `oursId` — T1's oracle (never a push on ours).
// It DRAWS its true faces into the host camera model (draw(): addToggle when seated, addRolledToggle
// mid-roll or jammed) — the test's world clears the scene and redraws every change.
//
// INDEPENDENCE: it includes none of the library's camera code — not its season data, not its tracker (a
// shared belief would cancel out in every test; a grep in the AF1S brief's checks keeps it so): the wall,
// the face geometry and the colour ids are the TEST's numbers. Ground truth for the tests only;
// unreachable from the library.

#include <cmath>
#include <cstdint>

#include "shulib/sim/camera_plant.hpp"
#include "shulib/sim/mechanism_plant.hpp"

namespace shulib::sim {

/// What the Toggle does when stroked.
enum class ToggleMode : std::uint8_t {
    Normal = 0,  ///< a stroke rolls one face and it seats after seatSeconds
    OverRoll,    ///< a stroke rolls TWO faces
    Sticky,      ///< a stroke rolls nothing; an attached flipper is stopped at contactDeg
    Jammed,      ///< a stroke rolls it to jamDeg and it sticks there, never seated
};

/// The host Toggle's numbers — the TEST's (header). Distances inches, angles degrees, times seconds.
struct TogglePlantConfig {
    double wallX = 70.2;        ///< the wall's inner face at the Toggle's centre: field x
    double wallY = 0.0;         ///< ... field y
    double inX = -1.0;          ///< the unit vector INTO the field: x
    double inY = 0.0;           ///< ... y
    double length = 25.8;       ///< the Toggle's length
    double bottom = 12.55;      ///< seated: the field face's lower edge above the floor
    double top = 14.39;         ///< seated: the apex above the floor
    double setBack = 1.2;       ///< seated: the apex's set-back behind the lower edge
    double axleZ = 13.20;       ///< mid-roll: the axle's height (addRolledToggle)
    double circumR = 1.19;      ///< mid-roll: the prism's circumradius (addRolledToggle)
    std::uint8_t fieldId = 3;   ///< the field face's colour id at the start (yellow in the scenes)
    std::uint8_t bottomId = 2;  ///< the bottom face's (the roll order: this comes up next)
    std::uint8_t outerId = 1;   ///< the outer face's
    std::uint8_t yellowId = 3;  ///< the colour id scoredColour() reports for "not set"
    ToggleMode mode = ToggleMode::Normal;  ///< what a stroke does
    double jamDeg = 70.0;       ///< JAMMED: where the roll sticks (of its 120)
    double seatSeconds = 0.2;   ///< a roll's time from the stroke to seated
    double rollDeg = 60.0;      ///< a flipper passing this angle outward rolls it
    double contactDeg = 30.0;   ///< a flipper at or past this angle touches it
    double rearmDeg = 10.0;     ///< a flipper re-arms below this angle
    double reachIn = 6.0;       ///< a flipper reaches the face only with the robot's front this close
    bool passive = false;       ///< a PASSIVE bar: the drive is the stroke
    double passiveRollIn = 5.0;   ///< passive: the front this close strokes it
    double passiveRearmIn = 3.0;  ///< passive: re-armed beyond passiveRollIn + this
    double touchIn = 1.5;       ///< the robot's front this close touches it, whatever the flipper
};

/// The host Toggle (header). Deterministic; single-threaded; test-only.
class TogglePlant {
public:
    /// A seated Toggle with `cfg`'s faces.
    explicit TogglePlant(const TogglePlantConfig& cfg) noexcept : cfg_{cfg} {
        faces_[0] = cfg.fieldId;
        faces_[1] = cfg.bottomId;
        faces_[2] = cfg.outerId;
    }

    /// Couple a motor flipper's plant: in STICKY mode it is stopped where it meets the Toggle.
    void attachArm(ArmPlant& arm) noexcept { arm_ = &arm; }
    /// Change the mode (a Toggle that becomes sticky part-way through a test).
    void setMode(ToggleMode m) noexcept { cfg_.mode = m; }
    /// The colour id `oursId` is counted against for flipperContactsWhileOurs().
    void setOurs(std::uint8_t oursId) noexcept { oursId_ = oursId; }

    /// One step of `dt`: the flipper's TRUE angle (ignored when passive) and the robot's front's TRUE
    /// distance from the wall's inner face, inches.
    void step(double dt, double flipperDeg, double frontToWallIn) {
        t_ += dt;
        // The roll in progress, and seating.
        if (rolling_) {
            const double target = rollTarget_;
            rollAt_ = std::fmin(target, rollAt_ + (target / cfg_.seatSeconds) * dt);
            if (cfg_.mode == ToggleMode::Jammed && rollAt_ >= cfg_.jamDeg) {
                rollAt_ = cfg_.jamDeg;
                jammed_ = true;
                rolling_ = false;
            } else if (rollAt_ >= target - 1e-9) {
                for (int k = 0; k < rollFaces_; ++k) {
                    rotateFaces();
                }
                rollAt_ = 0.0;
                rolling_ = false;
            }
            changed_ = true;
        }
        // Contact.
        bool contact = frontToWallIn <= cfg_.touchIn;
        bool stroke = false;
        if (cfg_.passive) {
            contact = contact || frontToWallIn <= cfg_.passiveRollIn;
            if (armed_ && frontToWallIn <= cfg_.passiveRollIn) {
                stroke = true;
                armed_ = false;
            } else if (!armed_ && frontToWallIn > cfg_.passiveRollIn + cfg_.passiveRearmIn) {
                armed_ = true;
            }
        } else {
            const bool reaches = frontToWallIn <= cfg_.reachIn;
            contact = contact || (reaches && flipperDeg >= cfg_.contactDeg);
            if (arm_ != nullptr) {  // only the obstacle THIS plant set is ever cleared
                const bool block = cfg_.mode == ToggleMode::Sticky && reaches;
                if (block && !blocking_) {
                    arm_->setObstacleDeg(cfg_.contactDeg + 0.5);  // it meets the Toggle just past contact
                } else if (!block && blocking_) {
                    arm_->clearObstacle();
                }
                blocking_ = block;
            }
            if (armed_ && reaches && flipperDeg >= cfg_.rollDeg) {
                stroke = true;
                armed_ = false;
            } else if (!armed_ && flipperDeg < cfg_.rearmDeg) {
                armed_ = true;
            }
        }
        const bool newContact = contact && !touched_;
        touched_ = contact;
        if (newContact && seatedFace() == oursId_ && oursId_ != 0) {
            ++contactsWhileOurs_;
        }
        if (stroke) {
            ++strokes_;
            beginRoll();
        }
    }

    /// An outside stroke now (our other robot): rolls as the mode says.
    void rollNow() {
        ++strokes_;
        beginRoll();
    }

    /// The TRUE field face's colour id (mid-roll: the face it is rolling FROM; jammed: likewise).
    [[nodiscard]] std::uint8_t fieldFace() const noexcept { return faces_[0]; }
    /// The colour id the game would count NOW: the field face when seated and untouched, else yellow.
    [[nodiscard]] std::uint8_t scoredColour() const noexcept {
        return seated() && !touched_ ? faces_[0] : cfg_.yellowId;
    }
    /// True when it is at rest on a face (not mid-roll, not jammed).
    [[nodiscard]] bool seated() const noexcept { return !rolling_ && !jammed_; }
    /// True while something of the robot touches it.
    [[nodiscard]] bool touched() const noexcept { return touched_; }
    /// True when it stuck part-way (JAMMED mode, after a stroke).
    [[nodiscard]] bool jammed() const noexcept { return jammed_; }
    /// Strokes that reached it (a sticky Toggle's included; air pushes are not strokes).
    [[nodiscard]] int strokes() const noexcept { return strokes_; }
    /// Faces rolled so far (an over-roll counts two).
    [[nodiscard]] int facesRolled() const noexcept { return facesRolled_; }
    /// New contacts made while the SEATED field face was `setOurs`'s colour (header).
    [[nodiscard]] int flipperContactsWhileOurs() const noexcept { return contactsWhileOurs_; }
    /// True when something changed since the last draw().
    [[nodiscard]] bool changed() const noexcept { return changed_; }

    /// Draw its true faces into `cam` (header): the seated faces, or the rolled prism mid-roll / jammed.
    void draw(CameraPlant& cam) {
        changed_ = false;
        if (seated()) {
            cam.addToggle(cfg_.wallX, cfg_.wallY, cfg_.inX, cfg_.inY, cfg_.length, cfg_.bottom, cfg_.top, cfg_.setBack,
                          faces_[0], faces_[1]);
            return;
        }
        cam.addRolledToggle(cfg_.wallX, cfg_.wallY, cfg_.inX, cfg_.inY, cfg_.length, cfg_.axleZ, cfg_.circumR,
                            rollAt_, 0.0, faces_[0], faces_[1], faces_[2]);
    }

private:
    [[nodiscard]] std::uint8_t seatedFace() const noexcept { return seated() ? faces_[0] : 0; }
    void rotateFaces() noexcept {  // field <- bottom <- outer <- field
        const std::uint8_t f = faces_[0];
        faces_[0] = faces_[1];
        faces_[1] = faces_[2];
        faces_[2] = f;
        ++facesRolled_;
    }
    void beginRoll() {
        if (cfg_.mode == ToggleMode::Sticky || jammed_ || rolling_) {
            return;  // sticky: nothing rolls; a jammed or rolling Toggle takes no new roll
        }
        rollFaces_ = cfg_.mode == ToggleMode::OverRoll ? 2 : 1;
        rollTarget_ = 120.0 * rollFaces_;
        rollAt_ = 0.0;
        rolling_ = true;
        changed_ = true;
    }

    TogglePlantConfig cfg_;
    std::uint8_t faces_[3]{};
    ArmPlant* arm_ = nullptr;
    std::uint8_t oursId_ = 0;
    double t_ = 0.0;
    bool rolling_ = false;
    bool jammed_ = false;
    bool touched_ = false;
    bool blocking_ = false;
    bool armed_ = true;
    bool changed_ = true;
    int rollFaces_ = 1;
    double rollTarget_ = 120.0;
    double rollAt_ = 0.0;
    int strokes_ = 0;
    int facesRolled_ = 0;
    int contactsWhileOurs_ = 0;
};

}  // namespace shulib::sim
