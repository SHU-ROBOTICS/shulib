#pragma once
//
// ProsAiVision — hal::IVisionFrames (and hal::IReadFaults, MECH-A1's channel) over the vendored PROS
// 4.2.2 AI Vision C API, include/pros/ai_vision.h (chunk VIS1-A1, 2026-10-18).
//
// BINDS (the C API, as Bench Tests station 12 does; the C++ pros::AIVision::get_all_objects() returns a
// std::vector, which allocates, and its header includes <expected>, a C++23 header):
//  * aivision_set_color (:352), then aivision_get_color (:369) read back — each descriptor;
//  * aivision_set_enabled_detection_types (:252) with a mask covering TAGS|COLORS|OBJECTS|COLOR_MERGE,
//    so the write sets EXACTLY the wanted modes, TAGS off; aivision_get_enabled_detection_types (:232)
//    read back (the mask's meaning rests on the doc's one example, :242-244 — A4 register HA-196);
//  * aivision_get_object_count (:463) and aivision_get_object (:479) — a READ (A4 register HA-195,
//    HA-203);
//  * aivision_get_class_name (:389) — with `objects` on, the name of each EXPECTED class id is read and
//    compared BY NAME with the robot file's table (A4 register HA-197). Until every one Matches, no
//    Object detection is passed on, and an Object whose id is past the table is NEVER passed on (its
//    name was never checked). Re-asked after every set-up and at every 1 Hz check, matching or not (the
//    doc: names may read wrong "for the first several hundred milliseconds of being plugged in",
//    :373-374; and a camera swapped while nothing read it may carry another model) — so a match is
//    never more than a second old.
//
// THERE IS NO CLASS COUNT. The header says id -1 "will return the number of class names" (:375); the
// vendored firmware/libpros.a (kernel 4.2.2; object vdml_ai_vision.c.o, aivision_get_class_name,
// disassembled with arm-none-eabi-objdump) is `bl vexDeviceAiVisionClassNameGet` → `port_mutex_give` →
// `movs r0, #1`: the SDK's value is DISCARDED and EVERY id, -1 included, returns PROS_SUCCESS (1);
// PROS_ERR only when the binding or the port mutex fails. So the check never asks for -1, zeroes the
// buffer before each read (a name the camera does not write reads as ''), and counts PROS_ERR as
// Unreadable and an empty or wrong name as Different. Where the PROS header and the binary disagree, the
// binary is the fact (VIS1's brief, re-check T1; re-read at VIS1-A1).
//
// NEVER CALLED (pinned as text by test/pros_adapter_fence_test.cpp): aivision_reset (:209 — "the
// initial state" is undocumented: it might drop the model the AI Vision Utility chose), any tag-family
// call (no tags in VIS1), aivision_start_awb (:418 — white balance is the camera's own; a balance taken
// mid-match would move every colour the team measured), the USB bounding-box overlay (a bench aid), and
// aivision_set_code (VIS1 sets no colour codes).
//
// THE SETTINGS ARE RE-WRITTEN, NEVER TRUSTED (A4 register HA-196): at construction; after ANY failed
// read (a camera re-plugged mid-run may come back without them); and whenever the 1 Hz check finds one
// changed. Each colour is read back after it is written (id and RGB compared — the two floats are NOT,
// because whether the camera stores them exactly is HA-196's question and refusing on a rounding would
// refuse a working camera). Every PROS setter returns 1 whatever the SDK did (the binary: each ends
// `movs r0, #1`), so the read-back is the only check there is. A set-up that fails is retried at most
// every kRetrySeconds and reads NotConfigured until it succeeds — never a frame from a camera configured
// by someone else.
//
// THE 1 Hz CHECK, before the first read due after each second: the modes and EVERY descriptor read
// back, then the model re-asked. A read-back that came back DIFFERENT is a lost setting (settingsLost(),
// and everything re-written); one the camera did not ANSWER (the mode read's PROS_ERR, or get_color's
// failure record — EXACTLY id, red, green, blue 127 and both floats +Inf, read in the binary; any other
// answer, an empty slot included, is compared) is a failed READ: a ReadFault frame, counted in
// faultedReads(), exactly as a failed count is — so an unplug counts the same whichever call meets it
// first. Its cost — 1 + the colours + the expected names, at most 28 device calls, once a second — is
// HA-203's, as is a set-up's (two calls per colour, two for the modes, one per expected name: at most 36,
// inside the read that finds it due). What it cannot see: a camera swapped AND answering again between
// two reads less than a second apart, with no read meeting it unplugged — its objects pass under the old
// check until the next second.
//
// A READ (A4 register HA-195 — the count and each object are separate calls; the camera may update between
// them — and A4 register HA-192: no frame counter and no timestamp, so the frame is stamped when the read
// COMPLETES): the count first, compared with PROS_ERR BEFORE anything else (PROS_ERR is INT32_MAX — a
// "count >= 24" check alone would read a failure as a full camera); a failed count is a ReadFault frame
// with NO detections, one faulted read, and the set-up marked lost. A count of 24 OR MORE is "possibly
// full" (`VisionFrame::truncated`; the vendored header calls 24 a parameter "given by VEX", :33, so a
// camera holding more may still REPORT 24 — A4 register HA-198) and reads 0..23. Each object is copied out
// BY VALUE — never a pointer or reference into the packed record (undefined on the ARM) — its box as the
// left edge, top edge, width and height (HA-193), an Object's score whole (the raw uint16; its scale is
// HA-198's). An object with no valid type (the binary's failure record is {id 127, type 127}) or a TAG is
// skipped, and a read with any skip is a PartialFault, counted: a torn read is not evidence. A colour blob
// whose descriptor THIS program did not write (a slot another program left set) and any colour CODE (VIS1
// writes none) are WITHHELD and counted (`VisionFrame::coloursWithheld`) — so every Colour detection's id
// is one the settings name.
//
// faultedReads() (IReadFaults) counts every ReadFault and PartialFault frame — monotonic, MECH-A1's
// contract — so a consumer written for that channel works unchanged. Unlike the scalar sensor adapters,
// nothing is HELD: the frame itself says the read failed (hal/vision_frames.hpp).
//
// NO HARDWARE CLAIM: host-tested through test/pros_shim/pros/ai_vision.h only, which tests this class
// against our BELIEF about PROS (HA-192..HA-198) and cannot test the belief. No AI Vision sensor has
// been read by this code; Bench Tests station 12's LIBRARY page (VIS1-B) is its first run on a robot.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/ai_vision.h"
#include "pros/error.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "shulib/hal/clock.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/hal/vision_frames.hpp"

namespace shulib::hal::pros {

static_assert(AIVISION_MAX_OBJECT_COUNT == static_cast<int>(kMaxVisionDetections),
              "the frame's capacity must equal the camera's (ai_vision.h :33)");
static_assert(AIVISION_MAX_CLASSNAME_COUNT == static_cast<int>(kMaxVisionClassNames),
              "the settings' class-name capacity must equal the camera's (ai_vision.h :34)");

/// `hal::IVisionFrames` (+ `hal::IReadFaults`) over one AI Vision sensor: the settings written and read
/// back, the AI model checked by name, a frame per read() that says how the read went (header).
/// Constructed by the device provider only (`hal::pros::ProsDeviceProvider::aiVision()`), which refuses
/// a bad settings block or a port that does not carry an AI Vision sensor first.
class ProsAiVision final : public IVisionFrames, public IReadFaults {
public:
    /// Every mode write uses this mask: the write sets EXACTLY the wanted modes, TAGS included (off).
    static constexpr std::uint8_t kModeMask = static_cast<std::uint8_t>(
        ::pros::E_AIVISION_MODE_TAGS | ::pros::E_AIVISION_MODE_COLORS |
        ::pros::E_AIVISION_MODE_OBJECTS | ::pros::E_AIVISION_MODE_COLOR_MERGE);
    /// A failed set-up is retried at most this often, in seconds (the camera may still be booting).
    static constexpr double kRetrySeconds = 0.5;
    /// While configured, the modes and every descriptor are read back, and the model re-asked, this
    /// often, in seconds (a lost setting, or another model, is found within it).
    static constexpr double kVerifySeconds = 1.0;

    /// Write the set-up NOW, on `port` (1..21), with `settings`, timing everything with `clock` (borrowed;
    /// it must outlive this adapter). PRECONDITION: `aiVisionSettingsProblem(settings) == nullptr` (the
    /// provider refuses a bad block before constructing). The settings are COPIED; the class-name table
    /// they point at is borrowed (AiVisionSettings says so).
    ProsAiVision(std::uint8_t port, const AiVisionSettings& settings, const IClock& clock)
        : port_{port}, settings_{settings}, clock_{clock}, wantedModes_{modesOf(settings)} {
        configure();
    }

    /// Non-copyable: one adapter per camera (two would fight over its settings).
    ProsAiVision(const ProsAiVision&) = delete;
    /// Non-movable: the provider holds it in place, and the status strings point into it.
    ProsAiVision(ProsAiVision&&) = delete;
    /// Non-copy-assignable, same reasoning.
    ProsAiVision& operator=(const ProsAiVision&) = delete;
    /// Non-move-assignable, same reasoning.
    ProsAiVision& operator=(ProsAiVision&&) = delete;
    /// Nothing to release: the camera keeps whatever was last written.
    ~ProsAiVision() override = default;

    /// One READ (header): the 1 Hz check of the settings and the model first, then any due re-set-up,
    /// then the count and each object. NotConfigured until a set-up succeeds; a failed count — or a
    /// read-back the 1 Hz check could not make — is a ReadFault with no detections (and the set-up
    /// marked lost); a skipped object makes the frame a PartialFault.
    [[nodiscard]] VisionFrame read() noexcept override {
        VisionFrame f{};
        f.readSeq = ++reads_;
        const units::Time now = clock_.now();
        if (configured_ && (now - lastVerify_).value() >= kVerifySeconds) {
            lastVerify_ = now;
            const Verify v = verify();
            if (v == Verify::Unanswered) {  // a FAILED READ, not a changed setting (HA-196)
                configured_ = false;
                problem_ = "the camera did not answer the 1 Hz check -- the set-up is re-written";
                ++faulted_;
                f.model = model_;
                f.status = FrameStatus::ReadFault;
                f.readAt = clock_.now();
                return f;
            }
            if (v == Verify::Changed) {
                configured_ = false;  // lost (a re-plug, another writer): write it all again
                problem_ = "the camera's settings changed under this program -- re-written";
                ++lost_;
            } else if (settings_.objects) {
                checkModel();  // a match is never more than a second old (a swapped camera, landmine 6)
            }
        }
        if (!configured_ && (now - lastAttempt_).value() >= kRetrySeconds) {
            configure();
        }
        f.model = model_;
        if (!configured_) {
            f.status = FrameStatus::NotConfigured;
            f.readAt = clock_.now();
            return f;
        }
        const std::int32_t count = ::pros::c::aivision_get_object_count(port_);
        if (count == PROS_ERR || count < 0) {  // PROS_ERR FIRST: it is INT32_MAX
            f.status = FrameStatus::ReadFault;
            f.readAt = clock_.now();
            ++faulted_;
            configured_ = false;  // HA-196: a camera that failed a read may have lost its settings
            problem_ = "a read failed (PROS_ERR) -- the set-up is re-written";
            return f;
        }
        f.reportedCount = count;
        // POSSIBLY FULL at the cap, not only above it (header; HA-198).
        f.truncated = count >= static_cast<std::int32_t>(kMaxVisionDetections);
        const int n = f.truncated ? static_cast<int>(kMaxVisionDetections) : static_cast<int>(count);
        for (int i = 0; i < n; ++i) {
            // BY VALUE: never a pointer or reference into the packed record (A4 register HA-193: the
            // box is its left edge, top edge, width and height, in image pixels).
            const ::pros::aivision_object_s_t o =
                ::pros::c::aivision_get_object(port_, static_cast<std::uint32_t>(i));
            Detection d{};
            const unsigned type = o.type;
            if (type == ::pros::E_AIVISION_DETECTED_COLOR || type == ::pros::E_AIVISION_DETECTED_CODE) {
                // Only a descriptor THIS program wrote: a slot or a colour code another program left on
                // the camera fires blobs too, and its id means nothing here (VIS1 writes no codes).
                if (type == ::pros::E_AIVISION_DETECTED_CODE || !wroteColour(o.id)) {
                    ++f.coloursWithheld;
                    continue;
                }
                d.kind = DetectionKind::Colour;
                const ::pros::aivision_object_color_s_t c = o.object.color;
                d.left = static_cast<std::int16_t>(c.xoffset);
                d.top = static_cast<std::int16_t>(c.yoffset);
                d.width = static_cast<std::int16_t>(c.width);
                d.height = static_cast<std::int16_t>(c.height);
                d.angleTenths = static_cast<std::int16_t>(c.angle);
            } else if (type == ::pros::E_AIVISION_DETECTED_OBJECT) {
                // An id means nothing until its name matched — and an id past the expected table never
                // had its name read (PROS gives no count to say how long the camera's table is).
                if (model_ != ModelCheck::Matches || o.id >= settings_.expectedClassCount) {
                    ++f.objectsWithheld;
                    continue;
                }
                d.kind = DetectionKind::Object;
                const ::pros::aivision_object_element_s_t e = o.object.element;
                d.left = static_cast<std::int16_t>(e.xoffset);
                d.top = static_cast<std::int16_t>(e.yoffset);
                d.width = static_cast<std::int16_t>(e.width);
                d.height = static_cast<std::int16_t>(e.height);
                d.score = static_cast<std::int32_t>(e.score);  // all of the raw uint16 (HA-198)
            } else {
                ++f.skipped;  // no valid type (a torn read, HA-195) or a TAG (turned off)
                continue;
            }
            d.id = o.id;
            f.detections[f.count++] = d;
        }
        if (f.skipped > 0) {
            f.status = FrameStatus::PartialFault;
            ++faulted_;
        } else {
            f.status = FrameStatus::Ok;
        }
        f.readAt = clock_.now();
        return f;
    }

    /// The set-up and the model check (IVisionFrames), for the boot line: `{configured(),
    /// setupProblem(), model(), modelProblem()}`. Touches nothing.
    [[nodiscard]] CameraStatus status() const noexcept override {
        return CameraStatus{configured_, problem_, model_, modelProblem_};
    }

    /// Reads that were not evidence because a device call failed — a failed count, a 1 Hz check the
    /// camera did not answer, or a read with a skipped object (IReadFaults). Monotonic.
    [[nodiscard]] int faultedReads() const noexcept override { return faulted_; }

    /// The AI model check's state.
    [[nodiscard]] ModelCheck model() const noexcept { return model_; }
    /// Why the model check is not Matches, in words — e.g. "class 3 is 'blueRing', the robot file
    /// expects 'yellowPin'"; "" while it Matches or was never asked.
    [[nodiscard]] const char* modelProblem() const noexcept { return modelProblem_; }
    /// True once the set-up was written AND read back; false again when it was found lost.
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    /// Why the last set-up attempt failed or was re-done ("" while it holds).
    [[nodiscard]] const char* setupProblem() const noexcept { return problem_; }
    /// How many set-up attempts were made (the boot's counts as 1; a failed one counts too).
    [[nodiscard]] int setups() const noexcept { return setups_; }
    /// How often the 1 Hz check found a setting read back DIFFERENT (a check the camera did not answer
    /// is a failed read — faultedReads() — never counted here).
    [[nodiscard]] int settingsLost() const noexcept { return lost_; }
    /// The settings this adapter writes, as it was constructed with them (the provider compares a
    /// second request's against them).
    [[nodiscard]] const AiVisionSettings& settings() const noexcept { return settings_; }

private:
    /// What the 1 Hz check found.
    enum class Verify : std::uint8_t {
        Holds,       ///< every setting read back as written
        Changed,     ///< a setting read back DIFFERENT (a re-plug, another writer)
        Unanswered,  ///< a read-back failed (PROS_ERR, or get_color's failure record): a failed READ
    };

    /// get_color's FAILURE RECORD, read in the binary (A4 register HA-196): its only failure — a failed
    /// port claim — answers EXACTLY id, red, green and blue 127 and both floats +Inf (0x7f800000). Only
    /// that record is a read that failed. Any other answer came from the SDK's success path and is
    /// COMPARED: an empty slot, whatever the SDK reports for one (the binary copies out whatever the SDK
    /// wrote; that is unread), is a setting read back DIFFERENT — never a camera that did not answer.
    [[nodiscard]] static bool colourReadFailed(const ::pros::aivision_color_s_t& r) noexcept {
        constexpr std::uint8_t kFailed = 127;
        return r.id == kFailed && r.red == kFailed && r.green == kFailed && r.blue == kFailed
               && std::isinf(r.hue_range) && !std::signbit(r.hue_range) && std::isinf(r.saturation_range)
               && !std::signbit(r.saturation_range);
    }

    /// The 1 Hz check: the modes, then every descriptor, read back against what was written.
    [[nodiscard]] Verify verify() noexcept {
        const std::int32_t m = ::pros::c::aivision_get_enabled_detection_types(port_);
        if (m == PROS_ERR) {
            return Verify::Unanswered;
        }
        if ((static_cast<std::uint32_t>(m) & kModeMask) != wantedModes_) {
            return Verify::Changed;
        }
        for (std::uint8_t i = 0; i < settings_.colourCount; ++i) {
            const ColourDescriptor& c = settings_.colours[i];
            const ::pros::aivision_color_s_t r = ::pros::c::aivision_get_color(port_, c.id);
            if (colourReadFailed(r)) {
                return Verify::Unanswered;
            }
            if (r.id != c.id || r.red != c.red || r.green != c.green || r.blue != c.blue) {
                return Verify::Changed;
            }
        }
        return Verify::Holds;
    }

    /// Whether descriptor id `id` is one this adapter writes.
    [[nodiscard]] bool wroteColour(std::uint8_t id) const noexcept {
        for (std::uint8_t i = 0; i < settings_.colourCount; ++i) {
            if (settings_.colours[i].id == id) {
                return true;
            }
        }
        return false;
    }

    /// The mode bits `s` asks for (TAGS never).
    [[nodiscard]] static std::uint32_t modesOf(const AiVisionSettings& s) noexcept {
        std::uint32_t bits = 0;
        if (s.colourCount > 0) {
            bits |= ::pros::E_AIVISION_MODE_COLORS;
        }
        if (s.objects) {
            bits |= ::pros::E_AIVISION_MODE_OBJECTS;
        }
        if (s.colourMerge) {
            bits |= ::pros::E_AIVISION_MODE_COLOR_MERGE;
        }
        return bits;
    }

    /// Write every descriptor (each read back), then the modes (read back), then — with objects on —
    /// check the model. Leaves `configured_` true only when every step held.
    void configure() noexcept {
        lastAttempt_ = clock_.now();
        lastVerify_ = lastAttempt_;
        ++setups_;
        configured_ = false;
        for (std::uint8_t i = 0; i < settings_.colourCount; ++i) {
            const ColourDescriptor& c = settings_.colours[i];
            ::pros::aivision_color_s_t w{};
            w.id = c.id;
            w.red = c.red;
            w.green = c.green;
            w.blue = c.blue;
            w.hue_range = c.hueRange;
            w.saturation_range = c.saturationRange;
            if (::pros::c::aivision_set_color(port_, &w) == PROS_ERR) {
                problem_ = "a colour write failed (PROS_ERR)";
                return;
            }
            const ::pros::aivision_color_s_t r = ::pros::c::aivision_get_color(port_, c.id);
            if (colourReadFailed(r)) {
                problem_ = "a colour read-back failed (the camera did not answer)";
                return;
            }
            if (r.id != c.id || r.red != c.red || r.green != c.green || r.blue != c.blue) {
                problem_ = "a colour read back different from what was written";
                return;
            }
        }
        if (::pros::c::aivision_set_enabled_detection_types(
                port_, static_cast<std::uint8_t>(wantedModes_), kModeMask) == PROS_ERR) {
            problem_ = "the detection-mode write failed (PROS_ERR)";
            return;
        }
        const std::int32_t m = ::pros::c::aivision_get_enabled_detection_types(port_);
        if (m == PROS_ERR || (static_cast<std::uint32_t>(m) & kModeMask) != wantedModes_) {
            problem_ = "the detection modes read back different from what was written";
            return;
        }
        problem_ = "";
        configured_ = true;
        if (settings_.objects) {
            checkModel();  // after EVERY set-up: a re-plugged camera may carry another model
        }
    }

    /// The camera's class-name table against the expected one, BY NAME (header; HA-197): each EXPECTED
    /// id, into a zeroed buffer — never id -1, which PROS 4.2.2 answers with 1, not a count.
    void checkModel() noexcept {
        for (std::uint8_t i = 0; i < settings_.expectedClassCount; ++i) {
            std::uint8_t got[64] = {};  // the doc asks for >= 20 bytes; '' if the camera writes nothing
            if (::pros::c::aivision_get_class_name(port_, i, got) == PROS_ERR) {
                model_ = ModelCheck::Unreadable;
                std::snprintf(modelProblem_, sizeof modelProblem_,
                              "class %u's name could not be read (PROS_ERR)", static_cast<unsigned>(i));
                return;
            }
            got[sizeof got - 1] = 0;
            const char* name = reinterpret_cast<const char*>(got);
            if (std::strcmp(name, settings_.expectedClasses[i]) != 0) {
                model_ = ModelCheck::Different;
                std::snprintf(modelProblem_, sizeof modelProblem_,
                              "class %u is '%.20s', the robot file expects '%.20s'",
                              static_cast<unsigned>(i), name, settings_.expectedClasses[i]);
                sanitize(modelProblem_);
                return;
            }
        }
        model_ = ModelCheck::Matches;
        modelProblem_[0] = '\0';
    }

    /// The camera's bytes reach a boot line and the brain's screen: anything outside printable ASCII
    /// becomes '?' (the screen turns non-ASCII into '?' anyway; a control byte would break a log line).
    static void sanitize(char* text) noexcept {
        for (std::size_t k = 0; text[k] != '\0'; ++k) {
            const auto byte = static_cast<unsigned char>(text[k]);
            if (byte < 0x20U || byte >= 0x7FU) {
                text[k] = '?';
            }
        }
    }

    std::uint8_t port_;
    AiVisionSettings settings_;
    const IClock& clock_;
    std::uint32_t wantedModes_;
    bool configured_ = false;
    units::Time lastAttempt_{};
    units::Time lastVerify_{};
    const char* problem_ = "not yet configured";
    std::uint32_t reads_ = 0;
    int faulted_ = 0;
    int setups_ = 0;
    int lost_ = 0;
    ModelCheck model_ = ModelCheck::NotAsked;
    char modelProblem_[96] = "";
};

}  // namespace shulib::hal::pros
