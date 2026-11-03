#pragma once
//
// IVisionFrames — the V5 AI Vision camera read a FRAME at a time (chunk VIS1-A1, 2026-10-18).
//
// ── An F4-ADDITIVE SIBLING of the frozen hal::IVision ───────────────────────────────
// hal/vision.hpp is untouched, byte for byte (F4, LOCKED 2026-06-19). This seam sits beside it, exactly
// as IDigitalOut (F11), IController (F13), IDigitalIn (F14) and IReadFaults (MECH-A1) were added beside
// the frozen set. It is NOT FROZEN (the Freeze Register says so out loud, row F15).
//
// ── Why a sibling and not IVision ───────────────────────────────────────────────────
// IVision yields a class id, a bearing and a confidence (hal/vision.hpp's ObjectObservation). It has no
// detection TYPE — a colour descriptor 1 and a model class 1 are the same `classId` —, no BOX (so no
// size, no elevation, no range) and no TIME. And the PROS AI Vision API has NO frame counter and NO
// timestamp (the vendored include/pros/ai_vision.h, read whole): freshness can only be TIME-based. So a
// frame carries the time its READ completed, stamped from the run's IClock, and a read count that
// counts the ADAPTER's polls — never camera frames (A4 register HA-192). The image itself is OLDER than
// that read by an age nobody has measured (HA-192 again): a consumer that pairs a frame with a pose
// widens the pose's bound for it (vision::boundAtImage).
//
// ── A FAILED READ IS A FRAME THAT SAYS SO, WITH NO DETECTIONS ──────────────────────
// Never the last good frame held. The F4 sensor adapters answer a PROS_ERR with their last good value
// (MECH-A1's read-fault channel exists because of it); a frame cannot, because a held frame would read
// as a fresh "nothing changed". Only `FrameStatus::Ok` is evidence of anything.
//
// ── What the adapter writes, as data ────────────────────────────────────────────────
// AiVisionSettings is the robot file's camera block's adapter half (config/camera_config.hpp carries
// it): the colour descriptors the team measured with VEXcode's AI Vision Utility, whether the AI model's
// OBJECTS are on, and the class names the model must report BY NAME before an object id means anything
// (ModelCheck; HA-197). Its refusals are named — an all-zero descriptor is UNSET, never a colour.
//
// ── status(): the set-up and the model check, through the SEAM ──────────────────────
// The Robot's boot line and a bench page report whether the camera was set up and whether its AI model
// matched — and the builder never reads a frame and must never downcast to the PROS adapter. So the
// seam carries `status()`, which touches nothing (VIS1's re-check T2).
//
// Allocation-free (a fixed 24-entry array — AIVISION_MAX_OBJECT_COUNT, ai_vision.h :33), PROS-free,
// sim-free. Single-task by contract, like every device here.

#include <cstddef>
#include <cstdint>

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// The camera's most detections per read (AIVISION_MAX_OBJECT_COUNT, the vendored ai_vision.h :33 —
/// restated so this header stays PROS-free; hal/pros/ai_vision.hpp static_asserts the two agree).
inline constexpr std::size_t kMaxVisionDetections = 24;
/// Colour descriptor ids run 1..7 (ai_vision.h :86).
inline constexpr std::uint8_t kMaxColourDescriptors = 7;
/// The most class names a camera's model can name (AIVISION_MAX_CLASSNAME_COUNT, ai_vision.h :34).
inline constexpr std::uint8_t kMaxVisionClassNames = 20;
/// The camera's report width, in pixels (api.vex.com: 320 x 240; A4 register HA-193).
inline constexpr int kVisionImageWidthPx = 320;
/// The camera's report height, in pixels (A4 register HA-193).
inline constexpr int kVisionImageHeightPx = 240;

/// What a detection IS. OURS, not PROS's bit flags — the adapter maps each. No Tag: VIS1 turns tag
/// detection OFF, and a tag that arrives anyway is counted in `VisionFrame::skipped`, never passed on.
enum class DetectionKind : std::uint8_t {
    Colour = 1,  ///< a colour descriptor's blob: `id` is the descriptor, 1..7 — one the settings wrote
    Code = 2,    ///< a colour code's merged blob, `id` 1..5 (VIS1 writes none; its adapter withholds each)
    Object = 3,  ///< the AI model's element: `id` is its class (meaningful only once ModelCheck::Matches)
};

/// One detection, copied OUT of the camera's packed record, in IMAGE pixels: +x right, +y DOWN, (0,0)
/// the top-left corner; the box's LEFT and TOP edges plus its size (ai_vision.h :129-130's "left edge"
/// / "top edge" — A4 register HA-193).
struct Detection {
    DetectionKind kind = DetectionKind::Colour;  ///< what it is
    std::uint8_t id = 0;            ///< descriptor 1..7 / code 1..5 / model class 0..
    std::int16_t left = 0;          ///< px from the image's left edge
    std::int16_t top = 0;           ///< px from the image's top edge
    std::int16_t width = 0;         ///< px
    std::int16_t height = 0;        ///< px
    std::int16_t angleTenths = 0;   ///< Colour/Code only: the blob's angle, tenths of a degree
    std::int32_t score = -1;        ///< Object only: the raw score, 0..65535 (its scale: HA-198); -1 otherwise

    /// True when the box comes within `marginPx` of an edge of the image (inclusive at both ends) —
    /// part of the thing may be outside the picture, so its size says nothing about its distance.
    [[nodiscard]] constexpr bool clipped(int marginPx = 1) const noexcept {
        return left <= marginPx || top <= marginPx ||
               left + width >= kVisionImageWidthPx - marginPx ||
               top + height >= kVisionImageHeightPx - marginPx;
    }
};

/// How a read went. ONLY `Ok` is evidence of anything.
enum class FrameStatus : std::uint8_t {
    NotRead = 0,    ///< default-constructed: no read happened
    Ok,             ///< the count and every object read answered with a valid type
    ReadFault,      ///< a read failed — the count (PROS_ERR), or a read-back the adapter's once-a-second
                    ///< check could not make: NO detections — never a held frame
    PartialFault,   ///< the count answered, an object read did not (a torn read, HA-195): not evidence
    NotConfigured,  ///< the adapter's set-up has not succeeded yet (or was lost): nothing was read
};

/// Whether the camera's AI model is the one the robot file expects, checked BY NAME (the camera's own
/// class-name table, ai_vision.h :371-389) — a model class id means nothing until this says Matches:
/// class 3 is `yellowPin` in VEX's Override model and `blueRing` in its Classroom model (A4 register
/// HA-197). Object detections are passed on ONLY while it reads Matches.
enum class ModelCheck : std::uint8_t {
    NotAsked = 0,  ///< objects are off, or no read has asked yet
    Matches,       ///< every expected class name read back, in order
    Different,     ///< the camera names another table (another model is selected on it), or nothing
    Unreadable,    ///< the class-name read failed (PROS_ERR) — the camera may still be booting
};

/// One READ of the camera — not a camera frame: the camera may update between the count read and the
/// object reads (HA-195). What ONE poll saw, and when the poll finished.
struct VisionFrame {
    FrameStatus status = FrameStatus::NotRead;  ///< only Ok is evidence
    /// The clock when the read COMPLETED — not the exposure: the IMAGE is older by an UNMEASURED age
    /// (A4 register HA-192; a consumer pairing a frame with a pose widens the pose's bound for it —
    /// vision::boundAtImage).
    units::Time readAt{};
    std::uint32_t readSeq = 0;       ///< the adapter's poll count — NOT a camera frame number (HA-192)
    std::int32_t reportedCount = 0;  ///< what the camera said it holds (it may say more than 24)
    /// POSSIBLY FULL: the camera reported `kMaxVisionDetections` OR MORE, so detections may exist that
    /// were never sent (a read of exactly 24 cannot show that the 25th was not the one that mattered —
    /// whether a camera can report more than 24 is A4 register HA-198's question; the design does not
    /// depend on the answer). Never evidence of absence.
    bool truncated = false;
    std::uint8_t count = 0;            ///< valid entries in `detections`
    std::uint8_t skipped = 0;          ///< object reads with no valid type, or a TAG (tags are off)
    ModelCheck model = ModelCheck::NotAsked;  ///< the AI model's check at this read
    std::uint8_t objectsWithheld = 0;  ///< Object detections NOT passed on (the model not Matches, or an id past the table)
    /// Colour and Code detections NOT passed on: a descriptor this program did not write (a slot another
    /// program left set), or any code (VIS1 writes none).
    std::uint8_t coloursWithheld = 0;
    Detection detections[kMaxVisionDetections]{};  ///< [0, count) are valid; their order is UNVERIFIED (HA-198)

    /// True only when this read is EVIDENCE (status Ok).
    [[nodiscard]] constexpr bool usable() const noexcept { return status == FrameStatus::Ok; }
};

/// One colour descriptor as the camera takes it (ai_vision.h :85-92): an id 1..7, an RGB, a hue
/// tolerance in degrees (1..40) and a saturation tolerance (0.1..1). EVERY FIELD IS A MEASURED FACT the
/// team types (from VEXcode's AI Vision Utility); the defaults are UNSET and refuse.
struct ColourDescriptor {
    std::uint8_t id = 0;            ///< 1..7; 0 = UNSET
    std::uint8_t red = 0;           ///< 0..255
    std::uint8_t green = 0;         ///< 0..255
    std::uint8_t blue = 0;          ///< 0..255
    float hueRange = 0.0F;          ///< degrees, 1..40; 0 = UNSET
    float saturationRange = 0.0F;   ///< 0.1..1; 0 = UNSET
    const char* name = "";          ///< the team's word for it ("toggle red"), for every log line
    const char* provenance = "";    ///< who measured it, how, when, under what light — required
};

/// Why `c` is not acceptable, or nullptr. An all-zero descriptor (id, hue and saturation all 0) is UNSET
/// and refuses; otherwise an id outside 1..7, a hue outside 1..40 or a saturation outside 0.1..1 (each
/// inclusive, NaN refused), a null name, or a missing provenance each refuses by name. Nothing inside
/// the vendored ranges is refused.
[[nodiscard]] constexpr const char* colourDescriptorProblem(const ColourDescriptor& c) noexcept {
    if (c.id == 0 && c.hueRange == 0.0F && c.saturationRange == 0.0F) {
        return "UNSET -- measure it with VEXcode's AI Vision Utility and type it";
    }
    if (c.id < 1 || c.id > kMaxColourDescriptors) {
        return "id outside 1..7";
    }
    if (!(c.hueRange >= 1.0F && c.hueRange <= 40.0F)) {
        return "hue range outside 1..40 degrees";
    }
    if (!(c.saturationRange >= 0.1F && c.saturationRange <= 1.0F)) {
        return "saturation range outside 0.1..1";
    }
    if (c.name == nullptr) {
        return "the colour's name is null -- give it a word (\"toggle red\")";
    }
    if (c.provenance == nullptr || c.provenance[0] == '\0') {
        return "no provenance -- who measured it, how, when?";
    }
    return nullptr;
}

/// What the adapter writes to the camera: EXACTLY these modes — TAGS always OFF (VIS1 reads no tags,
/// and Bench Tests station 12 leaves tags ON; a tag would take one of the 24 slots) — and these
/// descriptors.
struct AiVisionSettings {
    ColourDescriptor colours[kMaxColourDescriptors]{};  ///< [0, colourCount) are written
    std::uint8_t colourCount = 0;   ///< 0 => COLORS off
    bool objects = false;           ///< the AI model's game elements (ai_vision.h :73)
    /// With `objects`: the class names the camera's model must report, in id order (the season's table —
    /// vision/override_elements.hpp's kOverrideClassNames). Required: an object id is opaque without it
    /// (ModelCheck). BORROWED: the array and its strings must outlive every adapter built from it (a
    /// `constexpr` table does).
    const char* const* expectedClasses = nullptr;
    std::uint8_t expectedClassCount = 0;  ///< entries in expectedClasses, 1..kMaxVisionClassNames
    bool colourMerge = false;       ///< COLOR_MERGE (ai_vision.h :74)
};

/// The first problem with a settings block, or nullptr: more than 7 colours; a colour's own problem;
/// two colours on one id (the camera silently keeps the second write, ai_vision.h :340); COLOR_MERGE
/// with no colours; NOTHING turned on (a camera configured to see nothing is a wiring mistake, not a
/// choice); objects on with no class table, an empty one, more than 20 names, or a null or empty name.
[[nodiscard]] constexpr const char* aiVisionSettingsProblem(const AiVisionSettings& s) noexcept {
    if (s.colourCount > kMaxColourDescriptors) {
        return "more than 7 colours";
    }
    for (std::uint8_t i = 0; i < s.colourCount; ++i) {
        if (const char* p = colourDescriptorProblem(s.colours[i])) {
            return p;
        }
        for (std::uint8_t j = 0; j < i; ++j) {
            if (s.colours[j].id == s.colours[i].id) {
                return "two colours share one id -- the camera keeps only the second";
            }
        }
    }
    if (s.colourMerge && s.colourCount == 0) {
        return "COLOR_MERGE with no colours";
    }
    if (s.colourCount == 0 && !s.objects) {
        return "nothing turned on: no colours and no objects";
    }
    if (s.objects) {
        if (s.expectedClasses == nullptr || s.expectedClassCount == 0) {
            return "objects on with no expected class table -- a class id would mean nothing";
        }
        if (s.expectedClassCount > kMaxVisionClassNames) {
            return "more than 20 expected class names (the camera names at most 20)";
        }
        for (std::uint8_t i = 0; i < s.expectedClassCount; ++i) {
            if (s.expectedClasses[i] == nullptr || s.expectedClasses[i][0] == '\0') {
                return "an expected class name is empty";
            }
        }
    }
    return nullptr;
}

/// Two C strings equal by CONTENT (two null pointers are equal; a null and a non-null are not).
[[nodiscard]] constexpr bool sameVisionText(const char* a, const char* b) noexcept {
    if (a == nullptr || b == nullptr) {
        return a == b;
    }
    std::size_t i = 0;
    for (; a[i] != '\0' && b[i] != '\0'; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return a[i] == b[i];
}

/// Whether two settings blocks would write the SAME thing to a camera: field by field, every string by
/// content, the colours and class names only up to their counts (entries past a count are never
/// written). The device provider serves a second request for a camera only with settings equal to the
/// first's — two owners disagreeing about one camera is refused (hal/pros/device_provider.hpp).
[[nodiscard]] constexpr bool sameAiVisionSettings(const AiVisionSettings& a,
                                                  const AiVisionSettings& b) noexcept {
    if (a.colourCount != b.colourCount || a.objects != b.objects || a.colourMerge != b.colourMerge
        || a.expectedClassCount != b.expectedClassCount) {
        return false;
    }
    for (std::uint8_t i = 0; i < a.colourCount && i < kMaxColourDescriptors; ++i) {
        const ColourDescriptor& x = a.colours[i];
        const ColourDescriptor& y = b.colours[i];
        if (x.id != y.id || x.red != y.red || x.green != y.green || x.blue != y.blue
            || !(x.hueRange == y.hueRange) || !(x.saturationRange == y.saturationRange)
            || !sameVisionText(x.name, y.name) || !sameVisionText(x.provenance, y.provenance)) {
            return false;
        }
    }
    if (a.expectedClasses == nullptr || b.expectedClasses == nullptr) {
        return a.expectedClasses == b.expectedClasses || a.expectedClassCount == 0;
    }
    for (std::uint8_t i = 0; i < a.expectedClassCount; ++i) {
        if (!sameVisionText(a.expectedClasses[i], b.expectedClasses[i])) {
            return false;
        }
    }
    return true;
}

/// What the camera's adapter knows about its own set-up and AI model, WITHOUT touching the device. The
/// two strings are the adapter's own words, "" when there is nothing to say; they live as long as the
/// adapter and change only on a read() (single-task).
struct CameraStatus {
    bool configured = false;                  ///< the set-up was written AND read back
    const char* setupProblem = "";            ///< why the last set-up failed or was re-done ("" when it holds)
    ModelCheck model = ModelCheck::NotAsked;  ///< the AI model's check (NotAsked: objects off)
    const char* modelProblem = "";            ///< e.g. "class 0 is 'blueBall', the robot file expects 'redBluePin'"
};

/// The camera's state in a few ASCII words for a boot line or a bench page (the brain's screen turns
/// non-ASCII into '?'): "NOT set up", "set up, objects off", "set up, model OK", "set up, model check
/// pending" (the names did not read — re-asked once a second), "set up, model DIFFERENT". A caller that
/// needs the reason prints `setupProblem` / `modelProblem` after it.
[[nodiscard]] constexpr const char* cameraStatusWords(const CameraStatus& s) noexcept {
    if (!s.configured) {
        return "NOT set up";
    }
    switch (s.model) {
        case ModelCheck::NotAsked: return "set up, objects off";
        case ModelCheck::Matches: return "set up, model OK";
        case ModelCheck::Unreadable: return "set up, model check pending";
        case ModelCheck::Different: return "set up, model DIFFERENT";
    }
    return "set up, model check ?";  // a byte outside the enum: never invents a verdict
}

/// The camera, read a frame at a time. read() HITS THE DEVICE (one count read, then one read per
/// detection; once a second, the PROS adapter's check of its settings and model; and, after a lost
/// setting or a failed read, at most every half second, its set-up written again — A4 register
/// HA-203), so it is not const, and reading at the rate a consumer needs is the consumer's job.
/// Single-task by contract, like every device here.
class IVisionFrames {
public:
    /// Polymorphic base: delete-through-base is well-defined; the provider owns every camera.
    virtual ~IVisionFrames() = default;
    /// Default-constructible: the base has no state.
    IVisionFrames() = default;
    /// Copyable at the base (stateless); an adapter decides for itself.
    IVisionFrames(const IVisionFrames&) = default;
    /// Movable, same reasoning.
    IVisionFrames(IVisionFrames&&) = default;
    /// Copy-assignable, same reasoning.
    IVisionFrames& operator=(const IVisionFrames&) = default;
    /// Move-assignable, same reasoning.
    IVisionFrames& operator=(IVisionFrames&&) = default;

    /// Read the camera NOW (A4 register HA-203: one call per detection, plus the count; once a second
    /// the PROS adapter's check, up to 28 more; and a set-up written again, up to 36 more). A failed
    /// read is a frame whose status says so, with NO detections — never the last good frame.
    [[nodiscard]] virtual VisionFrame read() noexcept = 0;
    /// The adapter's set-up and model check as it last found them — never touches the device.
    [[nodiscard]] virtual CameraStatus status() const noexcept = 0;
};

}  // namespace shulib::hal
