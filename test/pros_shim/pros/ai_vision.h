#pragma once
//
// HOST SHIM for <pros/ai_vision.h> (chunk VIS1-A1, 2026-10-18) — a programmable V5 AI Vision sensor, as
// far as the library's adapter (include/shulib/hal/pros/ai_vision.hpp) calls it.
//
// Beliefs modelled — each registered in docs/hardware-assumptions.md, each transcribed from the VENDORED
// include/pros/ai_vision.h, never invented to make an adapter pass:
//  * at most 24 objects per read (:33, AIVISION_MAX_OBJECT_COUNT, "given by VEX"); the shim can REPORT a
//    count above 24, and the adapter must treat 24 or more alike ("possibly full" — HA-198);
//  * the modes are bits TAGS 1, COLORS 2, OBJECTS 4, COLOR_MERGE 16 (:70-76), and
//    set_enabled_detection_types(bits, mask) changes ONLY the masked bits (the doc's one example,
//    :242-244 — HA-196);
//  * colour descriptor ids 1..7, hue 1..40, saturation 0.1..1 (:85-92); a second write to one id
//    overwrites the first (:340);
//  * a failed count read returns PROS_ERR (:463);
//  * the class names are read one id at a time (:371-389).
//
// WHERE THE HEADER AND THE BINARY DIFFER, THE SHIM FOLLOWS THE BINARY. The vendored firmware/libpros.a
// (kernel 4.2.2, project.pros), object vdml_ai_vision.c.o, disassembled with `arm-none-eabi-objdump -dr`
// (VIS1's brief, re-check T1/T4; re-read at VIS1-A1, 2026-10-18):
//  * aivision_get_class_name: `bl vexDeviceAiVisionClassNameGet` → `port_mutex_give` → `movs r0, #1` —
//    the SDK's value is DISCARDED and EVERY id returns PROS_SUCCESS (1), -1 included. The header's "-1
//    returns the number of class names" (:375) is documentation only: NO class count can be read through
//    PROS 4.2.2 (HA-197). PROS_ERR only when the binding or the port mutex fails (errno EACCES). What the
//    SDK writes for an id the camera does not name is UNKNOWN: the shim writes NOTHING there, so an
//    adapter that does not zero its buffer reads whatever was in it.
//  * aivision_get_object: the record is zeroed, then its id AND type are set to 127 BEFORE the port
//    claim; a failed claim returns it so — {127, 127, 0...} (HA-195).
//  * aivision_get_color: a failed claim returns id, red, green, blue = 127 and both floats +Inf
//    (0x7f800000).
//  * every setter ends `movs r0, #1`: it reports success WHATEVER the SDK did, so a read-back is the only
//    check a program has (HA-196). The knobs `modeWriteIgnored` and `colourWriteIgnored` model exactly
//    that: a write that "succeeds" and changes nothing.
//
// ADVERSARIAL DEFAULTS: a fresh shim camera has TAGS + OBJECTS ON (what Bench Tests station 12 leaves),
// holds a colour descriptor in slot 1 that nobody in this program wrote, and carries VEX's CLASSROOM AI
// model (api.vex.com) — whose id 3 is `blueRing`, where the Override model's id 3 is `yellowPin`.
// Reset with pros::shim::resetAiVisions(), which pros::shim::resetAll() (shim_control.hpp) also calls.
//
// Counters for every call the adapter must make and every call it must NEVER make (aivision_reset, the
// tag-family calls, aivision_start_awb, the USB overlay, aivision_set_code), and for id -1 asks.
//
// HONEST LIMIT: this shim tests the adapter against OUR BELIEF about PROS; it cannot test the belief
// (shim_control.hpp says it in full). The robot tests the belief (VIS1's Step 0, Bench Tests station 12).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>

#include "pros/error.h"

#define AIVISION_MAX_OBJECT_COUNT 24
#define AIVISION_MAX_CLASSNAME_COUNT 20

namespace pros {

typedef enum aivision_detected_type {
    E_AIVISION_DETECTED_COLOR = (1 << 0),
    E_AIVISION_DETECTED_CODE = (1 << 1),
    E_AIVISION_DETECTED_OBJECT = (1 << 2),
    E_AIVISION_DETECTED_TAG = (1 << 3)
} aivision_detected_type_e_t;

typedef enum aivision_mode_type {
    E_AIVISION_MODE_TAGS = (1 << 0),
    E_AIVISION_MODE_COLORS = (1 << 1),
    E_AIVISION_MODE_OBJECTS = (1 << 2),
    E_AIVISION_MODE_COLOR_MERGE = (1 << 4),
    E_AIVISION_MODE_ALL = (1 << 0) | (1 << 1) | (1 << 2),
} aivision_mode_type_e_t;

typedef struct aivision_color_s {
    uint8_t id;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    float hue_range;
    float saturation_range;
} aivision_color_s_t;

typedef struct aivision_code_s {
    uint8_t id;
    uint8_t length;
    int16_t c1;
    int16_t c2;
    int16_t c3;
    int16_t c4;
    int16_t c5;
} aivision_code_s_t;

typedef enum aivision_tag_family_e {
    TAG_CIRCLE_21H7 = 0,
    TAG_16H5 = 1,
    TAG_25H9 = 2,
    TAG_61H11 = 3
} aivision_tag_family_e_t;

typedef struct __attribute__((packed)) aivision_object_color_s {
    uint16_t xoffset;  // left edge (from camera's view)
    uint16_t yoffset;  // top edge
    uint16_t width;
    uint16_t height;
    uint16_t angle;  // angle, in tenths of a degree
} aivision_object_color_s_t;

typedef struct __attribute__((packed)) aivision_object_tag_s {
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
    int16_t x3;
    int16_t y3;
} aivision_object_tag_s_t;

typedef struct __attribute__((packed)) aivision_object_element_s {
    uint16_t xoffset;  // left
    uint16_t yoffset;  // top
    uint16_t width;
    uint16_t height;
    uint16_t score;
} aivision_object_element_s_t;

typedef struct __attribute__((packed)) aivision_object_s {
    uint8_t id;
    uint8_t type;
    union {
        aivision_object_color_s_t color;
        aivision_object_tag_s_t tag;
        aivision_object_element_s_t element;
    } object;
} aivision_object_s_t;

namespace shim {

/// One AI Vision port's programmable state.
struct AiVisionPortState {
    bool disconnected = false;           ///< every call -> PROS_ERR / the binary's failure record
    std::uint8_t modes = 0x05;           ///< ADVERSARIAL: TAGS + OBJECTS on, as station 12 leaves it
    bool modeWriteIgnored = false;       ///< a mode write "succeeds" (1) and changes nothing
    bool colourWriteFails = false;       ///< aivision_set_color -> PROS_ERR
    bool colourWriteIgnored = false;     ///< a colour write "succeeds" (1, as the binary does) and stores nothing
    bool colourReadFails = false;        ///< aivision_get_color's port claim fails: the binary's failure record
    std::array<aivision_color_s_t, 8> colours{};  ///< slots 1..7 (0 unused)
    std::array<aivision_object_s_t, 32> objects{};
    std::int32_t objectCount = 0;        ///< what get_object_count reports (may exceed 24; PROS_ERR too)
    int tornFrom = -1;                   ///< get_object(i >= tornFrom) returns the failure record
    /// The camera's class-name table. ADVERSARIAL DEFAULT: VEX's CLASSROOM model (api.vex.com) — ids that
    /// collide with the Override table's (3 = blueRing, not yellowPin).
    std::array<const char*, 20> classNames{"blueBall", "greenBall", "redBall", "blueRing", "greenRing",
                                           "redRing", "blueCube", "greenCube", "redCube"};
    std::int32_t classCount = 9;         ///< how many of classNames the camera names (NOT readable through PROS)
    bool classNamesFail = false;         ///< get_class_name -> PROS_ERR (the port claim failed)
    // call counters (what the adapter must and must not do)
    int resets = 0;            ///< aivision_reset — must stay 0
    int tagFamilyWrites = 0;   ///< aivision_set_tag_family / _override — must stay 0
    int awbStarts = 0;         ///< aivision_start_awb — must stay 0
    int overlayWrites = 0;     ///< aivision_set_usb_bounding_box_overlay — must stay 0
    int codeWrites = 0;        ///< aivision_set_code — must stay 0
    int colourWrites = 0;      ///< aivision_set_color
    int colourReads = 0;       ///< aivision_get_color
    int modeWrites = 0;        ///< aivision_set_enabled_detection_types (and enable_/disable_)
    int modeReads = 0;         ///< aivision_get_enabled_detection_types
    int countReads = 0;        ///< aivision_get_object_count
    int objectReads = 0;       ///< aivision_get_object
    int classNameReads = 0;    ///< aivision_get_class_name, any id
    int classCountAsks = 0;    ///< aivision_get_class_name with id -1 — must stay 0
    std::uint8_t lastModeBits = 0;  ///< the bits of the last mode write
    std::uint8_t lastModeMask = 0;  ///< the mask of the last mode write
};
inline std::array<AiVisionPortState, 22>& aiVisionPorts() {
    static std::array<AiVisionPortState, 22> ports{};
    return ports;
}
/// The state of physical port `port` (1..21; index 0 unused).
inline AiVisionPortState& aiVisionState(int port) {
    return aiVisionPorts()[static_cast<std::size_t>(port)];
}
/// Every AI Vision port back to its ADVERSARIAL defaults — including a descriptor nobody wrote.
inline void resetAiVisions() {
    aiVisionPorts() = {};
    for (auto& p : aiVisionPorts()) {
        p.colours[1] = aivision_color_s_t{1, 200, 200, 200, 30.0F, 0.9F};
    }
}
/// A colour blob at (x, y, w, h), descriptor `id`.
inline aivision_object_s_t colourObject(std::uint8_t id, int x, int y, int w, int h, int angle = 0) {
    aivision_object_s_t o{};
    o.id = id;
    o.type = E_AIVISION_DETECTED_COLOR;
    o.object.color = {static_cast<uint16_t>(x), static_cast<uint16_t>(y), static_cast<uint16_t>(w),
                      static_cast<uint16_t>(h), static_cast<uint16_t>(angle)};
    return o;
}
/// A colour CODE's merged blob at (x, y, w, h), code `id` (VIS1 writes no codes; one that arrives was
/// left on the camera by another program).
inline aivision_object_s_t codeObject(std::uint8_t id, int x, int y, int w, int h, int angle = 0) {
    aivision_object_s_t o = colourObject(id, x, y, w, h, angle);
    o.type = E_AIVISION_DETECTED_CODE;
    return o;
}
/// A model element of class `id` with `score`.
inline aivision_object_s_t elementObject(std::uint8_t id, int x, int y, int w, int h, int score) {
    aivision_object_s_t o{};
    o.id = id;
    o.type = E_AIVISION_DETECTED_OBJECT;
    o.object.element = {static_cast<uint16_t>(x), static_cast<uint16_t>(y), static_cast<uint16_t>(w),
                        static_cast<uint16_t>(h), static_cast<uint16_t>(score)};
    return o;
}
/// An AprilTag detection (tags are off in VIS1; a tag that arrives anyway must be skipped).
inline aivision_object_s_t tagObject(std::uint8_t id) {
    aivision_object_s_t o{};
    o.id = id;
    o.type = E_AIVISION_DETECTED_TAG;
    o.object.tag = {10, 10, 20, 10, 20, 20, 10, 20};
    return o;
}
/// The Override model's table (api.vex.com, 2026-10-15), for a test that wants the RIGHT camera.
inline void setOverrideModel(int port) {
    auto& s = aiVisionState(port);
    s.classNames = {"redBluePin", "blueYellowPin", "redYellowPin", "yellowPin", "cup"};
    s.classCount = 5;
}
}  // namespace shim

namespace c {

inline int32_t aivision_reset(uint8_t port) {
    auto& s = shim::aiVisionState(port);
    ++s.resets;
    return s.disconnected ? PROS_ERR : 1;
}
inline int32_t aivision_get_enabled_detection_types(uint8_t port) {
    auto& s = shim::aiVisionState(port);
    ++s.modeReads;
    if (s.disconnected) {
        errno = ENODEV;
        return PROS_ERR;
    }
    return s.modes;
}
inline int32_t aivision_set_enabled_detection_types(uint8_t port, uint8_t bits, uint8_t bitmask) {
    auto& s = shim::aiVisionState(port);
    ++s.modeWrites;
    s.lastModeBits = bits;
    s.lastModeMask = bitmask;
    if (s.disconnected) {
        errno = ENODEV;
        return PROS_ERR;
    }
    if (!s.modeWriteIgnored) {
        s.modes = static_cast<uint8_t>((s.modes & ~bitmask) | (bits & bitmask));
    }
    return 1;
}
// The binary: enable_ is EnableSet(t, t), disable_ is EnableSet(0, t).
inline int32_t aivision_enable_detection_types(uint8_t port, uint8_t types_mask) {
    return aivision_set_enabled_detection_types(port, types_mask, types_mask);
}
inline int32_t aivision_disable_detection_types(uint8_t port, uint8_t types_mask) {
    return aivision_set_enabled_detection_types(port, 0, types_mask);
}
inline int32_t aivision_set_tag_family_override(uint8_t port, aivision_tag_family_e_t) {
    ++shim::aiVisionState(port).tagFamilyWrites;
    return 1;
}
inline int32_t aivision_set_tag_family(uint8_t port, aivision_tag_family_e_t) {
    ++shim::aiVisionState(port).tagFamilyWrites;
    return 1;
}
inline int32_t aivision_set_color(uint8_t port, const aivision_color_s_t* color) {
    auto& s = shim::aiVisionState(port);
    ++s.colourWrites;
    if (s.disconnected || s.colourWriteFails) {
        errno = ENODEV;
        return PROS_ERR;
    }
    if (!s.colourWriteIgnored && color->id >= 1 && color->id <= 7) {
        s.colours[color->id] = *color;  // a second write to one id overwrites the first (:340)
    }
    return 1;
}
inline aivision_color_s_t aivision_get_color(uint8_t port, uint32_t id) {
    auto& s = shim::aiVisionState(port);
    ++s.colourReads;
    // A failed read, as libpros.a 4.2.2 builds it: id, red, green, blue = 127 and both floats +Inf — a
    // cable out, or (colourReadFails) a claim that fails while the writes still land. An id outside 1..7
    // is undefined behaviour (:355-357) — the shim answers the failure record there too.
    if (s.disconnected || s.colourReadFails || id < 1 || id > 7) {
        return aivision_color_s_t{127, 127, 127, 127, __builtin_inff(), __builtin_inff()};
    }
    return s.colours[id];
}
// AS THE VENDORED BINARY DOES, NOT AS THE HEADER SAYS (this file's header): 1 for EVERY id, -1
// included; nothing written for an id the camera's table does not name.
inline int32_t aivision_get_class_name(uint8_t port, int32_t id, uint8_t* class_name) {
    auto& s = shim::aiVisionState(port);
    ++s.classNameReads;
    if (id == -1) {
        ++s.classCountAsks;
    }
    if (s.disconnected || s.classNamesFail) {
        errno = EACCES;
        return PROS_ERR;
    }
    if (id < 0 || id >= s.classCount || id >= AIVISION_MAX_CLASSNAME_COUNT) {
        return 1;  // nothing written, "success"
    }
    const char* n = s.classNames[static_cast<std::size_t>(id)];
    std::size_t k = 0;
    for (; n != nullptr && n[k] != '\0' && k < 19; ++k) {
        class_name[k] = static_cast<uint8_t>(n[k]);
    }
    class_name[k] = 0;
    return 1;
}
inline int32_t aivision_set_usb_bounding_box_overlay(uint8_t port, bool) {
    ++shim::aiVisionState(port).overlayWrites;
    return 1;
}
inline int32_t aivision_start_awb(uint8_t port) {
    ++shim::aiVisionState(port).awbStarts;
    return 1;
}
inline aivision_code_s_t aivision_get_code(uint8_t, uint32_t id) {
    aivision_code_s_t code{};
    code.id = static_cast<uint8_t>(id);
    return code;
}
inline int32_t aivision_set_code(uint8_t port, const aivision_code_s_t*) {
    ++shim::aiVisionState(port).codeWrites;
    return 1;
}
inline int32_t aivision_get_object_count(uint8_t port) {
    auto& s = shim::aiVisionState(port);
    ++s.countReads;
    if (s.disconnected) {
        errno = ENODEV;
        return PROS_ERR;
    }
    return s.objectCount;
}
inline aivision_object_s_t aivision_get_object(uint8_t port, uint32_t object_index) {
    auto& s = shim::aiVisionState(port);
    ++s.objectReads;
    // A failed read, as libpros.a 4.2.2 builds it: the record zeroed, then id AND type set to 127 BEFORE
    // the port claim. What the SDK writes for an index past its own count is UNKNOWN (HA-195): the shim
    // answers the same failure record.
    aivision_object_s_t bad{};
    bad.id = 127;
    bad.type = 127;
    if (s.disconnected || object_index >= s.objects.size()
        || static_cast<int32_t>(object_index) >= s.objectCount
        || (s.tornFrom >= 0 && static_cast<int>(object_index) >= s.tornFrom)) {
        return bad;
    }
    return s.objects[object_index];
}
inline double aivision_get_temperature(uint8_t) { return 30.0; }

}  // namespace c
}  // namespace pros
