// hal::IVisionFrames — the AI Vision camera's frame seam, its settings block and its status words, and
// the host's settable camera hal::fake::FakeVisionFrames (chunk VIS1-A1, the VIS1 brief's §1.1, §1.3 and
// §2.2 "the seam").
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * EVERY refusal of a colour descriptor and of a settings block, BY NAME, on both sides of each
//     range (hue 1 and 40 accepted, 0.99 and 40.01 refused; saturation 0.1 and 1 accepted, just past
//     each refused; id 7 accepted, 0 and 8 refused; NaN refused) — and the all-zero descriptor refused as
//     UNSET, never as "id outside 1..7", because the UNSET sentence is the one that tells a team what to
//     do (mutation: the UNSET descriptor ACCEPTED).
//   * usable() is true for Ok ONLY; clipped() at each of the four edges, exactly on the margin and one
//     pixel inside it.
//   * sameAiVisionSettings compares by CONTENT (two copies of one string are equal; a changed letter,
//     id, colour, float, flag or class name is not) and ignores entries past the counts.
//   * cameraStatusWords: every state's words, ASCII only, and a byte outside the enum never invents a
//     verdict.
//   * The fake camera: a failing read is a ReadFault with NO detections (never the set frame held), and
//     its read-fault count moves WITH the fault frames it returns — the same object answers as the
//     IReadFaults (VIS1's re-check T9).

#include "doctest.h"

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "shulib/hal/fake/fake_vision_frames.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/hal/vision_frames.hpp"

using shulib::hal::AiVisionSettings;
using shulib::hal::CameraStatus;
using shulib::hal::ColourDescriptor;
using shulib::hal::Detection;
using shulib::hal::DetectionKind;
using shulib::hal::FrameStatus;
using shulib::hal::ModelCheck;
using shulib::hal::VisionFrame;
using shulib::hal::aiVisionSettingsProblem;
using shulib::hal::cameraStatusWords;
using shulib::hal::colourDescriptorProblem;
using shulib::hal::sameAiVisionSettings;

namespace {

constexpr const char* kNames[5] = {"redBluePin", "blueYellowPin", "redYellowPin", "yellowPin", "cup"};

ColourDescriptor red() { return ColourDescriptor{1, 200, 30, 40, 12.0F, 0.4F, "toggle red", "bench, 10-22"}; }

AiVisionSettings colourAndObjects() {
    AiVisionSettings s{};
    s.colours[0] = red();
    s.colours[1] = ColourDescriptor{2, 30, 60, 200, 12.0F, 0.4F, "toggle blue", "bench, 10-22"};
    s.colourCount = 2;
    s.objects = true;
    s.expectedClasses = kNames;
    s.expectedClassCount = 5;
    return s;
}

std::string problemOf(const ColourDescriptor& c) {
    const char* p = colourDescriptorProblem(c);
    return p == nullptr ? std::string{"(none)"} : std::string{p};
}

std::string problemOf(const AiVisionSettings& s) {
    const char* p = aiVisionSettingsProblem(s);
    return p == nullptr ? std::string{"(none)"} : std::string{p};
}

bool ascii(const char* s) {
    for (std::size_t i = 0; s[i] != '\0'; ++i) {
        const auto b = static_cast<unsigned char>(s[i]);
        if (b < 0x20U || b >= 0x7FU) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("VIS1-A1 seam: a colour descriptor is refused BY NAME on both sides of every range") {
    CHECK(problemOf(red()) == "(none)");

    SUBCASE("the all-zero descriptor is UNSET -- its sentence says how to measure it") {
        CHECK(problemOf(ColourDescriptor{}) ==
              "UNSET -- measure it with VEXcode's AI Vision Utility and type it");
        // A name and a provenance with the numbers still zero is still UNSET (the numbers are the
        // measurement; words are not).
        ColourDescriptor named{};
        named.name = "toggle red";
        named.provenance = "someone";
        CHECK(problemOf(named).rfind("UNSET", 0) == 0);
    }
    SUBCASE("the id: 0 with ranges set, and 8, refused; 1 and 7 accepted") {
        ColourDescriptor c = red();
        c.id = 0;
        CHECK(problemOf(c) == "id outside 1..7");
        c.id = 8;
        CHECK(problemOf(c) == "id outside 1..7");
        c.id = 255;
        CHECK(problemOf(c) == "id outside 1..7");
        c.id = 1;
        CHECK(problemOf(c) == "(none)");
        c.id = 7;
        CHECK(problemOf(c) == "(none)");
    }
    SUBCASE("the hue range: 1 and 40 accepted, 0.99, 40.01, 0 and NaN refused") {
        ColourDescriptor c = red();
        for (const float ok : {1.0F, 40.0F, 20.5F}) {
            CAPTURE(ok);
            c.hueRange = ok;
            CHECK(problemOf(c) == "(none)");
        }
        for (const float bad : {0.99F, 40.01F, 0.0F, -5.0F, std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::infinity()}) {
            CAPTURE(bad);
            c.hueRange = bad;
            CHECK(problemOf(c) == "hue range outside 1..40 degrees");
        }
    }
    SUBCASE("the saturation range: 0.1 and 1 accepted, just past each and NaN refused") {
        ColourDescriptor c = red();
        for (const float ok : {0.1F, 1.0F, 0.55F}) {
            CAPTURE(ok);
            c.saturationRange = ok;
            CHECK(problemOf(c) == "(none)");
        }
        for (const float bad : {0.0999F, 1.0001F, 0.0F, std::numeric_limits<float>::quiet_NaN()}) {
            CAPTURE(bad);
            c.saturationRange = bad;
            CHECK(problemOf(c) == "saturation range outside 0.1..1");
        }
    }
    SUBCASE("a null name; a null or empty provenance") {
        ColourDescriptor c = red();
        c.name = nullptr;
        CHECK(problemOf(c).find("name is null") != std::string::npos);
        c = red();
        c.provenance = "";
        CHECK(problemOf(c) == "no provenance -- who measured it, how, when?");
        c.provenance = nullptr;
        CHECK(problemOf(c) == "no provenance -- who measured it, how, when?");
        c = red();
        c.name = "";  // an empty word is allowed: the id still names it
        CHECK(problemOf(c) == "(none)");
    }
}

TEST_CASE("VIS1-A1 seam: a settings block is refused by name -- and a good one passes") {
    CHECK(problemOf(colourAndObjects()) == "(none)");

    SUBCASE("nothing turned on") {
        CHECK(problemOf(AiVisionSettings{}) == "nothing turned on: no colours and no objects");
    }
    SUBCASE("more than 7 colours (8), even when every one is good") {
        AiVisionSettings s = colourAndObjects();
        s.colourCount = 8;
        CHECK(problemOf(s) == "more than 7 colours");
    }
    SUBCASE("seven distinct colours are accepted") {
        AiVisionSettings s{};
        for (std::uint8_t i = 0; i < 7; ++i) {
            s.colours[i] = red();
            s.colours[i].id = static_cast<std::uint8_t>(i + 1);
        }
        s.colourCount = 7;
        CHECK(problemOf(s) == "(none)");
    }
    SUBCASE("a colour's own problem, past the first entry") {
        AiVisionSettings s = colourAndObjects();
        s.colours[1].hueRange = 41.0F;
        CHECK(problemOf(s) == "hue range outside 1..40 degrees");
        s = colourAndObjects();
        s.colourCount = 3;  // the third entry is a default-constructed descriptor: UNSET
        CHECK(problemOf(s).rfind("UNSET", 0) == 0);
    }
    SUBCASE("two colours on one id -- the camera keeps only the second write") {
        AiVisionSettings s = colourAndObjects();
        s.colours[1].id = 1;
        CHECK(problemOf(s) == "two colours share one id -- the camera keeps only the second");
    }
    SUBCASE("COLOR_MERGE with no colours") {
        AiVisionSettings s = colourAndObjects();
        s.colourCount = 0;
        s.colourMerge = true;
        CHECK(problemOf(s) == "COLOR_MERGE with no colours");
    }
    SUBCASE("objects on with no class table, an empty one, a 21st name, a null or empty name") {
        AiVisionSettings s = colourAndObjects();
        s.expectedClasses = nullptr;
        CHECK(problemOf(s) == "objects on with no expected class table -- a class id would mean nothing");
        s = colourAndObjects();
        s.expectedClassCount = 0;
        CHECK(problemOf(s) == "objects on with no expected class table -- a class id would mean nothing");
        const char* twentyOne[21] = {};
        for (auto& n : twentyOne) {
            n = "x";
        }
        s = colourAndObjects();
        s.expectedClasses = twentyOne;
        s.expectedClassCount = 21;
        CHECK(problemOf(s) == "more than 20 expected class names (the camera names at most 20)");
        s.expectedClassCount = 20;
        CHECK(problemOf(s) == "(none)");
        const char* withEmpty[3] = {"a", "", "c"};
        s.expectedClasses = withEmpty;
        s.expectedClassCount = 3;
        CHECK(problemOf(s) == "an expected class name is empty");
        const char* withNull[2] = {"a", nullptr};
        s.expectedClasses = withNull;
        s.expectedClassCount = 2;
        CHECK(problemOf(s) == "an expected class name is empty");
    }
    SUBCASE("objects OFF needs no class table; colours alone are enough") {
        AiVisionSettings s = colourAndObjects();
        s.objects = false;
        s.expectedClasses = nullptr;
        s.expectedClassCount = 0;
        CHECK(problemOf(s) == "(none)");
    }
    SUBCASE("objects alone are enough") {
        AiVisionSettings s = colourAndObjects();
        s.colourCount = 0;
        CHECK(problemOf(s) == "(none)");
    }
}

TEST_CASE("VIS1-A1 seam: usable() is Ok ONLY -- every other status is not evidence") {
    VisionFrame f{};
    CHECK(f.status == FrameStatus::NotRead);
    CHECK_FALSE(f.usable());
    for (const FrameStatus s : {FrameStatus::NotRead, FrameStatus::ReadFault, FrameStatus::PartialFault,
                                FrameStatus::NotConfigured}) {
        f.status = s;
        CHECK_FALSE(f.usable());
    }
    f.status = FrameStatus::Ok;
    CHECK(f.usable());
    // A fresh frame says nothing: no detections, not truncated, the model not asked.
    const VisionFrame fresh{};
    CHECK(fresh.count == 0);
    CHECK_FALSE(fresh.truncated);
    CHECK(fresh.model == ModelCheck::NotAsked);
    CHECK(fresh.objectsWithheld == 0);
    CHECK(fresh.coloursWithheld == 0);
    // A detection's score holds the camera's whole raw uint16 — and its "not an Object" value is -1,
    // outside that range, so no raw score can be mistaken for it.
    CHECK(Detection{}.score == -1);
    Detection big{};
    big.score = 65535;
    CHECK(big.score == 65535);
    CHECK(std::numeric_limits<decltype(Detection::score)>::max() >= 65535);
}

TEST_CASE("VIS1-A1 seam: clipped() at each edge -- exactly on the margin is clipped, one pixel inside is not") {
    // A box well inside the 320 x 240 picture.
    const Detection inside{DetectionKind::Colour, 1, 100, 100, 50, 20, 0, -1};
    CHECK_FALSE(inside.clipped());
    CHECK_FALSE(inside.clipped(0));

    // margin 1 (the default): left 1 is clipped, left 2 is not.
    Detection d = inside;
    d.left = 1;
    CHECK(d.clipped());
    d.left = 2;
    CHECK_FALSE(d.clipped());
    d = inside;
    d.top = 1;
    CHECK(d.clipped());
    d.top = 2;
    CHECK_FALSE(d.clipped());
    // the right edge: left + width = 319 is clipped (320 - 1), 318 is not
    d = inside;
    d.left = 269;
    d.width = 50;
    CHECK(d.clipped());
    d.left = 268;
    CHECK_FALSE(d.clipped());
    // the bottom edge: top + height = 239 is clipped, 238 is not
    d = inside;
    d.top = 219;
    d.height = 20;
    CHECK(d.clipped());
    d.top = 218;
    CHECK_FALSE(d.clipped());
    // margin 0: only the edge itself
    d = inside;
    d.left = 0;
    CHECK(d.clipped(0));
    d.left = 1;
    CHECK_FALSE(d.clipped(0));
    // a wider margin
    d = inside;
    d.left = 10;
    CHECK(d.clipped(10));
    CHECK_FALSE(d.clipped(9));
}

TEST_CASE("VIS1-A1 seam: sameAiVisionSettings compares CONTENT, field by field, up to the counts") {
    const AiVisionSettings a = colourAndObjects();
    AiVisionSettings b = colourAndObjects();
    CHECK(sameAiVisionSettings(a, b));
    // Two COPIES of each string (different pointers, same letters) are the same settings.
    char nameCopy[] = "toggle red";
    char provCopy[] = "bench, 10-22";
    b.colours[0].name = nameCopy;
    b.colours[0].provenance = provCopy;
    const char* namesCopy[5] = {};
    char storage[5][16] = {};
    for (std::size_t i = 0; i < 5; ++i) {
        std::strcpy(storage[i], kNames[i]);
        namesCopy[i] = storage[i];
    }
    b.expectedClasses = namesCopy;
    CHECK(sameAiVisionSettings(a, b));
    // Entries past colourCount are never written, so they never differ.
    b.colours[5] = red();
    CHECK(sameAiVisionSettings(a, b));

    SUBCASE("each field, changed alone, differs") {
        AiVisionSettings c = colourAndObjects();
        c.colours[1].green = 61;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colours[0].red = static_cast<std::uint8_t>(c.colours[0].red + 1);  // review X11: red alone
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colours[1].blue = static_cast<std::uint8_t>(c.colours[1].blue + 1);
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colours[0].id = 3;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colours[0].hueRange = 12.5F;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colours[0].saturationRange = 0.41F;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colours[0].provenance = "bench, 10-23";  // the provenance and names compared by content
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colours[0].name = "toggle RED";
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colourCount = 1;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.objects = false;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.colourMerge = true;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.expectedClassCount = 4;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        const char* other[5] = {"redBluePin", "blueYellowPin", "redYellowPin", "blueRing", "cup"};
        c = colourAndObjects();
        c.expectedClasses = other;
        CHECK_FALSE(sameAiVisionSettings(a, c));
        c = colourAndObjects();
        c.expectedClasses = nullptr;
        CHECK_FALSE(sameAiVisionSettings(a, c));
    }
    SUBCASE("a null text equals only a null text") {
        CHECK(shulib::hal::sameVisionText(nullptr, nullptr));
        CHECK_FALSE(shulib::hal::sameVisionText(nullptr, ""));
        CHECK_FALSE(shulib::hal::sameVisionText("", nullptr));
        CHECK(shulib::hal::sameVisionText("", ""));
        CHECK_FALSE(shulib::hal::sameVisionText("cup", "cups"));
        CHECK_FALSE(shulib::hal::sameVisionText("cups", "cup"));
    }
}

TEST_CASE("VIS1-A1 seam: the status words for every state the seam can report, ASCII only") {
    CameraStatus s{};
    CHECK(std::string{cameraStatusWords(s)} == "NOT set up");
    s.model = ModelCheck::Matches;  // a model verdict without a set-up is still NOT set up
    CHECK(std::string{cameraStatusWords(s)} == "NOT set up");
    s.configured = true;
    s.model = ModelCheck::NotAsked;
    CHECK(std::string{cameraStatusWords(s)} == "set up, objects off");
    s.model = ModelCheck::Matches;
    CHECK(std::string{cameraStatusWords(s)} == "set up, model OK");
    s.model = ModelCheck::Unreadable;
    CHECK(std::string{cameraStatusWords(s)} == "set up, model check pending");
    s.model = ModelCheck::Different;
    CHECK(std::string{cameraStatusWords(s)} == "set up, model DIFFERENT");
    // A byte outside the enum never reads as a verdict.
    s.model = static_cast<ModelCheck>(9);
    CHECK(std::string{cameraStatusWords(s)} == "set up, model check ?");
    for (const ModelCheck m : {ModelCheck::NotAsked, ModelCheck::Matches, ModelCheck::Different,
                               ModelCheck::Unreadable}) {
        for (const bool configured : {false, true}) {
            const CameraStatus st{configured, "", m, ""};
            CHECK(ascii(cameraStatusWords(st)));
        }
    }
}

TEST_CASE("VIS1-A1 fake camera: a failing read is a ReadFault with NO detections; the count moves with the "
          "fault frames, through the SAME object as its IReadFaults") {
    shulib::hal::fake::FakeVisionFrames cam;
    // NOT set up until told otherwise.
    CHECK(std::string{cameraStatusWords(cam.status())} == "NOT set up");
    cam.setStatus(CameraStatus{true, "", ModelCheck::Matches, ""});
    CHECK(std::string{cameraStatusWords(cam.status())} == "set up, model OK");

    VisionFrame good{};
    good.status = FrameStatus::Ok;
    good.readAt = shulib::units::Time{4.5};
    good.count = 2;
    good.detections[0] = Detection{DetectionKind::Colour, 1, 10, 20, 30, 8, 0, -1};
    good.detections[1] = Detection{DetectionKind::Object, 3, 40, 50, 10, 30, 0, 77};
    good.readSeq = 999;  // overwritten: the fake's own poll count
    cam.setNext(good);

    const shulib::hal::IReadFaults& faults = cam;
    const VisionFrame a = cam.read();
    CHECK(a.usable());
    CHECK(a.count == 2);
    CHECK(a.detections[1].score == 77);
    CHECK(a.readSeq == 1U);
    CHECK(faults.faultedReads() == 0);

    cam.setFailing(true);
    const VisionFrame bad = cam.read();
    CHECK(bad.status == FrameStatus::ReadFault);
    CHECK(bad.count == 0);              // never the set frame held over
    CHECK(bad.detections[0].width == 0);
    CHECK(bad.readSeq == 2U);
    CHECK(bad.readAt.value() == doctest::Approx(4.5));
    CHECK(faults.faultedReads() == 1);
    (void)cam.read();
    CHECK(faults.faultedReads() == 2);

    cam.setFailing(false);
    CHECK(cam.read().usable());
    CHECK(faults.faultedReads() == 2);  // a good read does not move it

    // A set frame whose own status is a fault counts too (a torn read), and the count never falls.
    VisionFrame torn = good;
    torn.status = FrameStatus::PartialFault;
    cam.setNext(torn);
    CHECK(cam.read().status == FrameStatus::PartialFault);
    CHECK(faults.faultedReads() == 3);
    VisionFrame notConfigured{};
    notConfigured.status = FrameStatus::NotConfigured;
    cam.setNext(notConfigured);
    CHECK_FALSE(cam.read().usable());
    CHECK(faults.faultedReads() == 3);  // not configured is not a read fault: nothing was read
    CHECK(cam.reads() == 6U);
}
