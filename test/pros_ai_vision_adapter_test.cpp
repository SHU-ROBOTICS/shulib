// hal::pros::ProsAiVision THROUGH THE HOST SHIM, and ProsDeviceProvider::aiVision() (chunk VIS1-A1, the
// VIS1 brief's §1.2, §1.4, §1.6 and §2.2 "the adapter over the shim" and "the provider").
//
// The shim (test/pros_shim/pros/ai_vision.h) answers as the VENDORED libpros.a 4.2.2 does where the
// header and the binary differ — get_class_name returns 1 for EVERY id and writes nothing for one the
// camera does not name; a failed get_object is {127, 127, 0...}; a failed get_color is 127s and +Inf —
// and starts ADVERSARIAL: TAGS + OBJECTS on (what Bench Tests station 12 leaves), a stray descriptor in
// slot 1, and VEX's CLASSROOM model (its class 3 is `blueRing`, the Override model's is `yellowPin`).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * THE SET-UP: exactly COLORS + OBJECTS written with the ALL-MODES mask (TAGS off), every descriptor
//     read back — EACH of its id, red, green and blue compared on its own —, a write that "succeeds" and
//     changes nothing caught by its read-back, get_color's failure record (the binary's 127s and +Inf)
//     refusing the set-up, a failed set-up NotConfigured and retried no sooner than 0.5 s (adapter
//     mutations M1, M5, M6, M13; the review's X3-X5).
//   * THE 1 Hz CHECK: the modes AND every descriptor read back and the model re-asked, landing EXACTLY on
//     1 s and re-armed after a check that held (M7, the review's X21); a camera swapped while nothing read
//     it — the modes already right — caught BEFORE its objects pass; names that read WRONG at boot
//     (Different, the binary's realistic path) re-asked into Matches; an unplug the check meets first is a
//     FAILED READ (faultedReads), never a lost setting, exactly as one the count meets; EACH read-back
//     field compared on its own; ONLY get_color's exact failure record a failed read (a wiped slot, or
//     the record with one field off, is a lost setting — the verification round); descriptor 7 holds.
//   * THE READ: a PROS_ERR count is a ReadFault with NO detections — never "2,147,483,647 detections,
//     possibly full" — and re-writes the set-up (M2, the run's own "PROS_ERR treated as a number"); 24
//     or more is POSSIBLY FULL and exactly 24 objects are read (M3, M14); a torn read or a TAG is a
//     PartialFault, not evidence (M4); every record copied by value — an Object's four box values each
//     checked (X8, X9), its raw score WHOLE (65535 is never -1) —, stamped when the read COMPLETED, through
//     a clock that moves at every reading (X7); a colour from a slot this program did not write, and any
//     colour CODE, withheld and counted.
//   * THE MODEL, BY NAME: the Classroom model withholds every Object and keeps the colours; one wrong
//     name of five is caught; an unreadable table is pending, never a match; a camera that names
//     nothing is Different; id -1 is never asked; an Object past the table is withheld even when the
//     model matches; a camera re-plugged with another model is Different again (M8-M12); objects OFF never
//     asks a name, not at the 1 Hz check either (X6); a name's bytes reach the boot line printable, DEL
//     included (X1).
//   * NEVER CALLED: aivision_reset, any tag-family call, aivision_start_awb, the USB overlay,
//     aivision_set_code — the counters stay 0 through EVERY case here (and the adapter fence test pins
//     the source text).
//   * THE PROVIDER: a port outside 1..21, an empty port, a motor and the OLDER Vision sensor — every
//     kind the brain reports — refused by name; a settings problem refused before the device is
//     touched; the SAME camera for equal settings (and nothing written again — the run's own "a second
//     adapter" mutation); a refusal for different ones; readFaults(port) nullptr until it is made.
//
// NO HARDWARE CLAIM: the shim tests the adapter against our BELIEF about PROS (HA-192..HA-198); it
// cannot test the belief. No AI Vision sensor has been read by this code.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "pros/shim_control.hpp"

#include "shulib/hal/clock.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/pros/ai_vision.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/override_elements.hpp"

using shulib::hal::AiVisionSettings;
using shulib::hal::DetectionKind;
using shulib::hal::FrameStatus;
using shulib::hal::ModelCheck;
using shulib::hal::VisionFrame;
using shulib::hal::fake::FakeClock;
using shulib::hal::pros::ProsAiVision;
using shulib::hal::pros::ProsDeviceProvider;
using shulib::units::Time;
namespace shim = pros::shim;
namespace override2026 = shulib::vision::override2026;

namespace {

AiVisionSettings threeColoursAndObjects() {
    AiVisionSettings s{};
    s.colours[0] = {1, 200, 30, 40, 12.0F, 0.4F, "toggle red", "host test"};
    s.colours[1] = {2, 30, 60, 200, 12.0F, 0.4F, "toggle blue", "host test"};
    s.colours[2] = {3, 230, 200, 40, 10.0F, 0.4F, "toggle yellow", "host test"};
    s.colourCount = 3;
    s.objects = true;
    s.expectedClasses = override2026::kOverrideClassNames;
    s.expectedClassCount = override2026::kModelClassCount;
    return s;
}

AiVisionSettings objectsOnly() {
    AiVisionSettings s{};
    s.objects = true;
    s.expectedClasses = override2026::kOverrideClassNames;
    s.expectedClassCount = override2026::kModelClassCount;
    return s;
}

/// The camera carries the Override model (the shim's default is VEX's Classroom model).
void overrideModel(int port) {
    auto& st = shim::aiVisionState(port);
    st.classNames = {};
    for (std::size_t i = 0; i < 5; ++i) {
        st.classNames[i] = override2026::kOverrideClassNames[i];
    }
    st.classCount = 5;
}

/// The calls the adapter must NEVER make, on `port`, all still zero.
void noForbiddenCalls(int port) {
    const auto& s = shim::aiVisionState(port);
    CHECK(s.resets == 0);
    CHECK(s.tagFamilyWrites == 0);
    CHECK(s.awbStarts == 0);
    CHECK(s.overlayWrites == 0);
    CHECK(s.codeWrites == 0);
    CHECK(s.classCountAsks == 0);
}

bool contains(const char* haystack, const char* needle) {
    return haystack != nullptr && std::string{haystack}.find(needle) != std::string::npos;
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

constexpr std::uint8_t kColorsObjects =
    static_cast<std::uint8_t>(pros::E_AIVISION_MODE_COLORS | pros::E_AIVISION_MODE_OBJECTS);

/// An IClock that moves 1 ms forward EVERY time it is read — so a stamp taken at a read's START differs
/// from one taken at its END (review X7: with a clock that never moves inside read(), the two cannot be
/// told apart).
class TickingClock final : public shulib::hal::IClock {
public:
    [[nodiscard]] Time now() const override {
        last_ = next_;
        next_ += 0.001;
        ++readings_;
        return Time{last_};
    }
    /// The value the last now() returned.
    [[nodiscard]] double last() const { return last_; }
    /// How many times now() was called.
    [[nodiscard]] int readings() const { return readings_; }

private:
    mutable double next_ = 10.0;
    mutable double last_ = 0.0;
    mutable int readings_ = 0;
};

}  // namespace

// ═══ The set-up ════════════════════════════════════════════════════════════════════

TEST_CASE("ProsAiVision P1.1: the set-up writes EXACTLY colours + objects with the all-modes mask -- TAGS "
          "off -- and every descriptor") {
    // BUG CAUGHT (M1): a mask that leaves TAGS alone keeps station 12's tags on, and a tag takes one of
    // the 24 slots of every read.
    shim::resetAiVisions();
    FakeClock clk;
    auto& s = shim::aiVisionState(3);
    CHECK(s.modes == 0x05);  // as station 12 leaves it: TAGS + OBJECTS
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    CHECK(cam.configured());
    CHECK(std::string{cam.setupProblem()}.empty());
    CHECK(cam.setups() == 1);
    CHECK(s.modes == kColorsObjects);
    CHECK(s.lastModeBits == kColorsObjects);
    CHECK(s.lastModeMask == 0x17);  // TAGS | COLORS | OBJECTS | COLOR_MERGE
    CHECK(ProsAiVision::kModeMask == 0x17);
    CHECK(s.colourWrites == 3);
    CHECK(s.colourReads == 3);
    CHECK(s.colours[1].red == 200);  // the stray descriptor in slot 1 overwritten
    CHECK(s.colours[1].green == 30);
    CHECK(s.colours[2].blue == 200);
    CHECK(s.colours[3].green == 200);
    CHECK(s.colours[1].hue_range == doctest::Approx(12.0));
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.1b: COLOR_MERGE and objects-off each reach the mode bits exactly") {
    shim::resetAiVisions();
    FakeClock clk;
    AiVisionSettings s = threeColoursAndObjects();
    s.objects = false;
    s.colourMerge = true;
    ProsAiVision cam{4, s, clk};
    CHECK(cam.configured());
    CHECK(shim::aiVisionState(4).modes ==
          (pros::E_AIVISION_MODE_COLORS | pros::E_AIVISION_MODE_COLOR_MERGE));
    CHECK(cam.model() == ModelCheck::NotAsked);
    CHECK(shim::aiVisionState(4).classNameReads == 0);  // objects off: the model is never asked
    // ... not at the 1 Hz check either (review X6: a re-ask without the `objects` guard would read the
    // names of a model this camera does not use, every second, and change its status words).
    clk.advance(Time{1.0});
    CHECK(cam.read().usable());
    CHECK(shim::aiVisionState(4).modeReads == 2);  // the 1 Hz check DID run (the set-up's read + this)
    CHECK(shim::aiVisionState(4).classNameReads == 0);
    CHECK(cam.model() == ModelCheck::NotAsked);
    CHECK(std::string{shulib::hal::cameraStatusWords(cam.status())} == "set up, objects off");
    ProsAiVision objectsCam{5, objectsOnly(), clk};
    CHECK(shim::aiVisionState(5).modes == pros::E_AIVISION_MODE_OBJECTS);
    CHECK(shim::aiVisionState(5).colourWrites == 0);
    noForbiddenCalls(4);
    noForbiddenCalls(5);
}

TEST_CASE("ProsAiVision P1.7: a failed colour write -- NotConfigured, NOTHING read, retried no sooner "
          "than 0.5 s") {
    // BUG CAUGHT (M6): a set-up retried on every read hammers a booting camera with writes.
    shim::resetAiVisions();
    FakeClock clk;
    auto& s = shim::aiVisionState(3);
    s.colourWriteFails = true;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    CHECK_FALSE(cam.configured());
    CHECK(std::string{cam.setupProblem()} == "a colour write failed (PROS_ERR)");
    const int counts0 = s.countReads;
    const int writes0 = s.colourWrites;
    const VisionFrame f = cam.read();
    CHECK(f.status == FrameStatus::NotConfigured);
    CHECK(f.count == 0);
    CHECK(s.countReads == counts0);  // nothing was read
    CHECK(s.colourWrites == writes0);  // and not retried at once
    s.colourWriteFails = false;
    clk.advance(Time{0.25});
    CHECK(cam.read().status == FrameStatus::NotConfigured);  // too soon to retry
    CHECK(s.colourWrites == writes0);
    clk.advance(Time{0.25});  // EXACTLY 0.5 s after the attempt (binary-exact): due
    CHECK(cam.read().usable());
    CHECK(cam.setups() == 2);
    CHECK(cam.faultedReads() == 0);  // NotConfigured reads are not read faults
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.8: a mode write that 'succeeds' and changes nothing is caught by the read-back") {
    // BUG CAUGHT (M5): PROS's setters return 1 whatever the SDK did (the binary) — only the read-back
    // can see a camera that kept station 12's tags on.
    shim::resetAiVisions();
    FakeClock clk;
    shim::aiVisionState(3).modeWriteIgnored = true;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    CHECK_FALSE(cam.configured());
    CHECK(std::string{cam.setupProblem()} == "the detection modes read back different from what was written");
    CHECK(cam.read().status == FrameStatus::NotConfigured);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.19: a colour write that 'succeeds' and stores nothing is caught by the read-back") {
    // BUG CAUGHT (M13): without the read-back compare, the stray descriptor in slot 1 ({200,200,200},
    // which nobody in this program wrote) would be the camera's "toggle red".
    shim::resetAiVisions();
    shim::aiVisionState(3).colourWriteIgnored = true;
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    CHECK_FALSE(cam.configured());
    CHECK(std::string{cam.setupProblem()} == "a colour read back different from what was written");
    CHECK(cam.read().status == FrameStatus::NotConfigured);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: the colour read-back compares the id and the RGB, NOT the two floats") {
    // The floats' storage is HA-196's open question; refusing a working camera over a rounding would cost
    // the colours for nothing. The camera here keeps the right id and RGB with different floats.
    shim::resetAiVisions();
    auto& s = shim::aiVisionState(3);
    s.colourWriteIgnored = true;
    s.colours[1] = pros::aivision_color_s_t{1, 200, 30, 40, 11.9F, 0.39F};
    FakeClock clk;
    AiVisionSettings one = threeColoursAndObjects();
    one.colourCount = 1;
    ProsAiVision cam{3, one, clk};
    CHECK(cam.configured());
    // ... and EACH field of the four, off on its own, is refused (review X3-X5: the three tests before
    // this one changed only blue, or two fields at once, so dropping any other comparison survived).
    // The written descriptor is {1, 200, 30, 40}.
    const pros::aivision_color_s_t oneOff[] = {
        {1, 201, 30, 40, 12.0F, 0.4F},  // red
        {1, 200, 31, 40, 12.0F, 0.4F},  // green
        {1, 200, 30, 41, 12.0F, 0.4F},  // blue
        {2, 200, 30, 40, 12.0F, 0.4F},  // the id: the camera answers for ANOTHER slot
    };
    for (const pros::aivision_color_s_t& stored : oneOff) {
        CAPTURE(static_cast<int>(stored.id));
        CAPTURE(static_cast<int>(stored.red));
        CAPTURE(static_cast<int>(stored.green));
        CAPTURE(static_cast<int>(stored.blue));
        shim::resetAiVisions();
        auto& s2 = shim::aiVisionState(3);
        s2.colourWriteIgnored = true;
        s2.colours[1] = stored;
        ProsAiVision cam2{3, one, clk};
        CHECK_FALSE(cam2.configured());
        CHECK(std::string{cam2.setupProblem()} == "a colour read back different from what was written");
        CHECK(cam2.read().status == FrameStatus::NotConfigured);
    }
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.18 (second half): get_color's FAILURE RECORD -- 127s and +Inf, the binary's "
          "failed claim -- refuses the set-up, even for a descriptor whose RGB is 127,127,127") {
    // The writes land; only the read-back's port claim fails (a cable lost mid-set-up). The binary answers
    // id, red, green, blue = 127 and both floats +Inf — EXACTLY that record, the binary's failure path (a
    // descriptor whose RGB is 127,127,127 still has its own id and finite floats).
    for (const std::uint8_t rgb : {std::uint8_t{40}, std::uint8_t{127}}) {
        CAPTURE(static_cast<int>(rgb));
        shim::resetAiVisions();
        auto& s = shim::aiVisionState(3);
        s.colourReadFails = true;
        const pros::aivision_color_s_t raw = pros::c::aivision_get_color(3, 1);
        CHECK(raw.id == 127);
        CHECK(raw.red == 127);
        CHECK(std::isinf(raw.hue_range));
        CHECK(std::isinf(raw.saturation_range));
        FakeClock clk;
        AiVisionSettings one = threeColoursAndObjects();
        one.colourCount = 1;
        one.colours[0].red = rgb;
        one.colours[0].green = rgb;
        one.colours[0].blue = rgb;
        ProsAiVision cam{3, one, clk};
        CHECK(s.colourWrites == 1);  // the write was made
        CHECK(s.colours[1].red == rgb);  // and landed
        CHECK_FALSE(cam.configured());
        CHECK(std::string{cam.setupProblem()} == "a colour read-back failed (the camera did not answer)");
        CHECK(cam.read().status == FrameStatus::NotConfigured);
        CHECK(s.modeWrites == 0);  // the set-up stopped at the colour: no mode was written
        // The claim works again: the retry, no sooner than 0.5 s, configures it.
        s.colourReadFails = false;
        clk.advance(Time{0.5});
        CHECK(cam.read().usable());
        CHECK(cam.configured());
        noForbiddenCalls(3);
    }
}

TEST_CASE("ProsAiVision P1.9: the 1 Hz read-back finds the modes changed under the program and re-writes "
          "them") {
    // BUG CAUGHT (M7): a camera another program (or a re-plug) set back to TAGS + OBJECTS would stay so
    // for the rest of the run.
    shim::resetAiVisions();
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    s.modes = 0x05;  // someone turned tags back on
    clk.advance(Time{0.5});
    CHECK(cam.read().usable());
    CHECK(s.modes == 0x05);  // not yet: inside the second
    CHECK(cam.settingsLost() == 0);
    clk.advance(Time{0.5});  // exactly 1 s after the set-up: due
    const VisionFrame f = cam.read();
    CHECK(f.usable());
    CHECK(s.modes == kColorsObjects);
    CHECK(cam.settingsLost() == 1);
    CHECK(cam.setups() == 2);
    CHECK(cam.faultedReads() == 0);  // a CHANGED setting is not a failed read
    // The check is re-armed: three reads inside the next second make NO mode or colour read-back (review
    // X21: a timer never re-armed would read the settings back on every read, past HA-203's budget).
    const int modeReads0 = s.modeReads;
    const int colourReads0 = s.colourReads;
    for (int k = 0; k < 3; ++k) {
        clk.advance(Time{0.25});
        CHECK(cam.read().usable());
    }
    CHECK(s.modeReads == modeReads0);
    CHECK(s.colourReads == colourReads0);
    clk.advance(Time{0.25});  // exactly 1 s after the re-set-up: due again
    CHECK(cam.read().usable());
    CHECK(s.modeReads == modeReads0 + 1);
    CHECK(s.colourReads == colourReads0 + 3);  // every descriptor read back
    CHECK(cam.settingsLost() == 1);  // and it held
    // A check that HELD (no re-set-up to re-arm the timer) re-arms it too: nothing more inside the second.
    for (int k = 0; k < 3; ++k) {
        clk.advance(Time{0.25});
        CHECK(cam.read().usable());
    }
    CHECK(s.modeReads == modeReads0 + 1);
    CHECK(s.colourReads == colourReads0 + 3);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: the 1 Hz check reads EVERY descriptor back -- a colour changed under the program "
          "(the modes untouched) is re-written") {
    shim::resetAiVisions();
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    s.colours[3] = pros::aivision_color_s_t{3, 10, 10, 10, 5.0F, 0.2F};  // another writer, slot 3 only
    clk.advance(Time{1.0});
    CHECK(cam.read().usable());
    CHECK(cam.settingsLost() == 1);
    CHECK(cam.setups() == 2);
    CHECK(s.colours[3].red == 230);  // written again
    CHECK(cam.faultedReads() == 0);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: the 1 Hz check compares EACH field on its own -- only red, only green, only blue, "
          "only the id off by one is a changed setting; only the floats off is not") {
    // BUG CAUGHT (verification round): the check's compare was pinned only by a slot whose red, green AND
    // blue all changed, and by the swap's stray (green AND blue) — a check blind to any ONE field passed
    // every test, and a slot another program re-wrote in that field alone kept its stale colour (its
    // blobs then taken for the team's Toggle colours). The set-up's own compare is pinned the same way
    // (the "NOT the two floats" case above). The written descriptor 1 is {1, 200, 30, 40, 12, 0.4}.
    struct OneOff {
        const char* what;
        pros::aivision_color_s_t stored;
        bool changed;
    };
    const OneOff cases[] = {
        {"red", {1, 201, 30, 40, 12.0F, 0.4F}, true},
        {"green", {1, 200, 31, 40, 12.0F, 0.4F}, true},
        {"blue", {1, 200, 30, 41, 12.0F, 0.4F}, true},
        {"the id: the camera answers for ANOTHER slot", {2, 200, 30, 40, 12.0F, 0.4F}, true},
        {"only the two floats (never compared: HA-196)", {1, 200, 30, 40, 11.9F, 0.39F}, false},
    };
    for (const OneOff& c : cases) {
        CAPTURE(c.what);
        shim::resetAiVisions();
        FakeClock clk;
        ProsAiVision cam{3, threeColoursAndObjects(), clk};
        REQUIRE(cam.configured());
        auto& s = shim::aiVisionState(3);
        s.colours[1] = c.stored;
        clk.advance(Time{1.0});
        const VisionFrame f = cam.read();
        CHECK(f.usable());
        CHECK(cam.faultedReads() == 0);  // an answer, compared: never a failed read
        CHECK(cam.settingsLost() == (c.changed ? 1 : 0));
        CHECK(cam.setups() == (c.changed ? 2 : 1));
        if (c.changed) {  // written again, every field
            CHECK(s.colours[1].id == 1);
            CHECK(s.colours[1].red == 200);
            CHECK(s.colours[1].green == 30);
            CHECK(s.colours[1].blue == 40);
        }
        noForbiddenCalls(3);
    }
}

TEST_CASE("ProsAiVision: ONLY get_color's EXACT failure record is a failed read -- a wiped slot, or the "
          "record with any one field off, is an ANSWER: a changed setting, re-written") {
    // BUG CAUGHT (verification round): any read-back id outside 1..7 was taken for "the camera did not
    // answer" — wider than the binary, whose only failure record is the claim-fail path's 127s and +Inf
    // (its success path copies out whatever the SDK wrote, unread). A camera that came back WITHOUT its
    // descriptors (the shim's empty slot reads all zero, id 0) — HA-196's own settle case for
    // settingsLost — counted as a faulted read with settingsLost 0, and the boot words said the camera
    // did not answer when it had.
    const float inf = std::numeric_limits<float>::infinity();
    const pros::aivision_color_s_t record{127, 127, 127, 127, inf, inf};

    SUBCASE("every slot wiped, the modes kept: a lost setting, never a faulted read") {
        shim::resetAiVisions();
        FakeClock clk;
        ProsAiVision cam{3, threeColoursAndObjects(), clk};
        REQUIRE(cam.configured());
        auto& s = shim::aiVisionState(3);
        s.colours = {};
        clk.advance(Time{1.0});
        const VisionFrame f = cam.read();
        CHECK(f.usable());  // re-written in this very read
        CHECK(f.status == FrameStatus::Ok);
        CHECK(cam.settingsLost() == 1);
        CHECK(cam.faultedReads() == 0);
        CHECK(cam.setups() == 2);
        CHECK(s.colours[1].red == 200);
        CHECK(s.colours[2].blue == 200);
        CHECK(s.colours[3].red == 230);
        noForbiddenCalls(3);
    }
    SUBCASE("a write that 'succeeds' into an EMPTY slot and stores nothing: 'different', never 'did not "
            "answer'") {
        shim::resetAiVisions();
        auto& s = shim::aiVisionState(3);
        s.colours = {};
        s.colourWriteIgnored = true;
        FakeClock clk;
        ProsAiVision cam{3, threeColoursAndObjects(), clk};
        CHECK_FALSE(cam.configured());
        CHECK(std::string{cam.setupProblem()} == "a colour read back different from what was written");
    }
    SUBCASE("the EXACT record met by the 1 Hz check: a failed read; one field off: an answer") {
        // The written descriptor 2 is {2, 30, 60, 200}. Each variant differs from the record in ONE field.
        struct Variant {
            const char* what;
            pros::aivision_color_s_t stored;
            bool failed;
        };
        const Variant variants[] = {
            {"the record itself", record, true},
            {"id 126", {126, 127, 127, 127, inf, inf}, false},
            {"red 126", {127, 126, 127, 127, inf, inf}, false},
            {"green 126", {127, 127, 126, 127, inf, inf}, false},
            {"blue 126", {127, 127, 127, 126, inf, inf}, false},
            {"a finite hue range", {127, 127, 127, 127, 12.0F, inf}, false},
            {"a finite saturation range", {127, 127, 127, 127, inf, 0.4F}, false},
            {"hue -Inf", {127, 127, 127, 127, -inf, inf}, false},
            {"saturation -Inf", {127, 127, 127, 127, inf, -inf}, false},
        };
        for (const Variant& v : variants) {
            CAPTURE(v.what);
            shim::resetAiVisions();
            FakeClock clk;
            ProsAiVision cam{3, threeColoursAndObjects(), clk};
            REQUIRE(cam.configured());
            auto& s = shim::aiVisionState(3);
            s.colours[2] = v.stored;
            clk.advance(Time{1.0});
            const VisionFrame f = cam.read();
            if (v.failed) {
                CHECK(f.status == FrameStatus::ReadFault);
                CHECK(f.count == 0);
                CHECK(cam.faultedReads() == 1);
                CHECK(cam.settingsLost() == 0);
                CHECK(std::string{cam.setupProblem()}
                      == "the camera did not answer the 1 Hz check -- the set-up is re-written");
            } else {
                CHECK(f.usable());
                CHECK(cam.faultedReads() == 0);
                CHECK(cam.settingsLost() == 1);
                CHECK(cam.setups() == 2);
                CHECK(s.colours[2].blue == 200);  // written again
            }
            noForbiddenCalls(3);
        }
    }
    SUBCASE("the set-up's read-back: the record is 'did not answer'; one field off is 'different'") {
        for (const bool exact : {true, false}) {
            CAPTURE(exact);
            shim::resetAiVisions();
            auto& s = shim::aiVisionState(3);
            s.colourWriteIgnored = true;
            s.colours[1] = record;
            if (!exact) {
                s.colours[1].green = 126;
            }
            FakeClock clk;
            ProsAiVision cam{3, threeColoursAndObjects(), clk};
            CHECK_FALSE(cam.configured());
            CHECK(std::string{cam.setupProblem()}
                  == (exact ? "a colour read-back failed (the camera did not answer)"
                            : "a colour read back different from what was written"));
        }
    }
}

TEST_CASE("ProsAiVision: descriptor 7 -- the last id the camera has -- configures, holds at the 1 Hz check "
          "and its blobs pass") {
    // The verification round's CRF7: no test configured id 7, so treating the top id as a failed read
    // (a range check one off) refused a team that uses slot 7 — "a colour read-back failed" at every boot.
    shim::resetAiVisions();
    FakeClock clk;
    AiVisionSettings s7{};
    s7.colours[0] = {7, 120, 40, 220, 12.0F, 0.4F, "slot seven", "host test"};
    s7.colours[1] = {1, 200, 30, 40, 12.0F, 0.4F, "toggle red", "host test"};
    s7.colourCount = 2;
    ProsAiVision cam{3, s7, clk};
    CHECK(cam.configured());
    CHECK(std::string{cam.setupProblem()}.empty());
    auto& s = shim::aiVisionState(3);
    CHECK(s.colours[7].id == 7);
    CHECK(s.colours[7].blue == 220);
    s.objects[0] = shim::colourObject(7, 40, 100, 20, 12);
    s.objectCount = 1;
    for (int k = 0; k < 3; ++k) {  // 0 s, then the 1 Hz check at exactly 1 s and 2 s
        const VisionFrame f = cam.read();
        CHECK(f.status == FrameStatus::Ok);
        REQUIRE(f.count == 1);
        CHECK(f.detections[0].id == 7);
        CHECK(f.coloursWithheld == 0);
        clk.advance(Time{1.0});
    }
    CHECK(cam.settingsLost() == 0);
    CHECK(cam.faultedReads() == 0);
    CHECK(cam.setups() == 1);
    CHECK(s.colourReads == 2 + 2 * 2);  // the set-up's two, then two per check: each one HELD
    noForbiddenCalls(3);
}

// ═══ The read ══════════════════════════════════════════════════════════════════════

TEST_CASE("ProsAiVision P1.2: a read copies out colour and object boxes BY VALUE, stamped when it "
          "COMPLETED") {
    shim::resetAiVisions();
    overrideModel(3);
    FakeClock clk{Time{5.0}};
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::colourObject(1, 40, 100, 200, 12, 5);
    s.objects[1] = shim::elementObject(3, 150, 120, 10, 40, 87);
    s.objectCount = 2;
    clk.advance(Time{0.25});
    const VisionFrame f = cam.read();
    REQUIRE(f.usable());
    CHECK(f.count == 2);
    CHECK(f.reportedCount == 2);
    CHECK_FALSE(f.truncated);
    CHECK(f.skipped == 0);
    CHECK(f.objectsWithheld == 0);
    CHECK(f.model == ModelCheck::Matches);
    CHECK(f.readAt.value() == doctest::Approx(5.25));
    CHECK(f.readSeq == 1U);
    CHECK(f.detections[0].kind == DetectionKind::Colour);
    CHECK(f.detections[0].id == 1);
    CHECK(f.detections[0].left == 40);
    CHECK(f.detections[0].top == 100);
    CHECK(f.detections[0].width == 200);
    CHECK(f.detections[0].height == 12);
    CHECK(f.detections[0].angleTenths == 5);
    CHECK(f.detections[0].score == -1);
    CHECK(f.detections[1].kind == DetectionKind::Object);
    CHECK(f.detections[1].id == 3);
    CHECK(f.detections[1].left == 150);
    CHECK(f.detections[1].top == 120);   // review X8: the TOP from the y offset, never the x
    CHECK(f.detections[1].width == 10);  // review X9: the WIDTH from the width, never the height
    CHECK(f.detections[1].height == 40);
    CHECK(f.detections[1].score == 87);
    CHECK(f.detections[1].angleTenths == 0);  // an Object has no angle
    CHECK(cam.read().readSeq == 2U);  // the adapter's own poll count
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: a frame is stamped when its read COMPLETED -- the clock's LAST reading inside "
          "read(), after every device call -- never its start") {
    // BUG CAUGHT (review X7): with a clock that never moves inside read(), a stamp taken at the START
    // passed every test. Here the clock moves 1 ms at every reading, so the two differ.
    shim::resetAiVisions();
    overrideModel(3);
    TickingClock tick;
    ProsAiVision cam{3, threeColoursAndObjects(), tick};
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::colourObject(1, 40, 100, 200, 12, 5);
    s.objectCount = 1;
    for (int k = 0; k < 3; ++k) {
        const double before = tick.last();
        const int readings0 = tick.readings();
        const VisionFrame f = cam.read();
        REQUIRE(f.usable());
        CHECK(tick.readings() - readings0 >= 2);  // a start AND an end reading
        CHECK(f.readAt.value() == tick.last());   // the LAST reading: nothing read the clock after it
        CHECK(f.readAt.value() > before + 0.0015);  // and not the first reading of this read
    }
    // The fault frames too: a ReadFault and a NotConfigured frame are stamped at their end.
    s.disconnected = true;
    const VisionFrame bad = cam.read();
    CHECK(bad.status == FrameStatus::ReadFault);
    CHECK(bad.readAt.value() == tick.last());
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: a colour from a slot this program did NOT write, and any colour CODE, are "
          "WITHHELD and counted -- the frame stays evidence") {
    // BUG CAUGHT (review): every Colour and Code record passed as an Ok frame's detection — a descriptor
    // VEXcode's Utility (or the shim's own stray in slot 1) left on the camera fires blobs whose id means
    // nothing to this program, and VIS1 writes no colour codes.
    shim::resetAiVisions();  // the stray descriptor sits in slot 1
    FakeClock clk;
    AiVisionSettings twoThree = threeColoursAndObjects();
    twoThree.colours[0] = twoThree.colours[1];  // ids 2 and 3 only
    twoThree.colours[1] = twoThree.colours[2];
    twoThree.colourCount = 2;
    ProsAiVision cam{3, twoThree, clk};
    REQUIRE(cam.configured());
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::colourObject(1, 40, 100, 200, 12);  // the stray slot: nobody here wrote id 1
    s.objects[1] = shim::colourObject(2, 50, 110, 20, 22);   // written
    s.objects[2] = shim::codeObject(2, 60, 120, 30, 32);     // a colour CODE 2 (descriptor 2 IS written): no codes
    s.objects[3] = shim::colourObject(3, 70, 130, 40, 42);   // written
    s.objects[4] = shim::colourObject(7, 80, 140, 50, 52);   // never written
    s.objectCount = 5;
    const VisionFrame f = cam.read();
    CHECK(f.usable());  // withheld is not torn: the frame is still evidence
    CHECK(f.status == FrameStatus::Ok);
    CHECK(f.reportedCount == 5);
    CHECK(f.count == 2);
    CHECK(f.coloursWithheld == 3);
    CHECK(f.skipped == 0);
    CHECK(f.objectsWithheld == 0);
    CHECK(f.detections[0].kind == DetectionKind::Colour);
    CHECK(f.detections[0].id == 2);
    CHECK(f.detections[0].left == 50);
    CHECK(f.detections[1].id == 3);
    CHECK(f.detections[1].left == 70);
    CHECK(cam.faultedReads() == 0);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: an Object's raw score is carried WHOLE -- 65535 is never the -1 'not an Object'") {
    // BUG CAUGHT (review): the uint16 score narrowed to int16 read 65535 as -1 — the documented value for
    // "not an Object" — and 40000 as -25536. Its scale is HA-198's unknown, so all of it is kept.
    shim::resetAiVisions();
    overrideModel(3);
    FakeClock clk;
    ProsAiVision cam{3, objectsOnly(), clk};
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::elementObject(1, 10, 20, 30, 40, 65535);
    s.objects[1] = shim::elementObject(2, 10, 20, 30, 40, 40000);
    s.objects[2] = shim::elementObject(3, 10, 20, 30, 40, 0);
    s.objects[3] = shim::elementObject(4, 10, 20, 30, 40, 32767);
    s.objects[4] = shim::elementObject(0, 10, 20, 30, 40, 32768);
    s.objectCount = 5;
    const VisionFrame f = cam.read();
    REQUIRE(f.usable());
    REQUIRE(f.count == 5);
    CHECK(f.detections[0].score == 65535);
    CHECK(f.detections[1].score == 40000);
    CHECK(f.detections[2].score == 0);
    CHECK(f.detections[3].score == 32767);
    CHECK(f.detections[4].score == 32768);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.3: a PROS_ERR count is a fault with NO detections -- never the last frame, never "
          "a number -- and the set-up is re-written") {
    // BUG CAUGHT (M2; the run's own "PROS_ERR treated as a number"): PROS_ERR is INT32_MAX — a count
    // check that reads it as a count calls a dead camera "2,147,483,647 detections, possibly full".
    shim::resetAiVisions();
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    shulib::hal::IReadFaults& faults = cam;
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::colourObject(1, 40, 100, 200, 12);
    s.objectCount = 1;
    REQUIRE(cam.read().count == 1);
    s.disconnected = true;
    const int objectReads0 = s.objectReads;
    const VisionFrame bad = cam.read();
    CHECK(bad.status == FrameStatus::ReadFault);
    CHECK(bad.count == 0);
    CHECK(bad.reportedCount == 0);
    CHECK_FALSE(bad.truncated);
    CHECK(bad.skipped == 0);
    CHECK(s.objectReads == objectReads0);  // no object was asked for
    CHECK(faults.faultedReads() == 1);
    CHECK_FALSE(cam.configured());
    CHECK(std::string{cam.status().setupProblem} == "a read failed (PROS_ERR) -- the set-up is re-written");
    // The camera comes back WITHOUT its settings (a re-plug): the adapter writes them again.
    s.disconnected = false;
    s.modes = 0x05;
    s.colours = {};
    clk.advance(Time{0.6});
    const VisionFrame back = cam.read();
    CHECK(back.usable());
    CHECK(back.count == 1);
    CHECK(s.modes == kColorsObjects);
    CHECK(s.colours[1].red == 200);
    CHECK(cam.setups() == 2);
    CHECK(faults.faultedReads() == 1);  // monotonic: the good read does not move it
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: a NEGATIVE count is a fault too") {
    shim::resetAiVisions();
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    shim::aiVisionState(3).objectCount = -3;
    const VisionFrame f = cam.read();
    CHECK(f.status == FrameStatus::ReadFault);
    CHECK(f.count == 0);
    CHECK(cam.faultedReads() == 1);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.4 / P1.18: a torn read -- the binary's 127/127 record -- is a PartialFault, "
          "skipped, counted, NOT evidence") {
    // BUG CAUGHT (M4): a read with a skipped object still Ok would let a consumer take "no contradicting
    // band" from a read that lost one.
    shim::resetAiVisions();
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::colourObject(1, 40, 100, 200, 12);
    s.objects[1] = shim::colourObject(2, 40, 140, 200, 12);
    s.objectCount = 2;
    s.tornFrom = 1;
    const pros::aivision_object_s_t raw = pros::c::aivision_get_object(3, 1);
    CHECK(raw.type == 127);
    CHECK(raw.id == 127);
    const VisionFrame f = cam.read();
    CHECK(f.status == FrameStatus::PartialFault);
    CHECK_FALSE(f.usable());
    CHECK(f.skipped == 1);
    CHECK(f.count == 1);  // the good one is still copied, for a bench page — but the frame is not evidence
    CHECK(cam.faultedReads() == 1);
    CHECK(cam.configured());  // a torn read is not a lost set-up
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.6: a TAG that arrives anyway is skipped, and the read is not evidence") {
    shim::resetAiVisions();
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::tagObject(7);
    s.objects[1] = shim::colourObject(1, 40, 100, 200, 12);
    s.objectCount = 2;
    const VisionFrame f = cam.read();
    CHECK(f.status == FrameStatus::PartialFault);
    CHECK(f.skipped == 1);
    CHECK(f.count == 1);
    CHECK(f.detections[0].kind == DetectionKind::Colour);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.5 / P1.20: POSSIBLY FULL at the cap -- 23 is complete; 24, 25 and 30 are "
          "truncated with EXACTLY 24 read") {
    // BUG CAUGHT (M3, M14): reading index 24 and past (the camera holds at most 24 — the vendored
    // header's "given by VEX"), and trusting a read of exactly 24 as complete (VIS1's re-check S4: a
    // camera holding more may still REPORT 24, and the band that contradicts a colour could be the 25th).
    for (const int n : {0, 1, 23, 24, 25, 30}) {
        CAPTURE(n);
        shim::resetAiVisions();
        FakeClock clk;
        ProsAiVision cam{3, threeColoursAndObjects(), clk};
        auto& s = shim::aiVisionState(3);
        for (int i = 0; i < 32; ++i) {
            s.objects[static_cast<std::size_t>(i)] = shim::colourObject(1, i, 10, 5, 5);
        }
        s.objectCount = n;
        const int before = s.objectReads;
        const VisionFrame f = cam.read();
        CHECK(f.usable());
        CHECK(f.reportedCount == n);
        CHECK(f.truncated == (n >= 24));
        CHECK(f.count == (n < 24 ? n : 24));
        CHECK(s.objectReads - before == (n < 24 ? n : 24));
        if (n >= 24) {
            CHECK(f.detections[23].left == 23);  // the 24th, by value
        }
        noForbiddenCalls(3);
    }
}

// ═══ The model, by name ════════════════════════════════════════════════════════════

TEST_CASE("ProsAiVision P1.11: the camera carries ANOTHER model (VEX's Classroom table) -- every Object "
          "WITHHELD, the colours kept") {
    // BUG CAUGHT (M8): without the check, class 3 — the Classroom model's blueRing — reads as the
    // Override model's yellowPin, and a route chases a ring.
    shim::resetAiVisions();  // the shim's default model is Classroom: class 3 = blueRing
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::colourObject(1, 40, 100, 200, 12);
    s.objects[1] = shim::elementObject(3, 150, 120, 10, 40, 87);
    s.objectCount = 2;
    const VisionFrame f = cam.read();
    CHECK(f.usable());  // the colours are still evidence
    CHECK(f.model == ModelCheck::Different);
    CHECK(f.count == 1);
    CHECK(f.detections[0].kind == DetectionKind::Colour);
    CHECK(f.objectsWithheld == 1);
    CHECK(std::string{cam.modelProblem()} == "class 0 is 'blueBall', the robot file expects 'redBluePin'");
    CHECK(cam.status().model == ModelCheck::Different);
    CHECK(std::string{cam.status().modelProblem} == std::string{cam.modelProblem()});
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.14: one wrong name of five -- class 3 -- is caught (not only class 0)") {
    // BUG CAUGHT (M9): a check that compares only class 0 would pass a model whose class 3 is not a
    // yellow Pin.
    shim::resetAiVisions();
    overrideModel(3);
    shim::aiVisionState(3).classNames[3] = "yellowPinX";
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    CHECK(cam.model() == ModelCheck::Different);
    CHECK(std::string{cam.modelProblem()} == "class 3 is 'yellowPinX', the robot file expects 'yellowPin'");
    // The LAST name too.
    shim::resetAiVisions();
    overrideModel(3);
    shim::aiVisionState(3).classNames[4] = "Cup";
    ProsAiVision cam2{3, threeColoursAndObjects(), clk};
    CHECK(cam2.model() == ModelCheck::Different);
    CHECK(contains(cam2.modelProblem(), "class 4 is 'Cup'"));
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.12: names unreadable at boot (just plugged in) are PENDING, never a match -- "
          "re-asked once a second, then Matches") {
    // BUG CAUGHT (M10): an unreadable table counted as a match passes every Object from a camera whose
    // model nobody has seen.
    shim::resetAiVisions();
    overrideModel(3);
    FakeClock clk;
    auto& s = shim::aiVisionState(3);
    s.classNamesFail = true;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    CHECK(cam.configured());
    CHECK(cam.model() == ModelCheck::Unreadable);
    CHECK(std::string{cam.modelProblem()} == "class 0's name could not be read (PROS_ERR)");
    s.objects[0] = shim::elementObject(3, 150, 120, 10, 40, 87);
    s.objectCount = 1;
    CHECK(cam.read().objectsWithheld == 1);
    s.classNamesFail = false;
    const int asks0 = s.classNameReads;
    clk.advance(Time{0.5});
    CHECK(cam.read().objectsWithheld == 1);  // not re-asked inside the second
    CHECK(s.classNameReads == asks0);
    clk.advance(Time{0.5});
    const VisionFrame f = cam.read();
    CHECK(f.model == ModelCheck::Matches);
    CHECK(f.count == 1);
    CHECK(f.objectsWithheld == 0);
    CHECK(f.detections[0].kind == DetectionKind::Object);
    CHECK(std::string{cam.modelProblem()}.empty());
    // Matched: re-asked at EVERY 1 Hz check all the same — a match is never more than a second old (a
    // camera swapped while nothing read it may carry another model) — and never inside the second.
    const int asks1 = s.classNameReads;
    clk.advance(Time{0.5});
    (void)cam.read();
    CHECK(s.classNameReads == asks1);
    clk.advance(Time{0.5});
    CHECK(cam.read().model == ModelCheck::Matches);
    CHECK(s.classNameReads == asks1 + 5);  // the whole expected table, once
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: names that read WRONG at boot (Different, not PROS_ERR -- the binary's realistic "
          "boot path) are re-asked at the 1 Hz check, and Matches then") {
    // BUG CAUGHT (review): only the Unreadable path was tested, so a re-ask limited to Unreadable passed the
    // whole suite. The vendored binary answers get_class_name with 1 whatever the SDK did (`movs r0, #1`),
    // so a camera still booting reads '' or another table — Different — never PROS_ERR; without the
    // re-ask, every Object is withheld for the whole run.
    shim::resetAiVisions();  // VEX's Classroom table at first
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    REQUIRE(cam.model() == ModelCheck::Different);
    s.objects[0] = shim::elementObject(3, 150, 120, 10, 40, 87);
    s.objectCount = 1;
    CHECK(cam.read().objectsWithheld == 1);
    overrideModel(3);  // the names now read right (the camera finished booting)
    const int asks0 = s.classNameReads;
    clk.advance(Time{0.5});
    const VisionFrame early = cam.read();
    CHECK(early.objectsWithheld == 1);  // not re-asked inside the second
    CHECK(early.model == ModelCheck::Different);
    CHECK(s.classNameReads == asks0);
    clk.advance(Time{0.5});  // EXACTLY 1 s after the set-up (binary-exact): due
    const VisionFrame f = cam.read();
    CHECK(s.classNameReads == asks0 + 5);
    CHECK(f.model == ModelCheck::Matches);
    CHECK(f.objectsWithheld == 0);
    REQUIRE(f.count == 1);
    CHECK(f.detections[0].kind == DetectionKind::Object);
    CHECK(std::string{cam.modelProblem()}.empty());
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: a camera SWAPPED while nothing read it -- the modes already right -- is caught at "
          "the next read's 1 Hz check, BEFORE any object passes") {
    // BUG CAUGHT (review): the 1 Hz check compared only the modes, and the model was re-asked only while
    // it did not match — so a camera swapped (or re-seated) between reads, coming back with the modes
    // already COLORS + OBJECTS, kept the old Matches and its program's descriptors were never re-written.
    SUBCASE("another model, the same descriptors") {
        shim::resetAiVisions();
        overrideModel(3);
        FakeClock clk;
        ProsAiVision cam{3, threeColoursAndObjects(), clk};
        auto& s = shim::aiVisionState(3);
        s.objects[0] = shim::elementObject(3, 150, 120, 10, 40, 87);
        s.objectCount = 1;
        REQUIRE(cam.read().count == 1);
        s.classNames = {"blueBall", "greenBall", "redBall", "blueRing", "greenRing"};
        s.classCount = 5;
        clk.advance(Time{5.0});  // nothing read it for five seconds
        const VisionFrame f = cam.read();
        CHECK(f.usable());
        CHECK(f.model == ModelCheck::Different);
        CHECK(f.count == 0);  // class 3 is blueRing now: withheld in THIS read
        CHECK(f.objectsWithheld == 1);
        CHECK(cam.settingsLost() == 0);  // the settings held
        noForbiddenCalls(3);
    }
    SUBCASE("other descriptors (VEXcode's), the same modes and model") {
        shim::resetAiVisions();
        overrideModel(3);
        FakeClock clk;
        ProsAiVision cam{3, threeColoursAndObjects(), clk};
        auto& s = shim::aiVisionState(3);
        s.colours = {};
        s.colours[1] = pros::aivision_color_s_t{1, 200, 200, 200, 30.0F, 0.9F};  // the stray, back
        s.objects[0] = shim::colourObject(1, 40, 100, 200, 12);
        s.objectCount = 1;
        clk.advance(Time{5.0});
        const VisionFrame f = cam.read();
        CHECK(f.usable());
        CHECK(cam.settingsLost() == 1);
        CHECK(cam.setups() == 2);
        CHECK(s.colours[1].green == 30);  // the program's red, written again BEFORE the read
        CHECK(s.colours[2].blue == 200);
        CHECK(f.model == ModelCheck::Matches);  // the re-set-up re-asked it
        noForbiddenCalls(3);
    }
}

TEST_CASE("ProsAiVision: an unplug first met by the 1 Hz check is a FAILED READ -- faultedReads moves, "
          "settingsLost does not -- exactly as one met by the count") {
    // BUG CAUGHT (review): a PROS_ERR on the check's mode read was taken for "the modes changed": no
    // faulted read (so IReadFaults never moved for that unplug), and settingsLost — the bench's
    // re-plug measure (HA-196) — counted a cable fault as lost settings. Which call meets an unplug first
    // is chance; the counts must not depend on it.
    for (const bool atCheck : {true, false}) {
        CAPTURE(atCheck);
        shim::resetAiVisions();
        FakeClock clk;
        ProsAiVision cam{3, threeColoursAndObjects(), clk};
        auto& s = shim::aiVisionState(3);
        s.objects[0] = shim::colourObject(1, 40, 100, 200, 12);
        s.objectCount = 1;
        clk.advance(Time{atCheck ? 1.0 : 0.9});  // ON the check's tick, or a read before it
        s.disconnected = true;
        const VisionFrame bad = cam.read();
        CHECK(bad.status == FrameStatus::ReadFault);
        CHECK(bad.count == 0);
        CHECK(cam.faultedReads() == 1);
        CHECK(cam.settingsLost() == 0);
        CHECK_FALSE(cam.configured());
        if (atCheck) {
            CHECK(std::string{cam.setupProblem()}
                  == "the camera did not answer the 1 Hz check -- the set-up is re-written");
            CHECK(s.countReads == 0);  // the check met it first; the count was never asked
        }
        // Still unplugged: the re-set-up fails (NotConfigured); the one unplug is ONE faulted read.
        for (int k = 0; k < 6; ++k) {
            clk.advance(Time{0.1});
            CHECK(cam.read().status == FrameStatus::NotConfigured);
        }
        CHECK(cam.faultedReads() == 1);
        CHECK(cam.settingsLost() == 0);
        // Back: written again, evidence again.
        s.disconnected = false;
        clk.advance(Time{0.5});
        CHECK(cam.read().usable());
        noForbiddenCalls(3);
    }
    // A colour read-back the check could not make (the binary's failure record) is a failed read too.
    shim::resetAiVisions();
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    s.colourReadFails = true;
    clk.advance(Time{1.0});
    CHECK(cam.read().status == FrameStatus::ReadFault);
    CHECK(cam.faultedReads() == 1);
    CHECK(cam.settingsLost() == 0);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.15: a camera with NO model names nothing ('' everywhere) -- Different, every "
          "Object withheld") {
    shim::resetAiVisions();
    auto& s = shim::aiVisionState(3);
    s.classNames = {};
    s.classCount = 0;  // the SDK writes nothing; PROS still answers PROS_SUCCESS (the binary)
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    CHECK(cam.model() == ModelCheck::Different);
    CHECK(std::string{cam.modelProblem()} == "class 0 is '', the robot file expects 'redBluePin'");
    s.objects[0] = shim::elementObject(0, 150, 120, 10, 40, 87);
    s.objectCount = 1;
    const VisionFrame f = cam.read();
    CHECK(f.objectsWithheld == 1);
    CHECK(f.count == 0);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision M12's case: a camera that names only the first four classes -- the fifth's buffer "
          "must read '' (ZEROED), never the expected name") {
    // BUG CAUGHT (M12): a buffer not zeroed before each read keeps whatever was in it; the worst case is
    // the expected name itself, and the camera "matches" a table it never wrote.
    shim::resetAiVisions();
    overrideModel(3);
    shim::aiVisionState(3).classCount = 4;  // names ids 0..3; id 4 gets nothing written
    FakeClock clk;
    ProsAiVision cam{3, objectsOnly(), clk};
    CHECK(cam.model() == ModelCheck::Different);
    CHECK(std::string{cam.modelProblem()} == "class 4 is '', the robot file expects 'cup'");
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.16: the check never asks for a count (id -1) -- PROS 4.2.2 returns 1 there -- "
          "and reads exactly the expected ids") {
    shim::resetAiVisions();
    overrideModel(3);
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    CHECK(cam.model() == ModelCheck::Matches);
    CHECK(shim::aiVisionState(3).classCountAsks == 0);
    CHECK(shim::aiVisionState(3).classNameReads == 5);  // exactly the expected ids, once each
    // The binary's answer for -1 is PROS_SUCCESS whatever the table holds — what the shim models.
    std::uint8_t buf[64] = {};
    CHECK(pros::c::aivision_get_class_name(3, -1, buf) == 1);
    CHECK(buf[0] == 0);
    shim::aiVisionState(3).classCountAsks = 0;  // the test's own -1 ask, not the adapter's
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision P1.17: a model with MORE classes -- an Object past the expected table is withheld "
          "even while the first five match") {
    // BUG CAUGHT (M11): an id whose name was never read passed on as if it were checked.
    shim::resetAiVisions();
    overrideModel(3);
    auto& s = shim::aiVisionState(3);
    s.classNames[5] = "redRing";  // a superset model: ids 0..4 named as Override's, id 5 unknown
    s.classCount = 6;
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    REQUIRE(cam.model() == ModelCheck::Matches);
    s.objects[0] = shim::elementObject(5, 150, 120, 10, 40, 87);
    s.objects[1] = shim::elementObject(4, 100, 120, 10, 40, 87);  // the last id IN the table passes
    s.objects[2] = shim::elementObject(200, 60, 120, 10, 40, 87);
    s.objectCount = 3;
    const VisionFrame f = cam.read();
    CHECK(f.usable());
    CHECK(f.count == 1);
    CHECK(f.detections[0].id == 4);
    CHECK(f.objectsWithheld == 2);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: a camera RE-PLUGGED carrying another model -- after the re-set-up the check reads "
          "Different and every Object is withheld again") {
    // A match is never cached across a re-plug (landmine 6).
    shim::resetAiVisions();
    overrideModel(3);
    FakeClock clk;
    ProsAiVision cam{3, threeColoursAndObjects(), clk};
    auto& s = shim::aiVisionState(3);
    s.objects[0] = shim::elementObject(3, 150, 120, 10, 40, 87);
    s.objectCount = 1;
    REQUIRE(cam.read().count == 1);
    REQUIRE(cam.model() == ModelCheck::Matches);
    s.disconnected = true;
    CHECK(cam.read().status == FrameStatus::ReadFault);
    // Back, with VEX's Classroom table on it.
    s.disconnected = false;
    s.classNames = {"blueBall", "greenBall", "redBall", "blueRing", "greenRing"};
    s.classCount = 5;
    clk.advance(Time{0.6});
    const VisionFrame f = cam.read();
    CHECK(f.usable());
    CHECK(f.model == ModelCheck::Different);
    CHECK(f.count == 0);
    CHECK(f.objectsWithheld == 1);
    CHECK(contains(cam.modelProblem(), "class 0 is 'blueBall'"));
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision: a camera's name bytes reach the boot line as printable ASCII, cut at 20 "
          "characters") {
    shim::resetAiVisions();
    overrideModel(3);
    static const char kOdd[] = "red\xC3\xA9\x01Pin\x7F";  // a UTF-8 letter, a control byte, DEL
    shim::aiVisionState(3).classNames[0] = kOdd;
    FakeClock clk;
    ProsAiVision cam{3, objectsOnly(), clk};
    CHECK(cam.model() == ModelCheck::Different);
    CHECK(ascii(cam.modelProblem()));
    CHECK(contains(cam.modelProblem(), "class 0 is 'red???Pin?'"));  // DEL (0x7F) too (review X1)
    CHECK(std::strchr(cam.modelProblem(), '\x7F') == nullptr);
    noForbiddenCalls(3);
}

TEST_CASE("ProsAiVision R3.1: status() carries the boot line's states and touches NOTHING") {
    shim::resetAiVisions();  // Classroom model
    FakeClock clk;
    ProsAiVision cam{3, objectsOnly(), clk};
    const auto& s = shim::aiVisionState(3);
    const int calls0 = s.countReads + s.objectReads + s.modeReads + s.modeWrites + s.colourWrites
                       + s.colourReads + s.classNameReads;
    const shulib::hal::CameraStatus st = cam.status();
    const int calls1 = s.countReads + s.objectReads + s.modeReads + s.modeWrites + s.colourWrites
                       + s.colourReads + s.classNameReads;
    CHECK(calls1 == calls0);
    CHECK(st.configured);
    CHECK(st.model == ModelCheck::Different);
    CHECK(std::string{shulib::hal::cameraStatusWords(st)} == "set up, model DIFFERENT");
    shim::aiVisionState(4).classNamesFail = true;
    ProsAiVision cam2{4, objectsOnly(), clk};
    CHECK(std::string{shulib::hal::cameraStatusWords(cam2.status())} == "set up, model check pending");
    shim::aiVisionState(5).modeWriteIgnored = true;
    ProsAiVision cam3{5, objectsOnly(), clk};
    CHECK(std::string{shulib::hal::cameraStatusWords(cam3.status())} == "NOT set up");
    CHECK(contains(cam3.status().setupProblem, "read back"));
    AiVisionSettings colours = threeColoursAndObjects();
    colours.objects = false;
    overrideModel(6);
    ProsAiVision cam4{6, colours, clk};
    CHECK(std::string{shulib::hal::cameraStatusWords(cam4.status())} == "set up, objects off");
    overrideModel(7);
    ProsAiVision cam5{7, objectsOnly(), clk};
    CHECK(std::string{shulib::hal::cameraStatusWords(cam5.status())} == "set up, model OK");
    for (const int port : {3, 4, 5, 6, 7}) {
        noForbiddenCalls(port);
    }
}

// ═══ The provider ══════════════════════════════════════════════════════════════════

TEST_CASE("ProsDeviceProvider::aiVision: a port outside 1..21 is refused before anything is asked") {
    shim::resetAll();
    ProsDeviceProvider devices;
    for (const std::uint8_t p : {std::uint8_t{0}, std::uint8_t{22}, std::uint8_t{255}}) {
        CAPTURE(static_cast<int>(p));
        CHECK(devices.aiVision(p, objectsOnly()) == nullptr);
        CHECK(contains(devices.lastRefusal(), "is not a smart port (ports are 1..21)"));
    }
    CHECK(shim::registryState().queries == 0);
}

TEST_CASE("ProsDeviceProvider::aiVision: served ONLY on an AI Vision sensor -- every kind the brain can "
          "report is refused naming what was found; the OLDER Vision sensor by name") {
    // BUG CAUGHT (the run's own: E_DEVICE_AIVISION back to Vision): until VIS1-A1 the registry called
    // both cameras Vision, so no check could refuse the older sensor on the camera's port (D4).
    const pros::c::v5_device_e_t every[] = {
        pros::c::E_DEVICE_NONE,     pros::c::E_DEVICE_MOTOR, pros::c::E_DEVICE_ROTATION,
        pros::c::E_DEVICE_IMU,      pros::c::E_DEVICE_DISTANCE, pros::c::E_DEVICE_RADIO,
        pros::c::E_DEVICE_VISION,   pros::c::E_DEVICE_ADI,   pros::c::E_DEVICE_OPTICAL,
        pros::c::E_DEVICE_GPS,      pros::c::E_DEVICE_AIVISION, pros::c::E_DEVICE_SERIAL,
        pros::c::E_DEVICE_UNDEFINED};
    for (const pros::c::v5_device_e_t raw : every) {
        CAPTURE(static_cast<int>(raw));
        shim::resetAll();
        shim::setPluggedAtPort(3, raw);
        ProsDeviceProvider devices;
        shulib::hal::IVisionFrames* cam = devices.aiVision(3, objectsOnly());
        CHECK((cam != nullptr) == (raw == pros::c::E_DEVICE_AIVISION));
        if (cam == nullptr) {
            CHECK(contains(devices.lastRefusal(), "port 3: "));
            CHECK(contains(devices.lastRefusal(), "(expected an AI Vision sensor)"));
            CHECK(ascii(devices.lastRefusal()));
            CHECK(shim::aiVisionState(3).modeWrites == 0);  // never touched
            CHECK(devices.readFaults(3) == nullptr);
        }
    }
    shim::resetAll();
    shim::setPluggedAtPort(3, pros::c::E_DEVICE_VISION);
    shim::setPluggedAtPort(4, pros::c::E_DEVICE_MOTOR);
    ProsDeviceProvider devices;
    CHECK(devices.aiVision(3, objectsOnly()) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port 3: a vision sensor (expected an AI Vision sensor)");
    CHECK(devices.aiVision(4, objectsOnly()) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port 4: a motor (expected an AI Vision sensor)");
    CHECK(devices.aiVision(5, objectsOnly()) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port 5: nothing plugged in (expected an AI Vision sensor)");
}

TEST_CASE("ProsDeviceProvider::aiVision: a settings problem is refused with its sentence, BEFORE the "
          "registry or the device is asked") {
    shim::resetAll();
    shim::setPluggedAtPort(3, pros::c::E_DEVICE_AIVISION);
    ProsDeviceProvider devices;
    CHECK(devices.aiVision(3, AiVisionSettings{}) == nullptr);
    CHECK(std::string{devices.lastRefusal()}
          == "port 3: the camera's settings: nothing turned on: no colours and no objects");
    AiVisionSettings unset = threeColoursAndObjects();
    unset.colours[1] = shulib::hal::ColourDescriptor{};
    CHECK(devices.aiVision(3, unset) == nullptr);
    CHECK(std::string{devices.lastRefusal()}
          == "port 3: the camera's settings: UNSET -- measure it with VEXcode's AI Vision Utility and "
             "type it");
    CHECK(shim::registryState().queries == 0);
    CHECK(shim::aiVisionState(3).modeWrites == 0);
    CHECK(shim::aiVisionState(3).colourWrites == 0);
    // A refused request leaves the port free for a good one.
    CHECK(devices.aiVision(3, threeColoursAndObjects()) != nullptr);
}

TEST_CASE("ProsDeviceProvider::aiVision: MAKING it writes the camera; the SAME camera for equal settings "
          "(by content) with NOTHING written again; different settings refused -- one owner") {
    // BUG CAUGHT (the run's own: a second adapter for a second request): two adapters on one camera
    // would each re-write its settings and fight over its model check.
    shim::resetAll();
    shim::setPluggedAtPort(3, pros::c::E_DEVICE_AIVISION);
    overrideModel(3);
    ProsDeviceProvider devices;
    CHECK(devices.readFaults(3) == nullptr);  // nothing made yet, and asking makes nothing
    CHECK(shim::aiVisionState(3).modeWrites == 0);
    shulib::hal::IVisionFrames* first = devices.aiVision(3, threeColoursAndObjects());
    REQUIRE(first != nullptr);
    auto& s = shim::aiVisionState(3);
    CHECK(s.modes == kColorsObjects);  // written at construction
    CHECK(s.colourWrites == 3);
    CHECK(first->status().configured);
    CHECK(first->status().model == ModelCheck::Matches);
    const int writes = s.colourWrites + s.modeWrites + s.classNameReads + s.modeReads;
    const int queries = shim::registryState().queries;

    // Equal settings, from DIFFERENT storage (strings copied): the same object, nothing written.
    char redName[] = "toggle red";
    char provenance[] = "host test";
    AiVisionSettings again = threeColoursAndObjects();
    again.colours[0].name = redName;
    again.colours[0].provenance = provenance;
    CHECK(devices.aiVision(3, again) == first);
    CHECK(s.colourWrites + s.modeWrites + s.classNameReads + s.modeReads == writes);
    CHECK(shim::registryState().queries == queries);

    // Different settings: refused, naming the port; the first camera stands.
    AiVisionSettings other = threeColoursAndObjects();
    other.colours[2].blue = 41;
    CHECK(devices.aiVision(3, other) == nullptr);
    CHECK(std::string{devices.lastRefusal()}
          == "port 3: the camera was already set up with other settings -- one owner");
    CHECK(devices.aiVision(3, threeColoursAndObjects()) == first);

    // readFaults(3) IS the camera, and moves with its failed reads.
    const shulib::hal::IReadFaults* rf = devices.readFaults(3);
    REQUIRE(rf != nullptr);
    CHECK(rf == dynamic_cast<const shulib::hal::IReadFaults*>(first));
    CHECK(rf->faultedReads() == 0);
    s.disconnected = true;
    CHECK(first->read().status == FrameStatus::ReadFault);
    CHECK(rf->faultedReads() == 1);
    CHECK(devices.readFaults(4) == nullptr);
    noForbiddenCalls(3);
}
