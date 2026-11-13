// vision::CameraFeed — one read of the camera per period, the same frame to every consumer (chunk VIS1-A2;
// the VIS1 brief's §1.14, D10 and §2.2 "the feed").
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * The period LANDED ON: a tick exactly minReadPeriodSeconds after the last read BEGAN reads; the
//     double before it does not; the first tick always reads; never two reads in one tick, and two
//     consumers ticking in the same instant cost ONE read (D10) — mutation "reading every tick".
//   * The period runs from a read's START: a slow read does not push the next one later.
//   * latest() is the latest READ — a fault frame (no detections) included, never the last good one held
//     (a held frame reads as a fresh "nothing changed") — mutation "latest() keeps the last good frame".
//   * The stats count each status, `truncated` only on usable reads, the withheld counts summed.
//   * Each read is TIMED with the feed's own clock, before and after: a camera that takes 4 ms shows 4 ms;
//     the slowest read is the largest so far and a faster one never lowers it — mutation "the slowest
//     read never updated".
//   * The config: an UNSET, zero, negative, NaN or infinite period refused by its words.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_vision_frames.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/camera_feed.hpp"

namespace {

namespace hal = shulib::hal;
namespace vision = shulib::vision;
using shulib::units::Time;

constexpr double kPeriod = 0.03125;  // 1/32 s: binary-exact, so the period's edge can be landed on

hal::VisionFrame okFrame(std::uint8_t detections, double at = 0.0) {
    hal::VisionFrame f{};
    f.status = hal::FrameStatus::Ok;
    f.model = hal::ModelCheck::Matches;
    f.readAt = Time{at};
    for (std::uint8_t i = 0; i < detections; ++i) {
        hal::Detection d{};
        d.id = 1;
        d.width = 10;
        d.height = 5;
        f.detections[f.count++] = d;
    }
    f.reportedCount = detections;
    return f;
}

// A camera whose read takes time on the feed's own clock (the brief: "a fake camera that advances the fake
// clock during read()").
class SlowCamera final : public hal::IVisionFrames {
public:
    explicit SlowCamera(hal::fake::FakeClock& clock) : clock_{clock} {}
    void setTakes(double seconds) { takes_ = seconds; }
    [[nodiscard]] std::uint32_t reads() const { return reads_; }
    [[nodiscard]] hal::VisionFrame read() noexcept override {
        ++reads_;
        clock_.advance(Time{takes_});
        return okFrame(1, clock_.now().value());
    }
    [[nodiscard]] hal::CameraStatus status() const noexcept override { return {}; }

private:
    hal::fake::FakeClock& clock_;
    double takes_ = 0.0;
    std::uint32_t reads_ = 0;
};

}  // namespace

TEST_CASE("camera feed: the config -- an UNSET, zero, negative, NaN or infinite period refused by its words; the "
          "smallest positive accepted") {
    const std::string words = "minReadPeriodSeconds UNSET -- the team's read period, seconds (about 0.033 for 30 a second)";
    for (const double bad : {0.0, -0.033, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
        CAPTURE(bad);
        const char* p = vision::cameraFeedConfigProblem(vision::CameraFeedConfig{bad});
        REQUIRE(p != nullptr);
        CHECK(std::string{p} == words);
    }
    CHECK(vision::cameraFeedConfigProblem(vision::CameraFeedConfig{}) != nullptr);  // the default is UNSET
    CHECK(vision::cameraFeedConfigProblem(vision::CameraFeedConfig{std::numeric_limits<double>::denorm_min()}) ==
          nullptr);
    CHECK(vision::cameraFeedConfigProblem(vision::CameraFeedConfig{0.033}) == nullptr);
}

TEST_CASE("camera feed: the period LANDED ON -- the first tick reads; a tick exactly one period after the last read "
          "BEGAN reads, the double before it does not; never two reads in one tick") {
    hal::fake::FakeClock clock{Time{1.0}};
    hal::fake::FakeVisionFrames cam;
    cam.setNext(okFrame(2));
    vision::CameraFeed feed{cam, clock, vision::CameraFeedConfig{kPeriod}};
    CHECK(feed.latest().status == hal::FrameStatus::NotRead);  // before the first read
    CHECK(feed.tick());
    CHECK(cam.reads() == 1U);
    CHECK(feed.latest().status == hal::FrameStatus::Ok);
    // two consumers ticking in the same instant: ONE read
    CHECK_FALSE(feed.tick());
    CHECK_FALSE(feed.tick());
    CHECK(cam.reads() == 1U);
    clock.set(Time{std::nextafter(1.0 + kPeriod, 0.0)});
    CHECK_FALSE(feed.tick());
    CHECK(cam.reads() == 1U);
    clock.set(Time{1.0 + kPeriod});
    CHECK(feed.tick());
    CHECK(cam.reads() == 2U);
    // a long gap still reads ONCE per tick
    clock.set(Time{5.0});
    CHECK(feed.tick());
    CHECK_FALSE(feed.tick());
    CHECK(cam.reads() == 3U);
    CHECK(feed.stats().reads == 3U);
}

TEST_CASE("camera feed: the period runs from a read's START -- a read that takes 20 ms does not push the next one "
          "later") {
    hal::fake::FakeClock clock{Time{0.0}};
    SlowCamera cam{clock};
    cam.setTakes(0.015625);  // 1/64 s
    vision::CameraFeed feed{cam, clock, vision::CameraFeedConfig{kPeriod}};
    CHECK(feed.tick());  // began at 0, the clock now 1/64
    clock.set(Time{std::nextafter(kPeriod, 0.0)});
    CHECK_FALSE(feed.tick());
    clock.set(Time{kPeriod});  // one period after the first read BEGAN (not after it ended)
    CHECK(feed.tick());
    CHECK(cam.reads() == 2U);
}

TEST_CASE("camera feed: latest() is the latest READ -- a fault frame with NO detections replaces a good one, never "
          "the last good frame held over") {
    hal::fake::FakeClock clock{Time{0.0}};
    hal::fake::FakeVisionFrames cam;
    cam.setNext(okFrame(3));
    vision::CameraFeed feed{cam, clock, vision::CameraFeedConfig{kPeriod}};
    REQUIRE(feed.tick());
    REQUIRE(feed.latest().count == 3U);
    cam.setFailing(true);
    clock.advance(Time{kPeriod});
    REQUIRE(feed.tick());
    CHECK(feed.latest().status == hal::FrameStatus::ReadFault);
    CHECK(feed.latest().count == 0U);
    CHECK_FALSE(feed.latest().usable());
    CHECK(feed.latest().readSeq == 2U);
    cam.setFailing(false);
    clock.advance(Time{kPeriod});
    REQUIRE(feed.tick());
    CHECK(feed.latest().count == 3U);
    CHECK(feed.latest().readSeq == 3U);
}

TEST_CASE("camera feed: the stats count each status; 'truncated' only on usable reads; the withheld counts summed") {
    hal::fake::FakeClock clock{Time{0.0}};
    hal::fake::FakeVisionFrames cam;
    vision::CameraFeed feed{cam, clock, vision::CameraFeedConfig{kPeriod}};
    auto readAs = [&](hal::FrameStatus s, bool truncated, std::uint8_t objectsWithheld, std::uint8_t coloursWithheld) {
        hal::VisionFrame f = okFrame(1);
        f.status = s;
        f.truncated = truncated;
        f.objectsWithheld = objectsWithheld;
        f.coloursWithheld = coloursWithheld;
        cam.setNext(f);
        clock.advance(Time{kPeriod});
        REQUIRE(feed.tick());
    };
    readAs(hal::FrameStatus::Ok, false, 2, 1);
    readAs(hal::FrameStatus::Ok, true, 0, 0);
    readAs(hal::FrameStatus::ReadFault, true, 0, 0);  // a fault's flag is no evidence: not counted
    readAs(hal::FrameStatus::PartialFault, false, 1, 0);
    readAs(hal::FrameStatus::PartialFault, false, 0, 0);
    readAs(hal::FrameStatus::NotConfigured, false, 0, 3);
    readAs(hal::FrameStatus::NotRead, false, 0, 0);
    const vision::CameraFeedStats& s = feed.stats();
    CHECK(s.reads == 7U);
    CHECK(s.usable == 2U);
    CHECK(s.truncated == 1U);
    CHECK(s.readFaults == 1U);
    CHECK(s.partialFaults == 2U);
    CHECK(s.notConfigured == 1U);
    CHECK(s.objectsWithheld == 3U);
    CHECK(s.coloursWithheld == 4U);
}

TEST_CASE("camera feed: each read TIMED with the feed's clock -- 4 ms shows 4 ms; the slowest is the largest so far "
          "and a faster read never lowers it") {
    hal::fake::FakeClock clock{Time{0.0}};
    SlowCamera cam{clock};
    vision::CameraFeed feed{cam, clock, vision::CameraFeedConfig{kPeriod}};
    CHECK(feed.stats().lastReadSeconds == 0.0);
    cam.setTakes(0.00390625);  // 1/256 s, binary-exact
    REQUIRE(feed.tick());
    CHECK(feed.stats().lastReadSeconds == 0.00390625);
    CHECK(feed.stats().slowestReadSeconds == 0.00390625);
    cam.setTakes(0.0078125);
    clock.set(Time{1.0});
    REQUIRE(feed.tick());
    CHECK(feed.stats().lastReadSeconds == 0.0078125);
    CHECK(feed.stats().slowestReadSeconds == 0.0078125);
    cam.setTakes(0.001953125);
    clock.set(Time{2.0});
    REQUIRE(feed.tick());
    CHECK(feed.stats().lastReadSeconds == 0.001953125);
    CHECK(feed.stats().slowestReadSeconds == 0.0078125);  // never lowered
}
