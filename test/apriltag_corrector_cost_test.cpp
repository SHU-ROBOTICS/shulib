// The COST PIN for AprilTagCorrector (chunk E3, tension T4).
//
// `ITagSource::tags()` returns `std::vector<TagObservation>` BY VALUE and is FROZEN (F4): it
// heap-allocates on every call and that cannot be changed. hal/vision.hpp is explicit that vision
// runs OFF the 10 ms control path. So the corrector splits into `poll()` (vision cadence, the only
// method that touches the HAL, the only one that allocates) and `propose()` (every control tick,
// allocation-free on every path).
//
// The brief's instruction was "pin the cost — do not assert it", and the difference matters: a
// comment saying "propose() never allocates" is a wish, and a `static_assert` cannot see a heap
// allocation at all. So this file REPLACES THE GLOBAL ALLOCATOR and counts. If a future change
// puts a std::vector, a std::function, a std::string or a tags() call anywhere on the tick path,
// the count moves and this file goes red.
//
// The instrument is proved before it is trusted: `poll()` MUST show a non-zero count. Without
// that check, a counter that was silently broken would make every "zero allocations" assertion
// below vacuously true — which is exactly the shape of test that looks like evidence and is not.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <span>

#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/vision.hpp"
#include "shulib/localization/apriltag_corrector.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tag_map.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib_alloc_probe {
// Deliberately NOT in an anonymous namespace: the replacement operators below are at global
// scope and must see these. Single-threaded by contract, like every test in this suite.
std::size_t allocations = 0;
bool counting = false;

/// Counts allocations across a scope. Nothing inside a Probe may use doctest macros — doctest
/// allocates for its own bookkeeping and would be counted as if it were the code under test.
struct Probe {
    Probe() {
        allocations = 0;
        counting = true;
    }
    ~Probe() { counting = false; }
    Probe(const Probe&) = delete;
    Probe& operator=(const Probe&) = delete;
    [[nodiscard]] static std::size_t count() { return allocations; }
};
}  // namespace shulib_alloc_probe

void* operator new(std::size_t size) {
    if (shulib_alloc_probe::counting) {
        ++shulib_alloc_probe::allocations;
    }
    void* p = std::malloc(size != 0 ? size : 1);
    if (p == nullptr) {
        throw std::bad_alloc{};
    }
    return p;
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using shulib::hal::TagObservation;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::hal::fake::FakeTagSource;
using shulib::localization::AprilTagCorrector;
using shulib::localization::ComplementaryFusion;
using shulib::localization::CorrectionProposal;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TagMap;
using shulib::localization::TagPlacement;
using shulib::localization::TagProvenance;
using shulib::localization::TrackingWheel;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;
using shulib_alloc_probe::Probe;

namespace {

[[nodiscard]] Pose2d tagAsSeenFrom(const Pose2d& robot, const Pose2d& tag) {
    const double dx = tag.x().value() - robot.x().value();
    const double dy = tag.y().value() - robot.y().value();
    const double c = std::cos(robot.heading().radians());
    const double s = std::sin(robot.heading().radians());
    return Pose2d{Length{dx * c + dy * s}, Length{-dx * s + dy * c},
                  Angle::radians(tag.heading().radians() - robot.heading().radians())};
}

const Pose2d kTruth{Length{18.0}, Length{-9.0}, Angle::degrees(25.0)};
const Pose2d kTag{Length{18.0 + 28.0 * 0.9063077870366499},
                  Length{-9.0 + 28.0 * 0.42261826174069944},
                  Angle::degrees(25.0 + 180.0)};

}  // namespace

// Would catch: the instrument being broken, which would make every assertion below vacuously
// true. poll() calls the FROZEN tags() seam, which returns a vector by value, so it MUST show
// allocations. If this case ever reads zero, nothing else in this file means anything.
TEST_CASE("cost: the allocation counter actually works — poll() allocates, by design") {
    FakeClock clk{Time{5.0}};
    FakeImu imu;
    FakeTagSource source;
    TagMap map;
    map.add(TagPlacement{7, kTag, TagProvenance::Invented, "host test fixture"});
    AprilTagCorrector corrector{clk, source, imu, map};
    source.setTags({TagObservation{7, tagAsSeenFrom(kTruth, kTag), 0.9}});

    corrector.poll();  // warm any one-time state before measuring
    std::size_t seen = 0;
    {
        Probe probe;
        for (int k = 0; k < 50; ++k) {
            corrector.poll();
        }
        seen = Probe::count();
    }
    CHECK(seen > 0);  // the F4 seam's by-value vector, off the control path where it belongs
}

// Would catch: THE T4 VIOLATION — a heap allocation on the 10 ms control path. A propose() that
// called tags() (or grew a vector, or captured into a std::function) would allocate here, and
// A1's cost contract forbids it. Calling tags() "only every fifth tick" would not fix that
// either: it would make the allocation intermittent, which is harder to diagnose, not cheaper.
//
// Twenty thousand ticks — 200 seconds of control loop, longer than three matches — with tags
// visible the whole time and a mixture of accepted, stale and rejected verdicts, so every branch
// of propose() is exercised and not merely the early-out.
TEST_CASE("cost: propose() allocates EXACTLY ZERO times across 20,000 ticks") {
    FakeClock clk{Time{5.0}};
    FakeImu imu;
    FakeTagSource source;
    TagMap map;
    map.add(TagPlacement{7, kTag, TagProvenance::Invented, "host test fixture"});
    AprilTagCorrector corrector{clk, source, imu, map};
    source.setTags({TagObservation{7, tagAsSeenFrom(kTruth, kTag), 0.9},
                    TagObservation{99, tagAsSeenFrom(kTruth, kTag), 0.9}});
    corrector.poll();
    (void)corrector.propose(kTruth, Time{0.01});  // warm every lazily-initialised path

    std::size_t seen = 0;
    int accepted = 0;
    {
        Probe probe;
        for (int k = 0; k < 20000; ++k) {
            clk.advance(Time{0.01});
            const CorrectionProposal p = corrector.propose(kTruth, Time{0.01});
            if (p.valid) {
                ++accepted;
            }
            if (k % 5 == 0) {
                // poll() is OUTSIDE the measurement window on purpose — it is the vision-cadence
                // method and it is allowed to allocate. Counting is paused around it so this test
                // measures the CONTROL PATH and only the control path.
                shulib_alloc_probe::counting = false;
                corrector.poll();
                shulib_alloc_probe::counting = true;
            }
        }
        seen = Probe::count();
    }
    CHECK(seen == 0);
    CHECK(accepted > 3500);  // the loop really did fold fixes, not just early-out
}

// Would catch: an allocation introduced anywhere else on the fused tick — the fusion policy's
// heading loop, the Localizer's proposal buffer, the audit substitution. propose() being clean is
// necessary and not sufficient; what the robot actually runs is Localizer::update().
TEST_CASE("cost: a full Localizer tick with a tag corrector allocates zero times") {
    FakeClock clk{Time{5.0}};
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    FakeTagSource source;
    TagMap map;
    map.add(TagPlacement{7, kTag, TagProvenance::Invented, "host test fixture"});
    PilonsOdometry odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
                        TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})};
    ComplementaryFusion fusion;
    AprilTagCorrector corrector{clk, source, imu, map};
    std::array<ICorrector*, 1> correctors{&corrector};
    Localizer loc{clk, imu, odom, fusion, std::span<ICorrector* const>{correctors}};

    imu.setHeading(kTruth.heading());
    loc.setPose(kTruth);
    // The IMU is 5 degrees wrong, so the heading path is LIVE for the whole measurement — a
    // measurement taken with the correction switched off would prove nothing about it.
    imu.setHeading(Angle::degrees(kTruth.heading().degrees() - 5.0));
    source.setTags({TagObservation{7, tagAsSeenFrom(kTruth, kTag), 0.9}});
    corrector.poll();
    clk.advance(Time{0.01});
    loc.update();

    std::size_t seen = 0;
    {
        Probe probe;
        for (int k = 0; k < 5000; ++k) {
            clk.advance(Time{0.01});
            if (k % 5 == 0) {
                shulib_alloc_probe::counting = false;
                corrector.poll();
                shulib_alloc_probe::counting = true;
            }
            fwdRot.setPosition(shulib::units::AngleDim{static_cast<double>(k) * 0.05});
            loc.update();
        }
        seen = Probe::count();
    }
    CHECK(seen == 0);
    CHECK(loc.headingBias().value() != 0.0);  // the heading path really was live throughout
}
