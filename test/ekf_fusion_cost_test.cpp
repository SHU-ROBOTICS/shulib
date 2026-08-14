// The COST PIN for EkfFusion (chunk E4).
//
// The brief's instruction is "pin the cost — do not assert it". A 5x5 covariance, three matrix
// products and up to five measurement updates every 10 ms is the largest arithmetic load the
// control loop has ever carried, and the one place it could go wrong is not speed but
// ALLOCATION: a `std::vector` for the proposal ordering, a `std::function` for a callback, a
// temporary matrix type that heap-allocates its storage. Any of those is invisible in a host
// test that only checks numbers, and on a V5 brain a heap allocation inside the control loop is
// a latency spike nobody can attribute afterwards.
//
// So this file COUNTS. It does not re-replace the global allocator — E3's
// `apriltag_corrector_cost_test.cpp` already does that, once, for the whole binary, and a second
// replacement would be a duplicate definition. It borrows that instrument, and it proves the
// instrument is live before trusting it: a deliberate allocation MUST show up. Without that
// check a silently broken counter would make every "zero allocations" assertion below vacuously
// true, which is the exact shape of a test that looks like evidence and is not.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/localization/ekf_fusion.hpp"
#include "shulib/localization/fake/fake_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

// The counters live in `apriltag_corrector_cost_test.cpp`, at namespace scope with external
// linkage, behind the global operator new/delete replacements defined there. Declared (not
// defined) here so both files instrument the same program-wide allocator.
namespace shulib_alloc_probe {
extern std::size_t allocations;
extern bool counting;
}  // namespace shulib_alloc_probe

using shulib::localization::CorrectionProposal;
using shulib::localization::EkfFusion;
using shulib::localization::EkfFusionConfig;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TrackingWheel;
using shulib::localization::fake::FakeCorrector;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// Counts allocations across a scope. Nothing inside may use a doctest macro — doctest
/// allocates for its own bookkeeping and would be counted as if it were the code under test.
struct CountScope {
    CountScope() {
        shulib_alloc_probe::allocations = 0;
        shulib_alloc_probe::counting = true;
    }
    ~CountScope() { shulib_alloc_probe::counting = false; }
    CountScope(const CountScope&) = delete;
    CountScope& operator=(const CountScope&) = delete;
    [[nodiscard]] static std::size_t count() { return shulib_alloc_probe::allocations; }
};

[[nodiscard]] CorrectionProposal fixAt(double x, double y, double sigma, bool heading = false) {
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{x}, Length{y}, Angle::degrees(heading ? 3.0 : 0.0)};
    p.confidence = 0.8;
    p.positionStdDev = Length{sigma};
    p.providesHeading = heading;
    return p;
}

}  // namespace

// Would catch: the instrument being dead, which would make every count below meaningless. A
// std::vector that grows MUST allocate.
TEST_CASE("[cost] the allocation counter is live (positive control)") {
    std::size_t n = 0;
    {
        CountScope probe;
        std::vector<double> v;
        for (int i = 0; i < 100; ++i) {
            v.push_back(static_cast<double>(i));
        }
        n = CountScope::count();
    }
    CHECK(n > 0);
}

// Would catch: a heap allocation on the fusion hot path. Every branch is exercised inside the
// window — the prediction, the odometry update, four simultaneous proposals in a non-trivial
// sigma order, the gate accepting and rejecting, the rate clamp binding, the heading channel,
// and the re-init path — so this is not measuring an early-out.
TEST_CASE("[cost] EkfFusion::fuse allocates ZERO times across 20,000 ticks") {
    EkfFusion f{};
    std::size_t allocations = 0;
    int applied = 0;
    int gated = 0;
    int headingApplied = 0;
    double x = 0.0;
    double y = 0.0;
    {
        CountScope probe;
        for (int i = 0; i < 20000; ++i) {
            // a moving prediction, so the propagation and the Jacobians are real work
            const double h = 0.01;
            const double heading = 0.4 * std::sin(0.001 * i);
            const Pose2d predicted{Length{x + 0.2 * std::cos(heading)},
                                   Length{y + 0.2 * std::sin(heading)}, Angle::radians(heading)};
            // four proposals whose sigmas are deliberately NOT in arrival order, so the
            // ordering pass does real work; the last one also claims a heading; one of them is
            // far enough away to be gated for most of the run.
            const std::array<CorrectionProposal, 4> ps{
                fixAt(predicted.x().value() + 0.5, predicted.y().value(), 1.2),
                fixAt(predicted.x().value() - 0.3, predicted.y().value() + 0.4, 0.4),
                fixAt(predicted.x().value() + 40.0, predicted.y().value(), 0.9),
                fixAt(predicted.x().value(), predicted.y().value() - 0.2, 0.7, true)};
            const auto r = f.fuse(predicted, std::span<const CorrectionProposal>{ps}, Time{h});
            x = r.x.value();
            y = r.y.value();
            applied += r.applied ? 1 : 0;
            gated += r.gated ? 1 : 0;
            headingApplied += r.headingApplied ? 1 : 0;
        }
        allocations = CountScope::count();
    }
    MESSAGE("20,000 ticks: applied on ", applied, ", gated on ", gated, ", heading applied on ",
            headingApplied, "; re-inits ", f.reinitCount());
    CHECK(allocations == 0);
    // …and the window really did exercise the branches, so the zero is not an early-out.
    CHECK(applied > 15000);
    CHECK(gated > 15000);
    CHECK(headingApplied > 15000);
    CHECK(f.numericGuardTrips() == 0);
}

// Would catch: an allocation somewhere ELSE on the tick that only appears once the EKF is the
// installed policy — in the Localizer's proposal buffer, in the audit record, in the heading
// bias path. The fusion policy being clean is necessary and not sufficient; what has to be
// allocation-free is the whole `Localizer::update()`.
TEST_CASE("[cost] a full Localizer::update() with the EKF installed allocates ZERO times") {
    shulib::hal::fake::FakeClock clk;
    shulib::hal::fake::FakeImu imu;
    shulib::hal::fake::FakeRotation fwdRot;
    shulib::hal::fake::FakeRotation latRot;
    PilonsOdometry odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
                        TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})};
    EkfFusion fusion{};
    FakeCorrector a{"a"};
    FakeCorrector b{"b"};
    std::array<ICorrector*, 2> correctors{&a, &b};
    Localizer loc{clk, imu, odom, fusion, std::span<ICorrector* const>{correctors}};
    imu.setReady(true);
    imu.setYawRate(AngularVelocity{0.0});

    // warm up outside the counting window (the first ticks initialise the filter)
    for (int i = 0; i < 20; ++i) {
        clk.advance(Time{0.01});
        loc.update();
    }

    std::size_t allocations = 0;
    double travel = 0.0;
    {
        CountScope probe;
        for (int i = 0; i < 5000; ++i) {
            const Pose2d p = loc.pose();
            a.setProposal(fixAt(p.x().value() + 0.4, p.y().value(), 0.5));
            b.setProposal(fixAt(p.x().value(), p.y().value() + 0.3, 1.1, true));
            clk.advance(Time{0.01});
            travel += 0.1;
            fwdRot.setPosition(AngleDim{travel});
            loc.update();
        }
        allocations = CountScope::count();
    }
    CHECK(allocations == 0);
    CHECK(loc.headingBias().value() != 0.0);  // the heading path was live during the window
    CHECK(fusion.acceptedFixes() > 1000);     // …and so was the correction path
}
