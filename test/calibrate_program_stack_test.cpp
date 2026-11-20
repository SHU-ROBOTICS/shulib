// calibrate_program_stack_test.cpp — CAL2 §6: the calibrator and the run's record live in STATIC
// storage, off the opcontrol task's stack (robot one's Calibrate overflowed its 32 KiB stack at the
// save, 2026-11-17). Each run below goes whole on a thread whose stack is painted first; the bytes it
// wrote must stay under kHostBudget. GCC 13 at -O0 (the CI's compiler) measured 9.7 KB on 2026-11-17;
// the calibrator (8.2 KB here) back on the stack makes it 17.8 KB or more.

#include <pthread.h>

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <string>

#include "calibrate_program_rig.hpp"
#include "doctest.h"
#include "shulib/calibration/calibrate_program.hpp"

namespace {

using namespace cal_prog_rig;
using shulib::calibration::DrivetrainCalibrator;
using shulib::calibration::Phase;
using shulib::calibration::SaveOutcome;
using shulib::calibration::detail::ProgramStorage;
using shulib::calibration::detail::programStorage;

constexpr std::size_t kHostBudget = 12 * 1024;

struct Probe {
    Rig* rig = nullptr;
    bool threw = false;
};

void* runProbe(void* p) {
    auto* probe = static_cast<Probe*>(p);
    try {
        probe->rig->run(robotTwoFacts());
    } catch (...) {
        probe->threw = true;
    }
    return nullptr;
}

void* idleProbe(void*) { return nullptr; }

// The bytes `fn` wrote on a fresh painted thread stack (1 MiB, page-aligned).
std::size_t stackWritten(void* (*fn)(void*), void* arg) {
    constexpr std::size_t kSize = std::size_t{1} << 20;
    constexpr unsigned char kPaint = 0xA5;
    auto* mem = static_cast<unsigned char*>(std::aligned_alloc(4096, kSize));
    REQUIRE(mem != nullptr);
    std::memset(mem, kPaint, kSize);
    pthread_attr_t attr;
    REQUIRE(pthread_attr_init(&attr) == 0);
    REQUIRE(pthread_attr_setstack(&attr, mem, kSize) == 0);
    pthread_t t;
    REQUIRE(pthread_create(&t, &attr, fn, arg) == 0);
    REQUIRE(pthread_join(t, nullptr) == 0);
    pthread_attr_destroy(&attr);
    std::size_t clean = 0;
    while (clean < kSize && mem[clean] == kPaint) {
        ++clean;
    }
    std::free(mem);
    return kSize - clean;
}

// A whole run's stack, less what an empty thread writes (glibc keeps the thread's own block on top).
// The static storage is poisoned first: after the run it must hold THAT run's calibrator and record.
std::size_t runStack(Rig& rig, int lastRun, SaveOutcome save) {
    ProgramStorage& st = programStorage();
    st.cal.reset();
    st.run.number = -7;
    Probe probe{&rig};
    const std::size_t idle = stackWritten(&idleProbe, nullptr);
    const std::size_t whole = stackWritten(&runProbe, &probe);
    REQUIRE_FALSE(probe.threw);
    REQUIRE_FALSE(rig.timedOut);
    REQUIRE(whole > idle);
    REQUIRE(st.cal.has_value());
    CHECK(st.cal->phase() == Phase::Done);
    CHECK(st.run.number == lastRun);
    CHECK(st.run.save == save);
    INFO("stack ", whole - idle, " B; a calibrator ", sizeof(DrivetrainCalibrator), " B");
    CHECK(whole - idle < kHostBudget);
    return whole - idle;
}

}  // namespace

TEST_CASE("CalibrateProgram T-STACK (CAL2): a whole run's stack stays under 12 KiB and its calibrator and "
          "record are the static ones -- the save with a card, with no card, a HELD run saved by USE NEW") {
    static_assert(sizeof(DrivetrainCalibrator) > kHostBudget / 2, "a calibrator on the stack must break the budget");
    SUBCASE("the save, with a card") {
        Scenario sc;
        Rig rig{sc};
        runStack(rig, 1, SaveOutcome::Saved);
        CHECK(countPrefix(rig.io.lines, "SAVED -- /usd/shulib_drivetrain.bin") == 1);
    }
    SUBCASE("no card") {
        Scenario sc;
        sc.card = false;
        Rig rig{sc};
        runStack(rig, 1, SaveOutcome::NotSaved);
        CHECK(findLine(rig.io.lines, "NOT SAVED -- no SD card in the brain.") != std::string::npos);
    }
    SUBCASE("FRONT FLIPPED held, then USE NEW") {
        Scenario sc;
        sc.runs = 2;
        sc.pushDir[1] = -1.0;
        sc.heldFlip[1] = kLeftBox;
        Rig rig{sc};
        runStack(rig, 2, SaveOutcome::Saved);
        CHECK(findPrefix(rig.io.lines, "front: USE NEW -- the end pushed first THIS time") != std::string::npos);
        CHECK(countPrefix(rig.io.lines, "SAVED -- /usd/shulib_drivetrain.bin") == 2);
    }
}
