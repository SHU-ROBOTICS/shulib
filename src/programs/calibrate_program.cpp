// calibrate_program.cpp — "shulib Calibrate" for the robot this build carries (slot 4): the
// standard Calibrate (include/shulib/hal/pros/calibrate.hpp, chunk CAL1) with this build's facts
// (src/robots/calibrate_facts.hpp). Until CAL1 Part B (2026-11-05) this file WAS the program (1,397 lines).

#include "programs/calibrate_program.hpp"

#include "robots/calibrate_facts.hpp"
#include "shulib/hal/pros/calibrate.hpp"

namespace shulib::bench {
namespace {
/// This build's stamp: the banner prints it whole, the screen's header its time.
constexpr const char* kBuildStamp = __DATE__ " " __TIME__;
}  // namespace

void calibrateInitialize(const char* variantBeacon, const char* /*competitionStatusBits*/) {
    hal::pros::calibrateInitialize(variantBeacon, kBuildStamp);
}
void calibrateDisabled() { hal::pros::calibrateDisabled(); }
void calibrateConnected() { hal::pros::calibrateConnected(); }
void calibrateAutonomous() { hal::pros::calibrateAutonomous(); }
void runCalibrate() {
    const calibration::CalibrateFacts facts = withTypedDriveFrom(kCalibrateFacts, kChassis);
    hal::pros::runCalibrate(facts, kBuildStamp);
}

}  // namespace shulib::bench
