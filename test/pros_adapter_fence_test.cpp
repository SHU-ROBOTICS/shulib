// Structural guard test for the hal/pros adapter tree (chunk R1a). Reads the
// adapter SOURCE (SHULIB_SOURCE_DIR, the api_reference_fidelity_test pattern)
// and pins the properties that no runtime test can see:
//
//  * the diagnostic fence's suppression set is EXACTLY {-Wshadow,
//    -Wsign-conversion} — measured sufficient (brief M2/M3); anything wider is
//    warnings turned off for our own code with a pragma in front of it
//    (mutation M11's subject);
//  * the fence CLOSES (pop) before any shulib code — a pop at end-of-file
//    covers the adapter body and -Wconversion et al. go dark silently;
//  * every <pros/...> include sits INSIDE a fence;
//  * the forbidden PROS calls are ABSENT as text: the tare family in the IMU
//    adapter (HA-05), set_offset/initialize_full in the GPS adapter (HA-06),
//    get_digital_new_press in the controller adapter (HA-104/trap C);
//  * every shim header carries the #error + SHULIB_HOST_PROS_SHIM gate
//    (mutation M12's subject) — a shim that can compile outside the host test
//    build is a robot binary that drives nothing.

#include "doctest.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

std::string readFile(const fs::path& p) {
    std::ifstream in(p);
    REQUIRE_MESSAGE(in.good(), "cannot open ", p.string(),
                    " — a guard that cannot read its subject is decoration");
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// Strip // comments (the tree's only comment style) so the forbidden-call
/// scan reads CODE — the adapters' own headers legitimately NAME the banned
/// calls while documenting why they are banned.
std::string withoutLineComments(const std::string& src) {
    std::istringstream in(src);
    std::ostringstream out;
    std::string line;
    while (std::getline(in, line)) {
        const auto slashes = line.find("//");
        out << (slashes == std::string::npos ? line : line.substr(0, slashes)) << '\n';
    }
    return out.str();
}

std::vector<fs::path> filesIn(const fs::path& dir, const std::string& ext) {
    std::vector<fs::path> out;
    REQUIRE_MESSAGE(fs::exists(dir), dir.string(), " does not exist");
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() == ext) {
            out.push_back(e.path());
        }
    }
    REQUIRE_FALSE(out.empty());
    return out;
}

const fs::path kAdapterDir = fs::path(SHULIB_SOURCE_DIR) / "include/shulib/hal/pros";
const fs::path kShimDir = fs::path(SHULIB_SOURCE_DIR) / "test/pros_shim/pros";

}  // namespace

TEST_CASE("fence: every adapter's suppression set is EXACTLY -Wshadow + -Wsign-conversion") {
    // BUG CAUGHT (mutation M11 family): the list growing — someone adds
    // "-Wconversion" (measured NOT needed, M3) to make a sloppy adapter
    // compile, and from then on real conversion bugs in the include chain are
    // invisible. Also catches a fence with a MISSING flag, which would break
    // the ARM gate on PROS's own headers.
    for (const auto& f : filesIn(kAdapterDir, ".hpp")) {
        const std::string src = readFile(f);
        if (src.find("#include \"pros/") == std::string::npos) {
            // char_sink.hpp: deliberately PROS-free (stdout only) — no fence needed.
            CHECK(src.find("#pragma GCC diagnostic") == std::string::npos);
            continue;
        }
        const auto push = src.find("#pragma GCC diagnostic push");
        const auto pop = src.find("#pragma GCC diagnostic pop");
        REQUIRE_MESSAGE(push != std::string::npos, f.filename().string(), ": no fence push");
        REQUIRE_MESSAGE(pop != std::string::npos, f.filename().string(), ": no fence pop");

        // Count EVERY "ignored" pragma in the whole file (not just the fence
        // block) — a second fence elsewhere is the same hole.
        std::size_t ignoredCount = 0;
        std::size_t at = 0;
        while ((at = src.find("#pragma GCC diagnostic ignored", at)) != std::string::npos) {
            ignoredCount += 1;
            at += 1;
        }
        CHECK_MESSAGE(ignoredCount == 2, f.filename().string(),
                      ": suppression set must be exactly two flags, found ", ignoredCount);
        CHECK(src.find("#pragma GCC diagnostic ignored \"-Wshadow\"") != std::string::npos);
        CHECK(src.find("#pragma GCC diagnostic ignored \"-Wsign-conversion\"")
              != std::string::npos);
    }
}

TEST_CASE("fence: closes BEFORE shulib code and covers every <pros/> include") {
    // BUG CAUGHT (mutation M11): the pop moved to end-of-file — the fence
    // then covers the adapter BODY, and -Wshadow/-Wsign-conversion defects in
    // OUR code compile silently (the M6 negative control proved the fence
    // does not protect adapter code only BECAUSE the pop comes first).
    for (const auto& f : filesIn(kAdapterDir, ".hpp")) {
        const std::string src = readFile(f);
        const auto pop = src.find("#pragma GCC diagnostic pop");
        const auto ns = src.find("namespace shulib");
        if (src.find("#include \"pros/") == std::string::npos) {
            continue;  // char_sink.hpp — no fence to scope-check
        }
        REQUIRE(ns != std::string::npos);
        CHECK_MESSAGE(pop < ns, f.filename().string(),
                      ": the fence pop must come BEFORE any shulib code");
        // Every <pros/...> include sits between push and pop:
        const auto push = src.find("#pragma GCC diagnostic push");
        std::size_t at = 0;
        while ((at = src.find("#include \"pros/", at)) != std::string::npos) {
            CHECK_MESSAGE(push < at, f.filename().string(), ": a <pros/> include before push");
            CHECK_MESSAGE(at < pop, f.filename().string(), ": a <pros/> include after pop");
            at += 1;
        }
    }
}

TEST_CASE("forbidden calls are textually absent from the adapters (the contract bans)") {
    // BUG CAUGHT: a future edit "helpfully" adding imu.tare_heading() after a
    // fault, gps.set_offset() to "simplify" the lever arm, or the consuming
    // get_digital_new_press() — each is a documented contract breach (HA-05,
    // HA-06, HA-104) whose runtime symptom is silent.
    const std::string imu = withoutLineComments(readFile(kAdapterDir / "imu.hpp"));
    for (const char* banned : {"tare(", "tare_rotation(", "set_rotation(", "tare_heading(",
                               "set_heading("}) {
        CHECK_MESSAGE(imu.find(banned) == std::string::npos, "imu.hpp calls ", banned);
    }
    // get_heading() is banned by HA-03's binding contract even though a
    // heading()-site swap is OBSERVATIONALLY EQUIVALENT through the wrapping
    // Angle (mutation M4b stayed GREEN behaviourally, and honestly so — the
    // wrap erases the difference at that one seam). The contract still
    // mandates get_rotation() because the DIFFERENTIATOR is not equivalent
    // (mutation M4 went RED on a phantom 624 rad/s spike at the 360° seam),
    // and a future consumer of cumulative heading would not be either. A
    // behaviour test cannot pin an equivalence; this textual pin can.
    CHECK_MESSAGE(imu.find("get_heading(") == std::string::npos,
                  "imu.hpp calls get_heading — HA-03 mandates the cumulative get_rotation()");

    const std::string gps = withoutLineComments(readFile(kAdapterDir / "gps.hpp"));
    for (const char* banned : {"set_offset(", "initialize_full("}) {
        CHECK_MESSAGE(gps.find(banned) == std::string::npos, "gps.hpp calls ", banned);
    }

    const std::string controller =
        withoutLineComments(readFile(kAdapterDir / "controller.hpp"));
    CHECK(controller.find("get_digital_new_press") == std::string::npos);
}

TEST_CASE("shim: every header #errors without SHULIB_HOST_PROS_SHIM (structurally robot-proof)") {
    // BUG CAUGHT (mutation M12): a shim header losing its guard — if
    // test/pros_shim/ ever reached the PROS Makefile's include path, the
    // robot binary would build against in-memory fake motors, upload cleanly,
    // boot cleanly, print a healthy banner, and drive NOTHING. The #error is
    // what makes that failure impossible rather than unlikely.
    for (const auto& f : filesIn(kShimDir, ".hpp")) {
        const std::string src = readFile(f);
        CHECK_MESSAGE(src.find("#ifndef SHULIB_HOST_PROS_SHIM") != std::string::npos,
                      f.filename().string(), ": missing the shim guard");
        CHECK_MESSAGE(src.find("#error") != std::string::npos, f.filename().string(),
                      ": missing the #error");
    }
    for (const auto& f : filesIn(kShimDir, ".h")) {
        const std::string src = readFile(f);
        CHECK_MESSAGE(src.find("#ifndef SHULIB_HOST_PROS_SHIM") != std::string::npos,
                      f.filename().string(), ": missing the shim guard");
        CHECK_MESSAGE(src.find("#error") != std::string::npos, f.filename().string(),
                      ": missing the #error");
    }
}

TEST_CASE("only hal/pros includes <pros/> — the in-tree mirror of the CI guard") {
    // BUG CAUGHT (mutation M13's subject, from inside the suite): a <pros/>
    // include smuggled anywhere else under include/shulib/ — the PATH-ANCHORED
    // scan here mirrors the amended CI guard, so a violation fails locally
    // before CI ever sees it. (--exclude-dir=pros was measured to MISS
    // include/shulib/localization/pros/ — this scan, like CI's, exempts the
    // one path by prefix instead.)
    const fs::path root = fs::path(SHULIB_SOURCE_DIR) / "include/shulib";
    for (const auto& e : fs::recursive_directory_iterator(root)) {
        if (!e.is_regular_file() || e.path().extension() != ".hpp") {
            continue;
        }
        const std::string rel = fs::relative(e.path(), root).generic_string();
        if (rel.rfind("hal/pros/", 0) == 0) {
            continue;  // the ONE exempt path, anchored at the tree root
        }
        const std::string src = readFile(e.path());
        CHECK_MESSAGE(src.find("#include <pros/") == std::string::npos, rel,
                      " includes <pros/> outside hal/pros/");
        CHECK_MESSAGE(src.find("#include \"pros/") == std::string::npos, rel,
                      " includes \"pros/\" outside hal/pros/");
    }
}
