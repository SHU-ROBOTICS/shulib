#pragma once
// The driver file's header map, read as the route builder reads it — shared by both robots' driver tests (moved
// out of test/robot_one_driver_test.cpp on 2026-11-20, when robot two's map became its own).

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "doctest.h"
#include "shulib/hal/controller.hpp"

namespace driver_map {

/// A button's name as the driver file spells it (hal::ControllerButton's order).
inline const char* buttonName(shulib::hal::ControllerButton b) {
    static constexpr const char* kNames[12] = {"L1", "L2", "R1", "R2", "Up", "Down", "Left", "Right", "X", "B", "Y", "A"};
    return kNames[static_cast<std::size_t>(b)];
}

/// The header map's rows (`//   <key>  <words>`, continuation lines joined): key -> words. Read from the line after
/// the route builder's `// [control words …]` marker, as the page reads it (the `[the map]` heading above it is the
/// team's prose, one line or more).
inline std::vector<std::pair<std::string, std::string>> mapRows(const std::string& text) {
    std::vector<std::pair<std::string, std::string>> rows;
    std::size_t at = text.find("// [control words ");
    if (at == std::string::npos) at = text.find("// [the map]");
    REQUIRE(at != std::string::npos);
    at = text.find('\n', at) + 1;
    while (at < text.size()) {
        const std::size_t end = text.find('\n', at);
        const std::string line = text.substr(at, end - at);
        at = end + 1;
        if (line.starts_with("// [")) continue;  // the route builder's marker lines
        if (!line.starts_with("//   ")) break;
        if (line.size() > 5 && line[5] == ' ') {
            if (!rows.empty()) rows.back().second += " " + line.substr(5);
            continue;
        }
        const std::size_t gap = line.find("  ", 5);
        rows.emplace_back(line.substr(5, gap - 5), gap == std::string::npos ? "" : line.substr(gap));
    }
    return rows;
}

/// The buttons a map key names, read as the route builder reads them ("R1 / R2", "Left/Right", "d-pad", "B, X").
inline std::vector<shulib::hal::ControllerButton> keyButtons(const std::string& key) {
    using shulib::hal::ControllerButton;
    std::vector<ControllerButton> out;
    std::size_t from = 0;
    while (from <= key.size()) {
        std::size_t to = key.find_first_of("/,", from);
        if (to == std::string::npos) to = key.size();
        std::string word = key.substr(from, to - from);
        word.erase(0, word.find_first_not_of(' '));
        word.erase(word.find_last_not_of(' ') + 1);
        if (word == "d-pad") {
            for (const auto b : {ControllerButton::Up, ControllerButton::Down, ControllerButton::Left, ControllerButton::Right}) out.push_back(b);
        }
        for (int i = 0; i < 12; ++i) {
            if (word == buttonName(static_cast<ControllerButton>(i))) out.push_back(static_cast<ControllerButton>(i));
        }
        from = to + 1;
    }
    return out;
}

}  // namespace driver_map
