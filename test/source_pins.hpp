#pragma once
//
// source_pins.hpp — the helpers a SOURCE PIN reads a src/ file with: what the host cannot compile
// (a program TU includes pros/screen.h) is read as text through SHULIB_SOURCE_DIR (the
// api_reference_fidelity_test pattern), comments are stripped before any code pin — so a sentence
// that mentions a name is never mistaken for a call — and a pin counts, finds or compares code.
//
// Written by chunk COMP-t (2026-09-28) inside test/auton_tester_source_test.cpp; MOVED here,
// unchanged, at COMP-b (2026-09-30), when the competition program's pins
// (test/competition_source_test.cpp) became the second user — one copy, never two.
//
// A pin catches only the spellings it names. What it guards best is what no host test can reach:
// a PROS read, the order of a boot, the wiring of a program's objects.

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "doctest.h"

namespace source_pins {

/// The file at `rel` (relative to the repository root), whole. A file that cannot be read FAILS
/// the case: a pin that cannot read its subject is decoration.
inline std::string readFile(const std::string& rel) {
    const std::filesystem::path p = std::filesystem::path(SHULIB_SOURCE_DIR) / rel;
    std::ifstream in(p);
    REQUIRE_MESSAGE(in.good(), "cannot open ", p.string(),
                    " — a pin that cannot read its subject is decoration");
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// `text` with // and /* */ comments removed (string and character literals kept, line breaks
/// kept).
inline std::string stripComments(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    std::size_t i = 0;
    const std::size_t n = text.size();
    while (i < n) {
        const char c = text[i];
        if (c == '"' || c == '\'') {
            const char q = c;
            out += c;
            ++i;
            while (i < n && text[i] != q) {
                if (text[i] == '\\' && i + 1 < n) {
                    out += text[i];
                    ++i;
                }
                out += text[i];
                ++i;
            }
            if (i < n) {
                out += text[i];
                ++i;
            }
        } else if (c == '/' && i + 1 < n && text[i + 1] == '/') {
            while (i < n && text[i] != '\n') {
                ++i;
            }
        } else if (c == '/' && i + 1 < n && text[i + 1] == '*') {
            i += 2;
            while (i + 1 < n && !(text[i] == '*' && text[i + 1] == '/')) {
                if (text[i] == '\n') {
                    out += '\n';
                }
                ++i;
            }
            i += 2;
        } else {
            out += c;
            ++i;
        }
    }
    return out;
}

/// Every string literal's contents in `code` (comments already stripped), adjacent ones joined.
inline std::vector<std::string> stringLiterals(const std::string& code) {
    std::vector<std::string> out;
    std::size_t i = 0;
    bool joining = false;
    while (i < code.size()) {
        if (code[i] == '\'') {  // a character literal: skip it whole
            ++i;
            while (i < code.size() && code[i] != '\'') {
                i += code[i] == '\\' ? std::size_t{2} : std::size_t{1};
            }
            ++i;
            continue;
        }
        if (code[i] != '"') {
            if (code[i] != ' ' && code[i] != '\n' && code[i] != '\t') {
                joining = false;
            }
            ++i;
            continue;
        }
        std::string lit;
        ++i;
        while (i < code.size() && code[i] != '"') {
            if (code[i] == '\\' && i + 1 < code.size()) {
                lit += code[i];
                ++i;
            }
            lit += code[i];
            ++i;
        }
        ++i;
        if (joining && !out.empty()) {
            out.back() += lit;
        } else {
            out.push_back(lit);
        }
        joining = true;
    }
    return out;
}

/// How many times `needle` occurs in `hay`, without overlap.
inline std::size_t count(const std::string& hay, const std::string& needle) {
    std::size_t n = 0;
    for (std::size_t at = hay.find(needle); at != std::string::npos;
         at = hay.find(needle, at + needle.size())) {
        ++n;
    }
    return n;
}

/// The body of the first function whose header contains `signature`: from its `{` to the
/// matching `}` (braces inside literals are not in the files this reads).
inline std::string bodyOf(const std::string& code, const std::string& signature) {
    const std::size_t at = code.find(signature);
    REQUIRE_MESSAGE(at != std::string::npos, "no function '", signature, "'");
    const std::size_t open = code.find('{', at);
    int depth = 0;
    for (std::size_t i = open; i < code.size(); ++i) {
        if (code[i] == '{') {
            ++depth;
        } else if (code[i] == '}' && --depth == 0) {
            return code.substr(open + 1, i - open - 1);
        }
    }
    FAIL("unbalanced braces after '", signature, "'");
    return {};
}

/// `text` with every run of whitespace collapsed to one space, trimmed.
inline std::string squash(const std::string& text) {
    std::string out;
    bool space = false;
    for (const char c : text) {
        if (c == ' ' || c == '\n' || c == '\t' || c == '\r') {
            space = !out.empty();
            continue;
        }
        if (space) {
            out += ' ';
            space = false;
        }
        out += c;
    }
    return out;
}

}  // namespace source_pins
