// record_store_platform_scan_test.cpp — the brain has no rename, no delete and no read-write open
// (HA-166), and a HOST test cannot see it: Linux has all three, so a save built on any of them passes
// every host test and fails on the robot. This scan is the only place the host can hold the line
// (SAFE-SAVE Part A, landmine 1). It reads the CODE of every include/shulib header and every src/
// file — comments stripped, trailing ones included, and string/char literals skipped — and checks:
//   * no call to rename, remove, unlink, link, stat or mkdir (HA-166 (2));
//   * no call to freopen, fdopen, openat or creat, and no #include of <fcntl.h>, <sys/fcntl.h>,
//     <fstream> or <filesystem> — the ways to open a file AROUND the adapters (since the review of
//     2026-10-01: a read-write freopen passed the first version of this scan);
//   * fopen appears ONLY in hal/pros/block_sink.hpp, hal/pros/block_source.hpp and
//     src/programs/bench_r3a.cpp (its SD probe) — the adapters own every open;
//   * every fopen's mode is a string LITERAL with no '+' (HA-166 (3): a read-write open fails) —
//     except COMP-t's one pinned forwarding opener, every mode handed to which is held to the same
//     rule (see kForwardingOpener);
//   * hal/pros/file_store.hpp spells no "/usd/" of its own (HA-122: the adapters own the prefix).
// The scanner's self-test pins EVERY banned name, open and header from its own list, so dropping
// one from the scan's tables goes red even though the tree has no such call.
// At 9239538 the only raw match in the whole tree was a trailing COMMENT —
// motion_scheduler.hpp's "innermost link (validation hook)" — which is why comments are stripped
// before anything is matched.

#include "doctest.h"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

// ── codeOnly — COPIED from test/mechanism_table_test.cpp :154-204 (SAFE-SAVE landmine 12: that
//    file is edited by chunks in flight; consolidating the copies into one test header waits until
//    they land). Every comment removed and every run of whitespace squeezed to one space; string and
//    character literals copied whole; a ' right after a letter, digit or _ is a digit separator.
[[nodiscard]] std::string codeOnly(const std::string& src) {
    std::string out;
    out.reserve(src.size());
    bool space = false;
    const auto flush = [&] {
        if (space && !out.empty()) out.push_back(' ');
        space = false;
    };
    std::size_t i = 0;
    while (i < src.size()) {
        const char c = src[i];
        const char next = i + 1 < src.size() ? src[i + 1] : '\0';
        if (c == '/' && next == '/') {
            while (i < src.size() && src[i] != '\n') ++i;
            space = true;
            continue;
        }
        if (c == '/' && next == '*') {
            const std::size_t end = src.find("*/", i + 2);
            i = end == std::string::npos ? src.size() : end + 2;
            space = true;
            continue;
        }
        const bool separator =
            c == '\'' && i > 0
            && (std::isalnum(static_cast<unsigned char>(src[i - 1])) != 0 || src[i - 1] == '_');
        if ((c == '"' || c == '\'') && !separator) {
            flush();
            out.push_back(c);
            ++i;
            while (i < src.size() && src[i] != c) {
                if (src[i] == '\\' && i + 1 < src.size()) out.push_back(src[i++]);
                out.push_back(src[i++]);
            }
            if (i < src.size()) out.push_back(src[i++]);
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            space = true;
            ++i;
            continue;
        }
        flush();
        out.push_back(c);
        ++i;
    }
    return out;
}
// ── end of the copy ─────────────────────────────────────────────────────────────────────────

[[nodiscard]] bool identChar(char c) {
    return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
}

/// Skip a string or character literal starting at `i` (which holds the quote); returns the index
/// just past it. A ' right after an identifier character is a digit separator, not a literal.
[[nodiscard]] std::size_t skipLiteral(const std::string& code, std::size_t i) {
    const char q = code[i];
    ++i;
    while (i < code.size() && code[i] != q) {
        if (code[i] == '\\') ++i;
        ++i;
    }
    return i < code.size() ? i + 1 : i;
}

struct Call {
    std::size_t at;       // index of the name
    std::size_t argsAt;   // index of the '('
};

/// Every CALL of `name` in `code` (already codeOnly): the whole identifier, not after '.' or '->'
/// (a member of some object is not the C library's function), followed by '(' — literals skipped.
[[nodiscard]] std::vector<Call> callsOf(const std::string& code, const std::string& name) {
    std::vector<Call> found;
    std::size_t i = 0;
    while (i < code.size()) {
        const char c = code[i];
        if ((c == '"' || c == '\'') && !(c == '\'' && i > 0 && identChar(code[i - 1]))) {
            i = skipLiteral(code, i);
            continue;
        }
        if (identChar(c) && (i == 0 || !identChar(code[i - 1]))) {
            std::size_t end = i;
            while (end < code.size() && identChar(code[end])) ++end;
            if (code.compare(i, end - i, name) == 0 && end - i == name.size()) {
                const bool member = (i >= 1 && code[i - 1] == '.')
                                    || (i >= 2 && code[i - 2] == '-' && code[i - 1] == '>')
                                    || (i >= 2 && code[i - 1] == ' ' && code[i - 2] == '.');
                std::size_t p = end;
                while (p < code.size() && code[p] == ' ') ++p;
                if (!member && p < code.size() && code[p] == '(') found.push_back({i, p});
            }
            i = end;
            continue;
        }
        ++i;
    }
    return found;
}

/// The last argument of the call whose '(' is at `open`, trimmed — or "" when unbalanced.
[[nodiscard]] std::string lastArgument(const std::string& code, std::size_t open) {
    int depth = 0;
    std::size_t lastComma = open;
    for (std::size_t i = open; i < code.size(); ++i) {
        const char c = code[i];
        if (c == '"' || (c == '\'' && !(i > 0 && identChar(code[i - 1])))) {
            i = skipLiteral(code, i) - 1;
            continue;
        }
        if (c == '(') ++depth;
        if (c == ')' && --depth == 0) {
            std::string arg = code.substr(lastComma + 1, i - lastComma - 1);
            while (!arg.empty() && arg.front() == ' ') arg.erase(arg.begin());
            while (!arg.empty() && arg.back() == ' ') arg.pop_back();
            return arg;
        }
        if (c == ',' && depth == 1) lastComma = i;
    }
    return "";
}

const char* const kBanned[] = {"rename", "remove", "unlink", "link", "stat", "mkdir"};

/// Opens that would go AROUND the two adapters' fopen: a REopen (it can make any FILE* read-write —
/// HA-166 (3)), a FILE* over a descriptor, and the POSIX creates. POSIX open() itself is not
/// matched by name — this scanner cannot tell a call from a member function DECLARED `open(`
/// (diag/sd_sink.hpp has one) — so it is caught by its header instead: <fcntl.h>, where O_RDWR is.
const char* const kBannedOpens[] = {"freopen", "fdopen", "openat", "creat"};

/// Headers whose only use in the library or src/ would be a file path around the adapters.
const char* const kBannedHeaders[] = {"<fcntl.h>", "<sys/fcntl.h>", "<fstream>", "<filesystem>"};

const char* const kFopenAllowed[] = {"include/shulib/hal/pros/block_sink.hpp",
                                     "include/shulib/hal/pros/block_source.hpp",
                                     "src/programs/bench_r3a.cpp"};

// The ONE place a mode may reach fopen() as a VARIABLE: chunk COMP-t's Append mode (on shulib-v2
// since 0f56301), where ProsBlockSink hands std::fopen to openForAppend() as a forwarding opener.
// Its exact text is pinned, in block_sink.hpp only, once. Then every mode handed to it must be a
// '+'-free literal, through calls of its parameter `open` — EXACTLY three of them ("rb", "ab",
// "wb"), so renaming that parameter (which leaves `open(` matching nothing) cannot quietly leave
// the check with nothing to look at. (Absent from this tree until that merge; the scanner
// self-test exercises it.)
const char* const kForwardingOpenerFile = "include/shulib/hal/pros/block_sink.hpp";
const char* const kForwardingOpener =
    "[](const char* p, const char* m) { return std::fopen(p, m); }";
constexpr std::size_t kForwardedModes = 3;

/// Whether `code` (already codeOnly) holds `#include <header>` outside a literal.
[[nodiscard]] bool includes(const std::string& code, const std::string& header) {
    std::size_t i = 0;
    while (i < code.size()) {
        const char c = code[i];
        if ((c == '"' || c == '\'') && !(c == '\'' && i > 0 && identChar(code[i - 1]))) {
            i = skipLiteral(code, i);
            continue;
        }
        if (code.compare(i, 8, "#include") == 0) {
            std::size_t h = i + 8;
            while (h < code.size() && code[h] == ' ') ++h;
            if (code.compare(h, header.size(), header) == 0) return true;
        }
        ++i;
    }
    return false;
}

/// Whether `relative`'s code holds the pinned forwarding opener (once merged with COMP-t).
[[nodiscard]] bool holdsForwardingOpener(const std::string& relative, const std::string& code) {
    return relative == kForwardingOpenerFile && code.find(kForwardingOpener) != std::string::npos;
}

/// A mode argument that is a string literal.
[[nodiscard]] bool literalMode(const std::string& mode) {
    return mode.size() >= 2 && mode.front() == '"' && mode.back() == '"';
}

/// What is wrong with one file's code, as lines ("" = nothing).
[[nodiscard]] std::string scan(const std::string& relative, const std::string& code) {
    std::string problems;
    for (const char* name : kBanned) {
        for (const Call& c : callsOf(code, name)) {
            problems += relative + ": calls " + name + "() at code offset " + std::to_string(c.at)
                        + " -- the brain has no rename/delete/stat/mkdir (HA-166 (2))\n";
        }
    }
    for (const char* name : kBannedOpens) {
        for (const Call& c : callsOf(code, name)) {
            problems += relative + ": calls " + name + "() at code offset " + std::to_string(c.at)
                        + " -- an open around the SD adapters (HA-166 (3))\n";
        }
    }
    for (const char* header : kBannedHeaders) {
        if (includes(code, header)) {
            problems += relative + ": includes " + header + " -- a file path around the SD adapters\n";
        }
    }
    const std::vector<Call> opens = callsOf(code, "fopen");
    bool allowed = false;
    for (const char* a : kFopenAllowed) allowed = allowed || relative == a;
    if (!opens.empty() && !allowed) {
        problems += relative + ": calls fopen() -- only the two SD adapters and Bench Tests' probe may\n";
    }
    int forwarded = 0;
    for (const Call& c : opens) {
        const std::string mode = lastArgument(code, c.argsAt);
        if (literalMode(mode)) {
            if (mode.find('+') != std::string::npos) {
                problems += relative + ": a read-write fopen() mode " + mode + " (HA-166 (3))\n";
            }
        } else if (!(mode == "m" && holdsForwardingOpener(relative, code) && ++forwarded == 1)) {
            problems += relative + ": an fopen() mode that is not a string literal: " + mode + "\n";
        }
    }
    if (forwarded == 1) {
        const std::vector<Call> handed = callsOf(code, "open");
        if (handed.size() != kForwardedModes) {
            problems += relative + ": " + std::to_string(handed.size())
                        + " modes handed to the forwarding opener through `open(`, not 3 -- renamed"
                          " parameter? the mode check would see nothing\n";
        }
        for (const Call& c : handed) {
            const std::string mode = lastArgument(code, c.argsAt);
            if (!literalMode(mode)) {
                problems += relative + ": a mode handed to the forwarding opener that is not a "
                            "string literal: " + mode + "\n";
            } else if (mode.find('+') != std::string::npos) {
                problems += relative + ": a read-write mode " + mode
                            + " handed to the forwarding opener (HA-166 (3))\n";
            }
        }
    }
    return problems;
}

[[nodiscard]] std::string readFile(const fs::path& p) {
    std::ifstream in{p};
    REQUIRE(in.good());
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

}  // namespace

TEST_CASE("SAFE-SAVE A scan: the scanner itself -- calls found, comments and literals and members "
          "ignored, fopen modes read") {
    // BUG CAUGHT: a scanner that matches a comment (the trailing "innermost link (validation hook)"
    // that a naive regex trips on), a string literal, a member function, or a longer identifier —
    // or that misses a qualified call. A scan that cannot fail, or that fails on prose, is noise.
    CHECK(callsOf(codeOnly("StatsHolder s_; // innermost link (validation hook)\n"), "link").empty());
    CHECK(callsOf(codeOnly("emitf(\"2. rename(a) : %s\", x);"), "rename").empty());
    CHECK(callsOf(codeOnly("tags.remove(3); p->unlink(); q . stat();"), "remove").empty());
    CHECK(callsOf(codeOnly("p->unlink();"), "unlink").empty());
    CHECK(callsOf(codeOnly("q . stat();"), "stat").empty());
    CHECK(callsOf(codeOnly("statistics(x); linker(y); std::remove_if(a, b, f);"), "stat").empty());
    CHECK(callsOf(codeOnly("statistics(x); linker(y); std::remove_if(a, b, f);"), "remove").empty());
    CHECK(callsOf(codeOnly("std::rename(a, b);"), "rename").size() == 1);
    CHECK(callsOf(codeOnly("::remove (path);"), "remove").size() == 1);
    CHECK(callsOf(codeOnly("x = 1'000; mkdir(p, 0);"), "mkdir").size() == 1);
    const std::string open = codeOnly("f = std::fopen(path_.c_str(), \"wb\");");
    REQUIRE(callsOf(open, "fopen").size() == 1);
    CHECK(lastArgument(open, callsOf(open, "fopen")[0].argsAt) == "\"wb\"");
    CHECK(scan("include/shulib/hal/pros/block_sink.hpp", open).empty());
    CHECK_FALSE(scan("include/shulib/hal/pros/file_store.hpp", open).empty());  // not allowed there
    CHECK_FALSE(scan("include/shulib/hal/pros/block_sink.hpp",
                     codeOnly("f = std::fopen(p, \"r+b\");"))
                    .empty());
    CHECK_FALSE(scan("include/shulib/hal/pros/block_sink.hpp", codeOnly("f = std::fopen(p, mode);"))
                    .empty());
    CHECK(scan("x.hpp", codeOnly("const char* s = \"/usd/\"; // rename(a, b)\n")).empty());
}

TEST_CASE("SAFE-SAVE A scan: EVERY banned call, open and header is pinned from the test's OWN list -- "
          "and a read-write freopen is refused") {
    // BUG CAUGHT: review mutants N09 (a name dropped from the scan's table: the tree has no such
    // call, so the tree scan alone stays green) and N10 (a std::freopen(..., "r+b", ...) in
    // ProsFileStore, which the first version of the scan did not look for). These lists are typed
    // here on purpose, NOT read from the scan's tables, so a table that loses an entry goes red.
    for (const char* name : {"rename", "remove", "unlink", "link", "stat", "mkdir", "freopen",
                             "fdopen", "openat", "creat"}) {
        CAPTURE(std::string{name});
        CHECK_FALSE(scan("x.hpp", codeOnly(std::string{"std::"} + name + "(a, b);")).empty());
        CHECK_FALSE(scan("x.hpp", codeOnly(std::string{name} + "(a, b);")).empty());
        CHECK(scan("x.hpp", codeOnly(std::string{"obj."} + name + "(a, b);")).empty());  // a member
    }
    for (const char* header : {"<fcntl.h>", "<sys/fcntl.h>", "<fstream>", "<filesystem>"}) {
        CAPTURE(std::string{header});
        CHECK_FALSE(scan("x.hpp", codeOnly(std::string{"#include "} + header + "\n")).empty());
        CHECK_FALSE(scan("x.hpp", codeOnly(std::string{"#include"} + header + "\n")).empty());
        CHECK(scan("x.hpp", codeOnly(std::string{"const char* s = \"#include "} + header + "\";"))
                  .empty());  // inside a literal
        CHECK(scan("x.hpp", codeOnly(std::string{"// #include "} + header + "\n")).empty());
    }
    CHECK_FALSE(scan("include/shulib/hal/pros/file_store.hpp",
                     codeOnly("std::FILE* f = std::freopen(\"/x\", \"r+b\", stdin);"))
                    .empty());
    CHECK(scan("x.hpp", codeOnly("#include <cstdio>\n#include \"shulib/hal/file_store.hpp\"\n")).empty());
}

TEST_CASE("SAFE-SAVE A scan: COMP-t's forwarding opener -- allowed only in its pinned form in "
          "block_sink.hpp, and every mode handed to it a '+'-free literal in exactly three calls") {
    // BUG CAUGHT: a second non-literal fopen beside the pinned one, the opener moved to another
    // file, a '+' (or a variable) handed to it, and — the review of 2026-10-01 — the opener's
    // parameter RENAMED, which used to leave the mode check looking at no calls at all. The text
    // below is block_sink.hpp's on shulib-v2 (0f56301), reduced to what the scan reads.
    const std::string opener = codeOnly(
        "template <class Open, class Close>\n"
        "[[nodiscard]] std::FILE* openForAppend(const char* path, Open&& open, Close&& close,\n"
        "                                       AppendOpen& how) {\n"
        "  if (std::FILE* probe = open(path, \"rb\"); probe != nullptr) { close(probe); }\n"
        "  if (std::FILE* f = open(path, \"ab\"); f != nullptr) { return f; }\n"
        "  if (std::FILE* f = open(path, \"wb\"); f != nullptr) { return f; }\n"
        "  return nullptr; }\n"
        "file_ = openForAppend(\n"
        "    path_.c_str(), [](const char* p, const char* m) { return std::fopen(p, m); },\n"
        "    [](std::FILE* f) { std::fclose(f); }, append_);\n"
        "file_ = std::fopen(path_.c_str(), \"wb\");\n");
    const std::string sink = "include/shulib/hal/pros/block_sink.hpp";
    CHECK(scan(sink, opener).empty());
    CHECK(holdsForwardingOpener(sink, opener));
    CHECK_FALSE(scan("include/shulib/hal/pros/block_source.hpp", opener).empty());  // only there
    const auto with = [&](const std::string& from, const std::string& to) {
        std::string changed = opener;
        const std::size_t at = changed.find(from);
        REQUIRE(at != std::string::npos);
        changed.replace(at, from.size(), to);
        return changed;
    };
    CHECK_FALSE(scan(sink, with("\"ab\"", "\"a+b\"")).empty());     // '+' handed to it
    CHECK_FALSE(scan(sink, with("\"rb\"", "mode")).empty());          // a variable handed to it
    CHECK_FALSE(scan(sink, with("std::fopen(path_.c_str(), \"wb\")",
                                "std::fopen(path_.c_str(), \"w+b\")"))
                    .empty());  // the plain open's '+'
    CHECK_FALSE(scan(sink, opener + codeOnly("g = std::fopen(q, m);")).empty());  // a 2nd variable
    // The parameter RENAMED: `open(` now matches none of the three calls, so the mode check would
    // be vacuous — the count refuses the shape instead (calls renamed alone, and with the
    // declaration).
    std::string callsRenamed = opener;
    for (std::size_t at = callsRenamed.find(" open(path"); at != std::string::npos;
         at = callsRenamed.find(" open(path", at + 1)) {
        callsRenamed.replace(at, 5, " opener");
    }
    REQUIRE(callsOf(callsRenamed, "open").empty());
    CHECK_FALSE(scan(sink, callsRenamed).empty());
    std::string renamed = callsRenamed;
    const std::size_t declared = renamed.find("Open&& open,");
    REQUIRE(declared != std::string::npos);
    renamed.replace(declared, 12, "Open&& opener,");
    CHECK_FALSE(scan(sink, renamed).empty());
    // A FOURTH mode handed to it (or one fewer) is not the pinned shape either.
    CHECK_FALSE(scan(sink, with("return nullptr; }", "open(path, \"rb\"); return nullptr; }")).empty());
}

TEST_CASE("SAFE-SAVE A scan: no rename, remove, unlink, link, stat or mkdir anywhere in the library or "
          "src/; fopen only in the two adapters and Bench Tests' probe; no read-write mode") {
    // BUG CAUGHT: mutation 13 (a std::rename inserted in ProsFileStore) and 12(b) (an fopen "r+b" in
    // ProsFileStore — caught twice: fopen outside the adapters, and a '+' mode). Also anyone who
    // "simplifies" the safe save back to write-temp-then-rename, which the host would pass.
    const fs::path root{SHULIB_SOURCE_DIR};
    std::vector<fs::path> files;
    for (const auto& e : fs::recursive_directory_iterator(root / "include/shulib")) {
        if (e.is_regular_file() && e.path().extension() == ".hpp") files.push_back(e.path());
    }
    for (const auto& e : fs::recursive_directory_iterator(root / "src")) {
        const auto ext = e.path().extension();
        if (e.is_regular_file() && (ext == ".hpp" || ext == ".cpp")) files.push_back(e.path());
    }
    REQUIRE(files.size() > 150);
    std::string problems;
    std::size_t fopenSites = 0;
    std::size_t forwardingOpeners = 0;
    for (const fs::path& f : files) {
        const std::string relative = fs::relative(f, root).generic_string();
        const std::string code = codeOnly(readFile(f));
        problems += scan(relative, code);
        fopenSites += callsOf(code, "fopen").size();
        forwardingOpeners += holdsForwardingOpener(relative, code) ? 1U : 0U;
    }
    CHECK_MESSAGE(problems.empty(), problems);
    // "wb" in the sink, "rb" in the source, "wb" in Bench Tests' probe — plus, once COMP-t's Append
    // mode is in the tree, its one forwarding opener in the sink.
    CHECK(fopenSites == 3 + forwardingOpeners);
}

TEST_CASE("SAFE-SAVE A scan: ProsFileStore spells no \"/usd/\" and calls no fopen of its own") {
    // BUG CAUGHT: a default constructor that passes a root of its own — a second spelling of the
    // prefix, which the default-path test alone cannot tell from the adapters' default.
    const std::string code =
        codeOnly(readFile(fs::path{SHULIB_SOURCE_DIR} / "include/shulib/hal/pros/file_store.hpp"));
    CHECK(code.find("\"/usd/\"") == std::string::npos);
    CHECK(code.find("usd/") == std::string::npos);
    CHECK(callsOf(code, "fopen").empty());
    CHECK(code.find("ProsBlockSource") != std::string::npos);
    CHECK(code.find("ProsBlockSink") != std::string::npos);
}
