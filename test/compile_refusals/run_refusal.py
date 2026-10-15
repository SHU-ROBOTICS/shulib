#!/usr/bin/env python3
"""Run ONE compile-refusal TU and decide whether the build refused it FOR ITS STATED REASON.

The mechanism core's front door refuses mistakes AT BUILD (a consteval check calls a function whose
NAME is the reason; a deleted overload names the fix in its template parameter). A refusal no test
can see is a refusal a mutation can delete, so every one a robot file can reach has a TU here (the
MECH brief's §3.0; the two unreachable "used twice" fallbacks in mechanism/wiring.hpp have none), and
this script is how CTest judges it:

    run_refusal.py --build-dir <build/test> --target <the TU's object target> --source <the TU>

PASS (exit 0) only when ALL of these hold:
  * the build of the target FAILS (a TU that compiles prints no reason, and is a red test);
  * every `// expect: <text>` line of the TU appears, as a FIXED string, in the MESSAGE part of a
    GCC diagnostic — the text after ": error: " or ": note: " on a line. Never in a path: the TU, its
    object target and make's own "*** [...] Error 1" line all carry the file's name, and a file named
    after its reason would otherwise "pass" on any error at all (the harness's own self-test,
    harness_target_is_outside_the_travel_limits.cpp — refused for a misspelt preset, its stated
    reason only in its file name — is that trap, and must FAIL);
  * every `// expect-at: <line>` names a line of the TU itself that GCC cites (an error or a note
    located at "<the TU>:<line>:") — how the location of a refusal is pinned, not only its words.

The expected text lives IN the TU, beside the code it judges (one file per refusal, no CMake
escaping of quotes). Read the TU's own comment for why it must be refused.
"""
import argparse
import os
import re
import subprocess
import sys

MESSAGE = re.compile(r":\s(?:error|note):\s(.*)$")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--target", required=True)
    ap.add_argument("--source", required=True)
    a = ap.parse_args()

    with open(a.source, encoding="utf-8") as f:
        text = f.read()
    expects = [m.group(1).rstrip() for m in re.finditer(r"^// expect: (.+)$", text, re.M)]
    ats = [int(m.group(1)) for m in re.finditer(r"^// expect-at: (\d+)\s*$", text, re.M)]
    if not expects:
        print(f"{a.source}: no `// expect: <text>` line — a refusal with no stated reason proves nothing")
        return 2

    # GCC quotes names with U+2018/U+2019 in a UTF-8 locale and with ' in C: no expected text
    # includes GCC's quotes, and the output is decoded as UTF-8 either way.
    proc = subprocess.run(["cmake", "--build", a.build_dir, "--target", a.target],
                          capture_output=True, text=True, encoding="utf-8", errors="replace")
    out = proc.stdout + proc.stderr
    lines = out.splitlines()

    if proc.returncode == 0:
        print(f"COMPILED: {a.source} was NOT refused; it must be refused naming: {expects}")
        return 1

    messages = []
    for ln in lines:
        m = MESSAGE.search(ln)
        if m:
            messages.append(m.group(1))
    missing = [e for e in expects if not any(e in msg for msg in messages)]

    tu = os.path.basename(a.source)
    cited = set()
    for ln in lines:
        m = re.match(r"^(.*?):(\d+):\d+:\s", ln)
        if m and os.path.basename(m.group(1)) == tu:
            cited.add(int(m.group(2)))
    missingAt = [n for n in ats if n not in cited]

    if missing or missingAt:
        print(f"REFUSED FOR THE WRONG REASON: {a.source}")
        for e in missing:
            print(f"  expected in a diagnostic message: {e!r}")
        for n in missingAt:
            print(f"  expected GCC to cite {tu}:{n} (it cited lines {sorted(cited)})")
        print("  the diagnostics, first 12:")
        for msg in [ln for ln in lines if MESSAGE.search(ln)][:12]:
            print("   ", msg[:300])
        return 1
    print(f"refused as expected: {tu} -> {expects}" + (f" at line(s) {ats}" if ats else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
