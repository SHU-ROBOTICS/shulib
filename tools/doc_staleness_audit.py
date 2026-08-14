#!/usr/bin/env python3
"""Doc staleness audit — the rot the other four gates structurally cannot see.

WHY THIS EXISTS
---------------
`api_doc_tool.py`'s four gates are solid and they have NO OPINION about whether a
sentence is still TRUE. Coverage checks that a member is documented, not that the
documentation is right. Freshness checks that generated output matches the headers.
Verbatim checks that an example still compiles. Removability checks link hygiene.
A chapter can claim the wrong number and pass all four.

Every staleness incident this project has recorded lives in exactly that blind
spot, and every one was found by a person reading rather than by a machine:

  * a guide chapter claiming 57 hardware assumptions when there were 67
  * README claiming 659 tests twenty lines from a claim of 867
  * an API chapter describing an exception that had been deleted
  * a chapter claiming no cookbook existed after D3 shipped one
  * (R1a) README's suite counts going stale TWICE IN ONE DAY — once when the
    chunk added tests, once when the reviewer of that chunk added more

Three of those classes are mechanizable. This does them, so reading time is spent
on conceptual drift — the part no tool can reach — instead of on arithmetic.

  A. NUMERIC CLAIMS     a doc says "N tests" / "N assumptions" / "N headers";
                        the repo is asked what N actually is.
  B. SELF-DISAGREEMENT  the same KIND of claim with two different values anywhere
                        in the live doc set. Fires even where the truth cannot be
                        derived — this is the 659-vs-867 case.
  C. DEAD PATHS         a doc names a file path that does not exist.
  D. DEAD SYMBOLS       a doc names `someIdentifier()` that appears in no header.
                        REPORT-ONLY: high false-positive rate by nature.

WHAT THIS DELIBERATELY DOES NOT DO, and why the scope is narrow
---------------------------------------------------------------
It does not scan `roadmap.md`, `hardware-assumptions.md`, the changelog, or the
per-chunk records. Those are LEDGERS: "chunk E4: suite 915 cases / 1,521,419
assertions" is a correctly frozen historical record, not a lie, and a gate that
calls it stale is a gate somebody switches off within a week.

The first draft of this tool scanned everything and produced 30+ findings of which
exactly ONE was real. That ratio is the failure mode, not a tuning problem. If you
extend this, extend the SCOPE reluctantly and the FILTERS eagerly.

  python3 tools/doc_staleness_audit.py            # audit; exit 1 on A/B/C findings
  python3 tools/doc_staleness_audit.py self-test  # prove the detectors can fire
"""

import glob as _glob
import os
import re
import subprocess
import sys
from collections import defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Documents that describe the PRESENT. Only these carry numeric claims worth
# checking — see the scope note in the module docstring.
LIVE_GLOBS = ["docs/guide/*.md", "docs/cookbook/*.md"]
LIVE_FILES = ["README.md", "docs/README.md", "docs/faq.md", "test/README.md"]

# The dead-path sweep has the same history problem — a chunk record naming
# `src/legacy/shulib/chassis/odometry.cpp` correctly records a file deleted at C7 —
# so it covers public docs plus the three planning documents, never the records.
PATH_GLOBS = ["docs/*.md", "docs/guide/*.md", "docs/cookbook/*.md", "docs/api/*.md"]
PATH_FILES = ["README.md", "test/README.md", "docs/internal/RESUMING.md",
              "docs/internal/build-order.md", "docs/internal/PROJECT-BRIEFING.md"]

NUM = r"([0-9][0-9,]*)"
CLAIMS = [
    (re.compile(NUM + r"\s*(?:test\s+)?cases\b", re.I), "cases"),
    (re.compile(NUM + r"\s+tests\b", re.I), "cases"),
    (re.compile(NUM + r"\s+assertions\b", re.I), "assertions"),
    (re.compile(NUM + r"\s+(?:hardware\s+)?assumptions\b", re.I), "assumptions"),
    (re.compile(NUM + r"\s+(?:falsifiable\s+)?entries\b", re.I), "assumptions"),
    (re.compile(NUM + r"\s+(?:v2\s+|public\s+)?headers\b", re.I), "headers"),
]

# A line quoting history is a record, not a lie.
HISTORICAL = re.compile(
    r"\b(was|were|used to|previously|at the time|"
    r"before |until |grew from|up from|historic|"
    r"verified 20|closed 20|measured 20)\b|→|->", re.I)

# A claim SCOPED to one file or directory — "test/pid_test.cpp (9 cases)" — is
# about that thing, not the suite. Biggest false-positive source in the first draft.
SCOPED = re.compile(r"(?:test|include|src|tools|docs)/[A-Za-z0-9_./-]+")

# A claim tagged with a chunk label is a per-chunk evidence citation.
CHUNK_TAG = re.compile(r"\b(?:chunk\s+)?(?:post-)?[ACDEFGHIRT]\d+[a-z]?\b")

PATH_PAT = re.compile(
    r"`((?:docs|include|src|test|tools)/[A-Za-z0-9_./-]+"
    r"\.(?:hpp|cpp|h|c|md|py|sh|yml|csv|json|txt))`")

# A document whose SUBJECT is deleted code legitimately names files that no longer
# exist. It opts out by saying so IN ITSELF, with a reason, rather than being
# quietly listed inside this tool: D3's lesson is that a gate's exclusion list is
# where its holes live, so this exclusion lives where a READER of the document will
# see it. Format:
#     <!-- staleness-audit: historical-paths — <reason> -->
HISTORICAL_DOC = re.compile(r"<!--\s*staleness-audit:\s*historical-paths\b")

# A path a doc tells the reader to CREATE, or records as DELETED, is correctly
# absent. Three of this check's first four findings were these.
ABSENT_ON_PURPOSE = re.compile(
    r"\b(creat|writ|add|make|name it|delete|deleted|remove|removed|drop|dropped|"
    r"quarantin|move[d]? to|renamed)\w*\b", re.I)

SYM_PAT = re.compile(r"`([a-z][A-Za-z0-9_]{3,})\(\)`")


def _docs(globs, files):
    out = []
    for g in globs:
        out.extend(_glob.glob(os.path.join(REPO, g)))
    for f in files:
        p = os.path.join(REPO, f)
        if os.path.exists(p):
            out.append(p)
    return sorted(os.path.relpath(p, REPO) for p in out)


def truth():
    """What the repo actually says. Missing entries simply are not checked."""
    t = {}
    n = 0
    for root, _d, files in os.walk(os.path.join(REPO, "include/shulib")):
        n += sum(1 for f in files if f.endswith(".hpp"))
    t["headers"] = n

    ha = os.path.join(REPO, "docs/hardware-assumptions.md")
    if os.path.exists(ha):
        ids = {int(m) for m in re.findall(r"^\|\s*HA-(\d+)", open(ha, encoding="utf-8").read(), re.M)}
        if ids:
            t["assumptions"] = len(ids)

    # Suite counts come from the built binary — but this audit runs as a BUILD
    # gate, i.e. before that binary is relinked. Reading a stale one would let the
    # audit check a document against yesterday's numbers and call it clean, which
    # is the same stale-binary trap that made C4's mutation nearly read as green
    # (and that bit R1a's review once already, via a cp -a that preserved mtime).
    # So: if anything under test/ or include/ is newer than the binary, the counts
    # are NOT DERIVED AT ALL rather than derived wrongly. Absent keys are simply
    # not checked, and the caller says so out loud.
    binpath = os.path.join(REPO, "build/test/shulib_tests")
    if os.path.exists(binpath):
        btime = os.path.getmtime(binpath)
        newest = 0.0
        for base in ("test", "include"):
            for root, _d, files in os.walk(os.path.join(REPO, base)):
                for f in files:
                    if f.endswith((".cpp", ".hpp", ".h")):
                        newest = max(newest, os.path.getmtime(os.path.join(root, f)))
        if newest <= btime:
            try:
                run = subprocess.run([binpath], capture_output=True, text=True,
                                     timeout=1800).stdout
                m = re.search(r"test cases:\s*(\d+)", run)
                if m:
                    t["cases"] = int(m.group(1))
                m = re.search(r"assertions:\s*(\d+)", run)
                if m:
                    t["assertions"] = int(m.group(1))
            except Exception:
                pass
    return t


def scan_claims(texts):
    """texts: {relpath: content} -> [(rel, line, key, value, context)]"""
    found = []
    for rel, text in texts.items():
        in_fence = False
        for i, line in enumerate(text.splitlines(), 1):
            if line.lstrip().startswith("```"):
                in_fence = not in_fence
                continue
            if (in_fence or HISTORICAL.search(line)
                    or SCOPED.search(line) or CHUNK_TAG.search(line)):
                continue
            for pat, key in CLAIMS:
                for m in pat.finditer(line):
                    found.append((rel, i, key, int(m.group(1).replace(",", "")),
                                  line.strip()[:110]))
    return found


def scan_paths(texts, exists):
    dead = []
    for rel, text in texts.items():
        if HISTORICAL_DOC.search(text):
            continue
        for i, line in enumerate(text.splitlines(), 1):
            if ABSENT_ON_PURPOSE.search(line):
                continue
            for m in PATH_PAT.finditer(line):
                if not exists(m.group(1)):
                    dead.append(f"{rel}:{i}  names `{m.group(1)}` — no such file")
    return dead


def contradicted(claims, t):
    out = []
    for rel, line, key, val, ctx in claims:
        if key in t and val != t[key]:
            out.append(f"{rel}:{line}  claims {key}={val:,} but the repo says {t[key]:,}\n"
                       f"        | {ctx}")
    return out


def disagreements(claims):
    bykey, where = defaultdict(set), defaultdict(list)
    for rel, line, key, val, _ctx in claims:
        bykey[key].add(val)
        where[key].append(f"{rel}:{line}={val:,}")
    return [f"{k}: {len(v)} different values in the live doc set — "
            + "; ".join(sorted(set(where[k])))
            for k, v in sorted(bykey.items()) if len(v) > 1]


def do_self_test():
    """Prove each detector CAN fire. A gate nobody has seen go red is a rumour."""
    failures = []
    checks = 0

    def expect(condition, message):
        nonlocal checks
        checks += 1
        if not condition:
            failures.append(message)

    c = scan_claims({"f.md": "The suite has 42 test cases today."})
    expect(contradicted(c, {"cases": 99}), "A: a wrong numeric claim was not caught")
    expect(not contradicted(scan_claims({"f.md": "It grew from 42 test cases."}), {"cases": 99}),
           "A: a HISTORICAL claim was wrongly flagged")
    expect(not contradicted(scan_claims({"f.md": "`test/pid_test.cpp` (42 cases)"}), {"cases": 99}),
           "A: a FILE-SCOPED claim was wrongly flagged")
    expect(not contradicted(scan_claims({"f.md": "Chunk E4: 42 test cases."}), {"cases": 99}),
           "A: a CHUNK-TAGGED claim was wrongly flagged")
    expect(disagreements(scan_claims({"a.md": "10 test cases.", "b.md": "20 test cases."})),
           "B: two docs disagreeing was not caught")
    expect(not disagreements(scan_claims({"a.md": "10 test cases.", "b.md": "10 test cases."})),
           "B: two docs AGREEING was wrongly flagged")
    expect(scan_paths({"f.md": "see `docs/guide/99-nope.md` for more"}, lambda p: False),
           "C: a dead path was not caught")
    expect(not scan_paths({"f.md": "create `test/new_test.cpp` yourself"}, lambda p: False),
           "C: a to-be-CREATED path was wrongly flagged")
    expect(scan_paths({"f.md": "`src/gone.h` and `src/gone.c`"}, lambda p: False),
           "C: .h/.c extensions are not covered")
    expect(not scan_paths({"f.md": "<!-- staleness-audit: historical-paths — why -->\n`src/x.c`"},
                          lambda p: False),
           "C: a doc declaring itself historical was still flagged")

    for f in failures:
        print(f"  SELF-TEST FAILURE — {f}", file=sys.stderr)
    if failures:
        return 1
    print(f"doc staleness audit self-test: OK ({checks} detector/filter cases)")
    return 0


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "self-test":
        return do_self_test()

    t = truth()
    live = {r: open(os.path.join(REPO, r), encoding="utf-8", errors="ignore").read()
            for r in _docs(LIVE_GLOBS, LIVE_FILES)}
    pathdocs = {r: open(os.path.join(REPO, r), encoding="utf-8", errors="ignore").read()
                for r in _docs(PATH_GLOBS, PATH_FILES)}

    claims = scan_claims(live)
    wrong = contradicted(claims, t)
    disagree = disagreements(claims)
    dead = scan_paths(pathdocs, lambda p: os.path.exists(os.path.join(REPO, p)))

    hdr = ""
    for root, _d, files in os.walk(os.path.join(REPO, "include/shulib")):
        for f in files:
            if f.endswith(".hpp"):
                hdr += open(os.path.join(root, f), encoding="utf-8", errors="ignore").read()
    ghosts = [f"{rel}:{i}  names `{m.group(1)}()` — in no header"
              for rel, text in live.items()
              for i, line in enumerate(text.splitlines(), 1)
              for m in SYM_PAT.finditer(line) if m.group(1) not in hdr]

    findings = len(wrong) + len(disagree) + len(dead)
    if findings:
        print("", file=sys.stderr)
        print("DOC STALENESS — a document asserts something the repo contradicts.",
              file=sys.stderr)
        print("The other four doc gates cannot see this: they check that prose EXISTS",
              file=sys.stderr)
        print("and is well-formed, never that it is still TRUE.", file=sys.stderr)
        for title, rows in (("CLAIMS CONTRADICTED BY THE REPO", wrong),
                            ("THE DOC SET DISAGREEING WITH ITSELF", disagree),
                            ("DEAD FILE PATHS", dead)):
            if rows:
                print(f"\n  {title}", file=sys.stderr)
                for r in rows:
                    print(f"    {r}", file=sys.stderr)
        print("\nFix the prose, or if the number must live in a document at all, ask",
              file=sys.stderr)
        print("whether it belongs in a COMMAND instead — a figure kept true by hand is",
              file=sys.stderr)
        print("wrong the moment anyone does the work it describes.\n", file=sys.stderr)
        return 1

    note = ""
    unchecked = [k for k in ("cases", "assertions") if k not in t]
    if unchecked:
        note += (f" [{'/'.join(unchecked)} NOT derived — the test binary is older than "
                 f"a source file, so it was not trusted]")
    if not claims:
        # Said out loud rather than passing silently: check-examples has the same
        # guard for the same reason. Here, though, zero is the GOAL — a figure that
        # must be maintained by hand belongs in a command — so this is a note, not
        # a failure. The self-test is what keeps the detectors honest when the
        # live docs give them nothing to chew on.
        note = " — no live doc states a checkable figure, which is the intended end state"
    if ghosts:
        note += f"; {len(ghosts)} report-only symbol notes suppressed"
    print(f"doc staleness audit: clean ({len(live)} live docs, "
          f"{len(claims)} numeric claims checked){note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
