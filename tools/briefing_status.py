#!/usr/bin/env python3
"""Generate (and gate) the STATUS block of docs/internal/PROJECT-BRIEFING.md.

WHY THIS EXISTS
---------------
A handoff document that restates facts which live elsewhere is a document that
goes stale, and a stale briefing is worse than none: a new session trusts it and
starts from a false picture. D3 learned the general form of this lesson —
"generation alone does not solve staleness" — and the answer that worked was
generation PLUS a gate that fails the build when the generated thing drifts.

So PROJECT-BRIEFING.md is split:

  * DURABLE narrative, hand-written — who the team is, the thesis, the governing
    constraint, the architecture, the chunk loop, the standards, the traps. These
    change once a phase, and a human should be the one changing them.
  * A GENERATED STATUS BLOCK, between the markers below, derived from the repo
    itself. Nothing in it is typed by hand, so nothing in it can rot silently.

Every number here is DERIVED, and each carries its source in the output so a
reader can re-check it without trusting this script.

USAGE
  python3 tools/briefing_status.py generate   # rewrite the block in place
  python3 tools/briefing_status.py check      # exit 1 if the block is stale
  python3 tools/briefing_status.py show       # print the block, touch nothing
  python3 tools/briefing_status.py self-test  # prove the chunk/part rule and the bounded suite run

`check` runs as a build-time doc gate beside check-coverage / check-fresh /
check-examples / check-removability.

THE SUITE LINE comes from tools/suite_run.py, the bounded run every tool uses (4 GiB of
memory on Linux, 256 MiB of output, 600 s of CPU, 300 s; killed on a timeout, and on Linux it dies
with the tool — tools/suite_run.py has the details). A binary that
is present but BROKEN is never written into the briefing (`generate` refuses and prints the
recovery order) and never silently mistaken for "not built" (`check` exempts its suite line
because the build runs it before relinking the binary, and SAYS so).

DELIBERATELY NOT IN THE GENERATED BLOCK: anything that changes without the repo
changing — unpushed-commit counts (depends on whether a remote ref exists),
wall-clock dates, or anything read from a network. A gate that fails for reasons
unrelated to the work is a gate people learn to ignore.
"""

import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRIEFING = os.path.join(REPO, "docs/internal/PROJECT-BRIEFING.md")

BEGIN = "<!-- BEGIN GENERATED STATUS — regenerate with: python3 tools/briefing_status.py generate -->"
END = "<!-- END GENERATED STATUS -->"


def _run(args, cwd=REPO):
    try:
        return subprocess.run(args, cwd=cwd, capture_output=True, text=True,
                              timeout=600).stdout
    except Exception:
        return ""


# A PART of a chunk is named `<CHUNK>-<part>`, the part holding at least one lower-case letter —
# `COMP-t` is COMP's auton-tester part (COMP = parts a, t and b); `T4-2a` would be T4's. A part may
# keep its own live log and its own record (`COMP-t-PROGRESS.md`, `COMP-t-COMPLETED.md`), but the
# plan counts CHUNKS: a part's record is not a completed chunk, and a part's log keeps its PARENT open
# until the parent's own record exists. (Found 2026-09-29 at COMP-t's merge: its record made this
# block count 33 of 63 while the phase table has COMP as one chunk.) Chunk names never match: they
# are upper case, digits and hyphens (`SAFE-SAVE`, `DEFECTS2`), and where one ends in a lower-case
# letter it follows a digit with no hyphen (`R1a`, `R2a`, `R5a` — chunks the phase table names).
# `self-test` pins both sides of the rule.
PART_NAME = re.compile(r"^([A-Z][A-Z0-9]*(?:-[A-Z0-9]+)*)-([a-z0-9]*[a-z][a-z0-9]*)$")


def chunk_of(name):
    """The chunk a record or log belongs to: a PART's (PART_NAME) is its parent's."""
    m = PART_NAME.match(name)
    return m.group(1) if m else name


def completed_chunks(d=None):
    """Chunks with a completion record — the project's own definition of done. A PART's record
    (PART_NAME) is not a chunk and is left out. `d` is the records' directory (self-test only)."""
    d = d or os.path.join(REPO, "docs/internal/chunks")
    if not os.path.isdir(d):
        return []
    names = [f[: -len("-COMPLETED.md")] for f in os.listdir(d)
             if f.endswith("-COMPLETED.md")]
    names = [n for n in names if not PART_NAME.match(n)]

    def key(n):
        # The suffix is part of the key, and that is load-bearing — DOCS1.
        #
        # This used to be (prefix, number) only, which is not a TOTAL order: R1a
        # and R1b both key to ("R", 1). Python's sort is stable, so ties keep
        # their input order — and the input is os.listdir(), which is
        # filesystem-dependent. The block therefore generated "R1a · R1b" on one
        # machine and "R1b · R1a" on another, and the gate failed in CI while
        # passing locally on the identical commit. Two pushes were spent on that.
        #
        # The bug was unreachable until R1a/R1b became the first two chunks to
        # share a prefix and a number, which is why it shipped quietly at R1a and
        # only fired the first time CI ever ran this gate.
        m = re.match(r"([A-Z]+)(\d+)(.*)$", n)
        return (m.group(1), int(m.group(2)), m.group(3)) if m else (n, 0, "")

    return sorted(names, key=key)


def in_flight_chunks(d=None):
    """A PROGRESS log with no COMPLETED record = a chunk that is OPEN.

    Open means one of two things, and this function cannot tell which (the tree's
    cleanliness is a property of the commit being made, which this block must not
    depend on — see the "DO NOT ADD THEM BACK" note): interrupted mid-chunk (then `git
    status` is dirty and RESUMING says to read the log first), or landed in parts
    with work remaining (G1 Part 1 before Part 2, R3d before its bench run). It was
    labelled INTERRUPTED until 2026-09-24, which was false for every chunk in the
    second case and told a new session to treat parked work as a crash.
    """
    d = d or os.path.join(REPO, "docs/internal/chunks")
    if not os.path.isdir(d):
        return []
    done = set(completed_chunks(d))
    logs = {f[: -len("-PROGRESS.md")] for f in os.listdir(d) if f.endswith("-PROGRESS.md")}
    # A part's log (PART_NAME) keeps its PARENT open; the part itself is never listed.
    return sorted({chunk_of(n) for n in logs} - done)


def header_count():
    n = 0
    for root, _dirs, files in os.walk(os.path.join(REPO, "include/shulib")):
        n += sum(1 for f in files if f.endswith(".hpp"))
    return n


# The register's ONE "Held numbers" line: the HA numbers briefs have reserved and not yet registered
# (never reissued). The next free number is above every registered AND every held one — without this,
# the block named a brief's reserved number as free (found 2026-09-30: "next free is HA-181" while
# G3a's amended brief held HA-181 and the mechanism core's HA-182..HA-186). `self-test` pins it.
HELD_LINE = re.compile(r"^> \*\*Held numbers\*\*[^:\n]*:([^\n]*)$", re.M)


def held_numbers(text):
    """The numbers the register's Held line lists (`HA-n` or `HA-a..HA-b`), as a set."""
    m = HELD_LINE.search(text)
    held = set()
    if m:
        for a, b in re.findall(r"HA-(\d+)(?:\.\.HA-(\d+))?", m.group(1)):
            held.update(range(int(a), int(b or a) + 1))
    return held


def next_free(ids, held):
    """The first number above every registered and every held one."""
    return max(set(ids) | held, default=0) + 1


def ha_entries(text=None):
    """Registered hardware assumptions, and the next free number (held numbers skipped)."""
    if text is None:
        path = os.path.join(REPO, "docs/hardware-assumptions.md")
        if not os.path.exists(path):
            return 0, 1, 0
        text = open(path, encoding="utf-8").read()
    ids = [int(m) for m in re.findall(r"^\|\s*HA-(\d+)", text, re.M)]
    settled = len(re.findall(r"^\|\s*HA-\d+.*\bsettled\b", text, re.M | re.I))
    return (len(ids), next_free(ids, held_numbers(text)), settled)


# ── THE SUITE RUN IS BOUNDED (tools/suite_run.py, 2026-10-22) ─────────────────────────────
# The suite line comes from tools/suite_run.py — the one bounded way every tool runs the test binary
# (its header says why: on 2026-10-15 a mutant left linked by a killed mutation run reached 54 GB
# through this tool's own unbounded `generate` and took the desktop down). A binary that is PRESENT
# but BROKEN is never "not built" (which `check` exempts silently) and never a count to commit:
# `generate` refuses it and prints the recovery; `check` exempts its suite line and SAYS so, because
# the build runs this gate before it relinks the binary (RELEASING.md §3) — failing would block the
# very rebuild that replaces it. suite_rule() below is that decision, pinned by the self-test.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # also under python -P / -I
import suite_run  # noqa: E402

_SUITE_MEMO = {}  # render() and main() both ask: the suite runs once per invocation


def suite_counts(binary=None):
    """The authoritative test numbers, from the built binary (tools/suite_run.py): None if it is not
    built; a dict holding "broken" (why) if it is present but BROKEN."""
    path = os.path.abspath(binary or suite_run.BINARY)
    if path not in _SUITE_MEMO:
        _SUITE_MEMO[path] = suite_run.counts(path)
    return _SUITE_MEMO[path]


def suite_rule(cmd, suite):
    """What `cmd` does with this suite verdict (the header above): "refuse" — generate never writes a
    BROKEN binary; "exempt" — check does not compare the suite line when there is no binary or a
    BROKEN one (and says which); "compare" — everything else, the suite line included."""
    broken = bool(suite and "broken" in suite)
    if cmd == "generate" and broken:
        return "refuse"
    if cmd == "check" and (suite is None or broken):
        return "exempt"
    return "compare"


def freeze_rows():
    """The Freeze Register, parsed from the roadmap — its owning document."""
    path = os.path.join(REPO, "docs/roadmap.md")
    if not os.path.exists(path):
        return []
    rows = []
    for line in open(path, encoding="utf-8"):
        m = re.match(r"^\|\s*(F\d+)\s*\|\s*(.+?)\s*\|", line)
        if not m:
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        status = cells[-1] if cells else ""
        subject = re.sub(r"[*`]", "", m.group(2))
        subject = re.sub(r"\s+", " ", subject)
        if len(subject) > 58:
            subject = subject[:55].rstrip() + "…"
        if "LOCKED" in status:
            short = "LOCKED"
        elif "NOT FROZEN" in status.upper() or "open by design" in status:
            short = "open by design"
        else:
            short = "pending"
        rows.append((m.group(1), subject, short))
    return rows


def next_chunk():
    """The 'Next:' pointer in build-order.md — that file owns this fact."""
    path = os.path.join(REPO, "docs/internal/build-order.md")
    if not os.path.exists(path):
        return "(build-order.md missing)"
    text = open(path, encoding="utf-8").read()
    m = re.search(r"\*\*Next:\s*(.+?)\*\*", text, re.S)
    if not m:
        return "(no 'Next:' pointer found in build-order.md)"
    return re.sub(r"\s+", " ", re.sub(r"[`*]", "", m.group(1))).strip()


def total_planned():
    path = os.path.join(REPO, "docs/internal/build-order.md")
    if not os.path.exists(path):
        return None
    m = re.search(r"\*\*(\d+)\s+chunks", open(path, encoding="utf-8").read())
    return int(m.group(1)) if m else None



# ── the rest of the derivable context (added after "if we miss one piece we are
# ── screwed" — the answer is to derive MUCH more, and to gate the part that
# ── cannot be derived at all; see durable_stamp below)

# ── WHY THERE IS NO HEAD SHA, NO COMMITS-AHEAD COUNT AND NO COMMIT LIST HERE ──────
# All three used to be generated into the block, and all three made this gate
# STRUCTURALLY UNSATISFIABLE: each is a function of the commit you are in the act
# of making. Regenerate, stage, commit — and the new commit's SHA, the new
# ahead-count and the new log entry are all instantly wrong, so `check` fails on
# the very commit that just fixed it. There is no fixed point. Every commit from
# the one that introduced this gate left the build red, and nobody noticed only
# because the gate's CMake DEPENDS list did not include this file or .git/HEAD,
# so it re-ran solely when some unrelated document changed.
#
# The cut is not a compromise: the block's job is to describe THE WORK (chunks
# done, suite counts, guards, assumptions, skips, open defects), which changes
# when work lands. The commit graph is not work, and §2 of the briefing already
# instructs the reader to run `git log --oneline -20` and `git status` — which
# answers those three questions live, and cannot go stale at all.
#
# DO NOT ADD THEM BACK. A field this block cannot hold without lying is a field
# that belongs in a command the reader runs.


def skipped_tests():
    """WHICH tests are skipped — each is a deliberate pending-evidence marker."""
    out = []
    for root, _d, files in os.walk(os.path.join(REPO, "test")):
        for f in files:
            if not f.endswith(".cpp"):
                continue
            path = os.path.join(root, f)
            text = open(path, encoding="utf-8", errors="ignore").read()
            for m in re.finditer(r'TEST_CASE\(\s*"([^"]{0,110})"[^)]*?doctest::skip', text, re.S):
                out.append((os.path.relpath(path, REPO), m.group(1)))
    return sorted(out)


def todo_markers():
    """TODO(<CHUNK>) markers in shipped code — outstanding work, by owner."""
    counts = {}
    for base in ("include/shulib", "src"):
        for root, _d, files in os.walk(os.path.join(REPO, base)):
            for f in files:
                if not f.endswith((".hpp", ".cpp")):
                    continue
                text = open(os.path.join(root, f), encoding="utf-8", errors="ignore").read()
                for owner in re.findall(r"TODO\(([A-Za-z0-9/+\u2032\u2019\']{1,12})\)", text):
                    counts[owner] = counts.get(owner, 0) + 1
    return dict(sorted(counts.items()))


def roadmap_partials():
    """`[~]` items — the project's honest-partial marker. Each names an owner."""
    path = os.path.join(REPO, "docs/roadmap.md")
    if not os.path.exists(path):
        return 0
    return len(re.findall(r"^\s*-\s*\[~\]", open(path, encoding="utf-8").read(), re.M))


def open_flags():
    """Explicitly-flagged open defects in the internal record (the 🔴 convention)."""
    out = []
    d = os.path.join(REPO, "docs/internal")
    for root, _dirs, files in os.walk(d):
        for f in files:
            if not f.endswith(".md"):
                continue
            path = os.path.join(root, f)
            for line in open(path, encoding="utf-8", errors="ignore"):
                if line.startswith("### \U0001F534") or line.startswith("## \U0001F534"):
                    out.append((os.path.relpath(path, REPO),
                                re.sub(r"^#+\s*\U0001F534\s*", "", line).strip()))
    return out


def verify_harnesses():
    d = os.path.join(REPO, "docs/internal/verify")
    if not os.path.isdir(d):
        return []
    return sorted(f for f in os.listdir(d) if f.startswith("verify-") and f.endswith(".sh"))


def guard_state():
    """The two CI guards, run for real — not reported from memory.

    The PROS-free guard is PATH-ANCHORED since R1a, exactly like CI's: the one
    exempt path is include/shulib/hal/pros/ (the adapter tree), matched as a
    path PREFIX — never --exclude-dir=pros, which was measured missing a
    violation smuggled into any directory that happens to be named "pros".
    """
    inc = os.path.join(REPO, "include/shulib")
    exempt = os.path.join(inc, "hal/pros/")
    pros_hits = [line for line
                 in _run(["grep", "-rnE",
                          r'#[[:space:]]*include[[:space:]]*[<"]pros/', inc]).splitlines()
                 if line.strip() and not line.startswith(exempt)]
    sim = _run(["grep", "-rnE", "--exclude-dir=sim",
                r'#[[:space:]]*include[[:space:]]*[<"]shulib/sim/', inc])
    return (not pros_hits, not sim.strip())


def pin_counts():
    out = {}
    for name, path in (("F6", "test/f6_signature_pin_test.cpp"),
                       ("F8", "test/f8_signature_pin_test.cpp"),   # frozen at G2, 2026-09-18
                       ("F10", "test/routine_signature_pin_test.cpp")):
        full = os.path.join(REPO, path)
        if os.path.exists(full):
            text = open(full, encoding="utf-8").read()
            # Invocations only: the lookbehind skips the two `#define SHULIB_F…(` lines, which
            # this counted as pins until 2026-09-24 (F6 read 50 for 48 pins, F10 52 for 50).
            out[name] = len(re.findall(r"(?<!#define )SHULIB_F\d+_(?:PIN|NOT_NOEXCEPT)\(", text))
    return out


# ── THE DURABLE-REVIEW STAMP WAS REMOVED AT R1a. Read this before adding it back.
#
# There used to be a `<!-- DURABLE-REVIEWED-AT: <chunk> -->` marker here, and this
# gate failed when a newer chunk had completed than the stamp named. Its stated
# meaning was "a PERSON re-read the hand-written sections", and the briefing said
# outright: "the gate cannot check the prose, so it makes a human check it."
#
# It could not do that, structurally. The chunk's own builder writes the
# `-COMPLETED.md` record that makes the gate fire, AND can edit the stamp that
# silences it — it holds both the trigger and the key. At R1a the build agent did
# exactly that: moved the stamp from F2 to R1a itself. Nothing misbehaved; the
# mechanism simply cannot distinguish a human re-reading from a machine editing a
# marker, and a checkbox that anything can tick is worse than no checkbox, because
# a green one reads as assurance.
#
# What replaced it, honestly: `tools/doc_staleness_audit.py`, which checks a real
# SUBSET mechanically — numeric claims against the repo, documents disagreeing with
# each other, dead file paths — and has a self-test proving each detector fires.
# It checks LESS than the stamp claimed to. Everything it checks is real.
#
# What is therefore NOT enforced any more, said plainly so nobody assumes it is:
# whether the trap list, the standards, the architecture tour or the per-chunk
# narrative are still TRUE. No tool can see that. It is a reading task, it lives in
# the chunk loop as an instruction, and the honest position is that an instruction
# is what it always was.


def render():
    done = completed_chunks()
    flight = in_flight_chunks()
    total = total_planned()
    ha_n, ha_next, ha_settled = ha_entries()
    suite = suite_counts()

    L = [BEGIN, ""]
    L.append("> **Everything below is DERIVED FROM THE REPO, not typed.** A build gate")
    L.append("> (`briefing_status.py check`) fails if it drifts, so it cannot go stale")
    L.append("> silently. Each line names where it comes from — re-check any of it.")
    L.append(">")
    L.append("> **Nothing here is derived from the commit graph** — no HEAD SHA, no")
    L.append("> commits-ahead count, no commit list. Those three cannot be written into a")
    L.append("> committed file without lying, because each is a function of the commit")
    L.append("> being made; they made this gate unsatisfiable and are deliberately gone.")
    L.append("> Run `git log --oneline -20` and `git status` for them — §2 says so already,")
    L.append("> and a command cannot go stale.")
    L.append("")
    L.append(f"**Position:** {len(done)} of {total if total else '?'} chunks complete")
    L.append("")
    L.append(f"- **Next up:** {next_chunk()}  \n  *(source: `build-order.md`'s `Next:` pointer)*")
    if flight:
        L.append(f"- ⚠️ **OPEN CHUNK(S): {', '.join(flight)}** — a `-PROGRESS.md` exists with no "
                 "completion record: either interrupted mid-chunk (`git status` is dirty — read "
                 "that log before anything else) or landed in parts with work remaining (read "
                 "the log before starting its next part).")
    else:
        L.append("- **No open chunks** — every `-PROGRESS.md` has a matching "
                 "`-COMPLETED.md`.")
    if suite and "broken" in suite:
        L.append(f"- **Suite:** **BROKEN** — `./build/test/shulib_tests` is present but "
                 f"{suite['broken']}. Rebuild it from source.")
    elif suite:
        health = ("**green**" if suite["failed"] == 0 and suite["afailed"] == 0
                  else f"**RED — {suite['failed']} case(s) failing**")
        L.append(f"- **Suite:** {suite['cases']:,} cases / {suite['assertions']:,} "
                 f"assertions, {suite['skipped']} skipped — {health}  \n"
                 "  *(source: `./build/test/shulib_tests`. Assertion counts flatter — "
                 "they measure seeds swept. Mutation results are the measure this "
                 "project trusts.)*")
    else:
        L.append("- **Suite:** not built — run `cmake --build build/test` for the count.")
    L.append(f"- **Public headers:** {header_count()}  *(source: `find include/shulib -name '*.hpp'`; "
             "the ARM gate compiles every one)*")
    L.append(f"- **Hardware assumptions:** {ha_n} registered, **{ha_settled} settled** — "
             f"next free is **HA-{ha_next}**  \n"
             "  *(source: `docs/hardware-assumptions.md` — its rows, and its Held numbers line; "
             "the next free is above both. Nothing is settled until "
             "hardware measures it.)*")
    L.append("")
    L.append("**Completed chunks** *(source: the `-COMPLETED.md` records, which are the "
             "project's own definition of done)*:")
    L.append("")
    L.append("> " + " · ".join(f"`{c}`" for c in done))
    L.append("")
    rows = freeze_rows()
    if rows:
        L.append("**Freeze Register** *(source: `docs/roadmap.md`, which owns it)*:")
        L.append("")
        L.append("| Row | Contract | Status |")
        L.append("|---|---|---|")
        for rid, subject, status in rows:
            mark = ("✅" if status == "LOCKED" else
                    "🚧" if status == "open by design" else "🎯")
            L.append(f"| **{rid}** | {subject} | {mark} {status} |")
        L.append("")
        L.append("⚠️ **Register rows F1–F5 are NOT chunks F1–F5.** Row F2 is the accuracy "
                 "targets; chunk F2 was the sequence engine. The collision is in shipped "
                 "code (`spec/accuracy.hpp`). **Never edit rows F1–F5.**")
    L.append("")

    g1, g2 = guard_state()
    pins = pin_counts()
    L.append(f"**Guards, run just now:** PROS-free {'PASS' if g1 else '**FAIL**'} · "
             f"sim-layering {'PASS' if g2 else '**FAIL**'}"
             + (f" · freeze pins: " + ", ".join(f"{k} {v}" for k, v in pins.items()) if pins else ""))
    L.append("")

    harnesses = verify_harnesses()
    if harnesses:
        L.append(f"**Verification harnesses:** {', '.join('`' + h + '`' for h in harnesses)} "
                 "*(the reviewer's, not the chunk's — a chunk must not rewrite one)*")
        L.append("")

    todos = todo_markers()
    if todos:
        L.append("**Outstanding `TODO(chunk)` markers in shipped code** "
                 "*(source: grep over `include/` and `src/`)*:")
        L.append("")
        L.append("> " + " · ".join(f"**{k}**: {v}" for k, v in todos.items()))
        L.append("")

    partials = roadmap_partials()
    L.append(f"**Honest partials:** {partials} `[~]` items in the roadmap, each naming its owner "
             "*(under-claiming is a standard here, so a nonzero count is health, not debt)*")
    L.append("")

    skips = skipped_tests()
    if skips:
        L.append(f"**Deliberately skipped tests ({len(skips)})** — each is evidence that does not "
                 "exist yet, usually pending hardware:")
        L.append("")
        for path, name in skips:
            L.append(f"- `{path}` — {name}")
        L.append("")

    flags = open_flags()
    if flags:
        L.append("**\U0001F534 OPEN DEFECTS flagged in the internal record:**")
        L.append("")
        for path, title in flags:
            L.append(f"- **{title}**  \n  *(recorded in `{path}`)*")
        L.append("")
    else:
        L.append("**No open defects** are flagged with the \U0001F534 convention in "
                 "`docs/internal/`.")
        L.append("")

    L.append("**What just happened:** run `git log --oneline -20`. It is deliberately not "
             "reproduced here — see the note at the top of this block.")
    L.append("")

    L.append(END)
    return "\n".join(L)


def splice(text, block):
    if BEGIN in text and END in text:
        pre = text.split(BEGIN)[0]
        post = text.split(END, 1)[1]
        return pre + block + post
    return text.rstrip() + "\n\n" + block + "\n"


def self_test():
    """The chunk/part rule, both ways, on names and on a scratch records directory — a rule that
    decides the briefing's position line has to be seen failing and passing. Exit 0 or 1."""
    import tempfile
    fails = []
    # (name, the chunk it belongs to): a PART names its parent; every chunk name is its own.
    names = [("COMP-t", "COMP"), ("SAFE-SAVE-a", "SAFE-SAVE"), ("T4-2a", "T4"), ("MECH-core", "MECH"),
             ("COMP", "COMP"), ("R1a", "R1a"), ("R2a", "R2a"), ("R5a", "R5a"), ("SAFE-SAVE", "SAFE-SAVE"),
             ("DEFECTS2", "DEFECTS2"), ("GATE1", "GATE1"), ("L9-A1", "L9-A1"), ("COMP-T", "COMP-T")]
    for n, want in names:
        if chunk_of(n) != want:
            fails.append(f"chunk_of({n!r}) = {chunk_of(n)!r}, want {want!r}")
    with tempfile.TemporaryDirectory() as d:
        for f in ["A1-COMPLETED.md", "R2a-COMPLETED.md", "COMP-t-COMPLETED.md", "COMP-t-PROGRESS.md",
                  "COMP-PROGRESS.md", "T4-PROGRESS.md", "SAFE-SAVE-a-PROGRESS.md", "DIAG1-PROGRESS.md",
                  "DIAG1-COMPLETED.md", "MECH-core-COMPLETED.md"]:
            open(os.path.join(d, f), "w").close()
        got, want = completed_chunks(d), ["A1", "DIAG1", "R2a"]
        if got != want:
            fails.append(f"completed_chunks = {got}, want {want} (a part's record is not a chunk)")
        got, want = in_flight_chunks(d), ["COMP", "SAFE-SAVE", "T4"]
        if got != want:
            fails.append(f"in_flight_chunks = {got}, want {want} (a part's log keeps its parent open; "
                         f"a chunk with its own record is closed)")
        # The parent's OWN record closes it, whatever its parts' records and logs say.
        open(os.path.join(d, "COMP-COMPLETED.md"), "w").close()
        got, want = in_flight_chunks(d), ["SAFE-SAVE", "T4"]
        if got != want:
            fails.append(f"after COMP-COMPLETED.md: in_flight_chunks = {got}, want {want}")
        if completed_chunks(d) != ["A1", "COMP", "DIAG1", "R2a"]:
            fails.append(f"after COMP-COMPLETED.md: completed_chunks = {completed_chunks(d)}")
    # The next free HA number skips what briefs hold.
    reg = "| HA-160 | x |\n| HA-161 | x |\n| HA-167 | x |\n"
    held_line = "> **Held numbers** (reserved by briefs): HA-162..HA-176, HA-181..HA-186.\n"
    ha_cases = [
        (reg, 168, "no Held line: max + 1"),
        (reg + held_line, 187, "held ranges above the rows are skipped"),
        (reg + "> **Held numbers** (x): HA-165.\n", 168, "a held number below the max changes nothing"),
        (reg + "> **Held numbers** (x): HA-170, HA-200.\n", 201, "single held numbers"),
        ("> **Held numbers** (x): HA-3..HA-5.\n", 6, "no rows at all"),
        (reg + "> Held numbers: HA-190.\n", 168, "only the bold, labelled line counts"),
    ]
    for text, want, why in ha_cases:
        got = ha_entries(text)[1]
        if got != want:
            fails.append(f"next free HA = {got}, want {want} ({why})")
    if held_numbers(held_line) != set(range(162, 177)) | set(range(181, 187)):
        fails.append("held_numbers() misread 'HA-162..HA-176, HA-181..HA-186'")
    # The memo: two asks in one invocation run the suite ONCE (a counting stub stands in for the
    # bounded run — nothing is executed, so a noexec temporary directory cannot fail this).
    calls = []
    real_counts = suite_run.counts
    try:
        suite_run.counts = lambda path: calls.append(path) or {"cases": 1, "failed": 0, "skipped": 0,
                                                               "assertions": 1, "afailed": 0}
        _SUITE_MEMO.pop(os.path.abspath("memo-probe"), None)
        first, second = suite_counts("memo-probe"), suite_counts("./memo-probe")
    finally:
        suite_run.counts = real_counts
        _SUITE_MEMO.pop(os.path.abspath("memo-probe"), None)
    if not (first == second and len(calls) == 1):
        fails.append(f"suite_counts memo: {len(calls)} runs for two asks of the same path — want 1")
    # The rule main() applies (the header above suite_rule).
    ok_suite, broken_suite = {"cases": 1, "failed": 0, "skipped": 0, "assertions": 1, "afailed": 0}, {"broken": "x"}
    for cmd, suite, want in (("generate", broken_suite, "refuse"), ("generate", None, "compare"),
                             ("generate", ok_suite, "compare"), ("check", broken_suite, "exempt"),
                             ("check", None, "exempt"), ("check", ok_suite, "compare"),
                             ("show", broken_suite, "compare")):
        if suite_rule(cmd, suite) != want:
            fails.append(f"suite_rule({cmd!r}, {suite}) = {suite_rule(cmd, suite)!r}, want {want!r}")
    # main() itself, end to end, on a scratch copy of the briefing (suite_counts stubbed): generate
    # writes a counted suite and refuses a BROKEN one (file untouched); check then reads current,
    # exempts a BROKEN or absent binary, and fails on a stale block.
    global BRIEFING
    real_briefing, real_counts, real_argv = BRIEFING, suite_counts, sys.argv
    import contextlib
    import io
    with tempfile.TemporaryDirectory() as d:
        try:
            BRIEFING = os.path.join(d, "PROJECT-BRIEFING.md")
            with open(BRIEFING, "w", encoding="utf-8") as f:
                f.write(f"# scratch briefing\n\n{BEGIN}\nold\n{END}\n")
            ok_suite = {"cases": 7, "failed": 0, "skipped": 0, "assertions": 9, "afailed": 0}

            def drive(cmd, suite):
                globals()["suite_counts"] = lambda binary=None: suite
                sys.argv = ["briefing_status.py", cmd]
                with contextlib.redirect_stdout(io.StringIO()) as out, \
                        contextlib.redirect_stderr(io.StringIO()) as err:
                    rc = main()
                return rc, out.getvalue() + err.getvalue()
            rc, _ = drive("generate", ok_suite)
            text = open(BRIEFING, encoding="utf-8").read()
            if rc != 0 or "7 cases" not in text:
                fails.append(f"main(): generate with a counted suite: rc {rc}, suite line written: {'7 cases' in text}")
            rc, msg = drive("generate", {"broken": "probe"})
            if rc != 1 or open(BRIEFING, encoding="utf-8").read() != text or "NOTHING WRITTEN" not in msg:
                fails.append(f"main(): generate with a BROKEN suite: rc {rc} (want 1, the file untouched)")
            for suite, want_rc, word, why in ((ok_suite, 0, "current", "a counted suite, the block current"),
                                              ({"broken": "probe"}, 0, "BROKEN", "a BROKEN binary is exempted, said"),
                                              (None, 0, "no test binary is built", "no binary is exempted, said"),
                                              ({**ok_suite, "cases": 8}, 1, "STALE", "a changed count is stale")):
                rc, msg = drive("check", suite)
                if rc != want_rc or word not in msg:
                    fails.append(f"main(): check, {why}: rc {rc} (want {want_rc}), {word!r} said: {word in msg}")
        finally:
            BRIEFING, sys.argv = real_briefing, real_argv
            globals()["suite_counts"] = real_counts
    # The bounded run itself: tools/suite_run.py's own self-test (every bound and verdict, seen firing).
    if suite_run.self_test() != 0:
        fails.append("tools/suite_run.py self-test failed (its lines above say which case)")
    for f in fails:
        print(f"briefing status self-test: FAIL {f}", file=sys.stderr)
    if fails:
        return 1
    print(f"briefing status self-test: OK ({len(names) + 4 + len(ha_cases) + 1 + 1 + 7 + 6} cases: the chunk/part "
          f"rule on names and on a scratch records directory; the next free HA number past held ones; "
          f"the suite run's memo; the generate/check rule; main() end to end on a scratch briefing) — and "
          f"tools/suite_run.py's own self-test, above")
    return 0


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "self-test":
        return self_test()
    block = render()
    if cmd == "show":
        print(block)
        return 0
    if not os.path.exists(BRIEFING):
        # A MISSING BRIEFING IS EXPECTED ON THE PUBLISHED TREE — DOCS1.
        #
        # `main` is the release branch with docs/internal/ dropped, so the
        # briefing genuinely is not there and never will be. Returning 1 made CI
        # red on main for a condition that is the design working, and no edit to
        # any document could ever have fixed it.
        #
        # `check` therefore succeeds and says why. `generate` still fails, because
        # being asked to rewrite a file that does not exist IS a real error — you
        # are on the wrong branch, and silently doing nothing would hide that.
        if cmd == "check":
            print("briefing status: SKIPPED — docs/internal/ is absent, so there is no "
                  "briefing to check. This is the published tree, and that is correct.")
            return 0
        print(f"briefing status: {BRIEFING} not found", file=sys.stderr)
        return 1
    text = open(BRIEFING, encoding="utf-8").read()
    updated = splice(text, block)
    suite = suite_counts()
    broken = bool(suite and "broken" in suite)
    rule = suite_rule(cmd, suite)
    if rule == "refuse":
        # A broken binary's line is never written into the briefing (header: bounded).
        print(f"briefing status: NOTHING WRITTEN — build/test/shulib_tests is present but BROKEN: "
              f"{suite['broken']}. Recover in this order (each step needs the one before): "
              f"{suite_run.recovery()}", file=sys.stderr)
        return 1
    if cmd == "generate":
        open(BRIEFING, "w", encoding="utf-8").write(updated)
        print("briefing status: regenerated")
        return 0
    if cmd == "check":
        # This gate now checks exactly one thing: the GENERATED block matches the
        # repo. It makes no claim about the hand-written half — see the removal
        # note above the render() function for why the stamp that used to pretend
        # otherwise is gone.
        #
        # ── THE SUITE LINE IS EXEMPT WHEN THERE IS NO BINARY (DOCS1) ──────────
        # This gate went red in CI the first time it ever ran there, and would
        # have stayed red forever, because CI runs the documentation gates BEFORE
        # it builds. With no `build/test/shulib_tests` on disk, suite_counts()
        # returns None, render() writes "Suite: not built", and that differs from
        # the committed block — which carries real numbers, correctly, because a
        # developer generated it against a real build.
        #
        # So the gate was failing on the ABSENCE OF A BUILD ARTIFACT and calling
        # it documentation drift. Those are different things, and conflating them
        # makes the gate unsatisfiable: no committed value can be right both with
        # and without a binary present.
        #
        # The fix is narrow on purpose. Only the one line that cannot be derived
        # is excluded, and only when it genuinely cannot be derived. Everything
        # else in the block — position, next chunk, interrupted chunks, headers,
        # assumptions, the Freeze Register, the guards, the skipped tests — is
        # still compared byte for byte. When a binary IS present the suite line is
        # compared like everything else, so a developer cannot commit a stale
        # count.
        #
        # A binary that is present but BROKEN (crashed, hit a bound, printed no summary — the
        # header's bounded run) is exempted the same way, and SAYS so: the build runs this gate
        # BEFORE it relinks the binary (RELEASING.md §3), so failing here would block the very
        # rebuild that replaces it. `generate` refuses it instead.
        if rule == "exempt":
            drop = lambda b: "\n".join(
                ln for ln in b.splitlines()
                if not ln.startswith("- **Suite:**")
                and "source: `./build/test/shulib_tests`" not in ln)
            if drop(updated) == drop(text):
                if broken:
                    print(f"briefing status: generated block current, EXCEPT the suite line, "
                          f"which was not compared — build/test/shulib_tests is present but "
                          f"BROKEN: {suite['broken']}. The build relinks it; then run "
                          f"generate. (Stuck? {suite_run.recovery()})")
                else:
                    print("briefing status: generated block current, EXCEPT the suite line, "
                          "which was not compared — no test binary is built, so the count "
                          "cannot be derived here. Build the suite to check it.")
                return 0
        if updated == text:
            print("briefing status: generated block current "
                  "(the durable narrative is NOT gated — see doc_staleness_audit.py)")
            return 0
        print("", file=sys.stderr)
        print("PROJECT-BRIEFING IS STALE — its generated status block no longer "
              "matches the repo.", file=sys.stderr)
        print("A stale briefing is worse than none: the next session trusts it and "
              "starts from a false picture.", file=sys.stderr)
        print("", file=sys.stderr)
        if broken:
            # generate refuses a broken binary: the order that works (tools/suite_run.py, recovery()).
            print(f"    the test binary is also BROKEN ({suite['broken']}), so:", file=sys.stderr)
            print(f"    {suite_run.recovery()}", file=sys.stderr)
        else:
            print("    python3 tools/briefing_status.py generate", file=sys.stderr)
        print("", file=sys.stderr)
        print("Only the block between the markers is generated. If a DURABLE section "
              "(the traps, the", file=sys.stderr)
        print("standards, the architecture) is what went stale, fix that by hand — no "
              "tool can see it.", file=sys.stderr)
        # DOCS1: say WHAT drifted, not just THAT something did. This gate failed in
        # CI for two pushes running and the message named nothing, so the only way
        # to see the cause was to reproduce the environment. A gate that reports a
        # failure you cannot act on costs more than it saves.
        import difflib
        diff = list(difflib.unified_diff(
            text.splitlines(), updated.splitlines(),
            fromfile="committed", tofile="regenerated", lineterm="", n=1))
        if diff:
            print("", file=sys.stderr)
            print("The exact drift (committed → regenerated):", file=sys.stderr)
            for line in diff[:60]:
                print(f"  {line}", file=sys.stderr)
            if len(diff) > 60:
                print(f"  … {len(diff) - 60} more diff lines", file=sys.stderr)
        return 1
    print(f"briefing status: unknown command '{cmd}'", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
