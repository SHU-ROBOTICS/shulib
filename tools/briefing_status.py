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

`check` runs as a build-time doc gate beside check-coverage / check-fresh /
check-examples / check-removability.

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


def completed_chunks():
    """Chunks with a completion record — the project's own definition of done."""
    d = os.path.join(REPO, "docs/internal/chunks")
    if not os.path.isdir(d):
        return []
    names = [f[: -len("-COMPLETED.md")] for f in os.listdir(d)
             if f.endswith("-COMPLETED.md")]

    def key(n):
        m = re.match(r"([A-Z]+)(\d+)", n)
        return (m.group(1), int(m.group(2))) if m else (n, 0)

    return sorted(names, key=key)


def in_flight_chunks():
    """A PROGRESS log with no COMPLETED record = a chunk that was interrupted."""
    d = os.path.join(REPO, "docs/internal/chunks")
    if not os.path.isdir(d):
        return []
    done = set(completed_chunks())
    return sorted({f[: -len("-PROGRESS.md")] for f in os.listdir(d)
                   if f.endswith("-PROGRESS.md")} - done)


def header_count():
    n = 0
    for root, _dirs, files in os.walk(os.path.join(REPO, "include/shulib")):
        n += sum(1 for f in files if f.endswith(".hpp"))
    return n


def ha_entries():
    """Registered hardware assumptions, and the next free number."""
    path = os.path.join(REPO, "docs/hardware-assumptions.md")
    if not os.path.exists(path):
        return 0, 1, 0
    text = open(path, encoding="utf-8").read()
    ids = [int(m) for m in re.findall(r"^\|\s*HA-(\d+)", text, re.M)]
    settled = len(re.findall(r"^\|\s*HA-\d+.*\bsettled\b", text, re.M | re.I))
    return (len(ids), (max(ids) + 1) if ids else 1, settled)


def suite_counts():
    """The authoritative test numbers, from the built binary. ('—' if unbuilt.)"""
    binary = os.path.join(REPO, "build/test/shulib_tests")
    if not os.path.exists(binary):
        return None
    out = _run([binary, "--no-intro=true"])
    cases = re.search(r"test cases:\s*(\d+)\s*\|\s*(\d+) passed \|\s*(\d+) failed"
                      r"\s*\|\s*(\d+) skipped", out)
    asserts = re.search(r"assertions:\s*(\d+)\s*\|\s*(\d+) passed \|\s*(\d+) failed", out)
    if not cases or not asserts:
        return None
    return {"cases": int(cases.group(1)), "failed": int(cases.group(3)),
            "skipped": int(cases.group(4)), "assertions": int(asserts.group(1)),
            "afailed": int(asserts.group(3))}


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
    """The two CI guards, run for real — not reported from memory."""
    inc = os.path.join(REPO, "include/shulib")
    pros = _run(["grep", "-rnE", r'#[[:space:]]*include[[:space:]]*[<"]pros/', inc])
    sim = _run(["grep", "-rnE", "--exclude-dir=sim",
                r'#[[:space:]]*include[[:space:]]*[<"]shulib/sim/', inc])
    return (not pros.strip(), not sim.strip())


def pin_counts():
    out = {}
    for name, path in (("F6", "test/f6_signature_pin_test.cpp"),
                       ("F10", "test/routine_signature_pin_test.cpp")):
        full = os.path.join(REPO, path)
        if os.path.exists(full):
            text = open(full, encoding="utf-8").read()
            out[name] = len(re.findall(r"SHULIB_F\d+_(?:PIN|NOT_NOEXCEPT)\(", text))
    return out


DURABLE_STAMP = re.compile(r"<!--\s*DURABLE-REVIEWED-AT:\s*([A-Z]+\d+)\s*-->")


def durable_stamp(text):
    m = DURABLE_STAMP.search(text)
    return m.group(1) if m else None


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
        L.append(f"- ⚠️ **INTERRUPTED CHUNK(S): {', '.join(flight)}** — a `-PROGRESS.md` exists "
                 "with no completion record. **Read that log before anything else.**")
    else:
        L.append("- **No interrupted chunks** — every `-PROGRESS.md` has a matching "
                 "`-COMPLETED.md`.")
    if suite:
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
             "  *(source: `docs/hardware-assumptions.md`. Nothing is settled until "
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


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    block = render()
    if cmd == "show":
        print(block)
        return 0
    if not os.path.exists(BRIEFING):
        print(f"briefing status: {BRIEFING} not found", file=sys.stderr)
        return 1
    text = open(BRIEFING, encoding="utf-8").read()
    updated = splice(text, block)
    if cmd == "generate":
        open(BRIEFING, "w", encoding="utf-8").write(updated)
        print("briefing status: regenerated")
        return 0
    if cmd == "check":
        # ── the half no script can judge ──────────────────────────────────────
        # Generation covers what is DERIVABLE. It cannot tell whether the durable
        # narrative — the traps, the standards, the architecture, "what each chunk
        # taught" — is still TRUE. Pretending otherwise is the real danger, so
        # instead of pretending, this forces a human pass: the briefing carries a
        # stamp naming the last chunk at which a person re-read those sections,
        # and the gate fails when work has landed since.
        done = completed_chunks()
        newest = done[-1] if done else None
        stamp = durable_stamp(text)
        if newest and stamp != newest:
            print("", file=sys.stderr)
            print(f"PROJECT-BRIEFING'S DURABLE SECTIONS HAVE NOT BEEN RE-READ SINCE "
                  f"{stamp or '(never)'} — {newest} has landed since.", file=sys.stderr)
            print("", file=sys.stderr)
            print("The generated block is only the DERIVABLE half. No tool can tell you "
                  "whether the", file=sys.stderr)
            print("architecture tour, the trap list, the standards or the per-chunk "
                  "narrative are still", file=sys.stderr)
            print("true. Re-read them against what just changed, fix what drifted, then "
                  "stamp it:", file=sys.stderr)
            print("", file=sys.stderr)
            print(f"    <!-- DURABLE-REVIEWED-AT: {newest} -->", file=sys.stderr)
            print("", file=sys.stderr)
            print("Ask specifically: did this chunk add a trap, invalidate a standard, "
                  "change a layer,", file=sys.stderr)
            print("or make a sentence in the briefing false? If yes, that edit is the "
                  "point of this gate.", file=sys.stderr)
            return 1
        if updated == text:
            print(f"briefing status: current (durable sections reviewed at {stamp})")
            return 0
        print("", file=sys.stderr)
        print("PROJECT-BRIEFING IS STALE — its generated status block no longer "
              "matches the repo.", file=sys.stderr)
        print("A stale briefing is worse than none: the next session trusts it and "
              "starts from a false picture.", file=sys.stderr)
        print("", file=sys.stderr)
        print("    python3 tools/briefing_status.py generate", file=sys.stderr)
        print("", file=sys.stderr)
        print("Only the block between the markers is generated. If a DURABLE section "
              "(the traps, the", file=sys.stderr)
        print("standards, the architecture) is what went stale, fix that by hand — no "
              "tool can see it.", file=sys.stderr)
        return 1
    print(f"briefing status: unknown command '{cmd}'", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
