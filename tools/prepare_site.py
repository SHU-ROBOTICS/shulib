#!/usr/bin/env python3
"""Prepare docs/ for static-site rendering, without modifying the repository.

Why this exists
---------------
The documentation is written to be read TWO ways, and the two disagree about
what a link means:

  * In the repository (GitHub renders the markdown directly). Links like
    `](../../include/shulib/chassis/chassis.hpp)` are correct here, and they
    are the reason a reader can jump straight from a doc to the code it
    describes. There are 117 of them.

  * On a rendered site, whose root is `docs/`. Those same links point outside
    the site root and 404.

Rather than degrade one to serve the other, this script produces a *copy* of
`docs/` with escaping links rewritten to absolute GitHub URLs. The committed
docs keep working in the repo; the published site works too. Nothing in the
repository is modified — the copy is a build artifact.

It also enforces two safety properties, because this is the step where a
mistake becomes public:

  1. `docs/internal/` never reaches the site. The published branch is `main`,
     which does not contain it at all, so this is belt-and-braces — but a
     publish step is exactly where "structurally impossible" quietly becomes
     "nobody checked". C7 established that property; this keeps it true at the
     last moment it can be broken.
  2. No surviving link escapes the site root. If the rewrite misses a shape,
     the build fails here rather than shipping a dead link.

It also gives every row of the Hardware Assumptions Register its own anchor
(`#ha-136`), so a page can link the one row an estimate rests on. The anchors are
added to the COPY only: every robot-code change edits register rows, and an
anchor edit on every row of the source would conflict with each of them. On GitHub
(the raw docs/) such a link lands on the page top; the site is where readers read.

Usage:  python3 tools/prepare_site.py <output-dir>
        python3 tools/prepare_site.py check-register   (the real register's anchors, without a copy)
        python3 tools/prepare_site.py self-test
"""

from __future__ import annotations

import os
import re
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DOCS = REPO / "docs"

# The blob root for source links. Pinned to main because that is the published
# branch (D3's publishing decision) — a docs site linking into a development
# branch would rot the moment that branch moves.
BLOB = "https://github.com/SHU-ROBOTICS/shulib/blob/main"

# Directories under docs/ that must never be published.
EXCLUDED = {"internal"}

# The register, and a row of it: "| HA-136 | ..." (the first cell is the id).
REGISTER = "hardware-assumptions.md"
HA_ROW = re.compile(r"^\|\s*(HA-(\d+))\s*\|", re.M)

# ](target) where target is relative — captures the path and any #anchor.
LINK = re.compile(r"\]\((?!https?://|#|mailto:)([^)#\s]+)(#[^)\s]*)?\)")


def build_stamp() -> str:
    """The 'last updated' footer, derived from the commit being published.

    WHY THIS IS GENERATED AND NOT WRITTEN DOWN
    ------------------------------------------
    A hand-maintained date is wrong the moment anyone publishes without touching
    it, and a stale date carrying a confident tone is worse than none — the exact
    failure this project has hit repeatedly with test counts and register sizes.
    So the stamp is derived here, at the only moment it can be correct: the
    publish that produces the artifact.

    WHY IT SAYS WHICH BRANCH
    ------------------------
    The site publishes from the release branch, not from the working branch, so
    it lags the repository ON PURPOSE — that is what keeps the development log
    off the public site by construction. A reader seeing a date two weeks old
    should be told that is the design and not rot, and told where the current
    tree is. Saying only "last updated" would invite exactly the wrong inference.

    `git log -1` works under actions/checkout's default shallow clone, which is
    what CI uses. If git is unavailable the stamp degrades to naming the branch
    and omits what it cannot know, rather than inventing a date.
    """
    import subprocess

    def git(*args: str) -> str:
        try:
            return subprocess.run(("git", *args), cwd=REPO, capture_output=True,
                                  text=True, timeout=30).stdout.strip()
        except Exception:
            return ""

    sha = git("log", "-1", "--format=%h")
    date = git("log", "-1", "--format=%cd", "--date=format:%Y-%m-%d")
    repo_url = "https://github.com/SHU-ROBOTICS/shulib"

    # The date is the COMMIT date of what is being published, not the time the
    # renderer happened to run. That is the honest reading of "last updated": it
    # is when this content last changed. A rebuild with no content change must
    # not advance it, or the stamp starts meaning "a machine ran", which is not
    # what anyone reads it as.
    if sha and date:
        what = (f"**Last updated {date}** — published from the `main` release branch, "
                f"commit [`{sha}`]({repo_url}/commit/{sha}).")
    else:
        what = "Published from the `main` release branch."

    return (
        "\n\n---\n\n"
        f"<small>{what} This site tracks **releases**, so between them it can lag the "
        "repository — that lag is deliberate, and it is what keeps the development log off "
        f"the public site. For the current state of the code, see "
        f"[the repository]({repo_url}).</small>\n"
    )


def rewrite(text: str, md_path: Path) -> tuple[str, int]:
    """Rewrite links that escape the docs root into absolute GitHub URLs."""
    rewrites = 0

    def sub(m: re.Match[str]) -> str:
        nonlocal rewrites
        target, anchor = m.group(1), m.group(2) or ""
        resolved = (md_path.parent / target).resolve()
        try:
            resolved.relative_to(DOCS)
            return m.group(0)  # stays inside the site — leave it alone
        except ValueError:
            pass
        try:
            rel = resolved.relative_to(REPO)
        except ValueError:
            # Escapes the repository entirely. Never seen; fail loudly rather
            # than emit a URL that is definitely wrong.
            raise SystemExit(
                f"ERROR: {md_path.relative_to(REPO)} links outside the repository: {target}"
            )
        rewrites += 1
        return f"]({BLOB}/{rel.as_posix()}{anchor})"

    return LINK.sub(sub, text), rewrites


def anchor_rows(text: str) -> tuple[str, int]:
    """Give each register row's first cell an <a id="ha-NNN"></a>."""
    return HA_ROW.subn(lambda m: f'| <a id="ha-{m.group(2)}"></a>{m.group(1)} |', text)


def unanchored_rows(text: str) -> list[str]:
    """Register rows whose id has no anchor, and ids anchored twice."""
    ids = re.findall(r'^\|\s*(?:<a id="ha-\d+"></a>)?\s*(HA-\d+)\s*\|', text, re.M)
    anchors = re.findall(r'<a id="ha-(\d+)"></a>', text)
    bad = [i for i in ids if i[3:] not in anchors]
    bad += [f"ha-{a} twice" for a in sorted(set(anchors)) if anchors.count(a) > 1]
    return bad


def prepare_page(rel: str, text: str, src: Path) -> tuple[str, int, int]:
    """One page as the site gets it: (text, source links rewritten, register rows anchored).
    The register fails here if it has no rows, or a row without exactly one anchor."""
    text, n = rewrite(text, src)
    rows = 0
    if rel == REGISTER:
        text, rows = anchor_rows(text)
        missing = unanchored_rows(text)
        if not rows or missing:
            raise SystemExit(f"ERROR: register row anchors: {rows} added, missing {missing[:5]}")
    return text, n, rows


def register_rows(text: str) -> int:
    """The register's rows counted another way, WIDER than HA_ROW on purpose: any table line (spaces
    before its bar allowed) whose first cell holds an HA id anywhere ("**HA-7**" too). A row HA_ROW
    cannot read is counted here and not anchored there, so check-register fails on it."""
    count = 0
    for line in text.split("\n"):
        if line.lstrip().startswith("|") and re.search(r"HA-\d+", line.split("|")[1]):
            count += 1
    return count


def check_register(src: Path = DOCS / REGISTER) -> int:
    """The real register through the copy step's own code: every row anchored, none missed."""
    if not src.is_file():
        print(f"check-register: FAIL — {src} is missing", file=sys.stderr)
        return 1
    text = src.read_text(encoding="utf-8")
    try:
        _out, _n, rows = prepare_page(REGISTER, text, src)
    except SystemExit as e:
        print(f"check-register: FAIL — {e}", file=sys.stderr)
        return 1
    if rows != register_rows(text):
        print(f"check-register: FAIL — {rows} rows anchored, {register_rows(text)} in the register: write "
              "each row's first cell as '| HA-NNN |' (no bold, no space before the bar)", file=sys.stderr)
        return 1
    print(f"check-register: OK — {rows} register rows, each anchored once")
    return 0


def self_test(plant: bool = False) -> int:
    import contextlib
    import io
    import tempfile

    rows = "| HA-01 | a | b |\n| HA-136 | c | d |\n| Old | HA-12 | e |\n"
    out, n = anchor_rows(rows)

    def refused(text):
        try:
            prepare_page(REGISTER, text, DOCS / REGISTER)
        except SystemExit:
            return True
        return False

    def checked(text):
        with tempfile.TemporaryDirectory(prefix="shulib-site-") as d:
            path = Path(d) / REGISTER
            if text is not None:
                path.write_text(text, encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                return check_register(path)

    cases = [
        ("every row gets its anchor", n == 2 and unanchored_rows(out) == []),
        ("the anchor is the row's id", '<a id="ha-136"></a>HA-136' in out and '<a id="ha-01"></a>HA-01' in out),
        ("a row that names an id in a later cell is not a row of it", "<a id=\"ha-12\">" not in out),
        ("a row without its anchor fails", unanchored_rows(rows) == ["HA-01", "HA-136"]),
        ("an id anchored twice fails", unanchored_rows(out + out) != []),
        ("the copy step anchors the register", prepare_page(REGISTER, rows, DOCS / REGISTER)[2] == 2),
        ("the copy step leaves every other page's rows alone", prepare_page("faq.md", rows, DOCS / "faq.md")[0] == rows),
        ("the copy step refuses a register with no rows", refused("# Register\n")),
        ("the copy step refuses an id twice", refused(rows + rows)),
        ("rows are counted another way", register_rows(rows) == 2),
        ("check-register passes a good register", checked(rows) == 0),
        ("check-register fails a register with no rows", checked("# Register\n") == 1),
        ("check-register fails a missing register", checked(None) == 1),
        ("check-register fails a row the anchor pattern misses", checked(rows + "| HA-7\n") == 1),
        ("check-register fails a bold id", checked(rows + "| **HA-7** | x |\n") == 1),
        ("check-register fails a row with a space before its bar", checked(rows + " | HA-7 | x |\n") == 1),
        ("a row anchored with another row's id fails",
         unanchored_rows('| <a id="ha-02"></a>HA-01 | x |\n') == ["HA-01"]),
    ]
    if plant:
        cases.append(("a planted failing case", False))
    for name, ok in cases:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    return 0 if all(ok for _n, ok in cases) else 1


def main() -> int:
    if sys.argv[1:] == ["self-test"]:
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            planted = self_test(plant=True)
        if planted != 1:  # the exit itself is checked: a run with a failing case must exit 1
            print("self-test: FAIL — a run with a planted failing case exits 0", file=sys.stderr)
            return 1
        return self_test()
    if sys.argv[1:] == ["check-register"]:
        return check_register()
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    out = Path(sys.argv[1]).resolve()

    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    # Derived once, so every page carries the SAME stamp — a site whose pages
    # disagree about when it was published is worse than one with no stamp.
    stamp = build_stamp()

    copied = rewritten = stamped = rows = 0
    for src in sorted(DOCS.rglob("*")):
        rel = src.relative_to(DOCS)
        if rel.parts and rel.parts[0] in EXCLUDED:
            continue
        dst = out / rel
        if src.is_dir():
            dst.mkdir(parents=True, exist_ok=True)
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        if src.suffix == ".md":
            text, n, page_rows = prepare_page(rel.as_posix(), src.read_text(encoding="utf-8"), src)
            rows += page_rows
            # Appended AFTER the rewrite, deliberately: the stamp's links are
            # already absolute, and running them through the escape-check would
            # be checking this function's own output rather than the documents.
            text += stamp
            dst.write_text(text, encoding="utf-8")
            rewritten += n
            stamped += 1
        else:
            shutil.copy2(src, dst)
        copied += 1

    # mkdocs wants a homepage (index.md); the repo wants a README (GitHub shows
    # README.md when you browse a directory, and ignores index.md). One file
    # does both jobs — but it must be RENAMED here, not copied: mkdocs treats a
    # README.md and an index.md in the same directory as a conflict, warns, and
    # under `strict: true` that warning fails the build. Renaming in the copy
    # leaves docs/README.md untouched in the repository.
    #
    # Only the docs ROOT README is renamed. guide/README.md, cookbook/README.md
    # and api/README.md are section landing pages that the nav and 4 in-tree
    # links point at by name; they have no sibling index.md, so they neither
    # conflict nor move.
    if not rows:
        raise SystemExit(f"ERROR: docs/{REGISTER} was not copied — no register row has an anchor")

    readme = out / "README.md"
    if not readme.exists():
        raise SystemExit("ERROR: docs/README.md is missing — the site has no homepage")
    readme.rename(out / "index.md")

    # ── safety property 1: no internal documentation reached the site ─────────
    leaks = []
    for md in out.rglob("*.md"):
        body = md.read_text(encoding="utf-8")
        for pat in ("docs/internal", "](internal/", "](../internal/", "-COMPLETED.md",
                    "-PROGRESS.md", "build-order", "RESUMING"):
            if pat in body:
                leaks.append(f"{md.relative_to(out)}: {pat}")
    if (out / "internal").exists():
        leaks.append("internal/ directory was copied")
    if leaks:
        print("ERROR: internal development docs would be PUBLISHED:", file=sys.stderr)
        for leak in leaks:
            print(f"  {leak}", file=sys.stderr)
        return 1

    # ── safety property 2: no surviving relative link escapes the site root ───
    dead = []
    for md in out.rglob("*.md"):
        for m in LINK.finditer(md.read_text(encoding="utf-8")):
            resolved = (md.parent / m.group(1)).resolve()
            try:
                resolved.relative_to(out)
            except ValueError:
                dead.append(f"{md.relative_to(out)} -> {m.group(1)}")
    if dead:
        print("ERROR: links escape the site root (the rewrite missed a shape):",
              file=sys.stderr)
        for d in dead:
            print(f"  {d}", file=sys.stderr)
        return 1

    print(f"site source prepared: {copied} files, {rewritten} source links "
          f"rewritten to {BLOB}")
    print(f"  publish stamp: added to {stamped} pages")
    print("  internal docs: absent (verified)")
    print("  escaping links: none (verified)")
    print(f"  register rows: {rows} anchored (#ha-NNN)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
