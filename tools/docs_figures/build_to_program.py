#!/usr/bin/env python3
"""Draws docs/assets/build-to-program.svg, "Where your code goes"'s picture of one build becoming one
program, FROM that page's two tables — so the picture and the page cannot disagree.

  python3 tools/docs_figures/build_to_program.py            write the picture
  python3 tools/docs_figures/build_to_program.py check      fail if the picture no longer matches the page
  python3 tools/docs_figures/build_to_program.py self-test  the check, shown failing on scratch files

The picture follows the "shulib Auton Tester" row of "## The programs": its build, then the two files
of "## The folders" a build passes through (src/build_selection.hpp, src/main.cpp, each with its "What
is in it" words), then the row's program file, then the row's team files.
"""
import contextlib
import io
import os
import re
import sys
import tempfile
from xml.sax.saxutils import escape

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PAGE = os.path.join(REPO, "docs/where-your-code-goes.md")
OUT = os.path.join(REPO, "docs/assets/build-to-program.svg")
PROGRAM = "shulib Auton Tester"
PLUMBING = ("src/build_selection.hpp", "src/main.cpp")
W, BOX_H, GAP, TOP, LINE = 540, 58, 26, 48, 17
BLUE, GREY, GREEN = ("#e8eaf6", "#3f51b5"), ("#eceff1", "#607d8b"), ("#e8f5e9", "#2e7d32")


class FigureError(Exception):
    """The page's tables cannot be drawn."""


def table(text, heading):
    """The rows of the first table under `heading`, each a list of cells with their backticks removed."""
    if heading not in text:
        raise FigureError(f"the page has no '{heading}' section")
    body = text.split(heading, 1)[1].split("\n## ", 1)[0]
    rows = [line for line in body.splitlines() if line.startswith("|")]
    if len(rows) < 3:
        raise FigureError(f"no table under '{heading}'")
    return [[c.strip().replace("`", "") for c in r.strip().strip("|").split("|")] for r in rows[2:]]


def boxes(text):
    """[(title, caption, colours)] top to bottom."""
    folders = {r[0]: r[1] for r in table(text, "## The folders") if len(r) >= 2}
    row = next((r for r in table(text, "## The programs") if len(r) >= 5 and r[1] == PROGRAM), None)
    if row is None:
        raise FigureError(f"'## The programs' has no '{PROGRAM}' row")
    build, name, slot, program, files = row[:5]
    out = [(build, "you type this", BLUE)]
    for f in PLUMBING:
        if f not in folders:
            raise FigureError(f"'## The folders' has no {f} row")
        out.append((f, folders[f], GREY))
    out.append((program, f"{name}, slot {slot}", GREY))
    team = [f.strip() for f in files.split(",") if f.strip().startswith("src/team/")]
    if not team:
        raise FigureError(f"the '{PROGRAM}' row names no file in src/team/")
    out.append((" + ".join(team), "your code", GREEN))
    return out


def render(text):
    bx = boxes(text)
    h = TOP + len(bx) * BOX_H + (len(bx) - 1) * GAP + 16
    names = "; then ".join(t for t, _c, _k in bx)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {h}" width="{W}" height="{h}" '
           f'font-family="Helvetica, Arial, sans-serif" role="img" aria-label="One build, one program">',
           f"<title>One build, one program: {escape(names)}</title>",
           '<defs><marker id="bp" markerUnits="userSpaceOnUse" markerWidth="12" markerHeight="10" refX="11" '
           'refY="5" orient="auto"><path d="M0,0 L12,5 L0,10 z" fill="#3f51b5"/></marker></defs>',
           f'<rect width="{W}" height="{h}" fill="#ffffff"/>',
           f'<text x="{W // 2}" y="28" text-anchor="middle" font-size="17" font-weight="bold" fill="#1a237e">'
           "One build, one program</text>"]
    y = TOP
    for i, (title, caption, (fill, stroke)) in enumerate(bx):
        out.append(f'<rect x="12" y="{y}" width="{W - 24}" height="{BOX_H}" rx="10" fill="{fill}" stroke="{stroke}"/>')
        out.append(f'<text x="28" y="{y + 25}" font-size="14" font-family="Menlo, Consolas, monospace" '
                   f'fill="#111111">{escape(title)}</text>')
        out.append(f'<text x="28" y="{y + 25 + LINE + 2}" font-size="13" fill="{stroke}">{escape(caption)}</text>')
        if i < len(bx) - 1:
            out.append(f'<line x1="{W // 2}" y1="{y + BOX_H + 2}" x2="{W // 2}" y2="{y + BOX_H + GAP - 2}" '
                       'stroke="#3f51b5" stroke-width="2.5" marker-end="url(#bp)"/>')
        y += BOX_H + GAP
    out.append("</svg>")
    return "\n".join(out) + "\n"


def drawn(page):
    """The picture the page draws, or None after saying why it cannot be drawn."""
    try:
        return render(open(page, encoding="utf-8").read())
    except (OSError, FigureError) as e:
        print(f"build_to_program: FAIL — {e}", file=sys.stderr)
        return None


def check(page=PAGE, out=OUT):
    svg = drawn(page)
    if svg is None:
        return 1
    have = open(out, encoding="utf-8").read() if os.path.exists(out) else ""
    if have != svg:
        print("build_to_program: FAIL — docs/assets/build-to-program.svg no longer matches "
              "docs/where-your-code-goes.md — run: python3 tools/docs_figures/build_to_program.py", file=sys.stderr)
        return 1
    print("build_to_program: OK — the picture matches the page's tables")
    return 0


def write(page=PAGE, out=OUT):
    svg = drawn(page)
    if svg is None:
        return 1
    with open(out, "w", encoding="utf-8") as f:
        f.write(svg)
    print("wrote", out)
    return 0


# A scratch page: both tables, an extra row on each, and a later section whose table is not read.
SCRATCH = """# Where

## The folders

| Folder | What is in it | Change it when |
|---|---|---|
| `src/team/` | yours | always |
| `src/build_selection.hpp` | picks it | rarely |
| `src/main.cpp` | hands it over | rarely |

## The programs

| Build | On the brain | Slot | Program file | Files |
|---|---|---|---|---|
| `make ROBOT=one PROGRAM=drive` | shulib Drive | 1 | `src/programs/drive_program.cpp` | `src/team/robot_one_driver.hpp` |
| `make ROBOT=one PROGRAM=autontest` | shulib Auton Tester | 2 | `src/programs/auton_tester_program.cpp` | `src/team/a.cpp`, `src/team/b.cpp` |

## Later

| Build | On the brain | Slot | Program file | Files |
|---|---|---|---|---|
| `make WRONG` | shulib Auton Tester | 9 | `x` | `src/team/z.cpp` |
"""


def self_test(plant=False):
    cases = [("a planted failing case", False)] if plant else []

    def listed(text):
        try:
            return [(t, c) for t, c, _k in boxes(text)]
        except FigureError:
            return None

    cases.append(("the five boxes in order, from the right rows and nothing from a later section", listed(SCRATCH) == [
        ("make ROBOT=one PROGRAM=autontest", "you type this"), ("src/build_selection.hpp", "picks it"),
        ("src/main.cpp", "hands it over"), ("src/programs/auton_tester_program.cpp", "shulib Auton Tester, slot 2"),
        ("src/team/a.cpp + src/team/b.cpp", "your code")]))
    with tempfile.TemporaryDirectory(prefix="shulib-build-to-program-") as d:
        page, out = os.path.join(d, "page.md"), os.path.join(d, "build-to-program.svg")

        def put(path, text):
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)

        def run(command):
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                return command(page, out)

        put(page, SCRATCH)
        cases.append(("writing the picture, then checking it, passes", run(write) == 0 and run(check) == 0))
        good = open(out, encoding="utf-8").read() if os.path.exists(out) else ""
        cases.append(("the picture names every box", all(s in good for s in (
            "make ROBOT=one PROGRAM=autontest", "picks it", "hands it over", "shulib Auton Tester, slot 2",
            "src/team/a.cpp + src/team/b.cpp"))))
        put(page, SCRATCH.replace("| 2 | `src/programs/auton_tester_program.cpp`", "| 3 | `src/programs/auton_tester_program.cpp`"))
        cases.append(("a changed slot fails", run(check) == 1))
        put(page, SCRATCH.replace("| hands it over |", "| hands it on |"))
        cases.append(("main.cpp's changed words fail", run(check) == 1))
        put(page, SCRATCH)
        put(out, good + " ")
        cases.append(("a stale picture fails", run(check) == 1))
        os.remove(out)
        cases.append(("a missing picture fails", run(check) == 1))
        put(out, good)
        put(page, SCRATCH.replace("| shulib Auton Tester | 2 |", "| shulib Tester | 2 |", 1))
        cases.append(("a page without the Auton Tester row fails", run(check) == 1))
        put(page, SCRATCH.replace("| `src/main.cpp` | hands it over | rarely |\n", ""))
        cases.append(("a page without main.cpp's row fails", run(check) == 1))
        put(page, SCRATCH.replace("`src/team/a.cpp`, `src/team/b.cpp` |\n\n## Later", "none |\n\n## Later"))
        cases.append(("a row with no team file fails", run(check) == 1))
        put(page, SCRATCH.replace("## The folders", "## Folders"))
        cases.append(("a page without the folders heading fails", run(check) == 1))
        os.remove(page)
        cases.append(("a missing page fails", run(check) == 1))
    for name, ok in cases:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if any(not ok for _n, ok in cases):
        return 1
    print(f"build_to_program: self-test OK — {len(cases)} cases")
    return 0


def main(argv):
    if argv[1:] == ["check"]:
        return check()
    if argv[1:] == ["self-test"]:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            planted = self_test(plant=True)
        if planted != 1:  # the exit itself is checked: a run with a failing case must exit 1
            print("build_to_program: self-test FAIL — a run with a planted failing case exits 0", file=sys.stderr)
            return 1
        return self_test()
    if argv[1:]:
        print(__doc__, file=sys.stderr)
        return 2
    return write()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
