#!/usr/bin/env python3
"""Draws docs/assets/first-week.svg, the front page's "a new robot's first week", FROM the numbered list
under "## The order on the robot" in guide 21 — so the picture and the page cannot disagree.

  python3 tools/docs_figures/first_week.py            write the picture
  python3 tools/docs_figures/first_week.py check      fail if the picture no longer matches guide 21
  python3 tools/docs_figures/first_week.py self-test  the check, shown failing on scratch files

Each step's name is the step's programs in bold with their slots ("**Bench Tests** (slot 3)"), or else
its first bold words.
"""
import contextlib
import io
import os
import re
import sys
import tempfile
from xml.sax.saxutils import escape

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GUIDE = os.path.join(REPO, "docs/guide/21-adding-your-robot-by-hand.md")
OUT = os.path.join(REPO, "docs/assets/first-week.svg")
HEADING = "## The order on the robot"
W, BOX_H, GAP, TOP, LINE = 460, 56, 28, 48, 17


class FigureError(Exception):
    """Guide 21's list cannot be drawn."""


def steps(text):
    """[(name, slots)] for each numbered item under HEADING."""
    if HEADING not in text:
        raise FigureError(f"guide 21 has no '{HEADING}' section")
    body = text.split(HEADING, 1)[1].split("\n## ", 1)[0]
    items = re.split(r"\n(?=\d+\. )", "\n" + body.strip())
    out = []
    for item in items:
        if not re.match(r"\d+\. ", item.strip()):
            continue
        flat = " ".join(item.split())
        programs = re.findall(r"\*\*([^*]+)\*\* \(slot (\d+)\)", flat)
        if programs:
            out.append((" and ".join(p for p, _ in programs), [s for _, s in programs]))
        else:
            first = re.search(r"\*\*([^*]+)\*\*", flat)
            if not first:
                raise FigureError(f"a step in guide 21 has no bold name: {flat[:60]!r}")
            out.append((first.group(1), []))
    if not out:
        raise FigureError("no numbered steps under " + HEADING)
    return out


def wrap(words, width=38):
    lines, cur = [], ""
    for w in words.split():
        if cur and len(cur) + 1 + len(w) > width:
            lines.append(cur)
            cur = w
        else:
            cur = f"{cur} {w}".strip()
    return lines + [cur]


def render(text):
    st = steps(text)
    h = TOP + len(st) * BOX_H + (len(st) - 1) * GAP + 16
    names = "; then ".join(n for n, _ in st)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {h}" width="{W}" height="{h}" '
           f'font-family="Helvetica, Arial, sans-serif" role="img" aria-label="A new robot\'s first week">',
           f"<title>A new robot's first week: {escape(names)}</title>",
           '<defs><marker id="fw" markerUnits="userSpaceOnUse" markerWidth="12" markerHeight="10" refX="11" '
           'refY="5" orient="auto"><path d="M0,0 L12,5 L0,10 z" fill="#3f51b5"/></marker></defs>',
           f'<rect width="{W}" height="{h}" fill="#ffffff"/>',
           f'<text x="{W // 2}" y="28" text-anchor="middle" font-size="17" font-weight="bold" fill="#1a237e">'
           "A new robot's first week</text>"]
    y = TOP
    for i, (name, slots) in enumerate(st, 1):
        out.append(f'<rect x="12" y="{y}" width="{W - 24}" height="{BOX_H}" rx="10" fill="#e8eaf6" stroke="#3f51b5"/>')
        out.append(f'<circle cx="44" cy="{y + BOX_H // 2}" r="16" fill="#3f51b5"/>')
        out.append(f'<text x="44" y="{y + BOX_H // 2 + 6}" text-anchor="middle" font-size="17" '
                   f'font-weight="bold" fill="#ffffff">{i}</text>')
        lines = wrap(name)
        if slots:
            lines.append(("slot " if len(slots) == 1 else "slots ") + " and ".join(slots))
        y0 = y + BOX_H // 2 - (len(lines) - 1) * LINE // 2 + 5
        for j, line in enumerate(lines):
            small = slots and j == len(lines) - 1
            out.append(f'<text x="72" y="{y0 + j * LINE}" font-size="{13 if small else 15}" '
                       f'fill="{"#455a64" if small else "#111111"}">{escape(line)}</text>')
        if i < len(st):
            out.append(f'<line x1="{W // 2}" y1="{y + BOX_H + 2}" x2="{W // 2}" y2="{y + BOX_H + GAP - 2}" '
                       'stroke="#3f51b5" stroke-width="2.5" marker-end="url(#fw)"/>')
        y += BOX_H + GAP
    out.append("</svg>")
    return "\n".join(out) + "\n"


def drawn(guide):
    """The picture guide 21 draws, or None after saying why it cannot be drawn."""
    try:
        return render(open(guide, encoding="utf-8").read())
    except (OSError, FigureError) as e:
        print(f"first_week: FAIL — {e}", file=sys.stderr)
        return None


def check(guide=GUIDE, out=OUT):
    svg = drawn(guide)
    if svg is None:
        return 1
    have = open(out, encoding="utf-8").read() if os.path.exists(out) else ""
    if have != svg:
        print("first_week: FAIL — docs/assets/first-week.svg no longer matches guide 21's "
              f"'{HEADING[3:]}' — run: python3 tools/docs_figures/first_week.py", file=sys.stderr)
        return 1
    print("first_week: OK — the front page's picture matches guide 21's order")
    return 0


def write(guide=GUIDE, out=OUT):
    svg = drawn(guide)
    if svg is None:
        return 1
    with open(out, "w", encoding="utf-8") as f:
        f.write(svg)
    print("wrote", out)
    return 0


# A scratch guide 21: an intro line, three steps (one with two programs and slots), then a later
# section whose numbered list is not part of the order.
SCRATCH = """# 21

## The order on the robot

Do these in order.

1. **Read the Devices screen** (no program needed).
2. **Bench Tests** (slot 3): the census. **Calibrate** (slot 4): the drive.
3. **Measure the cascade** at station 13, then **type** it.

## The next section

1. **Not a step** of the order.
"""


def self_test(plant=False):
    cases = [("a planted failing case", False)] if plant else []

    def listed(text):
        try:
            return steps(text)
        except FigureError:
            return None

    cases.append(("each step's name and slots, and nothing from the next section", listed(SCRATCH) == [
        ("Read the Devices screen", []), ("Bench Tests and Calibrate", ["3", "4"]), ("Measure the cascade", [])]))
    with tempfile.TemporaryDirectory(prefix="shulib-first-week-") as d:
        guide, out = os.path.join(d, "21.md"), os.path.join(d, "first-week.svg")

        def put(path, text):
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)

        def run(command):
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                return command(guide, out)

        put(guide, SCRATCH)
        cases.append(("writing the picture, then checking it, passes", run(write) == 0 and run(check) == 0))
        good = open(out, encoding="utf-8").read() if os.path.exists(out) else ""
        cases.append(("the picture names every step and its slots",
                      all(s in good for s in ("Read the Devices screen", "Bench Tests and Calibrate",
                                              "slots 3 and 4", "Measure the cascade"))))
        put(guide, SCRATCH.replace("**Measure the cascade**", "**Measure it**"))
        cases.append(("a renamed step fails", run(check) == 1))
        put(guide, SCRATCH)
        put(out, good + " ")
        cases.append(("a stale picture fails", run(check) == 1))
        os.remove(out)
        cases.append(("a missing picture fails", run(check) == 1))
        put(out, good)
        put(guide, SCRATCH.replace("3. **Measure the cascade** at station 13, then **type** it.",
                                   "3. Measure the cascade at station 13."))
        cases.append(("a step with no bold name fails", run(check) == 1))
        put(guide, SCRATCH.replace(HEADING, "## Another order"))
        cases.append(("a guide without the heading fails", run(check) == 1))
        put(guide, re.sub(r"(?m)^\d\. ", "- ", SCRATCH))
        cases.append(("a list with no numbered step fails", run(check) == 1))
        os.remove(guide)
        cases.append(("a missing guide fails", run(check) == 1))
    for name, ok in cases:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if any(not ok for _n, ok in cases):
        return 1
    print(f"first_week: self-test OK — {len(cases)} cases")
    return 0


def main(argv):
    if argv[1:] == ["check"]:
        return check()
    if argv[1:] == ["self-test"]:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            planted = self_test(plant=True)
        if planted != 1:  # the exit itself is checked: a run with a failing case must exit 1
            print("first_week: self-test FAIL — a run with a planted failing case exits 0", file=sys.stderr)
            return 1
        return self_test()
    if argv[1:]:
        print(__doc__, file=sys.stderr)
        return 2
    return write()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
