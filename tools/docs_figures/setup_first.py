#!/usr/bin/env python3
"""Draws docs/assets/setup-first.svg, guide 23's picture of its five steps, FROM the page's own numbered
steps — so the picture and the page cannot disagree.

  python3 tools/docs_figures/setup_first.py            write the picture
  python3 tools/docs_figures/setup_first.py check      fail if the picture no longer matches guide 23
  python3 tools/docs_figures/setup_first.py self-test  the check, shown failing on scratch files

A step is a heading "## N. Title" (numbered 1, 2, 3 … in order) whose first line below is one line in
italics, "*Slots 3 and 4, from the laptop*" — the box's small words. The last step is drawn green: it is
where the page leads.
"""
import contextlib
import io
import os
import re
import sys
import tempfile
from xml.sax.saxutils import escape

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GUIDE = os.path.join(REPO, "docs/guide/23-set-up-your-robot-first.md")
OUT = os.path.join(REPO, "docs/assets/setup-first.svg")
STEP = re.compile(r"^## (\d+)\. (\S.*)$")
CAPTION = re.compile(r"^\*([^*]+)\*$")
W, BOX_H, GAP, TOP, LINE = 560, 60, 26, 48, 18
MAX_TITLE, MAX_CAPTION = 44, 64  # what fits in a box (16 px bold, 13 px) with room for a wider font
BLUE, GREEN = ("#e8eaf6", "#3f51b5"), ("#e8f5e9", "#2e7d32")


class FigureError(Exception):
    """Guide 23's steps cannot be drawn."""


def steps(text):
    """[(title, caption)] for each '## N. Title' heading, in order."""
    lines = text.splitlines()
    fenced = False
    out = []
    for i, line in enumerate(lines):
        if line.lstrip().startswith("```"):
            fenced = not fenced
            continue
        m = None if fenced else STEP.match(line)
        if not m:
            continue
        if int(m.group(1)) != len(out) + 1:
            raise FigureError(f"step '{line}' is numbered {m.group(1)}, expected {len(out) + 1}")
        below = next((l.strip() for l in lines[i + 1:] if l.strip()), "")
        cap = CAPTION.match(below)
        if not cap:
            raise FigureError(f"step {m.group(1)} has no one-line *caption* under its heading")
        title, caption = m.group(2).strip(), cap.group(1).strip()
        if len(title) > MAX_TITLE or len(caption) > MAX_CAPTION:
            raise FigureError(f"step {m.group(1)}'s title (at most {MAX_TITLE} characters) or caption (at most "
                              f"{MAX_CAPTION}) is too long for its box")
        out.append((title, caption))
    if len(out) < 2:
        raise FigureError("guide 23 has fewer than two numbered steps ('## 1. …')")
    return out


def render(text):
    st = steps(text)
    h = TOP + len(st) * BOX_H + (len(st) - 1) * GAP + 16
    names = "; then ".join(f"{i} {t}" for i, (t, _c) in enumerate(st, 1))
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {h}" width="{W}" height="{h}" '
           f'font-family="Helvetica, Arial, sans-serif" role="img" aria-label="Set up your robot first">',
           f"<title>Set up your robot first: {escape(names)}</title>",
           '<defs><marker id="sf" markerUnits="userSpaceOnUse" markerWidth="12" markerHeight="10" refX="11" '
           'refY="5" orient="auto"><path d="M0,0 L12,5 L0,10 z" fill="#3f51b5"/></marker></defs>',
           f'<rect width="{W}" height="{h}" fill="#ffffff"/>',
           f'<text x="{W // 2}" y="28" text-anchor="middle" font-size="17" font-weight="bold" fill="#1a237e">'
           "Before the route builder, in this order</text>"]
    y = TOP
    for i, (title, caption) in enumerate(st, 1):
        fill, stroke = GREEN if i == len(st) else BLUE
        out.append(f'<rect x="12" y="{y}" width="{W - 24}" height="{BOX_H}" rx="10" fill="{fill}" '
                   f'stroke="{stroke}" stroke-width="1.5"/>')
        out.append(f'<circle cx="46" cy="{y + BOX_H // 2}" r="17" fill="{stroke}"/>')
        out.append(f'<text x="46" y="{y + BOX_H // 2 + 6}" text-anchor="middle" font-size="17" '
                   f'font-weight="bold" fill="#ffffff">{i}</text>')
        out.append(f'<text x="78" y="{y + 26}" font-size="16" font-weight="bold" fill="#111111">'
                   f"{escape(title)}</text>")
        out.append(f'<text x="78" y="{y + 26 + LINE + 1}" font-size="13" fill="{stroke}">{escape(caption)}</text>')
        if i < len(st):
            out.append(f'<line x1="46" y1="{y + BOX_H + 2}" x2="46" y2="{y + BOX_H + GAP - 2}" '
                       'stroke="#3f51b5" stroke-width="2.5" marker-end="url(#sf)"/>')
        y += BOX_H + GAP
    out.append("</svg>")
    return "\n".join(out) + "\n"


def drawn(guide):
    """The picture guide 23 draws, or None after saying why it cannot be drawn."""
    try:
        return render(open(guide, encoding="utf-8").read())
    except (OSError, FigureError) as e:
        print(f"setup_first: FAIL — {e}", file=sys.stderr)
        return None


def check(guide=GUIDE, out=OUT):
    svg = drawn(guide)
    if svg is None:
        return 1
    have = open(out, encoding="utf-8").read() if os.path.exists(out) else ""
    if have != svg:
        print("setup_first: FAIL — docs/assets/setup-first.svg no longer matches guide 23's steps "
              "— run: python3 tools/docs_figures/setup_first.py", file=sys.stderr)
        return 1
    print("setup_first: OK — the picture matches guide 23's steps")
    return 0


def write(guide=GUIDE, out=OUT):
    svg = drawn(guide)
    if svg is None:
        return 1
    with open(out, "w", encoding="utf-8") as f:
        f.write(svg)
    print("wrote", out)
    return 0


# A scratch guide 23: an unnumbered section, three steps with their captions, a "## 9." heading inside a
# code block (not a step), and an unnumbered section after.
SCRATCH = """# 23

## What you need

*not a step*

## 1. Upload them

*Slots 3 and 4*

Text.

```text
## 9. Not a step
```

## 2. Check every part

*Slot 3*

## 3. Now make your route

*The route builder*

## If something goes wrong

| a | b |
"""


def self_test(plant=False):
    cases = [("a planted failing case", False)] if plant else []

    def listed(text):
        try:
            return steps(text)
        except FigureError:
            return None

    cases.append(("each step's title and caption, in order, and nothing from a code block", listed(SCRATCH) == [
        ("Upload them", "Slots 3 and 4"), ("Check every part", "Slot 3"),
        ("Now make your route", "The route builder")]))
    with tempfile.TemporaryDirectory(prefix="shulib-setup-first-") as d:
        guide, out = os.path.join(d, "23.md"), os.path.join(d, "setup-first.svg")

        def put(path, text):
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)

        def run(command):
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                return command(guide, out)

        put(guide, SCRATCH)
        cases.append(("writing the picture, then checking it, passes", run(write) == 0 and run(check) == 0))
        good = open(out, encoding="utf-8").read() if os.path.exists(out) else ""
        cases.append(("the picture names every step and caption", all(s in good for s in (
            "Upload them", "Slots 3 and 4", "Check every part", "Slot 3", "Now make your route",
            "The route builder"))))
        cases.append(("only the last step is green", good.count("#e8f5e9") == 1
                      and good.index("#e8f5e9") > good.index("Check every part")))
        put(guide, SCRATCH.replace("## 2. Check every part", "## 2. Check each part"))
        cases.append(("a renamed step fails", run(check) == 1))
        put(guide, SCRATCH.replace("*Slot 3*", "*Slot 4*"))
        cases.append(("a changed caption fails", run(check) == 1))
        put(guide, SCRATCH)
        put(out, good + " ")
        cases.append(("a stale picture fails", run(check) == 1))
        os.remove(out)
        cases.append(("a missing picture fails", run(check) == 1))
        put(out, good)
        put(guide, SCRATCH.replace("## 2. Check every part\n\n*Slot 3*\n", "## 2. Check every part\n\nSlot 3.\n"))
        cases.append(("a step with no caption fails", run(check) == 1))
        put(guide, SCRATCH.replace("*Slot 3*", "*" + "x" * (MAX_CAPTION + 1) + "*"))
        cases.append(("a caption too long for its box fails", run(check) == 1))
        put(guide, SCRATCH.replace("## 2. Check every part", "## 2. " + "x" * (MAX_TITLE + 1)))
        cases.append(("a title too long for its box fails", run(check) == 1))
        put(guide, SCRATCH.replace("## 3. Now make", "## 4. Now make"))
        cases.append(("a skipped number fails", run(check) == 1))
        put(guide, SCRATCH.replace("## 2. Check every part\n\n*Slot 3*\n\n", ""))
        cases.append(("a removed step fails", run(check) == 1))
        put(guide, re.sub(r"(?m)^## \d+\. ", "## ", SCRATCH))
        cases.append(("a page with no numbered step fails", run(check) == 1))
        os.remove(guide)
        cases.append(("a missing guide fails", run(check) == 1))
    for name, ok in cases:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if any(not ok for _n, ok in cases):
        return 1
    print(f"setup_first: self-test OK — {len(cases)} cases")
    return 0


def main(argv):
    if argv[1:] == ["check"]:
        return check()
    if argv[1:] == ["self-test"]:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            planted = self_test(plant=True)
        if planted != 1:  # the exit itself is checked: a run with a failing case must exit 1
            print("setup_first: self-test FAIL — a run with a planted failing case exits 0", file=sys.stderr)
            return 1
        return self_test()
    if argv[1:]:
        print(__doc__, file=sys.stderr)
        return 2
    return write()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
