#!/usr/bin/env python3
"""Draws docs/assets/record-names-its-robot.svg: what the boot of a "shulib Drive", "shulib Auton Tester" or "shulib
Competition" does with the calibration record on the SD card in ROBOT ONE's brain, now that the record names its
robot (chunk ROB1-BT): its own record boots; robot two's record (the card moved from the twin) and a record written
before 2026-11-16 (no name) are each refused, by the headline the boot prints (src/common/boot_source.hpp, step 0).
The verdict words are the boot's own (`the card's record is ANOTHER ROBOT'S`, `the card's record names NO ROBOT`).
Regenerate after any change: python3 tools/docs_figures/record_name.py"""
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                   'docs/assets/record-names-its-robot.svg')
W, H = 780, 330
PANEL_W, GAP, TOP = 240, 30, 16

ONE = '#2a5fb8'     # robot one: blue (as docs/assets/twin-mirror.png, chapter 21)
TWO = '#c46210'     # robot two: orange
NONE = '#6b7280'    # no robot: grey
GOOD = '#1e7d3a'    # boots
BAD = '#b3261e'     # refused
INK = '#1f2328'

PANELS = [
    # (heading, card colour, card's first word, card's second line, verdict, verdict colour, verdict lines)
    ('Its own card', ONE, 'robot=robot-one', 'Nov 16 2026 09:00:00 | …', 'BOOTS', GOOD,
     ['then the port check,', 'as before']),
    ("Robot two's card, moved", TWO, 'robot=robot-two', 'Nov 16 2026 10:30:00 | …', 'REFUSED', BAD,
     ["the card's record is", "ANOTHER ROBOT'S"]),
    ('A card from before Nov 16', NONE, 'shulib Calibrate', 'Sep 24 2026 16:05:00 | …', 'REFUSED', BAD,
     ["the card's record", 'names NO ROBOT']),
]


def esc(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def panel(x, heading, card, first, second, verdict, vcol, vlines):
    out = []
    cx = x + PANEL_W / 2
    out.append(f'<text x="{cx}" y="{TOP + 18}" text-anchor="middle" font-size="15" font-weight="bold" '
               f'fill="{INK}">{esc(heading)}</text>')
    # robot one's brain
    by = TOP + 34
    out.append(f'<rect x="{x + 20}" y="{by}" width="{PANEL_W - 40}" height="118" rx="10" fill="#eef1f4" '
               f'stroke="{ONE}" stroke-width="2.5"/>')
    out.append(f'<text x="{cx}" y="{by + 22}" text-anchor="middle" font-size="13" font-weight="bold" '
               f'fill="{ONE}">ROBOT ONE\'S BRAIN</text>')
    # the SD card in it (a card with a clipped corner)
    kx, ky, kw, kh = x + 36, by + 36, PANEL_W - 72, 68
    out.append(f'<path d="M{kx},{ky + 12} L{kx + 12},{ky} L{kx + kw},{ky} L{kx + kw},{ky + kh} L{kx},{ky + kh} z" '
               f'fill="#ffffff" stroke="{card}" stroke-width="2.5"/>')
    out.append(f'<text x="{kx + kw / 2}" y="{ky + 20}" text-anchor="middle" font-size="11" fill="{INK}">'
               f'shulib_drivetrain.bin</text>')
    out.append(f'<text x="{kx + kw / 2}" y="{ky + 40}" text-anchor="middle" font-size="13" font-weight="bold" '
               f'font-family="DejaVu Sans Mono, Menlo, Consolas, monospace" fill="{card}">{esc(first)}</text>')
    out.append(f'<text x="{kx + kw / 2}" y="{ky + 57}" text-anchor="middle" font-size="10" fill="{NONE}">'
               f'{esc(second)}</text>')
    # the arrow down to the verdict
    ay = by + 118
    out.append(f'<line x1="{cx}" y1="{ay + 4}" x2="{cx}" y2="{ay + 34}" stroke="{INK}" stroke-width="2.5" '
               f'marker-end="url(#ah)"/>')
    out.append(f'<text x="{cx + 8}" y="{ay + 24}" font-size="11" fill="{INK}">boot, step 0</text>')
    # the verdict
    vy = ay + 40
    out.append(f'<rect x="{x + 20}" y="{vy}" width="{PANEL_W - 40}" height="78" rx="10" fill="#ffffff" '
               f'stroke="{vcol}" stroke-width="3"/>')
    out.append(f'<text x="{cx}" y="{vy + 25}" text-anchor="middle" font-size="17" font-weight="bold" '
               f'fill="{vcol}">{verdict}</text>')
    for i, line in enumerate(vlines):
        out.append(f'<text x="{cx}" y="{vy + 46 + 16 * i}" text-anchor="middle" font-size="12" fill="{INK}">'
                   f'{esc(line)}</text>')
    return out


def main():
    h = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
         'font-family="Helvetica, Arial, sans-serif" role="img" aria-label="The record names its robot: on robot '
         'one, its own record boots; robot two\'s record and a record that names no robot are refused">',
         '<title>Robot one\'s brain at boot: a record naming robot-one boots; a record naming robot-two is refused '
         '(the card\'s record is ANOTHER ROBOT\'S); a record written before 2026-11-16 names no robot and is '
         'refused (the card\'s record names NO ROBOT). The fix both times: run shulib Calibrate on this robot.</title>',
         f'<defs><marker id="ah" markerUnits="userSpaceOnUse" markerWidth="14" markerHeight="12" refX="13" refY="6" '
         f'orient="auto"><path d="M0,0 L14,6 L0,12 z" fill="{INK}"/></marker></defs>',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
    x0 = (W - 3 * PANEL_W - 2 * GAP) / 2
    for i, p in enumerate(PANELS):
        h += panel(x0 + i * (PANEL_W + GAP), *p)
    h.append(f'<text x="{W / 2}" y="{H - 14}" text-anchor="middle" font-size="13" fill="{INK}">The fix, both times: '
             '<tspan font-weight="bold">run shulib Calibrate (slot 4) on THIS robot</tspan> — its save writes a '
             'record that names it.</text>')
    h.append('</svg>')
    with open(OUT, 'w', encoding='utf-8') as f:
        f.write('\n'.join(h) + '\n')
    print('wrote', OUT)


if __name__ == '__main__':
    main()
