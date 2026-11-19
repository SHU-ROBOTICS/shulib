#!/usr/bin/env python3
"""Draws docs/assets/cascade-controls.svg: the builder's controller map of 2026-11-15, the SAME on both robots
(src/team/robot_one_driver.hpp; robot two's file is its twin since chunk ROB1-BT), on a drawing of a V5
controller. Each label is the driver files' own `control words` header, shortened. The map is the team's to change
(the guardrail G4) — when it changes, change MAP below and regenerate:

    python3 tools/docs_figures/controller_map.py

The controller is a schematic (sticks, d-pad, A/B/X/Y, the four shoulder buttons), not a photograph."""
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                   'docs/assets/cascade-controls.svg')
W, H = 860, 470

INK = '#1f2328'
BODY = '#3a3f47'
FACE = '#4b515b'
KEY = '#d9dde3'
CASCADE = '#2a5fb8'   # the cascade: blue
PISTON = '#8a3fb8'    # the pistons: purple
TOGGLE = '#b8622a'    # the side toggles: orange
DRIVE = '#1e7d3a'     # the drive: green
FREE = '#6b7280'      # free: grey

# The controller's parts (centre x, centre y).
LSTICK, RSTICK = (330, 215), (530, 215)
DPAD, ABXY = (345, 305), (515, 305)
L_SHOULDER, R_SHOULDER = (318, 128), (542, 128)

# (label lines, colour, the point the leader line ends at, the label's anchor x, y, text-anchor)
MAP = [
    (['L1 held: the RIGHT side toggle runs', 'L2 held: the LEFT side toggle runs', '(one way; let go: it stops)'],
     TOGGLE, (300, 124), 210, 62, 'end'),
    (['left stick up / down: DRIVE', 'forward / back (left / right: nothing,', 'a tank cannot strafe)'],
     DRIVE, (310, 205), 210, 172, 'end'),
    (['d-pad: FREE (the cascade\'s', 'preset heights go here later)'],
     FREE, (330, 305), 210, 290, 'end'),
    (['R1 / R2 held: the cascade UP / DOWN', 'let go: it holds (before it is homed,', 'at most its unhomed volts)'],
     CASCADE, (560, 124), 650, 62, 'start'),
    (['right stick left / right: TURN'], DRIVE, (550, 205), 650, 182, 'start'),
    (['A: the claw   B: the front toggle', 'X: the ramp — each press flips it', '(they start where they rest)'],
     PISTON, (515, 283), 650, 270, 'start'),
    (['Y: field-centric re-zero', '(field-centric is off)'], FREE, (493, 305), 650, 360, 'start'),
]


def esc(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def main():
    h = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
         'font-family="Helvetica, Arial, sans-serif" role="img" aria-label="The controller map of both '
         'robots\' driver files">',
         '<title>Both robots\' controls (the builder, 2026-11-15): left stick drives, right stick turns; R1 / R2 held '
         'move the cascade up / down; A the claw, B the front toggle, X the ramp; L1 held runs the right side toggle, '
         'L2 the left; Y is the field-centric re-zero (off); the d-pad is free.</title>',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
    # shoulder buttons (behind the body)
    for (cx, cy), names in ((L_SHOULDER, ('L2', 'L1')), (R_SHOULDER, ('R2', 'R1'))):
        h.append(f'<rect x="{cx - 34}" y="{cy - 26}" width="68" height="16" rx="6" fill="{KEY}" stroke="{INK}"/>')
        h.append(f'<rect x="{cx - 38}" y="{cy - 10}" width="76" height="18" rx="6" fill="{KEY}" stroke="{INK}"/>')
        h.append(f'<text x="{cx}" y="{cy - 14}" text-anchor="middle" font-size="11" font-weight="bold" '
                 f'fill="{INK}">{names[0]}</text>')
        h.append(f'<text x="{cx}" y="{cy + 3}" text-anchor="middle" font-size="11" font-weight="bold" '
                 f'fill="{INK}">{names[1]}</text>')
    # the body: two grips and a middle
    h.append(f'<path d="M262,138 L598,138 Q640,138 646,190 L662,340 Q668,392 620,392 Q585,392 570,352 L555,330 '
             f'L305,330 L290,352 Q275,392 240,392 Q192,392 198,340 L214,190 Q220,138 262,138 z" fill="{BODY}"/>')
    h.append(f'<rect x="395" y="160" width="70" height="44" rx="6" fill="#20242a" stroke="#6b7280"/>')
    h.append(f'<text x="430" y="186" text-anchor="middle" font-size="10" fill="#9aa3ad">screen</text>')
    # sticks
    for cx, cy in (LSTICK, RSTICK):
        h.append(f'<circle cx="{cx}" cy="{cy}" r="30" fill="{FACE}" stroke="#20242a" stroke-width="2"/>')
        h.append(f'<circle cx="{cx}" cy="{cy}" r="17" fill="#2b2f35" stroke="#9aa3ad"/>')
    # d-pad
    cx, cy = DPAD
    for dx, dy, lab in ((0, -22, 'Up'), (0, 22, 'Down'), (-22, 0, 'Left'), (22, 0, 'Right')):
        h.append(f'<rect x="{cx + dx - 11}" y="{cy + dy - 11}" width="22" height="22" rx="4" fill="{KEY}" '
                 f'stroke="{FREE}" stroke-width="2.5"/>')
    h.append(f'<text x="{cx}" y="{cy - 18}" text-anchor="middle" font-size="9" fill="{INK}">▲</text>')
    h.append(f'<text x="{cx}" y="{cy + 26}" text-anchor="middle" font-size="9" fill="{INK}">▼</text>')
    h.append(f'<text x="{cx - 22}" y="{cy + 4}" text-anchor="middle" font-size="9" fill="{INK}">◀</text>')
    h.append(f'<text x="{cx + 22}" y="{cy + 4}" text-anchor="middle" font-size="9" fill="{INK}">▶</text>')
    # A B X Y (X top, A right, B bottom, Y left)
    cx, cy = ABXY
    for dx, dy, lab, col in ((0, -22, 'X', PISTON), (22, 0, 'A', PISTON), (0, 22, 'B', PISTON), (-22, 0, 'Y', FREE)):
        h.append(f'<circle cx="{cx + dx}" cy="{cy + dy}" r="11" fill="{KEY}" stroke="{col}" stroke-width="2.5"/>')
        h.append(f'<text x="{cx + dx}" y="{cy + dy + 4}" text-anchor="middle" font-size="11" font-weight="bold" '
                 f'fill="{INK}">{lab}</text>')
    # labels and their leader lines
    for lines, col, (px, py), lx, ly, anchor in MAP:
        ex = lx + (8 if anchor == 'end' else -8)
        h.append(f'<polyline points="{ex},{ly - 4} {px},{py}" fill="none" stroke="{col}" stroke-width="1.6"/>')
        h.append(f'<circle cx="{px}" cy="{py}" r="3.5" fill="{col}"/>')
        for i, line in enumerate(lines):
            weight = ' font-weight="bold"' if i == 0 else ''
            h.append(f'<text x="{lx}" y="{ly + 16 * i}" text-anchor="{anchor}" font-size="13"{weight} '
                     f'fill="{col if i == 0 else INK}">{esc(line)}</text>')
    h.append(f'<text x="{W / 2}" y="{H - 40}" text-anchor="middle" font-size="13" fill="{INK}">'
             'The first time driver control starts in a run, the cascade homes (down, at most 3 s).</text>')
    h.append(f'<text x="{W / 2}" y="{H - 20}" text-anchor="middle" font-size="13" fill="{INK}">Let go of the '
             'sticks and the drive coasts. The map is the team\'s to change: '
             '<tspan font-family="DejaVu Sans Mono, Menlo, Consolas, monospace">src/team/robot_one_driver.hpp'
             '</tspan>.</text>')
    h.append('</svg>')
    with open(OUT, 'w', encoding='utf-8') as f:
        f.write('\n'.join(h) + '\n')
    print('wrote', OUT)


if __name__ == '__main__':
    main()
