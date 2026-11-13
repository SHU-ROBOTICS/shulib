#!/usr/bin/env python3
"""Draws docs/assets/field-heading.svg: shulib's heading convention — 0 degrees along +X (right, as seen from the red
alliance station), angles growing counterclockwise. Regenerate: python3 tools/docs_figures/field_heading.py"""
import math, os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'docs/assets/field-heading.svg')
W, H, CX, CY, R = 360, 330, 180, 165, 95


def main():
    h = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="Helvetica, Arial, sans-serif" role="img" aria-label="Heading angles: 0 degrees along +X, increasing counterclockwise">',
         '<title>Heading: 0° points along +X (right), angles grow counterclockwise: 90° along +Y, 180° along −X, 270° (or −90°) along −Y</title>',
         '<defs><marker id="ah2" markerUnits="userSpaceOnUse" markerWidth="14" markerHeight="12" refX="13" refY="6" orient="auto"><path d="M0,0 L14,6 L0,12 z" fill="#222"/></marker>'
         '<marker id="ah3" markerUnits="userSpaceOnUse" markerWidth="14" markerHeight="12" refX="13" refY="6" orient="auto"><path d="M0,0 L14,6 L0,12 z" fill="#7a3db8"/></marker></defs>',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
         f'<circle cx="{CX}" cy="{CY}" r="{R}" fill="#eef1f4" stroke="#c5ccd3"/>']
    for ang, lab in ((0, '0°  (+X, right)'), (90, '90°  (+Y, toward blue)'), (180, '180°  (−X)'), (270, '270° = −90°  (−Y, toward red)')):
        a = math.radians(ang)
        h.append(f'<line x1="{CX}" y1="{CY}" x2="{CX + R * math.cos(a):.1f}" y2="{CY - R * math.sin(a):.1f}" stroke="#222" stroke-width="2.5" marker-end="url(#ah2)"/>')
        tx, ty = CX + (R + 12) * math.cos(a), CY - (R + 12) * math.sin(a) + 4
        anchor = 'start' if ang == 0 else ('end' if ang == 180 else 'middle')
        ty += -6 if ang == 90 else (12 if ang == 270 else 0)
        h.append(f'<text x="{tx:.1f}" y="{ty:.1f}" text-anchor="{anchor}" font-size="13" fill="#222">{lab}</text>')
    r2 = 55
    h.append(f'<path d="M{CX + r2},{CY} A{r2},{r2} 0 0 0 {CX},{CY - r2}" fill="none" stroke="#7a3db8" stroke-width="2.5" marker-end="url(#ah3)"/>')
    h.append(f'<text x="{CX + 44}" y="{CY - 44}" font-size="13" fill="#4b2275" font-weight="bold">counter-</text>')
    h.append(f'<text x="{CX + 44}" y="{CY - 30}" font-size="13" fill="#4b2275" font-weight="bold">clockwise</text>')
    h.append('</svg>')
    open(OUT, 'w').write('\n'.join(h) + '\n')
    print('wrote', OUT)


if __name__ == '__main__':
    main()
