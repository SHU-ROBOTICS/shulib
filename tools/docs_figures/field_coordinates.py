#!/usr/bin/env python3
"""Draws docs/assets/field-coordinates.png: the REAL Override field (the game manual's top view) turned so the red
alliance station is at the bottom — shulib's frame: (0, 0) at the centre, +X to the right, +Y away from the red station,
inches — with the axes and four example points drawn over it. Regenerate after any change:

    python3 tools/docs_figures/field_coordinates.py

The source image is tools/docs_figures/source/override-field-top.png (VEX V5 Robotics Competition "Override" game manual, the
skills field's top view). Its red-outlined Loaders stand on the RED wall (the image's left edge), so the image is turned
90 degrees counterclockwise. Scale, measured on the source: one 24 in tile = 180 px (7.5 px per inch); the centre goal,
at the field's centre, is at (565, 570) in the source.
"""
import os
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'tools/docs_figures/source/override-field-top.png')
OUT = os.path.join(ROOT, 'docs/assets/field-coordinates.png')
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FONT_REG = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

PX_PER_IN = 7.5
SRC_CENTRE = (565, 570)          # the centre goal in the source image
OUT_WIDTH = 760                  # the published width


def main():
    src = Image.open(SRC).convert('RGB')
    w, h = src.size
    field = src.rotate(90, expand=True)              # counterclockwise: the red wall (left) comes to the bottom
    cx, cy = SRC_CENTRE[1], (w - 1) - SRC_CENTRE[0]  # the centre after the turn
    k = OUT_WIDTH / field.size[0]
    field = field.resize((OUT_WIDTH, round(field.size[1] * k)), Image.LANCZOS)
    cx, cy, s = cx * k, cy * k, PX_PER_IN * k

    band = 46
    img = Image.new('RGB', (OUT_WIDTH, field.size[1] + 2 * band + 40), 'white')
    img.paste(field, (0, band))
    d = ImageDraw.Draw(img, 'RGBA')
    big = ImageFont.truetype(FONT, 18)
    mid = ImageFont.truetype(FONT, 15)
    small = ImageFont.truetype(FONT_REG, 13)

    def at(x, y):                                    # field inches -> picture pixels
        return cx + x * s, band + cy - y * s

    def centred(text, y, font, fill):
        tw = d.textlength(text, font=font)
        d.text(((OUT_WIDTH - tw) / 2, y), text, font=font, fill=fill)

    centred('BLUE alliance station', 12, big, (42, 95, 184))
    centred('RED alliance station  (you stand here)', band + field.size[1] + 12, big, (184, 44, 44))

    def arrow(x0, y0, x1, y1, colour, width=6):
        d.line([at(x0, y0), at(x1, y1)], fill=colour, width=width)
        ax, ay = at(x1, y1)
        if x1 != x0:                                 # horizontal arrowhead
            d.polygon([(ax + 16, ay), (ax, ay - 10), (ax, ay + 10)], fill=colour)
        else:                                        # vertical arrowhead
            d.polygon([(ax, ay - 16), (ax - 10, ay), (ax + 10, ay)], fill=colour)

    white = (255, 255, 255, 255)
    arrow(0, 0, 22, 0, white, 9); arrow(0, 0, 0, 22, white, 9)
    black = (20, 20, 20, 255)
    arrow(0, 0, 22, 0, black, 5); arrow(0, 0, 0, 22, black, 5)

    def label(x, y, lines, dx, dy):
        tx, ty = at(x, y)
        tx += dx; ty += dy
        widths = [d.textlength(t, font=mid) for t in lines]
        pad = 5
        d.rounded_rectangle([tx - pad, ty - pad, tx + max(widths) + pad, ty + 19 * len(lines) + pad - 2],
                            radius=6, fill=(255, 255, 255, 225), outline=(122, 61, 184, 255), width=2)
        for i, t in enumerate(lines):
            d.text((tx, ty + 19 * i), t, font=mid, fill=(75, 34, 117))

    ax_x, ax_y = at(22, 0); d.text((ax_x + 20, ax_y - 12), '+X', font=big, fill=black, stroke_width=3, stroke_fill='white')
    ay_x, ay_y = at(0, 22); d.text((ay_x - 40, ay_y - 22), '+Y', font=big, fill=black, stroke_width=3, stroke_fill='white')

    # dashed guides to (24, 36)
    for a in range(0, 24, 3):
        d.line([at(a, 36), at(a + 1.6, 36)], fill=(122, 61, 184, 255), width=3)
    for b in range(0, 36, 3):
        d.line([at(24, b), at(24, b + 1.6)], fill=(122, 61, 184, 255), width=3)

    points = [((0, 0), ['(0, 0)', 'the centre'], -128, 14),
              ((24, 36), ['(24, 36)', '24 in right,', '36 in toward blue'], 14, -30),
              ((48, 0), ['(48, 0)', 'two tiles right'], -40, 16),
              ((-48, -24), ['(-48, -24)', 'two tiles left,', 'one toward red'], 14, 10)]
    for (x, y), lines, dx, dy in points:
        px, py = at(x, y)
        d.ellipse([px - 9, py - 9, px + 9, py + 9], fill=(122, 61, 184, 255), outline=white, width=3)
        label(x, y, lines, dx, dy)

    foot = band * 2 + field.size[1] + 2
    centred('Seen from above, from behind the red alliance station. One tile is 24 in; every position is in inches.', foot, small, (70, 70, 70))
    centred('Field image: VEX V5 Robotics Competition "Override" game manual. Axes and points: shulib.', foot + 18, small, (110, 110, 110))
    img.save(OUT, optimize=True)
    print('wrote', OUT, img.size)


if __name__ == '__main__':
    main()
