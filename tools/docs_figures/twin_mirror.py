#!/usr/bin/env python3
"""Draws docs/assets/twin-mirror.png: the REAL Override field (the game manual's top view) in shulib's frame — (0, 0) at
the centre, +X to the right, +Y away from the red alliance station, inches — with the field's skills DIAGONAL drawn (the
line y = -x, from the corner of the red and right walls to the corner of the blue and left walls — competition::
override2026::kSkillsMirror) and ONE EXAMPLE LEG twice: as robot one runs it
(the numbers as written) and as robot two runs the same function MIRRORED across that line (every point (x, y) becomes
(-y, -x), every heading h becomes -90 - h degrees) — and, at its end, what robot one's RIGHT side toggle reaches,
mirrored: on robot two's LEFT (a mirror swaps a robot's sides). The leg is an illustration of the reflection, not the team's route
(the route is the students' — src/team/skills_route.cpp; chunk ROB1-BT). Regenerate after any change:

    python3 tools/docs_figures/twin_mirror.py

The source image and its scale are tools/docs_figures/field_coordinates.py's (one 24 in tile = 180 px in the source;
the centre goal at (565, 570) there), turned the same way: the red wall comes to the bottom.
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'tools/docs_figures/source/override-field-top.png')
OUT = os.path.join(ROOT, 'docs/assets/twin-mirror.png')
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FONT_REG = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

PX_PER_IN = 7.5
SRC_CENTRE = (565, 570)          # the centre goal in the source image
OUT_WIDTH = 760                  # the published width

ONE = (42, 95, 184, 255)         # robot one: blue
TWO = (196, 98, 16, 255)         # robot two: orange
LINE = (122, 61, 184, 255)       # the mirror line: purple


def mirror(x, y):
    """The reflection across y = -x: (x, y) -> (-y, -x)."""
    return -y, -x


def mirror_heading(h):
    """A heading reflected across y = -x: -90 - h degrees (a reflection reverses turning)."""
    return (-90.0 - h) % 360.0


def main():
    src = Image.open(SRC).convert('RGB')
    w, _h = src.size
    field = src.rotate(90, expand=True)              # counterclockwise: the red wall (left) comes to the bottom
    cx, cy = SRC_CENTRE[1], (w - 1) - SRC_CENTRE[0]  # the centre after the turn
    k = OUT_WIDTH / field.size[0]
    field = field.resize((OUT_WIDTH, round(field.size[1] * k)), Image.LANCZOS)
    cx, cy, s = cx * k, cy * k, PX_PER_IN * k

    band = 46
    img = Image.new('RGB', (OUT_WIDTH, field.size[1] + 2 * band + 76), 'white')
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

    # THE MIRROR LINE, y = -x, dashed, corner to corner.
    for t in range(-72, 72, 6):
        d.line([at(t, -t), at(t + 3.5, -(t + 3.5))], fill=(255, 255, 255, 255), width=8)
        d.line([at(t, -t), at(t + 3.5, -(t + 3.5))], fill=LINE, width=4)

    def label(x, y, lines, dx, dy, colour):
        tx, ty = at(x, y)
        tx += dx
        ty += dy
        widths = [d.textlength(t, font=mid) for t in lines]
        pad = 5
        d.rounded_rectangle([tx - pad, ty - pad, tx + max(widths) + pad, ty + 19 * len(lines) + pad - 2],
                            radius=6, fill=(255, 255, 255, 230), outline=colour, width=2)
        for i, t in enumerate(lines):
            d.text((tx, ty + 19 * i), t, font=mid, fill=colour[:3])

    def leg(start, end, heading, colour):
        """A start pose (a dot and a heading tick) and a leg to `end` (an arrow)."""
        (x0, y0), (x1, y1) = start, end
        d.line([at(x0, y0), at(x1, y1)], fill=(255, 255, 255, 255), width=10)
        d.line([at(x0, y0), at(x1, y1)], fill=colour, width=6)
        ex, ey = at(x1, y1)
        ang = math.atan2(-(y1 - y0), x1 - x0)        # picture y grows downward
        tip = 18
        pts = [(ex + tip * math.cos(ang), ey + tip * math.sin(ang)),
               (ex + 10 * math.cos(ang + 2.2), ey + 10 * math.sin(ang + 2.2)),
               (ex + 10 * math.cos(ang - 2.2), ey + 10 * math.sin(ang - 2.2))]
        d.polygon(pts, fill=colour, outline=(255, 255, 255, 255))
        px, py = at(x0, y0)
        d.ellipse([px - 11, py - 11, px + 11, py + 11], fill=colour, outline=(255, 255, 255, 255), width=3)
        hr = math.radians(heading)
        d.line([(px, py), (px + 22 * math.cos(hr), py - 22 * math.sin(hr))], fill=(255, 255, 255, 255), width=4)

    # ONE EXAMPLE LEG: robot one starts at (-36, -60) facing 90 deg (toward blue) and drives to (-36, -24).
    one_start, one_end, one_heading = (-36.0, -60.0), (-36.0, -24.0), 90.0
    two_start, two_end, two_heading = mirror(*one_start), mirror(*one_end), mirror_heading(one_heading)
    leg(one_start, one_end, one_heading, ONE)
    leg(two_start, two_end, two_heading, TWO)
    label(*one_start, ['robot one: the route', 'AS WRITTEN', '(-36, -60), facing 90'], 16, -6, ONE)
    label(*two_start, ['robot two: the SAME', 'function, MIRRORED', '(60, 36), facing 180'], -150, 18, TWO)
    label(30, -30, ['the mirror line', 'y = -x'], 16, 2, LINE)

    # A MIRROR SWAPS THE ROBOT'S SIDES: a target 8 in to robot one's RIGHT at the leg's end (facing +y, its
    # right is +x) is, mirrored, 8 in to robot two's LEFT (facing 180, its left is -y).
    def target(robot_at, at_xy, colour, text, dx, dy):
        d.line([at(*robot_at), at(*at_xy)], fill=colour, width=3)
        tx, ty = at(*at_xy)
        d.rectangle([tx - 7, ty - 7, tx + 7, ty + 7], fill=(250, 204, 21, 255), outline=colour, width=3)
        label(at_xy[0], at_xy[1], text, dx, dy, colour)
    one_target = (one_end[0] + 8.0, one_end[1])
    target(one_end, one_target, ONE, ['its RIGHT toggle'], 14, -10)
    target(two_end, mirror(*one_target), TWO, ['its LEFT toggle'], -140, 4)

    foot = band * 2 + field.size[1] + 2
    centred('An EXAMPLE leg, not the team\'s route. Each point (x, y) of robot one\'s route is (-y, -x) on robot two,',
            foot, small, (70, 70, 70))
    centred('and each heading h is -90 - h. A mirror swaps sides: robot one\'s RIGHT toggle\'s job is robot two\'s LEFT one.',
            foot + 18, small, (70, 70, 70))
    centred('Seen from above, from behind the red station; inches, 24 in a tile.', foot + 36, small, (70, 70, 70))
    centred('Field image: VEX V5 Robotics Competition "Override" game manual. Line, legs and labels: shulib.', foot + 54,
            small, (110, 110, 110))
    img.save(OUT, optimize=True)
    print('wrote', OUT, img.size)


if __name__ == '__main__':
    main()
