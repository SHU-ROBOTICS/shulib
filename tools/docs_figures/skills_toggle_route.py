#!/usr/bin/env python3
"""Draws docs/assets/skills-toggle-look-spots.png: the REAL Override field (the game manual's top view, turned exactly as
field_coordinates.py turns it — the red alliance station at the bottom, shulib's frame F1, inches) with the cookbook's
skills Toggle recipe drawn over it (docs/cookbook/08-skills-toggles.md, compiled and run as cookbook-08a in
test/cookbook_examples_test.cpp): each Toggle in its Quadrant's colour, the route's three LOOK SPOTS in the order it visits
them (the robot's centre, its heading toward the wall, its lens 16 in from the Toggle's face), the push spot 4 in from the
face (a purple ring — the ONLY hollow marker), the straight back-off after each visit before the robot turns (AF1S-A2's
review: after FLIPPER STUCK the robot is still at the push spot), the start and where it backs off to at the end — every
point the robot backs straight off to is a grey x, its words beside it (AF1S-A2's verification round: two hollow dots
meant two things) — and what each visit does on the host test plant. The Quadrant colours are a READING of the manual's drawings (A4 register HA-243; Step 0.7 checks them). Regenerate after any change:

    python3 tools/docs_figures/skills_toggle_route.py

Every number is the recipe's PLACEHOLDER (the team's come from Step 0); nothing here has run on a robot. The lens is 7 in
forward of the robot's centre (the recipe robot's camera mount).
"""
import os
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'tools/docs_figures/source/override-field-top.png')
OUT = os.path.join(ROOT, 'docs/assets/skills-toggle-look-spots.png')
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FONT_REG = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

PX_PER_IN = 7.5                  # field_coordinates.py's scale, measured on the source
SRC_CENTRE = (565, 570)          # the centre goal in the source image
OUT_WIDTH = 760
WALL = 70.2                      # each wall's inner face, inches from the centre (pA10: 140.41 / 2)
LENS_FWD = 7.0                   # the recipe robot's lens, forward of its centre
LOOK_FROM = 16.0                 # the recipe's lookFrom: lens to the face at the look spot
PUSH_FROM = 4.0                  # the recipe's pushFrom

RED = (196, 40, 40, 255)
BLUE = (36, 92, 196, 255)
PURPLE = (122, 61, 184, 255)
INK = (75, 34, 117)
WHITE = (255, 255, 255, 255)


def main():
    src = Image.open(SRC).convert('RGB')
    w, _ = src.size
    field = src.rotate(90, expand=True)
    cx, cy = SRC_CENTRE[1], (w - 1) - SRC_CENTRE[0]
    k = OUT_WIDTH / field.size[0]
    field = field.resize((OUT_WIDTH, round(field.size[1] * k)), Image.LANCZOS)
    cx, cy, s = cx * k, cy * k, PX_PER_IN * k

    band = 46
    img = Image.new('RGB', (OUT_WIDTH, field.size[1] + 2 * band + 94), 'white')
    img.paste(field, (0, band))
    d = ImageDraw.Draw(img, 'RGBA')
    big = ImageFont.truetype(FONT, 18)
    mid = ImageFont.truetype(FONT, 14)
    small = ImageFont.truetype(FONT_REG, 13)

    def at(x, y):
        return cx + x * s, band + cy - y * s

    def centred(text, y, font, fill):
        tw = d.textlength(text, font=font)
        d.text(((OUT_WIDTH - tw) / 2, y), text, font=font, fill=fill)

    centred('BLUE alliance station', 12, big, (42, 95, 184))
    centred('RED alliance station  (you stand here)', band + field.size[1] + 12, big, (184, 44, 44))

    def label(px, py, lines, outline):
        widths = [d.textlength(t, font=mid) for t in lines]
        pad = 5
        d.rounded_rectangle([px - pad, py - pad, px + max(widths) + pad, py + 18 * len(lines) + pad - 2],
                            radius=6, fill=(255, 255, 255, 232), outline=outline, width=2)
        for i, t in enumerate(lines):
            d.text((px, py + 18 * i), t, font=mid, fill=INK)

    # Each Toggle outlined in its Quadrant's colour (vision::override2026::kToggleQuadrantColour, A4 register HA-243).
    toggles = [((WALL, 0.0), (-1, 0), RED), ((0.0, WALL), (0, -1), BLUE), ((-WALL, 0.0), (1, 0), BLUE),
               ((0.0, -WALL), (0, 1), RED)]
    # The Toggle sits ON the wall, so in the picture it lies just outside the wall's inner face: the box spans 2.5 to
    # 7 in beyond it (read off the picture — the yellow bar at each wall's centre), 12.9 in each side of the centre.
    for (x, y), (ix, iy), colour in toggles:
        half = 12.9  # the Toggle's half-length along the wall
        near, far = -2.5, -7.0  # along the inward normal: negative = beyond the inner face
        ax, ay = x + ix * near - iy * half, y + iy * near + ix * half
        bx, by = x + ix * far + iy * half, y + iy * far - ix * half
        p0, p1 = at(min(ax, bx), max(ay, by)), at(max(ax, bx), min(ay, by))
        d.rectangle([p0[0] - 2, p0[1] - 2, p1[0] + 2, p1[1] + 2], outline=colour, width=4)

    def arrow(x0, y0, x1, y1, colour, width=5):
        d.line([at(x0, y0), at(x1, y1)], fill=colour, width=width)
        (ax, ay), (bx, by) = at(x0, y0), at(x1, y1)
        dx, dy = bx - ax, by - ay
        n = (dx * dx + dy * dy) ** 0.5
        ux, uy = dx / n, dy / n
        d.polygon([(bx + ux * 12, by + uy * 12), (bx - uy * 8, by + ux * 8), (bx + uy * 8, by - ux * 8)], fill=colour)

    def dot(x, y, colour, r=9, hollow=False):
        px, py = at(x, y)
        if hollow:
            d.ellipse([px - r, py - r, px + r, py + r], fill=WHITE, outline=colour, width=3)
        else:
            d.ellipse([px - r, py - r, px + r, py + r], fill=colour, outline=WHITE, width=3)
        return px, py

    def cross(x, y, colour, r=7):
        # A back-off point: a grey x on a white halo (never a dot — a hollow dot is the push spot).
        px, py = at(x, y)
        for c, wd in ((WHITE, 7), (colour, 3)):
            d.line([(px - r, py - r), (px + r, py + r)], fill=c, width=wd)
            d.line([(px - r, py + r), (px + r, py - r)], fill=c, width=wd)
        return px, py

    # The three look spots, in the order the recipe visits them: the robot's centre, an arrow along its heading to
    # its lens, a dashed line from the lens to the Toggle's face (16 in).
    spots = [((WALL - LOOK_FROM - LENS_FWD, 0.0), (1, 0), '1'), ((0.0, WALL - LOOK_FROM - LENS_FWD), (0, 1), '2'),
             ((0.0, -(WALL - LOOK_FROM - LENS_FWD)), (0, -1), '3')]
    for (x, y), (hx, hy), n in spots:
        lx, ly = x + hx * LENS_FWD, y + hy * LENS_FWD
        arrow(x, y, lx - hx * 1.5, ly - hy * 1.5, PURPLE, 5)
        for t in range(0, int(LOOK_FROM), 2):
            d.line([at(lx + hx * t, ly + hy * t), at(lx + hx * (t + 1.1), ly + hy * (t + 1.1))], fill=PURPLE, width=3)
        px, py = dot(x, y, PURPLE, 13)
        tw = d.textlength(n, font=big)
        d.text((px - tw / 2, py - 11), n, font=big, fill='white')

    # The push spot on the first visit: the lens 4 in from the face (the step drives straight in to it and out).
    x1 = WALL - PUSH_FROM - LENS_FWD
    dot(x1, 0.0, PURPLE, 7, hollow=True)

    # After visits 1 and 2: a straight back-off along the robot's line BEFORE the turn to the next look spot (the
    # recipe's reverseTo(40, 0) and reverseTo(0, 40)).
    for (x0, y0), (x1b, y1b) in (((WALL - LOOK_FROM - LENS_FWD, 0.0), (40.0, 0.0)),
                                 ((0.0, WALL - LOOK_FROM - LENS_FWD), (0.0, 40.0))):
        arrow(x0, y0, x1b + (x0 - x1b) * 0.45, y1b + (y0 - y1b) * 0.45, (40, 40, 40, 255), 3)
        cross(x1b, y1b, (40, 40, 40, 255))

    # The start and where the route backs off to.
    sx, sy = dot(24.0, 0.0, (40, 40, 40, 255), 9)
    arrow(24.0, 0.0, 33.0, 0.0, (40, 40, 40, 255), 3)
    ex, ey = cross(0.0, -30.0, (40, 40, 40, 255), 8)

    rx, ry = at(WALL - LOOK_FROM - LENS_FWD, 0.0)
    label(rx - 150, ry + 26, ['1  right wall (a RED Quadrant):', 'already RED -- nothing pushed', 'look spot: lens 16 in out;',
                              'purple ring: push spot, 4 in;', 'grey x: back off to (40, 0)'], PURPLE)
    bx, by = at(0.0, WALL - LOOK_FROM - LENS_FWD)
    label(bx + 22, by + 4, ['2  blue wall (a BLUE Quadrant):', 'YELLOW -> BLUE, 2 pushes;', 'grey x: back off to (0, 40)'],
          PURPLE)
    qx, qy = at(0.0, -(WALL - LOOK_FROM - LENS_FWD))
    label(qx + 22, qy - 20, ['3  red wall (a RED Quadrant):', 'sticky -- NO CHANGE,', "skip to 'back off'"], PURPLE)
    label(sx - 60, sy - 70, ['start (24, 0), facing 0 deg:', 'the cascade homes on the way'], (40, 40, 40, 255))
    label(ex - 236, ey - 9, ['grey x: back off to (0, -30)'], (40, 40, 40, 255))

    foot = band * 2 + field.size[1] + 2
    centred("Toggles outlined in their Quadrant's colour (red: the right and red walls; blue: the blue and left walls)",
            foot, small, (70, 70, 70))
    centred("-- a reading of the manual's drawings (HA-243), checked on the event's field at Step 0.7.", foot + 18, small,
            (70, 70, 70))
    centred('Purple ring: the push spot. Grey x: where the robot backs straight off to before it turns.', foot + 36, small,
            (70, 70, 70))
    centred("The cookbook's teaching route (cookbook-08a) on the host test plant.", foot + 54, small, (70, 70, 70))
    centred('Field image: VEX V5 Robotics Competition "Override" game manual. Spots and labels: shulib.', foot + 72, small,
            (110, 110, 110))
    img.save(OUT, optimize=True)
    print('wrote', OUT, img.size)


if __name__ == '__main__':
    main()
