#!/usr/bin/env python3
"""Makes the route builder's field picture: tools/route_maker/assets/field.webp and field.json (chunk RMP1-A1,
2026-11-14; the RMP1 brief [R11], FP1/FP2). Run once after the source image changes, then commit both files:

    python3 tools/docs_figures/route_maker_field.py

The picture is the REAL Override field — the game manual's skills-field top view,
tools/docs_figures/source/override-field-top.png, the same source as docs/assets/field-coordinates.png — turned 90
degrees counterclockwise exactly as tools/docs_figures/field_coordinates.py turns it, so the RED alliance station is
at the bottom and +x is to the right (shulib's frame, guide 2). Nothing is drawn on it: the route builder draws the
robot, the steps and the grid over it in its own SVG layer.

THE TRANSFORM (field.json) is MEASURED on the turned picture, not assumed: the inner edge of the perimeter's top rail
is found on each of the four walls (the brightest pixel of the rail along 300 rows or columns away from the corners
and the Toggles, then 2 px inward to the inner face); the field's centre is their midpoint and the scale is their
span over the field's 140.41 in (2 x localization/override_walls.hpp's kWallInnerFace). field_coordinates.py's
hand-measured 7.5 px per inch and (565, 570) were within about 1.5 in of these walls; the route builder draws a robot
against the walls, so it uses the measured fit. field.json also records the FIVE LANDMARKS of FP1 as measured pixels
— the centre goal's dark body (its centroid, found independently of the walls) and the wall faces at the wall
centres, where the four Toggles stand (vision/override_elements.hpp's kTogglePlaces) — and the first red and the
first blue pixel of the Load Zones' tape along the left wall, each searched from the TOP of the picture (FP2).
tools/route_maker/test/field.test.mjs maps them through the page's own transform and compares them with the C++
positions in the robot description: within 1 in, and red at the bottom.

FP3 — THE SCALE, CHECKED BY SOMETHING THE TRANSFORM WAS NOT MADE FROM. The four wall landmarks ARE the pixels the
transform is fitted to, so they agree with it by construction, and the centre goal pins only the origin: a scale
measured 10 px too far out on every wall (the rail's outer edge) would still pass them. So field.json also records
the TILE SEAMS, measured from the picture's own tiles: the field is six foam tiles across, wall face to wall face
(2 x kWallInnerFace = 140.41 in, so a tile is 23.40 in), and neighbouring tiles are printed in two greys. For each
seam (+-1 and +-2 tiles from the centre, across x and across y) the brightness profile across it is taken in each of
the six tile bands along it (grey tile pixels only — no tape, no field element), and the seam is where the profile
crosses halfway between the two tiles' levels; the seam's pixel is the mean over the bands (the seams' teeth
alternate by ~4 px band to band). The wall transform only says roughly WHERE to look (+-15 px); the crossing itself
is the picture's. The test then holds each seam within 1 in of its tile line and the outer seams' SPAN (93.6 in)
within 1 in — the scale, independently of the walls.

Pillow only (as the docs figures); the route builder's build (tools/route_maker/build.py, stdlib) only inlines the
committed files.
"""
import json
import os
import statistics

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'tools/docs_figures/source/override-field-top.png')
OUT_DIR = os.path.join(ROOT, 'tools/route_maker/assets')
WALL_INNER_FACE_IN = 1783.2 / 25.4  # localization/override_walls.hpp: kWallInnerFace (the walls' inner faces)
QUALITY = 88                         # WebP quality: ~70 KB at full size


def main():
    field = Image.open(SRC).convert('RGB').rotate(90, expand=True)  # counterclockwise: the red wall comes to the bottom
    grey = field.convert('L')
    px = grey.load()
    w, h = field.size
    rows = list(range(150, 450)) + list(range(700, 1000))   # away from the corners and the wall-centre Toggles

    def brightest(along, lo, hi, cross):
        found = []
        for c in cross:
            if along == 'x':
                found.append(max(range(lo, hi), key=lambda x: px[x, c]))
            else:
                found.append(max(range(lo, hi), key=lambda y: px[c, y]))
        return statistics.median(found)

    # the rail's bright top edge on each wall, then 2 px inward to the inner face
    left = brightest('x', 20, 60, rows) + 2
    right = brightest('x', w - 60, w - 25, rows) - 2
    top = brightest('y', 20, 60, rows) + 2
    bottom = brightest('y', h - 60, h - 25, rows) - 2
    ox, oy = (left + right) / 2, (top + bottom) / 2
    scale = ((right - left) + (bottom - top)) / 2 / (2 * WALL_INNER_FACE_IN)

    # the centre goal: the dark body nearest the middle (its centroid), independent of the walls
    dark = [(x, y) for x in range(int(ox) - 45, int(ox) + 46) for y in range(int(oy) - 45, int(oy) + 46) if px[x, y] < 60]
    goal = (sum(p[0] for p in dark) / len(dark), sum(p[1] for p in dark) / len(dark))

    # FP2: the first RED and the first BLUE pixel of the Load Zones' tape in the columns along the left wall, each
    # searched over the WHOLE height from the top (a picture turned the wrong way finds red at the top)
    rgb = field.load()
    cols = range(int(left) + 8, int(left) + 90)
    red = next((x, y) for y in range(h) for x in cols if rgb[x, y][0] > 170 and rgb[x, y][1] < 90 and rgb[x, y][2] < 90)
    blue = next((x, y) for y in range(h) for x in cols if rgb[x, y][2] > 170 and rgb[x, y][0] < 90)

    # FP3: the tile seams (header) -- the grey tile pixels only, a trimmed mean per column, then smoothed
    tile_in = 2 * WALL_INNER_FACE_IN / 6

    def is_tile(c):
        r, g, b = c
        return abs(r - g) < 12 and abs(g - b) < 12 and 60 < r < 200

    def profile(axis, band):
        raw = {}
        for i in range(40, w - 38):
            if axis == 'x':
                vals = sorted(px[i, j] for j in range(*band) if is_tile(rgb[i, j]))
            else:
                vals = sorted(px[j, i] for j in range(*band) if is_tile(rgb[j, i]))
            raw[i] = statistics.mean(vals[len(vals) // 4:3 * len(vals) // 4]) if len(vals) > 30 else None
        out = {}
        for i in raw:
            near = [raw.get(k) for k in (i - 1, i, i + 1)]
            out[i] = sum(near) / 3 if all(v is not None for v in near) else None
        return out

    def crossing(prof, guess):
        left_lv = statistics.median(v for i in range(guess - 45, guess - 15) if (v := prof.get(i)) is not None)
        right_lv = statistics.median(v for i in range(guess + 15, guess + 45) if (v := prof.get(i)) is not None)
        mid = (left_lv + right_lv) / 2
        best = None
        for i in range(guess - 15, guess + 15):
            a, b = prof.get(i), prof.get(i + 1)
            if a is None or b is None or a == b or (a - mid) * (b - mid) > 0:
                continue
            t = i + (mid - a) / (b - a)
            if best is None or abs(t - guess) < abs(best - guess):
                best = t
        return best

    seams = {'x': {}, 'y': {}}
    for axis, o in (('x', ox), ('y', oy)):
        bands = [(int(o + k * tile_in * scale) - 55, int(o + k * tile_in * scale) + 55) for k in (-2.5, -1.5, -0.5, 0.5, 1.5, 2.5)]
        profs = [profile(axis, b) for b in bands]
        for k in (-2, -1, 1, 2):
            guess = int(round(o + k * tile_in * scale))
            found = [c for c in (crossing(p, guess) for p in profs) if c is not None]
            assert len(found) >= 4, f'seam {axis} {k}: found in only {len(found)} bands'
            # the key is the seam's field coordinate in tiles (x: +right; y: +up, so the picture's +k is field -k)
            seams[axis][str(k if axis == 'x' else -k)] = round(statistics.mean(found), 2)

    os.makedirs(OUT_DIR, exist_ok=True)
    field.save(os.path.join(OUT_DIR, 'field.webp'), 'WEBP', quality=QUALITY, method=6)
    meta = {
        'source': 'tools/docs_figures/source/override-field-top.png, turned 90 degrees counterclockwise (red at the bottom)',
        'width': w,
        'height': h,
        'pxPerIn': round(scale, 4),
        'originPx': [round(ox, 2), round(oy, 2)],
        'landmarks': {
            'centre goal': [round(goal[0], 2), round(goal[1], 2)],
            'right wall': [right, round(oy, 2)],
            'left wall': [left, round(oy, 2)],
            'blue wall': [round(ox, 2), top],
            'red wall': [round(ox, 2), bottom],
        },
        'redTapePx': list(red),
        'blueTapePx': list(blue),
        'tileSeamsPx': seams,
    }
    with open(os.path.join(OUT_DIR, 'field.json'), 'w', encoding='utf-8', newline='\n') as f:
        json.dump(meta, f, indent=2)
        f.write('\n')
    print(json.dumps(meta, indent=2))
    print('field.webp', os.path.getsize(os.path.join(OUT_DIR, 'field.webp')), 'bytes')


if __name__ == '__main__':
    main()
