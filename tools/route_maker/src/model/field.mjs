// field.mjs — the field's numbers and the snapping rules (RMP1 brief §1.1 `field.mjs`, §1.4 "Snapping").
//
// The field's numbers come from the robot description the C++ wrote (test/route_maker_describe.hpp: the
// walls' inner faces from localization/override_walls.hpp, the Toggles and the centre goal from
// vision/override_elements.hpp) — never hand-copied here. The picture's numbers come from field.json, which
// tools/docs_figures/route_maker_field.py wrote beside field.webp when it turned the manual's top view
// red-at-bottom (FP1 and FP2 hold the two together).
//
// SNAPPING (§1.4): a point snaps to the half-tile grid (every 12 in: the tile lines and the tile centres),
// to a wall (the robot's square flush against it), and a heading to 15 deg; Alt holds snapping off and
// Shift turns headings to 1 deg. A snapped point is a point the student chose — the builder never places
// one for them.
//
// Pure: no DOM, no file API.

import { round3 } from './numbers.mjs';
import { wrapDeg } from './geometry.mjs';

/** The half-tile grid (in): tile lines (every 24 in) and tile centres (12 in off them). */
export const GRID_IN = 12;

/** Picture pixels -> field inches, from field.json's numbers. */
export function pictureToField(px, py, pic) {
    return { x: (px - pic.originPx[0]) / pic.pxPerIn, y: (pic.originPx[1] - py) / pic.pxPerIn };
}

/** Field inches -> picture pixels. */
export function fieldToPicture(x, y, pic) {
    return { px: pic.originPx[0] + x * pic.pxPerIn, py: pic.originPx[1] - y * pic.pxPerIn };
}

/**
 * The point (x, y) snapped: each coordinate to the 12 in grid, or flush to a wall (the square's edge on the
 * wall's inner face), when it is within `within` in of it; else unchanged. Rounded as it is stored.
 */
export function snapPoint(x, y, field, size, within) {
    const flush = field.wallInnerFaceIn - size / 2;
    const one = (v) => {
        const candidates = [Math.round(v / GRID_IN) * GRID_IN, flush, -flush];
        let best = v;
        let bestD = within;
        for (const c of candidates) {
            const d = Math.abs(c - v);
            if (d <= bestD) {
                best = c;
                bestD = d;
            }
        }
        return best;
    };
    return { x: round3(one(x)), y: round3(one(y)) };
}

/** A heading snapped to `step` deg (15 normally; 1 with Shift), wrapped into (-180, 180]. */
export function snapHeading(deg, step = 15) {
    return round3(wrapDeg(Math.round(deg / step) * step));
}

/** True when (x, y) is on the field (inside the walls' inner faces). */
export function onField(x, y, field) {
    return Math.abs(x) <= field.wallInnerFaceIn && Math.abs(y) <= field.wallInnerFaceIn;
}
