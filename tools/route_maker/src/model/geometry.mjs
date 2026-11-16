// geometry.mjs — the field frame the route builder draws in, and the few pure helpers every model
// module shares (RMP1 brief, landmines 2 and 3).
//
// THE FRAME (shulib's, guide 2): inches, (0, 0) at the field's centre, +x toward the RIGHT wall, +y away
// from the red alliance station (toward BLUE), headings in DEGREES, 0 faces +x, growing counterclockwise,
// always in (-180, 180] — an exact antipode is +180, never -180 (the library's Angle rule, F3). A student's
// "north" (toward blue) is 90. Radians never reach the page; the only radians anywhere in the builder are
// the ones a turn cap is converted to inside the export, and that conversion is written in the C++ itself.
//
// Pure: no DOM, no file API.

/** Degrees -> radians (the only place the model converts). */
export const DEG = Math.PI / 180;

/**
 * `deg` wrapped into (-180, 180]: an exact antipode is +180 (F3). Uses the same remainder rule as the
 * library's Angle (std::remainder, then -180 moved to +180), so the page and the C++ agree on the edge.
 */
export function wrapDeg(deg) {
    if (!Number.isFinite(deg)) {
        return Number.NaN;
    }
    // IEEE remainder: deg - 360 * round-half-even(deg / 360)
    const q = deg / 360;
    let n = Math.round(q);
    if (Math.abs(q - Math.trunc(q)) === 0.5) {
        n = 2 * Math.round(q / 2);  // ties to even, as std::remainder
    }
    let w = deg - 360 * n;
    if (w <= -180) {
        w += 360;
    }
    return Object.is(w, -0) ? 0 : w;
}

/** The bearing (degrees, (-180, 180]) from (x0, y0) to (x1, y1); NaN when the two coincide. */
export function bearingDeg(x0, y0, x1, y1) {
    const dx = x1 - x0;
    const dy = y1 - y0;
    if (dx === 0 && dy === 0) {
        return Number.NaN;
    }
    return wrapDeg(Math.atan2(dy, dx) / DEG);
}

/** The straight distance (in) between two points. */
export function dist(x0, y0, x1, y1) {
    return Math.hypot(x1 - x0, y1 - y0);
}

/** The smallest angle (degrees, [0, 180]) between two headings. */
export function angleBetweenDeg(a, b) {
    return Math.abs(wrapDeg(a - b));
}

/**
 * The four corners of a `size`-in square robot at pose (x, y, headingDeg): front-left, front-right,
 * back-right, back-left (the FRONT is +headingDeg). The route builder's robot is RPV1's 15 in square.
 */
export function squareCorners(x, y, headingDeg, size) {
    const h = size / 2;
    const c = Math.cos(headingDeg * DEG);
    const s = Math.sin(headingDeg * DEG);
    const at = (fx, fy) => ({ x: x + fx * c - fy * s, y: y + fx * s + fy * c });
    return [at(h, h), at(h, -h), at(-h, -h), at(-h, h)];
}
