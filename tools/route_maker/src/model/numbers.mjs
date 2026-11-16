// numbers.mjs — THE number rule of the route builder (RMP1 brief, A.2 [W7]; landmine 25).
//
// Every number a route carries is stored ALREADY ROUNDED, at the moment it is edited, to what the export
// prints: at most 3 decimals (the range the brief's probe P-W1c proved prints and parses back exactly
// through the unit literals). A dragged point is rounded as it is dropped; a typed value is rounded as it
// is entered and shown rounded. So the C++ the builder writes and the route the page shows are the same
// numbers, character for character — and the preview (a later run) can never run a 12.3456 the file
// prints as 12.346. ONE formatter (`fmt`) prints every number the export writes; nothing else formats a
// number for the C++.
//
// Pure: no DOM, no file API. Tested by test/numbers.test.mjs (NF1 and its edges).

/** The most decimals a stored number keeps (P-W1c's proven range). */
export const DECIMALS = 3;

/**
 * `v` rounded to the 0.001 grid, the way it is stored. `toFixed` reads the exact binary value, so the
 * rounding is the same on every engine. -0 becomes 0 (a "-0" never reaches the file). A non-finite input
 * stays NaN: the checks refuse it by name rather than inventing a number.
 */
export function round3(v) {
    if (typeof v !== 'number' || !Number.isFinite(v)) {
        return Number.NaN;
    }
    const r = Number(v.toFixed(DECIMALS));
    return Object.is(r, -0) ? 0 : r;
}

/**
 * The text the export prints for a stored number: at most 3 decimals, no trailing zeros, no "-0", never
 * an exponent ("1e-7" is not a number a student reads). `fmt(round3(x))` is what the file shows.
 */
export function fmt(v) {
    if (typeof v !== 'number' || !Number.isFinite(v)) {
        throw new Error(`fmt: not a finite number (${v}) -- the checks refuse this before an export`);
    }
    let s = round3(v).toFixed(DECIMALS);
    if (s.includes('.')) {
        s = s.replace(/0+$/, '').replace(/\.$/, '');
    }
    return s === '-0' ? '0' : s;
}

/**
 * A number typed by a student: "12", "-3.5", ".5", "  7 " → the stored (rounded) value; anything else
 * (empty, "12in", "1e3", "0x10", a comma) → NaN, which the field marks as not a number. Exponents and
 * hex are refused on purpose: the page shows plain decimals only.
 */
export function parseTyped(text) {
    if (typeof text === 'number') {
        return round3(text);
    }
    const t = String(text ?? '').trim();
    if (!/^[-+]?(\d+\.?\d*|\.\d+)$/.test(t)) {
        return Number.NaN;
    }
    return round3(Number(t));
}

/** True when `v` is a finite number already on the stored grid (what every stored number must be). */
export function isStored(v) {
    return typeof v === 'number' && Number.isFinite(v) && round3(v) === v && !Object.is(v, -0);
}
