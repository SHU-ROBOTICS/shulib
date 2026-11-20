// region.mjs — THE RULES for writing into a team file (RMP1 brief A.3.7, A.6 [W12]; §6.3; landmines 9, 19).
//
// The ONE place the marker, hand-edit, splice and line-ending rules live. A host (files_fsaccess.mjs in
// Chrome/Edge today; a VS Code panel or a local helper after the 21st) only reads and writes TEXT; it never
// re-implements anything here. The rules:
//   * write ONLY between the two marker lines that already exist in the file (§6.3: `// ═══ ROUTE MAKER —
//     route "<name>" — BEGIN ═══…` and its END), each alone on its line, exactly one pair; never create
//     them (a student's Copy replaces the marked lines with a copy that carries them; never a new pair);
//   * nothing outside the region changes, byte for byte;
//   * the file's OWN line ending (CRLF from a Git-for-Windows checkout, or LF) and its final newline are kept
//     (this repository has no repository-wide .gitattributes, on purpose);
//   * a HAND EDIT inside the region is never overwritten silently: the project file keeps the hash of what
//     the builder last wrote (`exports`, never inside a source file); a region that is neither that nor the
//     empty starter region is a hand edit, refused unless the student says "overwrite";
//   * only files under src/team/ (and never with "..");
//   * the export into skills_route.cpp is READ BACK (readBack): a file that reads back different is a refusal,
//     never a success (RMP1-F).
// Hashes are SHA-256 of the region with LF line endings, so a CRLF checkout is not mistaken for an edit.
//
// Pure: no DOM, no file API. Tested directly and through the in-memory host (FA1-FA4).

import { exportRegion } from './export.mjs';
import { newRoute } from './route.mjs';

const BEGIN_RE = /^\/\/ ═══ ROUTE MAKER — route "([^"\r\n]*)" — BEGIN ═+$/;
const END_RE = /^\/\/ ═══ ROUTE MAKER — route "([^"\r\n]*)" — END ═+$/;

/** The project-relative file the team's routes are written into (§6.3). */
export const SKILLS_ROUTE_FILE = 'src/team/skills_route.cpp';
/** Robot two's autons file, read (never written) for its line that registers the team's routes (D6, R2; RMP1-C D2). */
export const ROBOT_TWO_AUTONS_FILE = 'src/team/autons.cpp';
/** Robot one's autons file, read (never written) the same way. */
export const ROBOT_ONE_AUTONS_FILE = 'src/team/robot_one_autons.cpp';

/** True for a path the builder may write: under src/team/, relative, no "..", no backslash tricks. */
export function isTeamPath(path) {
    if (typeof path !== 'string' || path.length === 0) {
        return false;
    }
    if (path.startsWith('/') || path.includes('\\') || /^[A-Za-z]:/.test(path) || path.includes('\0')) {
        return false;
    }
    const parts = path.split('/');
    if (parts.some((p) => p === '..' || p === '.' || p === '')) {
        return false;
    }
    return parts.length >= 3 && parts[0] === 'src' && parts[1] === 'team';
}

/** The file's line ending: "\r\n" when its lines end CRLF (the first line ending decides a mixed file). */
export function lineEnding(text) {
    const i = text.indexOf('\n');
    return i > 0 && text[i - 1] === '\r' ? '\r\n' : '\n';
}

/**
 * Where the region is: {ok: true, name, start, end, eol} — `start` the index of the BEGIN line's first
 * character, `end` the index just past the END line's last character (before its line ending) — or
 * {ok: false, why, words}.
 */
export function findRegion(text) {
    const eol = lineEnding(text);
    const lines = text.split('\n');
    let offset = 0;
    const begins = [];
    const ends = [];
    for (const raw of lines) {
        const line = raw.endsWith('\r') ? raw.slice(0, -1) : raw;
        const b = BEGIN_RE.exec(line);
        const e = END_RE.exec(line);
        if (b) {
            begins.push({ name: b[1], start: offset, stop: offset + line.length });
        }
        if (e) {
            ends.push({ name: e[1], start: offset, stop: offset + line.length });
        }
        offset += raw.length + 1;
    }
    if (begins.length === 0 && ends.length === 0) {
        return { ok: false, why: 'noMarkers',
                 words: 'This file has no ROUTE MAKER lines, so the route builder will not write into it: someone removed '
                      + 'them, or this is not the skills-route file.' };
    }
    if (begins.length !== 1 || ends.length !== 1) {
        return { ok: false, why: 'markerCount',
                 words: `This file has ${begins.length} BEGIN and ${ends.length} END ROUTE MAKER lines; the route builder `
                      + 'writes only into exactly one pair.' };
    }
    const [b] = begins;
    const [e] = ends;
    if (e.start < b.start || b.name !== e.name) {
        return { ok: false, why: 'markerOrder',
                 words: 'The ROUTE MAKER END line comes before its BEGIN line, or they name different routes.' };
    }
    return { ok: true, name: b.name, start: b.start, end: e.stop, eol };
}

/** The region's text (BEGIN to END) with LF line endings. */
export function regionText(text, found) {
    return text.slice(found.start, found.end).replace(/\r\n/g, '\n');
}

// ── SHA-256 (FIPS 180-4), synchronous and pure, so the same hash comes out of every host ────────────
const K = new Uint32Array([
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98,
    0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8,
    0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
    0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
    0xc67178f2,
]);

/** SHA-256 of `text` (UTF-8), as lowercase hex. */
export function sha256Hex(text) {
    const msg = new TextEncoder().encode(text);
    const bitLen = msg.length * 8;
    const padded = new Uint8Array(((msg.length + 9 + 63) >> 6) << 6);
    padded.set(msg);
    padded[msg.length] = 0x80;
    const dv = new DataView(padded.buffer);
    dv.setUint32(padded.length - 8, Math.floor(bitLen / 0x100000000));
    dv.setUint32(padded.length - 4, bitLen >>> 0);
    const H = new Uint32Array([0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
                               0x5be0cd19]);
    const W = new Uint32Array(64);
    const rotr = (x, n) => (x >>> n) | (x << (32 - n));
    for (let off = 0; off < padded.length; off += 64) {
        for (let i = 0; i < 16; ++i) {
            W[i] = dv.getUint32(off + i * 4);
        }
        for (let i = 16; i < 64; ++i) {
            const s0 = rotr(W[i - 15], 7) ^ rotr(W[i - 15], 18) ^ (W[i - 15] >>> 3);
            const s1 = rotr(W[i - 2], 17) ^ rotr(W[i - 2], 19) ^ (W[i - 2] >>> 10);
            W[i] = (W[i - 16] + s0 + W[i - 7] + s1) >>> 0;
        }
        let [a, b, c, d, e, f, g, h] = H;
        for (let i = 0; i < 64; ++i) {
            const S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const ch = (e & f) ^ (~e & g);
            const t1 = (h + S1 + ch + K[i] + W[i]) >>> 0;
            const S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const maj = (a & b) ^ (a & c) ^ (b & c);
            const t2 = (S0 + maj) >>> 0;
            h = g;
            g = f;
            f = e;
            e = (d + t1) >>> 0;
            d = c;
            c = b;
            b = a;
            a = (t1 + t2) >>> 0;
        }
        H[0] = (H[0] + a) >>> 0; H[1] = (H[1] + b) >>> 0; H[2] = (H[2] + c) >>> 0; H[3] = (H[3] + d) >>> 0;
        H[4] = (H[4] + e) >>> 0; H[5] = (H[5] + f) >>> 0; H[6] = (H[6] + g) >>> 0; H[7] = (H[7] + h) >>> 0;
    }
    return [...H].map((x) => x.toString(16).padStart(8, '0')).join('');
}

/** The hash the project file keeps for a region (LF text). */
export function regionHash(regionLF) {
    return `sha256:${sha256Hex(regionLF)}`;
}

/** The `exports` key of a route's region in the project file (D3: "src/team/skills_route.cpp#skills"). */
export function exportKey(path, routeName) {
    return `${path}#${routeName}`;
}

/**
 * Splice `newRegionLF` (exportRegion's output: BEGIN..END, LF, no final newline) into `fileText` in place
 * of its region. `lastHash`: the hash the project recorded at the last write (or undefined); `overwrite`:
 * the student said "overwrite" for a hand edit. Returns {ok: true, text, hash, changed} or
 * {ok: false, why, words}. Nothing outside the region changes; the file's line ending and final newline
 * are kept.
 */
export function spliceRegion(fileText, newRegionLF, { lastHash, overwrite = false } = {}) {
    const found = findRegion(fileText);
    if (!found.ok) {
        return found;
    }
    const current = regionText(fileText, found);
    const currentHash = regionHash(current);
    const emptyStarter = exportRegion([newRoute()]);
    const known = currentHash === lastHash || current === emptyStarter || current === newRegionLF;
    if (!known && !overwrite) {
        return { ok: false, why: 'handEdit', currentHash,
                 words: 'Someone has changed the code between the ROUTE MAKER lines by hand since the route builder last '
                      + 'wrote it. Exporting would replace their change. Keep it (and copy the code instead), or choose '
                      + '"Overwrite" to replace it with this route.' };
    }
    const eol = found.eol;
    const body = eol === '\n' ? newRegionLF : newRegionLF.replace(/\n/g, eol);
    const text = fileText.slice(0, found.start) + body + fileText.slice(found.end);
    return { ok: true, text, hash: regionHash(newRegionLF), changed: text !== fileText };
}

/**
 * THE READ-BACK (RMP1-F D2): the file as read AFTER a write. `text`: null when it is gone; `regionLF`: the export;
 * `wrote`: the whole text written (undefined: nothing was written, the file already had it). {ok: true}, or
 * {ok: false, why: 'gone' | 'differs', words} (markers lost in the write are 'differs': the page wrote them).
 */
export function readBack(text, regionLF, wrote) {
    if (text === null) {
        return { ok: false, why: 'gone', words: 'The file was not there when the page read it back.' };
    }
    const found = findRegion(text);
    if (!found.ok || regionText(text, found) !== regionLF || (wrote !== undefined && text !== wrote)) {
        return { ok: false, why: 'differs', words: 'The file read back is not what the page wrote: something changed it as it was '
                 + 'written (an editor with it open, or a sync folder).' };
    }
    return { ok: true };
}

/**
 * Does an autons file register the team's routes — `addTeamRoutes(registry, TeamRobot::One)` in robot one's,
 * `...::Two` in robot two's (RMP1-C D2: plumbing; the page only CHECKS)? Comments are ignored, so a commented-out line
 * is "missing".
 */
export function hasTeamRoutesLine(autonsText, robot) {
    const code = autonsText.replace(/\/\*[\s\S]*?\*\//g, '').replace(/\/\/[^\n]*/g, '');
    return new RegExp(`addTeamRoutes\\s*\\(\\s*registry\\s*,\\s*TeamRobot::${robot}\\s*\\)\\s*;`).test(code);
}

/** The line each autons file carries (shown, with Copy, ONLY when it is missing — D6). */
export const teamRoutesLine = (robot) => `addTeamRoutes(registry, TeamRobot::${robot});`;
