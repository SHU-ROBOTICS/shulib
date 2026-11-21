// driver_file.mjs — A ROBOT'S DRIVER FILE (RMP1-B; the brief's D9, §6.3 item 5): its three regions found, read
// into the Controls tab's state, made again from it, and written back — ONLY between the three marker pairs
// (`controls`, `control words`, `feel`), all three or none, the file's own line endings kept, a hand edit never
// overwritten without the student's "Overwrite" (the hashes live in the project file, never in the source).
// Until the `control words` and `feel` markers are in the file (robot two's update places them) the page
// still READS the file — the map and THE FEEL by their shape — and offers Copy only. Pure.

import { lineEnding, regionHash } from './region.mjs';
import { parseControls, parseControlWords, exportControls, exportControlWords } from './controls.mjs';
import { parseFeel, exportFeel, FEEL_HEADING } from './feel.mjs';
import { robotName } from './robot_names.mjs';

/** The driver file of robot `id` ("one", "two"). */
export const driverFilePath = (id) => `src/team/robot_${id}_driver.hpp`;

/** The three regions' marker lines (each alone on its line). `controls` is MECH-A5's; the other two are placed
 *  by robot two's update (ROB1-BT) — this table is the one place their text is written. */
export const REGIONS = ['controls', 'control words', 'feel'];
export function markers(id) {
    return {
        controls: [`// [driver file ${id}]`, `// [/driver file ${id}]`],
        'control words': [`// [control words ${id}]`, `// [/control words ${id}]`],
        feel: [`// [feel ${id}]`, `// [/feel ${id}]`],
    };
}

/** The project's `exports` key of one region. */
export const regionKey = (id, region) => `${driverFilePath(id)}#${region}`;

/**
 * Where each region's lines are: {eol, lines (the file's, without line endings), found: {region: {from, to, marked}}}
 * — `from`..`to` the region's inner lines (to exclusive); `marked` false when found by its shape only (no markers:
 * readable, never written). A region whose markers are broken (one of the pair, or twice) is not found.
 */
export function findRegions(text, id) {
    const eol = lineEnding(text);
    const raw = text.split('\n');
    const lines = raw.map((l) => (l.endsWith('\r') ? l.slice(0, -1) : l));
    const found = {};
    const problems = [];
    for (const [region, [begin, end]] of Object.entries(markers(id))) {
        const b = lines.reduce((a, l, i) => (l === begin ? [...a, i] : a), []);
        const e = lines.reduce((a, l, i) => (l === end ? [...a, i] : a), []);
        if (b.length === 1 && e.length === 1 && e[0] > b[0]) {
            found[region] = { from: b[0] + 1, to: e[0], marked: true };
        } else if (b.length || e.length) {
            problems.push(`its ${region} markers are not one "${begin}" line followed by one "${end}" line`);
        }
    }
    // each part closes before the next begins: a write into nested parts would put one's old lines back over the other
    const spans = Object.entries(found).sort((a, b) => a[1].from - b[1].from);
    for (let k = 1; k < spans.length; ++k) {
        if (spans[k][1].from - 1 <= spans[k - 1][1].to) {
            problems.push(`its "${spans[k - 1][0]}" and "${spans[k][0]}" markers overlap: each part must end before the next begins`);
        }
    }
    if (!found['control words']) {
        const from = lines.findIndex((l) => /^\/\/ {3}\S.*stick/.test(l));
        if (from >= 0) {
            let to = from;
            while (to < lines.length && /^\/\/( {3}| {16})\S/.test(lines[to])) {
                ++to;
            }
            found['control words'] = { from, to, marked: false };
        }
    }
    if (!found.feel) {
        const from = lines.indexOf(FEEL_HEADING);
        const loop = lines.findIndex((l, i) => i > from && /^inline constexpr DriverLoopConfig kLoop\{$/.test(l));
        const to = loop < 0 ? -1 : lines.findIndex((l, i) => i > loop && l === '};');
        if (from >= 0 && to > from) {
            found.feel = { from, to: to + 1, marked: false };
        }
    }
    return { eol, raw, lines, found, problems };
}

/** A region's text, LF line endings, no final newline (what its hash is of). */
export const regionOf = (f, region) => f.lines.slice(f.found[region].from, f.found[region].to).join('\n');

/**
 * Read a driver file into the tab's state: {ok, state: {controls, driving}, hashes, marked, words} or
 * {ok: false, words}. `robot` is its description (for nothing but the table's default name).
 */
export function readDriverFile(text, id) {
    const f = findRegions(text, id);
    if (!f.found.controls) {
        return { ok: false, words: `${driverFilePath(id)} has no "${markers(id).controls[0]}" region${f.problems.length ? ` (${f.problems[0]})` : ''}.` };
    }
    const c = parseControls(regionOf(f, 'controls').split('\n'));
    if (!c.ok) {
        return { ok: false, words: `${driverFilePath(id)}: ${c.words}` };
    }
    const problems = [...f.problems];
    let driving = { mode: 'arcade', rezeroButton: 'Y', onRelease: 'coast' };
    const read = { controls: true, feel: false, 'control words': false };  // a region read: the page may write it
    if (f.found.feel) {
        const d = parseFeel(regionOf(f, 'feel').split('\n'));
        if (d.ok) {
            driving = d.driving;
            read.feel = true;
        } else {
            problems.push(d.words);
        }
    } else {
        problems.push('THE FEEL was not found');
    }
    let controls = c.state;
    if (f.found['control words']) {
        const w = parseControlWords(regionOf(f, 'control words').split('\n'), controls, { rezero: driving.rezeroButton });
        if (w.ok) {
            controls = w.state;
            read['control words'] = true;
        } else {
            problems.push(w.words);
        }
    } else {
        problems.push('the button map in the header was not found');
    }
    // a region the page could not read gets no base: writing it then asks first (Keep / Overwrite), never silently
    const hashes = {};
    for (const r of REGIONS) {
        if (f.found[r] && read[r]) {
            hashes[r] = regionHash(regionOf(f, r));
        }
    }
    const marked = REGIONS.every((r) => f.found[r] && f.found[r].marked);
    return { ok: true, state: { controls, driving }, hashes, marked, problems };
}

/** The three regions' lines, made from the tab's state. */
export function exportRegions(state, robot) {
    return {
        controls: exportControls(state.controls, robot),
        'control words': exportControlWords(state.controls),
        feel: exportFeel(state.driving),
    };
}

/**
 * Write the regions into the file's text: {ok, text, hashes, changed} or {ok: false, why, words, regions}.
 * `base`: {region: hash} the page last read or wrote (a region that is neither that nor the new text is a hand
 * edit — refused, all three regions untouched, unless `overwrite`). Nothing outside the regions changes.
 */
export function spliceRegions(text, id, regions, { base = {}, overwrite = false } = {}) {
    const f = findRegions(text, id);
    const missing = REGIONS.filter((r) => !(f.found[r] && f.found[r].marked));
    if (missing.length || f.problems.length) {
        return { ok: false, why: 'noMarkers', regions: missing,
                 words: f.problems[0] ? `${driverFilePath(id)}: ${f.problems[0]}.`
                     : `${driverFilePath(id)} has no ${missing.map((r) => `"${r}"`).join(' or ')} markers yet, so the route builder `
                     + `will not write into it (it gets them with ${robotName('two')}'s update). Until then use Copy.` };
    }
    const edited = REGIONS.filter((r) => {
        const now = regionOf(f, r);
        return regionHash(now) !== base[r] && now !== regions[r].join('\n');
    });
    if (edited.length && !overwrite) {
        return { ok: false, why: 'handEdit', regions: edited,
                 words: `Someone has changed ${edited.map((r) => `the ${r}`).join(' and ')} in ${driverFilePath(id)} by hand since the `
                      + 'route builder last read or wrote it. Writing would replace their change. Keep it (and copy the code instead), or '
                      + 'choose "Overwrite".' };
    }
    const out = [...f.raw];  // every line outside the regions exactly as it was, its own ending included
    const cr = f.eol === '\r\n' ? '\r' : '';
    for (const r of [...REGIONS].sort((a, b) => f.found[b].from - f.found[a].from)) {
        out.splice(f.found[r].from, f.found[r].to - f.found[r].from, ...regions[r].map((l) => l + cr));
    }
    const next = out.join('\n');
    const hashes = Object.fromEntries(REGIONS.map((r) => [r, regionHash(regions[r].join('\n'))]));
    return { ok: true, text: next, hashes, changed: next !== text };
}
