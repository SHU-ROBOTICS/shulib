// project.mjs — THE PROJECT FILE, src/team/route_maker.json (RMP1 brief D3 (a), D14 (a); run RMP1-A1).
//
// One file beside the autons, committed with the code: the routes (G3a's R.4 shape, `onFail` inside `care`,
// `mirroredFor`), each robot's later blocks (`robots.<id>`: the controls and feel are RMP1-B's; `seen` says
// which published builder last checked the route), and `exports` — the hash of each region the builder last
// wrote, the hand-edit check's memory, kept OUT of every source file (R4, R12). Inches, degrees, seconds,
// volts — the units the student sees.
//
// Loading never half-reads (D3): a file from a NEWER route builder is refused by name, nothing loaded — a
// schemaVersion above 1 (RT4), and ALSO a schemaVersion-1 file that carries what this builder cannot write:
// a step kind it does not know (A1b's `wallReset`), a field it does not know on a route, a step, a step's
// `care` or a curve point (A1b's `during` trigger), or a second route (RT7) — because opening such a file and
// exporting it would silently drop that part from the robot's code. A step whose mechanism the robot does not
// have is KEPT and marked by the checks, never dropped (RT5). A top-level block this builder does not use
// (`robots.<id>` — RMP1-B's controls and feel — or any other) is kept and written back untouched. Saving is
// deterministic: the same project always writes the same bytes (RT1), two-space JSON, keys in one order, LF,
// a final newline.
//
// Pure: no DOM, no file API.

import { normalizeRoute, newRoute, STEP_KINDS, STEP_KEYS, CARE_KEYS, POINT_KEYS, ROUTE_KEYS } from './route.mjs';

/** The project file's schema this builder reads and writes. */
export const SCHEMA_VERSION = 1;
/** Where the project file lives in the team's repository. */
export const PROJECT_FILE = 'src/team/route_maker.json';

/** A new, empty project: robot one's skills route, no steps. */
export function newProject(madeWith = 'shulib route builder') {
    return { schemaVersion: SCHEMA_VERSION, madeWith, routes: [newRoute()], robots: {}, exports: {} };
}

const ROUTE_ORDER = ['name', 'kind', 'robot', 'mirroredFor', 'start', 'endActionAtS', 'hardStopAtS', 'steps'];
const TOP_ORDER = ['schemaVersion', 'madeWith', 'routes', 'robots', 'exports'];

function ordered(obj, order) {
    const out = {};
    for (const k of order) {
        if (obj[k] !== undefined) {
            out[k] = obj[k];
        }
    }
    for (const k of Object.keys(obj)) {
        if (!(k in out) && obj[k] !== undefined) {
            out[k] = obj[k];
        }
    }
    return out;
}

/** The project as the bytes saved: deterministic two-space JSON with a final newline. */
export function saveProject(project) {
    const p = ordered({ ...project, schemaVersion: SCHEMA_VERSION,
                        routes: project.routes.map((r) => ordered(normalizeRoute(r), ROUTE_ORDER)) }, TOP_ORDER);
    return `${JSON.stringify(p, null, 2)}\n`;
}

const plainObject = (v) => v !== null && typeof v === 'object' && !Array.isArray(v);
const unknownKey = (obj, known) => Object.keys(obj).find((k) => !known.includes(k));

/**
 * What in `data` (a schemaVersion-1 project) this builder cannot write, in words ("a step kind \"wallReset\"
 * (route \"skills\", step 2)"), or null when every part is known (header: never half-read).
 */
export function newerParts(data) {
    if (data.routes.length > 1) {
        return `${data.routes.length} routes (this one edits the one skills route)`;
    }
    for (const r of data.routes) {
        const where = `route "${r.name}"`;
        const rk = unknownKey(r, ROUTE_KEYS);
        if (rk !== undefined) {
            return `a route field "${rk}" (${where})`;
        }
        if (r.kind !== undefined && r.kind !== 'skills') {
            return `a ${JSON.stringify(r.kind)} route (${where}; this one writes the skills route)`;
        }
        if (r.mirroredFor !== undefined && r.mirroredFor !== 'two') {
            return `a route mirrored for ${JSON.stringify(r.mirroredFor)} (${where})`;
        }
        for (const k of ['endActionAtS', 'hardStopAtS']) {
            if (r[k] !== undefined && !(typeof r[k] === 'number' && Number.isFinite(r[k]))) {
                return `a "${k}" that is not a number (${where})`;
            }
        }
        if (r.start !== undefined && r.start !== null) {
            if (!plainObject(r.start)) {
                return `a start that is not {x, y, headingDeg} (${where})`;
            }
            const sk = unknownKey(r.start, ['x', 'y', 'headingDeg']);
            if (sk !== undefined) {
                return `a start field "${sk}" (${where})`;
            }
        }
        for (let i = 0; i < r.steps.length; ++i) {
            const s = r.steps[i];
            const at = `${where}, step ${i + 1}`;
            if (!STEP_KINDS.includes(s.do)) {
                return `a step kind "${s.do}" (${at})`;
            }
            const sk = unknownKey(s, STEP_KEYS);
            if (sk !== undefined) {
                return `a step field "${sk}" (${at})`;
            }
            if (s.care !== undefined && s.care !== null) {
                if (!plainObject(s.care)) {
                    return `a "care" that is not a set of fields (${at})`;
                }
                const ck = unknownKey(s.care, CARE_KEYS);
                if (ck !== undefined) {
                    return `a "care" field "${ck}" (${at})`;
                }
                const f = s.care.onFail;
                if (f !== undefined && f !== 'stop' && f !== 'carryOn'
                    && !(plainObject(f) && unknownKey(f, ['skipTo']) === undefined && typeof f.skipTo === 'string')) {
                    return `an "onFail" of ${JSON.stringify(f)} (${at})`;
                }
            }
            if (s.points !== undefined) {
                if (!Array.isArray(s.points) || !s.points.every(plainObject)) {
                    return `a "points" that is not a list of points (${at})`;
                }
                for (const p of s.points) {
                    const pk = unknownKey(p, POINT_KEYS);
                    if (pk !== undefined) {
                        return `a curve point field "${pk}" (${at})`;
                    }
                }
            }
            if (s.preset !== undefined && typeof s.preset !== 'number' && typeof s.preset !== 'string') {
                return `a "preset" of ${JSON.stringify(s.preset)} (${at})`;
            }
        }
    }
    return null;
}

/**
 * The project in `text`: {ok: true, project} or {ok: false, words}. Accepts the file's own line endings
 * (a Windows checkout's CRLF) and a byte-order mark.
 */
export function loadProject(text) {
    let data;
    try {
        data = JSON.parse(String(text).replace(/^﻿/, ''));
    } catch (e) {
        return { ok: false, words: `This is not a route builder project file (it is not readable JSON: ${e.message}).` };
    }
    if (data === null || typeof data !== 'object' || Array.isArray(data)) {
        return { ok: false, words: 'This is not a route builder project file (no project in it).' };
    }
    if (typeof data.schemaVersion !== 'number' || !Number.isInteger(data.schemaVersion)) {
        return { ok: false, words: 'This is not a route builder project file (it has no schemaVersion).' };
    }
    if (data.schemaVersion > SCHEMA_VERSION) {
        return { ok: false, words: `This project was saved by a NEWER route builder (schemaVersion ${data.schemaVersion}; this one `
            + `reads ${SCHEMA_VERSION}). Nothing was opened. Open it in the newest route builder on the docs site.` };
    }
    if (data.schemaVersion < 1) {
        return { ok: false, words: `This project file's schemaVersion ${data.schemaVersion} is not one any route builder wrote.` };
    }
    if (!Array.isArray(data.routes) || data.routes.length === 0) {
        return { ok: false, words: 'This project file has no route in it.' };
    }
    for (const r of data.routes) {
        if (!r || typeof r !== 'object' || !Array.isArray(r.steps) || typeof r.name !== 'string') {
            return { ok: false, words: 'A route in this project file has no name or no steps list.' };
        }
        for (const s of r.steps) {
            if (!s || typeof s !== 'object' || typeof s.do !== 'string') {
                return { ok: false, words: `A step of route "${r.name}" says nothing about what it does ("do" is missing).` };
            }
        }
    }
    const newer = newerParts(data);
    if (newer) {
        return { ok: false, words: `This project has ${newer}, which this route builder cannot write: it was made by a NEWER `
            + 'route builder. Nothing was opened (exporting it here would leave that part out of the robot\'s code). '
            + 'Open it in the newest route builder on the docs site.' };
    }
    const project = {
        ...data,
        routes: data.routes.map((r) => normalizeRoute({ ...newRoute(r.name), ...r })),
        robots: data.robots && typeof data.robots === 'object' ? data.robots : {},
        exports: data.exports && typeof data.exports === 'object' ? data.exports : {},
    };
    return { ok: true, project };
}
