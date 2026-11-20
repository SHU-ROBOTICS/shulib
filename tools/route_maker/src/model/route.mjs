// route.mjs — THE ROUTE MODEL of the route builder (RMP1 brief §1.1 `route.mjs`, D2, D3; run RMP1-A1).
//
// A route is plain data in the project file's own shape (D3 / D14: G3a's R.4 route schema, `onFail` inside
// `care`, `mirroredFor` for the mirrored robot), in the units the student sees: inches, degrees, seconds,
// volts. Every edit is a PURE function that returns a NEW route (the undo history keeps the old ones), and
// every number is stored already rounded (numbers.mjs, W7), headings wrapped into (-180, 180].
//
// THE STEP KINDS OF RMP1-A1 (D2 (a), cut to A1 by A.7) — each is ONE library verb (chassis/routine.hpp):
//   driveTo / reverseTo   {x, y, faceFirst}          r.face / r.faceAway first when faceFirst (R6), then the leg
//   turnTo                {headingDeg}                r.turnTo
//   face / faceAway       {x, y}                      r.face / r.faceAway
//   curveTo / reverseCurveTo {points: [{x, y, headingDeg?}]}   r.curveTo / r.reverseCurveTo (CURVE1)
//   move / startMove      {mechanism, preset, boundS} r.move / r.startMove (an arm's or a lift's preset; a
//                                                      NUMBER preset is the robot's facts ledger's level i,
//                                                      written `m[facts::lv(i).name]`; a STRING is `m["name"]`)
//                         or ONE of {up, down, to, bottom} in place of `preset` (RMP1-C D7): "go up 6", "go down
//                         6", "move to 18", "back to the bottom" -- written as the library's constant target
//                         `m.to(<height>)` from the height the plan tracks (lift.mjs)
//   forward / backward    {inches}                    RMP1-C D9: "drive forward 6 in", "back up 6 in" from the
//                                                      planned pose -- written as the driveTo / reverseTo point
//   set                   {mechanism, position}       r.set (a pneumatic's named position)
//   run / startRun        {mechanism, volts, forS}    r.run / r.startRun (a power mechanism, for a time)
//   home / startHome      {mechanism, boundS}         r.home / r.startHome (an arm or a lift)
//   waitUntilDone         {mechanism, boundS}         r.waitUntilDone
//   wait                  {seconds}                   the run guard's pause (never r.pause: it cannot be cut)
// and on EVERY step: `label` (a name a skip can land on), `touches` (RMP1-H: the ids of field structures it touches on
// purpose -- page-only, silences their "too close"), and `care` — {timeoutS, maxSpeedInPerS, maxTurnDegPerS,
// toleranceIn, toleranceDeg} on a motion (0 or absent = the robot's own), and `onFail`: "stop" (the default,
// absent), "carryOn", or {"skipTo": "<label>"}. A `mechanism` is the declaration's C++ NAME (what the export
// writes); the page shows the mechanism's `.name` beside it from the robot's description.
// NOT in A1 (A1b): triggers partway, the wall reset, the Toggle steps, "who runs it".
// RMP1-G: and `view` — how the step shows on the page (hidden, locked, a section: step_view.mjs); never exported.
//
// Pure: no DOM, no file API.

import { round3, isStored } from './numbers.mjs';
import { wrapDeg, bearingDeg, dist } from './geometry.mjs';
import { CurvePath, kDefaultCurveConfig } from './curve.mjs';
import { normalizeView, passSection, unfoldAt } from './step_view.mjs';

/** Every step kind A1 offers, in the toolbar's order. */
export const STEP_KINDS = Object.freeze([
    'driveTo', 'reverseTo', 'turnTo', 'face', 'faceAway', 'curveTo', 'reverseCurveTo', 'forward', 'backward',
    'move', 'startMove', 'set', 'run', 'startRun', 'home', 'startHome', 'waitUntilDone', 'wait',
]);
/** The steps that drive the robot (they carry motion options). */
export const MOTION_KINDS = new Set(['driveTo', 'reverseTo', 'turnTo', 'face', 'faceAway', 'curveTo', 'reverseCurveTo',
                                     'forward', 'backward']);
/** A move's target fields: exactly one of them (or `preset`) says where it goes. */
export const TARGET_KEYS = Object.freeze(['up', 'down', 'to', 'bottom']);
/** The steps that name a mechanism. */
export const MECHANISM_KINDS = new Set(['move', 'startMove', 'set', 'run', 'startRun', 'home', 'startHome', 'waitUntilDone']);
/** The motion option fields, in MotionOptions' declaration order (the export writes them in this order). Their
 *  names are G3a's R.4 route schema's (D3, D14 (a)): the panel after the 21st reads the SAME file. */
export const CARE_MOTION_FIELDS = Object.freeze(['timeoutS', 'maxSpeedInPerS', 'maxTurnDegPerS', 'toleranceIn', 'toleranceDeg']);
/** Every field a step of THIS builder carries (D3: a file with any other is from a newer builder, refused). */
export const STEP_KEYS = Object.freeze(['label', 'do', 'x', 'y', 'faceFirst', 'headingDeg', 'points', 'inches', 'mechanism', 'preset',
                                        'up', 'down', 'to', 'bottom', 'position', 'volts', 'forS', 'boundS', 'seconds', 'care',
                                        'touches', 'view']);
/** Every field of a step's `care` this builder knows. */
export const CARE_KEYS = Object.freeze([...CARE_MOTION_FIELDS, 'onFail']);
/** Every field of a curve point. */
export const POINT_KEYS = Object.freeze(['x', 'y', 'headingDeg']);
/** Every field of a route (the project file's `routes[i]`). */
export const ROUTE_KEYS = Object.freeze(['name', 'kind', 'robot', 'mirroredFor', 'together', 'start', 'endActionAtS', 'hardStopAtS',
                                         'steps']);
/** The competition's skills route (RMP1-C D1): every project has it, and it keeps this name — the marker lines, the
 *  docs and the competition program point at it. The team's other routes are named by the team. */
export const ROUTE_NAME = 'skills';

/** The route an empty project starts with: robot one's skills route, mirrored for robot two (MIRROR1). */
export function newRoute(name = 'skills') {
    return {
        name,
        kind: 'skills',
        robot: 'one',
        mirroredFor: 'two',
        start: null,
        endActionAtS: 57,
        hardStopAtS: 59,
        steps: [],
    };
}

const NUMBER_FIELDS = ['x', 'y', 'inches', 'up', 'down', 'to', 'boundS', 'volts', 'forS', 'seconds'];

/** True for a value that means "nothing typed": absent, null (JSON's form of a lost number) or empty. */
const blank = (v) => v === undefined || v === null || v === '';

function roundCare(care) {
    if (care === undefined || care === null) {
        return undefined;
    }
    const out = {};
    for (const f of CARE_MOTION_FIELDS) {
        if (blank(care[f]) || (typeof care[f] === 'number' && !Number.isFinite(care[f]))) {
            continue;  // nothing typed (or a number JSON cannot hold): the robot's own -- never stored as NaN
        }
        const v = round3(Number(care[f]));
        out[f] = Number.isNaN(v) ? String(care[f]) : v;  // a hand-edited "fast" kept AS TEXT, so the check names it
    }
    if (care.onFail !== undefined && care.onFail !== null && care.onFail !== 'stop') {
        out.onFail = typeof care.onFail === 'object' ? { skipTo: String(care.onFail.skipTo ?? '') } : care.onFail;
    }
    return Object.keys(out).length ? out : undefined;
}

/**
 * The step as it is STORED: every number rounded (W7), every heading wrapped into (-180, 180], `care`
 * pruned of empty fields, the keys in one fixed order (label, do, the kind's fields, care) so a saved file
 * is byte-stable (RT1). An unknown key is kept here (loadProject refuses a FILE that has one, by name: D3).
 */
export function normalizeStep(step) {
    const out = {};
    if (step.label !== undefined && step.label !== null && step.label !== '') {
        out.label = String(step.label);
    }
    out.do = step.do;
    const order = ['x', 'y', 'faceFirst', 'headingDeg', 'points', 'inches', 'mechanism', 'preset', 'up', 'down', 'to', 'bottom',
                   'position', 'volts', 'forS', 'boundS', 'seconds'];
    for (const k of order) {
        if (step[k] === undefined) {
            continue;
        }
        if (NUMBER_FIELDS.includes(k)) {
            const v = round3(Number(step[k]));
            if (!Number.isNaN(v) && step[k] !== null && step[k] !== '') {
                out[k] = v;  // an empty or unreadable number is left OUT: the checks ask for it by name
            } else if (TARGET_KEYS.includes(k)) {
                out[k] = null;  // "go up _ in" before the number is typed: WHICH move it is stays (the checks ask for it)
            }
        } else if (k === 'headingDeg') {
            const v = round3(wrapDeg(round3(Number(step[k]))));
            if (!Number.isNaN(v) && step[k] !== null && step[k] !== '') {
                out[k] = v;
            }
        } else if (k === 'points') {
            // A coordinate that is not a number is left OUT (never NaN, which JSON writes as null and reads back
            // as 0): the point keeps its hole, RC26 asks for it, and save -> open is the identity (RT6).
            out.points = Array.isArray(step.points) ? step.points.map((p) => {
                const q = {};
                for (const c of ['x', 'y']) {
                    const v = blank(p[c]) ? Number.NaN : round3(Number(p[c]));
                    if (!Number.isNaN(v)) {
                        q[c] = v;
                    }
                }
                if (!blank(p.headingDeg)) {
                    const v = round3(wrapDeg(round3(Number(p.headingDeg))));
                    if (!Number.isNaN(v)) {
                        q.headingDeg = v;
                    }
                }
                return q;
            }) : step.points;
        } else if (k === 'preset') {
            out.preset = typeof step.preset === 'number' ? Math.trunc(step.preset) : String(step.preset);
        } else if (k === 'faceFirst') {
            out.faceFirst = !!step.faceFirst;
        } else if (k === 'bottom') {
            if (step.bottom) {
                out.bottom = true;
            }
        } else {
            out[k] = step[k];
        }
    }
    for (const k of Object.keys(step)) {
        if (!(k in out) && k !== 'care' && k !== 'label' && k !== 'view' && step[k] !== undefined) {
            out[k] = step[k];
        }
    }
    const care = roundCare(step.care);
    if (care) {
        out.care = care;
    }
    // RMP1-H: the structures this step touches on purpose (page-only: no export reads it), sorted, each once
    delete out.touches;
    const touches = Array.isArray(step.touches) ? [...new Set(step.touches.filter((t) => typeof t === 'string' && t))].sort() : [];
    if (touches.length) {
        out.touches = touches;
    }
    const view = normalizeView(step.view);
    if (view) {
        out.view = view;
    }
    return out;
}

/** A start as it is stored: {x, y, headingDeg}, rounded and wrapped; a part that is not a number is left OUT
 *  (RC3 asks for it) — never NaN, which JSON reads back as 0. */
export function normalizeStart(start) {
    if (start === null || start === undefined) {
        return null;
    }
    const out = {};
    for (const k of ['x', 'y', 'headingDeg']) {
        let v = blank(start[k]) ? Number.NaN : round3(Number(start[k]));
        if (k === 'headingDeg' && !Number.isNaN(v)) {
            v = round3(wrapDeg(v));
        }
        if (!Number.isNaN(v)) {
            out[k] = v;
        }
    }
    return out;
}

/** The start as a pose — {x, y, headingDeg}, every part a number — or null. A start missing a part (a hand-edited
 *  file's `null`, a key left out) is no place the robot can be: nothing is planned or drawn from it, the step list
 *  shows "?" for the part and RC3 asks for it. */
export function startPose(start) {
    return start && [start.x, start.y, start.headingDeg].every(Number.isFinite) ? start : null;
}

/** The route with every step and the start normalized (what a loaded or edited route becomes). */
export function normalizeRoute(route) {
    return { ...route, start: normalizeStart(route.start), steps: route.steps.map(normalizeStep) };
}

// ── the edits (each returns a NEW route) ────────────────────────────────────────────────────────────

/** Place (or move) the start. */
export function setStart(route, start) {
    return { ...route, start: normalizeStart(start) };
}

/** Insert `step` at `index` (default: the end). */
export function addStep(route, step, index = route.steps.length) {
    const steps = route.steps.slice();
    steps.splice(Math.max(0, Math.min(index, steps.length)), 0, normalizeStep(step));
    return { ...route, steps };
}

/** Replace step `index` with `{...old, ...patch}` (a `care` patch merges into the old care). */
export function updateStep(route, index, patch) {
    const steps = route.steps.slice();
    const old = steps[index];
    const merged = { ...old, ...patch };
    if (patch.care !== undefined) {
        merged.care = patch.care === null ? undefined : { ...(old.care || {}), ...patch.care };
    }
    steps[index] = normalizeStep(merged);
    return { ...route, steps };
}

/** Remove step `index` (a section it started stays, from the step after: step_view.mjs passSection). */
export function removeStep(route, index) {
    const steps = route.steps.slice();
    passSection(steps, index, steps.splice(index, 1)[0]);
    return { ...route, steps };
}

/** Move step `from` to position `to` (the index it will have afterwards); a section it started stays where it was,
 *  and a folded section it lands in unfolds. */
export function moveStep(route, from, to) {
    const at = Math.max(0, Math.min(to, route.steps.length - 1));
    if (at === from) {
        return route;
    }
    const steps = route.steps.slice();
    const s = passSection(steps, from, steps.splice(from, 1)[0]);
    steps.splice(at, 0, s);
    return unfoldAt({ ...route, steps }, at);
}

/** Duplicate step `index` just after it (its label is not copied: labels are unique; nor its view: a copy shows). */
export function duplicateStep(route, index) {
    const copy = { ...route.steps[index] };
    delete copy.label;
    delete copy.view;
    return addStep(route, copy, index + 1);
}

/** Rename the route (the auton's name on the brain's menu). */
export function renameRoute(route, name) {
    return { ...route, name: String(name) };
}

// ── the plan: where each step leaves the robot ─────────────────────────────────────────────────────

/** A pose {x, y, headingDeg}. */
function pose(x, y, headingDeg) {
    return { x, y, headingDeg: wrapDeg(headingDeg) };
}

/**
 * The planned pose before and after every step, from the start (null everywhere when there is no start, or the
 * start is missing a part: startPose).
 * A drive ends at its point facing its bearing from where it began (+180 reversing); a turn at its heading;
 * a face at the bearing; a curve at its last point, facing that point's heading or the path's own end
 * direction (the library's CurvePath, built from the planned pose); a mechanism step, a wait and a
 * waitUntilDone where the robot stood. These are the PLAN — where the library is told to go — never a
 * claim about where the robot will be (that is the preview's, a later run).
 *
 * Each entry: {from, to, bearingDeg?, offDeg?, curve?}. `offDeg` (a drive) is how far the leg starts off its
 * own line when it is NOT turned first (RC24); `curve` is the built CurvePath.
 */
export function plannedPoses(route, curveCfg = kDefaultCurveConfig) {
    const out = [];
    const s0 = startPose(route.start);
    let at = s0 ? pose(s0.x, s0.y, s0.headingDeg) : null;
    for (const step of route.steps) {
        const entry = { from: at, to: at };
        if (at !== null) {
            switch (step.do) {
                case 'driveTo':
                case 'reverseTo': {
                    const b = bearingDeg(at.x, at.y, step.x, step.y);
                    if (Number.isNaN(b)) {
                        entry.to = pose(step.x, step.y, at.headingDeg);
                        break;
                    }
                    const leg = step.do === 'reverseTo' ? wrapDeg(b + 180) : b;
                    entry.bearingDeg = leg;
                    entry.offDeg = Math.abs(wrapDeg(leg - at.headingDeg));
                    entry.to = pose(step.x, step.y, leg);
                    break;
                }
                case 'forward':
                case 'backward': {
                    const q = relativePoint(at, step);
                    if (q) {
                        entry.point = q;
                        const b = bearingDeg(at.x, at.y, q.x, q.y);
                        const leg = Number.isNaN(b) ? at.headingDeg : step.do === 'backward' ? wrapDeg(b + 180) : b;
                        entry.bearingDeg = leg;
                        entry.offDeg = Math.abs(wrapDeg(leg - at.headingDeg));
                        entry.to = pose(q.x, q.y, leg);
                    }
                    break;
                }
                case 'turnTo':
                    entry.to = pose(at.x, at.y, step.headingDeg);
                    break;
                case 'face':
                case 'faceAway': {
                    const b = bearingDeg(at.x, at.y, step.x, step.y);
                    if (!Number.isNaN(b)) {
                        entry.to = pose(at.x, at.y, step.do === 'faceAway' ? b + 180 : b);
                    }
                    break;
                }
                case 'curveTo':
                case 'reverseCurveTo': {
                    if (Array.isArray(step.points) && step.points.length >= 1 && step.points.length <= 16
                        && step.points.every((p) => Number.isFinite(p.x) && Number.isFinite(p.y))) {
                        const path = new CurvePath(step.points, step.do === 'reverseCurveTo').build(at, curveCfg);
                        entry.curve = path;
                        const e = path.endPose();
                        entry.to = pose(e.x, e.y, e.headingDeg);
                    }
                    break;
                }
                default:
                    break;
            }
        }
        out.push(entry);
        at = entry.to;
    }
    return out;
}

/** Where a "drive forward / back up N in" step goes from the planned pose `at`: the point N in ahead of (behind) it,
 *  on the stored grid — what the export writes as its driveTo / reverseTo point. null without a pose or a number. */
export function relativePoint(at, step) {
    if (!at || ![at.x, at.y, at.headingDeg, step.inches].every(Number.isFinite)) {
        return null;  // from a pose with a hole (a step before it has an error): nowhere, never NaN
    }
    const d = step.do === 'backward' ? -step.inches : step.inches;
    const rad = (at.headingDeg * Math.PI) / 180;
    return { x: round3(at.x + d * Math.cos(rad)), y: round3(at.y + d * Math.sin(rad)) };
}

/** The distance (in) of a drive or face target from where the plan has the robot before it. */
export function targetDistance(entry, step) {
    if (!entry.from || step.x === undefined) {
        return Number.NaN;
    }
    return dist(entry.from.x, entry.from.y, step.x, step.y);
}

/** True when every number the route stores is on the stored grid (the W7 rule; NF1 reads it). A hole (a part left
 *  OUT: a start's or a curve point's) stores no number, so it is not one off the grid. */
export function everyNumberStored(route) {
    const nums = [];
    const part = (obj, k) => {
        if (obj[k] !== undefined) {
            nums.push(obj[k]);
        }
    };
    if (route.start) {
        for (const k of ['x', 'y', 'headingDeg']) {
            part(route.start, k);
        }
    }
    for (const s of route.steps) {
        for (const k of NUMBER_FIELDS) {
            if (s[k] !== undefined && s[k] !== null) {  // null: a target not typed yet (a hole, not a number)
                nums.push(s[k]);
            }
        }
        if (s.headingDeg !== undefined) {
            nums.push(s.headingDeg);
        }
        for (const p of s.points || []) {
            for (const k of ['x', 'y', 'headingDeg']) {
                part(p, k);
            }
        }
        for (const f of CARE_MOTION_FIELDS) {
            if (s.care && s.care[f] !== undefined) {
                nums.push(s.care[f]);
            }
        }
    }
    return nums.every(isStored);
}
