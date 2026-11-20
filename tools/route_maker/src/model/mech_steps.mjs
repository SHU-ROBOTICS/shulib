// mech_steps.mjs — RMP1-J: the rules behind the simple mechanism steps. Pure: no DOM.
// D1 the menu's two steps, D2 one height (it writes today's step kinds), D3 where the route waits for a started
// step, D4 a time limit suggested from PREVIEW's own run (never written without the click), D6 the home card.

import { normalizeRoute, MOTION_KINDS } from './route.mjs';
import { moveTarget, unitOf, plannedHeights } from './lift.mjs';
import { round3, fmt } from './numbers.mjs';

const cap = (s) => `${s[0].toUpperCase()}${s.slice(1)}`;
const KIND_ICON = { lift: 'lift', arm: 'arm', pneumatic: 'piston', power: 'motor', flywheel: 'motor' };
const STARTS = new Set(['startMove', 'startHome', 'startRun']);
/** A step kind and the one that does the same without waiting (D3). */
export const WAIT_PAIR = { move: 'startMove', startMove: 'move', run: 'startRun', startRun: 'run', home: 'startHome', startHome: 'home' };

/** A mechanism's kind as an icon's name (dom.mjs). */
export const kindIcon = (m) => KIND_ICON[m.kind] || 'mechanism';

/** D1, the first step: one row per mechanism — its name and its kind's icon. */
export function menuRows(robot) {
    return ((robot && robot.mechanisms) || []).map((m) => ({ ident: m.ident, name: cap(m.name), kind: m.kind, icon: kindIcon(m) }));
}

/**
 * D1, the second step: what `m` does, from today's `actions` (words.mjs `mechanismActions`), each kind of step ONCE —
 * every height move is one "Move the cascade" (D2 asks where), a position keeps its own button. "Wait for it" only
 * when a step before `at` starts it without waiting (else the wait has nothing to wait for: RC17).
 */
export function menuActions(m, actions, route, at) {
    const started = route.steps.slice(0, at).some((s) => s.mechanism === m.ident && STARTS.has(s.do));
    const out = new Map();
    for (const a of actions) {
        const d = a.step.do;
        const key = d === 'set' ? `set:${a.step.position}` : d;
        if (out.has(key) || (d === 'waitUntilDone' && !started)) {
            continue;
        }
        const words = { move: `Move the ${m.name}`, home: 'Home it', run: 'Run it', waitUntilDone: 'Wait for it' }[d] || a.words;
        out.set(key, { words, icon: a.icon === 'mechanism' ? kindIcon(m) : a.icon, step: d === 'move' ? { mechanism: m.ident, do: 'move' } : a.step });
    }
    return [...out.values()];
}

const CLEAR = { preset: undefined, up: undefined, down: undefined, to: undefined, bottom: undefined };

/** D2: what a height choice writes — {level} (a preset), {bottom: true}, or {to} (a number; none: a hole to type). */
export function heightPatch(pick) {
    if (pick.level) {
        return { ...CLEAR, preset: pick.level.ledger !== undefined ? pick.level.ledger : pick.level.name };
    }
    if (pick.bottom) {
        return { ...CLEAR, bottom: true };
    }
    return { ...CLEAR, to: Number.isFinite(pick.to) ? round3(pick.to) : null };
}

/** D2: the height a click at `frac` (0 the bottom, 1 the top) of `m`'s travel picks: 0.5 in, or 1 deg, inside it. */
export function barHeight(m, frac) {
    const [lo, hi] = m.range;
    const grid = unitOf(m) === 'deg' ? 1 : 0.5;
    const v = Math.round((lo + Math.min(Math.max(frac, 0), 1) * (hi - lo)) / grid) * grid;
    return round3(Math.min(Math.max(v, lo), hi));
}

/** D2: what a click on the bar picks — a measured level when it lands on its mark (3% of the travel), else a height. */
export function barPick(m, frac) {
    const [lo, hi] = m.range;
    const at = lo + Math.min(Math.max(frac, 0), 1) * (hi - lo);
    const near = (m.presets || []).filter((p) => !p.owed && Number.isFinite(p.value) && Math.abs(p.value - at) <= 0.03 * (hi - lo))
        .sort((a, b) => Math.abs(a.value - at) - Math.abs(b.value - at))[0];
    return near ? { level: near } : { to: barHeight(m, frac) };
}

/** D2: an old project's "up 6" / "down 6" as a height — the patch, or null while where it starts is not known. */
export function asHeight(route, robot, i) {
    const step = route.steps[i];
    const m = ((robot && robot.mechanisms) || []).find((x) => x.ident === step.mechanism);
    const before = m ? plannedHeights(route, robot)[i].before[m.ident] : null;
    const to = m && (step.up !== undefined || step.down !== undefined) ? moveTarget(step, m, before) : null;
    return to === null ? null : heightPatch({ to });
}

/**
 * D3: where the route waits for started step `i` — {wait: j} (a later "wait for it"), {needed: j} (step j uses the
 * mechanism again first: the fix puts a wait before it), or {} (nothing does: if the route ends first, the library
 * stops it — RunGuard cancels every mechanism when the routine returns).
 */
export function waitsFor(route, i) {
    const m = route.steps[i].mechanism;
    for (let j = i + 1; j < route.steps.length; ++j) {
        const s = route.steps[j];
        if (s.mechanism === m) {
            return s.do === 'waitUntilDone' ? { wait: j } : { needed: j };
        }
    }
    return {};
}

// ── D4: the time limit, from PREVIEW's run ─────────────────────────────────────────────────────────────────

const BOUNDED = new Set(['move', 'startMove', 'home', 'startHome', 'waitUntilDone']);
/** The margin PREVIEW's time gets (D4). */
export const MARGIN_S = 0.5;
/** Which time limit step `step` has: 'boundS' (a mechanism's), 'timeoutS' (a motion's care), or null. */
export const limitOf = (step) => (BOUNDED.has(step.do) ? 'boundS' : MOTION_KINDS.has(step.do) ? 'timeoutS' : null);
/** PREVIEW's time + 0.5 s, up to the next 0.1 s. */
export const suggestS = (took) => Math.ceil(Math.round((took + MARGIN_S) * 1e6) / 1e5) / 10;

const canon = (v) => (Array.isArray(v) ? `[${v.map(canon).join(',')}]`
    : v && typeof v === 'object' ? `{${Object.keys(v).filter((k) => v[k] !== undefined).sort().map((k) => `${JSON.stringify(k)}:${canon(v[k])}`).join(',')}}`
        : JSON.stringify(v));
/** The route up to step `i` as the robot runs it, time limits aside (`sameRun` checks those). */
function headOf(route, i) {
    const r = normalizeRoute(route);
    const strip = ({ boundS, touches, view, care, ...rest }) => {
        const { timeoutS, ...c } = care || {};
        return Object.keys(c).length ? { ...rest, care: c } : rest;
    };
    return canon({ robot: r.robot, start: r.start, steps: r.steps.slice(0, i + 1).map(strip) });
}

const limitIn = (step) => (limitOf(step) === 'boundS' ? step.boundS : (step.care || {}).timeoutS);

/** Earlier step `k`'s limit changed since the run, but the step still runs as it ran: it finished, within the new one
 *  (an empty mechanism limit: the cut, as PREVIEW ran it; an empty drive limit: the robot's own, not known here). */
function sameRun(run, route, k) {
    const was = limitIn(run.route.steps[k]);
    const now = limitIn(route.steps[k]);
    if (was === now || (!Number.isFinite(was) && !Number.isFinite(now))) {
        return true;
    }
    const t = run.play.one.steps[k] && run.play.one.steps[k].took;
    const cap = Number.isFinite(now) ? now : limitOf(route.steps[k]) === 'boundS' ? Infinity : NaN;
    return !!t && t.ok && cap > t.s;
}

/**
 * D4: what PREVIEW's last `run` ({route, play}) says about step `i`'s time limit — {took, s, placeholder, wait?,
 * afterDrive?} (it finished: `s` is the suggestion; a wait: whether a drive, or a PLACEHOLDER step, ran since the start it collects),
 * {ranOut: true} (its limit stopped it), {notRun: why} (this route's run did not finish it), {untimed: together} (it
 * ran, but a start with no library step of its own: `together`, another start of its mechanism on its pace), or null
 * (no run of this route up to this step). Never a number the run did not measure.
 */
export function limitHint(run, route, i) {
    const step = route.steps[i];
    if (!run || !step || !limitOf(step) || !run.route || !run.play || !run.play.one.steps[i] || headOf(run.route, i) !== headOf(route, i)
        || route.steps.slice(0, i).some((x, k) => !sameRun(run, route, k))) {
        return null;
    }
    const s = run.play.one.steps[i];
    const t = s.took;
    if (!t && s.state === 'ok') {
        const ps = run.play.one.steps;
        return { untimed: route.steps.some((x, k) => k !== i && x.mechanism === step.mechanism && STARTS.has(x.do) && ps[k] && ps[k].t0 === s.t0) };
    }
    if (!t || (!t.ok && !t.ranOut)) {
        return { notRun: !t ? s.words : t.verdict === 'CANCELLED' ? 'it was stopped before it was done' : s.state !== 'ok' ? s.words : `the library ended it ${t.verdict}` };
    }
    if (!t.ok) {
        return { ranOut: true };
    }
    if (step.do !== 'waitUntilDone') {
        return { took: t.s, s: suggestS(t.s), placeholder: !!s.placeholder };
    }
    const k = route.steps.slice(0, i).findLastIndex((x) => x.mechanism === step.mechanism);  // the start it collects
    const since = k < 0 ? [] : [...route.steps.keys()].slice(k + 1, i);  // its time is theirs too
    return { took: t.s, s: suggestS(t.s), placeholder: !!s.placeholder || since.some((j) => !!(run.play.one.steps[j] || {}).placeholder), wait: true,
             afterDrive: since.some((j) => MOTION_KINDS.has(route.steps[j].do)) };
}

/**
 * D4's line, in words: {words, use} — `use` the one-click limit (null: none to offer; `inUse`: it is the limit already),
 * or null when there is nothing to say. `drive`: a motion's limit (empty is the robot's own); `current`: the limit now;
 * `name`: the step's mechanism's.
 */
export function limitWords(hint, { drive, current, previewable, name }) {
    if (!hint) {
        return previewable && !drive && !Number.isFinite(current) ? { words: 'Run PREVIEW to get a suggestion.', use: null } : null;
    }
    if (hint.notRun) {
        return { words: `PREVIEW did not finish this step (${hint.notRun}): no suggestion.`, use: null };
    }
    if (hint.untimed !== undefined) {
        return { words: `PREVIEW ran it but cannot time it on its own${hint.untimed ? ` (another step starts the ${name || 'mechanism'} at the same moment)` : ''}: no suggestion.`, use: null };
    }
    if (hint.ranOut) {
        return { words: drive && !Number.isFinite(current) ? 'In PREVIEW the robot\'s own time limit stopped it: type a longer one and run PREVIEW again.'
            : 'In PREVIEW its time limit stopped it: give it longer and run PREVIEW again.', use: null };
    }
    const took = fmt(Math.round(hint.took * 100) / 100);
    const why = [hint.placeholder && 'on PLACEHOLDER numbers', hint.afterDrive && 'after the stand-in drive, not your robot'].filter(Boolean).join(', ');
    return { words: drive ? `PREVIEW's stand-in drive (not your robot) took ${took} s here, + ${MARGIN_S} s: time it on the robot too.`
        : `PREVIEW ${hint.wait ? 'waited' : 'took'} ${took} s here on the host test plant${why ? ` (${why})` : ''}, + ${MARGIN_S} s${hint.afterDrive ? ': time it on the robot too' : ''}.`,
    use: hint.s, inUse: current === hint.s };
}

/** For PREVIEW: the route with every EMPTY mechanism time limit set to the run's cut (it measures them; the route
 *  itself keeps them empty and Export still refuses it) — {route, filled}: how many were empty. */
export function measureEmptyLimits(route) {
    let filled = 0;
    const steps = route.steps.map((s) => {
        if (BOUNDED.has(s.do) && !Number.isFinite(s.boundS)) {
            ++filled;
            return { ...s, boundS: route.endActionAtS };
        }
        return s;
    });
    return { route: { ...route, steps }, filled };
}

// ── D6: the home card ─────────────────────────────────────────────────────────────────────────────────────

/**
 * D6: step `i` is the FIRST height move of an arm or a lift that must find its zero (stall or switch) in a route that
 * never homes it: the one-click fix (today's "home while the first leg drives"), else null.
 */
export function homeCard(route, robot, i) {
    const step = route.steps[i];
    const m = ((robot && robot.mechanisms) || []).find((x) => x.ident === step.mechanism);
    if (!m || !(m.homing === 'stall' || m.homing === 'switch') || !(step.do === 'move' || step.do === 'startMove')) {
        return null;
    }
    const mine = route.steps.map((s, k) => (s.mechanism === m.ident ? k : -1)).filter((k) => k >= 0);
    const homed = mine.some((k) => route.steps[k].do === 'home' || route.steps[k].do === 'startHome');
    const first = mine.find((k) => route.steps[k].do === 'move' || route.steps[k].do === 'startMove');
    return homed || first !== i ? null : { m, fix: { kind: 'homeWhileDriving', stepIndex: i, mechanism: m.ident } };
}
