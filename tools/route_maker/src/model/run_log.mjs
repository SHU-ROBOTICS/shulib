// run_log.mjs — WATCH THE RUN (RMP1-K D2): the brain's own lines, read into how each step of a run went.
//
// The formats are the library's and the Auton Tester's (never guessed): chassis/routine.hpp — `[RTN] routine 'x': step
// N (verb) done in S s[, at …]` (API 2.11, D4: the live tick), `[WARN][RTN] routine 'x' STOPPED at step N (verb): why — …`,
// `[WARN][RTN] routine 'x': step N (verb) FAILED: why — carrying on|skipping to …`, `[RTN] routine 'x': step N (verb)
// skipped — stopped at step M` and `… jumped — skipping to …`; mechanism/service.hpp — `[MECH] 'name': OUTCOME[ -- by]
// in S.SSs[ (note)]` (read onto the step that STARTED that mechanism); src/programs/auton_tester_program.cpp — `[ATS] run
// R: 'x' (…) -- RUN touched`; auton_tester_run.hpp — `[ATS] run R: 'x' WORD -- …` (the robot's verdict: FINISHED, DONE,
// n FAILED, ROUTINE STOPPED, ABORTED, …), `[ATS] run R: routine STOPPED, k of N steps done, stopped at verb (why)` (or
// `routine OK|DONE|STOPPED, k of N steps done[, f failed, j jumped]…`) and `[ATS] run R: seated (…) …; END estimate (x,
// y) facing h deg LIVE|NOT LIVE`. Run numbers start at 1 at every boot: a run's later lines go to the LAST run of that
// number. A line may carry the TermSink's `[t=   1.23] ` and a level tag, a pros terminal's \r and NULs, or be cut or
// garbled (an em dash in pieces too): it is read only when it matches, never half-read. Pure: no DOM.

import { fmt } from './numbers.mjs';

const RTN = /\[RTN\] routine '(.*?)'(?::)? (.*)$/;
const ATS = /\[ATS\] run (\d+): (.*)$/;
const STEP = /^step (\d+) \((.*?)\) (done|started) in (\d+(?:\.\d+)?) s(?:, at (.*?))?(?: \(([A-Z_ ]+)\))?$/;
const DASH = ' [^\\s\\w]{1,3} (?=[a-z])';  // the routine's " — ", whole or garbled into 1-3 other characters
const STOPPED = new RegExp(`^STOPPED at step (\\d+) \\((.*?)\\): (.*?)${DASH}`);
const FAILED = new RegExp(`^step (\\d+) \\((.*?)\\) FAILED: (.*?)${DASH}(carrying on|skipping to '.*')`);
const SKIPPED = new RegExp(`^step (\\d+) \\((.*?)\\) (skipped|jumped)${DASH}`);
const MECH = /\[MECH\] '(.*?)': ([A-Z_]+)(?: -- [a-z ]+)? in (\d+(?:\.\d+)?)s(?: \((.*)\))?$/;
const START = /^'(.*)' \(.*\) -- RUN touched/;
const VERDICT = /^'.*' (FINISHED|ROUTINE STOPPED|DONE, \d+\+? FAILED|ABORTED|PROGRAM ERROR|REFUSED|ABORT IGNORED|NOT STARTED|HARD STOP|CUT BY DEADLINE)(?: --| in the countdown)(?:.*, end action (succeeded|FAILED|skipped|CUT by the abort))?/;
const SUMMARY = /^(?:routine (\w+), )?(\d+) of (\d+) steps done(?:, stopped at (.+?) \(([^()]*)\))?(?:, (\d+) failed, (\d+) jumped)?/;
const END = /END estimate \((-?\d+(?:\.\d+)?), (-?\d+(?:\.\d+)?)\) facing (-?\d+(?:\.\d+)?) deg (LIVE|NOT LIVE)/;
const VARIANT = /\[shulib-robot-variant=(one|tank)-/;

/** A fresh reader's state: no run yet. */
export function newWatch() {
    return { robot: null, runs: [], lines: 0 };
}

/** One step's mark: {state: 'done'|'started'|'stopped'|'failed'|'skipped', verb, words, seconds?, at?, mech?} — `mech`:
 *  the mechanism's own end, read onto the step that started it (never the routine's failure). */
function mark(run, n, m) {
    const old = run.steps[n];
    if (old && (old.state === 'stopped' || old.state === 'failed') && m.state !== 'stopped') {
        return;  // the robot's own failure is never painted over
    }
    run.steps[n] = m;
}

/** A mechanism's end (its `[MECH]` outcome line) onto the step that STARTED it and is still open: a start is not a
 *  finished step until the mechanism says how it ended. */
function mechEnd(run, name, outcome, seconds, note) {
    const n = Object.keys(run.open).find((k) => run.open[k] === name || run.open[k].slice(0, 23) === name);
    if (n === undefined) {
        return;
    }
    delete run.open[n];
    const ok = outcome === 'SUCCEEDED' || (outcome === 'UNCONFIRMED' && /counted done/.test(note || ''));
    mark(run, Number(n), { ...run.steps[n], state: ok ? 'done' : 'failed', mech: true,
                           words: `started, then ${outcome} in ${seconds} s${note ? ` (${note})` : ''}` });
}

/** Read one more line of the brain's output into `w` (mutated, and returned); a line it does not know changes nothing. */
export function readLine(w, raw) {
    const line = String(raw).replace(/[\0\r]/g, '').replace(/\x1b\[[0-9;]*[A-Za-z]/g, '').trimEnd();
    w.lines += 1;
    const v = VARIANT.exec(line);
    if (v) {
        w.robot = v[1] === 'one' ? 'one' : 'two';
    }
    const a = ATS.exec(line);
    if (a) {
        const number = Number(a[1]);
        const s = START.exec(a[2]);
        if (s) {
            w.runs.push({ number, name: s[1], robot: w.robot, steps: {}, open: {}, live: false, verdict: null, endAction: null, summary: null, end: null,
                          ended: false });  // robot: the boot line's, when one was read (RMP1-M: a test's run is its robot's)
            return w;
        }
        const run = w.runs.findLast((r) => r.number === number);  // this boot's run N (each boot counts from 1 again)
        if (!run) {
            return w;
        }
        const v = VERDICT.exec(a[2]);
        if (v) {
            run.verdict = v[1];
            run.endAction = v[2] || null;
            return w;
        }
        const e = END.exec(a[2]);
        if (e) {
            run.end = { x: Number(e[1]), y: Number(e[2]), headingDeg: Number(e[3]), live: e[4] === 'LIVE' };
            run.ended = true;
            return w;
        }
        const sm = SUMMARY.exec(a[2]);
        if (sm) {
            run.summary = { word: sm[1] || null, done: Number(sm[2]), total: Number(sm[3]), stoppedVerb: sm[4] || null, why: sm[5] || null,
                            failed: Number(sm[6] || 0), jumped: Number(sm[7] || 0) };
            settle(run);
        }
        return w;
    }
    const run = w.runs[w.runs.length - 1];
    const mm = MECH.exec(line);
    if (mm && run && !run.summary) {
        mechEnd(run, mm[1], mm[2], mm[3], mm[4]);
        return w;
    }
    const r = RTN.exec(line);
    if (!r || !run || r[1] !== run.name || run.summary) {
        return w;
    }
    const body = r[2];
    let m;
    if ((m = STEP.exec(body))) {
        run.live = true;
        const started = m[3] === 'started';
        mark(run, Number(m[1]), { state: started ? 'started' : 'done', verb: m[2], words: m[3], seconds: Number(m[4]),
                                  at: m[5] || null, outcome: m[6] || null });
        if (started) {
            run.open[m[1]] = m[2];
        }
    } else if ((m = STOPPED.exec(body))) {
        mark(run, Number(m[1]), { state: 'stopped', verb: m[2], words: m[3] });
        run.stoppedAt = Number(m[1]);
    } else if ((m = FAILED.exec(body))) {
        mark(run, Number(m[1]), { state: 'failed', verb: m[2], words: m[3] });
    } else if ((m = SKIPPED.exec(body))) {
        mark(run, Number(m[1]), { state: 'skipped', verb: m[2], words: m[3] });
    }
    return w;
}

/** At the run's end (its "k of N steps done" line): every step not yet marked before the stop is done — how a run of a
 *  program older than API 2.11 (no live ticks) is read; only when that adds up to the robot's own k (a line lost — a
 *  stop never read — leaves the steps as the robot said them, never painted done). */
function settle(run) {
    const until = run.stoppedAt || run.summary.total + 1;
    const fill = [];
    for (let n = 1; n < until && n <= run.summary.total; ++n) {
        if (!run.steps[n]) {
            fill.push(n);
        }
    }
    const counted = Object.values(run.steps).filter((x) => x.state === 'done' || x.state === 'started' || x.mech).length;  // the robot's k
    if (counted + fill.length !== run.summary.done) {
        return;
    }
    fill.forEach((n) => { run.steps[n] = { state: 'done', verb: '', words: 'done' }; });
}

const routineFailed = (run) => Object.values(run.steps).filter((x) => x.state === 'failed' && !x.mech).length;

/** The run's one summary line, in the robot's own words first: "1 of 23 steps done — stopped at step 2, driveTo: motion
 *  TIMEOUT"; "The robot says ABORTED" for a run that never got to its count; null before either. */
export function summaryWords(run) {
    const s = run && run.summary;
    if (!s) {
        return run && run.verdict ? `The robot says ${run.verdict}` : null;
    }
    const at = run.stoppedAt ? run.steps[run.stoppedAt] : null;
    const failed = Math.max(s.failed, routineFailed(run));
    const stopped = at ? ` — stopped at step ${run.stoppedAt}, ${at.verb}: ${at.words}` : s.stoppedVerb ? ` — stopped at ${s.stoppedVerb}: ${s.why}`
        : s.word === 'STOPPED' ? ' — the robot says STOPPED' : '';
    const end = run.endAction === 'FAILED' ? ' — the end action FAILED'
        : !stopped && run.verdict && runTone({ verdict: run.verdict, steps: {} }) === 'bad' ? ` — the robot says ${run.verdict}` : '';
    return `${s.done} of ${s.total} steps done${stopped}${failed ? ` — ${failed} failed${stopped ? '' : ' and carried on'}` : ''}${end}`;
}

/**
 * How the run went, as one colour — the robot's own verdict first (its `run R: 'x' WORD` line, then its summary's word),
 * then the marks: 'bad' (stopped, aborted, any word but FINISHED / DONE), 'warn' (DONE, n FAILED: carried on), 'ok'
 * (FINISHED / OK, nothing failed), '' (not over yet). A lost or garbled stop line never turns a stopped run green.
 */
export function runTone(run) {
    if (!run || (!run.verdict && !run.summary)) {
        return '';
    }
    const rank = { '': 0, ok: 1, warn: 2, bad: 3 };
    const of = (word) => (!word ? '' : word === 'FINISHED' || word === 'OK' ? 'ok' : /^DONE\b/.test(word) ? 'warn' : 'bad');
    const s = run.summary;
    const marks = run.stoppedAt ? 'bad' : routineFailed(run) || (s && s.failed) ? 'warn' : 'ok';
    return [of(run.verdict), of(s && s.word), marks].reduce((a, b) => (rank[b] > rank[a] ? b : a), '');
}

/**
 * The marks of `run` on a route's page steps: [{state, words, lib}] per page step (null: no word yet). `calls[i]`: how many
 * library steps page step i is (export.mjs libraryCalls). A step is its worst call's mark (stopped, failed, skipped,
 * started, then done), and done only when every call is.
 */
export function pageMarks(run, calls) {
    const out = [];
    let lib = 1;
    const rank = { stopped: 5, failed: 4, skipped: 3, started: 2, done: 1 };
    for (const c of calls) {
        const ms = Array.from({ length: c }, (_, k) => run.steps[lib + k]);
        lib += c;
        const worst = ms.filter(Boolean).sort((a, b) => rank[b.state] - rank[a.state])[0];
        out.push(!worst || (worst.state === 'done' && ms.some((x) => !x)) ? null : { ...worst, lib: lib - c });
    }
    return out;
}

/** The verb the brain's log gives each of `route`'s library steps, in order (export.mjs writes one call per step, two for
 *  a turned-first drive; a mechanism step is named by its mechanism, a wait by its seconds). */
export function routeVerbs(route, robot) {
    const name = (ident) => (((robot && robot.mechanisms) || []).find((m) => m.ident === ident) || { name: ident }).name;
    return route.steps.flatMap((st) => {
        switch (st.do) {
            case 'driveTo':
            case 'reverseTo':
                return st.faceFirst ? [st.do === 'driveTo' ? 'face' : 'faceAway', st.do] : [st.do];
            case 'forward':
                return ['driveTo'];
            case 'backward':
                return ['reverseTo'];
            case 'wait':
                return [`wait ${fmt(st.seconds)} s`];
            default:
                return [st.mechanism !== undefined ? name(st.mechanism) : st.do];
        }
    });
}

/** Null when `run` could be of `verbs` (routeVerbs()), else the words: the robot ran another version of the route. */
export function runDiffers(run, verbs) {
    for (const [n, m] of Object.entries(run.steps)) {
        const want = verbs[Number(n) - 1];
        if (want === undefined || (m.verb && m.verb !== want)) {
            return `The robot ran another version of this route (its step ${n} is ${m.verb || 'more than this route has'}, `
                + `here it is ${want || 'not there'}): send this one, then run it again.`;
        }
    }
    if (run.summary && run.summary.total !== verbs.length) {
        return `The robot ran another version of this route (${run.summary.total} steps; this one has ${verbs.length}): send this one, then run it again.`;
    }
    return null;
}
