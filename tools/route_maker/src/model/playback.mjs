// playback.mjs — a PREVIEW's run, ready to play (RMP1-A2u; brief D5, D13 (c), A.3.10). The engine's record (schema 1,
// tools/route_maker/README.md) holds the robot's TRUE pose every 10 ms and each page step's first and last pace; this
// turns it into what the screen draws: where each robot is at time t, which step is playing, each step's verdict in
// plain words beside the library's own lines, and the marks on the timeline. Nothing here guesses a pose: every one
// comes from the record (landmine 1). Pure: no DOM.

import { normalizeRoute, MOTION_KINDS, MECHANISM_KINDS } from './route.mjs';

const RAD = 180 / Math.PI;

/** What a preview is OF: the route as canonical text (keys sorted), so an undo back to it finds its run again. */
export function routeKey(route) {
    const canon = (v) => (Array.isArray(v) ? `[${v.map(canon).join(',')}]`
        : v && typeof v === 'object' ? `{${Object.keys(v).filter((k) => v[k] !== undefined).sort().map((k) => `${JSON.stringify(k)}:${canon(v[k])}`).join(',')}}`
            : JSON.stringify(v));
    return canon(normalizeRoute(route));
}

/** Each page-step verdict of the record: its colour on the screen and its plain words. */
export const VERDICTS = {
    ok: { state: 'ok', words: 'Done' },
    failed: { state: 'failed', words: 'Failed: the route stopped here' },
    failedWentOn: { state: 'failed', words: 'Failed, and the route went on' },
    threw: { state: 'failed', words: 'The program stopped here' },
    looping: { state: 'failed', words: 'Never let time pass: stopped' },
    cut: { state: 'cut', words: 'Cut: the run\'s time was up' },
    skipped: { state: 'skipped', words: 'Skipped: the route had stopped' },
    jumped: { state: 'skipped', words: 'Skipped over: the route skipped ahead' },
    notReached: { state: 'skipped', words: 'Not reached' },
};

/** The index of the last sample at or before `t` (0 before the first). */
export function sampleIndexAt(samples, t) {
    let lo = 0;
    let hi = samples.length - 1;
    if (hi < 0 || t <= samples[0][0]) {
        return 0;
    }
    while (lo < hi) {
        const mid = (lo + hi + 1) >> 1;
        if (samples[mid][0] <= t) {
            lo = mid;
        } else {
            hi = mid - 1;
        }
    }
    return lo;
}

/** The robot's pose at `t` (inches, degrees), straight between two paces; held at the ends. */
export function poseAt(samples, t) {
    const i = sampleIndexAt(samples, t);
    const a = samples[i];
    const b = samples[Math.min(i + 1, samples.length - 1)];
    const k = b[0] > a[0] ? Math.min(Math.max((t - a[0]) / (b[0] - a[0]), 0), 1) : 0;
    return { x: a[1] + (b[1] - a[1]) * k, y: a[2] + (b[2] - a[2]) * k, headingDeg: (a[3] + (b[3] - a[3]) * k) * RAD };
}

/** The path a robot drove, thinned to a point every `step` in (each keeps its sample index): cheap to redraw. */
export function thinPath(samples, step = 0.25) {
    const out = [];
    samples.forEach((s, i) => {
        const last = out[out.length - 1];
        if (!last || Math.hypot(s[1] - last.x, s[2] - last.y) >= step || i === samples.length - 1) {
            out.push({ i, x: s[1], y: s[2] });
        }
    });
    return out;
}

const DID_NOT_RUN = new Set(['skipped', 'jumped', 'notReached']);
const STARTS = new Set([...MECHANISM_KINDS].filter((k) => k !== 'waitUntilDone'));  // the page steps that begin one

/**
 * Which page step each step seen from below belongs to (-1: none). A page step that took time owns what begins in
 * [first, last): its last pace is the next step's first. One that took no time (a start form, a drive refused at
 * once) owns what it began on its pace. A mechanism step goes BY NAME (`names[i]`, the page step's mechanism's
 * `.name`: RPV1 lists one pace's mechanism steps in the robot's slot order, not the code's), only to a page step
 * naming that mechanism; two starting it on one pace cannot be told apart, so neither takes it. A motion goes to the
 * first not yet owned (motions run in the code's order).
 */
export function ownersOf(rec, names = []) {
    const ps = rec.pageSteps || [];
    const ran = (p) => p.first !== null && !DID_NOT_RUN.has(p.verdict);
    const used = new Set();
    return rec.steps.map((s) => {
        if (s.endAction) {
            return -1;
        }
        const mech = s.kind === 'mechanism';
        const zero = ps.flatMap((p, k) => (ran(p) && p.first === p.last && p.first === s.s0 && !used.has(k)
            && (mech ? STARTS.has(p.do) && names[k] === s.what : MOTION_KINDS.has(p.do)) ? [k] : []));
        if (mech && zero.length > 1) {
            return -1;
        }
        if (zero.length) {
            used.add(zero[0]);
            return zero[0];
        }
        return ps.findIndex((p, k) => ran(p) && p.first !== p.last && s.s0 >= p.first && s.s0 < p.last
            && (!mech || (MECHANISM_KINDS.has(p.do) && names[k] === s.what)));
    });
}

/** The library's own lines for page step `i`: the routine's lines naming its steps, and its failed steps' sentences. */
function libraryLines(rec, p, mine) {
    const out = [];
    if (p.lib) {
        for (const l of rec.lines) {
            const m = l.sub === 'RTN' && /\bstep (\d+) \(/.exec(l.text);
            if (m && Number(m[1]) >= p.lib[0] && Number(m[1]) <= p.lib[1]) {
                out.push(l.text);
            }
        }
    }
    for (const s of mine) {
        if (s.mark === 'failed' && s.sentence && !out.includes(s.sentence)) {
            out.push(s.sentence);
        }
    }
    if (p.verdict === 'threw' && rec.why) {
        out.push(rec.why);
    }
    if (p.verdict === 'looping' && rec.loopSentence) {
        out.push(rec.loopSentence);
    }
    return out;
}

/** The page step a pace belongs to: its span [first, last) holds it (the last step: [first, last]); -1 after it. */
function stepOfPace(ps, s) {
    const i = ps.findIndex((p) => p.first !== null && s >= p.first && s < p.last);
    return i >= 0 ? i : ps.findLastIndex((p) => p.first !== null && s >= p.first && s <= p.last);
}

/** A field note (RPV1's field check) in plain words, its step the PAGE's (`f.at`), never RPV1's from-below number. */
export function noteWords(f) {
    const at = `at ${f.t.toFixed(2)} s, ${f.at >= 0 ? `in step ${f.at + 1}` : 'after the last step'}`;
    return f.kind === 'THROUGH'
        ? `On the robot it hits the ${f.element} ${at} (${f.depth.toFixed(2)} in past its face): the host test plant has no walls.`
        : f.kind === 'TOUCH' ? `Touches the ${f.element} ${at}: normal, not a failure.`
            : `Drives over a ${f.element} ${at} (the host test plant does not push pieces).`;
}

const AFTER_THROUGH = 'After the THROUGH: the robot would not get here';

/** One robot's run: its samples, its thinned path, its page steps with times, verdicts and words, its field notes. */
function runOf(id, rec, names) {
    const t = (i) => (i === null || i === undefined ? null : rec.samples[i][0]);
    const owner = ownersOf(rec, names);
    const ps = rec.pageSteps || [];
    const notes = (rec.field || []).map((f) => ({ ...f, at: stepOfPace(ps, f.s) }));
    const hit = notes.find((f) => f.kind === 'THROUGH') || null;  // the first is the run's THROUGH (RPV1)
    const steps = ps.map((p, i) => {
        const v = VERDICTS[p.verdict] || VERDICTS.notReached;
        const mine = rec.steps.filter((s, k) => owner[k] === i);
        const ph = mine.find((s) => s.placeholder);
        const step = { n: p.n, do: p.do, verdict: p.verdict, state: v.state, words: v.words, t0: t(p.first), t1: t(p.last),
                       library: libraryLines(rec, p, mine), placeholder: ph ? ph.placeholder : '' };
        if (!hit || hit.at < 0 || i < hit.at || step.t0 === null) {
            return step;
        }
        return i === hit.at ? { ...step, state: 'failed', words: `Went THROUGH the ${hit.element}`, library: [noteWords(hit), ...step.library] }
            : { ...step, state: 'skipped', words: AFTER_THROUGH };
    });
    return { id, record: rec, samples: rec.samples, path: thinPath(rec.samples), steps, notes, through: hit,
             headline: headlineOf(rec, steps, hit) };
}

/** The headline of a record: the Auton Tester's word (and a THROUGH, as RPV1 says it) and how it reads. */
export function headlineOf(rec, steps, hit = null) {
    const failed = rec.refused || rec.threw || rec.looped || rec.through || steps.some((s) => s.state === 'failed');
    return { word: hit ? `${rec.word}, but THROUGH the ${hit.element}` : rec.word, mark: failed ? 'failed' : rec.cut ? 'cut' : 'ok' };
}

const RANK = { ok: 0, cut: 1, failed: 2 };

/**
 * The engine's result ({one, two?, verdict?}) ready to play: robot one's run (and robot two's, mirrored), the
 * duration, the run's own end times (the 57 s cut, the hard stop, the period) and the pair check's verdict.
 * `names` = {one, two}: each page step's mechanism `.name` on that robot (ownersOf). Robot two's verdict on a page
 * step is kept on robot one's (`.two`) where it went otherwise and not done; the headline is the worse of the two.
 */
export function playbackOf(result, names = {}) {
    const one = runOf('one', result.one, names.one);
    const two = result.two ? runOf('two', result.two, names.two) : null;
    let headline = one.headline;
    if (two) {
        one.steps.forEach((s, i) => {
            const q = two.steps[i];
            if (q && q.state !== s.state && q.state !== 'ok') {
                s.two = q;
            }
        });
        const [a, b] = [one.headline, two.headline];
        headline = { word: a.word, mark: RANK[b.mark] > RANK[a.mark] ? b.mark : a.mark,
                     two: b.word !== a.word || b.mark !== a.mark ? b.word : '' };
    }
    const last = (r) => (r && r.samples.length ? r.samples[r.samples.length - 1][0] : 0);
    const rec = result.one;
    return {
        one, two, pair: result.verdict || null,
        duration: Math.max(last(one), last(two)),
        park: rec.park, stop: rec.stop, period: rec.period,
        headline,
        owed: rec.owed || [],
    };
}

/** The page step playing at `t` (robot one's), or -1 before the first or after the last. */
export function stepAt(play, t) {
    let at = -1;
    play.one.steps.forEach((s, i) => {
        if (s.t0 !== null && t >= s.t0 && t < s.t1) {  // a step that took no time never plays
            at = i;
        }
    });
    return at;
}

/**
 * The start of the next (dir 1) or previous (dir -1) step from `t`, or the run's start or end. Only while it plays
 * does "previous" pass a start less than 50 ms behind it (as a player's does): paused, every start is a stop.
 */
export function stepTime(play, t, dir, playing = false) {
    const marks = [...new Set([0, ...play.one.steps.filter((s) => s.t0 !== null).map((s) => s.t0), play.duration])].sort((a, b) => a - b);
    const back = playing ? 0.05 : 1e-6;
    return dir > 0 ? marks.find((m) => m > t + 1e-6) ?? play.duration : [...marks].reverse().find((m) => m < t - back) ?? 0;
}
