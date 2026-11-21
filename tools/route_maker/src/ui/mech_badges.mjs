// mech_badges.mjs — RMP1-J D5: each mechanism step on the field where it happens: a badge (its number, its icon, one
// word) at the robot's pose when it starts, stacked above that spot's number pill; a click selects the step. At the
// start, the stack sits past the start robot, away from its heading knob (both stay draggable). A step that keeps
// driving draws a thin band along the path to the step that waits for it. Hidden: not drawn; locked: clicks pass
// through; focus fades the others (RMP1-G).

import { s, iconPath } from './dom.mjs';
import { stepIcon, stepTitle } from './words.mjs';
import { MECHANISM_KINDS, startPose } from '../model/route.mjs';
import { isHidden, isLocked } from '../model/step_view.mjs';
import { stepPoses } from '../model/pair.mjs';
import { waitsFor, kindIcon } from '../model/mech_steps.mjs';
import { targetKind, unitOf, homeEnd } from '../model/lift.mjs';
import { fmt } from '../model/numbers.mjs';

const finite = (q) => !!q && Number.isFinite(q.x) && Number.isFinite(q.y);

/** A badge's words: "close", "18 in", "level 2", "find bottom", "wait", "run" (RMP1-M D5: never "home"). */
export function badgeWord(step, m) {
    switch (step.do) {
        case 'set':
            return { closed: 'close', open: 'open' }[step.position] || step.position || '?';
        case 'move':
        case 'startMove': {
            const tk = targetKind(step);
            const u = unitOf(m);
            return tk === 'to' && Number.isFinite(step.to) ? `${fmt(step.to)} ${u}` : tk === 'bottom' ? 'bottom'
                : tk === 'preset' ? `level ${step.preset}` : (tk === 'up' || tk === 'down') && Number.isFinite(step[tk]) ? `${tk} ${fmt(step[tk])}` : 'move';
        }
        case 'home':
        case 'startHome':
            return homeEnd(m) === 'zero' ? 'find' : `find ${homeEnd(m)}`;
        case 'waitUntilDone':
            return 'wait';
        default:
            return 'run';
    }
}

/**
 * Where each shown step's badge goes (pure): [{i, at, dir, row, n, band}] in step order — `at` the field point its
 * stack starts from, `dir` the way it goes ('up' over its spot; at the START, past the robot on the side away from its
 * knob and toward the field's middle: 'up' / 'down' for a knob pointing sideways, 'right' / 'left' for one pointing
 * up or down), `row` its place (the first step on top), `n` the stack's size, `band` the poses to the step that waits
 * for a started one (or null).
 */
export function badgeSpots(route, plan, size = 15) {
    const s0 = startPose(route.start);
    const spots = new Map();
    route.steps.forEach((step, i) => {
        const f = plan[i] && plan[i].from;
        if (MECHANISM_KINDS.has(step.do) && !isHidden(step) && finite(f)) {  // a drive with a hole before it: no place
            const key = `${f.x.toFixed(2)},${f.y.toFixed(2)}`;
            spots.set(key, [...(spots.get(key) || []), i]);
        }
    });
    const out = [];
    for (const all of spots.values()) {
        const f = plan[all[0]].from;
        let dir = 'up';
        let at = f;
        if (finite(s0) && Math.hypot(f.x - s0.x, f.y - s0.y) < 0.5) {
            const half = (size / 2) * Math.SQRT2;  // the start robot's reach at any heading
            const side = Math.abs(Math.sin((s0.headingDeg * Math.PI) / 180)) > 0.5;  // its knob points up or down
            dir = side ? (s0.x <= 0 ? 'right' : 'left') : s0.y <= 0 ? 'up' : 'down';
            at = { x: s0.x + (dir === 'right' ? half : dir === 'left' ? -half : 0), y: s0.y + (dir === 'up' ? half : dir === 'down' ? -half : 0) };
        }
        all.forEach((i, k) => out.push({ i, at, dir, n: all.length, row: dir === 'up' ? all.length - 1 - k : k, band: bandOf(route, plan, i) }));
    }
    return out.sort((a, b) => a.i - b.i);
}

/** A started step's band: where it starts, then the path of every step up to the wait that collects it. */
function bandOf(route, plan, i) {
    const w = route.steps[i].do.startsWith('start') ? waitsFor(route, i).wait : undefined;
    if (w === undefined) {
        return null;
    }
    const path = [plan[i].from, ...route.steps.slice(i + 1, w).flatMap((x, k) => stepPoses(x, plan[i + 1 + k]))].filter(finite);
    return path.length > 1 ? path : null;
}

/** The badges and bands for `app`'s route on its `plan` (P: field -> picture, inPx: inches -> picture px). */
export function mechBadges(app, plan, P, inPx, near) {
    const route = app.route();
    const robot = app.robot();
    const level = app.levels();
    const sel = app.state().selected;
    const out = [];
    const bands = [];
    const hgt = inPx(3.4);
    const gap = inPx(0.6);
    for (const { i, at, dir, row, n, band } of badgeSpots(route, plan, app.size())) {
        const step = route.steps[i];
        const faded = near && !near.has(i) ? ' faded' : '';
        if (band) {
            bands.push(s('polyline', { class: `g-mech-band${faded}`, 'data-band': i, points: band.map((q) => P(q.x, q.y)).map((q) => `${q.px},${q.py}`).join(' ') }));
        }
        const m = ((robot && robot.mechanisms) || []).find((x) => x.ident === step.mechanism);
        const label = `${i + 1} ${badgeWord(step, m)}`;
        const q = P(at.x, at.y);
        const wid = label.length * 7 + hgt + 12;
        const x = dir === 'right' ? q.px + gap : dir === 'left' ? q.px - gap - wid : q.px - wid / 2;
        const y = dir === 'up' ? q.py - inPx(3) - (row + 1) * (hgt + gap)  // over its spot: clear of the spot's pill
            : dir === 'down' ? q.py + gap + row * (hgt + gap) : q.py - (n * (hgt + gap)) / 2 + row * (hgt + gap);
        const lv = level[i] ? ` badge-${level[i]}` : '';
        out.push(s('g', { class: `g-mech${lv}${sel === i ? ' sel' : ''}${faded}${isLocked(step) ? ' locked' : ''}`, 'data-badge': i, role: 'button', 'aria-label': `Step ${i + 1}`,
                          onpointerdown: (e) => e.stopPropagation(), onclick: () => app.select(i) },
                   s('title', {}, `Step ${i + 1}: ${stepTitle(step, robot)[0]}`),
                   s('rect', { class: 'g-mech-box', x, y, width: wid, height: hgt, rx: hgt / 2, ry: hgt / 2 }),
                   s('path', { class: 'g-mech-icon', d: iconPath(stepIcon(step) === 'mechanism' && m ? kindIcon(m) : stepIcon(step)),
                               transform: `translate(${x + 6} ${y + (hgt - 14) / 2}) scale(${14 / 24})` }),
                   s('text', { class: 'g-mech-text', x: x + 24, y: y + hgt / 2 }, label)));
    }
    return [...bands, ...out];
}
