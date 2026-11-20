// field_view.mjs — THE FIELD: the real picture under an SVG layer (RMP1 brief §1.4; landmine 1).
//
// Pan (drag the empty field, or Space-drag, or two fingers), zoom (the wheel, a pinch, + / - / 0 to fit), and
// the route drawn over it: the robot's 15 in square at the start in the red alliance's colour with its FRONT
// marked and a heading knob; every step's PLANNED path dashed (a curve drawn as the library's own CurvePath,
// curve.mjs), a ghost square where it ends, a handle on every point to drag, a step's number at its spot, and
// any step with an error red (a warning amber). Nothing here is the robot's real path: that is the preview's,
// drawn on its own layer (drawRun, RMP1-A2u): each robot as the record placed it, its path so far solid, the rest
// faint, and for a mirrored route the pair check's 6 in band along the mirror line. RMP1-C D4: a mirrored route also
// shows ROBOT TWO — its start and planned path reflected, dimmed and labelled — and, when the two come too close, both
// squares red at that moment. A handle never grows past 8 screen px, so zoomed in two points 1 in apart are two targets.
// Snapping (field.mjs): the half-tile grid, a wall flush, 15 deg headings; Alt holds it off, Shift = 1 deg.

import { s } from './dom.mjs';
import { fieldToPicture, pictureToField, snapPoint, snapHeading } from '../model/field.mjs';
import { squareCorners, bearingDeg } from '../model/geometry.mjs';
import { round3 } from '../model/numbers.mjs';
import { startPose } from '../model/route.mjs';
import { reflect, stepPoses } from '../model/pair.mjs';

const KNOB_IN = 15;  // the heading knob sits this far ahead of the start's centre

/** The field view in `host` (an element); `app` is the shell's API. */
export function createFieldView(host, app) {
    const pic = app.picture;
    const svg = s('svg', { class: 'field-svg', role: 'application', 'aria-label': 'The field. Pan by dragging, zoom with the wheel.',
                           tabindex: '0' });
    const image = s('image', { href: pic.url, x: 0, y: 0, width: pic.width, height: pic.height, preserveAspectRatio: 'none' });
    const dim = s('rect', { class: 'g-dim', x: 0, y: 0, width: pic.width, height: pic.height });
    const layer = s('g');
    const runLayer = s('g', { class: 'g-runs' });
    svg.append(image, dim, layer, runLayer);
    host.append(svg);

    let view = { x: 0, y: 0, w: pic.width, h: pic.height };
    let drag = null;  // {kind, ...}
    const pointers = new Map();
    let space = false;

    const P = (x, y) => fieldToPicture(x, y, pic);
    const inPx = (inches) => inches * pic.pxPerIn;

    const handleR = () => Math.min(inPx(1.6), (8 * view.w) / (svg.getBoundingClientRect().width || view.w));
    function applyView() {
        svg.setAttribute('viewBox', `${view.x} ${view.y} ${view.w} ${view.h}`);
        layer.querySelectorAll('.g-handle[data-handle]').forEach((c) => c.setAttribute('r', handleR()));
    }
    function fit() {
        const r = svg.getBoundingClientRect();
        const aspect = r.width > 0 && r.height > 0 ? r.width / r.height : 1;
        const pad = 16;
        let w = pic.width + 2 * pad;
        let hh = pic.height + 2 * pad;
        if (w / hh < aspect) {
            w = hh * aspect;
        } else {
            hh = w / aspect;
        }
        view = { x: pic.width / 2 - w / 2, y: pic.height / 2 - hh / 2, w, h: hh };
        applyView();
    }
    function zoomAt(factor, cx, cy) {
        const nw = Math.min(Math.max(view.w * factor, 120), pic.width * 4);
        const k = nw / view.w;
        view = { x: cx - (cx - view.x) * k, y: cy - (cy - view.y) * k, w: nw, h: view.h * k };
        applyView();
    }
    function toSvg(ev) {
        const m = svg.getScreenCTM();
        if (!m) {
            return { x: 0, y: 0 };
        }
        const p = new DOMPoint(ev.clientX, ev.clientY).matrixTransform(m.inverse());
        return { x: p.x, y: p.y };
    }
    function toField(ev, snapOn) {
        const p = toSvg(ev);
        const f = pictureToField(p.x, p.y, pic);
        if (snapOn && app.state().snap && !ev.altKey) {
            const within = Math.max(1, (view.w / svg.getBoundingClientRect().width) * 10 / pic.pxPerIn);
            return snapPoint(f.x, f.y, app.field(), app.size(), within);
        }
        return { x: round3(f.x), y: round3(f.y) };
    }

    svg.addEventListener('wheel', (ev) => {
        ev.preventDefault();
        const p = toSvg(ev);
        zoomAt(ev.deltaY > 0 ? 1.15 : 1 / 1.15, p.x, p.y);
    }, { passive: false });

    svg.addEventListener('pointerdown', (ev) => {
        svg.focus({ preventScroll: true });
        pointers.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });
        if (pointers.size === 2) {
            const [a, b] = [...pointers.values()];
            drag = { kind: 'pinch', dist: Math.hypot(a.x - b.x, a.y - b.y) };
            return;
        }
        const target = ev.target.closest('[data-handle]');
        const tool = app.state().tool;
        if (target && !space && ev.button === 0) {
            ev.preventDefault();
            svg.setPointerCapture(ev.pointerId);
            drag = { kind: 'handle', handle: target.dataset.handle, index: Number(target.dataset.index), point: Number(target.dataset.point),
                     moved: false };
            app.beginDrag();
            return;
        }
        if (space || ev.button === 1 || (tool === 'select' && ev.button === 0)) {
            svg.setPointerCapture(ev.pointerId);
            drag = { kind: 'pan', x: ev.clientX, y: ev.clientY, view: { ...view }, click: tool === 'select' && !space && ev.button === 0,
                     at: toField(ev, true), ev: { detail: ev.detail } };
            svg.classList.add('panning');
            return;
        }
        if (ev.button === 0) {
            app.fieldClick(toField(ev, true), ev);
        }
    });
    svg.addEventListener('dblclick', (ev) => {
        if (app.state().tool === 'curveTo') {
            ev.preventDefault();
            app.finishCurve();
        }
    });
    svg.addEventListener('pointermove', (ev) => {
        const f = pictureToField(toSvg(ev).x, toSvg(ev).y, pic);
        app.hud(f);
        if (pointers.has(ev.pointerId)) {
            pointers.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });
        }
        if (!drag) {
            return;
        }
        if (drag.kind === 'pinch' && pointers.size === 2) {
            const [a, b] = [...pointers.values()];
            const d = Math.hypot(a.x - b.x, a.y - b.y);
            const mid = toSvg({ clientX: (a.x + b.x) / 2, clientY: (a.y + b.y) / 2 });
            zoomAt(drag.dist / d, mid.x, mid.y);
            drag.dist = d;
        } else if (drag.kind === 'pan') {
            const r = svg.getBoundingClientRect();
            const k = view.w / r.width;
            view = { ...drag.view, x: drag.view.x - (ev.clientX - drag.x) * k, y: drag.view.y - (ev.clientY - drag.y) * k };
            if (Math.abs(ev.clientX - drag.x) + Math.abs(ev.clientY - drag.y) > 3) {
                drag.click = false;
            }
            applyView();
        } else if (drag.kind === 'handle') {
            drag.moved = true;
            if (drag.handle === 'heading') {
                const st = app.route().start;
                const p = toField(ev, false);
                const raw = bearingDeg(st.x, st.y, p.x, p.y);
                if (Number.isFinite(raw)) {
                    app.dragTo({ handle: 'heading', headingDeg: ev.altKey ? round3(raw) : snapHeading(raw, ev.shiftKey ? 1 : 15) });
                }
            } else {
                app.dragTo({ handle: drag.handle, index: drag.index, point: drag.point, ...toField(ev, true) });
            }
        }
    });
    const end = (ev) => {
        pointers.delete(ev.pointerId);
        if (!drag) {
            return;
        }
        if (drag.kind === 'handle') {
            app.endDrag(drag.moved);
            app.selectHandle(drag.handle, drag.index, drag.point, !drag.moved);  // the one the arrows nudge; a drag opens no drawer
        }
        if (drag.kind === 'pan' && drag.click && ev.type === 'pointerup') {
            app.backgroundClick(drag.at, drag.ev);  // a click, not a pan: place the start, or clear the selection
        }
        svg.classList.remove('panning');
        drag = pointers.size === 2 ? drag : null;
    };
    svg.addEventListener('pointerup', end);
    svg.addEventListener('pointercancel', end);
    window.addEventListener('keydown', (ev) => {
        if (ev.code === 'Space' && !/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)) {
            space = true;
            svg.classList.add('panning');
        }
    });
    window.addEventListener('keyup', (ev) => {
        if (ev.code === 'Space') {
            space = false;
            svg.classList.remove('panning');
        }
    });
    new ResizeObserver(() => fit()).observe(host);

    // ── drawing ─────────────────────────────────────────────────────────────────────────────────────
    const pts = (list) => list.map((q) => P(q.x, q.y)).map((q) => `${q.px},${q.py}`).join(' ');
    const cornersOf = (pose, size = app.size()) => pts(squareCorners(pose.x, pose.y, pose.headingDeg, size));
    function square(pose, cls) {
        return s('polygon', { class: cls, points: cornersOf(pose) });
    }
    function frontPoints(pose, size = app.size()) {
        const half = size / 2;
        const rad = (pose.headingDeg * Math.PI) / 180;
        const tip = P(pose.x + Math.cos(rad) * (half - 1), pose.y + Math.sin(rad) * (half - 1));
        const l = P(pose.x + Math.cos(rad) * (half - 5) - Math.sin(rad) * 3.2, pose.y + Math.sin(rad) * (half - 5) + Math.cos(rad) * 3.2);
        const r = P(pose.x + Math.cos(rad) * (half - 5) + Math.sin(rad) * 3.2, pose.y + Math.sin(rad) * (half - 5) - Math.cos(rad) * 3.2);
        return `${tip.px},${tip.py} ${l.px},${l.py} ${r.px},${r.py}`;
    }
    function front(pose) {
        return s('polygon', { class: 'g-front', points: frontPoints(pose) });
    }
    function arc(at, fromDeg, toDeg) {
        const sweep = ((toDeg - fromDeg + 540) % 360) - 180;
        if (Math.abs(sweep) < 1) {
            return null;
        }
        const r = 10;
        const pts = [];
        for (let i = 0; i <= 24; ++i) {
            const a = ((fromDeg + (sweep * i) / 24) * Math.PI) / 180;
            const q = P(at.x + r * Math.cos(a), at.y + r * Math.sin(a));
            pts.push(`${q.px},${q.py}`);
        }
        return s('polyline', { class: 'g-turn', points: pts.join(' ') });
    }
    function handle(x, y, kind, index, point, cls = '') {
        const q = P(x, y);
        return s('circle', { class: `g-handle ${cls}`, cx: q.px, cy: q.py, r: handleR(), 'data-handle': kind, 'data-index': index,
                             'data-point': point ?? 0 });
    }
    /** One pill at a spot, its steps' numbers in it ("3-5": the curve's end, then a level and a wait there). */
    function badge(pose, label, cls = '') {
        const q = P(pose.x, pose.y);
        const r = inPx(2.4);
        const w = Math.max(2 * r, label.length * 8 + 14);  // 13 px bold digits are ~8 px wide
        return s('g', { class: cls }, s('rect', { class: 'g-badge', x: q.px - w / 2, y: q.py - r, width: w, height: 2 * r, rx: r, ry: r }),
                 s('text', { class: 'g-badge-text', x: q.px, y: q.py }, label));
    }

    /** "3-5" for a run of numbers, "2, 4" otherwise (the step numbers the student sees in the list). */
    function numbersLabel(nums) {
        const runs = [];
        for (const n of nums) {
            const last = runs[runs.length - 1];
            if (last && n === last[1] + 1) {
                last[1] = n;
            } else {
                runs.push([n, n]);
            }
        }
        return runs.map(([a, b]) => (a === b ? String(a) : b === a + 1 ? `${a}, ${b}` : `${a}\u2013${b}`)).join(', ');
    }

    /** D4: robot two on the field for a mirrored route — its start and planned path, robot one's reflected; D5: when
     *  both run it together and come too close, both squares red at that moment (the check's own issue). */
    function robotTwo(route, plan) {
        const mirror = app.robot().skillsMirror;
        const st0 = startPose(route.start);
        if (!route.mirroredFor || !mirror || !st0) {
            return [];
        }
        const out = [];
        const w = app.field().wallInnerFaceIn;
        const c = Math.atan2(mirror.sin2a, mirror.cos2a) / 2;  // the mirror line's heading
        const ends = [-1, 1].map((k) => P(k * w * Math.SQRT2 * Math.cos(c), k * w * Math.SQRT2 * Math.sin(c)));
        out.push(s('line', { class: 'g-mirror', x1: ends[0].px, y1: ends[0].py, x2: ends[1].px, y2: ends[1].py, 'data-mirror': '' }));
        const path = [st0];
        route.steps.forEach((step, i) => path.push(...stepPoses(step, plan[i])));
        const two = path.map((q) => reflect(q, mirror));
        out.push(s('polyline', { class: 'g-twin-path', points: pts(two) }));
        const t0 = two[0];
        out.push(s('g', { class: 'g-twin', 'data-twin': '' }, square(t0, 'g-twin-robot'), s('polygon', { class: 'g-twin-front', points: frontPoints(t0) })));
        const q = P(t0.x, t0.y + app.size() / 2 + 2.5);
        out.push(s('text', { class: 'g-twin-label', x: q.px, y: q.py }, 'robot two (mirrored)'));
        const hit = app.issues().find((x) => x.code === 'RCP1' && x.pair);
        if (hit) {
            out.push(square(hit.pair.one, 'g-too-close'), square(hit.pair.two, 'g-too-close'));
        }
        return out;
    }

    function render() {
        const st = app.state();
        const route = app.route();
        const plan = app.plan();
        const level = app.levels();
        layer.replaceChildren();
        svg.classList.toggle('tool-add', st.tool !== 'select');
        const parts = [];
        // the planned path
        route.steps.forEach((step, i) => {
            const p = plan[i];
            if (!p || !p.from) {
                return;
            }
            const cls = `g-plan ${level[i] || ''}`;
            const sel = st.selected === i;
            if (step.do === 'driveTo' || step.do === 'reverseTo') {
                if (step.faceFirst && Number.isFinite(p.bearingDeg)) {
                    parts.push(arc(p.from, p.from.headingDeg, p.bearingDeg));
                }
                const a = P(p.from.x, p.from.y);
                const b = P(step.x, step.y);
                if (Number.isFinite(step.x) && Number.isFinite(step.y)) {
                    parts.push(s('line', { class: cls, x1: a.px, y1: a.py, x2: b.px, y2: b.py }));
                    parts.push(square(p.to, `g-ghost ${sel ? 'sel' : ''}`));
                    parts.push(handle(step.x, step.y, 'point', i, 0, `${sel ? 'sel' : ''} ${level[i] === 'error' ? 'error' : ''}`));
                }
            } else if ((step.do === 'forward' || step.do === 'backward') && p.to) {
                const a = P(p.from.x, p.from.y);
                const b = P(p.to.x, p.to.y);
                parts.push(s('line', { class: cls, x1: a.px, y1: a.py, x2: b.px, y2: b.py }));
                parts.push(square(p.to, `g-ghost ${sel ? 'sel' : ''}`));
            } else if (step.do === 'face' || step.do === 'faceAway' || step.do === 'turnTo') {
                parts.push(arc(p.from, p.from.headingDeg, p.to.headingDeg));
                if (step.do !== 'turnTo' && Number.isFinite(step.x)) {
                    const a = P(p.from.x, p.from.y);
                    const b = P(step.x, step.y);
                    parts.push(s('line', { class: 'g-draft', x1: a.px, y1: a.py, x2: b.px, y2: b.py }));
                    parts.push(handle(step.x, step.y, 'point', i, 0, sel ? 'sel' : ''));
                }
                if (sel) {
                    parts.push(square(p.to, 'g-ghost sel'));
                }
            } else if ((step.do === 'curveTo' || step.do === 'reverseCurveTo') && p.curve) {
                const pts = p.curve.samples(1).map((q) => P(q.x, q.y)).map((q) => `${q.px},${q.py}`).join(' ');
                parts.push(s('polyline', { class: cls, points: pts }));
                parts.push(square(p.to, `g-ghost ${sel ? 'sel' : ''}`));
                (step.points || []).forEach((q, k) => parts.push(handle(q.x, q.y, 'curvePoint', i, k, sel ? `sel${st.point === k ? ' point' : ''}` : '')));
            }
        });
        // the curve being drawn
        if (st.tool === 'curveTo' && st.curveDraft.length) {
            const last = plan.length ? plan[plan.length - 1].to : startPose(route.start);
            const pts = [last, ...st.curveDraft].filter(Boolean).map((q) => P(q.x, q.y));
            parts.push(s('polyline', { class: 'g-draft', points: pts.map((q) => `${q.px},${q.py}`).join(' ') }));
            st.curveDraft.forEach((q) => {
                const c = P(q.x, q.y);
                parts.push(s('circle', { class: 'g-handle sel', cx: c.px, cy: c.py, r: inPx(1.2) }));
            });
        }
        parts.push(...robotTwo(route, plan));
        // the start: the robot, its front, its heading knob — drawn only when it is a place (a start missing a part,
        // from a hand-edited file, is not: the step list shows "?" for the part and the inspector asks for it)
        const st0 = startPose(route.start);
        if (st0) {
            parts.push(s('g', { 'data-handle': 'start', 'data-index': -1 }, square(st0, 'g-robot'), front(st0)));
            const rad = (st0.headingDeg * Math.PI) / 180;
            const a = P(st0.x, st0.y);
            const k = P(st0.x + KNOB_IN * Math.cos(rad), st0.y + KNOB_IN * Math.sin(rad));
            parts.push(s('line', { class: 'g-heading', x1: a.px, y1: a.py, x2: k.px, y2: k.py }));
            parts.push(s('circle', { class: 'g-heading-knob', cx: k.px, cy: k.py, r: inPx(2), 'data-handle': 'heading', 'data-index': -1,
                                     'aria-label': `heading ${st0.headingDeg} degrees` }));
        }
        // numbers at each step's spot, drawn LAST so the start's robot never hides the first (a home there); they
        // take no pointer (style.css), so the start and every handle under one stays draggable. The steps that END at one
        // spot share ONE pill, so none hides under another; it takes the worst of their marks (red over amber)
        const spots = new Map();
        route.steps.forEach((step, i) => {
            const p = plan[i];
            if (p && p.to && Number.isFinite(p.to.x) && Number.isFinite(p.to.y)) {
                const key = `${p.to.x.toFixed(2)},${p.to.y.toFixed(2)}`;
                const spot = spots.get(key) || { pose: p.to, nums: [], level: '' };
                spot.nums.push(i + 1);
                if (level[i] === 'error' || (level[i] === 'warn' && spot.level !== 'error')) {
                    spot.level = level[i];
                }
                spots.set(key, spot);
            }
        });
        for (const spot of spots.values()) {
            parts.push(badge(spot.pose, numbersLabel(spot.nums), spot.level ? `badge-${spot.level}` : ''));
        }
        layer.append(...parts.filter(Boolean));
    }

    // ── PREVIEW's run (RMP1-A2u): built once per run, then only the robots and their paths move ──
    let run = null;
    const kAt = (path, i) => {  // how many of the path's points are at or before pace i
        let lo = 0;
        let hi = path.length;
        while (lo < hi) {
            const mid = (lo + hi) >> 1;
            if (path[mid].i <= i) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return lo;
    };
    const grow = (acc, path, from, to) => {  // a polyline's points path[from, to), only the new ones added while it plays
        const a = !acc || acc.from !== from || to < acc.k ? { from, k: from, text: '' } : acc;
        return to > a.k ? { from, k: to, text: `${a.text} ${pts(path.slice(a.k, to))}` } : a;
    };
    function drawRun(frame) {
        svg.classList.toggle('previewing', !!frame);
        if (!frame) {
            runLayer.replaceChildren();
            run = null;
            return;
        }
        if (!run || run.key !== frame.key) {
            runLayer.replaceChildren();
            run = { key: frame.key, robots: [] };
            if (frame.pair) {
                // the pair check's 6 in band: everything within 3 in of the mirror line y = -x (inside the walls)
                const c = (frame.pair.gapIn / 2) * Math.SQRT2;
                const w = app.field().wallInnerFaceIn;
                const band = [[-w, w], [c - w, w], [w, c - w], [w, -w], [w - c, -w], [-w, w - c]].map(([x, y]) => ({ x, y }));
                runLayer.append(s('polygon', { class: 'g-band', points: pts(band) }));
                if (!frame.pair.ok && frame.pair.one) {
                    [frame.pair.one, frame.pair.two].forEach((q, i) => runLayer.append(s('polygon', {
                        class: 'g-too-close', points: cornersOf({ x: q[0], y: q[1], headingDeg: (q[2] * 180) / Math.PI }, frame.robots[i] && frame.robots[i].size) })));
                }
            }
            for (const r of frame.robots) {
                const g = s('g', { class: `g-run ${r.id}` },
                    s('polyline', { class: 'g-run-all', points: pts(r.path) }),
                    s('polyline', { class: 'g-run-done' }),
                    s('polyline', { class: 'g-run-after' }),
                    s('polygon', { class: 'g-run-robot' }),
                    s('polygon', { class: 'g-run-front' }));
                runLayer.append(g);
                run.robots.push(g);
            }
        }
        frame.robots.forEach((r, i) => {
            const g = run.robots[i];
            const [, done, after, body, nose] = g.children;
            const lo = kAt(r.path, r.upTo);
            // after a THROUGH the robot would not get further (RPV1): the rest is drawn faded
            const past = r.throughAt !== null && r.upTo >= r.throughAt;
            const cut = past ? kAt(r.path, r.throughAt) : lo;
            g.done = grow(g.done, r.path, 0, cut);
            g.after = past ? grow(g.after, r.path, cut - 1, lo) : null;
            done.setAttribute('points', past ? g.done.text : `${g.done.text} ${pts([r.pose])}`);
            after.setAttribute('points', past ? `${g.after.text} ${pts([r.pose])}` : '');
            g.classList.toggle('through', past);
            body.setAttribute('points', cornersOf(r.pose, r.size));
            nose.setAttribute('points', frontPoints(r.pose, r.size));
        });
    }

    return { render, fit, drawRun, zoom: (f) => zoomAt(f, view.x + view.w / 2, view.y + view.h / 2), element: svg };
}
