// feel_tab.mjs — DRIVING FEEL (RMP1-B; the brief §1.2): the drive mode, per stick axis a dead zone and a curve, THE
// GRAPH — the library's own shapeAxis (feel.mjs, bit-identical to the C++: FE1) from 201 points, the straight line
// faint beside it, a "try it" stick you drag —, the ramp with its "full speed from rest" readout, what the drive
// does when let go, and the re-zero button. A slider moving redraws at once; the edit is kept when it is let go.

import { h, s } from './dom.mjs';
import { fmt, parseTyped } from '../model/numbers.mjs';
import { shapeAxis, axisOf, rampWords, modeOf, DEFAULT_DEADBAND } from '../model/feel.mjs';

const MODE_WORDS = [['arcade', 'Arcade', 'left stick drives, right stick turns'], ['tankSticks', 'Tank', 'each stick drives its own side'],
                    ['arcadeField', 'Field arcade', 'push the stick the way to go on the field']];
const AXIS_WORDS = { forward: 'Forward and back', turn: 'Turning', strafe: 'Sideways (strafe)' };
const W = 360;
const H = 300;
const X0 = 46;
const X1 = W - 14;
const Y0 = 14;
const Y1 = H - 40;
const px = (x) => X0 + ((x + 1) / 2) * (X1 - X0);
const py = (y) => Y1 - ((y + 1) / 2) * (Y1 - Y0);

/** The feel card (`api`: the controls tab's driving, set, preview, ui). */
export function renderFeel(api) {
    const d = api.driving();
    const holonomic = !(api.robot().drive && api.robot().drive.tank);
    const axes = holonomic ? ['forward', 'turn', 'strafe'] : ['forward', 'turn'];
    if (!axes.includes(api.ui.axis)) {
        api.ui.axis = 'forward';
    }
    const axis = api.ui.axis;
    const live = { ...axisOf(d, axis) };  // what the sliders show while they move
    const graph = h('div', { class: 'graph-wrap' });
    const readout = h('p', { class: 'readout' });  // the dot's aria-valuetext says it: no announcement per pointer move
    const draw = () => mountGraph(graph, readout, live, api);
    const slider = (label, key, max, step, hint) => {
        const value = live[key];
        const num = h('input', { type: 'text', inputmode: 'decimal', class: 'num', name: `feel-${key}`, 'aria-label': `${label} (number)`, value: fmt(value), disabled: api.locked() });
        const range = h('input', { type: 'range', min: 0, max, step, value, name: `feel-${key}-slider`, 'aria-label': label, disabled: api.locked(),
            oninput: (e) => {
                live[key] = Number(e.target.value);
                num.value = fmt(live[key]);
                draw();
            },
            onchange: (e) => keep(key, Number(e.target.value)) });
        num.addEventListener('change', (e) => {
            const v = typed(e.target);
            if (v !== null) {
                keep(key, v);
            }
        });
        num.addEventListener('keydown', (e) => e.key === 'Enter' && e.target.blur());
        return h('label', { class: 'fld slider' }, h('span', {}, label, h('small', {}, hint)), h('div', { class: 'row nowrap' }, range, num));
    };
    /** A typed number, or null with the reason said in plain words (the box marked). */
    const typed = (box) => {
        const v = parseTyped(box.value);
        if (Number.isNaN(v)) {
            box.classList.add('bad');
            api.toast('Not a number: type digits, like 0.25.', 'error');
            return null;
        }
        return v;
    };
    const keep = (key, v) => {
        const cur = (d[axis] && { ...d[axis] }) || {};
        cur[key === 'expo' ? 'curve' : 'deadband'] = v;
        api.set({ [axis]: cur });
    };
    const errs = api.issues();
    const ramp = d.rampPerSecond || 0;
    const card = h('section', { class: 'card feel-card', 'aria-label': 'Driving feel' },
        h('div', { class: 'card-head' }, h('h2', {}, 'Driving feel'),
          h('span', { class: 'hint' }, 'How far the stick is pushed, and what the robot is asked for. A small push slow, a full push fast: raise the curve.')),
        errs.map((x) => h('div', { class: `issue ${x.level}`, role: 'alert' }, h('span', { class: 'ic' }, '!'), h('div', {}, x.words))),
        h('div', { class: 'seg wide', role: 'radiogroup', 'aria-label': 'Drive mode' }, MODE_WORDS.map(([m, t, sub]) => h('button', {
            role: 'radio', 'aria-checked': String(modeOf(d) === m), class: modeOf(d) === m ? 'on' : '', disabled: api.locked(),
            onclick: () => modeOf(d) !== m && api.set({ mode: m }) }, h('strong', {}, t), h('small', {}, sub)))),
        modeOf(d) === 'tankSticks' ? h('p', { class: 'hint' }, 'Tank: both sticks are first averaged into a forward request (left + right) / 2 and a turn request '
            + '(right − left) / 2; the curves below shape those.') : null,
        modeOf(d) === 'arcadeField' ? h('p', { class: 'hint' }, 'Field arcade needs a heading the robot trusts; without one it drives as plain arcade until the re-zero button is pressed.') : null,
        h('div', { class: 'feel-grid' },
          h('div', {},
            h('div', { class: 'seg', role: 'tablist', 'aria-label': 'Which stick axis' }, axes.map((a) => h('button', {
                role: 'tab', 'aria-selected': String(axis === a), class: axis === a ? 'on' : '', onclick: () => { api.ui.axis = a; api.preview(); } }, AXIS_WORDS[a]))),
            slider('Dead zone', 'deadband', 0.5, 0.01, `stick moves under this do nothing (the library's ${fmt(DEFAULT_DEADBAND)})`),
            slider('Curve', 'expo', 1, 0.05, '0 = straight; higher = finer control near the middle, still full speed at the end'),
            h('label', { class: 'fld slider' }, h('span', {}, 'Ramp', h('small', {}, rampWords(ramp))),
              h('div', { class: 'row nowrap' },
                h('input', { type: 'range', min: 0, max: 10, step: 0.5, value: ramp, name: 'feel-ramp-slider', 'aria-label': 'Ramp (per second)', disabled: api.locked(),
                             oninput: (e) => { e.target.closest('.fld').querySelector('small').textContent = rampWords(Number(e.target.value)); },
                             onchange: (e) => api.set({ rampPerSecond: Number(e.target.value) }) }),
                h('input', { type: 'text', inputmode: 'decimal', class: 'num', name: 'feel-ramp', 'aria-label': 'Ramp (per second, number)', value: fmt(ramp),
                             disabled: api.locked(), onchange: (e) => { const v = typed(e.target); if (v !== null) { api.set({ rampPerSecond: v }); } } }))),
            h('div', { class: 'field-row' },
              h('label', { class: 'fld' }, h('span', {}, 'When the sticks are let go'),
                h('select', { class: 'sel', name: 'onRelease', disabled: api.locked(), onchange: (e) => api.set({ onRelease: e.target.value }) },
                  [['coast', 'the drive coasts'], ['brake', 'the drive brakes'], ['hold', 'the drive holds where it is']].map(([v, t]) => h('option', { value: v, selected: (d.onRelease || 'coast') === v }, t)))),
              h('label', { class: 'fld' }, h('span', {}, 'Re-zero button (field arcade)'),
                h('select', { class: 'sel', name: 'rezeroButton', disabled: api.locked(), onchange: (e) => api.set({ rezeroButton: e.target.value }) },
                  [...new Set([d.rezeroButton || 'Y', ...api.free()])].map((b) => h('option', { value: b, selected: (d.rezeroButton || 'Y') === b }, b))))),
            h('button', { class: 'btn ghost', disabled: api.locked(), onclick: () => api.set({ forward: undefined, turn: undefined, strafe: undefined, rampPerSecond: undefined }) },
              'Use the library\'s defaults')),
          h('div', {}, graph, readout)));
    draw();
    return card;
}

/** THE GRAPH: stick (bottom) to the request (side), the library's curve from 201 points. A drag moves only the dot. */
function mountGraph(el, readout, a, api) {
    const pts = [];
    for (let i = 0; i <= 200; ++i) {
        const x = -1 + i / 100;
        pts.push(`${i ? 'L' : 'M'}${px(x).toFixed(2)} ${py(shapeAxis(x, a)).toFixed(2)}`);
    }
    const db = a.deadband > 0 && a.deadband < 1 ? a.deadband : 0;
    const grid = [-1, -0.5, 0, 0.5, 1];
    const line = s('line', { class: 'g-probe-line' });
    const dot = s('circle', { class: 'g-probe', r: 7, tabindex: 0, role: 'slider', 'aria-label': 'Try it: the stick', 'aria-valuemin': -100, 'aria-valuemax': 100,
                              onkeydown: (e) => {
                                  const d = { ArrowRight: 0.05, ArrowUp: 0.05, ArrowLeft: -0.05, ArrowDown: -0.05 }[e.key];
                                  if (d) {
                                      e.preventDefault();
                                      place(Math.round((api.ui.probe + d) * 100) / 100);
                                  }
                              } });
    const svg = s('svg', { class: 'graph', viewBox: `0 0 ${W} ${H}`, role: 'group' },
        s('rect', { class: 'g-zone', x: px(-db), y: Y0, width: px(db) - px(-db), height: Y1 - Y0 }),
        grid.map((g) => [s('line', { class: 'g-gridline', x1: px(g), y1: Y0, x2: px(g), y2: Y1 }), s('line', { class: 'g-gridline', x1: X0, y1: py(g), x2: X1, y2: py(g) })]),
        grid.map((g) => [s('text', { class: 'g-tick', x: px(g), y: Y1 + 16 }, `${g * 100}`), s('text', { class: 'g-tick end', x: X0 - 6, y: py(g) + 4 }, `${g * 100}`)]),
        s('line', { class: 'g-straight', x1: px(-1), y1: py(-1), x2: px(1), y2: py(1) }),
        s('path', { class: 'g-curve', d: pts.join(' ') }), line, dot,
        s('text', { class: 'g-axis', x: (X0 + X1) / 2, y: H - 6 }, 'stick, % (pushed back ← → pushed forward)'),
        s('text', { class: 'g-axis', x: 12, y: (Y0 + Y1) / 2, transform: `rotate(-90 12 ${(Y0 + Y1) / 2})` }, `the ${api.ui.axis} request, %`));
    /** The "try it" stick at `p` (-1..1): the dot, its line and the readout -- nothing else is re-drawn. */
    function place(p) {
        api.ui.probe = Math.max(-1, Math.min(1, p));
        const x = api.ui.probe;
        const y = shapeAxis(x, a);
        for (const [k, v] of Object.entries({ x1: px(x), y1: py(0), x2: px(x), y2: py(y) })) {
            line.setAttribute(k, v);
        }
        dot.setAttribute('cx', px(x));
        dot.setAttribute('cy', py(y));
        dot.setAttribute('aria-valuenow', Math.round(x * 100));
        dot.setAttribute('aria-valuetext', `${Math.round(x * 100)} % stick asks for ${Math.round(y * 100)} %`);
        svg.setAttribute('aria-label', `The ${api.ui.axis} curve: ${Math.round(x * 100)} % stick asks for ${Math.round(y * 100)} %`);
        readout.replaceChildren(h('strong', {}, `${Math.round(x * 100)} % stick → ${Math.round(y * 100)} % speed`),
            ' — what the library asks the drive for (the request, not a measured speed). Drag the dot to try another push.');
    }
    svg.addEventListener('pointerdown', (e) => {
        const at = (ev) => {
            const r = svg.getBoundingClientRect();
            place(Math.round(((((ev.clientX - r.left) / r.width) * W - X0) / (X1 - X0)) * 200 - 100) / 100);
        };
        at(e);
        const up = () => {
            window.removeEventListener('pointermove', at);
            window.removeEventListener('pointerup', up);
        };
        window.addEventListener('pointermove', at);
        window.addEventListener('pointerup', up);
    });
    el.replaceChildren(svg);
    place(api.ui.probe);
}
