// mech_strip.mjs — THE MECHANISMS BESIDE THE FIELD during PREVIEW (RMP1-E): a height bar per lift or arm, a piston
// per pneumatic, a motor chip per power mechanism or flywheel, each drawn from the record in step with the playback.
// One accented (the mechanism the step playing uses); the rest quiet.

import { h, s, mount } from './dom.mjs';
import { fmt } from '../model/numbers.mjs';

const WHO = { one: 'Robot one', two: 'Robot two' };

function ph(m) {
    return m.placeholder ? h('span', { class: 'ph', title: m.placeholder }, 'PLACEHOLDER') : null;
}

function liftRow(m) {
    const fill = h('div', { class: 'ms-fill' });
    const mark = h('div', { class: 'ms-target' });
    const track = h('div', { class: 'ms-track', role: 'meter', 'aria-label': `${m.name}: height`, 'aria-valuemin': m.bottom, 'aria-valuemax': m.top },
                    fill, mark);
    const value = h('strong', { class: 'ms-value' });
    const to = h('span', { class: 'ms-to' });
    const el = h('div', { class: 'ms-mech ms-lift', dataset: { mech: m.name } },
        h('div', { class: 'ms-name' }, m.name, ph(m)),
        h('div', { class: 'ms-body' },
          h('div', { class: 'ms-stops' }, h('span', {}, `${fmt(m.top)} ${m.units}`), h('span', {}, `${fmt(m.bottom)} ${m.units}`)),
          track, h('div', { class: 'ms-read' }, value, to)));
    return { el, set(it) {
        fill.style.height = `${100 * it.level}%`;
        mark.hidden = it.targetLevel === null;
        mark.style.bottom = `${100 * (it.targetLevel || 0)}%`;
        to.textContent = it.target === null ? '' : `to ${fmt(it.target)} ${m.units}`;
        value.textContent = it.words;
        track.setAttribute('aria-valuenow', String(it.value ?? m.bottom));
        track.setAttribute('aria-valuetext', it.words);
    } };
}

function pistonRow(m) {
    const n = Math.max(m.positions.length - 1, 1);
    const rod = s('rect', { class: 'ms-rod', x: 14, y: 8, height: 4, rx: 1 });
    const word = h('span', { class: 'ms-word' });
    const el = h('div', { class: 'ms-mech ms-piston', dataset: { mech: m.name } },
        s('svg', { viewBox: '0 0 34 20', width: 34, height: 20, 'aria-hidden': 'true' },
          s('rect', { class: 'ms-cyl', x: 1, y: 4, width: 14, height: 12, rx: 2 }), rod),
        h('span', { class: 'ms-name' }, m.name, ph(m)), word);
    return { el, set(it) {
        rod.setAttribute('width', String(4 + (14 * Math.max(it.index ?? 0, 0)) / n));
        word.textContent = it.words;
    } };
}

function motorRow(m) {
    const word = h('span', { class: 'ms-word' });
    const el = h('div', { class: 'ms-mech ms-motor', dataset: { mech: m.name } }, h('span', { class: 'ms-dot', 'aria-hidden': 'true' }),
                 h('span', { class: 'ms-name' }, m.name, ph(m)), word);
    return { el, set(it) {
        el.classList.toggle('running', !!it.value);
        word.textContent = it.words;
    } };
}

/** The strip in `host` (the field pane): show(record) draws it for a run (null hides it), update(items) every frame. */
export function createStrip(host) {
    const el = h('aside', { class: 'mech-strip', 'aria-label': 'The mechanisms during the run' });
    el.hidden = true;
    host.append(el);
    let shown = null;
    let rows = [];
    let lit = -1;
    return {
        element: el,
        show(rec) {
            const on = !!rec && (rec.mechanisms || []).length > 0;
            el.hidden = !on;
            host.classList.toggle('with-strip', on);
            if (on && rec !== shown) {
                rows = rec.mechanisms.map((m) => (m.kind === 'lift' || m.kind === 'arm' ? liftRow(m) : m.kind === 'pneumatic' ? pistonRow(m) : motorRow(m)));
                mount(el, h('h3', {}, `${WHO[rec.robot] || 'The robot'}'s mechanisms`), rows.map((r) => r.el));
                lit = -1;
            }
            shown = on ? rec : null;
        },
        update(items) {
            items.forEach((it, k) => {
                rows[k].set(it);
                rows[k].el.classList.toggle('on', it.on);
            });
            const k = items.findIndex((it) => it.on);
            if (k >= 0 && k !== lit) {  // a short window scrolls the strip: the accented one is kept in view
                const r = rows[k].el;
                el.scrollTop = Math.min(Math.max(el.scrollTop, r.offsetTop + r.offsetHeight - el.clientHeight), k ? r.offsetTop : 0);
            }
            lit = k;
        },
    };
}
