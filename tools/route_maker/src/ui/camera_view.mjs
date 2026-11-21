// camera_view.mjs — THE CAMERA TAB (RMP1-N) in Watch the robot. Only when the watched robot prints [CAM] lines (shulib
// Drive): two tabs, Run and Camera. Camera is one sentence ("Sees: AprilTag 3, Pin (87 %)") over the camera's picture,
// each tag, object and colour a box: a tag its id, big; an object its class name and score; a colour its id. A picture
// greys when its lines stop (camera_log.mjs's freshS: "last seen 2 s ago"). run_view.mjs's render() calls cameraCard().

import { h, s, mount } from './dom.mjs';
import { newCamera, readCamera, seesWords, itemWords, edgeWords, inPicture, freshS, ageS, IMAGE } from '../model/camera_log.mjs';
import { robotName } from '../model/robot_names.mjs';

/** The robot in words: its name (RMP1-L's one source), or "this robot" when the watch does not say which. */
const robotWords = (which) => (which === 'one' || which === 'two' ? robotName(which) : 'this robot');

const cards = new WeakMap();  // a watch (run_log.mjs's state) → its card
let hiddenAt = 0;  // when the page was last hidden: nothing paints while it is, so lines read after may be that old
globalThis.document?.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        hiddenAt = Date.now();
    }
});
const clamp = (v, lo, hi) => Math.min(Math.max(v, lo), hi);

/** One detection, in the picture's own pixels (the SVG's viewBox scales it); its label inside the picture: above the
 *  box, else below it, else inside its top (a text's box with its outline reaches ~0.9 of its size from its middle). */
function box(it, names) {
    const r = s('rect', { x: it.x, y: it.y, width: it.w, height: it.h });
    const text = (x, y, size, anchor, words) => s('text', { x, y, 'font-size': size, 'text-anchor': anchor, 'dominant-baseline': 'central' }, words);
    if (it.kind === 'tag') {
        const size = Math.max(14, Math.min(64, Math.min(it.w, it.h) * 0.7));
        const m = size * 0.9;
        return s('g', { class: 'cam-box', 'data-kind': 'tag' }, r,
            text(clamp(it.x + it.w / 2, m, IMAGE.w - m), clamp(it.y + it.h / 2, m, IMAGE.h - m), size, 'middle', String(it.id)));
    }
    const right = it.x + it.w / 2 > IMAGE.w / 2;
    const y = it.y >= 27 ? it.y - 13 : it.y + it.h <= IMAGE.h - 27 ? it.y + it.h + 13 : clamp(it.y + 14, 14, IMAGE.h - 14);
    return s('g', { class: 'cam-box', 'data-kind': it.kind }, r,
        text(right ? Math.min(it.x + it.w, IMAGE.w) - 2 : Math.max(it.x, 0) + 2, y, 14, right ? 'end' : 'start', itemWords(it, names)));
}

/** The card as `c` stands now: each part changed only when what it shows changed (a box under the pointer stays put). */
function paint(c) {
    const age = ageS(c.cam, Date.now());
    const stale = age !== null && age > freshS(c.cam);
    const seen = c.cam.seen;
    const words = seesWords(c.cam, c.robot);
    const when = !stale ? null : c.away ? 'seen while this page was hidden' : `last seen ${age < 1 ? 'under a second' : `${Math.round(age)} s`} ago`;
    const edge = edgeWords(c.cam);
    c.el.dataset.tab = c.tab;
    c.el.dataset.mode = c.mode;
    c.tabs.forEach((b) => {
        b.classList.toggle('on', b.dataset.tab === c.tab);
        b.setAttribute('aria-selected', String(b.dataset.tab === c.tab));
    });
    c.body.hidden = c.tab !== 'camera';
    const said = `${words}|${when}|${edge}`;
    if (c.said !== said) {
        c.said = said;
        mount(c.sees, words, when ? h('span', { class: 'cam-age' }, when) : null);
        mount(c.edge, edge);
        c.edge.hidden = !edge;
        c.svg.setAttribute('aria-label', words);
    }
    c.svg.classList.toggle('stale', stale);
    if (c.pic !== JSON.stringify([seen, c.cam.names])) {
        c.pic = JSON.stringify([seen, c.cam.names]);
        mount(c.boxes, seen && !seen.error ? seen.items.filter((it) => it.kind !== 'other' && inPicture(it) !== 'none').map((it) => box(it, c.cam.names)) : []);
    }
    if (c.sees.parentNode !== c.body || c.cam.none === (c.svg.parentNode === c.body)) {  // no sensor: the sentence alone
        mount(c.body, c.sees, c.edge, c.cam.none ? null : c.svg);
    }
}

/** The camera card for `watch`, read from `raw`'s lines since the last call (`mode`: run_view's); null while no [CAM]
 *  line has come. */
export function cameraCard(watch, raw, mode) {
    if (!watch) {
        return null;
    }
    let c = cards.get(watch);
    if (!c) {
        c = { cam: newCamera(), read: 0, readAt: Date.now(), runs: 0, tab: null, timer: 0 };
        cards.set(watch, c);
    }
    const now = Date.now();
    const at = hiddenAt > c.readAt ? hiddenAt : now;  // the first read after a hidden spell: as old as the spell
    const fresh = watch.lines - c.read;
    if (fresh > raw.length) {  // lines dropped unread (a long hidden spell): a boot among them is unknown
        Object.assign(c.cam, newCamera(), { lines: c.cam.lines });
    }
    const before = c.cam.at;
    raw.slice(raw.length - Math.min(fresh, raw.length)).forEach((line) => readCamera(c.cam, line, at));
    c.away = c.cam.at !== before ? at !== now : c.away;
    c.read = watch.lines;
    c.readAt = now;
    if (!c.cam.lines) {
        return null;
    }
    if (!c.el) {
        c.tabs = ['run', 'camera'].map((t) => h('button', { type: 'button', role: 'tab', 'data-tab': t,
            onclick: () => { c.tab = t; paint(c); } }, t === 'run' ? 'Run' : 'Camera'));
        c.sees = h('p', { class: 'cam-sees', id: 'cam-sees' });
        c.edge = h('p', { class: 'cam-edge', hidden: true });
        c.boxes = s('g', { class: 'cam-boxes' });
        c.svg = s('svg', { class: 'cam-frame', viewBox: `0 0 ${IMAGE.w} ${IMAGE.h}`, role: 'img' }, s('rect', { class: 'cam-bg', width: IMAGE.w, height: IMAGE.h }), c.boxes);
        c.body = h('div', { class: 'cam-body', role: 'tabpanel' });
        c.el = h('div', { class: 'cam', 'aria-live': 'off' }, h('div', { class: 'seg cam-tabs', role: 'tablist', 'aria-label': 'Watch' }, c.tabs), c.body);
    }
    c.tab = c.tab === null ? (watch.runs.length ? 'run' : 'camera') : watch.runs.length > c.runs ? 'run' : c.tab;  // a new run: its tab
    c.runs = watch.runs.length;
    c.robot = robotWords(watch.robot);
    c.mode = mode;
    paint(c);
    if (!c.timer) {  // the greying, with no line coming: while the card is on screen
        c.timer = setInterval(() => {
            if (!c.el.isConnected || c.el.closest('[hidden]')) {
                clearInterval(c.timer);
                c.timer = 0;
            } else {
                paint(c);
            }
        }, 100);
    }
    return c.el;
}
