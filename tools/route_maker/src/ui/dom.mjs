// dom.mjs — the few DOM helpers every screen uses, and the route builder's icon set (drawn for the app as inline
// SVG paths; no font, no file, no request).

const SVG_NS = 'http://www.w3.org/2000/svg';

/** An HTML element: h('button', {class: 'btn', onclick: f}, 'Save'). `null`/`false` children are skipped. */
export function h(tag, attrs = {}, ...children) {
    const el = document.createElement(tag);
    applyAttrs(el, attrs);
    appendAll(el, children);
    return el;
}

/** An SVG element (same form as h). */
export function s(tag, attrs = {}, ...children) {
    const el = document.createElementNS(SVG_NS, tag);
    applyAttrs(el, attrs);
    appendAll(el, children);
    return el;
}

function applyAttrs(el, attrs) {
    for (const [k, v] of Object.entries(attrs || {})) {
        if (v === undefined || v === null || v === false) {
            continue;
        }
        if (k.startsWith('on') && typeof v === 'function') {
            el.addEventListener(k.slice(2), v);
        } else if (k === 'dataset') {
            Object.assign(el.dataset, v);
        } else if (k === 'value' && 'value' in el) {
            el.value = v;
        } else if (k === 'checked') {
            el.checked = !!v;
        } else {
            el.setAttribute(k, v === true ? '' : String(v));
        }
    }
}

function appendAll(el, children) {
    for (const c of children.flat(Infinity)) {
        if (c === null || c === undefined || c === false) {
            continue;
        }
        el.append(c instanceof Node ? c : document.createTextNode(String(c)));
    }
}

/** Replace `el`'s children with `children`. */
export function mount(el, ...children) {
    el.replaceChildren();
    appendAll(el, children);
    return el;
}

const PATHS = {
    drive: 'M5 12h12m-5-5 5 5-5 5',
    backup: 'M19 12H7m5 5-5-5 5-5',
    turn: 'M4 12a8 8 0 1 0 2.3-5.6M4 4v4h4',
    face: 'M12 3v3m0 12v3M3 12h3m12 0h3M12 8a4 4 0 1 0 0 8 4 4 0 0 0 0-8Z',
    curve: 'M4 18c4 0 4-12 8-12s4 12 8 12',
    mechanism: 'M12 8.5a3.5 3.5 0 1 0 0 7 3.5 3.5 0 0 0 0-7Zm7.4 4.6 1.6 1.2-1.6 2.8-1.9-.7a7 7 0 0 1-1.6.9l-.3 2H12.4l-.3-2a7 7 0 0 1-1.6-.9l-1.9.7L7 14.3l1.6-1.2a7 7 0 0 1 0-1.9L7 10l1.6-2.8 1.9.7a7 7 0 0 1 1.6-.9l.3-2h3.2l.3 2a7 7 0 0 1 1.6.9l1.9-.7L21 10l-1.6 1.2a7 7 0 0 1 0 1.9Z',
    wait: 'M12 7v5l3 2M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18Z',
    home: 'M4 11 12 4l8 7M6 10v9h12v-9',
    start: 'M6 21V4m0 0h10l-2 4 2 4H6',
    select: 'M6 3l12 8-5.5 1.2L15 19l-2.4 1-2.5-6.7L6 16Z',
    snap: 'M6 4v7a6 6 0 0 0 12 0V4h-4v7a2 2 0 0 1-4 0V4Z',
    plus: 'M12 5v14M5 12h14',
    minus: 'M5 12h14',
    fit: 'M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5',
    undo: 'M9 14 4 9l5-5M4 9h10a6 6 0 0 1 0 12h-3',
    redo: 'M15 14l5-5-5-5M20 9H10a6 6 0 0 0 0 12h3',
    sun: 'M12 7a5 5 0 1 0 0 10 5 5 0 0 0 0-10Zm0-5v2m0 16v2M4.9 4.9l1.4 1.4m11.4 11.4 1.4 1.4M2 12h2m16 0h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4',
    moon: 'M20 14.5A8 8 0 0 1 9.5 4 8 8 0 1 0 20 14.5Z',
    help: 'M9.5 9a2.5 2.5 0 1 1 3.5 2.3c-.6.3-1 .9-1 1.6V14m0 3v.5M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18Z',
    code: 'm8 8-4 4 4 4m8-8 4 4-4 4m-6 3 4-14',
    play: 'M7 4v16l13-8Z',
    pause: 'M8 5v14M16 5v14',
    prev: 'M18 6l-8 6 8 6ZM6 5v14',
    next: 'M6 6l8 6-8 6ZM18 5v14',
    folder: 'M3 7a2 2 0 0 1 2-2h4l2 2h8a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2Z',
    save: 'M5 3h11l3 3v13a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2Zm2 0v5h8V3M7 21v-7h10v7',
    open: 'M3 7a2 2 0 0 1 2-2h4l2 2h8a2 2 0 0 1 2 2v1H7l-3 9',
    trash: 'M4 7h16M10 11v6m4-6v6M6 7l1 13h10l1-13M9 7V4h6v3',
    copy: 'M9 9h10v12H9ZM5 15V3h10',
    dup: 'M8 8h12v12H8ZM4 16V4h12',
    warn: 'M12 4 2 20h20Zm0 6v4m0 3v.5',
    error: 'M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18Zm0 5v5m0 3v.5',
    check: 'm5 12 5 5 9-10',
    close: 'M6 6l12 12M18 6 6 18',
    grip: 'M9 6h.01M15 6h.01M9 12h.01M15 12h.01M9 18h.01M15 18h.01',
    list: 'M8 6h12M8 12h12M8 18h12M4 6h.01M4 12h.01M4 18h.01',
    panel: 'M4 4h16v16H4ZM14 4v16',
    download: 'M12 4v11m-5-5 5 5 5-5M5 20h14',
    straight: 'M12 3v18m-4-14 4-4 4 4m-8 10 4 4 4-4',
    chevron: 'm6 9 6 6 6-6',
    edit: 'M4 20h4L19 9l-4-4L4 16Zm9-13 4 4',
    mirror: 'M12 3v18M4 7l5 5-5 5ZM20 7l-5 5 5 5Z',
};

/** An icon (24 x 24, stroked in the current colour). */
export function icon(name, extra = {}) {
    return s('svg', { viewBox: '0 0 24 24', width: 18, height: 18, fill: 'none', stroke: 'currentColor', 'stroke-width': 2,
                      'stroke-linecap': 'round', 'stroke-linejoin': 'round', 'aria-hidden': 'true', ...extra },
             s('path', { d: PATHS[name] || PATHS.help }));
}

/**
 * Ask the student to choose (the page's own confirm: no browser dialog). `choices`: [{label, value, kind}] — kind
 * 'primary' | 'danger' | '' — the first is focused; Esc or the scrim answers `cancel`. Resolves to the value chosen.
 */
export function ask({ title, words, choices, cancel = null }) {
    return new Promise((resolve) => {
        const done = (v) => {
            scrim.remove();
            dlg.remove();
            document.removeEventListener('keydown', onKey, true);
            resolve(v);
        };
        const onKey = (e) => {
            if (e.key === 'Escape') {
                e.stopPropagation();
                done(cancel);
            }
        };
        const scrim = h('div', { class: 'scrim', onclick: () => done(cancel) });
        const dlg = h('div', { class: 'dialog ask', role: 'alertdialog', 'aria-modal': 'true', 'aria-labelledby': 'ask-title', 'aria-describedby': 'ask-words' },
            h('h2', { id: 'ask-title' }, title),
            h('p', { id: 'ask-words' }, words),
            h('div', { class: 'row', style: 'justify-content:flex-end;margin-top:16px' },
              choices.map((c) => h('button', { class: `btn ${c.kind || ''}`, onclick: () => done(c.value) }, c.label))));
        document.addEventListener('keydown', onKey, true);
        document.body.append(scrim, dlg);
        dlg.querySelector('button').focus();
    });
}

/**
 * A menu under `anchor` (role=menu, closed by a click outside, Esc, or a choice): `children` are its rows. Returns the
 * function that closes it. The first enabled `.menu-item` is focused; Up / Down move between the items.
 */
export function openMenu(anchor, children, label) {
    const close = () => {
        pop.remove();
        document.removeEventListener('pointerdown', outside, true);
        document.removeEventListener('keydown', onKey, true);
    };
    const outside = (e) => !pop.contains(e.target) && !anchor.contains(e.target) && close();
    const onKey = (e) => {
        if (e.key === 'Escape') {
            e.stopPropagation();
            close();
            anchor.focus();
        } else if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
            const items = [...pop.querySelectorAll('.menu-item:not(:disabled)')];
            const i = items.indexOf(document.activeElement);
            if (items.length) {
                e.preventDefault();
                e.stopPropagation();
                items[(i + (e.key === 'ArrowDown' ? 1 : items.length - 1)) % items.length].focus();
            }
        }
    };
    const pop = h('div', { class: 'popover', role: 'menu', 'aria-label': label }, children);
    pop.addEventListener('click', (e) => e.target.closest('.menu-item') && close());
    const r = anchor.getBoundingClientRect();
    pop.style.left = `${Math.max(8, Math.min(r.left, window.innerWidth - 340))}px`;
    pop.style.top = `${r.bottom + 6}px`;
    document.body.append(pop);
    setTimeout(() => {
        document.addEventListener('pointerdown', outside, true);
        document.addEventListener('keydown', onKey, true);
        const first = pop.querySelector('.menu-item[aria-checked="true"], .menu-item:not(:disabled)');
        if (first) {
            first.focus();
        }
    }, 0);
    return close;
}

/**
 * Ask for a line of text (the page's own prompt). `check(text)` returns the problem in words, or null: OK is refused
 * while there is one, and the words show under the box as it is typed. Resolves to the text, or null (Cancel, Esc).
 */
export function askText({ title, words, value = '', check = () => null, ok = 'OK', label = title }) {
    return new Promise((resolve) => {
        const done = (v) => {
            scrim.remove();
            dlg.remove();
            document.removeEventListener('keydown', onKey, true);
            resolve(v);
        };
        const onKey = (e) => {
            if (e.key === 'Escape') {
                e.stopPropagation();
                done(null);
            }
        };
        const problem = h('p', { class: 'inline-err', role: 'alert', 'aria-live': 'polite' });
        const okBtn = h('button', { class: 'btn primary', type: 'submit' }, ok);
        const input = h('input', { class: 'txt', type: 'text', value, 'aria-label': label, maxlength: 40, spellcheck: 'false',
                                   oninput: () => update() });
        const update = () => {
            const p = check(input.value);
            problem.textContent = p || '';
            okBtn.disabled = !!p;
            return !p;
        };
        const form = h('form', { onsubmit: (e) => { e.preventDefault(); if (update()) { done(input.value); } } },
            input, problem,
            h('div', { class: 'row', style: 'justify-content:flex-end;margin-top:12px' },
              h('button', { class: 'btn', type: 'button', onclick: () => done(null) }, 'Cancel'), okBtn));
        const scrim = h('div', { class: 'scrim', onclick: () => done(null) });
        const dlg = h('div', { class: 'dialog ask', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'ask-title' },
            h('h2', { id: 'ask-title' }, title), words ? h('p', {}, words) : null, form);
        document.addEventListener('keydown', onKey, true);
        document.body.append(scrim, dlg);
        update();
        input.focus();
        input.select();
    });
}
