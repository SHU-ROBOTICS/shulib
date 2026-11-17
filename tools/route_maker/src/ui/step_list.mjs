// step_list.mjs — THE STEP LIST (RMP1 brief §1.4): numbered cards with icons, the start first; click to select,
// drag a card's grip to reorder (every later pose re-planned at once), a red or amber edge on a card with an
// error or a warning. Keyboard: each card is a button; Alt+Up / Alt+Down moves the selected step (app.mjs).
// While a PREVIEW shows (RMP1-A2u), each card is lit by how its step went in the run — red with the library's own
// line when it failed, amber when cut, grey when skipped — and a click goes to where that step began.

import { h, icon } from './dom.mjs';
import { stepIcon, stepTitle, startTitle } from './words.mjs';

/** Render the list into `el`. */
export function renderStepList(el, app) {
    const st = app.state();
    const route = app.route();
    const level = app.levels();
    const robot = app.robot();
    const [startText, startSub] = startTitle(route.start);
    const pick = (i) => {
        app.select(i);
        if (app.preview) {
            app.preview.selected(i);  // while a run shows, go to where this step began
        }
    };
    const start = h('li', {}, h('div', {
        class: `step start-card ${st.selected === 'start' ? 'selected' : ''} ${level.start ? `has-${level.start}` : ''}`,
        role: 'button', tabindex: '0', onclick: () => app.select('start'),
        'aria-label': `${startText}${level.start ? ` (${level.start === 'error' ? 'has an error' : 'has a warning'})` : ''}`,
        onkeydown: (e) => (e.key === 'Enter' || e.key === ' ') && (e.preventDefault(), app.select('start')),
    },
    h('span', { class: 'step-num' }, ''),
    h('span', { class: 'step-icon' }, icon('start')),
    h('span', { class: 'step-main' },
        h('div', { class: 'step-title' }, startText),
        h('div', { class: 'step-sub' }, startSub)),
    h('span')));
    const items = route.steps.map((step, i) => {
        const [title, sub] = stepTitle(step, robot);
        const pv = app.preview && app.preview.stepInfo(i);
        const card = h('div', {
            class: `step ${st.selected === i ? 'selected' : ''} ${level[i] ? `has-${level[i]}` : ''} ${pv ? `pv-${pv.state}` : ''}`,
            role: 'button', tabindex: '0', 'data-step': i,
            'aria-label': `Step ${i + 1}: ${title}${level[i] ? ` (${level[i] === 'error' ? 'has an error' : 'has a warning'})` : ''}${pv ? `. In the preview: ${pv.words}` : ''}${pv && pv.two ? `. Robot two: ${pv.two.words}` : ''}`,
            onclick: () => pick(i),
            onkeydown: (e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                    e.preventDefault();
                    pick(i);
                }
            },
        },
        h('span', { class: 'step-num' }, String(i + 1)),
        h('span', { class: 'step-icon' }, icon(stepIcon(step))),
        h('span', { class: 'step-main' },
            h('div', { class: 'step-title', title }, step.label ? h('span', { class: 'step-label' }, `${step.label}: `) : null, title),
            h('div', { class: 'step-sub' }, sub),
            pv ? h('div', { class: 'step-run', title: [pv.words, ...pv.library, pv.placeholder].filter(Boolean).join('\n') },
                   icon(pv.state === 'ok' ? 'check' : pv.state === 'skipped' ? 'minus' : pv.state === 'cut' ? 'warn' : 'error'),
                   h('span', {}, pv.state === 'ok' ? `Done in ${(pv.t1 - pv.t0).toFixed(2)} s` : pv.words),
                   pv.placeholder ? h('span', { class: 'ph' }, 'PLACEHOLDER') : null) : null,
            pv && pv.state !== 'ok' && pv.library.length ? h('div', { class: 'step-lib' }, pv.library[0]) : null,
            pv && pv.two ? h('div', { class: 'step-lib two', title: [pv.two.words, ...pv.two.library].join('\n') },
                             `Robot two: ${pv.two.words}${pv.two.library.length ? `. ${pv.two.library[0]}` : ''}`) : null),
        h('span', { class: 'grip', title: 'Drag to reorder', 'aria-hidden': 'true', draggable: 'true' }, icon('grip')));
        const grip = card.querySelector('.grip');
        grip.addEventListener('dragstart', (e) => {
            e.dataTransfer.setData('text/plain', String(i));
            e.dataTransfer.effectAllowed = 'move';
            card.classList.add('dragging');
        });
        grip.addEventListener('dragend', () => card.classList.remove('dragging'));
        card.addEventListener('dragover', (e) => {
            e.preventDefault();
            card.classList.add('drop-before');
        });
        card.addEventListener('dragleave', () => card.classList.remove('drop-before'));
        card.addEventListener('drop', (e) => {
            e.preventDefault();
            card.classList.remove('drop-before');
            const from = Number(e.dataTransfer.getData('text/plain'));
            if (Number.isInteger(from) && from !== i) {
                app.move(from, from < i ? i - 1 : i);
            }
        });
        return h('li', {}, card);
    });
    const empty = route.steps.length === 0
        ? h('li', { class: 'empty-list' }, route.start
            ? 'Add a step: pick Drive, Curve, Turn, Mechanism or Wait above the field, then click on the field.'
            : 'Place robot one\'s start first: click where it starts on the field.')
        : null;
    el.replaceChildren(h('ol', { class: 'steps', 'aria-label': 'The route\'s steps' }, start, items, empty));
}
