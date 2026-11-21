// step_list.mjs — THE STEP LIST (RMP1 brief §1.4): numbered cards with icons, the start first; click to select,
// drag a card's grip to reorder (every later pose re-planned at once), a red or amber edge on a card with an
// error or a warning. Keyboard: each card is a button; Alt+Up / Alt+Down moves the selected step (app.mjs).
// While a PREVIEW shows (RMP1-A2u), each card is lit by how its step went in the run — red with the library's own
// line when it failed, amber when cut, grey when skipped — and a click goes to where that step began.
// RMP1-G: on a card (hovered, focused or selected) the eye hides it on the field, the lock locks it, ... opens its
// menu (focus, a section); a hidden card is dimmed. A section's head row folds its steps and hides or locks them together.

import { h, icon, askText, openMenu } from './dom.mjs';
import { stepIcon, stepTitle, startTitle } from './words.mjs';
import { robotName } from '../model/robot_names.mjs';
import { isHidden, isLocked, sectionsOf, indicesOf } from '../model/step_view.mjs';
import { isGentle } from '../model/gently.mjs';

/** A small button on a card or a section's row (its click never selects the card). */
const act = (label, ic, on, fn) => h('button', { class: `step-act${on ? ' on' : ''}`, type: 'button', 'aria-label': label, title: label,
                                                'aria-pressed': String(on), onclick: (e) => { e.stopPropagation(); fn(e.currentTarget); } }, icon(ic));

const nameIt = (title, value, ok) => askText({ title, value, ok, label: 'The section\'s name',
    words: 'A name for the steps from here to the next section, like "Start" or "Claw 1". Only this page shows it.',
    check: (t) => (t.trim() ? null : 'Type a name.') });

function sectionMenu(anchor, app, head) {
    const name = app.route().steps[head].view.section;
    openMenu(anchor, [
        h('button', { class: 'menu-item', onclick: async () => {
            const n = await nameIt(`Rename "${name}"`, name, 'Rename');
            if (n !== null) {
                app.setSection(head, { section: n });
            }
        } }, icon('edit'), 'Rename the section'),
        h('button', { class: 'menu-item', onclick: () => app.setSection(head, { section: undefined, folded: undefined }) }, icon('close'),
          'Remove the section (its steps stay)')], 'Section');
}

function stepMenu(anchor, app, i) {
    const step = app.route().steps[i];
    const head = !!(step.view && step.view.section);
    openMenu(anchor, [
        h('button', { class: 'menu-item', onclick: () => app.toggleHidden(i) }, icon(isHidden(step) ? 'eye' : 'eyeOff'),
          isHidden(step) ? 'Show on the field (H)' : 'Hide on the field (H)'),
        h('button', { class: 'menu-item', onclick: () => app.toggleLocked(i) }, icon(isLocked(step) ? 'unlock' : 'lock'),
          isLocked(step) ? 'Unlock (L)' : 'Lock (L)'),
        h('button', { class: 'menu-item', onclick: () => { app.select(i); app.setFocus(true); } }, icon('target'), 'Focus on this step (O)'),
        head ? null : h('button', { class: 'menu-item', onclick: async () => {
            const n = await nameIt('A section starts here', '', 'Start the section');
            if (n !== null) {
                app.setSection(i, { section: n });
            }
        } }, icon('list'), 'Start a section here')], `Step ${i + 1}`);
}

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
    const sections = sectionsOf(route);
    const items = route.steps.map((step, i) => {
        const sec = sections.find((x) => x.from <= i && i <= x.to);
        const row = sec && sec.head === i ? sectionRow(sec) : null;
        if (sec && sec.folded && st.selected !== i) {
            return row;  // folded: only its head row (the selected step still shows under it)
        }
        const [title, sub] = stepTitle(step, robot);
        const pv = app.preview && app.preview.stepInfo(i);
        const hidden = isHidden(step);
        const locked = isLocked(step);
        const card = h('div', {
            class: `step ${st.selected === i ? 'selected' : ''} ${level[i] ? `has-${level[i]}` : ''} ${pv ? `pv-${pv.state}` : ''}${hidden ? ' is-hidden' : ''}${locked ? ' is-locked' : ''}`,
            role: 'button', tabindex: '0', 'data-step': i,
            'aria-label': `Step ${i + 1}: ${title}${level[i] ? ` (${level[i] === 'error' ? 'has an error' : 'has a warning'})` : ''}${pv ? `. In the preview: ${pv.words}` : ''}${pv && pv.two ? `. ${robotName('two')}: ${pv.two.words}` : ''}`,
            onclick: () => pick(i),
            onkeydown: (e) => {
                if (e.target === e.currentTarget && (e.key === 'Enter' || e.key === ' ')) {
                    e.preventDefault();
                    pick(i);
                }
            },
        },
        h('span', { class: 'step-num' }, String(i + 1)),
        h('span', { class: 'step-icon' }, icon(stepIcon(step))),
        h('span', { class: 'step-main' },
            h('div', { class: 'step-title', title }, step.label ? h('span', { class: 'step-label' }, `${step.label}: `) : null, title),
            h('div', { class: 'step-sub' }, hidden ? 'Hidden on the field · ' : '', locked ? 'Locked · ' : '',
              isGentle(step) ? h('span', { class: 'gently', title: 'Leave gently: waits, then backs out slowly' }, 'gently') : null, sub),  // RMP1-P
            pv ? h('div', { class: 'step-run', title: [pv.words, ...pv.library, pv.placeholder].filter(Boolean).join('\n') },
                   icon(pv.state === 'ok' ? 'check' : pv.state === 'skipped' ? 'minus' : pv.state === 'cut' ? 'warn' : 'error'),
                   h('span', {}, pv.state === 'ok' ? `Done in ${(pv.t1 - pv.t0).toFixed(2)} s` : pv.words),
                   pv.placeholder ? h('span', { class: 'ph' }, 'PLACEHOLDER') : null) : null,
            pv && pv.state !== 'ok' && pv.library.length ? h('div', { class: 'step-lib' }, pv.library[0]) : null,
            pv && pv.two ? h('div', { class: 'step-lib two', title: [pv.two.words, ...pv.two.library].join('\n') },
                             `${robotName('two')}: ${pv.two.words}${pv.two.library.length ? `. ${pv.two.library[0]}` : ''}`) : null),
        h('span', { class: 'step-acts' },
          act(`${hidden ? 'Show' : 'Hide'} step ${i + 1} on the field (H)`, hidden ? 'eyeOff' : 'eye', hidden, () => app.toggleHidden(i)),
          act(`${locked ? 'Unlock' : 'Lock'} step ${i + 1} (L)`, locked ? 'lock' : 'unlock', locked, () => app.toggleLocked(i)),
          act(`More for step ${i + 1}`, 'more', false, (b) => stepMenu(b, app, i))),
        h('span', { class: 'grip', title: 'Drag to reorder', 'aria-hidden': 'true', draggable: 'true' }, icon('grip')));
        // ═══ RMP1-K: how the robot's run went (run_view.mjs) ═══
        if (app.runView) {
            app.runView.markCard(card, i);
        }
        // ═══ end RMP1-K ═══
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
        return [row, h('li', {}, card)];
    });
    function sectionRow(sec) {
        const idx = indicesOf(sec).map((k) => route.steps[k]);
        const allHidden = idx.every(isHidden);
        const allLocked = idx.every(isLocked);
        return h('li', { class: `section-row${sec.folded ? ' folded' : ''}`, 'data-section': sec.name },
            h('button', { class: 'step-act sec-fold', type: 'button', 'aria-expanded': String(!sec.folded),
                          'aria-label': `${sec.folded ? 'Unfold' : 'Fold'} "${sec.name}"`, onclick: () => app.setSection(sec.head, { folded: !sec.folded }) }, icon('chevron')),
            h('span', { class: 'sec-name', title: sec.name }, sec.name),
            h('span', { class: 'sec-count' }, sec.from === sec.to ? `step ${sec.from + 1}` : `steps ${sec.from + 1}\u2013${sec.to + 1}`),
            act(`${allHidden ? 'Show' : 'Hide'} "${sec.name}" on the field`, allHidden ? 'eyeOff' : 'eye', allHidden, () => app.sectionSet(sec.head, 'hidden')),
            act(`${allLocked ? 'Unlock' : 'Lock'} "${sec.name}"`, allLocked ? 'lock' : 'unlock', allLocked, () => app.sectionSet(sec.head, 'locked')),
            act(`More for "${sec.name}"`, 'more', false, (b) => sectionMenu(b, app, sec.head)));
    }
    const nHidden = route.steps.filter(isHidden).length;
    const bar = nHidden ? h('li', { class: 'view-bar', role: 'status' }, `${nHidden} step${nHidden === 1 ? '' : 's'} hidden on the field`,
                            h('button', { class: 'link', type: 'button', onclick: () => app.showAll() }, 'Show all')) : null;
    const empty = route.steps.length === 0
        ? h('li', { class: 'empty-list' }, route.start
            ? 'Add a step: pick Drive, Curve, Turn, Mechanism or Wait above the field, then click on the field.'
            : `Place ${robotName('one')}'s start first: click where it starts on the field.`)
        : null;
    el.replaceChildren(h('ol', { class: 'steps', 'aria-label': 'The route\'s steps' }, bar, start, items, empty));
}
