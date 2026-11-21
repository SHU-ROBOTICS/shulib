// help.mjs — the help a student needs without leaving the page (RMP1 brief §1.4): the shortcuts list (`?`), and
// the three-line "your first route" coach on an empty field (dismissible, back from Help).

import { h, icon } from './dom.mjs';
import { robotName, robotBridge } from '../model/robot_names.mjs';

export const SHORTCUTS = [
    ['V or Esc', 'Select (and cancel what is being added)'],
    ['D', 'Drive: click where the robot drives to'],
    ['B', 'Back up: click where the robot backs up to'],
    ['T', 'Turn: click the point the robot turns to face'],
    ['C', 'Curve: click points; Enter or double-click finishes'],
    ['F', 'Straight: drive forward (or back up) a number of inches'],
    ['M', 'Mechanism: pick one, then what it does (Esc goes back)'],
    ['P', 'Wait (pause)'],
    ['H', 'Hide the selected step on the field (again: show it); it still runs'],
    ['L', 'Lock the selected step: no drag, no nudge, clicks pass through it'],
    ['O', 'Focus: only the selected step and its neighbours; Esc leaves'],
    ['Click again on one spot', 'The next point under it ("2 of 3 here")'],
    ['Delete', 'Delete the selected step'],
    ['Ctrl+D', 'Duplicate the selected step'],
    ['Ctrl+Z  /  Ctrl+Shift+Z', 'Undo / redo'],
    ['Ctrl+S', 'Save the project'],
    ['Ctrl+E', 'Export'],
    ['Ctrl+Enter', 'PREVIEW: run the route and watch the robot drive it'],
    ['Space', 'Play or pause the preview (a tap; held, it pans)'],
    ['Left / Right', 'The preview\'s step before or after (Shift: 0.1 s)'],
    ['Home / End', 'The preview\'s start or end; Esc closes it'],
    ['Alt+Up / Alt+Down', 'Move the selected step up or down the list'],
    ['Arrow keys', 'Nudge the selected point 1 in (Shift: 0.25 in); typed x and y are exact'],
    ['+  /  -  /  0', 'Zoom in, out, fit the field'],
    ['Space + drag', 'Pan the field (any tool)'],
    ['Alt while dragging', 'No snapping'],
    ['Shift on the heading knob', 'Turn by 1 degree instead of 15'],
    ['?', 'This list'],
];

/** The shortcuts and help dialog. */
export function openHelp(app) {
    const close = () => {
        scrim.remove();
        dlg.remove();
        document.removeEventListener('keydown', onKey);
        app.focusField();
    };
    const onKey = (e) => e.key === 'Escape' && close();
    document.addEventListener('keydown', onKey);
    const scrim = h('div', { class: 'scrim', onclick: close });
    const dlg = h('div', { class: 'dialog', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'help-title' },
        h('div', { class: 'row' }, h('h2', { id: 'help-title' }, 'Help'), h('span', { class: 'spacer' }),
          h('button', { class: 'btn icon ghost', 'aria-label': 'Close', onclick: close }, icon('close'))),
        h('p', {}, `Build your routes on the real field (the skills route and any other you name: the Route button at the top): place the start of ${robotBridge('one')}, add steps, and export the C++. Everything is in inches, degrees and seconds; 0 degrees faces the right wall and 90 faces blue. The dashed path is the PLAN — where the library is told to go.`),
        h('p', {}, 'A red mark is something the robot would refuse; its fix is one click away. Nothing is invented for you: a time limit stays empty until you type it.'),
        h('p', {}, 'The Controls tab: click a button on the controller to give it a job, or a used one to change it; the driving feel is its step 2. Ctrl+Z / Ctrl+Shift+Z undo and redo there too, and Ctrl+E exports the driver file\'s three parts.'),
        h('table', { class: 'keys' }, h('tbody', {}, SHORTCUTS.map(([k, w]) => h('tr', {}, h('td', {}, h('kbd', { class: 'k' }, k)), h('td', {}, w))))),
        h('div', { class: 'row', style: 'margin-top:16px' },
          h('button', { class: 'btn', onclick: () => { close(); app.showCoach(); } }, 'Show "your first route" again')));
    document.body.append(scrim, dlg);
    dlg.querySelector('button').focus();
}

/** The coach card for an empty field. */
export function coachCard(app) {
    return h('div', { class: 'coach', role: 'note', 'aria-label': 'Your first route' },
        h('h3', {}, 'Your first route'),
        h('ol', {},
          h('li', {}, `Click the field where ${robotName('one')} starts; drag its red knob to turn it.`),
          h('li', {}, 'Pick ', h('strong', {}, 'Drive'), ' (D) and click where it drives to.'),
          app.preview && app.preview.available()
              ? h('li', {}, 'Press ', h('strong', {}, 'PREVIEW'), ' to watch the robot drive it, then ', h('strong', {}, 'Export'), ' for the code.')
              : h('li', {}, 'Press ', h('strong', {}, 'Export'), ' to get the code for the robot.')),
        h('div', { class: 'row' }, h('button', { class: 'btn primary', onclick: () => app.hideCoach() }, 'Got it')));
}
