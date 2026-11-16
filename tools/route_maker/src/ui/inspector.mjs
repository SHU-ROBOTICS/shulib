// inspector.mjs — THE SELECTED STEP'S FORM (RMP1 brief §1.4): every number with its unit (rounded as it is
// typed, numbers.mjs), the step's checks right above it in plain words with the library's own sentence beside
// them and their one-click fixes, then "if it fails" and the label. With nothing selected: the start and the
// route's name. Nothing here invents a number: an empty time limit stays empty until the student types one.

import { h, icon, mount } from './dom.mjs';
import { parseTyped, fmt } from '../model/numbers.mjs';
import { stepTitle, stepIcon, KIND_LABEL, HOMING_WORDS } from './words.mjs';
import { ROUTE_NAME } from '../model/route.mjs';

/**
 * A number with its unit. Empty means "nothing typed" (the step keeps its hole, and the checks ask for it);
 * text that is not a plain number ("2s", "fast") is NOT stored: the box stays as typed, marked, with what to type
 * — so the route only ever holds a number or nothing (a lost number is never saved as one: save -> open is exact).
 */
function numField(label, value, unit, onSet, { placeholder = '', bad = false, name } = {}) {
    const wrap = h('div', { class: `unit ${bad ? 'bad' : ''}` });
    const said = h('small', { class: 'not-a-number', hidden: true, role: 'alert' }, `Not a number: type digits, like 2.5 (in ${unit}).`);
    const input = h('input', { type: 'text', inputmode: 'decimal', value: Number.isFinite(value) ? fmt(value) : '', placeholder,
                               'aria-label': `${label} (${unit})`, name: name || label,
                               onchange: (e) => {
                                   const t = e.target.value.trim();
                                   const v = t === '' ? undefined : parseTyped(t);
                                   if (v !== undefined && Number.isNaN(v)) {
                                       wrap.classList.add('bad');
                                       e.target.setAttribute('aria-invalid', 'true');
                                       said.hidden = false;
                                       return;
                                   }
                                   onSet(v);
                               },
                               onkeydown: (e) => e.key === 'Enter' && e.target.blur() });
    wrap.append(input, h('em', {}, unit));
    return h('label', { class: 'fld' }, h('span', {}, label), wrap, said);
}

function toggle(label, sub, checked, onSet, name) {
    return h('label', { class: 'switch' }, h('span', {}, label, sub ? h('small', {}, sub) : null),
             h('input', { type: 'checkbox', role: 'switch', checked, name, onchange: (e) => onSet(e.target.checked) }));
}

function select(label, value, options, onSet, name) {
    return h('label', { class: 'fld' }, h('span', {}, label),
             h('select', { class: 'sel', name, 'aria-label': label, onchange: (e) => onSet(e.target.value) },
               options.map(([v, t, dis]) => h('option', { value: v, selected: String(v) === String(value), disabled: dis }, t))));
}

function issueBox(issue, app) {
    const fixes = [];
    const fixWords = { faceFirst: 'Turn to face it first', homeFirst: 'Home it first', homeWhileDriving: 'Home it while the first leg drives',
                       waitForHome: 'Wait for the home first', faceCurveStart: issue.fix && issue.fix.away ? 'Turn its back to the first point first' : 'Turn to face the first point first',
                       nameSkills: 'Name it "skills"' };
    for (const f of [issue.fix, issue.alsoFix]) {
        if (f) {
            fixes.push(h('button', { class: 'btn', onclick: () => app.fix(f) }, fixWords[f.kind] || 'Fix it'));
        }
    }
    return h('div', { class: `issue ${issue.level}`, role: issue.level === 'error' ? 'alert' : 'status' },
             h('span', { class: 'ic' }, icon(issue.level === 'error' ? 'error' : 'warn')),
             h('div', {}, issue.words, issue.library ? h('span', { class: 'lib', title: 'The library\'s own words' }, `library: ${issue.library}`) : null,
               fixes.length ? h('div', { class: 'fixes' }, fixes) : null));
}

function careSection(step, i, app, issuesOf) {
    const c = step.care || {};
    const set = (f) => (v) => app.update(i, { care: { [f]: v } });
    const bad = (f) => issuesOf(f).length > 0;
    const turnFirst = (step.do === 'driveTo' || step.do === 'reverseTo') && step.faceFirst;
    return h('details', { class: 'more section', open: CARE_FIELDS.some((f) => bad(f)) || undefined },
        h('summary', {}, turnFirst ? 'Speed, time limit and how close (the drive)' : 'Speed, time limit and how close'),
        h('p', { class: 'hint' }, turnFirst ? 'Empty means the robot\'s own setting. These are the DRIVE\'s: the turn to face the point first uses the robot\'s own.'
                                            : 'Empty means the robot\'s own setting.'),
        h('div', { class: 'field-row' },
            numField('Time limit', c.timeoutS, 's', set('timeoutS'), { placeholder: 'robot\'s', bad: bad('timeoutS') }),
            step.do === 'turnTo' || step.do === 'face' || step.do === 'faceAway' ? null
                : numField('Top speed', c.maxSpeedInPerS, 'in/s', set('maxSpeedInPerS'), { placeholder: 'robot\'s', bad: bad('maxSpeedInPerS') })),
        h('div', { class: 'field-row' },
            numField('Turn speed', c.maxTurnDegPerS, 'deg/s', set('maxTurnDegPerS'), { placeholder: 'robot\'s', bad: bad('maxTurnDegPerS') }),
            step.do === 'turnTo' || step.do === 'face' || step.do === 'faceAway'
                ? numField('Heading within', c.toleranceDeg, 'deg', set('toleranceDeg'), { placeholder: 'robot\'s', bad: bad('toleranceDeg') })
                : numField('Arrive within', c.toleranceIn, 'in', set('toleranceIn'), { placeholder: 'robot\'s', bad: bad('toleranceIn') })));
}

const CARE_FIELDS = ['timeoutS', 'maxSpeedInPerS', 'maxTurnDegPerS', 'toleranceIn', 'toleranceDeg'];

function failSection(step, i, app) {
    const route = app.route();
    const f = step.care && step.care.onFail;
    const value = !f ? 'stop' : f === 'carryOn' ? 'carryOn' : `skip:${f.skipTo}`;
    const labels = route.steps.map((s, k) => [s.label, k]).filter(([l, k]) => l && k > i);
    const opts = [['stop', 'Stop the route (the default)'], ['carryOn', 'Carry on with the next step'],
                  ...labels.map(([l, k]) => [`skip:${l}`, `Skip ahead to "${l}" (step ${k + 1})`])];
    if (f && typeof f === 'object' && !labels.some(([l]) => l === f.skipTo)) {
        opts.push([value, `Skip to "${f.skipTo}" (no later step has it)`]);
    }
    return h('div', { class: 'section' }, h('h3', {}, 'If it fails'),
        select('If this step fails', value, opts, (v) => app.update(i, { care: { onFail: v === 'stop' ? 'stop' : v === 'carryOn' ? 'carryOn' : { skipTo: v.slice(5) } } }), 'onFail'),
        labels.length === 0 ? h('p', { class: 'hint' }, 'To skip ahead, give a LATER step a label first.') : null,
        h('label', { class: 'fld' }, h('span', {}, 'Label (a name a skip can land on)'),
          h('input', { class: 'txt', type: 'text', value: step.label || '', name: 'label', 'aria-label': 'Label', maxlength: 29,
                       onchange: (e) => app.update(i, { label: e.target.value.trim() === '' ? undefined : e.target.value.trim() }) })));
}

function mechanismFields(step, i, app) {
    const robot = app.robot();
    const mechs = (robot && robot.mechanisms) || [];
    const m = mechs.find((x) => x.ident === step.mechanism);
    const out = [select('Mechanism', step.mechanism, [...(m ? [] : [[step.mechanism, `${step.mechanism} (not on this robot)`]]),
        ...mechs.map((x) => [x.ident, `${x.name} (${KIND_LABEL[x.kind] || x.kind})`])], (v) => app.update(i, { mechanism: v }), 'mechanism')];
    if (m && (m.kind === 'arm' || m.kind === 'lift')) {
        out.push(h('p', { class: 'hint' }, `The ${m.name} ${HOMING_WORDS[m.homing] || ''}.`));
    }
    if ((step.do === 'move' || step.do === 'startMove') && m) {
        // a level whose height the robot's facts still OWE shows no number: the page never states a placeholder as a fact
        const height = (p) => (p.owed ? 'not measured yet' : `${fmt(p.value)} ${m.units}`);
        const opts = (m.presets || []).map((p) => (p.ledger !== undefined ? [`L${p.ledger}`, `level ${p.ledger}: "${p.name}" (${height(p)})`]
                                                                         : [`N${p.name}`, `"${p.name}" (${height(p)})`]));
        const cur = step.preset === undefined ? '' : typeof step.preset === 'number' ? `L${step.preset}` : `N${step.preset}`;
        out.push(select('Level', cur, [['', 'Choose a level', true], ...opts],
            (v) => app.update(i, { preset: v[0] === 'L' ? Number(v.slice(1)) : v.slice(1) }), 'preset'));
        if ((m.presets || []).some((p) => p.ledger !== undefined)) {
            out.push(h('p', { class: 'hint' }, 'A level is written through the robot\'s facts (facts::lv(i)), so renaming it there never breaks the route.'));
        }
    }
    if (step.do === 'set' && m) {
        out.push(select('Position', step.position ?? '', [['', 'Choose a position', true], ...(m.positions || []).map((p) => [p, p])],
                        (v) => app.update(i, { position: v }), 'position'));
    }
    if (step.do === 'run' || step.do === 'startRun') {
        out.push(h('div', { class: 'field-row' }, numField('Volts', step.volts, 'V', (v) => app.update(i, { volts: v }), { placeholder: '-12 to 12' }),
                   numField('For', step.forS, 's', (v) => app.update(i, { forS: v }))));
    }
    if (['move', 'startMove', 'home', 'startHome', 'waitUntilDone'].includes(step.do)) {
        out.push(h('div', { class: 'field-row one' }, numField(step.do === 'waitUntilDone' ? 'At most this much longer' : 'Time limit', step.boundS, 's',
            (v) => app.update(i, { boundS: v }), { placeholder: 'type it', bad: !Number.isFinite(step.boundS), name: 'boundS' })));
    }
    const pair = { move: 'startMove', startMove: 'move', run: 'startRun', startRun: 'run', home: 'startHome', startHome: 'home' };
    if (pair[step.do]) {
        const waits = ['move', 'run', 'home'].includes(step.do);
        out.push(toggle('Wait until it is done', waits ? 'the route waits here' : 'started: the route keeps going (a "wait until done" collects it)',
                        waits, (on) => app.update(i, { do: on ? (waits ? step.do : pair[step.do]) : (waits ? pair[step.do] : step.do) }), 'waitsHere'));
    }
    return out;
}

/** Render the inspector into `el`. */
export function renderInspector(el, app) {
    const st = app.state();
    const route = app.route();
    const issues = app.issues();
    if (st.selected === null || st.selected === 'start' || route.steps[st.selected] === undefined) {
        const startIssues = issues.filter((x) => x.stepIndex === -1);
        const s0 = route.start;
        mount(el, 
            h('h2', { class: 'insp-title' }, icon('start'), 'The route'),
            startIssues.map((x) => issueBox(x, app)),
            h('div', { class: 'fld' }, h('span', {}, 'Its name on the brain\'s menu'),
              h('p', { class: 'route-name-fixed' }, h('strong', {}, `"${route.name}"`), route.name === ROUTE_NAME
                  ? ' — both robots\' code registers the skills route by this name, so it stays.'
                  : ` — not "${ROUTE_NAME}", the name both robots' code registers the skills route by: Name it "${ROUTE_NAME}" above.`)),
            h('div', { class: 'section' }, h('h3', {}, 'Robot one\'s start'),
              s0 ? [
                  h('div', { class: 'field-row' },
                    numField('x', s0.x, 'in', (v) => app.setStart({ ...s0, x: v }), { name: 'startX' }),
                    numField('y', s0.y, 'in', (v) => app.setStart({ ...s0, y: v }), { name: 'startY' })),
                  h('div', { class: 'field-row' }, numField('Heading', s0.headingDeg, 'deg', (v) => app.setStart({ ...s0, headingDeg: v }), { name: 'startH' }),
                    h('div', { class: 'hint' }, '0 faces the right wall, 90 faces blue. Drag the red knob on the field.')),
              ] : h('p', { class: 'hint' }, 'Click the field to place robot one where it starts; then drag it, and its red knob to turn it.')),
            h('div', { class: 'section' }, h('h3', {}, 'The robots'),
              h('p', { class: 'hint' }, app.robotsWords())),
        );
        return;
    }
    const i = st.selected;
    const step = route.steps[i];
    const mine = issues.filter((x) => x.stepIndex === i);
    const issuesOf = (f) => mine.filter((x) => x.field === f);
    const [title] = stepTitle(step, app.robot());
    const body = [];
    const point = () => h('div', { class: 'field-row' },
        numField('x', step.x, 'in', (v) => app.update(i, { x: v }), { bad: issuesOf('x').length > 0, name: 'x' }),
        numField('y', step.y, 'in', (v) => app.update(i, { y: v }), { name: 'y' }));
    switch (step.do) {
        case 'driveTo':
        case 'reverseTo':
            body.push(point());
            if (app.robot() && app.robot().drive && app.robot().drive.tank) {
                body.push(toggle(step.do === 'reverseTo' ? 'Turn its back to the point first' : 'Turn to face it first',
                                 `a tank steers back onto its line only within ${fmt(app.robot().drive.maxSteerDeg)} deg`, !!step.faceFirst,
                                 (v) => app.update(i, { faceFirst: v }), 'faceFirst'));
            }
            body.push(toggle('Backwards', 'the robot\'s back leads', step.do === 'reverseTo', (v) => app.update(i, { do: v ? 'reverseTo' : 'driveTo' }), 'backwards'));
            body.push(careSection(step, i, app, issuesOf));
            break;
        case 'turnTo':
        case 'face':
        case 'faceAway': {
            body.push(select('Turn', step.do, [['face', 'to face a point'], ['faceAway', 'its back to a point'], ['turnTo', 'to a heading']],
                (v) => app.update(i, v === 'turnTo'
                    ? { do: 'turnTo', headingDeg: Number.isFinite(app.plan()[i].to?.headingDeg) ? app.plan()[i].to.headingDeg : 0, x: undefined, y: undefined }
                    : { do: v, x: step.x ?? app.plan()[i].from?.x, y: step.y ?? app.plan()[i].from?.y, headingDeg: undefined }), 'turnKind'));
            body.push(step.do === 'turnTo'
                ? h('div', { class: 'field-row' }, numField('Heading', step.headingDeg, 'deg', (v) => app.update(i, { headingDeg: v }), { name: 'headingDeg' }),
                    h('div', { class: 'hint' }, '0 faces the right wall, 90 faces blue, -90 faces red.'))
                : point());
            body.push(careSection(step, i, app, issuesOf));
            break;
        }
        case 'curveTo':
        case 'reverseCurveTo': {
            const pts = step.points || [];
            const setPt = (k, patch) => app.update(i, { points: pts.map((p, j) => (j === k ? { ...p, ...patch } : p)) });
            body.push(h('table', { class: 'points' }, h('tbody', {}, pts.map((p, k) => h('tr', {},
                h('td', { class: 'muted' }, String(k + 1)),
                h('td', {}, numField('x', p.x, 'in', (v) => setPt(k, { x: v }))),
                h('td', {}, numField('y', p.y, 'in', (v) => setPt(k, { y: v }))),
                h('td', {}, numField('nose', p.headingDeg, '°', (v) => setPt(k, { headingDeg: v }), { placeholder: 'any' })),
                h('td', {}, h('button', { class: 'btn icon ghost', 'aria-label': `Remove point ${k + 1}`, disabled: pts.length <= 1,
                                          onclick: () => app.update(i, { points: pts.filter((_, j) => j !== k) }) }, icon('trash'))))))));
            body.push(h('p', { class: 'hint' }, 'Drag a point on the field. "nose" is the way the robot faces there (empty: the library picks the smooth way).'));
            body.push(toggle('Backwards', 'the robot\'s back leads; every heading is still its nose', step.do === 'reverseCurveTo',
                             (v) => app.update(i, { do: v ? 'reverseCurveTo' : 'curveTo' }), 'backwards'));
            body.push(careSection(step, i, app, issuesOf));
            break;
        }
        case 'wait':
            body.push(h('div', { class: 'field-row one' }, numField('Wait', step.seconds, 's', (v) => app.update(i, { seconds: v }), { name: 'seconds' })));
            body.push(h('p', { class: 'hint' }, 'The run\'s own wait: at 57 s the run cuts it at once.'));
            break;
        default:
            body.push(...mechanismFields(step, i, app));
    }
    mount(el, 
        h('h2', { class: 'insp-title' }, h('span', { class: 'step-icon' }, icon(stepIcon(step))), `Step ${i + 1}`),
        h('p', { class: 'muted', style: 'margin-top:-8px' }, title),
        mine.map((x) => issueBox(x, app)),
        body,
        failSection(step, i, app),
        h('div', { class: 'section row' },
          h('button', { class: 'btn', onclick: () => app.duplicate(i) }, icon('dup'), 'Duplicate'),
          h('button', { class: 'btn danger', onclick: () => app.remove(i) }, icon('trash'), 'Delete step')),
    );
}
