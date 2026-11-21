// timeline.mjs — THE PREVIEW DOCK across the bottom of the field (RMP1-A2u; brief §1.4, D5): while PREVIEW runs, its
// progress and Cancel; then the TIMELINE SCRUBBER — play / pause, the step before / after, the speed, a track with
// every step's span in its verdict's colour (drag it, click it, or use the keys), the pair check's moment in red, the
// run against the 60 s (the 57 s cut marked) — and one line saying what the step playing did, in the library's words.

import { h, icon, mount } from './dom.mjs';
import { fmt } from '../model/numbers.mjs';
import { noteWords } from '../model/playback.mjs';
import { robotName } from '../model/robot_names.mjs';

const SPEEDS = [[0.5, 'half speed'], [1, 'normal speed'], [2, 'twice the speed'], [4, 'four times the speed']];
const secs = (t) => `${t.toFixed(2)} s`;
const TOP = { ok: 0, skipped: 1, cut: 2, failed: 3 };

/** The dock in `host` (the field pane). `pv` is the preview controller (preview.mjs). */
export function createDock(host, pv) {
    const el = h('section', { class: 'dock', 'aria-label': 'PREVIEW' });
    el.hidden = true;
    host.append(el);
    // the field, its controls and the toasts sit above the dock, however its rows wrap (a narrower window, the details)
    const room = () => {
        host.style.setProperty('--dock-h', `${el.offsetHeight}px`);
        document.documentElement.style.setProperty('--dock-room', el.hidden ? '0px' : `${el.offsetHeight + 12}px`);
    };
    new ResizeObserver(room).observe(el);
    let parts = null;

    function track(play) {
        const pct = (t) => `${(100 * t) / Math.max(play.duration, 1e-6)}%`;
        // the shortest on top (the worst on a tie): a step narrower than its edge covers the next one's start, not the reverse
        const len = (s) => s.t1 - s.t0;
        const ran = play.one.steps.filter((s) => s.t0 !== null).sort((a, b) => len(b) - len(a) || TOP[a.state] - TOP[b.state]);
        const segs = ran.map((s) => {
            const w = s.t1 > s.t0 ? `calc(${pct(s.t1 - s.t0)} - 2px)` : '';
            return h('div', { class: `tl-seg ${s.state}${w ? '' : ' tick'}`, style: `left:${pct(s.t0)};${w ? `width:${w}` : ''}`,
                              title: `Step ${s.n}: ${s.words}` }, w ? h('span', {}, String(s.n)) : null);
        });
        const fail = play.pair && !play.pair.ok && play.pair.failAt !== null
            ? h('button', { class: 'pair-mark', style: `left:${pct(play.pair.failAt)}`, title: 'The robots come too close here',
                            'aria-label': `Go to ${secs(play.pair.failAt)}: the robots come within ${fmt(play.pair.gapIn)} in`,
                            onclick: (e) => { e.stopPropagation(); pv.seek(play.pair.failAt); } }) : null;
        const head = h('div', { class: 'playhead' });
        const t = h('div', { class: 'track', role: 'slider', tabindex: '0', 'aria-label': 'The run, in time',
                             'aria-valuemin': '0', 'aria-valuemax': play.duration.toFixed(2) }, segs, fail, head);
        const seekAt = (e) => {
            const r = t.getBoundingClientRect();
            pv.seek((Math.min(Math.max(e.clientX - r.left, 0), r.width) / r.width) * play.duration);
        };
        t.addEventListener('pointerdown', (e) => {
            if (e.button !== 0 || e.target.closest('.pair-mark')) {
                return;
            }
            t.setPointerCapture(e.pointerId);
            pv.scrub(true);
            seekAt(e);
            const move = (m) => seekAt(m);
            const up = () => {
                t.removeEventListener('pointermove', move);
                t.removeEventListener('pointerup', up);
                t.removeEventListener('pointercancel', up);
                pv.scrub(false);
            };
            t.addEventListener('pointermove', move);
            t.addEventListener('pointerup', up);
            t.addEventListener('pointercancel', up);
        });
        return { t, head };
    }

    function budget(play) {
        const p = (t) => `${(100 * Math.min(t, play.period)) / play.period}%`;
        return h('div', { class: 'budget', title: `The run against the period: cut at ${fmt(play.park)} s, everything stops at ${fmt(play.stop)} s` },
            h('div', { class: 'budget-bar' }, h('div', { class: 'budget-run', style: `width:${p(play.duration)}` }),
              h('div', { class: 'budget-cut', style: `left:${p(play.park)}` })),
            h('span', {}, `${secs(play.duration)} of the ${fmt(play.period)} s · cut at ${fmt(play.park)} s`));
    }

    /** Draw the dock for the controller's state (a full redraw: on a new run, a phase, an error, stale). */
    function render() {
        draw();
        host.classList.toggle('with-dock', !el.hidden);
        room();
    }
    function draw() {
        const st = pv.state();
        el.hidden = st.phase === 'idle' && !st.words;
        parts = null;
        if (st.phase === 'running') {
            mount(el, h('div', { class: 'dock-row' },
                h('span', { class: 'spinner', 'aria-hidden': 'true' }),
                h('span', { class: 'dock-phase', role: 'status' }, st.progress),
                h('span', { class: 'spacer' }),
                h('button', { class: 'btn', onclick: () => pv.cancel() }, icon('close'), 'Cancel')));
            return;
        }
        if (!st.play || st.stale) {
            mount(el, h('div', { class: `dock-row ${st.stale ? '' : 'error'}` },
                icon(st.stale ? 'warn' : 'error'),
                h('span', { class: 'dock-words', role: 'status' }, st.stale
                    ? 'The route changed since this PREVIEW. Press PREVIEW (Ctrl+Enter) to run it again.' : st.words),
                h('span', { class: 'spacer' }),
                h('button', { class: 'btn icon ghost', 'aria-label': 'Close', title: 'Close (Esc)', onclick: () => pv.close() }, icon('close'))));
            return;
        }
        const play = st.play;
        const tr = track(play);
        const btn = (name, label, f, extra = {}) => h('button', { class: 'btn icon ghost', 'aria-label': label, title: label, onclick: f, ...extra }, icon(name));
        parts = {
            playBtn: btn('play', 'Play (Space)', () => pv.toggle(), { class: 'btn icon primary play' }),
            time: h('span', { class: 'dock-time', 'aria-live': 'off' }),
            step: h('span', { class: 'chip dock-step' }),
            words: h('div', { class: 'dock-words' }),
            ...tr,
        };
        const head = play.headline;
        mount(el,
            h('div', { class: 'dock-row' },
                btn('prev', 'The step before (Left)', () => pv.step(-1)),
                parts.playBtn,
                btn('next', 'The step after (Right)', () => pv.step(1)),
                parts.time, parts.step,
                h('span', { class: 'spacer' }),
                h('div', { class: 'speeds', role: 'group', 'aria-label': 'Speed' },
                  SPEEDS.map(([s, words]) => h('button', { class: `speed${st.speed === s ? ' on' : ''}`, 'aria-pressed': String(st.speed === s),
                                                           title: `Play at ${words}`, onclick: () => pv.speed(s) }, `${s}×`))),
                btn('help', 'About this run', () => pv.details(), { 'aria-expanded': String(st.details), title: 'About this run: what it is, and its PLACEHOLDER facts' }),
                btn('close', 'Close the preview (Esc)', () => pv.close())),
            parts.t,
            h('div', { class: 'dock-row foot' }, parts.words,
              h('span', { class: `chip ${head.mark === 'ok' ? 'ok' : head.mark === 'cut' ? 'warn' : 'error'}`, title: 'How the run ended (the Auton Tester\'s word)' },
                head.two ? `${robotName('one')}: ${head.word} · ${robotName('two')}: ${head.two}` : head.word),
              budget(play)),
            st.details ? details(st) : null);
        update();
    }

    function details(st) {
        const { play, pairWords } = st;
        const of = (run) => {
            const rec = run.record;
            const who = rec.robot ? robotName(rec.robot) : 'the robot';
            return [
                rec.owed.length ? h('div', { class: 'banner warn' }, icon('warn'), h('div', {},
                    h('strong', {}, `${rec.owed.length} of ${who}'s facts are PLACEHOLDERS`),
                    ` (not measured yet, in ${rec.ledgerFile}): a time that uses one is not the robot's.`,
                    h('ul', {}, rec.owed.map((x) => h('li', {}, x))))) : null,
                rec.notices.map((x) => h('p', { class: 'muted' }, x)),
                run.notes.map((f) => h('p', { class: f.kind === 'THROUGH' ? 'inline-err' : 'muted' }, noteWords(f))),
                h('details', { class: 'dock-log' }, h('summary', {}, `Every line the library printed for ${who} (${rec.lines.length})`),
                  h('pre', {}, rec.lines.map((l) => `${secs(l.t)}  ${l.sub}  ${l.text}`).join('\n'))),
            ];
        };
        return h('div', { class: 'dock-details' },
            h('p', {}, h('strong', {}, 'The host test plant, not the robot. '), play.one.record.robotLine),
            (play.one.record.mechanisms || []).length ? h('p', { class: 'ms-about' }, 'The mechanisms beside the field are the host '
                + 'test plant\'s too: each height and time comes from its fakes and the declaration\'s numbers, not from the robot.') : null,
            pairWords ? h('p', {}, pairWords) : null,
            of(play.one),
            play.two ? [h('h4', {}, `${robotName('two')}: ${play.two.headline.word}`), of(play.two)] : null,
            play.pair ? h('p', { class: play.pair.ok ? '' : 'inline-err' }, play.pair.sentence) : null);
    }

    /** Move the playhead, the time, the step chip and the line (every frame: no redraw). */
    function update() {
        if (!parts) {
            return;
        }
        const st = pv.state();
        const { play, t } = st;
        const i = st.stepIndex;
        const s = i >= 0 ? play.one.steps[i] : null;
        parts.head.style.left = `${(100 * t) / Math.max(play.duration, 1e-6)}%`;
        parts.time.textContent = `${secs(t)} / ${secs(play.duration)}`;
        parts.t.setAttribute('aria-valuenow', t.toFixed(2));
        parts.t.setAttribute('aria-valuetext', `${secs(t)}${s ? `, step ${s.n}` : ''}`);
        parts.step.textContent = s ? `Step ${s.n}: ${pv.stepName(i)}` : t >= play.duration ? 'The end' : 'Between steps';
        parts.step.className = `chip dock-step ${s ? s.state : ''}`;
        if (parts.playing !== st.playing) {
            parts.playing = st.playing;
            parts.playBtn.replaceChildren(icon(st.playing ? 'pause' : 'play'));
            parts.playBtn.setAttribute('aria-label', st.playing ? 'Pause (Space)' : 'Play (Space)');
        }
        const pairBad = play.pair && !play.pair.ok;
        const hit = [play.one, play.two].map((r) => r && r.through).find((f) => f && f.at < 0 && t >= f.t);  // in the park
        const key = `${i}|${pairBad}|${!!hit}`;
        if (parts.key === key) {
            return;
        }
        parts.key = key;
        const two = s && s.two ? [h('strong', {}, ` ${robotName('two')}: ${s.two.words}. `), s.two.library.join(' ')] : [];
        const lead = s && s.state !== 'ok' ? [h('strong', {}, `Step ${s.n}: ${s.words}. `), s.library.join(' '), ...two]
            : two.length ? [h('strong', {}, `Step ${s.n}:`), ...two]
                : pairBad ? [h('strong', {}, `${robotName('one')} and ${robotName('two')} come too close. `), play.pair.sentence]
                    : hit ? [h('strong', {}, `THROUGH the ${hit.element}. `), noteWords(hit)]
                        : [s && s.placeholder ? s.placeholder : 'The robot as the real library drove it on the host test plant — not the robot: time it on the robot.'];
        mount(parts.words, lead);
        parts.words.title = parts.words.textContent;
        const bad = s && s.state !== 'ok' ? s.state : s && s.two ? s.two.state : pairBad || hit ? 'failed' : '';
        parts.words.className = `dock-words ${bad}`;
    }

    return { render, update, element: el };
}
