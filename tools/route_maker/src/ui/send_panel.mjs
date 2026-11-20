// send_panel.mjs — SEND TO ROBOT (RMP1-I D4): one send at a time; tools/upload_auton.py's lines as they come, said
// plainly — the steps, ✓ BUILT, then ✓ UPLOADED and where the route is on the brain, or ✗ in its plain words — and
// "Show everything" (build/upload_auton.log). The panel outlives the export drawer: closing it mid-send loses nothing.

import { h, mount } from './dom.mjs';
import { sendStage } from '../model/to_robot.mjs';
import { resultCard } from './to_robot.mjs';

/** The panel over `send(robot, onLine) -> code` and `log() -> text`; `onChange` when it starts or ends. */
export function createSendPanel({ send, log, onChange = () => {} }) {
    let robot = 'one';
    let busy = false;
    let rows = [];
    const el = h('section', { class: 'send-panel', 'aria-live': 'polite', hidden: true });

    function render() {
        const steps = rows.filter((r) => r.kind === 'step' || r.kind === 'built');
        const ok = rows.find((r) => r.kind === 'uploaded');
        const bad = rows.find((r) => r.kind === 'failed');
        const details = bad ? rows.slice(rows.indexOf(bad) + 1).filter((r) => r.kind === 'detail') : [];
        const all = h('details', { class: 'send-all' }, h('summary', {}, 'Show everything'), h('pre', { class: 'code' }, '…'));
        all.addEventListener('toggle', async () => {
            if (all.open) {
                all.querySelector('pre').textContent = (await log().catch(() => '')) || 'Nothing was printed.';
            }
        });
        mount(el,
            h('ol', { class: 'send-steps' }, steps.map((r, k) => h('li', {
                class: r.kind === 'built' ? 'ok' : busy && k === steps.length - 1 ? 'now' : '' },
                h('span', { class: 'send-mark', 'aria-hidden': 'true' }, r.kind === 'built' ? '✓' : '•'), r.words))),
            ok ? resultCard('ok', ok.words, h('p', { class: 'send-brain' }, ok.then)) : null,
            bad ? resultCard('error', bad.words, bad.then ? h('p', {}, bad.then) : null, details.map((d) => h('p', { class: 'send-detail' }, d.words)))
                : null,
            busy ? null : all);
    }

    /** Send to robot `which`; resolves when it has ended (never throws: a failure is the panel's red card). */
    async function start(which) {
        if (busy) {
            return;
        }
        robot = which;
        busy = true;
        rows = [];
        el.hidden = false;
        render();
        onChange();
        let code = null;
        try {
            code = await send(which, (line) => {
                const r = sendStage(line);
                if (r) {
                    rows.push(r);
                    render();
                }
            });
        } catch (e) {
            rows.push({ kind: 'failed', words: 'Not sent', then: e && e.message ? e.message : String(e) });
        }
        if (code !== null && !rows.some((r) => r.kind === 'uploaded' || r.kind === 'failed')) {
            rows.push({ kind: code === 0 ? 'step' : 'failed', words: code === 0 ? 'Finished.' : `Not sent (tools/upload_auton.py ended with ${code})`, then: '' });
        }
        busy = false;
        render();
        onChange(rows.find((r) => r.kind === 'uploaded' || r.kind === 'failed'));
    }

    return { element: el, start, busy: () => busy, robot: () => robot };
}
