// export_drawer.mjs — EXPORT (RMP1 brief D4, A.3.7; RMP1-C; RMP1-F). While any route has an error the drawer lists ONLY
// the errors, each one click from its step. Otherwise ONE numbered card for THIS browser (Chrome or Edge: connect the
// folder, write into the ONE marked region of src/team/skills_route.cpp; any other: pick the file, get it back, replace
// it; or Copy), then a result that is never silent: a write is READ BACK (files.mjs), a refusal is red with its fix, a
// hand edit is the student's choice. A success hides the steps and shows "Will it build?" and Next (to_robot.mjs). The
// code is one click away. The line each autons file needs to register the routes is CHECKED, never written (D6, R2).
// RMP1-I: on the page tools/route_builder.py serves (the 'local' host), Next is Send to robot (send_panel.mjs), and
// `{send}` (the top bar's Send) writes, then sends. A robot whose autons file lacks its line is never sent to (its ✗ stays).

import { h, icon } from './dom.mjs';
import { exportRegion } from '../model/export.mjs';
import { teamRoutesLine, SKILLS_ROUTE_FILE, ROBOT_ONE_AUTONS_FILE, ROBOT_TWO_AUTONS_FILE } from '../model/region.mjs';
import { copyText } from '../host/files_download.mjs';
import { exportSummary } from '../model/to_robot.mjs';
import { robotName, robotBridge } from '../model/robot_names.mjs';
import { resultCard, copyButton, willItBuild, pastePicture, nextCard } from './to_robot.mjs';

/** Open the export drawer. */
export async function openExportDrawer(app, { send = null } = {}) {
    if (document.querySelector('.drawer')) {
        return;  // one at a time (Ctrl+E pressed again)
    }
    const routes = app.routes();
    const here = app.at();
    const errors = [...app.issues().filter((x) => x.level === 'error').map((x) => ({ ...x, route: here })), ...app.otherErrors()];
    const files = app.files();
    const local = files.kind === 'local' && app.sendPanel;
    const close = () => {
        scrim.remove();
        drawer.remove();
        document.removeEventListener('keydown', onKey);
        app.focusField();
    };
    const onKey = (e) => {
        if (e.key === 'Escape') {
            close();
        }
    };
    document.addEventListener('keydown', onKey);
    const scrim = h('div', { class: 'scrim', onclick: close });
    const body = h('div', { class: 'drawer-body' });
    const drawer = h('aside', { class: 'drawer', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'export-title' },
        h('div', { class: 'drawer-head' }, icon('code'), h('h2', { id: 'export-title' }, routes.length > 1 ? `Export the ${routes.length} routes` : 'Export the route'), h('span', { class: 'spacer' }),
          h('button', { class: 'btn icon ghost', 'aria-label': 'Close', onclick: close }, icon('close'))),
        body);
    document.body.append(scrim, drawer);

    const refuse = (title, list) => {
        body.append(h('div', { class: 'banner error', role: 'alert' }, icon('error'),
            h('div', {}, h('strong', {}, title),
              h('ul', {}, list.slice(0, 6).map((x) => h('li', {}, routes.length > 1 && x.route !== undefined ? h('strong', {}, `"${routes[x.route].name}" `) : '',
                  x.stepIndex >= 0 ? `Step ${x.stepIndex + 1}: ` : '', x.words, ' ',
                  h('button', { class: 'btn ghost', onclick: () => {
                      close();
                      if (x.route !== undefined) {
                          app.switchRoute(x.route);
                      }
                      app.select(x.stepIndex >= 0 ? x.stepIndex : 'start');
                  } }, 'Go there')))),
              list.length > 6 ? h('p', { class: 'hint' }, `…and ${list.length - 6} more: the status bar's "Go to step" walks them.`) : null)));
        drawer.querySelector('button[aria-label="Close"]').focus();
    };
    if (errors.length) {
        refuse(`Fix ${errors.length === 1 ? 'this error' : `these ${errors.length} errors`} first: the robot would not run ${routes.length > 1 ? 'these routes' : 'this route'}.`, errors);
        return;
    }
    let code;
    try {
        code = exportRegion(routes, app.robot());
    } catch (e) {
        // never reached while the checks hold (every number the export prints is one they ask for): if it is,
        // the student sees why, not a dead button
        refuse('The route builder could not write this route. Tell the coordinator these words:', [{ stepIndex: -1, words: String(e && e.message ? e.message : e) }]);
        return;
    }

    const summary = exportSummary(routes);
    const steps = h('div', { class: 'card xr-steps' });
    const result = h('div', { class: 'xr-result', 'aria-live': 'polite' });
    const next = h('div');
    const codeView = h('details', { class: 'xr-code' },
        h('summary', {}, `See the code: ${code.split('\n').length} lines of C++, every step explained in a comment`),
        h('pre', { class: 'code', tabindex: '0', 'aria-label': 'The exported code' }, code));
    /** A success: the result, then Next; the steps card has done its job. */
    const done = (card) => {
        steps.remove();
        result.replaceChildren(card);
        next.replaceChildren(local ? sendCard() : nextCard(summary.names));
    };
    /** RMP1-I: Send to robot one (and two, when robot two's build takes the routes too); the panel under them. A robot
     *  whose autons file lacks its addTeamRoutes line would build and upload and never list the routes: no Send for it. */
    const sendCard = () => {
        const panel = app.sendPanel;
        const shared = app.baked.sharedRoute && app.baked.sharedRoute.robots;
        const robots = ['one', ...(shared && shared.includes('two') ? ['two'] : [])];
        const lacks = (r) => (missing || []).some((m) => m.robot === r);
        const buttons = robots.map((r, k) => h('button', {
            class: `btn${k === 0 && !lacks(r) ? ' primary' : ''}`, 'data-send': r, disabled: panel.busy() || lacks(r), onclick: async () => {
                buttons.forEach((b) => { b.disabled = true; });
                missing = await linesMissing().catch(() => missing);  // read again now (a helper gone: the send says so)
                if (lacks(r)) {
                    next.replaceChildren(sendCard());
                    return;
                }
                result.querySelectorAll('.xr-build').forEach((x) => !x.querySelector('.bad') && x.remove());  // the build answers it now
                await panel.start(r);
                buttons.forEach((b, i) => { b.disabled = lacks(robots[i]); });
            } }, icon('send'), `Send to ${robotName(r)}`));
        return h('section', { class: 'card xr-next send-card', 'aria-labelledby': 'send-title' },
            h('h3', { id: 'send-title' }, 'Next: send it to the robot'),
            h('p', { class: 'muted' }, 'Plug the BRAIN in with USB (not the controller), then:'),
            h('div', { class: 'row' }, buttons),
            robots.filter(lacks).map((r) => h('p', { class: 'xr-warn', 'data-lacks': r },
                `${robotName(r)} is not sent to: ${r === 'one' ? ROBOT_ONE_AUTONS_FILE : ROBOT_TWO_AUTONS_FILE} is missing ${teamRoutesLine(r === 'one' ? 'One' : 'Two')} `
                + '— tell the coordinator.')),
            app.baked.build.differs ? h('p', { class: 'xr-warn' }, app.baked.build.differs) : null,
            panel.element);
    };
    /** A refusal: red, its reason, its one fix (the one highlighted button now). */
    const fail = (title, words, ...fix) => {
        steps.querySelectorAll('.btn.primary').forEach((b) => b.classList.remove('primary'));
        result.replaceChildren(resultCard('error', title, h('p', {}, words), fix.length ? h('div', { class: 'xr-fix' }, h('strong', {}, 'The fix: '), fix) : null));
        next.replaceChildren();
    };
    const copy = async () => {
        if (await copyText(code)) {
            done(resultCard('ok', 'Copied', h('p', {}, `Now paste it over the two ROUTE MAKER lines in ${SKILLS_ROUTE_FILE}, and everything between them:`),
                pastePicture(), willItBuild('copy', missing)));
        } else {
            codeView.open = true;
            fail('Not copied', 'This browser would not copy.', 'select the code below and copy it by hand.');
        }
    };
    const copyBtn = (label = 'Copy the code') => h('button', { class: 'btn', onclick: copy }, icon('copy'), label);
    const connectBtn = () => h('button', { class: 'btn primary', onclick: async () => {
        close();
        await app.connect();
        openExportDrawer(app);
    } }, icon('folder'), 'Connect your shulib folder');

    /** The student's choice for a hand edit: keep it (nothing written) or overwrite it (`again(true)`). */
    const askHandEdit = (out, again) => {
        result.replaceChildren(resultCard('warn', 'Someone changed this code by hand', h('p', {}, out.words),
            h('div', { class: 'row' },
                h('button', { class: 'btn', onclick: () => fail('Not written: their change was kept', 'Nothing was written.',
                    'copy the code and put your routes in by hand, or export again and choose Overwrite. ', copyBtn()) }, 'Keep their change'),
                h('button', { class: 'btn danger', onclick: () => again(true) }, 'Overwrite'))));
    };
    const orCopy = h('p', { class: 'xr-or' }, 'Or ', copyBtn(), ' and paste it in yourself.');
    /** Markers gone, doubled or out of order: they go back first, and Copy goes (it pastes over those lines). */
    const putBack = () => {
        orCopy.remove();
        return [`ask the coordinator to put back its one pair of ROUTE MAKER lines (git checkout ${SKILLS_ROUTE_FILE} `
            + 'undoes changes not yet committed).'];
    };
    /** A write that failed or vanished: write again (the local page has no folder to connect). */
    const reconnect = () => (local ? [h('button', { class: 'btn primary', onclick: () => writeOut(false) }, 'Write again')]
        : ['connect your shulib folder again, then write again. ', connectBtn()]);
    const FIX = {
        noMarkers: putBack,
        markerCount: putBack,
        markerOrder: putBack,
        failed: () => reconnect(),
        gone: () => reconnect(),
        noFolder: () => [connectBtn()],
        differs: () => ['close the file in your editor (or let the sync finish), then ',
                        h('button', { class: 'btn primary', onclick: () => writeOut(false) }, 'Write again')],
    };
    let missing = null;  // the robots whose autons file lacks its line (read when the folder is connected; null: not read)
    /** The robots whose autons file lacks its addTeamRoutes line (files.teamRoutesLines()'s `lines`): [{robot, file, line}]. */
    const missingIn = (lines) => [['One', ROBOT_ONE_AUTONS_FILE], ['Two', ROBOT_TWO_AUTONS_FILE]].filter(([robot]) => lines[robot] === 'missing')
        .map(([robot, file]) => ({ robot: robot.toLowerCase(), file, line: teamRoutesLine(robot) }));
    const linesMissing = async () => missingIn(await files.teamRoutesLines());
    /** A write that landed and then read back wrong is not "Not written": the file did change. */
    const TITLE = { differs: 'Written, but it read back different', gone: 'Written, then the file was gone' };
    const writeOut = async (overwrite) => {
        const out = await files.exportToFile(routes, app.project(), { overwrite, robot: app.robot() });
        if (out.ok) {
            app.setProject(out.project, 'exported');
            const saved = await app.saveQuietly().catch(() => false);  // false: route_maker.json changed in the folder since this page read it
            done(resultCard('ok', `Written and checked: ${SKILLS_ROUTE_FILE}`, h('p', { class: 'xr-sum' }, summary.words),
                saved === false ? h('p', { class: 'xr-warn' }, 'src/team/route_maker.json was NOT saved: it changed in the folder since this page read it. Press Save to choose which to keep.') : null,
                willItBuild('write', missing)));
            const go = send && local && next.querySelector(`[data-send="${send}"]:not([disabled])`);
            send = null;
            if (go) {
                go.click();
            }
        } else if (out.why === 'handEdit') {
            askHandEdit(out, writeOut);
        } else {
            fail(TITLE[out.why] || `Not written into ${SKILLS_ROUTE_FILE}`, out.words, ...(FIX[out.why] ? FIX[out.why]() : ['use Copy instead. ', copyBtn()]));
        }
    };
    const downloadOut = async (overwrite) => {
        const out = await files.exportByDownload(routes, app.project(), { overwrite, robot: app.robot() });
        if (out.ok) {
            // the project now remembers this export (its hash) -- but only in this page until route_maker.json is
            // saved: without that, the next export from the saved file would take this code for a hand edit
            app.setProject(out.project, 'unsaved');
            done(resultCard('ok', 'Downloaded skills_route.cpp',
                h('p', { class: 'xr-sum' }, `Now replace ${SKILLS_ROUTE_FILE} with it. `, copyButton(SKILLS_ROUTE_FILE, 'the path')),
                h('p', {}, 'Use the NEWEST skills_route.cpp in your Downloads: a second download gets a number, like skills_route(1).cpp.'),
                h('p', {}, 'Then save the project: route_maker.json remembers what was exported, so the next export knows this code is the builder\'s. ',
                  h('button', { class: 'btn small', onclick: () => app.save() }, icon('save'), 'Save route_maker.json')),
                willItBuild('download')));
        } else if (out.why === 'handEdit') {
            askHandEdit(out, downloadOut);
        } else if (out.why !== 'cancelled' && out.picked !== 'skills_route.cpp') {
            fail('Not downloaded', `You picked ${out.picked}: the routes go into ${SKILLS_ROUTE_FILE}.`, `pick ${SKILLS_ROUTE_FILE}. `,
                h('button', { class: 'btn primary', onclick: () => downloadOut(false) }, icon('download'), 'Pick skills_route.cpp'));
        } else if (out.why !== 'cancelled') {
            fail('Not downloaded', out.words, ...putBack());
        }
    };

    /** One numbered step: `state` 'now' (the one highlighted), 'done' or 'later'. */
    const step = (n, state, control, hint) => h('li', { class: `xr-step ${state}` },
        h('span', { class: 'xr-num' }, state === 'done' ? icon('check') : String(n)), h('div', {}, control, h('small', {}, hint)));
    steps.append(h('h3', {}, 'Put it into your robot\'s code'));
    let fixFirst = null;  // the banner when the markers must go back first: no Copy, and focus on it
    let ready = false;
    if (files.canWriteFolder) {
        const connected = files.connected();
        const st = connected ? await files.skillsFileState() : { state: 'unknown' };
        const lines = connected ? await files.teamRoutesLines() : { One: 'unknown', Two: 'unknown' };
        ready = connected && st.state === 'ready';
        missing = connected ? missingIn(lines) : null;
        steps.append(h('p', { class: 'muted' }, `${local ? 'The page' : 'Chrome or Edge: the page'} writes it into the file for you.`),
            h('ol', { class: 'xr-list' },
                step(1, connected ? 'done' : 'now', connected ? h('strong', {}, `Connected: ${files.folderName()}`) : connectBtn(),
                     'your shulib folder: the one with src inside'),
                step(2, ready ? 'now' : 'later', h('button', { class: `btn${ready ? ' primary' : ''}`, disabled: !ready, onclick: () => writeOut(false) },
                    icon('download'), `Write into ${SKILLS_ROUTE_FILE}`), 'only the lines between its two ROUTE MAKER lines change')));
        if (connected && !ready) {
            const banner = h('div', { class: 'banner warn', tabindex: '-1' }, icon('warn'),
                h('div', {}, st.state === 'noFile'
                    ? `${SKILLS_ROUTE_FILE} is not in this folder yet: it arrives with ${robotName('two')}'s update. Until then, use Copy.`
                    : [st.state === 'noMarkers' ? `${SKILLS_ROUTE_FILE} has no ROUTE MAKER lines: someone removed them.` : st.words,
                        h('div', { class: 'xr-fix' }, h('strong', {}, 'The fix: '), putBack())]));
            steps.append(banner);
            fixFirst = st.state === 'noFile' ? null : banner;
        }
        for (const [robot, file] of connected ? [['One', ROBOT_ONE_AUTONS_FILE], ['Two', ROBOT_TWO_AUTONS_FILE]] : []) {
            steps.append(h('p', { class: 'hint', 'data-line': robot }, lines[robot] === 'present' ? `${robotBridge(robot.toLowerCase())} registers every route: its line is in ${file}. ✓`
                : `The line that registers the routes on ${robotBridge(robot.toLowerCase())} is missing from ${file}: tell the coordinator. The line: ${teamRoutesLine(robot)}`));
        }
    } else {
        steps.append(h('p', { class: 'muted' }, 'This browser cannot write into a folder (Chrome or Edge can), so it is two steps:'),
            h('ol', { class: 'xr-list' },
                step(1, 'now', h('button', { class: 'btn primary', onclick: () => downloadOut(false) }, icon('download'), 'Pick skills_route.cpp and download the updated one'),
                     `pick your ${SKILLS_ROUTE_FILE}: you get it back with the routes in`),
                step(2, 'later', h('span', { class: 'xr-path' }, 'Replace ', h('code', {}, SKILLS_ROUTE_FILE), ' with the downloaded file ',
                    copyButton(SKILLS_ROUTE_FILE, 'the path')), 'in your shulib folder; your editor reloads it')));
    }
    if (!fixFirst) {
        steps.append(orCopy);
    }
    body.append(steps, result, next, codeView);
    (fixFirst || steps.querySelector('.btn.primary') || orCopy.querySelector('button')).focus();
    if (send && local && ready) {
        await writeOut(false);
    }
}
