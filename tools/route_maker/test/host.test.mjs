// host.test.mjs — FA1–FA4 THROUGH the file layer (files.mjs over the in-memory host: the same rules every
// host gets, A.6 [W12]), plus save/open and robots.mjs's choice of which robots a route is checked on.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { makeFiles, memoryHost } from '../src/host/files.mjs';
import { routeRobots } from '../src/host/robots.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { newProject, loadProject } from '../src/model/project.mjs';
import { newRoute, setStart, addStep } from '../src/model/route.mjs';
import { findRegion, teamRoutesLine } from '../src/model/region.mjs';
import { GOLDENS } from './helpers.mjs';

const EMPTY = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
const SKILLS = `#include "team/skills_route.hpp"\n\nnamespace shulib::bench {\n\n${EMPTY}\n}  // namespace shulib::bench\n`;
const ROUTE = addStep(setStart(newRoute(), { x: -36, y: -55.7, headingDeg: 0 }), { do: 'wait', seconds: 1 });

test('FA1 (host): export writes ONLY the region of src/team/skills_route.cpp; no file is created; no markers = refused', async () => {
    const host = memoryHost({ 'src/team/skills_route.cpp': SKILLS, 'src/team/autons.cpp': 'x' });
    const files = makeFiles(host);
    const out = await files.exportToFile(ROUTE, newProject());
    assert.ok(out.ok, out.words);
    assert.ok(host.store['src/team/skills_route.cpp'].includes('wait 1 s'));
    assert.deepEqual(Object.keys(host.store).sort(), ['src/team/autons.cpp', 'src/team/skills_route.cpp'], 'nothing else written');
    assert.equal(host.store['src/team/autons.cpp'], 'x');
    const noneHost = memoryHost({});
    assert.deepEqual(await makeFiles(noneHost).exportToFile(ROUTE, newProject()), { ok: false, why: 'noFile',
        words: 'There is no src/team/skills_route.cpp in this folder: it arrives with Black Bot\'s update. Until then, use Copy.' },
        'before robot two\'s update: no file, Copy only (RMP1-L: the robot named)');
    assert.deepEqual(noneHost.store, {}, 'the builder never creates a file under src/');
    const bare = memoryHost({ 'src/team/skills_route.cpp': 'int x;\n' });
    assert.equal((await makeFiles(bare).exportToFile(ROUTE, newProject())).why, 'noMarkers');
    assert.equal(bare.store['src/team/skills_route.cpp'], 'int x;\n', 'never pasted markers');
    const off = makeFiles(memoryHost({ 'src/team/skills_route.cpp': SKILLS }, { connected: false }));
    assert.equal((await off.exportToFile(ROUTE, newProject())).why, 'noFolder');
});

test('FA2 (host): the project keeps the hash; a hand edit is refused unless "overwrite"', async () => {
    const host = memoryHost({ 'src/team/skills_route.cpp': SKILLS });
    const files = makeFiles(host);
    const first = await files.exportToFile(ROUTE, newProject());
    assert.match(first.project.exports['src/team/skills_route.cpp#skills'], /^sha256:[0-9a-f]{64}$/);
    host.store['src/team/skills_route.cpp'] = host.store['src/team/skills_route.cpp'].replace('wait 1 s', 'wait 1 s (hand)');
    const r2 = addStep(ROUTE, { do: 'wait', seconds: 2 });
    const refused = await files.exportToFile(r2, first.project);
    assert.equal(refused.why, 'handEdit');
    assert.ok(host.store['src/team/skills_route.cpp'].includes('(hand)'), 'the hand edit is untouched');
    const forced = await files.exportToFile(r2, first.project, { overwrite: true });
    assert.ok(forced.ok);
    assert.ok(!host.store['src/team/skills_route.cpp'].includes('(hand)'));
    // the project saved and re-opened still carries the hash: the next export is not mistaken for a hand edit
    const reopened = loadProject((await files.saveProject(forced.project), host.store['src/team/route_maker.json'])).project;
    assert.ok((await files.exportToFile(ROUTE, reopened)).ok);
});

test('FA3 (host): a CRLF file stays CRLF, its final newline kept, nothing outside the region moved', async () => {
    const crlf = SKILLS.replace(/\n/g, '\r\n');
    const host = memoryHost({ 'src/team/skills_route.cpp': crlf });
    const out = await makeFiles(host).exportToFile(ROUTE, newProject());
    assert.ok(out.ok);
    const now = host.store['src/team/skills_route.cpp'];
    assert.ok(!/[^\r]\n/.test(now), 'a bare LF in a CRLF file (RM30)');
    assert.ok(now.endsWith('\r\n'));
    const a = findRegion(crlf);
    const b = findRegion(now);
    assert.equal(now.slice(0, b.start), crlf.slice(0, a.start));
    assert.equal(now.slice(b.end), crlf.slice(a.end));
});

test('FA4 (host): each autons file\'s line that registers the routes, read through the folder: present / missing / unknown without one', async () => {
    const both = { 'src/team/robot_one_autons.cpp': `  ${teamRoutesLine('One')}\n`, 'src/team/autons.cpp': `  ${teamRoutesLine('Two')}\n` };
    assert.deepEqual(await makeFiles(memoryHost(both)).teamRoutesLines(), { One: 'present', Two: 'present' });
    const swapped = { 'src/team/robot_one_autons.cpp': teamRoutesLine('Two'), 'src/team/autons.cpp': teamRoutesLine('One') };
    assert.deepEqual(await makeFiles(memoryHost(swapped)).teamRoutesLines(), { One: 'missing', Two: 'missing' }, 'each file its OWN robot');
    const missing = makeFiles(memoryHost({ 'src/team/autons.cpp': '  registry.add(forty);\n' }));
    assert.deepEqual(await missing.teamRoutesLines(), { One: 'missing', Two: 'missing' }, 'no robot-one file at all: missing');
    const off = makeFiles(memoryHost(both, { connected: false }));
    assert.deepEqual(await off.teamRoutesLines(), { One: 'unknown', Two: 'unknown' }, 'a browser without folder access says "not checked"');
});

test('save and open through the folder; the download way in another browser', async () => {
    const host = memoryHost({});
    const files = makeFiles(host);
    const p = { ...newProject(), routes: [ROUTE] };
    assert.equal((await files.saveProject(p)).where, 'folder');
    assert.deepEqual((await files.openProject()).project.routes[0], ROUTE);
    const off = memoryHost({}, { connected: false });
    const f2 = makeFiles(off);
    assert.equal((await f2.saveProject(p)).where, 'download');
    assert.equal(off.downloads[0].name, 'route_maker.json');
    off.willPick({ name: 'route_maker.json', text: off.downloads[0].text });
    assert.deepEqual((await f2.openProject()).project.routes[0], ROUTE);
    // the download export: pick the old file, get the updated one back
    off.willPick({ name: 'skills_route.cpp', text: SKILLS });
    const ex = await f2.exportByDownload(ROUTE, p);
    assert.ok(ex.ok);
    assert.equal(off.downloads[1].name, 'skills_route.cpp');
    assert.ok(off.downloads[1].text.includes(exportRegion(ROUTE)));
});

test('the download way remembers its export too: a second export from the SAVED project is not a hand edit; a real one is refused, and Overwrite works', async () => {
    const off = memoryHost({}, { connected: false });
    const files = makeFiles(off);
    // day 1: save the project, export by download, put the file in place -- and save the project again (the drawer asks)
    let project = { ...newProject(), routes: [ROUTE] };
    off.willPick({ name: 'skills_route.cpp', text: SKILLS });
    const day1 = await files.exportByDownload(ROUTE, project);
    assert.ok(day1.ok, day1.words);
    assert.match(day1.project.exports['src/team/skills_route.cpp#skills'], /^sha256:/, 'the hash is in the project it returns');
    const inPlace = off.downloads.at(-1).text;
    await files.saveProject(day1.project);
    const saved = off.downloads.at(-1).text;
    // day 2: open that saved project (another laptop), change the route, export by download again
    off.willPick({ name: 'route_maker.json', text: saved });
    project = (await files.openProject()).project;
    const r2 = addStep(ROUTE, { do: 'wait', seconds: 2 });
    off.willPick({ name: 'skills_route.cpp', text: inPlace });
    const day2 = await files.exportByDownload(r2, project);
    assert.ok(day2.ok, `the builder's own day-1 code is not a hand edit: ${day2.words}`);
    // a REAL hand edit in the picked file: refused, nothing downloaded; Overwrite replaces it
    const edited = off.downloads.at(-1).text.replace('wait 2 s', 'wait 2 s (by hand)');
    const before = off.downloads.length;
    off.willPick({ name: 'skills_route.cpp', text: edited });
    const refused = await files.exportByDownload(addStep(r2, { do: 'wait', seconds: 3 }), day2.project);
    assert.equal(refused.why, 'handEdit');
    assert.equal(off.downloads.length, before, 'nothing handed back');
    off.willPick({ name: 'skills_route.cpp', text: edited });
    const forced = await files.exportByDownload(addStep(r2, { do: 'wait', seconds: 3 }), day2.project, { overwrite: true });
    assert.ok(forced.ok);
    assert.ok(!off.downloads.at(-1).text.includes('(by hand)'));
});

test('Save never replaces a route_maker.json that changed in the folder since the page read it (unless the student says so)', async () => {
    const host = memoryHost({});
    const files = makeFiles(host);
    const mine = { ...newProject(), routes: [ROUTE] };
    // first save: no file yet (base null) -- written
    const first = await files.saveProject(mine, { base: null });
    assert.ok(first.ok);
    // a teammate's route arrives (git pull): the file is no longer what this page wrote
    const theirs = { ...newProject(), routes: [addStep(ROUTE, { do: 'wait', seconds: 9 })] };
    host.store['src/team/route_maker.json'] = (await import('../src/model/project.mjs')).saveProject(theirs);
    const pulled = host.store['src/team/route_maker.json'];
    const again = await files.saveProject({ ...mine, routes: [addStep(ROUTE, { do: 'wait', seconds: 4 })] }, { base: first.text });
    assert.equal(again.ok, false);
    assert.equal(again.why, 'changedOnDisk');
    assert.equal(again.diskText, pulled);
    assert.equal(host.store['src/team/route_maker.json'], pulled, 'the teammate\'s file is untouched');
    // the student chose "replace"
    assert.ok((await files.saveProject(mine, { base: first.text, force: true })).ok);
    // a file that has not changed since (base equal) is written without asking; one deleted since is nothing to lose
    const now = host.store['src/team/route_maker.json'];
    assert.ok((await files.saveProject(mine, { base: now })).ok);
    delete host.store['src/team/route_maker.json'];
    assert.ok((await files.saveProject(mine, { base: now })).ok);
    assert.equal(await files.readProjectFile(), host.store['src/team/route_maker.json']);
});

test('which robots a route is checked on: its own, and robot two once its build compiles the shared file', () => {
    const baked = { robots: { one: { id: 'one' }, two: { id: 'two' } }, sharedRoute: { present: false, robots: [] } };
    assert.deepEqual(routeRobots(baked, newRoute()).map((r) => r.id), ['one']);
    baked.sharedRoute = { present: true, robots: ['one', 'two'] };
    assert.deepEqual(routeRobots(baked, newRoute()).map((r) => r.id), ['one', 'two']);
});
