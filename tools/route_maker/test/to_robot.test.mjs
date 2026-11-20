// to_robot.test.mjs — RMP1-F: a write is READ BACK (region.mjs's readBack, through files.mjs over the in-memory host,
// with writes the test makes go wrong), what the page says was exported, and the commands that build and upload.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { makeFiles, memoryHost } from '../src/host/files.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { newProject } from '../src/model/project.mjs';
import { newRoute, setStart, addStep } from '../src/model/route.mjs';
import { readBack } from '../src/model/region.mjs';
import { robotCommands, exportSummary, TESTER } from '../src/model/to_robot.mjs';
import { GOLDENS, REPO } from './helpers.mjs';

const EMPTY = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
const SKILLS = `#include "team/skills_route.hpp"\r\n\r\nnamespace shulib::bench {\r\n\r\n${EMPTY.replace(/\n/g, '\r\n')}\r\n}  // namespace shulib::bench\r\n`;
const ROUTE = addStep(setStart(newRoute(), { x: -36, y: -55.7, headingDeg: 0 }), { do: 'wait', seconds: 1 });
const FILE = 'src/team/skills_route.cpp';

test('readBack: exactly what was written passes; a gone file, lost markers, a changed region or a changed outside do not', () => {
    const region = exportRegion(ROUTE);
    const wrote = SKILLS.replace(EMPTY.replace(/\n/g, '\r\n'), region.replace(/\n/g, '\r\n'));
    assert.notEqual(wrote, SKILLS);
    assert.deepEqual(readBack(wrote, region, wrote), { ok: true }, 'CRLF on disk is the same region');
    assert.deepEqual(readBack(wrote, region, undefined), { ok: true }, 'nothing written: the region alone decides');
    assert.equal(readBack(null, region, wrote).why, 'gone');
    assert.equal(readBack('int x;\n', region, wrote).why, 'differs', 'markers lost in the write: the page wrote them');
    assert.equal(readBack(wrote + wrote, region, wrote).why, 'differs', 'two pairs read back');
    assert.equal(readBack(wrote.replace('wait 1 s', 'wait 2 s'), region, wrote).why, 'differs', 'one character in the region');
    assert.equal(readBack(wrote.replace('#include', '// #include'), region, wrote).why, 'differs', 'outside the region');
    assert.equal(readBack(SKILLS, region, undefined).why, 'differs', 'the old region read back');
    assert.match(readBack(SKILLS, region, undefined).words, /not what the page wrote/);
});

test('exportToFile READS BACK: a write the folder changes is refused by name, the project not marked exported', async () => {
    const host = memoryHost({ [FILE]: SKILLS });
    host.writeText = async (p, t) => {
        host.store[p] = t.replace('wait 1 s', 'wait 9 s');  // something changed the file as it was written
    };
    const out = await makeFiles(host).exportToFile(ROUTE, newProject());
    assert.equal(out.ok, false);
    assert.equal(out.why, 'differs');
    assert.equal(out.project, undefined, 'no export hash from a write that did not land');
});

test('exportToFile READS BACK the WHOLE file: a write changed only OUTSIDE the markers is refused too', async () => {
    const host = memoryHost({ [FILE]: SKILLS });
    host.writeText = async (p, t) => {
        host.store[p] = t.replace('#include', '// #include');  // the plumbing changed as it landed; the region is right
    };
    const out = await makeFiles(host).exportToFile(ROUTE, newProject());
    assert.equal(out.why, 'differs');
    assert.equal(out.project, undefined);
});

test('exportByDownload: a refusal names the file picked (the wrong file, said as one)', async () => {
    const host = memoryHost({});
    host.willPick({ name: 'autons.cpp', text: 'void registerAutons() {}\n' });
    const out = await makeFiles(host).exportByDownload(ROUTE, newProject());
    assert.equal(out.why, 'noMarkers');
    assert.equal(out.picked, 'autons.cpp');
    assert.deepEqual(host.downloads, [], 'nothing handed back');
});

test('exportToFile READS BACK: a write that drops the file, or one the folder refuses, is a refusal, never a throw', async () => {
    const gone = memoryHost({ [FILE]: SKILLS });
    gone.writeText = async (p) => {
        delete gone.store[p];
    };
    assert.equal((await makeFiles(gone).exportToFile(ROUTE, newProject())).why, 'gone');
    const refused = memoryHost({ [FILE]: SKILLS });
    refused.writeText = async () => {
        throw new DOMException('The request is not allowed by the user agent', 'NotAllowedError');
    };
    const out = await makeFiles(refused).exportToFile(ROUTE, newProject());
    assert.equal(out.why, 'failed');
    assert.match(out.words, /^The folder refused: The request is not allowed/);
    assert.equal(refused.store[FILE], SKILLS);
});

test('exportToFile: a good write is read back, and an unchanged file is checked too', async () => {
    const host = memoryHost({ [FILE]: SKILLS });
    let reads = 0;
    const read = host.readText;
    host.readText = async (p) => {
        reads += 1;
        return read(p);
    };
    const files = makeFiles(host);
    const first = await files.exportToFile(ROUTE, newProject());
    assert.ok(first.ok && first.changed);
    assert.equal(reads, 2, 'read, write, read back');
    const again = await files.exportToFile(ROUTE, first.project);
    assert.ok(again.ok && !again.changed);
    assert.equal(reads, 4, 'nothing written: still read back');
});

test('what was exported: the routes\' names and every step counted', () => {
    const two = { ...addStep(ROUTE, { do: 'wait', seconds: 2 }), name: 'left 1' };
    assert.deepEqual(exportSummary([ROUTE, two]), { names: ['skills', 'left 1'], steps: 3, words: '2 routes (skills, left 1), 3 steps' });
    assert.equal(exportSummary([newRoute()]).words, '1 route (skills), 0 steps');
});

test('the commands: robot one is ROBOT=one, robot two ROBOT=tank; the tester is slot 6 "shulib Auton Tester", as the Makefile says', () => {
    assert.deepEqual(robotCommands('one'), {
        build: 'make ROBOT=one PROGRAM=autontest',
        upload: 'pros upload --slot 6 --name "shulib Auton Tester" --description "$(git describe --always --dirty)"',
        helper: 'python3 tools/upload_auton.py one',
    });
    assert.deepEqual(robotCommands('two', { windows: true }), {
        build: 'make ROBOT=tank PROGRAM=autontest',
        upload: 'pros upload --slot 6 --name "shulib Auton Tester" --description "$(git describe --always --dirty)"',
        helper: 'python tools/upload_auton.py two',
    });
    const make = fs.readFileSync(path.join(REPO, 'Makefile'), 'utf8');
    assert.match(make, /make ROBOT=tank PROGRAM=autontest → "shulib Auton Tester"/);
    assert.match(make, /slot 6 since\s*\n#\s+2026-11-20/);
    const script = fs.readFileSync(path.join(REPO, 'tools', 'upload_auton.py'), 'utf8');
    assert.ok(script.includes(`SLOT = ${TESTER.slot}`) && script.includes(`NAME = "${TESTER.name}"`), 'the script uploads where the page says');
});
