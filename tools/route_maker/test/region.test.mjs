// region.test.mjs — FA1–FA4: writing into a team file (A.3.7, A.6 [W12]; §6.3; landmines 9, 19), against
// region.mjs DIRECTLY (host.test.mjs runs the same rules through the in-memory host). The CRLF file is BUILT
// here from the LF fixture (W11: no CRLF file is committed).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { findRegion, spliceRegion, regionText, regionHash, sha256Hex, isTeamPath, hasTeamRoutesLine, lineEnding,
         teamRoutesLine } from '../src/model/region.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { newRoute, setStart, addStep } from '../src/model/route.mjs';
import { GOLDENS } from './helpers.mjs';

const EMPTY = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
/** A stand-in for §6.3's skills-route file: the team's lines around the starter region (LF). */
const FILE = `// skills_route.cpp -- the team's file\n#include "team/skills_route.hpp"\n\nnamespace shulib::bench {\nusing namespace units::literals;\n\n${EMPTY}\n}  // namespace shulib::bench\n`;
const ROUTE = addStep(setStart(newRoute(), { x: -36, y: -55.7, headingDeg: 0 }), { do: 'wait', seconds: 1 });
const crlf = (t) => t.replace(/\n/g, '\r\n');
const outside = (text) => {
    const f = findRegion(text);
    return [text.slice(0, f.start), text.slice(f.end)];
};

test('the stand-in file is the CONTRACT: its starter region is the empty export', () => {
    assert.ok(EMPTY.includes('NO STEPS YET'));
    const f = findRegion(FILE);
    assert.ok(f.ok);
    assert.equal(regionText(FILE, f), exportRegion(newRoute()));
});

test('FA1: only between existing markers, only under src/team/', () => {
    for (const p of ['src/team/skills_route.cpp', 'src/team/route_maker.json']) {
        assert.ok(isTeamPath(p), p);
    }
    for (const p of ['src/team/../main.cpp', '/src/team/x.cpp', 'src/robots/mechanism_table.hpp', 'src/team', 'src/team/',
                     'C:/src/team/x.cpp', 'src\\team\\x.cpp', 'src/team/./x.cpp', '', 'src/teams/x.cpp']) {
        assert.ok(!isTeamPath(p), `${p} must be refused`);
    }
    assert.equal(spliceRegion('int x;\n', exportRegion(ROUTE)).why, 'noMarkers');
    assert.equal(spliceRegion(FILE + EMPTY, exportRegion(ROUTE)).why, 'markerCount', 'two regions: refused');
    const swapped = FILE.replace('— BEGIN ═', '— TMP ═').replace('— END ═', '— BEGIN ═').replace('— TMP ═', '— END ═');
    assert.equal(spliceRegion(swapped, exportRegion(ROUTE)).why, 'markerOrder');
    const indented = FILE.replace('// ═══ ROUTE MAKER — route "skills" — BEGIN', '  // ═══ ROUTE MAKER — route "skills" — BEGIN');
    assert.equal(spliceRegion(indented, exportRegion(ROUTE)).why, 'markerCount', 'a marker must be alone on its line');
    // a BEGIN and an END that name different routes are not one region
    const mismatched = FILE.replace('route "skills" — END', 'route "match" — END');
    assert.equal(findRegion(mismatched).why, 'markerOrder');
    assert.equal(spliceRegion(mismatched, exportRegion(ROUTE)).why, 'markerOrder');
});

test('FA2: a hand edit inside the region is refused unless "overwrite"; the builder\'s own last write is not', () => {
    const first = spliceRegion(FILE, exportRegion(ROUTE));
    assert.ok(first.ok, 'the empty starter is the builder\'s own: no hash needed for the first export');
    const edited = first.text.replace('wait 1 s', 'wait 1 s -- tuned by hand');
    const refused = spliceRegion(edited, exportRegion(newRoute()), { lastHash: first.hash });
    assert.equal(refused.why, 'handEdit');
    assert.ok(spliceRegion(edited, exportRegion(newRoute()), { lastHash: first.hash, overwrite: true }).ok);
    // the same file re-exported with the hash the project kept: fine
    assert.ok(spliceRegion(first.text, exportRegion(newRoute()), { lastHash: first.hash }).ok);
    // without the project's hash a changed region is a hand edit (never assume)
    assert.equal(spliceRegion(first.text, exportRegion(newRoute())).why, 'handEdit');
    // a region equal to what is about to be written needs no hash
    assert.ok(spliceRegion(first.text, exportRegion(ROUTE)).ok);
});

test('FA3: NOTHING outside the region changes; the file\'s CRLF and its final newline are kept', () => {
    for (const [label, text] of [['LF', FILE], ['CRLF', crlf(FILE)], ['LF, no final newline', FILE.trimEnd()],
                                 ['CRLF, no final newline', crlf(FILE).trimEnd()]]) {
        const out = spliceRegion(text, exportRegion(ROUTE));
        assert.ok(out.ok, label);
        assert.deepEqual(outside(out.text), outside(text), `${label}: a byte outside the region moved`);
        assert.equal(lineEnding(out.text), lineEnding(text), `${label}: line ending`);
        if (lineEnding(text) === '\r\n') {
            assert.ok(!/[^\r]\n/.test(out.text), `${label}: a bare LF was written into a CRLF file (RM30)`);
        } else {
            assert.ok(!out.text.includes('\r'), label);
        }
        assert.equal(out.text.endsWith('\n'), text.endsWith('\n'), `${label}: final newline`);
        // and the hash does not see the line ending: a CRLF checkout is not a hand edit
        assert.equal(out.hash, regionHash(exportRegion(ROUTE)));
    }
    const back = spliceRegion(crlf(FILE), exportRegion(ROUTE));
    assert.ok(spliceRegion(back.text, exportRegion(newRoute()), { lastHash: back.hash }).ok, 'CRLF round trip');
});

test('FA4: each autons file\'s line that registers the team\'s routes found / missing / commented out / the other robot\'s', () => {
    const file = (body) => `void registerAutons(competition::AutonRegistry& registry) {\n${body}\n}\n`;
    for (const robot of ['One', 'Two']) {  // RMP1-C D2: each autons file registers the team's routes with ONE line
        const other = robot === 'One' ? 'Two' : 'One';
        assert.ok(hasTeamRoutesLine(file(`    ${teamRoutesLine(robot)}`), robot));
        assert.ok(hasTeamRoutesLine(file(`    addTeamRoutes( registry ,\n        TeamRobot::${robot} ) ;`), robot));
        assert.ok(!hasTeamRoutesLine(file(`    // ${teamRoutesLine(robot)}`), robot), 'a commented-out line does not register anything');
        assert.ok(!hasTeamRoutesLine(file(`    /* ${teamRoutesLine(robot)} */`), robot));
        assert.ok(!hasTeamRoutesLine(file(`    ${teamRoutesLine(other)}`), robot), `robot ${other}'s line is not robot ${robot}'s`);
        assert.ok(!hasTeamRoutesLine(file('    registry.add(skillsRoute());'), robot), 'the old one-route line');
        assert.ok(!hasTeamRoutesLine(file(''), robot));
    }
});

test('SHA-256 is the standard one (FIPS 180-4 vectors), so every host hashes alike', () => {
    assert.equal(sha256Hex(''), 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855');
    assert.equal(sha256Hex('abc'), 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
    assert.equal(sha256Hex('abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq'),
                 '248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1');
    assert.equal(sha256Hex('═'), sha256Hex('\u2550'), 'UTF-8');
    assert.equal(sha256Hex('a'.repeat(1000)).length, 64);
});
