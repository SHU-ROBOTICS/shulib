// library_words.test.mjs — the page shows the LIBRARY's words: each sentence must still be in its header,
// word for word (adjacent C++ string literals joined, escapes read), or this goes red until the page follows.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { LIBRARY_WORDS, tankNotSteeredWords } from '../src/model/library_words.mjs';
import { REPO } from './helpers.mjs';

/** Every string literal of a C++ file, adjacent ones (whitespace or comments between) joined. */
function literals(text) {
    const out = [];
    let cur = null;
    let i = 0;
    while (i < text.length) {
        const c = text[i];
        if (c === '/' && text[i + 1] === '/') {
            i = text.indexOf('\n', i);
            if (i < 0) break;
            continue;
        }
        if (c === '"' && text[i - 1] !== "'" ) {
            let s = '';
            ++i;
            while (i < text.length && text[i] !== '"') {
                if (text[i] === '\\') {
                    const n = text[i + 1];
                    s += n === 'n' ? '\n' : n === 't' ? '\t' : n;
                    i += 2;
                } else {
                    s += text[i++];
                }
            }
            ++i;
            cur = cur === null ? s : cur + s;
            continue;
        }
        if (!/\s/.test(c) && cur !== null) {
            out.push(cur);
            cur = null;
        }
        ++i;
    }
    if (cur !== null) out.push(cur);
    return out;
}

test('every library sentence the page quotes is still in its header, word for word', () => {
    for (const [key, { file, text, ident }] of Object.entries(LIBRARY_WORDS)) {
        if (ident) {
            continue;  // a build refusal: the next test
        }
        const lits = literals(fs.readFileSync(path.join(REPO, file), 'utf8'));
        assert.ok(lits.some((l) => l.includes(text)), `${key}: "${text}" is no longer in ${file} -- the library's words changed`);
    }
});

test('R9: a BUILD refusal is named by its function -- defined in its header, the twin fixture expecting exactly it', () => {
    const builds = Object.entries(LIBRARY_WORDS).filter(([, w]) => w.ident);
    assert.deepEqual(builds.map(([k]) => k).sort(), ['noPosition', 'noPreset']);
    for (const [key, { file, ident, twin, text }] of builds) {
        const header = fs.readFileSync(path.join(REPO, file), 'utf8');
        assert.ok(new RegExp(`\\binline void ${ident}\\(\\) \\{\\}`).test(header), `${key}: ${ident} is not defined in ${file}`);
        assert.ok(new RegExp(`detail::${ident}\\(\\);`).test(header), `${key}: nothing in ${file} calls ${ident}`);
        const fixture = fs.readFileSync(path.join(REPO, twin), 'utf8');
        assert.match(fixture, new RegExp(`^// expect: ${ident}$`, 'm'), `${key}: ${twin} is not the twin of ${ident}`);
        assert.ok(text.endsWith(ident), `${key}: the page must show the name the compiler prints`);
    }
});

test('RC24: the library BUILDS its sentence from three literals; the page joins the same three around its numbers', () => {
    const lits = literals(fs.readFileSync(path.join(REPO, 'include/shulib/motion/move_to_pose.hpp'), 'utf8'));
    const pieces = ['tankNotSteered1', 'tankNotSteered2', 'tankNotSteered3'].map((k) => LIBRARY_WORDS[k].text);
    // each piece is ONE whole literal of the header (not a substring of a longer one): the joins are where the numbers go
    for (const p of pieces) {
        assert.ok(lits.includes(p), `"${p}" is not one literal of move_to_pose.hpp`);
    }
    assert.equal(tankNotSteeredWords(16, 14.999999999999998),
                 'tank moveTo not steered: it starts 16.0 deg off its target heading (steers within 15.0 deg) -- turn first: face()');
});

test('the reader is not fooled: a sentence split across a literal boundary is found, one with a word changed is not', () => {
    const lits = literals('f("abc -- "\n  "def");\ng("other");');
    assert.ok(lits.includes('abc -- def'));
    assert.ok(!lits.some((l) => l.includes('abc -- deg')));
});
