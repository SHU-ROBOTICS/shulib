// camera_log.test.mjs — RMP1-N: shulib Drive's [CAM] lines read into the camera's latest picture (camera_log.mjs). Most
// lines are built from the C++ (FMT, through TermSink's framing and 200-byte cut); the first test pins each CameraTrace
// statement — format AND arguments — and TermSink's framing, so a change to what a number means turns it red. One test
// reads real lines: robot one's camera on 2026-11-20 (the coordinator's robot_logs/robot_one_2026-11-20.log).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { newCamera, readCamera, seesWords, edgeWords, inPicture, freshS, ageS, IMAGE, MAX_SHOWN, STALE_S, GONE_S } from '../src/model/camera_log.mjs';

const REPO = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..', '..');
const read = (...p) => fs.readFileSync(path.join(REPO, ...p), 'utf8').replace(/\s+/g, ' ');
const FMT = {
    none: 'no AI Vision sensor on any port: nothing to trace',
    port: 'AI Vision on port %d: tags, objects and colours on (%ld); a line every 100 ms: '
        + 'tag id c(x,y) WxH | obj class (x,y) WxH score%% | col id (x,y) WxH',
    name: "class %ld: '%.20s'",
    n: 'n %ld',
    tag: ' | tag %u c(%d,%d) %dx%d',
    obj: ' | obj %u (%d,%d) %dx%d %d%%',
    col: ' | col %u (%d,%d) %dx%d',
    other: ' | type %u id %u',
};
/** drive_program.cpp's CameraTrace, statement by statement (whitespace folded): what each printed number IS. */
const DRIVE = [
    `log_.log(hal::LogLevel::Info, "CAM", "${FMT.none}");`,
    `std::snprintf(line, sizeof line, "${FMT.port.slice(0, FMT.port.indexOf('tag id'))}" "${FMT.port.slice(FMT.port.indexOf('tag id'))}", port_, static_cast<long>(ret));`,
    'for (std::int32_t id = 0; id < 8; ++id) {',
    'if (pros::c::aivision_get_class_name(p, id, name) == 1 && name[0] != 0) {',
    `std::snprintf(line, sizeof line, "${FMT.name}", static_cast<long>(id), reinterpret_cast<const char*>(name));`,
    'if (port_ == 0 || ++tick_ % 10 != 0) {',  // a line every 10th 10 ms tick: 0.1 s (GONE_S)
    'if (n <= 0 && tick_ % 100 != 0) {',  // seeing nothing: every 100th, 1 s (STALE_S)
    `int k = std::snprintf(line, sizeof line, "${FMT.n}", static_cast<long>(n));`,
    `n = n < 0 ? 0 : n > ${MAX_SHOWN} ? ${MAX_SHOWN} : n;`,
    'const unsigned id = o.id;',
    'const unsigned type = o.type;',
    'const int xs[4] = {o.object.tag.x0, o.object.tag.x1, o.object.tag.x2, o.object.tag.x3};',
    'const int ys[4] = {o.object.tag.y0, o.object.tag.y1, o.object.tag.y2, o.object.tag.y3};',
    'if (type == pros::E_AIVISION_DETECTED_TAG) {',
    'int x0 = xs[0], x1 = xs[0], y0 = ys[0], y1 = ys[0]; for (int c = 1; c < 4; ++c) {',
    'x0 = std::min(x0, xs[c]); x1 = std::max(x1, xs[c]); y0 = std::min(y0, ys[c]); y1 = std::max(y1, ys[c]);',
    `k += std::snprintf(line + k, room, "${FMT.tag}", id, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0);`,
    '} else if (type == pros::E_AIVISION_DETECTED_OBJECT) {',
    'const int x = o.object.element.xoffset, y = o.object.element.yoffset;',
    'const int w = o.object.element.width, h = o.object.element.height, sc = o.object.element.score;',
    `k += std::snprintf(line + k, room, "${FMT.obj}", id, x, y, w, h, sc);`,
    '} else if (type == pros::E_AIVISION_DETECTED_COLOR || type == pros::E_AIVISION_DETECTED_CODE) {',
    'const int x = o.object.color.xoffset, y = o.object.color.yoffset;',
    'const int w = o.object.color.width, h = o.object.color.height;',
    `k += std::snprintf(line + k, room, "${FMT.col}", id, x, y, w, h);`,
    `k += std::snprintf(line + k, room, "${FMT.other}", type, id);`,
    'log_.log(hal::LogLevel::Info, "CAM", line);',
];
/** TermSink.log's framing: `[t=%7.2f] `, no level tag at Info, `[TAG] `, the message cut at 200 bytes with '…'. */
const SINK = [
    ['include/shulib/diag/term_sink.hpp', 'appendTimestamp(line, clock_.now().value()); line.appendLiteral(levelTag(level)); line.appendLiteral("["); '
        + 'line.appendSanitized(subsystem, kMaxTagBytes); line.appendLiteral("] "); line.appendSanitized(message, kMaxMessageBytes);'],
    ['include/shulib/diag/term_sink.hpp', 'case hal::LogLevel::Info: return "";'],
    ['include/shulib/diag/term_sink.hpp', 'kMaxMessageBytes = 200;'],
    ['include/shulib/diag/line_format.hpp', 'line.appendLiteral("[t="); appendNum(line, tSeconds, 7, 2); line.appendLiteral("] ");'],
    ['include/shulib/diag/line_format.hpp', 'appendLiteral("…");'],
];
/** C's snprintf for the conversions FMT uses. */
const printf = (fmt, ...a) => fmt.replace(/%(%|\.20s|ld|d|u)/g, (_, c) => (c === '%' ? '%' : c === '.20s' ? String(a.shift()).slice(0, 20) : String(a.shift())));
/** TermSink.log at Info: `[t=%7.2f] [CAM] ` and the message cut at 200 bytes with '…'. */
const sink = (t, msg) => `[t=${t.toFixed(2).padStart(7)}] [CAM] ${msg.length > 200 ? `${msg.slice(0, 200)}…` : msg}`;
const detections = (n, ...entries) => printf(FMT.n, n) + entries.map(([k, ...a]) => printf(FMT[k], ...a)).join('');
const BOOT = [sink(0.4, printf(FMT.port, 7, 1)), sink(0.4, printf(FMT.name, 0, 'redBluePin')), sink(0.4, printf(FMT.name, 4, 'cup'))];
const feed = (lines, cam = newCamera(), at = 1000) => lines.reduce((c, l) => readCamera(c, l, at), cam);

test('the C++ still prints what the parser reads: each CameraTrace statement, its arguments, TermSink\'s framing', () => {
    const cpp = read('src', 'programs', 'drive_program.cpp');
    DRIVE.forEach((st) => assert.ok(cpp.includes(st), `drive_program.cpp no longer says ${st}: read the change into camera_log.mjs`));
    SINK.forEach(([file, st]) => assert.ok(read(file).includes(st), `${file} no longer says ${st}`));
    assert.match(read('include', 'shulib', 'hal', 'vision_frames.hpp'), /kVisionImageWidthPx = 320;.*kVisionImageHeightPx = 240;/);
    assert.deepEqual(IMAGE, { w: 320, h: 240 });
    assert.ok(STALE_S > 100 * 0.01 && STALE_S < 2 * 100 * 0.01, 'nothing seen: grey after one missed 1 s line, not two');
    assert.ok(GONE_S > 3 * 10 * 0.01 && GONE_S < 4 * 10 * 0.01, 'something seen: grey after three missed 0.1 s lines');
});

test('boot, class names, then one tag, one object, one colour: tags by centre, the rest by their left/top corner', () => {
    const cam = feed([...BOOT, sink(1.0, detections(3, ['tag', 3, 160, 120, 40, 30], ['obj', 0, 40, 150, 60, 50, 87], ['col', 2, 250, 10, 30, 20]))]);
    assert.equal(cam.port, 7);
    assert.equal(cam.on, 1);
    assert.deepEqual(cam.names, { 0: 'redBluePin', 4: 'cup' });
    assert.deepEqual(cam.seen.items, [
        { kind: 'tag', id: 3, x: 140, y: 105, w: 40, h: 30 },
        { kind: 'obj', id: 0, x: 40, y: 150, w: 60, h: 50, score: 87 },
        { kind: 'col', id: 2, x: 250, y: 10, w: 30, h: 20 }]);
    assert.equal(seesWords(cam), 'Sees: AprilTag 3, redBluePin (87 %), colour 2');
    assert.equal(edgeWords(cam), null);
    assert.equal(cam.at, 1000);
    assert.equal(ageS(cam, 3500), 2.5);
    assert.equal(freshS(cam), GONE_S);
});

test('0 and 1 detections; an object without a class name says its id', () => {
    const none = feed([...BOOT, sink(1, detections(0))]);
    assert.equal(seesWords(none), 'Sees nothing');
    assert.deepEqual(none.seen, { count: 0, items: [], cut: false, error: false });
    assert.equal(freshS(none), STALE_S);
    const one = feed([sink(1, printf(FMT.port, 7, 1)), sink(1.1, detections(1, ['obj', 3, 0, 0, 10, 10, 41]))]);
    assert.equal(seesWords(one), 'Sees: class 3 (41 %)');
});

test('8 detections: all 8 when the line fits; cut at 200 bytes, the cut one skipped (never half-read), the rest counted', () => {
    const tags = Array.from({ length: 8 }, (_, k) => ['tag', k, 9, 9, 9, 9]);
    const fits = feed([sink(1, detections(8, ...tags))]);
    assert.equal(fits.seen.items.length, 8);
    assert.equal(fits.seen.cut, false);
    const objs = Array.from({ length: 8 }, (_, k) => ['obj', 4, 100 + k, 200 + k, 33, 44, 90 + k]);
    const line = sink(1, detections(8, ...objs));
    assert.ok(line.endsWith('…'), 'TermSink cut it');
    const cut = feed([...BOOT, line]);
    const whole = (line.match(/ \| /g) || []).length - 1;  // the entries before the cut one
    assert.equal(cut.seen.items.length, whole);
    assert.ok(whole >= 5 && whole < 8, `${whole}`);
    cut.seen.items.forEach((it, k) => assert.deepEqual(it, { kind: 'obj', id: 4, x: 100 + k, y: 200 + k, w: 33, h: 44, score: 90 + k }));
    assert.match(seesWords(cut), new RegExp(`^Sees: (cup \\(\\d+ %\\), ){${whole - 1}}cup \\(\\d+ %\\), and ${8 - whole} more$`));
    const twelve = feed([sink(1, detections(12, ...tags))]);
    assert.equal(twelve.seen.items.length, 8, 'the C++ prints 8 at most');
    assert.match(seesWords(twelve), /, and 4 more$/);
});

test('cut without TermSink\'s mark (bytes lost): the last entry is not trusted; garbled: skipped, or the line refused', () => {
    const lost = feed([sink(1, detections(3, ['tag', 3, 160, 120, 40, 40], ['tag', 4, 100, 100, 40, 40], ['tag', 5, 1, 1, 1, 1]))
        .replace(/0 \| tag 5.*$/, '')]);  // "… | tag 4 c(100,100) 40x4": fewer entries than counted
    assert.deepEqual(lost.seen.items.map((x) => x.id), [3], 'never "tag 4 ... 40x4"');
    assert.equal(seesWords(lost), 'Sees: AprilTag 3, and 2 more');
    const pct = feed([sink(1, detections(1, ['obj', 0, 40, 150, 60, 50, 87]).slice(0, -2))]);  // an object's '%' closes it
    assert.deepEqual(pct.seen.items, []);
    assert.equal(seesWords(pct), 'Sees 1 thing (the line could not be read)', 'whole but unreadable: never "cut"');
    const mid = feed([sink(1, `${printf(FMT.n, 3)}${printf(FMT.tag, 3, 1, 1, 2, 2)} | t#g 4 c(1,1) 2x2${printf(FMT.col, 1, 0, 0, 5, 5)}`)]);
    assert.deepEqual(mid.seen.items.map((x) => x.kind), ['tag', 'col']);
    assert.equal(seesWords(mid), 'Sees: AprilTag 3, colour 1, and 1 more');
    const before = feed([sink(1, detections(1, ['tag', 7, 5, 5, 4, 4]))]);
    const extra = feed([sink(2, detections(1, ['tag', 1, 1, 1, 1, 1], ['tag', 2, 1, 1, 1, 1]))], before, 2000);
    assert.deepEqual(extra.seen.items.map((x) => x.id), [7], 'more entries than counted: the line refused');
    assert.equal(extra.at, 1000);
    const noCount = feed([sink(2, `n x${printf(FMT.tag, 1, 1, 1, 1, 1)}`)], before, 2000);
    assert.equal(noCount.at, 1000);
    assert.equal(seesWords(feed([sink(1, printf(FMT.n, 2))])), 'Sees 2 things (the line was cut)');
    assert.equal(seesWords(feed([sink(1, `${printf(FMT.n, 1)}${printf(FMT.tag, 3, 1, 1, 2, 2).slice(0, -3)}…`)])), 'Sees 1 thing (the line was cut)');
    assert.equal(seesWords(feed([sink(1, `${printf(FMT.n, 2)} | tag 1 c(1,1) 1x1 r0 | tag 2 c(2,2) 2x2 r0`)])), 'Sees 2 things (the line could not be read)');
});

test('other lines change nothing: other tags, the boot line\'s own sample, [CAM] inside another message, a cut tag', () => {
    const cam = feed(BOOT);
    for (const l of ['[t=   1.00] [CAS] R1 0 R2 0 pos 0.00 tgt 0.00 homed 1', '[DRV] booted: shulib DRIVE build [shulib-robot-variant=one-drive]',
                     `[t=   1.00] [SES] note [CAM] ${detections(1, ['tag', 1, 1, 1, 1, 1])}`, `[t=   1.00] [CA${detections(1, ['tag', 1, 1, 1, 1, 1])}`,
                     'tag 3 c(160,120) 40x40', '']) {
        readCamera(cam, l, 5000);
    }
    assert.equal(cam.lines, 3);
    assert.equal(cam.seen, null, 'the boot line\'s "tag id c(x,y) WxH | obj class ..." is never a picture');
    assert.equal(seesWords(cam), 'Camera on port 7: waiting for its first picture.');
    assert.equal(seesWords(newCamera()), null, 'no [CAM] line: no card');
});

test('interleaved with other tags, a pros terminal\'s \\r, NULs and colour codes, a level tag, no time stamp', () => {
    const lines = ['[t=   0.90] [CAS] R1 1 R2 0', `\0\x1b[0m${sink(1, detections(1, ['tag', 5, 20, 30, 10, 10]))}\r`, '[t=   1.05] [CAS] R1 1 R2 0',
                   `[t=   1.10] [WARN]${sink(1.1, detections(1, ['col', 1, 2, 3, 4, 5])).slice(12)}`];
    const cam = feed(lines);
    assert.deepEqual(cam.seen.items, [{ kind: 'col', id: 1, x: 2, y: 3, w: 4, h: 5 }]);
    assert.equal(feed([`[CAM] ${detections(1, ['tag', 5, 20, 30, 10, 10])}`]).seen.items[0].id, 5);
});

test('no sensor; detections not switched on; the camera not answering (PROS_ERR, 25, -1, not 24); a new boot forgets', () => {
    const none = feed([sink(0.4, FMT.none)]);
    assert.equal(none.none, true);
    assert.equal(seesWords(none, 'White Bot'), 'No camera on White Bot: no AI Vision sensor on any port.');  // the robot's name, as the card passes it (RMP1-L)
    const off = feed([sink(0.3, printf(FMT.port, 7, 2147483647)), sink(1.3, detections(0))]);
    assert.equal(seesWords(off), 'Camera on port 7 did not switch its detections on (2147483647).');
    assert.equal(seesWords(feed([sink(0.3, printf(FMT.port, 7, 2147483647))])), 'Camera on port 7 did not switch its detections on (2147483647).');
    assert.equal(seesWords(feed([sink(1.4, detections(1, ['tag', 3, 9, 9, 4, 4]))], off)), 'Sees: AprilTag 3', 'what it sees is the evidence');
    for (const [count, error] of [[2147483647, true], [25, true], [-1, true], [24, false], [0, false]]) {
        const cam = feed([...BOOT, sink(1, detections(count, ...Array(Math.min(Math.max(count, 0), 8)).fill(['other', 0, 0])))]);
        assert.equal(cam.seen.error, error, `n ${count}`);
        assert.equal(seesWords(cam) === 'The camera is not answering.', error, `n ${count}`);
    }
    const again = feed([sink(9, printf(FMT.port, 3, 1))], feed([...BOOT, sink(1, detections(1, ['obj', 0, 1, 1, 1, 1, 50]))]));
    assert.deepEqual([again.port, again.names, again.seen, again.none, again.lines], [3, {}, null, false, 5]);
    assert.equal(feed([sink(1, FMT.none)], again).port, null);
    assert.equal(seesWords(feed([sink(1, detections(1, ['other', 16, 2]))])), 'Sees: something else (type 16)');
});

test('a box past the 320 x 240 picture: drawn as far as it is inside, and said once; a tag centre may be negative', () => {
    const at = (x, y, w, h) => inPicture({ x, y, w, h });
    assert.deepEqual([at(0, 0, 320, 240), at(-1, 0, 10, 10), at(315, 230, 10, 10), at(320, 0, 5, 5), at(0, 240, 5, 5), at(-20, 5, 20, 5)],
        ['all', 'part', 'part', 'none', 'none', 'none']);
    const left = feed([sink(1, detections(1, ['tag', 9, -5, 10, 20, 8]))]);
    assert.deepEqual(left.seen.items, [{ kind: 'tag', id: 9, x: -15, y: 6, w: 20, h: 8 }]);
    assert.equal(edgeWords(left), 'A box runs past the 320 x 240 picture this view assumes.');
    assert.equal(edgeWords(feed([sink(1, detections(1, ['obj', 0, 400, 100, 50, 50, 80]))])), 'A box runs past the 320 x 240 picture this view assumes.');
    assert.equal(edgeWords(feed([sink(1, detections(1, ['other', 16, 2]))])), null);
});

test('real lines: robot one\'s camera, 2026-11-20 (port 7, no class names printed), read whole, none refused', () => {
    const LOG = [  // the coordinator's robot_logs/robot_one_2026-11-20.log, each line as captured (\r and all), by its line number
        '[t=   0.30] [CAM] AI Vision on port 7: tags, objects and colours on (1); a line every 100 ms: tag id c(x,y) WxH | obj class (x,y) WxH score% | col id (x,y) WxH\r',  // :106485
        '[t=   1.30] [CAM] n 0\r',  // :106608
        '[t=  88.50] [CAM] n 1 | tag 2 c(189,143) 15x14\r',  // :144282
        '[t=  62.10] [CAM] n 1 | obj 0 (289,112) 31x49 71%\r',  // :184658 (its right edge at x 320)
        '[t= 147.00] [CAM] n 1 | obj 0 (0,0) 21x12 62%\r',  // :151412
        '[t=  47.00] [CAM] n 2 | tag 2 c(168,143) 12x13 | tag 3 c(297,202) 124x21\r',  // :138995 (tag 3 reaches x 359)
        '[t=  79.90] [CAM] n 2 | tag 2 c(175,142) 14x13 | tag 15 c(213,237) 36x13\r',  // :143204 (tag 15 reaches y 243.5)
    ];
    const seen = [];
    LOG.reduce((c, l) => { readCamera(c, l, 0); seen.push([seesWords(c), edgeWords(c)]); return c; }, newCamera());
    assert.deepEqual(seen, [
        ['Camera on port 7: waiting for its first picture.', null], ['Sees nothing', null], ['Sees: AprilTag 2', null],
        ['Sees: class 0 (71 %)', null], ['Sees: class 0 (62 %)', null],
        ['Sees: AprilTag 2, AprilTag 3', 'A box runs past the 320 x 240 picture this view assumes.'],
        ['Sees: AprilTag 2, AprilTag 15', 'A box runs past the 320 x 240 picture this view assumes.']]);
});
