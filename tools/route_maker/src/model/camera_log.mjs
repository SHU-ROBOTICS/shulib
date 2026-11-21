// camera_log.mjs — WHAT THE CAMERA SEES (RMP1-N): shulib Drive's `[CAM]` lines (src/programs/drive_program.cpp,
// CameraTrace; never guessed) read into its latest picture. At boot: `AI Vision on port P: … on (RET); …` or `no AI Vision
// sensor on any port: …`, then `class ID: 'NAME'`; every 100 ms (1 s while it sees nothing) `n COUNT`, then per detection,
// up to 8: ` | tag ID c(CX,CY) WxH` (centre), ` | obj CLASS (X,Y) WxH S%`, ` | col ID (X,Y) WxH` (X,Y: the box's LEFT and
// TOP edges, include/pros/ai_vision.h), ` | type T id ID`. The picture is 320 x 240 (include/shulib/hal/vision_frames.hpp;
// A4 register HA-193, not yet measured). TermSink cuts a message at 200 bytes with '…' (diag/term_sink.hpp): a cut or
// garbled entry is skipped, never half-read, and `n` still says how many there were. (A line short of its count drops its
// last entry; bytes lost inside the LAST counted tag or colour cannot be told from a whole one: an object's '%' closes it.)
// Pure: no DOM.

export const IMAGE = { w: 320, h: 240 };
export const MAX_SHOWN = 8;  // the C++ prints 8 entries at most
export const STALE_S = 1.5;  // nothing seen: a line every 1 s, so 1.5 s without one is the robot gone quiet
export const GONE_S = 0.35;  // something seen: a line every 0.1 s, so 3 missed lines say it stopped seeing it
const MAX_COUNT = 24;  // AIVISION_MAX_OBJECT_COUNT: a bigger `n` is PROS_ERR, the camera not answering
const PROS_SUCCESS = 1;

const CAM = /^(?:\[t=[^\]]*\] )?(?:\[[A-Z]+\])?\[CAM\] (.*)$/;
const PORT = /^AI Vision on port (\d+): /;
const ON = / on \((-?\d+)\)/;
const NONE = /^no AI Vision sensor on any port\b/;
const CLASS = /^class (\d+): '(.*)'$/;
const COUNT = /^n (-?\d+)$/;
const ENTRY = [
    [/^tag (\d+) c\((-?\d+),(-?\d+)\) (\d+)x(\d+)$/, (m) => ({ kind: 'tag', id: +m[1], x: +m[2] - m[4] / 2, y: +m[3] - m[5] / 2, w: +m[4], h: +m[5] })],
    [/^obj (\d+) \((-?\d+),(-?\d+)\) (\d+)x(\d+) (\d+)%$/, (m) => ({ kind: 'obj', id: +m[1], x: +m[2], y: +m[3], w: +m[4], h: +m[5], score: +m[6] })],
    [/^col (\d+) \((-?\d+),(-?\d+)\) (\d+)x(\d+)$/, (m) => ({ kind: 'col', id: +m[1], x: +m[2], y: +m[3], w: +m[4], h: +m[5] })],
    [/^type (\d+) id (\d+)$/, (m) => ({ kind: 'other', id: +m[2], type: +m[1] })],
];

/** A fresh reader: no [CAM] line yet. `on`: the boot's switch-on result (1 = PROS_SUCCESS). */
export function newCamera() {
    return { lines: 0, port: null, on: null, none: false, names: {}, seen: null, at: null };
}

/** One entry, whole, or null. */
function entry(text) {
    for (const [re, make] of ENTRY) {
        const m = re.exec(text);
        if (m) {
            return make(m);
        }
    }
    return null;
}

/** Read one more line of the brain's output into `cam` (mutated, and returned) at `now` (ms); any other line changes nothing. */
export function readCamera(cam, raw, now) {
    const c = CAM.exec(String(raw).replace(/[\0\r]/g, '').replace(/\x1b\[[0-9;]*[A-Za-z]/g, '').trim());
    if (!c) {
        return cam;
    }
    const body = c[1];
    let m;
    cam.lines += 1;
    if ((m = PORT.exec(body)) || NONE.test(body)) {  // a boot: what came before is another program's
        const on = m && ON.exec(body);
        Object.assign(cam, newCamera(), { lines: cam.lines, port: m ? +m[1] : null, on: on ? +on[1] : null, none: !m });
    } else if ((m = CLASS.exec(body))) {
        cam.names[m[1]] = m[2];
    } else {
        const parts = body.split(' | ');
        const n = COUNT.exec(parts[0]);
        if (!n) {
            return cam;
        }
        const count = +n[1];
        const want = Math.min(Math.max(count, 0), MAX_SHOWN);
        let items = parts.slice(1);
        if (items.length > want) {
            return cam;  // more entries than it counted: garbled
        }
        const cut = body.endsWith('…') || items.length < want;
        if (items.length < want) {
            items = items.slice(0, -1);  // cut short: its last entry may be cut too
        }
        items = items.map(entry).filter(Boolean);
        cam.seen = { count, items, cut, error: count < 0 || count > MAX_COUNT };
        cam.at = now;
    }
    return cam;
}

/** One detection in words: "AprilTag 3", "Pin (87 %)", "colour 2". */
export function itemWords(it, names = {}) {
    return it.kind === 'tag' ? `AprilTag ${it.id}` : it.kind === 'obj' ? `${names[it.id] || `class ${it.id}`} (${it.score} %)`
        : it.kind === 'col' ? `colour ${it.id}` : `something else (type ${it.type})`;
}

/** The card's one sentence; `robot`: the robot's name (robot_names.mjs). Null before any [CAM] line. */
export function seesWords(cam, robot = 'this robot') {
    if (!cam.lines) {
        return null;
    }
    if (cam.none) {
        return `No camera on ${robot}: no AI Vision sensor on any port.`;
    }
    const s = cam.seen;
    if (cam.on !== null && cam.on !== PROS_SUCCESS && !(s && s.items.length)) {
        return `Camera on port ${cam.port} did not switch its detections on (${cam.on}).`;
    }
    if (!s) {
        return cam.port ? `Camera on port ${cam.port}: waiting for its first picture.` : 'Waiting for the camera\'s first picture.';
    }
    if (s.error) {
        return 'The camera is not answering.';
    }
    if (!s.items.length) {
        return s.count ? `Sees ${s.count} thing${s.count === 1 ? '' : 's'} (the line ${s.cut ? 'was cut' : 'could not be read'})` : 'Sees nothing';
    }
    const more = s.count - s.items.length;
    return `Sees: ${s.items.map((it) => itemWords(it, cam.names)).join(', ')}${more > 0 ? `, and ${more} more` : ''}`;
}

/** How much of a box lies inside the picture: 'all', 'part' or 'none'. */
export function inPicture(it) {
    const x0 = Math.max(it.x, 0);
    const y0 = Math.max(it.y, 0);
    const x1 = Math.min(it.x + it.w, IMAGE.w);
    const y1 = Math.min(it.y + it.h, IMAGE.h);
    return x1 <= x0 || y1 <= y0 ? 'none' : x1 - x0 === it.w && y1 - y0 === it.h ? 'all' : 'part';
}

/** A muted line when a box runs past the 320 x 240 picture (the size is assumed, HA-193); null otherwise. */
export function edgeWords(cam) {
    const s = cam.seen;
    return s && !s.error && s.items.some((it) => it.kind !== 'other' && inPicture(it) !== 'all')
        ? `A box runs past the ${IMAGE.w} x ${IMAGE.h} picture this view assumes.` : null;
}

/** Seconds a picture stays current: 0.35 s while it sees something (a line every 0.1 s), 1.5 s while it sees nothing. */
export const freshS = (cam) => (cam.seen && cam.seen.count > 0 ? GONE_S : STALE_S);

/** Seconds since the latest picture at `now` (ms); null before one. */
export const ageS = (cam, now) => (cam.at === null ? null : (now - cam.at) / 1000);
