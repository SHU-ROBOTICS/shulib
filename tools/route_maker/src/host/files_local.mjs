// files_local.mjs — the LOCAL host (RMP1-I D3, D4): the page that tools/route_builder.py serves on this laptop reads and
// writes src/team/ through it (any browser, no folder dialog), and Send to robot runs tools/upload_auton.py there.
// Every rule stays region.mjs's through files.mjs; the helper also refuses any path that is not a file under src/team/.
// A refusal is thrown as an Error in the helper's words, as the folder host's are; a missing file (204) reads as null.

import { downloadHost } from './files_download.mjs';

export const GONE = 'The route builder on this laptop has stopped: start it again (VS Code: Terminal → Run Task → Open the route builder).';

/** {token, folder} when the helper served this page (its meta; the site never has one), else null. */
export function localPage(doc = typeof document !== 'undefined' ? document : null) {
    const m = doc && doc.querySelector('meta[name="shulib-local"]');
    return m && m.content ? { token: m.content, folder: m.dataset.folder || 'shulib' } : null;
}

const fetchOf = (f) => f || ((...a) => globalThis.fetch(...a));
async function call(f, token, method, url, body) {
    try {
        return await f(url, { method, body, cache: 'no-store', headers: { 'X-Shulib-Token': token } });
    } catch {
        throw new Error(GONE);
    }
}
async function refused(res) {
    const words = await res.text().catch(() => '');
    return new Error(words.trim() || `the route builder answered ${res.status}`);
}

/** The local host over the helper (files.mjs's host shape). */
export function localHost({ token, folder }, { fetch } = {}) {
    const f = fetchOf(fetch);
    const at = (path) => `api/file?path=${encodeURIComponent(path)}`;
    const dl = downloadHost();
    return {
        kind: 'local',
        canWriteFolder: true,
        connected: () => true,
        connect: async () => ({ ok: true }),
        folderName: () => folder,
        async readText(path) {
            const res = await call(f, token, 'GET', at(path));
            if (res.status === 204) {  // no such file (a 404 would be a console error in Chromium)
                return null;
            }
            if (!res.ok) {
                throw await refused(res);
            }
            return res.text();
        },
        async writeText(path, text) {
            const res = await call(f, token, 'POST', at(path), text);
            if (!res.ok) {
                throw await refused(res);
            }
        },
        offerDownload: dl.offerDownload,
        pickText: dl.pickText,
    };
}

/** Send to robot `robot` ('one' | 'two'): each line of tools/upload_auton.py to `onLine` AS IT COMES; resolves to its
 *  exit code (0: uploaded). Throws in words when the helper refused (a send already running) or stopped mid-send. */
export async function sendToRobot({ token }, robot, onLine, { fetch } = {}) {
    const res = await call(fetchOf(fetch), token, 'POST', `api/send?robot=${encodeURIComponent(robot)}`);
    if (!res.ok) {
        throw await refused(res);
    }
    const reader = res.body.getReader();
    const text = new TextDecoder();
    let rest = '';
    let code = null;
    const take = (raw) => {
        if (raw.trim()) {
            const m = JSON.parse(raw);
            if (typeof m.line === 'string') {
                onLine(m.line);
            }
            code = m.done ? m.code : code;
        }
    };
    for (;;) {
        let r;
        try {
            r = await reader.read();
        } catch {
            throw new Error(GONE);
        }
        if (r.done) {
            break;
        }
        const lines = (rest + text.decode(r.value, { stream: true })).split('\n');
        rest = lines.pop();
        lines.forEach(take);
    }
    take(rest + text.decode());
    if (code === null) {
        throw new Error(GONE);
    }
    return code;
}

/** build/upload_auton.log: everything make and pros printed ("Show everything"); '' when there is none. */
export async function sendLog({ token }, { fetch } = {}) {
    const res = await call(fetchOf(fetch), token, 'GET', 'api/log');
    return res.ok ? res.text() : '';
}
