// files_fsaccess.mjs — the FOLDER host: Chrome and Edge on a desktop (RMP1 brief A.3.7, A.6; landmine 23).
//
// "Connect your shulib folder" once (showDirectoryPicker, read and write); the folder's handle is remembered
// in this browser (IndexedDB), so a later visit needs one "Allow" click. It reads and writes TEXT only — every
// rule is region.mjs's, applied by files.mjs. A write is a file's writable stream, which replaces the file
// whole when it closes, or not at all. The folder must be the shulib repository (it has src/team/): a student
// who picks src/team itself is told so.
//
// Browser only (no node test reaches it; the page tests and the manual Windows check do).

import { downloadHost } from './files_download.mjs';

const DB = 'shulib-route-builder';
const STORE = 'handles';

function idb(mode, fn) {
    return new Promise((resolve, reject) => {
        const open = indexedDB.open(DB, 1);
        open.onupgradeneeded = () => open.result.createObjectStore(STORE);
        open.onerror = () => reject(open.error);
        open.onsuccess = () => {
            const tx = open.result.transaction(STORE, mode);
            const req = fn(tx.objectStore(STORE));
            tx.oncomplete = () => resolve(req && req.result);
            tx.onerror = () => reject(tx.error);
        };
    });
}

/** True where this host works: a secure page with the File System Access API (Chrome / Edge desktop). */
export function folderHostAvailable() {
    return typeof window !== 'undefined' && typeof window.showDirectoryPicker === 'function' && window.isSecureContext !== false;
}

/** The folder host (header). Falls back to the download host's offerDownload / pickText for those two calls. */
export async function folderHost() {
    const dl = downloadHost();
    let dir = null;
    let allowed = false;
    try {
        dir = await idb('readonly', (s) => s.get('folder'));
        if (dir && (await dir.queryPermission({ mode: 'readwrite' })) === 'granted') {
            allowed = true;
        }
    } catch {
        dir = null;
    }
    async function walk(path, create) {
        const parts = path.split('/');
        let d = dir;
        for (const p of parts.slice(0, -1)) {
            d = await d.getDirectoryHandle(p, { create: false });
        }
        return d.getFileHandle(parts[parts.length - 1], { create });
    }
    return {
        kind: 'folder',
        canWriteFolder: true,
        connected: () => !!dir && allowed,
        folderName: () => (dir ? dir.name : ''),
        hasRemembered: () => !!dir && !allowed,
        async connect() {
            if (dir && !allowed) {
                allowed = (await dir.requestPermission({ mode: 'readwrite' })) === 'granted';
                if (allowed) {
                    return { ok: true };
                }
            }
            const picked = await window.showDirectoryPicker({ id: 'shulib', mode: 'readwrite' });
            try {
                const src = await picked.getDirectoryHandle('src');
                await src.getDirectoryHandle('team');
            } catch {
                return { ok: false, words: `"${picked.name}" is not the shulib folder: pick the folder that HAS src/team/ in it `
                    + '(the repository\'s top folder).' };
            }
            dir = picked;
            allowed = true;
            try {
                await idb('readwrite', (s) => s.put(dir, 'folder'));
            } catch {
                // remembered next time only where IndexedDB keeps it; connecting still worked
            }
            return { ok: true };
        },
        async readText(path) {
            try {
                const f = await (await walk(path, false)).getFile();
                return await f.text();
            } catch (e) {
                if (e && e.name === 'NotFoundError') {
                    return null;
                }
                throw e;
            }
        },
        async writeText(path, text) {
            const h = await walk(path, true);
            const w = await h.createWritable();
            await w.write(text);
            await w.close();
        },
        offerDownload: dl.offerDownload,
        pickText: dl.pickText,
    };
}
