// files.mjs — THE FILE LAYER the route tab talks to (RMP1 brief A.3.7, A.6 [W12]; run RMP1-A1).
//
// The screens never call a browser file API; they call this. It is built over a HOST — a few primitives:
//   kind                  'folder' (Chrome/Edge: the connected shulib folder), 'download' (any other browser),
//                         'memory' (the tests' in-memory folder; later a VS Code panel or a local helper)
//   canWriteFolder        true when this host writes into the team's files
//   connected()           a folder is connected (and allowed) now
//   connect()             ask for the folder (Chrome/Edge: the folder dialog)
//   readText(path)        a file's text, or null when it does not exist
//   writeText(path, text) replace a file's text (all or nothing)
//   offerDownload(name, text)  hand the student a file to save (every browser)
//   pickText()            let the student pick a file; its {name, text}, or null
// and EVERY rule — only under src/team/, only between the markers, the hand-edit hash, the file's own line
// endings — is region.mjs's, applied here the same way for every host (so a later host re-implements a few
// calls, never a rule).

import { isTeamPath, spliceRegion, findRegion, regionText, regionHash, exportKey, hasMirroredLine, SKILLS_ROUTE_FILE,
         ROBOT_TWO_AUTONS_FILE } from '../model/region.mjs';
import { exportRegion } from '../model/export.mjs';
import { saveProject, loadProject, PROJECT_FILE } from '../model/project.mjs';

/** The file layer over `host` (header). */
export function makeFiles(host) {
    const guard = (path) => {
        if (!isTeamPath(path)) {
            throw new Error(`the route builder writes only under src/team/ -- not "${path}"`);
        }
    };
    return {
        kind: host.kind,
        canWriteFolder: host.canWriteFolder,
        connected: () => host.connected(),
        connect: () => host.connect(),
        folderName: () => (host.folderName ? host.folderName() : ''),

        /**
         * Save the project: into src/team/route_maker.json (a folder host), else a download. `base` is the file's
         * text when this page last read or wrote it (null: it did not exist; undefined: never read — then nothing
         * is compared). A folder file that is no longer `base` changed under the page (a teammate's route after a
         * `git pull`, another window): it is NOT replaced unless `force` — {ok: false, why: 'changedOnDisk', diskText}.
         */
        async saveProject(project, { base, force = false } = {}) {
            const text = saveProject(project);
            if (host.canWriteFolder && host.connected()) {
                guard(PROJECT_FILE);
                if (base !== undefined && !force) {
                    const disk = await host.readText(PROJECT_FILE);
                    if (disk !== null && disk !== base && disk !== text) {  // a file gone since is nothing to lose
                        return { ok: false, why: 'changedOnDisk', diskText: disk,
                                 words: `${PROJECT_FILE} changed in the folder since this page read it (a git pull, or another window).` };
                    }
                }
                await host.writeText(PROJECT_FILE, text);
                return { ok: true, where: 'folder', text, words: `Saved ${PROJECT_FILE}` };
            }
            await host.offerDownload('route_maker.json', text);
            return { ok: true, where: 'download', text, words: 'Downloaded route_maker.json: put it in your shulib folder\'s src/team/' };
        },

        /** The folder's project file as text (null: none, or no folder) — what the page compares itself with. */
        async readProjectFile() {
            if (!(host.canWriteFolder && host.connected())) {
                return null;
            }
            return host.readText(PROJECT_FILE);
        },

        /** Open the project: src/team/route_maker.json from the folder, else a file the student picks. */
        async openProject() {
            let text = null;
            let where = 'picked';
            if (host.canWriteFolder && host.connected()) {
                text = await host.readText(PROJECT_FILE);
                if (text === null) {
                    return { ok: false, why: 'none', words: `There is no ${PROJECT_FILE} in this folder yet: save once to make it.` };
                }
                where = 'folder';
            } else {
                const picked = await host.pickText();
                if (!picked) {
                    return { ok: false, why: 'cancelled', words: '' };
                }
                text = picked.text;
            }
            return { ...loadProject(text), text, where };
        },

        /** What the skills-route file looks like: 'ready' (one region), 'noMarkers', 'noFile', or 'unknown'. */
        async skillsFileState() {
            if (!(host.canWriteFolder && host.connected())) {
                return { state: 'unknown' };
            }
            const text = await host.readText(SKILLS_ROUTE_FILE);
            if (text === null) {
                return { state: 'noFile' };
            }
            const f = findRegion(text);
            return f.ok ? { state: 'ready', name: f.name } : { state: f.why === 'noMarkers' ? 'noMarkers' : 'bad', words: f.words };
        },

        /** Is robot two's mirrored line there (D6)? 'present', 'missing', or 'unknown' (a host that cannot read). */
        async mirroredLine() {
            if (!(host.canWriteFolder && host.connected())) {
                return 'unknown';
            }
            const text = await host.readText(ROBOT_TWO_AUTONS_FILE);
            if (text === null) {
                return 'missing';
            }
            return hasMirroredLine(text) ? 'present' : 'missing';
        },

        /**
         * Write the route into the skills-route file's region (D4), keeping its line endings. Returns
         * {ok, project (its `exports` hash updated), words} or {ok: false, why, words}. `overwrite`: the student
         * chose to replace a hand edit.
         */
        async exportToFile(route, project, { overwrite = false } = {}) {
            guard(SKILLS_ROUTE_FILE);
            if (!(host.canWriteFolder && host.connected())) {
                return { ok: false, why: 'noFolder', words: 'Connect your shulib folder first (Chrome or Edge).' };
            }
            const text = await host.readText(SKILLS_ROUTE_FILE);
            if (text === null) {
                return { ok: false, why: 'noFile', words: `There is no ${SKILLS_ROUTE_FILE} in this folder: it arrives with `
                    + 'robot two\'s update. Until then, use Copy.' };
            }
            const found = findRegion(text);
            const key = exportKey(SKILLS_ROUTE_FILE, found.ok ? found.name : route.name);
            const out = spliceRegion(text, exportRegion(route), { lastHash: project.exports[key], overwrite });
            if (!out.ok) {
                return out;
            }
            if (out.changed) {
                await host.writeText(SKILLS_ROUTE_FILE, out.text);
            }
            const exportsNow = { ...project.exports };
            delete exportsNow[key];
            exportsNow[exportKey(SKILLS_ROUTE_FILE, route.name)] = out.hash;
            return { ok: true, project: { ...project, exports: exportsNow }, changed: out.changed,
                     words: out.changed ? `Wrote the route into ${SKILLS_ROUTE_FILE}` : `${SKILLS_ROUTE_FILE} already has this route` };
        },

        /**
         * Every other browser: the student picks their skills_route.cpp, gets the updated file back as a
         * download to put in place (the same rules: only the region changes, its line endings kept).
         */
        async exportByDownload(route, project, { overwrite = false } = {}) {
            const picked = await host.pickText();
            if (!picked) {
                return { ok: false, why: 'cancelled', words: '' };
            }
            const found = findRegion(picked.text);
            const key = exportKey(SKILLS_ROUTE_FILE, found.ok ? found.name : route.name);
            const out = spliceRegion(picked.text, exportRegion(route), { lastHash: project.exports[key], overwrite });
            if (!out.ok) {
                return out;
            }
            await host.offerDownload('skills_route.cpp', out.text);
            const exportsNow = { ...project.exports };
            delete exportsNow[key];
            exportsNow[exportKey(SKILLS_ROUTE_FILE, route.name)] = out.hash;
            return { ok: true, project: { ...project, exports: exportsNow },
                     words: 'Downloaded the updated skills_route.cpp: put it back in src/team/ in place of the old one' };
        },

        /** The region's text currently in the file (for "what changed" in the export drawer), or null. */
        async currentRegion() {
            if (!(host.canWriteFolder && host.connected())) {
                return null;
            }
            const text = await host.readText(SKILLS_ROUTE_FILE);
            const f = text === null ? null : findRegion(text);
            return f && f.ok ? regionText(text, f) : null;
        },
        regionHash,
    };
}

/** An in-memory folder host: {path: text}. The tests' host, and the shape every host follows. */
export function memoryHost(files = {}, { connected = true } = {}) {
    const store = { ...files };
    const downloads = [];
    let picks = [];
    return {
        kind: 'memory',
        canWriteFolder: true,
        connected: () => connected,
        connect: async () => {
            connected = true;
            return true;
        },
        folderName: () => 'shulib',
        readText: async (p) => (p in store ? store[p] : null),
        writeText: async (p, t) => {
            store[p] = t;
        },
        offerDownload: async (name, text) => {
            downloads.push({ name, text });
        },
        pickText: async () => picks.shift() || null,
        // for the tests
        store,
        downloads,
        willPick(...f) {
            picks = f;
        },
    };
}
