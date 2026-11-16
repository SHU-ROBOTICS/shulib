// files_download.mjs — the DOWNLOAD host: every browser (Firefox, Safari, and Chrome/Edge before a folder is
// connected; RMP1 brief A.3.7). Save is a download of route_maker.json; Open picks a file; Export is Copy, or
// "pick your skills_route.cpp -> download the updated one" for the student to put back. It never writes
// into the team's folder, so `canWriteFolder` is false. Browser only.

/** The download host. */
export function downloadHost() {
    return {
        kind: 'download',
        canWriteFolder: false,
        connected: () => false,
        folderName: () => '',
        connect: async () => ({ ok: false, words: 'Writing into a folder needs Chrome or Edge.' }),
        readText: async () => null,
        writeText: async () => {
            throw new Error('this browser cannot write into a folder');
        },
        async offerDownload(name, text) {
            const url = URL.createObjectURL(new Blob([text], { type: 'text/plain;charset=utf-8' }));
            const a = document.createElement('a');
            a.href = url;
            a.download = name;
            a.style.display = 'none';
            document.body.append(a);
            a.click();
            a.remove();
            setTimeout(() => URL.revokeObjectURL(url), 4000);
        },
        pickText() {
            return new Promise((resolve) => {
                const input = document.createElement('input');
                input.type = 'file';
                input.accept = '.json,.cpp,.hpp,.txt';
                input.style.display = 'none';
                input.addEventListener('change', async () => {
                    const f = input.files && input.files[0];
                    input.remove();
                    resolve(f ? { name: f.name, text: await f.text() } : null);
                });
                input.addEventListener('cancel', () => {
                    input.remove();
                    resolve(null);
                });
                document.body.append(input);
                input.click();
            });
        },
    };
}

/** Copy text: the Clipboard API where the page may, else a selected text area (every browser). */
export async function copyText(text) {
    try {
        if (navigator.clipboard && window.isSecureContext !== false) {
            await navigator.clipboard.writeText(text);
            return true;
        }
    } catch {
        // fall through to the old way
    }
    const ta = document.createElement('textarea');
    ta.value = text;
    ta.setAttribute('readonly', '');
    ta.style.position = 'fixed';
    ta.style.opacity = '0';
    document.body.append(ta);
    ta.select();
    let ok = false;
    try {
        ok = document.execCommand('copy');
    } catch {
        ok = false;
    }
    ta.remove();
    return ok;
}
