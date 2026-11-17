// capabilities.mjs — WHAT THIS HOST CAN DO (RMP1 brief A.6). The shell shows a button only when its capability
// is here, and says ONCE which of them this browser lacks. A later Build / Upload / Debug panel ADDS
// capabilities (`build`, `upload`, `brainLog`) and pages; the route tab is not rewritten.

import { folderHostAvailable } from './files_fsaccess.mjs';
import { previewAvailability } from './engine.mjs';

/** This page's capabilities. */
export function capabilities() {
    return {
        writeFolder: folderHostAvailable(),
        preview: previewAvailability().available,
        build: false,
        upload: false,
        brainLog: false,
    };
}

/** The one sentence saying what this browser offers (§1.4's "said once"). */
export function capabilityWords(caps) {
    return caps.writeFolder
        ? 'Connect your shulib folder once: Save writes src/team/route_maker.json and Export writes the code into src/team/skills_route.cpp.'
        : 'In this browser Save downloads the project and Export copies the code. To write into the shulib folder, open the route builder in Chrome or Edge.';
}
