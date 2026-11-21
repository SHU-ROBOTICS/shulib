// robot_names.mjs — the robots' names in what people see (RMP1-L; the team lead, 2026-11-20): the page's one spelling
// (the Python tools': upload_auton.py's NAMES; the VS Code task labels: .vscode/tasks.json). Data keys ('one', 'two'),
// file names and the exported C++ keep "robot one" / "robot two".

const NAMES = { one: 'White Bot', two: 'Black Bot' };

/** Robot `id`'s name: 'one' -> "White Bot", 'two' -> "Black Bot" (another id: "robot <id>"). */
export function robotName(id) {
    return NAMES[id] || `robot ${id}`;
}

/** The bridge to a file or doc that still says "robot one" (D2): "White Bot (robot one)". */
export function robotBridge(id) {
    return NAMES[id] ? `${NAMES[id]} (robot ${id})` : robotName(id);
}
