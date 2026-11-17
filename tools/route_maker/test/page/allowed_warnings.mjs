// allowed_warnings.mjs — the ONE list of console warnings a page test lets pass (the RMP1 brief A.4 [W1, W6]): every
// other warning, every console error and every page error fails the test. engine.test.mjs uses it; page.test.mjs and
// A2u's PREVIEW test import it at A2u's merge (RMP1-A2e). Today one: Firefox's, on each instance of
// the module, about its legacy WebAssembly exception instructions (kept for the 21st, A.3.3 [W6]; RMP1-C moves to
// exnref). Firefox hands a Worker's warning to the page wrapped as `[JavaScript Warning: "<sentence>"]`.
export const ALLOWED_WARNINGS = [
    "The WebAssembly exception handling 'try' instruction is deprecated and should no longer be used. Please recompile to use the 'try_table' instruction instead.",
];

/** Is `text` (a console warning's) exactly one of the list's sentences, as it is or as Firefox forwards a Worker's? */
export const allowedWarning = (text) => {
    const forwarded = /^\[JavaScript Warning: "(.*)"\]$/s.exec(text);
    return ALLOWED_WARNINGS.includes(forwarded ? forwarded[1] : text);
};
