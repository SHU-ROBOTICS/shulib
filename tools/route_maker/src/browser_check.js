// browser_check.js — RUNS FIRST, as a classic inline script, before any module (RMP1 brief P-W3, [W16]; A.3.9).
//
// Two questions, answered in plain words before the app loads:
//   1. Can this browser run the route builder at all (ES modules)? If not, the page says so and stops — never a
//      blank or half-drawn page.
//   2. Can it run the route PREVIEW (WebAssembly with its exception instructions, and module Workers — what the
//      preview engine needs)? If not, a yellow box says which browsers can, and the builder itself still opens,
//      saves and exports. The floor the page states: Chrome or Edge 97, Firefox 115, Safari 16.
// It never requests anything (the module is 28 bytes, made here; the Worker is a blob: URL the page's own
// Content-Security-Policy allows). window.shulibBrowser holds the answers for the app.
(function () {
  'use strict';
  var NEEDS = 'Chrome or Edge 97, Firefox 115 or Safari 16 (or newer)';
  var canModules = 'noModule' in document.createElement('script');
  var canWasmEh = false;
  try {
    // a module whose one function uses the try / catch_all instructions the preview engine is built with
    var bytes = new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0, 1, 4, 1, 96, 0, 0, 3, 2, 1, 0, 10, 8, 1, 6, 0, 6, 64, 25, 11, 11]);
    canWasmEh = typeof WebAssembly === 'object' && WebAssembly.validate(bytes);
  } catch (e) {
    canWasmEh = false;
  }
  var canModuleWorker = false;
  try {
    var probe = { get type() { canModuleWorker = true; return 'module'; } };
    var url = URL.createObjectURL(new Blob([''], { type: 'text/javascript' }));
    new Worker(url, probe).terminate();
    URL.revokeObjectURL(url);
  } catch (e) {
    // a browser without Workers (or one that refuses this one) cannot run the preview's engine
  }
  window.shulibBrowser = { modules: canModules, wasm: canWasmEh, moduleWorker: canModuleWorker, preview: canWasmEh && canModuleWorker };
  function box(title, text) {
    var d = document.createElement('div');
    d.className = 'too-old';
    d.setAttribute('role', 'alert');
    var h = document.createElement('h1');
    h.textContent = title;
    var p = document.createElement('p');
    p.textContent = text;
    d.appendChild(h);
    d.appendChild(p);
    return d;
  }
  document.addEventListener('DOMContentLoaded', function () {
    if (!canModules) {
      document.body.innerHTML = '';
      document.body.style.overflow = 'auto';
      document.body.appendChild(box('This browser is too old for the route builder',
        'The route builder needs ' + NEEDS + '. Please update the browser, or open the route builder in another one.'));
      document.documentElement.setAttribute('data-ready', 'refused');
      return;
    }
    if (!window.shulibBrowser.preview) {
      var b = box('This browser is too old to run the route preview',
        'The preview needs ' + NEEDS + '. Please update the browser (or use another one); the route builder itself still opens, saves and exports.');
      b.id = 'too-old-preview';
      var close = document.createElement('button');
      close.className = 'btn';
      close.textContent = 'Use the route builder without the preview';
      close.addEventListener('click', function () { b.remove(); });
      b.appendChild(close);
      document.body.insertBefore(b, document.body.firstChild);
    }
  });
})();
