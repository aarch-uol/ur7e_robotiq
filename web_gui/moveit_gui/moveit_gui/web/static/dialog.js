// The GUI's own pop-ups, instead of the browser's confirm / prompt / alert (those look different in every browser,
// block the page, and some browsers offer to "stop this page from showing dialogs").
//   await ask("Delete part X?")                       true (OK) / false (Cancel, Esc, click outside)
//   await ask(text, { ok: "Give", cancel: "Keep", danger: true, timeout: 30 })
//   await askText("Your name:", "default")            the text, or null on Cancel
//   tell(text)                                        just OK
// The first line is the title, the rest the message. One pop-up at a time: others wait.
// valid: a function; a queued pop-up whose question no longer applies is skipped (answers Cancel).
// A pop-up with `timeout` (seconds) closes by itself when the time runs out (shown on the Cancel button) and answers
// null (not false: nobody pressed Cancel).

let queue = Promise.resolve();
let openNow = null;

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text != null) e.textContent = text;
  return e;
}

function show(text, o) {
  return new Promise((resolve) => {
    if (o.valid && !o.valid()) { resolve(o.input ? null : false); return; }    // waited in the queue: no longer asked
    // the first line is the title; a question that deletes or forgets gets a red button named after it
    const all = String(text), nl = all.indexOf("\n");
    const title = o.title || (nl < 0 ? all : all.slice(0, nl));
    const body = o.title ? all : nl < 0 ? "" : all.slice(nl + 1).replace(/^\s*\n/, "");
    const verb = /^(Delete|Forget|Clear|Remove|Discard)\b/.exec(title);
    if (verb && !o.ok) { o = { ...o, ok: verb[1], danger: true }; }
    if (!o.ok && /\n\nExecute\?$/.test(all)) o = { ...o, ok: "Execute" };
    const back = el("div", "dlg-back");
    const box = el("div", "dlg" + (o.danger ? " danger" : ""));
    box.setAttribute("role", "dialog");
    box.setAttribute("aria-modal", "true");
    box.append(el("div", "dlg-title", title));
    const shown = o.ok === "Execute" ? body.replace(/\n\nExecute\?$/, "") : body;
    if (shown.trim()) box.append(el("div", "dlg-text", shown));
    let input = null;
    if (o.input) {
      input = el("input", "dlg-input");
      input.type = "text";
      input.value = o.value || "";
      input.maxLength = o.maxLength || 80;
      if (o.placeholder) input.placeholder = o.placeholder;
      box.append(input);
    }
    const row = el("div", "dlg-btns");
    const no = o.cancel === false ? null : el("button", "btn quiet", o.cancel || "Cancel");
    const yes = el("button", "btn " + (o.danger ? "danger" : "primary"), o.ok || "OK");
    yes.type = "button";
    if (no) { no.type = "button"; row.append(no); }
    row.append(yes);
    box.append(row);
    back.append(box);
    const before = document.activeElement;
    let timer = 0, left = o.timeout || 0;
    const done = (v) => {
      if (!back.isConnected) return;
      clearInterval(timer);
      back.remove();
      document.removeEventListener("keydown", key, true);
      openNow = null;
      if (before && before.focus && before.isConnected) before.focus({ preventScroll: true });
      resolve(v);
    };
    const answer = (ok) => done(o.input ? (ok ? input.value : null) : ok);
    const key = (e) => {
      if (e.key === "Escape") { e.preventDefault(); e.stopPropagation(); answer(false); }
      else if (e.key === "Enter" && (e.target === input || !e.target.closest || !e.target.closest(".dlg button"))) {
        e.preventDefault(); e.stopPropagation(); answer(true);
      } else if (e.key === "Tab") {           // keep the focus inside the pop-up
        const f = [...box.querySelectorAll("button, input")];
        const i = f.indexOf(document.activeElement);
        e.preventDefault();
        f[(i + (e.shiftKey ? -1 : 1) + f.length) % f.length].focus();
      }
    };
    yes.onclick = () => answer(true);
    if (no) no.onclick = () => answer(false);
    back.addEventListener("pointerdown", (e) => { if (e.target === back && o.cancel !== false) answer(false); });
    document.addEventListener("keydown", key, true);
    if (left) {
      const lbl = no ? no.textContent : "";
      const tick = () => { if (no) no.textContent = `${lbl} (${left})`; if (left-- <= 0) done(null); };
      tick();
      timer = setInterval(tick, 1000);
    }
    document.body.append(back);
    openNow = { close: () => answer(false), key: o.key };
    (input || yes).focus();
    if (input) input.select();
  });
}

function enqueue(text, o) {
  const p = queue.then(() => show(text, o));
  queue = p.catch(() => {});
  return p;
}

export const ask = (text, o = {}) => enqueue(text, o);
export const askText = (text, value = "", o = {}) => enqueue(text, { ...o, input: true, value });
export const tell = (text, o = {}) => enqueue(text, { ...o, cancel: false });
// close the open pop-up if it is the one with this key (e.g. a question that no longer applies)
export function dismiss(key) { if (openNow && openNow.key === key) openNow.close(); }
