#!/usr/bin/env python3
"""
Encoder Tuner - a small control panel for tuning the latency compensation
on the RS485 encoder bridge, live over OSC.

Run it:

    python3 osc_tuner.py

It opens a control panel in your browser with two sliders (lead time and
velocity window) and a Save button, and sends the OSC messages the board
expects.

Deliberately uses nothing outside the Python standard library, so it runs on
a stock macOS Python 3 with no pip install and no tkinter (whose availability
varies depending on how Python was installed). The UI is served to your
browser; the UDP sending happens here in Python, because browsers cannot send
UDP themselves.

The web server binds to localhost only - nothing is exposed to the network.
"""

import http.server
import json
import socket
import struct
import sys
import threading
import webbrowser

# Defaults - editable in the UI, no need to change these.
DEFAULT_HOST = "10.8.0.241"   # the ESP32's IP address
DEFAULT_PORT = 9001           # OSC_LISTEN_PORT in the board's config.py
DEFAULT_PREFIX = "/encoder1"  # /encoder2 for the second board

UI_PORT = 8765


# --------------------------------------------------------------------------
# OSC
# --------------------------------------------------------------------------


def _pad(data: bytes) -> bytes:
    """OSC strings are null-terminated and padded to a multiple of 4 bytes.
    There must be at least one null, hence the padding when already aligned."""
    return data + b"\x00" * (4 - len(data) % 4)


def osc_float(address: str, value: float) -> bytes:
    return _pad(address.encode("utf-8")) + _pad(b",f") + struct.pack(">f", value)


def osc_bare(address: str) -> bytes:
    """A message with no arguments, e.g. the save trigger."""
    return _pad(address.encode("utf-8")) + _pad(b",")


def send_osc(host: str, port: int, message: bytes) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(message, (host, port))
    finally:
        sock.close()


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------

PAGE = r"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Encoder Tuner</title>
<style>
  :root {
    --bg:#15171c; --card:#1e222a; --line:#2c313c; --fg:#e8ebf0;
    --muted:#949cad; --accent:#5db3ff; --ok:#57d38c; --warn:#ffb454;
  }
  * { box-sizing:border-box; }
  body {
    margin:0; padding:28px 20px 40px;
    background:var(--bg); color:var(--fg);
    font:15px/1.5 -apple-system,BlinkMacSystemFont,"SF Pro Text",Helvetica,Arial,sans-serif;
  }
  .wrap { max-width:640px; margin:0 auto; }
  h1 { font-size:19px; margin:0 0 4px; letter-spacing:.2px; }
  .sub { color:var(--muted); font-size:13px; margin-bottom:22px; }
  .card { background:var(--card); border:1px solid var(--line);
          border-radius:12px; padding:18px 20px; margin-bottom:16px; }
  .target { display:flex; gap:10px; flex-wrap:wrap; align-items:flex-end; }
  .target label { display:block; font-size:11px; text-transform:uppercase;
                  letter-spacing:.7px; color:var(--muted); margin-bottom:5px; }
  input[type=text], input[type=number] {
    background:#12141a; border:1px solid var(--line); color:var(--fg);
    border-radius:7px; padding:8px 10px; font-size:14px;
    font-family:ui-monospace,SFMono-Regular,Menlo,monospace;
  }
  .ip { width:150px; } .pt { width:80px; } .px { width:120px; }
  .row { display:flex; justify-content:space-between; align-items:baseline;
         margin-bottom:2px; }
  .name { font-size:12px; text-transform:uppercase; letter-spacing:.8px;
          color:var(--muted); }
  .val { font-size:30px; font-weight:600; font-variant-numeric:tabular-nums;
         font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .unit { font-size:15px; color:var(--muted); font-weight:400; margin-left:3px; }
  .hint { color:var(--muted); font-size:12.5px; margin:10px 0 14px; }
  input[type=range] { width:100%; margin:6px 0 12px; accent-color:var(--accent); height:26px; }
  .nudge { display:flex; gap:8px; align-items:center; }
  button {
    background:#262b35; color:var(--fg); border:1px solid var(--line);
    border-radius:7px; padding:7px 13px; font-size:14px; cursor:pointer;
    font-variant-numeric:tabular-nums;
  }
  button:hover { background:#2f3542; }
  button:active { transform:translateY(1px); }
  .save { width:100%; padding:14px; font-size:15px; font-weight:600;
          background:#1f6b45; border-color:#2b8d5c; }
  .save:hover { background:#248052; }
  .save.done { background:#2fa06a; }
  #log { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:12px;
         color:var(--muted); background:#12141a; border:1px solid var(--line);
         border-radius:8px; padding:10px 12px; height:110px; overflow-y:auto;
         white-space:pre-wrap; }
  .err { color:#ff8080; }
  .sent { color:var(--ok); }
</style>
</head>
<body><div class="wrap">

  <h1>Encoder Tuner</h1>
  <div class="sub">Latency compensation for the RS485 encoder bridge.</div>

  <div class="card target">
    <div><label>Board IP</label><input class="ip" type="text" id="host"></div>
    <div><label>Port</label><input class="pt" type="number" id="port"></div>
    <div><label>OSC prefix</label><input class="px" type="text" id="prefix"></div>
  </div>

  <div class="card">
    <div class="row">
      <span class="name">Lead time</span>
      <span><span class="val" id="leadV">0.500</span><span class="unit">s</span></span>
    </div>
    <input type="range" id="lead" min="0" max="1.5" step="0.005">
    <div class="hint">How far ahead to project the position, in seconds &mdash; set
      this to the video pipeline's latency. Too low and the background lags behind
      the screen; too high and it runs ahead. Adjust while the screen moves at a
      steady, representative speed.</div>
    <div class="nudge">
      <button onclick="nudge('lead',-0.05)">&minus;0.05</button>
      <button onclick="nudge('lead',-0.01)">&minus;0.01</button>
      <button onclick="nudge('lead', 0.01)">+0.01</button>
      <button onclick="nudge('lead', 0.05)">+0.05</button>
    </div>
  </div>

  <div class="card">
    <div class="row">
      <span class="name">Velocity window</span>
      <span><span class="val" id="windowV">0.150</span><span class="unit">s</span></span>
    </div>
    <input type="range" id="window" min="0.02" max="0.5" step="0.005">
    <div class="hint">How long a window the speed is averaged over. Longer is
      steadier but slower to react; shorter is more responsive but noisier. If the
      image jitters while the screen is still, raise this. If it feels sluggish
      starting and stopping, lower it.</div>
    <div class="nudge">
      <button onclick="nudge('window',-0.02)">&minus;0.02</button>
      <button onclick="nudge('window',-0.005)">&minus;0.005</button>
      <button onclick="nudge('window', 0.005)">+0.005</button>
      <button onclick="nudge('window', 0.02)">+0.02</button>
    </div>
  </div>

  <div class="card">
    <button class="save" id="saveBtn" onclick="save()">Save to board</button>
    <div class="hint" style="margin:12px 0 0">Stores both values in the board's
      non-volatile memory so they survive a power cycle. Until you save, they
      apply immediately but revert on reboot.</div>
  </div>

  <div class="card" style="padding:12px 14px">
    <div class="name" style="margin-bottom:8px">Activity</div>
    <div id="log"></div>
  </div>

</div>
<script>
const $ = id => document.getElementById(id);
const DEF = __DEFAULTS__;
$('host').value = DEF.host; $('port').value = DEF.port; $('prefix').value = DEF.prefix;
$('lead').value = DEF.lead; $('window').value = DEF.window;

function log(msg, cls) {
  const el = $('log');
  const t = new Date().toLocaleTimeString();
  el.innerHTML += `<div class="${cls||''}">${t}  ${msg}</div>`;
  el.scrollTop = el.scrollHeight;
}

function fmt(v){ return Number(v).toFixed(3); }

async function post(path, value) {
  const body = { host:$('host').value, port:Number($('port').value), path:path };
  if (value !== undefined) body.value = Number(value);
  try {
    const r = await fetch('/send', {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body)
    });
    const j = await r.json();
    if (!j.ok) { log('FAILED  ' + j.error, 'err'); return false; }
    log(path + (value !== undefined ? '  ' + fmt(value) : ''), 'sent');
    return true;
  } catch (e) { log('FAILED  ' + e, 'err'); return false; }
}

// Rate-limit while dragging so the slider stays smooth, but always send the
// value the user finally lands on.
let pending = {}, timer = {};
function push(which) {
  const v = $(which).value;
  $(which + 'V').textContent = fmt(v);
  pending[which] = v;
  if (timer[which]) return;
  timer[which] = setTimeout(() => {
    timer[which] = null;
    post($('prefix').value + '/' + which, pending[which]);
  }, 40);
}

['lead','window'].forEach(w => {
  $(w).addEventListener('input', () => push(w));
  $(w).addEventListener('change', () => push(w));
  $(w + 'V').textContent = fmt($(w).value);
});

function nudge(which, delta) {
  const el = $(which);
  const lo = parseFloat(el.min), hi = parseFloat(el.max);
  el.value = Math.min(hi, Math.max(lo, parseFloat(el.value) + delta));
  push(which);
}

async function save() {
  if (await post($('prefix').value + '/save')) {
    const b = $('saveBtn');
    b.textContent = 'Saved ✓'; b.classList.add('done');
    setTimeout(() => { b.textContent = 'Save to board'; b.classList.remove('done'); }, 1400);
  }
}

log('Ready. Values apply as you drag; Save makes them permanent.');
</script>
</body></html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def _reply(self, code, payload, content_type="application/json"):
        body = payload if isinstance(payload, bytes) else payload.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            defaults = json.dumps(
                {
                    "host": DEFAULT_HOST,
                    "port": DEFAULT_PORT,
                    "prefix": DEFAULT_PREFIX,
                    "lead": 0.5,
                    "window": 0.15,
                }
            )
            self._reply(200, PAGE.replace("__DEFAULTS__", defaults), "text/html; charset=utf-8")
        else:
            self._reply(404, '{"ok":false,"error":"not found"}')

    def do_POST(self):
        if self.path != "/send":
            self._reply(404, '{"ok":false,"error":"not found"}')
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(length))
            host = str(req["host"]).strip()
            port = int(req["port"])
            path = str(req["path"])

            if "value" in req:
                message = osc_float(path, float(req["value"]))
            else:
                message = osc_bare(path)

            send_osc(host, port, message)
            self._reply(200, '{"ok":true}')
        except Exception as exc:  # noqa: BLE001 - report anything back to the UI
            self._reply(200, json.dumps({"ok": False, "error": str(exc)}))

    def log_message(self, fmt, *args):
        pass  # keep the terminal clean; the UI has its own activity log


def main():
    global DEFAULT_HOST, DEFAULT_PORT, DEFAULT_PREFIX
    if len(sys.argv) > 1:
        DEFAULT_HOST = sys.argv[1]
    if len(sys.argv) > 2:
        DEFAULT_PORT = int(sys.argv[2])
    if len(sys.argv) > 3:
        DEFAULT_PREFIX = sys.argv[3]

    for port in range(UI_PORT, UI_PORT + 20):
        try:
            server = http.server.HTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        print("Could not find a free local port for the UI.")
        return 1

    url = "http://127.0.0.1:%d/" % port
    print("Encoder Tuner running at %s" % url)
    print("Sending OSC to %s:%d, prefix %s" % (DEFAULT_HOST, DEFAULT_PORT, DEFAULT_PREFIX))
    print("Press Ctrl-C to quit.")
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nBye.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
