#!/usr/bin/env python3
"""
Encoder Tuner - a small control panel for tuning the latency compensation
on the RS485 encoder bridge, live over OSC.

Run it:

    python3 osc_tuner.py

It opens a control panel in your browser listing every board it hears on the
network, with sliders and editable fields for the lead time and velocity
window, the board's OSC identity and destination, and a live readout of what
it is actually doing. Boards are discovered from their broadcast beacon, so
there is no address to configure.

Deliberately uses nothing outside the Python standard library, so it runs on a
stock macOS Python 3 with no pip install and no tkinter (whose availability
varies depending on how Python was installed). The UI is served to your
browser; the UDP sending and receiving happen here in Python, because browsers
cannot do UDP themselves.

The web server binds to localhost only - nothing is exposed to the network.
"""

import http.server
import json
import socket
import struct
import sys
import threading
import time
import webbrowser

# Boards are found by their broadcast beacon, so there is no address to
# configure here. These must match the board's config.py.
# (Position messages themselves go to the board's own configured destination
# on port 9000 - nothing to do with this tool.)
CONTROL_PORT = 9001    # OSC_LISTEN_PORT on the board
STATUS_PORT = 9002     # OSC_STATUS_PORT on the board
ANNOUNCE_PORT = 9003   # OSC_ANNOUNCE_PORT on the board

UI_PORT = 8765
STATUS_STALE_S = 2.0
DEVICE_STALE_S = 8.0  # beacons arrive every ~2s


# --------------------------------------------------------------------------
# OSC
# --------------------------------------------------------------------------


def _pad(data: bytes) -> bytes:
    """OSC strings are null-terminated and padded to a multiple of 4 bytes.
    There must be at least one null, hence padding even when already aligned."""
    return data + b"\x00" * (4 - len(data) % 4)


def osc_float(address: str, value: float) -> bytes:
    return _pad(address.encode("utf-8")) + _pad(b",f") + struct.pack(">f", value)


def osc_string(address: str, value: str) -> bytes:
    return _pad(address.encode("utf-8")) + _pad(b",s") + _pad(value.encode("utf-8"))


def osc_bare(address: str) -> bytes:
    return _pad(address.encode("utf-8")) + _pad(b",")


def osc_args(address: str, args) -> bytes:
    """Build a message from a mixed list of strings/ints/floats.

    Used for /control/dest, which carries address and port together so the
    board never briefly points at a host/port pair that never existed.
    """
    tags = ","
    payload = b""
    for arg in args:
        if isinstance(arg, str):
            tags += "s"
            payload += _pad(arg.encode("utf-8"))
        elif isinstance(arg, bool) or isinstance(arg, int):
            tags += "i"
            payload += struct.pack(">i", int(arg))
        else:
            tags += "f"
            payload += struct.pack(">f", float(arg))
    return _pad(address.encode("utf-8")) + _pad(tags.encode("utf-8")) + payload


def parse_osc(data: bytes):
    """Minimal parser -> (address, [args]); floats and ints only."""
    try:
        end = data.find(b"\x00")
        if end < 0:
            return None, []
        address = data[:end].decode("utf-8")
        pos = (end // 4 + 1) * 4
        if len(data) <= pos or data[pos] != 0x2C:
            return address, []
        tag_end = data.find(b"\x00", pos)
        if tag_end < 0:
            return address, []
        tags = data[pos + 1 : tag_end].decode("utf-8")
        pos = (tag_end // 4 + 1) * 4
        args = []
        for tag in tags:
            if tag == "f":
                args.append(struct.unpack_from(">f", data, pos)[0])
                pos += 4
            elif tag == "i":
                args.append(struct.unpack_from(">i", data, pos)[0])
                pos += 4
            elif tag == "s":
                s_end = data.find(b"\x00", pos)
                if s_end < 0:
                    break
                args.append(data[pos:s_end].decode("utf-8"))
                pos += ((s_end - pos) // 4 + 1) * 4
            else:
                break
        return address, args
    except (ValueError, struct.error, UnicodeDecodeError):
        return None, []


def send_osc(host: str, port: int, message: bytes) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(message, (host, port))
    finally:
        sock.close()


# --------------------------------------------------------------------------
# Status listener
# --------------------------------------------------------------------------

_status_lock = threading.Lock()
_status = {"at": 0.0, "values": None, "address": None}


_devices_lock = threading.Lock()
_devices = {}  # ip -> {prefix, position, lead, window, at}


def announce_listener():
    """Collect discovery beacons so boards can be listed rather than typed in.

    Boards broadcast to 255.255.255.255; the sender's IP comes from the packet
    itself, so nothing has to be configured at either end.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", ANNOUNCE_PORT))
    except OSError as exc:
        print("Could not listen for announcements on port %d: %s" % (ANNOUNCE_PORT, exc))
        print("Board discovery will be unavailable; enter the IP manually.")
        return
    sock.settimeout(1.0)
    while True:
        try:
            data, sender = sock.recvfrom(512)
        except socket.timeout:
            continue
        except OSError:
            break
        address, args = parse_osc(data)
        if address == "/encoder/announce" and len(args) >= 6:
            with _devices_lock:
                _devices[sender[0]] = {
                    "prefix": args[0],
                    "dest": args[1],
                    "destPort": int(args[2]),
                    "position": args[3],
                    "lead": args[4],
                    "window": args[5],
                    "at": time.monotonic(),
                }


def status_listener():
    """Receive <prefix>/status from the board.

    The board sends these only to whoever last spoke to it, so simply using
    the tuner subscribes it; there is nothing to configure.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", STATUS_PORT))
    except OSError as exc:
        print("Could not listen for status on port %d: %s" % (STATUS_PORT, exc))
        print("The panel will still tune, but the status readout stays blank.")
        return
    sock.settimeout(1.0)
    while True:
        try:
            data, _ = sock.recvfrom(512)
        except socket.timeout:
            continue
        except OSError:
            break
        address, args = parse_osc(data)
        if address and address.endswith("/status") and len(args) >= 4:
            with _status_lock:
                _status["at"] = time.monotonic()
                _status["values"] = args[:4]
                _status["address"] = address


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
    --muted:#8f97a8; --accent:#5db3ff; --ok:#57d38c; --dim:#3a4150;
  }
  * { box-sizing:border-box; }
  body {
    margin:0; padding:12px 16px 8px;
    background:var(--bg); color:var(--fg);
    font:13px/1.45 -apple-system,BlinkMacSystemFont,"SF Pro Text",Helvetica,Arial,sans-serif;
  }
  .wrap { max-width:660px; margin:0 auto; }
  .top { display:flex; align-items:baseline; justify-content:space-between; margin-bottom:10px; }
  h1 { font-size:15px; margin:0; letter-spacing:.2px; }
  .top .sub { color:var(--muted); font-size:11.5px; }
  .card { background:var(--card); border:1px solid var(--line);
          border-radius:10px; padding:10px 13px; margin-bottom:8px; }
  .hd { font-size:10px; text-transform:uppercase; letter-spacing:.8px;
        color:var(--muted); margin-bottom:8px; display:flex;
        justify-content:space-between; align-items:center; }
  input[type=text], input[type=number] {
    background:#12141a; border:1px solid var(--line); color:var(--fg);
    border-radius:6px; padding:5px 8px; font-size:13px;
    font-family:ui-monospace,SFMono-Regular,Menlo,monospace;
  }
  input:focus { outline:none; border-color:var(--accent); }
  button {
    background:#262b35; color:var(--fg); border:1px solid var(--line);
    border-radius:6px; padding:5px 12px; font-size:13px; cursor:pointer;
  }
  button:hover { background:#2f3542; }
  button:active { transform:translateY(1px); }

  /* boards */
  .dev { display:flex; align-items:center; gap:10px; padding:6px 9px;
         background:#12141a; border:1px solid var(--line); border-radius:7px;
         margin-bottom:5px; cursor:pointer; }
  .dev:last-child { margin-bottom:0; }
  .dev:hover { border-color:#44506a; }
  .dev.sel { border-color:var(--accent); background:#16202c; }
  .dev .pfx { font-weight:600; font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .dev .ip, .dev .pos { color:var(--muted); font-size:12px;
                        font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .dev .pos { margin-left:auto; font-variant-numeric:tabular-nums; }
  .empty { color:var(--muted); font-size:12px; padding:3px 1px; }

  /* status tiles */
  .stat { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; }
  .stat div { background:#12141a; border:1px solid var(--line); border-radius:7px;
              padding:6px 9px; }
  .stat .k { font-size:9.5px; text-transform:uppercase; letter-spacing:.5px; color:var(--muted); }
  .stat .v { font-size:17px; font-weight:600; font-variant-numeric:tabular-nums;
             font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .dot { display:inline-block; width:7px; height:7px; border-radius:50%;
         background:var(--dim); margin-right:6px; vertical-align:middle; }
  .dot.live { background:var(--ok); }
  .stale .v { color:var(--dim); }

  /* sliders */
  .ctl { display:flex; align-items:center; gap:10px; }
  .ctl + .ctl { margin-top:9px; }
  .ctl .lbl { font-size:10px; text-transform:uppercase; letter-spacing:.7px;
              color:var(--muted); width:52px; flex:none; }
  .ctl input[type=range] { flex:1; accent-color:var(--accent); height:20px; margin:0; }
  .ctl .num { width:74px; flex:none; text-align:right; font-weight:600;
              font-variant-numeric:tabular-nums; }
  .ctl .u { color:var(--muted); font-size:11px; width:8px; flex:none; }

  /* routing row */
  .route { display:flex; align-items:center; gap:8px; flex-wrap:wrap; }
  .route .lbl { font-size:10px; text-transform:uppercase; letter-spacing:.7px;
                color:var(--muted); }
  #prefix { width:110px; } #dest { width:118px; } #destPort { width:64px; }

  #log { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:11px;
         color:var(--muted); background:#12141a; border:1px solid var(--line);
         border-radius:7px; padding:6px 9px; height:44px; overflow-y:auto;
         white-space:pre-wrap; }
  .err { color:#ff8080; } .sent { color:var(--ok); }
  .tip { color:var(--muted); font-size:11px; margin-top:7px; }
</style>
</head>
<body><div class="wrap">

  <div class="top">
    <h1>Encoder Tuner</h1>
  </div>

  <div class="card">
    <div class="hd"><span>Boards on the network</span><span id="devCount"></span></div>
    <div id="devList"></div>
    <div class="tip">Click to select. Unsure which is which? Pull a rope and watch a position move.</div>
  </div>

  <div class="card">
    <div class="hd"><span><span class="dot" id="dot"></span>Live</span><span id="statAddr"></span></div>
    <div class="stat" id="statGrid">
      <div><div class="k">Position</div><div class="v" id="sPos">&mdash;</div></div>
      <div><div class="k">Velocity</div><div class="v" id="sVel">&mdash;</div></div>
      <div><div class="k">Correction</div><div class="v" id="sCorr">&mdash;</div></div>
    </div>
  </div>

  <div class="card">
    <div class="ctl">
      <span class="lbl">Lead</span>
      <input type="range" id="lead" min="0" max="1.5" step="0.005">
      <input class="num" type="number" id="leadN" step="0.005" min="0" max="1.5">
      <span class="u">s</span>
    </div>
    <div class="ctl">
      <span class="lbl">Window</span>
      <input type="range" id="window" min="0.02" max="0.5" step="0.005">
      <input class="num" type="number" id="windowN" step="0.005" min="0.02" max="0.5">
      <span class="u">s</span>
    </div>
    <div class="tip">Lead = pipeline latency to project ahead. Window = how long velocity
      is averaged: longer is steadier, shorter is quicker. Slider applies on release,
      field on Enter.</div>
  </div>

  <div class="card">
    <div class="route">
      <span class="lbl">Sends as</span>
      <input type="text" id="prefix" placeholder="/encoder1">
      <span class="lbl">to</span>
      <input type="text" id="dest" placeholder="10.8.1.81">
      <span class="lbl">:</span>
      <input type="number" id="destPort" min="1" max="65535" placeholder="9000">
      <button onclick="applyRouting()">Apply</button>
    </div>
    <div class="tip">Changing either breaks an existing Millumin mapping. Everything
      here is saved to the board automatically.</div>
  </div>

  <div class="card" style="padding:8px 10px">
    <div id="log"></div>
  </div>

</div>
<script>
const $ = id => document.getElementById(id);
const DEF = __DEFAULTS__;
// The selected board's IP. There is no IP field; selection lives here and is
// shown by highlighting a row in the discovered list.
let selected = null;
$('lead').value = DEF.lead;   $('leadN').value = DEF.lead;
$('window').value = DEF.window; $('windowN').value = DEF.window;

function log(msg, cls) {
  const el = $('log');
  el.innerHTML += `<div class="${cls||''}">${new Date().toLocaleTimeString()}  ${msg}</div>`;
  el.scrollTop = el.scrollHeight;
}
const fmt = v => Number(v).toFixed(3);

async function post(path, value, args, quiet) {
  if (!selected) {
    if (!quiet) log('no board selected - pick one from the list', 'err');
    return false;
  }
  const body = { host:selected, port:DEF.cport, path:path };
  if (value !== undefined && value !== null) body.value = Number(value);
  if (args  !== undefined && args  !== null) body.args  = args;
  try {
    const r = await fetch('/send', {method:'POST',
      headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
    const j = await r.json();
    if (!j.ok) { log('FAILED  ' + j.error, 'err'); return false; }
    if (!quiet) {
      let s = '';
      if (value !== undefined && value !== null) s = '  ' + fmt(value);
      else if (args) s = '  ' + args.join(' ');
      log(path + s, 'sent');
    }
    return true;
  } catch (e) { if (!quiet) log('FAILED  ' + e, 'err'); return false; }
}

// Slider sends when released; number field sends on Enter or blur. They
// mirror each other without either re-triggering a send.
function wire(which) {
  const slider = $(which), num = $(which + 'N');
  const lo = parseFloat(slider.min), hi = parseFloat(slider.max);
  slider.addEventListener('input',  () => { num.value = fmt(slider.value); });
  slider.addEventListener('change', () => {
    num.value = fmt(slider.value);
    post('/control/' + which, slider.value);
  });
  const commit = () => {
    let v = parseFloat(num.value);
    if (isNaN(v)) { num.value = fmt(slider.value); return; }
    v = Math.min(hi, Math.max(lo, v));
    num.value = fmt(v); slider.value = v;
    post('/control/' + which, v);
  };
  num.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); num.blur(); } });
  num.addEventListener('blur', commit);
}
['lead','window'].forEach(wire);

// Prefix and destination go together on one button: address and port travel
// in a single message so the board never points somewhere that never existed.
async function applyRouting() {
  let p = $('prefix').value.trim();
  if (!p.startsWith('/')) p = '/' + p;
  if (p.endsWith('/')) p = p.slice(0, -1);
  if (!/^\/[A-Za-z0-9_\-\/]+$/.test(p)) {
    log('invalid prefix: letters, digits, _ or - e.g. /encoder2', 'err'); return;
  }
  const d = $('dest').value.trim();
  if (!/^(\d{1,3}\.){3}\d{1,3}$/.test(d) || d.split('.').some(n => +n > 255)) {
    log('invalid destination: expected an IPv4 address', 'err'); return;
  }
  const port = parseInt($('destPort').value, 10);
  if (!(port >= 1 && port <= 65535)) { log('invalid port', 'err'); return; }
  $('prefix').value = p; $('dest').value = d;
  await post('/control/prefix', null, [p]);
  await post('/control/dest', null, [d, port]);
}

function selectDevice(dev) {
  selected = dev.ip;
  // Show what the board reports, not whatever was last typed.
  $('prefix').value = dev.prefix;
  $('dest').value = dev.dest;
  $('destPort').value = dev.destPort;
  $('lead').value = dev.lead;     $('leadN').value = dev.lead.toFixed(3);
  $('window').value = dev.window; $('windowN').value = dev.window.toFixed(3);
  log('selected ' + dev.prefix + ' at ' + dev.ip, 'sent');
  post('/control/ping', null, null, true);
  renderDevices(lastDevices);
}

let lastDevices = [];
function renderDevices(devices) {
  const list = $('devList');
  $('devCount').textContent = devices.length ? (devices.length + ' found') : '';
  if (!devices.length) {
    list.innerHTML = '<div class="empty">Listening&hellip; none have announced yet.</div>';
    return;
  }
  list.innerHTML = devices.map((d, i) =>
    `<div class="dev ${d.ip === selected ? 'sel' : ''}" onclick="selectDevice(lastDevices[${i}])">
       <span class="pfx">${d.prefix}</span>
       <span class="ip">${d.ip}</span>
       <span class="pos">${d.position.toFixed(0)}</span>
     </div>`).join('');
}

async function refreshDevices() {
  try {
    const j = await (await fetch('/devices')).json();
    lastDevices = j.devices;
    if (!selected && j.devices.length === 1) { selectDevice(j.devices[0]); return; }
    renderDevices(j.devices);
  } catch (e) { /* server gone */ }
}
setInterval(refreshDevices, 1000);
refreshDevices();

async function refresh() {
  try {
    const j = await (await fetch('/status')).json();
    const grid = $('statGrid');
    if (j.live) {
      $('dot').classList.add('live'); grid.classList.remove('stale');
      $('sPos').textContent  = j.position.toFixed(0);
      $('sVel').textContent  = j.velocity.toFixed(0);
      $('sCorr').textContent = (j.velocity * j.lead).toFixed(0);
      $('statAddr').textContent = j.address || '';
    } else {
      $('dot').classList.remove('live'); grid.classList.add('stale');
      $('statAddr').textContent = 'no status';
    }
  } catch (e) { /* leave the last reading on screen */ }
}
setInterval(refresh, 400);
refresh();

// The board sends status only to whoever last talked to it, so a quiet ping
// keeps the subscription alive while the panel sits idle.
setInterval(() => post('/control/ping', null, null, true), 4000);
post('/control/ping', null, null, true);

log('Ready. Changes apply and are saved as you make them.');
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
            defaults = json.dumps({"cport": CONTROL_PORT, "lead": 0.5, "window": 0.15})
            page = (
                PAGE.replace("__DEFAULTS__", defaults)
            )
            self._reply(200, page, "text/html; charset=utf-8")

        elif self.path == "/devices":
            now = time.monotonic()
            with _devices_lock:
                for ip in [k for k, v in _devices.items() if now - v["at"] > DEVICE_STALE_S]:
                    del _devices[ip]
                found = [
                    {
                        "ip": ip,
                        "prefix": v["prefix"],
                        "dest": v["dest"],
                        "destPort": v["destPort"],
                        "position": v["position"],
                        "lead": v["lead"],
                        "window": v["window"],
                    }
                    for ip, v in _devices.items()
                ]
            found.sort(key=lambda d: (d["prefix"], d["ip"]))
            self._reply(200, json.dumps({"devices": found}))

        elif self.path == "/status":
            with _status_lock:
                at, values, address = _status["at"], _status["values"], _status["address"]
            live = values is not None and (time.monotonic() - at) < STATUS_STALE_S
            if live:
                payload = {
                    "live": True,
                    "address": address,
                    "position": values[0],
                    "velocity": values[1],
                    "lead": values[2],
                    "window": values[3],
                }
            else:
                payload = {"live": False}
            self._reply(200, json.dumps(payload))

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
            port = int(req.get("port", CONTROL_PORT))
            path = str(req["path"])

            if req.get("value") is not None:
                message = osc_float(path, float(req["value"]))
            elif req.get("args") is not None:
                message = osc_args(path, req["args"])
            elif req.get("text") is not None:
                message = osc_string(path, str(req["text"]))
            else:
                message = osc_bare(path)

            send_osc(host, port, message)
            self._reply(200, '{"ok":true}')
        except Exception as exc:  # noqa: BLE001 - surface anything to the UI
            self._reply(200, json.dumps({"ok": False, "error": str(exc)}))

    def log_message(self, fmt, *args):
        pass  # the UI has its own activity log


def main():
    threading.Thread(target=status_listener, daemon=True).start()
    threading.Thread(target=announce_listener, daemon=True).start()

    # Threading matters: browsers hold keep-alive connections open, and a
    # single-threaded server blocks on one of those until it times out,
    # freezing the whole UI.
    for port in range(UI_PORT, UI_PORT + 20):
        try:
            server = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            server.daemon_threads = True
            break
        except OSError:
            continue
    else:
        print("Could not find a free local port for the UI.")
        return 1

    url = "http://127.0.0.1:%d/" % port
    print("Encoder Tuner running at %s" % url)
    print("Listening for boards on port %d; commands go to port %d"
          % (ANNOUNCE_PORT, CONTROL_PORT))
    print("Press Ctrl-C to quit.")
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nBye.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
