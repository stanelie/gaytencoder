# gaytencoder

Streams the absolute position of a Briterencoder RS485 rope (linear
displacement) sensor over OSC/UDP, using a Waveshare ESP32-S3-ETH running
CircuitPython.

The board polls the encoder over RS485 with Modbus-RTU and sends an OSC
message on every change, over wired Ethernet. Measured throughput:
**~200 OSC messages/second** sustained while the rope is moving.

---

## Hardware

| Part | Detail |
|---|---|
| Board | Waveshare ESP32-S3-ETH (onboard W5500 Ethernet) |
| Firmware | CircuitPython 10.3.0 |
| Sensor | Briterencoder RS485 rope encoder, 4096 counts/turn, 100-turn range |
| Transceiver | MAX1348x-family auto-direction RS485 |

### Wiring

| Signal | ESP32-S3 pin | Notes |
|---|---|---|
| RS485 TX | `IO43` | to transceiver `DI` |
| RS485 RX | `IO44` | from transceiver `RO`, through a resistor divider |

The MAX1348x auto-senses direction, so there is no DE/RE control pin. It also
keeps its receiver enabled while transmitting, so the ESP32 hears its own
request echoed back before the encoder's reply — the code accounts for this.

W5500 Ethernet is on the board's fixed SPI pins: `CLK=IO13`, `MOSI=IO11`,
`MISO=IO12`, `CS=IO14`, `RST=IO9`.

---

## Setup

1. Flash CircuitPython 10.3.0 for `waveshare_esp32_s3_eth`.
2. Copy these onto the `CIRCUITPY` drive:
   - `code.py`
   - `config.py`
   - `lib/adafruit_wiznet5k/`
   - `lib/adafruit_bus_device/`
   - `lib/adafruit_ticks.mpy`
3. Edit `config.py` (at minimum `OSC_HOST`).
4. Plug in Ethernet and power. The console prints the acquired IP and a
   once-per-second rate line.

The libraries come from the Adafruit CircuitPython bundle
(`adafruit_wiznet5k`, `adafruit_bus_device`, `adafruit_ticks`) and are used
unmodified — all optimisations live in `code.py`.

### First run

The encoder ships at 9600 baud. On first boot `code.py` detects the current
baud rate, switches the encoder to 115200 (its maximum), and confirms the
change. That setting persists in the encoder's NVM, so later boots detect
115200 immediately and skip the step. A factory-fresh replacement encoder is
handled automatically.

---

## Configuration (`config.py`)

| Setting | Purpose |
|---|---|
| `OSC_HOST` / `OSC_PORT` | where to send OSC |
| `OSC_PREFIX` | this board's identity, e.g. `/encoder1` (default only; NVM wins) |
| `MODBUS_SLAVE_ADDR` | which encoder this board talks to (factory default `1`) |
| `HOSTNAME` | DHCP hostname; give each board a unique one |
| `USE_DHCP` | `False` selects a static IP derived from the board's MAC |
| `ENCODER_MODULUS` | counter span, `4096 * 100` — see [FINDINGS.md](FINDINGS.md) |
| `LEAD_TIME_S` | video-pipeline latency to compensate (default only; NVM wins) |
| `VELOCITY_WINDOW_S` | velocity averaging window (default only; NVM wins) |
| `OSC_LISTEN_PORT` | port the board listens on for tuning messages |
| `OSC_STATUS_PORT` | port the board sends diagnostics back to |
| `OSC_ANNOUNCE_PORT` / `OSC_ANNOUNCE_S` | discovery beacon port and interval (`0` disables) |

### Second board

The design uses one ESP32 per encoder, each with its own RS485 link and its
own Ethernet connection, rather than sharing one RS485 bus.

`code.py` is byte-for-byte identical on both boards. The only thing that has
to differ is the OSC prefix, and that can be set over the network — point the
tuner at the second board's IP, type `/encoder2`, press **Set prefix**, then
**Save**. Nothing needs editing or redeploying.

If you would rather have it right from first boot, set it in `config.py`
before copying:

```python
HOSTNAME = "encoder-bridge-2"   # DHCP hostname, unrelated to OSC
OSC_PREFIX = "/encoder2"        # default only; a saved value overrides it
MODBUS_SLAVE_ADDR = 1           # can stay 1 - separate buses, no collision
```

The control addresses do not change between boards — they are always
`/control/...`, and the IP distinguishes the two. Each board stores its own
tuning, so they can have different lead times if their video paths differ.

Keeping the encoders on separate buses means a failed encoder can be swapped
for a factory-fresh one without re-addressing it, each encoder gets the full
poll rate instead of halving it, and a fault on one board cannot take down the
other. The trade-off is that the two streams are not sample-synchronised —
fine here, since the two positions are consumed independently.

---

## Output

One OSC message per changed reading:

```
/encoder1/position  ,f  <float32 counts, latency-compensated>
```

Position is in encoder counts, ~11.8 counts/mm on this rig (measured: 3545
counts over 300 mm). Values are signed and continuous through zero — see
[FINDINGS.md](FINDINGS.md) for why that matters.

That is the **only** thing sent to `OSC_HOST` — the stream Millumin sees
contains nothing else.

There is also a diagnostic message, but it is deliberately kept off that
stream. It goes to whichever machine last sent a control message, on
`OSC_STATUS_PORT` — in practice the tuner app, which subscribes simply by
being used:

```
/encoder1/status  ,ffff  <position counts> <velocity counts/s> <lead s> <window s>
```

With no tuner running the board sends no status at all, so during a show it
costs nothing.

Each board also broadcasts a discovery beacon so the tuner can list what is on
the network without anyone knowing an IP address:

```
/encoder/announce  ,sfff  <prefix> <position> <lead> <window>
```

Sent to `255.255.255.255` every `OSC_ANNOUNCE_S` seconds. The limited
broadcast address is deliberate — the W5500 ARPs for ordinary destinations,
so a subnet-directed broadcast like `10.8.0.255` would ARP for an address
nothing answers to and the sends would silently fail.

---

## Latency compensation

The application is a moving LED screen acting as a physical "window" onto a
virtual background rendered in Millumin. For the illusion to hold, the image
must correspond to where the screen actually *is*. The video chain lags the
physical screen by roughly half a second, so without compensation the
background is always showing where the screen *was*.

The board therefore extrapolates:

```
predicted = position + velocity × LEAD_TIME_S
```

At rest the correction is zero and the position is exact; the faster the
screen moves, the further ahead it is projected.

**This is only exact at constant velocity.** During acceleration it is wrong
by roughly `Δvelocity × lead`. In practice the screen is ~300 lb and pushed by
hand, so it cannot change speed quickly and the ramps are gentle.

**Compensation is not a substitute for fixing the latency.** Half a second is
~30 frames; if any of it can be removed from the video chain, that is worth
more than predicting around it, because prediction can only guess.

### Tuning it live

`LEAD_TIME_S` has to be dialled in against the real pipeline, so both it and
the velocity window are adjustable over OSC at runtime — no redeploy, and no
USB access to a board mounted on a moving truck. Send to the board's IP on
port `9001`:

| Message | Effect |
|---|---|
| `/control/lead <float>` | seconds of latency to compensate (0–5) |
| `/control/window <float>` | velocity averaging window, seconds (0.005–2) |
| `/control/prefix <string>` | this board's outgoing OSC identity, e.g. `/encoder2` |
| `/control/dest <string> [<int>]` | where position messages go: IP, optionally port |
| `/control/ping` | keepalive; holds the status subscription open |
| `/control/save` | force an immediate write to NVM |

**Changes are saved automatically.** Anything accepted is written to NVM
within a second, batched so that a flurry of adjustments is still only one
flash write. `/control/save` remains for scripted use but is not needed.

These addresses are **fixed and carry no prefix**. The IP address already says
which board you are talking to, and if the control addresses depended on the
prefix you would have to know a board's current prefix in order to change it.

Out-of-range values and malformed prefixes are rejected and logged rather than
applied, and an unrecognised address says so on the console. Saved values live
in the board's NVM (not the filesystem, which the board cannot write while USB
is attached) and are restored on boot; `config.py` only supplies defaults for a
board that has never been tuned.

#### The tuner app (macOS / anything with Python 3)

`tools/osc_tuner.py` is a control panel for everything a board can be told.
It takes no arguments — boards are discovered, not addressed:

```bash
python3 tools/osc_tuner.py
```

From the repository root, or with the full path from anywhere:

```bash
python3 ~/gaytencoder/tools/osc_tuner.py
```

On macOS, `tools/Encoder Tuner.command` is a double-clickable launcher; make
it executable once with `chmod +x "tools/Encoder Tuner.command"`. It opens the
panel in your browser automatically.

**Boards are discovered automatically.** Each one broadcasts a beacon every
two seconds, and the panel lists what it hears — prefix, IP and live position.
Click a row to select it, and its current settings are filled in from the
board itself. With a single board on the network it selects itself. If two are
indistinguishable on paper, pull one of the ropes and watch which row's
position moves.

Everything fits on one screen:

| Control | Behaviour |
|---|---|
| Lead / window sliders | apply when you release the slider |
| Lead / window fields | type a value, apply on Enter |
| Sends as / to / port | apply on the **Apply** button |
| Status tiles | position, velocity and the resulting correction, live |

There is no Save button — every accepted change is written to the board's NVM
within a second.

The status readout appears only while the panel is open; a quiet ping keeps
that subscription alive. If discovery is blocked (some managed switches and
VLANs drop broadcast traffic) the list stays empty — that is the one case
where you would need to reach a board another way.

It uses only the Python standard library — no `pip install`, and no tkinter,
whose availability on macOS depends on how Python was installed. The UI is
served to your browser because browsers cannot send UDP themselves; the web
server binds to localhost only and nothing is exposed to the network. macOS
may ask for local-network permission the first time it sends.

#### Suggested procedure

1. Move the screen at a steady, representative speed.
2. Adjust `/control/lead` until the background stops sliding against the
   screen — too low and it lags behind the move, too high and it leads.
3. Stop the screen and watch for jitter or hunting in the still image. If
   present, raise `/control/window`; if the response feels sluggish when
   starting and stopping, lower it.
4. Nothing to confirm — each change is saved to the board as you make it.

`/encoder1/status` reports raw position, measured velocity and the current
tuning values throughout, which makes it much easier to see what the board
thinks is happening.

### Why the velocity window matters

Velocity is measured across a time window, never between adjacent samples.
At ~230 Hz, one count of quantisation (~0.085 mm) across a single ~4 ms poll
reads as ~20 mm/s of apparent velocity — which a 0.5 s lead turns into
~10 mm of position jitter while the screen is completely still. Measuring
across 150 ms instead divides that noise by roughly the ratio of the windows.

The window is the classic smoothing trade-off: longer is steadier but slower
to react, shorter is more responsive but noisier. Hence making it tunable
alongside the lead time.

---

## Utilities

Copy the one you need onto the board as `code.py`, run it, then restore the
real `code.py`.

### `zero_reset.py`

Sets the encoder's current physical position as zero (Modbus register
`0x0008`). Retract the rope to the position you want to be `0` **before**
running it. This is a real recalibration, not a reversible software setting.

### `address_config.py`

Scans every Modbus address and baud rate to find a **single** connected
encoder, reports what it found, then reassigns its address and baud rate.

Run with exactly one encoder on the bus. Modbus register `0x0004` (address)
is write-only — there is no "what is your address" query — so discovery is by
scan, which is only unambiguous with one device present. Not needed for the
one-encoder-per-board layout, but useful for identifying an unlabelled unit.

### `tools/test_velocity.py`

Checks the velocity estimator on a desktop Python, no hardware needed:

```bash
python3 tools/test_velocity.py
```

It extracts the class from `code.py` so the tests cannot drift from what runs
on the board. Worth running after touching anything in that class — it covers
the `ticks_ms` rollover, a path that otherwise only executes once every 6.2
days in production.

### `bench_spi.py`, `diag_send.py`

Diagnostics from the performance work. `bench_spi.py` measures per-SPI-call
overhead; `diag_send.py` sends known values through both the library path and
the optimised path to prove which one reaches the wire. Neither is deployed
in normal operation. See [FINDINGS.md](FINDINGS.md).

---

## Console output

```
Encoder responded at 115200 baud (value=-2)
Encoder ready @ address 1, 115200 baud
Ethernet link up.
IP address: 10.8.0.241
[rate] 212.4 polls/s, 206.4 sends/s (0 errors/s)
```

`polls/s` is the Modbus poll rate, `sends/s` the OSC rate (only changed
values are sent, so this drops to ~0 when the rope is still). A persistently
high `errors/s` means the encoder is not answering — check wiring, baud and
slave address.
