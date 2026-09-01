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
| `OSC_ADDRESS` | OSC address pattern, e.g. `/encoder1/position` |
| `MODBUS_SLAVE_ADDR` | which encoder this board talks to (factory default `1`) |
| `HOSTNAME` | DHCP hostname; give each board a unique one |
| `USE_DHCP` | `False` selects a static IP derived from the board's MAC |
| `ENCODER_MODULUS` | counter span, `4096 * 100` — see [FINDINGS.md](FINDINGS.md) |

### Second board

The design uses one ESP32 per encoder, each with its own RS485 link and its
own Ethernet connection, rather than sharing one RS485 bus. Only three values
need changing for board 2:

```python
HOSTNAME = "encoder-bridge-2"
OSC_ADDRESS = "/encoder2/position"
MODBUS_SLAVE_ADDR = 1          # can stay 1 - separate buses, no collision
```

`code.py` itself is identical on both boards.

Keeping the encoders on separate buses means a failed encoder can be swapped
for a factory-fresh one without re-addressing it, each encoder gets the full
poll rate instead of halving it, and a fault on one board cannot take down the
other. The trade-off is that the two streams are not sample-synchronised —
fine here, since the two positions are consumed independently.

---

## Output

One OSC message per changed reading:

```
/encoder1/position  ,i  <int32 counts>
```

Position is in raw encoder counts, ~11.8 counts/mm on this rig (measured:
3545 counts over 300 mm). Values are signed and continuous through zero —
see [FINDINGS.md](FINDINGS.md) for why that matters.

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
