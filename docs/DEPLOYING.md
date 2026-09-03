# Deploying a board

Everything needed to take a bare Waveshare ESP32-S3-ETH to a working encoder
bridge. No toolchain, no `pip install`, no internet beyond the CircuitPython
download in step 1.

Download `gaytencoder-<version>.zip` from the
[releases page](https://github.com/stanelie/gaytencoder/releases) and unpack
it. You get:

```
gaytencoder-1.0/
  CIRCUITPY/     copy the CONTENTS of this onto the board's drive
  tuner/         runs on your Mac or PC, not on the board
  utilities/     one-off jobs, run by temporarily replacing code.py
  MANIFEST.txt   sha256 of every file, to verify a copy landed intact
```

---

## 1. Flash CircuitPython

Download **CircuitPython 10.3.0 for `waveshare_esp32_s3_eth`** from
[circuitpython.org/board/waveshare_esp32_s3_eth](https://circuitpython.org/board/waveshare_esp32_s3_eth/).

Double-tap the board's **RESET** button. A drive called `ESP32S3` (or similar)
appears. Drop the `.uf2` file onto it. The board reboots and a drive called
**`CIRCUITPY`** appears instead.

This is the only step that needs anything downloaded. Everything else is here.

## 2. Copy the files

Copy the **contents** of `CIRCUITPY/` onto the `CIRCUITPY` drive, so the drive
ends up with:

```
CIRCUITPY/
  code.py
  config.py
  lib/adafruit_wiznet5k/
  lib/adafruit_bus_device/
  lib/adafruit_ticks.py
```

The board restarts on its own as soon as the files land.

The libraries are stock Adafruit, unmodified — every optimisation lives in
`code.py`. They are included so a board can be built without hunting through
the Adafruit bundle for matching versions.

## 3. Wire it

| Signal | Board pin | To |
|---|---|---|
| RS485 TX | `IO43` | transceiver `DI` |
| RS485 RX | `IO44` | transceiver `RO`, via the resistor divider |

Plug in Ethernet. The encoder needs its own 5–24V supply; PoE powers the board,
not the sensor.

## 4. Check the LED

| Colour | Meaning |
|---|---|
| red, blinking ~2/sec | encoder not answering — check RS485 wiring and sensor power |
| red, steady | booting, or no network link |
| blue | network up, not streaming yet |
| green | running |
| blue flicker over green | position is moving |
| red blink, 0.2s | a setting was saved |

> The **RJ45 link LED on this board reads backwards — dark means the link is
> up.** That is a wiring choice on the board and cannot be fixed in software.
> Trust the RGB LED instead.

A board whose encoder is missing still boots, still joins the network, and
still accepts configuration — it just cannot stream, and says so. It picks up
on its own once the encoder answers; no reset needed.

## 5. Configure it

```bash
python3 tuner/osc_tuner.py
```

On macOS you can double-click `tuner/Encoder Tuner.command` instead (once:
`chmod +x "tuner/Encoder Tuner.command"`).

It finds boards by their broadcast beacon, so there is no IP to look up. Click
one and set:

- **Sends as** — its OSC identity, e.g. `/encoder1`. Give each board a
  different one.
- **To** — the machine running Millumin, and the port.
- **Lead** and **window** — the latency compensation, tuned against the real
  video pipeline. See the main [README](../README.md).

Everything is saved to the board automatically and survives a power cycle.

**Two boards need nothing different in these files.** Deploy the identical
`CIRCUITPY/` contents to both and give them different prefixes from the tuner.

## 6. Confirm it works

The board sends, on every change:

```
<prefix>/position  ,i  <int32 counts>
```

Point an OSC monitor at the destination you set. Pull the rope; the numbers
should move.

---

## Utilities

Run by copying over `code.py` temporarily, then restoring it afterwards. Only
`code.py` runs automatically, so nothing here executes on its own.

**`utilities/zero_reset.py`** — makes the rope's current position the zero
point. Retract the rope first. This is a real recalibration of the sensor.

**`utilities/address_config.py`** — finds a lone encoder by scanning every
Modbus address and baud rate, then reassigns it. Only needed if a replacement
encoder turns up on a non-default address. Run it with exactly one encoder
connected, since discovery works by scanning.

---

## Verifying a copy

FAT drives can truncate a file without complaining. To check what landed:

Compare against `MANIFEST.txt`, or just check sizes — a short
`adafruit_wiznet5k.py` (should be ~56 KB) is the usual casualty.

## Building this archive

From a clone of the repository:

```bash
python3 tools/build_release.py
```

It writes `dist/gaytencoder-<version>.zip`, taking the version from `VERSION`
in `code.py` so the archive name and the string the board prints at boot
cannot disagree. The archive is not committed — it belongs on the releases
page, and a build artifact in the repository goes stale as soon as anything
changes.
