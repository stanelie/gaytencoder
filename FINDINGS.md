# Findings

Notes from bringing this rig up. Two things here are non-obvious and cost
real debugging time: the encoder's counter **wraps**, and the CircuitPython
W5500 driver is **per-SPI-call bound**, not bandwidth bound.

---

## 1. The encoder counter wraps at 409600 — it is not a glitch

### Symptom

Pulling the rope in fully and back out produced this on the OSC side:

```
... 20, 11, 3, 409593, 409590, 409591, 409592, 409594, 409597, 409599, 2, 5, 8, ...
```

Huge values appearing for a few samples right at full retraction, then normal
values again. It looks exactly like data corruption.

### It isn't corruption

- The Modbus CRC on those frames is **valid**. They are genuine readings.
- `409600 = 4096 × 100` — counts-per-turn × the encoder's 100-turn multi-turn
  range.

The multi-turn counter is **modular**: it wraps `409599 → 0` in one direction
and `0 → 409599` in the other. A raw reading of `409598` means *2 counts below
zero*, not a jump across the counter's whole span.

It shows up constantly here because the encoder's zero happens to sit exactly
**on** the wrap point, so ordinary small movements at full rope retraction
cross it every cycle.

Note that `zero_reset.py` cannot fix this: setting zero at the rope's resting
position is what puts the wrap point there in the first place.

### Fix

Re-express the top half of the span as negative, in `try_read_position()`:

```python
if raw >= ENCODER_MODULUS // 2:
    raw -= ENCODER_MODULUS
```

Position becomes continuous through zero (`8 → -11 → -1 → 0 → 14`), with a
usable range of ±204800 counts (about ±17 m at ~11.8 counts/mm) — comfortably
beyond the 15 m rope, so real positions are never ambiguous.

Stateless, lossless, and it works for any zero calibration.

### What not to do

The first attempt was a **delta filter**: reject any poll-to-poll jump larger
than ~20000 counts as physically impossible. It appeared to work, but:

1. It **deadlocked.** Rejected readings never updated the reference value, so
   once the rope travelled further than the threshold from a stale reference,
   every subsequent reading looked impossible and the stream stopped
   permanently. Symptom: `267 errors/s` at `269 polls/s` — every poll rejected.
2. Even with a bounded rejection streak added as a safety valve, it was
   **throwing away ~230 ms of perfectly valid data on every zero crossing**,
   because it was "filtering" a legitimate wrap.

Verification after switching to unwrapping — 35 zero crossings captured, with
the rope deliberately worked back and forth across the boundary:

```
PACKETS: 871   MALFORMED: 0
MIN: -11   MAX: 149
WRAP SPIKES (|v|>200000): 0     LARGEST SINGLE JUMP: 38
ZERO CROSSINGS: 35   e.g. 8->-11  -1->0  14->-9  -1->0
```

Largest single-sample jump of 38 counts (~3 mm) — fully continuous.

---

## 2. OSC throughput: 45 → 201 messages/second

### The bottleneck is per-SPI-call overhead

`bench_spi.py` measures it directly (Waveshare ESP32-S3-ETH, CircuitPython
10.3.0, 8 MHz SPI):

| Operation | Cost |
|---|---|
| `with device:` only (bus lock + configure + CS) | **24.8 µs** |
| read, split 3-byte header (4 busio calls) | 272.5 µs |
| read, combined header (2 busio calls) | 185.0 µs |
| read via `write_readinto` (1 busio call) | **118.8 µs** |
| single `write`, 3 bytes (1 call) | 112.3 µs |
| single `write`, 31 bytes (1 call) | **145.7 µs** |
| `eth._read(len=1)` (library) | 468.6 µs |
| `eth._read_sntx_wr()` (library, 2 × `_read`) | 1175.7 µs |

Two conclusions:

- **~90 µs fixed overhead per `busio` call**, but only ~1 µs per extra payload
  byte — a 3-byte write costs 112 µs and a 31-byte write 146 µs. Cost is
  **per-call, not per-byte**.
- The library adds **~196 µs of Python overhead per register op** on top of
  the raw SPI calls (`.to_bytes()` and `bytearray` allocations).

### Why the stock path was slow

Sending one 28-byte UDP packet through `adafruit_wiznet5k` costs **32 busio
SPI calls**:

| Step | busio calls |
|---|---|
| `_read_sntx_fsr` (2 × `_read`, 4 calls each) | 8 |
| `_read_sntx_wr` (2 × `_read`) | 8 |
| `_chip_socket_write` (1 × `_write`) | 4 |
| `_write_sntx_wr` (2 × `_write`) | 8 |
| `_write_sncr` (1 × `_write`) | 4 |
| **total** | **32** |

Because `_chip_read`/`_chip_write` emit the 3-byte register header as *three
separate* `device.write()` calls, and `_read_two_byte_sock_reg` reads a 16-bit
register as two independent single-byte transactions.

### What actually helped

| Change | Send cost |
|---|---|
| baseline: `sendto()` per packet (reconnects the socket every call) | 17.3 ms |
| `connect()` once, then `send()` | ~10 ms |
| bypass the forced `gc.collect()` inside `send()` | 7.6 ms |
| single-read TX-free-size instead of the defensive triple-read | 5.5 ms |
| **single-transaction register ops (32 calls → 5)** | **0.93 ms** |

Plus, off the send path:

- `while eth.link_status:` was a full ~470 µs register read **every loop
  iteration**. Amortised to once per second.
- The OSC message was fully rebuilt per packet (encode, pad, `struct.pack`,
  concatenate) though only the trailing int32 ever changes. Now prebuilt once
  and patched in place with `struct.pack_into`.
- The Modbus echo and reply were two blocking `uart.read()` calls; now a
  single 17-byte read.

Result: **99 Hz → 201 Hz** end-to-end, verified by decoding packets off the
wire (2012 packets in 10 s, 0 malformed).

### Remaining headroom is small

The loop is now poll-bound: ~3.7 ms of Modbus poll against ~0.93 ms of send.
Of that 3.7 ms, ~1.5 ms is unavoidable wire time (17 bytes at the encoder's
maximum 115200 baud) and the rest is the encoder's own response latency. ~250 Hz
is not reachable without different sensor hardware.

---

## 3. W5500 gotcha: one CS-low frame = one operation

Collapsing the split header writes into a single `write()` is safe and is
where the speedup comes from. Batching **several register operations** into
one `with device:` block is **not**.

The W5500's Variable Data Length Mode consumes the address + control bytes
**once per CS-low frame**; everything after them is payload. Putting five
register operations under one `with` meant only the first was interpreted as
a register access — the pointer update and the SEND command were silently
written *into the TX buffer as data*.

The failure is nasty because it is completely silent: every call returns
successfully, the send counter increments, `0 errors/s` — and no packet ever
reaches the wire.

```
SNTX_WR after fast send: 28 (expected 56)     # pointer never advanced
```

`diag_send.py` catches this by sending distinguishable values through the
library path and the fast path and checking which arrive. Each operation gets
its own `with device:` block; at 24.8 µs it is nowhere near worth batching.

---

## 4. Things that did not help

- **Caching the SPI `configure()` call** in `SPIDevice.__enter__`. No
  measurable effect — `with device:` is only 24.8 µs total, so reconfiguration
  was never the problem.
- **Raising the SPI clock.** The floor is per-call software overhead, not bit
  clocking; a few bytes at 8 MHz is already microseconds.
- **The encoder's "automatic return" mode** (register `0x0006`). The datasheet
  warns against intervals under 20 ms, and 20 ms is 50 Hz — slower than simply
  polling it at ~220 Hz.
- **A faster encoder baud rate.** 115200 is its maximum.

### Do not do this

Skipping the TX-free-space check *and* caching the TX write pointer in
software (to cut 5 SPI operations to 3) caused a **CircuitPython hard fault**
— "memory access or instruction error", safe mode, physical reset button
required. The gain would have been ~0.2 ms. Not worth it; the free-space check
also provides useful back-pressure.

---

## 5. Encoder reference

Briterencoder RS485 rope sensor, Modbus-RTU. Factory defaults: **9600 baud,
slave address 1, query mode**.

| Register | Meaning | Notes |
|---|---|---|
| `0x0000`–`0x0001` | position, 32-bit | read (`0x03`); wraps at 409600 |
| `0x0004` | Modbus address | **write-only** — discovery requires a scan |
| `0x0005` | baud rate | `0`=9600 … `4`=115200 |
| `0x0006` | mode | `0`=query, `1`=auto-return |
| `0x0008` | set zero | write `1` = current position becomes zero |

Measured scaling on this rig: **~11.8 counts/mm** (3545 counts over 300 mm),
so the 15 m rope spans ~177,000 counts — well inside both the 100-turn range
and the ±204800 unwrapped window.
