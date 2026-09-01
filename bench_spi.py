"""
BENCHMARK - measures where SPI time actually goes on the W5500 path.

Read-only: every operation here reads harmless registers, nothing is written
to the chip, so this is safe to run. Copy to CIRCUITPY as code.py, read the
numbers in the console, then restore the normal code.py.

Goal: separate the per-`with self._device` overhead (bus lock + configure +
CS toggle) from the per-busio-call overhead (each .write()/.readinto()).
That tells us whether collapsing the library's split header writes into
single transactions is actually worth doing.
"""
import time

import board
import busio
import digitalio
from adafruit_wiznet5k.adafruit_wiznet5k import WIZNET5K

cs = digitalio.DigitalInOut(board.IO14)
rst = digitalio.DigitalInOut(board.IO9)
spi = busio.SPI(board.IO13, MOSI=board.IO11, MISO=board.IO12)

print("Initializing W5500 (no DHCP)...")
eth = WIZNET5K(spi, cs, reset=rst, is_dhcp=False, debug=False)
print("chip:", eth._chip_type)

dev = eth._device
N = 300

# W5500 common register block: VERSIONR @ 0x0039, control byte 0x00.
# Reading it is harmless and returns a constant (0x04).
ADDR = 0x0039
CB = 0x00
HDR3 = bytes([(ADDR >> 8) & 0xFF, ADDR & 0xFF, CB])


def bench(label, fn, n=N):
    # one warmup, then timed run
    fn()
    start = time.monotonic_ns()
    for _ in range(n):
        fn()
    total_us = (time.monotonic_ns() - start) / 1000.0
    print("%-46s %8.1f us/op" % (label, total_us / n))


# --- 0. Pure `with device` overhead: lock + configure + CS, no SPI traffic ---
def t_with_only():
    with dev:
        pass


# --- 1. Library's current 4-call read (3 header writes + 1 readinto) ---
buf1 = bytearray(1)


def t_split_header_read():
    with dev as bus:
        bus.write((ADDR >> 8).to_bytes(1, "big"))
        bus.write((ADDR & 0xFF).to_bytes(1, "big"))
        bus.write(CB.to_bytes(1, "big"))
        bus.readinto(buf1)


# --- 2. Combined header, 2 calls (1 header write + 1 readinto) ---
def t_combined_header_read():
    with dev as bus:
        bus.write(HDR3)
        bus.readinto(buf1)


# --- 3. Single full-duplex call (write_readinto), 1 call ---
out4 = bytearray(HDR3 + b"\x00")
in4 = bytearray(4)


def t_write_readinto():
    with dev as bus:
        bus.write_readinto(out4, in4)


# --- 4. Does payload SIZE matter, or only call count? 31-byte write ---
hdr_plus_28 = bytes(HDR3) + bytes(28)


def t_one_big_write():
    with dev as bus:
        bus.write(hdr_plus_28)


def t_one_small_write():
    with dev as bus:
        bus.write(HDR3)


# --- 5. Library-level calls, as actually used in the hot path ---
def t_lib_read_1():
    eth._read(ADDR, CB, 1)


def t_lib_read_2():
    eth._read(ADDR, CB, 2)


def t_lib_read_sntx_wr():
    eth._read_sntx_wr(0)


print("\n--- per-operation cost ---")
bench("0. `with device:` only (lock+configure+CS)", t_with_only)
bench("1. read, split 3-byte header (4 calls)", t_split_header_read)
bench("2. read, combined header (2 calls)", t_combined_header_read)
bench("3. read, write_readinto (1 call)", t_write_readinto)
bench("4. single write, 3 bytes (1 call)", t_one_small_write)
bench("5. single write, 31 bytes (1 call)", t_one_big_write)
bench("6. eth._read(len=1)  [library]", t_lib_read_1)
bench("7. eth._read(len=2)  [library]", t_lib_read_2)
bench("8. eth._read_sntx_wr() [2x _read]", t_lib_read_sntx_wr)

print("\nIf 4 and 5 are nearly equal, cost is per-CALL, not per-byte:")
print("collapsing split writes into one transaction is the win.")
print("\nBenchmark done - restore your normal code.py.")
