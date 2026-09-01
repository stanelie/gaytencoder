"""
DIAGNOSTIC - isolates why the hand-rolled W5500 send produces no packet.

Compares the library's register reads against the single-transaction reads,
then sends three UDP packets with distinguishable values:
    111 -> via the library's sock.send()      (known-good reference)
    222 -> via the fast single-transaction path (under test)
    333 -> via the library again              (proves the socket still works)

Run a UDP listener on the OSC port while this executes. Whichever values
arrive tell us exactly which path is broken.
"""

import struct
import time

import board
import busio
import digitalio
import adafruit_wiznet5k.adafruit_wiznet5k_socketpool as socketpool
from adafruit_wiznet5k.adafruit_wiznet5k import WIZNET5K

from config import USE_DHCP, HOSTNAME, OSC_HOST, OSC_PORT, OSC_ADDRESS

cs = digitalio.DigitalInOut(board.IO14)
rst = digitalio.DigitalInOut(board.IO9)
spi = busio.SPI(board.IO13, MOSI=board.IO11, MISO=board.IO12)


def _osc_pad(data):
    data = data + b"\x00"
    return data + b"\x00" * ((-len(data)) % 4)


def osc_message_int(address, value):
    return _osc_pad(address.encode("utf-8")) + _osc_pad(b",i") + struct.pack(">i", value)


print("Connecting...")
eth = WIZNET5K(spi, cs, reset=rst, is_dhcp=USE_DHCP, hostname=HOSTNAME, debug=False)
print("link:", eth.link_status, "ip:", eth.pretty_ip(eth.ip_address))

pool = socketpool.SocketPool(eth)
sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
sock.settimeout(0)
sock.connect((OSC_HOST, OSC_PORT))

iface = sock._interface
sn = sock._socknum
dev = iface._device

print("\nsocknum =", sn)
print("chip    =", iface._chip_type)

reg_read = (sn << 5) + 0x08
print("reg_read ctrl = 0x%02X" % reg_read)

# --- compare library reads vs single-transaction reads ---
fsr_out = bytearray([0x00, 0x20, reg_read, 0x00, 0x00])
fsr_in = bytearray(5)
wrp_out = bytearray([0x00, 0x24, reg_read, 0x00, 0x00])
wrp_in = bytearray(5)

lib_fsr = iface._read_sntx_fsr(sn)
lib_wr = iface._read_sntx_wr(sn)

with dev as bus:
    bus.write_readinto(fsr_out, fsr_in)
with dev as bus:
    bus.write_readinto(wrp_out, wrp_in)

fast_fsr = (fsr_in[3] << 8) | fsr_in[4]
fast_wr = (wrp_in[3] << 8) | wrp_in[4]

print("\n            library   fast(1txn)   raw_in bytes")
print("SNTX_FSR    %6d   %8d   %r" % (lib_fsr, fast_fsr, bytes(fsr_in)))
print("SNTX_WR     %6d   %8d   %r" % (lib_wr, fast_wr, bytes(wrp_in)))
if lib_fsr != fast_fsr or lib_wr != fast_wr:
    print(">>> MISMATCH: the single-transaction read is not returning the")
    print(">>> same value as the library. Full-duplex framing is the suspect.")
else:
    print(">>> reads agree")

# --- three sends, distinguishable values ---
print("\nSending 111 via library sock.send() ...")
sock.send(osc_message_int(OSC_ADDRESS, 111))
time.sleep(0.4)

print("Sending 222 via fast single-transaction path ...")
msg = osc_message_int(OSC_ADDRESS, 222)
n = len(msg)
tx_ctrl = (sn << 5) + 0x14
reg_write = (sn << 5) + 0x0C
tx_base = sn * 0x800 + 0x8000

data_buf = bytearray(3) + msg
data_buf[2] = tx_ctrl

with dev as bus:
    bus.write_readinto(wrp_out, wrp_in)
pointer = (wrp_in[3] << 8) | wrp_in[4]
addr = tx_base + (pointer & 0x7FF)
data_buf[0] = (addr >> 8) & 0xFF
data_buf[1] = addr & 0xFF
print("  pointer=%d addr=0x%04X hdr=%r" % (pointer, addr, bytes(data_buf[:3])))
with dev as bus:
    bus.write(data_buf, end=3 + n)
newptr = (pointer + n) & 0xFFFF
with dev as bus:
    bus.write(bytearray([0x00, 0x24, reg_write, (newptr >> 8) & 0xFF, newptr & 0xFF]))
with dev as bus:
    bus.write(bytearray([0x00, 0x01, reg_write, 0x20]))

time.sleep(0.4)
print("  SNTX_WR after fast send:", iface._read_sntx_wr(sn), "(expected %d)" % newptr)

print("\nSending 333 via library sock.send() ...")
sock.send(osc_message_int(OSC_ADDRESS, 333))
time.sleep(0.4)

print("\nDone. Check which values arrived at the listener.")
