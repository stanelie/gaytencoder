"""
ONE-TIME UTILITY - re-zeroes the encoder's absolute position counter.

IMPORTANT: physically retract the rope fully (or move it to whatever position
you want to become "0") BEFORE running this. It tells the encoder "take your
current position as the zero point" (Modbus register 0x0008), which is a
real recalibration, not a reversible software setting.

Why you need this: this is a multi-turn absolute encoder - its position is
(turns x resolution + single-turn value). After enough back-and-forth motion,
minor mechanical backlash in a spring-loaded reel can let the turn count
drift from where it started. The counter itself stays internally consistent,
but "true zero" (rope fully retracted) can end up sitting at a large raw
count instead of near 0 - which is why you might see a large jump in the
readings right around full retraction even though nothing is actually wrong.
Resetting here re-anchors zero to wherever the rope physically is right now.

Usage:
  1. Physically retract the rope to the position you want to be "0".
  2. Copy this file to the CIRCUITPY drive as code.py and let it run.
  3. Check the Thonny shell for confirmation.
  4. Copy your normal streaming code.py back.
"""
import time

import board
import busio

UART_TX_PIN = board.IO43
UART_RX_PIN = board.IO44
SLAVE_ADDR = 0x01
BAUD = 115200

uart = busio.UART(
    tx=UART_TX_PIN,
    rx=UART_RX_PIN,
    baudrate=BAUD,
    bits=8,
    parity=None,
    stop=1,
    timeout=0.05,
)


def modbus_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def with_crc(frame):
    crc = modbus_crc16(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_read_holding_registers(addr, start_reg, num_regs):
    return with_crc(
        bytes([addr, 0x03, (start_reg >> 8) & 0xFF, start_reg & 0xFF, (num_regs >> 8) & 0xFF, num_regs & 0xFF])
    )


def build_write_single_register(addr, reg, value):
    return with_crc(
        bytes([addr, 0x06, (reg >> 8) & 0xFF, reg & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    )


def _flush_input():
    while uart.in_waiting:
        uart.read(uart.in_waiting)


def read_position():
    request = build_read_holding_registers(SLAVE_ADDR, 0x0000, 2)
    _flush_input()
    uart.write(request)
    echo = uart.read(len(request))
    if echo is None or len(echo) < len(request) or echo != request:
        return None
    response = uart.read(9)
    if response is None or len(response) < 9:
        return None
    if response[0] != SLAVE_ADDR or response[1] != 0x03 or response[2] != 4:
        return None
    crc_received = response[7] | (response[8] << 8)
    if crc_received != modbus_crc16(response[:7]):
        return None
    return (response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]


def write_register_confirmed(reg, value):
    request = build_write_single_register(SLAVE_ADDR, reg, value)
    _flush_input()
    uart.write(request)
    echo = uart.read(len(request))
    reply = uart.read(len(request))
    return echo == request and reply == request


before = read_position()
if before is None:
    raise RuntimeError("Could not read encoder - check wiring/baud/address.")
print("Position before reset:", before)

print("Resetting zero point to current position...")
if not write_register_confirmed(0x0008, 0x0001):
    raise RuntimeError("Zero-reset write failed/unconfirmed")

time.sleep(0.05)
after = read_position()
print("Position after reset:", after)

if after is not None and after < 50:
    print("DONE. Zero point reset successfully.")
else:
    print("Reset command was acknowledged, but the read-back value looks")
    print("unexpected - double check before trusting it.")
