"""
ONE-TIME UTILITY - run with exactly ONE encoder connected to the RS485 bus.

Discovers a lone encoder's current Modbus address and baud rate (by
scanning, since address 0x0004 is write-only - there's no "read my own
address" register), then reassigns it to NEW_ADDRESS / TARGET_BAUD below
and confirms the change.

Usage:
  1. Disconnect all encoders except the one you want to configure.
  2. Set NEW_ADDRESS (and TARGET_BAUD if you want) below.
  3. Copy this file to the CIRCUITPY drive as code.py and let it run.
  4. Check the Thonny shell for the "DONE" confirmation line.
  5. Disconnect this unit, repeat for the next one with a different address.
  6. Copy your normal streaming code.py back once all units are configured.
"""
import time
import board
import busio

UART_TX_PIN = board.IO43
UART_RX_PIN = board.IO44

NEW_ADDRESS = 2       # <-- set the address you want this connected unit to have
TARGET_BAUD = 115200  # <-- set to None to leave the baud rate as found

BAUD_REGISTER_VALUES = {9600: 0, 19200: 1, 38400: 2, 57600: 3, 115200: 4}
CANDIDATE_BAUDS = (115200, 9600, 19200, 38400, 57600)
CANDIDATE_ADDRESSES = range(1, 128)

uart = busio.UART(
    tx=UART_TX_PIN,
    rx=UART_RX_PIN,
    baudrate=9600,
    bits=8,
    parity=None,
    stop=1,
    timeout=0.03,
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


def try_read_position(addr):
    request = build_read_holding_registers(addr, 0x0000, 2)
    _flush_input()
    uart.write(request)

    echo = uart.read(len(request))
    if echo is None or len(echo) < len(request) or echo != request:
        return None

    response = uart.read(9)
    if response is None or len(response) < 9:
        return None
    if response[0] != addr or response[1] != 0x03 or response[2] != 4:
        return None

    crc_received = response[7] | (response[8] << 8)
    if crc_received != modbus_crc16(response[:7]):
        return None

    return (response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]


def write_register_confirmed(addr, reg, value):
    request = build_write_single_register(addr, reg, value)
    _flush_input()
    uart.write(request)
    echo = uart.read(len(request))
    reply = uart.read(len(request))
    return echo == request and reply == request


def discover():
    for baud in CANDIDATE_BAUDS:
        uart.baudrate = baud
        for addr in CANDIDATE_ADDRESSES:
            value = try_read_position(addr)
            if value is not None:
                return addr, baud, value
    return None, None, None


print("Scanning for a lone encoder (all addresses x all baud rates)...")
print("This can take up to ~20 seconds.")
addr, baud, value = discover()

if addr is None:
    raise RuntimeError(
        "No encoder found at any address/baud. Check wiring, and make sure "
        "exactly ONE unit is connected to the bus right now."
    )

print("Found encoder: address=%d baud=%d current_value=%d" % (addr, baud, value))

target_baud = TARGET_BAUD if TARGET_BAUD else baud

if addr != NEW_ADDRESS:
    print("Reassigning address %d -> %d ..." % (addr, NEW_ADDRESS))
    if not write_register_confirmed(addr, 0x0004, NEW_ADDRESS):
        raise RuntimeError("Address write failed/unconfirmed")
    addr = NEW_ADDRESS
    print("Address changed to %d" % addr)
else:
    print("Address already %d, leaving as-is." % addr)

if target_baud != baud:
    print("Reassigning baud %d -> %d ..." % (baud, target_baud))
    if not write_register_confirmed(addr, 0x0005, BAUD_REGISTER_VALUES[target_baud]):
        raise RuntimeError("Baud write failed/unconfirmed")
    time.sleep(0.05)
    uart.baudrate = target_baud
    baud = target_baud
    print("Baud changed to %d" % baud)
else:
    print("Baud already %d, leaving as-is." % baud)

value = try_read_position(addr)
if value is None:
    raise RuntimeError("Could not confirm encoder after reconfiguration!")

print("DONE. Encoder is now at address=%d baud=%d (value=%d)" % (addr, baud, value))
print("Disconnect this unit and connect the next one (with a different NEW_ADDRESS) if needed.")
