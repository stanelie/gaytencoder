"""
Poll a Briterencoder RS485 linear displacement sensor over Modbus-RTU and
send its position live over OSC/UDP via the onboard W5500 Ethernet.

RS485 wiring:
  ESP32-S3 IO43 (TX) -> MAX1348x DI
  ESP32-S3 IO44 (RX) <- MAX1348x RO (through the resistor divider)
The MAX1348x auto-senses direction, so no DE/RE control pin is needed.

Ethernet: onboard W5500 on the Waveshare ESP32-S3-ETH (SPI: CLK=IO13,
MOSI=IO11, MISO=IO12, CS=IO14, RST=IO9).

See config.py for network and OSC destination settings.
"""

import gc
import struct
import time

import board
import busio
import digitalio
import ipaddress
import adafruit_wiznet5k.adafruit_wiznet5k_socketpool as socketpool
from adafruit_wiznet5k.adafruit_wiznet5k import WIZNET5K

from config import (
    USE_DHCP,
    HOSTNAME,
    STATIC_NETMASK,
    STATIC_GATEWAY,
    OSC_HOST,
    OSC_PORT,
    OSC_ADDRESS,
    MODBUS_SLAVE_ADDR,
    ENCODER_MODULUS,
)

_HALF_MODULUS = ENCODER_MODULUS // 2

# ---------------------------------------------------------------------------
# RS485 / Modbus-RTU encoder reading
# ---------------------------------------------------------------------------

UART_TX_PIN = board.IO43
UART_RX_PIN = board.IO44

BAUD_REGISTER_VALUES = {9600: 0, 19200: 1, 38400: 2, 57600: 3, 115200: 4}
TARGET_BAUD = 115200

uart = busio.UART(
    tx=UART_TX_PIN,
    rx=UART_RX_PIN,
    baudrate=9600,
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
    return with_crc(bytes([addr, 0x06, (reg >> 8) & 0xFF, reg & 0xFF, (value >> 8) & 0xFF, value & 0xFF]))


READ_POSITION_REQUEST = build_read_holding_registers(MODBUS_SLAVE_ADDR, 0x0000, 2)
_REQ_LEN = len(READ_POSITION_REQUEST)  # 8
_REPLY_LEN = 9  # addr(1) func(1) bytecount(1) data(4) crc(2)
_FRAME_LEN = _REQ_LEN + _REPLY_LEN  # 17


def _flush_input():
    while uart.in_waiting:
        uart.read(uart.in_waiting)


def try_read_position():
    _flush_input()
    uart.write(READ_POSITION_REQUEST)

    # The auto-direction transceiver echoes our own request back before the
    # encoder's reply arrives, so both land in the same RX stream. Reading
    # all 17 bytes in one blocking call costs one call's overhead instead of
    # two, and the echo still gets verified below.
    frame = uart.read(_FRAME_LEN)
    if frame is None or len(frame) < _FRAME_LEN:
        return None, "short frame: %r" % (frame,)
    if frame[:_REQ_LEN] != READ_POSITION_REQUEST:
        return None, "bad echo (wrong baud or no wiring)"

    response = frame[_REQ_LEN:]
    if response[0] != MODBUS_SLAVE_ADDR or response[1] != 0x03 or response[2] != 4:
        return None, "unexpected header: %r" % (response,)

    crc_received = response[7] | (response[8] << 8)
    if crc_received != modbus_crc16(response[:7]):
        return None, "CRC mismatch"

    raw = (response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]

    # The multi-turn counter is modular and this encoder's zero sits right on
    # the wrap point, so raw 409598 means "2 counts below zero", not a jump of
    # the counter's full width. Re-expressing the top half as negative keeps
    # the position continuous through zero. Stateless, so there is nothing to
    # get stuck in a bad reference.
    if raw >= _HALF_MODULUS:
        raw -= ENCODER_MODULUS

    return raw, None


def try_set_baud_register(new_baud):
    request = build_write_single_register(MODBUS_SLAVE_ADDR, 0x0005, BAUD_REGISTER_VALUES[new_baud])
    _flush_input()
    uart.write(request)
    echo = uart.read(len(request))
    if echo != request:
        return False
    reply = uart.read(len(request))
    return reply == request


def ensure_encoder_baud(target_baud):
    for probe_baud in (target_baud, 9600, 19200, 38400, 57600):
        uart.baudrate = probe_baud
        value, err = try_read_position()
        if err is None:
            print("Encoder responded at %d baud (value=%d)" % (probe_baud, value))
            if probe_baud != target_baud:
                print("Switching encoder to %d baud..." % target_baud)
                if not try_set_baud_register(target_baud):
                    raise RuntimeError("Failed to write baud-rate register")
                time.sleep(0.05)
                uart.baudrate = target_baud
                value, err = try_read_position()
                if err is not None:
                    raise RuntimeError("Encoder did not respond after baud switch: %s" % err)
                print("Confirmed encoder now at %d baud" % target_baud)
            return
    raise RuntimeError("Encoder did not respond at any known baud rate - check wiring")


ensure_encoder_baud(TARGET_BAUD)
print("Encoder ready @ address %d, %d baud" % (MODBUS_SLAVE_ADDR, TARGET_BAUD))

# ---------------------------------------------------------------------------
# OSC message packing (address pattern + ",i" typetag + big-endian int32)
# ---------------------------------------------------------------------------


def _osc_pad(data):
    data = data + b"\x00"
    return data + b"\x00" * ((-len(data)) % 4)


def build_osc_int_template(address):
    """Return (message_bytes, offset_of_int32) for a fixed OSC int message.

    Only the trailing int32 ever changes, so the message is built once and
    the value patched in place rather than re-encoded on every packet.
    """
    addr_part = _osc_pad(address.encode("utf-8"))
    tag_part = _osc_pad(b",i")
    return bytearray(addr_part + tag_part + b"\x00\x00\x00\x00"), len(addr_part) + len(tag_part)


# ---------------------------------------------------------------------------
# Fast W5500 UDP sender
# ---------------------------------------------------------------------------


class FastOSCSender:
    """Send a fixed-shape OSC int message with minimal SPI overhead.

    The stock library needs ~32 busio SPI calls per packet: its 3-byte
    register header goes out as three separate write() calls, 16-bit
    registers are read/written one byte at a time (a full transaction each),
    and nearly every step allocates via .to_bytes()/bytearray().

    Measured on this board: ~90us of fixed overhead per busio call, but only
    ~1us per additional payload byte (a 3-byte write costs 112us, a 31-byte
    write 146us). The cost is per-CALL, not per-byte, and `with device:`
    itself is only ~25us - so the fix is fewer, larger transactions.

    This performs the identical W5500 send sequence - check TX free space,
    read the write pointer, write the payload, advance the pointer, issue
    SEND - as 5 single-transaction operations against preallocated buffers.
    Same wire protocol, no skipped steps, no cached/guessed state.
    """

    # W5500 control bytes (block select << 3 | R/W << 2 | variable-length mode)
    _SOCK_REG_READ = 0x08
    _SOCK_REG_WRITE = 0x0C
    _SOCK_TX_WRITE = 0x14
    # Socket register addresses
    _SNCR = 0x0001
    _SNTX_FSR = 0x0020
    _SNTX_WR = 0x0024
    _CMD_SEND = 0x20
    _SOCK_SIZE = 0x800
    _SOCK_MASK = 0x7FF

    def __init__(self, sock, osc_address):
        interface = sock._interface  # noqa: SLF001
        socknum = sock._socknum  # noqa: SLF001
        self._device = interface._device  # noqa: SLF001

        reg_read = (socknum << 5) + self._SOCK_REG_READ
        reg_write = (socknum << 5) + self._SOCK_REG_WRITE
        self._tx_ctrl = (socknum << 5) + self._SOCK_TX_WRITE
        self._tx_base = socknum * self._SOCK_SIZE + 0x8000

        # Read buffers: 3 header bytes then 2 bytes clocked back from the chip.
        self._fsr_out = bytearray([0x00, self._SNTX_FSR, reg_read, 0x00, 0x00])
        self._fsr_in = bytearray(5)
        self._wrp_out = bytearray([0x00, self._SNTX_WR, reg_read, 0x00, 0x00])
        self._wrp_in = bytearray(5)

        # Write buffers: header followed by the data bytes, one transaction each.
        self._ptr_buf = bytearray([0x00, self._SNTX_WR, reg_write, 0x00, 0x00])
        self._cmd_buf = bytearray([0x00, self._SNCR, reg_write, self._CMD_SEND])

        # Payload buffer: 3-byte header + the OSC message, patched in place.
        message, int_offset = build_osc_int_template(osc_address)
        self._msg_len = len(message)
        self._data_buf = bytearray(3) + message
        self._data_buf[2] = self._tx_ctrl
        self._int_offset = 3 + int_offset
        self._frame_len = 3 + self._msg_len

    def send(self, value):
        device = self._device
        n = self._msg_len

        # Patch the int32 payload in place - no message rebuild, no allocation.
        struct.pack_into(">i", self._data_buf, self._int_offset, value)

        # Each operation below MUST get its own `with device` block. In the
        # W5500's Variable Data Length Mode the address + control bytes are
        # consumed once per CS-low frame and everything after them is payload,
        # so batching several register operations under a single `with` turns
        # the later ones into buffer data and they silently do nothing. The
        # `with` costs only ~25us; the win comes from each operation issuing
        # its header and payload as ONE write() instead of four.

        # 1. Wait for room in the socket's TX buffer. Reading both bytes in a
        #    single burst also avoids the high/low-byte race that the
        #    library's repeated-read loop was guarding against.
        for _ in range(200):
            with device as bus:
                bus.write_readinto(self._fsr_out, self._fsr_in)
            if ((self._fsr_in[3] << 8) | self._fsr_in[4]) >= n:
                break
        else:
            raise OSError("W5500 TX buffer never freed up")

        # 2. Current write pointer.
        with device as bus:
            bus.write_readinto(self._wrp_out, self._wrp_in)
        pointer = (self._wrp_in[3] << 8) | self._wrp_in[4]

        # 3. Payload into the TX buffer at that offset.
        address = self._tx_base + (pointer & self._SOCK_MASK)
        self._data_buf[0] = (address >> 8) & 0xFF
        self._data_buf[1] = address & 0xFF
        with device as bus:
            bus.write(self._data_buf, end=self._frame_len)

        # 4. Advance the write pointer.
        pointer = (pointer + n) & 0xFFFF
        self._ptr_buf[3] = (pointer >> 8) & 0xFF
        self._ptr_buf[4] = pointer & 0xFF
        with device as bus:
            bus.write(self._ptr_buf)

        # 5. Issue SEND.
        with device as bus:
            bus.write(self._cmd_buf)


# ---------------------------------------------------------------------------
# Ethernet (W5500) - robust connect/reconnect
# ---------------------------------------------------------------------------

cs = digitalio.DigitalInOut(board.IO14)
rst = digitalio.DigitalInOut(board.IO9)
spi = busio.SPI(board.IO13, MOSI=board.IO11, MISO=board.IO12)


def configure_static_ip(eth, pool):
    mac = eth.mac_address
    ip_str = str(ipaddress.IPv4Address(bytes([10, 77, mac[4], mac[5]])))
    eth.ifconfig = tuple(
        pool.inet_aton(s) for s in (ip_str, STATIC_NETMASK, STATIC_GATEWAY, STATIC_GATEWAY)
    )


last_sent_value = None
poll_count = 0
error_count = 0
send_count = 0
rate_window_start = time.monotonic()

print("Connecting to Ethernet...")

while True:
    try:
        eth = WIZNET5K(spi, cs, reset=rst, is_dhcp=USE_DHCP, hostname=HOSTNAME, debug=False)

        if not eth.link_status:
            print("Ethernet link down, retrying...")
            time.sleep(5)
            continue

        print("Ethernet link up.")
        pool = socketpool.SocketPool(eth)
        if not USE_DHCP:
            configure_static_ip(eth, pool)
        print("IP address:", eth.pretty_ip(eth.ip_address))

        sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
        sock.settimeout(0)
        # sendto() reconnects the hardware socket on every call; connect()
        # once so each packet is just a buffer write plus a SEND command.
        sock.connect((OSC_HOST, OSC_PORT))
        sender = FastOSCSender(sock, OSC_ADDRESS)

        # eth.link_status is a full SPI register read (~470us). Checking it
        # every iteration cost ~5% of the loop budget, so it moves into the
        # once-per-second stats block below.
        while True:
            value, err = try_read_position()
            poll_count += 1

            if value is None:
                error_count += 1
            elif value != last_sent_value:
                last_sent_value = value
                try:
                    sender.send(value)
                    send_count += 1
                except OSError as send_err:
                    print("OSC send failed:", send_err)

            now = time.monotonic()
            elapsed = now - rate_window_start
            if elapsed >= 1.0:
                print(
                    "[rate] %.1f polls/s, %.1f sends/s (%d errors/s)"
                    % (poll_count / elapsed, send_count / elapsed, error_count / elapsed)
                )
                poll_count = 0
                error_count = 0
                send_count = 0
                gc.collect()
                rate_window_start = now
                if not eth.link_status:
                    break

        print("Ethernet link down, reconnecting...")

    except (ConnectionError, RuntimeError, OSError) as conn_err:
        print("Ethernet connection error:", conn_err)
        time.sleep(5)
