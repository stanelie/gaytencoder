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
from array import array

import board
import busio
import digitalio
import ipaddress
import microcontroller
import supervisor
import adafruit_wiznet5k.adafruit_wiznet5k_socketpool as socketpool
from adafruit_wiznet5k.adafruit_wiznet5k import WIZNET5K

from config import (
    USE_DHCP,
    HOSTNAME,
    STATIC_NETMASK,
    STATIC_GATEWAY,
    OSC_HOST,
    OSC_PORT,
    OSC_PREFIX,
    MODBUS_SLAVE_ADDR,
    ENCODER_MODULUS,
    LEAD_TIME_S,
    VELOCITY_WINDOW_S,
    MAX_SPEED_COUNTS_S,
    OSC_LISTEN_PORT,
    OSC_STATUS_HZ,
    OSC_STATUS_PORT,
    OSC_STATUS_TIMEOUT_S,
    OSC_ANNOUNCE_PORT,
    OSC_ANNOUNCE_S,
    STATUS_LED_BRIGHTNESS,
    STATUS_LED_ACTIVITY,
)

# Activity blink shape: short enough not to smear into the next one, spaced
# far enough apart for the eye to resolve each pulse.
_ACTIVITY_BLINK_MS = 45
_ACTIVITY_GAP_MS = 100

# Encoder fault blinks red at ~2Hz. Deliberately a different shape from the
# steady red of booting or a lost link, so "the sensor stopped answering" is
# distinguishable across a stage from "the network is not up yet".
_FAULT_BLINK_MS = 250

# Single source of truth for the version. tools/build_release.py reads this
# to name the release archive, so the two cannot disagree.
VERSION = "1.0"

# Discovery beacon. Fixed address, no prefix: it is how a board says who it
# is, so it cannot depend on knowing that already.
OSC_ANNOUNCE_ADDRESS = "/encoder/announce"

# Control addresses are fixed and carry no prefix. The IP address already
# identifies the board, and prefix-dependent control addresses would mean
# needing to know a board's current prefix before being able to change it.
OSC_CTRL_LEAD = "/control/lead"
OSC_CTRL_WINDOW = "/control/window"
OSC_CTRL_PREFIX = "/control/prefix"
OSC_CTRL_DEST = "/control/dest"
OSC_CTRL_SAVE = "/control/save"
# Keepalive. Sending anything refreshes the status subscription, but the tuner
# needs something harmless to send when nothing is being changed.
OSC_CTRL_PING = "/control/ping"

_HALF_MODULUS = ENCODER_MODULUS // 2

# supervisor.ticks_ms() counts 0..2**29-1 and then wraps (every ~6.2 days).
# Every interval measured here is well under a second, so a difference that
# comes out negative can only mean the counter wrapped in between, and adding
# one period back is enough. That is two operations instead of the four a
# general masked difference needs, in the hottest loop in the program.
_TICKS_PERIOD = 1 << 29
_TICKS_HALF = _TICKS_PERIOD >> 1


# ---------------------------------------------------------------------------
# Status LED
# ---------------------------------------------------------------------------


class StatusLED:
    """The onboard RGB LED as a state indicator you can read across a stage.

    Uses the core `neopixel_write` module rather than the `neopixel` library,
    so there is nothing extra to install on the CIRCUITPY drive.

    Flashing is non-blocking on purpose: sleeping 0.2s here would stall the
    Modbus polling for ~50 samples, which is exactly the wrong thing to do to
    acknowledge a settings change.
    """

    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    OFF = (0, 0, 0)

    def __init__(self, brightness):
        self.flashing = False
        self._base = self.OFF
        self._enabled = False
        self._flash_from = 0
        self._flash_ms = 0
        if not brightness or brightness <= 0:
            return
        try:
            import neopixel_write

            self._write = neopixel_write.neopixel_write
            self._pin = digitalio.DigitalInOut(board.NEOPIXEL)
            self._pin.direction = digitalio.Direction.OUTPUT
            self._level = min(1.0, brightness)
            self._buf = bytearray(3)
            self._enabled = True
        except (ImportError, AttributeError, ValueError) as exc:
            print("status LED unavailable:", exc)

    def _show(self, colour):
        level = self._level
        buf = self._buf
        # This LED takes green, red, blue in that order.
        buf[0] = int(colour[1] * level)
        buf[1] = int(colour[0] * level)
        buf[2] = int(colour[2] * level)
        self._write(self._pin, buf)

    def set(self, colour):
        """Set the persistent state colour, and show it unless mid-flash."""
        if not self._enabled:
            return
        self._base = colour
        if not self.flashing:
            self._show(colour)

    def show(self, colour):
        """Show a colour immediately without disturbing the state colour.

        The caller owns the timing: in the hot loop that is a plain local
        deadline, which avoids a global lookup and an attribute read on every
        single pass just to ask whether a blink is in progress.
        """
        if not self._enabled:
            return
        self.flashing = True
        self._show(colour)

    def restore(self):
        """Return to the state colour after a flash."""
        if not self._enabled:
            return
        self.flashing = False
        self._show(self._base)


status_led = StatusLED(STATUS_LED_BRIGHTNESS)
status_led.set(StatusLED.RED)  # booting

print("gaytencoder %s" % VERSION)

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
    """Return True once the encoder is answering at target_baud."""
    for probe_baud in (target_baud, 9600, 19200, 38400, 57600):
        uart.baudrate = probe_baud
        value, err = try_read_position()
        if err is None:
            print("Encoder responded at %d baud (value=%d)" % (probe_baud, value))
            if probe_baud != target_baud:
                print("Switching encoder to %d baud..." % target_baud)
                if not try_set_baud_register(target_baud):
                    print("Failed to write baud-rate register")
                    return False
                time.sleep(0.05)
                uart.baudrate = target_baud
                value, err = try_read_position()
                if err is not None:
                    print("No response after baud switch: %s" % err)
                    return False
                print("Confirmed encoder now at %d baud" % target_baud)
            return True
    # Leave the port where the next probe expects to start.
    uart.baudrate = target_baud
    return False


# A missing encoder must not stop the board coming up. It used to raise here,
# which killed the program: the board vanished from the network entirely, so
# it could not be discovered or inspected, and an encoder connected afterwards
# needed a power cycle. Now it carries on, stays discoverable, blinks red, and
# retries the encoder once a second until it answers.
if ensure_encoder_baud(TARGET_BAUD):
    print("Encoder ready @ address %d, %d baud" % (MODBUS_SLAVE_ADDR, TARGET_BAUD))
    encoder_ready = True
else:
    print("Encoder not answering - continuing anyway so the board stays reachable")
    encoder_ready = False

# ---------------------------------------------------------------------------
# Velocity estimation and latency compensation
# ---------------------------------------------------------------------------


class VelocityEstimator:
    """Velocity measured across a time window, not between adjacent samples.

    A sample-to-sample difference is far too noisy to extrapolate from: one
    count of quantisation (~0.085mm) across a single ~5ms poll reads as
    ~17 mm/s, and a 0.5s lead turns that into ~8.5mm of position jitter while
    the screen is standing still. Differencing across ~150ms instead divides
    that noise by roughly the ratio of the windows.

    Samples live in a ring buffer; the tail is advanced past anything older
    than the window, so changing the window at runtime takes effect
    immediately and costs nothing extra.

    Timestamps are supervisor.ticks_ms(), deliberately, and this matters for
    throughput. time.monotonic_ns() returns nanoseconds, which after a second
    of uptime exceed CircuitPython's small-integer range - so every timestamp
    subtraction and comparison allocated a heap object and ran through
    arbitrary-precision arithmetic. Measured on this board, that alone cost
    ~0.5ms per poll, about 12% of the loop. ticks_ms() stays a small int.

    time.monotonic() is not an option either: it is a float whose resolution
    degrades with uptime, by itself enough to corrupt a 150ms measurement
    during a long show.

    1ms resolution across a ~150ms window is ~0.7% of velocity - far below the
    accuracy to which the lead time can be judged by eye.
    """

    def __init__(self, capacity=400):
        self._pos = array("i", [0] * capacity)
        self._t = array("i", [0] * capacity)
        self._cap = capacity
        self._head = 0
        self._tail = 0

    def update(self, position, t_ms, window_ms):
        """Add a sample and return the current velocity in counts/second.

        `window_ms` is the window in whole milliseconds; the caller keeps it
        precomputed because this runs a couple of hundred times a second.
        """
        # Hoisted into locals: attribute lookups are markedly slower than
        # local slots, and this is the hottest code in the program.
        pos = self._pos
        times = self._t
        cap = self._cap
        idx = self._head

        pos[idx] = position
        times[idx] = t_ms

        nxt = idx + 1
        if nxt >= cap:
            nxt = 0
        tail = self._tail
        if nxt == tail:  # ring full - drop the oldest sample
            tail += 1
            if tail >= cap:
                tail = 0
        self._head = nxt

        # Keep the oldest sample that is still inside the window.
        while tail != idx:
            dt = t_ms - times[tail]
            if dt < 0:
                dt += _TICKS_PERIOD
            if dt <= window_ms:
                break
            tail += 1
            if tail >= cap:
                tail = 0
        else:
            dt = 0
        self._tail = tail

        if dt <= 0:
            return 0.0
        return (position - pos[tail]) * 1000.0 / dt


# Tuning values are held in NVM so they survive a power cycle. The board
# cannot write its own CIRCUITPY filesystem while USB is attached, so
# config.py only supplies the defaults for a board that has never been tuned.
# Every operator-settable value lives here, so a board keeps its whole
# identity across a power cycle. Layout:
#   magic(4) lead(4) window(4) prefix_len(1) prefix(31) dest_len(1) dest(15)
# The magic is bumped whenever the layout changes, so an older blob is
# ignored rather than misread as the new shape.
_NVM_MAGIC = b"ENC4"
_NVM_SIZE = 64
_PREFIX_MAX = 31
_DEST_MAX = 15  # "255.255.255.255"
_PREFIX_AT = 12
_DEST_AT = 44
_PORT_AT = 60


def valid_prefix(prefix):
    """A prefix must be a plain OSC container: '/name', no spaces or wildcards."""
    if not prefix or len(prefix) > _PREFIX_MAX or prefix[0] != "/":
        return False
    for ch in prefix:
        if ch in " #*,?[]{}\x00" or ord(ch) < 0x20 or ord(ch) > 0x7E:
            return False
    return True


def valid_ipv4(text):
    parts = text.split(".")
    if len(parts) != 4:
        return False
    for part in parts:
        if not part or len(part) > 3:
            return False
        for ch in part:
            if ch < "0" or ch > "9":
                return False
        if int(part) > 255:
            return False
    return True


def _read_string(blob, at, limit, validator, fallback):
    length = blob[at]
    if 0 < length <= limit:
        try:
            candidate = str(blob[at + 1 : at + 1 + length], "utf-8")
            if validator(candidate):
                return candidate
        except UnicodeError:
            pass
    return fallback


def load_tuning(defaults):
    """defaults = (lead, window, prefix, dest, port) -> same tuple + restored flag."""
    try:
        nvm = microcontroller.nvm
        if nvm is None:
            return defaults + (False,)
        blob = bytes(nvm[0:_NVM_SIZE])
        if blob[0:4] != _NVM_MAGIC:
            return defaults + (False,)
        lead, window = struct.unpack(">ff", blob[4:12])
        # Refuse implausible stored values rather than trusting NVM blindly.
        if not (0.0 <= lead <= 5.0 and 0.005 <= window <= 2.0):
            return defaults + (False,)
        prefix = _read_string(blob, _PREFIX_AT, _PREFIX_MAX, valid_prefix, defaults[2])
        dest = _read_string(blob, _DEST_AT, _DEST_MAX, valid_ipv4, defaults[3])
        port = struct.unpack(">H", blob[_PORT_AT : _PORT_AT + 2])[0]
        if not 1 <= port <= 65535:
            port = defaults[4]
        return lead, window, prefix, dest, port, True
    except (AttributeError, OSError, ValueError):
        pass
    return defaults + (False,)


def save_tuning(lead, window, prefix, dest, port):
    try:
        nvm = microcontroller.nvm
        if nvm is None:
            return False
        blob = bytearray(_NVM_SIZE)
        blob[0:4] = _NVM_MAGIC
        blob[4:12] = struct.pack(">ff", lead, window)
        encoded = prefix.encode("utf-8")[:_PREFIX_MAX]
        blob[_PREFIX_AT] = len(encoded)
        blob[_PREFIX_AT + 1 : _PREFIX_AT + 1 + len(encoded)] = encoded
        encoded = dest.encode("utf-8")[:_DEST_MAX]
        blob[_DEST_AT] = len(encoded)
        blob[_DEST_AT + 1 : _DEST_AT + 1 + len(encoded)] = encoded
        blob[_PORT_AT : _PORT_AT + 2] = struct.pack(">H", port)
        nvm[0:_NVM_SIZE] = blob
        return True
    except (AttributeError, OSError, ValueError):
        return False


(
    lead_time,
    velocity_window,
    osc_prefix,
    osc_dest,
    osc_dest_port,
    _restored,
) = load_tuning((LEAD_TIME_S, VELOCITY_WINDOW_S, OSC_PREFIX, OSC_HOST, OSC_PORT))
print(
    "Lead %.3fs, window %.3fs, prefix %s, destination %s:%d (%s)"
    % (
        lead_time,
        velocity_window,
        osc_prefix,
        osc_dest,
        osc_dest_port,
        "restored from NVM" if _restored else "from config.py",
    )
)

velocity_estimator = VelocityEstimator()


# ---------------------------------------------------------------------------
# OSC message packing
# ---------------------------------------------------------------------------


def _osc_pad(data):
    data = data + b"\x00"
    return data + b"\x00" * ((-len(data)) % 4)


def build_osc_int_template(address):
    """Return (message_bytes, offset_of_int32) for a fixed OSC int message.

    Only the trailing int32 ever changes, so the message is built once and the
    value patched in place rather than re-encoded on every packet.

    Position is sent as an integer count. One count is under 0.1mm on this rig,
    so a fractional part carries no usable information - and rounding to whole
    counts also stops the prediction from emitting a new value on every poll
    when the screen is barely moving, since velocity quantisation would jitter
    the fractional part even when the position had not really changed.
    """
    addr_part = _osc_pad(address.encode("utf-8"))
    tag_part = _osc_pad(b",i")
    return bytearray(addr_part + tag_part + b"\x00\x00\x00\x00"), len(addr_part) + len(tag_part)


def osc_message(address, *args):
    """Build an arbitrary OSC message. Used only for the low-rate status and
    announce messages, so the allocations here do not matter."""
    tags = ","
    payload = b""
    for arg in args:
        if isinstance(arg, str):
            tags += "s"
            payload += _osc_pad(arg.encode("utf-8"))
        elif isinstance(arg, float):
            tags += "f"
            payload += struct.pack(">f", arg)
        else:
            tags += "i"
            payload += struct.pack(">i", arg)
    return _osc_pad(address.encode("utf-8")) + _osc_pad(tags.encode("utf-8")) + payload


def parse_osc(data):
    """Minimal OSC parser -> (address, [args]). Returns (None, None) if the
    packet is not something we understand. Only 'f' and 'i' are decoded."""
    try:
        end = data.find(b"\x00")
        if end < 0:
            return None, None
        address = str(data[:end], "utf-8")

        # The address is null-padded to a multiple of 4 bytes.
        pos = (end // 4 + 1) * 4
        if len(data) <= pos or data[pos] != 0x2C:  # ord(",")
            return address, []  # no typetag: treat as a bare trigger

        tag_end = data.find(b"\x00", pos)
        if tag_end < 0:
            return address, []
        tags = str(data[pos + 1 : tag_end], "utf-8")
        # Typetag block is likewise null-padded to a multiple of 4.
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
                args.append(str(data[pos:s_end], "utf-8"))
                pos = ((s_end - pos) // 4 + 1) * 4 + pos
            else:
                break
        return address, args
    except (ValueError, IndexError, UnicodeError):
        return None, None


# ---------------------------------------------------------------------------
# Fast W5500 UDP sender
# ---------------------------------------------------------------------------


class FastOSCSender:
    """Send OSC messages with minimal SPI overhead.

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

    _MAX_PAYLOAD = 128

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

        # Two separate buffers, deliberately. The hot-path buffer holds the
        # prebuilt position message and only ever has its float patched; the
        # scratch buffer takes arbitrary messages. Sharing one buffer means a
        # longer status message overwrites the position template sitting in
        # the first bytes, and every subsequent position packet goes out
        # carrying the wrong address - silently, since the send itself
        # succeeds. (That regression is exactly what this comment exists to
        # prevent a repeat of.)
        self._scratch_buf = bytearray(3 + self._MAX_PAYLOAD)
        self._scratch_buf[2] = self._tx_ctrl
        self.set_address(osc_address)

    def set_address(self, osc_address):
        """Rebuild the prebuilt position message for a new OSC address."""
        message, int_offset = build_osc_int_template(osc_address)
        self._msg_len = len(message)
        self._data_buf = bytearray(3) + message
        self._data_buf[2] = self._tx_ctrl
        self._int_offset = 3 + int_offset

    def send(self, value):
        """Hot path: patch the int32 in place and send. No allocation."""
        struct.pack_into(">i", self._data_buf, self._int_offset, value)
        self._send_frame(self._data_buf, self._msg_len)

    def send_bytes(self, payload):
        """Send an arbitrary pre-built OSC message (low-rate use)."""
        n = len(payload)
        if n > self._MAX_PAYLOAD:
            raise ValueError("payload too large")
        self._scratch_buf[3 : 3 + n] = payload
        self._send_frame(self._scratch_buf, n)

    def _send_frame(self, buf, n):
        device = self._device

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
        buf[0] = (address >> 8) & 0xFF
        buf[1] = address & 0xFF
        with device as bus:
            bus.write(buf, end=3 + n)

        # 4. Advance the write pointer.
        pointer = (pointer + n) & 0xFFFF
        self._ptr_buf[3] = (pointer >> 8) & 0xFF
        self._ptr_buf[4] = pointer & 0xFF
        with device as bus:
            bus.write(self._ptr_buf)

        # 5. Issue SEND.
        with device as bus:
            bus.write(self._cmd_buf)


class ControlListener:
    """Non-blocking OSC control input for live tuning.

    Asking the library whether anything arrived is expensive, so availability
    is checked with the same single-transaction trick used for sending: one
    read of the socket's RX Received Size register (~120us). Only when that
    is non-zero do we fall back to the library's (slow) receive path, which
    happens rarely - a few tuning messages during tech.
    """

    _SNRX_RSR = 0x0026

    def __init__(self, sock):
        interface = sock._interface  # noqa: SLF001
        socknum = sock._socknum  # noqa: SLF001
        self._device = interface._device  # noqa: SLF001
        self._sock = sock
        reg_read = (socknum << 5) + 0x08
        self._rsr_out = bytearray([0x00, self._SNRX_RSR, reg_read, 0x00, 0x00])
        self._rsr_in = bytearray(5)
        self._buf = bytearray(256)

    def poll(self):
        """Return (address, args, sender_ip), or (None, None, None).

        The sender's address matters: status messages are sent back to
        whoever last talked to us, so the tuner receives them without needing
        to be configured anywhere.
        """
        with self._device as bus:
            bus.write_readinto(self._rsr_out, self._rsr_in)
        if ((self._rsr_in[3] << 8) | self._rsr_in[4]) == 0:
            return None, None, None
        try:
            count, sender = self._sock.recvfrom_into(self._buf, len(self._buf))
        except OSError:
            return None, None, None
        if not count:
            return None, None, None
        # bytes() copy rather than a memoryview: CircuitPython memoryviews
        # have no .find(). This path runs only when a tuning message actually
        # arrives, so the allocation is irrelevant.
        address, args = parse_osc(bytes(self._buf[:count]))
        return address, args, (sender[0] if sender else None)


def apply_control(address, args):
    """Handle one tuning command. Returns True if something changed."""
    global lead_time, velocity_window, osc_prefix, osc_dest, osc_dest_port

    if address == OSC_CTRL_PING:
        return False  # nothing to do; arriving at all refreshes the subscription

    if address == OSC_CTRL_DEST:
        # Address and port travel together so they change in one atomic step,
        # rather than briefly pointing at a host/port pair that never existed.
        if not args or not isinstance(args[0], str):
            print("%s needs an address, e.g. 10.8.1.81 [port]" % address)
            return False
        candidate = args[0].strip()
        if not valid_ipv4(candidate):
            print("ignored invalid destination: %r" % (args[0],))
            return False
        port = osc_dest_port
        if len(args) > 1:
            try:
                port = int(args[1])
            except (TypeError, ValueError):
                print("ignored invalid destination port: %r" % (args[1],))
                return False
            if not 1 <= port <= 65535:
                print("ignored out-of-range destination port:", port)
                return False
        if candidate == osc_dest and port == osc_dest_port:
            return False
        osc_dest = candidate
        osc_dest_port = port
        print("destination -> %s:%d" % (osc_dest, osc_dest_port))
        return True

    if address == OSC_CTRL_PREFIX:
        if not args or not isinstance(args[0], str):
            print("%s needs a string argument, e.g. /encoder2" % address)
            return False
        candidate = args[0].strip()
        if not candidate.startswith("/"):
            candidate = "/" + candidate
        if candidate.endswith("/"):
            candidate = candidate[:-1]
        if not valid_prefix(candidate):
            print("ignored invalid prefix: %r" % (args[0],))
            return False
        if candidate == osc_prefix:
            return False
        osc_prefix = candidate
        print("prefix -> %s (now sending %s/position)" % (osc_prefix, osc_prefix))
        return True

    if address == OSC_CTRL_LEAD:
        if not args:
            print("%s needs a float argument" % address)
            return False
        value = float(args[0])
        if 0.0 <= value <= 5.0:
            lead_time = value
            print("lead time -> %.3fs" % lead_time)
            return True
        print("ignored out-of-range lead:", value)

    elif address == OSC_CTRL_WINDOW:
        if not args:
            print("%s needs a float argument" % address)
            return False
        value = float(args[0])
        if 0.005 <= value <= 2.0:
            velocity_window = value
            print("velocity window -> %.3fs" % velocity_window)
            return True
        print("ignored out-of-range window:", value)

    elif address == OSC_CTRL_SAVE:
        if save_tuning(lead_time, velocity_window, osc_prefix, osc_dest, osc_dest_port):
            print(
                "saved: lead=%.3fs window=%.3fs prefix=%s dest=%s:%d"
                % (lead_time, velocity_window, osc_prefix, osc_dest, osc_dest_port)
            )
        else:
            print("save failed (no NVM available)")

    else:
        print(
            "ignored unknown address %s - expecting %s, %s, %s, %s or %s"
            % (
                address,
                OSC_CTRL_LEAD,
                OSC_CTRL_WINDOW,
                OSC_CTRL_PREFIX,
                OSC_CTRL_DEST,
                OSC_CTRL_SAVE,
            )
        )

    return False


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
# Set when a tuning value changes, cleared once written to NVM. Changes are
# persisted automatically, but batched to at most one flash write per second
# so that a flurry of adjustments does not become a flurry of writes.
tuning_dirty = False
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
        status_led.set(StatusLED.BLUE)  # network up, not streaming yet

        sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
        sock.settimeout(0)
        # sendto() reconnects the hardware socket on every call; connect()
        # once so each packet is just a buffer write plus a SEND command.
        sock.connect((osc_dest, osc_dest_port))
        sender = FastOSCSender(sock, osc_prefix + "/position")
        status_address = osc_prefix + "/status"
        active_prefix = osc_prefix
        active_dest = (osc_dest, osc_dest_port)
        print("Sending %s/position to %s:%d" % (osc_prefix, osc_dest, osc_dest_port))

        ctrl_sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
        ctrl_sock.settimeout(0)
        ctrl_sock.bind(("", OSC_LISTEN_PORT))
        control = ControlListener(ctrl_sock)
        print("Listening for tuning on port %d" % OSC_LISTEN_PORT)

        status_interval = 1.0 / OSC_STATUS_HZ if OSC_STATUS_HZ else 0
        next_status = time.monotonic()
        # Status goes to whoever is tuning, on its own socket, and only while
        # a subscription is live. With no tuner running - i.e. during a show -
        # nothing is sent and the position stream is the only traffic.
        status_sock = None
        status_sender = None
        status_dest = None
        status_deadline = 0.0

        # Discovery beacon, so the tuner can list boards without anyone
        # knowing an IP address.
        announce_sender = None
        next_announce = time.monotonic()
        if OSC_ANNOUNCE_S:
            try:
                announce_sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
                announce_sock.settimeout(0)
                announce_sock.connect(("255.255.255.255", OSC_ANNOUNCE_PORT))
                announce_sender = FastOSCSender(announce_sock, OSC_ANNOUNCE_ADDRESS)
                print("Announcing on port %d every %.0fs" % (OSC_ANNOUNCE_PORT, OSC_ANNOUNCE_S))
            except (OSError, RuntimeError) as announce_err:
                print("announce disabled:", announce_err)

        control_countdown = 0
        raw_value = 0
        velocity = 0.0
        cached_window = velocity_window
        cached_window_ms = int(velocity_window * 1000.0)
        # Bound method into a local: one attribute lookup saved per poll.
        estimator_update = velocity_estimator.update
        activity_led = STATUS_LED_ACTIVITY and STATUS_LED_BRIGHTNESS > 0
        last_blink_ms = 0
        # 0 means "no flash in progress". Kept as a loop local so the steady
        # state costs a single local truth test per pass.
        led_flash_until = 0
        led_show = status_led.show
        led_restore = status_led.restore
        encoder_fault = not encoder_ready
        fault_next_ms = 0
        fault_lit = False

        status_led.set(StatusLED.GREEN)  # streaming

        # eth.link_status is a full SPI register read (~470us). Checking it
        # every iteration cost ~5% of the loop budget, so it moves into the
        # once-per-second stats block below.
        while True:
            value, err = try_read_position()
            poll_count += 1

            # One reading of the clock per pass, shared by the velocity window
            # and the LED timing, rather than each fetching its own.
            now_ms = supervisor.ticks_ms()

            if value is None:
                error_count += 1
            else:
                raw_value = value
                if velocity_window != cached_window:
                    cached_window = velocity_window
                    cached_window_ms = int(velocity_window * 1000.0)
                velocity = estimator_update(value, now_ms, cached_window_ms)
                # One bad reading must not fling the prediction across the stage.
                if velocity > MAX_SPEED_COUNTS_S:
                    velocity = MAX_SPEED_COUNTS_S
                elif velocity < -MAX_SPEED_COUNTS_S:
                    velocity = -MAX_SPEED_COUNTS_S

                # Project forward by the pipeline latency, so the image lands
                # where the screen will BE when it is finally displayed. Zero
                # correction at rest; grows with speed.
                #
                # Rounded to a whole count: one count is under 0.1mm here, so
                # the fraction is noise, and rounding stops a barely-moving
                # screen from emitting a fresh value every poll purely from
                # jitter in the velocity estimate.
                predicted = round(value + velocity * lead_time)

                if predicted != last_sent_value:
                    last_sent_value = predicted
                    try:
                        sender.send(predicted)
                        send_count += 1
                    except OSError as send_err:
                        print("OSC send failed:", send_err)

                    # Blue pulse over the steady green while the position is
                    # moving. Rate-limited: the position changes far faster
                    # than the eye can follow, so one pulse per change would
                    # blur into a solid colour and cost hundreds of LED writes
                    # a second. Deferring while a flash is already running also
                    # leaves the red save blink undisturbed.
                    if activity_led and not led_flash_until:
                        since = now_ms - last_blink_ms
                        if since < 0:
                            since += _TICKS_PERIOD
                        if since >= _ACTIVITY_GAP_MS:
                            last_blink_ms = now_ms
                            led_flash_until = now_ms + _ACTIVITY_BLINK_MS
                            led_show(StatusLED.BLUE)

            # Tuning input, checked a few times a second rather than every
            # iteration - one RX-size register read, ~120us when idle.
            control_countdown -= 1
            if control_countdown <= 0:
                control_countdown = 20  # ~10 times/second at ~210Hz
                address, args, sender_ip = control.poll()
                if address is not None:
                    if apply_control(address, args):
                        tuning_dirty = True

                    # Any control message renews the status subscription and
                    # points it at the sender.
                    status_deadline = time.monotonic() + OSC_STATUS_TIMEOUT_S
                    if sender_ip and sender_ip != status_dest and status_interval:
                        try:
                            if status_sock is None:
                                status_sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
                                status_sock.settimeout(0)
                            status_sock.connect((sender_ip, OSC_STATUS_PORT))
                            if status_sender is None:
                                status_sender = FastOSCSender(status_sock, status_address)
                            status_dest = sender_ip
                            print("status -> %s:%d" % (status_dest, OSC_STATUS_PORT))
                        except (OSError, RuntimeError) as status_err:
                            print("status subscribe failed:", status_err)
                            status_sender = None
                            status_dest = None

                    if osc_prefix != active_prefix:
                        # Retarget both outgoing messages; the position
                        # template is prebuilt, so it has to be rebuilt.
                        active_prefix = osc_prefix
                        sender.set_address(osc_prefix + "/position")
                        status_address = osc_prefix + "/status"
                        last_sent_value = None  # resend under the new address

                    if (osc_dest, osc_dest_port) != active_dest:
                        # Point the position socket at the new receiver. The
                        # socket number is unchanged, so the sender's cached
                        # registers stay valid.
                        try:
                            sock.connect((osc_dest, osc_dest_port))
                            active_dest = (osc_dest, osc_dest_port)
                            last_sent_value = None
                            print("now sending to %s:%d" % (osc_dest, osc_dest_port))
                        except (OSError, RuntimeError) as dest_err:
                            print("could not retarget to %s: %s" % (osc_dest, dest_err))
                            osc_dest, osc_dest_port = active_dest

            # Encoder not answering: keep blinking red for as long as it
            # lasts, rather than a one-shot flash.
            if encoder_fault:
                due = now_ms - fault_next_ms
                if due < 0:
                    due += _TICKS_PERIOD
                if due >= 0:
                    fault_next_ms = now_ms + _FAULT_BLINK_MS
                    fault_lit = not fault_lit
                    led_show(StatusLED.RED if fault_lit else StatusLED.OFF)

            # Steady state is one local truth test; the rest only runs while a
            # blink is actually on screen.
            elif led_flash_until:
                remaining = led_flash_until - now_ms
                if remaining < -_TICKS_HALF:
                    remaining += _TICKS_PERIOD
                if remaining <= 0:
                    led_flash_until = 0
                    led_restore()

            now = time.monotonic()

            if announce_sender is not None and now >= next_announce:
                next_announce = now + OSC_ANNOUNCE_S
                try:
                    announce_sender.send_bytes(
                        osc_message(
                            OSC_ANNOUNCE_ADDRESS,
                            osc_prefix,
                            osc_dest,
                            osc_dest_port,
                            # Encoder health, so the tuner can say why a
                            # listed board is not producing anything.
                            0 if encoder_fault else 1,
                            float(raw_value),
                            float(lead_time),
                            float(velocity_window),
                        )
                    )
                except (OSError, ValueError):
                    pass

            if (
                status_interval
                and status_sender is not None
                and now < status_deadline
                and now >= next_status
            ):
                next_status = now + status_interval
                try:
                    status_sender.send_bytes(
                        osc_message(
                            status_address,
                            float(raw_value),
                            float(velocity),
                            float(lead_time),
                            float(velocity_window),
                        )
                    )
                except (OSError, ValueError):
                    pass

            elapsed = now - rate_window_start
            if elapsed >= 1.0:
                print(
                    "[rate] %.1f polls/s, %.1f sends/s (%d errors/s)  "
                    "pos=%d vel=%.0f cnt/s lead=%.3f win=%.3f"
                    % (
                        poll_count / elapsed,
                        send_count / elapsed,
                        error_count / elapsed,
                        raw_value,
                        velocity,
                        lead_time,
                        velocity_window,
                    )
                )
                # A dead sensor should be visible from across the stage, not
                # only in a console. Blinking red distinguishes it from the
                # steady red of booting or a lost link.
                fault_now = poll_count > 0 and error_count > poll_count // 2

                # While faulted, re-probe once a second: an encoder connected
                # late, or one that dropped and returned, may also need its
                # baud rate setting again. Costs ~0.3s when it fails, which is
                # free time - there is nothing to stream anyway.
                if fault_now and ensure_encoder_baud(TARGET_BAUD):
                    fault_now = False

                if fault_now != encoder_fault:
                    encoder_fault = fault_now
                    if fault_now:
                        print("encoder not answering")
                        fault_next_ms = 0  # start blinking on the next pass
                        fault_lit = False
                    else:
                        print("encoder answering again")
                        status_led.set(StatusLED.GREEN)
                        led_flash_until = 0
                        led_restore()
                elif not encoder_fault:
                    status_led.set(StatusLED.GREEN)

                poll_count = 0
                error_count = 0
                send_count = 0

                if tuning_dirty:
                    if save_tuning(lead_time, velocity_window, osc_prefix, osc_dest, osc_dest_port):
                        print(
                            "saved: lead=%.3fs window=%.3fs prefix=%s dest=%s:%d"
                            % (lead_time, velocity_window, osc_prefix, osc_dest, osc_dest_port)
                        )
                        # Acknowledge the write where the operator is looking.
                        # Overrides any activity blink; a save matters more.
                        led_flash_until = supervisor.ticks_ms() + 200
                        led_show(StatusLED.RED)
                    else:
                        print("auto-save failed (no NVM available)")
                    tuning_dirty = False

                gc.collect()
                rate_window_start = now
                if not eth.link_status:
                    break

        print("Ethernet link down, reconnecting...")
        status_led.set(StatusLED.RED)

    except (ConnectionError, RuntimeError, OSError) as conn_err:
        print("Ethernet connection error:", conn_err)
        status_led.set(StatusLED.RED)
        time.sleep(5)
