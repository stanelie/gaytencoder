# Network + OSC destination config for the encoder-to-OSC bridge.

NETWORK_MODE = "ETH"
USE_DHCP = True                    # False to use the static-IP-from-MAC scheme below
HOSTNAME = "encoder-bridge-1"       # give each board a unique hostname

# Only used when USE_DHCP = False: static IP is derived from this board's own
# MAC address (last two octets), so each board gets a distinct address
# automatically without per-board config, as long as they share this subnet.
STATIC_NETMASK = "255.255.255.0"
STATIC_GATEWAY = "10.8.1.1"

# Where to send OSC messages - point this at the computer running your OSC receiver.
OSC_HOST = "10.8.1.81"
OSC_PORT = 9000
OSC_ADDRESS = "/encoder1/position"

# Which Modbus slave address this board's RS485 bus should talk to.
MODBUS_SLAVE_ADDR = 1

# Full span of the encoder's multi-turn counter, in raw counts:
#   counts-per-turn (4096, the datasheet default) x multi-turn range (100 turns)
#
# The counter is modular - it wraps 409599 -> 0 going one way and 0 -> 409599
# going the other. That is normal behaviour, not corruption, and the readings
# either side of the wrap are genuine (their Modbus CRC is valid).
#
# It matters here because the encoder's zero happens to sit exactly on the
# wrap point, so ordinary small movements at full rope retraction cross it
# constantly, and a raw reading of 409598 really means "2 counts below zero".
# Values at or above half the span are therefore re-expressed as negative in
# code.py, which makes the position continuous through zero instead of
# jumping the full width of the counter.
#
# The usable range becomes -204800..204799 counts. At ~11.8 counts/mm that is
# about +/-17m, comfortably beyond the 15m rope, so real positions are never
# ambiguous.
ENCODER_MODULUS = 4096 * 100
