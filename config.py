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

# --- Latency compensation --------------------------------------------------
#
# The video chain (Millumin -> processor -> LED wall) lags the physical screen
# by roughly half a second, so the image always shows where the screen WAS.
# Compensate by extrapolating forward:
#
#     predicted = position + velocity * LEAD_TIME_S
#
# At rest the correction is zero and the position is exact; the faster the
# screen moves the further ahead it is projected. This is only exact at
# constant velocity - during acceleration it is wrong by about
# (change in velocity) x LEAD_TIME_S - but a ~300 lb screen pushed by hand
# cannot change speed quickly, so in practice the ramps are gentle.
#
# LEAD_TIME_S must be measured against the real pipeline. Both values below
# are live-tunable over OSC (see the control addresses further down) and can
# be saved to the board's NVM so they survive a power cycle.
LEAD_TIME_S = 0.5

# Window used to estimate velocity, in seconds.
#
# This is a trade-off, not a "correct" value. Longer = smoother velocity and
# less jitter when standing still, but slower to react to changes of speed.
# Shorter = more responsive, but noisier.
#
# Why it cannot be a simple sample-to-sample difference: at ~210Hz one count
# of quantisation (~0.085mm) across a single ~5ms poll is ~17 mm/s of apparent
# velocity, which a 0.5s lead turns into ~8.5mm of position jitter while the
# screen is completely still. Measuring across 150ms instead divides that
# noise by roughly the same factor the window is longer - about 0.6mm.
VELOCITY_WINDOW_S = 0.15

# Sanity clamp on velocity, in counts/second, so that one bad reading can
# never throw the prediction across the stage. ~11.8 counts/mm, so this is
# about 2 m/s - far above anything a hand-pushed screen will do.
MAX_SPEED_COUNTS_S = 11.8 * 2000

# --- Live tuning over OSC --------------------------------------------------
# Send these from Millumin (or any OSC sender) to this board's IP on
# OSC_LISTEN_PORT while watching the screen move:
#
#   /encoder1/lead    <float seconds>   pipeline latency to compensate
#   /encoder1/window  <float seconds>   velocity averaging window
#   /encoder1/save                      persist both to NVM
#
# Commands are polled a few times a second, so they apply promptly without
# costing anything in the hot loop.
OSC_LISTEN_PORT = 9001
OSC_CTRL_LEAD = "/encoder1/lead"
OSC_CTRL_WINDOW = "/encoder1/window"
OSC_CTRL_SAVE = "/encoder1/save"

# Low-rate diagnostic message: raw position, velocity (counts/s), and the
# current lead/window values, so you can see what the board is actually doing
# while tuning. Sent at OSC_STATUS_HZ, independent of the main position
# stream. Set OSC_STATUS_HZ = 0 to disable.
OSC_STATUS_ADDRESS = "/encoder1/status"
OSC_STATUS_HZ = 10

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
