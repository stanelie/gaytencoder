# Network + OSC destination config for the encoder-to-OSC bridge.

NETWORK_MODE = "ETH"
USE_DHCP = True                    # False to use the static-IP-from-MAC scheme below
HOSTNAME = "encoder-bridge-1"       # give each board a unique hostname

# Only used when USE_DHCP = False: static IP is derived from this board's own
# MAC address (last two octets), so each board gets a distinct address
# automatically without per-board config, as long as they share this subnet.
STATIC_NETMASK = "255.255.255.0"
STATIC_GATEWAY = "10.8.1.1"

# Where to send OSC position messages - the computer running your OSC receiver.
#
# OSC_HOST is a DEFAULT only: the destination can be set from the tuner and is
# stored in NVM, and a stored value wins over this one. It applies to a board
# that has never had its destination set.
#
# Discovery does not depend on it - beacons are broadcast - so a board with the
# wrong destination still appears in the tuner and can be corrected.
OSC_HOST = "10.8.1.81"
OSC_PORT = 9000

# This board's OSC identity. Outgoing messages are <prefix>/position and
# <prefix>/status, so a second board only needs a different prefix.
#
# This is a DEFAULT: the prefix can be set live over OSC and stored in NVM,
# and a stored value wins over this one. It only applies to a board that has
# never had its prefix set.
OSC_PREFIX = "/encoder1"

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
# Send these to this board's IP on OSC_LISTEN_PORT while watching the screen
# move:
#
#   /control/lead    <float seconds>   pipeline latency to compensate
#   /control/window  <float seconds>   velocity averaging window
#   /control/prefix  <string>          this board's outgoing OSC prefix
#   /control/save                      persist all three to NVM
#
# These control addresses are deliberately FIXED and carry no prefix: the IP
# address already says which board you are talking to, and if they depended
# on the prefix you would have to know a board's current prefix in order to
# change it.
#
# Commands are polled a few times a second, so they apply promptly without
# costing anything in the hot loop.
OSC_LISTEN_PORT = 9001

# Low-rate diagnostic message on <prefix>/status, carrying four floats:
#   position (counts), velocity (counts/s), lead (s), window (s)
#
# It is NOT sent to OSC_HOST alongside the position stream. It goes only to
# whichever machine last sent a control message, on OSC_STATUS_PORT - in
# practice, to the tuner app while you are actually tuning. That keeps the
# stream Millumin sees to nothing but position messages, and means that during
# a show, with no tuner running, the board sends no status at all and the
# diagnostic costs zero framerate.
#
# The subscription lapses OSC_STATUS_TIMEOUT_S after the last control message;
# the tuner pings periodically to hold it open.
OSC_STATUS_HZ = 10
OSC_STATUS_PORT = 9002
OSC_STATUS_TIMEOUT_S = 15

# --- Discovery -------------------------------------------------------------
#
# Each board broadcasts a beacon so the tuner can list what is on the network
# and the operator never has to know an IP address. The beacon carries the
# board's prefix, live position, and current tuning:
#
#   /encoder/announce  ,sfff  <prefix> <position> <lead> <window>
#
# Sent to 255.255.255.255 - the LIMITED broadcast address, deliberately. The
# W5500 ARPs for ordinary destinations, and a subnet-directed broadcast such
# as 10.8.0.255 would ARP for an address nothing answers to, so those sends
# would silently fail. 255.255.255.255 is special-cased to the broadcast MAC.
#
# Live position is included so the operator can identify which physical
# encoder is which: pull a rope and watch which entry in the list moves.
#
# One small packet every couple of seconds is far too little to affect the
# position framerate. Set OSC_ANNOUNCE_S = 0 to disable.
OSC_ANNOUNCE_PORT = 9003
OSC_ANNOUNCE_S = 2.0

# --- Status LED ------------------------------------------------------------
#
# The board's onboard RGB LED as a glanceable state indicator:
#
#   red     booting, or link lost / encoder not answering
#   blue    network up, IP acquired, not yet streaming
#   green   running normally
#   blue blink over green   position is moving
#   red blink (0.2s)        settings received and written to NVM
#
# Brightness 0.0-1.0. Keep it low: this board rides on the moving screen, so
# the LED may well be in the audience's sightline. Set 0 to disable the LED
# entirely for performances - the console still reports everything.
STATUS_LED_BRIGHTNESS = 0.15

# Blue activity blink whenever the position changes.
#
# Deliberately rate-limited rather than one blink per change: the position
# changes up to ~200 times a second while the screen moves, which the eye
# cannot resolve - it would read as a steady green-blue blend, and would cost
# hundreds of LED writes a second for nothing. Ten short pulses a second reads
# as a clear flicker and costs about 20 writes a second.
#
# Set False to keep the LED steady green while running.
STATUS_LED_ACTIVITY = True

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
