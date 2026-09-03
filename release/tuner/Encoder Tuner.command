#!/bin/bash
# Double-click this to open the Encoder Tuner.
# First time only, make it executable:   chmod +x "Encoder Tuner.command"
cd "$(dirname "$0")" || exit 1
exec python3 osc_tuner.py
