#!/bin/sh -xe

set -e

#arduino-cli lib install PinChangeInterrupt
arduino-cli compile --fqbn aceduino:avr:m8xt8m EtherBus
# arduino-cli upload -p /dev/ttyUSB0 --fqbn aceduino:avr:m168xt4m AceGrid
arduino-cli upload --verbose -P avrispmk2 --fqbn aceduino:avr:m8xt8m EtherBus
