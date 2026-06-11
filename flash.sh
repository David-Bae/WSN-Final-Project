#!/usr/bin/env bash
#
# Build + flash helper for the Final Project motes.
#
# The default Contiki "make ... upload" path needs python2 (gone on Ubuntu 24.04),
# so we flash with the python3 telosb BSL from python-msp430-tools instead.
#
# Usage:
#   ./flash.sh base                 # build + flash the base station
#   ./flash.sh node 2               # build + flash a sensor node with NODE_ID=2
#   ./flash.sh node 3               # ... NODE_ID=3, etc.
#
#   PORT=/dev/ttyUSB1 ./flash.sh node 4    # override serial port (default ttyUSB0)
#
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
source ~/sensornet-assignment/toolchain/env.sh

PORT="${PORT:-/dev/ttyUSB0}"
ROLE="$1"

case "$ROLE" in
  base)
    DIR="$HERE/base"; PROJ="fp-base"
    ( cd "$DIR" && make TARGET=sky )
    ;;
  node)
    NODE_ID="${2:?usage: ./flash.sh node <id>}"
    DIR="$HERE/node"; PROJ="fp-node"
    # NODE_ID is a -D compile flag; make only tracks source mtimes, so a stale
    # object from a previous NODE_ID would be relinked WITHOUT recompiling.
    # `touch fp-node.c` is not enough -- back-to-back builds can land in the same
    # filesystem-timestamp second and make then still skips the recompile (this
    # mis-flashed node 4 as node 3 once). Delete the object + .sky outright to
    # GUARANTEE a fresh compile with the correct NODE_ID.
    ( cd "$DIR" && rm -f obj_sky/fp-node.o fp-node.co fp-node.sky \
                && make TARGET=sky NODE_ID="$NODE_ID" )
    ;;
  *)
    echo "usage: ./flash.sh base | node <id>   (PORT=/dev/ttyUSBx to override)"
    exit 1
    ;;
esac

cd "$DIR"
msp430-objcopy "$PROJ.sky" -O ihex "$PROJ.ihex"
echo ">>> flashing $PROJ to $PORT ..."
python3 -m msp430.bsl.target.telosb -e -P -v -p "$PORT" -i ihex "$PROJ.ihex"
echo ">>> done: $PROJ on $PORT"
