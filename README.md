# WSN Final Project — build & run

## Build & flash

Requires the local MSP430 toolchain and Contiki 2.7 at
`~/sensornet-assignment/contiki-2.7` (the Makefiles use `CONTIKI = ../../../contiki-2.7`).

```bash
source ~/sensornet-assignment/toolchain/env.sh

# Base station (on the PC-connected mote):
./flash.sh base

# Sensor nodes (unique id each):
./flash.sh node 2
./flash.sh node 3
./flash.sh node 4
./flash.sh node 5
```

`PORT=/dev/ttyUSB1 ./flash.sh node 4` overrides the serial port (default `/dev/ttyUSB0`).

> The stock `make … upload` (python2 BSL) does not run on Ubuntu 24.04, so
> `flash.sh` uses the python3 `msp430.bsl.target.telosb` loader instead.

## Run

Open the base station's serial port at **115200 8N1** (e.g. `screen /dev/ttyUSB0 115200`)
and type commands:

```
get-led             # current LED state (0..7) of each node
set-led <0..7>      # set the 3 LEDs (bit2=red, bit1=green, bit0=blue)
get-voltage         # supply voltage of each node
get-temp            # internal temperature of each node
get-channel         # radio channel of each node
get-tx-power        # TX power level of each node
agg-max             # one round: find and blink the brightest node
agg-auto on|off     # continuous mode: blinking LED tracks a moving flashlight
help                # list commands
```
