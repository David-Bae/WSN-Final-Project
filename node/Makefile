CONTIKI = ../../../contiki-2.7
TARGET ?= sky

# Per-node identity. Override on the command line, e.g.
#   make NODE_ID=3
NODE_ID ?= 2

CFLAGS += -DNODE_ID=$(NODE_ID)
CFLAGS += -I../common
# Always-on radio (nullrdc) instead of duty-cycled ContikiMAC: low, predictable
# latency. Must be set as a -D here -- NETSTACK_CONF_* is read before a
# project-conf.h would be included.
CFLAGS += -DNETSTACK_CONF_RDC=nullrdc_driver
CFLAGS += -DNETSTACK_CONF_MAC=csma_driver

# The MSP430 internal temperature sensor driver is not part of the default sky
# platform build, so pull it in explicitly.
CONTIKI_TARGET_SOURCEFILES += temperature-sensor.c

all: fp-node

include $(CONTIKI)/Makefile.include
