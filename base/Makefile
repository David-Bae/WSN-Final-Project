CONTIKI = ../../../contiki-2.7
TARGET ?= sky

CFLAGS += -I../common
# Always-on radio (nullrdc) instead of duty-cycled ContikiMAC: low, predictable
# latency for the interactive command/response demo. Must be set as a -D here --
# NETSTACK_CONF_* is read before a project-conf.h would be included.
CFLAGS += -DNETSTACK_CONF_RDC=nullrdc_driver
CFLAGS += -DNETSTACK_CONF_MAC=csma_driver

all: fp-base

include $(CONTIKI)/Makefile.include
