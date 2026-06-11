/*
 * Final Project - Wireless Sensor Network with commands
 * Shared protocol for the base station and sensor nodes.
 * Target: Kmote / TelosB-class MSP430 mote, Contiki 2.7, TARGET=sky.
 *
 * Everything is Rime broadcast: reliable unicast (runicast) needs an 802.15.4
 * ACK within ~192 us that CC2420 + nullrdc cannot meet, so the base broadcasts
 * each command and every node answers.
 *
 * Part-2 is an in-network argmax of light: each node delays its reply by an
 * amount that encodes its reading (brighter -> sooner) and stays silent if it
 * overhears a brighter reply, so the network answers with a single packet.
 */
#ifndef FP_COMMON_H
#define FP_COMMON_H

#include "contiki.h"
#include <stdint.h>

/* Rime channels (all broadcast). */
#define FP_CMD_CHANNEL     140  /* base  -> nodes : commands               */
#define FP_RESP_CHANNEL    142  /* nodes -> base  : responses (nodes also  */
                                /*                  overhear each other)   */

#define FP_RADIO_CHANNEL   26       /* IEEE 802.15.4 channel */
#define FP_BASE_ID         1        /* base station node id  */
#define FP_MAGIC           0x4650u  /* "FP" packet tag       */

/* message types */
enum {
  FP_TYPE_CMD  = 1,   /* base -> node */
  FP_TYPE_RESP = 2    /* node -> base */
};

/* command ids */
enum {
  FP_CMD_GET_LED     = 1,  /* state of the 3 LEDs (0..7)     */
  FP_CMD_SET_LED     = 2,  /* set the 3 LEDs from arg (0..7) */
  FP_CMD_GET_VOLTAGE = 3,  /* supply voltage                 */
  FP_CMD_GET_TEMP    = 4,  /* internal (CPU) temperature     */
  FP_CMD_GET_CHANNEL = 5,  /* radio channel                  */
  FP_CMD_GET_TXPOWER = 6,  /* radio tx power                 */
  FP_CMD_AGG_TICK    = 7   /* Part-2 round trigger / reply   */
};

/*
 * Part-2 tuning. A node's reply delay (clock ticks) encodes its light reading:
 *   delay = (AGG_REF_LIGHT - light) / AGG_SCALE + (id-2) * AGG_TIE
 * Observed light: ~130 ambient, ~730 under a phone flashlight. The lit node
 * answers ~30 ticks (~230 ms) before the dimmest, so it suppresses the rest;
 * (id-2)*TIE breaks exact ties. The base collects replies for AGG_WINDOW.
 */
#define AGG_REF_LIGHT  800   /* light value mapped to delay 0      */
#define AGG_SCALE      20    /* light units per tick (~7.8 ms)     */
#define AGG_TIE        4     /* per-node tie-break spacing (ticks) */
#define AGG_MIN_DELAY  2     /* delay floor (never <= 0)           */
#define AGG_WINDOW     (CLOCK_SECOND * 2 / 5)  /* ~400 ms, > dimmest delay (~45 ticks) */

/*
 * Fixed 14-byte message. Base and nodes share one MSP430 build, so a raw
 * struct copy over the air is safe.
 */
struct fp_msg {
  uint16_t magic;   /* 0x4650 ("FP")                       */
  uint8_t  type;    /* FP_TYPE_*                           */
  uint8_t  cmd;     /* FP_CMD_*                            */
  uint8_t  arg;     /* set-led value / agg round number    */
  uint8_t  node;    /* responder id                        */
  int32_t  value;   /* primary result (e.g. light reading) */
  int32_t  value2;  /* secondary result (e.g. raw ADC)     */
};

/* Sensor conversions (ADC ref 2.5V, 12-bit). */

/* Battery: board divides Vbatt by 2, so Vbatt = raw/4096 * 2.5V * 2 (mV). */
static inline int32_t fp_battery_mv(int raw) {
  return ((int32_t)raw * 5000L) / 4096L;
}

/* MSP430 temp sensor: Vsensor = 0.00355*T + 0.986V; returns centi-degrees C. */
static inline int32_t fp_temp_centiC(int raw) {
  int32_t vmv = ((int32_t)raw * 2500L) / 4096L;   /* sensor voltage (mV) */
  return (vmv - 986L) * 10000L / 355L;            /* centi-degrees C     */
}

#endif /* FP_COMMON_H */
