/*
 * Final Project - Sensor node firmware
 * Target: Kmote / TelosB-class MSP430 mote, Contiki 2.7, TARGET=sky.
 *
 * Part-1: answers commands broadcast by the base (get/set-led, get-voltage,
 * get-temp, get-channel, get-tx-power).
 *
 * Part-2 (in-network argmax of light): on a round tick the node delays its
 * reply to encode its light reading (brighter -> sooner) and stays silent if
 * it overhears a brighter reply, so only the brightest node answers -- and it
 * pulses its own red LED. See fp-common.h.
 *
 * Each node is flashed with a unique NODE_ID (2..5), e.g. make NODE_ID=3
 */
#include "contiki.h"
#include "net/rime.h"
#include "net/packetbuf.h"
#include "dev/leds.h"
#include "dev/cc2420.h"
#include "dev/battery-sensor.h"
#include "dev/temperature-sensor.h"
#include "dev/light-sensor.h"

#include "fp-common.h"

#include <stdio.h>
#include <string.h>

#ifndef NODE_ID
#define NODE_ID 2
#endif

/*---------------------------------------------------------------------------*/
/*
 * LED colour control. set-led <v> is a 3-bit value in physical left-to-right
 * order RED-GREEN-BLUE (bit2=RED, bit1=GREEN, bit0=BLUE). This Kmote wires
 * GREEN and BLUE opposite to Contiki's default bit order, so every value is
 * mapped through fp_led_table[] (and inverted back in get-led). Verified on 4 motes.
 */
#define LED_GREEN  0x01   /* LEDS_GREEN  -> P5.5 -> physical GREEN */
#define LED_BLUE   0x02   /* LEDS_YELLOW -> P5.6 -> physical BLUE  */
#define LED_RED    0x04   /* LEDS_RED    -> P5.4 -> physical RED   */

/* set-led value (0..7) -> physical LED mask. */
static const unsigned char fp_led_table[8] = {
  /* 0  000 */ 0,
  /* 1  001 */ LED_BLUE,
  /* 2  010 */ LED_GREEN,
  /* 3  011 */ LED_GREEN | LED_BLUE,
  /* 4  100 */ LED_RED,
  /* 5  101 */ LED_RED   | LED_BLUE,
  /* 6  110 */ LED_RED   | LED_GREEN,
  /* 7  111 */ LED_RED   | LED_GREEN | LED_BLUE,
};
/*---------------------------------------------------------------------------*/
static struct broadcast_conn bc_cmd;      /* ch 140: receive commands           */
static struct broadcast_conn bc_resp;     /* ch 142: send + overhear responses  */

/* Part-1 staggered response. */
static struct ctimer  resp_timer;
static struct fp_msg  pending_resp;
static uint8_t        have_pending;

/* Part-2 in-network aggregation (argmax of light) state. */
static struct ctimer  agg_timer;
static uint8_t        agg_pending;   /* a reply is scheduled but not yet sent */
static uint8_t        agg_round;     /* round id (from the command's arg)     */
static int32_t        agg_value;     /* my light reading for this round       */

/* Winner self-indication: the node that transmits a round's reply pulses its
 * own red LED -- a blinking node IS the locally decided argmax. */
static struct ctimer  win_led_timer;
static void
win_led_off_cb(void *ptr)
{
  leds_off(LED_RED);
}
static void
pulse_win_led(void)
{
  leds_on(LED_RED);
  ctimer_set(&win_led_timer, CLOCK_SECOND / 6, win_led_off_cb, NULL);  /* ~165 ms */
}
/*---------------------------------------------------------------------------*/
static int
read_battery_raw(void)
{
  return battery_sensor.value(0);
}
static int
read_temp_raw(void)
{
  return temperature_sensor.value(0);
}
static int
read_light(void)
{
  return light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR);
}
/*---------------------------------------------------------------------------*/
/* Build a Part-1 response message (and perform any action). */
static void
fill_response(struct fp_msg *r, const struct fp_msg *cmd)
{
  memset(r, 0, sizeof(*r));
  r->magic = FP_MAGIC;
  r->type  = FP_TYPE_RESP;
  r->cmd   = cmd->cmd;
  r->node  = NODE_ID;

  switch(cmd->cmd) {
  case FP_CMD_GET_LED: {
    /* Report the same [RED GREEN BLUE] 3-bit value that set-led accepts. */
    unsigned char p = leds_get();
    r->value = ((p & LED_RED)   ? 0x04 : 0)
             | ((p & LED_GREEN) ? 0x02 : 0)
             | ((p & LED_BLUE)  ? 0x01 : 0);
    break;
  }
  case FP_CMD_SET_LED: {
    unsigned char v = cmd->arg & 0x07;
    leds_off(LEDS_ALL);
    leds_on(fp_led_table[v]);     /* explicit value -> physical colour */
    r->arg   = v;
    r->value = v;                 /* echo the requested 3-bit value */
    break;
  }
  case FP_CMD_GET_VOLTAGE: {
    int raw = read_battery_raw();
    r->value  = fp_battery_mv(raw);   /* millivolts */
    r->value2 = raw;                  /* raw ADC    */
    break;
  }
  case FP_CMD_GET_TEMP: {
    int raw = read_temp_raw();
    r->value  = fp_temp_centiC(raw);  /* centi-degrees C */
    r->value2 = raw;                  /* raw ADC         */
    break;
  }
  case FP_CMD_GET_CHANNEL:
    r->value = cc2420_get_channel();
    break;
  case FP_CMD_GET_TXPOWER:
    r->value = cc2420_get_txpower();
    break;
  default:
    r->cmd = 0;                       /* unknown -> ignored by base */
    break;
  }
}
/*---------------------------------------------------------------------------*/
/* Broadcast the pending Part-1 response to the base. */
static void
send_response_cb(void *ptr)
{
  if(!have_pending) {
    return;
  }
  packetbuf_clear();
  packetbuf_copyfrom(&pending_resp, sizeof(pending_resp));
  broadcast_send(&bc_resp);
  have_pending = 0;
}
/*---------------------------------------------------------------------------*/
/* Send my aggregation reply (the timer fired without anyone suppressing me). */
static void
agg_send_cb(void *ptr)
{
  struct fp_msg r;
  if(!agg_pending) {
    return;
  }
  memset(&r, 0, sizeof(r));
  r.magic = FP_MAGIC;
  r.type  = FP_TYPE_RESP;
  r.cmd   = FP_CMD_AGG_TICK;
  r.arg   = agg_round;
  r.node  = NODE_ID;
  r.value = agg_value;
  packetbuf_clear();
  packetbuf_copyfrom(&r, sizeof(r));
  broadcast_send(&bc_resp);
  agg_pending = 0;

  /* I won this round -> announce it on my own LED (no base involvement). */
  pulse_win_led();
}
/*---------------------------------------------------------------------------*/
/* Start a round: schedule a reply whose delay encodes my light reading
 * (brighter -> smaller delay, so the brightest node answers first). */
static void
handle_agg(const struct fp_msg *cmd)
{
  int32_t light, d;

  /* Ignore a repeated command for a round we are already contending in
   * (the base sends each round twice for robustness). */
  if(agg_pending && agg_round == cmd->arg) {
    return;
  }

  light = read_light();
  agg_round   = cmd->arg;
  agg_value   = light;
  agg_pending = 1;

  d = (AGG_REF_LIGHT - light) / AGG_SCALE;      /* brighter -> smaller delay */
  if(d < AGG_MIN_DELAY) {
    d = AGG_MIN_DELAY;
  }
  d += (int32_t)(NODE_ID - 2) * AGG_TIE;        /* tie-break by node id */

  ctimer_set(&agg_timer, (clock_time_t)d, agg_send_cb, NULL);
}
/*---------------------------------------------------------------------------*/
static void
handle_command(const struct fp_msg *cmd)
{
  if(cmd->magic != FP_MAGIC || cmd->type != FP_TYPE_CMD) {
    return;
  }

  if(cmd->cmd == FP_CMD_AGG_TICK) {
    handle_agg(cmd);
    return;
  }

  /* Part-1 command: build the response and send it after a per-node stagger
   * so the nodes do not all answer a broadcast at the exact same instant. */
  fill_response(&pending_resp, cmd);
  have_pending = 1;
  ctimer_set(&resp_timer,
             1 + (NODE_ID - FP_BASE_ID) * (CLOCK_SECOND / 16),
             send_response_cb, NULL);
}
/*---------------------------------------------------------------------------*/
static void
bc_cmd_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  struct fp_msg cmd;
  if(packetbuf_datalen() < (int)sizeof(cmd)) {
    return;
  }
  memcpy(&cmd, packetbuf_dataptr(), sizeof(cmd));
  handle_command(&cmd);
}
/*---------------------------------------------------------------------------*/
/*
 * Suppression: while waiting to send our own reply, if we overhear another
 * node's reply for the same round that is at least as bright, we cancel ours.
 */
static void
bc_resp_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  struct fp_msg m;
  if(packetbuf_datalen() < (int)sizeof(m)) {
    return;
  }
  memcpy(&m, packetbuf_dataptr(), sizeof(m));
  if(m.magic != FP_MAGIC || m.type != FP_TYPE_RESP) {
    return;
  }
  if(!agg_pending || m.node == NODE_ID || m.arg != agg_round) {
    return;
  }
  if(m.cmd != FP_CMD_AGG_TICK) {
    return;
  }
  /* Same round: someone at least as bright already answered -> I stay silent. */
  if(m.value >= agg_value) {
    ctimer_stop(&agg_timer);
    agg_pending = 0;
  }
}
static const struct broadcast_callbacks bc_cmd_cb  = { bc_cmd_recv };
static const struct broadcast_callbacks bc_resp_cb = { bc_resp_recv };
/*---------------------------------------------------------------------------*/
PROCESS(fp_node_process, "FP sensor node");
AUTOSTART_PROCESSES(&fp_node_process);

PROCESS_THREAD(fp_node_process, ev, data)
{
  PROCESS_EXITHANDLER(
    broadcast_close(&bc_cmd);
    broadcast_close(&bc_resp);
  )
  PROCESS_BEGIN();

  cc2420_set_channel(FP_RADIO_CHANNEL);
  cc2420_set_txpower(CC2420_TXPOWER_MAX);

  SENSORS_ACTIVATE(battery_sensor);
  SENSORS_ACTIVATE(temperature_sensor);
  SENSORS_ACTIVATE(light_sensor);

  broadcast_open(&bc_cmd,  FP_CMD_CHANNEL,  &bc_cmd_cb);
  broadcast_open(&bc_resp, FP_RESP_CHANNEL, &bc_resp_cb);

  printf("FP_NODE_READY,node=%u,channel=%u,txpower=%u\n",
         (unsigned)NODE_ID, (unsigned)cc2420_get_channel(),
         (unsigned)cc2420_get_txpower());

  /*
   * Boot-time identification: blink the BLUE LED NODE_ID times so you can tell
   * which physical mote is which (node 2 blinks twice, node 5 five times).
   * Purely a human aid; runs once at startup, then the node goes event-driven.
   */
  {
    static struct etimer bt;
    static uint8_t k;
    for(k = 0; k < NODE_ID; k++) {
      leds_on(LED_BLUE);
      etimer_set(&bt, CLOCK_SECOND / 6);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bt));
      leds_off(LED_BLUE);
      etimer_set(&bt, CLOCK_SECOND / 6);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bt));
    }
  }

  /* Commands are handled event-driven in the callbacks above. */
  while(1) {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}
