/*
 * Final Project - Base station firmware
 * Target: Kmote / TelosB-class MSP430 mote, Contiki 2.7, TARGET=sky.
 *
 * The only mote on USB: it reads command lines from the serial console,
 * broadcasts each to the nodes, and prints their replies.
 *
 * Commands: get-led, set-led <0..7>, get-voltage, get-temp, get-channel,
 * get-tx-power, help, plus Part-2: agg-max / agg-auto on|off.
 *
 * Part-2: the base only sends a round tick and listens. The nodes pick the
 * brightest among themselves (it answers first and suppresses the others), so
 * the base just reports the first packet it hears -- it never computes the max.
 */
#include "contiki.h"
#include "net/rime.h"
#include "net/packetbuf.h"
#include "dev/leds.h"
#include "dev/cc2420.h"
#include "dev/serial-line.h"

#include "fp-common.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*---------------------------------------------------------------------------*/
static struct broadcast_conn bc_cmd;     /* ch 140: send commands     */
static struct broadcast_conn bc_resp;    /* ch 142: receive responses */

/* Part-2 aggregation round state (argmax of light). */
static struct ctimer  agg_window;        /* reply-collection window  */
static struct ctimer  agg_resend;        /* re-broadcast the command */
static uint8_t        agg_active;
static uint8_t        agg_round;
static int32_t        agg_best_val;
static uint8_t        agg_best_node;
static uint8_t        agg_replies;

/* Continuous mode: re-run a round automatically so the winner LED tracks the
 * flashlight live, without typing agg-max each time. */
static uint8_t        agg_auto;
#define AGG_AUTO_PERIOD  (CLOCK_SECOND / 10)  /* gap between rounds (~0.5s total) */
/*---------------------------------------------------------------------------*/
static void
print_help(void)
{
  printf("commands: get-led | set-led <0..7> | get-voltage | get-temp | "
         "get-channel | get-tx-power\n");
  printf("aggregation: agg-max (one round) | "
         "agg-auto on|off (continuous; brightest node self-lights, tracks flashlight)\n");
}
/*---------------------------------------------------------------------------*/
/* Pretty-print a Part-1 node response. */
static void
print_response(const struct fp_msg *m)
{
  long v = (long)m->value;

  switch(m->cmd) {
  case FP_CMD_GET_LED:
    printf("node %u, led %ld\n", (unsigned)m->node, v);
    break;
  case FP_CMD_SET_LED:
    /* set-led just changes the LEDs -- no per-node echo (use get-led). */
    break;
  case FP_CMD_GET_VOLTAGE:
    printf("node %u, voltage %ld.%03ldV\n",
           (unsigned)m->node, v / 1000, v % 1000);
    break;
  case FP_CMD_GET_TEMP: {
    long frac = v % 100;
    if(frac < 0) {
      frac = -frac;
    }
    printf("node %u, temperature %ld.%02ldC\n",
           (unsigned)m->node, v / 100, frac);
    break;
  }
  case FP_CMD_GET_CHANNEL:
    printf("node %u, channel %ld\n", (unsigned)m->node, v);
    break;
  case FP_CMD_GET_TXPOWER:
    printf("node %u, tx-power %ld\n", (unsigned)m->node, v);
    break;
  default:
    printf("node %u, (unknown response cmd=%u)\n",
           (unsigned)m->node, (unsigned)m->cmd);
    break;
  }
}
/*---------------------------------------------------------------------------*/
static void
send_command(uint8_t cmd, uint8_t arg)
{
  struct fp_msg m;
  memset(&m, 0, sizeof(m));
  m.magic = FP_MAGIC;
  m.type  = FP_TYPE_CMD;
  m.cmd   = cmd;
  m.arg   = arg;
  packetbuf_clear();
  packetbuf_copyfrom(&m, sizeof(m));
  broadcast_send(&bc_cmd);
}
/*---------------------------------------------------------------------------*/
/* Re-broadcast the current round trigger once, for robustness. */
static void
agg_resend_cb(void *ptr)
{
  if(agg_active) {
    send_command(FP_CMD_AGG_TICK, agg_round);
  }
}
/*---------------------------------------------------------------------------*/
static void start_agg(void);   /* fwd: agg_window_cb re-arms a round in auto mode */
static void agg_auto_cb(void *ptr) { start_agg(); }   /* ctimer-shaped wrapper */
static uint8_t agg_last_winner;  /* for auto mode: only print on change */
/*---------------------------------------------------------------------------*/
/* End of the collection window: report whichever node we heard from (the nodes
 * already picked the brightest among themselves). */
static void
agg_window_cb(void *ptr)
{
  agg_active = 0;

  if(agg_replies > 0) {
    /* In auto mode print only when the winner changes, to avoid console spam. */
    if(!agg_auto || agg_best_node != agg_last_winner) {
      printf("  winner = node %u  (light %ld)   [%u packet%s received]\n",
             (unsigned)agg_best_node, (long)agg_best_val,
             (unsigned)agg_replies, agg_replies == 1 ? "" : "s");
    }
    agg_last_winner = agg_best_node;
  } else if(!agg_auto) {
    printf("  agg: no reply (try again)\n");
  }

  if(agg_auto) {
    ctimer_set(&agg_window, AGG_AUTO_PERIOD, agg_auto_cb, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
start_agg(void)
{
  agg_round++;
  agg_active    = 1;
  agg_replies   = 0;
  agg_best_val  = -1;
  agg_best_node = 0;

  send_command(FP_CMD_AGG_TICK, agg_round);
  ctimer_set(&agg_resend, CLOCK_SECOND / 32, agg_resend_cb, NULL); /* ~31 ms */
  ctimer_set(&agg_window, AGG_WINDOW, agg_window_cb, NULL);
}
/*---------------------------------------------------------------------------*/
/* Parse one console line and dispatch the command. */
static void
process_line(char *line)
{
  char *p = line;
  char *cmdline;
  char tok[20];
  int i;
  int arg = 0;
  int has_arg = 0;
  uint8_t cmd = 0;

  while(*p == ' ' || *p == '\t') {
    p++;
  }
  cmdline = p;   /* trimmed command, echoed back below */

  i = 0;
  while(*p && *p != ' ' && i < (int)sizeof(tok) - 1) {
    tok[i++] = *p++;
  }
  tok[i] = '\0';
  while(*p == ' ') {
    p++;
  }
  if(*p) {
    has_arg = 1;
    arg = atoi(p);
  }

  if(tok[0] == '\0') {
    return;
  }

  printf("$ %s\n", cmdline);   /* echo the command exactly as typed */

  if(!strcmp(tok, "get-led")) {
    cmd = FP_CMD_GET_LED;
  } else if(!strcmp(tok, "set-led")) {
    cmd = FP_CMD_SET_LED;
  } else if(!strcmp(tok, "get-voltage")) {
    cmd = FP_CMD_GET_VOLTAGE;
  } else if(!strcmp(tok, "get-temp")) {
    cmd = FP_CMD_GET_TEMP;
  } else if(!strcmp(tok, "get-channel")) {
    cmd = FP_CMD_GET_CHANNEL;
  } else if(!strcmp(tok, "get-tx-power")) {
    cmd = FP_CMD_GET_TXPOWER;
  } else if(!strcmp(tok, "agg-max")) {
    if(!agg_auto) {     /* one-shot (ignored while auto mode drives rounds) */
      start_agg();
    }
    return;
  } else if(!strcmp(tok, "agg-auto")) {
    /* continuous mode: rounds repeat so the winner LED tracks the flashlight */
    if(!strcmp(p, "on")) {
      if(!agg_auto) {
        agg_auto = 1;
        agg_last_winner = 0;
        printf("agg-auto: on (move the flashlight; the red LED follows)\n");
        start_agg();
      }
    } else if(!strcmp(p, "off")) {
      agg_auto = 0;
      printf("agg-auto: off\n");
    } else {
      printf("usage: agg-auto on|off\n");
    }
    return;
  } else if(!strcmp(tok, "help") || !strcmp(tok, "?")) {
    print_help();
    return;
  } else {
    printf("err: unknown command \"%s\" (type help)\n", tok);
    return;
  }

  if(cmd == FP_CMD_SET_LED) {
    if(!has_arg) {
      printf("err: set-led needs an integer 0..7\n");
      return;
    }
    arg &= 0x07;
  }

  send_command(cmd, (uint8_t)arg);
}
/*---------------------------------------------------------------------------*/
/* Response from a node (broadcast on ch 142). */
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

  /* Aggregation reply: the first packet of the current round is the winner
   * (nodes encode value as delay, so the brightest answers first). Late or
   * out-of-round replies are dropped so they never reach the Part-1 printer. */
  if(m.cmd == FP_CMD_AGG_TICK) {
    if(agg_active && m.arg == agg_round) {
      if(agg_replies == 0) {     /* first to arrive = the winner */
        agg_best_val  = m.value;
        agg_best_node = m.node;
      }
      agg_replies++;
    }
    return;
  }

  print_response(&m);
}
static const struct broadcast_callbacks bc_resp_cb = { bc_resp_recv };
static const struct broadcast_callbacks bc_cmd_cb  = { NULL };
/*---------------------------------------------------------------------------*/
PROCESS(fp_base_process, "FP base station");
AUTOSTART_PROCESSES(&fp_base_process);

PROCESS_THREAD(fp_base_process, ev, data)
{
  PROCESS_EXITHANDLER(
    broadcast_close(&bc_cmd);
    broadcast_close(&bc_resp);
  )
  PROCESS_BEGIN();

  cc2420_set_channel(FP_RADIO_CHANNEL);
  cc2420_set_txpower(CC2420_TXPOWER_MAX);

  broadcast_open(&bc_cmd,  FP_CMD_CHANNEL,  &bc_cmd_cb);
  broadcast_open(&bc_resp, FP_RESP_CHANNEL, &bc_resp_cb);

  leds_on(LEDS_RED);   /* base station indicator */

  printf("FP_BASE_READY,node=%u,channel=%u,txpower=%u\n",
         (unsigned)FP_BASE_ID, (unsigned)cc2420_get_channel(),
         (unsigned)cc2420_get_txpower());
  print_help();

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    process_line((char *)data);
  }

  PROCESS_END();
}
