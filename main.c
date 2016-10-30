/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "web.h"
#include <string.h>
#include "chregistry.h"
#include "chprintf.h"


#include "hal.h"
#include "test.h"
#include "ch.h"

#include "lwipthread.h"

#include "web/web.h"

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      8

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

#if LWIP_NETCONN
static struct netconn *conn;
static struct netbuf *buf;
static struct ip_addr *addr;
static unsigned short port;
/**
 * Stack area for the udp thread.
 */
THD_WORKING_AREA(wa_udp_echo_server, UDP_THREAD_STACK_SIZE);

/**
 * UDP echo server thread.
 */
THD_FUNCTION(udp_echo_server, p)
{
  err_t err, recv_err;

  LWIP_UNUSED_ARG(p);

  conn = netconn_new(NETCONN_UDP);
  if (conn!= NULL)
  {
    err = netconn_bind(conn, IP_ADDR_ANY, UDP_THREAD_PORT);
    if (err == ERR_OK)
    {
      while (1)
      {
        recv_err = netconn_recv(conn, &buf);

        if (recv_err == ERR_OK)
        {
          addr = netbuf_fromaddr(buf);
          port = netbuf_fromport(buf);
          netconn_connect(conn, addr, port);
          buf->addr.addr = 0;
          netconn_send(conn,buf);
          netbuf_delete(buf);
        }
      }
    }
    else
    {
      netconn_delete(conn);
      // printf("can not bind netconn");
    }
  }
  else
  {
    // printf("can create new UDP netconn");
  }
  // return MSG_OK;
}

THD_WORKING_AREA(wa_tcp_echo_server, TCP_THREAD_STACK_SIZE);

/**
 * TCP echo server thread.
 */
THD_FUNCTION(tcp_echo_server,p)
{
    struct netconn *conn, *newconn;
  err_t err, accept_err;
  struct netbuf *buf;
  void *data;
  u16_t len;
  err_t recv_err;

  LWIP_UNUSED_ARG(p);

  /* Create a new connection identifier. */
  conn = netconn_new(NETCONN_TCP);

  if (conn!=NULL)
  {
    /* Bind connection to well known port number 7. */
    err = netconn_bind(conn, NULL, TCP_THREAD_PORT);

    if (err == ERR_OK)
    {
      /* Tell connection to go into listening mode. */
      netconn_listen(conn);

      while (1)
      {
        /* Grab new connection. */
         accept_err = netconn_accept(conn, &newconn);

        /* Process the new connection. */
        if (accept_err == ERR_OK)
        {
          recv_err = netconn_recv(newconn, &buf);
          while ( recv_err == ERR_OK)
          {
            do
            {
              netbuf_data(buf, &data, &len);
              netconn_write(newconn, data, len, NETCONN_COPY);

            }
            while (netbuf_next(buf) >= 0);

            netbuf_delete(buf);
            recv_err = netconn_recv(newconn, &buf);
          }

          /* Close connection and discard connection identifier. */
          netconn_close(newconn);
          netconn_delete(newconn);
        }
      }
    }
    else
    {
      netconn_delete(newconn);
      // printf(" can not bind TCP netconn");
    }
  }
  else
  {
    // printf("can not create TCP netconn");
  }
  // return MSG_OK;
}

#endif /* LWIP_NETCONN */

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN12)
};




/*
 * Orange LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE)
  {
    palClearPad(GPIOD, GPIOD_LED6);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOD, GPIOD_LED6);
    chThdSleepMilliseconds(500);
  }
}

static void led5off(void *arg) {

  (void)arg;
  palClearPad(GPIOD, GPIOD_LED5);
}

/********************************************Thread2 implementation*******************************/

thread_t *thread2_p = NULL;     //pointer thread2 az noe "thread_t"(ghabele tavajoh ke in type_def dar chibi3 hast)


static THD_WORKING_AREA(waThread2, 128);//ekhtesase fazaye poshte hafeze be thread2 be andaze 128byte
static THD_FUNCTION(Thread2, arg) {

  (void)arg;

//  chRegSetThreadName("touchpad");    // baraye nam gozariye thread ha mishe az in estefade kard

  while(TRUE) {
    chEvtWaitAny((eventmask_t)1);  //montazer mimune ta yek event ya hamun msg_t az intrupt berese
    palSetPad(GPIOE,GPIOD_LED6);
    chThdSleepMilliseconds(500);
    palClearPad(GPIOE,GPIOD_LED6);
    adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);  //ba labe bala ravande yek bar ADC anjam mishavad
    chThdSleepMilliseconds(500);

  }
}


/* Triggered when the button is pressed or released. The LED5 is set to ON.*/
static void extcb1(EXTDriver *extp, expchannel_t channel) {
/*  static virtual_timer_t vt4;

  (void)extp;
  (void)channel;

  palSetPad(GPIOD, GPIOD_LED5);
  chSysLockFromISR();
  chVTResetI(&vt4);*/

  /* LED4 set to OFF after 200mS.*/
/*  chVTSetI(&vt4, MS2ST(200), led5off, NULL);
  chSysUnlockFromISR();*/

  (void)extp;
  (void)channel;

  chSysLockFromISR();
  chEvtSignalI(thread2_p, (eventmask_t)1);  //faal shodan flag baraye thread2
  chSysUnlockFromISR();
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

/*
 * Application entry point.
 */
int main(void)
{

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  lwipInit(NULL);

  /*
   * Setting up analog inputs used by the demo.
   */
  palSetGroupMode(GPIOC, PAL_PORT_BIT(1) | PAL_PORT_BIT(2),
                  0, PAL_MODE_INPUT_ANALOG);

  
  /*
   * our own thread to be called after EXT
   */
  thread2_p = chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL); //thread baraye interrupt ba arzeshe normal
  

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Creates the LWIP threads (it changes priority internally).
   */
  // chThdCreateStatic(wa_lwip_thread, LWIP_THREAD_STACK_SIZE, HIGHPRIO + 1,
  //                   lwip_thread, NULL);

  /*
   * Creates the UDP echo thread (it changes priority internally).
   */
  chThdCreateStatic(wa_udp_echo_server, sizeof(wa_udp_echo_server), NORMALPRIO + 1,
                    udp_echo_server, NULL);


  /*
   * Creates the TCP echo thread (it changes priority internally).
   */
  chThdCreateStatic(wa_tcp_echo_server, sizeof(wa_tcp_echo_server), NORMALPRIO + 1,
                    tcp_echo_server, NULL);

  /*
   * Activates the ADC1 driver and the temperature sensor.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();

  /*
   * Linear conversion.
   */
  adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
  chThdSleepMilliseconds(1000);

  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, 0);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (TRUE)
  {
    // if (palReadPad(GPIOA, GPIOA_BUTTON) == 1)
    //   TestThread(&SD3);
    chThdSleepMilliseconds(500);
  }
}
