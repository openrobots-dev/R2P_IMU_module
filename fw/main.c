/*
 ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
 2011 Giovanni Di Sirio.

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

#include <stdlib.h>
#include <stdio.h>

#include "ch.h"
#include "hal.h"
#include "halconf.h"
#include "test.h"
#include "shell.h"
#include "chprintf.h"

#include "l3g4200d.h"
#include "lsm303.h"
#include "madgwick.h"
#include "mahony.h"

#define WA_SIZE_256B      THD_WA_SIZE(256)
#define WA_SIZE_1K      THD_WA_SIZE(1024)

static msg_t stream_gyro_thread(void *arg);
static msg_t stream_acc_thread(void *arg);
static msg_t stream_mag_thread(void *arg);
static msg_t stream_raw_thread(void *arg);
static msg_t stream_madgwick_thread(void *arg);
static msg_t stream_mahony_thread(void *arg);
static msg_t stream_tilt_thread(void *arg);

Thread *gyrotp = NULL;
Thread *acctp = NULL;
Thread *magtp = NULL;
uint16_t period;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(4096)
#define TEST_WA_SIZE    THD_WA_SIZE(1024)

static void cmd_mem(BaseChannel *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseChannel *chp, int argc, char *argv[]) {
  static const char *states[] =
    {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n", (uint32_t)tp,
             (uint32_t)tp->p_ctx.r13, (uint32_t)tp->p_prio,
             (uint32_t)(tp->p_refs - 1), states[tp->p_state],
             (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_test(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  (void)argc;
  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: test\r\n");
    return;
  }
  tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriority(), TestThread,
                           chp);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }
  chThdWait(tp);
}

static void cmd_gyro(BaseChannel *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: gyro\r\n");
    return;
  }

  if (gyrotp == NULL)
    gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
}

static void cmd_acc(BaseChannel *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: acc\r\n");
    return;
  }

  if (acctp == NULL)
    acctp = accRun(&I2C_DRIVER, NORMALPRIO);
}

static void cmd_mag(BaseChannel *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mag\r\n");
    return;
  }

  if (magtp == NULL)
    magtp = magRun(&I2C_DRIVER, NORMALPRIO);
}

static void cmd_stream_gyro(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  if (argc != 1) {
    chprintf(chp, "Usage: sg <Hz>\r\n");
    return;
  }

  period = (1000 / atoi(argv[0]));

  if (gyrotp == NULL)
    gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);

  tp = chThdCreateFromHeap(NULL, WA_SIZE_256B, chThdGetPriority(),
                           stream_gyro_thread, (void *)&period);

  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }

  return;
}

static void cmd_stream_acc(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  if (argc != 1) {
    chprintf(chp, "Usage: sa <Hz>\r\n");
    return;
  }

  period = (1000 / atoi(argv[0]));

  if (acctp == NULL)
    acctp = accRun(&I2C_DRIVER, NORMALPRIO);

  tp = chThdCreateFromHeap(NULL, WA_SIZE_256B, NORMALPRIO, stream_acc_thread,
                           (void *)&period);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }

  return;
}

static void cmd_stream_mag(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  if (argc != 1) {
    chprintf(chp, "Usage: sm <Hz>\r\n");
    return;
  }

  period = (1000 / atoi(argv[0]));

  if (magtp == NULL)
    magtp = magRun(&I2C_DRIVER, NORMALPRIO);

  tp = chThdCreateFromHeap(NULL, WA_SIZE_1K, chThdGetPriority(),
                           stream_mag_thread, (void *)&period);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }

  return;
}

static void cmd_stream_raw(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  if (argc != 1) {
    chprintf(chp, "Usage: smad <Hz>\r\n");
    return;
  }

  period = (1000 / atoi(argv[0]));

  if (gyrotp == NULL)
    gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
  if (acctp == NULL)
    acctp = accRun(&I2C_DRIVER, NORMALPRIO);
  if (magtp == NULL)
    magtp = magRun(&I2C_DRIVER, NORMALPRIO);

  tp = chThdCreateFromHeap(NULL, WA_SIZE_1K, NORMALPRIO, stream_raw_thread,
                           (void *)&period);

  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }

  return;
}

static void cmd_stream_madgwick(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  if (argc != 1) {
    chprintf(chp, "Usage: smad <Hz>\r\n");
    return;
  }

  period = (1000 / atoi(argv[0]));

  if (gyrotp == NULL)
    gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
  if (acctp == NULL)
    acctp = accRun(&I2C_DRIVER, NORMALPRIO);
  if (magtp == NULL)
    magtp = magRun(&I2C_DRIVER, NORMALPRIO);

  tp = chThdCreateFromHeap(NULL, WA_SIZE_1K, NORMALPRIO, stream_madgwick_thread,
                           (void *)&period);

  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }

  return;
}

static void cmd_stream_mahony(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  if (argc != 1) {
    chprintf(chp, "Usage: smah <Hz>\r\n");
    return;
  }

  period = (1000 / atoi(argv[0]));

  if (gyrotp == NULL)
    gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
  if (acctp == NULL)
    acctp = accRun(&I2C_DRIVER, NORMALPRIO);
  if (magtp == NULL)
    magtp = magRun(&I2C_DRIVER, NORMALPRIO);

  tp = chThdCreateFromHeap(NULL, WA_SIZE_1K, NORMALPRIO, stream_mahony_thread,
                           (void *)&period);

  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }

  return;
}

static void cmd_stream_tilt(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  if (argc != 1) {
    chprintf(chp, "Usage: stilt <Hz>\r\n");
    return;
  }

  period = (1000 / atoi(argv[0]));

  if (gyrotp == NULL)
    gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
  if (acctp == NULL)
    acctp = accRun(&I2C_DRIVER, NORMALPRIO);

  tp = chThdCreateFromHeap(NULL, WA_SIZE_1K, NORMALPRIO, stream_tilt_thread,
                           (void *)&period);

  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }

  return;
}

static void cmd_reset(BaseChannel *chp, int argc, char *argv[]) {

  stm32_reset();

  return;
}

static const ShellCommand commands[] =
  {
    {"mem", cmd_mem},
     {"threads", cmd_threads},
     {"test", cmd_test},
     {"gyro", cmd_gyro},
     {"acc", cmd_acc},
     {"mag", cmd_mag},
     {"sg", cmd_stream_gyro},
     {"sa", cmd_stream_acc},
     {"sm", cmd_stream_mag},
     {"sraw", cmd_stream_raw},
     {"smad", cmd_stream_madgwick},
     {"smah", cmd_stream_mahony},
     {"stilt", cmd_stream_tilt},
     {"reset", cmd_reset},
     {NULL, NULL}};

static const ShellConfig shell_cfg1 =
  {(BaseChannel *)&SERIAL_DRIVER, commands};

/*===========================================================================*/
/* I2C related.                                                              */
/*===========================================================================*/

/* I2C1 */
//static const I2CConfig i2c1cfg =
//  {OPMODE_I2C, 400000, FAST_DUTY_CYCLE_16_9, };
/* I2C1 */
static const I2CConfig i2c1cfg =
  {OPMODE_I2C, 400000, FAST_DUTY_CYCLE_16_9};
/*===========================================================================*/
/* SPI related.                                                              */
/*===========================================================================*/

/* SPI1 configuration
 * Speed 9MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
 */
static const SPIConfig spi1cfg =
  {NULL, /* HW dependent part.*/GYRO_GPIO, GYRO_CS, SPI_CR1_BR_1 | SPI_CR1_CPOL
       | SPI_CR1_CPHA};

/*===========================================================================*/
/* EXTI related.                                                             */
/*===========================================================================*/

static const EXTConfig extcfg =
  {
    {
      {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_RISING_EDGE, l3g4200d_drdy_callback},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_FALLING_EDGE, lsm303_mag_drdy_cb},
       {EXT_CH_MODE_RISING_EDGE, lsm303_acc_int1_cb},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL},
       {EXT_CH_MODE_DISABLED, NULL}, },
   EXT_MODE_EXTI(0,
       EXT_MODE_GPIOB,
       0,
       0,
       0,
       EXT_MODE_GPIOB,
       EXT_MODE_GPIOB,
       0,
       0,
       0,
       0,
       0,
       0,
       0,
       0,
       0)};

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;

  chRegSetThreadName("blinker");
  while (TRUE) {
    palTogglePad(LED_GPIO, LED1);
    chThdSleepMilliseconds(500);
  }
  return 0;
}

/*
 * Gyroscope thread
 */
extern gyro_data_t gyro_data;

static msg_t stream_gyro_thread(void *arg) {
  uint16_t period = *(uint16_t *)arg;
  systime_t time = chTimeNow();

  chRegSetThreadName("l3g4200d_stream_gyro");

  while (TRUE) {
    chprintf((BaseChannel *)&SERIAL_DRIVER, "%6d %5d %5d %5d\r\n", (int)time,
             gyro_data.x, gyro_data.y, gyro_data.z);
    time += MS2ST(period);
    chThdSleepUntil(time);
  }
  return 0;
}

/*
 * Accelerometer thread
 */
extern acc_data_t acc_data;
extern uint8_t status_a;

static msg_t stream_acc_thread(void *arg) {
  uint16_t period = *(uint16_t *)arg;
  systime_t time = chTimeNow();

  chRegSetThreadName("lsm303_stream_acc");

  while (TRUE) {
    chprintf((BaseChannel *)&SERIAL_DRIVER, "%6d %5d %5d %5d %x\r\n", (int)time,
             acc_data.x, acc_data.y, acc_data.z, status_a);
    time += MS2ST(period);
    chThdSleepUntil(time);
  }
  return 0;
}

/*
 * Magnetometer thread
 */
extern mag_data_t mag_data;
extern uint8_t status_m;

static msg_t stream_mag_thread(void *arg) {
  char buf[128];
  uint16_t period = *(uint16_t *)arg;
  systime_t time = chTimeNow();

  chRegSetThreadName("lsm303_stream_mag");

  while (TRUE) {
    chprintf((BaseChannel *)&SERIAL_DRIVER, "%6d %5d %5d %5d %x\r\n", (int)time,
             mag_data.x, mag_data.y, mag_data.z, status_m);
    time += MS2ST(period);
    chThdSleepUntil(time);
  }
  return 0;
}

/*
 * Stream thread
 */
static msg_t stream_raw_thread(void *arg) {
  char buffer[256];
  uint16_t period = *(uint16_t *)arg;
  systime_t time = chTimeNow();

  while (TRUE) {
    sprintf(buffer, "%6d %f %f %f %f %f %f %d %d %d\r\n", (int)time,
            gyro_data.x / 57.143, gyro_data.y / 57.143, gyro_data.z / 57.143,
            acc_data.x / 1000.0, acc_data.y / 1000.0, acc_data.z / 1000.0,
            mag_data.x, mag_data.y, mag_data.z);
    chprintf((BaseChannel *)&SERIAL_DRIVER, buffer);
    time += MS2ST(period);
    chThdSleepUntil(time);
  }

  return 0;
}

/*
 * Madgwick stream thread
 */
static msg_t stream_madgwick_thread(void *arg) {
  attitude_t attitude_data;
  char buffer[64];
  uint16_t period = *(uint16_t *)arg;
  systime_t time = chTimeNow();

  while (TRUE) {
    float mx = ((float)mag_data.x - (-440.0)) / (510 - (-440)) * 2 - 1.0;
    float my = ((float)mag_data.y - (-740.0)) / (380 - (-740)) * 2 - 1.0;
    float mz = ((float)mag_data.z - (-500.0)) / (500 - (-500)) * 2 - 1.0;
    MadgwickAHRSupdate((-gyro_data.x / 57.143) * 3.141592 / 180.0,
                       (gyro_data.y / 57.143) * 3.141592 / 180.0,
                       -(gyro_data.z / 57.143) * 3.141592 / 180.0,
                       -acc_data.x / 1000.0, acc_data.y / 1000.0,
                       acc_data.z / 1000.0, mx, -my, -mz);
    getMadAttitude(&attitude_data);
    sprintf(buffer, "%6d %f %f %f\r\n", (int)time, attitude_data.roll,
            attitude_data.pitch, attitude_data.yaw);
    chprintf((BaseChannel *)&SERIAL_DRIVER, buffer);
    time += MS2ST(period);
    chThdSleepUntil(time);
  }

  return 0;
}

/*
 * Mahony stream thread
 */
static msg_t stream_mahony_thread(void *arg) {
  attitude_t attitude_data;
  char buffer[64];
  uint16_t period = *(uint16_t *)arg;
  systime_t time = chTimeNow();

  while (TRUE) {
    float mx = ((float)mag_data.x - (-440.0)) / (510 - (-440)) * 2 - 1.0;
    float my = ((float)mag_data.y - (-740.0)) / (380 - (-740)) * 2 - 1.0;
    float mz = ((float)mag_data.z - (-500.0)) / (500 - (-500)) * 2 - 1.0;
    MahonyAHRSupdate((-gyro_data.x / 57.143) * 3.141592 / 180.0,
                     (gyro_data.y / 57.143) * 3.141592 / 180.0,
                     -(gyro_data.z / 57.143) * 3.141592 / 180.0,
                     -acc_data.x / 1000.0, acc_data.y / 1000.0,
                     acc_data.z / 1000.0, mx, -my, -mz);
    getMahAttitude(&attitude_data);
    sprintf(buffer, "%6d %f %f %f\r\n", (int)time, attitude_data.roll,
            attitude_data.pitch, attitude_data.yaw);
    chprintf((BaseChannel *)&SERIAL_DRIVER, buffer);
    time += MS2ST(period);
    chThdSleepUntil(time);
  }

  return 0;
}

/*
 * Tilt stream thread
 */
static msg_t stream_tilt_thread(void *arg) {
  attitude_t attitude_data;
  char buffer[64];
  uint16_t period = *(uint16_t *)arg;
  systime_t time = chTimeNow();

  while (TRUE) {
    MahonyAHRSupdateIMU(0, (gyro_data.y / 57.143) * 3.141592 / 180.0, 0,
                        -acc_data.x / 1000.0, 0, acc_data.z / 1000.0);
    getMahAttitude(&attitude_data);
    sprintf(buffer, "%6d %f\r\n", (int)time,
            attitude_data.pitch * 180.0 / 3.141592);
    chprintf((BaseChannel *)&SERIAL_DRIVER, buffer);
    time += MS2ST(period);
    chThdSleepUntil(time);
  }

  return 0;
}

void stm32_reset(void) {

  chThdSleep(MS2ST(10));

  /* Ensure completion of memory access. */
  __DSB();

  /* Generate reset by setting VECTRESETK and SYSRESETREQ, keeping priority group unchanged.
   * If only SYSRESETREQ used, no reset is triggered, discovered while debugging.
   * If only VECTRESETK is used, if you want to read the source of the reset afterwards
   * from (RCC->CSR & RCC_CSR_SFTRSTF),
   * it won't be possible to see that it was a software-triggered reset.
   * */

  SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)
      | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) | SCB_AIRCR_VECTRESET_Msk
      | SCB_AIRCR_SYSRESETREQ_Msk);

  /* Ensure completion of memory access. */
  __DSB();

  /* Wait for reset. */
  while (1)
    ;
}

/*
 * Application entry point.
 */
int main(void) {
  Thread *shelltp = NULL;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SERIAL_DRIVER, NULL);

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Activates the EXT driver.
   */
  extStart(&EXTD1, &extcfg);

  /*
   * Activates the I2C driver.
   */
  i2cStart(&I2C_DRIVER, &i2c1cfg);

  /*
   * Activates the SPI driver.
   */
  spiStart(&SPI_DRIVER, &spi1cfg);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (TRUE) {
    if (!shelltp)
      shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO - 1);
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);
      shelltp = NULL;
    }
    chThdSleepMilliseconds(200);
  }
}
