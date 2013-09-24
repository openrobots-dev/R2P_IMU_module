/*
Copyright (c) 2010-2013, Politecnico di Milano - http://airlab.elet.polimi.it/.
Copyright (c) 2010-2013 Martino Migliavacca <martino.migliavacca@gmail.com> - http://www.openrobots.com/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

#define ACC_ZERO 38
#define ACC2GRAD 20.0

#define GYRO_ZERO 3
#define GYRO2GRAD -0.0175

#define WA_SIZE_256B      THD_WA_SIZE(256)
#define WA_SIZE_1K      THD_WA_SIZE(1024)

static msg_t stream_gyro_thread(void *arg);
static msg_t stream_acc_thread(void *arg);
static msg_t stream_mag_thread(void *arg);
static msg_t stream_raw_thread(void *arg);
static msg_t stream_madgwick_thread(void *arg);
static msg_t stream_madgwick_tilt_thread(void *arg);
static msg_t stream_mahony_thread(void *arg);
static msg_t stream_tilt_thread(void *arg);
float complemetary_filter_update(uint16_t period);

Thread *gyrotp = NULL;
Thread *acctp = NULL;
Thread *magtp = NULL;
uint16_t period;

void stm32_reset(void) {

	chThdSleep(MS2ST(10) );

	/* Ensure completion of memory access. */
	__DSB();

	/* Generate reset by setting VECTRESETK and SYSRESETREQ, keeping priority group unchanged.
	 * If only SYSRESETREQ used, no reset is triggered, discovered while debugging.
	 * If only VECTRESETK is used, if you want to read the source of the reset afterwards
	 * from (RCC->CSR & RCC_CSR_SFTRSTF),
	 * it won't be possible to see that it was a software-triggered reset.
	 * */

	SCB ->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)
			| (SCB ->AIRCR & SCB_AIRCR_PRIGROUP_Msk)| SCB_AIRCR_VECTRESET_Msk
			| SCB_AIRCR_SYSRESETREQ_Msk);

	/* Ensure completion of memory access. */
	__DSB();

	/* Wait for reset. */
	while (1)
		;
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(4096)
#define TEST_WA_SIZE    THD_WA_SIZE(1024)

static void cmd_mem(BaseSequentialStream*chp, int argc, char *argv[]) {
	size_t n, size;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}
	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream*chp, int argc, char *argv[]) {
	static const char *states[] = { THD_STATE_NAMES };
	Thread *tp;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: threads\r\n");
		return;
	}
	chprintf(chp, "    addr    stack prio refs     state time\r\n");
	tp = chRegFirstThread();
	do {
		chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n", (uint32_t) tp,
				(uint32_t) tp->p_ctx.r13, (uint32_t) tp->p_prio,
				(uint32_t)(tp->p_refs - 1), states[tp->p_state],
				(uint32_t) tp->p_time);
		tp = chRegNextThread(tp);
	} while (tp != NULL);
}

static void cmd_test(BaseSequentialStream*chp, int argc, char *argv[]) {
	Thread *tp;

	(void) argc;
	(void) argv;
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

static void cmd_gyro(BaseSequentialStream*chp, int argc, char *argv[]) {

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: gyro\r\n");
		return;
	}

	if (gyrotp == NULL)
		gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
}

static void cmd_acc(BaseSequentialStream*chp, int argc, char *argv[]) {

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: acc\r\n");
		return;
	}

	if (acctp == NULL)
		acctp = accRun(&I2C_DRIVER, NORMALPRIO);
}

static void cmd_mag(BaseSequentialStream*chp, int argc, char *argv[]) {

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: mag\r\n");
		return;
	}

	if (magtp == NULL)
		magtp = magRun(&I2C_DRIVER, NORMALPRIO);
}

static void cmd_stream_gyro(BaseSequentialStream*chp, int argc, char *argv[]) {
	Thread *tp;

	if (argc != 1) {
		chprintf(chp, "Usage: sg <Hz>\r\n");
		return;
	}

	period = (1000 / atoi(argv[0]));

	if (gyrotp == NULL)
		gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);

	tp = chThdCreateFromHeap(NULL, WA_SIZE_256B, chThdGetPriority(),
			stream_gyro_thread, (void *) &period);

	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}

	return;
}

static void cmd_stream_acc(BaseSequentialStream*chp, int argc, char *argv[]) {
	Thread *tp;

	if (argc != 1) {
		chprintf(chp, "Usage: sa <Hz>\r\n");
		return;
	}

	period = (1000 / atoi(argv[0]));

	if (acctp == NULL)
		acctp = accRun(&I2C_DRIVER, NORMALPRIO);

	tp = chThdCreateFromHeap(NULL, WA_SIZE_256B, NORMALPRIO, stream_acc_thread,
			(void *) &period);
	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}

	return;
}

static void cmd_stream_mag(BaseSequentialStream*chp, int argc, char *argv[]) {
	Thread *tp;

	if (argc != 1) {
		chprintf(chp, "Usage: sm <Hz>\r\n");
		return;
	}

	period = (1000 / atoi(argv[0]));

	if (magtp == NULL)
		magtp = magRun(&I2C_DRIVER, NORMALPRIO);

	tp = chThdCreateFromHeap(NULL, WA_SIZE_1K, chThdGetPriority(),
			stream_mag_thread, (void *) &period);
	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}

	return;
}

static void cmd_stream_raw(BaseSequentialStream*chp, int argc, char *argv[]) {
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
			(void *) &period);

	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}

	return;
}

static void cmd_stream_madgwick(BaseSequentialStream*chp, int argc,
		char *argv[]) {
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

	tp = chThdCreateFromHeap(NULL, WA_SIZE_1K, NORMALPRIO,
			stream_madgwick_thread, (void *) &period);

	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}

	return;
}

static void cmd_stream_madgwick_tilt(BaseSequentialStream*chp, int argc,
		char *argv[]) {
	Thread *tp;

	if (argc != 1) {
		chprintf(chp, "Usage: smad <Hz>\r\n");
		return;
	}

	period = (1000 / atoi(argv[0]));

	tp = chThdCreateFromHeap(NULL, WA_SIZE_1K, NORMALPRIO,
			stream_madgwick_tilt_thread, (void *) &period);

	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}

	return;
}

static void cmd_stream_mahony(BaseSequentialStream*chp, int argc, char *argv[]) {
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
			(void *) &period);

	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}

	return;
}

static void cmd_stream_tilt(BaseSequentialStream*chp, int argc, char *argv[]) {
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
			(void *) &period);

	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}

	return;
}

static void cmd_reset(BaseSequentialStream*chp, int argc, char *argv[]) {

	(void) chp;
	(void) argc;
	(void) argv;
	stm32_reset();
	return;
}

static const ShellCommand commands[] = { { "mem", cmd_mem }, { "threads",
		cmd_threads }, { "test", cmd_test }, { "gyro", cmd_gyro }, { "acc",
		cmd_acc }, { "mag", cmd_mag }, { "sg", cmd_stream_gyro }, { "sa",
		cmd_stream_acc }, { "sm", cmd_stream_mag }, { "sraw", cmd_stream_raw },
		{ "smad", cmd_stream_madgwick }, { "smadtilt", cmd_stream_madgwick_tilt }, { "smah", cmd_stream_mahony }, {
				"stilt", cmd_stream_tilt }, { "reset", cmd_reset },
		{ NULL, NULL } };

static const ShellConfig shell_cfg1 = { (BaseSequentialStream*) &SERIAL_DRIVER,
		commands };

/*===========================================================================*/
/* I2C related.                                                              */
/*===========================================================================*/

/* I2C1 */
//static const I2CConfig i2c1cfg =
//  {OPMODE_I2C, 400000, FAST_DUTY_CYCLE_16_9, };
/* I2C1 */
static const I2CConfig i2c1cfg = { OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2 };

/*===========================================================================*/
/* SPI related.                                                              */
/*===========================================================================*/

/* SPI1 configuration
 * Speed 9MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
 */
static const SPIConfig spi1cfg = { NULL, /* HW dependent part.*/GYRO_GPIO,
		GYRO_CS, SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA };

/*===========================================================================*/
/* EXTI related.                                                             */
/*===========================================================================*/

static const EXTConfig extcfg = { { { EXT_CH_MODE_DISABLED, NULL }, {
		EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOB, l3g4200d_drdy_callback }, {
		EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, {
		EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_FALLING_EDGE
		| EXT_MODE_GPIOB, lsm303_mag_drdy_cb }, { EXT_CH_MODE_RISING_EDGE
		| EXT_MODE_GPIOB, lsm303_acc_int1_cb }, { EXT_CH_MODE_DISABLED, NULL },
		{ EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, {
				EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, {
				EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, {
				EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL } } };

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

	(void) arg;

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
	uint16_t period = *(uint16_t *) arg;
	systime_t time = chTimeNow();

	chRegSetThreadName("l3g4200d_stream_gyro");

	while (TRUE) {
		chprintf((BaseSequentialStream*) &SERIAL_DRIVER, "%6d %5d %5d %5d\r\n",
				(int) time, gyro_data.x, gyro_data.y, gyro_data.z);
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
	uint16_t period = *(uint16_t *) arg;
	systime_t time = chTimeNow();

	chRegSetThreadName("lsm303_stream_acc");

	while (TRUE) {
		chprintf((BaseSequentialStream*) &SERIAL_DRIVER, "%6u %5d %5d %5d\r\n",
				(uint32_t) time, acc_data.x, acc_data.y, acc_data.z);
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
	uint16_t period = *(uint16_t *) arg;
	systime_t time = chTimeNow();

	chRegSetThreadName("lsm303_stream_mag");

	while (TRUE) {
		chprintf((BaseSequentialStream*) &SERIAL_DRIVER, "%6d %5d %5d %5d\r\n",
				(int) time, mag_data.x, mag_data.y, mag_data.z);
		time += MS2ST(period);
		chThdSleepUntil(time);
	}
	return 0;
}

/*
 * Stream thread
 */
static msg_t stream_raw_thread(void *arg) {
	uint16_t period = *(uint16_t *) arg;
	systime_t time = chTimeNow();

	while (TRUE) {
		chprintf((BaseSequentialStream*) &SERIAL_DRIVER,
				"%u %d %d %d %d %d %d %d %d %d\r\n", (uint32_t) time,
				gyro_data.x, gyro_data.y, gyro_data.z, acc_data.x, acc_data.y,
				acc_data.z, mag_data.x, mag_data.y, mag_data.z);
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
	uint16_t period = *(uint16_t *) arg;
	systime_t time = chTimeNow();

	while (TRUE) {
//		float mx = ((float) mag_data.x - (-440.0)) / (510 - (-440)) * 2 - 1.0;
//		float my = ((float) mag_data.y - (-740.0)) / (380 - (-740)) * 2 - 1.0;
//		float mz = ((float) mag_data.z - (-500.0)) / (500 - (-500)) * 2 - 1.0;
		float mx = ((float) (mag_data.x + 76) / 420.3);
		float my = ((float) (mag_data.y + 82) / 455.6);
		float mz = ((float) (mag_data.z - 113) / 401.5);

/*
		MadgwickAHRSupdate((-gyro_data.x / 57.143) * 3.141592 / 180.0,
				(gyro_data.y / 57.143) * 3.141592 / 180.0,
				-(gyro_data.z / 57.143) * 3.141592 / 180.0,
				-acc_data.x / 1000.0, acc_data.y / 1000.0, acc_data.z / 1000.0,
				mx, -my, -mz);
*/
		/* V1 */
//		MadgwickAHRSupdate((-(gyro_data.x - 3) / 57.143) * 3.141592 / 180.0,
//				(-(gyro_data.y - 107) / 57.143) * 3.141592 / 180.0,
//				((gyro_data.z + 118) / 57.143) * 3.141592 / 180.0,
//				(acc_data.x + 17) / 1000.0, (acc_data.y + 9)/ 1000.0, (acc_data.z - 0) / 969.0,
//				mx, my, mz);

		/* V2 */
		MadgwickAHRSupdate(((gyro_data.x + 15) / 57.143) * 3.141592 / 180.0,
				((gyro_data.y + 25) / 57.143) * 3.141592 / 180.0,
				((gyro_data.z - 10) / 57.143) * 3.141592 / 180.0,
				(acc_data.x + 17) / 1000.0, (acc_data.y - 17)/ 1000.0, (acc_data.z - 21) / 1000.0,
				mx, my, mz);

		getMadAttitude(&attitude_data);
		chprintf((BaseSequentialStream*) &SERIAL_DRIVER, "%6d %f %f %f\r\n",
				(int) time, attitude_data.roll, attitude_data.pitch,
				attitude_data.yaw);
		time += MS2ST(period);
		chThdSleepUntil(time);
	}

	return 0;
}

/*
 * Madgwick tilt stream thread
 */
static msg_t stream_madgwick_tilt_thread(void *arg) {
	attitude_t attitude_data;
	uint16_t period = *(uint16_t *) arg;
	systime_t time = chTimeNow();

	while (TRUE) {
		MadgwickAHRSupdateIMU((-gyro_data.x / 57.143) * 3.141592 / 180.0,
				(-gyro_data.y / 57.143) * 3.141592 / 180.0,
				(gyro_data.z / 57.143) * 3.141592 / 180.0,
				acc_data.x / 1000.0, acc_data.y / 1000.0, acc_data.z / 1000.0);
		getMadAttitude(&attitude_data);
		chprintf((BaseSequentialStream*) &SERIAL_DRIVER, "%6d %f %f %f\r\n",
				(int) time, attitude_data.roll, attitude_data.pitch,
				attitude_data.yaw);
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
	uint16_t period = *(uint16_t *) arg;
	systime_t time = chTimeNow();

	while (TRUE) {
		float mx = ((float) mag_data.x - (-440.0)) / (510 - (-440)) * 2 - 1.0;
		float my = ((float) mag_data.y - (-740.0)) / (380 - (-740)) * 2 - 1.0;
		float mz = ((float) mag_data.z - (-500.0)) / (500 - (-500)) * 2 - 1.0;
		MahonyAHRSupdate((-gyro_data.x / 57.143) * 3.141592 / 180.0,
				(gyro_data.y / 57.143) * 3.141592 / 180.0,
				-(gyro_data.z / 57.143) * 3.141592 / 180.0,
				-acc_data.x / 1000.0, acc_data.y / 1000.0, acc_data.z / 1000.0,
				mx, -my, -mz);
		getMahAttitude(&attitude_data);
		chprintf((BaseSequentialStream*) &SERIAL_DRIVER, "%6d %f %f %f\r\n",
				(int) time, attitude_data.roll, attitude_data.pitch,
				attitude_data.yaw);
		time += MS2ST(period);
		chThdSleepUntil(time);
	}

	return 0;
}

/*
 * Tilt stream thread
 */
static msg_t stream_tilt_thread(void *arg) {
	const uint16_t period = *(uint16_t *) arg;
	systime_t time = chTimeNow();
	float tilt = 0;

	while (TRUE) {
		tilt = complemetary_filter_update(period);

		chprintf((BaseSequentialStream*) &SERIAL_DRIVER, "%6d %f\r\n",
				(int) time, tilt);
		time += MS2ST(period);
		chThdSleepUntil(time);
	}

	return 0;
}

float complemetary_filter_update(uint16_t period) {
	const float dt = period / 1000.0;
	float acc_angle = 0;
	float gyro_rate = 0;
	static float angle;

	acc_angle = ((float)acc_data.y - ACC_ZERO) / ACC2GRAD;
	gyro_rate = ((float)gyro_data.x - GYRO_ZERO) * GYRO2GRAD;
	angle = (0.98) * (angle + (gyro_rate * dt) + (0.02 * acc_angle));

	return angle;
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

	chThdSleepMilliseconds(200);

	if (gyrotp == NULL)
		gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
	if (acctp == NULL)
		acctp = accRun(&I2C_DRIVER, NORMALPRIO);
	if (magtp == NULL)
		magtp = magRun(&I2C_DRIVER, NORMALPRIO);

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
