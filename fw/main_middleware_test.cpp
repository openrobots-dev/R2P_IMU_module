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

#include "ch.h"
#include "hal.h"

#include "l3g4200d.h"
#include "lsm303.h"
#include "madgwick.h"

#include "rtcan.h"
#include "Middleware.hpp"
#include "r2p_imu.h"

#define ACC_ZERO -40
#define ACC2GRAD 20.0

#define GYRO_ZERO -80
#define GYRO2GRAD -0.0175

#define WA_SIZE_256B      THD_WA_SIZE(256)
#define WA_SIZE_512B      THD_WA_SIZE(512)
#define WA_SIZE_1K        THD_WA_SIZE(1024)
#define WA_SIZE_2K        THD_WA_SIZE(2048)

static msg_t stream_raw_thread(void *arg);
static msg_t stream_madgwick_thread(void *arg);
static msg_t stream_tilt_thread(void *arg);

Thread *gyrotp = NULL;
Thread *acctp = NULL;
Thread *magtp = NULL;
uint16_t period;

RTCANConfig rtcan_config = {1000000, 100, 60};

/*===========================================================================*/
/* STM32 id & reset.                                                         */
/*===========================================================================*/

static uint8_t stm32_id8(void) {
	const unsigned long * uid = (const unsigned long *)0x1FFFF7E8;

	return (uid[2] & 0xFF);
}

static void stm32_reset(void) {

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
/* I2C related.                                                              */
/*===========================================================================*/

/* I2C1 configuration. */
static const I2CConfig i2c1cfg = { OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2 };

/*===========================================================================*/
/* SPI related.                                                              */
/*===========================================================================*/

/* SPI1 configuration.
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

extern gyro_data_t gyro_data;
extern acc_data_t acc_data;
extern mag_data_t mag_data;

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
 * Publisher threads.
 */
static msg_t PublisherRawThread(void *arg) {
	Middleware & mw = Middleware::instance();
	Node n("pubRaw");
	Publisher<tIMURaw9> pub("IMURaw");
	tIMURaw9 *msg;
	systime_t time;
	static const int period = 10;

	(void) arg;
	chRegSetThreadName("tIMURaw9 pub thread");

	mw.newNode(&n);

	if (! n.advertise(&pub)) {
		mw.delNode(&n);
		return 0;
	}

	time = chTimeNow();
	while (TRUE) {
		msg = pub.alloc();
		if (msg != NULL) {
			rtcanGetTime(&RTCAND1, (rtcan_time_t *)&(msg->timestamp));
			msg->gyro_x = gyro_data.x;
			msg->gyro_y = gyro_data.y;
			msg->gyro_z = gyro_data.z;
			msg->acc_x = acc_data.x;
			msg->acc_y = acc_data.y;
			msg->acc_z = acc_data.z;
			msg->mag_x = mag_data.x;
			msg->mag_y = mag_data.y;
			msg->mag_z = mag_data.z;
			pub.broadcast(msg);
		}

		time += MS2ST(period);
		chThdSleepUntil(time);
	}

	return 0;
}

// FIXME
RemoteSubscriberT<tIMURaw9, 5> rsub("IMURaw");

void remote_sub(const char * topic) {
	Middleware & mw = Middleware::instance();
	LocalPublisher * pub;

	pub = mw.findLocalPublisher(topic);

	if (pub) {
		rsub.id(IMURAW9_ID);
		rsub.subscribe(pub);
	}
}

/*
 * Application entry point.
 */
int main(void) {

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
	 * Activates the RTCAN driver.
	 */
	rtcanInit();
	rtcanStart(&RTCAND1, &rtcan_config);

	/*
	 * Creates the blinker thread.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	chThdSleepMilliseconds(100);

	/*
	 * Creates the sensor threads.
	 */
	gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
	acctp = accRun(&I2C_DRIVER, NORMALPRIO);
	magtp = magRun(&I2C_DRIVER, NORMALPRIO);

	/*
	 * Creates the publisher threads.
	 */
	chThdCreateFromHeap(NULL, WA_SIZE_2K, NORMALPRIO + 1, PublisherRawThread,
			NULL);

	chThdSleepMilliseconds(100);

	remote_sub("IMURaw");

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (TRUE) {
		chThdSleepMilliseconds(200);
	}
}
