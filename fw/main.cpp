#include "ch.h"
#include "hal.h"

#include "rtcan.h"

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Topic.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>
#include <r2p/Mutex.hpp>
#include <r2p/NamingTraits.hpp>
#include <r2p/Bootloader.hpp>
#include "r2p/transport/DebugTransport.hpp"
#include "r2p/transport/RTCANTransport.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "l3g4200d.h"
#include "lsm303.h"
#include "madgwick.h"
#include "mahony.h"

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "R2PMODX"
#endif

#define ACC_ZERO -40
#define ACC2GRAD 20.0

#define GYRO_ZERO -80
#define GYRO2GRAD -0.0175

Thread *gyrotp = NULL;
Thread *acctp = NULL;
Thread *magtp = NULL;

struct TiltMsg: public r2p::Message {
	float angle;
	float rate;
}R2P_PACKED;

extern "C" {
void *__dso_handle;
void __cxa_pure_virtual() {
	chSysHalt();
}
void _exit(int) {
	chSysHalt();
	for (;;) {
	}
}
int _kill(int, int) {
	chSysHalt();
	return -1;
}
int _getpid() {
	return 1;
}
} // extern "C"

static WORKING_AREA(wa_info, 2048);
static WORKING_AREA(wa_rx_dbgtra, 1024);
static WORKING_AREA(wa_tx_dbgtra, 1024);

static char dbgtra_namebuf[64];
static r2p::DebugTransport dbgtra(reinterpret_cast<BaseChannel *>(&SD2), dbgtra_namebuf);

// RTCAN transport
static r2p::RTCANTransport rtcantra(RTCAND1);

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

RTCANConfig rtcan_config = { 1000000, 100, 60 };


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

/*
 * Simple complementary filter.
 */
extern acc_data_t acc_data;
extern gyro_data_t gyro_data;

float complemetary_filter_update(uint16_t period) {
	const float dt = period / 1000.0;
	float acc_angle = 0;
	float gyro_rate = 0;
	static float angle;

	acc_angle = (acc_data.x - ACC_ZERO) / ACC2GRAD;
	gyro_rate = (gyro_data.z - GYRO_ZERO) * GYRO2GRAD;
	angle = (0.98) * (angle + (gyro_rate * dt) + (0.02 * acc_angle));

	return angle;
}

/*
 * Tilt publisher thread
 */
static msg_t tilt_node(void *arg) {
	r2p::Node node("tilt");
	r2p::Publisher<TiltMsg> tilt_pub;
	systime_t time = chTimeNow();
	const uint16_t period = 500;

	node.advertise(tilt_pub, "tilt");

	chThdSleepMilliseconds(100);

	for (;;) {
		TiltMsg *msgp;
		if (tilt_pub.alloc(msgp)) {
			msgp->angle = complemetary_filter_update(period);
			msgp->rate = (gyro_data.z - GYRO_ZERO) * GYRO2GRAD;
			if (!tilt_pub.publish(*msgp)) {
				chSysHalt();
			}
		}

		time += MS2ST(period);
		chThdSleepUntil(time);
	}
	return CH_SUCCESS;
}

/*
 * Balance node.
 */
msg_t balance_node(void *) {
	TiltMsg sub_msgbuf[5], *sub_queue[5];
	r2p::Node node("balance");
	r2p::Subscriber<TiltMsg> sub(sub_queue, 5);
	TiltMsg * msgp;

	node.subscribe(sub, "tilt", sub_msgbuf);

	for (;;) {
		node.spin();
		sub.fetch(msgp);
		palTogglePad(LED_GPIO, LED2);
		sub.release(*msgp);
	}
	return CH_SUCCESS;
}

void rtcan_blinker(void) {
	switch (RTCAND1.state) {
	case RTCAN_MASTER:
		palClearPad(LED_GPIO, LED1);
		chThdSleepMilliseconds(200);
		palSetPad(LED_GPIO, LED1);
		chThdSleepMilliseconds(100);
		palClearPad(LED_GPIO, LED1);
		chThdSleepMilliseconds(200);
		palSetPad(LED_GPIO, LED1);
		chThdSleepMilliseconds(500);
		break;
	case RTCAN_SYNCING:
		palTogglePad(LED_GPIO, LED1);
		chThdSleepMilliseconds(100);
		break;
	case RTCAN_SLAVE:
		palTogglePad(LED_GPIO, LED1);
		chThdSleepMilliseconds(500);
		break;
	case RTCAN_ERROR:
		palTogglePad(LED_GPIO, LED4);
		chThdSleepMilliseconds(200);
		break;
	default:
		chThdSleepMilliseconds(100);
		break;
	}
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	sdStart(&SD2, NULL);
	extStart(&EXTD1, &extcfg);
	i2cStart(&I2C_DRIVER, &i2c1cfg);
	spiStart(&SPI_DRIVER, &spi1cfg);

	r2p::Thread::set_priority(r2p::Thread::HIGHEST);
	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);

	rtcantra.initialize(rtcan_config);

	dbgtra.initialize(wa_rx_dbgtra, sizeof(wa_rx_dbgtra), r2p::Thread::LOWEST + 11, wa_tx_dbgtra, sizeof(wa_tx_dbgtra),
			r2p::Thread::LOWEST + 10);

	r2p::Thread::set_priority(r2p::Thread::NORMAL);
/*
	if (gyrotp == NULL)
		gyrotp = gyroRun(&SPI_DRIVER, NORMALPRIO);
	if (acctp == NULL)
		acctp = accRun(&I2C_DRIVER, NORMALPRIO);
*/

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 1, tilt_node, NULL);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 1, balance_node, NULL);

	for (;;) {
		rtcan_blinker();
	}
	return CH_SUCCESS;
}
}
