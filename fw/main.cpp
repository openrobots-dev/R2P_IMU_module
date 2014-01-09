#include "ch.h"
#include "hal.h"
#include "chprintf.h"

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

#include <r2p/node/led.hpp>

#include <r2p/msg/imu.hpp>
#include <r2p/msg/motor.hpp>

#include <l3g4200d.h>
#include <lsm303.h>
#include <mahony.h>
#include <madgwick.h>

#include <pid.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "R2PMODX"
#endif

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

static WORKING_AREA(wa_info, 1024);

// RTCAN transport
static r2p::RTCANTransport rtcantra(RTCAND1);

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

/*
 * Publisher node
 */
extern gyro_data_t gyro_data;
extern acc_data_t acc_data;
extern mag_data_t mag_data;

static const I2CConfig i2c1cfg = { OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2 };
static const SPIConfig spi1cfg = { NULL, /* HW dependent part.*/GYRO_GPIO, GYRO_CS, SPI_CR1_BR_1 | SPI_CR1_CPOL
		| SPI_CR1_CPHA };
static const EXTConfig extcfg = { { { EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOB,
		l3g4200d_drdy_callback }, { EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, {
		EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOB, lsm303_mag_drdy_cb }, {
		EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOB, lsm303_acc_int1_cb }, { EXT_CH_MODE_DISABLED, NULL }, {
		EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, {
		EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL }, {
		EXT_CH_MODE_DISABLED, NULL }, { EXT_CH_MODE_DISABLED, NULL } } };

msg_t publisher_node(void *arg) {
	r2p::Node node("imu_pub");
	r2p::Publisher<r2p::tIMURaw9> imu_pub;
	r2p::tIMURaw9 *msgp;
	systime_t time;

	(void) arg;
	chRegSetThreadName("imu_pub");

	i2cStart(&I2C_DRIVER, &i2c1cfg);
	spiStart(&SPI_DRIVER, &spi1cfg);
	extStart(&EXTD1, &extcfg);

	gyroRun(&SPI_DRIVER, NORMALPRIO);
	accRun(&I2C_DRIVER, NORMALPRIO);
	magRun(&I2C_DRIVER, NORMALPRIO);

	node.advertise(imu_pub, "imuraw");

	time = chTimeNow();

	for (;;) {
		if (imu_pub.alloc(msgp)) {
			msgp->gyro_x = gyro_data.x;
			msgp->gyro_y = gyro_data.y;
			msgp->gyro_z = gyro_data.z;
			msgp->acc_x = acc_data.x;
			msgp->acc_y = acc_data.y;
			msgp->acc_z = acc_data.z;
			msgp->mag_x = mag_data.x;
			msgp->mag_y = mag_data.y;
			msgp->mag_z = mag_data.z;

			imu_pub.publish(*msgp);
		}

		time += MS2ST(10);
		chThdSleepUntil(time);
	}
	return CH_SUCCESS;
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO + 1, r2p::ledsub_node, NULL);
	uint8_t led = 1;
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(256), NORMALPRIO + 1, r2p::ledpub_node, (void *) &led);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 2, publisher_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}
	return CH_SUCCESS;
}
}
