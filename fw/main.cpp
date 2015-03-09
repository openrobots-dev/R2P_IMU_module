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

#include <r2p/node/pid.hpp>

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
 * Velocity control node
 */
PID vel_pid;
float vel_setpoint = 0;
float w_setpoint = 0;
float angle_setpoint = 0;
float left_speed = 0;
float right_speed = 0;

bool qei1_callback(const r2p::QEIMsg &msg) {
	right_speed = (msg.delta / (165500.0f)) * 20; // ->m/s @20hz

	return true;
}

bool qei2_callback(const r2p::QEIMsg &msg) {
	left_speed = -(msg.delta / (165500.0f)) * 20; // ->m/s @20hz

	return true;
}

msg_t velocity_node(void *arg) {
	r2p::Node node("velocity");
	r2p::Subscriber<r2p::Velocity3Msg, 5> vel_sub;
	r2p::Velocity3Msg *velp;
	r2p::Subscriber<r2p::QEIMsg, 5> qei1_sub(qei1_callback);
	r2p::Subscriber<r2p::QEIMsg, 5> qei2_sub(qei2_callback);
	r2p::Publisher<r2p::Velocity3Msg> odometry_pub;
	float v;
	float w;

	(void) arg;
	chRegSetThreadName("velocity");

	node.subscribe(qei1_sub, "qei1");
	node.subscribe(qei2_sub, "qei2");
	node.subscribe(vel_sub, "velocity");
	if (!node.advertise(odometry_pub, "odometry")) while(1);

	vel_pid.config(2.0, 2, -0.1, 0.05, -5.0, 5.0);
//	vel_pid.config(5.0, 1.0, 0.00, 0.05, -5.0, 5.0);
//	vel_pid.config(5.0, 1.0, 0.05, 0.05, -5.0, 5.0);
	vel_pid.set(0);

	for (;;) {
		if (!node.spin(r2p::Time::ms(500))) {
			angle_setpoint = 0;
			continue;
		}

		v = (left_speed + right_speed) / 2;
		w = (right_speed - left_speed) / 0.425;
		angle_setpoint = vel_pid.update(v); // 20hz

		r2p::Velocity3Msg *msgp;
		if (odometry_pub.alloc(msgp)) {
			msgp->x = v;
			msgp->y = 0;
			msgp->w = w;
			odometry_pub.publish(*msgp);
		}

		while (vel_sub.fetch(velp)) {
			palTogglePad(LED2_GPIO, LED2);
			vel_setpoint = velp->x;
			w_setpoint = velp->w;
			vel_sub.release(*velp);
		}
		vel_pid.set(vel_setpoint);

	}
	return CH_SUCCESS;
}

/*
 * Balance control node
 */
msg_t balance_node(void *arg) {
	r2p::Node node("balance");
	r2p::Subscriber<r2p::TiltMsg, 2> tilt_sub;
	r2p::TiltMsg *tiltp;

	r2p::Publisher<r2p::PWM2Msg> pwm2_pub;
	r2p::PWM2Msg *pwmp;

	int32_t pwm = 0;

	(void) arg;
	chRegSetThreadName("balance");

	node.advertise(pwm2_pub, "pwm2");
	node.subscribe(tilt_sub, "tilt");

	PID pid;
//	pid.config(1000, 1, 0.005, 0.02, -4000, 4000);
	pid.config(600, 0.35, 0, 0.02, -4000, 4000);
	pid.set(angle_setpoint);

	for (;;) {
		pid.set(angle_setpoint);

		while (!tilt_sub.fetch(tiltp)) {
			r2p::Thread::sleep(r2p::Time::ms(1));
		}

		pwm = pid.update(tiltp->angle); //rad2grad
		tilt_sub.release(*tiltp);

		if (pwm2_pub.alloc(pwmp)) {
			pwmp->value[0] = (pwm - w_setpoint * 100);
			pwmp->value[1] = -(pwm + w_setpoint * 100);
			pwm2_pub.publish(*pwmp);
			palTogglePad(LED3_GPIO, LED3);
			palSetPad(LED4_GPIO, LED4);
		} else {
			palClearPad(LED4_GPIO, LED4);
		}

	}
	return CH_SUCCESS;
}

/*
 * Madgwick node
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

msg_t madgwick_node(void *arg) {
	r2p::Node node("madgwick");
	r2p::Publisher<r2p::TiltMsg> tilt_pub;
	attitude_t attitude_data;
	systime_t time;

	(void) arg;
	chRegSetThreadName("madgwick");

	i2cStart(&I2C_DRIVER, &i2c1cfg);
	spiStart(&SPI_DRIVER, &spi1cfg);
	extStart(&EXTD1, &extcfg);

	gyroRun(&SPI_DRIVER, NORMALPRIO);
	accRun(&I2C_DRIVER, NORMALPRIO);
//	magRun(&I2C_DRIVER, NORMALPRIO);

	node.advertise(tilt_pub, "tilt");

	time = chTimeNow();

	for (;;) {
		MadgwickAHRSupdateIMU((gyro_data.x / 57.143) * 3.141592 / 180.0, (gyro_data.y / 57.143) * 3.141592 / 180.0,
				(gyro_data.z / 57.143) * 3.141592 / 180.0, acc_data.x / 1000.0, acc_data.y / 1000.0,
				acc_data.z / 980.0);
		getMadAttitude(&attitude_data);

		r2p::TiltMsg *msgp;
		if (tilt_pub.alloc(msgp)) {
			msgp->angle = (-attitude_data.roll * 57.29578) - 3.35; // basketbot offset
			tilt_pub.publish(*msgp);
		}

		time += MS2ST(20);
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

	r2p::ledsub_conf ledsub_conf = { "leds" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO + 1, r2p::ledsub_node, (void *)&ledsub_conf);
	r2p::ledpub_conf ledpub_conf = {"leds", 1};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO + 1, r2p::ledpub_node, (void *) &ledpub_conf);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 3, madgwick_node, NULL);
	r2p::Thread::sleep(r2p::Time::ms(5000));

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 3, balance_node, NULL);
	r2p::Thread::sleep(r2p::Time::ms(500));
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 2, velocity_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}
	return CH_SUCCESS;
}
}
