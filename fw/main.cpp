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

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "R2PMODX"
#endif

struct LedMsg: public r2p::Message {
	uint32_t led;
	uint32_t value;
	uint32_t cnt;
} R2P_PACKED;

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
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Led subscriber node
 */
bool callback(const LedMsg &msg) {

	palWritePad((GPIO_TypeDef *)led2gpio(msg.led), led2pin(msg.led), msg.value);

	return true;
}

msg_t ledsub_node(void * arg) {
	LedMsg sub_msgbuf[5], *sub_queue[5];
	r2p::Node node("ledpub");
    r2p::Subscriber<LedMsg> sub(sub_queue, 5, callback);
	char * tnp = (char *) arg;

	chRegSetThreadName("ledsub");

	node.subscribe(sub, tnp, sub_msgbuf);

	for (;;) {
		node.spin(1000);
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

	r2p::Thread::set_priority(r2p::Thread::HIGHEST);
	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);

	rtcantra.initialize(rtcan_config);

	dbgtra.initialize(wa_rx_dbgtra, sizeof(wa_rx_dbgtra), r2p::Thread::LOWEST + 11, wa_tx_dbgtra, sizeof(wa_tx_dbgtra),
			r2p::Thread::LOWEST + 10);

	r2p::Thread::set_priority(r2p::Thread::NORMAL);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 1, ledsub_node, (void *)"leds");

	for (;;) {
		rtcan_blinker();
	}
	return CH_SUCCESS;
}
}
