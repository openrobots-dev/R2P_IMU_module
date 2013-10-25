#include "ch.h"
#include "hal.h"

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Topic.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>
#include <r2p/Mutex.hpp>
#include <r2p/NamingTraits.hpp>
#include <r2p/Bootloader.hpp>
#include <r2p/transport/RTCANTransport.hpp>
#include <r2p/transport/DebugTransport.hpp>

#include <r2p/node/led.hpp>
#include <r2p/msg/motor.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "IMU_2"
#endif

#define BOOT_STACKLEN   1024

#define DEBUGTRA 0
#define RTCANTRA 1

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

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

// Debug transport
#if DEBUGTRA
static WORKING_AREA(wa_rx_dbgtra, 1024);
static WORKING_AREA(wa_tx_dbgtra, 1024);
static char dbgtra_namebuf[64];
static r2p::DebugTransport dbgtra("dbg", reinterpret_cast<BaseChannel *>(&SD2), dbgtra_namebuf);
#endif

// RTCAN transport
#if RTCANTRA
static r2p::RTCANTransport rtcantra(RTCAND1);
RTCANConfig rtcan_config = { 1000000, 100, 60 };
#endif

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Thread::set_priority(r2p::Thread::HIGHEST);

    void *boot_stackp = NULL;
    if (r2p::Middleware::is_bootloader_mode()) {
      uint8_t *stackp = new uint8_t[BOOT_STACKLEN + sizeof(stkalign_t)];
      R2P_ASSERT(stackp != NULL);
      stackp += (sizeof(stkalign_t) - reinterpret_cast<uintptr_t>(stackp)) %
                sizeof(stkalign_t);
      boot_stackp = stackp;
    }

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST,
	                                     boot_stackp, BOOT_STACKLEN, r2p::Thread::LOWEST);

#if DEBUGTRA
    sdStart(&SD2, NULL);
    dbgtra.initialize(wa_rx_dbgtra, sizeof(wa_rx_dbgtra), r2p::Thread::LOWEST + 11, wa_tx_dbgtra, sizeof(wa_tx_dbgtra),
            r2p::Thread::LOWEST + 10);
#endif

#if RTCANTRA
	rtcantra.initialize(rtcan_config);
#endif

	r2p::Middleware::instance.start();

	unsigned int led = 1;

//	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledpub_node, (void *)&led, "ledpub");
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, NULL, "ledsub");

	r2p::Thread::set_priority(r2p::Thread::NORMAL);
	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}
	return CH_SUCCESS;
}
}
