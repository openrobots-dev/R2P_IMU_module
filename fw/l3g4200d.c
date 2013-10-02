/**
 * @file    l3g4200d.c
 * @brief   L3G4200D MEMS interface module through SPI code.
 *
 * @addtogroup l3g4200d
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "l3g4200d.h"

#define GYRO_WA_SIZE    THD_WA_SIZE(256)

static uint8_t txbuf[2];
static uint8_t rxbuf[2];

static Thread *gyrotp = NULL;

systime_t gyro_update_time;
gyro_data_t gyro_data =
  {0, 0, 0, 0};

/**
 * @brief   Reads a register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @return              The register value.
 */
int init_l3g4200d(SPIDriver *spip) {

  l3g4200dReadRegister(spip, L3G4200D_WHO_AM_I);
  l3g4200dWriteRegister(spip, L3G4200D_CTRL_REG1, 0x7F);
  l3g4200dWriteRegister(spip, L3G4200D_CTRL_REG2, 0x00);
  l3g4200dWriteRegister(spip, L3G4200D_CTRL_REG3, 0x08);
  l3g4200dWriteRegister(spip, L3G4200D_CTRL_REG4, 0x10);
  l3g4200dWriteRegister(spip, L3G4200D_CTRL_REG5, 0x00);

  chThdSleepMilliseconds(250);
  return 0;
}

/**
 * @brief   Reads a register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @return              The register value.
 */
uint8_t l3g4200dReadRegister(SPIDriver *spip, uint8_t reg) {

  spiSelect(spip);
  txbuf[0] = 0x80 | reg;
  txbuf[1] = 0xff;
  spiExchange(spip, 2, txbuf, rxbuf);
  spiUnselect(spip);
  return rxbuf[1];
}

/**
 * @brief   Writes a value into a register.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @param[in] value     the value to be written
 */
void l3g4200dWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value) {

  switch (reg) {
  case L3G4200D_WHO_AM_I:
  case L3G4200D_OUT_TEMP:
  case L3G4200D_STATUS_REG:
  case L3G4200D_OUT_X_L:
  case L3G4200D_OUT_X_H:
  case L3G4200D_OUT_Y_L:
  case L3G4200D_OUT_Y_H:
  case L3G4200D_OUT_Z_L:
  case L3G4200D_OUT_Z_H:
    /* Read only registers cannot be written, the command is ignored.*/
    return;
  case L3G4200D_CTRL_REG1:
  case L3G4200D_CTRL_REG2:
  case L3G4200D_CTRL_REG3:
  case L3G4200D_CTRL_REG4:
  case L3G4200D_CTRL_REG5:
  case L3G4200D_REFERENCE:
  case L3G4200D_FIFO_CTRL_REG:
  case L3G4200D_FIFO_SRC_REG:
  case L3G4200D_INT1_CFG:
  case L3G4200D_INT1_SRC:
  case L3G4200D_INT1_TSH_XH:
  case L3G4200D_INT1_TSH_XL:
  case L3G4200D_INT1_TSH_YH:
  case L3G4200D_INT1_TSH_YL:
  case L3G4200D_INT1_TSH_ZH:
  case L3G4200D_INT1_TSH_ZL:
  case L3G4200D_INT1_DURATION:
    spiSelect(spip);
    txbuf[0] = reg;
    txbuf[1] = value;
    spiSend(spip, 2, txbuf);
    spiUnselect(spip);
    break;
  default:
    /* Reserved register must not be written, according to the datasheet
     this could permanently damage the device.*/
    chDbgAssert(FALSE, "L3G4200DWriteRegister(), #1", "reserved register");
    break;
  }
}

int16_t l3g4200dGetAxis(SPIDriver *spip, uint8_t axis) {
  int16_t data = 0;

  switch (axis) {
  case L3G4200D_AXIS_X:
    data = (l3g4200dReadRegister(spip, L3G4200D_OUT_X_H) & 0xFF) << 8;
    data |= l3g4200dReadRegister(spip, L3G4200D_OUT_X_L) & 0xFF;
    break;
  case L3G4200D_AXIS_Y:
    data = (l3g4200dReadRegister(spip, L3G4200D_OUT_Y_H) & 0xFF) << 8;
    data |= l3g4200dReadRegister(spip, L3G4200D_OUT_Y_L) & 0xFF;
    break;
  case L3G4200D_AXIS_Z:
    data = (l3g4200dReadRegister(spip, L3G4200D_OUT_Z_H) & 0xFF) << 8;
    data |= l3g4200dReadRegister(spip, L3G4200D_OUT_Z_L) & 0xFF;
    break;
  }

  return data;
}

void l3g4200d_update(SPIDriver *spip) {
  int16_t data[3];

  gyro_data.t = chTimeNow();

  //XXX da fare lettura sequenziale!
  data[0] = (l3g4200dReadRegister(spip, L3G4200D_OUT_X_H) & 0xFF) << 8;
  data[0] |= l3g4200dReadRegister(spip, L3G4200D_OUT_X_L) & 0xFF;
  data[1] = (l3g4200dReadRegister(spip, L3G4200D_OUT_Y_H) & 0xFF) << 8;
  data[1] |= l3g4200dReadRegister(spip, L3G4200D_OUT_Y_L) & 0xFF;
  data[2] = (l3g4200dReadRegister(spip, L3G4200D_OUT_Z_H) & 0xFF) << 8;
  data[2] |= l3g4200dReadRegister(spip, L3G4200D_OUT_Z_L) & 0xFF;

  chSysLock();
  gyro_data.x = data[0];
  gyro_data.y = data[1];
  gyro_data.z = data[2];
  chSysUnlock();
}

void l3g4200d_drdy_callback(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  /* Wakes up the thread.*/
  chSysLockFromIsr();
  if (gyrotp != NULL) {
    gyrotp->p_u.rdymsg = (msg_t)GYRO_DATA_READY;
    chSchReadyI(gyrotp);
    gyrotp = NULL;
  }
  chSysUnlockFromIsr();
}

static msg_t l3g4200d_update_thread(void *p) {
  SPIDriver *spip = (SPIDriver *)p;

  while (TRUE) {
    msg_t msg;

    /* Waiting for the IRQ to happen.*/
    chSysLock();
    gyrotp = chThdSelf();
    chSchGoSleepS(THD_STATE_SUSPENDED);
    msg = chThdSelf()->p_u.rdymsg;
    chSysUnlock();

    /* If data ready, update all axis.*/
    if (msg == GYRO_DATA_READY) {
      l3g4200d_update(spip);
    }
  }

  return RDY_OK;
}

Thread *gyroRun(SPIDriver *spip, tprio_t prio) {
  Thread *tp;

  init_l3g4200d(spip);
  tp = chThdCreateFromHeap(NULL, GYRO_WA_SIZE, prio, l3g4200d_update_thread,
                           (void*)spip);
  extChannelEnable(&EXTD1, GYRO_INT2);
  l3g4200d_update(spip);

  return tp;
}

/** @} */
