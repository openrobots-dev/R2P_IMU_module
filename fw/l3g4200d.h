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

/**
 * @file    l3g4200d.h
 * @brief   L3G4200D MEMS interface module through SPI header.
 *
 * @addtogroup l3g4200d
 * @{
 */

#ifndef _L3G4200D_H_
#define _L3G4200D_H_

#include "ch.h"
#include "hal.h"

/**
 * @name    L3G4200D register names
 * @{
 */
#define L3G4200D_WHO_AM_I               0x0F
#define L3G4200D_CTRL_REG1              0x20
#define L3G4200D_CTRL_REG2              0x21
#define L3G4200D_CTRL_REG3              0x22
#define L3G4200D_CTRL_REG4              0x23
#define L3G4200D_CTRL_REG5              0x24
#define L3G4200D_REFERENCE		        0x25
#define L3G4200D_OUT_TEMP		        0x26
#define L3G4200D_STATUS_REG             0x27
#define L3G4200D_OUT_X_L				0x28
#define L3G4200D_OUT_X_H				0x29
#define L3G4200D_OUT_Y_L				0x2A
#define L3G4200D_OUT_Y_H				0x2B
#define L3G4200D_OUT_Z_L				0x2C
#define L3G4200D_OUT_Z_H				0x2D
#define L3G4200D_FIFO_CTRL_REG			0x2E
#define L3G4200D_FIFO_SRC_REG			0x2F
#define L3G4200D_INT1_CFG				0x30
#define L3G4200D_INT1_SRC				0x31
#define L3G4200D_INT1_TSH_XH			0x32
#define L3G4200D_INT1_TSH_XL			0x33
#define L3G4200D_INT1_TSH_YH			0x34
#define L3G4200D_INT1_TSH_YL			0x35
#define L3G4200D_INT1_TSH_ZH			0x36
#define L3G4200D_INT1_TSH_ZL			0x37
#define L3G4200D_INT1_DURATION			0x38

#define L3G4200D_AXIS_X					0
#define L3G4200D_AXIS_Y					1
#define L3G4200D_AXIS_Z					2

#define GYRO_DATA_READY                 123

/** @} */

typedef struct {
  uint32_t t;
  int16_t x;
  int16_t y;
  int16_t z;
} gyro_data_t;

#ifdef __cplusplus
extern "C" {
#endif
  int init_l3g4200d(SPIDriver *spip);
  uint8_t l3g4200dReadRegister(SPIDriver *spip, uint8_t reg);
  void l3g4200dWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value);
  int16_t l3g4200dGetAxis(SPIDriver *spip, uint8_t axis);
  void l3g4200d_update(SPIDriver *spip);
  void l3g4200d_drdy_callback(void);
  Thread *gyroRun(SPIDriver *spip, tprio_t prio);
#ifdef __cplusplus
}
#endif

#endif /* _L3G4200D_H_ */

/** @} */
