/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef FM_H_
#define FM_H_

#include "DataType.h"

/**
 * @defgroup BT2FM  BT2FM Access Module
 *
 * The host FM driver would issue HCI commands which are handled by BT firmware
 * to access FM.
 * @{
 */

/**
 * @struct FM_CONTROL_BLOCK
 * @brief BT2FM control block
 */
typedef struct FM_CONTROL_BLOCK_
{
    BOOLEAN enable; /**< Whether FM is enabled or not. */
    BOOLEAN i2s_enable; /**< Whether FM I2S is enabled or not. */
    BOOLEAN hw_enable; /**< Whether FM is enabled or not by hw enable pin. */	
} FM_CONTROL_BLOCK;

extern FM_CONTROL_BLOCK fm_ctrl;

/**
 * FM initialization.
 */
extern void (*fm_init)(void);

/**
 * Read FM register.
 *
 * This function blocks until FM is ready.
 * @param addr  FM register address.
 * @param data  Address to store data.
 * @return  \c TRUE if success; \c FALSE on timeout.
 */
BOOLEAN fm_read_register(UINT16 addr, UINT8 *data);

/**
 * Write FM register.
 *
 * This function blocks until FM is ready.
 * @param addr  FM register address.
 * @param data  Data to write.
 * @return  \c TRUE if success; \c FALSE on timeout.
 */
BOOLEAN fm_write_register(UINT16 addr, UINT8 data);

/**
 * Switch FM on/off.
 * @param state  non-zero value is on; zero is off.
 */
void fm_switch(UINT8 state);

/**
 * Switch FM I2S on/off.
 * @param state  non-zero value is on; zero is off.
 */
void fm_i2s_switch(UINT8 state);
/**@}*/

#ifdef FM_DEBUG
UINT8 fm_get_ready();
UINT32 fm_get_access_reg_raw();
#endif

#endif /* FM_H_ */
