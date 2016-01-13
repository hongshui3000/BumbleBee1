/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef FM_IMP_H_
#define FM_IMP_H_

#include "fm.h"
#include "platform.h"

/**
 * @addtogroup BT2FM
 * @{
 */

/**
 * @struct BT2FM_ACCESS_REG_
 * @brief BT2FM access register.
 *
 * This register controls reading/writing data from/to FM internal registers.
 * Poll #ready to test if you can start a read/write operation.
 * Set #fm_access_start to 1 to take read/write action.
 */
typedef struct BT2FM_ACCESS_REG_ {
    /** [7:0] FM read/write data. */
    UINT32 fm_rw_data : 8;
    /** [23:8] FM register address field. */
    UINT32 fm_reg_addr : 16;
    /** [28:24] Dummy bits. Unused now. */
    UINT32 dummy : 5;
    /** [29] Ready bit.
     *
     * FW can start to issue a FM access only when FW polls this bit to be 1.
     */
    UINT32 ready : 1;
    /** [30] Read/write mode (0: read; 1: write) */
    UINT32 rw_mode : 1;
    /** [31] FM access trigger.
     *
     * This bit is set to 1 when issuing a read/write operation. Note that the
     * corresponding value, like #fm_reg_data, #fm_reg_addr, #rw_mode, muse be
     * written simultaneously.
     */
    UINT32 fm_access_start : 1;
} BT2FM_ACCESS_REG;

/**
 * @struct BT2FM_FM_EN_REG_
 * @brief BT2FM FM enable register
 */
typedef struct BT2FM_FM_EN_REG_ {
    /** [0] Enable FM. */
    UINT32 fm_enable : 1;
    /** [7:1] Reserved. */
    UINT32 reserved0 : 7;
    /** [8] PCM MUX selector.
     *
     * 0: PCM.
     * 1: FM_I2S.
     */
    UINT32 pcm_mux_sel : 1;
    /** [9] Enable FM I2S. */
    UINT32 fm_i2s_enable : 1;
    /** [31:10] Reserved. */
    UINT32 reserved1 : 22;
} BT2FM_FM_EN_REG;

#define BT2FM_ACCESS_REG_MMIO VENDOR_REG_ADDR(0x360)
#define BT2FM_FM_EN_REG_MMIO VENDOR_REG_ADDR(0x364)

#define BT2FM_ACCESS_REG_READ_MODE 0x0
#define BT2FM_ACCESS_REG_WRITE_MODE 0x1

static inline UINT8 fm_is_ready()
{
    return (RD_REG_MMIO(BT2FM_ACCESS_REG, BT2FM_ACCESS_REG_MMIO).ready == 1);
}

static inline BT2FM_ACCESS_REG fm_bt2fm_access_reg_read()
{
    return RD_REG_MMIO(BT2FM_ACCESS_REG, BT2FM_ACCESS_REG_MMIO);
}

static inline void fm_bt2fm_access_reg_write(BT2FM_ACCESS_REG reg)
{
    WR_REG_MMIO(BT2FM_ACCESS_REG, BT2FM_ACCESS_REG_MMIO, reg);
}

static inline BT2FM_FM_EN_REG fm_bt2fm_fm_en_reg_read()
{
    return RD_REG_MMIO(BT2FM_FM_EN_REG, BT2FM_FM_EN_REG_MMIO);
}

static inline void fm_bt2fm_fm_en_reg_write(BT2FM_FM_EN_REG reg)
{
    WR_REG_MMIO(BT2FM_FM_EN_REG, BT2FM_FM_EN_REG_MMIO, reg);
}

/**@}*/

#endif /* FM_IMP_H_ */
