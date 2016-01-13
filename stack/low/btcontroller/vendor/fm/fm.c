/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

/********************************* Logger *************************/
enum { __FILE_NUM__= 140 };
/********************************* Logger *************************/

#include "fm_imp.h"
#include "logger.h"

#ifdef FM_POLL_TIMEOUT
#define FM_NUM_POLL_MAX 20000

#define FM_POLL_TIMEOUT_LOG() RT_BT_LOG(RED, LOG_FM_POLL_TIMEOUT, 0, 0)

#define FM_WAIT_FOR_READY_RET_FALSE() \
    do {\
        int n = 1;\
        while (!fm_is_ready())\
        {\
            if (n > FM_NUM_POLL_MAX)\
            {\
                FM_POLL_TIMEOUT_LOG();\
                return FALSE;\
            }\
            ++n;\
        }\
    } while (0)
#else
#define FM_WAIT_FOR_READY_RET_FALSE() do {} while (!fm_is_ready())
#endif

FM_CONTROL_BLOCK fm_ctrl;

void fm_init_imp(void)
{
    BT2FM_FM_EN_REG reg = fm_bt2fm_fm_en_reg_read();
    fm_ctrl.enable = reg.fm_enable;
    fm_ctrl.i2s_enable = reg.fm_i2s_enable;
}
void (*fm_init)(void) = fm_init_imp;

BOOLEAN fm_read_register(UINT16 addr, UINT8 *data)
{
    BT2FM_ACCESS_REG reg = {0};
    FM_WAIT_FOR_READY_RET_FALSE();
    reg.fm_access_start = 1;
    reg.rw_mode = BT2FM_ACCESS_REG_READ_MODE;
    reg.fm_reg_addr = addr;
    fm_bt2fm_access_reg_write(reg);
    FM_WAIT_FOR_READY_RET_FALSE();
    reg = fm_bt2fm_access_reg_read();
    *data = reg.fm_rw_data;
    FM_LOG(WHITE, LOG_LEVEL_HIGH, LOG_FM_READ_REG, 2, addr, reg.fm_rw_data);
    return TRUE;
}

BOOLEAN fm_write_register(UINT16 addr, UINT8 data)
{
    BT2FM_ACCESS_REG reg;
    FM_WAIT_FOR_READY_RET_FALSE();
    reg.fm_access_start = 1;
    reg.rw_mode = BT2FM_ACCESS_REG_WRITE_MODE;
    reg.fm_reg_addr = addr;
    reg.fm_rw_data = data;
    fm_bt2fm_access_reg_write(reg);
    FM_WAIT_FOR_READY_RET_FALSE();
    FM_LOG(WHITE, LOG_LEVEL_HIGH, LOG_FM_WRITE_REG, 2, addr, data);
    return TRUE;
}

void fm_switch(UINT8 state)
{
    UINT8 enable = !!state;
    BT2FM_FM_EN_REG reg = fm_bt2fm_fm_en_reg_read();
    reg.fm_enable = enable;
    fm_bt2fm_fm_en_reg_write(reg);
    fm_ctrl.enable = enable;
}

void fm_i2s_switch(UINT8 state)
{
    UINT8 enable = !!state;
    BT2FM_FM_EN_REG reg = fm_bt2fm_fm_en_reg_read();
    reg.fm_i2s_enable = enable;
    fm_bt2fm_fm_en_reg_write(reg);
    fm_ctrl.i2s_enable = enable;
}

#ifdef FM_DEBUG
UINT8 fm_get_ready()
{
    return fm_is_ready();
}

UINT32 fm_get_access_reg_raw()
{
    return RD_32BIT_IO(BT2FM_ACCESS_REG_MMIO, 0);
}
#endif
