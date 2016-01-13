/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

/**
 * \file
 *  Contains LC module initialization and shutdown routines. It also has
 *  utility functions and low power mode functions.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 218 };
/********************************* Logger *************************/

/* ========================= Include File Section ==================== */
#include "lc_internal.h"
#include "lc_phy_init.h"
#include "DataType.h"
#include "common_utils.h"
#include "efuse.h"

FT_EFUSE_MANAGE_S ft_efuse_manage;
UCHAR iqk_count_index;

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC  rcp_btrf_iqk_lok = NULL;
#endif
void read_and_parse_ft_efuse_records(void)
{
	UINT8 efuse_data;
    UINT16 efuse_read_index = FT_EFUSE_RECORDS_START;
    UINT32 ii = 0;

    for (ii = 0; ii < FT_EFUSE_RECORDS_LEN; ii ++)
    {
        /* select efuse bank 1 */
        EFUSE_BYTE_WRITE((EFUSE_CTL2_REG + 3), FT_EFUSE_RECORDS_BANK);
        /* parser 1st byte of header */
        efuse_one_byte_read(efuse_read_index, &efuse_data);
        ft_efuse_manage.ft_efuse_records.d8[ii] = efuse_data;
        efuse_read_index ++;
    }
}
void validate_ft_efuse_records(void)
{
    EFUSE_RF_SETTING_S efuse_rf_setting;
    *(UINT16*)&efuse_rf_setting = otp_str_data.efuse_ft_info;
    if (efuse_rf_setting.ignore_ft_efuse_records == 0)
    {
        if ((ft_efuse_manage.ft_efuse_records.tmeter_val)  &&
            (ft_efuse_manage.ft_efuse_records.tmeter_val <= 0x3f) &&
            (ft_efuse_manage.ft_efuse_records.rfc_lok_msb & BIT7)  &&
            ((ft_efuse_manage.ft_efuse_records.modem_iqkx_msb & 0xFC) == 0) &&
            ((ft_efuse_manage.ft_efuse_records.modem_iqky_msb & 0xFC) == 0))
        {
            ft_efuse_manage.ft_efuse_tmeter_valid = 1;
			ft_efuse_manage.ft_efuse_lok_valid = 1;
			ft_efuse_manage.ft_efuse_txiqk_valid = 1;
        }
        else
        {
            ft_efuse_manage.ft_efuse_tmeter_valid = 0;
            ft_efuse_manage.ft_efuse_txiqk_valid = 0;
            ft_efuse_manage.ft_efuse_lok_valid = 0;
        }
    }
    if (ft_efuse_manage.ft_efuse_tmeter_valid)
    {
        otp_str_data.EFuse_ThermalDefault = ft_efuse_manage.ft_efuse_records.tmeter_val;
    }
}
void set_lok_txiqk_manual(UINT16 lok_result, UINT16 txiqk_result_x,UINT16 txiqk_result_y)
{
    RTK_WRITE_RF_REG(0x21, lok_result);
#ifdef _NEW_MODEM_PI_ACCESS_
    RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x06), txiqk_result_x);
    RTK_WRITE_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x08), txiqk_result_y);
    RTK_UPDATE_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x02), BIT5, BIT5);    // Set 0x02[5] = 1 for iqk_en
    RTK_UPDATE_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x0C), BIT4, BIT4);    // Set 0x0C[4] = 1 to enable reg_iqk_tx_comp //
    RTK_UPDATE_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x0E), BIT15, 0x0);    // Set 0x0E[15] = 0 for txiqk manual mode //
#else
	RT_BT_LOG(RED, REG_PI_ACCESS_ERROR, 0,0);
#endif
}
void read_lok_txiqk_auto(UINT16 *lok_result, UINT16 *iqkx_reault, UINT16 *iqky_result)
{
    *lok_result = RTK_READ_RF_REG(0x21);            /* NOTE: 若使用SI來讀取，則要避免與Bluewiz衝突*/

#ifdef _NEW_MODEM_PI_ACCESS_
    RTK_UPDATE_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x2),0x1f,0x0);        //Write ModemPI page1.0x02[4:0] = 0;
    *iqkx_reault = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x7c));    /* IQKx format: {6`b0, s(10,8f)} */

    RTK_UPDATE_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x2),0x1f,0x1);        //Write ModemPI page1.0x02[4:0] = 0;
    *iqky_result = RTK_READ_MODEM_REG_PI(MODEM_PI_PAGE_1,TRANS_MODEM_REG(0x7c));    /* IQKx format: {6`b0, s(10,8f)} */
#else
	RT_BT_LOG(RED, REG_PI_ACCESS_ERROR, 0,0);
#endif
}

extern UINT16 cal_d_sq(INT16 i1, INT16 q1, INT16 i2, INT16 q2);
void execute_iqk_lok(void)
{
    UINT16 rf0x21;
    UINT16 lok_i[3];
    UINT16 lok_q[3];
    UINT16 lok_i_result;
    UINT16 lok_q_result;
    UINT16 dummy_uint16;    /* since RTL8703B */
    UINT16 iqkx_result;     /* since RTL8703B */
    UINT16 iqky_result;     /* since RTL8703B */

    UINT16 d01_sq;
    UINT16 d02_sq;
    UINT16 d12_sq;

    UINT8 lok_success = 1;  /* initialize lok_success = 1 */

    /* initialize to large/differentiable values */
    /* if this time LOK/IQK fails，not update */
    UINT8 ii;
    for (ii = 0; ii<3; ii++)
    {
       lok_i[ii] = ((ii+2)<<5);
       lok_q[ii] = ((ii+2)<<5);
    }

    /* DUMMY LOK. somehow 8723b's IQK/LOK result of first time is always not good enough*/
    rf0x21 = btrf_iqk_lok(1);

    /* 1st LOK/IQK */
#ifdef _IQK_HANDSHAKE_FLOW_
    iqk_count_index = 0;
#endif
    rf0x21 = btrf_iqk_lok(0);

    if (rf0x21!=0xFFFF)
    {
        /* if patch_rtl8821_btrf_lokiqk successes, update the lok_i and lok_q */
        lok_i[0] = (rf0x21 >> 10 ) & 0x1F;
        lok_q[0] = (rf0x21 >> 5 ) & 0x1F;
    }

    /* 2nd LOK/IQK */
#ifdef _IQK_HANDSHAKE_FLOW_
    iqk_count_index = 1;
#endif
    rf0x21 = btrf_iqk_lok(0);


    if (rf0x21!=0xFFFF)
    {
        lok_i[1] = (rf0x21>>10)&0x1F;
        lok_q[1] = (rf0x21>>5)&0x1F;
    }

    /* d01_sq = (i0-i1)^2 + (q0-q1)^2 */
    d01_sq = cal_d_sq(lok_i[0], lok_q[0], lok_i[1], lok_q[1]);
    if (d01_sq <= LOK_DISTANCE_SQUARE_TH)
    {
        /* if close enough , get average and enter the ending flow */
        lok_i_result = (lok_i[0] + lok_i[1])>>1;
        lok_q_result = (lok_q[0] + lok_q[1])>>1;
    }
    else    /* if not close enough , do the 3rd times*/
    {
        /* 3rd LOK/IQK */
#ifdef _IQK_HANDSHAKE_FLOW_
        iqk_count_index = 2;
#endif
        rf0x21 = btrf_iqk_lok(0);

        if (rf0x21!=0xFFFF)
        {
            lok_i[2] = (rf0x21>>10)&0x1F;
            lok_q[2] = (rf0x21>>5)&0x1F;
        }

    /* d02_sq = (i0-i2)^2 + (q0-q2)^2 */
    /* d12_sq = (i1-i2)^2 + (q1-q2)^2 */
        d02_sq = cal_d_sq(lok_i[0], lok_q[0], lok_i[2], lok_q[2]);
        d12_sq = cal_d_sq(lok_i[1], lok_q[1], lok_i[2], lok_q[2]);
        if ( d02_sq<= LOK_DISTANCE_SQUARE_TH)
        {
            if (d12_sq<d02_sq)
            {
                /* if 1 and 2 are more close , get the average of 1 and 2 , and enter ending flow */
                lok_i_result = (lok_i[1] + lok_i[2])>>1;
                lok_q_result = (lok_q[1] + lok_q[2])>>1;
            }
            else
            {
                /* if 0 and 2 are more close , get the average of 0 and 2 , and enter ending flow */
                lok_i_result = (lok_i[0] + lok_i[2])>>1;
                lok_q_result = (lok_q[0] + lok_q[2])>>1;
            }
        }
        else if ( d12_sq <=LOK_DISTANCE_SQUARE_TH)
        {
            /* if 1 and 2 are more close , get the average of 1 and 2 , and enter ending flow*/
            lok_i_result = (lok_i[1] + lok_i[2])>>1;
            lok_q_result = (lok_q[1] + lok_q[2])>>1;

        }
        else
        {
            /* if no good result , set lok_success = 0 , set lok_i/q as special pattern for further debug */
            lok_i_result = 0x10;
            lok_q_result = 0x11;     /* to signature LOK-FAIL */
            lok_success = 0;
        }
    }
    /* Ending flow */
    rf0x21 = RTK_READ_RF_REG(0x21);
    rf0x21 &= 0x1F;    /* bit0~bit4 keep the same , only assign bit5~bit15*/
	
    /* flow modified since 03b , 22b and bee */
    rf0x21 |= ((lok_i_result)<<10);
    rf0x21 |= ((lok_q_result)<<5);
    RTK_WRITE_RF_REG((0x021), rf0x21);

    if (lok_success)
    {
        rf0x21 |= BIT15;     /* LOK Compensation IDAC enable */
        /* added since RTL8703B. read_lok_txiqk_auto() refer to "section 12" */
        read_lok_txiqk_auto(&dummy_uint16, &iqkx_result, &iqky_result);
    }
    else
    {
        /* LOK/IQK flow fails，set TXIQK compensation OFF */
        iqkx_result = 0x0100;       /* added since RTL8703B , assign x = 1.0 */
        iqky_result = 0x0000;       /* added since RTL8703B , assign y = 0.0 */
    }
    set_lok_txiqk_manual(rf0x21,iqkx_result,iqky_result);
    RT_BT_LOG(RED,IQK_LOK_RESULT,3,rf0x21,iqkx_result,iqky_result);
}
UINT16 btrf_iqk_lok(UCHAR index)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_btrf_iqk_lok != NULL)
    {
    	return rcp_btrf_iqk_lok((void *)& index);
    }
#endif
	return 0x2100;
}
