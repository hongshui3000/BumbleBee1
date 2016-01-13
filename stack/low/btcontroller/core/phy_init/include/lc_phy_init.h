/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _LC_PHY_INIT_H_
#define _LC_PHY_INIT_H_

void read_and_parse_ft_efuse_records(void);
void validate_ft_efuse_records(void);
void execute_iqk_lok(void);
UINT16 btrf_iqk_lok(UCHAR index);
void set_lok_txiqk_manual(UINT16 lok_result, UINT16 txiqk_result_x,UINT16 txiqk_result_y);
void read_lok_txiqk_auto(UINT16 *lok_result, UINT16 *iqkx_reault, UINT16 *iqky_result);

#define FT_EFUSE_RECORDS_BANK EFUSE_BANK_1
#define FT_EFUSE_RECORDS_START 0x76
#define FT_EFUSE_RECORDS_LEN 7
#define LOK_DISTANCE_SQUARE_TH 16

typedef union FT_EFUSE_RECORDS_S_ {
    UINT8 d8[FT_EFUSE_RECORDS_LEN];
    struct
    {
        UINT8 modem_iqky_lsb;
        UINT8 modem_iqky_msb;
        UINT8 modem_iqkx_lsb;
        UINT8 modem_iqkx_msb;
        UINT8 rfc_lok_lsb;
        UINT8 rfc_lok_msb;
        UINT8 tmeter_val;
    };
} FT_EFUSE_RECORDS_S;


typedef struct EFUSE_RF_SETTING_S_ { /* EFUSE[0x1EA~0x1EB] */
    UINT16 ignore_ft_efuse_records:1;        // [0]
    UINT16 use_ft_efuse_lok_iqk:1;           // [1]
    UINT16 rsvd:14;                          // [15:2]
} EFUSE_RF_SETTING_S;

typedef struct FT_EFUSE_MANAGE_S_ {
    FT_EFUSE_RECORDS_S ft_efuse_records;
    UINT8 ft_efuse_tmeter_valid;
    UINT8 ft_efuse_lok_valid;
    UINT8 ft_efuse_txiqk_valid;
} FT_EFUSE_MANAGE_S;

#endif
