/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
//S3C2410_bt_power_ctrl.h

#ifdef POWER_SAVE_FEATURE

#include "platform.h"

/*=====================	Function declaration	=======================*/
//void S3C2410_init_power_control(void);
void S3C2410_set_dsm_exit(void);
void S3C2410_clear_dsm_exit(void);
//void S3C2410_clear_bb_clk_en(void);
//void S3C2410_set_bb_clk_en(void);
//void S3C2410_clear_rf_clk_en(void);
//void S3C2410_set_rf_clk_en(void);
/*=====================End function declaration	=======================*/

/* Power Control Register Address*/
#ifndef _YL_LPS
#define	POWER_CTRL			(BB_BASE_ADDR + 0x98)

/* Bit position in the Power Control register. Implementation specific */
#define	BB_CLK_EN			1
#define	RF_CLK_EN			2
#define	DSM_EXIT			4
#endif
#endif //#ifdef POWER_SAVE_FEATURE


