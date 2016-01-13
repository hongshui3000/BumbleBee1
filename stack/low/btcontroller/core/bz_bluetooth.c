/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/
enum { __FILE_NUM__= 26 };
/********************************* Logger *************************/

/**
 * \file bz_bluetooth.c
 *  BlueWiz Bluetooth host controller stack external interface implementation.
 *
 * \author Santhosh kumar M
 * \date 2008-04-09
 */
/* ========================= Include File Section ========================= */
#include "bz_bluetooth.h"
#include "platform.h"
#include "bt_fw_os.h"

/* ========================= Module specific header Files Section ========= */
#include "hci_td.h"
#include "lc.h"
#include "lmp.h"
#include "hci.h"
#ifdef LE_MODE_EN
#include "le_ll.h"
#endif
#ifdef _3DD_FUNCTION_SUPPORT_
#include "bt_3dd.h"
#endif

/* ====================== Macro Declaration Section ======================= */

/* ====================== External fuction declaration Section ============ */
extern void bt_fw_stack_init(void);

/* ==================== Structure Declaration Section ===================== */

/* ===================== Variable Declaration Section ===================== */

/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */

void bz_bluetooth_init(void)
{
    bt_fw_stack_init();
    hci_td_init();    
    lc_init();  
    lmp_init();    
    
#ifdef LE_MODE_EN
    //--- ll module init---
    if (IS_BT40)
    {
        ll_init();
    }
#endif      

#if defined (_3DD_FUNCTION_SUPPORT_) || defined (_DAPE_TEST_USE_GPIO1_AS_FRAME_SYNC)
#ifdef _3DD_FUNCTION_SUPPORT_
    if (IS_SUPPORT_3DG_APP)
#endif        
    {
        bt_3dd_driver_init();
    }
#endif

    init_hci();
}

