/***************************************************************************
 Copyright (C) Realtek Ltd.
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  Contains BZDMA handlers implementation.
 *
 * \author 
 *  austin <austin_chen@realtek.com>, (C) 2010
 */

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 200 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */
#include "bzdma.h"
#include "mem.h"
#include "bz_log_defines.h"
#include "mint_os.h"
#include "platform.h"
#include "bb_driver.h"
#include "g711.h"
#include "lc_internal.h"
#include "common_utils.h"
#include "timer.h"
#include "bt_fw_os.h"
#include "lc.h"
#ifdef _ENABLE_MAILBOX_
#include "mailbox.h"
#endif

#ifdef MWS_ENABLE
#include "mws_isr.h"
#include "mws.h"
#include "mws_imp.h"
extern UINT32 mws_current_interrupt_enable;
#endif
#ifdef _SUPPORT_WL_BT_SCOREBOARD_
#include "scoreboard.h"
#endif

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
UINT32 dape_acl_rx_rdptr = 0;
UINT32 dape_acl_rx_wrptr = 0;
#endif
/* ==================== Structure declaration Section ===================== */
/* ===================== Variable Declaration Section ===================== */
BZDMA_MANAGE Bzdma_Manager; /* the global variable of BZDMA management */
ALIGN(8) SECTION_SRAM BZDMA_TX_DESC_SET Bzdma_TxDesc_Set; 
BZDMA_TX_DESC_SET *pBzdma_TxDesc_Set; /* the pointer of Tx Descriptor Set */

UINT8 Bzdma_max_segs_of_tx_entry[BZDMA_TX_DESC_ENTRY_NUM] = {
    BZDMA_TX_ENTRY_MAX_SEGS_BROADCAST,
    BZDMA_TX_ENTRY_MAX_SEGS_SCO,        
    BZDMA_TX_ENTRY_MAX_SEGS_PICONET0,
    BZDMA_TX_ENTRY_MAX_SEGS_PICONET1,
    BZDMA_TX_ENTRY_MAX_SEGS_PICONET2,
    BZDMA_TX_ENTRY_MAX_SEGS_PICONET3,
    BZDMA_TX_ENTRY_MAX_SEGS_LE_ADV,
    BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST0,
    BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST1
#ifdef _NEW_BZDMA_FROM_V7_   
    ,BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO0,
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO1,
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO2
#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    ,BZDMA_TX_ENTRY_MAX_SEGS_NEW_SNIFF
#endif    
#endif
};
    
#ifdef _SCO_SEND_SINE_TONE_PKT_
const UINT16 Bzdma_500Hz_Sine_16BIT_SAMPLE[16] = {
    0x0000, 0x30FB, 0x5A81, 0x7640, 0x7FFF, 0x7640, 0x5A81, 0x30FB,
    0x0000, 0xCF05, 0xA57F, 0x89C0, 0x8001, 0x89C0, 0xA57F, 0xCF05
};
const UINT16 Bzdma_1KHz_Sine_16BIT_SAMPLE[8] = {
    0x0000, 0x5A81, 0x7FFF, 0x5A81, 0x0000, 0xA57F, 0x8001, 0xA57F    
};
const UINT16 Bzdma_2KHz_Sine_16BIT_SAMPLE[4] = {
    0x0000, 0x7FFF, 0x0000, 0x8001
};

SECTION_SRAM UINT16 Bzdma_Sw_SCO_Tx_Buf[3][48];
#endif

#ifdef _NEW_BZDMA_FROM_V8_
ALIGN(8) SECTION_SRAM BZDMA_BLEONLY_TX_DESC_SET Bzdma_Ble_TxDesc_Set; 
UINT8 bzdma_supported_le_link_num = LL_MAX_CONNECTION_UNITS;
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
UINT8 bzdma_supported_le_max_seg_num = 8;
UINT8 bzdma_supported_le_max_frag_num = 2;
UINT32 *pbzdma_used_heap_pointer = NULL;
#endif
#endif

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_bzdma_update_fw_rptr_of_ble_data_ring_fifo = NULL;
#endif

/* ================== Static Function Prototypes Section ================== */

/* ===================== Function Definition Section ====================== */

/**************************************************************************
 * Function     : bzdma_init
 *
 * Description  : This function initializes the FW/HW of BZDMA
 *
 * Parameters   : None.
 *
 * Returns      : None.
 *
 *************************************************************************/
void bzdma_init(void)
{
    UINT16 i;
    UINT8 seg_idx = 0;
    BZDMA_REG_S RegValue;

#ifdef _NEW_BZDMA_FROM_V8_
    BZDMA_BLEONLY_TX_DESC_SET *pBzdma_Ble_TxDesc_Set;
    UINT8 frag_idx = 0;
    UINT8 j;   
#endif

    /* disable BZDMA HW */
    WR_U32_BZDMA_REG(BZDMA_REG_CTRL, 0);
    
    memset(&Bzdma_Manager, 0, sizeof(Bzdma_Manager));
    memset(&Bzdma_TxDesc_Set, 0, sizeof(Bzdma_TxDesc_Set));

    /* let FW access Bzdma_TxDesc_Set uncacheable */
    pBzdma_TxDesc_Set = (BZDMA_TX_DESC_SET*)NON_DCATCH(&Bzdma_TxDesc_Set);

    /* init Tx Entry Status */
    for (i = 0; i < BZDMA_TX_DESC_ENTRY_NUM; i++)
    {
        Bzdma_Manager.TxEntSta[i].entry_idx = i;
        Bzdma_Manager.TxEntSta[i].pTxDesc = &pBzdma_TxDesc_Set->seg[seg_idx];
        Bzdma_Manager.TxEntSta[i].total_segs = Bzdma_max_segs_of_tx_entry[i];
        seg_idx += Bzdma_max_segs_of_tx_entry[i];
    }
    Bzdma_Manager.bmFreeTxEnt = BZDMA_DEFAULT_TX_FREE_ENTRY_BM;

    /* disable TXDMA commands and set tx descriptor based address */
    for (i = 0; i < BZDMA_TX_DESC_ENTRY_NUM; i++)
    {
        RegValue.DWord = 0;
        RegValue.TxCmd.desc_addr = ((UINT32)Bzdma_Manager.TxEntSta[i].pTxDesc) & 0xFFFF;
        RegValue.TxCmd.req = FALSE;
		
#if BZDMA_USE_DEDICATED_TX_ENTRY
        switch (i)
        {
        case BZDMA_TX_ENTRY_TYPE_BROADCAST:
            RegValue.TxCmd.pktid = BZDMA_TX_PID_BCST; 
            break;
        case BZDMA_TX_ENTRY_TYPE_SCO:
            RegValue.TxCmd.pktid = BZDMA_TX_PID_SCO;             
            break;
        case BZDMA_TX_ENTRY_TYPE_PICONET0:
        case BZDMA_TX_ENTRY_TYPE_PICONET1:
        case BZDMA_TX_ENTRY_TYPE_PICONET2:
        case BZDMA_TX_ENTRY_TYPE_PICONET3:
            RegValue.TxCmd.piconet_id = i - BZDMA_TX_ENTRY_TYPE_PICONET0;
            break;
#ifndef _NEW_BZDMA_FROM_V8_
        case BZDMA_TX_ENTRY_TYPE_LE_ADV:
        case BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST0:
        case BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST1:
            RegValue.TxCmd.le_en = 1;
            break;
#endif /* end of #ifndef _NEW_BZDMA_FROM_V8_ */

#ifdef _NEW_BZDMA_FROM_V7_  
        case BZDMA_TX_ENTRY_TYPE_NEW_SCO0:
            RegValue.TxCmd.pktid = BZDMA_TX_PID_SCO;
            RegValue.TxCmd.chid = 0;
            break;             

        case BZDMA_TX_ENTRY_TYPE_NEW_SCO1:
            RegValue.TxCmd.pktid = BZDMA_TX_PID_SCO;
            RegValue.TxCmd.chid = 1;
            break; 

#ifndef _SUPPORT_CSB_TRANSMITTER_
        case BZDMA_TX_ENTRY_TYPE_NEW_SCO2:
            RegValue.TxCmd.pktid = BZDMA_TX_PID_SCO;
            RegValue.TxCmd.chid = 2;
            break;        
#else
        case BZDMA_TX_ENTRY_TYPE_CSB:
            RegValue.TxCmd.pktid = BZDMA_TX_PID_CSB;
            break; 
#endif

#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
        case BZDMA_TX_ENTRY_TYPE_ACL_SNIFF:
            RegValue.TxCmd.pktid = BZDMA_TX_PID_ACL;
            RegValue.TxCmd.sniff_en = 1;
            break; 
#endif

#endif
        default:
            break;
        }
#endif
        WR_U32_BZDMA_REG(BZDMA_REG_TX_CMD(i), RegValue.DWord);
    }    

#ifdef _NEW_BZDMA_FROM_V8_
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
    UINT16 le_acl_segs =  bzdma_supported_le_link_num * bzdma_supported_le_max_seg_num;
    UINT16 le_acl_frags = le_acl_segs * bzdma_supported_le_max_frag_num;
    UINT16 total_segs = le_acl_segs + BZDMA_LEONLY_TX_ENTRY_TOTAL_ADV_SEGS;
    UINT16 total_frags = le_acl_frags + BZDMA_LEONLY_TX_ENTRY_TOTAL_ADV_FRAGS;
    UINT8 *pmem;
    UINT32 *pmem_dw;
    
    if (pbzdma_used_heap_pointer == NULL)
    {
        pmem = os_malloc((((total_segs + total_frags) << 2) + 8), RAM_TYPE_BUFFER_OFF);
        pmem_dw = (UINT32*)((UINT32)(((UINT32)pmem + 7) & ~0x07)); /* let 8 byte aligned */
        pbzdma_used_heap_pointer = pmem_dw;
    }
#endif

    memset(&Bzdma_Ble_TxDesc_Set, 0, sizeof(Bzdma_Ble_TxDesc_Set));
    /* let FW access BZDMA_BLEONLY_TX_DESC_SET uncacheable */
    pBzdma_Ble_TxDesc_Set = (BZDMA_BLEONLY_TX_DESC_SET*)NON_DCATCH(&Bzdma_Ble_TxDesc_Set);

#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
    pBzdma_Ble_TxDesc_Set->seg = (BZDMA_BLEONLY_TX_DESC_SEGMENT *)pbzdma_used_heap_pointer;
    pBzdma_Ble_TxDesc_Set->frag = (BZDMA_BLEONLY_TX_DESC_FRAGMENT *)(pbzdma_used_heap_pointer + total_segs);
#endif

    seg_idx = 0;

    /* init New Ble Adv Tx Entry Status */
    Bzdma_Manager.BleAdvTxEntSta.pTxSegDesc = &pBzdma_Ble_TxDesc_Set->seg[seg_idx];
    Bzdma_Manager.BleAdvTxEntSta.pTxFragDesc = &pBzdma_Ble_TxDesc_Set->frag[frag_idx];    
    Bzdma_Manager.BleAdvTxEntSta.max_segs = BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0;
    Bzdma_Manager.BleAdvTxEntSta.max_frags = BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0;

    for (i = 0; i < BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0; i++)
    {
        Bzdma_Manager.BleAdvTxEntSta.pTxSegDesc[i].DWord = 0;
        Bzdma_Manager.BleAdvTxEntSta.pTxSegDesc[i].start_addr = 
            (UINT32)&Bzdma_Manager.BleAdvTxEntSta.pTxFragDesc[i * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0];
    }

    seg_idx  += BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0;
    frag_idx += BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0 * BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0;
    
    /* init New Ble Data Tx Entry Status */
    for (i = 0; i < bzdma_supported_le_link_num; i++)
    {
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
        Bzdma_Manager.BleTxEntSta[i].pTxSegDesc = &pBzdma_Ble_TxDesc_Set->seg[seg_idx];
        seg_idx += bzdma_supported_le_max_seg_num;
        Bzdma_Manager.BleTxEntSta[i].pTxFragDesc = &pBzdma_Ble_TxDesc_Set->frag[frag_idx];
        frag_idx +=  bzdma_supported_le_max_seg_num * bzdma_supported_le_max_frag_num;
        Bzdma_Manager.BleTxEntSta[i].free_segs = bzdma_supported_le_max_seg_num;

        /* fill start address of all segment */
        for (j = 0; j < bzdma_supported_le_max_seg_num; j++)
        {
            Bzdma_Manager.BleTxEntSta[i].pTxSegDesc[j].start_addr = 
                (UINT32)&Bzdma_Manager.BleTxEntSta[i].pTxFragDesc[j * bzdma_supported_le_max_frag_num];
        }
#else
        Bzdma_Manager.BleTxEntSta[i].pTxSegDesc = &pBzdma_Ble_TxDesc_Set->seg[seg_idx];
        seg_idx += BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS;
        Bzdma_Manager.BleTxEntSta[i].pTxFragDesc = &pBzdma_Ble_TxDesc_Set->frag[frag_idx];
        frag_idx +=  BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_DATA;
        Bzdma_Manager.BleTxEntSta[i].free_segs = BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS;

        /* fill start address of all segment */
        for (j = 0; j < BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS; j++)
        {
            Bzdma_Manager.BleTxEntSta[i].pTxSegDesc[j].start_addr = 
                (UINT32)&Bzdma_Manager.BleTxEntSta[i].pTxFragDesc[j * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_DATA];
        }
#endif
    }
    Bzdma_Manager.bmFreeBleTxEnt = BZDMA_LEONLY_DEFAULT_RX_FREE_ENTRY_BM;

    /* disable Ble TXDMA command 0 (adv channel 37) and set tx descriptor based address */
    RegValue.DWord = 0;
    RegValue.BleTxCmd.desc_addr = (UINT32)Bzdma_Manager.BleAdvTxEntSta.pTxSegDesc;
    RegValue.BleTxCmd.adv_ch_id = 0;
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_CMD(0), RegValue.DWord);

    /* disable Ble TXDMA command 1 (adv channel 38) and set tx descriptor based address */
    RegValue.DWord = 0;
    RegValue.BleTxCmd.desc_addr = (UINT32)Bzdma_Manager.BleAdvTxEntSta.pTxSegDesc;
    RegValue.BleTxCmd.adv_ch_id = 1;
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_CMD(1), RegValue.DWord);

    /* disable Ble TXDMA command 2 (adv channel 39) and set tx descriptor based address */
    RegValue.DWord = 0;
    RegValue.BleTxCmd.desc_addr = (UINT32)Bzdma_Manager.BleAdvTxEntSta.pTxSegDesc;
    RegValue.BleTxCmd.adv_ch_id = 2;
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_CMD(2), RegValue.DWord);   

    /* disable Ble TXDMA command 3 and set tx descriptor based address */
    RegValue.DWord = 0;
    RegValue.BleTxCmd.desc_addr = (UINT32)Bzdma_Manager.BleTxEntSta[0].pTxSegDesc;
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_CMD(3), RegValue.DWord);

    /* fill descriptor control register */
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_    
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_DATA_DESC_CTRL, 
                        NEW_V8_BZDMA_SUPPORT_MAX_SEGNUM_GRADE_IN_ONE_LINK); 
#else
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_DATA_DESC_CTRL, 
                        BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS_GRADE); 
#endif

    /* clean all segment valid bits */
    for (i = 0; i < bzdma_supported_le_link_num; i++)
    {
        WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_ENTRY_SEGMENT_VALID(i), 0);
    }

    /* Disable ble only BZDMA interrupts - all TxDMA */
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_INT_ENABLE, 0);

    /* clear All Possible ble only interrupt pending bits */
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_INT_STATUS, 0x0000010F);    
#endif

    
    /* disable RXDMA commands */
    WR_U32_BZDMA_REG(BZDMA_REG_RX_CMD, 0);

    /* Disable BZDMA interrupts - all TxDMA and RxDMA */
    WR_U32_BZDMA_REG(BZDMA_REG_INT_ENABLE, 0);

    /* clear All Possible interrupt pending bits */
#ifndef _NEW_BZDMA_FROM_V7_  
    WR_U32_BZDMA_REG(BZDMA_REG_INT_STATUS, 0x0FFF);
#else
    WR_U32_BZDMA_REG(BZDMA_REG_INT_STATUS, 0x7FFF);
#endif

    /* flush TX and RX dma  */
	RegValue.DWord = RD_U32_BZDMA_REG(BZDMA_REG_ERR_RECOVERY);
    RegValue.ErrRecover.txdma_flush = 1;
    RegValue.ErrRecover.rxdma_flush = 1;    
    WR_U32_BZDMA_REG(BZDMA_REG_ERR_RECOVERY, RegValue.DWord);    
    /* wait tx and rx flush done */
    while (1)
    {
        RegValue.DWord = RD_U32_BZDMA_REG(BZDMA_REG_ERR_RECOVERY);
        if (RegValue.ErrRecover.rxdma_idle && RegValue.ErrRecover.txdma_idle)
        {
            break;
        }
    }
    RegValue.ErrRecover.txdma_flush = 0;
    RegValue.ErrRecover.rxdma_flush = 0;      
    WR_U32_BZDMA_REG(BZDMA_REG_ERR_RECOVERY, RegValue.DWord);

    /* flush codec (register from bluewiz) */
    RegValue.DWord = BB_read_baseband_register(VOICE_SETTING_REGISTER);
    RegValue.DWord |= BIT9 | BIT11;
    BB_write_baseband_register(VOICE_SETTING_REGISTER, RegValue.DWord);
    RegValue.DWord &= ~(BIT9 | BIT11);
    BB_write_baseband_register(VOICE_SETTING_REGISTER, RegValue.DWord);    

#ifndef _FIX_ESCO_RETRANSMIT_PAYLOAD_CONTENT_
#ifdef _DAPE_TEST_RESET_SCO_TX_FIFO
//#ifndef _ENABLE_8821_HW_SCO_DEFAULT_TX_DATA_
    /* set tx fifo to 0xAAAA */
    BB_write_baseband_register(TEST_CONTROL_REGISTER, 0x07);
    BB_write_baseband_register(0x9E, 0);

    /* push mute code of cvsd samples into sco tx fifo */
    for (i = 0; i < 1024; i++)
    {
        BB_write_baseband_register(SYNC_TX_FIFO1_REGISTER, 0xAAAA);
    }
//#endif
#endif
#endif

    /* enable BZDMA interrupts */
    WR_U32_BZDMA_REG(BZDMA_REG_INT_ENABLE, BZDMA_INT_EN_MASK);

#ifdef _NEW_BZDMA_FROM_V8_
    /* enable ble only BZDMA interrupts */
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_INT_ENABLE, BZDMA_LEONLY_TXCMD_INT_EN_MASK);
#endif

    /* enable BZDMA HW */
    WR_U32_BZDMA_REG(BZDMA_REG_CTRL, BZDMA_CTRL_CONFIG);

#ifdef _NEW_BZDMA_FROM_V8_
    bzdma_get_ble_dedicated_free_tx_entry(BZDMA_LEONLY_TX_ENTRY_TYPE_DATA);
    bzdma_send_ble_txcmd(BZDMA_LEONLY_TX_ENTRY_TYPE_DATA);
#endif

#ifdef _SCO_SEND_SINE_TONE_PKT_
    for (i = 0; i < 48; i++)
    {
        Bzdma_Sw_SCO_Tx_Buf[0][i] = Bzdma_500Hz_Sine_16BIT_SAMPLE[i & 0x0F];
        Bzdma_Sw_SCO_Tx_Buf[1][i] = Bzdma_1KHz_Sine_16BIT_SAMPLE[i & 0x07];
        Bzdma_Sw_SCO_Tx_Buf[2][i] = Bzdma_2KHz_Sine_16BIT_SAMPLE[i & 0x03];
    }   
#endif

    //RT_BT_LOG(GRAY, BZDMA_MSG_INIT, 1, 0);
}

/**************************************************************************
 * Function     : bzdma_get_dedicated_free_tx_entry
 *
 * Description  : This function is used to get a dedicated free entry for Tx DMA
 *
 * Parameters   : tx_type: the type of tx entry
 *
 * Returns      : the type of tx entry
 *
 *************************************************************************/
UINT8 bzdma_get_dedicated_free_tx_entry(UINT8 tx_type)
{   
    UINT8 ret_type;

	DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    do
    {
        if (tx_type >= BZDMA_TX_DESC_ENTRY_NUM)
        {
            /* error tx type */
            ret_type = BZDMA_TX_DESC_ENTRY_NUM;
            break;
        }

        if (!(Bzdma_Manager.bmFreeTxEnt & BIT(tx_type)))
        {
            /* no free tx entry */
            ret_type = BZDMA_TX_DESC_ENTRY_NUM;
            break;
        }

        Bzdma_Manager.bmFreeTxEnt &= ~(BIT(tx_type));
        Bzdma_Manager.TxEntSta[tx_type].used = TRUE;
        ret_type = tx_type;
    }
    while (0);
    
    MINT_OS_EXIT_CRITICAL();   

    if (ret_type == BZDMA_TX_DESC_ENTRY_NUM)
    {
        RT_BT_LOG(RED, BZDMA_MSG_GET_NO_FREE_TX_ENTRY, 1, tx_type);  
    }

    return ret_type;
}

/**************************************************************************
 * Function     : bzdma_get_free_tx_entry
 *
 * Description  : This function is used to get free entry for Tx DMA
 *
 * Parameters   : None.
 *
 * Returns      : entry_id: the id of free tx entry
 *
 *************************************************************************/
UINT8 bzdma_get_free_tx_entry(void)
{   
    UINT8 entry_id = BZDMA_TX_DESC_ENTRY_NUM;

	DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL(); 

    do
    {
        if (!Bzdma_Manager.bmFreeTxEnt)
        {
            /* no free tx entry */
            break;
        }

        /* search a free tx entry */
        for (entry_id = 0; entry_id < BZDMA_TX_DESC_ENTRY_NUM; entry_id++)
        {
            if (Bzdma_Manager.bmFreeTxEnt & BIT(entry_id))
            {
                Bzdma_Manager.TxEntSta[entry_id].used = TRUE;
                Bzdma_Manager.bmFreeTxEnt &= ~(BIT(entry_id));                
                break;
            }
        }
    }
    while (0);
    
    MINT_OS_EXIT_CRITICAL();   

    //RT_BT_LOG(GRAY, BZDMA_MSG_GET_FREE_TX_ENTRY, 1, entry_id);     
    return entry_id;
}

/**************************************************************************
 * Function     : bzdma_release_tx_entry
 *
 * Description  : This function is used to release free entry for Tx DMA. FW 
 *                needs to free the allocated memory in all segments of tx 
 *                descriptor 
 *
 * Parameters   : entry_id: the id of free tx entry
 *
 * Returns      : None
 *
 *************************************************************************/
void bzdma_release_tx_entry(UINT8 entry_id)
{
    DEF_CRITICAL_SECTION_STORAGE;
    
    MINT_OS_ENTER_CRITICAL();

    if ((entry_id < BZDMA_TX_DESC_ENTRY_NUM) && 
        !(Bzdma_Manager.bmFreeTxEnt & BIT(entry_id)))
    {
        Bzdma_Manager.bmFreeTxEnt |= BIT(entry_id);
        Bzdma_Manager.TxEntSta[entry_id].used = FALSE;        
        Bzdma_Manager.TxEntSta[entry_id].is_sco_lt = 0;
        Bzdma_Manager.TxEntSta[entry_id].ch_id = 0;
        Bzdma_Manager.TxEntSta[entry_id].lt_addr = 0;   
#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
        Bzdma_Manager.TxEntSta[entry_id].is_in_sniff = 0;
#endif
    }
    
    MINT_OS_EXIT_CRITICAL();

    //RT_BT_LOG(GRAY, BZDMA_MSG_FREE_TX_ENTRY, 1, entry_id);    
}

/**************************************************************************
 * Function     : bzdma_send_txcmd
 *
 * Description  : This function is used to send a TX command for Tx DMA. FW
 *                needs to prepare the relative settings of the entry first.
 *                HW will parse tx descriptor contents to copy SRAM data to 
 *                assigned TX FIFO
 *
 * Parameters   : entry_id: the id of free tx entry
 *
 * Returns      : None
 *
 *************************************************************************/
void bzdma_send_txcmd(UINT8 entry_id)
{
    BZDMA_REG_S Reg;
    
#if BZDMA_USE_DEDICATED_TX_ENTRY
    /* Flush Codec for SCO Tx Path */
    if (entry_id == BZDMA_TX_ENTRY_TYPE_SCO)
    {
        UINT16 reg_val;        
        DEF_CRITICAL_SECTION_STORAGE;

        MINT_OS_ENTER_CRITICAL();
        reg_val = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg_val | BIT9);
        BB_write_baseband_register(VOICE_SETTING_REGISTER, reg_val);        
        MINT_OS_EXIT_CRITICAL();
    }    
#endif

    /* fire TX cmd */    
    Reg.DWord = RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(entry_id));

    if (Reg.TxCmd.req)
    {
        RT_BT_LOG(RED, LE_MSG_TX_CMD_REENTRY, 2, entry_id, Reg.DWord);  	
    }

#if !BZDMA_USE_DEDICATED_TX_ENTRY
    Reg.TxCmd.pktid = Bzdma_Manager.TxEntSta[entry_id].pkt_id;
    Reg.TxCmd.piconet_id = Bzdma_Manager.TxEntSta[entry_id].piconet_id;
    Reg.TxCmd.le_en = Bzdma_Manager.TxEntSta[entry_id].le_mode_en;    
#endif
#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    Reg.TxCmd.sniff_en = Bzdma_Manager.TxEntSta[entry_id].is_in_sniff;
#endif
    Reg.TxCmd.chid = Bzdma_Manager.TxEntSta[entry_id].ch_id;
#ifndef _NEW_BZDMA_FROM_V8_
    Reg.TxCmd.le_cmd = Bzdma_Manager.TxEntSta[entry_id].le_pdu_cmd;
    Reg.TxCmd.conn_entry = Bzdma_Manager.TxEntSta[entry_id].le_conn_entry;
#endif
    Reg.TxCmd.req = 1;
    WR_U32_BZDMA_REG(BZDMA_REG_TX_CMD(entry_id), Reg.DWord); 
}

/**************************************************************************
 * Function     : bzdma_invalid_txcmd
 *
 * Description  : This function is used to invalid a TX command for Tx DMA. 
 *                FW uses this command to release hw command and sw entry 
 *                when HW can not invalidate the tx command in some case (
 *                for example: disconnect the channel)
 *
 * Parameters   : entry_id: the id of free tx entry
 *                am_addr: my LT_ADDR
 *                sco_id: my sco or esco entry index
 *
 * Returns      : result: (0: invalid txcmd success, 1: no txcomd to free)
 *
 *************************************************************************/
UINT8 bzdma_invalid_txcmd(UINT8 entry_id, UINT8 am_addr, UINT8 sco_id)
{
    BZDMA_REG_S Reg;
    UINT8 result = FALSE;
	
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL(); 
    
#ifdef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_
    if ((entry_id > BZDMA_TX_DESC_ENTRY_NUM) ||
        (Bzdma_Manager.bmFreeTxEnt & (1 << entry_id)))
#else
    if ((entry_id > BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST1) ||
    (Bzdma_Manager.bmFreeTxEnt & (1 << entry_id)))
#endif         
    {
        MINT_OS_EXIT_CRITICAL(); 
        return result;
    }

    switch (entry_id)
    {
        case BZDMA_TX_ENTRY_TYPE_BROADCAST:
            if (am_addr != 0)
            {
                result = TRUE;
            }
            break;
		
        case BZDMA_TX_ENTRY_TYPE_SCO:
#ifdef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_
        case BZDMA_TX_ENTRY_TYPE_NEW_SCO0:
#endif
            if (sco_id != Bzdma_Manager.TxEntSta[entry_id].ch_id)
            {
                result = TRUE;
            }
            break;
		
        case BZDMA_TX_ENTRY_TYPE_PICONET0:		
        case BZDMA_TX_ENTRY_TYPE_PICONET1:
        case BZDMA_TX_ENTRY_TYPE_PICONET2:
        case BZDMA_TX_ENTRY_TYPE_PICONET3:
            if (am_addr != Bzdma_Manager.TxEntSta[entry_id].lt_addr)
            {
                result = TRUE;
            }
            break;
		
        default:
            break;
    }
	
    if (!result)
    {
        Reg.DWord = RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(entry_id));
        if (Reg.TxCmd.req)
        {
            Reg.TxCmd.req = 0;
            WR_U32_BZDMA_REG(BZDMA_REG_TX_CMD(entry_id), Reg.DWord);  
        }

        bzdma_release_tx_entry(entry_id);
    }

    MINT_OS_EXIT_CRITICAL(); 
    return result;
}


#ifdef _NEW_BZDMA_FROM_V8_
/**************************************************************************
 * Function     : bzdma_get_dedicated_free_tx_entry
 *
 * Description  : This function is used to get a dedicated free entry for Tx DMA
 *
 * Parameters   : tx_type: the type of tx entry
 *
 * Returns      : the type of tx entry
 *
 *************************************************************************/
UINT8 bzdma_get_ble_dedicated_free_tx_entry(UINT8 tx_type)
{   
    UINT8 ret_type;

	DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    do
    {
        if (tx_type >= BZDMA_LEONLY_TX_DESC_ENTRY_NUM)
        {
            /* error tx type */
            ret_type = BZDMA_LEONLY_TX_DESC_ENTRY_NUM;
            break;
        }

        if (!(Bzdma_Manager.bmFreeBleTxEnt & BIT(tx_type)))
        {
            /* no free tx entry */
            ret_type = BZDMA_LEONLY_TX_DESC_ENTRY_NUM;
            break;
        }

        Bzdma_Manager.bmFreeBleTxEnt &= ~(BIT(tx_type));
        ret_type = tx_type;
    }
    while (0);
    
    MINT_OS_EXIT_CRITICAL();   

    if (ret_type == BZDMA_LEONLY_TX_DESC_ENTRY_NUM)
    {
        RT_BT_LOG(RED, BZDMA_MSG_GET_NO_FREE_TX_ENTRY, 1, tx_type);  
    }

    return ret_type;
}

/**************************************************************************
 * Function     : bzdma_release_tx_entry
 *
 * Description  : This function is used to release free entry for Tx DMA. FW 
 *                needs to free the allocated memory in all segments of tx 
 *                descriptor 
 *
 * Parameters   : entry_id: the id of free tx entry
 *
 * Returns      : None
 *
 *************************************************************************/
void bzdma_release_ble_tx_entry(UINT8 entry_id)
{
    DEF_CRITICAL_SECTION_STORAGE;
    
    MINT_OS_ENTER_CRITICAL();

    if ((entry_id < BZDMA_LEONLY_TX_DESC_ENTRY_NUM) && 
        !(Bzdma_Manager.bmFreeBleTxEnt & BIT(entry_id)))
    {
        Bzdma_Manager.bmFreeBleTxEnt |= BIT(entry_id); 
    }
    
    MINT_OS_EXIT_CRITICAL();

    //RT_BT_LOG(GRAY, BZDMA_MSG_FREE_TX_ENTRY, 1, entry_id);    
}


/**************************************************************************
 * Function     : bzdma_send_txcmd
 *
 * Description  : This function is used to send a TX command for Tx DMA. FW
 *                needs to prepare the relative settings of the entry first.
 *                HW will parse tx descriptor contents to copy SRAM data to 
 *                assigned TX FIFO
 *
 * Parameters   : entry_id: the id of free tx entry
 *
 * Returns      : None
 *
 *************************************************************************/
void bzdma_send_ble_txcmd(UINT8 entry_id)
{
    BZDMA_REG_S Reg;
    UINT8 loops = 1;
    UINT8 i;
    
    /* fire TX cmd */    

    if (entry_id == BZDMA_LEONLY_TX_ENTRY_TYPE_ADV0)
    {
        loops = 3;
    }

    for (i = 0; i < loops; i++)
    {
        Reg.DWord = RD_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_CMD(entry_id + i));

        if (Reg.BleTxCmd.req)
        {
            RT_BT_LOG(RED, LE_MSG_TX_CMD_REENTRY, 2, entry_id + i, Reg.DWord);  	
        }

        if (entry_id != BZDMA_LEONLY_TX_ENTRY_TYPE_DATA)
        {
            Reg.BleTxCmd.le_cmd = Bzdma_Manager.BleAdvTxEntSta.le_pdu_cmd;  
        }
        Reg.BleTxCmd.req = 1;
        WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_CMD(entry_id + i), Reg.DWord); 
    }
}

/**************************************************************************
 * Function     : bzdma_invalid_txcmd
 *
 * Description  : This function is used to invalid a TX command for Tx DMA. 
 *                FW uses this command to release hw command and sw entry 
 *                when HW can not invalidate the tx command in some case (
 *                for example: disconnect the channel)
 *
 * Parameters   : entry_id: the id of free tx entry
 *                am_addr: my LT_ADDR
 *                sco_id: my sco or esco entry index
 *
 * Returns      : result: (0: invalid txcmd success, 1: no txcomd to free)
 *
 *************************************************************************/
UINT8 bzdma_invalid_ble_txcmd(UINT8 entry_id)
{
    BZDMA_REG_S Reg;
    UINT8 loops = 1;
    UINT8 i;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL(); 

#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
    if ((entry_id >= bzdma_supported_le_link_num) ||
        (Bzdma_Manager.bmFreeBleTxEnt & (1 << entry_id)))
    {
        MINT_OS_EXIT_CRITICAL(); 
        return FALSE;
    }
#else
    if ((entry_id >= BZDMA_LEONLY_TX_MAX_ENTRY_NUM) ||
        (Bzdma_Manager.bmFreeBleTxEnt & (1 << entry_id)))
    {
        MINT_OS_EXIT_CRITICAL(); 
        return FALSE;
    }
#endif

    if (entry_id == BZDMA_LEONLY_TX_ENTRY_TYPE_ADV0)
    {
        loops = 3;
    }
    
    for (i = 0; i < loops; i++)
    {   
        Reg.DWord = RD_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_CMD(entry_id + i));
        if (Reg.BleTxCmd.req)
        {
            Reg.BleTxCmd.req = 0;
            WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_TX_CMD(entry_id + i), Reg.DWord);  
        }
    }
    bzdma_release_ble_tx_entry(entry_id);

    MINT_OS_EXIT_CRITICAL();
    
    return TRUE;
}


/**************************************************************************
 * Function     : bzdma_send_txcmd
 *
 * Description  : This function is used to send a TX command for Tx DMA. FW
 *                needs to prepare the relative settings of the entry first.
 *                HW will parse tx descriptor contents to copy SRAM data to 
 *                assigned TX FIFO
 *
 * Parameters   : entry_id: the id of free tx entry
 *
 * Returns      : None
 *
 *************************************************************************/
void bzdma_flush_ble_data_ring_fifo(UINT8 conn_entry, UINT8 flush_all)
{
    BZDMA_BLEONLY_DATA_TX_DESC_ENTRY_STATUS *pTxEnt;
    UINT8 cur_rptr;
    UINT32 cur_bm;
    
    /* fire TX cmd */    
    pTxEnt = &Bzdma_Manager.BleTxEntSta[conn_entry];

#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_    
    if (pTxEnt->free_segs == bzdma_supported_le_max_seg_num)
    {
        /* fifo is empty and no pending segments for releasing .. */
        return;
    }
#else
    if (pTxEnt->free_segs == BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS)
    {
        /* fifo is empty and no pending segments for releasing .. */
        return;
    }
#endif

    cur_rptr = pTxEnt->seg_rptr;

    /* update segment valid bit */
    // TODO: Capture actual rptr from ble cam
    cur_bm = RD_U32_BZDMA_REG(BZDMA_REG_BLEONLY_ENTRY_SEGMENT_VALID(conn_entry));

    if (flush_all)
    {
        pTxEnt->seg_wptr = pTxEnt->seg_rptr;
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_ 
        pTxEnt->free_segs = bzdma_supported_le_max_seg_num;
#else
        pTxEnt->free_segs = BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS;
#endif
        WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_ENTRY_SEGMENT_VALID(conn_entry), 0);        
    }
    else
    {
        if (cur_bm & (1 << cur_rptr))
        {
            /* update write pointer to next position of read pointer -- 
               keep one packet for sheduling */
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_         
            pTxEnt->seg_wptr = (pTxEnt->seg_rptr + 1) & (bzdma_supported_le_max_seg_num - 1);
            pTxEnt->free_segs = bzdma_supported_le_max_seg_num - 1;
#else
            pTxEnt->seg_wptr = (pTxEnt->seg_rptr + 1) & (BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS - 1);
            pTxEnt->free_segs = BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS - 1;
#endif
            WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_ENTRY_SEGMENT_VALID(conn_entry), 
                                                        (1 << cur_rptr));
        }
        else
        {
            /* mismatch !! or hw just updates read poinetr ?? */
            
        }
    }
}


/**************************************************************************
 * Function     : bzdma_update_fw_wptr_of_ble_data_ring_fifo
 *
 * Description  : This function is used to update write pointer of 
 *                ble only tx command for data packet atomicly.
 *
 * Parameters   : conn_entry: the id of tx entry
 *                write_pointer: new updated write pointer
 *
 * Returns      : None
 *
 *************************************************************************/
INLINE void bzdma_update_fw_wptr_of_ble_data_ring_fifo(UINT8 conn_entry, 
                                                  UINT8 write_pointer)
{
    VENDOR_WRITE(BTON_BT_LECMD_PUSH_REG, 
              BIT7 | ((conn_entry & 0x07) << 4) | (write_pointer & 0x0f)); 
}

/**************************************************************************
 * Function     : bzdma_send_packet_to_ble_data_ring_fifo
 *
 * Description  : This function is used to send a TX command for Tx DMA. FW
 *                needs to prepare the relative settings of the entry first.
 *                HW will parse tx descriptor contents to copy SRAM data to 
 *                assigned TX FIFO
 *
 * Parameters   : entry_id: the id of free tx entry
 *
 * Returns      : None
 *
 *************************************************************************/
UINT8 bzdma_send_packet_to_ble_data_ring_fifo(UINT8 conn_entry, UINT8 llid,
                            BZDMA_BLEONLY_TX_DESC_FRAGMENT *pfrag, UINT8 Nfrag)
{
    BZDMA_BLEONLY_DATA_TX_DESC_ENTRY_STATUS *pTxEnt;
    BZDMA_BLEONLY_TX_DESC_SEGMENT *pTxSeg;
    BZDMA_BLEONLY_TX_DESC_FRAGMENT *pTxFrag;
    UINT8 cur_wptr;
    UINT8 i;
    UINT8 PktSize = 0; 
#ifndef _NEW_BZDMA_BLECMD_PUSH_FLOW_
    UINT32 cur_bm;
#endif

    /* fire TX cmd */    
    pTxEnt = &Bzdma_Manager.BleTxEntSta[conn_entry];
    
    if (pTxEnt->free_segs == 0)
    {
        /* fifo is full and no free segments for scheduling .. */
        return FALSE;
    }

    cur_wptr = pTxEnt->seg_wptr;
    pTxSeg = &pTxEnt->pTxSegDesc[cur_wptr];
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_    
    pTxFrag = &pTxEnt->pTxFragDesc[cur_wptr * bzdma_supported_le_max_frag_num];
#else
    pTxFrag = &pTxEnt->pTxFragDesc[cur_wptr * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_DATA];
#endif

    /* fill all fragments */
    for (i = 0; i < Nfrag; i++)
    {
        pTxFrag[i].DWord = pfrag[i].DWord;
        PktSize +=  pfrag[i].len;   

        if (pfrag[i].len == 0)
        {
            /* hw can not handle zero length !! */
            return FALSE;
        }
    }

    /* fill full packet sie to tx segment */
    pTxSeg->len = PktSize;
    pTxSeg->llid = llid;

    /* update segment valid bit */
#ifdef _NEW_BZDMA_BLECMD_PUSH_FLOW_
    bzdma_update_fw_wptr_of_ble_data_ring_fifo(conn_entry, cur_wptr);
#else    
    cur_bm = RD_U32_BZDMA_REG(BZDMA_REG_BLEONLY_ENTRY_SEGMENT_VALID(conn_entry));
    cur_bm |= 1 << cur_wptr;
    WR_U32_BZDMA_REG(BZDMA_REG_BLEONLY_ENTRY_SEGMENT_VALID(conn_entry), cur_bm);
#endif

    /* update write pointer and decrease one segment space */
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_      
    pTxEnt->seg_wptr = (pTxEnt->seg_wptr + 1) & (bzdma_supported_le_max_seg_num - 1);
#else
    pTxEnt->seg_wptr = (pTxEnt->seg_wptr + 1) & (BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS - 1);
#endif
    pTxEnt->free_segs--;

    return TRUE;
}

/**************************************************************************
 * Function     : bzdma_update_fw_rptr_of_ble_data_ring_fifo
 *
 * Description  : This function is used to send a TX command for Tx DMA. FW
 *                needs to prepare the relative settings of the entry first.
 *                HW will parse tx descriptor contents to copy SRAM data to 
 *                assigned TX FIFO
 *
 * Parameters   : entry_id: the id of free tx entry
 *
 * Returns      : None
 *
 *************************************************************************/
UINT8 bzdma_update_fw_rptr_of_ble_data_ring_fifo(UINT8 conn_entry, 
                                           UINT8 free_cnts, 
                                           UINT8 check_cam)
{
    BZDMA_BLEONLY_DATA_TX_DESC_ENTRY_STATUS *pTxEnt;
    UINT8 i;
    UINT32 cur_bm;
    UINT8 mismatch = 0;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_bzdma_update_fw_rptr_of_ble_data_ring_fifo != NULL)
    {
        if (rcp_bzdma_update_fw_rptr_of_ble_data_ring_fifo(&conn_entry, 
                                                    free_cnts, check_cam))
        {
            return TRUE;
        }
    }
#endif

    /* fire TX cmd */    
    pTxEnt = &Bzdma_Manager.BleTxEntSta[conn_entry];

#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_     
    if ((pTxEnt->free_segs + free_cnts) > bzdma_supported_le_max_seg_num)
    {
        // TODO: add log ?
        /* fifo has no enough packets for releasing .. */
        free_cnts = bzdma_supported_le_max_seg_num - pTxEnt->free_segs;
    }
#else
    if ((pTxEnt->free_segs + free_cnts) > BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS)
    {
        // TODO: add log ?
        /* fifo has no enough packets for releasing .. */
        free_cnts = BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS - pTxEnt->free_segs;
    }
#endif

    if (free_cnts == 0)
    {
        return FALSE;
    }

    cur_bm = RD_U32_BZDMA_REG(BZDMA_REG_BLEONLY_ENTRY_SEGMENT_VALID(conn_entry));    

    /* update sw read pointer */
    for (i = 0; i < free_cnts; i++)
    {
        if (cur_bm & (1 << pTxEnt->seg_rptr))
        {
            /* hw can clean acked bit automatically */
            // TODO: add log ?
            mismatch = 0x44;  

#ifndef _WORKAROUND_BZDMA_HWFW_RPTR_MISMATCH_
            break;
#endif
        }
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
        pTxEnt->seg_rptr = (pTxEnt->seg_rptr + 1) & (bzdma_supported_le_max_seg_num - 1); 
#else
        pTxEnt->seg_rptr = (pTxEnt->seg_rptr + 1) & (BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS - 1); 
#endif
        pTxEnt->free_segs++;        
    }

#if defined(_EN_NEW_BZDMA_V8_LOG_) || defined(_WORKAROUND_BZDMA_HWFW_RPTR_MISMATCH_)
    if (check_cam || mismatch)
    {
        /* bluewiz only can update cam during ce end */
        if (check_cam)
        {
            UINT8 hw_rptr;
        hw_rptr = ll_driver_read_bzdma_rptr_from_cam(conn_entry);
        if ((hw_rptr != pTxEnt->seg_rptr) || mismatch)
        {
            RT_BT_LOG(RED, MSG_BZDMA_V8_RPTR_MISMATCH, 8,
                pTxEnt->seg_rptr, hw_rptr, cur_bm, pTxEnt->seg_wptr, 
                free_cnts, check_cam, mismatch, 
                ll_manager.conn_unit.handle[conn_entry].conn_counter); 
        }
    }
        else
    {
        RT_BT_LOG(RED, MSG_BZDMA_V8_RPTR_MISMATCH, 8,
            pTxEnt->seg_rptr, 88, cur_bm, pTxEnt->seg_wptr, 
            free_cnts, check_cam, mismatch, 
            ll_manager.conn_unit.handle[conn_entry].conn_counter);  
        }
    }
#endif
    
    return TRUE;
}


#endif



/**************************************************************************
 * Function     : bzdma_send_burst_rxcmd_and_wait_complete
 *
 * Description  : This function is used to send burst RX commands for Rx DMA and 
 *                wait the last command is idle. HW will follow the command to 
 *                copy data from RX FIFO to SRAM(s)
 *
 * Parameters   : pRxDesc: the pointer of rx descriptor
 *                cmd_count: the count of rx command
 *                fifo_type: the type of rx fifo
 *                sco_ch_id: sco channel id
 *
 * Returns      : None
 *
 *************************************************************************/
void bzdma_send_burst_rxcmd_and_wait_complete(BZDMA_RX_DESC_SEGMENT *pRxDesc, 
                             UINT8 cmd_count, UINT8 fifo_type, UINT8 sco_ch_id,
                             UINT8 wait_complete)
{
    BZDMA_REG_S Reg;
    UINT8 i;
    UINT32 cmd_id_settings;
    UINT32 ticks = 0;
    
    if ((cmd_count == 0) || (pRxDesc[0].len == 0))
    {
        return;
    }

#ifdef BZDMA_DONT_WAIT_RXCMD_COMPLETE
    while (1)
    {
        Reg.DWord = RD_U32_BZDMA_REG(BZDMA_REG_INT_STATUS); 
        if (!Reg.IntStatus.rx_busy)
        {
            break;
        }

        ticks++; 

        if (ticks > 100000)
        {
            RT_BT_LOG(RED, MSG_BZDMA_RXCMD_TIMEOUT, 0, 0); 
            break;
        }        
    }
#endif

    if (cmd_count > BZDMA_RX_DESC_ENTRY_NUM)
    {
        cmd_count = BZDMA_RX_DESC_ENTRY_NUM;
    }

    /* generate RX cmd */
    Reg.RxCmd.pktid = fifo_type;
    Reg.RxCmd.chid = sco_ch_id;

    for (i = 0; i < cmd_count; i++)
    {
        if (pRxDesc[i].len == 0)
        {
            break;
        }
        
        if (pRxDesc[i].flush)
        {
            cmd_id_settings = i | 0x08;
            Reg.RxCmd.dest_addr = 0;
        }
        else
        {
            cmd_id_settings = i;
            Reg.RxCmd.dest_addr = pRxDesc[i].addr;
        }
        Reg.RxCmd.len = pRxDesc[i].len;

        /* generate rx command id */
        WR_U32_BZDMA_REG(BZDMA_REG_RX_CMD1, cmd_id_settings);

        /* generate rx command and trigger source */
        Reg.RxCmd.start = 1;
        WR_U32_BZDMA_REG(BZDMA_REG_RX_CMD, Reg.DWord);        
        Reg.RxCmd.start = 0;
        WR_U32_BZDMA_REG(BZDMA_REG_RX_CMD, Reg.DWord); 
    }  
    
    /* polling the RX dma command queue until the last command is idle */
#ifdef BZDMA_DONT_WAIT_RXCMD_COMPLETE
	if (wait_complete)
#endif
	{
        ticks = 0;
	    while (1)
	    {
	        Reg.DWord = RD_U32_BZDMA_REG(BZDMA_REG_INT_STATUS); 
	        if (!Reg.IntStatus.rx_busy && 
	            (Reg.IntStatus.rxcmd_last_idx == (cmd_count - 1)))
	        {
	            break;
	        }

            ticks++; 

            if (ticks > 100000)
            {
                RT_BT_LOG(RED, MSG_BZDMA_RXCMD_TIMEOUT, 0, 0); 
                break;
            }
	    }
	} 

#ifdef _DAPE_TEST_ACL_RX_FIFO_DBG
    if (fifo_type == BZDMA_RX_PID_ACL)
    {
		///// dape test
		UINT32 dape_reg;
		UINT32 dape_acl_rx_rdptr_current;
		UINT32 dape_acl_rx_wrptr_current;
		dape_reg = RD_U32_BZDMA_REG(BZDMA_REG_ACL_RXFIFO_PTR);

		dape_acl_rx_rdptr_current = (dape_reg >> 16) & 0x7FF;
		dape_acl_rx_wrptr_current = dape_reg & 0x7FF;

		/*if (dape_acl_rx_rdptr_current == dape_acl_rx_rdptr)
		{
		    gpio_one_pull_high(0);
			gpio_one_pull_low(0);
			RT_BT_LOG(RED, DAPE_TEST_LOG501, 3, dape_reg, 
				dape_acl_rx_rdptr, dape_acl_wr_rdptr);
		}*/
		/*if (dape_acl_rx_rdptr_current != dape_acl_rx_wrptr_current)
		{
                    gpio_one_pull_high(1);
                    gpio_one_pull_low(1);
                    RT_BT_LOG(RED, DAPE_TEST_LOG501, 3, dape_reg, 
                              dape_acl_rx_rdptr, dape_acl_rx_wrptr);
		}*/
		
		dape_acl_rx_rdptr = dape_acl_rx_rdptr_current;
		dape_acl_rx_wrptr = dape_acl_rx_wrptr_current;
    }
#endif						
}

/**************************************************************************
 * Function     : bzdma_int_handler
 *
 * Description  : This function is used to handle BZDMA Interrupt Service
 *                Routine. It can bind to IRQ 10 for lx4181.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
SECTION_ISR_LOW void bzdma_int_handler(void)
{
    union {
        UINT32 u32tmp;
        BZDMA_REG_S_INT_STATUS IntStatus;
    } Reg;
    
    UINT8 i;
    UINT32 txcmd_status;
    while (1)
    {
        Reg.u32tmp = RD_U32_BZDMA_REG(BZDMA_REG_INT_STATUS);
        Reg.u32tmp &= BZDMA_INT_EN_MASK;
            
        if (!Reg.u32tmp)
        {
            /* no any pending tx/rx cmd interrupts, so quit now */
            break;
        }

        txcmd_status = Reg.u32tmp & BZDMA_TXCMD_INT_EN_MASK;
        
        /* check every possible txdma cmd finish interrupts (bit[8:0]) */
        for (i = 0; i < BZDMA_TX_DESC_ENTRY_NUM; i++)
        {
            if (txcmd_status == 0)
            {
                break;
            }

            if (txcmd_status & BIT(i))
            {
                /* write 1 to clear pending TX command interrupts */
                WR_U32_BZDMA_REG(BZDMA_REG_INT_STATUS, BIT(i));

                if (BZDMA_SCO_TXCMD_INT_MAP & BIT(i))
                {
                    /* Tx Command for (e)SCO Tx FIFO */                    
                    if (Bzdma_Manager.TxEntSta[i].is_sco_lt == TRUE)
                    {
                        pf_hci_free_tx_pkt(Bzdma_Manager.TxEntSta[i].ch_id);

                        /* reset tx queue of fw/hw if alternative setting is changed */
                        if (alternatset_change_tx_clear)
                        {
                            pf_hci_transport_reset_isoch_queue();
                        }                        
                    }
                    else
                    {
                        lc_handle_esco_free_pkts_callback_new(Bzdma_Manager.TxEntSta[i].ch_id);

                        /* reset tx queue of fw/hw if alternative setting is changed */
                        if (alternatset_change_tx_clear)
                        {
                            lc_reset_esco_fifo_new();
                        }
                    }
                }

                /* release tx entry */
                bzdma_release_tx_entry(i); 
                
                txcmd_status &= ~(BIT(i));    
            }
        }  
    }

#ifdef _ENABLE_MAILBOX_
    mailbox_interrupt_handler();
#endif

#ifdef  MWS_ENABLE
    if (IS_BT_MWS)
    {
        mws_current_interrupt_enable = mws_read_register_new(INTERRUPT_EN_REG);
        mws_int_handler();
    }
#endif 

#ifdef _SUPPORT_WL_BT_SCOREBOARD_
    if (IS_USE_SCOREBOARD)
    {
        wl_bt_scoreboard_interrupt_handler();
    }
#endif

}

