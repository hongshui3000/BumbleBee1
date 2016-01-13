/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Baseband driver implementation.
 */

/********************************* Logger *************************/
enum { __FILE_NUM__= 1 };
/********************************* Logger *************************/
#include "bt_fw_types.h"
#include "bb_driver.h"
#include "lc_internal.h"
#include "lc.h"
#include "led_debug.h"
#include "bt_fw_os.h"
#include "bz_debug.h"
#include "bzdma.h"
#include "le_ll_driver.h"

#ifdef _CCH_SC_ECDH_P256_
#include "bz_auth_internal.h"
#endif
#ifdef _DAPE_TEST_CHK_CSB_SYNC_TX
#include "gpio.h"
#endif
static UCHAR bb_flow_state = BB_GO;
extern UCHAR sync_link_codec_state;

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_bb_write_baseband_register_func = NULL;
#endif

#ifdef _DAPE_SNIFF_PKT_TEST
extern UINT32 lc_get_least_sniff_interval(UINT16 *sniff_ce_index, UINT16 *sniff_attempt);
#else
extern UINT32 lc_get_least_sniff_interval(void);
#endif

extern void* bz_isoch_get_rx_q(UINT16 sync_conn_handle);

#define BB_RX_FIFO_FLUSH_TEMP_GRADE     8
#define BB_RX_FIFO_FLUSH_TEMP_SIZE      (1 << (BB_RX_FIFO_FLUSH_TEMP_GRADE))
SECTION_SRAM UINT8 bb_rx_fifo_flush_target[BB_RX_FIFO_FLUSH_TEMP_SIZE];
#ifdef _DAPE_NO_BLOCK_LEGACY_WEHN_SLV_EXIT_SNIFF
extern UINT8 lc_le_pause_status;
extern UINT8 lc_le_pause_status_for_sniff;
#endif
#ifdef _DAPE_TEST_NEW_HW_BLOCK_ACL_FOR_LE
extern UINT8 g_enable_le_block_legacy;
#endif
/**
 * Resets the BB status variables.
 *
 * \param None.
 *
 * \return None.
 */
void BB_reset_variables(void)
{
    bb_flow_state = BB_GO;

    return;
}

/**
 * Reads a baseband register and returns the upper octet.
 *
 * \param bb_reg_offset The address of the baseband register
 *
 * \return The upper 8 bits of the value read from the register.
 */
UINT16 BB_read_baseband_register_upper_octet(UCHAR bb_reg_offset)
{
    volatile UINT16 *bb_reg_addr = (volatile UINT16*)(BB_BASE_ADDR +
                                   bb_reg_offset);
    return(UINT16)((*(volatile UINT16*)bb_reg_addr) >> 8);
}

/**
 * Writes a baseband register.
 *
 * \param bb_reg_offset The address of the baseband register
 * \param val The value to be written.
 *
 * \return None.
 */
void BB_write_baseband_register(UINT16 bb_reg_offset, UINT16 val)
{
    volatile UINT16 *bb_reg_addr =
        (volatile UINT16*)(BB_BASE_ADDR + bb_reg_offset);

/* add by yilinli, to reserve the flexibility by rom code patch */
#ifdef POWER_SAVE_FEATURE
    /* mark by austin. because current design no follow original design,
       it can not be a fatel case */
    if(sleep_mode_param.bb_sm_sts != BB_NORMAL_MODE)
    {
#ifdef _ROM_CODE_PATCHED_
        if (rcp_bb_write_baseband_register_func != NULL)
        {
            if ( rcp_bb_write_baseband_register_func((void*)(&bb_reg_offset), &val ) )
            {
                return;
            }
        }
        else
#endif
        {
             bb_write_baseband_register_func_imp_count ++;

             /* we can check to write which address for debug!! */
             RT_BT_LOG(RED, BB_DRIVER_134, 3,
                  bb_reg_offset, val, sleep_mode_param.bb_sm_sts);
        }
    }
#endif

    *((volatile UINT16 *)bb_reg_addr) = val;
    return;
}


/**
 * Writes a particular value to the upper octet of a baseband register.
 * This function assumes that the registger is RW.
 *
 * \param bb_reg_offset The address of the baseband register
 * \param val The value to be written.
 *

 * \return None.
 */
void BB_write_baseband_register_upper_octet(UINT16 bb_reg_offset, UINT16 val)
{
    volatile UINT16 *bb_reg_addr = (volatile UINT16*)(BB_BASE_ADDR +
                                   bb_reg_offset);

    val = (UINT16) ( (*bb_reg_addr & 0x00FF) | (val << 8));

    *((volatile UINT16 *)bb_reg_addr) = val;

    return;
}

/**
 * Writes a particular value to the lower octet of a baseband register.
 * This function assumes that the registger is RW.
 *
 * \param bb_reg_offset The address of the baseband register
 * \param val The value to be written.
 *
 * \return None.
 */
void BB_write_baseband_register_lower_octet(UINT16 bb_reg_offset, UINT16 val)
{
    volatile UINT16 *bb_reg_addr = (volatile UINT16*)(BB_BASE_ADDR +
                                   bb_reg_offset);

    val = (UINT16) ( (*bb_reg_addr & 0xFF00) | (val & 0x00FF));
    *((volatile UINT16 *)bb_reg_addr) = val;
}

/**
 * Writes inquiry parity bits to the baseband.
 *
 * \param val The parity bits value to be written to the baseband.
 *
 * \return None.
 */
void bb_write_inq_parity_bit(UINT16 val)
{
    volatile UINT16 *bb_reg_addr =
        (volatile UINT16*)(BB_BASE_ADDR+ INQUIRY_PARITY_BITS_REGISTER3);

    val = (UINT16) ( (*bb_reg_addr & 0x3FFF) | (val << 14));
    *((volatile UINT16 *)bb_reg_addr) = val;
}

/**
 * Writes remote device parity bits to the baseband. This function will
 * be invoked for paging procedure.
 *
 * \param val The parity bits value to be written to the baseband.
 *
 * \return None.
 */
void bb_write_remote_parity_bit(UINT16 val, UCHAR piconet_id)
{
    UINT16 address;

    if (piconet_id > SCA_PICONET_MAX)
    {
        return;
    }

    if(piconet_id == SCA_PICONET_FIRST)
    {
        address = PICONET1_PARITY_BITS_REGISTER3;
        val = (UINT16) ( ((BB_read_baseband_register(address)) & 0xCFFF) | (val << 12));
    }
    else
    {
        address = reg_PICONET_PARITY_BITS3[piconet_id];
    }

    BB_write_baseband_register(address, val);

    return;
}

/**
 * Writes local device parity bits to the baseband. This function will
 * be invoked for paging procedure.
 *
 * \param val The parity bits value to be written to the baseband.
 *
 * \return None.
 */
void bb_write_local_parity_bit(UINT16 val)
{
    volatile UINT16 *bb_reg_addr =
        (volatile UINT16*)(BB_BASE_ADDR+ LOCAL_PARITY_BITS_REGISTER3);

    val = (UINT16) ( (*bb_reg_addr & 0xF3FF) | (val << 10));
    *((volatile UINT16 *)bb_reg_addr) = val;
}

/**
 * Writes data to the ACL/Broadcast/SCO TX FIFO in the baseband from SRAM
 * via BZDMA.
 *
 * \param ptxdesc: Pointer to the tx descriptor that contains the memory block
 *                 information
 * \param seg_num: Number of segment.
 * \param am_addr: The logical transport address of the connection.
 * \param piconet_id: The physical piconet ID of the connection.
 * \param sco_en: write data to SCO tx fifo
 * \param sco_ch_id: the index of (e)SCO channel
 *
 * \return None.
 */
void BB_dma_write_baseband_TX_FIFO(BZDMA_TX_DESC_SEGMENT *ptxdesc,
                                   UINT8 seg_num, UINT8 am_addr,
                                   UINT8 piconet_id, UINT8 sco_en,
                                   UINT8 sco_ch_id)
{
    UINT8 tx_entry_type;
    UINT8 tx_id;
    UINT8 i;

    if ((ptxdesc == NULL) || (ptxdesc[0].len == 0) || (seg_num == 0))
    {
        /* do not need to move data to txfifo at zero length */
        return;
    }

    if (sco_en)
    {
        /* transmit SCO data */
#ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_
        tx_entry_type = BZDMA_TX_ENTRY_TYPE_SCO;
#else
        tx_entry_type = BZDMA_TX_ENTRY_TYPE_NEW_SCO0;
#endif
    }
    else
    {
        if (am_addr == 0)
        {
            /* transmit broadcast data. HW has known which piconet is master role */
            tx_entry_type = BZDMA_TX_ENTRY_TYPE_BROADCAST;
        }
        else
        {
            /* transmit piconet or scatternet data */
            tx_entry_type = BZDMA_TX_ENTRY_TYPE_PICONET0 + piconet_id;
        }
    }
    tx_id = bzdma_get_dedicated_free_tx_entry(tx_entry_type);

    if (tx_id == BZDMA_TX_DESC_ENTRY_NUM)
    {
        /* TODO: no get free entry. So we may add this command to queue ? */
        //RT_BT_LOG(RED, BT_DRIVER_TX_DMA_WRITE, 1, tx_entry_type);
        //RT_BT_LOG(RED, CCH_DBG_061, 3, tx_entry_type, piconet_id, am_addr);
    }
    else
    {
        BZDMA_TX_DESC_ENTRY_STATUS *pTxEnt = &Bzdma_Manager.TxEntSta[tx_id];
        pTxEnt->total_len = 0;
        pTxEnt->lt_addr = am_addr;

        /* fill tx descriptor */
        for (i = 0; i < seg_num; i++)
        {
            pTxEnt->pTxDesc[i].DWord0 = ptxdesc[i].DWord0;
            pTxEnt->pTxDesc[i].DWord1 = ptxdesc[i].DWord1;
            pTxEnt->total_len += ptxdesc[i].len;
        }

        if (sco_en)
        {
            pTxEnt->ch_id = sco_ch_id;
        }

        LC_EXIT_SM_MODE();

        /* send tx command */
        bzdma_send_txcmd(tx_id);
    }
}

#if (defined (SCO_OVER_HCI) || defined(COMPILE_ESCO))
#if 0
/**
 * Write data to Synchronous connection TX FIFO from SRAM via BZDMA.
 *
 * \param buf: Input buffer allocated in SRAM
 * \param length: Number of bytes to write.
 * \param sco_ch_id: the SCO Channel Index (0~2)
 *
 * \return None.
 */
void BB_dma_write_baseband_SYNC_TX_FIFO(UCHAR *buf, UINT16 length,
                                        UCHAR sco_ch_id)
{
    UINT8 tx_id;

    if (length == 0)
    {
        /* do not need to move data to txfifo at zero length */
        return;
    }

    tx_id = bzdma_get_dedicated_free_tx_entry(BZDMA_TX_ENTRY_TYPE_SCO);

    if (tx_id == BZDMA_TX_DESC_ENTRY_NUM)
    {
        /* TODO: no get free entry. So we may add this command to queue ? */
        //RT_BT_LOG(RED, BT_DRIVER_TX_DMA_WRITE, 1, BZDMA_TX_ENTRY_TYPE_SCO);
    }
    else
    {
        BZDMA_TX_DESC_ENTRY_STATUS *pTxEnt = &Bzdma_Manager.TxEntSta[tx_id];
        pTxEnt->total_len = length;

        /* fill tx descriptor */
        pTxEnt->pTxDesc[0].start_addr = (UINT32)buf;
        pTxEnt->pTxDesc[0].len = length;
        pTxEnt->pTxDesc[0].isLast = TRUE;
        pTxEnt->ch_id = sco_ch_id;

        /* send tx command */
        bzdma_send_txcmd(tx_id);
    }
}
#endif

/**
 * Move data from Synchronous connection RX fifo to dedicated SRAM via BZDMA.
 *
 * \param buf: Output buffer allocated in SRAM
 * \param length Number of bytes to read from the RX Fifo.
 * \param sco_ch_id: the SCO Channel Index (0~2)
 *
 * \return None.
 */
void BB_dma_read_baseband_SYNC_RX_FIFO(UCHAR *buf, UINT16 length,
                                        UCHAR sco_ch_id, UINT8 wait_complete)
{
    if (length > 0)
    {
        BZDMA_RX_DESC_SEGMENT rxdesc;
        rxdesc.addr = (UINT32)buf;
        rxdesc.len = length;
        rxdesc.flush = FALSE;
        bzdma_send_burst_rxcmd_and_wait_complete(&rxdesc, 1,
                BZDMA_RX_PID_SCO, sco_ch_id, wait_complete);
    }
}
#endif

/**
* Writes data to the TX FIFO in the baseband.
*
* \param buffer Pointer to the buffer that contains the payload.
* \param length Number of bytes to be written.
* \param am_addr The logical transport address of the connection.
* \param lcl_which_fifo_to_use The FIFO to be used for the connection.
*
* \return None.
*/
void BB_write_baseband_TX_FIFO_scatternet(UCHAR *buffer, UINT16 length,
        UCHAR am_addr, UCHAR lcl_which_fifo_to_use)
{
    if (length == 0)
    {
        return;
    }

    LC_EXIT_SM_MODE();

    if (lcl_which_fifo_to_use > SCA_PICONET_MAX)
    {
        lcl_which_fifo_to_use = SCA_PICONET_MAX;
    }

    BZDMA_TX_DESC_SEGMENT txdesc[1];
    txdesc[0].DWord0 = 0;
    txdesc[0].DWord1 = 0;
    txdesc[0].start_addr = (UINT32)buffer;
    txdesc[0].len = length;
    txdesc[0].isLast = TRUE;
    BB_dma_write_baseband_TX_FIFO(txdesc, 1, am_addr,
                                  lcl_which_fifo_to_use, 0, 0);
}

/**
* Starts the BaseBand Tpoll timer (and tpoll's) for the am_addr.
*
* \param am_addr AM Address.
* \param t_poll  T-Poll timer value in slots.
* \param phy_piconet_id  Phy piconet ID of the connection.
*
* \return None.
*/
void BB_start_tpoll(UCHAR am_addr, UINT16 t_poll, UCHAR phy_piconet_id)
{
    UCHAR lut_index;
    UINT16 temp_var;
    UINT32 tpoll_offset;

    DEF_CRITICAL_SECTION_STORAGE;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);
    temp_var = (UINT16) ((am_addr << 5) | (phy_piconet_id << 11));

    MINT_OS_ENTER_CRITICAL();
    lc_get_clock_in_scatternet(&tpoll_offset, phy_piconet_id);
    tpoll_offset = tpoll_offset >> 1;
    tpoll_offset += 20;
    BB_write_baseband_register(CONNECTOR_REGISTER, temp_var);
    BB_write_baseband_register(SNIFF_SLOT_OFFSET_INTERVAL_REGISTER,
                               (UINT16)(tpoll_offset));

    BB_write_baseband_register(TPOLL_HOLD_SNIFF_INTERVAL_REGISTER, t_poll);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_START_TPOLL);

    lc_is_tpoll_started[lut_index] = TRUE;
    MINT_OS_EXIT_CRITICAL();

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, BB_DRIVER_655, 3, am_addr, t_poll, phy_piconet_id);
#endif

    return;
}

/**
* Stops the Tpoll timer (and tpoll's) for the am_addr.
*
* \param am_addr AM Address of the connection.
* \param phy_piconet_id The physical piconet ID of the connection.
*
* \return None.
*/
void BB_stop_tpoll(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UCHAR lut_index;
    UINT16 temp_var;
    DEF_CRITICAL_SECTION_STORAGE;

    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);
    temp_var = (UINT16) ((am_addr << 5) | (phy_piconet_id << 11));

    MINT_OS_ENTER_CRITICAL();

    BB_write_baseband_register(CONNECTOR_REGISTER, temp_var);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_STOP_TPOLL);

    lc_is_tpoll_started[lut_index] = FALSE;
    MINT_OS_EXIT_CRITICAL();

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, BB_DRIVER_733, 2, am_addr, phy_piconet_id);
#endif

    return;
}

/**
* Writes AFH map in the baseband.
*
* \param am_addr AM Address.
* \param phy_piconet_id Phy piconet ID of the connection.
* \param afh_map The 79 channels to be wriiten to the baseband.
*
* \return None.
*/
void BB_write_afh_map(UCHAR am_addr, UCHAR phy_piconet_id, UCHAR *afh_map)
{

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT16 ce_index;

    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr,
       phy_piconet_id, &ce_index) == API_SUCCESS)
    {
        UCHAR lut_index = lc_get_lut_index_from_phy_piconet_id(
                am_addr, phy_piconet_id);
#ifdef _DAPE_TEST_CHK_CSB_SYNC_TX
RT_BT_LOG(WHITE, YL_DBG_HEX_3, 3,
              am_addr, phy_piconet_id, lut_index);
#endif

        DEF_CRITICAL_SECTION_STORAGE;
        LC_EXIT_SM_MODE();
        MINT_OS_ENTER_CRITICAL();

        BB_write_afh_in_cam(ce_index, afh_map);

        UINT16 val = BB_read_baseband_register(AFH_CHANNEL_MAP_EN_REG);
        BB_write_baseband_register(AFH_CHANNEL_MAP_EN_REG, val|(BIT0<<lut_index));

        MINT_OS_EXIT_CRITICAL();

    }
#else

#ifdef COMPILE_AFH_HOP_KERNEL
    UINT16 conn_reg;
    UINT16 temp_reg1;
    UINT16 temp_reg2;
    UINT16 temp_reg3;
    UINT16 temp_reg4;
    UINT16 temp_reg5;

    DEF_CRITICAL_SECTION_STORAGE;

    conn_reg = (UINT16) ( (am_addr << 5) | (phy_piconet_id << 11) );

    LC_EXIT_SM_MODE();

    temp_reg1 = (afh_map[1] << 8) | afh_map[0];
    temp_reg2 = (afh_map[3] << 8) | afh_map[2];
    temp_reg3 = (afh_map[5] << 8) | afh_map[4];
    temp_reg4 = (afh_map[7] << 8) | afh_map[6];
    temp_reg5 = (afh_map[9] << 8) | afh_map[8];
    temp_reg5 |= AFH_ENABLE_BIT;

    MINT_OS_ENTER_CRITICAL();

    BB_write_baseband_register(CONNECTOR_REGISTER, conn_reg);
    BB_write_baseband_register(AFH_CHANNEL_MAP_REGISTER0, temp_reg1);
    BB_write_baseband_register(AFH_CHANNEL_MAP_REGISTER1, temp_reg2);
    BB_write_baseband_register(AFH_CHANNEL_MAP_REGISTER2, temp_reg3);
    BB_write_baseband_register(AFH_CHANNEL_MAP_REGISTER3, temp_reg4);
    BB_write_baseband_register(AFH_CHANNEL_MAP_REGISTER4, temp_reg5);

    MINT_OS_EXIT_CRITICAL();

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, BB_DRIVER_846, 2, am_addr, phy_piconet_id);
#endif
#endif
#endif

    return;
}


#ifdef COMPILE_CHANNEL_ASSESSMENT

/**
 * Clears the AFH Enable bit in AFH_CHANNEL_MAP_REGISTER4
 *
 * \param am_addr AM Address.
 * \param phy_piconet_id Physical picoent ID of the connection.
 *
 * \return None.
 */
void BB_disable_afh(UCHAR am_addr, UCHAR phy_piconet_id)
{


#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id_for_disable_afh(
                am_addr, phy_piconet_id);

    DEF_CRITICAL_SECTION_STORAGE;
    LC_EXIT_SM_MODE();
    MINT_OS_ENTER_CRITICAL();


    UINT16 val = BB_read_baseband_register(AFH_CHANNEL_MAP_EN_REG);
    val &= (~(BIT0<<lut_index));
    BB_write_baseband_register(AFH_CHANNEL_MAP_EN_REG, val);

    MINT_OS_EXIT_CRITICAL();

#else

#ifdef COMPILE_AFH_HOP_KERNEL
    UINT16 conn_reg;
    UINT16 map;
    DEF_CRITICAL_SECTION_STORAGE;

    conn_reg = (UINT16) ( (am_addr << 5) | (phy_piconet_id << 11) );

    LC_EXIT_SM_MODE();

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(CONNECTOR_REGISTER, conn_reg);

    map = BB_read_baseband_register(AFH_CHANNEL_MAP_REGISTER4);
    map = (UINT16) (map & 0x7FFFU);
    BB_write_baseband_register(AFH_CHANNEL_MAP_REGISTER4, map);

    MINT_OS_EXIT_CRITICAL();
#endif
#endif

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, BB_DRIVER_968, 2, am_addr, phy_piconet_id);
#endif

    return;
}
#endif

/**
 * Disables NBC for CRC Packets. Please call this function only from
 * interrupts or critical section.
*
* \param None.
*
* \return None.
*/
void BB_disable_NBC(UCHAR phy_piconet_id)
{
    BZ_REG_S_PICONET_INFO pic_info_reg;
    UINT16 address;

    if(phy_piconet_id > SCA_PICONET_MAX)
    {
        return;
    }
    address = reg_PICONET_INFO[phy_piconet_id];

    *(UINT16*)&pic_info_reg = BB_read_baseband_register(address);
    pic_info_reg.retx_count_en = FALSE;
    BB_write_baseband_register(address, *(UINT16*)&pic_info_reg);

    return;
}

/**
* Enables NBC for CRC Packets.
*
 * \param phy_piconet_id The piconet ID of the connection..
*
* \return None.
*/
void BB_enable_NBC(UCHAR phy_piconet_id, UCHAR nbc_timeout,
                   UCHAR enable_slot_based_nbc)
{
    UINT16 reg;
    BZ_REG_S_PICONET_INFO pic_info_reg;
    UINT16 address;

    if(phy_piconet_id > SCA_PICONET_MAX)
    {
        return;
    }

    address = reg_PICONET_INFO[phy_piconet_id];

    reg = BB_read_baseband_register_upper_octet(M_ACCESS_REGISTER);
    reg = (UINT16)((reg & 0x0F) | ((nbc_timeout & 0x0f) << 4) );
    BB_write_baseband_register_upper_octet(M_ACCESS_REGISTER, reg);

    *(UINT16*)&pic_info_reg = BB_read_baseband_register(address);
    pic_info_reg.retx_count_en = TRUE;
    BB_write_baseband_register(address, *(UINT16*)&pic_info_reg);

    AND_val_with_bb_reg_macro(SCO_PACKET_TYPE_REGISTER, (~BIT13));

    return;
}

/**
* Writes the Encryption Key for the am_addr.
*
* \param am_addr  AM Address of the connection.
* \param key_dash Encryption Key.
*
* \return None.
*/
void BB_write_encryption_keys(UCHAR am_addr, UCHAR phy_piconet_id,
                                      UCHAR* key_dash)
{

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT16 ce_index;

    if (LMP_GET_CE_INDEX_FROM_AM_ADDR_PPI(am_addr,
       phy_piconet_id, &ce_index) == API_SUCCESS)
    {
        DEF_CRITICAL_SECTION_STORAGE;
        LC_EXIT_SM_MODE();
        MINT_OS_ENTER_CRITICAL();

        BB_write_e0_key_in_cam(ce_index, key_dash);

        MINT_OS_EXIT_CRITICAL();

    }

#else

    UINT16 conn_reg;
    UINT16 temp_reg[8];
    UINT8 i;

    DEF_CRITICAL_SECTION_STORAGE;

    conn_reg = (UINT16)( (am_addr << 5) | (phy_piconet_id << 11));

    /* non-2byte aligned */
    for (i = 0; i < 16; i += 2)
    {
        temp_reg[i >> 1] = key_dash[i] | (key_dash[i + 1] << 8);
    }

    LC_EXIT_SM_MODE();

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(CONNECTOR_REGISTER, conn_reg);
    BB_write_baseband_register(ENCRYPTION_KEY_REGISTER1, temp_reg[0]);
    BB_write_baseband_register(ENCRYPTION_KEY_REGISTER2, temp_reg[1]);
    BB_write_baseband_register(ENCRYPTION_KEY_REGISTER3, temp_reg[2]);
    BB_write_baseband_register(ENCRYPTION_KEY_REGISTER4, temp_reg[3]);
    BB_write_baseband_register(ENCRYPTION_KEY_REGISTER5, temp_reg[4]);
    BB_write_baseband_register(ENCRYPTION_KEY_REGISTER6, temp_reg[5]);
    BB_write_baseband_register(ENCRYPTION_KEY_REGISTER7, temp_reg[6]);
    BB_write_baseband_register(ENCRYPTION_KEY_REGISTER8, temp_reg[7]);
    MINT_OS_EXIT_CRITICAL();
#endif
    return;
}

#ifdef _CCH_SC_ECDH_P256_

void BB_write_sc_cam_ini(UINT16 ce_index)
{
    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);
    UINT8 i;


    if(( lut_index == 0)||( lut_index > 11))
    {
        return;
    }

    lut_index = lut_index -1;

// Initial CAM
#ifdef _CCH_AFH_E0_AES_IN_CAM_
#ifdef _DAPE_FIX_SLV_DISCONN_IN_SCATTERNET
    /* Reset Encryption Part. */
    for (i = 0; i < 12; i ++)
    {
        BB_write_sc_cam( (lut_index*12 + i), 0);
    }

    /* Reset AFH map part. Note we should not reset other lut index. */
    /* previous lut index is used for security.*/
    lut_index += 1;
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER2 + lut_index*CAM_ADDR_OFFSET_LAYER2_SIZE + CAM_ADDR_OFFSET_AFH;
    for (i = 0; i < 3; i ++)
    {
        BB_write_sc_cam( (cam_offset + i), 0);
    }
    RT_BT_LOG(BLUE, YL_DBG_HEX_3, 3, ce_index, lut_index, cam_offset);
#else
    for (i = 0; i < CAM_ADDR_OFFSET_SIZE; i ++)
    {
        BB_write_sc_cam(i, 0);
    }
#endif
#else
    for (i = 0; i < 12; i ++)
    {
        BB_write_sc_cam( (lut_index*12 + i), 0);
    }
#endif

#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(BLUE, CCH_DBG_169, 2, ce_index, lut_index);
#endif
}


void BB_write_sc_encryption_keys(UINT16 ce_index, UCHAR* key_dash)
{
    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);
    UINT8 i;
    UINT32 temp_u32[4];

    if( lut_index == 0)
    {
        return;
    }

    lut_index = lut_index -1;

    temp_u32[3] = (key_dash[0]<<24)|(key_dash[1]<<16)|(key_dash[2]<<8)|key_dash[3];
    temp_u32[2] = (key_dash[4]<<24)|(key_dash[5]<<16)|(key_dash[6]<<8)|key_dash[7];
    temp_u32[1] = (key_dash[8]<<24)|(key_dash[9]<<16)|(key_dash[10]<<8)|key_dash[11];
    temp_u32[0] = (key_dash[12]<<24)|(key_dash[13]<<16)|(key_dash[14]<<8)|key_dash[15];

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER1 + lut_index*CAM_ADDR_OFFSET_LAYER1_SIZE + CAM_ADDR_OFFSET_AES_KEY;
#endif

// Fill AES_ENC_KEY
    for (i = 0; i < 4; i ++)
    {
#ifdef _CCH_AFH_E0_AES_IN_CAM_
        BB_write_sc_cam( (cam_offset + i), temp_u32[i]);
#else
        BB_write_sc_cam( (lut_index*12 + 3 + i), temp_u32[i]);
#endif
    }
#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(BLUE, CCH_DBG_156, 5, lut_index, temp_u32[3], temp_u32[2], temp_u32[1], temp_u32[0]);
#endif
}

void BB_write_sc_iv(UINT16 ce_index, UCHAR* iv)
{
    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);
    UINT8 i;
    UINT32 temp_u32[2];

    if( lut_index == 0)
    {
        return;
    }

    lut_index = lut_index -1;

    temp_u32[1] = (iv[0]<<24)|(iv[1]<<16)|(iv[2]<<8)|iv[3];
    temp_u32[0] = (iv[4]<<24)|(iv[5]<<16)|(iv[6]<<8)|iv[7];

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER1 + lut_index*CAM_ADDR_OFFSET_LAYER1_SIZE + CAM_ADDR_OFFSET_IV;
#endif


// Fill IV
    for (i = 0; i < 2; i ++)
    {
#ifdef _CCH_AFH_E0_AES_IN_CAM_
        BB_write_sc_cam( (cam_offset + i), temp_u32[i]);
#else
        BB_write_sc_cam( (lut_index*12 + 1 + i), temp_u32[i]);
#endif
    }

#ifdef _CCH_SC_ECDH_P256_LOG
    RT_BT_LOG(BLUE, CCH_DBG_163, 3, lut_index, temp_u32[1], temp_u32[0]);
#endif
}

void BB_write_sc_encry_clear(UINT16 ce_index, UCHAR opcode)
{
//  opcode = (encry_en<<1) | decry_en

    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);

    if( lut_index == 0)
    {
        return;
    }

    lut_index = lut_index -1;


#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER1 + lut_index*CAM_ADDR_OFFSET_LAYER1_SIZE;
#endif

#ifdef _CCH_AFH_E0_AES_IN_CAM_

    if( opcode & BIT1 ) // Tx ENC Enable
    {
       BB_write_sc_cam( cam_offset + CAM_ADDR_OFFSET_ENC_TX_CNT, 0);
       BB_write_sc_cam( cam_offset + CAM_ADDR_OFFSET_ENC_TX_CNT + 1, 0);
    }


    if( opcode & BIT0 ) // ÿRx ENC Enable
    {
       BB_write_sc_cam( cam_offset + CAM_ADDR_OFFSET_DEC_RX_CNT, 0xFFFFFFFF);
       BB_write_sc_cam( cam_offset + CAM_ADDR_OFFSET_DEC_RX_CNT + 1, (BIT31 | 0xF));
    }
#ifndef _DAPE_SC_TEST_CORRECT_DAYCOUNTER_FOR_UPF45
#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
    BB_write_sc_cam( cam_offset + CAM_ADDR_OFFSET_DAY_CNT, 0);
#endif
#endif
#else
    if( opcode & BIT1 ) // Tx ENC Enable
    {
       BB_write_sc_cam( (lut_index*12) + 7, 0);
       BB_write_sc_cam( (lut_index*12) + 8, 0);
    }


    if( opcode & BIT0 ) // ÿRx ENC Enable
    {
       BB_write_sc_cam( (lut_index*12) + 9, 0xFFFFFFFF);
       BB_write_sc_cam( (lut_index*12) + 10, (BIT31 | 0xF));
    }

#ifdef _CCH_SC_TEST_20130201_ESCO_INI_FLAG
    BB_write_sc_cam( (lut_index*12) + 11, 0);
#endif

#endif
#ifdef _CCH_AFH_E0_AES_LOG_
    RT_BT_LOG(YELLOW, YL_DBG_HEX_1, 1,opcode);
#endif
}


void BB_read_sc_count(UINT16 ce_index, UINT32 *tx_msb, UINT32 *tx_lsb,
                      UINT32 *rx_msb, UINT32 *rx_lsb, UINT32  *daycounter)
{
//  opcode = (encry_en<<1) | decry_en

    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);

    if( lut_index == 0)
    {
        return;
    }

    lut_index = lut_index -1;


#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER1 + lut_index*CAM_ADDR_OFFSET_LAYER1_SIZE;
#endif

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    // Tx Count
    *tx_lsb = BB_read_sc_cam( cam_offset + CAM_ADDR_OFFSET_ENC_TX_CNT);
    *tx_msb = BB_read_sc_cam( cam_offset + CAM_ADDR_OFFSET_ENC_TX_CNT + 1);


    // Rx Count
    *rx_lsb = BB_read_sc_cam( cam_offset + CAM_ADDR_OFFSET_DEC_RX_CNT);
    *rx_msb = BB_read_sc_cam( cam_offset + CAM_ADDR_OFFSET_DEC_RX_CNT + 1);

    //UINT32 daycounter;
    *daycounter = BB_read_sc_cam( cam_offset + CAM_ADDR_OFFSET_DAY_CNT);

#else

    // Tx Count
    *tx_lsb = BB_read_sc_cam( (lut_index*12) + 7);
    *tx_msb = BB_read_sc_cam( (lut_index*12) + 8);


    // Rx Count
    *rx_lsb = BB_read_sc_cam( (lut_index*12) + 9);
    *rx_msb = BB_read_sc_cam( (lut_index*12) + 10);

    //UINT32 daycounter;
    *daycounter = BB_read_sc_cam( (lut_index*12) + 11);

#endif

//    RT_BT_LOG(BLUE, SECURE_CONN_LOG1, 5, *tx_msb, *tx_lsb, *rx_msb, *rx_lsb, *daycounter);
}



void BB_write_sc_esco_first_flag(UINT16 ce_index, UINT32 set_val)
{
    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);

    if( lut_index == 0)
    {
        return;
    }

    lut_index = lut_index -1;

// field[30]:flmp_esco_initproc2_flag
// field[31]:first_aes_esco_pkt_flag

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER1 + lut_index*CAM_ADDR_OFFSET_LAYER1_SIZE;
#endif

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    BB_write_sc_cam( cam_offset + CAM_ADDR_OFFSET_DAY_CNT, set_val);

    UINT32 daycounter;
    daycounter = BB_read_sc_cam( cam_offset + CAM_ADDR_OFFSET_DAY_CNT);
#else

    BB_write_sc_cam( (lut_index*12) + 11, set_val);

UINT32 daycounter;
    daycounter = BB_read_sc_cam( (lut_index*12) + 11);
#endif
#ifdef _CCH_AFH_E0_AES_LOG_
    RT_BT_LOG(GREEN, SECURE_CONN_LOG2, 3, ce_index, set_val, daycounter);
#endif
}

void BB_read_sc_esco_daycounter(UINT16 ce_index, UINT32 *read)
{

    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);

    if( lut_index == 0)
    {
        return;
    }

    lut_index = lut_index -1;

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER1 + lut_index*CAM_ADDR_OFFSET_LAYER1_SIZE;
#endif

#ifdef _CCH_AFH_E0_AES_IN_CAM_

    *read = BB_read_sc_cam( cam_offset + CAM_ADDR_OFFSET_DAY_CNT);
#else
    *read = BB_read_sc_cam( (lut_index*12) + 11);

#endif

}

void BB_write_sc_rxpkcnt_ignore_seqcompare(UINT16 ce_index, UINT8 set_val)
{

    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);

    if( lut_index == 0)
    {
        return;
    }

    lut_index = lut_index -1;

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER1 + lut_index*CAM_ADDR_OFFSET_LAYER1_SIZE + CAM_ADDR_OFFSET_DEC_RX_CNT;
#endif

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    BB_write_sc_cam( cam_offset + 1, ((set_val<<31)|0xF));
#else
    BB_write_sc_cam( (lut_index*12) + 10, ((set_val<<31)|0xF));
#endif

#ifdef _CCH_AFH_E0_AES_LOG_
#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT32 read = BB_read_sc_cam( cam_offset + 1);
#else
    UINT32 read = BB_read_sc_cam( (lut_index*12) + 10);
#endif
    RT_BT_LOG(YELLOW, CCH_DBG_168, 1,read);
#endif
}
#endif

void BB_write_sc_cam(UINT16 cam_addr, UINT32 wdata)
{
    /* fill data to write data port of CAM */
    BB_write_baseband_register(CAM_DATA_REG0, wdata & 0xFFFF);
    BB_write_baseband_register(CAM_DATA_REG1, wdata >> 16);

    /* set address of CAM and enable write command (bit9 = 1) */
    BB_write_baseband_register(CAM_ADDR_REG, (cam_addr & 0x1FF) | BIT9);

#ifndef _NO_SC_BITFILE_
    /* polling  0x236 (bit9) until it is completed */
    while (1)
    {   // confirm bt Chinwen (20130108)
        if (BB_read_baseband_register(CAM_ADDR_REG) & BIT9)
        {
            //RT_BT_LOG(YELLOW, YL_DBG_HEX_1, 1,BB_read_baseband_register(CAM_ADDR_REG));
            break;
        }
    }
#endif
}

UINT32 BB_read_sc_cam(UINT16 cam_addr)
{
    UINT32_S rdata;

    /* set address of CAM and enable write command (bit9 = 1) */
    BB_write_baseband_register(CAM_ADDR_REG, (cam_addr & 0x1FF));

#ifndef _NO_SC_BITFILE_
    /* polling  0x236 (bit9) until it is completed */
    while (1)
    {
        if (BB_read_baseband_register(CAM_ADDR_REG) & BIT9)
        {
            //RT_BT_LOG(YELLOW, YL_DBG_HEX_1, 1,BB_read_baseband_register(CAM_ADDR_REG));
            break;
        }
    }
#endif
    /* copy value from read data port of CAM */
    rdata.u2Byte[0] = BB_read_baseband_register(CAM_DATA_REG0);
    rdata.u2Byte[1] = BB_read_baseband_register(CAM_DATA_REG1);

    return rdata.u4Byte;

}


#ifdef _CCH_AFH_E0_AES_IN_CAM_

void BB_write_afh_in_cam(UINT16 ce_index, UCHAR *afh_map)
{

    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);
    //LMP_CONNECTION_ENTITY *ce_ptr;
    //ce_ptr = &lmp_connection_entity[ce_index];
    UINT8 i;

    UINT32 temp_u32[3];

    temp_u32[0] = (afh_map[9]<<24)|(afh_map[8]<<16)|(afh_map[7]<<8)|afh_map[6];
    temp_u32[1] = (afh_map[5]<<24)|(afh_map[4]<<16)|(afh_map[3]<<8)|afh_map[2];
    temp_u32[2] = (afh_map[1]<<8)|afh_map[0];


#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER2 + lut_index*CAM_ADDR_OFFSET_LAYER2_SIZE + CAM_ADDR_OFFSET_AFH;
#endif

#ifdef _DAPE_TEST_CHK_CSB_SYNC_TX
RT_BT_LOG(WHITE, YL_DBG_HEX_13, 13, cam_offset,
              ce_index, lut_index,
afh_map[0], afh_map[1], afh_map[2], afh_map[3], afh_map[4],
afh_map[5], afh_map[6], afh_map[7], afh_map[8], afh_map[9]);
#endif

// Fill AFH
    for (i = 0; i < 3; i ++)
    {
        BB_write_sc_cam( (cam_offset + i), temp_u32[i]);
    }
#ifdef _CCH_AFH_E0_AES_LOG_
    RT_BT_LOG(YELLOW, YL_DBG_HEX_4, 4,cam_offset, temp_u32[0], temp_u32[1], temp_u32[2]);
#endif
}

void BB_write_e0_key_in_cam(UINT16 ce_index, UCHAR *key_dash)
{

    UCHAR lut_index = lc_get_lut_index_from_ce_index(ce_index);

    //LMP_CONNECTION_ENTITY *ce_ptr;
    //ce_ptr = &lmp_connection_entity[ce_index];
    UINT8 i;

    UINT32 temp_u32[4];
    temp_u32[0] = (key_dash[15]<<24)|(key_dash[14]<<16)|(key_dash[13]<<8)|key_dash[12];
    temp_u32[1] = (key_dash[11]<<24)|(key_dash[10]<<16)|(key_dash[9]<<8)|key_dash[8];
    temp_u32[2] = (key_dash[7]<<24)|(key_dash[6]<<16)|(key_dash[5]<<8)|key_dash[4];
    temp_u32[3] = (key_dash[3]<<24)|(key_dash[2]<<16)|(key_dash[1]<<8)|key_dash[0];

#ifdef _CCH_AFH_E0_AES_IN_CAM_
    UINT8 cam_offset = CAM_ADDR_OFFSET_LAYER2 + lut_index*CAM_ADDR_OFFSET_LAYER2_SIZE + CAM_ADDR_OFFSET_E0_KEY;
#endif


// Fill E0 Key
    for (i = 0; i < 4; i ++)
    {
        BB_write_sc_cam( (cam_offset + i), temp_u32[i]);
    }
#ifdef _CCH_AFH_E0_AES_LOG_
    RT_BT_LOG(YELLOW, YL_DBG_HEX_5, 5,cam_offset, temp_u32[0], temp_u32[1], temp_u32[2], temp_u32[3]);
#endif
}
#endif

/**
* Controls the BB level encryption for the given \a am_addr.
*
* \param am_addr Active Member Address of the link.
* \param opcode Encryption Control opcode (Refer #enc_control_opcodes for
*               valid opcodes).
*
* \return None.
*/
void BB_encryption_control(UCHAR am_addr, UCHAR phy_piconet_id, UCHAR opcode)
{
    UINT16 ce_index;
    UINT16 reg_val;
    DEF_CRITICAL_SECTION_STORAGE;

    BZ_ASSERT(opcode >= BB_ENC_TX_DISBALE_RX_DISABLE
              && opcode <= BB_ENC_TX_ENABLE_RX_ENABLE,
              "Encryption control opcode is not valid.");
    lmp_get_CE_index_from_AM_ADDR_PPI(am_addr, phy_piconet_id, &ce_index);

    reg_val = (UINT16) ( (am_addr << 5) | (phy_piconet_id << 11) );

    LC_EXIT_SM_MODE();

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(CONNECTOR_REGISTER, reg_val);

    reg_val = BB_read_baseband_register(ENCRYPTION_ENABLE_REGISTER);

#ifdef _CCH_SC_ECDH_P256_
    if (ce_index != INVALID_CE_INDEX)
    {
        BZ_AUTH_LINK_PARAMS* auth;
        auth = lmp_connection_entity[ce_index].auth;
        if(auth->secure_conn_enabled)
        {
            BB_write_sc_encry_clear(ce_index, (opcode&(~reg_val)));
        }
#ifdef _CCH_SC_ECDH_P256_LOG
        RT_BT_LOG(YELLOW, YL_DBG_HEX_1, 1,opcode);
#endif
        reg_val = (UINT16)((reg_val & (~(BIT0|BIT1))) | opcode);
        reg_val = (UINT16)((reg_val & (~BIT2)) | ((auth->secure_conn_enabled)<<2));
        BB_write_baseband_register(ENCRYPTION_ENABLE_REGISTER, reg_val);
#ifdef _RESET_HW_LINK_INFO_TO_INIT_STATE_
        lmp_connection_entity[ce_index].enc_opcode = opcode;
#endif
#ifdef SECURE_CONN_PING_EN
        if(auth->secure_conn_enabled)
        {
            if (opcode == BB_ENC_TX_ENABLE_RX_ENABLE)
            {
                bz_auth_start_ping_req_timer(ce_index);
            }
            else
            {
                bz_auth_stop_ping_req_timer(ce_index);
            }
        }
        //RT_BT_LOG(GREEN, CCH_DBG_164, 3,BB_read_native_clock(), opcode, reg_val);
#endif
    }
#else
    reg_val = (UINT16)((reg_val & (~(BIT0|BIT1))) | opcode);
    BB_write_baseband_register(ENCRYPTION_ENABLE_REGISTER, reg_val);
#ifdef _RESET_HW_LINK_INFO_TO_INIT_STATE_
    if (ce_index != INVALID_CE_INDEX)
    {
        lmp_connection_entity[ce_index].enc_opcode = opcode;
    }
#endif

#endif

    MINT_OS_EXIT_CRITICAL();

    return;
}

#if (defined (SCO_OVER_HCI) || defined(COMPILE_ESCO))
/**
 * Flush \a pkt_length bytes from the synchronous fifo given by
 * \a fifo_num (#SYNC_FIFO1/#SYNC_FIFO2).
 *
 * \param fifo_num Fifo number (SYNC_FIFO1/SYNC_FIFO2).
 * \param pkt_length Number of bytes to be flushed.
 * \param fifo_addr Fifo address (0x00A0 for Tx Fifo, 0x00B0 for Rx Fifo).
 * \param is_mrl TRUE, if Most Recently Loaded packet has to be flushed (flush
 *               from write_pointer). FALSE, otherwise (flush from
 *               read_pointer)
 *
 * \return None.
 * \note Use the helper macros instead of this function
 *      (#BB_flush_baseband_SYNC_TX_FIFO, #BB_flush_baseband_SYNC_RX_FIFO
 *       #BB_MRL_flush_baseband_SYNC_TX_FIFO)
 */
void BB_flush_baseband_SYNC_FIFO(UCHAR fifo_num, UINT16 pkt_length,
                                 UINT16 fifo_addr, int is_mrl)
{
    UINT16 reg_val;

    reg_val = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);

    /* no fifo2 in 0380 */
    reg_val = (UINT16)(reg_val | 0x01);

    if (is_mrl)     /* Is Most Recently Loaded(MRL) packet selected? */
    {
        reg_val = (UINT16)(reg_val | (0x01 << 5));
    }
    else
    {
        reg_val = (UINT16)(reg_val & ~(0x01 << 5));
    }

    /* Write SYNC FIFO number and packet length */
    BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER, reg_val);

    /* flush sco tx pkt for eSCO */
    BB_write_baseband_register(ESCO_FIFO_FLUSH_LENGTH_AND_TX_PKT_LEN_REGISTER,
                    lc_var_esco_fifo_flush_length_and_tx_pkt_len_reg & 0xFC00);

    /* Select FIFO to be flushed */
    BB_write_baseband_register(TRANSMIT_FIFO_STATUS_REGISTER, fifo_addr);

    /* Instruct BB to flush the selected FIFO */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_FLUSH);
}

#endif /* (SCO_OVER_HCI || COMPILE_ESCO) */

/**
 * Flush the Baseband Broadcast FIFO.
 *
 * \param None.
 *
 * \return None.
 */
void bb_flush_broadcast_fifo(void)
{
    BB_write_baseband_register(TRANSMIT_FIFO_STATUS_REGISTER, 0x0090);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_FLUSH);
}

/**
 * Reads data from the baseband RX FIFO.
 *
 * \param buf Ponter to the buffer into which the data will be read.
 * \param length The length to be read from the FIFO.
 *
 * \return None.
 */
void BB_read_baseband_RX_FIFO(UCHAR *buf, UINT16 length,
                                         UINT8 wait_complete)
{
    LC_EXIT_SM_MODE();
    if (length > 0)
    {
        BZDMA_RX_DESC_SEGMENT rxdesc;
        rxdesc.addr = (UINT32)buf;
        rxdesc.len = length;
        rxdesc.flush = FALSE;
        bzdma_send_burst_rxcmd_and_wait_complete(&rxdesc, 1,
                BZDMA_RX_PID_ACL, 0, wait_complete);
    }
}

/**
 * Reads data from the baseband RX FIFO and flushes it.
 *
 * \param length The length to be read from the FIFO.
 *
 * \return None.
 */
void BB_read_baseband_RX_FIFO_and_flush(UINT16 length, UINT8 wait_complete)
{
    LC_EXIT_SM_MODE();
    if (length > 0)
    {
        UINT8 i;
        BZDMA_RX_DESC_SEGMENT rxdesc[8];

        UINT16 mod = length & (BB_RX_FIFO_FLUSH_TEMP_SIZE - 1);
        UINT16 cnt = length >> BB_RX_FIFO_FLUSH_TEMP_GRADE;

        for (i = 0; i < cnt; i ++)
        {
            rxdesc[i].addr = (UINT32)bb_rx_fifo_flush_target;
            rxdesc[i].len = BB_RX_FIFO_FLUSH_TEMP_SIZE;
            rxdesc[i].flush = FALSE;
        }

        if (mod != 0)
        {
            rxdesc[i].addr = (UINT32)bb_rx_fifo_flush_target;
            rxdesc[i].len = mod;
            rxdesc[i].flush = FALSE;
            i++;
        }

        bzdma_send_burst_rxcmd_and_wait_complete(rxdesc, i,
                BZDMA_RX_PID_ACL, 0, wait_complete);
    }
}

#ifdef _BRUCE_FLUSH_STALED_PKT_IN_ESCO_LINK_OVER_HCI
/**
 * Reads data from the baseband RX FIFO and flushes it for sync link.
 *
 * \param length The length to be read from the FIFO.
 *
 * \return None.
 */

void DMA_read_RXFIFO_and_flush_for_sync_link(UINT16 length, UINT8 wait_complete,UINT16 esco_ch)
{
    UINT16 esco_ce_index=0;
    UINT16 reg_val;
    UINT16 CoedcCodeTable = 0x0; //Transparent
    UINT16 CoedcCodeTable_ori;
    UINT8 tx_entry_type;
    tx_entry_type=BZDMA_TX_ENTRY_TYPE_SCO; // transmit SCO data
    BZDMA_REG_S Reg;

    if (lmp_get_esco_ce_index_from_esco_handle(esco_ch,&esco_ce_index)
            != API_SUCCESS)
    {
        return;
    }

    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    do
    {
        /* fire TX cmd */
        Reg.DWord = RD_U32_BZDMA_REG(BZDMA_REG_TX_CMD(tx_entry_type));
        if(Reg.TxCmd.req)
        {
            /*Wait Bzdma TX is completed,  otherwise TX will be affected when switched off coding*/
            //RT_BT_LOG(RED,BRUCE_DEBUG_001,0,0);
        }
        else
        {
            break;
        }
    }
    while(1);

    reg_val = BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG);
    CoedcCodeTable_ori=reg_val &(0x0F << (esco_ce_index << 2));
    //RT_BT_LOG(RED,YL_DBG_HEX_2,2,reg_val,CoedcCodeTable_ori);

    reg_val &= ~(0x0F << (esco_ce_index << 2));
    reg_val |= CoedcCodeTable << (esco_ce_index << 2);
    //RT_BT_LOG(RED,YL_DBG_HEX_1,1,reg_val);
    BB_write_baseband_register(BB_CODEC_CODE_TABLE_REG, reg_val);

    LC_EXIT_SM_MODE();
    if (length > 0)
    {
        UINT8 i;
        BZDMA_RX_DESC_SEGMENT rxdesc[8];
        UINT16 mod = length & (BB_RX_FIFO_FLUSH_TEMP_SIZE - 1);
        UINT16 cnt = length >> BB_RX_FIFO_FLUSH_TEMP_GRADE;

        for (i = 0; i < cnt; i ++)
        {
            rxdesc[i].addr = (UINT32)bb_rx_fifo_flush_target;
            rxdesc[i].len = BB_RX_FIFO_FLUSH_TEMP_SIZE;
            rxdesc[i].flush = FALSE;
        }

        if (mod != 0)
        {
            rxdesc[i].addr = (UINT32)bb_rx_fifo_flush_target;
            rxdesc[i].len = mod;
            rxdesc[i].flush = FALSE;
            i++;
        }
        bzdma_send_burst_rxcmd_and_wait_complete(rxdesc, i,
                BZDMA_RX_PID_SCO, esco_ce_index, wait_complete);
    }

    reg_val = BB_read_baseband_register(BB_CODEC_CODE_TABLE_REG);
    reg_val &= ~(0x0F << (esco_ce_index << 2));
    reg_val |= CoedcCodeTable_ori;
    //RT_BT_LOG(RED,YL_DBG_HEX_1,1,reg_val);
    BB_write_baseband_register(BB_CODEC_CODE_TABLE_REG, reg_val);

    MINT_OS_EXIT_CRITICAL();
}
#endif

UINT16 bb_new_program_codec(UCHAR air_mode, UINT16 input_coding,
                            UINT16 lmp_ce_index, UINT8 linear_format,
                            UINT8 linear_16bit)
{
    UINT8 linear_type;
    UINT8 trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_BYPASS;
    UINT8 input_8bit = 0;
    UINT8 output_8bit = 0;
    LMP_CONNECTION_ENTITY *plmp_ce = &lmp_connection_entity[lmp_ce_index];

    if (air_mode > 3)
    {
        /* invalid parameters*/
        return API_FAILURE;
    }

    if (air_mode != 3) /* air mode "3" is transparent data */
    {
        if (input_coding == INPUT_CODING_LINEAR)
        {
            switch (linear_format)
            {
                case 0:
                    linear_type = BZDMA_CODEC_PCM_TYPE_1S;
                    break;

                case 1:
                    linear_type = BZDMA_CODEC_PCM_TYPE_2S;
                    break;

                case 2:
                case 3:
                    /* sign-magnitude or unsigned */
                    linear_type = linear_format;
                    break;

                default:
                    return API_FAILURE;
            }

            /* convert linear format to 2's complement */
            plmp_ce->pcm_conv_type = linear_type;
            plmp_ce->is_pcm_input = TRUE;

            input_8bit = !linear_16bit;

            if (air_mode == 0)      /* linear to cvsd */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_L2C;
            }
            else if (air_mode == 1) /* linear to u-law */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_L2U;
                output_8bit = 1;
            }
            else                    /* linear to a-law */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_L2A;
                output_8bit = 1;
            }
        }
        else if (input_coding == INPUT_CODING_MU_LAW)
        {
            input_8bit = 1;

            if (air_mode == 0)      /* u-law to cvsd */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_U2C;
            }
            else if (air_mode == 1) /* u-law to u-law */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_BYPASS;
                output_8bit = 1;
            }
            else                    /* u-law to a-law */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_U2A;
                output_8bit = 1;
            }
        }
        else if (input_coding == INPUT_CODING_A_LAW)
        {
            input_8bit = 1;

            if (air_mode == 0)      /* a-law to cvsd */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_A2C;
            }
            else if (air_mode == 1) /* a-law to u-law */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_A2U;
                output_8bit = 1;
            }
            else                    /* a-law to a-law */
            {
                trx_conv_type = BZDMA_CODEC_TX_CONV_TYPE_BYPASS;
                output_8bit = 1;
            }
        }
        else
        {
            return API_FAILURE;
        }
    }

    /* set the codec conversion table of SCO link */
    plmp_ce->trx_codec_conv_type = trx_conv_type;
    plmp_ce->txfifo_in_8bit_code = input_8bit;
    plmp_ce->rxfifo_in_8bit_code = output_8bit;

    return API_SUCCESS;
}

/**
* Kills sniff in baseband.
*
* \param am_addr The am_addr of the connection.
* \param phy_piconet_id The Physical piconet ID of the connection.
 * \param ssr_flag TRUE if entering SSR, FALSE otherwise.
*
* \return None.
*/
void bb_kill_sniff_in_scatternet(UCHAR am_addr, UCHAR phy_piconet_id, UCHAR ssr_flag)
{
    UINT16 con_am_addr = (UINT16) (am_addr << 5);
    DEF_CRITICAL_SECTION_STORAGE;

    con_am_addr = (UINT16) (con_am_addr | (phy_piconet_id << 11));

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(CONNECTOR_REGISTER, con_am_addr);
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_SNIFF);

#ifndef _CCH_DISABLE_SNIFF_TRAN_
    if(ssr_flag == FALSE)
    {
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXIT_SNIFF_TRAN_MODE);
    }
#endif
#ifdef _DAPE_NO_BLOCK_LEGACY_WEHN_SLV_EXIT_SNIFF
    if (lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id) >= LC_SCA_SLAVE_1_LUT)
    {
        if (g_enable_le_block_legacy == 0)
        {
            ll_driver_block_legacy_slot_for_le(2);
            lc_le_pause_status |= (BIT0 << phy_piconet_id);
            lc_le_pause_status_for_sniff |= (BIT0 << phy_piconet_id);
        }
    }
#endif
    MINT_OS_EXIT_CRITICAL();

    return;
}

/**
 * Writes Flow 0 to all am addresses.
 *
 * \return None.
 */
void BB_write_flow_stop(void)
{
    if(bb_flow_state != BB_STOP)
    {
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_SEND_FLOW_ZERO);
        bb_flow_state = BB_STOP;

#ifdef SCNET_DEBUG
        RT_BT_LOG(GRAY, BB_DRIVER_1879, 0, 0);
#endif
    }

    return;
}

/**
 * Writes Flow 1 to all am addresses.
 *
 * \return None.
 */
void BB_write_flow_go(void)
{
    if(bb_flow_state != BB_GO)
    {
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_SEND_FLOW_GO);
        bb_flow_state = BB_GO;

#ifdef SCNET_DEBUG
        RT_BT_LOG(GRAY, BB_DRIVER_1899, 0, 0);
#endif
    }

    return;
}

/**
 * Get the current Baseband flow.
 *
 * \return Current Baseband flow.
 */
INLINE UCHAR BB_get_current_flow(void)
{
    return bb_flow_state;
}

/**
 * Enable/Disable the EIR (transmit) bit.
 * Also flushes Broadcase FIFO on disable.
 *
 * \param enable TRUE to enable and FALSE to disable.
 *
 * \return None.
 */
void bb_enable_eir(UCHAR enable)
{
    UINT16 read;

    /* (0x16 Bit 9) */
    read = BB_read_baseband_register(EIR_ENABLE_REGISTER);
    if(enable == FALSE)
    {
        read = (UINT16) (read & (~(EIR_ENABLE_BIT)));
    }
    else
    {
        read = (UINT16) (read | EIR_ENABLE_BIT);
    }
    BB_write_baseband_register(EIR_ENABLE_REGISTER, read);

    if(enable == FALSE)
    {
        /* Flush Broadcast FIFO */
        bb_flush_broadcast_fifo();
    }

    LC_LOG_INFO(LOG_LEVEL_LOW, ENABLE_EIR_TRANS,1,read);
    return;
}

/**
 * Writes EIR Data into the FIFO.
 * Assumes VER_2_0 + VER_2_1 !!!
 *
 * \param length Length of the EIR data.
 * \param packet_type Packet Type to be programmed.
 * \param data   Pointer to the data buffer.
 *
 * \return None.
 */
void bb_write_eir_data(UINT16 length, UINT16 packet_type, UCHAR *data)
{
    UINT16 lut_contents;

    /* Flush Broadcast FIFO */
    bb_flush_broadcast_fifo();

    /* Update packet header (only length, type and LCH) */
    lut_contents = (UINT16) (BB_L2CAP_START_PKT | packet_type | length);

    /* Update FIFO */
    BB_write_baseband_TX_FIFO_scatternet(data, length, BC_AM_ADDR, 0);

    BB_write_baseband_register(MASTER_BROADCAST_LOWER_LUT_ADDR, lut_contents);

    return;
}

/**
 * Enable/Disable EIR-recv bit.
 *
 * \param enable TRUE or FALSE.
 *
 * \return None.
 */
void bb_enable_eir_recv(UCHAR enable)
{
    UINT16 read;

    /* (0x16.Bit 8) */
    read = BB_read_baseband_register(EIR_ENABLE_REGISTER);
    if(enable == FALSE)
    {
        read = (UINT16) (read & (~(EIR_RECV_ENABLE_BIT)));
    }
    else
    {
        read = (UINT16) (read | EIR_RECV_ENABLE_BIT);
    }
    BB_write_baseband_register(EIR_ENABLE_REGISTER, read);

#ifdef SCNET_DEBUG
    LC_LOG_INFO(LOG_LEVEL_LOW, ENABLE_EIR_REC,1,read);
#endif

    return;
}

/**
* Modifies xtol value in baseband.
*
* \param val Value of the xtol to be updated.
* \param phy_piconet_id Physical piconet ID of the connection..
*
* \return None.
*/
void BB_modify_xtol_in_scatternet(UCHAR val, UCHAR phy_piconet_id)
{
    UINT16 read_val;

    if (phy_piconet_id > SCA_PICONET_SECOND)
    {
        read_val = BB_read_baseband_register(X_VALUE_FOR_TOLERANCE1_REGISTER);

        if(phy_piconet_id == SCA_PICONET_THIRD)
        {
            /* Third piconet. */
            read_val = (UINT16) (read_val & 0xFF00);

            read_val = (UINT16) (read_val | val);
        }
        else
        {
            /* Fourth piconet. */
            read_val = (UINT16) (read_val & 0x00FF);

            read_val = (UINT16) (read_val | (val << 8) );
        }

        BB_write_baseband_register(X_VALUE_FOR_TOLERANCE1_REGISTER, read_val);

        return;
    }

    read_val = BB_read_baseband_register(X_VALUE_FOR_TOLERANCE_REGISTER);

    if(phy_piconet_id == SCA_PICONET_FIRST)
    {
        /* First piconet. */
        read_val = (UINT16) (read_val & 0xFF00);

        read_val = (UINT16) (read_val | val);
    }
    else
    {
        /* Second piconet. */
        read_val = (UINT16) (read_val & 0x00FF);

        read_val = (UINT16) (read_val | (val << 8) );
    }

    BB_write_baseband_register(X_VALUE_FOR_TOLERANCE_REGISTER, read_val);

    return;
}


/**
*  Reads the native clock.
*
* \param None.
*
* \return value The instantaneous value of native clock.
*/
UINT32 BB_read_native_clock(void)
{
    UINT32 value;

    value = (BB_read_baseband_register(NATIVE_CLOCK2_REGISTER) << 16) |
             BB_read_baseband_register(NATIVE_CLOCK1_REGISTER);

    return value;
}

/**
 *  Updates the no of slots before which BB will suspend tx/rx before
 *  sniff anchor point.
 *
 * \param None.
 *
 * \return None.
 */
void BB_update_sniff_max_slot()
{
    UINT16 least_sniff_interval;
    UCHAR slots = 7;
    BZ_REG_S_PRI_CTRL1 reg_pri;

    /* Get the least sniff interval. */
#ifdef _DAPE_SNIFF_PKT_TEST
    UINT16 sniff_ce_index;
    UINT16 sniff_attempt;
    least_sniff_interval = lc_get_least_sniff_interval(&sniff_ce_index, &sniff_attempt);
#else
    least_sniff_interval = lc_get_least_sniff_interval();
#endif

    if(least_sniff_interval == 0)
    {
        /* No sniff connections. Dont care. (so set to 0 - austin ?) */
        slots = 0;
    }
    else if (least_sniff_interval < LC_MIN_SNIFF_INTERVAL_ALLOW_3_SLOT_PKT)
    {
        /* 3 slots are disabled. */
        slots = 3;
    }
    else if (least_sniff_interval < LC_MIN_SNIFF_INTERVAL_ALLOW_5_SLOT_PKT)
    {
        /* 5 slots are disabled. */
        slots = 5;
    }

    *(UINT16*)&reg_pri = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);
    reg_pri.acl_pause = slots;
    BB_write_baseband_register(SCA_PRIORITY_REGISTER2, *(UINT16*)&reg_pri);

    return;
}

#ifdef _YL_PATCH_UTILITY
/* added by yilinli */
// ROM size: 48 bytes
UCHAR BB_read_xtol_in_scatternet(UCHAR phy_piconet_id)
{
    UINT16 read_val;
    UINT16 xtol_addr;
    UINT8 is_msb;
    if (phy_piconet_id > SCA_PICONET_SECOND)
    {
        xtol_addr = X_VALUE_FOR_TOLERANCE1_REGISTER;
        if(phy_piconet_id == SCA_PICONET_THIRD)
        {
            /* Third piconet. */
            is_msb = 0;
        }
        else
        {
            /* Fourth piconet. */
            is_msb = 1;
        }
    }
    else
    {
        xtol_addr = X_VALUE_FOR_TOLERANCE_REGISTER;
        if(phy_piconet_id == SCA_PICONET_FIRST)
        {
            /* First piconet. */
            is_msb = 0;
        }
        else
        {
            /* Second piconet. */
            is_msb = 1;
        }
    }
    read_val = BB_read_baseband_register(xtol_addr);
    if(is_msb)
    {
        read_val = (UINT16) (read_val >> 8);
    }
    else
    {
        read_val = (UINT16) (read_val & 0x00FF);
    }

    return (UCHAR) read_val;
}

// ROM size: BB_read_native_counter+BB_write_native_counter = 48 bytes
UINT16 BB_read_native_counter(void)
{
    UINT16 read_value = BB_read_baseband_register(BB_NATIVE_COUNTER_REG) ;
    read_value &= 0x3FF;
    return read_value;
}

void BB_write_native_counter(UINT16 value)
{
    UINT16 read_value = BB_read_baseband_register(BB_NATIVE_COUNTER_REG) ;
    read_value &= 0xFC00;
    read_value += value;
    BB_write_baseband_register(BB_NATIVE_COUNTER_REG, read_value) ;
}

// ROM size: 112 bytes
API_RESULT BB_native_counter_shift(INT16 shift_us, UINT16 shift_guard_time_us)
{
    UINT16 native_counter;
    UINT16 native_counter_mod312;
    UINT16 i;
    for (i = 0; i < 0x8000; i++)
    {
        native_counter = BB_read_native_counter();
        native_counter_mod312 = (native_counter >= 312) ? (native_counter - 312) : native_counter;
        if (((shift_us > 0) && (native_counter_mod312 <= (312 - shift_guard_time_us - shift_us))) ||
             ((shift_us < 0) && (native_counter_mod312 >= (shift_guard_time_us - shift_us))))
        {
            native_counter += shift_us;
            BB_write_native_counter(native_counter);
            return API_SUCCESS;
        }
    }
    return API_FAILURE;
}

#endif
#ifdef _DAPE_TEST_PATCH_UTILITY
#if 0
UINT32 BB_read_debug_port_state(INT16 debug_port)
{
    VENDOR_WRITE(0x30, (VENDOR_READ(0x30)&BIT31) | 0x40);
    WR_16BIT_IO(BB_BASE_ADDR, 0x1FE, debug_port);
    UINT32 debug_port_state = 0;
    debug_port_state = VENDOR_READ(0x28);
    return debug_port_state;
}
#endif
UINT16 BB_read_instruction_status(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR am_addr;
    UCHAR phy_piconet_id;
    UCHAR lut_index;
    UINT16 instruction_status;

    if ((ce_index >= LMP_MAX_CE_DATABASE_ENTRIES))
    {
        /* check valid arguments */
        return 0xBEEF;
    }

    ce_ptr = &lmp_connection_entity[ce_index];
    am_addr = ce_ptr->am_addr;
    phy_piconet_id = ce_ptr->phy_piconet_id;
    lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, phy_piconet_id);

    /* If lut_index is not 0~11, Then HW will not use fw's value
       It has a default value.*/
    BB_write_baseband_register(BB_INSTRUCTION_STATUS_REG, (lut_index - 1)<<12);
    instruction_status = BB_read_baseband_register(BB_INSTRUCTION_STATUS_REG);

    return instruction_status;
}
#endif
/**
 * Pause sco
 */
void bb_pause_sco(UCHAR pause)
{
    BZ_REG_S_PRI_CTRL pri_ctrl;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    *(UINT16*)&pri_ctrl = BB_read_baseband_register(SCA_PRIORITY_REGISTER);
    if (pause == TRUE)
    {
        pri_ctrl.pause_sco = TRUE;
    }
    else
    {
        pri_ctrl.pause_sco = FALSE;
    }
    BB_write_baseband_register(SCA_PRIORITY_REGISTER, *(UINT16*)&pri_ctrl);

    MINT_OS_EXIT_CRITICAL();
}
/**
 * Pause esco
 */
#ifdef  _DAPE_TEST_NEW_HW_PAUSE_ESCO_WHEN_RETX
void bb_pause_esco(UCHAR pause)
{
    BZ_REG_S_PRI_CTRL2 pri_ctrl2;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    *(UINT16*)&pri_ctrl2 = BB_read_baseband_register(SCA_PRIORITY_REGISTER3);
    if (pause == TRUE)
    {
        pri_ctrl2.pause_esco = TRUE;
    }
    else
    {
        pri_ctrl2.pause_esco = FALSE;
    }
    BB_write_baseband_register(SCA_PRIORITY_REGISTER3, *(UINT16*)&pri_ctrl2);

    MINT_OS_EXIT_CRITICAL();
}
#endif

#if 0
void bb_legacy_priority_higher_than_le_acl(UCHAR flag)
{
    BZ_REG_S_PRI_CTRL1 pri_ctrl1;
    *(UINT16*)&pri_ctrl1 = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);
    if(flag)
    {
        pri_ctrl1.legacy_high_than_le= TRUE;
    }
    else
    {
        pri_ctrl1.legacy_high_than_le= FALSE;
    }
    BB_write_baseband_register(SCA_PRIORITY_REGISTER2, *(UINT16*)&pri_ctrl1);
}
#endif

#if ((defined _DAPE_NO_TRX_WHEN_LE_SEND_CONN_REQ_HW_TIMER) || \
	(defined _DAPE_NO_TRX_WHEN_LE_CONN_UPDT)) || \
	(defined _DAPE_EN_8821_MP_LE_SCAN_INTR)
void bb_switch_scheduler_to_legacy()
{
#ifndef _DAPE_EN_8821_MP_LE_SCAN_INTR
    BZ_REG_S_PRI_CTRL1 pri_ctrl1;
    *(UINT16*)&pri_ctrl1 = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);

    pri_ctrl1.mask_txrx_timing = TRUE;
    BB_write_baseband_register(SCA_PRIORITY_REGISTER2, *(UINT16*)&pri_ctrl1);
    lc_check_and_enable_scans_in_scatternet();
#else
    BZ_REG_S_PRI_CTRL2 pri_ctrl2;
    *(UINT16*)&pri_ctrl2 = BB_read_baseband_register(SCA_PRIORITY_REGISTER3);
    pri_ctrl2.block_legacy_acl = FALSE;
    BB_write_baseband_register(SCA_PRIORITY_REGISTER3, *(UINT16*)&pri_ctrl2);
#endif
    lc_start_tpoll_on_all_connections();

//// dape test
//RT_BT_LOG(GREEN, DAPE_TEST_LOG213, 2,BB_read_native_clock(),
//          BB_read_baseband_register(SCA_PRIORITY_REGISTER3));
}
UINT8 bb_switch_scheduler_to_le()
{
    /* if legacy is still in paging or inquiry, do not switch to le */
    if (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE)
    {
        return FALSE;
    }
    /* chinwen added an priority bit to switch between legacy-acl and LE. priority[39].
       So for 8821 MP, we only need to set this bit instead of kill scan, set invalid pkt type
       and turn off trx.*/
#ifndef _DAPE_EN_8821_MP_LE_SCAN_INTR
    lc_kill_scan_mode();

#if 0
    UINT8 i;

    for (i = 0; i < 8; i++)
    {
        Update_val_with_bb_reg(reg_MASTER_LOWER_LUT[i], 0xC000, 0xC000);
    }
#endif

    /* Turn off trx */
    BZ_REG_S_PRI_CTRL1 pri_ctrl1;
    *(UINT16*)&pri_ctrl1 = BB_read_baseband_register(SCA_PRIORITY_REGISTER2);
    pri_ctrl1.mask_txrx_timing = FALSE;
    BB_write_baseband_register(SCA_PRIORITY_REGISTER2, *(UINT16*)&pri_ctrl1);
#else
    BZ_REG_S_PRI_CTRL2 pri_ctrl2;
    *(UINT16*)&pri_ctrl2 = BB_read_baseband_register(SCA_PRIORITY_REGISTER3);
    pri_ctrl2.block_legacy_acl = TRUE;
    BB_write_baseband_register(SCA_PRIORITY_REGISTER3, *(UINT16*)&pri_ctrl2);
#endif
    /* We should stop tpoll for LE scan even if priority bit 39 is set high. */
    lc_stop_tpoll_on_all_connections_except(16);
//// dape test
//RT_BT_LOG(RED, DAPE_TEST_LOG213, 2,BB_read_native_clock(),
//          BB_read_baseband_register(SCA_PRIORITY_REGISTER3));

    return TRUE;

}
#endif

